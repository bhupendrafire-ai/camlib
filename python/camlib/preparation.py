"""Native reach/blank proposals and durable explicit placement confirmation.

CAD import, calibrated assemblies and acquisition of complete physical-stock
measurements remain caller responsibilities. None of these results releases NC.
"""

from dataclasses import asdict, dataclass, replace
import json
import math

from ._buffers import native
from .jobs import _digest, _finite
from .motion import WorkFrame
from .simulation import InitialStock
from .surface import SurfaceMesh
from .verification import Options


@dataclass(frozen=True)
class ReachSample:
    id: str
    point_mm: tuple
    normal: tuple


def _query(kind, value, cancel):
    return json.loads(native.prepare_geometry(kind, json.dumps(value, allow_nan=False), cancel))


def reach_map(
    surface,
    samples,
    tools,
    *,
    base_z_mm,
    scene,
    clearance_z_mm,
    tolerance_mm,
    max_pairs,
    max_work,
    scene_options=Options(),
    cancel=None,
):
    """Return one row per sample/calibrated assembly and smallest clear tested projection.

    tools is a finite collection of (family_id, ToolAssembly). Interrupted work
    has one unresolved row per unfinished pair and no selected proposals.
    max_work includes spatial-index/section construction, bin-entry traversal
    and facet/section queries; scene checks retain their own scene_options limits.
    Invalid native geometry and cancellation during initial preparation raise;
    cancellation after a pair starts returns an incomplete report. Coverage is
    supplied poses and vertical approaches. Follow with whole-recipe checks.
    """
    samples = _finite(samples, 'reach samples')
    tools = _finite(tools, 'reach tools')
    if not all(isinstance(s, ReachSample) for s in samples):
        raise TypeError('samples must be ReachSample values')
    return _query(
        'reach',
        dict(
            surface=surface.to_dict(),
            samples=[asdict(s) for s in samples],
            tools=[dict(family=family, assembly=tool.to_dict()) for family, tool in tools],
            base_z_mm=base_z_mm,
            scene=scene.to_dict(),
            clearance_z_mm=clearance_z_mm,
            tolerance_mm=tolerance_mm,
            max_pairs=max_pairs,
            max_work=max_work,
            scene_options=asdict(scene_options),
        ),
        cancel,
    )


def enclose_blank(surface, stock, declaration, *, tolerance_mm, max_work, source_enclosure, cancel=None):
    """Bound every whole grid cell using native facet/rectangle enclosures.

    Mesh is already in setup coordinates, declaration.bounds_mm must be empty.
    The explicit source_enclosure must be 'top_down_outer'; arbitrary closed
    meshes with voids/undercuts cannot establish that physical premise.
    """
    row = _query(
        'blank',
        dict(
            surface=surface.to_dict(),
            stock=stock.to_dict(),
            declaration=declaration.to_dict(),
            tolerance_mm=tolerance_mm,
            max_work=max_work,
            source_enclosure=source_enclosure,
        ),
        cancel,
    )
    return InitialStock(
        row['source_id'],
        row['revision'],
        row['source_sha256'],
        tuple((h['lower_mm'], h['upper_mm']) for h in row['bounds']),
        WorkFrame(**row['source_to_setup']),
        row['uncertainty_mm'],
    )


def blank_placements(
    surface,
    orientations,
    reference_bounds,
    stock,
    declaration,
    *,
    tolerance_mm,
    max_work,
    max_candidates,
    ambiguity_gap_mm,
    source_enclosure,
    cancel=None,
):
    """Finite upright bounding-box alignment proposals, never an ICP/physical fit proof.

    For each declared orientation align the rotated box centre in XY and bottom
    in Z to the reference box. Score is max absolute extent mismatch (mm), lower
    is better. A tie/near tie remains ambiguous. Every proposed mesh is enclosed
    natively; outside-domain proposals carry a refusal reason.
    """
    orientations = _finite(orientations, 'blank orientations')
    if (
        type(max_candidates) is not int
        or not 1 <= len(orientations) <= max_candidates
        or len(reference_bounds) != 2
        or any(len(p) != 3 for p in reference_bounds)
        or any(not math.isfinite(v) for p in reference_bounds for v in p)
        or any(reference_bounds[0][i] >= reference_bounds[1][i] for i in range(3))
        or not math.isfinite(ambiguity_gap_mm)
        or ambiguity_gap_mm < 0
    ):
        raise ValueError('supply ordered reference bounds, finite ambiguity gap and candidate budget')
    rows = []
    for orientation in orientations:
        if not isinstance(orientation, WorkFrame) or tuple(orientation.z) != (0, 0, 1):
            raise ValueError('registration supports explicitly declared upright WorkFrames')

        # Native validation checks orthonormality after the placement is constructed.
        def transform(p):
            return tuple(sum(p[j] * getattr(orientation, ('x', 'y', 'z')[j])[k] for j in range(3)) for k in range(3))

        rotated = tuple(tuple(transform(p) for p in tri) for tri in surface.triangles)
        lo = tuple(min(p[k] for tri in rotated for p in tri) for k in range(3))
        hi = tuple(max(p[k] for tri in rotated for p in tri) for k in range(3))
        shift = tuple(
            (
                (reference_bounds[0][k] + reference_bounds[1][k] - lo[k] - hi[k]) / 2
                if k < 2
                else reference_bounds[0][k] - lo[k]
            )
            for k in range(3)
        )
        frame = replace(orientation, origin_mm=shift)
        placed = SurfaceMesh(
            tuple(tuple(tuple(p[k] + shift[k] for k in range(3)) for p in tri) for tri in rotated),
            surface.surface_error_mm,
            surface.source,
        )
        metadata = replace(declaration, source_to_setup=frame, bounds_mm=())
        premise = dict(
            surface=placed.to_dict(),
            stock=stock.to_dict(),
            declaration=metadata.to_dict(),
            tolerance_mm=tolerance_mm,
            max_work=max_work,
            source_enclosure=source_enclosure,
        )
        identity = _digest(premise)
        row = dict(
            id=identity,
            score_mm=max(abs(hi[k] - lo[k] - (reference_bounds[1][k] - reference_bounds[0][k])) for k in range(3)),
            premise=premise,
            blank=None,
            reason=None,
        )
        try:
            row['blank'] = enclose_blank(
                placed,
                stock,
                metadata,
                tolerance_mm=tolerance_mm,
                max_work=max_work,
                source_enclosure=source_enclosure,
                cancel=cancel,
            ).to_dict()
        except ValueError as error:
            if cancel is not None and cancel.is_cancelled():
                raise
            row['reason'] = str(error)
        if not any(r['id'] == identity for r in rows):
            rows.append(row)
    rows.sort(key=lambda r: (r['score_mm'], r['id']))
    eligible = [r for r in rows if r['blank'] is not None]
    return dict(
        proposals=rows,
        ambiguous=len(eligible) > 1 and eligible[1]['score_mm'] - eligible[0]['score_mm'] <= ambiguity_gap_mm,
        ambiguity_gap_mm=ambiguity_gap_mm,
        selected=None,
    )


def _validate_engineer(who):
    if not isinstance(who, str) or not who.strip() or len(who) > 512:
        raise ValueError('confirmation needs an explicit engineer identity')


def confirm_blank(proposal, *, who):
    """Persist this record alongside the exact proposal, then validate on reopen."""
    _validate_engineer(who)
    if proposal['blank'] is None or proposal['id'] != _digest(proposal['premise']):
        raise ValueError('proposal is refused or stale')
    return dict(proposal_id=proposal['id'], enclosure_sha256=_digest(proposal['blank']), who=who)


def confirmed_initial_stock(proposal, confirmation, *, cancel=None):
    """Recompute native bounds and match the persisted confirmation before Setup."""
    _validate_engineer(confirmation.get('who'))
    if (
        proposal['id'] != _digest(proposal['premise'])
        or confirmation.get('proposal_id') != proposal['id']
        or confirmation.get('enclosure_sha256') != _digest(proposal['blank'])
    ):
        raise ValueError('blank confirmation is missing or stale')
    row = _query('blank', proposal['premise'], cancel)
    if row != proposal['blank']:
        raise ValueError('blank enclosure changed; confirmation is stale')
    return InitialStock(
        row['source_id'],
        row['revision'],
        row['source_sha256'],
        tuple((h['lower_mm'], h['upper_mm']) for h in row['bounds']),
        WorkFrame(**row['source_to_setup']),
        row['uncertainty_mm'],
    )
