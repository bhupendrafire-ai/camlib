"""Bounded remaining-material/gouge reports from actual whole-cell stock bounds.

The authoritative stock bounds are an explicit premise. Classification completion
does not mean tool-contact, job-generation or controller verification is complete.
"""

from dataclasses import asdict, dataclass
import json

from ._buffers import native
from .simulation import PartialStockView, StockState


@dataclass(frozen=True)
class Target:
    id: str
    revision: str
    machining_region: object
    floor_z_mm: float
    floor_error_mm: float = 0

    def to_dict(self):
        region = self.machining_region
        return dict(
            version=1,
            units="mm",
            id=self.id,
            revision=self.revision,
            machining_region=region.to_dict() if hasattr(region, "to_dict") else region,
            floor_z_mm=self.floor_z_mm,
            floor_error_mm=self.floor_error_mm,
        )


@dataclass(frozen=True)
class FinishingScope:
    """Depth/allowance and vertical ball-only access, independent of a toolpath.

    Fixtures, shank/holder and machine travel require separate motion checks.
    access_error_mm controls enclosure refinement, not machining tolerance.
    """

    floor_z_mm: float  # minimum cutter-tip depth, not the original CAD base
    tool_radius_mm: float
    stock_to_leave_mm: float = 0
    access_error_mm: float = 0.02
    max_refinement_depth: int = 3
    max_access_work: int = 2_000_000

    @classmethod
    def from_operation(cls, operation, *, tool=None, **analysis_limits):
        from .operations import Parallel, Pencil, Project, Radial, Scallop, Spiral, Waterline

        if not isinstance(operation, (Parallel, Spiral, Radial, Project, Pencil, Scallop, Waterline)):
            raise TypeError("finishing access needs a surface finishing operation")
        assembly = operation.tool if tool is None else tool
        if not hasattr(assembly, 'cutter') or getattr(assembly, 'id', None) != getattr(
            operation.tool, 'id', operation.tool
        ):
            raise ValueError("resolve the operation's matching tool assembly")
        if dict(assembly.cutter.shape).get('kind') != 'ball':
            raise ValueError("finishing access supports ball cutters")
        # Both planners protect the operation base with their normal allowance.
        # Waterline also has an independently authored lowest tip level.
        floor = operation.floor_z_mm + operation.stock_to_leave_mm
        if isinstance(operation, Waterline):
            floor = max(floor, operation.bottom_z_mm)
        return cls(floor, assembly.cutter.diameter_mm / 2, operation.stock_to_leave_mm, **analysis_limits)


@dataclass(frozen=True)
class ContactWitnesses:
    """Each point is within error_mm of some point of the intended target.

    These are exclusion witnesses for ball access, not a covering mesh, stock
    inner enclosure or clearance certificate. The source contract is explicit.
    """

    points_mm: object
    error_mm: float = 0
    source: object = None

    def __post_init__(self):
        import itertools
        import math

        points = tuple(tuple(itertools.islice(iter(p), 4)) for p in itertools.islice(iter(self.points_mm), 65537))
        if (
            not 1 <= len(points) <= 65536
            or any(len(p) != 3 for p in points)
            or any(not math.isfinite(x) or abs(x) > 1e6 for p in points for x in p)
            or not math.isfinite(self.error_mm)
            or not 0 <= self.error_mm <= 0.1
        ):
            raise ValueError('invalid material contact witness points/error/budget')
        object.__setattr__(self, 'points_mm', points)

    def to_dict(self):
        return dict(points_mm=self.points_mm, error_mm=self.error_mm, source=self.source)


@dataclass(frozen=True)
class InnerPlanarPatch:
    """A trimmed source face in local Z=0, with explicit placement uncertainty.

    The region interior after its boundary error margin must belong to the
    target face. This source premise cannot be inferred from arbitrary facets.
    Rust proves complete cell footprints; mixed footprints supply no lower bound.
    """

    region: object
    frame: object
    normal_error_mm: float = 0

    def to_dict(self):
        return dict(region=self.region.to_dict(), frame=asdict(self.frame), normal_error_mm=self.normal_error_mm)


@dataclass(frozen=True)
class SurfaceTarget:
    """Upper-envelope target; source_enclosure is an explicit geometry premise.

    bounded: Hausdorff outer bound only (nonzero error leaves inner uncertainty).
    inscribed: mesh solid lies inside target, target inside its error offset.
    offset: target lies between mesh erosion/dilation by the source error.
    All modes retain an exact protected base plane.
    """

    id: str
    revision: str
    surface: object
    base_z_mm: float
    source_enclosure: str = "bounded"
    finishing: FinishingScope | None = None
    contact_witnesses: ContactWitnesses | None = None
    inner_patches: tuple = ()

    def __post_init__(self):

        if not isinstance(self.inner_patches, (list, tuple)):
            raise TypeError('inner_patches must be a list or tuple (a finite collection)')
        patches = tuple(self.inner_patches)
        if any(not isinstance(p, InnerPlanarPatch) for p in patches):
            raise ValueError('inner_patches requires InnerPlanarPatch objects')
        object.__setattr__(self, 'inner_patches', patches)

    def to_dict(self):
        result = dict(
            version=1,
            units="mm",
            id=self.id,
            revision=self.revision,
            surface=self.surface.to_dict(),
            base_z_mm=self.base_z_mm,
            source_enclosure=self.source_enclosure,
        )
        if self.contact_witnesses is not None:
            result["contact_witnesses"] = self.contact_witnesses.to_dict()
        if self.inner_patches:
            result["inner_patches"] = [patch.to_dict() for patch in self.inner_patches]
        if self.finishing is not None:
            result["finishing"] = asdict(self.finishing)
        return result


@dataclass(frozen=True)
class Options:
    geometry_tolerance_mm: float = 1e-5
    comparison_tolerance_mm: float = 0.001
    max_tile_nodes: int = 1_000_000
    max_boundary_tests: int = 50_000_000
    max_classified_cells: int = 0xFFFFFFFF
    max_findings: int = 1024


def _json(value):
    return json.dumps(value, allow_nan=False, separators=(",", ":"))


class Verifier:
    """Prepare native target once; borrow complete or compact fractional stock.

    Finishing reports include overlapping uncertainty indicators for finding
    cells (including truncated findings), plus flags on each stored finding.
    Missing inner contact is a source-premise limitation, not proof the cutter
    cannot reach material.
    Zero allowance has exact zero volume even when other analysis is incomplete.
    """

    def __init__(self, target, options=Options()):
        self.prepare(target, options)

    def prepare(self, target, options=Options()):
        prepared = native.MaterialTarget(_json(target.to_dict()), _json(asdict(options)))
        self._prepared, self._target, self._options = prepared, target, options
        return prepared.digest

    @property
    def target(self):
        return self._target

    @property
    def digest(self):
        return self._prepared.digest

    def check(self, state):
        overlay = None
        if isinstance(state, PartialStockView):
            state._check_base()
            base = state._base
            overlay = (state._frame.raw, state._offset, state._count)
        elif isinstance(state, StockState):
            base = state
        else:
            raise TypeError("material verification needs authoritative StockState or PartialStockView")
        manifest = base._manifest
        context = dict(
            stock={k: v for k, v in manifest["stock"].items() if k != "mesh"},
            generation=state.completed,
            fraction=state.fraction,
            run=dict(run=manifest["run"], manifest_sha256=state._digest.hex()),
        )
        # No per-cell Python iteration/serialization and no dense partial clone.
        return json.loads(self._prepared.check(base.heights, _json(context), overlay))

    def check_bounds(self, heights, stock, *, generation=0, fraction=0):
        """Borrow caller-supplied whole-cell f64 pairs, without a stream run claim."""
        context = dict(
            stock=stock.grid_dict()
            if hasattr(stock, "grid_dict")
            else {k: v for k, v in (stock.to_dict() if hasattr(stock, "to_dict") else stock).items() if k != "mesh"},
            generation=generation,
            fraction=fraction,
            run=None,
        )
        return json.loads(self._prepared.check(heights, _json(context)))
