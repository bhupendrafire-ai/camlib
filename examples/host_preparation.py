"""Host-side colour intent and confirmed blank placement, without CAD imports.

The application extracts face colours/IDs, supplies its own versioned chart,
builds native geometry/targets, and confirms measured blank placement. Nothing
here infers cutting data, expands a tolerance or certifies a physical setup.
"""

from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math

from camlib.jobs import OperationNode, _plain


def _digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def _rgb_lab(rgb):
    if len(rgb) != 3 or any(type(c) is not int or not 0 <= c <= 255 for c in rgb):
        raise ValueError('supply three sRGB bytes per coloured face/chart entry')
    c = [v / 255 for v in rgb]
    r, g, b = [v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4 for v in c]
    xyz = (
        (0.4124564 * r + 0.3575761 * g + 0.1804375 * b) / 0.95047,
        0.2126729 * r + 0.7151522 * g + 0.072175 * b,
        (0.0193339 * r + 0.119192 * g + 0.9503041 * b) / 1.08883,
    )
    x, y, z = [v ** (1 / 3) if v > (6 / 29) ** 3 else v / (3 * (6 / 29) ** 2) + 4 / 29 for v in xyz]
    return 116 * y - 16, 500 * (x - y), 200 * (y - z)


def ciede2000(lab1, lab2):
    """CIEDE2000 with unit parametric weights; input is D65 L*a*b*."""
    l1, a1, b1 = lab1
    l2, a2, b2 = lab2
    cm = (math.hypot(a1, b1) + math.hypot(a2, b2)) / 2
    g = 0.5 * (1 - math.sqrt(cm**7 / (cm**7 + 25**7)))
    a1, a2 = (1 + g) * a1, (1 + g) * a2
    c1, c2 = math.hypot(a1, b1), math.hypot(a2, b2)
    h1, h2 = math.degrees(math.atan2(b1, a1)) % 360, math.degrees(math.atan2(b2, a2)) % 360
    dh = 0 if c1 * c2 == 0 else h2 - h1
    if dh > 180:
        dh -= 360
    elif dh < -180:
        dh += 360
    dl, dc = l2 - l1, c2 - c1
    d_h = 2 * math.sqrt(c1 * c2) * math.sin(math.radians(dh / 2))
    lm, cm = (l1 + l2) / 2, (c1 + c2) / 2
    hm = h1 + h2
    if c1 * c2 != 0:
        hm = hm / 2 if abs(h1 - h2) <= 180 else (hm + (360 if hm < 360 else -360)) / 2

    def cos(angle):
        return math.cos(math.radians(angle))

    t = 1 - 0.17 * cos(hm - 30) + 0.24 * cos(2 * hm) + 0.32 * cos(3 * hm + 6) - 0.20 * cos(4 * hm - 63)
    sl = 1 + 0.015 * (lm - 50) ** 2 / math.sqrt(20 + (lm - 50) ** 2)
    sc, sh = 1 + 0.045 * cm, 1 + 0.015 * cm * t
    rt = -2 * math.sqrt(cm**7 / (cm**7 + 25**7)) * math.sin(math.radians(60 * math.exp(-(((hm - 275) / 25) ** 2))))
    return math.sqrt((dl / sl) ** 2 + (dc / sc) ** 2 + (d_h / sh) ** 2 + rt * (dc / sc) * (d_h / sh))


@dataclass(frozen=True)
class IntentEntry:
    id: str
    rgb: tuple
    purpose: str
    lower_mm: float
    upper_mm: float
    allowance_mm: float
    keep_out: bool

    def __post_init__(self):
        _rgb_lab(self.rgb)
        if (
            not self.id
            or not self.purpose
            or not all(math.isfinite(v) for v in (self.lower_mm, self.upper_mm, self.allowance_mm))
            or self.lower_mm > self.upper_mm
            or self.allowance_mm < 0
            or type(self.keep_out) is not bool
        ):
            raise ValueError('intent needs explicit ordered tolerances, nonnegative allowance and keep-out policy')


def resolve_colours(faces, entries, *, source_sha256, chart_id, chart_revision, max_delta_e, ambiguity_delta_e):
    """Return every face, including unknown/missing/ambiguous colours.

    faces maps stable source face IDs to sRGB-byte triples or None. Thresholds
    are caller policy. A nearest match never silently resolves overlapping
    chart meanings. Geometry conversion must preserve this source-face map.
    """
    if not isinstance(entries, (tuple, list)) or not isinstance(faces, dict):
        raise TypeError('supply a finite chart collection and face-colour dictionary')
    entries = tuple(entries)
    if (
        not chart_id
        or not chart_revision
        or not entries
        or len({e.id for e in entries}) != len(entries)
        or len(source_sha256) != 64
        or any(c not in '0123456789abcdef' for c in source_sha256.lower())
        or not all(math.isfinite(v) and v >= 0 for v in (max_delta_e, ambiguity_delta_e))
    ):
        raise ValueError('supply source digest, versioned chart, unique entries and finite matching thresholds')
    chart = dict(
        id=chart_id,
        revision=chart_revision,
        entries=[asdict(e) for e in entries],
        max_delta_e=max_delta_e,
        ambiguity_delta_e=ambiguity_delta_e,
    )
    rows = {}
    for face, rgb in sorted(faces.items()):
        if not isinstance(face, str) or not face:
            raise ValueError('face IDs must be stable nonempty strings')
        if rgb is None:
            rows[face] = dict(status='no_colour', rgb=None)
            continue
        lab = _rgb_lab(rgb)
        distances = sorted((ciede2000(lab, _rgb_lab(e.rgb)), e.id) for e in entries)
        distance, closest = distances[0]
        matches = [
            entry_id for delta, entry_id in distances if delta <= max_delta_e and delta <= distance + ambiguity_delta_e
        ]
        status = 'unknown' if distance > max_delta_e else 'ambiguous' if len(matches) > 1 else 'resolved'
        entry = next(e for e in entries if e.id == closest)
        rows[face] = dict(
            status=status,
            rgb=tuple(rgb),
            delta_e=distance,
            matches=matches,
            intent=asdict(entry) if status == 'resolved' else None,
        )
    value = dict(source_sha256=source_sha256, chart=chart, chart_sha256=_digest(chart), faces=rows)
    return dict(value, identity=_digest(value))


def attach_intent(node, face_regions, resolution):
    """Attach resolved source-face/region intent to an ordinary operation node.

    Geometry, native targets/allowances and keep-outs must be authored by the
    host. This attaches provenance only, and cannot grant verification/release.
    Distinct face budgets survive; unsupported asymmetry must be refused by
    the host's target adapter instead of being averaged into a global value.
    """
    if not isinstance(node, OperationNode) or not face_regions:
        raise ValueError('supply an OperationNode and explicit source-face to native-region mapping')
    value = {key: v for key, v in resolution.items() if key != 'identity'}
    if _digest(value) != resolution['identity']:
        raise ValueError('intent resolution changed after matching')
    faces = {}
    for face, region in sorted(face_regions.items()):
        row = resolution['faces'].get(face)
        if not region or row is None or row['status'] != 'resolved':
            raise ValueError(f'face {face}: unresolved intent or missing converted region')
        faces[face] = dict(region=region, intent=row['intent'])
    return replace(
        node,
        inputs=dict(
            host_inputs=node.inputs,
            colour_intent=dict(
                identity=resolution['identity'],
                source_sha256=resolution['source_sha256'],
                chart_sha256=resolution['chart_sha256'],
                faces=faces,
            ),
        ),
    )


def confirmed_blank(placements, *, confirmed_id):
    """Select an engineer-confirmed InitialStock from host registration results.

    Each candidate is (placement_id, InitialStock), whose bounds already
    enclose source/placement error in setup coordinates. An ambiguous fit has
    no automatic winner. Weld/rework supplies a NEW complete blank/source
    revision, rather than a material-adding replay delta or guessed allowance.
    """
    from camlib.simulation import InitialStock

    if not isinstance(placements, (tuple, list)):
        raise TypeError('placement proposals must be a finite list or tuple')
    placements = tuple(placements)
    if (
        not confirmed_id
        or len({key for key, _ in placements}) != len(placements)
        or not all(isinstance(blank, InitialStock) for _, blank in placements)
    ):
        raise ValueError('supply distinct measured placement proposals and an explicit confirmed ID')
    matches = [blank for key, blank in placements if key == confirmed_id]
    if len(matches) != 1:
        raise ValueError('confirmed placement must identify one supplied proposal')
    return matches[0]


@dataclass(frozen=True)
class FaceRegion:
    region: object
    nominal_z_mm: float
    upward_normal: tuple = (0, 0, 1)
    keepout_z_mm: tuple | None = None


@dataclass(frozen=True)
class IntentPreparation:
    nodes: tuple
    material_checks: tuple
    protected_prisms: tuple
    identity: str
    source_sha256: str
    chart_sha256: str
    provenance: dict


def intent_targets(node, face_regions, resolution, *, stock, global_check, geometry_tolerance_mm, max_findings):
    """Translate resolved planar faces into actual floors, limits and checks.

    The caller extracts stable CAD faces and supplies a separate full-part final
    check. Local scopes do not protect unmapped geometry. Mixed boundary cells
    retain conservative native bounds; unresolved seams never grant release.
    """
    from camlib import Engine
    from camlib.limits import AvoidArea, Limits
    from camlib.material import Options, Target
    from camlib.operations import Facing, Pocket
    from camlib.planar import PlanarRegion
    from camlib.verification import MaterialCheck, Prism

    if (
        not isinstance(node, OperationNode)
        or not isinstance(node.operation, (Facing, Pocket))
        or not isinstance(face_regions, dict)
        or not face_regions
        or not isinstance(global_check, MaterialCheck)
        or global_check.after_operation is not None
        or (global_check.options is not None and global_check.options.scope_to_region)
    ):
        raise ValueError('intent targets need planar facing/pocket node, explicit faces and a full-part final check')
    if _digest({k: v for k, v in resolution.items() if k != 'identity'}) != resolution['identity']:
        raise ValueError('stale colour resolution')
    mapped = {}
    for face, spec in sorted(face_regions.items()):
        row = resolution['faces'].get(face)
        if (
            not isinstance(spec, FaceRegion)
            or not isinstance(spec.region, PlanarRegion)
            or tuple(spec.upward_normal) != (0, 0, 1)
            or not math.isfinite(spec.nominal_z_mm)
            or row is None
            or row['status'] != 'resolved'
        ):
            raise ValueError(f'{face}: unresolved or unsupported planar face')
        intent = row['intent']
        if not -1 <= intent['lower_mm'] <= 0 <= intent['upper_mm'] <= 1:
            raise ValueError(f'{face}: supported signed tolerance band is -1..0..1 mm')
        mapped[face] = dict(
            region=spec.region.to_dict(),
            nominal_z_mm=spec.nominal_z_mm,
            normal=spec.upward_normal,
            keepout_z_mm=spec.keepout_z_mm,
            intent=intent,
        )
    provenance = _plain(
        dict(
            resolution=resolution,
            faces=mapped,
            node=dict(
                operation=node.operation.to_dict(),
                sources=node.sources,
                depends_on=node.depends_on,
                inputs=node.inputs,
                limits=node.limits.to_dict() if node.limits else None,
            ),
            stock=stock.to_dict(),
            global_check=global_check.to_dict(),
            geometry_tolerance_mm=geometry_tolerance_mm,
            max_findings=max_findings,
        )
    )
    identity = _digest(provenance)
    # Native region construction refuses invalid geometry, rather than using a
    # centre/bounding-box overlap shortcut. Touching edges have no interior.
    with Engine() as engine:
        values = list(face_regions.values())
        for i, a in enumerate(values):
            engine.prepare_region(a.region, tolerance_mm=geometry_tolerance_mm)
            for b in values[:i]:
                overlap = engine.boolean_region(b.region, 'intersection', tolerance_mm=geometry_tolerance_mm)['region']
                if overlap['components']:
                    raise ValueError('face-region interiors overlap; resolve coverage before generation')
    avoid = list(node.limits.avoid if node.limits else ())
    protected = []
    for i, (face, spec) in enumerate(sorted(face_regions.items())):
        if mapped[face]['intent']['keep_out']:
            z = spec.keepout_z_mm
            if z is None or len(z) != 2 or not all(math.isfinite(v) for v in z) or z[0] >= z[1]:
                raise ValueError(f'{face}: keep-out needs explicit ordered finite Z range')
            avoid.append(AvoidArea(spec.region, z[1]))
            protected.append(Prism(f'intent-keepout-{identity[:16]}-{i}', spec.region, *z, role='protected_target'))
    nodes = []
    checks = [global_check]
    previous = None
    for i, (face, spec) in enumerate(sorted(face_regions.items())):
        intent = mapped[face]['intent']
        if intent['keep_out']:
            continue
        floor = spec.nominal_z_mm + intent['allowance_mm']
        if not stock.min_mm[2] <= floor <= stock.max_mm[2]:
            raise ValueError(f'{face}: intent floor outside stock')
        id = f'intent-{identity[:24]}-{i}'
        operation = replace(node.operation, id=id, floor_z_mm=floor)
        contain = spec.region
        if isinstance(operation, Pocket):
            operation = replace(operation, region=spec.region, bottom_allowance_mm=0)
        if node.limits and node.limits.contain is not None:
            with Engine() as engine:
                engine.prepare_region(contain, tolerance_mm=geometry_tolerance_mm)
                contain = PlanarRegion.from_dict(
                    engine.boolean_region(node.limits.contain, 'intersection', tolerance_mm=geometry_tolerance_mm)[
                        'region'
                    ]
                )
        dependencies = tuple(dict.fromkeys((*node.depends_on, *((previous,) if previous else ()))))
        nodes.append(
            replace(
                node,
                operation=operation,
                depends_on=dependencies,
                limits=Limits(contain, tuple(avoid)),
                inputs=dict(
                    host_inputs=node.inputs,
                    colour_intent=provenance,
                    colour_intent_identity=resolution['identity'],
                    conversion_identity=identity,
                    face=face,
                    geometry=mapped[face],
                ),
            )
        )
        previous = id
        checks.append(
            MaterialCheck(
                Target(f'face-{identity[:24]}-{i}', identity, spec.region, floor),
                options=Options(
                    geometry_tolerance_mm=geometry_tolerance_mm,
                    residual_tolerance_mm=intent['upper_mm'],
                    gouge_tolerance_mm=-intent['lower_mm'],
                    scope_to_region=True,
                    max_findings=max_findings,
                ),
            )
        )
    return IntentPreparation(
        tuple(nodes),
        tuple(checks),
        tuple(protected),
        identity,
        resolution['source_sha256'],
        resolution['chart_sha256'],
        provenance,
    )


def planar_cad_faces(faces, *, source, max_error_mm, work_frame=None, cancel=None):
    """Extract stable caller-ID CadQuery faces through the existing bounded adapter.

    All faces must be upward horizontal planes in the declared work frame;
    unsupported faces are refused explicitly rather than omitted.
    """
    from camlib.cadquery import export_floor_region, _fixture_frame

    if not isinstance(faces, dict) or not faces or not isinstance(source, dict):
        raise ValueError('supply stable face IDs and explicit source provenance')
    result = {}
    for face_id, face in sorted(faces.items()):
        placed = _fixture_frame(face, work_frame)[0] if work_frame is not None else face
        normal = placed.normalAt()
        if placed.geomType() != 'PLANE' or abs(normal.x) > 1e-12 or abs(normal.y) > 1e-12 or normal.z < 1 - 1e-12:
            raise ValueError(f'{face_id}: only upward horizontal planar CAD faces are supported')
        region = export_floor_region(
            face,
            name=face_id,
            source=dict(source, face_id=face_id),
            work_frame=work_frame,
            max_error_mm=max_error_mm,
            cancel=cancel,
        )
        result[face_id] = FaceRegion(region, placed.Center().z)
    return result
