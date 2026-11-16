"""Speeds and feeds from cutting data, using the standard machining formulas.

camlib supplies no cutting-data defaults. Surface speed and feed per tooth come from
the tool maker for the tool, the material and the machine. These helpers only
apply the usual formulas (as given in tooling handbooks):

* spindle speed ``n = 1000 * Vc / (pi * D)`` (rpm)
* table feed ``vf = n * z * fz`` (mm/min)
* radial chip thinning, for a radial engagement ``ae < D / 2``: the chip is
  thinner than ``fz``, so ``fz`` is raised by ``D / (2 * sqrt(ae * (D - ae)))``
  to keep the recommended chip thickness
* ball end mill effective diameter at axial depth ``ap < D / 2``:
  ``De = 2 * sqrt(ap * (D - ap))``; speed is computed at ``De``

Results go straight into an operation's ``spindle_rpm`` and ``feed_mm_min``.
Opt-in drilling recommendations match explicitly supplied tool/material records;
they do not certify cutting conditions or grant clearance.
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, replace
import json
import math
from itertools import islice

from .tools import Cutter, ToolAssembly


@dataclass(frozen=True)
class EngagementFeedBand:
    at_least_angle_deg: float
    max_feed_mm_min: float


@dataclass(frozen=True)
class ToolLoadLimit:
    tool: str
    max_axial_mm: float
    max_radial_mm: float
    max_cut_feed_mm_min: float


@dataclass(frozen=True)
class EngagementFeedPolicy:
    """Caller-declared feed/load caps; unresolved is 'refuse' or 'preserve'.

    Options, when supplied, are the native EngagementOptions numerical/work
    budgets. Optional air increases require a whole-sweep original-stock miss;
    tool_limits enables conservative general-shape load bounds. No cutting-data
    defaults or chip-thinning guarantee.
    """

    bands: tuple
    unresolved: str
    options: dict | None = None
    air_feed_mm_min: float | None = None
    tool_limits: tuple = ()

    def __post_init__(self):
        if not isinstance(self.bands, (list, tuple)) or not all(isinstance(b, EngagementFeedBand) for b in self.bands):
            raise TypeError('feed bands must be a finite collection of EngagementFeedBand values')
        object.__setattr__(self, 'bands', tuple(self.bands))
        if not isinstance(self.tool_limits, (list, tuple)) or not all(
            isinstance(limit, ToolLoadLimit) for limit in self.tool_limits
        ):
            raise TypeError('tool_limits must be a finite collection of ToolLoadLimit values')
        object.__setattr__(self, 'tool_limits', tuple(self.tool_limits))

    def to_dict(self):
        value = asdict(self)
        if value['options'] is None:
            value.pop('options')
        return value


@dataclass(frozen=True)
class EngagementScheduledProgram:
    program: object
    report: dict


def schedule_engagement(program, stock, *, policy, initial_stock=None, sweep=None, cancel=None):
    """Observe committed-stock engagement and apply explicitly declared caps.

    Horizontal flat-cutter cuts retain legacy behavior; tool_limits enables
    general-shape bounds and air_feed_mm_min permits only certified air changes.
    Other cut shapes/motions follow
    the explicit unresolved policy; tapping and canned-cycle motions retain
    their synchronized/authored feeds. This is offline, does not mutate the
    input, and returns a new revision bound to stock/policy/input identity.
    Follow with Machine.schedule and ordinary checked posted replay/export.
    """
    from ._buffers import native
    from .motion import Motion, Program
    from .simulation import SweepOptions

    if not isinstance(program, Program) or not isinstance(policy, EngagementFeedPolicy):
        raise TypeError('supply a typed Program and EngagementFeedPolicy')
    snapshot = deepcopy(program)
    stock_data = stock.to_dict() if hasattr(stock, 'to_dict') else stock
    initial = initial_stock.to_dict() if hasattr(initial_stock, 'to_dict') else initial_stock
    sweep_data = asdict(sweep or SweepOptions()) if not isinstance(sweep, dict) else sweep
    report = json.loads(
        native.schedule_engagement(
            json.dumps(snapshot.to_dict(), allow_nan=False),
            json.dumps((stock_data, initial, sweep_data), allow_nan=False),
            json.dumps(policy.to_dict(), allow_nan=False),
            cancel,
        )
    )
    events = list(snapshot.events)
    for row in report['observations']:
        index = row['event_index']
        event = events[index]
        if (
            not isinstance(event, Motion)
            or event.id != row['motion_id']
            or event.feed_mm_min != row['original_feed_mm_min']
        ):
            raise RuntimeError('engagement report/source event identity mismatch')
        feed = row['requested_feed_mm_min']
        if feed is not None and (
            feed <= 0
            or (
                feed > event.feed_mm_min
                and (
                    row['status'] != 'proven_air'
                    or not row['outside_original_stock']
                    or policy.air_feed_mm_min is None
                    or feed > policy.air_feed_mm_min
                )
            )
        ):
            raise RuntimeError('engagement feed increase lacks declared whole-sweep air proof')
        events[index] = replace(event, feed_mm_min=feed)
    return EngagementScheduledProgram(replace(snapshot, revision=report['revision'], events=tuple(events)), report)


@dataclass(frozen=True)
class CuttingData:
    spindle_rpm: float
    feed_mm_min: float
    feed_per_tooth_mm: float
    """Programmed feed per tooth (raised by chip-thinning compensation)."""
    effective_diameter_mm: float
    rpm_limited: bool
    """True when ``max_rpm`` capped the speed; the feed then follows the
    capped speed, so the feed per tooth is kept."""


def _positive(name, value):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be a positive finite number, got {value!r}")
    return float(value)


def cutting_data(
    tool,
    *,
    surface_speed_m_min,
    feed_per_tooth_mm,
    radial_engagement_mm=None,
    axial_depth_mm=None,
    max_rpm=None,
    chip_thinning=True,
):
    """Spindle speed and feed for ``tool`` (a ``Cutter`` or ``ToolAssembly``).

    ``radial_engagement_mm`` (ae) enables chip-thinning compensation when it is
    below half the diameter (set ``chip_thinning=False`` to use ``fz`` as given).
    ``axial_depth_mm`` (ap) gives a ball end mill's effective cutting diameter.
    ``max_rpm`` caps the speed at the machine's limit, reported in the result.
    """
    cutter = tool.cutter if isinstance(tool, ToolAssembly) else tool
    if not isinstance(cutter, Cutter):
        raise TypeError("tool must be a Cutter or ToolAssembly")
    vc = _positive("surface_speed_m_min", surface_speed_m_min)
    fz = _positive("feed_per_tooth_mm", feed_per_tooth_mm)
    d = _positive("cutter diameter", cutter.diameter_mm)
    if type(cutter.flutes) is not int or cutter.flutes < 1:
        raise ValueError(f"cutter flutes must be a positive integer, got {cutter.flutes!r}")
    effective = d
    if axial_depth_mm is not None:
        ap = _positive("axial_depth_mm", axial_depth_mm)
        if dict(cutter.shape).get("kind") == "ball" and ap < d / 2:
            effective = 2 * math.sqrt(ap * (d - ap))
    if radial_engagement_mm is not None:
        ae = _positive("radial_engagement_mm", radial_engagement_mm)
        if ae > d:
            raise ValueError(f"radial_engagement_mm {ae} exceeds the cutter diameter {d}")
        if chip_thinning and ae < effective / 2:
            fz *= effective / (2 * math.sqrt(ae * (effective - ae)))
    rpm = 1000 * vc / (math.pi * effective)
    limited = False
    if max_rpm is not None:
        cap = _positive("max_rpm", max_rpm)
        if rpm > cap:
            rpm, limited = cap, True
    return CuttingData(rpm, rpm * cutter.flutes * fz, fz, effective, limited)


@dataclass(frozen=True)
class DrillingCuttingData:
    """Caller-approved data for one tool, diameter, material and stated conditions.

    Vc is an allowed range, fn is mm/revolution, and depth ratio includes the
    tip. Source/conditions must identify the actual data, including any coolant,
    tool-system and material-condition requirements the caller must establish.
    """

    id: str
    tool: str
    material: str
    source: str
    conditions: str
    diameter_mm: float
    vc_min_m_min: float
    vc_max_m_min: float
    feed_per_rev_mm: float
    max_depth_diameter_ratio: float

    def to_dict(self):
        for name in ('id', 'tool', 'material', 'source', 'conditions'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip() or len(value) > 4096:
                raise ValueError(f'{name} must be a nonempty, bounded string')
        for name in ('diameter_mm', 'vc_min_m_min', 'vc_max_m_min', 'feed_per_rev_mm', 'max_depth_diameter_ratio'):
            if _positive(name, getattr(self, name)) > 2e6:
                raise ValueError(f'{name} exceeds the supported bound')
        if self.vc_min_m_min > self.vc_max_m_min:
            raise ValueError('vc_min_m_min must not exceed vc_max_m_min')
        return asdict(self)


@dataclass(frozen=True)
class DrillingRecommendation:
    tool: ToolAssembly
    data: DrillingCuttingData
    cutting: CuttingData
    actual_surface_speed_m_min: float
    feed_limited: bool


@dataclass(frozen=True)
class DrillingRecommendations:
    candidates: tuple[DrillingRecommendation, ...]
    rejected: tuple[dict, ...]
    scope: str = 'Declared nominal cutting data and reach only; ordinary stock, assembly, machine and posted-output checks retain authority'


def recommend_drilling(
    tools,
    data,
    *,
    material,
    diameter_mm,
    tip_depth_mm,
    diameter_tolerance_mm,
    machine,
    max_candidates=10000,
    cancel=None,
):
    """Rank declared candidates by shortest adequate flutes, then nominal feed and IDs.

    Returns proposals without changing tools, operations or source data. Every
    candidate requires an exact material/data-diameter match and a registered
    conical drill. Speed/feed caps may lower Vc only within its supplied range.
    ``cancel`` is a zero-argument predicate (e.g. threading.Event.is_set).
    No matches is an empty candidate list with reasons, never a default recipe.

    Sandvik drilling formulas, metric H81: n=1000*Vc/(pi*D), vf=fn*n.
    https://cdn.sandvik.coromant.com/files/sitecollectiondocuments/services/metal-cutting-e-learning/formulas-and-definitions/formulas-and-deinitions-for-drilling-metric-enu.pdf
    """
    from .postprocessor import Machine

    if not isinstance(machine, Machine):
        raise TypeError('Supply a declared Machine')
    if not isinstance(material, str) or not material.strip() or len(material) > 4096:
        raise ValueError('material must be an explicit bounded identity')
    d, depth = _positive('diameter_mm', diameter_mm), _positive('tip_depth_mm', tip_depth_mm)
    if (
        isinstance(diameter_tolerance_mm, bool)
        or not isinstance(diameter_tolerance_mm, (int, float))
        or not math.isfinite(diameter_tolerance_mm)
        or not 0 <= diameter_tolerance_mm < d
    ):
        raise ValueError('diameter_tolerance_mm must be finite, nonnegative and smaller than diameter')
    if type(max_candidates) is not int or not 1 <= max_candidates <= 100000:
        raise ValueError('max_candidates must be an integer in 1..100000')
    max_rpm = _positive('machine max_spindle_rpm', machine.max_spindle_rpm)
    max_feed = _positive('machine max_feed_mm_min', machine.max_feed_mm_min)
    if machine.max_axis_feed_mm_min is not None:
        max_feed = min(max_feed, _positive('machine Z feed limit', machine.max_axis_feed_mm_min[2]))

    def poll():
        if cancel is not None and cancel():
            raise RuntimeError('drilling recommendation cancelled; no proposal published')

    def bounded(values, name):
        result = []
        for value in islice(values, max_candidates + 1):
            poll()
            result.append(value)
        if len(result) > max_candidates:
            raise ValueError(f'{name} exceeds max_candidates; no partial proposal')
        return result

    by_id = {}
    for tool in bounded(tools, 'tools'):
        if not isinstance(tool, ToolAssembly):
            raise TypeError('tools must contain declared ToolAssembly objects')
        if tool.id in by_id:
            raise ValueError(f'duplicate tool ID {tool.id}')
        by_id[tool.id] = tool
    registered = {entry.tool: entry for entry in bounded(machine.tools, "machine tools")}
    if len(registered) != len(machine.tools):
        raise ValueError("duplicate tool ID in the declared machine table")
    candidates, rejected, seen = [], [], set()
    for row in bounded(data, 'cutting data'):
        poll()
        if not isinstance(row, DrillingCuttingData):
            raise TypeError('data must contain DrillingCuttingData records')
        row.to_dict()
        if (row.tool, row.id) in seen:
            raise ValueError('duplicate tool/cutting-data ID')
        seen.add((row.tool, row.id))
        tool = by_id.get(row.tool)
        try:
            if row.material != material:
                raise ValueError('material does not match the declared record')
            if tool is None or tool.id not in registered:
                raise ValueError('tool is unavailable or absent from the declared machine table')
            cutter = tool.cutter
            if registered[tool.id].gauge_length_mm != tool.gauge_length_mm:
                raise ValueError('machine table gauge length does not match the assembly')
            if abs(cutter.diameter_mm - d) > diameter_tolerance_mm or row.diameter_mm != cutter.diameter_mm:
                raise ValueError('tool/request/data diameters do not match')
            shape = dict(cutter.shape)
            angle = shape.get('point_angle_rad')
            if (
                shape.get('kind') != 'drill'
                or isinstance(angle, bool)
                or not isinstance(angle, (int, float))
                or not math.isfinite(angle)
                or not 0 < angle < math.pi
            ):
                raise ValueError('an explicitly declared conical drill point is required')
            point = cutter.diameter_mm / (2 * math.tan(angle / 2))
            if (
                not math.isfinite(cutter.cutting_length_mm)
                or not point <= cutter.cutting_length_mm
                or depth > cutter.cutting_length_mm
                or depth > row.max_depth_diameter_ratio * cutter.diameter_mm
            ):
                raise ValueError('tip depth/point exceeds the declared flutes or data depth ratio')
            if type(cutter.flutes) is not int or cutter.flutes < 1:
                raise ValueError('cutter flutes must be a positive integer')
            cap = min(max_rpm, max_feed / row.feed_per_rev_mm)
            result = cutting_data(
                tool,
                surface_speed_m_min=row.vc_max_m_min,
                feed_per_tooth_mm=row.feed_per_rev_mm / cutter.flutes,
                max_rpm=cap,
                chip_thinning=False,
            )
            vc = math.pi * cutter.diameter_mm * result.spindle_rpm / 1000
            if vc < row.vc_min_m_min - 8 * math.ulp(row.vc_min_m_min):
                raise ValueError('machine limits cannot satisfy the declared cutting-speed range')
            # Keep floating-point roundoff from exceeding the declared feed cap.
            feed = min(row.feed_per_rev_mm * result.spindle_rpm, max_feed)
            if not all(math.isfinite(v) and v > 0 for v in (result.spindle_rpm, feed, vc)):
                raise ValueError('derived speed/feed must remain finite and positive')
            result = replace(result, feed_mm_min=feed)
            candidates.append(
                DrillingRecommendation(
                    tool,
                    row,
                    result,
                    vc,
                    max_feed / row.feed_per_rev_mm
                    < min(max_rpm, 1000 * row.vc_max_m_min / (math.pi * cutter.diameter_mm)),
                )
            )
        except (ValueError, ZeroDivisionError, OverflowError) as exc:
            rejected.append({'tool': row.tool, 'data': row.id, 'reason': str(exc)})
    poll()
    covered = {tool for tool, _ in seen}
    rejected.extend(
        {'tool': tool, 'data': None, 'reason': 'no supplied cutting-data record'}
        for tool in by_id
        if tool not in covered
    )
    candidates.sort(key=lambda r: (r.tool.cutter.cutting_length_mm, -r.cutting.feed_mm_min, r.tool.id, r.data.id))
    rejected.sort(key=lambda r: (r['tool'], r['data'] or ''))
    return DrillingRecommendations(tuple(candidates), tuple(rejected))
