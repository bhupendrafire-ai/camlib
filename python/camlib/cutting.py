"""Speeds and feeds from cutting data, using the standard machining formulas.

camlib never chooses cutting data. Surface speed and feed per tooth come from
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
"""

from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass, replace
import json
import math

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
