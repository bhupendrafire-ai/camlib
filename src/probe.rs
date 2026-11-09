//! Three-axis probing: work-offset, feature and tool-setting cycles.
//!
//! camlib never talks to a machine, so probing here means planning,
//! simulating and posting probing cycles, not reading results back. A cycle
//! is one [`ProbeCycle`] (what is measured, where, and what the controller
//! stores) and the moves [`ProbeCycle::legs`] lays out for it:
//!
//! * **protected positioning** ([`MoveKind::ProbePosition`]): every probe
//!   move that is not a measuring move stops the machine if the stylus
//!   triggers (LinuxCNC `G38.3` with a trigger check; Renishaw Inspection
//!   Plus `O9810`, "the machine will stop in the event of a collision",
//!   H-2000-6222 chapter 5);
//! * **measuring moves**, each split at `search_mm` before the expected
//!   contact into [`MoveKind::ProbeMeasure`] (no contact expected: the whole
//!   assembly must be clear) and [`MoveKind::ProbeTouch`] (the stylus is
//!   expected to touch here), then [`MoveKind::ProbeReturn`] back to where
//!   the measuring move started. The controller searches up to `search_mm`
//!   beyond the expected contact (LinuxCNC: the `G38.2` end point; Inspection
//!   Plus: the `Q` overtravel, "the probe will then travel beyond the expected
//!   position when it searches for a surface", H-2000-6031 chapter 7). So the
//!   surface is expected within `search_mm` of its nominal position.
//!
//! The layout of each feature follows the Inspection Plus macros it can be
//! posted as (H-2000-6031, chapter 7): a single surface measures from its
//! start along one axis and returns there (`O9811`); a bore measures along
//! +X, -X, +Y and -Y from its centre (`O9814`, "four measuring moves along
//! the X Y axis"); a pocket along one axis from its centre (`O9812`); an
//! inside corner measures the Y surface, then the X surface, from one start
//! point and returns there (`O9815` without I/J: "only two gauging moves
//! occur. The corner feature is assumed to be parallel to the axes"). The
//! macros' internal order of the moves along those lines is the vendor's;
//! the swept moves are the same.
//!
//! The probe's controlled point is the bottom of the stylus ball, as for a
//! ball-end tool: an XY touch happens at the ball's equator, `ball radius`
//! above the tip. The ball radius is the declared (calibrated) one: a
//! premise, as are the controller's stored probe calibration and its
//! stopping distance after a trigger (within the stylus overtravel).
//!
//! Probing moves never change the simulated stock ([`MoveKind::cuts`]).
use crate::{
    motion::{Coolant, Curve, Event, Motion, MoveKind, SpindleDirection, Vec3},
    stock::{CancelSignal, Stock, StockSpec},
    tool::{CutterShape, ToolAssembly, valid_id},
};
use serde::{Deserialize, Serialize};

/// A cartesian axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Axis {
    X,
    Y,
    Z,
}
impl Axis {
    pub fn index(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }
}

/// What a probing cycle measures. Nominal geometry in work coordinates
/// (mm). `z_mm` is the probe tip (the bottom of the stylus ball) during the
/// XY touches.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ProbeFeature {
    /// One surface square to `axis` at `surface_mm`, measured from
    /// `start_mm` (the tip) along `axis` toward it. A Z surface is measured
    /// from above (the tip touches it).
    Surface {
        start_mm: Vec3,
        axis: Axis,
        surface_mm: f64,
    },
    /// An outside corner at `corner_mm`: the part's material lies on the
    /// `material` side (+1 or -1 per axis) of both faces. Each face is
    /// touched `inset_mm` along it from the corner, from `clearance_mm`
    /// away; the Y face first.
    OutsideCorner {
        corner_mm: [f64; 2],
        z_mm: f64,
        material: [i8; 2],
        inset_mm: f64,
        clearance_mm: f64,
    },
    /// An inside corner at `corner_mm`: the walls lie on the `material` side
    /// (+1 or -1 per axis) of the start point, which is `clearance_mm` from
    /// both walls. The Y wall first, then the X wall.
    InsideCorner {
        corner_mm: [f64; 2],
        z_mm: f64,
        material: [i8; 2],
        clearance_mm: f64,
    },
    /// A bore, from its centre: +X, -X, +Y, -Y.
    Bore {
        center_mm: [f64; 2],
        z_mm: f64,
        diameter_mm: f64,
    },
    /// A boss, from outside: each of +X, -X, +Y, -Y from `clearance_mm`
    /// outside it, reached over `retract_z_mm`.
    Boss {
        center_mm: [f64; 2],
        z_mm: f64,
        diameter_mm: f64,
        clearance_mm: f64,
    },
    /// A pocket (slot) of `width_mm` across `axis` (X or Y), from its
    /// centre: + then -.
    Pocket {
        center_mm: [f64; 2],
        z_mm: f64,
        axis: Axis,
        width_mm: f64,
    },
    /// A web (rib) of `width_mm` across `axis` (X or Y), from outside: the
    /// + side then the - side, each from `clearance_mm` outside it.
    Web {
        center_mm: [f64; 2],
        z_mm: f64,
        axis: Axis,
        width_mm: f64,
        clearance_mm: f64,
    },
    /// Tool length on a tool setter: the tool's tip comes down onto the
    /// setter's contact face at `contact_mm` (tip position at contact) from
    /// `clearance_mm` above it.
    ToolLength { contact_mm: Vec3, clearance_mm: f64 },
    /// Tool diameter on a tool setter: the cutter's side touches the
    /// setter's round contact face (`setter_diameter_mm`, top at
    /// `setter_top_z_mm`) from +X and from -X, the tip `depth_mm` below its
    /// top, from `clearance_mm` away; it rises `clearance_mm` above the top
    /// to cross over.
    ToolDiameter {
        setter_center_mm: [f64; 2],
        setter_top_z_mm: f64,
        setter_diameter_mm: f64,
        depth_mm: f64,
        clearance_mm: f64,
    },
}

/// What the controller stores from a cycle: a premise for what follows,
/// recorded in the program, not checked by camlib.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ProbeResult {
    /// The work offset (1 = G54 .. 9) is corrected by the measured error
    /// of the feature's position in the axes the feature measures: new
    /// offset = active offset + error (Inspection Plus `S`; LinuxCNC
    /// `G10 L2`).
    WorkOffset { number: u8 },
    /// The tool length in the controller's length register (`H`) becomes
    /// the measured one (LinuxCNC `G10 L10`, then `G43 H` again).
    ToolLength { register: u32 },
    /// The tool radius in the controller's tool table entry becomes the
    /// measured one (LinuxCNC `G10 L1 R`). Not wear: a program using
    /// camlib's wear compensation on that tool afterwards is refused.
    ToolRadius { register: u32 },
}

/// One probing cycle.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProbeCycle {
    pub feature: ProbeFeature,
    /// The protected positioning height every stylus cycle starts and ends
    /// at (None for tool setting, whose cutting tool is not protected).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_z_mm: Option<f64>,
    /// The surface is expected within this distance of its nominal position:
    /// the measuring move starts touching `search_mm` before the nominal
    /// contact and searches `search_mm` beyond it.
    pub search_mm: f64,
    pub result: ProbeResult,
    /// Protected positioning feed (stylus cycles only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub position_feed_mm_min: Option<f64>,
    /// The measuring move's feed (its search and its touch).
    pub measure_feed_mm_min: f64,
    /// The feed back from a contact.
    pub return_feed_mm_min: f64,
}

/// One move of a cycle: its kind, end point (each starts where the previous
/// one ended) and feed (None: rapid).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Leg {
    pub kind: MoveKind,
    pub end: Vec3,
    pub feed_mm_min: Option<f64>,
}

const LIMIT_MM: f64 = 1e6;
fn finite(v: f64) -> bool {
    v.is_finite() && v.abs() <= LIMIT_MM
}
fn positive(v: f64) -> bool {
    v.is_finite() && v > 0. && v <= LIMIT_MM
}
fn sign(v: i8) -> Result<f64, String> {
    match v {
        1 => Ok(1.),
        -1 => Ok(-1.),
        _ => Err("probe material sides must be +1 or -1 per axis".into()),
    }
}

impl ProbeFeature {
    /// Whether the feature is measured with a probe stylus (otherwise it is
    /// tool setting: the measured cutting tool on a tool setter).
    pub fn stylus(&self) -> bool {
        !matches!(
            self,
            ProbeFeature::ToolLength { .. } | ProbeFeature::ToolDiameter { .. }
        )
    }
    pub fn name(&self) -> &'static str {
        match self {
            ProbeFeature::Surface { .. } => "surface",
            ProbeFeature::OutsideCorner { .. } => "outside_corner",
            ProbeFeature::InsideCorner { .. } => "inside_corner",
            ProbeFeature::Bore { .. } => "bore",
            ProbeFeature::Boss { .. } => "boss",
            ProbeFeature::Pocket { .. } => "pocket",
            ProbeFeature::Web { .. } => "web",
            ProbeFeature::ToolLength { .. } => "tool_length",
            ProbeFeature::ToolDiameter { .. } => "tool_diameter",
        }
    }
    /// The axes whose work offset a work-offset result corrects.
    pub fn measured_axes(&self) -> &'static [Axis] {
        match self {
            ProbeFeature::Surface { axis, .. }
            | ProbeFeature::Pocket { axis, .. }
            | ProbeFeature::Web { axis, .. } => match axis {
                Axis::X => &[Axis::X],
                Axis::Y => &[Axis::Y],
                Axis::Z => &[Axis::Z],
            },
            ProbeFeature::OutsideCorner { .. }
            | ProbeFeature::InsideCorner { .. }
            | ProbeFeature::Bore { .. }
            | ProbeFeature::Boss { .. } => &[Axis::X, Axis::Y],
            ProbeFeature::ToolLength { .. } | ProbeFeature::ToolDiameter { .. } => &[],
        }
    }
}

/// Builds the legs of a cycle from its start point.
struct Layout {
    legs: Vec<Leg>,
    at: Vec3,
    search: f64,
    feeds: [Option<f64>; 3],
}
impl Layout {
    fn to(&mut self, kind: MoveKind, end: Vec3) -> Result<(), String> {
        if end == self.at {
            return Err("a probing move has zero length".into());
        }
        let feed_mm_min = match kind {
            MoveKind::ProbePosition => self.feeds[0],
            MoveKind::ProbeMeasure | MoveKind::ProbeTouch => self.feeds[1],
            MoveKind::ProbeReturn => self.feeds[2],
            _ => None,
        };
        self.legs.push(Leg {
            kind,
            end,
            feed_mm_min,
        });
        self.at = end;
        Ok(())
    }
    /// Protected positioning to `end`.
    fn position(&mut self, end: Vec3) -> Result<(), String> {
        self.to(MoveKind::ProbePosition, end)
    }
    /// A measuring move from here along `axis` in direction `dir` to the
    /// expected contact `contact` (that axis's coordinate of the tip), then
    /// back here.
    fn touch(&mut self, axis: usize, dir: f64, contact: f64) -> Result<(), String> {
        let start = self.at;
        let approach = dir * (contact - start[axis]);
        if approach <= self.search {
            return Err(format!(
                "the measuring move toward {contact:.4} along {} is {approach:.4} mm, not longer than search_mm {}: start farther from the surface or search less",
                ["X", "Y", "Z"][axis],
                self.search
            ));
        }
        let mut near = start;
        near[axis] = contact - dir * self.search;
        let mut end = start;
        end[axis] = contact;
        self.to(MoveKind::ProbeMeasure, near)?;
        self.to(MoveKind::ProbeTouch, end)?;
        self.to(MoveKind::ProbeReturn, start)
    }
}

impl ProbeCycle {
    /// Validate the cycle on its own (geometry, signs, search).
    pub fn validate(&self) -> Result<(), String> {
        if !positive(self.search_mm) {
            return Err("search_mm must be positive".into());
        }
        let stylus = self.feature.stylus();
        if !positive(self.measure_feed_mm_min)
            || !positive(self.return_feed_mm_min)
            || self.position_feed_mm_min.is_some_and(|f| !positive(f))
            || self.position_feed_mm_min.is_some() != stylus
        {
            return Err(
                "probing feeds must be positive; a protected positioning feed is given exactly for stylus cycles".into(),
            );
        }
        match (&self.result, stylus) {
            (ProbeResult::WorkOffset { number }, true) => {
                if !(1..=9).contains(number) {
                    return Err("a probed work offset must be 1..9 (G54..G59.3)".into());
                }
            }
            (ProbeResult::ToolLength { register }, false)
                if matches!(self.feature, ProbeFeature::ToolLength { .. }) =>
            {
                if *register == 0 {
                    return Err("the tool length register must be positive".into());
                }
            }
            (ProbeResult::ToolRadius { register }, false)
                if matches!(self.feature, ProbeFeature::ToolDiameter { .. }) =>
            {
                if *register == 0 {
                    return Err("the tool table entry must be positive".into());
                }
            }
            _ => {
                return Err(format!(
                    "a {} cycle stores {}",
                    self.feature.name(),
                    match self.feature {
                        ProbeFeature::ToolLength { .. } => "a tool length",
                        ProbeFeature::ToolDiameter { .. } => "a tool radius",
                        _ => "a work offset",
                    }
                ));
            }
        }
        match (self.retract_z_mm, stylus) {
            (Some(z), true) if finite(z) => {}
            (None, false) => {}
            _ => {
                return Err(
                    "a stylus cycle needs a finite retract_z_mm; tool setting has none".into(),
                );
            }
        }
        let xy = |p: &[f64; 2]| p.iter().all(|v| finite(*v));
        let ok = match &self.feature {
            ProbeFeature::Surface {
                start_mm,
                surface_mm,
                ..
            } => start_mm.iter().all(|v| finite(*v)) && finite(*surface_mm),
            ProbeFeature::OutsideCorner {
                corner_mm,
                z_mm,
                material,
                inset_mm,
                clearance_mm,
            } => {
                sign(material[0])?;
                sign(material[1])?;
                xy(corner_mm) && finite(*z_mm) && positive(*inset_mm) && positive(*clearance_mm)
            }
            ProbeFeature::InsideCorner {
                corner_mm,
                z_mm,
                material,
                clearance_mm,
            } => {
                sign(material[0])?;
                sign(material[1])?;
                xy(corner_mm) && finite(*z_mm) && positive(*clearance_mm)
            }
            ProbeFeature::Bore {
                center_mm,
                z_mm,
                diameter_mm,
            } => xy(center_mm) && finite(*z_mm) && positive(*diameter_mm),
            ProbeFeature::Boss {
                center_mm,
                z_mm,
                diameter_mm,
                clearance_mm,
            } => {
                xy(center_mm) && finite(*z_mm) && positive(*diameter_mm) && positive(*clearance_mm)
            }
            ProbeFeature::Pocket {
                center_mm,
                z_mm,
                axis,
                width_mm,
            } => {
                if *axis == Axis::Z {
                    return Err("a pocket is measured across X or Y".into());
                }
                xy(center_mm) && finite(*z_mm) && positive(*width_mm)
            }
            ProbeFeature::Web {
                center_mm,
                z_mm,
                axis,
                width_mm,
                clearance_mm,
            } => {
                if *axis == Axis::Z {
                    return Err("a web is measured across X or Y".into());
                }
                xy(center_mm) && finite(*z_mm) && positive(*width_mm) && positive(*clearance_mm)
            }
            ProbeFeature::ToolLength {
                contact_mm,
                clearance_mm,
            } => contact_mm.iter().all(|v| finite(*v)) && positive(*clearance_mm),
            ProbeFeature::ToolDiameter {
                setter_center_mm,
                setter_top_z_mm,
                setter_diameter_mm,
                depth_mm,
                clearance_mm,
            } => {
                xy(setter_center_mm)
                    && finite(*setter_top_z_mm)
                    && positive(*setter_diameter_mm)
                    && positive(*depth_mm)
                    && positive(*clearance_mm)
            }
        };
        if !ok {
            return Err(format!(
                "{} probing: every coordinate must be finite and every size, clearance and inset positive",
                self.feature.name()
            ));
        }
        Ok(())
    }

    /// Where the cycle starts: the tip position its first move leaves from.
    /// `radius` is the stylus ball radius, or for tool setting the
    /// measured cutter's radius.
    pub fn start(&self, radius: f64) -> Result<Vec3, String> {
        let r = radius;
        let retract = self.retract_z_mm.unwrap_or(0.);
        Ok(match &self.feature {
            ProbeFeature::Surface { start_mm, .. } => [start_mm[0], start_mm[1], retract],
            ProbeFeature::OutsideCorner {
                corner_mm: c,
                material: m,
                inset_mm,
                clearance_mm,
                ..
            } => {
                let (sx, sy) = (sign(m[0])?, sign(m[1])?);
                [
                    c[0] + sx * inset_mm,
                    c[1] - sy * (r + clearance_mm),
                    retract,
                ]
            }
            ProbeFeature::InsideCorner {
                corner_mm: c,
                material: m,
                clearance_mm,
                ..
            } => {
                let (sx, sy) = (sign(m[0])?, sign(m[1])?);
                [
                    c[0] - sx * (r + clearance_mm),
                    c[1] - sy * (r + clearance_mm),
                    retract,
                ]
            }
            ProbeFeature::Bore { center_mm: c, .. } | ProbeFeature::Pocket { center_mm: c, .. } => {
                [c[0], c[1], retract]
            }
            ProbeFeature::Boss {
                center_mm: c,
                diameter_mm,
                clearance_mm,
                ..
            } => [c[0] + diameter_mm / 2. + r + clearance_mm, c[1], retract],
            ProbeFeature::Web {
                center_mm: c,
                axis,
                width_mm,
                clearance_mm,
                ..
            } => {
                let mut p = [c[0], c[1], retract];
                p[axis.index()] += width_mm / 2. + r + clearance_mm;
                p
            }
            ProbeFeature::ToolLength {
                contact_mm,
                clearance_mm,
            } => [contact_mm[0], contact_mm[1], contact_mm[2] + clearance_mm],
            ProbeFeature::ToolDiameter {
                setter_center_mm: c,
                setter_top_z_mm,
                setter_diameter_mm,
                depth_mm,
                clearance_mm,
            } => [
                c[0] + setter_diameter_mm / 2. + r + clearance_mm,
                c[1],
                setter_top_z_mm - depth_mm,
            ],
        })
    }

    /// The moves of the cycle, from [`Self::start`]. `radius` as there.
    pub fn legs(&self, radius: f64) -> Result<Vec<Leg>, String> {
        self.validate()?;
        if !positive(radius) {
            return Err("the probe or tool radius must be positive".into());
        }
        let r = radius;
        let start = self.start(r)?;
        let retract = self.retract_z_mm.unwrap_or(0.);
        let mut l = Layout {
            legs: vec![],
            at: start,
            search: self.search_mm,
            feeds: [
                self.position_feed_mm_min,
                Some(self.measure_feed_mm_min),
                Some(self.return_feed_mm_min),
            ],
        };
        let below = |z: f64| -> Result<(), String> {
            if z >= retract {
                return Err(format!(
                    "the probing height {z} must be below retract_z_mm {retract}"
                ));
            }
            Ok(())
        };
        let at = |p: [f64; 2], z: f64| [p[0], p[1], z];
        match &self.feature {
            ProbeFeature::Surface {
                start_mm,
                axis,
                surface_mm,
            } => {
                below(start_mm[2])?;
                l.position(*start_mm)?;
                let a = axis.index();
                if *axis == Axis::Z {
                    if *surface_mm >= start_mm[2] {
                        return Err(
                            "a Z surface is probed from above: start above the surface".into()
                        );
                    }
                    l.touch(a, -1., *surface_mm)?;
                } else {
                    let dir = (surface_mm - start_mm[a]).signum();
                    l.touch(a, dir, surface_mm - dir * r)?;
                }
                l.position(at([start_mm[0], start_mm[1]], retract))?;
            }
            ProbeFeature::OutsideCorner {
                corner_mm: c,
                z_mm,
                material: m,
                inset_mm,
                clearance_mm,
            } => {
                below(*z_mm)?;
                let (sx, sy) = (sign(m[0])?, sign(m[1])?);
                let ys = [c[0] + sx * inset_mm, c[1] - sy * (r + clearance_mm)];
                let xs = [c[0] - sx * (r + clearance_mm), c[1] + sy * inset_mm];
                l.position(at(ys, *z_mm))?;
                l.touch(1, sy, c[1] - sy * r)?;
                l.position(at(ys, retract))?;
                l.position(at(xs, retract))?;
                l.position(at(xs, *z_mm))?;
                l.touch(0, sx, c[0] - sx * r)?;
                l.position(at(xs, retract))?;
            }
            ProbeFeature::InsideCorner {
                corner_mm: c,
                z_mm,
                material: m,
                ..
            } => {
                below(*z_mm)?;
                let (sx, sy) = (sign(m[0])?, sign(m[1])?);
                l.position([start[0], start[1], *z_mm])?;
                l.touch(1, sy, c[1] - sy * r)?;
                l.touch(0, sx, c[0] - sx * r)?;
                l.position(start)?;
            }
            ProbeFeature::Bore {
                center_mm: c,
                z_mm,
                diameter_mm,
            } => {
                below(*z_mm)?;
                let reach = diameter_mm / 2. - r;
                l.position(at(*c, *z_mm))?;
                for (axis, dir) in [(0, 1.), (0, -1.), (1, 1.), (1, -1.)] {
                    l.touch(axis, dir, c[axis] + dir * reach)?;
                }
                l.position(start)?;
            }
            ProbeFeature::Pocket {
                center_mm: c,
                z_mm,
                axis,
                width_mm,
            } => {
                below(*z_mm)?;
                let (a, reach) = (axis.index(), width_mm / 2. - r);
                l.position(at(*c, *z_mm))?;
                for dir in [1., -1.] {
                    l.touch(a, dir, c[a] + dir * reach)?;
                }
                l.position(start)?;
            }
            ProbeFeature::Boss {
                center_mm: c,
                z_mm,
                diameter_mm,
                clearance_mm,
            } => {
                below(*z_mm)?;
                let sides: [(usize, f64); 4] = [(0, 1.), (0, -1.), (1, 1.), (1, -1.)];
                self.outside(
                    &mut l,
                    *c,
                    *z_mm,
                    diameter_mm / 2. + r,
                    *clearance_mm,
                    &sides,
                )?;
            }
            ProbeFeature::Web {
                center_mm: c,
                z_mm,
                axis,
                width_mm,
                clearance_mm,
            } => {
                below(*z_mm)?;
                let a = axis.index();
                self.outside(
                    &mut l,
                    *c,
                    *z_mm,
                    width_mm / 2. + r,
                    *clearance_mm,
                    &[(a, 1.), (a, -1.)],
                )?;
            }
            ProbeFeature::ToolLength { contact_mm, .. } => {
                l.touch(2, -1., contact_mm[2])?;
            }
            ProbeFeature::ToolDiameter {
                setter_center_mm: c,
                setter_top_z_mm,
                setter_diameter_mm,
                clearance_mm,
                ..
            } => {
                let reach = setter_diameter_mm / 2. + r;
                let over = setter_top_z_mm + clearance_mm;
                let minus = [c[0] - reach - clearance_mm, c[1], start[2]];
                l.touch(0, -1., c[0] + reach)?;
                l.to(MoveKind::Rapid, [start[0], start[1], over])?;
                l.to(MoveKind::Rapid, [minus[0], minus[1], over])?;
                l.to(MoveKind::Rapid, minus)?;
                l.touch(0, 1., c[0] - reach)?;
            }
        }
        Ok(l.legs)
    }

    /// Touches from outside a boss or web: for each side, down from the
    /// retract height `reach + clearance` outside the centre, touch inward,
    /// and back up.
    fn outside(
        &self,
        l: &mut Layout,
        c: [f64; 2],
        z: f64,
        reach: f64,
        clearance: f64,
        sides: &[(usize, f64)],
    ) -> Result<(), String> {
        let retract = l.at[2];
        for (i, (axis, dir)) in sides.iter().enumerate() {
            let mut p = [c[0], c[1], retract];
            p[*axis] += dir * (reach + clearance);
            if i > 0 {
                l.position(p)?;
            }
            l.position([p[0], p[1], z])?;
            l.touch(*axis, -dir, c[*axis] + dir * reach)?;
            l.position(p)?;
        }
        Ok(())
    }
}

/// A probing or tool-setting operation: one cycle.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProbeOperation {
    pub id: String,
    /// A probe assembly (cutter shape `probe`) for stylus cycles; the
    /// measured cutting tool for tool setting.
    pub tool: String,
    pub tool_number: u32,
    pub length_register: u32,
    pub work_offset: u8,
    /// The plane rapids reach the cycle over; also the stylus cycles'
    /// protected retract height.
    pub clearance_z_mm: f64,
    pub feature: ProbeFeature,
    pub result: ProbeResult,
    pub search_mm: f64,
    /// Protected positioning feed (stylus cycles).
    #[serde(default)]
    pub position_feed_mm_min: Option<f64>,
    /// The measuring (guarded) move feed.
    pub measure_feed_mm_min: f64,
    /// The feed back from a contact.
    pub return_feed_mm_min: f64,
    /// Spindle during tool setting (0: stopped). Stylus cycles keep the
    /// spindle stopped.
    #[serde(default)]
    pub spindle_rpm: f64,
    #[serde(default)]
    pub spindle_direction: Option<SpindleDirection>,
    #[serde(default)]
    pub tolerance_mm: Option<f64>,
    #[serde(default)]
    pub limits: Option<crate::limits::Limits>,
}

#[derive(Debug, Serialize)]
pub struct ProbeReport {
    pub strategy: &'static str,
    /// Probing removes nothing: the residual is whatever it was.
    pub geometric_residual_status: crate::pocket::ResidualStatus,
    pub feature: &'static str,
    pub touches: usize,
    pub moves: usize,
    /// What the controller stores (a premise for what follows).
    pub stores: String,
    /// The stylus ball (or measured cutter) radius the cycle was laid out
    /// with: the controller's calibration must agree.
    pub radius_mm: f64,
    pub search_mm: f64,
}

pub struct ProbePlan {
    op: ProbeOperation,
    cycle: ProbeCycle,
    queue: std::collections::VecDeque<(MoveKind, Vec3, Option<f64>)>,
    /// Index into `queue` order where the cycle's covered moves start.
    cycle_first: usize,
    covers: u32,
    handed: usize,
    events: Vec<Event>,
    position: Vec3,
    initial: Vec3,
    next_id: u64,
    report: ProbeReport,
}

impl ProbePlan {
    pub fn prepare(
        op: ProbeOperation,
        tool: &ToolAssembly,
        stock: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let name = op.feature.name();
        let msg = |m: String| format!("probing {} ({name}): {m}", op.id);
        let fail = |m: String| Err(msg(m));
        if cancel.is_cancelled() {
            return Err("probe planning cancelled".into());
        }
        if !valid_id(&op.id) {
            return fail("id is not a valid identifier".into());
        }
        if op.tool != tool.id {
            return fail(format!(
                "tool {:?} does not match the assembly {:?}",
                op.tool, tool.id
            ));
        }
        let probe = matches!(tool.cutter.shape, CutterShape::Probe);
        let stylus = op.feature.stylus();
        if probe != stylus {
            return fail(if stylus {
                format!(
                    "{name} is measured with a probe; tool {} is not a probe (cutter shape \"probe\")",
                    tool.id
                )
            } else {
                format!(
                    "tool setting measures a cutting tool on the setter; {} is a probe",
                    tool.id
                )
            });
        }
        if !(1..=9).contains(&op.work_offset) || op.tool_number == 0 || op.length_register == 0 {
            return fail(
                "work_offset must be 1..9 and tool_number and length_register positive".into(),
            );
        }
        let feed_ok = |f: f64| f.is_finite() && f > 0. && f <= 1e6;
        if !feed_ok(op.measure_feed_mm_min)
            || !feed_ok(op.return_feed_mm_min)
            || (stylus && !op.position_feed_mm_min.is_some_and(feed_ok))
            || (!stylus && op.position_feed_mm_min.is_some())
        {
            return fail(
                "measure_feed_mm_min and return_feed_mm_min must be positive, and position_feed_mm_min given (positive) exactly for stylus cycles".into(),
            );
        }
        let tol = op.tolerance_mm.unwrap_or(1e-3);
        if !(1e-7..=0.01).contains(&tol) {
            return fail(format!("tolerance_mm {tol} must be within 1e-7..0.01"));
        }
        if stylus && op.spindle_rpm != 0. || op.spindle_direction.is_some() && op.spindle_rpm == 0.
        {
            return fail(
                "a probe's spindle stays stopped (spindle_rpm 0); a direction needs a speed".into(),
            );
        }
        if !(op.spindle_rpm.is_finite() && op.spindle_rpm >= 0.)
            || (op.spindle_rpm > 0.) != op.spindle_direction.is_some()
        {
            return fail("spindle_rpm must be 0, or positive with a spindle_direction".into());
        }
        if !(op.clearance_z_mm.is_finite() && op.clearance_z_mm > stock.max_mm[2])
            || op.clearance_z_mm.abs() > LIMIT_MM
        {
            return fail(format!(
                "clearance_z_mm {} must be above the stock top {}",
                op.clearance_z_mm, stock.max_mm[2]
            ));
        }
        if !initial.iter().all(|v| finite(*v)) || initial[2] < op.clearance_z_mm {
            return fail(format!(
                "start Z {} is below clearance_z_mm {}",
                initial[2], op.clearance_z_mm
            ));
        }
        if first_id == 0 || first_id.checked_add(64).is_none() {
            return fail("motion ID budget exhausted".into());
        }
        let cycle = ProbeCycle {
            feature: op.feature.clone(),
            retract_z_mm: stylus.then_some(op.clearance_z_mm),
            search_mm: op.search_mm,
            result: op.result.clone(),
            position_feed_mm_min: op.position_feed_mm_min,
            measure_feed_mm_min: op.measure_feed_mm_min,
            return_feed_mm_min: op.return_feed_mm_min,
        };
        cycle.validate().map_err(msg)?;
        if let ProbeResult::ToolLength { register } | ProbeResult::ToolRadius { register } =
            &op.result
            && *register != op.length_register
        {
            return fail(format!(
                "tool setting stores into the tool's own length_register {}, not {register}",
                op.length_register
            ));
        }
        let radius = tool.cutter.diameter_mm / 2.;
        let legs = cycle.legs(radius).map_err(msg)?;
        let start = cycle.start(radius)?;
        let mut queue = std::collections::VecDeque::new();
        let clearance = op.clearance_z_mm;
        // Over the clearance plane to the cycle's start.
        let mut at = initial;
        for p in [
            [start[0], start[1], initial[2]],
            [start[0], start[1], clearance],
            start,
        ] {
            if p != at {
                queue.push_back((MoveKind::Rapid, p, None));
                at = p;
            }
        }
        let cycle_first = queue.len();
        for leg in &legs {
            queue.push_back((leg.kind, leg.end, leg.feed_mm_min));
            at = leg.end;
        }
        if at[2] < clearance {
            queue.push_back((MoveKind::Rapid, [at[0], at[1], clearance], None));
        }
        let report = ProbeReport {
            geometric_residual_status: crate::pocket::ResidualStatus::Unknown,
            strategy: if stylus {
                "probe_protected_cycle"
            } else {
                "tool_setter_cycle"
            },
            feature: name,
            touches: legs
                .iter()
                .filter(|l| l.kind == MoveKind::ProbeTouch)
                .count(),
            moves: queue.len(),
            stores: match (&op.result, op.feature.measured_axes()) {
                (ProbeResult::WorkOffset { number }, axes) => format!(
                    "work offset {number} ({}) corrected by the measured {} error",
                    [
                        "G54", "G55", "G56", "G57", "G58", "G59", "G59.1", "G59.2", "G59.3"
                    ][*number as usize - 1],
                    axes.iter()
                        .map(|a| ["X", "Y", "Z"][a.index()])
                        .collect::<Vec<_>>()
                        .join("/")
                ),
                (ProbeResult::ToolLength { register }, _) => {
                    format!(
                        "the measured length of tool {} in length register {register}",
                        op.tool
                    )
                }
                (ProbeResult::ToolRadius { register }, _) => {
                    format!(
                        "the measured radius of tool {} in tool table entry {register}",
                        op.tool
                    )
                }
            },
            radius_mm: radius,
            search_mm: op.search_mm,
        };
        Ok(Self {
            covers: legs.len() as u32,
            cycle,
            cycle_first,
            queue,
            handed: 0,
            events: vec![],
            position: initial,
            initial,
            next_id: first_id,
            report,
            op,
        })
    }
    pub fn report(&self) -> &ProbeReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        let mut events = vec![
            Event::WorkOffset {
                number: self.op.work_offset,
            },
            Event::ToolChange {
                tool: self.op.tool.clone(),
                number: self.op.tool_number,
                length_register: self.op.length_register,
                position_after_mm: self.initial,
            },
        ];
        if let Some(direction) = self.op.spindle_direction {
            events.push(Event::Spindle {
                rpm: self.op.spindle_rpm,
                direction,
            });
        }
        events.push(Event::Coolant { mode: Coolant::Off });
        if self.cycle_first == 0 {
            // The cycle starts right where the tool change leaves the tip.
            events.push(self.probe_event());
        }
        events
    }
    fn probe_event(&self) -> Event {
        Event::Probe {
            cycle: self.cycle.clone(),
            covers: self.covers,
        }
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        if self.op.spindle_rpm > 0. {
            vec![Event::Spindle {
                rpm: 0.,
                direction: SpindleDirection::Cw,
            }]
        } else {
            vec![]
        }
    }
    pub fn next_motion_with_stock(
        &mut self,
        _stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if cancel.is_cancelled() {
            return Err("probe planning cancelled".into());
        }
        let Some((kind, end, feed)) = self.queue.pop_front() else {
            return Ok(None);
        };
        self.handed += 1;
        // Streams send an operation's events after the move just handed
        // out: the cycle's event goes with the move before its first one.
        if self.handed == self.cycle_first {
            self.events.push(self.probe_event());
        }
        let curve = Curve::Line {
            start: self.position,
            end,
        };
        curve.prepare(self.op.tolerance_mm.unwrap_or(1e-3))?;
        let motion = Motion {
            id: self.next_id,
            operation: self.op.id.clone(),
            tool: self.op.tool.clone(),
            kind,
            curve,
            feed_mm_min: feed,
        };
        self.position = end;
        self.next_id += 1;
        Ok(Some(motion))
    }
    /// The cycle's `Event::Probe`, due after the move before its first
    /// covered move.
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }
    pub fn finish_report(&mut self, _cancel: &impl CancelSignal) -> Result<(), String> {
        if !self.queue.is_empty() {
            return Err("probing cycle ended before all its moves".into());
        }
        Ok(())
    }
}
