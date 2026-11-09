//! Machining limits for any toolpath: an optional containment region the
//! whole cutter must stay inside, and keep-out areas (clamps, fixtures,
//! features to leave alone) the cutter must not enter while its tip is below
//! the area's top. Region operations machine the restricted region; path
//! operations keep only the parts of their moves that are proven allowed and
//! refuse moves they cannot leave out.
use crate::{
    Point,
    motion::Curve,
    planar::{Boolean, ConstructionOptions, PlanarRegion, PreparedRegion},
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Limits {
    /// The whole cutter stays inside this region.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contain: Option<PlanarRegion>,
    /// The cutter stays out of each area while its tip is below the top.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub avoid: Vec<AvoidArea>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct AvoidArea {
    pub region: PlanarRegion,
    pub top_z_mm: f64,
}
impl Limits {
    pub fn is_empty(&self) -> bool {
        self.contain.is_none() && self.avoid.is_empty()
    }
    pub(crate) fn validate(&self) -> Result<(), String> {
        if self.avoid.len() > 256
            || self
                .avoid
                .iter()
                .any(|a| !a.top_z_mm.is_finite() || a.top_z_mm.abs() > 1e6)
        {
            return Err("limits: at most 256 avoid areas, each with a finite top_z_mm".into());
        }
        Ok(())
    }
    /// The part of `region` (whose cutter edge stays inside it) that the
    /// limits allow at depths down to `floor_z`: inside the containment, outside
    /// every avoid area whose top is above the floor.
    pub fn restrict_region(
        &self,
        region: &PlanarRegion,
        floor_z: f64,
        tolerance_mm: f64,
    ) -> Result<PlanarRegion, String> {
        self.validate()?;
        let options = ConstructionOptions { tolerance_mm };
        // A clearance covering each area's boundary uncertainty and the
        // planner's tolerance, plus REGION_CLEARANCE, so planned cuts are
        // provably clear at the checks' sample spacing, not tangent.
        let clearance = |r: &PlanarRegion| {
            2. * (r.boundary_error_mm + tolerance_mm) + REGION_CLEARANCE.max(32. * tolerance_mm)
        };
        let mut current = region.prepare(options)?;
        if let Some(contain) = &self.contain {
            let shrunk = contain
                .prepare(options)?
                .offset_prepared(clearance(contain))?; // positive erodes
            current = current
                .boolean_prepared(&shrunk.into_parts().1, Boolean::Intersection)?
                .into_parts()
                .1;
        }
        for area in self.avoid.iter().filter(|a| a.top_z_mm > floor_z) {
            let grown = area
                .region
                .prepare(options)?
                .offset_prepared(-clearance(&area.region))?; // negative grows
            current = current
                .boolean_prepared(&grown.into_parts().1, Boolean::Difference)?
                .into_parts()
                .1;
        }
        let result = current.definition().clone();
        if result.components.is_empty() {
            return Err(
                "limits leave nothing of the region to machine (containment/avoid areas cover it)"
                    .into(),
            );
        }
        Ok(result)
    }
}

pub(crate) struct PreparedLimits {
    contain: Option<PreparedRegion>,
    avoid: Vec<(PreparedRegion, f64)>,
    radius: f64,
    tolerance: f64,
}
/// Smallest sample spacing when proving a move inside the limits.
const MIN_STEP: f64 = 0.005;
/// Extra clearance region operations keep from avoid areas and containment
/// edges (four smallest sample spacings, so it can be proven).
pub const REGION_CLEARANCE: f64 = 0.02;
impl PreparedLimits {
    pub(crate) fn new(limits: &Limits, radius: f64, tolerance_mm: f64) -> Result<Self, String> {
        limits.validate()?;
        let options = ConstructionOptions { tolerance_mm };
        Ok(Self {
            contain: limits
                .contain
                .as_ref()
                .map(|r| r.prepare(options))
                .transpose()?,
            avoid: limits
                .avoid
                .iter()
                .map(|a| Ok((a.region.prepare(options)?, a.top_z_mm)))
                .collect::<Result<_, String>>()?,
            radius,
            tolerance: tolerance_mm,
        })
    }
    /// Signed margin of the cutter disk at `xy` with its tip at (or reaching
    /// down to) `tip_z`: positive when allowed by at least that much. The
    /// containment limits cutting moves; avoid areas limit every move.
    pub(crate) fn margin(&self, xy: Point, tip_z: f64, cutting: bool) -> Result<f64, String> {
        let mut margin = f64::INFINITY;
        if let Some(contain) = self.contain.as_ref().filter(|_| cutting) {
            let q = contain.query(xy)?;
            let signed = q.signed_clearance_mm.unwrap_or(f64::NEG_INFINITY);
            margin = margin.min(signed - self.radius - q.uncertainty_mm - self.tolerance);
        }
        for (area, top) in &self.avoid {
            if tip_z >= *top {
                continue;
            }
            let q = area.query(xy)?;
            // Inside the area is positive clearance: the cutter must be out.
            let signed = q.signed_clearance_mm.unwrap_or(f64::INFINITY);
            margin = margin.min(-signed - self.radius - q.uncertainty_mm - self.tolerance);
        }
        Ok(margin)
    }
    pub(crate) fn allows(&self, xy: Point, tip_z: f64) -> Result<bool, String> {
        Ok(self.margin(xy, tip_z, true)? >= 0.)
    }
    /// The highest avoid area top (where moving over the areas is allowed).
    pub(crate) fn avoid_top(&self) -> Option<f64> {
        self.avoid.iter().map(|a| a.1).reduce(f64::max)
    }
    /// The parameter intervals of a move proven allowed (each margin covers
    /// the distance to the next sample; anything unproven is left out).
    pub(crate) fn allowed_spans(
        &self,
        curve: &Curve,
        cutting: bool,
    ) -> Result<Vec<[f64; 2]>, String> {
        let prepared = curve.prepare(self.tolerance)?;
        let length = prepared.length_mm();
        let low_z = |t0: f64, t1: f64| -> Result<f64, String> {
            Ok(prepared.position(t0)?[2].min(prepared.position(t1)?[2]))
        };
        let mut spans: Vec<[f64; 2]> = vec![];
        let mut stack = vec![[0., 1.]];
        let mut steps = 0u64;
        while let Some([t0, t1]) = stack.pop() {
            steps += 1;
            if steps > 4_000_000 {
                return Err("limits: move too long to prove at the minimum sample spacing".into());
            }
            let mid = (t0 + t1) / 2.;
            let p = prepared.position(mid)?;
            let half = length * (t1 - t0) / 2.;
            let margin = self.margin([p[0], p[1]], low_z(t0, t1)?, cutting)?;
            if margin >= half {
                match spans.last_mut() {
                    Some(last) if last[1] == t0 => last[1] = t1,
                    _ => spans.push([t0, t1]),
                }
            } else if half > MIN_STEP && margin > -half {
                // Unresolved: split (later half first, so intervals pop in order).
                stack.push([mid, t1]);
                stack.push([t0, mid]);
            }
        }
        Ok(spans)
    }
    /// Whether the whole move is proven allowed.
    pub(crate) fn allows_curve(&self, curve: &Curve, cutting: bool) -> Result<bool, String> {
        Ok(self.allowed_spans(curve, cutting)?.as_slice() == [[0., 1.]])
    }
}

/// Refuse holes whose whole footprint (radius around each center, reaching
/// down to `bottom_z`) is not proven inside the limits.
pub(crate) fn check_holes(
    limits: Option<Limits>,
    centers: &[Point],
    radius: f64,
    bottom_z: f64,
    tolerance_mm: f64,
    what: &str,
) -> Result<(), String> {
    let Some(limits) = limits.filter(|l| !l.is_empty()) else {
        return Ok(());
    };
    let prepared = PreparedLimits::new(&limits, radius, tolerance_mm)?;
    for (i, c) in centers.iter().enumerate() {
        if !prepared.allows(*c, bottom_z)? {
            return Err(format!(
                "{what} hole {i} at ({:.4}, {:.4}) reaches into an avoid area or out of the containment (footprint radius {radius:.4}); remove the hole or change the limits",
                c[0], c[1]
            ));
        }
    }
    Ok(())
}

/// Every rapid and link of an operation run stays out of the avoid areas
/// below their tops (cutting moves are kept inside the limits by each plan).
pub struct LinkGuard {
    limits: PreparedLimits,
}
impl LinkGuard {
    pub fn new(
        limits: Option<&Limits>,
        tool: &crate::tool::ToolAssembly,
        tolerance_mm: f64,
    ) -> Result<Option<Self>, String> {
        limits
            .filter(|l| !l.avoid.is_empty())
            .map(|l| {
                Ok(Self {
                    limits: PreparedLimits::new(l, tool.cutter.diameter_mm / 2., tolerance_mm)?,
                })
            })
            .transpose()
    }
    pub fn check(&self, motion: &crate::motion::Motion) -> Result<(), String> {
        use crate::motion::MoveKind;
        // An oriented-spindle shift stays inside the bore its own plunge cut
        // (stock-contact certificate), which was itself checked.
        // Probing moves too: the stylus stays out of the avoid areas.
        if !(matches!(motion.kind, MoveKind::Rapid | MoveKind::Link) || motion.kind.is_probe())
            || matches!(motion.curve, Curve::Line { start, end } if start == end)
        {
            return Ok(());
        }
        if self.limits.allows_curve(&motion.curve, false)? {
            return Ok(());
        }
        Err(format!(
            "operation {}: a {:?} move from {:?} to {:?} passes through an avoid area below its top; raise the clearance/retract heights above it",
            motion.operation,
            motion.kind,
            motion.curve.start(),
            motion.curve.end()
        ))
    }
}
