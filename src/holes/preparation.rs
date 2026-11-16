//! Declared spot/pilot dependencies; ordinary motion and stock checks retain authority.
//! Spot-angle practice: ISCAR Drilling Handbook, printed p.186 (spot drilling).
use super::{DepthReference, DrillingOperation};
use crate::{pocket::Operation, stock::CancelSignal, stream::StreamManifest, tool::CutterShape};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DrillingPreparation {
    pub spot: Option<String>,
    pub spot_diameter_mm: Option<f64>,
    pub pilot: Option<String>,
}
#[derive(Clone, Debug, Serialize)]
pub struct SpotReport {
    pub operation: String,
    pub requested_diameter_mm: f64,
    pub diameter_mm: f64,
    pub point_angle_deg: f64,
    pub tip_depth_mm: f64,
    pub chamfer_radial_width_mm: f64,
    pub chamfer_axial_depth_mm: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct PilotReport {
    pub operation: String,
    pub diameter_mm: f64,
    pub full_diameter_depth_mm: f64,
    pub tip_z_mm: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct PreparationReport {
    pub spot: Option<SpotReport>,
    pub pilot: Option<PilotReport>,
    pub drill_diameter_mm: f64,
    pub drill_point_angle_deg: f64,
    pub holes: usize,
    pub scope: &'static str,
}
struct Cone {
    radius: f64,
    slope: f64,
    angle: f64,
    tip: f64,
}
fn cone(m: &StreamManifest, op: &DrillingOperation) -> Result<Cone, String> {
    let t = m
        .tools
        .iter()
        .find(|t| t.id == op.tool)
        .ok_or("preparation tool missing")?;
    t.prepare()?;
    let CutterShape::Drill { point_angle_rad } = t.cutter.shape else {
        return Err(format!(
            "drilling preparation {} requires an explicitly declared conical drill point",
            op.id
        ));
    };
    let radius = t.cutter.diameter_mm / 2.;
    let slope = (point_angle_rad / 2.).tan();
    let tip = op.depth_z_mm
        - if op.depth_reference == DepthReference::FullDiameter {
            radius / slope
        } else {
            0.
        };
    if !op.tolerance_mm.is_finite()
        || op.tolerance_mm <= 0.
        || !tip.is_finite()
        || tip >= m.stock.view()?.max_mm[2]
    {
        return Err(format!(
            "drilling preparation {} needs a finite target below the stock top and a positive tolerance",
            op.id
        ));
    }
    Ok(Cone {
        radius,
        slope,
        angle: point_angle_rad.to_degrees(),
        tip,
    })
}
fn covers(
    a: &DrillingOperation,
    b: &DrillingOperation,
    cancel: &impl CancelSignal,
) -> Result<(), String> {
    if a.centers_mm.len() > 1_000_000 || b.centers_mm.len() > 1_000_000 || b.centers_mm.is_empty() {
        return Err(
            "drilling preparation needs 1..1000000 final centers and bounded prerequisite centers"
                .into(),
        );
    }
    let mut counts = HashMap::new();
    for (stage, points) in [(0, &a.centers_mm), (1, &b.centers_mm)] {
        for (i, c) in points.iter().enumerate() {
            if i % 256 == 0 && cancel.is_cancelled() {
                return Err("drilling preparation cancelled; no result committed".into());
            }
            if !c.iter().all(|x| x.is_finite() && x.abs() <= 1e6) {
                return Err("drilling preparation centers must be finite and bounded".into());
            }
            let key = c.map(|x| if x == 0. { 0 } else { x.to_bits() });
            let count = counts.entry(key).or_insert(0usize);
            if stage == 0 {
                *count += 1;
            } else if *count == 0 {
                return Err(format!(
                    "preparation {} does not cover every center of {} (including duplicate occurrences)",
                    a.id, b.id
                ));
            } else {
                *count -= 1;
            }
        }
    }
    Ok(())
}
/// Validate source-bound prerequisites. This is nominal geometry, never a clearance certificate.
pub fn preparation_report(
    m: &StreamManifest,
    index: usize,
    cancel: &impl CancelSignal,
) -> Result<Option<PreparationReport>, String> {
    let Some(Operation::Drilling(drill)) = m.operations.get(index) else {
        return Ok(None);
    };
    let Some(p) = &drill.preparation else {
        return Ok(None);
    };
    if m.version != 1
        || p.spot.is_none() && p.pilot.is_none()
        || p.spot.is_some() != p.spot_diameter_mm.is_some()
    {
        return Err("drilling preparation needs a version 1 operation run, a spot and/or pilot ID, and spot_diameter_mm exactly when a spot is named".into());
    }
    let get = |id: &str| -> Result<(usize, &DrillingOperation), String> {
        let (i, op) = m
            .operations
            .iter()
            .enumerate()
            .find(|(_, o)| o.id() == id)
            .ok_or_else(|| format!("drilling preparation {id} is missing"))?;
        let Operation::Drilling(op) = op else {
            return Err(format!("preparation {id} must be a drilling operation"));
        };
        if i >= index || op.work_offset != drill.work_offset {
            return Err(format!(
                "preparation {id} must precede {} in the same work frame",
                drill.id
            ));
        }
        covers(op, drill, cancel)?;
        Ok((i, op))
    };
    let d = cone(m, drill)?;
    let top = m.stock.view()?.max_mm[2];
    let mut first_angle = d.angle;
    let mut first_tip = d.tip;
    let mut pilot_index = None;
    let pilot = p.pilot.as_ref().map(|id| {
        let (i, op) = get(id)?;
        pilot_index = Some(i);
        let c = cone(m, op)?;
        let tol = drill.tolerance_mm.max(op.tolerance_mm);
        let shoulder = c.tip + c.radius / c.slope;
        if c.radius + 16. * tol >= d.radius || shoulder >= top - 16. * tol
            || c.tip + tol < d.tip || shoulder + tol < d.tip + c.radius / d.slope {
            return Err(format!("pilot {id} must have a smaller diameter, reach full diameter below the top, and stay inside the final drill's depth envelope"));
        }
        first_angle = first_angle.max(c.angle);
        first_tip = first_tip.max(c.tip);
        Ok(PilotReport { operation: id.clone(), diameter_mm: 2. * c.radius, full_diameter_depth_mm: top - shoulder, tip_z_mm: c.tip })
    }).transpose()?;
    let spot = p.spot.as_ref().map(|id| {
        let (i, op) = get(id)?;
        if pilot_index.is_some_and(|j| i >= j) { return Err("drilling preparation order must be spot, pilot, final drill".into()); }
        let c = cone(m, op)?;
        let requested = p.spot_diameter_mm.unwrap();
        let depth = top - c.tip;
        let diameter = 2. * depth * c.slope;
        if !requested.is_finite() || requested <= 16. * op.tolerance_mm || depth > c.radius / c.slope
            || (diameter - requested).abs() > op.tolerance_mm || c.tip < first_tip - drill.tolerance_mm {
            return Err(format!("spot {id} must stay on its conical point, match requested spot_diameter_mm within its tolerance, and not pass the final drill tip depth"));
        }
        // A sharper spot makes the following drill's outer edges engage first.
        // https://www.iscar.com/Catalogs/Publication/english_1/Drilling_Handbook/Drilling_Handbook_flipview/files/basic-html/page188.html
        if c.angle < first_angle { return Err(format!("spot {id} point angle must be at least every following drill point angle; select a compatible declared spot tool")); }
        Ok(SpotReport { operation: id.clone(), requested_diameter_mm: requested, diameter_mm: diameter, point_angle_deg: c.angle,
            tip_depth_mm: depth, chamfer_radial_width_mm: ((diameter - 2. * d.radius) / 2.).max(0.),
            chamfer_axial_depth_mm: (depth - d.radius / c.slope).max(0.) })
    }).transpose()?;
    Ok(Some(PreparationReport {
        spot,
        pilot,
        drill_diameter_mm: 2. * d.radius,
        drill_point_angle_deg: d.angle,
        holes: drill.centers_mm.len(),
        scope: "source-bound order and nominal conical geometry; ordinary stock, scene, material and posted-output checks remain authoritative; no rapid-entry credit",
    }))
}
