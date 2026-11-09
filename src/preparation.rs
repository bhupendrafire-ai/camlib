//! Bounded preparation queries. Results are proposals, never release authority.
use crate::{
    flat::{Index, bounded_sections, enclosing_sections},
    motion::{Curve, Motion, MoveKind, Vec3},
    stock::{CancelSignal, HeightBounds, InitialStock, StockSpec},
    surface::{SurfaceMesh, segment_triangle_sq},
    tool::{CutterShape, ToolAssembly, valid_id},
    verification::{Options, Scene, Status, identity},
};
use serde::{Deserialize, Serialize};
use std::collections::HashSet;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BlankRequest {
    /// Already placed in setup coordinates, with top-down outer-solid semantics.
    pub surface: SurfaceMesh,
    pub stock: StockSpec,
    /// Metadata only on input; bounds must be empty to prevent accidental reuse.
    pub declaration: InitialStock,
    pub source_enclosure: String,
    pub tolerance_mm: f64,
    pub max_work: u64,
}

pub fn enclose_blank(
    request: &BlankRequest,
    cancel: &impl CancelSignal,
) -> Result<InitialStock, String> {
    let declaration = &request.declaration;
    if request.source_enclosure != "top_down_outer"
        || !declaration.bounds.is_empty()
        || !request.tolerance_mm.is_finite()
        || !(1e-7..=0.01).contains(&request.tolerance_mm)
        || !(1..=1_000_000_000).contains(&request.max_work)
    {
        return Err("blank enclosure needs empty bounds, top_down_outer premise, finite error and explicit work/tolerance limits".into());
    }
    // Validate the original placement error independently: a negative value
    // must never cancel positive mesh error and then be overwritten in output.
    declaration.validate_metadata()?;
    request
        .surface
        .validate_input(request.stock.min_mm[2], request.tolerance_mm)?;
    let e = declaration.uncertainty_mm + request.surface.surface_error_mm;
    if !e.is_finite() || !(0. ..=1.).contains(&e) {
        return Err("blank enclosure needs combined finite error in 0..=1 mm".into());
    }
    let (nx, ny, step) = request.stock.grid()?;
    let cells = nx
        .checked_mul(ny)
        .ok_or("blank enclosure cell count overflow")?;
    // Every string is now length-bounded by the shared stock validator.
    let metadata_bytes = (declaration.source_id.len()
        + declaration.revision.len()
        + declaration.source_sha256.len()
        + declaration.representation.len()) as u128;
    // The domain scan and all native preparation/query work share one budget.
    // Refuse before examining a large mesh when even this scan cannot fit.
    let mut work =
        (request.surface.triangles.len() as u64).saturating_add((cells as u64).saturating_mul(2)); // result allocation + final cell validation
    if work > request.max_work {
        return Err("blank enclosure work limit; no declaration committed".into());
    }
    let bounds_bytes = cells as u128 * std::mem::size_of::<HeightBounds>() as u128 + metadata_bytes;
    crate::resources::check_memory(bounds_bytes, "blank enclosure bounds")?;
    // No clipping: every facet and its uncertainty must fit the stock domain.
    for triangle in request.surface.triangles.iter() {
        if cancel.is_cancelled() {
            return Err("blank enclosure cancelled".into());
        }
        for p in triangle {
            if (0..3).any(|k| {
                !p[k].is_finite()
                    || p[k] - e < request.stock.min_mm[k]
                    || p[k] + e > request.stock.max_mm[k]
            }) {
                return Err(
                    "blank source/error lies outside declared stock; enlarge stock domain".into(),
                );
            }
        }
    }
    let prepared = request.surface.prepare_bounded(
        request.stock.min_mm[2],
        request.tolerance_mm,
        &mut work,
        request.max_work,
        cancel,
    )?;
    crate::resources::check_memory(
        prepared.allocation_bytes() + bounds_bytes,
        "blank enclosure bounds",
    )?;
    // Bounds are known empty; clone only validated, bounded metadata after
    // work and memory checks, never a caller-supplied cell collection.
    let mut result = declaration.clone();
    result
        .bounds
        .try_reserve_exact(cells)
        .map_err(|e| format!("blank enclosure bounds allocation failed: {e}"))?;
    for y in 0..ny {
        for x in 0..nx {
            if cancel.is_cancelled() {
                return Err("blank enclosure cancelled".into());
            }
            let xy = [
                request.stock.min_mm[0] + x as f64 * step[0],
                request.stock.min_mm[1] + y as f64 * step[1],
                request.stock.min_mm[0] + (x + 1) as f64 * step[0],
                request.stock.min_mm[1] + (y + 1) as f64 * step[1],
            ];
            let b = prepared
                .material_bounds_with_memory(
                    xy,
                    e + request.tolerance_mm,
                    &mut work,
                    request.max_work,
                    bounds_bytes,
                    cancel,
                )?
                .ok_or("blank enclosure work limit; no declaration committed")?;
            result.bounds.push(HeightBounds {
                lower_mm: if e == 0. {
                    b[0]
                } else {
                    request.stock.min_mm[2]
                },
                // The base outside the mesh remains exact; padding can only overstate stock.
                upper_mm: (b[1] + e).min(request.stock.max_mm[2]),
            });
        }
    }
    result.uncertainty_mm = e;
    result.validate(&request.stock)?;
    Ok(result)
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReachSample {
    pub id: String,
    pub point_mm: Vec3,
    pub normal: Vec3,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReachTool {
    pub family: String,
    pub assembly: ToolAssembly,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ReachRequest {
    pub surface: SurfaceMesh,
    pub base_z_mm: f64,
    pub samples: Vec<ReachSample>,
    pub tools: Vec<ReachTool>,
    pub scene: Scene,
    pub clearance_z_mm: f64,
    pub tolerance_mm: f64,
    pub max_pairs: usize,
    /// Index construction/traversal and facet-section work charged before allocation/query.
    /// Scene verification uses its own explicit options and work limits.
    pub max_work: u64,
    pub scene_options: Options,
}
#[derive(Clone, Debug, Serialize)]
pub struct ReachRow {
    pub sample_id: String,
    pub family: String,
    pub tool_id: String,
    pub stickout_mm: f64,
    pub gauge_length_mm: f64,
    pub tip_mm: Option<Vec3>,
    pub status: String,
    pub reasons: Vec<String>,
}
#[derive(Clone, Debug, Serialize)]
pub struct ReachSelection {
    pub sample_id: String,
    pub family: String,
    pub tool_id: String,
}
#[derive(Clone, Debug, Serialize)]
pub struct ReachReport {
    pub input_sha256: String,
    pub complete: bool,
    pub rows: Vec<ReachRow>,
    pub selected: Vec<ReachSelection>,
    pub scope: String,
}

pub fn reach(request: &ReachRequest, cancel: &impl CancelSignal) -> Result<ReachReport, String> {
    if request.samples.is_empty()
        || request.tools.is_empty()
        || request
            .samples
            .len()
            .checked_mul(request.tools.len())
            .is_none_or(|n| n > request.max_pairs)
        || request.max_pairs > 1_000_000
        || request.max_work == 0
        || request.max_work > 1_000_000_000
        || !request.base_z_mm.is_finite()
        || request.base_z_mm.abs() > 1e6
        || !request.clearance_z_mm.is_finite()
        || request.clearance_z_mm.abs() > 1e6
        || !request.tolerance_mm.is_finite()
        || !(1e-7..=0.1).contains(&request.tolerance_mm)
        || request.surface.surface_error_mm >= request.tolerance_mm
    {
        return Err("reach query needs finite samples/tools, positive pair/work budgets and tolerance exceeding source error".into());
    }
    let input_sha256 = identity(request)?;
    let mut ids = HashSet::new();
    for sample in &request.samples {
        if !valid_id(&sample.id)
            || !ids.insert(&sample.id)
            || sample
                .point_mm
                .iter()
                .any(|v| !v.is_finite() || v.abs() > 1e6)
            || sample.normal.iter().any(|v| !v.is_finite())
            || (sample.normal.iter().map(|v| v * v).sum::<f64>() - 1.).abs() > 1e-10
            || sample.normal[2] < 0.
        {
            return Err(
                "reach samples need unique IDs, bounded points and upward unit normals".into(),
            );
        }
    }
    let scene = request.scene.prepare(request.scene_options)?;
    let mut work = 0u64;
    let index = Index::new_bounded(
        &request.surface,
        request.base_z_mm,
        1.,
        request.tolerance_mm,
        &mut work,
        request.max_work,
        cancel,
    )?;
    let mut tools = Vec::new();
    let mut interrupted = index.is_none();
    ids.clear();
    for t in &request.tools {
        if cancel.is_cancelled() {
            return Err("reach tool preparation cancelled".into());
        }
        if !valid_id(&t.family) || !ids.insert(&t.assembly.id) {
            return Err("reach tool families/IDs must be valid and assembly IDs unique".into());
        }
        let prepared = t.assembly.prepare()?;
        let r = t.assembly.cutter.diameter_mm / 2.;
        let rc = match t.assembly.cutter.shape {
            CutterShape::Flat => 0.,
            CutterShape::Ball => r,
            CutterShape::Bull { corner_radius_mm } => corner_radius_mm,
            _ => return Err("reach supports flat, ball and bull cutters".into()),
        };
        let height_step = (request.tolerance_mm - request.surface.surface_error_mm) / 8.;
        let curved_count = (rc / height_step).ceil();
        // Charge both section constructions and the curved-band/filter passes
        // before their allocations. Assembly validation remains indivisible.
        let preparation_cost = (t.assembly.holders.len() as u64 + 3)
            .saturating_mul(3)
            .saturating_add(if curved_count <= 4096. {
                2 * curved_count as u64
            } else {
                0
            });
        if interrupted || preparation_cost > request.max_work.saturating_sub(work) {
            interrupted = true;
            tools.push((t, prepared, r, rc, vec![], None));
            continue;
        }
        work += preparation_cost;
        let sections = enclosing_sections(&prepared)?;
        let cutter = bounded_sections(&prepared, height_step).ok().map(|v| {
            v.into_iter()
                .filter(|s| s.z0 < t.assembly.cutter.cutting_length_mm)
                .collect::<Vec<_>>()
        });
        tools.push((t, prepared, r, rc, sections, cutter));
    }
    tools.sort_by(|a, b| {
        a.0.assembly
            .stickout_mm
            .total_cmp(&b.0.assembly.stickout_mm)
            .then_with(|| a.0.assembly.id.cmp(&b.0.assembly.id))
    });
    let mut seen = vec![0; index.as_ref().map_or(0, Index::len)];
    let mut stamp = 0;
    let mut rows = Vec::new();
    let mut selected = Vec::new();
    for sample in &request.samples {
        let witness_cost = request.surface.triangles.len() as u64;
        let mut witness = false;
        if !interrupted && witness_cost <= request.max_work.saturating_sub(work) {
            work += witness_cost;
            for &triangle in request.surface.triangles.iter() {
                if cancel.is_cancelled() {
                    interrupted = true;
                    break;
                }
                if segment_triangle_sq(sample.point_mm, sample.point_mm, triangle).sqrt()
                    <= request.surface.surface_error_mm + request.tolerance_mm * 0.01
                {
                    witness = true;
                    break;
                }
            }
        } else {
            interrupted = true;
        }
        for (t, prepared, r, rc, sections, cutter) in &tools {
            let mut row = ReachRow {
                sample_id: sample.id.clone(),
                family: t.family.clone(),
                tool_id: t.assembly.id.clone(),
                stickout_mm: t.assembly.stickout_mm,
                gauge_length_mm: t.assembly.gauge_length_mm,
                tip_mm: None,
                status: "unresolved".into(),
                reasons: Vec::new(),
            };
            let cost = index.as_ref().map_or(u64::MAX, |index| {
                index
                    .section_work(cutter.as_ref().map_or(0, Vec::len))
                    .saturating_add(index.section_work(sections.len().saturating_sub(1)))
            });
            if interrupted || cancel.is_cancelled() || cost > request.max_work.saturating_sub(work)
            {
                interrupted = true;
                row.reasons.push("cancelled_or_work_limit".into());
            } else if !witness {
                row.reasons.push("source_point_not_witnessed".into());
            } else {
                work += cost;
                let index = index.as_ref().expect("prepared index when not interrupted");
                let n = sample.normal;
                let s = n[0].hypot(n[1]);
                let shift = if s > 0. { (*r - *rc + *rc * s) / s } else { 0. };
                let p = [
                    sample.point_mm[0] + shift * n[0],
                    sample.point_mm[1] + shift * n[1],
                    sample.point_mm[2] + rc * n[2] - rc,
                ];
                row.tip_mm = Some(p);
                let guard = request.surface.surface_error_mm
                    + 1024. * f64::EPSILON * (1. + p.iter().map(|v| v.abs()).sum::<f64>());
                // Once a pair starts, cooperative interruption must preserve its
                // unresolved row and every remaining pair in the same report.
                let checked = (|| -> Result<(), String> {
                    if request.clearance_z_mm <= p[2] {
                        row.reasons.push("clearance_not_above_pose".into());
                    }
                    if let Some(cutter) = cutter {
                        if index.max_sections_cancel(
                            [p[0], p[1]],
                            cutter,
                            &mut seen,
                            &mut stamp,
                            cancel,
                        )? + guard
                            > p[2] + request.tolerance_mm
                        {
                            row.reasons.push("cutter_clearance_unproven".into());
                        }
                    } else {
                        row.reasons.push("cutter_clearance_unproven".into());
                    }
                    if index.max_sections_cancel(
                        [p[0], p[1]],
                        &sections[1..],
                        &mut seen,
                        &mut stamp,
                        cancel,
                    )? + guard
                        > p[2] + request.tolerance_mm
                    {
                        row.reasons
                            .push("neck_shank_holder_clearance_unproven".into());
                    }
                    if request.clearance_z_mm > p[2] {
                        let motion = Motion {
                            id: 1,
                            operation: "reach-approach".into(),
                            tool: t.assembly.id.clone(),
                            kind: MoveKind::Rapid,
                            curve: Curve::Line {
                                start: [p[0], p[1], request.clearance_z_mm],
                                end: p,
                            },
                            feed_mm_min: None,
                        };
                        let check = scene.verify_motion(prepared, &motion, cancel)?;
                        match check.status {
                            Status::Clear => {}
                            Status::Violation => {
                                row.status = "obstructed".into();
                                row.reasons
                                    .push(format!("approach_violation:{:?}", check.findings));
                            }
                            Status::Uncertain => {
                                row.reasons.push("approach_clearance_unproven".into())
                            }
                        }
                    }
                    Ok(())
                })();
                match checked {
                    Err(error) if cancel.is_cancelled() || error.contains("cancelled") => {
                        interrupted = true;
                        row.status = "unresolved".into();
                        row.reasons.push("cancelled_or_work_limit".into());
                    }
                    Err(error) => return Err(error),
                    Ok(()) if row.reasons.is_empty() => row.status = "clear".into(),
                    Ok(()) => {}
                }
            }
            if row.status == "clear"
                && !selected.iter().any(|s: &ReachSelection| {
                    s.sample_id == row.sample_id && s.family == row.family
                })
            {
                selected.push(ReachSelection {
                    sample_id: row.sample_id.clone(),
                    family: row.family.clone(),
                    tool_id: row.tool_id.clone(),
                });
            }
            rows.push(row);
        }
    }
    let complete = !interrupted;
    if !complete {
        selected.clear();
    }
    Ok(ReachReport{input_sha256,complete,rows,selected,scope:"Supplied sample poses and vertical fixture approaches only; smallest clear caller-calibrated TESTED projection per family. No continuous minimum, complete-surface, incoming-stock, physical machine or release certificate.".into()})
}
