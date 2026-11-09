//! Undercut milling (issue #8): a T-slot, keyseat (Woodruff), lollipop or
//! dovetail cutter follows tool-centre paths at one depth under material,
//! going down to that depth and back up through open space.
//!
//! Established practice (T-slot and keyseat cutters: Machinery's Handbook,
//! "Milling cutters"; dovetail and lollipop cutters: cutter makers'
//! application guides, e.g. Harvey Tool): the slot or pocket above is
//! machined first, then the undercut cutter enters at full depth from open
//! space (outside the part, or a cleared pocket), cuts sideways and leaves
//! the same way. It never plunges into material: its neck is narrower than
//! its cutting part.
//!
//! The planner proves on the stock it reads (the run's height view) that the
//! columns where the tool goes down and up are open for the whole assembly
//! to the cut depth, and refuses otherwise. Moves at depth go under
//! material; the tri-dexel stock sweep decides them (refusing a move whose
//! neck, shank or holder may touch material), so undercut operations need
//! tri-dexel stock (`StreamManifest::validate`).
use crate::{
    motion::{Coolant, Curve, Event, Motion, MoveKind, Plane, Vec3},
    pocket::ResidualStatus,
    stock::{CancelSignal, Stock, StockSpec},
    tool::{ToolAssembly, valid_id},
    trace::{TracePath, TraceSegment},
};
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct UndercutOperation {
    pub id: String,
    pub tool: String,
    /// Tool-centre paths at the cut depth (lines and arcs, open or closed).
    pub paths: Vec<TracePath>,
    /// Tool tip Z of the cut.
    pub tip_z_mm: f64,
    /// Where the tool goes down to the cut depth: an XY point whose column
    /// is open for the whole assembly (outside the part for a T-slot, the
    /// cleared pocket for a dovetail). It moves at depth from there to each
    /// path's start.
    pub entry_mm: [f64; 2],
    /// Where it goes back up. None: back along each open path to its start
    /// and to the entry (the way it came in).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exit_mm: Option<[f64; 2]>,
    pub clearance_z_mm: f64,
    pub feed_mm_min: f64,
    pub plunge_mm_min: f64,
    pub tolerance_mm: f64,
    pub spindle_rpm: f64,
    pub work_offset: u8,
    pub tool_number: u32,
    pub length_register: u32,
    pub coolant: Coolant,
}

#[derive(Debug, Serialize)]
pub struct UndercutReport {
    pub strategy: &'static str,
    pub geometric_residual_status: ResidualStatus,
    pub paths: usize,
    pub segments: usize,
    pub tip_z_mm: f64,
    /// The cutting part's height band at the cut depth.
    pub cutting_band_z_mm: [f64; 2],
    pub entry_mm: [f64; 2],
    pub exit_mm: Option<[f64; 2]>,
    pub cut_segments_emitted: usize,
    pub machining_scope: &'static str,
}

pub struct UndercutPlan {
    op: UndercutOperation,
    initial: Vec3,
    /// The assembly's parts: (radius, height above the tip where it starts).
    envelope: Vec<(f64, f64)>,
    pending: VecDeque<(Motion, Option<[f64; 2]>)>,
    report: UndercutReport,
    done: bool,
}

/// The tool-centre curves of a path at height `z`, and whether it closes.
fn curves(path: &TracePath, z: f64, guard: f64) -> Result<(Vec<Curve>, bool), String> {
    let mut at = path.start_mm;
    let mut out = vec![];
    if path.segments.is_empty() || !path.start_mm.iter().all(|v| v.is_finite()) {
        return Err("a path needs a finite start and at least one segment".into());
    }
    for (i, s) in path.segments.iter().enumerate() {
        let (curve, end) = match *s {
            TraceSegment::Line { end_mm } => {
                if !end_mm.iter().all(|v| v.is_finite())
                    || (end_mm[0] - at[0]).hypot(end_mm[1] - at[1]) <= guard
                {
                    return Err(format!("segment {i}: a line needs a finite, distinct end"));
                }
                (
                    Curve::Line {
                        start: [at[0], at[1], z],
                        end: [end_mm[0], end_mm[1], z],
                    },
                    end_mm,
                )
            }
            TraceSegment::Arc {
                center_mm,
                sweep_rad,
            } => {
                let r = (at[0] - center_mm[0]).hypot(at[1] - center_mm[1]);
                if !center_mm.iter().all(|v| v.is_finite())
                    || !sweep_rad.is_finite()
                    || sweep_rad == 0.
                    || sweep_rad.abs() > std::f64::consts::TAU
                    || r <= guard
                {
                    return Err(format!(
                        "segment {i}: an arc needs a finite center away from its start and a sweep within (0, 2*pi]"
                    ));
                }
                let a = (at[1] - center_mm[1]).atan2(at[0] - center_mm[0]) + sweep_rad;
                let end = [center_mm[0] + r * a.cos(), center_mm[1] + r * a.sin()];
                (
                    Curve::Arc {
                        start: [at[0], at[1], z],
                        end: [end[0], end[1], z],
                        center: [center_mm[0], center_mm[1], z],
                        plane: Plane::Xy,
                        sweep_rad,
                    },
                    end,
                )
            }
        };
        out.push(curve);
        at = end;
    }
    let closed = (at[0] - path.start_mm[0]).hypot(at[1] - path.start_mm[1]) <= guard;
    if closed {
        // Close exactly onto the start, so the loop has no gap.
        match out.last_mut().unwrap() {
            Curve::Line { end, .. } | Curve::Arc { end, .. } | Curve::Helix { end, .. } => {
                *end = [path.start_mm[0], path.start_mm[1], z]
            }
        }
    }
    Ok((out, closed))
}
fn reversed(c: &Curve) -> Curve {
    match *c {
        Curve::Line { start, end } => Curve::Line {
            start: end,
            end: start,
        },
        Curve::Arc {
            start,
            end,
            center,
            plane,
            sweep_rad,
        } => Curve::Arc {
            start: end,
            end: start,
            center,
            plane,
            sweep_rad: -sweep_rad,
        },
        Curve::Helix {
            start,
            end,
            center,
            plane,
            sweep_rad,
        } => Curve::Helix {
            start: end,
            end: start,
            center,
            plane,
            sweep_rad: -sweep_rad,
        },
    }
}

type Step = (MoveKind, Curve, Option<f64>, Option<[f64; 2]>);

impl UndercutPlan {
    pub fn prepare(
        op: UndercutOperation,
        assembly: &ToolAssembly,
        spec: &StockSpec,
        initial: Vec3,
        first_id: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        if cancel.is_cancelled() {
            return Err("undercut planning cancelled".into());
        }
        spec.grid()?;
        let tool = assembly.prepare()?;
        let cutter = tool.cutter().definition();
        let guard = 16. * op.tolerance_mm;
        let values = [
            op.tip_z_mm,
            op.clearance_z_mm,
            op.feed_mm_min,
            op.plunge_mm_min,
            op.tolerance_mm,
            op.spindle_rpm,
            op.entry_mm[0],
            op.entry_mm[1],
        ];
        crate::heights::require(
            "undercut",
            [
                (
                    valid_id(&op.id),
                    format!("id {:?} is not a valid identifier", op.id),
                ),
                (
                    op.tool == assembly.id,
                    format!(
                        "tool {:?} does not match the assembly {:?}",
                        op.tool, assembly.id
                    ),
                ),
                (
                    cutter.shape.undercuts()
                        || cutter
                            .neck
                            .as_ref()
                            .is_some_and(|n| n.diameter_mm < cutter.diameter_mm),
                    "undercut milling needs an undercut cutter: a T-slot or keyseat cutter (a cutting disk on a narrower neck), a lollipop or a dovetail"
                        .into(),
                ),
                (
                    values.iter().all(|v| v.is_finite() && v.abs() <= 1e6)
                        && op
                            .exit_mm
                            .is_none_or(|p| p.iter().all(|v| v.is_finite() && v.abs() <= 1e6)),
                    "all numeric parameters must be finite".into(),
                ),
                (
                    (1e-7..=0.01).contains(&op.tolerance_mm),
                    format!("tolerance_mm {} must be within 1e-7..0.01", op.tolerance_mm),
                ),
                (!op.paths.is_empty(), "at least one path is needed".into()),
                (
                    op.tip_z_mm >= spec.min_mm[2] && op.tip_z_mm < spec.max_mm[2],
                    format!(
                        "tip_z_mm {} must lie within the stock height [{}, {})",
                        op.tip_z_mm, spec.min_mm[2], spec.max_mm[2]
                    ),
                ),
                (
                    op.clearance_z_mm > spec.max_mm[2] + guard,
                    format!(
                        "clearance_z_mm {} must be above the stock top {}",
                        op.clearance_z_mm, spec.max_mm[2]
                    ),
                ),
                (
                    initial.iter().all(|v| v.is_finite() && v.abs() <= 1e6)
                        && initial[2] >= op.clearance_z_mm,
                    format!(
                        "the start position {initial:?} must be finite and at or above clearance_z_mm {}",
                        op.clearance_z_mm
                    ),
                ),
                (
                    op.feed_mm_min > 0. && op.plunge_mm_min > 0. && op.spindle_rpm > 0.,
                    "feed_mm_min, plunge_mm_min and spindle_rpm must be positive".into(),
                ),
                (
                    (1..=9).contains(&op.work_offset),
                    format!("work_offset {} must be 1..9", op.work_offset),
                ),
                (
                    op.tool_number > 0 && op.length_register > 0,
                    "tool_number and length_register must be positive".into(),
                ),
                (
                    first_id != 0 && first_id.checked_add(1).is_some(),
                    "motion ID budget exhausted".into(),
                ),
            ],
        )?;
        let z = op.tip_z_mm;
        let feed = Some(op.feed_mm_min);
        let line = |a: [f64; 2], za: f64, b: [f64; 2], zb: f64| Curve::Line {
            start: [a[0], a[1], za],
            end: [b[0], b[1], zb],
        };
        let mut steps: Vec<Step> = vec![];
        let mut segments = 0;
        let up = op.clearance_z_mm;
        let start = [initial[0], initial[1]];
        if initial[2] != up {
            steps.push((
                MoveKind::Rapid,
                line(start, initial[2], start, up),
                None,
                None,
            ));
        }
        if start != op.entry_mm {
            steps.push((
                MoveKind::Rapid,
                line(start, up, op.entry_mm, up),
                None,
                None,
            ));
        }
        let mut at = op.entry_mm;
        // Down through the entry column, proven open when it is reached.
        steps.push((
            MoveKind::Plunge,
            line(at, up, at, z),
            Some(op.plunge_mm_min),
            Some(at),
        ));
        for (i, path) in op.paths.iter().enumerate() {
            if cancel.is_cancelled() {
                return Err("undercut planning cancelled".into());
            }
            let (cuts, closed) = curves(path, z, guard)
                .map_err(|e| format!("invalid undercut parameters: path {i}: {e}"))?;
            segments += cuts.len();
            if at != path.start_mm {
                steps.push((MoveKind::Lead, line(at, z, path.start_mm, z), feed, None));
            }
            for c in &cuts {
                steps.push((MoveKind::Cut, c.clone(), feed, None));
            }
            let end = cuts.last().unwrap().end();
            at = [end[0], end[1]];
            if op.exit_mm.is_none() {
                // Back the way it came: along an open path to its start,
                // then to the entry.
                if !closed {
                    for c in cuts.iter().rev() {
                        steps.push((MoveKind::Lead, reversed(c), feed, None));
                    }
                    at = path.start_mm;
                }
                if at != op.entry_mm {
                    steps.push((MoveKind::Lead, line(at, z, op.entry_mm, z), feed, None));
                    at = op.entry_mm;
                }
            }
        }
        let leave = op.exit_mm.unwrap_or(op.entry_mm);
        if at != leave {
            steps.push((MoveKind::Lead, line(at, z, leave, z), feed, None));
            at = leave;
        }
        steps.push((
            MoveKind::Retract,
            line(at, z, at, up),
            Some(op.plunge_mm_min),
            Some(at),
        ));
        let mut next_id = first_id;
        let mut pending = VecDeque::with_capacity(steps.len());
        for (kind, curve, feed, column) in steps {
            curve
                .prepare(op.tolerance_mm)
                .map_err(|e| format!("invalid undercut parameters: {e}"))?;
            pending.push_back((
                Motion {
                    id: next_id,
                    operation: op.id.clone(),
                    tool: op.tool.clone(),
                    kind,
                    curve,
                    feed_mm_min: feed,
                },
                column,
            ));
            next_id = next_id.checked_add(1).ok_or("motion ID budget exhausted")?;
        }
        // The assembly's parts: cutting part, neck, shank, holder sections.
        let mut envelope = vec![(cutter.diameter_mm / 2., 0.)];
        if let Some(n) = &cutter.neck {
            envelope.push((n.diameter_mm / 2., cutter.cutting_length_mm));
        }
        envelope.push((
            cutter.shank_diameter_mm / 2.,
            cutter.cutting_length_mm + cutter.neck.as_ref().map_or(0., |n| n.length_mm),
        ));
        for h in &tool.definition().holders {
            envelope.push((h.radius_min_mm.max(h.radius_max_mm), h.z_min_mm));
        }
        let report = UndercutReport {
            strategy: "undercut",
            geometric_residual_status: ResidualStatus::Pending,
            paths: op.paths.len(),
            segments,
            tip_z_mm: z,
            cutting_band_z_mm: [z, z + cutter.cutting_length_mm],
            entry_mm: op.entry_mm,
            exit_mm: op.exit_mm,
            cut_segments_emitted: 0,
            machining_scope: "tool center on the given paths at one depth, going down and up through columns proven open; moves under material are decided by the tri-dexel stock sweep",
        };
        Ok(Self {
            op,
            initial,
            envelope,
            pending,
            report,
            done: false,
        })
    }
    pub fn report(&self) -> &UndercutReport {
        &self.report
    }
    pub fn prefix_events(&self) -> Vec<Event> {
        vec![
            Event::WorkOffset {
                number: self.op.work_offset,
            },
            Event::ToolChange {
                tool: self.op.tool.clone(),
                number: self.op.tool_number,
                length_register: self.op.length_register,
                position_after_mm: self.initial,
            },
            Event::Spindle {
                rpm: self.op.spindle_rpm,
                direction: crate::motion::SpindleDirection::Cw,
            },
            Event::Coolant {
                mode: self.op.coolant,
            },
        ]
    }
    pub fn suffix_events(&self) -> Vec<Event> {
        vec![
            Event::Coolant { mode: Coolant::Off },
            Event::Spindle {
                rpm: 0.,
                direction: crate::motion::SpindleDirection::Cw,
            },
        ]
    }
    pub fn finish_report(&mut self, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("undercut report cancelled".into());
        }
        if !self.done {
            return Err("drain committed undercut motions first".into());
        }
        // The stock decides what was left; this plan estimates no residual.
        self.report.geometric_residual_status = ResidualStatus::Unknown;
        Ok(())
    }
    /// Whether the whole assembly's column at `xy` is open down to the cut
    /// depth on `stock`: in every cell a part of the assembly can reach
    /// (whole-cell bounds), no material stands above the height where the
    /// lowest such part starts.
    fn column_open(&self, stock: &Stock, xy: [f64; 2]) -> bool {
        let spec = stock.spec();
        let [dx, dy] = stock.cell_widths_mm();
        let [nx, ny] = stock.dimensions();
        let half = dx.hypot(dy) / 2.;
        let guard = self.op.tolerance_mm;
        let reach = self.envelope.iter().map(|e| e.0).fold(0., f64::max) + half + guard;
        if xy[0] + reach < spec.min_mm[0]
            || xy[0] - reach > spec.max_mm[0]
            || xy[1] + reach < spec.min_mm[1]
            || xy[1] - reach > spec.max_mm[1]
        {
            return true;
        }
        let cell = |v: f64, k: usize, n: usize, d: f64| {
            ((v - spec.min_mm[k]) / d).floor().clamp(0., (n - 1) as f64) as usize
        };
        let (i0, i1) = (
            cell(xy[0] - reach, 0, nx, dx),
            cell(xy[0] + reach, 0, nx, dx),
        );
        let (j0, j1) = (
            cell(xy[1] - reach, 1, ny, dy),
            cell(xy[1] + reach, 1, ny, dy),
        );
        for j in j0..=j1 {
            for i in i0..=i1 {
                let c = stock.cell_center(j * nx + i).unwrap();
                let d = (c[0] - xy[0]).hypot(c[1] - xy[1]) - half;
                let top = stock.cells()[j * nx + i].upper_mm;
                let lowest = self
                    .envelope
                    .iter()
                    .filter(|e| e.0 + guard >= d)
                    .map(|e| e.1)
                    .fold(f64::INFINITY, f64::min);
                if lowest.is_finite() && top > self.op.tip_z_mm + lowest + guard {
                    return false;
                }
            }
        }
        true
    }
    pub fn next_motion_with_stock(
        &mut self,
        stock: &Stock,
        cancel: &impl CancelSignal,
    ) -> Result<Option<Motion>, String> {
        if cancel.is_cancelled() {
            return Err("undercut planning cancelled".into());
        }
        let Some((motion, column)) = self.pending.pop_front() else {
            self.done = true;
            return Ok(None);
        };
        if let Some(xy) = column
            && !self.column_open(stock, xy)
        {
            return Err(format!(
                "undercut {}: the column at ({:.4}, {:.4}) is not proven open for the whole tool down to tip_z_mm {}: an undercut cutter goes down and up through open space (outside the part or a cleared pocket); move entry_mm or exit_mm, or clear that column first",
                self.op.id, xy[0], xy[1], self.op.tip_z_mm
            ));
        }
        if motion.kind == MoveKind::Cut {
            self.report.cut_segments_emitted += 1;
        }
        Ok(Some(motion))
    }
}
