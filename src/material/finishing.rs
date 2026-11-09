//! Operation-scoped material and vertical ball-cutter access enclosures.
//! The original target/gouge report remains unchanged. No fixtures, shank or
//! holder are included in the ball-access model; those require motion checks.
use super::*;
use crate::surface::ContactEnclosure;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct FinishingScope {
    pub floor_z_mm: f64,
    pub tool_radius_mm: f64,
    pub stock_to_leave_mm: f64,
    pub access_error_mm: f64,
    pub max_refinement_depth: u8,
    pub max_access_work: u64,
}
impl Default for FinishingScope {
    fn default() -> Self {
        Self {
            floor_z_mm: 0.,
            tool_radius_mm: 1.5,
            stock_to_leave_mm: 0.,
            access_error_mm: 0.02,
            max_refinement_depth: 3,
            max_access_work: 2_000_000,
        }
    }
}
impl FinishingScope {
    pub(super) fn validate(self, base: f64) -> Result<(), String> {
        if !self.floor_z_mm.is_finite()
            || self.floor_z_mm < base
            || self.floor_z_mm.abs() > 1e6
            || !self.tool_radius_mm.is_finite()
            || !(0.0001..=10000.).contains(&self.tool_radius_mm)
            || !self.stock_to_leave_mm.is_finite()
            || !(0. ..=100.).contains(&self.stock_to_leave_mm)
            || !self.access_error_mm.is_finite()
            || !(0.0001..=1.).contains(&self.access_error_mm)
            || self.max_refinement_depth > 6
            || !(1..=10_000_000).contains(&self.max_access_work)
        {
            return Err("invalid finishing material depth/ball/allowance/access budget".into());
        }
        Ok(())
    }
}
/// Overlapping limitations on a finding, not independent error contributions.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct AccessUncertainty {
    pub missing_inner_contact: bool,
    pub stock_height_interval: bool,
    pub access_height_interval: bool,
    pub cell_exceeds_ball_footprint: bool,
}
impl AccessUncertainty {
    fn flags(self) -> [bool; 4] {
        [
            self.missing_inner_contact,
            self.stock_height_interval,
            self.access_height_interval,
            self.cell_exceeds_ball_footprint,
        ]
    }
}
/// Counts cover all finding cells, including those omitted by max_findings.
#[derive(Debug, Default, Serialize, Deserialize)]
pub struct FinishingUncertainty {
    pub finding_cells: usize,
    pub missing_inner_contact_cells: usize,
    pub stock_height_interval_cells: usize,
    pub access_height_interval_cells: usize,
    pub cell_exceeds_ball_footprint_cells: usize,
}
impl FinishingUncertainty {
    fn counts(&self) -> [usize; 4] {
        [
            self.missing_inner_contact_cells,
            self.stock_height_interval_cells,
            self.access_height_interval_cells,
            self.cell_exceeds_ball_footprint_cells,
        ]
    }
    fn add(&mut self, flags: AccessUncertainty) {
        self.finding_cells += 1;
        self.missing_inner_contact_cells += usize::from(flags.missing_inner_contact);
        self.stock_height_interval_cells += usize::from(flags.stock_height_interval);
        self.access_height_interval_cells += usize::from(flags.access_height_interval);
        self.cell_exceeds_ball_footprint_cells += usize::from(flags.cell_exceeds_ball_footprint);
    }
}
fn zero_summary() -> Summary {
    Summary {
        assessment: Assessment::WithinTolerance,
        definite_cells: 0,
        possible_only_cells: 0,
        volume_mm3: [0.; 2],
    }
}
#[derive(Debug, Serialize, Deserialize)]
pub struct AccessFinding {
    pub cell_index: u32,
    pub xy_mm: [f64; 4],
    pub allowed_target_height_mm: [f64; 2],
    /// Bounds on the lowest remaining surface attainable by a vertical ball.
    /// An upper bound is a whole-cell feasible cutter witness, not a sample.
    pub attainable_height_mm: [f64; 2],
    pub reachable_residual_height_mm: [f64; 2],
    pub tool_limited_residual_height_mm: [f64; 2],
    /// Absent in older retained reports; absence does not mean no uncertainty.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uncertainty: Option<AccessUncertainty>,
}
#[derive(Debug, Serialize, Deserialize)]
pub struct FinishingReport {
    pub scope: FinishingScope,
    pub complete: bool,
    pub classified_cells: usize,
    pub unexamined_cells: usize,
    pub access_work: u64,
    pub inner_contact_available: bool,
    pub within_depth_exclusion: Summary,
    pub within_allowance: Summary,
    pub in_scope_residual: Summary,
    pub reachable_residual: Summary,
    pub tool_limited_residual: Summary,
    pub access_uncertain_cells: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uncertainty: Option<FinishingUncertainty>,
    pub findings: Vec<AccessFinding>,
    pub findings_truncated: bool,
    pub access_model: String,
}
fn remaining(actual: [f64; 2], cutoff: [f64; 2], guard: f64) -> [f64; 2] {
    [
        (actual[0] - cutoff[1] - guard).max(0.),
        (actual[1] - cutoff[0] + guard).max(0.),
    ]
}
fn band(actual: [f64; 2], bottom: [f64; 2], top: [f64; 2], guard: f64) -> [f64; 2] {
    [
        (actual[0].min(top[0]) - bottom[1] - guard).max(0.),
        (actual[1].min(top[1]) - bottom[0] + guard).max(0.),
    ]
}

type CellAccess = ([f64; 2], [f64; 2], [f64; 2]);

pub(super) struct FinishingAccumulator<'a> {
    target: &'a PreparedSurfaceTarget,
    contact_bounds: ContactEnclosure<'a>,
    spec: StockSpec,
    scope: FinishingScope,
    guard: f64,
    area: f64,
    cells: usize,
    inner: bool,
    mesh_inner: bool,
    witness_bounds: Option<super::contact_points::ContactGrid<'a>>,
    work: u64,
    classified: usize,
    uncertain: usize,
    uncertainty: FinishingUncertainty,
    exhausted: bool,
    deferred: Accumulator,
    allowed: Accumulator,
    residual: Accumulator,
    reachable: Accumulator,
    limited: Accumulator,
    findings: Vec<AccessFinding>,
    truncated: bool,
}
impl<'a> FinishingAccumulator<'a> {
    pub(super) fn new(target: &'a PreparedSurfaceTarget, spec: &StockSpec) -> Result<Self, String> {
        let scope = target
            .definition()
            .finishing
            .ok_or("missing finishing scope")?;
        if scope.floor_z_mm > spec.max_mm[2] {
            return Err("finishing material depth lies above stock".into());
        }
        let (nx, ny, step) = spec.grid()?;
        let guard = 1024.
            * f64::EPSILON
            * (1.
                + spec
                    .min_mm
                    .iter()
                    .chain(&spec.max_mm)
                    .map(|v| v.abs())
                    .sum::<f64>()
                + scope.tool_radius_mm);
        Ok(Self {
            target,
            witness_bounds: target
                .contacts
                .as_ref()
                .map(|p| super::contact_points::ContactGrid::new(p, scope, guard)),
            contact_bounds: ContactEnclosure::new(
                target.access_surface.as_ref().unwrap_or(&target.surface),
                scope.floor_z_mm + scope.tool_radius_mm,
            ),
            spec: spec.clone(),
            scope,
            guard,
            area: step[0] * step[1],
            cells: nx * ny,
            inner: !target.definition().inner_patches.is_empty()
                || target.contacts.is_some()
                || target.definition().surface.surface_error_mm == 0.
                || matches!(
                    target.definition().source_enclosure,
                    SurfaceEnclosure::Inscribed
                ),
            mesh_inner: target.definition().surface.surface_error_mm == 0.
                || matches!(
                    target.definition().source_enclosure,
                    SurfaceEnclosure::Inscribed
                ),
            work: 0,
            classified: 0,
            uncertain: 0,
            uncertainty: FinishingUncertainty::default(),
            exhausted: false,
            deferred: Accumulator::default(),
            allowed: Accumulator::default(),
            residual: Accumulator::default(),
            reachable: Accumulator::default(),
            limited: Accumulator::default(),
            findings: vec![],
            truncated: false,
        })
    }
    fn bounds(
        &mut self,
        xy: [f64; 4],
        actual: [f64; 2],
        target: [f64; 2],
        cancel: &impl CancelSignal,
    ) -> Result<Option<CellAccess>, String> {
        let scope = self.scope;
        let floor = scope.floor_z_mm;
        let radius = scope.tool_radius_mm;
        let allowance = scope.stock_to_leave_mm;
        let error = self.target.definition().surface.surface_error_mm;
        let guard = self.guard;
        let spec = &self.spec;
        let expanded = |e: f64| [xy[0] - e, xy[1] - e, xy[2] + e, xy[3] + e];
        let mut desired = target;
        if allowance > 0. {
            let Some(outer) = self.target.surface.material_bounds(
                expanded(error + allowance),
                self.target.options().geometry_tolerance_mm + guard,
                &mut self.work,
                scope.max_access_work,
                cancel,
            )?
            else {
                return Ok(None);
            };
            // The vertical segment of an allowance ball gives an inner
            // inclusion; an expanded XY box plus height gives an outer one.
            // Raw target bounds have already been clipped to the stock box.
            // Adding allowance to that clipped lower height invents protected
            // material when the original target lies BELOW the analysis box.
            // Recover an unclipped inner height from this existing query and
            // the declared source premise, then clip only after the offset.
            let base = self.target.definition().base_z_mm;
            let mut inner = match self.target.definition().source_enclosure {
                SurfaceEnclosure::Bounded if error > 0. => base,
                SurfaceEnclosure::Offset => (outer[0] - error).max(base),
                SurfaceEnclosure::Inscribed | SurfaceEnclosure::Bounded => outer[0],
            };
            let Some(patch_inner) = self.target.patches.lower_height(
                xy,
                inner,
                guard,
                (&mut self.work, scope.max_access_work),
                cancel,
            )?
            else {
                return Ok(None);
            };
            inner = patch_inner;
            if target[0] > spec.min_mm[2] {
                inner = inner.max(target[0]);
            }
            desired = [
                (inner + allowance).clamp(spec.min_mm[2], spec.max_mm[2]),
                (outer[1] + error + allowance).clamp(spec.min_mm[2], spec.max_mm[2]),
            ];
        }
        let cutoff = desired.map(|z| z.max(floor));
        let mut access = [cutoff[0], spec.max_mm[2]];
        // No material reaches even the lowest in-scope cutoff. Every possible
        // access height yields the same zero reachable/tool-limited residual;
        // avoid querying an irrelevant cutter position. Keep the honest broad
        // access interval and original depth/allowance/gouge accounting.
        if actual[1] + guard <= cutoff[0] {
            return Ok(Some((desired, cutoff, access)));
        }
        let rho = ((xy[2] - xy[0]) * 0.5).hypot((xy[3] - xy[1]) * 0.5) + guard;
        if rho < radius {
            // Once the allowed-target upper bound reaches the stock ceiling,
            // max(witness, cutoff).min(ceiling) is already exactly the ceiling.
            // A contact query cannot tighten it. Keep the same uncertainty;
            // inner exclusion refinement below still runs when applicable.
            if cutoff[1] < spec.max_mm[2] {
                let center = [(xy[0] + xy[2]) * 0.5, (xy[1] + xy[3]) * 0.5];
                let Some([_, height]) = self.contact_bounds.bounds(
                    center,
                    radius + allowance + error + guard,
                    scope.access_error_mm * 0.25,
                    &mut self.work,
                    scope.max_access_work,
                    cancel,
                )?
                else {
                    return Ok(None);
                };
                // One vertical ball approach covers the ENTIRE cell. Its
                // highest bottom is at the farthest corner. The source outer
                // offset guards the complete sphere, not only its tip.
                access[1] = (height.max(floor + radius) + guard
                    - ((radius - rho) * (radius + rho)).sqrt())
                .max(cutoff[1])
                .min(spec.max_mm[2]);
            }
            if self.mesh_inner && access[1] - access[0] > scope.access_error_mm {
                let mut stack = vec![(expanded(radius + guard), 0u8)];
                let mut minimum = access[1];
                while let Some((r, depth)) = stack.pop() {
                    cancelled(cancel)?;
                    let center = [(r[0] + r[2]) * 0.5, (r[1] + r[3]) * 0.5];
                    let delta = ((r[2] - r[0]) * 0.5).hypot((r[3] - r[1]) * 0.5) + guard;
                    let reduced = radius + allowance - delta - guard;
                    let lower = if reduced > 0. {
                        let Some([z, _]) = self.contact_bounds.bounds(
                            center,
                            reduced,
                            scope.access_error_mm * 0.25,
                            &mut self.work,
                            scope.max_access_work,
                            cancel,
                        )?
                        else {
                            return Ok(None);
                        };
                        (z - radius - guard).max(cutoff[0])
                    } else {
                        cutoff[0]
                    };
                    // Triangle inequality: a contact of the smaller ball
                    // remains a forbidden contact for EVERY full-radius
                    // center in this rectangle. All possible cutter centers
                    // covering any cell point lie in the expanded root.
                    if lower >= minimum {
                        continue;
                    }
                    if depth == scope.max_refinement_depth
                        || access[1] - lower <= scope.access_error_mm
                    {
                        minimum = minimum.min(lower);
                    } else {
                        for (a, b) in [(r[0], center[0]), (center[0], r[2])] {
                            for (c, d) in [(r[1], center[1]), (center[1], r[3])] {
                                stack.push(([a, c, b, d], depth + 1));
                            }
                        }
                    }
                }
                access[0] = access[0].max(minimum);
            }
            if let Some(grid) = self.witness_bounds.as_mut()
                && access[1] - access[0] > scope.access_error_mm
            {
                let Some(z) = grid.lower_height(
                    expanded(radius + guard),
                    &mut self.work,
                    scope.max_access_work,
                    cancel,
                )?
                else {
                    return Ok(None);
                };
                access[0] = access[0].max(z - radius - guard);
            }
        }
        access[0] = access[0].min(access[1]);
        Ok(Some((desired, cutoff, access)))
    }
    pub(super) fn add(
        &mut self,
        i: usize,
        xy: [f64; 4],
        actual: [f64; 2],
        target: [f64; 2],
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        if self.exhausted {
            return Ok(());
        }
        cancelled(cancel)?;
        let Some((desired, cutoff, access)) = self.bounds(xy, actual, target, cancel)? else {
            self.exhausted = true;
            return Ok(());
        };
        let guard = self.guard;
        let tol = self.target.options().residual_tolerance();
        let area = self.area;
        self.deferred.add(
            band(actual, target, [self.scope.floor_z_mm; 2], guard),
            area,
            tol,
        );
        // These endpoints are the SAME target when allowance is zero. Treating
        // their interval bounds independently invents a nonzero allowance band.
        if self.scope.stock_to_leave_mm > 0. {
            self.allowed.add(
                band(
                    actual,
                    target.map(|z| z.max(self.scope.floor_z_mm)),
                    desired,
                    guard,
                ),
                area,
                tol,
            );
        }
        self.residual
            .add(remaining(actual, cutoff, guard), area, tol);
        let reachable = remaining(actual, access, guard);
        let limited = band(actual, cutoff, access, guard);
        let rc = self.reachable.add(reachable, area, tol);
        let lc = self.limited.add(limited, area, tol);
        if access[1] - access[0] > self.scope.access_error_mm {
            self.uncertain += 1;
        }
        if rc != Assessment::WithinTolerance || lc != Assessment::WithinTolerance {
            let flags = AccessUncertainty {
                missing_inner_contact: !self.inner,
                stock_height_interval: actual[1] - actual[0] > tol,
                access_height_interval: access[1] - access[0] > self.scope.access_error_mm,
                cell_exceeds_ball_footprint: ((xy[2] - xy[0]) * 0.5).hypot((xy[3] - xy[1]) * 0.5)
                    + guard
                    >= self.scope.tool_radius_mm,
            };
            self.uncertainty.add(flags);
            if self.findings.len() < self.target.options().max_findings {
                self.findings.push(AccessFinding {
                    cell_index: i as u32,
                    xy_mm: xy,
                    allowed_target_height_mm: cutoff,
                    attainable_height_mm: access,
                    reachable_residual_height_mm: reachable,
                    tool_limited_residual_height_mm: limited,
                    uncertainty: Some(flags),
                });
            } else {
                self.truncated = true;
            }
        }
        self.classified += 1;
        Ok(())
    }
    pub(super) fn finish(self) -> FinishingReport {
        let unexamined = self.cells - self.classified;
        let unknown = unexamined as f64
            * self.area
            * (self.spec.max_mm[2] - self.spec.min_mm[2])
            * (1. + 32. * f64::EPSILON);
        let allowed = if self.scope.stock_to_leave_mm == 0. {
            zero_summary()
        } else {
            self.allowed.finish(unexamined, unknown)
        };
        let witness_note = if self.target.contacts.is_some() {
            " Point-to-source witnesses supply additional exclusion constraints; sparse points do not establish surface coverage or an inner volume enclosure."
        } else {
            ""
        };
        FinishingReport{scope:self.scope,complete:unexamined==0,classified_cells:self.classified,unexamined_cells:unexamined,access_work:self.work,inner_contact_available:self.inner,
            within_depth_exclusion:self.deferred.finish(unexamined,unknown),within_allowance:allowed,in_scope_residual:self.residual.finish(unexamined,unknown),reachable_residual:self.reachable.finish(unexamined,unknown),tool_limited_residual:self.limited.finish(unexamined,unknown),access_uncertain_cells:self.uncertain,uncertainty:Some(self.uncertainty),findings:self.findings,findings_truncated:self.truncated,
            access_model:"Vertical ball only, unrestricted XY approach above downward target; whole-cell feasible sphere and all-center exclusion bounds, clipped to stock analysis box. Excludes shank/holder/fixtures/machine travel and generated path coverage. Complete means all cells classified, not that reachable residual is absent. Volume intervals are correlated; do not add them as exact quantities.".to_owned()+witness_note}
    }
}

impl FinishingReport {
    fn valid_uncertainty(&self) -> bool {
        let Some(summary) = &self.uncertainty else {
            return self.findings.iter().all(|f| f.uncertainty.is_none());
        };
        if summary.finding_cells > self.classified_cells
            || summary.counts().iter().any(|n| *n > summary.finding_cells)
            || summary.missing_inner_contact_cells
                != if self.inner_contact_available {
                    0
                } else {
                    summary.finding_cells
                }
            || summary.access_height_interval_cells > self.access_uncertain_cells
            || self.findings.len() > summary.finding_cells
            || (!self.findings_truncated && self.findings.len() != summary.finding_cells)
        {
            return false;
        }
        let mut stored = [0usize; 4];
        for finding in &self.findings {
            let Some(flags) = finding.uncertainty else {
                return false;
            };
            if flags.missing_inner_contact == self.inner_contact_available
                || flags.access_height_interval
                    != (finding.attainable_height_mm[1] - finding.attainable_height_mm[0]
                        > self.scope.access_error_mm)
            {
                return false;
            }
            for (count, flag) in stored.iter_mut().zip(flags.flags()) {
                *count += usize::from(flag);
            }
        }
        stored
            .into_iter()
            .zip(summary.counts())
            .all(|(seen, total)| {
                if self.findings_truncated {
                    seen <= total
                } else {
                    seen == total
                }
            })
    }
    pub(crate) fn valid_binding(
        &self,
        scope: FinishingScope,
        options: Options,
        cells: usize,
    ) -> bool {
        self.scope == scope
            && self.classified_cells.checked_add(self.unexamined_cells) == Some(cells)
            && self.complete == (self.unexamined_cells == 0)
            && self.classified_cells <= options.max_classified_cells
            && self.access_work <= scope.max_access_work
            && self.access_uncertain_cells <= self.classified_cells
            && self.findings.len() <= options.max_findings
            && self.valid_uncertainty()
            && [
                &self.within_depth_exclusion,
                &self.within_allowance,
                &self.in_scope_residual,
                &self.reachable_residual,
                &self.tool_limited_residual,
            ]
            .iter()
            .enumerate()
            .all(|(index, s)| {
                let known_zero_allowance = index == 1
                    && scope.stock_to_leave_mm == 0.
                    && s.volume_mm3 == [0.; 2]
                    && s.definite_cells == 0
                    && s.possible_only_cells == 0
                    && s.assessment == Assessment::WithinTolerance;
                s.volume_mm3.iter().all(|v| v.is_finite())
                    && s.volume_mm3[0] >= 0.
                    && s.volume_mm3[1] >= s.volume_mm3[0]
                    && s.definite_cells
                        .checked_add(s.possible_only_cells)
                        .is_some_and(|n| n <= self.classified_cells)
                    && (known_zero_allowance
                        || (s.assessment == Assessment::Incomplete) == !self.complete)
                    && (s.assessment != Assessment::WithinTolerance
                        || s.definite_cells + s.possible_only_cells == 0)
            })
            && self.findings.iter().all(|f| {
                (f.cell_index as usize) < cells
                    && [
                        f.allowed_target_height_mm,
                        f.attainable_height_mm,
                        f.reachable_residual_height_mm,
                        f.tool_limited_residual_height_mm,
                    ]
                    .iter()
                    .all(|v| v.iter().all(|x| x.is_finite()) && v[0] <= v[1])
            })
    }
}
