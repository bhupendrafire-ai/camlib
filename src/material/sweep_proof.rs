//! Per-move gouge proof against a surface target (#72).
//!
//! The whole-cell height comparison cannot decide a cell that a vertical
//! wall crosses: a cutter run tangent to the wall and one a micrometre into
//! it leave identical cell bounds. This module proves each motion instead,
//! against the target mesh, and the material check then decides a
//! possible-gouge cell as within tolerance when no unproven motion's
//! footprint touches it.
//!
//! Claim, for effective gouge tolerance τ and the mesh's declared surface error
//! e: no point the tool sweeps lies deeper than τ inside the true part P*.
//!
//! - M is the solid under the mesh (the height-field semantics of the
//!   material check, with its base plane). The declared outer source bound
//!   gives P* ⊂ M ⊕ B_e. It does NOT give P* ⊖ B_e ⊂ M: a narrow missing
//!   mesh strip can be filled by the true part without violating that bound.
//! - Every tool section (flat to r0, then rising linearly; `flat::Section`)
//!   is shrunk radially by a and lifted by a. Each point of the real tool is
//!   within k·a of that eroded tool: k = √2 where the section is flat out
//!   to r0 >= a (move the point a inward and a up), and √(1 + (1 + rise)²)
//!   near a pointed tip.
//! - A ball or bull-nose end (radius R, corner rc >= a) is proven with its
//!   real shape (#107): eroded by a it is the bull of radius R - a and
//!   corner rc - a lifted by a (the corner's tube and the flat both shrink
//!   by a about the same centres), and every point of the real end is
//!   within a of it (k = 1: the end's boundary is a union of a-balls). Its
//!   contact is `Index::bull_chord_bound_clear`. A corner smaller than a is
//!   replaced by the enclosing flat end (a superset: sound, only
//!   conservative).
//! - Prove (E ⊕ B_e) disjoint from M, where E is the eroded tool: sections
//!   grow radially by e and lower by e, enclosing the spherical expansion
//!   (for the eroded bull end exactly: radius and corner grow by e and it
//!   lowers by e). Then E is disjoint from M ⊕ B_e and hence from P*. If a
//!   real swept point q were deeper than τ in P*, its eroded-tool point p at
//!   distance k·a < τ - e < τ would remain inside P*, a contradiction.
//!   Contact against every triangle uses `Index::sections_chord_clear` (and
//!   `Index::bull_chord_bound_clear` for a real end), with its continuous
//!   concavity bound; the source expansion is never replaced by samples.
//! - Arcs and helices are proven chord by chord. A point at fraction f of a
//!   sub-arc of half angle θ is within R[(1 - cos θ) + (θ - sin θ)] of the
//!   chord's point at f. Sections grow by that deviation plus the actual
//!   analytic-to-declared endpoint gap and a binary64 path guard. Non-XY
//!   chords need the same vertical clearance; XY chords still spend the
//!   numeric guard vertically, covering helix interpolation rounding.
//!
//! A cell is decided only when the whole history since the raw stock was
//! proven (the stream started there, or resumed from a producer checkpoint
//! that kept this proof's state, #108), and never from a definite gouge.
use super::*;
use crate::flat::{Index, Section, enclosing_sections};
use crate::motion::{Curve, Motion, Plane, Vec3};
use crate::tool::PreparedAssembly;
use std::collections::HashMap;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SweepProofReport {
    /// The proven depth bound: no swept point deeper than this inside the
    /// true part (the effective gouge tolerance).
    pub tolerance_mm: f64,
    /// The target mesh's declared surface error, spent from the tolerance.
    pub surface_error_mm: f64,
    pub proven_moves: u64,
    pub unproven_moves: u64,
    /// Whether every motion since the raw stock was seen: true for a stream
    /// started from the raw stock, or resumed from a producer checkpoint
    /// that kept the proof state (#108); false after resuming from a
    /// checkpoint without it (the motions before it are unseen).
    pub history_complete: bool,
    /// The report's possible-only gouge cells the proof decides (no
    /// unproven move touches them): when it equals the report's
    /// `possible_only_cells`, the gouge is within tolerance.
    pub decided_cells: usize,
    /// Why no move could be proven, when none could.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub unavailable: Option<String>,
}

struct ToolProof {
    /// The eroded sections and the erosion a.
    sections: Vec<Section>,
    erosion: f64,
    /// The eroded ball or bull-nose end (radius, corner), proven with its
    /// real shape; `sections` then holds the rest of the assembly.
    end: Option<[f64; 2]>,
    /// The real assembly's largest radius (unproven footprints).
    reach: f64,
}

/// What the motions are proven against.
enum Geometry {
    /// A surface target's mesh (height-field solid under it).
    Mesh(Index),
    /// A prismatic target: the solid below `floor` (when above the stock
    /// bottom), and outside `region` everything below `top` (the stock top).
    Prism {
        region: Box<crate::planar::PreparedRegion>,
        floor: Option<f64>,
        top: f64,
        local: bool,
    },
}

impl Geometry {
    /// The prism's walls: the disc of radius `rho` swept from p to q (XY)
    /// stays inside the region. The signed clearance to the boundary is
    /// 1-Lipschitz, so a piece of half length h whose midpoint clears
    /// rho plus h is proven; otherwise it is halved. A midpoint closer than
    /// rho is a real violation; a piece shorter than the arithmetic floor
    /// fails.
    fn segment_clearance(
        region: &crate::planar::PreparedRegion,
        p: [f64; 2],
        q: [f64; 2],
        rho: f64,
        inside: bool,
    ) -> Result<bool, String> {
        let length = (q[0] - p[0]).hypot(q[1] - p[1]);
        let mut stack = vec![(0f64, 1f64)];
        let mut queries = 0u32;
        while let Some((t0, t1)) = stack.pop() {
            queries += 1;
            if queries > 1_000_000 {
                return Ok(false);
            }
            let t = (t0 + t1) / 2.;
            let mid = [p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])];
            let Some(c) = region.query(mid)?.signed_clearance_mm else {
                return Ok(false);
            };
            let c = if inside { c } else { -c };
            let half = length * (t1 - t0) / 2.;
            if c - half >= rho {
                continue;
            }
            if c < rho || half < 1e-7 {
                return Ok(false);
            }
            stack.push((t, t1));
            stack.push((t0, t));
        }
        Ok(true)
    }
}

/// A proof state's fixed fields: history flag, proven and unproven counts,
/// cell count, encoding, payload length.
const STATE_FIXED: usize = 1 + 8 + 8 + 8 + 1 + 8;
/// Touched cells as raw bits (cell i is bit i % 8 of byte i / 8).
const STATE_BITS: u8 = 0;
/// Touched cells as alternating run lengths (LEB128), untouched first.
const STATE_RUNS: u8 = 1;

fn put_varint(out: &mut Vec<u8>, mut v: u64) {
    while v >= 0x80 {
        out.push(v as u8 | 0x80);
        v >>= 7;
    }
    out.push(v as u8);
}
/// A canonical LEB128 value (no redundant trailing groups).
fn get_varint(bytes: &[u8], at: &mut usize) -> Option<u64> {
    let mut v = 0u64;
    for shift in (0..64).step_by(7) {
        let b = *bytes.get(*at)?;
        *at += 1;
        if shift == 63 && b > 1 {
            return None;
        }
        v |= u64::from(b & 0x7f) << shift;
        if b & 0x80 == 0 {
            return (b != 0 || shift == 0).then_some(v);
        }
    }
    None
}

/// A decoded checkpoint state (see `SweepProof::state`).
pub(crate) struct ProofState {
    history_complete: bool,
    proven: u64,
    unproven: u64,
    touched: Vec<bool>,
}

pub(crate) struct SweepProof {
    geometry: Option<Geometry>,
    unavailable: Option<String>,
    tolerance: f64,
    surface_error: f64,
    budget: f64,
    spec: StockSpec,
    nx: usize,
    ny: usize,
    step: [f64; 2],
    /// Stock cells some unproven motion's footprint touches.
    touched: Vec<bool>,
    history_complete: bool,
    proven: u64,
    unproven: u64,
    tools: HashMap<String, Option<ToolProof>>,
    /// Only newly marked cells need undo if stream publication fails.
    pending: Option<(u64, u64, Vec<usize>)>,
    seen: Vec<u32>,
    stamp: u32,
}

impl SweepProof {
    pub(crate) fn new(
        target: &SurfaceTarget,
        options: Options,
        spec: &StockSpec,
        history_complete: bool,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let tolerance = options.gouge_tolerance();
        let surface_error = target.surface.surface_error_mm;
        let scale = spec
            .min_mm
            .into_iter()
            .chain(spec.max_mm)
            .map(f64::abs)
            .fold(1., f64::max);
        // Strict margin for the containment argument and binary64 rounding.
        let budget = tolerance - surface_error - 1024. * f64::EPSILON * scale;
        let (geometry, unavailable) = if budget > 0. {
            // Below the stock box there is no stock to gouge: the base plane
            // binds only when it lies above the stock's bottom.
            let floor = if target.base_z_mm > spec.min_mm[2] {
                target.base_z_mm
            } else {
                spec.min_mm[2] - 1e6
            };
            (
                Some(Geometry::Mesh(Index::new(
                    &target.surface,
                    floor,
                    1.,
                    cancel,
                )?)),
                None,
            )
        } else {
            (
                None,
                Some(format!(
                    "the effective gouge tolerance {tolerance} mm and target mesh surface error {surface_error} mm leave no proof budget after numerical guards; refine the source enclosure or explicitly revise the declared gouge policy"
                )),
            )
        };
        Self::with(
            geometry,
            unavailable,
            tolerance,
            surface_error,
            budget,
            spec,
            history_complete,
        )
    }

    /// A prismatic target: its region's boundary error (with the numerical
    /// guard and geometry tolerance, as the cell comparison uses) and its
    /// floor error are the declared errors spent from the tolerance.
    pub(crate) fn new_prismatic(
        target: &Target,
        options: Options,
        spec: &StockSpec,
        history_complete: bool,
    ) -> Result<Self, String> {
        let region = target
            .machining_region
            .prepare(crate::planar::ConstructionOptions {
                tolerance_mm: options.geometry_tolerance_mm,
            })?;
        let summary = region.summary();
        let lateral =
            summary.boundary_error_mm + summary.numerical_guard_mm + options.geometry_tolerance_mm;
        let surface_error = lateral.max(target.floor_error_mm);
        let tolerance = options.gouge_tolerance();
        let scale = spec
            .min_mm
            .into_iter()
            .chain(spec.max_mm)
            .map(f64::abs)
            .fold(1., f64::max);
        let budget = tolerance - surface_error - 1024. * f64::EPSILON * scale;
        let (geometry, unavailable) = if budget > 0. {
            (
                Some(Geometry::Prism {
                    region: Box::new(region),
                    floor: (target.floor_z_mm > spec.min_mm[2]).then_some(target.floor_z_mm),
                    top: spec.max_mm[2],
                    local: options.scope_to_region,
                }),
                None,
            )
        } else {
            (
                None,
                Some(format!(
                    "the effective gouge tolerance {tolerance} mm and target boundary and floor error {surface_error} mm leave no proof budget after numerical guards; refine the source enclosure or explicitly revise the declared gouge policy"
                )),
            )
        };
        Self::with(
            geometry,
            unavailable,
            tolerance,
            surface_error,
            budget,
            spec,
            history_complete,
        )
    }

    fn with(
        geometry: Option<Geometry>,
        unavailable: Option<String>,
        tolerance: f64,
        surface_error: f64,
        budget: f64,
        spec: &StockSpec,
        history_complete: bool,
    ) -> Result<Self, String> {
        let (nx, ny, step) = spec.grid()?;
        let seen = vec![
            0;
            match &geometry {
                Some(Geometry::Mesh(index)) => index.len(),
                _ => 0,
            }
        ];
        Ok(Self {
            geometry,
            unavailable,
            tolerance,
            surface_error,
            budget,
            spec: spec.clone(),
            nx,
            ny,
            step,
            touched: vec![false; nx * ny],
            history_complete,
            proven: 0,
            unproven: 0,
            tools: HashMap::new(),
            pending: None,
            seen,
            stamp: 0,
        })
    }

    pub(crate) fn begin_motion(&mut self) {
        self.pending = Some((self.proven, self.unproven, vec![]));
    }

    pub(crate) fn finish_motion(&mut self, commit: bool) {
        if let Some((proven, unproven, touched)) = self.pending.take()
            && !commit
        {
            self.proven = proven;
            self.unproven = unproven;
            for cell in touched {
                self.touched[cell] = false;
            }
        }
    }

    fn tool(&mut self, tool: &PreparedAssembly) -> Result<(), String> {
        let id = &tool.definition().id;
        if self.tools.contains_key(id) {
            return Ok(());
        }
        let proof = match enclosing_sections(tool) {
            Err(_) => None,
            Ok(sections) => {
                let reach = sections.iter().map(|s| s.r1).fold(0., f64::max);
                let flat = self.budget / std::f64::consts::SQRT_2;
                let k = sections
                    .iter()
                    .map(|s| {
                        if s.r0 >= flat {
                            std::f64::consts::SQRT_2
                        } else {
                            (1. + (1. + s.rise).powi(2)).sqrt()
                        }
                    })
                    .fold(std::f64::consts::SQRT_2, f64::max);
                // Every section must survive. Dropping a narrow cutter while
                // retaining its larger holder would not prove cutter clearance.
                let min_radius = sections.iter().map(|s| s.r1).fold(f64::INFINITY, f64::min);
                let a = (self.budget / k).min(min_radius / 2.);
                let c = tool.cutter().definition();
                let radius = c.diameter_mm / 2.;
                let corner = match c.shape {
                    crate::tool::CutterShape::Ball => Some(radius),
                    crate::tool::CutterShape::Bull { corner_radius_mm } => Some(corner_radius_mm),
                    _ => None,
                };
                // `enclosing_sections` puts the cutter's end first.
                let end = corner
                    .filter(|rc| *rc >= a && radius > a)
                    .map(|rc| [radius - a, rc - a]);
                let rest = if end.is_some() {
                    &sections[1..]
                } else {
                    &sections[..]
                };
                let sections = rest
                    .iter()
                    .map(|s| Section {
                        z0: s.z0 + s.rise * (a - s.r0).max(0.),
                        r0: (s.r0 - a).max(0.),
                        r1: s.r1 - a,
                        rise: s.rise,
                    })
                    .collect::<Vec<_>>();
                ((end.is_some() || !sections.is_empty()) && sections.iter().all(Section::valid))
                    .then_some(ToolProof {
                        sections,
                        erosion: a,
                        end,
                        reach,
                    })
            }
        };
        self.tools.insert(id.clone(), proof);
        Ok(())
    }

    /// Prove one motion; an unproven motion marks the stock cells its
    /// footprint (the whole assembly's reach) touches.
    /// `tolerance_mm`: the stream's curve tolerance (`SweepOptions`), which
    /// validated the motion already (a posted arc's rounded endpoints agree
    /// with its centre only to that).
    pub(crate) fn motion(
        &mut self,
        tool: &PreparedAssembly,
        motion: &Motion,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(), String> {
        self.tool(tool)?;
        let proven = self.prove(&tool.definition().id, motion, tolerance_mm, cancel)?;
        if proven {
            self.proven += 1;
            return Ok(());
        }
        self.unproven += 1;
        let reach = match self.tools.get(&tool.definition().id) {
            Some(Some(p)) => p.reach,
            _ => tool
                .definition()
                .holders
                .iter()
                .map(|h| h.radius_max_mm.max(h.radius_min_mm))
                .fold(tool.cutter().definition().diameter_mm / 2., f64::max),
        };
        let bounds = motion.curve.prepare(tolerance_mm)?.bounds();
        let pad = reach + self.step[0].max(self.step[1]);
        let cell = |v: f64, k: usize, n: usize| {
            (((v - self.spec.min_mm[k]) / self.step[k]).floor().max(0.) as usize).min(n - 1)
        };
        let (x0, x1) = (
            cell(bounds.min[0] - pad, 0, self.nx),
            cell(bounds.max[0] + pad, 0, self.nx),
        );
        let (y0, y1) = (
            cell(bounds.min[1] - pad, 1, self.ny),
            cell(bounds.max[1] + pad, 1, self.ny),
        );
        if bounds.max[0] + pad >= self.spec.min_mm[0]
            && bounds.min[0] - pad <= self.spec.max_mm[0]
            && bounds.max[1] + pad >= self.spec.min_mm[1]
            && bounds.min[1] - pad <= self.spec.max_mm[1]
        {
            for y in y0..=y1 {
                for cell in y * self.nx + x0..=y * self.nx + x1 {
                    if !self.touched[cell] {
                        if let Some((_, _, touched)) = &mut self.pending {
                            touched.push(cell);
                        }
                        self.touched[cell] = true;
                    }
                }
            }
        }
        Ok(())
    }

    fn prove(
        &mut self,
        tool: &str,
        motion: &Motion,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<bool, String> {
        let (Some(geometry), Some(Some(proof))) = (&self.geometry, self.tools.get(tool)) else {
            return Ok(false);
        };
        let a = proof.erosion;
        let source_error = self.surface_error;
        let mut remaining_pairs = 1_000_000u64;
        let mut chord = |p: Vec3,
                         q: Vec3,
                         grow: f64,
                         lift: f64,
                         seen: &mut [u32],
                         stamp: &mut u32|
         -> Result<bool, String> {
            let sections = proof
                .sections
                .iter()
                .map(|s| Section {
                    z0: s.z0,
                    r0: s.r0 + source_error + grow,
                    r1: s.r1 + source_error + grow,
                    rise: s.rise,
                })
                .collect::<Vec<_>>();
            // Enclose E ⊕ B_e by growing its radii by e and lowering it
            // by e. Additional growth/lift encloses sub-arc deviation.
            match geometry {
                Geometry::Mesh(index) => {
                    if cancel.is_cancelled() {
                        return Err("material sweep proof cancelled".into());
                    }
                    let checks = sections.len() as u64 + u64::from(proof.end.is_some());
                    let pairs = (index.len() as u64).saturating_mul(checks);
                    if pairs > remaining_pairs {
                        return Ok(false);
                    }
                    remaining_pairs -= pairs;
                    // The eroded end E ⊕ B_e ⊕ (radial growth): radius
                    // R - a + e + grow, corner rc - a + e, lowered by e.
                    Ok((sections.is_empty()
                        || index.sections_chord_clear(
                            p,
                            q,
                            &sections,
                            source_error + lift - a,
                            seen,
                            stamp,
                        ))
                        && proof.end.is_none_or(|[r, rc]| {
                            index.bull_chord_bound_clear(
                                p,
                                q,
                                r + source_error + grow,
                                rc + source_error,
                                source_error + lift - a,
                                seen,
                                stamp,
                            )
                        }))
                }
                Geometry::Prism {
                    region,
                    floor,
                    top,
                    local,
                } => {
                    // The prism is proven against the end's enclosing flat
                    // (its full radius at its tip), as before.
                    let mut sections = sections;
                    if let Some([r, _]) = proof.end {
                        sections.push(Section {
                            z0: 0.,
                            r0: r + source_error + grow,
                            r1: r + source_error + grow,
                            rise: 0.,
                        });
                    }
                    // The eroded tool's lowest point along the chord.
                    let tip = p[2].min(q[2]) + a - source_error - lift;
                    if *local {
                        let Some(floor) = floor else {
                            return Ok(true);
                        };
                        let rho = sections
                            .iter()
                            .filter(|s| tip + s.z0 < *floor)
                            .map(|s| s.r1)
                            .fold(f64::NEG_INFINITY, f64::max);
                        return if rho == f64::NEG_INFINITY {
                            Ok(true)
                        } else {
                            Geometry::segment_clearance(
                                region,
                                [p[0], p[1]],
                                [q[0], q[1]],
                                rho,
                                false,
                            )
                        };
                    }
                    if let Some(f) = floor
                        && tip + sections.iter().map(|s| s.z0).fold(f64::INFINITY, f64::min) < *f
                    {
                        return Ok(false);
                    }
                    // Every section reaching below the stock top must stay
                    // inside the region, at its full radius (conservative).
                    let rho = sections
                        .iter()
                        .filter(|s| tip + s.z0 < *top)
                        .map(|s| s.r1)
                        .fold(f64::NEG_INFINITY, f64::max);
                    if rho == f64::NEG_INFINITY {
                        return Ok(true);
                    }
                    Geometry::segment_clearance(region, [p[0], p[1]], [q[0], q[1]], rho, true)
                }
            }
        };
        let curve = &motion.curve;
        match *curve {
            Curve::Line { start, end } => {
                chord(start, end, 0., 0., &mut self.seen, &mut self.stamp)
            }
            Curve::Arc {
                start,
                end,
                center,
                plane,
                sweep_rad,
            }
            | Curve::Helix {
                start,
                end,
                center,
                plane,
                sweep_rad,
            } => {
                let [u, v, _] = plane.axes();
                let r = (start[u] - center[u]).hypot(start[v] - center[v]);
                let prepared = curve.prepare(tolerance_mm)?;
                let angle = (start[v] - center[v]).atan2(start[u] - center[u]) + sweep_rad;
                let (sin, cos) = angle.sin_cos();
                let endpoint_gap =
                    (center[u] + r * cos - end[u]).hypot(center[v] + r * sin - end[v]);
                // Zero admission tolerance requests only the shared binary64
                // guard. Spend the actual endpoint gap, not the caller's
                // potentially much larger circular admission tolerance.
                let numeric_guard = crate::verification::path_error(motion, 0.);
                let path_guard = endpoint_gap + numeric_guard;
                // Adaptive sub-chords: a piece of half angle θ lies within
                // d(θ) = R[(1 - cos θ) + (θ - sin θ)] of its chord at equal
                // fractions (θ <= π/4 here), so its chord is proven with the
                // sections grown by d plus the fixed path guard (with the
                // same vertical clearance out of XY). A piece that fails
                // is halved (d falls about 4x; endpoint error does not)
                // until d is a quarter of the erosion; failing there, the
                // move is unproven. Open-air arcs prove in one chord.
                let deviation = |f0: f64, f1: f64| {
                    let half = sweep_rad.abs() * (f1 - f0) / 2.;
                    r * ((1. - half.cos()) + (half - half.sin()))
                };
                let quarters = (sweep_rad.abs() / std::f64::consts::FRAC_PI_2)
                    .ceil()
                    .max(1.) as usize;
                let mut stack: Vec<(f64, f64)> = (0..quarters)
                    .rev()
                    .map(|k| (k as f64 / quarters as f64, (k + 1) as f64 / quarters as f64))
                    .collect();
                let mut chords = 0u32;
                while let Some((f0, f1)) = stack.pop() {
                    chords += 1;
                    if chords > 100_000 {
                        return Ok(false);
                    }
                    if chords.is_multiple_of(256) && cancel.is_cancelled() {
                        return Err("material sweep proof cancelled".into());
                    }
                    let d = deviation(f0, f1);
                    let grow = d + path_guard;
                    let lift = if plane == Plane::Xy {
                        numeric_guard
                    } else {
                        grow
                    };
                    let (p, q) = (prepared.position(f0)?, prepared.position(f1)?);
                    if chord(p, q, grow, lift, &mut self.seen, &mut self.stamp)? {
                        continue;
                    }
                    if d <= a / 4. {
                        return Ok(false);
                    }
                    let m = (f0 + f1) / 2.;
                    stack.push((m, f1));
                    stack.push((f0, m));
                }
                Ok(true)
            }
        }
    }

    /// Cells the proof decides: the whole history proven and no unproven
    /// motion's footprint on the cell.
    pub(crate) fn decides(&self, cell: usize) -> bool {
        self.history_complete && self.geometry.is_some() && !self.touched[cell]
    }

    /// The largest `state` encoding for a grid of `cells` (raw bits plus the
    /// fixed fields): a reader's bound before it allocates.
    pub(crate) fn max_state_bytes(cells: usize) -> usize {
        STATE_FIXED + cells.div_ceil(8)
    }

    /// The proof's state after the motions seen so far (#108), for a stream
    /// checkpoint at the same cursor: whether the history is complete, the
    /// proven and unproven counts, and the touched cells (one bit per cell,
    /// run-length coded when that is smaller). Everything else the proof
    /// holds is derived from the run's manifest.
    pub(crate) fn state(&self) -> Vec<u8> {
        let cells = self.touched.len();
        let raw = {
            let mut bits = vec![0u8; cells.div_ceil(8)];
            for (i, _) in self.touched.iter().enumerate().filter(|(_, t)| **t) {
                bits[i / 8] |= 1 << (i % 8);
            }
            bits
        };
        // Alternating run lengths, the first of untouched cells (may be 0).
        let mut runs = Vec::new();
        let (mut value, mut length) = (false, 0u64);
        for &t in &self.touched {
            if t != value {
                put_varint(&mut runs, length);
                (value, length) = (t, 0);
            }
            length += 1;
        }
        put_varint(&mut runs, length);
        let (mode, payload) = if runs.len() < raw.len() {
            (STATE_RUNS, runs)
        } else {
            (STATE_BITS, raw)
        };
        let mut out = Vec::with_capacity(STATE_FIXED + payload.len());
        out.push(u8::from(self.history_complete));
        out.extend_from_slice(&self.proven.to_le_bytes());
        out.extend_from_slice(&self.unproven.to_le_bytes());
        out.extend_from_slice(&(cells as u64).to_le_bytes());
        out.push(mode);
        out.extend_from_slice(&(payload.len() as u64).to_le_bytes());
        out.extend_from_slice(&payload);
        out
    }

    /// Decode a checkpoint's `state` for this proof (its cursor and run
    /// identity were checked by the stream). Refuses a state for another
    /// grid or a malformed one; `apply_state` then continues from it.
    pub(crate) fn decode_state(&self, state: &[u8]) -> Result<ProofState, String> {
        let bad = || "invalid material proof state in checkpoint".to_string();
        let u64_at = |at: usize| -> Result<u64, String> {
            state
                .get(at..at + 8)
                .map(|b| u64::from_le_bytes(b.try_into().unwrap()))
                .ok_or_else(bad)
        };
        if state.len() < STATE_FIXED || state[0] > 1 {
            return Err(bad());
        }
        let (proven, unproven) = (u64_at(1)?, u64_at(9)?);
        let cells = self.touched.len();
        if u64_at(17)? != cells as u64 {
            return Err("checkpoint material proof state is for another stock grid".into());
        }
        let mode = state[25];
        let length = u64_at(26)?;
        let payload = &state[STATE_FIXED..];
        if payload.len() as u64 != length {
            return Err(bad());
        }
        let mut touched = vec![false; cells];
        match mode {
            STATE_BITS => {
                if payload.len() != cells.div_ceil(8)
                    || (!cells.is_multiple_of(8) && payload[payload.len() - 1] >> (cells % 8) != 0)
                {
                    return Err(bad());
                }
                for (i, t) in touched.iter_mut().enumerate() {
                    *t = payload[i / 8] >> (i % 8) & 1 == 1;
                }
            }
            STATE_RUNS => {
                let (mut at, mut cell, mut value) = (0, 0usize, false);
                while at < payload.len() {
                    let run = get_varint(payload, &mut at).ok_or_else(bad)?;
                    // Only the first run (untouched) may be empty.
                    if (run == 0 && cell != 0) || run > (cells - cell) as u64 {
                        return Err(bad());
                    }
                    touched[cell..cell + run as usize].fill(value);
                    cell += run as usize;
                    value = !value;
                }
                if cell != cells {
                    return Err(bad());
                }
            }
            _ => return Err(bad()),
        }
        Ok(ProofState {
            history_complete: state[0] == 1,
            proven,
            unproven,
            touched,
        })
    }

    /// `raw`: whether the run starts from the raw stock (no declared initial
    /// stock); a kept state claims complete history only then.
    pub(crate) fn apply_state(&mut self, state: ProofState, raw: bool) {
        self.history_complete = state.history_complete && raw;
        self.proven = state.proven;
        self.unproven = state.unproven;
        self.touched = state.touched;
    }

    /// Forget every motion: the state of a stream with nothing restored.
    /// `fresh`: it starts here from the raw stock (cursor 0, no declared
    /// initial stock, not a retained snapshot); otherwise the history before
    /// it is unseen.
    pub(crate) fn reset(&mut self, fresh: bool) {
        self.history_complete = fresh;
        self.proven = 0;
        self.unproven = 0;
        self.touched.fill(false);
    }

    pub(crate) fn report(&self, decided_cells: usize) -> SweepProofReport {
        SweepProofReport {
            tolerance_mm: self.tolerance,
            surface_error_mm: self.surface_error,
            proven_moves: self.proven,
            unproven_moves: self.unproven,
            history_complete: self.history_complete,
            decided_cells: if self.history_complete && self.geometry.is_some() {
                decided_cells
            } else {
                0
            },
            unavailable: self.unavailable.clone().or_else(|| {
                (!self.history_complete).then(|| {
                    "the check did not start from raw stock with complete motion history: \
                     a declared initial stock, or a stream resumed from a checkpoint without \
                     this check's proof state (resume from the producer's own checkpoint, which \
                     keeps it)"
                        .to_string()
                })
            }),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::MoveKind;
    use crate::surface::SurfaceMesh;
    use crate::tool::{Cutter, CutterShape, HolderSection, ToolAssembly};
    use std::sync::atomic::AtomicBool;

    const PART: (Vec3, Vec3) = ([0., 0., 0.], [20., 20., 10.]);

    fn cuboid(lo: Vec3, hi: Vec3) -> Vec<[Vec3; 3]> {
        let p = |x: usize, y: usize, z: usize| {
            [[lo[0], hi[0]][x], [lo[1], hi[1]][y], [lo[2], hi[2]][z]]
        };
        let quad = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| vec![[a, b, c], [a, c, d]];
        [
            quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1)),
            quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0)),
            quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1)),
            quad(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0)),
            quad(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0)),
            quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1)),
        ]
        .concat()
    }

    fn target(error: f64) -> SurfaceTarget {
        SurfaceTarget {
            version: 1,
            units: "mm".into(),
            id: "box".into(),
            revision: "1".into(),
            surface: SurfaceMesh {
                version: 1,
                units: "mm".into(),
                triangles: std::sync::Arc::new(cuboid(PART.0, PART.1)),
                surface_error_mm: error,
                source: serde_json::json!({}),
            },
            base_z_mm: 0.,
            source_enclosure: SurfaceEnclosure::Bounded,
            contact_witnesses: None,
            inner_patches: vec![],
            finishing: None,
        }
    }

    fn spec() -> StockSpec {
        StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: [-15., -15., 0.],
            max_mm: [35., 35., 12.],
            cell_size_mm: 0.5,
            max_cells: u32::MAX as usize,
        }
    }

    fn tool(shape: CutterShape) -> ToolAssembly {
        ToolAssembly {
            id: "t".into(),
            cutter: Cutter {
                id: "t".into(),
                name: String::new(),
                shape,
                diameter_mm: 6.,
                cutting_length_mm: 16.,
                overall_length_mm: 60.,
                shank_diameter_mm: 6.,
                flutes: 2,
                center_cutting: true,
                material: "carbide".into(),
                neck: None,
            },
            stickout_mm: 30.,
            gauge_length_mm: 80.,
            holders: vec![HolderSection {
                z_min_mm: 30.,
                z_max_mm: 80.,
                radius_min_mm: 12.,
                radius_max_mm: 20.,
            }],
        }
    }

    fn motion(curve: Curve) -> Motion {
        Motion {
            id: 1,
            operation: "op".into(),
            tool: "t".into(),
            kind: MoveKind::Cut,
            curve,
            feed_mm_min: Some(600.),
        }
    }

    /// Independent oracle: the deepest sampled point of the real tool (its
    /// own radius profile, solid) inside the box, along the move.
    fn deepest(tool: &PreparedAssembly, curve: &Curve) -> f64 {
        let prepared = curve.prepare(1e-6).unwrap();
        let (lo, hi) = PART;
        let mut worst: f64 = 0.;
        for i in 0..=100 {
            let c = prepared.position(i as f64 / 100.).unwrap();
            for k in 0..=60 {
                let h = 12. * k as f64 / 60.;
                let Some(slice) = tool.radius_at(h) else {
                    continue;
                };
                for j in 0..=12 {
                    let rho = slice.radius_mm * j as f64 / 12.;
                    for m in 0..72 {
                        let phi = std::f64::consts::TAU * m as f64 / 72.;
                        let p = [c[0] + rho * phi.cos(), c[1] + rho * phi.sin(), c[2] + h];
                        let depth = (0..3)
                            .map(|a| (p[a] - lo[a]).min(hi[a] - p[a]))
                            .fold(f64::INFINITY, f64::min);
                        worst = worst.max(depth);
                    }
                }
            }
        }
        worst
    }

    fn line(a: Vec3, b: Vec3) -> Curve {
        Curve::Line { start: a, end: b }
    }

    fn proves(proof: &mut SweepProof, tool: &PreparedAssembly, curve: Curve) -> bool {
        let before = proof.proven;
        proof
            .motion(tool, &motion(curve), 1e-5, &AtomicBool::new(false))
            .unwrap();
        proof.proven > before
    }

    fn proof(tolerance: f64, error: f64, complete: bool) -> SweepProof {
        let options = Options {
            comparison_tolerance_mm: tolerance,
            ..Options::default()
        };
        SweepProof::new(
            &target(error),
            options,
            &spec(),
            complete,
            &AtomicBool::new(false),
        )
        .unwrap()
    }

    #[test]
    fn tangent_wall_passes_prove_and_cuts_into_the_wall_do_not() {
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        // A contour pass 3 mm off the x = 0 wall (radius 3), down at Z 4;
        // a positive offset moves it into the wall.
        for (offset, tolerance, error, expect) in [
            (0., 0.005, 0.0005, true),
            (0.0007, 0.005, 0.0005, true),
            (0.01, 0.005, 0.0005, false),
            (0.0007, 0.001, 0.00088, false),
            (-0.01, 0.001, 0.00088, true),
        ] {
            let mut p = proof(tolerance, error, true);
            let x = -3. + offset;
            let curve = line([x, -5., 4.], [x, 25., 4.]);
            let proven = proves(&mut p, &flat, curve.clone());
            assert_eq!(proven, expect, "offset {offset} tolerance {tolerance}");
            if proven {
                assert!(deepest(&flat, &curve) <= tolerance - error);
            }
        }
    }

    #[test]
    fn flat_profile_proves_analytic_inclined_tangency_only() {
        // For z = 4 + m*x, contact is the maximum of m*rho - profile(rho).
        // These closed-form supports are independent of section construction
        // and the native planner: flat R*m, ball R*(sqrt(1+m²)-1), and
        // bull (R-rc)*m + rc*(sqrt(1+m²)-1).
        let m: f64 = 0.25;
        let normal = (1. + m * m).sqrt();
        let mut definition = target(0.);
        let p = |x, y| [x, y, 4. + m * x];
        definition.surface.triangles = vec![
            [p(-10., -20.), p(30., -20.), p(30., 40.)],
            [p(-10., -20.), p(30., 40.), p(-10., 40.)],
        ]
        .into();
        definition.prepare(Options::default()).unwrap();
        let tolerance = 0.02;
        for (shape, rc) in [(CutterShape::Flat, 0.)] {
            let prepared = tool(shape).prepare().unwrap();
            let support = (3. - rc) * m + rc * (normal - 1.);
            let rho = 3. - rc + rc * m / normal;
            let profile = rc * (1. - 1. / normal);
            for (penetration, expected) in [(0., true), (2. * tolerance, false)] {
                let z = 6. + support - penetration * normal;
                // The exact contact witness penetrates the half-space by
                // precisely `penetration` along its unit normal.
                let depth = (4. + m * (8. + rho) - (z + profile)) / normal;
                assert!((depth - penetration).abs() < 1e-12);
                let mut proof = SweepProof::new(
                    &definition,
                    Options {
                        comparison_tolerance_mm: tolerance,
                        ..Options::default()
                    },
                    &spec(),
                    true,
                    &AtomicBool::new(false),
                )
                .unwrap();
                assert_eq!(
                    proves(&mut proof, &prepared, line([8., 5., z], [8., 15., z])),
                    expected,
                    "real shape {:?}, exact normal penetration {depth}",
                    prepared.cutter().definition().shape,
                );
            }
        }
    }

    #[test]
    fn source_outer_error_protects_true_material_filling_a_narrow_mesh_slit() {
        let mut tiny = tool(CutterShape::Flat);
        tiny.cutter.diameter_mm = 0.01;
        tiny.cutter.shank_diameter_mm = 0.01;
        let tiny = tiny.prepare().unwrap();
        let mut definition = target(0.05);
        let rectangle = |x0, x1| {
            let p = [[x0, 0., 10.], [x1, 0., 10.], [x1, 20., 10.], [x0, 20., 10.]];
            vec![[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
        };
        definition.surface.triangles = [rectangle(0., 9.98), rectangle(10.02, 20.)].concat().into();
        let options = Options {
            comparison_tolerance_mm: 0.1,
            ..Options::default()
        };
        let deep = line([10., 5., 5.], [10., 15., 5.]);
        // The full solid [0,20]² × [0,10] is a valid true source: every
        // filled slit point is <= 0.02 mm from the mesh solid, below the
        // declared 0.05 mm outer error. The real cutter's center at Z5 is
        // independently 5 mm inside that solid, far beyond tolerance.
        for enclosure in [SurfaceEnclosure::Bounded, SurfaceEnclosure::Offset] {
            definition.source_enclosure = enclosure;
            definition.prepare(options).unwrap();
            let mut proof =
                SweepProof::new(&definition, options, &spec(), true, &AtomicBool::new(false))
                    .unwrap();
            assert!(!proves(&mut proof, &tiny, deep.clone()));
            // Truly clear above even the complete source outer offset.
            assert!(proves(
                &mut proof,
                &tiny,
                line([10., 5., 10.06], [10., 15., 10.06])
            ));
        }
        // With an exact source, the slit really is empty: do not turn the
        // source-error protection into an unconditional missing-mesh refusal.
        definition.surface.surface_error_mm = 0.;
        let mut exact =
            SweepProof::new(&definition, options, &spec(), true, &AtomicBool::new(false)).unwrap();
        assert!(proves(&mut exact, &tiny, deep));
    }

    #[test]
    fn failed_motion_rolls_back_counts_and_only_its_newly_touched_cells() {
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        let mut proof = proof(0.02, 0., true);
        assert!(!proves(
            &mut proof,
            &flat,
            line([-2., 5., 4.], [-2., 6., 4.])
        ));
        let touched = proof.touched.clone();
        let counts = (proof.proven, proof.unproven);
        proof.begin_motion();
        assert!(!proves(
            &mut proof,
            &flat,
            line([22., 12., 4.], [22., 13., 4.])
        ));
        assert_ne!(proof.touched, touched);
        proof.finish_motion(false);
        assert_eq!(proof.touched, touched);
        assert_eq!((proof.proven, proof.unproven), counts);
        proof.begin_motion();
        assert!(proves(
            &mut proof,
            &flat,
            line([-5., 5., 11.], [-5., 6., 11.])
        ));
        proof.finish_motion(false);
        assert_eq!((proof.proven, proof.unproven), counts);
        proof.begin_motion();
        assert!(!proves(
            &mut proof,
            &flat,
            line([22., 12., 4.], [22., 13., 4.])
        ));
        proof.finish_motion(true);
        assert_ne!(proof.touched, touched);
        assert_eq!(proof.unproven, counts.1 + 1);
    }

    #[test]
    fn proven_moves_never_penetrate_deeper_than_the_tolerance() {
        // Lines, XY arcs and helices around the box, near its walls and
        // top, for flat, drill and ball ends; every proven move is checked
        // against the oracle, and clear moves well away must prove.
        let tools = [
            tool(CutterShape::Flat),
            tool(CutterShape::Drill {
                point_angle_rad: 118f64.to_radians(),
            }),
            tool(CutterShape::Ball),
        ]
        .map(|t| t.prepare().unwrap());
        let (tolerance, error) = (0.005, 0.0005);
        let mut seed = 0x9e3779b97f4a7c15u64;
        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let (mut proven, mut refused) = (0, 0);
        for i in 0..90 {
            let t = &tools[i % 3];
            let mut p = proof(tolerance, error, true);
            // A centre 3 mm (+/- 0.02) off a wall, or over the top.
            let off = 3. + (rand() - 0.5) * 0.04;
            let z = if i % 4 == 0 {
                10. + (rand() - 0.5) * 0.04
            } else {
                2. + 6. * rand()
            };
            let x = if i % 2 == 0 { -off } else { 20. + off };
            let curve = match i % 3 {
                0 => line(
                    [x, -4. + 8. * rand(), z],
                    [x, 16. + 8. * rand(), z + (rand() - 0.5) * 0.02],
                ),
                1 => {
                    // An XY arc of radius 3 + small about a centre on the
                    // wall line's outside, sweeping past the wall.
                    let r = 2. + rand();
                    let center = [
                        x + if x < 0. {
                            -(off - 3.) - r + 3.
                        } else {
                            (off - 3.) + r - 3.
                        },
                        10.,
                        z,
                    ];
                    let a0: f64 = if x < 0. {
                        -1.
                    } else {
                        std::f64::consts::PI - 1.
                    };
                    let start = [center[0] + r * a0.cos(), center[1] + r * a0.sin(), z];
                    let sweep = 2.;
                    let end = [
                        center[0] + r * (a0 + sweep).cos(),
                        center[1] + r * (a0 + sweep).sin(),
                        z,
                    ];
                    Curve::Arc {
                        start,
                        end,
                        center,
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    }
                }
                _ => {
                    let r = 1.5;
                    let cx = if x < 0. { x - r } else { x + r };
                    let a0: f64 = if x < 0. { 0. } else { std::f64::consts::PI };
                    let start = [cx + r * a0.cos(), 10. + r * a0.sin(), z];
                    let sweep = std::f64::consts::TAU;
                    let end = [start[0], start[1], z - 0.5];
                    Curve::Helix {
                        start,
                        end,
                        center: [cx, 10., z],
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    }
                }
            };
            if proves(&mut p, t, curve.clone()) {
                proven += 1;
                let depth = deepest(t, &curve);
                assert!(
                    depth <= tolerance - error,
                    "move {i}: proven, oracle depth {depth}"
                );
            } else {
                refused += 1;
            }
        }
        assert!(
            proven > 20 && refused > 5,
            "proven {proven}, refused {refused}"
        );
        // Well clear (0.05 mm off the walls): always proven.
        for t in &tools {
            let mut p = proof(tolerance, error, true);
            assert!(proves(&mut p, t, line([-3.05, -5., 3.], [-3.05, 25., 3.])));
            assert!(proves(
                &mut p,
                t,
                line([-5., -5., 10.05], [25., -5., 10.05])
            ));
        }
    }

    #[test]
    fn ball_and_bull_ends_are_proven_with_their_real_shape() {
        // #107: a ball (radius 3) and a bull nose (radius 3, corner 1) run
        // along the box's top edge (x = 0, z = 10) with the tip below the
        // top. The real end clears the edge from axis offset h* on; the
        // enclosing flat end (radius 3) would cut it at every h < 3, which
        // is what the proof used before.
        let ball = tool(CutterShape::Ball).prepare().unwrap();
        let bull = tool(CutterShape::Bull {
            corner_radius_mm: 1.,
        })
        .prepare()
        .unwrap();
        let (tolerance, error) = (0.005, 0.0005);
        // Ball, tip at Z 9: centre Z 12, clear iff h^2 + 2^2 >= 3^2.
        // Bull, tip at Z 9.5: tube centre radius 2 at Z 10.5, clear iff
        // (h - 2)^2 + 0.5^2 >= 1.
        let cases = [(&ball, 9., 5f64.sqrt()), (&bull, 9.5, 2. + 0.75f64.sqrt())];
        for (t, tip, tangent) in cases {
            for (offset, expect) in [(0.06, true), (0.02, true), (-0.03, false), (-0.2, false)] {
                let h = tangent + offset;
                let mut p = proof(tolerance, error, true);
                let curve = line([-h, -5., tip], [-h, 25., tip]);
                let proven = proves(&mut p, t, curve.clone());
                assert_eq!(proven, expect, "tip {tip} offset {offset}");
                let depth = deepest(t, &curve);
                if proven {
                    assert!(depth <= tolerance - error, "proven at depth {depth}");
                }
                // The enclosing flat end of the same radius cuts the edge.
                assert!(h < 3.);
            }
        }
        // A ball- or bull-end pass over the floor (the top face) is proven
        // exactly where a flat-end pass is. Touching the floor is provable
        // only while the source error leaves room for the erosion (#133:
        // the tool is dilated by e, so exact contact needs e < τ/(1+k)); at
        // τ 0.001 with e 0.00088 a 10 µm clearance is proven, as for walls
        // (`tangent_wall_passes_prove_and_cuts_into_the_wall_do_not`).
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        for (tolerance, error, z, expect) in [
            (0.005, 0.0005, 10., true),
            (0.005, 0.0005, 9.99, false),
            (0.001, 0.00088, 10.01, true),
            (0.001, 0.00088, 10., false),
            (0.001, 0.00088, 9.99, false),
        ] {
            for t in [&flat, &ball, &bull] {
                let mut p = proof(tolerance, error, true);
                assert_eq!(
                    proves(&mut p, t, line([4., 2., z], [16., 18., z])),
                    expect,
                    "tolerance {tolerance} error {error} z {z}"
                );
            }
        }
    }

    #[test]
    fn proven_curved_end_moves_never_penetrate_deeper_than_the_tolerance() {
        // Random lines, arcs and helices rolling ball and bull ends over the
        // box's top edges and along its walls; every proven move is checked
        // against the oracle (the tool's own radius profile, sampled).
        let tools = [
            tool(CutterShape::Ball),
            tool(CutterShape::Bull {
                corner_radius_mm: 1.,
            }),
            tool(CutterShape::Bull {
                corner_radius_mm: 2.5,
            }),
        ]
        .map(|t| t.prepare().unwrap());
        let (tolerance, error) = (0.005, 0.0005);
        let mut seed = 0x2545f4914f6cdd1du64;
        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        let (mut proven, mut refused) = (0, 0);
        for i in 0..90 {
            let t = &tools[(i / 3) % 3];
            let mut p = proof(tolerance, error, true);
            // Over the top edge (tip 0 to 2 mm below the top, axis 1.8 to
            // 3.1 mm out), or along a wall well below the top.
            let (h, z) = if i % 2 == 0 {
                (1.8 + 1.3 * rand(), 8. + 2. * rand())
            } else {
                (3. + (rand() - 0.5) * 0.04, 2. + 5. * rand())
            };
            let x = if i % 4 < 2 { -h } else { 20. + h };
            let curve = match i % 3 {
                0 => line(
                    [x, -4. + 8. * rand(), z],
                    [x, 16. + 8. * rand(), z + (rand() - 0.5) * 0.02],
                ),
                1 => {
                    // An XY arc tangent to the edge line at its middle.
                    let r = 4. + 2. * rand();
                    let (cx, a0) = if x < 0. {
                        (x - r, -0.6f64)
                    } else {
                        (x + r, std::f64::consts::PI - 0.6)
                    };
                    let center = [cx, 10., z];
                    let sweep = 1.2;
                    let at = |a: f64| [cx + r * a.cos(), 10. + r * a.sin(), z];
                    Curve::Arc {
                        start: at(a0),
                        end: at(a0 + sweep),
                        center,
                        plane: Plane::Xy,
                        sweep_rad: sweep,
                    }
                }
                _ => {
                    let r = 1.5;
                    let cx = if x < 0. { x - r } else { x + r };
                    let a0: f64 = if x < 0. { 0. } else { std::f64::consts::PI };
                    let start = [cx + r * a0.cos(), 10. + r * a0.sin(), z];
                    Curve::Helix {
                        start,
                        end: [start[0], start[1], z - 0.5],
                        center: [cx, 10., z],
                        plane: Plane::Xy,
                        sweep_rad: std::f64::consts::TAU,
                    }
                }
            };
            if proves(&mut p, t, curve.clone()) {
                proven += 1;
                let depth = deepest(t, &curve);
                assert!(
                    depth <= tolerance - error,
                    "move {i}: proven, oracle depth {depth}"
                );
            } else {
                refused += 1;
            }
        }
        assert!(
            proven > 20 && refused > 10,
            "proven {proven}, refused {refused}"
        );
    }

    /// A 40 x 30 pocket to a floor at Z 6 in stock to Z 10 (prismatic
    /// target: below the floor, and outside the pocket up to the top).
    fn prism(tolerance: f64, error: f64) -> SweepProof {
        use crate::planar::{Boundary, Component, PlanarRegion};
        let target = Target {
            version: 1,
            units: "mm".into(),
            id: "pocket".into(),
            revision: "1".into(),
            machining_region: PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "pocket".into(),
                boundary_error_mm: error,
                components: vec![Component {
                    outer: Boundary {
                        vertices: vec![[0., 0., 0.], [40., 0., 0.], [40., 30., 0.], [0., 30., 0.]],
                    },
                    holes: vec![],
                }],
                source: serde_json::json!({}),
            },
            floor_z_mm: 6.,
            floor_error_mm: error,
        };
        let options = Options {
            comparison_tolerance_mm: tolerance,
            ..Options::default()
        };
        let mut spec = spec();
        spec.max_mm[2] = 10.;
        SweepProof::new_prismatic(&target, options, &spec, true).unwrap()
    }

    /// Oracle: the deepest sampled point of the real tool inside the prism.
    fn deepest_in_prism(tool: &PreparedAssembly, curve: &Curve) -> f64 {
        let prepared = curve.prepare(1e-6).unwrap();
        let mut worst: f64 = 0.;
        for i in 0..=100 {
            let c = prepared.position(i as f64 / 100.).unwrap();
            for k in 0..=50 {
                let h = 10. * k as f64 / 50.;
                let Some(slice) = tool.radius_at(h) else {
                    continue;
                };
                for j in 0..=12 {
                    let rho = slice.radius_mm * j as f64 / 12.;
                    for m in 0..72 {
                        let phi = std::f64::consts::TAU * m as f64 / 72.;
                        let p = [c[0] + rho * phi.cos(), c[1] + rho * phi.sin(), c[2] + h];
                        let floor = 6. - p[2];
                        let outside = (-p[0]).max(p[0] - 40.).max(-p[1]).max(p[1] - 30.);
                        let wall = outside.min(10. - p[2]);
                        worst = worst.max(floor).max(wall);
                    }
                }
            }
        }
        worst
    }

    #[test]
    fn prismatic_target_walls_and_floor_are_proven_exactly() {
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        let (tolerance, error) = (0.005, 0.0005);
        for (curve, expect) in [
            // Tangent to the x = 0 wall on the floor (3 mm from the y
            // walls at its ends), and 0.0007 into both.
            (line([3., 3., 6.], [3., 27., 6.]), true),
            (line([2.9993, 3., 5.9993], [2.9993, 27., 5.9993]), true),
            // 0.01 mm into the wall, or into the floor: never proven.
            (line([2.99, 5., 6.], [2.99, 25., 6.]), false),
            (line([10., 5., 5.99], [30., 25., 5.99]), false),
            // A full orbit tangent to three walls' corner region.
            (
                Curve::Helix {
                    start: [6., 3.5, 7.],
                    end: [6., 3.5, 6.],
                    center: [6., 6.5, 7.],
                    plane: Plane::Xy,
                    sweep_rad: std::f64::consts::TAU,
                },
                true,
            ),
            // Above the stock top anywhere: clear.
            (line([-20., -20., 10.5], [60., 50., 10.5]), true),
        ] {
            let mut p = prism(tolerance, error);
            let proven = proves(&mut p, &flat, curve.clone());
            assert_eq!(proven, expect, "{curve:?}");
            if proven {
                assert!(deepest_in_prism(&flat, &curve) <= tolerance - error);
            }
        }
        // The default tolerance is spent by the declared errors here.
        let mut spent = prism(0.001, 0.001);
        assert!(!proves(
            &mut spent,
            &flat,
            line([10., 10., 8.], [20., 10., 8.])
        ));
        assert!(
            spent
                .report(0)
                .unavailable
                .unwrap()
                .contains("boundary and floor error")
        );
    }

    #[test]
    fn posted_arcs_are_proven_at_the_stream_tolerance() {
        // A 3-decimal arc's end agrees with its centre only to the stream's
        // tolerance (here 5e-6 mm off the radius): proven, never an error.
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        let mut p = proof(0.005, 0.0005, true);
        let arc = Curve::Arc {
            start: [-8., 10., 11.],
            end: [-8., 15.000005, 11.],
            center: [-8., 12.5, 11.],
            plane: Plane::Xy,
            sweep_rad: std::f64::consts::PI,
        };
        assert!(arc.prepare(1e-6).is_err());
        assert!(proves(&mut p, &flat, arc));
    }

    fn admitted_endpoint_move(
        definition: &SurfaceTarget,
        plane: Plane,
        coordinate: f64,
        endpoint_offset: f64,
        helix: bool,
        expected: bool,
    ) {
        let mut tiny = tool(CutterShape::Flat);
        tiny.cutter.diameter_mm = 0.1;
        tiny.cutter.shank_diameter_mm = 0.1;
        let tiny = tiny.prepare().unwrap();
        let (start, mut end, center) = match plane {
            Plane::Xy => (
                [coordinate - 1., 10., 5.],
                [coordinate + endpoint_offset, 9., 5.],
                [coordinate, 10., 5.],
            ),
            Plane::Xz => (
                [10., 10., coordinate + 1.],
                [11., 10., coordinate + endpoint_offset],
                [10., 10., coordinate],
            ),
            Plane::Yz => (
                [10., 10., coordinate + 1.],
                [10., 9., coordinate + endpoint_offset],
                [10., 10., coordinate],
            ),
        };
        if helix {
            end[plane.axes()[2]] += 0.5;
        }
        let curve = if helix {
            Curve::Helix {
                start,
                end,
                center,
                plane,
                sweep_rad: std::f64::consts::FRAC_PI_2,
            }
        } else {
            Curve::Arc {
                start,
                end,
                center,
                plane,
                sweep_rad: std::f64::consts::FRAC_PI_2,
            }
        };
        curve.prepare(0.1).unwrap();
        let mut proof = SweepProof::new(
            definition,
            Options {
                comparison_tolerance_mm: 0.005,
                ..Options::default()
            },
            &spec(),
            true,
            &AtomicBool::new(false),
        )
        .unwrap();
        proof
            .motion(&tiny, &motion(curve), 0.1, &AtomicBool::new(false))
            .unwrap();
        assert_eq!(
            proof.proven == 1,
            expected,
            "{plane:?}: coordinate {coordinate}, endpoint offset {endpoint_offset}, helix {helix}"
        );
        assert_eq!(proof.proven + proof.unproven, 1);
    }

    #[test]
    fn admitted_endpoint_gap_cannot_hide_near_terminal_wall_penetration() {
        let mut definition = target(0.);
        definition.surface.triangles = cuboid([2., 0., 0.], [20., 20., 10.]).into();
        definition.prepare(Options::default()).unwrap();
        // A real flat cutter of radius 0.05 at phase 1-1e-4 has the
        // independent witness x = 1.96 - sin(pi/2*1e-4) + 0.05.
        // It is over 0.009 mm inside the x>=2 wall, before the declared
        // endpoint jumps away to x=1.87. Both arcs and helices must refuse it.
        let depth = 1.96 - (std::f64::consts::FRAC_PI_2 * 1e-4).sin() + 0.05 - 2.;
        assert!(depth > 0.009);
        for helix in [false, true] {
            for (coordinate, offset, expected) in [
                (1.96, -0.09, false),
                (1.96, 0., false),  // Exact endpoint: the analytic path still cuts.
                (1.95, 0., true),   // Exact tangent, despite 0.1 mm admission tolerance.
                (1.7, -0.09, true), // A real admitted gap safely far from the wall.
            ] {
                admitted_endpoint_move(&definition, Plane::Xy, coordinate, offset, helix, expected);
            }
        }
    }

    #[test]
    fn admitted_endpoint_gap_also_guards_non_xy_vertical_clearance() {
        let definition = target(0.);
        // In either vertical plane, the analytic tip approaches z=9.99
        // from above. At phase 1-1e-4, its exact depth below z=10 is:
        let depth = 10. - (9.99 + (std::f64::consts::FRAC_PI_2 * 1e-4).sin());
        assert!(depth > 0.009);
        for plane in [Plane::Xz, Plane::Yz] {
            for helix in [false, true] {
                for (coordinate, offset, expected) in [
                    (9.99, 0.09, false), // Declared endpoint z=10.08 hides the true dip.
                    (9.99, 0., false),
                    (10.01, 0., true), // Do not charge the unused admission tolerance.
                    (10.3, 0.09, true),
                ] {
                    admitted_endpoint_move(&definition, plane, coordinate, offset, helix, expected);
                }
            }
        }
    }

    #[test]
    fn proof_state_round_trips_in_both_encodings_and_refuses_malformed_states() {
        let cells = 100 * 100;
        let mut seed = 0x2545f4914f6cdd1du64;
        let mut rand = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            seed
        };
        // Footprint rectangles (runs win) and scattered cells (bits win),
        // plus the empty and full maps.
        let mut patterns: Vec<Vec<bool>> = vec![vec![false; cells], vec![true; cells]];
        let mut rects = vec![false; cells];
        for _ in 0..5 {
            let (x0, y0) = ((rand() % 90) as usize, (rand() % 90) as usize);
            for y in y0..y0 + 8 {
                rects[y * 100 + x0..y * 100 + x0 + 8].fill(true);
            }
        }
        patterns.push(rects);
        patterns.push((0..cells).map(|_| rand() % 3 == 0).collect());
        let mut modes = std::collections::HashSet::new();
        for touched in patterns {
            let mut p = proof(0.005, 0.0005, true);
            assert_eq!(p.touched.len(), cells);
            p.touched = touched.clone();
            (p.proven, p.unproven, p.history_complete) = (rand() >> 8, rand() >> 8, false);
            let state = p.state();
            assert!(state.len() <= SweepProof::max_state_bytes(cells));
            modes.insert(state[25]);
            let mut q = proof(0.005, 0.0005, true);
            let decoded = q.decode_state(&state).unwrap();
            q.apply_state(decoded, true);
            assert_eq!(
                (&q.touched, q.proven, q.unproven, q.history_complete),
                (&touched, p.proven, p.unproven, false)
            );
            assert_eq!(q.state(), state);
        }
        assert_eq!(modes.len(), 2, "both encodings exercised");

        let mut p = proof(0.005, 0.0005, true);
        p.touched[4321] = true;
        let runs = p.state();
        assert_eq!(runs[25], STATE_RUNS);
        p.touched = (0..cells).map(|i| i % 2 == 0).collect();
        let bits = p.state();
        assert_eq!(bits[25], STATE_BITS);
        let q = proof(0.005, 0.0005, true);
        let refuse = |state: Vec<u8>| assert!(q.decode_state(&state).is_err(), "{state:?}");
        // Truncated, trailing byte, bad history flag, unknown encoding.
        refuse(runs[..runs.len() - 1].to_vec());
        refuse([runs.clone(), vec![0]].concat());
        refuse([vec![2], runs[1..].to_vec()].concat());
        let mut bad = runs.clone();
        bad[25] = 7;
        refuse(bad);
        // Another grid's state.
        let mut bad = bits.clone();
        bad[17..25].copy_from_slice(&(cells as u64 + 8).to_le_bytes());
        refuse(bad);
        // Runs: too short a total, an empty inner run, overlong and
        // non-canonical varints.
        let with_runs = |runs: &[u8]| {
            let mut state = bits[..STATE_FIXED].to_vec();
            state[25] = STATE_RUNS;
            state[26..34].copy_from_slice(&(runs.len() as u64).to_le_bytes());
            state.extend_from_slice(runs);
            state
        };
        let mut ok = Vec::new();
        put_varint(&mut ok, cells as u64);
        assert!(q.decode_state(&with_runs(&ok)).is_ok());
        let mut short = Vec::new();
        put_varint(&mut short, cells as u64 - 1);
        refuse(with_runs(&short));
        let mut empty = Vec::new();
        for v in [10, 0, cells as u64 - 10] {
            put_varint(&mut empty, v);
        }
        refuse(with_runs(&empty));
        let mut over = Vec::new();
        put_varint(&mut over, cells as u64 + 1);
        refuse(with_runs(&over));
        refuse(with_runs(&[0x90, 0xce, 0x00]));
        refuse(with_runs(&[0xff; 11]));
        // Bits: a set bit past the last cell (a grid of 9997 cells).
        let mut p = proof(0.005, 0.0005, true);
        p.touched = (0..cells - 3).map(|i| i % 2 == 0).collect();
        let mut odd = p.state();
        assert_eq!(odd[25], STATE_BITS);
        let mut q = proof(0.005, 0.0005, true);
        q.touched.truncate(cells - 3);
        assert!(q.decode_state(&odd).is_ok());
        let last = odd.len() - 1;
        odd[last] |= 0x80;
        assert!(q.decode_state(&odd).is_err());
    }

    #[test]
    fn resumed_history_or_spent_tolerance_decides_nothing() {
        let flat = tool(CutterShape::Flat).prepare().unwrap();
        let mut resumed = proof(0.005, 0.0005, false);
        assert!(proves(
            &mut resumed,
            &flat,
            line([-5., -5., 11.], [25., -5., 11.])
        ));
        assert!(!resumed.decides(0));
        let mut spent = proof(0.001, 0.001, true);
        assert!(!proves(
            &mut spent,
            &flat,
            line([-10., -10., 11.], [-9., -10., 11.])
        ));
        assert!(
            spent
                .report(0)
                .unavailable
                .unwrap()
                .contains("surface error")
        );
        // An unproven move marks the cells under its footprint only.
        let mut p = proof(0.005, 0.0005, true);
        assert!(!proves(&mut p, &flat, line([-2., 5., 4.], [-2., 6., 4.])));
        let cell = |x: f64, y: f64| (((y + 15.) / 0.5) as usize) * 100 + ((x + 15.) / 0.5) as usize;
        assert!(!p.decides(cell(-2., 5.5)));
        assert!(p.decides(cell(30., 30.)));
    }
    #[test]
    fn narrow_cutter_is_not_eroded_away_while_holder_stays_clear() {
        let mut t = tool(CutterShape::Flat);
        t.cutter.diameter_mm = 0.002;
        t.cutter.shank_diameter_mm = 0.002;
        t.cutter.cutting_length_mm = 0.1;
        let prepared = t.prepare().unwrap();
        let mut p = proof(0.1, 0., true);
        assert!(!proves(&mut p, &prepared, line([5., 5., 9.], [6., 5., 9.])));
        let retained = p.tools["t"].as_ref().unwrap();
        assert!(retained.sections.iter().any(|s| s.z0 < 0.01 && s.r1 > 0.));
    }
}
