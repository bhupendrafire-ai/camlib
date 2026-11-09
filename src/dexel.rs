//! Tri-dexel stock (issue #1): three orthogonal grids of rays through the
//! stock box, each ray a prism (its cell's rectangle times the ray's line)
//! holding two sorted, disjoint interval lists along the ray:
//!
//! - **possible** P: outside P the whole cross-section is empty;
//! - **certain** C ⊆ P: inside C the whole cross-section is material.
//!
//! Material can lie below empty space (an undercut), which the height field
//! (`crate::stock`) cannot hold. Grids are only combined conservatively: the
//! material lies inside P_X ∩ P_Y ∩ P_Z and contains C_X ∪ C_Y ∪ C_Z, so a
//! check is clear when one grid proves it, and a violation only on certain
//! material.
//!
//! Tri-dexel (dexel = depth element) is the standard volume representation of
//! NC verification: Van Hook, "Real-time shaded NC milling display",
//! SIGGRAPH 1986 (single dexels); Benouamer and Michelucci, "Bridging the gap
//! between CSG and Brep via a triple ray representation", ACM Solid Modeling
//! 1997 (three orthogonal grids). camlib keeps its own rule on top: every
//! ray holds bounds, not samples (docs/DECISIONS.md, 8).
//!
//! A dexel run also keeps a height-field view of its stock: per Z ray, the top
//! of the certain interval that starts at the stock bottom (lower) and the top
//! of the possible material (upper). That is a valid height-field bound, so
//! every planner and check that reads a height field keeps working, and stays
//! sound, on dexel stock. The view is derived, never sent.
use crate::{
    motion::{Motion, Vec3},
    stock::{CellChange, HeightBounds, Stock, StockDelta, StockSnapshot, StockSpec},
};
use serde::{Deserialize, Serialize};

mod material;
mod sweep;
pub use material::{SolidAssessment, SolidExcess, SolidReport};
pub use sweep::{Axis, DexelSweepStats};

/// Ray indices are u32 in every delta and checkpoint frame.
const MAX_RAYS: usize = u32::MAX as usize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DexelModel {
    TriDexel,
}

/// A tri-dexel stock box. `max_rays` bounds the three grids together;
/// `max_intervals` bounds the stored intervals (possible and certain, all
/// rays): an uncut box holds two per ray, and cuts that leave material on
/// both sides of a ray add more.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DexelStockSpec {
    pub version: u32,
    pub units: String,
    pub model: DexelModel,
    pub min_mm: Vec3,
    pub max_mm: Vec3,
    pub cell_size_mm: f64,
    pub max_rays: usize,
    pub max_intervals: usize,
}

/// The ray grids of a spec: cells per axis and their widths.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DexelGrid {
    pub cells: [usize; 3],
    pub step_mm: [f64; 3],
}
impl DexelGrid {
    /// The two cross-section axes of a ray along `axis`, in index order:
    /// a ray's index is `iv * cells[u] + iu`. For Z rays that is the height
    /// field's cell index (`j * nx + i`).
    pub fn cross(axis: usize) -> [usize; 2] {
        [(axis + 1) % 3, (axis + 2) % 3]
    }
    pub fn rays(&self, axis: usize) -> usize {
        let [u, v] = Self::cross(axis);
        self.cells[u] * self.cells[v]
    }
    pub fn total_rays(&self) -> usize {
        (0..3).map(|a| self.rays(a)).sum()
    }
}

impl DexelStockSpec {
    pub fn grid(&self) -> Result<DexelGrid, String> {
        if self.version != 1
            || self.units != "mm"
            || !self
                .min_mm
                .iter()
                .chain(&self.max_mm)
                .all(|n| n.is_finite() && n.abs() <= 1e6)
            || !self.cell_size_mm.is_finite()
            || !(1e-5..=1e6).contains(&self.cell_size_mm)
            || self.max_rays == 0
            || self.max_rays > MAX_RAYS
            || self.max_intervals == 0
            || (0..3).any(|k| self.max_mm[k] - self.min_mm[k] < 1e-6)
        {
            return Err(
                "dexel stock needs version 1/mm, bounded positive extents, spacing, and ray and interval budgets"
                    .into(),
            );
        }
        let dims: [f64; 3] = std::array::from_fn(|k| {
            ((self.max_mm[k] - self.min_mm[k]) / self.cell_size_mm)
                .ceil()
                .max(1.)
        });
        let rays = dims[0] * dims[1] + dims[1] * dims[2] + dims[2] * dims[0];
        if dims.iter().any(|n| *n > self.max_rays as f64) || rays > self.max_rays as f64 {
            return Err(format!(
                "dexel stock of {} x {} x {} cells needs {rays} rays, more than max_rays {} (hard ceiling {MAX_RAYS}); increase cell_size_mm or max_rays",
                dims[0], dims[1], dims[2], self.max_rays
            ));
        }
        if 2. * rays > self.max_intervals as f64 {
            return Err(format!(
                "dexel stock needs {} intervals for the uncut box (two per ray), more than max_intervals {}; increase max_intervals or cell_size_mm",
                2. * rays,
                self.max_intervals
            ));
        }
        let cells = dims.map(|n| n as usize);
        let step_mm = std::array::from_fn(|k| (self.max_mm[k] - self.min_mm[k]) / dims[k]);
        let magnitude = self
            .min_mm
            .iter()
            .chain(&self.max_mm)
            .map(|n| n.abs())
            .fold(1., f64::max);
        if step_mm
            .iter()
            .any(|d| *d < 1e-6_f64.max(128. * f64::EPSILON * magnitude))
        {
            return Err("dexel cells are below supported numerical resolution".into());
        }
        Ok(DexelGrid { cells, step_mm })
    }
    /// The height-field view's spec: the Z rays' cells.
    pub fn view_spec(&self) -> Result<StockSpec, String> {
        let grid = self.grid()?;
        Ok(StockSpec {
            version: 1,
            units: "mm".into(),
            min_mm: self.min_mm,
            max_mm: self.max_mm,
            cell_size_mm: self.cell_size_mm,
            max_cells: grid.rays(2),
        })
    }
}

/// A sorted list of disjoint closed intervals, stored inline when it holds
/// at most one.
#[derive(Clone, Debug, PartialEq)]
pub enum Spans {
    Empty,
    One([f64; 2]),
    Many(Box<[[f64; 2]]>),
}
impl Spans {
    pub fn as_slice(&self) -> &[[f64; 2]] {
        match self {
            Self::Empty => &[],
            Self::One(s) => std::slice::from_ref(s),
            Self::Many(s) => s,
        }
    }
    pub fn len(&self) -> usize {
        self.as_slice().len()
    }
    pub fn is_empty(&self) -> bool {
        matches!(self, Self::Empty)
    }
    pub fn from_vec(spans: Vec<[f64; 2]>) -> Self {
        match spans.len() {
            0 => Self::Empty,
            1 => Self::One(spans[0]),
            _ => Self::Many(spans.into_boxed_slice()),
        }
    }
    /// Total length.
    pub fn length(&self) -> f64 {
        self.as_slice().iter().map(|s| s[1] - s[0]).sum()
    }
}

/// One ray's bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct Ray {
    pub possible: Spans,
    pub certain: Spans,
}
impl Ray {
    fn full(lo: f64, hi: f64) -> Self {
        Self {
            possible: Spans::One([lo, hi]),
            certain: Spans::One([lo, hi]),
        }
    }
    fn intervals(&self) -> usize {
        self.possible.len() + self.certain.len()
    }
    /// The height-field bounds of a Z ray: lower is the top of the certain
    /// interval that starts at the stock bottom, upper the top of the
    /// possible material.
    fn height(&self, bottom: f64) -> HeightBounds {
        let upper = self.possible.as_slice().last().map_or(bottom, |s| s[1]);
        let lower = match self.certain.as_slice().first() {
            Some(s) if s[0] <= bottom => s[1],
            _ => bottom,
        };
        HeightBounds {
            lower_mm: lower,
            upper_mm: upper,
        }
    }
    /// Sorted, disjoint, finite, nonempty intervals inside [lo, hi], with
    /// the certain material inside the possible material.
    fn valid(&self, lo: f64, hi: f64) -> bool {
        let ok = |s: &Spans| {
            !matches!(s, Spans::Many(m) if m.len() < 2)
                && s.as_slice().iter().all(|i| {
                    i[0].is_finite() && i[1].is_finite() && lo <= i[0] && i[0] < i[1] && i[1] <= hi
                })
                && s.as_slice().windows(2).all(|w| w[0][1] < w[1][0])
        };
        ok(&self.possible)
            && ok(&self.certain)
            && contains(self.possible.as_slice(), self.certain.as_slice())
    }
}

/// Sort and merge touching or overlapping intervals; drop empty ones.
pub(crate) fn normalize(mut spans: Vec<[f64; 2]>) -> Vec<[f64; 2]> {
    spans.retain(|s| s[0] < s[1]);
    spans.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(spans.len());
    for s in spans {
        match out.last_mut() {
            Some(last) if s[0] <= last[1] => last[1] = last[1].max(s[1]),
            _ => out.push(s),
        }
    }
    out
}
/// `a` minus `b`, both normalized.
pub(crate) fn subtract(a: &[[f64; 2]], b: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut out = Vec::with_capacity(a.len() + 1);
    let mut j = 0;
    for &[mut lo, hi] in a {
        while j < b.len() && b[j][1] <= lo {
            j += 1;
        }
        let mut k = j;
        while k < b.len() && b[k][0] < hi {
            if b[k][0] > lo {
                out.push([lo, b[k][0]]);
            }
            lo = lo.max(b[k][1]);
            if lo >= hi {
                break;
            }
            k += 1;
        }
        if lo < hi {
            out.push([lo, hi]);
        }
    }
    out
}
/// `a` intersected with `b`, both normalized.
pub(crate) fn intersect(a: &[[f64; 2]], b: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let (mut i, mut j) = (0, 0);
    let mut out = vec![];
    while i < a.len() && j < b.len() {
        let lo = a[i][0].max(b[j][0]);
        let hi = a[i][1].min(b[j][1]);
        if lo < hi {
            out.push([lo, hi]);
        }
        if a[i][1] < b[j][1] {
            i += 1;
        } else {
            j += 1;
        }
    }
    out
}
/// Whether every interval of `inner` lies inside one of `outer` (normalized).
pub(crate) fn contains(outer: &[[f64; 2]], inner: &[[f64; 2]]) -> bool {
    let mut j = 0;
    inner.iter().all(|s| {
        while j < outer.len() && outer[j][1] < s[1] {
            j += 1;
        }
        j < outer.len() && outer[j][0] <= s[0] && s[1] <= outer[j][1]
    })
}
/// Whether two normalized lists share a point of positive length.
pub(crate) fn overlaps(a: &[[f64; 2]], b: &[[f64; 2]]) -> bool {
    !intersect(a, b).is_empty()
}

/// One ray's change in a delta.
#[derive(Clone, Debug, PartialEq)]
pub struct RayChange {
    pub axis: u8,
    pub index: u32,
    pub before: Ray,
    pub after: Ray,
}

/// A move's stock change: changed rays in (axis, index) order, and the
/// height view's matching transaction.
#[derive(Clone, Debug)]
pub struct DexelDelta {
    pub(crate) base_generation: u64,
    pub(crate) motion_id: u64,
    pub(crate) operation: String,
    pub(crate) fraction: f64,
    pub(crate) changes: Vec<RayChange>,
    pub(crate) view: StockDelta,
    pub(crate) stats: DexelSweepStats,
}
impl DexelDelta {
    pub fn changes(&self) -> &[RayChange] {
        &self.changes
    }
    pub fn stats(&self) -> &DexelSweepStats {
        &self.stats
    }
    pub fn fraction(&self) -> f64 {
        self.fraction
    }
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct DexelVolume {
    /// Per grid (X, Y, Z): [certain, possible] remaining material.
    pub grids_mm3: [[f64; 2]; 3],
    pub remaining_lower_mm3: f64,
    pub remaining_upper_mm3: f64,
    pub removed_lower_mm3: f64,
    pub removed_upper_mm3: f64,
}

#[derive(Clone, Debug)]
pub struct DexelStock {
    spec: DexelStockSpec,
    grid: DexelGrid,
    rays: [Vec<Ray>; 3],
    intervals: usize,
    generation: u64,
    view: Stock,
}

impl DexelStock {
    pub fn new(spec: DexelStockSpec) -> Result<Self, String> {
        let grid = spec.grid()?;
        crate::resources::check_memory(
            grid.total_rays() as u128 * std::mem::size_of::<Ray>() as u128
                + grid.rays(2) as u128 * std::mem::size_of::<HeightBounds>() as u128,
            "dexel stock",
        )?;
        let rays =
            std::array::from_fn(|a| vec![Ray::full(spec.min_mm[a], spec.max_mm[a]); grid.rays(a)]);
        let view = Stock::new(spec.view_spec()?)?;
        Ok(Self {
            intervals: 2 * grid.total_rays(),
            spec,
            grid,
            rays,
            generation: 0,
            view,
        })
    }
    pub fn spec(&self) -> &DexelStockSpec {
        &self.spec
    }
    pub fn grid(&self) -> DexelGrid {
        self.grid
    }
    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn rays(&self, axis: usize) -> &[Ray] {
        &self.rays[axis]
    }
    pub fn intervals(&self) -> usize {
        self.intervals
    }
    /// The height-field view (planners, height-field checks).
    pub fn view(&self) -> &Stock {
        &self.view
    }
    pub(crate) fn view_mut(&mut self) -> &mut Stock {
        &mut self.view
    }
    /// The centre of a ray's cross-section: its coordinates on the two
    /// cross axes (`DexelGrid::cross`).
    pub fn ray_center(&self, axis: usize, index: usize) -> [f64; 2] {
        let [u, v] = DexelGrid::cross(axis);
        let (iu, iv) = (index % self.grid.cells[u], index / self.grid.cells[u]);
        [
            self.spec.min_mm[u] + (iu as f64 + 0.5) * self.grid.step_mm[u],
            self.spec.min_mm[v] + (iv as f64 + 0.5) * self.grid.step_mm[v],
        ]
    }
    /// The boundary between cells `i - 1` and `i` along axis `w`: the
    /// box's own faces at both ends, exactly.
    pub fn cell_bound(&self, w: usize, i: usize) -> f64 {
        if i == 0 {
            self.spec.min_mm[w]
        } else if i >= self.grid.cells[w] {
            self.spec.max_mm[w]
        } else {
            self.spec.min_mm[w] + i as f64 * self.grid.step_mm[w]
        }
    }
    pub fn volume(&self) -> DexelVolume {
        let mut grids_mm3 = [[0.; 2]; 3];
        for (a, g) in grids_mm3.iter_mut().enumerate() {
            let [u, v] = DexelGrid::cross(a);
            let area = self.grid.step_mm[u] * self.grid.step_mm[v];
            for r in &self.rays[a] {
                g[0] += r.certain.length() * area;
                g[1] += r.possible.length() * area;
            }
        }
        let total: f64 = (0..3)
            .map(|k| self.spec.max_mm[k] - self.spec.min_mm[k])
            .product();
        let lower = grids_mm3.iter().map(|g| g[0]).fold(0., f64::max);
        let upper = grids_mm3.iter().map(|g| g[1]).fold(f64::INFINITY, f64::min);
        DexelVolume {
            grids_mm3,
            remaining_lower_mm3: lower,
            remaining_upper_mm3: upper,
            removed_lower_mm3: (total - upper).max(0.),
            removed_upper_mm3: (total - lower).max(0.),
        }
    }

    /// Validate a delta against this stock before anything changes: each
    /// ray's old value is this stock's, its lists are valid and only lose
    /// material, and the interval budget holds. Returns the view's changes
    /// (the projection of the changed Z rays).
    fn validate(&self, changes: &[RayChange]) -> Result<(Vec<CellChange>, usize), String> {
        let mut last: Option<(u8, u32)> = None;
        let mut intervals = self.intervals;
        let mut view = vec![];
        for c in changes {
            let axis = c.axis as usize;
            let index = c.index as usize;
            if axis > 2
                || last.is_some_and(|l| (c.axis, c.index) <= l)
                || index >= self.rays[axis].len()
                || self.rays[axis][index] != c.before
                || !c
                    .after
                    .valid(self.spec.min_mm[axis], self.spec.max_mm[axis])
                || !contains(c.before.possible.as_slice(), c.after.possible.as_slice())
                || !contains(c.before.certain.as_slice(), c.after.certain.as_slice())
                || c.after == c.before
            {
                return Err(
                    "dexel delta: invalid ray order or index, old value mismatch, or a ray that gains material"
                        .into(),
                );
            }
            last = Some((c.axis, c.index));
            intervals = intervals - c.before.intervals() + c.after.intervals();
            if axis == 2 {
                let bottom = self.spec.min_mm[2];
                let (before, after) = (c.before.height(bottom), c.after.height(bottom));
                if before != after {
                    view.push(CellChange {
                        index: c.index,
                        before,
                        after,
                    });
                }
            }
        }
        if intervals > self.spec.max_intervals {
            return Err(format!(
                "dexel stock would hold {intervals} intervals, more than max_intervals {}; increase max_intervals",
                self.spec.max_intervals
            ));
        }
        Ok((view, intervals))
    }

    /// Commit a delta prepared by this stock's own sweep (native height-view
    /// records), checking it like a received one.
    pub fn apply_delta(&mut self, delta: &DexelDelta) -> Result<(), String> {
        if delta.base_generation != self.generation {
            return Err("dexel delta generation mismatch".into());
        }
        let (view, intervals) = self.validate(&delta.changes)?;
        if view != delta.view.changes {
            return Err("dexel delta height view does not match its Z rays".into());
        }
        self.view.apply_delta(&delta.view)?;
        self.commit_rays(&delta.changes, intervals)
    }
    /// Commit received ray changes (a decoded frame): the view follows from
    /// the Z rays, with no native records.
    pub fn apply_changes(
        &mut self,
        motion: &Motion,
        fraction: f64,
        changes: &[RayChange],
    ) -> Result<(), String> {
        let (view, intervals) = self.validate(changes)?;
        let delta = self.view.wire_delta(motion, fraction, view);
        self.view.apply_delta(&delta)?;
        self.commit_rays(changes, intervals)
    }
    fn commit_rays(&mut self, changes: &[RayChange], intervals: usize) -> Result<(), String> {
        for c in changes {
            self.rays[c.axis as usize][c.index as usize] = c.after.clone();
        }
        self.intervals = intervals;
        self.generation = self
            .generation
            .checked_add(1)
            .ok_or("dexel stock generation overflow")?;
        Ok(())
    }

    /// Restore from checkpoint lists (validated, removal-only against the
    /// uncut box), at `generation`.
    pub fn from_parts(
        spec: DexelStockSpec,
        generation: u64,
        rays: [Vec<Ray>; 3],
    ) -> Result<Self, String> {
        let grid = spec.grid()?;
        let mut intervals = 0usize;
        for (a, list) in rays.iter().enumerate() {
            if list.len() != grid.rays(a) {
                return Err("dexel checkpoint ray count mismatch".into());
            }
            for r in list {
                if !r.valid(spec.min_mm[a], spec.max_mm[a]) {
                    return Err("dexel checkpoint holds an invalid ray".into());
                }
                intervals += r.intervals();
            }
        }
        if intervals > spec.max_intervals {
            return Err("dexel checkpoint exceeds max_intervals".into());
        }
        let view_spec = spec.view_spec()?;
        let bottom = spec.min_mm[2];
        let view = Stock::from_snapshot(StockSnapshot {
            spec: view_spec,
            generation,
            cells: rays[2].iter().map(|r| r.height(bottom)).collect(),
        })?;
        Ok(Self {
            spec,
            grid,
            rays,
            intervals,
            generation,
            view,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interval_algebra() {
        let a = normalize(vec![[5., 6.], [0., 2.], [1., 3.]]);
        assert_eq!(a, vec![[0., 3.], [5., 6.]]);
        assert_eq!(
            subtract(&a, &[[1., 2.]]),
            vec![[0., 1.], [2., 3.], [5., 6.]]
        );
        assert_eq!(subtract(&a, &[[-1., 10.]]), Vec::<[f64; 2]>::new());
        assert_eq!(subtract(&a, &[[2.5, 5.5]]), vec![[0., 2.5], [5.5, 6.]]);
        assert_eq!(intersect(&a, &[[2., 5.5]]), vec![[2., 3.], [5., 5.5]]);
        assert!(contains(&a, &[[0.5, 1.], [5., 6.]]));
        assert!(!contains(&a, &[[2., 4.]]));
        assert!(overlaps(&a, &[[2.9, 4.]]));
        assert!(!overlaps(&a, &[[3., 5.]]));
    }

    #[test]
    fn spec_budgets_and_view() {
        let spec = DexelStockSpec {
            version: 1,
            units: "mm".into(),
            model: DexelModel::TriDexel,
            min_mm: [0., 0., 0.],
            max_mm: [10., 5., 2.],
            cell_size_mm: 1.,
            max_rays: 1000,
            max_intervals: 1000,
        };
        let grid = spec.grid().unwrap();
        assert_eq!(grid.cells, [10, 5, 2]);
        assert_eq!(grid.total_rays(), 50 + 10 + 20);
        let stock = DexelStock::new(spec.clone()).unwrap();
        let v = stock.volume();
        assert_eq!(v.remaining_lower_mm3, 100.);
        assert_eq!(v.remaining_upper_mm3, 100.);
        assert_eq!(stock.view().cells().len(), 50);
        assert!(
            DexelStockSpec {
                max_rays: 79,
                ..spec.clone()
            }
            .grid()
            .unwrap_err()
            .contains("max_rays")
        );
        assert!(
            DexelStockSpec {
                max_intervals: 159,
                ..spec
            }
            .grid()
            .unwrap_err()
            .contains("max_intervals")
        );
    }

    #[test]
    fn ray_validity_and_height_projection() {
        let r = Ray {
            possible: Spans::from_vec(vec![[0., 1.], [2., 5.]]),
            certain: Spans::from_vec(vec![[0., 0.5], [3., 4.]]),
        };
        assert!(r.valid(0., 5.));
        assert_eq!(
            r.height(0.),
            HeightBounds {
                lower_mm: 0.5,
                upper_mm: 5.
            }
        );
        let bad = Ray {
            possible: Spans::One([0., 1.]),
            certain: Spans::One([0., 2.]),
        };
        assert!(!bad.valid(0., 5.));
        let unsorted = Ray {
            possible: Spans::Many(vec![[2., 3.], [0., 1.]].into_boxed_slice()),
            certain: Spans::Empty,
        };
        assert!(!unsorted.valid(0., 5.));
        let one_in_many = Ray {
            possible: Spans::Many(vec![[0., 1.]].into_boxed_slice()),
            certain: Spans::Empty,
        };
        assert!(!one_in_many.valid(0., 5.), "one interval is stored inline");
    }
}
