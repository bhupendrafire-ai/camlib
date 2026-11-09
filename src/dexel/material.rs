//! Tri-dexel stock against a closed triangle mesh of the part (a solid
//! target with a stated error): leftover material (residual) and over-cut
//! (gouge), including under overhangs, which a height field cannot compare.
//!
//! Per ray, the part's intervals come from the mesh: the centre line's
//! crossings give where the mesh is inside (parity, exact orientation
//! predicates; a line through an edge or vertex leaves that ray undecided),
//! and the band where the ray's prism, grown by the mesh error, meets a
//! facet is where the part's boundary may lie. Outside that band the whole
//! cross-section is inside the part or outside it.
//!
//! Tolerance is a 3D distance into (gouge) or away from (residual) the part.
//! A point's distance to the part's boundary never exceeds its distance along
//! a ray, so excess within tolerance along the rays of one grid proves the
//! check within tolerance. A definite finding needs a witness: a point
//! certainly cut (or certainly left) whose exact distance to the mesh, less
//! the mesh error, exceeds the tolerance.
use super::{DexelGrid, DexelStock, intersect, normalize, subtract};
use crate::{motion::Vec3, stock::CancelSignal};
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SolidAssessment {
    WithinTolerance,
    DefiniteExcess,
    PossibleExcess,
}

#[derive(Clone, Debug, Serialize)]
pub struct SolidExcess {
    pub assessment: SolidAssessment,
    /// Raw excess volume bounds [certain, possible] (no tolerance taken
    /// off): the largest certain and the smallest possible of the grids.
    pub volume_mm3: [f64; 2],
    /// Cells (one per X, Y and Z ray crossing) where every grid allows
    /// excess beyond the tolerance: the check is within tolerance when
    /// there are none.
    pub possible_cells: u64,
    /// The centre of one such cell, when there is one.
    pub possible_cell_mm: Option<Vec3>,
    /// A point certainly in excess beyond the tolerance, when found.
    pub witness_mm: Option<Vec3>,
    pub witness_depth_mm: Option<f64>,
}

#[derive(Clone, Debug, Serialize)]
pub struct SolidReport {
    pub tolerance_mm: f64,
    pub surface_error_mm: f64,
    pub residual: SolidExcess,
    pub gouge: SolidExcess,
    /// Rays left undecided by a centre line through a mesh edge or vertex.
    pub undecided_rays: u64,
}

const MAX_WITNESS_TESTS: usize = 256;

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
/// Exact distance from `p` to triangle `t` (Ericson, Real-Time Collision
/// Detection, 5.1.5: closest point on a triangle).
fn triangle_distance(p: Vec3, t: &[Vec3; 3]) -> f64 {
    let [a, b, c] = *t;
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    let at = |q: Vec3| {
        let d = sub(p, q);
        dot(d, d).sqrt()
    };
    let lerp = |x: Vec3, y: Vec3, s: f64| [x[0] + s * y[0], x[1] + s * y[1], x[2] + s * y[2]];
    if d1 <= 0. && d2 <= 0. {
        return at(a);
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0. && d4 <= d3 {
        return at(b);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0. && d1 >= 0. && d3 <= 0. {
        return at(lerp(a, ab, d1 / (d1 - d3)));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0. && d5 <= d6 {
        return at(c);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0. && d2 >= 0. && d6 <= 0. {
        return at(lerp(a, ac, d2 / (d2 - d6)));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0. && (d4 - d3) >= 0. && (d5 - d6) >= 0. {
        return at(lerp(b, sub(c, b), (d4 - d3) / ((d4 - d3) + (d5 - d6))));
    }
    let denom = 1. / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    at([
        a[0] + ab[0] * v + ac[0] * w,
        a[1] + ab[1] * v + ac[1] * w,
        a[2] + ab[2] * v + ac[2] * w,
    ])
}

/// Extent along `a` of a triangle clipped to the box `b` = [u0, u1, v0, v1]
/// across the ray.
fn clipped_extent(t: &[Vec3; 3], a: usize, u: usize, v: usize, b: [f64; 4]) -> Option<[f64; 2]> {
    let mut pts: Vec<Vec3> = t.to_vec();
    let sides = [
        (u, b[0], true),
        (u, b[1], false),
        (v, b[2], true),
        (v, b[3], false),
    ];
    for (axis, bound, above) in sides {
        let inside = |p: &Vec3| {
            if above {
                p[axis] >= bound
            } else {
                p[axis] <= bound
            }
        };
        let mut out = Vec::with_capacity(pts.len() + 2);
        for i in 0..pts.len() {
            let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
            if inside(&p) {
                out.push(p);
            }
            if inside(&p) != inside(&q) {
                let s = (bound - p[axis]) / (q[axis] - p[axis]);
                out.push(std::array::from_fn(|k| p[k] + s * (q[k] - p[k])));
            }
        }
        pts = out;
        if pts.is_empty() {
            return None;
        }
    }
    // Contact only on the box's boundary plane (zero width there) meets no
    // interior point of the cross-section.
    for (axis, bound, _) in sides {
        if pts.iter().all(|p| p[axis] == bound) {
            return None;
        }
    }
    let lo = pts.iter().map(|p| p[a]).fold(f64::INFINITY, f64::min);
    let hi = pts.iter().map(|p| p[a]).fold(f64::NEG_INFINITY, f64::max);
    Some([lo, hi])
}

/// (certainly inside, possibly inside) along a ray.
type PartIntervals = (Vec<[f64; 2]>, Vec<[f64; 2]>);

/// The part along one ray: (certainly inside, possibly inside), or None
/// when the centre line passes through an edge or vertex.
fn part_intervals(
    triangles: &[[Vec3; 3]],
    candidates: &[u32],
    a: usize,
    b: [f64; 4],
    error: f64,
    guard: f64,
) -> Result<Option<PartIntervals>, String> {
    let c = [(b[0] + b[1]) / 2., (b[2] + b[3]) / 2.];
    let h = [(b[1] - b[0]) / 2., (b[3] - b[2]) / 2.];
    let [u, v] = DexelGrid::cross(a);
    let mut band = vec![];
    for &i in candidates {
        let grown = [b[0] - error, b[1] + error, b[2] - error, b[3] + error];
        if let Some(e) = clipped_extent(&triangles[i as usize], a, u, v, grown) {
            band.push([e[0] - error - guard, e[1] + error + guard]);
        }
    }
    // Outside the band the whole cross-section is inside or outside, so any
    // line through it classifies the gaps: the centre line, or offset lines
    // when one passes through a facet's edge or vertex.
    let mut crossings = None;
    'lines: for [du, dv] in [
        [0., 0.],
        [0.3137, 0.2718],
        [-0.2236, 0.4142],
        [0.1732, -0.3606],
    ] {
        let q = robust::Coord {
            x: c[0] + du * h[0],
            y: c[1] + dv * h[1],
        };
        let mut found = vec![];
        for &i in candidates {
            let t = &triangles[i as usize];
            let p = |k: usize| robust::Coord {
                x: t[k][u],
                y: t[k][v],
            };
            let area = robust::orient2d(p(0), p(1), p(2));
            if area == 0. {
                continue; // a facet along the ray: its edges are its neighbours'
            }
            let o = [
                robust::orient2d(p(0), p(1), q),
                robust::orient2d(p(1), p(2), q),
                robust::orient2d(p(2), p(0), q),
            ];
            let sign = area.signum();
            if o.contains(&0.) && o.iter().all(|x| *x * sign >= 0.) {
                continue 'lines;
            }
            if o.iter().all(|x| *x * sign > 0.) {
                // Barycentric weights from the orientations (o[1] weighs the
                // first vertex): the crossing's position along the ray.
                let w = o.map(|x| x / (o[0] + o[1] + o[2]));
                found.push(w[1] * t[0][a] + w[2] * t[1][a] + w[0] * t[2][a]);
            }
        }
        crossings = Some(found);
        break;
    }
    let Some(mut crossings) = crossings else {
        return Ok(None);
    };
    if !crossings.len().is_multiple_of(2) {
        return Err(
            "the target mesh is not closed: a ray crosses its surface an odd number of times"
                .into(),
        );
    }
    crossings.sort_by(f64::total_cmp);
    let inside: Vec<[f64; 2]> = crossings.chunks(2).map(|p| [p[0], p[1]]).collect();
    let band = normalize(band);
    let certain = subtract(&inside, &band);
    let possible = normalize(inside.into_iter().chain(band).collect());
    Ok(Some((certain, possible)))
}

/// A point of the box `b` (per axis [lo, hi]) not proven within `reach`
/// of the facets `near`, after bisecting the box up to `depth` times along
/// its longest side; None when every part of it is proven within reach.
fn open_point(
    b: [[f64; 2]; 3],
    triangles: &[[Vec3; 3]],
    near: &[u32],
    reach: f64,
    depth: u32,
) -> Option<Vec3> {
    let c = b.map(|s| (s[0] + s[1]) / 2.);
    let half = b.iter().map(|s| (s[1] - s[0]).powi(2)).sum::<f64>().sqrt() / 2.;
    let d = near
        .iter()
        .map(|t| triangle_distance(c, &triangles[*t as usize]))
        .fold(f64::INFINITY, f64::min);
    if d + half <= reach {
        return None;
    }
    if depth == 0 || d - half > reach {
        return Some(c);
    }
    let k = (0..3)
        .max_by(|x, y| (b[*x][1] - b[*x][0]).total_cmp(&(b[*y][1] - b[*y][0])))
        .unwrap();
    let m = c[k];
    let (mut lo, mut hi) = (b, b);
    lo[k][1] = m;
    hi[k][0] = m;
    open_point(lo, triangles, near, reach, depth - 1)
        .or_else(|| open_point(hi, triangles, near, reach, depth - 1))
}

fn shrink(spans: &[[f64; 2]], by: f64) -> Vec<[f64; 2]> {
    spans
        .iter()
        .map(|s| [s[0] + by, s[1] - by])
        .filter(|s| s[0] < s[1])
        .collect()
}
fn length(spans: &[[f64; 2]]) -> f64 {
    spans.iter().map(|s| s[1] - s[0]).sum()
}

struct Excess {
    volume: [[f64; 2]; 3],
    /// Per grid, per ray: where excess beyond the tolerance is possible.
    suspects: [std::collections::HashMap<usize, Vec<[f64; 2]>>; 3],
    /// Candidate witnesses: certain excess midpoints, longest first.
    witnesses: Vec<(f64, Vec3)>,
}
impl Excess {
    fn new() -> Self {
        Self {
            volume: [[0.; 2]; 3],
            suspects: Default::default(),
            witnesses: vec![],
        }
    }
    /// Cells where all three grids allow excess beyond the tolerance. Each
    /// cell lies on exactly one ray of each grid, and a point in excess lies
    /// in every grid's suspect set, so within a cell excess can only lie in
    /// the boxes (X interval) x (Y interval) x (Z interval) of the three
    /// rays' suspects. A box whose every point lies within the tolerance of
    /// the part's surface holds none: its centre's distance to nearby facets
    /// (an upper bound of the distance to the mesh) plus its half-diagonal
    /// plus the mesh error is at most the tolerance.
    #[allow(clippy::too_many_arguments)]
    fn possible_cells(
        &self,
        stock: &DexelStock,
        triangles: &[[Vec3; 3]],
        near: &[Vec<u32>],
        error: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<(u64, Option<Vec3>), String> {
        let g = stock.grid;
        let span = |w: usize, i: usize| [stock.cell_bound(w, i), stock.cell_bound(w, i + 1)];
        let within = |a: usize, index: usize, w: usize, i: usize| {
            self.suspects[a]
                .get(&index)
                .map(|s| intersect(s, &[span(w, i)]))
                .unwrap_or_default()
        };
        let mut count = 0u64;
        let mut first: Option<Vec3> = None;
        let mut keys: Vec<_> = self.suspects[2].keys().copied().collect();
        keys.sort_unstable();
        for index in keys {
            if cancel.is_cancelled() {
                return Err("material comparison cancelled".into());
            }
            let (i, j) = (index % g.cells[0], index / g.cells[0]);
            for k in 0..g.cells[2] {
                let zs = within(2, index, 2, k);
                if zs.is_empty() {
                    continue;
                }
                // X rays: (u, v) = (Y, Z); Y rays: (u, v) = (Z, X).
                let xs = within(0, k * g.cells[1] + j, 0, i);
                let ys = within(1, i * g.cells[2] + k, 1, j);
                let mut open = None;
                'boxes: for x in &xs {
                    for y in &ys {
                        for z in &zs {
                            if let Some(c) = open_point(
                                [*x, *y, *z],
                                triangles,
                                &near[index],
                                tolerance - error,
                                12,
                            ) {
                                open = Some(c);
                                break 'boxes;
                            }
                        }
                    }
                }
                if let Some(c) = open {
                    count += 1;
                    first.get_or_insert(c);
                }
            }
        }
        Ok((count, first))
    }
    #[allow(clippy::too_many_arguments)]
    fn finish(
        mut self,
        stock: &DexelStock,
        triangles: &[[Vec3; 3]],
        near: &[Vec<u32>],
        error: f64,
        tolerance: f64,
        cancel: &impl CancelSignal,
    ) -> Result<SolidExcess, String> {
        let volume = [
            self.volume.iter().map(|g| g[0]).fold(0., f64::max),
            self.volume
                .iter()
                .map(|g| g[1])
                .fold(f64::INFINITY, f64::min),
        ];
        let (possible_cells, possible_cell_mm) =
            self.possible_cells(stock, triangles, near, error, tolerance, cancel)?;
        let mut out = SolidExcess {
            assessment: SolidAssessment::WithinTolerance,
            volume_mm3: volume,
            possible_cells,
            possible_cell_mm,
            witness_mm: None,
            witness_depth_mm: None,
        };
        if possible_cells == 0 {
            return Ok(out);
        }
        out.assessment = SolidAssessment::PossibleExcess;
        self.witnesses.sort_by(|x, y| y.0.total_cmp(&x.0));
        for (_, p) in self.witnesses.iter().take(MAX_WITNESS_TESTS) {
            if cancel.is_cancelled() {
                return Err("material comparison cancelled".into());
            }
            let d = triangles
                .iter()
                .map(|t| triangle_distance(*p, t))
                .fold(f64::INFINITY, f64::min);
            if d - error > tolerance {
                out.assessment = SolidAssessment::DefiniteExcess;
                out.witness_mm = Some(*p);
                out.witness_depth_mm = Some(d - error);
                break;
            }
        }
        Ok(out)
    }
}

impl DexelStock {
    /// Compare this stock with a closed triangle mesh of the part whose true
    /// surface lies within `surface_error_mm` of the mesh. `tolerance_mm` is
    /// the allowed distance of leftover material from the part and of an
    /// over-cut into it.
    pub fn compare_solid(
        &self,
        triangles: &[[Vec3; 3]],
        surface_error_mm: f64,
        tolerance_mm: f64,
        cancel: &impl CancelSignal,
    ) -> Result<SolidReport, String> {
        if triangles.is_empty()
            || triangles.iter().flatten().flatten().any(|x| !x.is_finite())
            || !surface_error_mm.is_finite()
            || !(0.0..=0.1).contains(&surface_error_mm)
            || !tolerance_mm.is_finite()
            || !(1e-7..=1.).contains(&tolerance_mm)
        {
            return Err("a solid target needs finite facets, a surface error in [0, 0.1] mm and a tolerance in [1e-7, 1] mm".into());
        }
        let magnitude = triangles
            .iter()
            .flatten()
            .flatten()
            .chain(self.spec.min_mm.iter())
            .chain(self.spec.max_mm.iter())
            .map(|x| x.abs())
            .fold(1., f64::max);
        let guard = 1e-9_f64.max(512. * f64::EPSILON * magnitude);
        let (mut gouge, mut residual) = (Excess::new(), Excess::new());
        let mut undecided = 0u64;
        let mut near_z = vec![];
        for a in 0..3 {
            let [u, v] = DexelGrid::cross(a);
            let (nu, nv) = (self.grid.cells[u], self.grid.cells[v]);
            let (su, sv) = (self.grid.step_mm[u], self.grid.step_mm[v]);
            let (lo, hi) = (self.spec.min_mm[a], self.spec.max_mm[a]);
            let area = su * sv;
            // Facets binned by the rays their grown footprint can reach.
            let mut bins: Vec<Vec<u32>> = vec![vec![]; nu * nv];
            for (i, t) in triangles.iter().enumerate() {
                let range = |w: usize, n: usize, s: f64| {
                    let (min, max) = t.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |m, p| {
                        (m.0.min(p[w]), m.1.max(p[w]))
                    });
                    let cell = |x: f64| ((x - self.spec.min_mm[w]) / s).floor();
                    let (i0, i1) = (
                        cell(min - surface_error_mm - guard),
                        cell(max + surface_error_mm + guard),
                    );
                    (i1 >= 0. && i0 < n as f64)
                        .then(|| (i0.max(0.) as usize, (i1 as usize).min(n - 1)))
                };
                if let (Some((u0, u1)), Some((v0, v1))) = (range(u, nu, su), range(v, nv, sv)) {
                    for iv in v0..=v1 {
                        for iu in u0..=u1 {
                            bins[iv * nu + iu].push(i as u32);
                        }
                    }
                }
            }
            if a == 2 {
                near_z = bins.clone();
            }
            for (index, ray) in self.rays[a].iter().enumerate() {
                if index.is_multiple_of(1024) && cancel.is_cancelled() {
                    return Err("material comparison cancelled".into());
                }
                let c = self.ray_center(a, index);
                let (iu, iv) = (index % nu, index / nu);
                let b = [
                    self.cell_bound(u, iu),
                    self.cell_bound(u, iu + 1),
                    self.cell_bound(v, iv),
                    self.cell_bound(v, iv + 1),
                ];
                let Some((inside, possible_inside)) =
                    part_intervals(triangles, &bins[index], a, b, surface_error_mm, guard)?
                else {
                    undecided += 1;
                    gouge.suspects[a].insert(index, vec![[lo, hi]]);
                    residual.suspects[a].insert(index, vec![[lo, hi]]);
                    continue;
                };
                let whole = [[lo, hi]];
                let (p, cm) = (ray.possible.as_slice(), ray.certain.as_slice());
                let not_certain = subtract(&whole, cm);
                let empty = subtract(&whole, p);
                let outside = subtract(&whole, &inside);
                let possible_outside = subtract(&whole, &possible_inside);
                // Over-cut: material possibly removed inside the part.
                let g_possible = intersect(&not_certain, &possible_inside);
                let g_certain = intersect(&empty, &inside);
                gouge.volume[a][0] += length(&g_certain) * area;
                gouge.volume[a][1] += length(&g_possible) * area;
                let deep = intersect(
                    &not_certain,
                    &intersect(&shrink(&possible_inside, tolerance_mm), &whole),
                );
                if !deep.is_empty() {
                    gouge.suspects[a].insert(index, deep);
                }
                // Leftover: material possibly present outside the part.
                let r_possible = intersect(p, &outside);
                let r_certain = intersect(cm, &possible_outside);
                residual.volume[a][0] += length(&r_certain) * area;
                residual.volume[a][1] += length(&r_possible) * area;
                let far_outside = shrink(
                    &subtract(&[[f64::NEG_INFINITY, f64::INFINITY]], &inside),
                    tolerance_mm,
                );
                let far = intersect(p, &intersect(&far_outside, &whole));
                if !far.is_empty() {
                    residual.suspects[a].insert(index, far);
                }
                for (list, excess) in [(&g_certain, &mut gouge), (&r_certain, &mut residual)] {
                    for s in list {
                        if s[1] - s[0] > 2. * tolerance_mm
                            && excess.witnesses.len() < 4 * MAX_WITNESS_TESTS
                        {
                            let mut q = [0.; 3];
                            q[a] = (s[0] + s[1]) / 2.;
                            q[u] = c[0];
                            q[v] = c[1];
                            excess.witnesses.push((s[1] - s[0], q));
                        }
                    }
                }
            }
        }
        Ok(SolidReport {
            tolerance_mm,
            surface_error_mm,
            gouge: gouge.finish(
                self,
                triangles,
                &near_z,
                surface_error_mm,
                tolerance_mm,
                cancel,
            )?,
            residual: residual.finish(
                self,
                triangles,
                &near_z,
                surface_error_mm,
                tolerance_mm,
                cancel,
            )?,
            undecided_rays: undecided,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_triangle_distance() {
        let t = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]];
        assert_eq!(triangle_distance([0.2, 0.2, 3.], &t), 3.);
        assert_eq!(triangle_distance([-1., 0., 0.], &t), 1.);
        assert!((triangle_distance([1., 1., 0.], &t) - 0.5f64.sqrt()).abs() < 1e-12);
    }
}
