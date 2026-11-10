//! Deburr's convex-edge list (#94): every convex edge of the part, as the
//! CAD adapter lists it (`camlib.cadquery.deburr_edges`), is accounted for in
//! the report, cut or skipped with a reason. Straight edges between two
//! planar faces that no level covers (sloped edges, and horizontal edges
//! under a sloped face) are broken with a ball-end mill.
//!
//! A ball of radius R whose centre runs on the line E(t) + s b, b the unit
//! outward bisector of the faces' normals n1, n2 (cos g = b . n1), removes
//! exactly the points within R of that line: a cylindrical cove with equal
//! legs L = sqrt(R^2 - s^2 cos^2 g) - s sin g along both faces, whatever the
//! edge's slope. `width_mm` gives L (s = sqrt(R^2 - L^2 cos^2 g) - L sin g),
//! `break_radius_mm` the fillet rho to stay inside (the fillet's centre is
//! rho / cos g from the edge on the same line, so tangency gives
//! s = R + rho - rho / cos g). With cos g = 1 / sqrt(2) these are the
//! horizontal edge's formulas.
//!
//! Proof: the part's mesh with the two faces' strips along the edge (where
//! the cove cuts them) replaced by the cove itself, of radius R + grow +
//! band, so the grown tool may touch nothing there; every other triangle
//! stays an obstacle. The whole assembly (ball, shank, holder) is proven
//! clear of that, so the tool removes only material inside the cove. A
//! straight sweep of the ball removes the cove exactly; the shank above it
//! is what the proof is for (on a steep edge it can reach the faces uphill).
use super::{UNPROVEN, clip_half, stroke_distance};
use crate::{
    motion::{Curve, Vec3},
    pocket::Stroke,
};
use serde::{Deserialize, Serialize};

/// One convex edge of the part (from the CAD model).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DeburrEdge {
    pub id: String,
    /// "horizontal", "vertical" or "sloped".
    pub kind: String,
    /// Points along the edge, in order: a polyline within `error_mm` of it
    /// (the two ends of a straight edge).
    pub points: Vec<Vec3>,
    pub error_mm: f64,
    /// A straight edge (a line).
    pub straight: bool,
    /// The two faces' outward unit normals at the edge's midpoint.
    pub normals: [Vec3; 2],
    /// Whether each face is planar.
    pub planar: [bool; 2],
}

#[derive(Clone, Debug, Serialize)]
pub struct DeburrEdgeReport {
    pub id: String,
    pub kind: String,
    pub length_mm: f64,
    /// "cut", "partly cut" or "skipped".
    pub status: &'static str,
    pub cut_length_mm: f64,
    /// Why the rest was not cut (empty when all of it was).
    pub reasons: Vec<&'static str>,
}

pub(super) const VERTICAL: &str =
    "vertical edge: breaking it needs the tool's side run along the edge (not supported yet)";
pub(super) const CURVED: &str = "curved edge not on an upward-facing planar face, or a face next to it is not planar: not supported yet (straight edges between planar faces only)";
pub(super) const DOWN: &str = "a face next to the edge faces down (or the edge's outward bisector does): a vertical tool cannot break it (needs a tilted tool, #5/#6)";
pub(super) const TOOL: &str = "an edge not on an upward-facing planar face is broken with a ball-end mill only (a chamfer or bull-nose end swept along it leaves no fixed break)";
pub(super) const SIZE: &str =
    "the requested break does not fit the ball on this edge (its faces' angle)";
pub(super) const TIP: &str =
    "the break on this edge would be cut within tip_clearance_mm of the ball's tip";

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|k| a[k] - b[k])
}
fn add(a: Vec3, b: Vec3) -> Vec3 {
    std::array::from_fn(|k| a[k] + b[k])
}
fn scale(a: Vec3, s: f64) -> Vec3 {
    a.map(|v| v * s)
}
fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn norm(a: Vec3) -> f64 {
    dot(a, a).sqrt()
}
fn unit(a: Vec3) -> Vec3 {
    scale(a, 1. / norm(a))
}

pub(super) fn validate(edges: &[DeburrEdge]) -> Result<(), String> {
    for e in edges {
        let finite = e
            .points
            .iter()
            .flatten()
            .chain(e.normals.iter().flatten())
            .all(|v| v.is_finite());
        if !matches!(e.kind.as_str(), "horizontal" | "vertical" | "sloped")
            || e.points.len() < 2
            || !finite
            || !(0. ..=0.1).contains(&e.error_mm)
            || e.normals.iter().any(|n| (norm(*n) - 1.).abs() > 1e-6)
        {
            return Err(format!(
                "deburr edge {:?}: needs kind horizontal, vertical or sloped, at least two finite points, error_mm within 0..0.1 and unit face normals",
                e.id
            ));
        }
    }
    Ok(())
}

pub(super) fn polyline_length(points: &[Vec3]) -> f64 {
    points.windows(2).map(|w| norm(sub(w[1], w[0]))).sum()
}

/// A straight edge's ball break: the tool tip's path along the whole edge,
/// and the part with the cove in place of the faces' strips.
pub(super) struct StraightBreak {
    pub start: Vec3,
    pub end: Vec3,
    pub obstacles: Vec<[Vec3; 3]>,
}

/// The ball break of a straight edge between planar faces (Err: the reason
/// it is skipped).
#[allow(clippy::too_many_arguments)]
pub(super) fn straight_break(
    edge: &DeburrEdge,
    ball: f64,
    width: Option<f64>,
    radius: Option<f64>,
    tip_clearance: f64,
    triangles: &[[Vec3; 3]],
    surface_error: f64,
    cove_margin: f64,
    band: f64,
) -> Result<StraightBreak, &'static str> {
    let (e0, e1) = (edge.points[0], *edge.points.last().unwrap());
    let len = norm(sub(e1, e0));
    if !edge.straight || !edge.planar[0] || !edge.planar[1] || len <= 0. {
        return Err(CURVED);
    }
    let v = unit(sub(e1, e0));
    let [n1, n2] = edge.normals;
    if dot(n1, v).abs() > 1e-6 || dot(n2, v).abs() > 1e-6 || n1[2] < 0. || n2[2] < 0. {
        return Err(DOWN);
    }
    let sum = add(n1, n2);
    if norm(sum) < 1e-9 {
        return Err(CURVED);
    }
    let b = unit(sum);
    if b[2] <= 0. {
        return Err(DOWN);
    }
    let cos_g = dot(b, n1).clamp(-1., 1.);
    let sin_g = (1. - cos_g * cos_g).sqrt();
    let r = ball;
    let s = match (width, radius) {
        (Some(leg), None) => {
            let q = r * r - leg * leg * cos_g * cos_g;
            if q < 0. {
                return Err(SIZE);
            }
            q.sqrt() - leg * sin_g
        }
        (None, Some(rho)) => r + rho - rho / cos_g,
        _ => return Err(SIZE),
    };
    // The centre outside the corner, and the ball reaching both faces.
    if !(s >= 0. && s * cos_g < r) {
        return Err(SIZE);
    }
    // In-face directions away from the edge.
    let away = |n: Vec3| {
        let f = unit(cross(n, v));
        if dot(f, b) > 0. { scale(f, -1.) } else { f }
    };
    let (f1, f2) = (away(n1), away(n2));
    let leg_at = |rho: f64| (rho * rho - s * s * cos_g * cos_g).sqrt() - s * sin_g;
    // Tip clearance: the ball cuts along the great circle normal to its
    // travel; over the arc between the break's two ends (through -b), the
    // contact's distance from the tool axis must stay at least
    // tip_clearance.
    let k = cross(v, b);
    let centre = scale(b, s);
    let angle = |p: Vec3| {
        let d = sub(p, centre);
        dot(d, k).atan2(dot(d, b)).rem_euclid(std::f64::consts::TAU)
    };
    let leg = leg_at(r);
    let (mut a1, mut a2) = (angle(scale(f1, leg)), angle(scale(f2, leg)));
    if a1 > a2 {
        std::mem::swap(&mut a1, &mut a2);
    }
    if !(a1 < std::f64::consts::PI && std::f64::consts::PI < a2) {
        return Err(SIZE);
    }
    for i in 0..=2000 {
        let a = a1 + (a2 - a1) * i as f64 / 2000.;
        let w = add(scale(b, a.cos()), scale(k, a.sin()));
        if r * w[0].hypot(w[1]) < tip_clearance {
            return Err(TIP);
        }
    }
    // The cove for the proof: radius rho_c, its chords outside it, its two
    // ends on the face lines.
    let rho_c = r + cove_margin;
    let step = 2. * (rho_c / (rho_c + band)).acos();
    let rho_v = |n: usize| rho_c / ((a2 - a1) / n as f64 / 2.).cos();
    let mut n = ((a2 - a1) / step).ceil().max(1.) as usize;
    while rho_v(n) - rho_c > band {
        n += 1;
    }
    let rho_v = rho_v(n);
    let u = leg_at(rho_v);
    let (b1, b2) = (angle(scale(f1, u)), angle(scale(f2, u)));
    let (c1, c2) = (b1.min(b2), b1.max(b2));
    let arc: Vec<Vec3> = (0..=n)
        .map(|i| {
            let a = c1 + (c2 - c1) * i as f64 / n as f64;
            add(
                centre,
                scale(add(scale(b, a.cos()), scale(k, a.sin())), rho_v),
            )
        })
        .collect();
    let gap = surface_error + 1e-9;
    let on_face = |t: &[Vec3; 3], n: Vec3| t.iter().all(|p| dot(sub(*p, e0), n).abs() <= gap);
    // Keep: beyond either end of the edge, beyond the cove's end on the
    // face, or (on the plane) beyond the edge line itself. The cove covers
    // the rest: within the error of the edge line the centre line is
    // s < R away.
    let strip = |t: [Vec3; 3], f: Vec3, out: &mut Vec<[Vec3; 3]>| {
        let (t0, t1) = (dot(v, e0), dot(v, e0) + len);
        out.extend(clip_half(t, scale(v, -1.), -t0));
        out.extend(clip_half(t, v, t1));
        for p in clip_half(t, v, t0) {
            for q in clip_half(p, scale(v, -1.), -t1) {
                out.extend(clip_half(q, f, dot(f, e0) + u));
                out.extend(clip_half(q, scale(f, -1.), -(dot(f, e0) - gap)));
            }
        }
    };
    let mut obstacles = Vec::with_capacity(triangles.len() + 2 * n);
    for t in triangles {
        if on_face(t, n1) {
            strip(*t, f1, &mut obstacles);
        } else if on_face(t, n2) {
            strip(*t, f2, &mut obstacles);
        } else {
            obstacles.push(*t);
        }
    }
    for i in 0..n {
        let (p, q) = (add(e0, arc[i]), add(e0, arc[i + 1]));
        let (pp, qq) = (add(p, scale(v, len)), add(q, scale(v, len)));
        obstacles.push([p, q, qq]);
        obstacles.push([p, qq, pp]);
    }
    let tip = |e: Vec3| sub(add(e, centre), [0., 0., r]);
    Ok(StraightBreak {
        start: tip(e0),
        end: tip(e1),
        obstacles,
    })
}

/// "cut", "partly cut" or "skipped" from the cut length.
pub(super) fn status(cut: f64, total: f64, tolerance: f64) -> &'static str {
    if cut >= total - tolerance {
        "cut"
    } else if cut <= tolerance {
        "skipped"
    } else {
        "partly cut"
    }
}

/// XY distance from `q` to a path piece (a line or an XY arc).
fn piece_distance(q: [f64; 2], curve: &Curve) -> f64 {
    let xy = |p: Vec3| [p[0], p[1]];
    match *curve {
        Curve::Arc {
            start,
            end,
            center,
            sweep_rad,
            ..
        } => stroke_distance(
            q,
            &Stroke {
                a: xy(start),
                b: xy(end),
                arc: Some((xy(center), sweep_rad)),
            },
        ),
        Curve::Line { start, end } | Curve::Helix { start, end, .. } => {
            stroke_distance(q, &Stroke::line(xy(start), xy(end)))
        }
    }
}

/// The point at arc length `at` along a polyline.
fn point_at(points: &[Vec3], at: f64) -> Vec3 {
    let mut left = at;
    for w in points.windows(2) {
        let l = norm(sub(w[1], w[0]));
        if left <= l && l > 0. {
            return add(w[0], scale(sub(w[1], w[0]), left / l));
        }
        left -= l;
    }
    *points.last().unwrap()
}

/// A horizontal edge on a level's boundary, from that level's pieces: each
/// sample of the edge (every 0.1 mm, at least 16) takes the status of the
/// nearest path piece of the level (the path runs on the boundary's
/// outward offset, so the nearest piece is the one cutting there).
pub(super) fn account_level(
    edge: &DeburrEdge,
    total: f64,
    level: usize,
    record: &[(usize, Curve, Option<&'static str>)],
) -> DeburrEdgeReport {
    let pieces: Vec<_> = record.iter().filter(|r| r.0 == level).collect();
    let n = ((total / 0.1).ceil() as usize).max(16);
    let (mut cut, mut reasons) = (0usize, vec![]);
    for i in 0..n {
        let q = point_at(&edge.points, (i as f64 + 0.5) / n as f64 * total);
        let q = [q[0], q[1]];
        let nearest = pieces
            .iter()
            .min_by(|a, b| piece_distance(q, &a.1).total_cmp(&piece_distance(q, &b.1)));
        match nearest.map(|p| p.2) {
            Some(None) => cut += 1,
            Some(Some(reason)) => {
                if !reasons.contains(&reason) {
                    reasons.push(reason);
                }
            }
            None => {
                if !reasons.contains(&UNPROVEN) {
                    reasons.push(UNPROVEN);
                }
            }
        }
    }
    DeburrEdgeReport {
        id: edge.id.clone(),
        kind: edge.kind.clone(),
        length_mm: total,
        status: if cut == n {
            "cut"
        } else if cut == 0 {
            "skipped"
        } else {
            "partly cut"
        },
        cut_length_mm: total * cut as f64 / n as f64,
        reasons,
    }
}
