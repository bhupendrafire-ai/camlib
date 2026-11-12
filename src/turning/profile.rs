//! Turned part profiles: the half-section of the part above the spindle axis.
//!
//! Internally points are (u, v) = (z, r), the G18 frame of XZ arcs (angles
//! from +Z toward +X, positive = G3 = counter-clockwise in (u, v)). The
//! profile runs from the front face (on the axis) toward the chuck with the
//! part's interior on the left of travel.
use crate::motion::{Curve, Plane, Vec3};
use serde::{Deserialize, Serialize};

/// A profile point: radius, z and the bulge (tan of a quarter of the arc's
/// sweep, positive = G3) of the segment to the next point.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TurnProfile {
    pub points: Vec<[f64; 3]>,
}

/// A line or arc in (u, v) = (z, r).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Seg {
    Line {
        a: [f64; 2],
        b: [f64; 2],
    },
    Arc {
        a: [f64; 2],
        b: [f64; 2],
        c: [f64; 2],
        sweep: f64,
    },
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}
fn len(a: [f64; 2]) -> f64 {
    a[0].hypot(a[1])
}
fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn right(t: [f64; 2]) -> [f64; 2] {
    [t[1], -t[0]]
}
fn along(p: [f64; 2], d: [f64; 2], k: f64) -> [f64; 2] {
    [p[0] + k * d[0], p[1] + k * d[1]]
}
fn angle_of(c: [f64; 2], p: [f64; 2]) -> f64 {
    (p[1] - c[1]).atan2(p[0] - c[0])
}

impl Seg {
    pub fn start(&self) -> [f64; 2] {
        match *self {
            Seg::Line { a, .. } | Seg::Arc { a, .. } => a,
        }
    }
    pub fn end(&self) -> [f64; 2] {
        match *self {
            Seg::Line { b, .. } | Seg::Arc { b, .. } => b,
        }
    }
    /// Unit tangent at the start (t = 0) or end (t = 1).
    pub fn tangent(&self, at_end: bool) -> [f64; 2] {
        match *self {
            Seg::Line { a, b } => {
                let d = sub(b, a);
                let l = len(d);
                [d[0] / l, d[1] / l]
            }
            Seg::Arc { a, b, c, sweep } => {
                let p = if at_end { b } else { a };
                let r = sub(p, c);
                let l = len(r);
                let s = sweep.signum();
                [-r[1] / l * s, r[0] / l * s]
            }
        }
    }
    pub fn length(&self) -> f64 {
        match *self {
            Seg::Line { a, b } => len(sub(b, a)),
            Seg::Arc { a, c, sweep, .. } => len(sub(a, c)) * sweep.abs(),
        }
    }
    /// The motion curve at y = 0: x = r, z = u.
    pub fn curve(&self) -> Curve {
        let p = |q: [f64; 2]| -> Vec3 { [q[1], 0., q[0]] };
        match *self {
            Seg::Line { a, b } => Curve::Line {
                start: p(a),
                end: p(b),
            },
            Seg::Arc { a, b, c, sweep } => Curve::Arc {
                start: p(a),
                end: p(b),
                center: p(c),
                plane: Plane::Xz,
                sweep_rad: sweep,
            },
        }
    }
    /// A polyline for the segment: a line itself; an arc by points within
    /// `sag` of it, on the arc (`outside` false) or as the corners of its
    /// tangent lines (`outside` true: the polyline contains the arc's
    /// convex side). Both ends are exact.
    fn polyline(&self, sag: f64, outside: bool) -> Vec<[f64; 2]> {
        match *self {
            Seg::Line { a, b } => vec![a, b],
            Seg::Arc { a, b, c, sweep } => {
                let r = len(sub(a, c));
                let a0 = angle_of(c, a);
                let n = ((sweep.abs() * (r / (8. * sag.max(1e-9))).sqrt()).ceil() as usize)
                    .clamp(1, 4096);
                let step = sweep / n as f64;
                let at = |t: f64, radius: f64| [c[0] + radius * t.cos(), c[1] + radius * t.sin()];
                if outside {
                    let grow = r / (step.abs() / 2.).cos();
                    let mut points = vec![a];
                    points.extend((0..n).map(|k| at(a0 + step * (k as f64 + 0.5), grow)));
                    points.push(b);
                    points
                } else {
                    let mut points: Vec<[f64; 2]> =
                        (0..n).map(|k| at(a0 + step * k as f64, r)).collect();
                    points.push(b);
                    points
                }
            }
        }
    }
}

impl TurnProfile {
    pub fn segments(&self) -> Result<Vec<Seg>, String> {
        let p = &self.points;
        if p.len() < 2
            || p.iter()
                .any(|q| !q.iter().all(|v| v.is_finite() && v.abs() <= 1e6) || q[0] < 0.)
        {
            return Err("turned profile needs at least two finite points with radius >= 0".into());
        }
        let mut out = vec![];
        for w in p.windows(2) {
            let (a, b) = ([w[0][1], w[0][0]], [w[1][1], w[1][0]]);
            if len(sub(b, a)) <= 1e-9 {
                return Err("turned profile has a zero-length segment".into());
            }
            let bulge = w[0][2];
            out.push(if bulge == 0. {
                Seg::Line { a, b }
            } else {
                if bulge.abs() > 1e3 {
                    return Err("turned profile arc bulge must be within +-1000".into());
                }
                let f = (1. - bulge * bulge) / (4. * bulge);
                let c = [
                    (a[0] + b[0]) / 2. - f * (b[1] - a[1]),
                    (a[1] + b[1]) / 2. + f * (b[0] - a[0]),
                ];
                Seg::Arc {
                    a,
                    b,
                    c,
                    sweep: 4. * bulge.atan(),
                }
            });
        }
        Ok(out)
    }

    /// For a bore (from the front face into the hole, the material on the
    /// right of travel): the nose-center path `d` inside it, from the front
    /// face inward. The profile is offset reversed (material on the left)
    /// and the path turned back.
    pub fn bore_path(&self, d: f64) -> Result<Vec<Seg>, String> {
        let n = self.points.len();
        let reversed = TurnProfile {
            points: (0..n)
                .rev()
                .map(|i| {
                    let p = self.points[i];
                    // The bulge of the segment ending at i, reversed.
                    let bulge = if i > 0 { -self.points[i - 1][2] } else { 0. };
                    [p[0], p[1], bulge]
                })
                .collect(),
        };
        let mut path = reversed.offset(d)?;
        path.reverse();
        Ok(path
            .into_iter()
            .map(|s| match s {
                Seg::Line { a, b } => Seg::Line { a: b, b: a },
                Seg::Arc { a, b, c, sweep } => Seg::Arc {
                    a: b,
                    b: a,
                    c,
                    sweep: -sweep,
                },
            })
            .collect())
    }

    /// The z range: the front face and the chuck-side end.
    pub fn z_range(&self) -> [f64; 2] {
        let z = self.points.iter().map(|p| p[1]);
        [
            z.clone().fold(f64::INFINITY, f64::min),
            z.fold(f64::NEG_INFINITY, f64::max),
        ]
    }

    /// An upper bound of the part's outer radius as a function of z: a
    /// polyline in (u, v) that contains the profile (arcs bulging outward
    /// are replaced by chords pushed out by their sagitta).
    pub fn envelope(&self, sag: f64) -> Result<Envelope, String> {
        let mut points: Vec<[f64; 2]> = vec![];
        for s in self.segments()? {
            // A counter-clockwise arc bulges out of the part (interior on
            // the left): use the tangent corners outside it. A clockwise
            // arc is concave: its chords lie on the air side already.
            let convex = matches!(s, Seg::Arc { sweep, .. } if sweep > 0.);
            let mut poly = s.polyline(sag, convex);
            if points.last() == poly.first() {
                poly.remove(0);
            }
            points.extend(poly);
        }
        Ok(Envelope::new(points))
    }

    /// The nose-center path at distance `d` outside the profile (left of
    /// travel is the interior): offsets joined by arcs at convex corners and
    /// trimmed at concave ones.
    pub fn offset(&self, d: f64) -> Result<Vec<Seg>, String> {
        let segs = self.segments()?;
        let offset = |s: &Seg| -> Result<Seg, String> {
            Ok(match *s {
                Seg::Line { a, b } => {
                    let n = right(s.tangent(false));
                    Seg::Line {
                        a: along(a, n, d),
                        b: along(b, n, d),
                    }
                }
                Seg::Arc { a, b, c, sweep } => {
                    let r = len(sub(a, c));
                    let r2 = if sweep > 0. { r + d } else { r - d };
                    if r2 <= 1e-9 {
                        return Err(format!(
                            "a concave arc of radius {r:.4} at z {:.4} is smaller than the nose radius plus allowance {d:.4}",
                            a[0]
                        ));
                    }
                    let k = r2 / r;
                    Seg::Arc {
                        a: along(c, sub(a, c), k),
                        b: along(c, sub(b, c), k),
                        c,
                        sweep,
                    }
                }
            })
        };
        let mut out: Vec<Seg> = vec![offset(&segs[0])?];
        for i in 1..segs.len() {
            let next = offset(&segs[i])?;
            let (t1, t2) = (segs[i - 1].tangent(true), segs[i].tangent(false));
            let turn = cross(t1, t2);
            let v = segs[i].start();
            let prev = *out.last().unwrap();
            if len(sub(prev.end(), next.start())) <= 1e-9 {
                // One represented junction: independently computed tangent
                // offsets can differ by an ulp and would need an invalid link.
                out.push(if prev.end() == next.start() {
                    next
                } else {
                    trim(&next, Some(prev.end()), None)?
                });
            } else if turn > 0. {
                // Convex corner: round it about the vertex.
                let a0 = angle_of(v, prev.end());
                let a1 = angle_of(v, next.start());
                let mut sweep = a1 - a0;
                while sweep <= 0. {
                    sweep += std::f64::consts::TAU;
                }
                out.push(Seg::Arc {
                    a: prev.end(),
                    b: next.start(),
                    c: v,
                    sweep,
                });
                out.push(next);
            } else {
                // Concave corner: both offsets end at their intersection.
                let p = intersect(&prev, &next, v).ok_or_else(|| {
                    format!(
                        "the concave corner at radius {:.4}, z {:.4} is too tight for the nose radius plus allowance {d:.4}",
                        v[1], v[0]
                    )
                })?;
                let last = out.pop().unwrap();
                out.push(trim(&last, None, Some(p))?);
                out.push(trim(&next, Some(p), None)?);
            }
        }
        Ok(out)
    }
}

/// The intersection of two offset segments nearest the corner `v`.
fn intersect(a: &Seg, b: &Seg, v: [f64; 2]) -> Option<[f64; 2]> {
    let circle_line = |c: [f64; 2], r: f64, p: [f64; 2], q: [f64; 2]| -> Vec<[f64; 2]> {
        let d = sub(q, p);
        let f = sub(p, c);
        let (aa, bb, cc) = (
            d[0] * d[0] + d[1] * d[1],
            2. * (f[0] * d[0] + f[1] * d[1]),
            f[0] * f[0] + f[1] * f[1] - r * r,
        );
        let disc = bb * bb - 4. * aa * cc;
        if disc < 0. || aa == 0. {
            return vec![];
        }
        let s = disc.sqrt();
        [(-bb - s) / (2. * aa), (-bb + s) / (2. * aa)]
            .into_iter()
            .map(|t| along(p, d, t))
            .collect()
    };
    let candidates: Vec<[f64; 2]> = match (*a, *b) {
        (Seg::Line { a: p, b: q }, Seg::Line { a: r, b: s }) => {
            let (d1, d2) = (sub(q, p), sub(s, r));
            let den = cross(d1, d2);
            if den.abs() < 1e-15 {
                vec![]
            } else {
                vec![along(p, d1, cross(sub(r, p), d2) / den)]
            }
        }
        (Seg::Line { a: p, b: q }, Seg::Arc { a: s, c, .. })
        | (Seg::Arc { a: s, c, .. }, Seg::Line { a: p, b: q }) => {
            circle_line(c, len(sub(s, c)), p, q)
        }
        (Seg::Arc { a: s1, c: c1, .. }, Seg::Arc { a: s2, c: c2, .. }) => {
            let (r1, r2) = (len(sub(s1, c1)), len(sub(s2, c2)));
            let d = len(sub(c2, c1));
            if d == 0. || d > r1 + r2 || d < (r1 - r2).abs() {
                vec![]
            } else {
                let x = (d * d + r1 * r1 - r2 * r2) / (2. * d);
                let h = (r1 * r1 - x * x).max(0.).sqrt();
                let e = [(c2[0] - c1[0]) / d, (c2[1] - c1[1]) / d];
                let m = along(c1, e, x);
                vec![along(m, [-e[1], e[0]], h), along(m, [e[1], -e[0]], h)]
            }
        }
    };
    candidates
        .into_iter()
        .filter(|p| on_segment(a, *p) && on_segment(b, *p))
        .min_by(|p, q| len(sub(*p, v)).total_cmp(&len(sub(*q, v))))
}

fn on_segment(s: &Seg, p: [f64; 2]) -> bool {
    let eps = 1e-9;
    match *s {
        Seg::Line { a, b } => {
            let d = sub(b, a);
            let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]);
            (-eps..=1. + eps).contains(&t)
        }
        Seg::Arc { a, c, sweep, .. } => {
            let rel = angle_of(c, p) - angle_of(c, a);
            let t = if sweep > 0. {
                rel.rem_euclid(std::f64::consts::TAU)
            } else {
                (-rel).rem_euclid(std::f64::consts::TAU)
            };
            t <= sweep.abs() + eps || t >= std::f64::consts::TAU - eps
        }
    }
}

/// The part of a segment from `from` (or its start) to `to` (or its end).
fn trim(s: &Seg, from: Option<[f64; 2]>, to: Option<[f64; 2]>) -> Result<Seg, String> {
    let (a, b) = (from.unwrap_or(s.start()), to.unwrap_or(s.end()));
    let out = match *s {
        Seg::Line { .. } => Seg::Line { a, b },
        Seg::Arc { c, sweep, .. } => {
            let rel = angle_of(c, b) - angle_of(c, a);
            let mut t = if sweep > 0. {
                rel.rem_euclid(std::f64::consts::TAU)
            } else {
                -(-rel).rem_euclid(std::f64::consts::TAU)
            };
            if t == 0. && sweep.abs() >= std::f64::consts::TAU - 1e-12 {
                t = sweep;
            }
            Seg::Arc { a, b, c, sweep: t }
        }
    };
    if out.length() <= 1e-9
        || out.tangent(false)[0] * s.tangent(false)[0] + out.tangent(false)[1] * s.tangent(false)[1]
            < 0.
    {
        return Err("a profile feature is too small for the nose radius plus allowance".into());
    }
    Ok(out)
}

/// The part's outer radius bound as a function of z: a polyline in (u, v).
#[derive(Clone, Debug)]
pub struct Envelope {
    points: Vec<[f64; 2]>,
    /// The chain's z order: +1 never decreasing, -1 never increasing, 0
    /// neither (queries then scan every point).
    order: i8,
}

impl Envelope {
    pub fn new(points: Vec<[f64; 2]>) -> Self {
        let up = points.windows(2).all(|w| w[0][0] <= w[1][0]);
        let down = points.windows(2).all(|w| w[0][0] >= w[1][0]);
        let order = if up {
            1
        } else if down {
            -1
        } else {
            0
        };
        Self { points, order }
    }
    pub fn points(&self) -> &[[f64; 2]] {
        &self.points
    }
    /// The indices of the points with lo <= u <= hi in a monotone chain
    /// (every index otherwise; callers still test each point).
    fn within(&self, lo: f64, hi: f64) -> std::ops::Range<usize> {
        let p = &self.points;
        match self.order {
            1 => p.partition_point(|x| x[0] < lo)..p.partition_point(|x| x[0] <= hi),
            -1 => p.partition_point(|x| x[0] > hi)..p.partition_point(|x| x[0] >= lo),
            _ => 0..p.len(),
        }
    }
    /// The lowest and highest envelope radius over lo <= u <= hi (None when
    /// the envelope does not reach that range).
    pub fn radius_range(&self, lo: f64, hi: f64) -> Option<[f64; 2]> {
        let mut range: Option<[f64; 2]> = None;
        let mut take = |v: f64| {
            range = Some(range.map_or([v, v], |[a, b]| [a.min(v), b.max(v)]));
        };
        let span = self.within(lo, hi);
        let first = span.start.saturating_sub(1);
        let last = (span.end + 1).min(self.points.len());
        for w in self.points[first..last].windows(2) {
            let (p, q) = (w[0], w[1]);
            let (a, b) = (p[0].min(q[0]).max(lo), p[0].max(q[0]).min(hi));
            if a > b {
                continue;
            }
            if q[0] == p[0] {
                take(p[1]);
                take(q[1]);
            } else {
                let at = |u: f64| p[1] + (u - p[0]) / (q[0] - p[0]) * (q[1] - p[1]);
                take(at(a));
                take(at(b));
            }
        }
        range
    }
    /// The highest envelope radius at u (over every segment spanning u).
    pub fn radius_at(&self, u: f64) -> Option<f64> {
        let mut best: Option<f64> = None;
        // The segments that may span u: those touching a point at u, or
        // the one across it.
        let range = self.within(u, u);
        let first = range.start.saturating_sub(1);
        let last = (range.end + 1).min(self.points.len());
        for w in self.points[first..last].windows(2) {
            let (p, q) = (w[0], w[1]);
            let (lo, hi) = (p[0].min(q[0]), p[0].max(q[0]));
            if u < lo || u > hi {
                continue;
            }
            let v = if q[0] == p[0] {
                p[1].max(q[1])
            } else {
                p[1] + (u - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
            };
            best = Some(best.map_or(v, |b: f64| b.max(v)));
        }
        best
    }
    /// The envelope with its radius limited to `radius` between z0 and z1
    /// (a thread cut into the profile's diameter there).
    pub fn carved(&self, z0: f64, z1: f64, radius: f64) -> Envelope {
        self.reshaped(z0, z1, |v| v.min(radius))
    }
    /// A bore envelope opened out to `radius` between z0 and z1 (an
    /// internal thread's roots there).
    pub fn raised(&self, z0: f64, z1: f64, radius: f64) -> Envelope {
        self.reshaped(z0, z1, |v| v.max(radius))
    }
    fn reshaped(&self, z0: f64, z1: f64, f: impl Fn(f64) -> f64) -> Envelope {
        let (lo, hi) = (z0.min(z1), z0.max(z1));
        let mut points: Vec<[f64; 2]> = vec![];
        for w in self.points.windows(2) {
            let (p, q) = (w[0], w[1]);
            let mut piece = vec![p];
            // Split the segment where it enters or leaves [lo, hi].
            for b in [lo, hi] {
                if (p[0] - b) * (q[0] - b) < 0. {
                    let t = (b - p[0]) / (q[0] - p[0]);
                    piece.push([b, p[1] + t * (q[1] - p[1])]);
                }
            }
            piece.sort_by(|a, b| {
                let ta = (a[0] - p[0]).abs();
                let tb = (b[0] - p[0]).abs();
                ta.total_cmp(&tb)
            });
            for x in piece {
                if points.last() != Some(&x) {
                    points.push(x);
                }
            }
        }
        if let Some(last) = self.points.last()
            && points.last() != Some(last)
        {
            points.push(*last);
        }
        for x in &mut points {
            if x[0] >= lo && x[0] <= hi {
                x[1] = f(x[1]);
            }
        }
        Envelope::new(points)
    }
    /// For an envelope that is a bore (the radius of a hole over z, from
    /// its deepest point to the front face): how far a convex polygon (the
    /// +r half of a tool on the axis) reaches beyond the bore dilated by
    /// `tol`, compared at every breakpoint (exact for piecewise linear
    /// shapes). In front of the bore there is no material; past its
    /// deepest point the hole has no radius. Positive: the tool would cut
    /// the part outside its bore.
    pub fn excess(&self, polygon: &[[f64; 2]], tol: f64) -> f64 {
        let (lo, hi) = polygon
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                (l.min(p[0]), h.max(p[0]))
            });
        let (front, deepest) = self
            .points
            .iter()
            .fold((f64::NEG_INFINITY, f64::INFINITY), |(f, d), p| {
                (f.max(p[0]), d.min(p[0]))
            });
        let bore = |u: f64| {
            if u > front {
                f64::INFINITY
            } else if u < deepest {
                0.
            } else {
                self.radius_at(u).unwrap_or(0.)
            }
        };
        let dilated = |u: f64| {
            let inside = self.points[self.within(u - tol, u + tol)]
                .iter()
                .map(|p| p[0])
                .filter(|b| *b > u - tol && *b < u + tol);
            [u - tol, u + tol]
                .into_iter()
                .chain(inside)
                .map(bore)
                .fold(f64::NEG_INFINITY, f64::max)
                + tol
        };
        let highest_at = |u: f64| -> Option<f64> {
            let mut best: Option<f64> = None;
            let n = polygon.len();
            for i in 0..n {
                let (p, q) = (polygon[i], polygon[(i + 1) % n]);
                let (a, b) = (p[0].min(q[0]), p[0].max(q[0]));
                if u < a || u > b {
                    continue;
                }
                let v = if q[0] == p[0] {
                    p[1].max(q[1])
                } else {
                    p[1] + (u - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
                };
                best = Some(best.map_or(v, |x: f64| x.max(v)));
            }
            best
        };
        let mut worst = f64::NEG_INFINITY;
        let candidates = self.points[self.within(lo - tol, hi + tol)]
            .iter()
            .flat_map(|p| [p[0] - tol, p[0], p[0] + tol])
            .chain(polygon.iter().map(|p| p[0]))
            .filter(|u| (lo..=hi).contains(u));
        for u in candidates {
            if let Some(high) = highest_at(u) {
                worst = worst.max(high - dilated(u));
            }
        }
        worst
    }
    /// The part eroded by `tol` (a square of half-size `tol`): the least
    /// radius within `tol` along z, less `tol`; None where the window leaves
    /// the part (near its ends no erosion of material exists).
    fn eroded(&self, u: f64, tol: f64) -> Option<f64> {
        let mut m = f64::INFINITY;
        let inside = self.points[self.within(u - tol, u + tol)]
            .iter()
            .map(|p| p[0])
            .filter(|b| *b > u - tol && *b < u + tol);
        for x in [u - tol, u + tol].into_iter().chain(inside) {
            m = m.min(self.radius_at(x)?);
        }
        Some(m - tol)
    }
    /// How far a convex polygon (in (u, v)) overlaps the part eroded by
    /// `tol`: the largest eroded radius minus the polygon's lowest radius
    /// over the z values both share, compared at every breakpoint of either
    /// (exact: both are piecewise linear in z between them). Positive: a
    /// gouge deeper than `tol`; negative: clear by about that much plus
    /// `tol`. Touching a face is not a gouge.
    pub fn penetration(&self, polygon: &[[f64; 2]], tol: f64) -> f64 {
        let (lo, hi) = polygon
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), p| {
                (l.min(p[0]), h.max(p[0]))
            });
        let lowest_at = |u: f64| -> Option<f64> {
            let mut best: Option<f64> = None;
            let n = polygon.len();
            for i in 0..n {
                let (p, q) = (polygon[i], polygon[(i + 1) % n]);
                let (a, b) = (p[0].min(q[0]), p[0].max(q[0]));
                if u < a || u > b {
                    continue;
                }
                let v = if q[0] == p[0] {
                    p[1].min(q[1])
                } else {
                    p[1] + (u - p[0]) / (q[0] - p[0]) * (q[1] - p[1])
                };
                best = Some(best.map_or(v, |x: f64| x.min(v)));
            }
            best
        };
        let mut worst = f64::NEG_INFINITY;
        let candidates = self.points[self.within(lo - tol, hi + tol)]
            .iter()
            .flat_map(|p| [p[0] - tol, p[0], p[0] + tol])
            .chain(polygon.iter().map(|p| p[0]))
            .filter(|u| (lo..=hi).contains(u));
        for u in candidates {
            // Where the eroded material has no radius there is none to
            // touch (a tool may reach across the axis over an emptied end).
            if let (Some(env), Some(low)) = (self.eroded(u, tol), lowest_at(u))
                && env > 0.
            {
                worst = worst.max(env - low);
            }
        }
        worst
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A shaft: face to r 10, chamfer-free shoulder down to r 15 at z -20,
    /// out to r 15 to z -40.
    fn shaft() -> TurnProfile {
        TurnProfile {
            points: vec![
                [0., 0., 0.],
                [10., 0., 0.],
                [10., -20., 0.],
                [15., -20., 0.],
                [15., -40., 0.],
            ],
        }
    }

    #[test]
    fn offset_rounds_convex_corners_and_trims_concave_ones() {
        let d = 0.8;
        let path = shaft().offset(d).unwrap();
        // Face offset forward, corner rounded, diameter offset out, the
        // inside corner at the shoulder trimmed, the shoulder face offset
        // forward, its outer corner rounded, the last diameter offset out.
        assert_eq!(path.first().unwrap().start(), [d, 0.]);
        let last = path.last().unwrap().end();
        assert!((last[0] + 40.).abs() < 1e-12 && (last[1] - 15.8).abs() < 1e-12);
        for w in path.windows(2) {
            assert!(len(sub(w[0].end(), w[1].start())) < 1e-9, "{path:?}");
        }
        // The concave corner: the nose center stays d from both faces.
        assert!(
            path.iter()
                .any(|s| len(sub(s.end(), [-20. + d, 10. + d])) < 1e-9)
        );
        // Every path point keeps at least d from the part (check samples).
        let env = shaft().envelope(1e-4).unwrap();
        for s in &path {
            for p in s.polyline(1e-4, false) {
                if let Some(r) = env.radius_at(p[0]) {
                    assert!(
                        p[1] >= r + d - 1e-6 || p[0] > -1e-9 || p[0] < -40. + 1e-9,
                        "{p:?} {r}"
                    );
                }
            }
        }
    }

    #[test]
    fn rounded_offset_segments_share_the_exact_junction() {
        // #242: quarter-circle bulges from the Python fixture across libm
        // implementations. Independent offsets can differ by an ulp at a
        // tangent join; its connector is too short to be a valid motion.
        for bulge in [0.41421356237309503, 0.4142135623730951] {
            let profile = TurnProfile {
                points: vec![
                    [0., 0., bulge],
                    [6., -6., 0.],
                    [6., -20., -bulge],
                    [9., -23., 0.],
                    [14., -23., 0.],
                    [14., -40., 0.],
                ],
            };
            for radius in [0.8, 0.4, 1.2] {
                let path = profile.offset(radius).unwrap();
                for pair in path.windows(2) {
                    assert_eq!(
                        pair[0].end(),
                        pair[1].start(),
                        "bulge {bulge}, radius {radius}: {pair:?}"
                    );
                }
                for segment in path {
                    segment.curve().prepare(1e-3).unwrap();
                }
            }
        }
    }

    #[test]
    fn penetration_is_exact_for_polygons() {
        let env = shaft().envelope(1e-4).unwrap();
        // A square nose region 1 mm on a side touching r 10.5 over z -5..-4.
        let tol = 1e-3;
        let square = [[-5., 10.5], [-4., 10.5], [-4., 11.5], [-5., 11.5]];
        assert!((env.penetration(&square, tol) + 0.5 + tol).abs() < 1e-12);
        // Reaching past the shoulder at z -20 into r 15 material.
        let deep = [[-21., 12.], [-19., 12.], [-19., 13.], [-21., 13.]];
        assert!((env.penetration(&deep, tol) - (3. - tol)).abs() < 1e-12);
        // Touching the front face (z 0) from outside is contact, not a gouge;
        // reaching 0.01 mm past it is.
        let touch = [[0., 0.], [1., 0.], [1., 1.], [0., 1.]];
        assert!(env.penetration(&touch, tol) < 0.);
        let past = [[-0.01, 0.], [1., 0.], [1., 1.], [-0.01, 1.]];
        assert!(env.penetration(&past, tol) > 0.);
    }

    #[test]
    fn a_groove_narrower_than_the_nose_is_refused() {
        let groove = TurnProfile {
            points: vec![
                [0., 0., 0.],
                [15., 0., 0.],
                [15., -10., 0.],
                [12., -10., 0.],
                [12., -11., 0.],
                [15., -11., 0.],
                [15., -30., 0.],
            ],
        };
        let err = groove.offset(0.8).unwrap_err();
        assert!(err.contains("too"), "{err}");
        assert!(groove.offset(0.4).is_ok());
    }
}
