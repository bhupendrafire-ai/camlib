//! Lazy exact endpoint-pair ordering for large guide graphs. Bounding-box pairs
//! carry an upper preferred-radius and lower distance key; only leaves yield
//! candidates. No finite-neighbor cutoff can silently disconnect a component.
use super::{Groups, Point, distance};
use crate::stock::CancelSignal;
use std::{cmp::Ordering, collections::BinaryHeap};
const LEAF: usize = 8;
struct Node {
    bounds: [Point; 2],
    max_radius: f64,
    group: Option<usize>,
    range: std::ops::Range<usize>,
    children: Option<[usize; 2]>,
}
#[derive(Clone, Copy)]
enum Kind {
    Boxes(usize, usize),
    Pair(usize, usize),
}
struct Entry {
    preferred: f64,
    distance: f64,
    serial: usize,
    kind: Kind,
}
impl PartialEq for Entry {
    fn eq(&self, o: &Self) -> bool {
        self.serial == o.serial
    }
}
impl Eq for Entry {}
impl PartialOrd for Entry {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Entry {
    fn cmp(&self, o: &Self) -> Ordering {
        self.preferred
            .total_cmp(&o.preferred)
            .then_with(|| o.distance.total_cmp(&self.distance))
            .then_with(|| o.serial.cmp(&self.serial))
    }
}
pub(super) struct Pairs<'a> {
    points: &'a [Point],
    radii: &'a [f64],
    order: Vec<usize>,
    nodes: Vec<Node>,
    heap: BinaryHeap<Entry>,
    visited: usize,
    serial: usize,
}
impl<'a> Pairs<'a> {
    pub(super) fn new(
        points: &'a [Point],
        radii: &'a [f64],
        groups: &mut Groups,
        cancel: &impl CancelSignal,
    ) -> Result<Self, String> {
        let labels: Vec<_> = (0..points.len()).map(|i| groups.root(i)).collect();
        let mut s = Self {
            points,
            radii,
            order: (0..points.len()).collect(),
            nodes: vec![],
            heap: BinaryHeap::new(),
            visited: 0,
            serial: 0,
        };
        if !points.is_empty() {
            let root = s.build(0..points.len(), &labels, cancel)?;
            s.boxes(root, root)?;
        }
        Ok(s)
    }
    fn charge(&mut self, n: usize, cancel: &impl CancelSignal) -> Result<(), String> {
        if cancel.is_cancelled() {
            return Err("adaptive navigation cancelled".into());
        }
        self.visited = self.visited.saturating_add(n);
        Ok(())
    }
    fn build(
        &mut self,
        range: std::ops::Range<usize>,
        labels: &[usize],
        cancel: &impl CancelSignal,
    ) -> Result<usize, String> {
        self.charge(range.len(), cancel)?;
        let first = self.order[range.start];
        let mut bounds = [self.points[first]; 2];
        let mut max_radius = self.radii[first];
        let mut group = Some(labels[first]);
        for (k, &i) in self.order[range.clone()].iter().enumerate() {
            if k.is_multiple_of(1024) && cancel.is_cancelled() {
                return Err("adaptive navigation cancelled".into());
            }
            for (a, _) in self.points[i].iter().enumerate() {
                bounds[0][a] = bounds[0][a].min(self.points[i][a]);
                bounds[1][a] = bounds[1][a].max(self.points[i][a]);
            }
            max_radius = max_radius.max(self.radii[i]);
            if group != Some(labels[i]) {
                group = None;
            }
        }
        let index = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            max_radius,
            group,
            range: range.clone(),
            children: None,
        });
        if range.len() > LEAF {
            let axis = usize::from(bounds[1][1] - bounds[0][1] > bounds[1][0] - bounds[0][0]);
            let mid = range.start + range.len() / 2;
            let points = self.points;
            self.order[range.clone()].select_nth_unstable_by(range.len() / 2, |a, b| {
                points[*a][axis].total_cmp(&points[*b][axis]).then(a.cmp(b))
            });
            let left = self.build(range.start..mid, labels, cancel)?;
            let right = self.build(mid..range.end, labels, cancel)?;
            self.nodes[index].children = Some([left, right]);
        }
        Ok(index)
    }
    fn push(&mut self, preferred: f64, distance: f64, kind: Kind) -> Result<(), String> {
        let serial = self.serial;
        self.serial += 1;
        self.heap.push(Entry {
            preferred,
            distance,
            serial,
            kind,
        });
        Ok(())
    }
    fn boxes(&mut self, a: usize, b: usize) -> Result<(), String> {
        let (x, y) = (&self.nodes[a], &self.nodes[b]);
        let d: Point = std::array::from_fn(|i| {
            (x.bounds[0][i] - y.bounds[1][i])
                .max(y.bounds[0][i] - x.bounds[1][i])
                .max(0.)
        });
        self.push(
            x.max_radius.min(y.max_radius),
            // Same length as `distance`: a lower bound in that metric, not libm hypot.
            (d[0] * d[0] + d[1] * d[1]).sqrt(),
            Kind::Boxes(a, b),
        )
    }
    pub(super) fn next(
        &mut self,
        groups: &mut Groups,
        cancel: &impl CancelSignal,
    ) -> Result<Option<(f64, f64, usize, usize)>, String> {
        while let Some(entry) = self.heap.pop() {
            self.charge(1, cancel)?;
            match entry.kind {
                Kind::Pair(a, b) => {
                    if groups.root(a) != groups.root(b) {
                        return Ok(Some((entry.preferred, entry.distance, a, b)));
                    }
                }
                Kind::Boxes(a, b) => {
                    let (x, y) = (&self.nodes[a], &self.nodes[b]);
                    if let (Some(x), Some(y)) = (x.group, y.group)
                        && groups.root(x) == groups.root(y)
                    {
                        continue;
                    }
                    match (x.children, y.children) {
                        (None, None) => {
                            let (xr, yr) = (x.range.clone(), y.range.clone());
                            for i in xr {
                                for j in yr.clone() {
                                    if a == b && j <= i {
                                        continue;
                                    }
                                    let (i, j) = (self.order[i], self.order[j]);
                                    if groups.root(i) != groups.root(j) {
                                        let (i, j) = (i.min(j), i.max(j));
                                        self.push(
                                            self.radii[i].min(self.radii[j]),
                                            distance(self.points[i], self.points[j]),
                                            Kind::Pair(i, j),
                                        )?;
                                    }
                                }
                            }
                        }
                        (Some([l, r]), _) if a == b => {
                            self.boxes(l, l)?;
                            self.boxes(l, r)?;
                            self.boxes(r, r)?;
                        }
                        (Some([l, r]), _)
                            if y.children.is_none() || x.range.len() >= y.range.len() =>
                        {
                            self.boxes(l, b)?;
                            self.boxes(r, b)?;
                        }
                        (_, Some([l, r])) => {
                            self.boxes(a, l)?;
                            self.boxes(a, r)?;
                        }
                        _ => unreachable!(),
                    }
                }
            }
        }
        Ok(None)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn box_distance_is_a_lower_bound_for_the_endpoint_ordering_key() {
        let critical = super::super::tests::competing_bridge_points()[2];
        // Two leaf boxes, each containing coincident endpoints. Their gap is
        // exactly the #245 separation, so the box bound must not exceed the
        // endpoint key by even one ulp or best-first ordering can skip a pair.
        let points: Vec<_> = (0..16)
            .map(|i| if i < 8 { [0., 0.] } else { critical })
            .collect();
        let radii = vec![1.; points.len()];
        let mut groups = Groups::new(points.len());
        let mut search = Pairs::new(&points, &radii, &mut groups, &|| false).unwrap();
        let [a, b] = search.nodes[0].children.unwrap();
        search.heap.clear();
        search.boxes(a, b).unwrap();
        let bound = search.heap.pop().unwrap().distance;
        assert_eq!(bound.to_bits(), 4630074070701655572);
        assert!(bound <= distance(points[0], points[8]));
    }

    #[test]
    fn lazy_pairs_match_every_brute_force_pair_and_priority() {
        let points: Vec<_> = (0..137)
            .map(|i| {
                [
                    (i as f64 * 1.732).sin() * 10.,
                    (i as f64 * 0.713).cos() * 7.,
                ]
            })
            .collect();
        let radii: Vec<_> = (0..points.len()).map(|i| (i % 5) as f64 * 0.2).collect();
        let mut groups = Groups::new(points.len());
        for i in (1..points.len()).step_by(3) {
            groups.join(i - 1, i);
        }
        let mut expected = vec![];
        for a in 0..points.len() {
            for b in a + 1..points.len() {
                if groups.root(a) != groups.root(b) {
                    expected.push((radii[a].min(radii[b]), distance(points[a], points[b]), a, b));
                }
            }
        }
        let mut search = Pairs::new(&points, &radii, &mut groups, &|| false).unwrap();
        let mut actual = vec![];
        while let Some(p) = search.next(&mut groups, &|| false).unwrap() {
            actual.push(p);
        }
        for p in actual.windows(2) {
            assert!(p[0].0 > p[1].0 || p[0].0 == p[1].0 && p[0].1 <= p[1].1);
        }
        expected.sort_unstable_by_key(|p| (p.2, p.3));
        actual.sort_unstable_by_key(|p| (p.2, p.3));
        assert_eq!(actual, expected);
    }
    #[test]
    fn large_connected_contours_skip_quadratic_pairs_and_follow_new_joins() {
        let points: Vec<_> = (0..8192).map(|i| [i as f64, 0.]).collect();
        let radii = vec![0.; points.len()];
        let mut g = Groups::new(points.len());
        for i in 1..points.len() {
            g.join(i / 4096 * 4096, i);
        }
        let mut search = Pairs::new(&points, &radii, &mut g, &|| false).unwrap();
        let p = search.next(&mut g, &|| false).unwrap().unwrap();
        assert_eq!((p.2, p.3), (4095, 4096));
        g.join(p.2, p.3);
        assert!(search.next(&mut g, &|| false).unwrap().is_none());
        assert!(search.visited < 200_000);
    }
    #[test]
    fn pair_search_cancel_and_work_exhaustion_are_explicit() {
        let points = vec![[0., 0.], [1., 0.]];
        let radii = vec![0.; 2];
        let mut g = Groups::new(2);
        assert!(Pairs::new(&points, &radii, &mut g, &|| true).is_err());
        let mut s = Pairs::new(&points, &radii, &mut g, &|| false).unwrap();
        // Search work is counted, never capped.
        s.visited = 1_000_000_000;
        assert!(s.next(&mut g, &|| false).is_ok());
    }
}
