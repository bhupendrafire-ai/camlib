//! Whole-rectangle upper-envelope bounds, independent of cutter paths.
use super::*;
use std::collections::BTreeMap;

type EdgeKey = ([u64; 2], [u64; 2]);

// A deliberately sparse tree-node/allocation allowance per distinct edge,
// including key/value storage and node links; never just the payload bytes.
const EDGE_STORAGE_BYTES: u128 = 1024;

fn query_memory(
    base_bytes: u128,
    candidate_capacity: usize,
    edges: usize,
    budget: Option<u64>,
) -> Result<(), String> {
    let bytes = base_bytes
        + candidate_capacity as u128 * std::mem::size_of::<(f64, usize)>() as u128
        + edges as u128 * EDGE_STORAGE_BYTES;
    if budget.is_some_and(|limit| bytes > limit as u128) {
        crate::resources::check_memory(bytes, "surface material query")?;
    }
    Ok(())
}

fn key(p: Vec3) -> [u64; 2] {
    [p[0], p[1]].map(|x| if x == 0. { 0 } else { x.to_bits() })
}
fn point(p: [u64; 2]) -> Vec3 {
    [f64::from_bits(p[0]), f64::from_bits(p[1]), 0.]
}
fn edge_meets_box(a: Vec3, b: Vec3, r: [f64; 4]) -> bool {
    if a[0].max(b[0]) < r[0]
        || a[0].min(b[0]) > r[2]
        || a[1].max(b[1]) < r[1]
        || a[1].min(b[1]) > r[3]
    {
        return false;
    }
    let corners = [
        [r[0], r[1], 0.],
        [r[2], r[1], 0.],
        [r[2], r[3], 0.],
        [r[0], r[3], 0.],
    ];
    // The segment's supporting line misses this convex rectangle iff all
    // corners lie strictly on one side. Combined with the AABB this is a
    // conservative intersection test (false positives only).
    let signs = corners.map(|p| orientation(a, b, p, 2));
    !(signs.iter().all(|s| *s > 0.) || signs.iter().all(|s| *s < 0.))
}
/// Whether the XY projection of facet `p` (exact orientation `area` of its
/// projection) meets the closed box `r`, given that their bounding boxes
/// overlap. Exact predicates: false only when one of the facet's edge lines
/// strictly separates the box (with the box axes, the complete separating-axis
/// test for two convex polygons). A degenerate (vertical) facet projects to
/// the segments of its edges.
fn projection_meets_box(p: [Vec3; 3], area: f64, r: [f64; 4]) -> bool {
    if area == 0. {
        return (0..3).any(|j| edge_meets_box(p[j], p[(j + 1) % 3], r));
    }
    let corners = [
        [r[0], r[1], 0.],
        [r[2], r[1], 0.],
        [r[2], r[3], 0.],
        [r[0], r[3], 0.],
    ];
    (0..3).all(|j| {
        let (a, b) = (p[j], p[(j + 1) % 3]);
        // The facet lies on the `area` side of each of its edge lines.
        !corners.iter().all(|&c| {
            let side = orientation(a, b, c, 2);
            side != 0. && (side > 0.) != (area > 0.)
        })
    })
}
fn charge(work: &mut u64, limit: u64, amount: u64) -> bool {
    if amount > limit.saturating_sub(*work) {
        return false;
    }
    *work += amount;
    true
}

impl PreparedSurface {
    pub(crate) fn material_bounds(
        &self,
        xy: [f64; 4],
        tolerance: f64,
        work: &mut u64,
        limit: u64,
        cancel: &impl CancelSignal,
    ) -> Result<Option<[f64; 2]>, String> {
        self.material_bounds_with_memory(xy, tolerance, work, limit, 0, cancel)
    }
    pub(crate) fn material_bounds_with_memory(
        &self,
        xy: [f64; 4],
        tolerance: f64,
        work: &mut u64,
        limit: u64,
        additional_live_bytes: u128,
        cancel: &impl CancelSignal,
    ) -> Result<Option<[f64; 2]>, String> {
        let r = [
            xy[0] - tolerance,
            xy[1] - tolerance,
            xy[2] + tolerance,
            xy[3] + tolerance,
        ];
        let stack_capacity = (usize::BITS - self.triangles.len().leading_zeros()) as usize + 1;
        if !charge(work, limit, stack_capacity as u64) {
            return Ok(None);
        }
        let base_bytes = self.allocation_bytes()
            + additional_live_bytes
            + stack_capacity as u128 * std::mem::size_of::<usize>() as u128;
        crate::resources::check_memory(base_bytes, "surface material query")?;
        let memory_budget = crate::resources::memory_budget();
        let mut stack = Vec::with_capacity(stack_capacity);
        stack.push(0);
        let mut candidates = Vec::new();
        let mut upper = self.floor;
        while let Some(i) = stack.pop() {
            if cancel.is_cancelled() {
                return Err("surface material check cancelled".into());
            }
            if !charge(work, limit, 1) {
                return Ok(None);
            }
            let node = &self.nodes[i];
            if node.max[0] < r[0] || node.min[0] > r[2] || node.max[1] < r[1] || node.min[1] > r[3]
            {
                continue;
            }
            if let Some(children) = node.children {
                stack.extend(children);
                continue;
            }
            for i in node.range.clone() {
                if cancel.is_cancelled() {
                    return Err("surface material check cancelled".into());
                }
                if !charge(work, limit, 1) {
                    return Ok(None);
                }
                let t = &self.triangles[i];
                if t.max[0] < r[0] || t.min[0] > r[2] || t.max[1] < r[1] || t.min[1] > r[3] {
                    continue;
                }
                let area = orientation(t.p[0], t.p[1], t.p[2], 2);
                let mut lo = t.min[2];
                let mut hi = t.max[2];
                if area != 0. {
                    let d = sub(t.p[1], t.p[0]);
                    let e = sub(t.p[2], t.p[0]);
                    let gx = (d[2] * e[1] - e[2] * d[1]) / area;
                    let gy = (d[0] * e[2] - e[0] * d[2]) / area;
                    let scale = 1.
                        + t.p.iter().flatten().map(|x| x.abs()).sum::<f64>()
                        + r.iter().map(|x| x.abs()).sum::<f64>();
                    let condition =
                        (d[0].abs() * e[1].abs() + e[0].abs() * d[1].abs()) / area.abs();
                    let guard = (tolerance + 512. * f64::EPSILON * scale * (1. + condition))
                        * (1. + gx.abs() + gy.abs());
                    let z = [[r[0], r[1]], [r[0], r[3]], [r[2], r[1]], [r[2], r[3]]]
                        .map(|p| t.p[0][2] + gx * (p[0] - t.p[0][0]) + gy * (p[1] - t.p[0][1]));
                    if guard.is_finite() && z.iter().all(|v| v.is_finite()) {
                        lo = lo.max(z.iter().copied().fold(f64::INFINITY, f64::min) - guard);
                        hi = hi.min(z.iter().copied().fold(f64::NEG_INFINITY, f64::max) + guard);
                    }
                }
                if lo > hi {
                    continue;
                } // affine plane excludes this AABB candidate
                // Only a facet whose projection meets the box bounds the
                // surface over it. A facet beside the box whose bounding box
                // overlaps it (a top face around a through hole) does not:
                // counting it raised the bound over the hole to the top face.
                let meets_box = projection_meets_box(t.p, area, r);
                if meets_box {
                    upper = upper.max(hi);
                }
                // A strictly disjoint facet cannot contribute lower coverage
                // anywhere in the closed query rectangle either.
                if area != 0. && meets_box {
                    if candidates.len() == candidates.capacity() {
                        let next = candidates
                            .capacity()
                            .saturating_mul(2)
                            .max(1)
                            .min(self.triangles.len());
                        if !charge(work, limit, (next - candidates.capacity()) as u64) {
                            return Ok(None);
                        }
                        // A realloc may temporarily retain the old allocation.
                        query_memory(base_bytes, next + candidates.capacity(), 0, memory_budget)?;
                        candidates
                            .try_reserve_exact(next - candidates.capacity())
                            .map_err(|e| {
                                format!("surface material candidate allocation failed: {e}")
                            })?;
                    }
                    candidates.push((lo, i));
                }
            }
        }
        match sort_preparation(
            &mut candidates,
            |a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)),
            work,
            limit,
            cancel,
            false,
        ) {
            Ok(()) => {}
            Err(error) if error.contains("work limit") => return Ok(None),
            Err(error) => return Err(error),
        }
        let candidate_capacity = candidates.capacity();
        let mut edges: BTreeMap<EdgeKey, u8> = BTreeMap::new();
        let center = [(r[0] + r[2]) * 0.5, (r[1] + r[3]) * 0.5, 0.];
        let mut covered_center = false;
        for (lower, i) in candidates {
            if cancel.is_cancelled() {
                return Err("surface material check cancelled".into());
            }
            if !charge(work, limit, 3) {
                return Ok(None);
            }
            let t = &self.triangles[i];
            covered_center |= inside(t.p, center, 2);
            for j in 0..3 {
                if cancel.is_cancelled() {
                    return Err("surface material check cancelled".into());
                }
                if !charge(work, limit, 1) {
                    return Ok(None);
                }
                let (a, b) = (t.p[j], t.p[(j + 1) % 3]);
                // The coverage proof only uses edges that may meet this box.
                // Omit irrelevant edges before tree lookup/storage, preserving
                // every touching edge and the side masks of relevant edges.
                if !edge_meets_box(a, b, r) {
                    continue;
                }
                let (a, b) = (key(a), key(b));
                let (a, b) = if a < b { (a, b) } else { (b, a) };
                let side = orientation(point(a), point(b), t.p[(j + 2) % 3], 2);
                let edge_count = edges.len();
                // Reserve a conservative search/update allowance, including
                // sparse tree levels, before the standard-library operation.
                let lookup_work = 16 * (u64::from((edge_count + 1).ilog2()) + 1);
                if cancel.is_cancelled() {
                    return Err("surface material check cancelled".into());
                }
                if !charge(work, limit, lookup_work) {
                    return Ok(None);
                }
                let value = if side > 0. { 1 } else { 2 };
                match edges.entry((a, b)) {
                    std::collections::btree_map::Entry::Occupied(mut entry) => {
                        *entry.get_mut() |= value;
                    }
                    std::collections::btree_map::Entry::Vacant(entry) => {
                        if !charge(work, limit, 1) {
                            return Ok(None);
                        }
                        query_memory(
                            base_bytes,
                            candidate_capacity,
                            edge_count + 1,
                            memory_budget,
                        )?;
                        entry.insert(value);
                    }
                }
            }
            if !covered_center {
                continue;
            }
            let mut boundary = false;
            for (&(a, b), &sides) in &edges {
                if cancel.is_cancelled() {
                    return Err("surface material check cancelled".into());
                }
                if !charge(work, limit, 1) {
                    return Ok(None);
                }
                if sides != 3 && edge_meets_box(point(a), point(b), r) {
                    boundary = true;
                    break;
                }
            }
            // Every boundary of this union is among the uncancelled edges.
            // Shared edges cancel only with triangles on BOTH sides, using
            // exact endpoint identity and robust orientation; no small gaps or
            // projected facets are rounded away. A connected box with one
            // covered point and no possible boundary crossing is fully covered.
            if !boundary {
                return Ok(Some([self.floor.max(lower), upper]));
            }
        }
        Ok(Some([self.floor, upper]))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::AtomicBool;

    /// A 20 x 20 plate, top Z10, base Z0, with a diamond hole around (10, 10)
    /// (corners 4 mm from its centre) whose floor is `hole_floor` (0: through).
    /// The bounding box of the top facet (0,0)-(20,0)-(14,10) overlaps the
    /// hole, while the facet itself ends at the hole's edge.
    fn plate(hole_floor: f64) -> PreparedSurface {
        let o = [[0., 0.], [20., 0.], [20., 20.], [0., 20.]];
        let i = [[10., 6.], [14., 10.], [10., 14.], [6., 10.]];
        let at = |p: [f64; 2], z: f64| [p[0], p[1], z];
        let mut triangles = vec![];
        for k in 0..4 {
            let n = (k + 1) % 4;
            triangles.push([at(o[k], 10.), at(o[n], 10.), at(i[n], 10.)]);
            triangles.push([at(o[k], 10.), at(i[n], 10.), at(i[k], 10.)]);
            // Hole walls (vertical), down to the hole floor.
            triangles.push([at(i[k], 10.), at(i[n], 10.), at(i[n], hole_floor)]);
            triangles.push([at(i[k], 10.), at(i[n], hole_floor), at(i[k], hole_floor)]);
        }
        if hole_floor > 0. {
            triangles.push([
                at(i[0], hole_floor),
                at(i[1], hole_floor),
                at(i[2], hole_floor),
            ]);
            triangles.push([
                at(i[0], hole_floor),
                at(i[2], hole_floor),
                at(i[3], hole_floor),
            ]);
        }
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: triangles.into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(0., 1e-5, &AtomicBool::new(false))
        .unwrap()
    }
    fn bounds(surface: &PreparedSurface, xy: [f64; 4]) -> [f64; 2] {
        surface
            .material_bounds(xy, 1e-5, &mut 0, u64::MAX, &AtomicBool::new(false))
            .unwrap()
            .unwrap()
    }

    #[test]
    fn a_cell_inside_a_through_hole_is_bounded_by_the_base_not_the_top_face() {
        let through = plate(0.);
        // Wholly inside the hole: no facet of the part above it.
        assert_eq!(bounds(&through, [10.5, 9., 11.5, 9.8]), [0., 0.]);
        assert_eq!(bounds(&through, [9., 9., 11., 11.]), [0., 0.]);
        // Touching or straddling a hole wall: the wall and top face count.
        assert_eq!(bounds(&through, [12., 9., 13., 10.])[1], 10.);
        assert_eq!(bounds(&through, [12.5, 8., 13.5, 9.])[1], 10.);
        // On the plate: the top face.
        assert_eq!(bounds(&through, [1., 1., 2., 2.])[1], 10.);
    }

    #[test]
    fn a_blind_hole_keeps_its_floor_as_the_bound() {
        let blind = plate(5.);
        let inside = bounds(&blind, [10.5, 9., 11.5, 9.8]);
        assert!(inside[1] >= 5., "{inside:?}");
        assert!(inside[1] < 10., "{inside:?}");
    }

    #[test]
    fn the_projection_test_never_separates_a_touching_facet() {
        let t = [[0., 0., 1.], [4., 0., 1.], [0., 4., 1.]];
        let area = orientation(t[0], t[1], t[2], 2);
        // Box touching the hypotenuse at one point, beyond it, and across.
        assert!(projection_meets_box(t, area, [2., 2., 3., 3.]));
        assert!(!projection_meets_box(t, area, [2.0001, 2.0001, 3., 3.]));
        assert!(projection_meets_box(t, area, [1., 1., 3., 3.]));
        // Either winding.
        let r = [t[0], t[2], t[1]];
        assert!(projection_meets_box(r, -area, [2., 2., 3., 3.]));
        assert!(!projection_meets_box(r, -area, [2.0001, 2.0001, 3., 3.]));
        // A vertical facet projects to a segment.
        let wall = [[0., 0., 0.], [4., 4., 0.], [4., 4., 5.]];
        assert!(projection_meets_box(wall, 0., [1.5, 1.5, 2.5, 2.5]));
        assert!(!projection_meets_box(wall, 0., [2.5, 0., 3.5, 1.]));
    }
    fn repeated_surface(count: usize) -> PreparedSurface {
        SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![[[-2., -2., 1.], [2., -2., 1.], [0., 2., 1.]]; count].into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(0., 1e-6, &|| false)
        .unwrap()
    }

    #[test]
    fn candidate_sort_is_charged_and_preserves_the_whole_cell_enclosure() {
        let surface = repeated_surface(128);
        let mut work = 0;
        assert!(
            surface
                .material_bounds([-0.1, -0.1, 0.1, 0.1], 1e-6, &mut work, 500, &|| false)
                .unwrap()
                .is_none()
        );
        assert!(work <= 500);
        assert_eq!(
            surface
                .material_bounds([-0.1, -0.1, 0.1, 0.1], 1e-6, &mut 0, 100000, &|| false)
                .unwrap(),
            Some([1., 1.])
        );
    }

    #[test]
    fn material_query_cancellation_stops_inside_the_shared_candidate_sort() {
        use std::cell::Cell;
        let surface = repeated_surface(128);
        let polls = Cell::new(0usize);
        let cancel_at = surface.nodes.len() + surface.triangles.len() + 2;
        let stack_capacity = (usize::BITS - surface.triangles.len().leading_zeros()) as usize + 1;
        let before_sort = stack_capacity + surface.nodes.len() + 2 * surface.triangles.len();
        let mut work = 0;
        let error = surface
            .material_bounds([-0.1, -0.1, 0.1, 0.1], 1e-6, &mut work, 100000, &|| {
                polls.set(polls.get() + 1);
                polls.get() >= cancel_at
            })
            .unwrap_err();
        assert!(error.contains("cancelled"), "{error}");
        assert_eq!(polls.get(), cancel_at);
        assert_eq!(
            work,
            before_sort as u64 + 1,
            "cancelled outside candidate sorting"
        );
    }

    #[test]
    fn query_scratch_and_edge_allocations_are_guarded_with_the_resident_index() {
        let surface = repeated_surface(1);
        let resident = preparation_bytes(surface.triangles.capacity(), surface.nodes.capacity());
        let stack_bytes = 2 * std::mem::size_of::<usize>() as u128;
        for budget in [
            resident + stack_bytes,
            resident
                + stack_bytes
                + std::mem::size_of::<(f64, usize)>() as u128
                + EDGE_STORAGE_BYTES
                - 1,
        ] {
            let error = crate::resources::with_memory_budget(budget as u64, || {
                surface.material_bounds([-0.1, -2.1, 0.1, -1.9], 1e-6, &mut 0, 100000, &|| false)
            })
            .unwrap_err();
            assert!(error.contains("surface material query needs"), "{error}");
        }
        let error =
            crate::resources::with_memory_budget((resident + stack_bytes + 4096) as u64, || {
                surface.material_bounds_with_memory(
                    [-0.1, -0.1, 0.1, 0.1],
                    1e-6,
                    &mut 0,
                    100000,
                    4096,
                    &|| false,
                )
            })
            .unwrap_err();
        assert!(error.contains("surface material query needs"), "{error}");
    }
    #[test]
    fn interior_cells_do_not_build_an_irrelevant_boundary_tree() {
        let surface = repeated_surface(1);
        let mut work = 0;
        assert_eq!(
            surface
                .material_bounds([-0.1, -0.1, 0.1, 0.1], 1e-6, &mut work, 16, &|| false)
                .unwrap(),
            Some([1., 1.])
        );
        assert!(work <= 16);
    }

    #[test]
    fn disjoint_facet_projections_do_not_allocate_or_sort_lower_candidates() {
        let surface = SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: vec![[[-1., -1., 1.], [1., -1., 1.], [-1., 1., 1.]]; 128].into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        }
        .prepare(0., 1e-6, &|| false)
        .unwrap();
        let mut work = 0;
        // The query is within every facet AABB but strictly beyond x+y=0.
        assert_eq!(
            surface
                .material_bounds([0.5, 0.5, 0.6, 0.6], 1e-6, &mut work, 200, &|| false)
                .unwrap(),
            Some([0., 0.])
        );
        assert!(work <= 200);
    }

    #[test]
    fn a_closed_box_touching_a_facet_edge_keeps_its_possible_material() {
        let surface = repeated_surface(1);
        // Outside the facet except for its closed top edge at y=-2.
        assert_eq!(
            surface
                .material_bounds([-0.1, -2.1, 0.1, -2.], 0., &mut 0, 1000, &|| false)
                .unwrap(),
            Some([0., 1.])
        );
    }
}
