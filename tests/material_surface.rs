use camlib::{
    material::*,
    stock::{HeightBounds, Stock, StockSpec},
    surface::SurfaceMesh,
};
use std::sync::atomic::AtomicBool;
fn plane(x0: f64, x1: f64, z: impl Fn(f64) -> f64) -> Vec<[[f64; 3]; 3]> {
    let p = [
        [x0, -2., z(x0)],
        [x1, -2., z(x1)],
        [x1, 6., z(x1)],
        [x0, 6., z(x0)],
    ];
    vec![[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
}
fn target(triangles: Vec<[[f64; 3]; 3]>) -> SurfaceTarget {
    SurfaceTarget {
        version: 1,
        units: "mm".into(),
        id: "surface".into(),
        revision: "1".into(),
        surface: SurfaceMesh {
            version: 1,
            units: "mm".into(),
            triangles: triangles.into(),
            surface_error_mm: 0.,
            source: serde_json::Value::Null,
        },
        base_z_mm: 0.,
        source_enclosure: SurfaceEnclosure::Bounded,
        contact_witnesses: None,
        inner_patches: vec![],
        finishing: None,
    }
}
fn stock(h: f64) -> Stock {
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0.; 3],
        max_mm: [4., 4., 10.],
        cell_size_mm: 1.,
        max_cells: 100,
    };
    let mut snapshot = Stock::new(spec).unwrap().snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: h,
        upper_mm: h,
    });
    Stock::from_snapshot(snapshot).unwrap()
}
#[test]
fn inclined_whole_cells_and_shared_diagonal_bound_independent_plane_volume() {
    let definition = target(plane(-2., 6., |x| 2. + x / 4.));
    let t = definition.prepare(Options::default()).unwrap();
    let r = t.check_stock(&stock(10.), &AtomicBool::new(false)).unwrap();
    assert!(r.complete);
    assert_eq!(r.residual.definite_cells, 16);
    assert!(r.residual.volume_mm3[0] <= 120. && r.residual.volume_mm3[1] >= 120.);
    for f in r.findings {
        let lo = 2. + f.xy_mm[0] / 4.;
        let hi = 2. + f.xy_mm[2] / 4.;
        assert!(
            f.target_height_mm[0] <= lo && f.target_height_mm[0] > lo - 0.0001,
            "{f:?}"
        );
        assert!(
            f.target_height_mm[1] >= hi && f.target_height_mm[1] < hi + 0.0001,
            "{f:?}"
        );
    }
    let gouged = t.check_stock(&stock(1.), &AtomicBool::new(false)).unwrap();
    assert_eq!(gouged.gouge.definite_cells, 16);
}
#[test]
fn narrow_gap_peak_vertical_facet_and_overlaps_never_disappear() {
    let mut mesh = plane(-2., 1.49999, |_| 3.);
    mesh.extend(plane(1.50001, 6., |_| 3.));
    mesh.push([[2.11, 2.11, 3.], [2.12, 2.11, 8.], [2.11, 2.12, 3.]]);
    mesh.push([[3.2, 2.1, 0.], [3.2, 2.2, 0.], [3.2, 2.15, 9.]]);
    // Hidden lower plane cannot lower a proved upper-envelope enclosure.
    mesh.extend(plane(-2., 6., |_| 1.));
    let t = target(mesh).prepare(Options::default()).unwrap();
    let r = t.check_stock(&stock(10.), &AtomicBool::new(false)).unwrap();
    for f in &r.findings {
        let x = f.cell_index % 4;
        let y = f.cell_index / 4;
        if x == 1 {
            assert!(f.target_height_mm[0] <= 1.);
        } else {
            assert!(f.target_height_mm[0] > 2.9999, "{f:?}");
        }
        if (x, y) == (2, 2) {
            assert!(f.target_height_mm[1] >= 8.);
        }
        if (x, y) == (3, 2) {
            assert!(f.target_height_mm[1] >= 9.);
        }
    }
}
#[test]
fn source_premises_work_exhaustion_cancellation_and_identity_remain_explicit() {
    let mut d = target(plane(-2., 6., |_| 4.));
    d.surface.surface_error_mm = 0.05;
    let bounded = d.prepare(Options::default()).unwrap();
    let r = bounded
        .check_stock(&stock(2.), &AtomicBool::new(false))
        .unwrap();
    assert_eq!(r.gouge.definite_cells, 0);
    assert_eq!(r.gouge.possible_only_cells, 16);
    d.source_enclosure = SurfaceEnclosure::Inscribed;
    let inner = d.prepare(Options::default()).unwrap();
    assert_ne!(bounded.digest(), inner.digest());
    assert_eq!(
        inner
            .check_stock(&stock(2.), &AtomicBool::new(false))
            .unwrap()
            .gouge
            .definite_cells,
        16
    );
    assert!(
        inner
            .check_stock(&stock(2.), &AtomicBool::new(true))
            .is_err()
    );
    assert!(
        d.prepare_cancel(Options::default(), &AtomicBool::new(true))
            .is_err()
    );
    let limited = d
        .prepare(Options {
            max_boundary_tests: Some(1),
            ..Options::default()
        })
        .unwrap();
    let r = limited
        .check_stock(&stock(2.), &AtomicBool::new(false))
        .unwrap();
    assert!(!r.complete);
    assert_eq!(r.stats.validated_cells, 16);
    assert_eq!(r.gouge.assessment, Assessment::Incomplete);
}

#[test]
fn translated_reversed_facets_and_sub_tolerance_holes_keep_whole_cell_bounds() {
    for reverse in [false, true] {
        let mut d = target(plane(-2., 6., |x| 2. + x / 4.));
        for tri in std::sync::Arc::make_mut(&mut d.surface.triangles) {
            for p in tri.iter_mut() {
                p[0] += 100_000.;
                p[1] -= 20_000.;
            }
            if reverse {
                tri.swap(0, 2);
            }
        }
        let mut snapshot = stock(10.).snapshot();
        snapshot.spec.min_mm[0] += 100_000.;
        snapshot.spec.max_mm[0] += 100_000.;
        snapshot.spec.min_mm[1] -= 20_000.;
        snapshot.spec.max_mm[1] -= 20_000.;
        let r = d
            .prepare(Options::default())
            .unwrap()
            .check_stock(
                &Stock::from_snapshot(snapshot).unwrap(),
                &AtomicBool::new(false),
            )
            .unwrap();
        for f in r.findings {
            let lo = 2. + (f.xy_mm[0] - 100_000.) / 4.;
            assert!(f.target_height_mm[0] <= lo && f.target_height_mm[0] > lo - 0.001);
        }
    }
    let mut mesh = plane(-2., 1.5 - 1e-9, |_| 4.);
    mesh.extend(plane(1.5 + 1e-9, 6., |_| 4.));
    let r = target(mesh)
        .prepare(Options::default())
        .unwrap()
        .check_stock(&stock(10.), &AtomicBool::new(false))
        .unwrap();
    assert!(
        r.findings
            .iter()
            .filter(|f| f.cell_index % 4 == 1)
            .all(|f| f.target_height_mm[0] == 0.)
    );
    let mut d = target(plane(-2., 6., |_| 4.));
    d.surface.surface_error_mm = 0.05;
    d.source_enclosure = SurfaceEnclosure::Offset;
    let r = d
        .prepare(Options::default())
        .unwrap()
        .check_stock(&stock(3.97), &AtomicBool::new(false))
        .unwrap();
    assert_eq!(r.gouge.definite_cells, 0);
    assert_eq!(r.gouge.possible_only_cells, 16);
}

#[test]
fn a_source_base_below_stock_keeps_exact_clipped_material_volumes() {
    let cancel = AtomicBool::new(false);
    for shift in [0., 100_000., -20_000.] {
        for base in [-f64::EPSILON, -3.] {
            let mut definition = target(plane(-2., 6., |_| shift + 2.));
            definition.base_z_mm = shift + base;
            let prepared = definition.prepare(Options::default()).unwrap();
            for height in [1., 4.] {
                let mut snapshot = stock(height).snapshot();
                snapshot.spec.min_mm[2] += shift;
                snapshot.spec.max_mm[2] += shift;
                for cell in &mut snapshot.cells {
                    cell.lower_mm += shift;
                    cell.upper_mm += shift;
                }
                let r = prepared
                    .check_stock(&Stock::from_snapshot(snapshot).unwrap(), &cancel)
                    .unwrap();
                assert!(r.complete);
                let (residual, gouge) = if height == 1. { (0., 16.) } else { (32., 0.) };
                for (bounds, exact) in [
                    (r.residual.volume_mm3, residual),
                    (r.gouge.volume_mm3, gouge),
                ] {
                    assert!(
                        bounds[0] <= exact && bounds[1] >= exact,
                        "{bounds:?} vs {exact}"
                    );
                    assert!(bounds[1] - bounds[0] < 0.01);
                }
                for finding in r.findings {
                    assert!(finding.target_height_mm[0] >= shift);
                    assert!(finding.target_height_mm[1] <= shift + 10.);
                }
            }
        }
    }
    // A sloped source passes through the analysis bottom. Independently,
    // integral_0^4 max(x-2,0) dx * 4 = 8 cubic mm of retained target.
    let mut definition = target(plane(-2., 6., |x| x - 2.));
    definition.base_z_mm = -5.;
    let r = definition
        .prepare(Options::default())
        .unwrap()
        .check_stock(&stock(4.), &cancel)
        .unwrap();
    assert!(r.residual.volume_mm3[0] <= 56. && r.residual.volume_mm3[1] >= 56.);
    assert_eq!(r.gouge.definite_cells, 0);
}

#[test]
fn local_mixed_cells_enclose_partial_area_without_inventing_positive_overlap() {
    use camlib::planar::{Boundary, Component, PlanarRegion};

    let mut snapshot = stock(6.).snapshot();
    snapshot.spec.max_mm = [1., 1., 10.];
    snapshot.cells.truncate(1);
    let cancel = AtomicBool::new(false);
    for (x0, x1, exact_volume) in [(0., 0.5, 0.5), (1.000001, 2., 0.)] {
        let definition = Target {
            version: 1,
            units: "mm".into(),
            id: "local-face".into(),
            revision: "1".into(),
            machining_region: PlanarRegion {
                version: 1,
                units: "mm".into(),
                name: "face".into(),
                boundary_error_mm: 0.,
                components: vec![Component {
                    outer: Boundary {
                        vertices: vec![[x0, 0., 0.], [x1, 0., 0.], [x1, 1., 0.], [x0, 1., 0.]],
                    },
                    holes: vec![],
                }],
                source: serde_json::Value::Null,
            },
            floor_z_mm: 5.,
            floor_error_mm: 0.,
        };
        let prepared = definition
            .prepare(Options {
                scope_to_region: true,
                geometry_tolerance_mm: 0.01,
                ..Options::default()
            })
            .unwrap();
        // One square millimetre cell: actual 6 minus floor 5, scoped to
        // exactly half its area, is 0.5 cubic mm. The other region only
        // touches the cell's conservative guard and has zero overlap.
        for (height, residual) in [(6., true), (4., false)] {
            snapshot.cells[0] = HeightBounds {
                lower_mm: height,
                upper_mm: height,
            };
            let report = prepared
                .check_stock(&Stock::from_snapshot(snapshot.clone()).unwrap(), &cancel)
                .unwrap();
            assert!(report.complete);
            assert_eq!(report.stats.mixed_cells, 1);
            let excess = if residual {
                &report.residual
            } else {
                &report.gouge
            };
            assert_eq!(excess.definite_cells, 0);
            assert_eq!(excess.assessment, Assessment::PossibleExcess);
            assert_eq!(excess.volume_mm3[0], 0.);
            assert!(excess.volume_mm3[1] >= exact_volume);
            assert!(excess.volume_mm3[1] < 1.000001);
            let finding = &report.findings[0];
            assert_eq!(finding.target_class, FootprintClass::Mixed);
            assert_eq!(finding.residual_height_mm[0], 0.);
            assert_eq!(finding.gouge_height_mm[0], 0.);
        }
    }
}
