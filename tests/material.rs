use camlib::{
    buffers::HeightRead,
    material::*,
    planar::{Boundary, Component, PlanarRegion},
    stock::{HeightBounds, Stock, StockSnapshot, StockSpec},
};
use std::{cell::Cell, sync::atomic::AtomicBool};

fn spec() -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [4., 4., 10.],
        cell_size_mm: 1.,
        max_cells: 1_048_576,
    }
}
fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "target".into(),
        boundary_error_mm: 0.,
        source: serde_json::Value::Null,
        components: vec![Component {
            outer: Boundary {
                vertices: vec![[x0, y0, 0.], [x1, y0, 0.], [x1, y1, 0.], [x0, y1, 0.]],
            },
            holes: vec![],
        }],
    }
}
fn target(region: PlanarRegion) -> Target {
    Target {
        version: 1,
        units: "mm".into(),
        id: "target".into(),
        revision: "cad-1".into(),
        machining_region: region,
        floor_z_mm: 5.,
        floor_error_mm: 0.,
    }
}
fn uniform(stock: StockSpec, lower: f64, upper: f64) -> Stock {
    let cells = Stock::new(stock.clone()).unwrap().cells().len();
    Stock::from_snapshot(StockSnapshot {
        spec: stock,
        generation: 7,
        cells: vec![
            HeightBounds {
                lower_mm: lower,
                upper_mm: upper
            };
            cells
        ],
    })
    .unwrap()
}
fn enclose(range: [f64; 2], exact: f64) {
    assert!(
        range[0] <= exact && exact <= range[1],
        "{range:?} misses{exact}"
    );
}

#[test]
fn independent_uniform_floor_volumes_residual_gouge_and_small_volume_tolerance() {
    let t = target(rectangle(-2., -2., 6., 6.))
        .prepare(Options::default())
        .unwrap();
    let cancel = AtomicBool::new(false);
    let r = t.check_stock(&uniform(spec(), 10., 10.), &cancel).unwrap();
    assert!(r.complete);
    assert_eq!(r.residual.assessment, Assessment::DefiniteExcess);
    assert_eq!(r.residual.definite_cells, 16);
    enclose(r.residual.volume_mm3, 80.);
    assert_eq!(r.gouge.assessment, Assessment::WithinTolerance);
    enclose(r.gouge.volume_mm3, 0.);
    let r = t.check_stock(&uniform(spec(), 2., 2.), &cancel).unwrap();
    assert_eq!(r.gouge.definite_cells, 16);
    enclose(r.gouge.volume_mm3, 48.);
    assert_eq!(r.residual.assessment, Assessment::WithinTolerance);
    let r = t
        .check_stock(&uniform(spec(), 5.0001, 5.0001), &cancel)
        .unwrap();
    assert_eq!(r.residual.assessment, Assessment::WithinTolerance);
    // Do not discard physical volume merely because height is within tolerance.
    enclose(r.residual.volume_mm3, 16. * 0.0001);
    assert!(r.residual.volume_mm3[0] > 0.);
}

#[test]
fn stock_uncertainty_and_floor_source_error_are_possible_not_definite() {
    let mut definition = target(rectangle(-2., -2., 6., 6.));
    definition.floor_error_mm = 0.1;
    let t = definition.prepare(Options::default()).unwrap();
    let r = t
        .check_stock(&uniform(spec(), 4.95, 5.05), &AtomicBool::new(false))
        .unwrap();
    assert_eq!(r.residual.assessment, Assessment::PossibleExcess);
    assert_eq!(r.gouge.assessment, Assessment::PossibleExcess);
    assert_eq!(r.residual.definite_cells, 0);
    assert_eq!(r.gouge.definite_cells, 0);
    enclose(r.residual.volume_mm3, 0.);
    enclose(r.gouge.volume_mm3, 0.);
}

#[test]
fn mixed_cell_target_never_becomes_center_only_clearance_and_retains_island() {
    let t = target(rectangle(1., 1., 3., 3.))
        .prepare(Options::default())
        .unwrap();
    let r = t
        .check_stock(&uniform(spec(), 10., 10.), &AtomicBool::new(false))
        .unwrap();
    // Exact target cut is2x2x5=20; coarse cells can only enclose it.
    enclose(r.residual.volume_mm3, 20.);
    assert!(r.stats.mixed_cells > 0);
    assert_ne!(r.residual.assessment, Assessment::WithinTolerance);
    let mut region = rectangle(-2., -2., 6., 6.);
    region.components[0].holes.push(Boundary {
        vertices: vec![[1., 1., 0.], [3., 1., 0.], [3., 3., 0.], [1., 3., 0.]],
    });
    let r = target(region)
        .prepare(Options::default())
        .unwrap()
        .check_stock(&uniform(spec(), 5., 5.), &AtomicBool::new(false))
        .unwrap();
    // Island must remain at10; actual flat5 removes exactly4*5=20 from it.
    enclose(r.gouge.volume_mm3, 20.);
    assert_ne!(r.gouge.assessment, Assessment::WithinTolerance);
}

#[test]
fn circular_target_volume_has_independent_area_oracle_and_bounded_queries() {
    let mut s = spec();
    s.max_mm = [100., 100., 10.];
    s.cell_size_mm = 0.2;
    let mut def = target(PlanarRegion::circle("round", [50., 50.], 20.).unwrap());
    def.floor_z_mm = 2.;
    let r = def
        .prepare(Options::default())
        .unwrap()
        .check_stock(&uniform(s, 10., 10.), &AtomicBool::new(false))
        .unwrap();
    let exact = std::f64::consts::PI * 20. * 20. * 8.;
    enclose(r.residual.volume_mm3, exact);
    assert!(r.complete);
    assert!(r.residual.volume_mm3[1] - r.residual.volume_mm3[0] < exact * 0.04);
    assert!(r.stats.geometry_queries < r.stats.classified_cells as u64 / 10);
    assert_eq!(r.gouge.assessment, Assessment::WithinTolerance);
}

#[test]
fn capped_analysis_keeps_upper_volume_and_findings_truncation_does_not_change_summary() {
    let definition = target(rectangle(-2., -2., 6., 6.));
    let stock = uniform(spec(), 10., 10.);
    let cancel = AtomicBool::new(false);
    let full = definition
        .prepare(Options::default())
        .unwrap()
        .check_stock(&stock, &cancel)
        .unwrap();
    let limited = definition
        .prepare(Options {
            max_findings: 1,
            ..Options::default()
        })
        .unwrap()
        .check_stock(&stock, &cancel)
        .unwrap();
    assert!(limited.complete && limited.stats.findings_truncated);
    assert_eq!(limited.findings.len(), 1);
    assert_eq!(limited.residual.volume_mm3, full.residual.volume_mm3);
    assert_eq!(
        limited.residual.definite_cells,
        full.residual.definite_cells
    );
    let capped = definition
        .prepare(Options {
            max_classified_cells: 1,
            ..Options::default()
        })
        .unwrap()
        .check_stock(&stock, &cancel)
        .unwrap();
    assert!(!capped.complete);
    assert_eq!(capped.stats.unexamined_cells, 15);
    assert_eq!(capped.residual.assessment, Assessment::Incomplete);
    assert_eq!(capped.gouge.assessment, Assessment::Incomplete);
    enclose(capped.residual.volume_mm3, 80.);
    let capped = definition
        .prepare(Options {
            max_boundary_tests: Some(1),
            ..Options::default()
        })
        .unwrap()
        .check_stock(&stock, &cancel)
        .unwrap();
    assert!(!capped.complete);
    assert_eq!(capped.stats.classified_cells, 0);
    enclose(capped.residual.volume_mm3, 80.);
}

struct Buffer(Vec<f64>);
impl HeightRead for Buffer {
    fn scalar_len(&self) -> usize {
        self.0.len()
    }
    fn read(&self, i: usize) -> f64 {
        self.0[i]
    }
}
#[test]
fn malformed_late_input_is_rejected_even_with_early_work_cap_and_cancel_has_no_result() {
    let t = target(rectangle(-2., -2., 6., 6.))
        .prepare(Options {
            max_classified_cells: 1,
            ..Options::default()
        })
        .unwrap();
    let context = Context {
        stock: spec(),
        generation: 1,
        fraction: 0.,
        run: None,
    };
    let mut buffer = Buffer(vec![10.; 32]);
    buffer.0[31] = f64::NAN;
    assert!(
        t.check(&buffer, &context, &AtomicBool::new(false))
            .unwrap_err()
            .contains("height")
    );
    let before = buffer.0.clone();
    let polls = Cell::new(0);
    let cancel = || {
        polls.set(polls.get() + 1);
        polls.get() > 2
    };
    buffer.0[31] = 10.;
    assert!(
        t.check(&buffer, &context, &cancel)
            .unwrap_err()
            .contains("cancelled")
    );
    assert_eq!(buffer.0[..31], before[..31]);
}

#[test]
fn identities_bind_target_options_cursor_generation_and_actual_bounds() {
    let definition = target(rectangle(-2., -2., 6., 6.));
    let t = definition.prepare(Options::default()).unwrap();
    let stock = uniform(spec(), 10., 10.);
    let cancel = AtomicBool::new(false);
    let context = Context {
        stock: spec(),
        generation: 7,
        fraction: 0.5,
        run: None,
    };
    let a = t.check(&stock, &context, &cancel).unwrap();
    let mut changed = context.clone();
    changed.fraction = 1.;
    let b = t.check(&stock, &changed, &cancel).unwrap();
    assert_ne!(a.stock_sha256, b.stock_sha256);
    let c = t
        .check(&uniform(spec(), 9., 10.), &context, &cancel)
        .unwrap();
    assert_ne!(a.stock_sha256, c.stock_sha256);
    assert_ne!(
        t.digest(),
        definition
            .prepare(Options {
                max_findings: 1,
                ..Options::default()
            })
            .unwrap()
            .digest()
    );
    assert_eq!(a.context.generation, 7);
    assert_eq!(a.context.fraction, 0.5);
}

#[test]
fn empty_target_preserves_stock_and_invalid_definitions_contexts_are_rejected() {
    let mut region = rectangle(-2., -2., 6., 6.);
    region.components.clear();
    let t = target(region).prepare(Options::default()).unwrap();
    let r = t
        .check_stock(&uniform(spec(), 10., 10.), &AtomicBool::new(false))
        .unwrap();
    assert!(r.complete);
    assert_eq!(r.residual.assessment, Assessment::WithinTolerance);
    assert_eq!(r.gouge.assessment, Assessment::WithinTolerance);
    assert!(
        target(rectangle(-2., -2., 6., 6.))
            .prepare(Options {
                max_tile_nodes: Some(0),
                ..Options::default()
            })
            .is_err()
    );
    let mut bad = target(rectangle(-2., -2., 6., 6.));
    bad.floor_z_mm = -1.;
    assert!(
        bad.prepare(Options::default())
            .unwrap()
            .check_stock(&uniform(spec(), 10., 10.), &AtomicBool::new(false))
            .is_err()
    );
}
