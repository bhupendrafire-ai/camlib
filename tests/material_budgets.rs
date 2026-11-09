//! The material check's work budgets (#108): unset, they grow with the
//! stock grid to the most work the check can need; a check stopped by a
//! budget names it, and the value it gives completes the check.
use camlib::{
    material::*,
    planar::{Boundary, Component, PlanarRegion},
    stock::{HeightBounds, Stock, StockSpec},
    surface::SurfaceMesh,
};
use std::sync::atomic::AtomicBool;

fn spec(size: [f64; 2], cell: f64) -> StockSpec {
    StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0., 0., 0.],
        max_mm: [size[0], size[1], 21.],
        cell_size_mm: cell,
        max_cells: 4_000_000,
    }
}
fn stock(spec: StockSpec, h: f64) -> Stock {
    let mut snapshot = Stock::new(spec).unwrap().snapshot();
    snapshot.cells.fill(HeightBounds {
        lower_mm: h,
        upper_mm: h,
    });
    Stock::from_snapshot(snapshot).unwrap()
}
fn quad(p: [[f64; 3]; 4]) -> Vec<[[f64; 3]; 3]> {
    vec![[p[0], p[1], p[2]], [p[0], p[2], p[3]]]
}
/// The plate's top (Z 20) as an `n` x `n` grid of facets over the stock.
fn plate(size: [f64; 2], n: usize) -> SurfaceTarget {
    let mut triangles = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let x = |k: usize| size[0] * k as f64 / n as f64;
            let y = |k: usize| size[1] * k as f64 / n as f64;
            triangles.extend(quad([
                [x(i), y(j), 20.],
                [x(i + 1), y(j), 20.],
                [x(i + 1), y(j + 1), 20.],
                [x(i), y(j + 1), 20.],
            ]));
        }
    }
    SurfaceTarget {
        version: 1,
        units: "mm".into(),
        id: "plate".into(),
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
fn pocket() -> Target {
    let ring = |c: [f64; 2], r: f64, n: usize| Boundary {
        vertices: (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                [c[0] + r * a.cos(), c[1] + r * a.sin(), 0.]
            })
            .collect(),
    };
    Target {
        version: 1,
        units: "mm".into(),
        id: "pocket".into(),
        revision: "1".into(),
        machining_region: PlanarRegion {
            version: 1,
            units: "mm".into(),
            name: "pocket".into(),
            boundary_error_mm: 0.,
            source: serde_json::Value::Null,
            components: vec![Component {
                outer: ring([20., 15.], 12., 64),
                holes: vec![],
            }],
        },
        floor_z_mm: 12.,
        floor_error_mm: 0.,
    }
}
fn options(tiles: Option<u64>, tests: Option<u64>) -> Options {
    Options {
        max_tile_nodes: tiles,
        max_boundary_tests: tests,
        ..Options::default()
    }
}
/// Gouge and residual verdicts, volumes and cell counts of a report.
fn verdict(r: &Report) -> serde_json::Value {
    serde_json::json!([r.complete, r.residual, r.gouge, r.stock_sha256])
}

#[test]
fn the_tile_tree_count_matches_the_traversal_it_bounds() {
    // An independent count: split every tile above a cell as the check
    // does (halves w / 2 and w - w / 2), without memoising.
    fn walk(w: usize, h: usize) -> u64 {
        if w == 1 && h == 1 {
            return 1;
        }
        let (xs, ys) = (
            if w > 1 {
                vec![w / 2, w - w / 2]
            } else {
                vec![w]
            },
            if h > 1 {
                vec![h / 2, h - h / 2]
            } else {
                vec![h]
            },
        );
        1 + ys
            .iter()
            .flat_map(|&a| xs.iter().map(move |&b| walk(b, a)))
            .sum::<u64>()
    }
    for w in 1..=40 {
        for h in 1..=40 {
            assert_eq!(tile_tree_nodes(w, h), walk(w, h), "{w} x {h}");
        }
    }
    assert_eq!(tile_tree_nodes(1, 1), 1);
}

#[test]
fn a_fine_grid_completes_with_default_budgets_and_a_stop_names_the_value_that_completes() {
    // #72's stock (110 x 70) at 0.1 mm: 770 000 cells, more tile nodes
    // than the old fixed default of 1 000 000.
    let spec = spec([110., 70.], 0.1);
    let (nx, ny) = (1100, 700);
    let needed = tile_tree_nodes(nx, ny);
    assert!(needed > 1_000_000 && needed <= MAX_TILE_NODES, "{needed}");
    let stock = stock(spec, 20.);
    let cancel = AtomicBool::new(false);
    let target = plate([110., 70.], 1);
    let check = |o: Options| {
        target
            .prepare(o)
            .unwrap()
            .check_stock(&stock, &cancel)
            .unwrap()
    };
    let full = check(Options::default());
    assert!(full.complete, "{:?}", full.stats);
    assert_eq!(full.stats.tile_nodes, needed);
    assert_eq!(full.stats.max_tile_nodes, needed);
    assert!(full.stats.stopped_by.is_none() && !full.stats.work_exhausted);

    let stopped = check(options(Some(1_000_000), None));
    assert!(!stopped.complete && stopped.stats.work_exhausted);
    let stop = stopped.stats.stopped_by.clone().unwrap();
    assert_eq!(
        (
            stop.budget.as_str(),
            stop.limit,
            stop.completes_at,
            stop.allowed_maximum
        ),
        ("max_tile_nodes", 1_000_000, Some(needed), MAX_TILE_NODES)
    );
    assert!(stop.message.contains("max_tile_nodes") && stop.message.contains(&needed.to_string()));
    let rerun = check(options(Some(needed), None));
    assert!(rerun.complete);
    assert_eq!(verdict(&rerun), verdict(&full));

    // The boundary-test bound of a small mesh lies within the maximum:
    // stopped early, the value it names completes the check.
    let stopped = check(options(None, Some(1000)));
    let stop = stopped.stats.stopped_by.clone().unwrap();
    assert_eq!(stop.budget, "max_boundary_tests");
    let completes = stop.completes_at.unwrap();
    assert!(completes <= MAX_BOUNDARY_TESTS);
    assert_eq!(full.stats.max_boundary_tests, completes);
    let rerun = check(options(None, Some(completes)));
    assert!(rerun.complete && rerun.stats.estimated_boundary_tests <= completes);
    assert_eq!(verdict(&rerun), verdict(&full));
}

#[test]
fn a_prismatic_boundary_stop_names_the_value_that_completes() {
    let spec = spec([40., 30.], 0.25);
    let (nx, ny) = (160, 120);
    let stock = stock(spec, 12.);
    let cancel = AtomicBool::new(false);
    let target = pocket();
    let check = |o: Options| {
        target
            .prepare(o)
            .unwrap()
            .check_stock(&stock, &cancel)
            .unwrap()
    };
    let full = check(Options::default());
    assert!(full.complete);
    // Every tile node charges the 64 segments once.
    assert_eq!(full.stats.max_boundary_tests, tile_tree_nodes(nx, ny) * 64);
    let stopped = check(options(None, Some(64 * 10)));
    assert!(!stopped.complete);
    let stop = stopped.stats.stopped_by.unwrap();
    assert_eq!(stop.budget, "max_boundary_tests");
    let rerun = check(options(None, stop.completes_at));
    assert!(rerun.complete);
    assert_eq!(verdict(&rerun), verdict(&full));
    // The classified-cell cap names itself and the grid's cell count.
    let capped = target
        .prepare(Options {
            max_classified_cells: 5,
            ..Options::default()
        })
        .unwrap()
        .check_stock(&stock, &cancel)
        .unwrap();
    let stop = capped.stats.stopped_by.unwrap();
    assert_eq!(
        (stop.budget.as_str(), stop.limit, stop.completes_at),
        ("max_classified_cells", 5, Some((nx * ny) as u64))
    );
}

#[test]
fn a_stop_without_a_known_bound_says_what_to_raise_or_coarsen() {
    // 20 000 facets: the per-cell bound is quadratic in the facets, far
    // past the allowed maximum, so the default is the maximum itself.
    let spec = spec([40., 30.], 1.);
    let stock = stock(spec, 20.);
    let cancel = AtomicBool::new(false);
    let target = plate([40., 30.], 100);
    let check = |o: Options| {
        target
            .prepare(o)
            .unwrap()
            .check_stock(&stock, &cancel)
            .unwrap()
    };
    let full = check(Options::default());
    assert!(full.complete);
    assert_eq!(full.stats.max_boundary_tests, MAX_BOUNDARY_TESTS);
    let stop = check(options(None, Some(10))).stats.stopped_by.unwrap();
    assert_eq!(
        (stop.budget.as_str(), stop.completes_at),
        ("max_boundary_tests", None)
    );
    assert!(
        stop.message.contains("raise max_boundary_tests"),
        "{}",
        stop.message
    );
    let stop = check(options(None, Some(MAX_BOUNDARY_TESTS)))
        .stats
        .stopped_by;
    assert!(stop.is_none());
    // Out-of-range budgets are refused, unset ones are not serialized.
    assert!(
        target
            .prepare(options(Some(MAX_TILE_NODES + 1), None))
            .is_err()
    );
    assert!(target.prepare(options(None, Some(0))).is_err());
    let json = serde_json::to_value(Options::default()).unwrap();
    assert!(json.get("max_tile_nodes").is_none() && json.get("max_boundary_tests").is_none());
    let parsed: Options =
        serde_json::from_str(r#"{"max_tile_nodes":null,"max_boundary_tests":5}"#).unwrap();
    assert_eq!(
        (parsed.max_tile_nodes, parsed.max_boundary_tests),
        (None, Some(5))
    );
}
