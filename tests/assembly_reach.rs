//! Issue #129: a pocket's top below the remaining stock outside its region,
//! judged by the tool assembly; typed reach refusals; what a stock-contact
//! finding's part must clear.
use camlib::{
    motion::{Coolant, Curve, Motion, MoveKind},
    operation::OperationPlan,
    planar::{Boundary, Component, PlanarRegion},
    pocket::{Operation, PocketOperation},
    reach::{ReachLimit, ReachRefusal},
    stock::{HeightBounds, Stock, StockSpec, SweepOptions},
    stock_contact::{self, Options},
    tool::{Cutter, CutterShape, HolderSection, ToolAssembly},
    verification::{Status, ToolPart},
};
use std::sync::atomic::AtomicBool;

const WALL_X: f64 = 30.;
const WALL_TOP: f64 = 60.;
const SHELF: f64 = 8.;
const FLOOR: f64 = 4.;
const START: [f64; 3] = [5., 5., 70.];

fn never() -> AtomicBool {
    AtomicBool::new(false)
}
/// 60 x 40 mm stock, 60 mm tall: an earlier operation left a shelf at Z8
/// for x < 30 and a wall to the full height beyond it.
fn walled(upper: impl Fn([f64; 2]) -> f64) -> Stock {
    let spec = StockSpec {
        version: 1,
        units: "mm".into(),
        min_mm: [0.; 3],
        max_mm: [60., 40., WALL_TOP],
        cell_size_mm: 0.5,
        max_cells: 1 << 20,
    };
    let mut s = Stock::new(spec).unwrap().snapshot();
    for (i, h) in s.cells.iter_mut().enumerate() {
        let p = [(i % 120) as f64 * 0.5 + 0.25, (i / 120) as f64 * 0.5 + 0.25];
        let z = upper(p);
        *h = HeightBounds {
            lower_mm: z,
            upper_mm: z,
        };
    }
    Stock::from_snapshot(s).unwrap()
}
fn wall(p: [f64; 2]) -> f64 {
    if p[0] < WALL_X { SHELF } else { WALL_TOP }
}
/// A 6 mm end mill with 6 mm flutes, 15 mm stickout and one holder section.
fn tool(cutting_length: f64, holder_radius: f64) -> ToolAssembly {
    ToolAssembly {
        id: "em6".into(),
        cutter: Cutter {
            id: "c6".into(),
            name: "em6".into(),
            shape: CutterShape::Flat,
            diameter_mm: 6.,
            cutting_length_mm: cutting_length,
            overall_length_mm: 40.,
            shank_diameter_mm: 6.,
            flutes: 3,
            center_cutting: true,
            material: "carbide".into(),
            neck: None,
        },
        stickout_mm: 15.,
        gauge_length_mm: 60.,
        holders: vec![HolderSection {
            z_min_mm: 15.,
            z_max_mm: 60.,
            radius_min_mm: holder_radius,
            radius_max_mm: holder_radius,
        }],
    }
}
fn rectangle(x0: f64, y0: f64, x1: f64, y1: f64) -> PlanarRegion {
    PlanarRegion {
        version: 1,
        units: "mm".into(),
        name: "pocket".into(),
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
/// A 4 mm deep pocket whose edge is 8 mm from the 56 mm wall.
fn pocket(region: PlanarRegion, clearance: f64) -> Operation {
    Operation::Pocket(PocketOperation {
        max_stay_down_mm: None,
        limits: None,
        id: "low-pocket".into(),
        tool: "em6".into(),
        region,
        top_z_mm: SHELF,
        floor_z_mm: FLOOR,
        stepdown_mm: 2.,
        stepover_mm: 2.,
        feed_mm_min: 600.,
        plunge_mm_min: 150.,
        clearance_z_mm: clearance,
        radial_allowance_mm: 0.,
        bottom_allowance_mm: 0.,
        tolerance_mm: 1e-5,
        spindle_rpm: 12000.,
        work_offset: 1,
        tool_number: 1,
        length_register: 1,
        coolant: Coolant::Flood,
        rest_tolerance_mm: None,
        boundary_clockwise: None,
        entry: None,
        retract_z_mm: None,
        feed_z_mm: None,
    })
}
fn beside_wall() -> PlanarRegion {
    rectangle(10., 10., 22., 30.)
}
fn prepare(op: Operation, t: &ToolAssembly, stock: &Stock) -> Result<OperationPlan, String> {
    OperationPlan::prepare_with_stock(op, t, stock, START, 1, &never())
}
fn contact(stock: &Stock, t: &ToolAssembly, m: &Motion) -> stock_contact::Report {
    stock_contact::check(
        stock,
        &t.prepare().unwrap(),
        m,
        Options::default(),
        &never(),
    )
    .unwrap()
}
fn cut(a: [f64; 3], b: [f64; 3]) -> Motion {
    Motion {
        id: 1,
        operation: "low-pocket".into(),
        tool: "em6".into(),
        kind: MoveKind::Cut,
        curve: Curve::Line { start: a, end: b },
        feed_mm_min: Some(600.),
    }
}

#[test]
fn slim_assembly_reaches_low_pocket_beside_tall_wall_and_stock_contact_agrees() {
    let t = tool(6., 4.);
    let mut stock = walled(wall);
    stock.enable_contact_queries(&never()).unwrap();
    let mut plan = prepare(pocket(beside_wall(), 65.), &t, &stock).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert_eq!(report["stock_top"]["authored_top_z_mm"], SHELF);
    assert_eq!(report["stock_top"]["outside_upper_z_mm"], WALL_TOP);
    assert!(
        report["stock_top"]["assembly_checked_cells"]
            .as_u64()
            .unwrap()
            > 0
    );
    let prepared = t.prepare().unwrap();
    let mut motions = 0;
    while let Some(m) = plan.next_motion_with_stock(&stock, &never()).unwrap() {
        // The independent continuous check judges every move's assembly
        // (rapids whole, fed moves above the flutes) against current stock.
        let r = contact(&stock, &t, &m);
        assert_eq!(r.status, Status::Clear, "{:?} {:?}", m, r.findings);
        if m.kind == MoveKind::Cut {
            assert!(m.curve.start()[2] >= FLOOR - 1e-9 && m.curve.start()[2] <= SHELF);
        }
        let delta = stock
            .preview_sweep(&prepared, &m, 1., SweepOptions::default(), &never())
            .unwrap();
        stock.apply_delta(&delta).unwrap();
        motions += 1;
    }
    assert!(motions > 10);
    // The floor is cut and the wall is untouched.
    let cells = stock.cells();
    let at = |x: f64, y: f64| cells[(y / 0.5) as usize * 120 + (x / 0.5) as usize];
    assert!(at(16., 20.).upper_mm <= FLOOR + 1e-3);
    assert_eq!(at(45., 20.).upper_mm, WALL_TOP);
    assert_eq!(at(30.25, 20.).lower_mm, WALL_TOP);
}

#[test]
fn wide_holder_is_refused_before_any_motion_with_a_typed_reach_refusal() {
    let fat = tool(6., 20.);
    let stock = walled(wall);
    let error = prepare(pocket(beside_wall(), 65.), &fat, &stock)
        .err()
        .unwrap();
    assert!(error.contains("holder section 0"), "{error}");
    let r = ReachRefusal::parse(&error).expect(&error);
    assert_eq!(r.operation, "low-pocket");
    assert_eq!(r.tool, "em6");
    assert_eq!(r.limit, ReachLimit::Holder);
    assert_eq!(r.holder_section, Some(0));
    assert_eq!(r.top_z_mm, WALL_TOP);
    assert_eq!(r.z_mm, FLOOR);
    assert_eq!(r.depth_needed_mm, WALL_TOP - FLOOR);
    assert_eq!(r.depth_available_mm, 15.);

    // The independent check finds the same holder in the wall on a floor
    // stroke the slim assembly makes, and says what would clear.
    let mut stock = stock;
    stock.enable_contact_queries(&never()).unwrap();
    let stroke = cut([19., 13., FLOOR], [19., 27., FLOOR]);
    let report = contact(&stock, &fat, &stroke);
    assert_eq!(report.status, Status::Violation);
    let f = report
        .findings
        .iter()
        .find(|f| f.finding.part == Some(ToolPart::Holder))
        .unwrap();
    assert_eq!(f.finding.holder_section, Some(0));
    assert_eq!(f.obstruction_top_mm, Some(WALL_TOP));
    assert_eq!(f.lowest_tip_z_mm, Some(FLOOR));
    // The wall face is 11 mm from the axis; whole-cell distances bound it
    // from below by at most a cell's half diagonal.
    let clear = f.max_clear_radius_mm.unwrap();
    assert!(clear <= 11. && clear > 11. - 0.36, "{clear}");
    // A holder that slim clears; the slim test assembly's 4 mm does.
    assert_eq!(
        contact(&stock, &tool(6., 4.), &stroke).status,
        Status::Clear
    );
}

#[test]
fn short_flutes_get_a_typed_cutting_length_refusal() {
    let short = tool(3., 4.);
    let stock = walled(wall);
    let error = prepare(pocket(beside_wall(), 65.), &short, &stock)
        .err()
        .unwrap();
    assert!(
        error.contains("pocket depth 4 exceeds the cutting length 3"),
        "{error}"
    );
    let r = ReachRefusal::parse(&error).unwrap();
    assert_eq!(r.limit, ReachLimit::CuttingLength);
    assert_eq!(r.holder_section, None);
    assert_eq!((r.top_z_mm, r.z_mm), (SHELF, FLOOR));
    assert_eq!((r.depth_needed_mm, r.depth_available_mm), (4., 3.));
}

#[test]
fn stock_the_cutter_can_reach_and_low_clearance_are_still_refused() {
    let t = tool(6., 4.);
    let stock = walled(wall);
    // The region reaches into the wall's cells: the cutter would meet them.
    let error = prepare(pocket(rectangle(10., 10., 30.2, 30.), 65.), &t, &stock)
        .err()
        .unwrap();
    assert!(
        error.contains("below remaining stock") && error.contains("where the cutter can reach it"),
        "{error}"
    );
    assert!(ReachRefusal::parse(&error).is_none());
    // Rapids at a clearance plane below the wall would cross it.
    let error = prepare(pocket(beside_wall(), 30.), &t, &stock)
        .err()
        .unwrap();
    assert!(
        error.contains("clearance_z_mm 30 must be above the remaining stock"),
        "{error}"
    );
    // Without stock above the top, nothing changes: no outside report.
    let flat = walled(|_| SHELF);
    let plan = prepare(pocket(beside_wall(), 30.), &t, &flat).unwrap();
    let report = serde_json::to_value(plan.report()).unwrap();
    assert!(report["stock_top"].get("outside_upper_z_mm").is_none());
}

#[test]
fn sweep_reach_judges_cells_the_cutter_can_touch_not_its_bounding_box() {
    let t = tool(6., 4.).prepare().unwrap();
    // A diagonal floor cut; a tall column sits inside its bounding box but
    // 6.8 mm from its axis (cutter radius 3).
    let diagonal = cut([12., 12., FLOOR], [20., 28., FLOOR]);
    let far = walled(|p| {
        if (19.5..20.).contains(&p[0]) && (12. ..12.5).contains(&p[1]) {
            WALL_TOP
        } else {
            SHELF
        }
    });
    let delta = far
        .preview_sweep(&t, &diagonal, 1., SweepOptions::default(), &never())
        .unwrap();
    // The cut is swept; the tall column is not touched.
    assert!(delta.changes().iter().any(|c| c.after.upper_mm < SHELF));
    assert!(delta.changes().iter().all(|c| c.before.upper_mm == SHELF));
    // The same column where the cutter's side touches it is still refused.
    let near = walled(|p| {
        if (15.5..16.).contains(&p[0]) && (16. ..16.5).contains(&p[1]) {
            WALL_TOP
        } else {
            SHELF
        }
    });
    let error = near
        .preview_sweep(&t, &diagonal, 1., SweepOptions::default(), &never())
        .unwrap_err();
    assert!(error.contains("cutting length cannot span"), "{error}");
}
