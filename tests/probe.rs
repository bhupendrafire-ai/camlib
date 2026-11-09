//! Probing cycles (issue #7): the planned layout, the program contract,
//! stock that never changes, and stock contact with the stylus.
use camlib::{
    motion::*, operation::OperationPlan, pocket::Operation, probe::*, stock::*, stock_contact,
    tool::ToolAssembly, verification::Status,
};
use serde_json::json;
use std::sync::atomic::AtomicBool;

const BALL: f64 = 4.;

fn probe() -> ToolAssembly {
    serde_json::from_value(json!({
        "id": "probe",
        "cutter": {"id": "stylus", "shape": {"kind": "probe"}, "diameter_mm": BALL,
            "cutting_length_mm": BALL, "overall_length_mm": 110, "shank_diameter_mm": 3,
            "flutes": 1, "center_cutting": false},
        "stickout_mm": 50, "gauge_length_mm": 110,
        "holders": [{"z_min_mm": 50, "z_max_mm": 110, "radius_min_mm": 20, "radius_max_mm": 20}]
    }))
    .unwrap()
}

fn mill() -> ToolAssembly {
    serde_json::from_value(json!({
        "id": "mill",
        "cutter": {"id": "mill", "shape": {"kind": "flat"}, "diameter_mm": 10,
            "cutting_length_mm": 20, "overall_length_mm": 60, "shank_diameter_mm": 10,
            "flutes": 3, "center_cutting": true},
        "stickout_mm": 40, "gauge_length_mm": 80,
        "holders": [{"z_min_mm": 40, "z_max_mm": 80, "radius_min_mm": 15, "radius_max_mm": 15}]
    }))
    .unwrap()
}

fn spec() -> StockSpec {
    serde_json::from_value(json!({"version": 1, "units": "mm", "min_mm": [0, 0, 0],
        "max_mm": [40, 30, 10], "cell_size_mm": 0.25, "max_cells": 1048576}))
    .unwrap()
}

fn operation(feature: serde_json::Value, tool: &str) -> ProbeOperation {
    let stylus = tool == "probe";
    let mut op = json!({
        "id": "probing", "tool": tool, "tool_number": 1, "length_register": 1, "work_offset": 1,
        "clearance_z_mm": 20, "feature": feature, "search_mm": 2,
        "measure_feed_mm_min": 100, "return_feed_mm_min": 1000,
        "result": if stylus { json!({"kind": "work_offset", "number": 1}) }
            else if feature["kind"] == "tool_length" { json!({"kind": "tool_length", "register": 1}) }
            else { json!({"kind": "tool_radius", "register": 1}) },
    });
    if stylus {
        op["position_feed_mm_min"] = json!(1000);
    }
    serde_json::from_value(op).unwrap()
}

fn corner() -> serde_json::Value {
    json!({"kind": "outside_corner", "corner_mm": [0, 0], "z_mm": 5, "material": [1, 1],
        "inset_mm": 5, "clearance_mm": 5})
}

fn bore() -> serde_json::Value {
    json!({"kind": "bore", "center_mm": [20, 15], "z_mm": 5, "diameter_mm": 12})
}

/// Every move of a planned operation, with its events in stream order.
fn plan(op: ProbeOperation, tool: &ToolAssembly) -> Result<(Vec<Event>, Vec<Motion>), String> {
    let never = AtomicBool::new(false);
    let stock = Stock::new(spec())?;
    let mut plan = OperationPlan::prepare(
        Operation::Probe(op),
        tool,
        &spec(),
        [0., 0., 50.],
        1,
        &never,
    )?;
    let mut events = plan.prefix_events();
    let mut motions = vec![];
    while let Some(m) = plan.next_motion_with_stock(&stock, &never)? {
        // As a stream sends them: the move, then the events due after it.
        events.push(Event::Motion { motion: m.clone() });
        events.extend(plan.take_events());
        motions.push(m);
    }
    plan.finish_report(&never)?;
    events.extend(plan.suffix_events());
    Ok((events, motions))
}

fn program(tools: Vec<ToolAssembly>, events: Vec<Event>) -> Program {
    Program {
        version: 1,
        id: "probe".into(),
        revision: "1".into(),
        units: "mm".into(),
        tolerance_mm: 1e-6,
        frame: WorkFrame::identity(),
        initial_position_mm: [0., 0., 50.],
        tools,
        events,
    }
}

fn kinds(motions: &[Motion]) -> Vec<MoveKind> {
    motions.iter().map(|m| m.kind).collect()
}

#[test]
fn an_outside_corner_touches_y_then_x_from_its_clearance() {
    use MoveKind::*;
    let (events, motions) = plan(operation(corner(), "probe"), &probe()).unwrap();
    let r = BALL / 2.;
    // Rapids over the clearance plane, then the covered cycle.
    assert_eq!(
        kinds(&motions),
        [
            Rapid,
            Rapid,
            ProbePosition,
            ProbeMeasure,
            ProbeTouch,
            ProbeReturn,
            ProbePosition,
            ProbePosition,
            ProbePosition,
            ProbeMeasure,
            ProbeTouch,
            ProbeReturn,
            ProbePosition
        ]
    );
    // The Y face: inset 5 along it, from 5 + r below; the expected contact
    // leaves the ball tangent to y = 0, the touch zone is search_mm long.
    let end = |i: usize| motions[i].curve.end();
    assert_eq!(end(1), [5., -(r + 5.), 20.]);
    assert_eq!(end(3), [5., -r - 2., 5.]);
    assert_eq!(end(4), [5., -r, 5.]);
    assert_eq!(end(5), [5., -(r + 5.), 5.]);
    // The X face.
    assert_eq!(end(10), [-r, 5., 5.]);
    let cycle = events
        .iter()
        .position(|e| matches!(e, Event::Probe { covers: 11, .. }))
        .unwrap();
    // Due right before its first covered move.
    assert!(matches!(&events[cycle + 1], Event::Motion { motion } if motion.kind == ProbePosition));
    program(vec![probe()], events).validate(None).unwrap();
}

#[test]
fn a_bore_measures_both_axes_from_its_centre() {
    let (events, motions) = plan(operation(bore(), "probe"), &probe()).unwrap();
    let contacts: Vec<Vec3> = motions
        .iter()
        .filter(|m| m.kind == MoveKind::ProbeTouch)
        .map(|m| m.curve.end())
        .collect();
    let reach = 6. - BALL / 2.;
    assert_eq!(
        contacts,
        [
            [20. + reach, 15., 5.],
            [20. - reach, 15., 5.],
            [20., 15. + reach, 5.],
            [20., 15. - reach, 5.]
        ]
    );
    program(vec![probe()], events).validate(None).unwrap();
}

#[test]
fn probing_moves_never_change_the_stock() {
    let never = AtomicBool::new(false);
    for feature in [corner(), bore()] {
        let tool = probe().prepare().unwrap();
        let (_, motions) = plan(operation(feature, "probe"), &probe()).unwrap();
        let mut stock = Stock::new(spec()).unwrap();
        for m in &motions {
            // The stylus ball ends inside the cell bounds at each contact;
            // still nothing is removed.
            let delta = stock
                .preview_sweep(&tool, m, 1., SweepOptions::default(), &never)
                .unwrap();
            assert!(delta.changes().is_empty(), "{:?} changed stock", m.kind);
            stock.apply_delta(&delta).unwrap();
        }
        assert_eq!(stock.volume().removed_upper_mm3, 0.);
    }
}

#[test]
fn the_stylus_is_checked_against_stock_until_its_touch_zone() {
    let never = AtomicBool::new(false);
    let tool = probe().prepare().unwrap();
    let mut stock = Stock::new(spec()).unwrap();
    stock.enable_contact_queries(&never).unwrap();
    let (_, motions) = plan(operation(bore(), "probe"), &probe()).unwrap();
    let status = |m: &Motion| {
        stock_contact::check(&stock, &tool, m, Default::default(), &never)
            .unwrap()
            .status
    };
    // The bore is not machined: the protected move down into it hits the
    // stock, and so does the first measuring move (the ball is checked).
    let down = motions
        .iter()
        .find(|m| m.kind == MoveKind::ProbePosition)
        .unwrap();
    assert_eq!(status(down), Status::Violation);
    let measure = motions
        .iter()
        .find(|m| m.kind == MoveKind::ProbeMeasure)
        .unwrap();
    assert_ne!(status(measure), Status::Clear);
    // Outside the stock: the corner's moves are clear, including the touch
    // zone, where only the stem and body are checked.
    let (_, motions) = plan(operation(corner(), "probe"), &probe()).unwrap();
    for m in &motions {
        assert_eq!(status(m), Status::Clear, "{:?}", m.kind);
    }
}

#[test]
fn the_program_contract_holds_probing_to_its_cycle() {
    let (events, _) = plan(operation(corner(), "probe"), &probe()).unwrap();
    let cycle = events
        .iter()
        .position(|e| matches!(e, Event::Probe { .. }))
        .unwrap();
    // A probing move outside its cycle.
    let mut loose = events.clone();
    loose.remove(cycle);
    let e = program(vec![probe()], loose).validate(None).unwrap_err();
    assert!(e.contains("belong to a probing cycle"), "{e}");
    // A covered move that is not the cycle's next move.
    let mut moved = events.clone();
    if let Event::Motion { motion } = &mut moved[cycle + 2] {
        motion.curve = Curve::Line {
            start: motion.curve.start(),
            end: [6., -4., 5.],
        };
    }
    let e = program(vec![probe()], moved).validate(None).unwrap_err();
    assert!(e.contains("next move"), "{e}");
    // A different feed than the cycle declares.
    let mut fed = events.clone();
    if let Event::Motion { motion } = &mut fed[cycle + 2] {
        motion.feed_mm_min = Some(150.);
    }
    assert!(program(vec![probe()], fed).validate(None).is_err());
    // A spindle running under a probe.
    let mut spinning = events.clone();
    spinning.insert(
        cycle,
        Event::Spindle {
            rpm: 500.,
            direction: SpindleDirection::Cw,
        },
    );
    let e = program(vec![probe()], spinning).validate(None).unwrap_err();
    assert!(e.contains("stop the spindle"), "{e}");
}

#[test]
fn a_tool_length_set_after_probing_the_work_offset_is_refused() {
    let (probing, _) = plan(operation(corner(), "probe"), &probe()).unwrap();
    let length = json!({"kind": "tool_length", "contact_mm": [55, 15, 2], "clearance_mm": 5});
    let mut setting = operation(length, "mill");
    setting.spindle_rpm = 0.;
    let (tool_set, motions) = plan(setting, &mill()).unwrap();
    assert_eq!(
        kinds(&motions)[motions.len() - 4..],
        [
            MoveKind::ProbeMeasure,
            MoveKind::ProbeTouch,
            MoveKind::ProbeReturn,
            MoveKind::Rapid
        ]
    );
    // Lengths first, then the work offset: accepted.
    let start = |events: &[Event]| match &events[1] {
        Event::ToolChange {
            position_after_mm, ..
        } => *position_after_mm,
        _ => panic!(),
    };
    assert_eq!(start(&tool_set), start(&probing));
    let mut ids = 1000;
    let renumber = |events: &[Event], ids: &mut u64| -> Vec<Event> {
        events
            .iter()
            .map(|e| match e {
                Event::Motion { motion } => {
                    *ids += 1;
                    Event::Motion {
                        motion: Motion {
                            id: *ids,
                            ..motion.clone()
                        },
                    }
                }
                other => other.clone(),
            })
            .collect()
    };
    let back_up = |events: &[Event]| -> Vec<Event> {
        // Each operation ends over the clearance plane; return to the
        // shared start before the next tool change.
        let mut out = events.to_vec();
        let end = out
            .iter()
            .rev()
            .find_map(|e| match e {
                Event::Motion { motion } => Some(motion.curve.end()),
                _ => None,
            })
            .unwrap();
        out.push(Event::Motion {
            motion: Motion {
                id: 0,
                operation: "back".into(),
                tool: match &out[1] {
                    Event::ToolChange { tool, .. } => tool.clone(),
                    _ => panic!(),
                },
                kind: MoveKind::Rapid,
                curve: Curve::Line {
                    start: end,
                    end: [0., 0., 50.],
                },
                feed_mm_min: None,
            },
        });
        out
    };
    let ordered = [back_up(&tool_set), back_up(&probing)].concat();
    program(vec![probe(), mill()], renumber(&ordered, &mut ids))
        .validate(None)
        .unwrap();
    let reversed = [back_up(&probing), back_up(&tool_set)].concat();
    let e = program(vec![probe(), mill()], renumber(&reversed, &mut ids))
        .validate(None)
        .unwrap_err();
    assert!(e.contains("set tool lengths before probing"), "{e}");
}

#[test]
fn refusals_name_the_problem() {
    let refuse = |op: ProbeOperation, tool: &ToolAssembly| plan(op, tool).unwrap_err();
    // A probe never cuts.
    let holes: camlib::stream::StreamManifest =
        serde_json::from_str(include_str!("../fixtures/operations/holes.json")).unwrap();
    let mut drilling = serde_json::to_value(&holes.operations[0]).unwrap();
    drilling["tool"] = json!("probe");
    let pocket: Operation = serde_json::from_value(drilling).unwrap();
    let e = OperationPlan::prepare(
        pocket,
        &probe(),
        &spec(),
        [0., 0., 50.],
        1,
        &AtomicBool::new(false),
    )
    .err()
    .unwrap();
    assert!(e.contains("never cuts"), "{e}");
    // Stylus cycles need a probe; tool setting a cutting tool.
    assert!(refuse(operation(bore(), "mill"), &mill()).contains("not a probe"));
    // The search must be shorter than the approach.
    let mut op = operation(bore(), "probe");
    op.search_mm = 5.;
    let e = refuse(op, &probe());
    assert!(e.contains("not longer than search_mm"), "{e}");
    // The probing height below the retract height.
    let high = json!({"kind": "bore", "center_mm": [20, 15], "z_mm": 25, "diameter_mm": 12});
    assert!(refuse(operation(high, "probe"), &probe()).contains("below retract_z_mm"));
    // A probe's spindle stays stopped.
    let mut op = operation(bore(), "probe");
    op.spindle_rpm = 500.;
    op.spindle_direction = Some(SpindleDirection::Cw);
    assert!(refuse(op, &probe()).contains("stays stopped"));
}

#[test]
fn the_probe_tool_is_a_ball_on_a_thinner_stem() {
    let mut fat = probe();
    fat.cutter.shank_diameter_mm = BALL;
    assert!(
        fat.prepare()
            .err()
            .unwrap()
            .contains("thinner than the ball")
    );
    let tool = probe().prepare().unwrap();
    // The ball's lower half, then its radius up to the cutting length.
    let at = |z: f64| tool.radius_at(z).unwrap().radius_mm;
    assert!((at(BALL / 4.) - (BALL / 4. * (BALL - BALL / 4.)).sqrt()).abs() < 1e-12);
    assert_eq!(at(BALL * 0.75), BALL / 2.);
    assert_eq!(at(BALL + 1.), 1.5);
    assert_eq!(at(60.), 20.);
}
