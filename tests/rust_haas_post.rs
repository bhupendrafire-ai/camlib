//! Golden program authored by the Python Haas post, kept equal by its parity test.
use camlib::gcode::{Machine, decode::Decoder};
use serde_json::Value;
use std::{collections::BTreeSet, sync::atomic::AtomicBool};
const INPUT: &str = include_str!("../fixtures/posts/rust/haas-program.json");
const NC: &str = include_str!("../fixtures/posts/rust/haas-program.nc");
#[test]
fn python_haas_fixture_is_byte_identical() {
    let text = camlib::gcode::post_program(INPUT, &AtomicBool::new(false)).unwrap();
    let result: Value = serde_json::from_str(&text).unwrap();
    assert_eq!(result["program"], NC);
    assert_eq!(result["lines"], NC.lines().count());
    assert_eq!(result["motions"], 1);
}
fn decoder() -> Decoder {
    let input: Value = serde_json::from_str(INPUT).unwrap();
    let machine: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    Decoder::new(&machine, [5., 0., 3.], BTreeSet::new()).unwrap()
}
fn initialized() -> Decoder {
    let mut d = decoder();
    for line in NC.lines().take(17) {
        d.read_line(line, None).unwrap();
    }
    d.validate_initialized().unwrap();
    d
}
#[test]
fn independent_decoder_rejects_tampered_words_and_framing() {
    for (line, message) in [
        ("G0 X1. Y2. Z3.", "G0 dogleg"),
        ("G1 X1 Y2. Z3. F100.", "explicit decimal"),
        ("G4 P10", "milliseconds"),
        ("G1 X1.0001 Y2. Z3. F100.", "precision exceeded"),
        ("G52 X1 Y0 Z0", "explicit decimal"),
        ("G100 X0 Y0", "zero XYZ"),
        ("G187 P1", "milliseconds"),
        ("G187 P1.", "smoothing parameters"),
        ("M1", "optional stop"),
        ("M3 S100.5", "integral"),
        ("G86 X1. Y1. Z0. R2. F10.", "undeclared"),
        ("%", "explicit M30"),
        ("O1000", "repeated O"),
    ] {
        let error = initialized().read_line(line, None).unwrap_err();
        assert!(error.contains(message), "{line}: {error}");
    }
    for omitted in ["%", "O1000", "G50", "G69", "G100 X0 Y0 Z0", "G187"] {
        let mut d = decoder();
        let result = NC
            .lines()
            .take(17)
            .filter(|l| *l != omitted)
            .try_for_each(|l| d.read_line(l, None).map(|_| ()));
        assert!(
            result.is_err() || d.validate_initialized().is_err(),
            "{omitted}"
        );
    }
    let mut d = initialized();
    d.read_line("M30", None).unwrap();
    assert!(!d.ended);
    assert!(d.read_line("G21", None).unwrap_err().contains("framing"));
    d.read_line("%", None).unwrap();
    assert!(d.ended);
    assert!(d.read_line("%", None).is_err());
}
#[test]
fn machine_contract_refuses_unknown_controller_and_unproven_g86() {
    let mut input: Value = serde_json::from_str(INPUT).unwrap();
    input["machine"]["controller"]["name"] = "unknown".into();
    assert!(
        camlib::gcode::post_program(&input.to_string(), &AtomicBool::new(false))
            .unwrap_err()
            .contains("unknown variant")
    );
    let mut input: Value = serde_json::from_str(INPUT).unwrap();
    input["machine"]["dialect"]["commands"]
        .as_array_mut()
        .unwrap()
        .push(serde_json::json!({"effect":"bore_stop_cycle","code":"G86"}));
    assert!(
        camlib::gcode::post_program(&input.to_string(), &AtomicBool::new(false))
            .unwrap_err()
            .contains("command mapping")
    );
}

#[test]
fn haas_job_replays_and_applies_the_gate() {
    let source = include_str!("../fixtures/posts/rust/haas-drill.json");
    let result = camlib::post_job::run(source, &AtomicBool::new(false)).unwrap();
    let report: Value = serde_json::from_str(&result).unwrap();
    assert_eq!(report["released"], true);
    assert_eq!(
        report["program"],
        include_str!("../fixtures/posts/rust/haas-drill.nc")
    );
    assert_eq!(report["motions"], report["posting"]["checked_motions"]);
    let mut input: Value = serde_json::from_str(source).unwrap();
    input["accept"] = serde_json::json!([]);
    let result = camlib::post_job::run(&input.to_string(), &AtomicBool::new(false)).unwrap();
    let report: Value = serde_json::from_str(&result).unwrap();
    assert_eq!(report["released"], false);
    assert!(report["program"].is_null());
    assert_eq!(
        report["refused"]["statuses"],
        serde_json::json!(["unchecked_machine"])
    );
}

#[test]
fn controller_declaration_cannot_be_overridden_by_a_default_post_or_dialect() {
    let cancel = AtomicBool::new(false);
    let mut input: Value = serde_json::from_str(INPUT).unwrap();
    input["post"] = "default".into();
    assert!(
        camlib::gcode::post_program(&input.to_string(), &cancel)
            .unwrap_err()
            .contains("unknown field `post`")
    );
    let mut input: Value = serde_json::from_str(INPUT).unwrap();
    let default: Value =
        serde_json::from_str(include_str!("../fixtures/posts/rust/standalone-rest.json")).unwrap();
    input["machine"]["dialect"] = default["machine"]["dialect"].clone();
    // Match the declared peck distances so this isolates command identity,
    // rather than refusing an earlier Setting 22 disagreement.
    input["machine"]["controller"]["setting_22_mm"] = 0.254.into();
    input["machine"]["controller"]["setting_52_mm"] = 0.into();
    assert!(
        camlib::gcode::post_program(&input.to_string(), &cancel)
            .unwrap_err()
            .contains("reference command mapping")
    );
    // With the actual matching declaration, variant dispatch selects Haas,
    // including its own independently versioned decoder, never the default.
    let report: Value =
        serde_json::from_str(&camlib::gcode::post_program(INPUT, &cancel).unwrap()).unwrap();
    assert!(
        report["program"]
            .as_str()
            .unwrap()
            .starts_with("%\nO1000\n")
    );
    assert_eq!(
        report["definition"]["post_version"],
        camlib::gcode::post::HAAS_VERSION
    );
    assert_eq!(
        report["definition"]["decoder_version"],
        camlib::gcode::decode::HAAS_VERSION
    );
}

#[test]
fn missing_peck_return_distance_is_refused_without_nan() {
    use camlib::gcode::controller::Controller;
    use camlib::gcode::post::{DefaultPost, Hook};
    use camlib::motion::{DrillCycle, Event};
    let input: Value = serde_json::from_str(INPUT).unwrap();
    let mut machine: Machine = serde_json::from_value(input["machine"].clone()).unwrap();
    let Some(Controller::HaasNGC(h)) = &mut machine.controller else {
        panic!("fixture controller")
    };
    h.setting_22_mm = Some(0.254);
    h.setting_52_mm = Some(0.);
    machine.dialect.peck_reentry_mm = Some(0.254);
    machine.dialect.chip_break_lift_mm = Some(0.254);
    machine.dialect.commands.extend([
        camlib::gcode::Command {
            effect: "peck_cycle".into(),
            code: "G83".into(),
        },
        camlib::gcode::Command {
            effect: "chip_break_cycle".into(),
            code: "G73".into(),
        },
    ]);
    machine.validate().unwrap();
    for (cycle, name) in [
        (DrillCycle::Peck, "reentry_clearance_mm"),
        (DrillCycle::ChipBreak, "chip_break_lift_mm"),
    ] {
        let event = Event::Cycle {
            cycle,
            center_mm: [8., 8.],
            z_mm: 4.,
            r_mm: 11.,
            initial_z_mm: 12.,
            return_to_r: false,
            feed_mm_min: 150.,
            peck_mm: Some(1.5),
            dwell_s: None,
            reentry_mm: None,
            lift_mm: None,
            shift_mm: None,
            covers: 1,
        };
        let error = DefaultPost::new(machine.clone())
            .call(Hook::Event(&event), 8)
            .unwrap_err();
        assert_eq!(
            error,
            format!(
                "Haas {} requires the operation {name}: declare its return distance in mm",
                if cycle == DrillCycle::Peck {
                    "G83"
                } else {
                    "G73"
                }
            )
        );
        assert!(!error.contains("NaN"));
    }
}
