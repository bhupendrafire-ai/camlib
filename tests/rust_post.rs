//! The Rust default post (issue #120) without Python: the program it posts for
//! a fixture job equals, byte for byte, the program the Python default post
//! wrote for it (fixtures/posts/rust/*.nc, produced by `Job.post_nc`;
//! python/tests/test_rust_post.py keeps the Python side equal to it), and
//! the same gate withholds it.
use camlib::gcode::Machine;
use camlib::gcode::decode::{Context, Decoder};
use camlib::motion::MoveKind;
use serde_json::Value;
use std::sync::atomic::AtomicBool;

const INPUT: &str = include_str!("../fixtures/posts/rust/standalone-rest.json");
const PROGRAM: &str = include_str!("../fixtures/posts/rust/standalone-rest.nc");

fn post(input: &Value) -> Result<Value, String> {
    let report = camlib::post_job::run(&input.to_string(), &AtomicBool::new(false))?;
    Ok(serde_json::from_str(&report).unwrap())
}
fn fixture() -> Value {
    serde_json::from_str(INPUT).unwrap()
}

#[test]
fn posts_the_python_program_byte_for_byte() {
    let report = post(&fixture()).unwrap();
    assert_eq!(report["released"], true);
    assert_eq!(report["program"].as_str().unwrap(), PROGRAM);
    assert_eq!(report["lines"], PROGRAM.lines().count());
    assert_eq!(report["accepted"], serde_json::json!(["unchecked_machine"]));
    assert_eq!(
        report["unchecked"],
        serde_json::json!(["scene", "material", "machine"])
    );
    assert_eq!(report["machine"]["status"], "unchecked_machine");
    // Every planned motion was decoded back and replayed.
    assert_eq!(report["motions"], report["posting"]["checked_motions"]);
}

#[test]
fn withholds_the_program_unless_findings_are_accepted() {
    let mut input = fixture();
    input["accept"] = serde_json::json!([]);
    let report = post(&input).unwrap();
    assert_eq!(report["released"], false);
    assert!(report["program"].is_null());
    assert_eq!(
        report["refused"]["statuses"],
        serde_json::json!(["unchecked_machine"])
    );
    input["accept"] = serde_json::json!(["everything"]);
    assert!(post(&input).unwrap_err().contains("accept takes"));
}

#[test]
fn the_machine_json_states_every_field_and_is_checked() {
    for (change, message) in [
        ("decimals", "missing field `decimals`"),
        ("rapid_mm_min", "missing field `rapid_mm_min`"),
    ] {
        let mut input = fixture();
        input["machine"].as_object_mut().unwrap().remove(change);
        assert!(post(&input).unwrap_err().contains(message), "{change}");
    }
    let mut input = fixture();
    input["machine"]["decimals"] = 1.into();
    assert!(
        post(&input)
            .unwrap_err()
            .contains("invalid machine output units/precision")
    );
    let mut input = fixture();
    input["machine"]["dialect"]["axes"] = serde_json::json!(["X", "Y", "X"]);
    assert!(
        post(&input)
            .unwrap_err()
            .contains("distinct uppercase letters")
    );
    let mut input = fixture();
    input["manifest"]["turning"] = serde_json::json!({});
    assert!(post(&input).is_err());
}

#[test]
fn the_decoder_reads_only_declared_words() {
    let machine = Machine::from_json(fixture()["machine"].clone()).unwrap();
    let mut decoder = Decoder::new(&machine, [0., 0., 8.], Default::default()).unwrap();
    let lines: Vec<String> = [
        "G21", "G90", "G91.1", "G17", "G94", "G97", "G40", "G49", "G80", "G92.1", "G61.1", "M5",
        "M9",
    ]
    .map(String::from)
    .into();
    assert!(
        decoder
            .read(&lines, None)
            .unwrap()
            .iter()
            .all(|e| !matches!(e, camlib::motion::Event::Motion { .. }))
    );
    decoder.validate_initialized().unwrap();
    let context = Context {
        id: 1,
        operation: "op".into(),
        kind: MoveKind::Cut,
    };
    for (line, message) in [
        ("G5 X1", "undeclared controller command"),
        ("G1 X1 Q2 F100", "unused/unsupported NC words"),
        ("G1 X1 X2 F100", "duplicate NC word"),
        (
            "G0 X1",
            "NC motion with unknown attribution/tool/length/offset",
        ),
    ] {
        let error = decoder
            .clone()
            .read(&[line.to_string()], Some(&context))
            .unwrap_err();
        assert!(error.contains(message), "{line}: {error}");
    }
}

#[test]
fn cam_post_command_reports_on_stdout() {
    use std::io::Write;
    use std::process::{Command, Stdio};
    let mut child = Command::new(env!("CARGO_BIN_EXE_cam-post"))
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .unwrap();
    child
        .stdin
        .take()
        .unwrap()
        .write_all(INPUT.as_bytes())
        .unwrap();
    let out = child.wait_with_output().unwrap();
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    let report: Value = serde_json::from_slice(&out.stdout).unwrap();
    assert_eq!(report["program"].as_str().unwrap(), PROGRAM);
    let failed = Command::new(env!("CARGO_BIN_EXE_cam-post"))
        .arg("--help")
        .output()
        .unwrap();
    assert_eq!(failed.status.code(), Some(1));
}
