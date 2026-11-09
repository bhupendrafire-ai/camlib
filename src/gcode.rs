//! The default G-code post in Rust: a declared machine and controller
//! dialect as data (the JSON form of `camlib.post.Machine`), the default
//! formatter (a port of the Python `GCodePost`), an independent decoder of
//! the written words (a port of `GCodeDecoder`/`NativeGCodeDecoder`), and
//! the posting engine that has the native intent checker compare every
//! decoded block with its planned source event.
//!
//! The Python posts stay the user-editable posts (docs/DECISIONS.md). This
//! module serves callers without a Python runtime, such as a WebAssembly
//! build. Its output is byte-identical to the Python default post's for the
//! same program and machine (python/tests/test_rust_post.py).
pub mod decode;
pub mod engine;
pub mod machine;
pub mod post;

pub use engine::{Engine, LoweringOptions};
pub use machine::{Command, Dialect, Machine, ToolSetting};

use crate::motion::Program;
use crate::stock::CancelSignal;
use serde::Deserialize;

/// The JSON input of [`post_program`].
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ProgramInput {
    /// A `camlib.motion.Program` with its events.
    pub program: Program,
    /// `camlib.post.Machine.to_dict()`.
    pub machine: serde_json::Value,
    #[serde(default)]
    pub lowering: Option<LoweringOptions>,
    #[serde(default)]
    pub run_id: Option<String>,
}

/// Post an explicit program's events (the Rust counterpart of Python's
/// `camlib.post.post_events`): every block is decoded and compared with its
/// source event, but nothing is replayed on stock and no export gate is
/// applied, so the result is not a released program (use
/// `crate::post_job` for that). Returns JSON: the program text, its line
/// and checked-motion counts, the arc output counts and the checked chunks.
pub fn post_program(input: &str, cancel: &impl CancelSignal) -> Result<String, String> {
    let parsed: ProgramInput =
        serde_json::from_str(input).map_err(|e| format!("post program input: {e}"))?;
    let machine = Machine::from_json(parsed.machine)?;
    let mut header = parsed.program;
    let events = std::mem::take(&mut header.events);
    let run_id = parsed.run_id.unwrap_or_else(|| {
        use sha2::{Digest, Sha256};
        format!(
            "post-{}",
            &format!("{:x}", Sha256::digest(input.as_bytes()))[..32]
        )
    });
    let (mut engine, first) = Engine::begin(&header, machine, parsed.lowering, run_id, cancel)?;
    let mut chunks = vec![first];
    engine.consume_events(events, cancel, |o| {
        chunks.push(o);
        Ok(())
    })?;
    chunks.push(engine.finish(cancel)?);
    let mut text = String::new();
    let mut lines = 0;
    let mut motions = 0;
    for chunk in &chunks {
        for line in &chunk.chunk.lines {
            text.push_str(&line.text);
            text.push('\n');
            lines += 1;
        }
        motions += chunk
            .chunk
            .checked
            .iter()
            .filter(|c| c.motion.is_some())
            .count();
    }
    serde_json::to_string(&serde_json::json!({
        "program": text,
        "lines": lines,
        "motions": motions,
        "arc_output": engine.arc_output,
        "definition": engine.definition,
        "chunks": chunks,
    }))
    .map_err(|e| e.to_string())
}
