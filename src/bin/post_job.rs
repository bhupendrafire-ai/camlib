//! Post one setup to checked G-code: a JSON job on stdin (see
//! `camlib::post_job::Input`), the JSON report on stdout (the program only
//! when every finding is clear or accepted). Errors go to stderr with exit
//! status 1. Separate from `cam-sim-stream`, which streams a run's frames.
use std::io::{self, Read, Write};

const INPUT_LIMIT: u64 = 1 << 30;

fn run() -> Result<(), String> {
    if std::env::args_os().len() > 1 {
        return Err(
            "cam-post takes no arguments: the JSON job on stdin, the JSON report on stdout".into(),
        );
    }
    let mut input = String::new();
    io::stdin()
        .lock()
        .take(INPUT_LIMIT + 1)
        .read_to_string(&mut input)
        .map_err(|e| e.to_string())?;
    if input.len() as u64 > INPUT_LIMIT {
        return Err("post job input exceeds 1 GiB".into());
    }
    let report = camlib::post_job::run(&input, &std::sync::atomic::AtomicBool::new(false))?;
    let mut out = io::stdout().lock();
    out.write_all(report.as_bytes())
        .and_then(|()| out.write_all(b"\n"))
        .and_then(|()| out.flush())
        .map_err(|e| e.to_string())
}
fn main() {
    if let Err(error) = run() {
        eprintln!("cam-post: {error}");
        std::process::exit(1);
    }
}
