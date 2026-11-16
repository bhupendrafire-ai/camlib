//! Release-only measurement helper for bench/model_bodies.py. Uses public APIs.
use camlib::turning::ops::{TurningJob, plan};
use sha2::{Digest, Sha256};
use std::{sync::atomic::AtomicBool, time::Instant};

fn physical(p: &camlib::turning::ops::TurnPlan) -> Vec<u8> {
    serde_json::to_vec(&serde_json::json!({
        "events": p.events, "cells": p.stock.cells(), "rings": p.stock.rings()
    }))
    .unwrap()
}

fn main() {
    let args: Vec<_> = std::env::args().collect();
    let job: TurningJob = serde_json::from_slice(&std::fs::read(&args[1]).unwrap()).unwrap();
    let repeat: usize = args[2].parse().unwrap();
    assert!(repeat > 0);
    let cancel = AtomicBool::new(false);
    // Input decoding is outside the timer. Cold native cost includes reduction.
    let started = Instant::now();
    let first = plan(&job, &cancel).unwrap();
    let cold = started.elapsed().as_secs_f64();
    let reference = physical(&first);
    let moves = first
        .events
        .iter()
        .filter(|event| matches!(event, camlib::motion::Event::Motion { .. }))
        .count();
    let mut seconds = Vec::new();
    for _ in 0..repeat {
        let started = Instant::now();
        let next = plan(&job, &cancel).unwrap();
        seconds.push(started.elapsed().as_secs_f64());
        assert_eq!(physical(&next), reference, "timed native plan changed");
    }
    let pieces = job.tools[1]
        .tool
        .body
        .as_ref()
        .and_then(|b| b.model.as_ref())
        .map(|m| m.pieces("bar", true).unwrap())
        .unwrap_or_default();
    println!(
        "{}",
        serde_json::json!({
            "cold_native_seconds": cold, "warm_plan_seconds": seconds, "moves": moves,
            "prisms": pieces.len(), "vertices": pieces.iter().map(Vec::len).sum::<usize>(),
            "physical_sha256": format!("{:x}", Sha256::digest(&reference))
        })
    );
}
