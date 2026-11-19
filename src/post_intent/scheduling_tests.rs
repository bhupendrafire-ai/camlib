use super::*;
use crate::{
    dynamics::{JunctionMode, Policy},
    motion::{Curve, Motion, MoveKind, Plane},
};
use std::{cell::Cell, f64::consts::PI};
fn fixture(lower: bool, normal: Option<f64>) -> IntentChecker {
    let (old, init, header) = super::normal_limit_tests::fixture(normal, lower);
    let mut settings = old.settings.clone();
    settings.capabilities.max_feed_mm_min = 300.;
    settings.motion_scheduling = Some(crate::dynamics::Settings {
        policy: Policy {
            tangential_accel_mm_s2: 100.,
            rapid_feed_mm_min: 6000.,
            junction_mode: JunctionMode::ExactStop,
            feed_reserve_fraction: 0.01,
            per_block_s: 0.,
        },
        max_feed_mm_min: 300.,
        max_normal_accel_mm_s2: normal,
        max_axis_feed_mm_min: None,
        work_to_machine: WorkFrame::identity(),
        feed_quantum_mm_min: 0.001,
        adjust_feeds: true,
    });
    let (mut checker, _) = IntentChecker::begin(&header, settings, &["BEGIN".into()]).unwrap();
    super::normal_limit_tests::initialize(&mut checker, &init, lower);
    checker
}
fn line(id: u64, a: f64, b: f64) -> Event {
    Event::Motion {
        motion: Motion {
            id,
            operation: "cut".into(),
            tool: "t1".into(),
            kind: MoveKind::Cut,
            curve: Curve::Line {
                start: [a, 0., 5.],
                end: [b, 0., 5.],
            },
            feed_mm_min: Some(600.),
        },
    }
}
fn summary(checker: &IntentChecker) -> serde_json::Value {
    serde_json::to_value(&checker.scheduling.as_ref().unwrap().summary).unwrap()
}
#[test]
fn stream_policy_preview_bad_output_and_late_cancel_are_atomic() {
    let mut c = fixture(false, None);
    let event = line(1, 2., 12.);
    let before = summary(&c);
    let first_line = c.line;
    let prepared = c
        .preview_source(std::slice::from_ref(&event), &|| false)
        .unwrap();
    assert_eq!(summary(&c), before);
    assert!(
        c.consume(
            std::slice::from_ref(&event),
            &[vec![event.clone()]],
            &[vec!["BAD".into()]],
            &|| false
        )
        .is_err()
    );
    assert_eq!(summary(&c), before);
    assert_eq!(c.line, first_line);
    assert_eq!(c.source.summary.events, 3);
    let out = c
        .consume(
            std::slice::from_ref(&event),
            &[prepared],
            &[vec!["GOOD".into()]],
            &|| false,
        )
        .unwrap();
    let hash = format!("{:x}", Sha256::digest(serde_json::to_vec(&event).unwrap()));
    assert_eq!(out.mappings[0].source_event_sha256, hash);
    let report = out.motion_scheduling.unwrap();
    let row = &report.records[0];
    assert_eq!(row.authored_feed_mm_min, Some(600.));
    assert_eq!(row.configured_feed_mm_min, Some(600.));
    assert!(row.scheduled_feed_mm_min.unwrap() < 300.);
    assert_eq!(row.scheduled_feed_mm_min, row.decoded_feed_mm_min);
    assert_ne!(row.scheduled_source_event_sha256, hash);
    assert_eq!(report.summary.completed_source_events, 4);
    assert_eq!(report.summary.limited_source_motions, 1);
    let speed = row.scheduled_feed_mm_min.unwrap() / 60.;
    assert!(
        (report.summary.source_exact_stop_seconds - (10. / speed + speed / 100.)).abs() < 1e-12
    );
    let next = line(2, 12., 22.);
    let prepared = c
        .preview_source(std::slice::from_ref(&next), &|| false)
        .unwrap();
    let before = summary(&c);
    let count = Cell::new(0);
    assert!(
        c.consume(
            std::slice::from_ref(&next),
            std::slice::from_ref(&prepared),
            &[vec!["MOVE".into()]],
            &|| {
                count.set(count.get() + 1);
                count.get() >= 3
            }
        )
        .is_err()
    );
    assert_eq!(summary(&c), before);
    assert_eq!(c.source.summary.events, 4);
    c.consume(&[next], &[prepared], &[vec!["MOVE".into()]], &|| false)
        .unwrap();
    assert_eq!(c.scheduling.unwrap().summary.completed_source_motions, 2);
}
#[test]
fn stream_policy_lowered_original_is_limited_once_and_committed_only_on_completion() {
    let mut c = fixture(true, Some(10.));
    let event = Event::Motion {
        motion: Motion {
            id: 1,
            operation: "curve".into(),
            tool: "t1".into(),
            kind: MoveKind::Cut,
            curve: Curve::Arc {
                start: [2., 0., 5.],
                end: [-2., 0., 5.],
                center: [0., 0., 5.],
                plane: Plane::Xy,
                sweep_rad: PI,
            },
            feed_mm_min: Some(600.),
        },
    };
    let hash = format!("{:x}", Sha256::digest(serde_json::to_vec(&event).unwrap()));
    c.start(event, &|| false).unwrap();
    let before = summary(&c);
    let preview = c.preview(2, &|| false).unwrap();
    let mut bad = preview[0].event.clone();
    if let Event::Motion { motion } = &mut bad {
        motion.feed_mm_min = Some(299.);
    }
    assert!(
        c.check_lowered(
            &[preview[0].source_fraction_interval],
            &[vec![bad]],
            &[vec!["BAD".into()]],
            &|| false
        )
        .is_err()
    );
    assert_eq!(summary(&c), before);
    assert_eq!(c.pending.front().unwrap().next, 0.);
    let mut feed = None;
    let mut rows = 0;
    while c.has_pending() {
        let p = c.preview(2, &|| false).unwrap();
        let ranges: Vec<_> = p.iter().map(|p| p.source_fraction_interval).collect();
        let actual: Vec<_> = p.into_iter().map(|p| vec![p.event]).collect();
        let out = c
            .check_lowered(
                &ranges,
                &actual,
                &vec![vec!["MOVE".into()]; ranges.len()],
                &|| false,
            )
            .unwrap();
        let report = out.motion_scheduling.unwrap();
        for (m, r) in out.mappings.iter().zip(&report.records) {
            assert_eq!(m.source_event_sha256, hash);
            assert_eq!(r.authored_feed_mm_min, Some(600.));
            let f = r.scheduled_feed_mm_min.unwrap();
            assert!((f / 60.).powi(2) / 2. < 10.);
            assert!(f < 300.);
            if let Some(before) = feed {
                assert_eq!(f, before);
            }
            feed = Some(f);
            rows += 1;
            assert_eq!(
                r.completed_source_exact_stop_seconds.is_some(),
                r.source_fraction_interval[1] == 1.
            );
        }
        assert_eq!(
            report.summary.completed_source_motions,
            usize::from(!c.has_pending())
        );
    }
    let summary = c.scheduling.unwrap().summary;
    assert!(rows > 2);
    assert_eq!(summary.limited_source_motions, 1);
    assert_eq!(summary.decoded_motions, rows);
    assert!(summary.decoded_exact_stop_seconds > summary.source_exact_stop_seconds);
}
#[test]
fn stream_policy_rejects_mismatched_machine_limits_and_preview_lifecycle() {
    let (old, _, header) = super::normal_limit_tests::fixture(None, false);
    let mut c = fixture(false, None);
    c.settings
        .motion_scheduling
        .as_mut()
        .unwrap()
        .max_feed_mm_min = 500.;
    assert!(IntentChecker::begin(&header, c.settings, &["BEGIN".into()]).is_err());
    let mut c = fixture(false, None);
    c.finish(&["END".into()], &|| false).unwrap();
    assert!(c.preview_source(&[], &|| false).is_err());
    assert!(fixture(true, None).preview_source(&[], &|| false).is_err());
    assert!(old.scheduling.is_none());
}
