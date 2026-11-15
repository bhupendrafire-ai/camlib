//! Bounded first-improvement 2-opt over hole centers (#23/#29).
//!
//! This changes order only. Travel remains independently checked by the operation.
//! The last center is fixed, so downstream operations keep their entry position.
//! 2-opt reference: Croes (1958), https://doi.org/10.1287/opre.6.6.791.
use crate::{
    Point,
    dynamics::{Settings, capped_segment},
    motion::{Curve, Motion, MoveKind, Vec3},
    stock::CancelSignal,
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HoleOrdering {
    pub settings: Settings,
    /// Model evaluations and route-sum work, including before/after scoring.
    pub max_work: usize,
}

#[derive(Debug, Serialize)]
pub struct OrderingReport {
    pub order: Vec<usize>,
    pub given_seconds: f64,
    pub ordered_seconds: f64,
    pub work: usize,
    pub max_work: usize,
    pub work_limit_reached: bool,
    pub scope: &'static str,
}

pub(crate) fn holes(
    centers: &[Point],
    initial: Vec3,
    clearance: f64,
    tolerance: f64,
    options: &HoleOrdering,
    cancel: &impl CancelSignal,
) -> Result<OrderingReport, String> {
    options.settings.validate()?;
    let n = centers.len();
    let minimum = n.checked_mul(4).ok_or("hole ordering work overflow")?;
    if options.max_work < minimum || options.max_work == 0 {
        return Err(format!(
            "hole ordering needs max_work >= {minimum} to score every center; increase max_work"
        ));
    }
    let check = || {
        if cancel.is_cancelled() {
            Err("hole ordering cancelled; no result committed")
        } else {
            Ok(())
        }
    };
    let seconds = |a: Point, b: Point| -> Result<f64, String> {
        check()?;
        if a == b {
            return Ok(0.);
        }
        Ok(capped_segment(
            &Motion {
                id: 0,
                operation: "hole-order".into(),
                tool: "hole-order".into(),
                kind: MoveKind::Rapid,
                curve: Curve::Line {
                    start: [a[0], a[1], clearance],
                    end: [b[0], b[1], clearance],
                },
                feed_mm_min: None,
            },
            0,
            tolerance,
            &options.settings,
        )?
        .seconds)
    };
    let start = [initial[0], initial[1]];
    let sum = |values: &[f64]| -> Result<f64, String> {
        let mut total = 0.;
        for chunk in values.chunks(256) {
            check()?;
            for value in chunk {
                total += value;
            }
        }
        Ok(total)
    };
    let mut order: Vec<usize> = (0..n).collect();
    let mut costs = Vec::with_capacity(n);
    for (i, p) in centers.iter().enumerate() {
        costs.push(seconds(if i == 0 { start } else { centers[i - 1] }, *p)?);
    }
    let given = sum(&costs)?;
    let mut total = given;
    let mut work = 2 * n;
    // Reserve the final complete score; a partial route is never returned.
    let search_limit = options.max_work - 2 * n;
    let mut limited = false;
    'search: loop {
        let mut improved = false;
        for i in 0..n.saturating_sub(1) {
            for j in i + 1..n.saturating_sub(1) {
                check()?;
                if search_limit.saturating_sub(work) < 2 {
                    limited = true;
                    break 'search;
                }
                work += 2;
                let left = if i == 0 { start } else { centers[order[i - 1]] };
                let a = seconds(left, centers[order[j]])?;
                let b = seconds(centers[order[i]], centers[order[j + 1]])?;
                let old = (costs[i], costs[j + 1]);
                if a + b >= old.0 + old.1 {
                    continue;
                }
                if search_limit.saturating_sub(work) < n {
                    limited = true;
                    break 'search;
                }
                work += n;
                order[i..=j].reverse();
                costs[i + 1..=j].reverse();
                costs[i] = a;
                costs[j + 1] = b;
                let candidate = sum(&costs)?;
                if candidate < total {
                    total = candidate;
                    improved = true;
                    break;
                }
                order[i..=j].reverse();
                costs[i + 1..=j].reverse();
                costs[i] = old.0;
                costs[j + 1] = old.1;
            }
            if improved {
                break;
            }
        }
        if !improved {
            break;
        }
    }
    let mut final_cost = 0.;
    for (i, &index) in order.iter().enumerate() {
        final_cost += seconds(
            if i == 0 { start } else { centers[order[i - 1]] },
            centers[index],
        )?;
    }
    work += 2 * n;
    if final_cost >= given {
        for (i, v) in order.iter_mut().enumerate() {
            if i % 256 == 0 {
                check()?;
            }
            *v = i;
        }
        final_cost = given;
    }
    check()?;
    Ok(OrderingReport {
        order,
        given_seconds: given,
        ordered_seconds: final_cost,
        work,
        max_work: options.max_work,
        work_limit_reached: limited,
        scope: "nominal exact-stop XY rapid time under declared motion settings; fixed final hole; not a global optimum or clearance proof",
    })
}
