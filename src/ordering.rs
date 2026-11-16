//! Bounded first-improvement 2-opt over hole centers (#23/#29).
//!
//! This changes order only. Travel remains independently checked by the operation.
//! The last center is fixed, so downstream operations keep their entry position.
//! 2-opt reference: Croes (1958), https://doi.org/10.1287/opre.6.6.791.
//! Optional nearest-neighbour seed: Rosenkrantz et al. (1977),
//! https://doi.org/10.1137/0206041. The verified baseline is retained first.
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
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub nearest_neighbor: bool,
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
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nearest_neighbor: Option<NearestNeighborReport>,
}

#[derive(Debug, Serialize)]
pub struct NearestNeighborReport {
    pub status: &'static str,
    pub baseline_seconds: f64,
    pub seed_seconds: Option<f64>,
    pub optimized_seconds: Option<f64>,
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
            Err("hole ordering cancelled; no result committed".to_string())
        } else {
            Ok(())
        }
    };
    let seconds = |a: Point, b: Point| -> Result<f64, String> {
        check()?;
        if a == b {
            return Ok(0.);
        }
        let value = capped_segment(
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
        .seconds;
        if !value.is_finite() || value < 0. {
            return Err(
                "hole ordering needs finite nominal travel times; check machine settings".into(),
            );
        }
        Ok(value)
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
    let improve = |order: &mut [usize],
                   costs: &mut [f64],
                   total: &mut f64,
                   work: &mut usize,
                   search_limit: usize|
     -> Result<bool, String> {
        let mut limited = false;
        'search: loop {
            let mut improved = false;
            for i in 0..n.saturating_sub(1) {
                for j in i + 1..n.saturating_sub(1) {
                    check()?;
                    if search_limit.saturating_sub(*work) < 2 {
                        limited = true;
                        break 'search;
                    }
                    *work += 2;
                    let left = if i == 0 { start } else { centers[order[i - 1]] };
                    let a = seconds(left, centers[order[j]])?;
                    let b = seconds(centers[order[i]], centers[order[j + 1]])?;
                    let old = (costs[i], costs[j + 1]);
                    if a + b >= old.0 + old.1 {
                        continue;
                    }
                    if search_limit.saturating_sub(*work) < n {
                        limited = true;
                        break 'search;
                    }
                    *work += n;
                    order[i..=j].reverse();
                    costs[i + 1..=j].reverse();
                    costs[i] = a;
                    costs[j + 1] = b;
                    let candidate = sum(costs)?;
                    if candidate < *total {
                        *total = candidate;
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
        Ok(limited)
    };
    let mut limited = improve(&mut order, &mut costs, &mut total, &mut work, search_limit)?;
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
    let nearest_neighbor = if options.nearest_neighbor {
        let mut report = NearestNeighborReport {
            status: "not_applicable",
            baseline_seconds: final_cost,
            seed_seconds: None,
            optimized_seconds: None,
        };
        if n >= 3 {
            // n-1 free holes: triangular neighbour comparisons, one final edge,
            // a complete seed sum and a reserved complete final score (2*n).
            let required = n
                .checked_mul(n - 1)
                .map(|v| v / 2)
                .and_then(|v| n.checked_mul(3).and_then(|extra| v.checked_add(extra)))
                .and_then(|v| v.checked_add(1));
            if required.is_none_or(|required| options.max_work - work < required) {
                report.status = "insufficient_budget";
                limited = true;
            } else {
                let mut seed = Vec::with_capacity(n);
                let mut seed_costs = Vec::with_capacity(n);
                let mut used = vec![false; n - 1];
                let mut previous = start;
                for _ in 0..n - 1 {
                    check()?;
                    let mut best: Option<(usize, f64)> = None;
                    for index in 0..n - 1 {
                        if used[index] {
                            continue;
                        }
                        let cost = seconds(previous, centers[index])?;
                        work += 1;
                        if best.is_none_or(|(_, old)| cost < old) {
                            best = Some((index, cost));
                        }
                    }
                    let (index, cost) =
                        best.ok_or("nearest-neighbour seed lost a hole occurrence")?;
                    seed.push(index);
                    seed_costs.push(cost);
                    used[index] = true;
                    previous = centers[index];
                }
                seed.push(n - 1);
                seed_costs.push(seconds(previous, centers[n - 1])?);
                work += 1 + n;
                let mut seed_total = sum(&seed_costs)?;
                report.seed_seconds = Some(seed_total);
                limited |= improve(
                    &mut seed,
                    &mut seed_costs,
                    &mut seed_total,
                    &mut work,
                    options.max_work - 2 * n,
                )?;
                let mut candidate = 0.;
                for (i, &index) in seed.iter().enumerate() {
                    candidate += seconds(
                        if i == 0 { start } else { centers[seed[i - 1]] },
                        centers[index],
                    )?;
                }
                work += 2 * n;
                report.optimized_seconds = Some(candidate);
                report.status = "not_better";
                if candidate < final_cost {
                    order = seed;
                    final_cost = candidate;
                    report.status = "selected";
                }
            }
        }
        Some(report)
    } else {
        None
    };
    check()?;
    Ok(OrderingReport {
        order,
        given_seconds: given,
        ordered_seconds: final_cost,
        work,
        max_work: options.max_work,
        work_limit_reached: limited,
        nearest_neighbor,
        scope: "nominal exact-stop XY rapid time under declared motion settings; fixed final hole; not a global optimum or clearance proof",
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        dynamics::{JunctionMode, Policy},
        motion::WorkFrame,
    };
    use std::sync::atomic::{AtomicUsize, Ordering};

    const HARD: [Point; 9] = [
        [46., 62.],
        [38., 86.],
        [38., 30.],
        [62., 70.],
        [14., 54.],
        [94., 46.],
        [70., 38.],
        [22., 6.],
        [62., 38.],
    ];

    fn options(max_work: usize, nearest_neighbor: bool) -> HoleOrdering {
        HoleOrdering {
            settings: Settings {
                policy: Policy {
                    tangential_accel_mm_s2: 100.,
                    rapid_feed_mm_min: 6000.,
                    junction_mode: JunctionMode::ExactStop,
                    feed_reserve_fraction: 0.,
                },
                max_feed_mm_min: 6000.,
                max_normal_accel_mm_s2: None,
                max_axis_feed_mm_min: None,
                work_to_machine: WorkFrame::identity(),
                feed_quantum_mm_min: 0.001,
                adjust_feeds: false,
            },
            max_work,
            nearest_neighbor,
        }
    }
    struct Count {
        calls: AtomicUsize,
        stop: usize,
    }
    impl CancelSignal for Count {
        fn is_cancelled(&self) -> bool {
            self.calls.fetch_add(1, Ordering::Relaxed) >= self.stop
        }
    }
    fn run(points: &[Point], budget: usize, nearest: bool) -> OrderingReport {
        holes(
            points,
            [0., 0., 12.],
            12.,
            0.001,
            &options(budget, nearest),
            &Count {
                calls: AtomicUsize::new(0),
                stop: usize::MAX,
            },
        )
        .unwrap()
    }
    #[test]
    fn nearest_neighbour_retains_baseline_at_every_budget_and_every_occurrence() {
        for budget in (36..950).step_by(17).chain([100_000]) {
            let old = run(&HARD, budget, false);
            let new = run(&HARD, budget, true);
            assert!(old.nearest_neighbor.is_none());
            assert!(new.ordered_seconds <= old.ordered_seconds && new.work <= budget);
            assert_eq!(
                new.nearest_neighbor.as_ref().unwrap().baseline_seconds,
                old.ordered_seconds
            );
            assert_eq!(new.order.last(), Some(&8));
            let mut order = new.order.clone();
            order.sort_unstable();
            assert_eq!(order, (0..9).collect::<Vec<_>>());
            if budget == 36 {
                assert_eq!(new.order, old.order);
                assert_eq!(
                    new.nearest_neighbor.as_ref().unwrap().status,
                    "insufficient_budget"
                );
            }
            if budget == 100_000 {
                assert_eq!(new.nearest_neighbor.as_ref().unwrap().status, "selected");
                assert!(new.ordered_seconds < 0.95 * old.ordered_seconds);
            }
        }
        let points = [[10., 12.], [10., 12.], [28., 12.]];
        let first = run(&points, 1000, true);
        let second = run(&points, 1000, true);
        assert_eq!(first.order, vec![0, 1, 2]);
        assert_eq!(first.order, second.order);
        assert_eq!(first.nearest_neighbor.unwrap().status, "not_better");
        assert_eq!(
            run(&points[..2], 1000, true)
                .nearest_neighbor
                .unwrap()
                .status,
            "not_applicable"
        );
    }
    #[test]
    fn cancellation_inside_extra_seed_returns_no_report() {
        let counter = Count {
            calls: AtomicUsize::new(0),
            stop: usize::MAX,
        };
        holes(
            &HARD,
            [0., 0., 12.],
            12.,
            0.001,
            &options(100_000, false),
            &counter,
        )
        .unwrap();
        let cancel = Count {
            calls: AtomicUsize::new(0),
            stop: counter.calls.load(Ordering::Relaxed) + 2,
        };
        assert!(
            holes(
                &HARD,
                [0., 0., 12.],
                12.,
                0.001,
                &options(100_000, true),
                &cancel
            )
            .unwrap_err()
            .contains("cancelled")
        );
    }
}
