"""Measure #221 with the Release driver, sequentially on one host.

    python bench/model_bodies.py --output /path/to/results.json --repeat 3

Uses the existing lathe-bore workload/stream measurement. Geometry construction
is outside the timer; total includes transport and one-time model reduction.
The two bore operations measure per-move work after startup. Timing is reported,
never asserted in CI. Motions and final stock must match the built-in control.
"""

import argparse
from dataclasses import asdict, replace
import hashlib
import json
from pathlib import Path
import statistics

from run import _environment, _run_once
from workloads import BY_NAME, _lathe_bore
from support.tool_models import dense_bar_mesh


def measure(repeat):
    import camlib

    workload = BY_NAME["lathe-bore"]
    reference = None
    rows = []
    for name, facets, budget in [
        ("built-in", None, None),
        ("dense-exact", 512, None),
        ("dense-bounded", 512, 0.1),
        ("72-facets-exact", 72, None),
    ]:
        config = _lathe_bore()
        if facets:
            mesh = dense_bar_mesh(facets)
            model = camlib.TurningToolModel(
                body=mesh,
                placement=camlib.TurningModelPlacement(nose_center_mm=(0, 0, 0), x_axis=(1, 0, 0), z_axis=(0, 0, 1)),
                axial_resolution_mm=0.25,
                insert_check=camlib.TurningModelWithoutInsert(reason="synthetic bar inside the benchmark holder's box"),
                max_hull_error_mm=budget,
            )
            config["turning"]["tools"][1]["tool"]["body"]["model"] = model.to_dict()
        totals, seconds = [], []
        for _ in range(repeat):
            run = _run_once(replace(workload, build=lambda: config))
            physical = json.dumps(
                {"events": [asdict(e) for e in run["program"].events], "stock": run["state"].stock()}, sort_keys=True
            ).encode()
            signature = hashlib.sha256(physical).hexdigest()
            if reference is None:
                reference = signature
            if signature != reference:
                raise AssertionError(f"{name}: motions or stock differ from the built-in control")
            moves = sum(run["ops"][op]["moves"] for op in ("bore_rough", "bore_finish"))
            totals.append(run["total"])
            seconds.append(sum(run["ops"][op]["engine_seconds"] for op in ("bore_rough", "bore_finish")))
        rows.append(
            dict(
                name=name,
                triangles=len(mesh.triangles) if facets else 0,
                max_hull_error_mm=budget,
                repeats=repeat,
                total_seconds=totals,
                bore_seconds=seconds,
                bore_moves=moves,
                median_bore_ms_per_move=1000 * statistics.median(seconds) / moves,
                median_total_seconds=statistics.median(totals),
                physical_sha256=signature,
            )
        )
    for row in rows:
        row["bore_cost_multiple_of_builtin"] = row["median_bore_ms_per_move"] / rows[0]["median_bore_ms_per_move"]
    return dict(environment=_environment(), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeat", type=int, default=3)
    args = parser.parse_args()
    if args.repeat < 1:
        parser.error("--repeat must be positive")
    result = measure(args.repeat)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
