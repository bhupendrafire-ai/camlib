"""Measure #221's native planner using public APIs in shipping Release.

    python bench/model_bodies.py --output /path/to/results.json --repeat 3

Input decoding is outside the native timer. Cold native planning includes model
reduction; warm complete plans include all moves and geometry/collision checks.
The ordinary 17-move stream case is too short to separate transport/startup from
per-move work, so the same bore uses a declared 0.015 mm roughing pass depth.
Timing is reported, never asserted in CI. Motions and final stock must match.
"""

import argparse
import json
from pathlib import Path
import statistics
import subprocess
import tempfile

from run import _environment
from workloads import ROOT, _lathe_bore
from support.tool_models import dense_bar_mesh


def measure(repeat):
    import camlib

    # Cargo owns target-dir selection; its message supplies the actual executable.
    built = subprocess.check_output(
        [
            "cargo",
            "build",
            "--locked",
            "--release",
            "--all-features",
            "--example",
            "model-body-cost",
            "--message-format=json",
        ],
        cwd=ROOT,
        text=True,
    )
    (executable,) = [row["executable"] for line in built.splitlines() if (row := json.loads(line)).get("executable")]
    reference = None
    rows = []
    for name, facets, budget in [
        ("built-in", None, None),
        ("dense-exact", 512, None),
        ("dense-bounded", 512, 0.1),
        ("72-facets-exact", 72, None),
    ]:
        config = _lathe_bore()
        job = config["turning"]
        job["operations"][1]["depth_of_cut_mm"] = 0.015
        job.update(
            id="model-bodies",
            stock=config["stock"],
            initial_position_mm=[config["initial_position_mm"][0], config["initial_position_mm"][2]],
        )
        if facets:
            mesh = dense_bar_mesh(facets)
            model = camlib.TurningToolModel(
                body=mesh,
                placement=camlib.TurningModelPlacement(nose_center_mm=(0, 0, 0), x_axis=(1, 0, 0), z_axis=(0, 0, 1)),
                axial_resolution_mm=0.25,
                insert_check=camlib.TurningModelWithoutInsert(reason="synthetic bar inside the benchmark holder's box"),
                max_hull_error_mm=budget,
            )
            job["tools"][1]["tool"]["body"]["model"] = model.to_dict()
        with tempfile.TemporaryDirectory(prefix="camlib model cost ") as directory:
            path = Path(directory) / "job.json"
            path.write_text(json.dumps(job))
            result = json.loads(subprocess.check_output([executable, str(path), str(repeat)], text=True))
        signature = result["physical_sha256"]
        if reference is None:
            reference = signature
        if signature != reference:
            raise AssertionError(f"{name}: motions or stock differ from the built-in control")
        result.update(
            name=name,
            triangles=len(mesh.triangles) if facets else 0,
            max_hull_error_mm=budget,
            repeats=repeat,
            median_native_ms_per_move=1000 * statistics.median(result["warm_plan_seconds"]) / result["moves"],
        )
        rows.append(result)
    for row in rows:
        row["cost_multiple_of_builtin"] = row["median_native_ms_per_move"] / rows[0]["median_native_ms_per_move"]
    return dict(environment=_environment(), input_decoding_timed=False, rows=rows)


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
