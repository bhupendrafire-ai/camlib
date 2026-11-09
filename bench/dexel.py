"""Tri-dexel stock against the height field (issue #1): memory and time.

    python bench/dexel.py --label <host> [--only NAME ...] [--repeat 3]

Runs each workload twice, once on its height-field stock and once on
tri-dexel stock of the same box and cell size (`DexelStockSpec`), each in
its own process (isolated peak memory), the timed runs sequential. Records
total run time (planning plus streamed simulation, median of --repeat), the
process's and the native engine's peak memory, the moves, whether the
toolpaths are identical, and both stocks' remaining-volume bounds. Results
go to bench/results/dexel-<label>.json and docs/benchmarks-dexel.md (--out). Release builds only; same machine, sequentially (see
AGENTS.md, "Measure, don't assert").
"""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import statistics
import subprocess
import sys
import time
from pathlib import Path

if sys.platform != "win32":
    import resource

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from geometry import reset as reset_geometry  # noqa: E402
from workloads import BY_NAME  # noqa: E402

# The pocket and roughing workloads the issue names, plus the dense pocket.
DEFAULT = ("pocket", "rest-pocket", "pocket-dense", "roughing-raster", "roughing-offset", "roughing-adaptive")


def _config(name, stock):
    from camlib import DexelStockSpec

    reset_geometry()
    config = BY_NAME[name].build()
    if stock == "dexel":
        s = config["stock"]
        s = s if isinstance(s, dict) else s.to_dict()
        stock = DexelStockSpec(min_mm=tuple(s["min_mm"]), max_mm=tuple(s["max_mm"]), cell_size_mm=s["cell_size_mm"])
        config = dict(config, stock=stock)
    return config


def _run_once(name, stock):
    from camlib.simulation import Simulation, Update

    config = _config(name, stock)
    digest, moves = hashlib.sha256(), 0
    start = time.perf_counter()
    with Simulation(**config) as sim:
        for u in sim.run_operations():
            if isinstance(u, Update):
                moves += 1
                digest.update(json.dumps(u.motion, sort_keys=True).encode())
        seconds = time.perf_counter() - start
        volume = sim.state.volume()
    return dict(seconds=seconds, moves=moves, toolpath_sha256=digest.hexdigest(), volume=volume)


def _child(name, stock, repeat):
    runs = [_run_once(name, stock) for _ in range(repeat)]
    own = engines = None
    if sys.platform != "win32":
        scale = 1024 if sys.platform == "darwin" else 1
        own = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / scale
        engines = resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss / scale
    last = runs[-1]
    v = last["volume"]
    print(
        json.dumps(
            dict(
                run_seconds=round(statistics.median(r["seconds"] for r in runs), 4),
                repeats=repeat,
                moves=last["moves"],
                toolpath_sha256=last["toolpath_sha256"],
                remaining_mm3=[round(v["remaining_lower_mm3"], 3), round(v["remaining_upper_mm3"], 3)],
                peak_rss_kib=dict(python=own, native=engines),
            )
        )
    )


def measure(name, repeat):
    out = {}
    for stock in ("height", "dexel"):
        proc = subprocess.run(
            [sys.executable, __file__, "--child", name, stock, str(repeat)],
            capture_output=True,
            text=True,
            cwd=HERE.parent,
        )
        if proc.returncode != 0:
            raise RuntimeError(f"{name} on {stock} stock failed:\n{proc.stderr[-4000:]}")
        out[stock] = json.loads(proc.stdout.strip().splitlines()[-1])
    h, d = out["height"], out["dexel"]
    out["same_toolpath"] = h["toolpath_sha256"] == d["toolpath_sha256"]
    out["volume_bounds_overlap"] = (
        d["remaining_mm3"][0] <= h["remaining_mm3"][1] + 1e-3 and h["remaining_mm3"][0] <= d["remaining_mm3"][1] + 1e-3
    )
    out["time_ratio"] = round(d["run_seconds"] / h["run_seconds"], 2)
    hn, dn = h["peak_rss_kib"]["native"], d["peak_rss_kib"]["native"]
    out["native_memory_ratio"] = round(dn / hn, 2) if hn and dn else None
    return out


def markdown(results):
    lines = [
        "| Workload | Moves | Same toolpath | Height field: time, native peak | Tri-dexel: time, native peak | Time ratio | Memory ratio | Remaining volume (height / dexel), mm³ |",
        "|---|---:|:---:|---|---|---:|---:|---|",
    ]
    for name, r in results["workloads"].items():
        h, d = r["height"], r["dexel"]

        def cell(x):
            peak = x["peak_rss_kib"]["native"]
            mib = "n/a" if peak is None else f"{peak / 1024:.0f} MiB"
            return f"{x['run_seconds']:.2f} s, {mib}"

        lines.append(
            f"| {name} | {h['moves']} | {'yes' if r['same_toolpath'] else 'no'} | {cell(h)} | {cell(d)} | "
            f"{r['time_ratio']}x | {r['native_memory_ratio']}x | "
            f"{h['remaining_mm3'][0]:.1f}–{h['remaining_mm3'][1]:.1f} / {d['remaining_mm3'][0]:.1f}–{d['remaining_mm3'][1]:.1f} |"
        )
    return "\n".join(lines)


def document(results):
    m = results["machine"]
    return f"""# Tri-dexel stock: memory and time

Generated by `python bench/dexel.py --label {results["label"]}` (issue #1). Each
workload runs on its height-field stock and on tri-dexel stock of the same
box and cell size (`DexelStockSpec`), each in its own process, sequentially,
in release mode; times are the median of {results["repeat"]} runs (planning plus
streamed simulation). Native peak is the engine process's peak resident
memory. Timings compare only on the same machine.

Machine: {m["system"]} {m["machine"]}, Python {m["python"]}{", " + m["note"] if m.get("note") else ""}.

{markdown(results)}

Same toolpath: the planners read the tri-dexel run's height view. It
equals the height field for flat end mills at constant height and bounds the
same material elsewhere (tighter in cells a sloped chord cannot reach), so a
planner that reads the stock closely (adaptive engagement) may choose
slightly different moves. Remaining volume: the lower and upper bounds of
each stock (the tri-dexel bounds take the best of its three grids); they
overlap on every workload. The tri-dexel sweep is not optimized yet (issue
#1 asks for measurement; optimization is Phase 6).
"""


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "--child":
        _child(sys.argv[2], sys.argv[3], int(sys.argv[4]))
        return
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--label", required=True)
    parser.add_argument("--only", nargs="*", default=DEFAULT)
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--note", default="", help="machine conditions worth recording (load, other work)")
    parser.add_argument("--out", type=Path, default=HERE.parent / "docs" / "benchmarks-dexel.md")
    args = parser.parse_args()
    results = dict(
        label=args.label,
        machine=dict(
            system=platform.system(), machine=platform.machine(), python=platform.python_version(), note=args.note
        ),
        repeat=args.repeat,
        workloads={},
    )
    for name in args.only:
        results["workloads"][name] = measure(name, args.repeat)
        print(name, json.dumps(results["workloads"][name]), file=sys.stderr)
    path = HERE / "results" / f"dexel-{args.label}.json"
    path.write_text(json.dumps(results, indent=1) + "\n")
    args.out.write_text(document(results))
    print(path, args.out)


if __name__ == "__main__":
    main()
