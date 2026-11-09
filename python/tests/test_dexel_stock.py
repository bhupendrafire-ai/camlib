"""Tri-dexel stock runs (issue #1): the standalone three-axis jobs on dexel
stock against the height field, streaming, seeking within a move,
checkpoints, save and reopen, and posted replay."""

from dataclasses import replace
from functools import lru_cache
import importlib.util
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib import DexelStockSpec
from camlib.dexel_state import DexelStockState, heights_at
from camlib.persistence import open_run, save_run
from camlib.playback import Cursor, Player
from camlib.simulation import OperationUpdate, ProtocolError, StockState, Update, stock_state


@lru_cache(None)
def example():
    path = Path(__file__).resolve().parents[2] / "examples/standalone_job.py"
    spec = importlib.util.spec_from_file_location("dexel_standalone_example", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def engine():
    return os.environ.get("CAMLIB_TEST_ENGINE")


def dexel(job):
    """The same job on tri-dexel stock of the same box and cell size."""
    setup = job.setups[0]
    s = setup.stock
    stock = DexelStockSpec(min_mm=s.min_mm, max_mm=s.max_mm, cell_size_mm=s.cell_size_mm)
    return replace(job, setups=(replace(setup, stock=stock),))


def run(job):
    motions, checks, reports = [], [], {}
    with job.regenerate(None, executable=engine()) as r:
        for event in r:
            u = event.update
            if isinstance(u, Update):
                motions.append(u.motion)
            elif isinstance(u, OperationUpdate) and u.stage == "end":
                reports[u.operation] = u.report
            elif hasattr(u, "check") and u.check is not None:
                checks.append(u.check.get("status"))
        assert r.finished
        return dict(motions=motions, checks=checks, reports=reports, state=r.states["top"])


class DexelJobs(unittest.TestCase):
    """Acceptance: the existing three-axis jobs give the same results on
    dexel stock as on the height field (volume bounds overlap, the same
    check results)."""

    def compare(self, job):
        height = run(job)
        tri = run(dexel(job))
        self.assertIsInstance(height["state"], StockState)
        self.assertIsInstance(tri["state"], DexelStockState)
        self.assertEqual(len(tri["motions"]), len(height["motions"]))
        self.assertEqual(len(tri["checks"]), len(height["checks"]))
        self.assertEqual(tri["checks"].count("violation"), height["checks"].count("violation"))
        # The rays may decide contact the height view leaves uncertain.
        self.assertLessEqual(tri["checks"].count("uncertain"), height["checks"].count("uncertain"))
        h, d = height["state"].volume(), tri["state"].volume()
        self.assertLessEqual(d["remaining_lower_mm3"], h["remaining_upper_mm3"] + 1e-6)
        self.assertLessEqual(h["remaining_lower_mm3"], d["remaining_upper_mm3"] + 1e-6)
        self.assertGreater(d["removed_lower_mm3"], 0)
        # The height view bounds the same material as the height field.
        a = memoryview(bytes(height["state"].heights)).cast("d")
        b = memoryview(bytes(tri["state"].heights)).cast("d")
        self.assertEqual(len(a), len(b))
        for i in range(0, len(a), 2):
            self.assertLessEqual(b[i], a[i + 1] + 1e-9)
            self.assertLessEqual(a[i], b[i + 1] + 1e-9)
        return height, tri

    def test_standalone_pocket(self):
        height, tri = self.compare(example().build_job())

        # Same toolpath: the planners read the height view.
        def plain(motions):
            return [json.loads(json.dumps(m, sort_keys=True)) for m in motions]

        self.assertEqual(plain(tri["motions"]), plain(height["motions"]))

    def test_standalone_rest(self):
        self.compare(example().build_rest_job())

    def test_regeneration_cache_is_refused(self):
        from camlib.jobs import RegenerationCache
        from camlib.simulation import SimulationError

        with TemporaryDirectory() as directory, self.assertRaises((SimulationError, ProtocolError)) as raised:
            with dexel(example().build_job()).regenerate(RegenerationCache(directory), executable=engine()) as r:
                for _ in r:
                    pass
        self.assertIn("regeneration cache", str(raised.exception))


class DexelPlayback(unittest.TestCase):
    """Acceptance: streaming, seek (also within a move), checkpoint,
    save/reopen and posted replay work for dexel runs."""

    def test_seek_checkpoint_save_reopen_and_post(self):
        job = dexel(example().build_job())
        machine = example().machine_for(job)
        with TemporaryDirectory() as directory, job.post("top", machine=machine, executable=engine()) as posted:
            posted.wait()
            status = posted.status()
            end = status["available"]
            self.assertGreater(end, 10)
            player = Player(posted)
            final = player.seek(Cursor(end))
            self.assertIsInstance(final, DexelStockState)
            v_end = final.volume()
            # Within a move: between the stock before and after it.
            middle = end // 2
            while True:
                before = player.seek(Cursor(middle)).volume()
                after = player.seek(Cursor(middle + 1)).volume()
                if after["remaining_upper_mm3"] < before["remaining_upper_mm3"]:
                    break
                middle += 1
            shown = player.seek(Cursor(middle, 0.5))
            v = shown.volume()
            self.assertLessEqual(v["remaining_upper_mm3"], before["remaining_upper_mm3"])
            self.assertGreaterEqual(v["remaining_upper_mm3"], after["remaining_upper_mm3"])
            # Seeking back restores from a checkpoint and replays.
            self.assertEqual(player.seek(Cursor(3)).completed, 3)
            self.assertEqual(player.seek(Cursor(end)).volume(), v_end)
            # Checkpoint files round-trip.
            path = Path(directory) / "state.checkpoint.gz"
            final.write_checkpoint(path, compressed=True)
            again = stock_state(posted.wait_ready())
            again.restore_checkpoint(path, compressed=True)
            self.assertEqual(again.completed, final.completed)
            self.assertEqual(again.volume(), v_end)
            self.assertEqual(bytes(again.heights), bytes(final.heights))
            for axis in range(3):
                self.assertEqual(bytes(again.rays(axis)[1]), bytes(final.rays(axis)[1]))
            # Save and reopen without replanning.
            saved = Path(directory) / "saved run"
            save_run(posted, saved)
            reopened = open_run(saved)
            replayed = Player(reopened).seek(Cursor(end))
            self.assertEqual(replayed.volume(), v_end)
            # The posted program, decoded and replayed on dexel stock, exports
            # with the same refusals as a height-field run.
            exported = posted.export(Path(directory) / "pocket.nc", accept={"unchecked_machine"})
            self.assertTrue(Path(exported["path"]).exists())

    def test_receiver_refuses_a_height_field_frame(self):
        job = example().build_job()
        with job.regenerate(None, executable=engine()) as r:
            frame = next(e.update.frame for e in r if isinstance(e.update, Update))
        with dexel(job).regenerate(None, executable=engine()) as r:
            for _ in r:
                manifest = r._simulation.manifest_frame
                break
        fresh = stock_state(manifest)
        self.assertIsInstance(fresh, DexelStockState)
        with self.assertRaises(ProtocolError):
            fresh.accept(frame)
        self.assertEqual(fresh.completed, 0)

    def test_height_view_and_rays(self):
        out = run(dexel(example().build_job()))
        state = out["state"]
        # The pocket floor at Z 3, under the faced top at Z 5.
        lower, upper = heights_at(state, 9, 8)
        self.assertAlmostEqual(upper, 3, delta=1e-3)
        self.assertAlmostEqual(lower, 3, delta=1e-3)
        lower, upper = heights_at(state, 1, 1)
        self.assertAlmostEqual(upper, 5, delta=1e-3)
        grid = state.grid
        self.assertEqual(grid["cells"], [48, 32, 12])
        counts, values = state.rays("z")
        self.assertEqual(len(counts), 8 * grid["rays"][2])
        possible, certain = state.ray("z", 16 * 48 + 18)
        self.assertEqual(len(possible), 1)
        self.assertAlmostEqual(possible[0][1], 3, delta=1e-3)


T_PROFILE = ((0, 0), (20, 0), (20, 10), (12, 10), (12, 5), (15, 5), (15, 3), (5, 3), (5, 5), (8, 5), (8, 10), (0, 10))


def extrusion(profile, x0, x1):
    """A closed mesh of the simple counter-clockwise polygon ``profile`` (Y, Z)
    extruded along X."""

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    idx, caps = list(range(len(profile))), []
    while len(idx) > 3:
        n = len(idx)
        for i in range(n):
            a, b, c = (profile[idx[(i - 1) % n]], profile[idx[i]], profile[idx[(i + 1) % n]])
            if cross(a, b, c) > 0 and not any(
                p not in (a, b, c) and cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0
                for p in (profile[j] for j in idx)
            ):
                caps.append((idx[(i - 1) % n], idx[i], idx[(i + 1) % n]))
                del idx[i]
                break
    caps.append(tuple(idx))

    def at(x, p):
        return (float(x), float(p[0]), float(p[1]))

    out = []
    for a, b, c in caps:
        out.append((at(x0, profile[a]), at(x0, profile[c]), at(x0, profile[b])))
        out.append((at(x1, profile[a]), at(x1, profile[b]), at(x1, profile[c])))
    for i, p in enumerate(profile):
        q = profile[(i + 1) % len(profile)]
        out += [(at(x0, p), at(x0, q), at(x1, q)), (at(x0, p), at(x1, q), at(x1, p))]
    return out


class DexelUndercut(unittest.TestCase):
    """Acceptance: a T-slot cut in a block leaves an undercut the stock
    represents within tolerance (volume bounds enclose the analytic solid),
    and the material check against the part decides it."""

    def test_t_slot(self):
        from camlib.motion import Line, Motion
        from camlib.simulation import Simulation
        from camlib.surface import SurfaceMesh
        from camlib.tools import EndMill, HolderSection, Neck, ToolAssembly

        mill = ToolAssembly(
            "mill",
            EndMill("em4", diameter=4, flute_length=15, overall_length=60),
            40,
            70,
            (HolderSection(40, 70, 3, 3),),
        )
        tslot = ToolAssembly(
            "tslot",
            EndMill("ts10", diameter=10, flute_length=2, overall_length=60, shank_diameter=6, neck=Neck(3, 10)),
            20,
            60,
            (HolderSection(20, 60, 8, 8),),
        )
        moves = [
            Motion(1, "slot", "mill", Line((-5, 10, 3), (35, 10, 3)), 500),
            Motion(2, "slot", "mill", Line((35, 10, 3), (35, 10, 20)), None, kind="rapid"),
            Motion(3, "slot", "tslot", Line((35, 10, 20), (38, 10, 3)), None, kind="rapid"),
            Motion(4, "tslot", "tslot", Line((38, 10, 3), (-8, 10, 3)), 300),
        ]
        stock = DexelStockSpec((0, 0, 0), (30, 20, 10), cell_size_mm=0.1)
        with Simulation(stock, (mill, tslot), revision="t-slot", initial_position_mm=(-5, 10, 3)) as sim:
            for _ in sim.run(moves):
                pass
            state = sim.state
            self.assertIsInstance(state, DexelStockState)
            v = state.volume()
            exact = 6000 - 840 - 360
            self.assertLessEqual(v["remaining_lower_mm3"], exact)
            self.assertGreaterEqual(v["remaining_upper_mm3"], exact)
            self.assertLess(v["remaining_upper_mm3"] - v["remaining_lower_mm3"], 0.02 * exact)
            # Under the overhang: material below and above the T.
            possible, _ = state.ray("z", 60 * 300 + 60)
            self.assertEqual(len(possible), 2)
            self.assertAlmostEqual(possible[0][1], 3, delta=1e-4)
            self.assertAlmostEqual(possible[1][0], 5, delta=1e-4)
            part = SurfaceMesh(extrusion(T_PROFILE, 0, 30), surface_error_mm=0)
            report = state.compare_solid(part, 0.15)
            self.assertEqual(report["gouge"]["assessment"], "within_tolerance", report)
            self.assertEqual(report["residual"]["assessment"], "within_tolerance", report)


if __name__ == "__main__":
    unittest.main()
