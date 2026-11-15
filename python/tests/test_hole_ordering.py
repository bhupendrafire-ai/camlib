"""Order-only drilling controls shared by source and installed-package runs."""

from dataclasses import replace
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib import HoleOrdering, OperationNode, StockSpec
from camlib.dynamics import MotionPolicy
from camlib.export import ExportRefused
from camlib.simulation import OperationUpdate, Simulation, SimulationError
from test_drill_cycles import job, linuxcnc


CENTERS = ((34, 18), (6, 6), (34, 6), (6, 18), (20, 6), (20, 18))


def fixture(canned=False):
    source = job(canned_cycle=canned, peck_mm=2)
    setup = source.setups[0]
    operation = replace(setup.operations[0].operation, centers_mm=CENTERS)
    setup = replace(
        setup, stock=StockSpec((0, 0, 0), (40, 24, 10), 0.2), operations=(OperationNode(operation, sources=()),)
    )
    source = replace(source, setups=(setup,))
    machine = replace(linuxcnc(source), motion_policy=MotionPolicy(100, 6000, feed_reserve_fraction=0))
    ordered = replace(operation, ordering=HoleOrdering(machine, 10_000))
    return source, replace(source, setups=(replace(setup, operations=(OperationNode(ordered, sources=()),)),)), machine


def run(source):
    setup = source.compile().setup("top")
    moves, report = [], None
    with Simulation(**setup.simulation_options()) as simulation:
        for update in simulation.run_operations():
            if isinstance(update, OperationUpdate):
                if update.stage == "end":
                    report = update.report
            elif hasattr(update, "motion"):
                moves.append(update.motion)
        assert simulation.state.finished
        return moves, simulation.state.heights.tobytes(), report


class HoleOrderingTest(unittest.TestCase):
    def test_actual_explicit_and_canned_moves_preserve_cuts_stock_and_checked_post(self):
        for canned in (False, True):
            with self.subTest(canned=canned):
                source, ordered, machine = fixture(canned)
                old, old_stock, before = run(source)
                new, new_stock, after = run(ordered)
                self.assertNotIn("ordering", before)
                report = after["ordering"]
                self.assertLess(report["ordered_seconds"], 0.8 * report["given_seconds"])
                self.assertEqual(sorted(report["order"]), list(range(len(CENTERS))))
                self.assertEqual(report["order"][-1], len(CENTERS) - 1)
                self.assertLessEqual(report["work"], 10_000)
                self.assertEqual(old_stock, new_stock)

                def cuts(moves):
                    return sorted(
                        json.dumps({k: m[k] for k in ("curve", "kind", "feed_mm_min")}, sort_keys=True)
                        for m in moves
                        if m["kind"] != "rapid"
                    )

                self.assertEqual(cuts(old), cuts(new))
                for motion in new:
                    curve = motion["curve"]
                    if motion["kind"] == "rapid" and curve["start"][:2] != curve["end"][:2]:
                        self.assertGreaterEqual(min(curve["start"][2], curve["end"][2]), 12)
                self.assertEqual(old[-1]["curve"]["end"], new[-1]["curve"]["end"])
                with ordered.post("top", machine=machine) as posted:
                    posted.wait()
                    self.assertIsNone(posted.status()["error"])
                    self.assertTrue(posted.posting_status()["finished"])
                    self.assertEqual(posted.posting_status()["checked_motions"], posted.status()["available"])
                    for index in range(posted.status()["available"]):
                        self.assertTrue(posted.posted_motion(index)["stock_committed"])
                    with TemporaryDirectory(prefix="camlib ordered export ") as folder:
                        target = Path(folder) / "holes.nc"
                        with self.assertRaisesRegex(ExportRefused, "machine"):
                            posted.export(target)
                        self.assertFalse(target.exists())

    def test_explicit_settings_and_complete_route_work_budget_are_required(self):
        source, ordered, machine = fixture()
        with self.assertRaisesRegex(ValueError, "motion_policy"):
            HoleOrdering(replace(machine, motion_policy=None), 100).to_dict()
        for value in (0, -1, True, 1.5):
            with self.assertRaisesRegex(ValueError, "positive integer"):
                HoleOrdering(machine, value).to_dict()
        setup = ordered.setups[0]
        operation = setup.operations[0].operation
        limited = replace(operation, ordering=HoleOrdering(machine, 24))
        _, _, report = run(replace(ordered, setups=(replace(setup, operations=(OperationNode(limited, sources=()),)),)))
        self.assertTrue(report["ordering"]["work_limit_reached"])
        self.assertEqual(report["ordering"]["order"], list(range(6)))
        invalid = replace(operation, ordering=HoleOrdering(machine, 23))
        with self.assertRaisesRegex(SimulationError, "max_work"):
            run(replace(ordered, setups=(replace(setup, operations=(OperationNode(invalid, sources=()),)),)))
        self.assertNotIn("ordering", source.setups[0].operations[0].operation.to_dict())

    def test_ties_duplicate_centers_and_definition_identity(self):
        source, _, machine = fixture()
        setup = source.setups[0]
        operation = replace(setup.operations[0].operation, centers_mm=((10, 12), (10, 12), (28, 12)))
        operation = replace(operation, ordering=HoleOrdering(machine, 1000))
        ordered = replace(source, setups=(replace(setup, operations=(OperationNode(operation, sources=()),)),))
        first, stock, report = run(ordered)
        second, again, other = run(ordered)
        self.assertEqual(first, second)
        self.assertEqual(stock, again)
        self.assertEqual(report["ordering"], other["ordering"])
        self.assertEqual(report["ordering"]["order"], [0, 1, 2])
        self.assertIn("ordering", operation.to_dict())
        self.assertNotEqual(
            source.compile().setup("top").provenance[0]["input_digest"],
            ordered.compile().setup("top").provenance[0]["input_digest"],
        )
