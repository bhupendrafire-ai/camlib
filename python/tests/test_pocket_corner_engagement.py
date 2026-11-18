"""A hard angle also applies to confined adaptive-pocket wall strokes."""

from dataclasses import replace
import unittest

from camlib.simulation import OperationUpdate, Simulation, SimulationError
from support.adaptive_pocket import adaptive_job


class PocketCornerEngagementTest(unittest.TestCase):
    def run_pocket(self, job, **changes):
        op = replace(job["operations"][0], stepdown_mm=2, **changes)
        with Simulation(**dict(job, operations=(op,))) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
            return report, sim.state.volume()

    def test_bounded_confined_corners_keep_the_cad_l_pocket_coverage(self):
        job = adaptive_job()
        original, before = self.run_pocket(job)
        bounded, after = self.run_pocket(job, max_corner_engagement_deg=150)
        self.assertNotIn("max_corner_engagement_deg", original)
        self.assertEqual(bounded["max_corner_engagement_deg"], 150)
        base = original["layers"][0]["clearing"]
        proof = bounded["layers"][0]["clearing"]
        self.assertGreater(base["confined_max_engagement_deg"], 150)
        self.assertGreater(proof["confined_strokes"], 0)
        self.assertLessEqual(proof["confined_max_engagement_deg"], 150)
        self.assertLessEqual(proof["proven_max_engagement_deg"], 150)
        self.assertEqual(proof["wall_passes"], base["wall_passes"])
        self.assertEqual(proof["geometric_residual_area_mm2"], base["geometric_residual_area_mm2"])
        self.assertEqual(after, before)

    def test_an_unproven_corner_cannot_be_reclassified_to_clear_the_cap(self):
        job = adaptive_job()
        with self.assertRaisesRegex(SimulationError, "not proven within max_corner_engagement_deg 65"):
            self.run_pocket(job, max_corner_engagement_deg=65)

    def test_invalid_or_inapplicable_cap_is_refused(self):
        job = adaptive_job()
        with self.assertRaisesRegex(SimulationError, "max_corner_engagement_deg must be within"):
            self.run_pocket(job, max_corner_engagement_deg=180)
        for strategy in ("disk_front", "stock_front"):
            with (
                self.subTest(strategy=strategy),
                self.assertRaisesRegex(SimulationError, "requires path_strategy engagement_front"),
            ):
                self.run_pocket(job, path_strategy=strategy, max_corner_engagement_deg=150)
