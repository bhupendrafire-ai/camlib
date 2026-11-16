"""Small operations on large blanks retain the fine grid and ordinary stream."""

from dataclasses import replace
import unittest

from camlib.simulation import OperationUpdate, Simulation, StockSpec
from support.adaptive_pocket import adaptive_job


class BoundedFrontGridTest(unittest.TestCase):
    def test_large_blank_completes_small_pocket_without_coarsening_or_lifting_the_cap(self):
        config = adaptive_job()
        # 40K stock cells, but >100M steering cells at the required R/20.
        # The operation remains the fixture's 20 mm L-shaped pocket.
        config['stock'] = StockSpec((0, 0, 0), (1000, 1000, 10), 5)
        config['operations'] = (replace(config['operations'][0], floor_z_mm=9, stepdown_mm=1),)
        reports = []
        with Simulation(**config) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == 'end':
                    reports.append(update.report)
            self.assertTrue(sim.state.finished)
            self.assertGreater(sim.state.completed, 0)
        self.assertEqual(len(reports), 1)
        report = reports[0]
        self.assertEqual(report['completed_layers'], 1)
        clearing = report['layers'][0]['clearing']
        self.assertTrue(
            any('bounded to the operation region' in d and 'unchanged' in d for d in clearing['diagnostics'])
        )
        self.assertGreater(clearing['certified_strokes'], 0)
