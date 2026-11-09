"""The material check's work budgets grow with the stock grid (#108).

#72's 110 x 70 mm stock at 0.1 mm has 770 000 cells: more tile nodes than the
old fixed default of 1 000 000. Unset budgets complete it; a budget set too low
stops the check, and the report and export refusal name it and the value that
completes it.
"""

from array import array
from dataclasses import asdict
import unittest

from camlib.export import describe
from camlib.material import Options, SurfaceTarget, Verifier
from camlib.simulation import StockSpec
from camlib.surface import SurfaceMesh

PLATE = SurfaceMesh(
    (
        ((0, 0, 20), (110, 0, 20), (110, 70, 20)),
        ((0, 0, 20), (110, 70, 20), (0, 70, 20)),
    )
)


class MaterialBudgetTest(unittest.TestCase):
    def setUp(self):
        self.spec = StockSpec((0, 0, 0), (110, 70, 21), 0.1)
        self.heights = array("d", [20.0]) * (1100 * 700 * 2)
        self.target = SurfaceTarget("plate", "1", PLATE, 0)

    def test_unset_budgets_grow_with_the_grid(self):
        options = Options()
        self.assertIsNone(options.max_tile_nodes)
        self.assertIsNone(options.max_boundary_tests)
        self.assertIsNone(asdict(options)["max_tile_nodes"])
        report = Verifier(self.target, options).check_bounds(self.heights, self.spec)
        self.assertTrue(report["complete"])
        self.assertGreater(report["stats"]["max_tile_nodes"], 1_000_000)
        self.assertEqual(report["stats"]["tile_nodes"], report["stats"]["max_tile_nodes"])
        self.assertNotIn("stopped_by", report["stats"])

    def test_a_budget_that_stops_the_check_is_named_with_the_value_that_completes_it(self):
        report = Verifier(self.target, Options(max_tile_nodes=1_000_000)).check_bounds(self.heights, self.spec)
        self.assertFalse(report["complete"])
        stop = report["stats"]["stopped_by"]
        self.assertEqual((stop["budget"], stop["limit"]), ("max_tile_nodes", 1_000_000))
        rerun = Verifier(self.target, Options(max_tile_nodes=stop["completes_at"])).check_bounds(
            self.heights, self.spec
        )
        self.assertTrue(rerun["complete"])
        # The export refusal's text names it.
        text = describe(dict(status="uncertain", findings=[report]))
        self.assertIn("analysis incomplete", text)
        self.assertIn(f"max_tile_nodes = {stop['completes_at']} completes it", text)
        with self.assertRaises(ValueError):
            Verifier(self.target, Options(max_tile_nodes=10_000_001))


if __name__ == "__main__":
    unittest.main()
