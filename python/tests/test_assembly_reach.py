"""Issue #129: a low pocket beside a tall wall, judged by the tool assembly.

A long tool first clears the stock down to a shelf at Z8 up to a wall at
X30 that stays at the full 60 mm. A pocket 4 mm deep, 8 mm from that wall,
then takes its top from the shelf: a short tool in a slim holder reaches it,
and the independent stock-contact check finds every move clear; a wide holder
is refused before any motion with a typed reach refusal."""

import unittest

import camlib
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import (
    OperationUpdate,
    ReachRefused,
    Simulation,
    SimulationError,
    StockSpec,
    VerificationUpdate,
)
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StockContactOptions, StreamChecks

CUTTING = dict(feed_mm_min=600, plunge_mm_min=150, spindle_rpm=12000, coolant="flood")


def square(x0, y0, x1, y1):
    return PlanarRegion("r", (Component(Boundary.polygon(((x0, y0), (x1, y0), (x1, y1), (x0, y1)))),))


def assemblies(holder_radius):
    long = ToolAssembly(
        "long",
        EndMill("em20", diameter=20, flute_length=60, overall_length=90),
        65,
        100,
        (HolderSection(65, 100, 12, 12),),
    )
    short = ToolAssembly(
        "short",
        EndMill("em6", diameter=6, flute_length=6, overall_length=40),
        15,
        60,
        (HolderSection(15, 60, holder_radius, holder_radius),),
    )
    return long, short


def run(holder_radius, *, verification=None):
    long, short = assemblies(holder_radius)
    shelf = camlib.Pocket(
        "shelf", long, square(-12, -12, 30, 52), 60, 8, 13, 8, 65, tool_number=1, length_register=1, **CUTTING
    )
    low = camlib.Pocket(
        "low", short, square(10, 10, 22, 30), 8, 4, 2, 2, 65, tool_number=2, length_register=2, **CUTTING
    )
    return Simulation(
        StockSpec((0, 0, 0), (60, 40, 60), 0.5),
        (long, short),
        operations=(shelf, low),
        revision="1",
        initial_position_mm=(-20, -20, 70),
        verification=verification,
    )


class AssemblyReachTest(unittest.TestCase):
    def test_slim_holder_reaches_low_pocket_beside_tall_wall(self):
        checks = StreamChecks(stock_contact=StockContactOptions())
        with run(4, verification=checks) as sim:
            updates = list(sim.run_operations())
            self.assertTrue(sim.state.finished)
            reports = [u for u in updates if isinstance(u, VerificationUpdate) and u.check["kind"] == "motion"]
            self.assertTrue(reports)
            self.assertEqual({u.status for u in reports}, {"clear"})
            ends = [u for u in updates if isinstance(u, OperationUpdate) and u.stage == "end" and u.operation == "low"]
            top = ends[0].report["stock_top"]
            self.assertEqual(top["authored_top_z_mm"], 8)
            self.assertEqual(top["outside_upper_z_mm"], 60)

    def test_wide_holder_is_refused_with_a_typed_reach_refusal(self):
        with run(20) as sim, self.assertRaises(ReachRefused) as caught:
            list(sim.run_operations())
        error = caught.exception
        self.assertIsInstance(error, SimulationError)
        self.assertIn("holder section 0", str(error))
        self.assertNotIn("camlib-reach-refusal", str(error))
        r = error.refusal
        self.assertEqual((r.operation, r.tool, r.limit, r.holder_section), ("low", "short", "holder", 0))
        self.assertEqual((r.top_z_mm, r.z_mm), (60, 4))
        self.assertEqual((r.depth_needed_mm, r.depth_available_mm), (56, 15))


if __name__ == "__main__":
    unittest.main()
