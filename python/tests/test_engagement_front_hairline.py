"""engagement_front roughing beside a hairline feature between grid centres.

The engagement_front planner steers on a grid and certifies every stroke
against the exact region. A boundary feature narrower than a grid cell (here
the shadow of a 2 um fin, as thin walls and seams of real meshes leave) that
falls between cell centres used to be missed by the grid, so the planner
steered over it and the certification refused the stroke as a planner defect.
The fin lies exactly on a grid line (the steering cell is 0.125 mm and the
grid starts a whole number of cells below the stock), between two centre rows.
"""

import unittest

from camlib.material import SurfaceTarget, Verifier
from camlib.operations import AdaptiveClearing, Roughing
from camlib.simulation import OperationUpdate, Simulation, StockSpec
from camlib.surface import SurfaceMesh
from camlib.tools import EndMill, HolderSection, ToolAssembly

TOOL = ToolAssembly(
    "em6",
    EndMill("em6", diameter=6, flute_length=20, overall_length=60),
    30,
    60,
    (HolderSection(30, 60, 10, 10),),
)
CUT = dict(feed_mm_min=900, plunge_mm_min=250, spindle_rpm=6000, tool_number=1, length_register=1, coolant="flood")


def box(x0, x1, y0, y1, z0, z1):
    p = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    quads = ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5))
    triangles = []
    for a, b, c, d in quads:
        triangles += [(p[a], p[b], p[c]), (p[a], p[c], p[d])]
    return triangles


class HairlineFeatureTest(unittest.TestCase):
    def test_engagement_front_steers_around_a_fin_between_grid_centres(self):
        # The fin stands 4 mm on the stock bottom, 8 mm long and 0.002 mm
        # thick, centred on Y10.
        mesh = SurfaceMesh(box(6, 14, 9.999, 10.001, 0, 4), 0.0)
        rough = Roughing(
            "rough",
            TOOL,
            mesh,
            top_z_mm=5,
            floor_z_mm=1,
            stepdown_mm=2,
            clearance_z_mm=10,
            clearing=AdaptiveClearing(1.0, path_strategy="engagement_front"),
            radial_allowance_mm=0.2,
            bottom_allowance_mm=0.2,
            **CUT,
        )
        report = None
        with Simulation(
            stock=StockSpec((0, 0, 0), (20, 20, 5), 0.5),
            tools=(TOOL,),
            operations=(rough,),
            revision="1",
            initial_position_mm=(0, 0, 10),
        ) as sim:
            for update in sim.run_operations():
                if isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
            self.assertTrue(sim.state.finished)
            material = Verifier(SurfaceTarget("fin", "1", mesh, 0)).check(sim.state)
        self.assertEqual(report["completed_layers"], 2)
        for layer in report["layers"]:
            self.assertEqual(layer["clearing"]["strategy"], "engagement_front")
            self.assertGreater(layer["clearing"]["passes"], 0)
        self.assertEqual(material["gouge"]["definite_cells"], 0)


if __name__ == "__main__":
    unittest.main()
