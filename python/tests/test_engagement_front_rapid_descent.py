"""engagement_front rapid descents that camlib's own stream check certifies.

With rapid_descent_clearance_mm, a descent into a column the committed cuts
prove clear is rapid down to just above it. The proof is exact, but the
stream's stock-contact check reads whole-cell stock bounds: a rapid that
stopped above the exact height only, beside a partly cut cell still bounded
at the stock top, came back "uncertain" and the program could not be exported.
The rapid now also stops above the cell bounds under the tool.
"""

import unittest

from camlib.operations import AdaptiveClearing, Roughing
from camlib.simulation import OperationUpdate, Simulation, StockSpec, Update, VerificationUpdate
from camlib.surface import SurfaceMesh
from camlib.tools import EndMill, HolderSection, ToolAssembly
from camlib.verification import StockContactOptions, StreamChecks

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


class RapidDescentTest(unittest.TestCase):
    def run_rough(self, clearance):
        mesh = SurfaceMesh(box(12, 28, 12, 28, 0, 9), 0.0)
        rough = Roughing(
            "rough",
            TOOL,
            mesh,
            top_z_mm=10,
            floor_z_mm=1,
            stepdown_mm=4,
            stepup_mm=1,
            clearance_z_mm=20,
            retract_z_mm=11,
            feed_z_mm=11,
            rapid_descent_clearance_mm=clearance,
            clearing=AdaptiveClearing(1.0, path_strategy="engagement_front"),
            radial_allowance_mm=0.2,
            bottom_allowance_mm=0.2,
            **CUT,
        )
        statuses, deep, report = [], 0, None
        with Simulation(
            stock=StockSpec((0, 0, 0), (40, 40, 10), 0.5),
            tools=(TOOL,),
            operations=(rough,),
            revision="1",
            initial_position_mm=(0, 0, 20),
            verification=StreamChecks(stock_contact=StockContactOptions()),
        ) as sim:
            for u in sim.run_operations():
                if isinstance(u, OperationUpdate) and u.stage == "end":
                    report = u.report
                elif isinstance(u, Update):
                    c = u.motion["curve"]
                    if u.motion["kind"] == "rapid" and c["end"][2] < c["start"][2] and c["end"][2] < 11 - 1e-9:
                        deep += 1
                elif isinstance(u, VerificationUpdate) and u.status not in (None, "clear"):
                    statuses.append((u.completed, u.status))
            self.assertTrue(sim.state.finished)
        return statuses, deep, report

    def test_rapid_descents_are_certified_by_the_stream_check(self):
        statuses, deep, report = self.run_rough(1.0)
        self.assertEqual(statuses, [])
        self.assertGreater(deep, 0)
        self.assertGreater(report["completed_layers"], 2)
        plain, none_deep, _ = self.run_rough(None)
        self.assertEqual(plain, [])
        self.assertEqual(none_deep, 0)


if __name__ == "__main__":
    unittest.main()
