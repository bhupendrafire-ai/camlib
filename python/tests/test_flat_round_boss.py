"""Regression for contour-parallel flat finishing around a faceted round boss."""

from dataclasses import replace
import math
import unittest

import cadquery as cq

import camlib
from camlib.cadquery import export_stock_mesh, export_surface_mesh
from camlib.simulation import OperationUpdate, Simulation, SimulationError, Update, VerificationUpdate
from camlib.tools import HolderSection, ToolAssembly


def round_boss_job(*, faceted=False):
    base = cq.Workplane("XY").box(40, 30, 4, centered=False)
    plane = cq.Workplane("XY").center(21.372, 16.018).workplane(offset=4)
    if faceted:
        angles = [0, 0.00003 / 7, *(math.tau * i / 72 for i in range(1, 72))]
        boss = plane.polyline([(7 * math.cos(a), 7 * math.sin(a)) for a in angles]).close().extrude(3)
    else:
        boss = plane.circle(7).extrude(3)
    target = base.union(boss)
    # The supported polyhedral stock model circumscribes the boss and leaves
    # 0.1 mm on both flats. The circular CAD wall is faceted by the exporter.
    radius = 7 / math.cos(math.pi / 32)
    angles = [math.tau * i / 32 for i in range(32)]
    polygon = [(radius * math.cos(a), radius * math.sin(a)) for a in angles]
    blank = cq.Workplane("XY").box(40, 30, 4.1, centered=False).union(plane.polyline(polygon).close().extrude(3.1))
    tool = ToolAssembly(
        "mill6",
        camlib.EndMill("em6", diameter=6, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )
    op = camlib.Flat(
        "flats",
        tool,
        export_surface_mesh(target, max_error_mm=0.04),
        0,
        2,
        14,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        entry=camlib.RampEntry(angle_deg=3, min_length_mm=1),
    )
    return dict(
        stock=camlib.StockSpec.from_mesh(export_stock_mesh(blank), 0.4),
        tools=(tool,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 14),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )


class FlatRoundBossTest(unittest.TestCase):
    def check_finish(self, job, step, radial):
        op = replace(job["operations"][0], stepover_mm=step, radial_stock_to_leave_mm=radial)
        moves = 0
        report = None
        with Simulation(**dict(job, operations=(op,))) as sim:
            for update in sim.run_operations():
                if isinstance(update, Update):
                    moves += 1
                    self.assertLess(moves, 5000, "round-boss stream did not terminate")
                elif isinstance(update, OperationUpdate) and update.stage == "end":
                    report = update.report
                elif isinstance(update, VerificationUpdate):
                    self.assertNotEqual(update.status, "violation")
            self.assertTrue(sim.state.finished)
            self.assertGreater(sim.state.volume()["removed_lower_mm3"], 0)
        self.assertEqual(report["levels_z_mm"], [7, 4])
        self.assertEqual(report["runs_completed"], report["runs"])
        self.assertEqual(report["dropped_length_mm"], 0)

    def test_round_boss_finishes_both_levels_with_and_without_radial_stock(self):
        job = round_boss_job()
        for step in (2, 2.5, 3):
            for radial in (0, 0.2):
                with self.subTest(step=step, radial=radial):
                    self.check_finish(job, step, radial)

    def test_faceted_boss_with_a_30_nm_edge_finishes(self):
        self.check_finish(round_boss_job(faceted=True), 2.5, 0)

    def test_round_boss_under_stock_refuses_when_no_entry_is_declared(self):
        job = round_boss_job()
        op = replace(job["operations"][0], stepover_mm=3, entry=None)
        stock = camlib.StockSpec((-10, -10, 0), (50, 40, 8), 0.4)
        with Simulation(**dict(job, operations=(op,), stock=stock)) as sim:
            with self.assertRaisesRegex(SimulationError, "declares no entry"):
                for moves, _ in enumerate(sim.run_operations()):
                    self.assertLess(moves, 5000, "unplannable round-boss stream did not terminate")
