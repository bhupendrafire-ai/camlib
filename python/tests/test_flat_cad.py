"""CAD-driven Flat and its opt-in last wall contour, through the native stream."""

from dataclasses import replace
import math
import unittest

import cadquery as cq

import camlib
from camlib.cadquery import export_stock_mesh, export_surface_mesh, flat_floors
from camlib.simulation import OperationUpdate, Simulation, SimulationError, Update, VerificationUpdate
from camlib.tools import HolderSection, ToolAssembly
from test_flat_offset import level_cuts, level_distance


CENTER = (21.372, 16.018)
RADIUS = 7


def settings(*, surface_error_mm=0.04):
    plane = cq.Workplane("XY").center(*CENTER).workplane(offset=4)
    part = cq.Workplane("XY").box(40, 30, 4, centered=False).union(plane.circle(RADIUS).extrude(3))
    # The straight-sided blank circumscribes the rounded boss with stock
    # on both horizontal faces and extra radial material on the wall.
    radius = 7.35 / math.cos(math.pi / 32)
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
    floors = flat_floors(part)
    op = camlib.Flat(
        "flats",
        tool,
        export_surface_mesh(part, max_error_mm=surface_error_mm),
        0,
        2.5,
        14,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        tool_number=1,
        length_register=1,
        coolant="flood",
        radial_stock_to_leave_mm=0.2,
        entry=camlib.RampEntry(angle_deg=3, min_length_mm=1),
        floors=floors,
    )
    return dict(
        stock=camlib.StockSpec.from_mesh(export_stock_mesh(blank), 0.4),
        tools=(tool,),
        operations=(op,),
        revision="1",
        initial_position_mm=(0, 0, 14),
        verification=camlib.StreamChecks(stock_contact=camlib.StockContactOptions()),
    )


class FlatCadTest(unittest.TestCase):
    def test_cad_hole_drives_the_wall_and_the_opt_in_orders_it_last(self):
        config = settings()
        op = config["operations"][0]
        self.assertEqual([f.z_mm for f in op.floors], [7, 4])
        self.assertEqual(len(op.floors[1].region.components[0].holes), 1)
        for finish in (False, True):
            with self.subTest(finish=finish):
                motions = []
                with Simulation(**dict(config, operations=(replace(op, finish_walls=finish),))) as run:
                    for event in run.run_operations():
                        if isinstance(event, Update):
                            motions.append(event.motion)
                        elif isinstance(event, OperationUpdate) and event.stage == "end":
                            report = event.report
                        elif isinstance(event, VerificationUpdate):
                            self.assertEqual(event.status, "clear")
                    self.assertTrue(run.state.finished)
                    self.assertGreater(run.state.volume()["removed_lower_mm3"], 0)
                self.assertEqual(report["levels_z_mm"], [7, 4])
                self.assertEqual(report.get("wall_passes", 0), int(finish))
                cuts = [m for m in level_cuts(motions, 4) if m["kind"] == "cut"]
                distances = [level_distance(CENTER, m["curve"]) - RADIUS - 3 for m in cuts]
                self.assertGreaterEqual(min(distances), 0.199)
                self.assertLess(min(distances), 0.21)
                if finish:
                    self.assertLess(distances[-1], 0.21)
                else:
                    self.assertGreater(distances[-1], 0.21)

    def test_unmatched_trim_height_refuses_before_emitting_motion(self):
        config = settings()
        op = config["operations"][0]
        off_floor = replace(op, floors=(replace(op.floors[0], z_mm=6.5),))
        with Simulation(**dict(config, operations=(off_floor,))) as run:
            with self.assertRaisesRegex(SimulationError, "no matching horizontal mesh level"):
                for event in run.run_operations():
                    self.assertNotIsInstance(event, Update)

    def test_posted_wall_finish_passes_machine_stock_and_floor_material_checks(self):
        from camlib.material import SurfaceTarget
        from camlib.post import LoweringOptions, ToolSetting
        from camlib.verification import MaterialCheck
        from test_machine import model
        from test_post import fixture

        # Use a target enclosure finer than the default 0.001 mm material tolerance.
        config = settings(surface_error_mm=0.0004)
        op = replace(config["operations"][0], finish_walls=True)
        target = SurfaceTarget("round-boss", "1", op.surface, 0)
        checks = replace(config["verification"], material=(MaterialCheck(target),))
        setup = camlib.Setup(
            "top", config["stock"], config["tools"], (op,), config["initial_position_mm"], verification=checks
        )
        job = camlib.Job("round-boss", "1", (setup,))
        program = job.compile().setup("top").program_header()
        machine = replace(
            fixture()[1],
            tools=(
                ToolSetting.for_tool(
                    program.tools[0], number=1, length_register=1, position_after_mm=config["initial_position_mm"]
                ),
            ),
        )
        crash = replace(model(program, machine), initial_axes_mm=(0, 0, 14 + program.tools[0].gauge_length_mm))
        with job.post(
            "top",
            machine=replace(machine, crash_model=crash),
            lowering=LoweringOptions(native_arcs=True, chord_tolerance_mm=0.0005),
        ) as posted:
            result = posted.export()
            material = posted.verification_status()["material"]
        self.assertEqual(material["target_id"], target.id)
        self.assertEqual(material["status"], "complete")
        self.assertEqual(material["gouge"]["definite_cells"], 0)
        self.assertEqual(result["accepted"], ())
        self.assertEqual(result["findings"], ())
        self.assertEqual(result["machine"]["status"], "clear")
        self.assertGreater(result["motions"], 0)
