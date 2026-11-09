"""Deburr (#11, #92): the top and accessible side edges of a pocketed part are
broken to the requested size with a chamfer, ball-end or bull-nose mill;
concave edges and unreachable stretches are reported, never silently
dropped."""

import unittest

from camlib import Deburr, Job, OperationNode, Pocket, Setup, StockSpec
from camlib.operations import HelicalEntry
from camlib.simulation import OperationUpdate
from camlib.tools import BallEndMill, BullEndMill, ChamferMill, EndMill, HolderSection, ToolAssembly

try:
    import cadquery as cq
except ImportError:  # pragma: no cover - CI installs CadQuery
    cq = None

EM6 = ToolAssembly(
    "em6", EndMill("em6", diameter=6, flute_length=18, overall_length=60), 30, 100, (HolderSection(30, 100, 8, 12),)
)
C90 = ToolAssembly(
    "c90",
    ChamferMill("c90", angle=90, tip_diameter=0.2, diameter=6, flute_length=4, overall_length=50, center_cutting=True),
    20,
    60,
    (HolderSection(20, 60, 8, 16),),
)


B3 = ToolAssembly(
    "b3",
    BallEndMill("b3", diameter=3, flute_length=6, overall_length=50),
    20,
    60,
    (HolderSection(20, 60, 8, 16),),
)
R4 = ToolAssembly(
    "r4",
    BullEndMill("r4", corner_radius=1, diameter=4, flute_length=6, overall_length=50),
    20,
    60,
    (HolderSection(20, 60, 8, 16),),
)


def part():
    # A 40 x 30 x 10 block with a 20 x 10 pocket 4 deep.
    return (
        cq.Workplane("XY")
        .box(40, 30, 10, centered=False)
        .cut(cq.Workplane("XY").box(20, 10, 4, centered=False).translate((10, 10, 6)))
    )


def job(levels, surface, region, deburr=None):
    pocket = Pocket(
        "pocket",
        EM6,
        region,
        top_z_mm=10,
        floor_z_mm=6,
        stepdown_mm=2,
        stepover_mm=2.5,
        clearance_z_mm=30,
        entry=HelicalEntry(),
        feed_mm_min=800,
        plunge_mm_min=300,
        spindle_rpm=10000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    deburr = deburr or Deburr(
        "deburr",
        C90,
        surface,
        levels,
        0.5,
        30,
        tip_clearance_mm=0.3,
        feed_mm_min=600,
        plunge_mm_min=200,
        spindle_rpm=10000,
        tool_number=2,
        length_register=2,
        coolant="flood",
    )
    setup = Setup(
        "top",
        StockSpec((0, 0, 0), (40, 30, 10), 0.1),
        (EM6, deburr.tool),
        (OperationNode(pocket, sources=()), OperationNode(deburr, sources=())),
        (0, 0, 40),
    )
    return Job("deburr", "1", (setup,))


@unittest.skipIf(cq is None, "CadQuery is required")
class DeburrTest(unittest.TestCase):
    def test_pocketed_part_edges_are_broken_and_concave_edges_reported(self):
        from camlib.cadquery import deburr_levels, export_floor_region, export_surface_mesh

        solid = part()
        levels = deburr_levels(solid, source={"test": "deburr"})
        # The top (with the pocket's outline as a hole) and the pocket floor.
        self.assertEqual([level.z_mm for level in levels], [10, 6])
        floor = levels[1].regions[0]
        surface = export_surface_mesh(solid, max_error_mm=0.001)
        pocket_region = export_floor_region(
            next(f for f in solid.val().Faces() if f.normalAt().z > 0.999 and abs(f.Center().z - 6) < 1e-9),
            name="pocket",
            source={"test": "deburr"},
        )
        self.assertEqual(
            floor.components[0].outer.vertices.__len__(), pocket_region.components[0].outer.vertices.__len__()
        )
        report = None
        with job(levels, surface, pocket_region).regenerate() as run:
            for event in run:
                update = event.update
                if isinstance(update, OperationUpdate) and update.stage == "end" and update.operation == "deburr":
                    report = update.report
            self.assertTrue(run.finished)
            state = run.states["top"]
        # Material: the stock against the part with its top edges chamfered
        # 0.5 x 45 degrees (the requested break).
        from camlib.cadquery import export_inner_patches
        from camlib.export import material_status
        from camlib.material import Options, SurfaceTarget, Verifier

        broken = solid.faces(">Z").edges().chamfer(0.5)
        target = SurfaceTarget(
            "deburred",
            "1",
            export_surface_mesh(broken, max_error_mm=0.001),
            0,
            inner_patches=export_inner_patches(broken),
        )
        material = Verifier(target, Options(max_findings=4096)).check(state)
        self.assertEqual(material["gouge"]["definite_cells"], 0, material["gouge"])
        # Not a violation; possible-only cells remain where the target's
        # height varies inside a stock cell (the 45 degree breaks, walls).
        self.assertNotEqual(material_status(material), "violation")
        # The same stock against the sharp part is a definite excess: the
        # check sees the breaks.
        sharp = SurfaceTarget(
            "sharp", "1", export_surface_mesh(solid, max_error_mm=0.001), 0, inner_patches=export_inner_patches(solid)
        )
        self.assertEqual(material_status(Verifier(sharp, Options(max_findings=64)).check(state)), "violation")
        self.assertEqual(report["levels"], 2)
        # Every convex edge (the block's top and the pocket's rim) is broken.
        self.assertEqual(report["unreachable_length_mm"], 0)
        # The tool axis runs 0.405 out from each edge (tip radius 0.1 + tip
        # clearance 0.3 + half the boundary tolerance): round the block's
        # corners, and inside the pocket a rectangle 0.405 smaller each side.
        import math

        offset = 0.1 + 0.3 + 0.005
        expected = (2 * (40 + 30) + 2 * math.pi * offset) + (2 * (20 + 10) - 8 * offset)
        self.assertAlmostEqual(report["cut_length_mm"], expected, delta=0.01)
        # The pocket floor's boundary is concave all round: reported, not cut.
        self.assertGreater(report["concave_length_mm"], 2 * (20 + 10) - 1)
        self.assertTrue(all("concave" in s["reason"] for s in report["skipped"]))

    def test_ball_and_bull_nose_breaks_stay_inside_the_filleted_part(self):
        from camlib.cadquery import deburr_levels, export_inner_patches, export_surface_mesh
        from camlib.export import material_status
        from camlib.material import Options, SurfaceTarget, Verifier

        solid = part()
        levels = deburr_levels(solid, source={"test": "deburr"})
        surface = export_surface_mesh(solid, max_error_mm=0.001)
        pocket_region = levels[1].regions[0]
        sharp = SurfaceTarget(
            "sharp", "1", export_surface_mesh(solid, max_error_mm=0.001), 0, inner_patches=export_inner_patches(solid)
        )
        # A ball given the fillet to stay inside; a bull-nose given the leg.
        for tool, size in ((B3, {"break_radius_mm": 0.8}), (R4, {"width_mm": 0.4})):
            with self.subTest(tool=tool.id):
                deburr = Deburr(
                    "deburr",
                    tool,
                    surface,
                    levels,
                    size.get("width_mm"),
                    30,
                    tip_clearance_mm=0.3,
                    break_radius_mm=size.get("break_radius_mm"),
                    feed_mm_min=600,
                    plunge_mm_min=200,
                    spindle_rpm=10000,
                    tool_number=2,
                    length_register=2,
                    coolant="flood",
                )
                report = None
                with job(levels, surface, pocket_region, deburr).regenerate() as run:
                    for event in run:
                        update = event.update
                        if (
                            isinstance(update, OperationUpdate)
                            and update.stage == "end"
                            and update.operation == "deburr"
                        ):
                            report = update.report
                    self.assertTrue(run.finished)
                    state = run.states["top"]
                self.assertEqual(report["break_profile"], "arc")
                self.assertEqual(report["unreachable_length_mm"], 0)
                # The pocket floor's boundary is concave all round: reported.
                self.assertGreater(report["concave_length_mm"], 2 * (20 + 10) - 1)
                self.assertTrue(all("concave" in s["reason"] for s in report["skipped"]))
                if "break_radius_mm" in size:
                    self.assertAlmostEqual(report["fillet_radius_mm"], 0.8, places=12)
                else:
                    self.assertAlmostEqual(report["face_width_mm"], 0.4, places=12)
                    self.assertAlmostEqual(report["wall_depth_mm"], 0.4, places=12)
                # Material: against the part filleted (CadQuery) with the
                # fillet the break stays inside, no definite gouge.
                rho = report["fillet_radius_mm"]
                filleted = solid.faces(">Z").edges().fillet(rho)
                target = SurfaceTarget(
                    "filleted",
                    "1",
                    export_surface_mesh(filleted, max_error_mm=0.001),
                    0,
                    inner_patches=export_inner_patches(filleted),
                )
                material = Verifier(target, Options(max_findings=4096)).check(state)
                self.assertEqual(material["gouge"]["definite_cells"], 0, material["gouge"])
                # Against the sharp part the breaks are a definite violation.
                self.assertEqual(material_status(Verifier(sharp, Options(max_findings=64)).check(state)), "violation")

    def test_needs_a_chamfer_mill(self):
        from camlib.cadquery import deburr_levels, export_surface_mesh

        solid = part()
        op = Deburr(
            "deburr",
            EM6,
            export_surface_mesh(solid),
            deburr_levels(solid)[:1],
            0.5,
            30,
            feed_mm_min=600,
            plunge_mm_min=200,
            spindle_rpm=10000,
            tool_number=1,
            length_register=1,
            coolant="flood",
        )
        setup = Setup(
            "top", StockSpec((0, 0, 0), (40, 30, 10), 0.1), (EM6,), (OperationNode(op, sources=()),), (0, 0, 40)
        )
        with self.assertRaisesRegex(Exception, "chamfer mill"):
            with Job("d", "1", (setup,)).regenerate() as run:
                list(run)


if __name__ == "__main__":
    unittest.main()
