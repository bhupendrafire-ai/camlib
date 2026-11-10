"""Deburr (#11, #92, #93, #94): the top and accessible side edges of a part
are broken to the requested size with a chamfer, ball-end or bull-nose mill,
next to vertical or drafted walls, and sloped edges with a ball; every convex
edge is accounted for; concave, already broken (filleted) and unreachable
stretches are reported, never silently dropped."""

import math
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


def _chamfer_top(solid, top_leg, wall_leg):
    """The solid with its top face's edges chamfered top_leg on the top face
    and wall_leg on the walls (CadQuery's two lengths go to the faces in an
    order of its own: the one that leaves the top face as asked is kept)."""
    for first, second in ((top_leg, wall_leg), (wall_leg, top_leg)):
        chamfered = solid.faces(">Z").edges().chamfer(first, second)
        top = chamfered.faces(">Z").val()
        expected = solid.faces(">Z").val().Area()
        box = solid.faces(">Z").val().BoundingBox()
        # Each side moves in by top_leg.
        width, depth = box.xlen - 2 * top_leg, box.ylen - 2 * top_leg
        if abs(top.Area() - width * depth) < 1e-6 * expected:
            return chamfered
    raise AssertionError("CadQuery's chamfer did not leave the requested top leg")


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

    def _deburr_only(self, solid, width_mm=0.5, tool=C90):
        """Runs Deburr alone on a block of stock around ``solid`` (the stock
        outside the part is not machined first: only gouges are checked)."""
        from camlib.cadquery import deburr_levels, export_surface_mesh

        levels = deburr_levels(solid, source={"test": "deburr"})
        surface = export_surface_mesh(solid, max_error_mm=0.001)
        box = solid.val().BoundingBox()
        op = Deburr(
            "deburr",
            tool,
            surface,
            levels,
            width_mm,
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
            StockSpec((box.xmin - 4, box.ymin - 4, box.zmin), (box.xmax + 4, box.ymax + 4, box.zmax), 0.1),
            (tool,),
            (OperationNode(op, sources=()),),
            (box.xmin - 4, box.ymin - 4, 40),
        )
        report = None
        with Job("deburr", "1", (setup,)).regenerate() as run:
            for event in run:
                update = event.update
                if isinstance(update, OperationUpdate) and update.stage == "end" and update.operation == "deburr":
                    report = update.report
            self.assertTrue(run.finished)
            state = run.states["top"]
        return report, state

    def test_drafted_block_is_broken_all_round_to_the_requested_width(self):
        from camlib.cadquery import export_inner_patches, export_surface_mesh
        from camlib.material import Options, SurfaceTarget, Verifier

        draft = 5
        solid = cq.Workplane("XY").rect(30, 20).extrude(10, taper=draft)
        report, state = self._deburr_only(solid)
        self.assertEqual(report["skipped"], [])
        self.assertEqual(report["unreachable_length_mm"], 0)
        (level,) = report["level_details"]
        self.assertAlmostEqual(level["max_wall_draft_deg"], draft, places=6)
        # The cone (45 degrees) ends the break on the drafted wall at depth
        # h = w / (tan 45 - tan 5); its lower edge keeps the tip clearance:
        # the axis is at least h tan 5 further out than for a vertical wall,
        # and at most w k / (1 - k), k = sqrt 2 tan 5, where the drafted
        # corner edges lean out sqrt 2 times as far.
        t = math.tan(math.radians(draft))
        h, k = 0.5 / (1 - t), math.sqrt(2) * t
        x = level["axis_offset_mm"]
        self.assertTrue(0.4 + h * t - 1e-9 <= x <= 0.4 + 0.5 * k / (1 - k) + 1e-9, level)
        # The face width is exact: the cone's radius at the face is x + 0.5.
        self.assertAlmostEqual(level["tip_z_mm"], 10 - (x + 0.5 - 0.1), places=9)
        # Material: against the part chamfered by CadQuery with those legs
        # (0.5 on the top face, h / cos(draft) along the drafted wall).
        chamfered = _chamfer_top(solid, 0.5, h / math.cos(math.radians(draft)))
        target = SurfaceTarget(
            "chamfered",
            "1",
            export_surface_mesh(chamfered, max_error_mm=0.001),
            0,
            inner_patches=export_inner_patches(chamfered),
        )
        material = Verifier(target, Options(max_findings=4096)).check(state)
        self.assertEqual(material["gouge"]["definite_cells"], 0, material["gouge"])

    def test_draft_beyond_the_half_angle_is_reported(self):
        solid = cq.Workplane("XY").rect(30, 20).extrude(2, taper=50)
        report, _ = self._deburr_only(solid)
        self.assertEqual(report["runs"], 0)
        self.assertGreater(report["unreachable_length_mm"], 2 * (30 + 20) - 2 * 4 * 2 * math.tan(math.radians(50)))
        self.assertTrue(all("drafted too steeply" in s["reason"] for s in report["skipped"]), report["skipped"])

    def test_filleted_edges_are_reported_as_already_broken(self):
        # A block with its two long top edges filleted: those count as broken.
        solid = cq.Workplane("XY").box(30, 20, 10, centered=False).edges("|X and >Z").fillet(1)
        report, _ = self._deburr_only(solid)
        broken = [s for s in report["skipped"] if "already broken" in s["reason"]]
        # Both long edges (30 each) and the path's corner arcs around them.
        self.assertGreater(report["already_broken_length_mm"], 2 * 30)
        self.assertTrue(broken)
        # The sharp short edges are cut; only where the cone would reach the
        # fillet's end (a curved edge, #94) next to each corner are they not.
        self.assertGreater(report["cut_length_mm"], 2 * 18 - 4 * 2)
        self.assertLess(report["unreachable_length_mm"], 4 * 2)

    def test_every_convex_edge_is_accounted_for_and_sloped_edges_are_broken(self):
        from camlib.cadquery import deburr_edges, deburr_levels, export_inner_patches, export_surface_mesh
        from camlib.export import material_status
        from camlib.material import Options, SurfaceTarget, Verifier

        # A block 40 x 20: flat top Z10 for x in 20..40, rising on a slope
        # to Z16 at x = 0 (a concave edge at x = 20).
        solid = cq.Workplane("XZ").polyline([(0, 0), (40, 0), (40, 10), (20, 10), (0, 16)]).close().extrude(-20)
        edges = deburr_edges(solid)
        # Bottom 4, the flat top's 3 and the slope's top edge: horizontal;
        # the slope's 2 sides: sloped; the 4 corners: vertical. The valley
        # at x = 20 is concave: not listed.
        self.assertEqual(sorted(e.kind for e in edges), ["horizontal"] * 8 + ["sloped"] * 2 + ["vertical"] * 4)
        levels = deburr_levels(solid)
        surface = export_surface_mesh(solid, max_error_mm=0.001)
        # The stock is not machined to the part first: flutes long enough to
        # reach the lowest edge through it.
        ball = ToolAssembly(
            "b3l",
            BallEndMill("b3l", diameter=3, flute_length=18, overall_length=50),
            20,
            60,
            (HolderSection(20, 60, 8, 16),),
        )
        deburr = Deburr(
            "deburr",
            ball,
            surface,
            levels,
            None,
            30,
            tip_clearance_mm=0.3,
            break_radius_mm=0.8,
            edges=edges,
            feed_mm_min=600,
            plunge_mm_min=200,
            spindle_rpm=10000,
            tool_number=2,
            length_register=2,
            coolant="flood",
        )
        box = solid.val().BoundingBox()
        setup = Setup(
            "top",
            StockSpec((box.xmin - 4, box.ymin - 4, box.zmin), (box.xmax + 4, box.ymax + 4, box.zmax), 0.1),
            (ball,),
            (OperationNode(deburr, sources=()),),
            (box.xmin - 4, box.ymin - 4, 40),
        )
        report = None
        with Job("deburr", "1", (setup,)).regenerate() as run:
            for event in run:
                update = event.update
                if isinstance(update, OperationUpdate) and update.stage == "end" and update.operation == "deburr":
                    report = update.report
            self.assertTrue(run.finished)
            state = run.states["top"]
        # Every listed edge is accounted for: cut, or not with a reason.
        by_id = {e["id"]: e for e in report["edges"]}
        self.assertEqual(sorted(by_id), sorted(e.id for e in edges))
        for edge in edges:
            entry = by_id[edge.id]
            self.assertEqual(entry["kind"], edge.kind)
            if entry["status"] != "cut":
                self.assertTrue(entry["reasons"], entry)
            if edge.kind == "vertical":
                self.assertEqual(entry["status"], "skipped")
                self.assertIn("vertical edge", entry["reasons"][0])
            elif max(p[2] for p in edge.points) < 1e-9:
                # The bottom edges: a face next to them faces down.
                self.assertEqual(entry["status"], "skipped")
                self.assertIn("faces down", entry["reasons"][0])
            else:
                # The top edges, sloped or horizontal, are broken (their
                # ends, where they meet other edges, may stay unproven).
                self.assertIn(entry["status"], ("cut", "partly cut"), entry)
                self.assertGreater(entry["cut_length_mm"], 0.5 * entry["length_mm"], entry)
        sloped = [by_id[e.id] for e in edges if e.kind == "sloped"]
        self.assertTrue(all(e["cut_length_mm"] > 0.75 * e["length_mm"] for e in sloped), sloped)
        # Material: against the part with the broken edges filleted 0.8 by
        # CadQuery (the fillet the breaks stay inside), no definite gouge;
        # against the sharp part, a violation.
        cut_ids = {e.id for e in edges if by_id[e.id]["cut_length_mm"] > 0}
        cut_edges = [edge for index, edge in enumerate(solid.val().Edges()) if f"edge-{index}" in cut_ids]
        filleted = cq.Workplane("XY").add(solid.val().fillet(0.8, cut_edges))
        target = SurfaceTarget(
            "filleted",
            "1",
            export_surface_mesh(filleted, max_error_mm=0.001),
            0,
            inner_patches=export_inner_patches(filleted),
        )
        material = Verifier(target, Options(max_findings=4096)).check(state)
        self.assertEqual(material["gouge"]["definite_cells"], 0, material["gouge"])
        sharp = SurfaceTarget(
            "sharp", "1", export_surface_mesh(solid, max_error_mm=0.001), 0, inner_patches=export_inner_patches(solid)
        )
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
