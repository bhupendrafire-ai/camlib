"""Stock given as a closed mesh, and finished stock returned as one (#121).

Expected stock is worked out here by hand (box faces, an extruded step), not
by the engine's conversion; containment is checked by ray casting in this file.
"""

from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import random
from tempfile import TemporaryDirectory
import unittest

from camlib.operations import RestPocket
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.simulation import OperationUpdate, Simulation, StockMesh, StockSpec
from camlib.tools import EndMill, HolderSection, ToolAssembly

ROOT = Path(__file__).resolve().parents[2]


def box_triangles(a, b):
    def p(i):
        return (b[0] if i & 1 else a[0], b[1] if i & 2 else a[1], b[2] if i & 4 else a[2])

    faces = ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5))
    return [t for f in faces for t in ((p(f[0]), p(f[1]), p(f[2])), (p(f[0]), p(f[2]), p(f[3])))]


def extrude(profile, caps, y0, y1):
    """A profile in XZ (counter-clockwise, x right, z up) extruded along Y."""

    def at(k, y):
        return (profile[k][0], y, profile[k][1])

    triangles = []
    for a, b, c in caps:
        triangles += [(at(a, y0), at(b, y0), at(c, y0)), (at(a, y1), at(c, y1), at(b, y1))]
    for k in range(len(profile)):
        n = (k + 1) % len(profile)
        triangles += [(at(k, y0), at(k, y1), at(n, y1)), (at(k, y0), at(n, y1), at(n, y0))]
    return triangles


def stepped_blank():
    """24 x 16 mm: 3 mm high for x < 12, 6 mm high for x > 12."""
    profile = ((0, 0), (12, 0), (24, 0), (24, 6), (12, 6), (12, 3), (0, 3))
    return extrude(profile, ((0, 1, 5), (0, 5, 6), (2, 3, 4), (2, 4, 5), (2, 5, 1)), 0, 16)


def inside(triangles, p):
    d = (0.5773502691896258, 0.6123724356957945, 0.5400617248673217)

    def sub(a, b):
        return tuple(x - y for x, y in zip(a, b))

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    def dot(a, b):
        return sum(x * y for x, y in zip(a, b))

    hits = 0
    for t in triangles:
        e1, e2 = sub(t[1], t[0]), sub(t[2], t[0])
        h = cross(d, e2)
        det = dot(e1, h)
        if abs(det) < 1e-14:
            continue
        s = sub(p, t[0])
        u = dot(s, h) / det
        q = cross(s, e1)
        v = dot(d, q) / det
        if u >= 0 and v >= 0 and u + v <= 1 and dot(e2, q) / det > 0:
            hits += 1
    return hits % 2 == 1


def tool(diameter=4):
    return ToolAssembly(
        f"mill{diameter}",
        EndMill(f"em{diameter}", diameter=diameter, flute_length=20, overall_length=50),
        30,
        60,
        (HolderSection(30, 60, 4, 4),),
    )


def rest_pocket(t):
    region = PlanarRegion("top", (Component(Boundary.polygon(((2, 2), (22, 2), (22, 14), (2, 14)))),))
    return RestPocket(
        "rest",
        t,
        region,
        6,
        2,
        1,
        1.5,
        8,
        feed_mm_min=600,
        plunge_mm_min=150,
        spindle_rpm=12000,
        coolant="flood",
        tool_number=1,
        length_register=1,
    )


def run(stock, operations=None):
    t = tool()
    with Simulation(
        stock,
        (t,),
        operations=operations or (rest_pocket(t),),
        revision="mesh-stock-1",
        initial_position_mm=(0, 0, 8),
    ) as sim:
        initial = bytes(sim.state.heights.tobytes())
        motions, reports = [], []
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate):
                if update.stage == "end":
                    reports.append(update.report)
            else:
                motions.append(update.motion)
        return dict(initial=initial, motions=motions, reports=reports, state=sim.state.clone())


class MeshStockTest(unittest.TestCase):
    def test_box_mesh_runs_exactly_like_the_box(self):
        box = StockSpec((0, 0, 0), (24, 16, 6), 0.3)
        mesh = StockSpec.from_mesh(StockMesh.from_triangles(box_triangles((0, 0, 0), (24, 16, 6))), 0.3)
        self.assertEqual((mesh.min_mm, mesh.max_mm), ((0, 0, 0), (24, 16, 6)))
        a, b = run(box), run(mesh)
        self.assertEqual(a["initial"], b["initial"])
        self.assertEqual(a["motions"], b["motions"])
        self.assertEqual(a["reports"], b["reports"])
        self.assertEqual(a["state"].heights.tobytes(), b["state"].heights.tobytes())

    def test_a_stepped_blank_starts_stepped_and_is_not_cut_in_the_air(self):
        blank = StockMesh.from_triangles(stepped_blank())
        self.assertAlmostEqual(blank.volume_mm3, 16 * (12 * 3 + 12 * 6))
        spec = StockSpec.from_mesh(blank, 0.5)
        stepped, block = run(spec), run(replace(spec, mesh=None))
        heights = memoryview(stepped["initial"]).cast("d")
        nx = 48
        for c in range(len(heights) // 2):
            z = 3 if c % nx < 24 else 6
            self.assertEqual((heights[2 * c], heights[2 * c + 1]), (z, z))
        # Rest machining reads the stock: strokes over the low step above
        # z 3 remove nothing, so they are skipped.
        skipped = stepped["reports"][0]["rest_skipped_strokes"]
        self.assertGreater(skipped, block["reports"][0]["rest_skipped_strokes"])
        self.assertLess(len(stepped["motions"]), len(block["motions"]))
        cuts_over_low_step = [
            m
            for m in stepped["motions"]
            if m["kind"] == "cut"
            and max(m["curve"]["start"][0], m["curve"]["end"][0]) + 2 < 12
            and min(m["curve"]["start"][2], m["curve"]["end"][2]) > 3
        ]
        self.assertEqual(cuts_over_low_step, [])
        # The same pocket inside the region (the blank only lacked air), and
        # never more material anywhere.
        a, b = stepped["state"].heights, block["state"].heights
        for c in range(len(a) // 2):
            x, y = (c % nx + 0.5) * 0.5, (c // nx + 0.5) * 0.5
            self.assertLessEqual(a[2 * c + 1], b[2 * c + 1])
            if 4 < x < 20 and 4 < y < 12:
                self.assertEqual((a[2 * c], a[2 * c + 1]), (b[2 * c], b[2 * c + 1]))

    def test_finished_stock_mesh_holds_the_material_and_starts_the_next_setup(self):
        first = run(StockSpec((0, 0, 0), (24, 16, 6), 0.25))["state"]
        mesh = first.stock_mesh()
        self.assertIn("approximate", mesh.approximation)
        self.assertAlmostEqual(mesh.volume_mm3, first.volume()["remaining_upper_mm3"], places=6)
        self.assertEqual(mesh.bounds, ((0, 0, 0), (24, 16, 6)))
        # Every point of the stock model's material is inside the mesh.
        triangles = list(mesh.triangles())
        heights = first.heights
        rng = random.Random(4)
        for _ in range(3000):
            x, y, z = rng.uniform(0, 24), rng.uniform(0, 16), rng.uniform(0, 6)
            i, j = int(x / first.dx), int(y / first.dy)
            if min(x / first.dx - i, y / first.dy - j) < 1e-6:
                continue
            if z < heights[2 * (j * first.nx + i) + 1] - 1e-9:
                self.assertTrue(inside(triangles, (x, y, z)), (x, y, z))
        # Heat treat, then finish in the same frame: the same stock back.
        again = run(StockSpec.from_mesh(mesh, 0.25))
        initial = memoryview(again["initial"]).cast("d")
        self.assertEqual(list(initial[1::2]), list(heights[1::2]))
        self.assertEqual(list(initial[::2]), list(heights[1::2]))
        # Turn it over for the second setup: the pocket now opens downward,
        # so its columns are possible material up to the new top, never lost.
        turned = mesh.transformed(((1, 0, 0), (0, -1, 0), (0, 0, -1)), (0, 16, 6))
        self.assertIsNotNone(turned.approximation)
        second = run(StockSpec.from_mesh(turned, 0.25))
        initial = memoryview(second["initial"]).cast("d")
        real = first.volume()["remaining_lower_mm3"]
        certain = sum(initial[::2]) * 0.25 * 0.25
        possible = sum(initial[1::2]) * 0.25 * 0.25
        self.assertAlmostEqual(possible, 24 * 16 * 6, places=6)
        self.assertLess(certain, real)

    def test_bad_meshes_and_moves_are_refused(self):
        triangles = box_triangles((0, 0, 0), (10, 10, 10))
        with self.assertRaisesRegex(ValueError, "not closed"):
            StockMesh.from_triangles(triangles[:-1])
        with self.assertRaisesRegex(ValueError, "inside out"):
            StockMesh.from_triangles([(a, c, b) for a, b, c in triangles])
        mesh = StockMesh.from_triangles(triangles)
        self.assertIsNone(mesh.approximation)
        with self.assertRaisesRegex(ValueError, "outside the stock box"):
            StockSpec((0, 0, 0), (10, 10, 9), 1, mesh=mesh)
        with self.assertRaisesRegex(ValueError, "mirror"):
            mesh.transformed(((1, 0, 0), (0, 1, 0), (0, 0, -1)))
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            mesh.transformed(((2, 0, 0), (0, 1, 0), (0, 0, 1)))
        turned = mesh.transformed(((0, -1, 0), (1, 0, 0), (0, 0, 1)), (5, 0, 0))
        self.assertEqual(turned.bounds, ((-5, 0, 0), (5, 10, 10)))
        self.assertAlmostEqual(turned.volume_mm3, 1000)
        # Indexed form: the same solid.
        indexed = StockMesh(
            [mesh.vertices[k : k + 3] for k in range(0, len(mesh.vertices), 3)],
            [mesh.indices[k : k + 3] for k in range(0, len(mesh.indices), 3)],
        )
        self.assertEqual(indexed.to_dict(), mesh.to_dict())
        with self.assertRaisesRegex(ValueError, "vertex index|does not exist"):
            StockMesh([(0, 0, 0)], [(0, 1, 2)])

    def test_material_checks_and_posting_use_the_mesh_stock(self):
        spec = importlib.util.spec_from_file_location("mesh_stock_example", ROOT / "examples/cadquery_job.py")
        example = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(example)
        job = example.build_job()
        setup = job.setups[0]
        # A pre-machined blank: a 0.5 mm step along one side, in the job's box.
        profile = ((0, 0), (2, 0), (24, 0), (24, 6), (2, 6), (2, 5.5), (0, 5.5))
        blank = StockMesh.from_triangles(
            extrude(profile, ((0, 1, 5), (0, 5, 6), (2, 3, 4), (2, 4, 5), (2, 5, 1)), 0, 16)
        )
        meshed = replace(job, setups=(replace(setup, stock=StockSpec.from_mesh(blank, 0.5)),))
        with TemporaryDirectory() as directory:
            with meshed.post("top", machine=example.machine_for(meshed)) as posted:
                posted.wait()
                exported = posted.export(Path(directory) / "blank.nc", accept={"unchecked_machine"})
                self.assertTrue(Path(exported["path"]).is_file())
        from camlib.material import Target, Verifier

        boxed = run(StockSpec((0, 0, 0), (24, 16, 6), 0.5))["state"]
        meshed_box = run(StockSpec.from_mesh(StockMesh.from_triangles(box_triangles((0, 0, 0), (24, 16, 6))), 0.5))
        floor = PlanarRegion("all", (Component(Boundary.polygon(((0, 0), (24, 0), (24, 16), (0, 16)))),))
        verifier = Verifier(Target("part", "1", floor, 2))
        a, b = verifier.check(meshed_box["state"]), verifier.check(boxed)
        self.assertEqual(a["context"]["stock"], b["context"]["stock"])
        self.assertEqual(meshed_box["state"].heights.tobytes(), boxed.heights.tobytes())
        for key in ("residual", "gouge", "findings"):
            self.assertEqual(a[key], b[key], key)

    def test_cadquery_planar_blank_and_curved_refusal(self):
        try:
            import cadquery as cq
        except ImportError:
            self.skipTest("CadQuery is not installed")
        from camlib.cadquery import export_stock_mesh

        solid = (
            cq.Workplane("XY")
            .box(24, 16, 6, centered=False)
            .cut(cq.Workplane("XY").box(12, 16, 3, centered=False).translate((0, 0, 3)))
        )
        mesh = export_stock_mesh(solid)
        self.assertAlmostEqual(mesh.volume_mm3, solid.val().Volume(), places=6)
        hand = run(StockSpec.from_mesh(StockMesh.from_triangles(stepped_blank()), 0.5))["initial"]
        self.assertEqual(run(StockSpec.from_mesh(mesh, 0.5))["initial"], hand)
        with self.assertRaisesRegex(ValueError, "planar faces"):
            export_stock_mesh(cq.Workplane("XY").cylinder(5, 3))
        self.assertTrue(math.isfinite(mesh.volume_mm3))


if __name__ == "__main__":
    unittest.main()
