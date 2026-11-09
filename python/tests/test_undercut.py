"""Undercut milling (issue #8): T-slot, keyseat, lollipop and dovetail
cutters on tri-dexel stock, checked against the analytic part."""

import importlib.util
import math
from pathlib import Path
import unittest

import camlib
from camlib.simulation import SimulationError
from camlib.surface import SurfaceMesh

ROOT = Path(__file__).resolve().parents[2]
CUT = dict(feed_mm_min=300, plunge_mm_min=100, spindle_rpm=6000, coolant="flood")


def example():
    spec = importlib.util.spec_from_file_location("standalone_undercut", ROOT / "examples/standalone_job.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def extrusion(profile, x0, x1):
    """A closed mesh of the simple counter-clockwise polygon ``profile`` (Y, Z)
    extruded along X."""

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    idx, caps = list(range(len(profile))), []
    while len(idx) > 3:
        n = len(idx)
        for i in range(n):
            a, b, c = (profile[idx[(i - 1) % n]], profile[idx[i]], profile[idx[(i + 1) % n]])
            if cross(a, b, c) > 0 and not any(
                p not in (a, b, c) and cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0
                for p in (profile[j] for j in idx)
            ):
                caps.append((idx[(i - 1) % n], idx[i], idx[(i + 1) % n]))
                del idx[i]
                break
    caps.append(tuple(idx))

    def at(x, p):
        return (float(x), float(p[0]), float(p[1]))

    out = []
    for a, b, c in caps:
        out.append((at(x0, profile[a]), at(x0, profile[c]), at(x0, profile[b])))
        out.append((at(x1, profile[a]), at(x1, profile[b]), at(x1, profile[c])))
    for i, p in enumerate(profile):
        q = profile[(i + 1) % len(profile)]
        out += [(at(x0, p), at(x0, q), at(x1, q)), (at(x0, p), at(x1, q), at(x1, p))]
    return out


def area(profile):
    return abs(sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(profile, profile[1:] + profile[:1]))) / 2


def holder(cutter, stickout=30):
    return camlib.ToolAssembly(
        cutter.id + "-holder", cutter, stickout, stickout + 30, (camlib.HolderSection(stickout, stickout + 30, 9, 9),)
    )


def slot_mill(diameter):
    return holder(camlib.EndMill(f"em{diameter:g}", diameter=diameter, flute_length=15, overall_length=50))


def slot(tool, width_center_y, depth_z):
    """A slot through the 30 mm block along X, cut by the end mill's centre line."""
    return camlib.Trace(
        "slot",
        tool,
        (camlib.TracePath((-6, width_center_y), (camlib.TraceLine((36, width_center_y)),)),),
        top_z_mm=10,
        depth_z_mm=depth_z,
        stepdown_mm=10 - depth_z,
        clearance_z_mm=20,
        tool_number=1,
        length_register=1,
        **CUT,
    )


def undercut(tool, tip_z, **values):
    return camlib.Undercut(
        "undercut",
        tool,
        (camlib.TracePath((-8, 10), (camlib.TraceLine((38, 10)),)),),
        **{
            "tip_z_mm": tip_z,
            "entry_mm": (-8, 10),
            "exit_mm": (38, 10),
            "clearance_z_mm": 20,
            "tool_number": 2,
            "length_register": 2,
            **CUT,
            **values,
        },
    )


def job(operations, tools, stock=None):
    stock = stock or camlib.DexelStockSpec((0, 0, 0), (30, 20, 10), cell_size_mm=0.1)
    setup = camlib.Setup(
        "top",
        stock=stock,
        tools=tools,
        operations=tuple(camlib.OperationNode(op, sources=()) for op in operations),
        initial_position_mm=(-10, 10, 20),
    )
    return camlib.Job("undercut", "1", (setup,))


def machine(j):
    header = j.compile().setup("top").program_header()
    return camlib.Machine(
        "Undercut test mill",
        tuple(
            camlib.ToolSetting.for_tool(t, number=n, length_register=n, position_after_mm=header.initial_position_mm)
            for n, t in enumerate(header.tools, 1)
        ),
        tool_change_position_mode="preserve_tip",
        **example().MACHINE,
    )


def run(j):
    with j.regenerate(None) as r:
        for _ in r:
            pass
        return r.states["top"]


# The T-slotted block (Y, Z): a 4 mm slot 7 mm deep, 2 mm wings 10 mm wide.
T_PROFILE = [(0, 0), (20, 0), (20, 10), (12, 10), (12, 5), (15, 5), (15, 3), (5, 3), (5, 5), (8, 5), (8, 10), (0, 10)]
# The dovetail slot (Y, Z): 8 mm wide, 5 mm deep, its floor 10 mm wide, the
# flanks at 60 degrees from the floor.
RISE = math.tan(math.radians(60))
DOVETAIL_PROFILE = [
    (0, 0),
    (20, 0),
    (20, 10),
    (14, 10),
    (14, 5 + RISE),
    (15, 5),
    (5, 5),
    (6, 5 + RISE),
    (6, 10),
    (0, 10),
]


class UndercutTest(unittest.TestCase):
    def check(self, state, profile):
        # Volume bounds enclose the analytic part; the material check against
        # it is within tolerance (on 0.1 mm cells, 0.15 mm: see compare_solid).
        exact = area(profile) * 30
        v = state.volume()
        self.assertLessEqual(v["remaining_lower_mm3"], exact + 1e-6)
        self.assertGreaterEqual(v["remaining_upper_mm3"], exact - 1e-6)
        self.assertLess(v["remaining_upper_mm3"] - v["remaining_lower_mm3"], 0.02 * exact)
        report = state.compare_solid(SurfaceMesh(extrusion(profile, 0, 30)), 0.15)
        self.assertEqual(report["gouge"]["assessment"], "within_tolerance", report["gouge"])
        self.assertEqual(report["residual"]["assessment"], "within_tolerance", report["residual"])

    def test_t_slot_under_a_slot_leaves_the_t_profile(self):
        mill = slot_mill(4)
        cutter = holder(
            camlib.TSlotCutter(
                "tslot", diameter=10, width=2, neck_diameter=3, neck_length=10, overall_length=50, shank_diameter=6
            )
        )
        j = job((slot(mill, 10, 3), undercut(cutter, 3)), (mill, cutter))
        state = run(j)
        self.check(state, T_PROFILE)
        # The program posts and decodes back: lines at the T-slot depth.
        program = j.post_nc("top", machine=machine(j), accept={"unchecked_machine"})
        self.assertIn("Z3", program.text)

    def test_a_neck_that_would_hit_the_slot_walls_is_refused(self):
        mill = slot_mill(4)
        cutter = holder(
            camlib.TSlotCutter(
                "tslot", diameter=10, width=2, neck_diameter=4.2, neck_length=10, overall_length=50, shank_diameter=6
            )
        )
        with self.assertRaises(SimulationError) as raised:
            run(job((slot(mill, 10, 3), undercut(cutter, 3)), (mill, cutter)))
        self.assertIn("neck, shank or holder", str(raised.exception))

    def test_dovetail_on_a_pocket_wall_passes_the_material_check(self):
        mill = slot_mill(8)
        cutter = holder(
            camlib.DovetailCutter(
                "dovetail",
                diameter=10,
                angle=60,
                cutting_length=2,
                neck_diameter=6,
                neck_length=10,
                overall_length=50,
                shank_diameter=6,
            )
        )
        state = run(job((slot(mill, 10, 5), undercut(cutter, 5)), (mill, cutter)))
        self.check(state, DOVETAIL_PROFILE)

    def test_the_entry_must_be_open(self):
        mill = slot_mill(4)
        cutter = holder(
            camlib.TSlotCutter(
                "tslot", diameter=10, width=2, neck_diameter=3, neck_length=10, overall_length=50, shank_diameter=6
            )
        )
        blocked = undercut(cutter, 3, entry_mm=(15, 4))
        with self.assertRaises(SimulationError) as raised:
            run(job((slot(mill, 10, 3), blocked), (mill, cutter)))
        self.assertIn("not proven open", str(raised.exception))

    def test_height_field_stock_is_refused(self):
        mill = slot_mill(4)
        cutter = holder(
            camlib.LollipopCutter(
                "lolli", diameter=6, neck_diameter=3, neck_length=10, overall_length=50, shank_diameter=6
            )
        )
        stock = camlib.StockSpec((0, 0, 0), (30, 20, 10), 0.1)
        with self.assertRaises((SimulationError, ValueError)) as raised:
            run(job((slot(mill, 10, 3), undercut(cutter, 3)), (mill, cutter), stock))
        self.assertIn("tri-dexel", str(raised.exception))

    def test_cutter_shapes(self):
        lolli = camlib.LollipopCutter(
            "lolli", diameter=6, neck_diameter=3, neck_length=10, overall_length=50, shank_diameter=6
        )
        self.assertAlmostEqual(lolli.cutting_length_mm, 3 + math.sqrt(9 - 2.25))
        with self.assertRaises((SimulationError, ValueError)):
            # A dovetail flank ending narrower than its neck is refused natively.
            cutter = camlib.DovetailCutter(
                "bad",
                diameter=10,
                angle=45,
                cutting_length=4,
                neck_diameter=6,
                neck_length=10,
                overall_length=50,
                shank_diameter=6,
            )
            run(job((undercut(holder(cutter), 3),), (holder(cutter),)))


if __name__ == "__main__":
    unittest.main()
