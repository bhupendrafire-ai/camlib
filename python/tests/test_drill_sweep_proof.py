"""Pointed tools must prove nominal holes without spending extra tolerance (#324)."""

import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib import Drilling, Job, OperationNode, Setup, StockSpec, StreamChecks
from camlib.export import ExportRefused
from camlib.material import Options, SurfaceTarget
from camlib.surface import SurfaceMesh
from camlib.tools import Drill, HolderSection, ToolAssembly
from camlib.verification import MaterialCheck
from test_drill_cycles import linuxcnc


def annulus():
    """True outer polygon, nominal circular R2.1 hole; chord error < .013."""
    count, radius = 32, 2.1
    assert radius * (1 - math.cos(math.pi / count)) < 0.013
    triangles = []

    def point(r, i, z):
        a = 2 * math.pi * i / count
        return r * math.cos(a), r * math.sin(a), z

    def quad(a, b, c, d):
        triangles.extend(((a, b, c), (a, c, d)))

    for i in range(count):
        j = (i + 1) % count
        quad(point(10, i, 4), point(10, j, 4), point(radius, j, 4), point(radius, i, 4))
        quad(point(10, i, 0), point(radius, i, 0), point(radius, j, 0), point(10, j, 0))
        quad(point(10, i, 0), point(10, j, 0), point(10, j, 4), point(10, i, 4))
        quad(point(radius, i, 0), point(radius, i, 4), point(radius, j, 4), point(radius, j, 0))
    return triangles


def job(offset=0.0, error=0.013):
    tool = ToolAssembly(
        "drill",
        Drill("drill", diameter=4.2, flute_length=16, overall_length=60, point_angle=118),
        30,
        80,
        (HolderSection(30, 80, 20, 20),),
    )
    op = Drilling(
        "hole",
        tool,
        ((offset, 0.0),),
        -0.5,
        12.0,
        depth_reference="full_diameter",
        feed_mm_min=150,
        retract_feed_mm_min=600,
        spindle_rpm=6000,
        tool_number=1,
        length_register=1,
        coolant="flood",
    )
    target = SurfaceTarget("annulus", "1", SurfaceMesh(annulus(), error), -2.0)
    verification = StreamChecks(material=(MaterialCheck(target, options=Options(gouge_tolerance_mm=0.05)),))
    setup = Setup(
        "top",
        stock=StockSpec((-12, -12, -2), (12, 12, 4), 0.25),
        tools=(tool,),
        operations=(OperationNode(op, sources=()),),
        initial_position_mm=(0, 0, 12),
        verification=verification,
    )
    return Job("pointed-proof", "1", (setup,))


class DrillSweepProofTest(unittest.TestCase):
    def test_nominal_mesh_hole_releases_without_accepting_uncertain_and_default_gate_stays_closed(self):
        source = job()
        with source.post("top", machine=linuxcnc(source)) as posted:
            posted.wait()
            with TemporaryDirectory(prefix="drill sweep proof ") as folder:
                path = Path(folder) / "hole.nc"
                with self.assertRaisesRegex(ExportRefused, "machine"):
                    posted.export(path)
                self.assertFalse(path.exists())
                posted.export(path, accept=("unchecked_machine",))
                self.assertTrue(path.is_file())

    def test_real_wall_overcut_and_insufficient_source_budget_still_refuse(self):
        for source in (job(offset=0.2), job(error=0.05)):
            with self.subTest(source=source.id), source.post("top", machine=linuxcnc(source)) as posted:
                posted.wait()
                with TemporaryDirectory(prefix="refused pointed sweep ") as folder:
                    path = Path(folder) / "unsafe.nc"
                    with self.assertRaises(ExportRefused):
                        posted.export(path, accept=("unchecked_machine",))
                    self.assertFalse(path.exists())
