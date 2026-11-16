"""Actual stock/motion and independently decoded posting for explicit preparation."""

from dataclasses import replace
import math
from tempfile import TemporaryDirectory
import unittest

import camlib
from camlib import DrillingPreparation, DrillingSequence
from camlib.export import ExportRefused
from camlib.simulation import OperationUpdate
from test_drilling_options import fixture, run
from test_drill_cycles import linuxcnc


def job(*, declared=True, canned=False, diameter=2.0):
    source = fixture(canned_cycle=canned, peck_mm=None)
    setup = source.setups[0]
    final = setup.operations[0].operation
    final = replace(final, tool_number=3, length_register=3)
    base = setup.tools[0]

    def tool(name, dia, angle):
        return replace(
            base,
            id=name,
            cutter=camlib.Drill(name + "-point", diameter=dia, flute_length=30, overall_length=60, point_angle=angle),
        )

    spot_tool, pilot_tool = tool("spot6", 6, 150), tool("pilot2", 2, 140)
    spot = replace(
        final,
        id="spot",
        tool=spot_tool,
        depth_z_mm=10 - diameter / (2 * math.tan(math.radians(75))),
        tool_number=1,
        length_register=1,
    )
    pilot = replace(
        final,
        id="pilot",
        tool=pilot_tool,
        depth_z_mm=8.5,
        depth_reference="full_diameter",
        tool_number=2,
        length_register=2,
    )
    sequence = DrillingSequence(final, spot=spot, pilot=pilot, spot_diameter_mm=diameter)
    operations = sequence.operations() if declared else (spot, pilot, final)
    return replace(
        source,
        setups=(
            replace(
                setup,
                tools=(spot_tool, pilot_tool, base),
                operations=tuple(camlib.OperationNode(op, sources=()) for op in operations),
            ),
        ),
    )


def operations(source):
    return [entry.operation for entry in source.setups[0].operations]


def with_operations(source, ops):
    return replace(
        source,
        setups=(replace(source.setups[0], operations=tuple(camlib.OperationNode(op, sources=()) for op in ops)),),
    )


class PreparationTest(unittest.TestCase):
    def test_sequence_runs_the_same_moves_and_stock_as_explicit_stages(self):
        for canned in (False, True):
            plain = run(job(declared=False, canned=canned))
            prepared = run(job(canned=canned))
            self.assertEqual(plain[:2], prepared[:2])
            report = prepared[2]["preparation"]
            self.assertEqual(report["holes"], 2)
            self.assertAlmostEqual(report["spot"]["diameter_mm"], 2.0)
            self.assertEqual(report["spot"]["chamfer_radial_width_mm"], 0.0)
            self.assertEqual(report["pilot"]["full_diameter_depth_mm"], 1.5)
            source = job(canned=canned)
            posted = source.post_nc("top", machine=linuxcnc(source), accept={"unchecked_machine"})
            self.assertTrue(posted.text)
            with self.assertRaises(ExportRefused):
                source.post_nc("top", machine=linuxcnc(source))

    def test_spot_chamfer_is_reported_from_the_declared_cone(self):
        report = run(job(diameter=5.5))[2]["preparation"]["spot"]
        self.assertAlmostEqual(report["chamfer_radial_width_mm"], 0.25)
        self.assertAlmostEqual(report["chamfer_axial_depth_mm"], 0.25 / math.tan(math.radians(75)))

    def test_invalid_references_order_coverage_geometry_and_depth_are_refused(self):
        source = job()
        spot, pilot, final = operations(source)
        cases = [
            ((pilot, spot, final), "precede|order"),
            ((spot, final), "missing"),
            ((replace(spot, centers_mm=((8, 8),)), pilot, final), "cover"),
            ((replace(spot, depth_z_mm=9), pilot, final), "spot_diameter"),
            ((spot, replace(pilot, depth_z_mm=3), final), "envelope"),
            (
                (spot, pilot, replace(final, preparation=DrillingPreparation(spot="holes", spot_diameter_mm=2))),
                "precede",
            ),
        ]
        for ops, message in cases:
            with self.subTest(message=message), self.assertRaisesRegex((ValueError, RuntimeError), message):
                run(with_operations(source, ops))
        bad_tool = replace(
            source.setups[0].tools[0],
            cutter=camlib.Drill("sharp", diameter=6, flute_length=30, overall_length=60, point_angle=90),
        )
        bad_spot = replace(spot, tool=bad_tool, depth_z_mm=9.0)
        changed = with_operations(source, (bad_spot, pilot, final))
        changed = replace(changed, setups=(replace(changed.setups[0], tools=(bad_tool, *changed.setups[0].tools[1:])),))
        with self.assertRaisesRegex((ValueError, RuntimeError), "angle"):
            run(changed)

    def test_cached_preparation_reports_are_bound_to_the_current_geometry(self):
        with TemporaryDirectory() as directory:
            cache = camlib.RegenerationCache(directory)
            for expected_hit in (False, True):
                with job().regenerate(cache) as result:
                    updates = [u.update for u in result]
                report = [u.report for u in updates if isinstance(u, OperationUpdate) and u.stage == "end"][-1]
                self.assertEqual(report["regeneration"]["cache_hit"], expected_hit)
                self.assertAlmostEqual(report["preparation"]["spot"]["diameter_mm"], 2.0)
            with job(diameter=2.4).regenerate(cache) as result:
                updates = [u.update for u in result]
            report = [u.report for u in updates if isinstance(u, OperationUpdate) and u.stage == "end"][-1]
            self.assertFalse(report["regeneration"]["cache_hit"])
            self.assertAlmostEqual(report["preparation"]["spot"]["diameter_mm"], 2.4)

    def test_builder_rejects_ambiguous_stages_and_serializes_no_default_requirement(self):
        source = job(declared=False)
        spot, pilot, final = operations(source)
        self.assertNotIn("preparation", final.to_dict())
        for sequence in (
            DrillingSequence(final),
            DrillingSequence(final, spot=replace(spot, id=final.id), spot_diameter_mm=2),
            DrillingSequence(final, pilot=replace(pilot, preparation=DrillingPreparation(pilot="x"))),
        ):
            with self.assertRaises(ValueError):
                sequence.operations()
        for value in (-1, float("nan"), True):
            with self.assertRaises(ValueError):
                DrillingPreparation(spot="s", spot_diameter_mm=value).to_dict()


if __name__ == "__main__":
    unittest.main()
