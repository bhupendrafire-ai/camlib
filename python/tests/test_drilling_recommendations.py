"""Declared synthetic cutting records; no manufacturer-rate default table."""

from dataclasses import replace
import math
import unittest

import camlib
from camlib.cutting import DrillingCuttingData, recommend_drilling
from camlib.export import ExportRefused
from camlib.simulation import SimulationError
from test_drilling_options import fixture, run
from test_drill_cycles import linuxcnc


def record(tool, **changes):
    values = dict(
        id="synthetic-p20",
        tool=tool.id,
        material="synthetic-p20",
        source="Synthetic qualification only",
        conditions="Fictional declared tool/material conditions; not machining advice",
        diameter_mm=tool.cutter.diameter_mm,
        vc_min_m_min=10.0,
        vc_max_m_min=50.0,
        feed_per_rev_mm=0.08,
        max_depth_diameter_ratio=5.0,
    )
    return DrillingCuttingData(**{**values, **changes})


def recommendation(source=None, **changes):
    source = source or fixture(peck_mm=None)
    setup = source.setups[0]
    tool = setup.tools[0]
    kwargs = dict(
        material="synthetic-p20",
        diameter_mm=tool.cutter.diameter_mm,
        tip_depth_mm=6.0,
        diameter_tolerance_mm=0.001,
        machine=replace(linuxcnc(source), max_spindle_rpm=2000.0, max_feed_mm_min=1000.0),
    )
    return recommend_drilling((tool,), (record(tool),), **{**kwargs, **changes})


class RecommendationsTest(unittest.TestCase):
    def test_feed_is_per_revolution_and_caps_keep_the_supplied_speed_range(self):
        candidate = recommendation().candidates[0]
        self.assertEqual(candidate.cutting.spindle_rpm, 2000.0)
        self.assertAlmostEqual(candidate.cutting.feed_mm_min, 2000 * 0.08)
        self.assertAlmostEqual(candidate.actual_surface_speed_m_min, math.pi * 5 * 2000 / 1000)
        self.assertTrue(candidate.cutting.rpm_limited)
        self.assertFalse(candidate.feed_limited)
        source = fixture(peck_mm=None)
        machine = replace(linuxcnc(source), max_spindle_rpm=10000.0, max_feed_mm_min=80.0)
        candidate = recommendation(source, machine=machine).candidates[0]
        self.assertTrue(candidate.feed_limited)
        self.assertAlmostEqual(candidate.cutting.feed_mm_min, 80.0)
        self.assertAlmostEqual(candidate.cutting.spindle_rpm, 1000.0)

    def test_recommendation_uses_the_existing_native_and_independent_posted_gates(self):
        source = fixture(peck_mm=None)
        setup = source.setups[0]
        old = setup.operations[0].operation
        candidate = recommendation(source).candidates[0]
        selected = replace(
            old,
            tool=candidate.tool,
            spindle_rpm=candidate.cutting.spindle_rpm,
            feed_mm_min=candidate.cutting.feed_mm_min,
        )
        proposed = replace(source, setups=(replace(setup, operations=(camlib.OperationNode(selected, sources=()),)),))
        self.assertEqual(run(source)[1], run(proposed)[1])
        posted = proposed.post_nc("top", machine=linuxcnc(proposed), accept={"unchecked_machine"})
        self.assertTrue(posted.text)
        with self.assertRaises(ExportRefused):
            proposed.post_nc("top", machine=linuxcnc(proposed))
        self.assertNotEqual(old.spindle_rpm, selected.spindle_rpm)
        self.assertEqual(source.setups[0].operations[0].operation, old)

    def test_invalid_or_incompatible_declarations_never_fall_back(self):
        source = fixture(peck_mm=None)
        tool = source.setups[0].tools[0]
        kwargs = dict(
            material="synthetic-p20",
            diameter_mm=5.0,
            tip_depth_mm=6.0,
            diameter_tolerance_mm=0.001,
            machine=linuxcnc(source),
        )
        for data, changes, reason in [
            ((record(tool, material="other"),), {}, "material"),
            ((record(tool, diameter_mm=4.0),), {}, "diameter"),
            ((record(tool, max_depth_diameter_ratio=0.5),), {}, "depth"),
            ((record(tool),), {"tip_depth_mm": 100.0}, "flute"),
            ((record(tool),), {"machine": replace(linuxcnc(source), max_spindle_rpm=10.0)}, "speed range"),
            ((), {}, "no supplied"),
        ]:
            with self.subTest(reason=reason):
                result = recommend_drilling((tool,), data, **{**kwargs, **changes})
                self.assertFalse(result.candidates)
                self.assertTrue(any(reason in r['reason'] for r in result.rejected))
        for bad in (
            record(tool, feed_per_rev_mm=float('nan')),
            record(tool, source=""),
            record(tool, vc_min_m_min=60.0),
        ):
            with self.assertRaises(ValueError):
                recommend_drilling((tool,), (bad,), **kwargs)

    def test_declared_machine_table_and_real_obstacles_keep_authority(self):
        source = fixture(peck_mm=None, r_plane_travel=True, obstacle=True)
        machine = linuxcnc(source)
        missing = replace(machine, tools=(replace(machine.tools[0], tool="unavailable"),))
        self.assertFalse(recommendation(source, machine=missing).candidates)
        with self.assertRaises(ValueError):
            replace(machine, tools=(*machine.tools, *machine.tools))
        axis_limited = replace(machine, max_axis_feed_mm_min=(1000.0, 1000.0, 80.0))
        self.assertAlmostEqual(recommendation(source, machine=axis_limited).candidates[0].cutting.feed_mm_min, 80.0)
        # A plausible rate proposal cannot bypass independent fixture/stock checks.
        self.assertTrue(recommendation(source).candidates)
        with self.assertRaisesRegex(SimulationError, "not proven Clear"):
            source.post_nc("top", machine=machine, accept={"unchecked_machine"})

    def test_order_budgets_duplicates_and_cancellation_publish_no_partial_result(self):
        source = fixture(peck_mm=None)
        tool = source.setups[0].tools[0]
        other = replace(tool, id="longer", cutter=replace(tool.cutter, cutting_length_mm=35.0))
        machine = replace(
            linuxcnc(source),
            tools=(*linuxcnc(source).tools, camlib.ToolSetting.for_tool(other, number=2, length_register=2)),
        )
        kwargs = dict(
            material="synthetic-p20", diameter_mm=5.0, tip_depth_mm=6.0, diameter_tolerance_mm=0.001, machine=machine
        )
        for tools, data in [
            ((tool, other), (record(tool), record(other))),
            ((other, tool), (record(other), record(tool))),
        ]:
            result = recommend_drilling(tools, data, **kwargs)
            self.assertEqual([r.tool.id for r in result.candidates], [tool.id, other.id])
        for tools, data in [((tool, tool), (record(tool),)), ((tool,), (record(tool), record(tool)))]:
            with self.assertRaisesRegex(ValueError, "duplicate"):
                recommend_drilling(tools, data, **kwargs)
        with self.assertRaisesRegex(ValueError, "max_candidates"):
            recommend_drilling((tool, other), (record(tool),), max_candidates=1, **kwargs)
        calls = 0

        def cancelled():
            nonlocal calls
            calls += 1
            return calls == 4

        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            recommend_drilling((tool, other), (record(tool), record(other)), cancel=cancelled, **kwargs)


if __name__ == '__main__':
    unittest.main()
