"""Feed advice uses committed native stock, declared load limits and proved air."""

from dataclasses import replace
import unittest

from camlib.cutting import EngagementFeedBand, EngagementFeedPolicy, schedule_engagement
from camlib.motion import Line
from camlib.post import Cancellation, post_program
from camlib.posted import OperationEvents
from camlib.postprocessor import _event_data, _typed
from camlib.simulation import Simulation, StockSpec
from test_post import fixture


STOCK = StockSpec((0, -5, 0), (20, 5, 5), 0.5)
POLICY = EngagementFeedPolicy((EngagementFeedBand(90, 300),), 'refuse')


class EngagementFeedsTest(unittest.TestCase):
    def test_native_load_refusals_explain_bounds_limits_and_how_to_revise(self):
        from camlib.cutting import ToolLoadLimit

        program, _ = fixture()
        tool = program.tools[0]
        ball = replace(tool, cutter=replace(tool.cutter, shape=(('kind', 'ball'),)))
        cut = replace(program.events[4], curve=Line((5, 0, 3), (10, 0, 2)))
        program = replace(program, tools=(ball,), events=(*program.events[:4], cut, *program.events[5:]))
        for axial, radial, axis in ((0.1, 4, 'axial'), (5, 0.1, 'radial')):
            policy = replace(POLICY, tool_limits=(ToolLoadLimit('t1', axial, radial, 300),))
            with self.subTest(axis=axis), self.assertRaises(ValueError) as refused:
                schedule_engagement(program, STOCK, policy=policy)
            message = str(refused.exception)
            self.assertIn("motion 1 (tool 't1')", message)
            self.assertIn(f'{axis} load bound', message)
            self.assertIn('mm exceeds approved maximum 0.1 mm', message)
            self.assertIn('reviewed tool load limits', message)
            self.assertNotIn('Some(', message)
            preserved = schedule_engagement(program, STOCK, policy=replace(policy, unresolved='preserve'))
            self.assertEqual(preserved.program.events, program.events)
            self.assertEqual(preserved.report['observations'][0]['reason'], 'approved_load_limit_missing_or_exceeded')
        other = replace(ball, id='other-tool')
        missing = replace(program, tools=(ball, other))
        policy = replace(POLICY, tool_limits=(ToolLoadLimit('other-tool', 5, 4, 300),))
        with self.assertRaises(ValueError) as refused:
            schedule_engagement(missing, STOCK, policy=policy)
        self.assertIn("tool 't1'", str(refused.exception))
        self.assertIn('approved tool load limits are missing', str(refused.exception))
        self.assertIn('ToolLoadLimit', str(refused.exception))
        self.assertIn('mm/min', str(refused.exception))

    def test_general_shape_load_caps_and_proven_air_increase(self):
        from camlib.cutting import ToolLoadLimit

        program, machine = fixture()
        tool = program.tools[0]
        policy = replace(
            POLICY, tool_limits=(ToolLoadLimit(tool.id, 5, tool.cutter.diameter_mm, 450),), air_feed_mm_min=1200
        )
        cut = program.events[4]
        air = replace(cut, curve=Line((5, 0, 8), (10, 0, 8)), feed_mm_min=600)
        air_prefix = (
            program.events[0],
            replace(program.events[1], position_after_mm=air.curve.start),
            *program.events[2:4],
        )
        air_program = replace(
            program, initial_position_mm=air.curve.start, events=(*air_prefix, air, *program.events[5:])
        )
        machine = replace(machine, tools=(replace(machine.tools[0], position_after_mm=air.curve.start),))
        result = schedule_engagement(air_program, STOCK, policy=policy)
        row = result.report['observations'][0]
        self.assertEqual(row['status'], 'proven_air')
        self.assertEqual(row['requested_feed_mm_min'], 450)  # explicit tool cap still applies
        policy = replace(policy, tool_limits=(replace(policy.tool_limits[0], max_cut_feed_mm_min=2000),))
        result = schedule_engagement(air_program, STOCK, policy=policy)
        self.assertEqual(result.program.events[4].feed_mm_min, 1200)
        self.assertEqual(
            [m.feed_mm_min for c in post_program(result.program, machine=machine) for m in c.motions], [1200]
        )
        near = replace(air, curve=Line((5, 0, 5), (10, 0, 5)))
        result = schedule_engagement(
            replace(
                air_program,
                events=(*air_prefix, replace(near, curve=Line(air.curve.start, near.curve.end)), *program.events[5:]),
            ),
            STOCK,
            policy=policy,
        )
        self.assertFalse(result.report['observations'][0]['outside_original_stock'])
        self.assertLessEqual(result.program.events[4].feed_mm_min, near.feed_mm_min)
        ball = replace(tool, cutter=replace(tool.cutter, shape=(('kind', 'ball'),)))
        sloped = replace(cut, curve=Line((5, 0, 3), (10, 0, 2)))
        result = schedule_engagement(
            replace(program, tools=(ball,), events=(*program.events[:4], sloped, *program.events[5:])),
            STOCK,
            policy=policy,
        )
        row = result.report['observations'][0]
        self.assertEqual(row['status'], 'bounded_load')
        self.assertGreaterEqual(row['axial_upper_mm'], 3)
        self.assertEqual(row['radial_upper_mm'], tool.cutter.diameter_mm)
        exceeded = replace(policy, tool_limits=(replace(policy.tool_limits[0], max_axial_mm=0.1),))
        with self.assertRaisesRegex(ValueError, 'load_limit'):
            schedule_engagement(program, STOCK, policy=exceeded)
        preserved = schedule_engagement(program, STOCK, policy=replace(exceeded, unresolved='preserve'))
        self.assertEqual(preserved.program.events, program.events)
        self.assertEqual(preserved.report['observations'][0]['status'], 'unresolved')

    def test_general_cutter_shapes_and_helix_keep_conservative_load_bounds(self):
        import math
        from camlib.cutting import ToolLoadLimit
        from camlib.motion import Helix

        program, _ = fixture()
        original = program.tools[0]
        policy = replace(POLICY, tool_limits=(ToolLoadLimit(original.id, 5, 4, 500),))
        for shape in (
            (('kind', 'flat'),),
            (('kind', 'ball'),),
            (('kind', 'bull'), ('corner_radius_mm', 1)),
            (('kind', 'drill'), ('point_angle_rad', math.radians(118))),
            (('kind', 'chamfer'), ('included_angle_rad', math.pi / 2), ('tip_diameter_mm', 0.5)),
        ):
            with self.subTest(shape=shape):
                tool = replace(original, cutter=replace(original.cutter, shape=shape))
                cut = replace(program.events[4], curve=Helix((5, 0, 3), (5, 0, 2), (5, 2, 3), math.tau))
                source = replace(program, tools=(tool,), events=(*program.events[:4], cut, *program.events[5:]))
                result = schedule_engagement(source, STOCK, policy=policy)
                row = result.report['observations'][0]
                self.assertEqual(row['status'], 'bounded_load')
                self.assertEqual(row['radial_upper_mm'], 4)
                self.assertGreaterEqual(row['axial_upper_mm'], 3)
                self.assertLessEqual(row['requested_feed_mm_min'], 300)
                self.assertEqual(result.program.events[4].curve, cut.curve)

    def test_declared_load_limits_apply_to_unsynchronized_plunges(self):
        from camlib.cutting import ToolLoadLimit

        program, _ = fixture()
        tool = program.tools[0]
        plunge = replace(program.events[4], curve=Line((5, 0, 3), (5, 0, 2)), kind='plunge')
        source = replace(program, events=(*program.events[:4], plunge, *program.events[5:]))
        limit = ToolLoadLimit(tool.id, 5, tool.cutter.diameter_mm, 150)
        policy = replace(POLICY, tool_limits=(limit,))
        result = schedule_engagement(source, STOCK, policy=policy)
        row = result.report['observations'][0]
        self.assertEqual(row['status'], 'bounded_load')
        self.assertGreaterEqual(row['axial_upper_mm'], 3)
        self.assertEqual(row['radial_upper_mm'], tool.cutter.diameter_mm)
        self.assertFalse(row['outside_original_stock'])
        self.assertEqual(result.program.events[4].feed_mm_min, 150)
        self.assertEqual(result.program.events[4].curve, plunge.curve)
        for invalid in (replace(limit, max_axial_mm=0.1), replace(limit, max_radial_mm=0.1)):
            refused = replace(policy, tool_limits=(invalid,))
            with self.assertRaisesRegex(ValueError, 'load_limit'):
                schedule_engagement(source, STOCK, policy=refused)
            result = schedule_engagement(source, STOCK, policy=replace(refused, unresolved='preserve'))
            self.assertEqual(result.program.events[4], plunge)
            self.assertEqual(result.report['observations'][0]['status'], 'unresolved')
        # The opt-in mode does not change legacy plunge behaviour.
        self.assertEqual(schedule_engagement(source, STOCK, policy=POLICY).program.events[4], plunge)

    def test_air_policy_is_machine_scheduled_then_independently_replayed_before_release(self):
        import os
        from pathlib import Path
        from tempfile import TemporaryDirectory
        from camlib.cutting import ToolLoadLimit
        from camlib.dynamics import MotionPolicy
        from camlib.material import Target
        from camlib.planar import PlanarRegion, Component, Boundary
        from camlib.posted import PostedRun
        from camlib.verification import Scene, MachineLimits, StreamChecks, StockContactOptions, MaterialCheck
        from test_machine import model

        program, machine = fixture()
        program = replace(
            program, events=(*program.events[:4], replace(program.events[4], feed_mm_min=600), *program.events[5:])
        )
        stock = StockSpec((0, -5, 0), (20, 5, 1), 0.5)
        policy = replace(POLICY, air_feed_mm_min=1200, tool_limits=(ToolLoadLimit('t1', 5, 4, 2000),))
        advised = schedule_engagement(program, stock, policy=policy)
        self.assertEqual(advised.program.events[4].feed_mm_min, 1200)
        machine = replace(machine, motion_policy=MotionPolicy(100, 6000), max_feed_mm_min=700, schedule_post_feeds=True)
        machine = replace(machine, crash_model=model(program, machine))
        scheduled = machine.schedule(advised.program)
        self.assertLessEqual(scheduled.program.events[4].feed_mm_min, 700)
        region = PlanarRegion('part', (Component(Boundary.polygon(((-10, -10), (30, -10), (30, 10), (-10, 10)))),))
        checks = StreamChecks(
            Scene('limits', '1', machine=MachineLimits((-100,) * 3, (100,) * 3, 10000)),
            stock_contact=StockContactOptions(),
            material=(MaterialCheck(Target('part', '1', region, 1)),),
        )
        with (
            TemporaryDirectory() as folder,
            PostedRun(
                scheduled.program,
                machine=machine,
                stock=stock,
                verification=checks,
                executable=os.environ.get('CAMLIB_TEST_ENGINE'),
            ) as run,
        ):
            checked = run.check_release()
            self.assertEqual(checked['machine']['status'], 'clear')
            self.assertFalse(checked['unchecked'])
            result = run.export(Path(folder) / 'synthetic-air-test.nc')
            self.assertFalse(result['accepted'])
            import re

            feeds = [
                float(value)
                for value in re.findall(r'\bF([0-9.]+)', (Path(folder) / 'synthetic-air-test.nc').read_text())
            ]
            self.assertTrue(feeds)
            self.assertTrue(all(0 < feed <= 700 for feed in feeds))

    def test_declared_caps_use_committed_prefix_and_post_actual_new_feeds(self):
        program, machine = fixture()
        cut = program.events[4]
        back = replace(cut, id=2, curve=Line(cut.curve.end, cut.curve.start))
        program = replace(program, events=(*program.events[:5], back, *program.events[5:]))
        before = program.to_dict()
        result = schedule_engagement(program, STOCK, policy=POLICY)
        first, second = result.report['observations']
        self.assertEqual(first['stock_generation'], 0)
        self.assertEqual(second['stock_generation'], 1)
        self.assertEqual(first['status'], 'bounded')
        self.assertGreaterEqual(first['upper_angle_deg'], 90)
        self.assertEqual(first['requested_feed_mm_min'], 300)
        self.assertLessEqual(second['upper_angle_deg'], first['upper_angle_deg'])
        self.assertEqual(program.to_dict(), before)
        self.assertNotEqual(result.program.revision, program.revision)
        self.assertEqual(schedule_engagement(program, STOCK, policy=POLICY).report, result.report)
        motions = [m for chunk in post_program(result.program, machine=machine) for m in chunk.motions]
        self.assertEqual(
            [m.feed_mm_min for m in motions], [r['requested_feed_mm_min'] for r in result.report['observations']]
        )
        self.assertEqual([m.curve for m in motions], [cut.curve, back.curve])
        with self.assertRaisesRegex(ValueError, 'feed'):
            list(post_program(result.program, machine=replace(machine, max_feed_mm_min=200)))

    def test_native_tapping_and_drill_cycles_keep_their_authored_feeds(self):
        from test_tapping import job, tapping

        compiled = job(tapping).compile().setup('top')
        with Simulation(**compiled.simulation_options()) as sim:
            events = tuple(_typed(_event_data(e)) for e in OperationEvents(sim))
        program = replace(compiled.program_header(), events=events)
        result = schedule_engagement(program, compiled.config['stock'], policy=POLICY)
        self.assertEqual(result.program.events, program.events)
        from camlib.cutting import ToolLoadLimit

        limits = tuple(
            ToolLoadLimit(tool.id, tool.cutter.cutting_length_mm, tool.cutter.diameter_mm, 0.001)
            for tool in program.tools
        )
        result = schedule_engagement(program, compiled.config['stock'], policy=replace(POLICY, tool_limits=limits))
        covered = [row for row in result.report['observations'] if row['status'] == 'unchanged_cycle']
        self.assertTrue(covered)
        for row in covered:
            self.assertEqual(result.program.events[row['event_index']], program.events[row['event_index']])
        # Ordinary lead-in plunges are outside the synchronized cycle and
        # correctly obey the explicit cap; the tap/cycle feeds above do not.
        self.assertTrue(any(row['status'] == 'bounded_load' for row in result.report['observations']))

    def test_work_exhaustion_requires_explicit_preserve(self):
        program, _ = fixture()
        policy = replace(
            POLICY, options=dict(tolerance_mm=0.00001, resolution_mm=0.001, max_depth=24, max_nodes=0, max_work=0)
        )
        with self.assertRaisesRegex(ValueError, 'unresolved') as refused:
            schedule_engagement(program, STOCK, policy=policy)
        self.assertIn('max_work=0', str(refused.exception))
        self.assertIn('increase the declared analysis budget', str(refused.exception))
        self.assertNotIn('Some(', str(refused.exception))
        result = schedule_engagement(program, STOCK, policy=replace(policy, unresolved='preserve'))
        self.assertEqual(result.program.events, program.events)
        self.assertEqual(result.report['observations'][0]['status'], 'unresolved')

    def test_lower_authored_feed_is_never_raised_and_premises_change_identity(self):
        program, _ = fixture()
        cut = replace(program.events[4], feed_mm_min=100)
        program = replace(program, events=(*program.events[:4], cut, *program.events[5:]))
        result = schedule_engagement(program, STOCK, policy=POLICY)
        self.assertEqual(result.program.events[4].feed_mm_min, 100)
        changed = schedule_engagement(program, replace(STOCK, max_mm=(21, 5, 5)), policy=POLICY)
        self.assertNotEqual(changed.report['identity'], result.report['identity'])

    def test_unsupported_moves_need_explicit_preserve_and_plunges_stay_authored(self):
        program, _ = fixture()
        cut = program.events[4]
        sloped = replace(cut, curve=Line((5, 0, 3), (10, 0, 2)))
        program = replace(program, events=(*program.events[:4], sloped, *program.events[5:]))
        with self.assertRaisesRegex(ValueError, 'horizontal'):
            schedule_engagement(program, STOCK, policy=POLICY)
        result = schedule_engagement(program, STOCK, policy=replace(POLICY, unresolved='preserve'))
        self.assertEqual(result.program.events[4], sloped)
        self.assertEqual(result.report['observations'][0]['status'], 'unsupported')
        plunge = replace(sloped, curve=Line((5, 0, 3), (5, 0, 2)), kind='plunge')
        result = schedule_engagement(
            replace(program, events=(*program.events[:4], plunge, *program.events[5:])), STOCK, policy=POLICY
        )
        self.assertEqual(result.program.events[4], plunge)

    def test_invalid_policy_and_cancellation_never_return_a_program(self):
        program, _ = fixture()
        for policy in (
            replace(POLICY, unresolved='guess'),
            replace(POLICY, bands=()),
            replace(POLICY, bands=(EngagementFeedBand(90, 100), EngagementFeedBand(180, 200))),
            replace(
                POLICY, options=dict(tolerance_mm=-1, resolution_mm=0.001, max_depth=24, max_nodes=100, max_work=100)
            ),
        ):
            with self.subTest(policy=policy):
                with self.assertRaises(ValueError):
                    schedule_engagement(program, STOCK, policy=policy)
        cancelled = Cancellation()
        cancelled.cancel()
        with self.assertRaisesRegex(ValueError, 'cancelled'):
            schedule_engagement(program, STOCK, policy=POLICY, cancel=cancelled)


if __name__ == '__main__':
    unittest.main()
