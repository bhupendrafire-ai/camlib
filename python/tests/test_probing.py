"""Probing (issue #7): typed probing and tool-setting cycles posted for
LinuxCNC (G38.x) and Renishaw Inspection Plus macros (FANUC), decoded back
from the written blocks and checked against the plan; the stylus checked
against fixtures; the stock never changed."""

from pathlib import Path
import sys
import tempfile
import unittest

import camlib
from camlib.export import ExportRefused
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.post import ToolSetting
from camlib.postprocessor import GCodeDialect, GCodePost
from camlib.simulation import OperationUpdate, Simulation
from camlib.verification import Prism, Scene, StockContactOptions, StreamChecks
from test_drill_cycles import linuxcnc
from test_tapping import posted_text

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'examples'))
try:
    from fanuc_0imb_post import Fanuc0iMBDecoder, Fanuc0iMBPost, define_machine
finally:
    sys.path.pop(0)

PROBE = camlib.ProbeTool('probe', ball_diameter=4, stem_diameter=3, stylus_length=50, body_diameter=40, body_length=60)
MILL = camlib.ToolAssembly(
    'mill',
    camlib.EndMill('mill', diameter=10, flute_length=20, overall_length=60),
    40,
    80,
    (camlib.HolderSection(40, 80, 15, 15),),
)
STOCK = camlib.StockSpec((0, 0, 0), (40, 30, 10), 0.25)
FEEDS = dict(position_feed_mm_min=1000, measure_feed_mm_min=100, return_feed_mm_min=1000)
COMMON = dict(clearance_z_mm=20, search_mm=2, tool_number=1, length_register=1, update_work_offset=1)
# Inspection Plus with fast feed #119 = 2000: gauging at 0.6 x #119,
# positioning back at #119 in XY.
FANUC_FEEDS = dict(position_feed_mm_min=1000, measure_feed_mm_min=1200, return_feed_mm_min=2000)


def corner(**changes):
    values = {**COMMON, **FEEDS, **changes}
    return camlib.ProbeOutsideCorner(
        'corner', PROBE, corner_mm=(0, 0), z_mm=5, material=(1, 1), inset_mm=5, clearance_mm=5, **values
    )


def bore(**changes):
    values = {**COMMON, **FEEDS, **changes}
    return camlib.ProbeBore('bore', PROBE, center_mm=(20, 15), z_mm=5, diameter_mm=12, **values)


def tool_length(**changes):
    values = dict(
        clearance_z_mm=20,
        search_mm=2,
        tool_number=1,
        length_register=1,
        measure_feed_mm_min=100,
        return_feed_mm_min=1000,
        spindle_rpm=0,
    )
    values.update(changes)
    return camlib.ToolSetLength('length', MILL, contact_mm=(55, 15, 2), clearance_mm=5, **values)


def job(*operations, scene=None):
    tools = tuple({id(t): t for op in operations for t in (op.tool,)}.values())
    setup = camlib.Setup(
        'top',
        stock=STOCK,
        tools=tools,
        operations=tuple(camlib.OperationNode(op, sources=()) for op in operations),
        initial_position_mm=(0, 0, 50),
        verification=StreamChecks(scene=scene, stock_contact=StockContactOptions()),
    )
    return camlib.Job('probing', '1', (setup,))


def g38(j):
    return linuxcnc(j, GCodeDialect.linuxcnc(probing='g38'))


def fanuc(j, **settings):
    header = j.compile().setup('top').program_header()
    tools = tuple(
        ToolSetting.for_tool(t, number=i, length_register=i, position_after_mm=header.initial_position_mm)
        for i, t in enumerate(header.tools, 1)
    )
    return define_machine(
        'probing mill',
        tools,
        startup_units='mm',
        startup_standard_modes=True,
        g92_shift_mm=(0, 0, 0),
        m_codes=dict(
            tool_change='M6', spindle_cw='M3', spindle_ccw='M4', spindle_off='M5', coolant_off='M9', flood='M8'
        ),
        traverse_feed_mm_min=3000,
        work_to_machine=camlib.motion.WorkFrame(origin_mm=(100, 200, 300)),
        max_feed_mm_min=10000,
        max_spindle_rpm=30000,
        max_sweep_rad=6.283185307179586,
        tool_changes=True,
        flood=True,
        tool_change_position_mode='table',
        rapid_mm_min=6000,
        output_units='mm',
        decimals=3,
        **{'inspection_plus_fast_feed_mm_min': 2000, **settings},
    )


def fanuc_text(j, machine):
    return posted_text(j, machine, Fanuc0iMBPost, Fanuc0iMBDecoder)


def protected(z_or_words, label):
    return [
        f'G38.3 {z_or_words} F1000',
        f'O{label} IF [#5070 NE 0]',
        '(ABORT, camlib: protected probe move obstructed)',
        f'O{label} ENDIF',
    ]


def cycle(nc, first, last):
    start = next(i for i, line in enumerate(nc) if line.startswith(first))
    end = max(i for i, line in enumerate(nc) if line.startswith(last))
    return nc[start : end + 1]


class LinuxCNCTest(unittest.TestCase):
    def test_outside_corner_posts_g38_and_decodes_back(self):
        nc = posted_text(job(corner()), g38(job(corner())))
        # Ball radius 2: the Y face touched at x = 5 from y = -7, expected
        # contact y = -2, search 2 beyond it; then the X face.
        self.assertEqual(
            cycle(nc, 'G38.3', 'G10'),
            [
                *protected('Z5', 100),
                'G38.2 Y0 F100',
                '#<probe1> = [#5062 + 2]',
                'G1 Y-7 F1000',
                *protected('Z20', 101),
                *protected('X-7 Y5', 102),
                *protected('Z5', 103),
                'G38.2 X0 F100',
                '#<probe2> = [#5061 + 2]',
                'G1 X-7 F1000',
                *protected('Z20', 104),
                # New offset = active offset (G54: #5221..) + each axis's error.
                'G10 L2 P1 X[#5221 + #<probe2>] Y[#5222 + #<probe1>]',
            ],
        )

    def test_bore_averages_opposite_touches(self):
        nc = posted_text(job(bore()), g38(job(bore())))
        self.assertEqual(
            cycle(nc, 'G38.3', 'G10'),
            [
                *protected('Z5', 100),
                'G38.2 X26 F100',
                '#<probe1> = [#5061 - 24]',
                'G1 X20 F1000',
                'G38.2 X14 F100',
                '#<probe2> = [#5061 - 16]',
                'G1 X20 F1000',
                'G38.2 Y21 F100',
                '#<probe3> = [#5062 - 19]',
                'G1 Y15 F1000',
                'G38.2 Y9 F100',
                '#<probe4> = [#5062 - 11]',
                'G1 Y15 F1000',
                *protected('Z20', 101),
                'G10 L2 P1 X[#5221 + [#<probe1> + #<probe2>] / 2] Y[#5222 + [#<probe3> + #<probe4>] / 2]',
            ],
        )

    def test_every_feature_posts(self):
        values = {**COMMON, **FEEDS}
        features = (
            camlib.ProbeSurface('s', PROBE, start_mm=(-8, 15, 5), axis='x', surface_mm=0, **values),
            camlib.ProbeSurface('z', PROBE, start_mm=(20, 15, 15), axis='z', surface_mm=10, **values),
            camlib.ProbeInsideCorner('i', PROBE, corner_mm=(40, 30), z_mm=5, material=(1, 1), clearance_mm=5, **values),
            camlib.ProbeBoss('b', PROBE, center_mm=(20, 15), z_mm=5, diameter_mm=12, clearance_mm=4, **values),
            camlib.ProbePocket('p', PROBE, center_mm=(20, 15), z_mm=5, axis='y', width_mm=10, **values),
            camlib.ProbeWeb('w', PROBE, center_mm=(20, 15), z_mm=5, axis='x', width_mm=8, clearance_mm=4, **values),
        )
        expected = {
            's': 'G10 L2 P1 X[#5221 + #<probe1>]',
            'z': 'G10 L2 P1 Z[#5223 + #<probe1>]',
            'i': 'G10 L2 P1 X[#5221 + #<probe2>] Y[#5222 + #<probe1>]',
            'b': 'G10 L2 P1 X[#5221 + [#<probe1> + #<probe2>] / 2] Y[#5222 + [#<probe3> + #<probe4>] / 2]',
            'p': 'G10 L2 P1 Y[#5222 + [#<probe1> + #<probe2>] / 2]',
            'w': 'G10 L2 P1 X[#5221 + [#<probe1> + #<probe2>] / 2]',
        }
        for op in features:
            j = job(op)
            nc = posted_text(j, g38(j))
            self.assertEqual([line for line in nc if line.startswith('G10')], [expected[op.id]], op.id)

    def test_tool_length_on_a_declared_setter_records_its_premise(self):
        j = job(tool_length())
        nc = posted_text(j, g38(j))
        self.assertEqual(
            cycle(nc, 'G38.2', 'G43 H1'),
            [
                'G38.2 Z0 F100',
                '#<probe1> = [#5063 - 2]',
                'G1 Z7 F1000',
                # The length register then holds the measured length, reloaded.
                'G10 L10 P1 Z[#5422 - #<probe1>]',
                'G43 H1',
            ],
        )
        with Simulation(**j.compile().setup('top').simulation_options()) as sim:
            reports = [u.report for u in sim.run_operations() if isinstance(u, OperationUpdate) and u.stage == 'end']
        self.assertEqual(reports[0]['stores'], 'the measured length of tool mill in length register 1')
        self.assertEqual(reports[0]['strategy'], 'tool_setter_cycle')

    def test_tool_diameter_from_both_sides(self):
        op = camlib.ToolSetDiameter(
            'diameter',
            MILL,
            setter_center_mm=(60, 15),
            setter_top_z_mm=2,
            setter_diameter_mm=10,
            depth_mm=3,
            clearance_mm=4,
            clearance_z_mm=20,
            search_mm=1,
            tool_number=1,
            length_register=1,
            measure_feed_mm_min=50,
            return_feed_mm_min=500,
            spindle_rpm=800,
            spindle_direction='ccw',
        )
        j = job(op)
        nc = posted_text(j, g38(j))
        self.assertIn('G10 L1 P1 R[5 + [#<probe1> - #<probe2>] / 2]', nc)
        self.assertIn('G38.2 X69 F50', nc)  # contact 70 (setter 60 + 5 + 5), searching 1 beyond.

    def test_the_decoder_refuses_a_protected_move_without_its_trip_check(self):
        class Unchecked(GCodePost):
            def on_probe(self, event):
                return [line for line in super().on_probe(event) if not line.startswith(('O1', '(ABORT'))]

        j = job(corner())
        with self.assertRaisesRegex(Exception, 'must stop the program when the probe trips'):
            posted_text(j, g38(j), Unchecked, camlib.NativeGCodeDecoder)

    def test_a_written_contact_that_differs_from_the_plan_is_refused(self):
        class Shifted(GCodePost):
            def on_probe(self, event):
                return [line.replace('[#5062 + 2]', '[#5062 + 2.5]') for line in super().on_probe(event)]

        j = job(corner())
        with self.assertRaisesRegex(Exception, 'deviation|differs|next move|search'):
            posted_text(j, g38(j), Shifted, camlib.NativeGCodeDecoder)

    def test_a_dialect_without_probing_refuses(self):
        j = job(corner())
        with self.assertRaisesRegex(Exception, 'declares no probing back end'):
            posted_text(j, linuxcnc(j))


class InspectionPlusTest(unittest.TestCase):
    def test_outside_corner_and_bore_post_as_macro_calls(self):
        j = job(corner(**FANUC_FEEDS))
        nc = fanuc_text(j, fanuc(j))
        self.assertEqual(
            cycle(nc, 'G65', 'G65'),
            [
                'G65 P9810 Z5. F1000.',
                'G65 P9811 Y0. S1. Q2.',
                'G65 P9810 Z20. F1000.',
                'G65 P9810 X-7. Y5. F1000.',
                'G65 P9810 Z5. F1000.',
                'G65 P9811 X0. S1. Q2.',
                'G65 P9810 Z20. F1000.',
            ],
        )
        j = job(bore(**FANUC_FEEDS))
        nc = fanuc_text(j, fanuc(j, inspection_plus_switching=True))
        self.assertEqual(
            cycle(nc, 'G65', 'G65'),
            ['G65 P9832', 'G65 P9810 Z5. F1000.', 'G65 P9814 D12. S1. Q2.', 'G65 P9810 Z20. F1000.', 'G65 P9833'],
        )

    def test_feeds_must_be_the_installed_macro_feeds(self):
        j = job(corner())
        with self.assertRaisesRegex(Exception, r'0.6 x the fast feed #119 \(1200 mm/min\)'):
            fanuc_text(j, fanuc(j))

    def test_unmodelled_macros_are_refused(self):
        values = {**COMMON, **FANUC_FEEDS}
        boss = camlib.ProbeBoss('b', PROBE, center_mm=(20, 15), z_mm=5, diameter_mm=12, clearance_mm=4, **values)
        for op in (boss, tool_length(measure_feed_mm_min=1200, return_feed_mm_min=2000)):
            j = job(op)
            with self.assertRaisesRegex(Exception, 'not modelled for the Inspection Plus macros'):
                fanuc_text(j, fanuc(j))

    def test_the_decoder_reads_the_macro_motion_itself(self):
        # A bore call with another diameter decodes to other contacts.
        class Wider(Fanuc0iMBPost):
            def on_probe(self, event):
                return [line.replace('D12.', 'D13.') for line in super().on_probe(event)]

        j = job(bore(**FANUC_FEEDS))
        with self.assertRaisesRegex(Exception, 'deviation|differs|next move'):
            posted_text(j, fanuc(j), Wider, Fanuc0iMBDecoder)


class SafetyTest(unittest.TestCase):
    def test_a_stylus_collision_with_a_fixture_in_the_approach_is_refused(self):
        # A clamp right where the probe comes down beside the Y face.
        clamp = Prism(
            'clamp', PlanarRegion('clamp', (Component(Boundary.polygon(((3, -9), (7, -9), (7, -6), (3, -6)))),)), 0, 6
        )
        j = job(corner(), scene=Scene('fixtures', '1', bodies=(clamp,)))
        with j.post('top', machine=g38(j)) as posted:
            posted.wait()
            with tempfile.TemporaryDirectory() as out:
                with self.assertRaises(ExportRefused) as refused:
                    posted.export(Path(out) / 'probe.ngc', accept={'unchecked_machine'})
                self.assertFalse((Path(out) / 'probe.ngc').exists())
        detail = str(refused.exception)
        self.assertIn('clamp', detail)
        # Without the clamp the same program is released.
        j = job(corner())
        with j.post('top', machine=g38(j)) as posted:
            posted.wait()
            with tempfile.TemporaryDirectory() as out:
                posted.export(Path(out) / 'probe.ngc', accept={'unchecked_machine'})
                self.assertTrue((Path(out) / 'probe.ngc').exists())

    def test_probing_never_changes_the_stock(self):
        # The bore is not machined: its protected moves hit the stock, and
        # still nothing is removed.
        for op in (corner(), bore(), tool_length()):
            with Simulation(**job(op).compile().setup('top').simulation_options()) as sim:
                kinds = {u.motion['kind'] for u in sim.run_operations() if hasattr(u, 'motion')}
                self.assertEqual(sim.state.volume()['removed_upper_mm3'], 0, op.id)
            self.assertTrue({'probe_measure', 'probe_touch', 'probe_return'} <= kinds, kinds)

    def test_a_probe_never_cuts(self):
        pocket = camlib.Drilling(
            'drill',
            PROBE,
            ((10, 10),),
            5,
            20,
            feed_mm_min=100,
            retract_feed_mm_min=100,
            spindle_rpm=1000,
            tool_number=1,
            length_register=1,
            coolant='off',
        )
        with self.assertRaisesRegex(Exception, 'never cuts'):
            with Simulation(**job(pocket).compile().setup('top').simulation_options()) as sim:
                list(sim.run_operations())


if __name__ == '__main__':
    unittest.main()
