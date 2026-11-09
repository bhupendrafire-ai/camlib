"""Host example preserves source/intent decisions without weakening native checks."""

from dataclasses import replace
import importlib.util
from pathlib import Path
import unittest

from camlib.jobs import OperationNode
from test_initial_stock import blank
from test_planning import candidate

_spec = importlib.util.spec_from_file_location(
    'host_preparation', Path(__file__).resolve().parents[2] / 'examples/host_preparation.py'
)
host = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(host)


class HostPreparationTest(unittest.TestCase):
    def test_colour_difference_reference_pairs(self):
        for a, b, expected in (
            ((50, 2.6772, -79.7751), (50, 0, -82.7485), 2.0425),
            ((50, 0, 0), (50, -1, 2), 2.3669),
            ((50, 2.5, 0), (73, 25, -18), 27.1492),
            ((60.2574, -34.0099, 36.2677), (60.4626, -34.1751, 39.4387), 1.2644),
            ((22.7233, 20.0904, -46.6940), (23.0331, 14.9730, -42.5619), 2.0373),
        ):
            self.assertAlmostEqual(host.ciede2000(a, b), expected, places=4)

    def test_unknown_ambiguous_and_missing_colours_remain_unresolved(self):
        finish = host.IntentEntry('finish', (255, 0, 0), 'finish', -0.01, 0.02, 0.3, False)
        keep = host.IntentEntry('keep', (0, 255, 0), 'preserve', 0, 0, 0, True)
        report = host.resolve_colours(
            dict(face1=(255, 0, 0), face2=None, face3=(0, 0, 255)),
            (finish, keep),
            source_sha256='a' * 64,
            chart_id='shop',
            chart_revision='1',
            max_delta_e=1,
            ambiguity_delta_e=0.1,
        )
        self.assertEqual([r['status'] for r in report['faces'].values()], ['resolved', 'no_colour', 'unknown'])
        intent = report['faces']['face1']['intent']
        self.assertEqual((intent['lower_mm'], intent['upper_mm'], intent['allowance_mm']), (-0.01, 0.02, 0.3))
        conflict = host.resolve_colours(
            dict(face1=(255, 0, 0)),
            (finish, replace(keep, rgb=finish.rgb)),
            source_sha256='a' * 64,
            chart_id='shop',
            chart_revision='1',
            max_delta_e=1,
            ambiguity_delta_e=0.1,
        )
        self.assertEqual(conflict['faces']['face1']['status'], 'ambiguous')
        with self.assertRaisesRegex(ValueError, 'unresolved'):
            host.attach_intent(OperationNode(object()), dict(face1='region'), conflict)

    def test_chart_source_and_face_mapping_bind_to_job_regeneration(self):
        base = candidate()
        setup = base.job.setups[0]
        node = OperationNode(setup.operations[0], inputs={'existing': 'retained'})
        entry = host.IntentEntry('finish', (255, 0, 0), 'finish', -0.01, 0.02, 0.3, False)

        def compile(chart_revision='1', source='a' * 64, region='converted-face'):
            resolution = host.resolve_colours(
                dict(face1=(255, 0, 0)),
                (entry,),
                source_sha256=source,
                chart_id='shop',
                chart_revision=chart_revision,
                max_delta_e=1,
                ambiguity_delta_e=0.1,
            )
            bound = host.attach_intent(node, dict(face1=region), resolution)
            self.assertEqual(bound.inputs['host_inputs'], node.inputs)
            return replace(base.job, setups=(replace(setup, operations=(bound,)),)).compile().setup('top')

        original = compile()
        for changed in (compile(chart_revision='2'), compile(source='b' * 64), compile(region='remapped-face')):
            self.assertNotEqual(changed.dependencies, original.dependencies)
        self.assertEqual(compile().dependencies, original.dependencies)

    def test_ambiguous_registration_requires_an_explicit_supplied_placement(self):
        a = blank()
        b = replace(a, source_to_setup=replace(a.source_to_setup, origin_mm=(1, 0, 0)))
        proposals = (('left', a), ('right', b))
        self.assertIs(host.confirmed_blank(proposals, confirmed_id='right'), b)
        for key in (None, '', 'unmeasured'):
            with self.assertRaises(ValueError):
                host.confirmed_blank(proposals, confirmed_id=key)


if __name__ == '__main__':
    unittest.main()
