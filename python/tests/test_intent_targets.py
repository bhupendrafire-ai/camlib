"""Resolved intent changes actual floors, limits and native material verdicts."""

from dataclasses import replace
import os
import unittest

from camlib.jobs import OperationNode
from camlib.limits import Limits
from camlib.material import Verifier
from camlib.motion import WorkFrame
from camlib.planar import PlanarRegion, Component, Boundary
from camlib.simulation import Simulation, InitialStock
from test_host_preparation import host
from test_planning import candidate


def region(name, x0, x1):
    return PlanarRegion(name, (Component(Boundary.polygon(((x0, 0), (x1, 0), (x1, 8), (x0, 8)))),))


def resolution(entries, faces):
    return host.resolve_colours(
        faces,
        entries,
        source_sha256='a' * 64,
        chart_id='shop',
        chart_revision='1',
        max_delta_e=1,
        ambiguity_delta_e=0.1,
    )


class IntentTargetsTest(unittest.TestCase):
    def test_original_operation_and_node_semantics_bind_identity_and_complete_provenance(self):
        base = candidate()
        setup = base.job.setups[0]
        entry = host.IntentEntry('finish', (255, 0, 0), 'finish', -0.02, 0.02, 0, False)
        resolved = resolution((entry,), {'f': entry.rgb, 'unmapped': None})
        faces = {'f': host.FaceRegion(region('face', 0, 8), 0.5)}
        node = OperationNode(setup.operations[0], sources=(), inputs={'drawing_revision': 'A'})

        def prepare(value):
            return host.intent_targets(
                value,
                faces,
                resolved,
                stock=setup.stock,
                global_check=setup.verification.material[0],
                geometry_tolerance_mm=1e-5,
                max_findings=1024,
            )

        first = prepare(node)
        second = prepare(replace(node, operation=replace(node.operation, id='face-again')))
        self.assertNotEqual(first.identity, second.identity)
        self.assertNotEqual(first.nodes[0].operation.id, second.nodes[0].operation.id)
        # Both independently authored operations can be placed in one setup.
        combined = replace(setup, operations=(*first.nodes, *second.nodes))
        self.assertEqual(len(replace(base.job, setups=(combined,)).compile().setup('top').provenance), 2)
        self.assertEqual(first.provenance['resolution']['faces']['unmapped']['status'], 'no_colour')
        self.assertEqual(first.provenance['resolution']['chart']['entries'][0]['id'], 'finish')
        self.assertEqual(first.provenance['faces']['f']['region'], host._plain(faces['f'].region))
        self.assertEqual(first.provenance['node']['operation']['id'], 'face')
        self.assertEqual(first.nodes[0].inputs['colour_intent'], first.provenance)
        for changed in (
            replace(node, sources=None),
            replace(node, depends_on=('upstream',)),
            replace(node, inputs={'drawing_revision': 'B'}),
            replace(node, limits=Limits(faces['f'].region)),
            replace(node, operation=replace(node.operation, feed_mm_min=900)),
        ):
            self.assertNotEqual(first.identity, prepare(changed).identity)
        resolved['faces']['unmapped']['status'] = 'changed'
        self.assertEqual(first.provenance['resolution']['faces']['unmapped']['status'], 'no_colour')

    def test_asymmetric_intent_changes_floor_and_native_verdict(self):
        base = candidate()
        setup = base.job.setups[0]
        entry = host.IntentEntry('finish', (255, 0, 0), 'finish', -0.02, 0.003, 0.1, False)

        def prepare(entry):
            return host.intent_targets(
                OperationNode(setup.operations[0]),
                {'f': host.FaceRegion(region('face', 0, 8), 0.4)},
                resolution((entry,), {'f': entry.rgb}),
                stock=setup.stock,
                global_check=setup.verification.material[0],
                geometry_tolerance_mm=1e-5,
                max_findings=1024,
            )

        result = prepare(entry)
        self.assertEqual(result.nodes[0].operation.floor_z_mm, 0.5)
        self.assertEqual(result.nodes[0].limits.contain.name, 'face')
        self.assertEqual(result.material_checks[1].options.gouge_tolerance_mm, 0.02)
        initial = InitialStock('measured', '1', 'b' * 64, ((0.51, 0.51),) * 256, WorkFrame(), 0)
        exe = os.environ.get('CAMLIB_TEST_ENGINE')
        with Simulation(
            setup.stock,
            setup.tools,
            initial_stock=initial,
            executable=exe,
            revision='1',
            initial_position_mm=setup.initial_position_mm,
        ) as sim:
            check = result.material_checks[1]
            report = Verifier(check.target, check.options).check(sim.state)
            self.assertGreater(report['residual']['definite_cells'], 0)
            tolerant = Verifier(check.target, replace(check.options, residual_tolerance_mm=0.02)).check(sim.state)
            self.assertEqual(tolerant['residual']['assessment'], 'within_tolerance')
        changed = prepare(replace(entry, allowance_mm=0.2))
        self.assertAlmostEqual(changed.nodes[0].operation.floor_z_mm, 0.6)
        self.assertNotEqual(result.identity, changed.identity)

    def test_keepouts_change_geometry_and_overlapping_or_unsupported_faces_refuse(self):
        setup = candidate().job.setups[0]
        finish = host.IntentEntry('finish', (255, 0, 0), 'finish', -0.01, 0.02, 0, False)
        keep = host.IntentEntry('keep', (0, 255, 0), 'preserve', 0, 0, 0, True)
        resolved = resolution((finish, keep), {'a': finish.rgb, 'b': keep.rgb})
        kwargs = dict(
            stock=setup.stock,
            global_check=setup.verification.material[0],
            geometry_tolerance_mm=1e-5,
            max_findings=1024,
        )
        faces = {
            'a': host.FaceRegion(region('a', 0, 4), 0.5),
            'b': host.FaceRegion(region('b', 4, 8), 0.5, keepout_z_mm=(0, 2)),
        }
        result = host.intent_targets(OperationNode(setup.operations[0]), faces, resolved, **kwargs)
        self.assertEqual(len(result.nodes), 1)
        self.assertEqual(len(result.protected_prisms), 1)
        self.assertEqual(result.nodes[0].limits.avoid[0].top_z_mm, 2)
        with self.assertRaisesRegex(ValueError, 'overlap'):
            host.intent_targets(
                OperationNode(setup.operations[0]),
                dict(faces, b=replace(faces['b'], region=region('b', 3, 8))),
                resolved,
                **kwargs,
            )
        with self.assertRaisesRegex(ValueError, 'unsupported'):
            host.intent_targets(
                OperationNode(setup.operations[0]),
                dict(faces, a=replace(faces['a'], upward_normal=(1, 0, 0))),
                resolved,
                **kwargs,
            )

    def test_two_local_floors_share_one_replay_and_each_detects_its_own_overcut(self):
        from camlib.export import material_status
        from camlib.material import Target, Options
        from camlib.motion import Motion, Line
        from camlib.simulation import StockSpec
        from camlib.verification import MaterialCheck, StreamChecks
        from test_simulation import tool

        stock = StockSpec((0, 0, 0), (12, 8, 1), 0.5)
        checks = StreamChecks(
            material=(
                MaterialCheck(Target('full-part', '1', region('whole', -10, 22), 0.5)),
                MaterialCheck(
                    Target('left', '1', region('left', 0, 6), 0.5),
                    options=Options(scope_to_region=True, gouge_tolerance_mm=0.01, residual_tolerance_mm=0.02),
                ),
                MaterialCheck(
                    Target('right', '1', region('right', 6, 12), 0.7),
                    options=Options(scope_to_region=True, gouge_tolerance_mm=0.02, residual_tolerance_mm=0.03),
                ),
            )
        )

        def replay(left, right, initial=None, left_end=4):
            moves = (
                Motion(1, 'left', 't1', Line((2, 4, 3), (2, 4, left)), 200, kind='plunge'),
                Motion(2, 'left', 't1', Line((2, 4, left), (left_end, 4, left)), 600),
                Motion(3, 'link', 't1', Line((left_end, 4, left), (left_end, 4, 3)), kind='rapid'),
                Motion(4, 'link', 't1', Line((left_end, 4, 3), (8, 4, 3)), kind='rapid'),
                Motion(5, 'right', 't1', Line((8, 4, 3), (8, 4, right)), 200, kind='plunge'),
                Motion(6, 'right', 't1', Line((8, 4, right), (10, 4, right)), 600),
            )
            with Simulation(
                stock,
                (tool(),),
                initial_stock=initial,
                revision='1',
                initial_position_mm=(2, 4, 3),
                verification=checks,
                executable=os.environ.get('CAMLIB_TEST_ENGINE'),
            ) as sim:
                list(sim.motions(moves))
                return {u.check['report']['target_id']: u.check for u in sim.finish_updates() if u.kind == 'material'}

        reports = replay(0.5, 0.7)
        self.assertEqual(set(reports), {'full-part', 'left', 'right'})
        for value in reports.values():
            self.assertEqual(material_status(value['report'], value['sweep_proof']), 'clear', value)
        for left, right, target in ((0.4, 0.7, 'left'), (0.5, 0.6, 'right')):
            value = replay(left, right)[target]
            self.assertNotEqual(material_status(value['report'], value['sweep_proof']), 'clear')

        # Radius 2 reaches x=6.5: the low left pass penetrates the higher right
        # region, even though it remains safe for the whole-part low floor.
        crossed = replay(0.5, 0.7, left_end=4.5)
        for target in ('full-part', 'left'):
            value = crossed[target]
            self.assertEqual(material_status(value['report'], value['sweep_proof']), 'clear', value)
        value = crossed['right']
        self.assertNotEqual(material_status(value['report'], value['sweep_proof']), 'clear', value)
        self.assertGreater(
            value['report']['gouge']['possible_only_cells'] + value['report']['gouge']['definite_cells'], 0
        )
        # The identical crossing at/above both floors remains allowed.
        for value in replay(0.7, 0.7, left_end=4.5).values():
            self.assertEqual(material_status(value['report'], value['sweep_proof']), 'clear', value)

    def test_uncertain_initial_blank_does_not_prove_missing_part_existed(self):
        from camlib.material import Target
        from camlib.verification import MaterialCheck, StreamChecks

        setup = candidate().job.setups[0]
        initial = InitialStock('uncertain', '1', 'b' * 64, ((0.2, 0.8),) * 256, WorkFrame(), 0.1)
        checks = StreamChecks(material=(MaterialCheck(Target('part', '1', region('whole', -10, 18), 0.5)),))
        with Simulation(
            setup.stock,
            setup.tools,
            initial_stock=initial,
            revision='1',
            initial_position_mm=setup.initial_position_mm,
            verification=checks,
            executable=os.environ.get('CAMLIB_TEST_ENGINE'),
        ) as sim:
            value = next(u.check for u in sim.finish_updates() if u.kind == 'material')
        self.assertFalse(value['sweep_proof']['history_complete'])
        self.assertGreater(value['report']['gouge']['possible_only_cells'], 0)
        self.assertEqual(value['sweep_proof']['decided_cells'], 0)


if __name__ == '__main__':
    unittest.main()
