"""Geometry proposals stay conservative, bounded and tied to explicit confirmation."""

from dataclasses import replace
import json
import os
import unittest
from unittest.mock import patch

from camlib.motion import WorkFrame
from camlib.preparation import (
    ReachSample,
    reach_map,
    enclose_blank,
    blank_placements,
    confirm_blank,
    confirmed_initial_stock,
)
from camlib.simulation import InitialStock, StockSpec
from camlib.surface import SurfaceMesh
from camlib.verification import Scene, MachineLimits, Prism
from camlib.tools import HolderSection
from test_post import fixture


def surface():
    return SurfaceMesh((((0.2, 0.2, 1), (2.8, 0.2, 2), (2.8, 2.8, 2)), ((0.2, 0.2, 1), (2.8, 2.8, 2), (0.2, 2.8, 1))))


def declaration():
    return InitialStock('scan', '1', 'a' * 64, (), WorkFrame(), 0)


class PreparationTest(unittest.TestCase):
    def test_native_blank_bounds_and_saved_confirmation_are_recomputed(self):
        stock = StockSpec((0, 0, 0), (3, 3, 3), 1.1)
        blank = enclose_blank(
            surface(), stock, declaration(), tolerance_mm=1e-6, max_work=100000, source_enclosure='top_down_outer'
        )
        self.assertEqual(len(blank.bounds_mm), 9)
        self.assertGreater(blank.bounds_mm[4][1] - blank.bounds_mm[4][0], 0.38)
        report = blank_placements(
            surface(),
            (WorkFrame(),),
            ((0.2, 0.2, 1), (2.8, 2.8, 2)),
            stock,
            declaration(),
            tolerance_mm=1e-6,
            max_work=100000,
            max_candidates=4,
            ambiguity_gap_mm=0.01,
            source_enclosure='top_down_outer',
        )
        proposal = report['proposals'][0]
        self.assertEqual(proposal['score_mm'], 0)
        confirmation = confirm_blank(proposal, who='test engineer')
        proposal, confirmation = json.loads(json.dumps((proposal, confirmation)))
        self.assertEqual(confirmed_initial_stock(proposal, confirmation).bounds_mm, blank.bounds_mm)
        for invalid in (True, 123, None, '', '   ', 'x' * 513):
            with self.subTest(who=repr(invalid)[:30]), self.assertRaisesRegex(ValueError, 'engineer identity'):
                confirm_blank(proposal, who=invalid)
            tampered = json.loads(json.dumps(dict(confirmation, who=invalid)))
            with self.subTest(saved_who=repr(invalid)[:30]), self.assertRaisesRegex(ValueError, 'engineer identity'):
                confirmed_initial_stock(proposal, tampered)
        proposal['premise']['declaration']['revision'] = 'weld-2'
        with self.assertRaisesRegex(ValueError, 'stale'):
            confirmed_initial_stock(proposal, confirmation)

    def test_ambiguous_upright_fit_never_selects_automatically(self):
        orientations = (WorkFrame(), WorkFrame(x=(0, 1, 0), y=(-1, 0, 0)))
        report = blank_placements(
            surface(),
            orientations,
            ((0.2, 0.2, 1), (2.8, 2.8, 2)),
            StockSpec((0, 0, 0), (3, 3, 3), 1),
            declaration(),
            tolerance_mm=1e-6,
            max_work=100000,
            max_candidates=2,
            ambiguity_gap_mm=0.01,
            source_enclosure='top_down_outer',
        )
        self.assertTrue(report['ambiguous'])
        self.assertIsNone(report['selected'])
        self.assertEqual(len(report['proposals']), 2)

    def test_reach_and_tested_projection_have_native_identity(self):
        program, _ = fixture()
        plane = SurfaceMesh((((-20, -20, 1), (20, -20, 1), (20, 20, 1)), ((-20, -20, 1), (20, 20, 1), (-20, 20, 1))))
        report = reach_map(
            plane,
            (ReachSample('p', (0, 0, 1), (0, 0, 1)),),
            (('mill', program.tools[0]),),
            base_z_mm=0,
            scene=Scene('fixtures', '1', machine=MachineLimits((-100,) * 3, (100,) * 3, 10000)),
            clearance_z_mm=30,
            tolerance_mm=0.02,
            max_pairs=2,
            max_work=100000,
        )
        self.assertTrue(report['complete'])
        self.assertEqual(report['rows'][0]['status'], 'clear')
        self.assertEqual(report['selected'][0]['tool_id'], program.tools[0].id)
        self.assertEqual(len(report['input_sha256']), 64)

    def reach_options(self):
        program, _ = fixture()
        tool = replace(
            program.tools[0],
            cutter=replace(program.tools[0].cutter, cutting_length_mm=20),
            stickout_mm=30,
            holders=(HolderSection(30, 60, 4, 4),),
        )
        return dict(
            surface=SurfaceMesh(
                (((-20, -20, 1), (20, -20, 1), (20, 20, 1)), ((-20, -20, 1), (20, 20, 1), (-20, 20, 1)))
            ),
            samples=(ReachSample('p', (0, 0, 1), (0, 0, 1)),),
            tools=(('mill', tool),),
            base_z_mm=0,
            scene=Scene('fixtures', '1', machine=MachineLimits((-100,) * 3, (100,) * 3, 10000)),
            clearance_z_mm=30,
            tolerance_mm=0.02,
            max_pairs=10,
            max_work=100000,
        )

    def test_native_reach_refuses_mesh_metadata_base_and_degenerate_facets(self):
        class WireSurface:
            def __init__(self, wire):
                self.wire = wire

            def to_dict(self):
                return self.wire

        for field, value in (
            ('version', 2),
            ('units', 'inch'),
            ('surface_error_mm', -0.01),
            ('triangles', (((0, 0, 1),) * 3,)),
        ):
            with self.subTest(field=field):
                options = self.reach_options()
                wire = options['surface'].to_dict()
                wire[field] = value
                options['surface'] = WireSurface(wire)
                with self.assertRaises(ValueError):
                    reach_map(**options)
        for base in (-1000001, 2):
            with self.subTest(base=base), self.assertRaises(ValueError):
                reach_map(**dict(self.reach_options(), base_z_mm=base))

    def test_flat_ball_and_bull_reach_match_analytic_inclined_contacts(self):
        for kind, corner_radius in (('flat', 0), ('ball', 2), ('bull', 0.5)):
            with self.subTest(kind=kind):
                options = self.reach_options()
                options['surface'] = SurfaceMesh(
                    (((-20, -20, 5), (20, -20, 35), (20, 20, 35)), ((-20, -20, 5), (20, 20, 35), (-20, 20, 5)))
                )
                options['samples'] = (ReachSample('p', (0, 0, 20), (-0.6, 0, 0.8)),)
                shape = dict(kind=kind)
                if kind == 'bull':
                    shape['corner_radius_mm'] = corner_radius
                tool = options['tools'][0][1]
                options['tools'] = (('mill', replace(tool, cutter=replace(tool.cutter, shape=tuple(shape.items())))),)
                report = reach_map(**options)
                self.assertTrue(report['complete'])
                self.assertEqual(report['rows'][0]['status'], 'clear', report)
                tip = report['rows'][0]['tip_mm']
                # Closed-form support of a rounded disk on z = 20 + 0.75*x.
                exact = 20 + 0.75 * tip[0] + 0.75 * (2 - corner_radius) + 0.25 * corner_radius
                self.assertAlmostEqual(tip[2], exact, places=12)
                self.assertAlmostEqual(tip[0], -(2 - corner_radius) - 0.6 * corner_radius, places=12)

    def test_holder_obstruction_and_calibrated_projection_ties(self):
        options = self.reach_options()
        tool = options['tools'][0][1]
        variants = tuple(
            ('mill', replace(tool, id=id, stickout_mm=stickout, holders=(HolderSection(stickout, 60, 4, 4),)))
            for id, stickout in (('z-long', 40), ('b-short', 25), ('a-short', 25), ('short-blocked', 20))
        )
        footprint = dict(
            version=1,
            units='mm',
            name='clamp',
            boundary_error_mm=0,
            components=[
                dict(outer=dict(vertices=((3, -0.5, 0), (3.5, -0.5, 0), (3.5, 0.5, 0), (3, 0.5, 0))), holes=[])
            ],
        )
        options['tools'] = variants
        options['scene'] = replace(options['scene'], bodies=(Prism('clamp', footprint, 21.5, 22),))
        report = reach_map(**options)
        self.assertTrue(report['complete'])
        self.assertEqual(len(report['rows']), 4)
        self.assertEqual(report['rows'][0]['status'], 'obstructed', report)
        self.assertIn('Holder', str(report['rows'][0]['reasons']))
        self.assertEqual(report['selected'][0]['tool_id'], 'a-short')

    def test_index_memory_and_preparation_work_are_bounded(self):
        options = self.reach_options()
        options['surface'] = SurfaceMesh((((-1e6, -1e6, 1), (1e6, -1e6, 1), (1e6, 1e6, 1)),) * 4096)
        options['max_work'] = 1000000000
        with patch.dict(os.environ, CAMLIB_MEMORY_BUDGET_BYTES='1048576'):
            with self.assertRaisesRegex(ValueError, 'triangle index needs'):
                reach_map(**options)
        options['max_work'] = 3 * 4096
        report = reach_map(**options)
        self.assertFalse(report['complete'])
        self.assertEqual(len(report['rows']), 1)
        self.assertEqual(report['rows'][0]['status'], 'unresolved')
        self.assertEqual(report['selected'], [])

    def test_curved_preparation_and_queries_share_one_work_budget(self):
        options = self.reach_options()
        tool = options['tools'][0][1]
        tool = replace(tool, cutter=replace(tool.cutter, shape=(('kind', 'ball'),)))
        options['tools'] = (('mill', tool), ('mill', replace(tool, id='t2')))
        options['max_work'] = 6000
        partial = reach_map(**options)
        self.assertFalse(partial['complete'])
        self.assertEqual(len(partial['rows']), 2)
        self.assertEqual(partial['selected'], [])
        options['max_work'] = 10000
        complete = reach_map(**options)
        self.assertTrue(complete['complete'])
        self.assertTrue(all(row['status'] == 'clear' for row in complete['rows']))

    def test_blank_preparation_honors_work_and_memory_before_building_the_bvh(self):
        large = SurfaceMesh((surface().triangles[0],) * 4096)
        options = dict(
            surface=large,
            stock=StockSpec((0, 0, 0), (3, 3, 3), 1.1),
            declaration=declaration(),
            tolerance_mm=1e-6,
            max_work=1,
            source_enclosure='top_down_outer',
        )
        with self.assertRaisesRegex(ValueError, 'work limit'):
            enclose_blank(**options)
        options['max_work'] = 1000000000
        with patch.dict(os.environ, CAMLIB_MEMORY_BUDGET_BYTES='1024'):
            with self.assertRaisesRegex(ValueError, 'surface preparation needs'):
                enclose_blank(**options)

    def test_placement_error_cannot_cancel_positive_mesh_error(self):
        mesh = replace(surface(), surface_error_mm=0.1)
        for uncertainty in (-0.05, -0.1):
            with self.subTest(uncertainty=uncertainty), self.assertRaisesRegex(ValueError, 'initial stock'):
                enclose_blank(
                    mesh,
                    StockSpec((0, 0, 0), (3, 3, 3), 1.1),
                    replace(declaration(), uncertainty_mm=uncertainty),
                    tolerance_mm=1e-6,
                    max_work=100000,
                    source_enclosure='top_down_outer',
                )
        stock = StockSpec((-2, -2, -2), (5, 5, 5), 1.1)
        blank = enclose_blank(
            mesh,
            stock,
            replace(declaration(), uncertainty_mm=0.9),
            tolerance_mm=1e-6,
            max_work=100000,
            source_enclosure='top_down_outer',
        )
        self.assertEqual(blank.uncertainty_mm, 1)
        with self.assertRaisesRegex(ValueError, 'combined finite error'):
            enclose_blank(
                mesh,
                stock,
                replace(declaration(), uncertainty_mm=0.91),
                tolerance_mm=1e-6,
                max_work=100000,
                source_enclosure='top_down_outer',
            )

    def test_invalid_metadata_and_supplied_bounds_are_refused_before_work(self):
        stock = StockSpec((0, 0, 0), (3, 3, 3), 1.1)
        for field, value in (('source_id', 's' * 129), ('revision', 'r' * 513), ('source_sha256', 'invalid')):
            with self.subTest(field=field), patch.dict(os.environ, CAMLIB_MEMORY_BUDGET_BYTES='1'):
                with self.assertRaisesRegex(ValueError, 'initial stock'):
                    enclose_blank(
                        surface(),
                        stock,
                        replace(declaration(), **{field: value}),
                        tolerance_mm=1e-6,
                        max_work=1,
                        source_enclosure='top_down_outer',
                    )
        supplied = replace(declaration(), bounds_mm=((0, 1),) * 10000)
        with patch.dict(os.environ, CAMLIB_MEMORY_BUDGET_BYTES='1'):
            with self.assertRaisesRegex(ValueError, 'empty bounds'):
                enclose_blank(
                    surface(),
                    stock,
                    supplied,
                    tolerance_mm=1e-6,
                    max_work=1,
                    source_enclosure='top_down_outer',
                )
        with patch.dict(os.environ, CAMLIB_MEMORY_BUDGET_BYTES='144'):
            with self.assertRaisesRegex(ValueError, 'blank enclosure bounds needs'):
                enclose_blank(
                    surface(),
                    stock,
                    replace(declaration(), revision='r' * 512),
                    tolerance_mm=1e-6,
                    max_work=100000,
                    source_enclosure='top_down_outer',
                )

    def test_enclosure_refuses_unsupported_source_and_outside_material(self):
        for premise in ('closed_mesh', 'top_down_outer'):
            with self.assertRaises(ValueError):
                enclose_blank(
                    surface(),
                    StockSpec((0, 0, 0), (2, 2, 2), 1),
                    declaration(),
                    tolerance_mm=1e-6,
                    max_work=100000,
                    source_enclosure=premise,
                )


if __name__ == '__main__':
    unittest.main()
