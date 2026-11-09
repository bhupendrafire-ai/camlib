"""Declared blank bounds remain authoritative through streams and posted replay."""

from dataclasses import replace
import hashlib
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib.motion import Line, Motion, WorkFrame
from camlib.playback import RecordedRun, Player, Cursor
from camlib.simulation import InitialStock, Simulation, SimulationError, StockSpec, StockState
from camlib.verification import StreamChecks, StockContactOptions
from test_simulation import tool
from test_planning import candidate


def blank():
    return InitialStock(
        'pre-machined',
        '1',
        hashlib.sha256(b'synthetic blank').hexdigest(),
        tuple((0, 0) if x < 20 else (10, 10) for _ in range(40) for x in range(40)),
        WorkFrame(),
        0,
    )


class InitialStockTest(unittest.TestCase):
    def test_empty_columns_are_real_initial_stock_not_a_snapshot_shortcut(self):
        spec = StockSpec((0, 0, 0), (20, 20, 10), 0.5)
        motion = Motion(1, 'traverse', 't1', Line((3, 5, 6), (7, 5, 6)), kind='rapid')
        checks = StreamChecks(stock_contact=StockContactOptions())
        with Simulation(
            spec, (tool(),), initial_stock=blank(), revision='1', initial_position_mm=(3, 5, 6), verification=checks
        ) as sim:
            self.assertEqual(sim.state.volume()['remaining_upper_mm3'], 2000)
            self.assertEqual(sim.state.volume()['removed_upper_mm3'], 0)
            update, report = list(sim.motions((motion,)))
            self.assertEqual(report.status, 'clear')
            self.assertEqual(update.changed_count, 0)
            self.assertEqual(sim.state.volume()['removed_upper_mm3'], 0)
            receiver = StockState(sim.manifest_frame)
            receiver.accept(update.frame)
            self.assertEqual(receiver.heights.tobytes(), sim.state.heights.tobytes())
            sim.finish()
        with Simulation(spec, (tool(),), revision='box', initial_position_mm=(3, 5, 6), verification=checks) as sim:
            _, report = list(sim.motions((motion,)))
            self.assertEqual(report.status, 'violation')
            sim.finish()

    def test_saved_and_fractional_stock_retain_the_blank_and_volume_origin(self):
        spec = StockSpec((0, 0, 0), (20, 20, 10), 0.5)
        motions = (Motion(1, 'cut', 't1', Line((12, 5, 6), (17, 5, 6)), 1000),)
        with TemporaryDirectory() as folder:
            with RecordedRun(
                spec,
                (tool(),),
                motions,
                initial_stock=blank(),
                revision='1',
                initial_position_mm=(12, 5, 6),
                rapid_mm_min=6000,
            ) as run:
                run.wait()
                player = Player(run)
                player.seek(Cursor(0, 0.5))
                halfway = player.state.volume()
                self.assertLess(halfway['remaining_upper_mm3'], 2000)
                self.assertLess(halfway['removed_upper_mm3'], 1000)
                saved = Path(folder) / 'saved'
                run.save(saved)
            with RecordedRun.open(saved) as reopened:
                player = Player(reopened)
                player.seek(Cursor(0))
                self.assertEqual(player.state.volume()['remaining_upper_mm3'], 2000)
                self.assertEqual(player.state.volume()['removed_upper_mm3'], 0)
                player.seek(Cursor(1))
                self.assertGreater(player.state.volume()['removed_lower_mm3'], 0)

    def test_job_keys_and_posted_replay_use_the_same_explicit_blank(self):
        base = candidate()
        setup = base.job.setups[0]
        initial = InitialStock('blank', '1', 'a' * 64, ((0.8, 0.8),) * 256, WorkFrame(), 0)
        setup = replace(setup, initial_stock=initial)
        job = replace(base.job, setups=(setup,))
        compiled = job.compile().setup('top')
        changed = (
            replace(job, setups=(replace(setup, initial_stock=replace(initial, revision='2')),)).compile().setup('top')
        )
        self.assertNotEqual(compiled.dependencies, changed.dependencies)
        self.assertNotEqual(compiled.config['revision'], changed.config['revision'])
        with compiled.post(machine=base.machine) as run:
            self.assertEqual(run.check_release()['accepted'], ())
            player = Player(run)
            player.seek(Cursor(0))
            self.assertAlmostEqual(player.state.volume()['remaining_upper_mm3'], 51.2)
        with self.assertRaisesRegex(ValueError, 'initial stock differs'):
            compiled.post(machine=base.machine, initial_stock=replace(initial, revision='foreign'))

    def test_invalid_or_unsupported_initial_premises_are_refused(self):
        spec = StockSpec((0, 0, 0), (20, 20, 10), 0.5)
        for initial in (
            replace(blank(), source_sha256='not-a-digest'),
            replace(blank(), bounds_mm=((0, 0),)),
            replace(blank(), source_to_setup=WorkFrame(x=(2, 0, 0))),
            replace(blank(), uncertainty_mm=-1),
            replace(blank(), bounds_mm=((11, 10),) * 1600),
        ):
            with self.subTest(initial=initial.source_sha256):
                with self.assertRaises((ValueError, SimulationError)):
                    Simulation(spec, (tool(),), initial_stock=initial, revision='1', initial_position_mm=(3, 5, 6))
        with self.assertRaises(TypeError):
            InitialStock('blank', '1', 'a' * 64, iter(((0, 1),)), WorkFrame(), 0)

    def test_a_blank_missing_required_part_material_cannot_be_released(self):
        base = candidate()
        initial = InitialStock('thin-blank', '1', 'a' * 64, ((0.2, 0.2),) * 256, WorkFrame(), 0)
        job = replace(base.job, setups=(replace(base.job.setups[0], initial_stock=initial),))
        with job.post('top', machine=base.machine) as run:
            with self.assertRaisesRegex(ValueError, 'refused'):
                run.check_release()


if __name__ == '__main__':
    unittest.main()
