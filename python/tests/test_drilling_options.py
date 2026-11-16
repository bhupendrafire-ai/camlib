"""Actual R-plane/peck paths, independent checks and decoded posted output."""

from dataclasses import replace
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from camlib import PeckSchedule
from camlib.export import ExportRefused
from camlib.planar import Boundary, Component, PlanarRegion
from camlib.postprocessor import GCodePost, GCodeDecoder, NativeGCodeDecoder, GCodeDialect
from camlib.simulation import Simulation, SimulationError, OperationUpdate, VerificationUpdate
from camlib.verification import Scene, Prism, Options, MachineLimits
from test_drill_cycles import job, linuxcnc
from test_tapping import posted_text


def fixture(*, obstacle=False, uncertain=False, **options):
    source = job(**options)
    setup = source.setups[0]
    bodies = ()
    if obstacle or uncertain:
        y = 7 if obstacle else 9.9
        footprint = PlanarRegion(
            "fixture", (Component(Boundary.polygon(((13, y), (15, y), (15, y + 2), (13, y + 2)))),)
        )
        bodies = (Prism("bridge", footprint, 10.8, 11.5),)
    checks = replace(
        setup.verification,
        scene=Scene(
            "declared-drilling-scene", "1", bodies, machine=MachineLimits((-100, -100, -100), (100, 100, 100), 10000)
        ),
        options=Options(max_nodes=1, max_boundary_tests=1) if uncertain else Options(),
    )
    return replace(source, setups=(replace(setup, verification=checks),))


def run(source):
    moves, checks, report = [], {}, None
    with Simulation(**source.compile().setup("top").simulation_options()) as sim:
        for update in sim.run_operations():
            if isinstance(update, OperationUpdate) and update.stage == "end":
                report = update.report
            elif isinstance(update, VerificationUpdate) and update.kind == "motion":
                checks[update.completed] = update.check
            elif hasattr(update, "motion"):
                moves.append(update.motion)
        assert sim.state.finished
        return moves, sim.state.heights.tobytes(), report, checks


class DrillingOptionsTest(unittest.TestCase):
    def test_r_plane_preserves_plunges_stock_and_final_clearance_with_real_proofs(self):
        for canned in (False, True):
            for pecks in ({}, {"peck_mm": 1.5}, {"peck_mm": 1.5, "peck_mode": "chip_break"}):
                with self.subTest(canned=canned, **pecks):
                    old, old_stock, _, _ = run(fixture(canned_cycle=canned, **pecks))
                    new, stock, report, checks = run(fixture(canned_cycle=canned, r_plane_travel=True, **pecks))
                    self.assertEqual(old_stock, stock)
                    self.assertEqual(old[-1]["curve"]["end"], new[-1]["curve"]["end"])
                    plunges = lambda rows: sorted(
                        (tuple(m["curve"]["start"]), tuple(m["curve"]["end"]), m["feed_mm_min"])
                        for m in rows
                        if m["kind"] == "plunge"
                    )
                    self.assertEqual(plunges(old), plunges(new))
                    low = [
                        m
                        for m in new
                        if m["kind"] == "rapid"
                        and m["curve"]["start"][:2] != m["curve"]["end"][:2]
                        and m["curve"]["start"][2] == 11
                    ]
                    self.assertEqual(len(low), 1)
                    self.assertEqual(report["r_plane_travel"]["candidate_links"], 1)
                    for m in low:
                        proof = checks[m["id"]]
                        self.assertEqual(proof["report"]["status"], "clear")
                        self.assertEqual(proof["stock_contact"]["status"], "clear")

    def test_g99_is_written_and_independently_decoded_by_native_and_python_posts(self):
        for options in ({}, {"peck_mm": 1.5}, {"peck_mm": 1.5, "peck_mode": "chip_break"}, {"dwell_s": 0.5}):
            source = fixture(r_plane_travel=True, **options)
            machine = linuxcnc(source)
            for post, decoder in ((None, None), (GCodePost, GCodeDecoder)):
                with self.subTest(options=options, post=type(post).__name__):
                    nc = posted_text(source, machine, post, decoder)
                    self.assertEqual(nc.count("G99"), 1)
                    self.assertIn("G0 X20 Y8 Z12", nc)
            with source.post("top", machine=machine) as posted:
                posted.wait()
                self.assertEqual(posted.posting_status()["checked_motions"], posted.status()["available"])
                with TemporaryDirectory(prefix="drilling options export ") as folder:
                    target = Path(folder) / "holes.nc"
                    with self.assertRaises(ExportRefused):
                        posted.export(target)
                    self.assertFalse(target.exists())

    def test_wrong_return_word_is_refused_by_the_independent_decoder(self):
        class WrongReturn(GCodePost):
            def emit(self, *words):
                return super().emit(*("G98" if word == "G99" else word for word in words))

        source = fixture(r_plane_travel=True)
        machine = linuxcnc(source)
        with self.assertRaisesRegex(SimulationError, "canned cycle moves still pending"):
            posted_text(source, machine, WrongReturn, GCodeDecoder)

    def test_a_dialect_without_g99_writes_the_same_checked_path_explicitly(self):
        source = fixture(r_plane_travel=True, peck_mm=1.5)
        nc = posted_text(source, linuxcnc(source, GCodeDialect()))
        self.assertNotIn("G99", nc)
        self.assertFalse(any(line.startswith("G83 ") for line in nc))
        self.assertTrue(any(line.startswith("G0 ") for line in nc))

    def test_g99_then_g98_keeps_the_series_initial_plane_until_g80(self):
        source = fixture(r_plane_travel=True)
        machine = linuxcnc(source)
        header = source.compile().setup("top").program_header()
        nc = posted_text(source, machine)
        for cls in (GCodeDecoder, NativeGCodeDecoder):
            decoder = cls(machine, header)
            decoder.read(nc[: nc.index("G99")])
            for mode, x, expected_z in (("G99", 8, 11), ("G98", 20, 12), ("G99", 8, 11)):
                decoder.read([mode, f"G81 X{x} Y8 Z4 R11 F150"])
                moves = []
                while decoder.cycle_queue:
                    rapid = decoder.cycle_queue[0][2]
                    moves.extend(
                        decoder.read(
                            [], {"id": len(moves) + 1, "operation": "holes", "kind": "rapid" if rapid else "plunge"}
                        )
                    )
                self.assertEqual(decoder.position, (x, 8, expected_z))
            decoder.read(["G80"])

    def test_obstructed_uncertain_and_unconfigured_r_plane_travel_are_refused(self):
        for options in ({"obstacle": True}, {"uncertain": True}):
            with self.subTest(**options):
                with self.assertRaisesRegex(SimulationError, "R-plane.*not proven Clear"):
                    run(fixture(r_plane_travel=True, **options))
        run(fixture(obstacle=True))  # The original clearance route remains usable.
        with self.assertRaisesRegex(SimulationError, "R-plane.*both scene and stock"):
            run(job(r_plane_travel=True))

    def test_decreasing_pecks_stop_at_target_keep_stock_and_report_the_declared_schedule(self):
        for mode in ("full_retract", "chip_break"):
            with self.subTest(mode=mode):
                _, old_stock, _, _ = run(fixture(canned_cycle=False, peck_mm=1, peck_mode=mode))
                source = fixture(canned_cycle=False, peck_schedule=PeckSchedule(3, 1, 1, 10), peck_mode=mode)
                moves, stock, report, _ = run(source)
                self.assertEqual(stock, old_stock)
                self.assertEqual([m["curve"]["end"][2] for m in moves if m["kind"] == "plunge"], [7, 5, 4] * 2)
                self.assertEqual(report["pecks_per_hole"], 3)
                self.assertEqual(report["peck_returns"], 4)
                self.assertEqual(report["peck_schedule"]["minimum_mm"], 1)
                posted_text(source, linuxcnc(source))
        moves, _, _, _ = run(fixture(canned_cycle=False, peck_schedule=PeckSchedule(4, 1, 3, 10)))
        self.assertEqual([m["curve"]["end"][2] for m in moves if m["kind"] == "plunge"], [6, 4] * 2)

    def test_invalid_schedules_and_exhausted_budgets_refuse_without_motion(self):
        for schedule in (
            PeckSchedule(1, 0, 1, 2),
            PeckSchedule(1, -1, 1, 10),
            PeckSchedule(1, 0, 2, 10),
            PeckSchedule(float("nan"), 0, 1, 10),
        ):
            with self.subTest(schedule=schedule):
                with self.assertRaises((SimulationError, ValueError)):
                    run(fixture(canned_cycle=False, peck_schedule=schedule))
        with self.assertRaisesRegex(SimulationError, "explicit drilling"):
            run(fixture(peck_schedule=PeckSchedule(3, 1, 1)))
        self.assertNotIn("r_plane_travel", job().setups[0].operations[0].operation.to_dict())
        self.assertNotIn("peck_schedule", job().setups[0].operations[0].operation.to_dict())
