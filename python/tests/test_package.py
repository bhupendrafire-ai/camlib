import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest

import camlib

ROOT = Path(__file__).resolve().parents[2]


class PackageTest(unittest.TestCase):
    def test_top_level_names_resolve_lazily_without_cad(self):
        for name in camlib.__all__:
            self.assertTrue(hasattr(camlib, name), name)
        from camlib import jobs

        self.assertIs(camlib.Setup, jobs.Setup)
        with self.assertRaises(AttributeError):
            camlib.not_a_name
        # A fresh interpreter: importing camlib must not pull in CadQuery.
        code = "import sys, camlib; camlib.Pocket; print('cadquery' in sys.modules)"
        out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
        self.assertEqual(out.stdout.strip(), "False")

    def check_example(self, name, accept, *options):
        with TemporaryDirectory(prefix="camlib example ") as folder:
            output = Path(folder) / "pocket.nc"
            command = [sys.executable, str(ROOT / "examples" / name), folder, *options]
            if accept:
                command.append("--accept-unchecked-machine")
            result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, timeout=120)
            if accept:
                self.assertEqual(result.returncode, 0, result.stderr)
                report = json.loads(result.stdout)
                self.assertIn("machine", report["unchecked"])
                self.assertEqual(Path(report["output"]), output)
                self.assertGreater(output.stat().st_size, 0)
                if "--validation" in options:
                    text = output.read_text()
                    for word in ("T7\nM6\nG43 H9", "T8\nM6\nG43 H10", "M8", "\nG2 "):
                        self.assertIn(word, text)
            else:
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("ExportRefused", result.stderr)
                self.assertIn("unchecked_machine", result.stderr)
                self.assertFalse(output.exists())

    def test_standalone_cli_requires_machine_coverage(self):
        self.check_example("standalone_job.py", False)

    def test_standalone_cli_accepts_explicit_inspection_export(self):
        self.check_example("standalone_job.py", True)

    def test_fixed_milling_validation_fixture_requires_machine_coverage(self):
        self.check_example("standalone_job.py", False, "--validation")

    def test_fixed_milling_validation_fixture_exports_for_inspection(self):
        self.check_example("standalone_job.py", True, "--validation")

    def test_fixed_milling_fixture_keeps_the_island_and_cuts_the_floor(self):
        from camlib.playback import Cursor, Player
        from camlib.post import LoweringOptions

        spec = importlib.util.spec_from_file_location("standalone_job", ROOT / "examples/standalone_job.py")
        example = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(example)
        job = example.build_validation_job()
        with job.post("top", machine=example.rest_machine_for(job), lowering=LoweringOptions(native_arcs=True)) as run:
            run.wait()
            player = Player(run)
            player.seek(Cursor(run.status()["available"]))
            stock = player.state
            # Independent analytic controls: island centre, pocket floor, faced top.
            for x, y, expected in ((16, 12, 5), (8, 8, 3), (2, 2, 5)):
                i = int((x - stock.min_mm[0]) / stock.dx)
                j = int((y - stock.min_mm[1]) / stock.dy)
                low, high = stock.heights[2 * (j * stock.nx + i) : 2 * (j * stock.nx + i) + 2]
                self.assertLessEqual(abs(low - expected), 2e-5)
                self.assertLessEqual(abs(high - expected), 2e-5)
            progress = player.verification["progress"]
            self.assertEqual(progress["violation"], 0)
            self.assertEqual(progress["uncertain"], 0)
            self.assertEqual(progress["clear"], stock.completed)

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "CadQuery is optional")
    def test_cadquery_cli_requires_machine_coverage(self):
        self.check_example("cadquery_job.py", False)

    @unittest.skipUnless(importlib.util.find_spec("cadquery"), "CadQuery is optional")
    def test_cadquery_cli_accepts_explicit_inspection_export(self):
        self.check_example("cadquery_job.py", True)


if __name__ == "__main__":
    unittest.main()
