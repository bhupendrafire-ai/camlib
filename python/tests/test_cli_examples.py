"""Run the documented entry points as users do, including expected refusals."""

import os
from pathlib import Path
import re
import subprocess
import sys
from tempfile import TemporaryDirectory
import unittest


ROOT = Path(__file__).resolve().parents[2]


def run_example(arguments, directory):
    env = dict(os.environ)
    if env.get("PYTHONPATH"):
        env["PYTHONPATH"] = os.pathsep.join(str(Path(p).resolve()) for p in env["PYTHONPATH"].split(os.pathsep))
    return subprocess.run(
        [sys.executable, *arguments], cwd=directory, env=env, text=True, capture_output=True, timeout=120
    )


class CliExamplesTest(unittest.TestCase):
    def test_export_refusal_is_readable_and_never_writes_or_replaces_nc(self):
        for existing in (False, True):
            with self.subTest(existing=existing), TemporaryDirectory() as directory:
                output = Path(directory) / "output"
                output.mkdir()
                nc = output / "pocket.nc"
                sentinel = b"Owner's existing NC must remain unchanged.\n"
                if existing:
                    nc.write_bytes(sentinel)
                result = run_example([str(ROOT / "examples/standalone_job.py"), str(output)], directory)
                self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
                self.assertIn("unchecked_machine", result.stderr)
                self.assertIn("No NC file was written or replaced.", result.stderr)
                self.assertNotIn("Traceback", result.stderr)
                if existing:
                    self.assertEqual(nc.read_bytes(), sentinel)
                else:
                    self.assertFalse(nc.exists())

    def test_complete_getting_started_example_handles_its_declared_machine_refusal(self):
        guide = (ROOT / "docs/getting-started.md").read_text()
        example = re.search(r"```python\n(.*?)\n```", guide, flags=re.DOTALL).group(1)
        with TemporaryDirectory() as directory:
            result = run_example(["-c", example], directory)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            self.assertIn("lines of G-code", result.stdout)
            self.assertIn("unchecked_machine", result.stdout)
            self.assertIn("No NC file was written or replaced.", result.stdout)
            self.assertNotIn("Traceback", result.stderr)
            self.assertFalse((Path(directory) / "top.nc").exists())


if __name__ == "__main__":
    unittest.main()
