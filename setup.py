"""Build the native engine and its Python module into one installable wheel."""
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext

ROOT = Path(__file__).resolve().parent


class CargoBuild(build_ext):
    def build_extension(self, ext):
        env = dict(os.environ, PYO3_PYTHON=sys.executable)
        # Rust embeds source locations in panic messages even in stripped builds.
        # Keep build-host paths out of distributed native artifacts.
        flags = (env["CARGO_ENCODED_RUSTFLAGS"].split("\x1f")
                 if env.get("CARGO_ENCODED_RUSTFLAGS")
                 else shlex.split(env.get("RUSTFLAGS", "")))
        for path, label in ((Path.home(), "build"),
                            (Path(env.get("CARGO_HOME", Path.home() / ".cargo")), "cargo"),
                            (Path(env.get("RUSTUP_HOME", Path.home() / ".rustup")), "rustup"),
                            (ROOT, "camlib")):
            flags.append(f"--remap-path-prefix={path.resolve()}={label}")
        env["CARGO_ENCODED_RUSTFLAGS"] = "\x1f".join(flags)
        # Match the checked release feature set. Cargo reuses unchanged artifacts.
        command = ["cargo", "build", "--locked", "--release", "--workspace",
                   "--all-features", "--lib",
                   "--bin", "cam-sim-stream", "--message-format=json"]
        result = subprocess.run(command, cwd=ROOT, env=env, check=False,
                                stdout=subprocess.PIPE, text=True)
        artifacts = {}
        for line in result.stdout.splitlines():
            record = json.loads(line)
            if record.get("reason") == "compiler-message":
                rendered = record.get("message", {}).get("rendered")
                if rendered:
                    sys.stderr.write(rendered)
            if record.get("reason") != "compiler-artifact":
                continue
            name = record["target"]["name"]
            if name == "_native":
                artifacts[name] = next(Path(p) for p in record["filenames"]
                                       if Path(p).suffix in (".so", ".dylib", ".dll"))
            elif name == "cam-sim-stream":
                artifacts[name] = Path(record["executable"])
        if result.returncode:
            raise RuntimeError(f"Cargo build failed (exit {result.returncode}); see the compiler diagnostics above")
        if set(artifacts) != {"_native", "cam-sim-stream"}:
            raise RuntimeError("Cargo did not produce the native CAM module and its stream driver")
        destination = Path(self.get_ext_fullpath(ext.name))
        if sys.platform != "win32":
            # Every CPython 3 on Linux/macOS loads '<name>.abi3.so'. Some setuptools
            # versions emit a multiarch '.abi3-<triple>.so' that CPython 3.12 does
            # not search for, which installs a wheel that cannot import camlib.
            destination = destination.with_name(ext.name.rpartition(".")[2] + ".abi3.so")
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(artifacts["_native"], destination)
        for name in ("cam-sim-stream",):
            target = destination.parent / "_bin" / artifacts[name].name
            target.parent.mkdir(exist_ok=True)
            shutil.copy2(artifacts[name], target)
            target.chmod(target.stat().st_mode | 0o111)



setup(ext_modules=[Extension("camlib._native", sources=[], py_limited_api=True)],
      cmdclass={"build_ext": CargoBuild},
      options={"bdist_wheel": {"py_limited_api": "cp312"}})
