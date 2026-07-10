"""Wheel packaging for the CyberCadKernel Python binding.

The project metadata lives in ``pyproject.toml``; this ``setup.py`` exists only
to BUNDLE the prebuilt native shared library into the wheel so ``pip install .``
produces a self-contained, importable package (no separate ``build-mac`` needed
at runtime).

At build time it copies the dylib into ``cybercadkernel/lib/`` (declared as
package data). The source of the dylib is, in order:

  1. ``$CYBERCADKERNEL_DYLIB`` (an explicit path), else
  2. ``build-mac/libcybercadkernel.dylib`` (the proven macOS OCCT build), else
  3. ``build/<lib>`` (a generic build dir).

If none exists the build still succeeds but ships NO dylib; the installed
package then falls back to the ``CYBERCADKERNEL_DYLIB`` env var / a
``build-mac`` checkout at runtime (see ``_cffi._candidate_paths``). Build the
dylib first with ``python/build_dylib.sh`` for a fully self-contained wheel.
"""

from __future__ import annotations

import os
import shutil
import sys

from setuptools import setup
from setuptools.command.build_py import build_py

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, os.pardir))
_PKG_LIB_DIR = os.path.join(_HERE, "cybercadkernel", "lib")

_DYLIB_NAME = {
    "darwin": "libcybercadkernel.dylib",
    "linux": "libcybercadkernel.so",
}.get(sys.platform, "libcybercadkernel.so")


def _find_prebuilt_dylib() -> str | None:
    env = os.environ.get("CYBERCADKERNEL_DYLIB")
    candidates = []
    if env:
        candidates.append(env)
    candidates.append(os.path.join(_REPO_ROOT, "build-mac", _DYLIB_NAME))
    candidates.append(os.path.join(_REPO_ROOT, "build", _DYLIB_NAME))
    for path in candidates:
        if path and os.path.isfile(path):
            return path
    return None


class BuildPyWithDylib(build_py):
    """Copy the prebuilt native dylib into the package before packaging."""

    def run(self) -> None:
        src = _find_prebuilt_dylib()
        if src:
            os.makedirs(_PKG_LIB_DIR, exist_ok=True)
            dst = os.path.join(_PKG_LIB_DIR, os.path.basename(src))
            shutil.copy2(src, dst)
            self.announce(f"bundling native library: {src} -> {dst}", level=2)
        else:
            self.announce(
                "no prebuilt libcybercadkernel found; the wheel will ship "
                "WITHOUT the dylib (set CYBERCADKERNEL_DYLIB or run "
                "python/build_dylib.sh, then rebuild for a self-contained wheel)",
                level=3,
            )
        super().run()


setup(
    cmdclass={"build_py": BuildPyWithDylib},
    package_data={"cybercadkernel": ["lib/*.dylib", "lib/*.so"]},
)
