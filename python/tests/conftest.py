"""Shared pytest fixtures for the cybercadkernel binding.

Guarantees, in order, that:

1. the shared library (``build-mac/libcybercadkernel.dylib``) can be *loaded* —
   if not, every test is skipped with an actionable message pointing at the
   proven build recipe rather than erroring out with a raw ``OSError``;
2. the loaded build links a *real* B-rep engine (``cc_brep_available() == 1``) —
   the desktop OCCT build, not the host stub. Geometry tests assert exact
   volumes/areas, which are meaningless on the stub, so they skip loudly there.

The ``kernel`` fixture is session-scoped: the ``CDLL`` is a process singleton
(see ``_cffi.lib``), so there is nothing to tear down between tests.

Profile note: the pythonic ``Kernel.extrude`` takes an iterable of ``(x, y)``
*pairs* (see ``_flatten_xy`` in ``cybercadkernel.api``); the ``box`` helper below
builds a rectangle in that shape so tests never hand-roll it.
"""

import os
import subprocess
import sys

import pytest

# python/ is a sibling of build-mac/ under the repo root.
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, os.pardir, os.pardir))
_BUILD_SCRIPT = os.path.join(_REPO_ROOT, "python", "build_dylib.sh")
_DEFAULT_DYLIB = os.path.join(_REPO_ROOT, "build-mac", "libcybercadkernel.dylib")


def _load_or_skip_reason():
    """Try to load the library; return ``None`` on success or a skip message.

    If the dylib is absent we make ONE attempt to build it with the proven
    recipe (``python/build_dylib.sh``) when running on macOS with Homebrew
    OCCT present; otherwise we return a clear, actionable skip reason.
    """
    from cybercadkernel import _cffi

    def _try_load():
        try:
            _cffi.lib()
            return None
        except _cffi.KernelLibraryNotFound as exc:
            return str(exc)
        except OSError as exc:  # dylib present but its deps (OCCT) won't resolve
            return f"could not load libcybercadkernel: {exc}"

    reason = _try_load()
    if reason is None:
        return None

    # Attempt the codified build once, only if we can plausibly succeed.
    env_dylib = os.environ.get("CYBERCADKERNEL_DYLIB")
    already_built = (
        os.path.exists(env_dylib) if env_dylib else os.path.exists(_DEFAULT_DYLIB)
    )
    homebrew_occt = os.path.isdir("/opt/homebrew/opt/opencascade/include/opencascade")
    if (
        not already_built
        and sys.platform == "darwin"
        and homebrew_occt
        and os.path.exists(_BUILD_SCRIPT)
    ):
        try:
            subprocess.run(
                ["bash", _BUILD_SCRIPT],
                cwd=_REPO_ROOT,
                check=True,
                capture_output=True,
                timeout=1800,
            )
        except Exception as exc:  # build failed / timed out — skip, don't error
            return f"libcybercadkernel not built and auto-build failed: {exc}"
        _cffi._lib = None  # drop the cached handle so the rebuild is picked up
        return _try_load()

    return (
        reason
        + "\n\nBuild it with the proven recipe:\n"
        + f"    bash {_BUILD_SCRIPT}\n"
        + "or point CYBERCADKERNEL_DYLIB at an existing libcybercadkernel.dylib."
    )


@pytest.fixture(scope="session")
def kernel():
    """A loaded :class:`~cybercadkernel.Kernel`, or skip the whole session.

    Constructed with ``require_brep=False`` so a stub build still yields a usable
    object; the real-engine gate below (``_require_real_engine``) does the skip.
    """
    reason = _load_or_skip_reason()
    if reason is not None:
        pytest.skip(reason)
    from cybercadkernel import Kernel

    return Kernel(require_brep=False)


@pytest.fixture(autouse=True)
def _require_real_engine(kernel):
    """Gate every test on the real B-rep engine.

    Skip loudly rather than let an assertion on exact geometry pass trivially
    (or crash) on the stub build.
    """
    if not kernel.brep_available:
        pytest.skip("cc_brep_available()==0: no B-rep engine linked (stub build)")


@pytest.fixture
def box(kernel):
    """Factory for an axis-aligned box with a corner at the origin.

    Returns a callable ``make(dx, dy, dz) -> Shape`` and closes every shape it
    hands out at teardown, so tests never leak handles.
    """
    made = []

    def make(dx, dy, dz):
        rect = [(0.0, 0.0), (dx, 0.0), (dx, dy), (0.0, dy)]
        shape = kernel.extrude(rect, dz)
        made.append(shape)
        return shape

    yield make
    for s in made:
        try:
            s.close()
        except Exception:
            pass


@pytest.fixture(scope="session")
def trimesh_or_skip():
    """Import ``trimesh`` or skip. Isolated here so its (NumPy/shapely ABI)
    import quirks are handled in one place, not scattered across tests."""
    return pytest.importorskip("trimesh")
