"""Shared gallery helper for the CyberCadKernel example pieces.

Each example script builds a :class:`cybercadkernel.Shape` and hands it to
:func:`emit`, which:

* validates the solid (positive volume, finite bbox);
* writes STEP + STL (native writer) and, when trimesh can serialize it, a glTF
  (``.glb``) into ``examples/out/<name>/``;
* renders an offscreen PNG thumbnail via ``cybercadkernel.viz`` — trying the
  trimesh GL path first, falling back to the headless matplotlib rasterizer;
* collects the exact B-rep mass properties (volume, area, bbox) as a caption.

The helper is deterministic: it prints/returns numbers, never wall-clock, so the
generated gallery README is stable across runs.

Run any example directly (``python3 examples/01_pipe_flange.py``) or the whole
set via ``python3 examples/run_all.py``.
"""

from __future__ import annotations

import contextlib
import json
import os
import sys
import warnings
from dataclasses import asdict, dataclass, field

# The environment's shapely is built against NumPy 1.x while NumPy 2.x is
# installed; the ABI mismatch is a noisy-but-harmless import warning. Silence it
# so the gallery output stays readable (trimesh still works for our mesh ops).
warnings.filterwarnings("ignore", message=".*NumPy 1.x.*")
warnings.filterwarnings("ignore", message=".*_ARRAY_API.*")

# Make the in-repo binding importable without an install step, and locate the
# real-engine dylib the same way the test-suite conftest does.
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT = os.path.abspath(os.path.join(_HERE, os.pardir))
_PYTHON_PKG = os.path.join(_REPO_ROOT, "python")
if _PYTHON_PKG not in sys.path:
    sys.path.insert(0, _PYTHON_PKG)

if "CYBERCADKERNEL_DYLIB" not in os.environ:
    _default_dylib = os.path.join(_REPO_ROOT, "build-mac", "libcybercadkernel.dylib")
    if os.path.exists(_default_dylib):
        os.environ["CYBERCADKERNEL_DYLIB"] = _default_dylib

OUT_ROOT = os.path.join(_HERE, "out")


def _preimport_trimesh_quietly() -> None:
    """Import trimesh once with stderr muted to swallow the shapely/NumPy ABI
    banner (a NumPy 1.x-compiled shapely under a NumPy 2.x runtime). After this
    the module is cached, so later imports in viz.py are silent and cheap."""
    import contextlib
    import io

    try:
        with contextlib.redirect_stderr(io.StringIO()):
            import trimesh  # noqa: F401
    except Exception:
        pass  # trimesh optional; PNG/glTF paths degrade on their own


@contextlib.contextmanager
def _muted_fd(fd: int):
    """Temporarily redirect an OS-level file descriptor to /dev/null.

    Used to swallow OCCT's C-level STEP-writer banner (printed straight to the
    process's stderr, below Python's stream layer) so the gallery output is
    clean.
    """
    saved = os.dup(fd)
    devnull = os.open(os.devnull, os.O_WRONLY)
    try:
        os.dup2(devnull, fd)
        yield
    finally:
        os.dup2(saved, fd)
        os.close(devnull)
        os.close(saved)


_preimport_trimesh_quietly()

# Chord tolerance (mm) used for every tessellation / STL / thumbnail. Small
# enough that curved rims read cleanly; large enough to keep meshes light.
DEFLECTION = 0.25


@dataclass
class PieceResult:
    """Everything the gallery README needs about one built piece."""

    name: str
    title: str
    description: str
    features: list[str]
    volume_mm3: float
    area_mm2: float
    bbox_min: tuple[float, float, float]
    bbox_max: tuple[float, float, float]
    bbox_size: tuple[float, float, float]
    center_of_mass: tuple[float, float, float]
    artifacts: dict[str, str] = field(default_factory=dict)
    render_backend: str = ""

    @property
    def png_rel(self) -> str:
        """PNG path relative to the ``examples/`` dir (for README links)."""
        return self.artifacts.get("png", "")


def _kernel():
    from cybercadkernel import Kernel

    return Kernel()  # require_brep=True: fail loudly on a stub build


def _render_png(mesh, png_path: str) -> str:
    """Render ``mesh`` to ``png_path``; return the backend actually used.

    Tries the trimesh GL offscreen path, then the headless matplotlib
    rasterizer. Returns ``"trimesh-gl"`` / ``"matplotlib"`` / ``""`` (on failure).
    """
    from cybercadkernel import viz

    # GL first (nicer shading), but degrade to matplotlib on any headless box.
    try:
        viz.render_png(mesh, png_path, resolution=(640, 640), prefer="gl")
        if os.path.getsize(png_path) > 0:
            return "trimesh-gl"
    except Exception:
        pass
    try:
        viz.render_png(mesh, png_path, resolution=(640, 640), prefer="matplotlib")
        if os.path.getsize(png_path) > 0:
            return "matplotlib"
    except Exception:
        return ""
    return ""


def _try_export_glb(mesh, glb_path: str) -> bool:
    """Best-effort glTF (.glb) export via trimesh. Returns success."""
    try:
        from cybercadkernel import viz

        viz.export_glb(mesh, glb_path)
        return os.path.getsize(glb_path) > 0
    except Exception:
        return False


def emit(
    shape,
    name: str,
    title: str,
    description: str,
    features: list[str],
) -> PieceResult:
    """Validate ``shape`` and write all gallery artifacts for it.

    Raises :class:`ValueError` if the solid is empty / non-positive volume — an
    example that yields that is a bug, not something to paper over.
    """
    out_dir = os.path.join(OUT_ROOT, name)
    os.makedirs(out_dir, exist_ok=True)

    mp = shape.mass_properties()
    if not (mp.valid and mp.volume > 0.0):
        raise ValueError(f"{name}: invalid/empty solid (volume={mp.volume})")
    bb = shape.bounding_box()

    artifacts: dict[str, str] = {}

    step_path = os.path.join(out_dir, f"{name}.step")
    with _muted_fd(2):  # swallow OCCT's STEP-writer banner
        shape.step_export(step_path)
    if os.path.getsize(step_path) > 0:
        artifacts["step"] = os.path.relpath(step_path, _HERE)

    stl_path = os.path.join(out_dir, f"{name}.stl")
    with _muted_fd(2):
        shape.stl_export(stl_path, deflection=DEFLECTION, binary=True)
    if os.path.getsize(stl_path) > 0:
        artifacts["stl"] = os.path.relpath(stl_path, _HERE)

    mesh = shape.tessellate(deflection=DEFLECTION)

    glb_path = os.path.join(out_dir, f"{name}.glb")
    if _try_export_glb(mesh, glb_path):
        artifacts["glb"] = os.path.relpath(glb_path, _HERE)

    png_path = os.path.join(out_dir, f"{name}.png")
    backend = _render_png(mesh, png_path)
    if backend and os.path.exists(png_path) and os.path.getsize(png_path) > 0:
        artifacts["png"] = os.path.relpath(png_path, _HERE)

    result = PieceResult(
        name=name,
        title=title,
        description=description,
        features=features,
        volume_mm3=round(float(mp.volume), 3),
        area_mm2=round(float(mp.area), 3),
        bbox_min=tuple(round(v, 3) for v in bb.min),
        bbox_max=tuple(round(v, 3) for v in bb.max),
        bbox_size=tuple(round(v, 3) for v in bb.size),
        center_of_mass=tuple(round(v, 3) for v in mp.center_of_mass),
        artifacts=artifacts,
        render_backend=backend,
    )

    # A machine-readable sidecar so run_all can rebuild the README without
    # re-running every script if it wants to.
    with open(os.path.join(out_dir, "meta.json"), "w") as fh:
        json.dump(asdict(result), fh, indent=2, sort_keys=True)

    return result


def run_and_report(build_fn, name, title, description, features) -> PieceResult:
    """Convenience for a script's ``__main__``: build, emit, print a caption."""
    kernel = _kernel()
    shape = build_fn(kernel)
    result = emit(shape, name, title, description, features)
    sz = result.bbox_size
    print(f"[{name}] {title}")
    print(f"    volume = {result.volume_mm3:.3f} mm^3   area = {result.area_mm2:.3f} mm^2")
    print(f"    bbox   = {sz[0]:.2f} x {sz[1]:.2f} x {sz[2]:.2f} mm")
    print(f"    render = {result.render_backend or 'none'}")
    for kind, path in sorted(result.artifacts.items()):
        print(f"    {kind:4s} -> examples/{path}")
    return result
