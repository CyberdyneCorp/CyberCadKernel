"""Shared helper for the exact-NURBS example gallery (Wave J7).

Where the B-rep gallery (`_gallery.py`) drives the OCCT `Kernel` / `Shape`
object model, THIS helper drives the exact-NURBS Python layer
(`cybercadkernel.nurbs`): the `Curve` / `Surface` handles and the module-level
`nurbs.skin` / `nurbs.revolve` / `nurbs.fillet_freeform_g2` / ... wrappers over
the additive `cc_nurbs_*` C facade.

Each NURBS example script builds one or more :class:`cybercadkernel.nurbs.Surface`
objects, tessellates each to a single-surface DISPLAY mesh, and hands the mesh
list to :func:`emit_meshes`, which:

* validates each mesh (non-empty vertices + triangles, finite bbox);
* renders an offscreen PNG thumbnail via ``cybercadkernel.viz`` — trying the
  trimesh GL path first, falling back to the headless matplotlib rasterizer, and
  asserting the PNG magic bytes on the written file;
* writes a machine-readable ``meta.json`` sidecar.

DISPLAY-TESSELLATION CAVEAT (nurbs.py module docstring, design.md §4): a single
`Surface.tessellate()` is REAL geometry — the exact rational surface sampled to a
display mesh (a closed single surface, e.g. a revolved sphere, is watertight).
A piece assembled from SEVERAL surfaces (a fillet band + its two host faces, the
sub-patches of a vertex blend) is shown as a SET of such meshes — a face-set, not
a sewn watertight solid. The gallery is honest about which is which per piece.

Run any example directly (``python3 examples/n1_lofted_bracket.py``) or the whole
NURBS set via ``python3 examples/run_all_nurbs.py``.
"""

from __future__ import annotations

import io
import os
import sys
import warnings
from dataclasses import asdict, dataclass, field

# shapely in this env is built against NumPy 1.x under a NumPy 2.x runtime; the
# ABI-mismatch banner is noisy-but-harmless. Silence it so the output is readable.
warnings.filterwarnings("ignore", message=".*NumPy 1.x.*")
warnings.filterwarnings("ignore", message=".*_ARRAY_API.*")

# Make the in-repo binding importable without an install step, and locate the
# real-engine dylib the same way the test-suite conftest / _gallery.py does.
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

# Per-piece tessellation sample counts kept modest: dense enough that curved rims
# and blend bands read cleanly, light enough that the committed PNGs stay small.
TESS_NU = 40
TESS_NV = 24


def _preimport_trimesh_quietly() -> None:
    """Import trimesh once with stderr muted to swallow the shapely/NumPy ABI
    banner; after this the module is cached so viz.py's later import is silent."""
    import contextlib

    try:
        with contextlib.redirect_stderr(io.StringIO()):
            import trimesh  # noqa: F401
    except Exception:
        pass  # trimesh optional; PNG path degrades to matplotlib on its own


_preimport_trimesh_quietly()


@dataclass
class NurbsPieceResult:
    """Everything the NURBS gallery index needs about one built piece."""

    name: str
    title: str
    description: str
    feature: str  # the cc_nurbs_* feature this piece exercises
    api_calls: list[str]  # the nurbs.* wrappers it calls
    surface_count: int
    is_single_surface: bool  # True ⇒ real geometry; False ⇒ honest face-set
    vertex_count: int
    triangle_count: int
    bbox_min: tuple[float, float, float]
    bbox_max: tuple[float, float, float]
    bbox_size: tuple[float, float, float]
    artifacts: dict[str, str] = field(default_factory=dict)
    render_backend: str = ""

    @property
    def png_rel(self) -> str:
        return self.artifacts.get("png", "")


def _combine_meshes(meshes):
    """Concatenate a list of Mesh-like objects into one (vertices, triangles) pair.

    Each mesh contributes its own vertices; triangle indices are offset so the
    combined mesh renders every surface in one scene. This is a DISPLAY combine
    (a face-set) — it does NOT sew the surfaces into a watertight solid.
    """
    import numpy as np

    all_v = []
    all_f = []
    offset = 0
    for m in meshes:
        v = np.asarray(m.vertices, dtype=np.float64).reshape(-1, 3)
        f = np.asarray(m.triangles, dtype=np.int32).reshape(-1, 3)
        all_v.append(v)
        all_f.append(f + offset)
        offset += v.shape[0]
    if not all_v:
        raise ValueError("no meshes to combine")
    return np.vstack(all_v), np.vstack(all_f)


def _render_png(vertices, triangles, png_path: str) -> str:
    """Render (vertices, triangles) to ``png_path``; return the backend used.

    Tries the trimesh GL offscreen path, then the headless matplotlib
    rasterizer. Asserts the written file starts with the PNG magic bytes.
    Returns ``"trimesh-gl"`` / ``"matplotlib"`` / ``""`` (on failure).
    """
    from cybercadkernel import viz

    mesh = (vertices, triangles)
    for backend, prefer in (("trimesh-gl", "gl"), ("matplotlib", "matplotlib")):
        try:
            viz.render_png(mesh, png_path, resolution=(640, 640), prefer=prefer)
        except Exception:
            continue
        if os.path.exists(png_path) and os.path.getsize(png_path) >= len(viz.PNG_MAGIC):
            with open(png_path, "rb") as fh:
                head = fh.read(len(viz.PNG_MAGIC))
            if head == viz.PNG_MAGIC:
                return backend
    return ""


def emit_meshes(
    meshes,
    name: str,
    title: str,
    description: str,
    feature: str,
    api_calls: list[str],
    is_single_surface: bool,
) -> NurbsPieceResult:
    """Validate the display meshes for a NURBS piece and write its artifacts.

    ``meshes`` is a list of :class:`cybercadkernel.mesh.Mesh` (one per tessellated
    surface). Raises :class:`ValueError` if the combined display mesh is empty —
    an example that yields that is a bug, not something to paper over. Writes a
    PNG thumbnail (magic-validated) and a ``meta.json`` sidecar into
    ``examples/out/<name>/``.
    """
    import numpy as np

    out_dir = os.path.join(OUT_ROOT, name)
    os.makedirs(out_dir, exist_ok=True)

    vertices, triangles = _combine_meshes(meshes)
    if vertices.shape[0] == 0 or triangles.shape[0] == 0:
        raise ValueError(f"{name}: empty display mesh (verts={vertices.shape[0]})")

    lo = vertices.min(axis=0)
    hi = vertices.max(axis=0)
    if not (np.all(np.isfinite(lo)) and np.all(np.isfinite(hi))):
        raise ValueError(f"{name}: non-finite bounding box")

    png_path = os.path.join(out_dir, f"{name}.png")
    backend = _render_png(vertices, triangles, png_path)
    artifacts: dict[str, str] = {}
    if backend:
        artifacts["png"] = os.path.relpath(png_path, _HERE)

    result = NurbsPieceResult(
        name=name,
        title=title,
        description=description,
        feature=feature,
        api_calls=list(api_calls),
        surface_count=len(meshes),
        is_single_surface=is_single_surface,
        vertex_count=int(vertices.shape[0]),
        triangle_count=int(triangles.shape[0]),
        bbox_min=tuple(round(float(v), 3) for v in lo),
        bbox_max=tuple(round(float(v), 3) for v in hi),
        bbox_size=tuple(round(float(v), 3) for v in (hi - lo)),
        artifacts=artifacts,
        render_backend=backend,
    )

    import json

    with open(os.path.join(out_dir, "meta.json"), "w") as fh:
        json.dump(asdict(result), fh, indent=2, sort_keys=True)

    return result


def run_and_report(build_fn) -> NurbsPieceResult:
    """Convenience for a script's ``__main__``: call ``build_fn()`` (which returns
    the kwargs for :func:`emit_meshes`), emit artifacts, and print a caption."""
    kwargs = build_fn()
    result = emit_meshes(**kwargs)
    sz = result.bbox_size
    kind = "single surface (real geometry)" if result.is_single_surface else "face-set (display)"
    print(f"[{result.name}] {result.title}")
    print(f"    feature = {result.feature}")
    print(f"    api     = {', '.join(result.api_calls)}")
    print(f"    kind    = {kind}  ({result.surface_count} surface(s))")
    print(f"    mesh    = {result.vertex_count} verts / {result.triangle_count} tris")
    print(f"    bbox    = {sz[0]:.2f} x {sz[1]:.2f} x {sz[2]:.2f}")
    print(f"    render  = {result.render_backend or 'none'}")
    if result.png_rel:
        print(f"    png     -> examples/{result.png_rel}")
    return result
