"""Visualization helpers: turn a kernel tessellation into a ``trimesh.Trimesh``
and export it (STL/PLY/GLB/OBJ) or render an offscreen PNG.

Design notes
------------
* ``trimesh`` and ``matplotlib`` are *optional*: importing this module pulls in
  neither, and each entry point raises a clear error only if the backend it
  needs is missing. The core binding stays usable with just NumPy.
* Every function accepts either a pythonic :class:`~cybercadkernel.mesh.Mesh`
  (via its ``.vertices`` / ``.triangles`` arrays, or a ``.to_trimesh()`` method
  if one exists) *or* a raw ``(vertices, triangles)`` pair of NumPy arrays. This
  keeps viz decoupled from the exact shape of the mesh type.
* :func:`render_png` renders **offscreen** and is headless-safe: it first tries
  trimesh's GL path (``scene.save_image``) and, if no GL context is available
  (the common headless case — no pyglet/pyrender), it degrades gracefully to a
  pure-matplotlib ``Poly3DCollection`` raster with the ``Agg`` backend.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Tuple, Union

import numpy as np

if TYPE_CHECKING:  # pragma: no cover
    import trimesh

    from .mesh import Mesh

# A mesh source is either a Mesh-like object or a (vertices, triangles) pair.
MeshLike = Union["Mesh", Tuple[np.ndarray, np.ndarray]]

# ── PNG magic, used by callers/tests to validate output ─────────────────────
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _require_trimesh():
    try:
        import trimesh  # noqa: F401

        return trimesh
    except Exception as exc:  # pragma: no cover - env dependent
        raise RuntimeError(
            "trimesh is required for this operation; install it with "
            "`pip install trimesh`"
        ) from exc


def _require_matplotlib():
    try:
        import matplotlib

        matplotlib.use("Agg", force=True)  # headless raster backend
        import matplotlib.pyplot as plt  # noqa: F401
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: F401

        return matplotlib
    except Exception as exc:  # pragma: no cover - env dependent
        raise RuntimeError(
            "matplotlib is required for the headless render fallback; install "
            "it with `pip install matplotlib`"
        ) from exc


def _as_arrays(mesh: MeshLike) -> Tuple[np.ndarray, np.ndarray]:
    """Normalize any accepted mesh source to owned ``(vertices, triangles)``.

    Accepts a Mesh-like object exposing ``.vertices`` / ``.triangles`` or a
    plain ``(vertices, triangles)`` tuple. Returns ``(N,3)`` float64 vertices
    and ``(M,3)`` int32 triangle indices.
    """
    if hasattr(mesh, "vertices") and hasattr(mesh, "triangles"):
        vertices, triangles = mesh.vertices, mesh.triangles
    else:
        try:
            vertices, triangles = mesh  # type: ignore[misc]
        except (TypeError, ValueError) as exc:
            raise TypeError(
                "expected a Mesh with .vertices/.triangles or a "
                "(vertices, triangles) pair"
            ) from exc

    v = np.asarray(vertices, dtype=np.float64).reshape(-1, 3)
    f = np.asarray(triangles, dtype=np.int32).reshape(-1, 3)
    return v, f


def to_trimesh(mesh: MeshLike) -> "trimesh.Trimesh":
    """Convert a mesh source to a ``trimesh.Trimesh``.

    If a Mesh-like object provides its own ``.to_trimesh()``, that is used;
    otherwise the vertices/triangles are wrapped directly (``process=False`` so
    the geometry is preserved verbatim — exact volumes and watertightness are
    the caller's to assert).
    """
    to_tm = getattr(mesh, "to_trimesh", None)
    if callable(to_tm):
        return to_tm()
    tm = _require_trimesh()
    v, f = _as_arrays(mesh)
    return tm.Trimesh(vertices=v, faces=f, process=False)


def export_mesh(mesh: MeshLike, path: str) -> str:
    """Export ``mesh`` to ``path``; format is inferred from the extension
    (``.stl`` / ``.ply`` / ``.glb`` / ``.obj`` / ...). Returns ``path``."""
    to_trimesh(mesh).export(path)
    return path


def export_stl(mesh: MeshLike, path: str) -> str:
    """Export ``mesh`` as binary STL. Returns ``path``."""
    to_trimesh(mesh).export(path, file_type="stl")
    return path


def export_ply(mesh: MeshLike, path: str) -> str:
    """Export ``mesh`` as PLY. Returns ``path``."""
    to_trimesh(mesh).export(path, file_type="ply")
    return path


def export_glb(mesh: MeshLike, path: str) -> str:
    """Export ``mesh`` as a binary glTF (GLB). Returns ``path``."""
    to_trimesh(mesh).export(path, file_type="glb")
    return path


def _render_png_gl(
    tm_mesh: "trimesh.Trimesh", path: str, resolution: Tuple[int, int]
) -> bool:
    """Try trimesh's offscreen GL render. Returns True on success, False if no
    GL context / backend is available (so the caller can fall back)."""
    try:
        png = tm_mesh.scene().save_image(resolution=resolution)
    except (ImportError, ModuleNotFoundError, RuntimeError, ValueError, OSError):
        return False  # no pyglet/pyrender or no GL context: degrade gracefully
    if not png:
        return False
    with open(path, "wb") as fh:
        fh.write(png)
    return True


def _set_equal_aspect(ax, vertices: np.ndarray) -> None:
    """Center the 3D axes on the mesh with an equal-aspect cubic bound."""
    lo, hi = vertices.min(axis=0), vertices.max(axis=0)
    center = (lo + hi) / 2.0
    radius = float((hi - lo).max()) / 2.0 or 1.0
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)


def _render_png_matplotlib(
    vertices: np.ndarray,
    triangles: np.ndarray,
    path: str,
    resolution: Tuple[int, int],
) -> str:
    """Headless fallback: rasterize the triangle mesh with matplotlib's
    ``Poly3DCollection`` on the Agg backend. Works with no GL context."""
    _require_matplotlib()
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    dpi = 100
    width, height = resolution
    fig = plt.figure(figsize=(width / dpi, height / dpi), dpi=dpi)
    try:
        ax = fig.add_subplot(111, projection="3d")
        if triangles.size:
            faces = vertices[triangles]  # (M, 3, 3)
            collection = Poly3DCollection(faces, edgecolor="k", linewidths=0.15)
            collection.set_facecolor((0.60, 0.70, 0.85))
            ax.add_collection3d(collection)
            _set_equal_aspect(ax, vertices)
        ax.set_axis_off()
        fig.savefig(path, dpi=dpi, bbox_inches="tight", pad_inches=0)
    finally:
        plt.close(fig)
    return path


def render_png(
    mesh: MeshLike,
    path: str,
    resolution: Tuple[int, int] = (512, 512),
    prefer: str = "auto",
) -> str:
    """Render ``mesh`` to a PNG at ``path``, **offscreen** and headless-safe.

    ``prefer`` selects the render path:

    * ``"auto"`` (default) — try the trimesh GL renderer, then fall back to the
      matplotlib rasterizer if no GL context is available.
    * ``"gl"`` — trimesh GL only; raises if unavailable.
    * ``"matplotlib"`` — matplotlib fallback only (always headless-safe).

    Returns ``path``. The file starts with the PNG magic (:data:`PNG_MAGIC`).
    """
    if prefer not in ("auto", "gl", "matplotlib"):
        raise ValueError("prefer must be 'auto', 'gl', or 'matplotlib'")

    v, f = _as_arrays(mesh)

    if prefer in ("auto", "gl"):
        if _render_png_gl(to_trimesh(mesh), path, resolution):
            return path
        if prefer == "gl":
            raise RuntimeError(
                "offscreen GL render unavailable (no GL context / pyglet); "
                "use prefer='matplotlib' or 'auto'"
            )

    return _render_png_matplotlib(v, f, path, resolution)
