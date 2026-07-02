"""Pythonic object model over the ``cc_*`` ABI.

Two types:

* :class:`Kernel` — the facade / factory. Loads the library, reports engine
  availability, constructs primitives, imports files, and toggles the additive
  parallel/GPU controls.
* :class:`Shape` — an RAII wrapper around a ``CCShapeId``. It is a context
  manager and calls ``cc_shape_release`` on ``__exit__`` (and as a GC backstop),
  so handle lifetime is scoped, not leaked. Every operation that returns a new
  body returns a new :class:`Shape`; the operands are left untouched (the ABI is
  functional — it never mutates an input body).

Failure handling: any ``cc_*`` call that yields ``0`` / ``valid == 0`` raises
:class:`~cybercadkernel.errors.KernelError` carrying ``cc_last_error()`` instead
of returning a sentinel.
"""

from __future__ import annotations

import ctypes
from typing import Iterable, Sequence

import numpy as np

from . import _ffi, _marshal
from .errors import BRepUnavailableError, KernelError
from .mesh import Mesh

_BOOL_OPS = {"fuse": 0, "cut": 1, "common": 2}


def _last_error() -> str:
    raw = _ffi.lib().cc_last_error()
    if not raw:
        return ""
    return raw.decode("utf-8", "replace") if isinstance(raw, bytes) else str(raw)


class Shape:
    """A kernel body referenced by an opaque handle.

    Construct via :class:`Kernel` factory methods, not directly. Use as a
    context manager to bound its lifetime::

        with kernel.box(10, 10, 10) as b:
            print(b.volume())
    """

    __slots__ = ("_id", "_kernel", "_released")

    def __init__(self, kernel: "Kernel", shape_id: int) -> None:
        self._kernel = kernel
        self._id = int(shape_id)
        self._released = False

    # ── lifetime ────────────────────────────────────────────────────────────

    @property
    def id(self) -> int:
        return self._id

    def _check(self) -> None:
        if self._released:
            raise KernelError("operation on released shape")

    def release(self) -> None:
        if not self._released and self._id != 0:
            _ffi.lib().cc_shape_release(self._id)
        self._released = True

    def __enter__(self) -> "Shape":
        return self

    def __exit__(self, *exc) -> None:
        self.release()

    def __del__(self) -> None:  # GC backstop; explicit release is preferred
        try:
            self.release()
        except Exception:
            pass

    def _wrap(self, new_id: int, op: str) -> "Shape":
        if new_id == 0:
            raise KernelError(op, _last_error())
        return Shape(self._kernel, new_id)

    # ── queries (real geometry) ───────────────────────────────────────────────

    def mass_properties(self) -> "MassProps":
        self._check()
        mp = _ffi.lib().cc_mass_properties(self._id)
        if not mp.valid:
            raise KernelError("cc_mass_properties", _last_error())
        return MassProps(mp.volume, mp.area, (mp.cx, mp.cy, mp.cz))

    def volume(self) -> float:
        return self.mass_properties().volume

    def area(self) -> float:
        return self.mass_properties().area

    def centroid(self) -> tuple[float, float, float]:
        return self.mass_properties().center_of_mass

    def bounding_box(self) -> tuple[np.ndarray, np.ndarray]:
        """Exact B-rep AABB as ``(min_xyz, max_xyz)`` float64 arrays."""
        self._check()
        out = (ctypes.c_double * 6)()
        if not _ffi.lib().cc_bounding_box(self._id, out):
            raise KernelError("cc_bounding_box", _last_error())
        v = np.array(out, dtype=np.float64)
        return v[:3], v[3:]

    def principal_moments(self) -> np.ndarray:
        self._check()
        out = (ctypes.c_double * 3)()
        if not _ffi.lib().cc_principal_moments(self._id, out):
            raise KernelError("cc_principal_moments", _last_error())
        return np.array(out, dtype=np.float64)

    def face_ids(self) -> list[int]:
        return self._subshape_ids(2)

    def edge_ids(self) -> list[int]:
        return self._subshape_ids(1)

    def vertex_ids(self) -> list[int]:
        return self._subshape_ids(0)

    def _subshape_ids(self, kind: int) -> list[int]:
        self._check()
        out = ctypes.POINTER(ctypes.c_int)()
        n = _ffi.lib().cc_subshape_ids(self._id, kind, ctypes.byref(out))
        if n <= 0 or not out:
            return []
        try:
            return [int(out[i]) for i in range(n)]
        finally:
            _ffi.lib().cc_ints_free(out)

    # ── tessellation → NumPy ──────────────────────────────────────────────────

    def tessellate(self, deflection: float = 0.1) -> Mesh:
        self._check()
        m = _ffi.lib().cc_tessellate(self._id, deflection)
        try:
            if m.vertexCount == 0:
                raise KernelError("cc_tessellate", _last_error())
            return Mesh.from_ccmesh(m)
        finally:
            _ffi.lib().cc_mesh_free(m)

    def face_meshes(self, deflection: float = 0.1) -> list[Mesh]:
        self._check()
        out = ctypes.POINTER(_ffi.CCFaceMesh)()
        n = _ffi.lib().cc_face_meshes(self._id, deflection, ctypes.byref(out))
        if n <= 0 or not out:
            return []
        try:
            return [Mesh.from_ccfacemesh(out[i]) for i in range(n)]
        finally:
            _ffi.lib().cc_face_meshes_free(out, n)

    # ── booleans ──────────────────────────────────────────────────────────────

    def boolean(self, other: "Shape", op: str) -> "Shape":
        self._check()
        other._check()
        if op not in _BOOL_OPS:
            raise ValueError(f"unknown boolean op {op!r}; use {list(_BOOL_OPS)}")
        new_id = _ffi.lib().cc_boolean(self._id, other._id, _BOOL_OPS[op])
        return self._wrap(new_id, f"cc_boolean/{op}")

    def fuse(self, other: "Shape") -> "Shape":
        return self.boolean(other, "fuse")

    def cut(self, other: "Shape") -> "Shape":
        return self.boolean(other, "cut")

    def common(self, other: "Shape") -> "Shape":
        return self.boolean(other, "common")

    # ── feature edits ─────────────────────────────────────────────────────────

    def fillet(self, edge_ids: Sequence[int], radius: float) -> "Shape":
        self._check()
        ids, n = _marshal.int_array(edge_ids)
        return self._wrap(
            _ffi.lib().cc_fillet_edges(self._id, ids, n, radius), "cc_fillet_edges"
        )

    def chamfer(self, edge_ids: Sequence[int], distance: float) -> "Shape":
        self._check()
        ids, n = _marshal.int_array(edge_ids)
        return self._wrap(
            _ffi.lib().cc_chamfer_edges(self._id, ids, n, distance),
            "cc_chamfer_edges",
        )

    def shell(self, face_ids: Sequence[int], thickness: float) -> "Shape":
        self._check()
        ids, n = _marshal.int_array(face_ids)
        return self._wrap(
            _ffi.lib().cc_shell(self._id, ids, n, thickness), "cc_shell"
        )

    # ── transforms ────────────────────────────────────────────────────────────

    def translate(self, tx: float, ty: float, tz: float) -> "Shape":
        self._check()
        return self._wrap(
            _ffi.lib().cc_translate_shape(self._id, tx, ty, tz),
            "cc_translate_shape",
        )

    def scale(self, factor: float) -> "Shape":
        self._check()
        return self._wrap(
            _ffi.lib().cc_scale_shape(self._id, factor), "cc_scale_shape"
        )

    # ── data exchange ─────────────────────────────────────────────────────────

    def export_step(self, path: str) -> None:
        self._check()
        if not _ffi.lib().cc_step_export(self._id, path.encode("utf-8")):
            raise KernelError("cc_step_export", _last_error())

    def export_iges(self, path: str) -> None:
        self._check()
        if not _ffi.lib().cc_iges_export(self._id, path.encode("utf-8")):
            raise KernelError("cc_iges_export", _last_error())

    def __repr__(self) -> str:
        state = "released" if self._released else f"id={self._id}"
        return f"<Shape {state}>"


class MassProps:
    """Exact mass properties (volume from the B-rep, not the mesh)."""

    __slots__ = ("volume", "area", "center_of_mass")

    def __init__(self, volume: float, area: float, com: tuple) -> None:
        self.volume = float(volume)
        self.area = float(area)
        self.center_of_mass = (float(com[0]), float(com[1]), float(com[2]))

    def __repr__(self) -> str:
        return (
            f"MassProps(volume={self.volume:.6g}, area={self.area:.6g}, "
            f"com={self.center_of_mass})"
        )


class Kernel:
    """The kernel facade: engine availability, construction, and controls."""

    def __init__(self) -> None:
        self._lib = _ffi.lib()

    # ── availability + error + controls ───────────────────────────────────────

    def brep_available(self) -> bool:
        return bool(self._lib.cc_brep_available())

    def require_brep(self) -> None:
        if not self.brep_available():
            raise BRepUnavailableError("cc_brep_available", "no B-rep engine linked")

    def last_error(self) -> str:
        return _last_error()

    def set_parallel(self, enabled: bool) -> None:
        self._lib.cc_set_parallel(1 if enabled else 0)

    def parallel_enabled(self) -> bool:
        return bool(self._lib.cc_parallel_enabled())

    def set_gpu_tessellation(self, enabled: bool) -> None:
        self._lib.cc_set_gpu_tessellation(1 if enabled else 0)

    def gpu_tessellation_enabled(self) -> bool:
        return bool(self._lib.cc_gpu_tessellation_enabled())

    # ── construction ──────────────────────────────────────────────────────────

    def _wrap(self, new_id: int, op: str) -> Shape:
        if new_id == 0:
            raise KernelError(op, _last_error())
        return Shape(self, new_id)

    def extrude(self, profile_xy, depth: float) -> Shape:
        """Extrude a closed 2-D profile (sequence of (x,y) or flat list) by
        ``depth`` along +Z into a solid."""
        buf, _ = _marshal.dbl_array(profile_xy)
        n = _marshal.point_count(profile_xy)
        return self._wrap(self._lib.cc_solid_extrude(buf, n, depth), "cc_solid_extrude")

    def revolve(self, profile_xy, angle_radians: float) -> Shape:
        buf, _ = _marshal.dbl_array(profile_xy)
        n = _marshal.point_count(profile_xy)
        return self._wrap(
            self._lib.cc_solid_revolve(buf, n, angle_radians), "cc_solid_revolve"
        )

    def box(self, dx: float, dy: float, dz: float) -> Shape:
        """Axis-aligned box with a corner at the origin (extrude of a rectangle)."""
        return self.extrude([0, 0, dx, 0, dx, dy, 0, dy], dz)

    def import_step(self, path: str) -> Shape:
        return self._wrap(
            self._lib.cc_step_import(path.encode("utf-8")), "cc_step_import"
        )

    def import_iges(self, path: str) -> Shape:
        return self._wrap(
            self._lib.cc_iges_import(path.encode("utf-8")), "cc_iges_import"
        )

    def __repr__(self) -> str:
        return f"<Kernel brep_available={self.brep_available()}>"
