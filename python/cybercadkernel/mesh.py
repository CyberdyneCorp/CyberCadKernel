"""NumPy interop for the kernel's POD mesh structs.

``cc_tessellate`` / ``cc_face_meshes`` return ``CCMesh``/``CCFaceMesh`` with
raw C-owned pointer buffers. The functions here copy those buffers into owned
NumPy arrays *before* the C buffer is freed, so the resulting :class:`Mesh` has
no lifetime coupling to the kernel and is safe to keep, mutate, and export.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass

import numpy as np

from . import _ffi


def _copy_vertices(ptr, vertex_count: int) -> np.ndarray:
    """Copy ``vertex_count*3`` doubles into an owned (N, 3) float64 array."""
    if vertex_count <= 0 or not ptr:
        return np.empty((0, 3), dtype=np.float64)
    flat = np.ctypeslib.as_array(ptr, shape=(vertex_count * 3,))
    return flat.reshape(vertex_count, 3).astype(np.float64, copy=True)


def _copy_triangles(ptr, triangle_count: int) -> np.ndarray:
    """Copy ``triangle_count*3`` ints into an owned (M, 3) int32 array."""
    if triangle_count <= 0 or not ptr:
        return np.empty((0, 3), dtype=np.int32)
    flat = np.ctypeslib.as_array(ptr, shape=(triangle_count * 3,))
    return flat.reshape(triangle_count, 3).astype(np.int32, copy=True)


@dataclass(frozen=True)
class Mesh:
    """A display mesh as owned NumPy arrays.

    ``vertices`` is ``(N, 3)`` float64; ``triangles`` is ``(M, 3)`` int32 of
    vertex indices. ``face_id`` is set when the mesh came from a per-face
    ``CCFaceMesh`` (``-1`` for a whole-body tessellation).
    """

    vertices: np.ndarray
    triangles: np.ndarray
    face_id: int = -1

    @classmethod
    def from_ccmesh(cls, m: "_ffi.CCMesh") -> "Mesh":
        return cls(
            vertices=_copy_vertices(m.vertices, m.vertexCount),
            triangles=_copy_triangles(m.triangles, m.triangleCount),
        )

    @classmethod
    def from_ccfacemesh(cls, m: "_ffi.CCFaceMesh") -> "Mesh":
        return cls(
            vertices=_copy_vertices(m.vertices, m.vertexCount),
            triangles=_copy_triangles(m.triangles, m.triangleCount),
            face_id=m.faceId,
        )

    @property
    def vertex_count(self) -> int:
        return int(self.vertices.shape[0])

    @property
    def triangle_count(self) -> int:
        return int(self.triangles.shape[0])

    def bounding_box(self) -> tuple[np.ndarray, np.ndarray]:
        """(min_xyz, max_xyz) of the vertices."""
        if self.vertex_count == 0:
            z = np.zeros(3)
            return z, z
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def is_empty(self) -> bool:
        return self.vertex_count == 0 or self.triangle_count == 0
