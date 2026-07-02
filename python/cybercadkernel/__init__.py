"""CyberCadKernel — a pythonic Python binding over the ``cc_*`` C ABI.

A pure *consumer* of the stable C ABI declared in
``include/cybercadkernel/cc_kernel.h``, backed on macOS by a Homebrew-OCCT build
of the kernel (``build-mac/libcybercadkernel.dylib``). It exposes an ergonomic
object model that never touches the ABI:

* :class:`Kernel` — a facade over engine state (B-rep availability, the
  parallel / GPU-tessellation toggles) and the shape factories
  (:meth:`~Kernel.extrude`, :meth:`~Kernel.revolve`, lofts, sweeps, threads,
  STEP/IGES import, reference-geometry helpers).
* :class:`Shape` — a body wrapping a :c:type:`CCShapeId` with a
  context-managed, GC-safe lifetime (auto ``cc_shape_release``). Booleans,
  fillets/chamfers, transforms, queries, tessellation and STEP/IGES export all
  live here; every op returns a **new** :class:`Shape`.
* :class:`Mesh` — a tessellation result as owned NumPy arrays
  (vertices ``(N, 3)`` float64, triangles ``(M, 3)`` int32) with
  :meth:`~Mesh.to_trimesh` and geometry helpers.
* :class:`MassProps`, :class:`BoundingBox`, :class:`ReferencePlane`,
  :class:`ReferenceAxis`, :class:`FaceMesh`, :class:`EdgePolyline` — query
  results as dataclasses.
* :class:`CyberCadError` — raised (from :c:func:`cc_last_error`) whenever a call
  returns ``0`` / invalid, instead of returning ``nil``.

The low-level 1:1 ctypes binding lives in :mod:`cybercadkernel._cffi`.
"""

from __future__ import annotations

from ._cffi import KernelLibraryNotFound
from .api import (
    BooleanOp,
    BoundingBox,
    CyberCadError,
    EdgePolyline,
    FaceMesh,
    Kernel,
    MassProps,
    Mesh,
    ReferenceAxis,
    ReferencePlane,
    Shape,
    SubShapeKind,
)

# Backwards-compatible alias: the exception was historically ``KernelError``.
KernelError = CyberCadError

__all__ = [
    "Kernel",
    "Shape",
    "Mesh",
    "FaceMesh",
    "EdgePolyline",
    "MassProps",
    "BoundingBox",
    "ReferencePlane",
    "ReferenceAxis",
    "BooleanOp",
    "SubShapeKind",
    "CyberCadError",
    "KernelError",
    "KernelLibraryNotFound",
]

__version__ = "0.1.0"
