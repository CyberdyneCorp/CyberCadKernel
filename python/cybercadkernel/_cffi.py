"""Low-level ctypes binding of the CyberCadKernel ``cc_*`` C ABI.

This module is a faithful *1:1* mirror of
``include/cybercadkernel/cc_kernel.h``. It is a pure **consumer** of that stable,
plain-C boundary: it does not touch the ABI, add ergonomics, or interpret
results. It only

* declares one :class:`ctypes.Structure` for every POD value type
  (``CCMesh``, ``CCProfileSeg``, ``CCMassProps``, ``CCEdgePolyline``,
  ``CCFaceMesh``) with field order and C types matching the header byte-for-byte
  so the ctypes layout equals the C ABI layout;
* aliases ``CCShapeId`` to the header's ``typedef long`` (LP64 -> 64-bit);
* declares ``argtypes`` / ``restype`` for every one of the 73 ``cc_*`` symbols
  (so ctypes marshals arguments and, crucially, the *by-value* struct returns
  ``CCMesh`` / ``CCMassProps`` correctly);
* loads ``libcybercadkernel.dylib`` (searching ``build-mac/`` then the
  ``CYBERCADKERNEL_DYLIB`` env override) and exposes it via :func:`lib`.

Nothing here is "pythonic": handles are raw ``int``s, meshes are raw structs
with owned pointer fields the caller must free with the matching ``cc_*_free``,
and a failed call returns ``0`` / an empty struct exactly as in C. The ergonomic
object model lives in :mod:`cybercadkernel.kernel`.
"""

from __future__ import annotations

import ctypes
import os
import sys
from ctypes import (
    POINTER,
    c_char_p,
    c_double,
    c_int,
    c_long,
    c_void_p,
)

# ── POD value types (mirror cc_kernel.h byte-for-byte) ───────────────────────

# `typedef long CCShapeId;` — `long` is 64-bit on the LP64 targets (macOS arm64,
# Linux x86_64/arm64) this binding runs on. 0 = invalid / not built.
CCShapeId = c_long


class CCMesh(ctypes.Structure):
    """typedef struct { double *vertices; int vertexCount;
                        int *triangles; int triangleCount; } CCMesh;"""

    _fields_ = [
        ("vertices", POINTER(c_double)),  # x,y,z triplets, len = vertexCount*3
        ("vertexCount", c_int),
        ("triangles", POINTER(c_int)),  # i,j,k triplets, len = triangleCount*3
        ("triangleCount", c_int),
    ]


class CCProfileSeg(ctypes.Structure):
    """typedef struct { int kind; double x0,y0,x1,y1; double cx,cy,r;
                        double a0,a1; int ptOffset,ptCount; } CCProfileSeg;"""

    _fields_ = [
        ("kind", c_int),
        ("x0", c_double),
        ("y0", c_double),
        ("x1", c_double),
        ("y1", c_double),
        ("cx", c_double),
        ("cy", c_double),
        ("r", c_double),
        ("a0", c_double),
        ("a1", c_double),
        ("ptOffset", c_int),
        ("ptCount", c_int),
    ]


class CCMassProps(ctypes.Structure):
    """typedef struct { double volume, area, cx, cy, cz; int valid; } CCMassProps;"""

    _fields_ = [
        ("volume", c_double),
        ("area", c_double),
        ("cx", c_double),
        ("cy", c_double),
        ("cz", c_double),
        ("valid", c_int),
    ]


class CCEdgePolyline(ctypes.Structure):
    """typedef struct { int edgeId; double *points; int pointCount; } CCEdgePolyline;"""

    _fields_ = [
        ("edgeId", c_int),
        ("points", POINTER(c_double)),  # x,y,z triplets, len = pointCount*3
        ("pointCount", c_int),
    ]


class CCFaceMesh(ctypes.Structure):
    """typedef struct { int faceId; double *vertices; int vertexCount;
                        int *triangles; int triangleCount; } CCFaceMesh;"""

    _fields_ = [
        ("faceId", c_int),
        ("vertices", POINTER(c_double)),  # x,y,z triplets, len = vertexCount*3
        ("vertexCount", c_int),
        ("triangles", POINTER(c_int)),  # i,j,k face-local indices, len = triangleCount*3
        ("triangleCount", c_int),
    ]


# ── Library discovery ────────────────────────────────────────────────────────

_DYLIB_NAME = {
    "darwin": "libcybercadkernel.dylib",
    "linux": "libcybercadkernel.so",
}.get(sys.platform, "libcybercadkernel.so")


class KernelLibraryNotFound(RuntimeError):
    """Raised when the shared library cannot be located / dlopened."""


def _candidate_paths() -> list[str]:
    """Ordered locations to probe for the shared library.

    Priority: explicit ``CYBERCADKERNEL_DYLIB`` env override, then the proven
    ``build-mac`` output (``python/`` is a sibling of ``build-mac/`` under the
    repo root), then ``build/`` and finally the bare name for the system loader.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(here, os.pardir, os.pardir))
    paths: list[str] = []
    env = os.environ.get("CYBERCADKERNEL_DYLIB")
    if env:
        paths.append(env)
    paths.append(os.path.join(repo_root, "build-mac", _DYLIB_NAME))
    paths.append(os.path.join(repo_root, "build", _DYLIB_NAME))
    paths.append(_DYLIB_NAME)  # last resort: DYLD/LD search path
    return paths


def _load_library() -> ctypes.CDLL:
    tried: list[str] = []
    for path in _candidate_paths():
        tried.append(path)
        try:
            return ctypes.CDLL(path)
        except OSError:
            continue
    raise KernelLibraryNotFound(
        "could not load "
        + _DYLIB_NAME
        + "; set CYBERCADKERNEL_DYLIB or build it with "
        "python/build_dylib.sh. Tried:\n  " + "\n  ".join(tried)
    )


# ── Signature table ───────────────────────────────────────────────────────────

_DBL_P = POINTER(c_double)
_INT_P = POINTER(c_int)
_SEG_P = POINTER(CCProfileSeg)
_EDGE_PP = POINTER(POINTER(CCEdgePolyline))
_FACE_PP = POINTER(POINTER(CCFaceMesh))

# Short aliases used only to make the table line up with the header at a glance.
_S = CCShapeId
_D = c_double
_I = c_int

# name -> (argtypes, restype). One entry per cc_* declaration in cc_kernel.h,
# in header order. This is the single auditable point of the 1:1 mapping.
_SIGS: "dict[str, tuple[list, object]]" = {
    # ── Legacy mesh extrude ──────────────────────────────────────────────────
    "cc_extrude": ([_DBL_P, _I, _D], CCMesh),
    "cc_mesh_free": ([CCMesh], None),
    # ── Kernel availability + error ──────────────────────────────────────────
    "cc_brep_available": ([], _I),
    "cc_last_error": ([], c_char_p),
    # ── Parallel control ─────────────────────────────────────────────────────
    "cc_set_parallel": ([_I], None),
    "cc_parallel_enabled": ([], _I),
    # ── GPU tessellation control ─────────────────────────────────────────────
    "cc_set_gpu_tessellation": ([_I], None),
    "cc_gpu_tessellation_enabled": ([], _I),
    # ── Construction ─────────────────────────────────────────────────────────
    "cc_solid_extrude": ([_DBL_P, _I, _D], _S),
    "cc_solid_revolve": ([_DBL_P, _I, _D], _S),
    "cc_solid_loft": ([_DBL_P, _I, _DBL_P, _I, _D], _S),
    "cc_solid_loft_wires": ([_DBL_P, _I, _DBL_P, _I], _S),
    "cc_solid_sweep": ([_DBL_P, _I, _DBL_P, _I], _S),
    "cc_twisted_sweep": ([_DBL_P, _I, _DBL_P, _I, _D, _D], _S),
    "cc_loft_along_rail": ([_DBL_P, _I, _DBL_P, _I, _DBL_P, _I], _S),
    "cc_guided_sweep": ([_DBL_P, _I, _DBL_P, _I, _DBL_P, _I], _S),
    "cc_wrap_emboss": ([_S, _I, _DBL_P, _I, _D, _I], _S),
    "cc_helical_thread": ([_D, _D, _D, _D, _D, _D, _I], _S),
    "cc_tapered_thread": ([_D, _D, _D, _D, _D, _D, _D, _I], _S),
    "cc_tapered_shank": ([_D, _D, _D, _D], _S),
    "cc_solid_extrude_holes": ([_DBL_P, _I, _DBL_P, _I, _D], _S),
    "cc_solid_extrude_polyholes": ([_DBL_P, _I, _DBL_P, _INT_P, _I, _D], _S),
    "cc_solid_extrude_profile": ([_SEG_P, _I, _DBL_P, _I, _DBL_P, _I, _D], _S),
    "cc_solid_extrude_profile_polyholes": (
        [_SEG_P, _I, _DBL_P, _I, _DBL_P, _INT_P, _I, _DBL_P, _I, _D],
        _S,
    ),
    "cc_solid_revolve_profile": (
        [_SEG_P, _I, _D, _D, _D, _D, _DBL_P, _I, _D],
        _S,
    ),
    # ── Feature edits ────────────────────────────────────────────────────────
    "cc_fillet_edges": ([_S, _INT_P, _I, _D], _S),
    "cc_fillet_edges_variable": ([_S, _INT_P, _I, _D, _D], _S),
    "cc_chamfer_edges": ([_S, _INT_P, _I, _D], _S),
    "cc_shell": ([_S, _INT_P, _I, _D], _S),
    "cc_offset_face": ([_S, _I, _D], _S),
    "cc_replace_face": ([_S, _I, _D, _D], _S),
    "cc_replace_face_to_plane": ([_S, _I, _D, _D, _D, _D, _D, _D], _S),
    "cc_fillet_face": ([_S, _I, _D], _S),
    "cc_split_plane": ([_S, _D, _D, _D, _D, _D, _D, _I], _S),
    # ── Booleans ─────────────────────────────────────────────────────────────
    "cc_boolean": ([_S, _S, _I], _S),
    # ── Tessellation ─────────────────────────────────────────────────────────
    "cc_tessellate": ([_S, _D], CCMesh),
    # ── Queries ──────────────────────────────────────────────────────────────
    "cc_mass_properties": ([_S], CCMassProps),
    "cc_principal_moments": ([_S, _DBL_P], _I),
    "cc_bounding_box": ([_S, _DBL_P], _I),
    "cc_face_axis": ([_S, _I, _DBL_P], _I),
    "cc_subshape_ids": ([_S, _I, POINTER(_INT_P)], _I),
    "cc_ints_free": ([_INT_P], None),
    "cc_tangent_chain": ([_S, _INT_P, _I, POINTER(_INT_P)], _I),
    "cc_outer_rim_chain": ([_S, _INT_P, _I, POINTER(_INT_P)], _I),
    "cc_edge_polylines": ([_S, _EDGE_PP], _I),
    "cc_edge_polylines_free": ([POINTER(CCEdgePolyline), _I], None),
    "cc_offset_face_boundary": ([_S, _I, _D, POINTER(_DBL_P)], _I),
    "cc_points_free": ([_DBL_P], None),
    "cc_face_meshes": ([_S, _D, _FACE_PP], _I),
    "cc_face_meshes_free": ([POINTER(CCFaceMesh), _I], None),
    # ── Data exchange ────────────────────────────────────────────────────────
    "cc_step_export": ([_S, c_char_p], _I),
    "cc_step_import": ([c_char_p], _S),
    "cc_iges_export": ([_S, c_char_p], _I),
    "cc_iges_import": ([c_char_p], _S),
    # ── STL exchange (triangle mesh; binary=1 => binary, 0 => ASCII) ─────────
    "cc_stl_export": ([_S, c_char_p, _D, _I], _I),
    "cc_stl_import": ([c_char_p], _S),
    # ── Transforms ───────────────────────────────────────────────────────────
    "cc_scale_shape": ([_S, _D], _S),
    "cc_scale_shape_about": ([_S, _D, _D, _D, _D], _S),
    "cc_rotate_shape_about": ([_S, _D, _D, _D, _D, _D, _D, _D], _S),
    "cc_mirror_shape": ([_S, _D, _D, _D, _D, _D, _D], _S),
    "cc_translate_shape": ([_S, _D, _D, _D], _S),
    "cc_place_on_frame": ([_S, _D, _D, _D, _D, _D, _D, _D, _D, _D], _S),
    # ── Reference geometry (datum planes / axes) ─────────────────────────────
    "cc_ref_plane_from_points": ([_DBL_P, _DBL_P, _DBL_P, _DBL_P], _I),
    "cc_ref_plane_offset": ([_DBL_P, _DBL_P, _D, _DBL_P], _I),
    "cc_ref_plane_from_face": ([_S, _I, _DBL_P], _I),
    "cc_ref_axis_from_points": ([_DBL_P, _DBL_P, _DBL_P], _I),
    "cc_ref_axis_from_edge": ([_S, _I, _DBL_P], _I),
    "cc_ref_axis_from_face": ([_S, _I, _DBL_P], _I),
    # ── Robust thread boolean ────────────────────────────────────────────────
    "cc_thread_apply": ([_S, _S, _I], _S),
    # ── Full-round fillet ────────────────────────────────────────────────────
    "cc_full_round_fillet": ([_S, _I], _S),
    "cc_full_round_fillet_faces": ([_S, _I, _I, _I], _S),
    # ── G2 blend fillet ──────────────────────────────────────────────────────
    "cc_fillet_edges_g2": ([_S, _INT_P, _I, _D], _S),
    # ── Lifecycle ────────────────────────────────────────────────────────────
    "cc_shape_release": ([_S], None),
}


def _bind(lib: ctypes.CDLL) -> None:
    """Apply ``argtypes`` / ``restype`` to every ``cc_*`` symbol in ``_SIGS``."""
    for name, (argtypes, restype) in _SIGS.items():
        fn = getattr(lib, name)
        fn.argtypes = argtypes
        fn.restype = restype


# ── Public accessor ────────────────────────────────────────────────────────────

_lib: "ctypes.CDLL | None" = None


def lib() -> ctypes.CDLL:
    """Return the lazily loaded, bound, singleton ``CDLL`` for the kernel."""
    global _lib
    if _lib is None:
        loaded = _load_library()
        _bind(loaded)
        _lib = loaded
    return _lib


__all__ = [
    "CCShapeId",
    "CCMesh",
    "CCProfileSeg",
    "CCMassProps",
    "CCEdgePolyline",
    "CCFaceMesh",
    "KernelLibraryNotFound",
    "lib",
]
