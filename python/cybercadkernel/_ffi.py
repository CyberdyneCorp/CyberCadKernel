"""Low-level ctypes binding of the CyberCadKernel ``cc_*`` C ABI.

This module is a *1:1* mirror of ``include/cybercadkernel/cc_kernel.h``. It
declares every POD ``struct`` and every ``cc_*`` function signature exactly as
the header defines them, then loads the shared library and applies the
``argtypes`` / ``restype`` annotations so ctypes marshals arguments and, in
particular, the by-value struct returns (``CCMesh``, ``CCMassProps``) correctly.

Nothing here is "pythonic": handles are raw ``int``s, meshes are raw structs
with pointer fields, and a failed call returns ``0`` / an empty struct just like
in C. The ergonomic object model lives in :mod:`cybercadkernel.kernel`; this
layer only guarantees a faithful, well-typed FFI surface.
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

# ── POD value types (mirror cc_kernel.h byte-for-byte) ──────────────────────

# CCShapeId is `typedef long CCShapeId;` — 64-bit on LP64 (macOS/Linux).
CCShapeId = c_long


class CCMesh(ctypes.Structure):
    _fields_ = [
        ("vertices", POINTER(c_double)),  # x,y,z triplets, len = vertexCount*3
        ("vertexCount", c_int),
        ("triangles", POINTER(c_int)),  # i,j,k triplets, len = triangleCount*3
        ("triangleCount", c_int),
    ]


class CCProfileSeg(ctypes.Structure):
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
    _fields_ = [
        ("volume", c_double),
        ("area", c_double),
        ("cx", c_double),
        ("cy", c_double),
        ("cz", c_double),
        ("valid", c_int),
    ]


class CCEdgePolyline(ctypes.Structure):
    _fields_ = [
        ("edgeId", c_int),
        ("points", POINTER(c_double)),  # x,y,z triplets, len = pointCount*3
        ("pointCount", c_int),
    ]


class CCFaceMesh(ctypes.Structure):
    _fields_ = [
        ("faceId", c_int),
        ("vertices", POINTER(c_double)),
        ("vertexCount", c_int),
        ("triangles", POINTER(c_int)),
        ("triangleCount", c_int),
    ]


# ── Library discovery ───────────────────────────────────────────────────────

_DYLIB_NAME = {
    "darwin": "libcybercadkernel.dylib",
    "linux": "libcybercadkernel.so",
}.get(sys.platform, "libcybercadkernel.so")


def _candidate_paths() -> list[str]:
    """Ordered locations to probe for the shared library.

    Priority: explicit env var, then the proven ``build-mac`` output relative to
    this package (python/ is a sibling of build-mac/), then a bare name for the
    system loader.
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


class KernelLibraryNotFound(RuntimeError):
    pass


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


# ── Signature table ─────────────────────────────────────────────────────────

_DBL_P = POINTER(c_double)
_INT_P = POINTER(c_int)
_SEG_P = POINTER(CCProfileSeg)


def _bind(lib: ctypes.CDLL) -> None:
    """Apply argtypes/restype to every cc_* symbol. Central so the mapping is
    auditable against the header in one place."""
    S = CCShapeId
    D = c_double
    I = c_int

    sigs: dict[str, tuple[list, object]] = {
        # legacy mesh extrude
        "cc_extrude": ([_DBL_P, I, D], CCMesh),
        "cc_mesh_free": ([CCMesh], None),
        # availability + error
        "cc_brep_available": ([], I),
        "cc_last_error": ([], c_char_p),
        # parallel control
        "cc_set_parallel": ([I], None),
        "cc_parallel_enabled": ([], I),
        # gpu tessellation control
        "cc_set_gpu_tessellation": ([I], None),
        "cc_gpu_tessellation_enabled": ([], I),
        # construction
        "cc_solid_extrude": ([_DBL_P, I, D], S),
        "cc_solid_revolve": ([_DBL_P, I, D], S),
        "cc_solid_loft": ([_DBL_P, I, _DBL_P, I, D], S),
        "cc_solid_loft_wires": ([_DBL_P, I, _DBL_P, I], S),
        "cc_solid_sweep": ([_DBL_P, I, _DBL_P, I], S),
        "cc_twisted_sweep": ([_DBL_P, I, _DBL_P, I, D, D], S),
        "cc_loft_along_rail": ([_DBL_P, I, _DBL_P, I, _DBL_P, I], S),
        "cc_guided_sweep": ([_DBL_P, I, _DBL_P, I, _DBL_P, I], S),
        "cc_wrap_emboss": ([S, I, _DBL_P, I, D, I], S),
        "cc_helical_thread": ([D, D, D, D, D, D, I], S),
        "cc_tapered_thread": ([D, D, D, D, D, D, D, I], S),
        "cc_tapered_shank": ([D, D, D, D], S),
        "cc_solid_extrude_holes": ([_DBL_P, I, _DBL_P, I, D], S),
        "cc_solid_extrude_polyholes": ([_DBL_P, I, _DBL_P, _INT_P, I, D], S),
        "cc_solid_extrude_profile": ([_SEG_P, I, _DBL_P, I, _DBL_P, I, D], S),
        "cc_solid_extrude_profile_polyholes": (
            [_SEG_P, I, _DBL_P, I, _DBL_P, _INT_P, I, _DBL_P, I, D],
            S,
        ),
        "cc_solid_revolve_profile": (
            [_SEG_P, I, D, D, D, D, _DBL_P, I, D],
            S,
        ),
        # feature edits
        "cc_fillet_edges": ([S, _INT_P, I, D], S),
        "cc_fillet_edges_variable": ([S, _INT_P, I, D, D], S),
        "cc_chamfer_edges": ([S, _INT_P, I, D], S),
        "cc_shell": ([S, _INT_P, I, D], S),
        "cc_offset_face": ([S, I, D], S),
        "cc_replace_face": ([S, I, D, D], S),
        "cc_replace_face_to_plane": ([S, I, D, D, D, D, D, D], S),
        "cc_fillet_face": ([S, I, D], S),
        "cc_split_plane": ([S, D, D, D, D, D, D, I], S),
        # booleans
        "cc_boolean": ([S, S, I], S),
        # tessellation
        "cc_tessellate": ([S, D], CCMesh),
        # queries
        "cc_mass_properties": ([S], CCMassProps),
        "cc_principal_moments": ([S, _DBL_P], I),
        "cc_bounding_box": ([S, _DBL_P], I),
        "cc_face_axis": ([S, I, _DBL_P], I),
        "cc_subshape_ids": ([S, I, POINTER(_INT_P)], I),
        "cc_ints_free": ([_INT_P], None),
        "cc_tangent_chain": ([S, _INT_P, I, POINTER(_INT_P)], I),
        "cc_outer_rim_chain": ([S, _INT_P, I, POINTER(_INT_P)], I),
        "cc_edge_polylines": ([S, POINTER(POINTER(CCEdgePolyline))], I),
        "cc_edge_polylines_free": ([POINTER(CCEdgePolyline), I], None),
        "cc_offset_face_boundary": ([S, I, D, POINTER(_DBL_P)], I),
        "cc_points_free": ([_DBL_P], None),
        "cc_face_meshes": ([S, D, POINTER(POINTER(CCFaceMesh))], I),
        "cc_face_meshes_free": ([POINTER(CCFaceMesh), I], None),
        # data exchange
        "cc_step_export": ([S, c_char_p], I),
        "cc_step_import": ([c_char_p], S),
        "cc_iges_export": ([S, c_char_p], I),
        "cc_iges_import": ([c_char_p], S),
        # transforms
        "cc_scale_shape": ([S, D], S),
        "cc_scale_shape_about": ([S, D, D, D, D], S),
        "cc_rotate_shape_about": ([S, D, D, D, D, D, D, D], S),
        "cc_mirror_shape": ([S, D, D, D, D, D, D], S),
        "cc_translate_shape": ([S, D, D, D], S),
        "cc_place_on_frame": ([S, D, D, D, D, D, D, D, D, D], S),
        # reference geometry
        "cc_ref_plane_from_points": ([_DBL_P, _DBL_P, _DBL_P, _DBL_P], I),
        "cc_ref_plane_offset": ([_DBL_P, _DBL_P, D, _DBL_P], I),
        "cc_ref_plane_from_face": ([S, I, _DBL_P], I),
        "cc_ref_axis_from_points": ([_DBL_P, _DBL_P, _DBL_P], I),
        "cc_ref_axis_from_edge": ([S, I, _DBL_P], I),
        "cc_ref_axis_from_face": ([S, I, _DBL_P], I),
        # phase-3 additive
        "cc_thread_apply": ([S, S, I], S),
        "cc_full_round_fillet": ([S, I], S),
        "cc_full_round_fillet_faces": ([S, I, I, I], S),
        "cc_fillet_edges_g2": ([S, _INT_P, I, D], S),
        # lifecycle
        "cc_shape_release": ([S], None),
    }

    for name, (argtypes, restype) in sigs.items():
        fn = getattr(lib, name)
        fn.argtypes = argtypes
        fn.restype = restype


# ── Public accessor ──────────────────────────────────────────────────────────

_lib: ctypes.CDLL | None = None


def lib() -> ctypes.CDLL:
    """Return the (lazily loaded, singleton) bound ``CDLL``."""
    global _lib
    if _lib is None:
        loaded = _load_library()
        _bind(loaded)
        _lib = loaded
    return _lib
