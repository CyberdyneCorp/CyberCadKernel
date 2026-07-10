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


class CCProjection(ctypes.Structure):
    _fields_ = [
        ("footX", c_double),
        ("footY", c_double),
        ("footZ", c_double),
        ("distance", c_double),
        ("valid", c_int),
    ]


class CCInterference(ctypes.Structure):
    _fields_ = [
        ("state", c_int),
        ("clash", c_int),
        ("decided", c_int),
        ("overlap_volume", c_double),
        ("min_distance", c_double),
        ("has_witness", c_int),
        ("witness_lo", c_double * 3),
        ("witness_hi", c_double * 3),
        ("witness_point", c_double * 3),
    ]


class CCValidityReport(ctypes.Structure):
    _fields_ = [
        ("valid", c_int),
        ("decided", c_int),
        ("finite", c_int),
        ("closed_manifold", c_int),
        ("consistent_orientation", c_int),
        ("no_degenerate", c_int),
        ("no_self_intersection", c_int),
        ("first_failure", c_int),
    ]


class CCDisplayMesh(ctypes.Structure):
    _fields_ = [
        ("positions", POINTER(c_double)),
        ("vertexCount", c_int),
        ("normals", POINTER(c_double)),
        ("uvs", POINTER(c_double)),
        ("triangles", POINTER(c_int)),
        ("triangleCount", c_int),
    ]


class CCTetMesh(ctypes.Structure):
    _fields_ = [
        ("nodes", POINTER(c_double)),
        ("nodeCount", c_int),
        ("elements", POINTER(c_int)),
        ("elementCount", c_int),
        ("nodesPerElement", c_int),
        ("order", c_int),
    ]


class CCVolumeMeshOptions(ctypes.Structure):
    _fields_ = [
        ("order", c_int),
        ("target_element_size", c_double),
        ("grading", c_double),
        ("min_scaled_jacobian", c_double),
    ]


class CCQualityReport(ctypes.Structure):
    _fields_ = [
        ("min_dihedral_angle", c_double),
        ("max_dihedral_angle", c_double),
        ("min_scaled_jacobian", c_double),
        ("mean_scaled_jacobian", c_double),
        ("max_aspect_ratio", c_double),
        ("elements_below_threshold", c_int),
        ("flagged_elements", POINTER(c_int)),
        ("valid", c_int),
    ]


class CCPmiSummary(ctypes.Structure):
    _fields_ = [
        ("dimensions", c_int),
        ("tolerances", c_int),
        ("datums", c_int),
        ("datum_targets", c_int),
        ("notes", c_int),
        ("annotation_geometry", c_int),
        ("unknown", c_int),
        ("total", c_int),
    ]


class CCDrawingSegment(ctypes.Structure):
    _fields_ = [
        ("ax", c_double),
        ("ay", c_double),
        ("bx", c_double),
        ("by", c_double),
    ]


class CCDrawing(ctypes.Structure):
    _fields_ = [
        ("visible", POINTER(CCDrawingSegment)),
        ("visibleCount", c_int),
        ("hidden", POINTER(CCDrawingSegment)),
        ("hiddenCount", c_int),
    ]


class CCHlrOptions(ctypes.Structure):
    _fields_ = [
        ("deflection", c_double),
        ("samplesPerEdge", c_int),
        ("surfaceOffset", c_double),
    ]


class CCSectionLoop(ctypes.Structure):
    _fields_ = [
        ("pointsXYZ", POINTER(c_double)),
        ("pointCount", c_int),
        ("shape", c_int),
        ("length", c_double),
        ("area", c_double),
    ]


class CCSection(ctypes.Structure):
    _fields_ = [
        ("loops", POINTER(CCSectionLoop)),
        ("loopCount", c_int),
        ("totalLength", c_double),
        ("totalArea", c_double),
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
    # Bundled in the installed wheel (package_data copies the dylib here).
    paths.append(os.path.join(here, "lib", _DYLIB_NAME))
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
        # stl exchange (triangle mesh; binary=1 => binary, 0 => ASCII)
        "cc_stl_export": ([S, c_char_p, D, I], I),
        "cc_stl_import": ([c_char_p], S),
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
        # active engine selection
        "cc_set_engine": ([I], None),
        "cc_active_engine": ([], I),
        # loft family / variable + guided-orientation sweeps
        "cc_loft_circles": ([_DBL_P, _DBL_P, D, _DBL_P, _DBL_P, D], S),
        "cc_loft_circle_wire": ([_DBL_P, _DBL_P, D, _DBL_P, I], S),
        "cc_loft_along_rails": ([_DBL_P, I, _DBL_P, I, _DBL_P, I, _DBL_P, I], S),
        "cc_loft_typed": (
            [_SEG_P, I, _DBL_P, I, _DBL_P, _SEG_P, I, _DBL_P, I, _DBL_P],
            S,
        ),
        "cc_solid_loft_sections": ([_DBL_P, _INT_P, I], S),
        "cc_guided_orient_sweep": ([_DBL_P, I, _DBL_P, I, _DBL_P, I], S),
        "cc_variable_sweep": ([_DBL_P, I, _DBL_P, I, _DBL_P, I, _DBL_P, I], S),
        # feature edits (asym chamfer / draft)
        "cc_chamfer_edges_asym": ([S, _INT_P, I, D, D], S),
        "cc_draft_faces": ([S, _INT_P, I, _DBL_P, _DBL_P, D], S),
        # sheet metal (native-only)
        "cc_sheet_base_flange": ([_DBL_P, I, D], S),
        "cc_sheet_edge_flange": ([S, I, D, D, D], S),
        "cc_sheet_unfold": ([S, D], S),
        # surfacing: bounded N-gon fill + point projection
        "cc_fill_ngon": ([_DBL_P, I, _INT_P, _DBL_P, I], S),
        "cc_project_point_on_face": ([S, I, D, D, D], CCProjection),
        # render-quality display mesh
        "cc_display_mesh": ([S, D, D, I, I, POINTER(CCDisplayMesh)], I),
        "cc_display_mesh_free": ([POINTER(CCDisplayMesh)], None),
        # analysis: validity / interference / solid enumeration
        "cc_check_solid": ([S, POINTER(CCValidityReport)], I),
        "cc_interference": ([S, S, POINTER(CCInterference)], I),
        "cc_shape_solid_count": ([S], I),
        "cc_shape_solid_at": ([S, I], S),
        # measurement & curvature (requires CYBERCAD_HAS_NUMSCI)
        "cc_measure_distance": ([S, I, I, I, I, _DBL_P], I),
        "cc_measure_angle": ([S, I, I, I, I, _DBL_P], I),
        "cc_surface_curvature": ([S, I, D, D, _DBL_P], I),
        "cc_edge_curvature": ([S, I, D, _DBL_P], I),
        # AR mesh export + STEP PMI scan
        "cc_gltf_export": ([S, c_char_p, D, I], I),
        "cc_usdz_export": ([S, c_char_p, D], I),
        "cc_step_pmi_scan": ([c_char_p, POINTER(CCPmiSummary)], I),
        # tetrahedral volume meshing (requires CYBERCAD_HAS_TETGEN)
        "cc_tet_mesh": ([S, D, CCVolumeMeshOptions], CCTetMesh),
        "cc_tet_mesh_surface": ([_DBL_P, I, _INT_P, I, CCVolumeMeshOptions], CCTetMesh),
        "cc_tet_mesh_free": ([CCTetMesh], None),
        "cc_mesh_quality": ([CCTetMesh, D], CCQualityReport),
        "cc_quality_report_free": ([CCQualityReport], None),
        # drafting HLR + planar section curves
        "cc_hlr_project": ([S, _DBL_P, _DBL_P, CCHlrOptions], CCDrawing),
        "cc_drawing_free": ([CCDrawing], None),
        "cc_section_plane": ([S, _DBL_P, _DBL_P], CCSection),
        "cc_section_free": ([CCSection], None),
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
