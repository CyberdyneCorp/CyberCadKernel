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
* declares ``argtypes`` / ``restype`` for every one of the 111 ``cc_*`` symbols
  (so ctypes marshals arguments and, crucially, the *by-value* struct returns
  ``CCMesh`` / ``CCMassProps`` / ``CCTetMesh`` / ``CCDrawing`` / ``CCSection`` /
  ``CCQualityReport`` / ``CCProjection`` correctly);
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


class CCProjection(ctypes.Structure):
    """typedef struct { double footX,footY,footZ; double distance; int valid; }
    CCProjection; — foot-of-perpendicular of a point onto a face's surface."""

    _fields_ = [
        ("footX", c_double),
        ("footY", c_double),
        ("footZ", c_double),
        ("distance", c_double),
        ("valid", c_int),
    ]


class CCInterference(ctypes.Structure):
    """typedef struct { int state,clash,decided; double overlap_volume,min_distance;
    int has_witness; double witness_lo[3],witness_hi[3],witness_point[3]; }
    CCInterference; — clash verdict between two solids."""

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
    """typedef struct { int valid,decided,finite,closed_manifold,
    consistent_orientation,no_degenerate,no_self_intersection,first_failure; }
    CCValidityReport; — structural-validity breakdown of a solid."""

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
    """typedef struct { double *positions; int vertexCount; double *normals;
    double *uvs; int *triangles; int triangleCount; } CCDisplayMesh; —
    render-quality shading mesh (smooth normals, crease edges, optional UVs)."""

    _fields_ = [
        ("positions", POINTER(c_double)),  # x,y,z triplets, len = vertexCount*3
        ("vertexCount", c_int),
        ("normals", POINTER(c_double)),  # x,y,z unit triplets, len = vertexCount*3
        ("uvs", POINTER(c_double)),  # u,v pairs, len = vertexCount*2 (NULL if none)
        ("triangles", POINTER(c_int)),  # i,j,k triplets, len = triangleCount*3
        ("triangleCount", c_int),
    ]


class CCTetMesh(ctypes.Structure):
    """typedef struct { double *nodes; int nodeCount; int *elements;
    int elementCount; int nodesPerElement; int order; } CCTetMesh; —
    tetrahedral volume mesh (C3D4/C3D10 node order)."""

    _fields_ = [
        ("nodes", POINTER(c_double)),  # x,y,z triplets, len = nodeCount*3
        ("nodeCount", c_int),
        ("elements", POINTER(c_int)),  # len = elementCount*nodesPerElement
        ("elementCount", c_int),
        ("nodesPerElement", c_int),  # 4 (linear) or 10 (quadratic)
        ("order", c_int),  # 4 = C3D4, 10 = C3D10
    ]


class CCVolumeMeshOptions(ctypes.Structure):
    """typedef struct { int order; double target_element_size,grading,
    min_scaled_jacobian; } CCVolumeMeshOptions; — options for cc_tet_mesh*."""

    _fields_ = [
        ("order", c_int),
        ("target_element_size", c_double),
        ("grading", c_double),
        ("min_scaled_jacobian", c_double),
    ]


class CCQualityReport(ctypes.Structure):
    """typedef struct { double min_dihedral_angle,max_dihedral_angle,
    min_scaled_jacobian,mean_scaled_jacobian,max_aspect_ratio;
    int elements_below_threshold; int *flagged_elements; int valid; }
    CCQualityReport; — native tet-mesh quality census."""

    _fields_ = [
        ("min_dihedral_angle", c_double),
        ("max_dihedral_angle", c_double),
        ("min_scaled_jacobian", c_double),
        ("mean_scaled_jacobian", c_double),
        ("max_aspect_ratio", c_double),
        ("elements_below_threshold", c_int),
        ("flagged_elements", POINTER(c_int)),  # owned; free with cc_quality_report_free
        ("valid", c_int),
    ]


class CCPmiSummary(ctypes.Structure):
    """typedef struct { int dimensions,tolerances,datums,datum_targets,notes,
    annotation_geometry,unknown,total; } CCPmiSummary; — AP242 PMI census."""

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
    """typedef struct { double ax,ay,bx,by; } CCDrawingSegment; — one projected
    2D drawing-plane segment (drawing coords u,w in mm)."""

    _fields_ = [
        ("ax", c_double),
        ("ay", c_double),
        ("bx", c_double),
        ("by", c_double),
    ]


class CCDrawing(ctypes.Structure):
    """typedef struct { CCDrawingSegment *visible; int visibleCount;
    CCDrawingSegment *hidden; int hiddenCount; } CCDrawing; — HLR result."""

    _fields_ = [
        ("visible", POINTER(CCDrawingSegment)),
        ("visibleCount", c_int),
        ("hidden", POINTER(CCDrawingSegment)),
        ("hiddenCount", c_int),
    ]


class CCHlrOptions(ctypes.Structure):
    """typedef struct { double deflection; int samplesPerEdge;
    double surfaceOffset; } CCHlrOptions; — options for cc_hlr_project."""

    _fields_ = [
        ("deflection", c_double),
        ("samplesPerEdge", c_int),
        ("surfaceOffset", c_double),
    ]


class CCSectionLoop(ctypes.Structure):
    """typedef struct { double *pointsXYZ; int pointCount; int shape;
    double length,area; } CCSectionLoop; — one closed section loop on the cut
    plane (shape: 0 polygon, 1 circle, 2 ellipse)."""

    _fields_ = [
        ("pointsXYZ", POINTER(c_double)),  # x,y,z triplets, len = pointCount*3
        ("pointCount", c_int),
        ("shape", c_int),
        ("length", c_double),
        ("area", c_double),
    ]


class CCSection(ctypes.Structure):
    """typedef struct { CCSectionLoop *loops; int loopCount; double totalLength,
    totalArea; } CCSection; — result of cc_section_plane."""

    _fields_ = [
        ("loops", POINTER(CCSectionLoop)),
        ("loopCount", c_int),
        ("totalLength", c_double),
        ("totalArea", c_double),
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
    # Bundled in the installed wheel (package_data copies the dylib here).
    paths.append(os.path.join(here, "lib", _DYLIB_NAME))
    # Source-tree layout: python/ is a sibling of build-mac/ under the repo root.
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
_VALID_P = POINTER(CCValidityReport)
_INTF_P = POINTER(CCInterference)
_DISP_P = POINTER(CCDisplayMesh)
_QUAL_P = POINTER(CCQualityReport)  # (by-value return; alias kept for symmetry)
_PMI_P = POINTER(CCPmiSummary)

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
    # ── Active engine selection ──────────────────────────────────────────────
    "cc_set_engine": ([_I], None),
    "cc_active_engine": ([], _I),
    # ── Construction ─────────────────────────────────────────────────────────
    "cc_solid_extrude": ([_DBL_P, _I, _D], _S),
    "cc_solid_revolve": ([_DBL_P, _I, _D], _S),
    "cc_solid_loft": ([_DBL_P, _I, _DBL_P, _I, _D], _S),
    "cc_solid_loft_wires": ([_DBL_P, _I, _DBL_P, _I], _S),
    "cc_solid_sweep": ([_DBL_P, _I, _DBL_P, _I], _S),
    "cc_twisted_sweep": ([_DBL_P, _I, _DBL_P, _I, _D, _D], _S),
    "cc_loft_along_rail": ([_DBL_P, _I, _DBL_P, _I, _DBL_P, _I], _S),
    "cc_guided_sweep": ([_DBL_P, _I, _DBL_P, _I, _DBL_P, _I], _S),
    # loft family (true circles / typed sections / rails / N sections)
    "cc_loft_circles": ([_DBL_P, _DBL_P, _D, _DBL_P, _DBL_P, _D], _S),
    "cc_loft_circle_wire": ([_DBL_P, _DBL_P, _D, _DBL_P, _I], _S),
    "cc_loft_along_rails": ([_DBL_P, _I, _DBL_P, _I, _DBL_P, _I, _DBL_P, _I], _S),
    "cc_loft_typed": (
        [_SEG_P, _I, _DBL_P, _I, _DBL_P, _SEG_P, _I, _DBL_P, _I, _DBL_P],
        _S,
    ),
    "cc_solid_loft_sections": ([_DBL_P, _INT_P, _I], _S),
    # variable / guided-orientation sweeps
    "cc_guided_orient_sweep": ([_DBL_P, _I, _DBL_P, _I, _DBL_P, _I], _S),
    "cc_variable_sweep": (
        [_DBL_P, _I, _DBL_P, _I, _DBL_P, _I, _DBL_P, _I],
        _S,
    ),
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
    "cc_chamfer_edges_asym": ([_S, _INT_P, _I, _D, _D], _S),
    "cc_shell": ([_S, _INT_P, _I, _D], _S),
    "cc_offset_face": ([_S, _I, _D], _S),
    "cc_replace_face": ([_S, _I, _D, _D], _S),
    "cc_replace_face_to_plane": ([_S, _I, _D, _D, _D, _D, _D, _D], _S),
    "cc_draft_faces": ([_S, _INT_P, _I, _DBL_P, _DBL_P, _D], _S),
    "cc_fillet_face": ([_S, _I, _D], _S),
    "cc_split_plane": ([_S, _D, _D, _D, _D, _D, _D, _I], _S),
    # ── Sheet metal (native-only) ────────────────────────────────────────────
    "cc_sheet_base_flange": ([_DBL_P, _I, _D], _S),
    "cc_sheet_edge_flange": ([_S, _I, _D, _D, _D], _S),
    "cc_sheet_unfold": ([_S, _D], _S),
    # ── Surfacing: bounded N-gon fill patch ──────────────────────────────────
    "cc_fill_ngon": ([_DBL_P, _I, _INT_P, _DBL_P, _I], _S),
    "cc_project_point_on_face": ([_S, _I, _D, _D, _D], CCProjection),
    # ── Booleans ─────────────────────────────────────────────────────────────
    "cc_boolean": ([_S, _S, _I], _S),
    # ── Tessellation ─────────────────────────────────────────────────────────
    "cc_tessellate": ([_S, _D], CCMesh),
    # render-quality display mesh (smooth normals / crease edges / optional UVs)
    "cc_display_mesh": ([_S, _D, _D, _I, _I, _DISP_P], _I),
    "cc_display_mesh_free": ([_DISP_P], None),
    # ── Queries ──────────────────────────────────────────────────────────────
    "cc_mass_properties": ([_S], CCMassProps),
    "cc_principal_moments": ([_S, _DBL_P], _I),
    "cc_check_solid": ([_S, _VALID_P], _I),
    "cc_interference": ([_S, _S, _INTF_P], _I),
    "cc_bounding_box": ([_S, _DBL_P], _I),
    "cc_face_axis": ([_S, _I, _DBL_P], _I),
    # connected-solid enumeration
    "cc_shape_solid_count": ([_S], _I),
    "cc_shape_solid_at": ([_S, _I], _S),
    # measurement & curvature analysis (requires a CYBERCAD_HAS_NUMSCI build)
    "cc_measure_distance": ([_S, _I, _I, _I, _I, _DBL_P], _I),
    "cc_measure_angle": ([_S, _I, _I, _I, _I, _DBL_P], _I),
    "cc_surface_curvature": ([_S, _I, _D, _D, _DBL_P], _I),
    "cc_edge_curvature": ([_S, _I, _D, _DBL_P], _I),
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
    "cc_step_pmi_scan": ([c_char_p, _PMI_P], _I),
    "cc_iges_export": ([_S, c_char_p], _I),
    "cc_iges_import": ([c_char_p], _S),
    # ── STL exchange (triangle mesh; binary=1 => binary, 0 => ASCII) ─────────
    "cc_stl_export": ([_S, c_char_p, _D, _I], _I),
    "cc_stl_import": ([c_char_p], _S),
    # ── AR mesh export (glTF 2.0 / USDZ) ─────────────────────────────────────
    "cc_gltf_export": ([_S, c_char_p, _D, _I], _I),
    "cc_usdz_export": ([_S, c_char_p, _D], _I),
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
    # ── Tetrahedral volume meshing (requires a CYBERCAD_HAS_TETGEN build) ─────
    "cc_tet_mesh": ([_S, _D, CCVolumeMeshOptions], CCTetMesh),
    "cc_tet_mesh_surface": (
        [_DBL_P, _I, _INT_P, _I, CCVolumeMeshOptions],
        CCTetMesh,
    ),
    "cc_tet_mesh_free": ([CCTetMesh], None),
    "cc_mesh_quality": ([CCTetMesh, _D], CCQualityReport),
    "cc_quality_report_free": ([CCQualityReport], None),
    # ── Drafting: orthographic hidden-line removal (HLR) ──────────────────────
    "cc_hlr_project": ([_S, _DBL_P, _DBL_P, CCHlrOptions], CCDrawing),
    "cc_drawing_free": ([CCDrawing], None),
    # ── Planar section curves ────────────────────────────────────────────────
    "cc_section_plane": ([_S, _DBL_P, _DBL_P], CCSection),
    "cc_section_free": ([CCSection], None),
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
    "CCProjection",
    "CCInterference",
    "CCValidityReport",
    "CCDisplayMesh",
    "CCTetMesh",
    "CCVolumeMeshOptions",
    "CCQualityReport",
    "CCPmiSummary",
    "CCDrawingSegment",
    "CCDrawing",
    "CCHlrOptions",
    "CCSectionLoop",
    "CCSection",
    "KernelLibraryNotFound",
    "lib",
]
