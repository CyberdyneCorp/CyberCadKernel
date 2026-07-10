"""Pythonic API over the low-level ``cc_*`` C ABI.

This module is the *ergonomic* layer of the binding. It sits directly on top of
:mod:`cybercadkernel._cffi` (the faithful 1:1 ctypes mirror of
``include/cybercadkernel/cc_kernel.h``) and turns the plain-C surface into
idiomatic Python:

* :class:`Kernel` — a small facade for engine-wide state (availability, the
  parallel / GPU-tessellation toggles) and for importing shapes from files.
* :class:`Shape` — an object wrapping a :c:type:`CCShapeId` with a
  *context-managed, GC-safe* lifetime. Every construction / feature / transform
  op returns a **new** :class:`Shape`; the handle is released exactly once via
  :meth:`Shape.close`, ``__exit__`` or ``__del__``.
* :class:`Mesh` — a tessellation result as owned NumPy arrays (vertices
  ``(N, 3)`` float64, triangles ``(M, 3)`` int32) with a :meth:`Mesh.to_trimesh`
  bridge and geometry helpers.
* :class:`MassProps` / :class:`BoundingBox` — POD query results as dataclasses.
* :class:`CyberCadError` — raised (from :c:func:`cc_last_error`) whenever a call
  returns ``0`` / an invalid handle, instead of silently returning ``nil``.

The module never touches the ABI. It is a pure consumer: it marshals Python
values into the ctypes calls, checks return codes, copies out any engine-owned
buffers into NumPy, and frees those buffers with the matching ``cc_*_free``.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import TYPE_CHECKING, Iterable, Sequence

import numpy as np

from . import _cffi
from ._cffi import (
    CCDisplayMesh,
    CCDrawing,
    CCEdgePolyline,
    CCFaceMesh,
    CCHlrOptions,
    CCInterference,
    CCMassProps,
    CCMesh,
    CCPmiSummary,
    CCProfileSeg,
    CCProjection,
    CCQualityReport,
    CCSection,
    CCTetMesh,
    CCValidityReport,
    CCVolumeMeshOptions,
    KernelLibraryNotFound,
)

if TYPE_CHECKING:  # pragma: no cover - typing only
    import trimesh as _trimesh_mod

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
    "KernelLibraryNotFound",
    "DisplayMesh",
    "TetMesh",
    "QualityReport",
    "ValidityReport",
    "Interference",
    "Projection",
    "PmiSummary",
    "SectionLoop",
    "Section",
    "Drawing",
]


# ── Exceptions ───────────────────────────────────────────────────────────────


class CyberCadError(RuntimeError):
    """A kernel operation failed.

    Raised whenever a ``cc_*`` call reports failure (a ``0`` / invalid shape id,
    or a ``0`` status flag). The message is the engine's thread-local
    :c:func:`cc_last_error` string when available, else a generic description of
    the failed operation.
    """


def _last_error() -> str:
    """Return the engine's thread-local last-error string (``""`` if none)."""
    raw = _cffi.lib().cc_last_error()
    if not raw:
        return ""
    try:
        return raw.decode("utf-8", "replace")
    except Exception:  # pragma: no cover - defensive
        return str(raw)


def _fail(op: str) -> "CyberCadError":
    """Build a :class:`CyberCadError` for a failed ``op``, using cc_last_error."""
    detail = _last_error()
    return CyberCadError(f"{op} failed: {detail}" if detail else f"{op} failed")


# ── Enums (small ints the ABI takes) ──────────────────────────────────────────


class BooleanOp:
    """Boolean operation selectors for :func:`cc_boolean` (``op`` argument)."""

    FUSE = 0
    CUT = 1
    COMMON = 2


class SubShapeKind:
    """Sub-shape kinds for :meth:`Shape.subshape_ids` (``kind`` argument)."""

    VERTEX = 0
    EDGE = 1
    FACE = 2


# ── Value / result dataclasses ─────────────────────────────────────────────────


@dataclass(frozen=True)
class MassProps:
    """Exact B-rep mass properties of a solid (not derived from the mesh).

    Attributes
    ----------
    volume : float
        Volume in mm^3.
    area : float
        Total surface area in mm^2.
    center_of_mass : tuple[float, float, float]
        Centroid ``(cx, cy, cz)``.
    valid : bool
        ``True`` when the engine produced valid props for a known body.
    """

    volume: float
    area: float
    center_of_mass: tuple[float, float, float]
    valid: bool


@dataclass(frozen=True)
class BoundingBox:
    """Axis-aligned bounding box of a body's B-rep (not its tessellation)."""

    min: tuple[float, float, float]
    max: tuple[float, float, float]

    @property
    def size(self) -> tuple[float, float, float]:
        """The box extents ``(dx, dy, dz)``."""
        return (
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        )

    @property
    def center(self) -> tuple[float, float, float]:
        """The box center ``(x, y, z)``."""
        return (
            0.5 * (self.min[0] + self.max[0]),
            0.5 * (self.min[1] + self.max[1]),
            0.5 * (self.min[2] + self.max[2]),
        )


@dataclass(frozen=True)
class ReferencePlane:
    """A datum plane: an origin point and a unit normal."""

    origin: tuple[float, float, float]
    normal: tuple[float, float, float]


@dataclass(frozen=True)
class ReferenceAxis:
    """A datum axis: an origin point and a unit direction."""

    origin: tuple[float, float, float]
    direction: tuple[float, float, float]


@dataclass
class Mesh:
    """A triangle mesh as owned NumPy arrays.

    Attributes
    ----------
    vertices : numpy.ndarray
        ``(N, 3)`` float64 vertex positions.
    triangles : numpy.ndarray
        ``(M, 3)`` int32 triangle vertex indices.
    """

    vertices: np.ndarray
    triangles: np.ndarray

    def __post_init__(self) -> None:
        self.vertices = np.ascontiguousarray(self.vertices, dtype=np.float64).reshape(
            -1, 3
        )
        self.triangles = np.ascontiguousarray(self.triangles, dtype=np.int32).reshape(
            -1, 3
        )

    # -- counts -------------------------------------------------------------
    @property
    def vertex_count(self) -> int:
        """Number of vertices."""
        return int(self.vertices.shape[0])

    @property
    def triangle_count(self) -> int:
        """Number of triangles."""
        return int(self.triangles.shape[0])

    def __len__(self) -> int:
        return self.triangle_count

    # -- helpers ------------------------------------------------------------
    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(min_xyz, max_xyz)`` as two ``(3,)`` float64 arrays."""
        if self.vertex_count == 0:
            zero = np.zeros(3, dtype=np.float64)
            return zero, zero.copy()
        return self.vertices.min(axis=0), self.vertices.max(axis=0)

    def triangle_areas(self) -> np.ndarray:
        """Return the per-triangle area as an ``(M,)`` float64 array."""
        v = self.vertices
        t = self.triangles
        if self.triangle_count == 0:
            return np.zeros(0, dtype=np.float64)
        a = v[t[:, 0]]
        b = v[t[:, 1]]
        c = v[t[:, 2]]
        cross = np.cross(b - a, c - a)
        return 0.5 * np.linalg.norm(cross, axis=1)

    def surface_area(self) -> float:
        """Total mesh surface area (sum of triangle areas)."""
        return float(self.triangle_areas().sum())

    def is_empty(self) -> bool:
        """``True`` when the mesh has no triangles."""
        return self.triangle_count == 0

    def to_trimesh(self, process: bool = False) -> "_trimesh_mod.Trimesh":
        """Return a :class:`trimesh.Trimesh` copy of this mesh.

        ``trimesh`` is imported lazily so the core API has no hard dependency on
        it. ``process=False`` (default) keeps vertices/faces exactly as produced
        by the kernel; pass ``process=True`` to let trimesh merge/clean them.
        """
        import trimesh  # local import: optional dependency

        return trimesh.Trimesh(
            vertices=self.vertices.copy(),
            faces=self.triangles.astype(np.int64),
            process=process,
        )


@dataclass
class FaceMesh:
    """One B-rep face as its own mesh, tagged with its 1-based ``face_id``."""

    face_id: int
    mesh: Mesh


@dataclass
class EdgePolyline:
    """One B-rep edge as a single 3-D polyline, tagged with its ``edge_id``."""

    edge_id: int
    points: np.ndarray  # (P, 3) float64


@dataclass
class DisplayMesh:
    """A render-quality shading mesh: positions, per-vertex smooth normals with
    crease-angle hard edges, optional UVs, and triangle indices — all owned
    NumPy arrays."""

    positions: np.ndarray  # (N, 3) float64
    normals: np.ndarray  # (N, 3) float64 unit normals
    triangles: np.ndarray  # (M, 3) int32
    uvs: "np.ndarray | None" = None  # (N, 2) float64 or None

    @property
    def vertex_count(self) -> int:
        return int(self.positions.shape[0])

    @property
    def triangle_count(self) -> int:
        return int(self.triangles.shape[0])


@dataclass
class TetMesh:
    """A tetrahedral volume mesh as owned NumPy arrays.

    ``nodes`` is ``(N, 3)`` float64; ``elements`` is
    ``(E, nodes_per_element)`` int32 (4 for linear C3D4, 10 for quadratic
    C3D10), 0-based indices into ``nodes``.
    """

    nodes: np.ndarray
    elements: np.ndarray
    nodes_per_element: int
    order: int

    @property
    def node_count(self) -> int:
        return int(self.nodes.shape[0])

    @property
    def element_count(self) -> int:
        return int(self.elements.shape[0])


@dataclass(frozen=True)
class QualityReport:
    """Native tet-mesh quality census (angles in degrees)."""

    min_dihedral_angle: float
    max_dihedral_angle: float
    min_scaled_jacobian: float
    mean_scaled_jacobian: float
    max_aspect_ratio: float
    elements_below_threshold: int
    flagged_elements: np.ndarray  # (K,) int32 ids of below-threshold tets
    valid: bool


@dataclass(frozen=True)
class ValidityReport:
    """Structural-validity breakdown of a solid (``cc_check_solid``)."""

    valid: bool
    decided: bool
    finite: bool
    closed_manifold: bool
    consistent_orientation: bool
    no_degenerate: bool
    no_self_intersection: bool
    first_failure: int


@dataclass(frozen=True)
class Interference:
    """Clash verdict between two solids (``cc_interference``).

    ``state`` is 0 CLEAR, 1 TOUCHING, 2 CLASH.
    """

    state: int
    clash: bool
    decided: bool
    overlap_volume: float
    min_distance: float
    has_witness: bool
    witness_lo: "tuple[float, float, float] | None"
    witness_hi: "tuple[float, float, float] | None"
    witness_point: "tuple[float, float, float] | None"


@dataclass(frozen=True)
class Projection:
    """Foot-of-perpendicular of a point onto a face's analytic surface."""

    foot: tuple[float, float, float]
    distance: float
    valid: bool


@dataclass(frozen=True)
class PmiSummary:
    """AP242 PMI / GD&T annotation census of a STEP file (``cc_step_pmi_scan``)."""

    dimensions: int
    tolerances: int
    datums: int
    datum_targets: int
    notes: int
    annotation_geometry: int
    unknown: int
    total: int


@dataclass(frozen=True)
class SectionLoop:
    """One closed section loop on a cut plane.

    ``shape`` tags the analytic form (0 polygon, 1 circle, 2 ellipse).
    """

    points: np.ndarray  # (P, 3) float64
    shape: int
    length: float
    area: float


@dataclass(frozen=True)
class Section:
    """Planar section curves of a solid (``cc_section_plane``)."""

    loops: list[SectionLoop]
    total_length: float
    total_area: float

    @property
    def loop_count(self) -> int:
        return len(self.loops)


@dataclass(frozen=True)
class Drawing:
    """Orthographic hidden-line-removal result: disjoint visible + hidden
    2-D drawing-plane segments (each ``(4,)`` = ``[ax, ay, bx, by]`` in mm)."""

    visible: np.ndarray  # (V, 4) float64
    hidden: np.ndarray  # (H, 4) float64

    @property
    def visible_count(self) -> int:
        return int(self.visible.shape[0])

    @property
    def hidden_count(self) -> int:
        return int(self.hidden.shape[0])


# ── ctypes marshalling helpers ─────────────────────────────────────────────────


def _c_double_array(values: Sequence[float]) -> ctypes.Array:
    """Pack a flat sequence of floats into a ``c_double`` array."""
    n = len(values)
    arr = (ctypes.c_double * n)(*[float(v) for v in values])
    return arr


def _c_int_array(values: Sequence[int]) -> ctypes.Array:
    """Pack a sequence of ints into a ``c_int`` array."""
    n = len(values)
    arr = (ctypes.c_int * n)(*[int(v) for v in values])
    return arr


def _flatten_xy(points: Iterable[Sequence[float]]) -> tuple[ctypes.Array, int]:
    """Flatten an iterable of ``(x, y)`` pairs to a c_double array + point count."""
    flat: list[float] = []
    n = 0
    for p in points:
        flat.append(float(p[0]))
        flat.append(float(p[1]))
        n += 1
    return _c_double_array(flat), n


def _flatten_xyz(points: Iterable[Sequence[float]]) -> tuple[ctypes.Array, int]:
    """Flatten an iterable of ``(x, y, z)`` triples to a c_double array + count."""
    flat: list[float] = []
    n = 0
    for p in points:
        flat.append(float(p[0]))
        flat.append(float(p[1]))
        flat.append(float(p[2]))
        n += 1
    return _c_double_array(flat), n


def _mesh_from_ccmesh(cm: CCMesh) -> Mesh:
    """Copy an engine-owned :class:`CCMesh` into NumPy arrays (no ownership taken).

    The caller remains responsible for freeing ``cm`` with ``cc_mesh_free``.
    """
    vcount = int(cm.vertexCount)
    tcount = int(cm.triangleCount)
    if vcount > 0 and cm.vertices:
        verts = np.ctypeslib.as_array(cm.vertices, shape=(vcount * 3,)).copy()
    else:
        verts = np.zeros(0, dtype=np.float64)
    if tcount > 0 and cm.triangles:
        tris = np.ctypeslib.as_array(cm.triangles, shape=(tcount * 3,)).copy()
    else:
        tris = np.zeros(0, dtype=np.int32)
    return Mesh(vertices=verts, triangles=tris)


def _tet_mesh_from_cc(tm: CCTetMesh, op: str) -> TetMesh:
    """Copy an engine-owned :class:`CCTetMesh` into a :class:`TetMesh`, then free
    the C buffers. Raises on an empty result (honest decline / unavailable)."""
    try:
        ncount = int(tm.nodeCount)
        ecount = int(tm.elementCount)
        npe = int(tm.nodesPerElement) or 4
        if ncount <= 0 or ecount <= 0 or not tm.nodes or not tm.elements:
            raise _fail(op)
        nodes = (
            np.ctypeslib.as_array(tm.nodes, shape=(ncount * 3,)).copy().reshape(-1, 3)
        )
        elements = (
            np.ctypeslib.as_array(tm.elements, shape=(ecount * npe,))
            .copy()
            .astype(np.int32)
            .reshape(-1, npe)
        )
        return TetMesh(
            nodes=nodes,
            elements=elements,
            nodes_per_element=npe,
            order=int(tm.order),
        )
    finally:
        _cffi.lib().cc_tet_mesh_free(tm)


def _quality_from_cc(qr: CCQualityReport) -> QualityReport:
    """Copy an engine-owned :class:`CCQualityReport` into a :class:`QualityReport`,
    then free the flagged-element buffer."""
    try:
        below = int(qr.elements_below_threshold)
        if below > 0 and qr.flagged_elements:
            flagged = (
                np.ctypeslib.as_array(qr.flagged_elements, shape=(below,))
                .copy()
                .astype(np.int32)
            )
        else:
            flagged = np.zeros(0, dtype=np.int32)
        return QualityReport(
            min_dihedral_angle=float(qr.min_dihedral_angle),
            max_dihedral_angle=float(qr.max_dihedral_angle),
            min_scaled_jacobian=float(qr.min_scaled_jacobian),
            mean_scaled_jacobian=float(qr.mean_scaled_jacobian),
            max_aspect_ratio=float(qr.max_aspect_ratio),
            elements_below_threshold=below,
            flagged_elements=flagged,
            valid=bool(qr.valid),
        )
    finally:
        _cffi.lib().cc_quality_report_free(qr)


def _cc_tetmesh_from_tetmesh(tm: TetMesh) -> CCTetMesh:
    """Pack a Python :class:`TetMesh` back into a borrowed :class:`CCTetMesh`
    view (its buffers are owned by the NumPy arrays, which the caller must keep
    alive for the duration of the C call). Used to feed ``cc_mesh_quality``."""
    nodes = np.ascontiguousarray(tm.nodes, dtype=np.float64).reshape(-1)
    elems = np.ascontiguousarray(tm.elements, dtype=np.int32).reshape(-1)
    cc = CCTetMesh()
    cc.nodes = nodes.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
    cc.nodeCount = int(tm.node_count)
    cc.elements = elems.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
    cc.elementCount = int(tm.element_count)
    cc.nodesPerElement = int(tm.nodes_per_element)
    cc.order = int(tm.order)
    # keep the backing arrays alive on the struct so they outlive the C call
    cc._keepalive = (nodes, elems)  # type: ignore[attr-defined]
    return cc


# ── Shape ───────────────────────────────────────────────────────────────────


class Shape:
    """A kernel body: an owned handle to a :c:type:`CCShapeId`.

    A :class:`Shape` owns its handle and releases it exactly once — via
    :meth:`close`, on ``__exit__`` of a ``with`` block, or on garbage collection.
    Do not construct directly; obtain shapes from :class:`Kernel` factory methods
    (e.g. :meth:`Kernel.extrude`) or from operations on an existing shape (which
    always return a **new** :class:`Shape`). The originals are left untouched, so
    callers may keep chaining or release them independently.
    """

    __slots__ = ("_id", "_kernel", "_closed", "__weakref__")

    def __init__(self, kernel: "Kernel", shape_id: int) -> None:
        # Internal: `shape_id` must already be a valid (non-zero) id.
        self._kernel = kernel
        self._id = int(shape_id)
        self._closed = False

    # -- identity / lifetime -----------------------------------------------
    @property
    def id(self) -> int:
        """The raw :c:type:`CCShapeId`. Raises if the shape is closed."""
        if self._closed:
            raise CyberCadError("operation on a released Shape")
        return self._id

    @property
    def closed(self) -> bool:
        """``True`` once the underlying handle has been released."""
        return self._closed

    def close(self) -> None:
        """Release the underlying handle (idempotent)."""
        if not self._closed and self._id:
            try:
                _cffi.lib().cc_shape_release(self._id)
            finally:
                self._closed = True

    def __enter__(self) -> "Shape":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self) -> None:  # best-effort GC release
        try:
            self.close()
        except Exception:  # pragma: no cover - interpreter shutdown
            pass

    def __repr__(self) -> str:
        state = "closed" if self._closed else f"id={self._id}"
        return f"<Shape {state}>"

    # -- internal: wrap a returned id, raising on failure -------------------
    def _wrap(self, new_id: int, op: str) -> "Shape":
        return self._kernel._shape(new_id, op)

    # ── Feature edits ──────────────────────────────────────────────────────
    def fillet_edges(self, edge_ids: Sequence[int], radius: float) -> "Shape":
        """Constant-radius (G1) fillet on the given 1-based ``edge_ids``."""
        ids = _c_int_array(edge_ids)
        nid = _cffi.lib().cc_fillet_edges(self.id, ids, len(edge_ids), float(radius))
        return self._wrap(nid, "fillet_edges")

    def fillet_edges_variable(
        self, edge_ids: Sequence[int], radius1: float, radius2: float
    ) -> "Shape":
        """Variable-radius fillet ramping from ``radius1`` to ``radius2``."""
        ids = _c_int_array(edge_ids)
        nid = _cffi.lib().cc_fillet_edges_variable(
            self.id, ids, len(edge_ids), float(radius1), float(radius2)
        )
        return self._wrap(nid, "fillet_edges_variable")

    def fillet_edges_g2(self, edge_ids: Sequence[int], radius: float) -> "Shape":
        """Curvature-continuous (G2, best-achievable) blend on ``edge_ids``."""
        ids = _c_int_array(edge_ids)
        nid = _cffi.lib().cc_fillet_edges_g2(
            self.id, ids, len(edge_ids), float(radius)
        )
        return self._wrap(nid, "fillet_edges_g2")

    def chamfer_edges(self, edge_ids: Sequence[int], distance: float) -> "Shape":
        """Chamfer the given edges by ``distance``."""
        ids = _c_int_array(edge_ids)
        nid = _cffi.lib().cc_chamfer_edges(
            self.id, ids, len(edge_ids), float(distance)
        )
        return self._wrap(nid, "chamfer_edges")

    def chamfer_edges_asym(
        self, edge_ids: Sequence[int], distance1: float, distance2: float
    ) -> "Shape":
        """Asymmetric two-distance chamfer (``distance1`` on the first / wall
        face, ``distance2`` on the second / cap face)."""
        ids = _c_int_array(edge_ids)
        nid = _cffi.lib().cc_chamfer_edges_asym(
            self.id, ids, len(edge_ids), float(distance1), float(distance2)
        )
        return self._wrap(nid, "chamfer_edges_asym")

    def draft_faces(
        self,
        face_ids: Sequence[int],
        neutral_origin: Sequence[float],
        pull_dir: Sequence[float],
        angle_deg: float,
    ) -> "Shape":
        """Taper the given planar side faces about the neutral plane for mold
        release. A positive ``angle_deg`` draws material in as the face recedes
        from the neutral plane along ``+pull_dir``."""
        ids = _c_int_array(face_ids)
        o = _c_double_array([neutral_origin[0], neutral_origin[1], neutral_origin[2]])
        p = _c_double_array([pull_dir[0], pull_dir[1], pull_dir[2]])
        nid = _cffi.lib().cc_draft_faces(
            self.id, ids, len(face_ids), o, p, float(angle_deg)
        )
        return self._wrap(nid, "draft_faces")

    def shell(self, face_ids: Sequence[int], thickness: float) -> "Shape":
        """Hollow the solid, removing the given faces, with wall ``thickness``."""
        ids = _c_int_array(face_ids)
        nid = _cffi.lib().cc_shell(self.id, ids, len(face_ids), float(thickness))
        return self._wrap(nid, "shell")

    # ── Sheet metal (native engine only) ─────────────────────────────────────
    def sheet_edge_flange(
        self, edge_id: int, height: float, bend_radius: float, angle_deg: float
    ) -> "Shape":
        """Add a bent flange off a straight rim ``edge_id`` of a base flange.

        Native engine only (``kernel.engine = "native"``).
        """
        nid = _cffi.lib().cc_sheet_edge_flange(
            self.id, int(edge_id), float(height), float(bend_radius), float(angle_deg)
        )
        return self._wrap(nid, "sheet_edge_flange")

    def sheet_unfold(self, k_factor: float = 0.4) -> "Shape":
        """Flat-pattern unfold of a single-bend sheet-metal part.

        Native engine only; ``k_factor`` ∈ [0, 1] is the neutral-fibre position.
        """
        nid = _cffi.lib().cc_sheet_unfold(self.id, float(k_factor))
        return self._wrap(nid, "sheet_unfold")

    def offset_face(self, face_id: int, distance: float) -> "Shape":
        """Offset a single face by ``distance`` (positive = outward)."""
        nid = _cffi.lib().cc_offset_face(self.id, int(face_id), float(distance))
        return self._wrap(nid, "offset_face")

    def fillet_face(self, face_id: int, radius: float) -> "Shape":
        """Fillet all edges of a face at ``radius``."""
        nid = _cffi.lib().cc_fillet_face(self.id, int(face_id), float(radius))
        return self._wrap(nid, "fillet_face")

    def full_round_fillet(self, face_id: int) -> "Shape":
        """Replace a narrow face with a rolling-ball blend (auto neighbours)."""
        nid = _cffi.lib().cc_full_round_fillet(self.id, int(face_id))
        return self._wrap(nid, "full_round_fillet")

    def full_round_fillet_faces(
        self, left_face_id: int, middle_face_id: int, right_face_id: int
    ) -> "Shape":
        """Rolling-ball blend consuming ``middle_face_id`` between the two sides."""
        nid = _cffi.lib().cc_full_round_fillet_faces(
            self.id, int(left_face_id), int(middle_face_id), int(right_face_id)
        )
        return self._wrap(nid, "full_round_fillet_faces")

    def split_plane(
        self,
        origin: Sequence[float],
        normal: Sequence[float],
        keep_positive: bool = True,
    ) -> "Shape":
        """Split by a plane; keep the half on the ``+normal`` side if requested."""
        o = origin
        n = normal
        nid = _cffi.lib().cc_split_plane(
            self.id,
            float(o[0]), float(o[1]), float(o[2]),
            float(n[0]), float(n[1]), float(n[2]),
            1 if keep_positive else 0,
        )
        return self._wrap(nid, "split_plane")

    # ── Booleans ────────────────────────────────────────────────────────────
    def boolean(self, other: "Shape", op: int) -> "Shape":
        """Raw boolean with ``other`` using a :class:`BooleanOp` selector."""
        nid = _cffi.lib().cc_boolean(self.id, other.id, int(op))
        return self._wrap(nid, "boolean")

    def fuse(self, other: "Shape") -> "Shape":
        """Union of ``self`` and ``other``."""
        return self.boolean(other, BooleanOp.FUSE)

    def cut(self, other: "Shape") -> "Shape":
        """Subtract ``other`` from ``self``."""
        return self.boolean(other, BooleanOp.CUT)

    def common(self, other: "Shape") -> "Shape":
        """Intersection of ``self`` and ``other``."""
        return self.boolean(other, BooleanOp.COMMON)

    def thread_apply(self, thread: "Shape", op: int) -> "Shape":
        """Robust segmented boolean of a helical ``thread`` onto this shaft.

        ``op`` = :attr:`BooleanOp.FUSE` for an external thread,
        :attr:`BooleanOp.CUT` for an internal thread.
        """
        nid = _cffi.lib().cc_thread_apply(self.id, thread.id, int(op))
        return self._wrap(nid, "thread_apply")

    # ── Transforms (all return a new Shape) ──────────────────────────────────
    def translate(self, tx: float, ty: float, tz: float) -> "Shape":
        """Translate by ``(tx, ty, tz)``."""
        nid = _cffi.lib().cc_translate_shape(
            self.id, float(tx), float(ty), float(tz)
        )
        return self._wrap(nid, "translate")

    def scale(self, factor: float) -> "Shape":
        """Uniform scale about the origin."""
        nid = _cffi.lib().cc_scale_shape(self.id, float(factor))
        return self._wrap(nid, "scale")

    def scale_about(
        self, center: Sequence[float], factor: float
    ) -> "Shape":
        """Uniform scale about ``center``."""
        c = center
        nid = _cffi.lib().cc_scale_shape_about(
            self.id, float(c[0]), float(c[1]), float(c[2]), float(factor)
        )
        return self._wrap(nid, "scale_about")

    def rotate(
        self,
        center: Sequence[float],
        axis: Sequence[float],
        angle_radians: float,
    ) -> "Shape":
        """Rotate ``angle_radians`` about the line through ``center`` along ``axis``."""
        c = center
        a = axis
        nid = _cffi.lib().cc_rotate_shape_about(
            self.id,
            float(c[0]), float(c[1]), float(c[2]),
            float(a[0]), float(a[1]), float(a[2]),
            float(angle_radians),
        )
        return self._wrap(nid, "rotate")

    def mirror(
        self, point: Sequence[float], normal: Sequence[float]
    ) -> "Shape":
        """Mirror across the plane at ``point`` with the given ``normal``."""
        p = point
        n = normal
        nid = _cffi.lib().cc_mirror_shape(
            self.id,
            float(p[0]), float(p[1]), float(p[2]),
            float(n[0]), float(n[1]), float(n[2]),
        )
        return self._wrap(nid, "mirror")

    def place_on_frame(
        self,
        origin: Sequence[float],
        u_axis: Sequence[float],
        v_axis: Sequence[float],
    ) -> "Shape":
        """Reposition onto the frame defined by ``origin`` and the ``u``/``v`` axes."""
        o, u, v = origin, u_axis, v_axis
        nid = _cffi.lib().cc_place_on_frame(
            self.id,
            float(o[0]), float(o[1]), float(o[2]),
            float(u[0]), float(u[1]), float(u[2]),
            float(v[0]), float(v[1]), float(v[2]),
        )
        return self._wrap(nid, "place_on_frame")

    # ── Queries ──────────────────────────────────────────────────────────────
    def mass_properties(self) -> MassProps:
        """Exact B-rep mass properties (raises if the engine reports invalid)."""
        mp: CCMassProps = _cffi.lib().cc_mass_properties(self.id)
        if not mp.valid:
            raise _fail("mass_properties")
        return MassProps(
            volume=float(mp.volume),
            area=float(mp.area),
            center_of_mass=(float(mp.cx), float(mp.cy), float(mp.cz)),
            valid=True,
        )

    def principal_moments(self) -> tuple[float, float, float]:
        """Principal moments of inertia ``(I1, I2, I3)`` (unit-density)."""
        out3 = (ctypes.c_double * 3)()
        ok = _cffi.lib().cc_principal_moments(self.id, out3)
        if not ok:
            raise _fail("principal_moments")
        return (float(out3[0]), float(out3[1]), float(out3[2]))

    def check_solid(self) -> ValidityReport:
        """Structural-validity report of the solid (raises on an honest decline)."""
        out = CCValidityReport()
        ok = _cffi.lib().cc_check_solid(self.id, ctypes.byref(out))
        if not ok:
            raise _fail("check_solid")
        return ValidityReport(
            valid=bool(out.valid),
            decided=bool(out.decided),
            finite=bool(out.finite),
            closed_manifold=bool(out.closed_manifold),
            consistent_orientation=bool(out.consistent_orientation),
            no_degenerate=bool(out.no_degenerate),
            no_self_intersection=bool(out.no_self_intersection),
            first_failure=int(out.first_failure),
        )

    def interference(self, other: "Shape") -> Interference:
        """Clash / touching / clear verdict against ``other`` (raises on decline)."""
        out = CCInterference()
        ok = _cffi.lib().cc_interference(self.id, other.id, ctypes.byref(out))
        if not ok:
            raise _fail("interference")
        has_w = bool(out.has_witness)
        return Interference(
            state=int(out.state),
            clash=bool(out.clash),
            decided=bool(out.decided),
            overlap_volume=float(out.overlap_volume),
            min_distance=float(out.min_distance),
            has_witness=has_w,
            witness_lo=tuple(out.witness_lo) if has_w else None,
            witness_hi=tuple(out.witness_hi) if has_w else None,
            witness_point=tuple(out.witness_point) if has_w else None,
        )

    def project_point_on_face(
        self, face_id: int, point: Sequence[float]
    ) -> Projection:
        """Foot-of-perpendicular of ``point`` onto face ``face_id``'s surface.

        Raises on an honest decline (cone / torus / freeform / ambiguous pose).
        """
        pr: CCProjection = _cffi.lib().cc_project_point_on_face(
            self.id, int(face_id), float(point[0]), float(point[1]), float(point[2])
        )
        if not pr.valid:
            raise _fail("project_point_on_face")
        return Projection(
            foot=(float(pr.footX), float(pr.footY), float(pr.footZ)),
            distance=float(pr.distance),
            valid=True,
        )

    # ── Connected-solid enumeration ──────────────────────────────────────────
    def solid_count(self) -> int:
        """Number of connected solid lumps in this body."""
        return int(_cffi.lib().cc_shape_solid_count(self.id))

    def solid_at(self, index: int) -> "Shape":
        """The ``index``-th connected solid lump as its own :class:`Shape`."""
        nid = _cffi.lib().cc_shape_solid_at(self.id, int(index))
        return self._wrap(nid, "solid_at")

    def solids(self) -> list["Shape"]:
        """Every connected solid lump as an independent :class:`Shape`."""
        return [self.solid_at(i) for i in range(self.solid_count())]

    # ── Measurement & curvature (requires a CYBERCAD_HAS_NUMSCI build) ────────
    def measure_distance(
        self, kind_a: int, id_a: int, kind_b: int, id_b: int
    ) -> tuple[float, tuple[float, float, float], tuple[float, float, float]]:
        """Minimum distance between two sub-shapes.

        Returns ``(gap, witness_on_a, witness_on_b)``. ``kind_*`` uses
        :class:`SubShapeKind`. Raises on an honest decline (or on a build without
        the numerical-science backend).
        """
        out7 = (ctypes.c_double * 7)()
        ok = _cffi.lib().cc_measure_distance(
            self.id, int(kind_a), int(id_a), int(kind_b), int(id_b), out7
        )
        if not ok:
            raise _fail("measure_distance")
        return (
            float(out7[0]),
            (float(out7[1]), float(out7[2]), float(out7[3])),
            (float(out7[4]), float(out7[5]), float(out7[6])),
        )

    def measure_angle(
        self, kind_a: int, id_a: int, kind_b: int, id_b: int
    ) -> float:
        """Angle (radians) between two line/plane sub-shapes. Raises on decline."""
        out = ctypes.c_double()
        ok = _cffi.lib().cc_measure_angle(
            self.id, int(kind_a), int(id_a), int(kind_b), int(id_b), ctypes.byref(out)
        )
        if not ok:
            raise _fail("measure_angle")
        return float(out.value)

    def surface_curvature(
        self, face_id: int, u: float, v: float
    ) -> tuple[float, float, float, float]:
        """Surface curvature ``(K, H, k1, k2)`` at face ``face_id`` param ``(u, v)``."""
        out4 = (ctypes.c_double * 4)()
        ok = _cffi.lib().cc_surface_curvature(
            self.id, int(face_id), float(u), float(v), out4
        )
        if not ok:
            raise _fail("surface_curvature")
        return (float(out4[0]), float(out4[1]), float(out4[2]), float(out4[3]))

    def edge_curvature(self, edge_id: int, t: float) -> float:
        """Edge curvature κ at edge ``edge_id`` parameter ``t``. Raises on decline."""
        out = ctypes.c_double()
        ok = _cffi.lib().cc_edge_curvature(
            self.id, int(edge_id), float(t), ctypes.byref(out)
        )
        if not ok:
            raise _fail("edge_curvature")
        return float(out.value)

    def bounding_box(self) -> BoundingBox:
        """Exact axis-aligned bounding box of the B-rep."""
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_bounding_box(self.id, out6)
        if not ok:
            raise _fail("bounding_box")
        return BoundingBox(
            min=(float(out6[0]), float(out6[1]), float(out6[2])),
            max=(float(out6[3]), float(out6[4]), float(out6[5])),
        )

    def face_axis(self, face_id: int) -> ReferenceAxis:
        """Axis of a cylindrical / conical face (raises if the face is not one)."""
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_face_axis(self.id, int(face_id), out6)
        if not ok:
            raise _fail("face_axis")
        return ReferenceAxis(
            origin=(float(out6[0]), float(out6[1]), float(out6[2])),
            direction=(float(out6[3]), float(out6[4]), float(out6[5])),
        )

    def subshape_ids(self, kind: int) -> list[int]:
        """Stable sub-shape ids for picking (``kind`` from :class:`SubShapeKind`)."""
        out_ptr = ctypes.POINTER(ctypes.c_int)()
        count = _cffi.lib().cc_subshape_ids(
            self.id, int(kind), ctypes.byref(out_ptr)
        )
        if count < 0:
            raise _fail("subshape_ids")
        try:
            if count == 0 or not out_ptr:
                return []
            return [int(out_ptr[i]) for i in range(count)]
        finally:
            if out_ptr:
                _cffi.lib().cc_ints_free(out_ptr)

    def edge_ids(self) -> list[int]:
        """Convenience: all 1-based edge ids."""
        return self.subshape_ids(SubShapeKind.EDGE)

    def face_ids(self) -> list[int]:
        """Convenience: all 1-based face ids."""
        return self.subshape_ids(SubShapeKind.FACE)

    def vertex_ids(self) -> list[int]:
        """Convenience: all 1-based vertex ids."""
        return self.subshape_ids(SubShapeKind.VERTEX)

    def tangent_chain(self, edge_ids: Sequence[int]) -> list[int]:
        """Extend the given seed edges along tangent continuity."""
        ids = _c_int_array(edge_ids)
        out_ptr = ctypes.POINTER(ctypes.c_int)()
        count = _cffi.lib().cc_tangent_chain(
            self.id, ids, len(edge_ids), ctypes.byref(out_ptr)
        )
        if count < 0:
            raise _fail("tangent_chain")
        try:
            if count == 0 or not out_ptr:
                return []
            return [int(out_ptr[i]) for i in range(count)]
        finally:
            if out_ptr:
                _cffi.lib().cc_ints_free(out_ptr)

    def outer_rim_chain(self, edge_ids: Sequence[int]) -> list[int]:
        """Return the outer-rim edge loop reachable from the seed edges."""
        ids = _c_int_array(edge_ids)
        out_ptr = ctypes.POINTER(ctypes.c_int)()
        count = _cffi.lib().cc_outer_rim_chain(
            self.id, ids, len(edge_ids), ctypes.byref(out_ptr)
        )
        if count < 0:
            raise _fail("outer_rim_chain")
        try:
            if count == 0 or not out_ptr:
                return []
            return [int(out_ptr[i]) for i in range(count)]
        finally:
            if out_ptr:
                _cffi.lib().cc_ints_free(out_ptr)

    def edge_polylines(self) -> list[EdgePolyline]:
        """Every B-rep edge discretized to a single 3-D polyline."""
        out_ptr = ctypes.POINTER(CCEdgePolyline)()
        count = _cffi.lib().cc_edge_polylines(self.id, ctypes.byref(out_ptr))
        if count < 0:
            raise _fail("edge_polylines")
        try:
            result: list[EdgePolyline] = []
            for i in range(count):
                e = out_ptr[i]
                pcount = int(e.pointCount)
                if pcount > 0 and e.points:
                    pts = np.ctypeslib.as_array(
                        e.points, shape=(pcount * 3,)
                    ).copy().reshape(-1, 3)
                else:
                    pts = np.zeros((0, 3), dtype=np.float64)
                result.append(EdgePolyline(edge_id=int(e.edgeId), points=pts))
            return result
        finally:
            if out_ptr and count > 0:
                _cffi.lib().cc_edge_polylines_free(out_ptr, count)

    def offset_face_boundary(
        self, face_id: int, distance: float
    ) -> np.ndarray:
        """Offset a face's outer boundary in-plane -> an ``(P, 3)`` polyline.

        Negative ``distance`` offsets inward.
        """
        out_ptr = ctypes.POINTER(ctypes.c_double)()
        count = _cffi.lib().cc_offset_face_boundary(
            self.id, int(face_id), float(distance), ctypes.byref(out_ptr)
        )
        if count < 0:
            raise _fail("offset_face_boundary")
        try:
            if count == 0 or not out_ptr:
                return np.zeros((0, 3), dtype=np.float64)
            flat = np.ctypeslib.as_array(out_ptr, shape=(count * 3,)).copy()
            return flat.reshape(-1, 3)
        finally:
            if out_ptr and count > 0:
                _cffi.lib().cc_points_free(out_ptr)

    def face_meshes(self, deflection: float = 0.1) -> list[FaceMesh]:
        """Tessellate each face separately, tagged with its 1-based ``face_id``."""
        out_ptr = ctypes.POINTER(CCFaceMesh)()
        count = _cffi.lib().cc_face_meshes(
            self.id, float(deflection), ctypes.byref(out_ptr)
        )
        if count < 0:
            raise _fail("face_meshes")
        try:
            result: list[FaceMesh] = []
            for i in range(count):
                fm = out_ptr[i]
                vcount = int(fm.vertexCount)
                tcount = int(fm.triangleCount)
                verts = (
                    np.ctypeslib.as_array(fm.vertices, shape=(vcount * 3,)).copy()
                    if vcount > 0 and fm.vertices
                    else np.zeros(0, dtype=np.float64)
                )
                tris = (
                    np.ctypeslib.as_array(fm.triangles, shape=(tcount * 3,)).copy()
                    if tcount > 0 and fm.triangles
                    else np.zeros(0, dtype=np.int32)
                )
                result.append(
                    FaceMesh(
                        face_id=int(fm.faceId),
                        mesh=Mesh(vertices=verts, triangles=tris),
                    )
                )
            return result
        finally:
            if out_ptr and count > 0:
                _cffi.lib().cc_face_meshes_free(out_ptr, count)

    # ── Reference geometry derived from this body ────────────────────────────
    def ref_plane_from_face(self, face_id: int) -> ReferencePlane:
        """Datum plane coincident with a planar face."""
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_ref_plane_from_face(self.id, int(face_id), out6)
        if not ok:
            raise _fail("ref_plane_from_face")
        return ReferencePlane(
            origin=(float(out6[0]), float(out6[1]), float(out6[2])),
            normal=(float(out6[3]), float(out6[4]), float(out6[5])),
        )

    def ref_axis_from_edge(self, edge_id: int) -> ReferenceAxis:
        """Datum axis along a straight edge."""
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_ref_axis_from_edge(self.id, int(edge_id), out6)
        if not ok:
            raise _fail("ref_axis_from_edge")
        return ReferenceAxis(
            origin=(float(out6[0]), float(out6[1]), float(out6[2])),
            direction=(float(out6[3]), float(out6[4]), float(out6[5])),
        )

    def ref_axis_from_face(self, face_id: int) -> ReferenceAxis:
        """Datum axis along a cylindrical / conical face's axis."""
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_ref_axis_from_face(self.id, int(face_id), out6)
        if not ok:
            raise _fail("ref_axis_from_face")
        return ReferenceAxis(
            origin=(float(out6[0]), float(out6[1]), float(out6[2])),
            direction=(float(out6[3]), float(out6[4]), float(out6[5])),
        )

    # ── Tessellation / export ────────────────────────────────────────────────
    def tessellate(self, deflection: float = 0.1) -> Mesh:
        """Tessellate the whole body to a display :class:`Mesh` at ``deflection`` mm."""
        cm: CCMesh = _cffi.lib().cc_tessellate(self.id, float(deflection))
        try:
            mesh = _mesh_from_ccmesh(cm)
        finally:
            _cffi.lib().cc_mesh_free(cm)
        if mesh.is_empty():
            # An empty tessellation of a real body signals a failure.
            err = _last_error()
            if err:
                raise CyberCadError(f"tessellate failed: {err}")
        return mesh

    def display_mesh(
        self,
        deflection: float = 0.1,
        crease_angle_deg: float = 30.0,
        lod_target_tris: int = 0,
        want_uvs: bool = False,
    ) -> DisplayMesh:
        """Render-quality display mesh: smooth per-vertex normals with
        crease-angle hard edges, optional UVs and LOD decimation."""
        out = CCDisplayMesh()
        n = _cffi.lib().cc_display_mesh(
            self.id,
            float(deflection),
            float(crease_angle_deg),
            int(lod_target_tris),
            1 if want_uvs else 0,
            ctypes.byref(out),
        )
        if n == 0:
            raise _fail("display_mesh")
        try:
            vcount = int(out.vertexCount)
            tcount = int(out.triangleCount)
            pos = np.ctypeslib.as_array(out.positions, shape=(vcount * 3,)).copy()
            nrm = np.ctypeslib.as_array(out.normals, shape=(vcount * 3,)).copy()
            tris = np.ctypeslib.as_array(out.triangles, shape=(tcount * 3,)).copy()
            uvs = None
            if out.uvs:
                uvs = (
                    np.ctypeslib.as_array(out.uvs, shape=(vcount * 2,))
                    .copy()
                    .reshape(-1, 2)
                )
            return DisplayMesh(
                positions=pos.reshape(-1, 3),
                normals=nrm.reshape(-1, 3),
                triangles=tris.astype(np.int32).reshape(-1, 3),
                uvs=uvs,
            )
        finally:
            _cffi.lib().cc_display_mesh_free(ctypes.byref(out))

    def section_plane(
        self, origin: Sequence[float], normal: Sequence[float]
    ) -> Section:
        """Planar section curves where a cut plane carves this solid.

        Native engine only (``kernel.engine = "native"``). Raises on an honest
        decline / empty section.
        """
        o = _c_double_array([origin[0], origin[1], origin[2]])
        n = _c_double_array([normal[0], normal[1], normal[2]])
        sec: CCSection = _cffi.lib().cc_section_plane(self.id, o, n)
        try:
            if sec.loopCount <= 0 or not sec.loops:
                raise _fail("section_plane")
            loops: list[SectionLoop] = []
            for i in range(int(sec.loopCount)):
                lp = sec.loops[i]
                pc = int(lp.pointCount)
                if pc > 0 and lp.pointsXYZ:
                    pts = (
                        np.ctypeslib.as_array(lp.pointsXYZ, shape=(pc * 3,))
                        .copy()
                        .reshape(-1, 3)
                    )
                else:
                    pts = np.zeros((0, 3), dtype=np.float64)
                loops.append(
                    SectionLoop(
                        points=pts,
                        shape=int(lp.shape),
                        length=float(lp.length),
                        area=float(lp.area),
                    )
                )
            return Section(
                loops=loops,
                total_length=float(sec.totalLength),
                total_area=float(sec.totalArea),
            )
        finally:
            _cffi.lib().cc_section_free(sec)

    def hlr_project(
        self,
        view_dir: Sequence[float],
        up: Sequence[float],
        deflection: float = 0.0,
        samples_per_edge: int = 0,
        surface_offset: float = 0.0,
    ) -> Drawing:
        """Orthographic hidden-line-removal onto a drawing plane.

        ``view_dir`` is the camera direction; ``up`` an up hint (not parallel).
        Returns disjoint visible + hidden 2-D segment sets. Raises on an honest
        decline (a curved-silhouette / freeform body in this slice).
        """
        vd = _c_double_array([view_dir[0], view_dir[1], view_dir[2]])
        u = _c_double_array([up[0], up[1], up[2]])
        opts = CCHlrOptions(
            deflection=float(deflection),
            samplesPerEdge=int(samples_per_edge),
            surfaceOffset=float(surface_offset),
        )
        dr: CCDrawing = _cffi.lib().cc_hlr_project(self.id, vd, u, opts)
        try:

            def _segs(ptr, count):
                count = int(count)
                if count <= 0 or not ptr:
                    return np.zeros((0, 4), dtype=np.float64)
                out = np.empty((count, 4), dtype=np.float64)
                for i in range(count):
                    s = ptr[i]
                    out[i] = (s.ax, s.ay, s.bx, s.by)
                return out

            visible = _segs(dr.visible, dr.visibleCount)
            hidden = _segs(dr.hidden, dr.hiddenCount)
            if visible.shape[0] == 0 and hidden.shape[0] == 0:
                raise _fail("hlr_project")
            return Drawing(visible=visible, hidden=hidden)
        finally:
            _cffi.lib().cc_drawing_free(dr)

    def tet_mesh(
        self,
        deflection: float = 0.5,
        order: int = 10,
        target_element_size: float = 0.0,
        grading: float = 1.4,
        min_scaled_jacobian: float = 0.02,
    ) -> TetMesh:
        """Tetrahedral volume mesh of this body.

        Requires a ``CYBERCAD_HAS_TETGEN`` build; raises :class:`CyberCadError`
        (honest "tet meshing unavailable" decline) otherwise.
        """
        opts = CCVolumeMeshOptions(
            order=int(order),
            target_element_size=float(target_element_size),
            grading=float(grading),
            min_scaled_jacobian=float(min_scaled_jacobian),
        )
        tm: CCTetMesh = _cffi.lib().cc_tet_mesh(self.id, float(deflection), opts)
        return _tet_mesh_from_cc(tm, "tet_mesh")

    def gltf_export(
        self, path: str, deflection: float = 0.1, glb: bool = True
    ) -> None:
        """Export this body as glTF 2.0. ``glb=True`` writes a binary .glb,
        ``glb=False`` a self-contained .gltf JSON. Raises on failure."""
        ok = _cffi.lib().cc_gltf_export(
            self.id, str(path).encode("utf-8"), float(deflection), 1 if glb else 0
        )
        if not ok:
            raise _fail("gltf_export")

    def usdz_export(self, path: str, deflection: float = 0.1) -> None:
        """Export this body as a USDZ package (Apple QuickLook AR). Raises on
        failure."""
        ok = _cffi.lib().cc_usdz_export(
            self.id, str(path).encode("utf-8"), float(deflection)
        )
        if not ok:
            raise _fail("usdz_export")

    def step_export(self, path: str) -> None:
        """Write this body to a STEP file (raises on failure)."""
        ok = _cffi.lib().cc_step_export(self.id, str(path).encode("utf-8"))
        if not ok:
            raise _fail("step_export")

    def iges_export(self, path: str) -> None:
        """Write this body to an IGES file (raises on failure)."""
        ok = _cffi.lib().cc_iges_export(self.id, str(path).encode("utf-8"))
        if not ok:
            raise _fail("iges_export")

    def stl_export(
        self, path: str, deflection: float = 0.1, binary: bool = True
    ) -> None:
        """Write this body's tessellated triangle mesh to an STL file.

        ``deflection`` is the chord tolerance (mm) for tessellation; ``binary``
        selects binary STL (default) versus ASCII. Raises on failure.
        """
        ok = _cffi.lib().cc_stl_export(
            self.id,
            str(path).encode("utf-8"),
            float(deflection),
            1 if binary else 0,
        )
        if not ok:
            raise _fail("stl_export")


# ── Kernel facade ─────────────────────────────────────────────────────────────


class Kernel:
    """Facade over the kernel: engine state and shape factories.

    A :class:`Kernel` is a thin, stateless handle onto the singleton native
    library (loaded lazily by :mod:`cybercadkernel._cffi`). Constructing one
    triggers the ``dlopen`` and, by default, asserts that a B-rep engine (OCCT)
    is actually linked so failures surface immediately rather than as empty
    geometry later.
    """

    def __init__(self, require_brep: bool = True) -> None:
        # Force the library to load now so KernelLibraryNotFound surfaces here.
        _cffi.lib()
        if require_brep and not self.brep_available:
            raise CyberCadError(
                "no B-rep engine is linked into libcybercadkernel "
                "(cc_brep_available() == 0)"
            )

    # ── Engine state ────────────────────────────────────────────────────────
    @property
    def brep_available(self) -> bool:
        """``True`` when a B-rep engine (OCCT) is linked in."""
        return bool(_cffi.lib().cc_brep_available())

    @property
    def last_error(self) -> str:
        """The engine's thread-local last-error string (``""`` if none)."""
        return _last_error()

    @property
    def parallel(self) -> bool:
        """Whether multi-core execution of boolean/mesh ops is enabled."""
        return bool(_cffi.lib().cc_parallel_enabled())

    @parallel.setter
    def parallel(self, enabled: bool) -> None:
        _cffi.lib().cc_set_parallel(1 if enabled else 0)

    def set_parallel(self, enabled: bool) -> None:
        """Toggle multi-core execution (alias for the :attr:`parallel` setter)."""
        self.parallel = enabled

    @property
    def gpu_tessellation(self) -> bool:
        """Whether the GPU tessellation path is on **and** available."""
        return bool(_cffi.lib().cc_gpu_tessellation_enabled())

    @gpu_tessellation.setter
    def gpu_tessellation(self, enabled: bool) -> None:
        _cffi.lib().cc_set_gpu_tessellation(1 if enabled else 0)

    def set_gpu_tessellation(self, enabled: bool) -> None:
        """Toggle the GPU tessellation path (no-op on builds without Metal)."""
        self.gpu_tessellation = enabled

    # ── Active engine selection (OCCT default vs the native C++20 engine) ─────
    @property
    def engine(self) -> str:
        """The active engine: ``"native"`` if the NativeEngine is on, else
        ``"occt"`` (the build default)."""
        return "native" if _cffi.lib().cc_active_engine() else "occt"

    @engine.setter
    def engine(self, which: str) -> None:
        if which not in ("native", "occt"):
            raise ValueError("engine must be 'native' or 'occt'")
        _cffi.lib().cc_set_engine(1 if which == "native" else 0)

    def set_engine(self, native: bool) -> None:
        """Activate the native engine (``True``) or restore OCCT (``False``).

        Some features (sheet metal, planar section curves, native booleans) are
        native-only; call ``set_engine(True)`` before using them and build the
        operands under the same engine.
        """
        _cffi.lib().cc_set_engine(1 if native else 0)

    # ── internal: adopt a returned id, raising on 0 ──────────────────────────
    def _shape(self, shape_id: int, op: str) -> Shape:
        """Wrap a returned :c:type:`CCShapeId`, raising :class:`CyberCadError` on 0."""
        if not shape_id:
            raise _fail(op)
        return Shape(self, int(shape_id))

    # ── Construction ──────────────────────────────────────────────────────────
    def extrude(
        self, profile_xy: Iterable[Sequence[float]], depth: float
    ) -> Shape:
        """Extrude a closed 2-D ``(x, y)`` profile along +Z by ``depth``."""
        arr, n = _flatten_xy(profile_xy)
        return self._shape(
            _cffi.lib().cc_solid_extrude(arr, n, float(depth)), "extrude"
        )

    def revolve(
        self, profile_xy: Iterable[Sequence[float]], angle_radians: float
    ) -> Shape:
        """Revolve a closed 2-D ``(x, y)`` profile about the Y axis."""
        arr, n = _flatten_xy(profile_xy)
        return self._shape(
            _cffi.lib().cc_solid_revolve(arr, n, float(angle_radians)), "revolve"
        )

    def loft(
        self,
        bottom_xy: Iterable[Sequence[float]],
        top_xy: Iterable[Sequence[float]],
        depth: float,
    ) -> Shape:
        """Loft between a bottom and a top 2-D profile separated by ``depth``."""
        b, bn = _flatten_xy(bottom_xy)
        t, tn = _flatten_xy(top_xy)
        return self._shape(
            _cffi.lib().cc_solid_loft(b, bn, t, tn, float(depth)), "loft"
        )

    def loft_wires(
        self,
        wire_a_xyz: Iterable[Sequence[float]],
        wire_b_xyz: Iterable[Sequence[float]],
    ) -> Shape:
        """Loft between two 3-D wires given as ``(x, y, z)`` point lists."""
        a, an = _flatten_xyz(wire_a_xyz)
        b, bn = _flatten_xyz(wire_b_xyz)
        return self._shape(
            _cffi.lib().cc_solid_loft_wires(a, an, b, bn), "loft_wires"
        )

    def sweep(
        self,
        profile_xy: Iterable[Sequence[float]],
        path_xyz: Iterable[Sequence[float]],
    ) -> Shape:
        """Sweep a 2-D profile along a 3-D path."""
        p, pn = _flatten_xy(profile_xy)
        q, qn = _flatten_xyz(path_xyz)
        return self._shape(
            _cffi.lib().cc_solid_sweep(p, pn, q, qn), "sweep"
        )

    def twisted_sweep(
        self,
        profile_xy: Iterable[Sequence[float]],
        path_xyz: Iterable[Sequence[float]],
        twist_radians: float,
        scale_end: float,
    ) -> Shape:
        """Sweep with a twist and an end-scale applied along the path."""
        p, pn = _flatten_xy(profile_xy)
        q, qn = _flatten_xyz(path_xyz)
        return self._shape(
            _cffi.lib().cc_twisted_sweep(
                p, pn, q, qn, float(twist_radians), float(scale_end)
            ),
            "twisted_sweep",
        )

    def loft_along_rail(
        self,
        rail_xyz: Iterable[Sequence[float]],
        profile_a_xy: Iterable[Sequence[float]],
        profile_b_xy: Iterable[Sequence[float]],
    ) -> Shape:
        """Loft profile A to profile B along a guiding rail."""
        r, rn = _flatten_xyz(rail_xyz)
        a, an = _flatten_xy(profile_a_xy)
        b, bn = _flatten_xy(profile_b_xy)
        return self._shape(
            _cffi.lib().cc_loft_along_rail(r, rn, a, an, b, bn), "loft_along_rail"
        )

    def guided_sweep(
        self,
        profile_xy: Iterable[Sequence[float]],
        path_xyz: Iterable[Sequence[float]],
        guide_xyz: Iterable[Sequence[float]],
    ) -> Shape:
        """Sweep a profile along a path, constrained by a guide curve."""
        p, pn = _flatten_xy(profile_xy)
        q, qn = _flatten_xyz(path_xyz)
        g, gn = _flatten_xyz(guide_xyz)
        return self._shape(
            _cffi.lib().cc_guided_sweep(p, pn, q, qn, g, gn), "guided_sweep"
        )

    def guided_orient_sweep(
        self,
        profile_xy: Iterable[Sequence[float]],
        path_xyz: Iterable[Sequence[float]],
        guide_xyz: Iterable[Sequence[float]],
    ) -> Shape:
        """Sweep whose section orientation is steered by a guide wire (the guide
        fixes the section frame, not its size — distinct from
        :meth:`guided_sweep`)."""
        p, pn = _flatten_xy(profile_xy)
        q, qn = _flatten_xyz(path_xyz)
        g, gn = _flatten_xyz(guide_xyz)
        return self._shape(
            _cffi.lib().cc_guided_orient_sweep(p, pn, q, qn, g, gn),
            "guided_orient_sweep",
        )

    def variable_sweep(
        self,
        profile_a_xy: Iterable[Sequence[float]],
        profile_b_xy: Iterable[Sequence[float]],
        spine_xyz: Iterable[Sequence[float]],
        guide_xyz: Iterable[Sequence[float]] | None = None,
    ) -> Shape:
        """Variable-section sweep: morph profile A → profile B (same vertex
        count) along ``spine_xyz``, optionally scaled by a ``guide_xyz`` rail.

        A circle→circle radius-varying morph along a straight spine is a cone.
        """
        a, an = _flatten_xy(profile_a_xy)
        b, bn = _flatten_xy(profile_b_xy)
        s, sn = _flatten_xyz(spine_xyz)
        if guide_xyz is None:
            g, gn = None, 0
        else:
            g, gn = _flatten_xyz(guide_xyz)
        return self._shape(
            _cffi.lib().cc_variable_sweep(a, an, b, bn, s, sn, g, gn),
            "variable_sweep",
        )

    def loft_circles(
        self,
        center1: Sequence[float],
        normal1: Sequence[float],
        radius1: float,
        center2: Sequence[float],
        normal2: Sequence[float],
        radius2: float,
    ) -> Shape:
        """Loft between two true circles → a smooth conical / cylindrical B-rep."""
        c1 = _c_double_array([center1[0], center1[1], center1[2]])
        n1 = _c_double_array([normal1[0], normal1[1], normal1[2]])
        c2 = _c_double_array([center2[0], center2[1], center2[2]])
        n2 = _c_double_array([normal2[0], normal2[1], normal2[2]])
        return self._shape(
            _cffi.lib().cc_loft_circles(
                c1, n1, float(radius1), c2, n2, float(radius2)
            ),
            "loft_circles",
        )

    def loft_circle_wire(
        self,
        center: Sequence[float],
        normal: Sequence[float],
        radius: float,
        wire_xyz: Iterable[Sequence[float]],
    ) -> Shape:
        """Loft a true-circle section to an arbitrary polygon wire."""
        c = _c_double_array([center[0], center[1], center[2]])
        n = _c_double_array([normal[0], normal[1], normal[2]])
        w, wn = _flatten_xyz(wire_xyz)
        return self._shape(
            _cffi.lib().cc_loft_circle_wire(c, n, float(radius), w, wn),
            "loft_circle_wire",
        )

    def loft_along_rails(
        self,
        rail_xyz: Iterable[Sequence[float]],
        guide_xyz: Iterable[Sequence[float]],
        profile_a_xy: Iterable[Sequence[float]],
        profile_b_xy: Iterable[Sequence[float]],
    ) -> Shape:
        """Two-rail loft: a spine plus a guide steering the section shape."""
        r, rn = _flatten_xyz(rail_xyz)
        g, gn = _flatten_xyz(guide_xyz)
        a, an = _flatten_xy(profile_a_xy)
        b, bn = _flatten_xy(profile_b_xy)
        return self._shape(
            _cffi.lib().cc_loft_along_rails(r, rn, g, gn, a, an, b, bn),
            "loft_along_rails",
        )

    def loft_sections(
        self, sections_xyz: Sequence[Iterable[Sequence[float]]]
    ) -> Shape:
        """Ruled loft through 2..N ordered planar section wires (each a list of
        ``(x, y, z)`` points, ≥3 each)."""
        flat: list[float] = []
        counts: list[int] = []
        for sec in sections_xyz:
            c = 0
            for pt in sec:
                flat.extend((float(pt[0]), float(pt[1]), float(pt[2])))
                c += 1
            counts.append(c)
        arr = _c_double_array(flat)
        cnt = _c_int_array(counts)
        return self._shape(
            _cffi.lib().cc_solid_loft_sections(arr, cnt, len(counts)),
            "loft_sections",
        )

    def loft_typed(
        self,
        segs_a: Sequence["CCProfileSeg"],
        frame_a: Sequence[float],
        segs_b: Sequence["CCProfileSeg"],
        frame_b: Sequence[float],
        spline_a: Sequence[float] = (),
        spline_b: Sequence[float] = (),
    ) -> Shape:
        """General loft between two TYPED section profiles, each on its own plane
        frame (``origin(3) + u(3) + v(3)`` = 9 doubles). ``segs_*`` are
        :class:`CCProfileSeg` loops (kind 0 line / 1 arc / 2 circle / 3 spline);
        ``spline_*`` are the flat ``x, y`` spline side-channels."""
        seg_a = (CCProfileSeg * len(segs_a))(*segs_a)
        seg_b = (CCProfileSeg * len(segs_b))(*segs_b)
        sa = _c_double_array(list(spline_a))
        sb = _c_double_array(list(spline_b))
        fa = _c_double_array([float(x) for x in frame_a])
        fb = _c_double_array([float(x) for x in frame_b])
        return self._shape(
            _cffi.lib().cc_loft_typed(
                seg_a, len(segs_a), sa, len(spline_a), fa,
                seg_b, len(segs_b), sb, len(spline_b), fb,
            ),
            "loft_typed",
        )

    def fill_ngon(
        self,
        boundary_xyz: Sequence[Sequence[float]],
        edge_kinds: Sequence[int] | None = None,
        arc_mids: Sequence[Sequence[float]] | None = None,
        grid_n: int = 8,
    ) -> Shape:
        """Fill an N-sided (3..6) analytic boundary loop with a smooth patch,
        returned as a mesh-backed body. Native engine only.

        ``boundary_xyz`` are the N corner points in order; ``edge_kinds[i]`` is
        0 (straight) or 1 (arc); ``arc_mids`` gives one mid-arc point per arc
        side in arc order.
        """
        flat: list[float] = []
        n = 0
        for c in boundary_xyz:
            flat.extend((float(c[0]), float(c[1]), float(c[2])))
            n += 1
        b = _c_double_array(flat)
        kinds = _c_int_array(list(edge_kinds)) if edge_kinds else None
        if arc_mids:
            mids_flat: list[float] = []
            for m in arc_mids:
                mids_flat.extend((float(m[0]), float(m[1]), float(m[2])))
            mids = _c_double_array(mids_flat)
        else:
            mids = None
        return self._shape(
            _cffi.lib().cc_fill_ngon(b, n, kinds, mids, int(grid_n)),
            "fill_ngon",
        )

    def sheet_base_flange(
        self, profile_xy: Iterable[Sequence[float]], thickness: float
    ) -> Shape:
        """Base sheet-metal flange: a closed 2-D polygon extruded by
        ``thickness``. Native engine only (``kernel.engine = "native"``)."""
        arr, n = _flatten_xy(profile_xy)
        return self._shape(
            _cffi.lib().cc_sheet_base_flange(arr, n, float(thickness)),
            "sheet_base_flange",
        )

    def extrude_with_holes(
        self,
        outer_xy: Iterable[Sequence[float]],
        holes_center_radius: Iterable[Sequence[float]],
        depth: float,
    ) -> Shape:
        """Extrude an outer profile with circular holes ``(cx, cy, r)``."""
        o, on = _flatten_xy(outer_xy)
        flat: list[float] = []
        hcount = 0
        for h in holes_center_radius:
            flat.extend((float(h[0]), float(h[1]), float(h[2])))
            hcount += 1
        holes = _c_double_array(flat)
        return self._shape(
            _cffi.lib().cc_solid_extrude_holes(o, on, holes, hcount, float(depth)),
            "extrude_with_holes",
        )

    def extrude_with_poly_holes(
        self,
        outer_xy: Iterable[Sequence[float]],
        holes_xy: Sequence[Iterable[Sequence[float]]],
        depth: float,
    ) -> Shape:
        """Extrude an outer profile with polygonal holes.

        ``holes_xy`` is a sequence of hole loops, each an iterable of ``(x, y)``.
        """
        o, on = _flatten_xy(outer_xy)
        flat: list[float] = []
        counts: list[int] = []
        for loop in holes_xy:
            c = 0
            for pt in loop:
                flat.extend((float(pt[0]), float(pt[1])))
                c += 1
            counts.append(c)
        holes = _c_double_array(flat)
        hole_counts = _c_int_array(counts)
        return self._shape(
            _cffi.lib().cc_solid_extrude_polyholes(
                o, on, holes, hole_counts, len(counts), float(depth)
            ),
            "extrude_with_poly_holes",
        )

    # ── Threads / shanks ──────────────────────────────────────────────────────
    def helical_thread(
        self,
        major_radius_mm: float,
        pitch_mm: float,
        turns: float,
        depth_mm: float,
        flank_angle_deg: float,
        points_per_mm: float,
        samples_per_turn: int,
    ) -> Shape:
        """Build a helical thread solid."""
        return self._shape(
            _cffi.lib().cc_helical_thread(
                float(major_radius_mm), float(pitch_mm), float(turns),
                float(depth_mm), float(flank_angle_deg),
                float(points_per_mm), int(samples_per_turn),
            ),
            "helical_thread",
        )

    def tapered_thread(
        self,
        top_radius_mm: float,
        tip_radius_mm: float,
        pitch_mm: float,
        turns: float,
        depth_mm: float,
        flank_angle_deg: float,
        points_per_mm: float,
        samples_per_turn: int,
    ) -> Shape:
        """Build a tapered (e.g. NPT-style) helical thread solid."""
        return self._shape(
            _cffi.lib().cc_tapered_thread(
                float(top_radius_mm), float(tip_radius_mm), float(pitch_mm),
                float(turns), float(depth_mm), float(flank_angle_deg),
                float(points_per_mm), int(samples_per_turn),
            ),
            "tapered_thread",
        )

    def tapered_shank(
        self,
        radius_mm: float,
        full_height_mm: float,
        taper_height_mm: float,
        points_per_mm: float,
    ) -> Shape:
        """Build a cylindrical shank with a tapered tip."""
        return self._shape(
            _cffi.lib().cc_tapered_shank(
                float(radius_mm), float(full_height_mm),
                float(taper_height_mm), float(points_per_mm),
            ),
            "tapered_shank",
        )

    # ── Data exchange (import) ────────────────────────────────────────────────
    def step_import(self, path: str) -> Shape:
        """Import a body from a STEP file."""
        return self._shape(
            _cffi.lib().cc_step_import(str(path).encode("utf-8")), "step_import"
        )

    def iges_import(self, path: str) -> Shape:
        """Import a body from an IGES file."""
        return self._shape(
            _cffi.lib().cc_iges_import(str(path).encode("utf-8")), "iges_import"
        )

    def stl_import(self, path: str) -> Shape:
        """Import an STL file (ASCII or binary, auto-detected) as a mesh body."""
        return self._shape(
            _cffi.lib().cc_stl_import(str(path).encode("utf-8")), "stl_import"
        )

    def step_pmi_scan(self, path: str) -> PmiSummary:
        """Read-only AP242 PMI / GD&T annotation census of a STEP file.

        Native engine only (``kernel.engine = "native"``). Does not import
        geometry. Raises on failure.
        """
        out = CCPmiSummary()
        ok = _cffi.lib().cc_step_pmi_scan(str(path).encode("utf-8"), ctypes.byref(out))
        if not ok:
            raise _fail("step_pmi_scan")
        return PmiSummary(
            dimensions=int(out.dimensions),
            tolerances=int(out.tolerances),
            datums=int(out.datums),
            datum_targets=int(out.datum_targets),
            notes=int(out.notes),
            annotation_geometry=int(out.annotation_geometry),
            unknown=int(out.unknown),
            total=int(out.total),
        )

    # ── Tetrahedral volume meshing (requires a CYBERCAD_HAS_TETGEN build) ─────
    def tet_mesh_surface(
        self,
        vertices: np.ndarray,
        triangles: np.ndarray,
        order: int = 10,
        target_element_size: float = 0.0,
        grading: float = 1.4,
        min_scaled_jacobian: float = 0.02,
    ) -> TetMesh:
        """Tet-mesh a raw closed triangle surface (no B-rep needed).

        ``vertices`` is ``(N, 3)`` float; ``triangles`` is ``(M, 3)`` 0-based
        int. Requires a ``CYBERCAD_HAS_TETGEN`` build; raises otherwise.
        """
        v = np.ascontiguousarray(vertices, dtype=np.float64).reshape(-1)
        t = np.ascontiguousarray(triangles, dtype=np.int32).reshape(-1)
        vp = v.ctypes.data_as(ctypes.POINTER(ctypes.c_double))
        tp = t.ctypes.data_as(ctypes.POINTER(ctypes.c_int))
        opts = CCVolumeMeshOptions(
            order=int(order),
            target_element_size=float(target_element_size),
            grading=float(grading),
            min_scaled_jacobian=float(min_scaled_jacobian),
        )
        tm: CCTetMesh = _cffi.lib().cc_tet_mesh_surface(
            vp, v.size // 3, tp, t.size // 3, opts
        )
        return _tet_mesh_from_cc(tm, "tet_mesh_surface")

    def mesh_quality(
        self, tet_mesh: TetMesh, min_scaled_jacobian: float = 0.02
    ) -> QualityReport:
        """Native tet-mesh quality census (always available, TetGen-independent).

        Raises on empty / degenerate input.
        """
        cc = _cc_tetmesh_from_tetmesh(tet_mesh)
        qr: CCQualityReport = _cffi.lib().cc_mesh_quality(
            cc, float(min_scaled_jacobian)
        )
        report = _quality_from_cc(qr)
        if not report.valid:
            raise _fail("mesh_quality")
        return report

    # ── Reference geometry (pure point math, no shape handle) ─────────────────
    def ref_plane_from_points(
        self,
        p0: Sequence[float],
        p1: Sequence[float],
        p2: Sequence[float],
    ) -> ReferencePlane:
        """Datum plane through three points (origin ``p0``, right-hand normal)."""
        a = _c_double_array([p0[0], p0[1], p0[2]])
        b = _c_double_array([p1[0], p1[1], p1[2]])
        c = _c_double_array([p2[0], p2[1], p2[2]])
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_ref_plane_from_points(a, b, c, out6)
        if not ok:
            raise _fail("ref_plane_from_points")
        return ReferencePlane(
            origin=(float(out6[0]), float(out6[1]), float(out6[2])),
            normal=(float(out6[3]), float(out6[4]), float(out6[5])),
        )

    def ref_plane_offset(
        self,
        origin: Sequence[float],
        normal: Sequence[float],
        distance: float,
    ) -> ReferencePlane:
        """Datum plane offset ``distance`` from ``origin`` along ``normal``."""
        o = _c_double_array([origin[0], origin[1], origin[2]])
        n = _c_double_array([normal[0], normal[1], normal[2]])
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_ref_plane_offset(o, n, float(distance), out6)
        if not ok:
            raise _fail("ref_plane_offset")
        return ReferencePlane(
            origin=(float(out6[0]), float(out6[1]), float(out6[2])),
            normal=(float(out6[3]), float(out6[4]), float(out6[5])),
        )

    def ref_axis_from_points(
        self, a: Sequence[float], b: Sequence[float]
    ) -> ReferenceAxis:
        """Datum axis from ``a`` toward ``b`` (origin ``a``, unit direction)."""
        pa = _c_double_array([a[0], a[1], a[2]])
        pb = _c_double_array([b[0], b[1], b[2]])
        out6 = (ctypes.c_double * 6)()
        ok = _cffi.lib().cc_ref_axis_from_points(pa, pb, out6)
        if not ok:
            raise _fail("ref_axis_from_points")
        return ReferenceAxis(
            origin=(float(out6[0]), float(out6[1]), float(out6[2])),
            direction=(float(out6[3]), float(out6[4]), float(out6[5])),
        )
