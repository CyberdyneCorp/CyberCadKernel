"""Pythonic object model over the exact-NURBS ``cc_nurbs_*`` C facade.

This module is the ergonomic layer over ``cc_kernel_nurbs.h`` (the additive NURBS
geometry surface: opaque ``cc_curve`` / ``cc_surface`` handles, POD accessors,
evaluators, the display-tessellation bridge, and the Wave D–I feature wrappers).
It sits on the faithful 1:1 ctypes mirror in :mod:`cybercadkernel._cffi` and turns
the plain-C surface into idiomatic Python:

* :class:`Curve` / :class:`Surface` — RAII wrappers owning a ``cc_curve`` /
  ``cc_surface`` handle, released exactly once via :meth:`close`, ``__exit__`` or
  ``__del__`` (a GC backstop), with a stale-handle guard reusing the
  :class:`~cybercadkernel.Shape` pattern. They expose NumPy knot / homogeneous-pole
  accessors, ``.eval(...)`` on the exact rational geometry, and (for surfaces)
  ``.tessellate()`` → the shared :class:`~cybercadkernel.Mesh` interop.
* the module-level functions (:func:`circle`, :func:`revolve`, :func:`fit_curve`,
  :func:`recognize_curve`, …) surface every ``cc_nurbs_*`` wrapper, following the
  ``api.py`` RAII / :class:`~cybercadkernel.KernelError`-from-``cc_last_error``
  pattern. **An honest ABI decline (a ``0`` handle / a ``< 0`` count) RAISES**
  :class:`~cybercadkernel.KernelError`; it never returns a degenerate object.

DISPLAY-TESSELLATION CAVEAT (design.md §4): :meth:`Surface.tessellate` produces a
SINGLE-SURFACE display mesh for visualization. It is NOT the watertight multi-face
curved-seam weld; a multi-face assembly must be shown as a set of such meshes, not
a sewn solid. A closed single surface (a revolved sphere) does tessellate to a
watertight mesh.
"""

from __future__ import annotations

import ctypes
from dataclasses import dataclass
from typing import Sequence

import numpy as np

from . import _cffi
from ._cffi import (
    CCCurveEndConstraint,
    CCCurveHit,
    CCCurveInfo,
    CCCurveRecognition,
    CCCurveSurfaceHit,
    CCMesh,
    CCPrimitiveDetection,
    CCSurfaceInfo,
    CCSurfacePoleConstraint,
    CCSurfaceRecognition,
    CCTessOptions,
    CCTrimLoop,
    cc_curve,
    cc_surface,
)
from .api import CyberCadError, Mesh, _c_double_array, _last_error, _mesh_from_ccmesh

__all__ = [
    "Curve",
    "Surface",
    "PrimitiveType",
    "CurveKind",
    "SurfaceKind",
    "NSidedMode",
    "JoinMode",
    "SurfaceEdge",
    "TrimBoolOp",
    "PrimitiveDetection",
    "CurveRecognition",
    "SurfaceRecognition",
    "CurveHit",
    "CurveSurfaceHit",
    "TrimLoop",
    # constructors / wrappers
    "circle",
    "arc",
    "ellipse",
    "plane",
    "cylinder",
    "cone",
    "sphere",
    "torus",
    "fit_curve",
    "interp_curve",
    "fit_surface",
    "estimate_weights_curve",
    "estimate_weights_surface",
    "fit_curve_constrained",
    "fit_surface_constrained",
    "fair_curve",
    "fair_surface",
    "simplify_curve",
    "detect_primitive",
    "recognize_curve",
    "recognize_surface",
    "skin",
    "gordon",
    "coons",
    "nsided_fill",
    "sweep_variable",
    "sweep_two_rail",
    "revolve",
    "join",
    "fillet_freeform_g2",
    "vertex_blend",
    "chamfer_variable",
    "chamfer_freeform",
    "offset_rational",
    "offset_trimmed",
    "thicken_trimmed",
    "shell_trimmed",
    "BoolOp",
    "solid_boolean",
    "intersect_cc",
    "intersect_cs",
    "trim_region_boolean",
]


# ── Enums (mirror the header's small enums) ────────────────────────────────────


class PrimitiveType:
    """``CCPrimitiveType`` — detected analytic primitive (``detect_primitive``)."""

    FREEFORM = 0
    PLANE = 1
    SPHERE = 2
    CYLINDER = 3
    CONE = 4


class CurveKind:
    """``CCCurveKind`` — recognized analytic curve (``recognize_curve``)."""

    GENERAL = 0
    LINE = 1
    CIRCLE = 2
    ARC = 3
    ELLIPSE = 4


class SurfaceKind:
    """``CCSurfaceKind`` — recognized analytic surface (``recognize_surface``)."""

    GENERAL = 0
    PLANE = 1
    CYLINDER = 2
    CONE = 3
    SPHERE = 4


class NSidedMode:
    """``CCNSidedMode`` — continuity mode for :func:`nsided_fill`."""

    C0 = 0
    G1 = 1
    G2 = 2
    RATIONAL = 3


class JoinMode:
    """``CCJoinMode`` — continuity mode for :func:`join`."""

    G1 = 1
    G2 = 2


class SurfaceEdge:
    """``CCSurfaceEdge`` — which boundary of a surface is the shared edge."""

    U0 = 0
    U1 = 1
    V0 = 2
    V1 = 3


class TrimBoolOp:
    """``CCTrimBoolOp`` — the 2-D region boolean operator."""

    UNION = 0
    INTERSECT = 1
    DIFFERENCE = 2


class BoolOp:
    """``CCBoolOp`` — the general NURBS solid boolean operator (:func:`solid_boolean`)."""

    FUSE = 0
    CUT = 1
    COMMON = 2


# ── Result dataclasses ─────────────────────────────────────────────────────────


@dataclass(frozen=True)
class PrimitiveDetection:
    """Result of :func:`detect_primitive` on a raw point cloud.

    Only the fields the :attr:`type` selects are meaningful; the rest are 0.
    """

    type: int
    rms: float
    rel_error: float
    plane_normal: tuple[float, float, float]
    plane_offset: float
    center: tuple[float, float, float]
    axis: tuple[float, float, float]
    radius: float
    half_angle: float


@dataclass(frozen=True)
class CurveRecognition:
    """Result of :func:`recognize_curve` (only :attr:`kind`-selected fields hold)."""

    kind: int
    residual: float
    line_start: tuple[float, float, float]
    line_end: tuple[float, float, float]
    center: tuple[float, float, float]
    normal: tuple[float, float, float]
    x_axis: tuple[float, float, float]
    radius: float
    minor_radius: float
    start_angle: float
    sweep_angle: float


@dataclass(frozen=True)
class SurfaceRecognition:
    """Result of :func:`recognize_surface` (only :attr:`kind`-selected fields hold)."""

    kind: int
    residual: float
    origin: tuple[float, float, float]
    axis: tuple[float, float, float]
    x_axis: tuple[float, float, float]
    radius: float
    half_angle: float


@dataclass(frozen=True)
class CurveHit:
    """One curve<->curve intersection point (:func:`intersect_cc`)."""

    xyz: tuple[float, float, float]
    t_a: float
    t_b: float
    tangential: bool


@dataclass(frozen=True)
class CurveSurfaceHit:
    """One curve<->surface pierce point (:func:`intersect_cs`)."""

    xyz: tuple[float, float, float]
    t: float
    u: float
    v: float
    tangential: bool


@dataclass(frozen=True)
class TrimLoop:
    """One result loop of a trim-region boolean, as a closed UV polyline."""

    uv: np.ndarray  # (P, 2) float64
    outer: bool
    signed_area: float


# ── ctypes helpers ─────────────────────────────────────────────────────────────


def _vec3(v: Sequence[float]) -> ctypes.Array:
    return _c_double_array([float(v[0]), float(v[1]), float(v[2])])


def _tup3(arr) -> tuple[float, float, float]:
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _flatten_xyz_flat(points) -> tuple[ctypes.Array, int]:
    """Flatten (N,3)/flat XYZ points into a ``c_double[]`` and return (buf, N)."""
    a = np.ascontiguousarray(np.asarray(points, dtype=np.float64).reshape(-1))
    if a.size % 3 != 0:
        raise ValueError("point sequence length must be a multiple of 3 (x, y, z)")
    return _c_double_array(a.tolist()), a.size // 3


# ── Curve ──────────────────────────────────────────────────────────────────────


class Curve:
    """A NURBS curve: an owned :c:type:`cc_curve` handle.

    Obtain one from a module-level constructor (:func:`circle`, :func:`fit_curve`,
    …) or :meth:`create`; do not construct from a raw id directly. The handle is
    released exactly once — via :meth:`close`, on ``__exit__`` of a ``with`` block,
    or on garbage collection. Every operation raises :class:`KernelError` on an
    honest decline, never returning a degenerate handle.
    """

    __slots__ = ("_id", "_closed", "__weakref__")

    def __init__(self, curve_id: int) -> None:
        # Internal: `curve_id` must already be a valid (non-zero) id.
        self._id = int(curve_id)
        self._closed = False

    # -- construction --------------------------------------------------------
    @classmethod
    def create(
        cls,
        degree: int,
        poles_xyzw: Sequence[float],
        knots: Sequence[float],
    ) -> "Curve":
        """Build a NURBS curve from raw data.

        ``poles_xyzw`` is ``n_ctrl`` HOMOGENEOUS control points packed
        ``(x, y, z, w)`` (``4*n_ctrl`` doubles, every ``w > 0``); ``knots`` is the
        FLAT knot vector (length ``n_ctrl + degree + 1``, non-decreasing). Raises
        on invalid input.
        """
        p = np.ascontiguousarray(np.asarray(poles_xyzw, dtype=np.float64).reshape(-1))
        k = np.ascontiguousarray(np.asarray(knots, dtype=np.float64).reshape(-1))
        if p.size % 4 != 0:
            raise ValueError("poles_xyzw length must be a multiple of 4 (x, y, z, w)")
        n_ctrl = p.size // 4
        h: cc_curve = _cffi.lib().cc_curve_create(
            int(degree),
            _c_double_array(p.tolist()),
            n_ctrl,
            _c_double_array(k.tolist()),
            int(k.size),
        )
        return _wrap_curve(h, "cc_curve_create")

    # -- lifetime ------------------------------------------------------------
    @property
    def id(self) -> int:
        """The raw :c:type:`cc_curve` id. Raises if the curve is closed."""
        if self._closed:
            raise CyberCadError("operation on a released Curve")
        return self._id

    @property
    def closed(self) -> bool:
        """``True`` once the underlying handle has been released."""
        return self._closed

    def _handle(self) -> cc_curve:
        return cc_curve(self.id)

    def close(self) -> None:
        """Release the underlying handle (idempotent)."""
        if not self._closed and self._id:
            try:
                _cffi.lib().cc_curve_release(cc_curve(self._id))
            finally:
                self._closed = True

    def __enter__(self) -> "Curve":
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
        return f"<Curve {state}>"

    # -- accessors -----------------------------------------------------------
    def info(self) -> CCCurveInfo:
        """The raw curve header (degree, n_ctrl, n_knots, rational)."""
        out = CCCurveInfo()
        if not _cffi.lib().cc_curve_info(self._handle(), ctypes.byref(out)):
            raise CyberCadError("cc_curve_info failed: " + (_last_error() or ""))
        return out

    @property
    def degree(self) -> int:
        return int(self.info().degree)

    @property
    def rational(self) -> bool:
        return bool(self.info().rational)

    def knots(self) -> np.ndarray:
        """The FLAT knot vector as an ``(n_knots,)`` float64 array."""
        info = self.info()
        n = int(info.n_knots)
        buf = (ctypes.c_double * n)()
        written = _cffi.lib().cc_curve_knots(self._handle(), buf, n)
        if written < 0:
            raise CyberCadError("cc_curve_knots failed: " + (_last_error() or ""))
        return np.ctypeslib.as_array(buf, shape=(written,)).copy()

    def poles(self) -> np.ndarray:
        """The HOMOGENEOUS poles as an ``(n_ctrl, 4)`` float64 array ``(x,y,z,w)``.

        Non-rational curves report every ``w = 1``.
        """
        info = self.info()
        n = 4 * int(info.n_ctrl)
        buf = (ctypes.c_double * n)()
        written = _cffi.lib().cc_curve_poles(self._handle(), buf, n)
        if written < 0:
            raise CyberCadError("cc_curve_poles failed: " + (_last_error() or ""))
        return np.ctypeslib.as_array(buf, shape=(written,)).copy().reshape(-1, 4)

    # -- evaluation ----------------------------------------------------------
    def eval(self, t: float) -> np.ndarray:
        """Point ``(x, y, z)`` on the exact rational curve at parameter ``t``
        (clamped to the knot domain)."""
        out = (ctypes.c_double * 3)()
        if not _cffi.lib().cc_curve_eval(self._handle(), float(t), out):
            raise CyberCadError("cc_curve_eval failed: " + (_last_error() or ""))
        return np.array(out, dtype=np.float64)

    def polyline(self, n_samples: int = 64) -> np.ndarray:
        """Sample the curve into a display polyline of ``n_samples`` points
        (clamped to >= 2) as an ``(P, 3)`` float64 array."""
        out = _cffi.CCEdgePolyline()
        ok = _cffi.lib().cc_curve_polyline(self._handle(), int(n_samples), ctypes.byref(out))
        if not ok:
            raise CyberCadError("cc_curve_polyline failed: " + (_last_error() or ""))
        try:
            pc = int(out.pointCount)
            if pc <= 0 or not out.points:
                return np.zeros((0, 3), dtype=np.float64)
            return np.ctypeslib.as_array(out.points, shape=(pc * 3,)).copy().reshape(-1, 3)
        finally:
            if out.points:
                _cffi.lib().cc_points_free(out.points)


# ── Surface ────────────────────────────────────────────────────────────────────


class Surface:
    """A NURBS surface: an owned :c:type:`cc_surface` handle.

    Same RAII contract as :class:`Curve`. Adds row-major-U-outer pole / per-
    direction knot accessors, ``.eval(u, v)`` on the exact rational geometry, and
    :meth:`tessellate` → a single-surface display :class:`Mesh` (see the module
    caveat).
    """

    __slots__ = ("_id", "_closed", "__weakref__")

    def __init__(self, surface_id: int) -> None:
        self._id = int(surface_id)
        self._closed = False

    # -- construction --------------------------------------------------------
    @classmethod
    def create(
        cls,
        degree_u: int,
        degree_v: int,
        poles_xyzw: Sequence[float],
        n_ctrl_u: int,
        n_ctrl_v: int,
        knots_u: Sequence[float],
        knots_v: Sequence[float],
    ) -> "Surface":
        """Build a tensor-product NURBS surface from raw data.

        ``poles_xyzw`` is ``(n_ctrl_u * n_ctrl_v)`` HOMOGENEOUS poles ``(x,y,z,w)``,
        ROW-MAJOR with U outer (``index i*n_ctrl_v + j``); ``knots_u`` / ``knots_v``
        are FLAT, non-decreasing. Raises on invalid input.
        """
        p = np.ascontiguousarray(np.asarray(poles_xyzw, dtype=np.float64).reshape(-1))
        ku = np.ascontiguousarray(np.asarray(knots_u, dtype=np.float64).reshape(-1))
        kv = np.ascontiguousarray(np.asarray(knots_v, dtype=np.float64).reshape(-1))
        h: cc_surface = _cffi.lib().cc_surface_create(
            int(degree_u),
            int(degree_v),
            _c_double_array(p.tolist()),
            int(n_ctrl_u),
            int(n_ctrl_v),
            _c_double_array(ku.tolist()),
            int(ku.size),
            _c_double_array(kv.tolist()),
            int(kv.size),
        )
        return _wrap_surface(h, "cc_surface_create")

    # -- lifetime ------------------------------------------------------------
    @property
    def id(self) -> int:
        if self._closed:
            raise CyberCadError("operation on a released Surface")
        return self._id

    @property
    def closed(self) -> bool:
        return self._closed

    def _handle(self) -> cc_surface:
        return cc_surface(self.id)

    def close(self) -> None:
        if not self._closed and self._id:
            try:
                _cffi.lib().cc_surface_release(cc_surface(self._id))
            finally:
                self._closed = True

    def __enter__(self) -> "Surface":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    def __del__(self) -> None:
        try:
            self.close()
        except Exception:  # pragma: no cover
            pass

    def __repr__(self) -> str:
        state = "closed" if self._closed else f"id={self._id}"
        return f"<Surface {state}>"

    # -- accessors -----------------------------------------------------------
    def info(self) -> CCSurfaceInfo:
        """The raw surface header (degrees, control counts, knot counts, rational)."""
        out = CCSurfaceInfo()
        if not _cffi.lib().cc_surface_info(self._handle(), ctypes.byref(out)):
            raise CyberCadError("cc_surface_info failed: " + (_last_error() or ""))
        return out

    @property
    def rational(self) -> bool:
        return bool(self.info().rational)

    def _knots(self, which: str) -> np.ndarray:
        info = self.info()
        n = int(info.n_knots_u if which == "u" else info.n_knots_v)
        buf = (ctypes.c_double * n)()
        fn = _cffi.lib().cc_surface_knots_u if which == "u" else _cffi.lib().cc_surface_knots_v
        written = fn(self._handle(), buf, n)
        if written < 0:
            raise CyberCadError(f"cc_surface_knots_{which} failed: " + (_last_error() or ""))
        return np.ctypeslib.as_array(buf, shape=(written,)).copy()

    def knots_u(self) -> np.ndarray:
        """The FLAT U knot vector as an ``(n_knots_u,)`` float64 array."""
        return self._knots("u")

    def knots_v(self) -> np.ndarray:
        """The FLAT V knot vector as an ``(n_knots_v,)`` float64 array."""
        return self._knots("v")

    def poles(self) -> np.ndarray:
        """The HOMOGENEOUS poles as an ``(n_ctrl_u, n_ctrl_v, 4)`` float64 array,
        ROW-MAJOR with U outer. Non-rational surfaces report every ``w = 1``."""
        info = self.info()
        nu, nv = int(info.n_ctrl_u), int(info.n_ctrl_v)
        n = 4 * nu * nv
        buf = (ctypes.c_double * n)()
        written = _cffi.lib().cc_surface_poles(self._handle(), buf, n)
        if written < 0:
            raise CyberCadError("cc_surface_poles failed: " + (_last_error() or ""))
        return np.ctypeslib.as_array(buf, shape=(written,)).copy().reshape(nu, nv, 4)

    # -- evaluation ----------------------------------------------------------
    def eval(self, u: float, v: float) -> np.ndarray:
        """Point ``(x, y, z)`` on the exact rational surface at ``(u, v)``
        (clamped to the knot domain)."""
        out = (ctypes.c_double * 3)()
        if not _cffi.lib().cc_surface_eval(self._handle(), float(u), float(v), out):
            raise CyberCadError("cc_surface_eval failed: " + (_last_error() or ""))
        return np.array(out, dtype=np.float64)

    # -- tessellation --------------------------------------------------------
    def tessellate(self, n_u: int = 0, n_v: int = 0) -> Mesh:
        """Single-surface DISPLAY :class:`Mesh` over the ``(u, v)`` domain.

        ``n_u`` / ``n_v`` are the sample counts (each clamped to >= 2; <= 0 uses a
        built-in default). NOT the watertight multi-face weld — see the module
        caveat; a closed single surface (a revolved sphere) is watertight.
        """
        opt = CCTessOptions(n_u=int(n_u), n_v=int(n_v))
        out = CCMesh()
        ok = _cffi.lib().cc_surface_tessellate(
            self._handle(), ctypes.byref(opt), ctypes.byref(out)
        )
        if not ok:
            raise CyberCadError("cc_surface_tessellate failed: " + (_last_error() or ""))
        try:
            return _mesh_from_ccmesh(out)
        finally:
            _cffi.lib().cc_mesh_free(out)


# ── internal: wrap a returned handle, raising on the honest decline ────────────


def _wrap_curve(h: cc_curve, op: str) -> Curve:
    cid = int(h.id) if isinstance(h, cc_curve) else int(h)
    if not cid:
        raise CyberCadError(f"{op} failed: {_last_error()}" if _last_error() else f"{op} failed")
    return Curve(cid)


def _wrap_surface(h: cc_surface, op: str) -> Surface:
    sid = int(h.id) if isinstance(h, cc_surface) else int(h)
    if not sid:
        raise CyberCadError(f"{op} failed: {_last_error()}" if _last_error() else f"{op} failed")
    return Surface(sid)


def _curve_handles(curves: Sequence[Curve]) -> ctypes.Array:
    arr = (cc_curve * len(curves))()
    for i, c in enumerate(curves):
        arr[i] = cc_curve(c.id)
    return arr


def _surface_handles(surfaces: Sequence[Surface]) -> ctypes.Array:
    arr = (cc_surface * len(surfaces))()
    for i, s in enumerate(surfaces):
        arr[i] = cc_surface(s.id)
    return arr


# ── Analytic → exact rational NURBS ────────────────────────────────────────────


def circle(center, normal, x_axis, radius: float) -> Curve:
    """A full circle as a piecewise rational-quadratic B-spline. Raises on a
    non-positive radius or a degenerate frame."""
    return _wrap_curve(
        _cffi.lib().cc_nurbs_circle(_vec3(center), _vec3(normal), _vec3(x_axis), float(radius)),
        "cc_nurbs_circle",
    )


def arc(center, normal, x_axis, radius: float, start_angle: float, sweep_angle: float) -> Curve:
    """A circular arc (``sweep_angle`` in (0, 2π]). Raises on a bad frame / sweep."""
    return _wrap_curve(
        _cffi.lib().cc_nurbs_arc(
            _vec3(center), _vec3(normal), _vec3(x_axis),
            float(radius), float(start_angle), float(sweep_angle),
        ),
        "cc_nurbs_arc",
    )


def ellipse(center, normal, x_axis, major_radius: float, minor_radius: float) -> Curve:
    """A full ellipse. Raises on a non-positive semi-axis or a degenerate frame."""
    return _wrap_curve(
        _cffi.lib().cc_nurbs_ellipse(
            _vec3(center), _vec3(normal), _vec3(x_axis), float(major_radius), float(minor_radius)
        ),
        "cc_nurbs_ellipse",
    )


def plane(origin, normal, x_axis, u0: float, u1: float, v0: float, v1: float) -> Surface:
    """A finite window ``[u0,u1]×[v0,v1]`` of a plane as a bilinear patch."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_plane(
            _vec3(origin), _vec3(normal), _vec3(x_axis),
            float(u0), float(u1), float(v0), float(v1),
        ),
        "cc_nurbs_plane",
    )


def cylinder(origin, axis, x_axis, radius: float, v0: float, v1: float) -> Surface:
    """A finite-height cylinder as an exact rational surface of revolution."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_cylinder(
            _vec3(origin), _vec3(axis), _vec3(x_axis), float(radius), float(v0), float(v1)
        ),
        "cc_nurbs_cylinder",
    )


def cone(origin, axis, x_axis, radius: float, semi_angle: float, v0: float, v1: float) -> Surface:
    """A finite-height cone as an exact rational surface of revolution."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_cone(
            _vec3(origin), _vec3(axis), _vec3(x_axis),
            float(radius), float(semi_angle), float(v0), float(v1),
        ),
        "cc_nurbs_cone",
    )


def sphere(center, axis, x_axis, radius: float) -> Surface:
    """A full sphere as an exact rational surface of revolution."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_sphere(_vec3(center), _vec3(axis), _vec3(x_axis), float(radius)),
        "cc_nurbs_sphere",
    )


def torus(center, axis, x_axis, major_radius: float, minor_radius: float) -> Surface:
    """A full torus as an exact rational surface of revolution. Raises on a
    spindle torus (R < r) / non-positive minor radius / degenerate frame."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_torus(
            _vec3(center), _vec3(axis), _vec3(x_axis), float(major_radius), float(minor_radius)
        ),
        "cc_nurbs_torus",
    )


# ── Fitting / interpolation / weight estimation ────────────────────────────────


def fit_curve(points_xyz, degree: int, n_ctrl: int, param_method: int = 1) -> Curve:
    """Least-squares curve APPROXIMATION with ``n_ctrl`` poles, endpoints pinned.
    Raises on a bad degree / count, too few points, or a degenerate input."""
    buf, n = _flatten_xyz_flat(points_xyz)
    return _wrap_curve(
        _cffi.lib().cc_nurbs_fit_curve(buf, n, int(degree), int(n_ctrl), int(param_method)),
        "cc_nurbs_fit_curve",
    )


def interp_curve(points_xyz, degree: int, param_method: int = 1) -> Curve:
    """Global curve INTERPOLATION through every point. Raises on a bad degree /
    too few points / degenerate input."""
    buf, n = _flatten_xyz_flat(points_xyz)
    return _wrap_curve(
        _cffi.lib().cc_nurbs_interp_curve(buf, n, int(degree), int(param_method)),
        "cc_nurbs_interp_curve",
    )


def fit_surface(
    grid_xyz, n_u: int, n_v: int, degree_u: int, degree_v: int,
    n_ctrl_u: int, n_ctrl_v: int, param_method: int = 1,
) -> Surface:
    """Least-squares tensor-product surface APPROXIMATION over an ``n_u×n_v`` grid
    (row-major, U outer). Raises on bad dimensions / a degenerate grid."""
    buf, _ = _flatten_xyz_flat(grid_xyz)
    return _wrap_surface(
        _cffi.lib().cc_nurbs_fit_surface(
            buf, int(n_u), int(n_v), int(degree_u), int(degree_v),
            int(n_ctrl_u), int(n_ctrl_v), int(param_method),
        ),
        "cc_nurbs_fit_surface",
    )


def estimate_weights_curve(points_xyz, degree: int, n_ctrl: int, param_method: int = 1) -> Curve:
    """Ma–Kruth: recover BOTH poles AND weights of a rational curve fitting the
    data. Raises on an under-determined / rank-deficient / sign-mixed system."""
    buf, n = _flatten_xyz_flat(points_xyz)
    return _wrap_curve(
        _cffi.lib().cc_nurbs_estimate_weights_curve(
            buf, n, int(degree), int(n_ctrl), int(param_method)
        ),
        "cc_nurbs_estimate_weights_curve",
    )


def estimate_weights_surface(
    grid_xyz, n_u: int, n_v: int, degree_u: int, degree_v: int,
    n_ctrl_u: int, n_ctrl_v: int, param_method: int = 1,
) -> Surface:
    """Surface analogue of :func:`estimate_weights_curve`."""
    buf, _ = _flatten_xyz_flat(grid_xyz)
    return _wrap_surface(
        _cffi.lib().cc_nurbs_estimate_weights_surface(
            buf, int(n_u), int(n_v), int(degree_u), int(degree_v),
            int(n_ctrl_u), int(n_ctrl_v), int(param_method),
        ),
        "cc_nurbs_estimate_weights_surface",
    )


def fit_curve_constrained(
    points_xyz,
    constraints: Sequence[CCCurveEndConstraint],
    degree: int,
    n_ctrl: int,
    param_method: int = 1,
) -> Curve:
    """Equality-CONSTRAINED least-squares curve fit. ``constraints`` is a sequence
    of :class:`CCCurveEndConstraint`. Raises on an over-constrained / singular set."""
    buf, n = _flatten_xyz_flat(points_xyz)
    arr = (CCCurveEndConstraint * len(constraints))(*constraints)
    return _wrap_curve(
        _cffi.lib().cc_nurbs_fit_curve_constrained(
            buf, n, arr, len(constraints), int(degree), int(n_ctrl), int(param_method)
        ),
        "cc_nurbs_fit_curve_constrained",
    )


def fit_surface_constrained(
    grid_xyz, n_u: int, n_v: int,
    constraints: Sequence[CCSurfacePoleConstraint],
    degree_u: int, degree_v: int, n_ctrl_u: int, n_ctrl_v: int,
    param_method: int = 1,
) -> Surface:
    """Equality-CONSTRAINED least-squares surface fit. ``constraints`` is a
    sequence of :class:`CCSurfacePoleConstraint`."""
    buf, _ = _flatten_xyz_flat(grid_xyz)
    arr = (CCSurfacePoleConstraint * len(constraints))(*constraints)
    return _wrap_surface(
        _cffi.lib().cc_nurbs_fit_surface_constrained(
            buf, int(n_u), int(n_v), arr, len(constraints),
            int(degree_u), int(degree_v), int(n_ctrl_u), int(n_ctrl_v), int(param_method),
        ),
        "cc_nurbs_fit_surface_constrained",
    )


# ── Fairing / simplification ───────────────────────────────────────────────────


def fair_curve(curve: Curve, tol: float, keep_ends: bool = True) -> Curve:
    """Minimal-energy fairing of a curve within ``tol`` of the input. Raises when
    ``tol`` is too tight / the input is rational / malformed."""
    return _wrap_curve(
        _cffi.lib().cc_nurbs_fair_curve(cc_curve(curve.id), float(tol), 1 if keep_ends else 0),
        "cc_nurbs_fair_curve",
    )


def fair_surface(surface: Surface, tol: float, keep_boundary: bool = True) -> Surface:
    """Minimal-energy thin-plate fairing of a surface within ``tol`` of the input."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_fair_surface(
            cc_surface(surface.id), float(tol), 1 if keep_boundary else 0
        ),
        "cc_nurbs_fair_surface",
    )


def simplify_curve(curve: Curve, tol: float) -> Curve:
    """Bounded knot removal within ``tol``. Raises only on an unknown handle /
    non-finite tol."""
    return _wrap_curve(
        _cffi.lib().cc_nurbs_simplify_curve(cc_curve(curve.id), float(tol)),
        "cc_nurbs_simplify_curve",
    )


# ── Reverse-engineering ────────────────────────────────────────────────────────


def detect_primitive(points_xyz, rel_tol: float = 0.0) -> PrimitiveDetection:
    """Try plane / sphere / cylinder / cone against a point cloud and report the
    best qualifying fit (``type == FREEFORM`` if none). Raises on too few points."""
    buf, n = _flatten_xyz_flat(points_xyz)
    out = CCPrimitiveDetection()
    if not _cffi.lib().cc_nurbs_detect_primitive(buf, n, float(rel_tol), ctypes.byref(out)):
        raise CyberCadError("cc_nurbs_detect_primitive failed: " + (_last_error() or ""))
    return PrimitiveDetection(
        type=int(out.type),
        rms=float(out.rms),
        rel_error=float(out.rel_error),
        plane_normal=_tup3(out.plane_normal),
        plane_offset=float(out.plane_offset),
        center=_tup3(out.center),
        axis=_tup3(out.axis),
        radius=float(out.radius),
        half_angle=float(out.half_angle),
    )


def recognize_curve(curve: Curve, tol: float = 0.0) -> CurveRecognition:
    """Recognize whether ``curve`` is EXACTLY a line / circle / arc / ellipse."""
    out = CCCurveRecognition()
    if not _cffi.lib().cc_nurbs_recognize_curve(cc_curve(curve.id), float(tol), ctypes.byref(out)):
        raise CyberCadError("cc_nurbs_recognize_curve failed: " + (_last_error() or ""))
    return CurveRecognition(
        kind=int(out.kind),
        residual=float(out.residual),
        line_start=_tup3(out.line_start),
        line_end=_tup3(out.line_end),
        center=_tup3(out.center),
        normal=_tup3(out.normal),
        x_axis=_tup3(out.x_axis),
        radius=float(out.radius),
        minor_radius=float(out.minor_radius),
        start_angle=float(out.start_angle),
        sweep_angle=float(out.sweep_angle),
    )


def recognize_surface(surface: Surface, tol: float = 0.0) -> SurfaceRecognition:
    """Recognize whether ``surface`` is EXACTLY a plane / cylinder / cone / sphere."""
    out = CCSurfaceRecognition()
    if not _cffi.lib().cc_nurbs_recognize_surface(
        cc_surface(surface.id), float(tol), ctypes.byref(out)
    ):
        raise CyberCadError("cc_nurbs_recognize_surface failed: " + (_last_error() or ""))
    return SurfaceRecognition(
        kind=int(out.kind),
        residual=float(out.residual),
        origin=_tup3(out.origin),
        axis=_tup3(out.axis),
        x_axis=_tup3(out.x_axis),
        radius=float(out.radius),
        half_angle=float(out.half_angle),
    )


# ── Surfacing ──────────────────────────────────────────────────────────────────


def skin(sections: Sequence[Curve], degree_v: int) -> Surface:
    """SKIN / LOFT one surface containing every section curve as an iso-curve."""
    if len(sections) < 2:
        raise ValueError("skin needs >= 2 sections")
    return _wrap_surface(
        _cffi.lib().cc_nurbs_skin(_curve_handles(sections), len(sections), int(degree_v)),
        "cc_nurbs_skin",
    )


def gordon(
    u_curves: Sequence[Curve], v_curves: Sequence[Curve],
    v_params: Sequence[float], u_params: Sequence[float],
) -> Surface:
    """GORDON / NETWORK boolean-sum surface interpolating a u-curve × v-curve grid."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_gordon(
            _curve_handles(u_curves), len(u_curves),
            _curve_handles(v_curves), len(v_curves),
            _c_double_array([float(x) for x in v_params]),
            _c_double_array([float(x) for x in u_params]),
        ),
        "cc_nurbs_gordon",
    )


def coons(c0: Curve, c1: Curve, d0: Curve, d1: Curve, tol: float = 0.0) -> Surface:
    """COONS PATCH filling four boundary curves (c0=v0, c1=v1, d0=u0, d1=u1)."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_coons(
            cc_curve(c0.id), cc_curve(c1.id), cc_curve(d0.id), cc_curve(d1.id), float(tol)
        ),
        "cc_nurbs_coons",
    )


def nsided_fill(
    boundary: Sequence[Curve], mode: int = NSidedMode.G2, tol: float = 0.0, cap: int = 64
) -> list[Surface]:
    """N-SIDED FILL of a closed loop of >= 3 boundary curves → 1..N patches.

    Returns the produced patches. Raises on a non-closed loop, a mode-forbidden
    rational boundary, or a G1/G2-infeasible creased corner (never a residual crease).
    """
    n = len(boundary)
    if n < 3:
        raise ValueError("nsided_fill needs >= 3 boundary curves")
    out = (cc_surface * int(cap))()
    count = _cffi.lib().cc_nurbs_nsided_fill(
        _curve_handles(boundary), n, int(mode), float(tol), out, int(cap)
    )
    if count < 0:
        raise CyberCadError("cc_nurbs_nsided_fill failed: " + (_last_error() or ""))
    return [Surface(int(out[i].id)) for i in range(count)]


def sweep_variable(
    profile: Curve, path: Curve, section_normal,
    scales: Sequence[float], twists: Sequence[float], stations: int, degree_v: int,
) -> Surface:
    """VARIABLE-SECTION SWEEP: per-station scale + twist on an RMF-transported
    section, then skin. ``scales`` / ``twists`` are each ``stations`` long."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_sweep_variable(
            cc_curve(profile.id), cc_curve(path.id), _vec3(section_normal),
            _c_double_array([float(x) for x in scales]),
            _c_double_array([float(x) for x in twists]),
            int(stations), int(degree_v),
        ),
        "cc_nurbs_sweep_variable",
    )


def sweep_two_rail(
    profile: Curve, rail0: Curve, rail1: Curve, section_normal,
    anchor0: int, anchor1: int, stations: int, degree_v: int,
) -> Surface:
    """TWO-RAIL SWEEP: section pole indices ``anchor0`` / ``anchor1`` ride the two
    rails at every station."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_sweep_two_rail(
            cc_curve(profile.id), cc_curve(rail0.id), cc_curve(rail1.id),
            _vec3(section_normal), int(anchor0), int(anchor1), int(stations), int(degree_v),
        ),
        "cc_nurbs_sweep_two_rail",
    )


def revolve(profile: Curve, axis_point, axis_dir, angle: float) -> Surface:
    """REVOLVE: exact rational surface of revolution of ``profile`` about the axis
    through ``angle`` radians. A semicircle → exact sphere. Raises on a null / non-
    unit axis, ~zero angle, or a profile lying on the axis."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_revolve(cc_curve(profile.id), _vec3(axis_point), _vec3(axis_dir), float(angle)),
        "cc_nurbs_revolve",
    )


def join(
    a: Surface, b: Surface, edge_a: int, edge_b: int, reversed_: bool,
    mode: int = JoinMode.G1, max_movement_cap: float = 0.0,
) -> tuple[Surface, Surface, float]:
    """JOIN: reposition the near-boundary rows of two adjacent surfaces so they
    meet with continuity ``mode`` across the shared edge. Returns
    ``(new_a, new_b, residual)``. Raises on a non-coincident edge / mismatch / a
    required movement beyond ``max_movement_cap``."""
    out_a = cc_surface()
    out_b = cc_surface()
    residual = ctypes.c_double()
    ok = _cffi.lib().cc_nurbs_join(
        cc_surface(a.id), cc_surface(b.id), int(edge_a), int(edge_b),
        1 if reversed_ else 0, int(mode), float(max_movement_cap),
        ctypes.byref(residual), ctypes.byref(out_a), ctypes.byref(out_b),
    )
    if not ok:
        raise CyberCadError("cc_nurbs_join failed: " + (_last_error() or ""))
    return Surface(int(out_a.id)), Surface(int(out_b.id)), float(residual.value)


# ── Blend + offset / thicken / shell ───────────────────────────────────────────


def fillet_freeform_g2(
    face_a: Surface, face_b: Surface, radius: float, center0, spine_dir,
    side_a: float, side_b: float, step_len: float, n_stations: int, n_section_samples: int,
) -> Surface:
    """FREEFORM G2 rolling-ball fillet between two freeform faces. Raises when the
    ball won't fit / over-radius / section fold / too few stations (never a self-
    intersecting fillet)."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_fillet_freeform_g2(
            cc_surface(face_a.id), cc_surface(face_b.id), float(radius),
            _vec3(center0), _vec3(spine_dir), float(side_a), float(side_b),
            float(step_len), int(n_stations), int(n_section_samples),
        ),
        "cc_nurbs_fillet_freeform_g2",
    )


def vertex_blend(
    fillets: Sequence[Surface], sides: Sequence[int], setbacks: Sequence[float],
    reverses: Sequence[int], mode: int = JoinMode.G1, cap: int = 64,
) -> list[Surface]:
    """SETBACK VERTEX BLEND: fill the N-sided corner where N (>=3) incident fillets
    meet. Returns the produced corner sub-patches. Raises on N<3 / a malformed
    fillet / a non-closed gap loop / a G1/G2-infeasible corner."""
    n = len(fillets)
    if n < 3:
        raise ValueError("vertex_blend needs >= 3 fillets")
    out = (cc_surface * int(cap))()
    count = _cffi.lib().cc_nurbs_vertex_blend(
        _surface_handles(fillets), n,
        (ctypes.c_int * n)(*[int(s) for s in sides]),
        _c_double_array([float(x) for x in setbacks]),
        (ctypes.c_int * n)(*[int(r) for r in reverses]),
        int(mode), out, int(cap),
    )
    if count < 0:
        raise CyberCadError("cc_nurbs_vertex_blend failed: " + (_last_error() or ""))
    return [Surface(int(out[i].id)) for i in range(count)]


def chamfer_variable(sub_a, sub_b, edge, n_stations: int, d0: float, d1: float) -> Surface:
    """VARIABLE-DISTANCE analytic chamfer along an edge shared by two analytic
    faces (see the header for the ``sub_a`` / ``sub_b`` / ``edge`` packing)."""
    a = _c_double_array([float(x) for x in np.asarray(sub_a, dtype=np.float64).reshape(-1)])
    b = _c_double_array([float(x) for x in np.asarray(sub_b, dtype=np.float64).reshape(-1)])
    e = _c_double_array([float(x) for x in np.asarray(edge, dtype=np.float64).reshape(-1)])
    return _wrap_surface(
        _cffi.lib().cc_nurbs_chamfer_variable(a, b, e, int(n_stations), float(d0), float(d1)),
        "cc_nurbs_chamfer_variable",
    )


def chamfer_freeform(face_a: Surface, face_b: Surface, edge, n_stations: int, d: float) -> Surface:
    """FREEFORM-EDGE chamfer along a crease shared by two freeform faces."""
    e = _c_double_array([float(x) for x in np.asarray(edge, dtype=np.float64).reshape(-1)])
    return _wrap_surface(
        _cffi.lib().cc_nurbs_chamfer_freeform(
            cc_surface(face_a.id), cc_surface(face_b.id), e, int(n_stations), float(d)
        ),
        "cc_nurbs_chamfer_freeform",
    )


def offset_rational(surface: Surface, dist: float, tol: float = 0.0) -> Surface:
    """RATIONAL OFFSET at signed distance ``dist`` (exact conic → exact offset
    conic). Raises on a degenerate normal / self-intersection / fit failure / no
    numsci substrate."""
    return _wrap_surface(
        _cffi.lib().cc_nurbs_offset_rational(cc_surface(surface.id), float(dist), float(tol)),
        "cc_nurbs_offset_rational",
    )


def offset_trimmed(surface: Surface, dist: float, tol: float = 0.0) -> tuple[Surface, tuple, bool]:
    """FOLD-TRIMMED OFFSET at ``dist``: trims to the maximal fold-free rectangle.
    Returns ``(surface, kept_rect (u0,u1,v0,v1), trimmed)``. Raises when no fold-
    free region of meaningful area remains."""
    kept = (ctypes.c_double * 4)()
    trimmed = ctypes.c_int()
    h = _cffi.lib().cc_nurbs_offset_trimmed(
        cc_surface(surface.id), float(dist), float(tol), kept, ctypes.byref(trimmed)
    )
    s = _wrap_surface(h, "cc_nurbs_offset_trimmed")
    return s, (float(kept[0]), float(kept[1]), float(kept[2]), float(kept[3])), bool(trimmed.value)


def thicken_trimmed(surface: Surface, dist: float, tol: float = 0.0) -> tuple[Mesh, tuple, bool]:
    """SELF-INTERSECTION-TRIMMED THICKEN into a CLOSED watertight solid mesh.
    Returns ``(mesh, kept_rect, trimmed)``. Raises on a fully folding / degenerate /
    zero-thickness / non-closed input or an absent numsci substrate."""
    kept = (ctypes.c_double * 4)()
    trimmed = ctypes.c_int()
    out = CCMesh()
    ok = _cffi.lib().cc_nurbs_thicken_trimmed(
        cc_surface(surface.id), float(dist), float(tol), ctypes.byref(out), kept, ctypes.byref(trimmed)
    )
    if not ok:
        raise CyberCadError("cc_nurbs_thicken_trimmed failed: " + (_last_error() or ""))
    try:
        mesh = _mesh_from_ccmesh(out)
    finally:
        _cffi.lib().cc_mesh_free(out)
    return mesh, (float(kept[0]), float(kept[1]), float(kept[2]), float(kept[3])), bool(trimmed.value)


def shell_trimmed(
    faces: Sequence[Surface], adjacency: Sequence[int], dist: float, tol: float = 0.0
) -> Mesh:
    """SLAB-OVERLAP-TRIMMED MULTI-FACE SHELL into one closed watertight solid mesh.
    ``adjacency`` is ``n_edges`` records packed 5 ints each (faceA, faceB, edgeA,
    edgeB, reversed). Raises on a fold / adjacency mismatch / non-manifold seam /
    un-trimmable overlap / absent numsci substrate."""
    adj = np.asarray(adjacency, dtype=np.int32).reshape(-1)
    if adj.size % 5 != 0:
        raise ValueError("adjacency must be a multiple of 5 ints (faceA,faceB,edgeA,edgeB,reversed)")
    out = CCMesh()
    ok = _cffi.lib().cc_nurbs_shell_trimmed(
        _surface_handles(faces), len(faces),
        (ctypes.c_int * adj.size)(*adj.tolist()), adj.size // 5,
        float(dist), float(tol), ctypes.byref(out),
    )
    if not ok:
        raise CyberCadError("cc_nurbs_shell_trimmed failed: " + (_last_error() or ""))
    try:
        return _mesh_from_ccmesh(out)
    finally:
        _cffi.lib().cc_mesh_free(out)


# ── General NURBS solid boolean ────────────────────────────────────────────────


def solid_boolean(
    face_a: Surface,
    rim_a: float,
    lid_a: float,
    face_b: Surface,
    rim_b: float,
    lid_b: float,
    op: int = BoolOp.COMMON,
    deflection: float = 0.0,
) -> Mesh:
    """GENERAL NURBS SOLID BOOLEAN (fuse / cut / common) of two freeform bowl-cup solids.

    Each operand is built from a freeform WALL surface (``face_a`` / ``face_b`` — each
    a single-patch Bézier: clamped knots, no interior knots) trimmed by a rim CIRCLE of
    radius ``rim_a`` / ``rim_b`` in the wall's ``(u, v)`` domain (centred at the domain
    midpoint), closed by a flat LID plane at world-z ``lid_a`` / ``lid_b`` sharing that
    rim. ``op`` selects :attr:`BoolOp.FUSE` / :attr:`BoolOp.CUT` / :attr:`BoolOp.COMMON`;
    ``deflection`` (``<= 0`` → native default) is the result-mesh tessellation deflection.

    Returns the WATERTIGHT result :class:`Mesh` (a closed, positive-volume shell whose
    volume matches the native/OCCT op-volume within the tessellation band). **Raises**
    :class:`KernelError` on an honest decline — an unknown / non-single-patch-Bézier wall,
    a non-admissible operand, or the MULTI-SEAM / non-watertight annulus↔annulus pose the
    orchestrator declines. It NEVER returns a leaky / partial mesh.
    """
    out = CCMesh()
    ok = _cffi.lib().cc_nurbs_solid_boolean(
        cc_surface(face_a.id), float(rim_a), float(lid_a),
        cc_surface(face_b.id), float(rim_b), float(lid_b),
        int(op), float(deflection), ctypes.byref(out),
    )
    if not ok:
        raise CyberCadError("cc_nurbs_solid_boolean failed: " + (_last_error() or ""))
    try:
        return _mesh_from_ccmesh(out)
    finally:
        _cffi.lib().cc_mesh_free(out)


# ── Intersection + trim boolean ────────────────────────────────────────────────


def intersect_cc(a: Curve, b: Curve, tol: float = 0.0) -> list[CurveHit]:
    """Intersect two NURBS curves → isolated hit points. Raises on an unknown
    handle or COINCIDENT / OVERLAPPING curves (an infinite set, never faked). A
    disjoint pair returns an empty list."""
    out_ptr = ctypes.POINTER(CCCurveHit)()
    count = _cffi.lib().cc_nurbs_intersect_cc(
        cc_curve(a.id), cc_curve(b.id), float(tol), ctypes.byref(out_ptr)
    )
    if count < 0:
        raise CyberCadError("cc_nurbs_intersect_cc failed: " + (_last_error() or ""))
    try:
        hits = []
        for i in range(count):
            h = out_ptr[i]
            hits.append(CurveHit(_tup3(h.xyz), float(h.tA), float(h.tB), bool(h.tangential)))
        return hits
    finally:
        if out_ptr:
            _cffi.lib().cc_nurbs_hits_cc_free(out_ptr)


def intersect_cs(c: Curve, s: Surface, tol: float = 0.0) -> list[CurveSurfaceHit]:
    """Intersect a NURBS curve with a NURBS surface → pierce points. Raises when
    the curve lies ON the surface over a sub-arc. A disjoint pair returns []."""
    out_ptr = ctypes.POINTER(CCCurveSurfaceHit)()
    count = _cffi.lib().cc_nurbs_intersect_cs(
        cc_curve(c.id), cc_surface(s.id), float(tol), ctypes.byref(out_ptr)
    )
    if count < 0:
        raise CyberCadError("cc_nurbs_intersect_cs failed: " + (_last_error() or ""))
    try:
        hits = []
        for i in range(count):
            h = out_ptr[i]
            hits.append(
                CurveSurfaceHit(_tup3(h.xyz), float(h.t), float(h.u), float(h.v), bool(h.tangential))
            )
        return hits
    finally:
        if out_ptr:
            _cffi.lib().cc_nurbs_hits_cs_free(out_ptr)


def trim_region_boolean(
    region_a: Sequence[Curve], region_b: Sequence[Curve], op: int = TrimBoolOp.UNION
) -> tuple[list[TrimLoop], float]:
    """Parameter-space trim-region boolean (∪/∩/∖) on two regions (each a list of
    loop curves in the SAME surface (u,v) domain; loop 0 = outer, 1.. = holes).
    Returns ``(loops, total_signed_area)``. A count of 0 with area 0 is a valid
    EMPTY region. Raises on an unknown handle / malformed loop / coincident-
    boundary overlap."""
    out_ptr = ctypes.POINTER(CCTrimLoop)()
    area = ctypes.c_double()
    count = _cffi.lib().cc_nurbs_trim_region_boolean(
        _curve_handles(region_a), len(region_a),
        _curve_handles(region_b), len(region_b), int(op),
        ctypes.byref(out_ptr), ctypes.byref(area),
    )
    if count < 0:
        raise CyberCadError("cc_nurbs_trim_region_boolean failed: " + (_last_error() or ""))
    try:
        loops = []
        for i in range(count):
            lp = out_ptr[i]
            pc = int(lp.pointCount)
            if pc > 0 and lp.uv:
                uv = np.ctypeslib.as_array(lp.uv, shape=(pc * 2,)).copy().reshape(-1, 2)
            else:
                uv = np.zeros((0, 2), dtype=np.float64)
            loops.append(TrimLoop(uv=uv, outer=bool(lp.outer), signed_area=float(lp.signedArea)))
        return loops, float(area.value)
    finally:
        if out_ptr and count > 0:
            _cffi.lib().cc_nurbs_trim_loops_free(out_ptr, count)
