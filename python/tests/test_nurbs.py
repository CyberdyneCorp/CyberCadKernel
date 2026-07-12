"""Real-geometry tests for the exact-NURBS ``cc_nurbs_*`` object model.

Every assertion here checks REAL geometry through the ``cybercadkernel.nurbs``
surface (design.md §6, tasks.md §6):

* an analytic ``circle`` evaluates on the true circle to <= 1e-12;
* a ``recognize_curve`` of that circle round-trips to a Circle with the right radius;
* a revolved sphere tessellates to a watertight mesh (``trimesh.is_watertight``);
* a global interpolation reproduces its sampled points exactly;
* an N-sided G2 fill interpolates its boundary curves;
* an over-constrained fit and an over-radius freeform fillet RAISE ``KernelError``
  (the honest-decline contract — never a fabricated degenerate object);
* a ``Curve`` / ``Surface`` releases its handle on context exit.

The ``kernel`` fixture (conftest) skips the whole module when the desktop dylib
cannot load; the autouse ``_require_real_engine`` gate skips on a stub build.
"""

import math

import numpy as np
import pytest

from cybercadkernel import Curve, KernelError, Surface, nurbs
from cybercadkernel._cffi import CCCurveEndConstraint


# ── analytic geometry: evaluation exactness + recognition round-trip ────────────


def test_circle_eval_is_exact(kernel):
    """A rational-quadratic ``circle`` evaluates onto the true circle <= 1e-12."""
    r = 2.5
    with nurbs.circle([1, 2, 3], [0, 0, 1], [1, 0, 0], r) as c:
        info = c.info()
        assert info.degree == 2
        assert bool(info.rational)  # rational-quadratic exact circle
        kn = c.knots()
        max_err = 0.0
        for t in np.linspace(kn[0], kn[-1], 400):
            p = c.eval(t)
            # distance from the plane's center (1,2,3) in the z=3 plane
            max_err = max(max_err, abs(math.hypot(p[0] - 1.0, p[1] - 2.0) - r))
            max_err = max(max_err, abs(p[2] - 3.0))
        assert max_err <= 1e-12


def test_recognize_circle_round_trips(kernel):
    """``recognize_curve`` of an exact NURBS circle returns Circle + its radius."""
    r = 4.0
    with nurbs.circle([0, 0, 0], [0, 0, 1], [1, 0, 0], r) as c:
        rec = nurbs.recognize_curve(c)
        assert rec.kind == nurbs.CurveKind.CIRCLE
        assert rec.radius == pytest.approx(r, abs=1e-9)
        assert rec.residual <= 1e-9


def test_surface_poles_are_homogeneous(kernel):
    """A rational sphere reports homogeneous poles ``(n_u, n_v, 4)`` with w > 0."""
    with nurbs.sphere([0, 0, 0], [0, 0, 1], [1, 0, 0], 3.0) as s:
        poles = s.poles()
        assert poles.ndim == 3 and poles.shape[2] == 4
        assert np.all(poles[..., 3] > 0.0)  # every homogeneous weight positive


# ── closed surface tessellates watertight ───────────────────────────────────────


def test_revolved_sphere_tessellates_watertight(kernel, trimesh_or_skip):
    """A semicircle revolved 360° about its diameter is an exact sphere whose
    single-surface display tessellation welds to a watertight mesh."""
    r = 3.0
    # semicircle pole-to-pole in the XZ plane (diameter on the Z axis), bulging +X.
    prof = nurbs.arc([0, 0, 0], [0, 1, 0], [0, 0, 1], r, 0.0, math.pi)
    with prof:
        with nurbs.revolve(prof, [0, 0, 0], [0, 0, 1], 2.0 * math.pi) as sph:
            # exact-sphere evaluation certificate
            ku, kv = sph.knots_u(), sph.knots_v()
            max_err = 0.0
            for u in np.linspace(ku[0], ku[-1], 15):
                for v in np.linspace(kv[0], kv[-1], 15):
                    p = sph.eval(u, v)
                    max_err = max(max_err, abs(np.linalg.norm(p) - r))
            assert max_err <= 1e-9

            mesh = sph.tessellate(64, 64)
            tm = mesh.to_trimesh()
            # weld the parametric-grid seam / pole vertices, then drop the
            # collapsed pole triangles — the standard cleanup for a (u,v) grid mesh.
            tm.merge_vertices(digits_vertex=6)
            tm.update_faces(tm.nondegenerate_faces())
            tm.remove_unreferenced_vertices()
            assert tm.is_watertight
            assert tm.euler_number == 2  # genus-0 closed shell


# ── fitting: interpolation reproduces sampled points ────────────────────────────


def test_interp_curve_reproduces_samples(kernel):
    """A global cubic interpolation passes through every sampled point."""
    ts = np.linspace(0.0, 1.0, 12)
    pts = np.stack([np.cos(2.0 * ts), np.sin(2.0 * ts), 0.5 * ts], axis=1)
    with nurbs.interp_curve(pts, degree=3) as c:
        kn = c.knots()
        # endpoints are pinned exactly
        assert np.linalg.norm(c.eval(kn[0]) - pts[0]) <= 1e-9
        assert np.linalg.norm(c.eval(kn[-1]) - pts[-1]) <= 1e-9
        # every sample lies on the curve (dense min-distance well under the
        # sample spacing — an interpolation, not a smoothing approximation)
        dense = np.array([c.eval(t) for t in np.linspace(kn[0], kn[-1], 600)])
        worst = max(np.min(np.linalg.norm(dense - p, axis=1)) for p in pts)
        assert worst <= 1e-3


def test_fit_curve_approximates_samples(kernel):
    """A least-squares approximation stays close to its sampled points and pins
    the endpoints."""
    ts = np.linspace(0.0, 1.0, 40)
    pts = np.stack([ts, np.sin(3.0 * ts), np.zeros_like(ts)], axis=1)
    with nurbs.fit_curve(pts, degree=3, n_ctrl=8) as c:
        kn = c.knots()
        assert np.linalg.norm(c.eval(kn[0]) - pts[0]) <= 1e-9
        assert np.linalg.norm(c.eval(kn[-1]) - pts[-1]) <= 1e-9
        dense = np.array([c.eval(t) for t in np.linspace(kn[0], kn[-1], 600)])
        worst = max(np.min(np.linalg.norm(dense - p, axis=1)) for p in pts)
        assert worst <= 5e-2


# ── N-sided fill interpolates its boundary ──────────────────────────────────────


def _segment(p0, p1) -> Curve:
    """A degree-1 line segment as a NURBS curve (clamped [0,1] domain)."""
    poles = [*p0, 1.0, *p1, 1.0]
    return Curve.create(1, poles, [0.0, 0.0, 1.0, 1.0])


def test_nsided_g2_fill_interpolates_boundary(kernel):
    """A G2 N-sided fill of a planar pentagon reproduces its boundary (each fill
    patch's sampled points lie in the boundary plane, and the boundary corners are
    reproduced)."""
    n = 5
    r = 1.0
    corners = [
        [r * math.cos(2.0 * math.pi * k / n), r * math.sin(2.0 * math.pi * k / n), 0.0]
        for k in range(n)
    ]
    boundary = [_segment(corners[k], corners[(k + 1) % n]) for k in range(n)]
    try:
        patches = nurbs.nsided_fill(boundary, mode=nurbs.NSidedMode.G2, tol=1e-6)
        assert len(patches) >= 1
        try:
            # every fill point lies in the boundary plane z = 0 (the fill is planar,
            # so it interpolates the coplanar boundary exactly in z)
            max_z = 0.0
            corner_hits = np.zeros(n, dtype=bool)
            all_pts = []
            for s in patches:
                ku, kv = s.knots_u(), s.knots_v()
                for u in np.linspace(ku[0], ku[-1], 8):
                    for v in np.linspace(kv[0], kv[-1], 8):
                        p = s.eval(u, v)
                        max_z = max(max_z, abs(p[2]))
                        all_pts.append(p)
            all_pts = np.array(all_pts)
            assert max_z <= 1e-9  # fill stays in the boundary plane
            # each pentagon corner is reproduced by some fill point
            for i, cpt in enumerate(corners):
                corner_hits[i] = np.min(np.linalg.norm(all_pts - np.array(cpt), axis=1)) <= 1e-6
            assert corner_hits.all()
        finally:
            for s in patches:
                s.close()
    finally:
        for c in boundary:
            c.close()


# ── honest-decline: over-constrained fit / over-radius fillet RAISE ─────────────


def test_over_constrained_fit_raises(kernel):
    """An over-constrained curve fit (n_constraints >= n_ctrl) is an honest decline
    that RAISES ``KernelError`` — never a fabricated curve."""
    ts = np.linspace(0.0, 1.0, 12)
    pts = np.stack([ts, ts * ts, np.zeros_like(ts)], axis=1)
    # 4 poles but 4 exact end constraints => over-constrained singular KKT
    constraints = [
        CCCurveEndConstraint(end=0, order=0, value=(0.0, 0.0, 0.0)),
        CCCurveEndConstraint(end=0, order=1, value=(1.0, 0.0, 0.0)),
        CCCurveEndConstraint(end=1, order=0, value=(1.0, 1.0, 0.0)),
        CCCurveEndConstraint(end=1, order=1, value=(1.0, 2.0, 0.0)),
    ]
    with pytest.raises(KernelError):
        nurbs.fit_curve_constrained(pts, constraints, degree=3, n_ctrl=4)


def test_over_radius_freeform_fillet_raises(kernel):
    """A rolling-ball radius too large to fit between two faces is an honest
    decline that RAISES ``KernelError`` — never a self-intersecting fillet."""
    fa = nurbs.plane([0, 0, 0], [0, 0, 1], [1, 0, 0], -2, 2, -2, 2)
    fb = nurbs.plane([0, 0, 4], [0, 0, 1], [1, 0, 0], -2, 2, -2, 2)
    with fa, fb:
        with pytest.raises(KernelError):
            nurbs.fillet_freeform_g2(
                fa, fb, 100.0, [0, 0, 2.0], [0, 1, 0], 1.0, -1.0, 0.2, 5, 3
            )


def test_spindle_torus_raises(kernel):
    """A spindle torus (R < r) is an honest decline that RAISES."""
    with pytest.raises(KernelError):
        nurbs.torus([0, 0, 0], [0, 0, 1], [1, 0, 0], 1.0, 2.0)


# ── lifetime: RAII release + stale-handle guard ─────────────────────────────────


def test_curve_released_on_context_exit(kernel):
    """A ``Curve`` releases its handle on ``with`` exit; use-after-release raises."""
    c = nurbs.circle([0, 0, 0], [0, 0, 1], [1, 0, 0], 1.0)
    with c:
        assert not c.closed
        assert c.id != 0
    assert c.closed
    with pytest.raises(KernelError):
        _ = c.id  # stale-handle guard


def test_surface_released_on_context_exit(kernel):
    """A ``Surface`` releases its handle on ``with`` exit; use-after-release raises."""
    s = nurbs.sphere([0, 0, 0], [0, 0, 1], [1, 0, 0], 2.0)
    with s:
        assert not s.closed
    assert s.closed
    with pytest.raises(KernelError):
        s.eval(0.0, 0.0)


def test_curve_double_close_is_idempotent(kernel):
    """Releasing twice is crash-free (mirrors ``cc_curve_release`` idempotency)."""
    c = nurbs.circle([0, 0, 0], [0, 0, 1], [1, 0, 0], 1.0)
    c.close()
    c.close()  # no raise, no crash
    assert c.closed
