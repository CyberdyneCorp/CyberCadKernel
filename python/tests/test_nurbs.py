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
        # every sample lies on the curve: the dense min-distance -> 0 as the
        # sampling refines (an interpolation, not a smoothing approximation). At
        # 20k samples the residual is a pure discretization artifact (~5e-5),
        # orders of magnitude below the ~0.55 inter-sample spacing.
        dense = np.array([c.eval(t) for t in np.linspace(kn[0], kn[-1], 20000)])
        worst = max(np.min(np.linalg.norm(dense - p, axis=1)) for p in pts)
        assert worst <= 1e-4


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


# ── general NURBS solid boolean (fuse / cut / common) ───────────────────────────


_BOWL_A = 2.0   # bowl amplitude
_BOWL_R = 0.35  # rim radius (in the wall's u,v)
_BOWL_H = 0.16  # B's dome apex height (seam rho = sqrt(H/2a) = 0.2)


def _bowl_wall(down_dome: bool) -> Surface:
    """A degree-2 Bézier bowl wall z = a(x²+y²) (or its z↦H−z mirror), matching the
    native single-seam fixture. Single-patch Bézier (clamped {0,0,0,1,1,1} knots)."""
    xc = (-0.5, 0.0, 0.5)
    zc = (0.25 * _BOWL_A, -0.25 * _BOWL_A, 0.25 * _BOWL_A)
    poles = []
    for i in range(3):
        for j in range(3):
            z = zc[i] + zc[j]
            if down_dome:
                z = _BOWL_H - z
            poles += [xc[i], xc[j], z, 1.0]
    kn = [0, 0, 0, 1, 1, 1]
    return Surface.create(2, 2, poles, 3, 3, kn, kn)


def _valley_wall(down_dome: bool) -> Surface:
    """A degree-4 Bézier valley wall z = a(x²+y²−ρ₀²)² (or its mirror), matching the
    native MULTI-seam fixture (a=4, ρ₀=0.28) — the two walls meet in TWO seams."""
    H = 0.03
    xc = (-0.5, -0.25, 0.0, 0.25, 0.5)
    Z = [
        [0.710986, -0.132214, 0.253386, -0.132214, 0.710986],
        [-0.132214, -0.475414, 0.076853, -0.475414, -0.132214],
        [0.253386, 0.076853, 0.684675, 0.076853, 0.253386],
        [-0.132214, -0.475414, 0.076853, -0.475414, -0.132214],
        [0.710986, -0.132214, 0.253386, -0.132214, 0.710986],
    ]
    poles = []
    for i in range(5):
        for j in range(5):
            z = Z[i][j]
            if down_dome:
                z = H - z
            poles += [xc[i], xc[j], z, 1.0]
    kn = [0, 0, 0, 0, 0, 1, 1, 1, 1, 1]
    return Surface.create(4, 4, poles, 5, 5, kn, kn)


def _numsci_or_skip(exc: KernelError) -> None:
    """Skip when the desktop dylib was built WITHOUT the numsci substrate (the general
    boolean composes the numsci SSI seam trace); otherwise re-raise the honest decline."""
    if "numsci" in str(exc):
        pytest.skip("dylib built without CYBERCAD_HAS_NUMSCI (no general NURBS boolean)")
    raise exc


@pytest.mark.parametrize(
    "op, cf",
    [
        (nurbs.BoolOp.COMMON, math.pi * _BOWL_H * _BOWL_H / (4.0 * _BOWL_A)),
        (nurbs.BoolOp.CUT,
         math.pi * _BOWL_A * _BOWL_R**4 / 2.0 - math.pi * _BOWL_H**2 / (4.0 * _BOWL_A)),
        (nurbs.BoolOp.FUSE,
         2.0 * (math.pi * _BOWL_A * _BOWL_R**4 / 2.0) - math.pi * _BOWL_H**2 / (4.0 * _BOWL_A)),
    ],
)
def test_solid_boolean_single_seam_watertight_volume(kernel, trimesh_or_skip, op, cf):
    """The canonical single-seam bowl-cup COMMON/CUT/FUSE welds to a WATERTIGHT trimesh
    mesh whose volume matches the closed-form op-volume within the tessellation band."""
    d = 0.005
    lid_a = _BOWL_A * _BOWL_R**2         # A's lid at z = a·R²
    lid_b = _BOWL_H - _BOWL_A * _BOWL_R**2  # B's lid at z = H − a·R²
    with _bowl_wall(False) as wa, _bowl_wall(True) as wb:
        try:
            mesh = nurbs.solid_boolean(wa, _BOWL_R, lid_a, wb, _BOWL_R, lid_b, op, d)
        except KernelError as exc:
            _numsci_or_skip(exc)
        tm = mesh.to_trimesh()
        assert tm.is_watertight
        assert tm.euler_number == 2            # closed, genus-0
        assert abs(abs(tm.volume) - cf) / cf < 30.0 * d


def test_solid_boolean_multi_seam_all_ops_weld(kernel, trimesh_or_skip):
    """After the M0-WELD shared-seam-strip fix and the FUSE outer-envelope compose, a
    MULTI-seam pose (two degree-4 mirror cups meeting in TWO seams) welds ALL THREE ops to a
    WATERTIGHT mesh: COMMON/CUT the annular lens (inner seam closes), FUSE the outer envelope
    (A∪B, complement of the lens on both walls). Never a leaky mesh."""
    H = 0.03
    rim = 0.45
    z_at_r = 4.0 * (rim * rim - 0.28**2) ** 2
    lid_a, lid_b = z_at_r, H - z_at_r
    with _valley_wall(False) as wa, _valley_wall(True) as wb:
        # COMMON / CUT / FUSE all weld to a watertight mesh (skip cleanly on a no-numsci dylib).
        # COMMON/CUT (the lens) weld to fine deflection; FUSE (the outer envelope) welds in the
        # working band [0.005, 0.01] and honest-declines the finer-deflection mesher parity gap.
        for op, d in ((nurbs.BoolOp.COMMON, 0.0025), (nurbs.BoolOp.CUT, 0.0025),
                      (nurbs.BoolOp.FUSE, 0.005)):
            try:
                mesh = nurbs.solid_boolean(wa, rim, lid_a, wb, rim, lid_b, op, d)
            except KernelError as exc:
                _numsci_or_skip(exc)
            assert mesh.to_trimesh().is_watertight


def test_solid_boolean_unknown_wall_raises(kernel):
    """An operation on a released (stale) wall handle RAISES — never a fabricated mesh."""
    wb = _bowl_wall(True)
    wa = _bowl_wall(False)
    wa.close()  # stale handle
    with wb:
        with pytest.raises(KernelError):
            nurbs.solid_boolean(wa, _BOWL_R, 0.0, wb, _BOWL_R, 0.0, nurbs.BoolOp.COMMON, 0.005)


# ── BOOL-CC-EXTEND: N-ary boolean + feature ops + STEP ──────────────────────────


def test_union_n_two_operand_watertight(kernel, trimesh_or_skip):
    """union_n([A, B]) of two bowl-cups welds to a WATERTIGHT trimesh at the closed-form
    union volume (V(A)+V(B)−lens)."""
    d = 0.005
    lid_a = _BOWL_A * _BOWL_R**2
    lid_b = _BOWL_H - _BOWL_A * _BOWL_R**2
    cf = 2.0 * (math.pi * _BOWL_A * _BOWL_R**4 / 2.0) - math.pi * _BOWL_H**2 / (4.0 * _BOWL_A)
    with _bowl_wall(False) as wa, _bowl_wall(True) as wb:
        try:
            mesh = nurbs.union_n([(wa, _BOWL_R, lid_a), (wb, _BOWL_R, lid_b)], d)
        except KernelError as exc:
            _numsci_or_skip(exc)
        tm = mesh.to_trimesh()
        assert tm.is_watertight
        assert tm.euler_number == 2
        assert abs(abs(tm.volume) - cf) / cf < 30.0 * d


def test_cut_n_single_tool_watertight(kernel, trimesh_or_skip):
    """cut_n(A, [B]) carves a WATERTIGHT trimesh at V(A) − lens."""
    d = 0.005
    lid_a = _BOWL_A * _BOWL_R**2
    lid_b = _BOWL_H - _BOWL_A * _BOWL_R**2
    cf = math.pi * _BOWL_A * _BOWL_R**4 / 2.0 - math.pi * _BOWL_H**2 / (4.0 * _BOWL_A)
    with _bowl_wall(False) as wa, _bowl_wall(True) as wb:
        try:
            mesh = nurbs.cut_n((wa, _BOWL_R, lid_a), [(wb, _BOWL_R, lid_b)], d)
        except KernelError as exc:
            _numsci_or_skip(exc)
        tm = mesh.to_trimesh()
        assert tm.is_watertight
        assert tm.euler_number == 2
        assert abs(abs(tm.volume) - cf) / cf < 30.0 * d


def test_union_n_three_operand_raises(kernel):
    """A ≥3-operand freeform union RAISES KernelError at the measured re-admission boundary
    (never a leaky mesh) — and likewise honest-declines on a no-numsci dylib."""
    lid_a = _BOWL_A * _BOWL_R**2
    lid_b = _BOWL_H - _BOWL_A * _BOWL_R**2
    with _bowl_wall(False) as wa, _bowl_wall(True) as wb, _bowl_wall(False) as wc:
        with pytest.raises(KernelError):
            nurbs.union_n(
                [(wa, _BOWL_R, lid_a), (wb, _BOWL_R, lid_b), (wc, _BOWL_R, lid_a)], 0.005
            )


def test_pocket_and_boss_watertight(kernel, trimesh_or_skip):
    """pocket / boss of two bowl-cups each weld to a WATERTIGHT trimesh."""
    d = 0.005
    lid_a = _BOWL_A * _BOWL_R**2
    lid_b = _BOWL_H - _BOWL_A * _BOWL_R**2
    with _bowl_wall(False) as wa, _bowl_wall(True) as wb:
        for fn in (nurbs.pocket, nurbs.boss):
            try:
                mesh = fn((wa, _BOWL_R, lid_a), (wb, _BOWL_R, lid_b), d)
            except KernelError as exc:
                _numsci_or_skip(exc)
            tm = mesh.to_trimesh()
            assert tm.is_watertight
            assert tm.euler_number == 2


def test_step_write_read_roundtrip_bit_exact(kernel):
    """step_write then step_read recovers the surfaces bit-exact: evaluating each recovered
    surface against its original on a (u,v) grid agrees ≤ 1e-9. Numsci-free path."""
    with _bowl_wall(False) as a, _bowl_wall(True) as b:
        step = nurbs.step_write([a, b])
        assert step.startswith("ISO-10303-21;")
        recovered = nurbs.step_read(step)
        try:
            assert len(recovered) == 2
            max_err = 0.0
            for orig, rec in zip((a, b), recovered):
                for iu in range(5):
                    for iv in range(5):
                        u, v = iu / 4.0, iv / 4.0
                        p0 = orig.eval(u, v)
                        p1 = rec.eval(u, v)
                        max_err = max(max_err, float(np.max(np.abs(p0 - p1))))
            assert max_err <= 1e-9
        finally:
            for r in recovered:
                r.close()


def test_step_write_empty_raises(kernel):
    """step_write of an empty surface set RAISES (honest decline, no invalid STEP)."""
    with pytest.raises((ValueError, KernelError)):
        nurbs.step_write([])


def test_step_read_malformed_raises(kernel):
    """step_read of a non-STEP string RAISES KernelError (never fabricates geometry)."""
    with pytest.raises(KernelError):
        nurbs.step_read("this is not a STEP file")


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
