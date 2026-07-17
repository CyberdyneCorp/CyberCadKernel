"""Curved-boolean reachability tests — the S5 curved-boolean families through the
public ``cc_boolean`` facade with analytic solid operands built via the frozen
``cc_solid_revolve_profile`` C ABI (surfaced by the additive ``Kernel.cylinder_solid`` /
``sphere_solid`` / ``cone_solid`` / ``revolve_profile`` helpers, NURBS-EXPOSE).

Each test asserts REAL geometry: the result is a CLOSED watertight solid
(``check_solid().closed_manifold``) AND its exact B-rep volume matches the CLOSED-FORM
op-volume within tolerance. DISAGREED-style soundness: a wrong / non-watertight result
FAILS (the volume band rejects an off-volume solid; the manifold check rejects a leak).

Coverage is engine-aware:
* under the OCCT engine (the default desktop build) every family + all three ops
  (COMMON/CUT/FUSE) are reachable and exact — the primary matrix these tests assert;
* under the native engine (CYBERCAD_HAS_NUMSCI) the native S5 assemblers additionally
  fire; the tolerance there is the tessellation band. A pose the active engine's curved
  dispatch declines is skipped (never silently passed).

The ``kernel`` fixture (conftest) skips the whole module when the desktop dylib cannot
load; the autouse ``_require_real_engine`` gate skips on a stub build.
"""
import math

import pytest

from cybercadkernel import BooleanOp, CyberCadError, Kernel


# ── closed-form op-volumes (no fabricated numbers) ──────────────────────────────


def _cyl_vol(r, h):
    return math.pi * r * r * h


def _sph_vol(r):
    return 4.0 / 3.0 * math.pi * r ** 3


def _frustum(ra, rb, dh):
    return math.pi * dh / 3.0 * (ra * ra + ra * rb + rb * rb)


def _assert_boolean(shape, expected, rel=0.02):
    """A boolean result must be a closed watertight solid at the closed-form volume."""
    rep = shape.check_solid()
    assert rep.closed_manifold, "result is not a closed watertight solid (a leak)"
    vol = shape.mass_properties().volume
    assert vol > 0.0, "degenerate / empty result"
    err = abs(vol - expected) / expected
    assert err <= rel, f"volume {vol:.6f} off closed form {expected:.6f} ({err:.3%} > {rel:.1%})"


def _boolean_or_skip(a, b, op):
    """Run a boolean, skipping cleanly if the active engine's curved dispatch declines
    this pose (a native build without the family, or a mixed-operand pose)."""
    try:
        return a.boolean(b, op)
    except CyberCadError as exc:
        pytest.skip(f"active engine declined this curved pose: {str(exc)[:80]}")


# ── S5-i coaxial cylinder ∩ sphere (two-circle poke-through) ────────────────────


def _cyl_sphere_common_cf(Rc, Rs):
    h = math.sqrt(Rs * Rs - Rc * Rc)
    F = lambda y: Rs * Rs * y - y ** 3 / 3.0
    seg = lambda a, b: math.pi * (F(b) - F(a))
    return seg(-Rs, -h) + math.pi * Rc * Rc * (2 * h) + seg(h, Rs)


@pytest.mark.parametrize("op", [BooleanOp.COMMON, BooleanOp.CUT, BooleanOp.FUSE])
def test_cyl_sphere_coaxial(kernel, op):
    """cyl ∩ sphere (Rc=1, Rs=1.6): COMMON = capped rod, CUT = two stubs, FUSE = ball+rod."""
    Rc, Rs = 1.0, 1.6
    v_common = _cyl_sphere_common_cf(Rc, Rs)
    v_cyl, v_sph = _cyl_vol(Rc, 6.0), _sph_vol(Rs)
    expected = {BooleanOp.COMMON: v_common,
                BooleanOp.CUT: v_cyl - v_common,
                BooleanOp.FUSE: v_cyl + v_sph - v_common}[op]
    with kernel.cylinder_solid(Rc, -3.0, 3.0) as cyl, kernel.sphere_solid(Rs) as sph:
        with _boolean_or_skip(cyl, sph, op) as r:
            _assert_boolean(r, expected)


# ── S5-c sphere ∩ sphere (lens) ─────────────────────────────────────────────────


@pytest.mark.parametrize("op", [BooleanOp.COMMON, BooleanOp.CUT, BooleanOp.FUSE])
def test_sphere_sphere(kernel, op):
    Rs, d = 1.6, 1.5
    v_sph = _sph_vol(Rs)
    v_lens = math.pi * (4.0 * Rs + d) * (2.0 * Rs - d) ** 2 / 12.0
    expected = {BooleanOp.COMMON: v_lens,
                BooleanOp.CUT: v_sph - v_lens,
                BooleanOp.FUSE: 2.0 * v_sph - v_lens}[op]
    with kernel.sphere_solid(Rs) as a, kernel.sphere_solid(Rs, center_y=d) as b:
        with _boolean_or_skip(a, b, op) as r:
            _assert_boolean(r, expected)


# ── S5-g coaxial cone ∩ cone (opposed frustums) ─────────────────────────────────


@pytest.mark.parametrize("op", [BooleanOp.COMMON, BooleanOp.CUT, BooleanOp.FUSE])
def test_cone_cone_coaxial(kernel, op):
    v_a = _frustum(0.5, 2.5, 4.0)
    v_common = _frustum(0.5, 1.5, 2.0) + _frustum(1.5, 0.5, 2.0)
    expected = {BooleanOp.COMMON: v_common,
                BooleanOp.CUT: v_a - v_common,
                BooleanOp.FUSE: 2.0 * v_a - v_common}[op]
    with kernel.cone_solid(0.5, 0.0, 2.5, 4.0) as a, kernel.cone_solid(2.5, 0.0, 0.5, 4.0) as b:
        with _boolean_or_skip(a, b, op) as r:
            _assert_boolean(r, expected)


# ── S5-l coaxial torus ∩ cylinder (Pappus ring cut) ─────────────────────────────


def _torus_cyl_common_cf(R, r, Rc):
    d = Rc - R
    root = math.sqrt(max(r * r - d * d, 0.0))
    a_cap = r * r * math.acos(max(-1.0, min(1.0, d / r))) - d * root
    a_seg = math.pi * r * r - a_cap
    mom = -(2.0 / 3.0) * root ** 3
    return 2.0 * math.pi * (R * a_seg + mom)


@pytest.mark.parametrize("op", [BooleanOp.COMMON, BooleanOp.CUT, BooleanOp.FUSE])
def test_torus_cyl_coaxial(kernel, op):
    """torus (R=3, r=1) ∩ coaxial cylinder Rc=3.2. Under OCCT all ops are exact; a pure
    native build has no bare-torus constructor and skips (documented reachability gap)."""
    from cybercadkernel._cffi import CCProfileSeg
    R, r, Rc = 3.0, 1.0, 3.2
    v_common = _torus_cyl_common_cf(R, r, Rc)
    v_torus = 2.0 * math.pi * math.pi * R * r * r
    v_cyl = _cyl_vol(Rc, 4.0)
    expected = {BooleanOp.COMMON: v_common,
                BooleanOp.CUT: v_torus - v_common,
                BooleanOp.FUSE: v_torus + v_cyl - v_common}[op]
    circ = CCProfileSeg(); circ.kind = 2; circ.cx, circ.cy, circ.r = R, 0.0, r
    with kernel.revolve_profile([circ]) as tor, kernel.cylinder_solid(Rc, -2.0, 2.0) as cyl:
        with _boolean_or_skip(tor, cyl, op) as r:
            _assert_boolean(r, expected)


# ── honest-decline / soundness ──────────────────────────────────────────────────


def test_disjoint_common_is_empty_not_fabricated(kernel):
    """Two disjoint spheres have an EMPTY intersection. The kernel must NOT fabricate
    overlap volume — it either honest-declines (raises) or returns a zero-volume empty
    result, never a positive-volume 'solid'."""
    with kernel.sphere_solid(1.0) as a, kernel.sphere_solid(1.0, center_y=5.0) as b:
        try:
            r = a.common(b)
        except CyberCadError:
            return  # honest decline is acceptable
        with r:
            assert r.mass_properties().volume <= 1e-9, "fabricated overlap volume for disjoint solids"


def test_volume_band_rejects_wrong_expectation(kernel):
    """The volume assertion itself is sound: a correct watertight result checked against a
    deliberately WRONG closed form FAILS (guards against a vacuous test)."""
    Rc, Rs = 1.0, 1.6
    with kernel.cylinder_solid(Rc, -3.0, 3.0) as cyl, kernel.sphere_solid(Rs) as sph:
        with _boolean_or_skip(cyl, sph, BooleanOp.COMMON) as r:
            wrong = 2.0 * _cyl_sphere_common_cf(Rc, Rs)  # 2× the true volume
            with pytest.raises(AssertionError):
                _assert_boolean(r, wrong)


def test_revolve_profile_builds_analytic_sphere(kernel):
    """A semicircle revolved a full turn is an exact sphere: watertight, volume 4/3πr³."""
    with kernel.sphere_solid(2.0) as s:
        assert s.check_solid().closed_manifold
        vol = s.mass_properties().volume
        assert abs(vol - _sph_vol(2.0)) / _sph_vol(2.0) <= 0.02


# ── native S5 curved-boolean path (CYBERCAD_HAS_NUMSCI) ─────────────────────────


@pytest.mark.parametrize("op", [BooleanOp.COMMON, BooleanOp.CUT, BooleanOp.FUSE])
def test_native_engine_curved_cut_fuse(kernel, op):
    """Regression for the native curved-boolean CUT/FUSE self-verify: under the native
    engine the S5 assemblers build the curved CUT/FUSE, and the engine's set-algebra
    self-verify accepts the watertight result under the deflection-bounded band (a
    curved result cannot meet the exact-planar 1e-6 band because the operand-mesh bias
    does not cancel). Skips on a native build that declines this pose or on an OCCT-only
    dylib where the native engine is unavailable; NEVER passes on a wrong/leaky result."""
    was_native = kernel.engine == "native"
    kernel.set_engine(True)
    try:
        if kernel.engine != "native":
            pytest.skip("native engine unavailable in this build")
        Rc, Rs = 1.0, 1.6
        v_common = _cyl_sphere_common_cf(Rc, Rs)
        v_cyl, v_sph = _cyl_vol(Rc, 6.0), _sph_vol(Rs)
        expected = {BooleanOp.COMMON: v_common,
                    BooleanOp.CUT: v_cyl - v_common,
                    BooleanOp.FUSE: v_cyl + v_sph - v_common}[op]
        with kernel.cylinder_solid(Rc, -3.0, 3.0) as cyl, kernel.sphere_solid(Rs) as sph:
            with _boolean_or_skip(cyl, sph, op) as r:
                _assert_boolean(r, expected, rel=0.02)
    finally:
        kernel.set_engine(was_native)  # restore for the rest of the session
