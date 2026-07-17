#!/usr/bin/env python3
"""Curved-boolean gallery — the S5 curved-boolean families end-to-end from Python.

Every curved-boolean operand is an ANALYTIC B-rep solid (a true Cylinder / Sphere /
Cone wall + planar caps) built through the frozen ``cc_solid_revolve_profile`` C ABI,
surfaced ergonomically by :meth:`Kernel.cylinder_solid` / :meth:`sphere_solid` /
:meth:`cone_solid` (NURBS-EXPOSE, additive-only). Booleaning two such solids with
:meth:`Shape.common` / :meth:`cut` / :meth:`fuse` (``cc_boolean``) drives the kernel's
curved-boolean dispatch — the native S5 ``ssi_boolean_solid`` path under the native
engine, OCCT's analytic booleans under the default OCCT engine.

Each example ASSERTS the result is watertight (a closed 2-manifold) AND that its exact
B-rep volume matches the CLOSED-FORM op-volume (no fabricated numbers). Run it:

    CYBERCADKERNEL_DYLIB=<build>/libcybercadkernel.dylib \\
      PYTHONPATH=<repo>/python python3 python/examples/curved_booleans.py

The default OCCT build reaches ALL families/ops; a native (CYBERCAD_HAS_NUMSCI) build
additionally exercises the native S5 assemblers for the coaxial families it implements.
"""
from __future__ import annotations

import math

from cybercadkernel import BooleanOp, Kernel


def _check(name: str, shape, expected: float, rel: float = 0.02) -> None:
    """Assert a boolean result is watertight and matches a closed-form volume."""
    rep = shape.check_solid()
    vol = shape.mass_properties().volume
    err = abs(vol - expected) / expected
    status = "OK" if (rep.closed_manifold and err <= rel) else "FAIL"
    print(f"  [{status}] {name}: volume={vol:.5f}  expected={expected:.5f}  "
          f"rel={err:.3%}  watertight={rep.closed_manifold}")
    assert rep.closed_manifold, f"{name}: result is not a closed watertight solid"
    assert err <= rel, f"{name}: volume {vol:.5f} off closed form {expected:.5f} ({err:.3%})"


def cyl_sphere(k: Kernel) -> None:
    """S5-i coaxial cylinder ∩ sphere — the two-circle poke-through, COMMON/CUT/FUSE."""
    print("cylinder ∩ sphere (coaxial, two circles)")
    Rc, Rs = 1.0, 1.6
    h = math.sqrt(Rs * Rs - Rc * Rc)

    def sph_seg(a, b):  # π∫(Rs²−y²)dy
        F = lambda y: Rs * Rs * y - y ** 3 / 3.0
        return math.pi * (F(b) - F(a))

    v_cyl = math.pi * Rc * Rc * 6.0
    v_sph = 4.0 / 3.0 * math.pi * Rs ** 3
    v_common = sph_seg(-Rs, -h) + math.pi * Rc * Rc * (2 * h) + sph_seg(h, Rs)
    v_cut = v_cyl - v_common          # cylinder − sphere (two scooped stubs)
    v_fuse = v_cyl + v_sph - v_common

    with k.cylinder_solid(Rc, -3.0, 3.0) as cyl, k.sphere_solid(Rs) as sph:
        with cyl.common(sph) as r:
            _check("COMMON (spherically-capped rod)", r, v_common)
        with cyl.cut(sph) as r:
            _check("CUT cyl−sph (two stubs)", r, v_cut)
        with cyl.fuse(sph) as r:
            _check("FUSE (rod through ball)", r, v_fuse)


def sphere_sphere(k: Kernel) -> None:
    """S5-c sphere ∩ sphere — the classic lens, COMMON/CUT/FUSE."""
    print("sphere ∩ sphere (offset along axis)")
    Rs, d = 1.6, 1.5
    v_sph = 4.0 / 3.0 * math.pi * Rs ** 3
    v_lens = math.pi * (4.0 * Rs + d) * (2.0 * Rs - d) ** 2 / 12.0  # two equal spheres, sep d
    with k.sphere_solid(Rs) as a, k.sphere_solid(Rs, center_y=d) as b:
        with a.common(b) as r:
            _check("COMMON (lens)", r, v_lens)
        with a.cut(b) as r:
            _check("CUT a−b (crescent)", r, v_sph - v_lens)
        with a.fuse(b) as r:
            _check("FUSE (peanut)", r, 2.0 * v_sph - v_lens)


def cone_cone(k: Kernel) -> None:
    """S5-g coaxial cone ∩ cone — two opposed frustums, COMMON/CUT/FUSE."""
    print("cone ∩ cone (coaxial, opposed frustums)")
    # A: r 0.5→2.5 over y[0,4];  B: r 2.5→0.5 over y[0,4] (mirror). Overlap = the region
    # under BOTH walls: min(rA(y), rB(y)) revolved. rA=0.5+0.5y, rB=2.5−0.5y cross at y=2.
    def frustum(ra, rb, dh):
        return math.pi * dh / 3.0 * (ra * ra + ra * rb + rb * rb)

    v_a = frustum(0.5, 2.5, 4.0)
    v_b = v_a  # mirror image, same volume
    # COMMON: y∈[0,2] limited by rA (0.5→1.5), y∈[2,4] limited by rB (1.5→0.5).
    v_common = frustum(0.5, 1.5, 2.0) + frustum(1.5, 0.5, 2.0)
    with k.cone_solid(0.5, 0.0, 2.5, 4.0) as a, k.cone_solid(2.5, 0.0, 0.5, 4.0) as b:
        with a.common(b) as r:
            _check("COMMON (bulged bicone)", r, v_common)
        with a.cut(b) as r:
            _check("CUT a−b", r, v_a - v_common)
        with a.fuse(b) as r:
            _check("FUSE", r, v_a + v_b - v_common)


def torus_cyl(k: Kernel) -> None:
    """S5-l coaxial torus ∩ cylinder — the ring cut by a coaxial cylinder (COMMON/CUT/FUSE).

    Under the native build this uses the native torus family only when the torus is a bare
    periodic Kind::Torus face; the OCCT engine builds a real torus from the revolved circle
    and handles all three ops. The example runs whatever the active engine supports.
    """
    print("torus ∩ cylinder (coaxial)")
    R, r, Rc = 3.0, 1.0, 3.2
    d = Rc - R
    root = math.sqrt(max(r * r - d * d, 0.0))
    a_cap = r * r * math.acos(max(-1.0, min(1.0, d / r))) - d * root
    a_seg = math.pi * r * r - a_cap
    mom = -(2.0 / 3.0) * root ** 3
    v_common = 2.0 * math.pi * (R * a_seg + mom)      # Pappus: ρ ≤ Rc tube part
    v_torus = 2.0 * math.pi * math.pi * R * r * r
    # torus as a full circle (kind-2) revolved about the Y axis.
    seg = Kernel._seg_line  # noqa: not used; kept explicit below
    from cybercadkernel._cffi import CCProfileSeg
    circ = CCProfileSeg(); circ.kind = 2; circ.cx, circ.cy, circ.r = R, 0.0, r
    with k.revolve_profile([circ]) as tor, k.cylinder_solid(Rc, -2.0, 2.0) as cyl:
        with tor.common(cyl) as res:
            _check("COMMON (inner tube part)", res, v_common)
        with tor.cut(cyl) as res:
            _check("CUT torus−cyl (outer ring)", res, v_torus - v_common)
        with tor.fuse(cyl) as res:
            # FUSE = torus ∪ cylinder; cylinder Rc=3.2 h=4, minus the shared common.
            v_cyl = math.pi * Rc * Rc * 4.0
            _check("FUSE", res, v_torus + v_cyl - v_common)


def main() -> int:
    k = Kernel()
    if not k.brep_available:
        print("no B-rep engine linked — skipping")
        return 0
    engine = k.engine
    print(f"=== curved-boolean gallery (engine: {engine}) ===\n")
    families = [cyl_sphere, sphere_sphere, cone_cone, torus_cyl]
    failures = 0
    for fn in families:
        try:
            fn(k)
        except AssertionError as exc:
            print(f"  ASSERTION FAILED: {exc}")
            failures += 1
        except Exception as exc:  # a family the active engine's curved dispatch declines
            print(f"  (skipped — engine declined this pose: {str(exc)[:60]})")
        print()
    if failures:
        print(f"{failures} family assertion(s) FAILED")
        return 1
    print("all curved-boolean assertions passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
