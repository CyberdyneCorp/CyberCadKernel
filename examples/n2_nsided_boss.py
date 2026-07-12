"""N2 — N-sided boss cap (G2 N-sided fill over a pentagon of boundary curves).

The rounded top cap of a 5-sided boss, filled by ``nurbs.nsided_fill`` in G2
(curvature-continuous) mode. Five boundary curves form a closed pentagonal loop;
the N-sided fill returns a small fan of tensor-product patches that together fill
the loop with G2 continuity at the interior star point — the standard way to cap
an odd-sided pocket or boss where a single 4-sided patch cannot reach.

Feature family: Wave-D surfacing — ``cc_nurbs_nsided_fill`` (mode G2).

Exact-NURBS API exercised:
    nurbs.interp_curve                         -> 5 boundary Curves (closed loop)
    nurbs.nsided_fill(boundary, mode=G2)       -> list[Surface] (the fill patches)
    Surface.tessellate()                       -> per-patch display Mesh

This piece is an honest FACE-SET: the N-sided fill returns several patches that
share edges but are not sewn into one watertight surface (design.md §4 caveat).
"""

from __future__ import annotations

import numpy as np

from _nurbs_gallery import TESS_NV, run_and_report
from cybercadkernel.nurbs import Curve, NSidedMode

NAME = "n2_nsided_boss"
N_SIDES = 5
RADIUS = 12.0


def _rim_loop(t):
    """One smooth analytic closed rim: a 5-lobed radius modulation, domed in z
    with a period of 1/N so the loop is genuinely curvature-continuous where the
    N boundary segments meet (a prerequisite for a G2 N-sided fill)."""
    a = 2.0 * np.pi * t
    rr = RADIUS * (1.0 + 0.10 * np.cos(N_SIDES * a))
    return np.array([rr * np.cos(a), rr * np.sin(a), 4.0 * np.sin(N_SIDES * a)])


def _quintic_segment(nurbs, t0, t1):
    """A NON-rational quintic Bézier boundary segment matching the rim's
    position, first, AND second derivative at both ends — so adjacent segments
    join C2 (hence G2), and the segment is non-rational (G2 mode forbids rational
    boundary edges). G2 mode of the fill needs exactly this smoothness."""
    def d1(t, h=1e-5):
        return (_rim_loop(t + h) - _rim_loop(t - h)) / (2 * h)

    def d2(t, h=1e-4):
        return (_rim_loop(t + h) - 2 * _rim_loop(t) + _rim_loop(t - h)) / (h * h)

    h = t1 - t0
    p0, p1 = _rim_loop(t0), _rim_loop(t1)
    v0, v1 = d1(t0) * h, d1(t1) * h        # 1st derivative in local parameter
    a0, a1 = d2(t0) * h * h, d2(t1) * h * h  # 2nd derivative in local parameter
    # Quintic Bézier control points from endpoint (P, P', P'').
    g = [
        p0,
        p0 + v0 / 5.0,
        p0 + 2 * v0 / 5.0 + a0 / 20.0,
        p1 - 2 * v1 / 5.0 + a1 / 20.0,
        p1 - v1 / 5.0,
        p1,
    ]
    poles = []
    for p in g:
        poles += [p[0], p[1], p[2], 1.0]
    knots = [0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1]  # degree-5 clamped, 6 poles
    return Curve.create(5, poles, knots)


def build():
    from cybercadkernel import nurbs

    # A closed loop of 5 curvature-continuous boundary segments (a domed
    # pentagonal boss rim).
    boundary = [
        _quintic_segment(nurbs, k / N_SIDES, (k + 1) / N_SIDES) for k in range(N_SIDES)
    ]

    patches = nurbs.nsided_fill(boundary, mode=NSidedMode.G2, tol=1e-4, cap=64)
    meshes = [p.tessellate(TESS_NV, TESS_NV) for p in patches]

    for c in boundary:
        c.close()
    for p in patches:
        p.close()

    return dict(
        meshes=meshes,
        name=NAME,
        title="N-sided boss cap (G2)",
        description=(
            "A 5-sided boss cap filled by an N-sided fill in G2 mode: five domed "
            "boundary curves close a pentagonal rim, and the fill returns a fan of "
            f"{len(patches)} curvature-continuous patches meeting at a central star point."
        ),
        feature="cc_nurbs_nsided_fill (mode G2, pentagon boundary)",
        api_calls=["Curve.create", "nurbs.nsided_fill", "Surface.tessellate"],
        is_single_surface=False,  # honest face-set: several patches, not sewn
    )


if __name__ == "__main__":
    run_and_report(build)
