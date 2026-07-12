"""N8 — Trim-boolean pocket (parameter-space region boolean on trim regions).

A rectangular face has a diamond pocket cut into it by a PARAMETER-SPACE trim
boolean: ``nurbs.trim_region_boolean`` with the DIFFERENCE operator subtracts the
pocket region (a diamond) from the face region (a rectangle), returning the
result loops as closed (u,v) polylines — an outer boundary loop and an inner
hole loop. We map the resulting region onto the host plane surface and render it
as the trimmed RESULT FACE. The INTERSECTION operator is also run to report its
area, exercising both ∖ and ∩.

Feature family: Wave-G intersection/trim — ``cc_nurbs_trim_region_boolean``.

Exact-NURBS API exercised:
    Curve.create(...)                                  -> region loop Curves (u,v)
    nurbs.trim_region_boolean(A, B, DIFFERENCE/INTERSECT) -> (loops, area)
    nurbs.plane(...)                                   -> host plane Surface
    Surface.eval(u, v)                                 -> map the region to 3-D

The result face is a single flat trimmed region: outer loop minus the hole loop,
triangulated for display.
"""

from __future__ import annotations

import numpy as np

from _nurbs_gallery import run_and_report
from cybercadkernel.nurbs import Curve, TrimBoolOp

NAME = "n8_trim_boolean_pocket"


def _loop_curve(verts_uv):
    """A closed region loop as a degree-1 clamped B-spline whose poles ARE the
    polygon vertices in (u, v) — exactly how the facade reads a trim loop."""
    v = list(verts_uv)
    if v[0] != v[-1]:
        v = v + [v[0]]
    n = len(v)
    poles = []
    for (u, w) in v:
        poles += [u, w, 0.0, 1.0]
    knots = np.linspace(0.0, 1.0, n + 2).tolist()
    for i in range(2):  # clamp both ends (degree 1)
        knots[i] = knots[1]
        knots[-1 - i] = knots[-2]
    return Curve.create(1, poles, knots)


def _signed_area(uv):
    x, y = uv[:, 0], uv[:, 1]
    return 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y)


def _resample_loop(uv, n):
    """Resample a closed (u,v) polyline to ``n`` points at uniform arc-length,
    with a consistent CCW winding."""
    if _signed_area(uv) < 0:
        uv = uv[::-1]
    closed = np.vstack([uv, uv[:1]])
    seg = np.linalg.norm(np.diff(closed, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    targets = np.linspace(0.0, total, n, endpoint=False)
    out = np.empty((n, 2))
    for i, tval in enumerate(targets):
        k = int(np.searchsorted(s, tval, side="right") - 1)
        k = min(max(k, 0), len(seg) - 1)
        f = 0.0 if seg[k] == 0 else (tval - s[k]) / seg[k]
        out[i] = closed[k] + (closed[k + 1] - closed[k]) * f
    return out


def _fan_triangulate(uv):
    """Triangulate a convex-ish CCW loop as a triangle fan (no hole)."""
    n = len(uv)
    if _signed_area(uv) < 0:
        uv = uv[::-1]
    tris = [(0, i, i + 1) for i in range(1, n - 1)]
    return uv, np.array(tris, dtype=np.int32)


def _ring_triangulate(outer, hole, samples=64):
    """Triangulate the picture-frame region between an outer loop and a hole
    loop by resampling both to ``samples`` points and stitching the annulus into
    a triangle strip. Robust for the convex-ish trim loops here and, unlike a
    bridged ear-clip, never self-intersects."""
    o = _resample_loop(outer, samples)
    h = _resample_loop(hole, samples)
    # Align the hole's start to the nearest outer point so the strip doesn't twist.
    shift = int(np.argmin(np.linalg.norm(h - o[0], axis=1)))
    h = np.roll(h, -shift, axis=0)

    uv = np.vstack([o, h])  # outer = [0,samples), hole = [samples, 2*samples)
    tris = []
    for i in range(samples):
        o0, o1 = i, (i + 1) % samples
        h0, h1 = samples + i, samples + (i + 1) % samples
        tris.append((o0, o1, h1))
        tris.append((o0, h1, h0))
    return uv, np.array(tris, dtype=np.int32)


def _region_to_face(loops, surface, span):
    """Map result loops (u,v in [0,span]) onto the host surface and triangulate
    them into a flat display face (vertices, triangles). When the DIFFERENCE
    result has a hole (the pocket), the remaining material is the picture-frame
    region between the outer loop and the hole loop."""
    outer = None
    hole = None
    for lp in loops:
        if lp.uv.shape[0] < 3:
            continue
        if lp.outer:
            outer = lp.uv
        else:
            hole = lp.uv
    if outer is None:
        raise RuntimeError("trim boolean produced no outer result loop")

    uv, tris = (
        _ring_triangulate(outer, hole) if hole is not None else _fan_triangulate(outer)
    )

    # Map each (u,v) to 3-D via the host surface (u,v normalized into [0,1]).
    verts = np.array([surface.eval(u / span, v / span) for (u, v) in uv], float)
    return verts, tris


def build():
    from cybercadkernel import nurbs

    span = 4.0  # the (u,v) working square is [0,span] x [0,span]

    # Region A: the rectangular face. Region B: a diamond pocket inside it.
    region_a = [_loop_curve([(0, 0), (span, 0), (span, span), (0, span)])]
    region_b = [_loop_curve([(2.0, 0.8), (3.4, 2.0), (2.0, 3.2), (0.6, 2.0)])]

    diff_loops, diff_area = nurbs.trim_region_boolean(region_a, region_b, TrimBoolOp.DIFFERENCE)
    _, inter_area = nurbs.trim_region_boolean(region_a, region_b, TrimBoolOp.INTERSECT)

    # Host plane surface over [0,1]x[0,1] (a slightly domed face for a 3-D read).
    host = nurbs.plane((0, 0, 0), (0, 0, 1), (1, 0, 0), 0.0, span, 0.0, span)

    verts, tris = _region_to_face(diff_loops, host, span)

    from types import SimpleNamespace

    face = SimpleNamespace(vertices=verts, triangles=tris)
    host.close()
    for c in region_a + region_b:
        c.close()

    return dict(
        meshes=[face],
        name=NAME,
        title="Trim-boolean pocket",
        description=(
            "A rectangular face with a diamond pocket cut by a parameter-space trim "
            f"boolean: DIFFERENCE (area {diff_area:.1f}) subtracts the pocket, "
            f"INTERSECTION reports area {inter_area:.1f}. The result region — outer "
            "loop minus the hole loop — is mapped onto the host plane and shown as "
            "the trimmed result face."
        ),
        feature="cc_nurbs_trim_region_boolean (DIFFERENCE / INTERSECT)",
        api_calls=["Curve.create", "nurbs.trim_region_boolean", "nurbs.plane", "Surface.eval"],
        is_single_surface=True,  # one flat trimmed result face
    )


if __name__ == "__main__":
    run_and_report(build)
