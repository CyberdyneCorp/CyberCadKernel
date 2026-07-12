"""N4 — Chamfered edge + setback vertex-blended corner.

Two exact-NURBS feature families in one piece:

* ``nurbs.chamfer_variable`` — a VARIABLE-DISTANCE analytic chamfer along the
  edge where two planar faces meet (a ruled bevel band whose setback tapers
  linearly from one end of the edge to the other).
* ``nurbs.vertex_blend`` — a SETBACK VERTEX BLEND filling the N-sided corner
  where three incident fillet bands meet at a shared vertex, returning the N
  corner sub-patches that close the gap with G1 continuity across their spokes.

Feature family: Wave-F blend — ``cc_nurbs_chamfer_variable`` + ``cc_nurbs_vertex_blend``.

Exact-NURBS API exercised:
    nurbs.chamfer_variable(subA, subB, edge, ...)   -> one ruled chamfer band
    Surface.create(...)                             -> three ruled fillet bands
    nurbs.vertex_blend(fillets, sides, ...)         -> list[Surface] corner patches
    Surface.tessellate()                            -> per-surface display Mesh

Rendered as a FACE-SET (design.md §4 caveat): the chamfer band and the vertex-
blend corner patches are shown together; each is exact geometry, not sewn.
"""

from __future__ import annotations

import math

import numpy as np

from _nurbs_gallery import TESS_NU, TESS_NV, run_and_report
from cybercadkernel.nurbs import JoinMode, Surface, SurfaceEdge

NAME = "n4_chamfer_vertex_blend"


def _plane_substrate(point, normal):
    """Analytic-plane substrate packing for chamfer_variable: kind=0 (plane), a
    point on the plane, its unit normal, and the axis triple (= the normal for a
    plane), then radius + half-angle (unused for a plane)."""
    return [0.0, *point, *normal, *normal, 0.0, 0.0]


def _chamfer_band(nurbs):
    """A variable-distance chamfer along the x-axis edge shared by the z=0 floor
    (normal +z) and the y=0 wall (normal +y). The setback tapers 0.4 -> 1.0."""
    face_a = _plane_substrate((0, 0, 0), (0, 0, 1))  # floor
    face_b = _plane_substrate((0, 0, 0), (0, 1, 0))  # wall
    n_stations = 6
    edge = []
    for k in range(n_stations):
        px = k / (n_stations - 1) * 6.0
        # (point, unit edge tangent, faceA normal, faceB normal)
        edge += [px, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0]
    return nurbs.chamfer_variable(face_a, face_b, edge, n_stations, d0=0.4, d1=1.0)


def _ruled_fillet(g, body):
    """One degree-3x1 ruled fillet band: the top row is the gap arc g[0..3], the
    bottom row is g + body (the fillet running into the face)."""
    poles = []
    for i in range(4):
        p0 = g[i]
        p1 = g[i] + body[i]
        poles += [p0[0], p0[1], p0[2], 1.0, p1[0], p1[1], p1[2], 1.0]
    knots_u = [0, 0, 0, 0, 1, 1, 1, 1]
    knots_v = [0, 0, 1, 1]
    return Surface.create(3, 1, poles, 4, 2, knots_u, knots_v)


def _corner_fillets(nurbs):
    """Three ruled fillet bands whose gap arcs form a smooth, tangent-continuous
    closed 3-D loop — the well-formed setback-corner case that the vertex blend
    fills. (Mirrors the native vertex-blend 'smooth gap loop' construction.)"""
    n, radius = 3, 3.0

    def loop(t):
        a = 2 * math.pi * t
        return np.array([radius * math.cos(a), radius * math.sin(a), 0.5 * math.sin(3 * a)])

    def loop_t(t):
        a = 2 * math.pi * t
        da = 2 * math.pi
        return np.array([-radius * math.sin(a) * da, radius * math.cos(a) * da,
                         0.5 * math.cos(3 * a) * 3 * da])

    fillets = []
    for k in range(n):
        t0, t1 = k / n, (k + 1) / n
        h = t1 - t0
        p0, p1 = loop(t0), loop(t1)
        t0v, t1v = loop_t(t0), loop_t(t1)
        g = [p0, p0 + t0v * h / 3, p1 - t1v * h / 3, p1]
        body = []
        for gi in g:
            rad = np.array([gi[0], gi[1], 0.0])
            body.append(rad / np.linalg.norm(rad) + np.array([0, 0, 1.0]))
        fillets.append(_ruled_fillet(g, body))
    return fillets


def build():
    from cybercadkernel import nurbs

    meshes = []

    # (1) The chamfer band on the plane/plane edge.
    chamfer = _chamfer_band(nurbs)
    meshes.append(chamfer.tessellate(TESS_NU, TESS_NV))

    # (2) The three incident fillet bands + the vertex-blend corner patches that
    #     fill the gap where they meet.
    fillets = _corner_fillets(nurbs)
    sides = [SurfaceEdge.V0] * len(fillets)   # the V0 iso-edge of each faces the gap
    setbacks = [0.0] * len(fillets)
    reverses = [0] * len(fillets)
    corner_patches = nurbs.vertex_blend(fillets, sides, setbacks, reverses,
                                        mode=JoinMode.G1, cap=64)
    for f in fillets:
        meshes.append(f.tessellate(TESS_NV, 6))
    for p in corner_patches:
        meshes.append(p.tessellate(TESS_NV, TESS_NV))

    chamfer.close()
    for f in fillets:
        f.close()
    for p in corner_patches:
        p.close()

    return dict(
        meshes=meshes,
        name=NAME,
        title="Chamfered edge + vertex-blended corner",
        description=(
            "A variable-distance analytic chamfer along a plane/plane edge (setback "
            "tapering 0.4 -> 1.0), plus a setback vertex blend filling the 3-sided "
            f"corner where three fillet bands meet ({len(corner_patches)} G1 corner "
            "sub-patches). Shown as a face-set; each surface is exact geometry."
        ),
        feature="cc_nurbs_chamfer_variable + cc_nurbs_vertex_blend",
        api_calls=["nurbs.chamfer_variable", "Surface.create", "nurbs.vertex_blend",
                   "Surface.tessellate"],
        is_single_surface=False,
    )


if __name__ == "__main__":
    run_and_report(build)
