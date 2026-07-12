"""N1 — Lofted bracket (skin / loft over section curves).

A tapered bracket transition built by SKINNING a stack of section curves: a
wide rounded-rectangle mouth at the base narrows through intermediate sections
to a small circular spout at the top. ``nurbs.skin`` produces ONE tensor-product
surface that contains every section curve as an iso-curve — the classic loft.

Feature family: Wave-D surfacing — ``cc_nurbs_skin`` (with ``cc_nurbs_gordon`` as
the network-interpolation sibling; skin is used here because a clean loft over
ordered sections is the canonical bracket-transition case).

Exact-NURBS API exercised:
    nurbs.circle / nurbs.interp_curve  -> section Curves
    nurbs.skin(sections, degree_v)     -> one lofted Surface
    Surface.tessellate()               -> single-surface display Mesh
"""

from __future__ import annotations

import numpy as np

from _nurbs_gallery import TESS_NU, TESS_NV, run_and_report

NAME = "n1_lofted_bracket"


def _superellipse_curve(nurbs, z, half_w, half_h, power, n=64):
    """A closed section curve in the plane z=const: a superellipse whose
    ``power`` blends between a rectangle-ish mouth (power<1) and a circle
    (power=1). Interpolated as a NON-rational cubic so every section skinned
    together shares the same (non-rational) representation, which ``nurbs.skin``
    requires."""
    pts = []
    for t in np.linspace(0.0, 2.0 * np.pi, n, endpoint=True):
        c, s = np.cos(t), np.sin(t)
        x = np.sign(c) * (abs(c) ** power) * half_w
        y = np.sign(s) * (abs(s) ** power) * half_h
        pts.append((x, y, z))
    pts[-1] = pts[0]  # close exactly
    return nurbs.interp_curve(np.array(pts, float), 3, 1)


def build():
    from cybercadkernel import nurbs

    # Ordered sections from a wide rectangle-ish mouth (z=0) up to a small round
    # spout (z=30). Every section is a NON-rational interpolated curve so the
    # loft is homogeneous; nurbs.skin lofts one surface through all of them.
    sections = [
        _superellipse_curve(nurbs, 0.0, 20.0, 12.0, 0.45),
        _superellipse_curve(nurbs, 10.0, 14.0, 10.0, 0.65),
        _superellipse_curve(nurbs, 20.0, 8.0, 8.0, 1.0),
        _superellipse_curve(nurbs, 30.0, 6.0, 6.0, 1.0),
    ]

    surface = nurbs.skin(sections, degree_v=3)
    mesh = surface.tessellate(TESS_NU, TESS_NV)

    for c in sections:
        c.close()
    surface.close()

    return dict(
        meshes=[mesh],
        name=NAME,
        title="Lofted bracket transition",
        description=(
            "A skinned transition bracket: a wide rounded-rectangle mouth lofted "
            "through intermediate sections up to a small round spout — one "
            "tensor-product surface containing every section as an iso-curve."
        ),
        feature="cc_nurbs_skin (loft over section curves; gordon sibling)",
        api_calls=["nurbs.interp_curve", "nurbs.skin", "Surface.tessellate"],
        is_single_surface=True,
    )


if __name__ == "__main__":
    run_and_report(build)
