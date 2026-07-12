"""N3 — Filleted freeform boss (G2 rolling-ball fillet between two freeform faces).

Two freeform faces — a floor and a wall meeting at a concave dihedral — are
joined by a ``nurbs.fillet_freeform_g2`` rolling-ball fillet at a fixed radius.
The fillet is a curvature-continuous (G2) band that blends smoothly into both
host faces; the honest-decline contract means an over-radius ball that won't fit
raises rather than emitting a self-intersecting band.

Feature family: Wave-F blend — ``cc_nurbs_fillet_freeform_g2``.

Exact-NURBS API exercised:
    Surface.create(...)                        -> two freeform bicubic faces
    nurbs.fillet_freeform_g2(a, b, r, ...)     -> the G2 fillet band (a Surface)
    Surface.tessellate()                       -> per-surface display Mesh

Rendered as a FACE-SET of the floor + wall + fillet band (design.md §4 caveat):
each surface is real exact geometry; they are shown together, not sewn.
"""

from __future__ import annotations

from _nurbs_gallery import TESS_NU, TESS_NV, run_and_report

NAME = "n3_filleted_freeform_boss"


def _bicubic(nurbs, fn, x0, x1, y0, y1, n=6):
    """A non-rational bicubic (degree 3x3, 6x6 net) freeform face, with pole
    (i,j) placed at fn(x, y) over the parameter rectangle."""
    from cybercadkernel.nurbs import Surface

    knots = [0, 0, 0, 0, 1 / 3, 2 / 3, 1, 1, 1, 1]
    poles = []
    for i in range(n):
        u = i / (n - 1)
        for j in range(n):
            v = j / (n - 1)
            x = x0 + (x1 - x0) * u
            y = y0 + (y1 - y0) * v
            X, Y, Z = fn(x, y)
            poles += [X, Y, Z, 1.0]
    return Surface.create(3, 3, poles, n, n, knots, knots)


def build():
    from cybercadkernel import nurbs

    # A concave floor/wall dihedral: floor lies in z=0 for y>=0; wall rises in +z
    # from the shared edge at y=0. The rolling ball sits in the valley between.
    floor = _bicubic(nurbs, lambda x, y: (x, y, 0.0), 0, 4, 0.0, 3.0)
    wall = _bicubic(nurbs, lambda x, y: (x, 0.0, y), 0, 4, 0.0, 3.0)

    radius = 0.3
    fillet = nurbs.fillet_freeform_g2(
        floor, wall, radius,
        center0=(2.0, radius * 1.5, radius * 1.5),  # ball centre seeded in the valley
        spine_dir=(1.0, 0.0, 0.0),                  # crease runs along +x
        side_a=1.0, side_b=-1.0,                    # which side of each face the ball rides
        step_len=0.4, n_stations=6, n_section_samples=8,
    )

    meshes = [
        floor.tessellate(TESS_NV, TESS_NV),
        wall.tessellate(TESS_NV, TESS_NV),
        fillet.tessellate(TESS_NU, TESS_NV),
    ]

    floor.close()
    wall.close()
    fillet.close()

    return dict(
        meshes=meshes,
        name=NAME,
        title="Filleted freeform boss (G2 rolling-ball fillet)",
        description=(
            "A G2 rolling-ball fillet blends a freeform floor into a freeform wall "
            "across their concave dihedral. Shown as a face-set of floor + wall + "
            "fillet band; each is exact geometry, the ball radius is 0.3."
        ),
        feature="cc_nurbs_fillet_freeform_g2 (freeform G2 fillet)",
        api_calls=["Surface.create", "nurbs.fillet_freeform_g2", "Surface.tessellate"],
        is_single_surface=False,
    )


if __name__ == "__main__":
    run_and_report(build)
