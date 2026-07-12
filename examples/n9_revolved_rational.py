"""N9 — Revolved rational surface (exact surface of revolution, watertight).

A profile curve is revolved a full turn about an axis by ``nurbs.revolve``,
producing an EXACT RATIONAL surface of revolution — the revolve of a semicircle
is a mathematically exact sphere, and a full-turn revolve of a closed profile
tessellates to a WATERTIGHT single surface (the one case where a single
`Surface.tessellate()` IS a sewn solid, per the module caveat).

Here the profile is an off-axis circle (in the x-z plane, offset from the z-axis),
so the full revolution is an exact torus-like ring — a rational surface whose
weighted poles reproduce the circular sections exactly.

Feature family: Wave-D surfacing — ``cc_nurbs_revolve`` (exact rational).

Exact-NURBS API exercised:
    nurbs.circle(...)                          -> the rational profile Curve
    nurbs.revolve(profile, axis_pt, axis, ang) -> exact rational Surface
    Surface.rational / Surface.tessellate()    -> confirm rational + display Mesh
"""

from __future__ import annotations

import math

from _nurbs_gallery import TESS_NU, TESS_NV, run_and_report

NAME = "n9_revolved_rational"


def build():
    from cybercadkernel import nurbs

    # Profile: a circle of radius 4 centred at x=12 in the x-z plane (normal +y),
    # so revolving it about the z-axis sweeps an exact ring (torus-like) surface.
    ring_radius = 12.0
    tube_radius = 4.0
    profile = nurbs.circle((ring_radius, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), tube_radius)

    surface = nurbs.revolve(profile, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), 2.0 * math.pi)
    is_rational = surface.rational

    mesh = surface.tessellate(TESS_NU, TESS_NV)

    profile.close()
    surface.close()

    return dict(
        meshes=[mesh],
        name=NAME,
        title="Revolved rational ring",
        description=(
            "An off-axis circle revolved a full turn about the z-axis by nurbs.revolve, "
            "yielding an EXACT rational surface of revolution (a torus-like ring). "
            f"rational={is_rational}; a full-turn revolve of a closed profile "
            "tessellates watertight — the single-surface case that IS a sewn solid."
        ),
        feature="cc_nurbs_revolve (exact rational surface of revolution)",
        api_calls=["nurbs.circle", "nurbs.revolve", "Surface.tessellate"],
        is_single_surface=True,
    )


if __name__ == "__main__":
    run_and_report(build)
