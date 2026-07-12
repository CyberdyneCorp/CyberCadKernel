"""N7 — Variable-section swept handle (variable-section sweep + two-rail sweep).

An ergonomic handle body built by ``nurbs.sweep_variable``: a circular profile
is transported along a curved path with a per-station SCALE (the grip bulges in
the middle and necks at the ends) and TWIST, then skinned into one surface. The
companion ``nurbs.sweep_two_rail`` is also exercised to build a second surface
(a spine bar whose section rides two rail curves), showing both sweep flavours.

Feature family: Wave-D surfacing — ``cc_nurbs_sweep_variable`` / ``cc_nurbs_sweep_two_rail``.

Exact-NURBS API exercised:
    nurbs.interp_curve / nurbs.circle          -> profile, path, rail Curves
    nurbs.sweep_variable(profile, path, ...)   -> the variable-section handle Surface
    nurbs.sweep_two_rail(profile, r0, r1, ...) -> the two-rail spine Surface
    Surface.tessellate()                       -> per-surface display Mesh
"""

from __future__ import annotations

import numpy as np

from _nurbs_gallery import TESS_NU, TESS_NV, run_and_report

NAME = "n7_variable_swept_handle"


def build():
    from cybercadkernel import nurbs

    # ── Variable-section sweep: a round grip along a gently curved path ──────────
    profile = nurbs.circle((0, 0, 0), (0, 0, 1), (1, 0, 0), 2.5)
    path_pts = [(0, 0, z) for z in np.linspace(0.0, 40.0, 6)]
    # Bow the path in x so the handle arcs like a grip.
    for k, z in enumerate(np.linspace(0.0, 40.0, 6)):
        path_pts[k] = (6.0 * np.sin(np.pi * z / 40.0), 0.0, z)
    path = nurbs.interp_curve(np.array(path_pts, float), 3, 1)

    stations = 7
    # Grip bulges in the middle (scale up), necks at the ends; slight twist.
    scales = [0.8, 1.1, 1.4, 1.5, 1.4, 1.1, 0.8]
    twists = [0.0, 0.1, 0.25, 0.35, 0.25, 0.1, 0.0]
    handle = nurbs.sweep_variable(profile, path, (0, 0, 1), scales, twists, stations, degree_v=3)

    # ── Two-rail sweep: a spine bar whose section rides two rails ────────────────
    rail0 = nurbs.interp_curve(np.array([(3, 12, 0), (3.4, 12, 20), (3, 12, 40)], float), 2, 1)
    rail1 = nurbs.interp_curve(np.array([(-3, 12, 0), (-3.4, 12, 20), (-3, 12, 40)], float), 2, 1)
    spine_profile = nurbs.interp_curve(
        np.array([(3, 12, 0), (0, 14.5, 0), (-3, 12, 0)], float), 2, 1
    )
    spine = nurbs.sweep_two_rail(spine_profile, rail0, rail1, (0, 0, 1),
                                 anchor0=0, anchor1=2, stations=6, degree_v=3)

    meshes = [
        handle.tessellate(TESS_NU, TESS_NV),
        spine.tessellate(TESS_NU, TESS_NV),
    ]

    for h in (profile, path, handle, rail0, rail1, spine_profile, spine):
        h.close()

    return dict(
        meshes=meshes,
        name=NAME,
        title="Variable-section swept handle",
        description=(
            "A round grip swept along a bowed path with a per-station scale (bulging "
            "mid-grip, necking at the ends) and twist, plus a two-rail-swept spine bar "
            "whose section rides two rail curves. Both sweep flavours in one piece."
        ),
        feature="cc_nurbs_sweep_variable + cc_nurbs_sweep_two_rail",
        api_calls=["nurbs.circle", "nurbs.interp_curve", "nurbs.sweep_variable",
                   "nurbs.sweep_two_rail", "Surface.tessellate"],
        is_single_surface=False,  # two swept surfaces shown together
    )


if __name__ == "__main__":
    run_and_report(build)
