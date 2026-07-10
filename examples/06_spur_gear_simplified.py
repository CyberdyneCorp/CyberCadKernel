"""06 — Simplified spur gear.

A simplified spur gear: a cylindrical blank with N tooth gaps cut by a circular
array of boolean cuts, a central bore, and a rectangular keyway. The tooth gaps
are trapezoidal notches cut around the rim — this is a *stylised* gear, NOT a
true involute profile (real involute teeth need a dedicated gear generator).

Features exercised: cylinder primitive, a circular boolean pattern (N positioned
notch cutters cut around the rim), boolean CUT for the bore and keyway.
"""

from __future__ import annotations

import math

from _gallery import run_and_report
from _shapes import cylinder_z_at

NAME = "06_spur_gear_simplified"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
TEETH = 20
OUTER_RADIUS = 55.0     # tip radius
ROOT_RADIUS = 47.0      # bottom of the tooth gaps
THICKNESS = 16.0
BORE_RADIUS = 12.0
KEYWAY_WIDTH = 6.0
KEYWAY_DEPTH = 3.0      # extra depth beyond the bore
GAP_TOP_WIDTH = 9.0     # circumferential width of a gap at the tip
GAP_ROOT_WIDTH = 5.0    # narrower at the root -> trapezoidal notch

TITLE = "Spur gear (simplified, 20 teeth)"
DESCRIPTION = (
    "Stylised 20-tooth spur gear: a blank with a circular array of trapezoidal "
    "tooth-gap cuts, a central bore and a keyway. Not a true involute profile."
)
FEATURES = [
    "cylinder primitive",
    "circular boolean pattern",
    "boolean cut (bore + keyway)",
]


def _notch_cutter(kernel):
    """A single trapezoidal tooth-gap cutter, placed at angle 0 (along +X).

    Built as an extruded trapezoid spanning from just outside the tip radius
    down to the root radius, narrowing toward the root. Extruded through the
    full gear thickness (with clearance top and bottom).
    """
    r_out = OUTER_RADIUS + 2.0   # start outside the tip so the cut is clean
    r_root = ROOT_RADIUS
    top_h = GAP_TOP_WIDTH / 2.0
    root_h = GAP_ROOT_WIDTH / 2.0
    # Trapezoid in the XY plane, symmetric about the X axis.
    profile = [
        (r_root, -root_h),
        (r_out, -top_h),
        (r_out, top_h),
        (r_root, root_h),
    ]
    return kernel.extrude(profile, THICKNESS + 2.0).translate(0, 0, -1.0)


def build(kernel):
    # Gear blank: a cylinder at the tip radius.
    body = cylinder_z_at(kernel, 0.0, 0.0, OUTER_RADIUS, THICKNESS)

    # Cut N tooth gaps in a circular array.
    for i in range(TEETH):
        angle = 2.0 * math.pi * i / TEETH
        cutter = _notch_cutter(kernel).rotate((0, 0, 0), (0, 0, 1), angle)
        body = body.cut(cutter)

    # Central bore.
    bore = cylinder_z_at(kernel, 0.0, 0.0, BORE_RADIUS, THICKNESS + 2.0).translate(
        0, 0, -1.0
    )
    body = body.cut(bore)

    # Keyway: a rectangular slot on +Y side of the bore.
    key_w = KEYWAY_WIDTH
    key_r = BORE_RADIUS + KEYWAY_DEPTH
    keyway_profile = [
        (-key_w / 2.0, 0.0),
        (key_w / 2.0, 0.0),
        (key_w / 2.0, key_r),
        (-key_w / 2.0, key_r),
    ]
    keyway = kernel.extrude(keyway_profile, THICKNESS + 2.0).translate(0, 0, -1.0)
    body = body.cut(keyway)
    return body


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
