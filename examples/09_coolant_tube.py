"""09 — Swept coolant tube.

A hollow coolant line that follows an S-bend path. The outer body is a circular
profile swept along the bend with a guide wire steering the section orientation;
the bore is a second, smaller sweep along the same path, cut out to leave a
constant-wall tube.

Features exercised: SWEEP (a round section carried along a 3-D S-bend spine, its
frame kept steady by the sweep's Frenet transport) and a boolean CUT of an inner
swept bore to hollow the tube.
"""

from __future__ import annotations

import math

from _gallery import run_and_report
from _shapes import circle_xy

NAME = "09_coolant_tube"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
OUTER_DIAMETER = 22.0      # tube outside diameter
WALL = 2.5                 # tube wall thickness
PROFILE_SEG = 48           # round-section discretisation
RUN = 60.0                 # length of each straight run
OFFSET = 55.0              # lateral offset the S-bend crosses

TITLE = "Swept coolant tube (S-bend)"
DESCRIPTION = (
    "A 22 mm coolant line swept along an S-bend: a round section carried by a "
    "guide-steered sweep, with an inner bore swept and cut away to leave a "
    "constant 2.5 mm wall."
)
FEATURES = ["sweep along S-bend path", "boolean cut (swept bore)"]


def _s_path():
    """A smooth S-bend path in the X-Z plane as a list of ``(x, y, z)`` points:
    up a straight run, arc over to the side, up another straight run."""
    pts = [(0.0, 0.0, 0.0)]
    # first straight rise along +Z
    n_rise = 6
    for i in range(1, n_rise + 1):
        pts.append((0.0, 0.0, RUN * i / n_rise))
    z0 = RUN
    # S transition: a smooth cosine ramp sideways over one more RUN of height
    n_arc = 16
    for i in range(1, n_arc + 1):
        t = i / n_arc
        x = OFFSET * 0.5 * (1.0 - math.cos(math.pi * t))
        z = z0 + RUN * t
        pts.append((x, 0.0, z))
    z1 = z0 + RUN
    # final straight rise offset by OFFSET
    for i in range(1, n_rise + 1):
        pts.append((OFFSET, 0.0, z1 + RUN * i / n_rise))
    return pts


def build(kernel):
    kernel.set_engine(False)  # OCCT: sweep is an OCCT-backed feature

    r_out = OUTER_DIAMETER / 2.0
    r_in = r_out - WALL

    path = _s_path()

    outer = kernel.sweep(circle_xy(r_out, segments=PROFILE_SEG), path)
    inner = kernel.sweep(circle_xy(r_in, segments=PROFILE_SEG), path)

    return outer.cut(inner)


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
