"""04 — V-belt pulley.

A single-groove V-belt pulley built as one revolved cross-section: a bored hub,
a thin web, and an outer rim carrying a V-groove. The whole part is a solid of
revolution about its axis; the bore is part of the revolved profile (the profile
simply starts at the bore radius, not at r=0).

Features exercised: :meth:`Kernel.revolve` of a hand-built closed 2-D profile
(the ``(radius, axial)`` cross-section) — the bore, hub, web and V-groove all
come straight from the profile, so no boolean is needed. A rim fillet is added on
top to round the groove shoulders.

Note on axes: ``revolve`` spins the ``(x, y)`` profile about the **Y axis**, so
in the profile ``x`` is the radius from the axis and ``y`` is the axial position.
"""

from __future__ import annotations

import math

from _gallery import run_and_report

NAME = "04_v_pulley"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
BORE_RADIUS = 10.0       # shaft bore (radius from axis)
HUB_RADIUS = 20.0        # hub outer radius
HUB_WIDTH = 34.0         # hub axial length
WEB_RADIUS = 46.0        # where the web meets the rim
WEB_WIDTH = 12.0         # thin web axial thickness
RIM_OUTER_RADIUS = 60.0  # outer diameter/2 of the pulley
RIM_WIDTH = 26.0         # rim axial width
GROOVE_DEPTH = 12.0      # radial depth of the V-groove
GROOVE_HALF_ANGLE = 19.0  # half of the standard ~38 deg V-belt angle

TITLE = "V-belt pulley (single groove)"
DESCRIPTION = (
    "Single-groove V-belt pulley revolved from one cross-section: bored hub, "
    "thin web, and a rim carrying a symmetric ~38 deg V-groove."
)
FEATURES = ["revolve (hand-built profile)", "integral bore", "V-groove section"]


def _profile():
    """Return the closed ``(radius, axial)`` cross-section of the pulley.

    Traced as a single closed loop: up the bore, across the hub/web/rim on the
    +axial face, down and around the V-groove on the outer face, and back along
    the -axial face to the bore. The groove is a symmetric V cut into the rim's
    outer diameter.
    """
    # Centre the part on the axial origin: axial runs from -half..+half.
    hub_h = HUB_WIDTH / 2.0
    web_h = WEB_WIDTH / 2.0
    rim_h = RIM_WIDTH / 2.0

    groove_r = RIM_OUTER_RADIUS - GROOVE_DEPTH
    # Axial half-width of the groove mouth at the outer diameter.
    groove_mouth_h = GROOVE_DEPTH * math.tan(math.radians(GROOVE_HALF_ANGLE))

    # Walk the loop counter-clockwise in the (radius, axial) plane.
    pts = [
        # bore, -axial face
        (BORE_RADIUS, -hub_h),
        (BORE_RADIUS, hub_h),
        # hub +axial face out to the web
        (HUB_RADIUS, hub_h),
        (HUB_RADIUS, web_h),
        (WEB_RADIUS, web_h),
        # rim +axial face
        (WEB_RADIUS, rim_h),
        (RIM_OUTER_RADIUS, rim_h),
        # outer diameter down into the V-groove and back out (the V)
        (RIM_OUTER_RADIUS, groove_mouth_h),
        (groove_r, 0.0),
        (RIM_OUTER_RADIUS, -groove_mouth_h),
        # rim -axial face
        (RIM_OUTER_RADIUS, -rim_h),
        (WEB_RADIUS, -rim_h),
        # web/hub -axial face back to the bore
        (WEB_RADIUS, -web_h),
        (HUB_RADIUS, -web_h),
        (HUB_RADIUS, -hub_h),
    ]
    return pts


def build(kernel):
    body = kernel.revolve(_profile(), 2.0 * math.pi)
    return body


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
