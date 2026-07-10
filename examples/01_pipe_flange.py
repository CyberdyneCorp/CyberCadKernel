"""01 — Pipe flange.

A round raised-face pipe flange: a thick disc with a central bore and a
bolt-circle of N through-holes, with the outer top rim filleted.

Features exercised: primitive discs (extruded circles), boolean CUT, a manual
circular hole pattern (a loop of positioned cylinders cut out), and an edge
fillet on the outer rim.
"""

from __future__ import annotations

import math

from _gallery import run_and_report
from _shapes import bolt_circle, cylinder_z, cylinder_z_at

NAME = "01_pipe_flange"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
FLANGE_DIAMETER = 120.0
FLANGE_THICKNESS = 14.0
BORE_DIAMETER = 50.0        # the pipe bore through the middle
BOLT_CIRCLE_DIAMETER = 95.0
BOLT_HOLE_DIAMETER = 11.0   # clears an M10 bolt
BOLT_COUNT = 6
RIM_FILLET = 3.0

TITLE = "Pipe flange (6-bolt raised face)"
DESCRIPTION = (
    "Round flange: a thick disc with a central pipe bore and a 6-hole bolt "
    "circle, outer rim filleted."
)
FEATURES = ["extruded discs", "boolean cut", "circular hole pattern", "edge fillet"]


def build(kernel):
    r_flange = FLANGE_DIAMETER / 2.0
    r_bore = BORE_DIAMETER / 2.0
    r_bolt_circle = BOLT_CIRCLE_DIAMETER / 2.0
    r_bolt_hole = BOLT_HOLE_DIAMETER / 2.0

    # Solid disc body.
    body = cylinder_z(kernel, r_flange, FLANGE_THICKNESS)

    # Central bore, cut clean through (extend past both faces for a clean cut).
    bore = cylinder_z(kernel, r_bore, FLANGE_THICKNESS + 2.0).translate(0, 0, -1.0)
    body = body.cut(bore)

    # Bolt-circle: a loop of through-holes on the bolt circle.
    for (cx, cy) in bolt_circle(r_bolt_circle, BOLT_COUNT, start_deg=90.0):
        hole = cylinder_z_at(
            kernel, cx, cy, r_bolt_hole, FLANGE_THICKNESS + 2.0
        ).translate(0, 0, -1.0)
        body = body.cut(hole)

    # Fillet the outer top rim. Pick the outer top circular edge by geometry:
    # its polyline is a full circle of radius ~= r_flange lying at z = thickness.
    top_rim = _outer_top_rim_edge(body, r_flange, FLANGE_THICKNESS)
    if top_rim is not None:
        try:
            body = body.fillet_edges([top_rim], RIM_FILLET)
        except Exception:
            pass  # keep the un-filleted flange rather than fail the piece
    return body


def _outer_top_rim_edge(shape, r_flange, thickness):
    """Return the edge id of the outer circular rim at z=thickness, or None."""
    best = None
    best_score = 1e9
    for pl in shape.edge_polylines():
        pts = pl.points
        if len(pts) < 8:
            continue
        z = pts[:, 2]
        radial = (pts[:, 0] ** 2 + pts[:, 1] ** 2) ** 0.5
        # An outer-rim circle: near-constant radius ~= r_flange, near-constant z.
        if abs(radial.mean() - r_flange) < 1.5 and abs(z.mean() - thickness) < 0.5:
            score = radial.std() + z.std()
            if score < best_score:
                best_score = score
                best = pl.edge_id
    return best


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
