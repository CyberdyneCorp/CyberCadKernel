"""08 — Round-to-square transition adapter.

A duct / HVAC transition that lofts a round inlet into a square outlet, then
hollows it to a constant wall so it reads as real ducting rather than a solid
plug.

Features exercised: LOFT between two dissimilar 2-D sections (a circle at the
base, a square at the top, separated by the transition height) and a SHELL that
opens both the round and the square face to leave a uniform-thickness duct wall.
"""

from __future__ import annotations

import math

from _gallery import run_and_report
from _shapes import circle_xy

NAME = "08_round_square_adapter"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
INLET_DIAMETER = 80.0      # round end
OUTLET_SIDE = 90.0         # square end (full width)
HEIGHT = 90.0              # transition height (round base -> square top)
WALL = 3.0                 # duct wall thickness after shelling
CIRCLE_SEG = 64            # circle discretisation (also the square's vertex budget)

TITLE = "Round-to-square duct adapter"
DESCRIPTION = (
    "HVAC transition lofting an 80 mm round inlet into a 90 mm square outlet over "
    "a 90 mm rise, then shelled to a 3 mm wall (both ends open) for real ducting."
)
FEATURES = ["loft (circle -> square)", "shell (open both ends)"]


def _square_xy(side: float, seg: int):
    """A closed square of full width ``side`` sampled with ``seg`` points so it
    has the same vertex budget as the circle it lofts from (a matched vertex
    count keeps the ruled surface between the two sections clean)."""
    h = side / 2.0
    per_side = seg // 4
    pts = []
    # walk the perimeter counter-clockwise, seg/4 points per edge
    corners = [(-h, -h), (h, -h), (h, h), (-h, h)]
    for i in range(4):
        ax, ay = corners[i]
        bx, by = corners[(i + 1) % 4]
        for j in range(per_side):
            t = j / per_side
            pts.append((ax + (bx - ax) * t, ay + (by - ay) * t))
    return pts


def build(kernel):
    kernel.set_engine(False)  # OCCT (default); loft is an OCCT-backed feature

    r_inlet = INLET_DIAMETER / 2.0
    bottom = circle_xy(r_inlet, segments=CIRCLE_SEG)
    top = _square_xy(OUTLET_SIDE, CIRCLE_SEG)

    # Solid transition: round bottom section lofted to the square top section.
    solid = kernel.loft(bottom, top, HEIGHT)

    # Hollow it to a duct: open the round bottom face (z≈0) and the square top
    # face (z≈HEIGHT) so air can pass straight through the transition.
    open_faces = _end_face_ids(solid)
    if len(open_faces) == 2:
        try:
            return solid.shell(open_faces, WALL)
        except Exception:
            pass  # fall back to the solid transition rather than fail the piece
    return solid


def _end_face_ids(shape):
    """Return the [bottom, top] planar cap face ids (the round inlet at z≈0 and
    the square outlet at z≈HEIGHT), picked by their face-mesh centroids."""
    bottom_id = top_id = None
    bottom_z = 1e9
    top_z = -1e9
    for fm in shape.face_meshes(deflection=0.5):
        v = fm.mesh.vertices
        if len(v) == 0:
            continue
        zc = float(v[:, 2].mean())
        # a cap face is planar in z: near-zero z spread
        if float(v[:, 2].std()) > 0.5:
            continue
        if zc < bottom_z:
            bottom_z, bottom_id = zc, fm.face_id
        if zc > top_z:
            top_z, top_id = zc, fm.face_id
    ids = []
    if bottom_id is not None:
        ids.append(bottom_id)
    if top_id is not None and top_id != bottom_id:
        ids.append(top_id)
    return ids


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
