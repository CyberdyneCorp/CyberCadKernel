"""02 — L-bracket.

An L-shaped mounting bracket: two plates fused at a right angle, stiffened by a
triangular gusset, with mounting holes through each flange, the inner corner
filleted and the outer vertical edges chamfered.

Features exercised: boolean FUSE (plates + gusset), boolean CUT (mounting holes),
edge fillet (inner corner) and edge chamfer (outer corners).
"""

from __future__ import annotations

from _gallery import run_and_report
from _shapes import box, cylinder_z_at

NAME = "02_l_bracket"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
WIDTH = 60.0          # bracket depth along Y
LEG = 70.0            # length of each leg (along X for the base, Z for the wall)
THICK = 8.0           # plate thickness
HOLE_DIA = 9.0        # M8 clearance
HOLE_INSET = 20.0     # hole centre distance from the outer end / top
GUSSET = 34.0         # size of the triangular stiffener legs
INNER_FILLET = 6.0
OUTER_CHAMFER = 4.0

TITLE = "L-bracket with gusset"
DESCRIPTION = (
    "Right-angle mounting bracket: two plates fused with a triangular gusset, a "
    "mounting hole per flange, filleted inner corner, chamfered outer edges."
)
FEATURES = ["boolean fuse", "boolean cut", "edge fillet", "edge chamfer"]


def build(kernel):
    r_hole = HOLE_DIA / 2.0

    # Horizontal base plate: lies on Z=0, extends along +X.
    base = box(kernel, LEG, WIDTH, THICK)
    # Vertical wall plate: stands up along +Z at the X origin corner.
    wall = box(kernel, THICK, WIDTH, LEG)
    body = base.fuse(wall)

    # Triangular gusset in the X-Z plane, extruded across the full width Y.
    # Profile is a right triangle rising from the inner corner.
    tri = [
        (THICK, 0.0),
        (THICK + GUSSET, 0.0),
        (THICK, GUSSET),
    ]
    gusset = kernel.extrude(tri, WIDTH)
    # extrude runs the triangle along +Z; rotate +90 deg about +X so the
    # triangle stands in the X-Z plane, then seat it across the full width and
    # on top of the base plate.
    import math

    gusset = gusset.rotate((0, 0, 0), (1, 0, 0), math.pi / 2).translate(0, WIDTH, THICK)
    body = body.fuse(gusset)

    # Mounting hole through the base plate (drilled along +Z).
    base_hole = cylinder_z_at(
        kernel, LEG - HOLE_INSET, WIDTH / 2.0, r_hole, THICK + 2.0
    ).translate(0, 0, -1.0)
    body = body.cut(base_hole)

    # Mounting hole through the vertical wall (drilled along +X).
    wall_hole = cylinder_z_at(
        kernel, 0.0, 0.0, r_hole, THICK + 2.0
    )  # cylinder about +Z, base at Z=0
    wall_hole = (
        wall_hole.rotate((0, 0, 0), (0, 1, 0), math.pi / 2)  # +Z axis -> +X axis
        .translate(-1.0, WIDTH / 2.0, LEG - HOLE_INSET)
    )
    body = body.cut(wall_hole)

    body = _fillet_inner_corner(body)
    body = _chamfer_outer_vertical_edges(body)
    return body


def _fillet_inner_corner(shape):
    """Fillet the concave inner corner edge (runs along Y at x≈THICK, z≈THICK)."""
    for pl in shape.edge_polylines():
        pts = pl.points
        if len(pts) < 2:
            continue
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        spans_y = (y.max() - y.min()) > WIDTH * 0.6
        at_corner = abs(x.mean() - THICK) < 0.6 and abs(z.mean() - THICK) < 0.6
        if spans_y and at_corner:
            try:
                return shape.fillet_edges([pl.edge_id], INNER_FILLET)
            except Exception:
                return shape
    return shape


def _chamfer_outer_vertical_edges(shape):
    """Chamfer the two tall outer vertical edges of the wall (x≈0)."""
    ids = []
    for pl in shape.edge_polylines():
        pts = pl.points
        if len(pts) < 2:
            continue
        x, z = pts[:, 0], pts[:, 2]
        vertical = (z.max() - z.min()) > LEG * 0.5
        at_back = abs(x.mean()) < 0.6
        if vertical and at_back:
            ids.append(pl.edge_id)
    if ids:
        try:
            return shape.chamfer_edges(ids, OUTER_CHAMFER)
        except Exception:
            return shape
    return shape


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
