"""07 — Hydraulic manifold block.

A hydraulic manifold: a rectangular block with three cross-drilled bores that
intersect internally (one long gallery along X plus two ports down from the top),
four mounting holes at the corners, and the top edges chamfered.

Features exercised: box primitive, boolean CUT of intersecting cross-drillings
(the internal galleries meet), a corner hole pattern (boolean cut), and edge
chamfer on the top rim.
"""

from __future__ import annotations

import math

from _gallery import run_and_report
from _shapes import box, cylinder_z_at

NAME = "07_manifold_block"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
LENGTH = 90.0     # X
WIDTH = 40.0      # Y
HEIGHT = 40.0     # Z
GALLERY_RADIUS = 6.0     # main horizontal gallery (along X)
PORT_RADIUS = 5.0        # vertical ports down from the top
PORT_X = (25.0, 65.0)    # X positions of the two vertical ports
MOUNT_HOLE_RADIUS = 3.5
MOUNT_INSET = 8.0
TOP_CHAMFER = 3.0

TITLE = "Hydraulic manifold block"
DESCRIPTION = (
    "Manifold block with a through horizontal gallery, two vertical ports that "
    "intersect it internally, four corner mounting holes and a chamfered top rim."
)
FEATURES = [
    "box primitive",
    "intersecting cross-drillings",
    "corner hole pattern",
    "edge chamfer",
]


def build(kernel):
    body = box(kernel, LENGTH, WIDTH, HEIGHT)

    # Main horizontal gallery along X, at mid-height, drilled fully through.
    # Build a +Z cylinder then lay it along +X.
    gallery = cylinder_z_at(kernel, 0.0, 0.0, GALLERY_RADIUS, LENGTH + 2.0)
    gallery = gallery.rotate((0, 0, 0), (0, 1, 0), math.pi / 2)  # +Z axis -> +X
    gallery = gallery.translate(-1.0, WIDTH / 2.0, HEIGHT / 2.0)
    body = body.cut(gallery)

    # Two vertical ports down from the top, intersecting the gallery.
    for px in PORT_X:
        port = cylinder_z_at(kernel, px, WIDTH / 2.0, PORT_RADIUS, HEIGHT + 2.0)
        port = port.translate(0, 0, -1.0)
        body = body.cut(port)

    # Four corner mounting holes through the block.
    for cx in (MOUNT_INSET, LENGTH - MOUNT_INSET):
        for cy in (MOUNT_INSET, WIDTH - MOUNT_INSET):
            hole = cylinder_z_at(
                kernel, cx, cy, MOUNT_HOLE_RADIUS, HEIGHT + 2.0
            ).translate(0, 0, -1.0)
            body = body.cut(hole)

    body = _chamfer_top_rim(body)
    return body


def _chamfer_top_rim(shape):
    """Chamfer the outer top rectangular rim (edges lying at z≈HEIGHT)."""
    ids = []
    for pl in shape.edge_polylines():
        pts = pl.points
        if len(pts) < 2:
            continue
        z = pts[:, 2]
        x = pts[:, 0]
        y = pts[:, 1]
        on_top = abs(z.mean() - HEIGHT) < 0.4 and (z.max() - z.min()) < 0.4
        # Only the outer boundary edges (touch a block side), not the port rims.
        on_boundary = (
            x.min() < 0.4 or x.max() > LENGTH - 0.4
            or y.min() < 0.4 or y.max() > WIDTH - 0.4
        )
        length = ((x.max() - x.min()) ** 2 + (y.max() - y.min()) ** 2) ** 0.5
        if on_top and on_boundary and length > 10.0:
            ids.append(pl.edge_id)
    if ids:
        try:
            return shape.chamfer_edges(ids, TOP_CHAMFER)
        except Exception:
            return shape
    return shape


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
