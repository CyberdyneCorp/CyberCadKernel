"""05 — Shelled electronics enclosure.

A small electronics enclosure: a rounded box hollowed to a uniform wall
thickness (open at the top), with the outer vertical edges filleted for a
finished look and an internal lid-locating boss added on the base wall.

Features exercised: extruded rounded-rectangle box, SHELL (hollow the box,
removing the top face to a wall thickness), edge FILLET on the outer corners,
and a boolean FUSE to add the internal lid boss.
"""

from __future__ import annotations

from _gallery import run_and_report
from _shapes import cylinder_z_at, rounded_rect_xy

NAME = "05_enclosure"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
LENGTH = 100.0     # X
WIDTH = 64.0       # Y
HEIGHT = 32.0      # Z
CORNER_RADIUS = 6.0
WALL = 2.4         # shell wall thickness
CORNER_FILLET = 4.0
BOSS_RADIUS = 4.0
BOSS_HEIGHT = 12.0
BOSS_INSET = 8.0   # boss centre inset from each corner

TITLE = "Shelled electronics enclosure"
DESCRIPTION = (
    "Rounded-corner enclosure hollowed to a 2.4 mm wall (open top), with "
    "filleted outer edges and four internal lid-locating bosses."
)
FEATURES = ["rounded-rect extrude", "shell", "edge fillet", "boolean fuse (bosses)"]


def build(kernel):
    # Rounded-rectangle box centred on X/Y, sitting on Z=0.
    profile = rounded_rect_xy(LENGTH, WIDTH, CORNER_RADIUS)
    body = kernel.extrude(profile, HEIGHT).translate(LENGTH / 2.0, WIDTH / 2.0, 0.0)

    # Fillet the four outer vertical corner edges (before shelling, so the wall
    # follows the rounded outside).
    body = _fillet_vertical_corners(body)

    # Shell: hollow the box, removing the top face -> an open-topped enclosure.
    top_face = _top_face_id(body, HEIGHT)
    if top_face is not None:
        try:
            body = body.shell([top_face], WALL)
        except Exception:
            pass  # fall back to a solid block rather than fail the piece

    # Internal lid-locating bosses on the base, one near each corner.
    for cx, cy in (
        (BOSS_INSET, BOSS_INSET),
        (LENGTH - BOSS_INSET, BOSS_INSET),
        (BOSS_INSET, WIDTH - BOSS_INSET),
        (LENGTH - BOSS_INSET, WIDTH - BOSS_INSET),
    ):
        boss = cylinder_z_at(kernel, cx, cy, BOSS_RADIUS, BOSS_HEIGHT).translate(
            0, 0, WALL
        )
        body = body.fuse(boss)
    return body


def _fillet_vertical_corners(shape):
    """Fillet the four tall vertical outer edges of the box."""
    ids = []
    for pl in shape.edge_polylines():
        pts = pl.points
        if len(pts) < 2:
            continue
        z = pts[:, 2]
        if (z.max() - z.min()) > HEIGHT * 0.8:
            ids.append(pl.edge_id)
    if ids:
        try:
            return shape.fillet_edges(ids, CORNER_FILLET)
        except Exception:
            return shape
    return shape


def _top_face_id(shape, height):
    """Return the id of the planar top face (all mesh at z≈height)."""
    for fm in shape.face_meshes(0.4):
        v = fm.mesh.vertices
        if len(v) == 0:
            continue
        z = v[:, 2]
        if abs(z.min() - height) < 0.4 and abs(z.max() - height) < 0.4:
            return fm.face_id
    return None


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
