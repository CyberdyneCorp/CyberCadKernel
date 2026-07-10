"""03 — Pillow-block bearing housing.

A pillow-block bearing housing: a base plate with two mounting slots carries a
raised boss bored out to a cylindrical bearing seat. The bore is hollowed with a
shell so the seat has a finite back wall, and the top of the boss is filleted.

Features exercised: extruded box + cylinder primitives, boolean FUSE (base +
boss), boolean CUT (bearing seat + mounting slots), shell (hollow the seat), and
an edge fillet on the boss.
"""

from __future__ import annotations

from _gallery import run_and_report
from _shapes import box_centered, cylinder_z_at

NAME = "03_bearing_block"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
BASE_LEN = 120.0       # along X
BASE_WIDTH = 44.0      # along Y
BASE_THICK = 12.0
BOSS_DIAMETER = 62.0
BOSS_HEIGHT = 46.0     # top of boss above Z=0
SEAT_DIAMETER = 40.0   # bearing outer-race seat
SEAT_BACK_WALL = 4.0   # material left behind the seat (via shell)
SLOT_DIAMETER = 12.0   # mounting slot width
SLOT_CENTRE_X = 44.0   # +/- from centre
BOSS_TOP_FILLET = 3.0

TITLE = "Pillow-block bearing housing"
DESCRIPTION = (
    "Bearing housing: a slotted base plate carrying a bored cylindrical boss "
    "(bearing seat with a back wall) and a filleted boss rim."
)
FEATURES = ["box + cylinder", "boolean fuse/cut", "bored seat", "edge fillet"]


def build(kernel):
    r_boss = BOSS_DIAMETER / 2.0
    r_seat = SEAT_DIAMETER / 2.0
    r_slot = SLOT_DIAMETER / 2.0

    # Base plate, centred on X/Y, sitting on Z=0.
    base = box_centered(kernel, BASE_LEN, BASE_WIDTH, BASE_THICK)

    # Cylindrical boss rising from the plate.
    boss = cylinder_z_at(kernel, 0.0, 0.0, r_boss, BOSS_HEIGHT)
    body = base.fuse(boss)

    # Bore the bearing seat from the top, leaving a back wall via the shell step.
    seat = cylinder_z_at(kernel, 0.0, 0.0, r_seat, BOSS_HEIGHT + 2.0).translate(
        0, 0, SEAT_BACK_WALL
    )
    body = body.cut(seat)

    # Two mounting slots through the base (round holes at each end).
    for sx in (-SLOT_CENTRE_X, SLOT_CENTRE_X):
        slot = cylinder_z_at(kernel, sx, 0.0, r_slot, BASE_THICK + 2.0).translate(
            0, 0, -1.0
        )
        body = body.cut(slot)

    body = _fillet_boss_top(body, r_boss, BOSS_HEIGHT)
    return body


def _fillet_boss_top(shape, r_boss, boss_height):
    """Fillet the outer top rim of the boss (circle radius r_boss at z=height)."""
    for pl in shape.edge_polylines():
        pts = pl.points
        if len(pts) < 8:
            continue
        z = pts[:, 2]
        radial = (pts[:, 0] ** 2 + pts[:, 1] ** 2) ** 0.5
        if abs(radial.mean() - r_boss) < 1.5 and abs(z.mean() - boss_height) < 0.5:
            try:
                return shape.fillet_edges([pl.edge_id], BOSS_TOP_FILLET)
            except Exception:
                return shape
    return shape


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
