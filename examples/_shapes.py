"""Small parametric primitive helpers on top of the bound ``extrude`` / ``revolve``.

The pythonic :class:`cybercadkernel.Kernel` exposes only two solid factories
today — :meth:`~cybercadkernel.Kernel.extrude` (a closed 2-D ``(x, y)`` profile
along +Z) and :meth:`~cybercadkernel.Kernel.revolve` (a closed profile about the
Y axis). Everything mechanical in this gallery is built from those two plus
booleans / fillets / chamfers / shells / transforms.

These helpers wrap the common primitives (box, cylinder, disc, regular n-gon
prism) so each example script stays about the *part*, not the profile algebra.
All dimensions are millimetres.
"""

from __future__ import annotations

import math
from typing import Sequence


def box(kernel, dx: float, dy: float, dz: float):
    """Axis-aligned box with its min corner at the origin (extruded rectangle)."""
    rect = [(0.0, 0.0), (dx, 0.0), (dx, dy), (0.0, dy)]
    return kernel.extrude(rect, dz)


def box_centered(kernel, dx: float, dy: float, dz: float):
    """Box centred on X/Y, sitting on the Z=0 plane."""
    return box(kernel, dx, dy, dz).translate(-dx / 2.0, -dy / 2.0, 0.0)


def circle_xy(radius: float, cx: float = 0.0, cy: float = 0.0, segments: int = 96):
    """A closed circular polygon in the XY plane as a list of ``(x, y)`` points."""
    pts = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        pts.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
    return pts


def cylinder_z(kernel, radius: float, height: float, segments: int = 96):
    """Solid cylinder about the +Z axis, base on Z=0 (extruded circle)."""
    return kernel.extrude(circle_xy(radius, segments=segments), height)


def cylinder_z_at(
    kernel, cx: float, cy: float, radius: float, height: float, segments: int = 96
):
    """Solid cylinder about +Z centred at ``(cx, cy)``, base on Z=0."""
    return kernel.extrude(circle_xy(radius, cx=cx, cy=cy, segments=segments), height)


def disc(kernel, radius: float, thickness: float, segments: int = 96):
    """Alias for a short cylinder — a flat disc about +Z."""
    return cylinder_z(kernel, radius, thickness, segments=segments)


def regular_prism(kernel, radius: float, height: float, sides: int, rotate_deg: float = 0.0):
    """Extruded regular ``sides``-gon (circumradius ``radius``) about +Z."""
    off = math.radians(rotate_deg)
    pts = []
    for i in range(sides):
        a = off + 2.0 * math.pi * i / sides
        pts.append((radius * math.cos(a), radius * math.sin(a)))
    return kernel.extrude(pts, height)


def bolt_circle(radius: float, count: int, start_deg: float = 0.0):
    """Yield ``(x, y)`` centres of ``count`` points on a circle of ``radius``."""
    off = math.radians(start_deg)
    for i in range(count):
        a = off + 2.0 * math.pi * i / count
        yield (radius * math.cos(a), radius * math.sin(a))


def rounded_rect_xy(
    dx: float, dy: float, r: float, cx: float = 0.0, cy: float = 0.0, seg_per_corner: int = 8
):
    """Closed rounded-rectangle polygon centred at ``(cx, cy)`` as ``(x, y)`` pts.

    ``dx`` / ``dy`` are the full outer extents; ``r`` is the corner radius
    (clamped to half the smaller side).
    """
    r = min(r, dx / 2.0, dy / 2.0)
    hx, hy = dx / 2.0, dy / 2.0
    # Corner arc centres, counter-clockwise from +X/+Y (top-right) corner.
    corners = [
        (hx - r, hy - r, 0.0),          # top-right,   sweeps 0   -> 90
        (-hx + r, hy - r, math.pi / 2), # top-left,    sweeps 90  -> 180
        (-hx + r, -hy + r, math.pi),    # bottom-left, sweeps 180 -> 270
        (hx - r, -hy + r, 3 * math.pi / 2),  # bottom-right
    ]
    pts = []
    for ccx, ccy, a0 in corners:
        for j in range(seg_per_corner + 1):
            a = a0 + (math.pi / 2) * j / seg_per_corner
            pts.append((cx + ccx + r * math.cos(a), cy + ccy + r * math.sin(a)))
    return pts
