"""10 — Threaded hex bolt.

A hex-head bolt: a regular hexagon head extruded to a flat, a cylindrical shank,
fused into one blank, then a real helical thread welded onto the shank with the
robust segmented thread-boolean.

Features exercised: EXTRUDE (the hex head as a regular-6-gon prism), a boolean
FUSE joining the head onto the shank blank, a helical THREAD solid, and a robust
per-turn thread BOOLEAN (``thread_apply`` FUSE) welding the thread ridge onto the
blank.

Two engine facts drive the construction (see the report notes):

* ``helical_thread`` sizes the thread by ``majorRadius × pointsPerMM`` — the
  size arguments set *proportions*, not literal millimetres — so the crest
  radius is *measured* from the thread's bounding box and the blank is sized to
  it.
* Booleans onto a body that has already been through ``thread_apply`` are
  unreliable, and the per-turn thread boolean has a wall-clock budget. So the
  head is fused to the shank *first* (the whole blank kept just inside the thread
  crest so the fine-thread gate accepts it) and the thread is applied *last*,
  with a modest turn count that completes inside the budget.
"""

from __future__ import annotations

import math

from _gallery import run_and_report
from _shapes import circle_xy, regular_prism

NAME = "10_threaded_bolt"

# ── Thread parameters (proportional; crest radius is measured, see module doc) ─
THREAD_MAJOR = 3.0         # thread size arg (crest ≈ major × points_per_mm)
THREAD_PITCH = 2.5         # axial advance per turn
THREAD_TURNS = 2.0         # turns (kept modest so thread_apply finishes in budget)
THREAD_DEPTH = 0.5         # V-groove depth
THREAD_FLANK_DEG = 60.0    # ISO-style 60° flank
THREAD_PPMM = 6.0          # helix sampling density (also the geometric scale)
THREAD_SPT = 20            # samples per turn

SHANK_CLEARANCE = 0.5      # shank radius = crest − this (so the ridge welds on)
HEAD_CLEARANCE = 0.6       # hex head circumradius = crest − this (gate-safe)
HEAD_HEIGHT_FACTOR = 1.0   # head height vs crest radius

TITLE = "Threaded hex bolt"
DESCRIPTION = (
    "A regular-hexagon head fused onto a cylindrical shank, then a real helical "
    "thread welded onto the blank by the robust per-turn thread-boolean "
    "(thread_apply FUSE) — one valid B-rep solid."
)
FEATURES = ["extrude (hex head)", "boolean fuse", "helical thread", "thread_apply fuse"]


def build(kernel):
    kernel.set_engine(False)  # OCCT: thread + booleans are OCCT-backed

    from cybercadkernel import BooleanOp

    # 1. Build the thread solid; measure its true crest radius and z-extent (the
    #    size args are proportional, so we read the real geometry back).
    thread = kernel.helical_thread(
        THREAD_MAJOR, THREAD_PITCH, THREAD_TURNS, THREAD_DEPTH,
        THREAD_FLANK_DEG, THREAD_PPMM, THREAD_SPT,
    )
    tb = thread.bounding_box()
    crest_r = max(abs(tb.max[0]), abs(tb.min[0]), abs(tb.max[1]), abs(tb.min[1]))
    thread_z0 = tb.min[2]
    thread_len = tb.size[2]

    shank_r = crest_r - SHANK_CLEARANCE
    head_r = crest_r - HEAD_CLEARANCE
    head_h = crest_r * HEAD_HEIGHT_FACTOR

    # 2. Blank = cylindrical shank + hex head, fused before threading. The blank
    #    stays inside the thread crest so the fine-thread gate accepts it.
    shank = kernel.extrude(circle_xy(shank_r, segments=96), thread_len).translate(
        0.0, 0.0, thread_z0
    )
    head = regular_prism(kernel, head_r, head_h, sides=6).translate(
        0.0, 0.0, thread_z0 + thread_len
    )
    blank = shank.fuse(head)

    # 3. Weld the thread ridge onto the blank (external thread → FUSE).
    return blank.thread_apply(thread, BooleanOp.FUSE)


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
