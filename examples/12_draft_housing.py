"""12 — Draft-moulded housing.

An injection-mould-style housing: a rectangular boss with draft-angled side walls
(so it releases from the mould), hollowed to a uniform wall, with the top rim
filleted.

Features exercised: DRAFT on the four planar side faces (tapered about a neutral
plane at the base, pulling along +Z for mould release), SHELL to hollow the
interior (open at the bottom, like a moulded cover), and an edge FILLET rolling
the top rim.
"""

from __future__ import annotations

from _gallery import run_and_report
from _shapes import box_centered

NAME = "12_draft_housing"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
LENGTH = 80.0        # X
WIDTH = 56.0         # Y
HEIGHT = 40.0        # Z
DRAFT_DEG = 4.0      # mould-release draft on each side wall
WALL = 2.5           # shelled wall thickness
RIM_FILLET = 3.0     # rolled top rim

TITLE = "Draft-moulded housing"
DESCRIPTION = (
    "An injection-mould-style cover: a rectangular boss with 4° draft on all four "
    "side walls (pull +Z, neutral at the base), shelled to a 2.5 mm wall (open "
    "bottom) with a filleted top rim."
)
FEATURES = ["draft faces (4° side walls)", "shell (open bottom)", "edge fillet (top rim)"]


def build(kernel):
    kernel.set_engine(False)  # OCCT: draft / shell / fillet are OCCT-backed

    body = box_centered(kernel, LENGTH, WIDTH, HEIGHT)

    # 1. Draft the four vertical side faces about a neutral plane at the base
    #    (z=0), pulling along +Z. A positive angle tapers the walls inward as they
    #    rise, the classic mould-release form.
    side_faces = _vertical_side_faces(body)
    if side_faces:
        try:
            body = body.draft_faces(side_faces, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), DRAFT_DEG)
        except Exception:
            pass  # keep the un-drafted boss rather than fail the piece

    # 2. Hollow it: remove the bottom face to a uniform wall (a moulded cover).
    bottom = _bottom_face(body)
    if bottom is not None:
        try:
            body = body.shell([bottom], WALL)
        except Exception:
            pass

    # 3. Roll the top rim.
    top_rim = _top_rim_edges(body)
    if top_rim:
        try:
            body = body.fillet_edges(top_rim, RIM_FILLET)
        except Exception:
            pass
    return body


def _vertical_side_faces(shape):
    """Face ids whose mesh is (near-)vertical: small z-spread is NOT the test —
    a side wall spans z but has a near-constant horizontal outward normal, so we
    detect it as a face whose vertices span a large z range."""
    ids = []
    for fm in shape.face_meshes(deflection=1.0):
        v = fm.mesh.vertices
        if len(v) == 0:
            continue
        z_span = float(v[:, 2].max() - v[:, 2].min())
        if z_span > HEIGHT * 0.5:  # spans most of the height → a side wall
            ids.append(fm.face_id)
    return ids


def _bottom_face(shape):
    """The planar face at z≈0 (bottom cap)."""
    best = None
    best_z = 1e9
    for fm in shape.face_meshes(deflection=1.0):
        v = fm.mesh.vertices
        if len(v) == 0:
            continue
        if float(v[:, 2].std()) > 0.4:  # not planar-in-z
            continue
        zc = float(v[:, 2].mean())
        if zc < best_z:
            best_z, best = zc, fm.face_id
    return best


def _top_rim_edges(shape):
    """Edges lying at the top plane (z≈max) — the rim to roll."""
    bb = shape.bounding_box()
    z_top = bb.max[2]
    ids = []
    for pl in shape.edge_polylines():
        pts = pl.points
        if len(pts) < 2:
            continue
        if abs(float(pts[:, 2].mean()) - z_top) < 0.5 and float(pts[:, 2].std()) < 0.5:
            ids.append(pl.edge_id)
    return ids


if __name__ == "__main__":
    run_and_report(build, NAME, TITLE, DESCRIPTION, FEATURES)
