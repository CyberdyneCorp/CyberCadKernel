"""13 — Section + HLR engineering drawing (showpiece).

Takes a machined step-block (an L-profile prism with a rectangular notch), then
produces a real orthographic engineering drawing of it:

* a planar SECTION through the part (the cut-plane loops, with their enclosed
  area and perimeter reported in the caption), and
* three orthographic HLR views (front / top / right) plus an isometric, each an
  exact hidden-line-removal projection — visible edges solid, hidden edges
  dashed — laid out as a proper multi-view drawing sheet and rendered to PNG
  with matplotlib.

Features exercised: planar SECTION curves and orthographic HLR projection
(``hlr_project``) rendered as a 2-D drawing.

Engine split (both honoured without ever crossing a body between engines):

* HLR runs under **OCCT**, which projects the true B-rep feature/silhouette
  edges — a clean drawing. (Native HLR projects the tessellation, so the views
  come out triangulated.)
* The planar SECTION op is **native-only**, and the native boolean domain is
  planar — so the identical planar solid is rebuilt from the same profiles under
  the native engine purely to compute the section.

Because both solids are rebuilt from scratch under their own engine (never handed
across), the known cross-engine hazard is never touched.
"""

from __future__ import annotations

import os

from _gallery import DEFLECTION, OUT_ROOT, emit

NAME = "13_section_hlr_drawing"

# ── Part parameters (mm) ──────────────────────────────────────────────────────
DEPTH = 40.0           # extrusion depth (Z)
# L-profile in XY (all straight edges → planar solid, native-boolean friendly):
L_PROFILE = [(0.0, 0.0), (80.0, 0.0), (80.0, 20.0), (30.0, 20.0), (30.0, 60.0), (0.0, 60.0)]
NOTCH = [(50.0, -1.0), (70.0, -1.0), (70.0, 10.0), (50.0, 10.0)]  # rectangular notch
SECTION_Z = 20.0       # cut-plane height for the section

TITLE = "Section + HLR engineering drawing"
DESCRIPTION = (
    "A machined step-block (L-profile prism with a rectangular notch) rendered as "
    "a real orthographic drawing: a planar section (area/perimeter in the "
    "caption) plus front/top/right HLR views and an isometric — visible edges "
    "solid, hidden edges dashed. See the drawing PNG artifact."
)
FEATURES = ["planar section", "HLR projection (multi-view drawing)"]


def _make_block(kernel):
    """Build the planar step-block from the profiles (engine already selected)."""
    body = kernel.extrude(L_PROFILE, DEPTH)
    notch = kernel.extrude(NOTCH, DEPTH + 2.0).translate(0.0, 0.0, -1.0)
    return body.cut(notch)


def build(kernel):
    """The step-block under OCCT — the body used for the 3-D thumbnail and for the
    clean B-rep HLR drawing."""
    kernel.set_engine(False)  # OCCT: clean HLR feature edges (native HLR is triangulated)
    return _make_block(kernel)


def _draw_view(ax, drawing, title):
    """Plot one HLR view: visible edges solid black, hidden edges dashed grey."""
    for seg in drawing.hidden:
        ax.plot([seg[0], seg[2]], [seg[1], seg[3]],
                color="0.55", linewidth=0.7, linestyle=(0, (4, 3)), zorder=1)
    for seg in drawing.visible:
        ax.plot([seg[0], seg[2]], [seg[1], seg[3]],
                color="black", linewidth=1.4, solid_capstyle="round", zorder=2)
    ax.set_title(title, fontsize=10, fontfamily="monospace")
    ax.set_aspect("equal")
    ax.axis("off")


def _draw_section(ax, section, title):
    """Plot the section loops as filled cut faces (hatched outline)."""
    import numpy as np

    for lp in section.loops:
        pts = lp.points
        if len(pts) < 3:
            continue
        # project the 3-D loop points to the cut plane's XY (the cut is z=const)
        xy = np.asarray(pts)[:, :2]
        xy = np.vstack([xy, xy[0]])  # close the loop
        ax.fill(xy[:, 0], xy[:, 1], facecolor="0.82", edgecolor="black",
                linewidth=1.4, zorder=1)
    ax.set_title(title, fontsize=10, fontfamily="monospace")
    ax.set_aspect("equal")
    ax.axis("off")


def _render_drawing(shape, section, out_png):
    """Render a 4-panel orthographic drawing sheet to ``out_png``. Returns the
    per-view (visible, hidden) edge counts for the caption."""
    import matplotlib

    matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    # Standard third-angle-ish view directions (camera looks along −dir):
    views = {
        "FRONT (−Y)": ((0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
        "TOP (−Z)": ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0)),
        "RIGHT (+X)": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        "ISO": ((1.0, -1.0, 0.8), (0.0, 0.0, 1.0)),
    }
    drawings = {name: shape.hlr_project(vd, up) for name, (vd, up) in views.items()}
    counts = {name: (d.visible_count, d.hidden_count) for name, d in drawings.items()}

    fig, axes = plt.subplots(2, 3, figsize=(11, 7))
    fig.suptitle("STEP-BLOCK — orthographic drawing (HLR) + section",
                 fontsize=12, fontfamily="monospace")

    _draw_view(axes[0][0], drawings["FRONT (−Y)"], "FRONT (−Y)")
    _draw_view(axes[0][1], drawings["TOP (−Z)"], "TOP (−Z)")
    _draw_view(axes[0][2], drawings["RIGHT (+X)"], "RIGHT (+X)")
    _draw_view(axes[1][0], drawings["ISO"], "ISOMETRIC")
    _draw_section(axes[1][1], section, f"SECTION @ Z={SECTION_Z:g}")

    # legend / notes panel
    ax = axes[1][2]
    ax.axis("off")
    notes = (
        "visible edges: solid\n"
        "hidden edges:  dashed\n\n"
        f"section area   = {section.total_area:,.1f} mm²\n"
        f"section perim. = {section.total_length:,.1f} mm\n"
        f"section loops  = {section.loop_count}"
    )
    ax.text(0.02, 0.95, notes, va="top", ha="left", fontsize=9,
            fontfamily="monospace", transform=ax.transAxes)

    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out_png, dpi=130)
    plt.close(fig)
    return counts


def emit_entries(kernel):
    """Multi-output hook for ``run_all``: emit the solid gallery entry, render the
    section + HLR drawing sheet, and record the drawing PNG as an extra artifact
    on the result. Returns ``[PieceResult]``."""
    # OCCT solid: 3-D thumbnail + clean B-rep HLR drawing.
    shape = build(kernel)

    # Emit the solid as a normal gallery entry (3-D thumbnail + STEP/STL/glTF).
    result = emit(shape, NAME, TITLE, DESCRIPTION, FEATURES)

    # Section through the part — the section op is native-only, so rebuild the
    # identical planar solid under the native engine (nothing crosses engines).
    kernel.set_engine(True)
    native_block = _make_block(kernel)
    section = native_block.section_plane((0.0, 0.0, SECTION_Z), (0.0, 0.0, 1.0))
    kernel.set_engine(False)  # restore OCCT for the HLR projection below

    # Render the engineering-drawing sheet alongside it and record it as an extra
    # artifact so the gallery can show the actual drawing.
    drawing_png = os.path.join(OUT_ROOT, NAME, f"{NAME}_drawing.png")
    counts = _render_drawing(shape, section, drawing_png)
    if os.path.getsize(drawing_png) > 0:
        result.artifacts["drawing"] = os.path.relpath(drawing_png, os.path.dirname(OUT_ROOT))

    # Fold the section metrics into the description so the README caption carries
    # them (the gallery schema has no dedicated section field).
    result.description = (
        DESCRIPTION
        + f" Section @ Z={SECTION_Z:g}: {section.loop_count} loop, "
        + f"area {section.total_area:,.0f} mm², perimeter {section.total_length:,.0f} mm. "
        + "HLR edge counts (visible/hidden): "
        + ", ".join(f"{n.split()[0].lower()} {v}/{h}" for n, (v, h) in counts.items())
        + "."
    )

    # Stash the drawing metrics for standalone printing.
    result._section = section  # type: ignore[attr-defined]
    result._hlr_counts = counts  # type: ignore[attr-defined]

    native_block.close()
    shape.close()
    return [result]


def _run():
    from cybercadkernel import Kernel

    kernel = Kernel()
    (result,) = emit_entries(kernel)
    section = result._section  # type: ignore[attr-defined]
    counts = result._hlr_counts  # type: ignore[attr-defined]

    print(f"[{NAME}] {TITLE}")
    print(
        f"    section: loops={section.loop_count}  "
        f"area={section.total_area:.1f} mm^2  perimeter={section.total_length:.1f} mm"
    )
    for name, (vis, hid) in counts.items():
        print(f"    HLR {name:12s}: visible={vis:3d}  hidden={hid:3d}")
    sz = result.bbox_size
    print(f"    solid volume = {result.volume_mm3:.1f} mm^3   bbox = {sz[0]:.1f} x {sz[1]:.1f} x {sz[2]:.1f} mm")
    return result


if __name__ == "__main__":
    _run()
