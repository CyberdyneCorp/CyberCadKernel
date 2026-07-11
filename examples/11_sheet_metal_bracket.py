"""11 — Sheet-metal L-bracket (folded + flat pattern).

A single-bend sheet-metal bracket: a flat base flange with one 90° edge flange
bent up off a straight rim, then its developed FLAT PATTERN produced by unfolding
the bend. Both the folded solid and the flat blank are emitted as gallery
entries; the flat-pattern plan area is (base run + bend allowance + wall) × width
and is conserved through the fold ↔ unfold.

Features exercised: sheet-metal BASE FLANGE (a blank of constant thickness), a
90° EDGE FLANGE bent off a straight rim, and UNFOLD to the developed flat
pattern.

These sheet-metal ops are provided by the kernel's *native* engine only. This
piece builds **and** consumes every body entirely under the native engine (it
never feeds a native body to an OCCT op), so it stays clear of the known
cross-engine hazard.
"""

from __future__ import annotations

from _gallery import emit

NAME_FOLDED = "11a_sheet_bracket_folded"
NAME_FLAT = "11b_sheet_bracket_flat"

# ── Parameters (mm) ───────────────────────────────────────────────────────────
BLANK_LENGTH = 60.0    # base flange run (X)
BLANK_WIDTH = 40.0     # width (Y)
THICKNESS = 2.0        # sheet thickness
FLANGE_HEIGHT = 25.0   # bent wall height
BEND_RADIUS = 3.0      # inside bend radius
BEND_ANGLE = 90.0      # fold angle (degrees)
K_FACTOR = 0.4         # neutral-fibre position for the unfold allowance

TITLE_FOLDED = "Sheet-metal L-bracket (folded)"
DESCRIPTION_FOLDED = (
    "A 2 mm sheet-metal bracket: a flat base flange with one 90° edge flange bent "
    "up off a straight rim (3 mm inside bend radius). Native sheet-metal ops."
)
FEATURES_FOLDED = ["sheet base flange", "sheet edge flange (90° bend)"]

TITLE_FLAT = "Sheet-metal L-bracket (flat pattern)"
DESCRIPTION_FLAT = (
    "The developed flat blank of the folded bracket, produced by unfolding the "
    "bend (k-factor 0.4). Plan area = (base run + bend allowance + wall) × width, "
    "conserved through fold ↔ unfold — so the flat blank is strictly larger in "
    "plan than the base flange."
)
FEATURES_FLAT = ["sheet base flange", "sheet edge flange", "sheet unfold (flat pattern)"]


def build_folded(kernel):
    """Return the folded single-bend bracket (native engine)."""
    kernel.set_engine(True)  # native: sheet-metal ops are native-only

    base = kernel.sheet_base_flange(
        [(0.0, 0.0), (BLANK_LENGTH, 0.0), (BLANK_LENGTH, BLANK_WIDTH), (0.0, BLANK_WIDTH)],
        THICKNESS,
    )

    # Bend an edge flange up off some straight rim of the base. Not every rim is
    # a valid single-bend seat, so take the first one the engine accepts (the
    # kernel's own tests use exactly this "first rim that works" strategy).
    for eid in base.edge_ids():
        try:
            return base.sheet_edge_flange(eid, FLANGE_HEIGHT, BEND_RADIUS, BEND_ANGLE)
        except Exception:
            continue
    raise RuntimeError("no straight rim accepted a 90° edge flange")


def build(kernel):
    """Gallery-convention entry point: the primary (folded) solid."""
    return build_folded(kernel)


def emit_entries(kernel):
    """Multi-entry hook for ``run_all``: emit the folded solid and its flat
    pattern, returning both :class:`PieceResult` objects as a list."""
    folded = build_folded(kernel)
    folded_mp = folded.mass_properties()
    r_folded = emit(folded, NAME_FOLDED, TITLE_FOLDED, DESCRIPTION_FOLDED, FEATURES_FOLDED)

    # Unfold to the developed flat pattern (still native, same body lineage).
    flat = folded.sheet_unfold(K_FACTOR)
    flat_mp = flat.mass_properties()
    # Record the fold->unfold volume conservation in the flat entry's caption.
    r_flat = emit(flat, NAME_FLAT, TITLE_FLAT, DESCRIPTION_FLAT, FEATURES_FLAT)
    r_flat.description = (
        DESCRIPTION_FLAT
        + f" Fold→unfold volume: {folded_mp.volume:,.0f} → {flat_mp.volume:,.0f} mm³ "
        + f"({100.0 * abs(flat_mp.volume - folded_mp.volume) / folded_mp.volume:.1f}% "
        + "bend-allowance model difference)."
    )

    kernel.set_engine(False)  # leave the shared kernel back on OCCT for later pieces
    flat.close()
    folded.close()
    return [r_folded, r_flat]


def _run():
    from cybercadkernel import Kernel

    kernel = Kernel()

    r_folded, r_flat = emit_entries(kernel)

    for r in (r_folded, r_flat):
        sz = r.bbox_size
        print(f"[{r.name}] {r.title}")
        print(
            f"    volume = {r.volume_mm3:.3f} mm^3   area = {r.area_mm2:.3f} mm^2   "
            f"bbox = {sz[0]:.1f} x {sz[1]:.1f} x {sz[2]:.1f} mm"
        )
    # The flat plan footprint is larger than the folded base footprint, and the
    # volume is ~conserved through fold ↔ unfold (see the flat entry's caption).
    print(
        f"    fold->unfold volume: folded {r_folded.volume_mm3:.1f} mm^3  "
        f"flat {r_flat.volume_mm3:.1f} mm^3  (Δ {r_flat.volume_mm3 - r_folded.volume_mm3:+.1f})"
    )


if __name__ == "__main__":
    _run()
