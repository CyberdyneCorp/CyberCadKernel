# Proposal — moat-m2cc-corner-chamfer-weld (MOAT M2 convex-corner chamfer weld)

## Why

The byte-frozen `chamfer_edges` (`src/native/blend/chamfer_edges.h`) chamfers picked
edges SEQUENTIALLY: each edge's chamfer plane clips the whole polygon soup, then the
NEXT edge is looked up in the ALREADY-CLIPPED soup. This works for NON-adjacent edges
(an opposite pair) but DECLINES a set of mutually-ADJACENT convex edges that share a
CORNER — the first cut removes the shared corner vertex, so the next edge is no longer
present in the soup (`facesOnEdgeInSoup == 0`) and the whole op returns NULL → OCCT.
(Measured: a 10³ box, edges `{x,y}` or `{x,y,z}` at one corner → `chamfer_edges` returns
NULL; only single / opposite-pair edges land — confirmed in source.)

This is the PLANAR analogue of the M2 spherical corner weld the roadmap names as the
enabler for `fillet_face` full-face fillets. The same corner blocker (the shared corner
lost after the first edge's set-back) is what makes `fillet_face` decline
(`WeldGatesM2`) on a face's corner-sharing edge loop. This change lands the planar
(chamfer) corner weld, which is all-planar and therefore does NOT depend on the
tessellator's curved-seam weld.

## What

One additive OCCT-free header-only verb
`src/native/blend/corner_chamfer_weld.h` (`chamfer_corner`) — an ADDITIVE SIBLING of
`chamfer_edges` (which stays BYTE-FROZEN, its sequential path unchanged). It resolves
EVERY picked edge and its chamfer plane UP FRONT against the ORIGINAL (un-clipped) soup
— where the shared corner still exists — then applies ALL the clips. The mutual
intersection of the chamfer planes near a DIHEDRAL corner is closed automatically by the
boolean's exposed-ring face synthesis (`chamfer_edges` `detail::applyCut`) +
`assembleSolid`'s T-junction repair, which synthesises the corner facet with NO extra
geometry. Being ALL-PLANAR it welds watertight through the SAME `assembleSolid` path
with NO tessellator change. It runs a mandatory watertight + shrink self-verify and
DECLINES (NULL → OCCT) with a measured reason otherwise.

The native engine's `chamfer_edges` glue (`src/engine/native/native_engine.cpp`) tries
`chamfer_corner` as an ADDITIVE candidate after the sequential planar chamfer and before
the curved chamfer, gated by the SAME shrink self-verify — so `cc_chamfer_edges` lands
the adjacent-corner case with NO `cc_*` ABI change.

**Oracle-matched scope (honest).** The 2-edge DIHEDRAL corner (a union of two setback
half-space prisms) matches OCCT `BRepFilletAPI_MakeChamfer` EXACTLY (measured native ==
OCCT to fp64, 990.333 on a 10³ box, d=1). A TRIPLE corner (≥3 picked edges at one
vertex) does NOT: OCCT breaks the triple corner into chamfer-chamfer facets that trim
MORE material than a plain intersection of the three setback half-spaces (measured on
the booted sim: OCCT 985.667 vs the half-space 985.75, d=1). We CANNOT MATCH the OCCT
oracle at a triple corner, so the verb DECLINES it (`TripleCornerOracleGap`) → OCCT,
rather than emit a solid the oracle disagrees with. No tolerance is widened.

## Impact

- **Additive only.** New file `corner_chamfer_weld.h`; `chamfer_edges.h`,
  `fillet_edges.h`, `full_round.h`, `fillet_face.h`, the whole `src/native/tessellate/**`
  and M0/M1/M2 weld paths are BYTE-IDENTICAL. `src/native/**` stays OCCT-free. No `cc_*`
  ABI change (routed through the existing `cc_chamfer_edges`). The engine glue gains ONE
  additive candidate.
- **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_corner_chamfer.cpp`
  (6/6). On a 10³ box: `chamfer_corner` welds 1 edge, a 2-adjacent-edge corner, and a
  non-orthogonal (60° triangular-prism) corner watertight at the EXACT
  inclusion-exclusion closed-form volume (1-edge `d²L/2`; 2-adjacent `d²(L − d/3)`),
  swept over four setbacks; the sequential `chamfer_edges` DECLINES the same adjacent
  set (the contrast asserted); the triple corner DECLINES with `TripleCornerOracleGap`;
  and the honest declines (curved solid → `NonPlanarSolid`, bad id → `EdgeNotFound`,
  degenerate distance → `BadInput`, oversized → self-verify decline) are each measured.
- **Gate B (SIM native-vs-OCCT)** — `tests/sim/native_blend_parity.mm` +
  `scripts/run-sim-native-blend.sh` (20 passed / 0 failed, booted iOS simulator, through
  `cc_chamfer_edges` under both engines): the new `chamfer-corner2` case is native ==
  OCCT `BRepFilletAPI_MakeChamfer` to fp64 (vol 990.333, rel 0.00e+00, watertight); the
  `triple-corner-guard` case proves the 3-edge triple corner DECLINES to id 0 (never a
  solid the oracle disagrees with); all pre-existing blend cases still pass (zero
  regression). The runner gains `TKHLR` to its OCCT link set (a pre-existing missing
  toolkit surfaced by relinking, not a behaviour change).
- Unblocks `chamfer` on adjacent convex-corner edge sets and is the planar sibling of
  the still-tessellator-gated spherical fillet corner weld.
