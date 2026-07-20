# Proposal — moat-m3nc-oblique-canal-fillet (MOAT M3, NON-ORTHOGONAL unequal cyl↔cyl canal fillet)

## Why

`cc_fillet_edges` is the app's HEAVIEST curved-blend gap. The landed canal-fillet arms serve
the crossing crease of two EQUAL-radius ORTHOGONAL cylinders (Steinmetz, `canal_fillet.h`) and
two UNEQUAL-radius ORTHOGONAL cylinders (`canal_fillet_unequal.h`). Both explicitly declined the
NON-ORTHOGONAL crossing:

> "Unequal radii / non-orthogonal / TORUS / elliptical creases still decline → OCCT."
> "non-orthogonal / non-crossing axes ... still decline → OCCT."

The immediate deferred sibling is the crossing crease of two cylinders whose axes meet at an
OBLIQUE angle α ≠ 90° — an angled branch pipe / slanted through-bore, a common mechanical
feature. The old decline was too pessimistic: with DISTINCT radii the geometry stays in the
REGULAR (disjoint-loop) topology at ANY crossing angle, and the rolling-ball tangency
construction is UNCHANGED. Only OCCT served this rim before.

## What changes

- **`src/native/blend/canal_fillet_oblique.h` (new):** `detail::obliqueCylFrame` /
  `detail::oblCanalCz` / `detail::oblCanalStripPoint` / `detail::buildObliqueCanalSoup` and the
  public entry `blend::oblique_canal_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`. It
  recognises the body WHOLESALE from the planar-facet soup: two cylinder wall families whose
  axes CROSS at a clearly non-orthogonal, non-parallel angle, with DISTINCT radii. In a canonical
  frame (thin axis `ẑ`, thick axis `b̂ = sinα·x̂ + cosα·ẑ`) the crease/spine solves a quadric in
  the axial coordinate:

      cz±(u) = [ R0·cos u·cosα ± √(R0b² − R0² sin²u) ] / sinα ,   R0 = Ra−r, R0b = Rb−r.

  Because `Rb > Ra` the discriminant never vanishes, so the two crease loops are DISJOINT — each
  gets a full closed canal strip (G1-tangent to both walls by construction), welded to the thin
  wall's waist tube and two thick-wall cap patches through the existing `assembleSolid` — **no
  tessellator change**. At α=90° the formula reduces EXACTLY to the orthogonal unequal case, which
  routes to `canal_fillet_unequal.h`; this arm fires only for α clearly ≠ 90°.
- **`NativeEngine::fillet_edges` (additive candidate #8):** after the orthogonal unequal arm (#7),
  try `nblend::oblique_canal_fillet_edge`, gated by the SAME `blendResultVerified` SHRINK
  self-verify (0 < Vr < Vo). Orthogonal axes route to #7; equal radii and near-parallel axes
  decline.

## Honest scope / declines (→ OCCT)

- ORTHOGONAL axes (|cosα| ≤ ~0.05) → `canal_fillet_unequal.h`; near-parallel axes
  (|cosα| ≥ 0.97) → NULL (not a crossing bicylinder / sinα → 0 blows up the spine).
- EQUAL radii pinch at two poles at ANY angle — the disjoint-loop builder declines (that is a
  separate, harder slice needing the pole/lune weld of the Steinmetz case).
- Ring-torus guard `Ra ≥ 2r`; a third wall family / a stray facet / `r ≤ 0` / a multi-edge pick →
  NULL → OCCT.
- The idealized perpendicular canal cross-section differs from OCCT's variable-dihedral canal by
  a small modeling-convention gap (REPORTED, never gated). A MANDATORY internal self-verify
  (consistent orientation + removed-volume bound) rejects any large-radius fold → NULL → OCCT.
- BODY-BUILD CAVEAT (measured, mirrors the orthogonal unequal slice): the native SSI boolean does
  not build the oblique COMMON practically, so the host gate feeds the recognizer a clean directly-
  built oblique-bicylinder soup; the sim gate would build via OCCT.

## Impact

- The landed blend substrate (`canal_fillet.h`, `canal_fillet_unequal.h`, the cylinder/cone/sphere
  arms of `curved_fillet.h`), M0/M1/M2 and the byte-frozen boolean welds remain **untouched**; the
  oblique arm is an additive sibling of the orthogonal unequal arm.
- `src/native/**` stays OCCT-free; the tessellator is consumed as-is, UNTOUCHED.
- `cc_*` ABI unchanged (native path only, behind the existing `fillet_edges` facade seam); existing
  blend paths are BYTE-IDENTICAL when the new gate does not match.
- Readiness: `cc_fillet_edges` moves the analytic non-orthogonal unequal cyl↔cyl crossing-crease
  envelope from OCCT-forward to native; the equal-radius pinch and torus/elliptical residual stays
  OCCT.
