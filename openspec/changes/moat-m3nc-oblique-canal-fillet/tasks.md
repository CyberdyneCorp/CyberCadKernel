# Tasks — moat-m3nc-oblique-canal-fillet

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `src/native/blend/canal_fillet_oblique.h` — `detail::obliqueCylFrame` recognizer
  FROM THE PLANAR-FACET SOUP: recover the two crossing axes (⟂ the facet-normal families) and TWO
  DISTINCT radii classified PER FACET BY RADIUS; REQUIRE a clearly non-orthogonal, non-parallel
  crossing (|cosα| ∈ (0.05, 0.97)); decline on equal radii, orthogonal/near-parallel axes, a third
  family, or any stray facet.
- [x] 1.2 Add the oblique canal geometry: canonical frame (thin axis `ez`, thick axis
  `b̂ = sinα·ex + cosα·ez`); spine `cz±(u) = [R0·cosu·cosα ± √(R0b²−R0²sin²u)]/sinα`, never a pole
  since `Rb > Ra`; seam directions = the two cylinder radials; one canonical slerp shared by both
  incident faces (bit-identical seams).
- [x] 1.3 Rebuild the whole filleted COMMON as a planar-facet soup: two closed canal strips
  (top/bottom disjoint crease loops), the thin wall's waist tube band, the two thick-wall cap
  patches (ring-fanned in the thick cylinder's OWN frame so any axis tilt is handled uniformly);
  flood-fill to a coherent outward orientation; no caps, no poles.
- [x] 1.4 Add public `blend::oblique_canal_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`
  with a MANDATORY internal self-verify (consistently oriented + removed-volume bound scaled by
  1/sinα catches folds); register in `native_blend.h`.

## 2. Engine wiring
- [x] 2.1 Add `nblend::oblique_canal_fillet_edge` as candidate #8 in `NativeEngine::fillet_edges`,
  gated by the existing `blendResultVerified` SHRINK self-verify (0 < Vr < Vo). Orthogonal axes
  route to #7; equal radii / near-parallel decline.

## 3. Gate A — host (analytic, no OCCT)
- [x] 3.1 `oblique_canal_fillet_watertight_volume_reduced` — watertight + χ=2 + consistent
  orientation + enclosed volume < sharp oblique bicylinder (measured 60°, r=0.2:
  vSharp=11.004063 → vFilleted=10.811591, removed=0.192472).
- [x] 3.2 `oblique_canal_fillet_converges_with_deflection` — volume tightens as deflection refines.
- [x] 3.3 `oblique_canal_fillet_radius_and_angle_range` — lands native + watertight + oriented +
  shrinks across `r ∈ [0.1, 0.4]` × α ∈ {45°, 60°, 75°}.
- [x] 3.4 `oblique_canal_fillet_g1_tangent` — ANALYTIC G1: strip normal == thin-wall radial at t=0
  seam and == thick-wall radial at t=1 seam; each seam point lies ON its wall (no OCCT/mesh).
- [x] 3.5 `oblique_canal_fillet_scope_defers` — the ORTHOGONAL sibling arms (equal Steinmetz,
  orthogonal unequal) BOTH decline on an oblique body (before/after: it fell through to OCCT); an
  orthogonal body routes AWAY from this arm; box / equal radii / thin `Ra<2r` / `r≤0` / multi-edge
  all NULL; control lands.
- [x] 3.6 Full host `test_native_blend` green (102 cases, 0 failed) + `test_native_engine` green
  (50 cases, 0 failed).

## 4. Gate B — sim (native-vs-OCCT parity)
- [ ] 4.1 Add `runObliqueCanalCase` to `native_curved_fillet_parity.mm`: OCCT oracle
  (`BRepPrimAPI_MakeCylinder` ×2 at an oblique relative transform + `Common` +
  `BRepFilletAPI_MakeFillet` + `BRepGProp`) confirms the oblique-bicylinder fillet; native fillet
  host-gated; record the boolean-track body-build gap honestly. (Encoded structurally; run on the
  booted simulator when the sim toolchain is reachable — mirrors the orthogonal unequal slice.)

## 5. Byte-identical proof (tessellator unchanged)
- [x] 5.1 `git diff src/native/tessellate` empty (no mesher change) → firewall trivially met.
- [x] 5.2 Existing blend/engine suites unchanged when the new arm's gate does not match.
