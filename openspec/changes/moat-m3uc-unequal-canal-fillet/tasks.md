# Tasks — moat-m3uc-unequal-canal-fillet

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `src/native/blend/canal_fillet_unequal.h` — `detail::unequalCylFrame` recognizer
  FROM THE PLANAR-FACET SOUP: recover the two orthogonal axes (⟂ the facet-normal families) and
  TWO DISTINCT radii (`Ra`, `Rb`) classified PER FACET BY RADIUS (robust to coarse-cap normal
  tilt); decline on equal radii, a third family, or any stray facet.
- [x] 1.2 Add the canal geometry: rolling-ball centre at distance `R0a=Ra−r` from the thin axis
  and `R0b=Rb−r` from the thick axis (`cz=±√(R0b²−R0a² sin²u)`, never a pole); seam directions =
  the two cylinder radials; one canonical slerp shared by both incident faces (bit-identical
  seams).
- [x] 1.3 Rebuild the whole filleted COMMON as a planar-facet soup: two closed canal strips
  (top/bottom disjoint crease loops), the thin wall's waist tube band, the two thick-wall cap
  patches (ring-fanned from the loop centroid on the thick wall); flood-fill to a coherent
  outward orientation; no caps, no poles.
- [x] 1.4 Add public `blend::unequal_canal_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`
  with a MANDATORY internal self-verify (consistently oriented + removed-volume bound catches
  folds); register in `native_blend.h`.

## 2. Engine wiring
- [x] 2.1 Add `nblend::unequal_canal_fillet_edge` as candidate #7 in `NativeEngine::fillet_edges`,
  gated by the existing `blendResultVerified` SHRINK self-verify (0 < Vr < Vo). Equal radii route
  to #6.

## 3. Gate A — host (analytic, no OCCT)
- [x] 3.1 `unequal_canal_fillet_watertight_volume_reduced` — watertight + χ=2 + consistent
  orientation + enclosed volume < sharp unequal bicylinder.
- [x] 3.2 `unequal_canal_fillet_converges_with_deflection` — volume tightens as deflection refines.
- [x] 3.3 `unequal_canal_fillet_radius_range` — lands native + watertight + oriented + shrinks
  across `r ∈ [0.1, 0.4]`.
- [x] 3.4 `unequal_canal_fillet_g1_tangent` — ANALYTIC G1: strip normal == thin-wall radial at
  t=0 seam and == thick-wall radial at t=1 seam; each seam point lies ON its wall (no OCCT/mesh).
- [x] 3.5 `unequal_canal_fillet_scope_defers` — box / equal radii / thin `Ra<2r` / `r≤0` /
  multi-edge all NULL; control lands.
- [x] 3.6 Full host `ctest` green.

## 4. Gate B — sim (native-vs-OCCT parity)
- [ ] 4.1 Add `runUnequalCanalCase` to `native_curved_fillet_parity.mm`: OCCT oracle
  (`BRepPrimAPI_MakeCylinder` ×2 + `Common` + `BRepFilletAPI_MakeFillet` + `BRepGProp`) confirms
  the unequal-bicylinder fillet; native fillet host-gated; record the boolean-track body-build gap
  honestly when the native COMMON declines through the facade. (Encoded structurally; run on the
  booted simulator when the sim toolchain is reachable.)

## 5. Byte-identical proof (tessellator unchanged)
- [x] 5.1 `git diff src/native/tessellate` empty (no mesher change) → firewall trivially met.
- [x] 5.2 Existing blend/engine suites unchanged when the new arm's gate does not match.
