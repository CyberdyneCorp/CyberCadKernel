# Tasks — moat-canal-cyl-cyl-fillet

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `src/native/blend/canal_fillet.h` — `detail::steinmetzFrame` recognizer FROM THE
  PLANAR-FACET SOUP (the native SSI boolean bakes cylinders into facets): recover the two
  orthogonal axes (⟂ the facet-normal families), the common Rc, and the crossing point.
- [x] 1.2 Add the canal-strip geometry: centre curve at CONSTANT distance R0 = Rc − r from both
  axes in each crease plane (z=±x), seam directions = the two cylinder radials, one canonical
  `slerp` sample shared by both incident faces so seams weld bit-identically.
- [x] 1.3 Rebuild the whole filleted bicylinder COMMON as a planar-facet soup: four trimmed
  lune wall patches (cyl A by azimuth, cyl B by its crease-arc pairing), two canal strips
  tapering to the shared pole vertices, no caps; flood-fill to a coherent outward orientation.
- [x] 1.4 Add public `blend::canal_fillet_edge(solid, edgeIds, edgeCount, r, deflection)` with
  a MANDATORY internal self-verify (consistently oriented + removed-volume bound catches folds);
  register in `native_blend.h`.

## 2. Engine wiring
- [x] 2.1 Add `nblend::canal_fillet_edge` as candidate #6 in `NativeEngine::fillet_edges`,
  gated by the existing `blendResultVerified` SHRINK self-verify (0 < Vr < Vo). Replace the
  stale T3 decline comment with the landed-arm description.

## 3. Gate A — host (analytic, no OCCT)
- [x] 3.1 `canal_fillet_steinmetz_watertight_volume_reduced` — watertight + χ=2 + consistent
  orientation + enclosed volume < sharp bicylinder (16/3·Rc³) and ≈ closed form.
- [x] 3.2 `canal_fillet_converges_with_deflection` — volume tightens as deflection refines.
- [x] 3.3 `canal_fillet_radius_range` — lands native + watertight + oriented + shrinks across
  r/Rc ∈ [0.1, 0.4] (the canal seams are G1-tangent to both walls by construction: the strip
  normal is the wall radial at each seam).
- [x] 3.4 `canal_fillet_scope_defers` — box / single cyl / Rc<2r / multi-edge / r≤0.
- [x] 3.5 Full host `ctest` green.

## 4. Gate B — sim (native-vs-OCCT parity)
- [x] 4.1 Add `runCanalCase` to `native_curved_fillet_parity.mm`: OCCT oracle
  (`BRepFilletAPI_MakeFillet` + `BRepGProp`) confirms the bicylinder-COMMON fillet; native path
  via the `cc_boolean` facade (records the honest boolean-track body-build gap when it declines).
- [x] 4.2 Run on the booted simulator; 50/50 PASS (all existing 44 + the 3 canal cases' oracle
  + native-note, unchanged existing suite).

## 5. Byte-identical proof (tessellator unchanged)
- [x] 5.1 `git diff src/native/tessellate` empty (no mesher change) → firewall trivially met;
  the full host `ctest` (65/65) confirms every existing surface-kind mesh is unchanged.

## 6. Validate
- [x] 6.1 `openspec validate moat-canal-cyl-cyl-fillet --strict`.
- [x] 6.2 Confirm `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
- [x] 6.3 Update `openspec/MOAT-ROADMAP.md` + `DROP-OCCT-READINESS.md` M3 row.
