# Tasks — moat-m2xc-cyl-cyl-common-facade (HANDOFF)

## 1. Localize (host probe)
- [x] 1.1 Probe added to `tests/native/test_native_ssi_curved_boolean.cpp` (later replaced by
  the gate). Built both ways and printed recognise / trace / ssi-null / volume. Result: the
  miss is CASE 1 (SSI RECOGNITION) — `recogniseCurvedSolid` MISSED the located/rotated X
  cylinder (`recogB=0`), and even after that, a second recogniser bug (cap-plane normal) made
  the COMMON lune-survival classify reject the interior. Two distinct recogniser bugs, both on
  the located/rotated operand.

## 2. Fix (src/native/boolean — owning track)
- [x] 2.1 Recognition, bug A: `ssidetail::worldFrame` DOUBLE-applied the face's cumulative
  location (once as `surf.location`, once as `face.location()`). `surfaceOf` already returns the
  cumulative location, so a 90°-about-Y fold became 180° (+Z→−Z), collapsing the axial vertex
  extent → miss. Fixed to fold ONCE (matching `feature/wrap_emboss.h`).
- [x] 2.2 Recognition, bug B: the cap-plane outward normal was taken from (frame-z, orientation),
  which the facade extruder (`build_prism_profile`) does NOT use to encode outwardness (it winds
  the wire instead) → both caps got +Z. Fixed to orient each cap normal GEOMETRICALLY (away from
  the solid's interior vertex centroid), construction-agnostic. No tolerance widened; the 16 r³/3
  oracle + `isConsistentlyOriented` self-verify are unchanged and still gate the result.
- [x] 2.3 Honest decline kept: unequal-radius ORTHOGONAL crossings remain a legitimate
  transversal native pass; PARALLEL-axis / non-crossing pairs still decline → NULL → OCCT.

## 3. Gate A — host (no OCCT)
- [x] 3.1 `steinmetz_facade_common_watertight_matches_analytic` — a native COMMON of two
  facade-shaped (extruded full-circle + located 90° rotation) equal-radius orthogonal cylinders
  is watertight + consistently oriented, volume 5.3287 vs 16/3·Rc³ = 5.3333 (0.09% deflection
  deficit), matching the DIRECT buildCommonSegment body.
- [x] 3.2 `non_steinmetz_facade_cyl_cyl_declines` — a parallel-axis facade pair declines (NULL).

## 4. Gate B — sim (native-vs-OCCT)
- [x] 4.1 `scripts/run-sim-native-curved-fillet.sh` (rebuilt with the numsci iossim substrate so
  the native SSI boolean is LIVE, not the decline-stub) — the 3 canal cases now report a NEW
  `native-body` PASS (the native Steinmetz body builds via `cc_boolean(cylZ, cylX, 2)`, volume
  → 16 Rc³/3 vs the OCCT oracle to the deflection bound), closing the body-build gap.
- [x] 4.2 HONEST DECLINE (sharpened, measured): the body builds, but the native SSI assembler
  emits a FACETED planar-facet shell (2592 two-point facet edges, no smooth crossing-crease
  ARC — measured in the host `native_facade_steinmetz_common` gate and the sim `native-note`),
  so `cc_fillet_edges` cannot seat the LANDED canal fillet through the facade. The remaining
  work is a smooth-crease native boolean emit (an assembler-representation change), NOT this
  recogniser stage, the fillet builder, or the tessellator.

## 5. Validate
- [x] 5.1 `openspec validate moat-m2xc-cyl-cyl-common-facade --strict` — valid.
- [x] 5.2 `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
