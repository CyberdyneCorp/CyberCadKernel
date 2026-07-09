# Tasks — moat-m2xc-cyl-cyl-common-facade (HANDOFF)

## 1. Localize (host probe)
- [ ] 1.1 Probe: build the two equal-radius orthogonal cylinders BOTH ways — (a)
  `nb::curved::buildCommonSegment` axis Z & X (the working case), (b) `build_prism_profile`
  full-circle extrude + a located 90°-about-Y rotation (the facade case). For each, print
  `ssi_boolean_solid(Common)` non-null, `boolean_solid(Common)` non-null + watertight, and
  `booleanResultVerified`. Confirm whether the miss is SSI RECOGNITION (case 1) or VERIFY
  (case 2).

## 2. Fix (src/native/boolean — owning track)
- [ ] 2.1 If recognition: make `ssidetail::recogniseCurvedSolid` / `steinmetzPreGate` accept a
  LOCATED/rotated cylinder operand (fold the location into the axis/frame before the
  equal-radius / orthogonal / axis-crossing tests), so a facade-rotated X-cylinder is
  recognized identically to a `buildCommonSegment` one.
- [ ] 2.2 If verify: reconcile `ssiCurvedBooleanVerified` (or the inclusion–exclusion fallback)
  so a correct facade-built Steinmetz passes (never widen a tolerance to pass a wrong solid;
  keep `isConsistentlyOriented` + the 16 r³/3 oracle).
- [ ] 2.3 Keep the honest decline for non-Steinmetz cyl-cyl (unequal / non-orthogonal /
  non-crossing / near-tangent) → OCCT.

## 3. Gate A — host (no OCCT)
- [ ] 3.1 A native COMMON of two facade-shaped (extruded + rotated) equal-radius orthogonal
  cylinders is watertight, consistently oriented, volume → 16/3·Rc³ to the deflection bound.
- [ ] 3.2 A non-Steinmetz cyl-cyl pair still declines.

## 4. Gate B — sim (native-vs-OCCT)
- [ ] 4.1 `run-sim-native-curved-fillet.sh` — the 3 canal cases flip from `native-note`
  (body-build gap) to full native `mass` + `tessellate` + `occt-parity` PASS. No change needed
  in `native_curved_fillet_parity.mm` (the native-vs-OCCT path is already wired behind the
  body-build).

## 5. Validate
- [ ] 5.1 `openspec validate moat-m2xc-cyl-cyl-common-facade --strict`.
- [ ] 5.2 `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
