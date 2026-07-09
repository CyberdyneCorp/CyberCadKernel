# Proposal — moat-m7t-construct-tails (MOAT M7 construct tails)

## Why

Two construct ops are still Class-B in `DROP-OCCT-READINESS.md` — native covers only
a degenerate slice, so the typical use forwards to the LGPL engine on the critical
path:

- **`twisted_sweep` with a REAL twist** (readiness row 65, Class B): the native
  `build_twisted_sweep` today serves ONLY the no-op reduction (`twist ≈ 0` AND
  `scale ≈ 1`), forwarding to `build_sweep`. Any genuine twist — the *typical* use of
  a twisted sweep — hits a `kTwistDeferThreshold` guard in
  `build_section_thrusections` and returns NULL → OCCT `ThruSections`. The recorded
  reason ("a densified twisted saddle ruled tube does not weld robustly watertight")
  is **empirically false**: a densified Frenet-framed twisted ruled tube welds
  watertight at every deflection and its volume converges to the true (area-preserving)
  value.
- **`loft_along_rail` with a CURVED rail** (readiness row 66, Class B): the native
  `build_loft_along_rail` serves ONLY a *straight* rail (a perpendicular-framed ruled
  loft). A curved rail — the *typical* use — hits the `spineIsStraight` gate and returns
  NULL → OCCT `MakePipeShell`.

Both are bounded, disjoint construct sites that reuse the already-landed native
substrate (`detail::frenetSectionFrames`, `detail::rmfFrames`,
`detail::assembleRingTube`, `detail::sectionRing`) proven by the landed
`guided_sweep` / `guided_orient_sweep` / `solid_loft_sections`. They land natively
where the tube self-verifies watertight and oracle-correct, and DECLINE honestly where
it cannot (a self-folding tight twist, a tight-curvature rail whose per-band turn the
native mesher cannot weld).

## What changes

- **`src/native/construct/sweep.h`** (OCCT-free, additive):
  - `build_section_thrusections` — REMOVE the blanket `kTwistDeferThreshold` early
    return; instead DENSIFY the spine so each band's accumulated twist stays under a
    small per-band bound (the native ruled mesher welds gentle-rotation bands, as the
    landed `guided_orient_sweep` already proves), then build the twisted ruled tube via
    `assembleRingTube`. Keep the `sectionSweepUnsafe` self-fold guard → NULL → OCCT.
  - `build_twisted_sweep` — a real twist now reaches the densified path (the no-op
    reduction to `build_sweep` is unchanged).
  - `build_loft_along_rail` — a CURVED rail is now served: densify the rail (bounding
    the per-band turn), RMF-transport the section morph A→B along the rail, tile with
    `assembleRingTube`. A tight-curvature rail whose per-band turn is too coarse to weld
    is caught by the engine's `robustlyWatertight` self-verify → NULL → OCCT. The
    straight-rail path (perpendicular-framed ruled loft) is unchanged.
- **`NativeEngine::twisted_sweep` / `NativeEngine::loft_along_rail`** — unchanged in
  shape; they already self-verify `robustlyWatertight` + positive volume and fall
  through on NULL. (`twisted_sweep` gains the same self-verify as `loft_along_rail`.)
- **Regression tests** — host analytic gate (`tests/native/test_native_sweep.cpp`):
  a twisted prism's volume converges to the untwisted `area·L` (twist preserves the
  cross-section area along a straight spine); a profile lofted along a circular-arc rail
  converges to the Pappus torus-sector volume. Plus a self-folding twist and a
  tight-curvature rail assert NULL (honest decline).
- **New sim parity harness** `tests/sim/native_construct_tails_parity.mm` (own
  `main()` + `scripts/run-sim-native-construct-tails.sh` + a SKIP entry in
  `run-sim-suite.sh`): native-vs-OCCT parity for a densified real-twist
  `cc_twisted_sweep` (vs `ThruSections`) and a densified curved-rail
  `cc_loft_along_rail` (vs `MakePipeShell`), comparing volume / area / watertight /
  Euler χ = 2 / bbox, plus an honest-decline case each.

Nothing in `cc_*` changes (additive native path behind the existing facade seam). No
tessellator change. NumPP/SciPP unaffected.

## Impact

- Affected specs: `native-construction` (MODIFIED: the twisted-sweep no-op-only
  requirement and the curved-rail deferral become native-served with self-verify).
- Affected code: `src/native/construct/sweep.h`; `tests/native/test_native_sweep.cpp`;
  new `tests/sim/native_construct_tails_parity.mm`,
  `scripts/run-sim-native-construct-tails.sh`; `scripts/run-sim-suite.sh` (SKIP entry);
  `openspec/MOAT-ROADMAP.md`, `openspec/DROP-OCCT-READINESS.md` (status B→A).
- ABI: unchanged. `src/native/**` stays OCCT-free. Fallback discipline preserved
  (native NULL / self-verify DISCARD → OCCT, never a faked or leaky solid).
