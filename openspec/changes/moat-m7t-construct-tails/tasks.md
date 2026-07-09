# Tasks — moat-m7t-construct-tails

## 1. Native builders (src/native/construct/sweep.h, OCCT-free)

- [x] 1.1 `build_section_thrusections`: replace the blanket `kTwistDeferThreshold`
      early return with a per-band-twist densification of the straight spine
      (`nBands = ceil(|totalTwist| / kMaxBandTwist)`, capped ≤ 512), then build the
      twisted ruled tube via `assembleRingTube`; keep `sectionSweepUnsafe`.
- [x] 1.2 `build_twisted_sweep`: real twist now reaches the densified path; the no-op
      reduction (`twist≈0 && scale≈1` → `build_sweep`) is unchanged.
- [x] 1.3 `build_loft_along_rail`: serve a CURVED rail — densify the rail by a per-band
      turn bound, RMF-transport a linear A→B section morph, tile via
      `assembleRingTube`. Straight-rail path unchanged. Tight/degenerate → NULL.
- [x] 1.4 Confirm cognitive complexity stays within the compilers/parsers band
      (≤25–35) for the touched functions.

## 2. Engine wiring (src/engine/native/native_engine.cpp)

- [x] 2.1 `NativeEngine::twisted_sweep`: add the `robustlyWatertight` + positive-volume
      self-verify (same shape as `loft_along_rail`) so a non-watertight twisted
      candidate is discarded → OCCT.
- [x] 2.2 Verify `loft_along_rail` self-verify already discards a non-watertight curved
      candidate → OCCT (no change beyond confirming).

## 3. Host analytic gate (tests/native/test_native_sweep.cpp, OCCT-free)

- [x] 3.1 Real-twist prism: volume converges to `area·L`; watertight across the
      deflection ladder; topology correct.
- [x] 3.2 Twisted + scaled prism: volume converges to the frustum-of-twist estimate.
- [x] 3.3 Curved arc-rail loft: volume converges to the Pappus torus-sector
      `polyArea·R·φ`; watertight.
- [x] 3.4 Honest declines: a self-folding twist → NULL; a too-tight rail → NULL or
      non-watertight (engine discards).
- [x] 3.5 Update the stale header comment describing the twist/rail deferrals.

## 4. Sim parity gate (Gate 2)

- [x] 4.1 New `tests/sim/native_construct_tails_parity.mm` (own `main()`): densified
      real-twist `cc_twisted_sweep` vs OCCT `ThruSections`; densified curved-rail
      `cc_loft_along_rail` vs OCCT `MakePipeShell`; compare volume / area / watertight /
      Euler χ=2 / bbox; plus one honest-decline case each (fall-through, rel 0).
- [x] 4.2 New `scripts/run-sim-native-construct-tails.sh` (sibling of
      run-sim-native-sweep.sh).
- [x] 4.3 Add the harness to the SKIP list in `scripts/run-sim-suite.sh`.

## 5. Docs + validation

- [x] 5.1 Spec delta `specs/native-construction/spec.md` (MODIFIED requirements).
- [x] 5.2 `openspec validate moat-m7t-construct-tails --strict` until clean.
- [x] 5.3 `openspec/DROP-OCCT-READINESS.md` rows 65/66 B→A.
- [x] 5.4 `openspec/MOAT-ROADMAP.md` M7 status: twisted-sweep + curved-rail landed.

## 6. Verify + commit

- [x] 6.1 Host: `scripts/build-numsci.sh host`, configure, build, `ctest` green.
- [x] 6.2 iossim: `scripts/build-numsci.sh iossim` exit 0 (sim slice compiles).
- [x] 6.3 Structural: `git diff src/native` OCCT-free & additive; `cc_*` unchanged.
- [x] 6.4 Commit to branch `moat-m7t` (concise technical message).
