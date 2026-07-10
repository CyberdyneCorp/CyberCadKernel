# Tasks — moat-vsfix-coupled-morph

## 1. The fix (src/native/construct/sweep.h)

- [x] 1.1 Add `kMaxCoupledVolErr` (0.2% relative volume) and `straightCoupledStations(rA,
      rB, guideScaleAt)` — the fewest uniform bands whose piecewise-linear-`g` area integral
      `∫(chord g)²` matches the true `∫g²` (g = section-radius envelope · guide scale) to
      within the bound; returns 2 when `g` is linear (the two exact sub-regimes), bounded by
      `kMaxDensifyStations`.
- [x] 1.2 Wire it into `build_variable_sweep_tube`: STRAIGHT spine → `nStations =
      straightCoupledStations(...)`; CURVED spine → the turn-driven count raised to the
      coupled count. Reuse the perpendicular / RMF framing, `assembleRingTube`, and the
      `sectionSweepUnsafe` self-fold guard byte-identically. Compute the cap circumradii
      once and reuse for both the station count and `circumR`.
- [x] 1.3 Confirm the two exact sub-regimes stay at 2 stations (byte-identical old path):
      pure morph (constant scale) and pure guide-scale (A==B) → g linear → 2 stations.

## 2. Regression tests

- [x] 2.1 `tests/native/test_native_vsweep.cpp` — add the coupled circle→circle + splaying
      guide straight case, asserting the native volume matches the exact polygon-clip
      closed form `C·H·∫₀¹ g² df` (Gate a) to a tight tolerance; add a convergence case over
      polygon counts; keep the two exact sub-regime cases and assert they stay exact.
- [x] 2.2 `tests/sim/native_vsweep_parity.mm` — add the coupled morph×scale straight parity
      case (native vs OCCT `MakePipeShell` within the deflection band) (Gate b).
- [x] 2.3 `tests/sim/native_vsweep_fuzz.mm` — remove the coupled-regime scope restriction on
      the `GUIDED_STRAIGHT` family (now three regimes incl. coupled); upgrade the closed-form
      quadrature to Boole's rule (exact for the ≤quartic coupled area); update the header
      notes to record the fix.

## 3. Build & gates

- [x] 3.1 `scripts/build-numsci.sh host` + `iossim` both exit 0.
- [x] 3.2 GATE (a): host `ctest` all-green, including the new coupled + convergence cases;
      the coupled divergence (19% → deflection-bounded match) captured; the two sub-regimes
      exact.
- [ ] 3.3 GATE (b): the coupled case native vs OCCT `MakePipeShell` within the deflection
      band on the booted simulator; no existing vsweep parity case regresses; the broadened
      fuzzer 0 DISAGREED across ≥2 seeds. *(SIM gate requires a booted iOS simulator; the
      host analytic gate + iossim build are the primary evidence in this environment.)*

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md`: vsweep coupled morph×scale now native, the
      M6-breadth-19 documented limitation resolved.
- [x] 4.2 `openspec validate moat-vsfix-coupled-morph --strict` passes.
- [x] 4.3 Structural check: `git diff src/native` OCCT-free and touches ONLY `sweep.h`;
      `cc_*` ABI unchanged; interference.h / tessellator / other modules untouched.
- [x] 4.4 Commit to branch `moat-vsfix` (concise technical message, no Claude/AI mention).
