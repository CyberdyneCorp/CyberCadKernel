# Tasks — moat-m1f-ssi-fit-conditioning

## 1. Diagnose (host)
- [x] 1.1 Confirm the trigger is looser than the verification budget: fitTarget 2.203e-03 vs the
  failing pose's maxFitError 6.690100e-04 (reproduced to 7 digits by four instrumentations).
- [x] 1.2 Find the load-bearing defect: `target = min(m, 200)` reaches `m` for m ≤ 200, making the
  fit interpolate — maxFitError 3.639e-06 while true off-surface deviation is 4.990e-01.
- [x] 1.3 Prove the naive fix is harmful: tightening the trigger ALONE gives Gate B 21/2 —
  dx=0.597 → 1.85e-01 and dx=0.595 regresses 1.73e-04 → 1.90e-01.
- [x] 1.4 Reject the knot-vector remedy: `approxKnots` breaks `skew cyl unequal`
  (onSurf 7.39e-07 → 1.25e-06 vs its own 1e-6 tol) and perturbs 16–18 of 23 gate lines.
- [x] 1.5 Refute node spacing as a cause: nodes sit 2.93e-10 from the analytic locus and support
  ~1e-6; the achievable fit is pole-limited, not spacing-limited.

## 2. Implement (src/native/ssi)
- [x] 2.1 Conditioning guard: `target = min(m * 2 / 3, kDensifyMaxPoles)` (relative, not flat).
- [x] 2.2 Between-node accept test (`midpointDeviation`) ANDed with the at-node comparison.
- [x] 2.3 `MarchOptions::fitDensifyTargetScale` (default 2e-5) replacing the hardcoded 2e-4.

## 3. Gate A — host self-consistency
- [x] 3.1 NEW `march_densify_refit_never_interpolates_s4c` — refit fires (nPoles > 64) but stops
  short of interpolation (nPoles < m), and the curve sampled BETWEEN nodes stays on both surfaces
  under 5e-4, asserted OCCT-free.
- [x] 3.2 Existing `march_densify_refit_high_curvature_loop` stays green — proves the relative cap
  did not starve a loop that genuinely needs the full 200 poles.
- [x] 3.3 Full suite 25/25; ssi 11/11, seeding 11/11, exact_fuzz 147 agreed / 0 disagreed.

## 4. Gate B — native-vs-OCCT parity
- [x] 4.1 Restore dx = 0.597 to `pairWideBandIncrementalOrientationS4c` and delete the exclusion.
- [x] 4.2 Host Gate B 22 passed / 0 failed; dx=0.597 onCurve 2.30e-05; 20 of 22 lines
  byte-identical; `skew cyl unequal` unchanged at 7.39e-07.
- [ ] 4.3 Run `run-sim-native-ssi-marching` on the booted simulator. **(macOS-only — left for the Mac.)**

## 5. Structural + finalize
- [x] 5.1 Diff confined to `ssi/marching.{h,cpp}`; OCCT-free; `cc_*` unchanged; no tolerance widened.
- [x] 5.2 Update `openspec/MOAT-ROADMAP.md`.
- [x] 5.3 `openspec validate --strict moat-m1f-ssi-fit-conditioning`.
