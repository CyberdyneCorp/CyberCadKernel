# Tasks — moat-m1b-ssi-breadth

## 1. Empirical confirmation (no code shipped)
- [x] Host-probe the S2 seeder + S3 tracer on general non-coaxial / skew analytic quadric poses;
      confirm all trace with `nearTangentGaps == 0` and max on-both-surfaces residual ≈ 1e-11
      (recorded in design.md).
- [x] Confirm the decline is S1-closed-form-only (`quadric_pairs.h`), not a tracer gap.

## 2. Gate A — host analytic (OCCT-free), additive
- [x] Add `march_skew_cylinders_general_single_loop` (gap + oblique 60° tilt → one connected
      quartic loop; all nodes on both cylinders ≤ 1e-9; Closed; `nearTangentGaps == 0`) to
      `tests/native/test_native_ssi_marching.cpp`.
- [x] Add `march_sphere_cone_offaxis` (off-axis sphere∩cone → one loop, nodes on both ≤ 1e-9).
- [x] All prior `test_native_ssi_marching.cpp` cases stay green (frozen) — host 16/16.

## 3. Gate B — sim native-vs-OCCT parity, additive
- [x] Add `pairSkewCylindersGeneral` (single quartic loop, gap + oblique) to
      `tests/sim/native_ssi_marching_parity.mm`, verified vs `GeomAPI_IntSS` via `reportPair`.
- [x] Add `pairSphereConeOffAxis` (off-axis sphere∩cone, finite-sphere-bounded single loop).
- [x] Register the two new `pair*()` in `main()`; existing cases unchanged.
- [x] Run `scripts/run-sim-native-ssi-marching.sh` on a booted iOS simulator: **14 passed /
      0 failed** — both new families PASS (skew cyl general 1/1 onCurve 9.4e-5 onSurf 5.2e-5
      lenDelta 2.7e-6; sphere cone off-axis 1/1 onCurve 3.4e-5 onSurf 1.2e-5 lenDelta 1.8e-6);
      all 12 prior cases still PASS.
- [x] Honestly DECLINE general cone∩cone / off-axis cyl∩cone / off-axis sphere∩cyl with a
      MEASURED oracle reason (infinite-quadric multi-loop vs finite native patch + seeding
      recall) recorded in the oracle-setup note; no fake, no weakened tolerance.

## 4. Honest-decline preservation
- [x] S1 `intersect_surfaces` still returns `NotAnalytic` for skew cyl∩cyl / general cone∩cone /
      non-coaxial quadrics; `test_native_ssi.cpp` `NotAnalytic` assertions unchanged.
- [x] No tolerance widened to force a pass; the unbounded-quadric multi-loop poses are the
      honest declined tail (measured on the oracle, not padded).

## 5. Discipline
- [x] `src/native/**` byte-identical (`git diff src/native` empty).
- [x] Tessellator untouched (`git diff src/native/tessellate` empty).
- [x] `cc_*` ABI unchanged (`git diff include/` empty).
- [x] NumPP/SciPP by absolute path only; substrate never vendored.

## 6. Spec + roadmap
- [x] Add the `native-ssi` delta requirement (general non-coaxial / skew analytic quadric
      breadth, verified vs OCCT).
- [x] `openspec validate moat-m1b-ssi-breadth --strict` passes.
- [x] Update `openspec/MOAT-ROADMAP.md` M1 status.
