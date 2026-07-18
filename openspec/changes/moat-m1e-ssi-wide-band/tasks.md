# Tasks — moat-m1e-ssi-wide-band

## 1. Diagnose (host, OCCT-free)
- [x] 1.1 Refute the M1d wide-band hypothesis with measurements: band arc fraction (6.76–9.27%,
  closed-form verified to 1.5e-15), budget non-binding (crossMaxSteps 256→16384 changes nothing),
  and the declining runs' period-2/4 spatial limit cycle (arc 3.86 for net transport 0.21).
- [x] 1.2 Identify the mechanism: both the sign test (`marching.cpp:552`) and the adoption gate
  (`:553`) resolve against the frozen `t★` and degenerate together at a 90° accumulated turn.
- [x] 1.3 Confirm with an analytic predictor computed with no marcher: 90° crossover at
  dx★ = 0.592787 vs measured boundary dx = 0.5926 crosses / 0.5927 declines (agreement 1e-4).
- [x] 1.4 Correct the inherited ground-truth map: resolved `scale` is 11.014390, not 3.4641 — every
  budget figure derived from it was understated 3.18×.

## 2. Incremental orientation (src/native/ssi, additive, default-off)
- [x] 2.1 Add `MarchOptions::reanchorIncrementalOrientation` (default OFF); AND it with
  `adaptiveCrossReanchor` when resolving into `Tuned` so it can never act on the frozen path.
- [x] 2.2 Re-reference BOTH half-space tests to the previous accepted step direction, seeded to
  `t★`. (Sign-test-only was tested and rejected: it relocates the decline to step-floor collapse.)
- [x] 2.3 Add the net-transport termination guard (arc vs net displacement, 4×, floored by the
  crossing step cap so early steps cannot trip it).
- [x] 2.4 Prove non-regression with the flag default off: ssi 11/11, seeding 11/11,
  exact_fuzz 147 agreed / 0 disagreed, marching 22→24 with the new cases.

## 3. Gate A — host self-consistency (`test_native_ssi_marching.cpp`)
- [x] 3.1 `march_wide_band_incremental_orientation_s4c` — dx = 0.593/0.595/0.597 decline with the
  flag off and cross to ONE closed loop with it on, every node on both surfaces ≤ 1e-9, length
  within the step-bounded window; dx = 0.5975/0.598 still decline; true tangency still defers.
- [x] 3.2 `march_wide_band_flag_off_preserves_m1d_decline_s4c` — the default path still declines at
  dx = 0.595, and the option is inert without `adaptiveCrossReanchor`.

## 4. Gate B — native-vs-OCCT parity
- [x] 4.1 `pairWideBandIncrementalOrientationS4c` (dx = 0.595) — flag off declines, flag on crosses
  to one closed loop on the OCCT locus and both surfaces within tol.
- [x] 4.2 `pairWideBandHonestDeclineS4c` (dx = 0.598) — native declines below `minCrossSine` while
  OCCT reports a locus.
- [x] 4.3 Host Gate B green on Linux OCCT: `scripts/run-host-native-ssi-marching.sh` → 21 passed,
  0 failed (19 prior frozen + 2 new).
- [ ] 4.4 Run `run-sim-native-ssi-marching` on the booted simulator; confirm prior cases frozen and
  the two new cases pass. **(macOS-only — cannot run on the Linux host; left for the Mac.)**

## 5. Structural + finalize
- [x] 5.1 `git diff src/native` OCCT-free and additive; diff confined to `ssi/marching.{h,cpp}`;
  tessellator / boolean / blend UNTOUCHED; `cc_*` unchanged; no tolerance weakened.
- [x] 5.2 Update `openspec/MOAT-ROADMAP.md` M1 status: RETRACT the wide-band explanation as
  measurably false, record the angular law, the new floor, and the newly-exposed fit blocker.
- [x] 5.3 `openspec validate --strict moat-m1e-ssi-wide-band`.
