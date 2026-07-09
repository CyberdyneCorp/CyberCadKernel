# Tasks — moat-m1c-ssi-tail

## 1. Diagnose (host, OCCT-free)
- [x] 1.1 Reproduce the M1b decline: probe general cone∩cone, off-axis cyl∩cone, off-axis
  sphere∩cyl with FINITE native adapters across seed densities.
- [x] 1.2 Confirm the twice-piercing sphere∩cyl has TWO ground-truth loops (dense grid), and that
  the coarse fixed grid finds only ONE (`refinedAccepted == 1`, `numClusters == 1`) → root cause is
  a topological-cluster OVER-MERGE, not a subdivision or refine miss (LM converges to both loops
  from coarse starts).

## 2. Fix 2 — seeding-recall bump (src/native/ssi, additive, default-off)
- [x] 2.1 Add `SeedOptions::criticTargetedReseed` (+ `criticMaxCells`), both DEFAULT OFF.
- [x] 2.2 In the S4-f critic (`marching.cpp`), when `criticTargetedReseed` is set, re-seed only the
  uncovered A-cells (A clamped to each cell) vs B's full domain, accumulating recovered seeds.
- [x] 2.3 Prove non-regression: every prior SSI host suite passes UNCHANGED with the flags default
  off (ssi 11, s4_classification 22, seeding 9, marching 16→19 with the new cases, s4e 7, s4f 6).

## 3. Fix 1 — domain-clipped oracle (test harness only)
- [x] 3.1 Add `clipOracle` (`Geom_RectangularTrimmedSurface` to the native ParamBox) to
  `native_ssi_marching_parity.mm`.

## 4. Gate A — host self-consistency (`test_native_ssi_marching.cpp`)
- [x] 4.1 `march_cone_cone_general_single_loop` — 1 closed loop, every node on both cones ≤ 1e-9.
- [x] 4.2 `march_cyl_cone_offaxis_open_arc` — 1 open (BoundaryExit) arc, every node on both ≤ 1e-9.
- [x] 4.3 `march_sphere_cyl_twice_piercing_two_loops` — baseline finds 1 loop; the recall bump
  recovers the second → 2 closed loops, `criticRecoveredLoops ≥ 1`, every node on both ≤ 1e-9.

## 5. Gate B — sim native-vs-OCCT parity (`native_ssi_marching_parity.mm`)
- [x] 5.1 `pairConeConeGeneral` (clipped oracle) — branches 1/1, closed 1/1 vs `GeomAPI_IntSS`.
- [x] 5.2 `pairCylConeOffAxis` (clipped oracle) — branches 1/1, closed 0/0 (open arc) vs oracle.
- [x] 5.3 `pairSphereCylTwicePiercing` (clipped oracle + recall bump) — branches 2/2, closed 2/2.
- [x] 5.4 Register all three in `main()`; confirm all 14 prior cases stay green (17 passed, 0 failed).

## 6. Finalize
- [x] 6.1 Spec delta: one ADDED requirement in `native-ssi` (declined tail promoted to verified).
- [x] 6.2 `openspec validate moat-m1c-ssi-tail --strict` passes.
- [x] 6.3 Update `openspec/MOAT-ROADMAP.md` M1 status.
- [x] 6.4 Structural check: `git diff src/native` OCCT-free + additive; tessellator + boolean
  untouched; `cc_*` unchanged. Commit to branch moat-m1c.
