# Tasks — moat-m6u-sheetmetal-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_sheetmetal_fuzz.mm` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), the exact shoelace profile-area closed
      form (mirroring `sheetmetal::signedArea`), and the fold→unfold area-invariant residual
      (the additive decomposition vs the direct developed area).
- [x] 1.2 Implement the three ops: BASE flange (random rectangle / regular n-gon /
      convex-jittered n-gon profile × random thickness); EDGE flange (random `L×W×t` base
      flanged off its +X straight rim, random bend radius / angle / wall height, rim edge id
      probed deterministically); UNFOLD (the flat pattern of that folded part at a random
      k-factor ∈[0,1], paired with the fold so it develops a REAL folded body).
- [x] 1.3 Drive each op through the public `cc_sheet_base_flange` / `cc_sheet_edge_flange` /
      `cc_sheet_unfold` facade under the NATIVE engine (`cc_set_engine(1)`) only — there is
      NO OCCT sheet-metal oracle. Read the volume via `cc_mass_properties` and the geometric
      validity (closed_manifold ∧ consistent_orientation ∧ no_degenerate ∧ finite ∧ watertight
      ∧ vol>0) via `cc_check_solid`'s per-check breakdown, recording GS6 `no_self_intersection`
      separately.
- [x] 1.4 Implement the classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      NATIVE-CHECK-INACCURATE): base + unfold arbitrated by the EXACT planar-prism volume
      (band ≤ 1e-6) + the fold→unfold area invariant (residual ≤ 1e-6); the fold arbitrated by
      the closed-form volume converging from below (band ≤ 1.5%, must not exceed) + validity.
      FIXED, never widened. Out-of-slice decline probes (wrong/non-straight edge id,
      degenerate profile, angle outside (0°,180°), unfold of a non-fold body) exercise the
      native NULL branch.
- [x] 1.5 Print a per-op coverage table; `std::_Exit(0)` IFF `DISAGREED == 0` with each of
      the three ops ≥1 geometrically-correct trial (AGREED or NATIVE-CHECK-INACCURATE); report
      any DISAGREE with seed + case index + the L/W/t/r/h/θ (or k-factor) tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-sheetmetal-fuzz.sh` cloned from
      `run-sim-native-sheetmetal.sh` + the two-seed loop of `run-sim-native-ngon-fill-fuzz.sh`
      (whole kernel + OCCT toolkit set for the facade's `create_default_engine`,
      `TKHLR`/`TKShHealing` retained; the harness never enters an OCCT path), WITHOUT the
      numsci substrate (the sheet-metal path is not `CYBERCAD_HAS_NUMSCI`-gated), seeded ONLY
      by `FUZZ_SEED`/argv (default N=72), runs TWO default seeds and fails if either fails.
- [x] 2.2 Add `native_sheetmetal_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own `main()`,
      `std::_Exit`).

## 3. Build & gate

- [x] 3.1 `scripts/build-numsci.sh` host + iossim both exit 0 (product unchanged).
- [x] 3.2 Run the harness on the booted simulator across 2 seeds, N = 72/seed; capture the
      coverage table; verify `DISAGREED == 0` on both.
      (0x5EE7EA1F00 → 40 AGREED / 14 HONESTLY-DECLINED / 0 DISAGREED / 18 NATIVE-CHECK-INACCURATE;
      0xB3ADF01DCC → 40 / 14 / 0 / 18 — every base + unfold AGREE native==closed-form to
      ~1e-16, every folded part vol==closed-form within the 1.5% band, area invariant residual
      ≤ ~5e-13.)
- [x] 3.3 Localize the fold `cc_check_solid` result: `firstFail=5` (GS6 `no_self_intersection`)
      while closed/oriented/χ=2/watertight/volume all hold — a PRE-EXISTING GS6-vs-tessellated-
      cylinder false positive (the landed `native_sheetmetal_selftest.mm` `edge_flange
      cc_check_solid valid` FAILs on the base commit too). REPORTED, not fixed;
      NATIVE-CHECK-INACCURATE, never a bar DISAGREE.

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×20 → ×21 (add the sheet-metal
      domain entry; the concurrent wrap-emboss fuzzer also bumps it — reconcile at merge).
- [x] 4.2 `openspec validate moat-m6u-sheetmetal-fuzz --strict` passes.
- [x] 4.3 Structural check: `git diff` touches ONLY `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`).
- [x] 4.4 Commit to branch `moat-m6u` (concise technical message, no Claude/AI mention).
