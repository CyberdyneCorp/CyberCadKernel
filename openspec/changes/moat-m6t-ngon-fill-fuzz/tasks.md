# Tasks — moat-m6t-ngon-fill-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_ngon_fill_fuzz.mm` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), a minimal 3-D vector kit, the exact
      Newell 3-D polygon-area closed form, and the analytic-boundary residual self-check
      (straight = exact; arc = on-circle by construction).
- [x] 1.2 Implement the four families × N∈{3,4,5,6}: `planar-Ngon` (coplanar straight loop
      in a RANDOM plane), `planar-hole-completion` (coplanar straight regular loop in a
      COORDINATE plane — the box-face-restore pose), `saddle-4sided` (NON-planar loop,
      corners alternating ±h in the small-warp regime), `arc-boundary` (loop with ≥1
      circular-arc side). Build the POD facade arrays (boundaryXYZ / edgeKinds / arcMids).
- [x] 1.3 Drive the SAME loop through the public `cc_fill_ngon` facade under BOTH engines
      (`cc_set_engine(1)`=NativeEngine Coons/Gregory patch, `cc_set_engine(0)`=OCCT
      `BRepFill_Filling`); measure patch AREA via `cc_mass_properties` (read `.area`
      directly — a patch is an OPEN surface, so `.valid` is 0 by design) and bbox via
      `cc_bounding_box`.
- [x] 1.4 Implement the six-way classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED): planar families arbitrated by
      the EXACT polygon area (bands planarX ≤ 1e-4, planarO ≤ 1e-3); non-planar families by
      OCCT area (≤ 1.2e-1) + bbox-containment (≤ 8e-2) + boundary residual (≤ 1e-6). FIXED,
      never widened. Add a sparse (every-11th, N≥4) self-intersecting bowtie decline probe
      exercising the native NULL → OCCT branch (N=3 degenerate skipped — it crashes the
      OCCT oracle, an oracle-robustness fragility outside the native scope).
- [x] 1.5 Print a per-family coverage table; `std::_Exit(0)` IFF `DISAGREED == 0 &&
      ORACLE_UNRELIABLE == 0` with each of the four families ≥1 AGREED; report any
      DISAGREE / ORACLE-INACCURATE with seed + case index + family/N/param tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-ngon-fill-fuzz.sh` cloned from
      `run-sim-native-draft-faces-fuzz.sh` (whole kernel + full OCCT toolkit set,
      `TKHLR`/`TKShHealing` retained), WITHOUT the numsci substrate (the native fill path is
      not `CYBERCAD_HAS_NUMSCI`-gated), seeded ONLY by `FUZZ_SEED`/argv (default N=72), runs
      TWO default seeds and fails if either fails.
- [x] 2.2 Add `native_ngon_fill_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
      `main()`, `std::_Exit`).

## 3. Build & gate

- [x] 3.1 `scripts/build-numsci.sh` host + iossim both exit 0 (product unchanged).
- [x] 3.2 Run the harness on the booted simulator across 2 seeds, N = 72/seed; capture
      the coverage table; verify `DISAGREED == 0` and `ORACLE_UNRELIABLE == 0` on both.
      (0xF117A11FEE → 68 AGREED / 0 DISAGREED / 0 ORACLE_UNRELIABLE / 4 BOTH-DECLINED;
      0x5EEDF111A6 → 68 / 0 / 0 / 4 — every planar AGREE native==OCCT==closed-form to ~6
      digits.)
- [x] 3.3 Re-run seed 0xF117A11FEE twice → byte-identical batch (determinism proof).

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×17 → ×18 (add the N-fill
      domain entry; reconcile to ×20 at merge with the concurrent freeform-boolean +
      variable-sweep tracks).
- [x] 4.2 `openspec validate moat-m6t-ngon-fill-fuzz --strict` passes.
- [x] 4.3 Structural check: `git diff` touches ONLY `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`).
- [x] 4.4 Commit to branch `moat-m6t` (concise technical message, no Claude/AI mention).
