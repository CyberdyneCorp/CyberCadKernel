# Tasks — moat-m6n-healing-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_healing_fuzz.mm` with the shared splitmix64 → xoshiro256**
      `Rng` (keyed ONLY by `FUZZ_SEED`), reusing the proven defect builders from
      `native_heal_parity.mm` (cube/box/prism quad soups + seam/short-edge/collinear/
      missing-face generators + `nativeSoup`/`occtSoup`/`occtSoupWithPlanarCap(s)`).
- [x] 1.2 Generalize the unit-cube generators to a randomly sized axis-aligned BOX and a
      random convex N-gon PRISM, keeping the closed-form volume+area exact.
- [x] 1.3 Implement the per-trial classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE-INACCURATE / BOTH-DECLINED) arbitrated by the undamaged base solid's
      closed-form volume+area, enforcing the equal-or-more-conservative contract with
      FIXED never-widened tolerances.
- [x] 1.4 Print a per-base-family and per-defect-family coverage table; `std::_Exit(0)`
      IFF `DISAGREED == 0 && ORACLE_UNRELIABLE == 0`; report any DISAGREE with seed +
      case index + defect descriptor.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-healing-fuzz.sh` cloned from `run-sim-native-heal.sh`
      (native heal+math TUs + `occt_shapefix.cpp` oracle TU + `TKShHealing…TKernel`) with
      the ≥2-seed loop from `run-sim-native-reference-geometry-fuzz.sh` (fails if any seed
      fails; default two seeds; `FUZZ_SEED`/argv override).
- [x] 2.2 Add `native_healing_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own `main()`).

## 3. Build & gate

- [x] 3.1 `scripts/build-numsci.sh` host + iossim both exit 0 (product unchanged).
- [x] 3.2 Run the harness on the booted simulator across ≥2 seeds, N ≥ 60/seed; capture
      the coverage table; verify `DISAGREED == 0`.
- [x] 3.3 Re-run one seed twice → byte-identical batch (determinism proof).

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×13 → ×14 (add the healing
      domain entry).
- [x] 4.2 `openspec validate moat-m6n-healing-fuzz --strict` passes.
- [x] 4.3 Structural check: `git diff` touches ONLY `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`).
- [x] 4.4 Commit to branch `moat-m6n` (concise technical message, no Claude/AI mention).
