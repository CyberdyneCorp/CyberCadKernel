# Tasks — moat-bench-native-vs-occt

## 1. Latency harness
- [x] 1.1 `tests/sim/native_vs_occt_bench.cpp` — drive boolean/tessellate/mass_properties/
      section/fillet_edges through `cc_*` under `cc_set_engine(0|1)`; median-of-25, 3 warm-up
      discarded, fixed N-gon inputs, `steady_clock`, `-O2`; honest served-labelling
      (forwarded → no native time; OCCT-declined section → native-only).
- [x] 1.2 `scripts/bench-native-vs-occt.sh` — host build + run against Homebrew OCCT, whole
      kernel + OCCT adapter + `-lTKHLR`; prints `[ROW]` lines + a summary table.

## 2. Binary-size runner
- [x] 2.1 `scripts/bench-binary-size.sh` — two iossim arm64 kernel `.a` builds (OCCT vs
      `CYBERCAD_M8_REHEARSAL` native-only), kernel `.a` delta, OCCT linked-subset + full-
      install footprint, dead-stripped final-link delta, OCCT TUs/symbols eliminated; prints
      `[SIZEROW]` lines + a table.

## 3. Findings + roadmap
- [x] 3.1 `docs/BENCH-native-vs-occt.md` — methodology + the latency table + the size table +
      the honest conclusion (wins / native-only / forwarded-declines).
- [x] 3.2 `openspec/MOAT-ROADMAP.md` — short "drop-OCCT payoff (measured)" note referencing
      the numbers.

## 4. Discipline / gates
- [x] 4.1 `scripts/build-numsci.sh host` and `iossim` both exit 0 (substrate for the size
      build).
- [x] 4.2 Structural check: `git diff` touches ONLY `tests/`, `scripts/`, `docs/`, `openspec/`
      — NOT `src/native`, `src/engine`, `include`, or the `cc_*` ABI (byte-unchanged).
- [x] 4.3 Determinism: no `Date.now()` / `rand()` / `Math.random()` in any committed harness;
      fixed inputs, median-of-N, variance (min/max) reported.
