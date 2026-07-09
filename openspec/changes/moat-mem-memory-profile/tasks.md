# Tasks — moat-mem-memory-profile

## 1. Memory harness
- [x] 1.1 `tests/sim/native_vs_occt_mem.cpp` — drive boolean/tessellate/mass_properties/
      section/fillet_edges through `cc_*` under `cc_set_engine(0|1)`; SAME fixed N-gon inputs
      as the latency bench; measure per-op peak RSS (`ru_maxrss`) + per-op `phys_footprint`
      delta (sampled at the op's live peak) + a whole-process peak RSS; `--single` / `--script`
      / `--list` modes; honest served-labelling (forwarded → no native number; OCCT-declined
      section → native-only). No `rand()` / no wall-clock seeding.
- [x] 1.2 `scripts/bench-memory-native-vs-occt.sh` — compile the harness once (whole kernel +
      OCCT adapter + `-lTKHLR`, same measurement-only link as the latency runner); invoke it
      once per (op,size,engine) for a clean per-op high-water mark and once per engine for the
      process-level number; scrape `[MEMROW]` / `[PROCROW]` into per-op + process tables.

## 2. Findings + pointers
- [x] 2.1 `docs/BENCH-memory-native-vs-occt.md` — methodology (what is counted; why malloc-zone
      stats are rejected for `phys_footprint`/`ru_maxrss`) + per-op table + process-level table
      + honest conclusion (wins / comparable / native-only / forwarded).
- [x] 2.2 Pointer from `docs/BENCH-native-vs-occt.md` (top table + companion link).
- [x] 2.3 `openspec/MOAT-ROADMAP.md` — short "drop-OCCT payoff — memory (measured)" note
      referencing the numbers.

## 3. Discipline / gates
- [x] 3.1 `scripts/build-numsci.sh host` and `iossim` both exit 0.
- [x] 3.2 Runner exits 0 and produces the per-op table, the process-level table, and the
      process peak-RSS ratio.
- [x] 3.3 Structural check: `git diff` touches ONLY `tests/`, `scripts/`, `docs/`, `openspec/`
      — NOT `src/native`, `src/engine`, `include`, or the `cc_*` ABI (byte-unchanged).
- [x] 3.4 Determinism: no `rand()` / no wall-clock seeding in the committed harness; fixed
      inputs; per-op isolation (one op per process) to avoid high-water contamination;
      reproducibility of the primary signal (peak RSS) noted in the doc.
