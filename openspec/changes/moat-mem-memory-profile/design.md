# Design — moat-mem-memory-profile

## Goal

An honest, reproducible measurement of the drop-OCCT **runtime-memory** payoff (the third leg,
after latency and binary size), added WITHOUT touching any product code (`src/native/**`,
`src/engine/**`, `include/**`, `cc_*` ABI stay byte-unchanged).

## Measurement method (`tests/sim/native_vs_occt_mem.cpp` + `scripts/bench-memory-native-vs-occt.sh`)

- **Driver:** the public `cc_*` facade under both engines via `cc_set_engine(0)` (OCCT oracle)
  / `cc_set_engine(1)` (NativeEngine) — the exact A/B pattern of the latency bench. Same fixed
  regular-N-gon prism inputs (small=octagon, medium=32-gon, large=96-gon) as the latency bench,
  for comparability. Deterministic: no `rand`, no wall-clock seeding.

- **Host, not device.** Homebrew OCCT 7.9.3 on macOS arm64, for a stable allocator and no
  simulator scheduler/thermal noise. Absolute device RAM differs; the portable signals are the
  **OCCT/native ratio** and the **process-level delta**. iOS uses the same `malloc` family so
  the direction carries. Stated in the harness header, the runner, and the doc.

- **Why `phys_footprint` / `ru_maxrss`, not malloc-zone stats.** On macOS arm64 large
  allocations are `mmap`-backed and do NOT appear in `malloc_zone_statistics` `size_in_use`
  (verified empirically — an 8 MB `malloc` moved `size_in_use` by 0). A malloc-zone / `mstats`
  delta would therefore *under-count* geometry working sets. `phys_footprint`
  (`task_info(TASK_VM_INFO)` — what iOS charges the process, the Xcode memory-gauge value) and
  `ru_maxrss` (peak resident) count them, so they are the honest signals.

- **Three complementary signals, each labelled:**
  1. **Per-op PEAK RSS** (`ru_maxrss`) — a per-process high-water mark that never falls, so it
     is measured **one (op,size,engine) per process invocation**. Each process does a bounded
     warm-up build (fault in code pages) then the measured op, so `ru_maxrss` at exit is a
     **clean peak** for that engine + op, with no contamination from a prior op. This is the
     **primary, reproducible** signal (±0.1–0.3 MB native, ±0.5 MB OCCT across runs).
  2. **Per-op FOOTPRINT DELTA** — `phys_footprint` sampled just before the op and at its **live
     peak** (result still alive), reported as a KB delta that subtracts the fixed baseline.
     Cleanest per-op engine-to-engine comparison in principle, but **noisy** (page-quantized;
     transient alloc+free inside the sample window reads a smaller/zero delta) — labelled a
     **supporting** signal, with peak RSS as the headline.
  3. **Process-level PEAK RSS** — the whole representative script (construct + all three
     booleans + tessellate + mass + section, all sizes) run once per engine in a fresh process;
     captures each engine's full working set + (for OCCT) static/global footprint faulted in.

- **Honest served-labelling** (same rule as the latency bench): a native op the NativeEngine
  FORWARDS to OCCT (curved fillet) gets NO native number and is labelled `FORWARDED`; an op the
  OCCT adapter does not implement (`section`) is detected at runtime and labelled `DECLINED`
  (OCCT side) / `native-only` (native side). No forwarded/unavailable op is presented as a win.

- **Link:** the whole kernel (facade + core + engine[native+occt] + `src/native/**`) against
  the Homebrew OCCT toolkit set from the macOS desktop recipe PLUS `-lTKHLR` — the identical
  measurement-only link as `bench-native-vs-occt.sh`. No product CMake is modified.

## Why this is honest, not a memory-gamed number

- Same inputs, same process entry points for both engines; boolean results are the same
  self-verified-correct shapes as the latency bench (identical fuse volume to the OCCT oracle).
- The primary signal (peak RSS) is reproducible and reported alongside the noisy footprint
  delta, which is explicitly labelled supporting; the one place the delta reads 0
  (`mass_properties` under OCCT) is called out as a page-granularity artifact, with peak RSS
  showing the real (still native-lower) picture.
- Forwarded / native-only ops get no fake ratio; the process-level number is stated as the
  single most honest absolute figure.

## Non-goals

- Not a device benchmark (host only; the ratio + process delta are the portable signals). Not a
  CTest-integrated gate (a non-shipping measurement runner with its own `main()`). No product
  behavior / ABI / CMake change. No leak-hunt (if a measurement surfaced a real leak it would be
  reported, not fixed here — none did: footprints settle at a stable high-water per op).
