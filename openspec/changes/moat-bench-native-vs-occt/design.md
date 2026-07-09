# Design — moat-bench-native-vs-occt

## Goal

Two honest, reproducible measurements of the drop-OCCT payoff, added WITHOUT touching any
product code (`src/native/**`, `src/engine/**`, `include/**`, `cc_*` ABI stay byte-unchanged).

## Latency measurement (`tests/sim/native_vs_occt_bench.cpp` + `scripts/bench-native-vs-occt.sh`)

- **Driver:** the public `cc_*` facade under both engines via `cc_set_engine(0)` (OCCT
  oracle) / `cc_set_engine(1)` (NativeEngine) — the exact A/B pattern of
  `native_boolean_parity.mm`. No internal API is reached; the harness times what the app runs.
- **Host, not device.** Homebrew OCCT 7.9.3 on macOS arm64. A booted simulator adds
  scheduler/thermal noise; the clean engine-to-engine signal is the RATIO on the same host
  CPU. This is stated in the harness header, every report line, and the doc.
- **Determinism:** fixed regular-N-gon prism inputs (no `rand`, no wall-clock seeding);
  median of N=25 measured iterations after 3 discarded warm-ups; min/max captured;
  `steady_clock`; `-O2`.
- **Model-size spread:** octagon (small) / 32-gon (medium) / 96-gon (large) so the ratio is
  not a single point.
- **Honest served-labelling.** The NativeEngine forwards out-of-domain ops to OCCT; timing a
  forwarded op "under native" would re-time OCCT, so forwarded ops get NO native time and are
  labelled `forwarded`. The OCCT adapter does not implement `cc_section_plane` — the harness
  probes for that decline at runtime and labels the OCCT side `DECLINED` and the native side
  `native-only`, printing no bogus ratio.
- **Link:** the whole kernel (facade + core + engine[native+occt] + `src/native/**`) against
  the Homebrew OCCT toolkit set from the macOS desktop recipe PLUS `-lTKHLR` (the desktop
  dylib recipe omits it, but `occt_drafting.cpp`'s HLR projection needs it). This is a
  measurement link only — no product CMake is modified.

## Size measurement (`scripts/bench-binary-size.sh`)

- **Two kernel archives, controlled delta.** Compile the same `src/**` TU set (facade + core
  + engine + native + the numsci numeric facade) for iossim arm64 twice, differing ONLY by
  the OCCT adapter: `-DCYBERCAD_HAS_OCCT` (adds `src/engine/occt/*`, 16 TUs) vs
  `-DCYBERCAD_M8_REHEARSAL` (no OCCT TUs, native default — the post-unlink wiring).
- **OCCT footprint, three honest ways:** the LINKED-toolkit subset (20 `TK*.a` incl. TKHLR —
  what the app pulls in), the FULL trimmed install (upper bound; the app links a subset), and
  a DEAD-STRIPPED final-link delta (link a small exe exercising a representative reachable set
  — construct+boolean+tessellate+mass+section+fillet+STEP+query — against each kernel with
  `-dead_strip`, compare stripped exe sizes). The dead-stripped delta is the truest in-binary
  "what ships" number and is smaller than the archive subset; both are reported and the
  difference explained.
- **Elimination counts:** OCCT adapter TUs (16), OCCT-side symbols in the OCCT kernel `.a`
  (`nm`), and archive members in the linked subset.

## Why this is honest, not a benchmark-gamed number

- Same inputs, same process, same public entry points for both engines; correctness is
  cross-checked (native BSP-CSG fuse volume == OCCT fuse volume to fp round-off).
- Ops the native engine does NOT accelerate are labelled `forwarded` / `native-only` and get
  no fake ratio.
- The size story distinguishes on-disk archive footprint (112/140 MB) from the dead-stripped
  in-binary delta (28 MB) rather than quoting the biggest number.

## Non-goals

- Not a device benchmark (host only; device absolute latency is out of scope, ratio is the
  signal). Not a CTest-integrated gate (these are non-shipping measurement runners with their
  own `main()`). No product behavior/ABI/CMake change.
