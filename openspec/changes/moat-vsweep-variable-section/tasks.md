# Tasks — moat-vsweep-variable-section (variable-section / guide+spine sweep)

Order: substrate → variable-sweep builder → ABI + engine dispatch + additive facade →
OCCT oracle → host analytic gate (a) → sim native-vs-OCCT gate (b) → docs. All new native
code is header-only, OCCT-free, host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::construct`. The landed sweep/loft substrate is byte-frozen; `cc_*`
additive-only. No tolerance weakened; a measured decline (mismatched counts / non-planar
guided spine / coincident or collapsing guide / self-fold) is a first-class outcome.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh host` (exit 0). iossim substrate is present from
      the landed tracks; the sim harness compiles the kernel fresh vs the OCCT sim install.

## 1. Native variable-sweep verb

- [x] 1.1 `src/native/construct/sweep.h` — `build_variable_sweep` morphs A→B along the
      spine, guide-scaled, reusing the landed frame/tile substrate. NO-guide path forwards
      to `build_loft_along_rail` byte-identically; the guided path runs the new
      `build_variable_sweep_tube` (RMF/perp guide-scaled morph + `sectionSweepUnsafe`
      self-fold guard + `assembleRingTube`). Declines mismatched counts / non-planar guided
      spine / coincident-or-collapsing guide / degenerate input → NULL.
- [x] 1.2 Doc the new builder in `src/native/construct/native_construct.h`.

## 2. ABI + engine dispatch

- [x] 2.1 Additive `cc_variable_sweep` in `include/cybercadkernel/cc_kernel.h`;
      `IEngine::variable_sweep` default `engine_unsupported`; facade wiring in
      `src/facade/cc_kernel.cpp`.
- [x] 2.2 `NativeEngine::variable_sweep` → native (self-verified robustly watertight +
      positive volume), honest decline → OCCT.
- [x] 2.3 `OcctEngine::variable_sweep` = the `BRepOffsetAPI_MakePipeShell` MULTI-SECTION
      oracle (per-station morphed+scaled section wires), built + linked on the OCCT sim.

## 3. Gate (a) — host analytic (OCCT-free)

- [x] 3.1 `tests/native/test_native_vsweep.cpp` — circle→circle straight = truncated cone
      (`πH/3·(r0²+r0r1+r1²)`); constant section = prism; guide-scaled square = frustum;
      curved-arc morph watertight + χ=2 + volume-stable; honest declines. Registered in
      `CMakeLists.txt` (`test_native_vsweep`). 8/8 pass.

## 4. Gate (b) — sim native-vs-OCCT parity

- [x] 4.1 `tests/sim/native_vsweep_parity.mm` + `scripts/run-sim-native-vsweep.sh`
      (TKHLR added to the toolkit link set) + `run-sim-suite.sh` SKIP entry. Native vs OCCT
      `MakePipeShell` multi-section + `BRepGProp` on a booted simulator. 13/13 pass
      (straight morph rel≈1e-14, arc morph rel≈4.4e-4 within the deflection bound, non-
      planar guided morph a verified OCCT fall-through).

## 5. Docs + structural discipline

- [x] 5.1 `openspec/MOAT-ROADMAP.md` — new "construct — variable-section sweep" entry.
- [x] 5.2 `git diff src/native` OCCT-free & additive; tessellator + boolean + blend +
      analysis + exchange UNTOUCHED; `cc_*` additions only. Commit to branch `moat-vsweep`.
