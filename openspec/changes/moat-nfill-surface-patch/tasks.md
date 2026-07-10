# Tasks — moat-nfill-surface-patch

## STATUS: IN-PROGRESS

## 1. Baseline / explore
- [x] Read the mesh/weld substrate (`boolean::assembleSolid` / `Polygon`, `tess::Mesh`,
      `tess::isWatertight` / `isConsistentlyOriented` / `enclosedVolume` / `surfaceArea`).
- [x] Read `heal/cap_hole.h` (planar cap), `heal/self_verify.h`, `heal/face_soup.h`.
- [x] Read the feature-op ABI idiom (`cc_draft_faces` / `cc_variable_sweep`), the engine
      dispatch bridge, and the sim-parity `.mm` idiom.

## 2. Native module `src/native/surface/`
- [x] `ngon_fill.h` — Coons (N=3,4) + Gregory-style (N=5,6) patch evaluated to a
      `tess::Mesh` grid; boundary rows reuse boundary samples bit-exactly; analytic-edge
      (line/arc) sampling; `NGonDecline` enum; on-boundary residual measurement.
- [x] `fill_solid.h` — free-boundary loop trace of an open shell; build patch; weld to the
      shell's `boolean::Polygon`s via `assembleSolid`; self-verify (watertight + volume +
      oriented); planar-fan fast path.
- [x] `native_surface.h` — aggregate header.

## 3. Engine + facade wiring (additive only)
- [x] `IEngine::fill_ngon` default `engine_unsupported`.
- [x] `NativeEngine::fill_ngon` — build patch mesh natively, verify boundary-coincidence,
      wrap as a mesh-backed body; else honest decline (native void never handed to OCCT).
- [x] `OcctEngine::fill_ngon` — `BRepFill_Filling` oracle (guarded by `CYBERCAD_HAS_OCCT`).
- [x] `cc_fill_ngon` in `include/cybercadkernel/cc_kernel.h` + `src/facade/cc_kernel.cpp`.

## 4. Host GATE (a) — analytic, OCCT-free
- [x] `tests/native/test_native_surfacing.cpp`:
      - planar quad hole in a box → `fillHoleSolid` watertight, χ=2, oriented, volume
        restored EXACTLY;
      - planar N-gon (3..6) → patch area = polygon area (≤ 1e-9 rel);
      - saddle 4-sided analytic boundary → on-boundary residual ≤ 1e-9, deviation bounded +
        converging as gridN doubles;
      - honest declines: non-analytic (spline) edge, 7-sided, degenerate corner.
- [x] Register `test_native_surfacing` in `CMakeLists.txt` (always-on native suite).

## 5. SIM GATE (b) — native-vs-OCCT parity
- [x] `tests/sim/native_surfacing_parity.mm` — own `main()`, builds the same boundary under
      native + OCCT `BRepFill_Filling`, compares patch area / bbox / boundary-coincidence.
- [x] `scripts/run-sim-native-surfacing.sh` runner + `run-sim-suite.sh` SKIP entry.

## 6. Discipline / finalize
- [x] `git diff src/native` OCCT-free; only the new `surface/` module + engine/facade added;
      all other modules untouched; `cc_*` additions only.
- [x] Update `openspec/MOAT-ROADMAP.md` with the new surfacing entry + the scope bound.
- [x] `openspec validate --strict`.
- [x] Commit to branch `moat-nfill` (no push).
