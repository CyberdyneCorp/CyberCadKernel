# Tasks — trimmed-nurbs-brep-model

## 1. Data model (new module, no shape.h change)
- [x] 1.1 Create `src/native/topology/trimmed_nurbs.h` — `ParamPoint`, `PcurveSegment`, `TrimLoop`,
      `Containment`, `TrimmedNurbsFace`, and the `makeTrimmedFace` / `classify` / `pcurveFidelity`
      / `constructPcurve` declarations. Reuses the existing `shape.h` `FaceSurface` / `PCurve` /
      `EdgeCurve` storage; adds nothing to `shape.h`.
- [x] 1.2 `makeTrimmedFace(Shape)` — snapshot an existing topology face (child 0 = outer wire, rest
      = holes) into a `TrimmedNurbsFace`, resolving each edge's pcurve via `pcurveOf` with the
      single-pcurve fallback; honest `nullopt` when not a face / no surface / no usable outer loop.

## 2. Point-in-trimmed-region classification
- [x] 2.1 Local pcurve / surface / edge-curve evaluators (topology-scoped; mirror
      `tessellate::pcurveValue` + `surface_eval` but keep `topology` off `tessellate`).
- [x] 2.2 Loop flattening (drop duplicate join + closing vertices) + `loopWellFormed` self-touch /
      open-loop detection.
- [x] 2.3 `classify` — on-edge band first (`OnBoundary`), Franklin PNPOLY half-open parity, keep
      rule (inside outer AND outside every hole), honest `Unknown` for degenerate loops.

## 3. Pcurve fidelity + construction
- [x] 3.1 `pcurveFidelity` — dense `S(p(t)) == C(t)` sampling with a scale-relative tolerance;
      report max/mean deviation + the applied tolerance + worst-`t`.
- [x] 3.2 `constructPcurve` (numsci-gated) — project sampled edge points via
      `closest_point_on_surface`, decline if the edge is not on `S`, fit a 2-D B-spline
      (`interpolateCurve`), reparametrize knots onto `[first,last]`, round-trip-verify fidelity.

## 4. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 4.1 `tests/native/test_native_trimmed_nurbs.cpp` + CMake wiring (always-built gate; the
      construction leg under `CYBERCAD_HAS_NUMSCI`, mirroring `test_native_nurbs_fit`).
- [x] 4.2 Containment correctness (rect sub-region + circular hole: In/Out/OnBoundary; hole → Out).
- [x] 4.3 Pcurve fidelity (exact iso-curve ~1e-9; wrong pcurve DETECTED; planar machine-exact).
- [x] 4.4 Pcurve construction round-trip (on-S edge → fit → fidelity holds; off-surface declines).
- [x] 4.5 Degenerate guards (empty / open / self-touching / degenerate-hole → Unknown).

## 5. No-regression + docs
- [x] 5.1 Full host ctest green (esp. `test_native_step_reader`, `test_native_topology` — the
      `FaceSurface` consumers — unbroken); `cc_*` unchanged; `src/native` OCCT-free;
      `math`/`ssi`/`blend`/`boolean` untouched (only `#include`d).
- [x] 5.2 Update `docs/NURBS-SCOPE.md` §4 Layer-8 row (❌→partial).
- [x] 5.3 `openspec validate --all --strict`.
