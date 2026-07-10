# Tasks — nurbs-exact-geometry-kernel

## 1. Foundation & rational evaluation hardening
- [ ] 1.1 Create `src/native/math/bspline_ops.h` — data types (`BsplineCurveData`,
      `BsplineSurfaceData`, `ParamDir`, result structs) + curve/surface op declarations.
- [ ] 1.2 Internal `Homog4` lift/project helper + a single templated curve core so the
      non-rational and rational paths are single-sourced.
- [ ] 1.3 Audit the existing evaluators for the degenerate cases the ops stress (full-mult
      interior knots, endpoint params, high degree, clamped/unclamped ends, non-positive
      weight guard); add regression coverage. No signature change; byte-identical for every
      currently-passing evaluation case.

## 2. Curve operations
- [ ] 2.1 `insertKnotCurve` (A5.1, r-fold Boehm), rational-aware.
- [ ] 2.2 `refineKnotCurve` (A5.4 Oslo-class whole-vector refinement).
- [ ] 2.3 `removeKnotCurve` (A5.8, tolerance-bounded, reports `removed` + `maxError`).
- [ ] 2.4 `elevateDegreeCurve` (A5.9, raise by t).
- [ ] 2.5 `reduceDegreeCurve` (A5.11, reduce by 1, exact-when-reducible + honest error).
- [ ] 2.6 `splitCurve` + `decomposeCurveToBezier` (A5.6) via full-multiplicity insertion.
- [ ] 2.7 `reparamCurve` (affine knot remap).

## 3. Surface operations (tensor-product, reusing the curve core per row/column)
- [ ] 3.1 `insertKnotSurface` / `refineKnotSurface` (U and V).
- [ ] 3.2 `removeKnotSurface` (U and V).
- [ ] 3.3 `elevateDegreeSurface` (U and V).
- [ ] 3.4 `reduceDegreeSurface` (U and V).
- [ ] 3.5 `splitSurface` (U and V).

## 4. HOST-analytic gate (no OCCT — the exact-oracle primary gate)
- [ ] 4.1 `tests/native/test_native_nurbs_ops.cpp` + CMake wiring (always-on, `_SRC` in the
      line-575 block; add `test_native_nurbs_ops` to `CYBERCAD_TESTS`).
- [ ] 4.2 Exact-preservation tests: insert/refine/elevate/split/decompose/reparam preserve
      the curve & surface pointwise on a dense sample (~1e-12), non-rational AND rational.
- [ ] 4.3 Round-trip tests: insert↔remove identity; elevate→reduce identity on reducible
      inputs; split pieces reconstruct; refine == repeated single insert; decompose segments
      re-evaluate to source.
- [ ] 4.4 Honesty tests: an irreducible curve returns `ok=false`/`removed<num` with the true
      error bound; non-positive weight guarded; degenerate/endpoint params handled.

## 5. SIM native-vs-OCCT parity gate
- [ ] 5.1 `tests/sim/native_nurbs_ops_parity.mm` + `run-sim-suite.sh` wiring (SKIP-list
      convention for the `.mm` own-main harness).
- [ ] 5.2 Diff insert / elevate / remove / segment against OCCT `BSplCLib` /
      `Geom_BSplineCurve` / `Geom_BSplineSurface` — resulting `(poles,weights,knots,degree)`
      and sampled points within a fixed tolerance.

## 6. Docs & close-out
- [ ] 6.1 Update `docs/NURBS-SCOPE.md` §2 layer table: Layer 1 "Geometry algorithms" row
      moves from ❌ to ✅ (with the module reference); note Layers 2–8 remain demand-gated.
- [ ] 6.2 Run `openspec validate --all --strict`, full host ctest, and the SIM leg; record
      results. No `cc_*` ABI change; `src/native` stays OCCT-free.
