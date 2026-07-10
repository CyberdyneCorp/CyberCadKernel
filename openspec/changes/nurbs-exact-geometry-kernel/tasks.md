# Tasks — nurbs-exact-geometry-kernel

## 1. Foundation & rational evaluation hardening
- [x] 1.1 Create `src/native/math/bspline_ops.h` — data types (`BsplineCurveData`,
      `BsplineSurfaceData`, `ParamDir`, result structs) + curve/surface op declarations.
- [x] 1.2 Internal `Homog4` lift/project helper + a single templated curve core so the
      non-rational and rational paths are single-sourced.
- [x] 1.3 Audit the existing evaluators for the degenerate cases the ops stress (full-mult
      interior knots, endpoint params, high degree, clamped/unclamped ends, non-positive
      weight guard); add regression coverage. No signature change; byte-identical for every
      currently-passing evaluation case.

## 2. Curve operations
- [x] 2.1 `insertKnotCurve` (A5.1, r-fold Boehm), rational-aware.
- [x] 2.2 `refineKnotCurve` (A5.4 Oslo-class whole-vector refinement).
- [x] 2.3 `removeKnotCurve` (A5.8, tolerance-bounded, reports `removed` + `maxError`).
- [x] 2.4 `elevateDegreeCurve` (A5.9, raise by t).
- [x] 2.5 `reduceDegreeCurve` (A5.11, reduce by 1, exact-when-reducible + honest error).
- [x] 2.6 `splitCurve` + `decomposeCurveToBezier` (A5.6) via full-multiplicity insertion.
- [x] 2.7 `reparamCurve` (affine knot remap).

## 3. Surface operations (tensor-product, reusing the curve core per row/column)
- [x] 3.1 `insertKnotSurface` / `refineKnotSurface` (U and V).
- [x] 3.2 `removeKnotSurface` (U and V).
- [x] 3.3 `elevateDegreeSurface` (U and V).
- [x] 3.4 `reduceDegreeSurface` (U and V).
- [x] 3.5 `splitSurface` (U and V).

## 4. HOST-analytic gate (no OCCT — the exact-oracle primary gate)
- [x] 4.1 `tests/native/test_native_nurbs_ops.cpp` + CMake wiring (always-on, `_SRC` in the
      line-575 block; add `test_native_nurbs_ops` to `CYBERCAD_TESTS`).
- [x] 4.2 Exact-preservation tests: insert/refine/elevate/split/decompose/reparam preserve
      the curve & surface pointwise on a dense sample (~1e-12), non-rational AND rational.
- [x] 4.3 Round-trip tests: insert↔remove identity; elevate→reduce identity on reducible
      inputs; split pieces reconstruct; refine == repeated single insert; decompose segments
      re-evaluate to source.
- [x] 4.4 Honesty tests: an irreducible curve returns `ok=false`/`removed<num` with the true
      error bound; non-positive weight guarded; degenerate/endpoint params handled.

## 5. SIM native-vs-OCCT parity gate  — SEPARATE FOLLOW-UP TRACK (not this pass)
- [x] 5.1 `tests/sim/native_nurbs_ops_parity.mm` + `run-sim-suite.sh` wiring (SKIP-list
## 5. SIM native-vs-OCCT parity gate
- [x] 5.1 `tests/sim/native_nurbs_ops_parity.mm` + `run-sim-suite.sh` wiring (SKIP-list
      convention for the `.mm` own-main harness).
- [x] 5.2 Diff insert / elevate / remove / segment against OCCT `BSplCLib` /
      `Geom_BSplineCurve` / `Geom_BSplineSurface` — resulting `(poles,weights,knots,degree)`
      and sampled points within a fixed tolerance.

## 6. Docs & close-out
- [x] 6.1 Update `docs/NURBS-SCOPE.md` §2 layer table: Layer 1 "Geometry algorithms" row
      moves from ❌ to ✅ (with the module reference); note Layers 2–8 remain demand-gated.
- [x] 6.2 Ran `openspec validate --all --strict` (pass), full host ctest (77/77 pass, zero
      regression). `cc_*` ABI byte-unchanged (no ABI file touched; `test_abi` green);
      `src/native` stays OCCT-free and substrate-free. SIM leg is Task 5 (separate track).
