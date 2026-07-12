# Tasks — nurbs-rational-weight-estimation

## 1. Implementation (`src/native/math/bspline_fit.{h,cpp}`)

- [x] 1.1 Add `RationalFitResult` struct (curve, max/RMS error, `weightSpread`,
      `rationalityDetected`, `diagnostic`).
- [x] 1.2 Add `fitRationalCurveEstimateWeightsWithParams(points, params, knots, nCtrl, degree,
      flatTol)` — build `MᵀM` for the homogeneous bilinear system, solve the smallest-eigenvalue
      eigenvector by shifted inverse iteration through `lin_solve`, de-homogenize `Pᵢ = Hᵢ/wᵢ`,
      gauge to `w₀ = 1`.
- [x] 1.3 Add `fitRationalCurveEstimateWeights(points, nCtrl, degree, method, flatTol)` — derive
      chord/centripetal params + approximation knots, then delegate to the `WithParams` core.
- [x] 1.4 Gauge freedom: normalize the null vector so the shared weight sign is positive and
      `w₀ = 1`.
- [x] 1.5 Guards (HONEST-DECLINE with `diagnostic`): invalid dimensions, under-determined system,
      degenerate params, mismatched params / knot length, rank-deficient null space (eigenvalue
      gap test), sign-flipping / near-zero recovered weight.
- [x] 1.6 Non-rationality detection: `weightSpread ≤ flatTol ⇒ rationalityDetected = false`.

## 2. Tests (`tests/native/test_native_nurbs_fit.cpp`)

- [x] 2.1 Conic recovery — circle + ellipse recovered to ≤ 1e-8 (weights = cos 45°).
- [x] 2.2 Polynomial degenerate — flat weights (spread ≤ 1e-6), rationality not detected.
- [x] 2.3 De-homogenize consistency — recovered `(Pᵢ,wᵢ)` reproduce the input points.
- [x] 2.4 Guards — under-determined, all-coincident, mismatched params, wrong knots decline.

## 3. Invariants

- [x] 3.1 `src/native` stays OCCT-free (0 OCCT refs in changed files).
- [x] 3.2 `cc_*` ABI byte-unchanged (additive only; ABI test passes).
- [x] 3.3 Existing fit API byte-unchanged; new entry points only.
- [x] 3.4 CMake `if(`/`endif(` balance preserved; numsci block wiring intact.
- [x] 3.5 Full NURBS/numerics ctest subset green (17/17).
