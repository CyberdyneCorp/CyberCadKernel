# Tasks — nurbs-rational-surface-weight-estimation

## 1. Implementation (`src/native/math/bspline_fit.{h,cpp}`)

- [x] 1.1 Add `RationalSurfaceFitResult` struct (surface, max/RMS error, `weightSpread`,
      `rationalityDetected`, `diagnostic`).
- [x] 1.2 Add `fitRationalSurfaceEstimateWeightsWithParams(grid, uParams, vParams, knotsU, knotsV,
      nCtrlU, nCtrlV, degreeU, degreeV, flatTol)` — build `MᵀM` for the homogeneous bilinear tensor
      system (unknown layout `[Hx|Hy|Hz|w]`, control `(i,j)` at flat `i·nCtrlV+j`), solve the
      smallest-eigenvalue eigenvector by shifted inverse iteration through `lin_solve` (reusing the
      curve's `smallestEigvec`), de-homogenize `Pᵢⱼ = Hᵢⱼ/wᵢⱼ`, gauge to `w₀₀ = 1`.
- [x] 1.3 Add `fitRationalSurfaceEstimateWeights(grid, nCtrlU, nCtrlV, degreeU, degreeV, method,
      flatTol)` — derive averaged U/V params + approximation knots, then delegate to the
      `WithParams` core.
- [x] 1.4 Gauge freedom: normalize the null vector so the shared weight sign is positive and
      `w₀₀ = 1`.
- [x] 1.5 Guards (HONEST-DECLINE with `diagnostic`): invalid dimensions, under-determined system
      (`3·nU·nV ≤ 4·nCtrlU·nCtrlV`), degenerate params, mismatched params / knot length,
      rank-deficient null space (eigenvalue gap test), sign-flipping / near-zero recovered weight.
- [x] 1.6 Non-rationality detection: `weightSpread ≤ flatTol ⇒ rationalityDetected = false`.

## 2. Tests (`tests/native/test_native_nurbs_fit.cpp`)

- [x] 2.1 Quadric recovery — quarter-cylinder recovered to ≤ 1e-8 (middle weights = cos 45°,
      surface on the true cylinder), achieved ~1e-15.
- [x] 2.2 Polynomial degenerate — biquadratic patch yields flat weights (spread ≤ 1e-6),
      rationality not detected.
- [x] 2.3 De-homogenize consistency — recovered `(Pᵢⱼ,wᵢⱼ)` reproduce the input grid to ≤ 1e-8.
- [x] 2.4 Guards — under-determined, all-coincident, mismatched params, wrong knots decline.

## 3. Invariants

- [x] 3.1 `src/native` stays OCCT-free (0 OCCT refs in changed files).
- [x] 3.2 `cc_*` ABI byte-unchanged (additive only; ABI test passes).
- [x] 3.3 Numsci facade NOT widened (same `lin_solve` shifted-inverse-iteration path as the curve).
- [x] 3.4 No widened tolerances; honest-decline with diagnostic on rank-deficient / sign-flip.
