# native-math

## ADDED Requirements

### Requirement: Rational B-spline curve weight estimation from unweighted data

The native math library SHALL recover, in the OCCT-free `CYBERCAD_HAS_NUMSCI`-gated fitting module
(`src/native/math/bspline_fit.{h,cpp}`), BOTH the control points AND the weights of a
degree-`degree` rational B-spline that fits `N` **unweighted** input points `Qₖ`
(`fitRationalCurveEstimateWeights(points, nCtrl, degree, method, flatTol)` and the explicit-parameter
form `fitRationalCurveEstimateWeightsWithParams(points, params, knots, nCtrl, degree, flatTol)`).

The rational fit condition `C(uₖ) = Qₖ` SHALL be expressed in its bilinear form: writing the
weighted control points `Hᵢ = wᵢ·Pᵢ`, the condition `Σᵢ Nᵢ(uₖ)·wᵢ·(Pᵢ − Qₖ) = 0` becomes the LINEAR
HOMOGENEOUS system `Σᵢ Nᵢ(uₖ)·Hᵢ − Qₖ·(Σᵢ Nᵢ(uₖ)·wᵢ) = 0` in the unknown `z = (H₀..Hₘ, w₀..wₘ)`.
Stacked over all data (3 scalar rows per datum) this is `M·z = 0`; the fit SHALL be the
smallest-singular-vector of `M`, recovered as the smallest-eigenvalue eigenvector of `MᵀM` by
shifted inverse iteration through the numerics facade `lin_solve` (no external SVD). The control net
SHALL then be de-homogenized `Pᵢ = Hᵢ/wᵢ`.

The system is homogeneous, so the routine SHALL require it to be OVER-determined (`3·N > 4·nCtrl`).
Because the weights are defined only up to a common scale (gauge freedom), the result SHALL be
normalized so `w₀ = 1`, with the shared weight sign fixed positive. The returned curve SHALL be
rational (one weight per pole) and SHALL evaluate as a rational NURBS to the data within the
reported error. When the recovered weights are all ≈ equal (`weightSpread ≤ flatTol`) the routine
SHALL report `rationalityDetected == false` (the data needs no rationality).

The routine SHALL HONEST-DECLINE (`ok = false`, `diagnostic` set, no crash, no fabricated curve)
when: the dimensions are invalid (`degree+1 ≤ nCtrl ≤ N` violated); the system is under-determined
(`3·N ≤ 4·nCtrl`); the parametrization is degenerate (all-coincident input); the explicit params
length ≠ point count or the explicit knot length ≠ `nCtrl+degree+1`; the null space is not cleanly
one-dimensional (rank-deficient — the two smallest eigenvalues are comparable, so weights are not
unique); or the recovered weights do not all share one sign / include a near-zero weight (an invalid
rational whose denominator vanishes inside the domain). The routine SHALL NOT widen any tolerance to
force a result and SHALL NOT emit a rational with a non-positive weight.

#### Scenario: A known conic is recovered exactly

- GIVEN a KNOWN rational unit circle (and, separately, an ellipse) — a quadratic NURBS with `cos(45°)` middle weights — sampled at `N ≫ nCtrl` of its own projective NURBS parameters, with those parameters and the shape's flat knot vector supplied to the explicit-parameter overload
- WHEN the weights are estimated with `nCtrl = 9`, `degree = 2`
- THEN the recovered middle weights SHALL equal `cos(45°) = √2/2` to within ~1e-8, the recovered curve SHALL lie on the true circle / ellipse everywhere (radius / implicit deviation ≤ 1e-8), `rationalityDetected` SHALL be true, and the recovered `(Pᵢ, wᵢ)` SHALL reproduce every input point to ~1e-8 (achieved: machine precision ~1e-14)

#### Scenario: Polynomial (non-rational) data yields flat weights

- GIVEN points sampled from a NON-rational polynomial B-spline, supplied with the polynomial's own parameters and knots
- WHEN the weights are estimated
- THEN the recovered weights SHALL all be ≈ equal (`weightSpread ≤ 1e-6`; achieved ~1e-13), `rationalityDetected` SHALL be false, and the fit SHALL reproduce the data to the fit tolerance — the estimator correctly detects that no rationality is needed

#### Scenario: Ill-posed or degenerate weight estimation is declined honestly

- GIVEN an under-determined request (`3·N ≤ 4·nCtrl`), all-coincident data, a mismatched explicit-parameter length, a wrong explicit-knot length, a rank-deficient (non-unique-weight) configuration, or a configuration whose recovered weights flip sign
- WHEN weight estimation is attempted
- THEN the routine SHALL return `ok = false` with a populated `diagnostic` and an empty (non-rational) result, never crashing, never widening a tolerance, and never emitting an invalid rational

### Requirement: Weight estimation is curve-only; surface weight estimation is an explicit residual

The weight-estimation routines SHALL cover the CURVE case only. The module SHALL NOT estimate the
weights of a rational tensor-product SURFACE from unweighted grid data, and SHALL NOT fabricate
surface weights. Surface rational weight estimation remains a documented residual. Rational surface
fitting continues to require PRESCRIBED weights (`interpolateRationalSurface`).

#### Scenario: The module never fakes surface weight estimation

- GIVEN a request to recover a rational SURFACE from an unweighted grid
- WHEN no per-node weights are prescribed
- THEN the module SHALL offer only curve weight estimation and prescribed-weight surface fitting — it SHALL NOT invent surface weights to present a rational surface result
