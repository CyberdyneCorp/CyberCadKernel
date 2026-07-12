# native-math

## ADDED Requirements

### Requirement: Rational tensor-product surface weight estimation from unweighted grid data

The native math library SHALL recover, in the OCCT-free `CYBERCAD_HAS_NUMSCI`-gated fitting module
(`src/native/math/bspline_fit.{h,cpp}`), BOTH the control net AND the weights of a
degree-`(degreeU,degreeV)` rational tensor-product B-spline surface that fits an **unweighted**
grid of input points `Q(k,l)` (`fitRationalSurfaceEstimateWeights(grid, nCtrlU, nCtrlV, degreeU,
degreeV, method, flatTol)` and the explicit-parameter form
`fitRationalSurfaceEstimateWeightsWithParams(grid, uParams, vParams, knotsU, knotsV, nCtrlU, nCtrlV,
degreeU, degreeV, flatTol)`).

The tensor rational fit condition `S(uₖ,vₗ) = Q(k,l)` SHALL be expressed in its bilinear form:
writing the weighted control net `Hᵢⱼ = wᵢⱼ·Pᵢⱼ`, the condition
`Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·wᵢⱼ·(Pᵢⱼ − Q(k,l)) = 0` becomes the LINEAR HOMOGENEOUS system
`Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·Hᵢⱼ − Q(k,l)·(Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·wᵢⱼ) = 0` in the unknown `z = (Hᵢⱼ, wᵢⱼ)`.
Stacked over the whole grid (3 scalar rows per datum) this is `M·z = 0`; the fit SHALL be the
smallest-singular-vector of `M`, recovered as the smallest-eigenvalue eigenvector of `MᵀM` by the
same shifted inverse iteration through the numerics facade `lin_solve` as the curve case (no
external SVD, the numsci facade SHALL NOT be widened). The control net SHALL then be de-homogenized
`Pᵢⱼ = Hᵢⱼ/wᵢⱼ`.

The system is homogeneous, so the routine SHALL require it to be OVER-determined
(`3·nU·nV > 4·nCtrlU·nCtrlV`). Because the weights are defined only up to a common scale (gauge
freedom), the result SHALL be normalized so `w₀₀ = 1`, with the shared weight sign fixed positive.
The returned surface SHALL be rational (one weight per pole) and SHALL evaluate as a rational NURBS
to the data within the reported error. When the recovered weights are all ≈ equal
(`weightSpread ≤ flatTol`) the routine SHALL report `rationalityDetected == false` (the data needs
no rationality).

The routine SHALL HONEST-DECLINE (`ok = false`, `diagnostic` set, no crash, no fabricated surface)
when: the dimensions are invalid (`degree+1 ≤ nCtrl ≤ nGrid` violated in either direction); the
system is under-determined (`3·nU·nV ≤ 4·nCtrlU·nCtrlV`); the parametrization is degenerate
(all-coincident grid line); the explicit params length ≠ grid dimension or the explicit knot length
≠ `nCtrl+degree+1` in either direction; the null space is not cleanly one-dimensional
(rank-deficient — the two smallest eigenvalues are comparable, so weights are not unique); or the
recovered weights do not all share one sign / include a near-zero weight (an invalid rational whose
denominator vanishes inside the domain). The routine SHALL NOT widen any tolerance to force a result
and SHALL NOT emit a rational with a non-positive weight.

#### Scenario: A known quadric patch is recovered exactly

- GIVEN a KNOWN rational quarter-cylinder patch — degree `(2,1)`, circular U direction with `cos(45°)` middle-arc weight — sampled on a dense `nU × nV` grid at its own projective NURBS parameters, with those parameters and the patch's flat knot vectors supplied to the explicit-parameter overload
- WHEN the weights are estimated with `nCtrlU = 3`, `nCtrlV = 2`, `degreeU = 2`, `degreeV = 1`
- THEN the recovered middle-arc weights SHALL equal `cos(45°) = √2/2` to within ~1e-8, the recovered surface SHALL lie on the true unit-radius cylinder everywhere (radius deviation ≤ 1e-8), `rationalityDetected` SHALL be true, and the recovered `(Pᵢⱼ, wᵢⱼ)` SHALL reproduce every grid point to ~1e-8 (achieved: machine precision ~1e-15)

#### Scenario: Polynomial (non-rational) surface data yields flat weights

- GIVEN grid points sampled from a NON-rational polynomial tensor-product B-spline surface, supplied with the surface's own parameters and knots
- WHEN the weights are estimated
- THEN the recovered weights SHALL all be ≈ equal (`weightSpread ≤ 1e-6`; achieved ~1e-10), `rationalityDetected` SHALL be false, and the fit SHALL reproduce the grid to the fit tolerance — the estimator correctly detects that no rationality is needed

#### Scenario: Ill-posed or degenerate surface weight estimation is declined honestly

- GIVEN an under-determined request (`3·nU·nV ≤ 4·nCtrlU·nCtrlV`), an all-coincident grid, a mismatched explicit-parameter length, a wrong explicit-knot length, a rank-deficient (non-unique-weight) configuration, or a configuration whose recovered weights flip sign
- WHEN surface weight estimation is attempted
- THEN the routine SHALL return `ok = false` with a populated `diagnostic` and an empty (non-rational) result, never crashing, never widening a tolerance, and never emitting an invalid rational

## MODIFIED Requirements

### Requirement: Weight estimation covers both curves and surfaces

The weight-estimation routines SHALL cover BOTH the curve case
(`fitRationalCurveEstimateWeights` / `…WithParams`) AND the tensor-product surface case
(`fitRationalSurfaceEstimateWeights` / `…WithParams`). The module SHALL recover the weights of a
rational tensor-product SURFACE from unweighted grid data via the same homogeneous null-space
formulation, and SHALL NOT fabricate weights: when a rational fit is ill-posed it SHALL
honest-decline rather than invent weights. Prescribed-weight surface fitting
(`interpolateRationalSurface`) remains available for the case where per-node weights are supplied.

#### Scenario: The module estimates surface weights or declines honestly

- GIVEN a request to recover a rational SURFACE from an unweighted grid
- WHEN no per-node weights are prescribed
- THEN the module SHALL estimate the surface weights via the homogeneous Ma–Kruth solve when the system is well-posed and over-determined, and SHALL honest-decline (with a diagnostic, no fabricated weights) when it is not — it SHALL NOT invent surface weights to present an invalid rational surface result
