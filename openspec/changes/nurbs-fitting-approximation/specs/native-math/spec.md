# native-math

## ADDED Requirements

### Requirement: Point-sequence parametrization for fitting

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_fit.{h,cpp}`, namespace `cybercad::native::math`, `CYBERCAD_HAS_NUMSCI`-gated),
parameter assignment for a sequence of points — **uniform** (*The NURBS Book* Eq 9.3),
**chord-length** (Eq 9.4/9.5), and **centripetal** (Eq 9.6) — producing a parameter `uₖ ∈ [0,1]`
for each point. The result SHALL be monotone non-decreasing with `u₀ = 0` and `u_{last} = 1`.
Coincident (duplicate) points SHALL contribute zero chord length so a run of identical points
shares a parameter (never a divide-by-zero), and an all-coincident input (total chord length
zero) SHALL return an empty result rather than crash, so callers can decline honestly.

#### Scenario: Chord-length and centripetal parameters are monotone in [0,1]

- GIVEN a sequence of at least two distinct points
- WHEN parameters are assigned by the chord-length or centripetal method
- THEN every parameter SHALL lie in `[0,1]`, the sequence SHALL be monotone non-decreasing, the first SHALL be `0` and the last SHALL be `1`

#### Scenario: Duplicate and all-coincident points are handled honestly

- GIVEN a sequence containing a repeated (coincident) point, or a sequence in which every point is identical
- WHEN parameters are assigned
- THEN a repeated point SHALL share its neighbour's parameter with all values finite, and an all-coincident sequence SHALL yield an empty parameter vector (an honest guard, not a crash)

### Requirement: Global B-spline curve interpolation through points

The module SHALL construct, for `N` input points and a chosen degree `p` (`N ≥ p+1`), a
**non-rational** B-spline curve (empty weights) that passes through **every** input point
(*The NURBS Book* A9.1: parametrization, averaging knots Eq 9.8, and a basis collocation solve
via the numerics facade `lin_solve`). The result SHALL satisfy the knot-length invariant
`knots.size() == poles.size() + degree + 1`, and SHALL evaluate to the input point `Qₖ` at its
parameter `uₖ` to solver precision. Degenerate input (fewer than `p+1` points, or all-coincident)
SHALL return `ok = false` without crashing.

#### Scenario: The interpolating curve passes through every input point

- GIVEN `N` points and a degree `p` with `N ≥ p+1`
- WHEN the curve is interpolated
- THEN the result SHALL have `N` control points, satisfy the knot-length invariant, be non-rational, and evaluate to `Qₖ` at each node parameter `uₖ` to within ~1e-9 (achieved: machine precision)

#### Scenario: Interpolation reconstructs known geometry (idempotent round-trip)

- GIVEN a point set interpolated to a curve `C1`, then re-sampled at `C1`'s node parameters and interpolated again to `C2`
- WHEN `C1` and `C2` are compared
- THEN their control nets SHALL be identical and they SHALL evaluate to the same point at every parameter to within ~1e-9 (achieved: ~1e-14) — proving the fit reconstructs a known B-spline pointwise

### Requirement: Least-squares B-spline curve approximation with honest error

The module SHALL fit, for `N` input points, a **non-rational** B-spline curve with exactly
`H < N` control points (`p+1 ≤ H`) minimizing the summed squared distance to the data, with the
first and last control points **pinned** to the first and last data points (endpoint
interpolation), solving the free interior control points via the numerics facade `lstsq`
(*The NURBS Book* A9.4/9.6, knot placement Eq 9.68/9.69). It SHALL report the **achieved**
maximum and RMS error `‖C(uₖ) − Qₖ‖` over the data, and SHALL NOT widen any tolerance to claim
success. Increasing `H` toward `N` SHALL not increase the error (monotone convergence toward
interpolation).

#### Scenario: Approximation pins the endpoints and reports the achieved error

- GIVEN `N` points and a control-point count `H` with `p+1 ≤ H < N`
- WHEN the curve is approximated
- THEN the first and last control points SHALL equal the first and last data points exactly, the result SHALL be non-rational with `H` control points, and the reported max/RMS error SHALL equal the independently-recomputed deviation (no widening)

#### Scenario: Approximation error decreases monotonically toward interpolation

- GIVEN a fixed point set fitted with a growing sequence of control-point counts `H₁ < H₂ < … < N`
- WHEN the max error is recorded for each fit
- THEN the error SHALL decrease monotonically as `H` grows, converging toward the interpolation residual as `H → N`

### Requirement: Tensor-product B-spline surface interpolation and approximation

The module SHALL interpolate and least-squares-approximate a **grid** of points (row-major,
U-outer) into a **non-rational** tensor-product B-spline surface (*The NURBS Book* A9.4-class),
by fitting each row in V then each resulting column in U with the curve routines (so the curve
and surface paths never diverge). Interpolation SHALL pass through every grid point; approximation
SHALL use fewer control points in each direction with the boundary rows/columns pinned, and SHALL
report the achieved max/RMS grid error. Degenerate input (a direction smaller than its
degree`+1`, or a control count outside `[degree+1, n]`) SHALL return `ok = false` without crashing.

#### Scenario: The interpolating surface passes through every grid point

- GIVEN an `nU × nV` grid of points and degrees `(degreeU, degreeV)` with `nU ≥ degreeU+1` and `nV ≥ degreeV+1`
- WHEN the surface is interpolated
- THEN the result SHALL have an `nU × nV` control net, be non-rational, and evaluate to the grid point `Q(i,j)` at `(uᵢ, vⱼ)` for every `(i,j)` to within ~1e-8 (achieved: machine precision)

#### Scenario: Surface approximation reduces the control net and converges

- GIVEN an `nU × nV` grid fitted with `nCtrlU × nCtrlV` control points (each ≤ its grid dimension)
- WHEN the surface is approximated
- THEN the result SHALL have the requested control-net size, report the achieved max/RMS grid error, and — at full control resolution (`nCtrlU = nU`, `nCtrlV = nV`) — reproduce the interpolating surface (grid error → machine precision)

### Requirement: Non-rational fitting scope with rational as an explicit residual

The fitting module SHALL produce **non-rational** B-spline curves and surfaces (all weights
implicitly one; the output `weights` vector is empty). Rational / weighted fitting (fitting the
control-point weights) SHALL NOT be fabricated: the module SHALL NOT emit weights it did not
solve for. Rational fitting is a documented residual for a later slice, recorded in
`docs/NURBS-SCOPE.md`.

#### Scenario: Fitted geometry is non-rational, not faked-rational

- GIVEN any successful curve or surface fit (interpolation or approximation)
- WHEN the result is inspected
- THEN its `weights` vector SHALL be empty (non-rational), and the module SHALL NOT attach fabricated weights or claim rational-fitting capability
