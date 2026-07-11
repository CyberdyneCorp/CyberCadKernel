# native-math

## ADDED Requirements

### Requirement: Rational B-spline curve interpolation with prescribed weights

The native math library SHALL construct, in the OCCT-free `CYBERCAD_HAS_NUMSCI`-gated fitting
module (`src/native/math/bspline_fit.{h,cpp}`), a **rational** NURBS curve that passes through
`N` input points `Qₖ` when each point is given a strictly positive **prescribed** weight `wₖ`
(`interpolateRationalCurve(points, weights, degree, method)`, `N ≥ degree+1`). Each datum SHALL
be lifted to the homogeneous point `Qʷₖ = (wₖ·xₖ, wₖ·yₖ, wₖ·zₖ, wₖ) ∈ R⁴`; the **same** averaging-knot
collocation matrix used for non-rational interpolation SHALL be solved (via the numerics facade
`lin_solve`) for **all four** homogeneous coordinates — the fourth yielding the control weights
`Wᵢ` — and the control net SHALL be projected back `Pᵢ = (Xᵢ/Wᵢ, Yᵢ/Wᵢ, Zᵢ/Wᵢ)`, `weightᵢ = Wᵢ`.
The returned curve SHALL be rational (one weight per pole, non-empty `weights`) and SHALL evaluate
(as a rational NURBS) to the **Euclidean** datum `Qₖ` at its parameter `uₖ` to solver precision.

The routine SHALL decline (`ok = false`, no crash, no fabricated curve) when the weight count does
not match the point count, when any input weight is not strictly positive, when there are fewer
than `degree+1` points, when the parametrization is degenerate (all-coincident input), or when any
**solved control weight** `Wᵢ` is not strictly positive (a projected non-positive weight is a
documented guard — the fit never divides by a weight ≤ 0). Weights are PRESCRIBED inputs; the
routine SHALL NOT estimate weights from unweighted data.

#### Scenario: The rational curve passes through every Euclidean point

- GIVEN `N` points, a degree `p` with `N ≥ p+1`, and a strictly positive prescribed weight per point that yields positive interpolated control weights
- WHEN the rational curve is interpolated
- THEN the result SHALL be rational (one positive weight per pole), have `N` control points, satisfy `knots.size() == poles.size() + degree + 1`, and evaluate as a rational NURBS to `Qₖ` at each node parameter `uₖ` to within ~1e-9 (achieved: machine precision)

#### Scenario: A known rational conic is reconstructed exactly

- GIVEN a KNOWN rational unit circle (quadratic NURBS) sampled at `N` parameters, each sample carrying its Euclidean point and the exact rational denominator `Σ Nᵢ(u)·wᵢ` as its prescribed weight
- WHEN those (point, weight) pairs are interpolated rationally, giving `C1`, and `C1` is resampled at its own node parameters with the exact denominators and interpolated again to `C2`
- THEN the recovered curve SHALL be the unit circle at every sampled node to ~1e-9 and stay planar, and `C1 ≡ C2` SHALL hold POINTWISE to ~1e-9 (achieved ~1e-15) — proving exact rational reconstruction

#### Scenario: Non-positive and mismatched weights are declined honestly

- GIVEN a mismatched weight count, a zero or negative input weight, a too-few-point or all-coincident input, or a positive-but-wild weight sequence that drives an interpolated control weight non-positive
- WHEN rational interpolation is attempted
- THEN the routine SHALL return `ok = false` with an empty (non-rational) result, never crashing and never emitting a curve with a non-positive control weight

### Requirement: Rational tensor-product B-spline surface interpolation with prescribed weights

The module SHALL construct a **rational** tensor-product NURBS surface that passes through every
point of an `nU × nV` grid `Q(i,j)` (row-major, U-outer) when each grid point is given a strictly
positive **prescribed** weight `w(i,j)` supplied as a parallel `WeightGrid`
(`interpolateRationalSurface(grid, weightGrid, degreeU, degreeV, method)`). Each grid datum SHALL
be lifted to `Qʷ(i,j) = (w·x, w·y, w·z, w) ∈ R⁴`; the surface SHALL be interpolated on the 4-D
homogeneous net by fitting each row in V then each resulting column in U (the weight coordinate
interpolated exactly like x/y/z, reusing the non-rational tensor machinery), and the control net
SHALL be projected back to (pole, weight). The returned surface SHALL be rational (one weight per
pole) and SHALL evaluate as a rational NURBS to the Euclidean `Q(i,j)` at `(uᵢ, vⱼ)` to solver
precision.

The routine SHALL decline (`ok = false`, no crash) when the weight-grid dimensions do not match
the point grid, when any input weight is not strictly positive, when a direction is smaller than
its degree`+1`, or when any solved control weight is not strictly positive.

#### Scenario: The rational surface passes through every grid point

- GIVEN an `nU × nV` point grid with `nU ≥ degreeU+1`, `nV ≥ degreeV+1`, a matching `WeightGrid` of strictly positive weights yielding positive control weights
- WHEN the rational surface is interpolated
- THEN the result SHALL be rational with an `nU × nV` control net (one positive weight per pole) and evaluate as a rational NURBS to `Q(i,j)` at `(uᵢ, vⱼ)` for every `(i,j)` to within ~1e-8 (achieved: machine precision)

#### Scenario: A known rational cylinder is reconstructed exactly

- GIVEN a KNOWN rational half-cylinder sampled on a grid, each node carrying its Euclidean point and the exact rational surface denominator as its prescribed weight
- WHEN the grid is interpolated rationally to `S1`, then `S1` is resampled at the fit's own averaged node parameters with the exact denominators and interpolated again to `S2`
- THEN `S1` SHALL be the cylinder at every grid node to ~1e-8 and `S1 ≡ S2` SHALL hold POINTWISE to ~1e-8 (achieved ~1e-15) — proving exact rational surface reconstruction

#### Scenario: Mismatched weight grid or non-positive weight is declined honestly

- GIVEN a `WeightGrid` whose dimensions differ from the point grid, or a grid containing a non-positive weight
- WHEN rational surface interpolation is attempted
- THEN the routine SHALL return `ok = false` with an empty result, never crashing

### Requirement: Rational fitting is prescribed-weight only; weight estimation is an explicit residual

The rational fitting routines SHALL fit only when the weights are **prescribed** by the caller.
The module SHALL NOT estimate the weights from unweighted data (e.g. Ma–Kruth NURBS fitting) and
SHALL NOT fabricate weights it did not solve for from prescribed inputs. Weight estimation is a
documented residual recorded in `docs/NURBS-SCOPE.md`. Non-rational fitting continues to return an
empty `weights` vector; rational fitting returns one solved weight per pole.

#### Scenario: The module never fakes weight estimation

- GIVEN a request to fit rational geometry
- WHEN no per-point weights are prescribed
- THEN the module SHALL offer only non-rational fitting (empty weights) — it SHALL NOT invent weights to present a rational result, and the rational routines require an explicit positive weight per datum
