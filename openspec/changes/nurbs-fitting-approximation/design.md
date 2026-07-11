# Design — nurbs-fitting-approximation

## Placement & conventions

New module `src/native/math/bspline_fit.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_ops.{h,cpp}`. Reuses `math::Point3` / `Vec3` (`native/math/vec.h`), the evaluators
(`findSpan`, `basisFuns`, `curvePoint`, `surfacePoint` from `bspline.h`), and the **Layer-1
data types** `BsplineCurveData` / `BsplineSurfaceData` (from `bspline_ops.h`) as the fit OUTPUT
— a fitted curve/surface drops straight into the rest of the NURBS stack. OCCT-free, fp64,
deterministic. Added to the `native_math.h` aggregator.

**numsci gate.** The fit routines solve linear systems through the numsci facade
(`numerics::lin_solve` for the square interpolation collocation, `numerics::lstsq` for the
least-squares approximation), so the whole `.cpp` is under `CYBERCAD_HAS_NUMSCI` exactly like
`src/native/ssi/marching.cpp`: the header declares everything; with the guard OFF the
implementation TU is inert and the functions are absent. `CYBERCAD_HAS_NUMSCI` is defined
library-wide (`target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_NUMSCI=1)`), so
`bspline_fit.cpp` — though it lives in the default `src/native` glob — sees it when the option
is ON, matching how `marching.cpp` / `seeding.cpp` are gated.

Conventions match the rest of the kernel: **flat clamped knot vectors** (degree+1 end
multiplicity, length `nPoles + degree + 1`); **row-major, U-outer** surface poles
`pole(i,j) = poles[i*nPolesV + j]`; **non-rational** (weights empty).

## Parametrization (§9.2.1)

`assignParams(points, method)` returns `uₖ ∈ [0,1]`, monotone, `u₀=0`, `u_last=1`:
- **Uniform**: `uₖ = k/(n)` — ignores geometry.
- **ChordLength**: `uₖ` proportional to cumulative `‖Qₖ − Qₖ₋₁‖` (exponent 1).
- **Centripetal** (Lee): proportional to cumulative `√‖Qₖ − Qₖ₋₁‖` (exponent ½) — tamer near
  sharp turns.

Duplicate points contribute zero (exponentiated) chord length, so a run of identical points
**shares** a parameter (no divide-by-zero). If the TOTAL chord length is zero (all points
coincident) the function returns empty — there is no length to normalize against, and the
caller (`interpolateCurve` / `interpolateSurface`) declines with `ok=false`.

## Knot generation

- **averagingKnots(u, p)** (Eq 9.8): clamped vector whose interior knot `U[j+p]` is the running
  average of `p` consecutive parameters `u[j..j+p-1]`. This makes the collocation matrix totally
  positive and banded (well-conditioned). Length `= u.size() + p + 1`.
- **approxKnots(u, p, nCtrl)** (Eq 9.68/9.69): clamped vector spreading `nCtrl−p−1` interior
  knots across the data via `d = (n+1)/(nCtrl−p)`, `U[p+j] = (1−α)u[i−1] + α u[i]` with
  `i = ⌊j·d⌋`, `α = j·d − i`. Length `= nCtrl + p + 1`.

## Curve interpolation (A9.1)

For `n+1` points of degree `p`: derive parameters, build averaging knots, assemble the
`(n+1)×(n+1)` collocation matrix `A(k,i) = N_{i,p}(u_k)` (row-major; each row has ≤ `p+1`
non-zeros located by `findSpan` + `basisFuns`), and solve `A·P = Q` once per coordinate with
`numerics::lin_solve`. The three RHS reuse the same factorization-free dense solve. The result
passes through `Qₖ` at `uₖ` to solver precision. Guards: `points.size() ≥ p+1`; degenerate
(all-coincident) input → `ok=false`.

## Curve least-squares approximation (A9.4/9.6)

Fit `H = nCtrl` control points (`p+1 ≤ H < N`), with `P₀` and `P_{H−1}` **pinned** to `Q₀` and
`Q_{N−1}`. The interior data points `Q₁..Q_{N−2}` drive the fit. For each interior data point `k`
the pinned end contributions move to the RHS (Eq 9.63):
`R_k = Q_k − N_{0,p}(u_k)·Q₀ − N_{H−1,p}(u_k)·Q_{N−1}`, and the overdetermined
`(N−2)×(H−2)` design matrix of the free basis values is handed to `numerics::lstsq` (three RHS
columns for x/y/z). Reports the ACHIEVED max / RMS `‖C(uₖ) − Qₖ‖` — never widened. `nCtrl==2`
(a straight degree-1 segment) is the degenerate no-free-unknown case (endpoint line).

## Surface fitting — tensor product (A9.4-class)

`fitSurface` drives both interpolation (`nCtrl == n` in a direction) and approximation
(`nCtrl < n`). One shared U parametrization and one shared V parametrization are obtained by
**averaging** the per-line assignments across the grid (§9.2.5). Then:
1. Fit each of the `nU` rows in V to `nCtrlV` control points → an intermediate `nU × nCtrlV` net
   sharing one V knot vector.
2. Fit each of the `nCtrlV` resulting columns in U to `nCtrlU` control points → the final
   `nCtrlU × nCtrlV` net sharing one U knot vector.

Each row/column fit is `fitLine`, which is interpolation (square `lin_solve`) when
`nCtrl == npts` and least-squares (`lstsq`, endpoints pinned) otherwise — the SAME code the
stand-alone curve routines use, so curve and surface paths never diverge. `interpolateSurface`
is `fitSurface(g, nU, nV, …)`; `approximateSurface` passes smaller `nCtrlU/nCtrlV`.

## Oracle strategy (why this layer is airtight)

| Property | Exact invariant (HOST, no OCCT) | Achieved |
|---|---|---|
| Interpolation exactness | `C(uₖ) == Qₖ` for every input, dense | ~1e-15 |
| Round-trip (idempotence) | interpolate → resample at nodes → interpolate ≡ original, POINTWISE | ~1e-14 |
| Approximation error | reported max/RMS is the ACHIEVED error; monotone ↓ as `H↑` toward `N` | 3.7e-1 → 9.5e-7 |
| Endpoint pinning | approx first/last control point == first/last data point | exact |
| Surface interpolation | `S(uᵢ,vⱼ) == Q(i,j)` for every grid point | ~1e-15 |
| Parametrization | chord-length + centripetal in `[0,1]`, monotone; dup shares param; all-coincident → honest empty | — |

The idempotence round-trip is the key: it avoids the parametrization confound (a uniform-knot
source resampled at chord-length nodes is a *different-but-node-coincident* spline), and instead
asserts that re-fitting a curve's OWN resampled points reproduces it everywhere to machine
precision — proving the fit reconstructs known B-spline geometry exactly.

## Complexity & structure

Chapter 9's algorithms are index-dense; per the cognitive-complexity policy the
compilers/parsers band (25–35) applies. Each routine is one focused function with the book's
Eq/algorithm numbers in comments; the collocation-assembly + coordinate-solve and the
"fit each line then transpose direction" surface driver are shared helpers (`fitLine`,
`fitSurface`, `curveErrors`, `surfaceErrors`), not copy-paste, so the interpolation/approximation
and curve/surface paths are single-sourced.

## Risks & honest residuals

- **Non-rational only.** Rational/weighted fitting (fitting the weights) is materially harder and
  is deliberately out of scope; the module returns non-rational curves/surfaces (empty weights)
  and never fabricates weights. Documented in `docs/NURBS-SCOPE.md` Layer-7 row.
- **Parametrization/knot heuristics are the standard book choices** (chord-length + averaging /
  Eq 9.69). Pathological data (extreme clustering) can ill-condition the collocation matrix; the
  dense solve returns empty on a singular system and the fit declines with `ok=false` (honest),
  never a silently-wrong curve.
- **Approximation at `H=N−1`** is one DOF short of interpolation, so its error is small but not
  machine-zero (achieved ~1e-6 for the test curve) — reported honestly, not forced to a tolerance.
