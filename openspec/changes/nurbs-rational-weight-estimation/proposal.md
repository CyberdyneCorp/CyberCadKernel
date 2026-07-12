# Proposal — nurbs-rational-weight-estimation (NURBS roadmap Layer 7, weight-recovery slice)

## Why

The `nurbs-rational-fitting` change landed rational fitting with **prescribed** weights and
explicitly recorded **weight ESTIMATION** (recovering the `wₖ` themselves from unweighted data,
Ma–Kruth) as a non-goal / residual. This change lands that residual for the curve case.

Weight estimation is the capability that lets a fit *exactly represent a conic* (circle, ellipse)
that a polynomial fit cannot: the fitter recovers both the control points AND the weights, so a
circular arc comes back as a true rational quadratic with the `cos(half-angle)` weight, not a
polynomial approximation of it. It is the classic reverse-engineering primitive (Ma & Kruth,
"NURBS curve and surface fitting for reverse engineering", CAD 1998; Piegl & Tiller Ch. 9).

## What

Extend `src/native/math/bspline_fit.{h,cpp}` **additively** (no existing signature changes) with:

- `fitRationalCurveEstimateWeights(points, nCtrl, degree, method, flatTol)` — recover the `nCtrl`
  control points AND their weights of a degree-`degree` rational B-spline that fits the (unweighted)
  data. The rational fit condition `C(uₖ) = Qₖ` is BILINEAR: writing the weighted control points
  `Hᵢ = wᵢ·Pᵢ`, `Σᵢ Nᵢ(uₖ)·wᵢ·(Pᵢ − Qₖ) = 0` becomes `Σᵢ Nᵢ(uₖ)·Hᵢ − Qₖ·(Σᵢ Nᵢ(uₖ)·wᵢ) = 0`, which
  is LINEAR and HOMOGENEOUS in `z = (H₀..Hₘ, w₀..wₘ)`. Stacked over all data (3 scalar rows per
  datum) this is `M·z = 0`; the least-squares fit is the smallest-singular-vector of `M`,
  recovered as the smallest-eigenvalue eigenvector of `MᵀM` by **shifted inverse iteration**
  through the numerics facade `lin_solve` (no external SVD). Then `Pᵢ = Hᵢ/wᵢ`.
- `fitRationalCurveEstimateWeightsWithParams(points, params, knots, nCtrl, degree, flatTol)` — the
  same solve with EXPLICIT parameters and flat knots. This is the airtight form: a rational shape's
  projective parameter is not its chord length, so an exact circular/elliptical arc is recovered to
  MACHINE precision only when the data carries its own NURBS parameter.
- A `RationalFitResult` struct carrying the fitted curve, achieved max/RMS error, the recovered
  weight `weightSpread` (after the `w₀ = 1` gauge), a `rationalityDetected` flag, and a
  `diagnostic` string used on decline.

The problem is homogeneous, so it must be OVER-determined (`3·nData > 4·nCtrl`): a conic arc is a
rational quadratic with `nCtrl = 3`, so sample it at `≫ 3` points and fit with `nCtrl = 3`,
`degree = 2`. The estimator handles the GAUGE freedom (weights are defined only up to a common
scale) by normalizing to `w₀ = 1`, and guards the sign so the shared weight sign is positive.

## Verification (HOST-analytic, airtight oracles)

`tests/native/test_native_nurbs_fit.cpp` (extended, numsci-gated):

1. **Conic recovery** — sample a KNOWN rational unit **circle** and an **ellipse** (quadratic NURBS,
   `w = √2/2` middle weights) at their own projective parameters; the estimator recovers the middle
   weights `= cos(45°)` to ~1e-8 and a curve that lies on the true circle / ellipse (radius /
   implicit deviation ≤ 1e-8; achieved ~1e-14). The strongest oracle — the fit is the exact conic.
2. **Polynomial degenerate** — data from a NON-rational polynomial cubic yields recovered weights
   all ≈ equal (`weightSpread ≤ 1e-6`; achieved ~1e-13) and `rationalityDetected == false`: the
   estimator correctly detects that no rationality is needed.
3. **De-homogenize consistency** — the recovered `(Pᵢ, wᵢ)` reproduce every input point within the
   fit tolerance (node pass-through ≤ 1e-8).
4. **Honest guards** — under-determined (`3·nData ≤ 4·nCtrl`), degenerate / all-coincident data,
   mismatched parameter length, and wrong knot length all DECLINE (`ok = false`, `diagnostic` set,
   no crash, no curve). A rank-deficient null space (weights not unique) and a sign-flipping /
   near-zero recovered weight (an invalid rational whose denominator vanishes) also decline.

## Scope

- Extends `src/native/math/bspline_fit.{h,cpp}` — additive only; `assignParams`, `interpolateCurve`
  / `approximateCurve`, `interpolateSurface` / `approximateSurface`, and both
  `interpolateRational*` signatures and behavior are byte-unchanged.
- Extends `tests/native/test_native_nurbs_fit.cpp` (already CMake-wired, numsci-gated).
- **`cc_*` ABI unchanged.** `src/native` stays OCCT-free. No change to `bspline_ops`, ssi, blend,
  boolean, topology, or any evaluator signature.

## Non-goals

- **No rational SURFACE weight estimation** — the tensor-product weight-recovery inverse is a
  separate, harder residual and is not attempted here; only the curve case lands.
- **No automatic parametrization for exact conic recovery** — machine-exact conic recovery uses the
  `WithParams` overload with the shape's own projective parameters; the method-based overload uses
  chord/centripetal parameters and HONEST-DECLINEs when they leave no unique rational.
- No new `cc_*` ABI.
