# Proposal — nurbs-rational-surface-weight-estimation (NURBS roadmap Layer 7, surface weight-recovery slice)

## Why

The `nurbs-rational-weight-estimation` change landed rational weight ESTIMATION for the CURVE case
(recovering both control points and weights from unweighted data, Ma–Kruth) and explicitly recorded
rational **SURFACE** weight estimation as a documented residual ("Weight estimation is curve-only;
surface weight estimation is an explicit residual"). This change lands that residual.

Surface weight estimation is what lets a fit *exactly represent a quadric patch* — a quarter-cylinder
or a sphere octant — from an unweighted grid of sampled points, recovering the tensor-product control
net AND the weight net so the patch comes back as a true rational surface (with the `cos(half-angle)`
weights) rather than a polynomial approximation of it. It is the surface analogue of the curve
reverse-engineering primitive (Ma & Kruth, "NURBS curve and surface fitting for reverse engineering",
CAD 1998; Piegl & Tiller Ch. 9).

## What

Extend `src/native/math/bspline_fit.{h,cpp}` **additively** (no existing signature changes) with:

- `fitRationalSurfaceEstimateWeights(grid, nCtrlU, nCtrlV, degreeU, degreeV, method, flatTol)` —
  recover the `nCtrlU × nCtrlV` control net AND its weights of a degree-`(degreeU,degreeV)` rational
  tensor-product B-spline surface that fits the (unweighted) grid. Derives averaged U/V params +
  approximation knots, then delegates to the explicit-parameter core.
- `fitRationalSurfaceEstimateWeightsWithParams(grid, uParams, vParams, knotsU, knotsV, nCtrlU,
  nCtrlV, degreeU, degreeV, flatTol)` — the airtight form for exact quadric recovery: the caller
  supplies the per-row/per-column projective NURBS parameters and both flat knot vectors.
- `RationalSurfaceFitResult` struct (surface, max/RMS error, `weightSpread`, `rationalityDetected`,
  `diagnostic`).

The tensor rational-fit condition `S(uₖ,vₗ) = Q(k,l)` is expressed in its bilinear form: writing the
weighted control net `Hᵢⱼ = wᵢⱼ·Pᵢⱼ`, the condition
`Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·wᵢⱼ·(Pᵢⱼ − Q(k,l)) = 0` becomes the LINEAR HOMOGENEOUS system
`Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·Hᵢⱼ − Q(k,l)·(Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·wᵢⱼ) = 0` in `z = (Hᵢⱼ, wᵢⱼ)`. Stacked over the
grid (3 rows per datum) this is `M·z = 0`; the fit is the smallest-eigenvalue eigenvector of `MᵀM`,
recovered by the SAME shifted inverse iteration through the numerics facade `lin_solve` as the curve
case (no external SVD, the numsci facade is NOT widened). The net is de-homogenized `Pᵢⱼ = Hᵢⱼ/wᵢⱼ`
and gauged to `w₀₀ = 1`, shared sign positive.

## Impact

- Additive only — no existing `bspline_fit` signature changes; `cc_*` ABI byte-unchanged.
- `src/native/**` stays OCCT-free.
- Supersedes the "surface weight estimation is an explicit residual" requirement from
  `nurbs-rational-weight-estimation` with the implemented capability.
- Tests extend `tests/native/test_native_nurbs_fit.cpp` (already wired under `CYBERCAD_HAS_NUMSCI`).
