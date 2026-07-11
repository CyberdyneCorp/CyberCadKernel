# Design — nurbs-surface-offset

## Placement & conventions

New module `src/native/math/bspline_offset.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_ops.{h,cpp}`, `bspline_fit.{h,cpp}`, and `bspline_skin.{h,cpp}`. Reuses `math::Point3` /
`Dir3` (`native/math/vec.h`), the evaluators `surfacePoint` / `nurbsSurfacePoint` / `surfaceNormal`
/ `surfaceDerivs` / `nurbsSurfaceDerivs` (`bspline.h`), the **Layer-1 data type**
`BsplineSurfaceData`, the **Layer-7 fitter** `interpolateSurface` (`bspline_fit.h`), and the
**numerics facade** `closest_point_on_surface` (`numerics.h`). OCCT-free, fp64, deterministic. Added
to the `native_math.h` aggregator.

**numsci gate.** The fit solves the collocation system through the numsci facade
(`numerics::lin_solve`, via `interpolateSurface`) and the deviation metric projects through
`numerics::closest_point_on_surface`, so the whole `.cpp` is under `CYBERCAD_HAS_NUMSCI`, exactly
like `bspline_fit.cpp` / `bspline_skin.cpp`: the header declares everything; with the guard OFF the
implementation TU is inert and the function is absent. `CYBERCAD_HAS_NUMSCI` is defined library-wide
(`target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_NUMSCI=1)`), so
`bspline_offset.cpp` — though in the default `src/native` glob — sees it when the option is ON.

Conventions match the rest of the kernel: **flat clamped knot vectors** (degree+1 end multiplicity,
length `nPoles + degree + 1`); **row-major, U-outer** surface poles `pole(i,j) = poles[i*nPolesV + j]`;
the fitted offset is **non-rational** (weights empty).

## Why the offset is fitted, not exact

The true offset locus `O(u,v) = S(u,v) + d·N(u,v)` uses the unit normal
`N = (Sᵤ × Sᵥ) / ‖Sᵤ × Sᵥ‖`. The `1/‖·‖` factor is a square root of a polynomial, so `O` is not a
rational function of `(u,v)` and cannot be represented exactly as a NURBS (except in trivial cases:
a plane offsets to a plane, and a cylinder/sphere offset to a coaxial/concentric one, because there
the normal field is constant/radial). The general offset is therefore APPROXIMATED by sampling `O`
and fitting — the standard CAD-kernel approach (Piegl & Tiller Ch. 10).

## Self-intersection (fold) guard — the second fundamental form

The offset map `Φ_d : (u,v) ↦ S + d·N` is regular exactly where its differential is non-singular.
In the principal-curvature frame the differential's eigenvalues are `(1 + d·κ₁)` and `(1 + d·κ₂)`,
where `κ₁,κ₂` are the principal curvatures of `S`. The offset stays a valid (non-folded) surface iff
`(1 + d·κᵢ) > 0` for both `i`; it folds (self-intersects) the instant some `1 + d·κ = 0`, i.e. when
`|d|` reaches the radius of curvature `1/|κ|` on the side the offset bends toward (`d·κ < 0`).

`principalCurvatures(u,v)` computes `κ₁,κ₂` from the fundamental forms:

- First form `E = Sᵤ·Sᵤ`, `F = Sᵤ·Sᵥ`, `G = Sᵥ·Sᵥ` (metric determinant `EG − F²`).
- Second form `L = Sᵤᵤ·n`, `M = Sᵤᵥ·n`, `N₂ = Sᵥᵥ·n` with unit normal `n`.
- Gaussian `K = (L·N₂ − M²)/(EG − F²)`, mean `H = (E·N₂ − 2F·M + G·L)/(2(EG − F²))`.
- `κ = H ± √(max(0, H² − K))` (the `max(0,·)` guards an umbilic point's `H² − K → 0⁻` fp noise).

Derivatives come from `surfaceDerivs` (or `nurbsSurfaceDerivs`) at order 2. A near-null normal
(`‖Sᵤ × Sᵥ‖ ≤ 1e-9`) or a degenerate metric (`EG − F² ≤ 1e-14`) declines as `DegenerateNormal`.

The guard runs on a dense analysis grid (`≥ 11×11`) BEFORE any fit. If the smallest
`(1 + d·κ)` over the grid is `≤ 0`, `offsetSurface` returns `SelfIntersection` with an empty surface
and the reported `minCurvatureRadius` (the smallest `1/|κ|` on the folding side). It NEVER returns a
folded surface.

## Sample → fit → refine loop

For grid resolution `g` (from `startGrid`, doubling `g ← 2g−1` up to `maxGrid`):

1. Sample the offset locus `O = S + d·N` on the `g × g` uniform parameter grid over `S`'s domain
   (rational-aware `evalS` + `surfaceNormal`).
2. `interpolateSurface` fits a degree-`(min(3,degU), min(3,degV))` tensor B-spline THROUGH the
   samples (`lin_solve` collocation). Interpolation reproduces the samples exactly (`~1e-15`).
3. **Deviation metric** — the fitter re-parametrizes by chord length, so its knots do NOT align with
   `S`'s grid; comparing the two surfaces at matching parameters is invalid. Instead the deviation is
   measured GEOMETRICALLY and parametrization-free: a FIXED `7×7` check grid over the fitted
   surface's own domain (cell-centred, strictly between the reproduced samples) is projected onto `S`
   with `closest_point_on_surface`; the offset error is `max |dist(fittedPoint, S) − |d||`. This is
   exactly the offset property (every point at distance `|d|` from `S`), independent of how either
   surface is parametrized.
4. Keep the best (lowest-error) fit. Stop when the error ≤ `tol` or `g` reaches `maxGrid`.

The check grid is a fixed, modest size (not `∝ g`) to keep the per-level projection cost bounded;
its cell-centred points probe the interpolant where it is free to stray, not at the reproduced-exactly
nodes. The returned `OffsetResult` carries the achieved `maxError` (the honest deviation, never
widened), `status` (`Ok` / `ToleranceNotMet` / `SelfIntersection` / `DegenerateNormal` /
`DegenerateInput` / `FitFailed`), the accepted grid resolution, and `minCurvatureRadius`.

## Guarantees & guards summary

- **Offset distance** — every returned point is at distance `|d|` from `S` along its normal within
  the reported `maxError` (the host gate asserts this on a dense projection grid).
- **Exact cases land exactly** — a plane offsets to a parallel plane and a cylinder to radius `r±d`
  to the fit tolerance (both verified analytically in the host gate).
- **Monotone convergence** — refining the grid lowers the reported error monotonically (fp slack).
- **Honest decline** — a fold (self-intersection) or a degenerate normal returns `ok=false` with an
  empty surface and a diagnostic status; a fit that cannot reach `tol` within the budget returns the
  best surface flagged `ToleranceNotMet` with its true error.

## Residuals (documented, never faked)

- **Solid thicken / shell / hollow** — offsetting both faces of a solid and stitching side walls into
  a closed B-rep solid is a separate construction (a `BRepOffset`-class shell). This module produces
  the single offset SURFACE such a shell is assembled from.
- **Self-intersecting-offset trimming** — recovering a valid offset by trimming the folded region
  (rather than declining) is a robustness residual; this module declines folds honestly.
- **Rational offset fit** — the fitted offset is non-rational; the input may be rational but weights
  are not fitted.
