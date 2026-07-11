# Proposal — nurbs-rational-fitting (NURBS roadmap Layer 7, rational slice)

## Why

Layer 7 (`src/native/math/bspline_fit.{h,cpp}`) landed the **non-rational** side of fitting —
points → B-spline curve/surface, interpolation + least-squares approximation (change
`nurbs-fitting-approximation`). That change explicitly deferred **rational (weighted) fitting**
as a residual. This change lands the **tractable exact case** of it: rational
INTERPOLATION with **prescribed weights**.

The full rational-fitting problem has two halves that differ enormously in difficulty:

1. **Prescribed weights (this change)** — the caller already knows the weight `wₖ` each data
   point should carry (they come from a known conic/circle, a CAD feature, or a designer). Fit the
   homogeneous control net so a rational NURBS with those weights passes through the data. This is
   a *linear* problem: exactly the non-rational collocation solve, run in homogeneous R⁴.
2. **Weight estimation (NOT this change)** — recover the `wₖ` *themselves* from unweighted points
   (e.g. Ma–Kruth). This is a *non-linear* inverse problem and remains an explicit residual; this
   change does not attempt it and never fakes it.

The prescribed-weights case is worth landing now for the same reasons Layer 7 was: it is small,
built entirely on existing machinery (the Layer-1 homogeneous-lift convention already used by
`bspline_ops`, the existing collocation solve, the numsci facade), and **uniquely airtight** —
the rational curve must pass through every Euclidean datum exactly, and a KNOWN rational conic
(a unit circle, a cylinder) must be reconstructed pointwise. It also unblocks *rational*
skinning/offset/thicken consumers later.

## What

Extend `src/native/math/bspline_fit.{h,cpp}` **additively** (no existing signature changes) with:

- `interpolateRationalCurve(points, weights, degree, method)` — given data points `Qₖ` and a
  positive weight `wₖ` each, lift to the homogeneous point `Qʷₖ = (wₖ·xₖ, wₖ·yₖ, wₖ·zₖ, wₖ) ∈ R⁴`,
  run the **same** averaging-knot collocation matrix as `interpolateCurve` for **all four**
  homogeneous coordinates (the 4th solves for the control weights `Wᵢ`), then project the net back
  `Pᵢ = (Xᵢ/Wᵢ, Yᵢ/Wᵢ, Zᵢ/Wᵢ)`, `weightᵢ = Wᵢ`. Because `Cʷ(uₖ) = Qʷₖ`, the projected rational
  curve interpolates the **Euclidean** `Qₖ` exactly. Returns a rational `BsplineCurveData`
  (poles + non-empty weights).
- `interpolateRationalSurface(grid, weightGrid, degreeU, degreeV, method)` — the tensor-product
  analogue: lift the grid to R⁴ with per-node prescribed weights, interpolate each row in V then
  each column in U on the 4-D net (the weight coordinate is interpolated exactly like x/y/z),
  project back to (pole, weight). Passes through every Euclidean grid datum.
- A `WeightGrid` struct (parallel to `PointGrid`) for the surface weights.

The rational-lift convention is identical to `bspline_ops.h` (Layer 1), so the curve, surface and
rational paths never diverge. A projected **non-positive control weight** is a documented guard
(the fit declines rather than divide by ≤ 0).

## Verification (HOST-analytic, airtight rational oracles)

`tests/native/test_native_nurbs_fit.cpp` (extended, numsci-gated):

1. **Rational interpolation exactness** — the rational curve/surface passes through every
   Euclidean datum to ~1e-9 (achieved ~1e-15) for arbitrary positive prescribed weights.
2. **Known-rational round-trip** — sample a KNOWN rational unit **circle** (quadratic NURBS,
   `w = √2/2` middle weights) and a rational half-**cylinder** at `N` params, capturing at each
   param the Euclidean point AND the exact rational denominator `Σ Nᵢ(u)wᵢ` as the prescribed
   weight; interpolate; recover the conic at every node to ~1e-15 and reconstruct it POINTWISE by
   rational **idempotence** to ~1e-15 (the strongest oracle — exact rational reconstruction).
3. **Recovered weights match input** — an idempotent rational round-trip reproduces the control
   weights (up to the invariant common scale) and the projected poles to ~1e-10.
4. **Guards** — mismatched weight count / grid dims, a zero or negative input weight, an
   all-coincident or too-few-point input, and a wild weight sequence that drives an interpolated
   **control** weight non-positive, all DECLINE honestly (`ok = false`, no crash, no faked curve).

## Scope

- Extends `src/native/math/bspline_fit.{h,cpp}` — additive only; existing `assignParams`,
  `interpolateCurve` / `approximateCurve`, `interpolateSurface` / `approximateSurface` signatures
  and behavior are byte-unchanged.
- Extends `tests/native/test_native_nurbs_fit.cpp` (already CMake-wired, numsci-gated).
- Updates `docs/NURBS-SCOPE.md` §4 Layer-7 rows (rational-with-prescribed-weights landed; weight
  estimation residual).
- **`cc_*` ABI unchanged.** `src/native` stays OCCT-free. No change to `bspline_ops`, ssi, blend,
  boolean, topology, or any evaluator signature.

## Non-goals

- **No weight ESTIMATION** — recovering the `wₖ` from unweighted data (Ma–Kruth and relatives) is
  a non-linear inverse problem and remains an explicit residual. This change fits only when the
  weights are **prescribed** inputs, and never fabricates them.
- **No rational APPROXIMATION** — least-squares rational fitting (fewer control points) is not in
  scope; only the exact (interpolation) rational case.
- No advanced rational surfacing (rational skin/loft/Gordon/offset); those remain demand-gated.
- No new `cc_*` ABI.
