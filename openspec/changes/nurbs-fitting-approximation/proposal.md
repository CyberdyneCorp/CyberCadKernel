# Proposal — nurbs-fitting-approximation (NURBS roadmap Layer 7)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 1 (the
exact-NURBS *geometry kernel* — knot/degree/split ops) is landed
(`src/native/math/bspline_ops.{h,cpp}`). Layer 7 is **fitting / approximation**: given a
sampled sequence or grid of **points**, construct a B-spline curve or surface that either
**interpolates** them (passes through every point) or **approximates** them (least-squares,
fewer control points). This is the scan-to-CAD / point-cloud / data-reduction direction — the
inverse of evaluation: instead of "curve → points" it is "points → curve".

This layer is worth building **now**, like Layer 1, because it is (a) small and well-bounded
(*The NURBS Book* Chapter 9, a handful of algorithms), (b) built entirely on machinery that
already exists — the Layer-1 data types (`BsplineCurveData` / `BsplineSurfaceData`), the
evaluators (`findSpan` / `basisFuns` / `curvePoint` / `surfacePoint`), and the numsci facade's
dense `lin_solve` / `lstsq` — and (c) **uniquely airtight to verify**: an interpolant must pass
through every input point (closed-form residual → 0), and re-fitting a curve's own resampled
points must reproduce it *pointwise* to machine precision (idempotent round-trip). It reconstructs
KNOWN geometry, which is the strongest oracle available.

## What

A new OCCT-free module `src/native/math/bspline_fit.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_ops`), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`, like
`src/native/ssi/marching.cpp`) because the linear systems are solved through the numsci facade.
It reuses the Layer-1 `BsplineCurveData` / `BsplineSurfaceData` types as its fit output.
**Non-rational only** (all weights = 1); rational/weighted fitting is an explicit residual.

From *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 9:

1. **Parametrization** — assign a parameter `uₖ ∈ [0,1]` to each point: **uniform** (Eq 9.3),
   **chord-length** (Eq 9.4/9.5), and **centripetal** (Eq 9.6). Monotone, in `[0,1]`;
   duplicate/coincident points handled with an honest guard (no divide-by-zero, no crash).
2. **Knot-vector generation** — **averaging knots** for interpolation (Eq 9.8) and **knot
   placement** for approximation (Eq 9.68/9.69).
3. **Curve interpolation** (A9.1) — global degree-`p` B-spline through N points: build the
   `(N×N)` basis collocation matrix, solve once per coordinate with `numerics::lin_solve`; the
   curve passes through every point exactly.
4. **Curve least-squares approximation** (A9.4/9.6) — fit `H < N` control points minimizing the
   summed squared distance, first/last control points pinned (endpoint interpolation); the free
   interior points solve via `numerics::lstsq`. Reports the achieved max / RMS error.
5. **Surface interpolation + approximation** (A9.4-class, tensor-product) — interpolate /
   approximate a grid of points by fitting each row (V) then each resulting column (U), reusing
   the curve routines so the two never diverge.

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_fit.cpp`:
  1. **Interpolation exactness** — the interpolating curve/surface passes through EVERY input
     point to ~1e-9 (achieved: machine precision ~1e-15).
  2. **Round-trip recovery (idempotence)** — interpolate a point set → C1; resample C1 at its
     node parameters → interpolate again → C2; C1 ≡ C2 pointwise to ~1e-9 (achieved ~1e-14).
     The strongest oracle: the fit reconstructs known B-spline geometry exactly.
  3. **Approximation error** — fitting `H < N` control points reports the ACHIEVED max/RMS error
     (never widened), and that error DECREASES monotonically as `H` grows toward `N`, converging
     to interpolation (achieved series ≈ 3.7e-1 → 3.0e-1 → 8.5e-2 → 1.1e-2 → 6.1e-4 → 9.5e-7).
  4. **Parametrization sanity** — chord-length + centripetal both in `[0,1]`, monotone;
     duplicate points share a parameter; all-coincident input declines honestly (empty guard).
- **SIM native-vs-OCCT parity** — OPTIONAL cross-check against OCCT `GeomAPI_Interpolate` /
  `GeomAPI_PointsToBSpline` (a separate track; HOST is primary and sufficient).

## Scope

- Adds `src/native/math/bspline_fit.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob. Added to `native_math.h`.
- Adds `tests/native/test_native_nurbs_fit.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_ssi_seeding`.
- **`cc_*` ABI unchanged.** Layer 7 is an internal geometry-algorithm library; its consumers are
  later surfacing features, not the app today. No ABI is added until a consumer needs it —
  consistent with the demand-driven policy.

## Non-goals

- **No rational / weighted fitting** — fitting the weights (e.g. Ma–Kruth NURBS fitting) is
  materially harder and is an explicit residual for a later slice. This module fits non-rational
  B-splines only and never fakes rational output.
- No advanced surfacing (skinning / lofting / Gordon / network / plate), no error-driven
  adaptive knot refinement, no automatic degree/knot selection — those remain demand-gated.
- No new `cc_*` ABI; no change to STEP admission, the tessellator, or any evaluator signature.
