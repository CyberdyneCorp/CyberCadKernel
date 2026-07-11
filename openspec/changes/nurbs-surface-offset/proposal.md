# Proposal — nurbs-surface-offset (NURBS roadmap Layer 5)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 1 (the
exact-NURBS *geometry kernel*, `src/native/math/bspline_ops.{h,cpp}`), Layer 7 (fitting /
approximation, `src/native/math/bspline_fit.{h,cpp}`), and Layer 6 (skinning / lofting,
`src/native/math/bspline_skin.{h,cpp}`) are landed. Layer 5 is **surface offset**: given a NURBS
surface `S(u,v)`, construct the OFFSET surface at signed distance `d` — the true offset locus is the
point set `O(u,v) = S(u,v) + d·N(u,v)`, where `N` is the unit surface normal. Offset is the core
modeling op underneath **shell / thicken / hollow** (Shapr3D's shell workflow).

The exact offset of a NURBS surface is **not** a NURBS in general (the unit normal carries a square
root, so `O` is not piecewise-rational), so it is **approximated**: sample the offset locus on an
adaptive grid and FIT a NURBS surface through the samples with the landed Layer-7 fitter, refining
the grid until the fitted surface's deviation from the true locus is within tolerance — reporting
the achieved offset error honestly.

This layer is worth building **now** because it (a) is small and well-bounded (*The NURBS Book*
Piegl & Tiller Ch. 10, offset approximation — sample + fit + refine), (b) is built entirely on
machinery that already exists — the evaluators `surfacePoint` / `surfaceNormal` / `surfaceDerivs`
(Layer-1), the Layer-7 `interpolateSurface` (the constructor for the fitted offset), and the
numerics facade `closest_point_on_surface` (the offset-distance metric) — and (c) has an
**airtight, closed-form oracle**: every point of the fitted offset must lie at distance `|d|` from
`S` along its normal, and for a cylinder / plane the offset has an exact analytic form (radius `r±d`
/ a parallel plane). It also demands an **honest self-intersection guard**: an offset by `|d|` past
a principal radius of curvature on the concave side folds the surface onto itself — this must be
DETECTED and declined, never returned folded.

## What

A new OCCT-free module `src/native/math/bspline_offset.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_ops` / `bspline_fit` / `bspline_skin`), **numsci-gated**
(`CYBERCAD_HAS_NUMSCI`, like `bspline_fit.cpp`) because the fit solves linear systems through the
numsci facade and the deviation metric projects through it. It reuses the Layer-1
`BsplineSurfaceData` type as both input (the surface `S`, which may be rational — its weights are
honoured through `nurbsSurfacePoint` / `surfaceNormal`) and output (the fitted offset, **non-rational**).

From *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 10 (offset approximation):

1. **Guards first** (`offsetSurface`) — sweep a dense analysis grid over `S`'s parameter domain. At
   every node evaluate the unit normal (degenerate / near-null normal → decline `DegenerateNormal`)
   and the principal curvatures `κ₁,κ₂` (via `surfaceDerivs`, the first + second fundamental forms).
   The offset map `S ↦ S + d·N` has Jacobian eigenvalues `(1 + d·κᵢ)`; it stays regular iff every
   `(1 + d·κᵢ) > 0`. If any node has `1 + d·κ ≤ 0`, the offset FOLDS (self-intersects) → decline
   `SelfIntersection`, reporting the minimum curvature radius it tripped on.
2. **Sample + fit + refine** — sample the true offset locus `O = S + d·N` on a `g × g` grid,
   INTERPOLATE a degree-`min(3,·)` tensor B-spline through it with the Layer-7 `interpolateSurface`
   (`lin_solve` collocation). Measure the fit's deviation from the locus GEOMETRICALLY (project a
   fixed check grid over the fitted surface onto `S` with `closest_point_on_surface`; the deviation
   is `max |dist − |d||`, which is parametrization-independent). Refine (double `g`, capped by
   `maxGrid`) until the deviation ≤ `tol` or the budget is spent; return the best fit with its true
   achieved error (flagged `ToleranceNotMet` if short — the surface is still returned, never widened).

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_offset.cpp`:
  1. **Offset distance (the core oracle)** — for a curved bicubic-bump patch, every point of the
     fitted offset surface, projected onto `S`, is at distance `|d|` within the reported tolerance
     (dense grid, both signs of `d`).
  2. **Analytic cross-check** — offsetting a NURBS-represented CYLINDER of radius `r` by `d` yields a
     surface on the coaxial cylinder of radius `r±d` (to fitting tol); a PLANE offsets to an exact
     parallel plane (~1e-9).
  3. **Error convergence** — the reported max offset error DECREASES monotonically as the fit grid
     refines (`~O(h⁴)` for cubic interpolation), reported honestly, never widened.
  4. **Self-intersection guard** — offsetting a tightly-curved dome by `d` past its minimum radius of
     curvature is DECLINED (`SelfIntersection`, `ok=false`, empty surface), NOT returned folded; the
     reported curvature radius ≈ the dome radius; a safe small offset of the same dome succeeds
     (guard is not a blanket reject); a degenerate near-null-normal patch declines.
- **SIM native-vs-OCCT parity** — OPTIONAL cross-check against OCCT `BRepOffsetAPI_MakeOffsetShape`
  / `BRepOffset` (a separate track; HOST is primary and sufficient).

## Scope

- Adds `src/native/math/bspline_offset.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob. Added to `native_math.h`.
- Adds `tests/native/test_native_nurbs_offset.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_fit` / `test_native_nurbs_skin`.
- Only `#include`s `bspline.h`, `bspline_ops.h` (Layer 1), `bspline_fit.h` (Layer 7), and the
  numerics facade — it does NOT modify them.
- **`cc_*` ABI unchanged.** Layer 5 is an internal geometry-algorithm library; its consumers are
  later shell/thicken features, not the app today. No ABI is added until a consumer needs it —
  consistent with the demand-driven policy.

## Non-goals

- **No solid thicken / shell / hollow** — offsetting BOTH faces of a solid and stitching side walls
  into a closed solid (the full `BRepOffset` shell) is a distinct B-rep construction and remains a
  documented residual. This module produces the single offset SURFACE the shell is built from.
- **No self-intersecting-offset TRIMMING** — recovering a valid offset from a folded region by
  trimming (rather than declining) is materially harder and is an explicit residual. This module
  DECLINES a folded offset honestly; it never returns folded geometry.
- **No rational offset fit** — the fitted offset is non-rational (empty weights). The input surface
  may be rational, but the approximation does not fit weights.
- No error-driven adaptive *knot* placement (only grid refinement); no automatic degree selection;
  no new `cc_*` ABI; no change to STEP admission, the tessellator, or any evaluator signature.
