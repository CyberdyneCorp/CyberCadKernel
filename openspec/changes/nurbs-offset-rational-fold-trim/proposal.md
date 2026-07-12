# Proposal — nurbs-offset-rational-fold-trim

## Why

The landed Layer-5 surface offset (`src/native/math/bspline_offset.{h,cpp}`,
`offsetSurface`) fits the true offset locus `O = S + d·N` **non-rationally** and, on a
self-intersection, DECLINES the whole request. Two documented residuals remain (recorded
in the `nurbs-surface-offset` change and `docs/NURBS-SCOPE.md`):

1. **Rational offset.** The offset of a *rational* NURBS surface (e.g. an exact NURBS
   cylinder) is, for the analytic cases that matter (cylinder → coaxial cylinder radius
   `r±d`, sphere → concentric sphere radius `R±d`), itself an EXACT rational surface with
   the SAME weights. The non-rational fitter cannot reproduce those conics exactly and
   loses the weight structure. A rational-aware offset that samples the offset locus in
   homogeneous form and fits a rational approximant with the input's weight pattern
   recovers those conics to a tight bound (≤1e-6 for analytic inputs).

2. **Fold trimming.** A CAD "thicken/offset" that folds over PART of the domain should not
   fail wholesale — it should TRIM away the folded (self-intersecting) region and keep the
   valid remainder. The current guard only detects the fold and declines. We add detection
   of the fold-free sub-region and return a valid offset over the maximal fold-free
   rectangle, plus a report of the trimmed-away parameter region. Only when NO meaningful
   fold-free region remains do we honest-decline.

## What changes

- **Additive API** in `bspline_offset.h` (existing `offsetSurface` byte-unchanged):
  - `offsetSurfaceRational(S, d, tol, …)` — samples the offset locus, lifts to homogeneous
    form using the input surface's per-node weight, and fits a RATIONAL approximant with
    prescribed weights (`interpolateRationalSurface`). Falls back to the non-rational fit if
    the input is non-rational or the rational fit does not improve. Verified to ≤1e-6 on
    sphere/cylinder analytic offsets; round-trips (offset by `d` then `−d` recovers `S`).
  - `offsetSurfaceTrimmed(S, d, tol, …)` — on a fold, finds the maximal fold-free
    axis-aligned sub-rectangle `[uₐ,u_b]×[vₐ,v_b]` of the parameter domain, offsets over
    just that trimmed region (reusing the existing sample→fit→refine core), and returns the
    valid offset plus the trimmed-away region. Honest-declines only when no fold-free region
    of meaningful area remains. NEVER returns a self-intersecting surface as valid.
  - `OffsetResult` gains additive fields (`trimmed`, `keptU0/1`, `keptV0/1`) — default-valued
    so existing callers are unaffected.

- **Regression tests** wired into the numsci-gated block (host-analytic, OCCT-free):
  sphere/cylinder/plane analytic exactness under the rational path, rational offset
  round-trip, and fold-trim fold-free correctness (Jacobian sign constant over the kept
  region, kept region matches the true offset locus).

## Non-goals

- Solid thicken / shell / hollow (that is `bspline_thicken` / `bspline_shell`).
- Non-rectangular (trimmed-curve) fold boundaries — the kept region is an axis-aligned
  parameter rectangle, the simplest honest fold-free region. A curved trim boundary is a
  later residual.
- Weight ESTIMATION from unweighted points (Ma–Kruth) — the rational fit uses the input's
  prescribed weight pattern only, never fabricates weights.

## Impact

- `src/native/math/bspline_offset.{h,cpp}` — additive entry points, existing API and ABI
  byte-unchanged. `cc_*` facade untouched. `src/native` stays OCCT-free.
- `tests/native/test_native_nurbs_offset.cpp` + `CMakeLists.txt` (test already wired; new
  cases added to the existing target).
- `docs/NURBS-SCOPE.md` Layer-5 row: rational offset + fold-trimming residuals resolved.
