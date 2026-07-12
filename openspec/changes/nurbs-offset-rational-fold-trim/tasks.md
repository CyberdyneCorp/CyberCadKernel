# Tasks — nurbs-offset-rational-fold-trim

## 1. Header — additive API
- [x] 1.1 `OffsetResult` gains additive default-valued fields: `bool trimmed`, and the kept
      parameter rectangle `keptU0/keptU1/keptV0/keptV1`. Existing fields byte-unchanged.
- [x] 1.2 Declare `offsetSurfaceRational(S, d, tol, startGrid, maxGrid)` — rational-aware
      offset that fits a rational approximant preserving the input weight pattern.
- [x] 1.3 Declare `offsetSurfaceTrimmed(S, d, tol, startGrid, maxGrid)` — offset that, on a
      fold, trims to the maximal fold-free parameter rectangle instead of declining.

## 2. Implementation
- [x] 2.1 Factor the existing sample→fit→refine body into an internal
      `offsetCore(S, d, tol, [u0,u1], [v0,v1], rational, …)` so `offsetSurface`,
      `offsetSurfaceRational`, and the trimmed path share one loop. `offsetSurface` behaviour
      unchanged (delegates over the full domain, non-rational).
- [x] 2.2 Rational path — sample the offset locus, evaluate the input's effective rational
      weight `w(u,v)` per node, lift each sample to homogeneous `(w·x,w·y,w·z,w)`, fit with
      `interpolateRationalSurface`; measure geometric deviation as before; fall back to the
      non-rational fit for non-rational input or if it does not improve.
- [x] 2.3 Fold-free rectangle — build the dense regularity map `(1 + d·κ) > εfold`, run a
      deterministic largest-all-true-rectangle scan, map to a parameter rectangle shrunk
      inward by ½ cell; honest-decline if kept area < `kMinKeptFraction`.
- [x] 2.4 `offsetSurfaceTrimmed` — if the full domain is fold-free, behave like
      `offsetSurface` with `trimmed=false`; otherwise offset over the kept rectangle and set
      `trimmed=true` + the kept-rectangle fields. Never return a folded surface.

## 3. Host-analytic gate (OCCT-free, numsci-gated)
- [x] 3.1 Rational analytic exactness — offset of a NURBS SPHERE by `d` lies on radius `R±d`
      (≤1e-6); a NURBS CYLINDER by `d` on radius `r±d` (≤1e-6); a plane translates by `d·N̂`
      (≤1e-9), all via `offsetSurfaceRational`, and the fitted offset is RATIONAL (weights
      non-empty) for the conic inputs.
- [x] 3.2 Rational round-trip — `offsetSurfaceRational(S, d)` then offsetting that by `−d`
      recovers `S` within tolerance (dense point comparison).
- [x] 3.3 Fold-trim correctness — a high-curvature bump offset past its min radius over PART
      of the domain returns `trimmed=true` with a valid offset over the kept rectangle:
      `(1 + d·κ)` keeps CONSTANT positive sign across the kept region (fold-free), every kept
      point is at distance `|d|` from `S`, and the kept rectangle is a strict sub-region.
- [x] 3.4 Honest decline — a surface that folds over essentially all of the domain still
      returns `SelfIntersection`, `ok=false`, empty surface (no meaningful fold-free region).

## 4. Docs & close-out
- [x] 4.1 Update `docs/NURBS-SCOPE.md` Layer-5 row: rational offset + robust
      self-intersection trimming residuals resolved (single non-rational surface → adds
      rational + fold-trim). Keep solid thicken/shell as `bspline_thicken`/`bspline_shell`.
- [x] 4.2 `openspec validate --all --strict`; full host ctest green (no regression);
      `cc_*` ABI byte-unchanged; `src/native` OCCT-free; existing `offsetSurface` unchanged.
