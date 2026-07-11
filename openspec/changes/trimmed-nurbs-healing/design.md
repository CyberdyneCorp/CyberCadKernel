# Design — trimmed-nurbs-healing (NURBS roadmap Layer 8, healing extension)

## Where the defects live — the flattened polyline + segment-join gaps

A gap between two consecutive pcurve segments, or a near-coincident pair of pcurve endpoints, is
not visible in a single pcurve — it is a property of the JOIN between two segments (or the closing
join from the last segment back to the first). So healing operates on the flattened loop polyline,
plus a `joinGaps` vector recording `‖end of segment k − start of segment k+1‖` at each join.

`flattenLoopForHeal()` produces both: the polyline keeps every sampled vertex (only exact
bit-identical oversampling duplicates are dropped, so a 1e-8 inter-segment gap stays visible), and
`joinGaps` carries the raw join distances. The production `flattenLoop()` (with its 1e-15 dedup and
closing-duplicate drop) is unchanged and still used when `heal` is off.

## Distinguishing a GAP from a long shape edge

The core problem: a rectangle's long side is a huge polyline step but is NOT a gap, while a 1e-8
step at a segment join IS a gap. Consecutive-vertex proximity alone cannot tell them apart. The
`joinGaps` vector does: only a JOIN can be a gap.

`healLoop()`:
1. **Pass 0 (triage)** — any `joinGap > tol` is a genuine large gap → `largeGap=true`, decline. This
   is the ONLY reliable large-gap test (a long shape edge is never a join).
2. **Pass 1 (weld)** — walk consecutive polyline vertices; a step in `(0, tol]` is a gap: merge the
   two vertices to their midpoint (each moves ≤ tol/2). The closing edge is welded explicitly. Small
   join gaps and near-coincident vertex pairs are both closed here.
3. **Pass 2 (pinch)** — a repeated non-adjacent vertex (within `tol`) is a self-touch: `pinch=true`,
   decline. Split is a residual; this slice declines honestly.

`tol = gapTol · max(extent, 1)` is scale-relative (extent = loop UV bounding-box diagonal).

## Region-preservation — why a heal never flips a classification

A ray-cast verdict for a point P changes only if the boundary crosses P, i.e. only if some boundary
point moves across P. Every weld moves a vertex by at most `tol/2` (midpoint of two vertices ≤ tol
apart). Therefore any P whose distance to the boundary exceeds `tol/2` keeps its verdict. Points
within `tol` of the boundary are the `OnBoundary` band anyway. A genuine large gap is NOT welded
(declined) and a pinch is NOT split (declined) — so a heal only nudges an already-near-valid loop;
it never re-routes the boundary. The host gate proves this empirically with a SWEEP of gap sizes
across `[1e-10, 0.9·tol]`, asserting no probe's `In`/`Out` verdict ever flips vs the exact loop.

## classify() integration — opt-in, no regression

`ClassifyOptions::heal` (default true) + `healGapTol`. A small `preparedLoop()` helper flattens +
heals each loop and returns the raycast polyline plus an `ok` flag; a heal failure (large gap /
pinch / degeneracy) makes `ok=false` → `classify` declines `Unknown` — identical to the pre-healing
decline path, so every previously-declined broken loop still declines and every previously-accepted
valid loop is unchanged (its joins are already exact ⇒ nothing welded ⇒ same polyline). With
`heal=false` the exact former code path (`flattenLoop` + `loopWellFormed`) runs.
