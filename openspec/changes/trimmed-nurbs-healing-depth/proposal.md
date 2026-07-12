# Proposal — trimmed-nurbs-healing-depth (NURBS roadmap Layer 8, healing depth / G5)

## Why

`trimmed-nurbs-healing` landed a clean 2-way pinch split and a non-rational pcurve construction,
leaving TWO explicit residuals: (1) a **3+-way / crossing pinch** (a vertex where three-or-more
loop strands meet, or two pinch points that cross — a figure-8-of-figure-8) still declined
`Unknown`; (2) `constructPcurve` fitted a genuinely **rational** trim curve (a circle/ellipse)
only as a NON-rational approximation, so a circular trim edge's pcurve sagged (~1e-3) rather than
being exact.

This change DEEPENS the healing to close both residuals where they are mathematically closable,
and to decline honestly (with a residual map) where they are not. It is purely additive: the
existing healing API and its default-OFF behaviour are byte-unchanged.

## What

Extend `src/native/topology/trimmed_nurbs.{h,cpp}` (no `shape.h` change, no `cc_*` change,
`src/native` stays OCCT-free, only existing math is `#include`d) with:

1. **`splitAtPinches()` / `splitTrimLoopAtPinches()`** (returning a `MultiSplitReport`) — the
   GENERAL N-way / crossing pinch resolver. At a pinch vertex where N≥3 strands meet, each INCOMING
   strand is paired with an OUTGOING strand by the **leftmost-turn** (largest signed CCW turn) rule
   — the standard orientation-preserving planar-subdivision face-tracing rule — decomposing the
   vertex into N simple sub-loops that share only the pinch point. Two pinch points that CROSS are
   resolved by ITERATING the single-vertex resolution to a **fixpoint**. The decomposition is
   **region- and area-preserving**: re-routing only re-partitions the same directed edges into
   cycles, so the sub-loops' total SIGNED area equals the original loop's, and the EVEN-ODD region
   is preserved (`classify()` combines the sub-loops by parity/XOR). A pinch whose strands do not
   alternate cleanly (a genuinely non-manifold touch), a sub-loop that stays self-touching after the
   fixpoint, or a signed-area mismatch, is DECLINED honestly (`ambiguous`), never force-split.

2. **`classify()` exposes it** — new `ClassifyOptions::splitNWay` (default OFF; requires
   `splitPinch`). When the 2-way split declines and `splitNWay` is ON, `classify()` falls back to
   `splitAtPinches()` and classifies the EVEN-ODD union of the resolved simple sub-loops. Default
   OFF keeps the prior 2-way-only behaviour byte-unchanged; a genuinely-ambiguous pinch still
   declines `Unknown`.

3. **RATIONAL pcurve construction** — `constructPcurve()` gains `ConstructOptions::rational`
   (default ON) and `PcurveConstruction::rational`. When the 3-D edge is RATIONAL (a NURBS/Bezier
   with a parallel weight vector) and the surface parametrization is affine in the curve's (u,v)
   region (a plane, exactly), the pcurve is built by PROJECTING the edge's control net to (u,v) and
   reusing the edge's EXACT degree, knots and weights — a rational pcurve that reproduces the 3-D
   circle through the surface to ~1e-9 (Piegl & Tiller homogeneous lift), where the old polynomial
   fit sagged. The round-trip fidelity is the gate: a surface whose parametrization is NOT affine in
   the region (a cylinder's transcendental angle coordinate) honestly FALLS BACK to the non-rational
   fit and reports its true, non-zero deviation — never a widened/faked tolerance.

## Scope / residuals

- N-way / crossing pinch splitting closes the "3+-way / crossing pinch" residual for cleanly-
  resolvable (alternating, area-preserving) configurations. **Healing across surface seams** and
  **re-parametrizing a badly-drifting pcurve** remain documented residuals; a non-alternating /
  degenerate pinch is DECLINED honestly (`ambiguous` / `Unknown`).
- The rational pcurve is EXACT on surfaces affine-in-(u,v) (planes). On a cylinder / non-affine
  surface a circular edge's pcurve is transcendental; the rational path honestly declines to the
  non-rational fit with a reported deviation. Weight-UNKNOWN rational edges recovered only
  approximately (Ma–Kruth) remain an explicit residual, reported never faked.

`cc_*` unchanged; `src/native` stays OCCT-free; the existing healing API + default-OFF behaviour is
byte-unchanged (additive only). No existing `FaceSurface`/`PCurve`/`step_reader` consumer changes.
