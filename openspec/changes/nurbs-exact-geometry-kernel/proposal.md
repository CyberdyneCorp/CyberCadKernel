# Proposal — nurbs-exact-geometry-kernel (NURBS roadmap Layer 1)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack whose
**bottom layer** is the *exact-NURBS geometry kernel*: the fundamental geometric algorithms
that manipulate a B-spline/NURBS curve or surface **while preserving the geometry it
represents** — knot insertion, knot removal, degree elevation, degree reduction, and
splitting — plus robust rational evaluation. Every higher NURBS layer (general SSI, exact
B-rep boolean, NURBS fillet, exact sweep/loft) is built on these primitives; they are the
prerequisite for all of it.

Today the kernel **evaluates** NURBS (`src/native/math/bspline.{h,cpp}`, `bezier.{h,cpp}` —
Piegl & Tiller *The NURBS Book* A2.1–A4.4: `findSpan`, `basisFuns`, `dersBasisFuns`,
`curvePoint`, `curveDerivs`, `nurbsCurvePoint/Derivs`, tensor-product surfaces, Bézier),
admits B-spline curves/surfaces from STEP (`step_reader.cpp`), and stores them in the B-rep
(`EdgeCurve` / `FaceSurface` in `shape.h`). But the **construction algorithms** that
transform one NURBS representation into an equivalent one are entirely absent. Confirmed by
search: no knot insertion, no knot removal, no degree elevation, no degree reduction, no
splitting. This change builds that layer to completion.

This is the one NURBS layer worth building **now** even under the demand-driven policy,
because it is (a) small and well-bounded (*The NURBS Book* Chapter 5, ~10 algorithms), (b)
**OCCT-free and substrate-free** (pure control-point + knot arithmetic — no NumPP/SciPP),
and (c) uniquely **exactly verifiable**: every operation here has a closed-form oracle — the
result must represent the *same* curve/surface pointwise (degree reduction: bounded-error /
exact-when-reducible). That makes it the highest-confidence, lowest-risk NURBS work
available, and it unblocks every later layer.

## What

A new OCCT-free module `src/native/math/bspline_ops.{h,cpp}` (namespace
`cybercad::native::math`, beside the existing evaluators), providing for both **curves** and
**tensor-product surfaces**, and **rational as well as non-rational** (rational handled by
lifting `(x,y,z,w)` to homogeneous 4-space, running the algorithm, projecting back):

1. **Robust rational evaluation** — audit and harden the existing evaluators against the
   degenerate cases the construction algorithms will stress: interior knots at full
   multiplicity, clamped/unclamped ends, endpoint parameters, high degree, and a documented
   guard against non-positive weights. (Evaluation exists; this hardens + regression-tests
   it as the foundation the ops rely on.)
2. **Knot insertion** — insert a knot `r` times (Boehm, A5.1 curve / A5.3 surface) and
   **knot refinement** (insert a whole new-knot vector at once, A5.4 — the Oslo-class
   refinement).
3. **Knot removal** — remove a knot up to `num` times within a tolerance, returning how many
   removals were actually achievable (A5.8 curve / surface direction).
4. **Degree elevation** — raise degree by `t` while preserving the curve/surface exactly
   (A5.9 curve / A5.10 surface).
5. **Degree reduction** — reduce degree by one with a reported max error; exact when the
   geometry is genuinely reducible, honest bounded error otherwise (A5.11).
6. **Splitting / subdivision** — split a curve at a parameter into two curves, or a surface
   along a U or V isoparameter into two patches, via knot insertion to full multiplicity;
   the pieces reconstruct the original pointwise. Includes **Bézier decomposition**
   (A5.6/A5.7) as the full-multiplicity special case.
7. **Reparametrization** — affine remap of a knot vector to a new `[a,b]` domain
   (poles/weights unchanged), the trivial companion these algorithms need.

## Verification (two gates, the exact-oracle is the whole point)

- **HOST-analytic (no OCCT), the primary gate.** Every op is checked by its closed-form
  invariant on a dense parameter sample and by round-trips: insert↔remove is identity;
  elevate-then-reduce is identity when exact; split's two pieces reconstruct the original;
  refinement equals repeated single insertion; a decomposed Bézier segment set re-evaluates
  to the source curve. Partition-of-unity and endpoint interpolation preserved. Tolerances
  are FIXED (~1e-12 for exact ops; the removal/reduction error is the *reported* bound,
  never a widened pass threshold).
- **SIM native-vs-OCCT parity.** Cross-check against OCCT `BSplCLib` / `Geom_BSplineCurve` /
  `Geom_BSplineSurface` (`InsertKnots`, `IncreaseDegree`, `RemoveKnot`, `Segment`) on the
  booted simulator — the same conventions (flat knots, row-major U-outer poles) make this a
  direct numeric compare.

## Scope

- Adds `src/native/math/bspline_ops.{h,cpp}` — OCCT-free, substrate-free, always-on (no
  `CYBERCAD_HAS_NUMSCI` guard). Compiles into the core lib via the existing `src/native` glob.
- Adds `tests/native/test_native_nurbs_ops.cpp` (host, always-on) and
  `tests/sim/native_nurbs_ops_parity.mm` (SIM), each wired into CMake / `run-sim-suite.sh`.
- **`cc_*` ABI unchanged.** Layer 1 is an internal geometry-algorithm library; its consumers
  are the *later* NURBS layers, not the app. No ABI surface is added until a consumer needs
  it — consistent with the demand-driven principle. (Additive `cc_*` exposure is a
  later change if/when a surfacing feature calls for it.)

## Non-goals

- No general NURBS↔NURBS SSI, no exact-NURBS B-rep boolean, no NURBS fillet/offset, no
  sweep/loft surfacing, no fitting/approximation — those are Layers 2–8 of the NURBS
  roadmap, explicitly out of scope here and still demand-gated per `docs/NURBS-SCOPE.md`.
- No new `cc_*` ABI. No change to STEP admission, the tessellator, or any existing evaluator
  signature (hardening is additive/behaviour-preserving, regression-gated byte-identical for
  every currently-passing evaluation case).
