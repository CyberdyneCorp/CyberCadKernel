# Design — moat-m4rc-rational-curve-trims

## Context

M4-rational admitted a foreign **rational B-spline SURFACE** via the combined
`RATIONAL_B_SPLINE_SURFACE` Part-21 record: read the surface weights into
`FaceSurface::weights`, and every downstream stage (the rational-aware faithful guard,
the M0 mesher, the engine self-verify) was already weight-aware. The **curve/edge**
basis path was deliberately left non-rational (the M4-rational proposal listed "any
rational curve read" as out of scope). This change closes that one residual: a foreign
**rational B-spline CURVE** used as an edge's 3D geometry or as a `TRIMMED_CURVE` basis.

The parse geometry is the exact one-dimension-down analogue of M4-rational. The relevant
`step_reader.cpp` facts (verified against the current tree):

- `curve()` (line 881) declines every combined record at line 883
  (`if (!r || r->combined) { return std::nullopt; }`); line 899 handles only the
  `B_SPLINE_CURVE_WITH_KNOTS` keyword; line 901 documents RATIONAL_B_SPLINE_* as out of
  scope.
- `bsplineCurve()` (line 985) reads the non-rational keyword form, leaves `weights`
  empty, and enforces `knots.size() == poles.size() + degree + 1`.
- The combined-record scan `findSub` / `hasSub` (lines 1444–1449) and the surface
  precedent `rationalBsplineSurface()` (lines 1512–1542, with the shared `fillBsplineGrid`
  at 1457) already exist.
- `evalEdge()` (line 2325) — the reader's faithful-guard curve evaluator — evaluates the
  `Kind::BSpline` case with `math::curvePoint` unconditionally (line 2340).
- The tessellator `edgeCurveLocal()` (`tessellate/edge_mesher.h:114`) already routes to
  `math::nurbsCurvePoint` when `weights` is non-empty (lines 137–141).
- `EdgeCurve::weights` (`topology/shape.h:190`), `PCurve::weights` (`shape.h:203`), and
  `math::nurbsCurvePoint` (`math/bspline.h:77`) already exist.

## Goals / Non-Goals

**Goals.** Admit ONE foreign rational B-spline curve as edge/trim geometry, meshing
watertight matching OCCT `STEPControl_Reader` + `BRepMesh`; keep every other path
byte-identical; keep the honest decline on any malformed / non-faithful / non-reachable
rational curve; stay OCCT-free in `src/native/**`; no `cc_*` ABI change; no tessellator
change.

**Non-Goals.** A general NURBS-curve importer; periodic/closed rational curves needing a
seam close; rational 2D PCURVEs from the file (the reader synthesises its own analytic
pcurves and ignores the STEP pcurves); any surface widening beyond M4-rational.

## Decisions

### D1 — Dispatch on the combined record, reuse the existing sub-record machinery

Add the rational-curve check **before** the `r->combined` decline in `curve()`:

```
if (r->combined) {
  if (hasSub(*r, "RATIONAL_B_SPLINE_CURVE")) return rationalBsplineCurve(*r);
  return std::nullopt;   // every other combined curve keeps its current decline
}
```

This mirrors `surface()` (line 1017) exactly. No new tokenizer; `Record::subs` already
holds the split sub-records for combined instances.

### D2 — Field layout of the combined rational curve record

OCCT `STEPControl_Writer` splits the rational curve across sub-records:

- `B_SPLINE_CURVE(degree, (#pole…), form, closed, selfInt)` — degree + pole refs
- `B_SPLINE_CURVE_WITH_KNOTS((mults), (knots), spec)` — RLE knot vector
- `RATIONAL_B_SPLINE_CURVE((weight…))` — a flat weight list, one per pole

`rationalBsplineCurve()` reads `degree` and the pole list from `B_SPLINE_CURVE`, the
RLE-expanded knots via the existing `expandKnots(...)` from `B_SPLINE_CURVE_WITH_KNOTS`,
and the weight list from `RATIONAL_B_SPLINE_CURVE` into `EdgeCurve::weights` in pole
order. The shared degree/pole/knot parse is factored into a small helper
(`fillBsplineCurve`, the curve analogue of `fillBsplineGrid`) so the non-rational
`bsplineCurve()` keyword path is re-pointed at it and stays byte-identical.

### D3 — Weight validation (the decline gate)

Decline (`std::nullopt` → OCCT) when any of:

- a required sibling sub-record (`B_SPLINE_CURVE`, `B_SPLINE_CURVE_WITH_KNOTS`,
  `RATIONAL_B_SPLINE_CURVE`) is missing or mistyped;
- `knots.size() != poles.size() + degree + 1` (the same invariant the non-rational path
  enforces);
- `weights.size() != poles.size()` (cardinality mismatch);
- any weight is non-finite or **not strictly positive**.

A weight is NEVER clamped and no tolerance is introduced by this well-formedness check —
identical policy to `rationalBsplineSurface()` (line 1535).

### D4 — One rational route in the guard evaluator; everything else unchanged

`evalEdge()` (line 2325) is the ONLY reader-side evaluator not yet rational-aware. Change
its `Kind::BSpline` arm to:

```
return c.weights.empty()
    ? math::curvePoint(c.degree, {c.poles…}, {c.knots…}, t)
    : math::nurbsCurvePoint(c.degree, {c.poles…}, {c.weights…}, {c.knots…}, t);
```

This mirrors `edgeCurveLocal` (edge_mesher.h:137–141) and the rational-aware surface
evaluator. For every non-rational edge (`weights` empty) it is byte-identical. With this,
the per-edge faithful guard (`pcurveEdgeFaithful` → `pcurveFaithful`) evaluates the
rational edge correctly. The trim machinery (`trimmedCurve`, `trimmedRange`,
`curveRange`'s B-spline arm at 1622–1627) is kind-agnostic and reused unchanged: a
rational curve reaches `curve()` either directly as an EDGE_CURVE's `#curve` (via the
SURFACE_CURVE unwrap at line 891) or as a `TRIMMED_CURVE` basis (line 914), and in both
cases the covered knot sub-domain is recovered from the two `PARAMETER_VALUE` trims.

### D5 — Reachability of a foreign rational edge curve (measured, honest)

Unlike a rational surface (which OCCT authors readily for any NURBS patch), OCCT's
`STEPControl_Writer` typically emits edge geometry as analytic (line/circle/ellipse) or
non-rational B-spline curves; a genuinely **rational** edge curve is authored only for
specific inputs (e.g. a conic edge exported as a rational-quadratic B-spline, or a
trimmed rational free-form edge). Task 4 measures whether such a foreign file is robustly
producible from the OCCT writer. If it is NOT, the SIM gate records the measured gap and
the slice ships as: rational-curve READ + HOST ANALYTIC gate proven, with the SIM
native-vs-OCCT parity demonstrated on a native-authored combined record round-tripped
through OCCT — and the honest decline documented for the un-authorable foreign case. No
wrong solid is ever emitted either way.

## Verification strategy (the two gates)

**Gate A — HOST ANALYTIC (no OCCT linked).** Build an `EdgeCurve` whose rational-quadratic
weights make it reproduce an EXACT circular arc of known closed-form geometry — degree 2,
poles at the arc's control triangle, weights `{1, cos(Δ/2), 1}` for a single span of
half-angle `Δ` (the standard rational-quadratic conic). Assert `evalEdge(t)` equals the
closed-form circle point `O + R(cos θ, sin θ)` within `1e-9` at a grid of parameters, and
that the per-edge faithful guard ACCEPTS the faithful edge and REJECTS a deliberately
perturbed off-curve weight — proven against an independent closed-form oracle with no OCCT
symbol linked.

**Gate B — SIM native-vs-OCCT (booted iOS simulator, OCCT linked).** Import a solid whose
edge/trim boundary is a rational B-spline curve with `cc_set_engine(1)` and assert the
native solid's enclosed volume, surface area, watertight status, triangle envelope, and
sub-shape topology match the OCCT `STEPControl_Reader` + `BRepMesh_IncrementalMesh` oracle
within tolerance. A malformed / non-positive-weight record and a non-faithful edge both
DECLINE and import byte-identically to `cc_set_engine(0)`.

## Risks / honest-decline triggers

- **Foreign reachability (D5).** If OCCT will not author a rational edge curve for any
  in-scope solid, the SIM parity is shown on a native-authored round-trip and the foreign
  gap is recorded — the read + guard are still proven, and the decline is honest.
- **Closed/periodic rational curve.** A closed rational curve whose seam needs a wrap
  beyond the knot-span clamp declines (out of scope this slice).
- **Guard non-convergence.** If `pcurveFaithful` cannot reconstruct the boundary pcurve
  within the scale-relative tolerance, the edge declines → OCCT.
- **Self-verify discard.** An admitted rational edge whose native mesh is not watertight
  or is off-volume is discarded by the engine → OCCT.

## Cognitive complexity

`rationalBsplineCurve()` and `fillBsplineCurve()` are small linear readers (validate →
fill → return), well within the compiler/parser band (25–35). `curve()` gains one
combined-record branch mirroring `surface()`. `evalEdge()` gains one ternary. No function
is pushed out of band.
