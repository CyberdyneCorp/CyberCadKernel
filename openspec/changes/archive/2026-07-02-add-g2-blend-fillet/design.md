## Context

A standard fillet blends two faces with a circular-arc cross-section: it is
G1-continuous (tangent) at the seam but its curvature is constant `1/radius`,
which almost never equals the neighbour surface's curvature there — so curvature
JUMPS across the seam. G2 (curvature-continuous) blends remove that jump: the
blend's second fundamental form matches the neighbour's at the rail, so reflection
lines flow smoothly through the seam (class-A surfacing). OCCT's
`BRepFilletAPI_MakeFillet` is G1/circular only and exposes no G2 mode, so a
curvature-continuous blend must be built natively on OCCT's surface primitives
(`GeomFill_ConstrainedFilling`, high-degree B-spline surfaces, conic/rho blends).

This is the hardest, research-grade Phase-3 item. The governing principle is the
HONESTY RULE: the acceptance bar is a MEASURED curvature-continuity property, not
an assumed one. We build the best curvature-matched blend we can and then measure
the curvature gap at the seam with `BRepLProp_SLProps` / `GeomLProp` (second
order), on both the blend side and the neighbour side, and compare it to OCCT's
stock G1 fillet at the same seam. We only claim G2 if the gap is within tolerance
AND smaller than the G1 baseline; otherwise we report the number and defer.

Constraints:
- **Real, non-trivial bars**: (a) result `BRepCheck_Analyzer::IsValid` +
  watertight; (b) sampled seam curvature gap within a G2 tolerance; (c) that gap
  measurably smaller than OCCT's stock G1 fillet's gap at the same seam. (b) and
  (c) are the substantive checks — a G1 fillet would FAIL (b)/(c).
- **ABI**: one additive entry point; no existing signature changes.
- **Honesty**: if (b) is not met, report the measured gap and defer — never claim
  G2.

## Goals / Non-Goals

Goals:
- `cc_fillet_edges_g2(body, edgeIds, edgeCount, radius)` building a
  curvature-matched blend.
- Rebuild into a valid watertight solid.
- Measure seam curvature continuity (`BRepLProp`/`GeomLProp` 2nd order) on both
  sides and vs the stock G1 fillet; require within-tolerance AND better-than-G1 to
  claim G2.
- Honest deferral (report the gap) when G2 is not achieved.

Non-Goals:
- Guaranteeing G2 for every edge/geometry (research-grade; some cases defer).
- Changing the stock `cc_fillet_edges` (kept as the G1 baseline for comparison).
- G3+ / full class-A surface optimization.
- A native (non-OCCT) surface kernel (Phase 4).

## Decisions

- **Additive entry point mirroring `cc_fillet_edges`.** `cc_fillet_edges_g2(body,
  edgeIds, edgeCount, radius)` routes through a new `IEngine::fillet_edges_g2`
  virtual (default `engine_unsupported`); stub → `0`. Same argument shape as
  `cc_fillet_edges` so the app can swap it in per-edge.
- **Curvature-matched blend surface.** For each edge, get the two neighbour faces;
  build the blend cross-section as a higher-degree curve (degree ≥ 5 B-spline, or
  a conic/rho blend) constrained at both rails to match position + tangent (G1)
  AND curvature (G2) of the neighbour surfaces, then sweep it along the seam via
  `GeomFill_ConstrainedFilling` / a B-spline surface loft. The nominal `radius`
  sets the blend width; curvature at the rails is taken from the neighbours, not
  forced to `1/radius`.
- **Rebuild + sew.** Remove the original sharp edge's adjacent strip, insert the
  blend face(s), sew (`BRepBuilderAPI_Sewing` + `ShapeFix`) into a solid, gate on
  `BRepCheck_Analyzer::IsValid`.
- **Measure curvature continuity (the real bar).** At sample points along the
  seam, evaluate principal/normal curvatures with `BRepLProp_SLProps` (or
  `GeomLProp_SLProps` on the surfaces) on the blend side and the neighbour side;
  the curvature gap is the max normalized difference over the samples. Require it
  below a documented G2 tolerance.
- **Compare to the G1 baseline.** Build OCCT's stock `cc_fillet_edges` on the same
  edge/radius and measure its seam curvature gap the same way. Require the G2
  blend's gap to be measurably smaller (e.g. ≤ a fraction of the G1 gap). This
  proves the blend is genuinely better than circular, not just "valid".
- **Honesty gate.** The change claims G2 for a case ONLY if IsValid + watertight
  AND gap ≤ G2 tolerance AND gap < G1 baseline. If the gap exceeds tolerance, the
  change records the measured gap (and the G1 baseline for context) and marks the
  case deferred — it does NOT assert G2.

## Risks / Trade-offs

- **G2 may not be achievable for arbitrary seams.** Research-grade; some
  geometries will only reach "better than G1 but not within G2 tolerance". Those
  are reported with their measured gap and deferred — the honest outcome, not a
  faked pass.
- **Constrained fills can fail / self-intersect.** High-degree curvature-matched
  fills are numerically delicate; failures fall back (no body / or the G1 fillet)
  and are deferred, gated by IsValid.
- **Curvature-sampling robustness.** `BRepLProp` second-order values are sensitive
  near seam endpoints/degeneracies; sample in the seam interior and normalize by a
  characteristic curvature, documented, so the measured gap is meaningful.
- **G2 tolerance + "measurably better" threshold are judgement calls.** Pinned so
  a genuine curvature-continuous blend passes and a stock circular fillet fails
  both; recorded with the numbers so the claim is auditable.

## Migration Plan

1. Add `IEngine::fillet_edges_g2(body, edgeIds, edgeCount, radius)` (default
   `engine_unsupported`); stub → `0`.
2. OCCT adapter: per-edge neighbour extraction; curvature-matched blend
   cross-section + `GeomFill`/B-spline sweep; rebuild + sew; gate on
   `BRepCheck_Analyzer::IsValid`.
3. Add `cc_fillet_edges_g2` to the header + facade (additive); `test_abi`
   unchanged.
4. Curvature measurement harness: sample `BRepLProp_SLProps` 2nd-order on both
   sides of the seam for the G2 blend AND for the stock G1 fillet; compute both
   gaps.
5. iOS-sim check: result IsValid + watertight; G2 gap within tolerance AND
   measurably smaller than the G1 baseline gap → claim G2; else report the gap and
   defer.
6. Host stub `0`; `run-sim-suite.sh` unchanged (additive).
7. `openspec validate --all --strict`; update `ROADMAP.md` Phase 3 status with the
   measured curvature numbers (and any deferred cases).

## Open Questions

- Which blend construction (constrained high-degree B-spline vs conic/rho blend)
  most reliably reaches within-tolerance G2 on the test fixtures — decided
  empirically from the measured gaps; documented.
- The G2 curvature tolerance and the "measurably better than G1" threshold
  (fraction of the G1 gap) — pinned so a genuine curvature-continuous blend passes
  and a circular fillet fails; recorded with the numbers.
- The seam sampling scheme (count, interior margin, curvature normalization) that
  makes the measured gap robust and reproducible.
- Which real geometries reach genuine G2 vs only "better than G1" (deferred) —
  recorded honestly from the sim results with the measured gaps.
