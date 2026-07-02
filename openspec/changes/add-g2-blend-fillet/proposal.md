## Why

OCCT's fillets are **G1 / circular only**: `BRepFilletAPI_MakeFillet` produces a
constant- or variable-radius circular-arc cross-section blend that is tangent
(G1-continuous) to the neighbour faces but has a curvature JUMP at the seam (the
neighbour is flat/curved at some curvature, the arc is at `1/radius`), so a
reflection line "breaks" at the fillet edge. High-end class-A / industrial-design
work wants a **curvature-continuous (G2)** blend where curvature varies smoothly
across the seam and reflection lines flow through (`ROADMAP.md` Phase 3; GitHub
#284; `occt-usage` §Fillets & chamfers limitation). OCCT does not offer this, so
it is a native feature — built on OCCT surfaces (`GeomFill`, B-spline surfaces)
but delivering behaviour OCCT's stock fillet API cannot.

This is explicitly a **research-grade** item: genuine G2 across an arbitrary seam
is hard, so the honesty rule governs it — we build the best curvature-continuous
blend we can, MEASURE the curvature continuity on both sides of the seam, and only
claim G2 if the numbers show it; otherwise we report the measured gap and mark it
deferred.

## What Changes

- Add an additive `cc_*` entry point:
  - `CCShapeId cc_fillet_edges_g2(CCShapeId body, const int *edgeIds, int
    edgeCount, double radius)` — build a curvature-continuous blend along the given
    edges at the nominal `radius`, replacing the stock circular fillet with a G2
    (or best-achievable) blend surface.
- Implement in the OCCT adapter, building on OCCT surfaces:
  - For each edge, extract the two neighbour faces and, across the blend strip,
    build a **higher-degree blend surface with curvature matching** — e.g. a
    `GeomFill_ConstrainedFilling` / degree-≥5 B-spline / conic (rho) blend whose
    cross-section matches BOTH the position/tangent (G1) AND the second-order
    (curvature, G2) of the neighbour surfaces at the two rails, instead of a
    circular arc.
  - Rebuild the solid with the blend faces sewn in (`BRepBuilderAPI_Sewing` +
    `ShapeFix`), gated on `BRepCheck_Analyzer::IsValid`.
- **Measure and report curvature continuity honestly**: sample the second-order
  surface properties (`BRepLProp_SLProps` / `GeomLProp`) on the blend side and the
  neighbour side of each seam, compute the curvature gap, and compare it to OCCT's
  stock G1 fillet at the SAME seam. The check requires (a) the blend is
  `BRepCheck_Analyzer::IsValid` + watertight, (b) the curvature gap is within a G2
  tolerance, AND (c) it is measurably smaller than the stock G1 fillet's gap. If
  (b) is not met, the change reports the measured gap and marks the case
  **deferred** — it does NOT claim G2.

C ABI change is **ADDITIVE only**: one new entry point; no existing `cc_*`
signature or POD layout changes. OCCT-only; the host stub returns `0`.

## Capabilities

### New Capabilities
- `g2-blend`: a curvature-continuous (G2) blend fillet (`cc_fillet_edges_g2`)
  that replaces OCCT's circular G1 fillet cross-section with a higher-degree
  curvature-matched blend surface, rebuilt into a `BRepCheck_Analyzer::IsValid`,
  watertight solid, with the seam curvature continuity MEASURED (`BRepLProp` /
  `GeomLProp` second-order) and required to be within a G2 tolerance AND
  measurably better than OCCT's stock G1 fillet at the same seam. Research-grade:
  when genuine G2 is not achieved the measured curvature gap is reported and the
  case is marked deferred — G2 is never claimed unless the numbers show it. Builds
  on `engine-adapter` (OCCT `GeomFill` / B-spline surfaces + sewing).

## Impact

- **Contract**: delivers the #284 curvature-continuous fillet that OCCT's
  G1-only fillet cannot; `occt-usage` §Fillets & chamfers limitation. Additive
  alongside the stock `cc_fillet_edges` (unchanged).
- **App**: gains a class-A-grade G2 blend option for edges where reflection-line
  quality matters.
- **Build**: OCCT-only; guarded by `CYBERCAD_HAS_OCCT`; host stub returns `0`.
- **Determinism**: fixed blend construction + fixed sampling; the measured
  curvature gap is reproducible.
- **Risk / honesty (primary)**: genuine G2 is research-grade. The acceptance is a
  MEASURED curvature-continuity property vs the stock G1 baseline; if the numbers
  do not show G2, the change honestly reports the gap and defers — no faked G2
  claim.
