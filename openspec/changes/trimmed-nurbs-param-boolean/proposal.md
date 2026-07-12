# Proposal — trimmed-nurbs-param-boolean (NURBS roadmap Layer 8, Wave-I track I5)

## Why

The trimmed-NURBS B-rep boolean (roadmap Layer 3) has two halves. The 3-D half — SSI
(`src/native/ssi` + `src/native/boolean`) — intersects the operand SURFACES and produces the
cut curves, then their pcurves on each face (`constructPcurve`). The PARAMETER-SPACE half — given
two sets of trim loops (pcurves) now living in ONE face's (u,v) domain — must assemble the trimmed
result: compute, in 2-D, the union / intersection / difference of the two enclosed trim REGIONS and
emit the output loops (correctly nested outer/holes, correctly oriented). That parameter-space
region arithmetic was missing; it is tractable and airtight INDEPENDENT of the 3-D mesher (pure
planar computational geometry, verified against closed-form polygon / circular-lens areas), and it
is the box that turns SSI's cut pcurves + the operands' original trims into the boolean result's
trimmed face.

## What

Add `src/native/topology/trim_boolean.{h,cpp}` (additive; reuses `trimmed_nurbs` flatten + the
`classify` even-odd rule + the surface-consistent pcurve evaluator; no `shape.h` change, no `cc_*`
change; `src/native` stays OCCT-free, only existing math/topology is `#include`d):

1. **`trimRegionBoolean(regionA, regionB, op)`** for `op ∈ {Union, Intersect, Difference}` on two
   trim REGIONS (each an outer loop + hole loops) sharing a surface (u,v) domain. It runs the
   standard planar region-boolean arc-walk (Greiner–Hormann):
   - FLATTEN each loop's pcurve segments to a UV polygon via the shared `flattenTrimLoop` evaluator
     (a rational circle/ellipse pcurve flattens with no sag through the homogeneous lift); drop
     collinear interior vertices so a straight pcurve is a single edge (avoiding the classic
     crossing-through-vertex degeneracy).
   - INTERSECT the two regions' edges pairwise in (u,v) via a uniform-grid broad-phase (near-linear,
     so a machine-precision circular lens stays fast) and SPLIT both loops at every crossing.
   - CLASSIFY each arc inside/outside the OTHER region by an even-odd ray-cast of its midpoint (the
     same rule `trimmed_nurbs::classify` uses).
   - WALK the selected arcs to assemble the output loops per op, normalising orientation by NESTING
     depth (outer CCW / holes CW), so the total SIGNED area equals the true region area.

2. **A public `flattenTrimLoop()`** helper on `trimmed_nurbs` — a thin wrapper over the existing
   anonymous `flattenLoop`, so the region boolean flattens loops with the IDENTICAL, seam-consistent
   pcurve evaluator rather than re-deriving one.

3. **Honest-decline of degenerate overlaps** — a coincident-boundary edge (the two loops share a
   boundary segment) or a tangential-only touch (boundaries meet with no transversal crossing) is
   AMBIGUOUS for a region boolean and is declined (`Degenerate`), never resolved into a fabricated
   (and possibly wrong) region. No tolerance is ever widened to force a crossing.

## Scope / residuals

- The result loops are emitted as UV POLYLINES (the arcs the walk traced). A caller that needs
  pcurve segments back fits these polylines; the airtight oracles operate directly on the polylines
  (exact vertices + signed area). Re-fitting the output arcs back into rational pcurves (so a circle
  ∩ circle lens boundary is stored as two rational arcs rather than a fine polyline) is a documented
  residual — the AREA is machine-precision, the stored boundary is polygonal.
- Degenerate overlaps (coincident boundary / tangential-only) DECLINE honestly rather than resolve.
- Composes with the L3 SSI cut pcurves: SSI (stage 1) produces the seam pcurves that split the
  operands' trim loops; `constructPcurve` (stage 2) lifts them into (u,v); this module (the
  parameter-space assembly) then does the region arithmetic that yields the boolean result's trimmed
  face.

`cc_*` unchanged; `src/native` stays OCCT-free; `trimmed_nurbs`'s existing API is byte-unchanged
(only the additive `flattenTrimLoop` wrapper is exposed). No existing `FaceSurface`/`PCurve`/
`step_reader` consumer changes.
