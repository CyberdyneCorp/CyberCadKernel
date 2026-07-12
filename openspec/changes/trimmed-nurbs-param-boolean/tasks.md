# Tasks — trimmed-nurbs-param-boolean

## Implementation

- [x] Add a public `flattenTrimLoop()` to `trimmed_nurbs.{h,cpp}` (thin wrapper over the existing
      anonymous `flattenLoop`) so the region boolean shares the seam-consistent pcurve evaluator.
- [x] Add `trim_boolean.h`: `TrimBoolOp`, `TrimBoolStatus`, `ResultLoop`, `TrimBoolResult`,
      `TrimRegion`, `TrimBoolOptions`, and `trimRegionBoolean()` with the contract docs.
- [x] Implement `trim_boolean.cpp` (Greiner–Hormann arc-walk), decomposed into phase helpers:
  - [x] `flattenRegion()` — flatten + `dropCollinear` (avoid crossing-through-vertex degeneracy).
  - [x] `findCrossings()` — uniform-grid broad-phase pairwise segment crossings + dedupe.
  - [x] `resolveNoCrossings()` — disjoint / nested resolution via genuine region interior points.
  - [x] `buildRings()` / `crossingsOnEdge()` — mated crossing rings.
  - [x] `anyTransversal()` — tangential-only-touch honest decline.
  - [x] `selectAndTrace()` — per-op arc selection + canonical GH trace.
  - [x] `assembleLoops()` — nesting-depth orientation normalisation (outer CCW / holes CW).
- [x] Honest-decline coincident-boundary / tangential-only overlaps (`Degenerate`).
- [x] Wire `trim_boolean.cpp` (library glob) + `test_native_trim_boolean` into `CMakeLists.txt`.

## Tests (regression, airtight oracles)

- [x] Disjoint squares — Union = both, Intersect = empty, Difference = A.
- [x] Overlapping squares — A∪B / A∩B / A∖B areas exact to ≤1e-10 (machine-zero).
- [x] Hole handling — annulus ∪/∩ disk (in-hole island + straddle), correct signed area.
- [x] Circular-lens intersection (rational pcurves) — area ≤1e-8 of the closed form.
- [x] Coincident-edge overlap — honest `Degenerate` decline.

## Verification

- [x] `src/native` OCCT-free (0 OCCT/Geom/BRep/TK refs in changed code).
- [x] `cc_*` ABI byte-unchanged (additive only).
- [x] `ctest` green (trim_boolean 25/25 checks; related topology/tessellate/construct/heal suites).
- [x] Cognitive complexity within the compilers/geometry band (no Critical function).
- [x] `openspec validate --all` passes.
