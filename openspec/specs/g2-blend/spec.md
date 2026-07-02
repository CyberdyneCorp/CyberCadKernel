# g2-blend Specification

## Purpose
TBD - created by archiving change add-g2-blend-fillet. Update Purpose after archive.
## Requirements
### Requirement: Additive G2 blend-fillet C ABI
The library SHALL expose one additive C-ABI entry point `CCShapeId
cc_fillet_edges_g2(CCShapeId body, const int *edgeIds, int edgeCount, double
radius)` that builds a curvature-continuous (G2, or best-achievable) blend along
the given edges at the nominal `radius`, returning a new body id on success and
`0` on failure. This is the ONLY C ABI addition; no existing `cc_*` signature or
POD struct layout SHALL change, and the stock `cc_fillet_edges` (the G1 baseline)
SHALL be unchanged.

#### Scenario: ABI addition is source-compatible
- GIVEN the host app previously linked the kernel
- WHEN it links the version with `cc_fillet_edges_g2`
- THEN every existing `cc_*` signature SHALL be unchanged AND the ABI contract
  test (`tests/test_abi.cpp`) SHALL still pass

#### Scenario: Host stub is a safe no-op
- GIVEN a build with no B-rep engine (the host stub)
- WHEN `cc_fillet_edges_g2` is called
- THEN it SHALL return `0` without crashing

### Requirement: G2 blend produces a valid watertight solid
When `cc_fillet_edges_g2` returns a non-zero body, that body SHALL be
`BRepCheck_Analyzer::IsValid` and watertight (a closed shell with no free
boundary).

#### Scenario: Blended body is valid and watertight
- GIVEN a body with a fillettable edge, on a booted iOS simulator
- WHEN `cc_fillet_edges_g2` builds a blend on that edge and returns a non-zero
  body
- THEN the result SHALL be `BRepCheck_Analyzer::IsValid` and watertight

### Requirement: Measured seam curvature continuity, better than the G1 baseline
Curvature continuity across the blend seam SHALL be MEASURED, not assumed:
sampling the second-order surface properties (`BRepLProp_SLProps` / `GeomLProp`)
at seam-interior points on the blend side and the neighbour side, the normalized
curvature gap SHALL be computed. To CLAIM G2 for a case, the blend SHALL be
`BRepCheck_Analyzer::IsValid` + watertight, the measured curvature gap SHALL be
within the documented G2 tolerance, AND that gap SHALL be measurably smaller than
the gap of OCCT's stock G1 circular fillet (`cc_fillet_edges`) built on the same
edge and radius (a stock G1 fillet does NOT satisfy this).

#### Scenario: G2 blend curvature gap is within tolerance and beats G1
- GIVEN a body and edge on a booted iOS simulator, blended once with
  `cc_fillet_edges_g2` and once with the stock `cc_fillet_edges` at the same radius
- WHEN the seam curvature gap is measured for both via `BRepLProp_SLProps`
  second-order sampling on both sides of the seam
- THEN to claim G2 the `cc_fillet_edges_g2` gap SHALL be within the documented G2
  tolerance AND SHALL be measurably smaller than the `cc_fillet_edges` (G1) gap

#### Scenario: Stock G1 fillet fails the G2 bar (control)
- GIVEN the stock `cc_fillet_edges` result on the same edge
- WHEN its seam curvature gap is measured the same way
- THEN it SHALL exceed the G2 tolerance (its curvature is discontinuous at the
  seam), confirming the check is non-trivial

### Requirement: Honest reporting when G2 is not achieved
This capability is research-grade: G2 SHALL NOT be claimed unless the measured
numbers show it. If the measured curvature gap exceeds the G2 tolerance (or the
blend cannot be built), the change SHALL record the measured curvature gap (and
the G1 baseline gap for context) and mark the case as deferred; it SHALL NOT
assert G2 continuity it did not measure, and SHALL NOT fake a passing curvature
result.

#### Scenario: Sub-G2 result is reported and deferred, not claimed
- GIVEN a case where the best blend `cc_fillet_edges_g2` can build has a measured
  seam curvature gap ABOVE the G2 tolerance, on a booted iOS simulator
- WHEN the curvature is measured
- THEN the change SHALL record the measured gap (and the G1 baseline) and mark the
  case deferred, and SHALL NOT claim G2 continuity for it

### Requirement: Deterministic and OCCT-guarded
The blend SHALL be built and sampled deterministically so repeated runs on the
same input produce the same measured curvature gap, volume, and bounding box, and
SHALL be compiled only under `#ifdef CYBERCAD_HAS_OCCT`, leaving the host stub a
safe no-op.

#### Scenario: Repeated G2 blend is reproducible
- GIVEN the same body, edges, and radius on a booted iOS simulator
- WHEN `cc_fillet_edges_g2` runs twice and both succeed
- THEN both results SHALL have the same measured seam curvature gap, exact volume,
  and bounding box within a tight tolerance

