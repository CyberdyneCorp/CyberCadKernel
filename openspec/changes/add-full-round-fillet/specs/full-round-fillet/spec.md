# full-round-fillet

## ADDED Requirements

### Requirement: Additive full-round-fillet C ABI
The library SHALL expose two additive C-ABI entry points: `CCShapeId
cc_full_round_fillet(CCShapeId body, int faceId)`, which replaces the narrow
`faceId` with a rolling-ball blend tangent to its two opposite neighbour faces
(consuming the middle face); and `CCShapeId cc_full_round_fillet_faces(CCShapeId
body, int leftFaceId, int middleFaceId, int rightFaceId)`, which consumes
`middleFaceId` and blends tangent to `leftFaceId` and `rightFaceId`. On success
each SHALL return a new body id; on failure `0`. These are the ONLY C ABI
additions; no existing `cc_*` signature or POD struct layout SHALL change.

#### Scenario: ABI addition is source-compatible
- GIVEN the host app previously linked the kernel
- WHEN it links the version with the two new entry points
- THEN every existing `cc_*` signature SHALL be unchanged AND the ABI contract
  test (`tests/test_abi.cpp`) SHALL still pass

#### Scenario: Host stub is a safe no-op
- GIVEN a build with no B-rep engine (the host stub)
- WHEN `cc_full_round_fillet` or `cc_full_round_fillet_faces` is called
- THEN it SHALL return `0` without crashing

### Requirement: Full round consumes the target face into a valid watertight solid
When a full round succeeds, the returned body SHALL be
`BRepCheck_Analyzer::IsValid` and watertight (a closed shell with no free
boundary), and the consumed middle face SHALL be GONE — the middle face id SHALL
NOT resolve to a face in the rebuilt body's face set.

#### Scenario: Rib round is valid, watertight, and consumes the top face
- GIVEN a body with a narrow middle face between two side faces (e.g. the top of
  a rib), on a booted iOS simulator
- WHEN `cc_full_round_fillet_faces(body, left, middle, right)` runs successfully
- THEN the result SHALL be `BRepCheck_Analyzer::IsValid` and watertight
- AND the `middle` face id SHALL no longer resolve to a face in the returned body

### Requirement: Blend is G1-tangent to both neighbours at the seam
The blend surface produced by a successful full round SHALL be G1-tangent to BOTH
neighbour faces at the two seams: sampling the blend-face normal and the
neighbour-face normal (`BRepLProp_SLProps`) at points along each seam, the angle
between them SHALL be within the documented tangency tolerance.

#### Scenario: Sampled seam normals agree on both neighbours
- GIVEN a successful full round on the rib body, on a booted iOS simulator
- WHEN the blend-face normal and each neighbour-face normal are sampled at
  matching points along both seams
- THEN at every sample the normals SHALL agree within the documented tangency
  tolerance (their dot product ≥ cos(tol)) for BOTH the left and right neighbour

### Requirement: Honest fallback to a standard fillet when a full round is not achievable
The operation SHALL, if a true face-consuming tangent full round cannot be built
for a given case, fall back to a standard edge fillet (`BRepFilletAPI_MakeFillet` on
the middle-face edges at a radius derived from the strip width) that is
`BRepCheck_Analyzer::IsValid`, and the case SHALL be recorded as deferred with the
measured tangency gap. The operation SHALL NOT report a faked G1-tangent /
face-consuming result, and SHALL NOT assert tangency it did not achieve.

#### Scenario: Unbuildable full round falls back to a valid fillet and is deferred
- GIVEN a case for which the tangent face-consuming blend cannot be built or is
  invalid, on a booted iOS simulator
- WHEN `cc_full_round_fillet` runs
- THEN it SHALL return a body produced by a standard edge fillet that IS
  `BRepCheck_Analyzer::IsValid`
- AND the change SHALL record this case as deferred with the measured tangency gap,
  NOT claim a full-round G1 pass for it

### Requirement: Deterministic and OCCT-guarded
The full round SHALL identify neighbours/seams and build the blend
deterministically so repeated runs on the same input are reproducible, and SHALL
be compiled only under `#ifdef CYBERCAD_HAS_OCCT`, leaving the host stub a safe
no-op.

#### Scenario: Repeated full round is reproducible
- GIVEN the same body and face ids on a booted iOS simulator
- WHEN `cc_full_round_fillet_faces` runs twice and both succeed
- THEN both results SHALL have the same exact volume and bounding box within a
  tight tolerance
