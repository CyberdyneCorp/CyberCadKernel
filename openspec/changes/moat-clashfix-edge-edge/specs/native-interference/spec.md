# native-interference

## MODIFIED Requirements

### Requirement: Interference / clash classification of two solids, OCCT-free

The library SHALL classify the interference of two solids A and B as **CLASH**
(their interiors intersect over a set of positive volume), **TOUCHING** (their
boundaries make contact but the interiors do not overlap), or **CLEAR** (a positive
clearance gap), with **no OCCT** (`src/native/analysis/interference.h` includes only
`src/native/{math,tessellate,boolean}` headers; zero OCCT includes) and header-only,
in namespace `cybercad::native::analysis`, consuming the landed B3 point-in-solid
classifier (`boolean::classifyPointInMesh` / `minDistanceToMesh` /
`pointTriangleDistance`) and the M0 mesh vocabulary (`tessellate::isWatertight`)
READ-ONLY.

CLASH SHALL be detected coplanar-safely: a boundary VERTEX or a boundary TRIANGLE
CENTROID of one solid that the B3 classifier reports strictly `In` the other. A
shared/coincident boundary SHALL read as contact (the classifier's `On` band), never
as a clash. When no penetration signature fires, the minimum triangle–triangle
distance SHALL decide TOUCHING (within the mesh-fidelity contact band) vs CLEAR (the
minimum distance is the reported clearance).

The minimum triangle–triangle distance SHALL be computed as the minimum over BOTH the
six VERTEX–FACE sub-tests AND the nine EDGE–EDGE sub-tests of each candidate triangle
pair — the exact tri–tri minimum for disjoint convex triangles, which is attained at
either a vertex–face pair OR an edge–edge pair. The edge–edge term SHALL be a
closed-form clamped segment–segment distance (handling the parallel / degenerate
zero-length cases) so that a flush contact whose closest approach is edge–edge — in
particular a COPLANAR OVERLAP where two faces cross with NO mutually contained vertex
(a plus/cross footprint) — is correctly reported as TOUCHING at distance 0 rather
than mis-classified as CLEAR. The service SHALL NOT modify the tessellator or the
boolean layer; it consumes them.

#### Scenario: Two overlapping axis-aligned boxes clash at the exact overlap volume (host, no OCCT)

- GIVEN box A = `[0,2]³` and box B = `[1,3]³` (their intersection is `[1,2]³`,
  volume 1)
- WHEN the interference of A and B is classified
- THEN the state SHALL be CLASH AND the reported overlap volume SHALL equal `1`
  within `1e-6` AND a witness point SHALL lie in the overlap interior `(1,2)³`

#### Scenario: Two disjoint boxes are clear at the exact gap (host, no OCCT)

- GIVEN box A = `[0,1]³` and box B = `[11,12]×[0,1]×[0,1]` (axis gap 10)
- WHEN the interference of A and B is classified
- THEN the state SHALL be CLEAR AND the reported minimum distance SHALL equal `10`
  within `1e-6` AND no clash witness SHALL be present

#### Scenario: Two face-touching boxes touch with zero overlap volume (host, no OCCT)

- GIVEN box A = `[0,1]³` and box B = `[1,2]×[0,1]×[0,1]` sharing the face `x=1`
- WHEN the interference of A and B is classified
- THEN the state SHALL be TOUCHING AND the reported overlap volume SHALL be `0`
  (a zero-volume boundary contact, not a clash)

#### Scenario: Coplanar plus-sign-cross faces touch via the edge–edge term (host, no OCCT)

- GIVEN box A = `[0,3]×[1,2]×[0,1]` (top face `z=1`) and box B = `[1,2]×[0,3]×[1,2]`
  (bottom face `z=1`), coplanar at `z=1` with footprints overlapping in a plus/cross
  (`[1,2]×[1,2]`) and NEITHER box having a corner inside the other
- WHEN the interference of A and B is classified
- THEN the state SHALL be TOUCHING at minimum distance `0` (the closest approach is
  A's top edges crossing B's bottom edges — an EDGE–EDGE pair the vertex–face tests
  miss), NOT the CLEAR verdict the vertex–face-only distance produced

#### Scenario: Raising one cross bar off the shared plane is clear at the exact gap (host, no OCCT)

- GIVEN the same plus-sign-cross footprints with box B raised so its bottom face is
  `z=1.5`, a `0.5` clearance above box A's top face `z=1`
- WHEN the interference of A and B is classified
- THEN the state SHALL be CLEAR AND the reported minimum distance SHALL equal `0.5`
  within `1e-6` (the edge–edge / face–face gap), confirming the edge–edge term
  reports the true clearance and does not fabricate a contact

#### Scenario: Coplanar-cross contact matches the OCCT oracle on the simulator (sim, OCCT)

- GIVEN the coplanar plus-sign-cross box pair built identically as native meshes and
  as OCCT shapes
- WHEN `cc_interference` runs under the native engine and `BRepExtrema_DistShapeShape`
  runs on the OCCT shapes
- THEN the native TOUCHING verdict SHALL match the OCCT oracle (distance `0` / contact,
  `Common` volume `0`) within the fixed parity tolerance
