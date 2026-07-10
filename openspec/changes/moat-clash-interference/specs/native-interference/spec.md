# native-interference

## ADDED Requirements

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
minimum distance is the reported clearance). The service SHALL NOT modify the
tessellator or the boolean layer; it consumes them.

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

### Requirement: Overlap volume from the native COMMON with a two-sided self-verify

On a CLASH the library SHALL report the overlap VOLUME as the enclosed volume of the
native boolean COMMON `A ∩ B`, computed by the engine (the header stays boolean-free).
The value SHALL be admitted ONLY when it passes a two-sided self-verify: the COMMON
solid is watertight AND its volume does not exceed `min(V(A), V(B))` (an intersection
cannot be larger than either operand). When the native COMMON is not robustly
available (a null / non-watertight result for a curved or near-tangent overlap, or a
mesh-soup operand with no B-rep), or the volume fails the self-verify, the service
SHALL DECLINE rather than emit a wrong volume, and the engine SHALL fall through to
the OCCT `BRepAlgoAPI_Common` + `BRepGProp` oracle. A wrong overlap volume SHALL NEVER
be returned.

#### Scenario: A fully-nested box clashes (host, no OCCT)

- GIVEN an outer box `[0,10]³` and an inner box `[3,5]³` fully inside it
- WHEN the interference of the inner and outer boxes is classified
- THEN the state SHALL be CLASH AND a witness SHALL be present

#### Scenario: Overlap volume matches the OCCT oracle on the simulator (sim, OCCT)

- GIVEN two overlapping native solids and the same two solids built as OCCT shapes
- WHEN `cc_interference` runs under the native engine and `BRepAlgoAPI_Common` +
  `BRepGProp` runs on the OCCT shapes
- THEN the native overlap volume SHALL match the OCCT `Common` volume within the
  fixed parity tolerance AND the CLASH / TOUCHING / CLEAR state and clearance SHALL
  match `BRepExtrema_DistShapeShape`

### Requirement: Honest decline over a wrong interference verdict

The service SHALL return an UNKNOWN decline — the engine returning a clean error so
the facade falls through to the OCCT oracle — rather than a wrong clash flag or
overlap volume, whenever the mesh evidence is ambiguous: a non-watertight operand
mesh, or a boundary point the B3 ray-parity classifier cannot decide (`Unknown`) that
lies strictly inside the other solid's bounding box and beyond the contact band (the
only region a masked interior overlap could occupy). A spurious grazing `Unknown` on
a far face, or an `Unknown` at a contact seam, SHALL NOT force a decline. The facade
`cc_interference` SHALL return 0 with `decided == 0` and `cc_last_error` set on a
decline, and SHALL NEVER report a clash flag or overlap volume it cannot verify.

#### Scenario: A non-watertight operand declines (host, no OCCT)

- GIVEN an operand whose boundary mesh is not watertight (an open shell) and a
  second watertight box overlapping its region
- WHEN the interference is classified
- THEN the state SHALL be UNKNOWN (a decline) AND no clash SHALL be asserted

#### Scenario: The facade reports an honest decline, never a fabricated verdict (host, no OCCT)

- GIVEN two bodies for which the native engine cannot robustly produce the overlap
  volume
- WHEN `cc_interference(a, b, out)` is called
- THEN it SHALL return `0` with `out->decided == 0` and `cc_last_error` set, and
  `out->clash` SHALL NOT be `1`
