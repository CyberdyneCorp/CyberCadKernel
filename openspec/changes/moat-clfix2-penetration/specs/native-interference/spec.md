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
`pointTriangleDistance`) and the Möller–Trumbore ray–triangle kernel
(`boolean::mollerTrumbore`) and the M0 mesh vocabulary (`tessellate::isWatertight`)
READ-ONLY.

CLASH SHALL be detected coplanar-safely by TWO complementary signatures:

1. **ENCLOSURE** — a boundary VERTEX or a boundary TRIANGLE CENTROID of one solid that
   the B3 classifier reports strictly `In` the other. A shared/coincident boundary
   SHALL read as contact (the classifier's `On` band), never as a clash.
2. **PASS-THROUGH** — an EDGE of one solid that pierces a FACE of the other
   TRANSVERSALLY through its interior (a segment–triangle crossing strictly interior to
   BOTH the segment and the triangle). This SHALL detect a positive-volume overlap where
   NEITHER solid has a vertex nor a centroid inside the other (a bar poking clean through
   a slab, or a slab pierced by a thin bar) — the case the enclosure signature alone
   misses. The pass-through test SHALL be SEAM-SAFE: a coplanar face contact or an
   endpoint seat SHALL NOT register as a pierce, so TOUCHING and CLEAR are unaffected.
   The pass-through scan SHALL be evaluated only when the enclosure signature found no
   contained point.

When no penetration signature fires, the minimum triangle–triangle distance SHALL
decide TOUCHING (within the mesh-fidelity contact band) vs CLEAR (the minimum distance
is the reported clearance).

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

The classifier SHALL NOT widen any tolerance to obtain a CLASH verdict; the
pass-through gates SHALL only ever REJECT a grazing/endpoint crossing (they never
accept a wider contact), and the contact band SHALL be unchanged. When the mesh
evidence is ambiguous (a non-watertight operand, or a boundary point the ray-parity
classifier declines) the classifier SHALL return an honest decline (Unknown) rather
than a guessed verdict.

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

#### Scenario: A bar poking clean through a slab clashes with no contained vertex (host, no OCCT)

- GIVEN slab B = `[0,10]×[0,10]×[0,1]` and bar A = `[4,6]×[4,6]×[-5,20]`, where the bar
  passes clean through the slab so NEITHER solid has a vertex NOR a triangle centroid
  inside the other (the bar's ends stick out both slab faces; the slab is wider than
  the bar) — the enclosure signature alone finds no contained point
- WHEN the interference of A and B is classified
- THEN the state SHALL be CLASH (via the PASS-THROUGH signature: a bar edge pierces a
  slab face transversally through its interior) AND a witness SHALL be present within
  the true overlap box `[4,6]×[4,6]×[0,1]` AND the reported overlap volume through the
  facade SHALL equal `4` within `1e-6` (the bar∩slab intersection box) AND the verdict
  SHALL be independent of operand order

#### Scenario: A bar seated flush on the slab top touches, not clashes (host, no OCCT)

- GIVEN the same footprints with bar A lifted so its BOTTOM face `z=1` is flush with the
  slab's TOP face `z=1` (a coplanar seat)
- WHEN the interference of A and B is classified
- THEN the state SHALL be TOUCHING (no edge pierces an interior; the contact is
  coplanar / at endpoints) AND the reported overlap volume SHALL be `0`

#### Scenario: A bar held above the slab is clear at the exact gap (host, no OCCT)

- GIVEN the same footprints with bar A raised so its bottom face is `z=1.5`, a `0.5`
  clearance above the slab top `z=1`
- WHEN the interference of A and B is classified
- THEN the state SHALL be CLEAR AND the reported minimum distance SHALL equal `0.5`
  within `1e-6`, confirming the pass-through term does not fabricate a contact

#### Scenario: The pass-through pose matches the OCCT oracle on the simulator (sim, OCCT)

- GIVEN the bar-through-slab pose built identically as native meshes and as OCCT shapes
- WHEN `cc_interference` runs under the native engine and `BRepAlgoAPI_Common` +
  `BRepExtrema_DistShapeShape` run on the OCCT shapes
- THEN the native CLASH verdict SHALL match the OCCT oracle (`Common` volume `> 0`,
  equal to the closed-form `4`) within the fixed parity tolerance, and the touching /
  gapped variants SHALL match the oracle's TOUCHING / CLEAR verdicts
