# native-reference

## ADDED Requirements

### Requirement: Datum axis of a cylindrical or conical face, OCCT-free

The library SHALL compute the axis (a point on the axis + a unit direction) of a
cylindrical or conical face directly from the native B-rep, with **no OCCT**
(`src/native/reference/**` includes only `src/native/{math,topology}` headers;
zero OCCT includes) and header-only, in namespace `cybercad::native::reference`.
The axis SHALL be world-placed (the sub-shape `Location` baked into the frame).
Only a `Cylinder` or `Cone` face has an axis — matching the `cc_face_axis`
oracle; a `Plane`/`Sphere`/`Torus`/freeform face SHALL be an HONEST DECLINE, never
a fabricated axis. `cc_ref_axis_from_face` SHALL be identical to `cc_face_axis`.

#### Scenario: Cylinder face axis matches the closed-form axis (host, no OCCT)

- GIVEN a cylinder of radius `R` and height `H` along `+Z` with its base at the
  origin
- WHEN `faceAxis(lateralFace)` is called
- THEN the result SHALL be origin `(0,0,0)` and unit direction `(0,0,1)` within
  `1e-9`, AND `refAxisFromFace(lateralFace)` SHALL equal it

#### Scenario: A planar face has no axis and declines (host, no OCCT)

- GIVEN any planar face of a box
- WHEN `faceAxis(face)` (and `refAxisFromFace(face)`) is called
- THEN the result SHALL be a DECLINE (no value)

#### Scenario: Cylinder axis matches gp_Cylinder::Axis (sim, vs OCCT)

- GIVEN identical cylinders built natively and via `BRepPrimAPI_MakeCylinder`
- WHEN the native `faceAxis` of the lateral face is compared to the OCCT
  `gp_Cylinder::Axis`
- THEN the native direction SHALL be parallel to the OCCT axis direction within
  `1e-9` AND the native origin SHALL lie on the OCCT axis within `1e-7`

### Requirement: Datum plane from a planar face, OCCT-free

The library SHALL compute a datum plane (an on-plane origin + a unit outward
normal) from a planar face, OCCT-free and header-only. The normal SHALL be the
plane's Z axis, REVERSED for a `Reversed`-oriented face so it points outward
(matching `offset_face`/`replace_face`). The origin SHALL be a point that provably
lies on the planar face (the outer-wire vertex centroid). A non-planar face SHALL
be an HONEST DECLINE.

#### Scenario: Box face datum planes match the closed-form outward planes (host, no OCCT)

- GIVEN an axis-aligned box `2 × 3 × 4` at the origin
- WHEN `refPlaneFromFace` is called on the `z=0` and `x=2` faces
- THEN the `z=0` face SHALL yield normal `(0,0,-1)` and origin `(1,1.5,0)`, AND
  the `x=2` face SHALL yield normal `(1,0,0)` and origin with `x=2`, all within
  `1e-9`

#### Scenario: Datum plane matches gp_Pln with a coplanar origin (sim, vs OCCT)

- GIVEN identical boxes built natively and via `BRepPrimAPI_MakeBox`
- WHEN each native face datum plane is matched by outward normal to the OCCT face
  `gp_Pln`
- THEN every native normal SHALL equal the OCCT outward normal within `1e-9` AND
  the native origin SHALL be coplanar with the OCCT plane (`(o_native−o_occt)·n`
  within `1e-9`)

#### Scenario: A non-planar face declines (host, no OCCT)

- GIVEN the lateral (cylindrical) face of a cylinder
- WHEN `refPlaneFromFace(face)` is called
- THEN the result SHALL be a DECLINE (no value)

### Requirement: Datum axis from a straight edge, OCCT-free

The library SHALL compute the axis line (origin + unit direction) of a straight
edge, OCCT-free and header-only, world-placed. Only a `Line` edge has an axis —
matching the `cc_ref_axis_from_edge` oracle, which yields no `gp_Lin` for a
non-linear edge; a circular / elliptical / freeform edge SHALL be an HONEST
DECLINE, never a fabricated axis.

#### Scenario: Straight edge yields its line axis (host, no OCCT)

- GIVEN a straight edge from `(0,0,0)` to `(3,0,0)`
- WHEN `refAxisFromEdge(edge)` is called
- THEN the result SHALL be origin `(0,0,0)` and unit direction `(1,0,0)` within
  `1e-9`

#### Scenario: A circular edge declines (host, no OCCT)

- GIVEN a circular edge
- WHEN `refAxisFromEdge(edge)` is called
- THEN the result SHALL be a DECLINE (no value)

#### Scenario: Straight edge axis matches gp_Lin (sim, vs OCCT)

- GIVEN identical boxes built natively and via `BRepPrimAPI_MakeBox`
- WHEN each OCCT line edge is matched to a native straight edge whose axis is
  parallel and passes through the OCCT edge midpoint
- THEN every OCCT line edge SHALL have a matching native axis (direction within
  `1e-9`, midpoint on the line within `1e-7`)

### Requirement: Tangent-continuous edge chain, OCCT-free

The library SHALL grow a seed set of edge ids to the connected set of
tangent-continuous edges — edges meeting C1 at a shared vertex, `|t1·t2| ≥
cos(15°) = 0.966` on their unit tangents — by breadth-first walk over the
vertex→edge ancestry, OCCT-free and header-only, matching the `cc_tangent_chain`
oracle. `Line`/`Circle`/`Ellipse` tangents SHALL be closed-form. A freeform
(`BSpline`/`Bezier`) edge incident to the walk SHALL cause an HONEST DECLINE
(deferred to the oracle) rather than an under-grown chain. An empty grow (no C1
neighbour) is a VALID result, not a decline.

#### Scenario: Chain grows across a C1 joint and stops at a corner (host, no OCCT)

- GIVEN two collinear line edges sharing a vertex, and separately two
  perpendicular line edges sharing a vertex
- WHEN `tangentChain(root, {firstEdge})` is called on each
- THEN the collinear case SHALL return BOTH edges AND the perpendicular case
  SHALL return only the seed

#### Scenario: A line tangent to an arc joins the chain (host, no OCCT)

- GIVEN a straight edge whose end meets a circular arc tangentially at the shared
  vertex
- WHEN `tangentChain(root, {lineEdge})` is called
- THEN the result SHALL include BOTH the line and the arc

#### Scenario: Grow/stop decision agrees with the OCCT tangent oracle (sim, vs OCCT)

- GIVEN the collinear and perpendicular native pairs and the equivalent OCCT edges
- WHEN the native grow/stop decision is compared to OCCT `BRepAdaptor_Curve::D1`
  tangents at the shared vertex
- THEN the chain SHALL grow exactly when the OCCT `|t1·t2| ≥ 0.966` and stop
  otherwise

### Requirement: Outer-rim edge chain of a planar cap, OCCT-free

The library SHALL return the OUTER-wire edge ids of the planar cap face(s) that a
seed edge set bounds, OCCT-free and header-only, matching the `cc_outer_rim_chain`
oracle. A planar face SHALL qualify as a cap only if its plane contains ALL seed
vertices within `1.0` model unit (which selects the cap and rejects a
perpendicular side wall that shares only one seed edge); each qualifying cap
contributes its outer-wire edges (hole wires excluded). An empty result (no cap)
is a VALID result, not a decline.

#### Scenario: Seeding a cap edge returns the whole outer rim (host, no OCCT)

- GIVEN a planar quad face (a `4 × 4` rectangle) with a welded 4-edge outer wire
- WHEN `outerRimChain(face, {oneEdge})` is called
- THEN the result SHALL be exactly the 4 outer-wire edge ids

#### Scenario: Outer rim matches BRepTools::OuterWire (sim, vs OCCT)

- GIVEN a native rectangle cap and the equivalent OCCT planar face
- WHEN the native `outerRimChain` edge midpoints are compared to the edge
  midpoints of `BRepTools::OuterWire`
- THEN the two edge-midpoint sets SHALL match within `1e-7` and both SHALL have 4
  edges

### Requirement: In-plane offset of a planar polygon boundary, OCCT-free

The library SHALL offset a planar face's OUTER boundary loop by a signed distance
in the face plane, returning a closed xyz polyline, OCCT-free and header-only.
NATIVE SCOPE is a POLYGON boundary (every outer-wire edge is a straight line)
offset with SHARP (miter) joins — the case that provably coincides with
`BRepOffsetAPI_MakeOffset` (whose `GeomAbs_Arc` joins add no arc where the offset
SHARPENS a corner). The service SHALL HONESTLY DECLINE — leaving the oracle to
serve — a non-planar face, any non-line boundary edge (an arc), a growing convex
offset (OCCT would arc-round the corners), and a self-intersecting / collapsing
offset. It SHALL NEVER emit sharp corners where the oracle would emit arcs.

#### Scenario: Rectangle offset inward is the exact inner rectangle (host, no OCCT)

- GIVEN a `10 × 6` rectangle face in the `z=0` plane
- WHEN `offsetFaceBoundary(face, 1.0)` (inward) is called
- THEN the result SHALL be the 4 corners of the `[1,9] × [1,5]` rectangle within
  `1e-9`

#### Scenario: A growing convex offset declines (host, no OCCT)

- GIVEN the same rectangle face
- WHEN `offsetFaceBoundary(face, -1.0)` (a loop-growing convex offset) is called
- THEN the result SHALL be a DECLINE (no value), because OCCT would arc-round the
  convex corners

#### Scenario: Inward offset matches BRepOffsetAPI_MakeOffset area and bbox (sim, vs OCCT)

- GIVEN a native `10 × 6` rectangle offset inward by `1` and the OCCT inward
  `BRepOffsetAPI_MakeOffset` of the same wire
- WHEN the enclosed area and axis-aligned bounding box are compared
- THEN the native area SHALL equal the OCCT area (`32`) within `1e-6` AND the
  bounding boxes SHALL match within `1e-7`
