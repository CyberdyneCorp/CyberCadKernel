# native-topology Specification

## Purpose
TBD - created by archiving change add-native-brep-topology. Update Purpose after archive.
## Requirements
### Requirement: OCCT-free, host-buildable topology library

The native topology model SHALL live under `src/native/topology/` and SHALL include
NO OCCT header in any of its translation units, so that it compiles and unit-tests
with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT and NO simulator.
The library MAY include `src/native/math`. The library SHALL link no OCCT; OCCT
SHALL appear ONLY in the simulator native-vs-OCCT parity test (a test-only importer
that loads an OCCT shape and compares), never in the library itself. This change
SHALL make no `cc_*` signature or POD struct layout change and SHALL NOT wire the
library into the active engine.

#### Scenario: Library builds on the host without OCCT
- GIVEN the sources under `src/native/topology/`
- WHEN they are compiled with `clang++ -std=c++20` with no OCCT and no simulator (the `src/native/math` headers available)
- THEN the build SHALL succeed AND no compiled translation unit SHALL include any OCCT header

#### Scenario: No ABI change and no engine wiring
- GIVEN this change applied
- WHEN the public headers and the active engine are inspected
- THEN no `cc_*` signature or POD struct layout SHALL have changed AND the native topology library SHALL NOT be reachable through the `cc_*` facade

### Requirement: B-rep topology data model with orientation and location

The library SHALL provide the boundary-representation shape kinds `Vertex`, `Edge`,
`Wire`, `Face`, `Shell`, `Solid`, and `Compound` (mirroring `TopAbs_ShapeEnum`).
Each shape SHALL be an oriented, located *use* over a shared, immutable *underlying*
entity — a `(underlying, orientation, location)` triple — so that a sub-shape shared
between two parents is one underlying entity referenced by two uses. **Orientation**
SHALL be `Forward` or `Reversed` (with `Internal` and `External` reserved to match
`TopAbs_Orientation`) and SHALL compose through nesting (Forward is identity,
Reversed flips), matching the OCCT `TopAbs::Compose` convention. **Location** SHALL
be a `TopLoc`-style rigid placement backed by a `native-math` `Transform` that
composes down the hierarchy, so a sub-shape's effective placement is the product of
its ancestors' placements. All structure SHALL be deterministic.

#### Scenario: Orientation round-trips and composes (host)
- GIVEN shapes built on the host with no OCCT
- WHEN a use is reversed twice and when orientations are composed through nesting
- THEN reversing twice SHALL yield the original orientation AND the composed orientation SHALL match the documented Forward-identity / Reversed-flip table

#### Scenario: Location composes down the hierarchy (host)
- GIVEN a shape whose sub-shape carries a non-identity location nested under a parent that also carries a location
- WHEN the sub-shape's effective placement is resolved
- THEN it SHALL equal the product of the ancestor locations (a `native-math` `Transform` compose) within the documented fp64 tolerance

#### Scenario: A shared sub-shape is one underlying entity (host)
- GIVEN two faces that share one edge
- WHEN the shared edge is inspected from each face
- THEN both uses SHALL reference the same single underlying edge entity (not two copies)

#### Scenario: Data model matches OCCT structure (parity)
- GIVEN a representative OCCT `TopoDS_Shape` imported into the native model on a booted iOS simulator
- WHEN the shape kinds, per-use orientations, and resolved locations are compared against the OCCT shape (`TopoDS`, `TopAbs`, `TopLoc_Location`)
- THEN the native shape kinds and resolved orientations SHALL match exactly AND the resolved locations SHALL match within the documented tight fp64 tolerance

### Requirement: Geometry attachment to vertices, edges, and faces

The library SHALL attach geometry to the underlying entities, referencing
`native-math` descriptors without duplicating them. A `Vertex` SHALL carry a
`Point3` and a `double` tolerance. An `Edge` SHALL carry a handle to a 3D curve, a
parameter range `[first, last]`, and zero or more **pcurves**, each a 2D
parameter-space curve associated with a specific `Face` (so the same edge can carry
a distinct pcurve per adjacent face, matching `BRep_Tool::CurveOnSurface`). A `Face`
SHALL carry a handle to a surface, an ordered list of boundary `Wire`s with the
first designated the **outer** boundary and the remainder **inner** (holes) per a
documented convention, and a `double` tolerance. All attached values SHALL be fp64.

#### Scenario: Vertex, edge, and face read back their geometry (host)
- GIVEN a vertex, an edge, and a face built with known attached geometry on the host with no OCCT
- WHEN their geometry is read back
- THEN the vertex SHALL return its `Point3` and tolerance, the edge SHALL return its 3D curve and `[first, last]` range, AND the face SHALL return its surface, its outer wire, and its inner (hole) wires in the documented order, all within the documented fp64 tolerance

#### Scenario: An edge carries a distinct pcurve per face (host)
- GIVEN an edge shared by two faces, each with its own pcurve of that edge
- WHEN the pcurve is requested for each face
- THEN the edge SHALL return the pcurve associated with that specific face

#### Scenario: Attached geometry matches OCCT (parity)
- GIVEN an OCCT shape imported into the native model on a booted iOS simulator
- WHEN vertex points, edge curves sampled over their ranges, and face surfaces sampled over their domains are compared against `BRep_Tool` (`Pnt`, `Curve`, `Surface`, `Tolerance`)
- THEN the native attached-geometry values and tolerances SHALL match the OCCT oracle within the documented tight fp64 tolerance

### Requirement: Stable sub-shape identification with deterministic enumeration

The library SHALL assign every distinct underlying sub-shape a **stable integer id**
under a single deterministic depth-first enumeration that visits a shape before its
sub-shapes and recurses children in a fixed per-type order, matching the
`TopExp::MapShapes` conventions. The first time an underlying entity is encountered
it SHALL receive the next integer id; a subsequent encounter of a shared sub-shape
SHALL reuse the same id (no duplicate id, no duplicate node). The enumeration order
and the id assignment SHALL be deterministic and stable for a given shape across
repeated runs.

#### Scenario: Sub-shape counts and ids are stable (host)
- GIVEN a known shape built on the host with no OCCT
- WHEN it is enumerated twice
- THEN the sub-shape counts per type SHALL match the known values AND the id assigned to each sub-shape SHALL be identical across the two enumerations

#### Scenario: A shared sub-shape has a single id (host)
- GIVEN two faces that share one edge
- WHEN the shape is enumerated
- THEN the shared edge SHALL be assigned exactly one id, referenced from both faces

#### Scenario: Enumeration order and ids match TopExp::MapShapes (parity)
- GIVEN an OCCT shape imported into the native model on a booted iOS simulator
- WHEN the native deterministic enumeration is compared against `TopExp::MapShapes` over the same shape
- THEN the native visitation order and the resulting per-sub-shape ids SHALL match the OCCT `TopExp::MapShapes` order (shape before sub-shapes, shared sub-shapes mapped once)

### Requirement: Traversal by explorer over a shape by type

The library SHALL provide an **explorer** that iterates the sub-shapes of a shape
filtered by a requested `ShapeType` (e.g. all `Edge`s of a `Solid`), yielding each
matching *use* with its resolved orientation and location, in the deterministic
enumeration order — the `TopExp_Explorer` analogue. The explorer SHALL be a lazy
forward traversal over the same deterministic walk that backs enumeration, so its
order agrees with the enumeration and it need not materialize a full list.

#### Scenario: Explorer yields the typed sub-shapes in enumeration order (host)
- GIVEN a shape built on the host with no OCCT
- WHEN an explorer is opened over it for a given `ShapeType`
- THEN it SHALL yield exactly the sub-shapes of that type, each with its resolved orientation, in the same order as the deterministic enumeration

#### Scenario: Explorer matches TopExp_Explorer (parity)
- GIVEN an OCCT shape imported into the native model on a booted iOS simulator
- WHEN a native explorer for a given type is compared against a `TopExp_Explorer` of the same type over the OCCT shape
- THEN the native explorer SHALL yield the same sub-shapes in the same order with the same resolved orientations as `TopExp_Explorer`

### Requirement: Ancestry from sub-shape to parents

The library SHALL build, over a shape, a mapping from a sub-shape to the list of its
parent sub-shapes for a given child and parent type (e.g. `Edge` → adjacent `Face`s,
`Vertex` → incident `Edge`s) — the `TopExp::MapShapesAndAncestors` analogue. The
parent list for each child SHALL be in first-encountered order and de-duplicated per
parent. The ancestry SHALL be consistent with the explorer: every child the explorer
yields under a parent SHALL appear in that child's ancestor list, and vice-versa.

#### Scenario: Edge-to-face ancestry is symmetric with the explorer (host)
- GIVEN a two-face shell sharing one edge, built on the host with no OCCT
- WHEN the edge → face ancestry is built and each face is explored for edges
- THEN the shared edge's ancestor list SHALL contain both faces AND each of those faces SHALL yield the shared edge when explored for edges

#### Scenario: A boundary edge has a single parent face (host)
- GIVEN a shape with an edge on the outer boundary of exactly one face
- WHEN the edge → face ancestry is built
- THEN that edge's ancestor list SHALL contain exactly one face

#### Scenario: Ancestry matches TopExp::MapShapesAndAncestors (parity)
- GIVEN an OCCT shape imported into the native model on a booted iOS simulator
- WHEN the native edge → face (and vertex → edge) ancestry is compared against `TopExp::MapShapesAndAncestors` over the OCCT shape
- THEN the native ancestor lists SHALL match the OCCT oracle in membership and documented order

### Requirement: BRep_Tool-style geometry accessors

The library SHALL provide free-function accessors that read the geometry off a shape
by resolving the underlying entity through the use's location — the `BRep_Tool`
analogue and the only read path into attached geometry: `pnt(Vertex)` (the vertex
point in resolved placement), `tolerance(shape)`, `curve(Edge)` returning the 3D
curve with its `first`/`last` range, `curve_on_surface(Edge, Face)` returning the
edge's pcurve on that face, and `surface(Face)`. Values SHALL be returned in the
resolved (located) placement and SHALL be fp64.

#### Scenario: Accessors return located geometry (host)
- GIVEN a vertex, an edge, and a face carrying a non-identity location, built on the host with no OCCT
- WHEN `pnt`, `curve`, `curve_on_surface`, `surface`, and `tolerance` are called
- THEN `pnt` SHALL return the vertex point transformed by the resolved location, `curve` SHALL return the 3D curve with its parameter range, `curve_on_surface` SHALL return the pcurve for the given face, `surface` SHALL return the face surface, AND `tolerance` SHALL return the stored tolerance, all within the documented fp64 tolerance

#### Scenario: Accessors match BRep_Tool (parity)
- GIVEN an OCCT shape imported into the native model on a booted iOS simulator
- WHEN the native accessors are compared against `BRep_Tool` (`Pnt`, `Curve`, `CurveOnSurface`, `Surface`, `Tolerance`) over the same sub-shapes
- THEN the native accessor results SHALL match the OCCT oracle within the documented tight fp64 tolerance

### Requirement: Deterministic topology structure and traversal

All native topology structure and traversal SHALL be deterministic: the enumeration
order, the assigned sub-shape ids, the explorer iteration order, and the ancestry
lists SHALL be identical across repeated runs over the same shape.

#### Scenario: Repeated enumeration, exploration, and ancestry are identical (host)
- GIVEN any shape and a fixed traversal request
- WHEN the enumeration, an explorer, and an ancestry map are computed twice on the host
- THEN the two enumeration orders and id assignments SHALL be identical, the two explorer sequences SHALL be identical, AND the two ancestry maps SHALL be identical

### Requirement: FaceSurface carries an analytic Torus kind additively

The `FaceSurface` surface descriptor SHALL support an analytic **`Torus`** kind alongside the existing
`Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`, and `Bezier` kinds, referencing the `native-math`
`Torus` descriptor without duplicating it. A torus `FaceSurface` SHALL carry its placement frame (`Ax3`),
a **major** radius (axis → tube-centre distance, reusing the existing `radius` field), and a **minor**
radius (the tube cross-section radius, a new `minorRadius` field), such that the attached surface equals
`math::Torus{frame, majorRadius = radius, minorRadius}`. The addition SHALL be **additive and
byte-neutral** for every existing kind: the new `Torus` enumerator SHALL be appended so no existing
enumerator's value changes, and the new `minorRadius` field SHALL default such that every existing kind's
in-memory meaning of `radius` / `semiAngle` is unchanged and no existing construction, accessor, or
serialization keys on the enum ordinal. All attached values SHALL be fp64. The topology library SHALL
remain OCCT-free and host-buildable.

#### Scenario: A face reads back an attached torus surface (host)
- GIVEN a `Face` built with an attached `Torus` `FaceSurface` (known frame, major radius `R`, minor radius `r`) on the host with no OCCT
- WHEN its surface is read back
- THEN the face SHALL return a surface of kind `Torus` whose frame, major radius, and minor radius equal the attached values within the documented fp64 tolerance, corresponding to `math::Torus{frame, R, r}`

#### Scenario: Adding the Torus kind leaves every existing kind byte-identical (host)
- GIVEN faces built with each existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`, `Bezier`) on the host with no OCCT, before and after the `Torus` kind and `minorRadius` field are added
- WHEN each face's surface descriptor is read back and its in-memory layout is inspected
- THEN every existing kind SHALL read back IDENTICAL values (the `radius` / `semiAngle` semantics are unchanged, `minorRadius` defaults so it does not affect any existing kind, and no existing enumerator's ordinal changed) — the addition is purely additive

