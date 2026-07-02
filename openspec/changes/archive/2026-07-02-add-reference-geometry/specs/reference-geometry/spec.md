# reference-geometry

## ADDED Requirements

### Requirement: Additive reference-geometry C ABI
The library SHALL expose six additive C-ABI entry points that compute datum
reference geometry and return it as POD out-arrays with a `1`/`0` success flag,
creating NO shape handle: `int cc_ref_plane_from_points(const double p0[3], const
double p1[3], const double p2[3], double out6[6])`, `int
cc_ref_plane_offset(const double origin[3], const double normal[3], double dist,
double out6[6])`, `int cc_ref_plane_from_face(CCShapeId body, int faceId, double
out6[6])`, `int cc_ref_axis_from_points(const double a[3], const double b[3],
double out6[6])`, `int cc_ref_axis_from_edge(CCShapeId body, int edgeId, double
out6[6])`, and `int cc_ref_axis_from_face(CCShapeId body, int faceId, double
out6[6])`. For a plane, `out6 = [ox,oy,oz, nx,ny,nz]` (origin + unit normal); for
an axis, `out6 = [ox,oy,oz, dx,dy,dz]` (origin + unit direction). These are the
ONLY additions to the public C ABI; no existing `cc_*` signature or POD struct
layout SHALL change.

#### Scenario: ABI addition is source-compatible
- GIVEN the host app previously linked the kernel
- WHEN it links the version with the six new entry points
- THEN every existing `cc_*` signature and POD layout SHALL be unchanged AND the
  ABI contract test (`tests/test_abi.cpp`) SHALL still pass against
  `KernelBridgeAPI.h`

#### Scenario: A successful constructor returns a unit direction/normal
- GIVEN any reference-geometry constructor that returns `1`
- WHEN its `out6` is inspected
- THEN the direction/normal component `(out6[3],out6[4],out6[5])` SHALL be
  unit-length within `1e-9`

### Requirement: Plane from three points (exact, host-portable)
`cc_ref_plane_from_points` SHALL compute the plane through three points as origin
`p0` and unit normal `normalize((p1-p0) x (p2-p0))`, in exact double precision,
available in every build (including the no-OCCT host). Colinear or coincident
input (cross-product magnitude below the degeneracy tolerance) SHALL return `0`.

#### Scenario: Normal of a plane through three known points equals the known normal
- GIVEN three non-colinear points that lie in the plane `z = 5` (e.g.
  `(0,0,5)`, `(1,0,5)`, `(0,1,5)`)
- WHEN `cc_ref_plane_from_points` is called (in the host or OCCT build)
- THEN it SHALL return `1`, `out6` origin SHALL equal `p0` within `1e-9`, AND the
  unit normal SHALL equal `(0,0,1)` (up to the right-hand-rule sign) within `1e-9`

#### Scenario: Colinear points fail
- GIVEN three colinear points (e.g. `(0,0,0)`, `(1,1,1)`, `(2,2,2)`)
- WHEN `cc_ref_plane_from_points` is called
- THEN it SHALL return `0` and SHALL NOT write a valid plane

### Requirement: Offset plane (exact, host-portable)
`cc_ref_plane_offset` SHALL return a plane with unit normal `normalize(normal)`
and origin `origin + dist * normalize(normal)`, in exact double precision,
available in every build. `dist` MAY be negative. A zero-length `normal` (below
the degeneracy tolerance) SHALL return `0`.

#### Scenario: Offset plane origin moves by dist along the normal
- GIVEN origin `O = (2,3,4)`, normal `N = (0,0,2)`, and `dist = 5`
- WHEN `cc_ref_plane_offset` is called
- THEN it SHALL return `1`, the unit normal SHALL equal `(0,0,1)` within `1e-9`,
  AND the returned origin SHALL equal `(2,3,9)` (`O + dist * normalize(N)`)
  within `1e-9`

#### Scenario: Zero-length normal fails
- GIVEN a normal `(0,0,0)`
- WHEN `cc_ref_plane_offset` is called
- THEN it SHALL return `0`

### Requirement: Axis from two points (exact, host-portable)
`cc_ref_axis_from_points` SHALL return an axis with origin `a` and unit direction
`normalize(b-a)`, in exact double precision, available in every build.
Coincident points (`|b-a|` below the degeneracy tolerance) SHALL return `0`.

#### Scenario: Axis through two points has the expected unit direction
- GIVEN `a = (1,1,1)` and `b = (1,1,4)`
- WHEN `cc_ref_axis_from_points` is called
- THEN it SHALL return `1`, the origin SHALL equal `a` within `1e-9`, AND the
  unit direction SHALL equal `(0,0,1)` within `1e-9`

#### Scenario: Coincident points fail
- GIVEN `a == b`
- WHEN `cc_ref_axis_from_points` is called
- THEN it SHALL return `0`

### Requirement: Derived datum from existing geometry (OCCT)
`cc_ref_plane_from_face` SHALL return the plane (origin + unit normal) of a
planar face; `cc_ref_axis_from_edge` SHALL return the origin + unit direction of a
linear edge; and `cc_ref_axis_from_face` SHALL return the axis of a
cylindrical/conical face using the SAME logic as `cc_face_axis`. A non-planar
face, a non-linear edge, a non-cylindrical/conical face, or an unknown
body/subshape id SHALL return `0`. In a build without a B-rep engine (the host
stub), all three SHALL return `0` (unsupported), without crashing.

#### Scenario: Plane from a planar face equals the known face normal
- GIVEN a box body on a booted iOS simulator and the face id of its top face
  (outward normal `(0,0,1)`)
- WHEN `cc_ref_plane_from_face` is called for that face id
- THEN it SHALL return `1`, the unit normal SHALL equal the known face normal
  `(0,0,1)` within `1e-9`, AND the origin SHALL lie on that face

#### Scenario: Axis from a linear edge equals the known edge direction
- GIVEN a box body on a booted iOS simulator and the id of a vertical edge
  (direction `(0,0,1)`)
- WHEN `cc_ref_axis_from_edge` is called for that edge id
- THEN it SHALL return `1` AND the unit direction SHALL equal `(0,0,1)` (up to
  sign) within `1e-9`

#### Scenario: Axis from a cylindrical face matches cc_face_axis
- GIVEN a cylindrical body on a booted iOS simulator and a cylindrical face id
- WHEN both `cc_ref_axis_from_face` and `cc_face_axis` are called for that face
- THEN both SHALL return `1` AND their `out6` values SHALL be equal within `1e-9`

#### Scenario: Non-planar face fails
- GIVEN a cylindrical (non-planar) face id
- WHEN `cc_ref_plane_from_face` is called
- THEN it SHALL return `0`

#### Scenario: Derived constructors are unsupported in the host stub
- GIVEN a build with no B-rep engine (the host stub)
- WHEN `cc_ref_plane_from_face`, `cc_ref_axis_from_edge`, or
  `cc_ref_axis_from_face` is called with any arguments
- THEN each SHALL return `0` without crashing
