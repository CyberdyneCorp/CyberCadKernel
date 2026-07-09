# native-blend

## ADDED Requirements

### Requirement: Native `cc_fillet_face` path is wired and honestly declines the corner weld

The engine SHALL provide a NATIVE, OCCT-free path for `cc_fillet_face(body, faceId,
radius)` — round every edge bounding the picked face `faceId` at constant `radius`,
matching the OCCT adapter which Adds a fillet on every edge of the face. On a native
all-planar body with a planar picked face the engine SHALL collect the CONVEX
planar-dihedral bounding edges (probed with the same tangent-cylinder guard the
dihedral fillet uses) and re-solve them with the landed multi-edge
`blend::fillet_edges` blend. A produced candidate SHALL be accepted ONLY under the
engine's SHRINK self-verify (watertight closed 2-manifold, enclosed volume strictly
less than the sharp input). This native path SHALL remain OCCT-free and SHALL NOT
change the `cc_*` ABI.

Because rounding a FULL face perimeter on a convex planar solid requires a SPHERICAL
corner patch between the cylinder blends of every two adjacent bounding edges — the
corner weld that gates on M2, which the landed `blend::fillet_edges` does not yet build
(it welds only NON-adjacent edge sets) — the full-face fillet on a planar solid SHALL
be honestly DECLINED this wave: the native op SHALL return a NULL result with a
MEASURED reason (`WeldGatesM2`) and the engine SHALL fall through to OCCT
`BRepFilletAPI_MakeFillet`. The engine SHALL NEVER hand a native void to OCCT, SHALL
NEVER silently drop edges to fake a partial fillet, and SHALL NEVER emit an unverified
solid. The path SHALL land automatically (no engine change) once M2 supplies the
corner weld and `blend::fillet_edges` welds corner-sharing edge sets.

#### Scenario: A full-face fillet on a box declines with a measured corner-weld reason (host)

- GIVEN a native box and a picked planar face whose bounding edges are all convex but form a corner-sharing loop, with the native engine active and no OCCT
- WHEN `cc_fillet_face(B, id(F), r)` is invoked
- THEN the convex bounding edges SHALL be identified AND the multi-edge weld SHALL fail at the shared corners AND the native op SHALL return a NULL result with the measured reason `WeldGatesM2` AND SHALL NOT emit a solid (the engine falls through to OCCT)

#### Scenario: A curved solid / non-planar face / oversized radius is honestly declined (host)

- GIVEN a native solid with a curved face, or a non-planar picked face, or a zero/oversized radius, with the native engine active and no OCCT
- WHEN `cc_fillet_face` is invoked
- THEN the native op SHALL return a NULL result with a measured decline reason (`NonPlanarSolid` / `NonPlanarFace` / `BadInput` / `NoConvexEdges` / `WeldGatesM2`) AND SHALL NOT emit a solid (the engine falls through to OCCT)

#### Scenario: A native fillet_face honestly declines and OCCT owns the reference (sim)

- GIVEN a native box face, on a booted simulator, and the OCCT engine's `BRepFilletAPI_MakeFillet` reference rounding every edge of that face
- WHEN `cc_fillet_face` is invoked under the native engine on the native body
- THEN it SHALL return id 0 with an honest error (the corner weld gates on M2, a native void is never handed to OCCT) AND the OCCT engine SHALL independently produce the valid full-face fillet reference

### Requirement: Native full-round fillet caps a prismatic rib with a tangent cylinder

The engine SHALL compute `cc_full_round_fillet(body, faceId)` and
`cc_full_round_fillet_faces(body, leftFaceId, middleFaceId, rightFaceId)` — replace
the narrow middle face with a full round tangent to its two neighbour walls, consuming
the middle face — NATIVELY, without OCCT, when `body` is a native all-planar solid, the
middle face is a planar strip, and its two side walls are PARALLEL planar faces a
distance `w` apart with STRAIGHT equal-length seam edges (the analytic PRISMATIC case).
The engine SHALL build the rolling-ball cap of radius `r = w/2` — the r = w/2 special
case of the tangent-cylinder blend — by re-solving the two seam edges (`middle↔left`,
`middle↔right`) with the landed `blend::fillet_edges` at radius `w/2`, so the two arcs
meet tangentially on the strip mid-plane and the middle face is consumed. The
single-face entry SHALL auto-detect the two longest opposite edges of the middle face
as the seams and their across-neighbours as the walls; the three-face entry SHALL use
the shared seam edges of the explicit left/middle/right faces.

The returned solid SHALL be a watertight `Solid` with the cap cylinder G1-tangent to
both walls at the two seams, the middle face provably gone (fewer distinct planes than
the input), and enclosed volume strictly less than the input (a convex full round
REMOVES material). For a plain box of top width `w` and length `L` the removed volume
SHALL equal the closed form `(w²/2)(1 − π/4)·L` within the deflection tolerance. This
native path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

A DIHEDRAL (non-parallel) middle, a curved wall, a non-planar middle face, a
CLOSED-SEAM / annulus configuration (a full round on a circular boss top), or a
non-all-planar solid SHALL be honestly DECLINED: the native op SHALL return a NULL
result with a measured reason (the dihedral valley-solve and closed-seam weld gate on
M2 freeform booleans) and the engine SHALL fall through to OCCT. The engine SHALL NEVER
hand a native void to OCCT and SHALL NEVER emit an unverified solid.

#### Scenario: A prismatic rib top rounds to the closed-form half-cylinder volume (host)

- GIVEN a native box of top width `w` and length `L` with the native engine active and no OCCT
- WHEN `cc_full_round_fillet` rounds the top face between the two parallel side walls
- THEN the returned solid SHALL be watertight AND the removed volume SHALL equal `(w²/2)(1 − π/4)·L` within the deflection tolerance AND the cap cylinder (radius `w/2`) SHALL be tangent to both walls AND the middle face SHALL be gone

#### Scenario: The explicit three-face entry matches the auto entry (host)

- GIVEN a native prismatic rib with a planar top and two parallel walls, with the native engine active and no OCCT
- WHEN `cc_full_round_fillet_faces(B, left, top, right)` and `cc_full_round_fillet(B, top)` are invoked
- THEN both SHALL return the SAME watertight cap solid (same volume, area, topology) within tolerance

#### Scenario: A native full round matches the OCCT full-round oracle (sim)

- GIVEN a native prismatic rib and the OCCT full-round reference for the same faces, on a booted simulator
- WHEN both compute the full round
- THEN the native result SHALL match the OCCT reference on volume, area, watertightness, Euler χ = 2, per-axis bbox and Hausdorff distance within deflection-bounded tolerances

#### Scenario: A dihedral / curved / closed-seam full round is honestly declined (host)

- GIVEN a native solid whose middle-face walls are non-parallel, or curved, or form a closed-seam annulus, with the native engine active and no OCCT
- WHEN `cc_full_round_fillet[_faces]` is invoked
- THEN the native op SHALL return a NULL result with a measured decline reason (dihedral valley-solve / closed-seam weld gates on M2) AND SHALL NOT emit a solid (the engine falls through to OCCT)
