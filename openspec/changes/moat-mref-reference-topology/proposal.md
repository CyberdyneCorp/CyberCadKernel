# Proposal — moat-mref-reference-topology (MOAT M-REF)

## Why

The app's datum / reference-geometry features (datum planes and axes, the
offset-edge tool, tangent-chain and outer-rim edge selection, cylinder-axis
picking) read reference geometry off a body through OCCT today
(`BRepAdaptor_Surface`/`gp_Pln`, `BRepAdaptor_Curve`/`gp_Lin`/`gp_Circ`,
`BRepTools::OuterWire`, `TopExp` ancestry, `BRepOffsetAPI_MakeOffset`). These are
READ-ONLY topology/geometry queries — no new geometry is built — so they are a
bounded, high-value slice to move onto the native B-rep and off the LGPL engine.

Every input the queries need is already landed and OCCT-verified in this
worktree: the native B-rep graph (`src/native/topology/{shape,accessors,explore}`
— `surfaceOf`/`curveOf`/`pointOf`, face outer/hole wires, `mapShapes`,
`mapShapesAndAncestors` ancestry, shared-vertex/edge identity) and the elementary
frame math (`src/native/math` — `Ax3`/`Dir3`, closed-form circle/ellipse
tangents). M-REF assembles these into datum results; it does NOT touch geometry
ops (`boolean/`, `ssi/`, `blend/`) — those are sibling tracks.

Ambiguous or unsupported inputs are an **HONEST DECLINE**, never a wrong datum: a
non-planar face where a plane is required, a non-linear edge where an axis line is
required, a face with no cylinder/cone axis, a freeform edge in a tangent walk,
and an offset that OCCT would arc-round or that self-intersects. A decline returns
a clean error and the facade falls through to OCCT; no tolerance is weakened.

## What Changes

1. **A new header-only, OCCT-free module `src/native/reference/reference.h`** in
   namespace `cybercad::native::reference`, consuming `src/native/{math,topology}`
   read-only:
   - `faceAxis(face)` / `refAxisFromFace(face)` — the axis of a cylindrical or
     conical face (origin + unit direction). Cylinder/cone ONLY (matches
     `cc_face_axis`); a plane/sphere/torus/freeform face declines.
   - `refPlaneFromFace(face)` — a datum plane from a planar face: outward normal
     (the plane Z, flipped for a Reversed face) + an on-plane origin (the outer-
     wire vertex centroid). Non-planar face declines.
   - `refAxisFromEdge(edge)` — the axis line of a straight edge. LINE ONLY (a
     circular edge yields no `gp_Lin` in the OCCT oracle → decline).
   - `tangentChain(root, seeds)` — the connected set of tangent-continuous edge
     ids grown from a seed set (`|cos(tangent angle)| ≥ cos 15° = 0.966` at a
     shared vertex; Line/Circle/Ellipse tangents closed-form). A freeform edge in
     the walk declines (deferred to the oracle).
   - `outerRimChain(root, seeds)` — the OUTER-wire edge ids of the planar cap
     face(s) the seeds bound (a face qualifies only if its plane contains ALL
     seed vertices within 1.0 model unit — picks the cap, rejects side walls).
   - `offsetFaceBoundary(face, distance)` — the in-plane offset of a planar
     POLYGON boundary with sharp (miter) joins, as a closed xyz polyline. Scoped
     to the case that provably coincides with OCCT `BRepOffsetAPI_MakeOffset`
     (an inward / corner-sharpening offset of the polygon); a non-planar face, a
     non-line (arc) boundary edge, a growing convex offset (OCCT arc-rounds the
     corners), and a self-intersecting/collapsing offset all decline.

2. **Native dispatch in `src/engine/native/native_engine.cpp`** for the seven
   `cc_*` reference ops (`face_axis`, `ref_plane_from_face`, `ref_axis_from_edge`,
   `ref_axis_from_face`, `tangent_chain`, `outer_rim_chain`, `offset_face_boundary`):
   resolve the sub-shape id on the native B-rep and call `reference.h`. A native
   body is served natively; a decline returns a clean `Error` so the facade falls
   through to OCCT (a native void is NEVER handed to OCCT); a mesh body errors
   cleanly. **No `cc_*` signature changes** (ADDITIVE dispatch only); the OCCT
   engine path is untouched and remains the oracle.

3. **Two verification gates** (the non-negotiable discipline):
   - **GATE A — HOST ANALYTIC (no OCCT):** `tests/native/test_native_reference.cpp`
     asserts closed-form fixtures to machine precision — a box's face normals /
     datum-plane origins, a cylinder's face axis, a straight edge's axis, tangent-
     chain growth across a C1 (collinear / line-tangent-arc) joint and NO growth
     across a 90° corner, an outer-rim loop, and a rectangle offset inward — plus
     the declines (non-planar plane, circular-edge axis, growing convex offset).
   - **GATE B — SIM native-vs-OCCT:** `tests/sim/native_reference_parity.mm` (run
     by `scripts/run-sim-native-reference.sh`) asserts, on a booted iOS simulator,
     `refPlaneFromFace` vs `gp_Pln`, `faceAxis`/`refAxisFromFace` vs
     `gp_Cylinder::Axis`, `refAxisFromEdge` vs `gp_Lin`, `outerRimChain` vs
     `BRepTools::OuterWire`, `offsetFaceBoundary` vs `BRepOffsetAPI_MakeOffset`
     (inward), and `tangentChain`'s grow/stop decision vs the `BRepAdaptor_Curve`
     D1 tangent oracle — all at fixed, never-widened tolerances.

Out of scope (honest declines, tracked): a circular-edge axis (no `gp_Lin`
oracle), a freeform-edge tangent walk, a non-planar / arc-boundary / growing-
convex / self-intersecting offset. These decline at the service boundary and stay
on OCCT.
