# native-blends Specification

## Purpose
TBD - created by archiving change add-native-fillets-offsets. Update Purpose after archive.
## Requirements
### Requirement: Native planar chamfer (`cc_chamfer_edges`)

The native blend library SHALL compute `cc_chamfer_edges(body, edgeIds, edgeCount, distance)`
NATIVELY when `body` is a native solid and every selected edge is a STRAIGHT edge shared by
exactly two `FaceSurface`-kind-`Plane` faces with a CONVEX dihedral and no multi-edge corner
interference. For each such edge the builder SHALL cut both adjoining faces back by `distance`
(along their in-face perpendiculars to the edge) and insert one new planar chamfer face
spanning the two setback lines — realised via a planar cutter subtracted with the native
`src/native/boolean` BSP-CSG, or an equivalent direct topology edit. The result SHALL be a
native `topology::Shape` of type `Solid`, watertight (every edge shared by exactly two faces),
with an enclosed volume strictly LESS than the input's. On a box edge the native result SHALL
match the OCCT `BRepFilletAPI_MakeChamfer` oracle EXACTLY (fp64), with removed volume
`½ · distance² · edgeLength`. This builder SHALL remain OCCT-free and host-buildable and SHALL
reference no OCCT / `IEngine` / `EngineShape` type.

#### Scenario: Box edge chamfer is a watertight solid with exact removed-prism volume (host)
- GIVEN an axis-aligned box `B` (native planar solid, built on the host with no OCCT) and one convex edge of length `L`
- WHEN `cc_chamfer_edges(B, {edge}, 1, d)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) with every edge shared by exactly two faces, exactly one new planar chamfer face, AND its enclosed volume SHALL equal `|B| − ½·d²·L` exactly within fp64 tolerance

#### Scenario: Box chamfer matches the OCCT BRepFilletAPI_MakeChamfer oracle (parity)
- GIVEN an axis-aligned box edge chamfered by `distance` on a booted iOS simulator
- WHEN `cc_chamfer_edges` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT `BRepFilletAPI_MakeChamfer` oracle

### Requirement: Native constant-radius fillet (`cc_fillet_edges`)

The native blend library SHALL compute `cc_fillet_edges(body, edgeIds, edgeCount, radius)`
NATIVELY when `body` is a native solid and every selected edge is a STRAIGHT edge shared by
exactly two `FaceSurface`-kind-`Plane` faces (a planar dihedral) with a CONVEX dihedral and no
multi-edge corner interference. For each such edge the builder SHALL construct the rolling-ball
tangent cylinder (radius `radius`, axis parallel to the edge on the interior bisector at
distance `radius / sin(halfAngle)`, tangent to both faces along two contact lines) — REUSING
the Phase-3 full-round tangent-cylinder construction in `src/native/construct` — trim both
faces back to their contact lines, insert the cylindrical blend face and the planar setback
faces, and close the solid. The result SHALL be a native `topology::Shape` of type `Solid`,
watertight, whose blend face is a `FaceSurface` of kind `Cylinder` of radius `radius` tangent
to both original faces, with an enclosed volume strictly LESS than the input's. On a box convex
edge the native result SHALL match the OCCT `BRepFilletAPI_MakeFillet` oracle within a
deflection bound, with removed volume `(1 − π/4)·radius²·edgeLength`. This builder SHALL remain
OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` / `EngineShape` type.

#### Scenario: Box convex-edge fillet is a watertight cylinder-blended solid (host)
- GIVEN an axis-aligned box `B` (native planar solid, built on the host with no OCCT) and one convex edge of length `L`
- WHEN `cc_fillet_edges(B, {edge}, 1, r)` is computed and the result is tessellated
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose blend face is a `Cylinder` of radius `r` tangent to both adjoining faces AND its enclosed volume SHALL equal `|B| − (1 − π/4)·r²·L` within the tessellation deflection bound

#### Scenario: Box fillet matches the OCCT BRepFilletAPI_MakeFillet oracle within a deflection bound (parity)
- GIVEN an axis-aligned box convex edge filleted by `radius` on a booted iOS simulator
- WHEN `cc_fillet_edges` is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the tessellation deflection bound with the OCCT `BRepFilletAPI_MakeFillet` oracle, and the native blend face SHALL be a cylinder of radius `radius`

### Requirement: Native planar face offset (`cc_offset_face`)

The native blend library SHALL compute `cc_offset_face(body, faceId, distance)` NATIVELY when
`body` is a native solid and the selected face is a `FaceSurface` of kind `Plane` whose
adjacent faces are prismatic to it. The builder SHALL translate the selected face along its
outward unit normal by `distance` (positive grows the solid, negative shrinks it), re-plane the
moved face, re-loft the adjacent side faces to the moved boundary, and close the solid. The
result SHALL be a native `topology::Shape` of type `Solid`, watertight, whose enclosed volume
equals `|body| + area(face)·distance` EXACTLY (fp64). This builder SHALL remain OCCT-free and
host-buildable and SHALL reference no OCCT / `IEngine` / `EngineShape` type.

#### Scenario: Box face offset outward grows the volume by exactly area×distance (host)
- GIVEN an axis-aligned box `B` (native planar solid, built on the host with no OCCT) with a planar top face of area `A`
- WHEN `cc_offset_face(B, top, d)` is computed with `d > 0` and the result is tessellated
- THEN the result SHALL be a watertight `Solid` AND its enclosed volume SHALL equal `|B| + A·d` exactly within fp64 tolerance

#### Scenario: Box face offset inward shrinks the volume by exactly area×distance (host)
- GIVEN an axis-aligned box `B` with a planar top face of area `A`, built on the host with no OCCT
- WHEN `cc_offset_face(B, top, d)` is computed with `d < 0` (and `|d|` less than the box height)
- THEN the result SHALL be a watertight `Solid` AND its enclosed volume SHALL equal `|B| + A·d` (i.e. `|B| − A·|d|`) exactly within fp64 tolerance

### Requirement: Native uniform-thickness shell (`cc_shell`)

The native blend library SHALL compute `cc_shell(body, faceIds, faceCount, thickness)` NATIVELY
when `body` is a native PLANAR / box-like solid, the retained faces are planar, `thickness`
uniform and positive, and `thickness` is less than half the smallest span (no self-intersecting
inner offset). The builder SHALL offset every RETAINED face inward by `thickness` to form the
inner void (the selected opening faces left open), subtract the inner void from the outer solid
via the native `src/native/boolean` BSP-CSG (offset + boolean), and produce a uniform-thickness
wall open at the selected faces. The result SHALL be a native `topology::Shape` of type `Solid`,
watertight, with an enclosed volume `Vwall` satisfying `0 < Vwall < |body|`. On a box shelled to
wall `t` with the top face removed the native result SHALL match the OCCT
`BRepOffsetAPI_MakeThickSolid` oracle EXACTLY (fp64), with wall volume
`|box| − (a−2t)·(b−2t)·(c−t)` for a box `[0,a]×[0,b]×[0,c]`. This builder SHALL remain OCCT-free
and host-buildable and SHALL reference no OCCT / `IEngine` / `EngineShape` type.

#### Scenario: Box shell with the top removed is a watertight wall with exact volume (host)
- GIVEN an axis-aligned box `[0,a]×[0,b]×[0,c]` (native planar solid, built on the host with no OCCT)
- WHEN `cc_shell(B, {top}, 1, t)` is computed with `0 < t < min(a,b,c)/2` and the result is tessellated
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) open at the top face AND its enclosed volume SHALL equal `|B| − (a−2t)·(b−2t)·(c−t)` exactly within fp64 tolerance

#### Scenario: Box shell matches the OCCT BRepOffsetAPI oracle (parity)
- GIVEN an axis-aligned box shelled to a uniform wall `t` with one face removed on a booted iOS simulator
- WHEN `cc_shell` is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT `BRepOffsetAPI_MakeThickSolid` oracle

### Requirement: Mandatory blend self-verify guard (discard and fall through)

The engine SHALL accept a native blend / offset / shell result as native ONLY when it PASSES a
mandatory self-verify: the candidate SHALL be (a) a **closed watertight 2-manifold** (closed at
every deflection in the mesher's deflection ladder, positive enclosed volume) AND (b) have a
**sane volume for the op** — a convex-edge fillet and a chamfer SHALL strictly REDUCE the volume
(`Vr < |body|`, matching the removed-prism / removed-corner estimate within tolerance); an offset
SHALL satisfy `Vr ≈ |body| + area·distance` (grow for `distance > 0`, shrink for `distance < 0`);
a shell SHALL satisfy `0 < Vr < |body|` (matching `|body| − |inner void|`). If EITHER check fails
(not watertight, or the WRONG volume direction / magnitude for the op), the engine SHALL
**DISCARD** the native result and fall through to OCCT. The engine SHALL NEVER emit an
unverified, leaky, or wrong blend / offset / shell solid.

#### Scenario: A bad native blend result is discarded and the call falls through (host)
- GIVEN a native blend / offset / shell candidate that is open / non-manifold OR has the wrong volume direction or magnitude for its op (e.g. a fillet that grew the solid), built on the host
- WHEN the self-verify guard is applied
- THEN the guard SHALL reject the candidate AND `NativeEngine` SHALL fall through to the fallback engine for that call (no leaky or wrong solid is emitted)

#### Scenario: A verified native blend is read back by the native paths (host)
- GIVEN a native blend / offset / shell result that PASSES the self-verify (watertight 2-manifold with a sane volume for the op)
- WHEN its mass properties, bounding box, sub-shape ids, and tessellation are queried
- THEN they SHALL be served by the native body-consuming paths with no fallback call

### Requirement: Curved-face, concave, variable-radius, face-fillet, and multi-edge cases fall through to OCCT

The native blend builders SHALL DECLINE (return a NULL `Shape`) — and `NativeEngine` SHALL fall
through to OCCT — for any case outside the tractable-planar slice: (1) any face touching the
operation is CURVED (`FaceSurface::kind != Plane` — cylinder / sphere / cone / NURBS); (2) a
selected edge's dihedral is CONCAVE (the blend adds material into a reflex corner); (3) MULTI-EDGE
interference (two or more selected edges whose blends overlap at a shared corner / setback);
(4) a NON-PLANAR / non-convex shell solid, a non-uniform thickness, or a wall ≥ half the smallest
span; (5) `body` is NOT a native body (a foreign / OCCT-built shape id); or (6) degenerate input
(zero-length edge, zero-area face, non-positive radius / distance / thickness). In addition,
`cc_fillet_edges_variable` (variable radius) and `cc_fillet_face` (face fillet) SHALL always fall
through to OCCT in this change (not implemented natively). Each such case SHALL produce EXACTLY
the fallback (OCCT) engine's result. The change SHALL NOT fake, stub-out, or partially implement
any deferred case; each SHALL be labelled and verified as a fall-through, never faked.

#### Scenario: A curved-face input falls through (host + parity)
- GIVEN a fillet / chamfer / offset / shell where a face touching the operation is curved (e.g. a filleted-box or cylinder body), with the native engine active (`cc_set_engine(1)`)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return a NULL `Shape` AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A concave edge falls through (host)
- GIVEN a `cc_fillet_edges` or `cc_chamfer_edges` on a CONCAVE (reflex-dihedral) edge, with the native engine active
- WHEN the op is invoked
- THEN the native builder SHALL return a NULL `Shape` (rather than emit a wrong / self-intersecting solid) AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: Variable-radius and face-fillet always fall through (host + parity)
- GIVEN a `cc_fillet_edges_variable` (variable radius) call OR a `cc_fillet_face` call, with the native engine active
- WHEN the op is invoked
- THEN `NativeEngine` SHALL fall through to OCCT for that call (these are not implemented natively in this change), identical to `cc_set_engine(0)`

#### Scenario: A multi-edge-interference selection falls through (host)
- GIVEN a `cc_fillet_edges` / `cc_chamfer_edges` selection of two or more edges whose blends would overlap at a shared corner, with the native engine active
- WHEN the op is invoked
- THEN the native builder SHALL return a NULL `Shape` AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: A non-native (foreign) body falls through (host)
- GIVEN a fillet / chamfer / offset / shell where `body` is not a native body (a foreign / OCCT-built shape id), with the native engine active
- WHEN the op is invoked
- THEN `NativeEngine` SHALL fall through to the fallback engine for that call, identical to `cc_set_engine(0)`

### Requirement: NativeEngine computes the blend natively, else falls through

`NativeEngine` SHALL override `chamfer_edges`, `fillet_edges`, `offset_face`, and `shell` to run
the corresponding native builder in `src/native/blend/` when `body` is a native body and the case
is in the tractable-planar slice, and apply the mandatory self-verify guard, type-erasing a
verified native `topology::Shape` into a tracked native `EngineShape`. When `body` is not native,
when the native builder returns a NULL `Shape` (a curved / concave / multi-edge / out-of-range
DECLINE), or when the self-verify guard rejects the candidate, the override SHALL fall through to
the held fallback engine with **no native interception**, producing exactly the fallback's result.
`fillet_edges_variable` and `fillet_face` SHALL remain pure fall-throughs (no native builder
call). OCCT SHALL be referenced ONLY under `CYBERCAD_HAS_OCCT` (in the fallback wiring); the
native builder SHALL reference no OCCT / `IEngine` / `EngineShape` type. No `cc_*` signature or
POD layout SHALL change and the default engine SHALL remain OCCT (opt-in via `cc_set_engine(1)`).

#### Scenario: A supported planar blend is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a native planar-faced solid with a supported (convex planar-dihedral edge / planar face / box shell) selection
- WHEN `cc_chamfer_edges` / `cc_fillet_edges` / `cc_offset_face` / `cc_shell` is invoked for the supported case
- THEN the shape SHALL be built by `src/native/blend/` and PASS the self-verify with no fallback call AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: An unsupported or unverified blend falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred case (curved face, concave edge, variable radius, face fillet, multi-edge interference, foreign body) OR a candidate that fails the self-verify
- WHEN the `cc_*` op is invoked
- THEN the native builder SHALL return NULL (or the guard SHALL reject, or the method is a pure fall-through) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

### Requirement: Blend parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME `cc_chamfer_edges`
/ `cc_fillet_edges` / `cc_offset_face` / `cc_shell` calls SHALL be issued once with the native
engine active (`cc_set_engine(1)`) and once with the OCCT default (`cc_set_engine(0)`), and the
two resulting shapes SHALL be compared through the `cc_*` facade — mass properties, bounding box,
sub-shape counts, and watertight tessellation — against the OCCT `BRepFilletAPI` / `BRepOffsetAPI`
oracle. On **box** chamfer / offset / shell the native result SHALL match the oracle EXACTLY
(volume / bbox / centroid relative error ~0, fp precision); the constant-radius fillet SHALL match
within the tessellation deflection bound. The fall-through cases (a curved-face input, a concave
edge, a variable-radius call, a `cc_fillet_face` call, and a multi-edge-interference selection)
SHALL be asserted identical under both engines (fall-through proof, `cc_active_engine() == 1`). The
parity test SHALL restore the OCCT default in teardown and SHALL carry its own `main()` (on the
`run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

#### Scenario: Native box blends match the OCCT BRepFilletAPI / BRepOffsetAPI oracle (parity)
- GIVEN box chamfer / offset / shell cases and a box convex-edge fillet on a booted iOS simulator
- WHEN each `cc_*` op is called with the native engine active and with the OCCT default
- THEN the chamfer / offset / shell shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT oracle, and the fillet SHALL agree within the tessellation deflection bound

#### Scenario: Fall-through blend cases are identical under both engines (parity)
- GIVEN a curved-face blend, a concave-edge blend, a variable-radius call, a `cc_fillet_face` call, and a multi-edge-interference selection on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

