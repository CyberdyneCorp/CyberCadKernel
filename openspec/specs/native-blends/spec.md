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
NATIVELY in the following three cases (else return a NULL Shape so the engine falls
through to the OCCT `BRepFilletAPI_MakeFillet` oracle):

1. **Straight planar dihedral (existing).** A STRAIGHT edge shared by exactly two
   `FaceSurface`-kind-`Plane` faces with a CONVEX dihedral and no multi-edge corner
   interference. For each such edge the builder SHALL construct the rolling-ball tangent
   CYLINDER (radius `radius`, axis parallel to the edge on the interior bisector at distance
   `radius / sin(halfAngle)`, tangent to both faces along two contact lines) — REUSING the
   full-round tangent-cylinder construction in `src/native/construct` — trim both faces back
   to their contact lines, insert the cylindrical blend face and the planar setback faces, and
   close the solid. On a box convex edge the result SHALL match the OCCT oracle within a
   deflection bound, with removed volume `(1 − π/4)·radius²·edgeLength`.

2. **CONVEX circular cylinder↔cap crease (existing curved slice).** A CIRCULAR edge (a
   `Circle` `EdgeCurve` of radius `Rc` coaxial with a cylinder axis `A`) shared by exactly two
   faces — one `FaceSurface`-kind-`Cylinder` lateral face (radius `Rc`, axis `A`) and one
   `FaceSurface`-kind-`Plane` CAP whose normal is parallel to `A` — with a CONVEX crease and
   `radius` small enough that both tangent circles stay inside their faces (in particular
   `Rc ≥ 2·radius`, a ring torus). For such a rim the builder SHALL construct the rolling-ball
   canal surface as a coaxial **TORUS** (major radius `Rc − radius`, minor radius `radius`,
   centred on the axis at the cap height offset axially by `−radius` INTO the material),
   compute its two tangent trim seams as the CIRCLES torus∩cylinder (coaxial, radius `Rc`) and
   torus∩plane (axis-perpendicular, radius `Rc − radius`) using the native SSI Stage-S1
   closed-form handlers, SELF-VERIFY each seam lies on the torus AND on its neighbouring
   original surface, trim the cylinder lateral face and the cap face back to their seam
   circles, tile the torus blend patch between the two seams into deflection-bounded facets,
   and weld the patch and trimmed faces watertight through the native `src/native/boolean`
   `assembleSolid`. The inserted torus blend face SHALL be G1-tangent to BOTH the cylinder and
   the cap along its seam circles. This convex fillet REMOVES material: its enclosed volume is
   strictly LESS than the input's, and it is accepted only by the engine self-verify's SHRINK
   branch (`0 < Vr < Vo`). On a native cylinder solid's top rim the result SHALL match the
   OCCT `BRepFilletAPI_MakeFillet` oracle within a deflection / curved-parity bound.

3. **CONCAVE circular cylinder↔plane crease (this change).** A CIRCULAR edge (a `Circle`
   `EdgeCurve` of radius `Rc` coaxial with a cylinder axis `A`) shared by exactly two faces —
   one `FaceSurface`-kind-`Cylinder` lateral face (radius `Rc`, axis `A`) and one
   `FaceSurface`-kind-`Plane` face whose normal is parallel to `A` and whose extent reaches
   BEYOND `Rc` (a LARGER plane the cylinder stands on, or the flat bottom of a blind hole) —
   meeting in a CONCAVE dihedral (material fills the corner; a point just outside the wall at
   the plane height is INSIDE the solid). For such a rim the builder SHALL construct the
   rolling-ball canal surface as a coaxial **TORUS** on the MATERIAL side — major radius
   `Rc + radius`, minor radius `radius`, centred on the axis at the plane height offset axially
   by `+radius` INTO the material (the offset sign is FLIPPED vs the convex case) — compute its
   two tangent trim seams as the CIRCLES torus∩cylinder (coaxial, radius `Rc`, the `v=0`
   inner-equator ring) and torus∩plane (axis-perpendicular, radius `Rc + radius`, the `v=π/2`
   ring) using the native SSI Stage-S1 closed-form handlers, SELF-VERIFY each seam lies on the
   torus AND on its neighbouring original surface, trim the cylinder lateral face back to its
   seam circle, rebuild the LARGER plane as an ANNULUS whose inner radius is the seam radius
   `Rc + radius`, tile the concave torus blend patch between the two seams into
   deflection-bounded facets, and weld the patch and trimmed faces watertight through the
   native `src/native/boolean` `assembleSolid`. The inserted torus blend face SHALL be
   G1-tangent to BOTH the cylinder and the plane along its seam circles. This concave fillet
   ADDS material: its enclosed volume is strictly GREATER than the input's, and it is accepted
   only by the engine self-verify's GROW branch (`Vr > Vo`). On a native boss-on-plate base rim
   (and a blind-hole bottom rim) the result SHALL match the OCCT `BRepFilletAPI_MakeFillet`
   oracle within a deflection / curved-parity bound.

In cases 1 and 2 the result's enclosed volume SHALL be strictly LESS than the input's; in
case 3 it SHALL be strictly GREATER. In every case the result SHALL be a native
`topology::Shape` of type `Solid`, watertight (every edge shared by exactly two faces), and it
SHALL be accepted only after the engine's mandatory watertight + correctly-SIGNED-volume
self-verify (SHRINK for cases 1–2, GROW for case 3; see the self-verify requirement) — else
DISCARDED → OCCT. This builder SHALL remain OCCT-free and host-buildable and SHALL reference
no OCCT / `IEngine` / `EngineShape` type. No `cc_*` signature or POD struct SHALL change.

#### Scenario: Box convex-edge fillet is a watertight cylinder-blended solid (host)
- GIVEN an axis-aligned box `B` (native planar solid, built on the host with no OCCT) and one convex STRAIGHT edge of length `L`
- WHEN `cc_fillet_edges(B, {edge}, 1, r)` is computed and the result is tessellated
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose blend face is a `Cylinder` of radius `r` tangent to both adjoining faces AND its enclosed volume SHALL equal `|B| − (1 − π/4)·r²·L` within the tessellation deflection bound

#### Scenario: Box fillet matches the OCCT BRepFilletAPI_MakeFillet oracle within a deflection bound (parity)
- GIVEN an axis-aligned box convex STRAIGHT edge filleted by `radius` on a booted iOS simulator
- WHEN `cc_fillet_edges` is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the tessellation deflection bound with the OCCT `BRepFilletAPI_MakeFillet` oracle, and the native blend face SHALL be a cylinder of radius `radius`

#### Scenario: Convex cylinder top-rim fillet inserts a material-removing torus patch and is watertight (host)
- GIVEN a native cylinder solid of radius `Rc` and height `H` (built on the host with no OCCT) and its top rim (the CIRCULAR edge where the lateral `Cylinder` face meets the top `Plane` CAP), with `0 < r` and `Rc ≥ 2r`
- WHEN `cc_fillet_edges(cyl, {rim}, 1, r)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose inserted blend face is a TORUS patch of minor radius `r` and major radius `Rc − r` coaxial with the cylinder axis, tangent (G1) to the cylinder along the circle of radius `Rc` and to the cap along the circle of radius `Rc − r`, AND its enclosed volume SHALL be strictly LESS than the cylinder's

#### Scenario: Concave boss-on-plate base-rim fillet inserts a material-adding torus patch and is watertight (host)
- GIVEN a native boss-on-plate solid (a `Cylinder` boss of radius `Rc` standing on a LARGER planar slab, built on the host with no OCCT) and its base rim (the CIRCULAR edge where the lateral `Cylinder` face meets the slab's top `Plane` in a CONCAVE dihedral), with `0 < r` and the slab reaching beyond `Rc + r`
- WHEN `cc_fillet_edges(body, {baseRim}, 1, r)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose inserted blend face is a coaxial TORUS patch of minor radius `r` and major radius `Rc + r`, tangent (G1) to the cylinder along the circle of radius `Rc` and to the plane along the circle of radius `Rc + r`, AND its enclosed volume SHALL be strictly GREATER than the input's, equal to `|body| + V_fill` (the closed-form ADDED concave rim-band) within the tessellation deflection bound

#### Scenario: Concave fillet matches the OCCT BRepFilletAPI_MakeFillet oracle (parity)
- GIVEN a native boss-on-plate base rim (and a blind-hole bottom rim) filleted by `radius` on a booted iOS simulator
- WHEN `cc_fillet_edges` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of a toroidal blend face SHALL agree within the curved-parity tolerance with the OCCT `BRepFilletAPI_MakeFillet` oracle, the native torus patch SHALL be G1-tangent to the cylinder and to the plane at its two seam circles, AND the native result's volume SHALL EXCEED the input's (material added)

#### Scenario: Out-of-slice curved rim defers to OCCT (never faked)
- GIVEN a fillet request that is NOT a supported slice — a variable radius, a cylinder↔cylinder canal rim, a non-circular curved crease (cone↔plane / sphere / ellipse / spline), a tilted or non-coaxial plane, a freeform adjacent face, an edge shared by ≠2 faces, or a near-degenerate radius (a convex rim with `Rc < 2·radius`, or a seam leaving its face)
- WHEN `cc_fillet_edges` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape (or the engine self-verify SHALL discard the candidate) AND the engine SHALL fall through to the OCCT `BRepFilletAPI_MakeFillet` oracle — it SHALL NOT emit an approximate, hand-tuned, or fabricated curved blend

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
through to OCCT — for any case outside the tractable slice: (1) any face touching the
operation is CURVED in a way no native curved slice handles (`FaceSurface::kind != Plane` and
not one of the supported cylinder↔cap fillet/chamfer rims — sphere / cone / NURBS); (2) a
selected edge's dihedral is CONCAVE and no native concave slice handles it; (3) MULTI-EDGE
interference (two or more selected edges whose blends overlap at a shared corner / setback);
(4) a NON-PLANAR / non-convex shell solid, a non-uniform thickness, or a wall ≥ half the smallest
span; (5) `body` is NOT a native body (a foreign / OCCT-built shape id); or (6) degenerate input
(zero-length edge, zero-area face, non-positive radius / distance / thickness). For
`cc_fillet_edges_variable` (variable radius), the native path handles ONLY the LINEAR-law
CONVEX circular cylinder↔cap rim (`r(θ) = radius1 + (radius2 − radius1)·θ/2π`, both seams
inside their faces, gradient within the curved-parity tolerance); every OTHER variable case —
a NON-linear radius law, a CONCAVE variable rim, a cylinder↔cylinder canal, a non-circular /
tilted / non-coaxial / freeform crease, or an out-of-parity gradient — SHALL fall through to
OCCT. For `cc_chamfer_edges` (chamfer), the native path handles the CONVEX PLANAR-dihedral
edge (the setback-plane corner slice) AND the SYMMETRIC-distance CONVEX circular cylinder↔cap
rim (the cone-frustum bevel, `Rc − d > 0`); every OTHER chamfer case — an ASYMMETRIC
two-distance chamfer, a CONCAVE circular rim, a cylinder↔cylinder (curved↔curved) rim, a
non-circular / tilted / non-coaxial / freeform crease, or `Rc ≤ d` — SHALL fall through to
OCCT. `cc_fillet_face` (face fillet) SHALL always fall through to OCCT (not implemented
natively). Each fall-through case SHALL produce EXACTLY the fallback (OCCT) engine's result.
The change SHALL NOT fake, stub-out, or partially implement any deferred case; each SHALL be
labelled and verified as a fall-through, never faked.

#### Scenario: A curved-face input falls through (host + parity)
- GIVEN a fillet / chamfer / offset / shell where a face touching the operation is curved and no native curved slice handles it (e.g. a cone body, or a sphere↔plane rim), with the native engine active (`cc_set_engine(1)`)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return a NULL `Shape` AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A concave edge with no native slice falls through (host)
- GIVEN a `cc_fillet_edges` or `cc_chamfer_edges` on a CONCAVE (reflex-dihedral) edge that no native concave slice handles (including a concave circular chamfer rim), with the native engine active
- WHEN the op is invoked
- THEN the native builder SHALL return a NULL `Shape` (rather than emit a wrong / self-intersecting solid) AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: An out-of-slice variable-radius, asymmetric-chamfer, or face-fillet call falls through (host + parity)
- GIVEN a `cc_fillet_edges_variable` call OUTSIDE the linear-law convex circular slice (a non-linear law, a concave variable rim, a cyl↔cyl canal, a non-circular / tilted / freeform crease, or an out-of-parity gradient), OR a `cc_chamfer_edges` call OUTSIDE the supported slices (an asymmetric two-distance chamfer, a cyl↔cyl chamfer, a non-circular curved crease, or `Rc ≤ d`), OR a `cc_fillet_face` call, with the native engine active
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

`NativeEngine` SHALL override `chamfer_edges`, `fillet_edges`, `fillet_edges_variable`,
`offset_face`, and `shell` to run the corresponding native builder in `src/native/blend/`
when `body` is a native body and the case is in the tractable slice, and apply the mandatory
self-verify guard, type-erasing a verified native `topology::Shape` into a tracked native
`EngineShape`. For `chamfer_edges` the override SHALL first call the PLANAR
`nblend::chamfer_edges(body, edgeIds, edgeCount, distance)` and, if that returns a NULL
`Shape` or fails the self-verify, SHALL call the CURVED
`nblend::curved_chamfer_edge(body, edgeIds, edgeCount, distance)`; each candidate SHALL be
accepted ONLY through `blendResultVerified(result, body, wantGrow=false)` (watertight +
`0 < Vr < Vo`), since a chamfer REMOVES material — the SAME SHRINK branch for both the planar
and curved candidate. When `body` is not native, when both native builders return a NULL
`Shape` (an out-of-slice DECLINE), or when the self-verify guard rejects the candidate, the
override SHALL fall through to the held fallback engine with **no native interception** for a
foreign body, and for a native body it cannot forward (OCCT would misread the native void)
SHALL return an honest error so the call is served by the OCCT engine — exactly how
`fillet_edges` treats an unbuildable native rim. `fillet_face` SHALL remain a pure
fall-through (no native builder call). OCCT SHALL be referenced ONLY under
`CYBERCAD_HAS_OCCT` (in the fallback wiring); the native builders SHALL reference no OCCT /
`IEngine` / `EngineShape` type. No `cc_*` signature or POD layout SHALL change and the default
engine SHALL remain OCCT (opt-in via `cc_set_engine(1)`).

#### Scenario: A supported planar or curved blend is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a native solid with a supported selection (a convex planar-dihedral chamfer / fillet edge / planar face / box shell / convex or concave circular cylinder↔plane fillet rim / a linear-law convex circular variable fillet rim / a SYMMETRIC convex circular cylinder↔cap chamfer rim)
- WHEN `cc_chamfer_edges` / `cc_fillet_edges` / `cc_fillet_edges_variable` / `cc_offset_face` / `cc_shell` is invoked for the supported case
- THEN the shape SHALL be built by `src/native/blend/` and PASS its correctly-signed self-verify with no fallback call AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: An unsupported or unverified blend falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred case (curved face with no native slice, concave edge with no native slice, out-of-slice variable radius, asymmetric or non-circular chamfer, face fillet, multi-edge interference, foreign body) OR a candidate that fails the self-verify
- WHEN the `cc_*` op is invoked
- THEN the native builder(s) SHALL return NULL (or the guard SHALL reject, or the method is a pure fall-through) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`) for a foreign body, or an honest error served by the OCCT engine for a native body — proving fall-through with no native interception

### Requirement: Blend parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME `cc_chamfer_edges`
/ `cc_fillet_edges` / `cc_fillet_edges_variable` / `cc_offset_face` / `cc_shell` calls SHALL be
issued once with the native engine active (`cc_set_engine(1)`) and once with the OCCT default
(`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through the `cc_*` facade —
mass properties, bounding box, sub-shape counts, and watertight tessellation — against the OCCT
`BRepFilletAPI` / `BRepOffsetAPI` oracle. On **box** chamfer / offset / shell the native result
SHALL match the oracle EXACTLY (volume / bbox / centroid relative error ~0, fp precision); the
constant-radius fillet, the linear-law variable fillet, AND the SYMMETRIC curved (circular-rim)
chamfer SHALL match within the tessellation deflection bound (the curved chamfer compared against
OCCT `BRepFilletAPI_MakeChamfer` with `Add(distance, edge)`; because a symmetric chamfer IS
EXACTLY a cone frustum, that bound is TIGHT — the angular deflection — not a loosened curved-parity
band). The fall-through cases (a curved-face input, a concave edge with no native slice, an
OUT-OF-SLICE variable-radius or asymmetric / non-circular chamfer call, a `cc_fillet_face` call,
and a multi-edge-interference selection) SHALL be asserted identical under both engines
(fall-through proof, `cc_active_engine() == 1`). The parity test SHALL restore the OCCT default in
teardown and SHALL carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite count
is unchanged.

#### Scenario: Native box blends match the OCCT BRepFilletAPI / BRepOffsetAPI oracle (parity)
- GIVEN box chamfer / offset / shell cases and a box convex-edge fillet on a booted iOS simulator
- WHEN each `cc_*` op is called with the native engine active and with the OCCT default
- THEN the chamfer / offset / shell shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT oracle, and the fillet SHALL agree within the tessellation deflection bound

#### Scenario: Native curved (circular-rim) chamfer matches the OCCT MakeChamfer oracle (parity)
- GIVEN a native cylinder top rim chamfered by a symmetric distance `d` on a booted iOS simulator, with `Rc − d > 0`
- WHEN `cc_chamfer_edges` is called with the native engine active and with the OCCT default (the OCCT side building `BRepFilletAPI_MakeChamfer` with `Add(d, edge)`)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of a flat cone-frustum bevel face SHALL agree within the tessellation deflection bound, the native bevel SHALL be C0 (at the chamfer angle) to the cylinder and the cap at its two setback-circle seams and NOT G1, and the native result's volume SHALL be LESS than the input's (material removed)

#### Scenario: Fall-through blend cases are identical under both engines (parity)
- GIVEN a curved-face blend, a concave-edge blend with no native slice, an out-of-slice variable-radius or asymmetric / non-circular chamfer call, a `cc_fillet_face` call, and a multi-edge-interference selection on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

### Requirement: Circular-crease fillet seams and torus patch are self-verified before assembly

Before assembling the circular cylinder↔cap fillet, the native builder SHALL compute the two
trim seams as closed-form CIRCLES using the native SSI Stage-S1 handlers — torus∩cylinder
(coaxial) at radius `R` and torus∩plane (axis-perpendicular) at radius `R − radius` — and
SHALL SELF-VERIFY each seam by sampling it and confirming every sample lies BOTH on the
rolling-ball torus AND on its neighbouring original surface (the cylinder / the cap plane)
within a tolerance derived from the operands' scale. If EITHER seam fails the on-both-surfaces
check, OR the required ring-torus / seam-inside-face preconditions do not hold, the builder
SHALL return a NULL Shape and the operation SHALL defer to OCCT. The builder SHALL NEVER emit
an unverified seam, weaken a tolerance to pass, or fabricate a curved patch. This is the #6
instance of the roadmap's mandatory self-verify → OCCT-fallback discipline.

#### Scenario: Both seam circles lie on the torus and their neighbour surfaces (host)
- GIVEN a native cylinder solid and its top rim with `R ≥ 2r`, built on the host with no OCCT
- WHEN the circular-crease fillet builder computes the two seam circles via native SSI S1
- THEN every sampled point of the cylinder seam SHALL lie on both the rolling-ball torus and the cylinder within tol, AND every sampled point of the cap seam SHALL lie on both the torus and the cap plane within tol, BEFORE the patch is assembled

#### Scenario: A seam that fails self-verify defers to OCCT (host)
- GIVEN a configuration whose computed seam does NOT lie on both surfaces within tol, OR whose ring-torus / seam-inside-face precondition fails (e.g. `r ≥ R/2`), built on the host
- WHEN the circular-crease fillet builder runs
- THEN it SHALL return a NULL Shape (no patch assembled) AND the operation SHALL defer to the OCCT `BRepFilletAPI_MakeFillet` oracle — it SHALL NOT return the unverified seam or a fabricated patch

### Requirement: Fillet self-verify selects the volume-change sign per convex / concave crease

The engine SHALL accept a native `cc_fillet_edges` candidate only after a mandatory watertight
+ volume-change self-verify whose SIGN is selected per crease type: a CONVEX fillet (a straight
planar dihedral or a convex circular cylinder↔cap rim) REMOVES material and SHALL be verified
with the SHRINK inequality `0 < Vr < Vo`; a CONCAVE fillet (a concave circular cylinder↔larger-
plane rim) ADDS material and SHALL be verified with the GROW inequality `Vr > Vo` — the SAME
grow branch (`blendResultVerified(..., wantGrow=true)`) the `offset_face` grow uses. The
dispatch SHALL try the planar path, then the convex-curved path (each verified SHRINK), then the
concave-curved path (verified GROW), and SHALL accept the FIRST candidate that passes ITS
correctly-signed self-verify. Because the convex and concave circular-rim classifiers are
MUTUALLY EXCLUSIVE (a coaxial cap versus a larger plane plus a material-side probe), at most one
curved builder SHALL return a non-NULL candidate for a given rim, so the sign is unambiguous. A
convex candidate SHALL NEVER be accepted by the grow branch and a concave candidate SHALL NEVER
be accepted by the shrink branch. If every builder returns NULL OR no candidate passes its
correctly-signed self-verify, the engine SHALL fall through to the OCCT `BRepFilletAPI_MakeFillet`
oracle. The engine SHALL NEVER accept a mis-signed candidate or weaken a tolerance to pass.

#### Scenario: Concave fillet is accepted only by the GROW self-verify branch (host)
- GIVEN a native boss-on-plate base rim filleted natively, whose watertight result adds material (`Vr > Vo`)
- WHEN the engine runs its mandatory self-verify on the candidate
- THEN the candidate SHALL be accepted by the GROW branch (`blendResultVerified(result, body, wantGrow=true)`), AND the SAME result SHALL FAIL the SHRINK branch (`wantGrow=false`) — confirming the sign is selected per crease and cannot be spoofed

#### Scenario: Convex fillet is accepted only by the SHRINK self-verify branch (host)
- GIVEN a native convex circular cylinder↔cap rim filleted natively, whose watertight result removes material (`0 < Vr < Vo`)
- WHEN the engine runs its mandatory self-verify on the candidate
- THEN the candidate SHALL be accepted by the SHRINK branch (`wantGrow=false`), AND the SAME result SHALL FAIL the GROW branch — no sign confusion between the convex and concave curved paths

### Requirement: Concave circular-crease fillet seams and torus patch are self-verified before assembly

Before assembling the CONCAVE circular cylinder↔larger-plane fillet, the native builder SHALL
compute the two trim seams as closed-form CIRCLES using the native SSI Stage-S1 handlers —
torus∩cylinder (coaxial) at radius `Rc` (the `v=0` inner-equator ring) and torus∩plane
(axis-perpendicular) at radius `Rc + radius` (the `v=π/2` ring) — and SHALL SELF-VERIFY each
seam by confirming it lies BOTH on the material-side rolling-ball torus (major `Rc + radius`,
minor `radius`) AND on its neighbouring original surface (the cylinder / the larger plane)
within a tolerance derived from the operands' scale, and SHALL assert G1-tangency (the torus
normal equals the cylinder radial at `v=0` and the plane normal at `v=π/2`, `cos = 1`). If EITHER
seam fails the on-both-surfaces or G1 check, OR the seam-inside-face preconditions do not hold
(the larger plane's extent does not reach `Rc + radius`, or the wall length does not cover the
`v=0` seam axial `H + radius`), the builder SHALL return a NULL Shape and the operation SHALL
defer to OCCT. The builder SHALL NEVER emit an unverified seam, weaken a tolerance to pass, or
fabricate a curved patch. This is a #6 instance of the roadmap's mandatory self-verify → OCCT-
fallback discipline.

#### Scenario: Both concave seam circles lie on the torus and their neighbour surfaces (host)
- GIVEN a native boss-on-plate base rim, built on the host with no OCCT
- WHEN the concave circular-crease fillet builder computes the two seam circles via native SSI S1
- THEN every sampled point of the cylinder seam SHALL lie on both the material-side torus and the cylinder within tol, AND every sampled point of the plane seam SHALL lie on both the torus and the larger plane within tol, AND both seams SHALL be G1-tangent (`cos = 1`), BEFORE the patch is assembled

#### Scenario: A concave seam that leaves its face defers to OCCT (host)
- GIVEN a configuration whose `Rc + radius` plane seam does NOT fit inside the larger plane's extent, OR whose `v=0` wall seam exceeds the wall length, built on the host
- WHEN the concave circular-crease fillet builder runs
- THEN it SHALL return a NULL Shape (no patch assembled) AND the operation SHALL defer to the OCCT `BRepFilletAPI_MakeFillet` oracle — it SHALL NOT return the unverified seam or a fabricated patch

### Requirement: Native variable-radius fillet (`cc_fillet_edges_variable`)

The native blend library SHALL compute `cc_fillet_edges_variable(body, edgeIds, edgeCount,
radius1, radius2)` NATIVELY for exactly ONE slice — a CONVEX circular cylinder↔cap crease
with a LINEAR radius law — and SHALL return a NULL Shape for everything else so the engine
defers to the OCCT `BRepFilletAPI_MakeFillet` (variable / evolved-radius) oracle.

The native slice is a CIRCULAR edge (a `Circle` `EdgeCurve` of radius `Rc` coaxial with a
cylinder axis `A`) shared by exactly two faces — one `FaceSurface`-kind-`Cylinder` lateral
face (radius `Rc`, axis `A`) and one `FaceSurface`-kind-`Plane` CAP whose normal is
parallel to `A` — meeting in a CONVEX dihedral (the SAME classifier the constant convex
`cc_fillet_edges` slice uses). For such a rim the builder SHALL apply the LINEAR radius law
`r(θ) = radius1 + (radius2 − radius1)·θ/2π` for the rim angle `θ ∈ [0, 2π)` (`r = radius1`
at `θ = 0`, `r → radius2` as `θ → 2π`) and SHALL construct the rolling-ball blend as a
SWEPT variable-radius canal on the air side: at each angular STATION `θ` the ball of radius
`r(θ)` tangent to the cylinder from outside and to the cap from the material side has its
centre at cylindrical radius `Rc − r(θ)` and axial `H − s·r(θ)` (`s` the axial sign toward
the cap), and the blend cross-section is the meridian circular arc of radius `r(θ)` from
the cylinder-seam point `(Rc, H − s·r(θ))` to the cap-seam point `(Rc − r(θ), H)`. The two
trim seams SHALL be the NON-circular varying-radius curves — the cylinder seam at radius
`Rc` with axial height `H − s·r(θ)` (the `v = 0` locus) and the cap seam at radius
`Rc − r(θ)` in the plane `z = H` (the `v = π/2` locus) — each lying on its neighbour
surface BY CONSTRUCTION (the wall seam has `radius = Rc` exactly; the cap seam has
`axial = H` exactly). The builder SHALL SELF-VERIFY G1-tangency at both seams at every
station (the canal normal is RADIAL == the cylinder normal at `v = 0` and AXIAL == the cap
normal at `v = π/2`, `cos = 1`, which holds for any radius law because `∂radius/∂v = 0` at
`v = 0` and `∂axial/∂v = 0` at `v = π/2`), tile the canal patch between the two seams into
deflection-bounded planar TRIANGLE facets sharing a common set of `N` angular stations with
the rebuilt wall and cap, and weld the patch and trimmed faces watertight through the
native `src/native/boolean` `assembleSolid`. The inserted variable canal face SHALL be
G1-tangent to BOTH the cylinder and the cap along its two varying-radius seams.

This variable fillet REMOVES material: its enclosed volume SHALL be strictly LESS than the
input's, and it SHALL be accepted ONLY by the engine self-verify's SHRINK branch
(`0 < Vr < Vo`). The result SHALL be a native `topology::Shape` of type `Solid`,
watertight (every edge shared by exactly two faces), accepted only after the engine's
mandatory watertight + volume-SHRINK self-verify — else DISCARDED → OCCT. This builder
SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. No `cc_*` signature or POD struct SHALL change. The linear-law slice is
gated additionally by the seam-inside-face precondition using `rmax = max(radius1,
radius2)` (`Rc ≥ 2·rmax`, cap radius `Rc − rmax > 0`, wall length ≥ the worst-case seam)
and by the curved-parity tolerance against OCCT; a NON-linear radius law, a CONCAVE
variable rim, a cylinder↔cylinder canal, a non-circular / tilted / non-coaxial / freeform
crease, or a radius gradient whose swept canal exceeds the curved-parity tolerance SHALL
return a NULL Shape → OCCT, and the measured OCCT-fallback gap SHALL be REPORTED, never
masked with a weakened tolerance.

#### Scenario: Cylinder top-rim variable fillet inserts a material-removing swept canal and is watertight (host)
- GIVEN a native cylinder solid of radius `Rc` and height `H` (built on the host with no OCCT) and its top rim (the CIRCULAR edge where the lateral `Cylinder` face meets the top `Plane` CAP in a CONVEX dihedral), with `0 < radius1`, `0 < radius2`, and `Rc ≥ 2·max(radius1, radius2)`
- WHEN `cc_fillet_edges_variable(cyl, {rim}, 1, radius1, radius2)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose inserted blend face is a SWEPT variable-radius canal coaxial with the cylinder axis, G1-tangent to the cylinder along the varying-height cylinder seam (radius `Rc`) and to the cap along the varying-radius cap seam (radius `Rc − r(θ)`), AND its enclosed volume SHALL be strictly LESS than the cylinder's, equal to `|cyl| − V_removed` (the closed-form SWEPT removed rim-band, the meridian corner area `r(θ)²·(1 − π/4)` integrated around the rim) within the tessellation deflection bound

#### Scenario: The constant limit reproduces the constant-radius torus fillet (host)
- GIVEN a native cylinder top rim filleted natively with `radius1 == radius2 == r` (`Rc ≥ 2r`)
- WHEN `cc_fillet_edges_variable(cyl, {rim}, 1, r, r)` is computed
- THEN the swept canal SHALL degenerate to the constant coaxial TORUS (major `Rc − r`, minor `r`) and its removed volume SHALL equal the constant convex `cc_fillet_edges` slice's within the tessellation deflection bound — no sign confusion, no regression

#### Scenario: Native variable fillet matches the OCCT BRepFilletAPI_MakeFillet (variable) oracle (parity)
- GIVEN a native cylinder top rim filleted by a linear law `(radius1, radius2)` on a booted iOS simulator
- WHEN `cc_fillet_edges_variable` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`), the OCCT side building the evolved fillet (`BRepFilletAPI_MakeFillet` with `SetRadius(radius1)` at one rim vertex and `SetRadius(radius2)` at the other)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of a variable blend face SHALL agree within the curved-parity tolerance, the native canal SHALL be G1-tangent to the cylinder and to the cap at its two varying-radius seams, AND the native result's volume SHALL be LESS than the input's (material removed); a fixture whose measured gap exceeds the tolerance SHALL be declared out of slice (NULL → OCCT) with the gap REPORTED, not passed with a loosened bound

#### Scenario: Out-of-slice variable fillet defers to OCCT (never faked)
- GIVEN a `cc_fillet_edges_variable` request that is NOT the supported slice — a NON-linear radius law, a CONCAVE variable rim (cylinder↔larger plane), a cylinder↔cylinder canal rim, a non-circular curved crease (cone↔plane / sphere / ellipse / spline), a tilted or non-coaxial plane, a freeform adjacent face, an edge shared by ≠2 faces, a near-degenerate radius (`Rc < 2·max(radius1, radius2)` or a seam leaving its face), or a radius gradient beyond the curved-parity tolerance
- WHEN `cc_fillet_edges_variable` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape (or the engine self-verify SHALL discard the candidate) AND the engine SHALL defer to the OCCT `BRepFilletAPI_MakeFillet` oracle — it SHALL NOT emit an approximate, hand-tuned, or fabricated variable blend, and the measured OCCT-fallback gap SHALL be REPORTED

### Requirement: Variable-radius fillet swept-canal seams and G1 tangency are self-verified before assembly

Before assembling the VARIABLE-radius convex cylinder↔cap fillet, the native builder SHALL
compute the two trim seams as closed-form NON-circular varying-radius curves — the cylinder
seam at radius `Rc` with axial `H − s·r(θ)` (the `v = 0` locus) and the cap seam at radius
`Rc − r(θ)` in the plane `z = H` (the `v = π/2` locus) — and SHALL SELF-VERIFY, at a set of
`N` angular stations covering `θ ∈ [0, 2π)`, that each cylinder-seam station lies on the
cylinder (`radius = Rc`) AND on the swept canal, and each cap-seam station lies on the cap
plane (`axial = H`) AND on the swept canal, within a tolerance derived from the operands'
scale, and SHALL assert G1-tangency at every station (the canal normal equals the cylinder
radial at `v = 0` and the cap axial at `v = π/2`, `cos = 1`, independent of the radius
gradient `r'(θ)`). If ANY station fails the on-both-surfaces or G1 check, OR the
seam-inside-face preconditions do not hold for `rmax = max(radius1, radius2)` (`Rc <
2·rmax`, the cap radius `Rc − rmax ≤ 0`, or the wall length does not cover the worst-case
`v = 0` seam), the builder SHALL return a NULL Shape and the operation SHALL defer to OCCT.
The builder SHALL NEVER emit an unverified seam, weaken a tolerance to pass, or fabricate a
curved patch. This is a #6 instance of the roadmap's mandatory self-verify → OCCT-fallback
discipline.

#### Scenario: Both non-circular seams lie on the canal and their neighbour surfaces at every station (host)
- GIVEN a native cylinder top rim with `Rc ≥ 2·max(radius1, radius2)`, built on the host with no OCCT
- WHEN the variable circular-crease fillet builder computes the two seam loci at `N` angular stations
- THEN every cylinder-seam station SHALL lie on both the swept canal and the cylinder (`radius = Rc`) within tol, AND every cap-seam station SHALL lie on both the canal and the cap plane (`axial = H`) within tol, AND both seams SHALL be G1-tangent (`cos = 1`) at every station, BEFORE the patch is assembled

#### Scenario: A variable seam that leaves its face defers to OCCT (host)
- GIVEN a configuration whose `Rc < 2·max(radius1, radius2)` (the swept centre curve reaches the axis) OR whose cap radius `Rc − max(radius1, radius2) ≤ 0` OR whose `v = 0` wall seam exceeds the wall length, built on the host
- WHEN the variable circular-crease fillet builder runs
- THEN it SHALL return a NULL Shape (no patch assembled) AND the operation SHALL defer to the OCCT `BRepFilletAPI_MakeFillet` oracle — it SHALL NOT return the unverified seam or a fabricated patch

### Requirement: Native curved (circular-rim) chamfer (`cc_chamfer_edges`)

The native blend library SHALL compute `cc_chamfer_edges(body, edgeIds, edgeCount,
distance)` NATIVELY for a CURVED slice — a CONVEX circular cylinder↔cap crease with a
SYMMETRIC chamfer distance — and SHALL return a NULL Shape for everything else so the
engine defers to the OCCT `BRepFilletAPI_MakeChamfer` (`Add(distance, edge)`) oracle.

The native curved slice is a CIRCULAR edge (a `Circle` `EdgeCurve` of radius `Rc` coaxial
with a cylinder axis `A`) shared by exactly two faces — one `FaceSurface`-kind-`Cylinder`
lateral face (radius `Rc`, axis `A`) and one `FaceSurface`-kind-`Plane` CAP whose normal
is parallel to `A` — meeting in a CONVEX dihedral (the SAME classifier the curved
`cc_fillet_edges` slice uses, `detail::facesOnRim` + `detail::rimGeom`). For such a rim the
builder SHALL apply the SYMMETRIC setback: the cylinder wall set back AXIALLY by `distance =
d` to the cylinder seam CIRCLE (radius `Rc`, axial `H − s·d`, `s` the axial sign toward the
cap) and the cap set back RADIALLY by `d` to the cap seam CIRCLE (radius `Rc − d`, axial
`H`). The builder SHALL construct the chamfer surface as a CONE FRUSTUM (a ruled truncated
cone — a STRAIGHT bevel, NOT a torus arc) bridging the two setback circles: `radius(τ) =
Rc − d·τ`, `axial(τ) = (H − s·d) + s·d·τ`, `τ ∈ [0, 1]`, revolved about `A`. The two trim
seams SHALL be the setback CIRCLES — the cylinder seam (radius `Rc`, `τ = 0`) and the cap
seam (radius `Rc − d`, `τ = 1`) — each lying on its neighbour surface BY CONSTRUCTION (the
cylinder seam has `radius = Rc` exactly; the cap seam has `axial = H` exactly).

The builder SHALL SELF-VERIFY the CORRECT BEVEL geometry — that the frustum meets each face
at the CHAMFER ANGLE and is **C0, NOT G1** (NOT tangent): the frustum outward normal makes
`cos = 1/√2` (≈ 0.70710678, the 45° symmetric bevel) with the cylinder radial normal at
`τ = 0` and `cos = 1/√2` with the cap axial normal at `τ = 1`, and the builder SHALL assert
these are NOT `1` (a chamfer is a straight bevel; asserting tangency would be geometrically
WRONG). The builder SHALL tile the frustum band between the two setback circles into
deflection-bounded planar TRIANGLE facets (`N` angular quads, each split into two planar
triangles, ONE meridian step) sharing a common set of `N` angular stations with the rebuilt
wall and both caps, and weld the patch and trimmed faces watertight through the native
`src/native/boolean` `assembleSolid`. The inserted cone-frustum face SHALL be C0 (at the
chamfer angle) to BOTH the cylinder and the cap along its two setback-circle seams.

This chamfer REMOVES material: its enclosed volume SHALL be strictly LESS than the input's,
equal to `|body| − π·d²·(Rc − d/3)` (the closed-form Pappus removed corner-ring volume)
within the tessellation deflection bound, and it SHALL be accepted ONLY by the engine
self-verify's SHRINK branch (`0 < Vr < Vo`). The result SHALL be a native `topology::Shape`
of type `Solid`, watertight (every edge shared by exactly two faces), accepted only after
the engine's mandatory watertight + volume-SHRINK self-verify — else DISCARDED → OCCT. This
builder SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. No `cc_*` signature or POD struct SHALL change. The curved slice is
gated additionally by the seam-inside-face precondition (`Rc − d > 0` so the cap circle is
real; the wall length covers the axial setback `H − s·d`); an ASYMMETRIC two-distance
chamfer, a CONCAVE circular rim, a cylinder↔cylinder chamfer, a non-circular / tilted /
non-coaxial / freeform crease, `Rc ≤ d`, or a multi-edge selection SHALL return a NULL
Shape → OCCT, and the measured OCCT-fallback gap SHALL be REPORTED, never masked with a
weakened tolerance.

#### Scenario: Cylinder top-rim chamfer inserts a material-removing cone frustum and is watertight (host)
- GIVEN a native cylinder solid of radius `Rc` and height `H` (built on the host with no OCCT) and its top rim (the CIRCULAR edge where the lateral `Cylinder` face meets the top `Plane` CAP in a CONVEX dihedral), with `0 < distance = d` and `Rc − d > 0`
- WHEN `cc_chamfer_edges(cyl, {rim}, 1, d)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose inserted blend face is a CONE FRUSTUM coaxial with the cylinder axis, meeting the cylinder along the setback circle (radius `Rc`, axial `H − s·d`) and the cap along the setback circle (radius `Rc − d`, axial `H`) at the chamfer angle (C0, NOT G1), AND its enclosed volume SHALL be strictly LESS than the cylinder's, equal to `|cyl| − π·d²·(Rc − d/3)` within the tessellation deflection bound

#### Scenario: The bevel is C0 at the chamfer angle, NOT tangent (host)
- GIVEN a native cylinder top rim chamfered natively with a symmetric distance `d` (`Rc − d > 0`)
- WHEN the cone-frustum chamfer builder computes the bevel and its two setback seams
- THEN the frustum outward normal SHALL make `cos = 1/√2` (the 45° symmetric bevel) with the cylinder radial normal at the cylinder seam AND `cos = 1/√2` with the cap axial normal at the cap seam, and the builder SHALL assert these are NOT `1` (the bevel is C0 — a straight chamfer, NOT a G1-tangent fillet) — proving the chamfer is a flat bevel, not a rounded arc

#### Scenario: A chamfer removes more material than the equal-setback fillet (host)
- GIVEN a native cylinder top rim, a setback `d`, chamfered natively (removed `π·d²·(Rc − d/3)`) and, separately, filleted natively with radius `d`
- WHEN both removed volumes are computed
- THEN the chamfer's removed volume SHALL be strictly GREATER than the fillet's (the flat bevel cuts inside the rounded arc — the meridian triangle `d²/2` vs the fillet's `d²(1 − π/4)`), confirming the correct sign and blend type with no confusion between the frustum and the torus

#### Scenario: Native curved chamfer matches the OCCT BRepFilletAPI_MakeChamfer oracle (parity)
- GIVEN a native cylinder top rim chamfered by a symmetric distance `d` on a booted iOS simulator
- WHEN `cc_chamfer_edges` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`), the OCCT side building the chamfer with `BRepFilletAPI_MakeChamfer` + `Add(d, edge)` (symmetric)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of a flat cone-frustum bevel face SHALL agree within the tessellation deflection bound (a TIGHT bound, since the symmetric chamfer IS EXACTLY a cone frustum — not a loosened curved-parity band), the native bevel SHALL be C0 (at the chamfer angle) to the cylinder and to the cap at its two setback-circle seams and NOT G1, AND the native result's volume SHALL be LESS than the input's (material removed); a fixture whose measured gap exceeds the bound SHALL be declared out of slice (NULL → OCCT) with the gap REPORTED, not passed with a loosened bound

#### Scenario: Out-of-slice curved chamfer defers to OCCT (never faked)
- GIVEN a `cc_chamfer_edges` request that is NOT the supported slice — an ASYMMETRIC two-distance chamfer, a CONCAVE circular rim (cylinder↔larger plane), a cylinder↔cylinder (curved↔curved) rim, a non-circular curved crease (cone↔plane / sphere / ellipse / spline), a tilted or non-coaxial plane, a freeform adjacent face, an edge shared by ≠2 faces, a near-degenerate distance (`Rc ≤ d` or a wall shorter than `d`), or a multi-edge selection
- WHEN `cc_chamfer_edges` is invoked with the native engine active
- THEN the native curved builder SHALL return a NULL Shape (or the engine self-verify SHALL discard the candidate) AND the engine SHALL defer to the OCCT `BRepFilletAPI_MakeChamfer` oracle — it SHALL NOT emit an approximate, hand-tuned, or fabricated curved chamfer, and the measured OCCT-fallback gap SHALL be REPORTED

### Requirement: Curved-chamfer cone-frustum seams and C0 bevel geometry are self-verified before assembly

Before assembling the CURVED (circular-rim) chamfer, the native builder SHALL compute the
two trim seams as closed-form setback CIRCLES — the cylinder seam at radius `Rc`, axial
`H − s·d` (the `τ = 0` locus) and the cap seam at radius `Rc − d`, `z = H` (the `τ = 1`
locus) — and SHALL SELF-VERIFY, at a set of `N` angular stations covering `θ ∈ [0, 2π)`,
that each cylinder-seam station lies on the cylinder (`radius = Rc`) AND on the cone
frustum, and each cap-seam station lies on the cap plane (`axial = H`) AND on the cone
frustum, within a tolerance derived from the operands' scale. The builder SHALL further
SELF-VERIFY the CORRECT BEVEL geometry — that the frustum meets each face at the chamfer
angle and is **C0, NOT G1** — by asserting the frustum normal makes `cos = 1/√2` (the 45°
symmetric bevel) with the cylinder radial normal at `τ = 0` and `cos = 1/√2` with the cap
axial normal at `τ = 1`, and that neither equals `1` (tangency, which would be WRONG for a
chamfer). If ANY station fails the on-both-surfaces or bevel-angle check, OR the
seam-inside-face preconditions do not hold (`Rc ≤ d` so the cap circle collapses, or the
wall length does not cover the `τ = 0` seam at `H − s·d`), the builder SHALL return a NULL
Shape and the operation SHALL defer to OCCT. The builder SHALL NEVER emit an unverified
seam, weaken a tolerance to pass, assert G1 tangency for the bevel, or fabricate a curved
patch. This is a #6 instance of the roadmap's mandatory self-verify → OCCT-fallback
discipline.

#### Scenario: Both setback circles lie on the frustum and their neighbour surfaces (host)
- GIVEN a native cylinder top rim with `Rc − d > 0`, built on the host with no OCCT
- WHEN the curved-chamfer builder computes the two seam circles at `N` angular stations
- THEN every cylinder-seam station SHALL lie on both the cone frustum and the cylinder (`radius = Rc`) within tol, AND every cap-seam station SHALL lie on both the frustum and the cap plane (`axial = H`) within tol, AND the frustum SHALL be C0 (chamfer-angle `cos = 1/√2`, NOT `1`) to each face at its seam, BEFORE the patch is assembled

#### Scenario: A degenerate curved chamfer that leaves its face defers to OCCT (host)
- GIVEN a configuration whose `Rc ≤ d` (the cap circle `Rc − d ≤ 0` collapses / crosses the axis) OR whose `τ = 0` cylinder seam at `H − s·d` exceeds the wall length, built on the host
- WHEN the curved-chamfer builder runs
- THEN it SHALL return a NULL Shape (no patch assembled) AND the operation SHALL defer to the OCCT `BRepFilletAPI_MakeChamfer` oracle — it SHALL NOT return the unverified seam or a fabricated patch

