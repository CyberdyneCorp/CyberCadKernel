# native-blends

This change (Phase 4 #6 curved blends, CONCAVE circular-rim slice) extends the living
`native-blends` capability: the native constant-radius `cc_fillet_edges` CURVED path
gains the CONCAVE circular crease — a CIRCULAR edge where a cylinder LATERAL face meets
a LARGER coaxial PLANE in a CONCAVE dihedral (a boss base rim; a blind-hole bottom
rim). Its rolling-ball canal surface is still a coaxial TORUS (major `Rc + radius`,
minor `radius`) and its two tangent trim seams are still CIRCLES (torus∩cylinder at
`Rc` + torus∩plane at `Rc + radius`, native SSI S1) — but the ball seats on the
MATERIAL side (offset sign FLIPS to `+radius`), so the fillet ADDS material and the
volume GROWS. No `cc_*` ABI change; the default engine stays OCCT; everything outside
the named slice returns NULL → OCCT.

## MODIFIED Requirements

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

## ADDED Requirements

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
