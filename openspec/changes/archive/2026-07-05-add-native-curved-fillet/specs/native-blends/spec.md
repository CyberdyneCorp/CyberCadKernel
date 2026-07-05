# native-blends

This change (Phase 4 #6 curved blends, FIRST SLICE) extends the living
`native-blends` capability: the native constant-radius `cc_fillet_edges` gains its
FIRST curved crease — a CIRCULAR edge where a cylinder LATERAL face meets a PLANAR
CAP, whose rolling-ball canal surface is a TORUS and whose two tangent trim seams
are CIRCLES (torus∩cylinder + torus∩plane, native SSI S1). No `cc_*` ABI change; the
default engine stays OCCT; everything outside the named slice returns NULL → OCCT.

## MODIFIED Requirements

### Requirement: Native constant-radius fillet (`cc_fillet_edges`)

The native blend library SHALL compute `cc_fillet_edges(body, edgeIds, edgeCount, radius)`
NATIVELY in the following two cases (else return a NULL Shape so the engine falls through
to the OCCT `BRepFilletAPI_MakeFillet` oracle):

1. **Straight planar dihedral (existing).** A STRAIGHT edge shared by exactly two
   `FaceSurface`-kind-`Plane` faces with a CONVEX dihedral and no multi-edge corner
   interference. For each such edge the builder SHALL construct the rolling-ball tangent
   CYLINDER (radius `radius`, axis parallel to the edge on the interior bisector at distance
   `radius / sin(halfAngle)`, tangent to both faces along two contact lines) — REUSING the
   full-round tangent-cylinder construction in `src/native/construct` — trim both faces back
   to their contact lines, insert the cylindrical blend face and the planar setback faces, and
   close the solid. On a box convex edge the result SHALL match the OCCT oracle within a
   deflection bound, with removed volume `(1 − π/4)·radius²·edgeLength`.

2. **Circular cylinder↔cap crease (curved, first slice).** A CIRCULAR edge (a `Circle`
   `EdgeCurve` of radius `R` coaxial with a cylinder axis `A`) shared by exactly two faces —
   one `FaceSurface`-kind-`Cylinder` lateral face (radius `R`, axis `A`) and one
   `FaceSurface`-kind-`Plane` CAP whose normal is parallel to `A` — with a CONVEX crease and
   `radius` small enough that both tangent circles stay inside their faces (in particular
   `R ≥ 2·radius`, a ring torus). For such a rim the builder SHALL construct the rolling-ball
   canal surface as a coaxial **TORUS** (major radius `R − radius`, minor radius `radius`,
   centred on the axis at the cap height offset axially by `radius`) via `native-math`
   `Torus` and the `native-construction` revolve, compute its two tangent trim seams as the
   **CIRCLES** torus∩cylinder (coaxial) and torus∩plane (axis-perpendicular) using the native
   SSI Stage-S1 closed-form handlers, SELF-VERIFY each seam lies on the torus AND on its
   neighbouring original surface within a scale-derived tolerance, trim the cylinder lateral
   face and the cap face back to their seam circles, tile the torus blend patch between the
   two seams into deflection-bounded facets, and weld the patch and trimmed faces watertight
   through the native `src/native/boolean` `assembleSolid`. The inserted torus blend face
   SHALL be G1-tangent to BOTH the cylinder and the cap along its seam circles. On a native
   cylinder solid's top rim the result SHALL match the OCCT `BRepFilletAPI_MakeFillet` oracle
   within a deflection / curved-parity bound.

In both cases the result SHALL be a native `topology::Shape` of type `Solid`, watertight
(every edge shared by exactly two faces), whose enclosed volume is strictly LESS than the
input's, and it SHALL be accepted only after the engine's mandatory watertight +
volume-reducing self-verify (see the self-verify requirement) — else DISCARDED → OCCT. This
builder SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. No `cc_*` signature or POD struct SHALL change.

#### Scenario: Box convex-edge fillet is a watertight cylinder-blended solid (host)
- GIVEN an axis-aligned box `B` (native planar solid, built on the host with no OCCT) and one convex STRAIGHT edge of length `L`
- WHEN `cc_fillet_edges(B, {edge}, 1, r)` is computed and the result is tessellated
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose blend face is a `Cylinder` of radius `r` tangent to both adjoining faces AND its enclosed volume SHALL equal `|B| − (1 − π/4)·r²·L` within the tessellation deflection bound

#### Scenario: Box fillet matches the OCCT BRepFilletAPI_MakeFillet oracle within a deflection bound (parity)
- GIVEN an axis-aligned box convex STRAIGHT edge filleted by `radius` on a booted iOS simulator
- WHEN `cc_fillet_edges` is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the tessellation deflection bound with the OCCT `BRepFilletAPI_MakeFillet` oracle, and the native blend face SHALL be a cylinder of radius `radius`

#### Scenario: Cylinder top-rim fillet inserts a torus blend patch and is watertight (host)
- GIVEN a native cylinder solid of radius `R` and height `H` (built on the host with no OCCT) and its top rim (the CIRCULAR edge where the lateral `Cylinder` face meets the top `Plane` cap), with `0 < r` and `R ≥ 2r`
- WHEN `cc_fillet_edges(cyl, {rim}, 1, r)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose inserted blend face is a TORUS patch of minor radius `r` and major radius `R − r` coaxial with the cylinder axis, tangent (G1) to the cylinder along the circle of radius `R` and to the cap along the circle of radius `R − r`, AND its enclosed volume SHALL be strictly LESS than the cylinder's, equal to `|cyl| − V_wedge` (the closed-form removed rim-band) within the tessellation deflection bound

#### Scenario: Cylinder top-rim fillet matches the OCCT BRepFilletAPI_MakeFillet oracle (parity)
- GIVEN a native cylinder solid's top rim filleted by `radius` (with `R ≥ 2·radius`) on a booted iOS simulator
- WHEN `cc_fillet_edges` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of a toroidal blend face SHALL agree within the curved-parity tolerance with the OCCT `BRepFilletAPI_MakeFillet` oracle, and the native torus patch SHALL be G1-tangent to the cylinder and to the cap at its two seam circles

#### Scenario: Out-of-slice curved rim defers to OCCT (never faked)
- GIVEN a fillet request that is NOT the supported slice — a CONCAVE circular rim, a variable radius, a cylinder↔cylinder canal rim, a non-circular curved crease (cone↔plane / sphere / ellipse / spline), a freeform adjacent face, an edge shared by ≠2 faces, or a near-degenerate radius (`r ≥ R/2`, or a seam leaving its face)
- WHEN `cc_fillet_edges` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape (or the engine self-verify SHALL discard the candidate) AND the engine SHALL fall through to the OCCT `BRepFilletAPI_MakeFillet` oracle — it SHALL NOT emit an approximate, hand-tuned, or fabricated curved blend

## ADDED Requirements

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
