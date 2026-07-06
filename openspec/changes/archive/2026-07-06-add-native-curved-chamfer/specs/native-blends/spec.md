# native-blends

This change (Phase 4 #6 curved blends, CURVED circular-rim CHAMFER slice) extends the
living `native-blends` capability: `cc_chamfer_edges` gains a NATIVE path for the CONVEX
circular crease — a CIRCULAR edge where a cylinder LATERAL face meets a coaxial PLANAR cap
in a CONVEX dihedral — with a SYMMETRIC chamfer distance `d`. Unlike the curved FILLET
(which inserts a G1-tangent torus arc), the chamfer cuts a FLAT BEVEL: a CONE FRUSTUM
between the two SETBACK circles (cylinder seam at radius `Rc`, axial `H − s·d`; cap seam
at radius `Rc − d`, `z = H`), meeting each face at the chamfer angle **C0, NOT G1**. The
chamfer REMOVES material (exact removed volume `π·d²·(Rc − d/3)`), so the volume SHRINKS.
No `cc_*` ABI change; the default engine stays OCCT; everything outside the named slice
(asymmetric / non-circular / concave / curved↔curved / tilted / freeform / `Rc ≤ d`)
returns NULL → OCCT.

## ADDED Requirements

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

## MODIFIED Requirements

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
