# native-blends

This change (Phase 4 #6 curved blends, VARIABLE-radius circular-rim slice) extends the
living `native-blends` capability: `cc_fillet_edges_variable` gains a NATIVE path for the
CONVEX circular crease — a CIRCULAR edge where a cylinder LATERAL face meets a coaxial
PLANAR cap in a CONVEX dihedral — with a LINEAR radius law
`r(θ) = r1 + (r2 − r1)·θ/2π`, `θ ∈ [0, 2π)`. Its rolling-ball centre is a SWEPT curve
(radius `Rc − r(θ)`, axial `H − s·r(θ)`), the blend surface is a variable-radius canal (a
per-station meridian arc of the local `r(θ)`), and its two trim seams are NON-circular
varying-radius curves (cylinder seam at `Rc`, varying axial `H − s·r(θ)`; cap seam at
varying radius `Rc − r(θ)`, `z = H`) — G1-tangent to both faces at every station. The
fillet REMOVES material, so the volume SHRINKS. No `cc_*` ABI change; the default engine
stays OCCT; everything outside the named slice (non-linear law / non-circular / concave-
variable / cyl↔cyl-canal / tilted / freeform / out-of-parity gradient) returns NULL →
OCCT.

## ADDED Requirements

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

## MODIFIED Requirements

### Requirement: Curved-face, concave, variable-radius, face-fillet, and multi-edge cases fall through to OCCT

The native blend builders SHALL DECLINE (return a NULL `Shape`) — and `NativeEngine` SHALL fall
through to OCCT — for any case outside the tractable slice: (1) any face touching the
operation is CURVED in a way no native curved slice handles (`FaceSurface::kind != Plane` and
not one of the supported cylinder↔cap fillet rims — sphere / cone / NURBS); (2) a selected
edge's dihedral is CONCAVE and no native concave slice handles it; (3) MULTI-EDGE
interference (two or more selected edges whose blends overlap at a shared corner / setback);
(4) a NON-PLANAR / non-convex shell solid, a non-uniform thickness, or a wall ≥ half the smallest
span; (5) `body` is NOT a native body (a foreign / OCCT-built shape id); or (6) degenerate input
(zero-length edge, zero-area face, non-positive radius / distance / thickness). For
`cc_fillet_edges_variable` (variable radius), the native path handles ONLY the LINEAR-law
CONVEX circular cylinder↔cap rim (`r(θ) = radius1 + (radius2 − radius1)·θ/2π`, both seams
inside their faces, gradient within the curved-parity tolerance); every OTHER variable case —
a NON-linear radius law, a CONCAVE variable rim, a cylinder↔cylinder canal, a non-circular /
tilted / non-coaxial / freeform crease, or an out-of-parity gradient — SHALL fall through to
OCCT. `cc_fillet_face` (face fillet) SHALL always fall through to OCCT (not implemented
natively). Each fall-through case SHALL produce EXACTLY the fallback (OCCT) engine's result.
The change SHALL NOT fake, stub-out, or partially implement any deferred case; each SHALL be
labelled and verified as a fall-through, never faked.

#### Scenario: A curved-face input falls through (host + parity)
- GIVEN a fillet / chamfer / offset / shell where a face touching the operation is curved and no native curved slice handles it (e.g. a filleted-box or cone body), with the native engine active (`cc_set_engine(1)`)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return a NULL `Shape` AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A concave edge with no native slice falls through (host)
- GIVEN a `cc_fillet_edges` or `cc_chamfer_edges` on a CONCAVE (reflex-dihedral) edge that no native concave slice handles, with the native engine active
- WHEN the op is invoked
- THEN the native builder SHALL return a NULL `Shape` (rather than emit a wrong / self-intersecting solid) AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: An out-of-slice variable-radius or face-fillet call falls through (host + parity)
- GIVEN a `cc_fillet_edges_variable` call OUTSIDE the linear-law convex circular slice (a non-linear law, a concave variable rim, a cyl↔cyl canal, a non-circular / tilted / freeform crease, or an out-of-parity gradient) OR a `cc_fillet_face` call, with the native engine active
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
`EngineShape`. For `fillet_edges_variable` the override SHALL call
`nblend::variable_fillet_edge(body, edgeIds, edgeCount, radius1, radius2)` and accept the
candidate ONLY through `blendResultVerified(result, body, wantGrow=false)` (watertight +
`0 < Vr < Vo`), since a convex variable fillet REMOVES material — the SAME SHRINK branch the
constant convex `fillet_edges` candidate uses. When `body` is not native, when the native
builder returns a NULL `Shape` (an out-of-slice DECLINE), or when the self-verify guard rejects
the candidate, the override SHALL fall through to the held fallback engine with **no native
interception** for a foreign body, and for a native body it cannot forward (OCCT would misread
the native void) SHALL return an honest error so the call is served by the OCCT engine — exactly
how the constant `fillet_edges` treats an unbuildable native rim. `fillet_face` SHALL remain a
pure fall-through (no native builder call). OCCT SHALL be referenced ONLY under
`CYBERCAD_HAS_OCCT` (in the fallback wiring); the native builder SHALL reference no OCCT /
`IEngine` / `EngineShape` type. No `cc_*` signature or POD layout SHALL change and the default
engine SHALL remain OCCT (opt-in via `cc_set_engine(1)`).

#### Scenario: A supported planar or curved blend is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a native solid with a supported selection (a convex planar-dihedral edge / planar face / box shell / convex or concave circular cylinder↔plane rim / a linear-law convex circular variable rim)
- WHEN `cc_chamfer_edges` / `cc_fillet_edges` / `cc_fillet_edges_variable` / `cc_offset_face` / `cc_shell` is invoked for the supported case
- THEN the shape SHALL be built by `src/native/blend/` and PASS its correctly-signed self-verify with no fallback call AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: An unsupported or unverified blend falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred case (curved face with no native slice, concave edge with no native slice, out-of-slice variable radius, face fillet, multi-edge interference, foreign body) OR a candidate that fails the self-verify
- WHEN the `cc_*` op is invoked
- THEN the native builder SHALL return NULL (or the guard SHALL reject, or the method is a pure fall-through) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`) for a foreign body, or an honest error served by the OCCT engine for a native body — proving fall-through with no native interception

### Requirement: Blend parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME `cc_chamfer_edges`
/ `cc_fillet_edges` / `cc_fillet_edges_variable` / `cc_offset_face` / `cc_shell` calls SHALL be
issued once with the native engine active (`cc_set_engine(1)`) and once with the OCCT default
(`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through the `cc_*` facade —
mass properties, bounding box, sub-shape counts, and watertight tessellation — against the OCCT
`BRepFilletAPI` / `BRepOffsetAPI` oracle. On **box** chamfer / offset / shell the native result
SHALL match the oracle EXACTLY (volume / bbox / centroid relative error ~0, fp precision); the
constant-radius fillet AND the linear-law variable fillet SHALL match within the tessellation
deflection / curved-parity bound (the variable side compared against OCCT
`BRepFilletAPI_MakeFillet` with the evolved-radius law `SetRadius(radius1)` / `SetRadius(radius2)`
at the two rim vertices). The fall-through cases (a curved-face input, a concave edge with no
native slice, an OUT-OF-SLICE variable-radius call, a `cc_fillet_face` call, and a
multi-edge-interference selection) SHALL be asserted identical under both engines (fall-through
proof, `cc_active_engine() == 1`). The parity test SHALL restore the OCCT default in teardown and
SHALL carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite count is unchanged.

#### Scenario: Native box blends match the OCCT BRepFilletAPI / BRepOffsetAPI oracle (parity)
- GIVEN box chamfer / offset / shell cases and a box convex-edge fillet on a booted iOS simulator
- WHEN each `cc_*` op is called with the native engine active and with the OCCT default
- THEN the chamfer / offset / shell shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT oracle, and the fillet SHALL agree within the tessellation deflection bound

#### Scenario: Native linear-law variable fillet matches the OCCT evolved-law oracle (parity)
- GIVEN a native cylinder top rim filleted by a linear law `(radius1, radius2)` on a booted iOS simulator, with `Rc ≥ 2·max(radius1, radius2)` and the gradient within the curved-parity tolerance
- WHEN `cc_fillet_edges_variable` is called with the native engine active and with the OCCT default (the OCCT side building `BRepFilletAPI_MakeFillet` with `SetRadius(radius1)` / `SetRadius(radius2)` at the two rim vertices)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of a variable blend face SHALL agree within the curved-parity tolerance, and the native result's volume SHALL be LESS than the input's (material removed)

#### Scenario: Fall-through blend cases are identical under both engines (parity)
- GIVEN a curved-face blend, a concave-edge blend with no native slice, an out-of-slice variable-radius call, a `cc_fillet_face` call, and a multi-edge-interference selection on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change
