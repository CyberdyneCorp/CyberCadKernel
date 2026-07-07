# native-blends

This change (Phase 4 #6 curved blends, the OFF-THE-CIRCLE breadth batch) extends the
living `native-blends` capability with three tracks: (T1) an ASYMMETRIC two-distance chamfer
on the convex circular cylinder↔cap rim (`cc_chamfer_edges_asym`, an OBLIQUE cone frustum) —
LANDED, EXACT; (T2) a NON-CIRCULAR (elliptical) crease fillet (`cc_fillet_edges` on a
cylinder↔oblique-plane ELLIPSE rim, a general `r`-circle canal) OR its documented HONEST
DECLINE; and (T3) a cylinder↔cylinder canal fillet (`cc_fillet_edges` on a cyl↔cyl marching
crease) OR its documented HONEST DECLINE. IMPLEMENTATION OUTCOME: T1 LANDED (native, exact,
host + iOS-sim parity vs `MakeChamfer::Add(d1,d2,edge,face)`); T2 and T3 are both HONEST
DECLINES (no dead code) — T2 because no OCCT-free constructor yields a native
Cylinder+oblique-Plane+Ellipse body (the native path is unreachable), T3 because a single
swept-`r`-circle canal cannot close the Steinmetz-pole corner blend watertight/G1. Chamfers
stay C0 (asserted at the per-seam bevel angle); fillets stay G1 (asserted at both contact
curves). All REMOVE material (SHRINK-verified). `cc_fillet_edges` / `cc_chamfer_edges` keep
their signatures; T1 adds the ADDITIVE `cc_chamfer_edges_asym`. Everything outside the landed
slices returns NULL → OCCT `BRepFilletAPI`, and the measured gap is REPORTED.

## ADDED Requirements

### Requirement: Native asymmetric two-distance chamfer (`cc_chamfer_edges_asym`)

The native blend library SHALL compute an ASYMMETRIC two-distance chamfer
`cc_chamfer_edges_asym(body, edgeIds, edgeCount, distance1, distance2)` NATIVELY for the
CONVEX circular cylinder↔cap crease, and SHALL return a NULL Shape for everything else so
the engine defers to the OCCT `BRepFilletAPI_MakeChamfer` (`Add(distance1, distance2, edge,
face)`) oracle.

The native slice is a CIRCULAR edge (a `Circle` `EdgeCurve` of radius `Rc` coaxial with a
cylinder axis `A`) shared by exactly two faces — one `FaceSurface`-kind-`Cylinder` lateral
face (radius `Rc`, axis `A`) and one `FaceSurface`-kind-`Plane` CAP whose normal is parallel
to `A` — meeting in a CONVEX dihedral (the SAME classifier the symmetric curved chamfer uses,
`detail::facesOnRim` + `detail::rimGeom`). For such a rim the builder SHALL apply the
ASYMMETRIC setback: the cylinder wall set back AXIALLY by `distance1 = d1` to the cylinder
seam CIRCLE (radius `Rc`, axial `H − s·d1`, `s` the axial sign toward the cap) and the cap
set back RADIALLY by `distance2 = d2` to the cap seam CIRCLE (radius `Rc − d2`, axial `H`).
The builder SHALL construct the chamfer surface as an OBLIQUE CONE FRUSTUM (a ruled truncated
cone — a STRAIGHT bevel, NOT a torus arc) bridging the two setback circles: `radius(τ) =
Rc − d2·τ`, `axial(τ) = (H − s·d1) + s·d1·τ`, `τ ∈ [0, 1]`, revolved about `A`. The two trim
seams SHALL be the setback CIRCLES — the cylinder seam (radius `Rc`, `τ = 0`) and the cap
seam (radius `Rc − d2`, `τ = 1`) — each lying on its neighbour surface BY CONSTRUCTION.

The builder SHALL SELF-VERIFY the CORRECT BEVEL geometry — that the frustum meets each face
at its chamfer angle and is **C0, NOT G1** (NOT tangent): the frustum outward normal
(`radial·d1 + axial·s·d2`, normalized) makes `cos = d1/√(d1²+d2²)` with the cylinder radial
normal at `τ = 0` and `cos = d2/√(d1²+d2²)` with the cap axial normal at `τ = 1` (equal only
when `d1 = d2`), and the builder SHALL assert NEITHER equals `1` (a chamfer is a straight
bevel; asserting tangency would be geometrically WRONG). The builder SHALL tile the frustum
band between the two setback circles into deflection-bounded planar TRIANGLE facets (`N`
angular quads, each split into two planar triangles, ONE meridian step) sharing a common set
of `N` angular stations with the rebuilt wall and both caps, and weld the patch and trimmed
faces watertight through the native `src/native/boolean` `assembleSolid`.

This chamfer REMOVES material: its enclosed volume SHALL be strictly LESS than the input's,
equal to `|body| − π·d1·d2·(Rc − d2/3)` (the closed-form Pappus removed corner-ring volume,
reducing to the symmetric `π·d²·(Rc − d/3)` when `d1 = d2 = d`) within the tessellation
deflection bound, and it SHALL be accepted ONLY by the engine self-verify's SHRINK branch
(`0 < Vr < Vo`). The result SHALL be a native `topology::Shape` of type `Solid`, watertight,
accepted only after the engine's mandatory watertight + volume-SHRINK self-verify — else
DISCARDED → OCCT. This builder SHALL remain OCCT-free and host-buildable and SHALL reference
no OCCT / `IEngine` / `EngineShape` type. The curved slice is gated by the seam-inside-face
precondition (`Rc − d2 > 0` so the cap circle is real; the wall length covers the axial
setback `H − s·d1`); a CONCAVE circular rim, a cylinder↔cylinder chamfer, a non-circular /
tilted / non-coaxial / freeform crease, `Rc ≤ d2`, a wall shorter than `d1`, or a multi-edge
selection SHALL return a NULL Shape → OCCT, and the measured OCCT-fallback gap SHALL be
REPORTED, never masked with a weakened tolerance.

#### Scenario: Cylinder top-rim asymmetric chamfer inserts an oblique cone frustum and is watertight (host)
- GIVEN a native cylinder solid of radius `Rc` and height `H` (built on the host with no OCCT) and its top rim (the CIRCULAR edge where the lateral `Cylinder` face meets the top `Plane` CAP in a CONVEX dihedral), with `0 < d1`, `0 < d2`, and `Rc − d2 > 0`
- WHEN `cc_chamfer_edges_asym(cyl, {rim}, 1, d1, d2)` is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose inserted blend face is an OBLIQUE CONE FRUSTUM coaxial with the cylinder axis, meeting the cylinder along the setback circle (radius `Rc`, axial `H − s·d1`) and the cap along the setback circle (radius `Rc − d2`, axial `H`), AND its enclosed volume SHALL be strictly LESS than the cylinder's, equal to `|cyl| − π·d1·d2·(Rc − d2/3)` within the tessellation deflection bound

#### Scenario: The oblique bevel is C0 at two DIFFERENT chamfer angles, NOT tangent (host)
- GIVEN a native cylinder top rim chamfered natively with `d1 ≠ d2` (`Rc − d2 > 0`)
- WHEN the oblique cone-frustum chamfer builder computes the bevel and its two setback seams
- THEN the frustum outward normal SHALL make `cos = d1/√(d1²+d2²)` with the cylinder radial normal at the cylinder seam AND `cos = d2/√(d1²+d2²)` with the cap axial normal at the cap seam, the two values SHALL DIFFER (since `d1 ≠ d2`), and the builder SHALL assert NEITHER equals `1` (the bevel is C0 — a straight chamfer, NOT a G1-tangent fillet)

#### Scenario: The symmetric distance is the exact `d1 = d2` special case (host)
- GIVEN a native cylinder top rim chamfered natively with `d1 = d2 = d`
- WHEN the asymmetric builder computes the frustum and the removed volume
- THEN the removed volume SHALL equal the symmetric `π·d²·(Rc − d/3)` and both bevel angles SHALL be `cos = 1/√2`, matching the existing symmetric curved chamfer exactly

#### Scenario: Native asymmetric chamfer matches the OCCT MakeChamfer(d1,d2) oracle (parity)
- GIVEN a native cylinder top rim chamfered by two distances `d1 ≠ d2` on a booted iOS simulator, with `Rc − d2 > 0`
- WHEN `cc_chamfer_edges_asym` is called with the native engine active (`cc_set_engine(1)`) and the OCCT side builds `BRepFilletAPI_MakeChamfer` with `Add(d1, d2, edge, face)` (`face` = the cylinder wall carrying `d1`)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of an oblique cone-frustum bevel face SHALL agree within the tessellation deflection bound (a TIGHT bound, since the oblique frustum is EXACT), the native bevel SHALL be C0 (at the two chamfer angles) and NOT G1, AND the native result's volume SHALL be LESS than the input's; a fixture whose measured gap exceeds the bound SHALL be declared out of slice (NULL → OCCT) with the gap REPORTED

#### Scenario: Out-of-slice asymmetric chamfer defers to OCCT (never faked)
- GIVEN a `cc_chamfer_edges_asym` request that is NOT the supported slice — a CONCAVE circular rim, a cylinder↔cylinder rim, a non-circular curved crease (cone↔plane / sphere / ellipse / spline), a tilted or non-coaxial plane, a freeform adjacent face, an edge shared by ≠2 faces, `Rc ≤ d2` (the cap circle collapses), a wall shorter than `d1`, or a multi-edge selection
- WHEN `cc_chamfer_edges_asym` is invoked with the native engine active
- THEN the native builder SHALL return a NULL Shape (or the engine self-verify SHALL discard the candidate) AND the engine SHALL defer to the OCCT `BRepFilletAPI_MakeChamfer` oracle — it SHALL NOT emit an approximate or fabricated asymmetric chamfer, and the measured OCCT-fallback gap SHALL be REPORTED

### Requirement: Native non-circular (elliptical) crease fillet (`cc_fillet_edges`)

The native blend library SHALL compute `cc_fillet_edges(body, edgeIds, edgeCount, radius)`
NATIVELY for a NON-CIRCULAR CURVED slice — a CONSTANT-radius rolling-ball fillet on the
ELLIPTICAL rim where a cylinder lateral face meets an OBLIQUE plane — ONLY IF such a rim can
be produced and reached by a native (OCCT-free) body; otherwise the elliptical-crease fillet
SHALL be an HONEST DECLINE (a NULL Shape → OCCT), with NO always-NULL dead builder retained,
and the measured OCCT-fallback gap SHALL be REPORTED. In everything outside the landed slice
the engine defers to the OCCT `BRepFilletAPI_MakeFillet` oracle.

IMPLEMENTATION OUTCOME (this change): T2 is an HONEST DECLINE. Reaching a native
elliptical-crease fillet requires a native `topology::Shape` carrying a true `Cylinder` face
+ an OBLIQUE `Plane` face meeting at an `Ellipse` `EdgeCurve`. No OCCT-FREE constructor
produces that topology — native booleans are PLANAR-faced only, and the native SSI curved
boolean recognizes only quadric↔quadric pairs (cyl↔cyl / sphere↔sphere / cone↔cyl), NOT a
cylinder cut by an oblique half-space. An oblique cut is therefore OCCT-built, so the body is
never a `NativeShape` and the elliptical path is UNREACHABLE natively; a builder would be
untestable/unreachable DEAD CODE (forbidden by the self-verify discipline). The track is a
documented OCCT-fallthrough (`NativeEngine::fillet_edges`), gap REPORTED (OCCT ref: `Rc=5`,
`H=10`, 60°-oblique, `r=1` → filleted `383.454285`, Δ `−9.244796` vs `MakeFillet`). The
following scenarios describe the geometry that WOULD apply were the native path reachable;
they hold conditionally ("ONLY IF ... landed") and do not assert a landed native builder.

The native slice is an `Ellipse` `EdgeCurve` shared by exactly two faces — one
`FaceSurface`-kind-`Cylinder` lateral face (radius `Rc`, axis `A`) and one
`FaceSurface`-kind-`Plane` face whose normal makes an angle `θ` with `A`, `0 < θ < 90°`
(bounded away from both limits by a scale tolerance) — meeting in a CONVEX dihedral. The
crease ellipse (semi-minor `Rc`, semi-major `Rc/sinθ`) SHALL be obtained in CLOSED FORM from
the native SSI Stage-S1 `plane_conics` handler (`intersectPlaneCylinder`). For radius `r` the
builder SHALL compute, in closed form: the ball-centre SPINE as the plane∩cylinder ellipse
`(plane shifted `r` along its outward normal) ∩ (cylinder radius `Rc − r`)`; the
CYLINDER-CONTACT ellipse on the cylinder (radius `Rc`); and the PLANE-CONTACT ellipse on the
plane (spine `− r·n_out`). The fillet surface SHALL be the GENERAL CANAL — the envelope of
the constant-`r` sphere family centred on the spine ellipse — realized as swept `r`-CIRCLES
in the planes NORMAL to the spine tangent, tiled into deflection-bounded planar TRIANGLE
facets and trimmed between the two contact ellipses, welded watertight through
`assembleSolid`.

The builder SHALL SELF-VERIFY G1 tangency — the canal surface normal at each
cylinder-contact station equals the cylinder radial normal (`cos = 1`) and at each
plane-contact station equals the plane normal (`cos = 1`) — and that both contact ellipses
lie on their faces (radius `Rc`; on the plane). Asserting C0 (a bevel) here would be WRONG —
a fillet is G1-tangent. This fillet REMOVES material (a convex crease): its enclosed volume
SHALL be strictly LESS than the input's within the tessellation deflection bound, accepted
ONLY by the engine self-verify's SHRINK branch (`0 < Vr < Vo`); else DISCARDED → OCCT. The
builder SHALL remain OCCT-free and host-buildable (closed-form ellipse conics + swept
circles, NO NUMSCI) and SHALL reference no OCCT / `IEngine` / `EngineShape` type. The slice
is gated by `r < ρ_min = Rc·sinθ` (the crease's tightest curvature radius, else the canal
self-intersects on the concave side); a non-oblique plane (axis-perpendicular circle /
axis-parallel lines), a non-elliptical crease, a CONCAVE elliptical rim, a freeform adjacent
face, `r ≥ Rc·sinθ`, or a multi-edge selection SHALL return a NULL Shape → OCCT, and the
measured gap SHALL be REPORTED.

#### Scenario: Cylinder-oblique-plane elliptical rim fillet inserts a general canal and is watertight (host)
- GIVEN a native solid whose cylinder lateral face (radius `Rc`, axis `A`) is cut by an OBLIQUE plane (angle `θ`, `0 < θ < 90°`), meeting at an `Ellipse` rim in a CONVEX dihedral, built on the host with no OCCT, with `0 < r < Rc·sinθ`
- WHEN `cc_fillet_edges(solid, {ellipticalRim}, 1, r)` is computed and tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose inserted blend face is a GENERAL `r`-radius CANAL meeting the cylinder along the cylinder-contact ellipse (radius `Rc`) and the plane along the plane-contact ellipse, AND its enclosed volume SHALL be strictly LESS than the input's within the tessellation deflection bound

#### Scenario: The elliptical fillet is G1-tangent to both faces, NOT a bevel (host)
- GIVEN a native cylinder↔oblique-plane elliptical rim filleted natively with radius `r < Rc·sinθ`
- WHEN the general-canal builder computes the swept surface and its two contact ellipses
- THEN the canal surface normal SHALL equal the cylinder radial normal (`cos = 1`) at every cylinder-contact station AND the plane normal (`cos = 1`) at every plane-contact station, proving the fillet is G1-tangent (a rounded blend, NOT a C0 chamfer bevel)

#### Scenario: A radius at or beyond the crease curvature bound defers to OCCT (host)
- GIVEN a native cylinder↔oblique-plane elliptical rim with `r ≥ ρ_min = Rc·sinθ` (the canal would self-intersect on the concave side of the semi-major vertices), built on the host
- WHEN the elliptical-crease fillet builder runs
- THEN it SHALL return a NULL Shape (no patch assembled) AND the operation SHALL defer to the OCCT `BRepFilletAPI_MakeFillet` oracle — it SHALL NOT emit a self-intersecting or fabricated canal, and the measured gap SHALL be REPORTED

#### Scenario: Native elliptical-crease fillet matches the OCCT MakeFillet oracle (parity)
- GIVEN a native cylinder↔oblique-plane elliptical rim filleted by radius `r` on a booted iOS simulator, with `r < Rc·sinθ`
- WHEN `cc_fillet_edges` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`, building `BRepFilletAPI_MakeFillet`)
- THEN the two shapes' mass properties, bounding box, watertightness, and the presence of a G1 canal blend face SHALL agree within the tessellation deflection bound, the native canal SHALL be G1 to the cylinder and the plane at its two contact ellipses, AND the native result's volume SHALL be LESS than the input's; a fixture whose measured gap exceeds the bound SHALL be declared out of slice (NULL → OCCT) with the gap REPORTED

### Requirement: Native cylinder-cylinder canal fillet or honest decline (`cc_fillet_edges`)

The native blend library SHALL compute `cc_fillet_edges(body, edgeIds, edgeCount, radius)`
NATIVELY for the narrowest ROBUST cylinder↔cylinder slice — a CONSTANT-radius rolling-ball
fillet on the CURVED↔CURVED crease between two EQUAL-radius PERPENDICULAR cylinders (a
symmetric Steinmetz crease) — ONLY IF it can be built watertight, G1, and volume-correct;
otherwise the cylinder↔cylinder fillet SHALL be an HONEST DECLINE (a NULL Shape → OCCT) with
NO always-NULL dead builder retained, and the measured OCCT-fallback gap SHALL be REPORTED.

When the slice is landed, the crease curve and the ball-centre SPINE SHALL be obtained from
the native SSI marching tracer (`trace_intersection`, `CYBERCAD_HAS_NUMSCI`-gated): the
crease from `cyl1 ∩ cyl2` and the spine from `(offset-cyl1 radius Rc − r) ∩ (offset-cyl2
radius Rc − r)`, each a transversal `Closed` `WLine`. The fillet surface SHALL be the GENERAL
CANAL — swept `r`-CIRCLES in the planes NORMAL to the fitted spine tangent — trimmed between
the two cylinder-contact curves and welded watertight through `assembleSolid`, including the
saddle sub-regions. The builder SHALL SELF-VERIFY G1 tangency (the canal normal at each
cylinder-contact station equals that cylinder's radial normal, `cos = 1`), watertightness,
and the SHRINK volume branch (a convex crease REMOVES material). When the crease is NOT the
robust slice (unequal radii, non-orthogonal axes, a branched / self-intersecting crease), when
NUMSCI is unavailable, when a marching trace does not close, or when the swept canal cannot be
welded watertight (e.g. at the saddle points), the builder SHALL return a NULL Shape → OCCT.
The builder SHALL remain OCCT-free and SHALL reference no OCCT / `IEngine` / `EngineShape`
type; the OCCT `BRepFilletAPI_MakeFillet` is the verification ORACLE only. The change SHALL
NOT retain an always-NULL builder purely to signal the track — an untractable T3 is a
documented OCCT-fallthrough with the gap REPORTED, not dead code.

#### Scenario: A robust equal-radius orthogonal cyl-cyl fillet builds natively and is watertight (host, NUMSCI-ON)
- GIVEN two EQUAL-radius PERPENDICULAR cylinders (radius `Rc`) meeting at a symmetric Steinmetz crease, with `r` safely below the crease minimum curvature radius, built on the host with NUMSCI ON
- WHEN `cc_fillet_edges(solid, {cylCylCrease}, 1, r)` is computed and the native slice is landed
- THEN the result SHALL be a watertight `Solid` whose inserted blend face is a GENERAL `r`-radius CANAL meeting BOTH cylinders along their contact curves, G1 to each (`cos = 1`), AND its enclosed volume SHALL be strictly LESS than the input's within the tessellation deflection bound

#### Scenario: An untractable cyl-cyl fillet declines honestly to OCCT (host + parity)
- GIVEN a cylinder↔cylinder fillet OUTSIDE the robust slice (unequal radii, non-orthogonal axes, a branched crease, or a saddle that cannot be welded watertight), OR a build with NUMSCI unavailable, with the native engine active
- WHEN `cc_fillet_edges` is invoked
- THEN the native builder SHALL return a NULL Shape (or the track SHALL be a documented OCCT-fallthrough with no dead builder) AND the operation SHALL defer to the OCCT `BRepFilletAPI_MakeFillet` oracle, identical to `cc_set_engine(0)`, with the measured gap REPORTED — never a faked or self-intersecting canal

### Requirement: Off-the-circle fillet/chamfer seams and continuity are self-verified before assembly

The native builder SHALL, before assembling any OFF-THE-CIRCLE blend (the asymmetric chamfer,
the elliptical fillet, or the cyl↔cyl fillet), compute its trim seams / contact curves in the
stated closed form (T1: setback circles; T2: plane∩cylinder ellipses; T3: SSI marching
curves) and SHALL SELF-VERIFY, at a set of stations covering the full crease, that each seam
lies on BOTH the blend surface AND its neighbour original surface within a tolerance derived
from the operands' scale, AND that the CONTINUITY is correct for the blend type — a CHAMFER is
**C0, NOT G1** (the frustum normal makes the per-seam chamfer angle, `cos ≠ 1`), a FILLET is
**G1, NOT C0** (the canal normal equals the neighbour surface normal, `cos = 1`). If ANY
station fails the on-both-surfaces or continuity check, OR the slice preconditions do not hold
(`Rc ≤ d2`; `r ≥ Rc·sinθ`; a non-robust cyl↔cyl crease; a non-closing marching trace), the
builder SHALL return a NULL Shape and the operation SHALL defer to OCCT. The builder SHALL
NEVER emit an unverified seam, weaken a tolerance to pass, assert G1 for a chamfer or C0 for a
fillet, or fabricate a curved patch. This is a #6 instance of the roadmap's mandatory
self-verify → OCCT-fallback discipline.

#### Scenario: The asymmetric chamfer seams lie on the frustum and their surfaces at the two bevel angles (host)
- GIVEN a native cylinder top rim with `Rc − d2 > 0`, built on the host with no OCCT
- WHEN the asymmetric chamfer builder computes the two seam circles and the bevel
- THEN every cylinder-seam station SHALL lie on both the oblique frustum and the cylinder (`radius = Rc`) AND every cap-seam station on both the frustum and the cap (`axial = H`), AND the frustum SHALL be C0 at the per-seam angles (`cos = d1/√(d1²+d2²)` / `d2/√(d1²+d2²)`, neither `1`), BEFORE the patch is assembled

#### Scenario: The elliptical / cyl-cyl fillet contact curves lie on the canal and their surfaces at G1 (host)
- GIVEN a native cylinder↔oblique-plane elliptical rim (or the robust cyl↔cyl crease under NUMSCI), built on the host
- WHEN the general-canal fillet builder computes the spine and the two contact curves at their stations
- THEN every cylinder-contact station SHALL lie on both the canal and the cylinder (`radius = Rc`) AND every plane-/second-cylinder-contact station on both the canal and its neighbour surface, AND the canal SHALL be G1 (`cos = 1`) to each face at its contact curve, BEFORE the patch is assembled

#### Scenario: A degenerate off-the-circle blend that fails its precondition defers to OCCT (host)
- GIVEN a configuration whose `Rc ≤ d2` (asymmetric chamfer cap circle collapses), OR whose `r ≥ Rc·sinθ` (elliptical canal self-intersects), OR a non-robust / non-closing cyl↔cyl crease, built on the host
- WHEN the corresponding off-the-circle builder runs
- THEN it SHALL return a NULL Shape (no patch assembled) AND the operation SHALL defer to the OCCT `BRepFilletAPI` oracle — it SHALL NOT return the unverified seam or a fabricated patch, and the measured gap SHALL be REPORTED

## MODIFIED Requirements

### Requirement: Curved-face, concave, variable-radius, face-fillet, and multi-edge cases fall through to OCCT

The native blend builders SHALL DECLINE (return a NULL `Shape`) — and `NativeEngine` SHALL fall
through to OCCT — for any case outside the tractable slice: (1) any face touching the
operation is CURVED in a way no native curved slice handles (`FaceSurface::kind != Plane` and
not one of the supported cylinder↔cap fillet/chamfer rims, the cylinder↔oblique-plane ellipse
fillet rim, or the robust cylinder↔cylinder fillet crease — sphere / cone / NURBS); (2) a
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
OCCT. For `cc_fillet_edges` (constant radius), the native path handles the CONVEX + CONCAVE
circular cylinder↔cap rim (the torus canal), the CONVEX cylinder↔OBLIQUE-plane ELLIPSE rim
(the general `r`-circle canal, `0 < θ < 90°`, `r < Rc·sinθ`), AND — only when robustly
buildable — the EQUAL-radius PERPENDICULAR cylinder↔cylinder crease (the general canal on the
SSI marching spine); every OTHER constant-radius case — a non-oblique / non-elliptical /
concave elliptical crease, `r ≥ Rc·sinθ`, an unequal-radius or non-orthogonal or branched
cyl↔cyl crease, a cone↔plane / sphere / spline crease, or a freeform face — SHALL fall through
to OCCT. For `cc_chamfer_edges` (chamfer), the native path handles the CONVEX PLANAR-dihedral
edge AND the SYMMETRIC-distance CONVEX circular cylinder↔cap rim (the cone-frustum bevel,
`Rc − d > 0`); for `cc_chamfer_edges_asym` (two-distance chamfer) the native path handles the
ASYMMETRIC-distance CONVEX circular cylinder↔cap rim (the oblique cone-frustum bevel,
`Rc − d2 > 0`); every OTHER chamfer case — a CONCAVE circular rim, a cylinder↔cylinder
(curved↔curved) rim, a non-circular / tilted / non-coaxial / freeform crease, `Rc ≤ d2`, or a
wall shorter than the axial setback — SHALL fall through to OCCT. `cc_fillet_face` (face
fillet) SHALL always fall through to OCCT (not implemented natively). Each fall-through case
SHALL produce EXACTLY the fallback (OCCT) engine's result. The change SHALL NOT fake, stub-out,
or partially implement any deferred case; each SHALL be labelled and verified as a
fall-through, never faked.

#### Scenario: A curved-face input falls through (host + parity)
- GIVEN a fillet / chamfer / offset / shell where a face touching the operation is curved and no native curved slice handles it (e.g. a cone body, or a sphere↔plane rim), with the native engine active (`cc_set_engine(1)`)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return a NULL `Shape` AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A concave edge with no native slice falls through (host)
- GIVEN a `cc_fillet_edges` or `cc_chamfer_edges` / `cc_chamfer_edges_asym` on a CONCAVE (reflex-dihedral) edge that no native concave slice handles (including a concave circular chamfer rim or a concave elliptical fillet rim), with the native engine active
- WHEN the op is invoked
- THEN the native builder SHALL return a NULL `Shape` (rather than emit a wrong / self-intersecting solid) AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: An out-of-slice variable-radius, non-circular-fillet, asymmetric-chamfer, or face-fillet call falls through (host + parity)
- GIVEN a `cc_fillet_edges_variable` call OUTSIDE the linear-law convex circular slice, OR a `cc_fillet_edges` call OUTSIDE the supported constant-radius slices (a non-oblique / non-elliptical / concave elliptical crease, `r ≥ Rc·sinθ`, an unequal-radius / non-orthogonal / branched cyl↔cyl crease, a cone↔plane / sphere / spline crease), OR a `cc_chamfer_edges` / `cc_chamfer_edges_asym` call OUTSIDE the supported chamfer slices (a concave rim, a cyl↔cyl chamfer, a non-circular curved crease, `Rc ≤ d2`), OR a `cc_fillet_face` call, with the native engine active
- WHEN the op is invoked
- THEN `NativeEngine` SHALL fall through to OCCT for that call (these are not implemented natively in this change), identical to `cc_set_engine(0)`

#### Scenario: A multi-edge-interference selection falls through (host)
- GIVEN a `cc_fillet_edges` / `cc_chamfer_edges` / `cc_chamfer_edges_asym` selection of two or more edges whose blends would overlap at a shared corner, with the native engine active
- WHEN the op is invoked
- THEN the native builder SHALL return a NULL `Shape` AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: A non-native (foreign) body falls through (host)
- GIVEN a fillet / chamfer / offset / shell where `body` is not a native body (a foreign / OCCT-built shape id), with the native engine active
- WHEN the op is invoked
- THEN `NativeEngine` SHALL fall through to the fallback engine for that call, identical to `cc_set_engine(0)`

### Requirement: NativeEngine computes the blend natively, else falls through

`NativeEngine` SHALL override `chamfer_edges`, `chamfer_edges_asym`, `fillet_edges`,
`fillet_edges_variable`, `offset_face`, and `shell` to run the corresponding native builder in
`src/native/blend/` when `body` is a native body and the case is in the tractable slice, and
apply the mandatory self-verify guard, type-erasing a verified native `topology::Shape` into a
tracked native `EngineShape`. For `chamfer_edges` (symmetric, UNCHANGED) the override SHALL
first call the PLANAR `nblend::chamfer_edges` and, if NULL / unverified, the CURVED
`nblend::curved_chamfer_edge`. For `chamfer_edges_asym` the override SHALL call
`nblend::curved_chamfer_edge_asym(body, edgeIds, edgeCount, distance1, distance2)`. For
`fillet_edges` the override SHALL try, in order, the PLANAR `nblend::fillet_edges`, the CURVED
circular `nblend::curved_fillet_edge` (convex + concave), and — WHEN a robust native slice is
landed — the ELLIPTICAL (T2) and cyl↔cyl (T3) canal builders. In THIS change T2 and T3 are
HONEST DECLINES (no native builder is added, since neither is reachable/robustly buildable
OCCT-free), so `fillet_edges` ends at the circular candidates and returns an honest error →
OCCT for any off-the-circle rim, the measured gap REPORTED. Each candidate SHALL be accepted
ONLY through
`blendResultVerified(result, body, wantGrow=false)` (watertight + `0 < Vr < Vo`), since a
chamfer and a convex fillet REMOVE material — the SAME SHRINK branch. When `body` is not
native, when all native builders return a NULL `Shape` (an out-of-slice DECLINE), or when the
self-verify guard rejects the candidate, the override SHALL fall through to the held fallback
engine with **no native interception** for a foreign body, and for a native body it cannot
forward (OCCT would misread the native void) SHALL return an honest error so the call is served
by the OCCT engine — exactly how `fillet_edges` treats an unbuildable native rim. `fillet_face`
SHALL remain a pure fall-through (no native builder call). OCCT SHALL be referenced ONLY under
`CYBERCAD_HAS_OCCT` (in the fallback wiring); the native builders SHALL reference no OCCT /
`IEngine` / `EngineShape` type. `cc_fillet_edges` / `cc_chamfer_edges` signatures and POD
layouts SHALL NOT change (the new `cc_chamfer_edges_asym` is ADDITIVE), and the default engine
SHALL remain OCCT (opt-in via `cc_set_engine(1)`).

#### Scenario: A supported planar, circular, or off-the-circle blend is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a native solid with a supported selection (a convex planar-dihedral chamfer / fillet edge / planar face / box shell / convex or concave circular cylinder↔plane fillet rim / a linear-law convex circular variable fillet rim / a symmetric or ASYMMETRIC convex circular cylinder↔cap chamfer rim / a convex cylinder↔oblique-plane ELLIPSE fillet rim / a robust equal-radius orthogonal cyl↔cyl fillet crease)
- WHEN `cc_chamfer_edges` / `cc_chamfer_edges_asym` / `cc_fillet_edges` / `cc_fillet_edges_variable` / `cc_offset_face` / `cc_shell` is invoked for the supported case
- THEN the shape SHALL be built by `src/native/blend/` and PASS its correctly-signed self-verify with no fallback call AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: An unsupported or unverified blend falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred case (curved face with no native slice, concave edge with no native slice, out-of-slice variable radius, out-of-slice non-circular fillet, out-of-slice or asymmetric chamfer, untractable cyl↔cyl fillet, face fillet, multi-edge interference, foreign body) OR a candidate that fails the self-verify
- WHEN the `cc_*` op is invoked
- THEN the native builder(s) SHALL return NULL (or the guard SHALL reject, or the method is a pure fall-through) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`) for a foreign body, or an honest error served by the OCCT engine for a native body — proving fall-through with no native interception

### Requirement: Blend parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME `cc_chamfer_edges`
/ `cc_chamfer_edges_asym` / `cc_fillet_edges` / `cc_fillet_edges_variable` / `cc_offset_face` /
`cc_shell` calls SHALL be issued once with the native engine active (`cc_set_engine(1)`) and once
with the OCCT default (`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through
the `cc_*` facade — mass properties, bounding box, sub-shape counts, and watertight tessellation —
against the OCCT `BRepFilletAPI` / `BRepOffsetAPI` oracle. On **box** chamfer / offset / shell the
native result SHALL match the oracle EXACTLY (volume / bbox / centroid relative error ~0, fp
precision); the constant-radius fillet, the linear-law variable fillet, the SYMMETRIC curved
chamfer, AND the ASYMMETRIC two-distance chamfer SHALL match within the tessellation deflection
bound (the asymmetric chamfer compared against OCCT `BRepFilletAPI_MakeChamfer` with
`Add(d1, d2, edge, face)`; because an oblique frustum is EXACT, that bound is TIGHT — the angular
deflection). The NON-CIRCULAR (elliptical) fillet and, when landed, the cyl↔cyl fillet SHALL match
their OCCT `BRepFilletAPI_MakeFillet` oracle within the curved-parity deflection bound; a fixture
whose measured gap exceeds the bound SHALL be declared out of slice (NULL → OCCT) with the gap
REPORTED. The fall-through cases (a curved-face input, a concave edge with no native slice, an
OUT-OF-SLICE variable-radius / non-circular fillet / asymmetric chamfer call, an untractable
cyl↔cyl fillet, a `cc_fillet_face` call, and a multi-edge-interference selection) SHALL be asserted
identical under both engines (fall-through proof, `cc_active_engine() == 1`). The parity test SHALL
restore the OCCT default in teardown and SHALL carry its own `main()` (on the `run-sim-suite.sh`
SKIP list) so the suite count is unchanged.

#### Scenario: Native box blends match the OCCT BRepFilletAPI / BRepOffsetAPI oracle (parity)
- GIVEN box chamfer / offset / shell cases and a box convex-edge fillet on a booted iOS simulator
- WHEN each `cc_*` op is called with the native engine active and with the OCCT default
- THEN the chamfer / offset / shell shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT oracle, and the fillet SHALL agree within the tessellation deflection bound

#### Scenario: Native asymmetric chamfer and non-circular fillet match their OCCT oracles (parity)
- GIVEN a native cylinder top rim chamfered by two distances `d1 ≠ d2`, a native cylinder↔oblique-plane elliptical fillet rim, and (when landed) a native equal-radius orthogonal cyl↔cyl fillet crease on a booted iOS simulator
- WHEN each `cc_*` op is called with the native engine active and with the OCCT default (the asymmetric chamfer built by `BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)`; the fillets by `BRepFilletAPI_MakeFillet`)
- THEN the two shapes' mass properties, bounding box, watertightness, and blend-face presence SHALL agree within the deflection bound (TIGHT for the exact oblique frustum; curved-parity for the fillet canals), the asymmetric bevel SHALL be C0 at its two angles and the fillet canals G1 at their contact curves, AND each native result's volume SHALL be LESS than the input's (material removed); a fixture beyond tol SHALL be declared out of slice (NULL → OCCT) with the gap REPORTED

#### Scenario: Fall-through blend cases are identical under both engines (parity)
- GIVEN a curved-face blend, a concave-edge blend with no native slice, an out-of-slice variable-radius / non-circular fillet / asymmetric chamfer call, an untractable cyl↔cyl fillet, a `cc_fillet_face` call, and a multi-edge-interference selection on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change
