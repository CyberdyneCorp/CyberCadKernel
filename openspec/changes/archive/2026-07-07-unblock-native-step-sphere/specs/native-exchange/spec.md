# native-exchange

This change **unblocks native STEP import of a full SPHERE** by closing the ONE remaining topology gap
the archived `add-native-step-revolution-quadrics` slice flagged: the reader already maps the sphere
SURFACE (a `SPHERICAL_SURFACE` keyword AND an on-axis-circle `SURFACE_OF_REVOLUTION` both reduce to
native `FaceSurface::Kind::Sphere`, host-verified), but an OCCT full sphere is a SINGLE spherical FACE
that is simultaneously **u-PERIODIC** (the longitude `0/2π` seam — already dropped) and
**DOUBLE-POLE-DEGENERATE** (at each pole `v = ±π/2` a whole parametric `u`-edge collapses to a single
point — a zero-length / on-axis DEGENERATE edge the reader does NOT yet handle). The surviving pole
edges leave the face wire unclosed → the watertight self-verify fails → honest DECLINE → OCCT.

The fix is **reader / topology-only**: recognise the degenerate pole edges (drop them alongside the
seam) and, when a spherical face is bounded by NOTHING ELSE (a genuine FULL sphere), build the native
`Sphere` face as a **BARE periodic surface** (a Face with a NULL outer wire — `makeFace` already
accepts one) so it routes to the tessellator's **no-boundary structured grid over the sphere's natural
parametric bounds** (`u ∈ [0,2π], v ∈ [−π/2,π/2]`), which ALREADY meshes an analytic sphere watertight
(pole rows fan to the collapsed pole point, `u = 0/2π` columns weld at the seam) — the same path every
native-constructed sphere uses. No tessellator change; no writer change; no new topology primitive; no
`cc_*` ABI change; the default engine stays OCCT.

> NOTE (honest scope + honest-out): this change accepts a **full periodic double-pole SPHERE**
> `ADVANCED_FACE` (a `SPHERICAL_SURFACE` keyword OR an on-axis-circle `SURFACE_OF_REVOLUTION`). If the
> pole-degenerate sphere face cannot be built ROBUSTLY watertight — a **partial / pole-capped**
> spherical zone with a surviving real latitude trim, a surface that does not reduce to `Sphere`,
> parametric coverage that is not the full turn pole-to-pole, or any assembled sphere solid that fails
> the engine `robustlyWatertightImport` self-verify — the reader **KEEPS the current honest OCCT
> deferral** (the sphere already imports correctly via OCCT). A correct still-deferred outcome is
> acceptable and reported plainly. Still declined → OCCT `STEPControl_Reader`: a `TOROIDAL_SURFACE`, an
> off-axis-circle revolution (torus), an ellipse / B-spline revolution (general revolved surface), a
> skew oblique line (hyperboloid), a directly-authored arbitrary rational (weighted) B-spline surface,
> and every general swept / bounded / offset surface. IGES import/export stay OCCT. #8 `drop-occt` stays
> blocked; this change does NOT unblock it.
> **No non-watertight or wrong sphere is ever fabricated; the imported sphere is the EXACT analytic
> sphere the file describes, merely re-tiled onto its natural parametric mesh; no tolerance is
> weakened.**

## MODIFIED Requirements

### Requirement: Native STEP AP203 import of a writer-emitted manifold-solid-brep

The native exchange library SHALL provide `step_import_native(path)` that reads an
ISO-10303-21 (STEP Part 21) file — **independently of its `FILE_SCHEMA` header** (AP203,
AP214 `AUTOMOTIVE_DESIGN`, or AP242 are all accepted; the reader gates on entities + the mm
length-unit context, not the schema string, and **skips** AP242 PMI / annotation entities and
additive plane-angle / solid-angle / PMI unit contexts) — and reconstructs a native
`topology::Shape`: a `Solid` when the file has exactly one root `MANIFOLD_SOLID_BREP`; a **flat**
`Compound` of `Solid`s when it has more than one co-equal root `MANIFOLD_SOLID_BREP` with **no**
product-placement transform tree; or a **placed** `Compound` of `Solid`s when the file is a
**single-level assembly** — each component `MANIFOLD_SOLID_BREP` reconstructed at its
component-local coordinates then placed by the composed transform the file's transform tree
carries, where the composed per-component placement is classified as one of **rigid** (rotation +
translation), **uniform scale** (`R·kI`, one positive factor `k>0`; the placed solid's volume
scales by `k³`), or **mirror** (an orthonormal reflection, det ≈ −k³ < 0, optionally uniformly
scaled) — a **non-uniform / shear** placement (a linear part whose `MᵀM` is not a scalar multiple
of the identity) SHALL DECLINE. For a **mirror** placement the reader SHALL compensate the
reflection's handedness flip by complementing the component solid's face orientation (the existing
`topology::Orientation` algebra) so the tessellator's tangent-derived world normal
(`cross(place(∂u), place(∂v))`, which reverses under a reflection) points OUTWARD and the placed
solid self-verifies watertight with positive volume — the tessellator SHALL NOT be modified and no
normal SHALL be fabricated. The faces SHALL carry surfaces of kind `PLANE`,
`CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS`
(non-rational), or a **`SURFACE_OF_REVOLUTION` that reduces to a native analytic quadric** — a
straight generatrix **parallel** to its axis (→ cylinder), a straight generatrix **oblique** to and
**meeting** its axis (→ cone), a straight generatrix **perpendicular** to its axis (→ plane), or an
**on-axis circle / arc** whose plane contains the axis (→ sphere) — and the edges curves of kind
`LINE`, `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (non-rational), or a **`TRIMMED_CURVE`** whose
basis is one of those kinds. A `SPHERICAL_SURFACE` (or on-axis-circle-`SURFACE_OF_REVOLUTION`) face that
OCCT emits as a **SINGLE periodic FACE with a longitude SEAM and TWO DEGENERATE POLE edges** — a full
sphere — SHALL be reconstructed as a native `Sphere` **bare periodic surface** (a face whose outer wire
is NULL) by dropping the seam pair AND the two degenerate pole edges, so the tessellator meshes it
watertight over the sphere's natural parametric bounds; a spherical face that still carries a REAL
latitude-trim edge after the seam+pole drop keeps the ordinary trimmed-wire path. Specifically:

- **A `TRIMMED_CURVE`** `('',#basis,(trim_1),(trim_2),sense_agreement,master_representation)` SHALL
  be mapped by resolving its **basis** curve (recursively — a `LINE` / `CIRCLE` / `ELLIPSE` /
  `B_SPLINE_CURVE_WITH_KNOTS`, including a basis reached through the existing `SURFACE_CURVE` /
  `SEAM_CURVE` / `INTERSECTION_CURVE` wrapper) as the native `EdgeCurve`, and caching its two
  `PARAMETER_VALUE` trims (if present) keyed by the `TRIMMED_CURVE`'s `#id`. When the basis is a
  **`B_SPLINE_CURVE_WITH_KNOTS`** and both parameter trims are present, the native `Edge`'s
  `[first,last]` range SHALL be taken from those trims (min/max, clamped to the clamped knot span; a
  wide / degenerate span reduces to the full curve) — the covered knot sub-domain the endpoint
  vertices cannot recover. When the basis is **analytic** (`LINE` / `CIRCLE` / `ELLIPSE`) the reader
  SHALL keep the existing vertex-derived range (the endpoint vertices fix the range exactly and the
  parameter trims are redundant). A `TRIMMED_CURVE` whose basis is out of slice (rational /
  unsupported curve) or absent / malformed SHALL DECLINE. No new topology is added — the native
  `Edge` already stores an arbitrary trimmed `[first,last]` range.
- **A DEGENERATE POLE edge** — an `EDGE_CURVE` (or a `VERTEX_LOOP` bound) whose 3D curve collapses to a
  SINGLE point over its whole parameter range (both endpoint `VERTEX_POINT`s coincide within a
  scale-relative tolerance AND the curve sweeps zero arc length / lies on the revolution axis) — is the
  pole singularity of a periodic surface, NOT a real trimming edge, and SHALL be DROPPED from the
  `EDGE_LOOP` alongside the periodic seam. A legitimate full-circle rim edge (`v0 == v1` with a
  NON-zero circular sweep — a cylinder / cone cap rim) SHALL NOT be dropped.
- **A `SURFACE_OF_REVOLUTION`** `('',#profile,#axis1)` SHALL be mapped by resolving the axis
  (`AXIS1_PLACEMENT('',#origin,#axis)` — origin + one direction, `$` axis defaulting to +Z) and the
  **profile** curve (via the same curve dispatcher, including a `TRIMMED_CURVE` profile), then
  classifying the profile + axis by MEASUREMENT (never by trusting a keyword) and mapping it to an
  EXACT native analytic quadric in FOUR cases, each built with the existing analytic `FaceSurface`
  machinery so the reduced surface is identical to the analytic-keyword-equivalent surface AND
  VERIFIED to pass through the profile within a scale-relative tolerance before it is emitted:
  - a straight `LINE` generatrix **parallel** to the axis → a native **`Cylinder`** (radius = the
    perpendicular distance from the line to the axis, frame on the axis);
  - a straight `LINE` generatrix **oblique** to the axis whose support is **coplanar with and
    intersects** the axis → a native **`Cone`** (apex at the intersection, `semiAngle` = the line-axis
    angle folded into `(0, π/2)`, reference radius = the perpendicular distance at the frame origin, a
    regular on-axis point NOT the apex, per the native `S(u,v)=O+(R+v·sinα)(cos u·X+sin u·Y)+v·cosα·Z`
    convention);
  - a straight `LINE` generatrix **perpendicular** to the axis → a native **`Plane`** (a flat annulus
    through the line, normal = the axis direction, frame at the foot on the axis);
  - a `CIRCLE` / arc generatrix whose **centre lies ON the axis** AND whose **plane contains the axis
    direction** (revolved about a diameter) → a native **`Sphere`** (centre = the circle centre,
    radius = the circle radius). A full-turn on-axis-circle revolution whose reconstructed face is a
    complete periodic double-pole sphere SHALL be closed watertight via the bare-periodic-surface path
    above.

  In **every** other case the reader SHALL DECLINE (NULL → OCCT): a `CIRCLE` / arc whose **centre is
  OFF the axis** (a **torus** — there is no native `FaceSurface::Kind::Torus`), a `CIRCLE` whose plane
  does not contain the axis (non-spherical / degenerate), an **`ELLIPSE`** or
  **`B_SPLINE_CURVE_WITH_KNOTS`** generatrix (a general revolved surface — the reader authors no
  revolved-B-spline surface), a **skew** oblique `LINE` whose support does NOT meet the axis (a
  **hyperboloid of one sheet** — no native kind), a `LINE` **on** the axis (degenerate), a **degenerate
  axis**, and any reduced cone / plane / sphere face that fails the faithful-reduction guard — kept a
  DECLINE consistent with the `TOROIDAL_SURFACE` decline.

The reader SHALL (a) tokenize the DATA section into a `map<#id, Record>` handling integer refs `#M`,
reals including typed forms (`1.`, `1.E2`, `-3.5E-07`), strings (`'...'` with embedded `''`), enums
(`.T.` / `.PLANE.`), lists `( ... )`, `$` (null), `*` (derived), and combined-instance
`( SUB(...) SUB(...) )` records; (b) resolve leaf geometry — `CARTESIAN_POINT` → `math::Point3` in
**millimetres**, `DIRECTION` → `math::Dir3`, `AXIS2_PLACEMENT_3D` → `math::Ax3`, `AXIS1_PLACEMENT` →
axis (origin + direction), the in-scope curves (including a `TRIMMED_CURVE`'s basis) → `EdgeCurve`,
the in-scope surfaces (including a quadric-reducing `SURFACE_OF_REVOLUTION` → cylinder / cone / plane /
sphere) → `FaceSurface`; (c) build topology following refs — `VERTEX_POINT` → vertex, `EDGE_CURVE` →
one shared edge per `#id` (its `[first,last]` from the trims when the 3D curve is a `TRIMMED_CURVE`
over a B-spline basis), `ORIENTED_EDGE` → the oriented shared edge, `EDGE_LOOP` → wire,
`FACE_OUTER_BOUND` / `FACE_BOUND` + `ADVANCED_FACE` sense → face, `CLOSED_SHELL` /
`MANIFOLD_SOLID_BREP` → shell/solid (all roots when there are several) — **dropping the writer's
periodic-wall SEAM edge AND the two degenerate POLE edges of a full periodic sphere face** (a full
sphere face whose loop reduced to seam + poles only → a native `Sphere` bare periodic surface meshed
over its natural bounds); and (d) **when a product-placement transform tree is present**, compose it
exactly as the archived assembly slices do (rigid / uniform-scale / mirror, else DECLINE), applying the
mirror orientation compensation where needed, then `Shape::located(Location{T})` per component solid. A
`TOROIDAL_SURFACE` face, a `SURFACE_OF_REVOLUTION` the reader cannot reduce to a native quadric
(cylinder / cone / plane / sphere), and a partial / pole-capped spherical zone that cannot close
watertight, SHALL DECLINE, and an assembly structure the reader cannot compose to a supported placement
for every geometric root SHALL DECLINE. This reader SHALL remain OCCT-free and host-buildable and SHALL
reference no OCCT / `IEngine` / `EngineShape` type. It SHALL NOT modify the STEP writer or the
tessellator, SHALL NOT import PMI / annotation entities as geometry, and SHALL NOT fabricate a curve, a
surface, a trim, a placement, or a solid the file does not describe.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A full periodic double-pole SPHERICAL_SURFACE face imports as a watertight native sphere (host)
- GIVEN an in-scope ISO-10303-21 buffer with a SINGLE `SPHERICAL_SURFACE` `ADVANCED_FACE` whose `EDGE_LOOP` is the longitude SEAM meridian (one `EDGE_CURVE` referenced forward AND reversed) plus TWO DEGENERATE POLE edges (each collapsing a whole `u`-edge to the pole vertex at `v = ±π/2`), read on the host with no OCCT
- WHEN `step_import_native` reconstructs the face
- THEN it SHALL drop the seam pair AND the two degenerate pole edges, build the native `Sphere` face as a BARE periodic surface (NULL outer wire), AND the assembled solid SHALL be valid + watertight with volume `4/3·π·R³` and bbox `[−R,R]³`, IDENTICAL to the `SPHERICAL_SURFACE`-keyword multi-lune / on-axis-circle-revolution sphere — never a fabricated or non-watertight face

#### Scenario: A rigid / uniform-scale / mirror assembly still imports as a placed compound (host)
- GIVEN an in-scope ISO-10303-21 buffer describing a single-level assembly of components placed by rigid, uniform-scale, or mirror `ITEM_DEFINED_TRANSFORMATION` transforms, read on the host with no OCCT
- WHEN `step_import_native` composes the transform tree
- THEN it SHALL return a `Compound` of the placed `Solid`s exactly as the archived scaled/mirrored-assembly slice does (the placed-assembly paths are unchanged by this change)

#### Scenario: A TRIMMED_CURVE edge is accepted and unwrapped onto the native trimmed edge (host)
- GIVEN an in-scope ISO-10303-21 buffer where one `EDGE_CURVE`'s 3D curve is a `TRIMMED_CURVE` over a `LINE` / `CIRCLE` / `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` basis, read on the host with no OCCT
- WHEN `step_import_native` resolves the edge
- THEN it SHALL unwrap the `TRIMMED_CURVE` to the basis curve as the native `EdgeCurve`, setting the native `Edge`'s `[first,last]` from the `PARAMETER_VALUE` trims (clamped to the clamped knot span) when the basis is a B-spline, or from the endpoint vertices when the basis is analytic, AND the assembled solid SHALL be valid + watertight (the trimmed-curve slice is unchanged by this change)

#### Scenario: A SURFACE_OF_REVOLUTION of a line parallel to its axis reduces to an exact native cylinder (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a straight `LINE` generatrix parallel to the revolution axis, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL reduce it to the EXACT native analytic `Cylinder` (radius = the perpendicular distance from the line to the axis, frame on the axis), AND the assembled solid SHALL be valid + watertight and identical to the `CYLINDRICAL_SURFACE`-keyword-equivalent solid (the landed cylinder reduction is unchanged)

#### Scenario: A SURFACE_OF_REVOLUTION of an oblique line meeting the axis reduces to an exact native cone (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a straight `LINE` generatrix OBLIQUE to the axis, whose support is coplanar with and intersects the axis (a truncated-cone / frustum wall), read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL reduce it to the EXACT native analytic `Cone` (apex at the line-axis intersection, `semiAngle` = the line-axis angle, reference radius = the perpendicular distance at the frame origin — a regular on-axis point, NOT the apex), VERIFIED to pass through the profile, AND the assembled frustum solid SHALL be valid + watertight and identical to the `CONICAL_SURFACE`-keyword-equivalent solid

#### Scenario: A SURFACE_OF_REVOLUTION of a line perpendicular to the axis reduces to an exact native plane (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a straight `LINE` generatrix PERPENDICULAR to the revolution axis (a flat annular cap), read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL reduce it to the EXACT native analytic `Plane` (a flat annulus through the line, normal = the axis direction, frame at the foot on the axis), VERIFIED that both line endpoints share one axial coordinate, AND the reconstructed face SHALL be valid + watertight and identical to the `PLANE`-keyword-equivalent face

#### Scenario: A SURFACE_OF_REVOLUTION of an on-axis circle reduces to an exact native sphere (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a `CIRCLE` / semicircular arc whose centre lies ON the axis and whose plane contains the axis direction (revolved about a diameter), read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL reduce it to the EXACT native analytic `Sphere` (centre = the circle centre, radius = the circle radius), VERIFIED that the centre is on the axis and the circle plane contains the axis, AND — when the reconstructed face is a full periodic double-pole sphere — it SHALL close it watertight via the bare-periodic-surface path so the assembled solid is valid + watertight and identical to the `SPHERICAL_SURFACE`-keyword-equivalent solid

#### Scenario: A SURFACE_OF_REVOLUTION with no faithful native quadric declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a `CIRCLE` / arc whose centre is OFF the axis (a torus), an `ELLIPSE` or `B_SPLINE_CURVE_WITH_KNOTS` generatrix (a general revolved surface), a SKEW oblique `LINE` whose support does NOT meet the axis (a hyperboloid of one sheet), or a `LINE` on the axis (degenerate), read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid — the reader authors no torus (there is no native `FaceSurface::Kind::Torus`), no revolved-B-spline surface, and no hyperboloid — so the engine can fall through to OCCT; the decline is kept consistent with the landed `TOROIDAL_SURFACE` decline and never a forced or approximate face

### Requirement: Native STEP import runs healShell and returns NULL for out-of-scope or unhealable files

`step_import_native` SHALL rely on the shared-node reconstruction and SHALL return the assembled
`Solid` / flat `Compound` / **placed `Compound`** for the engine to self-verify. A placed component
solid SHALL be reconstructed at its local coordinates then placed by `Shape::located()`: a **rigid**
or **uniform-scale (`k>0`)** placement is conformal and preserves the watertight 2-manifold; a
**mirror** placement SHALL have the component's face orientation complemented so the reflected solid
meshes with outward normals and self-verifies watertight with positive volume. A `TRIMMED_CURVE` edge
SHALL be reconstructed onto the native trimmed `Edge` (basis `EdgeCurve` + trim-driven `[first,last]`
for a B-spline basis, vertex-derived range otherwise), a **quadric-reducing**
`SURFACE_OF_REVOLUTION` face onto its exact native analytic surface — `Cylinder` (line ∥ axis),
`Cone` (line oblique, meeting the axis), `Plane` (line ⟂ axis), or `Sphere` (on-axis circle / arc) —
each VERIFIED to pass through the profile, and a **full periodic double-pole SPHERE** face (a
`SPHERICAL_SURFACE` keyword or an on-axis-circle `SURFACE_OF_REVOLUTION` whose loop reduced to the seam
+ the two degenerate poles only) onto a native `Sphere` **bare periodic surface** meshed watertight
over its natural bounds — all subject to the same watertight self-verify. The reader
SHALL return a **NULL Shape (DECLINE)** — and never a partial or invented solid — when ANY of: (i) the
assembled shell is a genuinely open / non-manifold B-rep, or a placed member fails the self-verify (an
apex-reaching cone with a degenerate collapsed seam, OR a partial / pole-capped spherical zone that
cannot close watertight, is a member that fails the self-verify → DECLINE);
(ii) the file has **zero** root `MANIFOLD_SOLID_BREP`, OR carries a product-placement transform tree
the reader **cannot compose** to a supported placement for every geometric component (a **non-uniform /
shear** transform, a root reached by no placement, or a **deep multi-level nested** /
**external-reference** product structure); (iii) a referenced entity has an unsupported keyword or a
surface kind outside
{`PLANE`,`CYLINDRICAL_SURFACE`,`CONICAL_SURFACE`,`SPHERICAL_SURFACE`,`B_SPLINE_SURFACE_WITH_KNOTS`, a
**quadric-reducing** `SURFACE_OF_REVOLUTION` (a line parallel / oblique-meeting / perpendicular, or an
on-axis circle)} — explicitly INCLUDING `TOROIDAL_SURFACE`, a `SURFACE_OF_REVOLUTION` of an
**off-axis circle** (torus), an **ellipse / B-spline** profile (general revolved surface), a **skew**
oblique line (hyperboloid), or a line **on** the axis (degenerate), a **directly-authored arbitrary
rational (weighted)** B-spline surface, and a general swept / bounded / offset surface
(`SURFACE_OF_LINEAR_EXTRUSION`, `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`),
in ANY component — or a curve kind outside {`LINE`,`CIRCLE`,`ELLIPSE`,`B_SPLINE_CURVE_WITH_KNOTS`, a
`TRIMMED_CURVE` over one of those}, a `TRIMMED_CURVE` over an out-of-slice basis, or a rational
(weighted) B-spline wrap; (iv) a non-millimetre LENGTH-unit context (no silent rescale; additive
plane-angle / solid-angle / PMI unit contexts are skipped and do NOT count as non-mm); or (v) a
malformed / dangling record. AP242 PMI / annotation entities SHALL be **skipped** (never a decline
trigger, never imported). The tolerance SHALL NEVER be widened to force a pass; the honest residual
SHALL be reported, not hidden — a full sphere that cannot be built robustly watertight KEEPS the honest
OCCT deferral rather than emitting a non-watertight or approximate sphere.

#### Scenario: A file whose B-rep cannot form a watertight solid returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap so the assembled shell is a genuinely open shell, read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and the engine self-verifies it
- THEN the result SHALL NOT self-verify watertight AND the import SHALL DECLINE (NULL) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: A partial / pole-capped spherical zone that cannot close keeps the honest OCCT deferral (host)
- GIVEN an ISO-10303-21 buffer with a spherical `ADVANCED_FACE` that is NOT a full sphere — a partial spherical zone or a pole-capped hemisphere whose `EDGE_LOOP` still carries a REAL latitude-trim edge after the seam+pole drop, and whose reconstructed face does not self-verify watertight — read on the host with no OCCT
- WHEN `step_import_native` reconstructs the face and the engine self-verifies it
- THEN the reader SHALL NOT force the bare-periodic-surface full-sphere path AND the import SHALL DECLINE (NULL) so the engine falls through to OCCT — a correct still-deferred outcome, never a non-watertight or invented sphere, with no tolerance widened

#### Scenario: A TOROIDAL_SURFACE or a non-quadric SURFACE_OF_REVOLUTION or out-of-slice surface returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE`, a `SURFACE_OF_REVOLUTION` that does not reduce to a native quadric (an off-axis circle → torus, an ellipse / B-spline profile → general revolved surface, a skew oblique line → hyperboloid, or an on-axis line → degenerate), a directly-authored arbitrary rational B-spline surface, or a general swept / bounded / offset surface — as a lone solid OR as one component of an assembly — read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid (the whole file declines — no partial import), so the engine can fall through to OCCT — no torus / hyperboloid / rational / swept surface is faked (the tessellator is not modified)

#### Scenario: An AP242 PMI entity still never triggers a decline (host)
- GIVEN an ISO-10303-21 AP242 buffer carrying an in-slice solid PLUS PMI / annotation entities and additive plane-angle / PMI unit contexts, read on the host with no OCCT
- WHEN `step_import_native` runs its unit-context gate and its assembly-trigger scan
- THEN the PMI / annotation entities SHALL be SKIPPED (they SHALL NOT fail the mm length gate and SHALL NOT force the assembly path) AND the solid SHALL import; the file SHALL NOT decline merely because AP242 PMI entities are present (unchanged by this change)

## ADDED Requirements

### Requirement: Native STEP import full periodic double-pole sphere face verified vs OCCT

The full-sphere-face unblock SHALL be verified by (a) **host** unit / decline cases (OCCT-free): a
hand-authored SINGLE `SPHERICAL_SURFACE` `ADVANCED_FACE` whose `EDGE_LOOP` is the longitude seam
(forward + reversed) plus two DEGENERATE pole edges reconstructs to a native `Sphere` solid that is
valid + watertight, volume `4/3·π·R³`, bbox `[−R,R]³`, IDENTICAL to the `SPHERICAL_SURFACE`-keyword
multi-lune / on-axis-circle-revolution sphere; the on-axis-circle `SURFACE_OF_REVOLUTION` form of the
same face also imports watertight (both reduce to `Sphere`); a partial / pole-capped spherical zone that
cannot close, and a non-sphere revolution, DECLINE (NULL); and the parallel / oblique / perpendicular
revolution reductions, the single-solid, flat multi-solid, placed rigid / uniform-scale / mirror
assembly, AP242, trimmed-curve, quadric, and bspline-face round-trip cases STILL pass. And (b) a
**simulator sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade under `cc_set_engine(1)`: a
FOREIGN OCCT-authored SPHERE solid (`BRepPrimAPI_MakeSphere`, the exact single periodic-pole-face
B-rep) imports **NATIVELY** watertight and matches the OCCT re-import (solid count / volume / area /
watertight / bbox) — flipping the previously-deferred case (native parsed=0 → OCCT) to a native import;
and a FOREIGN OCCT-authored torus / general revolution still DECLINES natively and imports via OCCT
identical to `cc_set_engine(0)`. If the pole-degenerate sphere face cannot be built robustly watertight,
the reader SHALL KEEP the honest OCCT deferral and the parity test SHALL report that plainly (never a
faked native-watertight claim). The parity test SHALL restore the OCCT default in teardown and SHALL
carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite assertion count is unchanged.
Every existing suite (`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and every prior native
capability (the STEP export slice, the flat multi-solid + ELLIPSE + bspline-face + rigid / uniform-scale
/ mirror assembly + AP242 + trimmed-curve + revolution cylinder / cone / plane / sphere-reduction import
slices, shape healing, SSI S1–S5, native blends + #6/#7, curved-boolean native-pass=13, marching,
boolean, construct, tessellation) SHALL stay green at the OCCT default with no regression.

#### Scenario: A foreign OCCT-authored full sphere imports natively watertight and matches OCCT (sim)
- GIVEN an OCCT-authored full sphere solid (`BRepPrimAPI_MakeSphere(R)`) — a SINGLE periodic spherical face with a longitude seam and two degenerate pole vertices — written to STEP, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN `step_import_native` SHALL return a valid + watertight native `Sphere` solid (raw native parse succeeds, not a fallback) whose solid COUNT, volume, surface area, watertightness, and bounding box match the OCCT re-import within tolerance, proving the full periodic double-pole sphere face now closes watertight natively

#### Scenario: A full sphere that cannot be built robustly watertight keeps the honest OCCT deferral (sim)
- GIVEN an OCCT-authored spherical B-rep the reader cannot reconstruct into a robustly watertight native sphere face, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND `cc_step_import` SHALL fall through to OCCT `STEPControl_Reader` identical to `cc_set_engine(0)`, proving an honest still-deferred outcome — never a fabricated or non-watertight native sphere

#### Scenario: A foreign OCCT-authored torus / general revolution still declines to OCCT (sim)
- GIVEN an OCCT-authored solid whose face is a `SURFACE_OF_REVOLUTION` of an off-axis circular arc (a torus) or of an ellipse / B-spline profile (a general revolved surface), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND OCCT `STEPControl_Reader` SHALL import the file identical to `cc_set_engine(0)`, proving honest fall-through with no fabricated torus / hyperboloid geometry, consistent with the `TOROIDAL_SURFACE` decline (unchanged by this change)

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, the revolution cylinder / cone / plane / sphere-reduction import slices, the multi-solid + assembly + AP242 + trimmed-curve slices, shape healing, SSI S1–S5, native blends + #6/#7, curved-boolean native-pass=13, marching, boolean, construct, and tessellation SHALL NOT regress
