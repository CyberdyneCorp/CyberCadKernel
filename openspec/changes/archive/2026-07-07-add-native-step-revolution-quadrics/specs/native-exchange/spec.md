# native-exchange

This change (Phase 4 #8) **extends the working native STEP import reader's `SURFACE_OF_REVOLUTION`
arm** (landed by `add-native-step-general-surfaces`, which reduces ONLY a straight `LINE` generatrix
**parallel** to the axis → an exact native `Cylinder`) to the other **analytic-quadric** revolution
cases, each reducing to a native `FaceSurface` kind the reader ALREADY builds for the direct analytic
keyword, mapping ONLY the sub-cases that reconstruct onto an EXACT native kind, VERIFY they pass
through the profile, AND self-verify watertight, and DECLINING the rest honestly (NULL → OCCT, exactly
like the landed `TOROIDAL_SURFACE` decline):

- **(R1) A `LINE` generatrix OBLIQUE to the axis** whose support is coplanar with and INTERSECTS the
  axis → a native **`Cone`** (apex at the intersection, half-angle = the line-axis angle, reference
  radius at the frame origin per the native cone convention). `FaceSurface::Kind::Cone` already exists
  and round-trips through the STEP writer.
- **(R2) A `LINE` generatrix PERPENDICULAR to the axis** → a native **`Plane`** (a flat annulus / disk
  face normal to the axis). `FaceSurface::Kind::Plane` already exists.
- **(R3) A `CIRCLE` / arc generatrix whose CENTRE lies ON the axis and whose plane CONTAINS the axis**
  (revolved about a diameter) → a native **`Sphere`** (radius = the circle radius).
  `FaceSurface::Kind::Sphere` already exists and round-trips through the STEP writer.

Everything with no faithful native kind stays an honest DECLINE (R4): an off-axis circle (a **torus** —
no `Kind::Torus`), an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` generatrix (a general revolved surface —
the reader authors no revolved B-spline), a **skew** oblique line whose support does NOT meet the axis
(a **hyperboloid of one sheet**), a degenerate axis / on-axis line, and any reduced face that fails the
watertight self-verify.

No `cc_*` ABI change; the default engine stays OCCT. The STEP writer (`step_writer.cpp`) and the
tessellator are NOT modified (the native `Cone` / `Plane` / `Sphere` evaluation + writer round-trip
already exist and are verified).

> NOTE (honest scope): this widening **accepts** a `SURFACE_OF_REVOLUTION` of a **line parallel** (→
> cylinder, landed), a **line oblique meeting the axis** (→ cone, R1), a **line perpendicular** (→
> plane, R2), and an **on-axis circle / arc** (→ sphere, R3); it is NOT a general-surface reader and it
> authors **no** revolved-B-spline surface. Still declined → OCCT `STEPControl_Reader`: a
> `TOROIDAL_SURFACE`; a `SURFACE_OF_REVOLUTION` of an **off-axis circle** (torus), an **ellipse /
> B-spline** profile (general revolved surface), or a **skew** oblique line (hyperboloid); a
> directly-authored **arbitrary rational (weighted) B-spline** surface / curve; a general **swept /
> bounded / offset** surface (`SURFACE_OF_LINEAR_EXTRUSION`, `RECTANGULAR_TRIMMED_SURFACE`,
> `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`, …); and everything the prior slices declined. IGES
> import/export stay OCCT `IGESControl_*`. A general native STEP/AP242 reader + IGES + a general-curved
> kernel still block #8 `drop-occt`; this change does NOT unblock it.
> **No surface or solid is ever fabricated: the reader maps only the quadric the revolved generatrix
> exactly defines, VERIFIED to pass through the profile; a `SURFACE_OF_REVOLUTION` the reader cannot
> reduce faithfully AND self-verify DECLINES rather than being forced onto a wrong or approximate
> native kind. No tolerance is weakened.**

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
basis is one of those kinds. Specifically:

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
    radius = the circle radius).

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
`MANIFOLD_SOLID_BREP` → shell/solid (all roots when there are several) — dropping the writer's
periodic-wall SEAM edge; and (d) **when a product-placement transform tree is present**, compose it
exactly as the archived assembly slices do (rigid / uniform-scale / mirror, else DECLINE), applying the
mirror orientation compensation where needed, then `Shape::located(Location{T})` per component solid. A
`TOROIDAL_SURFACE` face, and a `SURFACE_OF_REVOLUTION` the reader cannot reduce to a native quadric
(cylinder / cone / plane / sphere), SHALL DECLINE, and an assembly structure the reader cannot compose
to a supported placement for every geometric root SHALL DECLINE. This reader SHALL remain OCCT-free and
host-buildable and SHALL reference no OCCT / `IEngine` / `EngineShape` type. It SHALL NOT modify the
STEP writer or the tessellator, SHALL NOT import PMI / annotation entities as geometry, and SHALL NOT
fabricate a curve, a surface, a trim, a placement, or a solid the file does not describe.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

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
- THEN it SHALL reduce it to the EXACT native analytic `Sphere` (centre = the circle centre, radius = the circle radius), VERIFIED that the centre is on the axis and the circle plane contains the axis, AND the assembled solid SHALL be valid + watertight and identical to the `SPHERICAL_SURFACE`-keyword-equivalent solid

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
for a B-spline basis, vertex-derived range otherwise), and a **quadric-reducing**
`SURFACE_OF_REVOLUTION` face onto its exact native analytic surface — `Cylinder` (line ∥ axis),
`Cone` (line oblique, meeting the axis), `Plane` (line ⟂ axis), or `Sphere` (on-axis circle / arc) —
each VERIFIED to pass through the profile and subject to the same watertight self-verify. The reader
SHALL return a **NULL Shape (DECLINE)** — and never a partial or invented solid — when ANY of: (i) the
assembled shell is a genuinely open / non-manifold B-rep, or a placed member fails the self-verify (an
apex-reaching cone with a degenerate collapsed seam is a member that fails the self-verify → DECLINE);
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
SHALL be reported, not hidden.

#### Scenario: A file whose B-rep cannot form a watertight solid returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap so the assembled shell is a genuinely open shell, read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and the engine self-verifies it
- THEN the result SHALL NOT self-verify watertight AND the import SHALL DECLINE (NULL) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: A TOROIDAL_SURFACE or a non-quadric SURFACE_OF_REVOLUTION or out-of-slice surface returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE`, a `SURFACE_OF_REVOLUTION` that does not reduce to a native quadric (an off-axis circle → torus, an ellipse / B-spline profile → general revolved surface, a skew oblique line → hyperboloid, or an on-axis line → degenerate), a directly-authored arbitrary rational B-spline surface, or a general swept / bounded / offset surface — as a lone solid OR as one component of an assembly — read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid (the whole file declines — no partial import), so the engine can fall through to OCCT — no torus / hyperboloid / rational / swept surface is faked (the tessellator is not modified)

#### Scenario: An AP242 PMI entity still never triggers a decline (host)
- GIVEN an ISO-10303-21 AP242 buffer carrying an in-slice solid PLUS PMI / annotation entities and additive plane-angle / PMI unit contexts, read on the host with no OCCT
- WHEN `step_import_native` runs its unit-context gate and its assembly-trigger scan
- THEN the PMI / annotation entities SHALL be SKIPPED (they SHALL NOT fail the mm length gate and SHALL NOT force the assembly path) AND the solid SHALL import; the file SHALL NOT decline merely because AP242 PMI entities are present (unchanged by this change)

## ADDED Requirements

### Requirement: Native STEP import SURFACE_OF_REVOLUTION analytic-quadric reductions verified vs OCCT

The `SURFACE_OF_REVOLUTION` cone / plane / sphere widening SHALL be verified by (a) **host** unit /
decline cases (OCCT-free): a `SURFACE_OF_REVOLUTION` of an **oblique** `LINE` meeting the axis reduces
to an exact native `Cone` (watertight frustum solid, identical to the `CONICAL_SURFACE`-keyword
equivalent); of a **perpendicular** `LINE` to a native `Plane` (flat annulus, identical to the
`PLANE`-keyword equivalent); of an **on-axis** `CIRCLE` / arc to a native `Sphere` (radius = the circle
radius, watertight, identical to the `SPHERICAL_SURFACE`-keyword equivalent); each reduction VERIFIED
to pass through the profile; and a `SURFACE_OF_REVOLUTION` of an **off-axis circle** (torus), an
**`ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS`** profile (general revolved surface), and a **skew** oblique
line (hyperboloid) each DECLINE to NULL, like the landed `TOROIDAL_SURFACE`; and the parallel-line →
cylinder case, and the single-solid, flat multi-solid, placed rigid / uniform-scale / mirror assembly,
AP242, trimmed-curve, quadric, and bspline-face round-trip cases STILL pass. And (b) a **simulator
sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade: a FOREIGN OCCT-authored CONE solid whose
lateral wall is a `SURFACE_OF_REVOLUTION` of an oblique line imports natively as a cone and matches the
OCCT re-import (count / volume / watertight / bbox); a FOREIGN OCCT-authored solid whose face is a
`SURFACE_OF_REVOLUTION` of a perpendicular line imports natively as a plane and matches the OCCT
re-import; a FOREIGN OCCT-authored SPHERE solid whose face is a `SURFACE_OF_REVOLUTION` of an on-axis
semicircle imports natively as a sphere and matches the OCCT re-import; and a FOREIGN OCCT-authored
solid whose face is a `SURFACE_OF_REVOLUTION` of an off-axis circle (a torus) or of an ellipse /
B-spline profile DECLINES natively and imports via OCCT identical to `cc_set_engine(0)`. The parity
test SHALL restore the OCCT default in teardown and SHALL carry its own `main()` (on the
`run-sim-suite.sh` SKIP list) so the suite assertion count is unchanged. Every existing suite
(`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and every prior native capability (the STEP
export slice, the flat multi-solid + ELLIPSE + bspline-face + rigid / uniform-scale / mirror assembly +
AP242 + trimmed-curve + revolution-cylinder import slices, shape healing, SSI S1–S5, native blends +
#6/#7, marching, boolean, construct, tessellation) SHALL stay green at the OCCT default with no
regression.

#### Scenario: A foreign OCCT-authored cone-revolution solid imports natively and matches OCCT (sim)
- GIVEN an OCCT-authored solid whose lateral face is a `SURFACE_OF_REVOLUTION` of a straight line oblique to and meeting the axis (a truncated cone / frustum), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance, proving the exact analytic cone reduction of a foreign `SURFACE_OF_REVOLUTION`

#### Scenario: A foreign OCCT-authored plane-revolution face imports natively and matches OCCT (sim)
- GIVEN an OCCT-authored solid whose face is a `SURFACE_OF_REVOLUTION` of a straight line perpendicular to the axis (a flat annular cap), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance, proving the exact analytic plane reduction of a foreign `SURFACE_OF_REVOLUTION`

#### Scenario: A foreign OCCT-authored sphere-revolution solid imports natively and matches OCCT (sim)
- GIVEN an OCCT-authored sphere solid whose face is a `SURFACE_OF_REVOLUTION` of an on-axis semicircular arc revolved about its diameter, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance, proving the exact analytic sphere reduction of a foreign `SURFACE_OF_REVOLUTION`

#### Scenario: A foreign OCCT-authored torus / general revolution declines to OCCT (sim)
- GIVEN an OCCT-authored solid whose face is a `SURFACE_OF_REVOLUTION` of an off-axis circular arc (a torus) or of an ellipse / B-spline profile (a general revolved surface), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND OCCT `STEPControl_Reader` SHALL import the file identical to `cc_set_engine(0)`, proving honest fall-through with no fabricated torus / hyperboloid / revolved geometry, consistent with the `TOROIDAL_SURFACE` decline

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, the flat multi-solid + ELLIPSE + bspline-face + rigid / uniform-scale / mirror assembly + AP242 + trimmed-curve + revolution-cylinder import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress
