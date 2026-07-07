# native-exchange Specification

## Purpose
TBD - created by archiving change add-native-data-exchange. Update Purpose after archive.
## Requirements
### Requirement: Native STEP AP203 export of a native-representable solid

The native exchange library SHALL serialize a native-built solid (`topology::Shape` of
type `Solid`) to a valid **ISO 10303-21** (STEP AP203) exchange file when the solid is
**native-representable** — it contains a closed / manifold `Shell` whose faces carry a
`FaceSurface` of kind `Plane`, `Cylinder`, `Cone`, `Sphere`, or `BSpline`, and whose edges
carry an `EdgeCurve` of kind `Line`, `Circle`, or `BSpline`. The emitted file SHALL
contain (a) the ISO-10303-21 framing (`ISO-10303-21;` … `HEADER;` … `DATA;` … `ENDSEC;` …
`END-ISO-10303-21;`) with a `FILE_SCHEMA` naming the AP203 configuration-controlled-3d
schema; (b) a **millimetre** unit context (`SI_UNIT(.MILLI.,.METRE.)` length plus plane /
solid angle units, an `UNCERTAINTY_MEASURE_WITH_UNIT`, and a
`GEOMETRIC_REPRESENTATION_CONTEXT`); (c) the Part-42 B-rep entity graph — exactly one
`MANIFOLD_SOLID_BREP` referencing one `CLOSED_SHELL`, one `ADVANCED_FACE` per native face
whose surface entity matches the native `FaceSurface::kind` (`PLANE` /
`CYLINDRICAL_SURFACE` / `CONICAL_SURFACE` / `SPHERICAL_SURFACE` / `B_SPLINE_SURFACE`), each
face bounded by `FACE_OUTER_BOUND` / `FACE_BOUND` → `EDGE_LOOP` → `ORIENTED_EDGE` →
`EDGE_CURVE` with `LINE` / `CIRCLE` / `B_SPLINE_CURVE_WITH_KNOTS` curves and `VERTEX_POINT`
/ `CARTESIAN_POINT` / `DIRECTION` leaves; and (d) the AP203 product / context wrapper
(`APPLICATION_CONTEXT` … `PRODUCT_DEFINITION` …) with an
`ADVANCED_BREP_SHAPE_REPRESENTATION` binding the `MANIFOLD_SOLID_BREP` to the mm context.
All coordinates SHALL be in **true millimetres**. Shared native nodes (a `CARTESIAN_POINT`,
`DIRECTION`, `VERTEX_POINT`, or an `EDGE_CURVE` used by two adjacent faces) SHALL emit ONE
deduplicated entity record, and every `#n` reference SHALL be defined before it is used (no
dangling forward reference). This writer SHALL remain OCCT-free and host-buildable and
SHALL reference no OCCT / `IEngine` / `EngineShape` type.

#### Scenario: A native box exports a well-formed AP203 STEP file with mm units (host)
- GIVEN a native-built axis-aligned box `Solid` (six `Plane` faces, `Line`-edged loops), built on the host with no OCCT
- WHEN the native writer serializes it to an ISO-10303-21 buffer
- THEN the buffer SHALL be framed `ISO-10303-21;` … `END-ISO-10303-21;` with a `FILE_SCHEMA` naming the AP203 schema and a `SI_UNIT(.MILLI.,.METRE.)` length unit AND SHALL contain exactly one `MANIFOLD_SOLID_BREP` referencing one `CLOSED_SHELL` with six `ADVANCED_FACE`s over `PLANE` surfaces AND every `#n` reference SHALL resolve

#### Scenario: A native cylinder emits the correct curved surface and circle edge entities (host)
- GIVEN a native-built cylinder `Solid` (a `Cylinder` lateral face + two `Plane` caps bounded by `Circle` edges), built on the host with no OCCT
- WHEN the native writer serializes it
- THEN the lateral `ADVANCED_FACE` SHALL reference a `CYLINDRICAL_SURFACE` (placement + radius in mm) AND the cap boundary edges SHALL be `EDGE_CURVE`s over `CIRCLE` curves AND the two caps SHALL share the same `VERTEX_POINT` / `EDGE_CURVE` records with the lateral face (deduplicated shared nodes)

#### Scenario: A native B-spline-surfaced solid emits B_SPLINE entities (host)
- GIVEN a native-built solid with a `FaceSurface::Kind::BSpline` face and `BSpline` edges, built on the host with no OCCT
- WHEN the native writer serializes it
- THEN the face SHALL reference a `B_SPLINE_SURFACE_WITH_KNOTS` (degrees, control-point grid, knots + multiplicities; weight-wrapped when rational) AND its boundary curves SHALL be `B_SPLINE_CURVE_WITH_KNOTS` entities, all with coordinates in mm

### Requirement: Native STEP export is native-else-fallback, guarded by an OCCT-read round-trip self-check

`NativeEngine::step_export(body, path)` SHALL, when `body` is a native body, attempt the
native writer; when the writer returns a non-empty buffer it SHALL write the buffer to
`path` and then run a **mandatory OCCT-read round-trip self-check**: OCCT
`STEPControl_Reader` SHALL re-read the written file, the reconstructed shape SHALL be a
**valid** single solid (`BRepCheck`), and its **volume**, **bounding box**, and
**sub-shape counts / topology** SHALL match the source native solid within tolerance (the
SOLID geometrically identical, up to OCCT's shared / periodic representation). Only when
the self-check passes SHALL `step_export` return `1` (per the `cc_step_export` contract).
If the native writer DECLINES (empty buffer — an unrepresentable / non-manifold solid), or
the round-trip self-check FAILS, or `body` is a foreign / OCCT-built solid, the engine
SHALL **DISCARD** any native file and fall through to OCCT `STEPControl_Writer` (labelled)
— except that a native body which DECLINEs (no OCCT B-rep available to serialize) SHALL
report an honest failure (`0`) rather than a faked or lossy file. The engine SHALL NEVER
leave a STEP file on disk that re-reads as a different or invalid solid. The native writer
and the round-trip check SHALL keep OCCT behind `CYBERCAD_HAS_OCCT`; on the host the
structural buffer tests stand in for the round-trip. This SHALL NOT change the `cc_*` ABI
and SHALL NOT change the default engine (stays OCCT).

#### Scenario: A native STEP file re-reads through OCCT to the same solid (parity — the correctness gate)
- GIVEN a native-representable solid and the native engine active (`cc_set_engine(1)`) on a booted iOS simulator (OCCT linked)
- WHEN `cc_step_export(body, path)` writes the file and OCCT `STEPControl_Reader` re-reads it
- THEN `cc_step_export` SHALL return `1` AND the round-tripped shape SHALL be a valid single solid whose volume / bbox / sub-shape counts / topology match the source native solid within tolerance (with OCCT `STEPControl_Writer` under `cc_set_engine(0)` as the oracle)

#### Scenario: A native file that fails the round-trip self-check is discarded and re-exported by OCCT (parity)
- GIVEN a native export whose written file, when re-read by OCCT `STEPControl_Reader`, yields an invalid shape OR a volume / bbox / topology outside tolerance
- WHEN the mandatory self-check is applied
- THEN the engine SHALL DISCARD the native file and re-export via OCCT `STEPControl_Writer` (labelled) AND SHALL NOT leave a native file that reads back as a different or invalid solid

#### Scenario: A foreign or unrepresentable solid falls through to OCCT (host + parity)
- GIVEN `cc_step_export` of a foreign / OCCT-built solid, OR of a native solid with an `Ellipse` edge / an un-down-converted `Bezier` surface / a non-manifold shell, with the native engine active
- WHEN `cc_step_export` is invoked
- THEN the native writer SHALL return an empty buffer (DECLINE) AND (for a foreign body) the file SHALL be written by OCCT `STEPControl_Writer` identical to `cc_set_engine(0)`, proving fall-through with no native interception (a native unrepresentable body reports an honest `0`, never a faked file)

### Requirement: STEP import and IGES export/import stay OCCT (out of scope, honest)

Native IGES import and export SHALL be DESCOPED (STEP-only interchange): no native IGES
reader or writer SHALL ever be built. `cc_iges_export` and `cc_iges_import` SHALL remain
unconditional fall-throughs to the OCCT engine (`IGESControl_*`) under both engine
settings — the `cc_*` ABI SHALL be preserved (additive-only) — and at `#8 drop-occt` the
`cc_iges_*` entries SHALL be removed/stubbed (return `0`/`nil`), NOT reimplemented
natively. Native STEP import HAS landed as a first slice (the AP203 manifold-solid-brep
subset — see the native-STEP-import requirements), so STEP SHALL be the SOLE native
interchange format; IGES SHALL NOT be a `drop-occt` blocker, and the remaining
`drop-occt` exchange work SHALL be a general STEP/AP242 reader on top of the landed AP203
slice.

#### Scenario: STEP import is native-first-slice, else OCCT (parity)
- GIVEN a STEP file on a booted iOS simulator
- WHEN `cc_step_import(path)` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN a writer-emitted AP203 manifold-solid-brep subset file SHALL be read by the native reader (self-verified watertight) matching the OCCT `STEPControl_Reader` result within tolerance, and any out-of-scope file SHALL decline to OCCT — never a fabricated shape

#### Scenario: IGES export and import are identical under both engines (parity), pending descope
- GIVEN a solid and an IGES file on a booted iOS simulator
- WHEN `cc_iges_export(body, path)` and `cc_iges_import(path)` are called with the native engine active and with the OCCT default
- THEN the results SHALL be identical under both engines (IGES stays OCCT `IGESControl_*`; the native engine intercepts neither), and at `drop-occt` the `cc_iges_*` entries SHALL be removed/stubbed rather than reimplemented natively

### Requirement: STEP export parity and existing suites through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked) through the `cc_*`
facade: the native-write / OCCT-read round-trip (above) SHALL be the correctness gate for
native-representable solids, and the OCCT `STEPControl_Writer` export under
`cc_set_engine(0)` SHALL be the oracle. The fall-through cases (a foreign / OCCT-built
solid, an unrepresentable native solid) SHALL be asserted to produce a valid OCCT-written
file identical to `cc_set_engine(0)` (fall-through proof), and `cc_step_import` /
`cc_iges_export` / `cc_iges_import` SHALL be asserted identical under both engines. The
parity test SHALL restore the OCCT default in teardown and SHALL carry its own `main()`
(on the `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged. Every
existing suite (`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) SHALL stay
green at the OCCT default.

#### Scenario: Native STEP export matches the OCCT oracle round-trip (parity)
- GIVEN native-built solids (box / cylinder / a down-convertible B-spline solid) on a booted iOS simulator
- WHEN each is exported with the native engine active and its file re-read by OCCT `STEPControl_Reader`, and the same solid is exported with OCCT `STEPControl_Writer` under `cc_set_engine(0)`
- THEN the native-written file's round-tripped shape SHALL match the source solid within tolerance AND agree with the OCCT-oracle export (volume / bbox / topology), proving the native STEP file is faithful

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

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

### Requirement: Native STEP import is native-else-fallback, self-verified, guarded by OCCT

`NativeEngine::step_import(path)` SHALL first call `step_import_native(path)`. When it returns a
non-null shape, the engine SHALL **self-verify** it — for a `Solid`, a valid watertight solid with
enclosed volume > 0; for a `Compound` (flat OR placed), EVERY member `Solid` SHALL independently
self-verify watertight with enclosed volume > 0. A **uniformly-scaled** placed member SHALL
self-verify with enclosed volume `k³ × V₀ > 0`; a **mirror** placed member SHALL self-verify (after
the reader's orientation compensation) with the correct POSITIVE enclosed volume — a mirror whose
world normals point inward yields a negative enclosed volume and FAILS the self-verify → OCCT. When
`step_import_native` returns a NULL Shape (DECLINE) OR the self-verify FAILS, the engine SHALL fall
through to OCCT `STEPControl_Reader` (labelled), re-reading the SAME file from scratch. The native
reader and the OCCT fallback SHALL keep OCCT behind `CYBERCAD_HAS_OCCT`; `src/native/**` SHALL contain
zero OCCT includes/symbols. `NativeEngine::iges_export`, `NativeEngine::iges_import`, and
`NativeEngine::step_export` SHALL remain UNCHANGED. This SHALL NOT change the `cc_*` ABI and SHALL NOT
change the default engine (stays OCCT).

#### Scenario: A native STEP file imports natively and matches OCCT (sim vs OCCT — the correctness gate)
- GIVEN a file the native STEP writer produced, on a booted iOS simulator (OCCT linked), with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid, watertight solid AND its volume / bounding box SHALL match the OCCT `STEPControl_Reader` import within tolerance

#### Scenario: A foreign OCCT-authored scaled assembly imports natively as a placed compound (sim vs OCCT)
- GIVEN an OCCT-authored 2-component assembly with one component at 2× uniform scale (written by `STEPControl_Writer` on a compound of transformed solids or by `STEPCAFControl_Writer` on an XCAF assembly document), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose two member solids each self-verify valid + watertight AND whose solid COUNT, TOTAL volume (the scaled component contributing `k³ × V₀`), and per-solid bounding box + centroid/placement match the OCCT re-import within tolerance, proving uniform-scale placement parity

#### Scenario: A foreign OCCT-authored mirrored assembly imports natively watertight (sim vs OCCT)
- GIVEN an OCCT-authored assembly with a mirrored (reflected) component, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose mirrored member solid self-verifies valid + watertight with POSITIVE volume AND whose solid COUNT, TOTAL volume, and per-solid bounding box + centroid/placement match the OCCT re-import within tolerance, proving the mirror orientation compensation yields the correct outward-facing solid

#### Scenario: A foreign AP242 file with PMI imports its geometry natively (sim vs OCCT)
- GIVEN an OCCT-authored AP242 STEP file carrying an in-slice solid PLUS PMI annotations (written by `STEPCAFControl_Writer` with PMI), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return the SOLID valid + watertight AND its volume / bounding box / count SHALL match the OCCT re-import within tolerance, with the PMI ignored on both sides, proving AP242 geometry import with PMI skipped

#### Scenario: An out-of-scope file (torus / non-uniform-scale or deep-nested assembly) falls through to OCCT (sim vs OCCT)
- GIVEN a foreign OCCT-authored STEP with a `TOROIDAL_SURFACE` face, or an assembly with a component placed by a non-uniform-scale / shear transform, or a deep-nested structure, with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND the file SHALL be imported by OCCT `STEPControl_Reader` identical to `cc_set_engine(0)`, proving fall-through with no native interception and no fabricated geometry

### Requirement: IGES import/export and the STEP writer stay unchanged (out of scope, honest)

IGES import and export SHALL be DESCOPED (STEP-only decision): no native `cc_iges_export`
/ `cc_iges_import` path SHALL be built; `NativeEngine::iges_export` /
`NativeEngine::iges_import` SHALL remain unconditional OCCT fall-throughs, removed/stubbed
at `drop-occt`. This change SHALL NOT modify the native STEP writer (`step_writer.cpp`) or
the tessellator — native STEP import inverts what the writer already produces. Native STEP
import of the writer-emitted AP203 manifold-solid-brep subset SHALL be recognised as the
landed first import slice; a general STEP/AP242 reader + a general-curved kernel are what
still block `#8 drop-occt` — IGES SHALL NOT be on that list.

#### Scenario: IGES import/export are identical under both engines (parity), pending descope
- GIVEN a solid and an IGES file on a booted iOS simulator
- WHEN `cc_iges_export(body, path)` and `cc_iges_import(path)` are called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the results SHALL be identical under both engines (IGES stays OCCT `IGESControl_*`; the native engine intercepts neither) until `drop-occt` removes/stubs them

#### Scenario: The STEP writer and tessellator are byte-for-byte unchanged
- GIVEN this change applied
- WHEN `step_export_native` serializes a native solid and the tessellator meshes it
- THEN their output SHALL be identical to before this change (this descope is documentation-only; it reads/changes no code and does not alter the writer or the tessellator)

### Requirement: Native STEP import verification and existing suites through the facade

The change SHALL be verified by (a) a **host round-trip** (OCCT-free): a native solid →
`step_export_native` → `step_import_native` → tessellate reconstructs the SAME solid
(volume / bbox / topology EXACT), plus Part-21 tokenizer unit cases and DECLINE cases
returning NULL; and (b) a **simulator sim-vs-OCCT** gate (OCCT linked) through the `cc_*`
facade: importing a native-written file with the native reader and with OCCT
`STEPControl_Reader` agree within tolerance, a foreign OCCT-written STEP imports natively and
agrees with the OCCT re-import, and out-of-scope files fall through to OCCT identical to
`cc_set_engine(0)`. The parity test SHALL restore the OCCT default in teardown and SHALL
carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite assertion count is
unchanged. Every existing suite (`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and
every prior native capability (the STEP export slice, shape healing, SSI S1–S4, S5
native-pass=6, native blends + #6/#7, marching, boolean, construct, tessellation) SHALL stay
green at the OCCT default with no regression.

#### Scenario: Host round-trip reconstructs native solids exactly (host)
- GIVEN native-built solids (a box, a cylinder / capped cylinder, a holed / typed-profile solid) on the host with no OCCT
- WHEN each is exported by `step_export_native` and re-imported by `step_import_native`
- THEN each imported solid SHALL be valid + watertight with volume / bbox / face+edge+vertex counts / topology matching the original EXACTLY (the reader inverts the writer)

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, shape healing, SSI S1–S4, S5 native-pass=6, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

### Requirement: Native STEP import widening covers foreign ELLIPSE edges and multi-solid compounds, verified vs OCCT

The widening SHALL be verified by (a) **host** unit / round-trip cases (OCCT-free): an
`ELLIPSE` edge maps to `EdgeCurve::Kind::Ellipse` (major/minor from the two semi-axes) and a
degenerate ellipse DECLINEs; a `TOROIDAL_SURFACE` face DECLINEs; a two-root buffer imports as
a `Compound` of two `Solid`s while a single-root buffer still returns a `Solid` and a nested
-assembly buffer DECLINEs; and (if a non-fabricated native op builds a watertight bspline-face
solid) a B-spline-face export→import round-trip is EXACT (degrees / row-major poles / RLE
knots / volume within analytic tolerance), else documented as skipped with no fixture
fabricated; and (b) a **simulator sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade:
a FOREIGN OCCT-authored solid with an `ELLIPSE` edge imports natively and agrees with the OCCT
re-import within tolerance; a FOREIGN OCCT-authored 2-solid compound imports natively as a
compound whose per-solid mass properties / bbox / count match the OCCT re-import; and a
`TOROIDAL_SURFACE` foreign file DECLINEs natively and imports via OCCT identical to
`cc_set_engine(0)`. The parity test SHALL restore the OCCT default in teardown and SHALL carry
its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite assertion count is
unchanged. Every existing suite (`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and
every prior native capability (the STEP export slice, the first STEP import slice, shape
healing, SSI S1–S4, S5 native-pass, native blends + #6/#7, marching, boolean, construct,
tessellation) SHALL stay green at the OCCT default with no regression.

#### Scenario: A foreign OCCT-authored ellipse-edge solid imports natively and matches OCCT (T1, sim)
- GIVEN a solid carrying an `ELLIPSE` edge authored by OCCT `STEPControl_Writer` (the native writer emits no ellipse), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid, watertight solid whose volume / bounding box match the OCCT re-import within tolerance, proving the native reader reads a foreign-authored ELLIPSE the writer never produced

#### Scenario: The B-spline-face round-trip is closed only if a non-fabricated fixture exists (T3, host)
- GIVEN the reader's `B_SPLINE_SURFACE_WITH_KNOTS` / `B_SPLINE_CURVE_WITH_KNOTS` mapping and the deferred round-trip task 7.4
- WHEN an existing native construct op is checked for a watertight bspline-face solid
- THEN IF one is genuinely constructible the round-trip SHALL be added and asserted EXACT (degrees / poles / knots / volume within analytic tolerance), ELSE the round-trip SHALL be recorded as an honest skip and NO bspline-face fixture SHALL be fabricated and the STEP writer SHALL NOT be modified to synthesize one

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, the first STEP import slice, shape healing, SSI S1–S4, S5 native-pass, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

### Requirement: Native STEP import placed-assembly widening verified vs OCCT

The placed-assembly widening SHALL be verified by (a) **host** unit / decline cases
(OCCT-free): an `ITEM_DEFINED_TRANSFORMATION` (or `MAPPED_ITEM`) frame pair composes to the
expected rigid `Location` (rotation `Mat3` + translation `Vec3` checked against the frame
delta); a two-component transform-tree buffer imports as a `Compound` of two `Solid`s at their
composed WORLD placements (per-solid centroid at the placed position, not the origin); a
non-rigid / scaled / mirrored transform DECLINEs (NULL); an out-of-slice component DECLINEs
(NULL, no partial import); a transform tree with an unplaced root DECLINEs (NULL); an
AP214 / AP242 `FILE_SCHEMA` header with in-slice entities imports; and the FLAT multi-solid,
single-solid, quadric, and bspline-face round-trip cases STILL pass; and (b) a **simulator
sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade: a FOREIGN OCCT-authored 2-component
rigid assembly imports natively as a placed compound whose per-solid mass properties / bbox /
placement / count and TOTAL volume match the OCCT re-import; an UNSUPPORTED assembly (out-of-
slice component or a scaled instance) DECLINEs natively and imports via OCCT identical to
`cc_set_engine(0)`; and a foreign AP214 file with in-slice entities imports natively and
matches the OCCT re-import. The parity test SHALL restore the OCCT default in teardown and SHALL
carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite assertion count is
unchanged. Every existing suite (`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and
every prior native capability (the STEP export slice, the flat multi-solid + ELLIPSE + bspline-
face import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean,
construct, tessellation) SHALL stay green at the OCCT default with no regression.

#### Scenario: A foreign OCCT-authored rigid 2-component assembly imports natively and matches OCCT (sim)
- GIVEN a 2-component assembly (two boxes at distinct rigid placements via the STEP transform tree) authored by OCCT, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose solid COUNT, TOTAL volume, and per-solid bounding box + centroid/placement match the OCCT re-import within tolerance, proving the native reader composes a foreign-authored assembly transform tree the native writer never produced

#### Scenario: A non-rigid or out-of-slice assembly declines to OCCT (sim)
- GIVEN an OCCT-authored assembly whose one component is out of the import slice (or is placed by a scaled/mirrored transform), with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND OCCT `STEPControl_Reader` SHALL import the file identical to `cc_set_engine(0)`, proving honest fall-through with no fabricated placement or geometry

#### Scenario: A foreign AP214 file with in-slice entities imports natively (sim)
- GIVEN an OCCT-authored STEP file whose `FILE_SCHEMA` header is AP214 (`AUTOMOTIVE_DESIGN`) and whose entities are all in the import slice, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL succeed and match the OCCT re-import within tolerance, proving the reader is schema-independent (AP203 / AP214 / AP242 headers are all accepted)

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, the flat multi-solid + ELLIPSE + bspline-face import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

### Requirement: Native STEP import scaled/mirrored + AP242 widening verified vs OCCT

The scaled/mirrored-placement + AP242-tolerance widening SHALL be verified by (a) **host** unit /
decline cases (OCCT-free): the placement classifier maps a uniform-scale linear part (`k·R`, `k = 2`)
to `UniformScale(2)`, a reflection (det < 0) to `Mirror`, a rigid to `Rigid(1)`, and a non-uniform
(`diag(2,1,1)`) / shear to DECLINE (nullopt); a two-component transform-tree buffer with a 2×
uniform-scale component imports as a `Compound` whose scaled solid has volume `8 × V₀` and a scaled
centroid; a mirrored-component buffer imports as a `Compound` whose mirrored solid is valid +
watertight with POSITIVE volume and a reflected centroid (proving the orientation compensation); an
AP242 buffer (a solid + PMI / annotation entities + additive plane-angle / PMI unit contexts) imports
the solid and drops the PMI; a non-uniform / shear placement DECLINEs (NULL); and the rigid assembly,
FLAT multi-solid, single-solid, quadric, and bspline-face round-trip cases STILL pass; and (b) a
**simulator sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade: a FOREIGN OCCT-authored
2-component SCALED assembly (one component at 2× uniform scale) imports natively as a placed compound
whose per-solid mass properties / bbox / placement / count and TOTAL volume (the scaled component
contributing `k³ × V₀`) match the OCCT re-import; a FOREIGN OCCT-authored MIRRORED assembly imports
natively as a placed compound whose mirrored member is watertight with positive volume and matches the
OCCT re-import; a FOREIGN OCCT-authored AP242 file with PMI imports its SOLID natively and matches the
OCCT re-import with PMI ignored; and a NON-uniform / shear assembly DECLINEs natively and imports via
OCCT identical to `cc_set_engine(0)`. The parity test SHALL restore the OCCT default in teardown and
SHALL carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite assertion count is
unchanged. Every existing suite (`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and every prior
native capability (the STEP export slice, the flat multi-solid + ELLIPSE + bspline-face + rigid-assembly
import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct,
tessellation) SHALL stay green at the OCCT default with no regression.

#### Scenario: A foreign OCCT-authored scaled 2-component assembly imports natively and matches OCCT (sim)
- GIVEN a 2-component assembly with one component at 2× uniform scale authored by OCCT, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose solid COUNT, TOTAL volume (the scaled component contributing `k³ × V₀`), and per-solid bounding box + centroid/placement match the OCCT re-import within tolerance, proving the native reader composes a foreign-authored uniform-scale placement the native writer never produced

#### Scenario: A foreign OCCT-authored mirrored assembly imports natively watertight (sim)
- GIVEN an assembly with a mirrored (reflected) component authored by OCCT, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose mirrored member is valid + watertight with POSITIVE volume AND whose count / total volume / per-solid bbox + centroid match the OCCT re-import within tolerance, proving the topology orientation compensation produces the correct outward-facing mirrored solid

#### Scenario: A foreign AP242 file with PMI imports its geometry natively (sim)
- GIVEN an OCCT-authored AP242 STEP file carrying an in-slice solid PLUS PMI annotations, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return the SOLID valid + watertight and match the OCCT re-import (count / volume / bbox) within tolerance with the PMI ignored, proving the reader imports AP242 geometry while skipping PMI

#### Scenario: A non-uniform / shear assembly declines to OCCT (sim)
- GIVEN an OCCT-authored assembly whose one component is placed by a non-uniform-scale / shear transform, with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND OCCT `STEPControl_Reader` SHALL import the file identical to `cc_set_engine(0)`, proving honest fall-through with no fabricated placement or geometry

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, the flat multi-solid + ELLIPSE + bspline-face + rigid-assembly import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

### Requirement: Native STEP import general-surfaces (trimmed curves + cylinder-revolution) verified vs OCCT

The `TRIMMED_CURVE` + `SURFACE_OF_REVOLUTION` widening SHALL be verified by (a) **host** unit /
decline cases (OCCT-free): a `TRIMMED_CURVE` over a `LINE` / `CIRCLE` / `B_SPLINE_CURVE_WITH_KNOTS`
basis is accepted (the keyword declined before), unwrapping to the basis `EdgeCurve` and round-tripping
the solid watertight with exact / analytic volume — the B-spline basis exercising the trim-cache knot
sub-domain arm (wide trims → full span) and the analytic bases the vertex-derived range; a
`SURFACE_OF_REVOLUTION` of a `LINE` **parallel** to the axis reduces to an exact native `Cylinder`
(watertight, volume π·r²·h, identical to the `CYLINDRICAL_SURFACE`-keyword equivalent); a
`SURFACE_OF_REVOLUTION` of an **oblique** line (a cone) DECLINES, and of a **`CIRCLE`** generatrix
(a sphere / torus / general revolved surface) DECLINES — both to NULL, like the landed
`TOROIDAL_SURFACE`; and the single-solid, flat multi-solid, placed rigid / uniform-scale / mirror
assembly, AP242, quadric, and bspline-face round-trip cases STILL pass. And (b) a **simulator
sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade: a FOREIGN OCCT-authored solid with a
`TRIMMED_CURVE` edge imports natively and matches the OCCT re-import (count / volume / watertight /
bbox); a FOREIGN OCCT-authored solid whose lateral wall is a `SURFACE_OF_REVOLUTION` of a line
parallel to the axis imports natively as a cylinder and matches the OCCT re-import; and a FOREIGN
OCCT-authored solid whose face is a `SURFACE_OF_REVOLUTION` of an off-axis `CIRCLE` (a torus / general
revolved surface) DECLINES natively and imports via OCCT identical to `cc_set_engine(0)`. The parity
test SHALL restore the OCCT default in teardown and SHALL carry its own `main()` (on the
`run-sim-suite.sh` SKIP list) so the suite assertion count is unchanged. Every existing suite
(`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and every prior native capability (the STEP
export slice, the flat multi-solid + ELLIPSE + bspline-face + rigid / uniform-scale / mirror assembly
+ AP242 import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct,
tessellation) SHALL stay green at the OCCT default with no regression.

#### Scenario: A foreign OCCT-authored TRIMMED_CURVE-edge solid imports natively and matches OCCT (sim)
- GIVEN an OCCT-authored solid one of whose edges' 3D geometry is wrapped in a `TRIMMED_CURVE`, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance, proving the native reader unwraps a foreign `TRIMMED_CURVE` onto the native trimmed edge

#### Scenario: A foreign OCCT-authored cylinder-revolution face imports natively and matches OCCT (sim)
- GIVEN an OCCT-authored solid whose lateral face is a `SURFACE_OF_REVOLUTION` of a straight line parallel to the axis (reducing to a cylinder), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance, proving the exact analytic cylinder reduction of a foreign `SURFACE_OF_REVOLUTION`

#### Scenario: A foreign OCCT-authored non-line revolution declines to OCCT (sim)
- GIVEN an OCCT-authored solid whose face is a `SURFACE_OF_REVOLUTION` of an off-axis circular arc (a torus / general revolved surface), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND OCCT `STEPControl_Reader` SHALL import the file identical to `cc_set_engine(0)`, proving honest fall-through with no fabricated torus / revolved geometry, consistent with the `TOROIDAL_SURFACE` decline

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, the flat multi-solid + ELLIPSE + bspline-face + rigid / uniform-scale / mirror assembly + AP242 import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

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

