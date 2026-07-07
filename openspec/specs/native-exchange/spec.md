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

The native exchange library SHALL provide `step_import_native(path)` that reads an ISO-10303-21 (STEP
Part 21) file — **independently of its `FILE_SCHEMA` header** (AP203, AP214 `AUTOMOTIVE_DESIGN`, or AP242
are all accepted; the reader gates on entities + the mm length-unit context, not the schema string, and
**skips** AP242 PMI / annotation entities and additive plane-angle / solid-angle / PMI unit contexts) —
and reconstructs a native `topology::Shape`: a `Solid` (one root `MANIFOLD_SOLID_BREP`), a **flat**
`Compound` (several co-equal roots, no transform tree), or a **placed** `Compound` (a single-level
assembly composed by a rigid / uniform-scale / mirror transform, else DECLINE, with mirror
orientation-compensation so each placed solid self-verifies watertight; the tessellator SHALL NOT be
modified and no normal SHALL be fabricated). The faces SHALL carry surfaces of kind `PLANE`,
`CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS`
(non-rational), a **`TOROIDAL_SURFACE`** (→ native `Kind::Torus`, when T1 lands), or a
**`SURFACE_OF_REVOLUTION`** that maps to a native surface — a straight generatrix **parallel** (→
cylinder), **oblique-meeting** (→ cone), or **perpendicular** (→ plane); an **on-axis circle / arc** (→
sphere); an **off-axis circle / arc** (→ **torus**, `Kind::Torus`, when T1 lands); or an **ellipse /
B-spline** profile (→ a **rational `Kind::BSpline`** revolved surface, when T2 lands) — and the edge
curves of kind `LINE`, `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (non-rational), or a
**`TRIMMED_CURVE`** whose basis is one of those. A full periodic **sphere** face OCCT emits as a single
seam+double-pole face SHALL be reconstructed as a native `Sphere` **bare periodic surface** (NULL outer
wire) as before. A full periodic **torus** face (doubly periodic, NO pole) SHALL be reconstructed
watertight per the diagnosed OCCT bound (a seam EDGE_LOOP trimmed face, or a doubly-periodic bare-surface
face analogous to the sphere path extended to two seams) — and if it cannot close watertight within the
additive budget, the torus SHALL DECLINE. Specifically:

- **A `TRIMMED_CURVE`** SHALL be unwrapped to its basis curve (recursively; B-spline basis takes its
  `[first,last]` from the `PARAMETER_VALUE` trims clamped to the clamped knot span, analytic basis keeps
  the vertex-derived range) exactly as the landed trimmed-curve slice does (unchanged).
- **A `TOROIDAL_SURFACE`** `('',#axis2placement, major_radius, minor_radius)` SHALL be mapped by resolving
  the `AXIS2_PLACEMENT_3D` frame and the two trailing reals and building a native `FaceSurface` of kind
  `Torus` (`radius` = major, `minorRadius` = minor) — reconstructed watertight (T1) or DECLINED if the
  additive torus path does not close.
- **A `SURFACE_OF_REVOLUTION`** `('',#profile,#axis1)` SHALL be mapped by resolving the axis
  (`AXIS1_PLACEMENT` — origin + one direction, `$` axis → +Z) and the **profile** curve, then classifying
  the profile + axis by MEASUREMENT (never by a keyword) and mapping it to the EXACT native surface it
  sweeps, VERIFIED to pass through the profile within a scale-relative tolerance before emission:
  - a straight `LINE` **parallel** → native `Cylinder`; **oblique meeting** the axis → native `Cone`;
    **perpendicular** → native `Plane` (all landed, unchanged);
  - a `CIRCLE` / arc **centred ON the axis** with its plane **containing the axis** → native `Sphere`
    (landed, unchanged);
  - a `CIRCLE` / arc **centred OFF the axis** whose plane admits a ring torus → a native **`Torus`**
    (`radius` = the perpendicular distance from the circle centre to the axis = major; `minorRadius` = the
    circle radius = minor; frame origin on the axis, Z = the axis), VERIFIED the generatrix circle lies on
    the torus tube (**T1**);
  - an **`ELLIPSE`** or **`B_SPLINE_CURVE_WITH_KNOTS`** profile → a native **rational `Kind::BSpline`**
    surface: the revolution's rational-quadratic full circle in `u` tensored with the profile's own
    rational-or-nonrational representation in `v` (poles `P_ij` placed on the revolution circle at each
    profile pole, weights `w_ij = w^u_i · w^v_j`), VERIFIED sampled profile points lie on the reconstructed
    surface (**T2**).

  In **every** other case, AND whenever the mapped torus face (T1) or revolved B-spline face (T2) does not
  reconstruct watertight, the reader SHALL DECLINE (NULL → OCCT): a `CIRCLE` whose plane does not admit a
  ring torus (degenerate), a **skew** oblique `LINE` (a hyperboloid of one sheet — no native kind), a
  `LINE` **on** the axis (degenerate), a **degenerate axis**, a profile whose revolution is not faithfully
  representable, and any mapped face that fails the faithful-reduction guard or the watertight self-verify.

The reader SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. It SHALL make the tessellator change ADDITIVE-ONLY (a new `Kind::Torus` mesh branch
that does not perturb any existing mesh path — proven byte-identical), SHALL prefer to leave the STEP
writer unchanged (OCCT-authored fixtures), SHALL NOT import PMI / annotation entities as geometry, and
SHALL NOT fabricate a curve, a surface, a trim, a placement, or a solid the file does not describe, nor
weaken any tolerance.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A TOROIDAL_SURFACE face maps to a native torus or declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE('',#axis2,major,minor)`, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL build a native `FaceSurface` of kind `Torus` (`radius` = major, `minorRadius` = minor) AND — when the additive torus mesh path + face reconstruction close watertight (T1 lands) — the assembled solid SHALL be valid + watertight with the torus volume `2·π²·R·r²` and matching bbox; OTHERWISE it SHALL return a NULL Shape (DECLINE) so the engine falls through to OCCT — never a fabricated or non-watertight torus

#### Scenario: A SURFACE_OF_REVOLUTION of an off-axis circle maps to a native torus or declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a `CIRCLE` / arc whose centre is OFF the axis and whose plane admits a ring torus, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL build a native `Torus` (`radius` = the perpendicular distance from the circle centre to the axis, `minorRadius` = the circle radius), VERIFIED the generatrix circle lies on the torus tube, AND — when T1 lands — the assembled solid SHALL be valid + watertight and identical to the `TOROIDAL_SURFACE`-keyword-equivalent solid; OTHERWISE it SHALL DECLINE (NULL) so the engine falls through to OCCT

#### Scenario: A SURFACE_OF_REVOLUTION of an ellipse / B-spline profile maps to a native rational B-spline or declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is an `ELLIPSE` or a `B_SPLINE_CURVE_WITH_KNOTS`, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL build the EXACT revolved rational tensor-product B-spline (`Kind::BSpline`, the rational-quadratic full circle in `u` ⊗ the profile in `v`), VERIFIED sampled profile points lie on the surface, AND — when the surface is faithfully representable and self-verifies watertight (T2 lands) — the assembled solid SHALL be valid + watertight and match the OCCT re-import within tolerance; OTHERWISE it SHALL DECLINE (NULL) so the engine falls through to OCCT — never a mangled or approximate surface

#### Scenario: The on-axis circle / line quadric reductions and prior slices are unchanged (host)
- GIVEN in-scope ISO-10303-21 buffers exercising a `LINE` parallel / oblique-meeting / perpendicular and an on-axis `CIRCLE` `SURFACE_OF_REVOLUTION`, plus the trimmed-curve, full-sphere bare-periodic, quadric, bspline-face, and rigid / uniform-scale / mirror assembly cases, read on the host with no OCCT
- WHEN `step_import_native` resolves each
- THEN the cylinder / cone / plane / sphere reductions and every prior import path SHALL behave EXACTLY as before (the new torus + general-revolution arms are additive; adding `Kind::Torus` and the `minorRadius` field leaves every existing kind byte-identical)

### Requirement: Native STEP import runs healShell and returns NULL for out-of-scope or unhealable files

`step_import_native` SHALL rely on the shared-node reconstruction and SHALL return the assembled `Solid` /
flat `Compound` / **placed `Compound`** for the engine to self-verify. A `TOROIDAL_SURFACE` face and an
off-axis-circle `SURFACE_OF_REVOLUTION` SHALL be reconstructed onto a native `Kind::Torus` face (**T1**),
and an ellipse / B-spline `SURFACE_OF_REVOLUTION` onto a native rational `Kind::BSpline` face (**T2**),
each VERIFIED to pass through the profile and subject to the same watertight self-verify. The reader SHALL
return a **NULL Shape (DECLINE)** — and never a partial or invented solid — when ANY of: (i) the assembled
shell is a genuinely open / non-manifold B-rep, or a placed member fails the self-verify, or **a mapped
torus / revolved-B-spline face does not reconstruct watertight** (the additive torus mesh path or the
periodic revolved B-spline leaves a gap); (ii) the file has **zero** root `MANIFOLD_SOLID_BREP`, or carries
a transform tree the reader cannot compose; (iii) a referenced entity has a surface kind outside
{`PLANE`,`CYLINDRICAL_SURFACE`,`CONICAL_SURFACE`,`SPHERICAL_SURFACE`,`B_SPLINE_SURFACE_WITH_KNOTS`,
`TOROIDAL_SURFACE`, a mappable `SURFACE_OF_REVOLUTION`} — explicitly INCLUDING a **skew** oblique-line
revolution (hyperboloid), a directly-authored **arbitrary rational (weighted)** B-spline surface, and a
general swept / bounded / offset surface — or a curve kind outside
{`LINE`,`CIRCLE`,`ELLIPSE`,`B_SPLINE_CURVE_WITH_KNOTS`, a `TRIMMED_CURVE` over one of those}; (iv) a
non-millimetre LENGTH-unit context; or (v) a malformed / dangling record. AP242 PMI / annotation entities
SHALL be **skipped**. The tolerance SHALL NEVER be widened to force a pass, and no existing tessellation
path SHALL be perturbed; the honest residual SHALL be reported, not hidden.

#### Scenario: A file whose B-rep cannot form a watertight solid returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap so the assembled shell is a genuinely open shell, read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and the engine self-verifies it
- THEN the result SHALL NOT self-verify watertight AND the import SHALL DECLINE (NULL) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: A mapped torus or revolved-B-spline face that leaves a gap declines to OCCT (host)
- GIVEN an in-scope ISO-10303-21 buffer whose `TOROIDAL_SURFACE` / off-axis-circle-revolution torus face, or whose ellipse / B-spline revolution face, reconstructs with a seam or pole gap that does not self-verify watertight, read on the host with no OCCT
- WHEN `step_import_native` assembles the solid and the engine self-verifies it
- THEN the import SHALL DECLINE (NULL) — keeping the honest OCCT fallback for that track — never a leaky or fabricated face, and the tolerance SHALL NOT be widened

#### Scenario: A hyperboloid / arbitrary-rational / swept surface still returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a face over a skew-oblique-line `SURFACE_OF_REVOLUTION` (hyperboloid), a directly-authored arbitrary rational B-spline surface, or a general swept / bounded / offset surface, read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid, so the engine can fall through to OCCT — no hyperboloid / rational / swept surface is faked (the tessellator is not perturbed)

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

### Requirement: Native STEP import TORUS and general-revolution mapping verified vs OCCT with tessellation zero-regression proof

The torus (T1) and general-revolution (T2) widening SHALL be verified by (a) **host** unit / decline
cases (OCCT-free): a `TOROIDAL_SURFACE` face and an off-axis-circle `SURFACE_OF_REVOLUTION` map to a native
`Kind::Torus` (major = the axis distance, minor = the circle radius), VERIFIED the circle lies on the
torus, watertight IF T1 lands else an honest DECLINE; an ellipse / B-spline `SURFACE_OF_REVOLUTION` maps to
a native rational `Kind::BSpline`, VERIFIED the profile lies on the surface, watertight IF T2 lands else an
honest DECLINE; and the on-axis-circle → sphere, line → cylinder/cone/plane, trimmed-curve, quadric,
bspline-face, and rigid / uniform-scale / mirror assembly cases STILL pass byte-identical. And (b) a
**simulator sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade under `cc_set_engine(1)`: a FOREIGN
OCCT-authored TORUS solid (off-axis-circle revolution / `TOROIDAL_SURFACE`) imports natively as a torus and
matches the OCCT re-import (count / volume / watertight / bbox) IF T1 lands, else DECLINES natively and
imports via OCCT identical to `cc_set_engine(0)`; a FOREIGN OCCT-authored general-revolution (ellipse
profile) solid imports natively as a rational B-spline and matches the OCCT re-import IF T2 lands, else
DECLINES to OCCT identical to `cc_set_engine(0)`. And (c) a **tessellation ZERO-REGRESSION proof**: the
full tessellation-sensitive sim set (`scripts/run-sim-suite.sh`, curved-fillet, curved-chamfer,
curved-boolean, wrap-emboss, phase3) SHALL stay green with IDENTICAL triangle counts, watertight status,
and volumes for every existing sphere / cylinder / cone / plane / B-spline face — proving the additive
`Kind::Torus` mesh path perturbs NOTHING. If ANY existing mesh changes, T1's mesh path SHALL be reverted
and the torus SHALL keep the OCCT decline. The parity test SHALL restore the OCCT default in teardown and
SHALL carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite assertion count is
unchanged. Every existing suite (host CTest, GPU / Phase-3) and every prior native capability (STEP export,
the flat multi-solid + ELLIPSE + bspline-face + assembly + AP242 + trimmed-curve + revolution-quadric +
full-sphere import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct,
tessellation) SHALL stay green at the OCCT default with no regression.

#### Scenario: A foreign OCCT-authored torus solid imports natively or declines to OCCT (sim)
- GIVEN an OCCT-authored TORUS solid whose face is a `TOROIDAL_SURFACE` or an off-axis-circle `SURFACE_OF_REVOLUTION`, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN — if T1 landed — the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance; OTHERWISE `step_import_native` SHALL return NULL and OCCT SHALL import the file identical to `cc_set_engine(0)`, proving the honest torus fallback

#### Scenario: A foreign OCCT-authored general-revolution solid imports natively or declines to OCCT (sim)
- GIVEN an OCCT-authored solid whose face is an ellipse-profile (or B-spline-profile) `SURFACE_OF_REVOLUTION`, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN — if T2 landed — the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance; OTHERWISE `step_import_native` SHALL return NULL and OCCT SHALL import the file identical to `cc_set_engine(0)`, proving the honest general-revolution fallback

#### Scenario: The additive torus mesh path leaves every existing tessellation byte-identical (sim)
- GIVEN this change applied on an OCCT build, with the full tessellation-sensitive sim set (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3) run on a booted iOS simulator
- WHEN every existing sphere / cylinder / cone / plane / B-spline face is meshed and compared against the pre-change baseline
- THEN the triangle counts, watertight status, and volumes SHALL be IDENTICAL to the baseline (the `Kind::Torus` mesh branch is additive and perturbs no existing path); if ANY differs, the torus mesh path SHALL be reverted and the torus SHALL keep the OCCT decline

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and STEP export, the flat multi-solid + ELLIPSE + bspline-face + assembly + AP242 + trimmed-curve + revolution-quadric + full-sphere import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

