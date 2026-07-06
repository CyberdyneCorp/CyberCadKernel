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
ISO-10303-21 (STEP Part 21) file and reconstructs a native `topology::Shape` of type
`Solid` when the file's root is a single `ADVANCED_BREP_SHAPE_REPRESENTATION` →
`MANIFOLD_SOLID_BREP` → `CLOSED_SHELL` whose faces carry only surfaces of kind `PLANE`,
`CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, or
`B_SPLINE_SURFACE_WITH_KNOTS` (non-rational) and whose edges carry only curves of kind
`LINE`, `CIRCLE`, or `B_SPLINE_CURVE_WITH_KNOTS` (non-rational) — the exact entity set the
native STEP writer emits. The reader SHALL (a) tokenize the DATA section into a
`map<#id, Record>` handling integer refs `#M`, reals including typed forms (`1.`, `1.E2`,
`-3.5E-07`), strings (`'...'` with embedded `''`), enums (`.T.` / `.PLANE.`), lists
`( ... )`, `$` (null), `*` (derived), and combined-instance `( SUB(...) SUB(...) )` records;
(b) resolve leaf geometry — `CARTESIAN_POINT` → `math::Point3` in **millimetres**,
`DIRECTION` → `math::Dir3`, `AXIS2_PLACEMENT_3D` → `math::Ax3`, the in-scope curves →
`EdgeCurve`, the in-scope surfaces → `FaceSurface` (knots RLE-expanded from the
`(mults),(knots)` pair, surface poles read row-major, U outer); (c) build topology following
refs — `VERTEX_POINT` → vertex, `EDGE_CURVE` → one shared edge per `#id`, `ORIENTED_EDGE` →
the oriented shared edge, `EDGE_LOOP` → wire, `FACE_OUTER_BOUND`/`FACE_BOUND` +
`ADVANCED_FACE` sense → face, `CLOSED_SHELL`/`MANIFOLD_SOLID_BREP` → shell/solid — dropping
the writer's periodic-wall SEAM edge (an `EDGE_CURVE` referenced by two opposite-sense
`ORIENTED_EDGE`s within one loop); and (d) run `heal::healShell` on the assembled shell to
close the file's sub-tolerance gaps. This reader SHALL remain OCCT-free and host-buildable
and SHALL reference no OCCT / `IEngine` / `EngineShape` type. It SHALL NOT modify the STEP
writer or the tessellator.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A native-written cylinder imports back with correct quadric geometry (host round-trip)
- GIVEN a native-built cylinder / capped-cylinder `Solid` (a `CYLINDRICAL_SURFACE` lateral face + `PLANE` caps bounded by `CIRCLE` edges) serialized by `step_export_native`, on the host with no OCCT
- WHEN `step_import_native` reads it back
- THEN the lateral face SHALL reconstruct a `FaceSurface::Kind::Cylinder` (frame + radius in mm), the cap boundaries `EdgeCurve::Kind::Circle` AND the writer's full-turn-wall SEAM edge SHALL be dropped so the reconstructed edge count matches the native source EXACTLY AND the solid SHALL be valid + watertight after `healShell`

#### Scenario: A native-written B-spline-surfaced solid imports its B_SPLINE entities (host round-trip)
- GIVEN a native-built solid with a `B_SPLINE_SURFACE_WITH_KNOTS` face and `B_SPLINE_CURVE_WITH_KNOTS` edges serialized by `step_export_native`, on the host with no OCCT
- WHEN `step_import_native` reads it back
- THEN the face SHALL reconstruct a `FaceSurface::Kind::BSpline` with the same degrees, control-point grid (row-major), and RLE-expanded knot vectors, and its boundary curves `EdgeCurve::Kind::BSpline`, AND the round-tripped solid's volume SHALL match the original within analytic tolerance

### Requirement: Native STEP import runs healShell and returns NULL for out-of-scope or unhealable files

`step_import_native` SHALL run `heal::healShell(candidateSolid, HealOptions{tolerance})`
after assembling the B-rep, because a STEP file writes each face's boundary independently at
floating-point precision so the reconstructed shell is a face soup / open shell coincident
only within tolerance. When `healShell` returns `Healed`, the reader SHALL return the
watertight, positively-oriented solid it produced. The reader SHALL return a **NULL Shape
(DECLINE)** — and never a partial or invented solid — when ANY of: (i) `healShell` returns
`Unhealed` (a gap beyond tolerance, a genuinely open shell, a non-manifold input, or a
self-verify failure); (ii) the file has zero or more than one root `MANIFOLD_SOLID_BREP`
(assembly / multiple roots); (iii) a referenced entity has an unsupported keyword or a
surface/curve kind outside the in-scope set, or a rational (weighted) B-spline wrap; (iv) a
non-millimetre length-unit context (no silent rescale); or (v) a malformed / dangling
record. The tolerance SHALL NEVER be widened to force a `Healed` result; the honest residual
gap SHALL be reported, not hidden.

#### Scenario: A file whose B-rep cannot be healed returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap larger than the heal tolerance (a genuinely open shell), read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and runs `healShell`
- THEN `healShell` SHALL return `Unhealed` AND `step_import_native` SHALL return a NULL Shape (DECLINE) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: An assembly / multi-root or unsupported-surface file returns NULL (host)
- GIVEN an ISO-10303-21 buffer with two `MANIFOLD_SOLID_BREP` roots (an assembly), OR a face over a `TOROIDAL_SURFACE` / `SURFACE_OF_REVOLUTION` / rational B-spline (out of the writer-emitted subset), read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid, so the engine can fall through to OCCT — no geometry the file did not describe is invented

### Requirement: Native STEP import is native-else-fallback, self-verified, guarded by OCCT

`NativeEngine::step_import(path)` SHALL first call `step_import_native(path)`. When it
returns a non-null solid, the engine SHALL **self-verify** the solid (a valid, watertight
solid with enclosed volume > 0, via the native tessellate self-verify already used by the
healer and booleans) and, on success, wrap it as a native `EngineShape` and return the
tracked handle. When `step_import_native` returns a NULL Shape (DECLINE) OR the self-verify
FAILS, the engine SHALL fall through to OCCT `STEPControl_Reader` (labelled), re-reading the
SAME file from scratch — never handing a native void to OCCT. The native reader and the OCCT
fallback SHALL keep OCCT behind `CYBERCAD_HAS_OCCT`; `src/native/**` SHALL contain zero OCCT
includes/symbols. `NativeEngine::iges_export`, `NativeEngine::iges_import`, and
`NativeEngine::step_export` SHALL remain UNCHANGED. This SHALL NOT change the `cc_*` ABI and
SHALL NOT change the default engine (stays OCCT).

#### Scenario: A native STEP file imports natively and matches OCCT (sim vs OCCT — the correctness gate)
- GIVEN a file the native STEP writer produced, on a booted iOS simulator (OCCT linked), with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid, watertight solid AND its volume / bounding box SHALL match the OCCT `STEPControl_Reader` import within tolerance

#### Scenario: A foreign OCCT-written STEP imports natively (sim vs OCCT — foreign STEP)
- GIVEN an OCCT-built box / cylinder written to a STEP file by OCCT `STEPControl_Writer`, on a booted iOS simulator
- WHEN `cc_step_import(path)` imports that foreign file with the native engine active (`cc_set_engine(1)`)
- THEN the native import SHALL return a valid, watertight solid whose volume / bounding box match the OCCT `STEPControl_Reader` re-import within tolerance, proving the native reader reads foreign-generated STEP of the writer-emitted subset

#### Scenario: An out-of-scope file falls through to OCCT (sim vs OCCT)
- GIVEN an assembly / AP242 / unsupported-surface STEP file, with the native engine active (`cc_set_engine(1)`)
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

