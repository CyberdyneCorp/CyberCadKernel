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

The change SHALL NOT add any native code path for `cc_step_import`, `cc_iges_export`, or
`cc_iges_import`. `NativeEngine::step_import`, `NativeEngine::iges_export`, and
`NativeEngine::iges_import` SHALL remain unconditional fall-throughs to the OCCT engine
under BOTH engine settings. STEP import (an arbitrary ISO-10303-21 parser + whole-schema
Part-42 + AP203/AP214 entity resolver + B-rep reconstruction + foreign-file healing) and
IGES (a separate older format writer + parser) are explicitly OUT OF SCOPE — the large,
long-lived parts that remain OCCT-backed. The spec SHALL state this split truthfully:
native STEP export is the achievable native ceiling for exchange, and full native STEP
import + IGES + a general-curved kernel are what still block #8 `drop-occt`.

#### Scenario: STEP import is identical under both engines (parity)
- GIVEN a STEP file on a booted iOS simulator
- WHEN `cc_step_import(path)` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two imported shapes SHALL be identical (the native engine intercepts none of the parse — it stays OCCT `STEPControl_Reader`)

#### Scenario: IGES export and import are identical under both engines (parity)
- GIVEN a solid and an IGES file on a booted iOS simulator
- WHEN `cc_iges_export(body, path)` and `cc_iges_import(path)` are called with the native engine active and with the OCCT default
- THEN the results SHALL be identical under both engines (IGES stays OCCT `IGESControl_*`; the native engine intercepts neither)

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

