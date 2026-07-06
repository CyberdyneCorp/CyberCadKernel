# native-exchange

This change (Phase 4 #7) **widens the working native STEP import reader** (landed by
`add-native-step-import`) along three bounded, independent, honestly-gated tracks: **T1**
maps `ELLIPSE` curves onto the native ellipse edge kind and keeps `TOROIDAL_SURFACE` a
DECLINE (there is NO native torus surface kind and the tessellator must not be modified);
**T2** lifts the blanket `>1 MANIFOLD_SOLID_BREP` assembly-decline into a multi-solid
**Compound** import; **T3** closes the deferred B-spline-face round-trip IF a non-fabricated
native fixture is constructible, else reports the honest skip. Each track maps ONLY onto
native geometry that genuinely exists and DECLINEs (returns a NULL `topology::Shape` → OCCT)
otherwise. No `cc_*` ABI change; the default engine stays OCCT. The STEP writer
(`step_writer.cpp`) and the tessellator are NOT modified.

> NOTE (honest scope): this WIDENS the working import slice; it is NOT a general AP242
> parser. `TOROIDAL_SURFACE` (no native `FaceSurface::Kind::Torus`), `SURFACE_OF_REVOLUTION`,
> `TRIMMED_CURVE`, rational/weighted B-splines, `BEZIER`, **nested product-structure
> assemblies** (transform trees / mapped items / external refs), AP242 / PMI / colours /
> names, non-mm units, and non-manifold / unhealable B-reps all still DECLINE → OCCT
> `STEPControl_Reader`. IGES import/export stay OCCT `IGESControl_*`. A general native
> STEP/AP242 reader + IGES + a general-curved kernel still block #8 `drop-occt`; this change
> does NOT unblock it. **No geometry is ever fabricated: the reader parses only what is in
> the file and maps only what it genuinely supports; no B-spline-face fixture is fabricated
> for T3.**

## MODIFIED Requirements

### Requirement: Native STEP AP203 import of a writer-emitted manifold-solid-brep

The native exchange library SHALL provide `step_import_native(path)` that reads an
ISO-10303-21 (STEP Part 21) file and reconstructs a native `topology::Shape` — a `Solid`
when the file has exactly one root `MANIFOLD_SOLID_BREP`, or a `Compound` of `Solid`s when it
has more than one co-equal root `MANIFOLD_SOLID_BREP` in its representation (T2, no assembly
transform tree) — whose faces carry only surfaces of kind `PLANE`, `CYLINDRICAL_SURFACE`,
`CONICAL_SURFACE`, `SPHERICAL_SURFACE`, or `B_SPLINE_SURFACE_WITH_KNOTS` (non-rational) and
whose edges carry only curves of kind `LINE`, `CIRCLE`, `ELLIPSE`, or
`B_SPLINE_CURVE_WITH_KNOTS` (non-rational). The reader SHALL (a) tokenize the DATA section
into a `map<#id, Record>` handling integer refs `#M`, reals including typed forms (`1.`,
`1.E2`, `-3.5E-07`), strings (`'...'` with embedded `''`), enums (`.T.` / `.PLANE.`), lists
`( ... )`, `$` (null), `*` (derived), and combined-instance `( SUB(...) SUB(...) )` records;
(b) resolve leaf geometry — `CARTESIAN_POINT` → `math::Point3` in **millimetres**,
`DIRECTION` → `math::Dir3`, `AXIS2_PLACEMENT_3D` → `math::Ax3`, the in-scope curves →
`EdgeCurve` (including `ELLIPSE('',#position,semiAxis1,semiAxis2)` →
`EdgeCurve::Kind::Ellipse` with `radius = semiAxis1` along frame X and
`minorRadius = semiAxis2` along frame Y — the inverse of a `CIRCLE`), the in-scope surfaces →
`FaceSurface` (knots RLE-expanded from the `(mults),(knots)` pair, surface poles read
row-major, U outer); (c) build topology following refs — `VERTEX_POINT` → vertex,
`EDGE_CURVE` → one shared edge per `#id`, `ORIENTED_EDGE` → the oriented shared edge,
`EDGE_LOOP` → wire, `FACE_OUTER_BOUND`/`FACE_BOUND` + `ADVANCED_FACE` sense → face,
`CLOSED_SHELL`/`MANIFOLD_SOLID_BREP` → shell/solid (all roots when there are several) —
dropping the writer's periodic-wall SEAM edge (an `EDGE_CURVE` referenced by two
opposite-sense `ORIENTED_EDGE`s within one loop). A `TOROIDAL_SURFACE` face SHALL DECLINE
(there is no native `FaceSurface::Kind::Torus` and the tessellator is not modified), and a
nested product-structure assembly (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `MAPPED_ITEM`,
`REPRESENTATION_RELATIONSHIP*`, `ITEM_DEFINED_TRANSFORMATION`,
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`) SHALL DECLINE. This reader SHALL remain OCCT-free
and host-buildable and SHALL reference no OCCT / `IEngine` / `EngineShape` type. It SHALL NOT
modify the STEP writer or the tessellator.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A native-written cylinder imports back with correct quadric geometry (host round-trip)
- GIVEN a native-built cylinder / capped-cylinder `Solid` (a `CYLINDRICAL_SURFACE` lateral face + `PLANE` caps bounded by `CIRCLE` edges) serialized by `step_export_native`, on the host with no OCCT
- WHEN `step_import_native` reads it back
- THEN the lateral face SHALL reconstruct a `FaceSurface::Kind::Cylinder` (frame + radius in mm), the cap boundaries `EdgeCurve::Kind::Circle` AND the writer's full-turn-wall SEAM edge SHALL be dropped so the reconstructed edge count matches the native source EXACTLY AND the solid SHALL be valid + watertight after `healShell`

#### Scenario: An ELLIPSE edge maps to the native ellipse curve kind (T1, host)
- GIVEN an in-scope ISO-10303-21 buffer whose edge carries an `ELLIPSE('',#position,semiAxis1,semiAxis2)` curve, read on the host with no OCCT
- WHEN `step_import_native` resolves the edge's curve
- THEN it SHALL reconstruct an `EdgeCurve::Kind::Ellipse` with `frame` from `#position`, `radius == semiAxis1` (semi-major, frame X), and `minorRadius == semiAxis2` (semi-minor, frame Y), AND a degenerate ellipse (either semi-axis ≤ 0 or non-finite) SHALL DECLINE (NULL) rather than reconstruct a wrong curve

#### Scenario: Multiple root solids import as a compound (T2, host)
- GIVEN an in-scope ISO-10303-21 buffer with two co-equal root `MANIFOLD_SOLID_BREP`s (each a `CLOSED_SHELL` of in-scope faces) and no assembly transform entity, read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a `Compound` containing two `Solid`s (one per root), each reconstructed exactly as the single-solid path would, AND a buffer with exactly one root SHALL still return a bare `Solid` (the single-solid behaviour is unchanged)

#### Scenario: A native-written B-spline-surfaced solid imports its B_SPLINE entities (host round-trip)
- GIVEN a native-built solid with a `B_SPLINE_SURFACE_WITH_KNOTS` face and `B_SPLINE_CURVE_WITH_KNOTS` edges serialized by `step_export_native`, on the host with no OCCT
- WHEN `step_import_native` reads it back
- THEN the face SHALL reconstruct a `FaceSurface::Kind::BSpline` with the same degrees, control-point grid (row-major), and RLE-expanded knot vectors, and its boundary curves `EdgeCurve::Kind::BSpline`, AND the round-tripped solid's volume SHALL match the original within analytic tolerance — verified ONLY when a non-fabricated native op is confirmed to build such a watertight bspline-face solid (T3); otherwise the round-trip SHALL be documented as skipped and no fixture SHALL be fabricated

### Requirement: Native STEP import runs healShell and returns NULL for out-of-scope or unhealable files

`step_import_native` SHALL rely on the shared-node reconstruction (adjacent faces referencing
the same `EDGE_CURVE` node by `#id`, watertight by construction) and SHALL return the
assembled `Solid` / `Compound` for the engine to self-verify; the `heal::healShell` deferral
described by `add-native-step-import` applies where a file carries independent per-face edges.
The reader SHALL return a **NULL Shape (DECLINE)** — and never a partial or invented solid —
when ANY of: (i) the assembled shell is a genuinely open / non-manifold B-rep that cannot
form a watertight solid (surfaced as a failed engine self-verify); (ii) the file has **zero**
root `MANIFOLD_SOLID_BREP`, OR carries a **nested product-structure assembly** entity
(`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `MAPPED_ITEM`, `REPRESENTATION_RELATIONSHIP*`,
`ITEM_DEFINED_TRANSFORMATION`, `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`) — a transform tree
out of scope (multiple co-equal root solids are NOT a decline; they import as a Compound per
T2); (iii) a referenced entity has an unsupported keyword or a surface kind outside
{`PLANE`,`CYLINDRICAL_SURFACE`,`CONICAL_SURFACE`,`SPHERICAL_SURFACE`,`B_SPLINE_SURFACE_WITH_KNOTS`}
— explicitly INCLUDING `TOROIDAL_SURFACE` and `SURFACE_OF_REVOLUTION` — or a curve kind
outside {`LINE`,`CIRCLE`,`ELLIPSE`,`B_SPLINE_CURVE_WITH_KNOTS`}, or a rational (weighted)
B-spline wrap; (iv) a non-millimetre length-unit context (no silent rescale); or (v) a
malformed / dangling record. The tolerance SHALL NEVER be widened to force a pass; the honest
residual SHALL be reported, not hidden.

#### Scenario: A file whose B-rep cannot form a watertight solid returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap so the assembled shell is a genuinely open shell, read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and the engine self-verifies it
- THEN the result SHALL NOT self-verify watertight AND the import SHALL DECLINE (NULL) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: A TOROIDAL_SURFACE or nested-assembly file returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE` (no native surface kind) / `SURFACE_OF_REVOLUTION` / rational B-spline, OR a nested product-structure assembly entity (e.g. `NEXT_ASSEMBLY_USAGE_OCCURRENCE`), read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid, so the engine can fall through to OCCT — no torus surface is faked (the tessellator is not modified) and no assembly transform tree is silently flattened

### Requirement: Native STEP import is native-else-fallback, self-verified, guarded by OCCT

`NativeEngine::step_import(path)` SHALL first call `step_import_native(path)`. When it returns
a non-null shape, the engine SHALL **self-verify** it — for a `Solid`, a valid watertight
solid with enclosed volume > 0 (the native tessellate self-verify already used by the healer
and booleans); for a `Compound` (T2), EVERY member `Solid` SHALL independently self-verify
watertight with enclosed volume > 0 (a non-empty compound; any leaky member fails the whole
import) — and, on success, wrap it as a native `EngineShape` and return the tracked handle.
When `step_import_native` returns a NULL Shape (DECLINE) OR the self-verify FAILS, the engine
SHALL fall through to OCCT `STEPControl_Reader` (labelled), re-reading the SAME file from
scratch — never handing a native void to OCCT. The native reader and the OCCT fallback SHALL
keep OCCT behind `CYBERCAD_HAS_OCCT`; `src/native/**` SHALL contain zero OCCT
includes/symbols. `NativeEngine::iges_export`, `NativeEngine::iges_import`, and
`NativeEngine::step_export` SHALL remain UNCHANGED. This SHALL NOT change the `cc_*` ABI and
SHALL NOT change the default engine (stays OCCT).

#### Scenario: A native STEP file imports natively and matches OCCT (sim vs OCCT — the correctness gate)
- GIVEN a file the native STEP writer produced, on a booted iOS simulator (OCCT linked), with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid, watertight solid AND its volume / bounding box SHALL match the OCCT `STEPControl_Reader` import within tolerance

#### Scenario: A foreign OCCT-written multi-solid STEP imports natively as a compound (T2, sim vs OCCT)
- GIVEN an OCCT-built `TopoDS_Compound` of two disjoint solids written to a STEP file by OCCT `STEPControl_Writer`, on a booted iOS simulator
- WHEN `cc_step_import(path)` imports that foreign file with the native engine active (`cc_set_engine(1)`)
- THEN the native import SHALL return a native shape whose two member solids each self-verify valid + watertight AND whose per-solid centroid / volume / bounding box / count match the OCCT `STEPControl_Reader` re-import within tolerance, proving multi-solid import parity

#### Scenario: An out-of-scope file (torus / nested assembly) falls through to OCCT (sim vs OCCT)
- GIVEN a foreign OCCT-authored STEP with a `TOROIDAL_SURFACE` face (or a nested assembly), with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND the file SHALL be imported by OCCT `STEPControl_Reader` identical to `cc_set_engine(0)`, proving fall-through with no native interception and no fabricated geometry

## ADDED Requirements

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
