# native-exchange

This change (Phase 4 #7, the first native IMPORT slice) extends the `native-exchange`
capability: it makes **`cc_step_import`** NATIVE in `NativeEngine` for the **AP203
`MANIFOLD_SOLID_BREP` subset the native writer (and OCCT `STEPControl_Writer`) emit** for
planar + elementary-quadric + basic B-spline solids. The native reader tokenizes an
ISO-10303-21 (STEP Part 21) file into an entity table, maps the writer's entity set back to
native geometry/topology (the inverse of the export §3 map), drops the periodic-wall seam
artefact, and runs `heal::healShell` to close the file's sub-tolerance gaps into a
watertight native `topology::Shape` solid. A file OUTSIDE the subset — an unsupported
entity/surface, an assembly, multiple roots, a non-manifold shell, or a B-rep `healShell`
cannot heal to a valid watertight solid — makes the reader return a **NULL / undecided
result**, and the engine falls through to OCCT `STEPControl_Reader` (labelled, verified,
never faked). No `cc_*` ABI change; the default engine stays OCCT. The STEP writer
(`step_writer.cpp`) and the tessellator are NOT modified — import reads what the writer
already produces.

> NOTE (honest scope): this is the **first native import slice**, scoped to the
> writer-emitted AP203 manifold-solid-brep subset. **Arbitrary / foreign-schema STEP import
> stays OCCT** `STEPControl_Reader` (the full Part-42 + AP203/AP214 + AP242 entity schema,
> external references, PMI, assemblies, non-conforming foreign files, freeform
> re-approximation — the large, long-lived part, out of scope). **AP242 / PMI / assemblies /
> multiple roots / non-manifold / unsupported surfaces** are out of scope and DECLINE to
> OCCT. **IGES export/import (`cc_iges_export` / `cc_iges_import`) stay OCCT**
> `IGESControl_*`. It is fully acceptable that ONLY the writer-emitted subset imports
> natively and everything else is OCCT; this spec states that split truthfully. A general
> native STEP/AP242 reader + IGES + a general-curved kernel are what still block #8
> `drop-occt` — this change does NOT unblock it. **No geometry is ever fabricated: the
> reader parses only what is in the file and maps only what it genuinely supports.**

## ADDED Requirements

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

This change SHALL NOT add any native code path for `cc_iges_export` or `cc_iges_import`;
`NativeEngine::iges_export` and `NativeEngine::iges_import` SHALL remain unconditional
fall-throughs to the OCCT engine under both engine settings. This change SHALL NOT modify
the native STEP writer (`step_writer.cpp`) or the tessellator — the reader inverts what the
writer already produces. Arbitrary / AP242 STEP import and IGES (a separate older format)
remain OUT OF SCOPE and OCCT-backed; the spec SHALL state this split truthfully: native STEP
import of the writer-emitted AP203 manifold-solid-brep subset is the first import slice, and
a general STEP/AP242 reader + IGES + a general-curved kernel are what still block #8
`drop-occt`.

#### Scenario: IGES import/export are identical under both engines (parity)
- GIVEN a solid and an IGES file on a booted iOS simulator
- WHEN `cc_iges_export(body, path)` and `cc_iges_import(path)` are called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the results SHALL be identical under both engines (IGES stays OCCT `IGESControl_*`; the native engine intercepts neither)

#### Scenario: The STEP writer and tessellator are byte-for-byte unchanged
- GIVEN this change applied
- WHEN `step_export_native` serializes a native solid and the tessellator meshes it
- THEN their output SHALL be identical to before this change (import is additive; it reads what the writer produces and does not alter the writer or the tessellator)

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
