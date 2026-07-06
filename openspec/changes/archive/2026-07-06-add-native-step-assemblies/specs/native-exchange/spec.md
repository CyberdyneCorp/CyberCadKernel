# native-exchange

This change (Phase 4 #7) **widens the working native STEP import reader** (landed by
`add-native-step-import` and widened by `widen-native-step-import`) from a **flat** multi-solid
`Compound` to a **placed** `Compound`: it parses the STEP assembly transform tree
(`NEXT_ASSEMBLY_USAGE_OCCURRENCE` /
`REPRESENTATION_RELATIONSHIP[_WITH_TRANSFORMATION]` + `ITEM_DEFINED_TRANSFORMATION`, and/or
`MAPPED_ITEM` + `REPRESENTATION_MAP`), composes each component's **rigid** placement
(rotation + translation, from the `AXIS2_PLACEMENT_3D` pair) into a native
`topology::Location` (`math::Transform`), applies it per component `MANIFOLD_SOLID_BREP` via
`Shape::located()`, and returns a placed `Compound`. It also **confirms + regression-pins**
that AP214 (`AUTOMOTIVE_DESIGN`) / AP242 `FILE_SCHEMA` headers are accepted (the reader was
already schema-independent — it enters at `DATA;` and never gated on the schema string). No
`cc_*` ABI change; the default engine stays OCCT. The STEP writer (`step_writer.cpp`) and the
tessellator are NOT modified. The native topology already models a placed sub-shape (a located
node resolves world coordinates by composing its `Location` down the graph), so a placed solid
tessellates + self-verifies watertight unchanged — no tessellator arm is added.

> NOTE (honest scope): this WIDENS the working import slice to **rigid, single-level
> assemblies of in-slice solids**; it is NOT a general product-structure importer. A
> **non-rigid / scaled / mirrored** placement transform, a **deep multi-level nested** product
> structure, an **external part reference**, a component whose geometry is out of the import
> slice (`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`, rational/weighted B-splines, `BEZIER`),
> a placement chain the reader **cannot compose**, PMI / GD&T / colours / names, non-mm units,
> and non-manifold / unhealable component B-reps all still DECLINE → OCCT
> `STEPControl_Reader`. IGES import/export stay OCCT `IGESControl_*`. A general native
> STEP/AP242 reader + IGES + a general-curved kernel still block #8 `drop-occt`; this change
> does NOT unblock it. **No placement or solid is ever fabricated: the reader composes only
> the transforms the file carries and maps only the solids it genuinely supports; a structure
> it cannot compose to a rigid placement for every component DECLINES rather than defaulting a
> component to identity or inventing geometry. No tolerance is weakened.**

## MODIFIED Requirements

### Requirement: Native STEP AP203 import of a writer-emitted manifold-solid-brep

The native exchange library SHALL provide `step_import_native(path)` that reads an
ISO-10303-21 (STEP Part 21) file — **independently of its `FILE_SCHEMA` header** (AP203,
AP214 `AUTOMOTIVE_DESIGN`, or AP242 are all accepted; the reader gates on entities + the unit
context, not the schema string) — and reconstructs a native `topology::Shape`: a `Solid` when
the file has exactly one root `MANIFOLD_SOLID_BREP`; a **flat** `Compound` of `Solid`s when it
has more than one co-equal root `MANIFOLD_SOLID_BREP` with **no** assembly transform tree; or a
**placed** `Compound` of `Solid`s when the file is a **rigid, single-level assembly** — each
component `MANIFOLD_SOLID_BREP` reconstructed at its component-local coordinates then placed by
the composed rigid transform (rotation + translation) the file's transform tree carries. The
faces SHALL carry only surfaces of kind `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`,
`SPHERICAL_SURFACE`, or `B_SPLINE_SURFACE_WITH_KNOTS` (non-rational) and the edges only curves
of kind `LINE`, `CIRCLE`, `ELLIPSE`, or `B_SPLINE_CURVE_WITH_KNOTS` (non-rational). The reader
SHALL (a) tokenize the DATA section into a `map<#id, Record>` handling integer refs `#M`, reals
including typed forms (`1.`, `1.E2`, `-3.5E-07`), strings (`'...'` with embedded `''`), enums
(`.T.` / `.PLANE.`), lists `( ... )`, `$` (null), `*` (derived), and combined-instance
`( SUB(...) SUB(...) )` records; (b) resolve leaf geometry — `CARTESIAN_POINT` →
`math::Point3` in **millimetres**, `DIRECTION` → `math::Dir3`, `AXIS2_PLACEMENT_3D` →
`math::Ax3`, the in-scope curves → `EdgeCurve` (including
`ELLIPSE('',#position,semiAxis1,semiAxis2)` → `EdgeCurve::Kind::Ellipse` with
`radius = semiAxis1` along frame X and `minorRadius = semiAxis2` along frame Y), the in-scope
surfaces → `FaceSurface` (knots RLE-expanded from the `(mults),(knots)` pair, surface poles
read row-major, U outer); (c) build topology following refs — `VERTEX_POINT` → vertex,
`EDGE_CURVE` → one shared edge per `#id`, `ORIENTED_EDGE` → the oriented shared edge,
`EDGE_LOOP` → wire, `FACE_OUTER_BOUND`/`FACE_BOUND` + `ADVANCED_FACE` sense → face,
`CLOSED_SHELL`/`MANIFOLD_SOLID_BREP` → shell/solid (all roots when there are several) —
dropping the writer's periodic-wall SEAM edge (an `EDGE_CURVE` referenced by two opposite-sense
`ORIENTED_EDGE`s within one loop); and (d) **when a transform tree is present**, resolve each
component's rigid placement by reading `ITEM_DEFINED_TRANSFORMATION('',desc,#from,#to)` (the
map carrying the `#from` `AXIS2_PLACEMENT_3D` frame onto the `#to` frame,
`T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`) and/or `MAPPED_ITEM` + `REPRESENTATION_MAP`
(`T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹`), associating each via
`NEXT_ASSEMBLY_USAGE_OCCURRENCE` / `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` /
`REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` with its component representation's root
`MANIFOLD_SOLID_BREP`(s), then applying `Shape::located(Location{T})` per component solid. Each
composed placement SHALL be verified **rigid** (orthonormal linear part with det ≈ +1); a
non-rigid / scaled / mirrored transform SHALL DECLINE. A `TOROIDAL_SURFACE` face SHALL DECLINE
(there is no native `FaceSurface::Kind::Torus` and the tessellator is not modified), and an
assembly structure the reader cannot compose to a rigid placement for every component (a
non-rigid transform, an unmapped root, a deep-nested / external-ref structure) SHALL DECLINE.
This reader SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. It SHALL NOT modify the STEP writer or the tessellator, and SHALL NOT
fabricate a placement or a solid the file does not describe.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A native-written cylinder imports back with correct quadric geometry (host round-trip)
- GIVEN a native-built cylinder / capped-cylinder `Solid` (a `CYLINDRICAL_SURFACE` lateral face + `PLANE` caps bounded by `CIRCLE` edges) serialized by `step_export_native`, on the host with no OCCT
- WHEN `step_import_native` reads it back
- THEN the lateral face SHALL reconstruct a `FaceSurface::Kind::Cylinder` (frame + radius in mm), the cap boundaries `EdgeCurve::Kind::Circle` AND the writer's full-turn-wall SEAM edge SHALL be dropped so the reconstructed edge count matches the native source EXACTLY AND the solid SHALL be valid + watertight after `healShell`

#### Scenario: An ELLIPSE edge maps to the native ellipse curve kind (host)
- GIVEN an in-scope ISO-10303-21 buffer whose edge carries an `ELLIPSE('',#position,semiAxis1,semiAxis2)` curve, read on the host with no OCCT
- WHEN `step_import_native` resolves the edge's curve
- THEN it SHALL reconstruct an `EdgeCurve::Kind::Ellipse` with `frame` from `#position`, `radius == semiAxis1` (semi-major, frame X), and `minorRadius == semiAxis2` (semi-minor, frame Y), AND a degenerate ellipse (either semi-axis ≤ 0 or non-finite) SHALL DECLINE (NULL) rather than reconstruct a wrong curve

#### Scenario: Multiple co-equal root solids import as a flat compound (host)
- GIVEN an in-scope ISO-10303-21 buffer with two co-equal root `MANIFOLD_SOLID_BREP`s (each a `CLOSED_SHELL` of in-scope faces) and no assembly transform entity, read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a `Compound` containing two `Solid`s (one per root) at their world coordinates, each reconstructed exactly as the single-solid path would, AND a buffer with exactly one root SHALL still return a bare `Solid` (the single-solid + flat multi-solid behaviour is unchanged)

#### Scenario: A rigid assembly imports as a placed compound (host)
- GIVEN an in-scope ISO-10303-21 buffer describing a single-level assembly of two components — each a root `MANIFOLD_SOLID_BREP` placed by an `ITEM_DEFINED_TRANSFORMATION` (an `AXIS2_PLACEMENT_3D` pair) carried by a `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` / `NEXT_ASSEMBLY_USAGE_OCCURRENCE`, or by a `MAPPED_ITEM` + `REPRESENTATION_MAP` — with rigid (rotation + translation) placements, read on the host with no OCCT
- WHEN `step_import_native` composes the transform tree
- THEN it SHALL return a `Compound` of two `Solid`s each placed by the composed rigid `Location` (each component's world centroid at the placed position, NOT at the origin), reconstructing each component solid at its local coordinates then applying `Shape::located(Location{T})`, AND SHALL invent no placement (a component with no resolvable placement DECLINEs the whole file)

#### Scenario: A non-rigid or uncomposable assembly declines (host)
- GIVEN an ISO-10303-21 assembly buffer whose component placement is a scaled / mirrored (non-rigid) transform, OR whose transform tree leaves a root `MANIFOLD_SOLID_BREP` reached by no placement, read on the host with no OCCT
- WHEN `step_import_native` attempts to compose the transform tree
- THEN it SHALL return a NULL Shape (DECLINE) — the non-rigid transform fails the `isRigid` gate and the unmapped root fails the "every root placed exactly once" gate — without constructing any solid, so the engine can fall through to OCCT; no component is silently placed at identity and no volume-changing scale is applied

#### Scenario: An AP214 / AP242 header with in-slice entities imports (host)
- GIVEN an ISO-10303-21 buffer whose `FILE_SCHEMA` header names AP214 (`AUTOMOTIVE_DESIGN`) or AP242, and whose DATA entities + unit context are all in the import slice, read on the host with no OCCT
- WHEN `step_import_native` parses it
- THEN it SHALL import it exactly as it would an AP203 file (the reader is schema-independent — it enters at `DATA;` and never rejects on the schema string)

#### Scenario: A native-written B-spline-surfaced solid imports its B_SPLINE entities (host round-trip)
- GIVEN a native-built solid with a `B_SPLINE_SURFACE_WITH_KNOTS` face and `B_SPLINE_CURVE_WITH_KNOTS` edges serialized by `step_export_native`, on the host with no OCCT
- WHEN `step_import_native` reads it back
- THEN the face SHALL reconstruct a `FaceSurface::Kind::BSpline` with the same degrees, control-point grid (row-major), and RLE-expanded knot vectors, and its boundary curves `EdgeCurve::Kind::BSpline`, AND the round-tripped solid's volume SHALL match the original within analytic tolerance

### Requirement: Native STEP import runs healShell and returns NULL for out-of-scope or unhealable files

`step_import_native` SHALL rely on the shared-node reconstruction (adjacent faces referencing
the same `EDGE_CURVE` node by `#id`, watertight by construction) and SHALL return the assembled
`Solid` / flat `Compound` / **placed `Compound`** for the engine to self-verify; the
`heal::healShell` deferral described by `add-native-step-import` applies where a file carries
independent per-face edges. A placed component solid SHALL be reconstructed at its local
coordinates then rigidly re-placed by `Shape::located()` (a rotation + translation preserves
the watertight 2-manifold), so the engine self-verifies the WORLD-placed solid via the
Location-resolving accessors. The reader SHALL return a **NULL Shape (DECLINE)** — and never a
partial or invented solid — when ANY of: (i) the assembled shell is a genuinely open /
non-manifold B-rep that cannot form a watertight solid (surfaced as a failed engine
self-verify), or a placed member fails the self-verify; (ii) the file has **zero** root
`MANIFOLD_SOLID_BREP`, OR carries an assembly transform tree the reader **cannot compose** to a
rigid placement for every component — a **non-rigid / scaled / mirrored**
`ITEM_DEFINED_TRANSFORMATION` / `MAPPED_ITEM` transform, a root `MANIFOLD_SOLID_BREP` reached by
no placement, or a **deep multi-level nested** / **external-reference** product structure
(multiple co-equal root solids with no transform tree are NOT a decline — they import as a flat
Compound; a single-level RIGID assembly is NOT a decline — it imports as a placed Compound);
(iii) a referenced entity has an unsupported keyword or a surface kind outside
{`PLANE`,`CYLINDRICAL_SURFACE`,`CONICAL_SURFACE`,`SPHERICAL_SURFACE`,`B_SPLINE_SURFACE_WITH_KNOTS`}
— explicitly INCLUDING `TOROIDAL_SURFACE` and `SURFACE_OF_REVOLUTION`, in ANY component — or a
curve kind outside {`LINE`,`CIRCLE`,`ELLIPSE`,`B_SPLINE_CURVE_WITH_KNOTS`}, or a rational
(weighted) B-spline wrap; (iv) a non-millimetre length-unit context (no silent rescale); or (v)
a malformed / dangling record. The tolerance SHALL NEVER be widened to force a pass; the honest
residual SHALL be reported, not hidden.

#### Scenario: A file whose B-rep cannot form a watertight solid returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap so the assembled shell is a genuinely open shell, read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and the engine self-verifies it
- THEN the result SHALL NOT self-verify watertight AND the import SHALL DECLINE (NULL) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: A TOROIDAL_SURFACE or out-of-slice component returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE` (no native surface kind) / `SURFACE_OF_REVOLUTION` / rational B-spline — as a lone solid OR as one component of an assembly — read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid (the whole assembly declines — no partial import), so the engine can fall through to OCCT — no torus surface is faked (the tessellator is not modified)

#### Scenario: A non-rigid or uncomposable assembly transform returns NULL (host)
- GIVEN an ISO-10303-21 assembly buffer whose component placement is non-rigid (scaled / mirrored), OR whose transform tree leaves a root `MANIFOLD_SOLID_BREP` unplaced, OR which is a deep multi-level nested / external-reference product structure the reader cannot compose, read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid, so the engine can fall through to OCCT — no assembly transform tree is silently flattened, no component is placed at identity, and no non-rigid transform is applied

### Requirement: Native STEP import is native-else-fallback, self-verified, guarded by OCCT

`NativeEngine::step_import(path)` SHALL first call `step_import_native(path)`. When it returns
a non-null shape, the engine SHALL **self-verify** it — for a `Solid`, a valid watertight solid
with enclosed volume > 0 (the native tessellate self-verify already used by the healer and
booleans); for a `Compound` (flat OR placed), EVERY member `Solid` SHALL independently
self-verify watertight with enclosed volume > 0 (a non-empty compound; any leaky member fails
the whole import) — and, on success, wrap it as a native `EngineShape` and return the tracked
handle. A **placed** member solid SHALL be self-verified in its WORLD placement (the accessors
resolve the composed `Location`), so a rigid transform preserves the watertight check and no
new engine gate is required (`robustlyWatertightImport` already per-member-verifies a Compound).
When `step_import_native` returns a NULL Shape (DECLINE) OR the self-verify FAILS, the engine
SHALL fall through to OCCT `STEPControl_Reader` (labelled), re-reading the SAME file from
scratch — never handing a native void to OCCT. The native reader and the OCCT fallback SHALL
keep OCCT behind `CYBERCAD_HAS_OCCT`; `src/native/**` SHALL contain zero OCCT includes/symbols.
`NativeEngine::iges_export`, `NativeEngine::iges_import`, and `NativeEngine::step_export` SHALL
remain UNCHANGED. This SHALL NOT change the `cc_*` ABI and SHALL NOT change the default engine
(stays OCCT).

#### Scenario: A native STEP file imports natively and matches OCCT (sim vs OCCT — the correctness gate)
- GIVEN a file the native STEP writer produced, on a booted iOS simulator (OCCT linked), with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a valid, watertight solid AND its volume / bounding box SHALL match the OCCT `STEPControl_Reader` import within tolerance

#### Scenario: A foreign OCCT-written flat multi-solid STEP imports natively as a compound (sim vs OCCT)
- GIVEN an OCCT-built `TopoDS_Compound` of two disjoint solids written to a STEP file by OCCT `STEPControl_Writer` (no transform tree), on a booted iOS simulator
- WHEN `cc_step_import(path)` imports that foreign file with the native engine active (`cc_set_engine(1)`)
- THEN the native import SHALL return a native shape whose two member solids each self-verify valid + watertight AND whose per-solid centroid / volume / bounding box / count match the OCCT `STEPControl_Reader` re-import within tolerance, proving flat multi-solid import parity

#### Scenario: A foreign OCCT-authored rigid assembly imports natively as a placed compound (sim vs OCCT)
- GIVEN an OCCT-authored 2-component assembly (two boxes at distinct rigid placements via the STEP transform tree, written by `STEPControl_Writer` on a compound of transformed solids or by `STEPCAFControl_Writer` on an XCAF assembly document), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose two member solids each self-verify valid + watertight AND whose solid COUNT, TOTAL volume, and per-solid bounding box + centroid/placement match the OCCT re-import within tolerance, proving rigid-assembly placement parity

#### Scenario: An out-of-scope file (torus / non-rigid or deep-nested assembly) falls through to OCCT (sim vs OCCT)
- GIVEN a foreign OCCT-authored STEP with a `TOROIDAL_SURFACE` face, or an assembly with a component out of slice / a scaled instance / a deep-nested structure, with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND the file SHALL be imported by OCCT `STEPControl_Reader` identical to `cc_set_engine(0)`, proving fall-through with no native interception and no fabricated geometry

## ADDED Requirements

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
