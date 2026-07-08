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
`Compound` (several co-equal roots, no transform tree), or a **placed** `Compound` (a single- OR
**multi-level (nested)** assembly composed by walking the `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` /
`REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` relationship chain from each leaf shape-representation to
its UNIQUE root, composing the per-level rigid / uniform-scale / mirror transforms into ONE world placement
per leaf, else DECLINE — a `MAPPED_ITEM` / `REPRESENTATION_MAP`, a cyclic / ambiguous / dangling chain, or a
non-conformal composed transform declines — with mirror orientation-compensation so each placed solid
self-verifies watertight; the tessellator SHALL NOT be modified and no normal SHALL be fabricated). The faces SHALL carry surfaces of kind `PLANE`,
`CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS`
(non-rational), a **`TOROIDAL_SURFACE`** (→ native `Kind::Torus`), or a **`SURFACE_OF_REVOLUTION`** that
maps to a native surface — a straight generatrix **parallel** (→ cylinder), **oblique-meeting** (→ cone),
or **perpendicular** (→ plane); an **on-axis circle / arc** (→ sphere); an **off-axis circle / arc** (→
**torus**, `Kind::Torus`); or an **ellipse / non-rational B-spline** profile (→ a **rational `Kind::BSpline`**
revolved surface, the exact revolved rational tensor-product B-spline, gated by the watertight self-verify)
— and the edge curves of kind `LINE`, `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (non-rational), or a
**`TRIMMED_CURVE`** whose basis is one of those. A full periodic **sphere** face OCCT emits as a single
seam+double-pole face SHALL be reconstructed as a native `Sphere` **bare periodic surface** (NULL outer
wire) as before. A full periodic **torus** face (doubly periodic, NO pole) SHALL be reconstructed watertight
as a native `Torus` bare-periodic face as before. Specifically:

- **A `TRIMMED_CURVE`** SHALL be unwrapped to its basis curve (recursively; B-spline basis takes its
  `[first,last]` from the `PARAMETER_VALUE` trims clamped to the clamped knot span, analytic basis keeps
  the vertex-derived range) exactly as the landed trimmed-curve slice does (unchanged).
- **A `TOROIDAL_SURFACE`** `('',#axis2placement, major_radius, minor_radius)` SHALL be mapped by resolving
  the `AXIS2_PLACEMENT_3D` frame and the two trailing reals and building a native `FaceSurface` of kind
  `Torus` (`radius` = major, `minorRadius` = minor), reconstructed watertight (unchanged).
- **A `SURFACE_OF_REVOLUTION`** `('',#profile,#axis1)` SHALL be mapped by resolving the axis
  (`AXIS1_PLACEMENT` — origin + one direction, `$` axis → +Z) and the **profile** curve, then classifying
  the profile + axis by MEASUREMENT (never by a keyword) and mapping it to the EXACT native surface it
  sweeps, VERIFIED to pass through the profile within a scale-relative tolerance before emission:
  - a straight `LINE` **parallel** → native `Cylinder`; **oblique meeting** the axis → native `Cone`;
    **perpendicular** → native `Plane` (all landed, unchanged);
  - a `CIRCLE` / arc **centred ON the axis** with its plane **containing the axis** → native `Sphere`
    (landed, unchanged);
  - a `CIRCLE` / arc **centred OFF the axis** whose plane admits a ring torus → a native **`Torus`**
    (landed, unchanged);
  - an **`ELLIPSE`** or a **non-rational `B_SPLINE_CURVE_WITH_KNOTS`** profile → a native **rational
    `Kind::BSpline`** surface built as the EXACT revolved rational tensor-product B-spline: the `u`
    direction is the standard rational-quadratic full circle (`degreeU = 2`, **9** control poles, rational
    weights `{1, 1/√2, 1, 1/√2, 1, 1/√2, 1, 1/√2, 1}` — the on-circle poles at the quadrant angles
    `0, π/2, π, 3π/2, 2π` weight `1`, the four in-between corner poles weight `cos(45°) = 1/√2` — with knot
    vector `{0,0,0, π/2,π/2, π,π, 3π/2,3π/2, 2π,2π,2π}`); the `v` direction is the profile's own
    representation (an ellipse promoted to its exact rational-quadratic B-spline; a non-rational B-spline
    used directly); the tensor pole `P_ij` places the `i`-th revolution-circle control point at the `j`-th
    profile pole's axial height + radius, and the tensor weight `w_ij = w^u_i · w^v_j`. It SHALL be emitted
    ONLY when the sampled profile points lie on the reconstructed surface at `u=0` within a scale-relative
    tolerance AND the assembled face self-verifies watertight (the `u=0≡2π` seam welds; a profile-endpoint
    axis pole closes through the EXISTING rational-B-spline mesh path).

  In **every** other case, AND whenever the mapped revolved B-spline face does not reconstruct watertight
  (a leaky `u`-seam, an unclosable profile-endpoint axis pole) or would require perturbing an existing
  tessellation path to close, the reader SHALL DECLINE (NULL → OCCT): a `CIRCLE` whose plane does not admit
  a ring torus (degenerate), a **skew** oblique `LINE` (a hyperboloid of one sheet — no native kind), a
  `LINE` **on** the axis (degenerate), a **degenerate axis**, a **rational (weighted)** B-spline profile (the
  curve reader is non-rational only), a profile whose revolution is not faithfully representable, and any
  mapped face that fails the faithful-reduction guard or the watertight self-verify.

The reader SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. It SHALL prefer to leave the tessellator UNCHANGED (the rational `Kind::BSpline` mesh
path already exists); it SHALL touch the tessellator ONLY through a STRICTLY ADDITIVE branch proven
byte-identical for every existing mesh, and otherwise SHALL keep the revolution's OCCT decline. It SHALL
prefer to leave the STEP writer unchanged (OCCT-authored fixtures), SHALL NOT import PMI / annotation
entities as geometry, and SHALL NOT fabricate a curve, a surface, a trim, a placement, or a solid the file
does not describe, nor weaken any tolerance, nor commit any dead reconstruction code.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A 2-level nested rigid assembly composes each leaf's world placement by walking the relationship chain (host)
- GIVEN an in-scope ISO-10303-21 buffer describing a NESTED assembly — a leaf `MANIFOLD_SOLID_BREP` placed into a SUB-assembly shape-representation by a rigid `ITEM_DEFINED_TRANSFORMATION` `T₂` (one `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`), and that sub-assembly placed into the ROOT shape-representation by a rigid `T₁` (a second `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`), read on the host with no OCCT
- WHEN `step_import_native` walks the relationship chain from the leaf shape-representation to its unique root
- THEN it SHALL return a placed `Compound` whose leaf `Solid` is located by the COMPOSED world transform `W = T₁ ∘ T₂` (its world centroid at `W` applied to the leaf-local centroid — NOT at `T₂` alone and NOT at the origin), the composition matching an INDEPENDENT matrix multiplication of the two frame-pair transforms read from the file, AND a single-level chain (length 1) SHALL still compose to exactly today's placement (the landed single-level path is byte-identical)

#### Scenario: A SURFACE_OF_REVOLUTION of an ellipse / B-spline profile maps to a native rational B-spline or declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is an `ELLIPSE` or a non-rational `B_SPLINE_CURVE_WITH_KNOTS`, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL build the EXACT revolved rational tensor-product B-spline (`Kind::BSpline`, `degreeU=2`, 9 `u`-poles, weights `{1,1/√2,…}`, the standard revolution knots, tensored with the profile in `v`), VERIFIED sampled profile points lie on the surface at `u=0`, AND — when the surface is faithfully representable and self-verifies watertight — the assembled solid SHALL be valid + watertight and match the OCCT re-import within tolerance; OTHERWISE it SHALL return a NULL Shape (DECLINE) so the engine falls through to OCCT — never a mangled, approximate, or non-watertight surface, and never dead reconstruction code

#### Scenario: A revolved B-spline face that leaves a seam or pole gap declines to OCCT (host)
- GIVEN an in-scope ISO-10303-21 buffer whose ellipse / B-spline revolution reconstructs a rational `Kind::BSpline` face whose `u=0≡2π` seam does not weld, or whose profile-endpoint axis pole does not close, through the existing mesh path
- WHEN `step_import_native` assembles the solid and the engine self-verifies it
- THEN the import SHALL DECLINE (NULL) — keeping the honest OCCT fallback — never a leaky or fabricated face, never a tessellator perturbed to force the close, and the tolerance SHALL NOT be widened

#### Scenario: The circle / line / torus reductions and prior slices are unchanged (host)
- GIVEN in-scope ISO-10303-21 buffers exercising a `LINE` parallel / oblique-meeting / perpendicular, an on-axis `CIRCLE` (→ sphere), and an off-axis `CIRCLE` (→ torus) `SURFACE_OF_REVOLUTION`, plus the trimmed-curve, full-sphere / full-torus bare-periodic, quadric, bspline-face, and rigid / uniform-scale / mirror assembly cases, read on the host with no OCCT
- WHEN `step_import_native` resolves each
- THEN the cylinder / cone / plane / sphere / torus reductions and every prior import path SHALL behave EXACTLY as before (the ellipse / B-spline revolution arm is additive; the topology, math, and tessellator are unchanged)

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
world normals point inward yields a negative enclosed volume and FAILS the self-verify → OCCT. A
**multi-level (nested)** placed member SHALL be self-verified identically — its `Location` is the composed
leaf→root relationship chain, and a rigid / uniform-scale / mirror composition still preserves (or, for a
reflection, compensates to preserve) the outward watertight solid with the correct POSITIVE enclosed volume.
When
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

#### Scenario: An out-of-scope file (torus / non-uniform-scale or a MAPPED_ITEM / cyclic assembly) falls through to OCCT (sim vs OCCT)
- GIVEN a foreign OCCT-authored STEP with a `TOROIDAL_SURFACE` face, or an assembly with a component placed by a non-uniform-scale / shear transform, or a `MAPPED_ITEM` / `REPRESENTATION_MAP` (Form-B) instance, or a cyclic / ambiguous relationship chain the reader cannot compose to a unique root, with the native engine active (`cc_set_engine(1)`)
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

### Requirement: Native STEP import general-revolution (ellipse / B-spline) mapping verified vs OCCT with no tessellation perturbation

The general-revolution (T2) widening SHALL be verified by (a) **host** unit / decline cases (OCCT-free): an
`ELLIPSE` (and, where OCCT authors one, a non-rational `B_SPLINE_CURVE_WITH_KNOTS`) `SURFACE_OF_REVOLUTION`
maps to a native rational `FaceSurface::Kind::BSpline` built as the exact revolved rational tensor-product
B-spline (the standard rational-quadratic full circle in `u` tensored with the profile in `v`), VERIFIED
sampled profile points lie on the surface, and — when the reconstructed periodic B-spline self-verifies
watertight — the assembled solid is valid + watertight with a volume / bbox matching the analytic revolved
solid (e.g. a spheroid of revolution); ELSE an honest DECLINE (NULL) with no dead reconstruction code; and
the on-axis-circle → sphere, off-axis-circle → torus, line → cylinder/cone/plane, trimmed-curve, quadric,
bspline-face, and rigid / uniform-scale / mirror assembly cases STILL pass byte-identical. And (b) a
**simulator sim-vs-OCCT** gate (OCCT linked) through the `cc_*` facade under `cc_set_engine(1)`: a FOREIGN
OCCT-authored general-revolution (ellipse profile) solid imports natively as a rational B-spline and matches
the OCCT re-import (count / volume / watertight / bbox) IF T2 lands, else DECLINES natively and imports via
OCCT identical to `cc_set_engine(0)`. And (c) a **no-tessellation-perturbation guarantee**: the change SHALL
touch neither `face_mesher.h`, `surface_eval.h`, nor `trim.h` (the rational B-spline mesh path already
exists) UNLESS a periodic-seam / axis-pole close is genuinely required AND added as a STRICTLY ADDITIVE
branch proven byte-identical for every existing mesh across the full tessellation-sensitive suite
(`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3); if that additive,
byte-identical bar cannot be met, T2 SHALL keep the OCCT decline. The parity test SHALL restore the OCCT
default in teardown and SHALL carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite
assertion count is unchanged. Every existing suite (host CTest, GPU / Phase-3) and every prior native
capability (STEP export, the flat multi-solid + ELLIPSE + bspline-face + assembly + AP242 + trimmed-curve +
revolution-quadric + full-sphere + torus import slices, shape healing, SSI S1–S5, native blends + #6/#7,
marching, boolean, construct, tessellation) SHALL stay green at the OCCT default with no regression. No
tolerance SHALL be weakened.

#### Scenario: A foreign OCCT-authored general-revolution solid imports natively or declines to OCCT (sim)
- GIVEN an OCCT-authored solid whose face is an ellipse-profile (or non-rational-B-spline-profile) `SURFACE_OF_REVOLUTION`, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN — if T2 landed — the native import SHALL return a valid + watertight solid whose COUNT, volume, watertightness, and bounding box match the OCCT re-import within tolerance; OTHERWISE `step_import_native` SHALL return NULL and OCCT SHALL import the file identical to `cc_set_engine(0)`, proving the honest general-revolution fallback

#### Scenario: The general-revolution arm perturbs no existing tessellation (host + sim)
- GIVEN this change applied, with the rational `Kind::BSpline` revolved face meshed through the existing rational B-spline mesh path, and the full tessellation-sensitive sim set (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3) run on a booted iOS simulator
- WHEN every existing plane / cylinder / cone / sphere / torus / B-spline face is meshed and compared against the pre-change baseline
- THEN the triangle counts, watertight status, and volumes SHALL be IDENTICAL to the baseline (the tessellator is preferred untouched, and any mesher branch is additive-only + byte-identical); if the byte-identical bar cannot be met, T2 SHALL keep the OCCT decline and no existing tessellation SHALL have been perturbed

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and STEP export, the flat multi-solid + ELLIPSE + bspline-face + assembly + AP242 + trimmed-curve + revolution-quadric + full-sphere + torus import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

### Requirement: Sim-verified STEP admission of a foreign trimmed B-spline surface face via a faithful curved-edge pcurve arm and reconstruction guard, else DECLINE

The STEP reader SHALL admit ONE foreign `B_SPLINE_SURFACE_WITH_KNOTS` surface
(rational `weights` included) bounded by a real, genuinely trimmed `EDGE_LOOP` — a
foreign trimmed B-spline/NURBS patch — as a native trimmed `Kind::BSpline` face **only
when a faithful 2D pcurve is reconstructed and verified for EVERY boundary edge**, so
that the landed native trimmed-freeform mesh path (`native-tessellation`,
`trimmedFreeformMesh`) — which SHALL NOT be modified — can mesh it watertight. This
fulfils the STEP-reader admission that the M0 keystone DEFERRED to the OCCT-linked
simulator gate.

For each boundary edge on a `Kind::BSpline` face surface the reader SHALL reconstruct
the pcurve by inverting the surface at the edge's sampled points (`projectBSplineUV`, a
grid-seeded damped-Newton inversion on the analytic surface derivatives — no OCCT, no
external solver): a **straight-in-`(u,v)`** edge (rim / seam / isoparametric trim) SHALL
become a UV `Line` through the two projected endpoints, byte-identical to the reader's
prior generic-linear pcurve for that case; a **genuinely curved** boundary edge SHALL
become a UV `B_SPLINE` pcurve whose 2D poles are densified projected samples and whose
degree and knots are preserved from the 3D edge curve, evaluated as-is by the landed
`trim.h::pcurveValue` `K::BSpline` evaluator.

The reader SHALL then run a **faithful-reconstruction guard** on every boundary edge:
using the SAME `pcurveValue` evaluator the mesher will flatten, and a rational-aware
forward evaluation of the B-spline surface (`math::nurbsSurfacePoint` over the surface
poles / knots / `weights`), it SHALL re-evaluate `S_face(pcurve(t)) = C_edge(t)` at
several parameters across the edge range and require the gap to be within a
**scale-relative** tolerance (`1e-6 · max(1, scale)`, `scale` = the surface control-net
extent) that SHALL NOT be weakened. If ANY boundary edge fails the guard — the surface
inversion did not converge, the reconstructed pcurve does not lie on the patch within
tolerance, or the boundary gap exceeds tolerance — the reader SHALL `decline()` the face
(NULL → OCCT), exactly as it declines any other non-faithful reduction.

An admitted face SHALL be subject to the engine's mandatory watertight + volume/area
self-verify against the OCCT oracle downstream; a native result that is not watertight or
off-volume SHALL be DISCARDED → OCCT, so a wrong or leaky solid is never emitted. The
reader SHALL remain OCCT-free (`src/native/**` links no OCCT / `IEngine` / `EngineShape`
type), the native tessellator SHALL NOT be modified, no tolerance SHALL be weakened, and
the `cc_*` ABI SHALL be unchanged (additive reader behaviour only). The existing Plane /
Cylinder / Cone / Sphere pcurve arms and the bare-periodic full-sphere / full-torus /
full-revolution B-spline admission paths SHALL remain byte-identical.

#### Scenario: A foreign trimmed B-spline face with faithful curved pcurves imports and meshes watertight matching BRepMesh (sim, parity)

- GIVEN a foreign STEP file carrying a `B_SPLINE_SURFACE_WITH_KNOTS` face bounded by a genuinely trimmed `EDGE_LOOP` with a curved boundary edge whose pcurve reconstructs faithfully, imported on a booted iOS simulator with OCCT linked and the native engine active (`cc_set_engine(1)`)
- WHEN the reader reconstructs each boundary pcurve, passes the `S_face(pcurve(t)) = C_edge(t)` guard, admits the face, and the landed M0 tessellator meshes the resulting solid
- THEN the native solid's enclosed volume, surface area, watertight status, triangle envelope, and sub-shape topology SHALL match the OCCT `STEPControl_Reader` re-import + `BRepMesh_IncrementalMesh` oracle within tolerance (the foreign trimmed patch that previously could not be faithfully admitted now meshes watertight)

#### Scenario: A boundary edge whose pcurve does not reconstruct faithfully declines to OCCT (sim)

- GIVEN a foreign STEP file whose trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face has at least one boundary edge whose pcurve cannot be reconstructed within the scale-relative tolerance (non-converging inversion, off-surface reconstructed pcurve, or beyond-tolerance boundary gap)
- WHEN the reader evaluates the faithful-reconstruction guard for that edge
- THEN the reader SHALL `decline()` the face (NULL → OCCT), the file SHALL round-trip through OCCT `STEPControl_Reader` unchanged, no tolerance SHALL have been weakened, and no approximate or leaky native face SHALL be emitted — the honest decline is reported

#### Scenario: The reconstruction guard accepts a faithful pcurve and rejects an off-surface edge (host analytic, no OCCT)

- GIVEN a native-built trimmed `Kind::BSpline` face with a closed-form curved boundary, built on the host with NO OCCT linked, plus a deliberately perturbed off-surface variant of one boundary edge
- WHEN the reader reconstructs each pcurve and runs the `S_face(pcurve(t)) = C_edge(t)` guard (rational eval via `math::nurbsSurfacePoint`)
- THEN the guard SHALL ACCEPT the faithful face (every sampled `t` within `1e-6 · max(1, scale)`, and the meshed solid is watertight with the closed-form enclosed volume within tolerance) AND SHALL REJECT the perturbed variant (`decline()` fires) — proven against a closed-form oracle with no OCCT symbol linked

#### Scenario: The engine self-verify discards a non-watertight admitted face (sim)

- GIVEN a foreign trimmed B-spline face that passes the per-edge pcurve guard but whose native mesh does not close watertight (or whose volume/area does not match the OCCT oracle)
- WHEN the engine runs its mandatory watertight + volume/area self-verify
- THEN the native result SHALL be DISCARDED and the import SHALL fall through to OCCT, so a wrong or leaky mesh is never emitted downstream

#### Scenario: Existing STEP round-trips and non-B-spline pcurve arms are unchanged (zero-regression)

- GIVEN the existing STEP round-trip fixtures (box / cylinder / sphere / native B-spline-wall solids) and the full simulator suite
- WHEN the reader is exercised with the new B-spline-surface pcurve arm and guard in place
- THEN a straight-in-`(u,v)` B-spline-wall edge SHALL emit a pcurve byte-identical to the prior generic-linear reconstruction, the Plane / Cylinder / Cone / Sphere arms and the bare-periodic admission paths SHALL be unchanged, and `run-sim-suite` (221/221) and STEP import (77/77) SHALL stay green

### Requirement: Native STEP admission of a foreign RATIONAL B-spline surface face via the combined RATIONAL_B_SPLINE_SURFACE record, else DECLINE

The STEP reader SHALL admit ONE foreign **rational** B-spline surface — emitted by
OCCT `STEPControl_Writer` as a **combined** Part-21 instance
`( BOUNDED_SURFACE() B_SPLINE_SURFACE(degU, degV, ((#pole)…), form, uClosed, vClosed,
selfInt) B_SPLINE_SURFACE_WITH_KNOTS((uMults), (vMults), (uKnots), (vKnots), spec)
GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((weight)…)) REPRESENTATION_ITEM('')
SURFACE() )` — bounded by a real, genuinely trimmed `EDGE_LOOP`, as a native trimmed
`Kind::BSpline` face carrying surface `weights`, **only when** the rational record is
well-formed AND a faithful 2D pcurve is reconstructed and verified for EVERY boundary
edge, so the landed native trimmed-freeform mesh path (`native-tessellation`,
`trimmedFreeformMesh`) — which SHALL NOT be modified — can mesh it watertight. This
extends the M4 non-rational `B_SPLINE_SURFACE_WITH_KNOTS` admission to rational NURBS
surfaces by reading the surface weights the reader does not read today.

The reader SHALL dispatch the combined record using the existing combined-instance
machinery it already uses for rational curves / assembly relationships (`Record::subs`
sub-record scan) — no new tokenizer. It SHALL read `degU, degV, uClosed, vClosed` and the
row-major (U-outer / V-inner) pole grid from the `B_SPLINE_SURFACE` sub-record, the
RLE-expanded `(uMults, vMults, uKnots, vKnots)` knot vectors from the
`B_SPLINE_SURFACE_WITH_KNOTS` sub-record, and the weight grid from the
`RATIONAL_B_SPLINE_SURFACE` sub-record into `FaceSurface::weights` in the SAME row-major
order as the poles. The shared pole/knot parse SHALL be factored so the non-rational
`B_SPLINE_SURFACE_WITH_KNOTS` keyword path produces a byte-identical `FaceSurface`.

The reader SHALL `decline()` the face (NULL → OCCT) when the rational record is malformed:
the weight grid is ragged, its cardinality does not equal the pole-grid cardinality
(`nPolesU × nPolesV`), or any weight is non-finite or not strictly positive. A weight is
NEVER clamped and no tolerance is introduced by this well-formedness check.

An admitted rational face SHALL flow UNMODIFIED through the existing per-edge pcurve arm
and the **faithful-reconstruction guard**, which SHALL be rational-aware: the guard's
surface evaluation SHALL use `math::nurbsSurfacePoint` over the surface poles / knots /
`weights` (the reader's `bsplineSurfaceValue` already routes to the rational evaluator
when `weights` is non-empty), re-evaluate `S_face(pcurve(t)) = C_edge(t)` at several
parameters within the SAME scale-relative tolerance as the non-rational path, and
`decline()` the face if any boundary edge fails. An admitted face SHALL be subject to the
engine's mandatory watertight + volume/area self-verify against the OCCT oracle
downstream; a native result that is not watertight or off-volume SHALL be DISCARDED →
OCCT, so a wrong or leaky solid is never emitted.

The reader SHALL remain OCCT-free (`src/native/**` links no OCCT / `IEngine` /
`EngineShape` type), the native tessellator SHALL NOT be modified, `FaceSurface` SHALL be
unchanged (`weights` already exists), no tolerance SHALL be weakened, and the `cc_*` ABI
SHALL be unchanged (additive read-side behaviour only). The non-rational
`B_SPLINE_SURFACE_WITH_KNOTS` keyword path, the analytic Plane / Cylinder / Cone / Sphere
/ Torus arms, and the bare-periodic full-sphere / full-torus / full-revolution B-spline
admission paths SHALL remain byte-identical. A rational **curve** read remains out of
scope (unchanged decline).

#### Scenario: A foreign rational B-spline surface face imports and meshes watertight matching BRepMesh (sim, parity)

- GIVEN a foreign STEP file carrying a rational B-spline surface face as the combined `RATIONAL_B_SPLINE_SURFACE` record bounded by a genuinely trimmed `EDGE_LOOP` whose boundary pcurves reconstruct faithfully, imported on a booted iOS simulator with OCCT linked and the native engine active (`cc_set_engine(1)`)
- WHEN the reader parses the combined record, populates `FaceSurface::weights`, reconstructs each boundary pcurve, passes the rational-aware `S_face(pcurve(t)) = C_edge(t)` guard, admits the face, and the landed M0 tessellator meshes the resulting solid
- THEN the native solid's enclosed volume, surface area, watertight status, triangle envelope, and sub-shape topology SHALL match the OCCT `STEPControl_Reader` re-import + `BRepMesh_IncrementalMesh` oracle within tolerance (a foreign rational NURBS patch that previously declined for lack of a weights read now meshes watertight)

#### Scenario: The parsed rational surface reproduces a closed-form value and the guard accepts it, rejects an off-surface edge (host analytic, no OCCT)

- GIVEN a native-built trimmed `Kind::BSpline` face whose rational-quadratic weights (`{1, 1/√2, 1, …}`) make it reproduce an EXACT cylindrical/spherical section of known closed-form geometry, built on the host with NO OCCT linked, plus a deliberately perturbed off-surface variant of one boundary edge
- WHEN the reader evaluates the surface at a grid of `(u,v)` and runs the faithful-reconstruction guard (rational eval via `math::nurbsSurfacePoint`)
- THEN `surfaceValue(u,v)` SHALL equal the closed-form surface point within `1e-9`, the guard SHALL ACCEPT the faithful face (every sampled `t` within the scale-relative tolerance and the meshed solid watertight with the closed-form enclosed volume within tolerance), AND SHALL REJECT the perturbed variant (`decline()` fires) — proven against an independent closed-form oracle with no OCCT symbol linked

#### Scenario: A malformed or non-positive-weight rational record declines to OCCT (host + sim)

- GIVEN a rational B-spline surface record whose weight grid is ragged, whose weight count does not equal the pole-grid cardinality, or that contains a non-finite or non-positive weight
- WHEN the reader parses the `RATIONAL_B_SPLINE_SURFACE` sub-record
- THEN the reader SHALL `decline()` the face (NULL → OCCT), no weight SHALL be clamped, no tolerance SHALL be weakened, the file SHALL round-trip through OCCT `STEPControl_Reader` unchanged, and no approximate or leaky native face SHALL be emitted — the honest decline is reported

#### Scenario: A rational face whose boundary pcurve does not reconstruct faithfully declines to OCCT (sim)

- GIVEN a foreign STEP file whose rational B-spline surface face has at least one boundary edge whose pcurve cannot be reconstructed within the scale-relative tolerance (non-converging inversion, off-surface reconstructed pcurve, or beyond-tolerance boundary gap)
- WHEN the reader evaluates the rational-aware faithful-reconstruction guard for that edge
- THEN the reader SHALL `decline()` the face (NULL → OCCT), the file SHALL import via OCCT identical to `cc_set_engine(0)`, and no wrong or leaky native rational face SHALL be emitted

#### Scenario: The engine self-verify discards a non-watertight admitted rational face (sim)

- GIVEN a foreign rational B-spline face that passes the per-edge pcurve guard but whose native mesh does not close watertight (or whose volume/area does not match the OCCT oracle)
- WHEN the engine runs its mandatory watertight + volume/area self-verify
- THEN the native result SHALL be DISCARDED and the import SHALL fall through to OCCT, so a wrong or leaky mesh is never emitted downstream

### Requirement: Native STEP import composes a multi-level (nested) assembly transform chain, verified vs OCCT

The reader SHALL compose a **nested** assembly by treating the
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` /
`REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` graph as a set of **parent edges**
`childShapeRepresentation → (parentShapeRepresentation, T, mirrorFlag)`, where `T` is the
per-level conformal transform read exactly as the landed single-level path reads it
(`ITEM_DEFINED_TRANSFORMATION('',desc,#from,#to)` → `T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`,
or `CARTESIAN_TRANSFORMATION_OPERATOR_3D` for a uniform-scale / mirror). For each leaf
`MANIFOLD_SOLID_BREP`, the reader SHALL locate its owning shape-representation and **walk the
parent edges to a UNIQUE root** (a shape-representation with no outgoing parent edge), composing
the per-level transforms — outermost (root) first — into ONE world transform
`W = T_root ∘ … ∘ T_leaf`. `W` SHALL be classified ONCE by the landed
`classifyPlacement` (rigid / uniform-scale / mirror); the leaf `Solid` SHALL be reconstructed at
its component-local coordinates by the unchanged `mapManifoldBrep`, then placed by
`Shape::located(Location{W})` with the landed mirror orientation-compensation, and collected into a
placed `Compound`. The composition SHALL be **depth-general** (the FIRST slice VERIFIES it at 2
levels; there is NO artificial depth cap) yet SHALL **DECLINE → OCCT** (NULL, never a partial or
identity-placed import) whenever the graph is not a clean forest of conformal placements to a unique
root: a **`MAPPED_ITEM` / `REPRESENTATION_MAP`** (Form-B) instance, a **cyclic** parent chain, an
**ambiguous** chain (a representation reached by two distinct parents, or a leaf placed more than
once), a **dangling / missing** relationship reference, or a **non-conformal** composed transform.
A **single-level** chain (length 1) SHALL compose to EXACTLY the landed single-level placement, so
the landed single-level assembly path (and the flat multi-solid / single-solid paths) SHALL remain
**byte-identical**. This walk SHALL remain OCCT-free and host-buildable, SHALL reference no OCCT /
`IEngine` / `EngineShape` type, SHALL NOT modify the STEP writer or the tessellator, and SHALL NOT
fabricate a placement or a solid the file does not describe, nor weaken any tolerance.

The slice SHALL be verified by (a) a **HOST ANALYTIC gate (no OCCT)**: for an OCCT-free nested
buffer with known per-level transforms, each leaf's composed world `Location` SHALL equal an
INDEPENDENT matrix-composition of the file's frame-pair transforms, and each placed leaf's world
centroid SHALL sit at that composed placement; the single-level, flat, and round-trip cases SHALL
still pass unchanged; and the decline cases (`MAPPED_ITEM`, cyclic, ambiguous, dangling,
non-conformal) SHALL return NULL; and (b) a **SIM native-vs-OCCT gate** (booted iOS simulator, OCCT
linked) through the `cc_*` facade: a FOREIGN OCCT-authored 2-level nested rigid assembly (authored by
`STEPCAFControl_Writer` on a nested XCAF assembly document) SHALL import natively as a placed
`Compound` whose solid **COUNT**, per-solid **volume**, per-solid **bounding box**, and per-solid
**centroid / placement** (and hence TOTAL volume) match the OCCT `STEPControl_Reader` re-import
within tolerance; and a `MAPPED_ITEM` / non-conformal / cyclic file SHALL DECLINE natively and import
via OCCT identical to `cc_set_engine(0)`. The parity test SHALL restore the OCCT default in teardown
and SHALL carry its own `main()` (on the `run-sim-suite.sh` SKIP list) so the suite assertion count
is unchanged. Every existing suite (`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) and every
prior native capability SHALL stay green at the OCCT default with no regression.

#### Scenario: A 2-level nested rigid assembly matches OCCT and the single-level path is byte-identical (host analytic)
- GIVEN an OCCT-free ISO-10303-21 nested buffer — a leaf solid placed into a sub-assembly by a rigid `T₂`, the sub-assembly placed into the root by a rigid `T₁` — read on the host with no OCCT
- WHEN `step_import_native` walks the leaf's relationship chain to its unique root
- THEN the leaf `Solid` SHALL be located by the composed `W = T₁ ∘ T₂`, VERIFIED against an independent matrix multiplication of the two frame-pair transforms (its world centroid at `W` applied to the leaf-local centroid), AND an otherwise-identical SINGLE-level buffer (chain length 1) SHALL produce the byte-identical placement the landed path produces today

#### Scenario: A foreign OCCT-authored 2-level nested assembly imports natively and matches OCCT (sim vs OCCT — the correctness gate)
- GIVEN a 2-level nested assembly (a leaf part placed in a sub-assembly, the sub-assembly placed in the top assembly by distinct rigid transforms) authored by OCCT `STEPCAFControl_Writer` on a nested XCAF document, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose solid COUNT, per-solid volume, per-solid bounding box, and per-solid centroid / placement (and TOTAL volume) match the OCCT re-import within tolerance, proving the native reader composes a foreign-authored NESTED transform tree the native writer never produces

#### Scenario: A MAPPED_ITEM, cyclic, ambiguous, or non-conformal chain declines to OCCT (host + sim)
- GIVEN a STEP file whose placement is a `MAPPED_ITEM` / `REPRESENTATION_MAP` (Form-B) instance, OR whose relationship graph is cyclic, OR whose leaf representation is reached by two distinct parents (ambiguous), OR whose composed transform is non-conformal (non-uniform-scale / shear), read on the host with no OCCT and on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `step_import_native` attempts to walk the chain
- THEN it SHALL return a NULL Shape (DECLINE) — no leaf placed at a partial or identity location, no non-conformal transform applied, no cycle silently truncated — and the engine SHALL fall through to OCCT `STEPControl_Reader` identical to `cc_set_engine(0)`, proving honest fall-through with no fabricated placement or geometry

#### Scenario: Existing suites and prior native capabilities stay green (no regression)
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green with no behavioural change, and the STEP export slice, the flat multi-solid + ELLIPSE + bspline-face + torus + revolution + single-level / scaled / mirrored assembly import slices, shape healing, SSI S1–S5, native blends + #6/#7, marching, boolean, construct, and tessellation SHALL NOT regress

### Requirement: Native STEP admission of a foreign RATIONAL B-spline CURVE as edge/trim geometry via the combined RATIONAL_B_SPLINE_CURVE record, else DECLINE

The STEP reader SHALL admit ONE foreign **rational** B-spline **curve** — emitted by
OCCT `STEPControl_Writer` as a **combined** Part-21 instance
`( BOUNDED_CURVE() B_SPLINE_CURVE(degree, (#pole…), form, closed, selfInt)
B_SPLINE_CURVE_WITH_KNOTS((mults), (knots), spec) RATIONAL_B_SPLINE_CURVE((weight…))
CURVE() REPRESENTATION_ITEM('') GEOMETRIC_REPRESENTATION_ITEM() )` — used as an
`EDGE_CURVE`'s 3D geometry (directly or through a `SURFACE_CURVE` / `SEAM_CURVE` /
`INTERSECTION_CURVE` unwrap) or as a `TRIMMED_CURVE` basis, as a native
`EdgeCurve` of `Kind::BSpline` carrying curve `weights`, **only when** the combined
record is well-formed AND the per-edge faithful-reconstruction guard verifies the edge,
so the landed native mesh path (`native-tessellation`, whose edge evaluator already
consumes `weights`) — which SHALL NOT be modified — can mesh the solid watertight. This
extends the M4-rational `RATIONAL_B_SPLINE_SURFACE` admission one dimension down to
rational NURBS **curves** by reading the curve weights the reader does not read today.

The reader SHALL dispatch the combined record using the existing combined-instance
machinery it already uses for the rational surface and assembly relationships
(`Record::subs` sub-record scan, `hasSub` / `findSub`) — no new tokenizer. It SHALL read
`degree` and the pole list from the `B_SPLINE_CURVE` sub-record, the RLE-expanded `knots`
(via the existing `expandKnots`) from the `B_SPLINE_CURVE_WITH_KNOTS` sub-record, and the
flat weight list from the `RATIONAL_B_SPLINE_CURVE` sub-record into `EdgeCurve::weights`
in the SAME order as the poles. The shared degree/pole/knot parse SHALL be factored so the
non-rational `B_SPLINE_CURVE_WITH_KNOTS` keyword path produces a byte-identical
`EdgeCurve`.

The reader SHALL `decline()` the curve (NULL → OCCT) when the combined record is
malformed: a required sibling sub-record (`B_SPLINE_CURVE`, `B_SPLINE_CURVE_WITH_KNOTS`,
`RATIONAL_B_SPLINE_CURVE`) is missing or mistyped, the knot vector length does not equal
`poles + degree + 1`, the weight count does not equal the pole count, or any weight is
non-finite or not strictly positive. A weight is NEVER clamped and no tolerance is
introduced by this well-formedness check.

The reader's faithful-guard curve evaluator SHALL be rational-aware: for a `Kind::BSpline`
edge it SHALL evaluate with `math::nurbsCurvePoint` over the curve poles / knots /
`weights` when `weights` is non-empty and with the existing non-rational `math::curvePoint`
otherwise, so the per-edge `pcurve(t) = C_edge(t)` reconstruction guard evaluates a
rational edge correctly. The `TRIMMED_CURVE` sub-domain machinery (the two
`PARAMETER_VALUE` trims selecting the covered knot span) SHALL be reused UNCHANGED for a
rational basis. An admitted rational edge SHALL be subject to the engine's mandatory
watertight + volume/area self-verify against the OCCT oracle downstream; a native result
that is not watertight or off-volume SHALL be DISCARDED → OCCT, so a wrong or leaky solid
is never emitted.

The reader SHALL remain OCCT-free (`src/native/**` links no OCCT / `IEngine` /
`EngineShape` type), the native tessellator SHALL NOT be modified, `EdgeCurve` and
`PCurve` SHALL be unchanged (`weights` already exists), no tolerance SHALL be weakened, and
the `cc_*` ABI SHALL be unchanged (additive read-side behaviour only). The non-rational
`B_SPLINE_CURVE_WITH_KNOTS` keyword path, the analytic Line / Circle / Ellipse curve arms,
the M4 non-rational surface path, and the M4-rational `RATIONAL_B_SPLINE_SURFACE` surface
path SHALL remain byte-identical. A closed/periodic rational curve needing a seam close
beyond the knot-span clamp, and a rational 2D PCURVE authored in the file, remain out of
scope (unchanged decline / ignored, as the reader synthesises its own analytic pcurves).

#### Scenario: A foreign solid with a rational B-spline edge/trim curve imports and meshes watertight matching BRepMesh (sim, parity)

- GIVEN a foreign STEP file carrying a rational B-spline curve as an `EDGE_CURVE`'s 3D geometry or a `TRIMMED_CURVE` basis, expressed as the combined `RATIONAL_B_SPLINE_CURVE` record, imported on a booted iOS simulator with OCCT linked and the native engine active (`cc_set_engine(1)`)
- WHEN the reader parses the combined record, populates `EdgeCurve::weights`, recovers the trimmed knot sub-domain, passes the rational-aware per-edge faithful guard, admits the edge, and the landed M0 tessellator meshes the resulting solid
- THEN the native solid's enclosed volume, surface area, watertight status, triangle envelope, and sub-shape topology SHALL match the OCCT `STEPControl_Reader` re-import + `BRepMesh_IncrementalMesh` oracle within tolerance (a foreign rational NURBS edge that previously declined for lack of a curve-weights read now meshes watertight)

#### Scenario: The parsed rational curve reproduces a closed-form value and the guard accepts it, rejects an off-curve weight (host analytic, no OCCT)

- GIVEN a native-built `Kind::BSpline` `EdgeCurve` whose rational-quadratic weights (`{1, cos(Δ/2), 1}` over the arc's control triangle) make it reproduce an EXACT circular arc of known closed-form geometry, built on the host with NO OCCT linked, plus a deliberately perturbed off-curve weight variant
- WHEN the reader evaluates the curve at a grid of parameters `t` and runs the per-edge faithful-reconstruction guard (rational eval via `math::nurbsCurvePoint`)
- THEN `evalEdge(t)` SHALL equal the closed-form circle point `O + R(cos θ, sin θ)` within `1e-9`, the guard SHALL ACCEPT the faithful edge, AND SHALL REJECT the perturbed variant (`decline()` fires) — proven against an independent closed-form oracle with no OCCT symbol linked

#### Scenario: A malformed or non-positive-weight rational curve record declines to OCCT (host + sim)

- GIVEN a combined rational B-spline curve record whose weight count does not equal the pole count, whose knot vector length does not equal `poles + degree + 1`, that is missing a required sibling sub-record, or that contains a non-finite or non-positive weight
- WHEN the reader parses the `RATIONAL_B_SPLINE_CURVE` sub-record
- THEN the reader SHALL `decline()` the curve (NULL → OCCT), no weight SHALL be clamped, no tolerance SHALL be weakened, the file SHALL round-trip through OCCT `STEPControl_Reader` unchanged, and no approximate native edge SHALL be emitted — the honest decline is reported

#### Scenario: A rational edge whose faithful guard fails, or whose native mesh is not watertight, declines to OCCT (sim)

- GIVEN a foreign STEP file whose rational B-spline edge either cannot be reconstructed within the scale-relative faithful-guard tolerance, or whose admitted native mesh does not close watertight (or whose volume/area does not match the OCCT oracle)
- WHEN the reader runs the rational-aware per-edge faithful guard, or the engine runs its mandatory watertight + volume/area self-verify
- THEN the reader SHALL `decline()` (or the engine SHALL DISCARD) the native result and the import SHALL fall through to OCCT identical to `cc_set_engine(0)`, so a wrong or leaky native solid is never emitted downstream

#### Scenario: The non-rational and rational-surface paths stay byte-identical (zero-regression)

- GIVEN the existing STEP round-trip suite and the full tessellation-sensitive sim suite, with the rational-curve arm reachable ONLY by a combined `RATIONAL_B_SPLINE_CURVE` record (which declines today at `curve()`'s `r->combined` guard)
- WHEN the suite runs with this change applied
- THEN the non-rational `B_SPLINE_CURVE_WITH_KNOTS` keyword path, the analytic Line / Circle / Ellipse arms, the M4 non-rational surface path, and the M4-rational `RATIONAL_B_SPLINE_SURFACE` surface path SHALL produce byte-identical native results, `src/native/**` SHALL contain 0 OCCT includes, the tessellator SHALL be unmodified, and the `cc_*` ABI SHALL be unchanged — otherwise the rational-curve arm is reverted and the rational curve keeps the OCCT decline
### Requirement: Native AP242 PMI recognise + classify + count as additive read-only metadata

The native STEP reader SHALL provide a read-only scan
(`step_scan_pmi(path) → PmiSummary`, and the additive facade
`cc_step_pmi_scan(path, &CCPmiSummary)`) that RECOGNISES the AP242 PMI / GD&T /
draughting annotation entities in a STEP file, CLASSIFIES each into a native
`PmiClass` (Dimension, GeometricTolerance, Datum, DatumTarget, Note,
AnnotationGeometry, or Unknown), COUNTS them per class, and records for each
annotation its raw STEP keyword and its referenced attachment target `#id` (the
`SHAPE_ASPECT` / geometric-item / dimensional-characteristic it points at, `0` when
none). The scan SHALL be a SEPARATE pass over the parsed Part-21 record table that
does NOT invoke or modify the geometry mapper (`Mapper::build`) — it is pure,
deterministic (annotations in ascending `#id` order), and OCCT-free
(`src/native/**` retains 0 OCCT includes). A keyword reached as PMI but outside the
recognised classification table SHALL be counted `Unknown` and NEVER fabricated into
a specific class; tolerance magnitudes, zones, modifiers, feature-control-frame
semantics, and datum reference frames are explicitly OUT of this slice and SHALL NOT
be invented.

#### Scenario: A recognised PMI census is classified and counted

- GIVEN a STEP file carrying AP242 PMI — dimensions (`DIMENSIONAL_SIZE` /
  `DIMENSIONAL_LOCATION` / `ANGULAR_SIZE`), geometric tolerances
  (`GEOMETRIC_TOLERANCE` and its `*_TOLERANCE` subtypes), datums (`DATUM` /
  `DATUM_FEATURE`), and notes / annotation-occurrence entities
- WHEN `step_scan_pmi` (or `cc_step_pmi_scan`) is called on the file
- THEN each recognised annotation SHALL be classified into the correct `PmiClass`,
  the per-class counts and `total` SHALL reflect the file's PMI content, each item
  SHALL carry its raw keyword and its referenced attachment `#id`, and `anyPmi`
  SHALL be true

#### Scenario: A PMI-adjacent but unclassifiable entity is counted Unknown, not faked

- GIVEN a STEP file whose PMI graph reaches an entity keyword outside the recognised
  classification table
- WHEN the scan classifies that entity
- THEN it SHALL be counted as `PmiClass::Unknown` (its keyword preserved), no
  specific dimension / tolerance / datum class SHALL be fabricated for it, and no
  tolerance value or semantic meaning SHALL be invented

#### Scenario: A solid-only file reports no PMI

- GIVEN a STEP file with a `MANIFOLD_SOLID_BREP` and no PMI / annotation entities
- WHEN `step_scan_pmi` is called
- THEN `anyPmi` SHALL be false, `total` SHALL be 0, and every per-class count SHALL
  be 0

### Requirement: The PMI scan is additive and keeps the geometry import byte-identical

Adding the PMI scan SHALL NOT change the geometry import in any way. The existing
`step_import_native(path) → topo::Shape`, `readStepFile`, and the `cc_step_import`
facade SHALL retain their exact signatures and behaviour, and the imported solid for
ANY file — PMI-bearing or not — SHALL be BYTE-IDENTICAL to the pre-change result. The
new PMI accessor SHALL be additive-only: `cc_step_pmi_scan` / `CCPmiSummary` are NEW
symbols, and no existing `cc_*` accessor SHALL be modified or removed. The AP242
PMI-skip behaviour (a relationship / annotation graph reaching no
`MANIFOLD_SOLID_BREP` is skipped so the solid still imports) SHALL be preserved
unchanged; the PMI is now additionally EXPOSED as metadata, not folded into the
geometry.

#### Scenario: Geometry import is unchanged for a PMI-bearing file

- GIVEN an AP242 file carrying both a `MANIFOLD_SOLID_BREP` and PMI annotations
- WHEN the file is imported via `step_import_native` / `cc_step_import`
- THEN the resulting solid's volume, area, centroid, face and edge counts, and
  serialized-shape hash SHALL be bit-for-bit identical to the pre-change import, and
  the existing STEP-import suite SHALL stay green

#### Scenario: The PMI accessor is purely additive

- GIVEN the change introduces `cc_step_pmi_scan` / `CCPmiSummary`
- WHEN the public ABI is compared to the pre-change ABI
- THEN `cc_step_import` and every other existing `cc_*` symbol SHALL be unchanged,
  and only the new PMI symbols SHALL have been added

### Requirement: PMI scan is verified host-analytically against a known census (no OCCT)

The PMI scan SHALL be verified on the HOST ANALYTIC gate (`clang++ -std=c++20`, no
OCCT linked): against a fixture whose PMI content is KNOWN, the scan's per-class
counts, per-item `PmiClass` + keyword, and per-item `attachedTo` target `#id` SHALL
match the fixture's known census exactly. This gate SHALL cover every class the scan
recognises, including graphical-only notes that an OCCT XDE oracle does not surface.

#### Scenario: Scan matches the hand-authored fixture census exactly

- GIVEN a hand-authored AP242 fixture with a known census (e.g. 2 dimensions, 1
  position tolerance, 1 datum, 1 note, each with a known `SHAPE_ASPECT` attachment
  `#id`)
- WHEN `step_scan_pmi` is run on it with no OCCT present
- THEN the returned counts, per-item classes + keywords, and `attachedTo` ids SHALL
  equal the known census, proving recognition/classification/count without any oracle
  dependency

### Requirement: PMI scan matches OCCT XDE on the covered semantic-PMI set (simulator gate)

Where OCCT exposes PMI, the native scan SHALL be verified against the OCCT oracle on
the SIM gate: the same fixture (and a real foreign AP242 PMI file) loaded via
`STEPCAFControl_Reader` into an `XCAFDoc` document and queried through
`XCAFDoc_DimTolTool` for dimensions, geometric tolerances, and datums. The native
scan's per-class counts for that semantic-PMI set SHALL match OCCT XDE. Classes OCCT
XDE does not surface (free graphical notes) SHALL be covered by the host-analytic
gate and that boundary SHALL be documented — never silently claimed as
OCCT-verified.

#### Scenario: Native counts match OCCT XDE for dimensions, tolerances, and datums

- GIVEN a booted simulator with the XDE toolkits linked (`TKXCAF` / `TKDESTEP`) and
  an AP242 PMI file
- WHEN the file is read by OCCT `STEPCAFControl_Reader` into `XCAFDoc` and its
  dimensions / geometric tolerances / datums are counted via `XCAFDoc_DimTolTool`,
  and independently scanned by the native `step_scan_pmi`
- THEN the native per-class counts for the semantic-PMI set SHALL match the OCCT XDE
  counts

#### Scenario: A class OCCT XDE does not expose is verified host-analytically, not faked

- GIVEN a PMI class (e.g. a free graphical note) that OCCT XDE `XCAFDoc_DimTolTool`
  does not surface
- WHEN that class is verified
- THEN it SHALL be checked against the known-census host gate and the design SHALL
  state that this class is host-verified, not OCCT-verified — no false OCCT-parity
  claim SHALL be made

### Requirement: Honest decline when a PMI slice is unreachable

The reader SHALL DECLINE a PMI scope honestly whenever a reachable slice does not
exist — no native PMI data model can faithfully represent a construct, the trimmed
sim toolkit exposes no XDE PMI, AND no known-census fixture can be authored to verify
it. In that case the reader SHALL report the specific blocker and SHALL leave the
geometry import exactly as today (the PMI-skip solid, byte-identical). No annotation data SHALL be fabricated to
manufacture a passing result, and no tolerance SHALL be weakened to force recognition.

#### Scenario: Unrepresentable PMI is declined, geometry preserved

- GIVEN a PMI construct the first slice cannot faithfully recognise or verify on
  either gate
- WHEN the reader processes the file
- THEN it SHALL count that construct `Unknown` or omit it (never fabricate a class or
  value), the specific blocker SHALL be reported, and the imported solid SHALL remain
  the current byte-identical PMI-skip result

