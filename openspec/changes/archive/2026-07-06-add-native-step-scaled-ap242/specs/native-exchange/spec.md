# native-exchange

This change (Phase 4 #7) **widens the working native STEP import reader** (landed by
`add-native-step-import`, widened by `widen-native-step-import`, extended to rigid assemblies by
`add-native-step-assemblies`) along two bounded axes: **(T1)** it accepts **uniform-scale** and
**mirror (reflection)** single-level component placements (in addition to the rigid placements it
composes today) by replacing the rigid-only gate with a conformal-affine classifier
(`Rigid` | `UniformScale(k>0)` | `Mirror(det<0)`, non-uniform/shear DECLINE) and compensating a
mirror's handedness flip at the topology level (complementing the mirrored component's face
orientation with the existing `Orientation` algebra, so the un-modified tessellator meshes it with
outward normals and it self-verifies watertight with positive volume); and **(T2)** it imports the
**geometry** of an **AP242** file while **skipping** its PMI / GD&T / annotation entities (refining
the mm unit gate to ignore additive PLANE_ANGLE / PMI unit contexts, and scoping the assembly
trigger + composer to the product-placement graph so annotation entities are skipped, not fatal).
No `cc_*` ABI change; the default engine stays OCCT. The STEP writer (`step_writer.cpp`) and the
tessellator are NOT modified. The native `math::Transform` already models the full affine map, and
the tessellator derives the world normal from the transformed tangents
(`cross(place(∂u), place(∂v))`), so a uniform scale (`k>0`) renders correctly with no change — no
tessellator arm is added.

> NOTE (honest scope): this WIDENS the working import slice to **uniform-scale + mirror
> single-level assemblies of in-slice solids** and to **AP242 geometry with PMI skipped**; it is
> NOT a general product-structure importer and NOT a PMI importer. A **non-uniform / shear**
> placement transform, a **deep multi-level nested** product structure, an **external part
> reference**, a component whose geometry is out of the import slice (`TOROIDAL_SURFACE`,
> `SURFACE_OF_REVOLUTION`, rational/weighted B-splines, `BEZIER`), a placement chain the reader
> **cannot compose**, **PMI / GD&T semantics**, non-mm length units, and non-manifold / unhealable
> component B-reps all still DECLINE → OCCT `STEPControl_Reader`. AP242 annotation / PMI entities
> are **skipped** (the geometry imports; the PMI is dropped), never imported or turned into
> geometry. IGES import/export stay OCCT `IGESControl_*`. A general native STEP/AP242 reader +
> IGES + a general-curved kernel still block #8 `drop-occt`; this change does NOT unblock it.
> **No placement, scale, reflection, or solid is ever fabricated: the reader composes only the
> transforms the file carries and maps only the solids it genuinely supports; a mirror is
> compensated by the topology orientation algebra, never by faking a normal; a structure it cannot
> compose to a supported placement for every geometric root DECLINES rather than defaulting a
> component to identity or inventing geometry. No tolerance is weakened — the mm-length gate is
> unchanged.**

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
normal SHALL be fabricated. The faces SHALL carry only surfaces of kind `PLANE`,
`CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, or
`B_SPLINE_SURFACE_WITH_KNOTS` (non-rational) and the edges only curves of kind `LINE`, `CIRCLE`,
`ELLIPSE`, or `B_SPLINE_CURVE_WITH_KNOTS` (non-rational). The reader SHALL (a) tokenize the DATA
section into a `map<#id, Record>` handling integer refs `#M`, reals including typed forms (`1.`,
`1.E2`, `-3.5E-07`), strings (`'...'` with embedded `''`), enums (`.T.` / `.PLANE.`), lists
`( ... )`, `$` (null), `*` (derived), and combined-instance `( SUB(...) SUB(...) )` records; (b)
resolve leaf geometry — `CARTESIAN_POINT` → `math::Point3` in **millimetres**, `DIRECTION` →
`math::Dir3`, `AXIS2_PLACEMENT_3D` → `math::Ax3`, the in-scope curves → `EdgeCurve`, the in-scope
surfaces → `FaceSurface`; (c) build topology following refs — `VERTEX_POINT` → vertex,
`EDGE_CURVE` → one shared edge per `#id`, `ORIENTED_EDGE` → the oriented shared edge, `EDGE_LOOP` →
wire, `FACE_OUTER_BOUND`/`FACE_BOUND` + `ADVANCED_FACE` sense → face,
`CLOSED_SHELL`/`MANIFOLD_SOLID_BREP` → shell/solid (all roots when there are several) — dropping the
writer's periodic-wall SEAM edge; and (d) **when a product-placement transform tree is present**,
resolve each component's placement by reading `ITEM_DEFINED_TRANSFORMATION('',desc,#from,#to)`
(the map carrying the `#from` `AXIS2_PLACEMENT_3D` frame onto the `#to` frame,
`T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`) and/or a `CARTESIAN_TRANSFORMATION_OPERATOR_3D`
(uniform `scale` operator; a `_NON_UNIFORM` / unequal `scale1/2/3` operator DECLINEs),
associating each via `NEXT_ASSEMBLY_USAGE_OCCURRENCE` /
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` / `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` with
its component representation's root `MANIFOLD_SOLID_BREP`(s), classifying the composed placement
(rigid / uniform-scale / mirror, else DECLINE), applying the mirror orientation compensation where
needed, then applying `Shape::located(Location{T})` per component solid. A `TOROIDAL_SURFACE` face
SHALL DECLINE (there is no native `FaceSurface::Kind::Torus` and the tessellator is not modified),
and an assembly structure the reader cannot compose to a supported placement for every geometric
root (a non-uniform / shear transform, an unmapped root, a deep-nested / external-ref structure)
SHALL DECLINE. This reader SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT /
`IEngine` / `EngineShape` type. It SHALL NOT modify the STEP writer or the tessellator, SHALL NOT
import PMI / annotation entities as geometry, and SHALL NOT fabricate a placement, a scale, a
reflection, or a solid the file does not describe.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: Multiple co-equal root solids import as a flat compound (host)
- GIVEN an in-scope ISO-10303-21 buffer with two co-equal root `MANIFOLD_SOLID_BREP`s and no product-placement transform entity, read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a `Compound` containing two `Solid`s at their world coordinates, each reconstructed exactly as the single-solid path would, AND a buffer with exactly one root SHALL still return a bare `Solid` (the single-solid + flat multi-solid behaviour is unchanged)

#### Scenario: A rigid assembly imports as a placed compound (host)
- GIVEN an in-scope ISO-10303-21 buffer describing a single-level assembly of two components placed by rigid (rotation + translation) `ITEM_DEFINED_TRANSFORMATION` transforms, read on the host with no OCCT
- WHEN `step_import_native` composes the transform tree
- THEN it SHALL return a `Compound` of two `Solid`s each placed by the composed rigid `Location` (each component's world centroid at the placed position), exactly as the archived rigid-assembly slice does (the rigid path is unchanged)

#### Scenario: A uniform-scale component imports as a scaled placed solid (host)
- GIVEN an in-scope ISO-10303-21 assembly buffer whose one component is placed by a UNIFORM-SCALE transform (a single positive factor `k`, e.g. `k = 2`, carried in the `ITEM_DEFINED_TRANSFORMATION` frames or a `CARTESIAN_TRANSFORMATION_OPERATOR_3D`), read on the host with no OCCT
- WHEN `step_import_native` classifies the composed placement
- THEN it SHALL classify it `UniformScale(k)`, reconstruct the component solid at local coordinates, apply `Shape::located(Location{T})`, and return a `Compound` whose scaled member `Solid` is valid + watertight with enclosed volume `k³ × V₀` (the unscaled volume) AND whose centroid sits at the scaled world placement — the tessellator being unmodified (a uniform `k>0` scales the tangent-derived normal magnitude but preserves its direction + winding)

#### Scenario: A mirrored component imports watertight with compensated orientation (host)
- GIVEN an in-scope ISO-10303-21 assembly buffer whose one component is placed by a MIRROR (reflection, det < 0) transform, read on the host with no OCCT
- WHEN `step_import_native` classifies the composed placement as `Mirror` and applies the placement
- THEN it SHALL complement the mirrored component solid's face orientation (the existing `topology::Orientation` algebra) BEFORE applying the reflection `Location`, so the tessellator's tangent-derived world normal points OUTWARD, AND return a `Compound` whose mirrored member `Solid` is valid + watertight with POSITIVE enclosed volume and a reflected centroid — without the compensation the enclosed volume would be negative / the solid non-watertight; the tessellator SHALL NOT be modified and no normal SHALL be fabricated

#### Scenario: A non-uniform or shear placement declines (host)
- GIVEN an ISO-10303-21 assembly buffer whose component placement is a NON-UNIFORM scale (`diag(2,1,1)`) or a shear (a linear part whose `MᵀM` is not a scalar multiple of the identity), read on the host with no OCCT
- WHEN `step_import_native` classifies the composed placement
- THEN it SHALL classify it as unsupported and return a NULL Shape (DECLINE) — without constructing any solid — so the engine can fall through to OCCT; no non-conformal transform is applied and no volume-distorting placement is imported

#### Scenario: An AP242 file with PMI imports its geometry and skips the PMI (host)
- GIVEN an ISO-10303-21 buffer whose `FILE_SCHEMA` header names AP242, carrying an in-slice solid PLUS PMI / GD&T / annotation entities (a datum, a geometric tolerance, an annotation) and additive PLANE_ANGLE / PMI unit contexts, read on the host with no OCCT
- WHEN `step_import_native` parses it
- THEN it SHALL import the SOLID exactly as it would the AP203 equivalent (the mm length-unit gate is unchanged; the additive angle / PMI unit contexts are skipped, not read as non-mm; the annotation / PMI entities are skipped and never force the assembly path or fail a scan) AND SHALL NOT import any PMI entity as geometry

### Requirement: Native STEP import runs healShell and returns NULL for out-of-scope or unhealable files

`step_import_native` SHALL rely on the shared-node reconstruction and SHALL return the assembled
`Solid` / flat `Compound` / **placed `Compound`** for the engine to self-verify. A placed component
solid SHALL be reconstructed at its local coordinates then placed by `Shape::located()`: a **rigid**
or **uniform-scale (`k>0`)** placement is conformal and preserves the watertight 2-manifold (a
uniform scale scales the volume by `k³`); a **mirror** placement SHALL have the component's face
orientation complemented (the existing `topology::Orientation` algebra) so the reflected solid meshes
with outward normals and self-verifies watertight with positive volume. The reader SHALL return a
**NULL Shape (DECLINE)** — and never a partial or invented solid — when ANY of: (i) the assembled
shell is a genuinely open / non-manifold B-rep, or a placed member fails the self-verify (including a
mirror whose orientation compensation does not yield a positive-volume watertight solid); (ii) the
file has **zero** root `MANIFOLD_SOLID_BREP`, OR carries a product-placement transform tree the reader
**cannot compose** to a supported placement for every geometric component — a **non-uniform / shear**
transform (a linear part whose `MᵀM` is not a scalar multiple of the identity), a root
`MANIFOLD_SOLID_BREP` reached by no placement, or a **deep multi-level nested** / **external-reference**
product structure (multiple co-equal root solids with no transform tree import as a flat Compound; a
single-level rigid / uniform-scale / mirror assembly imports as a placed Compound); (iii) a referenced
entity has an unsupported keyword or a surface kind outside
{`PLANE`,`CYLINDRICAL_SURFACE`,`CONICAL_SURFACE`,`SPHERICAL_SURFACE`,`B_SPLINE_SURFACE_WITH_KNOTS`} —
explicitly INCLUDING `TOROIDAL_SURFACE` and `SURFACE_OF_REVOLUTION`, in ANY component — or a curve kind
outside {`LINE`,`CIRCLE`,`ELLIPSE`,`B_SPLINE_CURVE_WITH_KNOTS`}, or a rational (weighted) B-spline wrap;
(iv) a non-millimetre LENGTH-unit context (no silent rescale; additive plane-angle / solid-angle / PMI
unit contexts are skipped and do NOT count as non-mm); or (v) a malformed / dangling record. AP242 PMI
/ annotation entities SHALL be **skipped** (never a decline trigger, never imported). The tolerance
SHALL NEVER be widened to force a pass; the honest residual SHALL be reported, not hidden.

#### Scenario: A file whose B-rep cannot form a watertight solid returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap so the assembled shell is a genuinely open shell, read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and the engine self-verifies it
- THEN the result SHALL NOT self-verify watertight AND the import SHALL DECLINE (NULL) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: A TOROIDAL_SURFACE or out-of-slice component returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE` / `SURFACE_OF_REVOLUTION` / rational B-spline — as a lone solid OR as one component of an assembly — read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid (the whole file declines — no partial import), so the engine can fall through to OCCT — no torus surface is faked (the tessellator is not modified)

#### Scenario: A non-uniform / shear or uncomposable assembly transform returns NULL (host)
- GIVEN an ISO-10303-21 assembly buffer whose component placement is a non-uniform scale / shear, OR whose transform tree leaves a root `MANIFOLD_SOLID_BREP` unplaced, OR which is a deep multi-level nested / external-reference product structure the reader cannot compose, read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid, so the engine can fall through to OCCT — no transform tree is silently flattened, no component is placed at identity, and no non-conformal (non-uniform / shear) transform is applied

#### Scenario: An AP242 PMI entity never triggers a decline (host)
- GIVEN an ISO-10303-21 AP242 buffer carrying an in-slice solid PLUS PMI / annotation entities (draughting model, annotation, datum, geometric tolerance) and additive plane-angle / PMI unit contexts, read on the host with no OCCT
- WHEN `step_import_native` runs its unit-context gate and its assembly-trigger scan
- THEN the PMI / annotation entities SHALL be SKIPPED (they SHALL NOT fail the mm length gate — the additive angle / PMI unit contexts are ignored — and SHALL NOT force the assembly path) AND the solid SHALL import; the file SHALL NOT decline merely because AP242 PMI entities are present, and no PMI entity SHALL be imported as geometry

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

## ADDED Requirements

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
