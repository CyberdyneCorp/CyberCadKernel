# native-exchange

## MODIFIED Requirements

### Requirement: Native STEP AP203 import of a writer-emitted manifold-solid-brep

The native exchange library SHALL provide `step_import_native(path)` that reads an ISO-10303-21 (STEP
Part 21) file — **independently of its `FILE_SCHEMA` header** (AP203, AP214 `AUTOMOTIVE_DESIGN`, or AP242
are all accepted; the reader gates on entities + the mm length-unit context, not the schema string, and
**skips** AP242 PMI / annotation entities and additive plane-angle / solid-angle / PMI unit contexts) —
and reconstructs a native `topology::Shape`: a `Solid` (one root `MANIFOLD_SOLID_BREP`), a **flat**
`Compound` (several co-equal roots, no transform tree), a **placed** `Compound` (a single- OR
**multi-level (nested)** **Form-A** assembly composed by walking the `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` /
`REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` relationship chain from each leaf shape-representation to
its UNIQUE root, composing the per-level rigid / uniform-scale / mirror transforms into ONE world placement
per leaf), or a **placed** `Compound` from a **Form-B** `MAPPED_ITEM` / `REPRESENTATION_MAP` assembly-reuse
instancing (see below), else DECLINE — a cyclic / ambiguous / dangling Form-A chain, a non-conformal composed
transform, a Form-B mapped representation the reader cannot resolve to exactly one shared brep, a file MIXING
Form-A (a CDSR reaching a brep) AND Form-B, or a lone `REPRESENTATION_MAP` with no `MAPPED_ITEM` instancing it
declines — with mirror orientation-compensation so each placed solid self-verifies watertight; the
tessellator SHALL NOT be modified and no normal SHALL be fabricated. The faces SHALL carry surfaces of kind
`PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS`
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

- **A Form-B `MAPPED_ITEM` / `REPRESENTATION_MAP` assembly** (the standard AP242 assembly-reuse mechanism)
  SHALL be admitted as follows. A `REPRESENTATION_MAP(#mapping_origin, #mapped_representation)` names a
  SHARED representation and its source frame; a `MAPPED_ITEM('', #mapping_source, #mapping_target)` instances
  that representation once, where `#mapping_source` is the `REPRESENTATION_MAP` and `#mapping_target` is the
  instance placement item. The reader SHALL resolve the SHARED `MANIFOLD_SOLID_BREP` from
  `#mapped_representation`'s item list (exactly ONE brep, else DECLINE — an ambiguous or brep-less mapped
  representation), compute the instance world transform `T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹`
  for an `AXIS2_PLACEMENT_3D` `#mapping_target` (matching OCCT `STEPControl_ActorRead`'s
  `SetTransformation(target, origin)`) OR resolve a `CARTESIAN_TRANSFORMATION_OPERATOR_3D` `#mapping_target`
  through the existing operator path, and CLASSIFY the composed transform conformal (rigid / uniform-scale /
  mirror; a non-uniform / shear / singular transform DECLINES). It SHALL map the shared brep ONCE (memoized
  by its #id) and re-instance it through the SHARED geometry node, applying each `MAPPED_ITEM`'s `T` via
  `Shape::located(Location{T})` (and complementing a mirrored instance's faces so the world normals point
  outward), yielding a placed `Compound` (a single `MAPPED_ITEM` yields a single located `Solid`). Multiple
  `MAPPED_ITEM`s referencing the SAME `REPRESENTATION_MAP` SHALL each produce their own placed instance of
  the ONE shared solid.
- **A `TRIMMED_CURVE`** SHALL be unwrapped to its basis curve (recursively; B-spline basis takes its
  `[first,last]` from the `PARAMETER_VALUE` trims clamped to the clamped knot span, analytic basis keeps
  the vertex-derived range) exactly as the landed trimmed-curve slice does (unchanged).
- **A `TOROIDAL_SURFACE`** `('',#axis2placement, major_radius, minor_radius)` SHALL be mapped by resolving
  the `AXIS2_PLACEMENT_3D` frame and the two trailing reals and building a native `FaceSurface` of kind
  `Torus` (`radius` = major, `minorRadius` = minor), reconstructed watertight (unchanged).
- **A `SURFACE_OF_REVOLUTION`** `('',#profile,#axis1)` SHALL be mapped exactly as the landed revolution
  slice does (straight generatrix → cylinder / cone / plane; on-axis circle → sphere; off-axis circle →
  torus; ellipse / non-rational B-spline → the exact revolved rational tensor-product `Kind::BSpline`),
  VERIFIED to pass through the profile and self-verify watertight before emission, else DECLINE (unchanged).

The Form-B admission SHALL reuse the SAME `frameToWorld` / `classifyPlacement` / `Shape::located` /
`reversedShape` substrate as the Form-A path; it SHALL NOT modify `mapManifoldBrep`, the leaf-geometry
mappers, the tessellator, or the STEP writer. The reader SHALL remain OCCT-free and host-buildable and
SHALL reference no OCCT / `IEngine` / `EngineShape` type. It SHALL NOT import PMI / annotation entities as
geometry (PMI *semantics* — GD&T tolerance zones, feature-control-frame evaluation, datum reference frames —
remain a measured DECLINE; only the read-only `pmi_scan` census recognises them), and SHALL NOT fabricate a
curve, a surface, a trim, a placement, or a solid the file does not describe, nor weaken any tolerance.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A Form-B MAPPED_ITEM assembly instances one shared representation at N known placements (host)
- GIVEN an in-scope ISO-10303-21 buffer describing a `REPRESENTATION_MAP` over a shared box `MANIFOLD_SOLID_BREP`, instanced by N `MAPPED_ITEM`s whose `AXIS2_PLACEMENT_3D` targets are N known rigid placements (translations and a rotation), read on the host with no OCCT
- WHEN `step_import_native` reads the buffer
- THEN it SHALL return a placed `Compound` of N `Solid`s, each valid + watertight, each with the shared box's EXACT closed-form volume, and each located at its target frame so its world bounding box equals the box translated / rotated by that `MAPPED_ITEM`'s target placement (the shared brep is mapped once and re-instanced through the shared node)

#### Scenario: A Form-B mapped representation with no brep, a lone REPRESENTATION_MAP, or a mixed Form-A+B file declines (host)
- GIVEN an in-scope ISO-10303-21 buffer that is either a `MAPPED_ITEM` whose `REPRESENTATION_MAP`'s mapped representation lists NO `MANIFOLD_SOLID_BREP` (or more than one), a lone `REPRESENTATION_MAP` with no `MAPPED_ITEM` instancing it, or a file that MIXES a Form-A brep-reaching `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` with a Form-B `MAPPED_ITEM`, read on the host with no OCCT
- WHEN `step_import_native` reads the buffer
- THEN it SHALL return a NULL Shape (DECLINE → OCCT), fabricating no geometry and no placement

#### Scenario: A 2-level nested rigid assembly composes each leaf's world placement by walking the relationship chain (host)
- GIVEN an in-scope ISO-10303-21 buffer describing a NESTED assembly — a leaf `MANIFOLD_SOLID_BREP` placed into a SUB-assembly shape-representation by a rigid `ITEM_DEFINED_TRANSFORMATION` `T₂` (one `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`), and that sub-assembly placed into the ROOT shape-representation by a rigid `T₁` (a second `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`), read on the host with no OCCT
- WHEN `step_import_native` walks the relationship chain from the leaf shape-representation to its unique root
- THEN it SHALL return a placed `Compound` whose leaf `Solid` is located by the COMPOSED world transform `W = T₁ ∘ T₂` (its world centroid at `W` applied to the leaf-local centroid — NOT at `T₂` alone and NOT at the origin), the composition matching an INDEPENDENT matrix multiplication of the two frame-pair transforms read from the file, AND a single-level chain (length 1) SHALL still compose to exactly today's placement (the landed single-level path is byte-identical)

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
A **Form-B `MAPPED_ITEM`** placed member SHALL be self-verified identically — each instance's `Location` is
the resolved mapped-item world transform, and every instance (including repeated instances of the ONE shared
brep) SHALL independently self-verify watertight with the correct POSITIVE enclosed volume.
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

#### Scenario: A foreign MAPPED_ITEM-instanced assembly imports natively as a placed compound (sim vs OCCT)
- GIVEN a STEP file that instances one shared representation multiple times via `MAPPED_ITEM` / `REPRESENTATION_MAP` with `AXIS2_PLACEMENT_3D` targets, on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return a placed `Compound` whose member solids each self-verify valid + watertight AND whose solid COUNT, TOTAL volume, and per-solid volume / area / centroid / bounding box / face + edge counts match the OCCT re-import within tolerance, proving Form-B instancing parity

#### Scenario: A foreign AP242 file with PMI imports its geometry natively (sim vs OCCT)
- GIVEN an OCCT-authored AP242 STEP file carrying an in-slice solid PLUS PMI annotations (written by `STEPCAFControl_Writer` with PMI), on a booted iOS simulator with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` imports it natively AND OCCT `STEPControl_Reader` imports the same file
- THEN the native import SHALL return the SOLID valid + watertight AND its volume / bounding box / count SHALL match the OCCT re-import within tolerance, with the PMI ignored on both sides, proving AP242 geometry import with PMI skipped

#### Scenario: An out-of-scope file (torus / non-uniform-scale or a non-conformal / ambiguous Form-B assembly) falls through to OCCT (sim vs OCCT)
- GIVEN a foreign OCCT-authored STEP with a `TOROIDAL_SURFACE` face, or an assembly with a component placed by a non-uniform-scale / shear transform, or a `MAPPED_ITEM` / `REPRESENTATION_MAP` whose mapped representation resolves to ≠ 1 brep or whose target transform is non-conformal, or a cyclic / ambiguous relationship chain the reader cannot compose to a unique root, with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_step_import(path)` is called
- THEN `step_import_native` SHALL return NULL (DECLINE) AND the file SHALL be imported by OCCT `STEPControl_Reader` identical to `cc_set_engine(0)`, proving fall-through with no native interception and no fabricated geometry
