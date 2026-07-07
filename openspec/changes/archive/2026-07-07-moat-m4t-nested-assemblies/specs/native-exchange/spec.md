# native-exchange

This change (MOAT **M4-tail**) **extends the landed native STEP assembly import from a
SINGLE-level placed assembly to a MULTI-LEVEL (nested) tree** — the FIRST slice: a **2-level
nested rigid / conformal assembly** (a component that is itself an assembly). The landed reader
(`add-native-step-assemblies` → `add-native-step-scaled-ap242`, both archived) composes a
component's world placement from the ONE `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` whose child
shape-representation *directly* reaches a `MANIFOLD_SOLID_BREP`; it has no notion of an ancestor
placement. This slice replaces that per-brep single-transform association with a **relationship-chain
walk**: for each leaf `MANIFOLD_SOLID_BREP`, find its owning shape-representation, walk the
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` / `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` parent
edges to a UNIQUE root shape-representation, and compose the per-level conformal transforms
(`T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`) into ONE world `topology::Location` per leaf. The
composed transform is classified ONCE by the LANDED `classifyPlacement` (rigid / uniform-scale /
mirror) and applied by the LANDED `Shape::located()` + mirror orientation-compensation. A
single-level chain (length 1) composes to EXACTLY today's placement, so the landed single-level path
stays **byte-identical** (a strict superset).

No `cc_*` ABI change; the default engine stays OCCT. The STEP writer (`step_writer.cpp`) and the
tessellator are NOT modified (a nested placed leaf is still a rigidly-located solid; a conformal
composition preserves — or, for a reflection, compensates to preserve — the watertight 2-manifold, so
it tessellates + self-verifies through the existing Location-resolving accessors). OCCT stays the
ORACLE and the honest fallback: any structure the walk cannot resolve DECLINEs → OCCT
`STEPControl_Reader`.

> NOTE (honest scope — this ONE slice). IN: a **2-level (and, by the same depth-general chain walk,
> deeper) NESTED assembly** whose every per-level transform is a conformal `ITEM_DEFINED_TRANSFORMATION`
> (rigid) or `CARTESIAN_TRANSFORMATION_OPERATOR_3D` (uniform-scale / mirror), composing to a unique
> root. OUT — **DECLINE → OCCT, never faked**: a `MAPPED_ITEM` / `REPRESENTATION_MAP` (Form-B)
> instance (still declined — this slice extends the Form-A / CDSR path only); a **cyclic** parent
> chain; an **ambiguous** chain (a leaf's owning representation reached by two distinct parents, or a
> shared sub-assembly with more than one placement into the same tree); a **dangling / missing**
> relationship reference; a **non-conformal** (non-uniform-scale / shear) composed transform; an
> **out-of-slice component geometry** (`TOROIDAL_SURFACE` etc. still decline per the landed gates); PMI
> / GD&T / colours / names; non-mm units; non-manifold / unhealable component B-reps. This does NOT
> unblock #8 `drop-occt`. **No placement or solid is fabricated: the walk composes only the transforms
> the file carries and maps only the solids the landed slice supports; a chain it cannot resolve to a
> unique root for every leaf DECLINEs rather than placing a leaf at a partial or identity location. No
> tolerance is weakened; no dead code is committed.**

## MODIFIED Requirements

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

## ADDED Requirements

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
