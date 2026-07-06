# native-exchange

This change (Phase 4 #7) **widens the working native STEP import reader** (landed by
`add-native-step-import`, widened by `widen-native-step-import`, extended to rigid assemblies by
`add-native-step-assemblies`, and to scaled/mirrored placements + AP242 by
`add-native-step-scaled-ap242`) to accept two general-surface families the leaf-geometry
dispatchers previously DECLINED outright, mapping ONLY the sub-cases that reconstruct onto an EXACT
native kind and self-verify watertight, and DECLINING the rest honestly (NULL → OCCT, exactly like
the landed `TOROIDAL_SURFACE` decline):

- **(T1) A `TRIMMED_CURVE` edge** — a basis curve (`LINE` / `CIRCLE` / `ELLIPSE` /
  `B_SPLINE_CURVE_WITH_KNOTS`, including a basis reached through the existing `SURFACE_CURVE` /
  `SEAM_CURVE` / `INTERSECTION_CURVE` wrapper) bounded by two trims — is now **accepted** and
  unwrapped onto the EXISTING native trimmed `Edge` (a trimmed 3D curve over a stored `[first,last]`
  parameter range). For a **B-spline** basis the two `PARAMETER_VALUE` trims select the covered knot
  sub-domain (clamped to the clamped knot span; a wide / degenerate trim reduces to the full curve) —
  information the endpoint vertices cannot recover. For an **analytic** basis (`LINE` / `CIRCLE` /
  `ELLIPSE`) the endpoint vertices already fix the range exactly, so the existing vertex-derived range
  is kept and the parameter trims are redundant. (The `sense_agreement` / `master_representation`
  fields are not consulted.) A basis outside the supported slice, or an absent / malformed basis,
  DECLINES.

- **(T2) A `SURFACE_OF_REVOLUTION` face** — a profile (generatrix) curve revolved about an
  `AXIS1_PLACEMENT` axis — is mapped onto an EXACT native surface **only** in the one case that
  reduces in closed form to a native analytic kind AND reconstructs watertight: a **straight `LINE`
  generatrix parallel to the axis**, which revolves to an exact native `Cylinder` (radius =
  perpendicular distance from the line to the axis, frame on the axis). **Every other revolution
  DECLINES honestly** → OCCT: an **oblique** line (a cone — the reader's apex-carrying cone
  reconstruction does not yet round-trip watertight, a separate pre-existing reader gap, so a mapping
  could not pass the engine's watertight self-verify), a line **perpendicular** to the axis (a planar
  annulus, meaningful only on such a cone face), a line **on** the axis (degenerate), and any
  **non-line** profile (a `CIRCLE` / arc → sphere / torus; an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS`
  → a general revolved surface) — for which there is **no faithful native `FaceSurface` kind** (no
  `Kind::Torus`; the reader authors no rational revolved-B-spline surface), kept consistent with the
  landed `TOROIDAL_SURFACE` decline.

No `cc_*` ABI change; the default engine stays OCCT. The STEP writer (`step_writer.cpp`) and the
tessellator are NOT modified.

> NOTE (honest scope): this widening **accepts** `TRIMMED_CURVE` edges over an in-slice basis and a
> `SURFACE_OF_REVOLUTION` of a **straight generatrix parallel to its axis (→ an exact native
> cylinder)**; it is NOT a general-surface reader and it authors **no** revolved-B-spline or
> apex-carrying-cone surface. Still declined → OCCT `STEPControl_Reader`: a `TOROIDAL_SURFACE`; a
> `SURFACE_OF_REVOLUTION` of an **oblique / perpendicular / on-axis line** (cone / planar annulus /
> degenerate) or of **any non-line profile** (circle / arc / ellipse / B-spline → sphere / torus /
> general revolved surface); a directly-authored **arbitrary rational (weighted) B-spline** surface /
> curve; a general **swept / bounded / offset** surface (`SURFACE_OF_LINEAR_EXTRUSION`,
> `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`, …); a `TRIMMED_CURVE` over
> an out-of-slice basis; and everything the prior slices declined (Form-B assemblies, non-mm units,
> non-uniform / shear placements). IGES import/export stay OCCT `IGESControl_*`. A general native
> STEP/AP242 reader + IGES + a general-curved kernel still block #8 `drop-occt`; this change does NOT
> unblock it.
> **No curve, surface, trim, or solid is ever fabricated: the reader maps only the geometry the file
> describes; a `TRIMMED_CURVE` / `SURFACE_OF_REVOLUTION` the reader cannot represent faithfully AND
> self-verify DECLINES rather than being forced onto a wrong or approximate native kind. No tolerance
> is weakened.**

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
normal SHALL be fabricated. The faces SHALL carry surfaces of kind `PLANE`,
`CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS`
(non-rational), or a **`SURFACE_OF_REVOLUTION` of a straight generatrix parallel to its axis**
(which reduces to an exact native cylinder), and the edges curves of kind `LINE`, `CIRCLE`,
`ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (non-rational), or a **`TRIMMED_CURVE`** whose basis is one of
those kinds. Specifically:

- **A `TRIMMED_CURVE`** `('',#basis,(trim_1),(trim_2),sense_agreement,master_representation)` SHALL
  be mapped by resolving its **basis** curve (recursively — a `LINE` / `CIRCLE` / `ELLIPSE` /
  `B_SPLINE_CURVE_WITH_KNOTS`, including a basis reached through the existing `SURFACE_CURVE` /
  `SEAM_CURVE` / `INTERSECTION_CURVE` wrapper) as the native `EdgeCurve`, and caching its two
  `PARAMETER_VALUE` trims (if present) keyed by the `TRIMMED_CURVE`'s `#id`. When the basis is a
  **`B_SPLINE_CURVE_WITH_KNOTS`** and both parameter trims are present, the native `Edge`'s
  `[first,last]` range SHALL be taken from those trims (min/max, clamped to the clamped knot span; a
  wide / degenerate span reduces to the full curve) — the covered knot sub-domain the endpoint
  vertices cannot recover. When the basis is **analytic** (`LINE` / `CIRCLE` / `ELLIPSE`) the reader
  SHALL keep the existing vertex-derived range (the endpoint vertices fix the range exactly and the
  parameter trims are redundant). A `TRIMMED_CURVE` whose basis is out of slice (rational /
  unsupported curve) or absent / malformed SHALL DECLINE. No new topology is added — the native
  `Edge` already stores an arbitrary trimmed `[first,last]` range.
- **A `SURFACE_OF_REVOLUTION`** `('',#profile,#axis1)` SHALL be mapped by resolving the axis
  (`AXIS1_PLACEMENT('',#origin,#axis)` — origin + one direction, `$` axis defaulting to +Z) and the
  **profile** curve (via the same curve dispatcher, including a `TRIMMED_CURVE` profile), then mapping
  it to an EXACT native surface **only** when the profile is a **straight `LINE` generatrix parallel
  to the axis** — a native `Cylinder` of radius = the perpendicular distance from the line to the
  axis, frame on the axis, built with the existing analytic `FaceSurface` machinery so the reduced
  surface is identical to the analytic-keyword-equivalent cylinder. In **every** other case the reader
  SHALL DECLINE: an **oblique** line (a cone — the reader's apex-carrying cone reconstruction does not
  round-trip watertight), a line **perpendicular** to the axis (a planar annulus on such a cone), a
  line **on** the axis (degenerate), and any **non-line** profile (a `CIRCLE` / arc → sphere / torus,
  an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` → a general revolved surface) — there is no faithful
  native `FaceSurface` kind (no `Kind::Torus`; no authored revolved-B-spline surface), kept a DECLINE
  consistent with the `TOROIDAL_SURFACE` decline.

The reader SHALL (a) tokenize the DATA section into a `map<#id, Record>` handling integer refs `#M`,
reals including typed forms (`1.`, `1.E2`, `-3.5E-07`), strings (`'...'` with embedded `''`), enums
(`.T.` / `.PLANE.`), lists `( ... )`, `$` (null), `*` (derived), and combined-instance
`( SUB(...) SUB(...) )` records; (b) resolve leaf geometry — `CARTESIAN_POINT` → `math::Point3` in
**millimetres**, `DIRECTION` → `math::Dir3`, `AXIS2_PLACEMENT_3D` → `math::Ax3`, `AXIS1_PLACEMENT` →
axis (origin + direction), the in-scope curves (including a `TRIMMED_CURVE`'s basis) → `EdgeCurve`,
the in-scope surfaces (including a cylinder-reducing `SURFACE_OF_REVOLUTION`) → `FaceSurface`;
(c) build topology following refs — `VERTEX_POINT` → vertex, `EDGE_CURVE` → one shared edge per
`#id` (its `[first,last]` from the trims when the 3D curve is a `TRIMMED_CURVE` over a B-spline
basis), `ORIENTED_EDGE` → the oriented shared edge, `EDGE_LOOP` → wire, `FACE_OUTER_BOUND` /
`FACE_BOUND` + `ADVANCED_FACE` sense → face, `CLOSED_SHELL` / `MANIFOLD_SOLID_BREP` → shell/solid (all
roots when there are several) — dropping the writer's periodic-wall SEAM edge; and (d) **when a
product-placement transform tree is present**, compose it exactly as the archived assembly slices do
(rigid / uniform-scale / mirror, else DECLINE), applying the mirror orientation compensation where
needed, then `Shape::located(Location{T})` per component solid. A `TOROIDAL_SURFACE` face, and a
`SURFACE_OF_REVOLUTION` the reader cannot reduce to a cylinder, SHALL DECLINE, and an assembly
structure the reader cannot compose to a supported placement for every geometric root SHALL DECLINE.
This reader SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. It SHALL NOT modify the STEP writer or the tessellator, SHALL NOT import PMI /
annotation entities as geometry, and SHALL NOT fabricate a curve, a surface, a trim, a placement, or a
solid the file does not describe.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: Multiple co-equal root solids import as a flat compound (host)
- GIVEN an in-scope ISO-10303-21 buffer with two co-equal root `MANIFOLD_SOLID_BREP`s and no product-placement transform entity, read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a `Compound` containing two `Solid`s at their world coordinates, each reconstructed exactly as the single-solid path would, AND a buffer with exactly one root SHALL still return a bare `Solid` (the single-solid + flat multi-solid behaviour is unchanged)

#### Scenario: A rigid / uniform-scale / mirror assembly still imports as a placed compound (host)
- GIVEN an in-scope ISO-10303-21 buffer describing a single-level assembly of components placed by rigid, uniform-scale, or mirror `ITEM_DEFINED_TRANSFORMATION` transforms, read on the host with no OCCT
- WHEN `step_import_native` composes the transform tree
- THEN it SHALL return a `Compound` of the placed `Solid`s exactly as the archived scaled/mirrored-assembly slice does (the placed-assembly paths are unchanged by this change)

#### Scenario: A TRIMMED_CURVE edge is accepted and unwrapped onto the native trimmed edge (host)
- GIVEN an in-scope ISO-10303-21 buffer where one `EDGE_CURVE`'s 3D curve is a `TRIMMED_CURVE` over a `LINE` / `CIRCLE` / `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` basis, read on the host with no OCCT
- WHEN `step_import_native` resolves the edge
- THEN it SHALL unwrap the `TRIMMED_CURVE` to the basis curve as the native `EdgeCurve` (the keyword declined before), setting the native `Edge`'s `[first,last]` from the `PARAMETER_VALUE` trims (clamped to the clamped knot span) when the basis is a B-spline, or from the endpoint vertices when the basis is analytic, AND the assembled solid SHALL be valid + watertight; no new topology is introduced (the native `Edge` already stores an arbitrary trimmed range)

#### Scenario: A SURFACE_OF_REVOLUTION of a line parallel to its axis reduces to an exact native cylinder (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a straight `LINE` generatrix parallel to the revolution axis, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL reduce it to the EXACT native analytic `Cylinder` (radius = the perpendicular distance from the line to the axis, frame on the axis) that represents the revolution, built with the existing analytic machinery, AND the assembled solid SHALL be valid + watertight and identical to the `CYLINDRICAL_SURFACE`-keyword-equivalent solid

#### Scenario: A SURFACE_OF_REVOLUTION of an oblique / non-line generatrix declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is an **oblique** or **perpendicular** `LINE` (a cone / planar annulus), a `LINE` on the axis (degenerate), or any **non-line** generatrix — a `CIRCLE` / arc (a sphere / torus) or an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` (a general revolved surface) — read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid — the reader authors no apex-carrying cone, no torus (there is no native `FaceSurface::Kind::Torus`), and no revolved-B-spline surface — so the engine can fall through to OCCT; the decline is kept consistent with the landed `TOROIDAL_SURFACE` decline and never a forced or approximate face

### Requirement: Native STEP import runs healShell and returns NULL for out-of-scope or unhealable files

`step_import_native` SHALL rely on the shared-node reconstruction and SHALL return the assembled
`Solid` / flat `Compound` / **placed `Compound`** for the engine to self-verify. A placed component
solid SHALL be reconstructed at its local coordinates then placed by `Shape::located()`: a **rigid**
or **uniform-scale (`k>0`)** placement is conformal and preserves the watertight 2-manifold; a
**mirror** placement SHALL have the component's face orientation complemented so the reflected solid
meshes with outward normals and self-verifies watertight with positive volume. A `TRIMMED_CURVE` edge
SHALL be reconstructed onto the native trimmed `Edge` (basis `EdgeCurve` + trim-driven `[first,last]`
for a B-spline basis, vertex-derived range otherwise), and a **cylinder-reducing**
`SURFACE_OF_REVOLUTION` face onto its exact native `Cylinder` — both subject to the same watertight
self-verify. The reader SHALL return a **NULL Shape (DECLINE)** — and never a partial or invented
solid — when ANY of: (i) the assembled shell is a genuinely open / non-manifold B-rep, or a placed
member fails the self-verify; (ii) the file has **zero** root `MANIFOLD_SOLID_BREP`, OR carries a
product-placement transform tree the reader **cannot compose** to a supported placement for every
geometric component (a **non-uniform / shear** transform, a root reached by no placement, or a **deep
multi-level nested** / **external-reference** product structure); (iii) a referenced entity has an
unsupported keyword or a surface kind outside
{`PLANE`,`CYLINDRICAL_SURFACE`,`CONICAL_SURFACE`,`SPHERICAL_SURFACE`,`B_SPLINE_SURFACE_WITH_KNOTS`, a
**cylinder-reducing** `SURFACE_OF_REVOLUTION`} — explicitly INCLUDING `TOROIDAL_SURFACE`, a
`SURFACE_OF_REVOLUTION` of an **oblique / perpendicular / on-axis line** (cone / planar annulus /
degenerate) or of **any non-line profile** (a circle / arc → sphere / torus, an ellipse / B-spline →
general revolved surface), a **directly-authored arbitrary rational (weighted)** B-spline surface, and
a general swept / bounded / offset surface (`SURFACE_OF_LINEAR_EXTRUSION`,
`RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`), in ANY component — or a
curve kind outside {`LINE`,`CIRCLE`,`ELLIPSE`,`B_SPLINE_CURVE_WITH_KNOTS`, a `TRIMMED_CURVE` over one
of those}, a `TRIMMED_CURVE` over an out-of-slice basis, or a rational (weighted) B-spline wrap; (iv) a
non-millimetre LENGTH-unit context (no silent rescale; additive plane-angle / solid-angle / PMI unit
contexts are skipped and do NOT count as non-mm); or (v) a malformed / dangling record. AP242 PMI /
annotation entities SHALL be **skipped** (never a decline trigger, never imported). The tolerance
SHALL NEVER be widened to force a pass; the honest residual SHALL be reported, not hidden.

#### Scenario: A file whose B-rep cannot form a watertight solid returns NULL (host)
- GIVEN an ISO-10303-21 buffer describing an in-scope entity graph whose faces leave a boundary gap so the assembled shell is a genuinely open shell, read on the host with no OCCT
- WHEN `step_import_native` assembles the B-rep and the engine self-verifies it
- THEN the result SHALL NOT self-verify watertight AND the import SHALL DECLINE (NULL) with the tolerance NOT widened — never a fabricated closed solid

#### Scenario: A TOROIDAL_SURFACE or a non-cylinder SURFACE_OF_REVOLUTION or out-of-slice surface returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE`, a `SURFACE_OF_REVOLUTION` whose profile is not a line parallel to its axis (an oblique / perpendicular line, or a circle / arc / ellipse / B-spline profile — a cone / sphere / torus / general revolved surface), a directly-authored arbitrary rational B-spline surface, or a general swept / bounded / offset surface — as a lone solid OR as one component of an assembly — read on the host with no OCCT
- WHEN `step_import_native` maps the entity table
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any solid (the whole file declines — no partial import), so the engine can fall through to OCCT — no cone / sphere / torus / rational / swept surface is faked (the tessellator is not modified)

#### Scenario: A TRIMMED_CURVE over an out-of-slice basis returns NULL (host)
- GIVEN an ISO-10303-21 buffer with a `TRIMMED_CURVE` whose basis is a rational / unsupported curve, or whose basis record is absent / malformed, read on the host with no OCCT
- WHEN `step_import_native` resolves the edge
- THEN it SHALL return a NULL Shape (DECLINE) without constructing any edge or solid, so the engine can fall through to OCCT — no fabricated-basis edge is produced

#### Scenario: An AP242 PMI entity still never triggers a decline (host)
- GIVEN an ISO-10303-21 AP242 buffer carrying an in-slice solid PLUS PMI / annotation entities and additive plane-angle / PMI unit contexts, read on the host with no OCCT
- WHEN `step_import_native` runs its unit-context gate and its assembly-trigger scan
- THEN the PMI / annotation entities SHALL be SKIPPED (they SHALL NOT fail the mm length gate and SHALL NOT force the assembly path) AND the solid SHALL import; the file SHALL NOT decline merely because AP242 PMI entities are present (unchanged by this change)

## ADDED Requirements

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
