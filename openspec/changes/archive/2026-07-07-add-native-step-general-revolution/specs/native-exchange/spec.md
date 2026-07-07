# native-exchange

This change (Phase 4 #8) closes the LAST `SURFACE_OF_REVOLUTION` gap the `add-native-step-torus` slice
deferred: an `ELLIPSE` or `B_SPLINE_CURVE_WITH_KNOTS` generatrix **general revolution** (**T2**). The torus
slice landed T1 (off-axis circle / `TOROIDAL_SURFACE` → native `Kind::Torus`, watertight through the
already-proven sphere bare-periodic path, `face_mesher.h` untouched) and kept the honest OCCT decline for
T2. This change maps the T2 profile onto a native **rational** `FaceSurface::Kind::BSpline` — the EXACT
revolved rational tensor-product B-spline (the revolution's standard rational-quadratic full circle in `u`,
control net revolved at the standard knot angles `0, π/2, π, 3π/2, 2π` with the standard rational weights
`1, 1/√2, 1, 1/√2, 1`, tensored with the profile in `v`) — behind an HONEST watertight gate.

The pivotal substrate check PASSES on evaluation: `FaceSurface::Kind::BSpline` carries `weights` (rational)
and the tessellator already meshes rational B-spline faces (`nurbsSurfacePoint` / `nurbsSurfaceDerivs` in
`surface_eval.h`), so NO topology / math / tessellator change is needed to REPRESENT or EVALUATE the
surface. The genuine open gate is **watertight meshing of a `u`-periodic revolved rational B-spline face
(seam weld + a profile-endpoint axis pole)** through the freeform grid path — which a `Kind::BSpline` face
uses (there is no analytic bare-periodic reconstruction for it). This change LANDS T2 ONLY if that
self-verifies watertight (with NO tessellator perturbation, or with a STRICTLY ADDITIVE + byte-identical
mesher branch); otherwise it keeps the current OCCT decline with NO dead code.

Both the mapping and the guard emit ONLY geometry the file exactly describes, VERIFIED to pass through the
profile before emission, then gated by the engine watertight self-verify. No `cc_*` ABI change; the default
engine stays OCCT; the STEP writer is UNCHANGED (OCCT-authored fixtures).

> NOTE (honest scope): still declined → OCCT after this change — a **rational (weighted)**
> `B_SPLINE_CURVE_WITH_KNOTS` profile (the curve reader is non-rational only, so a weighted-profile
> revolution declines at the curve level upstream), a directly-authored **arbitrary rational (weighted)
> B-spline surface**, a **skew** oblique line (hyperboloid), a general **swept / bounded / offset** surface
> (`SURFACE_OF_LINEAR_EXTRUSION`, `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`,
> …), and everything prior slices declined; and the ellipse / B-spline revolution ITSELF whenever its
> reconstructed periodic B-spline does not self-verify watertight OR would require perturbing an existing
> tessellation path. IGES import/export stay OCCT `IGESControl_*`. A general native STEP/AP242 reader + IGES
> + a general-curved kernel still block #8 `drop-occt`; this change does NOT unblock it. **No surface or
> solid is ever fabricated; no existing tessellation path is ever perturbed; no tolerance is ever weakened;
> no dead reconstruction code is committed. A revolution the reader cannot represent faithfully AND
> self-verify watertight DECLINES.**

## MODIFIED Requirements

### Requirement: Native STEP AP203 import of a writer-emitted manifold-solid-brep

The native exchange library SHALL provide `step_import_native(path)` that reads an ISO-10303-21 (STEP
Part 21) file — **independently of its `FILE_SCHEMA` header** (AP203, AP214 `AUTOMOTIVE_DESIGN`, or AP242
are all accepted; the reader gates on entities + the mm length-unit context, not the schema string, and
**skips** AP242 PMI / annotation entities and additive plane-angle / solid-angle / PMI unit contexts) —
and reconstructs a native `topology::Shape`: a `Solid` (one root `MANIFOLD_SOLID_BREP`), a **flat**
`Compound` (several co-equal roots, no transform tree), or a **placed** `Compound` (a single-level
assembly composed by a rigid / uniform-scale / mirror transform, else DECLINE, with mirror
orientation-compensation so each placed solid self-verifies watertight; the tessellator SHALL NOT be
modified and no normal SHALL be fabricated). The faces SHALL carry surfaces of kind `PLANE`,
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

## ADDED Requirements

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
