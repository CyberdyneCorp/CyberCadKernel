# native-exchange

This change (Phase 4 #8) closes the LAST `SURFACE_OF_REVOLUTION` gap in the native STEP import reader, in
two tracks each behind an HONEST per-track gate. It **extends** the working reader (landed by
`add-native-step-revolution-quadrics`, which reduces a `LINE` parallel/oblique/perpendicular →
cylinder/cone/plane and an on-axis `CIRCLE` → sphere) so that:

- **(T1) An OFF-AXIS `CIRCLE` / arc generatrix, and the direct `TOROIDAL_SURFACE` keyword,** map onto a
  **NEW** native `FaceSurface::Kind::Torus` (`math::Torus` already exists; the native-topology delta adds
  the kind, the native-tessellation delta adds an ADDITIVE torus mesh path). This LANDS only if the torus
  mesh path is ADDITIVE + PROVEN byte-identical for every existing mesh AND the torus face reconstructs
  watertight; otherwise T1 **keeps the current OCCT decline**.
- **(T2) An `ELLIPSE` or `B_SPLINE_CURVE_WITH_KNOTS` generatrix** maps onto a native **rational**
  `FaceSurface::Kind::BSpline` — the exact revolved rational tensor-product B-spline (the revolution's
  rational-quadratic full circle in `u` ⊗ the profile in `v`). This LANDS only if the surface is faithfully
  representable AND self-verifies watertight; otherwise T2 **keeps the current OCCT decline**.

Both tracks map ONLY geometry the file exactly describes, VERIFIED to pass through the profile before
emission, then gated by the engine watertight self-verify. No `cc_*` ABI change; the default engine stays
OCCT. The tessellator change is ADDITIVE-ONLY (a new torus mesh branch) and the STEP writer is preferred
UNCHANGED (OCCT-authored fixtures).

> NOTE (honest scope): still declined → OCCT after this change — a **skew** oblique line (hyperboloid), a
> directly-authored **arbitrary rational (weighted) B-spline** surface, a general **swept / bounded /
> offset** surface (`SURFACE_OF_LINEAR_EXTRUSION`, `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`,
> `CURVE_BOUNDED_SURFACE`, …), and everything prior slices declined; and — for whichever track does NOT
> meet its honest gate — the `TOROIDAL_SURFACE` / off-axis-circle revolution (T1) and/or the ellipse /
> B-spline revolution (T2). IGES import/export stay OCCT `IGESControl_*`. A general native STEP/AP242
> reader + IGES + a general-curved kernel still block #8 `drop-occt`; this change does NOT unblock it.
> **No surface or solid is ever fabricated; no existing tessellation path is ever perturbed; no tolerance
> is ever weakened. A revolution the reader cannot represent faithfully AND self-verify watertight
> DECLINES.**

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
(non-rational), a **`TOROIDAL_SURFACE`** (→ native `Kind::Torus`, when T1 lands), or a
**`SURFACE_OF_REVOLUTION`** that maps to a native surface — a straight generatrix **parallel** (→
cylinder), **oblique-meeting** (→ cone), or **perpendicular** (→ plane); an **on-axis circle / arc** (→
sphere); an **off-axis circle / arc** (→ **torus**, `Kind::Torus`, when T1 lands); or an **ellipse /
B-spline** profile (→ a **rational `Kind::BSpline`** revolved surface, when T2 lands) — and the edge
curves of kind `LINE`, `CIRCLE`, `ELLIPSE`, `B_SPLINE_CURVE_WITH_KNOTS` (non-rational), or a
**`TRIMMED_CURVE`** whose basis is one of those. A full periodic **sphere** face OCCT emits as a single
seam+double-pole face SHALL be reconstructed as a native `Sphere` **bare periodic surface** (NULL outer
wire) as before. A full periodic **torus** face (doubly periodic, NO pole) SHALL be reconstructed
watertight per the diagnosed OCCT bound (a seam EDGE_LOOP trimmed face, or a doubly-periodic bare-surface
face analogous to the sphere path extended to two seams) — and if it cannot close watertight within the
additive budget, the torus SHALL DECLINE. Specifically:

- **A `TRIMMED_CURVE`** SHALL be unwrapped to its basis curve (recursively; B-spline basis takes its
  `[first,last]` from the `PARAMETER_VALUE` trims clamped to the clamped knot span, analytic basis keeps
  the vertex-derived range) exactly as the landed trimmed-curve slice does (unchanged).
- **A `TOROIDAL_SURFACE`** `('',#axis2placement, major_radius, minor_radius)` SHALL be mapped by resolving
  the `AXIS2_PLACEMENT_3D` frame and the two trailing reals and building a native `FaceSurface` of kind
  `Torus` (`radius` = major, `minorRadius` = minor) — reconstructed watertight (T1) or DECLINED if the
  additive torus path does not close.
- **A `SURFACE_OF_REVOLUTION`** `('',#profile,#axis1)` SHALL be mapped by resolving the axis
  (`AXIS1_PLACEMENT` — origin + one direction, `$` axis → +Z) and the **profile** curve, then classifying
  the profile + axis by MEASUREMENT (never by a keyword) and mapping it to the EXACT native surface it
  sweeps, VERIFIED to pass through the profile within a scale-relative tolerance before emission:
  - a straight `LINE` **parallel** → native `Cylinder`; **oblique meeting** the axis → native `Cone`;
    **perpendicular** → native `Plane` (all landed, unchanged);
  - a `CIRCLE` / arc **centred ON the axis** with its plane **containing the axis** → native `Sphere`
    (landed, unchanged);
  - a `CIRCLE` / arc **centred OFF the axis** whose plane admits a ring torus → a native **`Torus`**
    (`radius` = the perpendicular distance from the circle centre to the axis = major; `minorRadius` = the
    circle radius = minor; frame origin on the axis, Z = the axis), VERIFIED the generatrix circle lies on
    the torus tube (**T1**);
  - an **`ELLIPSE`** or **`B_SPLINE_CURVE_WITH_KNOTS`** profile → a native **rational `Kind::BSpline`**
    surface: the revolution's rational-quadratic full circle in `u` tensored with the profile's own
    rational-or-nonrational representation in `v` (poles `P_ij` placed on the revolution circle at each
    profile pole, weights `w_ij = w^u_i · w^v_j`), VERIFIED sampled profile points lie on the reconstructed
    surface (**T2**).

  In **every** other case, AND whenever the mapped torus face (T1) or revolved B-spline face (T2) does not
  reconstruct watertight, the reader SHALL DECLINE (NULL → OCCT): a `CIRCLE` whose plane does not admit a
  ring torus (degenerate), a **skew** oblique `LINE` (a hyperboloid of one sheet — no native kind), a
  `LINE` **on** the axis (degenerate), a **degenerate axis**, a profile whose revolution is not faithfully
  representable, and any mapped face that fails the faithful-reduction guard or the watertight self-verify.

The reader SHALL remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` /
`EngineShape` type. It SHALL make the tessellator change ADDITIVE-ONLY (a new `Kind::Torus` mesh branch
that does not perturb any existing mesh path — proven byte-identical), SHALL prefer to leave the STEP
writer unchanged (OCCT-authored fixtures), SHALL NOT import PMI / annotation entities as geometry, and
SHALL NOT fabricate a curve, a surface, a trim, a placement, or a solid the file does not describe, nor
weaken any tolerance.

#### Scenario: A native-written box imports back to the same solid (host round-trip)
- GIVEN a native-built axis-aligned box `Solid` serialized by `step_export_native` to an ISO-10303-21 buffer, on the host with no OCCT
- WHEN `step_import_native` reads the buffer back and the result is tessellated
- THEN the reader SHALL return a `Solid` that is valid + watertight AND whose volume, bounding box, and face / edge / vertex counts / topology match the original box EXACTLY (the reader inverts the writer)

#### Scenario: A TOROIDAL_SURFACE face maps to a native torus or declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a face over a `TOROIDAL_SURFACE('',#axis2,major,minor)`, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL build a native `FaceSurface` of kind `Torus` (`radius` = major, `minorRadius` = minor) AND — when the additive torus mesh path + face reconstruction close watertight (T1 lands) — the assembled solid SHALL be valid + watertight with the torus volume `2·π²·R·r²` and matching bbox; OTHERWISE it SHALL return a NULL Shape (DECLINE) so the engine falls through to OCCT — never a fabricated or non-watertight torus

#### Scenario: A SURFACE_OF_REVOLUTION of an off-axis circle maps to a native torus or declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is a `CIRCLE` / arc whose centre is OFF the axis and whose plane admits a ring torus, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL build a native `Torus` (`radius` = the perpendicular distance from the circle centre to the axis, `minorRadius` = the circle radius), VERIFIED the generatrix circle lies on the torus tube, AND — when T1 lands — the assembled solid SHALL be valid + watertight and identical to the `TOROIDAL_SURFACE`-keyword-equivalent solid; OTHERWISE it SHALL DECLINE (NULL) so the engine falls through to OCCT

#### Scenario: A SURFACE_OF_REVOLUTION of an ellipse / B-spline profile maps to a native rational B-spline or declines honestly (host)
- GIVEN an in-scope ISO-10303-21 buffer with a `SURFACE_OF_REVOLUTION` face whose profile is an `ELLIPSE` or a `B_SPLINE_CURVE_WITH_KNOTS`, read on the host with no OCCT
- WHEN `step_import_native` resolves the surface
- THEN it SHALL build the EXACT revolved rational tensor-product B-spline (`Kind::BSpline`, the rational-quadratic full circle in `u` ⊗ the profile in `v`), VERIFIED sampled profile points lie on the surface, AND — when the surface is faithfully representable and self-verifies watertight (T2 lands) — the assembled solid SHALL be valid + watertight and match the OCCT re-import within tolerance; OTHERWISE it SHALL DECLINE (NULL) so the engine falls through to OCCT — never a mangled or approximate surface

#### Scenario: The on-axis circle / line quadric reductions and prior slices are unchanged (host)
- GIVEN in-scope ISO-10303-21 buffers exercising a `LINE` parallel / oblique-meeting / perpendicular and an on-axis `CIRCLE` `SURFACE_OF_REVOLUTION`, plus the trimmed-curve, full-sphere bare-periodic, quadric, bspline-face, and rigid / uniform-scale / mirror assembly cases, read on the host with no OCCT
- WHEN `step_import_native` resolves each
- THEN the cylinder / cone / plane / sphere reductions and every prior import path SHALL behave EXACTLY as before (the new torus + general-revolution arms are additive; adding `Kind::Torus` and the `minorRadius` field leaves every existing kind byte-identical)

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

## ADDED Requirements

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
