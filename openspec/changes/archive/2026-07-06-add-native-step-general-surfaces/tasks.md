# Tasks — add-native-step-general-surfaces (Phase 4 #7 — TRIMMED_CURVE edges + SURFACE_OF_REVOLUTION faces)

> **COMPLETION STATUS (as archived — honest reconciliation).** The checklist below is the original
> design plan. What actually LANDED (verified: host `test_native_step_reader` 26/26, sim `[NIMPORT]`
> 53/53 vs OCCT, no regression) is narrower, because the deferred cases could not pass the engine's
> watertight self-verify and so DECLINE honestly rather than being forced:
> - **LANDED — T1:** §1.1, §2.1, §2.2 (narrowed: caches the `PARAMETER_VALUE` trims only — no
>   point-trim / `sense` / `master` handling), §2.3 (narrowed: trim-driven range for a **B-spline
>   basis only**; analytic bases keep the vertex-derived range), §8.x, §9.1 (line/circle/B-spline
>   accept + watertight), §9.5, §10.1–10.2, §11.1–11.4.
> - **LANDED — T2 (cylinder case only):** §1.2, §3.1 (`axis1placement`), §3.2 + §4.1's **line-∥-axis
>   → Cylinder** branch (`surfaceOfRevolution` + `revolvedLine`), §6.1 realized as a **blanket
>   DECLINE of every non-cylinder revolution**, §9.2 (line-∥→cylinder + oblique-line-declines), §9.4
>   (off-axis-arc declines), §10.3 (narrowed: cylinder only), §10.5 (torus decline → OCCT).
> - **DEFERRED — NOT implemented → honest DECLINE → OCCT (like `TOROIDAL_SURFACE`):** §4.1's cone /
>   plane / sphere reductions, the entire §5.1 T2b exact rational revolved B-spline, §5.2 T2b pcurve
>   synthesis, §9.3 + §10.4 T2b build cases. The code has **no** `reduceToQuadric` and **no**
>   `revolveToRationalBSpline`; a non-parallel line or any non-line profile returns `nullopt`. The
>   living spec + STATUS docs claim ONLY the landed behavior.


Widen the WORKING native STEP reader (`src/native/exchange/step_reader.{h,cpp}`) to import two
general-surface families it currently DECLINES: **(T1)** a `TRIMMED_CURVE` edge onto the native
trimmed edge, and **(T2)** a `SURFACE_OF_REVOLUTION` face onto the exact native surface that
represents it (analytic quadric T2a, or exact rational revolved B-spline T2b), DECLINING honestly
(NULL → OCCT, like the landed `TOROIDAL_SURFACE`) wherever no native kind is faithful (T2c). Map
ONLY onto native geometry the file describes and the un-modified tessellator renders correctly;
otherwise DECLINE. Native code stays OCCT-free + host-buildable
(`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*` ABI change. Default engine stays OCCT.
`step_writer.cpp` + the tessellator are NOT modified. `iges_*` / `step_export` stay unchanged.
NEVER invent a curve, a surface, a trim, or a solid; NEVER force a wrong native kind.

## 1. Confirm the exact OCCT-emitted TRIMMED_CURVE + SURFACE_OF_REVOLUTION entities (grounding, before coding)

- [ ] 1.1 Author (OCCT `STEPControl_Writer`) a solid whose one edge is a `TRIMMED_CURVE` — a circular
      arc trimmed by parameters (ideally one that sweeps clockwise or through `> π` so the vertex-only
      heuristic would mis-range it). Diagnose the emitted DATA: the exact `TRIMMED_CURVE('',#basis,
      (trim1),(trim2),sense,master)` arg order, whether OCCT emits `PARAMETER_VALUE` and / or
      `CARTESIAN_POINT` trims, the `sense_agreement` (`.T.`/`.F.`) and `master_representation`
      enum values, and the parameter UNIT for a circle/ellipse trim (degrees vs radians per the unit
      context).
- [ ] 1.2 Author (OCCT `STEPControl_Writer`) turned / grooved solids whose lateral face is a
      `SURFACE_OF_REVOLUTION`: (a) a LINE profile (→ cylinder / cone), (b) a B-spline / ellipse
      profile (→ general turned face), (c) an off-axis circular arc (→ a torus / groove). Diagnose the
      emitted DATA: the exact `SURFACE_OF_REVOLUTION('',#profile,#axis1)` + `AXIS1_PLACEMENT('',
      #origin,#axis)` arg order, and confirm which profiles OCCT actually emits as
      `SURFACE_OF_REVOLUTION` vs collapses to an analytic keyword. Confirm whether OCCT ever emits
      `SURFACE_OF_REVOLUTION` for a quadric (T2a coverage) or only for genuine profiled / torus faces.

## 2. T1 — TRIMMED_CURVE → native trimmed edge (`step_reader.cpp`)

- [ ] 2.1 `trimmedCurve(const Record&) → optional<EdgeCurve>`: resolve the basis curve
      (`args[1]`, recursively via `curve()` — LINE / CIRCLE / ELLIPSE / B_SPLINE_CURVE_WITH_KNOTS,
      incl. a basis reached through the existing SURFACE_CURVE wrapper) and return it unchanged. A
      basis that is itself out of slice → `nullopt` (DECLINE, unchanged).
- [ ] 2.2 Add the `TRIMMED_CURVE` arm to `curve()` (before the final `nullopt`). Parse the two trims
      (`args[2]`,`args[3]` SETs) into a `TrimSpec{ hasParams, t0, t1, senseForward, masterIsParameter,
      p0, p1 }` (a `PARAMETER_VALUE(x)` REAL and / or a `CARTESIAN_POINT`), plus `sense_agreement`
      (`args[4]`) and `master_representation` (`args[5]`). Surface the `TrimSpec` to `edgeCurve()`
      (keyed by the EDGE_CURVE's 3D-curve `#id`, or returned alongside the `EdgeCurve`).
- [ ] 2.3 Trim-driven `[first,last]` in `edgeCurve()` / `curveRange()`: when the 3D curve resolved
      through a `TRIMMED_CURVE` with a `PARAMETER_VALUE` pair AND `master_representation` is
      `.PARAMETER.` (or `.UNSPECIFIED.` with only params present), set the edge range to the trims
      `[t0,t1]` (swapped when `sense_agreement = .F.`), matched to the curve kind's parametrization and
      the unit context (radians for the native circle/ellipse angle). Otherwise (point-only trims, or
      `master = .CARTESIAN.`) keep the existing vertex-derived `curveRange` (identical to today). A
      degenerate / inconsistent trim (`t0==t1`, param out of range, point matching no vertex) →
      DECLINE.

## 3. T2 — AXIS1_PLACEMENT + SURFACE_OF_REVOLUTION dispatcher (`step_reader.cpp`)

- [ ] 3.1 `axis1placement(id) → optional<pair<Point3,Dir3>>`: read `AXIS1_PLACEMENT('',#origin,#axis)`
      (origin `CARTESIAN_POINT`; `#axis` `DIRECTION`, `$` ⇒ +Z). Mirror `axis2placement`. A missing
      origin or a zero-length axis → `nullopt` (DECLINE).
- [ ] 3.2 `surfaceOfRevolution(const Record&) → optional<FaceSurface>`: read
      `SURFACE_OF_REVOLUTION('',#profile,#axis1)`; resolve `axis1placement(args[2])` +
      `curve(args[1])` (reusing the curve dispatcher incl. T1's TRIMMED_CURVE). Dispatch:
      `reduceToQuadric` (T2a) → else `isOffAxisArc` (T2c torus DECLINE) → else
      `revolveToRationalBSpline` (T2b) → else `nullopt` (T2c DECLINE). Add the `SURFACE_OF_REVOLUTION`
      arm to `surface()` (before the final `nullopt`); `TOROIDAL_SURFACE` etc. still DECLINE.

## 4. T2a — exact analytic reduction (`step_reader.cpp`)

- [ ] 4.1 `reduceToQuadric(profile, axis) → optional<FaceSurface>`: exact geometric classification
      (scale-relative tol) — a LINE ∥ axis → `Cylinder`(radius = ⊥-distance to axis, frame = axis); a
      LINE whose support meets the axis → `Cone`(apex, semiAngle = ∠(line,axis), reference radius per
      the native `Cone` convention); a LINE ⟂ axis → `Plane`; an on-axis CIRCLE / arc (center on the
      axis, plane containing it) → `Sphere`(center,radius). Build with the EXISTING `placedSurface`
      machinery so the reduced surface is byte-identical to the analytic-keyword surface. Anything
      else → `nullopt` (falls to T2b/T2c).

## 5. T2b — exact rational revolved B-spline (`step_reader.cpp`)

- [ ] 5.1 `revolveToRationalBSpline(profile, axis) → optional<FaceSurface>`: for an in-slice
      generatrix that did not reduce to a quadric (a `B_SPLINE_CURVE_WITH_KNOTS` or `ELLIPSE`
      profile), construct the EXACT surface of revolution as a rational tensor-product B-spline
      (Piegl–Tiller A8.1 `MakeRevolvedSurf`): U = rational-quadratic circle (degU = 2, `2m+1` poles,
      weights `{1,√2⁄2,…}`, knots per the swept angle), V = the profile (an ELLIPSE first expressed as
      its exact rational quadratic B-spline). Populate `FaceSurface{ kind = BSpline, degreeU = 2,
      degreeV, nPolesU, nPolesV, poles (row-major), weights, knotsU, knotsV }` — the EXACT
      representation, not an approximation. A profile out of the buildable set → `nullopt` (DECLINE).
- [ ] 5.2 pcurve synthesis for a revolved face: extend `pcurveFor` to emit the boundary pcurves in the
      revolved `(u = circle param, v = profile param)` plane (profile-end circles = `v = const` lines,
      seam / profile edges = `u = const` lines). If a faithful pcurve cannot be synthesized for a T2b
      face → that face DECLINES (NULL, never a wrong pcurve). NO tessellator change (its rational-NURBS
      path already exists + is verified).

## 6. T2c — honest DECLINE (`step_reader.cpp`)

- [ ] 6.1 `isOffAxisArc(profile, axis)`: a CIRCLE / arc whose center is OFF the axis → the revolved
      surface is a TORUS → `nullopt` (DECLINE), kept consistent with the landed `TOROIDAL_SURFACE`
      decline (no torus through the revolution back door). A rational / self-intersecting / out-of-slice
      profile, or a degenerate / profile-intersecting axis → DECLINE. The watertight self-verify
      (§7 engine gate) is the final arbiter for any T2a / T2b face; a face that does not self-verify →
      DECLINE → OCCT. NO tolerance widened to force a pass.

## 7. Engine hook + OCCT fallback (`src/engine/native/native_engine.cpp`) — logic unchanged

- [ ] 7.1 Confirm `step_import` still calls `step_import_native` then `robustlyWatertightImport`
      (per-member, volume > 0). A `TRIMMED_CURVE` edge and a reduced / revolved-B-spline face
      self-verify exactly as any other; a mis-trimmed / wrongly-revolved / non-watertight result fails →
      OCCT. No new engine gate. `iges_*` / `step_export` untouched.

## 8. Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`)

- [ ] 8.1 `step_import_native` signature unchanged. Update the doc-comment: `TRIMMED_CURVE` edges
      import onto the native trimmed edge; `SURFACE_OF_REVOLUTION` faces import onto the exact native
      surface (analytic quadric or exact rational revolved B-spline) when representable + self-verifying,
      else DECLINE (like `TOROIDAL_SURFACE`); torus / arbitrary-rational / out-of-slice swept surfaces
      stay OCCT.
- [ ] 8.2 Confirm `src/native/exchange/` still compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO simulator.
      Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 9. Gate 1 — host unit / decline `tests/native/test_native_step_reader.cpp`

- [ ] 9.1 T1: a `TRIMMED_CURVE` over a LINE / CIRCLE unwraps to the basis `EdgeCurve`; the edge's
      `[first,last]` comes from the trim `PARAMETER_VALUE`s, ranging a clockwise / `> π` arc CORRECTLY
      (a case vertex-only `curveRange` mis-ranges); a point-only trim reproduces the vertex-derived
      range; a rational-basis or `t0==t1` TRIMMED_CURVE DECLINES.
- [ ] 9.2 T2a: `SURFACE_OF_REVOLUTION` of a LINE ∥ axis → `Cylinder`; LINE meeting axis → `Cone`; LINE
      ⟂ axis → `Plane`; on-axis arc → `Sphere`; each reconstructed solid valid + watertight, matching
      the analytic-keyword-equivalent solid.
- [ ] 9.3 T2b: `SURFACE_OF_REVOLUTION` of a `B_SPLINE_CURVE_WITH_KNOTS` / `ELLIPSE` profile builds a
      `Kind::BSpline` `FaceSurface` with `weights` set (degU = 2); the assembled solid is kept iff it
      self-verifies watertight, else DECLINES (NULL) — both outcomes asserted.
- [ ] 9.4 T2c: `SURFACE_OF_REVOLUTION` of an OFF-axis circular arc DECLINES (NULL), like
      `TOROIDAL_SURFACE`.
- [ ] 9.5 No regression: the single-solid, flat multi-solid, placed rigid / uniform-scale / mirror
      assembly, AP242, quadric, and bspline-face round-trip cases STILL pass. Wire into host CTest.

## 10. Gate 2 — sim vs OCCT `tests/sim/native_step_import_parity.mm`

- [ ] 10.1 Extend the harness + `scripts/run-sim-native-step-import.sh`; `xcrun simctl list devices
      booted` first; own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in
      teardown (suite assertion count unchanged).
- [ ] 10.2 (A) OCCT authors a solid with a `TRIMMED_CURVE` edge; native `cc_step_import` (engine 1)
      imports it; OCCT `STEPControl_Reader` re-imports; assert same solid COUNT / volume / watertight /
      bbox within tolerance.
- [ ] 10.3 (B) OCCT authors a turned solid whose lateral face is a `SURFACE_OF_REVOLUTION` reducing to
      a cylinder / cone (T2a); native import vs OCCT re-import agree on count / volume / watertight /
      bbox.
- [ ] 10.4 (C) OCCT authors a turned solid whose face is a `SURFACE_OF_REVOLUTION` of a B-spline /
      ellipse profile (T2b); native import EITHER matches OCCT (count / volume / watertight / bbox) when
      it self-verifies, OR DECLINES → OCCT identical to `cc_set_engine(0)` — both outcomes asserted
      honest.
- [ ] 10.5 (D) OCCT authors a grooved solid whose face is a `SURFACE_OF_REVOLUTION` of an off-axis arc
      (a torus, T2c); native `cc_step_import` DECLINES → OCCT and matches `cc_set_engine(0)`.

## 11. No-regression + NUMSCI + complexity + docs + validation

- [ ] 11.1 No regression: prior import slices (flat multi-solid + bspline-face + rigid / uniform-scale
      / mirror assemblies + AP242, sim `[NIMPORT]` 41/41), STEP export, healing, SSI S1–S5, native
      blends + #6/#7, marching, boolean, construct, tessellation, phase3 — all green (host CTest +
      `run-sim-suite.sh`).
- [ ] 11.2 NUMSCI ON build proves no interaction: `bash scripts/build-numsci.sh host`; configure
      `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON` + the NUMSCI/NUMPP/SCIPP dirs
      (`-DCYBERCAD_NUMSCI_DIR=.../build-numsci/host`); build + ctest.
- [ ] 11.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`trimmedCurve`, the `edgeCurve`/`curveRange` trim override, `axis1placement`,
      `surfaceOfRevolution`, `reduceToQuadric`, `revolveToRationalBSpline`, the pcurve helper) all
      acceptable for the parser/systems band; none pushed to a higher band.
- [ ] 11.4 `openspec validate add-native-step-general-surfaces --strict` green.
- [ ] 11.5 Update `openspec/NATIVE-REWRITE.md` + `docs/STATUS-phase-4.md`: native STEP import now
      covers `TRIMMED_CURVE` edges + `SURFACE_OF_REVOLUTION` faces that map onto an exact native surface
      (analytic quadric or exact rational revolved B-spline) with a watertight self-verify gate; torus
      / arbitrary-rational / out-of-slice swept surfaces stay OCCT; #8 `drop-occt` stays blocked.
      Living-spec sync/archive when the gates are green.
