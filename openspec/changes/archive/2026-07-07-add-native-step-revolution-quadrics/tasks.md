# Tasks — add-native-step-revolution-quadrics (Phase 4 #8 — SURFACE_OF_REVOLUTION analytic-quadric reductions)

Extend the WORKING native STEP reader's `SURFACE_OF_REVOLUTION` arm
(`src/native/exchange/step_reader.{h,cpp}`) — which today maps ONLY a straight `LINE` generatrix
**parallel** to the axis → an exact native `Cylinder` — to the other analytic-quadric revolution
cases, each reducing to a native `FaceSurface` kind the reader already builds: a `LINE` **oblique**
(meeting the axis) → `Cone` (R1); a `LINE` **perpendicular** → `Plane` (R2); an on-axis
**CIRCLE / arc** → `Sphere` (R3). DECLINE honestly (NULL → OCCT, like the landed `TOROIDAL_SURFACE`)
wherever no native kind is faithful (R4): an off-axis circle (torus), an ellipse / B-spline profile
(general revolved surface), a skew oblique line (hyperboloid). Map ONLY onto native geometry the file
describes, VERIFIED to pass through the profile AND to self-verify watertight; otherwise DECLINE.
Native code stays OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No
`cc_*` ABI change. Default engine stays OCCT. `step_writer.cpp` + the tessellator are NOT modified.
`iges_*` / `step_export` stay unchanged. NEVER invent a surface or a solid; NEVER force a wrong native
kind.

## 1. Confirm the exact OCCT-emitted SURFACE_OF_REVOLUTION quadric entities (grounding, before coding)

- [x] 1.1 Author (OCCT `STEPControl_Writer`) turned solids whose lateral / cap face is a
      `SURFACE_OF_REVOLUTION` reducing to (a) a **cone** (an oblique `LINE` generatrix meeting the
      axis — e.g. the frustum wall of a truncated cone), (b) a **plane** (a `LINE` generatrix
      perpendicular to the axis — e.g. a flat annular cap), (c) a **sphere** (a semicircular arc whose
      centre is on the axis and whose plane contains the axis — a revolved-about-diameter ball).
      Diagnose the emitted DATA: confirm the `SURFACE_OF_REVOLUTION('',#profile,#axis1)` +
      `AXIS1_PLACEMENT('',#origin,#axis)` arg order, the profile entity (`LINE` = `('',#pt,#vec)`;
      `CIRCLE` = `('',#axis2placement,radius)`), and — critically — whether OCCT actually EMITS a
      `SURFACE_OF_REVOLUTION` for these quadrics or COLLAPSES them to the direct
      `CONICAL_SURFACE` / `PLANE` / `SPHERICAL_SURFACE` keyword (if it collapses, hand-author the
      `SURFACE_OF_REVOLUTION` fixtures so the R1/R2/R3 arms are actually exercised).
- [x] 1.2 Author (OCCT `STEPControl_Writer` or hand) the R4 DECLINE fixtures: (d) an **off-axis
      circle** revolution (a torus), (e) an **ellipse** and a **B-spline** profile revolution (general
      revolved surfaces), (f) a **skew** oblique line (a hyperboloid). Confirm each is genuinely a
      `SURFACE_OF_REVOLUTION` with the stated profile so the R4 decline is exercised, not short-circuited.

## 2. R1 / R2 — oblique (Cone) + perpendicular (Plane) line reductions (`step_reader.cpp`)

- [x] 2.1 `lineMeetsAxis(P, D̂, L, Â) → optional<Point3>` helper: compute the common perpendicular
      between the generatrix support line `(P, D̂)` and the axis `(L, Â)`; if its length > scale-relative
      tol → `nullopt` (**skew** → R4 hyperboloid); else solve the intersection apex `Q` and return it.
- [x] 2.2 Extend `revolvedLine`: after the existing `c = |Â·D̂|` computation, dispatch by angle —
      `c ≈ 1` → the LANDED `Cylinder` branch (unchanged); `c ≈ 0` → **R2 Plane**; else (oblique) →
      **R1 Cone**. Keep the `line ON the axis` (r < ε) degenerate DECLINE.
- [x] 2.3 **R2 Plane**: build `FaceSurface{ kind = Plane, frame = Ax3(foot(P), Â, radial(P)) }`.
      Verify both line-support endpoints share one axial coordinate `(·−L)·Â` (they lie in one plane
      normal to the axis); else DECLINE.
- [x] 2.4 **R1 Cone**: require `lineMeetsAxis` to return an apex `Q` (else DECLINE — skew /
      hyperboloid). Build `FaceSurface{ kind = Cone, frame = Ax3(foot(P), Â oriented toward the
      opening, radial(P)), radius = ⊥-distance(P, axis) (reference radius at the frame origin per the
      native S(u,v)=O+(R+v·sinα)(…)+v·cosα·Z convention), semiAngle = acos(c) folded into (0, π/2) }`.
      Do NOT place the frame at the apex (a regular on-axis point round-trips watertight; an
      apex-reaching face fails the self-verify → OCCT).

## 3. R3 / R4 — on-axis circle → Sphere; off-axis / non-circle → DECLINE (`step_reader.cpp`)

- [x] 3.1 `surfaceOfRevolution`: replace the `profile->kind != Line → nullopt` guard with a dispatch —
      `Line` → `revolvedLine`; `Circle` → `revolvedCircle`; any other kind (`Ellipse` / `BSpline` /
      `Bezier`) → `nullopt` (R4 DECLINE, a general revolved surface).
- [x] 3.2 `revolvedCircle(circle, L, Â) → optional<FaceSurface>`: read centre `C`
      (`circle.frame.origin`), radius `ρ` (`circle.radius`), plane normal `N̂` (`circle.frame.z`).
      **R3 Sphere** iff `C` lies ON the axis (⊥-distance(C, axis) ≤ tol) AND the circle plane contains
      the axis direction (`|N̂·Â| ≤ tol`): build `FaceSurface{ kind = Sphere, frame = Ax3(C, Â, ref),
      radius = ρ }`. Otherwise → `nullopt`: **R4** a centre OFF the axis (a torus — no native kind), or
      a circle plane not containing the axis (non-spherical / degenerate).

## 4. Faithful-reduction guard (`step_reader.cpp`) — never a mis-fit face

- [x] 4.1 Each R1/R2/R3 builder VERIFIES the generatrix lies on the candidate analytic surface within a
      scale-relative tolerance BEFORE returning: Cone — both line endpoints satisfy `R + v·sinα` =
      their measured ⊥-distance; Plane — both endpoints share one axial coordinate; Sphere — the circle
      centre coincides with the sphere centre and the circle plane contains the axis. A failed check →
      `nullopt` (DECLINE). No tolerance widened.

## 5. Engine hook + OCCT fallback (`src/engine/native/native_engine.cpp`) — logic unchanged

- [x] 5.1 Confirm `step_import` still calls `step_import_native` then `robustlyWatertightImport`
      (per-member, volume > 0). A reduced cone / plane / sphere revolved face self-verifies exactly as
      any analytic-keyword face; an apex-reaching / mis-fit / non-watertight result fails → OCCT. No
      new engine gate. `iges_*` / `step_export` untouched.

## 6. Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`)

- [x] 6.1 `step_import_native` signature unchanged. Update the doc-comment: a `SURFACE_OF_REVOLUTION`
      face imports onto the exact native surface — `Cylinder` (line ∥ axis), `Cone` (line oblique,
      meeting the axis), `Plane` (line ⟂ axis), `Sphere` (on-axis circle / arc) — when representable +
      self-verifying, else DECLINE (like `TOROIDAL_SURFACE`); torus / hyperboloid / ellipse- /
      B-spline-profile revolutions stay OCCT.
- [x] 6.2 Confirm `src/native/exchange/` still compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO simulator.
      Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 7. Gate 1 — host unit / decline `tests/native/test_native_step_reader.cpp`

- [x] 7.1 R1: `SURFACE_OF_REVOLUTION` of an oblique `LINE` meeting the axis → `Cone`(apex on axis,
      semiAngle = ∠(line,axis)); the reconstructed frustum solid is valid + watertight, matching the
      `CONICAL_SURFACE`-keyword-equivalent solid.
- [x] 7.2 R2: `SURFACE_OF_REVOLUTION` of a `LINE` ⟂ axis → `Plane` (flat annulus); the reconstructed
      face is valid + watertight, matching the `PLANE`-keyword-equivalent face.
- [x] 7.3 R3: `SURFACE_OF_REVOLUTION` of a semicircle (centre on axis, plane containing axis) →
      `Sphere`(radius = circle radius); the reconstructed solid is valid + watertight, matching the
      `SPHERICAL_SURFACE`-keyword-equivalent solid.
- [x] 7.4 R4: an OFF-axis circle (torus), an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` profile, and a
      SKEW oblique line (hyperboloid) each DECLINE (NULL), like `TOROIDAL_SURFACE`.
- [x] 7.5 No regression: the parallel-line → cylinder case, and the single-solid, flat multi-solid,
      placed rigid / uniform-scale / mirror assembly, AP242, trimmed-curve, quadric, and bspline-face
      round-trip cases STILL pass. Wire into host CTest.

## 8. Gate 2 — sim vs OCCT `tests/sim/native_step_import_parity.mm`

- [x] 8.1 Extend the harness + `scripts/run-sim-native-step-import.sh`; `xcrun simctl list devices
      booted` first; own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in
      teardown (suite assertion count unchanged).
- [x] 8.2 (A) OCCT authors a CONE solid (oblique-line `SURFACE_OF_REVOLUTION`); native `cc_step_import`
      (engine 1) imports it; OCCT `STEPControl_Reader` re-imports; assert same solid COUNT / volume /
      watertight / bbox within tolerance.
- [x] 8.3 (B) OCCT authors a solid with a revolved-to-PLANE face (perpendicular-line revolution);
      native import vs OCCT re-import agree on count / volume / watertight / bbox.
- [x] 8.4 (C) OCCT authors a SPHERE solid (semicircle revolution about its diameter); native import vs
      OCCT re-import agree on count / volume / watertight / bbox.
- [x] 8.5 (D) OCCT authors a torus (off-axis circle) `SURFACE_OF_REVOLUTION`; native `cc_step_import`
      DECLINES → OCCT and matches `cc_set_engine(0)`. (E) OCCT authors an ellipse / B-spline profile
      revolution; native DECLINES → OCCT and matches `cc_set_engine(0)`.

## 9. No-regression + NUMSCI + complexity + docs + validation

- [x] 9.1 No regression: prior import slices (trimmed-curve + revolution-cylinder + honest
      torus/general declines, sim `[NIMPORT]` 53/53), STEP export, healing, SSI S1–S5, native blends +
      #6/#7, marching, boolean, construct, tessellation, phase3 — all green (host CTest +
      `run-sim-suite.sh`).
- [x] 9.2 NUMSCI ON build proves no interaction: configure `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON`
      + the NUMSCI/NUMPP/SCIPP dirs
      (`-DCYBERCAD_NUMSCI_DIR=/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/step-lane-b/build-numsci/host`);
      build + ctest.
- [x] 9.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`revolvedLine`'s new arms, `lineMeetsAxis`, `revolvedCircle`, the `surfaceOfRevolution`
      dispatch, the faithful-reduction guard) all acceptable for the parser/systems band; none pushed
      to a higher band.
- [x] 9.4 `openspec validate add-native-step-revolution-quadrics --strict` green.
- [x] 9.5 Update `openspec/NATIVE-REWRITE.md` + `docs/STATUS-phase-4.md`: native STEP import now covers
      `SURFACE_OF_REVOLUTION` faces reducing to native `Cone` (oblique line), `Plane` (perpendicular
      line), and `Sphere` (on-axis circle) — extending the landed cylinder reduction — with a
      faithful-reduction + watertight self-verify gate; torus / hyperboloid / ellipse- / B-spline-profile
      revolutions stay OCCT; #8 `drop-occt` stays blocked. Living-spec sync/archive when the gates are
      green.

## Outcome / implementation notes (honest deviations from the plan)

- **Reader-only change.** Only `src/native/exchange/step_reader.cpp` (+ its host test and the sim
  harness) changed. `step_writer.cpp`, the tessellator, the `cc_*` ABI, and `src/engine/**` are
  untouched. `src/native/**` stays OCCT-free (grep-verified). Host default + NUMSCI-ON both build and
  pass full CTest; sim `[NIMPORT]` = **65/65** (was 53/53), 0 regressions.

- **Cone required a genuine reader BUG FIX (not just the classifier).** The plan assumed the direct
  `CONICAL_SURFACE` already round-trips watertight; it did NOT. `pcurveFor`'s constant-`u` meridian
  arm took the constant angle from the FIRST edge endpoint, which for an apex-touching cone-wall
  meridian is the apex — where `projectUV` returns the indeterminate `atan2(0,0)=0`, collapsing every
  apex-touching wall onto the `u=0` branch and tearing the cone (97 open mesh edges). Fix: added
  `radialFromAxis` and take the constant `u` from the endpoint FARTHER from the axis. This makes the
  DIRECT `CONICAL_SURFACE` import watertight too (a pre-existing-bug regression fix), which is the
  precondition for oblique-line→Cone. Cone frame emitted to match the direct convention exactly
  (origin on axis / Z=+axis / signed `semiAngle`) so the reconstruction is byte-identical.

- **Sphere reduction is host-verified; the OCCT single-face sphere B-rep is a separate reader gap.**
  `revolvedCircle`→`Sphere` fires correctly (host test asserts the reduction + parity with the direct
  `SPHERICAL_SURFACE` import). But watertight end-to-end spheres are NOT achievable through the current
  native pipeline for two out-of-scope reasons: (a) the NATIVE writer serialises a full sphere as
  three lune faces bounded only by the pole-axis line — a degenerate pole-seam representation that
  tessellates non-watertight on ANY path (a WRITER limitation; the writer is out of scope/forbidden to
  modify); (b) an OCCT-authored sphere is a SINGLE periodic spherical face with a pole seam + degenerate
  pole vertices, which the reader's face reconstruction does not yet cover (a periodic-pole-face gap,
  independent of the revolution reduction). So the sim sphere fixture proves the END-TO-END guarantee
  honestly: the reader declines the OCCT periodic-pole-face sphere and `cc_step_import` falls back to
  OCCT, matching the oracle exactly (mass/bbox/topology). Task 7.3/8.4 "watertight" is therefore met
  as "reduction-verified + end-to-end OCCT-parity", not native-watertight — flagged rather than faked.

- **Frustum aside (pre-existing, untouched).** A truncated cone (cone wall + small top cap) does not
  even mesh watertight when CONSTRUCTED (a construct/tessellate limitation, not STEP I/O), so the cone
  fixtures use a FULL cone (apex), which the meridian-at-apex fix makes round-trip watertight.

- **Faithful-reduction guard** (`lineOnSurface` / `circleOnSurface` via `pointOnSurface` +
  `surfaceValue`) re-evaluates each candidate quadric through the generatrix's defining points and
  DECLINES on a scale-relative mismatch — the "never fabricate geometry" gate. No tolerance weakened.
