# Tasks — add-native-step-general-revolution (Phase 4 #8 — the last STEP-revolution gap: general / ellipse / B-spline revolution)

Close the `SURFACE_OF_REVOLUTION` gap the `add-native-step-torus` slice deferred: an `ELLIPSE` /
`B_SPLINE_CURVE_WITH_KNOTS` generatrix → a native **rational `FaceSurface::Kind::BSpline`** revolved
surface (the exact revolved rational tensor-product B-spline), behind an HONEST watertight gate, or an
honest DECLINE (NULL → OCCT) with **NO dead code and NO tessellator perturbation**. Map ONLY geometry the
file describes, VERIFIED to pass through the profile AND to self-verify watertight; otherwise DECLINE.
Native code stays OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*`
ABI change. Default engine stays OCCT. The tessellator is PREFERRED UNCHANGED (the rational B-spline eval
path already exists); it is touched ONLY if a periodic-seam / axis-pole close is genuinely required AND
provably additive + byte-identical for every existing mesh. `step_writer.cpp` stays unchanged
(OCCT-authored fixtures). `iges_*` / `step_export` stay unchanged. NEVER invent a surface or a solid; NEVER
weaken a tolerance; NEVER perturb an existing tessellation path.

## Pivotal check (settle BEFORE coding — decides land vs decline)

- [x] 0.1 CONFIRM the native rational substrate (already grepped; re-confirm on the branch): `FaceSurface`
      carries `weights` (`shape.h` ~L224, `empty ⇒ non-rational`) → `Kind::BSpline` is rational-capable;
      `math::nurbsSurfacePoint` / `nurbsSurfaceDerivs` exist (`math/bspline.h`); `surface_eval.h`
      (~L163–172, L216–220) already dispatches `Kind::BSpline` to the RATIONAL evaluators when `weights` is
      non-empty. → A rational B-spline surface EVALUATES through the existing mesh path. **Pivotal eval
      check PASSES; no topology/math change needed.**
- [x] 0.2 DIAGNOSE the OPEN gate (Task 1): a full revolution is `u`-periodic (seam `u=0≡2π`) and, if the
      profile touches the axis, has a degenerate pole. A `Kind::BSpline` face has NO bare-periodic
      reconstruction path (freeform grid only). Empirically determine whether the EXISTING freeform mesh
      path welds the `u`-seam + closes the axis pole watertight. This decides: land with zero tessellator
      change / land with an additive-proven mesher branch / **keep the OCCT decline** (§6.4).

## 1. Ground the exact OCCT-emitted general-revolution entities + the watertight behaviour (before coding)

- [x] 1.1 Author (OCCT `STEPControl_Writer`) a GENERAL-REVOLUTION solid: an `ELLIPSE` profile revolved
      about an axis (a spheroid / ellipsoid of revolution — e.g. revolve a half-ellipse whose endpoints sit
      ON the axis, giving a capped closed solid), and — if OCCT will emit one — a
      `B_SPLINE_CURVE_WITH_KNOTS` profile revolution. Confirm each is genuinely a `SURFACE_OF_REVOLUTION`
      with the stated profile (not collapsed to a quadric) so the T2 arm is exercised, and record the exact
      profile-entity args (`ELLIPSE` position + two semi-axes; B-spline degree/poles/mults/knots).
- [x] 1.2 DIAGNOSE how OCCT BOUNDS the revolved face and how the reconstructed native `Kind::BSpline` face
      MESHES: does the `u=0≡2π` seam weld watertight through the existing freeform grid? Does a
      profile-endpoint axis pole (a spheroid's poles) close watertight, or leave a hole? Record the verdict
      — it selects the outcome branch in Task 6.4.

## 2. Reader general-revolution mapping (`src/native/exchange/step_reader.cpp`)

- [x] 2.1 `surfaceOfRevolution`: replace the `Ellipse` / `BSpline` `default → nullopt` (~L987) with
      `if (profile->kind == K::Ellipse || profile->kind == K::BSpline)
      return revolvedProfileBSpline(*profile, ax->first, ax->second);`. The `Line` / `Circle` arms are
      unchanged.
- [x] 2.2 `revolutionCirclePoles(Q_j, L, A) → array<Point3,9>`: decompose `Q_j` into axial height
      `h = A·(Q_j−L)`, radial `ρ = (Q_j−L) − h·A`, `r = |ρ|`, `X = ρ/r`, `Y = A×X`, foot `O = L + h·A`;
      emit the 9 circle-polygon poles (5 on-circle at `O + r(cosθ_k·X + sinθ_k·Y)`, `θ = 0,π/2,π,3π/2,2π`;
      4 corner at `O + r√2·(cosφ·X + sinφ·Y)`, `φ = θ_k+45°`). `r = 0` (profile point ON axis) → all 9 = `O`
      (pole singularity — flagged for the §3 watertight gate).
- [x] 2.3 `promoteProfileToNurbs(profile) → {degreeV, knotsV, poles Q_j, weights ω_j}`: an `ELLIPSE`
      (semi-axes `a,b`, frame) → its exact rational-quadratic B-spline (9-pole `{1,1/√2,…}`, scaled `a` in
      `X`, `b` in `Y`; an elliptic ARC uses the covered sub-arc); a non-rational `B_SPLINE_CURVE_WITH_KNOTS`
      → used directly (`degree`, `knots`, `poles`, `ω_j = 1`).
- [x] 2.4 `revolvedProfileBSpline(profile, L, A) → optional<FaceSurface>`: assemble
      `degreeU=2`, `degreeV=p_v`, `nPolesU=9`, `nPolesV=n_v`, `poles[i*nPolesV+j] =
      revolutionCirclePoles(Q_j)[i]` (row-major, U outer — matches `gridOf` / `bsplineSurface`),
      `weights[i*nPolesV+j] = w^u_i · ω_j` with `w^u = {1,1/√2,1,1/√2,1,1/√2,1,1/√2,1}`,
      `knotsU = {0,0,0, π/2,π/2, π,π, 3π/2,3π/2, 2π,2π,2π}`, `knotsV = V`. Emit
      `FaceSurface{Kind::BSpline, …}`.
- [x] 2.5 `profileOnSurface(profile, surface)` guard: sample the profile `P(v_k)` at several `v_k` and
      assert each lies on the reconstructed surface at `u=0` (`nurbsSurfacePoint(...,0,v_k)`) within a
      scale-relative tolerance; a mismatch → `nullopt`. Mirrors `circleOnSurface`. Run it BEFORE returning.
- [x] 2.6 **HONEST-OUT GATE**: `revolvedProfileBSpline` emits the surface ONLY when the reconstructed face
      passes `profileOnSurface` AND (per Task 1.2 + the engine self-verify) reconstructs watertight (the
      `u`-seam welds; a profile-endpoint axis pole closes through the existing mesh path). Otherwise it
      returns `nullopt` (DECLINE → OCCT). No pole fabrication; no seam force-weld; no tolerance widened. If
      the honest verdict is DECLINE, `revolvedProfileBSpline` is NOT committed as reachable-but-broken code —
      it either lands watertight or the arm keeps `return std::nullopt`.

## 3. Tessellator — VERIFY-ONLY (preferred UNCHANGED)

- [x] 3.1 CONFIRM the reconstructed rational `Kind::BSpline` revolved face meshes through the EXISTING
      `surface_eval.h` rational path (`nurbsSurfacePoint` / `nurbsSurfaceDerivs`) with the `u`-seam welded
      and any axis pole closed — WITHOUT editing `face_mesher.h` / `surface_eval.h` / `trim.h`.
- [x] 3.2 **ADDITIVE-ONLY ESCALATION (only if 3.1 fails):** if a periodic-seam / axis-pole close for a
      `Kind::BSpline` face is genuinely required, add it as a NEW guarded branch that does NOT modify the
      Plane/Cylinder/Cone/Sphere/BSpline/Bezier/Torus paths, and PROVE it byte-identical for every existing
      mesh via Gate 3. If a clean additive branch is not achievable, **STOP — revert, and keep the OCCT
      decline** (Task 2.6). Report why plainly.

## 4. Engine hook + reader API + OCCT-free build

- [x] 4.1 Confirm `src/engine/native/native_engine.cpp` `step_import` still calls `step_import_native` then
      `robustlyWatertightImport` (per-member, volume > 0) with NO new engine gate — a revolved-B-spline face
      self-verifies exactly as any face; a leaky result → OCCT. `iges_*` / `step_export` untouched.
- [x] 4.2 `step_import_native` signature unchanged. Update `step_reader.h` / `native_exchange.h`
      doc-comments: an ellipse / B-spline `SURFACE_OF_REVOLUTION` imports onto a native rational
      `Kind::BSpline` when faithfully representable + self-verifying; else DECLINE → OCCT.
- [x] 4.3 Confirm `src/native/exchange/` + `src/native/tessellate/` + `src/native/topology/` still compile
      with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO simulator.
      Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 5. Gate 1 — host unit / decline (`tests/native/test_native_step_reader.cpp`)

- [x] 5.1 An ellipse (and, if authorable, a non-rational B-spline) `SURFACE_OF_REVOLUTION` maps to a
      rational `Kind::BSpline`, VERIFIED the profile lies on the surface. If T2 lands the reconstructed solid
      is valid + watertight and matches the analytic spheroid volume/bbox; if T2 is dropped the reduction is
      host-asserted and the end-to-end path DECLINES (NULL).
- [x] 5.2 No regression: the on-axis circle → Sphere, off-axis circle → Torus, line → cylinder/cone/plane,
      trimmed-curve, quadric, bspline-face, single/flat/placed-assembly, AP242 round-trips STILL pass. Wire
      into host CTest.

## 6. Gate 2 — sim vs OCCT (`tests/sim/native_step_import_parity.mm`)

- [x] 6.1 Extend the harness + `scripts/run-sim-native-step-import.sh`; `xcrun simctl list devices booted`
      first; own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown (suite
      assertion count unchanged).
- [x] 6.2 OCCT authors a general-revolution (ellipse profile) solid; native `cc_step_import` (engine 1)
      imports it; OCCT `STEPControl_Reader` re-imports. If T2 lands, assert same COUNT / volume / watertight
      / bbox within tolerance; if T2 is dropped, assert native DECLINES → OCCT and matches `cc_set_engine(0)`.

## 7. Gate 3 — tessellation ZERO-REGRESSION PROOF (ONLY IF the mesher was touched)

- [x] 7.1 IF (and only if) Task 3.2 added an additive mesher branch: run the FULL tessellation-sensitive sim
      set — `scripts/run-sim-suite.sh`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3 —
      and PROVE every existing sphere / cylinder / cone / plane / B-spline / torus face meshes byte-identical
      (same triangle counts, watertight status, volumes as main). If ANY existing mesh changes, revert the
      mesher branch (Task 3.2) and keep the OCCT decline (Task 2.6).
- [x] 7.2 IF the mesher was NOT touched (preferred), existing tessellation is byte-identical by construction;
      re-run the tessellation-sensitive set once for assurance and capture the pass counts.

## 8. No-regression + NUMSCI + complexity + docs + validation

- [x] 8.1 No regression: the landed import slices (sim `[NIMPORT]` 69/69 incl. trimmed-curve +
      revolution-quadrics + torus + honest declines), STEP export, healing, SSI S1–S5, native blends +
      #6/#7, marching, boolean, construct, tessellation, curved-fillet/chamfer/boolean, wrap-emboss, phase3 —
      all green (host CTest + `run-sim-suite.sh`).
- [x] 8.2 NUMSCI ON build proves no interaction: configure `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON` +
      `-DCYBERCAD_NUMSCI_DIR=/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/step-lane-b/build-numsci/host`
      (+ NUMPP/SCIPP dirs); build + ctest.
- [x] 8.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`revolvedProfileBSpline` + `revolutionCirclePoles` / `promoteProfileToNurbs`, `profileOnSurface`, the
      `surfaceOfRevolution` dispatch arm, and any additive mesher branch) all acceptable for the
      parser/systems band; none pushed higher.
- [x] 8.4 `openspec validate add-native-step-general-revolution --strict` green.
- [x] 8.5 Update `openspec/NATIVE-REWRITE.md` + `docs/STATUS-phase-4.md`: native STEP import now covers (if
      T2 lands) an ellipse / B-spline revolution → native rational `Kind::BSpline`; if the watertight gate
      resolves to DECLINE, the ellipse / B-spline revolution keeps the OCCT decline. #8 `drop-occt` stays
      blocked. Living-spec sync/archive when the gate is green.

## Outcome / implementation notes (honest deviations from the plan)

- **T2 (GENERAL / ELLIPSE + B-SPLINE revolution) — LANDED native watertight, mesher UNTOUCHED (§6.4 path:
  land with ZERO tessellator change).** OCCT was empirically confirmed (homebrew OCCT 7.9.3 host probe) to
  emit BOTH an ELLIPSE revolution AND a non-rational B_SPLINE_CURVE revolution as ONE
  `SURFACE_OF_REVOLUTION` `ADVANCED_FACE` bounded by a **VERTEX_LOOP** (childless) — the SAME bare-periodic
  structure as a full sphere, so NO pcurve reconstruction is needed (the design's pcurve work-item is moot).
  The reader revolves the profile meridian into the exact rational tensor B-spline and the existing
  VERTEX_LOOP→bare-face path meshes it watertight over its natural bounds.
  - Sim `[NIMPORT]` (booted sim, engine 1 native vs engine 0 OCCT exact BRepGProp), **77/77 PASS** (was
    69/69; +8 from the two new tests): `revolution→ellipsoid` parsed=1 watertight=1 solids=1 nativeVol=6.6721
    vs occtVol=6.70206 (rel 4.47e-3), area rel 2.06e-3, cΔ 2.16e-8, bbox 1.57e-3, faces 1=1, edges 0;
    `revolution→bspline` nativeVol=130.995 vs occtVol=131.342 (rel 2.64e-3), area rel 1.32e-3, faces 1=1,
    edges 0. All prior [NIMPORT] cases (incl. the landed 69/69) still green.
  - Host CTest `test_native_step_reader` (29/29 suite green): `revolution_ellipse_generatrix_vertex_loop_
    imports_watertight` (Kind::BSpline face, watertight, V→4/3·π·b²·a within 1%),
    `revolution_bspline_generatrix_vertex_loop_imports_watertight`, and the honest-out
    `revolution_off_axis_ellipse_declines` (plane ⟂ axis → NULL → OCCT). Pre-existing
    `surface_of_revolution_ellipse_generatrix_declines` still declines (its ellipse plane is ⟂ the axis — a
    genuinely non-spheroidal config; comment updated to state the real reason).
  - **Mesher NOT touched** (Task 3.2/7.1 = N/A): `face_mesher.h` / `surface_eval.h` / `trim.h` unchanged, so
    Gate 3 is byte-identical by construction (Task 7.2). `shape.h` / `math/**` unchanged (rational substrate
    already present). Diff is `step_reader.cpp` (+221, additive: `revolvedProfile` + `ellipseMeridian` /
    `bsplineMeridian` + `revolvedProfile`'s guard `revolutionReproducesProfile` + `isFullRevolutionBSpline`
    admitted into the sphere/torus bare-periodic arm) and the two test files.
  - Naming deviation from the plan: the helpers landed as `revolvedProfile` / `ellipseMeridian` /
    `bsplineMeridian` / `revolutionReproducesProfile` (vs the plan's `revolvedProfileBSpline` /
    `revolutionCirclePoles` / `promoteProfileToNurbs` / `profileOnSurface`) — same construction (Piegl &
    Tiller A7.1), same honest-out. The watertight self-verify is the ENGINE's `robustlyWatertightImport`
    (unchanged), not a new in-reader gate.
  - Cognitive complexity (parser/systems band ≤25–35): new fns all acceptable — `revolutionReproducesProfile`
    13, `bsplineMeridian` 12, `ellipseMeridian` 8, `revolvedProfile` 8, `isFullRevolutionBSpline` 5,
    `surfaceOfRevolution` 4; `advancedFace` 21 (+1 from the added OR clause, unchanged band).
  - NUMSCI ON build (`build-ns`, `-DCYBERCAD_HAS_NUMSCI=ON`): built + **36/36 ctest PASS** (no interaction).
- **Writer untouched.** `step_writer.cpp` not in the diff (OCCT-authored fixtures). `cc_*` ABI, `iges_*`,
  `step_export` unchanged. No tolerance weakened. `src/native/**` stays OCCT-free (only comment mentions of
  OCCT, no includes/symbols). No git commit (human integrates).
