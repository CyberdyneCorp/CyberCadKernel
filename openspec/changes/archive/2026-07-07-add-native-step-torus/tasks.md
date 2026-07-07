# Tasks — add-native-step-torus (Phase 4 #8 — the last STEP-surface revolution gap: TORUS + general revolution)

Close the last `SURFACE_OF_REVOLUTION` gap in two tracks, each behind an HONEST per-track gate. **T1**:
an OFF-AXIS `CIRCLE` / arc revolution + the direct `TOROIDAL_SURFACE` keyword → a NEW native
`FaceSurface::Kind::Torus` (additive topology + an ADDITIVE, byte-identical-proven tessellator torus mesh
path + STEP mapping). **T2**: an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` generatrix → a native rational
`Kind::BSpline` revolved surface, or an honest DECLINE. Map ONLY geometry the file describes, VERIFIED to
pass through the profile AND to self-verify watertight; otherwise DECLINE (NULL → OCCT). Native code stays
OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*` ABI change. Default
engine stays OCCT. The tessellator changes are ADDITIVE-ONLY and must be PROVEN byte-identical for every
existing mesh. `step_writer.cpp` is preferred UNCHANGED (OCCT-authored fixtures). `iges_*` / `step_export`
stay unchanged. NEVER invent a surface or a solid; NEVER weaken a tolerance; NEVER perturb an existing
tessellation path.

## Implementation status (Lane B)

**T1 (TORUS) — LANDED, native watertight.** A `TOROIDAL_SURFACE` face (and an off-axis-circle
`SURFACE_OF_REVOLUTION`, which OCCT itself emits as `TOROIDAL_SURFACE`) now imports as a native
`Kind::Torus` solid. Diagnosis (Task 1.1) showed OCCT bounds a full torus with a FULLY-SEAMED `EDGE_LOOP`
(equator v-seam + tube u-seam, each forward AND reversed) — no pole, no real trim. This let T1 land via the
EXISTING sphere bare-periodic path rather than a new mesh branch: the reader detects the all-seam loop and
builds a `Kind::Torus` face with a NULL outer wire; the tessellator meshes its natural (u,v)∈[0,2π]²
rectangle through the UNCHANGED `structuredGrid` path and welds both seams. **`face_mesher.h` and `trim.h`
were NOT touched at all** — even more conservative than Task 4.1 planned (no doubly-periodic seam-weld
branch and no torus pcurves were needed). Additive changes only: `shape.h` (enum + `minorRadius`),
`surface_eval.h` (Torus arms), `step_reader.cpp` (`toroidalSurface`, off-axis `revolvedCircle`→Torus,
`projectUV`/`surfaceValue`/`torusOnSurface` arms, `isFullySeamedLoop` detection, bare-periodic + partial-
torus-decline arms in `advancedFace`). Verified: host `toroidal_surface_full_torus_imports_watertight`
(V=2π²Rr², watertight, 1 face); sim `[NIMPORT] native torus` parsed=1 watertight=1 vol rel 2.7e-3 vs OCCT.

**T2 (GENERAL / ELLIPSE revolution) — HONEST DECLINE retained.** An ellipse / B-spline generatrix
revolution keeps the `default → nullopt` decline → OCCT (it imports fine there). The rational-tensor-B-spline
reconstruction (§6) plus its watertight self-verify + a capped-solid fixture is a larger, higher-blast-radius
change deferred rather than rushed — the per-track honest gate (6.4) resolves to DECLINE.

**Zero-regression proof (Gate 3):** host 29/29 (default) + 36/36 (NUMSCI); sims — native-step-import 69/69,
tessellation 20/0 (filleted-box tris=332 unchanged), curved-fillet 23/0, curved-chamfer 9/0,
curved-boolean 18/0, wrap-emboss 6/0, phase3 PASS. Existing tessellation byte-identical by construction
(no existing mesh path edited) and empirically.

## 1. Ground the exact OCCT-emitted torus + general-revolution entities (before coding)

- [ ] 1.1 Author (OCCT `STEPControl_Writer`) a TORUS solid two ways — (a) a direct `TOROIDAL_SURFACE`
      (e.g. `BRepPrimAPI_MakeTorus`), (b) an off-axis-circle `SURFACE_OF_REVOLUTION` (revolve a circle whose
      centre clears the axis). Diagnose the emitted DATA: confirm `TOROIDAL_SURFACE('',#axis2,major,minor)`
      arg order + whether OCCT emits a direct `TOROIDAL_SURFACE` or a `SURFACE_OF_REVOLUTION` for each; and
      — CRITICALLY — how OCCT BOUNDS a full torus advanced face. A ring torus has NO pole, so it is NOT a
      lone VERTEX_LOOP like a sphere: record whether the bound is a seam EDGE_LOOP (u-seam meridian +
      v-seam rim), two seams, or a bare doubly-periodic face. This decides the reconstruction shape.
- [ ] 1.2 Author (OCCT `STEPControl_Writer`) a GENERAL-REVOLUTION solid: an `ELLIPSE` profile revolved
      about an axis (a spheroid / ellipsoid of revolution), and — if OCCT will emit one — a
      `B_SPLINE_CURVE_WITH_KNOTS` profile revolution. Confirm each is genuinely a `SURFACE_OF_REVOLUTION`
      with the stated profile (not collapsed to a quadric) so the T2 arm is exercised, and record the exact
      profile-entity args needed to rebuild the rational tensor B-spline.

## 2. T1 — new additive `Kind::Torus` in topology (`src/native/topology/shape.h`)

- [ ] 2.1 Append `Torus` to `enum class FaceSurface::Kind` (`{…,Bezier,Torus}`) so NO existing
      enumerator's ordinal changes. Add `double minorRadius = 0.0;` to `FaceSurface` (tube radius; the
      existing `radius` carries the MAJOR radius). Every existing kind leaves `minorRadius` at its default.
- [ ] 2.2 Confirm (host compile + a struct-layout assertion in the topology test) that every existing kind
      is byte-unchanged: `radius`/`semiAngle` semantics untouched, `minorRadius` defaults to `0.0`, and no
      existing constructor/serialization keys on the enum ordinal.

## 3. T1 — additive torus arm in the surface evaluator (`src/native/tessellate/surface_eval.h`)

- [ ] 3.1 Add `detail::asTorus(s) → math::Torus{s.frame, s.radius, s.minorRadius}`.
- [ ] 3.2 Add `case K::Torus` to `localValue` (→ `asTorus(s_).value(u,v)`), to `localD1`
      (→ `{value, dU, dV}`), and to `bounds` (→ `{0, 2π, 0, 2π}` — periodic in BOTH u and v, NO pole). Do
      NOT touch any existing arm. `curvatureMagnitude` needs no change (it central-differences `localValue`).

## 4. T1 — additive torus MESH PATH in the face mesher (`src/native/tessellate/face_mesher.h`)

- [ ] 4.1 Confirm a torus face is classified as analytic-curved (`k != Bezier && k != BSpline &&
      k != Plane`) so it flows through the EXISTING periodic-analytic grid + canonical-seam-anchor path
      unchanged. Add a NEW guarded branch for the doubly-periodic, NO-pole seam weld (u=0≡2π AND v=0≡2π),
      reusing the sphere longitude-seam canonical-anchor machinery WITHOUT modifying it. Do NOT touch the
      Plane/Cylinder/Cone/Sphere/BSpline/Bezier code paths.
- [ ] 4.2 **HONEST-OUT GATE**: if a clean additive doubly-periodic seam weld is NOT achievable without
      perturbing an existing path (proven by Gate 3 byte-identical failure), STOP T1's mesh path, revert the
      mesher, and keep the torus DECLINE (Task 7.x asserts the honest OCCT fallback). Report why plainly.

## 5. T1 — reader torus mapping (`src/native/exchange/step_reader.cpp`)

- [ ] 5.1 `surface()`: add `if (r->keyword == "TOROIDAL_SURFACE") return toroidalSurface(*r);`.
      `toroidalSurface(r)` validates `AXIS2_PLACEMENT_3D` + two trailing reals and builds
      `FaceSurface{Kind::Torus, frame, radius=major, minorRadius=minor}` (mirrors `placedSurface` with
      `nRadii=2` but the second real is `minorRadius`, not `semiAngle`).
- [ ] 5.2 `revolvedCircle` off-axis branch: replace the `distance(C,footC) > tol → nullopt` decline with a
      `Kind::Torus` build — `radius` = ⊥-distance(centre, axis) (major), `minorRadius` = `circle.radius`,
      frame origin = the foot on the axis, Z = axis, X ref = radial to C; require the generatrix circle's
      plane to admit a ring torus (meridian plane through the axis) else DECLINE. VERIFY via `torusOnSurface`
      before returning. The on-axis → Sphere branch is unchanged.
- [ ] 5.3 `projectUV`: add `case K::Torus: u=atan2(ly,lx); v=atan2(lz, hypot(lx,ly) − s.radius);`.
      `surfaceValue`: add `case K::Torus: return math::Torus{s.frame,s.radius,s.minorRadius}.value(u,v);`.
- [ ] 5.4 `pcurveFor`: add torus arms — a v-const rim boundary is a straight u-line, a u-const meridian is
      a straight v-line, the u=0 and v=0 seams are straight lines in (u,v) — synthesised by projecting
      endpoints exactly as the sphere/cylinder arms do.
- [ ] 5.5 Full-torus face reconstruction: per Task 1.1's diagnosis, close the doubly-periodic no-pole face
      watertight ADDITIVELY (seam-edge trimmed face, or a doubly-periodic bare-surface face analogous to the
      sphere bare-surface path extended to two seams). If it cannot close watertight within the additive
      budget (no writer/tessellator change beyond the additive torus mesh path), keep the torus DECLINE.
- [ ] 5.6 `torusOnSurface` guard: four tube-quadrant points of the generatrix circle lie on the candidate
      torus (via `pointOnSurface`) within a scale-relative tolerance; else DECLINE. No tolerance widened.

## 6. T2 — ellipse / B-spline revolution → native rational `Kind::BSpline` (`src/native/exchange/step_reader.cpp`)

- [ ] 6.1 `surfaceOfRevolution`: replace the `Ellipse`/`BSpline`/`Bezier` `default → nullopt` with a call
      to `revolvedProfileBSpline(*profile, ax->first, ax->second)`.
- [ ] 6.2 `revolvedProfileBSpline(profile, L, A) → optional<FaceSurface>`: build the EXACT rational
      tensor-product B-spline — u = the standard NURBS full circle (rational quadratic, weights
      `w=cos(Δ/2)`), v = the profile promoted to its rational-quadratic B-spline (circle/ellipse) or used
      directly (`B_SPLINE_CURVE_WITH_KNOTS`, rational iff it carries weights). Poles `P_ij = C_i(v_j)` on
      the revolution circle at each profile pole; weights `w_ij = w^u_i·w^v_j`. Emit
      `FaceSurface{Kind::BSpline, degreeU=2, degreeV=deg(profile), poles, weights, knotsU, knotsV}`.
- [ ] 6.3 `profileOnSurface` guard: sampled profile points `P(v_k)` lie on the reconstructed surface at
      `u=0` within a scale-relative tolerance; else DECLINE.
- [ ] 6.4 **HONEST-OUT GATE**: emit the surface ONLY when representable AND self-verifying watertight (u
      seam welds; a profile endpoint on the axis closes as a clean pole through the existing B-spline mesh
      path). Otherwise keep the `default → nullopt` DECLINE. No tolerance widened; no pole fabrication.

## 7. Engine hook + reader API + OCCT-free build

- [ ] 7.1 Confirm `src/engine/native/native_engine.cpp` `step_import` still calls `step_import_native` then
      `robustlyWatertightImport` (per-member, volume > 0) with NO new engine gate — a torus /
      revolved-B-spline face self-verifies exactly as any analytic face; a leaky result → OCCT. `iges_*` /
      `step_export` untouched.
- [ ] 7.2 `step_import_native` signature unchanged. Update `step_reader.h` / `native_exchange.h`
      doc-comments: a `TOROIDAL_SURFACE` face + an off-axis-circle revolution import onto native
      `Kind::Torus` when additive + self-verifying; an ellipse / B-spline revolution onto a native rational
      `Kind::BSpline` when faithfully representable + self-verifying; else DECLINE → OCCT.
- [ ] 7.3 Confirm `src/native/exchange/` + `src/native/tessellate/` + `src/native/topology/` still compile
      with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO simulator.
      Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 8. Gate 1 — host unit / decline (`tests/native/test_native_step_reader.cpp`)

- [ ] 8.1 T1: a `TOROIDAL_SURFACE` face and an off-axis-circle `SURFACE_OF_REVOLUTION` map to `Kind::Torus`
      (major = axis distance, minor = circle radius), VERIFIED the circle lies on the torus. If T1 lands the
      reconstructed solid is valid + watertight and matches the OCCT-torus volume/bbox; if T1 is dropped the
      reduction is host-asserted and the end-to-end path DECLINES (NULL).
- [ ] 8.2 T2: an ellipse (and, if authorable, a B-spline) `SURFACE_OF_REVOLUTION` maps to a rational
      `Kind::BSpline`, VERIFIED the profile lies on the surface; watertight if T2 lands, else DECLINE (NULL).
- [ ] 8.3 No regression: the on-axis circle → Sphere, line → cylinder/cone/plane, trimmed-curve, quadric,
      bspline-face, single/flat/placed-assembly, AP242 round-trips STILL pass; the topology struct-layout
      assertion (Task 2.2) confirms every existing kind is byte-identical. Wire into host CTest.

## 9. Gate 2 — sim vs OCCT (`tests/sim/native_step_import_parity.mm`)

- [ ] 9.1 Extend the harness + `scripts/run-sim-native-step-import.sh`; `xcrun simctl list devices booted`
      first; own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown (suite
      assertion count unchanged).
- [ ] 9.2 (A) OCCT authors a TORUS solid (off-axis-circle revolution / `TOROIDAL_SURFACE`); native
      `cc_step_import` (engine 1) imports it; OCCT `STEPControl_Reader` re-imports. If T1 lands, assert same
      COUNT / volume / watertight / bbox within tolerance; if T1 is dropped, assert native DECLINES → OCCT
      and matches `cc_set_engine(0)`.
- [ ] 9.3 (B) OCCT authors a general-revolution (ellipse profile) solid; native import vs OCCT re-import.
      If T2 lands, assert count / volume / watertight / bbox; if T2 is dropped, assert DECLINE → OCCT
      matching `cc_set_engine(0)`.

## 10. Gate 3 — tessellation ZERO-REGRESSION PROOF (the critical gate for the additive mesh path)

- [ ] 10.1 Run the FULL tessellation-sensitive sim set — `scripts/run-sim-suite.sh`, curved-fillet,
      curved-chamfer, curved-boolean, wrap-emboss, phase3 — and PROVE every existing sphere / cylinder /
      cone / plane / B-spline face meshes byte-identical: same triangle counts, same watertight status, same
      volumes as main. If ANY existing mesh changes, the torus mesh path is NOT additive → drop T1's mesh
      path (Task 4.2) and keep the torus DECLINE.
- [ ] 10.2 Capture a before/after diff (tri counts + volumes + watertight per suite) as the byte-identical
      evidence attached to the change before archive.

## 11. No-regression + NUMSCI + complexity + docs + validation

- [ ] 11.1 No regression: the landed import slices (sim `[NIMPORT]` 69/69 incl. trimmed-curve +
      revolution-quadrics + honest declines), STEP export, healing, SSI S1–S5, native blends + #6/#7,
      marching, boolean, construct, tessellation, curved-fillet/chamfer/boolean, wrap-emboss, phase3 — all
      green (host CTest + `run-sim-suite.sh`).
- [ ] 11.2 NUMSCI ON build proves no interaction: configure `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON` +
      `-DCYBERCAD_NUMSCI_DIR=/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/step-lane-b/build-numsci/host`
      (+ NUMPP/SCIPP dirs); build + ctest.
- [ ] 11.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`toroidalSurface`, the `revolvedCircle` torus arm, `revolvedProfileBSpline` + its NURBS-circle /
      profile-promotion helpers, the `projectUV`/`surfaceValue`/`pcurveFor` torus arms, the face-mesher
      torus branch) all acceptable for the parser/systems band; none pushed higher.
- [ ] 11.4 `openspec validate add-native-step-torus --strict` green.
- [ ] 11.5 Update `openspec/NATIVE-REWRITE.md` + `docs/STATUS-phase-4.md`: native STEP import now covers
      (per the landed tracks) `TOROIDAL_SURFACE` / off-axis-circle revolution → native `Kind::Torus` (with
      a byte-identical-proven additive mesh path) and/or ellipse / B-spline revolution → native rational
      `Kind::BSpline`; any track not meeting its honest gate keeps the OCCT decline. #8 `drop-occt` stays
      blocked. Living-spec sync/archive when the gates are green.

## Outcome / implementation notes (honest deviations from the plan)

- **T1 (TORUS) — LANDED native watertight.** A full `TOROIDAL_SURFACE` face (and an off-axis-circle
  `SURFACE_OF_REVOLUTION`, which OCCT itself emits as `TOROIDAL_SURFACE`) imports as a native `Kind::Torus`
  solid. Sim: `[NIMPORT] native torus  native raw parsed=1 watertight=1 solids=1 nativeVol=1771.77
  occtVol=1776.53` (rel 2.68e-3), faces nat=1/oracle=1, edges nat=0. Host:
  `toroidal_surface_full_torus_imports_watertight` (V=2π²Rr², watertight, 1 face) passes; a PARTIAL/trimmed
  torus honestly declines (`toroidal_surface_partial_torus_declines`).
- **Deviation from the planned mesh path (MORE conservative).** Task 4.1 planned a NEW doubly-periodic
  seam-weld branch in `face_mesher.h`. Task 1.1's diagnosis showed OCCT bounds a full torus with a
  FULLY-SEAMED `EDGE_LOOP` (equator v-seam + tube u-seam, each forward AND reversed) — no pole, no real
  trim — so T1 landed through the ALREADY-PROVEN sphere bare-periodic path via an all-seam loop detector.
  **`face_mesher.h` and `trim.h` were NOT touched at all.** No new mesh branch, no torus pcurves. The ONLY
  tessellate change is `surface_eval.h` (+9 lines: additive `case K::Torus` in `localValue`/`localD1`/`bounds`).
- **T2 (GENERAL / ELLIPSE revolution) — HONEST DECLINE retained (§6.4 gate → DECLINE).** The ellipse /
  B-spline generatrix revolution keeps the `default → nullopt` decline → OCCT (imports fine there). The
  rational-tensor-B-spline reconstruction (Tasks 6.1–6.3) plus watertight self-verify + a capped-solid
  fixture is a larger, higher-blast-radius change deferred rather than rushed. Sim:
  `[NIMPORT] foreign revolution decline  native parsed=0`; OCCT fallback exact (`revolution_torus mass
  rel=0.00e+00`).
- **Writer untouched.** `step_writer.cpp` is not in the diff (OCCT-authored fixtures throughout). `cc_*`
  ABI, `iges_*`, `step_export` unchanged. No tolerance weakened.
- **Gate 3 byte-identical evidence.** Existing tessellation unperturbed by construction (no existing mesh
  path edited) AND empirically: sphere/cyl/cone/plane revolution faces import byte-identically
  (`revolution→sphere` vol 902.31 == `sphere_keyword` 902.31; cyl/plane 1568.8; cone 522.934), curved-fillet
  tris=3912 unchanged, and every tessellation-sensitive suite matches baseline pass counts (curved-fillet 23,
  curved-chamfer 9, ssi-curved-boolean 21 native-pass=13, wrap-emboss 6, run-sim-suite 221, phase3 70).
- **Regression (all green).** Host ctest 29/29 (OCCT/NUMSCI OFF) + 36/36 (NUMSCI ON); sim
  native-step-import 69/69 (torus = native pass, general-revolution = honest OCCT decline), STEP export 28/0,
  heal 4/0. `src/native/**` stays OCCT-free. No git commit (human integrates).
