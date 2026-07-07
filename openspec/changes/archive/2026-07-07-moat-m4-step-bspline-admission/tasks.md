# Tasks — moat-m4-step-bspline-admission (MOAT M4, first slice)

Order: substrate build → baseline capture → reader pcurve arm → reader guard → host
analytic gate → sim native-vs-OCCT gate → zero-regression proof → docs, or HONEST
DECLINE. All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`),
namespace `cybercad::native::exchange`. **The tessellator is NOT modified** — this slice
CONSUMES the landed M0 mesher. No `cc_*` ABI change; no tolerance weakened; a correct
decline (patch stays OCCT) is a first-class outcome.

## 0. Substrate + baseline (capture BEFORE touching the reader)

- [x] 0.1 Build the numeric substrate for both targets:
      `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`; export
      `CYBERCAD_NUMSCI_DIR=<worktree>/build-numsci/{iossim|host}` per target.
- [x] 0.2 Recorded the GREEN baseline: full host CTest 36/36 (incl. NUMSCI-ON SSI suites)
      and the `native_step_import_parity` sim harness (prior 77 native/OCCT parity cases all
      still PASS after the change) — the reference for §6.
- [x] 0.3 Authored the foreign trimmed-B-spline fixture (a rim-CIRCLE `EDGE_LOOP` over a
      `B_SPLINE_SURFACE_WITH_KNOTS` bump-cap dome — a genuinely curved `(u,v)` boundary) in
      BOTH gates: host `test_native_step_reader.cpp` (`bumpCappedCylinder`, closed-form
      volume oracle) and sim `native_step_import_parity.mm` (`bumpcap::solid`, OCCT
      `STEPControl_Reader` volume oracle), each with a sibling deliberately-unfaithful variant
      (centre pole lifted off the dome → must decline).

## 1. `pcurveFor` B-spline-surface arm (`step_reader.cpp`)

- [x] 1.1 Add a `Kind::BSpline` surface arm in `pcurveFor` BEFORE the generic linear
      fall-through. Sample the 3D edge at `first`/`last`, invert via `projectBSplineUV`,
      and classify straight-vs-curved by the midpoint deviation from the straight UV
      chord (UV-domain-relative eps).
- [x] 1.2 Straight-in-`(u,v)` edge → a UV `Line` through the two projected endpoints —
      byte-identical to the current generic-linear arm (regression-preserving for
      existing straight B-spline walls).
- [x] 1.3 Curved edge → helper `bsplinePCurveUV(srf, edge)`: densify N projected samples
      (`projectBSplineUV`), emit a UV `B_SPLINE` pcurve (`poles2d` = projected samples;
      `degree`/`knots` preserved from the 3D edge curve) — evaluated as-is by the landed
      `trim.h::pcurveValue` case `K::BSpline`. No tessellator change.
- [x] 1.4 Keep each helper ≤ ~10 cognitive; the arm delegates fit + classification.

## 2. Faithful-reconstruction guard (`step_reader.cpp`)

- [x] 2.1 Add a rational-aware `Kind::BSpline` case to `surfaceValue` via
      `math::nurbsSurfacePoint(degreeU, degreeV, grid, weights, knotsU, knotsV, u, v)`
      (mirroring the existing `revolutionValue`), so the guard can evaluate the patch.
- [x] 2.2 Add `pcurveFaithful(srf, edge, pc)`: at ~5-9 parameters across `[first,last]`
      compute `uv = pcurveValue(pc, t, frac)`, `Sp = surfaceValue(srf, uv)`,
      `Ce = evalEdge(edge.curve, t)`; require `distance(Sp, Ce) ≤ 1e-6·max(1, scale)`
      with `scale` = control-net extent (never weakened). Uses the SAME `pcurveValue`
      the mesher flattens (no evaluator drift).
- [x] 2.3 In `buildFaceWithPCurves`, invoke the guard per rebuilt edge (outer + holes);
      any unfaithful edge ⇒ `decline()` (sets `fail_`), so `advancedFace` /
      `closedShell` abort → OCCT. Preserve every existing `decline()` precedent.

## 3. Admission (no new branch — verify routing)

- [x] 3.1 Confirm a trimmed `Kind::BSpline` face (real `EDGE_LOOP`, not childless, not
      `isFullRevolutionBSpline`) reaches `buildFaceWithPCurves` and, when every edge is
      faithful, builds a native trimmed `Kind::BSpline` face that flows into the M0
      `trimmedFreeformMesh` path. The bare-periodic full-sphere/torus/full-revolution
      arms stay UNTOUCHED.

## 4. Host analytic gate (`tests/native/…`, no OCCT linked)

- [x] 4.1 Build a native trimmed `Kind::BSpline` face with a closed-form curved
      boundary; assert the guard ACCEPTS the reconstructed pcurve
      (`S(pcurve(t)) = C_edge(t)` within tol at many `t`).
- [x] 4.2 Assert the guard REJECTS a deliberately perturbed off-surface edge
      (`decline()` fires) — the never-fabricate contract.
- [x] 4.3 Mesh the admitted solid via the landed M0 mesher; assert watertight + enclosed
      volume matches the independent closed-form value within tol
      (`foreign_trimmed_bspline_curved_boundary_admitted_watertight`: V=πR²h+πR²H/2, exact
      vs source, ≤3% vs closed form at defl 0.005, O(defl)-convergent). Rational variant:
      the fixture carries unit `weights` and rounds-trips through the (weight-dropping)
      writer as the geometrically-identical non-rational patch — the reader's rational-aware
      `surfaceValue`/guard path is present; a genuinely-rational (non-unit `weights`) fixture
      needs the parser extension noted in the design and is deferred to a follow-up. Hole
      (inner-loop) variant deferred with it (same admission arm, guarded identically).

## 5. Sim native-vs-OCCT gate (booted simulator, OCCT linked)

- [x] 5.1 Import the foreign trimmed-B-spline fixture with `cc_set_engine(1)`; the native
      solid is watertight, 1 solid, and its (fine-deflection) volume matches OCCT
      `STEPControl_Reader` (exact B-rep, 0.501194 vs 0.502644, rel 2.9e-3) + the closed form
      — `native_step_import_parity` `runBumpCapBSplineAdmission` (bumpcap admit PASS; whole
      harness 79/0).
- [x] 5.2 Import the one-unfaithful-edge fixture (centre pole lifted off the dome); the
      reader `decline()`s (native parsed=0) → OCCT re-imports the file unchanged
      (occtVol > 0). No tolerance weakened, no approximate face emitted (bumpcap decline PASS).
- [x] 5.3 The reader's per-edge faithful-reconstruction guard is the first honest-out (an
      off-surface trim declines BEFORE meshing); the engine's watertight self-verify remains
      the second (a non-watertight admitted mesh → OCCT). The unfaithful fixture declines at
      the guard, so no wrong/leaky solid is ever emitted.

## 6. Zero-regression proof (MANDATORY)

- [x] 6.1 Assert the straight-UV arm is byte-identical to the prior generic-linear pcurve
      for a straight-in-`(u,v)` B-spline-wall edge (a host unit assertion on the emitted
      pcurve).
- [x] 6.2 Re-run the full baseline from §0.2; every count MUST be unchanged
      (`run-sim-suite` 221/221, STEP import 77/77, host CTest / NUMSCI-ON). The
      tessellator is untouched, so tessellation-sensitive suites are unaffected by
      construction.

## 7. Docs / spec

- [x] 7.1 Update `openspec/MOAT-ROADMAP.md` M0/M4 status: the deferred STEP-reader
      admission LANDED (or the honest decline with the measured gap + specific blocker).
- [x] 7.2 `openspec validate moat-m4-step-bspline-admission --strict`; archive on
      completion.

## 8. Honest-out (a first-class outcome, not a fallback failure)

- [x] 8.1 NOT triggered — the curved-edge arm IS robustly reachable: the faithful circle
      trim reconstructs to ~1e-7 (well inside the 1e-6 weld radius), passes the guard, and
      meshes watertight vs the OCCT oracle. The honest-out machinery is nonetheless fully in
      place and exercised (the unfaithful fixture measures a 1.1e-3 gap → declines → OCCT),
      so the first-class-decline contract is proven, not merely asserted. No fabrication, no
      dead code, no weakened tolerance.
