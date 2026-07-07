# Tasks — moat-m1-ssi-s4-general (SSI Stage S4-d, second slice: GENERAL / FREEFORM branch point)

Verification levels: **host** = OCCT-free host CTest (`tests/native/test_native_ssi_marching.cpp`) — a
freeform-saddle ∩ plane OPEN X-crossing TRACED, a freeform definite contact ENDS with no arms, and the
analytic Steinmetz / transversal / S4-c / S4-e cases UNCHANGED, all under `CYBERCAD_HAS_NUMSCI`;
**sim** = native-vs-OCCT parity (`tests/sim/native_ssi_marching_parity.mm` +
`scripts/run-sim-native-ssi-marching.sh`) vs `GeomAPI_IntSS` / `IntPatch` (branch/arm count + on-locus +
on-both-surfaces + arc length). SSI is INTERNAL — **no `cc_*` entry point is added or changed**. The
S4-d-g parts are compiled under `CYBERCAD_HAS_NUMSCI` (like S2/S3/S4-c/S4-d/S4-e). `src/native/**` stays
OCCT-free. **No change to `src/native/tessellate`**; **no weakened tolerance**; **no fabricated arm or
point past a branch**; **no unreachable dead code**.

## 0. Diagnose (confirm reachability + the one gap)
- [x] 0.1 Confirm the S4-d machinery is SURFACE-AGNOSTIC: `branchpt::localize` / `enumerateArms` /
  `sharedTangentFrame` / `relativeSecondForm` / `solveTangentCone` and the `marching.cpp`
  `localizeBranchPoints` / `routeArm` / `routeBranches` / `reclassifyBranchArcs` touch a surface only
  through `SurfaceAdapter`; a `makeBSplineAdapter` pair flows through the same `march_branch_impl` /
  `routeBranches` path. (**host** — recorded in design.md §"What is TRUE")
- [x] 0.2 Confirm the branch SIGNATURE + LOCALIZE + tangent cone fire correctly on freeform: a temporary
  marcher probe on a bicubic B-spline saddle ∩ plane-through-the-saddle-point localizes
  `branchPoints == 1` on both surfaces `≤ onSurfTol` and enumerates the correct FOUR arm rays with the
  EXISTING code. (**host** — probe reverted, tree clean)
- [x] 0.3 Confirm `branchpt::reproject` (arm re-projection) is NORMAL-FREE and `routeArm` VERIFIES each
  arm's first step on both surfaces `≤ onSurfTol` (nullopt otherwise), and `localizeBranchPoints` drops
  any stall that does not localize / yields no arms. (**host** — the verify-or-drop honesty gate is
  already present)
- [x] 0.4 REFUTE the original hypothesis: `relativeSecondForm`'s CENTRAL difference cancels odd-order
  terms, so the third-derivative bias is absent — probed κ at B is O(δ²)-accurate (~1e-7, error ratio
  ≈ 4 per halving), six orders below the discriminant margin. A Richardson bias-cancellation is a no-op
  ⇒ NOT shipped. Identify the REAL gap: `reclassifyBranchArcs` recognizes only the CLOSED
  (branch-to-branch) topology, so a single freeform branch with OPEN arms (branch-to-boundary) leaves
  all arms in `nearTangentGaps`. (**host** — design.md §"What is TRUE" items 4–5)

## 1. OPEN-ARM branch-arc reclassification (S4-d-g-2)  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 Add two `WLine` bool fields `frontNearTangent` / `backNearTangent` (default `false`) in
  `src/native/ssi/marching.h`, documenting that they record whether the front/back end of `points`
  stalled at a near-tangency and are consumed only by `reclassifyBranchArcs`. Additive ⇒ every
  non-branch trace byte-identical. (**host**)
- [x] 1.2 Set the flags in `march_branch_impl` on the `NearTangent` status branch:
  `frontNearTangent = (b.end == DirEnd::NearTangent)`, `backNearTangent = (f.end == DirEnd::NearTangent)`
  (points = `[reversed backward half] seed [forward half]`). Both stay `false` for `Closed` /
  `BoundaryExit`. (**host**)
- [x] 1.3 Generalise `reclassifyBranchArcs` to the OPEN-ARM topology: reclassify a `NearTangent` arc to
  `BranchArc` when every near-tangent END is on a localized branch node and at least one end is; a
  near-tangent end NOT on a branch keeps the arc a gap. Confirm this reduces to the existing
  both-ends-on-branch rule for Steinmetz (branch-to-branch, bit-identical). (**host** regression)

## 2. Routing + assembly + tangent cone (S4-d-g-1, S4-d-g-3) — reuse UNCHANGED
- [x] 2.1 Confirm `localizeBranchPoints` / `routeArm` / `connectArm` / `routeBranches`, the tangent cone
  (`branch_point.h` `relativeSecondForm` / `solveTangentCone` / `enumerateArms` / `sharedTangentFrame`),
  and the `BranchNode` / `BranchArc` / `branchPoints` result reuse are UNCHANGED — the freeform branch
  trace differs from the analytic one ONLY in the OPEN-ARM reclassification. No new corrector, no new
  witness, no new result counter, no change to `branch_point.h`, no Richardson. (**host**)
- [x] 2.2 Confirm the freeform-saddle ∩ plane-through-the-saddle-point X-crossing LOCALIZES B on both
  surfaces `≤ onSurfTol`, the tangent cone yields two distinct real lines, `routeArm` verifies + marches
  each arm, and `reclassifyBranchArcs` takes the assembled arcs out of `nearTangentGaps` —
  `branchPoints == 1`, `nearTangentGaps == 0`, `tracedBranches == 4`, `openCurves == 4`, every node on
  both surfaces `≤ onSurfTol`. (**host** — `march_freeform_saddle_branch_open_arms_s4d`)

## 3. Honest guard + non-transversal DECLINE (S4-d-g-4)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Confirm a freeform DEFINITE contact (a B-spline BUMP `z = 0.15·(x²+y²)` tangent to a plane,
  `H` definite, Δ ≤ 0) ENDS with NO arms (`solveTangentCone` empty ⇒ `localizeBranchPoints` drops it ⇒
  `branchPoints == 0`, `routedArms == 0`, `curveCount() == 0`, `deferredTangent ≥ 1`), never sprouting
  an arm. (**host** — `march_freeform_bump_definite_never_branches_s4d`)
- [x] 3.2 Confirm a near-tangent arc whose stall end is NOT on a localized branch node stays a
  `NearTangent` gap (the per-end reclassification guard) — the OFF case of the LAND fixture defers
  (`branchPoints == 0`, `nearTangentGaps == 1`), and the z=0 (not-through-saddle) plane still traces two
  DISJOINT open curves with `branchPoints == 0`. (**host**)

## 4. Honesty invariants (no fabrication, no weakened tolerance, no dead code)
- [x] 4.1 Confirm `src/native/**` never links OCCT; a freeform branch not robustly resolved leaves the
  backbone's truncated `NearTangent` WLine + `nearTangentGaps` (deferred → OCCT), never a fabricated
  arm; a definite contact ends with no arms. No unreachable code — the open-arm reclassification is
  exercised by the freeform-saddle fixture; the refuted Richardson piece is NOT shipped. (**host**)
- [x] 4.2 Confirm `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection`, and `branch_point.h` are
  UNCHANGED; the freeform-branch path introduces no weakened tolerance (each arm is verified `≤ onSurfTol`
  like every other node; the reclassification uses only the existing `mergeRadius`). Confirm
  `enableBranchPoints` OFF is byte-identical to the current S3/S4-c/S4-e/S4-f behaviour (the new WLine
  flags are unused there). (**host** regression — full CTest green NUMSCI ON, 36/36)

## 5. Verification (two gates)
- [x] 5.1 Host suite (NUMSCI): `tests/native/test_native_ssi_marching.cpp` grows the freeform-saddle ∩
  plane LAND case (`march_freeform_saddle_branch_open_arms_s4d`) + the bump-tangent DEFINITE control
  (`march_freeform_bump_definite_never_branches_s4d`); assert `branchPoints == 1` / `== 0` respectively,
  `nearTangentGaps` as specified, every node on both surfaces `≤ onSurfTol`. Regression: analytic
  Steinmetz still `branchPoints == 2` / `nearTangentGaps == 0`, transversal pairs, the S4-c graze, the
  S4-e sphere-pole / cone-apex / freeform-pole crossings, the S4-f completeness — all unchanged. Whole
  CTest green NUMSCI ON (36/36). (**host**)
- [x] 5.2 Sim parity (`scripts/run-sim-native-ssi-marching.sh` + `tests/sim/native_ssi_marching_parity.mm`):
  the `pairFreeformSaddleBranchS4d()` fixture builds the native freeform-saddle ∩ plane vs the OCCT ORACLE
  `Geom_BSplineSurface` (the saddle control net) ∩ `Geom_Plane`; the native freeform branch trace
  reproduces the OCCT `GeomAPI_IntSS` self-crossing — `branchPoints == 1`, `nearTangentGaps == 0`,
  `tracedBranches == 4`, every sampled native node on the OCCT locus (`onCurve = 8.9e-8`) AND on both OCCT
  surfaces (`onSurf = 5.1e-10`), the branch point at the saddle on the OCCT locus, OCCT reporting 4 locus
  branches. Analytic Steinmetz (`branchPoints == 2`) / transversal / S4-c / S4-e cases UNCHANGED. Ran via
  `xcrun simctl spawn` on a booted simulator: **12 passed, 0 failed**. (**sim** — PASS)
- [x] 5.3 `openspec validate moat-m1-ssi-s4-general --strict` green; updated `openspec/MOAT-ROADMAP.md`
  M1 (the FIRST freeform branch point landed — a freeform-saddle ∩ plane OPEN X-crossing now traced;
  non-transversal freeform contacts END with no arms; cusps / higher-multiplicity junctions defer;
  general near-tangent S4-c and coincident-freeform S4-a stay the M1 tail), plus
  `openspec/SSI-ROADMAP.md` (S4-d second slice), `openspec/NATIVE-REWRITE.md`, and `docs/STATUS.md`.

## Deferred to M1-tail (NOT in this change — honest)

- [ ] **Non-transversal (definite) freeform contacts** — the curve ends with NO arms (isolated tangent
  point), never a fabricated arm. (pinned by the bump control)
- [ ] **Freeform cusps (double root) and higher-multiplicity junctions (three+ tangent lines)** →
  DEFER → OCCT.
- [ ] **Both-operand-freeform saddle ∩ saddle** whose branch does not verify → DEFER → OCCT.
- [ ] **General near-tangent breadth (S4-c beyond the analytic graze)** and **coincident/overlapping
  freeform surfaces (S4-a)** — the other two M1 regimes, separate slices.
- [ ] **A full brep freeform-branch B-spline SOLID through the boolean pipeline** — the slice is at the
  marcher level (`trace_intersection` / `trace_from_seeds`).
- [ ] **Any branch not robustly localizable / enumerable / verifiable** → truncate + defer → OCCT,
  reported with the measured gap, never faked.
