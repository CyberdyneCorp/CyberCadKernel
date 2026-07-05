# Tasks — add-native-ssi-s4e-singularities (SSI Stage S4-e, first slice)

Verification levels: **host** = OCCT-free host CTest
(`tests/native/test_native_ssi_marching.cpp` or a new
`tests/native/test_native_ssi_s4e_singularities.cpp`) — full trace of the sphere-pole great
circle (closed loop through both poles) + the cone-apex line (both nappes) + genuine-boundary
still-exits + transversal / S4-c graze / S4-d Steinmetz regression, all under
`CYBERCAD_HAS_NUMSCI`; **sim** = native-vs-OCCT parity
(`tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-marching.sh`, or a new
`scripts/run-sim-native-ssi-s4e.sh`) vs `IntPatch` / `GeomAPI_IntSS` (on-locus + on-both-
surfaces + pole/apex-point match + arc/closure). SSI is INTERNAL — **no `cc_*` entry point is
added or changed**. The S4-e parts are compiled under `CYBERCAD_HAS_NUMSCI` (like
S2/S3/S4-c/S4-d). `src/native/**` stays OCCT-free. **No change to `src/native/tessellate`**; **no
weakened tolerance**; **no fabricated point across a singularity**.

## 0. Diagnose (confirm the "before")
- [x] 0.1 Confirm on the CURRENT marcher that the SPHERE-POLE great circle TRUNCATES: unit
  sphere ∩ plane `y = 0`, forced through marching (`trace_from_seeds` with a hand seed), traces
  one `BoundaryExit` WLine of ~189 pts with **arcLen ≈ 3.1415 (= π, HALF the closed loop)**,
  `u1 ∈ [0, 0]` (only the `u = 0` meridian, never `u = π`), `v1 ∈ [−π/2, +π/2]`, worst on-surf
  residual `2.95e-12` — the full closed great circle is never assembled. (**host** — recorded in
  design.md)
- [x] 0.2 Confirm the CONE-APEX line STEP-COLLAPSES: double cone (`R₀ = 0`, α = 45°) ∩ plane
  `y = 0`, forced through marching, traces one `BoundaryExit` WLine hitting **20042 pts
  (maxPoints)** with `v1 ∈ [−0.036, 2.0]` (stalls just short of the apex `v = 0`, never the
  `v < 0` nappe), worst residual `1.62e-10` — the far nappe is never traced. (**host** —
  recorded in design.md)
- [x] 0.3 Confirm the CONTROLS: the 5 transversal pairs trace `nt == 0`; the S4-c crossable
  graze still crosses (`nearTangentCrossed ≥ 1`); the S4-d Steinmetz still traces
  (`branchPoints == 2`). Record the analytic ground truth (sphere-pole great circle = closed
  loop, circumference `2π`, through both poles `(0,0,±1)`; cone-apex line = both nappes crossing
  the apex `(0,0,0)`) so host + OCCT parity are well-defined. (**host**)

## 1. Result + options plumbing (additive)  [host]
- [x] 1.1 Add `int chartSingularCrossed = 0;` to `WLine` and `int singularitiesCrossed = 0;` to
  `TraceSet` in `marching.h`. `nearTangentGaps` now counts ONLY singularities that could NOT be
  crossed. Keep all existing fields. Document the chart-singularity contract in the header.
  (**host**)
- [x] 1.2 Extend `MarchOptions` (`marching.h`): add `bool enableChartSingularities = false;`
  (off → the current S3/S4-c behaviour, byte-identical), `double chartCollapseFrac = -1.0;`
  (≤0 → `1e-3`), `double chartStepFrac = -1.0;` (≤0 → `h0/16`). Sentinel-resolved in `tune()`.
  Keep all existing fields. (**host**)

## 2. Chart-singularity detection (S4-e-1)  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 Add a new OCCT-free header `src/native/ssi/chart_singularity.h`
  (`namespace chartsing`): `chartConditionAt(S, u, v, scale)` → `{‖dU‖, ‖dV‖, collapsed}` using
  the SAME central finite-difference scheme as `advanceParams` — `collapsed` iff
  `‖dU‖ < chartCollapseFrac·‖dV‖` AND `‖dU‖ < chartCollapseFrac·scale` AND the normal is finite;
  `poleContinuationU(uIn, uPeriod)` → `uIn + π (mod period)` (the arbitrary-longitude pin).
  Document the single-surface-Jacobian-rank-drop witness and why it is DISTINCT from the S4-c
  pair sine and the S4-d locus flip. (**host**)
- [x] 2.2 Add `chartCondition(A,B, State, scale)` in `marching.cpp` (evaluated alongside the
  transversality tangent in `marchDir`): returns which surface (if any) collapsed + its
  `ChartCond`. A CURVE CUSP (curve velocity → 0 with both charts regular) is flagged separately.
  Confirm the witness is INDEPENDENT of the S4-c/S4-d seams (computed from single-surface `dU`/
  `dV`, not `‖n₁×n₂‖`). (**host**)

## 3. Point-based crossing driver (S4-e-2, S4-e-3)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Add `crossChartSingularity(A,B, stall, tStarFwd, whichSurf, t, scale, out)` in
  `marching.cpp`, modelled on `crossNearTangent`: enter FINE along `t★` (`chartStepFrac·h0`),
  point-based fixed-plane correct across the singular band (reuse the S4-c along-`t★` cut /
  `branchpt::reproject` — NO degenerate `dU` in the solve), pin the far-side chart coords
  (pole: clamp `v = ±π/2`, set `u = poleContinuationU(u_in)`; apex: re-seed `(u,v)` from the
  continued 3D tangent, `v` sign flipped), verify every node on both surfaces ≤ `onSurfTol`,
  resume when `‖dU‖` recovers above the collapse threshold on BOTH surfaces. Bounded by a
  `chartMaxSteps` budget. (**host**)
- [x] 3.2 Add `tryChartBand(...)` in `marching.cpp` (parallels `tryBandEntry`): at a detected
  chart collapse (`enableChartSingularities` on), call `crossChartSingularity`; on success
  update the walk to the far side and `++chartSingularCrossed`; on failure return a
  `NearTangent`/`BoundaryExit` stop (the caller STOPS + defers). Wire it into `marchDir` BEFORE
  the spurious `BoundaryExit` / near-tangent branches so a chart collapse routes to the crossing
  instead of truncating, and so the `advanceParams` step-crawl (the 20042-pt apex pathology)
  is short-circuited. (**host**)

## 4. Honesty guard + boundary discrimination (S4-e-4)  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 Confirm a chart crossing is EMITTED only if every node verifies on both surfaces ≤
  `onSurfTol`, the far side makes real progress (`‖dU‖` recovered + advanced past a fine step),
  and (pole) the continuity-pinned `u_out` node verifies; otherwise DISCARD the arc + STOP +
  defer (truncated `NearTangent`/`BoundaryExit` + `nearTangentGaps` increment → OCCT), reporting
  the measured gap. NEVER fabricate a pole/apex-crossing point. (**host**)
- [x] 4.2 Confirm the boundary test is GUARDED by the chart witness: a genuine DOMAIN boundary
  `v`-edge (a finite cap, `‖dU‖` does NOT collapse) still exits `BoundaryExit`
  (`singularitiesCrossed == 0`); only a pole/apex `v`-edge (`‖dU‖` collapse + finite normal) is a
  crossing candidate. A genuine curve CUSP endpoint (the point-based step cannot continue) ENDS.
  (**host**)

## 5. Honesty invariants (no fabrication, no weakened tolerance)
- [x] 5.1 Confirm `src/native/**` never links OCCT; a singularity not robustly crossed returns a
  truncated WLine + typed stop reason + `nearTangentGaps` increment (deferred → OCCT), never a
  fabricated point. Document the detect → point-based-cross → resume contract + the
  crossed-vs-defer table in the `marching.h` / `chart_singularity.h` headers. (**host**)
- [x] 5.2 Confirm `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection` are UNCHANGED and the
  chart discriminators (`chartCollapseFrac`, `chartStepFrac`, the finite-normal test, the
  re-projection ≤ `onSurfTol`) introduce no weakened tolerance. Confirm `enableChartSingularities`
  OFF is byte-identical to the current S3/S4-c behaviour. (**host** regression)

## 6. Verification (two gates)
- [x] 6.1 Host suite (NUMSCI): the SPHERE-POLE great circle now FULLY traced — the full closed
  loop (arcLen ≈ `2π` within deflection tol, both `u = 0` and `u = π` meridians visited, both
  poles crossed), `singularitiesCrossed ≥ 2`, `status == Closed`, every node on BOTH surfaces ≤
  `onSurfTol`; the CONE-APEX line now traced ACROSS the apex (both nappes `v ∈ [−2, +2]` in a
  bounded node count), `singularitiesCrossed ≥ 1`, every node ≤ `onSurfTol`; a genuine-boundary
  `v`-edge STILL exits `BoundaryExit` (`singularitiesCrossed == 0`); the 5 transversal pairs
  trace bit-identically (`nt == 0`); the S4-c graze STILL crosses (`nearTangentCrossed ≥ 1`,
  `singularitiesCrossed == 0`); the S4-d Steinmetz STILL traces (`branchPoints == 2`,
  `singularitiesCrossed == 0`). Full CTest green NUMSCI ON and OFF (S4-e assertions absent with
  NUMSCI off). No OCCT; no tolerance weakened. (**host**)
- [x] 6.2 Sim parity (`scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`, or a new `scripts/run-sim-native-ssi-s4e.sh`): add
  the sphere-pole great circle and the cone-apex line; assert they are now FULLY traced natively
  (crossing the pole/apex) matching OCCT `IntPatch` / `GeomAPI_IntSS` — every sampled native node
  on the OCCT locus ≤ `onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces ≤
  `onSurfTol`; the native arc length / loop closure reconciles with the OCCT curve within tol;
  the native curve passes through the OCCT pole/apex point to `tol`. AND the genuine-boundary
  control STILL exits, and the transversal + S4-c + S4-d cases are UNCHANGED. Report per-pair
  crossed vs still-deferred; run via `xcrun simctl spawn <booted udid>`
  (`xcrun simctl list devices booted`). (**sim**)
- [x] 6.3 `openspec validate add-native-ssi-s4e-singularities --strict` green; update
  `SSI-ROADMAP.md` S4 (S4-e first slice landed — the sphere-pole / cone-apex crossing S3
  truncated now fully traced vs OCCT; general/freeform parametric singularities, edge/higher-
  order cusps, S4-f self-intersection completeness stay the tail), and
  `ROADMAP.md` / `NATIVE-REWRITE.md` / `README.md` where they cite S4.

## Deferred to S4-e-general / S4-f (NOT in this change — honest)

- [ ] **General / freeform parametric singularities** — NURBS degenerate edges, collapsed
  control rows, seam singularities on freeform surfaces. Only the elementary sphere pole + cone
  apex are in scope; anything else DEFERS → OCCT.
- [ ] **Higher-order cusps / edge singularities** — a curve cusp the point-based step cannot
  continue through (genuine endpoint) ENDS; a surface ridge / edge singularity DEFERS.
- [ ] **S4-f: self-intersection completeness / global topology repair** — small loops below the
  seeding floor, full self-intersection resolution.
- [ ] **Any singularity not robustly crossable / verifiable** → truncate + defer → OCCT (engine
  self-verify), reported with the measured gap, never faked.
