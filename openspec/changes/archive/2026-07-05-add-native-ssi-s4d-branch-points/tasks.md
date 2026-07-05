# Tasks — add-native-ssi-s4d-branch-points (SSI Stage S4-d, first slice)

Verification levels: **host** = OCCT-free host CTest
(`tests/native/test_native_ssi_marching.cpp` or a new
`tests/native/test_native_ssi_s4d_branch_points.cpp`) — full trace of the Steinmetz
bicylinder (both branch points localized + routed, all arms assembled) + isolated
tangent-point still-ends + S4-c graze still-crosses + transversal regression, all under
`CYBERCAD_HAS_NUMSCI`; **sim** = native-vs-OCCT parity
(`tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-marching.sh`, or a
new `scripts/run-sim-native-ssi-s4d.sh`) vs `IntPatch` / `GeomAPI_IntSS` (on-locus + on-both-
surfaces + branch-point match + arm/loop count). SSI is INTERNAL — **no `cc_*` entry point is
added or changed**. The S4-d parts are compiled under `CYBERCAD_HAS_NUMSCI` (like
S2/S3/S4-c). `src/native/**` stays OCCT-free. **No change to `src/native/tessellate`**; **no
weakened tolerance**; **no fabricated arm or point past a degeneracy**.

## 0. Diagnose (confirm the "before")
- [x] 0.1 Confirm on the CURRENT marcher that the STEINMETZ bicylinder DEFERS at the branch
  point: two equal R=1 cylinders, axes Z and X crossing orthogonally at the origin, trace one
  `NearTangent` WLine (~229 pts) with `stopReason == NearTangentTransversal` (sine ≈ 9e-4),
  `tracedBranches == 0`, `nearTangentGaps == 1`, `nearTangentCrossed == 0`, `branchPoints ==
  0` — the four elliptical arms are never assembled. (**host** — recorded in design.md)
- [x] 0.2 Confirm the CONTROLS: an isolated `TangentPoint` (two spheres at `d = R₁+R₂`) ends
  with no curve; the S4-c crossable graze (sphere / offset cylinder) still crosses
  (`nearTangentCrossed ≥ 1`); the 5 transversal pairs trace with `nt == 0`. (**host**)
- [x] 0.3 Record the analytic Steinmetz ground truth (two ellipses in planes `x = ±z`,
  semi-axes `1` and `√2`, crossing at the two saddles `(0, ±1, 0)`) so host + OCCT parity are
  well-defined. (**host**)

## 1. Result + options plumbing (additive)  [host]
- [x] 1.1 Add `struct BranchNode { math::Point3 point; double branchSine = 0.0;
  std::vector<int> armLineIds; }` to `marching.h`. Extend `TraceSet`: add
  `int branchPoints = 0;` and `std::vector<BranchNode> branchNodes{};`. `nearTangentGaps` now
  counts ONLY branch points that could NOT be resolved. Keep all existing fields. Document the
  branch-point / arm-connectivity contract in the header. (**host**)
- [x] 1.2 Extend `MarchOptions` (`marching.h`): add `double armStepFrac = -1.0;` (≤0 →
  `h0/8`), `double branchMergeFrac = -1.0;` (≤0 → `1e-4`), `bool enableBranchPoints = true;`
  (off → the S4-c defer, exactly as today). Sentinel-resolved in `tune()`. Keep all existing
  fields. (**host**)

## 2. Branch-point detection at the S4-c seam (S4-d-1)  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 Add `isBranchPoint(A,B, stall, tStar, lastGoodSine, t, scale)` in `marching.cpp`:
  at the S4-c collapse+flip detection (where `crossNearTangent` would return "not crossable"
  for a `NearTangentTransversal` stall), return true ONLY when: (a)
  `classify_tangent_contact_seeded` is NOT `TangentPoint` / `TangentCurve` / `Undecided`; (b)
  the localized B is INTERIOR on both param domains (not a non-periodic edge); (c)
  `enumerateArms` yields ≥ 2 distinct real directions. Every other outcome falls through to
  the EXISTING S4-c defer, byte-for-byte. (**host**)
- [x] 2.2 Confirm the S4-c `crossNearTangent` / `crossNodeCrossable` single-branch-graze path
  is UNCHANGED (the branch machinery only fires where S4-c would have deferred). (**host**
  regression)

## 3. Branch-point localization (S4-d-2)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Add `localizeBranchPoint(A,B, approach, tStar, t)` in `marching.cpp`: minimize the
  transversality sine `g(s) = ‖nA×nB‖` along the bracketed approach (`nn::minimize` over the
  along-`tStar` arc coordinate, each trial re-projected onto both surfaces with the S4-c
  fixed-plane corrector), then a full re-project of the minimum onto both surfaces
  (`nn::least_squares`, `A.point − B.point`). Return the `State` B with
  `‖A.point − B.point‖ ≤ onSurfTol` and `sine(B)` at/near the floor; `nullopt` if no clear
  minimum brackets or B does not re-project ≤ `onSurfTol` (⇒ defer, no fabricated B). (**host**)

## 4. Arm enumeration — the tangent cone, real roots only (S4-d-3)  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 Add `enumerateArms(A,B, B_state)` in `marching.cpp`: build the shared tangent-plane
  basis `{e₁,e₂}` at B (⊥ the common normal), form the relative second fundamental form
  `H = II_A − II_B` (2×2 symmetric, from finite-difference surface Hessians projected on the
  normal), solve the tangent-cone quadratic `c·m² + 2b·m + a = 0` (with the `c≈0` branch).
  Discriminant `Δ = b² − a·c > 0` ⇒ TWO distinct real lines ⇒ up to FOUR world-space rays
  (`±T₁, ±T₂`); `Δ ≤ 0` ⇒ return EMPTY (definite ⇒ isolated `TangentPoint`, END; double root
  ⇒ cusp, out of scope, DEFER). NEVER fabricate a ray. (**host**)
- [x] 4.2 Confirm the discriminant sign matches the S4-b `TangentPoint` classification on the
  two-sphere isolated tangent (definite ⇒ empty ⇒ no arms) — the "isolated tangent must still
  end" invariant enforced by the same sign test. (**host**)

## 5. Arm routing + dedup / connectivity / assemble (S4-d-4, S4-d-5)  [CYBERCAD_HAS_NUMSCI]
- [x] 5.1 Add `routeArm(A,B, B_state, rayDir, t, scale, out)` in `marching.cpp`: `P₀ = B +
  armStep·rayDir` (`armStep = h0/8`); S4-c fixed-plane correct `P₀` back onto both surfaces
  (`t★ = rayDir`, verify `≤ onSurfTol`) → `S₀`; then `marchDir` the normal S3 walk from `S₀`
  in the `rayDir` sense to termination. DROP the arm if `S₀` does not verify on both surfaces
  or the march makes no real progress (never fabricate the arm). (**host**)
- [x] 5.2 Extend `retraces` so an arm routed from a branch point that retraces a kept arm is
  deduped AND its shared branch-point connectivity is merged into the existing `BranchNode`
  (not dropped silently); an arm that closes a loop back through the same branch point is a
  closed multi-arm loop. (**host**)
- [x] 5.3 Wire the branch machinery into `marchDir` / `trace_from_seeds`: on a resolved branch
  point, `++branchPoints`, append a `BranchNode` (`point = B`, `branchSine ≈ 0`, `armLineIds`
  = the WLine ids meeting there), route + assemble the arms into `lines`; merge two branch
  points within `branchMergeFrac·scale`; bound recursion with a branch-point budget. On a
  NON-resolvable branch: STOP + defer EXACTLY as S4-c (a `NearTangent` WLine counted in
  `nearTangentGaps` with the typed `stopReason`). (**host**)

## 6. Honesty invariants (no fabrication, no weakened tolerance)
- [x] 6.1 Confirm `src/native/**` never links OCCT; a branch not robustly localized /
  enumerated / routed returns a truncated `NearTangent` WLine + typed `stopReason` +
  `nearTangentGaps` increment (deferred → OCCT), never a fabricated arm/point; an isolated
  `TangentPoint` ENDS with no arms. Document the localize → tangent-cone → route → assemble
  contract + the branch-vs-defer table in the `marching.h` header. (**host**)
- [x] 6.2 Confirm `tangentSinTol`, `minCrossSine`, `onSurfTol`, `minStep`, `maxDeflection` are
  UNCHANGED and the branch discriminators (interior test, real-tangent-cone test,
  re-projection ≤ `onSurfTol`) introduce no weakened tolerance. (**host**)

## 7. Verification (two gates)
- [x] 7.1 Host suite (NUMSCI): the Steinmetz bicylinder now FULLY traced — `branchPoints == 2`
  (both saddles `(0, ±1, 0)` localized, each on both cylinders ≤ `onSurfTol`, `branchSine`
  at/near the floor), the two ellipses assembled from the routed arms, `nearTangentGaps == 0`,
  every node on BOTH cylinders ≤ `onSurfTol`, the assembled arcs matching the analytic
  Steinmetz ellipses within the deflection tolerance; the isolated `TangentPoint` (two spheres
  `d = R₁+R₂`) STILL ENDS (`branchPoints == 0`, no arms fabricated); the S4-c graze STILL
  crosses (`nearTangentCrossed ≥ 1`, `nearTangentGaps == 0`, `branchPoints == 0`); the 5
  transversal pairs trace bit-identically (`nt == 0`). Full CTest green NUMSCI ON and OFF
  (S4-d assertions absent with NUMSCI off). No OCCT; no tolerance weakened. (**host**)
- [x] 7.2 Sim parity (`scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`, or a new `scripts/run-sim-native-ssi-s4d.sh`):
  add the Steinmetz fixture; assert it is now FULLY traced natively (all arms, both branch
  points localized + routed) matching OCCT `IntPatch` / `GeomAPI_IntSS` — every sampled native
  node on the OCCT locus ≤ `onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces
  ≤ `onSurfTol`; the native arm/loop count reconciles with the OCCT branch count (two crossing
  ellipses); the localized native branch points match the OCCT branch points to `tol`. AND the
  isolated tangent-point pair STILL ends (reported, no arms) and the S4-c graze STILL crosses.
  Report per-pair resolved vs still-deferred; run via `xcrun simctl spawn <booted udid>`
  (`xcrun simctl list devices booted`). (**sim**)
- [x] 7.3 `openspec validate add-native-ssi-s4d-branch-points --strict` green; update
  `SSI-ROADMAP.md` S4 (S4-d first slice landed — the Steinmetz self-crossing S3+S4-c deferred
  now fully traced vs OCCT; general/freeform branch points, S4-e singularities, S4-f
  self-intersection completeness stay the tail), and `ROADMAP.md` / `NATIVE-REWRITE.md` /
  `README.md` where they cite S4.

## Deferred to S4-d-general / S4-e / S4-f (NOT in this change — honest)

- [ ] **General / freeform branch points** — arbitrary self-crossings on freeform surfaces,
  or three-plus tangent lines at one point. Only the elementary two-real-distinct-line
  transversal self-crossing (Steinmetz family) is in scope; anything else DEFERS → OCCT.
- [ ] **Cusps / degenerate branches** — a DOUBLE root of the tangent-cone quadratic (one
  tangent line, higher-order contact). Detected and DEFERRED, never routed.
- [ ] **S4-e: singular points** — a surface's own degeneracy (cone apex, sphere pole) on the
  intersection locus.
- [ ] **S4-f: self-intersection completeness / global topology repair** — small loops below
  the seeding floor, full self-intersection resolution.
- [ ] **Any branch point not robustly localizable / enumerable / routable** → truncate + defer
  → OCCT (engine self-verify), reported with the measured gap, never faked.
