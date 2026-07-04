# Tasks — add-native-ssi-curved-boolean-wider (SSI Stage S5-b + S5-c)

Verification levels: **host** = OCCT-free host CTest extending
`tests/native/test_native_ssi_curved_boolean.cpp` (S5-b: through-drill CUT =
`vol(fat) − vol(COMMON)`, FUSE = `vol(fat)+vol(thin)−vol(COMMON)`, each watertight; S5-c:
sphere∩sphere COMMON = the closed-form lens volume, watertight, seam nodes on both
surfaces ≤ tol; tangent-sphere fixture → NULL); **sim** = native-vs-OCCT
`BRepAlgoAPI_{Fuse,Cut,Common}` parity extending
`tests/sim/native_ssi_curved_boolean_parity.mm`. Invoked behind the existing `cc_boolean`
op codes — **no `cc_*` entry point is added or changed**. Consumes the shipped S3
`ssi::TraceSet`, so the new assembler paths are compiled under **`CYBERCAD_HAS_NUMSCI`**.
`src/native/**` stays OCCT-free. **No change to `src/native/tessellate`** (the S5-a
lesson) and **no change to the S5-a `buildCommon` path** — additive only.

## 1. Dispatch: route by topology, not blanket-decline  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 In `ssi_boolean_solid` (`ssi_boolean.cpp`), keep the gate + trace UNCHANGED
  (both operands recognised elementary curved solids; `nearTangentGaps == 0`; every
  consumed WLine `Closed`/`BoundaryExit`; ≥ 1 seam). Replace the S5-a `switch(op)` that
  blanket-declined Fuse/Cut with a dispatch on **seam count + operand kinds + op**:
  two-seam through-drill cyl∩cyl → `buildCommon`/`buildCut`/`buildFuse`; one closed-seam
  sphere∩sphere → `buildLensCommon` (Common only); anything else → NULL → OCCT. Short,
  linear table. (**host**)
- [x] 1.2 The dispatcher must leave the S5-a two-seam COMMON path (`buildCommon`)
  bit-identical in behaviour — the existing S5-a tests
  (`through_drill_common_watertight_matches_analytic`,
  `steinmetz_analytic_value_and_honest_native_fallback`,
  `near_tangent_and_disjoint_pairs_return_null`) stay green unchanged. (**host** ✓ S5-a
  regression)

## 2. S5-b — through-drill CUT (`A − B`)  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 `buildCut(fat, thin, seams)` — reuse the two rim seams + shared `VertexPool`:
  keep the fat wall OUTSIDE the thin tube (`classifyPoint` outside), re-trim the fat end
  caps to exclude the drilled region, and add the thin TUBE BAND (`appendTubeBand`)
  **reversed** as the inward tunnel wall; DROP the two mouth caps (removed material). Every
  seam-adjacent / re-trimmed face emits PLANAR-TRIANGLE facets through the shared pooled
  seam nodes (S5-a discipline). Non-recognised through-drill topology, ON-band sample, or
  a weld that cannot close → NULL. (**host**)
- [x] 2.2 Volume identity host check: `ssi_boolean_solid(fat, thin, Cut)` non-NULL,
  watertight (`watertightMeshVolume > 0`), enclosed volume = `vol(fat) − vol(COMMON)`
  within the 1% deflection band (`vol(COMMON)` = the S5-a-pinned through-drill 3.11685).
  (**host**)

## 3. S5-b — through-drill FUSE (`A ∪ B`)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 `buildFuse(fat, thin, seams)` — keep fat OUTSIDE thin + thin OUTSIDE fat (each
  outside-wall stretch with its own end-cap disc) + the operands' end caps; DROP the two
  mouth caps AND the inside tube band (now interior). Weld the two outer walls along the
  two shared rim seams; seam-adjacent faces are planar facets through the shared pool.
  Non-recognised topology / ON-band / open weld → NULL. (**host**)
- [x] 3.2 Volume identity host check: `ssi_boolean_solid(fat, thin, Fuse)` non-NULL,
  watertight, enclosed volume = `vol(fat) + vol(thin) − vol(COMMON)` within the band; the
  S5-a monotone invariants (`fuse ≥ max(A,B)`) still hold. Update / replace the S5-a
  `through_drill_fuse_cut_deferred_and_monotone_relations` test so it asserts the NATIVE
  fuse/cut results now (not the NULL deferral), keeping the closed-form invariants.
  (**host**)

## 4. S5-c — sphere∩sphere COMMON via single-seam / two-cap assembler  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 `buildLensCommon(A, B, seam)` — gate: exactly one `Closed` seam, both operands
  `CurvedKind::Sphere`, seam not a full sphere (coincident) → else NULL. (**host**)
- [x] 4.2 Cap poles: A's surface point nearest `cB` and B's nearest `cA`, evaluated on the
  analytic sphere from `recogniseCurvedSolid`. Survival (COMMON rule): keep the A-cap iff
  `classifyPoint(csB, poleA) == inside` and the B-cap iff `classifyPoint(csA, poleB) ==
  inside`; a pole in the ON-band (tangent) → abort → NULL. (**host**)
- [x] 4.3 Cap weld: emit each cap with the `appendMouthCap` radial-ring planar-facet
  discipline — fan from the pole out through rings (count from the seam angular span /
  `kCapSagitta` sagitta step) to the OUTER ring = the EXACT traced seam nodes from the
  SHARED `VertexPool`, every node on the analytic sphere, facet normals outward. Both
  caps' outer rings are the same pooled seam vertices → weld watertight along the ONE
  seam. `makeShell → makeSolid`. (**host** ✓ seam nodes on both surfaces ≤ tol)
- [x] 4.4 Volume host check: `ssi_boolean_solid(sA, sB, Common)` non-NULL, watertight,
  enclosed volume = the closed-form lens `V = π(rA+rB−d)²(d²+2d·rB−3rB²+2d·rA+6rA·rB−3rA²)
  /(12d)` (equal radii: `2 · π h²(3r−h)/3`, `h = r − d/2`) within the deflection band, on
  BOTH an equal-radius and an unequal-radius fixture. (**host**)

## 5. Engine self-verify — reuse the EXISTING generic guard (no engine change)
- [x] 5.1 Confirm (test-level) that the ENGINE's existing generic set-algebra guard
  (`native_engine.cpp booleanResultVerified`) ACCEPTS the S5-b fuse/cut candidates
  (`expected = va+vb−vc` / `va−vc` via the native COMMON) and the S5-c common candidate
  (`expected = vc`), and DISCARDS a mis-welded / wrong-volume candidate → OCCT. NO new
  oracle is added; the `ssiCurvedBooleanVerified` Steinmetz branch stays untouched and
  does not fire for these cases. (**host**)

## 6. Honest scope — deferrals (never faked)
- [x] 6.1 sphere∩sphere Fuse/Cut, tangent/coincident spheres (`nearTangentGaps > 0` /
  ON-band pole), oblique / multi-tube cyl∩cyl, and other curved-curved families
  (cyl∩cone, cyl∩sphere, cone∩cone, sphere∩box, freeform) → NULL (OCCT), documented in the
  `ssi_boolean.h` header scope note + the `native-booleans` / `native-ssi` namespace docs.
  (**host** ✓ NULL fixtures: tangent spheres, sphere fuse/cut)

## 7. Verification (two gates)
- [x] 7.1 Host suite extension of `test_native_ssi_curved_boolean.cpp`: S5-b Cut/Fuse
  volume-identity + watertight; S5-c equal & unequal sphere-lens COMMON volume +
  watertight + seam-node residual; tangent-sphere + sphere-fuse/cut NULL fixtures. Full
  CTest green NUMSCI ON and OFF (new assertions absent with NUMSCI off). No OCCT; no
  tolerance weakened. (**host**)
- [x] 7.2 Sim parity extension of `native_ssi_curved_boolean_parity.mm` +
  `run-sim-native-ssi-curved-boolean.sh`: through-drill Fuse/Cut vs
  `BRepAlgoAPI_{Fuse,Cut}` and sphere∩sphere Common vs `BRepAlgoAPI_Common`
  (`BRepPrimAPI_MakeSphere`); volume, surface area, watertight closed shell, `BRepCheck`
  validity per pair; report deltas + the count still deferred to OCCT; run via `xcrun
  simctl spawn <booted udid>`. (**sim**)
- [x] 7.3 `openspec validate add-native-ssi-curved-boolean-wider --strict` green; update
  `SSI-ROADMAP.md` S5 (S5-b fuse/cut + S5-c sphere∩sphere done at the bar with measured
  deltas), `ROADMAP.md` / `NATIVE-REWRITE.md` / `README.md` where they cite S5; the S4
  moat and the remaining curved-curved families stay the tail.

## Deferred to S4 / later S5 (NOT in this change — honest)

- [ ] **sphere∩sphere Fuse / Cut** — the outer-cap union + re-trimmed sphere remainder
  weld; deferred (S5-c ships COMMON only) → OCCT.
- [ ] **Near-tangent / coincident** curved pairs (`nearTangentGaps > 0`, tangent spheres,
  equal-radius orthogonal cyl∩cyl Steinmetz) → **S4 + OCCT fallback**; unchanged.
- [ ] **Oblique / multi-tube cyl∩cyl** piercings (seams not two clean full-circle rims) →
  OCCT (the `buildCommon`/`buildCut`/`buildFuse` gate already rejects).
- [ ] **Other curved-curved families** (cyl∩cone, cyl∩sphere, cone∩cone, sphere∩box,
  freeform) → OCCT until a later S5 stage.
