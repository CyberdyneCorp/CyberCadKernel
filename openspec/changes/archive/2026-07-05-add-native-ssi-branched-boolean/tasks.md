# Tasks — add-native-ssi-branched-boolean (SSI Stage S5-d)

Verification levels: **host** = OCCT-free host CTest extending
`tests/native/test_native_ssi_curved_boolean.cpp` (S5-d: equal-R orthogonal Steinmetz COMMON
= the EXACT `16 R³ / 3` analytic, watertight, arc nodes on both cylinders ≤ tol, the two
branch-point vertices a single shared node; FUSE/CUT = the set-algebra values if shipped;
unequal-R / non-orthogonal branched pair → NULL); **sim** = native-vs-OCCT
`BRepAlgoAPI_{Common,Fuse,Cut}` parity on the EXISTING `cyl=cyl(steinmetz)` cases in
`tests/sim/native_ssi_curved_boolean_parity.mm` (today native NULL → OCCT; after this change
native passes). Invoked behind the existing `cc_boolean` op codes — **no `cc_*` entry point
is added or changed**. Consumes the shipped S4-d branched `ssi::TraceSet`
(`MarchOptions.enableBranchPoints = true`), so the branched assembler path is compiled under
**`CYBERCAD_HAS_NUMSCI`**. `src/native/**` stays OCCT-free. **No change to
`src/native/tessellate`** (the S5-a lesson), **no change to `src/native/ssi`** (the S4-d
tracer is consumed unchanged), and **no change to the S5-a/b/c single-seam paths** — additive
only.

## 1. Dispatch: enter the branched re-trace only on the Steinmetz pre-gate edge  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 In `ssi_boolean_solid` (`ssi_boolean.cpp`), keep the gate + DEFAULT trace + the
  single-seam S5-a/b/c dispatch UNCHANGED. Add `steinmetzPreGate(csA, csB)` — both
  `CurvedKind::Cylinder`, `|rA − rB| ≤ tol·max(rA,rB)`, axis dirs orthogonal `|â·b̂| ≤ tol`,
  axis lines crossing within tol. ONLY when the default trace declined
  (`nearTangentGaps > 0`, no usable single seam) AND `steinmetzPreGate` matches, RE-TRACE with
  `MarchOptions{.enableBranchPoints = true}` and route to the branched builders; else NULL →
  OCCT. Short, linear. (**host**)
- [x] 1.2 The dispatcher must leave every single-seam S5-a/b/c path (through-drill
  cyl∩cyl COMMON/FUSE/CUT, sphere∩sphere lens COMMON) bit-identical — their DEFAULT trace is
  unchanged, and the branched re-trace never fires for them (the pre-gate requires equal-R
  orthogonal cylinders, which those pairs are not). Existing S5-a/b/c host tests stay green.
  (**host** ✓ S5-a/b/c regression)

## 2. S5-d — Steinmetz-family branched-trace recognition gate  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 `recogniseSteinmetzTrace(bt, ...)` accepts the branched `TraceSet` ONLY when:
  `nearTangentGaps == 0`; `branchPoints == 2` and `branchNodes.size() == 2`; exactly FOUR
  WLines all `status == BranchArc`; each arm `onSurfResidual ≤ onSurfTol`; each `BranchNode`
  connects all four arms and each arm's two endpoints coincide with the two branch-node points
  (≤ `branchMergeFrac·scale`). Anything else → NULL → OCCT. (**host**)
- [x] 2.2 Deferral host fixtures: an UNEQUAL-radius orthogonal cylinder pair (pre-gate
  reject) and a NON-orthogonal equal-R cylinder pair (pre-gate reject) both return NULL for
  every op; a branched trace with a wrong branch/arm count would be rejected by
  `recogniseSteinmetzTrace`. (**host** ✓ NULL fixtures)

## 3. S5-d — Steinmetz COMMON (the four lune patches)  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 `buildSteinmetzCommon(A, B, arcs, branchPts)` — group the two arcs on each cylinder
  (by their `(u,v)` track); for each of the four candidate lune patches, build it as a planar-
  triangle strip between its two arcs, walking both branch-to-branch in lockstep by arc-length
  fraction, every interior sample ON the analytic cylinder (`cs.point(u,v)`, u folded contiguous
  via `nearU`), the two branch vertices pooled ONCE. KEEP the patch iff its centroid is INSIDE
  the other cylinder (`classifyPoint(other, centroid) == +1`; ON-band → NULL). (**host**)
- [x] 3.2 Weld: ONE `VertexPool` for the whole shell; every arc's shared 3D nodes and the two
  branch-point vertices are pooled so both sides of every arc and all four arcs at each branch
  point draw the SAME vertices → watertight. Seam-adjacent facets are PLANAR triangles through
  the pooled nodes (S5-a discipline; NO analytic surface face on a shared seam), normals outward
  (`radialOut`). `makeShell → makeSolid`. Decimate dense arcs to `seamNodeTarget` first. (**host**)
- [x] 3.3 Volume host check: `ssi_boolean_solid(A, B, Common)` for the equal-R (R=1) orthogonal
  Steinmetz pair is non-NULL, watertight (`watertightMeshVolume > 0`, every edge shared by
  exactly two faces), enclosed volume = the EXACT `16 R³ / 3 = 5.33333` within the ~1%
  deflection band; every arc node on both cylinders ≤ tol; the two branch-point vertices are a
  single shared node. (**host**)

## 4. S5-d — Steinmetz FUSE / CUT (ship only if they verify)  [CYBERCAD_HAS_NUMSCI]
> DEFERRED (honest, verified): FUSE/CUT did NOT ship. `ssi_boolean_solid` dispatches only
> `Op::Common` to `buildSteinmetzCommon`; FUSE/CUT return NULL → OCCT (the `ship only if they
> verify` clause). This is asserted by `branched_fuse_cut_and_disjoint_return_null` (host) and
> shown as the two `fallback → OCCT` rows in the sim parity (volO 32.366 / 13.516, valid+closed).
> `[~]` = deferred, not done. COMMON is the guaranteed slice.
- [~] 4.1 `buildSteinmetzCut(A, B, ...)` = A − B: keep A's wall OUTSIDE B + B's inside-A lune
  patches REVERSED (the tunnel wall) + A's two end caps; drop A's inside-B lunes. Shared arcs
  weld the reversed B patches to A's outside wall; seam-adjacent faces are planar facets through
  the shared pool. Non-Steinmetz / ON-band / open weld → NULL. (**host**)
- [~] 4.2 `buildSteinmetzFuse(A, B, ...)` = A ∪ B: keep each cylinder's OUTSIDE-the-other wall +
  both cylinders' end caps; drop both inside-the-other lunes (now interior). Shared arcs weld the
  two outer walls; planar facets through the shared pool. Non-Steinmetz / ON-band / open weld →
  NULL. (**host**)
- [x] 4.3 Volume host check: CUT = `vol_cyl − 16R³/3` and FUSE = `2·vol_cyl − 16R³/3` within the
  band (`vol_cyl = π R² · axial-extent`), each watertight; monotone invariants (`fuse ≥
  max(A,B)`) hold. If a builder cannot assemble a watertight, correct-volume shell it returns
  NULL → OCCT and the case is reported deferred (COMMON is the guaranteed slice). (**host**)

## 5. Engine self-verify — reuse the EXISTING oracles (no engine change)
- [x] 5.1 Confirm (test-level) that the ENGINE's EXISTING Steinmetz oracle
  (`ssiCurvedBooleanVerified`, `16 R³/3`) ACCEPTS the S5-d COMMON candidate (previously it always
  found NULL and fell to OCCT), and the EXISTING generic set-algebra guard ACCEPTS the S5-d
  FUSE/CUT candidates (`Vr ≈ va+vb−vc` / `va−vc`, `vc` = the native branched COMMON), and both
  DISCARD a mis-welded / wrong-volume candidate → OCCT. NO new oracle is added; the single-seam
  S5-a/b/c guards stay untouched and do not fire for the branched Steinmetz case. (**host**)

## 6. Honest scope — deferrals (never faked)
- [x] 6.1 General / non-Steinmetz branched pairs — unequal-R or non-orthogonal or non-crossing
  branched cylinder pairs; cyl∩sphere / cyl∩cone / cone∩cone self-crossings; ≠ 2 branch points
  or ≠ 4 arms; any branched trace with `nearTangentGaps > 0`; freeform branched — all → NULL
  (OCCT), documented in the `ssi_boolean.h` header scope note + the `native-booleans` /
  `native-ssi` namespace docs. (**host** ✓ NULL fixtures: unequal-R + non-orthogonal branched)

## 7. Verification (two gates)
- [x] 7.1 Host suite extension of `test_native_ssi_curved_boolean.cpp`: S5-d Steinmetz COMMON
  volume = `16 R³/3` + watertight + arc-node residual + shared branch vertices; FUSE/CUT
  set-algebra volume + watertight (if shipped); unequal-R + non-orthogonal branched → NULL. Full
  CTest green NUMSCI ON and OFF (new assertions absent with NUMSCI off). No OCCT; no tolerance
  weakened. (**host**)
- [x] 7.2 Sim parity: the EXISTING `cyl=cyl(steinmetz)` COMMON/FUSE/CUT cases in
  `native_ssi_curved_boolean_parity.mm` + `run-sim-native-ssi-curved-boolean.sh` become NATIVE
  passes vs `BRepAlgoAPI_{Common,Fuse,Cut}` (volume, surface area, watertight closed shell,
  `BRepCheck` validity within tol); update the harness comments (the Steinmetz pair is now a
  native pass, not the honest fall-back it records today); report deltas + the count still
  deferred to OCCT; run via `xcrun simctl spawn <booted udid>`. (**sim**)
- [x] 7.3 `openspec validate add-native-ssi-branched-boolean --strict` green; update
  `SSI-ROADMAP.md` S5 (S5-d Steinmetz branched COMMON — and FUSE/CUT if shipped — done at the
  bar with measured deltas vs `16 R³/3` + OCCT), `ROADMAP.md` / `NATIVE-REWRITE.md` /
  `README.md` where they cite S5; general branched booleans stay the tail.

## Deferred to later S5 (NOT in this change — honest)

- [ ] **General / non-Steinmetz branched booleans** — unequal-R or non-orthogonal branched
  cylinder pairs, cyl∩sphere / cyl∩cone / cone∩cone self-crossings, ≥ 3 branch points, freeform
  branched → OCCT until a later S5 stage.
- [ ] **Steinmetz FUSE / CUT** if they do not robustly assemble a watertight, correct-volume
  shell in this change → OCCT (COMMON is the guaranteed slice; FUSE/CUT ship only if verified).
- [ ] **Branched traces with `nearTangentGaps > 0`** (an arm the S4-d marcher could not resolve)
  → the honest S4 boundary; decline → OCCT.
