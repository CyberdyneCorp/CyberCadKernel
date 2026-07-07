# Tasks — add-native-construction-breadth (#4 native construction, Tier-4 breadth)

## PER-TRACK HONESTY (the gate each track must pass to land)

- **T1 — mismatched-count ruled loft: EXPECTED TO LAND, EXACT.** An arc-length-parameter UNION
  correspondence (collinear point insertion, geometry-preserving) lets an M-gon loft to an
  N-gon behind the UNCHANGED `cc_solid_loft` / `cc_solid_loft_wires`; volume equals the true
  ruled-loft volume, self-verified watertight. Highest confidence.
- **T2 — orientation-constraining guided sweep: EXPECTED HONEST DECLINE (no dead code).**
  BLOCKER (to confirm on the sim): the shipped `cc_guided_sweep` oracle is the SCALE-splay
  ThruSections (already native); an orientation frame has no oracle behind that fixed entry
  without an ABI/semantics break, and its solid fails parity vs the shipped scale oracle. Land
  ONLY if a real MakePipeShell guide-orientation oracle is reachable behind an existing entry
  AND the planar slice self-verifies + matches it; else documented OCCT-fallthrough, gap
  REPORTED.
- **T3 — fine-pitch self-intersecting thread: EXPECTED HONEST DECLINE (no dead code).** BLOCKER
  (to confirm): crossing radial-V flanks are two intersecting helicoids; a single ruled tiling
  is non-manifold and volume-wrong, and trimming them needs Tier-4 SSI outside this batch. Land
  ONLY if a narrow crossing-flank slice self-verifies watertight + correct volume vs OCCT; else
  the `kMaxLeadRatio` guard stays and the self-intersecting thread remains OCCT-fallthrough, gap
  REPORTED.

Verification levels: **host** = OCCT-free host CTest (sections/skin watertight + correct
volume + the right geometry); **sim** = native-vs-OCCT `BRepOffsetAPI_ThruSections` /
`MakePipeShell` parity on a booted simulator (volume / area / bbox / watertight within the
bound). All three tracks land behind the UNCHANGED `cc_solid_loft` / `cc_solid_loft_wires` /
`cc_guided_sweep` / `cc_helical_thread` / `cc_tapered_thread` — no new ABI.

> Mechanism note: T1 generalizes `src/native/construct/loft.h` `build_ruled_loft_sections`
> (add `detail::correspondByArcLength` when counts differ → equal-`K` loops → existing
> `alignSectionB` + `appendRuledBand`). T2 adds `detail::orientedGuideFrames` +
> `build_guided_sweep_oriented` to `src/native/construct/sweep.h` — RETAINED only if it
> self-verifies + matches a real guide-orientation oracle, else an honest decline. T3 attempts a
> crossing-flank slice in `src/native/construct/thread.h` (SSI-trimmed flanks, NUMSCI-gated) —
> RETAINED only if it self-verifies, else an honest decline. The engine keeps the mandatory
> `robustlyWatertight` + positive-volume self-verify around every native result.

## T1 — Mismatched-count ruled loft (highest confidence)

### 1. Arc-length vertex correspondence
- [x] 1.1 Add `detail::loopParams(pts)` — the normalized cumulative arc-length parameter of
  every loop vertex (`param(p[i]) = Σ_{k<i}|edge_k| / perimeter`), and `detail::resampleAt(pts,
  params, t)` — the point ON the edge whose param interval contains `t`. (**host**)
- [x] 1.2 Add `detail::correspondByArcLength(loopA, loopB)` — resample BOTH loops at the sorted
  UNION of their params (dedup within `kProfileTol`), returning two equal-`K` loops
  (`K ≤ M + N`). Every original vertex survives; every inserted point is COLLINEAR (geometry
  preserved). (**host**)
- [x] 1.3 In `build_ruled_loft_sections`, when the counts differ, run the correspondence
  PAIRWISE down the chain (section `k` resampled against the already-fixed section `k−1`) before
  `alignSectionB`. The equal-count path is the `K = M = N` special case — BYTE-UNCHANGED. Guards
  unchanged: PLANAR + non-degenerate; a zero-perimeter loop declines → NULL → OCCT. (**host**)

### 2. Engine self-verify on the loft dispatch
- [x] 2.1 `NativeEngine::solid_loft` / `solid_loft_wires` gain the mandatory self-verify
  `robustlyWatertight(solid) && watertightVolume(solid) > 0.0`; NULL / unverified ⇒ forward the
  SAME arguments to OCCT. The equal-count result stays native (byte-identical behaviour). A
  self-intersecting resampled skin fails watertight → OCCT. (**host** + **sim**)

### 3. Host + parity for T1
- [x] 3.1 Host: an M-gon→N-gon loft (`M ≠ N`, e.g. triangle→hexagon, square→pentagon) meshes
  watertight; its volume EQUALS the analytic ruled-loft volume between the two polygons; the
  resampled sections preserve each polygon's area; the equal-count control is byte-identical; a
  non-planar / punctual / self-intersecting section declines → NULL. (**host**)
- [x] 3.2 Sim: `cc_solid_loft` / `cc_solid_loft_wires` native vs OCCT
  `BRepOffsetAPI_ThruSections` on ≥ 2 mismatched-count fixtures — TIGHT bound (volume / bbox /
  watertight agree to fp precision; the resample is exact). (**sim**)

## T2 — Orientation-constraining guided sweep (narrow slice OR honest decline)

### 4. Oriented-guide frame (planar slice)
- [ ] 4.1 Add `detail::orientedGuideFrames(spine, tan, guide)` — per station the up-axis
  `= normalize((guide(f) − path(f)) − tangent·((guide−path)·tangent))`, `nrm = tangent × up`; a
  pure per-station function (reproducible). Restrict to a PLANAR spine + PLANAR guide, no twist,
  `guide − path` never ∥ tangent. Any violation ⇒ NULL → OCCT. (**host**)
- [ ] 4.2 Add `build_guided_sweep_oriented(...)` reusing `assembleRingTube`; the section is
  AIMED at the guide (up-axis tracks the guide). (**host**)

### 5. Honest gate (no dead code)  — DECLINED (gap reported)
> STATUS: HONEST DECLINE. `sweep.h` is UNTOUCHED — no `orientedGuideFrames` /
> `build_guided_sweep_oriented` was added (4.1/4.2 intentionally unimplemented, no dead code).
> The shipped `cc_guided_sweep` oracle is the SCALE-splay `ThruSections` and native
> `build_guided_sweep` already matches it; an orientation-guide frame law has NO oracle behind
> that fixed entry without an ABI/semantics break and would fail parity vs the scale oracle.
> Orientation-guided sweep stays OCCT-fallthrough; RMF/scale-splay controls byte-identical.
- [x] 5.1 DECIDED (decline): determined on the sim whether a REAL OCCT MakePipeShell guide-ORIENTATION oracle is
  reachable behind an existing entry AND the planar slice self-verifies watertight + matches it
  within the bound. IF yes → wire `build_guided_sweep_oriented` into `NativeEngine::guided_sweep`
  before the scale-splay path (accepted only through `robustlyWatertight`). IF no (the realistic
  outcome — the fixed `cc_guided_sweep` oracle is scale-splay, an orientation law breaks its
  semantics or fails parity) → DO NOT retain a builder; orientation-guided sweep stays a
  documented OCCT-fallthrough, the measured gap REPORTED. No dead code. (**host** + **sim**)

## T3 — Fine-pitch self-intersecting thread (narrow slice OR honest decline)

### 6. Crossing-flank slice (NUMSCI-gated)
- [ ] 6.1 Attempt a narrow crossing-flank slice just past `kMaxLeadRatio`: compute the
  intersection curve of the two overlapping flank helicoids and TRIM each flank to it (SSI,
  `CYBERCAD_HAS_NUMSCI`-gated), re-tile into ruled bands + caps. Restrict to a shallow taper,
  root clear of the axis. Outside the slice / no NUMSCI ⇒ NULL → OCCT. (**host**, NUMSCI-ON)

### 7. Honest gate (no dead code)  — DECLINED (gap reported)
> STATUS: HONEST DECLINE. `thread.h` is UNTOUCHED — no crossing-flank SSI slice was added
> (6.1 intentionally unimplemented, no dead code). Crossing radial-V flanks are two intersecting
> helicoids; a single ruled tiling is non-manifold / volume-wrong, and trimming needs Tier-4 SSI.
> The watertight-only engine self-verify cannot catch the volume divergence, so the `kMaxLeadRatio`
> guard STAYS and the self-intersecting thread remains OCCT-fallthrough; helical/tapered controls
> byte-identical.
- [x] 7.1 DECIDED (decline): land ONLY IF the trimmed slice self-verifies watertight + correct volume + OCCT
  `BRepOffsetAPI_MakePipeShell` parity on its fixture; then raise the `kMaxLeadRatio` guard for
  exactly that slice. If it does NOT build robustly, DO NOT retain a dead builder — the
  self-intersecting thread stays a documented OCCT-fallthrough (`kMaxLeadRatio` unchanged), the
  measured gap REPORTED. No dead code. (**host** + **sim**)

## 8. Engine dispatch + out-of-slice defers
- [x] 8.1 `NativeEngine::solid_loft` / `solid_loft_wires` = native loft (count-agnostic) →
  self-verify → OCCT. `guided_sweep` = [oriented slice iff landed] → scale-splay → self-verify →
  OCCT. `helical_thread` / `tapered_thread` = [crossing-flank slice iff landed] → resolved
  thread → self-verify → OCCT. Each candidate under `robustlyWatertight` (+ positive volume for
  loft). (**host**)
- [x] 8.2 Out-of-slice inputs return NULL (asserted in scope-defer tests): T1 — non-planar /
  punctual / self-intersecting section; T2 — non-planar spine or guide, twist, `guide ∥ tangent`,
  no orientation oracle; T3 — a fold beyond the narrow slice, root-dive taper, no NUMSCI.
  (**host** + **sim**)

## 9. Verification (two gates)
- [x] 9.1 Host suite (no OCCT): **T1** `mismatched_loft_watertight_volume`,
  `mismatched_loft_area_preserved`, `equal_count_loft_byte_identical`, `loft_scope_defers`.
  **T2** `oriented_guide_planar_slice_watertight` OR `oriented_guide_declines` (documented).
  **T3** (NUMSCI-ON) `crossing_flank_slice_watertight_volume` OR `crossing_flank_declines`
  (documented). Built default + NUMSCI-ON. (**host**)
- [x] 9.2 Sim parity: extend `scripts/run-sim-native-loft.sh` + `tests/sim/native_loft_parity.mm`
  with T1 (`cc_solid_loft` / `cc_solid_loft_wires` vs `BRepOffsetAPI_ThruSections`, ≥ 2
  `M ≠ N` fixtures, TIGHT bound); extend `scripts/run-sim-native-sweep.sh` +
  `tests/sim/native_sweep_parity.mm` with T2 (`cc_guided_sweep` vs MakePipeShell guide mode iff
  landed, else decline-parity); extend `scripts/run-sim-native-thread.sh` +
  `tests/sim/native_thread_parity.mm` with T3 (fine-pitch crossing-flank vs MakePipeShell iff
  landed, else decline-parity). HARD native gates: watertight, mesh↔B-rep vol within bound. A
  fixture beyond tol → out of slice (NULL → OCCT), gap REPORTED. (**sim**)
- [x] 9.3 No regression: `run-sim-native-loft.sh` + `run-sim-native-sweep.sh` +
  `run-sim-native-thread.sh` stay green; native construct/profiles, booleans, SSI, healing,
  import host suites green under NUMSCI + default; `run-sim-suite.sh` unchanged count.
  (**sim** + **host**)
- [x] 9.4 `openspec validate add-native-construction-breadth --strict` green. (**host**)

## Deferred (NOT in this batch — honest NULL → OCCT, gap REPORTED)

- [ ] **Loft with a NON-PLANAR end section, a punctual section, or a self-intersecting
  resampled skin** → OCCT (T1 is the PLANAR polygon slice only).
- [ ] **Guided sweep beyond the planar oriented slice** (non-planar spine/guide, accumulated
  twist, `guide ∥ tangent`) → OCCT — and, if no orientation oracle is reachable behind
  `cc_guided_sweep`, the WHOLE T2 track is an honest decline (no dead code), gap REPORTED.
- [ ] **Fine-pitch thread beyond the narrow crossing-flank slice** (deeper folds, root-dive
  taper, non-NUMSCI) → OCCT — and, if the narrow slice is not robustly buildable, the WHOLE T3
  track is an honest decline (no dead code), gap REPORTED.
- [ ] **Any freeform (NURBS/Bézier/B-spline) section or spine** → OCCT.
