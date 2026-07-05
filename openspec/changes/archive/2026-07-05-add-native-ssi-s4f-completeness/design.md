# Design — add-native-ssi-s4f-completeness (SSI Stage S4-f, first slice)

## Context

The native SSI stack is S1 analytic → S2 seeding → S3 marching, hardened by S4-a/S4-b
CLASSIFICATION (typed `CoincidentRegion` + `TangentContact`), S4-c near-tangent MARCHING-THROUGH,
S4-d BRANCH-POINT localization + arm routing, and S4-e CHART-SINGULARITY crossing (sphere pole /
cone apex). Those slices all attack DEGENERACY. **S4-f is the least-glamorous, most-important-for-
production slice: it adds NO new geometry capability — it HARDENS the CORRECTNESS / COMPLETENESS
of curves the tracer already produces.** Two orthogonal, silent holes:

- **Hole 1 — loop closure is pure PROXIMITY.** `marching.cpp` `DirEnd::LoopClosed` fires when the
  march returns within `closeRadius(w,t) = loopClose · h` of the seed. Proximity is necessary but
  NOT sufficient:
  - **FALSE-CLOSE** — a curve passing NEAR its seed while heading the OTHER way is declared closed
    and truncated. `closeAligned()` guards direction, but ONLY once a graze was crossed
    (`w.crossedAny`); a pure S3 curve has no such guard (it returns `true` unconditionally).
  - **SELF-CROSSING SKIPPED** — a single-arm curve that genuinely self-intersects (crosses an
    EARLIER non-seed node) is not the seed, so the closure test cannot see it; it is stepped past
    unrecorded, or (if near the seed) mis-closed.
- **Hole 2 — the fixed-resolution seeder silently misses small loops.** S2 recall is gated by
  `SeedOptions.minPatchFrac` (default 1/32) + `initialGridU/V` + `maxDepth`. A loop entirely inside
  one leaf cell yields no candidate region → no seed → no curve. `seeding.h` documents the hole;
  nothing recovers it. Recall is silently `< 1` and the caller cannot see which loops were lost.

### The three ways S4-f is NOT the prior slices

- **S4-c near-tangent** = the SURFACE PAIR grazes (`‖n₁×n₂‖ → 0`). Witnessed by the sine collapse.
  S4-f closure / self-crossing is on TRANSVERSAL curves (healthy sine), so S4-c never fires there.
- **S4-d branch point** = the intersection LOCUS self-crosses (MULTIPLE arms meet at one point;
  sine → 0 + a raw-tangent flip). A S4-f self-intersection is ONE arm crossing ITSELF while both
  surfaces stay transversal — the locus does NOT branch, the sine does NOT collapse, no new arms
  emanate. Distinct witness (node-history crossing, not a locus flip).
- **S4-e chart singularity** = a single SURFACE's `(u,v)` chart degenerates (`‖dU‖ → 0`). S4-f is
  about COMPLETENESS + CLOSURE of well-charted transversal curves, not chart degeneracy.

### Diagnosis (the "before", confirmed on the current tracer)

Host build, `CYBERCAD_HAS_NUMSCI` ON:

| Fixture | S3 / S4-e result today | Verdict |
|---|---|---|
| **(A) SMALL LOOP MISSED** — a pair whose intersection has a small loop entirely inside one default (1/32) leaf cell | S2 emits 0 seeds for the loop (no candidate region survives the AABB prune at the default resolution); `RecallReport.recall() < 1` | **MISSED** — real, on both surfaces, below the fixed floor; nothing recovers it |
| **(B) FALSE-CLOSE** — an OPEN / longer curve whose trace passes within `loopClose·h` of the seed while heading the other way | one WLine, `status = Closed`, truncated at the near-pass; `closeAligned` inert (`crossedAny == false`) | **FALSE-CLOSED** — proximity stopped a real curve early |
| **(C) SELF-INTERSECTION** — a single-arm curve that genuinely self-crosses (ONE arm crossing itself, NOT the locus branching) | crossing near an earlier node mis-read as a loop close or stepped past unrecorded | **SKIPPED / MIS-CLOSED** |
| **(D) MANY SMALL LOOPS** — an adversarial pair with several disjoint small loops (the wrap-emboss / blend seam pattern) | most loops below the floor unseeded; `recall() ≪ 1` | **MOSTLY MISSED** |
| 5 transversal S3 pairs | `nt = 0`, bit-identical; each loop closes tangent-continuously | **must stay UNTOUCHED** |
| S4-c graze / S4-d Steinmetz / S4-e pole+apex | crossed / `branchPoints = 2` / crossed | **must stay UNCHANGED** |

The method is clean-room; OCCT (`GeomAPI_IntSS` / `IntPatch`) is the verification ORACLE only — the
true-return closure predicate, the self-intersection guard, and the completeness-critic re-seed are
re-derived, never copied.

**HONEST FRAMING (scope-setting — reflected in the spec, tests, docs).**
- **Completeness is ASYMPTOTIC, never a proof.** Below ANY fixed subdivision resolution there is
  always a smaller loop that can be missed. This slice claims MEASURED recall wins on concrete
  fixtures vs OCCT and RAISES the completeness floor — it does NOT claim a completeness guarantee.
  Recall is reported as a MEASURED figure with the residual acknowledged (a loop smaller than the
  finest re-seed round can still exist). A fixture's recall `→ 1` is a claim about THAT fixture at
  THAT floor, not a general theorem.
- **S4-f DE-RISKS (does NOT unblock / complete) curved blends (#6) + wrap-emboss (#7).** Their
  seams are exactly the small-loop / self-intersecting / many-loop patterns S4-f hardens, so the
  SSI-curve completeness they depend on becomes measurably more trustworthy — but #6/#7 still need
  their wider S5/S6/S7 assemblers, which this change delivers none of.

## Goals / Non-Goals

**Goals**
- (S4-f-1) TRUE-RETURN + TANGENT-CONTINUOUS loop closure: close ONLY on a verified return to the
  seed heading the way it left (generalize the `closeAligned` direction gate to ALL closures, not
  just post-graze). Reduces to the current proximity result on curves that truly close.
- (S4-f-2) SELF-INTERSECTION guard: detect the arm crossing an EARLIER NON-SEED node (proximity +
  a transverse, non-retrace tangent) as a typed self-intersection — NOT a loop close, NOT an S4-d
  branch; record it and continue the arm to its true termination.
- (S4-f-3) BOUNDED ADAPTIVE COMPLETENESS-CRITIC re-seed: after the initial seed + trace, re-seed
  the uncovered param regions at finer `minPatchFrac`, refine + dedup against all kept curves,
  trace new seeds, LOOP-UNTIL-DRY (K dry rounds) — bounded by a cost cap — to RECOVER small loops.
- (S4-f-4) HONEST RECALL FLOOR: report the finest re-seed fraction reached, the rounds run, dry-vs-
  cap stop, and the measured recall WITH the residual that a smaller loop can still exist.
- Verify MEASURED recall wins on fixtures A–D vs OCCT; the false-close curve traced end-to-end; the
  self-crossing detected + traced through; the residual acknowledged.

**Non-Goals (deferred — never faked here)**
- **A completeness GUARANTEE / proof.** The critic raises the floor; it never proves no loop is
  missed. The residual is always reported.
- **Global topology repair / watertight self-intersection resolution.** Splitting a self-crossing
  arm into topological sub-arcs, healing a self-intersecting shell — those are S5/S6 assembler
  concerns; S4-f DETECTS + REPORTS + traces-through the self-intersection, it does not repair
  topology around it.
- **Completing curved blends (#6) / wrap-emboss (#7).** S4-f de-risks them; their assemblers are
  out of scope.
- **Any change to `src/native/tessellate`, the `cc_*` ABI, the S3 transversal trace, the S4-c
  crossable-graze crossing, the S4-d branch machinery, or the S4-e chart-singularity crossing.**
  The closure generalization reduces to the current result on truly-closing curves; the critic +
  self-intersection guard are default-off.
- **Weakening `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection`, `loopCloseFrac`, or any
  tolerance to "pass".** A loop the critic cannot recover is an honestly reported measured gap
  (recall `< 1`).

## Module shape

```
src/native/ssi/marching.h            [extend — additive result fields + closure/critic knobs]
  struct WLine {
    // ... existing fields unchanged ...
    int selfIntersectionCount = 0;   // S4-f: genuine self-crossings on THIS arm (0 for a simple curve)
    std::vector<SelfIntersection> selfIntersections{}; // typed crossing points (point, params, tangents)
  }
  struct TraceSet {
    // ... existing fields unchanged ...
    int selfIntersections = 0;       // S4-f: self-crossings detected across all arms
    int criticRounds = 0;            // S4-f: completeness-critic re-seed rounds run (0 if critic off)
    int criticRecoveredLoops = 0;    // S4-f: NEW branches the critic recovered (verified, deduped)
    double criticFloorFrac = 0.0;    // S4-f: finest minPatchFrac the critic re-seeded at (the floor reached)
    bool  criticStoppedDry = false;  // S4-f: true → stopped after K dry rounds; false → hit the cost cap
    bool  completenessResidual = true; // S4-f: ALWAYS true — a loop smaller than criticFloorFrac can still exist
  }
  struct MarchOptions {
    // ... existing fields unchanged ...
    bool   enableSelfIntersection = false; // S4-f part 1: self-intersection guard (off → S3 behaviour)
    double selfIntersectRadiusFrac = -1.0; // proximity for a self-crossing vs an earlier node (≤0 → 1e-3·scale-ish)
    double closureTangentCos = -1.0;       // tangent-continuity cosine for a TRUE close (≤0 → -0.5: block only a clear pass-through)
    // (robust TRUE-RETURN closure is ALWAYS on — it reduces to today's result on truly-closing curves;
    //  closureTangentCos only sets how antiparallel a heading must be to BLOCK a false-close.)
  }
  struct SelfIntersection {           // S4-f typed self-crossing (additive)
    math::Point3 point{}; double u1,v1,u2,v2; // where the arm crosses itself (on both surfaces)
    int nodeIndexA = 0, nodeIndexB = 0;        // the two node indices of the arm that coincide
  }

src/native/ssi/seed.h                [extend — additive RecallReport floor fields]
  struct RecallReport {
    // ... existing fields unchanged ...
    double criticFloorFrac = 0.0;    // finest minPatchFrac reached by the critic (0 → critic not run)
    bool   residualAcknowledged = true; // ALWAYS true: a loop below criticFloorFrac can still exist
  }
  struct SeedOptions {                // (in seeding.h) extend — critic knobs
    // ... existing fields unchanged ...
    bool   completenessCritic = false; // S4-f master switch (off → the current fixed-resolution seed)
    double criticRefineFactor = 0.5;   // per-round minPatchFrac multiplier (finer each round)
    int    criticDryRounds = 2;        // K: consecutive rounds with NO new branch before stopping (loop-until-dry)
    int    criticMaxRounds = 6;        // hard cost cap on rounds
    int    criticMaxCandidates = 4096; // hard cost cap on total re-seed candidate regions
  }

src/native/ssi/completeness_critic.h [NEW — OCCT-free, the S4-f coverage + re-seed math]
  namespace critic {
    struct Coverage { /* coarse param-grid occupancy from traced polylines' footprints */ };
    // coverageOf(traced WLines, A.domain, B.domain, gridN) → Coverage
    // uncoveredBoxes(Coverage) → std::vector<ParamBox>  (regions with NO traced curve)
    // (No refine/trace here — the re-seed loop lives in seeding.cpp/marching.cpp, which own
    //  seed_intersection + dedup, exactly as branch_point.h leaves routing to marching.cpp.)
  }

src/native/ssi/seeding.cpp           [extend — the critic re-seed loop, CYBERCAD_HAS_NUMSCI]
  // seed_intersection_critic(A,B, opts) → SeedSet + RecallReport floor:
  //   run seed_intersection(opts) once (the fixed floor); then loop-until-dry:
  //   re-seed uncovered boxes at minPatchFrac *= criticRefineFactor, dedup NEW seeds vs kept,
  //   stop after criticDryRounds dry rounds or the cost cap; report criticFloorFrac + residual.

src/native/ssi/marching.cpp          [extend — closure predicate + self-intersection guard]
  // closeAligned(...) generalized: apply the tangent-continuity gate to ALL closures (not just
  //   crossedAny), with closureTangentCos as the block threshold — a near-pass the other way
  //   does NOT close. Byte-identical on the transversal pairs (they return tangent-continuously).
  // selfIntersect(w, out, radius): over the arm's own node history, detect a crossing of an
  //   EARLIER non-seed node with a transverse (non-retrace) tangent → record a SelfIntersection,
  //   ++selfIntersectionCount, DO NOT close, DO NOT stop — continue the arm. Off → no-op (S3).
  // trace_from_seeds: when SeedOptions.completenessCritic on, drive the critic re-seed loop and
  //   trace the recovered seeds (dedup vs kept), tallying criticRounds / criticRecoveredLoops /
  //   criticFloorFrac / criticStoppedDry. Off → byte-identical to today.
```

`src/native/**` stays OCCT-free. The new machinery lives in the new header
`completeness_critic.h` (coverage math, header-only) and in `marching.cpp` / `seeding.cpp` under
`CYBERCAD_HAS_NUMSCI` (the critic re-uses `seed_intersection` + the existing dedup; the closure /
self-intersection guard use only the march's own node history). No new substrate routine; no new
hand-tuned constant beyond the critic / closure discriminators (documented, sentinel-resolved,
never weakening a tolerance).

## S4-f-1 — True-return + tangent-continuous loop closure

Today `marchDir` closes when `distance(cur, seed) ≤ closeRadius(w,t)` AND `closeAligned(w, fwd)`,
and `closeAligned` returns `true` unconditionally unless a graze was crossed. The generalization:
the tangent-continuity gate applies to EVERY closure —

```
close  ⇔  step > 2
          AND distance(cur, seed) ≤ closeRadius(w,t)          (proximity — unchanged)
          AND dot(fwdNow, seedFwd) ≥ closureTangentCos·‖·‖    (TRUE-RETURN: heading the way it left)
```

`seedFwd` (the seed's outgoing forward tangent) is ALREADY captured in `Walk` (`haveSeedFwd`).
`closureTangentCos` defaults to a GENEROUS block threshold (`-0.5`): a close is BLOCKED only when
the return heading is CLEARLY the other way (a near-antiparallel pass-through). A genuine loop
returns nearly PARALLEL to `seedFwd` (`dot ≈ +1`), so it still closes exactly as today — the 5
transversal pairs and every existing loop-closing control are BYTE-IDENTICAL (they close because
they truly return tangent-continuously; the generalized gate accepts them). A FALSE-CLOSE
near-pass returns roughly ANTIPARALLEL (`dot ≈ −1 < −0.5`), so it is BLOCKED and the march
continues to the curve's true termination. This is a NECESSARY-condition tightening: it can only
REFUSE a close the proximity test would have made — it never MAKES a close the proximity test
would not. So a curve that truly closes is unaffected and a false-close is prevented; nothing new
is fabricated.

## S4-f-2 — Self-intersection guard

The arm keeps its accepted node history (`out`, already the march's node vector). At each accepted
node the guard scans EARLIER non-seed, non-adjacent nodes (index `< current − guardGap`):

```
self-crossing at node i vs earlier node j  ⇔
     distance(node_i.point, node_j.point) ≤ selfIntersectRadiusFrac·scale
     AND the arm's local tangent at i is TRANSVERSE to its tangent at j
         (|dot(t_i, t_j)| < 1 − εcont — NOT a continuation and NOT a retrace)
```

A match is a genuine SELF-CROSSING (the arm passes through the same 3D point twice, headed
differently). It is:
- NOT a loop close — node `j` is not the seed (the seed is node 0 and is excluded from the guard);
- NOT an S4-d branch — both surfaces are TRANSVERSAL at a curve self-crossing (`‖n₁×n₂‖` healthy,
  no locus flip, no new arms emanate — the locus does not branch, ONE arm crosses ITSELF).

The guard RECORDS a typed `SelfIntersection` (point, both param pairs, the two node indices),
increments `selfIntersectionCount`, and DOES NOT close and DOES NOT stop — the arm continues to
its true termination (boundary / genuine close / budget). Reporting it as data (not a stop) means
a false positive costs a spurious COUNT, not a wrong curve. Default OFF (`enableSelfIntersection`);
off → the guard is a no-op and the trace is byte-identical to S3. The retrace-vs-crossing
distinction (the `|dot| < 1 − εcont` transversality) is what separates a real self-crossing from
the arm merely running back over itself (a dedup / periodic-seam artifact, `dot ≈ ±1`), which is
NOT reported.

## S4-f-3 — Bounded adaptive completeness-critic re-seed (loop-until-dry)

After the initial `seed_intersection` (the fixed 1/32 floor) + the S3/S4 trace, with
`SeedOptions.completenessCritic` on:

1. **COVERAGE.** Build a coarse param-grid occupancy (`critic::coverageOf`) from the traced
   WLines' footprints on BOTH surfaces (each node marks its cell). `uncoveredBoxes` returns the
   `ParamBox`es with NO traced curve — the regions a small loop could hide in.
2. **RE-SEED FINER.** Re-run the subdivision seeder RESTRICTED to the uncovered boxes at
   `minPatchFrac *= criticRefineFactor` (finer than the previous round). Refine each new candidate
   (the SAME least-squares refine, SAME `onSurfTol` — no weakened tolerance); a candidate that
   does NOT refine to an on-both-surfaces point (or is near-tangent) is DISCARDED, never a seed.
3. **DEDUP + TRACE.** Dedup the new seeds against ALL kept curves (the existing 3D-proximity +
   locus dedup). Trace each genuinely NEW seed (one WLine per seed, same marcher). A traced curve
   that retraces a kept one is dropped (`dedupedRetraces`), never double-counted.
4. **LOOP-UNTIL-DRY.** Repeat 1–3. Stop when `criticDryRounds` (K, default 2) CONSECUTIVE rounds
   recover NO new branch, OR when the cost cap (`criticMaxRounds` / `criticMaxCandidates`) is hit —
   whichever first. Record `criticRounds`, `criticRecoveredLoops`, `criticFloorFrac` (the finest
   `minPatchFrac` reached), `criticStoppedDry`.

Every recovered loop is a VERIFIED on-both-surfaces seed that refined to a real curve — NEVER a
fabricated branch. The critic RAISES the floor (recovers loops down to `criticFloorFrac`); it does
not PROVE completeness (a loop below `criticFloorFrac` can still exist — reported as the residual).

## S4-f-4 — Honest recall floor reporting

The critic reports, and the sim gate asserts against OCCT:
- `criticFloorFrac` — the finest `minPatchFrac` re-seeded at (the resolution floor reached);
- `criticRounds` + `criticStoppedDry` — how many rounds ran and whether it stopped DRY (K dry
  rounds — the honest "nothing more found at this floor") or on the COST CAP (more may exist below
  the cap; explicitly the weaker outcome);
- the MEASURED `recall()` (RecallReport) — native branches carrying ≥1 seed ÷ OCCT branch count,
  NEVER asserted a blind 1.0;
- `completenessResidual` / `residualAcknowledged` — ALWAYS true: a loop smaller than
  `criticFloorFrac` can still exist. The recall figure is a MEASURED win on THIS fixture at THIS
  floor, not a completeness proof.

On fixture (A) the report shows recall `< 1` at the default floor and `= 1` after the critic (the
one small loop recovered) — WITH the residual still acknowledged. On (D) the report shows the
measured recall RISING toward OCCT's branch count as the floor tightens, stopping dry or on the cap,
with the residual acknowledged — a measured win, not a claim of totality.

## Crossed-vs-reported scope (honest)

| Configuration | S4-f action | counted as |
|---|---|---|
| Curve returns to the seed tangent-continuously (heading the way it left) | **CLOSE the loop** (as today) | `closedCurves` (unchanged) |
| Curve passes NEAR the seed heading the OTHER way (antiparallel) | do NOT close — continue to true termination | (open/longer curve, no false-close) |
| Arm crosses an EARLIER non-seed node transversally (self-intersection) | RECORD typed `SelfIntersection`, continue the arm | `selfIntersections`, `selfIntersectionCount` |
| Arm runs back over itself (retrace, `dot ≈ ±1`) — not a crossing | NOT reported (dedup/seam artifact) | — |
| Small loop inside a leaf cell, critic recovers a verified seed + curve | TRACE the recovered loop | `criticRecoveredLoops`, `tracedBranches` |
| Re-seed candidate that does NOT refine on both surfaces | DISCARD (never a fabricated branch) | — |
| Loop smaller than `criticFloorFrac` (below the finest round) | NOT recovered — reported as residual | measured recall `< 1` + `completenessResidual` |
| PAIR near-tangent graze (`‖n₁×n₂‖ → 0`) — S4-c | S4-c MARCH THROUGH (unchanged) | `nearTangentCrossed` |
| Locus self-crossing (branch point) — S4-d | S4-d localize + route (unchanged) | `branchPoints` |
| Chart singularity (sphere pole / cone apex) — S4-e | S4-e step across (unchanged) | `singularitiesCrossed` |
| Transversal curve, healthy chart | normal S3 march (unchanged) | traced |

## Verification model (two gates)

- **Host (no OCCT).** Extend `tests/native/test_native_ssi_marching.cpp` +
  `tests/native/test_native_ssi_seeding.cpp` (or a new
  `tests/native/test_native_ssi_s4f_completeness.cpp`), all under `CYBERCAD_HAS_NUMSCI`:
  - **(A) small loop recovered.** A pair with one small loop inside a default (1/32) leaf cell:
    assert the DEFAULT seed misses it (`recall() < 1`, 0 seeds for that loop), then the critic
    (`completenessCritic = true`) recovers it (`recall() == 1` on THIS fixture, `criticRecoveredLoops
    ≥ 1`, `criticFloorFrac` finer than 1/32) — with `completenessResidual == true` still reported.
  - **(B) false-close prevented.** An open/longer curve whose S3 trace passes within `loopClose·h`
    of the seed heading the other way: assert it is NO LONGER `Closed` early — it is traced to its
    TRUE termination (arc length matching the analytic ground truth); the true-return gate blocked
    the false-close.
  - **(C) self-intersection detected + traced through.** A single-arm self-crossing curve (ONE arm
    crossing itself, both surfaces transversal): assert `selfIntersections ≥ 1`, a typed
    `SelfIntersection` at the analytic crossing point (on both surfaces ≤ `onSurfTol`), the arm
    traced THROUGH it (not false-closed, not skipped), and DISTINCT from S4-d (`branchPoints == 0`).
  - **(D) many small loops.** An adversarial pair with several disjoint small loops: assert the
    critic MEASURABLY raises recall over the default (`recall_after > recall_before`), report the
    floor + `criticStoppedDry`, and acknowledge the residual — no claim of totality.
  - **Regression.** The 5 transversal pairs trace bit-identically (`nt == 0`, same closure — each
    still `Closed`); the S4-c graze STILL crosses (`nearTangentCrossed ≥ 1`); the S4-d Steinmetz
    STILL traces (`branchPoints == 2`, `selfIntersections == 0`); the S4-e pole + apex STILL cross
    (`singularitiesCrossed ≥ 1`). With `completenessCritic` + `enableSelfIntersection` OFF the whole
    trace is BYTE-IDENTICAL to today.
  - Full CTest green NUMSCI ON and OFF (S4-f assertions absent with NUMSCI off, like
    S2/S3/S4-c/S4-d/S4-e). No OCCT linked; no tolerance weakened.
- **Sim native-vs-OCCT (booted simulator).** Model on `scripts/run-sim-native-ssi-seeding.sh`
  (recall vs OCCT `GeomAPI_IntSS` branch count) + `scripts/run-sim-native-ssi-marching.sh`
  (loop-closure / self-intersection), or a new `scripts/run-sim-native-ssi-s4f.sh` +
  `tests/sim/native_ssi_completeness_parity.mm`:
  - Add fixtures A–D. Assert the MEASURED recall win vs OCCT: `recall_default < recall_critic ≤ 1`
    on the small-loop fixture; the many-loop fixture's recall RISES toward OCCT's branch count.
  - Every recovered / traced native node lies on the OCCT locus ≤ `onCurveTol`
    (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces ≤ `onSurfTol`.
  - The false-close curve now matches the OCCT curve END-TO-END (arc / endpoints within tol); the
    self-crossing is detected on the OCCT locus at the OCCT self-intersection point ≤ tol.
  - Report per-fixture the recall FLOOR + the RESIDUAL acknowledgement (a smaller loop can still
    exist — NOT a completeness proof). Confirm the transversal + S4-c + S4-d + S4-e controls are
    UNCHANGED. Run via `xcrun simctl spawn <booted udid>`; `xcrun simctl list devices booted`.

## Decisions

- **Generalize the tangent-continuity gate to ALL closures (necessary-condition tightening).** The
  gate already exists (`closeAligned`) but only post-graze; making it apply to every close turns
  proximity from a sufficient test into a necessary-AND-tangent-continuous test. Because it can
  only REFUSE a close (never make one), a truly-closing curve is unaffected and a false-close is
  prevented — the crux distinction, enforced by construction. The 5 transversal pairs pin it: they
  close because they return tangent-continuously.
- **A self-intersection is ONE arm crossing ITSELF, distinct from an S4-d locus branch.** At a
  curve self-crossing both surfaces are TRANSVERSAL (`‖n₁×n₂‖` healthy) and no new arms emanate;
  the witness is a NODE-HISTORY crossing (proximity + transverse tangent), NOT the S4-d sine
  collapse + locus flip. Reporting it as data (not a stop) keeps the arm intact and makes a false
  positive cheap (a spurious count, not a wrong curve).
- **The critic RAISES the floor, loop-until-dry, bounded — it does NOT prove completeness.** Re-
  seeding only the UNCOVERED regions at finer resolution recovers loops the fixed floor missed;
  stopping after K dry rounds (or the cost cap) bounds the cost; DISCARDING any candidate that does
  not refine on both surfaces keeps every recovered loop honest. The residual (a loop below
  `criticFloorFrac` can still exist) is ALWAYS reported — completeness is MEASURED + asymptotic,
  never a guarantee. This is the cardinal honesty decision of S4-f, baked into the report fields.
- **Recall is a MEASURED figure vs OCCT, never asserted 1.0 blindly.** A fixture's recall `→ 1` is
  a claim about THAT fixture at THAT floor; the many-loop fixture reports a measured RISE with the
  residual, not totality. The sim gate compares to OCCT's branch count and reports the floor.
- **`completenessCritic` + `enableSelfIntersection` master switches (default OFF).** The critic and
  the self-intersection guard are behind default-off switches (off → the current fixed-resolution
  seed + S3 trace, byte-identical) plus hard cost caps (`criticMaxRounds` / `criticMaxCandidates`);
  the true-return closure reduces to the current result on truly-closing curves. A caller opts in;
  no existing trace changes.
- **Additive, ABI-stable, tessellator-untouched.** New `TraceSet` / `WLine` self-intersection +
  critic fields, `RecallReport` floor fields, `SeedOptions` / `MarchOptions` knobs (sentinel-
  resolved), and the new `completeness_critic.h`; no `cc_*` change; `src/native/tessellate`
  untouched; the S4-f code under `CYBERCAD_HAS_NUMSCI` like S2/S3/S4-c/S4-d/S4-e.

## Risks / Trade-offs

- **True-return closure fails to close a genuine loop (noisy return tangent).** Mitigation: same
  proximity radius as today + a GENEROUS block cosine (`closureTangentCos = -0.5`) — only a
  near-antiparallel pass-through is blocked; a real loop returns nearly parallel and still closes.
  Pinned by the 5 transversal pairs + every existing loop-closing control staying byte-identical;
  if the gate ever changed a control result the gate is wrong and the control catches it. Accepted.
- **Self-intersection guard false-fires on a tight non-crossing bend.** Mitigation: require BOTH
  proximity AND a transverse (non-continuation, non-retrace) tangent; report it as data (it does
  not stop the march), so a false positive is a spurious count, not a wrong curve; verified against
  the analytic self-crossing fixture. Accepted.
- **Critic cost-explodes chasing phantom loops.** Mitigation: loop-until-dry (K dry rounds) + the
  hard cost cap (`criticMaxRounds` / `criticMaxCandidates`); DISCARD any candidate that does not
  refine on both surfaces; the floor reached is reported honestly (dry vs cap). Accepted.
- **Overclaiming completeness.** Mitigation BY CONSTRUCTION: recall is a MEASURED figure with the
  residual ALWAYS acknowledged (`completenessResidual == true`), never a blind 1.0; a fixture's
  recall `→ 1` is scoped to that fixture at that floor. This is the cardinal S4-f risk and it is
  closed by never asserting a guarantee. Accepted.
- **Transversal / S4-c / S4-d / S4-e regression.** Mitigation: the closure reduces to the current
  proximity result on truly-closing curves; the critic + self-intersection guard are default-off;
  the S3 corrector / deflection controller, the S4-c crossing driver, the S4-d branch handler, and
  the S4-e chart machinery are bit-identical; the 5 transversal pairs + the S4-c graze + the S4-d
  Steinmetz + the S4-e pole/apex are pinned green. Whatever the critic cannot recover is reported
  as a measured recall `< 1` with the floor; no loop, closure, or seed is faked, hand-tuned, or
  weakened to pass. Accepted.
