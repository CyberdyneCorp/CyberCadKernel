# native-ssi

Add the FIRST SSI Stage **S4-f COMPLETENESS + LOOP-ROBUSTNESS** slice (`openspec/SSI-ROADMAP.md`
S4 — the moat, degeneracy robustness) on top of the shipped S1 analytic / S2 seeding / S3 marching
stack, the S4-a/S4-b classification layers, the S4-c near-tangent marching-through slice, the S4-d
branch-point slice, and the S4-e chart-singularity slice:

- **S4-f** — a native, OCCT-free capability with two orthogonal parts that HARDEN the CORRECTNESS
  and COMPLETENESS of curves the tracer already produces (it adds no new geometry capability):
  - **Part 1 (robust closure).** Replace the pure-PROXIMITY loop-closure test with a TRUE-RETURN +
    TANGENT-CONTINUOUS test (a loop closes only on a verified return to the seed heading the way it
    left), and add a SELF-INTERSECTION guard (the arm crossing an EARLIER non-seed node
    transversally is a typed self-intersection — NOT a loop close, NOT an S4-d locus branch). So a
    curve that merely passes NEAR its seed the other way is NOT false-closed, and a genuine
    self-crossing is DETECTED and traced THROUGH rather than skipped.
  - **Part 2 (completeness critic).** Add a BOUNDED ADAPTIVE re-seed that, after the initial seed +
    trace, re-subdivides the param regions NOT covered by a traced curve at finer resolution,
    LOOP-UNTIL-DRY (K consecutive rounds with no new branch), to RECOVER small loops the fixed 1/32
    seeding floor silently missed — bounded by a cost cap and HONEST about the floor it reached.

This change is **additive** to the S3/S4-c/S4-d/S4-e marcher. **Completeness is MEASURED +
ASYMPTOTIC, never a proof:** below any fixed subdivision resolution a smaller loop can always be
missed, so the capability claims MEASURED recall wins on concrete fixtures vs OCCT and RAISES the
completeness floor, REPORTING the residual (a loop smaller than the finest re-seed round can still
exist) — it does NOT claim a completeness guarantee, and recall is never asserted a blind 1.0.
**S4-f DE-RISKS (does not unblock) curved blends (#6) and wrap-emboss (#7)** — their intersection
seams are exactly these small-loop / self-intersecting / many-loop patterns, so the SSI-curve
completeness they depend on becomes measurably more reliable, but #6/#7 still need their wider
S5/S6/S7 assemblers. The tracer NEVER fabricates a loop, a closure, or a seed: a loop closes only
on a verified true return; a re-seeded branch is kept only if it refines to a real on-both-surfaces
seed/curve; anything unresolved is reported honestly (measured recall `< 1`), never a faked branch.
The 5 transversal S3 pairs (`nt = 0`, bit-identical), the S4-c crossable graze, the S4-d Steinmetz
branch trace, and the S4-e pole/apex crossings are UNCHANGED — the true-return closure reduces to
the current proximity result on curves that truly close, and the critic + self-intersection guard
are behind default-off gates. SSI is INTERNAL: **no `cc_*` ABI change**; `src/native/**` stays
OCCT-free; the S4-f parts are compiled under `CYBERCAD_HAS_NUMSCI` (like S2/S3/S4-c/S4-d/S4-e). A
completeness guarantee/proof, global topology repair / self-intersection resolution, and completing
#6/#7 remain OUT OF SCOPE.

## ADDED Requirements

### Requirement: Native robust loop closure and self-intersection detection (S4-f part 1)

The kernel SHALL provide, in `cybercad::native::ssi`, a native, **OCCT-free** robust-closure and
self-intersection capability. Loop closure SHALL be a TRUE-RETURN test: a march SHALL declare a
closed loop ONLY when it returns within the loop-closure proximity radius of the SEED **AND** its
current forward tangent is CONTINUOUS with the seed's outgoing tangent (heading the way it left).
A march that returns NEAR the seed but heading substantially the OTHER way (a near-antiparallel
pass-through) SHALL NOT be declared closed; it SHALL continue to its true termination. This
true-return test SHALL reduce to the current proximity result for any curve that truly closes (it
tightens a necessary condition — it can only REFUSE a close the proximity test would have made,
never MAKE one), so the transversal S3 traces and every existing loop-closing case SHALL be
unchanged. The capability SHALL additionally DETECT a genuine SELF-INTERSECTION — the traced arm
crossing an EARLIER NON-SEED node of its own history within a tolerance-scaled radius with a
TRANSVERSE (non-continuation, non-retrace) tangent — and SHALL record it as a typed
self-intersection with its point and both surface parameters, WITHOUT declaring a loop close there
and WITHOUT stopping the march (the arm SHALL continue to its true termination). A self-intersection
SHALL be DISTINCT from an S4-d locus branch point: at a curve self-crossing both surfaces are
transversal (the pair transversality sine does not collapse) and no new arms emanate, so the
self-intersection witness (a node-history crossing) SHALL be independent of the S4-c pair sine and
the S4-d locus tangent flip. An arm merely running back over itself (a retrace, tangents nearly
parallel or antiparallel) SHALL NOT be reported as a self-intersection. The self-intersection guard
SHALL be behind a default-off switch; with it off the trace SHALL be byte-identical to the current
S3 result. The self-intersections detected SHALL be reported (a per-`WLine` count and typed list
and a per-`TraceSet` total). `src/native/**` SHALL NOT link OCCT; no `cc_*` entry point, signature,
or POD struct SHALL be added or changed; the marching entry points SHALL remain compiled under
`CYBERCAD_HAS_NUMSCI`.

#### Scenario: a curve passing near its seed the other way is not false-closed

- GIVEN an open or longer intersection curve whose S3 trace passes within the loop-closure
  proximity radius of the seed while heading substantially the OTHER way (a near-antiparallel
  pass-through), which the current pure-proximity test declares `Closed` and truncates at the
  near-pass, with `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-f marcher traces the pair
- THEN it SHALL NOT declare the curve closed at the near-pass (the return tangent is not continuous
  with the seed's outgoing tangent) and SHALL continue the march to the curve's TRUE termination
- AND the traced curve SHALL match the analytic ground truth end-to-end (arc length / endpoints
  within tolerance), never fabricating a closure

#### Scenario: a genuine self-intersection is detected and traced through

- GIVEN a single-arm intersection curve that genuinely self-crosses (one arm passing through the
  same 3D point twice, headed differently, while BOTH surfaces stay transversal — distinct from an
  S4-d branch where the intersection locus branches into multiple arms), with the self-intersection
  guard enabled
- WHEN the S4-f marcher traces the pair
- THEN it SHALL DETECT the self-crossing as a typed self-intersection at the crossing point (on
  both surfaces within `onSurfTol`), report at least one self-intersection, and continue the arm
  THROUGH the crossing to its true termination — it SHALL NOT declare a loop close at the crossing
  and SHALL NOT step past it unrecorded
- AND the self-intersection SHALL be reported as distinct from an S4-d branch point (the trace of
  this fixture yields zero localized branch points)

#### Scenario: the transversal closure, the S4-c graze, the S4-d branch trace, and the S4-e crossings are unchanged

- GIVEN the 5 transversal surface pairs whose intersection curves close by returning to the seed
  tangent-continuously, the S4-c `NearTangentTransversal` graze, the Steinmetz bicylinder whose
  locus self-crosses at two S4-d branch points, and the sphere-pole / cone-apex pairs the S4-e
  slice crosses
- WHEN the S4-f marcher traces them (with the self-intersection guard off)
- THEN each transversal curve SHALL still close (identical nodes, status, and counts to the S3
  result), the S4-c graze SHALL still be marched through, the S4-d Steinmetz SHALL still be
  assembled (two branch points), and the S4-e pole + apex SHALL still be crossed — each with zero
  self-intersections reported, because the true-return closure reduces to the current result on
  truly-closing curves and the self-intersection guard is off

### Requirement: Native bounded adaptive completeness-critic re-seed with an honestly reported recall floor (S4-f part 2)

The kernel SHALL provide, in `cybercad::native::ssi`, a native, **OCCT-free** completeness-critic
capability that, after the initial fixed-resolution seed and trace, RECOVERS small intersection
loops the fixed subdivision floor (default leaf fraction 1/32) silently missed. The critic SHALL,
when enabled, compute the param regions NOT covered by any traced curve, RE-SUBDIVIDE those
uncovered regions at a FINER resolution than the previous round, refine each new candidate to a
point on BOTH surfaces at the SAME `onSurfTol` (a candidate that does not refine to an
on-both-surfaces point, or that is near-tangent, SHALL be DISCARDED — never a fabricated seed),
DEDUP the new seeds against all kept curves, and TRACE each genuinely new seed. The critic SHALL
REPEAT this re-seed round LOOP-UNTIL-DRY — stopping after a configured number of CONSECUTIVE rounds
that recover NO new branch — OR when a hard cost cap (a maximum number of rounds and a maximum total
number of re-seed candidate regions) is reached, whichever occurs first. Every recovered loop SHALL
be a VERIFIED on-both-surfaces seed that refined to a real curve; the critic SHALL NEVER fabricate a
branch. The critic SHALL be behind a default-off switch; with it off the seeding and trace SHALL be
byte-identical to the current fixed-resolution result. The capability SHALL report the completeness
FLOOR it reached — the finest re-seed leaf fraction, the number of rounds run, and whether it
stopped DRY (the configured dry rounds elapsed with no new branch) or on the COST CAP — together
with the MEASURED recall (native branches carrying at least one seed ÷ the true transversal branch
count), which SHALL NEVER be asserted a blind 1.0. The report SHALL ALWAYS acknowledge the RESIDUAL:
a loop smaller than the finest re-seed round can still exist, so completeness is MEASURED and
ASYMPTOTIC, not a guarantee. `src/native/**` SHALL NOT link OCCT; no `cc_*` entry point, signature,
or POD struct SHALL be added or changed; the critic SHALL be compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a small loop below the fixed floor is recovered

- GIVEN a surface pair whose intersection has a small loop lying entirely inside one default (1/32)
  leaf cell, so the fixed-resolution seeder produces NO seed for it and the measured recall is below
  1 at the default resolution, with `CYBERCAD_HAS_NUMSCI` built
- WHEN the completeness critic is enabled and the pair is traced
- THEN the critic SHALL re-seed the uncovered region at a finer resolution, refine a verified
  on-both-surfaces seed for the small loop, trace it, and recover the loop — raising the measured
  recall to 1 ON THIS FIXTURE at the reached floor, with at least one recovered loop reported and
  the reached leaf fraction finer than 1/32
- AND the report SHALL STILL acknowledge the residual (a loop smaller than the reached floor can
  still exist) — the recall figure is a MEASURED win on this fixture at this floor, not a
  completeness proof

#### Scenario: many small loops measurably raise recall vs OCCT, with the floor and residual reported

- GIVEN an adversarial surface pair with several disjoint small loops (the wrap-emboss / blend seam
  pattern), most of them below the fixed seeding floor, so the default measured recall is well
  below 1
- WHEN the completeness critic is enabled and the pair is traced
- THEN the critic SHALL MEASURABLY raise the recall over the default (recovering loops down to the
  reached floor via the loop-until-dry re-seed), and SHALL report the reached floor, the number of
  rounds, whether it stopped dry or on the cost cap, and the measured recall against the OCCT branch
  count
- AND the report SHALL acknowledge the residual (a loop below the reached floor can still be
  missed) — it SHALL NOT claim total completeness, and no branch SHALL be fabricated

#### Scenario: a re-seed candidate that does not verify is discarded, never fabricated

- GIVEN a completeness-critic re-seed round in an uncovered region where a candidate fails to refine
  to a point on both surfaces within `onSurfTol`, or is near-tangent
- WHEN the critic processes the candidate
- THEN it SHALL DISCARD the candidate and SHALL NOT emit a seed or a branch for it — the recovered
  set contains only verified on-both-surfaces seeds that traced to real curves
- AND a loop the critic cannot recover within its cost cap SHALL be reported as a measured recall
  below 1 with the residual acknowledged, never a faked branch and never a weakened tolerance
