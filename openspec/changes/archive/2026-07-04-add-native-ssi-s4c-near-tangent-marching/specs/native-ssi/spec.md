# native-ssi

Add the FIRST SSI Stage **S4-c near-tangent MARCHING-THROUGH** slice
(`openspec/SSI-ROADMAP.md` S4 — the moat) on top of the shipped S1 analytic / S2 seeding /
S3 marching stack and the S4-a/S4-b classification layers:

- **S4-c** — a native, OCCT-free capability that MARCHES THROUGH a near-tangent region and
  emits the FULL intersection curve **only when the curve genuinely continues on the same
  branch** (the S4-b classification is `NearTangentTransversal` and the region is a
  single-branch graze), via a robust FIXED-PLANE-CUT constrained corrector (well-posed as
  `‖n₁ × n₂‖ → 0`), a curvature-aware predictor, and low-sine step control.

This change is **additive** to the S3 marcher. Outside the low-sine band the transversal
corrector, accept test, and deflection controller are UNCHANGED. A GENUINE tangency
(`TangentPoint` / `TangentCurve`), a branch crossing (S4-d), an `Undecided` local jet, or a
corrector that will not converge at the minimum step STILL STOPS + classifies + defers →
OCCT. The tracer NEVER fabricates a point past a degeneracy and NEVER weakens a tolerance;
a curve that still truncates is an honestly reported gap. SSI is INTERNAL: **no `cc_*` ABI
change**; `src/native/**` stays OCCT-free; the S4-c parts are compiled under
`CYBERCAD_HAS_NUMSCI` (like S2/S3). Branch points (S4-d), singularities (S4-e), and
self-intersection (S4-f) remain OUT OF SCOPE.

## ADDED Requirements

### Requirement: Native near-tangent marching-through of a continuing curve (S4-c)

The kernel SHALL provide a native, **OCCT-free** near-tangent marching capability in
`cybercad::native::ssi` that, when the S3 marcher reaches a near-tangent region
(‖n₁ × n₂‖ below the near-tangent gate) whose intersection curve GENUINELY CONTINUES on the
same branch, MARCHES THROUGH the region and emits the FULL curve rather than truncating.
The capability SHALL use a robust corrector that stays well-posed as ‖n₁ × n₂‖ → 0 by
pinning the new node to a FIXED PLANE perpendicular to the LAST-GOOD intersection tangent at
arc-distance equal to the step (a constrained residual-minimization over the native-numerics
substrate — minimizing the on-both-surfaces gap subject to the fixed-plane cut — NOT the
along-local-tangent Newton corrector that degenerates as the local tangent ill-conditions).
It SHALL seed that corrector with a curvature-aware predictor that bends the first-order
guess by the discrete curvature of the last two accepted nodes, and SHALL cross the
minimum-clearance region with a reduced step, resuming the normal deflection-driven step
once ‖n₁ × n₂‖ recovers. Outside the low-sine band the transversal corrector, accept test,
and deflection controller SHALL be unchanged, so every transversal march is traced as
before. The near-tangent regions the marcher successfully crosses SHALL be reported
(a per-branch and per-`TraceSet` crossed count) and SHALL NOT be counted in
`nearTangentGaps` (they are completed arcs). `src/native/**` SHALL NOT link OCCT; no `cc_*`
entry point, signature, or POD struct SHALL be added or changed; the marching entry points
SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a currently-truncating near-tangent transversal curve is now fully traced

- GIVEN a surface pair whose intersection curve grazes a near-tangent region but genuinely
  continues on the same branch (e.g. two equal cylinders whose axes cross, whose S3 trace
  today stops at the grazing region with `TraceStatus::NearTangent`,
  `stopReason == NearTangentTransversal`, and `nearTangentGaps == 1`), with
  `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-c near-tangent marcher traces the pair
- THEN it SHALL MARCH THROUGH the near-tangent region using the fixed-plane-cut corrector
  and the curvature-aware predictor, emit the FULL intersection curve, report the crossed
  region in the crossed count, and yield `nearTangentGaps == 0` for that curve
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the transversal march outside the low-sine band is unchanged

- GIVEN a transversal surface pair whose intersection never enters the low-sine band
- WHEN the S4-c marcher traces it
- THEN the traced curve SHALL be identical to the S3 result (same nodes, same status, same
  counts) — the corrector, accept test, and deflection controller SHALL be unchanged outside
  the low-sine band

### Requirement: The crossable gate defers genuine tangency and branch crossings (S4-c honesty)

The S4-c marcher SHALL attempt to march through a near-tangent region ONLY when the S4-b
tangent-contact classification at the stall is `NearTangentTransversal` AND the local
configuration is a SINGLE-BRANCH graze (the curve continues on the same branch), and SHALL
VERIFY any crossing before accepting it. When the S4-b classification is `TangentPoint` (an
isolated 0-dimensional genuine tangency where the curve does NOT continue), `TangentCurve`
(the surfaces are tangent along a whole seam), or `Undecided` (the local jet is within the
curvature-noise band), OR when the stall is a BRANCH CROSSING (multiple curve branches
meeting — S4-d), OR when the fixed-plane corrector cannot converge on both surfaces at the
minimum step, the marcher SHALL STOP, record the typed `TangentContact` stop reason, count
the region in `nearTangentGaps`, and DEFER it (→ OCCT) — exactly as the S3/S4-b tracer does
today. A crossing arc SHALL be accepted only if every node on it lies on BOTH surfaces
within `onSurfTol` and advances monotonically along the last-good tangent onto a
far-side tangent consistent with it; a crossing that fails this verification SHALL be
DISCARDED and the march SHALL truncate at the band entry. The marcher SHALL NEVER fabricate
an intersection-curve point past a degeneracy and SHALL NEVER weaken a tolerance to force a
crossing; a region that cannot be robustly crossed SHALL remain an honestly reported gap.

#### Scenario: a genuine tangency still stops and is classified, never crossed

- GIVEN a genuinely tangent surface pair — two spheres at centre distance `d = R₁ + R₂`
  (externally tangent, an isolated `TangentPoint`), or a sphere tangent to a cylinder along a
  circle (a `TangentCurve`)
- WHEN the S4-c marcher encounters the tangency
- THEN it SHALL STOP, classify the contact (`TangentPoint` / `TangentCurve`), count it in
  the deferred / near-tangent-gap tally, and NOT march through it
- AND it SHALL NOT fabricate any intersection-curve point across the tangency

#### Scenario: a branch crossing is deferred to S4-d, not crossed

- GIVEN a near-tangent stall that is a BRANCH CROSSING (multiple intersection-curve branches
  meeting at the point — e.g. the exact saddle where two equal crossing cylinders' branches
  touch), so more than one continuing branch is present
- WHEN the S4-c crossable gate evaluates it
- THEN it SHALL STOP and DEFER the region (S4-d owns branch topology), counting it in
  `nearTangentGaps` — it SHALL NOT march a fabricated single branch through the crossing

#### Scenario: an unverifiable or non-convergent crossing truncates honestly

- GIVEN a near-tangent region classified `NearTangentTransversal` where either the
  fixed-plane corrector cannot converge on both surfaces at the minimum step (a deep,
  near-coincident band) or a tentative crossing arc fails the on-both-surfaces / monotone
  verification
- WHEN the S4-c marcher attempts the crossing
- THEN it SHALL DISCARD any tentative arc, STOP at the band entry, and count the region in
  `nearTangentGaps` (deferred → OCCT) — it SHALL NOT emit a partially fabricated arc and
  SHALL NOT weaken a tolerance to make the crossing pass
