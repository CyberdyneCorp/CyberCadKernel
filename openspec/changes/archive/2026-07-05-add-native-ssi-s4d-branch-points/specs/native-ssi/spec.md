# native-ssi

Add the FIRST SSI Stage **S4-d BRANCH-POINT** slice (`openspec/SSI-ROADMAP.md` S4 — the
moat, the hardest SSI piece) on top of the shipped S1 analytic / S2 seeding / S3 marching
stack, the S4-a/S4-b classification layers, and the S4-c near-tangent marching-through slice:

- **S4-d** — a native, OCCT-free capability that, at a detected SELF-CROSSING of the
  intersection LOCUS (multiple real curve arms meeting at one point — the Steinmetz-family
  transversal self-crossing S3+S4-c currently DEFER), LOCALIZES the branch point, ENUMERATES
  the outgoing arms from the local second-order structure (the tangent cone — REAL roots
  only), ROUTES the normal march down each arm, and ASSEMBLES the multi-arm curve into the
  `TraceSet` with a reported branch-point count and arm connectivity.

This change is **additive** to the S3/S4-c marcher. The transversal S3 trace and the S4-c
crossable-graze crossing are UNCHANGED — the branch machinery engages ONLY at a detected
branch point (the S4-c collapse + flip detection). An ISOLATED `TangentPoint` (S4-b:
sign-definite relative second form — the curve ENDS there) STILL ENDS and NEVER sprouts
fabricated arms. A branch point that cannot be robustly LOCALIZED (its point does not
re-project on both surfaces within tolerance), ENUMERATED (the tangent-cone quadratic has no
two distinct real roots), or ROUTED (an arm's first step will not verify on both surfaces)
STILL STOPS + classifies + defers → OCCT. The tracer NEVER fabricates an arm or a point past
a degeneracy and NEVER weakens a tolerance; a branch it cannot resolve is an honestly reported
gap. SSI is INTERNAL: **no `cc_*` ABI change**; `src/native/**` stays OCCT-free; the S4-d
parts are compiled under `CYBERCAD_HAS_NUMSCI` (like S2/S3/S4-c). General / freeform branch
points, cusps / degenerate branches, singularities (S4-e), and self-intersection completeness
(S4-f) remain OUT OF SCOPE.

## ADDED Requirements

### Requirement: Native branch-point localization, arm enumeration, and routing (S4-d)

The kernel SHALL provide a native, **OCCT-free** branch-point capability in
`cybercad::native::ssi` that, when the S3/S4-c marcher reaches a genuine SELF-CROSSING of the
intersection locus (a point where multiple real intersection-curve arms meet — detected by the
existing S4-c transversality-sine COLLAPSE and raw-tangent FLIP that today force a defer),
LOCALIZES the branch point, ENUMERATES the outgoing arms, ROUTES the march down each arm, and
ASSEMBLES the multi-arm curve — rather than truncating at the crossing. The capability SHALL
LOCALIZE the branch point B as the on-both-surfaces point where the transversality sine
‖n₁ × n₂‖ reaches its minimum (≈ 0) along the approach, with B on BOTH surfaces within
`onSurfTol`. It SHALL ENUMERATE the outgoing arm directions as the REAL, distinct roots of the
tangent-cone quadratic formed from the local second-order structure of the two surfaces at B
(the relative second fundamental form restricted to the shared tangent plane) — a transversal
self-crossing (indefinite form) yielding two distinct tangent lines and hence up to four
outgoing rays; it SHALL return ONLY the real distinct directions and, when the quadratic has NO
two distinct real roots, it SHALL enumerate NO arms. It SHALL ROUTE each enumerated arm by
stepping off B a small distance along the ray, re-projecting onto both surfaces with the
fixed-plane corrector (verified within `onSurfTol`), then continuing the normal march to
termination; and it SHALL DEDUP an arm that retraces an already-traced arm and CONNECT the arms
meeting at the same branch point into a branch-point node in the assembled result. The
branch points the marcher localizes and routes SHALL be reported (a per-`TraceSet` branch-point
count and the localized point with its connected arm ids) and SHALL NOT be counted in
`nearTangentGaps`. Outside a detected branch point the transversal S3 trace and the S4-c
crossable-graze crossing SHALL be unchanged. `src/native/**` SHALL NOT link OCCT; no `cc_*`
entry point, signature, or POD struct SHALL be added or changed; the marching entry points
SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the Steinmetz self-crossing is now fully traced

- GIVEN two equal-radius cylinders (R = 1) whose axes cross orthogonally at the origin (the
  Steinmetz bicylinder — its intersection is two ellipses that CROSS each other at two branch
  points), whose S3+S4-c trace today stops at the branch point with `TraceStatus::NearTangent`,
  `stopReason == NearTangentTransversal`, `tracedBranches == 0`, and `nearTangentGaps == 1`,
  with `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-d marcher traces the pair
- THEN it SHALL LOCALIZE both branch points (each on BOTH cylinders within `onSurfTol`, at a
  near-zero transversality sine), ENUMERATE the outgoing arms from the tangent cone, ROUTE the
  march down each arm, and ASSEMBLE the two crossing ellipses — reporting `branchPoints == 2`
  and yielding `nearTangentGaps == 0` for the traced structure
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the transversal march and the S4-c graze are unchanged

- GIVEN a transversal surface pair whose intersection never reaches a branch point, and a
  surface pair whose intersection is a `NearTangentTransversal` single-branch graze the S4-c
  slice marches through
- WHEN the S4-d marcher traces them
- THEN the transversal traced curve SHALL be identical to the S3 result (same nodes, same
  status, same counts) and the graze SHALL still be MARCHED THROUGH by the S4-c crossing
  (reported in the crossed count, `branchPoints == 0` for both) — the branch machinery SHALL
  engage ONLY at a detected branch point

### Requirement: Branch-point honesty — isolated tangents end and unresolved branches defer (S4-d)

The S4-d marcher SHALL route arms out of a stall ONLY when it is a genuine branch point — a
self-crossing of the intersection locus whose tangent-cone quadratic has TWO DISTINCT REAL
roots (an indefinite relative second fundamental form). When the S4-b tangent-contact
classification at the stall is `TangentPoint` (an isolated 0-dimensional genuine tangency where
the curve does NOT continue — a sign-definite relative second form, equivalently no two
distinct real tangent-cone roots), the marcher SHALL let the curve END there and SHALL NOT
fabricate any outgoing arm. When the classification is `TangentCurve` or `Undecided`, when the
tangent-cone quadratic has a double root (a cusp / degenerate branch, out of scope), when the
branch point cannot be localized on both surfaces within `onSurfTol`, or when a would-be arm's
first step cannot be verified on both surfaces within `onSurfTol`, the marcher SHALL STOP,
record the typed `TangentContact` stop reason, count the region in `nearTangentGaps`, and DEFER
it (→ OCCT) — exactly as the S4-c tracer does today — or DROP the unverifiable arm. The marcher
SHALL NEVER fabricate an arm or an intersection-curve point past a degeneracy and SHALL NEVER
weaken a tolerance to force a branch; a branch that cannot be robustly resolved SHALL remain an
honestly reported gap.

#### Scenario: an isolated tangent point still ends, never sprouting arms

- GIVEN a genuinely tangent surface pair — two spheres at centre distance `d = R₁ + R₂`
  (externally tangent, an isolated `TangentPoint` with a sign-definite relative second form)
- WHEN the S4-d marcher encounters the tangency
- THEN it SHALL let the curve END at the isolated contact, classify it (`TangentPoint`), and
  enumerate NO outgoing arms (`branchPoints == 0`)
- AND it SHALL NOT fabricate any intersection-curve arm or point across the tangency

#### Scenario: a branch point that cannot be robustly resolved defers honestly

- GIVEN a suspected branch point where either the localization does not re-project onto both
  surfaces within `onSurfTol`, the tangent-cone quadratic has no two distinct real roots (a
  definite form or a double root), or a would-be arm's first step off the branch point fails
  the on-both-surfaces verification
- WHEN the S4-d marcher attempts to localize / enumerate / route it
- THEN it SHALL DROP any unverifiable arm, STOP at the stall, record the typed
  `TangentContact` stop reason, and count the region in `nearTangentGaps` (deferred → OCCT) —
  it SHALL NOT fabricate an arm or a point and SHALL NOT weaken a tolerance to force the branch
