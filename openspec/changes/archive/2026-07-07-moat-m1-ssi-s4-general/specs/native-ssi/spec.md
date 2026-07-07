# native-ssi

Extend the SSI Stage **S4-d BRANCH-POINT** capability (`openspec/MOAT-ROADMAP.md` M1 — general/freeform
SSI S4 robustness; `openspec/SSI-ROADMAP.md` S4 — the degeneracy moat) from the ANALYTIC Steinmetz
bicylinder that the archived `add-native-ssi-s4d-branch-points` landed, to the GENERAL / FREEFORM case:

- **S4-d (freeform open-arm branch point)** — a native, OCCT-free extension that traces a GENERAL /
  FREEFORM branch point: a FREEFORM (B-spline / NURBS) surface pair whose intersection LOCUS
  self-crosses at a point B where the pair transversality sine `‖n_A × n_B‖ → 0` with an INDEFINITE
  relative second fundamental form `H = II_A − II_B` (a saddle contact — the freeform analog of the
  Steinmetz self-crossing), but where, on a FINITE patch, the arms radiate OPEN to the domain boundary
  (branch-to-boundary) rather than closing branch-to-branch. The marcher CAPTURES the branch stall via
  the SAME scale-free sine-collapse signature, LOCALIZES B on both surfaces, ENUMERATES the outgoing
  arms from the tangent-cone quadratic of the (already freeform-accurate central-difference) relative
  second form, ROUTES each arm with the existing normal-free point-based corrector (verified on both
  surfaces within tolerance), and — the additive piece — RECLASSIFIES each OPEN arm (one end on the
  localized branch point, the other a clean domain boundary) as a resolved `BranchArc`, driving
  `nearTangentGaps` to 0 for the assembled X-crossing.

This change is **additive** to the S3/S4-c/S4-d/S4-e marcher. The analytic Steinmetz branch trace, the
transversal S3 trace, the S4-c crossable-graze crossing, and the S4-e chart-singularity crossings are
UNCHANGED — the open-arm reclassification is a STRICT GENERALISATION of the existing both-ends-on-branch
rule (it reduces to that rule when both ends are branch nodes, so the Steinmetz branch-to-branch network
is bit-identical), and it is guarded per-endpoint (two new `WLine` flags recording which end stalled at
a near-tangency) so a genuine still-open tangency is never reclassified. The relative-second-form tangent
cone is reused verbatim (its central difference is already O(δ²)-accurate on freeform — no bias-
cancellation is added). A FREEFORM contact whose tangent cone has NO two distinct real roots (a DEFINITE
contact — an isolated tangent point) lets the curve END with NO arms; a freeform branch that cannot be
localized on both surfaces, whose enumerated arm's first step will not re-project on both surfaces within
tolerance, that is a cusp (double root) / higher-multiplicity junction, or whose stall end is not on a
localized branch point STILL STOPS + defers → OCCT. The tracer NEVER fabricates an arm or a point past a
branch and NEVER weakens a tolerance; a branch it cannot resolve is an honestly reported gap. SSI is
INTERNAL: **no `cc_*` ABI change**; `src/native/**` stays OCCT-free; the S4-d-g parts are compiled under
`CYBERCAD_HAS_NUMSCI` (like S2/S3/S4-c/S4-d/S4-e). Non-transversal (definite) freeform contacts (which
end with no arms), cusps and higher-multiplicity junctions, both-operand-freeform saddle ∩ saddle
crossings that do not verify, general near-tangent breadth (S4-c) and coincident/overlapping freeform
surfaces (S4-a), and a full brep freeform-branch solid through the boolean pipeline remain OUT OF SCOPE.

## ADDED Requirements

### Requirement: Native general/freeform open-arm branch-point localization, arm routing, and assembly (S4-d general)

The kernel SHALL extend the native, **OCCT-free** branch-point capability in `cybercad::native::ssi`
to a GENERAL / FREEFORM branch point — a FREEFORM (B-spline / NURBS) surface pair whose intersection
LOCUS self-crosses at a point B where the pair transversality sine `‖n_A × n_B‖` reaches its minimum
(≈ 0) and the relative second fundamental form `H = II_A − II_B` is INDEFINITE (a saddle contact) — and,
on a FINITE freeform patch, whose arms radiate OPEN to the patch boundary (branch-to-boundary) rather
than closing between two branch points. When the S3/S4-c marcher reaches such a freeform self-crossing
(detected by the SAME scale-free transversality-sine COLLAPSE and band-minimum witness that captures the
analytic Steinmetz stall — a `branchSignature` hand-off gated on `enableBranchPoints`, independent of
any quadric assumption), the marcher SHALL LOCALIZE B on BOTH surfaces within `onSurfTol` (the existing
sine-minimization along the approach followed by a full re-projection onto both surfaces), ENUMERATE the
outgoing arm directions as the REAL distinct roots of the tangent-cone quadratic of `H` (computed with
the existing central-difference relative second form, which is already accurate on a freeform patch),
ROUTE each enumerated arm (step off B, re-project with the existing normal-free point-based corrector,
verify on both surfaces within `onSurfTol`, then continue the normal march to termination), and ASSEMBLE
the multi-arm freeform curve into branch-point-connected arcs — rather than truncating at the crossing.
To resolve the OPEN-ARM topology, the marcher SHALL reclassify a near-tangent-terminated arc out of the
`nearTangentGaps` count (as a `BranchArc`) when EVERY end of the arc that stalled at a near-tangency sits
on a LOCALIZED branch point and at least one end does — the remaining end being a clean domain-boundary
exit — thereby covering BOTH the Steinmetz branch-to-branch closed network AND the freeform
branch-to-boundary open arm. The marcher SHALL record, per traced arc, WHICH end (front / back) stalled
at a near-tangency, and SHALL NOT reclassify an arc that has a near-tangent end which is NOT on a
localized branch point (a genuine still-open S4 gap). The freeform branch points the marcher localizes
and routes SHALL be reported in the existing per-`TraceSet` `branchPoints` count (with the localized
point and its connected arm ids), and the assembled arcs SHALL be counted in `tracedBranches` /
`openCurves`. Outside a detected branch point the transversal S3 trace, the S4-c crossable graze, and
the S4-e chart crossings SHALL be unchanged, and the reclassification rule SHALL reduce to the existing
both-ends-on-branch behaviour for the analytic Steinmetz network (bit-identical). `src/native/**` SHALL
NOT link OCCT; no `cc_*` entry point, signature, or POD struct SHALL be added or changed; the marching
entry points SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a freeform-saddle open X-crossing is now fully traced

- GIVEN a FREEFORM pair — a bicubic B-spline SADDLE patch (height `≈ 0.15·(x² − y²)`, `makeBSplineAdapter`)
  TANGENT to a plane placed THROUGH the B-spline saddle point (the surface value at the patch centre,
  ≈ z 0.2449 — NOT z = 0, where the two hyperbola branches are DISJOINT), whose intersection locus is an
  X-shaped self-crossing (two curves crossing at the saddle point B, `H = diag(κ, −κ)` indefinite) with
  four arms radiating OPEN to the patch boundary, whose S3+S4-c trace today stops at the crossing with
  `TraceStatus::NearTangent`, `branchPoints == 0`, `tracedBranches == 0`, and `nearTangentGaps == 1`,
  with `CYBERCAD_HAS_NUMSCI` built and `enableBranchPoints` on
- WHEN the S4-d marcher traces the pair
- THEN it SHALL LOCALIZE B on BOTH surfaces within `onSurfTol` (`branchPoints == 1`), enumerate the
  outgoing arms from the tangent-cone quadratic (two distinct real tangent lines ⇒ the outgoing rays),
  ROUTE each arm with the normal-free point-based corrector, and RECLASSIFY each open arm (one end on
  the localized branch, the other a domain boundary) as a resolved `BranchArc` — yielding
  `nearTangentGaps == 0`, `tracedBranches == 4`, and each traced arc a `BranchArc` with EXACTLY one end
  on the branch point
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`, and the localized branch
  node SHALL lie on both surfaces within `onSurfTol` at the saddle point

#### Scenario: the analytic Steinmetz branch trace and the transversal / S4-c / S4-e cases are unchanged

- GIVEN the analytic Steinmetz bicylinder (two equal orthogonal cylinders whose intersection self-crosses
  at two branch points the S4-d slice localizes and routes as a branch-to-branch closed network), a
  transversal surface pair whose intersection never reaches a branch point, a `NearTangentTransversal`
  single-branch graze the S4-c slice marches through, and the sphere-pole / cone-apex / freeform-pole
  chart crossings the S4-e slice steps across
- WHEN the S4-d marcher traces them with the generalised open-arm reclassification in place
- THEN the Steinmetz SHALL still be assembled with `branchPoints == 2` and `nearTangentGaps == 0` (its
  branch-to-branch arcs still reclassify — the generalised rule reduces to the existing
  both-ends-on-branch rule when both ends are branch nodes), the transversal traced curve SHALL be
  identical to the S3 result (same nodes, same status, same counts — branch points OFF leaves the new
  per-end flags unused and the polyline byte-identical), the graze SHALL still be MARCHED THROUGH by the
  S4-c crossing, and the chart singularities SHALL still be crossed by the S4-e machinery — the freeform
  open-arm reclassification SHALL engage ONLY on a near-tangent arc terminating on a detected, localized
  branch point

### Requirement: Freeform branch-point honesty — definite contacts end and unresolved arcs defer (S4-d general)

The S4-d marcher SHALL route arms out of a freeform stall ONLY when it is a genuine transversal branch
point — a self-crossing whose tangent-cone quadratic has TWO DISTINCT REAL roots (an indefinite relative
second form). When the quadratic has NO two distinct real roots — a DEFINITE relative second form (an
isolated `TangentPoint`, e.g. a B-spline bump tangent to a plane) or a DOUBLE root (a cusp) — the marcher
SHALL let the curve END there and SHALL enumerate NO outgoing arm. When the branch point cannot be
localized on both surfaces within `onSurfTol`, when an enumerated arm's first step cannot be re-projected
on both surfaces within `onSurfTol` (an unverifiable freeform arm), when the junction is
higher-multiplicity (three+ tangent lines) or non-isolated, or when a near-tangent-terminated arc has a
stall end that is NOT on a localized branch point (a genuine still-open tangency), the marcher SHALL
STOP / DROP the unverifiable arm / LEAVE the arc a `NearTangent` gap (counted in `nearTangentGaps`) and
DEFER it (→ OCCT) — reporting the measured gap. The marcher SHALL NEVER fabricate an arm or an
intersection point past a freeform branch and SHALL NEVER weaken a tolerance to force a branch; the
open-arm reclassification SHALL NOT turn a definite contact into a spurious branch (a definite form
yields no arms ⇒ no branch node ⇒ nothing to reclassify) and SHALL NOT clear a genuine gap (an arc with
a near-tangent end not on a branch stays a gap). A freeform branch that cannot be robustly resolved SHALL
remain an honestly reported gap. No standalone freeform-branch mechanism beyond the open-arm branch-arc
reclassification (and its two per-end near-tangent flags) SHALL be added (no unreachable dead code); the
relative-second-form finite difference SHALL NOT be altered (its central difference is already accurate
on freeform, so no bias-cancellation is shipped).

#### Scenario: an isolated freeform tangent point ends, never sprouting arms

- GIVEN a FREEFORM DEFINITE contact — a B-spline BUMP patch (`z = 0.15·(x²+y²)`) tangent to a plane
  through its minimum, where the relative second form `H` is definite (both relative normal curvatures
  the same sign, so the tangent-cone quadratic has no two distinct real roots), with `enableBranchPoints`
  on
- WHEN the S4-d marcher encounters the contact
- THEN it SHALL let the curve END at the isolated contact, enumerate NO outgoing arm
  (`branchPoints == 0`, `routedArms == 0`, `curveCount() == 0`, `deferredTangent ≥ 1`), and NOT fabricate
  any intersection-curve arm or point across the tangency — exactly as for the analytic isolated tangent
  point

#### Scenario: an unverifiable, off-branch, or higher-multiplicity freeform arc defers honestly

- GIVEN a FREEFORM stall that is either a suspected branch whose enumerated arm's first step does NOT
  re-project on both surfaces within `onSurfTol`, a branch that cannot be localized on both surfaces
  within `onSurfTol`, a near-tangent-terminated arc whose stall end is NOT within the branch merge radius
  of any localized branch point, a freeform CUSP (double root), or a higher-multiplicity junction
  (three+ tangent lines), with `enableBranchPoints` on
- WHEN the S4-d marcher attempts to resolve it
- THEN it SHALL DROP the unverifiable arm and/or leave the backbone's truncated `NearTangent` WLine and
  count the region in `nearTangentGaps` (deferred → OCCT), reporting the measured gap — it SHALL NOT
  reclassify the arc as a `BranchArc`, SHALL NOT fabricate an arm or a point across the branch, and SHALL
  NOT weaken a tolerance to force the crossing
