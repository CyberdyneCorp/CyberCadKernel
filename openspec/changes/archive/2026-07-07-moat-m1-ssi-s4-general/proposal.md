## Why

`openspec/MOAT-ROADMAP.md` **M1** is the general/freeform SSI S4 robustness moat — re-earning
OCCT's `IntPatch`/`IntWalk` degeneracy tuning on adversarial FREEFORM input. The SSI marcher is
native through S1–S5 and S4-a…e. The archived `add-native-ssi-s4d-branch-points` landed the FIRST
S4-d slice: at a genuine SELF-CROSSING of the intersection LOCUS (the transversality sine
`‖n_A × n_B‖ → 0` with a raw-tangent FLIP), it LOCALIZES the branch point B, ENUMERATES the outgoing
arms from the tangent-cone quadratic of the RELATIVE second fundamental form `H = II_A − II_B`,
ROUTES each arm, and ASSEMBLES the multi-arm curve — proven on the ANALYTIC **Steinmetz bicylinder**
(two equal orthogonal cylinders → two ellipses crossing at two branch points, `branchPoints == 2`).

**This change extends S4-d beyond the analytic Steinmetz to a GENERAL / FREEFORM branch point** — the
first-named M1 degeneracy regime ("a general (non-Steinmetz) BRANCH POINT on a FREEFORM surface
pair"). It is scoped as ONE bounded slice with an explicit honest-out (generic machinery already
present + ONE additive assembly piece + verify-or-defer; no fabrication, no dead code).

**Diagnosis on the CURRENT stack (host, `CYBERCAD_HAS_NUMSCI` ON), re-derived and empirically probed;
OCCT is the ORACLE only.** Confirmed by reading `branch_point.h` / `marching.cpp` / `marching.h` AND
by running a temporary marcher probe (since reverted, tree clean) on a bicubic B-spline saddle:

1. **The S4-d branch machinery is already fully surface-agnostic.** `branchpt::localize`,
   `enumerateArms`, `sharedTangentFrame`, `relativeSecondForm`, `solveTangentCone`, and the
   `marching.cpp` `localizeBranchPoints` / `routeArm` / `routeBranches` pipeline touch a surface ONLY
   through `SurfaceAdapter` (`point`, `normal`, `domain`, `uPeriod`, `vPeriod`, `modelScale`). A
   freeform (`makeBSplineAdapter`) pair flows through the SAME `march_branch_impl` / `routeBranches`
   path today.
2. **The branch SIGNATURE + LOCALIZE + the tangent cone already fire correctly on freeform.** On a
   bicubic B-spline saddle `z ≈ 0.15·(x²−y²)` placed TANGENT to a plane through the saddle point (a
   genuine tangency — the probe measured the branch sine collapsing to ≈ 0), the marcher with
   `enableBranchPoints` on LOCALIZES B on both surfaces (`branchPoints == 1`, on-surface residual
   `≤ onSurfTol`) and `enumerateArms` returns the CORRECT FOUR arm rays of the X-crossing — with the
   EXISTING fixed-step code, no change to `relativeSecondForm`.
3. **The relative-second-form finite difference is NOT the freeform gap.** `relativeSecondForm` uses a
   CENTRAL difference `κ = (h(+δ)+h(−δ))/δ²`, which cancels ALL odd-order terms by construction — the
   third-derivative (cubic) term does NOT bias it at leading order. Probed on a genuine bicubic saddle,
   κ at B is stable to ~1e-7 across `δ`, `δ/2`, `δ/4` (error ratio ≈ 4 per halving ⇒ **O(δ²)**
   convergence, absolute error six orders below κ ≈ 0.2 and below the `1e-6·kscale²` discriminant
   margin). A Richardson bias-cancellation would correct a non-existent leading bias and cannot flip
   any Δ sign or arm ray — it would be no-op / dead code, which the moat forbids. (This refutes the
   ORIGINAL design hypothesis for this slice; the change is re-scoped below.)
4. **The ONE real freeform gap is in branch ASSEMBLY topology.** `reclassifyBranchArcs` clears a
   `NearTangent` arc out of `nearTangentGaps` ONLY when BOTH of its endpoints sit on a localized branch
   node — the Steinmetz **closed-network** topology (arcs run between TWO branch points). But a general
   freeform branch on a FINITE patch is an OPEN X: ONE localized branch point with FOUR arms radiating
   to the PATCH BOUNDARY. The probe confirmed each of the four arms has EXACTLY one end on the branch
   node and the other on a domain boundary. `reclassifyBranchArcs` never fires → the branch localizes
   and its arms enumerate + route (`branchPoints == 1`, `routedArms == 3` + the backbone arm), yet
   `nearTangentGaps` stays 4 and `tracedBranches` stays 0 — a partial, inconsistent state that fails a
   `nearTangentGaps == 0` gate. **This is the additive freeform gap.**

**The one additive piece — OPEN-ARM branch-arc reclassification.** Extend `reclassifyBranchArcs` to
recognise the OPEN-ARM topology: a `NearTangent` arc is a RESOLVED arm of the self-crossing locus when
EVERY end that stalled at a near-tangency sits on a LOCALIZED branch point, and at least one end does —
the OTHER end being a clean domain-boundary exit. This subsumes the existing Steinmetz both-ends
(branch-to-branch) rule and adds the freeform branch-to-boundary (open-arm) case. The honesty gate is
per-endpoint: a new pair of WLine flags (`frontNearTangent` / `backNearTangent`) records WHICH end
stalled, so a genuine STILL-OPEN tangency (a near-tangent end NOT on any branch point) is NEVER
reclassified — only a stall that IS a localized, arm-enumerated branch point is treated as resolved.
Everything else (localize, enumerate, route, the honesty gates) is reused UNCHANGED.

**Honest-out (no fabrication, no dead code).** `solveTangentCone`'s discriminant gate is the honesty
core (unchanged): Δ not robustly positive ⇒ NO distinct real tangent lines ⇒ NO arms ⇒ the freeform
contact is an isolated tangent point (the curve ENDS) or a cusp (DEFER → OCCT) — never a fabricated
arm, `branchPoints == 0`. An enumerated arm whose first step will not re-project on both surfaces
`≤ onSurfTol` is DROPPED (existing `routeArm`). An arc with a near-tangent end NOT on a branch stays a
`NearTangent` gap (deferred → OCCT). The Richardson bias-cancellation is **NOT shipped** (refuted as a
no-op). NB this is distinct from the archived "general-branched booleans = geometrically impossible"
decline, which was about the S5 assembler; the SSI-marcher branch point (Steinmetz) is landed and this
extends it to the OPEN freeform X-crossing.

## What Changes

- **(S4-d-g-1) FREEFORM BRANCH DETECTION + LOCALIZE + ENUMERATE + ROUTE (reuse, confirmed).** The
  existing branch-signature capture (`crossNearTangent.branchSignature` → `BranchStall`),
  `branchpt::localize` (sine-minimization + full re-project onto both surfaces), the tangent cone
  (`sharedTangentFrame` / `relativeSecondForm` / `solveTangentCone` / `enumerateArms`), and `routeArm`
  (normal-free `branchpt::reproject` + on-both-surfaces verify) all engage UNCHANGED on a FREEFORM pair
  whose intersection locus self-crosses — empirically confirmed: `branchPoints == 1`, the four arm rays
  enumerated correctly, arms routed. No new detector, no new corrector, no change to the tangent cone,
  no change to `relativeSecondForm`.
- **(S4-d-g-2) OPEN-ARM BRANCH-ARC RECLASSIFICATION (the one additive piece).** In
  `marching.cpp::reclassifyBranchArcs`, generalise the both-ends-on-branch rule: a `NearTangent` arc
  becomes a resolved `BranchArc` (out of `nearTangentGaps`, into `tracedBranches` / `openCurves`) when
  every near-tangent END sits on a localized branch node and at least one end does — covering BOTH the
  Steinmetz branch-to-branch closed network AND the general freeform branch-to-boundary open arm. Add
  two WLine flags (`frontNearTangent` / `backNearTangent`), set during `march_branch_impl` assembly, so
  the reclassifier can tell a resolved open arm (near-tangent end on the branch, other end a clean
  boundary) from an unresolved S4 gap (a near-tangent end NOT on any branch). Additive: the flags
  default `false`, so every non-branch trace (S3 / S4-c / S4-e / S4-f) is byte-identical.
- **(S4-d-g-3) TANGENT-CONE ENUMERATION (reuse VERBATIM).** `relativeSecondForm`, `solveTangentCone`,
  `enumerateArms`, `sharedTangentFrame` are UNCHANGED — the central-difference tangent cone is already
  freeform-accurate (O(δ²)). The Richardson bias-cancellation originally proposed for this slice is NOT
  added (refuted as a no-op — it would be dead code).
- **(S4-d-g-4) HONEST GUARD (reuse + the new per-end gate).** A freeform stall becomes routed arms ONLY
  if it localizes on both surfaces `≤ onSurfTol` and yields two distinct real tangent lines (Δ robustly
  positive); each routed arm verifies its first step `≤ onSurfTol` or is dropped. An arc reclassifies to
  `BranchArc` ONLY if every near-tangent end is a localized branch point — a near-tangent end that is
  NOT on a branch keeps the arc a `NearTangent` gap (deferred → OCCT). A DEFINITE freeform contact
  (Δ ≤ 0) lets the curve END with NO arms (`branchPoints == 0`). NEVER a fabricated arm or an
  intersection point past the branch; NEVER a weakened tolerance.
- **Result reuse (no new counter, no ABI).** Freeform branch points reuse `TraceSet.branchNodes`, the
  `branchPoints` count, and `TraceStatus::BranchArc`. The two new WLine fields are internal C++ (SSI is
  INTERNAL); no `cc_*` ABI change.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-ssi capability with the SECOND S4-d slice
(a GENERAL / FREEFORM branch point), reusing the surface-agnostic localize/enumerate/route machinery
plus ONE additive open-arm branch-arc reclassification. It adds no new capability spec and no cc_* ABI. -->

### Modified Capabilities
- `native-ssi`: extend the S4-d branch-point capability to a GENERAL / FREEFORM branch point — a
  FREEFORM (B-spline / NURBS) surface pair whose intersection LOCUS self-crosses at a point B where the
  pair transversality sine `‖n_A × n_B‖ → 0` with an INDEFINITE relative second fundamental form (a
  saddle contact — the freeform analog of the Steinmetz self-crossing) but where, on a FINITE patch,
  the arms radiate OPEN to the domain boundary rather than closing branch-to-branch. The marcher
  CAPTURES the branch stall via the SAME scale-free sine-collapse signature, LOCALIZES B on both
  surfaces, ENUMERATES the outgoing arms from the tangent-cone quadratic of the relative second form,
  ROUTES each arm with the normal-free point-based corrector (verified on both surfaces `≤ onSurfTol`),
  and — the additive piece — RECLASSIFIES each OPEN arm (one end on the localized branch, the other a
  clean domain boundary) as a resolved `BranchArc`, driving `nearTangentGaps` to 0 for the assembled
  X-crossing while reporting the freeform branch points in `branchPoints`. A freeform contact whose
  tangent-cone quadratic has NO two distinct real roots (a DEFINITE contact — an isolated tangent point)
  lets the curve END with NO arms; a freeform branch that cannot be localized on both surfaces, whose
  arm's first step will not verify, or an arc with a near-tangent end NOT on a branch STILL STOPS +
  defers → OCCT (never a fabricated arm). The analytic Steinmetz branch trace (branch-to-branch,
  `branchPoints == 2`), the transversal S3 trace, the S4-c crossable graze, and the S4-e chart crossings
  are UNCHANGED. The tracer NEVER fabricates an arm or a point past a branch and NEVER weakens a
  tolerance. No `cc_*` ABI change; `src/native/**` stays OCCT-free; the S4-d-g machinery is compiled
  under `CYBERCAD_HAS_NUMSCI` like the rest of S2/S3/S4.

## Impact

- **ABI**: none. SSI is INTERNAL — no `cc_*` entry point, signature, or POD struct change. Additive
  only; the tessellator (`src/native/tessellate`) and the CyberCad app are untouched.
- **Build**: `src/native/ssi/marching.h` gains two `WLine` bool fields (`frontNearTangent` /
  `backNearTangent`, default `false`); `src/native/ssi/marching.cpp` sets them in `march_branch_impl`
  and generalises `reclassifyBranchArcs` to the open-arm topology. No change to `branch_point.h`
  (`localize` / `enumerateArms` / `relativeSecondForm` / `solveTangentCone` / `sharedTangentFrame`
  UNCHANGED — no Richardson), to `routeArm` / `localizeBranchPoints` / `routeBranches`, or to the result
  struct counters (`branchNodes` / `branchPoints` / `BranchArc` reused). Under `CYBERCAD_HAS_NUMSCI`
  like the S3/S4-c/S4-d/S4-e marcher. NO weakening of `onSurfTol` / `tangentSinTol` / `minStep` /
  `maxDeflection`.
- **Verification**: two gates. **Host (no OCCT)** — extend `tests/native/test_native_ssi_marching.cpp`,
  under `CYBERCAD_HAS_NUMSCI`: the LAND fixture is a bicubic B-spline SADDLE `z ≈ 0.15·(x²−y²)`
  (`makeBSplineAdapter`) TANGENT to a plane placed THROUGH the B-spline saddle point (z ≈ 0.2449 at the
  patch centre — NOT z=0, where the two hyperbola branches are DISJOINT and trace as two clean open
  curves). OFF (`enableBranchPoints == false`): the march DEFERS at the crossing (`branchPoints == 0`,
  `nearTangentGaps == 1`, `tracedBranches == 0`). ON: it LOCALIZES B on both surfaces `≤ onSurfTol`, the
  tangent cone yields four arms, they route branch-to-boundary and RECLASSIFY — `branchPoints == 1`,
  `nearTangentGaps == 0`, `tracedBranches == 4`, each arm a `BranchArc` with EXACTLY one end on the
  branch, every node on BOTH surfaces `≤ onSurfTol`. Controls: (a) a freeform DEFINITE contact — a
  B-spline BUMP `z = 0.15·(x²+y²)` tangent to a plane through its minimum — still ENDS with NO arms
  (`branchPoints == 0`, `routedArms == 0`, no fabricated arc); (b) the plane at z=0 (NOT through the
  saddle point) still traces the two DISJOINT open branches unchanged (the existing
  `march_plane_wavy_bspline_open_segments`). Regression: the analytic Steinmetz still
  `branchPoints == 2`, `nearTangentGaps == 0` (branch-to-branch arcs still reclassify — the generalised
  rule reduces to the old both-ends rule when both ends are branches), the transversal pairs unchanged,
  the S4-c graze still crosses, the S4-e sphere-pole / cone-apex / freeform-pole crossings unchanged.
  Full CTest green NUMSCI ON. No OCCT; no tolerance weakened.
  **Sim native-vs-OCCT (booted sim)** — extend `scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`: build the same freeform-saddle ∩ plane as an OCCT
  `Geom_BSplineSurface` (the saddle control net) + `Geom_Plane`, and assert the native freeform branch
  trace matches OCCT `GeomAPI_IntSS` / `IntPatch` — the native branch point matches the OCCT
  self-crossing to `tol`; every sampled native node on the OCCT locus `≤ onCurveTol`
  (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces `≤ onSurfTol`; the branch/arm COUNT matches the
  OCCT locus; total arc length matches. Report per-pair resolved vs still-deferred.
- **Roadmap**: advances `openspec/MOAT-ROADMAP.md` M1 and `SSI-ROADMAP.md` S4-d from the ANALYTIC
  Steinmetz branch point (closed branch-to-branch network) to the FIRST FREEFORM branch point (a
  freeform-saddle ∩ plane OPEN X-crossing now traced vs OCCT). **Explicitly a narrow slice:** open
  freeform branch points on a finite patch land; non-transversal (definite) freeform contacts END with
  no arms; cusps (double root), higher-multiplicity junctions, both-operand-freeform saddle ∩ saddle
  crossings that do not verify, and general near-tangent breadth (S4-c) / coincident-freeform (S4-a)
  remain OUT OF SCOPE and DEFER → OCCT.
- **Risk (honest)**: (a) generalising `reclassifyBranchArcs` could wrongly clear a genuine near-tangent
  gap — mitigated by the per-end `frontNearTangent` / `backNearTangent` gate: an arc is reclassified
  ONLY if every near-tangent end is on a localized, arm-enumerated branch node; a near-tangent end not
  on a branch keeps the arc a gap. (b) The change could perturb the analytic Steinmetz — mitigated by
  construction: the generalised rule reduces to the old both-ends rule when both ends are branches, and
  Steinmetz currently leaves NO unreclassified `NearTangent` arcs (its `nearTangentGaps == 0` assertion
  pins it), so the result is identical. (c) A freeform DEFINITE contact could be mistaken for a branch —
  mitigated by the unchanged Δ ≤ 0 gate (no arms ⇒ no branch node ⇒ nothing to reclassify); the bump
  control pins it. Whatever does not resolve robustly still stops + defers → OCCT and is reported with
  the measured gap; no arm or point is faked, hand-tuned, or weakened to pass.
