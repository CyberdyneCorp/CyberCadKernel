# Design — moat-m1-ssi-s4-general (SSI Stage S4-d, second slice: GENERAL / FREEFORM branch point)

## Context

`openspec/MOAT-ROADMAP.md` **M1** is the general/freeform SSI S4 robustness moat. The archived
`add-native-ssi-s4d-branch-points` landed the FIRST S4-d slice for the ANALYTIC **Steinmetz
bicylinder** (two equal orthogonal cylinders → two ellipses crossing at two branch points), with:

- `branch_point.h`: `localize` (minimize the transversality sine along the approach, then fully
  re-project the minimizer onto both surfaces → B), `sharedTangentFrame` (the mean-normal tangent
  plane at B), `relativeSecondForm` (the relative normal-curvature form `H = II_A − II_B` by CENTRAL
  finite differences of the two surfaces' signed heights), `solveTangentCone` (the quadratic
  `c·m² + 2b·m + a = 0`; Δ = b² − ac > margin ⇒ two lines ⇒ four rays; Δ ≤ 0 ⇒ empty), and
  `enumerateArms`.
- `marching.cpp`: the branch-signature capture in `crossNearTangent`, the `BranchStall` hand-off
  (gated on `enableBranchPoints`), and the `routeBranches` pipeline (`localizeBranchPoints` → per-B
  `BranchNode` → `routeArm` for each ray → `reclassifyBranchArcs`).

`routeArm` re-projects each arm's first step with `branchpt::reproject` — residual `{A.point −
B.point, dot(A.point − anchor, ray) − armStep}` — surface POINT + ray only, NEVER a normal or `∂S/∂u`.
That is exactly why it is well-posed at a branch point where the pair normal degenerates.

**This change extends S4-d to a GENERAL / FREEFORM branch point** (§ Slice A — LANDS narrow), with a
non-transversal / unverifiable freeform contact an honest DECLINE (§ Slice B).

## What is TRUE in the current code (re-derived AND empirically probed)

Confirmed by reading `branch_point.h`, `marching.cpp`, `marching.h` and by a temporary marcher probe
(a bicubic B-spline saddle ∩ plane; since reverted — tree clean). OCCT is the verification ORACLE only.

1. **The S4-d branch machinery is surface-agnostic.** `localize`, `enumerateArms`,
   `sharedTangentFrame`, `relativeSecondForm`, `solveTangentCone`, and the `marching.cpp` routing all
   touch a surface ONLY through `SurfaceAdapter`. A `makeBSplineAdapter` pair flows through the SAME
   `march_branch_impl` / `routeBranches` path.
2. **The branch signature + localize + tangent cone already fire on freeform.** PROBE result: a
   bicubic B-spline saddle `z ≈ 0.15·(x²−y²)` placed tangent to a plane through the saddle point
   (sine at B measured ≈ 0) localizes `branchPoints == 1` on both surfaces `≤ onSurfTol`, and
   `enumerateArms` returns the CORRECT FOUR arm rays with the EXISTING fixed-step code.
3. **`branchpt::reproject` is normal-free.** `routeArm` VERIFIES the first arm node on both surfaces
   `≤ onSurfTol` (returns `nullopt` otherwise), and `localizeBranchPoints` drops any stall that does
   not localize or yields no arms. The verify-or-drop honesty gate is already present.
4. **`relativeSecondForm` is NOT the freeform gap (the original hypothesis for this slice is REFUTED).**
   It reads each relative normal curvature by a CENTRAL difference
   `κ(t) = (h_A(+δ) + h_A(−δ) − h_B(+δ) − h_B(−δ)) / δ²`. A central difference cancels ALL odd-order
   terms, so the third-derivative (cubic) term of a bicubic patch does NOT bias it at leading order.
   PROBE: on a genuine bicubic saddle (with an added cubic term), κ at B is stable to ~1e-7 across
   `δ`, `δ/2`, `δ/4` — error ratio ≈ 4.0 per halving ⇒ **O(δ²)** convergence, six orders below κ ≈ 0.2
   and below the `1e-6·kscale²` discriminant margin. A Richardson `κ★ = (4κ(δ/2) − κ(δ))/3` would
   cancel a NON-existent leading bias and change κ by ~1e-7 — it cannot flip any Δ sign or arm ray. It
   is dead / no-op code and is therefore NOT shipped.
5. **The ONE real freeform gap is in branch ASSEMBLY.** `reclassifyBranchArcs` clears a `NearTangent`
   arc out of `nearTangentGaps` ONLY when BOTH ends sit on a localized branch node — the Steinmetz
   CLOSED network (arcs run between TWO branch points, documented at `marching.h` `BranchArc` as "both
   ends meet LOCALIZED branch points"). A general freeform branch on a FINITE patch is an OPEN X: ONE
   branch point with FOUR arms radiating to the PATCH BOUNDARY. PROBE topology of the localized saddle
   branch (`branchNodes == 1`, four `NearTangent` lines): each line has EXACTLY one end on the branch
   node and the other on a domain boundary. So `reclassifyBranchArcs` never fires →
   `branchPoints == 1`, arms enumerated + routed, yet `nearTangentGaps == 4`, `tracedBranches == 0` — a
   partial, inconsistent state that fails a `nearTangentGaps == 0` gate. THIS is the additive gap.

## Slice A — GENERAL / FREEFORM OPEN-ARM BRANCH POINT (lands, narrow)

### The singularity

A FREEFORM pair whose intersection LOCUS self-crosses at B: the pair transversality sine
`‖n_A × n_B‖ → 0` at B (the surfaces are tangent there) AND the relative second form `H = II_A − II_B`
is INDEFINITE (a saddle contact). By the same tangent-cone argument as Steinmetz, an indefinite `H`
gives two distinct real tangent lines → four outgoing arms → a genuine transversal self-crossing. On a
FINITE freeform patch these four arms radiate OPEN to the patch boundary (branch-to-boundary), NOT the
Steinmetz branch-to-branch closed network.

**LAND fixture — a freeform SADDLE ∩ its tangent plane (plane THROUGH the saddle point).** A bicubic
B-spline saddle patch `z = 0.15·(x² − y²)` over `[-3,3]×[-2,2]` (`makeBSplineAdapter`, 4×4 net). Its
actual saddle point is the surface value at the patch CENTRE — measured `z ≈ 0.2449`, NOT `z = 0` (the
4×4 B-spline does NOT interpolate the analytic saddle at the interior; only the corners are
interpolated). A plane placed THROUGH the saddle point (`z ≈ 0.2449`) makes the `z = const` level set
DEGENERATE to the crossing: `x² = y² ⇒ y = ±x`, two curves crossing at B, `H = diag(κ, −κ)` indefinite
⇒ Δ > 0 ⇒ four arms. A plane at `z = 0` is NOT tangent (it sits below the saddle point) — the two
hyperbola branches are DISJOINT and trace as two clean open curves (the existing
`march_plane_wavy_bspline_open_segments`); that is the "not through the saddle point" control. One
operand is a genuine B-spline adapter (point/normal from B-spline evaluation), so the pair is a
FREEFORM pair — distinct from the two-quadric Steinmetz.

### The one additive piece — OPEN-ARM branch-arc reclassification

`reclassifyBranchArcs` today reclassifies a `NearTangent` arc to `BranchArc` ONLY when BOTH ends are on
a branch node (Steinmetz closed network). Generalise it to the OPEN-ARM topology, guarded per-endpoint:

```
for each NearTangent WLine w (≥ 2 pts):
    frontAtBr = some branch node within mergeRadius of w.points.front()
    backAtBr  = some branch node within mergeRadius of w.points.back()
    // an end that STALLED at a near-tangency but is NOT on a branch node is a genuine open S4 gap
    if (w.frontNearTangent && !frontAtBr) or (w.backNearTangent && !backAtBr):  continue   // DEFER
    if not (frontAtBr or backAtBr):  continue                                              // not a branch arc
    w.status = BranchArc;  --nearTangentGaps;  ++openCurves;  ++tracedBranches
```

- `w.frontNearTangent` / `w.backNearTangent` are two NEW `WLine` bool fields (default `false`). In
  `march_branch_impl` the stitched points are `[reversed backward half] seed [forward half]`, so
  `points.front()` is the backward-march terminus and `points.back()` the forward-march terminus:
  `frontNearTangent = (b.end == DirEnd::NearTangent)`, `backNearTangent = (f.end == DirEnd::NearTangent)`.
  Set ONLY on the `NearTangent` status branch; both stay `false` for `Closed` / `BoundaryExit`.
- This is a STRICT GENERALISATION: when both ends are branch nodes (Steinmetz), both near-tangent ends
  are on branches, so neither is "unresolved" and both `atBr` are true → reclassify, EXACTLY as before.
  When ONE end is a near-tangent stall on the branch and the OTHER is a clean boundary (freeform open
  arm), the boundary end is not `*NearTangent` so it does not block, and the arc reclassifies.
- HONESTY: an arc with a near-tangent end that is NOT on a localized branch point (a genuine remaining
  S4 gap) is NEVER reclassified — the per-end flags are exactly what distinguishes a resolved open arm
  from an unresolved gap. Only a stall that IS a localized, arm-enumerated branch point (`Δ > margin`
  already gated it) counts as resolved.

**Why the flags, not geometry alone.** Without recording WHICH end stalled, the reclassifier could not
tell a clean domain-boundary exit from an interior tangency stall, and a "one end at branch → resolve"
rule would risk hiding a real gap at the OTHER (still-tangent) end. The per-end flags make the gate
exact and additive (default `false` ⇒ every non-branch trace is byte-identical).

### Routing + assembly (reused unchanged)

`localizeBranchPoints` (localize + dedup + enumerate; drop if no localize / no arms), `routeArm` (step
off B, normal-free `branchpt::reproject`, verify on both surfaces `≤ onSurfTol`, march the arm with
branch points OFF, drop a winding/unverifiable arm), the `BranchNode` / `connectArm` assembly, and the
tangent cone (`relativeSecondForm` / `solveTangentCone` / `enumerateArms`) are all reused VERBATIM.
Only `reclassifyBranchArcs` is generalised, plus the two WLine flags it consumes.

### Honest guard (reused)

A freeform branch point becomes a routed `BranchNode` ONLY if it localizes on both surfaces
`≤ onSurfTol`, Δ is robustly positive (two distinct real lines), and each routed arm's first step
verifies on both surfaces `≤ onSurfTol` with real progress off B. Otherwise the stall is DROPPED and
the backbone's honest `NearTangent` gap stays in `nearTangentGaps` (deferred → OCCT). A freeform
DEFINITE contact (Δ ≤ 0) lets the curve END with NO arms. An arc whose near-tangent end is not on a
branch stays a gap. NEVER a fabricated arm or point; NEVER a weakened tolerance.

## Slice B — NON-TRANSVERSAL / UNVERIFIABLE FREEFORM CONTACT (honest DECLINE, no dead code)

The SAME tangent-cone honesty core that returns four arms for a freeform saddle returns ZERO for a
freeform DEFINITE contact — and that is the correct outcome:

- **Freeform isolated tangent point (Δ ≤ 0 — definite `H`).** A B-spline BUMP `z = 0.15·(x²+y²)`
  tangent to a plane through its minimum: `H = diag(κ, κ)` (definite) ⇒ Δ ≤ 0 ⇒ `solveTangentCone`
  returns EMPTY ⇒ `localizeBranchPoints` drops it ⇒ the curve ENDS at the isolated contact
  (`branchPoints == 0`, `routedArms == 0`), never sprouting arms. PROBE confirmed
  `curves == 0`, `branchPoints == 0`, `deferredTangent == 1`.
- **Freeform cusp (Δ ≈ 0 — double root).** A degenerate/tangential freeform contact whose tangent cone
  has a double root is a cusp — no two distinct lines — DEFER → OCCT (the same as the analytic cusp
  decline; by the IFT a curve cusp coincides with the pair-tangency regime).
- **Unverifiable / still-open freeform arc.** A `NearTangent` arc whose stall end is NOT on a localized
  branch point (an unverifiable or not-yet-localized tangency) is left as an honest `NearTangent` gap
  by the per-end reclassification guard, deferred → OCCT.

**No standalone freeform-branch mechanism is added beyond the open-arm reclassification.** Higher-
multiplicity junctions (three+ tangent lines), non-isolated branch curves, and self-intersection
completeness are OUT OF SCOPE and DEFER → OCCT.

## Goals / Non-Goals

**Goals**
- (S4-d-g-1) Confirm the branch signature, `localize`, the tangent cone, and `routeArm` engage
  UNCHANGED on a freeform self-crossing pair (empirically: `branchPoints == 1`, four arms enumerated).
- (S4-d-g-2) Add OPEN-ARM `reclassifyBranchArcs` (per-end `frontNearTangent` / `backNearTangent` gate)
  so a freeform branch-to-boundary X-crossing resolves; keep Steinmetz branch-to-branch identical.
- (S4-d-g-3) Trace a freeform-saddle ∩ plane X-crossing: `branchPoints == 1`, `nearTangentGaps == 0`,
  `tracedBranches == 4`, every node on both surfaces `≤ onSurfTol`, verified vs OCCT.
- (S4-d-g-4) Honest guard: END a definite freeform contact with no arms; DROP an unverifiable arm;
  keep an off-branch near-tangent end a gap; DEFER a cusp → OCCT; never fabricate; never weaken a
  tolerance.

**Non-Goals (deferred — never faked here)**
- **The Richardson bias-cancellation of `relativeSecondForm`** — REFUTED as a no-op (the central
  difference is already O(δ²)); NOT shipped.
- **Non-transversal (definite) freeform contacts** → the curve ends with NO arms (an isolated tangent
  point), no fabricated arm.
- **Freeform cusps** (double root) and higher-multiplicity junctions (three+ tangent lines) → DEFER → OCCT.
- **Both-operand-freeform saddle ∩ saddle** if it does not verify → DEFER → OCCT.
- **General near-tangent breadth (S4-c beyond the analytic graze)** and **coincident/overlapping
  freeform surfaces (S4-a)** — the other two M1 regimes, separate slices.
- **A full brep freeform-branch SOLID through the boolean pipeline** — the slice is at the marcher level
  (`trace_intersection` / `trace_from_seeds`), the SAME envelope as the archived Steinmetz / sphere-pole
  fixtures.
- **Any change to `src/native/tessellate`, the `cc_*` ABI, `branch_point.h`, the analytic Steinmetz
  branch trace, the S3 transversal trace, the S4-c graze crossing, or the S4-e chart crossings.**
- **Weakening `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection`, or any tolerance to "pass".**

## Module shape

```
src/native/ssi/marching.h            [extend — two additive WLine flags]
  struct WLine {
    // ... existing fields UNCHANGED ...
    bool frontNearTangent = false;   // did points.front() (backward-march terminus) stall at a tangency?
    bool backNearTangent  = false;   // did points.back()  (forward-march  terminus) stall at a tangency?
  }

src/native/ssi/marching.cpp          [extend — set the flags + generalise reclassify]
  march_branch_impl(...):   on the NearTangent branch, set frontNearTangent = (b.end==NearTangent),
                            backNearTangent = (f.end==NearTangent).   // else both stay false
  reclassifyBranchArcs(...): OPEN-ARM rule — reclassify a NearTangent arc to BranchArc when every
                            near-tangent END is on a localized branch node and at least one end is;
                            a near-tangent end NOT on a branch keeps the arc a gap.
  // crossNearTangent.branchSignature / BranchStall / localizeBranchPoints / routeArm / routeBranches
  // and branch_point.h (localize / tangent cone / relativeSecondForm) : UNCHANGED.
```

`src/native/**` stays OCCT-free. The only new code is the two WLine flags and the generalised
`reclassifyBranchArcs`. No new result counter (freeform branch points reuse `branchNodes` /
`branchPoints` / `BranchArc`); no `cc_*` change; `branch_point.h` untouched.

## Verification model (two gates)

- **Host (no OCCT).** Extend `tests/native/test_native_ssi_marching.cpp`, under `CYBERCAD_HAS_NUMSCI`:
  - **Freeform branch now traced** (`march_freeform_saddle_branch_open_arms_s4d`). The bicubic B-spline
    saddle tangent to a plane through the saddle point. OFF: DEFERS (`branchPoints == 0`,
    `nearTangentGaps == 1`, `tracedBranches == 0`). ON: `branchPoints == 1`, `nearTangentGaps == 0`,
    `tracedBranches == 4`, `openCurves == 4`, each arm a `BranchArc` with EXACTLY one end on the branch
    (the other a boundary), the branch node on both surfaces `≤ onSurfTol` and at the saddle point,
    every node on BOTH surfaces `≤ onSurfTol`.
  - **Freeform definite contact ends (no arms)** (`march_freeform_bump_definite_never_branches_s4d`). A
    B-spline BUMP tangent to a plane → `branchPoints == 0`, `routedArms == 0`, `curveCount() == 0`,
    `deferredTangent ≥ 1`, no fabricated arm.
  - **Not-through-the-saddle control** (existing `march_plane_wavy_bspline_open_segments`, z = 0): two
    DISJOINT open curves, `branchPoints == 0`, `nearTangentGaps == 0` — UNCHANGED.
  - **Analytic Steinmetz unchanged.** The Steinmetz bicylinder still `branchPoints == 2`,
    `nearTangentGaps == 0` (branch-to-branch arcs still reclassify — the generalised rule reduces to
    the old both-ends rule; both `march_steinmetz_branch_points_s4d` and the S4-e regression pin it).
  - **Regression.** The transversal S3 pairs, the S4-c graze, the S4-e sphere-pole / cone-apex /
    freeform-pole crossings, the S4-f completeness — all unchanged (branch points OFF ⇒ the flags are
    unused; the polylines are byte-identical). Full CTest green NUMSCI ON.
- **Sim native-vs-OCCT (booted simulator).** Extend `scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm`: build the freeform-saddle ∩ plane as an OCCT
  `Geom_BSplineSurface` (the saddle control net) + `Geom_Plane`; assert the native freeform branch
  trace reproduces the OCCT `GeomAPI_IntSS` / `IntPatch` self-crossing — branch/arm count matches, the
  branch point matches the OCCT self-crossing to `tol`, every sampled native node on the OCCT locus AND
  on both OCCT surfaces `≤ onSurfTol`, arc length matches. Report per-pair resolved vs still-deferred.

## Decisions

- **Reuse the surface-agnostic branch machinery — a freeform branch needs no new localize/route/cone.**
  `localize`, `enumerateArms`, `routeArm`, and `relativeSecondForm` are already `SurfaceAdapter`-generic
  and (probed) accurate on freeform; the slice adds NO branch mechanism and NO Richardson.
- **The ONE additive piece is OPEN-ARM branch-arc reclassification.** The gap was assembly topology, not
  the tangent cone: the closed-network `reclassifyBranchArcs` could not resolve a single freeform branch
  with open arms. Generalise it, guarded per-endpoint so no genuine gap is hidden.
- **The tangent-cone discriminant is the honesty core (unchanged).** Δ > margin ⇒ arms; Δ ≤ 0 ⇒ none. A
  definite freeform contact ENDS; a cusp defers; an unverifiable arm drops; an off-branch near-tangent
  end stays a gap. No arm is fabricated.
- **Do NOT ship the Richardson bias-cancellation.** It was the original hypothesis for this slice but is
  empirically a no-op (central difference already O(δ²)) — shipping it would be dead code, forbidden.
- **Marcher-level fixtures, the SAME envelope as the archived slices.** No native freeform-branch brep
  SOLID; the saddle ∩ plane adapter + `trace_intersection` mirrors the Steinmetz / sphere-pole fixtures.
- **Additive, ABI-stable, tessellator-untouched.** Two WLine flags + the generalised reclassify; no new
  result counter, no `cc_*` change; `branch_point.h` and `src/native/tessellate` untouched; under
  `CYBERCAD_HAS_NUMSCI`.

## Risks / Trade-offs

- **Generalised reclassify wrongly clears a real gap.** Mitigation: the per-end `frontNearTangent` /
  `backNearTangent` gate reclassifies ONLY when every near-tangent end is on a localized, arm-enumerated
  branch node; a near-tangent end not on a branch keeps the arc a gap. Accepted.
- **Perturbing the analytic Steinmetz branch trace.** Mitigation: the generalised rule reduces to the
  old both-ends rule when both ends are branches, and Steinmetz leaves no unreclassified `NearTangent`
  arcs (its `nearTangentGaps == 0` / `branchPoints == 2` assertions pin it). Accepted.
- **Freeform definite contact mistaken for a branch.** Mitigation: Δ ≤ 0 ⇒ no arms ⇒ no branch node ⇒
  nothing to reclassify; the bump control pins it. Accepted.
- **Both-freeform saddle ∩ saddle does not verify.** Mitigation: OUT OF SCOPE; if it does not verify it
  DEFERS → OCCT and the saddle ∩ plane freeform operand is the honest LAND. Whatever does not resolve
  robustly still stops + defers → OCCT and is reported with the measured gap; no arm or point is faked,
  hand-tuned, or weakened to pass.
