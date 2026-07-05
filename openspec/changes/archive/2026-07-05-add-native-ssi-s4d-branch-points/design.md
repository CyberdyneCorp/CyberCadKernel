# Design — add-native-ssi-s4d-branch-points (SSI Stage S4-d, first slice)

## Context

The native SSI stack is S1 analytic → S2 seeding → S3 marching, hardened by the S4-a/S4-b
CLASSIFICATION layers (typed `CoincidentRegion` + `TangentContact`) and the S4-c near-tangent
MARCHING-THROUGH slice (a `NearTangentTransversal` single-branch graze the S3 marcher used to
truncate is now MARCHED THROUGH via a fixed-plane-cut corrector). **S4-d is the hardest SSI
piece: BRANCH POINTS / BIFURCATIONS** — where the intersection LOCUS itself crosses or splits
(multiple curve arms meet at one point). The marcher must LOCALIZE the branch point, ENUMERATE
the outgoing arms, ROUTE down each, and ASSEMBLE the multi-arm curve.

The S4-c marcher already DETECTS a branch point — and currently DEFERS it. In
`marching.cpp`, `crossNearTangent` / `crossNodeCrossable` discriminate a crossable graze from
a branch point by TWO witnesses (the honesty core of S4-c):

- **STEEP SINE COLLAPSE.** A crossable graze keeps a BOUNDED minimum transversality sine
  (`‖nA × nB‖`); a true tangency / BRANCH POINT drives sine → 0. `crossNearTangent` defers
  when the stall sine falls below a fraction of the last-good sine (`steepCollapse =
  sine0 < 0.25·lastGoodSine`) or a fine look-ahead scan's minimum drops below `minCrossSine`
  (`0.3·bandEnterSin`).
- **RAW TANGENT FLIP.** The raw cross-product tangent field is smooth along a single branch;
  at a branch point it turns sharply / reverses. `crossNodeCrossable` rejects a `≥ 60°`
  turn or a continuity-oriented tangent that no longer heads the crossing way.

When it sees collapse + flip it ROLLS BACK and defers (`DirEnd::NearTangent`,
`stopReason = NearTangentTransversal`). **S4-d must, at that same detection, instead LOCALIZE
the branch point and ROUTE the arms.**

### Diagnosis (the "before", confirmed on the current marcher)

Host build, `CYBERCAD_HAS_NUMSCI` ON, `trace_intersection` on two equal orthogonal cylinders
(`makeCylinderAdapter`, R=1, axis Z and axis X crossing at the origin — the Steinmetz
bicylinder):

| Fixture | S3+S4-c result today | Verdict |
|---|---|---|
| Steinmetz — equal R=1 cylinders, axes crossing at 90° | one WLine, `status = NearTangent`, **229 pts**, `stopReason = NearTangentTransversal` (sine ≈ 9.2e-4), `tracedBranches = 0`, `nearTangentGaps = 1`, `nearTangentCrossed = 0`, `deferredTangent = 0` | **DEFERS at a branch point** — the four arms of the two crossing ellipses are never assembled |
| Two spheres at `d = R₁+R₂` (external tangent) | S4-b `TangentPoint`, no curve fabricated | **isolated tangent point — must STILL END** |
| Sphere grazed by an offset cylinder (S4-c graze) | `nearTangentGaps = 0`, `nearTangentCrossed ≥ 1`, full loop | **crossable graze — must stay CROSSED** |
| 5 transversal pairs | `nt = 0`, bit-identical | **must stay UNTOUCHED** |

The Steinmetz intersection is analytically known — two ellipses (in the planes `x = ±z`,
each of semi-axes `1` and `√2`) that CROSS each other at the two saddle points `(0, ±1, 0)`.
At each saddle FOUR elliptical arms meet: it is a genuine transversal self-crossing of the
locus, the canonical S4-d target. The S3+S4-c marcher walks 229 nodes up to the first saddle
and honestly stops (the two witnesses fire — sine collapses to ≈ 9e-4 and the raw tangent is
about to flip through the crossing). The full four-arm curve is well-defined, so OCCT
`IntPatch` parity is well-defined.

**Honest nuance (scope-setting).** S4-d's FIRST slice targets the ELEMENTARY TRANSVERSAL
SELF-CROSSING (Steinmetz family): the tangent-cone quadratic at B has TWO DISTINCT REAL roots
⇒ two tangent lines ⇒ up to four arms. A DOUBLE root (a cusp/degenerate branch), a
higher-order singularity, a surface's own parametric singularity (cone apex, sphere pole —
S4-e), or a self-intersection below the seeding floor (S4-f) is OUT OF SCOPE and DEFERS. An
isolated `TangentPoint` (S4-b definite form — the quadratic has no real distinct roots) is
NOT a branch point and MUST still END, never sprouting fabricated arms.

The method is clean-room; OCCT (`IntPatch` / `IntWalk` / `GeomAPI_IntSS`) is the verification
ORACLE only — the branch-point localization, tangent-cone enumeration, and arm routing are
re-derived, never copied.

## Goals / Non-Goals

**Goals**
- (S4-d-1) At the S4-c collapse + flip DETECTION, DISCRIMINATE a genuine branch point (locus
  self-crossing, multiple real arms) from a plain tangency, reusing the existing witnesses.
- (S4-d-2) LOCALIZE the branch point B: refine the interior on-both-surfaces point where the
  transversality sine reaches its minimum (≈ 0) along the approach, `B` on both surfaces
  ≤ `onSurfTol`.
- (S4-d-3) ENUMERATE the outgoing arms as the REAL, distinct roots of the tangent-cone
  quadratic from the local second-order structure of the two surfaces at B — up to four rays;
  NO real distinct roots ⇒ NOT a transversal branch ⇒ END / DEFER (never fabricate arms).
- (S4-d-4) ROUTE each arm: step off B along its ray with the S4-c fixed-plane corrector
  (verified on both surfaces), then continue the normal S3 march to termination.
- (S4-d-5) DEDUP arms already traced (extend `retraces`), CONNECT arms meeting at the same B,
  and ASSEMBLE the multi-arm curve into the `TraceSet` with a reported `branchPoints` count.
- Emit the FULL Steinmetz curve (both ellipses, both branch points localized + routed),
  verified vs OCCT (on-surface, on-locus, branch-point match, arm/loop count).

**Non-Goals (deferred — never faked here)**
- **General / freeform branch points** — arbitrary self-crossings of the intersection locus
  on freeform surfaces, or three-plus tangent lines at one point. Only the elementary
  two-real-distinct-line transversal self-crossing (Steinmetz family) is in scope.
- **Cusps / degenerate branches** (a DOUBLE root of the tangent-cone quadratic — one tangent
  line, higher-order contact). Detected and DEFERRED, never routed.
- **S4-e: singular points** (a surface's own degeneracy — cone apex, sphere pole) on the
  locus.
- **S4-f: self-intersection completeness / global intersection-topology repair** (small loops
  below the seeding floor, full self-intersection resolution).
- **Any change to `src/native/tessellate`, the `cc_*` ABI, the S3 transversal trace, or the
  S4-c crossable-graze crossing.** The S3 corrector / deflection controller and the S4-c
  crossing driver are bit-identical; the branch machinery engages ONLY at a detected branch
  point.
- **Weakening `tangentSinTol`, `minCrossSine`, `onSurfTol`, or any tolerance to "pass".** A
  branch that still defers is an honestly reported gap.

## Module shape

```
src/native/ssi/marching.h           [extend — additive result fields]
  struct BranchNode {                // S4-d: a localized branch point + its arm connectivity
    math::Point3 point{};            // the localized branch point B (on both surfaces ≤ onSurfTol)
    double branchSine = 0.0;         // ‖nA×nB‖ at B (≈ 0 — the transversality minimum)
    std::vector<int> armLineIds{};   // ids of the WLines whose ends meet at this branch point
  };
  struct TraceSet {
    // ... existing fields unchanged ...
    int branchPoints = 0;                 // S4-d: branch points localized + routed (arms assembled)
    std::vector<BranchNode> branchNodes{};// the localized branch points + their arm connectivity
    // nearTangentGaps keeps counting ONLY the branch points that could NOT be resolved (deferred)
  }
  struct MarchOptions {
    // ... existing fields unchanged ...
    double armStepFrac = -1.0;   // step off B when routing an arm (≤0 → h0/8); tangentSinTol-independent
    double branchMergeFrac = -1.0; // two branch points merge if within branchMergeFrac·scale (≤0 → 1e-4)
    bool   enableBranchPoints = true; // S4-d master switch (off → S4-c defer, exactly as today)
  }

src/native/ssi/marching.cpp         [extend — the S4-d branch machinery, CYBERCAD_HAS_NUMSCI]
  // isBranchPoint(A,B, stall, tStar, lastGoodSine, t, scale) → bool
  //   at the S4-c collapse+flip detection, decide a genuine locus self-crossing:
  //   NOT a TangentPoint/TangentCurve/Undecided (S4-b), interior on both surfaces, and the
  //   tangent-cone quadratic (enumerateArms) yields ≥ 2 distinct real directions.
  // localizeBranchPoint(A,B, approach, tStar, t) → optional<State>
  //   minimize sine(s) = ‖nA×nB‖ along the approach (nn::minimize / bracketed 1-D), then
  //   re-project B onto both surfaces (nn::least_squares). nullopt if not robust.
  // enumerateArms(A,B, B_state) → vector<Vec3>
  //   real distinct roots of the tangent-cone quadratic from the relative 2nd fundamental
  //   form at B; ±each line → up to 4 rays. Empty / <2 lines → not a transversal branch.
  // routeArm(A,B, B_state, rayDir, t, scale, out) → DirResult
  //   step off B by armStep along rayDir, S4-c fixed-plane correct back onto the curve
  //   (verify on both surfaces ≤ onSurfTol), then marchDir the normal S3 walk to termination.
  // marchDir / trace_from_seeds: on the branch-point detection, localize B, enumerate + route
  //   the arms, record a BranchNode, dedup retraced arms (extended retraces), CONNECT arms at
  //   B; on a non-resolvable branch, STOP + defer EXACTLY as S4-c does today.
```

`src/native/**` stays OCCT-free. The new machinery lives entirely in `marching.cpp` under
`CYBERCAD_HAS_NUMSCI` (it calls `nn::minimize` / `nn::least_squares`). It reuses the S4-b
`classify_tangent_contact_seeded` (`tangent_seeded.h`) unchanged to REJECT a `TangentPoint` /
`TangentCurve` / `Undecided`, and the S4-c `crossNearTangent` seam unchanged for the
collapse+flip detection. No new substrate routine; no new hand-tuned constant beyond the
branch discriminators (which are `tangentSinTol` / `minCrossSine`-derived, documented, and
never weaken a tolerance).

## S4-d-1 — Branch-point detection (reuse the S4-c seam)

The S4-c marcher stops at the branch point because `crossNearTangent` sees collapse + flip
and defers. S4-d hooks the SAME point: when `crossNearTangent` is about to return "not
crossable" for a `NearTangentTransversal` stall (`enableBranchPoints` on), run
`isBranchPoint`. A stall is a genuine BRANCH POINT iff ALL hold; otherwise fall through to the
existing S4-c defer:

1. **Not a decided tangency (S4-b).** `classify_tangent_contact_seeded` at the stall is NOT
   `TangentPoint` (sign-definite relative II — isolated, the curve ENDS) and NOT
   `TangentCurve` (rank-1 — a tangent seam) and NOT `Undecided`. (`NearTangentTransversal` /
   an indefinite form is the branch candidate.)
2. **Interior, not a boundary.** The localized minimum B lies in the INTERIOR of both param
   domains (not on a non-periodic edge — a curve leaving the patch is a `BoundaryExit`, not a
   branch).
3. **A real tangent cone.** `enumerateArms` at the localized B yields ≥ 2 DISTINCT REAL
   directions (two tangent lines). This is the definitive discriminator: a transversal
   self-crossing has an INDEFINITE relative second form ⇒ two real roots; an isolated
   `TangentPoint` has a definite form ⇒ NO real distinct roots ⇒ NOT a branch (END/DEFER).

`isBranchPoint` NEVER fabricates: it can only return true when B was robustly localized on
both surfaces AND the tangent cone is genuinely real. Every other outcome is the existing
S4-c defer, byte-for-byte.

## S4-d-2 — Branch-point localization

The S4-c stall gives an APPROACH: the last-good node (sine ≥ `bandEnterSin`) and the overshoot
node (sine collapsed below the floor), bracketing the sine minimum along the march direction
`tStar`. Localize B as the point on the intersection where the transversality sine is minimal:

```
minimize   g(s) = ‖nA(P(s)) × nB(P(s))‖    for s along the approach, P(s) re-projected onto both surfaces
```

Realized as a bracketed 1-D minimize (`nn::minimize` over the along-`tStar` arc coordinate,
each trial point re-projected onto both surfaces with the S4-c fixed-plane corrector so it
stays ON the intersection), giving the arc parameter `s★` of the minimum; then a full
re-project of `P(s★)` onto both surfaces (`nn::least_squares`, the S3 corrector residual
`A.point − B.point` without an advance term) yields B with `‖A.point − B.point‖ ≤ onSurfTol`
and `sine(B)` at/near the floor. If the minimize does not bracket a clear minimum, or B does
not re-project on both surfaces ≤ `onSurfTol`, LOCALIZATION FAILS ⇒ DEFER (no fabricated B).

## S4-d-3 — Arm enumeration: the tangent cone (real roots only)

At the branch point B the intersection locus is a curve with a singular point whose TANGENT
CONE is the set of directions `T` in the shared tangent plane along which the two surfaces
agree to second order. Working in the shared tangent plane at B (basis `{e₁, e₂}` orthonormal,
both `⊥` to the common normal `n ≈ nA ≈ nB`), the surfaces' second-order behaviour is captured
by their second fundamental forms `II_A`, `II_B` (2×2 symmetric, from the surface Hessians
projected on `n`). The intersection-locus tangent directions are the null directions of the
RELATIVE second fundamental form `H = II_A − II_B`:

```
H = [[a, b], [b, c]]  (2×2 symmetric)
For T = e₁·cosθ + e₂·sinθ, the quadratic   a·x² + 2b·x·y + c·y² = 0  (x=cosθ, y=sinθ)
⇒ in slope m = y/x:  c·m² + 2b·m + a = 0   (or the x-only branch when c ≈ 0)
```

- **Discriminant `Δ = b² − a·c > 0` (INDEFINITE `H`)** ⇒ TWO distinct real slopes ⇒ two
  tangent lines `T₁, T₂` in the tangent plane ⇒ FOUR outgoing rays `±T₁, ±T₂`. This is the
  transversal self-crossing — the Steinmetz saddle. Return the (up to four) real ray
  directions in world space (mapped back through `{e₁, e₂}`).
- **`Δ ≤ 0` (DEFINITE / SEMIDEFINITE `H`)** ⇒ NO two distinct real lines ⇒ NOT a transversal
  branch. `Δ < 0` (definite) is an isolated `TangentPoint` — the curve ENDS (no arms).
  `Δ ≈ 0` (a double root) is a cusp / degenerate branch — OUT OF SCOPE ⇒ DEFER. In BOTH cases
  return EMPTY: `isBranchPoint` fails and the marcher DEFERS / the isolated tangent ENDS.

The `II_A`, `II_B` are computed from finite-difference surface Hessians at B (the adapter
exposes `point`/`normal` only, like `advanceParams`), projected on `n`. This is the honesty
crux of S4-d: the SIGN of the tangent-cone quadratic's discriminant is exactly the S4-b
point-vs-cross distinction, so an isolated `TangentPoint` can NEVER produce arms.

## S4-d-4 — Arm routing

For each enumerated real ray direction `Tᵢ` (already a unit vector in world space, tangent to
both surfaces at B):

1. **Step off B.** `P₀ = B + armStep · Tᵢ` with `armStep = h0/8` (a small step clear of the
   near-zero-sine core so the local intersection tangent is well-conditioned again).
2. **Land back on the curve.** Re-project `P₀` onto both surfaces with the S4-c fixed-plane
   corrector (advance along `Tᵢ`, `t★ = Tᵢ`), giving `S₀` with `‖A.point − B.point‖ ≤
   onSurfTol`. If it does not converge on both surfaces, DROP this arm (do NOT fabricate it).
3. **March the arm.** Run the normal S3 `marchDir` from `S₀` in the `Tᵢ` sense to its natural
   termination (loop closure, boundary exit, or ANOTHER branch point — which recurses into
   the same detection, bounded by a total branch-point budget). The routed arm is emitted as a
   WLine exactly like an S3 branch, tagged with the branch-point id at its origin end.

An arm is emitted ONLY if step 2 verifies on both surfaces AND step 3 makes real progress
(more than a couple of nodes). A ray whose first verified step immediately re-enters the
branch-point core (sine collapses again without progress) is a spurious enumeration artifact
and is DROPPED. No arm is ever fabricated.

## S4-d-5 — Dedup + connectivity + assemble

- **Dedup (extend `retraces`).** Each of the up-to-four arms is a separate march; two of them
  are the SAME locus arm approached from opposite branch points (e.g. the elliptical arc
  between the two Steinmetz saddles is routed once from each end). The existing node-proximity
  `retraces` already folds a duplicate polyline; S4-d extends it so an arm routed from a
  branch point that retraces a kept arm is deduped AND its shared branch-point connectivity is
  merged into the existing `BranchNode` (not dropped silently). An arm that closes a loop back
  through the SAME branch point (Steinmetz: each ellipse is a loop through both saddles) is
  recorded as a closed multi-arm loop.
- **Connectivity.** Arms whose routed origin ends lie within `branchMergeFrac·scale` of the
  SAME localized B are recorded in one `BranchNode` (`point = B`, `branchSine ≈ 0`,
  `armLineIds = {the WLine ids meeting there}`). Two branch points within `branchMergeFrac`
  merge (robustness against localizing the same saddle twice from two arms).
- **Assemble.** `trace_from_seeds` gains: for each resolved branch point, `++branchPoints` and
  a `BranchNode` appended to `branchNodes`; the routed arms are WLines in `lines` like any
  branch. A branch point that could NOT be localized / enumerated / routed does NOT add a
  `BranchNode`; its stall stays a `NearTangent` WLine counted in `nearTangentGaps` with the
  typed `stopReason` — the honest defer, unchanged from S4-c.

## Branch-resolved-vs-deferred scope (honest)

| Configuration at the stall | S4-d action | counted as |
|---|---|---|
| Transversal self-crossing (indefinite `H`, ≥2 real tangent lines), B localized on both surfaces, arms route + verify | **LOCALIZE + ENUMERATE + ROUTE + ASSEMBLE** | `branchPoints`, arms in `lines` |
| Self-crossing suspected but B does NOT re-project on both surfaces ≤ `onSurfTol` | STOP + defer → OCCT | `nearTangentGaps` |
| Tangent-cone quadratic has NO 2 distinct real roots (definite `H`) — an isolated `TangentPoint` | END (curve ENDS, S4-b `TangentPoint`) — NO arms | (no branch; tangent-classified) |
| Double root (cusp / degenerate branch) | STOP + defer → OCCT (out of scope) | `nearTangentGaps` |
| `TangentCurve` (rank-1) / `Undecided` (noise band) at the stall | STOP + classify + defer → OCCT | `nearTangentGaps` |
| An enumerated arm's first step won't verify on both surfaces / makes no progress | DROP that arm (never fabricated); other arms still routed | — |
| `NearTangentTransversal` single-branch graze (S4-c) | S4-c MARCH THROUGH (unchanged) | `nearTangentCrossed` |
| Transversal region (`sine ≥ bandEnterSin`) | normal S3 march (unchanged) | traced |

## Verification model (two gates)

- **Host (no OCCT).** Extend `tests/native/test_native_ssi_marching.cpp` (or a new
  `tests/native/test_native_ssi_s4d_branch_points.cpp`), all under `CYBERCAD_HAS_NUMSCI`:
  - **Steinmetz now fully traced.** Two equal R=1 cylinders, axes Z and X crossing at the
    origin (the fixture that S3+S4-c today truncate — one `NearTangent` WLine, ~229 pts,
    `nearTangentGaps = 1`, `branchPoints = 0`) is now FULLY traced: `branchPoints = 2` (both
    saddles `(0, ±1, 0)` localized, each on both cylinders ≤ `onSurfTol`, `branchSine` at/near
    the floor), the two ellipses assembled from the routed arms, `nearTangentGaps = 0`, every
    emitted node on BOTH cylinders ≤ `onSurfTol`, and the assembled arc set matching the
    analytic Steinmetz ellipses (each in a plane `x = ±z`, semi-axes `1` and `√2`) within the
    deflection tolerance.
  - **Isolated tangent point STILL ends.** Two spheres at `d = R₁+R₂` STILL end as an S4-b
    `TangentPoint` — NO arms fabricated, `branchPoints = 0`, no curve across the tangency.
  - **S4-c graze STILL crosses.** The sphere / offset-cylinder crossable graze STILL crosses
    (`nearTangentCrossed ≥ 1`, `nearTangentGaps = 0`, `branchPoints = 0`) — the branch
    machinery does NOT fire on a single-branch graze.
  - **Transversal regression.** The 5 currently-passing S3 transversal fixtures trace
    bit-identically (`nt = 0`) — the branch machinery engages only at a detected branch point.
  - Full CTest green NUMSCI ON and OFF (S4-d assertions absent with NUMSCI off, like
    S2/S3/S4-c). No OCCT linked; no tolerance weakened.
- **Sim native-vs-OCCT (booted simulator).** Extend `scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm` (or a new `scripts/run-sim-native-ssi-s4d.sh`):
  add the Steinmetz fixture and assert it is now FULLY traced natively — all arms, both branch
  points localized + routed — matching OCCT `IntPatch` / `GeomAPI_IntSS`: every sampled native
  node on the OCCT locus ≤ `onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces
  ≤ `onSurfTol`; the native arm / loop count reconciles with the OCCT branch count (two
  crossing ellipses); the localized native branch points match the OCCT branch points to
  `tol`. Also assert the isolated tangent-point pair STILL ends (native reports the
  `TangentPoint`; no arms; OCCT's tangent restriction is the oracle) and the S4-c graze STILL
  crosses. Report per-pair the resolved (localized + routed) vs still-deferred count. Run via
  `xcrun simctl spawn <booted udid>`; `xcrun simctl list devices booted`.

## Decisions

- **Reuse the S4-c collapse + flip detector as the branch trigger.** The branch point is
  ALREADY detected — S4-c defers exactly there. S4-d hooks the same seam so there is no second
  detector to keep consistent, and the transversal / graze paths are untouched by
  construction.
- **The tangent-cone quadratic's discriminant IS the point-vs-branch decision.** Two distinct
  real roots ⇔ indefinite relative second form ⇔ transversal self-crossing (arms); no real
  distinct roots ⇔ definite form ⇔ isolated `TangentPoint` (END). This is the same
  differential-geometry invariant S4-b uses to classify `TangentPoint`, so the "isolated
  tangent must still end" guarantee is enforced by the SAME sign test — not a separate
  heuristic.
- **Localize by minimizing the transversality sine, re-project on both surfaces.** The branch
  point is where the locus is singular ⇔ `‖nA × nB‖` is minimal (≈ 0). Minimizing the
  substrate-computed sine along the bracketed approach, then re-projecting on both surfaces,
  reuses the existing corrector — no bespoke solver — and yields a B verified on both
  surfaces.
- **Route arms with the S4-c fixed-plane corrector, then hand to the normal S3 march.** The
  first step off B is exactly the S4-c well-posed-as-sine→0 corrector; once clear of the core
  the arm is an ordinary transversal branch marched by the unchanged S3 loop. No new marching
  path.
- **Emit ONLY verified arms; otherwise DROP the arm / DEFER the branch.** An arm whose first
  step will not verify on both surfaces, or a branch whose B will not localize or whose
  tangent cone is not genuinely real, is DROPPED / DEFERRED — never a fabricated arm or point.
  This closes the cardinal S4-d risk (a fake arm) by construction.
- **`enableBranchPoints` master switch + branch-point budget.** The whole S4-d path is behind
  a default-on switch (off → exactly the S4-c defer) and a total branch-point budget so a
  pathological recursion cannot spin; both additive, neither weakens a tolerance.
- **Additive, ABI-stable, tessellator-untouched.** New `TraceSet.branchPoints` /
  `branchNodes` + `BranchNode` + `MarchOptions` branch knobs (sentinel-resolved); no `cc_*`
  change; `src/native/tessellate` untouched; the S4-d code under `CYBERCAD_HAS_NUMSCI` like
  S2/S3/S4-c.

## Risks / Trade-offs

- **False branch point (a graze localized as a self-crossing).** Mitigation: require ≥ 2
  DISTINCT REAL tangent-cone roots (a genuine indefinite form) AND B re-projected on both
  surfaces ≤ `onSurfTol` before ANY arm is routed; the sim on-locus / branch-point-match gate
  catches a bad localization. A suspected branch failing either check DEFERS. Accepted.
- **Spurious enumerated arm (noisy second-order form at near-zero sine).** Mitigation: route
  an arm ONLY if its first fixed-plane step lands on the intersection ≤ `onSurfTol` and the
  march makes real progress; `retraces` folds an arm coinciding with a real one. An
  unverifiable arm is DROPPED. Accepted.
- **`TangentPoint` misread as a branch point (fabricated arms).** Mitigation: the S4-b
  classifier gate (definite relative II ⇒ `TangentPoint` ⇒ END) AND the no-real-distinct-roots
  reject in the tangent cone — the SAME sign invariant, applied twice. The two-sphere isolated
  tangent is the "must-still-end" pin. Accepted.
- **Transversal / S4-c-graze regression.** Any change near the marcher risks perturbing the
  passing S3 traces or the S4-c crossing. Mitigation: the branch machinery engages ONLY at a
  detected branch point (`enableBranchPoints` + the collapse+flip trigger); the S3 corrector /
  deflection controller and the S4-c crossing driver are bit-identical; the 5 transversal
  pairs + the S4-c graze are pinned green. Accepted.
- **Arm routing hits ANOTHER branch point (recursion / double-count).** The Steinmetz arms run
  saddle-to-saddle, so routing one arm reaches the other branch point. Mitigation: the
  branch-point budget bounds recursion, and the `branchMergeFrac` merge + `retraces` dedup fold
  the same saddle localized from two arms into ONE `BranchNode` and the same arc traced from
  two ends into ONE WLine. Accepted.
