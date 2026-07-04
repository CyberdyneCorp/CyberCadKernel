# Design — add-native-ssi-s4c-near-tangent-marching (SSI Stage S4-c, first slice)

## Context

The native SSI stack is S1 analytic → S2 seeding → S3 marching, hardened by the S4-a/S4-b
CLASSIFICATION layers (typed `CoincidentRegion` + `TangentContact`). S4-b explicitly did
NOT march through a tangency: a `NearTangentTransversal` was classified and handed on.
**S4-c is the hard core of the moat** — actually MARCHING THROUGH a near-tangent region
when the curve genuinely continues, and emitting the full curve.

The S3 marcher (`src/native/ssi/marching.cpp`) stops at a near-tangent region for one
concrete reason: its corrector's along-tangent advance constraint degenerates.

- `intersectionTangent(A,B,s)`: `t = normalize(nₐ × n_b)`; `sine = ‖nₐ × n_b‖` is the
  transversality witness. `marchDir` flags `DirEnd::NearTangent` and stops the walk the
  moment `sine < tangentSinTol` (default 1e-3) or `tryStep` cannot converge at `minStep`.
- `correct(A,B,prev,tdir,h,guess,onSurfTol)`: the 4-residual solve
  `r₀..₂ = A.point − B.point` (land on the intersection) plus
  `r₃ = dot(A.point − Pprev, t) − h` (advance ≈ h along `t`). As `sine → 0`, `t`
  ill-conditions, `r₃` degenerates, and `nn::least_squares` diverges — the root cause of
  the near-tangent stop.

### Diagnosis (the "before", confirmed on the current marcher)

Host build, `CYBERCAD_HAS_NUMSCI` ON, `trace_intersection` on elementary adapters:

| Fixture | S3 result today | Verdict |
|---|---|---|
| Equal cylinders, axes crossing at 90° (R=1) — Steinmetz | `NearTangent`, 167 pts, `stopReason = NearTangentTransversal` (sine ≈ 4.5e-4), `nearTangentGaps = 1` | **truncates** — curve continues |
| Equal cylinders, axes crossing at 45°/60°/80° | `NearTangent`, `nearTangentGaps = 1`, sine ≈ 1e-4…1e-3 | **truncates** — curve continues |
| Equal cylinders, tiny axis offset dy = 0.02…0.2 | `Closed`, 500–600 pts, `nearTangentGaps = 0` | traces fully (control) |
| Two spheres at d = R₁+R₂ (external tangent) | `deferredTangent = 1`, no curve | **genuine tangency — must stay stopped** |
| Sphere tangent to a cylinder | `deferredTangent = 1`, no curve | **genuine tangency — must stay stopped** |

The tiny-offset control confirms the truncating fixture's curve is genuinely continuous
through the grazing region (it becomes a smooth closed loop the instant the exact tangency
is perturbed). The full curve is analytically known (two Steinmetz ellipses), so OCCT
`IntPatch` parity is well-defined.

**Honest nuance (scope-setting).** At an EXACT crossing (the equal-cylinder saddle
`(0, ±1, 0)`) the two intersection branches TOUCH — that is a branch crossing, which is
S4-d topology. S4-c's FIRST slice targets the **single-branch graze**: a near-tangent
region the curve passes THROUGH on the SAME branch. Where the stall is a branch crossing
(multiple valid tangent directions / multiple nearby seeds), S4-c STILL STOPS + defers
(S4-d owns it). The crossable gate below encodes exactly this distinction so we never
fabricate a curve across a branch point.

The method is clean-room; OCCT (`IntWalk_PWalking` / `IntPatch` / `GeomAPI_IntSS`) is the
verification ORACLE only — the fixed-plane corrector, curvature predictor, and step band
are re-derived, never copied.

## Goals / Non-Goals

**Goals**
- (S4-c-1) A robust corrector that stays well-posed as `sine → 0`: a FIXED-PLANE CUT
  perpendicular to the last-good tangent `t★` at arc-distance `h`, solved as a constrained
  residual-minimization over `nn::least_squares` / `nn::minimize`, NOT the along-`t` 2×2
  Newton that diverges.
- (S4-c-2) A curvature-aware predictor that bends the `P + h·t★` guess by the discrete
  curvature of the last two accepted nodes, so the corrector starts in-basin across the
  band.
- (S4-c-3) Step control that shrinks `h` through the low-sine band (bounded by `minStep`)
  and resumes normal growth once `sine` recovers.
- (S4-c-4) A CROSSABLE GATE that attempts a crossing ONLY for a `NearTangentTransversal`
  single-branch graze, VERIFIES the crossing on both surfaces, and otherwise STOPS +
  classifies + defers.
- Emit the FULL curve for the confirmed truncating fixture (`nearTangentGaps → 0`),
  verified vs OCCT (on-surface, on-curve, arc length, closed/branch counts).

**Non-Goals (deferred — never faked here)**
- **S4-d: branch points / self-intersections of the intersection locus** (multiple curve
  branches meeting at a point — the equal-cylinder saddle crossing). Detected and DEFERRED,
  never crossed by this slice.
- **S4-e: singular points** (a surface's own degeneracy — cone apex, sphere pole).
- **S4-f: self-intersection resolution / global topology repair.**
- **Coincident-region marching** (a whole tangent SEAM where the surfaces coincide over a
  stretch, not just graze at a point) — `TangentCurve` / `CoincidentRegion`, handed on.
- **Any change to `src/native/tessellate`, the `cc_*` ABI, or the S3 transversal trace
  outside the low-sine band.** The transversal corrector + deflection controller are
  bit-identical outside the band.
- **Weakening `tangentSinTol` or any tolerance to "pass".** A curve that still truncates is
  an honestly reported gap, not a hidden failure.

## Module shape

```
src/native/ssi/marching.h           [extend — additive result fields + options]
  struct MarchOptions {
    // ... existing fields unchanged ...
    double bandEnterSin = -1.0;  // sine below which the low-sine band is entered (≤0 → k·tangentSinTol, k≈3)
    double bandExitSin  = -1.0;  // sine above which normal stepping resumes  (≤0 → k'·tangentSinTol, k'≈5)
    int    crossMaxSteps = 64;   // max small steps spent inside one low-sine band before deferring
  }
  struct WLine {
    // ... existing fields unchanged ...
    int    nearTangentCrossed = 0;   // near-tangent regions this branch MARCHED THROUGH (S4-c)
    double crossMaxResidual  = 0.0;  // worst ‖A.point − B.point‖ across a crossed band (≤ onSurfTol)
  }
  struct TraceSet {
    // ... existing fields unchanged ...
    int nearTangentCrossed = 0;      // total near-tangent regions crossed (completed arcs, S4-c)
    // nearTangentGaps keeps counting ONLY the regions that could NOT be crossed (still deferred)
  }

src/native/ssi/marching.cpp         [extend — the S4-c crossing machinery, CYBERCAD_HAS_NUMSCI]
  // fixedPlaneCorrect(A,B, prev, tStar, h, guess, onSurfTol)
  //   constrained min ‖A.point − B.point‖ s.t. dot(A.point − Pprev, tStar) = h
  //   (well-posed as sine→0 — tStar is the LAST-GOOD tangent, not the local one)
  // curvaturePredict(n_{k-1}, n_k, h, tStar) → bent guess P + h·tStar + ½·h²·κ̂
  // crossNearTangent(A,B, cur, sign, t, band...) → optional<crossing arc>
  //   drives the low-sine band: gate (S4-b) → shrink h → fixed-plane correct →
  //   verify on both surfaces → advance until sine recovers, else defer
  // marchDir: on the NearTangent stall, call crossNearTangent; on success splice the
  //   crossing arc and CONTINUE the walk; on defer, stop exactly as S3 does today.
```

`src/native/**` stays OCCT-free. The new machinery lives entirely in `marching.cpp` under
`CYBERCAD_HAS_NUMSCI` (it calls `nn::least_squares` / `nn::minimize`). It reuses the S4-b
`classify_tangent_contact_seeded` (`tangent_seeded.h`) for the crossable gate — no new
classifier. No new substrate routine, no new hand-tuned constant beyond the band
thresholds (which are `tangentSinTol`-derived, documented, and never weaken a tolerance).

## S4-c-1 — Robust corrector: the fixed-plane cut

### Why the along-`t` constraint fails

The S3 corrector pins the along-curve DOF with `r₃ = dot(A.point − Pprev, t) − h`, where
`t = normalize(nₐ × n_b)` is the LOCAL intersection tangent. As `sine = ‖nₐ × n_b‖ → 0`,
`t`'s direction is dominated by numerical noise in the two nearly-parallel normals, so
`r₃`'s gradient collapses and `nn::least_squares` diverges (the observed near-tangent stop).

### The fixed-plane cut

Replace the LOCAL tangent in the advance constraint with the **last-good tangent** `t★` —
the intersection tangent at the last node BEFORE the sine dropped into the band (where
`sine ≥ bandExitSin`, well-conditioned). Pin the new node to the hyperplane perpendicular
to `t★` at arc-distance `h` from the previous node `Pprev`:

```
minimize   f(u₁,v₁,u₂,v₂) = ‖A.point(u₁,v₁) − B.point(u₂,v₂)‖²
subject to c(u₁,v₁,…)      = dot(A.point(u₁,v₁) − Pprev, t★) − h = 0
```

Realized over the substrate as a 4-residual least-squares (like the S3 corrector but with
`t★` fixed) — `r₀..₂ = A.point − B.point`, `r₃ = dot(A.point − Pprev, t★) − h` — or, when
that is still ill-conditioned, as `nn::minimize` of `‖A.point − B.point‖` with the cut as
an equality penalty. The KEY property: the intersection curve crosses the fixed cut plane
`{X : dot(X − Pprev, t★) = h}` TRANSVERSALLY as long as the curve's true tangent has a
component along `t★` — which holds through a single-branch graze (the curve does not
reverse direction; it continues roughly along `t★`). So the constraint's gradient stays
bounded away from zero and the solve converges even where the LOCAL surface tangent `t`
degenerates. `t★` is refreshed to the local `t` as soon as `sine` recovers above
`bandExitSin`.

Outside the low-sine band the corrector is UNCHANGED (the along-local-`t` constraint, the
existing accept test, the deflection controller) — so every currently passing transversal
S3 trace is bit-identical.

## S4-c-2 — Curvature-aware predictor

The S3 predictor is first-order: `P_guess = P + h·t`. Across a grazing region the curve
bends sharply (the tangent swings quickly while `sine` is small), so a straight guess lands
far from the curve and the corrector — even the fixed-plane one — starts out of basin. Use
the discrete curvature from the last two accepted tangents `t_{k-1}, t_k`:

```
κ̂ · N̂ ≈ (t_k − t_{k-1}) / Δs_k              (finite-difference curvature vector)
P_guess = P + h·t★ + ½ · h² · (κ̂ · N̂)         (second-order, curvature-bent)
```

with `t★` the last-good tangent for the direction and `Δs_k` the last accepted arc step.
The param guesses on each surface are advanced through the existing `advanceParams` 2×2
tangent-plane solve for the bent world step. Falls back to the first-order guess when fewer
than two prior nodes exist, `Δs_k` is degenerate, or `κ̂` is not finite. This only sets the
corrector's starting point — the fixed-plane corrector still owns correctness; a bad
predictor costs iterations, never a wrong point.

## S4-c-3 — Step control through the low-sine band

Two `tangentSinTol`-derived thresholds bracket the band (defaults, sentinel-resolved):

- `bandEnterSin ≈ 3 · tangentSinTol` — below this, ENTER the band: stop growing `h`, and
  shrink `h ← max(minStep, 0.5·h)` each step so the minimum-clearance region is crossed in
  small, well-conditioned steps.
- `bandExitSin ≈ 5 · tangentSinTol` — once the corrected node's `sine` recovers above this,
  EXIT the band: refresh `t★` to the local tangent and resume the normal deflection-driven
  growth (`h ← min(1.5·h, maxStep)` on a cheap step).

Entering the band NO LONGER immediately truncates (as S3 does at `sine < tangentSinTol`);
instead it triggers the crossing attempt. The `minStep` floor is preserved: if `h` reaches
`minStep` and the fixed-plane corrector still cannot converge (a DEEP band — the surfaces
are near-coincident over a stretch, not a clean graze), the crossing is abandoned and the
march STOPS + defers exactly as S3 does today. `crossMaxSteps` bounds the small-step budget
inside one band so a pathological band cannot spin forever.

## S4-c-4 — The crossable gate (the honesty core)

A crossing is attempted ONLY when ALL hold; otherwise STOP + classify + defer:

1. **`NearTangentTransversal` classification.** At the stall, call the S4-b seeded
   classifier `classify_tangent_contact_seeded(A,B,…,sineAtStop,scale)`. Cross ONLY if it
   returns `NearTangentTransversal` (relative second fundamental form INDEFINITE — the
   surfaces graze but CROSS). `TangentPoint` (sign-definite — isolated contact, curve does
   NOT continue), `TangentCurve` (rank-1 — a tangent SEAM), or `Undecided` (within the
   curvature-noise band) ⇒ STOP + record the typed `stopReason` + defer → OCCT, EXACTLY as
   S4-b does today. This is the "genuine tangency stays stopped" guarantee.
2. **Single-branch graze, not a branch crossing.** Reject a branch crossing (S4-d): if the
   stall region has MULTIPLE valid intersection-tangent directions (the cross product
   `nₐ × n_b` is not just short but rank-deficient in a way that admits two curve branches),
   or MULTIPLE distinct seeds/near-curve points cluster at the stall, the region is a branch
   point ⇒ STOP + defer (S4-d owns it). Only a single continuing branch is crossed.
3. **Convergent fixed-plane corrector at ≥ `minStep`.** The fixed-plane cut must converge
   on both surfaces at some `h ≥ minStep` within `crossMaxSteps`; else defer.
4. **VERIFIED crossing.** Every node emitted across the band must lie on BOTH surfaces
   ≤ `onSurfTol` AND advance monotonically along `t★` (the along-`t★` coordinate strictly
   increases — the walk does not stall or reverse). The band is considered CROSSED only once
   `sine` recovers above `bandExitSin` on the far side with a well-conditioned local tangent
   that is CONSISTENT with `t★` (`dot(t_local, t★) > 0`). If verification fails at any node,
   the crossing arc is DISCARDED and the march truncates at the band entry exactly as S3 —
   never a partially-fabricated arc.

On a successful, verified crossing: splice the crossing arc into the WLine, increment
`WLine.nearTangentCrossed`, record `crossMaxResidual`, refresh `t★`, and CONTINUE the walk
(the far side may close a loop or reach a boundary normally). The region is NOT counted in
`nearTangentGaps` (it is a completed arc). On a defer: `nearTangentGaps` still counts it and
`stopReason` carries the S4-b type, exactly as today.

## Crossable-vs-deferred scope (honest)

| Configuration at the stall | S4-c action | counted as |
|---|---|---|
| `NearTangentTransversal`, single-branch graze, corrector converges, crossing verified | **MARCH THROUGH**, splice arc, continue | `nearTangentCrossed` (completed) |
| `NearTangentTransversal`, but corrector will not converge at `minStep` (deep band) | STOP + defer → OCCT | `nearTangentGaps` |
| `NearTangentTransversal`, but crossing fails on-surface / monotone verification | DISCARD arc, STOP + defer → OCCT | `nearTangentGaps` |
| Branch crossing (multiple tangent directions / clustered seeds) — S4-d | STOP + defer → OCCT | `nearTangentGaps` |
| `TangentPoint` (sign-definite relative II) — isolated genuine tangency | STOP + classify + defer | `nearTangentGaps` |
| `TangentCurve` (rank-1) — tangent seam | STOP + classify + defer | `nearTangentGaps` |
| `Undecided` (within curvature-noise band) | STOP + classify + defer → OCCT | `nearTangentGaps` |
| Transversal region (`sine ≥ bandEnterSin`) | normal S3 march (unchanged) | traced |

## Verification model (two gates)

- **Host (no OCCT).** Extend `tests/native/test_native_ssi_marching.cpp` (or a new
  `tests/native/test_native_ssi_s4c_near_tangent.cpp`), all under `CYBERCAD_HAS_NUMSCI`:
  - **Crossable fixture now fully traced.** The equal crossing-cylinder Steinmetz fixture
    (R=1, axes crossing at 90°) that S3 today truncates (`nearTangentGaps = 1`, ~167 pts) is
    now FULLY traced: `nearTangentGaps = 0`, `nearTangentCrossed ≥ 1`, the curve complete
    (both ellipse arcs), every emitted node on BOTH surfaces ≤ `onSurfTol`, and the trace
    length matches the analytic Steinmetz-ellipse length within the deflection tolerance.
    (If the 90° saddle is classified as a branch crossing rather than a single-branch graze,
    use a tilted-axis / small-offset variant whose stall IS a single-branch graze — the
    diagnose phase enumerates candidates; the fixture chosen is one whose crossable gate
    genuinely fires, honestly documented.)
  - **Genuine tangency STILL stops.** Two spheres at `d = R₁+R₂` and a sphere tangent to a
    cylinder STILL stop + classify (`TangentPoint` / `TangentCurve`), NO curve fabricated
    across the tangency; `nearTangentCrossed = 0` for these; the deferred count is unchanged.
  - **Branch crossing STILL stops.** The equal-cylinder exact saddle (if classified as a
    branch crossing) is NOT crossed — deferred, `nearTangentGaps ≥ 1`.
  - **Transversal regression.** Every currently passing S3 transversal fixture traces
    bit-identically (the corrector/predictor/step are unchanged outside the band).
  - Full CTest green NUMSCI ON and OFF (S4-c assertions absent with NUMSCI off, like S2/S3).
    No OCCT linked; no tolerance weakened.
- **Sim native-vs-OCCT (booted simulator).** Extend
  `scripts/run-sim-native-ssi-marching.sh` + `tests/sim/native_ssi_marching_parity.mm`:
  add the crossable near-tangent fixture and assert it is now FULLY traced — native
  `nearTangentGaps → 0`, and the full native curve matches OCCT `IntPatch` / `GeomAPI_IntSS`:
  every native WLine point on the OCCT curve ≤ `onCurveTol` (`GeomAPI_ProjectPointOnCurve`),
  every point on BOTH surfaces ≤ `onSurfTol`, native arc length ≈ OCCT arc length within the
  deflection/step tolerance (`GCPnts_AbscissaPoint`), and closed/branch counts agree. Also
  assert a GENUINE-tangency pair STILL stops (native reports the tangency; it is NOT crossed;
  OCCT's tangent restriction is the oracle). Report per-pair the crossed vs still-deferred
  count. Run via `xcrun simctl spawn <booted udid>`; `xcrun simctl list devices booted`.

## Decisions

- **Fixed-plane cut on the LAST-GOOD tangent, not the local one.** The whole failure is
  that the LOCAL tangent ill-conditions as `sine → 0`. `t★` from before the band is
  well-conditioned and the curve crosses its perpendicular plane transversally through a
  single-branch graze, so the constraint stays well-posed exactly where the along-local-`t`
  one collapses. This is the crux of S4-c-1.
- **Constrained minimization over the substrate, not a bespoke Newton.** Reuse
  `nn::least_squares` / `nn::minimize` (the S2/S3 substrate) so there is no new numerical
  code path to harden; the constraint change is the only difference from the S3 corrector.
- **Curvature predictor sets the START only.** Correctness is owned by the corrector +
  the on-surface/monotone verification; the predictor just keeps the corrector in-basin
  across the sharp bend, so a crude curvature estimate is safe.
- **Crossable gate reuses S4-b classification unchanged.** The point/curve/cross/undecided
  distinction is already the S4-b `TangentContact`; S4-c consumes it and adds ONLY the
  branch-crossing rejection + on-surface/monotone verification. No re-classification, no
  new tolerance for the type decision.
- **Cross ONLY what verifies; otherwise defer.** A crossing that does not stay on both
  surfaces and advance monotonically is discarded — the march truncates honestly rather
  than emit a fabricated arc. This is the cardinal S4-c risk (a bogus curve across a
  tangency) closed by construction.
- **Band thresholds are `tangentSinTol`-derived, additive, and never weaken a tolerance.**
  `bandEnterSin`/`bandExitSin` bracket the SAME `tangentSinTol` gate S3 uses; entering the
  band triggers a crossing ATTEMPT, not a looser accept. `onSurfTol`, `minStep`,
  `maxDeflection` are all unchanged. A curve that still truncates is a reported gap.
- **Additive, ABI-stable, tessellator-untouched.** New `MarchOptions` band knobs (sentinel-
  resolved) + additive `WLine` / `TraceSet` crossing counts; no `cc_*` change;
  `src/native/tessellate` untouched; the S4-c code under `CYBERCAD_HAS_NUMSCI` like S2/S3.

## Risks / Trade-offs

- **Deep low-sine band (near-coincident stretch).** The fixed-plane corrector can still
  fail to converge where the surfaces are nearly coincident over an arc (not a clean
  point-graze). Mitigation: `minStep` floor + `crossMaxSteps` budget → defer → OCCT
  (report, never fabricate). The sim on-surface/arc-length gate catches any bad crossing.
  Accepted — a correct truncation is honest.
- **Branch crossing misread as a single-branch graze (S4-d leakage).** If the branch-
  crossing detection misses a branch point, the marcher could try to cross a saddle.
  Mitigation: (i) the branch-crossing rejection in the gate (multiple tangent directions /
  clustered seeds), (ii) the monotone-along-`t★` + consistent-far-side-tangent verification
  (a branch crossing flips or forks the tangent, failing the check → arc discarded → defer).
  The exact equal-cylinder saddle is USED as a "must-still-stop" fixture to pin this.
  Accepted.
- **Transversal-trace regression.** Any change to the corrector/predictor/step risks
  perturbing the passing S3 traces. Mitigation: the along-local-`t` corrector, the accept
  test, and the deflection controller are bit-identical OUTSIDE the band; the S3 host + sim
  fixtures are pinned green. Accepted.
- **Curvature predictor overshoot on a high-curvature bend.** A large `½h²κ̂` term could
  land the guess past the curve. Mitigation: the predictor only seeds the corrector (which
  re-projects), `h` is already shrunk inside the band, and the on-surface verification
  rejects a bad node. Accepted.
