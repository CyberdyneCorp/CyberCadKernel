# Design — add-native-ssi-marching (SSI Stage S3)

## Context

`SSI-ROADMAP.md` stages SSI analytic-first. **S1** (shipped) returns closed-form
conics for the elementary-pair family and an honest `NotAnalytic` for everything
else. **S2** (shipped) supplies a **`SeedSet`** — ≥1 seed per distinct **transversal**
intersection branch, each with its `(u1,v1,u2,v2)` and a proof it lies on both
surfaces. **S3** is the next stage: from each seed, **trace the full intersection
curve** of the branch and fit a B-spline through it, producing one WLine per seed.

The gap S3 fills is exactly what the roadmap's substrate eval frames: local Newton/LM
**re-projection** onto both surfaces is *provided* by `native-numerics` (converges
1e-14…1e-6) — that is the S3 **corrector** — but a *global tracer* that predicts the
next point, adapts its step, and knows when to stop is not; S3 builds it on top. This
is OCCT's `IntWalk` / `IntPatch` `WLine`, re-derived on our substrate.

The method is **locked to CLEAN-ROOM** predictor-corrector marching derived from
`SSI-ROADMAP.md`, using OCCT (`IntWalk`, `IntPatch` `WLine`/`ALine`,
`GeomAPI_IntSS`) strictly as a verification **oracle** for curve parity, never
copied.

SSI is an **internal** capability (consumed by S5, not the `cc_*` ABI), verified at
the `cybercad::native::ssi` C++ boundary — the same discipline as S1 / S2 and
native-math parity — with **no ABI change**.

## Goals / Non-Goals

**Goals**
- From each S2 `Seed`, trace the full intersection curve of its **transversal**
  branch: predictor (`t = normalize(n₁ × n₂)` step) + corrector (substrate
  re-projection onto both surfaces) + adaptive step + termination.
- Terminate on **loop closure** (return near start) or **boundary exit**; march both
  directions for an open branch.
- **Dedup** seeds that retrace the same branch → one WLine per distinct branch.
- **Fit a B-spline** through the traced polyline (`native-math`), self-verified on
  both surfaces before return.
- **Curve parity** vs OCCT `IntPatch` / `GeomAPI_IntSS` (branch count / length /
  sampled distance) on the sim, reported.

**Non-Goals (deferred — never faked here)**
- **Near-tangent / coincident / degenerate** marching (`n₁ × n₂ → 0`: the corrector
  ill-conditions) → **S4**: trace up to the tangent, flag the remainder as an S4
  gap, no fabricated point.
- **Branch-point splitting** (a march reaching a singular crossing of two branches) →
  **S4**.
- **Self-intersection resolution** (a curve that crosses itself) → **S4**.
- The **seeding** itself (finding the start points) — that is S2, S3's input.
- Any `cc_*` facade entry point or ABI change.

## Module shape

```
src/native/ssi/
  marching.h / marching.cpp   // SSI Stage S3: predict → correct → step → terminate → fit   [CYBERCAD_HAS_NUMSCI]
  wline.h                     // WLine { polyline; BSpline fit; onSurfResidual; TraceStatus } + TraceSet + diagnostics
```

The tracer consumes the **same** native-math surface interface S1/S2 use — every
surface exposes `point(u,v)` / `dU` / `dV` / `normal(u,v)` — via the S2
`SurfaceEval`-style adapter, so elementary, torus, Bézier, B-spline and NURBS all
flow through the same march. It reuses `seed.h` (its `SeedSet` input), `curve.h`
(frames / result kinds), and `tolerance.h` from S1/S2, `native-numerics`
`least_squares` / `closest_point_on_surface` for the corrector, and `native-math`
`bspline` for the fit.

## Result type

```cpp
enum class TraceStatus {
  Closed,        // walk returned to its start — a closed loop
  BoundaryExit,  // walk left a surface's param range on both ends — an open branch
  NearTangent,   // walk hit ‖n₁ × n₂‖ → 0 — traced UP TO here, remainder is an S4 gap
  Failed,        // corrector could not advance from the seed at all (reported, no curve)
};

struct WLine {                     // one traced intersection branch
  std::vector<math::Point3> polyline;   // corrected march points, in order
  math::BSpline curve;                  // B-spline fitted through the polyline (Geom-quality)
  double onSurfResidual = 0.0;          // max ‖sample − surface‖ over BOTH surfaces (≤ tol)
  TraceStatus status = TraceStatus::Failed;
  int branchId = 0;                     // echoes the S2 seed branch id
};

struct TraceSet {
  std::vector<WLine> lines;      // one WLine per distinct traced transversal branch
  int tracedBranches = 0;        // WLines produced (Closed | BoundaryExit)
  int nearTangentGaps = 0;       // marches stopped at a near-tangent region → S4 (reported)
  int dedupedRetraces = 0;       // seeds whose march retraced an already-traced branch (diagnostic)
};
```

`TraceSet` is the S5 contract: S5 splits curved faces with each `WLine`.
`nearTangentGaps > 0` is the honest S4 signal — a branch was traced up to a tangent
and the remainder deferred.

## Algorithm (clean-room predictor-corrector)

### 1. Corrector (`native-numerics` re-projection)
The atom of the march. Given a predicted 3D point `Pₚ` and a parameter guess
`(u1,v1,u2,v2)`, drive the residual `r(x) = A.point(u1,v1) − B.point(u2,v2)`
(m=3, n=4) to zero with `least_squares` (LM handles the along-curve rank-deficiency,
same as S2's refine), seeded by `closest_point_on_surface` on each operand from `Pₚ`.
**Clamp** the converged params to each surface's box. The corrector **succeeds** iff
it converges with `onSurfResidual ≤ tol` AND is **transversal** (`‖n₁ × n₂‖` above
the near-tangent threshold) — else it **fails** (near-tangent / off-patch /
non-convergent), stopping the march. This is the only place a point becomes "on the
curve"; a predicted point is never accepted uncorrected.

### 2. Predictor (tangent step)
At a corrected point `P` with normals `n₁ = normalize(A.dU × A.dV)`,
`n₂ = normalize(B.dU × B.dV)`, the intersection tangent is `t = normalize(n₁ × n₂)`
(sign chosen to continue the current walk direction). Predict `Pₚ = P + h·t`. The
predicted point is only a *guess* — step 1 (corrector) is what lands it on the curve.

### 3. Adaptive step (`h`)
`h` is bounded by a scale/tol-derived `[h_min, h_max]`. **Shrink** `h` (e.g. halve)
when the corrector strains — it took many iterations, the correction distance
`‖Pₚ − P_corrected‖` was large relative to `h`, or the turn angle between successive
tangents exceeds a curvature threshold — and retry the step. **Grow** `h` (e.g. ×1.5,
capped) when several steps in a row corrected cleanly. If `h` must shrink below
`h_min` to converge, the region is near-tangent → stop (S4 gap). This keeps the
polyline dense where the curve bends and sparse where it is straight, matching the
`IntWalk` step-control role (clean-room).

### 4. March loop + termination
From the seed, walk in the `+t` direction stepping predict → correct → step-adapt,
appending each corrected point to the polyline, until one of:
- **Loop closure** — the point returns within a tol-scaled radius of the walk start
  (and enough steps have been taken to not trivially "close" on step 1) → the branch
  is a **closed loop** (`TraceStatus::Closed`); done.
- **Boundary exit** — a corrected param leaves `[u0,u1]×[v0,v1]` on either surface →
  this end of the branch runs off the patch; stop this direction. Then re-walk from
  the seed in `−t`; the branch is **open** (`TraceStatus::BoundaryExit`).
- **Near-tangent** — the corrector fails with `‖n₁ × n₂‖ → 0` (not a boundary, not a
  closure) → the branch runs into a tangent; **stop and flag** `nearTangentGaps`
  (`TraceStatus::NearTangent`), tracing only up to that point. **Never** force a
  point past it.
The loop is bounded by a max-step count (scale/tol-derived) so it always terminates
even if closure is missed. **Cognitive complexity**: the loop body is a guard-clause
sequence (correct → classify {closed | boundary | tangent | continue} → adapt); the
predictor, corrector, step-adapt and termination tests are isolated helpers (systems
band, Visitor-free).

### 5. Dedup retraced branches
Multiple S2 seeds can land on the same branch (dedup slack). Before marching a seed,
test whether its point already lies on an **already-traced** WLine (closest-point to
the traced polyline within a tol radius); if so, skip it and increment
`dedupedRetraces`. This keeps the output one WLine per distinct branch.

### 6. B-spline fit
Fit a B-spline through the ordered corrected polyline with the existing `native-math`
B-spline fitting (interpolation / least-squares fit to the points). Then **self-
verify**: sample the fitted curve and confirm every sample lies on both surfaces
within tol (`onSurfResidual`). If the fit residual exceeds tol, densify the polyline
(smaller `h`) and refit; if it still fails, the WLine is returned with its polyline
and a diagnostic rather than a bad fitted curve (never a leaky curve).

## Transversal-vs-deferred scope (honest)

| Configuration | S3 behavior |
|---|---|
| **Transversal** branch, closed loop (`n₁ × n₂ ≠ 0`, returns to start) | trace fully → `WLine{Closed}` |
| **Transversal** branch, open (exits both patch boundaries) | trace both directions → `WLine{BoundaryExit}` |
| Second seed on an already-traced branch | skip, `dedupedRetraces++` (one WLine per branch) |
| Branch **runs into a near-tangent** region (`n₁ × n₂ → 0`) | trace **up to** the tangent, then STOP → `WLine{NearTangent}` + `nearTangentGaps++` (S4 gap) |
| **Coincident / overlapping** surfaces | **deferred to S4** — no discrete curve to march; reported |
| **Branch point** (two branches cross at a singularity) | **deferred to S4** — do not attempt the split; trace up to it, flag |
| **Self-intersection** (curve crosses itself) | **deferred to S4** — reported, not resolved |
| Corrector cannot advance from the seed at all | `WLine{Failed}` (reported, no curve) — never a fabricated point |

## Verification model (two gates, per SSI-ROADMAP §Verification model)

- **Host (no OCCT) — known-shape pairs.** Construct native pairs whose intersection
  shape is known (a sphere piercing a freeform bump → a closed loop; two crossing
  cylinders → an open branch exiting the patch boundaries; a pair known to run into a
  tangent). Assert: (a) **every** sampled WLine point (and B-spline sample) lies on
  both surfaces ≤ tol; (b) a loop branch terminates `Closed` and the fitted curve is
  periodic-ish (endpoints coincide); (c) an open branch terminates `BoundaryExit`
  with endpoints on the patch boundary; (d) a near-tangent fixture traces up to the
  tangent and reports `nearTangentGaps`, emitting no point past it; (e) duplicate
  seeds dedup to one WLine. No OCCT.
- **Sim native-vs-OCCT — curve parity.** Build the same operands as OCCT
  `Geom_*Surface`, run `GeomAPI_IntSS` / `IntPatch`, and compare **branch count**,
  per-branch **curve length**, and **sampled point-to-curve distance** (native WLine
  samples projected onto OCCT's curve and vice-versa) within tol, plus each WLine's
  on-both-surfaces residual. Parity is a **reported figure** (with the near-tangent
  gap count called out), compared at the SSI C++ boundary; no `cc_*` call. Whatever
  S3 cannot trace robustly falls back to OCCT and is reported with the measured gap.

## Decisions

- **`least_squares` (LM) corrector, seeded by closest-point.** The re-projection is
  3 equations in 4 unknowns (along-curve DOF); LM's damping handles the
  rank-deficiency, exactly the substrate routine the roadmap earmarks for
  re-projection — and the same one S2's refine uses, so the corrector is a proven
  atom. Closest-point provides the per-step parameter seed.
- **Tangent predictor `t = normalize(n₁ × n₂)`.** The intersection curve's tangent is
  orthogonal to both surface normals — the exact analytic direction — so the
  predictor needs no curvature model to point the right way; the adaptive step
  handles magnitude. Degeneracy `‖n₁ × n₂‖ → 0` is precisely the near-tangent stop
  condition, so the predictor and the S4 seam share one test.
- **Adaptive step from corrector strain + turn angle.** Step control is the crux of a
  robust `WLine` (OCCT's `IntWalk` tuning) — clean-room: shrink on curvature / strain,
  grow on ease, bounded by scale/tol. A shrink below `h_min` *is* the near-tangent
  signal.
- **Stop, never force, at a tangent.** `nearTangentGaps` is data, not an error — a
  branch traced up to a tangent with the remainder deferred to S4, matching S1's
  `NotAnalytic`-is-data and S2's `deferredTangent`-is-data stance. S3 NEVER fabricates
  a point past a tangent nor claims a full trace that stopped short.
- **Self-verified B-spline fit.** A fitted curve is only returned after its samples
  are confirmed on both surfaces — the S3 instance of the roadmap's mandatory
  self-verify → OCCT-fallback discipline; S3 never emits a leaky curve.
- **Substrate-gated.** The predictor / termination / dedup are OCCT-free, but the
  corrector (re-projection) and the B-spline fit require the substrate, so the S3
  entry point is under `CYBERCAD_HAS_NUMSCI` (like the S2 seeder and
  `native-numerics` itself).

## Risks / Trade-offs

- **The near-tangent moat.** The dominant honest limit: a march into `n₁ × n₂ → 0`
  cannot continue robustly. Mitigation: trace up to it, flag `nearTangentGaps`, defer
  to S4 + OCCT fallback — the measured gap, never a faked continuation. Accepted,
  reported.
- **Off-branch corrector jump.** LM may converge onto a different intersection sheet
  when branches pass close. Mitigation: continuity guard (reject a corrected point
  whose step from the previous point far exceeds `h`) + the on-both-surfaces
  self-check; a rejected step shrinks `h` and retries, and a persistent failure stops
  the march.
- **Missed loop closure.** A closure test radius too tight (or a step overshooting the
  start) can miss closure and over-run. Mitigation: tol-scaled closure radius + max-
  step bound; an over-run that later exits a boundary is still a valid (if longer)
  trace, caught by the parity length check.
- **B-spline fit fidelity.** A sparse polyline under-fits a high-curvature branch.
  Mitigation: adaptive `h` densifies where it bends; the fit is self-verified on both
  surfaces and the polyline densified/refit if the residual exceeds tol.
- **Step-size tuning.** `h_min` / `h_max` / grow-shrink factors trade trace cost vs
  fidelity; scale/tol-derived defaults, exposed as knobs, with the sim length/distance
  parity as the fidelity check.
