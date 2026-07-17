# SSI → Curved Booleans — Implementation Sub-Roadmap

The keystone of the drop-OCCT endgame. **Surface-Surface Intersection (SSI)** is
the enabler; **general curved booleans** are the payoff (and blends + curved
wrap-emboss sit on top of them). This plan stages SSI analytic-first, each stage
verified native-vs-OCCT, with honest fallback — the same discipline every prior
native tier used.

Parent: [NATIVE-REWRITE.md](NATIVE-REWRITE.md) capability #5. Substrate eval:
[../docs/EVAL-numpp-scipp.md](../docs/EVAL-numpp-scipp.md).

## What we already have (the on-ramp — DONE)

- **Native geometry** — `src/native/math/` (plane/cylinder/cone/sphere/torus +
  Bézier/B-spline/NURBS curves & surfaces: point + dU/dV + normal).
- **Native topology + tessellation** — `src/native/{topology,tessellate}/`
  (watertight two-stage mesher, shared-edge weld).
- **Numeric substrate (#2, adopted)** — NumPP/SciPP behind
  `src/native/numerics/`: `fsolve` / `minimize`(BFGS) / `least_squares` /
  `solve` / `lstsq`, and **native closest-point / projection** (point→curve,
  point→surface) verified vs OCCT `Extrema`.
- **Boolean assembler** — `src/native/boolean/` BSP-CSG (planar polyhedra, exact)
  + the axis-aligned box∩cylinder analytic curved slice. This is what S5 extends.

## What the substrate does NOT buy (the moat — from the eval)

- Local Newton/LM **re-projection** onto both surfaces: ✅ provided (converges 1e-14…1e-6).
- **Finding** intersection points from a naive seed: ✗ (generic `fsolve` 0/7 on freeform).
- **Near-tangent / coincident** robustness: ✗ (both Newton and damped-LM break).

So the work is exactly: **seeding + marching + tangent/degeneracy robustness**,
built on the substrate. That is S2–S4 below.

## Verification model (every stage)

Two gates, as always: **host analytic** (intersection matches closed-form / all
sampled points lie on both surfaces within tol; no OCCT) + **sim native-vs-OCCT**
(vs `Geom`/`GeomAPI_IntSS` / `IntPatch` for curves; vs `BRepAlgoAPI` for the S5
boolean payoff — volume/watertight). Mandatory **self-verify → OCCT fallback**:
never emit a wrong/leaky curved result. Oracle source: `/Users/leonardoaraujo/work/OCCT/src`
(`IntPatch` Imp/Imp·Imp/Prm·Prm/Prm, `IntWalk`, `ALine`/`WLine`).

## Stages (dependency order, analytic-first)

### S1 — Analytic SSI (elementary-surface pairs) · ✅ DONE AT THE BAR
Closed-form intersection curves for elementary pairs: plane∩plane (line),
plane∩{cylinder,cone,sphere,torus} (line/conic/circle), coaxial/parallel
cylinder∩cylinder, coaxial sphere/cylinder/cone families, sphere∩sphere (circle).
**No marching** — pure closed-form math over `native-math` (generalizes the
plane∩cylinder we already shipped). Returns exact `Line`/`Circle`/`Ellipse`/
`Parabola`/`Hyperbola` `Geom`-quality curves. OCCT-free, header-only under
`src/native/ssi/`; INTERNAL (no `cc_*` entry point — parity asserted at the C++
boundary like native-math). Change `add-native-ssi-analytic` (**archived**).
- **Verify:** ✅ host analytic `test_native_ssi` (**11 cases, 0 failed**) + ✅ sim
  native-vs-OCCT `GeomAPI_IntSS` parity `run-sim-native-ssi.sh` (**18 pairs, 0
  failed**). No regressions (`run-sim-suite.sh` **221/221**).
- **Unlocks:** most CAD-primitive curved booleans (S5 restricted to elementary faces).

**Analytic-native pairs, native-vs-OCCT deltas** (onSurf = max residual of native
curve samples on both input surfaces; coin = native-vs-OCCT curve coincidence; all
at machine epsilon, well inside each pair's tol):

| Pair | native | OCCT | kind | onSurf | coin | tol |
|---|---|---|---|---|---|---|
| plane ∩ plane | 1 | 1 | Line | 0 | 0 | 1e-9 |
| plane ∩ sphere | 1 | 1 | Circle | 3.79e-15 | 3.82e-15 | 1e-9 |
| plane ⟂ cyl | 1 | 1 | Circle | 1.91e-15 | 1.91e-15 | 1e-9 |
| plane ∠ cyl | 1 | 1 | Ellipse | 1.42e-15 | 2.57e-15 | 1e-8 |
| plane ∥ cyl | 2 | 2 | Line/Line | 5.55e-17 | 0 | 1e-9 |
| plane ⟂ cone | 1 | 1 | Circle | 3.59e-15 | 3.59e-15 | 1e-7 |
| plane ∠ cone | 1 | 1 | Ellipse | 2.44e-15 | 5.37e-15 | 1e-6 |
| plane ∥ gen cone | 1 | 1 | Parabola | 2.03e-15 | 9.74e-16 | 1e-6 |
| plane steep cone | 2 | 2 | Hyperbola×2 | 5.61e-16 | 4.45e-16 | 1e-6 |
| plane ⟂ torus | 2 | 2 | Circle/Circle | 2.84e-15 | 2.84e-15 | 1e-9 |
| plane ∋ axis torus | 2 | 2 | Circle/Circle | 9.93e-16 | 1.67e-15 | 1e-8 |
| sphere ∩ sphere | 1 | 2 | Circle (OCCT arc-splits) | 4.12e-15 | 3.82e-15 | 1e-9 |
| coaxial sphere ∩ cyl | 2 | 2 | Circle/Circle | 1.88e-15 | 2.39e-15 | 1e-9 |
| coaxial sphere ∩ cone | 2 | 3 | Circle/Circle (OCCT arc-split) | 3.14e-15 | 2.78e-15 | 1e-7 |
| coaxial cyl ∩ cone | 2 | 3 | Circle/Circle (OCCT arc-split) | 1.79e-15 | 1.52e-15 | 1e-7 |
| parallel cyl ∩ cyl | 2 | 2 | Line/Line | 1.26e-15 | 0 | 1e-9 |
| coaxial cyl ∩ cyl | 0 | 0 | coincident (detected) | 0 | 0 | 1e-9 |

Curve-count deltas (sphere∩sphere, coaxial sphere∩cone/cyl, coaxial cyl∩cone) are
OCCT arc-splitting the SAME conic — curve TYPES match on every analytic pair.

**Deferred at S1 (honest `NotAnalytic`, verified — not faked):** **skew cyl∩cyl**
(native `NotAnalytic`; OCCT emits 7 Ellipse curves — general skew cyl/cyl is a
planar quartic, no degree-≤2 closed-form reduction), and by the same rule general
cone∩cone, non-coaxial cone∩cyl / sphere∩cyl / sphere∩cone, oblique plane∩torus
(spiric quartic), torus∩curved, and all freeform pairs. These route to S2/S3 below.

### S2 — Subdivision seeding · ✅ DONE AT THE BAR (transversal)
Find ≥1 seed point per **transversal** intersection branch for the **freeform**
(NURBS/Bézier/B-spline) and **non-closed-form quadric** pairs S1 defers: recursive
patch-AABB-overlap subdivision → candidate regions → refine to a point with
`least_squares(S1(u1,v1) − S2(u2,v2) = 0)` (substrate) → 3D/param dedup to ~one seed
per branch. Native, OCCT-free (`cybercad::native::ssi`), refine under
`CYBERCAD_HAS_NUMSCI`; INTERNAL (no `cc_*`). Change `add-native-ssi-seeding`.
- **Module:** `src/native/ssi/{seed.h,patch_bounds.h,seeding.h,seeding.cpp}`.
  Per-patch AABB = **control-net convex hull ∩ sampled-with-Lipschitz-margin** for
  freeform (both sound; the sampled bound guarantees the box shrinks under
  subdivision even for a single-span Bézier whose hull does not), **sampled+margin**
  for elementary+torus. Dedup is **topological** (union candidate regions adjacent in
  parameter space on both surfaces, periodic-seam aware) — scale-free and immune to
  the along-branch metric-gap problem — and clusters BEFORE the refine so
  `least_squares` runs ≈ once per branch, not per candidate.
- **Verify:** ✅ host `test_native_ssi_seeding` (**6 cases, 0 failed** — skew cyl→2,
  crossing spheres→1, sphere∩Bézier-bump→1, parallel planes→0, **tangent spheres →
  `deferredTangent`, no faked seed**, deeper resolution recovers a small loop; NUMSCI
  OFF CTest **23/23** with the NUMSCI-gated tests correctly ABSENT, NUMSCI ON CTest
  **25/25**) + ✅ sim native-vs-OCCT recall `native_ssi_seeding_recall.mm` (**3 pairs,
  recall 1.00**, tangent = 0 everywhere, max per-seed on-both-surfaces residual
  **3.51e-16** via `GeomAPI_ProjectPointOnSurf::LowerDistance`, well under the 1e-6
  `onSurfTol`; OCCT arc-splits the same loci — NbLines 3/2/2 ≥ native branch count 2/1/1,
  exactly as at S1). No regressions (`run-sim-suite.sh` **221/221**, xcframework rebuilt
  with `seeding.cpp`).
- **Honest scope / risk:** TRANSVERSAL only. **Near-tangent / coincident / degenerate**
  seeding ill-conditions the refine → **deferred to S4** (counted in
  `SeedSet.deferredTangent`, reported not faked). Completeness is a **measured recall**
  figure, not a blind 100%: too-shallow subdivision can miss a small loop (the
  acknowledged failure mode); `minPatchFrac` (default 1/32) is the resolution/recall
  knob — deeper recovers smaller loops at more cost.
- **Unlocks:** S3 marching — the `SeedSet` is its input contract (one WLine per seed).

### S3 — Marching-line tracer (WLine) · ✅ DONE AT THE BAR (transversal)
From each seed, walk the intersection curve: tangent = normalize(n₁×n₂), adaptive
step, **re-project** onto both surfaces via the substrate (Newton/LM), until the
curve closes or exits a boundary; fit a B-spline through the polyline. This is
OCCT's `IntWalk`/`WLine`, on our substrate. Native, OCCT-free
(`cybercad::native::ssi`); corrector / adaptive step / B-spline fit under
`CYBERCAD_HAS_NUMSCI` (empty TU with NUMSCI off); INTERNAL (no `cc_*`). Consumes the
S2 `SeedSet`, produces one `WLine` per transversal branch (`Closed`/`BoundaryExit`)
+ a `TraceSet` for S5. Change `add-native-ssi-marching` (**archived**).
- **Module:** `src/native/ssi/{marching.h,marching.cpp}` (result types + tracer in one
  OCCT-free header; `native_ssi.h` includes it).
- **Verify:** ✅ host known-shape `test_native_ssi_marching` (**7 cases, 0 failed** —
  crossing spheres / plane∩sphere / skew-cyl loops / sphere∩Bézier bump → Closed;
  ramp B-spline∩plane → BoundaryExit open segment; tangent spheres → no curve
  (deferred, not faked); duplicate seed → 1 WLine; every node on both surfaces < 1e-6,
  fit error < 1e-3; NUMSCI OFF CTest **23/23** S3 tests correctly ABSENT, NUMSCI ON
  CTest **26/26**) + ✅ sim native-vs-OCCT curve parity `native_ssi_marching_parity.mm`
  (**5 pairs, 9 branches, 0 failed**; `IntPatch`/`GeomAPI_IntSS` oracle). No regressions
  (`run-sim-suite.sh` **221/221**).

**Marching-native pairs, native-vs-OCCT deltas** (all transversal, all FULLY TRACED,
0 near-tangent-truncated → deferred to S4; onCurve = max native-sample distance to the
OCCT curve, onSurf = max residual on both input surfaces, lenΔ = |nat−occt| arc length):

| Pair | branches nat/occt | closed | onCurve | onSurf | lenΔ (nat / occt) | nt | seeds |
|---|---|---|---|---|---|---|---|
| bspline ∩ bspline | 1/1 | 1/1 | 1.86e-07 | 2.71e-08 | 4.35e-06 (2.8171 / 2.8171) | 0 | 1 |
| bspline ∩ plane | 4/4 | 0/0 | 5.75e-09 | 1.41e-11 | 2.28e-03 (0.6917 / 0.6933) | 0 | 4 |
| skew cyl unequal | 2/2 | 2/2 | 1.60e-06 | 6.81e-07 | 4.00e-05 (9.1521 / 9.1525) | 0 | 2 |
| sphere ∩ sphere | 1/1 | 1/1 | 1.43e-07 | 1.23e-07 | 1.58e-05 (5.4413 / 5.4414) | 0 | 1 |
| sphere ∩ bezier | 1/1 | 1/1 | 1.25e-07 | 3.37e-08 | 8.31e-05 (2.3696 / 2.3698) | 0 | 1 |

Aggregate: **9 branches / 5 pairs, all TRANSVERSAL fully-traced, 0 near-tangent-truncated**.
Closed-loop match **5/5** OCCT closed loops reproduced as Closed native WLines (bspline∩plane
correctly 0-closed / 4-open). Worst: max onCurve **1.60e-06**, max onSurf **6.81e-07** (both
skew-cyl-unequal); max lenΔ **2.28e-03** abs / ~0.33% rel (bspline∩plane — the only sub-mm-order
gap, within the deflection/step tol).

- **Honest scope / risk:** TRANSVERSAL only. **Near-tangent** branches are traced *up to*
  the tangent, marked `NearTangent`, counted in `nearTangentGaps` — never a point past it;
  **coincident / branch-point / self-intersection** deferred to S4. `nearTangentGaps > 0` is
  the honest S4 hand-off signal. Automatic densify-and-refit on a too-loose B-spline fit is
  not yet wired (the polyline stays the on-surface ground truth; the fit is a convenience
  curve) — follow-up.
- **Unlocks:** S5 curved booleans — the `TraceSet` (WLines with (u1,v1,u2,v2) per node) is
  its input contract.

### S4 — Tangent / degeneracy robustness · ◐ CLASSIFICATION LAYER (S4-a/b) + MARCHING-CORE SLICES (S4-c graze, S4-d branch points, S4-e sphere-pole/cone-apex + freeform (circular + asymmetric) chart singularities, S4-f robust closure + self-intersection guard + completeness critic) DONE AT THE BAR; S4-d general branches + S4-e higher-order/edge-seam + S4-f general topology repair pending
Near-tangent stepping (n₁×n₂→0: step control, higher-order predictor),
coincident/overlapping-surface detection, branch points & singularities,
self-intersection guards. **This is the moat** — OCCT's decades of tuning. Lands
as *progressively hardened*; whatever isn't robust **falls back to OCCT** and is
reported with the measured gap. Never "done"; hardened over time. Broken into
tractable sub-slices; the **detection + classification** layer (S4-a/b) landed
first, ahead of the hard **marching core** (S4-c).

#### S4-a — Coincident / overlapping-surface detection + typed region · ✅ DONE AT THE BAR
Robust coincidence detection on BOTH the analytic path and the seeded path, plus a
**typed `CoincidentRegion`** (`FullSurfaceSame` for a same-locus pair;
`OverlapSubRegion` with param bounds for a partial overlap; `Undecided` → OCCT when
the region cannot be robustly delimited) so downstream queries/booleans consume a
region descriptor instead of a bare flag. Analytic: closed-form `FullSurfaceSame`
predicates for ALL elementary families (plane, coaxial-equal cyl/cone, same sphere,
same torus), folding the pre-existing same-sphere / coaxial-equal-cyl `Coincident`.
Seeded (`CYBERCAD_HAS_NUMSCI`): grid-sample a candidate region (agree iff on-both
residual ≤ `onSurfTol` AND ‖n_A×n_B‖ ≤ `tangentSinTol`), grow to the agreement
boundary, emit `OverlapSubRegion` and suppress seeds/march inside — or `Undecided`
on a fuzzy boundary. `src/native/ssi/{coincidence.h,same_surface.h}`, OCCT-free.

#### S4-b — Tangent-contact CLASSIFICATION · ✅ DONE AT THE BAR
Replaces the blunt `SeedSet.deferredTangent` counter with a **typed
`TangentContact`**: `TransversalOnly` (no tangency), `TangentPoint` (isolated 0-dim
contact — e.g. spheres at d=R₁+R₂; emits the point), `TangentCurve` (tangency along
a whole curve — e.g. cylinder tangent to a plane along a line, coaxial sphere∩cyl
equator; emits the curve), `NearTangentTransversal` (grazes but crosses — the S4-c
gap, classified and handed on, **never traced**), or `Undecided` → OCCT. Analytic
tangent configs decided in closed form (`tangent_analytic.h`); seeded solutions
classified by local differential geometry — the relative second fundamental form
`H = II_A − II_B` in the shared tangent basis: sign-definite → `TangentPoint`,
rank-1 → `TangentCurve`, indefinite → `NearTangentTransversal`, within the
model-scale curvature-noise band → `Undecided` (never hand-tuned to force a verdict).
`src/native/ssi/{tangent_contact.h,tangent_seeded.h}`; marching (`marching.h` `WLine`)
carries an additive typed `stopReason` at a `NearTangent` stop — the tracer still
**stops at** the tangency, it does **not** step through (that is S4-c). Change
`add-native-ssi-s4-classification` (**archived** `2026-07-04`).
- **Verify:** ✅ host `test_native_ssi_s4_classification` (**14 analytic + 8 seeded
  cases, 0 failed**; NUMSCI OFF CTest **26/26** with the 8 seeded cases correctly
  ABSENT, NUMSCI ON CTest **31/31**) + ✅ sim native-vs-OCCT classification parity
  `run-sim-native-ssi-s4.sh` (**8 pairs, 0 failed, 0 deferred**; oracle
  `IntAna_QuadQuadGeo` / `IntAna_ResultType` for analytic, `IntPatch` /
  `GeomAPI_IntSS` + `GeomLProp_SLProps` for seeded). No regressions
  (`run-sim-suite.sh` **221/221**, all six pre-S4 parity scripts still green,
  S5 `native-pass=5` persists).

**S4-a/b classification pairs, native-vs-OCCT** (worstOnSurf = max residual of the
emitted point/curve on both surfaces; all at machine epsilon):

| Pair | native | OCCT | worstOnSurf |
|---|---|---|---|
| same sphere | Same (`FullSurfaceSame`) | Same | 0 |
| spheres d=R₁+R₂ | TangentPoint | Point | 1.22e-16 |
| spheres crossing | Transversal | proper section | 0 |
| plane tangent sphere | TangentPoint | Point | 6.12e-17 |
| coaxial sphere∩cyl equator | TangentCurve (Circle) | tangent Circle | 1.84e-16 |
| plane tangent cyl | TangentCurve (Line) | tangent Line | 0 |
| seeded sph∩sph (diff-geom) | TangentPoint | Point (sine 1.22e-16) | 1.22e-16 |
| seeded sph∩cyl (diff-geom) | TangentCurve | Circle (sine 0) | 0 |

Honestly **deferred / undecided** (asserted as such in the host seeded suite, NOT
weakened, NOT fabricated): opposite-saddle patch pair → `NearTangentTransversal`
(indefinite relative II — the S4-c gap, handed on, never traced);
matched-curvature contact below the model-scale curvature-noise floor → `Undecided`
→ OCCT. The sim parity set was **0 deferred** — every pair was decidable and agreed
with OCCT.
- **Honest scope / risk:** DETECTION + CLASSIFICATION only. S4-a/b **type** the
  degeneracy and emit the point/curve/region where determinable; they do **not**
  march through a tangency and do **not** fabricate a curve across a degeneracy —
  that is S4-c. A `NearTangentTransversal` is classified and handed on (still an
  S4-c → OCCT gap), never traced. `Undecided`/`None`/empty on every non-robust
  classification → engine-owned OCCT fallback + self-verify.
- **Unlocks:** S5 curved booleans can consume `CoincidentRegion` / `TangentCurve`
  (overlap handling, tangent-seam trimming) — a later S5 slice; and the marching
  core (S4-c) has a typed reason feeding it.

#### S4-c — Marching THROUGH a tangency · ◐ FIRST HONEST SLICE DONE AT THE BAR (grazing-but-continuous curves); deeper bands + branch crossings remain
The hard core of the moat: MARCH THROUGH a near-tangency **when the curve genuinely
continues**, rather than truncating. The first slice (`add-native-ssi-s4c-near-tangent-marching`,
gated `CYBERCAD_HAS_NUMSCI`, additive to `marching.cpp`) crosses a **NearTangentTransversal
single-branch graze** with four levers:
- **Fixed-plane-cut corrector** — the S3 corrector's along-`t` advance residual `r₃ =
  dot(A.point−Pprev, t) − h` ill-conditions as `t = normalize(nA×nB)` degenerates
  (`sine → 0`). Inside the crossing band `t` is replaced by the **last-good FORWARD
  tangent `t★`**, a hyperplane the curve crosses transversally even where the local
  surface tangent degenerates, so the `least_squares` solve stays well-posed.
- **Curvature-aware predictor** — bends `P + h·t★` by the discrete curvature of the last
  two nodes so the corrector starts in-basin across the sharp bend.
- **Step control** — enters the band at `sine < tangentSinTol`, steps FINELY (capped at
  `h₀/16`, deflection-bounded) so it RESOLVES the region instead of leaping it, exits once
  `sine ≥ 1.5·tangentSinTol` on the far side; `crossMaxSteps` budget + `minStep` floor.
- **Crossable gate (the honesty core)** — crosses ONLY when S4-b
  `classify_tangent_contact_seeded` types the stall `NearTangentTransversal` AND it is a
  genuine single-branch graze. Two witnesses force a defer: a **steep sine collapse**
  (stall sine < ¼ of the last-good sine ⇒ a tangency/branch drives `sine → 0`) and a
  **band-minimum floor** (a fine look-ahead scan whose minimum sine drops below
  `0.3·tangentSinTol`). A **branch crossing** (the equal-cylinder saddle — two branches
  meet, S4-d), a `TangentPoint`/`TangentCurve`/`Undecided`, non-convergence at `minStep`,
  or any node failing the on-both-surfaces / monotone-advance verification ⇒ the arc is
  **discarded** and the march still STOPS + classifies + defers → OCCT. No point is ever
  fabricated past a degeneracy; a crossed arc is emitted only if every node verified on
  both surfaces ≤ `onSurfTol`. Crossed grazes are counted in
  `TraceSet.nearTangentCrossed`; `nearTangentGaps` now counts only the regions that could
  NOT be crossed.

**At the bar (host + sim, `CYBERCAD_HAS_NUMSCI` ON):** a sphere grazed by an offset
cylinder that S3 TRUNCATES at `tangentSinTol=0.25` (sine dip ≈ 0.10) now traces the FULL
closed loop (`nearTangentGaps → 0`, `nearTangentCrossed ≥ 1`, every node on both surfaces
≤ 1e-6, crossed arc on the OCCT `GeomAPI_IntSS` locus ≤ 5e-4); the equal-radius
orthogonal cylinder **saddle (a branch crossing) STILL DEFERS with the flag off / at the
S4-c bar** (`nearTangentCrossed = 0`, `nearTangentGaps ≥ 1`) — that saddle is the S4-d
branch-point case, now localized + routed (below); genuine `TangentPoint`/`TangentCurve`
contacts still defer. Every S3 transversal fixture traces bit-identically (the
corrector/step outside the band is unchanged). Deeper near-coincident bands, general/freeform
+ higher-order-cusp singularities (S4-e tail; the sphere-pole/cone-apex chart singularities
are now crossed — see S4-e below) and self-intersection (S4-f) remain the tail: anything not
robustly crossable is still an honest `NearTangent` gap deferred to OCCT.

**S4-c deep-graze residual — RE-MEASURED, HONEST-DECLINE (freeform fuzzer, 3 seeds, N=48,
`0x5515D1FF0F0F`+2).** A pass targeting the "deeper near-tangent graze band the S4-c corrector
truncates" was investigated and DECLINED because the target does not exist in the current
measured build. Freeform fuzzer aggregate: **AGREED 124/144 (86.1%), HONESTLY-DECLINED 20/144
(13.9%), DISAGREED 0**. The decline reason histogram is **near-tangent 0, multi-branch 14,
small-loop 6** — the near-tangent MARCHING family is EMPTY. Per-decline anatomy (a diagnostic
probe over the marcher's `TraceSet`):
- **Every multi-branch decline is `seeded == traced`, `dedupedRetraces == 0`** — the recurring
  signature is `seeded=1 traced=1 deduped=0 occtComp=2`: S2 handed the marcher ONE seed, the
  marcher traced it perfectly (`BoundaryExit`, every node on both surfaces ≤ `onSurfTol`, node-on-
  OCCT-locus ≈ 5e-8), and OCCT found a SECOND distinct co-resident locus (`worstMissLen ≈ 2–5`,
  `genuineOcctOnNat ≈ 1.0–2.0`) the marcher was **never seeded for**. `deduped=0` proves the
  marcher did NOT drop a distinct loop — there was no second seed to drop. This is a pure **S2
  seeding-recall gap** (the subdivision seeder emits one seed for a twice-piercing pose), NOT a
  marching truncation. The completeness critic (finer subdivision + targeted uncovered-cell
  re-seed, loop-until-dry) recovers **zero** here (`criticRecoveredLoops == 0` at the finest
  floor) — the missed loci are co-resident in a way the AABB prune cannot separate, a seeding
  frontier, not a marcher lever.
- **Every small-loop decline is a fully-traced `Closed`/`BoundaryExit` with `nearTangentCrossed
  == 0`** whose native arc length matches the OCCT locus length (e.g. `arc=3.99` vs
  `occtTotLen=3.94`; `arc=0.203` vs `0.203`; `arc=5.67` vs `5.59`) with node-on-locus ≈ 5e-8.
  The 1.5e-3–4.6e-3 reverse-coverage miss (just over the fixed 1e-3 `onCurve` gate) is a
  **fitted-B-spline discretization bow** on a high-curvature loop — a fit-density artifact the
  harness already documents (`onCurve` comment: "a fitted-B-spline bow between nodes rides
  ~1e-4"), NOT a near-tangency the marcher truncated (`ntCross=0` confirms no crossing band was
  ever entered). S3 already traces these near-tangent loops fully and transversally.

**Conclusion / precise wall:** the S4 marching residual on the freeform fuzzer is **not a
near-tangent graze the marcher truncates** — the near-tangent marching family measures **0**.
The genuine ~14% decline is (a) **S2 seeding-recall of distinct co-resident loci** (dominant,
out of the marching scope and unrecovered by the completeness critic), and (b) a sub-`onCurve`
**fitted-spline coverage bow** on tight loops that ARE fully traced. No marching change to
`marching.{h,cpp}` can lower this decline without fabricating a curve across a locus that has no
seed, or re-baselining the `onCurve` fit gate — both refused. `DISAGREED == 0` holds; the
marcher's emitted geometry is correct on every node. The moat's next honest frontier is
**S2 seeding recall for co-resident branches**, not deeper S4-c graze marching.

**S2 CO-RESIDENT SEEDING-RECALL — FIXED + RE-MEASURED (freeform fuzzer, 3 seeds, N=48,
`0x5515D1FF0F0F`+2).** The frontier above was diagnosed and closed at the S2 layer (seeding.cpp
only; src/native stays OCCT-free; cc_* ABI byte-unchanged). Diagnosis (env-gated `CYBERCAD_SSI_SEED_DIAG`
per-cluster dump + host replay of the exact fuzz poses): on the `seeded=1 traced=1 deduped=0 occtComp=2`
cases, ALL candidate leaves of BOTH co-resident loci land in ONE param-adjacency cluster; the
distinct-locus split (`splitDistinctBranches`) then failed to separate them for a mechanical reason —
the per-cluster accumulator FIFO-appended refined seeds until a **flat 256 cap**, and one dense
locus tiles thousands of leaves that filled the cap in candidate-iteration order, **STARVING** the
co-resident locus's later leaves before the single-linkage split ever saw them (a second observed
sub-mode: `clusters=2` where one cluster's cells refine-missed). The fix (seeding.cpp): keep the
**FULL refined-seed density** per cluster (a thinned reservoir was tried and REJECTED — it left arc
gaps that spuriously split a SINGLE loop, breaking `march_bspline_bspline_closed_loop`), and make
the single-linkage split **O(m) via a uniform 3D spatial grid hash** so full density is affordable
(the old O(m²) linkage was what forced the small starving cap). The 256 cap becomes a pure safety
ceiling (65536) with no per-locus bias. HONEST/DISAGREED-safe: every retained seed is still a real
refined on-both-surfaces transversal crossing (`refineRegion` already gates point-on-both ≤ tol and
‖n₁×n₂‖ ≥ tangentSinTol); density only changes WHICH real seeds reach the split — never fabricates
a locus. **Measured before → after: AGREED 124→130/144 (86.1%→90.3%), HONESTLY-DECLINED 20→14
(13.9%→9.7%), multi-branch declines 14→9, DISAGREED 0→0 (bar holds).** Every recovered case became
AGREED (not DISAGREED), so the new seeds trace to curves on BOTH input surfaces AND on the OCCT
locus — genuineness confirmed by the oracle. Regression fixture: `seed_freeform_two_coresident_loci_recovered`
(the verbatim fuzz pose seed `0x5715d1ff1275` case 17 — two rational-NURBS∩B-spline loci; old S2
emitted 1 seed, new emits 2, each on both surfaces ≤ 1e-7 and transversal). Residual tail (9
declines): the remaining multi-branch cases are near-tangent-family poses where a co-resident locus
is itself near-tangent (a SEEDING-tangency deferral, honest → OCCT) or the `genuineOcctOnNat` miss
is a sub-`onCurve` fitted-spline bow (≈ 1.1e-3, just over the fixed 1e-3 gate) — the documented
fit-density artifact, not a fabrication.

#### S4-d — Branch points · ◐ TWO HONEST SLICES DONE AT THE BAR (analytic Steinmetz self-crossing + FIRST FREEFORM branch point: a B-spline saddle ∩ plane OPEN-ARM X-crossing; general multi-line/cusp/saddle∩saddle branches remain)
The hardest SSI piece: where the intersection **locus itself crosses** (multiple curve arms
meet at one point), LOCALIZE the branch point, ENUMERATE the outgoing arms from the local
second-order structure, ROUTE each arm with the S3 marcher, then ASSEMBLE the multi-arm
curve. The first slice (`add-native-ssi-s4d-branch-points`, archived `2026-07-04`, gated
`CYBERCAD_HAS_NUMSCI`, additive to `marching.cpp` + new `branch_point.h`, default-on
`enableBranchPoints`) fires **exactly where S4-c would have deferred** (the steep-sine-
collapse + tangent-flip witness) and resolves the elementary **transversal self-crossing**:
- **Localize** — `nn::minimize` the transversality sine `g(s) = ‖n_A×n_B‖` along the
  bracketed approach (each trial re-projected onto both surfaces with the S4-c fixed-plane
  corrector), then a full `nn::least_squares` re-project of the minimum onto both surfaces;
  accepted only when `‖A−B‖ ≤ onSurfTol` and the sine is at/near the floor, else DEFER
  (no fabricated B).
- **Enumerate arms** — build the shared tangent-plane basis at B, form the relative second
  fundamental form `H = II_A − II_B`, and solve the tangent-cone quadratic. Discriminant
  `Δ > 0` ⇒ two distinct real tangent lines ⇒ up to four world-space rays (`±T₁, ±T₂`);
  `Δ ≤ 0` ⇒ EMPTY (definite ⇒ isolated `TangentPoint`, END; double root ⇒ cusp, out of
  scope, DEFER). **Never fabricates a ray** — the same discriminant sign as S4-b's
  `TangentPoint` classification enforces "an isolated tangent point still ends".
- **Route + assemble** — step `h₀/8` off B along each real ray, S4-c-correct back onto both
  surfaces, then run the normal S3 walk to termination; dedup arms that retrace a kept arm
  (`retraces`) and merge their shared branch-point connectivity into the `BranchNode`; count
  `TraceSet.branchPoints` and record `armLineIds`. A branch not robustly
  localizable/enumerable/routable STOPS + defers **exactly as S4-c** (a `NearTangent` WLine
  counted in `nearTangentGaps`).

**At the bar (host + sim, `CYBERCAD_HAS_NUMSCI` ON):** the **Steinmetz bicylinder** (two
equal-radius R=1 cylinders, axes Z and X crossing orthogonally) — which S3+S4-c TRUNCATE at
the saddle (one `NearTangent` WLine, `branchPoints = 0`) — is now **FULLY traced**: both
branch points localized at `(0, ±1, 0)` (branch sine ≈ 5e-8 / 9e-8, re-projection residual
≈ 5e-13), four `BranchArc` arms routed and assembled into the two crossing ellipses,
`nearTangentGaps = 0`, every node on both cylinders ≤ `onSurfTol`. Sim parity vs OCCT
`IntPatch`/`GeomAPI_IntSS`: `eq-cyl s4d branchPts=2 traced=4 arms=3 onCurve=1.74e-6
onSurf=1.07e-8` — every native arc node on the OCCT locus and on both surfaces, both branch
points matching the OCCT saddles at `(0, ±1, 0)` to tol. The isolated `TangentPoint` (two
spheres at `d = R₁+R₂`) STILL ENDS with zero arms (definite `H` ⇒ no real roots); the S4-c
graze still crosses (`crossed = 22`); the flag-off eq-cyl control still defers; the 5
transversal pairs stay `nt = 0` bit-identical.

**Second slice — FIRST FREEFORM branch point (M1, `moat-m1-ssi-s4-general`, gated
`CYBERCAD_HAS_NUMSCI`, additive to `marching.{h,cpp}` only — no change to `branch_point.h`).**
The S4-d machinery (`localize` / `enumerateArms` / `sharedTangentFrame` / `relativeSecondForm` /
`solveTangentCone` / `routeArm` / `routeBranches`) is SURFACE-AGNOSTIC — it touches a surface only
through `SurfaceAdapter` — so a **bicubic B-spline saddle tangent to a plane through its saddle
point** localizes `branchPoints == 1` on both surfaces ≤ `onSurfTol` and enumerates the correct FOUR
arm rays with the EXISTING code. The ONE gap was ASSEMBLY, not geometry: the Steinmetz branch is a
CLOSED network (each arc runs branch-to-branch, so `reclassifyBranchArcs` recognised only
both-ends-on-a-branch), but a freeform X-crossing on a FINITE patch radiates FOUR **OPEN** arms
(each branch-to-boundary — one end the localized branch, the other a clean domain exit). Fix (the
whole slice): two additive `WLine` flags `frontNearTangent` / `backNearTangent` record WHICH end
stalled at a near-tangency, and `reclassifyBranchArcs` is generalised to the honest OPEN-ARM rule —
reclassify to `BranchArc` iff every END that stalled sits on a localized branch point and at least
one does; a near-tangent end NOT on a branch keeps the arc a `nearTangentGaps` gap (defer → OCCT).
This reduces EXACTLY to the both-ends rule for Steinmetz (bit-identical). At the bar (host + sim):
`saddle s4d-g branchPts=1 NTgaps=0 traced=4 arms=3 onCurve=8.93e-8 onSurf=5.10e-10 occtBr=4` — every
arm node on the OCCT `GeomAPI_IntSS` locus and on both surfaces, the branch at the saddle. The
honesty control (a B-spline BUMP `z=0.15·(x²+y²)` tangent to a plane, definite `H`, Δ ≤ 0) still ENDS
with NO arms (`branchPoints == 0`, never a fabricated arm); the not-through-saddle (`z=0`) plane
still traces two DISJOINT open curves with `branchPoints == 0`. NOTE: the once-hypothesised
Richardson third-derivative bias-cancellation was REFUTED (the central-difference `relativeSecondForm`
already cancels odd-order terms, κ at B is O(δ²)-accurate ~1e-7) and is NOT shipped — no dead code.

- **Honest scope / risk:** the **elementary two-real-distinct-line transversal self-crossing** is
  now traced for BOTH the analytic Steinmetz family (closed network) AND the first FREEFORM case (a
  B-spline saddle ∩ plane, open arms). Still DEFERRED → OCCT, reported with the measured gap, never
  faked: **non-transversal (definite) freeform contacts** (end with no arms — pinned by the bump
  control), **freeform cusps** (double root of the tangent-cone quadratic), **higher-multiplicity
  junctions** (three-plus tangent lines at one point), **both-operand-freeform saddle∩saddle** whose
  branch does not verify, and **general small-loop / topology repair (the S4-f residual)**. (**S4-e
  chart singularities** — the sphere parametric pole + cone apex — are crossed natively; and a
  **single-arm figure-eight self-intersection** is DETECTED + traced-through as typed data by the
  S4-f guard — `branchPts=0`, distinct from this locus branch — though it is not yet split into
  sub-arcs; see the S4-e and S4-f slices below.)
- **Unlocks:** **Steinmetz is now unblocked** natively; the multi-arm `TraceSet` +
  `BranchNode` connectivity is available to S5 curved booleans for self-crossing loci.

#### S4-e — Singularities · ◐ THREE HONEST SLICES DONE AT THE BAR (analytic sphere-pole + cone-apex crossed; FREEFORM circular parametric pole crossed; ASYMMETRIC / non-circular freeform pole crossed; curve cusp declined by IFT; higher-order + edge/seam degeneracies remain)
A **chart (removable) singularity** is where ONE surface's own `(u,v)` parametrization
degenerates while its 3D point + normal stay finite: a **sphere parametric pole**
(`v = ±π/2`, where `‖dU‖ = R·cos v → 0`) or a **cone apex** (signed radius
`R₀ + v·sin α = 0`, where the tangential `‖dU‖ → 0`). The intersection can be perfectly
**transversal** through such a point — the pair sine `‖n_A×n_B‖` need NOT collapse — yet
the S3 marcher breaks there: `advanceParams` solves each surface's single-surface 2×2
normal equations, and when that surface's `dU` row vanishes the 2×2 is rank-1, so the
`(u,v)` update is ill-conditioned even though the 3D residual + normal are fine (and the
pole sits on a non-periodic `v` edge, so the marcher also reports a spurious `BoundaryExit`
or step-crawls the node budget at the apex). The first slice
(`add-native-ssi-s4e-singularities`, archived `2026-07-05`, gated `CYBERCAD_HAS_NUMSCI`,
additive to `marching.cpp` + new OCCT-free `chart_singularity.h`, default-**off**
`enableChartSingularities`) detects and steps across the pole/apex:
- **Single-surface chart witness (the S4-e detector, DISTINCT from S4-c/S4-d)** —
  `chartConditionAt` finite-differences each surface's `‖dU‖` against `‖dV‖·scale`; a
  collapse (`‖dU‖ ≪ collapseFrac·‖dV‖` AND `≪ collapseFrac·scale`) with a **finite normal**
  flags a pole/apex on THAT surface. Computed from ONE surface's own Jacobian — NOT the pair
  sine (the S4-c near-tangent witness, which need not collapse at a pole) and NOT a locus-
  tangent flip (the S4-d branch witness). A finite cap keeps `‖dU‖ = O(‖dV‖)` so a genuine
  domain boundary is NOT mistaken for a pole (it exits as a normal `BoundaryExit`).
- **Point-based fixed-plane-cut crossing** — at a detected collapse, `crossChartSingularity`
  makes a bounded sequence of fine POINT-BASED jumps along the fixed last-good forward
  tangent `t★` (the branch_point.h / S4-c cut: drive `A.point − B.point → 0` under an
  along-`t★` hyperplane), which NEVER touches the degenerate single-surface `dU`, so it stays
  well-posed exactly where `advanceParams` failed.
- **Loose chart map-back** — the singular surface's far-side `(u,v)` are re-seeded LOOSELY by
  continuity: a **sphere pole** continues on the OPPOSITE meridian (`u_out = u_in + π mod 2π`,
  the free-longitude jump) with the latitude reflecting; a **cone apex** is a single 3D point
  the straight curve passes through to the far nappe (`v → −v`). The corrector confirms these;
  the singular point itself is never emitted.
- **Honest guard (the honesty core)** — a node is emitted ONLY if it verifies on BOTH
  surfaces ≤ `onSurfTol` and makes real along-`t★` progress. On ANY failure (won't verify, no
  progress, or the crossing budget exhausted) the whole band is DISCARDED (roll back) and the
  march STOPS + defers → OCCT as a `NearTangent` gap counted in `nearTangentGaps`. No
  pole/apex-crossing point is ever fabricated.

**At the bar (host + sim, `CYBERCAD_HAS_NUMSCI` ON):** a marched great circle crossing BOTH
sphere poles (`v = ±π/2`) that S3 TRUNCATES at the first pole (half loop, `len ≈ 3.1415`) is
now **FULLY traced** — `singularitiesCrossed = 2`, `nearTangentGaps = 0`, one closed loop,
`len` native `6.2829` vs OCCT `6.2832` (rel Δ 5.0e-05), every node on the OCCT
`GeomAPI_IntSS` locus + both surfaces ≤ 1.51e-07. A double-cone `∩` plane whose line passes
THROUGH the **cone apex** that S3 STEP-COLLAPSES at (`v` stalls at ≈ −0.04) is now **FULLY
traced across both nappes** — `singularitiesCrossed = 1`, `nearTangentGaps = 0`, bounded 159
nodes, `v ∈ [−2.00, +2.00]`, on-locus 7.11e-16 / on-surface 6.79e-16. Sim parity
`sphere-pole s4e singX=2 NTgaps=0 closed=1` and `cone-apex s4e singX=1 NTgaps=0 nodes=159`.
A genuine finite cylinder `v`-cap still exits as a `BoundaryExit` (chart machinery does NOT
misfire); the S4-c graze still crosses and the S4-d Steinmetz still traces with the flag on;
the 5 transversal pairs stay `nt = 0` bit-identical.

**Second slice — FREEFORM parametric pole (`add-native-ssi-s4e-general`, gated
`CYBERCAD_HAS_NUMSCI`, additive):** a B-spline/NURBS surface with a **collapsed control ROW**
(the whole `u` line at a `v`-edge maps to one point — the spline analog of the sphere pole:
`‖dU‖ → 0`, finite point, finite-limit normal) is detected by the SAME single-surface witness
(the degenerate freeform normal `normalize(Sᵤ×Sᵥ)` is a finite near-zero `Dir3`, so
`normalFinite` holds and `collapsed` fires) and crossed by the SAME point-based corrector. The
**only** new code is the far-side re-seed: a freeform adapter carries `uPeriod == 0`, so the
analytic `u_in + π` meridian jump does not apply — instead `chartsing::freeformChartInvert`
recovers the far LONGITUDE by a point-only search for the `u` at the SAME near-pole latitude
whose surface point is nearest the continued 3D tangent target (fixed-latitude, so it never
collapses onto the degenerate tip; the corrector then verifies on both surfaces). The analytic
`uPeriod > 0` path keeps the exact `u_in + π` jump BIT-IDENTICAL. **At the bar:** a NATIVE NURBS
unit sphere (a `uPeriod == 0` collapsed-row surface of revolution — a genuine freeform pole,
OCCT-freeform-oracle-free) `∩` plane that S3 truncates at the first pole (half circle) is now
FULLY traced — `singularitiesCrossed = 2`, closed great circle, every node on both surfaces
`≈ 4e-16` (host) — and vs OCCT `Geom_SphericalSurface` `GeomAPI_IntSS`
(`freeform-pole s4e singX=2 NTgaps=0 closed=1 onCurve=1.51e-07 onSurf=1.51e-07`, native len
`6.2829` vs OCCT `6.2832`). The **must-still-defer control** is a collapsed-row Bézier cone-tip
whose pole sits on the `v=1` DOMAIN BOUNDARY (a genuine surface ENDPOINT, no far side): the
witness fires but the far-side re-seed cannot verify past a nonexistent surface, so it correctly
DEFERS (`singularitiesCrossed = 0`, `NearTangent` → OCCT) — no fabricated point past a real tip.

**Third slice — ASYMMETRIC (NON-CIRCULAR) freeform pole (this change, gated
`CYBERCAD_HAS_NUMSCI`, additive):** the second slice's freeform crossing relies on the
POINTWISE `‖dU‖ ≪ collapseFrac·‖dV‖` witness, which fires reliably on a CIRCULAR collapsed-row
pole (`‖dU‖ → 0` uniformly along the whole `u` row) but only in a razor-thin band on a
NON-CIRCULAR (elliptical / lumpy) collapsed-row pole — there `‖dU‖` collapses FAST along the
major axis but SLOWLY along the minor axis, so a meridian marched along the minor axis reaches
the non-periodic `v` edge and spuriously `BoundaryExit`s before the pointwise ratio ever crosses
`collapseFrac` (the "asymmetric freeform pole" the second slice deferred). This slice adds two
strictly-additive, FREEFORM-only, default-gated pieces:
- **Degenerate-`v`-edge witness (`chartsing::degenerateVEdge` + `chartCondAugmented`)** — the
  collapse is recognised as a SURFACE property of the `v`-edge, not a pointwise ratio at the
  current node: the nearest non-periodic `v`-edge is DEGENERATE iff sampling the WHOLE `u` row at
  that edge collapses to essentially one 3D point (`spread ≪ collapseFrac·scale`). That is exactly
  a chart pole (every `u` → one point) and is FALSE at a finite `v`-cap (the edge row is a full
  circle of radius `R`, `spread = 2R`). A node APPROACHING such an edge (within
  `chartEdgeApproachV` = 2% of the `v`-domain) whose `‖dU‖` has already dropped below
  `chartEdgeCollapseFrac·‖dV‖` (5%) is ALSO flagged collapsed, so the slow minor-axis collapse
  fires BEFORE the boundary. Both knobs are looser than `chartCollapseFrac` and only widen WHERE
  the crossing fires; the augmentation is skipped entirely on ANALYTIC surfaces (`uPeriod > 0` —
  they keep the exact pointwise+`u+π` path BIT-IDENTICAL) and never trips at a cone apex (interior
  `v`, not near a `v`-edge) or a finite cap. `chartRecovered` uses the same augmented condition so
  the far side is recognised as recovered once `v` steps back out of the band.
- **Reflected-far-point crossing pin** — because the witness fires a finite distance BEFORE the
  pole (the minor-axis `‖dU‖` collapses slowly), the far meridian is `~2·(P_pole − anchor)` away,
  which the fine step `h` cannot reach. On the single CROSSING step of a freeform pole,
  `crossChartSingularity` pins the fixed-plane reproject at the along-`t★` distance to the
  REFLECTED far point `2·P_pole − anchor` (point-only, `P_pole = S.point(u, edgeV)`), which only
  ever WIDENS `pinDist` (never shrinks below `h`, never weakens `onSurfTol`) and only sets WHERE
  along `t★` the verified far node sits. `freeformChartInvert` then recovers the far longitude at
  that reflected target. Circular poles keep `h` (the witness fires AT the pole; jump ≈ `h`) and
  stay bit-identical.

**At the bar (host, `CYBERCAD_HAS_NUMSCI` ON):** a NATIVE NURBS ELLIPTICAL teardrop (`x` stretched
`1.6×`, both `v`-edges genuine collapsed-row poles — a `uPeriod == 0` freeform surface, NOT a
surface of revolution) `∩` the `x=0` plane traces the MINOR-axis meridian, which pre-slice reached
each `v`-edge and spuriously `BoundaryExit`ed at HALF the loop (`len ≈ π`). AFTER: the
degenerate-`v`-edge witness fires on approach, the crossing reflects through each pole point to the
far meridian, and BOTH non-circular poles are crossed — one closed loop, `singularitiesCrossed = 2`,
`nearTangentGaps = 0`, `len ≈ 2π`, `v ∈ [~0, ~2]`, every node on both surfaces ≤ `1e-6` (the `x=0`
minor-axis cross-section is exactly the unit circle, so the traced loop is the unit great circle).
The controls stay bit-identical: analytic sphere-pole / cone-apex / freeform-circular-pole
fixtures unchanged (they never trip the freeform edge-augmentation), the freeform Bézier cone-tip
still DEFERS at its `v=1` domain-boundary endpoint (a degenerate edge but a genuine surface end —
the far-side reproject cannot verify past a nonexistent surface), and the S4-c graze / S4-d
Steinmetz still cross with the flag on (`singularitiesCrossed = 0`).

**Curve cusp — DECLINED (no dead code):** a cusp of the intersection curve (arclength velocity
→ 0) requires `‖n_A×n_B‖ → 0`; by the implicit-function theorem, with regular charts AND healthy
pair sine the intersection is a smooth regular curve, so "a curve cusp with regular charts and
healthy sine" is the EMPTY set — a cusp always coincides with the pair-tangency regime already
owned by S4-c (graze march-through), S4-d (branch routing), or an honest OCCT deferral. A
standalone single-surface cusp witness would be unreachable dead code, so NONE is added; curve
cusps route to the existing S4-c/S4-d/OCCT path.

- **Honest scope / risk:** the two **analytic chart singularities** (sphere pole, cone apex), the
  **freeform circular parametric pole** (collapsed spline/NURBS row), and now the **asymmetric
  (non-circular / elliptical) freeform pole** (via the degenerate-`v`-edge witness + reflected-far
  crossing pin) are crossed, each verified node-by-node on both surfaces + on the OCCT locus. Still
  DEFERRED → OCCT (reported, never faked): **higher-order / edge / seam** degeneracies (and any
  asymmetric pole so extreme its reflected-far reproject does not verify — it defers the same
  honest way); a full brep **degenerate-pole B-spline SOLID
  through the boolean pipeline** (no native construct feeds a freeform-pole face to the marcher —
  the freeform-pole fixtures are hand-seeded, exactly as the analytic S4-e fixtures are); the
  **curve cusp** (declined above); and the **general self-intersection residual (S4-f)**. Any
  pole whose point-based crossing does not verify on both surfaces defers the same honest way.
- **Unlocks:** transversal intersection curves that pass through a sphere pole / cone apex / a
  freeform (collapsed-row spline) parametric pole — including a NON-CIRCULAR (elliptical / lumpy)
  freeform pole where `‖dU‖` collapses anisotropically — are now traced end-to-end natively
  instead of truncating at the chart singularity.

#### S4-f — Self-intersection / small-loops · ◐ FIRST COMPLETENESS + LOOP-ROBUSTNESS SLICE DONE AT THE BAR
Adds no new geometry capability — it HARDENS the correctness/completeness of the curves S3
already traces. Two orthogonal parts, both additive + gated so the S3/S4-c/S4-d/S4-e controls
stay byte-identical (`src/native/ssi/{marching.h,marching.cpp}` + new OCCT-free
`src/native/ssi/completeness_critic.h`, `CYBERCAD_HAS_NUMSCI`-gated, no `cc_*` change).

- **Robust TRUE-RETURN closure (always on).** S3 closed a loop on pure proximity
  (`distance(cur, seed) ≤ loopClose·h`), which FALSE-CLOSES a curve that merely re-approaches
  its seed / an earlier node while heading the other way. Closure is now a necessary-condition
  tightening: close only when the march has actually travelled a full circuit
  (`arcLen > 2·closeRadius`, the true-return gate) AND the return heading is tangent-continuous
  with the seed's outgoing tangent (`dot(fwdNow, seedFwd) ≥ closureTangentCos`, default 0.5).
  It can only REFUSE a close, never MAKE one — every truly-closing control still closes
  byte-identically, while an inflated-radius near-pass no longer truncates (fixture B: a crossing-
  spheres circle traced at 10× loopCloseFrac went from ~1.2% of the true length to ≥ 93%, default
  frac byte-identical at 99.6%).
- **Self-intersection guard (default off `enableSelfIntersection`).** A single arm that crosses
  ITSELF (a figure-eight section) is detected by a geometric segment-segment crossing test over
  the stitched polyline — two non-adjacent segments whose closest approach ≤ a tight touch radius
  at a TRANSVERSE angle (`|cos| < 0.7`, so a retrace / (anti)parallel doubling-back is excluded)
  — recorded as a typed `WLine.selfIntersection` (DATA), and the arm marches THROUGH it (never
  stopped, never closed). DISTINCT from an S4-d `BranchNode` (a locus flip, `‖nA×nB‖→0`, that
  spawns arms): a self-crossing keeps ONE arm, so `branchPoints == 0`. Off → byte-identical.
- **Adaptive completeness critic (default off `completenessCritic`).** After the initial fixed-
  resolution seed + trace, LOOP-UNTIL-DRY: build a coarse coverage grid over A's domain from the
  traced polylines (`critic::coverageOf` / `uncoveredBoxes`), re-seed FINER
  (`minPatchFrac *= criticRefineFactor` per round) at the SAME `onSurfTol` (a candidate that does
  not land on both surfaces is DISCARDED — never a fabricated seed), dedup the traced NEW branches
  by LOCUS vs all kept curves (so a finer re-trace of an already-found loop is not over-produced),
  keep the genuinely new ones. Stop after `criticDryRounds` (K) consecutive dry rounds or the cost
  cap (`criticMaxRounds` / `criticMaxCandidates`). Fixture A: a small loop missed at 1/16
  (recall 0.5) is RECOVERED (recall 1.0 on that fixture); fixture D: four disjoint loops rise from
  recall 0.25 to 1.0 — both stopped dry, no over-production (traced == true count).

**HONEST FRAMING (baked into the headers, tests, and this row):** completeness is MEASURED +
ASYMPTOTIC, never a proof. Below ANY fixed re-seed round a smaller loop can still be missed, so
`TraceSet.completenessResidual` / `RecallReport.residualAcknowledged` are ALWAYS true and the
critic reports the floor reached (`criticFloorFrac`, `criticStoppedDry`) — a fixture's recall→1
is scoped to that fixture at that floor, never a global claim. NEVER fabricates a loop, a closure,
or a seed; an unrecoverable loop is a reported measured recall < 1, a self-intersection is a
recorded typed crossing, a false-close is prevented (not a faked continuation).

Host gate green: `test_native_ssi_s4f_completeness` **6 cases, 0 failed** (fixtures A–D + the
transversal-loop and S4-d Steinmetz controls; NUMSCI ON CTest **33/33**, the S4-f TU ABSENT with
NUMSCI OFF). No tolerance weakened; `src/native/**` stays OCCT-free.

**S4-f DE-RISKS (does not unblock/complete) curved blends (#6) + wrap-emboss (#7)** — their
intersection seams are exactly the small-loop / self-intersecting / many-loop patterns this slice
hardens, but their assemblers stay S5/S6/S7. Global topology repair / watertight self-intersection
resolution (splitting a self-crossing arm, healing a self-intersecting shell) also stay the tail —
S4-f DETECTS + REPORTS + traces-through, it does not repair topology.

Archived change `openspec/changes/archive/2026-07-05-add-native-ssi-s4f-completeness`.

### S5 — Curved booleans via SSI (the payoff) · ◐ NATIVE SLICES S5-a/b/c/d/e/f/g/h/i/j landed + S5-k FIRST TRANSVERSAL (non-coaxial) slice + S5-l/m/n/o TORUS surface family (CONE surface family opened — coaxial cone∩cylinder, single- AND two-circle cone∩sphere, coaxial cone∩cone (frustum AND apex-to-apex hourglass), AND two-circle cylinder∩sphere op-sets COMMON/FUSE/CUT now COMPLETE 3/3 native; S5-k lands the FIRST non-coaxial pose — the offset cylinder∩sphere, COMMON/CUT/FUSE now COMPLETE 3/3 native via the TRANS-BAND seam-band shell (`appendSphereOuterZoneBetweenSeams` tiles the sphere outer zone on-surface, ΔV <0.1%); S5-l opens the TORUS family — coaxial torus∩cylinder COMMON/FUSE/CUT COMPLETE 3/3 native; S5-m extends it — coaxial torus∩sphere (sphere at torus centre) COMMON/FUSE/CUT COMPLETE 3/3 native; S5-n extends it again — coaxial torus∩cone (oblique-chord generalisation of S5-l) COMMON/FUSE/CUT COMPLETE 3/3 native; S5-o closes the family with the FIRST curved∩curved pair where BOTH operands are tori — coaxial torus∩torus COMMON/FUSE/CUT COMPLETE 3/3 native; S5-p lands the SECOND transversal (non-coaxial) slice and the FIRST transversal TORUS pair — the offset (axis-parallel) torus∩cylinder COMMON/CUT/FUSE now COMPLETE 3/3 native (the KEY finding: the two seams are LOCALIZED in the torus (u,v) — u∈[−0.2,0.2], NOT azimuth-wrapping like the sphere family — so the outer zone is the FULL torus tube MINUS the two seam cap patches, tiled on-surface by a (u,v)-grid + loop-zipper hole-split, ΔV <0.8%; NOT a sphere-style between-seams band); S5-q lands the THIRD transversal (non-coaxial) slice and the FIRST transversal CONE pair — the offset cone∩sphere, COMMON/CUT/FUSE now COMPLETE 3/3 native via the SAME TRANS-BAND seam-band shell about the cone axis (ΔV <0.15%); S5-r lands the FIRST curved∩PLANAR-half-space slice — the coaxial torus∩(axis-perpendicular plane) COMMON+CUT as a Pappus-exact revolution (tube-arc + annulus cap), FUSE + the plane-parallel/degenerate poses honest-decline; S5-s lands the FOURTH transversal (non-coaxial) slice and the FIRST transversal cone∩cylinder pair — the offset (axis-parallel) cone∩cylinder COMMON/CUT/FUSE now COMPLETE 3/3 native (a SINGLE-SEAM pose: a parallel-axis cone∩cyl crosses in ONE loop because the cone wall is monotonic; COMMON = cone cap + cyl band + end disc; cyl−cone = a clean seam-driven cyl stub (reversed cone cap); cone−cyl + FUSE = the FULL cone wall MINUS the seam cap (a HOLED cone surface, the SAME (u,v)-grid + loop-zipper hole-split as S5-p, in AXIAL cone coords) + cone discs + reversed cyl bite, ΔV <0.7%); BOOL-COMPLETE lands the NATIVE TORUS PRIMITIVE — `construct::build_torus` + the ADDITIVE facade `cc_torus` emit a bare periodic `Kind::Torus` face (a revolve builds B-spline bands that decline), so the coaxial torus∩{cyl/sphere/cone/torus} families now fire in the PURE-NATIVE path from a SHIPPING primitive (not just the in-test fixture); it also closes two order-sensitive reverse CUTs — `cyl−torus` (grooved cylinder, `buildCylTorusCut`) and `sphere−cyl` (tunnelled sphere, `buildSphereCyl2Cut`); BOOL-TAIL then lands the LAST three reverse CUTs the audit flagged — coaxial `sphere−torus` (grooved ball, `buildSphereTorusCut`) + coaxial `cone−torus` (grooved cone, `buildConeTorusCut`) via the hole-split-of-the-outer-surface + reversed-inner-tube-arc idiom, and transversal `cyl−torus` (lens-bitten cylinder, `buildTransCylTorusCut`) via the reversed tube inner-cap patches — all watertight, partition-correct, ΔV <1%, DISAGREED=0, so the ELEMENTARY curved boolean (cyl/sphere/cone/torus/plane, coaxial+transversal, COMMON/CUT/FUSE incl. order-sensitive reverses) is now FULLY complete on the pure-native path)
Use SSI curves to **split** the curved faces of two solids, **classify**
fragments inside/outside (reuse the BSP-CSG classifier + a curved point-in-solid
test), **assemble** the surviving shell watertight (curved-seam weld from the
mesher). Extends `src/native/boolean/` from planar/axis-aligned to general curved.
- **Verify:** native-vs-OCCT `BRepAlgoAPI` (volume/area/watertight) on
  cylinder∩cylinder, sphere∩box, cone∩box, fillet-shaped tools; self-verify →
  OCCT fallback for the rest.
- **Unlocks:** curved blends (#6) and curved wrap-emboss (#7) then compose on top.

**FACADE / PYTHON EXPOSURE (NURBS-EXPOSE, verified 2026-07) — no new `cc_*` entry point.**
The S5 curved-boolean families are reachable from the EXISTING frozen `cc_*` ABI: build the
analytic operands with `cc_solid_revolve_profile` (a revolved TYPED profile → true
`Kind::Cylinder`/`Sphere`/`Cone` walls + planar caps; the Python binding surfaces this as the
additive `Kernel.cylinder_solid`/`sphere_solid`/`cone_solid`/`revolve_profile`/`slab_box`
helpers) and boolean them with `cc_boolean` (`Shape.common`/`cut`/`fuse`), which routes
engine→`boolean_solid`→`ssi_boolean_solid` (native) or OCCT. Verified matrix:
- **OCCT engine (default desktop build): ALL families + all three ops (COMMON/CUT/FUSE)
  reachable and volume-exact** (`python/examples/curved_booleans.py`, `python/tests/test_curved_boolean.py`).
- **Native engine (`CYBERCAD_HAS_NUMSCI`): COMMON reachable for every family with a closed-form
  oracle; CUT/FUSE now ALSO reachable** after closing the ONE self-verify gap found —
  `NativeEngine::boolean_op`'s generic set-algebra check demanded the exact-planar `1e-6` band,
  unattainable for a curved CUT/FUSE result because the O(deflection) operand-mesh bias does not
  cancel across `va−vc` / `va+vb−vc`. Fix (engine-only, `src/engine/native/native_engine.cpp`;
  `src/native/**` incl. `ssi_boolean.cpp` UNCHANGED; `cc_*` ABI byte-unchanged): when an operand
  is a recognised curved solid, widen ONLY that case to a deflection-bounded 2% band (the same order
  the NURBS orchestrator's `analyticVolumeBandOk` uses) — a wrong/leaky result differs by a whole-
  feature volume and is still rejected (DISAGREED=0); the all-planar contract keeps the strict `1e-6`.
- **Gap (documented):** the native TORUS families need a bare periodic `Kind::Torus` face, which no
  `cc_*` entry constructs natively (a revolved full circle builds B-spline bands → declines / falls to
  OCCT), so a *pure native* torus boolean is not reachable; it IS reachable under OCCT. Closing it would
  need an additive `cc_torus`-style primitive — deferred (not required for the OCCT-served exposure).

**S5-a/b/c + S5-d + S5-e done at the bar (changes `add-native-ssi-curved-boolean` archived +
`add-native-ssi-curved-boolean-wider` + `add-native-ssi-branched-boolean` archived
`2026-07-05`; `add-native-cone-boolean` + `complete-cone-cyl-fuse-cut` archived `2026-07-07`;
`add-native-cone-sphere-boolean` archived `2026-07-07`):**
the SSI-curve-driven
split→classify→select→weld pipeline lives in
`src/native/boolean/ssi_boolean.{h,cpp}` (OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated, consumes the
S3 `TraceSet` — and, for S5-d, the S4-d branched re-trace with `MarchOptions.enableBranchPoints
= true`). It now produces **twenty-two native curved-boolean sub-cases verified vs OCCT
`BRepAlgoAPI_{Fuse,Cut,Common}`** (sim parity `native-pass=33`, 39 passed / 0 failed / 6 fall-backs
— the sphere∩sphere, the
Steinmetz bicylinder, the coaxial cone∩cylinder, the coaxial cone∩sphere SINGLE- and TWO-circle,
the coaxial cone∩cone, AND the coaxial TORUS∩cylinder op-sets are each COMPLETE 3/3 native; the
harness runs each of the sphere FUSE/CUT as an equal-R AND an unequal-R fixture; 6 honest fallbacks):
- **S5-a — through-drill cylinder∩cylinder COMMON** (unequal radii, transversal two-loop
  trace) — watertight, ΔV = 8.1e-04, ΔA = 2.8e-04.
- **S5-b — through-drill cylinder∩cylinder FUSE + CUT** (assembler-only extension: fat wall
  with the two mouths cut out + planar-facet caps + reversed tunnel band / protruding end
  tubes) — watertight, ΔV = 8.8e-05 (fuse) / 4.0e-05 (cut).
- **S5-c — sphere∩sphere COMMON / FUSE / CUT** (single closed seam; the op-set is now COMPLETE
  3/3 native). COMMON = the lens of the two inside-the-other (INNER) spherical caps; FUSE (A∪B)
  = the two OUTER caps (each sphere's far-pole cap) welded on the shared seam, `V = V(A)+V(B)−lens`;
  CUT (A−B, order-sensitive) = the OUTER cap of A + the INNER cap of B emitted REVERSED (inward
  normal, bounding the scooped cavity), `V = V(A)−lens`. All three share the SAME decimated seam
  and reuse one generalised `appendSphereCap(outer,reversed)` cap builder + `VertexPool` weld;
  direction-slerp cap facets, robust even when a cap apex sits at the sphere's parametric pole.
  Watertight, verified vs BOTH the analytic closed forms AND OCCT `BRepAlgoAPI`: COMMON ΔV = 4.1e-04
  (eq) / 4.7e-04 (uneq); FUSE ΔV = 6.5e-04 (eq) / 8.3e-04 (uneq); CUT ΔV = 7.0e-04 (eq) / 9.3e-04
  (uneq) — all inside the 1% curved-parity bar, no tolerance weakened. Survival gate declines
  (→ NULL → OCCT) any non-transversal pair (tangent / containment / concentric).
- **S5-d — Steinmetz (equal-radius orthogonal cyl∩cyl) COMMON / FUSE / CUT** (the
  *branched-trace* op-set, now COMPLETE 3/3 native): a `steinmetzPreGate` (equal-R, orthogonal,
  crossing axes) fires ONLY on the S4 decline edge (`nearTangentGaps > 0`), RE-TRACES with branch
  points enabled, and `recogniseSteinmetzTrace` accepts only the canonical structure
  (`branchPoints == 2`, four `BranchArc` arms). The lune/arc split + `VertexPool` weld machinery
  is shared across all three ops; the difference is which fragments survive + cap handling.
  - **COMMON** — `buildSteinmetzCommon` splits each cylinder along its two arcs into the
    inside-the-other lune patches, keeps the four whose centroid is inside the other cylinder, and
    welds them into ONE watertight shell sharing the four arc seams and the two branch-point
    vertices (S5-a planar-facet + `VertexPool` discipline). Byte-identical to the S5-d baseline.
  - **FUSE (A∪B)** — `buildSteinmetzFuse` keeps the OUTSIDE wall regions of BOTH cylinders + all
    four original end caps, welded along the four arcs, `V = V(A)+V(B)−V(common)`.
  - **CUT (A−B)** — `buildSteinmetzCut` keeps A's OUTSIDE wall + A's caps + B's inside lunes
    emitted REVERSED (inward normal, bounding the carved channel through A), `V = V(A)−V(common)`.
  Verified vs **BOTH** the exact analytic inclusion-exclusion volumes (host) **and** OCCT
  `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all watertight/closed/valid, inside the 1%
  curved-parity bar — no tolerance weakened:
  - COMMON: volN = 5.3287, ΔV = 8.75e-04 (analytic `16 R³/3 = 5.33333`), ΔA = 4.68e-04.
  - FUSE:   volN = 32.385 vs OCCT 32.366, ΔV = 5.82e-04, ΔA = 4.07e-03.
  - CUT:    volN = 13.526 vs OCCT 13.516, ΔV = 7.22e-04, ΔA = 3.17e-03.
- **S5-e — coaxial cone(frustum)∩cylinder COMMON / FUSE / CUT** (the CONE surface family opened;
  changes `add-native-cone-boolean` (COMMON) + `complete-cone-cyl-fuse-cut` (FUSE + CUT), op-set
  now COMPLETE 3/3 native). All three share the SAME seam — a SINGLE closed S1-analytic circle
  (where the frustum's cross-section radius `r_c(s) = R0 + s·tanα` equals the cylinder radius `Rc`),
  `nearTangentGaps == 0`, no branch points, not passing through the apex — resampled into ONE pooled
  ring by the shared `coneCylSetup` prologue (exactly one `Cone` + one coaxial `Cylinder`, apex-free
  frustum extent, the seam `s*` strictly interior to the axial overlap). The split machinery is
  reused; the difference is which fragments survive + cap handling.
  - **COMMON** — `buildConeCylCommon` welds the min-radius-profile solid of revolution: a frustum
    band (below `s*`, inside the cylinder) welded to a cylinder-segment band (above `s*`, inside the
    cone) along the seam ring, closed by two disc caps (`appendRevolvedBand` + `appendDiskCap` +
    `VertexPool`). Byte-identical to the S5-e COMMON baseline.
  - **FUSE (A∪B)** — `buildConeCylFuse` keeps the OUTER wall regions of both operands (each band kept
    iff its mid-sample classifies strictly OUTSIDE the other solid) + the union's terminal disc caps
    + the annular step caps (`appendAnnulusCap`, fixed axial ±ẑ normal) where an end-cap disc
    protrudes, `V = V(A)+V(B)−V(A∩B)`.
  - **CUT (A−B, cone minuend, order-sensitive)** — `buildConeCylCut` keeps A's OUTER wall + A's
    terminal/annular caps outside B + the cylinder's INSIDE-A band emitted REVERSED (inward radial,
    bounding the carved cavity, pinching to the pooled seam ring) + B's end-cap disc inside A
    reversed. A DISCONNECTED solid (detached cone tip + conical washer — one shell of two closed
    components), `V = V(A)−V(A∩B)`.
  Verified vs a **DUAL oracle** — the analytic inclusion-exclusion closed form
  `V(A∩B) = V_frustum(rBot→Rc over [sLo,s*]) + V_frustum(Rc→rTop over [s*,sHi])`,
  `V_frustum(ra,rb,Δh) = (π Δh/3)(ra²+ra·rb+rb²)` (engine `ssiCurvedBooleanVerified` COMMON arm +
  the generic `booleanResultVerified` `V(A)+V(B)−V(A∩B)` / `V(A)−V(A∩B)` for FUSE/CUT with the native
  `buildConeCylCommon` as `V(A∩B)`, same 1% deflection-bounded tol) **AND** OCCT
  `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all watertight/closed/valid, no tolerance weakened:
  - COMMON: volN = 19.107 vs analytic 19.111355 vs OCCT 19.111, ΔV = 2.03e-04, ΔA = 9.89e-05.
  - FUSE:   volN = 41.618 vs analytic 41.62610 vs OCCT 41.626, ΔV = 2.04e-04, ΔA = 1.13e-04 (a GROW).
  - CUT:    volN = 13.349 vs analytic 13.35177 vs OCCT 13.352, ΔV = 2.03e-04, ΔA = 1.02e-04 (a SHRINK).
  The reversed `cyl − cone` CUT (wrong minuend) declines → NULL → OCCT; a mis-selected band /
  mis-oriented reversed fragment fails the self-verify and falls back — never faked.
- **S5-f — coaxial cone(frustum)∩sphere COMMON / FUSE / CUT** (the CONE∩SPHERE family; change
  `add-native-cone-sphere-boolean` archived `2026-07-07`, op-set now COMPLETE 3/3 native). All
  three share the SAME seam — a SINGLE closed S1-analytic circle (`intersectSphereConeCoaxial`, a
  QUADRATIC in the cone parameter with EXACTLY ONE root strictly interior to both extents — the
  sphere on the FRUSTUM side, so the seam does NOT cross the cone apex), `nearTangentGaps == 0`,
  `curveCount == 1` — resampled into ONE pooled ring by a shared `coneSphereSetup` prologue
  (exactly one `Cone` + one `Sphere`, the sphere centre ON the cone axis, apex-free frustum, the
  crossing `s*` strictly interior). The CONE side reuses the S5-e cone-wall split
  (`appendRevolvedBand` + `appendDiskCap`); the SPHERE side reuses the sphere-lens cap builder
  (`appendSphereCap`, inner/outer apex + reversed-normal flags); the two poles are CLASSIFIED
  against the cone into inner (inside) / outer (outside).
  - **COMMON** — `buildConeSphereCommon` welds the min-cross-section overlap: the cone wall band
    inside the sphere + the cone terminal disc inside the sphere + the sphere INNER cap (inside the
    cone, closing to the inner pole), all sharing the pooled seam ring. `V = V_frustum +
    V_spherical-segment` — a closed form.
  - **FUSE (A∪B)** — `buildConeSphereFuse` keeps the sphere OUTER cap (outside the cone, closing to
    the far pole) + the cone OUTER wall band (outside the sphere) + the cone terminal disc bounding
    the union, `V = V(A)+V(B)−V(A∩B)` (a GROW).
  - **CUT (A−B, cone minuend, order-sensitive)** — `buildConeSphereCut` keeps A's OUTER wall band +
    A's terminal disc cap(s) outside B + the sphere INNER cap emitted REVERSED (inward radial, the
    spherical dimple bounding the carved cavity, pinching to the pooled seam ring). Unlike the
    cone∩cylinder CUT this single-crossing CUT is CONNECTED (ONE closed component — a frustum with a
    spherical dimple), `V = V(A)−V(A∩B)` (a SHRINK). A sphere-minuend (`sphere − cone`) declines →
    NULL → OCCT.
  Verified vs a **DUAL oracle** — the analytic inclusion-exclusion closed form
  `V(A∩B) = V_frustum(r_c(sLo)→r_c(s*)) + V_spherical-segment(s*→pole)` (engine
  `ssiCurvedBooleanVerified` COMMON arm + the generic `booleanResultVerified` `V(A)+V(B)−V(A∩B)` /
  `V(A)−V(A∩B)` for FUSE/CUT with the native `buildConeSphereCommon` as `V(A∩B)`, same 1%
  deflection-bounded tol) **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all
  watertight/closed/valid, no tolerance weakened:
  - COMMON: volN = 5.2546 vs OCCT 5.2558, ΔV = 2.41e-04, ΔA = 1.28e-04.
  - FUSE:   volN = 60.686 vs OCCT 60.718, ΔV = 5.22e-04, ΔA = 2.61e-04 (a GROW).
  - CUT (cone−sphere): volN = 27.202 vs OCCT 27.207, ΔV = 1.96e-04, ΔA = 1.34e-04 (a SHRINK).
  The reversed `sphere − cone` CUT declines → NULL → OCCT; a TWO-circle crossing (sphere passes
  fully through the cone / spans the apex), an apex-crossing / apex-in-extent frustum, and a
  TRANSVERSAL (non-coaxial) cone∩sphere (a quartic space curve) all decline → NULL → OCCT.
- **S5-g — coaxial CONE(frustum)∩CONE(frustum) COMMON / FUSE / CUT** (the CONE∩CONE family; op-set
  now COMPLETE 3/3 native). Two COAXIAL cone frustums whose walls `r_A(s)=R0_A+s·tanα_A` and
  `r_B(s)=R0_B+s·tanα_B` (s = axial projection onto the SHARED axis) meet where `r_A(s)=r_B(s)` — a
  SINGLE LINEAR equation → EXACTLY ONE analytic circle seam at `s*=(R0_B−R0_A)/(tanα_A−tanα_B)`,
  radius `r*=r_A(s*)`. This is the natural GENERALISATION of the S5-e cone∩cylinder pair (a cylinder
  is the `tanα_B==0` special case): the S5-g `coneConeSetup` prologue re-expresses cone B's wall in
  cone A's s-frame (handling an antiparallel axis) and the constant cylinder radius `Rc` becomes the
  linear `r_B(s)`. All the S5-e revolved-band + disc-cap + annulus-cap machinery
  (`appendRevolvedBand` / `appendDiskCap` / `appendAnnulusCap`) is REUSED verbatim; the difference is
  only WHICH radius profile the second operand contributes. `nearTangentGaps==0`, `curveCount==1`,
  seam cross-checked (height s* + radius r*) against the S3 trace, single STRICTLY-INTERIOR crossing.
  - **COMMON** — `buildConeConeCommon` welds the min-radius profile solid of revolution
    `r ≤ min(r_A(s), r_B(s))` over the shared axial span: the narrower-wall band below s* + the seam
    ring + the narrower-wall band above s* + two disc caps. `V = V_frustum(below) + V_frustum(above)`.
  - **FUSE (A∪B)** — `buildConeConeFuse` walks the max-radius outer profile over the union span as a
    corner list, emitting a revolved wall band per different-s pair (the wider operand, kept iff its
    mid classifies OUTSIDE the other) + a flat annulus cap per radial STEP + two terminal discs.
    `V = V(A)+V(B)−V(A∩B)` (a GROW).
  - **CUT (A−B, cone-A minuend, order-sensitive)** — `buildConeConeCut` keeps A's WIDER (r_A>r_B)
    side of the seam: a conical WASHER (A wall outward + B wall emitted REVERSED inward, pinching to
    the seam, closed by a flat annulus cap at A's end) + any A-only slice where B is absent (a
    possibly-detached component). `V = V(A)−V(A∩B)` (a SHRINK).
  Verified vs a **DUAL oracle** — the analytic frustum inclusion-exclusion closed form
  (`V_frustum(ra,rb,Δh)=(π Δh/3)(ra²+ra·rb+rb²)`, the intersection circle radius/height closed-form
  from the two cone equations) **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all
  watertight/closed/valid, inside the 1% curved-parity bar, no tolerance weakened. Host fixture
  (cone A `r=0.5+0.5y` widening up, cone B `r=3.0−0.5y` narrowing up, both y∈[0,4], seam y*=2.5,
  r*=1.75): COMMON volN≈20.093 (analytic 20.093103), FUSE≈66.824 (66.824294, GROW), CUT (A−B)≈12.370
  (12.370021, SHRINK). CUT is order-sensitive — B−A is the OTHER single-sided washer
  (`V(B)−V(A∩B)`), a genuinely different watertight solid, confirming the minuend gate. A
  parallel-wall pair (equal half-angle → no proper transversal circle), an apex-crossing /
  apex-in-extent seam, and a TRANSVERSAL (non-coaxial) cone∩cone (a quartic space curve) all decline
  → NULL → OCCT.
- **S5-h — TWO-CIRCLE coaxial CONE(frustum)∩SPHERE COMMON / FUSE / CUT** (the natural extension of
  the single-circle S5-f pair; op-set now COMPLETE 3/3 native). A cone frustum coaxial with a sphere
  (centre ON the cone axis) whose wall crosses the sphere at TWO latitudes → TWO analytic circle
  seams s*_lo < s*_hi and a spherical ZONE between them — the "sphere pokes THROUGH the cone wall"
  pose. The seam quadratic `(1+tanA²)s² + 2(R0·tanA − sc)s + (sc²+R0²−Rs²)=0` now has BOTH roots
  strictly interior to the cone extent AND to the sphere's axial span, with the sphere the wider
  operand on the mid-band (`r_sphere > r_cone` between the seams) and inside beyond them (each polar
  cap inside the cone). Both circles are S1-analytic (radius ρ=r_cone(s*), station s*); the S3 tracer
  returns only ONE of the two co-resident loops (the documented S2 co-resident seeding-recall limit —
  see S4/NURBS §), so the `coneSphere2Setup` prologue computes BOTH circles itself and CROSS-CHECKS
  the traced seam(s) against the analytic roots (height + radius) — never trusting a missing loop, so
  a traced loop matching neither → decline. The two rings are canonical azimuth samples through a
  shared `VertexPool` so every band/cap/zone welds byte-identically. Machinery is REUSED: the CONE
  side is `appendRevolvedBand` + `appendDiskCap` (S5-e); each SPHERE polar cap is `appendSphereCap`
  (inner/outer + reversed flags, S5-c); the ONE new builder is `appendSphereZone` — a revolved sphere
  band between two seam rings, each meridian slerp-subdivided to follow the bulge (great-circle exact,
  pole-robust), for the FUSE mid-band.
  - **COMMON** — `buildConeSphere2Common`: the min-radius profile of revolution — sphere LOWER cap
    (poleM→seamLo, inside the cone) + cone frustum band (seamLo→seamHi, inside the sphere) + sphere
    UPPER cap (seamHi→poleP). `V = V_sph-seg + V_frustum + V_sph-seg` — a closed form. Symmetric.
  - **FUSE (A∪B)** — `buildConeSphere2Fuse`: cone wall (coneNear→seamLo) + the sphere ZONE bulge
    (seamLo→seamHi, the outward mid-band) + cone wall (seamHi→coneFar) + two cone terminal discs.
    `V = V(cone)+V(sphere)−V(COMMON)` (a GROW).
  - **CUT (A−B, cone MINUEND, order-sensitive)** — `buildConeSphere2Cut`: the sphere fully engulfs
    the cone cross-section on the mid-band, so the result PINCHES into TWO DISCONNECTED components
    welded into one shell — a lower cone-tip piece (coneNear→seamLo, scooped by the sphere lower cap
    REVERSED) + an upper piece (seamHi→coneFar, scooped by the sphere upper cap REVERSED).
    `V = V(cone)−V(COMMON)` (a SHRINK). A sphere-minuend `sphere − cone` declines → OCCT.
  Verified vs a **DUAL oracle** — the analytic inclusion-exclusion closed form (two spherical
  segments + one frustum; the two circles' radii/heights closed-form from the cone+sphere equations)
  **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all watertight/closed/valid, inside the 1%
  curved-parity bar, no tolerance weakened. Host fixture (cone r(y)=0.5+0.5y, y∈[0,4]; sphere Rs=1.6,
  centre (0,2,0); seams y*≈0.62026/2.17974): COMMON volN=14.670 (analytic 14.674986, ΔV≈3.3e-4),
  FUSE volN=34.937 (analytic 34.945423, ΔV≈2.4e-4, GROW), CUT volN=17.786 (analytic 17.788138,
  ΔV≈1.0e-4, SHRINK). A single-crossing sphere (the S5-f case, ONE interior root), a sphere internally
  tangent (double root → near-tangent), a pole outside the cone, an apex-crossing seam, and a
  non-coaxial (transversal quartic) cone∩sphere all decline → NULL → OCCT (honest, never faked).
- **S5-i — TWO-CIRCLE coaxial CYLINDER(finite)∩SPHERE COMMON / FUSE / CUT** (the natural `tanα==0`
  special case of the S5-h cone∩sphere two-circle family — a cylinder is a cone with zero half-angle;
  op-set COMPLETE 3/3 native). A finite cylinder (radius Rc, axis coaxial with a sphere Rs > Rc whose
  centre lies ON the cylinder axis) whose wall crosses the sphere at TWO latitudes → TWO analytic
  circle seams and a spherical zone between them — the "sphere pokes THROUGH the cylinder wall" pose.
  The seam equation is EXACT and clean: `Rc = √(Rs²−(s−sc)²)` ⇒ `s = sc ± √(Rs²−Rc²) = sc ± h`, two
  circles of the SAME radius Rc at stations sLo=sc−h, sHi=sc+h. On the mid-band the sphere is the wider
  operand (bulges outside the cylinder); each polar cap is inside the cylinder, both poles strictly
  inside the cylinder's axial extent. Both circles are S1-analytic (the coaxial sphere∩cyl → 2-circle
  S1 pair); the S3 tracer returns ONE of the two co-resident loops (the documented S2 co-resident
  seeding-recall limit), so the `cylSphere2Setup` prologue computes BOTH circles itself and
  CROSS-CHECKS the traced seam(s) against the analytic roots (height + radius) — a traced loop matching
  neither → decline. NO new builder — the shared `appendSphereZone` was refactored to a
  centre+radius primitive (the S5-h wrapper delegates BIT-IDENTICALLY) so BOTH families share ONE zone
  builder; the cylinder wall is `appendRevolvedBand` (a straight ruling is EXACT on a cylinder) +
  `appendDiskCap` (S5-a/e); each SPHERE polar cap is `appendSphereCap` (inner/outer + reversed, S5-c).
  - **COMMON** — `buildCylSphere2Common`: the min-radius profile of revolution — sphere LOWER cap
    (poleM→seamLo, inside the cyl) + cylinder segment band (seamLo→seamHi, inside the sphere) + sphere
    UPPER cap (seamHi→poleP). `V = V_sph-seg + π·Rc²·(sHi−sLo) + V_sph-seg`. Symmetric.
  - **FUSE (A∪B)** — `buildCylSphere2Fuse`: cylinder wall (cylNear→seamLo) + the sphere ZONE bulge
    (seamLo→seamHi, the outward mid-band) + cylinder wall (seamHi→cylFar) + two cylinder terminal
    discs. `V = V(cyl)+V(sphere)−V(COMMON)` (a GROW).
  - **CUT (A−B, cylinder MINUEND, order-sensitive)** — `buildCylSphere2Cut`: the sphere fully engulfs
    the cylinder cross-section on the mid-band, so the result PINCHES into TWO DISCONNECTED components
    welded into one shell — a lower cyl-end piece (cylNear→seamLo, scooped by the sphere lower cap
    REVERSED) + an upper piece (seamHi→cylFar, scooped by the sphere upper cap REVERSED).
    `V = V(cyl)−V(COMMON)` (a SHRINK). The sphere-minuend `sphere − cyl` now LANDS via
    `buildSphereCyl2Cut` (BOOL-COMPLETE): the sphere with a coaxial cylindrical TUNNEL — the sphere
    equatorial belt (`appendSphereZone` between the two seams, outward) + the reversed cylinder bore
    (`appendRevolvedBand`, inward), a single annular solid of revolution, watertight at `V_sph−V_common`.
  Verified vs a **DUAL oracle** — the analytic inclusion-exclusion closed form (two spherical segments
  + one cylinder segment; the two circles' radii/heights closed-form from the cylinder+sphere
  equations) **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all watertight/closed/valid, inside
  the 1% curved-parity bar, no tolerance weakened. Host+sim fixture (cylinder Rc=1.0 about +Y,
  y∈[−3,3]; sphere Rs=1.6, centre origin; seams y*=±√1.56≈±1.24900, radius 1.0; the S3 tracer here
  recovers BOTH co-resident circles, `curveCount=2`):
  - COMMON: volN=8.9937 vs analytic 8.995681 vs OCCT 8.9957, ΔV=2.24e-04, ΔA=9.61e-05.
  - FUSE:   volN=27.001 vs analytic 27.011160 vs OCCT 27.011, ΔV=3.76e-04, ΔA=1.65e-04 (a GROW).
  - CUT (cyl−sphere): volN=9.8521 vs analytic 9.853875 vs OCCT 9.8539, ΔV=1.84e-04, ΔA=1.07e-04
    (a SHRINK, two disconnected end pieces).
  A sphere with Rs ≤ Rc (no proper
  two-circle crossing — internally tangent / nested), a sphere whose pole falls outside the cylinder's
  axial extent (a single-crossing end dent), and a non-coaxial (off-axis, transversal quartic)
  cyl∩sphere all decline → NULL → OCCT (honest, never faked).
- **S5-j — coaxial HOURGLASS (apex-to-apex / bowtie) CONE(frustum)∩CONE COMMON / FUSE / CUT** (the
  genuinely-different sibling of the S5-g coaxial cone∩cone frustum pair; op-set now COMPLETE 3/3
  native). Two COAXIAL cones pointing AT each other (bowtie): cone A ▽ opens downward to an APEX and
  cone B △ grows from an apex. Their walls still cross at EXACTLY ONE analytic circle s* (both linear
  in the shared axial coordinate → the S5-g `ConeConeSetup` prologue is REUSED VERBATIM), but the
  COMMON's min-radius profile `r ≤ min(r_A(s), r_B(s))` PINCHES to the AXIS (a cone apex, r→0) at ONE
  or BOTH overlap ends, so the S5-g COMMON/CUT apex gates (`rBot/rTop/rBCap>0`) DECLINE that pose.
  S5-j is the apex-terminated assembler: an apex end is closed by a REVOLVED BAND onto a degenerate
  apex ring (N coincident apex points → `appendRevolvedBand` emits a watertight cone-tip fan, the
  axis-side sliver of each quad dropped by `pushPlanarTri`, the apex vertex dedup'd through the
  `VertexPool`); an off-axis end keeps the flat `appendDiskCap`. The ONE new primitive is `apexRing`
  (the degenerate terminal). The seam ring stays the SHARED analytic circle so every band/cap welds
  byte-clean.
  - **COMMON** — `buildHourglassConeConeCommon`: the bicone (two cones apex-to-apex sharing the seam
    ring) / apex-terminated min-radius profile of revolution. Handles both the double-apex bicone AND
    the mixed one-apex (disc-cap + apex-band) case. `V = V_frustum(rBot, r*, s*−sLo) + V_frustum(r*,
    rTop, sHi−s*)`, each apex end a full cone (rBot or rTop = 0). Symmetric.
  - **FUSE (A∪B)** — the bowtie max-radius profile has OFF-axis terminal discs (a waist at the seam,
    not an apex), so the S5-g `buildConeConeFuse` builds it directly (dispatched below S5-j).
    `V = V(A)+V(B)−V(COMMON)` (a GROW).
  - **CUT (A−B, cone-A minuend, order-sensitive)** — `buildHourglassConeConeCut`: A keeps its wider
    side of the seam — a conical SHELL whose outer boundary is A's wall and inner boundary is B's wall
    REVERSED running into B's OWN apex (r_B→0), closed by a FULL A-end disc (B absent there — the pose
    S5-g's annulus-cap CUT, `rBCap>0`, declines). `V = V(A)−V(COMMON)` (a SHRINK).
  Verified vs a **DUAL oracle** — the analytic frustum inclusion-exclusion closed form (a full cone is
  `V_frustum(r,0,Δh)`; the seam radius/height closed-form from the two cone equations) **AND** OCCT
  `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all watertight/closed/valid, inside the 1% curved-parity bar,
  no tolerance weakened. Host fixtures — symmetric bowtie (cone A r=2−y, cone B r=y, both y∈[0,2], seam
  y*=1, r*=1): COMMON volN≈2.09397 (analytic 2π/3≈2.09440, ΔV≈4e-4), FUSE≈14.65778 (14π/3≈14.66077,
  GROW), CUT (A−B)≈6.28191 (2π≈6.28319, SHRINK); plus a one-apex hourglass (cone A r=2−y apex, cone B
  r=0.5+y off-axis base, seam y*=0.75/r*=1.25) exercising the mixed disc-cap+apex-band COMMON/CUT. CUT
  is order-sensitive (B−A is the other conical shell). A both-ends-off-axis frustum pair is left to
  S5-g; parallel-wall / non-coaxial (transversal quartic) cone∩cone still decline → NULL → OCCT.
- **S5-k — TRANSVERSAL (NON-COAXIAL) CYLINDER∩SPHERE COMMON** (the FIRST transversal / non-coaxial
  curved-boolean slice — the genuine frontier beyond the coaxial S5 families). Where S5-i handles
  the COAXIAL finite-cylinder∩sphere pose (sphere centre ON the cylinder axis → two ANALYTIC circle
  seams, PLANAR rings), S5-k handles the OFFSET pose: the cylinder axis is PARALLEL to but
  perpendicular-DISPLACED from the sphere centre, so the two seams are NON-PLANAR closed space
  curves (generalised Viviani curves) — NO analytic circle exists. The seam is consumed DIRECTLY
  from the S3 `TraceSet` (the general SSI freeform-seam machinery the roadmap names), not re-derived
  from a closed form. Pose: a THIN cylinder (Rc < Rs) whose offset axis still pierces BOTH poles of
  the sphere (`offset + Rc < Rs`) → the wall crosses the sphere in exactly TWO disjoint closed loops
  (a lower + an upper loop along the axis), both fully transversal (`nearTangentGaps==0`,
  `branchPoints==0`, both Closed). COMMON is the SAME TOPOLOGY as the coaxial S5-i COMMON — a
  cylinder mid-band capped by two spherical caps — but every ring is the TRACED non-planar seam:
  `buildTransCylSphereCommon` welds the sphere LOWER cap (inside the cyl) + the cylinder BAND
  (seamLo→seamHi, inside the sphere) + the sphere UPPER cap (inside the cyl) through one `VertexPool`.
  The new prologue `transCylSphereSetup` recognises the pierce-both-poles offset pose (offset > tol,
  parallel axis, Rc < Rs, both loops interior + the inside-the-other survival samples) and resamples
  BOTH traced seams onto a COMMON cylinder-azimuth grid (`resampleByAzimuth`) so the band welds
  ring-to-ring; the cap builder `appendSphereCap` already slerps an apex to ARBITRARY non-planar seam
  nodes (S5-c), and the band reuses `appendRevolvedBand` on the aligned rings (a straight ruling is
  exact on the cylinder). REDUCTION: as the offset → 0 the pose becomes coaxial and S5-i's
  `cylSphere2Setup` claims it FIRST in the dispatch (S5-k gates on a STRICTLY-POSITIVE offset),
  reproducing the landed coaxial COMMON — a pinned regression. Verified vs a deterministic
  numerical-integration oracle (no closed form exists for a non-analytic seam): host fixture
  (Y-cylinder Rc=1.0 over y∈[−3,3]; sphere Rs=2.0 centre (0.5,0,0), offset 0.5), COMMON volN≈11.275
  vs numeric 11.279, ΔV≈3.5e-4, watertight, symmetric (both operand orders), inside the 1%
  curved-parity bar — no tolerance weakened. **CUT/FUSE now LAND (TRANS-BAND).** They need the sphere
  OUTER SHELL (the sphere ZONE between the two NON-PLANAR seams, the long way round outside the bore),
  and the new primitive `appendSphereOuterZoneBetweenSeams(C, Rs, axis, seamHi, seamLo, …)` tiles it
  **exactly on-surface**: each seam node decodes to (θ around the cyl axis, φ from it); the two
  index-aligned nodes share θ, and the zone is swept at CONSTANT θ by interpolating φ LINEARLY from
  φ_hi (near the +axis pole) to φ_lo (near −axis) — CROSSING THE EQUATOR the far way, never near a
  pole (the raw 3-D great-circle slerp is ambiguous for the near-antipodal seam pair and picks the
  SHORT arc through the bore — the corrected long-way sweep does not). Every interior row is placed at
  `C + unit(dir)·Rs` (on-surface to machine precision) and the outer rows are the pooled seam nodes, so
  the zone welds ring-to-ring to the reversed cyl bore (CUT sphere−cyl) or the cyl end stubs (FUSE).
  Verified vs the numeric oracle (V(A)/V(B) − numeric COMMON): CUT sph−cyl 22.213 vs 22.235 (0.10%),
  CUT cyl−sph 7.569 vs 7.574 (0.07%), FUSE 41.058 vs 41.085 (0.07%) — all watertight, ≪ 1% bar. Gated
  on **both sphere poles strictly inside the cylinder** (`polesInsideCyl`) so the two-cap+outer-zone
  topology holds; a pole-grazing thin cylinder (offset+Rc→Rs, a pole falls outside the cyl) keeps the
  honest CUT/FUSE decline → OCCT (regression-pinned). The COMMON residual map (measured): the S3/S2
  co-resident seeding recall returns BOTH loops only up to offset ≈ 0.5·Rc; at larger offsets ONLY ONE
  loop is traced (the documented co-resident seeding-recall limit), so the two-loop gate declines
  beyond that. A grazing / internally-tangent offset (NearTangent), a fat cylinder (Rc ≥ Rs), a
  cylinder that does not pierce both poles (offset+Rc ≥ Rs, single loop), and a skew (non-parallel)
  axis all decline → NULL → OCCT.
- **MULTI-HOLE-SPLIT — general holed-face second-seam split** (`src/native/boolean/holed_face_split.h`,
  `splitFaceSmoothTrimHoled`; the enabler the S5-k/p/q/s CUT/FUSE residual + the BOOL-READMIT
  genuine-overlap ≥3 weld both name). ◐ CAPABILITY LANDED + GREEN; transversal shell-wiring DEFERRED.
  The verb splits a face carrying **N ≥ 1 existing hole loops** by ONE closed interior second seam into
  two genuinely-trimmed sub-faces that PRESERVE every existing hole, tiling the parent NET
  (holes-subtracted) area exactly: `faceInside` = the seam-enclosed region (seam as outer wire + every
  NESTED existing hole reused verbatim); `faceOutside` = the parent outer wire + the REVERSED seam as a
  hole + every non-nested existing hole. The seam is built ONCE, laid FORWARD on faceInside /
  REVERSED-as-hole on faceOutside ⇒ their bit-identical shared boundary (the smooth-trim
  watertight-share idiom); result is a `SmoothFaceSplit` the frozen `pickByMembership` consumes
  unchanged. Honest declines (nullopt, never leaky, no tolerance weakened): NoHole (hole-free →
  `splitFaceSmoothTrim`), SeamNotInterior/NotClosed/TooShort, SelfIntersecting, **SeamCrossesHole** (the
  genuinely-harder multi-crossing case, out of this two-region-per-seam slice), DegenerateSubRegion,
  TilingGap. Host gate `test_native_holed_face_split.cpp` **8/8 GREEN** on a planar-annulus identity-UV
  oracle: single-hole annulus (hole preserved, exact π-net tiling, sub-faces MESH+TILE+weld the seam 1:1
  by radius-localized boundary count, contrast vs `splitFaceSmoothTrim` DROPPING the hole), the general
  **≥2-hole** (seam encloses one → 1 in/1 out) and **≥3-hole** (seam encloses two → 2 nested in/1 out)
  cases, + the four decline branches. `boolean_readmit.h::splitAccWall` routes an already-holed acc wall
  through it (readmit 5/5). `src/native` OCCT-free; no `cc_*` ABI.
  **UPDATE (TRANS-BAND landed the sphere-outer-zone families):** the S5-k/S5-q CUT/FUSE weld is the
  shell primitive `appendSphereOuterZoneBetweenSeams` (a sphere BAND between the two index-aligned
  `resampleByAzimuth`-gridded non-planar loops, tiled on-surface by the cyl/cone-axis θ/φ sweep the
  LONG way over the equator — see the S5-k / S5-q entries). It confirms the holed-face verb's role as
  UV-region reasoning while the actual weld is this facet-shell band, and it retires the deferral for
  the two SPHERE-outer-zone families.
  **UPDATE (TUBE-BAND landed S5-p + S5-s CUT/FUSE):** the two deferred transversal families now land
  their CUT/FUSE — but NOT via a between-seams band (the sphere trick), because MEASUREMENT overturned
  the earlier framing: an offset cylinder pierces the torus tube / cone wall on ONE side, so the seams
  are LOCALIZED in the pierced surface's (u,v) (torus u∈[−0.2,0.2]), NOT azimuth-wrapping. The outer
  zone is therefore the FULL pierced surface MINUS the seam cap patch(es) — a HOLED surface. It is
  tiled EXACTLY on-surface by a **(u,v)-grid + loop-zipper hole-split** (`appendTorusTubeOuterZone` for
  the torus, `appendConeWallOuterZone` for the cone wall in AXIAL cone coords): mesh the full surface
  grid, drop the cells inside a seam loop (tight jagged hole hugging the seam ≤1 cell), chain the hole
  boundary into an ordered ring, zip it to the exact traced seam. **S5-p** torus∩cyl (2 holes) and the
  cone−cyl / FUSE leg of **S5-s** cone∩cyl (1 hole) both weld watertight, ΔV <0.8% (COMMON <0.2%);
  cyl−cone is a clean seam-driven cyl stub (no hole). Poses where a seam is not localizable clear of
  the rims / u-wrap HONEST-DECLINE → OCCT; DISAGREED=0 / never-leaky stays SACRED.
- **S5-l — coaxial TORUS(ring)∩CYLINDER COMMON / FUSE / CUT** (the TORUS surface family opened;
  op-set COMPLETE 3/3 native). A ring torus (major R, minor r, axis) and a COAXIAL cylinder (radius
  Rc, same axis) whose wall crosses the torus TUBE at TWO latitudes → TWO analytic circle seams. In
  the meridian (ρ,z) plane the tube is the disk of radius r centred at (R,0) and the cylinder is the
  vertical chord ρ = Rc; the chord cuts the tube iff |Rc − R| < r, giving `cos v0 = (Rc − R)/r` and
  the two seam circles at stations z = ±z0 = ±√(r² − (Rc − R)²), both of radius Rc. `recogniseCurvedSolid`
  now admits a fourth `CurvedKind::Torus` — a bare doubly-periodic `Kind::Torus` face (the STEP-import
  form; a native revolve builds a torus as B-spline bands, which still decline), ring-torus-only
  (R > r > 0; a self-intersecting spindle torus R ≤ r is declined at recognition). The S3 tracer
  returns ONE of the two co-resident loops (the documented S2 co-resident seeding-recall limit), so
  the `torusCylSetup` prologue computes BOTH circles itself and CROSS-CHECKS the traced seam(s)
  against the analytic roots (station + radius) — a traced loop matching neither → decline. Every op
  is a SOLID OF REVOLUTION welded from the S5-e…j machinery — `appendRevolvedBand` (the cylinder
  chord band + the tube-arc bands, each oriented by the TRUE tube-outward normal radiating from the
  tube-centre circle, correct on the tube's INNER half where the axis-radial reference would invert)
  + flat disc caps — through one `VertexPool`.
  - **COMMON** — `buildTorusCylCommon`: the ρ ≤ Rc part of the tube — the INNER tube arc
    (v ∈ [v0, 2π−v0], through the inner equator ρ = R−r) + the cylinder chord band (z ∈ [−z0, z0]).
  - **FUSE (A∪B)** — `buildTorusCylFuse`: the OUTER tube-arc bulge (v ∈ [−v0, v0], ρ > Rc) + the
    cylinder wall OUTSIDE the tube (z ∈ [cylEnd, ∓z0]) + two cylinder terminal disc caps. The
    cylinder fills the donut hole → the union is simply connected. A GROW.
  - **CUT (A−B, TORUS minuend, order-sensitive)** — `buildTorusCylCut`: the ρ > Rc outer tube ring
    (outer arc + the cylinder chord band REVERSED, bounding the carved bore). A SHRINK, one closed
    ring-of-revolution component. The cylinder-minuend `cyl − torus` now LANDS via `buildCylTorusCut`
    (BOOL-COMPLETE): the finite cylinder with a concave toroidal GROOVE carved into its wall
    (z ∈ [−z0, z0]) — bottom disc + wall stub + the INNER tube arc REVERSED + wall stub + top disc,
    a grooved-cylinder solid of revolution, watertight at `V_cyl − V_common`.
  Verified vs a **DUAL oracle** — the AIRTIGHT Pappus closed forms (engine `ssiCurvedBooleanVerified`
  S5-l arm for COMMON: `V = 2π·(R·A_seg + M)`, `A_seg = πr² − (r²·acos(d/r) − d·√(r²−d²))`,
  `M = −(2/3)(r²−d²)^{3/2}`, `d = Rc − R`; the generic `booleanResultVerified` `V_torus + V_cyl − V_common`
  / `V_torus − V_common` for FUSE/CUT with `V_torus = 2π²Rr²`) **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}`
  (sim), all watertight/closed/valid, inside the 1% curved-parity bar, no tolerance weakened. Host+sim
  fixture (torus R=3, r=1, axis +Z; cylinder Rc=3.2, z∈[−2,2]; seams z*=±√0.96≈±0.98, radius 3.2):
  - COMMON: volN=33.059 vs analytic 33.158 vs OCCT 33.158, ΔV=2.98e-03, ΔA=9.81e-04.
  - FUSE:   volN=154.45 vs OCCT 154.74, ΔV=1.86e-03, ΔA=1.20e-03 (a GROW).
  - CUT (torus−cyl): volN=25.986 vs OCCT 26.06, ΔV=2.84e-03, ΔA=9.34e-04 (a SHRINK).
  A SPINDLE / self-intersecting torus (R ≤ r) declines at recognition; a cylinder that clears the
  tube (Rc inside the donut hole or beyond the outer equator, |Rc − R| ≥ r → no proper two-circle
  crossing), a cylinder tangent to an equator, a short cylinder not axially spanning the tube, and a
  non-coaxial (off-axis / skew) torus∩cylinder all decline → NULL → OCCT (honest, never faked).
- **S5-m — coaxial TORUS(ring)∩SPHERE COMMON / FUSE / CUT** (the SECOND torus-family pair; op-set
  COMPLETE 3/3 native). A ring torus (major R, minor r, axis = frame Z, centre O) and a sphere
  (radius Rs) whose centre sits ON the torus axis AT the torus centre (sc = 0 — the CLEANEST-oracle
  SYMMETRIC pose). In the meridian (ρ,z) plane the tube is the disk of radius r centred at (R,0); the
  sphere is the circle ρ²+z²=Rs². Both meet at the SAME radius ρ* = (R²−r²+Rs²)/(2R) and z = ±z0 =
  ±√(r²−(ρ*−R)²) → TWO seam circles of EQUAL radius ρ* (like the S5-l cylinder chord, but the seam
  radius is derived from the sphere). For |z| ≤ z0 the sphere arc lies INSIDE the tube walls; for
  |z| > z0 the tube slice is entirely OUTSIDE the ball. Every op is a Pappus-exact solid of
  revolution welded from the S5-e…l machinery through one `VertexPool` — the revolved tube arcs
  (`appendTubeArc` topology, S5-l), the SPHERE ZONE between the two seam rings (`appendSphereZone`,
  S5-h/i — the ONLY change is a `outwardSign` param added so the CUT can reverse it, default +1 →
  every existing caller byte-identical), and the two sphere polar CAPS beyond the seams
  (`appendSphereCap`, S5-c). The tube-arc topology is IDENTICAL to S5-l (COMMON = inner arc; CUT =
  outer arc); only the CYLINDER chord band becomes the SPHERE zone/caps. The S3 tracer returns ONE
  of the two co-resident loops, so the `torusSphereSetup` prologue computes BOTH circles itself and
  CROSS-CHECKS the traced seam(s) against the analytic roots (station ±z0 + radius ρ*).
  - **COMMON** — `buildTorusSphereCommon`: the tube ∩ ball — the INNER tube arc (v ∈ [v0, 2π−v0],
    through the inner equator ρ = R−r) + the sphere zone ρ = √(Rs²−z²) between the two seam rings.
  - **CUT (A−B, TORUS minuend, order-sensitive)** — `buildTorusSphereCut`: the tube ∖ ball — the
    OUTER tube arc (v ∈ [−v0, v0], through the outer equator, covering the pole-region tube where
    |z| > z0) + the sphere zone REVERSED (inward normal, bounding the carved cavity). A SHRINK, one
    closed ring-of-revolution component. The sphere-minuend `sphere − torus` now LANDS via
    `buildSphereTorusCut` (BOOL-TAIL): the ball with a concave TOROIDAL GROOVE scooped into its
    equatorial belt — a hole-split of the OUTER (sphere) surface into the two sphere polar caps
    (the ball OUTSIDE the tube, the SAME caps the FUSE emits; the belt between the seams sits inside
    the tube and is removed) welded to the INNER tube arc REVERSED as the groove wall. Watertight,
    partition-correct, ΔV <1% vs `V_sph − V_common`; order-sensitive but tractable.
  - **FUSE (A∪B)** — `buildTorusSphereFuse`: the OUTER tube-arc bulge (ρ > ρ*, outside the ball) +
    the TWO sphere polar caps (each seam ring out to its pole, the sphere surface OUTSIDE the tube).
    The ball fills the donut hole + mid-band → the union is simply connected. A GROW.
  Verified vs a **DUAL oracle** — the AIRTIGHT Pappus closed forms (engine `ssiCurvedBooleanVerified`
  S5-m arm for COMMON: `V = 2π·[ (Rs²−R²−r²)·z0 + R·(z0·√(r²−z0²) + r²·asin(z0/r)) ]`; the generic
  `booleanResultVerified` `V_torus + V_sph − V_common` / `V_torus − V_common` for FUSE/CUT with
  `V_torus = 2π²Rr²`, `V_sph = 4/3·πRs³`) **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all
  watertight/closed/valid, inside the 1% curved-parity bar, no tolerance weakened. Host+sim fixture
  (torus R=3, r=1, axis +Z; sphere Rs=3.0 centred at origin; seams z*=±√(r²−(ρ*−R)²)≈±0.986,
  radius ρ*=17/6≈2.833):
  - COMMON: volN≈23.29 vs analytic 23.355 vs OCCT 23.355, ΔV≈2.8e-03 (a facet-inscription deficit).
  - FUSE:   volN≈148.62 vs OCCT 148.96, ΔV≈2.3e-03 (a GROW; hole filled by the ball).
  - CUT (torus−sphere): volN≈35.74 vs OCCT 35.86, ΔV≈3.5e-03 (a SHRINK, one ring-of-revolution component).
  A SPINDLE torus (R ≤ r) declines at recognition; an OFF-CENTRE coaxial sphere (sc ≠ 0 — the general
  spiric section with unequal seam radii), an OFF-AXIS sphere, a small sphere inside the donut hole,
  and a large sphere engulfing the inner tube (|ρ*−R| ≥ r → no proper two-circle crossing) all
  decline → NULL → OCCT (honest, never faked). The general OFF-CENTRE coaxial (sc ≠ 0) spiric section
  is the next torus∩sphere slice.
- **S5-n — coaxial TORUS(ring)∩CONE COMMON / FUSE / CUT** (the THIRD torus-family pair; op-set
  COMPLETE 3/3 native). A ring torus (major R, minor r, axis, centre O) and a COAXIAL cone
  (apex/axis on the shared axis, half-angle α) whose OBLIQUE wall crosses the torus TUBE at TWO
  latitudes → TWO analytic circle seams. This is the **oblique-chord generalisation of S5-l**: in
  the meridian (ρ,z) plane the tube is the disk of radius r centred at (R,0) and the cone wall is
  the SLANTED line ρ = a + b·z (b = ±tanα), where the S5-l cylinder is the b=0 vertical chord. The
  line cuts the tube-boundary circle (ρ−R)²+z²=r² where (1+b²)z² + 2b(a−R)z + (a−R)²−r² = 0 → two
  distinct real roots z1<z2, seam radii ρ_i = a+b·z_i — two analytic circles at DIFFERENT radii AND
  different axial stations (two surfaces of revolution about one axis meet in circles). The
  `torusConeSetup` prologue recognises the coaxial cone (`CurvedKind::Cone`, coaxial gate),
  computes BOTH seam circles from the seam quadratic and CROSS-CHECKS every S3-traced loop
  (station + radius) against them (a traced loop matching neither → decline). Every op is a
  Pappus-exact solid of revolution welded from the S5-l machinery — the revolved tube arc
  (`appendTorusConeTubeArc`, the S5-l tube-arc topology) + a **single revolved cone-chord band**
  (`appendRevolvedBand`; a straight ruling is EXACT on a cone wall, so the slanted band between the
  two seam rings is exact) + `appendAxisDiscCap` cone terminal discs (FUSE only) — through one
  `VertexPool`.
  - **COMMON** — `buildTorusConeCommon`: the ρ ≤ line part of the tube — the INNER tube arc
    (through the inner equator ρ=R−r) closed by the slanted cone chord band between the two seam
    rings. A closed watertight surface of revolution (no caps, exactly the S5-l COMMON topology).
  - **CUT (A−B, TORUS minuend, order-sensitive)** — `buildTorusConeCut`: the ρ > line OUTER tube
    arc (through the outer equator) + the cone chord band REVERSED (inward normal). A SHRINK, one
    closed ring-of-revolution component. The cone-minuend `cone − torus` now LANDS via
    `buildConeTorusCut` (BOOL-TAIL): the cone with a concave TOROIDAL GROOVE scooped into its
    slanted wall — the two cone wall stubs (coneS0→z1 seam, z2 seam→coneS1, the cone OUTSIDE the
    tube) + the two cone terminal discs welded to the INNER tube arc REVERSED as the groove wall.
    Watertight, partition-correct, ΔV <1% vs `V_cone − V_common`; order-sensitive but tractable.
  - **FUSE (A∪B)** — `buildTorusConeFuse`: the cone frustum fills the donut hole → simply
    connected: the cone terminal disc @ coneS0, the cone wall coneS0→z1 seam, the OUTER tube-arc
    bulge (ρ > line) between the seams, the cone wall z2 seam→coneS1, the cone terminal disc @
    coneS1. A GROW.
  Verified vs a **DUAL oracle** — the AIRTIGHT Pappus closed form (engine `ssiCurvedBooleanVerified`
  S5-n arm; working about the tube centre ρ'=ρ−R the cone chord has unit normal m̂=(1,−b)/√(1+b²)
  into the discarded ρ>line region and signed offset t0=(a−R)/√(1+b²); discarded segment
  `A_d = r²·acos(t0/r) − t0·√(r²−t0²)`, discarded ρ'-moment `(1/√(1+b²))·(2/3)(r²−t0²)^{3/2}`, so
  the KEPT segment `A_seg = πr² − A_d`, `M = −(1/√(1+b²))·(2/3)(r²−t0²)^{3/2}`, and by Pappus
  `V_common = 2π·(R·A_seg + M)` — which REDUCES to the S5-l torus∩cylinder closed form at b=0; the
  generic `booleanResultVerified` `V_torus + V_cone − V_common` / `V_torus − V_common` for FUSE/CUT
  with `V_torus = 2π²Rr²`) **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all
  watertight/closed/valid, inside the 1% curved-parity bar, no tolerance weakened. Host+sim fixture
  (torus about +Y, R=3, r=1; coaxial cone radius(z)=3.2+0.5·z over z∈[−2,2]; seams z1=−0.96 ρ=2.72,
  z2=0.8 ρ=3.6):
  - COMMON: volN≈32.66 vs analytic≈32.76 vs OCCT, ΔV≈0.3% (a facet-inscription deficit).
  - FUSE:   volN≈159.07 vs analytic≈159.32, ΔV≈0.16% (a GROW; cone fills the hole).
  - CUT (torus−cone): volN≈26.40 vs analytic≈26.46, ΔV≈0.2% (a SHRINK, one ring-of-revolution component).
  A SPINDLE torus (R ≤ r) declines at recognition; a near-cylindrical cone (tanα≈0 — the S5-l
  cylinder path owns it), a cone whose slant chord clears / is tangent to the tube (no proper
  two-circle crossing), a cone not axially spanning past both seams, and a non-coaxial / off-axis /
  skew cone all decline → NULL → OCCT (honest, never faked). The general TRANSVERSAL (non-coaxial)
  torus∩cone quartic space curve is a later slice.
- **S5-o — coaxial TORUS(ring)∩TORUS(ring) COMMON / FUSE / CUT** (the FOURTH torus-family pair and
  the FIRST curved∩curved slice where BOTH operands are tori; op-set COMPLETE 3/3 native). Two
  COAXIAL ring tori sharing an axis: torus A (major R1, minor r1, tube-centre circle at station zA)
  and torus B (major R2, minor r2, tube-centre at zB). In the meridian (ρ,z) plane BOTH tubes are
  DISKS — tube A radius r1 centred (R1,zA), tube B radius r2 centred (R2,zB) — and two circles in a
  plane meet in up to TWO points, so a proper poke-through gives TWO analytic seam circles at
  DIFFERENT radii AND stations (like S5-n, but now the second wall is ALSO a tube arc, not a flat
  cone chord). Seam solve is the classic two-circle intersection: centre distance D, chord offset
  a = (D²+r1²−r2²)/(2D) from A along the centre line, half-chord h = √(r1²−a²), the two seam points
  M ± h·û⊥ (M = A + a·û). The `torusTorusSetup` prologue recognises the coaxial torus pair
  (`CurvedKind::Torus` on both, coaxial gate), solves BOTH seam circles closed-form, and
  CROSS-CHECKS every S3-traced loop (station + radius) against them (a traced loop matching neither
  → decline). Every op is a Pappus-exact solid of revolution welded from the S5-l machinery — BOTH
  boundary walls are revolved tube arcs (`appendTorusTorusTubeArc`, the S5-l tube-arc topology on
  either tube, oriented by the TRUE tube-outward normal radiating from each tube-centre circle);
  NO flat chord band, NO caps — through one `VertexPool`.
  - **COMMON (A∩B)** — `buildTorusTorusCommon`: the revolved LENS (disk A ∩ disk B) — the INNER arc
    of tube A (inside B) + the INNER arc of tube B (inside A), both outward. A closed watertight
    ring of revolution.
  - **FUSE (A∪B)** — `buildTorusTorusFuse`: the revolved UNION (disk A ∪ disk B) — the OUTER arc of
    tube A (outside B) + the OUTER arc of tube B (outside A), both outward. A GROW; one ring of
    revolution. `V = V_torusA + V_torusB − V_common`.
  - **CUT (A−B, TORUS-A minuend, order-sensitive)** — `buildTorusTorusCut`: disk A ∖ lens — the
    OUTER arc of tube A + the INNER arc of tube B REVERSED (inward normal, bounding the carved
    bite). A SHRINK, one ring of revolution. `V = V_torusA − V_common`. B−A is a DIFFERENT solid
    (torus-B minuend) — built by SWAPPING operands (COMMON/FUSE are symmetric, CUT is order-sensitive).
  Verified vs a **DUAL oracle** — the AIRTIGHT Pappus closed form (engine `ssiCurvedBooleanVerified`
  S5-o arm for COMMON: the revolved lens = 2π times the sum of the two circular-segment
  first-moments about the axis; the equal-h segment moment terms CANCEL, leaving
  `V_common = 2π·(R1·A_segA + R2·A_segB)`, `A_segA = r1²·acos(a/r1) − a·h`,
  `A_segB = r2²·acos((D−a)/r2) − (D−a)·h`; the generic `booleanResultVerified`
  `V_torusA + V_torusB − V_common` / `V_torusA − V_common` for FUSE/CUT with `V_torus = 2π²Rr²`)
  **AND** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all watertight/closed/valid, inside the 1%
  curved-parity bar, no tolerance weakened. Host+sim fixture (torus A R1=3 r1=1 centre z=0; torus B
  R2=3.4 r2=0.9 centre z=0.6; both about +Z; D=0.721, seams (ρ,z)=(2.549,0.892) and (3.997,−0.073)):
  - COMMON: volN≈30.24 vs analytic 30.307 vs OCCT, ΔV≈0.2% (a facet-inscription deficit).
  - FUSE:   volN≈83.1 vs analytic 83.272 vs OCCT, ΔV≈0.2% (a GROW).
  - CUT (A−B): volN≈28.85 vs analytic 28.910 vs OCCT, ΔV≈0.2% (a SHRINK); B−A likewise via swap.
  A SPINDLE torus (R ≤ r) declines at recognition; two tubes that CLEAR each other (D ≥ r1+r2), a
  CONTAINED tube (D ≤ |r1−r2|), CONCENTRIC-coaxial tubes (same centre, D≈0), a seam ring collapsing
  on the axis, and a non-coaxial / off-axis / skew torus pair all decline → NULL → OCCT (honest,
  never faked). The general TRANSVERSAL (non-coaxial) torus∩torus quartic space curve is a later slice.
- **S5-p — TRANSVERSAL (NON-COAXIAL) TORUS(ring)∩CYLINDER COMMON** (the SECOND transversal / non-coaxial
  slice after S5-k, and the FIRST transversal TORUS pair). Where S5-l handles the COAXIAL torus∩cylinder
  pose (cylinder axis colinear with the torus axis → two ANALYTIC circle seams, a Pappus solid of
  revolution), S5-p handles the OFFSET pose: the cylinder axis is PARALLEL to the torus axis but
  perpendicular-DISPLACED from it, sitting over the tube rim so a THIN cylinder pierces the tube like a
  vertical rod through the ring. Non-coaxial → the two seams are NON-PLANAR closed space curves (the
  quartic cyl∩torus locus, NO analytic circle), consumed DIRECTLY from the S3 `TraceSet` — the torus
  sibling of the S5-k Viviani slice. Pose: a thin Z-parallel cylinder offset so it pokes fully THROUGH
  the tube (entering the lower sheet, exiting the upper sheet) → the wall crosses the tube in exactly
  TWO disjoint closed loops (a lower + an upper along the cyl axis), both fully transversal
  (`nearTangentGaps==0`, `branchPoints==0`, both Closed). COMMON is the SAME TOPOLOGY as the coaxial S5-l
  COMMON — a cylinder mid-band capped by two TUBE-surface caps — but every ring is the TRACED non-planar
  seam: `buildTransTorusCylCommon` welds the torus LOWER cap (the tube sheet inside the cyl) + the
  cylinder BAND (seamLo→seamHi, inside the tube) + the torus UPPER cap through one `VertexPool`. The new
  prologue `transTorusCylSetup` recognises the axis-parallel offset pierce-through pose (offset > tol,
  parallel axis, exactly two closed loops + the inside-the-other survival samples) and resamples BOTH
  seams onto a common cyl-azimuth grid (`resampleByAzimuth`, shared with S5-k) so the band welds
  ring-to-ring; the band reuses `appendRevolvedBand` (a straight ruling is exact on the cylinder). The
  new cap builder `appendTransTorusCap` fans from the cap's tube-surface centre (the mean seam torus
  (u,v)) out to the exact traced seam nodes, each interior node placed ON the torus by lerping the torus
  (u,v) (the S5-a `appendMouthCap` discipline generalised to the torus) and oriented by the TRUE
  tube-outward normal (`tubeOutwardAt`, correct on both tube halves). REDUCTION: as the offset → 0 the
  pose becomes coaxial and S5-l's `torusCylSetup` claims it FIRST in the dispatch (S5-p gates on a
  STRICTLY-POSITIVE offset), reproducing the landed coaxial COMMON — a pinned regression. Verified vs a
  deterministic numerical-integration oracle (no closed form for a non-analytic seam) AND OCCT
  `BRepAlgoAPI_Common` (sim): host fixture (torus R=3 r=1 about +Z; Z-cylinder Rc=0.6, offset x=3,
  z∈[−2,2]), COMMON volN≈2.153 vs numeric 2.150, ΔV≈0.1%, watertight, symmetric (both operand orders),
  inside the 1% curved-parity bar — no tolerance weakened, DISAGREED=0. **CUT/FUSE now LAND** via the
  TORUS TUBE OUTER ZONE (`appendTorusTubeOuterZone`). MEASUREMENT overturned the earlier "minor-angle
  long-way sweep" framing: the two seams are LOCALIZED in the torus (u,v) — torus major u ∈ [−0.2, 0.2],
  NOT azimuth-wrapping like the sphere family's pole-to-pole latitudes — so the tube surface outside the
  bore is the FULL torus tube MINUS the two seam cap patches (a doubly-holed torus surface), NOT a
  between-two-seams band. It is tiled EXACTLY on-surface by a (u,v)-grid + loop-zipper hole-split: mesh
  the full tube grid, drop each cell inside a seam rectangle, zip the rectangle ring to the exact seam.
  `buildTransTorusCylCut` (torus−cyl) = tube outer zone (outward) + reversed cyl tunnel; the cyl−torus
  reverse now LANDS via `buildTransCylTorusCut` (BOOL-TAIL): the thin offset cylinder with a lens-shaped
  bite scooped out where the tube pierces through — two disc-capped cylinder end stubs welded to the
  reversed tube INNER cap patches (`appendTransTorusCap`, outwardSign=−1) between the two localized
  seams, ΔV <1% vs V_cyl − V_common, watertight. `buildTransTorusCylFuse` = tube outer zone + two cyl end
  stubs + end discs. Gated on `cylPierces` (both cyl end discs OUTSIDE the tube) + the seam localizable
  in a clean grid rectangle; else HONEST-DECLINE. Verified: torus−cyl volN≈56.6 vs numeric 57.1 ΔV≈0.75%,
  FUSE≈61.2 vs 61.6 ΔV≈0.71% (symmetric), watertight (boundaryEdges=0), on-surface. A skew axis, a
  single-sheet grazing cylinder (< two loops), and the coaxial pose (offset ≤ tol, S5-l) decline → OCCT.
- **S5-q — TRANSVERSAL (NON-COAXIAL) CONE(frustum)∩SPHERE COMMON** (the THIRD transversal / non-coaxial
  slice after S5-k and S5-p, and the FIRST transversal CONE pair). Where S5-f/S5-h handle the COAXIAL
  cone∩sphere pose (sphere centre ON the cone axis → ANALYTIC circle seams, planar rings), S5-q handles
  the OFFSET pose: the cone axis is perpendicular-DISPLACED from the sphere centre, so the two seams are
  NON-PLANAR closed space curves (the cone∩sphere quartic locus, NO analytic circle), consumed DIRECTLY
  from the S3 `TraceSet` — the cone sibling of the S5-k Viviani slice. Pose: a cone frustum whose wall
  pierces fully THROUGH the sphere → the wall crosses in exactly TWO disjoint closed loops (a lower + an
  upper along the cone axis), both fully transversal (`nearTangentGaps==0`, `branchPoints==0`, both
  Closed). COMMON is the SAME TOPOLOGY as the transversal cyl∩sphere S5-k COMMON — a cone mid-band capped
  by two spherical caps — but every ring is the TRACED non-planar seam: `buildTransConeSphereCommon`
  welds the sphere LOWER cap (inside the cone) + the cone BAND (seamLo→seamHi, inside the sphere) + the
  sphere UPPER cap through one `VertexPool`. The new prologue `transConeSphereSetup` recognises the offset
  pierce-both-ends pose (offset > tol, non-degenerate half-angle tanα, exactly two closed loops + the
  inside-the-other survival samples) and resamples BOTH seams onto a common cone-azimuth grid
  (`resampleByAzimuth`, shared with S5-k/S5-p); the band reuses `appendRevolvedBand` (the cone-axis radial
  reference orients each facet outward for the widening wall exactly as for a cylinder wall), the caps
  reuse `appendSphereCap` with the S5-k `transConeCapRings` refinement. REDUCTION: as the offset → 0 the
  pose becomes coaxial and S5-h's `coneSphere2Setup` claims it FIRST in the dispatch (S5-q gates on a
  STRICTLY-POSITIVE offset), reproducing the landed coaxial COMMON — a pinned regression. Verified vs a
  deterministic numerical-integration oracle (no closed form for a non-analytic seam) AND OCCT
  `BRepAlgoAPI_Common` (sim): host fixture (cone r(y)=0.5→1.5 over y∈[−3,3] about +Y; sphere Rs=2, centre
  offset 0.5 in +X), COMMON volN≈11.278 vs numeric 11.280, ΔV≈1.4e-4 (0.014%), watertight, symmetric
  (both operand orders), inside the 1% curved-parity bar — no tolerance weakened, DISAGREED=0. **CUT/FUSE
  now LAND (TRANS-BAND)** — the SAME seam-band primitive as S5-k about the CONE axis:
  `appendSphereOuterZoneBetweenSeams` tiles the sphere OUTER ZONE (the long way round outside the cone)
  on-surface via the cone-axis θ/φ sweep, and the cone side is exact via straight-ruling revolved bands +
  cone end discs. The end rings use `coneRingAxial` — placed at the TRUE axial station, since the cone
  surface's own (u,v) `point` uses the SLANT v (axially short, would tilt the disc). Verified vs the
  numeric oracle (V(A)/V(B) − numeric COMMON): CUT sph−cone 22.204 vs 22.232 (0.13%), CUT cone−sph 9.136
  vs 9.142 (0.07%), FUSE 42.619 vs 42.652 (0.08%) — all watertight, ≪ 1% bar; gated on `polesInsideCone`.
  A near-cylindrical cone (tanα≈0, S5-k territory), a single-loop / tangent pose (< two closed loops), a
  pole-grazing pose (a pole outside the cone → CUT/FUSE keep the honest decline), and the coaxial pose
  (offset ≤ tol, owned by S5-f/S5-h) all decline → NULL → OCCT (honest, never faked).
- **S5-r — coaxial TORUS(ring)∩HALF-SPACE (PLANE ⟂ AXIS) COMMON + CUT** (the FIRST curved∩PLANAR-half-
  space slice; the plane operand is not a `CurvedSolid`, so it is dispatched by the dedicated
  `tryTorusHalfspace` entry ABOVE the both-operands-curved gate). A ring torus (major R, minor r, axis
  = frame Z) cut by an axis-PERPENDICULAR planar half-space (a box/slab whose ONE cutting face sits at
  axial station z=h, |h|<r) → a horizontal tube slice whose section is an ANNULUS (washer between
  ρ=R±√(r²−h²)). Because the cut is symmetric in ρ−R the segment's first moment about ρ=R VANISHES and
  the plane-cut torus-segment volume collapses to the Pappus closed form **V(z≤h)=2π·R·A_low**, with
  A_low=πr²−(r²·acos(h/r)−h·√(r²−h²)). COMMON is a SOLID OF REVOLUTION welded from the S5-l tube-arc
  machinery (`appendTorusPlaneTubeArc`, mirror of `appendTubeArc`) on the kept side + a flat `appendAnnulusCap`
  at z=h through one `VertexPool` (watertight, closed). `recognisePlaneHalfspace` recovers the single
  bracketing cut plane from a plane-only operand (all faces planar, exactly ONE straddling the torus, all
  others clearing it); `torusPlaneSetup` gates the axis-perpendicular orientation + tube-crossing. CUT
  (torus minuend) keeps the complementary segment (same revolution, kept side flipped). Verified vs the
  Pappus closed form cross-checked against a 200³ numeric-integration oracle: host fixtures (R=3,r=1,h=0.4
  keep-low; R=4,r=1.5,h=−0.6 keep-high), COMMON + CUT watertight, ΔV within the 1% curved-parity bar,
  COMMON≤min(A,B), swap-symmetric, COMMON+CUT partition the torus — no tolerance weakened, DISAGREED=0.
  FUSE HONEST-DECLINES (the torus∪half-space needs the box's REMAINING faces, outside the revolution
  scope → OCCT); a half-space−torus minuend CUT declines. The PLANE-PARALLEL-to-axis (Villarceau-like)
  orientation and every degenerate pose (plane tangent to the tube |h|≥r, slab clear of the torus, a
  narrow post through the hole that does not bracket the tube) HONEST-DECLINE → NULL → OCCT (never faked).
- **S5-s — TRANSVERSAL (NON-COAXIAL) CONE(frustum)∩CYLINDER COMMON** (the FOURTH transversal / non-coaxial
  slice after S5-k/S5-p/S5-q, and the FIRST transversal cone∩cylinder pair). Where S5-e handles the COAXIAL
  cone∩cylinder pose (cone axis colinear with the cyl axis → a single ANALYTIC circle seam, a solid of
  revolution), S5-s handles the OFFSET pose: the cylinder axis is PARALLEL to the cone axis but perpendicular-
  DISPLACED from it, so a thin cylinder crosses the SLANTED cone wall in a NON-PLANAR closed space curve (the
  cone∩cylinder quartic locus, NO analytic circle), consumed DIRECTLY from the S3 `TraceSet`. **KEY STRUCTURAL
  FINDING (measured, not assumed):** unlike S5-k/S5-p/S5-q — whose through-going solid pierces a BOUNDED body
  at TWO loops — a parallel-axis cone∩cylinder meets in exactly ONE closed loop, because the cone wall radius
  r(y)=R₀+y·tanα is MONOTONIC so a parallel-offset cylinder crosses it once along its length (verified over
  every parallel-axis pose probed: the trace returns ONE loop; a fat cylinder straddling the axis returns at
  most an end-clipped open arc, never two closed poke-through loops). S5-s is therefore a distinct SINGLE-SEAM
  assembler, NOT the two-loop caps+band machinery. Pose: a THIN cylinder whose whole cross-section is OUTSIDE
  the cone at its narrow end and fully INSIDE at its wide end (a clean single wall crossing → one fully-
  transversal closed seam, `nearTangentGaps==0`, `branchPoints==0`, Closed). COMMON = the cone-wall CAP
  (bounded by the traced seam) + the cylinder wall BAND (seam → inside-end rim) + the cylinder INSIDE-END DISC,
  all welded through one `VertexPool` → watertight. The new prologue `transConeCylSetup` recognises the offset
  single-crossing pose (offset > tol, non-degenerate tanα, exactly ONE closed loop, one end disc fully OUT /
  the other fully IN, band-midpoint inside-the-cone survival sample), resamples the seam onto a common cyl-
  azimuth grid (`resampleByAzimuth`, shared with S5-k/S5-p/S5-q); the cone cap reuses the S5-p `appendTransTorusCap`
  discipline generalised to the ruled cone surface (`appendTransConeCap`, oriented by the TRUE cone-outward
  normal radial·cosα−axial·sinα), the band reuses `appendRevolvedBand`, the end disc reuses `appendDiskCap`.
  REDUCTION: as the offset → 0 the pose becomes coaxial and S5-e's `coneCylSetup` claims it FIRST in the
  dispatch (S5-s gates on a STRICTLY-POSITIVE offset). Verified vs a deterministic 200³ numerical-integration
  oracle (no closed form for a non-analytic seam) AND OCCT `BRepAlgoAPI_Common` (encoded structurally per the
  S5-q pattern; host numeric oracle is the primary check): host fixture (cone r(y)=0.5→1.5 over y∈[−3,3] about
  +Y; cylinder Rc=0.3, axis offset 1.0 in +X, y∈[−2,2]), COMMON volN≈0.5460 vs numeric 0.5466, ΔV≈5.4e-4
  (0.098%), watertight, symmetric (both operand orders), COMMON≤min(A,B), inside the 1% curved-parity bar —
  no tolerance weakened, DISAGREED=0. The reduction hand-off is pinned as a real (non-trivial) boundary: the
  same thin cylinder taken COAXIAL sits fully inside the cone (analytic overlap = the whole cylinder, ≈1.131),
  strictly LARGER than the offset wedge (≈0.546); a coaxial CROSSING cylinder (Rc=1.0) lands via S5-e. **CUT/FUSE
  now LAND** via the SINGLE-seam outer weld. Two shapes: **cyl−cone** is a CLEAN seam-driven cylinder stub (no
  hole) — cyl narrow-end disc + cyl wall band (narrow rim → seam) + the COMMON cone cap REVERSED — verified
  volN≈0.584 vs numeric 0.584 (ΔV≈0.01%). **cone−cyl** and **FUSE** need the FULL cone wall MINUS the seam cap —
  a HOLED cone surface, built by `appendConeWallOuterZone` with the SAME (u,v)-grid + loop-zipper hole-split as
  S5-p, in AXIAL cone coordinates (the codebase convention: CurvedSolid.vLo/vHi are axial stations; a wall ring
  at axial s has radius radius+s·tanα, so the discs at coneLo/coneHi weld flush) — plus cone end discs + reversed
  cyl bite; verified cone−cyl volN≈20.0 vs numeric 19.9 (ΔV≈0.6%), FUSE≈21.1 vs 21.0 (ΔV≈0.6%, symmetric),
  watertight. Gated on the seam localizable clear of both rims + u-wrap and its hole boundary being a single
  clean loop; else HONEST-DECLINE. A near-cylindrical cone (tanα≈0), a skew cylinder axis, a straddling/fat pose
  without a clean single crossing, and the coaxial pose (offset ≤ tol, S5-e) all decline → NULL → OCCT.

Honest scope still declining → OCCT (measured NULL fallbacks, never faked):
- **the TRANSVERSAL (offset) cylinder∩sphere CUT + FUSE** (the sphere-outer-zone weld between two
  non-planar seams — the S5-k residual) and the LARGER-offset transversal cyl∩sphere COMMON (offset
  ≳ 0.5·Rc, where the S2 co-resident seeding recall returns only ONE of the two loops); the SMALL-
  offset pierce-both-poles COMMON is now native — S5-k. The TRANSVERSAL (offset) torus∩cylinder
  CUT + FUSE now LAND (the tube-outer-zone hole-split — S5-p) and the offset cone∩cylinder CUT/FUSE now
  LAND (the single-seam outer weld / holed cone wall — S5-s). Likewise **the TRANSVERSAL (offset)
  cone∩sphere CUT + FUSE** (the sphere-outer-zone weld between two non-planar seams — the S5-q residual,
  same class) and the LARGER-offset / near-cylindrical / single-loop transversal cone∩sphere COMMON (the
  co-resident second-loop recall + the two-non-planar-seam band watertight gate); the pierce-both-ends
  small-offset COMMON is now native — S5-q. Plus **oblique / multi-tube cyl∩cyl**, and
  other curved-curved families (sphere∩box, freeform), the TRANSVERSAL (non-coaxial) cone∩cylinder /
  cone∩cone quartic space curve, apex-crossing / apex-in-extent frustums, parallel-wall (equal-half-angle)
  coaxial cone∩cone, the APEX-SPANNING / internally-tangent coaxial cone∩sphere crossing (the
  two-circle POKE-THROUGH pose is now native — S5-h; only the apex-spanning / tangent sub-configs
  still decline), and the sphere-minuend `sphere − cone` CUT → decline; plus any
  branched pair that is NOT equal-R orthogonal Steinmetz (unequal-R / non-orthogonal / ≠ 2 branch
  points / ≠ 4 arms). A disjoint Steinmetz pair (no seam) also declines for all three ops.
  (sphere∩sphere, Steinmetz, the coaxial cone∩cylinder, cone∩sphere single-crossing, the coaxial
  cone∩cone, the TWO-CIRCLE coaxial cone∩sphere, the TWO-CIRCLE coaxial cylinder∩sphere, the coaxial
  torus∩cylinder, the coaxial (centred) torus∩sphere, AND the coaxial torus∩cone FUSE/CUT/COMMON
  op-sets are now COMPLETE 3/3 NATIVE — see S5-c/S5-d/S5-e/S5-f/S5-g/S5-h/S5-i/S5-l/S5-m/S5-n above.)
Remaining S5 work: the transversal (offset) cyl∩sphere CUT/FUSE + larger-offset COMMON (the
S5-k sphere-outer-zone weld + the co-resident second-loop recall), the OFF-CENTRE coaxial (sc≠0)
torus∩sphere spiric section, general (non-Steinmetz) branched pairs, transversal/apex cone pairs,
the apex-spanning / tangent cone∩sphere sub-configs, transversal (non-coaxial) cone pairs, and more
curved-curved families.

## NURBS Layer 2 — general-freeform measurement pass (empirical decline map) · ✅ MEASURED 2026-07-10 · ⛔ POST-HOC RECALL CAMPAIGN DECLINED 2026-07-11 · ✅ SCALE-ADAPTIVE INITIAL SEEDING LANDED 2026-07-11 (decline 28.5%→18.8%, DISAGREED==0) · ✅ LOCUS-COVERAGE ORACLE AUDIT + FREEFORM-PAIR SEEDING EXTENSION LANDED 2026-07-11 (true decline 18.8%; audit → 0 over-counts, residual 100% genuine; extension → 18.8%→16.7%/17.4% combined, DISAGREED==0) · ✅ SEED-CLUSTER DISTINCT-BRANCH SPLIT LANDED 2026-07-11 (decline 16.7%→13.9%, multi-branch declines 19→14, DISAGREED==0)

Before scoping further S4 slices, the general NURBS↔NURBS boundary was measured empirically
with two differential fuzzers (verification only; `src/native` untouched, `cc_*` unchanged):

- **SIM native-vs-OCCT freeform fuzzer** (`tests/sim/native_ssi_freeform_fuzz.mm`,
  `scripts/run-sim-native-ssi-freeform-fuzz.sh`) — random valid NURBS↔NURBS surface pairs
  (bicubic-ish, rational + non-rational, positioned to intersect) driven through the real
  `seed_intersection`+`trace_intersection` pipeline vs OCCT `GeomAPI_IntSS`. Fixed tol onSurf
  `1e-6` / onCurve `1e-3` / occt `1e-7`, never widened.
- **HOST exact-oracle fuzzer** (`tests/native/test_native_ssi_exact_fuzz.cpp`) — S1 analytic
  breadth + NURBS↔analytic known-answer (rational NURBS exactly representing quadrics ∩ plane),
  closed-form oracle, no OCCT, machine precision.

**Result — the no-silent-wrong invariant HELD for general NURBS SSI: `DISAGREED == 0`** across
800 SIM trials (10 seeds × N=40 ×2) + the HOST sweep. Native never traced a curve off both
surfaces and never declared completeness with a fabricated locus; the HOST leg confirmed the
traces it *does* produce are exact (nodes on both surfaces + on the known analytic curve to
`1e-15…1e-11`). **No real native SSI bug was found** — every early "native wrong" flag traced
to a harness/OCCT-oracle bug where native was in fact correct (conic-threshold `θ vs π/2−α`;
`GeomAPI_ProjectPointOnSurf` missing the global foot on wavy freeform patches; fit-bow measured
instead of corrected nodes), fixed harness-side without widening any tolerance.

**Decline rate ≈ 25% (24/96 canonical), and the histogram REDIRECTS the S4 priority:**

| decline reason | share of declines |
|---|---|
| **multi-branch** (OCCT found a loop native's seeding missed) | **83%** |
| **small-loop** (partial branch coverage) | **17%** |
| near-tangent marching stall | **0%** |
| no-seed / corrector-failed | **0%** |

**Key finding — the frontier is SEEDING-RECALL, not the near-tangent marching moat.** The
marcher steps through freeform grazes cleanly (the `near-tangent` decline bucket stayed empty
even on tightly-tuned glancing paraboloids); the declines are almost entirely
OCCT-found-a-branch-native-didn't. So the **highest-recovery next slice is targeted
seeding-recall on uncovered parameter cells** — generalize the existing
`completeness_critic` / `criticTargetedReseed` path (S4-f, already recovering small loops on
canonical fixtures) to arbitrary freeform multi-loop poses — NOT further S4-c/d near-tangent
march-through work. This reprioritizes the S4-d…f tail below.

**Also surfaced (a distinct S2/S3 slice):** the freeform `SurfaceAdapter` does not declare
**periodic/seam parametrization**, so a rational-NURBS full circle traces as an open arc
(~99.6% coverage) with a small seam gap; declaring `uPeriod` currently trips the near-tangent
detector on rational parametric-speed variation. A periodic-seam-aware adapter + seam-crossing
closure is a clean, bounded slice independent of the multi-loop recall work.

### Seeding-recall campaign (3 rounds, 2026-07-11) · ⛔ NOTHING LANDED — honest decline, baseline holds

A bounded workflow attacked the dominant **multi-branch** decline by generalizing the
`completeness_critic` / `criticTargetedReseed` seeding-recall path to arbitrary freeform
multi-loop poses. **Measured decline delta: 0.0 pt — baseline stands at ≈29.5%** (roadmap
canonical; the committed SIM fuzzer-as-shipped measured 28.5% throughout). **`DISAGREED == 0`
held in every configuration of every round — the hard no-silent-wrong invariant was never
violated, and every slice that would have broken a contract was reverted, tree restored clean
at HEAD `31765c5`.**

**Slices attempted and HONESTLY DECLINED (no commit):**

- **R1 — post-hoc critic recall generalization** (`completeness_critic.h` `coverageOf`
  `dilate=false` exact-cell mode + a new M1d covered-cell second pass re-seeding
  already-covered coarse cells at finer per-cell resolution). Default-OFF, DISAGREED-safe,
  verified not to regress S4-f canonical parity (small-loop 0.5→1.0, many-loops 0.25→1.0 all
  PASS). **Blocker: INERT on the unmodified fuzzer** — identical decline WITH and WITHOUT the
  edits at cell caps ≤256, rounds ≤8, floor to 1/256. FUZZ_DIAG proved the residual
  multi-branch declines are NOT co-resident transversal loops reachable by finer targeted
  re-seed (`critRecoveredLoops=0`, `criticStoppedDry=1` even at floor): they are near-tangent
  grazes the seeder correctly refuses (the S4-c moat) or OCCT arc-split over-counts of one
  native loop. The only levers that moved the fuzzer were **caller-side** `SeedOptions` on the
  measurement instrument (critic on → 21.5%; finer initial grid `initialGridU/V=12`,
  `minPatchFrac=1/64` → 17.4%, both DISAGREED=0) — editing the committed instrument to
  manufacture the win is disallowed; flipping `completenessCritic` default-on breaks the S4-f
  host BEFORE/AFTER contract (`test_native_ssi_s4f_completeness.cpp` asserts
  `before.tracedBranches==1`) and changes every SSI/boolean consumer — out of a single safe
  round's scope.
- **R2 — within-cluster distinct-branch split at the seeder** (`seeding.cpp` refine gained
  `clusterSplitDistinct/Frac/Cap`: the adjacency clustering merges two distinct small loops
  whose candidate boxes touch into one cluster and keeps only the tightest seed, dropping the
  second; the split emits every spatially-distinct refined seed per cluster, each still on BOTH
  surfaces at the SAME `onSurfTol`, marcher locus-dedup remains the correctness gate) + a
  coverage-halo width knob (`criticCoverDilation`). Default-OFF. **Blocker: self-verify
  INCOMPLETE at the commit gate** — the shipping-config end-to-end fuzzer run (≥3 seeds with
  `clusterSplitDistinct` on) was still executing when output was forced, and the host SSI test
  suite had not been re-run; the `dilation=0` experiment returned IDENTICAL numbers
  (halo is not this fuzzer's miss mechanism). Committing before both gate verifications pass
  would violate the self-verify-then-commit discipline, so the change was declined and reverted.

**Remaining dominant decline reason (honest residual / next frontier):** the multi-branch
declines are **NOT a POST-HOC seeding-recall gap** reachable by finer post-hoc re-seeding — that
hypothesis was tested to exhaustion and falsified. The genuine residual splits into (1)
**near-tangent grazes the seeder correctly refuses** — real S4-c/d marching-core moat work, not
recall; and (2) **OCCT arc-split over-counts** where OCCT reports one native loop as multiple
components (an oracle-side counting artifact, not a native miss). The only demonstrated,
DISAGREED-safe lever that lowers the decline is **initial seeding RESOLUTION** (finer initial
subdivision catches co-resident loops the post-hoc critic cannot, because the loops were already
inside one covered cluster). The productizable slice is therefore **scale-adaptive initial
seeding resolution inside `seed_intersection`** (default path finds more loops with no caller
knob) — now LANDED below.

### Scale-adaptive initial seeding resolution · ✅ LANDED 2026-07-11 (decline 28.5%→18.8%, DISAGREED==0)

The productizable slice deferred above is now shipped as a **DEFAULT** in `seed_intersection`
(`src/native/ssi/{seeding.cpp,patch_bounds.h}` only; OCCT-free, no `cc_*` change, no caller knob,
`src/native/boolean` untouched). The initial subdivision resolution (the initial-grid pre-split +
the leaf `minPatchFrac`) now ADAPTS to each operand pair's geometry so SIMPLE / canonical poses
are BYTE-IDENTICAL and only dense freeform multi-loop poses get finer initial seeding:

- **Adaptivity gate — FREEFORM↔FREEFORM density.** Two new scale-free, magnitude-independent,
  OCCT-free signals are set on the `SurfaceAdapter` at freeform-adapter-factory time:
  `freeformSpanCount` = spansU × spansV (polynomial-patch tiling = intrinsic density) and
  `freeformComplexity` = the control-net **multi-modal-line count** (a noise-band hysteresis
  counts the significant slope reversals per net row/column/coordinate; a plain
  bump/dish/tilted/monotone net scores 0, a genuine egg-carton scores high; wobble/jitter is
  filtered by a per-line flatness + retreat band). Both are 0 for ELEMENTARY / plane / torus
  surfaces (no control net), so any pair with an elementary operand — every S1 analytic pair,
  plane∩sphere, plane∩B-spline (the S4-f completeness fixtures), sphere∩Bézier — is left BYTE-
  IDENTICAL. Adaptivity fires ONLY on FREEFORM↔FREEFORM pairs whose leaner operand has ≥ 2 spans
  or a wavy net; a flat single-patch freeform pair is unchanged.
- **Strength — bounded.** A qualifying pair gets initial grid ×2 / leaf ½ (the proven sweet
  spot), stepping to grid ×3 / leaf ¼ for a dense-and-wavy pair, with hard caps (grid ≤ 16, leaf
  ≥ 1/256). Deterministic; `maxDepth` + the leaf floor still bound termination.
- **Why finer INITIAL seeding (not the post-hoc critic)** recovers these loops: a coarse initial
  grid bridges two close co-resident loops into ONE topological cluster, so the post-hoc critic
  (which re-seeds only UNcovered cells) never sees them as missing (`critRecoveredLoops=0`,
  confirmed in the declined campaign above). A finer INITIAL grid keeps them in SEPARATE clusters
  from the start — the mechanism the post-hoc path structurally cannot reach.

**At the bar (all gates, 3 seeds N=48 each):**
- **Freeform fuzzer** (`scripts/run-sim-native-ssi-freeform-fuzz.sh`, the UNMODIFIED committed
  instrument — its caller-side `SeedOptions` were NOT touched; the win comes entirely from the
  `src/native` default): decline **28.5% → 18.8%** (41 → 27 / 144), squarely in the target
  17–21% band and near the uniform-finer ceiling (18.1% measured for a caller-side
  `initialGrid=12, minPatchFrac=1/64`), but achieved ADAPTIVELY — only freeform↔freeform pairs
  finer, everything else unchanged. Multi-branch declines 35 → 19 (the recovered population is
  the multi-branch + near-tangent-family egg-carton poses). **`DISAGREED == 0`** on every seed.
- **SSI host suite** re-baselined CLEAN with **no assertion changes**: all 9 `test_native_ssi_*`
  pass (seeding, marching, s4_classification, s4e_singularities, **s4f_completeness** — its
  `before.tracedBranches==1` / fixture-D BEFORE/AFTER seed-count contracts stay byte-identical
  because those fixtures are plane∩freeform → the gate does not fire — exact_fuzz, boolean,
  curved_boolean). Timings unchanged (marching ~105 s, curved_boolean ~316 s), confirming the
  adaptive finer seeding does NOT blow up cost on the canonical cases.
- **SSI SIM parity** (`run-sim-native-ssi-marching.sh`): every pair PASS, residuals bit-matching
  the S3 record (bspline∩bspline onCurve 1.86e-07, sphere∩sphere lenΔ 1.58e-05, S4-c/S4-d slices
  green). (`run-sim-native-ssi-seeding-parity.sh` has a PRE-EXISTING `gp_Dir::Magnitude` OCCT-SDK
  compile error in the committed harness — verified present on clean HEAD, unrelated to this
  change, which touches no `tests/sim` harness.)

Honest framing carried forward: the residual 18.8% raises the RECALL FLOOR on the
co-resident/small transversal loops; it is not a completeness proof. (**The prior belief that
this residual was partly OCCT arc-split "over-counts" of one native loop was EMPIRICALLY
FALSIFIED by the locus-coverage oracle audit below: it is 0 — the entire residual is genuine
native seeding-recall of distinct OCCT loci.**)

### Locus-coverage oracle audit — the TRUE decline is genuine, not an over-count artifact · ✅ DONE 2026-07-11 (true decline 18.8%, 0 over-counts, DISAGREED==0)

The freeform fuzzer's native-vs-OCCT comparison was hardened into an EXPLICIT bidirectional
LOCUS-COVERAGE oracle (`tests/sim/native_ssi_freeform_fuzz.mm`; SIM harness only, `src/native`
untouched, `cc_*` unchanged) to settle whether the residual multi-branch declines were genuine
misses or OCCT `GeomAPI_IntSS` arc-split over-counts (OCCT splits one intersection locus into
several line components; a component/branch-COUNT comparison would mis-score native's single
correct loop as a decline whenever OCCT split it into more pieces than native traced).

- **The oracle compares LOCI, not counts.** Per OCCT line: (a) covered by native, (b) uncovered
  by native but covered by a SIBLING OCCT line native already covered — an arc-split OVER-COUNT
  credited to native (native traced the same geometric locus once; OCCT just chopped it), or (c)
  uncovered and on no other OCCT line — a GENUINE distinct locus native missed. The reverse
  residual now accumulates over genuine-miss lines only, so an over-count-only case falls through
  to AGREED regardless of how many arc-components OCCT emitted. Fixed onSurf/onCurve tolerances
  are REUSED, never widened; the DISAGREED gates (native node off both surfaces; native curve off
  the OCCT locus) are unchanged, so the no-silent-wrong invariant is preserved and STRENGTHENED
  (an over-count sibling can no longer be counted as native coverage of a locus it does not cover).
- **Measured (3 seeds, N=48, base 0x5515D1FF0F0F):** AGREED 117 / DECLINED 27 (**18.8%**) /
  DISAGREED 0 / ORACLE-INACCURATE 0. **LOCUS-COVERAGE AUDIT: 0 AGREED over-count trials, 0
  over-count decline lines, 39 genuine distinct-locus misses** (worst missed-locus 3D length
  ~2-6). **The TRUE decline equals the pre-audit 18.8% — the oracle was already coverage-correct;
  NONE of the "multi-branch declines" were OCCT over-counts.** The entire 18.8% residual is
  genuine native seeding-recall of distinct OCCT loci. `DISAGREED == 0` held.

### Freeform-pair scale-adaptive seeding extension — recover smooth co-resident loops · ✅ LANDED 2026-07-11 (decline 18.8%→16.7% primary / 19.1%→17.4% combined, DISAGREED==0)

With the audit proving the residual is genuine (not an oracle artifact), the seeding gate was
extended in `src/native/ssi/seeding.cpp` (OCCT-free, no `cc_*` change, no caller knob,
`src/native/boolean` untouched). The scale-adaptive gate previously fired ONLY on wavy/dense
freeform pairs (`minSpan ≥ 2 || osc ≥ 4`); the dominant remaining genuine misses were SMOOTH
freeform↔freeform pairs (low-span, non-wavy — two gently-bowed sheets / two paraboloids that
interpenetrate over a wide region and cross in MORE THAN ONE loop) that the coarse grid merged
into one cluster → one representative seed → the second loop missed. The gate now fires on ANY
freeform↔freeform pair; the STRENGTH still scales with density/waviness (base grid ×2 / leaf ½;
dense/wavy tier grid ×3 / leaf ¼), bounded by the same hard caps (grid ≤ 16, leaf ≥ 1/256):

- **Canonical safety — BYTE-IDENTICAL.** The gate keys off `freeformSpanCount ≥ 1` on BOTH
  operands. An ELEMENTARY / plane / torus operand has span count 0, so any pair with one — every
  S1 analytic pair, plane∩sphere, plane∩B-spline (the S4-f BEFORE/AFTER seed-count fixtures,
  which assert `before.tracedBranches == 1` / `curveCount() == 1`), sphere∩Bézier — is unchanged.
  The pure-freeform host fixtures assert INEQUALITIES (`branchCount() >= …`), so finer seeding
  keeps them green. **All 9 `test_native_ssi_*` host suites pass (seeding 9, marching 21, s4f 6,
  s4_classification 22, s4e 7, exact_fuzz 144-trial DISAGREED=0, boolean 4, curved_boolean 13);
  no assertion changes.**
- **Cost — flat.** Host marching 28.1 s → 29.0 s (+3%), seeding 3.40 s → 3.54 s (+3%): the
  elementary/mixed fixtures (gate off) are byte-identical work; the small pure-freeform fixtures'
  finer seeding is cheap. SSI marching SIM parity (`run-sim-native-ssi-marching.sh`) green.
- **Recall — a real, consistent gain (bounded).** Committed instrument (unmodified `SeedOptions`;
  the win is entirely the `src/native` default): decline **18.8% → 16.7%** on base 0x5515D1FF0F0F
  (27 → 24 / 144) and **19.4% → 18.1%** on base 0x1234ABCD (28 → 26 / 144) — **combined 6 seeds
  / 288 trials: 19.1% → 17.4% (55 → 50)**. Strictly in the right direction on BOTH bases; every
  Part-B decline index is a SUBSET of the baseline's (no previously-agreed case regressed). The
  recovered population is the smooth near-tangent-family / tilted-sheets co-resident second loops.
  **`DISAGREED == 0` on all 288 trials.** The caller-side finer-seeding CEILING (grid 12, leaf
  1/96) measured 9.7% — so meaningful genuine headroom remains, but is NOT bounded/safe to take
  everywhere (its cost is much higher); the landed default takes the bounded, canonical-safe slice.

**Remaining residual (honest / next frontier):** ~16-18% — the hard multi-branch moat. Many
remaining misses are second loci in pairs that ALREADY qualify for the gate yet still merge two
distinct loops into one seed at the ×2/×3 grid (the marching-core / seed-cluster-split frontier),
plus deeper co-resident structure the finer-everywhere ceiling (9.7%) reaches only at costs the
bounded default deliberately does not pay. The over-count-artifact hypothesis is closed (audited
to 0); the residual is genuine recall, attackable next only by seed-cluster distinct-branch
splitting or a targeted-cost critic — NOT by widening the initial grid further.

### Seed-cluster distinct-branch split — recover merged co-resident loops · ✅ LANDED 2026-07-11 (decline 16.7%→13.9%, multi-branch declines 19→14, DISAGREED==0)

The named next frontier ("seed-cluster distinct-branch splitting"). The param-box adjacency
clustering (`clusterRegions`) unites candidate regions whose param boxes touch on BOTH surfaces
into one cluster. Two DISTINCT co-resident transversal loops that a dense freeform pair hosts
close together can have touching candidate boxes and be MERGED into one cluster — the refine then
kept only the single tightest seed and DROPPED the second loop. This was the dominant residual
after the scale-adaptive seeding: audited GENUINE (0 over-counts), 79% of declines multi-branch,
each `traced=1 occtLines=2 genuineMiss=1` with the missed locus ~1.3 model-units off the traced
one. A prior workflow round (R2) attempted this exact idea but declined at an INCOMPLETE
self-verify gate (the shipping-config fuzzer run + host suite were not both re-run before the
commit was forced); this is the properly-verified re-attempt.

The refine pass (`refineClusters` in `src/native/ssi/seeding.cpp`, OCCT-free, no `cc_*` change,
`src/native/boolean` + `src/native/blend` untouched) now emits one seed per SPATIALLY-DISTINCT 3D
locus a cluster hosts, instead of only the tightest:

- **The distinct-branch split predicate — SINGLE-LINKAGE on the refined 3D points.** A cluster's
  accepted transversal seeds are grouped into connected components by single-linkage: two seeds
  join iff their 3D `point`s are within `sep = splitDistinctFrac · modelScale` (default `1/16`).
  A SINGLE physical loop's refined points tile it densely (consecutive candidate leaves are
  ~leaf-size apart, ≪ `sep`), so they chain into ONE component → the cluster still collapses to
  one seed (single-loop / canonical cases unchanged). Two loops separated in 3D by more than `sep`
  form TWO components → a seed per loop. The tightest seed of each component is emitted, capped at
  `splitMaxPerCluster` (default 8). Every emitted seed already passed the FULL refine gate — on
  BOTH surfaces at the SAME `onSurfTol`, transversal (‖n₁×n₂‖ ≥ `tangentSinTol`) — never a widened
  tolerance, never a fabricated seed. Scale-relative, deterministic, bounded.
- **Why it CANNOT over-produce a wrong result.** An over-split seed (two seeds that are actually
  the same loop, > `sep` apart) is HARMLESS: it re-traces the same loop and the S3 marcher's
  per-branch locus-dedup (`retraces` / `sameLocus` in `marching.cpp`) collapses the near-identical
  polyline. So the predicate is RECALL-ONLY — it can add a genuine second-loop seed or a harmless
  duplicate, never a curve off both surfaces. The marcher's locus-dedup remains the correctness gate.
- **FREEFORM↔FREEFORM GATE → canonical BYTE-IDENTICAL.** The split fires only when BOTH operands
  are freeform (`freeformSpanCount ≥ 1`) — the same gate the scale-adaptive seeding uses. Any pair
  with an ELEMENTARY / plane / torus operand (span count 0) keeps the running-tightest single seed
  per cluster (the pre-split path), so every S4-f BEFORE/AFTER seed-count fixture
  (`before.tracedBranches == 1`, `curveCount() == 1`, Steinmetz `branchPoints == 2`) and every
  exact-count seeding fixture (`branchCount() == 1` / `== 2`) is UNCHANGED. **All 9
  `test_native_ssi_*` host suites pass; no assertion changes; s4f runtime unchanged (8.2 s) —
  confirming zero added work on the byte-identical path.** SSI marching SIM parity
  (`run-sim-native-ssi-marching.sh`) green (19 passed, 0 failed).
- **Recall — a real, consistent gain (bounded).** Committed instrument (unmodified `SeedOptions`;
  the win is entirely the `src/native` default): decline **16.7% → 13.9%** on base 0x5515D1FF0F0F
  (24 → 20 / 144), with multi-branch declines **19 → 14** and genuine-miss lines **30 → 24**.
  Every one of the 3 seeds improved (declines 6→4, 7→6, 11→10); no previously-agreed case
  regressed. `DISAGREED == 0` on all 144 trials of the primary base and on an independent base
  (0xA11CE5EED, 3 seeds, 21.5% decline — a harder pose distribution, still 100% genuine, 0
  disagreed). **6 seeds total, DISAGREED == 0 throughout.**
- **Cost — flat.** Bounded single-linkage over per-cluster seeds (capped at 256 retained /
  8 emitted). Host `curved_boolean` 76 s → 80 s (+5%); the elementary/mixed path is byte-identical
  work. No blow-up on single-loop / canonical cases.

**Remaining residual (honest / next frontier):** ~14% — still the hard multi-branch moat, now
thinner. The remaining multi-branch declines are loops merged into a cluster whose refined points
CHAIN within `sep` (genuinely 3D-adjacent co-resident loops the single-linkage cannot separate
without risking a single-loop split), the near-tangent grazes the seeder correctly refuses (the
S4-c/d marching-core moat, not recall), and deeper co-resident structure the finer-everywhere
ceiling (9.7%) reaches only at costs the bounded default deliberately does not pay. The next
attackable lever is the S4-c/d near-tangent marching-core, not further seeding recall.

### SSI multi-branch floor (Wave-I / I1) — fit-density densify-refit + sharpened residual map · ✅ LANDED 2026-07-12 (fit-bow residuals lowered, DISAGREED==0; decline holds 9.7% — honest, no gate loosened)

The 9.7% freeform-fuzzer floor left after E1's co-resident-branch fix (`0x5515D1FF0F0F`+2, N=48×3
= 144 trials) was RE-MEASURED and its residual **anatomised per decline** to separate the honestly
recoverable part from the genuine person-decade moat. Baseline **AGREED 130/144 (90.3%),
HONESTLY-DECLINED 14/144 (9.7%), DISAGREED 0**; reason histogram **near-tangent 0, multi-branch 9,
small-loop 5** (the near-tangent MARCHING family stays EMPTY — confirmed again). A host replay of the
EXACT fuzz poses (OCCT-free, reproducing the fuzzer's splitmix64→xoshiro256** RNG + generator) joined
each decline's sim anatomy (`genuineOcctOnNat`, `worstMissLen`, `natOnOcct`) to its native trace
anatomy (per-loop node count, fitted `maxFitError`, between-node fit bow). The 14 declines split into
**three distinct sub-populations**, only one of which is a native geometry/recall gap:

- **FIT-DENSITY artifact on a DENSE high-curvature loop (recoverable fit quality) — the dominant
  small-loop sub-population.** On several tight glancing loops the marcher traces **1000+ on-locus
  nodes** (every node on BOTH surfaces ≤ `onSurfTol`, on the OCCT locus ≈ `natOnOcct` 2e-7…2e-6), yet
  the least-squares B-spline fit at the default 64-pole cap could NOT follow the curvature and BOWED
  off the on-locus polyline by ~2–5e-3. Because downstream coverage samples the FITTED curve (not the
  polyline), that bow read over the 1e-3 curve-coverage budget → a decline. This is a pole-COUNT
  artifact, not a corrector error, and is the S3 follow-up the roadmap named as "automatic
  densify-and-refit on a too-loose B-spline fit is not yet wired".
- **GENUINE distinct co-resident locus native's seeder missed (the real recall moat) — 3 declines**
  (`genuineOcctOnNat` ≈ 1.5…2.0, `worstMissLen` 2.8…4.8): native traced ≥ 1 locus and OCCT found a
  WHOLE separate co-resident locus. These are the residual after E1 — loops whose refined points CHAIN
  within `sep` (genuinely 3D-adjacent), which the single-linkage split cannot separate without risking
  a single-loop split. Attacking them further risks over-splitting one loop → declined (the documented
  hazard); this is the honest seeding-recall frontier, not a fit issue.
- **Sub-`onCurve` residual on a fully-traced loop (native-vs-OCCT curve divergence + node-spacing
  sagitta) — the rest.** `genuineOcctOnNat` sits at 1.0…1.6e-3, JUST over the fixed 1e-3 gate, with the
  fit already tight; the residual is the marcher's chord-deflection (`maxDeflection = scale·1e-3`,
  looser than the 1e-3 coverage gate) between on-locus nodes plus a genuine ~1e-3 native-vs-OCCT
  divergence on a glancing loop. Recovering these needs curvature-adaptive marching (the moat), not fit
  poles or seeds.

**Landed (fit-quality, `src/native/ssi/marching.cpp` only — OCCT-free, no `cc_*` change,
`src/native/boolean` + `src/native/blend` untouched, no caller knob): DENSIFY-AND-REFIT the fitted
convenience curve.** `fitBSpline` now, when its worst deviation from the on-locus NODES
(`maxFitError`, already computed) exceeds a scale-relative target (`scale·2e-4`, below the 1e-3 gate
yet above a smooth loop's error) with room to add poles, refits ONCE at a higher bounded pole count
(200) so the curve rides the on-locus polyline. Strictly cost-bounded: ONE extra `O(m·poles²)` solve,
hard-capped poles, and a **node-count guard** (skip loops with `m > 2000` — a chart-singularity
pole-crossing loop can circulate to ~20 000 nodes where a 200-pole solve is both unaffordable AND
fruitless). It is a **NO-OP on a smooth loop** whose 64-pole fit already rides the nodes (byte-
identical single fit), and it is **DISAGREED-safe by construction**: the polyline (ground truth) is
unchanged and NO on-curve / on-surface tolerance is touched — more poles only pull the convenience
curve CLOSER to the already-on-locus nodes, never move a node or fabricate geometry.

- **Measured effect — residual SHARPENED, DISAGREED-safe, decline HOLDS.** The densify lowers the
  fit-density residual on the dense loops it targets (final sim: `4.778e-3 → 1.531e-3` on
  `0x5615d1ff10c2` case 32, a 1068-node loop; `1.589e-3 → 1.060e-3` on `0x5715d1ff1275` case 22, a
  1122-node loop; `3.510e-3 → 3.343e-3` on `0x5515d1ff0f0f` case 24), proving the ~3e-3 fit-bow
  component is genuinely removed — but **none crosses the fixed 1e-3 gate** (the remaining 1.0…1.6e-3 is
  the native-vs-OCCT + node-spacing residual above, not fit), so the **decline rate honestly HOLDS at
  9.7% (14/144), DISAGREED == 0** (AGREED 130/144 = 90.3%, histogram near-tangent 0 / multi-branch 9 /
  small-loop 5, unchanged from baseline). The decline count
  was NOT lowered by loosening any gate (refused) nor by fabricating a curve/seed (refused). The floor
  is now precisely mapped: **fit-density is no longer a contributor** to the residual — it is genuine
  co-resident-locus seeding recall (3) plus a sub-`onCurve` glancing-loop divergence (the rest), both
  of which are the person-decade moat (curvature-adaptive marching / co-resident recall), not a bounded
  slice.
- **Verification.** All 9 `test_native_ssi_*` host suites pass with **no assertion changes**
  (native_ssi 11, seeding 10, marching 22 incl. the new regression, s4_classification 22,
  s4e_singularities 7, s4f_completeness 6, exact_fuzz DISAGREED=0, boolean 4, curved_boolean 13). Cost
  bounded: s4e 36 s → 59 s (the moderate-node pole-crossing loops densify; the 20k-node loop is guarded
  out), s4f 31 s, curved_boolean unaffected (S5 boolean volumes within tol — the densified curves it
  consumes are strictly tighter). Regression fixture `march_densify_refit_high_curvature_loop` (the
  verbatim fuzz pose `0x5615d1ff10c2` case 32 — a rational-NURBS ∩ B-spline tight loop of 1068 on-locus
  nodes): asserts the densify FIRES (fit poles > 64), every node on both surfaces ≤ 1e-6, and the
  densified curve rides the on-locus polyline with between-node bow < 1e-3 (the ~5e-3 bow at 64 poles
  gone) — no tolerance widened.

**Honest conclusion (the sharpened I1 floor map):** the remaining 9.7% freeform-fuzzer decline is
**not fit-density** (now removed as a contributor) and **not near-tangent marching** (0, empty). It is
(a) **3 genuinely-missed co-resident distinct loci** whose refined seeds chain within `sep` — the
single-linkage seeding-recall frontier, unrecoverable without risking a single-loop over-split; and
(b) a **sub-`onCurve` (1.0…1.6e-3) native-vs-OCCT divergence + node-spacing sagitta** on fully-traced
glancing loops, recoverable only by curvature-adaptive marching (`maxDeflection` is `scale·1e-3`,
looser than the 1e-3 coverage gate). Both are the genuine SSI moat — a bounded, DISAGREED-safe slice
cannot lower the decline further without loosening the gate or fabricating geometry, which this track
REFUSES. The densify is a real product improvement (every downstream S5/S6/S7 consumer of a dense
`TraceSet` curve now gets a tighter fit) and the honest floor is now precisely characterised.

### SSI-RECALL — co-resident split lever is EXHAUSTED (mechanism re-measured) · ✅ HONEST-DECLINE 2026-07-13 (decline HOLDS 9.7%, DISAGREED==0; the residual is UPSTREAM of the split, not a split-linkage merge)

Wave-I / I1 conjectured the 3 genuinely-missed co-resident loci were a **split-linkage** problem —
"refined seeds chain within `sep`, unrecoverable without risking a single-loop over-split." This
track **RE-MEASURED that conjecture with a per-cluster `SEED-DIAG` instrument** (env-gated, OCCT-free,
no `cc_*` change) on the exact `0x5515D1FF0F0F`+2 / N=48×3 base, and it is **WRONG**: the residual is
NOT a split-linkage merge the distinct-locus split can separate. It is UPSTREAM of the split. A
curvature/tangent-aware sub-cluster separation was implemented, verified DISAGREED-safe, and measured
to be a **complete no-op** — the honest evidence for retiring this lever.

- **The attempted approach (genuinely-new, DISAGREED-safe): TANGENT-GATED single-linkage.** The
  distance-only single-linkage in `splitDistinctLoci` was augmented with a curvature-aware GATE — two
  seeds unite iff (a) within `sep` in 3D (unchanged) AND (b) the chord between them aligns
  (`|chord·t| ≥ cosGate`, cos 60°) with BOTH seeds' intersection-loop tangents `t = normalize(nA×nB)`
  (the SAME tangent the S3 marcher steps along). It is a NECESSARY condition to KEEP a link (AND-ed with
  distance), so it can only split MORE, never merge → recall-only, DISAGREED-safe by construction (an
  over-split re-traces the loop and the marcher's `sameLocus`/`retraces` dedup collapses it). All 5 SSI
  host suites pass with NO assertion change (seeding 10, marching 22, s4f 6, exact_fuzz DISAGREED=0,
  s4_classification 22).
- **Measured effect — ZERO.** Freeform-fuzzer decline **HOLDS at 9.7% (14/144), DISAGREED == 0**,
  histogram **byte-identical** to baseline (near-tangent 0 / multi-branch 9 / small-loop 5), all 14
  decline anatomies (`traced`/`occtLines`/`genuineMiss`/`genuineOcctOnNat`/`worstMissLen`) unchanged. On
  a CONSTRUCTED transverse-crossing co-resident pair (flat sheet ∩ saddle → an X of two lines with ⟂
  tangents) the gate ALSO gave branches=1 in both modes. Because it is provably dead code on every pose
  measured or constructed — pure hot-path cost (a tangent per retained seed, up to the 65 536 cap) with
  no recall — it was **REVERTED**, not shipped (prefer the simpler design; do not gold-plate).
- **THE ACTUAL MECHANISM (`SEED-DIAG`, the sharpened map).** The 3 genuine co-resident misses are
  upstream of the split in three distinct ways, none a split-linkage merge:
  - **idx=33 (seed 0f0f), traced=1 / occtLines=2:** `candidates=272 847, clusters=1, seeds=1`, and the
    ONE cluster **hit the `kMaxSeedsPerCluster = 65 536` cap** (`xversalSeeds=65536, emitted=1`). Both
    co-resident loci collapse into ONE param-adjacency cluster, and its refined-seed field is TRUNCATED
    at the cap in candidate-iteration order → the co-resident locus's later leaves are **cap-starved**
    before the split ever sees them. This is the SAME starvation class E1 raised from 256 to 65 536,
    but these ultra-dense wavy poses genuinely exceed 65 536 (the E1 note's "a real intersection field
    never reaches it" fails here).
  - **idx=5 (seed 10c2), traced=2 / occtLines=3:** `candidates=215 834, clusters=1, seeds=2`, cluster
    also **capped** (`xversalSeeds=65536, emitted=2`) — the split DID fire (2 seeds), but a 3rd locus is
    still cap-starved.
  - **idx=43 (seed 0f0f), traced=2 / occtLines=3:** `candidates=30 421, clusters=2, seeds=2`, NO cap
    hit (cid=1 held only 53 seeds). A 3rd distinct locus **never produced a clustered candidate** — a
    subdivision/refine PLACEMENT miss, not a cap or a split problem.
  - Across the run: **92/144 trials collapse the whole intersection into `clusters=1`**, **113 clusters
    carry ≥ 10 000 refined seeds, 8 hit the 65 536 cap**, yet only **13 clusters ever `emitted=2` and 1
    `emitted=3`** — the split machinery works where loci ARE separable; it simply is not the residual's
    bottleneck.

**Honest conclusion (the corrected SSI-RECALL floor map).** The remaining co-resident misses are NOT a
split-linkage frontier — no linkage predicate (distance OR curvature/tangent) can recover them, because
the second locus never reaches the split as a distinct component. It is (a) **`kMaxSeedsPerCluster`
cap-starvation** on ultra-dense poses whose single param-adjacency cluster hosts > 65 536 refined
transversal crossings, and (b) a **subdivision/refine placement miss** where a co-resident locus never
produces a clustered candidate. The next attackable levers are therefore UPSTREAM: a **density-preserving
retention** when the per-cluster cap is hit (uniform/reservoir thinning so BOTH loci stay represented,
rather than FIFO truncation — but this interacts with the documented "thinned set leaves arc gaps that
spuriously split one loop" hazard, so it is not a free bounded slice), and **finer/adaptive initial
subdivision** for the placement-miss loci — both genuine person-decade-moat work, not a bounded
DISAGREED-safe slice. The split-linkage lever (distance and now tangent-gated) is EXHAUSTED. No gate was
loosened, no locus fabricated, no single loop over-split; DISAGREED stayed 0 throughout.

### SSI-SUBDIV — feature-adaptive initial subdivision recovers the placement-miss locus · ✅ LANDED (decline 9.0%→4.9%, idx=43 recovered, multi-branch 8→2; DISAGREED==0; small-loop 5→5 no over-split; SSI-CAP partition byte-unchanged)

The SSI-RECALL map above localized one of the residual co-resident misses (`idx=43`, seed `0f0f`,
`candidates=30 421, clusters=2, seeds=2`, NO cap hit) as a **subdivision/refine PLACEMENT miss** —
a 3rd distinct locus that **never produced a clustered candidate** because the uniform subdivision
leaf was too coarse near it. This track closed that specific locus at the S2 layer (`seeding.cpp`
only; `src/native` stays OCCT-free; `cc_*` byte-unchanged; SSI-CAP's cap-retention / partition /
distinct-locus-split code BYTE-UNCHANGED — the lever is strictly UPSTREAM of the split).

- **Mechanism (host replay of the exact fuzz pose + `SEED-DIAG`).** The pair (B-spline 6×5 ∩
  rational NURBS 6×6, `F_MULTIBRANCH`) hosts THREE co-resident transversal loci; the dense-tier
  scale-adaptive uniform leaf is 1/128. At that leaf the intersection collapses into `clusters=2`
  (`cid=0` a dense ~30 k-seed pile emitting 1, `cid=1` emitting 1) → 2 seeds. The 3rd locus shares
  the coarse leaf with the dominant locus and never becomes its own param-adjacency cluster. Probes
  isolated the cause: **tightening the clustering `epsFrac` alone does NOT separate them** (still
  `clusters=2`, `candidates=30 421`); only a **finer LEAF** does — refining the overlapping leaves
  one level finer (leaf 1/128 → 1/256, `candidates 30 421 → 60 111`) yields `clusters=3, seeds=3`.
- **The fix (additive, DISAGREED-safe): feature-adaptive initial subdivision.** New
  `SeedOptions.adaptiveSubdivision` (default ON, freeform↔freeform only — same gate as scale-adaptive
  seeding, so every elementary/mixed/S1/S4-f contract is byte-identical). At a would-be uniform leaf
  whose two patch AABBs OVERLAP (the caller already ruled out disjoint), `featureWarrantsFinerLeaf`
  refines it ONE level below the uniform leaf toward `adaptiveMinFrac` (1/256); `adaptiveOverlapFrac`
  (default 1.0) is an optional shallow guard. Purely ADDITIVE: it only adds candidate regions in
  overlapping (feature) cells; `refineRegion` still gates every seed on both surfaces ≤ `onSurfTol`
  AND transversal (‖n₁×n₂‖ ≥ `tangentSinTol`), so it can only surface a REAL co-resident locus the
  coarse leaf hid — it NEVER fabricates a locus, widens a tolerance, or over-splits (an extra
  candidate on an already-found locus refines to the same seed; the distinct-locus split / marcher
  dedup collapses a duplicate). Cost is bounded (~2× candidates on the dense pair only; the
  `adaptiveMinFrac` floor + `maxDepth` bound recursion).
- **Measured (freeform fuzzer `0x5515D1FF0F0F`+2, N=48×3).** idx=43 (seed 0f0f) RECOVERED: native
  now seeds all 3 loci (the recovered seed at x≈−1.095 is on both surfaces ≤ 3e-14 and transversal,
  sine 0.55). Before → after decline: **AGREED 131/144 (91.0%), DECLINED 13/144 (9.0%)** → **AGREED 137/144 (95.1%), DECLINED 7/144 (4.9%)**, DISAGREED 0 → 0 (bar holds),
  small-loop bucket UNCHANGED (5 → 5 — no over-split); multi-branch declines 8 → 2 (six placement-miss loci recovered, idx=43 among them). SSI-CAP's recovered cases stay recovered (its code is
  byte-unchanged). Regression fixture: `seed_freeform_adaptive_subdivision_recovers_third_locus` (the
  verbatim idx=43 pose — adaptive OFF ⇒ 2 seeds, ON ⇒ 3, each genuine on both surfaces, distinct
  loci spread across x). Host `test_native_ssi_seeding` 11/11, `test_native_ssi_exact_fuzz`
  DISAGREED==0.

### SSI-SMALLLOOP — the S4 recall tail sharpened; idx=33 co-resident locus RECOVERED but decline HOLDS (curve-divergence moat) · ⛔ HONEST-DECLINE 2026-07-14 (recall gain measured & DISAGREED-safe; NOT shipped — no decline drop; SSI-CAP split `sep` BYTE-UNCHANGED)

The remaining seed `0x5515D1FF0F0F` declines after SSI-SUBDIV (idx=23/24/33) were re-measured with an
OCCT-FREE HOST REPLAY of the exact fuzz poses (`tests/native/replay_freeform_seed.cpp` — reproduces
the splitmix64→xoshiro256** RNG + generator verbatim, advances the stream to a target idx, runs the
real `seed_intersection`/`trace_intersection` with an env-gated `SEED-DIAG` component-profile probe)
cross-checked against the SIM/OCCT fuzzer's `[DECLINE-DIAG]`. This CORRECTS the prior "native 0" framing
and localizes each decline to a distinct stage:

- **idx=24 (near-tangent, small-loop) — NOT a recall miss; a fully-traced glancing loop.** Host replay:
  seeder emits 1 seed from a 313 134-crossing cap-hit cluster that is ONE 3D component at EVERY
  separation scale (`rawComps@sep=1 … @~eps=1`); the marcher traces it to a **Closed loop, 714 nodes,
  onSurf 2e-10**. SIM `[DECLINE-DIAG]`: `traced=1 occtLines=1 genuineOcctOnNat=1.818e-3` — native DID
  seed AND trace the single locus; the decline is the fitted/marched curve sitting `1.8e-3` off the OCCT
  locus (just over the fixed `onCurve` 1e-3), a native-vs-OCCT curvature-adaptive-marching residual (the
  moat), NOT a seeding gap. No seeding lever can touch it.
- **idx=23 (near-tangent, multi-branch) — same moat.** `traced=4 occtLines=4 covByNat=3
  genuineOcctOnNat=1.334e-3 worstMissLen=0.94`: native traced 4 of 4 loci; one graze diverges by
  `1.33e-3` sub-`onCurve`. Curvature-adaptive marching, not recall.
- **idx=33 (near-tangent) — a GENUINE co-resident second-locus miss, RECOVERABLE.** Host replay:
  805 232 refined crossings collapse into ONE param-adjacency cluster (cap-hit), and the pile is ONE 3D
  component at the committed split `sep` (= `splitDistinctFrac·scale = scale/16`) and at `sep/2`, but
  splits into **TWO** components at `sep/4`…`sep/16`. The two loci are ~model-scale apart yet their
  refined point clouds CHAIN together within `sep` along their arcs, so the split emits ONE seed and
  DROPS the second locus. Baseline SIM: `traced=1 occtLines=2 covByNat=0 genuineMiss=2
  genuineOcctOnNat=1.371` — a whole-locus recall miss. **Over-split-safety margin measured:** a genuine
  SINGLE loop (idx=24) stays ONE component down to `sep/64`, so a finer split `sep` on a cap-hit cluster
  separates idx=33's two loci without fragmenting a real loop.

**Attempted (genuinely-new, bounded, additive, DISAGREED-safe): CAP-CARRIER FINER DISTINCT-LOCUS SPLIT.**
A finer split `sep` (`sep/4`) applied — for BOTH the loop-aware retention partition AND the distinct-locus
split — ONLY to a cluster whose RAW refined pile EXCEEDS the retention budget (the ultra-dense
small-loop-carrier signature); every under-budget cluster (all canonical / normal-density poses) keeps the
committed `sep` BYTE-IDENTICAL, so SSI-CAP's partition + split criterion is unchanged there. Recall-only by
construction: each emitted seed is a real refined on-both-surfaces transversal crossing (`refineRegion`
gates on-both ≤ `onSurfTol` AND ‖n₁×n₂‖ ≥ `tangentSinTol`); an over-split is dedup-collapsed by the S3
marcher (`retraces`). **Measured (host + SIM seed-1):** idx=33 seeds **1 → 2** and traces **BOTH loci to
complete BoundaryExit branches** (538 + 905 nodes, each on both surfaces ≤ 1.3e-9); idx=24 (single loop)
stays 1 seed (NO over-split); `DISAGREED == 0`; host `test_native_ssi_seeding`/`_exact_fuzz` green. The
recall gap is genuinely CLOSED (SIM idx=33: `covByNat 0→1, genuineMiss 2→1, genuineOcctOnNat 1.371→2.3e-3`).

**Honest conclusion (why it is NOT shipped — the sharpened floor).** The lever RECOVERS the locus but does
**NOT lower the decline**: the recovered glancing loop carries a `2.308e-3` native-vs-OCCT sub-`onCurve`
residual, so idx=33 stays declined — now as **multi-branch** (`traced=2`) rather than small-loop
(`traced=1`) — the SAME curvature-adaptive-marching moat that gates idx=23 (1.33e-3) and idx=24 (1.82e-3).
Seed-1 decline is **DECLINED=3, DISAGREED=0 BEFORE and AFTER** (AGREED 45/48 both; the exact same three
indices 23/24/33). Per the honest-decline discipline — a real recall gain that is a **no-op on the decline
metric** is the REVERT trigger, and shipping a change to the committed SSI-CAP split `sep` that does not
move the headline metric risks the SSI-CAP2 over-reach class — the behaviour change was **REVERTED**; the
SSI-CAP split `sep` is BYTE-UNCHANGED. Retained: the env-gated `SEED-DIAG` component-profile probe (pure
diagnostic) + the OCCT-free `replay_freeform_seed` instrument. **The true S4 tail on this seed is now one
mechanism, not three: a sub-`onCurve` (1.3…2.3e-3) native-vs-OCCT divergence on glancing near-tangent loops
that are FULLY SEEDED AND TRACED** — recoverable ONLY by curvature-adaptive marching (`maxDeflection` is
`scale·1e-3`, looser than the 1e-3 coverage gate), the person-decade moat, not a bounded seeding slice. No
tolerance widened, no locus fabricated, no single loop over-split; DISAGREED stayed 0 throughout.

### SSI-MARCH — curvature-adaptive marching does NOT close the residual; the tail is a coverage-EXTENT gap, not arc error · ⛔ HONEST-DECLINE 2026-07-14 (measurement CORRECTS the SSI-SMALLLOOP "sub-onCurve marching residual" framing; src/native BYTE-UNCHANGED)

SSI-SMALLLOOP framed the remaining seed-`0x5515D1FF0F0F` declines (idx=23/24/33) as a **sub-`onCurve`
native-vs-OCCT curvature-adaptive-marching residual** on glancing near-tangent loops. SSI-MARCH set out to
CLOSE that residual with a curvature-adaptive marching step and, MEASUREMENT-FIRST, **falsified the
framing**: the residual is NOT marching arc-error and the loops are NOT near-tangent. It is a
**coverage-EXTENT gap** — native traces only a short fragment of a much longer OCCT locus — which a finer /
curvature-adaptive step provably cannot touch.

**Direct host + OCCT measurements (OCCT-free `replay_freeform_seed` for the native geometry; SIM/OCCT
freeform-fuzz seed-`0x5515d1ff0f0f` `[DECLINE-DIAG]` for the coverage residual):**

- **The traces are WELL-CONDITIONED, not near-tangent.** idx=24 transversality sine ranges **0.30…0.36**;
  idx=33 **0.59…0.84** (near-tangent would be sine `< tangentSinTol = 1e-3`). The chord/arc SAGITTA is
  **6.3e-6** (idx=24) / **7.6e-6** (idx=33) — ~250× BELOW the `onCurve` 1e-3 gate and ~290× below the
  1.8e-3 residual. The marcher is already deflection-limited: forcing `maxDeflection → ∞` grows the step to
  5.8e-2 (11 nodes); the shipped `scale·1e-3` budget holds it at ≤ 7.3e-3 (41 nodes) at sagitta 6.3e-6. So
  the existing REACTIVE deflection controller is ALREADY curvature-adaptive and already dominates.
- **A curvature-adaptive step is a NO-OP on these poses.** A proactive step cap from the discrete curvature
  κ̂ of the last two chords (`h ≤ sqrt(8·maxDeflection/κ̂)`) was prototyped (additive, default-off,
  bounded by minStep). On idx=24 and idx=33 it is **byte-identical** even at a 20× tighter target fraction
  (0.05): κ̂ ≈ 0.8–0.96 gives a cap of ~0.17, far above the deflection-limited step of ≤ 9e-3, so it never
  bites. The high-frequency curvature that the reactive `stepDeflection` (reprojected-midpoint) controller
  actually responds to is finer than any 3-node discrete κ̂ can see. The prototype was **REVERTED**
  (`src/native/**` BYTE-UNCHANGED) — dead default-off code in the moat core is not justified when it moves
  no metric.
- **The trace is GEOMETRICALLY CONVERGED — refinement does not move the locus.** Tightening
  `maxDeflection` 100× (`scale·1e-5`, 41 → 3866 nodes, sagitta 6.3e-6 → 6.3e-10) moves the idx=24 locus by
  `< 1e-4` mid-arc. Node on-surface stays ~1.5e-11. The native curve is not step-error-limited.
- **THE ROOT CAUSE — a coverage-EXTENT gap.** idx=24: `traced=1 occtLines=1 covByNat=0 genuineMiss=1
  natOnOcct=1.733e-07 genuineOcctOnNat=1.818e-03 worstMissLen=3.938`. Native's nodes lie on the OCCT locus
  to **1.7e-7** (native is essentially EXACT), but native's traced arc is only **polyLen ≈ 0.203** while the
  OCCT locus is **length 3.938 — ~19× longer**. The `1.818e-3` `genuineOcctOnNat` is simply the closest
  native's short 0.20 fragment gets to the FAR reaches of OCCT's long locus. Native terminates as a genuine
  boundary-to-boundary `BoundaryExit` (front hits surface-A edge at param-gap 6.8e-6, back hits surface-B
  edge at 7.3e-5) after 0.20 arc; the FULL OCCT locus continues far beyond. Refining the step leaves the
  fragment length UNCHANGED (fine trace polyLen 0.2024). idx=23 is the analogous multi-branch case
  (`natOnOcct=6.5e-8` — native exactly on OCCT — with one distinct OCCT arc `worstMissLen=0.94` genuinely
  uncovered). The residual is therefore a **marching-TERMINATION / seeding-RECALL coverage gap** (native
  seeds+traces only a sub-arc of the true locus), NOT a sub-`onCurve` marching-accuracy divergence.

**Honest conclusion.** Curvature-adaptive marching is the WRONG lever for this tail and provably cannot
close it: the step is already fine (sagitta 250× under gate), the geometry is converged (100× refinement
moves the locus `< 1e-4`), and the fragment length is refinement-invariant (0.20 vs 0.20). The 1.3…1.8e-3
`genuineOcctOnNat` is a **coverage-extent artifact** of native covering a short sub-arc of a long
(3.94-unit) OCCT locus, not an arc-error moat. This RECLASSIFIES the SSI-SMALLLOOP framing: the seed-1 tail
is a marching-TERMINATION / recall-coverage gap (why does the native march/seed cover only a fragment of the
extended locus?), addressable — if at all — by the S4 termination/recall levers SSI-SMALLLOOP already found
exhausted for the seeding side, NOT by the marching STEP. Seed-1 decline is unchanged (**DECLINED=3,
DISAGREED=0**; idx 23/24/33). `src/native/**` is BYTE-UNCHANGED, `cc_*` byte-unchanged, SSI-CAP + SSI-SUBDIV
unchanged, no tolerance widened, no locus fabricated. Retained: the OCCT-free `replay_freeform_seed`
instrument (the host measurements above are reproducible through it with the env knobs
`MARCH_MAXDEFL`/`MARCH_MAXSTEP`). The person-decade moat here is marching-TERMINATION robustness + extended-
locus recall, not curvature-adaptive step placement — which the fixed-step trace already handles at
sagitta 6e-6.

### SSI-TERM — idx=24 does NOT terminate early; it fully covers + CLOSES. SSI-MARCH's "polyLen 0.203 / BoundaryExit" map FALSIFIED · ⛔ HONEST-DECLINE 2026-07-14 (no premature-termination bug; residual is near-tangent lateral divergence, refinement-invariant; `src/native/**` BYTE-UNCHANGED)

SSI-TERM set out to fix a PRESUMED premature-marching-termination bug: SSI-MARCH mapped idx=24 as a native
trace of **polyLen ≈ 0.203** of a **length-3.938 OCCT locus** (~19×), terminating as a boundary-to-boundary
`BoundaryExit` ("front hits surface-A edge at 6.8e-6, back hits surface-B edge at 7.3e-5"). SSI-TERM instrumented
the S3 tracer (`marchDir` termination: loop-closure / boundary-exit / step-cap / corrector-bail) and,
MEASUREMENT-FIRST, **the SSI-MARCH map does not reproduce on base `0942c2a`.** The presumed bug is absent.

**Direct measurement (OCCT-free `replay_freeform_seed` verbatim-reproducing the SIM's RNG + `grid=6/leaf=1/32`
+ default `MarchOptions`; a `CYBERCAD_SSI_MARCH_DIAG` env-gated `marchDir`-termination witness, reverted after;
SIM/OCCT freeform-fuzz seed-`0x5515d1ff0f0f` `[DECLINE-DIAG]` for the coverage residual):**

- **idx=24 traces the FULL extent and CLOSES — it does NOT stop early.** `status=Closed, nodes=714,
  polyLen=3.982` (≈ OCCT's 3.938), `onSurfResidual=1.99e-10`, `natOnOcct=1.7e-7` (native essentially EXACT on
  OCCT). Termination witness: `LoopClosed(step)` at step 712, `arcLen=3.976`, `sine=0.14` — a genuine
  full-circuit loop closure (the S4-f TRUE-RETURN gate `arcLen ≫ 2·closeRadius` + tangent-continuity, satisfied
  after a real circuit). FRONT≈BACK (uvA (0.240,0.429)↔(0.237,0.423)); `closeGap(front-back)=0.0116`. The
  footprint A:u[0.219,0.790]v[0.138,0.777], B:u[0.242,0.803]v[0.162,0.766] is DEEP INTERIOR — every node's edge
  gap ≥ 0.24, nowhere near a domain edge. **All four hypothesized termination causes are ABSENT:** not a
  premature loop-close (closes after a full 3.98 circuit), not a boundary-exit (footprint interior, no edge
  approach), not a step-cap (714 ≪ 20000 `maxPoints`), not a corrector-bail (onSurf 1.99e-10). SSI-MARCH's
  "0.203 / BoundaryExit / edge-6.8e-6" numbers are a STALE/WRONG map — pristine base-archive replay confirms 3.98
  Closed byte-for-byte.
- **THE ROOT CAUSE — a near-tangent LATERAL DIVERGENCE at a graze pinch, NOT a coverage-extent gap.** SIM idx=24
  `[DECLINE-DIAG]`: `traced=1 occtLines=1 covByNat=0 genuineMiss=1 natOnOcct=1.733e-07 genuineOcctOnNat=1.818e-03
  worstMissLen=3.938`. Native's whole 3.98 loop IS on the OCCT locus forward (`natOnOcct 1.7e-7`); the
  `genuineOcctOnNat=1.818e-3` is the REVERSE residual — one OCCT sample is 1.8e-3 from native's polyline at a
  single site. The scan localizes it: the loop has a near-tangent PINCH at node283 where transversality
  `sine=0.026` (well above `tangentSinTol=1e-3`, so no S4 stop) and the curve turns 12.26° in one step. There the
  two paraboloids graze, so the intersection root is LATERALLY ILL-CONDITIONED: native's corrector and OCCT's
  each land on their own root (both on-surface to ≤ their tol), and the two roots differ by 1.8e-3 laterally
  while the induced normal-distance change is `≈ 1.8e-3·sine ≈ 5e-5` — invisible to either `onSurf` gate. This is
  a native-vs-OCCT ACCURACY difference at a graze, JUST over the fixed 1e-3 `onCurve` gate — the SSI-SMALLLOOP
  curve-divergence moat, NOT the SSI-MARCH extent framing.
- **The residual is refinement-INVARIANT — tightening the step cannot close it and in fact BREAKS closure.**
  Tightening `maxDeflection` 100× (`scale·1e-5`) does NOT pull native's locus onto OCCT's: instead the step
  collapses (maxChord 7.5e-5), the loop no longer closes within `maxPoints`, and the trace degrades to a
  `Boundary(budgetExhausted)` open arc of polyLen 2.16 (`closeGap 1.115`). So the DEFAULT step (`scale·1e-3`,
  sagitta 6e-6, 714 nodes, clean closed loop) is already the correct operating point; the 1.8e-3 is not
  discretization error. This corroborates SSI-MARCH's "100× refinement moves the locus < 1e-4."

**Honest conclusion.** There is NO premature-marching-termination bug for idx=24 to fix — the trace covers the
full 3.98 extent and closes correctly, and its `1.818e-3` decline is the near-tangent lateral ill-conditioning
between two independent SSI solvers at a `sine≈0.026` graze pinch, which is refinement-invariant and provably
cannot be closed without WIDENING `onCurve` (forbidden) or FABRICATING a locus native's own converged root does
not have (forbidden). idx=23 (multi-branch) analogously traces all 4 corner arcs (line[0] polyLen 0.942 = the
mapped `worstMissLen`, on-surf 2.5e-11, genuine boundary-to-boundary exits) — again NO missed extent, a
`1.334e-3` graze divergence. idx=33 is a DISTINCT case (`occtLines=2 traced=1 genuineOcctOnNat=1.371
worstMissLen=4.755`): a genuine SEEDING-RECALL miss of a second co-resident locus (the SSI-SMALLLOOP finding),
not a termination bug. **Seed-`0x5515d1ff0f0f` unchanged: AGREED=45 DECLINED=3 DISAGREED=0** (idx 23/24/33),
matching the prior maps. `src/native/**` BYTE-UNCHANGED, `cc_*` byte-unchanged, SSI-CAP + SSI-SUBDIV unchanged,
no tolerance widened, no locus fabricated. Host suites GREEN on pristine src: `test_native_ssi_marching` 22/22,
`test_native_ssi_seeding` 11/11, `test_native_ssi_exact_fuzz` DISAGREED=0. Retained: the OCCT-free
`replay_freeform_seed` gains a trace-extent/footprint/min-sine scan dump (`REPLAY_DUMP_TRACE`) + `MARCH_MAXDEFL/
MARCH_MAXSTEP` refinement knobs — the instrument that reproduces all measurements above. The person-decade moat
here is near-tangent-graze ACCURACY parity (a laterally ill-conditioned root two solvers resolve differently),
and second-locus recall (idx=33) — NEITHER is a marching-termination defect; idx=24's termination is CORRECT.

### MESH-STRIP-IMPL — shared-seam-strip CACHE weld landed (fallback-only); fine-deflection multi-seam collapse SHARPENED to two distinct residuals · ◐ PARTIAL 2026-07-17 (weld mechanism lands + never-leaky; collapse-map corrected; `src/native/**` OCCT-free, `cc_*` byte-unchanged)

The M0-WELD gate carried a suspected fine-deflection failure: the shared-seam-strip weld of a
multi-seam curved↔curved solid was thought to collapse/degenerate as deflection → 0 (two annuli
sharing a closed seam whose INDEPENDENT per-face near-seam CDTs pair non-1:1 → a seam edge used
4× = NON-MANIFOLD whose residual grows with refinement). This track implemented the shared-seam-strip
CACHE weld (`src/native/tessellate/seam_strip.h` + `face_mesher.h`/`solid_mesher.h`) and MEASURED the
collapse precisely on the two-mirror-cup two-seam fixture (r₁≈0.131, r₂≈0.374), native-vs-closed-form.

- **The strip weld (LANDED, fallback-only, byte-identical elsewhere).** `SeamStripRegistry` (a
  SolidMesher pre-pass) detects every `isSeamChord` loop carried by EXACTLY TWO faces and records,
  from the SHARED bit-identical seam d.points, a concentric COLLAR ring + strip. A face carrying a
  registered seam substitutes its seam loop with the shared collar loop in the CDT, suppresses interior
  Steiner insertion inside the collar band (tested in the shared 3-D seam frame so both faces suppress
  IDENTICALLY), fills only the collar-outward remainder, and splices the fixed seam↔collar strip. Runs
  ONLY as a FALLBACK after the baseline + rim-pin passes fail to weld — so every already-watertight mesh
  is byte-identical, and the pass can only ever turn a non-watertight mesh watertight (or be discarded).
- **VERIFIED win — the fine-deflection multi-seam band welds watertight.** FUSE (the outer envelope whose
  survivors keep each wall's rim-to-r₂ background annulus, sharing the OUTER curved seam) welds into ONE
  watertight, coherent, positive-volume closed 2-manifold (χ consistent, every edge used exactly twice,
  be=0) across the fine band d ∈ [0.002, 0.0045], at the closed-form V(A)+V(B)−V(A∩B). COMMON/CUT (the
  annular lens sharing BOTH seams) weld watertight for d ≥ 0.002. Regression: `tests/native/test_native_seam_strip_weld.cpp` (FUSE d=0.0045 watertight; the never-leaky sub-threshold decline). This corrects
  the earlier `ffms_welds_fuse_outer_envelope` docstring's "below d≈0.004 always leaves a T-junction /
  honest-declines" framing — the fine band DOES weld watertight.
- **SHARPENED collapse map — two DISTINCT residuals, both refinement-measured, both honest-decline (NULL, never leaky), and the strip does NOT close either.** A byte-for-byte comparison against the pre-strip
  (main) mesher confirms the strip weld changes NO watertight OUTCOME on this fixture — it reduces the
  residual but does not flip a collapse to watertight:
  1. **COMMON/lens d ≤ 0.0018 — over-dense-seam weld-tolerance sliver.** The strip weld ELIMINATES the
     4×-used non-manifold (NONMANIF: 4×→0) but leaves a TINY OPEN-edge residual — exactly ONE degenerate
     triangle (3 unpaired edges, all AT the inner seam r₁, radius within 3e-6 of each other). Mechanism:
     the seam is sampled so densely that two adjacent seam vertices fall within the deflection-derived
     weld tolerance (`weldTol = max(deflection·0.5, 1e-7)`), so the welder merges them and collapses the
     strip quad into a sliver. This is the tradeoff the strip header itself predicted ("dropping the
     slivers to kill the non-manifold leaves OPEN edges"): the strip converts NON-MANIFOLD → OPEN, it does
     not close. Closing it needs a shared IDENTICAL seam-ring decimation (both faces) so no two consecutive
     collar/seam vertices sit within the weld tolerance — a mesher-core redesign, deferred.
  2. **FUSE d=0.004 — an ISOLATED SINGULAR-deflection baseline breakdown, UNRELATED to the shared seam.**
     At exactly d=0.004 (but NOT its neighbours 0.0045/0.0042/0.0038/0.0035/0.0032) the BASELINE mesher
     produces be=4439 open edges — a wholesale rim-to-r₂ background-annulus tessellation breakdown (a
     curvature-driven sampling coincidence), NOT the seam-as-hole 4×-collapse. It reproduces byte-for-byte
     on main (strip disabled), so it is a pre-existing frozen-mesher knife-edge, not introduced here; the
     strip weld does not rescue it and the verb correctly declines → NULL.

  Net: the shared-seam CACHE weld is a correct, safe, never-leaky mechanism that eliminates the
  non-manifold collapse and welds the fine multi-seam band watertight; the two remaining sub-threshold
  residuals above are the sharpened, honest-declining floor. `src/native/**` stays OCCT-free; `cc_*`
  byte-unchanged; no tolerance widened; the watertight self-verify is never bypassed.

## Sequencing & effort

```
substrate (#2 DONE) ──► S1 analytic (DONE) ──► S2 seeding (DONE) ──► S3 marching (DONE) ──► S4 robustness (moat)
                             │                                    │                          │
                             │                                    │                          ├─ S4-a coincident-region (DONE)
                             │                                    │                          ├─ S4-b tangent-classify (DONE)
                             │                                    │                          ├─ S4-c near-tangent march-through (FIRST SLICE DONE)
                             │                                    │                          └─ S4-d…f marching-core tail (PENDING)
                             └──────────────► S5 curved booleans ◄─┘  ──► #6 blends ──► #7 wrap-emboss
                                              (S5-a/b/c/d/e/f/g/h/i/j: drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT (3/3) + Steinmetz COMMON/FUSE/CUT (3/3) + coaxial cone∩cyl COMMON/FUSE/CUT (3/3) + coaxial cone∩sphere single- AND two-circle COMMON/FUSE/CUT (3/3) + coaxial cone∩cone frustum AND apex-to-apex hourglass COMMON/FUSE/CUT (3/3) + two-circle cyl∩sphere COMMON/FUSE/CUT (3/3) native ✓)
```

| Stage | Effort (robust) | Nature |
|---|---|---|
| S1 analytic SSI | ✅ DONE at the bar | bounded, closed-form — 17 analytic pairs verified vs OCCT |
| S2 seeding | ✅ DONE at the bar (transversal) | subdivision + substrate refine — verified host + sim recall |
| S3 marching | ✅ DONE at the bar (transversal) | tangent-step + substrate re-projection — 5 pairs / 9 branches vs OCCT |
| S4-a coincident-region | ✅ DONE at the bar | typed `CoincidentRegion` (analytic + seeded); classification vs OCCT `IntAna_Same` |
| S4-b tangent-classify | ✅ DONE at the bar | typed `TangentContact` (point/curve/near-tangent/undecided) — 8 pairs vs OCCT, 0 deferred |
| S4-c near-tangent march-through | ◐ FIRST SLICE DONE at the bar | fixed-plane-cut corrector marches a single-branch graze the S3 truncated (sphere∩offset-cyl: `nearTangentGaps → 0`, full loop on OCCT locus); branch saddle still defers |
| S4-d…f marching-core tail | multi-year, ongoing | the moat tail — branch points, singularities, self-intersect, deeper near-coincident bands; best-effort + fallback |
| S5 curved booleans | ◐ slices S5-a/b/c/d/e/f/g/h/i/j DONE at the bar (~months for full) | through-drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT (op-set COMPLETE 3/3) + branched Steinmetz COMMON/FUSE/CUT (op-set COMPLETE 3/3, `16R³/3`+incl-excl vs OCCT) + coaxial cone∩cyl COMMON/FUSE/CUT (op-set COMPLETE 3/3, CONE family, dual oracle: `V_frustum` inclusion-exclusion + OCCT) + coaxial cone∩sphere SINGLE-circle COMMON/FUSE/CUT (op-set COMPLETE 3/3, dual oracle: `V_frustum + V_spherical-segment` + OCCT) + coaxial cone∩sphere TWO-circle COMMON/FUSE/CUT (op-set COMPLETE 3/3, S5-h, dual oracle: `V_sph-seg + V_frustum + V_sph-seg` incl-excl + OCCT, CUT disconnected 2-body) + coaxial cone∩cone frustum COMMON/FUSE/CUT (op-set COMPLETE 3/3, S5-g, dual oracle) + coaxial cone∩cone APEX-TO-APEX HOURGLASS COMMON/FUSE/CUT (op-set COMPLETE 3/3, S5-j, apex-terminated bicone; dual oracle: full-cone `V_frustum(r,0,Δh)` incl-excl + OCCT) + TWO-circle cyl∩sphere COMMON/FUSE/CUT (op-set COMPLETE 3/3, S5-i, `tanα==0` special case of S5-h, dual oracle: `V_sph-seg + V_cyl-seg + V_sph-seg` incl-excl + OCCT, CUT disconnected 2-body) native (wt, ΔV ≤ 9e-4, native-pass=30, sim 36/0/6); transversal/apex-crossing cone pairs + general non-Steinmetz branched + apex-spanning cone∩sphere + more families remain |

SSI + curved booleans total ≈ **1.5–3 py** (substrate-accelerated) for *usable*
coverage; full OCCT-grade robustness (S4) is the long tail. Recommended cadence:
**S1 first** (highest bang-for-buck, unlocks elementary-pair curved booleans via
S5-restricted), then S2→S3, with S4 hardened continuously and S5 following as soon
as S1 (elementary) or S3 (freeform) curves are available.

## Honest framing

- Each stage ships as a **narrow verified slice + explicit OCCT fallback** — like
  planar booleans, box∩cylinder, and native threads did.
- **S4 is why "drop OCCT" stays a long-horizon goal**: the intersection *algorithm*
  is tractable on our substrate; the *robustness* on adversarial real-world inputs
  is the person-decade OCCT moat, re-earned only incrementally. The
  **detection/classification** layer (S4-a coincident-region, S4-b typed
  tangent-contact) is now landed and verified vs OCCT; the **marching core**
  (S4-c march-through-tangency, S4-d branch points, S4-e singularities, S4-f
  self-intersection) is the remaining tail — a `NearTangentTransversal` is typed
  and handed to OCCT, never traced natively yet.
- Shape healing (#4) and STEP import (#3) remain **separate parallel tracks**
  also gating `drop-occt` (both have landed first native slices); IGES is DESCOPED
  (STEP-only). They are not part of this SSI roadmap.
