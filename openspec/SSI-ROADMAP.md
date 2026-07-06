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

### S4 — Tangent / degeneracy robustness · ◐ CLASSIFICATION LAYER (S4-a/b) + MARCHING-CORE SLICES (S4-c graze, S4-d branch points, S4-e sphere-pole/cone-apex chart singularities, S4-f robust closure + self-intersection guard + completeness critic) DONE AT THE BAR; S4-d/e general/freeform + S4-f general topology repair pending
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

#### S4-d — Branch points · ◐ FIRST HONEST SLICE DONE AT THE BAR (elementary transversal self-crossing — the Steinmetz family; general/freeform/cusp branches remain)
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

- **Honest scope / risk:** only the **elementary two-real-distinct-line transversal
  self-crossing** (the Steinmetz family) is traced. **General/freeform branch points**
  (arbitrary self-crossings on freeform surfaces, three-plus tangent lines at one point),
  **cusps** (double root of the tangent-cone quadratic), and **general small-loop / topology
  repair (the S4-f residual)** all remain DEFERRED → OCCT, reported with the measured gap, never
  faked. (**S4-e chart singularities** — the sphere parametric pole + cone apex — are now crossed
  natively; and a **single-arm figure-eight self-intersection** is now DETECTED + traced-through
  as typed data by the S4-f guard — `branchPts=0`, distinct from this locus branch — though it is
  not yet split into sub-arcs; see the S4-e and S4-f slices below.)
- **Unlocks:** **Steinmetz is now unblocked** natively; the multi-arm `TraceSet` +
  `BranchNode` connectivity is available to S5 curved booleans for self-crossing loci.

#### S4-e — Singularities · ◐ FIRST HONEST SLICE DONE AT THE BAR (sphere parametric pole + cone apex crossed; general/freeform param degeneracies + higher-order cusps remain)
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

- **Honest scope / risk:** only the two **elementary chart (removable) singularities** —
  the **sphere parametric pole** and the **cone apex** — are crossed, each verified
  node-by-node on both surfaces + on the OCCT locus. **General / freeform parametric
  singularities** (NURBS degenerate edges, collapsed spline poles), **higher-order / curve
  cusps** (the curve's own velocity → 0 where the point-based step cannot recover a far side),
  and the **general small-loop / self-intersection topology residual (S4-f)** all remain
  DEFERRED → OCCT, reported with the measured gap, never faked. (S4-f now detects + traces a
  single-arm self-crossing and recovers small loops the fixed subdivision misses at a MEASURED
  recall floor — see the S4-f slice below — but that is a raised floor, not a completeness
  proof.) Any pole/apex whose point-based crossing does not verify on both surfaces defers the
  same honest way.
- **Unlocks:** transversal intersection curves that pass through a sphere pole / cone apex are
  now traced end-to-end natively instead of truncating at the chart singularity.

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

### S5 — Curved booleans via SSI (the payoff) · ◐ NATIVE SLICES S5-a/b/c/d landed (branched-trace Steinmetz op-set now COMPLETE 3/3 native; ~months for full coverage)
Use SSI curves to **split** the curved faces of two solids, **classify**
fragments inside/outside (reuse the BSP-CSG classifier + a curved point-in-solid
test), **assemble** the surviving shell watertight (curved-seam weld from the
mesher). Extends `src/native/boolean/` from planar/axis-aligned to general curved.
- **Verify:** native-vs-OCCT `BRepAlgoAPI` (volume/area/watertight) on
  cylinder∩cylinder, sphere∩box, cone∩box, fillet-shaped tools; self-verify →
  OCCT fallback for the rest.
- **Unlocks:** curved blends (#6) and curved wrap-emboss (#7) then compose on top.

**S5-a/b/c + S5-d done at the bar (changes `add-native-ssi-curved-boolean` archived +
`add-native-ssi-curved-boolean-wider` + `add-native-ssi-branched-boolean` archived
`2026-07-05`):** the SSI-curve-driven split→classify→select→weld pipeline lives in
`src/native/boolean/ssi_boolean.{h,cpp}` (OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated, consumes the
S3 `TraceSet` — and, for S5-d, the S4-d branched re-trace with `MarchOptions.enableBranchPoints
= true`). It now produces **ten native curved-boolean sub-cases verified vs OCCT
`BRepAlgoAPI_{Fuse,Cut,Common}`** (sim parity `native-pass=12` — the sphere∩sphere AND the
Steinmetz bicylinder op-sets are now COMPLETE 3/3 native, and the harness runs each of the
sphere FUSE/CUT as an equal-R AND an unequal-R fixture; 6 honest fallbacks):
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

Honest scope still declining → OCCT (measured NULL fallbacks, never faked):
- **oblique / multi-tube cyl∩cyl**, and other curved-curved families (cyl∩cone, cyl∩sphere,
  cone∩cone, sphere∩box, freeform), plus any branched pair that is NOT equal-R orthogonal
  Steinmetz (unequal-R / non-orthogonal / ≠ 2 branch points / ≠ 4 arms) → decline. A disjoint
  Steinmetz pair (no seam) also declines for all three ops. (sphere∩sphere AND Steinmetz
  FUSE/CUT/COMMON are now NATIVE — see S5-c/S5-d above.)
Remaining S5 work: general (non-Steinmetz) branched pairs, and more curved-curved families.

## Sequencing & effort

```
substrate (#2 DONE) ──► S1 analytic (DONE) ──► S2 seeding (DONE) ──► S3 marching (DONE) ──► S4 robustness (moat)
                             │                                    │                          │
                             │                                    │                          ├─ S4-a coincident-region (DONE)
                             │                                    │                          ├─ S4-b tangent-classify (DONE)
                             │                                    │                          ├─ S4-c near-tangent march-through (FIRST SLICE DONE)
                             │                                    │                          └─ S4-d…f marching-core tail (PENDING)
                             └──────────────► S5 curved booleans ◄─┘  ──► #6 blends ──► #7 wrap-emboss
                                              (S5-a/b/c/d: drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT (3/3) + Steinmetz COMMON native ✓)
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
| S5 curved booleans | ◐ slices S5-a/b/c/d DONE at the bar (~months for full) | through-drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT (op-set COMPLETE 3/3) + branched Steinmetz COMMON/FUSE/CUT (op-set COMPLETE 3/3, `16R³/3`+incl-excl vs OCCT) native (wt, ΔV ≤ 9e-4, native-pass=12); general non-Steinmetz branched + more families remain |

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
