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

### S4 — Tangent / degeneracy robustness · ◐ CLASSIFICATION LAYER (S4-a/b) DONE AT THE BAR; marching-core (S4-c…f) pending
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

#### S4-c — Marching THROUGH a tangency · ✗ PENDING (the hard core of the moat)
Near-tangent stepping, higher-order predictor across the degeneracy,
fabricating the curve where the two surfaces graze-and-cross. `NearTangentTransversal`
is currently classified and handed to OCCT; S4-c is what would trace it natively.

#### S4-d — Branch points · ✗ PENDING
Splitting the trace where intersection branches meet/cross (a point where n₁×n₂→0
but the locus is not a simple tangency).

#### S4-e — Singularities · ✗ PENDING
Degenerate surface points (parametric poles, apex/edge singularities) on the
intersection locus.

#### S4-f — Self-intersection / small-loops · ✗ PENDING
Self-intersection guards and small-loop recovery below the seeding resolution floor.

### S5 — Curved booleans via SSI (the payoff) · ◐ FIRST NATIVE SLICE landed (~months for full coverage)
Use SSI curves to **split** the curved faces of two solids, **classify**
fragments inside/outside (reuse the BSP-CSG classifier + a curved point-in-solid
test), **assemble** the surviving shell watertight (curved-seam weld from the
mesher). Extends `src/native/boolean/` from planar/axis-aligned to general curved.
- **Verify:** native-vs-OCCT `BRepAlgoAPI` (volume/area/watertight) on
  cylinder∩cylinder, sphere∩box, cone∩box, fillet-shaped tools; self-verify →
  OCCT fallback for the rest.
- **Unlocks:** curved blends (#6) and curved wrap-emboss (#7) then compose on top.

**S5-a/b/c done at the bar (changes `add-native-ssi-curved-boolean` archived +
`add-native-ssi-curved-boolean-wider`):** the SSI-curve-driven
split→classify→select→weld pipeline lives in `src/native/boolean/ssi_boolean.{h,cpp}`
(OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated, consumes the S3 `TraceSet`). It now produces
**five native curved-boolean sub-cases verified vs OCCT `BRepAlgoAPI_{Fuse,Cut,Common}`**
(sim parity `native-pass=5`, 13 honest fallbacks):
- **S5-a — through-drill cylinder∩cylinder COMMON** (unequal radii, transversal two-loop
  trace) — watertight, ΔV = 8.1e-04, ΔA = 2.8e-04.
- **S5-b — through-drill cylinder∩cylinder FUSE + CUT** (assembler-only extension: fat wall
  with the two mouths cut out + planar-facet caps + reversed tunnel band / protruding end
  tubes) — watertight, ΔV = 8.8e-05 (fuse) / 4.0e-05 (cut).
- **S5-c — sphere∩sphere COMMON** (single closed seam → the lens of the two inside-the-other
  spherical caps, welded along the one seam; direction-slerp cap facets, robust even when the
  cap apex sits at the sphere's parametric pole) — watertight, ΔV = 4.1e-04 (equal radii) /
  4.7e-04 (unequal radii).

Honest scope still declining → OCCT (measured NULL fallbacks, never faked):
- **Steinmetz** (equal-radius orthogonal cyl∩cyl) is **near-tangent** (`nearTangentGaps > 0`)
  → an **S4** case → declines.
- **sphere FUSE / CUT** (outer-cap union + re-trimmed remainder weld) → deferred → declines.
- **oblique / multi-tube cyl∩cyl**, tangent/coincident (incl. Steinmetz), and other
  curved-curved families (cyl∩cone, cyl∩sphere, cone∩cone, sphere∩box, freeform) → decline.
Remaining S5 work: sphere fuse/cut, more curved-curved families, and lifting the near-tangent
gate once S4 lands.

## Sequencing & effort

```
substrate (#2 DONE) ──► S1 analytic (DONE) ──► S2 seeding (DONE) ──► S3 marching (DONE) ──► S4 robustness (moat)
                             │                                    │                          │
                             │                                    │                          ├─ S4-a coincident-region (DONE)
                             │                                    │                          ├─ S4-b tangent-classify (DONE)
                             │                                    │                          └─ S4-c…f marching-core (PENDING)
                             └──────────────► S5 curved booleans ◄─┘  ──► #6 blends ──► #7 wrap-emboss
                                              (S5-a/b/c: drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON native ✓)
```

| Stage | Effort (robust) | Nature |
|---|---|---|
| S1 analytic SSI | ✅ DONE at the bar | bounded, closed-form — 17 analytic pairs verified vs OCCT |
| S2 seeding | ✅ DONE at the bar (transversal) | subdivision + substrate refine — verified host + sim recall |
| S3 marching | ✅ DONE at the bar (transversal) | tangent-step + substrate re-projection — 5 pairs / 9 branches vs OCCT |
| S4-a coincident-region | ✅ DONE at the bar | typed `CoincidentRegion` (analytic + seeded); classification vs OCCT `IntAna_Same` |
| S4-b tangent-classify | ✅ DONE at the bar | typed `TangentContact` (point/curve/near-tangent/undecided) — 8 pairs vs OCCT, 0 deferred |
| S4-c…f marching-core | multi-year, ongoing | the moat tail — march-through-tangency, branch points, singularities, self-intersect; best-effort + fallback |
| S5 curved booleans | ◐ slices S5-a/b/c DONE at the bar (~months for full) | through-drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON native vs OCCT (wt, ΔV ≤ 8e-4); sphere fuse/cut + more families + near-tangent gate remain |

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
- Shape healing (#4) and STEP/IGES import (#3) remain **separate parallel tracks**
  also gating `drop-occt`; they are not part of this SSI roadmap.
