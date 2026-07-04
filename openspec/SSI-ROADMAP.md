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

### S2 — Subdivision seeding · **← NEXT** · (~weeks–months)
Find ≥1 seed point per intersection branch for **freeform** (NURBS) pairs:
recursive patch bounding-box-overlap subdivision → candidate regions → refine to a
point with `least_squares(S1(u1,v1) − S2(u2,v2) = 0)` (substrate). Must find *all*
branches (recall), not just one.
- **Verify:** every seed lies on both surfaces (≤tol); branch recall vs OCCT on test pairs.
- **Risk:** completeness (missing a small loop) — the honest failure mode.

### S3 — Marching-line tracer (WLine) · (~months)
From each seed, walk the intersection curve: tangent = normalize(n₁×n₂), adaptive
step, **re-project** onto both surfaces via the substrate (Newton/LM), until the
curve closes or exits a boundary; fit a B-spline through the polyline. This is
OCCT's `IntWalk`/`WLine`, on our substrate.
- **Verify:** sampled curve points on both surfaces (≤tol); curve length/shape vs
  OCCT `IntPatch` on non-tangent freeform pairs.

### S4 — Tangent / degeneracy robustness · (research-grade; best-effort + fallback)
Near-tangent stepping (n₁×n₂→0: step control, higher-order predictor),
coincident/overlapping-surface detection, branch points & singularities,
self-intersection guards. **This is the moat** — OCCT's decades of tuning. Lands
as *progressively hardened*; whatever isn't robust **falls back to OCCT** and is
reported with the measured gap. Never "done"; hardened over time.

### S5 — Curved booleans via SSI (the payoff) · (~months on top of S1–S3)
Use SSI curves to **split** the curved faces of two solids, **classify**
fragments inside/outside (reuse the BSP-CSG classifier + a curved point-in-solid
test), **assemble** the surviving shell watertight (curved-seam weld from the
mesher). Extends `src/native/boolean/` from planar/axis-aligned to general curved.
- **Verify:** native-vs-OCCT `BRepAlgoAPI` (volume/area/watertight) on
  cylinder∩cylinder, sphere∩box, cone∩box, fillet-shaped tools; self-verify →
  OCCT fallback for the rest.
- **Unlocks:** curved blends (#6) and curved wrap-emboss (#7) then compose on top.

## Sequencing & effort

```
substrate (#2 DONE) ──► S1 analytic (DONE) ──► S2 seeding (NEXT) ──► S3 marching ──► S4 robustness (moat)
                             │                                    │
                             └──────────────► S5 curved booleans ◄─┘  ──► #6 blends ──► #7 wrap-emboss
```

| Stage | Effort (robust) | Nature |
|---|---|---|
| S1 analytic SSI | ✅ DONE at the bar | bounded, closed-form — 17 analytic pairs verified vs OCCT |
| S2 seeding | ~weeks–months | subdivision + substrate refine |
| S3 marching | ~months | core algorithm on substrate |
| S4 tangent robustness | multi-year, ongoing | the moat — best-effort + fallback |
| S5 curved booleans | ~months | extends existing assembler |

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
  is the person-decade OCCT moat, re-earned only incrementally.
- Shape healing (#4) and STEP/IGES import (#3) remain **separate parallel tracks**
  also gating `drop-occt`; they are not part of this SSI roadmap.
