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

### S1 — Analytic SSI (elementary-surface pairs) · workflow-sized (~weeks)
Closed-form intersection curves for elementary pairs: plane∩plane (line),
plane∩{cylinder,cone,sphere,torus} (line/conic/circle), cylinder∩cylinder
(≤quartic), cylinder∩sphere, sphere∩sphere (circle), etc. **No marching** — pure
math (generalizes the plane∩cylinder we already ship). Returns exact
`Line`/`Circle`/`Ellipse`/conic + `Geom`-quality curves.
- **Verify:** host analytic (known conics) + sim vs OCCT `GeomAPI_IntSS`.
- **Unlocks:** most CAD-primitive curved booleans (S5 restricted to elementary faces).

### S2 — Subdivision seeding · (~weeks–months)
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
substrate (#2 DONE) ──► S1 analytic ──► S2 seeding ──► S3 marching ──► S4 robustness (moat)
                             │                                    │
                             └──────────────► S5 curved booleans ◄─┘  ──► #6 blends ──► #7 wrap-emboss
```

| Stage | Effort (robust) | Nature |
|---|---|---|
| S1 analytic SSI | ~weeks | bounded, closed-form — workflow-achievable |
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
