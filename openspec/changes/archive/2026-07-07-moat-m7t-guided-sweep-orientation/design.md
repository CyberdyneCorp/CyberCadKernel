# Design — moat-m7t-guided-sweep-orientation

## Context

M7a declined the guided-orient sweep after a native "rigid guide-aimed up-axis" law was
volume-correct but ~7% spatially wrong (`bboxΔ 0.54`). The banked lesson: a self-verify
that checks only volume/watertightness will WRONGLY ACCEPT a spatially-wrong solid. This
design retries with the law OCCT's `MakePipeShell + SetMode(guideWire)` (NoContact)
actually realizes, scoped to a straight spine, gated on a SPATIAL metric.

## The oracle law (reverse-engineered from OCCT source)

Source of truth (not assumed): `GeomFill_GuideTrihedronPlan::D0` and
`GeomFill_LocationGuide::D0` (`rotation == false` branch).

`GeomFill_GuideTrihedronPlan::D0(param → T, N, B)`:
1. `P = spine(param)`; `T = spine Frenet tangent`.
2. Solve for guide parameter `W` such that `guide(W)` lies in the plane through `P` with
   normal `T` — `GeomFill_PlanFunc E(P, T, guide)` + `math_FunctionRoot`. Call it `Pprime`.
   This is the **perpendicular-plane correspondence**.
3. `N = normalize(Pprime − P)`; `B = T × N` (unit).

`GeomFill_LocationGuide::D0(param → M, V)` with `rotation == false` (NoContact):
- `V = P` (translation), `M = cols(N, B, T)` (rotation). No scaling, no per-station angle
  root-find, no homothety. (Those are the *contact*-mode branch. The `Approx_Curve3d`
  guide fit is `#ifdef DRAW` only.)

Section placement: the profile's local `(x, y)` map to `(N, B)`, local origin to `P`:
`world(x, y) = P + x·N + y·B`.

### Why this is natively reproducible on a STRAIGHT spine

For a straight spine along direction `T̂` (constant), the plane perpendicular to `T` at
`P` is a fixed-direction plane sliding along the spine. Intersecting it with a **polyline
guide** is an elementary segment/plane intersection (find the guide segment straddling the
plane, lerp) — **no guide surface, no BSpline fit, no OCCT** is required. The frame is then
`N = normalize(Pprime − P)`, `B = T × N`. The station→band ruled assembler already used by
the twisted/guided sweeps welds the rings.

The only OCCT-specific machinery on a straight spine is (i) the guide root-find, which
degenerates to a segment/plane intersection, and (ii) the final swept-surface BSpline
approximation, whose deflection is bounded and which the native ruled tube matches within
the tessellation bound (the same bound the shipped native sweeps already meet).

## Host-analytic validation (GATE a — no OCCT)

Closed-form probe (`scripts`/host, no OCCT), straight `+Z` spine `z∈[0,H]`, rectangle
half-extents `(a, b)` with `a ≠ b` (so orientation is spatially observable):

| Config | Guide | Check | Result |
|---|---|---|---|
| A | offset line `(ρ,0,z)` | volume vs exact prism `4ab·H` | `relErr = 0.0` |
| B | rotating `(ρcosθ, ρsinθ, z)`, `θ=Θz/H` | `angle(N)` vs closed form `Θz/H` | `2.2e-16 rad` |
| B | rotating | volume → `4ab·H` under refinement | `4.37e-3 → 1.09e-3 → 2.73e-4` (ratio `4.0`, quadratic) |
| — | guide-aimed vs orientation-blind | bbox spread | `2.16` (orientation is spatially load-bearing) |

Interpretation: the law is well-defined; volume is exact on a non-twisting guide and
converges quadratically on a twisting guide (the residual is the ruled-surface
discretization deficit OCCT shares); orientation matches an independent computation
exactly; and orientation error is spatially visible — so a bbox/Hausdorff gate is both
necessary and sufficient to catch the M7a failure mode, which a volume gate is not.

## Decision procedure (implement-or-decline)

1. Wire the OCCT oracle `guided_orient_sweep` = `MakePipeShell + SetMode(guideWire)` (NoContact).
2. Wire the native straight-spine builder (perpendicular-plane frame, no scaling).
3. Run GATE (b) on a booted simulator: `cc_guided_orient_sweep` native vs OCCT on
   {volume, area, watertight, face/edge topology} **AND {bbox corner Δ, Hausdorff}**.
4. **If** the native path meets the SPATIAL tolerance (`bboxΔ` and Hausdorff within the
   deflection bound) on the straight-spine fixtures → ship the native path guarded by the
   SPATIAL self-verify + OCCT fallback.
   **Else** → keep the native builder returning NULL → OCCT for the missing regime and ship
   a measured decline recording the residual spatial gap. Either way NO spatially-wrong
   solid is emitted.

Non-straight (kinked/curved) spines are **out of slice**: the spine Frenet frame is
discontinuous at polyline vertices and OCCT applies transition smoothing that the straight
slice deliberately does not model → NULL → OCCT.

## Self-verify (SPATIAL — this is the M7a fix)

`NativeEngine::guided_orient_sweep` accepts the native candidate iff ALL hold, else
forwards to OCCT:
- native solid non-null, robustly watertight, strictly positive volume;
- `|vol_native − vol_occt| / vol_occt` within tolerance;
- **`maxBBoxCornerDelta(bbox_native, bbox_occt)` within the linear deflection tolerance**;
- **Hausdorff(native surface, occt surface) within the deflection bound**.

The bbox/Hausdorff clauses are mandatory: volume equality is necessary but NOT sufficient
(a rigidly-mis-rotated section is volume-correct — the M7a trap). The tolerance is the
existing deflection bound; it is NOT weakened.

## Alternatives considered

- **Repurpose `cc_guided_sweep`** — REJECTED. The task forbids it (scale-splay semantics,
  different oracle). New additive `cc_guided_orient_sweep`.
- **Re-attempt the M7a rigid parameter-fraction aim** — REJECTED. Measured trap
  (`bboxΔ 0.54`). Corrected to the perpendicular-plane correspondence.
- **Volume/watertight-only self-verify** — REJECTED. Blind to orientation error; the M7a
  mistake. Replaced by the SPATIAL gate.
- **General curved/kinked spine now** — DEFERRED. Spine-frame transition smoothing is not
  modeled by the straight slice → OCCT.
