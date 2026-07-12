# Proposal — trimmed-nurbs-seam-healing (NURBS roadmap Layer 8, L8-SEAM)

## Why

`trimmed-nurbs-healing` and `trimmed-nurbs-healing-depth` landed gap-close / snap / pinch-split
(2-way + general N-way / crossing) and rational pcurve construction, leaving ONE named residual:
**healing across a surface's parametric SEAM**. On a periodic / closed surface (a full cylinder,
cone, sphere, or torus — u ∈ [0, 2π) with u=0 ≡ u=2π the SAME physical curve) a trim loop such as a
full cross-section circle CROSSES the u-seam: its (u,v) polyline leaves u=uPeriod and re-enters at
u=0. The existing classify / heal treat the seam as a HARD boundary, so a wrapped loop is
mis-handled — split into two open arcs, or classified wrong because a full wrap should enclose the
WHOLE u-band, not a thin slab. Neither classify, pcurve, nor healing identified u=0 and u=uPeriod.

This change closes that residual where it is mathematically closable and declines honestly (with a
measured note) where the seam topology is genuinely ambiguous. It is purely additive: every seam
verb is a NEW function and every seam option DEFAULTS OFF, so the existing entry points are
byte-compatible.

## What

Extend `src/native/topology/trimmed_nurbs.{h,cpp}` (no `shape.h` change, no `cc_*` change,
`src/native` stays OCCT-free, only existing math + topology is `#include`d) with:

1. **`surfacePeriod()`** — report a `FaceSurface`'s parametric periodicity. The analytic quadrics
   are periodic in the angular sweep u (period 2π): Cylinder / Cone / Sphere periodic in u; Torus
   in BOTH u and v. Plane and (this slice) free-form BSpline / Bezier are reported NON-periodic —
   a closed free-form surface's seam is an explicit residual, declined rather than guessed.

2. **`loopCrossesSeam()` / `healSeamLoop()` / `healTrimLoopSeam()`** (returning a `SeamHealReport`)
   — DETECT that a flattened loop crosses the u-seam (a consecutive u-jump of ~one full period, or
   a seam-tangent touch), then STITCH across it by UNWRAPPING the loop's u so u=0 and u=uPeriod are
   identified: a wrapped loop becomes ONE continuous polyline (one closed seam-crossing loop, not
   two arcs). The report distinguishes a **FULL WRAP** (the unwrapped u-span reaches a full period —
   the whole ring is enclosed) from a **FINITE STRADDLING region** (u-arc < period straddling the
   seam), records the net winding, and DECLINES (`ambiguous`) a seam-tangent graze (a touch without
   a clean crossing) or a non-simple wrap. Region-preserving: the unwrap only ADDS an exact multiple
   of the period (u and u+period are the SAME physical point on the periodic surface), so no
   interior/exterior verdict is flipped; a non-crossing loop is echoed byte-identically.

3. **`classifySeam()`** — point-in-trimmed-region WITH seam identification. Identical to `classify()`
   for a non-periodic surface or a loop that does not touch the seam (a strict NO-OP superset,
   byte-for-byte the same verdict). For a periodic surface whose outer loop CROSSES the seam, the
   loop is unwrapped and the query u is reduced modulo the period into the loop's unwrapped window
   before the raycast — a FULL wrap classifies EVERY u inside the v-band as `In` (the whole u-band
   is enclosed), a FINITE straddling region encloses exactly its u-arc (seam-continuous). Holes on a
   periodic surface are seam-healed the same way. A seam-tangent (ambiguous) loop declines `Unknown`.

## Scope / residuals

- Seam-crossing healing closes the "healing across a surface's parametric seam" residual for the
  analytic-periodic family (cylinder / cone / sphere / torus) and the two clean seam topologies
  (full wrap, finite straddling region). A **seam-tangent touch** and a **non-simple wrap** are
  DECLINED honestly (`ambiguous` / `Unknown`), never mis-wrapped.
- A **closed free-form (BSpline/Bezier) surface's seam** is NOT auto-detected here — `surfacePeriod`
  reports it non-periodic, so `classifySeam` is a strict no-op there (byte-identical to `classify`).
  Detecting a closed free-form surface's period from its poles/knots remains an explicit residual,
  reported never faked. Re-parametrizing a badly-drifting pcurve also remains a documented residual.

`cc_*` unchanged; `src/native` stays OCCT-free; the existing `classify` / `healLoop` /
`splitAtPinch(es)` API + their default-OFF behaviour are byte-unchanged (additive only). No existing
`FaceSurface` / `PCurve` / `step_reader` consumer changes.
