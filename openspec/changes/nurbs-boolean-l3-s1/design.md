# Design — nurbs-boolean-l3-s1

## Context

`openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` measured, stage by stage, how far the
existing kernel pieces get toward a general exact-NURBS B-rep boolean, and named the
first tractable slice (L3-S1: a NURBS face split by a plane) plus the exact composition
that reaches it while routing around every MISSING stage. This change implements that
composition as a bounded, two-gate-proven verb.

## Goals / Non-goals

- **Goal:** a shipped, verified verb that cuts a genuine NURBS `FaceSurface`
  (`Kind::BSpline`) by a planar half-space, welded watertight, gated by closed-form
  volume + OCCT parity.
- **Non-goal:** the general BOPAlgo-class NURBS↔NURBS boolean (the §4 deep tail). This
  slice deliberately handles ONE NURBS wall whose `wall ∩ P` is a closed interior seam,
  plus its flat closing base.

## Key decisions

1. **Self-contained verb, not an extension of `curvedWallHalfSpaceCut`.** That verb
   depends on `recogniseFreeformSolid`, whose admission gate only admits **Bézier**
   walls. Rather than widen that recognizer (a coordination-sensitive, owned path),
   `nurbsFacePlaneSplit` takes the NURBS `wall` FACE + flat `base` FACE directly and
   composes the same downstream pieces. This keeps the change additive and localized to
   one new header, touching neither `assemble.h` nor `face_split.h`.

2. **Route around `constructPcurve` by reading the WLine `(u,v)`.** The readiness map
   measured `constructPcurve` as PARTIAL at the boolean bar. The seam pcurve on the
   NURBS wall is instead the WLine's per-node `(u1,v1)`, which the marcher already lands
   on the operand. Its correctness is guarded explicitly: `S(u1,v1) == node.point ≤ tol`
   (the `S(pcurve)==C` invariant) — a drifted seam is REJECTED (`SeamOffSurface`). This
   is the honest substitute for the general pcurve construction the slice does not need.

3. **Surface-kind-agnostic split.** `splitFaceSmoothTrim` reads the wall through
   `tess::SurfaceEvaluator`, which evaluates a BSpline/NURBS grid natively (rational and
   non-rational). It required NO change to run on a `Kind::BSpline` face; its own tiling
   self-verify (`areaInside + areaOutside == parent`) confirms the split on the NURBS
   grid.

4. **Curved↔FLAT weld (M0w pin), avoiding the freeform↔freeform sew.** The kept NURBS
   sub-face welds to a flat cap on `P` via the same straight-chord seam nodes
   (`cwcdetail::synthCircularCap`), so the M0 mesher position-welds them. This is the
   dominant readiness risk (G5) and it resolved cleanly: watertight, χ=2,
   volume-convergent.

5. **Mandatory self-verify + honest decline.** The verb meshes the result and requires
   watertight AND positive enclosed volume; any decline returns NULL with a measured
   `NurbsPlaneSplitDecline`. No tolerance is weakened; no leaky solid is ever emitted.

## Oracle / verification

- **Host (a):** a `Kind::BSpline` degree-2 bowl reproduces `z = a·(x²+y²)` EXACTLY (a
  clamped degree-2 B-spline reproduces a quadratic), so the closed-form volume oracle is
  exact ON THIS surface (not a fit). CUT = `π·ρ²·c/2`, COMMON = `V(full)−that`, partition
  closure `V(below)+V(above)=V(full)`.
- **Sim (b):** the SAME operand is a `Geom_BSplineSurface` in OCCT, cut by
  `BRepAlgoAPI_Cut`; native vs OCCT volume/watertight/χ within the curved-tessellation
  band; OCCT cross-checked against the closed form.

## Risks

- The M0w curved↔flat weld watertightness on a genuine NURBS grid (G5) — retired by the
  host + sim gates.
- The stage-1 seed of the closed interior loop on this operand — measured to land here
  (the bowl-cup pose the M2 fixtures already trace); a pose the seeder misses would
  HONEST-DECLINE (`SeamUnusable`), never fabricate.
