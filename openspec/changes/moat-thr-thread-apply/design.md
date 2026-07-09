# Design — moat-thr-thread-apply (native `cc_thread_apply`)

## Context

`cc_thread_apply(shaft, thread, op)` applies a helical thread body to a shaft: `op 0` FUSES
the crest (external thread), `op 1` CUTS the groove (internal thread). It is the boolean of
a shaft cylinder with a swept helical thread ridge. OCCT hangs on a single-shot boolean of
a fine multi-turn helix (the app's #1 wall, GitHub #286); the OCCT adapter survives only by
rebuilding the thread per-turn and accumulating bounded booleans under a wall-clock budget.

The native engine previously hard-declined `thread_apply` on any native operand. The M2
freeform-boolean substrate + the planar-polyhedron BSP boolean (`boolean_solid`) are now
landed, so the native attempt is: recognise the tractable input, facet both operands to
planar-triangle solids, run the exact planar BSP set-algebra, and self-verify.

## Goals / non-goals

- GOAL: attempt `thread_apply` natively for the tractable input family (cylindrical shaft +
  coaxial native helical thread), reusing the landed BSP boolean, with a MANDATORY
  self-verify (watertight + χ=2 + consistently-oriented + two-sided closed-form volume).
- GOAL: on any self-verify failure, honest-decline to OCCT with a typed reason — never a
  leaky / partial / orientation-inconsistent / wrong-volume solid, never a native void
  forwarded to OCCT.
- NON-GOAL: fixing the `build_thread` orientation defect or adding robust dense-soup CSG
  with T-junction repair — both are the sharpened next blocker (M7b), out of scope here and
  explicitly NOT patched (the tessellator + `construct/thread.h` stay byte-identical).

## The tractable input + closed-form oracle

- SHAFT: an axis-parallel finite cylinder, recognised by `curved::recogniseCylinder`
  (axis/radius/extent). Volume `V_shaft = π r² L` (analytic).
- THREAD: a native helical-ridge solid coaxial with the shaft; crest radius `Rc` (max mesh
  radius), root radius `Rr` (min mesh radius), z-extent from the mesh bbox — mirroring the
  OCCT oracle's `measureThread` / `measureRootRadius`.
- CLOSED-FORM threaded-shaft volume (the two-sided verify oracle):
  - FUSE: `V ≈ V_shaft + V_ridge_outside`, bounded `V_shaft < V ≤ V_shaft + V_thread`.
  - CUT: `V ≈ V_shaft − V_groove_inside`, bounded `V_shaft − V_thread ≤ V < V_shaft`.
  The ridge/groove volume is measured from the thread mesh (exact for the tessellated
  ridge); the band scales with the faceting deflection (a triangulated ridge under-counts
  by O(deflection)).

## The pipeline

1. `facetSolid(solid, d)` — mesh at deflection `d` and rebuild each triangle as a
   `construct::detail::planarFace` carrying the triangle's winding normal (Forward), so the
   faceted solid inherits the mesh's own consistent winding and `extractPolygons`/BSP see
   outward normals.
2. `boolean_solid(shaftFacets, threadFacets, op)` — the landed planar BSP set-algebra
   (`op 0` fuse, `op 1` cut). Both operands are all-planar, so the exact planar path runs.
3. Self-verify gauntlet (the engine keeps native ONLY if all pass):
   - `isWatertight` (every undirected edge used exactly twice);
   - Euler χ = V − E + F = 2 (a genuine closed sphere-topology boundary);
   - `isConsistentlyOriented` (no same-direction duplicate half-edge — the signed volume is
     meaningful);
   - a positive finite enclosed volume within the two-sided closed-form band.
4. Any failure → NULL `Shape` + typed `ThreadApplyDecline` → OCCT fallthrough.

## Why this declines today (measured, not claimed)

- The native `build_thread` solid is watertight but NOT consistently oriented (measured
  `sameDirectionEdgeCount == 6` — a latent cap/band winding defect). An orientation-
  inconsistent operand breaks the BSP's inside/outside classification, so the fused/cut
  result is wrong or non-manifold. The self-verify's orientation gate catches it.
- Independently, the near-tangent helical root ↔ shaft-wall contact fragments the dense
  triangle-soup BSP into T-junction cracks (`boundaryEdgeCount` 15–140 across single-turn
  to 4-turn threads, insets 0.6–1.5, deflections 0.05–0.15). The watertight gate catches it.
- ISOLATION: the SAME verb welds a faceted cylinder CUT by a box `bnd=0, sd=0` at the
  analytic volume — so the BSP substrate is sound; the defect is specific to the thread
  operand's orientation + the helical near-tangency.

## Risks

- Faceted-BSP cost on dense soup: bounded — the tested cases run in ~0.1–0.2 s (well under
  any budget); the guard is the self-verify, not a timer.
- False-positive weld: the four-part self-verify (watertight AND χ=2 AND oriented AND the
  two-sided volume band) makes a wrong solid passing all four vanishingly unlikely; the
  orientation gate specifically rejects the watertight-but-misoriented shell whose signed
  volume would otherwise pass a one-sided bound.

## Alternatives considered

- Direct helical-ridge-to-cylinder shared-vertex assembly (no BSP): needs curved↔curved
  helical seam welding — beyond the M2 closed-planar-seam substrate; deferred to M7b.
- Healing the cracked BSP output (`heal/`): a band-aid over T-junctions the heal self-verify
  itself rejects; not robust.
- Per-turn accumulate like OCCT: reproduces the oracle's approximation without its healed
  fuzzy booleans; still gated on the same orientation defect.
