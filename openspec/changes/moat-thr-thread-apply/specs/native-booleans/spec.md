# native-booleans

## ADDED Requirements

### Requirement: Native `threadApply` attempts the thread↔shaft boolean under a full self-verify and honest-declines to OCCT

The native boolean layer SHALL provide an additive OCCT-free header-only verb
`threadApply(shaft, thread, op, deflection, why, analyticVolume)`
(`src/native/boolean/thread_apply.h`, namespace `cybercad::native::boolean`) that applies a
helical thread body to a shaft — `op 0` FUSES the crest (external thread), `op 1` CUTS the
groove (internal thread) — by reusing the landed native substrate. The verb SHALL:

1. recognise the tractable input family — the shaft an axis-parallel finite cylinder via
   `curved::recogniseCylinder`, and the thread a coaxial native helical-ridge solid whose
   crest/root radii and z-extent are measured from its mesh (crest = max radius, root = min
   radius) — declining `ShaftNotCylinder` / `ThreadDegenerate` / `CrestBelowShaft`
   otherwise;
2. facet BOTH operands into consistently-oriented planar-triangle B-rep solids at a
   controlled faceting deflection (each triangle a `construct::detail::planarFace` carrying
   the mesh triangle's winding normal), so the exact planar BSP set-algebra applies;
3. run the landed planar BSP boolean `boolean_solid(shaftFacets, threadFacets, op)`
   (declining `BooleanEmpty` on a null/degenerate result);
4. run a MANDATORY four-part self-verify on the result — WATERTIGHT (`tess::isWatertight`)
   AND Euler characteristic χ = V − E + F = 2 AND consistently-oriented
   (`tess::isConsistentlyOriented`, the DIRECTED-edge invariant with zero same-direction
   duplicate half-edges) AND a positive finite enclosed volume within a TWO-SIDED
   deflection-bounded band of the closed-form threaded-shaft volume (FUSE: `V_shaft < V ≤
   V_shaft + V_thread`, tracking `V_shaft + V_ridge_outside`; CUT: `V_shaft − V_thread ≤ V <
   V_shaft`, tracking `V_shaft − V_groove_inside`) — declining `NotWatertight` /
   `NotOriented` / `VolumeInconsistent` on any failure;
5. return the welded native solid ONLY when all four checks pass, else a NULL `Shape` with
   a typed `ThreadApplyDecline`.

The engine (`NativeEngine::thread_apply`) SHALL route two native operands through the verb,
keep a verified non-null result native, and otherwise forward the SAME arguments to the
OCCT `thread_apply` oracle; a mixed native/OCCT or OCCT-only operand pair SHALL forward to
OCCT. The verb SHALL NEVER emit a leaky / partial / orientation-inconsistent / wrong-volume
solid, SHALL NEVER forward a native void to OCCT, and SHALL NOT weaken any tolerance to
force a pass. The change SHALL be strictly ADDITIVE: it SHALL NOT modify
`construct/thread.h`, the M0 tessellator (mesher/CDT/weld), `boolean_solid`, the `curved.h`
recognizers, or any landed boolean header, SHALL consume their primitives BYTE-IDENTICAL,
SHALL remain OCCT-free, SHALL introduce no `cc_*` ABI surface, and SHALL keep its
per-function cognitive complexity within the backend band.

Because the native helical-thread solid (`construct::build_thread`) is watertight but NOT
consistently oriented (`tess::sameDirectionEdgeCount` is non-zero) and the near-tangent
helical root ↔ shaft-wall contact fragments the dense-soup BSP into T-junction cracks, a
multi-turn thread SHALL honest-decline to OCCT with a measured typed reason (`NotOriented`
/ `NotWatertight` / `VolumeInconsistent`) — the correct fallthrough until an
orientation-coherent thread builder + robust dense-soup CSG (M7b) land. The same verb SHALL
weld a tractable planar-cutter case (a faceted cylinder CUT by a box) to a watertight,
χ = 2, consistently-oriented solid at the analytic volume, demonstrating the machinery is
sound.

#### Scenario: The verb welds the tractable planar-cutter baseline and honest-declines the thread (host, no OCCT)

- GIVEN a native axis-Z cylinder shaft and (a) a native box cutter crossing its wall and
  (b) a native multi-turn helical thread coaxial with it, built on the host with NO OCCT
- WHEN `threadApply` runs the recognise → facet → `boolean_solid` → four-part self-verify
  pipeline for the cylinder−box CUT and for the thread FUSE / CUT across a deflection sweep
- THEN the shaft B1-recognises as a cylinder while a box shaft declines `ShaftNotCylinder`;
  the closed-form oracle tiles (`0 < V_fuse − V_shaft ≤ V_thread`, `0 < V_shaft − V_cut ≤
  V_thread`); the cylinder−box CUT returns a NON-NULL `Shape` with `why == Ok` that is
  WATERTIGHT AND χ = 2 AND consistently-oriented (`tess::isConsistentlyOriented`) at the
  analytic cut volume within the deflection band; and the thread FUSE / CUT returns a NULL
  `Shape` with a measured `ThreadApplyDecline` (`NotOriented` / `NotWatertight` /
  `VolumeInconsistent`), NEVER a leaky / partial / misoriented solid

#### Scenario: The native verb matches the OCCT oracle on the baseline and correctly falls through on the thread (sim, native-vs-OCCT)

- GIVEN the SAME cylinder shaft, box cutter and helical thread reconstructed in OCCT on a
  booted iOS simulator
- WHEN the native `threadApply` and OCCT `BRepAlgoAPI_{Fuse,Cut}` + `BRepGProp` (the ORACLE)
  are computed for the baseline and the thread
- THEN the native cylinder−box CUT welds and its meshed volume matches OCCT
  `BRepAlgoAPI_Cut` + `BRepGProp` within the deflection-bounded band; the native thread
  FUSE / CUT DECLINES to NULL (the correct honest fallthrough) while OCCT's per-turn
  `thread_apply` accumulate produces the reference threaded-shaft volume — the native path
  never emits a leaky / wrong solid, and OCCT owns the case
