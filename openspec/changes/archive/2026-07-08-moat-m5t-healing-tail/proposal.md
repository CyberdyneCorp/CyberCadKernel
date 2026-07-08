# Proposal — moat-m5t-healing-tail (MOAT M5 healing tail)

## Why

The landed healer (sew / unify / orientation + bounded gap-bridging + a single
simple **planar-hole cap**) closes exactly ONE missing planar face. The very next
defect it DECLINES is a shell missing **two or more** planar faces: the sew leaves
**several disjoint** rings of boundary edges, `traceSingleLoop` sees more than one
cycle, and `capPlanarHole` returns `declined` → the heal reports
`Unhealed{OpenShell}` (proven by the landed host test `heal_cap_two_holes_declines`).

Among the three candidate tail defects, this is the ONE most-tractable bounded
opt-in slice with a clean 1:1 OCCT `ShapeFix` oracle. The diagnosis (recorded in
`design.md`) rejects the other two as the arbitrary-broken-B-rep asymptotic tail:

- **Non-planar / curved missing-face cap** — synthesizing a curved cap requires
  fitting a support surface to the free boundary and re-approximating a freeform
  patch to match OCCT `BRepFill` / `GeomPlate`; there is no closed-form host volume
  for an arbitrary curved hole and no bounded 1:1 oracle. DECLINED (stays OCCT's moat).
- **Self-intersecting-wire repair** — reordering / splitting a tangled wire has no
  closed-form host oracle and no bounded acceptance test. DECLINED (stays OCCT's moat).
- **Multi-hole planar cap (≥ 2 disjoint simple planar holes)** — a DIRECT, small
  generalization of the landed single-hole cap: trace ALL boundary cycles instead of
  one, run the SAME planarity + simple-polygon layers per loop, cap EACH on its
  existing shared nodes, and let the UNCHANGED mandatory self-verify be authoritative.
  It has a **pristine analytic host oracle** (a unit cube missing two OPPOSITE planar
  faces caps to watertight `V = 1.0`) and a **clean 1:1 OCCT oracle** (one
  `BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire)` per hole + `ShapeFix_Shell` /
  `ShapeFix_Solid`). **CHOSEN.**

A throwaway feasibility spike over the landed code already drove the exact
`heal.cpp` tail — trace all cycles → cap each planar loop → re-sew → orient →
self-verify — on a cube missing two opposite faces and reached
`isWatertight == true`, `signedVolume == 1`, tessellated `enclosedVolume =
1.000000000000`. The slice is reachable, not speculative.

This is additive-only and the existing healer paths stay **byte-identical**: the
multi-hole pass is gated behind a NEW, default-off opt-in `capMultiplePlanarHoles`;
with it `false` (and with the landed `capPlanarHoles`) `healShell` behaves exactly
as today, including `heal_cap_two_holes_declines`. No `cc_*` entry point, signature,
or POD changes; `src/native/**` stays OCCT-free; the tolerance is NEVER weakened.

The honest DECLINE stays first-class and LIKELY-per-input: any branching boundary
(two ADJACENT missing faces share a degree-4 corner), any non-planar loop, or any
self-intersecting loop in the set declines the WHOLE shell as `Unhealed{OpenShell}`
with the input UNCHANGED — never a partial or fabricated closure.

## What Changes

1. **One new, default-off opt-in flag** `HealOptions.capMultiplePlanarHoles`
   (default `false`). When `false`, `healShell` is byte-identical to the landed
   slices (the landed single-hole `capPlanarHoles` path is untouched). When `true`,
   a shell that sews cleanly but is missing **two or more** faces MAY be closed by
   synthesizing one planar cap per hole — but ONLY when EVERY surviving boundary loop
   is a single simple cycle, coplanar within `tolerance`, and non-self-intersecting.

2. **A `cap_hole.h` generalization (additive)**: a `traceAllLoops` that partitions
   the boundary graph into ALL disjoint simple cycles (empty ⇒ any branching /
   non-closing boundary ⇒ decline whole), and a `capPlanarHoles(sr, tol)` that runs
   the EXISTING per-loop `bestFitPlane` / `maxPlaneDeviation` / `isSimplePolygon`
   layers on each cycle and emits one cap `FaceLoop` per accepted loop, or declines
   the whole set. The landed `capPlanarHole` (single) and its helpers are UNCHANGED.

3. **A tiny `heal.cpp` branch (additive, opt-in-guarded)**: when
   `capMultiplePlanarHoles == true`, append every emitted cap to the working soup,
   then run the UNCHANGED re-sew → orientation-fix → assemble → mandatory self-verify.
   `HealMetrics.nCappedFaces` now carries the count (`≥ 2` for a multi-hole heal);
   `maxCapPlanarityDev` carries the worst accepted-loop deviation (`≤ tolerance`). No
   new `UnhealedReason` (a declined multi-hole set stays `OpenShell`, exactly as the
   single-hole slice reuses `OpenShell`).

4. **Two verification gates** (the non-negotiable discipline):
   - **GATE A — HOST ANALYTIC (no OCCT):** a unit cube missing two OPPOSITE planar
     faces caps to a watertight solid at the closed-form `enclosedVolume = 1.0` with
     `nCappedFaces == 2`; and the DECLINES are asserted — two ADJACENT missing faces
     (branching degree-4 boundary), a two-hole set with one non-planar loop, and a
     self-intersecting loop each return `Unhealed{OpenShell}` with the input UNCHANGED.
     Default-off (`capMultiplePlanarHoles == false`) leaves `heal_cap_two_holes_declines`
     unchanged.
   - **GATE B — SIM native-vs-OCCT:** on a booted iOS simulator, the native multi-hole
     cap matches an OCCT reference built by ONE `BRepBuilderAPI_MakeFace(gp_Pln,
     freeBoundaryWire)` per hole + `ShapeFix_Shell` / `ShapeFix_Solid` — same
     watertight closed solid, same enclosed volume vs `BRepGProp`, at fixed
     (never-widened) tolerances; and the branching two-adjacent-holes fixture defers
     (native `Unhealed`, engine falls through to OCCT).

Out of scope (honest declines, tracked, stay OCCT's moat): a NON-PLANAR / curved
hole, a SELF-INTERSECTING boundary, a BRANCHING boundary (adjacent missing faces),
pcurve reconstruction, and self-intersecting-wire repair. These DECLINE at the heal
boundary (`Unhealed{OpenShell}`, input unchanged) and defer to OCCT `ShapeFix`.
