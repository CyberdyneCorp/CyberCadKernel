# Proposal — nurbs-boolean-l3-s1

## Why

`openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` maps the NURBS-roadmap **Layer 3**
(the exact-NURBS B-rep boolean) into five stages and identifies the FIRST tractable
slice — **L3-S1: a genuine NURBS face SPLIT BY A PLANE, welded watertight** — as the
one boolean that composes ONLY the pieces the readiness harness measured as
WORKS/near-WORKS and routes around every MISSING stage. It is the flagship: the entry
point of the exact-NURBS boolean moat. This change LANDS that slice.

Today the closest verb, `boolean/curved_wall_cut.h` (`curvedWallHalfSpaceCut`), lands
the closed-interior-seam half-space CUT/COMMON for a freeform wall that is a
degree-elevated **Bézier** patch (`FaceSurface::Kind::Bezier`). No boolean yet cuts a
**genuine B-spline / NURBS** face (`Kind::BSpline`, non-rational or rational). L3-S1 is
that same weld with the wall's surface kind left as a real NURBS `FaceSurface` — the
first boolean whose curved operand is an exact NURBS face rather than a single Bézier
patch.

The slice is deliberately bounded to the measured-reachable envelope and gated by the
two-gate discipline (host closed-form volume + SIM vs OCCT `BRepAlgoAPI_Cut`), so it is
a proven milestone, never a shaky general boolean.

## What Changes

1. **A new native boolean verb** `src/native/boolean/nurbs_plane_split.h`
   (`nurbsFacePlaneSplit`, namespace `cybercad::native::boolean`, OCCT-free,
   header-only, `clang++ -std=c++20`). Given a trimmed NURBS wall FACE
   (`Kind::BSpline`, non-rational first / rational admitted) whose `wall ∩ P` is a
   CLOSED interior seam, its flat closing base (a Plane face), a cut plane `P`, and a
   keep `side`, it composes the L3-S1 recipe:
   - **stage 1 TRACE** — `wall ∩ P` → the closed interior seam WLine, via a NURBS
     operand adapter (`ssi::makeBSplineAdapter` non-rational / `ssi::makeNurbsAdapter`
     rational) + `ssi::makePlaneAdapter` + `ssi::trace_intersection` (the measured
     transversal-WORKS path).
   - **stage 2 PCURVE (routed around `constructPcurve`)** — the seam pcurve on the
     NURBS wall is READ DIRECTLY from the WLine's per-node `(u1,v1)`, gated by an
     explicit fidelity check `S(u1,v1) == node.point ≤ tol` (the S(pcurve)==C invariant;
     a drifted seam is REJECTED, never welded).
   - **stage 3 SPLIT** — `splitFaceSmoothTrim` (surface-kind-agnostic, evaluates the
     BSpline/NURBS grid natively) partitions the NURBS wall into the enclosed disk + the
     annulus (seam as a hole), with its own tiling self-verify on the NURBS grid.
   - **stage 4 KEEP** — a plane HALF-SPACE side test at the kept sub-face's trim
     centroid (closed-form `signedDist` + `onKeepSide`), no general NURBS solid
     membership.
   - **stage 5 WELD** — synthesize ONE flat cap on `P` bounded by the seam polyline
     (the M0w curved↔FLAT weld, reusing `cwcdetail::synthCircularCap`) + the kept flat
     base → Shell → Solid; mesh (M0) and require watertight AND positive enclosed
     volume.

   It CONSUMES existing pieces byte-identically (`ssi::make*Adapter` /
   `trace_intersection`, `splitFaceSmoothTrim`, `cwcdetail::{seamLoop3d,
   loopAreaOnPlane, synthCircularCap}`, `hscdetail::{signedDist, onKeepSide}`, the M0
   `SolidMesher` / `isWatertight` / `enclosedVolume`) and MODIFIES none of them
   (`assemble.h` / `face_split.h` untouched).

2. **Two-gate acceptance tests** (additive, no lib change, no `cc_*` ABI change):
   - **Host GATE (a)** `tests/native/test_native_nurbs_plane_split.cpp` +
     `tests/native/nurbs_plane_split_fixture.h` — the OCCT-free closed-form proof: a
     genuine NURBS-walled bowl-cup (a `Kind::BSpline` degree-2 bowl reproducing the
     paraboloid `z = a·(x²+y²)` EXACTLY + a flat lid) cut by the horizontal plane z=c
     into CUT (Below) and COMMON (Above), each welding watertight (Euler χ=2) at the
     closed-form volume (π·ρ²·c/2 and V(full)−that) with partition closure
     V(below)+V(above)=V(full). A non-NURBS wall / no-closed-seam plane HONEST-DECLINES
     to NULL with a measured reason.
   - **Sim GATE (b)** `tests/sim/native_nurbs_plane_split_parity.mm` +
     `scripts/run-sim-native-nurbs-plane-split.sh` — native vs OCCT `BRepAlgoAPI_Cut` on
     the reconstructed NURBS bowl-cup (a `Geom_BSplineSurface`), asserting volume /
     watertight / topology parity within a curved-tessellation band.

3. **Docs**: `docs/NURBS-SCOPE.md` §4 Layer-3 row updated (❌ → first slice landed:
   NURBS face ∩ plane); `openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` marks L3-S1 as
   landed.

**Explicitly OUT OF SCOPE (the L3 deep tail, each a MISSING readiness stage):**
general NURBS↔NURBS split where BOTH operands are curved (needs the freeform↔freeform
sew), closed-interior-loop seams the stage-1 seeder misses (recall gap), multi-crossing
/ re-entrant / hole-crossing splits, general point-in-trimmed-NURBS-solid membership,
and a boolean-grade general `constructPcurve`.

## Impact

- **Affected specs:** `native-booleans` (ADDED: the L3-S1 exact-NURBS face-split verb).
- **Affected code:** NEW `src/native/boolean/nurbs_plane_split.h`; NEW tests +
  fixture + sim harness + run script; docs. `src/native` stays OCCT-free (OCCT only in
  the `.mm` sim oracle). `cc_*` ABI byte-unchanged (internal geometry, no facade entry).
  `src/native/ssi`, `src/native/topology/trimmed_nurbs`, `src/native/math` NOT modified;
  `boolean/assemble.h` and `boolean/face_split.h` NOT modified.
