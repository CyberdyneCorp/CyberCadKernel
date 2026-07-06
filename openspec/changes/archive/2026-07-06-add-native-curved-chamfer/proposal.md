## Why

Feature **#6 CURVED BLENDS** has landed the CURVED FILLET family — a CONSTANT-radius
rolling-ball fillet on a CONVEX circular crease (`add-native-curved-fillet`, archived),
its CONCAVE sibling (`add-native-concave-fillet`), and the first VARIABLE-radius fillet
(`add-native-variable-fillet`). All three round the rim where a cylinder LATERAL face
meets a coaxial PLANAR cap by inserting a G1-TANGENT torus canal.

The CHAMFER family, by contrast, is still native ONLY for PLANAR-dihedral edges.
`cc_chamfer_edges(body, edgeIds, count, distance)` today runs natively via
`src/native/blend/chamfer_edges.h` when the picked edge is a CONVEX edge between two
PLANAR faces — it slices the corner off with the single plane through the two setback
lines (a planar cut), re-welds, and self-verifies. A CURVED edge (a circular rim) falls
straight through to OCCT `BRepFilletAPI_MakeChamfer`.

This change lands the FIRST CURVED chamfer slice: a SYMMETRIC-distance chamfer on the
SAME convex circular cylinder↔cap rim the curved fillet handles. A chamfer is NOT a
fillet — it cuts a FLAT BEVEL, not a rounded tangent arc:

- the cylinder wall is set back AXIALLY by `distance = d` → a **cylinder seam** CIRCLE at
  radius `Rc`, axial `H − s·d` (`s` = axial sign toward the cap);
- the cap is set back RADIALLY by `d` → a **cap seam** CIRCLE at radius `Rc − d`,
  axial `H`;
- the band between the two setback circles is a **CONE FRUSTUM** (a straight bevel), NOT
  a torus arc — a ruled truncated cone from `(Rc, H − s·d)` to `(Rc − d, H)`;
- the bevel is **C0, NOT G1**: it meets each face at the setback line at the chamfer
  angle (for a SYMMETRIC distance, 45° to the cylinder wall and 45° to the cap), so the
  frustum outward normal is the exact bisector of the two face normals (`cos = 1/√2` to
  each, NOT `cos = 1`). Asserting G1 tangency here would be WRONG — a chamfer is a
  straight bevel, not tangent.
- the chamfer REMOVES material, so the solid's volume DECREASES (`0 < Vr < Vo`). The
  removed region is the sharp-corner ring ABOVE the bevel — the right-triangle corner
  (legs `d × d`, area `d²/2`) revolved about the axis — of exact closed-form volume
  `V_removed = π·d²·(Rc − d/3)` (Pappus, triangle centroid radial `Rc − d/3`).

The curved chamfer REUSES the curved fillet's whole classification + weld discipline —
the same `detail::facesOnRim` + `detail::rimGeom` rim recognition, the same `math::Ax3`
cylinder frame, the same `sagittaSteps` angular facet budget, the same `ringPoint`
sampler, the same planar-triangle weld through the boolean `assembleSolid` — and differs
ONLY in that the band between the two trim seams is a straight cone-frustum band (one
meridian step, exact) rather than a torus quarter-tube (a curved `M`-step arc). It trims
to the two SETBACK circles instead of the two TANGENT circles.

## What Changes

- Add a native **curved (circular-rim) chamfer** builder
  `curved_chamfer_edge(const topo::Shape&, const int* edgeIds, int edgeCount,
  double distance, double deflection)` in a new OCCT-free header
  `src/native/blend/curved_chamfer.h`, aggregated by `native_blend.h`. It runs NATIVELY
  when `cc_chamfer_edges(body, {edge}, 1, distance)` selects a CIRCULAR edge shared by
  exactly two faces — a `FaceSurface`-kind `Cylinder` lateral face and a coaxial
  `FaceSurface`-kind `Plane` CAP — meeting at a CONVEX dihedral (the SAME classifier the
  curved fillet uses, `detail::facesOnRim` + `detail::rimGeom`).
- Construct the chamfer as a **cone-frustum bevel band** on the air side:
  - the two trim seams are CIRCLES — the cylinder seam at radius `Rc`, axial `H − s·d`
    (the wall set back axially), and the cap seam at radius `Rc − d`, axial `H` (the cap
    set back radially);
  - the band between them is a single straight frustum (ruled cone) — tiled into `N`
    angular quads (each split into two exactly-planar triangles, since a frustum quad is
    not coplanar), NO meridian subdivision (the bevel is a straight line);
  - the frustum meets the cylinder wall and the cap at the chamfer angle, **C0 not G1**:
    the builder SELF-VERIFIES the correct BEVEL geometry — each seam lies on its
    neighbour surface by construction (`radius = Rc` at the wall seam, `axial = H` at the
    cap seam) AND the frustum normal makes the chamfer angle with each face normal
    (`cos = 1/√2` for the symmetric distance), asserting it is NOT tangent (`cos ≠ 1`).
- **Rebuild + weld** (reusing the curved fillet's planar-facet discipline): rebuild the
  capped-cylinder region as ONE deflection-bounded planar-triangle soup — the far cap
  (radius `Rc`); the cylinder wall from the far end up to the cylinder seam (`H − s·d`);
  the cone-frustum bevel band `Rc @ (H − s·d) → (Rc − d) @ H`; and the TRIMMED cap
  (radius `Rc − d` at `H`) — all sharing the SAME `N` angular samples so the seams weld
  with coincident vertices, closed through the boolean `assembleSolid`.
- **Wire the native curved path into `cc_chamfer_edges`** in `native_engine.cpp`:
  `NativeEngine::chamfer_edges` changes from `planar-chamfer → error/OCCT` to
  **`planar-chamfer → curved-circular-chamfer → error/OCCT`**, each candidate accepted
  ONLY through the SAME `blendResultVerified(result, body, wantGrow=false)` guard
  (a chamfer REMOVES material — the SHRINK branch). A NULL builder result OR a failed
  self-verify DISCARDS the candidate and the engine reports honestly (the OCCT engine
  serves the call). No new guard type, no weakened tolerance.
- Native code stays **OCCT-free**. The cone-frustum band uses only `math::Ax3` +
  header math; the seams are closed-form circles (no solver, no NUMSCI substrate). No
  `cc_*` signature or POD struct changes.

**No `cc_*` ABI change.** `cc_chamfer_edges` keeps its exact signature; the symmetric
curved (circular-rim) chamfer is a new NATIVE path behind it, OCCT for the rest.

## Capabilities

### New Capabilities
<!-- none — this EXTENDS the existing native-blends capability with the curved
     (circular-rim) chamfer; it does not introduce a new capability. -->

### Modified Capabilities
- `native-blends`: add a native `cc_chamfer_edges` CURVED path for the CONVEX circular
  rim (cylinder↔coaxial cap) with a SYMMETRIC chamfer distance `d`. The chamfer surface
  is a CONE FRUSTUM (a straight bevel) between the two SETBACK circles (cylinder seam at
  `Rc`, axial `H − s·d`; cap seam at `Rc − d`, `z = H`); the frustum meets each face at
  the setback line at the chamfer angle **C0 (NOT G1)** — the builder self-verifies the
  correct bevel geometry (seam on-surface + frustum normal at the chamfer angle,
  `cos = 1/√2` for symmetric `d`), NEVER asserts tangency. The chamfer REMOVES material,
  so the volume SHRINKS (`0 < Vr < Vo`, removed `= π·d²·(Rc − d/3)`); the patch welds
  watertight through the boolean `assembleSolid` and is accepted only through the SHRINK
  self-verify, else DISCARDED → OCCT `BRepFilletAPI_MakeChamfer` (`Add(distance, edge)`).
  Asymmetric two-distance chamfers, non-circular / concave / curved↔curved / tilted /
  freeform creases, `Rc ≤ d`, and multi-edge selections stay OCCT-fallthrough (never
  faked). Delivered behind the unchanged `cc_chamfer_edges` signature via
  `engine-adapter`.

## Impact

- **ABI**: none. `cc_chamfer_edges` keeps its signature and POD structs; the symmetric
  curved circular chamfer is an internal native path with OCCT fallback.
- **Build**: adds the OCCT-free header `src/native/blend/curved_chamfer.h` (reusing the
  curved fillet's `detail::facesOnRim` / `rimGeom` / `cylinderInfo` / `sagittaSteps` /
  `ringPoint` helpers via `#include "native/blend/curved_fillet.h"`), aggregates it in
  `native_blend.h`, and extends `NativeEngine::chamfer_edges` from a single native
  (planar) attempt to a native planar → native curved → error dispatch, both gated by
  the existing `wantGrow=false` self-verify. Consumes `native-math` (`Ax3`, cylinder /
  plane frames, cone/frustum geometry), `native-boolean` (`assembleSolid` weld),
  `native-tessellate` (watertight mesher — NOT modified). Builds with NUMSCI OFF (the
  seams are closed-form) and NUMSCI ON (regression).
- **Verification**: two gates. **host** (OCCT-free CTest): the cone-frustum bevel band +
  both setback circles lie on their surfaces within tol, the result meshes watertight,
  its volume equals `|body| − π·d²·(Rc − d/3)` within the tessellation deflection bound,
  and the bevel geometry is CORRECT — the frustum normal makes the chamfer angle
  (`cos = 1/√2` for symmetric `d`) with EACH face normal, explicitly NOT tangent
  (`cos ≠ 1`), C0. **sim** native-vs-OCCT parity (`run-sim-native-curved-chamfer.sh` +
  its `.mm`): the native curved chamfer vs OCCT `BRepFilletAPI_MakeChamfer`
  (`Add(distance, edge)`) on a cylinder top rim (volume / area / watertight + the
  cone-frustum bevel-angle geometry, NOT G1), over two distance fixtures. Because the
  symmetric chamfer IS EXACTLY a cone frustum, the native↔OCCT gap is bounded only by the
  angular deflection (tight, unlike the variable fillet's `O(r')` gap).
- **Roadmap**: continues `ROADMAP.md` #6 (curved blends) and the `SSI-ROADMAP.md`
  `S1 circle seams → #6 blends` on-ramp. Asymmetric / non-circular / concave / curved↔
  curved / freeform chamfers remain future #6 work.
- **Regression**: additive only. The PLANAR chamfer (`run-sim-native-blend.sh` 16/16),
  the constant + variable curved fillets (`run-sim-native-curved-fillet.sh` 23/23),
  native booleans, SSI S1–S5, healing, import, marching, and phase-3 suites are
  untouched; the curved-chamfer path only fires for the named symmetric circular input,
  is TRIED ONLY AFTER the planar chamfer returns NULL, and is gated by the SHRINK
  self-verify.
- **Risk / honesty**: honest scope — the two setback seams are exact circles (on the
  cylinder / cap by construction) and the cone frustum is the EXACT symmetric chamfer
  surface (a ruled truncated cone, not an approximation), so the only error is the
  angular tessellation deflection. The bevel is asserted C0 at the chamfer angle, NEVER
  G1 (asserting tangency would be geometrically wrong for a chamfer). Everything outside
  the symmetric convex circular slice returns NULL → OCCT with the measured gap REPORTED,
  never masked with a weakened tolerance.
