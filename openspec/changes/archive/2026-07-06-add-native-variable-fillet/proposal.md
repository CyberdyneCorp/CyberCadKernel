## Why

Feature **#6 CURVED BLENDS** landed a CONSTANT-radius rolling-ball fillet on a CONVEX
circular crease in `add-native-curved-fillet` (archived) — the rim where a cylinder
LATERAL face meets a coaxial PLANAR cap. There the ball radius `r` is FIXED, so the
ball centre traces a CIRCLE at radius `Rc − r`, axial `H − r`; the canal surface is a
coaxial **TORUS** (major `Rc − r`, minor `r`); the two trim seams are CIRCLES
(torus∩cylinder at `Rc`, torus∩plane at `Rc − r`); and the fillet REMOVES material
(`0 < Vr < Vo`). Both that slice and its concave sibling explicitly deferred a VARIABLE
radius to OCCT (Non-Goals: "Variable-radius fillet … that is `fillet_edges_variable`,
already OCCT").

This change lands the FIRST variable-radius slice. `cc_fillet_edges_variable(body,
edgeIds, count, r1, r2)` currently forwards EVERY input to OCCT
`BRepFilletAPI_MakeFillet` (evolved-radius law). We add a native slice on the SAME
convex circular cylinder↔cap rim, with the radius varying LINEARLY around the rim:

- the ball radius is `r(θ) = r1 + (r2 − r1)·θ/2π` for `θ ∈ [0, 2π)`, the rim angle
  measured from a fixed seam start (`r = r1` at `θ = 0`, `r = r2` at `θ → 2π`);
- so the ball-centre locus is no longer a circle at fixed offset but a **SWEPT curve**:
  radius `Rc − r(θ)`, axial `H − s·r(θ)` (`s` = axial sign toward the cap);
- the blend surface is a **variable-radius canal** (a swept-circle / general blend
  patch): at each angular STATION `θ` a meridian circular arc of the LOCAL radius
  `r(θ)`, tangent to the cylinder at the top and to the cap at the bottom, swept and
  facet-welded to its neighbours;
- the two trim seams are NON-circular: the cylinder seam is at radius `Rc` with VARYING
  axial height `H − s·r(θ)`; the cap seam is at VARYING radius `Rc − r(θ)` in the plane
  `z = H`;
- the fillet REMOVES material, so the solid's volume DECREASES (`0 < Vr < Vo`),
  G1-tangent to both faces along BOTH varying-radius seams.

The variable slice REUSES the constant convex builder's whole discipline — the same
`math::Ax3` cylinder frame, the same `sagittaSteps` facet budget, the same meridian-arc
cross-section, the same planar-triangle weld through the boolean `assembleSolid` — and
differs only in that `r` is read PER STATION as `r(θ)` instead of a constant. The key
correctness result is that the swept patch is G1 at both seams for ANY linear (indeed
any differentiable) radius law: at the cylinder seam (`v = 0`) `∂radius/∂v = 0` so the
station tangent is axial and the surface normal is RADIAL (== the cylinder normal); at
the cap seam (`v = π/2`) `∂axial/∂v = 0` so the station tangent is radial-in-plane and
the surface normal is AXIAL (== the cap normal) — independent of `r'(θ)`.

## What Changes

- Extend the native blend library (`src/native/blend/curved_fillet.h`) with a
  **variable-radius CONVEX circular-crease fillet** `variable_fillet_edge(...)` that
  runs NATIVELY when `cc_fillet_edges_variable(body, {edge}, 1, r1, r2)` selects a
  CIRCULAR edge shared by exactly two faces — a `FaceSurface`-kind `Cylinder` lateral
  face and a `FaceSurface`-kind coaxial `Plane` CAP — meeting at a CONVEX dihedral
  (the SAME classifier the constant convex builder uses, `facesOnRim` + `rimGeom`).
- Construct the blend as a **swept variable-radius canal patch** on the air side:
  - `r(θ) = r1 + (r2 − r1)·θ/2π`; at each of `N` angular stations the station centre is
    at radius `Rc − r(θ)`, axial `H − s·r(θ)`, and the meridian arc `v ∈ [0, π/2]` has
    `radius(θ,v) = (Rc − r(θ)) + r(θ)·cos v`, `axial(θ,v) = (H − s·r(θ)) + s·r(θ)·sin v`.
  - `v = 0` → radius `Rc`, axial `H − s·r(θ)`, normal RADIAL → the CYLINDER seam
    (non-circular: a curve on the wall at varying height).
  - `v = π/2` → radius `Rc − r(θ)`, axial `H`, normal AXIAL → the CAP seam
    (non-circular: a curve at varying radius in the cap plane).
  - Both seams lie on their neighbour surfaces BY CONSTRUCTION (`radius = Rc` exactly at
    the wall seam; `axial = H` exactly at the cap seam) and are asserted G1 analytically
    (`cos = 1`) at EVERY station.
- **Rebuild + weld** (reusing the constant builder's planar-facet discipline): rebuild
  the capped-cylinder region as ONE deflection-bounded planar-triangle soup — the far
  cap; the cylinder wall up to the (non-circular) `v = 0` seam; the variable canal
  `θ × v ∈ [0,2π) × [0,π/2]` tiled into `N·M` planar triangles, each station using its
  LOCAL `r(θ)`; and the TRIMMED cap trimmed to the (non-circular) `v = π/2` seam — all
  sharing the SAME `N` angular samples so the seams weld with coincident vertices,
  closed through the boolean `assembleSolid`.
- **Wire the native path behind `cc_fillet_edges_variable`** in `native_engine.cpp`:
  `NativeEngine::fillet_edges_variable` calls `nblend::variable_fillet_edge(...)` and
  accepts it ONLY through the mandatory `blendResultVerified(result, body,
  wantGrow=false)` guard (a convex variable fillet REDUCES volume) — mirroring how
  `fillet_edges` dispatches its convex candidate. A NULL builder result OR a failed
  self-verify DISCARDS the candidate and the engine reports honestly (the OCCT engine
  serves the call). No new guard type, no weakened tolerance.
- Native code stays **OCCT-free**. The variable canal patch uses only `math::Ax3` +
  header math; the seams are closed-form per-station (no solver, no NUMSCI substrate).
  No `cc_*` signature or POD struct changes.

**No `cc_*` ABI change.** `cc_fillet_edges_variable` keeps its exact signature; the
linear-radius convex circular rim is a new NATIVE path behind it, OCCT for the rest.

## Capabilities

### New Capabilities
<!-- none — this EXTENDS the existing native-blends capability with the variable-radius
     circular-crease fillet; it does not introduce a new capability. -->

### Modified Capabilities
- `native-blends`: add a native `cc_fillet_edges_variable` CURVED path for the CONVEX
  circular rim (cylinder↔coaxial cap) with a LINEAR radius law `r(θ) = r1 + (r2−r1)·
  θ/2π`. The blend surface is a swept variable-radius canal (per-station meridian arc of
  the local `r(θ)`) whose two trim seams are NON-circular varying-radius curves
  (cylinder seam at `Rc`, varying axial `H − s·r(θ)`; cap seam at varying radius
  `Rc − r(θ)`, `z = H`); the patch is inserted G1-tangent to both faces at every station
  (analytic `cos = 1`, holds for any `r'(θ)`) and welded watertight through the boolean
  `assembleSolid`. The engine self-verify accepts it only through the SHRINK branch
  (`0 < Vr < Vo`) and DISCARDS a failing candidate → OCCT `BRepFilletAPI_MakeFillet`
  (variable). Non-linear radius laws, cyl↔cyl-canal variable fillets, non-circular /
  concave / tilted variable creases, freeform faces, and gradients that leave the
  curved-parity tolerance stay OCCT-fallthrough (never faked). Delivered behind the
  unchanged `cc_fillet_edges_variable` signature via `engine-adapter`.

## Impact

- **ABI**: none. `cc_fillet_edges_variable` keeps its signature and POD structs; the
  linear-radius convex rim is an internal native path with OCCT fallback.
- **Build**: extends the OCCT-free header `src/native/blend/curved_fillet.h` (adds the
  variable builder next to the constant convex + concave ones, reusing their classifier
  + facet helpers) and rewires the `NativeEngine::fillet_edges_variable` body from a
  pure OCCT fall-through to a native-first dispatch with the `wantGrow=false` self-verify.
  Consumes `native-math` (`Ax3`, cylinder/plane frames), `native-boolean`
  (`assembleSolid` weld), `native-tessellate` (watertight mesher — NOT modified). Builds
  with NUMSCI OFF (the seams are closed-form) and NUMSCI ON (regression).
- **Verification**: two gates. **host** (OCCT-free CTest): the variable canal patch +
  both non-circular seams lie on their surfaces within tol, the result meshes
  watertight, its volume equals `|body| − V_removed` within the tessellation deflection
  bound (where `V_removed` is the closed-form swept removed rim-band — the meridian
  corner area `r(θ)²(1 − π/4)` integrated around the rim at the local seam radius), and
  the canal normal is G1-tangent to each neighbour at BOTH seams at every station
  (`cos = 1`). **sim** native-vs-OCCT parity (`run-sim-native-curved-fillet.sh`
  extended): the native variable fillet vs OCCT `BRepFilletAPI_MakeFillet` with the
  evolved-radius law (`SetRadius(r1)`@one rim vertex + `SetRadius(r2)`@the other) on a
  cylinder top rim (volume / area / watertight / G1 at the two varying-radius seams),
  over two `(r1, r2)` fixtures.
- **Roadmap**: continues `ROADMAP.md` #6 (curved blends) and the `SSI-ROADMAP.md`
  `S5 → #6 blends` on-ramp. Non-linear-law / cyl∩cyl-canal / non-circular / concave /
  freeform variable blends remain future #6 work.
- **Regression**: additive only. The CONSTANT-radius convex + concave circular fillets
  (`run-sim-native-curved-fillet.sh` 15/15), the planar blends
  (`run-sim-native-blend.sh` 16/16), native booleans, SSI S1–S5, healing, import,
  marching, and phase-3 suites are untouched; the variable-rim path only fires for the
  named linear-law convex input and is gated by the SHRINK self-verify.
- **Risk / honesty**: honest scope — the two non-circular seams are exact (on the
  cylinder / cap by construction) and G1 is analytic at every station for ANY linear
  law; the swept meridian-arc canal is a well-defined G1 blend of the correct local
  radius, and the assembled solid is gated by the engine's watertight + volume-SHRINKING
  guard. The interior of the swept patch is the meridian-arc canal rather than OCCT's
  exact evolved-law envelope (which tilts the characteristic circle by `r'(θ)`); the
  host swept-volume bound and the sim curved-parity tolerance gate the match, and any
  gradient / configuration that exceeds it returns NULL → OCCT with the measured gap
  REPORTED, never masked with a weakened tolerance.
