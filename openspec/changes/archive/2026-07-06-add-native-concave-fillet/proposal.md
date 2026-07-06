## Why

Feature **#6 CURVED BLENDS** landed its FIRST curved slice in
`add-native-curved-fillet` (archived): a constant-radius rolling-ball fillet on a
CONVEX circular crease — the rim where a cylinder LATERAL face meets a coaxial PLANAR
cap. The rolling ball seats on the OUTSIDE (air) side of that convex dihedral, its
centre circle has radius `Rc − r`, the canal surface is a coaxial **TORUS** (major
`Rc − r`, minor `r`), and the fillet REMOVES material (`0 < Vr < Vo`). That slice
explicitly deferred the CONCAVE circular rim to OCCT (`design.md` Non-Goals:
"CONCAVE circular rim … the ball centre circle is `R + r` … deferred").

This change lands that deferred case — the everyday **inside fillet**. A CONCAVE
circular rim is where a cylinder (a boss) meets a LARGER planar face in a CONCAVE
dihedral: the base rim of a boss standing on a plate, or the bottom rim of a blind
hole. It is the most common real-world fillet (every filleted boss base, every
softened pocket/hole bottom). The rolling ball now seats on the MATERIAL side of the
dihedral, so:

- the ball-centre locus flips to the CIRCLE of radius `Rc + r` (convex was `Rc − r`)
  at axial offset `+r` INTO the material (convex was `−r`);
- the canal surface is STILL a coaxial **TORUS** — major `Rc + r`, minor `r` — but
  its quarter-tube now fills the concave corner instead of rounding a convex one;
- the two trim seams are STILL **CIRCLES**: torus∩cylinder at radius `Rc` (the `v=0`
  seam on the wall) and torus∩plane at radius `Rc + r` (the `v=π/2` seam on the larger
  plane) — the same closed-form SSI-S1 coaxial / axis-perpendicular families;
- the fillet ADDS material, so the solid's volume INCREASES: `Vr > Vo`, G1-tangent
  to both faces.

So the concave slice REUSES essentially the whole convex builder — the same
`math::Torus` frame, the same `sagittaSteps` facet budget, the same planar-facet weld
through the boolean `assembleSolid` — and differs only in three signs: the offset sign
(`+r` not `−r`), the material side the quarter-tube fills, and the resulting volume
change (grow not shrink). `cc_fillet_edges` ALREADY exists (planar + convex-curved
native paths + OCCT fallback); this change adds the concave-curved native path behind
the SAME ABI — no signature change — with OCCT as the fallback for the rest.

## What Changes

- Extend the native blend library (`src/native/blend/curved_fillet.h`) with a
  **concave circular-crease rolling-ball fillet** that runs NATIVELY when
  `cc_fillet_edges(body, {edge}, 1, r)` selects a CIRCULAR edge shared by exactly two
  faces — a `FaceSurface`-kind `Cylinder` lateral face and a `FaceSurface`-kind
  `Plane` face — meeting at a CONCAVE dihedral (the plane extends OUTSIDE the cylinder
  radius, i.e. the cylinder stands ON the plane rather than being capped by it).
- Construct the blend as a **coaxial torus canal-surface patch** on the material side:
  - The rolling ball of radius `r` seated tangent to the cylinder (radius `Rc`, axis
    `A`) from OUTSIDE the wall AND to the plane (at height `H`, normal ∥ `A`) from the
    material side has its centre on the CIRCLE coaxial with `A` at radius `Rc + r`,
    axial offset `+r` INTO the material (`H + r`). That centre-circle + the ball
    cross-section is the **torus**: major radius `R_t = Rc + r`, minor radius `r`,
    coaxial with the cylinder, tube-centre circle at `H + r`. (No ring-torus guard is
    needed — `R_t = Rc + r > r` always — but the SEAM-inside-face guards remain.)
  - The tangent seam on the CYLINDER is the **circle** torus∩cylinder at radius `Rc`,
    axial `H + r` (the torus inner equator meridian, `v = 0`).
  - The tangent seam on the PLANE is the **circle** torus∩plane at radius `Rc + r` in
    the plane `z = H` (the `v = π/2` ring).
  - Both seams lie on the torus BY CONSTRUCTION (evaluated on it) and are asserted G1
    (torus normal == cylinder radial at `v=0`, == plane axial at `v=π/2`).
- **Rebuild + weld** (reusing the convex builder's planar-facet discipline): rebuild
  the boss-on-plate (or blind-hole) region as ONE deflection-bounded planar-facet soup
  — the cylinder wall up to the `v=0` seam, the torus concave quarter-tube `v∈[0,π/2]`
  filling the corner, and the larger plane trimmed to an ANNULUS whose inner boundary
  is the `v=π/2` seam (radius `Rc + r`) — all sharing the SAME `N` angular samples so
  the seams weld with coincident vertices, closed through the boolean `assembleSolid`.
- **Self-verify sign per convex/concave in the engine** (`native_engine.cpp`): the
  existing `blendResultVerified` already supports a `wantGrow` branch (used by
  `offset_face` grow). A convex fillet is verified with `wantGrow=false`
  (`0 < Vr < Vo`); a CONCAVE fillet is verified with `wantGrow=true` (`Vr > Vo`). The
  dispatch tries the planar path, then the convex-curved path (verified shrink), then
  the concave-curved path (verified grow); the first candidate that passes its
  correctly-signed self-verify wins, else → OCCT. No tolerance is weakened.
- Native code stays **OCCT-free**. The concave torus patch uses `math::Torus`; the
  seam circles are the closed-form SSI-S1 coaxial / axis-perpendicular families
  (header-math, no NUMSCI substrate). No `cc_*` signature or POD struct changes.

**No `cc_*` ABI change.** `cc_fillet_edges` keeps its exact signature; the concave
circular rim is a new NATIVE path behind it, OCCT for the rest.

## Capabilities

### New Capabilities
<!-- none — this EXTENDS the existing native-blends capability with the concave
     circular-crease fillet; it does not introduce a new capability. -->

### Modified Capabilities
- `native-blends`: extend the native constant-radius `cc_fillet_edges` CURVED path
  from the CONVEX circular rim (cylinder↔coaxial cap, ball centre `Rc − r`, canal
  torus REMOVES material) to the CONCAVE circular rim (cylinder↔larger plane, ball
  centre `Rc + r`, canal torus ADDS material). The concave canal surface is a coaxial
  TORUS (major `Rc + r`, minor `r`) and its two tangent trim seams are CIRCLES
  (torus∩cylinder at `Rc` + torus∩plane at `Rc + r`, native SSI S1); the patch is
  inserted G1-tangent to both faces and welded watertight through the boolean
  `assembleSolid`. The engine self-verify picks the volume-change SIGN per case —
  convex `0 < Vr < Vo`, concave `Vr > Vo` — and DISCARDS a failing candidate → OCCT
  `BRepFilletAPI_MakeFillet`. Variable radius, cyl↔cyl canal fillets, non-circular
  curved creases, tilted planes, and freeform stay OCCT-fallthrough (never faked).
  Delivered behind the unchanged `cc_fillet_edges` signature via `engine-adapter`.

## Impact

- **ABI**: none. `cc_fillet_edges` keeps its signature and POD structs; the concave
  rim is an internal native path with OCCT fallback.
- **Build**: extends the OCCT-free header `src/native/blend/curved_fillet.h` (adds the
  concave classifier + builder next to the convex one) and adds one `wantGrow=true`
  branch to the `NativeEngine::fillet_edges` dispatch. Consumes `native-math`
  (`Torus`, `Cylinder`, `Plane`), `native-boolean` (`assembleSolid` weld),
  `native-tessellate` (watertight mesher — NOT modified). Builds with NUMSCI OFF (the
  seams are closed-form) and NUMSCI ON (regression).
- **Verification**: two gates. **host** (OCCT-free CTest): the concave torus patch +
  both seam circles lie on their surfaces within tol, the result meshes watertight,
  and its volume equals `|body| + V_fill` within the tessellation deflection bound,
  where `V_fill` is the ADDED concave rim-band volume (a closed-form Pappus figure).
  **sim** native-vs-OCCT parity (`run-sim-native-curved-fillet.sh` extended): the
  native concave fillet vs OCCT `BRepFilletAPI_MakeFillet` on a boss-on-plate base rim
  and a blind-hole bottom rim (volume / area / watertight / G1 at the two seams).
- **Roadmap**: continues `ROADMAP.md` #6 (curved blends) and the `SSI-ROADMAP.md`
  `S5 → #6 blends` on-ramp (S1 circle seams consumed by a curved blend). Variable /
  cyl∩cyl-canal / non-circular-curved / freeform blends remain future #6 work.
- **Regression**: additive only. The CONVEX circular fillet
  (`run-sim-native-curved-fillet.sh` 9/9), the planar blends
  (`run-sim-native-blend.sh` 16/16), native booleans, SSI S1–S5, healing, import,
  marching, and phase-3 suites are untouched; the concave-rim path only fires for the
  named concave input and is gated by the correctly-signed self-verify.
- **Risk / honesty**: honest scope — the concave torus canal surface and its two
  circle seams are exact closed form; every seam lies on the torus by construction and
  is asserted G1, and the assembled solid is gated by the engine's watertight +
  volume-GROWING guard. Anything unsafe or out-of-slice returns NULL → OCCT, so the
  slice can never emit a wrong or leaky concave blend. The measured OCCT-fallback gap
  is REPORTED, never masked with a weakened tolerance.
