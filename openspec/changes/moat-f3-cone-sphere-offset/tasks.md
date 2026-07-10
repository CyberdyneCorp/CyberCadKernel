# Tasks — moat-f3-cone-sphere-offset

## 1. Native builder (OCCT-free, additive to `curved_offset.h`)
- [x] 1.1 `detail::cappedConeGeom` — wholesale capped-frustum recognizer about a picked Cone
  lateral face (coaxial Cone same σ/Rref + exactly two axis-normal caps, both cap radii > 0).
- [x] 1.2 `detail::buildCappedCone` — rebuild at `Rref + distance/cosσ` (same σ, same cap
  heights) as a planar-facet soup (wall band + two disc caps, shared N angular samples).
- [x] 1.3 `detail::sphereDomeGeom` — wholesale sphere-cap-dome recognizer about a picked
  Sphere face (coaxial spheres one centre/radius + EXACTLY ONE distinct axis-normal cap that
  cuts the ball; a full revolve fragments wall + disc into sectors, matched by geometry).
- [x] 1.4 `detail::buildSphereDome` — rebuild the concentric dome at `R+distance` (same cap
  plane) as a planar-facet soup (sphere-wall latitude bands pole→cap + one disc cap).
- [x] 1.5 Extend the public `blend::curved_offset_face` entry: try cylinder, then cone, then
  sphere arm; each declines (NULL) out of envelope.

## 2. Engine wiring
- [x] 2.1 UNCHANGED — `NativeEngine::offset_face` already calls `nblend::curved_offset_face`
  as candidate #2, gated by the correctly-signed `blendResultVerified` self-verify.

## 3. Gate A — host (no OCCT, closed-form)
- [x] 3.1 `curved_offset_cone_wall_grows` — narrowing frustum, watertight + oriented +
  `πH/3·(Rb'²+Rb'Rt'+Rt'²)` with Rb'/Rt' shifted by d/cosσ + grew.
- [x] 3.2 `curved_offset_cone_wall_shrinks` — widening frustum, watertight + shrank + closed
  form.
- [x] 3.3 `curved_offset_sphere_wall_grows` — hemisphere, watertight + oriented +
  `seg(R+d,a)` + grew.
- [x] 3.4 `curved_offset_sphere_wall_shrinks` — shallow cap + deep dome, watertight + shrank +
  closed form (converges as builder deflection refines: 1.8e-2→1.1e-3 measured).
- [x] 3.5 `curved_offset_cone_sphere_scope_defers` — planar cap / cone-cap-invert / R+d=0
  decline; curved-wall controls succeed.
- [x] 3.6 Full host `ctest` green (67/67).

## 4. Gate B — sim (native-vs-OCCT parity)
- [x] 4.1 Extend `tests/sim/native_curved_offset_parity.mm`: replace the cone honest-decline
  with `runConeCase` (native `cc_offset_face` on a `cc_solid_revolve` frustum wall vs the
  direct OCCT oracle `BRepPrimAPI_MakeCone(Rb+d/cosσ, Rt+d/cosσ, H)` + `BRepGProp` + the exact
  frustum closed form) and add `runSphereCase` (native on a `cc_solid_revolve_profile`
  sphere-cap dome wall vs the spherical-segment closed form + `MakeSphere` cross-check).
- [x] 4.2 Run on the booted simulator; all cases PASS (volume relO/relX, area, watertight,
  Euler χ=2, direction). Cone rel≈2e-3; sphere deflection-bounded rel<2e-2 (F2-consistent).

## 5. Validate
- [x] 5.1 `openspec validate moat-f3-cone-sphere-offset --strict`.
- [x] 5.2 `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
- [x] 5.3 Update readiness `cc_offset_face` row: cone-frustum + sphere-cap-dome curved walls
  move OCCT→native; freeform / stepped / spherical-zone residual stays OCCT.
