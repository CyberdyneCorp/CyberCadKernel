# Tasks — moat-m3sf-sphere-cap-fillet

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `detail::SphereInfo` + `detail::sphereInfo` (fold face location) to `curved_fillet.h`.
- [x] 1.2 Add `detail::sphereCapGeom` — wholesale truncated-ball recognizer about the rim axis
  (coaxial sphere of one centre/R + axis-normal plane at exactly ONE height, rim radius √(R²−h²)).
- [x] 1.3 Add `detail::buildFilletedSphere` — torus-band fillet (sphere-wall seam → cap seam)
  rebuilt as a planar-facet soup sharing N angular samples; faceted sphere wall below the seam;
  ring-torus + seam-below-cap guards.
- [x] 1.4 Add public `blend::sphere_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`.

## 2. Engine wiring
- [x] 2.1 Add `nblend::sphere_fillet_edge` as candidate #5 in `NativeEngine::fillet_edges`,
  gated by the existing `blendResultVerified` SHRINK self-verify (0 < Vr < Vo).

## 3. Gate A — host (analytic, no OCCT)
- [x] 3.1 `sphere_fillet_truncated_ball_watertight_volume_reduced` — watertight + closed-form.
- [x] 3.2 `sphere_fillet_converges_with_deflection` — volume tightens as deflection refines.
- [x] 3.3 `sphere_fillet_g1_tangent_at_both_seams` — analytic seam-normal match.
- [x] 3.4 `sphere_fillet_scope_defers` — cylinder / cone / spindle / box / multi-edge / r≤0.
- [x] 3.5 Full host `ctest` green.

## 4. Gate B — sim (native-vs-OCCT parity)
- [x] 4.1 Add `runSphereCase` to `native_curved_fillet_parity.mm`, vs OCCT
  `BRepFilletAPI_MakeFillet` + `BRepGProp` + closed form.
- [x] 4.2 Run on the booted simulator; sphere case PASSes.

## 5. Validate
- [x] 5.1 `openspec validate moat-m3sf-sphere-cap-fillet --strict`.
- [x] 5.2 Confirm `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
