# Tasks — moat-m3cf-cone-cap-fillet

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `detail::ConeInfo` + `detail::coneInfo` (fold face location) to `curved_fillet.h`.
- [x] 1.2 Add `detail::coneCapGeom` — wholesale capped-frustum recognizer about the rim axis
  (coaxial cone of one σ/Rref + axis-normal planes at exactly two heights, rim on a cap).
- [x] 1.3 Add `detail::buildFilletedCone` — torus-band fillet (tilted wall seam → cap seam)
  rebuilt as a planar-facet soup sharing N angular samples; ring-torus + seam-on-wall guards.
- [x] 1.4 Add public `blend::cone_fillet_edge(solid, edgeIds, edgeCount, r, deflection)`.

## 2. Engine wiring
- [x] 2.1 Add `nblend::cone_fillet_edge` as candidate #4 in `NativeEngine::fillet_edges`,
  gated by the existing `blendResultVerified` SHRINK self-verify (0 < Vr < Vo).

## 3. Gate A — host (analytic, no OCCT)
- [x] 3.1 `cone_fillet_narrowing_frustum_watertight_volume_reduced` — watertight + closed-form.
- [x] 3.2 `cone_fillet_widening_frustum_watertight_volume_reduced`.
- [x] 3.3 `cone_fillet_g1_tangent_at_both_seams` — analytic seam-normal match.
- [x] 3.4 `cone_fillet_scope_defers` — cylinder / spindle / box / multi-frustum / multi-edge / r≤0.
- [x] 3.5 Full host `ctest` green (64/64).

## 4. Gate B — sim (native-vs-OCCT parity)
- [x] 4.1 Add `runConeCase` to `native_curved_fillet_parity.mm` (narrowing + widening + steep),
  vs OCCT `BRepFilletAPI_MakeFillet` + `BRepGProp` + closed form.
- [x] 4.2 Add `TKHLR` to the sim runner link set (pre-existing missing toolkit).
- [x] 4.3 Run on the booted simulator; all cone cases PASS.

## 5. Validate
- [x] 5.1 `openspec validate moat-m3cf-cone-cap-fillet --strict`.
- [x] 5.2 Confirm `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
