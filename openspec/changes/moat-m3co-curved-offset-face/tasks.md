# Tasks — moat-m3co-curved-offset-face

## 1. Native builder (OCCT-free, additive)
- [x] 1.1 Add `src/native/blend/curved_offset.h` — `detail::cappedCylGeom` wholesale capped-
  cylinder recognizer about a picked Cylinder lateral face.
- [x] 1.2 `detail::buildCappedCyl` — rebuild at radius `Rc + distance` as a planar-facet soup
  (wall band + two disc caps, shared N angular samples).
- [x] 1.3 Public `blend::curved_offset_face(solid, faceId, distance, deflection)`; register in
  `native_blend.h`.

## 2. Engine wiring
- [x] 2.1 Add `nblend::curved_offset_face` as candidate #2 in `NativeEngine::offset_face`,
  gated by the existing `blendResultVerified` correctly-signed self-verify.

## 3. Gate A — host (no OCCT)
- [x] 3.1 `curved_offset_cylinder_wall_grows` — watertight + oriented + π(Rc+d)²H + grew.
- [x] 3.2 `curved_offset_cylinder_wall_shrinks` — watertight + oriented + π(Rc+d)²H + shrank.
- [x] 3.3 `curved_offset_scope_defers` — planar cap / Rc+d≤0 / zero / box decline.
- [x] 3.4 Full host `ctest` green (67/67).

## 4. Gate B — sim (native-vs-OCCT parity)
- [ ] 4.1 Add a curved-offset case to a sim parity harness (cyl wall radial offset) vs OCCT
  `BRepOffsetAPI` + `BRepGProp`.
- [ ] 4.2 Run on the booted simulator; case PASSes.

## 5. Validate
- [x] 5.1 `openspec validate moat-m3co-curved-offset-face --strict`.
- [x] 5.2 `src/native/**` OCCT-free, tessellator untouched, `cc_*` ABI additive-only.
- [ ] 5.3 Update readiness M3 offset_face row when the sim gate lands.
