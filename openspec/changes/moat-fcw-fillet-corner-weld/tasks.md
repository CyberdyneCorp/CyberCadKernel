# Tasks — moat-fcw-fillet-corner-weld

## 1. Reproduce the blocker
- [x] 1.1 Confirm the sequential `fillet_edges` on a box top-face loop is not watertight
      (V ≈ 1010/1011 vs 1000; corner region open) — the curved↔curved seam the corner
      weld must close.

## 2. Spherical fillet-corner weld (`fillet_corner.h`, OCCT-free, assembly layer)
- [x] 2.1 Per-edge rolling-ball cylinder (reuse `detail::filletArc`); per-corner sphere
      centre via a 3-plane offset solve (`triPlaneOffset`, `Mat3::inverse`).
- [x] 2.2 ONE canonical `arcSample` (great-circle slerp) consumed bit-identically by the
      cylinder strip end arc AND the sphere patch leg (the shared-vertex weld identity).
- [x] 2.3 Trimmed face F (inset polygon), side faces set back + fan-triangulated with
      the tangent/corner points inserted (avoid the collinear ear-clip degeneracy),
      cylinder strips, sphere octant grid, flat corner ledge.
- [x] 2.4 Centroid-verified outward normals for carried faces (robust to a constructor
      that stores a base face reversed — e.g. `build_prism_profile_spline`).
- [x] 2.5 Perpendicular-wall scope guard; oversized-radius (overlapping corner spheres)
      guard; mandatory `isConsistentlyOriented` + two-sided SHRINK volume self-verify.

## 3. Wire `fillet_face`
- [x] 3.1 `fillet_face` tries `fillet_corner` first, falls back to `fillet_edges`, else
      declines `WeldGatesM2`. Engine `NativeEngine::fillet_face` unchanged.
- [x] 3.2 Add `fillet_corner.h` to `native_blend.h`.

## 4. Gate (a) — HOST analytic (OCCT-free)
- [x] 4.1 New `tests/native/test_native_fillet_corner.cpp`: box top / bottom / side, a
      non-rectangular prism cap land watertight + consistently oriented at the closed
      form (converging as deflection refines); declines battery.
- [x] 4.2 Update `tests/native/test_native_analytic_fillet.cpp`: fillet_face LANDS at
      the closed form + converges (was the measured M2 decline).
- [x] 4.3 Register `test_native_fillet_corner` in CMakeLists; host build + ctest green.

## 5. Gate (b) — SIM native-vs-OCCT parity (OCCT oracle)
- [x] 5.1 Update `tests/sim/native_analytic_fillet_parity.mm`: fillet_face LANDS vs the
      OCCT `BRepFilletAPI_MakeFillet` oracle (volume/area/watertight/χ=2/bbox + closed
      form); document the O(r) corner-convention Hausdorff gap.
- [x] 5.2 Run on the booted simulator: 6/6 PASS.

## 6. Structural discipline + docs
- [x] 6.1 `src/native/**` OCCT-free (comments only); `cc_*` ABI unchanged; tessellator
      UNTOUCHED (zero diff → byte-identical trivially satisfied).
- [x] 6.2 Update `openspec/MOAT-ROADMAP.md` (M3: fillet-corner weld landed → fillet_face
      full-face fillets native) + the `fillet_face` row in `DROP-OCCT-READINESS.md` → A.
- [x] 6.3 `openspec validate moat-fcw-fillet-corner-weld --strict`.
