# Tasks — moat-mtx-native-transforms

## 1. Engine: native affine transforms

- [x] 1.1 Add `applyNativeTransform(holder, math::Transform)` helper in
  `native_engine.cpp` (B-rep via `Shape::located`; mesh via vertex/normal transform;
  singular-linear decline; B-rep self-verify robustly watertight + positive |vol|).
- [x] 1.2 `translate_shape`: native path (else forward to OCCT).
- [x] 1.3 `rotate_shape_about`: native path + zero-axis guard.
- [x] 1.4 `mirror_shape`: reflection `I − 2 u uᵀ`, `t = 2(p·u)u` + zero-normal guard.
- [x] 1.5 `scale_shape` / `scale_shape_about`: uniform, `f > 0` guard.
- [x] 1.6 `place_on_frame`: rigid frame motion (columns = destination axes) + degenerate-frame guard.
- [x] 1.7 A native body is NEVER forwarded to OCCT; an OCCT body forwards unchanged.

## 2. Engine: extrude_mesh rewire

- [x] 2.1 `extrude_mesh` attempts `build_prism` first, meshes at 0.1, else falls through to OCCT.

## 3. Gate (a) HOST-analytic (`tests/test_native_engine.cpp`, OCCT-free)

- [x] 3.1 translate/rotate/mirror/scale/scale_about/place_on_frame on a native 10³ box:
  volume = |det L|·vol, exact bbox/centroid, `mass_properties.valid` (watertight +|vol|).
- [x] 3.2 Every degenerate-input guard declines (zero scale, zero axis, zero normal, u∥v).
- [x] 3.3 `cc_extrude` native bbox == profile extents; degenerate profile falls through.

## 4. Gate (b) SIM parity (`tests/sim/native_transform_parity.mm` + run script)

- [x] 4.1 Drive the six `cc_*` transform ops under BOTH engines (box + cylinder);
  compare native vs OCCT volume/area/centroid/bbox + the analytic |det L|·vol anchor.
- [x] 4.2 Zero scale is an honest decline on both engines.
- [x] 4.3 Pre-existing `native_transform_fuzz.mm` still PASSES (located()+SolidMesher machinery).

## 5. Discipline + docs

- [x] 5.1 `git diff src/native` empty (OCCT-free, tessellator untouched); `cc_*` ABI unchanged.
- [x] 5.2 Host build + `ctest` green; OpenSpec validated `--strict`.
- [x] 5.3 Update `openspec/DROP-OCCT-READINESS.md` (M-TX row) + `openspec/MOAT-ROADMAP.md`.
