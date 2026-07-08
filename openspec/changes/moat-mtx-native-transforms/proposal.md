# Proposal — moat-mtx-native-transforms (MOAT M-TX)

## Why

The **rigid/affine TRANSFORM layer** is the largest app-facing OCCT fallback that is
also cheap and bounded. Six placement ops the CyberCad app's translate / rotate /
mirror / scale / place tools call — `cc_translate_shape`, `cc_rotate_shape_about`,
`cc_mirror_shape`, `cc_scale_shape`, `cc_scale_shape_about`, `cc_place_on_frame` —
today **hard-error** (`CC_NATIVE_BODY_UNSUPPORTED`) when handed a NATIVE body: they
never even reach OCCT (a native void must never be forwarded), so a native body simply
cannot be moved. In addition the legacy mesh extrude `cc_extrude` (`extrude_mesh`)
forwards **unconditionally** to OCCT even though the native prism builder that backs
`cc_solid_extrude` already handles the same input.

The native machinery is already proven: `topology::Shape::located(math::Transform)` +
`SolidMesher` is differentially fuzzed against the OCCT oracle
(`BRepBuilderAPI_Transform(gp_Trsf)` + `BRepGProp`) AND a closed-form similarity image
in `tests/sim/native_transform_fuzz.mm` (translate/rotate/uniform-scale/mirror chains,
mirror handedness-flip confirmed). This change wires that proven path into the six
`cc_*` transform ops for a native body, and rewires `cc_extrude` to attempt the native
prism first — turning the M-TX bucket from a Class-B fallback into native coverage.

## What Changes

- **`extrude_mesh` / `cc_extrude` rewire.** Attempt the native prism (`build_prism`,
  the same builder `solid_extrude` uses) FIRST and mesh it at the OCCT adapter's legacy
  0.1 deflection; fall through to OCCT only on an honest decline (NULL solid). Zero
  behavior change on the cases the native builder already handles (bbox/extents
  identical); OCCT still catches the rest.
- **Native affine transforms for native bodies.** `translate_shape`,
  `rotate_shape_about`, `mirror_shape`, `scale_shape`, `scale_shape_about`,
  `place_on_frame` on a NATIVE body apply a `math::Transform` via
  `Shape::located(math::Transform)` (B-rep) or transform the mesh vertices/normals
  (imported STL body), returning a new native body. A MIRROR flips
  orientation/handedness (the signed enclosed-volume sign) while staying a valid,
  watertight, positive-|vol| solid. A NON-native (OCCT) body forwards to OCCT exactly
  as before; a native body is NEVER forwarded to OCCT.
- **Honest decline.** A zero / degenerate (non-invertible) scale, a zero rotation axis,
  a zero mirror normal, and a degenerate `place_on_frame` frame return a clean error
  (the native body is not forwarded to OCCT); a B-rep result that does not self-verify
  robustly watertight with positive |vol| declines rather than shipping a leaky solid.

## Scope / Non-goals

- `cc_*` ABI is unchanged (no new/changed signatures); `src/native/**` is untouched and
  stays OCCT-free. Only `src/engine/native/native_engine.cpp` changes engine-side.
- Uniform scale only (matching the `cc_scale_shape` contract). Non-native bodies keep
  their exact OCCT behavior.

## Verification (two-gate, OCCT is the oracle)

- **Gate (a) HOST-analytic** (`tests/test_native_engine.cpp`, OCCT-free): a natively
  built 10³ box is transformed by each op and measured against closed-form invariants
  — volume = |det L|·vol, exact bbox/centroid affine image, `mass_properties.valid`
  ⇒ watertight positive-|vol| solid, and every degenerate-input guard declines. Plus
  `cc_extrude` native bbox == profile extents and a degenerate profile falls through.
- **Gate (b) SIM parity** (`tests/sim/native_transform_parity.mm`): the `cc_*` transform
  facade is driven under BOTH engines on a booted simulator — box (planar, exact) and
  cylinder (curved, deflection-bounded) — and native volume/area/centroid/bbox are
  compared to the OCCT oracle AND the analytic |det L|·vol anchor; a zero scale is
  asserted an honest decline on both engines. The pre-existing
  `tests/sim/native_transform_fuzz.mm` continues to certify the underlying
  `located()`+`SolidMesher` machinery this ABI reuses.
