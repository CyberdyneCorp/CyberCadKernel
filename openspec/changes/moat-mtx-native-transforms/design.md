# Design — moat-mtx-native-transforms

## The native transform is a placement

A native B-rep body is a `topology::Shape` (a shared geometry node + orientation +
`Location`). `Shape::located(Location{math::Transform})` composes an affine onto the
shape's `Location`; the tessellator (`surface_eval.h`, `edge_mesher.h`) world-places
every sample through that `Location`, and `Explorer` composes the location down the
graph. Meshing the located solid therefore yields the correctly world-placed solid.
This is exactly the path `tests/sim/native_transform_fuzz.mm` differentially fuzzes vs
OCCT `BRepBuilderAPI_Transform(gp_Trsf)` + `BRepGProp` and a closed-form similarity
image, so the ABI ops are a **thin composition** onto proven machinery.

`math::Transform` is an affine `v' = L·v + t` (a 3×3 linear part + translation,
mirroring `gp_Trsf`). It composes and inverts in closed form; `determinant() < 0`
signals a mirror (handedness flip).

## Per-op transform construction (matches the OCCT adapter semantics)

- **translate**: `Transform::translationOf({tx,ty,tz})`.
- **rotate_shape_about**: `Transform::rotationOf(center, Dir3(axis), angle)` — `Dir3`
  normalizes the axis (guard: zero axis → error). Rodrigues linear part, det = +1.
- **scale / scale_about**: `Transform::scaleOf(center, f)`, uniform. Guard: `f > 0`
  (matches the `cc_scale_shape` contract and avoids the OCCT zero-scale hang the fuzzer
  gated).
- **mirror**: reflection across the plane through `p` with UNIT normal `u` —
  `L = I − 2 u uᵀ` (det = −1), `t = 2(p·u)u` (guard: zero normal → error). Matches
  `gp_Trsf::SetMirror`.
- **place_on_frame**: rigid motion relocating the global XOY frame onto the destination
  frame `(origin, x-dir = u, main/z-dir = n = u×v, y-dir = n×x)`. `L`'s columns are the
  destination axes (orthonormal, det = +1); `t = origin`. Matches OCCT
  `gp_Ax3(o, dir(n), dir(u))` + `SetDisplacement`. Guard: `|u|`, `|v|`, or `|u×v|` ~ 0
  → error.

## Honesty gate and mirror handedness

`applyNativeTransform(holder, xf)`:
1. If `xf.inverse()` is nullopt (singular / zero-scale) → decline (`nullptr`).
2. Mesh body (imported STL): transform vertices via `applyToPoint`, normals via
   `applyToDir`; return a new mesh body. No winding reversal — consistent with the
   B-rep `located()` convention (a mirror flips the signed-volume sign).
3. B-rep body: `Shape::located(Location{xf})`; self-verify `robustlyWatertight` with
   positive `watertightVolume` before keeping (a similarity/rigid/mirror placement of a
   watertight solid stays watertight, so this holds for any legitimate body; a surprise
   declines honestly rather than shipping a leaky solid).

A **mirror** (det < 0) is kept: the placement flips the mesh's signed enclosed-volume
sign yet leaves a valid watertight positive-|vol| solid — exactly the convention
`native_transform_fuzz.mm` certifies (handedness = base sign × (−1)^#mirrors). The ABI
reports `|volume|`, so `mass_properties.valid` on the mirrored body confirms the
watertight positive-|vol| invariant at gate (a); the sign flip itself is asserted
parity-side by the fuzzer.

Each op: if the body is not native → forward to the fallback (OCCT) unchanged; if
native → apply and return the wrapped result, or a clean `make_error` on decline
(**never** forward a native void to OCCT).

## `extrude_mesh` rewire

`extrude_mesh` builds the native prism `build_prism(profileXY, count, depth)` (the same
builder `solid_extrude` uses; the OCCT adapter likewise builds a prism then
tessellates). On a non-null solid it meshes at deflection 0.1 (the OCCT adapter's
legacy value) → `toMeshData`. A NULL solid (degenerate / deferred) forwards the SAME
args to the fallback. Cases the native builder handles produce bbox/extents-identical
meshes; OCCT still catches the rest.

## Discipline

`src/native/**` untouched and OCCT-free; the tessellator is untouched; `cc_*` ABI
additive-only (no signature change). Only `src/engine/native/native_engine.cpp` gains
the helper + the six op bodies + the extrude rewire.
