# Proposal — add-native-tet-meshing

## Why

The kernel has no volume mesher: it produces surface tessellations but cannot fill
a closed body with tetrahedra, which is the keystone deliverable for CalculiX++'s
`CadMesher` (GitHub issue **#1**). This change adds a tetrahedral volume mesher and
a native mesh-quality reporter, exposed additively on the `cc_*` ABI, **kernel-only**
— wiring CalculiX++'s `CadMesher` to these entry points is a deliberate follow-up
and is NOT part of this PR.

Robust constrained Delaunay tetrahedralization of an arbitrary closed surface is a
large, exactness-sensitive algorithm; re-deriving it is out of scope. The mesher is
therefore backed by **TetGen**, which is used as an **optional, external, AGPL-3.0**
dependency under strict license discipline that mirrors how the repo already treats
OCCT (LGPL) and the NumSci substrate:

- **TetGen is OFF by default.** The volume mesher lives in a single translation unit
  (`src/native/mesh/tet_mesher.cpp`) that is the SOLE consumer of the AGPL code. It is
  excluded from the default source glob and compiled only under the new
  `CYBERCAD_HAS_TETGEN` option (default OFF), exactly like `CYBERCAD_HAS_NUMSCI`.
- **The default MIT build links ZERO AGPL code.** With the flag OFF, `cc_tet_mesh`
  and `cc_tet_mesh_surface` return an empty `CCTetMesh` and set `cc_last_error` to a
  clear "tet meshing unavailable" message — they never crash and never link TetGen.
- **TetGen sources are never vendored.** They live externally at
  `/home/leonardo/work/tetgen` and are referenced by absolute path only; `tetgen.h` /
  `tetgen.cxx` / `predicates.cxx` are never copied into or committed to this repo, and
  the external build output (`/build-tet/`) is git-ignored.
- **Shipping obligation is stated honestly.** A closed-source application that turns
  the flag ON and links TetGen must obtain a TetGen commercial license; the AGPL
  reciprocity applies only to that opt-in configuration, not to the default kernel.

Mesh **quality reporting** is independent of TetGen: it is pure geometry over the
four corner nodes of each tetrahedron. It is therefore ALWAYS-ON, compiled in the
default MIT build, and its test runs with the flag OFF. This lets the quality gate
be applied to any tet mesh regardless of how it was produced.

## What Changes

1. **A native meshing subtree** (`src/native/mesh/`, namespace
   `cybercad::native::mesh`, OCCT-free, host-buildable). `quality.{h,cpp}` is
   always-on and TetGen-free; `tet_mesher.{h,cpp}` is the single AGPL-consuming pair,
   its `.cpp` excluded from the default glob. `tet_mesher.h` stays TetGen-header-free
   (declares only `std::vector`-based signatures) so no other TU can reach `tetgen.h`.
2. **Two additive facade entry points** in `include/cybercadkernel/cc_kernel.h`:
   `cc_tet_mesh(body, deflection, opts) -> CCTetMesh` (tessellate the body's surface,
   then fill the PLC) and `cc_tet_mesh_surface(verts, vCount, tris, tCount, opts) ->
   CCTetMesh` (fill a raw closed triangle surface, no OCCT), both funnelling through
   one shared `tet_mesh_from_surface` helper. Plus `cc_tet_mesh_free`.
3. **One additive always-on entry point** `cc_mesh_quality(mesh, min_scaled_jacobian)
   -> CCQualityReport` (pure geometry, TetGen-independent, POD by value — no engine,
   no `resolve`) plus `cc_quality_report_free`. Additive PODs `CCTetMesh`,
   `CCVolumeMeshOptions`, `CCQualityReport` mirror CalculiX++ `VolumeMeshOptions` /
   `QualityReport` field names so `map_to_model` is trivial in the follow-up.
4. **TetGen produces LINEAR tets only** (no `-o2`). Quadratic **C3D10** mid-edge nodes
   are constructed NATIVELY in CalculiX `shape10tet` order, deduplicated per shared
   edge so the mesh stays watertight. Every emitted tetrahedron is corner-ordered to
   have POSITIVE signed volume so the FE Jacobian is positive.
5. **Native quality metrics**, each from the 4 corners with a cited, defensible
   formula: signed volume; scaled Jacobian (Verdict / Knupp, regular tet = 1, sliver
   → 0, inverted < 0); six dihedral angles (Shewchuk, regular tet = 70.53°); aspect
   ratio (Verdict radius ratio, regular tet = 1). The report aggregates min/max
   dihedral, min/mean scaled Jacobian, max aspect ratio, and the ids of elements below
   `min_scaled_jacobian`.
6. **CMake gating** mirroring the NumSci pattern: a new `CYBERCAD_HAS_TETGEN` option
   (default OFF) plus `CYBERCAD_TETGEN_DIR` (external `libtetgen_*.a`) and
   `CYBERCAD_TETGEN_SRC_DIR` (external headers). The AGPL define / include dir are
   per-source scoped to `tet_mesher.cpp` only. A `scripts/build-tetgen.sh` builds the
   external static archive from `/home/leonardo/work/tetgen` (never copying sources),
   and `/build-tet/` is git-ignored.
7. **Tests + docs**: an always-on host CTest `tests/native/test_native_quality.cpp`
   (regular-tet baseline, sliver flagged, inverted detection, C3D10 mid-node
   consistency, empty/degenerate) and a gated `tests/native/test_native_tet.cpp`
   (cube → watertight C3D4/C3D10, positive Jacobian, volume conservation, quality
   gate), plus README / ROADMAP / STATUS notes stating the TetGen terms honestly.

## Capabilities

### Added Capabilities

- `native-meshing`: adds native tetrahedral **volume meshing** (C3D4 / C3D10 in
  CalculiX ordering, TetGen-backed, optional) and always-on native **mesh-quality
  metrics and reporting** (pure geometry, no OCCT, no TetGen), together with the
  **license gating** contract that keeps the AGPL TetGen backend optional, external,
  and OFF in the default MIT build.

## Impact

- New OCCT-free, host-buildable files under `src/native/mesh/`
  (`native_mesh.h`, `quality.{h,cpp}` always-on; `tet_mesher.{h,cpp}` the sole AGPL
  pair). `quality.cpp` and the umbrella are auto-globbed; `tet_mesher.cpp` is
  explicitly excluded from the default glob and added only under `CYBERCAD_HAS_TETGEN`.
- `include/cybercadkernel/cc_kernel.h` — additive PODs (`CCTetMesh`,
  `CCVolumeMeshOptions`, `CCQualityReport`) and prototypes (`cc_tet_mesh`,
  `cc_tet_mesh_surface`, `cc_tet_mesh_free`, `cc_mesh_quality`,
  `cc_quality_report_free`). No existing signature changes; the ABI stays additive
  and these types are NOT added to the mirrored `KernelBridgeAPI.h` static-assert set.
- `src/facade/cc_kernel.cpp` — new entry-point bodies using the existing
  `guard` / `resolve` helpers and a shared `tet_mesh_from_surface`. The two
  `cc_tet_mesh*` entries are `#ifdef CYBERCAD_HAS_TETGEN`-gated at the call site and
  return "unavailable" otherwise; `cc_mesh_quality` is never gated.
- `CMakeLists.txt` — new `CYBERCAD_HAS_TETGEN` option, `CYBERCAD_TETGEN_DIR` /
  `CYBERCAD_TETGEN_SRC_DIR` cache paths, a glob exclusion for `tet_mesher.cpp`, a
  gated block that compiles + links it with per-source AGPL scoping, and registration
  of `test_native_quality` (always-on) + `test_native_tet` (gated).
- `scripts/build-tetgen.sh` — builds the external `libtetgen_*.a` from
  `/home/leonardo/work/tetgen` by absolute path (predicates.cxx at `-O0` for FP
  robustness, tetgen.cxx at `-O2`, both `-DTETLIBRARY`); output `/build-tet/` is
  git-ignored. No TetGen source is ever copied or committed.
- New host CTests under `tests/native/`; README / ROADMAP / STATUS updated with the
  build recipe and the honest TetGen terms.
- **Out of scope (follow-up):** wiring CalculiX++'s `CadMesher` (import / heal /
  triangulate / tet_mesh / quality / map_to_model) to these entry points. This PR is
  kernel-only.
- Behaviour otherwise unchanged: the default MIT build gains only the always-on
  quality path and the "unavailable" volume-mesh stubs; no existing engine or the
  OCCT default is touched, and no AGPL code compiles or links unless the flag is ON.
