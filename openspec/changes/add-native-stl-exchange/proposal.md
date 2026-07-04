# Proposal — add-native-stl-exchange

## Why

The kernel exchanges triangle meshes through two `cc_*` entry points that do not
yet exist: writing a tessellated body to `.stl`, and reading an `.stl` file back
as a body for display and measurement. STL is the lingua franca for 3D printing,
mesh preview, and interop with tools that never consume B-rep, so both directions
are needed at the facade — issue **#4** (export) and issue **#5** (import),
delivered together.

Both directions are cheap and exactly verifiable natively, so neither needs OCCT:

- **Export (#4)** is a pure **serialization** of the triangle mesh the kernel
  already produces. The facade calls the existing engine-neutral tessellation
  path (`IEngine::tessellate` → `MeshData`) and hands the flat vertex / index POD
  to a native writer. No new meshing, no OCCT type, no new export virtual — the
  same neutral `MeshData` serves both the OCCT and native engines. The writer is
  deterministic (same body + deflection ⇒ byte-identical file) and emits true
  millimetres with a per-facet geometric normal.

- **Import (#5)** is a **triangle-soup parser**, not B-rep reconstruction. It
  auto-detects ASCII vs binary STL, welds coincident vertices within tolerance,
  tolerates degenerate / zero-area facets, non-manifold edges, and inconsistent
  winding, and yields a welded `ntess::Mesh`. That mesh is carried directly by a
  **mesh-backed native body** (a triangle soup — no synthetic B-rep), which every
  native body-consuming op (`tessellate`, `mass_properties`, `bounding_box`,
  `face_meshes`, `subshape_ids`) already serves through the same mesh
  free-functions (`surfaceArea` / `enclosedVolume` / `isWatertight`). So display,
  bounding box, area, and volume-if-closed fall out with zero new geometry. B-rep
  reconstruction from a mesh is explicitly OUT OF SCOPE.

STL is a NEW, independent native exchange path. It does not modify or weaken the
existing "STEP import / IGES stay OCCT" requirement — those remain OCCT-backed and
untouched.

## What Changes

1. **A native STL writer + reader subtree** (`src/native/exchange/stl_writer.{h,cpp}`
   and `stl_reader.{h,cpp}`, namespace `cybercad::native::exchange`, OCCT-free,
   host-buildable, re-exported from the `native_exchange.h` umbrella).
   - Writer: serializes flat `(vertices, triangles)` POD to STL. Binary =
     80-byte deterministic header (never starting with `"solid"`, no timestamp /
     host / build-id) + `uint32` facet count + 50 bytes/facet (`float32` normal +
     3 `float32` vertices + `uint16` attr = 0), little-endian, one buffered write.
     ASCII = `solid` / `endsolid` framing with locale-independent floats emitted
     from the same `float32`-rounded values so ASCII and binary agree. Per-facet
     normal = `normalize((v1-v0) × (v2-v0))`, `(0,0,0)` for a zero-area facet.
   - Reader: auto-detects binary (size-identity `84 + 50·N`, then a non-text-byte
     sniff) vs ASCII, parses vertices, welds coincident vertices on a tolerance
     grid, skips degenerate triangles, and builds an `ntess::Mesh` entirely in
     locals. Any malformed input (short read, bad count, parse failure, zero valid
     triangles) sets an error string and returns no mesh — nothing partial.
2. **Two additive `cc_*` entry points** in `include/cybercadkernel/cc_kernel.h`:
   `cc_stl_export(body, path, deflection, binary) -> int` (1 ok / 0 fail) and
   `cc_stl_import(path) -> CCShapeId` (0 on failure). Additive only; no existing
   signature changes.
3. **Facade wiring** in `src/facade/cc_kernel.cpp`: `cc_stl_export` tessellates via
   the active engine then calls the native writer; `cc_stl_import` calls the engine
   and registers the resulting body (0 + `cc_last_error` on failure, nothing leaked).
4. **One additive engine virtual** `IEngine::stl_import(path)` (default = engine
   unsupported), implemented natively in `NativeEngine` to build a mesh-backed body
   via `wrapNativeMesh`. Export needs no virtual (it is facade-side). The native
   body holder gains a mesh + `isMesh` branch so the existing consuming ops serve
   the imported soup directly.
5. **Tests + docs**: a host CTest suite `tests/native/test_native_stl.cpp` and the
   additive Python ctypes / API / viz bindings.

## Capabilities

### Modified Capabilities
- `native-exchange`: adds native STL export (binary + ASCII) of a tessellated body
  and native STL import (ASCII / binary auto-detect) as a mesh body. This extends
  the capability with a new, independent mesh-exchange path; it does not alter the
  existing native STEP export or the "STEP import / IGES stay OCCT" requirements.

## Impact

- New OCCT-free, host-buildable files under `src/native/exchange/`
  (`stl_writer.{h,cpp}`, `stl_reader.{h,cpp}`), re-exported from
  `native_exchange.h`. Auto-globbed into the library — no CMake source edit.
- `include/cybercadkernel/cc_kernel.h` — two additive prototypes
  (`cc_stl_export`, `cc_stl_import`); reuses `CCMesh` / `CCShapeId`. No existing
  signature changes; ABI stays additive.
- `src/facade/cc_kernel.cpp` — two new entry-point bodies using the existing
  `guard` / `resolve` / `finish_shape` helpers.
- `src/engine/IEngine.h` — one additive virtual `stl_import` (default unsupported).
  `src/engine/native/native_engine.{h,cpp}` — native `stl_import`, a
  `wrapNativeMesh` factory, and an `isMesh` branch in the mesh-consuming ops so an
  imported soup is served directly. OCCT / stub engines inherit the unsupported
  default (STL import is a native-mesh capability).
- New host CTest `tests/native/test_native_stl.cpp` registered in `CMakeLists.txt`
  (`CYBERCAD_TESTS` + a `_SRC` mapping).
- Additive Python bindings (`_cffi.py`, `_ffi.py`, `api.py`, `viz.py`) + a gated
  `python/tests/test_viz.py` case (skips where the dylib is absent).
- Behaviour otherwise unchanged: existing engines and the OCCT default are
  untouched; only the two new entry points are added.
