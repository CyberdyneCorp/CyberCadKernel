# Tasks — add-native-stl-exchange (issues #4 export + #5 import, one PR)

Order: native STL writer → native STL reader → facade + engine wiring → tests +
docs. All new native code stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::exchange`. No `cc_*` ABI
change (additive entry points only). Keep per-function cognitive complexity green
(factor the little-endian putters, format detection, vertex weld, and the two
parsers into small helpers).

## 1. Native STL writer (`src/native/exchange/stl_writer.{h,cpp}`, issue #4)

- [x] 1.1 `stl_export_mesh(vertices, triangles, path, binary) -> bool`: flat POD
      in (x,y,z fp64 millimetres + i,j,k index triplets), STL out, `false` on I/O
      failure. Build one byte buffer, single buffered `std::ofstream(binary).write`.
- [x] 1.2 Little-endian primitives (`put_u16le` / `put_u32le` / `put_f32le`) and
      `to_f32(double)` normalizing `-0.0f` → `0.0f`, factored as small helpers.
- [x] 1.3 Binary layout `84 + 50·N`: 80-byte header (`"CyberCadKernel binary STL"`
      zero-padded, MUST NOT start with `"solid"`, no timestamp / host / build-id),
      `uint32 LE` facet count, per facet `float32[3]` normal + 3× `float32[3]`
      vertices + `uint16 LE` attr = 0, all LE, in mesh triangle + stored winding
      order (never reordered).
- [x] 1.4 ASCII layout: `solid CyberCadKernel` … `endsolid CyberCadKernel`, fixed
      indentation, `\n` endings (stream opened binary to defeat CRLF),
      locale-independent floats, emitted from the same `float32`-rounded values so
      ASCII and binary of one body agree; `-0` normalized to `0`, never a comma.
- [x] 1.5 Per-facet geometric normal = `normalize((v1-v0) × (v2-v0))`; a zero-area
      facet (`len < 1e-12`) emits `(0,0,0)` but STILL writes its 3 vertices —
      export never fails on a degenerate facet.
- [x] 1.6 Determinism: same `(vertices, triangles)` ⇒ byte-identical output for
      both binary and ASCII. Host-buildable, cognitive complexity 🟢.

## 2. Native STL reader (`src/native/exchange/stl_reader.{h,cpp}`, issue #5)

- [x] 2.1 `stl_read(path, err, weldTol) -> std::optional<ntess::Mesh>`: read whole
      file to bytes; on any failure set `err` and return `std::nullopt` (no partial
      output). Build the mesh entirely in locals.
- [x] 2.2 Auto-detect (first decisive check wins), factored into a `detect_format`
      helper: too-small → fail; `84 + 50·claimedN == size` (with an overflow guard
      on `claimedN`) → BINARY (definitive, even if the header starts with `"solid"`);
      a non-text byte in the first ≤512 bytes → BINARY; trimmed `"solid"` + `"facet"`
      → ASCII; else BINARY last resort.
- [x] 2.3 Binary parse: per facet read 12 floats (skip the recomputable normal),
      append 3 vertices, skip the `uint16` attr; bounds-check every 50-byte record
      against EOF — never index past the buffer.
- [x] 2.4 ASCII parse: tokenize on whitespace; each `vertex` reads 3 floats via a
      C-locale conversion; every 3 vertices forms one triangle.
- [x] 2.5 Weld / dedup coincident vertices on a `weldTol` quantization grid (hash
      map keyed by quantized coordinates); store the un-quantized fp64 vertex on a
      miss, reuse the index on a hit. Factor into a `weld_vertex` helper.
- [x] 2.6 Tolerate degenerate input: after welding, skip a triangle with a repeated
      index or near-zero area; do NOT fix winding or enforce manifold (soup stored
      as-is). Zero valid triangles after skipping → `err` + `std::nullopt`.
- [x] 2.7 Host-buildable, OCCT-free; cognitive complexity 🟢 (small `detect_format`
      / `weld_vertex` / `parse_binary` / `parse_ascii` helpers).

## 3. Facade + engine wiring (issues #4 + #5)

- [x] 3.1 `include/cybercadkernel/cc_kernel.h`: additive prototypes
      `int cc_stl_export(CCShapeId body, const char *path, double deflection, int binary)`
      and `CCShapeId cc_stl_import(const char *path)`, documented (binary=1 default
      / binary=0 ASCII; import auto-detects; returns per the ok/fail contract).
      Reuse `CCMesh` / `CCShapeId`; no existing signature changed.
- [x] 3.2 `src/facade/cc_kernel.cpp`: `cc_stl_export` guards, tessellates via the
      active engine (`resolve(body)` + `deflection`), then calls
      `stl_export_mesh(...)`; on tessellate or write failure sets `cc_last_error`
      and returns 0, else 1. Include the `native_exchange.h` umbrella.
- [x] 3.3 `src/facade/cc_kernel.cpp`: `cc_stl_import` guards, calls
      `active_engine()->stl_import(path)`, and returns `finish_shape(r)` — registers
      only a fully valid body, 0 + `cc_last_error` on failure, nothing leaked.
- [x] 3.4 `src/engine/IEngine.h`: one additive virtual
      `ShapeResult stl_import(const char* path)` defaulting to engine-unsupported.
      No `stl_export` virtual (export is facade-side). OCCT / stub inherit the default.
- [x] 3.5 `src/engine/native/native_engine.{h,cpp}`: implement `stl_import` — null
      path → error; `stl_read` failure → error carrying `err`; success →
      `wrapNativeMesh(std::move(*mesh))`. Declare the `override`.
- [x] 3.6 `src/engine/native/native_engine.cpp`: extend the `NativeShape` holder
      with `ntess::Mesh mesh` + `bool isMesh`, add the `wrapNativeMesh` factory, and
      add an `isMesh` branch at the top of `tessellate` / `mass_properties` /
      `bounding_box` / `face_meshes` / `subshape_ids` that serves `holder->mesh`
      directly through the existing mesh blocks (area / volume-if-closed / bbox /
      one face mesh / vertex+face ids). Do not change the existing B-rep paths.

## 4. Tests + docs

- [x] 4.1 Host CTest `tests/native/test_native_stl.cpp` (harness `CC_TEST` /
      `CC_CHECK`, facade-level): binary export round-trip (count field ==
      tessellated tri count, filesize == `84 + 50·count`, re-import triangle count
      + bbox match); ASCII export well-formed (`solid` / `facet normal` /
      `outer loop` / `vertex` ×3 / `endloop` / `endfacet` / `endsolid` balanced, no
      comma); determinism byte-identical (binary + ASCII); auto-detect ASCII vs
      binary import to equal triangle count + bbox; a binary header starting with
      `"solid "` still detected BINARY (regression); malformed / missing input → 0 +
      non-empty `cc_last_error`, no crash; import measurement (closed box → area > 0,
      volume ≈ expected, watertight; open mesh → area valid, volume flagged invalid);
      tolerate-and-recover (a mix of valid + degenerate facets with leading-`+`
      coordinates imports, skipping only the degenerate facet → the two valid survive).
- [x] 4.2 Register the suite in `CMakeLists.txt`: append `test_native_stl` to
      `CYBERCAD_TESTS` and add the `test_native_stl_SRC` mapping. Library sources are
      GLOB'd — no source-list edit for the new `.cpp`s.
- [x] 4.3 Additive Python bindings: `cc_stl_export` / `cc_stl_import` signatures in
      both `python/cybercadkernel/_cffi.py` and `_ffi.py`; `Shape.stl_export` +
      `Kernel.stl_import` in `api.py`; a native path in `viz.export_stl`
      (keyword-defaulted, trimesh default preserved).
- [x] 4.4 Additive `python/tests/test_viz.py` native round-trip case, gated on the
      real dylib (skips where absent).
- [x] 4.5 Verify: host rebuild + CTest green (`test_native_stl` passes; no new
      failure vs the pre-existing `test_native_boolean` baseline). Update docs /
      openspec (this change).
