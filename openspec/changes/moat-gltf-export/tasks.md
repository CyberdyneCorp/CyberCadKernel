# Tasks — moat-gltf-export (glTF 2.0 + USDZ mesh export, one PR)

Order: native glTF writer → native USDZ writer → ABI + facade wiring → tests +
docs. All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`),
namespace `cybercad::native::exchange`. Additive `cc_*` entry points only (no
existing-signature change). The tessellator is CONSUMED, never modified. Keep
per-function cognitive complexity green (factor the LE putters, base64, JSON
assembly, mesh compaction, and the zip container into small helpers).

## 1. Native glTF 2.0 writer (`src/native/exchange/gltf_writer.{h,cpp}`)

- [x] 1.1 `gltf_export_mesh(vertices, triangles, path, glb, name) -> bool`: flat POD
      in (x,y,z fp64 mm + i,j,k index triplets), glTF out, `false` on I/O failure.
- [x] 1.2 Mesh compaction: skip triangles with an out-of-range index (matching STL),
      keep only referenced vertices, reindex 0..N. Positions scaled mm → m (×1e-3).
- [x] 1.3 Smooth per-vertex NORMAL from area-weighted face-normal accumulation
      (un-normalized cross accumulate → normalize; degenerate → fallback +Z). The
      tessellator is NOT touched — normals are derived here.
- [x] 1.4 Binary buffer: three 4-byte-aligned blocks [indices u32][POSITION vec3f]
      [NORMAL vec3f]; track POSITION min/max (metres) for the accessor bounds.
- [x] 1.5 JSON document: asset 2.0, scene/node/mesh (one triangle primitive, mode 4),
      accessors (0 SCALAR/5125, 1 VEC3/5126 +min/max, 2 VEC3/5126), bufferViews
      (34963 / 34962), one default metallic-roughness PBR material. Locale-free floats.
- [x] 1.6 `.gltf`: buffer 0 as a base64 `data:` URI (self-contained). Base64 helper
      (standard alphabet, `=` padding).
- [x] 1.7 `.glb`: 12-byte header (magic `glTF`, version 2, total length) + JSON chunk
      (`0x4E4F534A`, space-padded to 4) + BIN chunk (`0x004E4942`, zero-padded to 4).
- [x] 1.8 Determinism: same mesh + mode ⇒ byte-identical file. Cognitive 🟢.

## 2. Native USDZ writer (`src/native/exchange/usdz_writer.{h,cpp}`)

- [x] 2.1 `usda_export_mesh` / `usdz_export_mesh(vertices, triangles, path, name)`.
      Same mesh compaction as glTF (referenced verts, mm → m).
- [x] 2.2 USDA layer: `#usda 1.0` header (defaultPrim, metersPerUnit=1, upAxis "Y"),
      `def Mesh` with faceVertexCounts / faceVertexIndices / point3f[] points /
      subdivisionScheme none / constant displayColor. USD-identifier-sanitized name.
- [x] 2.3 CRC-32 (ZIP polynomial) helper.
- [x] 2.4 STORE-only ZIP: local file header (method 0, sizes equal, deterministic
      time/date), central directory, EOCD. One entry `<name>.usda` (the default layer).
- [x] 2.5 64-byte DATA alignment (USDZ spec): size the local-header extra field so the
      data start is a 64-byte multiple (extra record id `0x1986`, zero payload; bump by
      a full 64 when a nonzero pad < 4 so the 4-byte extra header fits).
- [x] 2.6 Determinism; honest-scope note that the binary crate is a follow-up (ASCII
      layer is QuickLook-conformant). Cognitive 🟢.

## 3. ABI + facade wiring

- [x] 3.1 Add `int cc_gltf_export(CCShapeId, const char*, double deflection, int glb)`
      and `int cc_usdz_export(CCShapeId, const char*, double deflection)` to
      `include/cybercadkernel/cc_kernel.h` (ADDITIVE, after the STL block, documented).
- [x] 3.2 Implement both in `src/facade/cc_kernel.cpp` over `IEngine::tessellate`
      (reuse the neutral mesh, no duplicated meshing), returning 1/0 with cc_last_error.
- [x] 3.3 Re-export the writers from `src/native/exchange/native_exchange.h`.

## 4. Tests + docs

- [x] 4.1 `tests/native/test_native_gltf.cpp` (facade-driven, native engine):
      GATE (a) round-trip (box 8v/12t, cylinder; re-parse glTF/GLB/USDZ → same
      vert/tri/bbox in metres) + GATE (b) structural (glTF-2.0 accessor/bufferView/
      alignment/min-max, `.glb` chunk layout, `.usdz` STORE-zip + 64-byte align + CRC).
- [x] 4.2 Register `test_native_gltf` in CMakeLists.txt; `ctest` green (host).
- [x] 4.3 Update `openspec/MOAT-ROADMAP.md` (new exchange — glTF/USDZ entry).
- [x] 4.4 `openspec validate --strict` clean.
