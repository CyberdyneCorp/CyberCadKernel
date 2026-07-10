# Proposal — moat-gltf-export

## Why

The kernel already serialises meshes to STL, but the CyberCad iPad app's highest-
value runtime handoff — **View in AR / QuickLook / share / web-render** — needs the
two formats the Apple and web ecosystems actually consume:

- **glTF 2.0** (`.gltf` + `.glb`) — the Khronos runtime asset format that RealityKit,
  SceneKit, three.js, and every modern web/render pipeline load directly.
- **USDZ** — Apple's AR interchange package that QuickLook / RealityKit open for
  "View in AR" from Files, Messages, Safari, and the share sheet.

Neither exists at the facade today. This is **anticipatory** app value (not moat /
drop-OCCT completion): both are pure **mesh serialisation** of the triangle mesh the
kernel already produces, so neither needs OCCT and neither is an oracle-backed
capability. The facade calls the existing engine-neutral tessellation path
(`IEngine::tessellate` → `MeshData`) and hands the flat vertex / index POD to new
native writers — no new meshing, no OCCT type, no new export virtual, the SAME
neutral mesh the STL path uses.

glTF/USDZ are NEW, independent native exchange paths. They do not modify or weaken
any existing exchange requirement (STEP/IGES/STL are untouched), and they do not
touch the tessellator (they CONSUME its watertight indexed mesh).

## What Changes

1. **A native glTF 2.0 writer** (`src/native/exchange/gltf_writer.{h,cpp}`,
   namespace `cybercad::native::exchange`, OCCT-free, host-buildable). Mesh → glTF:
   one buffer (indices `uint32` / POSITION `vec3<f32>` / NORMAL `vec3<f32>`, each
   block 4-byte aligned), three accessors (indices SCALAR/5125, POSITION VEC3/5126
   with correct `min`/`max`, NORMAL VEC3/5126), two bufferViews (34963 / 34962), one
   mesh / one triangle primitive, one node, one scene, one default metallic-roughness
   PBR material. Smooth per-vertex normals are derived from the mesh geometry
   (area-weighted face-normal accumulation) — the tessellator carries no normals and
   is NOT modified. Emits BOTH `.gltf` (JSON + base64 `data:` URI buffer, fully
   self-contained) and `.glb` (binary container: 12-byte header + JSON chunk + BIN
   chunk, all 4-byte aligned). Positions are emitted in metres (kernel mm × 1e-3, the
   glTF linear unit). Deterministic.

2. **A native USDZ writer** (`src/native/exchange/usdz_writer.{h,cpp}`, OCCT-free).
   Mesh → a single ASCII-USD (`.usda`) `UsdGeomMesh` layer (points /
   faceVertexIndices / faceVertexCounts / displayColor, metersPerUnit=1, Y-up) packed
   into a USDZ container: a STORE-only (uncompressed) ZIP whose single entry's DATA is
   aligned to a 64-byte boundary per the USDZ spec (via a local-header extra-field
   pad), with a correct CRC-32 and central directory. QuickLook reads the ASCII layer;
   the USD **binary crate** encoder is a documented follow-up, NOT required for the AR
   handoff (ASCII layers are conformant inside a `.usdz`). A bare `.usda` export is
   also exposed for inspection.

3. **Two ADDITIVE `cc_*` ops** in `include/cybercadkernel/cc_kernel.h`, matching the
   `cc_stl_export` signature idiom (`CCShapeId` + path → `int` status):
   - `int cc_gltf_export(CCShapeId body, const char *path, double deflection, int glb)`
   - `int cc_usdz_export(CCShapeId body, const char *path, double deflection)`
   Wired through the facade (`src/facade/cc_kernel.cpp`) over the existing
   `IEngine::tessellate` path. No existing signature changes.

## Impact

- **Additive only.** No `cc_*` signature change; no engine virtual added; the
  tessellator is untouched (consumed read-only). `src/native/**` stays OCCT-free.
- **New capability** `native-exchange` gains glTF + USDZ export requirements.
- **Two-gate host-verified** (`test_native_gltf`): round-trip (native mesh →
  glTF/USDZ → re-parse → same vert/tri/bbox) + structural validity (glTF-2.0
  accessor/bufferView/alignment/min-max, `.glb` chunk layout, `.usdz` zip container).
- **Honest scope:** USDZ ships the ASCII-USD-in-zip path (QuickLook-conformant). The
  USD binary crate encoder is deferred as a follow-up — not a limitation of the AR
  handoff, and no third-party USD/glTF library is vendored.
