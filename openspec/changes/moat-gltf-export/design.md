# Design — moat-gltf-export

## Context

The app's AR / share / render handoff needs glTF 2.0 and USDZ, both of which are
**runtime mesh** formats, not B-rep. The kernel already produces the exact mesh they
need through the engine-neutral tessellation path (`IEngine::tessellate` →
`MeshData` = flat fp64 vertices + int index triples), the same mesh STL consumes.
So the entire capability is a **serialiser**, layered like the existing STL writer:
facade → `IEngine::tessellate` → native OCCT-free writer. No oracle, no fallback, no
new meshing, no tessellator change.

## Goals / non-goals

- **Goals:** valid glTF 2.0 (`.gltf` + `.glb`) and USDZ output from a native mesh;
  correct connectivity, bounds, and byte-level container structure; deterministic
  output; two-gate host verification; strictly additive ABI; OCCT-free.
- **Non-goals:** glTF/USDZ *import*; a USD *binary crate* encoder; materials/textures
  beyond a single default PBR / displayColor; animation, skins, cameras, lights;
  vendoring any third-party glTF/USD library.

## Decisions

### D1 — Consume the neutral mesh; never touch the tessellator
The facade calls `active_engine()->tessellate(resolve(body), deflection)` and hands
the resulting flat `(vertices, triangles)` POD to the writer — identical to
`cc_stl_export`. This reuses the watertight indexed mesh the tessellator already
produces and keeps `src/native/tessellate/**` byte-for-byte unchanged.

### D2 — Single mesh / single primitive
`MeshData` is one merged, welded, indexed mesh for the whole body. glTF is emitted as
one mesh with one triangle primitive (POSITION + NORMAL + indices). This is the
standard, minimal, universally-loadable shape; per-solid splitting would require the
tessellator to expose solid boundaries (it does not, at the neutral layer) and buys
nothing for the AR/preview use case.

### D3 — Derive smooth normals in the writer
`MeshData` carries no normals (STL derives per-facet geometric normals at write
time). glTF's NORMAL is per-vertex, so the writer accumulates each triangle's
un-normalized cross product (magnitude ∝ 2·area ⇒ area-weighted) into its three
vertices, then normalizes. A vertex touched only by degenerate faces gets a valid
fallback (+Z) so the accessor stays well-formed. This is pure derivation — the
tessellator is not modified.

### D4 — Millimetres → metres
The kernel works in mm; glTF's and USDZ's linear unit is the metre. Positions are
scaled ×1e-3 and USDA declares `metersPerUnit = 1`. This is the single semantic
transform; connectivity and (metre-space) bounds round-trip exactly under it, and the
gate asserts the metre bbox against `cc_bounding_box` × 1e-3.

### D5 — glTF buffer layout + alignment
One buffer, three contiguous 4-byte-aligned blocks in order
`[indices u32][POSITION vec3f][NORMAL vec3f]`. Each accessor's component type governs
its alignment (4 bytes for u32/f32), satisfied by construction. POSITION carries the
required `min`/`max` bounds. bufferViews use targets 34963 (ELEMENT_ARRAY) and 34962
(ARRAY). `.gltf` inlines the buffer as a base64 `data:` URI (self-contained, no
sidecar); `.glb` stores it as the implicit buffer-0 BIN chunk.

### D6 — GLB container
12-byte header (`glTF`, version 2, total length) + JSON chunk (type `0x4E4F534A`,
space-padded to 4 bytes) + BIN chunk (type `0x004E4942`, zero-padded to 4 bytes). The
`.glb` JSON's buffer 0 has NO `uri` (it is the BIN chunk).

### D7 — USDZ = ASCII USD in a STORE-zip, 64-byte data-aligned
The USDZ spec constrains the container: uncompressed (STORE method 0), no encryption,
one contiguous body per file, each file's DATA on a 64-byte boundary so USD's crate
reader can zero-copy `mmap`. We satisfy this by sizing the local file header's
**extra field** so `(headerOffset + 30 + nameLen + extraLen) % 64 == 0`; when the
required pad is nonzero but < 4 (too small for the 4-byte extra record header) we add
a full 64. The single entry is a `.usda` ASCII layer — the USDZ spec accepts ASCII
layers, so QuickLook / RealityKit open it directly without a binary crate encoder.

**Honest scope:** the USD *binary crate* (`.usdc`) encoder is deferred. It is a large,
intricate format (compressed integer/token/spec sections) whose value here is file
size, not capability — the ASCII layer is fully conformant for the AR handoff. This is
a documented follow-up, not a limitation of the exported package.

### D8 — Determinism
No timestamps / host / build-id anywhere (glTF generator string is fixed; the zip
mod-time/date are fixed 0). `-0.0f` is normalized to `+0.0f`. Same mesh + mode ⇒
byte-identical file, asserted in-test for all three outputs.

## Verification (two gates, OCCT is NOT the oracle)

- **GATE (a) round-trip (host):** a known 10 mm cube exports exactly 8 vertices / 12
  triangles and bounds `[0,0,0]..[0.01,0.01,0.01]` m; a cylinder round-trips its
  tessellated vert/tri counts. The test re-parses the glTF JSON, the decoded base64
  buffer, the GLB chunks, and the USDA-in-zip, reconstructs the POSITION bbox from the
  buffer, and asserts it equals `cc_bounding_box × 1e-3` to fp round-off.
- **GATE (b) structural (host):** the in-test assertions ARE the spec checks —
  glTF-2.0 accessor counts/types/componentTypes, bufferView offsets/lengths + 4-byte
  alignment, POSITION min/max; the `.glb` magic/version/length + JSON/BIN chunk types
  + 4-byte chunk alignment; the `.usdz` STORE method, 64-byte data alignment, CRC-32,
  and single-entry central directory / EOCD. (Python's `zipfile` and `json` also open
  the outputs during development as an independent cross-check.)
