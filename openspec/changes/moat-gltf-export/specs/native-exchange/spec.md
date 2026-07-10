# native-exchange

## ADDED Requirements

### Requirement: Native glTF 2.0 export (.gltf JSON + .glb binary) of a tessellated body

`cc_gltf_export(body, path, deflection, glb)` SHALL serialize the tessellated
triangle mesh of `body` to a glTF 2.0 asset at `path` and return `1` on success, `0`
on failure (with `cc_last_error` set). The facade SHALL obtain the mesh from the
already-neutral tessellation path (`IEngine::tessellate` → `MeshData`, honouring
`deflection`) and hand its flat `(vertices, triangles)` POD to a native writer —
reusing the existing meshing, adding no new export virtual, referencing no OCCT type
(the writer is OCCT-free and host-buildable) and NOT modifying the tessellator. The
asset SHALL be a single **mesh** with a single **triangle primitive** (`mode` 4)
whose attributes are `POSITION` and `NORMAL` accessors and whose `indices` accessor is
an `UNSIGNED_INT` (componentType 5125) `SCALAR` accessor, referencing one default
metallic-roughness PBR material. POSITION and NORMAL SHALL be `FLOAT` (componentType
5126) `VEC3` accessors; the POSITION accessor SHALL carry correct `min` and `max`
bounds. Positions SHALL be emitted in **metres** (kernel millimetres × 1e-3, the glTF
linear unit) with connectivity preserved index-for-index; triangles with an
out-of-range index SHALL be skipped and only referenced vertices SHALL be kept
(reindexed). Per-vertex normals SHALL be derived from the mesh geometry (area-weighted
face-normal accumulation, normalized) since the neutral mesh carries none. The single
buffer SHALL hold three blocks — indices (`uint32`), POSITION (`vec3<float32>`), NORMAL
(`vec3<float32>`) — each aligned to a 4-byte boundary, with two bufferViews (target
34963 for indices, 34962 for the vertex attributes) whose byteOffsets and byteLengths
match the blocks. When `glb` is zero the writer SHALL emit a self-contained **`.gltf`
JSON** whose buffer 0 is a base64 `data:` URI. When `glb` is non-zero the writer SHALL
emit a **`.glb`** binary container: a 12-byte header (magic `"glTF"`, version 2, total
length equal to the file size), a JSON chunk (type `0x4E4F534A`, padded to a 4-byte
boundary with spaces) and a BIN chunk (type `0x004E4942`, padded to a 4-byte boundary
with zeros) holding the buffer, and the JSON's buffer 0 SHALL have no `uri`. Floats
SHALL be locale-independent (never a decimal comma). A NULL / empty mesh (no valid
triangles) SHALL write an empty-but-valid asset and still return `1`. The writer SHALL
be **deterministic**: the same body + `deflection` + mode SHALL produce a
byte-identical file. glTF import is OUT OF SCOPE.

#### Scenario: A native cube exports a .gltf that round-trips to the same verts/tris/bbox (host)
- GIVEN a native-built 10 mm axis-aligned box body, built on the host with no OCCT
- WHEN `cc_gltf_export(body, path, deflection, 0)` writes a `.gltf` and the JSON + its base64 buffer are re-parsed
- THEN the call SHALL return `1` AND the asset SHALL declare version `"2.0"` AND the indices accessor SHALL be `SCALAR` componentType 5125 with count `= 3 × triangleCount` AND the POSITION accessor SHALL be `VEC3` componentType 5126 with count `= 8` (the box's 8 corners) AND the reconstructed POSITION bounding box SHALL equal `cc_bounding_box × 1e-3` (metres) to fp round-off

#### Scenario: The .glb binary container has a valid chunk layout (host)
- GIVEN a native-built body on the host with no OCCT
- WHEN `cc_gltf_export(body, path, deflection, 1)` writes a `.glb`
- THEN the call SHALL return `1` AND the 12-byte header SHALL be magic `"glTF"`, version `2`, total length equal to the file size AND the JSON chunk SHALL have type `0x4E4F534A` and a 4-byte-aligned length AND the BIN chunk SHALL have type `0x004E4942` and a 4-byte-aligned length AND the BIN chunk length SHALL equal the buffer `byteLength` in the JSON AND the JSON's buffer 0 SHALL have no `uri`

#### Scenario: glTF export is deterministic — byte-identical on repeat (host)
- GIVEN a native-built body and a fixed `deflection` on the host with no OCCT
- WHEN `cc_gltf_export` is invoked twice to two paths with the same `glb` mode
- THEN the two files SHALL be byte-identical (`memcmp` equal) for both the `.gltf` and the `.glb` mode

### Requirement: Native USDZ export (ASCII-USD layer, STORE-zip, 64-byte aligned) of a tessellated body

`cc_usdz_export(body, path, deflection)` SHALL serialize the tessellated triangle mesh
of `body` to a USDZ package at `path` (the Apple QuickLook / RealityKit AR handoff) and
return `1` on success, `0` on failure (with `cc_last_error` set). The facade SHALL
obtain the mesh from the same neutral tessellation path and hand its flat POD to a
native OCCT-free writer that does NOT modify the tessellator. The package SHALL contain
a single ASCII-USD (`.usda`) `UsdGeomMesh` layer describing the mesh as
`faceVertexCounts` (one `3` per triangle), `faceVertexIndices`, `point3f[] points` (in
**metres**), `subdivisionScheme "none"`, and a constant `displayColor` primvar, with a
`#usda 1.0` header declaring `metersPerUnit = 1` and `upAxis = "Y"`; the prim name SHALL
be sanitized to a valid USD identifier. Triangles with an out-of-range index SHALL be
skipped and only referenced vertices kept (matching the glTF writer). The USDZ container
SHALL be a **ZIP with no compression** (STORE method 0 only — compressed size equal to
uncompressed size), with a valid CRC-32, central directory, and end-of-central-directory
record, and the single entry's **data** SHALL begin on a **64-byte boundary** per the
USDZ specification (achieved via a local-file-header extra-field pad). A NULL / empty
mesh SHALL write an empty-but-valid package and still return `1`. The writer SHALL be
**deterministic** (no timestamps; fixed zip mod-time/date): the same body + `deflection`
SHALL produce a byte-identical package. The USD **binary crate** encoding is OUT OF
SCOPE (a documented follow-up) — the ASCII layer is USDZ-conformant and QuickLook reads
it directly. USDZ import is OUT OF SCOPE.

#### Scenario: A native cube exports a valid STORE-zip USDZ with 64-byte data alignment (host)
- GIVEN a native-built 10 mm box body on the host with no OCCT
- WHEN `cc_usdz_export(body, path, deflection)` writes a `.usdz`
- THEN the call SHALL return `1` AND the first local file header SHALL have compression method `0` (STORE) with equal compressed / uncompressed sizes AND the entry name SHALL end `.usda` AND the entry's data SHALL start on a 64-byte boundary AND the stored CRC-32 SHALL match the CRC-32 of the entry data AND the archive SHALL have exactly one entry in its end-of-central-directory record

#### Scenario: The embedded .usda is a valid ASCII USD mesh layer (host)
- GIVEN the USDZ package exported from the native box on the host
- WHEN the single entry's bytes are read as text
- THEN the text SHALL begin `#usda 1.0` AND declare `metersPerUnit = 1` AND contain a `def Mesh` prim with `point3f[] points`, `int[] faceVertexIndices`, and `int[] faceVertexCounts` AND the `faceVertexCounts` list SHALL contain exactly 12 entries of `3` (the box's 12 triangles)

#### Scenario: USDZ export is deterministic — byte-identical on repeat (host)
- GIVEN a native-built body and a fixed `deflection` on the host with no OCCT
- WHEN `cc_usdz_export` is invoked twice to two paths
- THEN the two packages SHALL be byte-identical (`memcmp` equal)
