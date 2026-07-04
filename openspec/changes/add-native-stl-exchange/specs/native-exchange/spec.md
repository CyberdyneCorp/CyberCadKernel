# native-exchange

## ADDED Requirements

### Requirement: Native STL export (binary + ASCII) of a tessellated body

`cc_stl_export(body, path, deflection, binary)` SHALL serialize the tessellated
triangle mesh of `body` to an STL file at `path` and return `1` on success, `0` on
failure (with `cc_last_error` set). The facade SHALL obtain the mesh from the
already-neutral tessellation path (`IEngine::tessellate` → `MeshData`, honouring
`deflection`) and hand its flat `(vertices, triangles)` POD to a native writer —
reusing the existing meshing, adding no new export virtual, and referencing no
OCCT type (the writer is OCCT-free and host-buildable). All coordinates SHALL be in
**true millimetres**. Each facet SHALL carry a geometric normal
`normalize((v1-v0) × (v2-v0))`, emitting `(0,0,0)` for a zero-area facet while still
writing that facet's three vertices (export SHALL NOT fail on a degenerate facet).
When `binary` is non-zero (the default) the file SHALL be a **binary STL**: an
80-byte header that does NOT begin with `"solid"` and carries no timestamp / host /
build-id, followed by a little-endian `uint32` facet count equal to the number of
emitted facets, followed by 50 bytes per facet (`float32[3]` normal, three
`float32[3]` vertices, a `uint16` attribute of `0`), in mesh triangle and stored
winding order. When `binary` is zero the file SHALL be an **ASCII STL** framed
`solid …` / `endsolid …` with balanced `facet normal` / `outer loop` / three
`vertex` / `endloop` / `endfacet` records and locale-independent floats (never a
decimal comma). The writer SHALL be **deterministic**: the same body and
`deflection` SHALL produce a byte-identical file on repeat, for both binary and
ASCII.

#### Scenario: A native box exports a binary STL whose facet count field matches (host)
- GIVEN a native-built axis-aligned box body, built on the host with no OCCT
- WHEN `cc_stl_export(body, path, deflection, 1)` writes a binary STL and the file is re-read
- THEN the call SHALL return `1` AND the little-endian `uint32` facet count at offset 80 SHALL equal the number of tessellated triangles AND the file size SHALL be exactly `84 + 50·count` bytes AND the 80-byte header SHALL NOT begin with `"solid"`

#### Scenario: The same body exports a well-formed ASCII STL (host)
- GIVEN the same native-built box body on the host with no OCCT
- WHEN `cc_stl_export(body, path, deflection, 0)` writes an ASCII STL
- THEN the call SHALL return `1` AND the file SHALL begin `solid` and end `endsolid` with balanced `facet normal` / `outer loop` / three `vertex` / `endloop` / `endfacet` records per facet AND SHALL contain no decimal comma

#### Scenario: Export is deterministic — byte-identical on repeat (host)
- GIVEN a native-built body and a fixed `deflection` on the host with no OCCT
- WHEN `cc_stl_export` is invoked twice to two paths with the same `binary` mode
- THEN the two files SHALL be byte-identical (`memcmp` equal) for both the binary and the ASCII mode

### Requirement: Native STL import (ASCII/binary auto-detect) as a mesh body

`cc_stl_import(path)` SHALL read an STL file — **auto-detecting ASCII vs binary** —
and return a **mesh body** (a welded triangle soup), or `0` on failure with
`cc_last_error` set. Auto-detection SHALL treat a file whose size equals
`84 + 50·N` for the little-endian `uint32` count `N` at offset 80 as binary
(definitive, even when its 80-byte header begins with `"solid"`), SHALL treat a
file containing a non-text byte in its leading bytes as binary, and SHALL treat a
text file beginning `solid` and containing `facet` as ASCII. The reader SHALL
**weld** coincident vertices within a tolerance and SHALL tolerate degenerate /
zero-area facets, non-manifold edges, and inconsistent winding — storing the soup
as-is without fixing winding or enforcing manifold. The result SHALL be an
import-as-mesh body only (NOT B-rep reconstruction), carried directly as a
mesh-backed native body so that display, `cc_tessellate`, bounding box, surface
area, and volume-if-closed all work through the existing mesh-consuming ops
(volume is meaningful only for a watertight mesh; an open soup reports area but an
invalid / zero volume). On **malformed** input (missing file, short read, bad
count, parse failure, or zero valid triangles after skipping degenerates) the
import SHALL fail cleanly — set `cc_last_error`, return `0`, and register NO partial
body (nothing is leaked). An export followed by an import of the same body SHALL
**round-trip**: the re-imported mesh SHALL preserve the triangle count and the
bounding box (within tolerance).

#### Scenario: A binary STL round-trips to the same triangle count and bounding box (host)
- GIVEN a native-built box body exported to a binary STL on the host with no OCCT
- WHEN `cc_stl_import(path)` reads it back and the resulting body is tessellated and measured
- THEN the call SHALL return a non-zero `CCShapeId` AND `cc_tessellate` of the imported body SHALL yield the exported triangle count AND `cc_bounding_box` SHALL match the source body within tolerance

#### Scenario: ASCII and binary of the same mesh auto-detect and import identically (host)
- GIVEN the same body exported once as ASCII and once as binary STL on the host
- WHEN each file is imported via `cc_stl_import` (format auto-detected), including a hand-crafted binary file whose 80-byte header begins with `"solid "`
- THEN both SHALL import successfully AND yield the same welded triangle count and the same bounding box (within float32 tolerance) AND the `"solid "`-headed binary file SHALL be detected as binary by size identity

#### Scenario: A malformed STL fails cleanly with no partial body (host)
- GIVEN a missing path and a truncated / garbage STL file on the host
- WHEN `cc_stl_import` is called on each
- THEN each call SHALL return `0` AND set a non-empty `cc_last_error` AND SHALL NOT crash or register a partial body

#### Scenario: An STL mixing valid and degenerate facets imports, skipping only the degenerate (host)
- GIVEN an otherwise-valid ASCII STL containing two well-formed facets (with leading-`+` signed coordinates) and one zero-area / repeated-vertex facet
- WHEN `cc_stl_import(path)` reads it
- THEN the call SHALL return a non-zero `CCShapeId` AND `cc_tessellate` of the imported body SHALL yield exactly the two valid triangles (the degenerate facet is skipped, the leading-`+` coordinates are accepted)
