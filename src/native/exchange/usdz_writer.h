// SPDX-License-Identifier: Apache-2.0
//
// usdz_writer.h — native USDZ exporter for a tessellated triangle mesh (MOAT
// exchange — glTF/USDZ, the iOS QuickLook AR handoff). OCCT-FREE, host-buildable.
//
// USDZ is Apple's AR interchange package: a ZIP container (Pixar's USD "crate"
// package format) that QuickLook / RealityKit open directly for "View in AR". The
// container is deliberately constrained by the USDZ spec:
//   * ZIP with NO compression (STORE method 0 only — data is mmap'd in place);
//   * NO encryption, ONE contiguous file body per entry;
//   * each file's DATA (not its local header) aligned to a 64-byte boundary, via
//     an "extra field" padding in the local file header, so USD's zero-copy crate
//     reader can memory-map each asset;
//   * the FIRST file is the default layer QuickLook opens.
//
// We ship the ASCII-USD (.usda) path: a single .usda layer describing the mesh as
// a UsdGeomMesh (points / faceVertexIndices / faceVertexCounts / a normals
// primvar) with meshinterpolation, packed as the sole entry. The .usda text is
// valid USD ASCII — USD accepts a .usda layer inside a .usdz package (the crate
// BINARY encoding is NOT required by the USDZ spec; ASCII layers are conformant),
// so this needs no hand-rolled USD binary crate encoder. See the honest-scope note
// in the source: the binary crate is a documented follow-up, not a limitation of
// the AR handoff (QuickLook reads the ASCII layer).
//
// The facade tessellates a body through the already-neutral tessellation path and
// hands the flat POD (fp64 vertices + int index triples) to this writer, so the
// mesh is produced ONCE — the same mesh the STL / glTF / tessellate path uses. NO
// OCCT type is referenced.
//
// Coordinate units: the kernel works in millimetres; USD's default metersPerUnit
// for USDZ is 1.0 (metres), so positions are scaled by 1e-3 and the layer declares
// metersPerUnit = 1 and upAxis = "Y" (the QuickLook convention).
//
// OCCT-FREE. Standard library only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_USDZ_WRITER_H
#define CYBERCAD_NATIVE_EXCHANGE_USDZ_WRITER_H

#include <string>
#include <vector>

namespace cybercad::native::exchange {

/// Serialize a flat triangle mesh (POD) to a USDZ package at `path`.
///   vertices : x,y,z triplets (fp64, millimetres); length = 3 * vertexCount.
///   triangles: i,j,k index triplets into `vertices`; length = 3 * facetCount.
///   name     : the mesh prim name (sanitized to a valid USD identifier); may be empty.
/// Positions are emitted in METRES (millimetres × 1e-3); connectivity is preserved.
/// Triangles with an out-of-range index are skipped and only referenced vertices are
/// kept (matching the glTF/STL writers). A NULL / empty mesh writes NOTHING and
/// returns true (empty export = no-op success). Deterministic: same mesh → byte-
/// identical package. Returns false only on an I/O failure.
///
/// The package is a STORE-only ZIP whose single .usda entry's data is aligned to a
/// 64-byte boundary (USDZ spec), so QuickLook / RealityKit can zero-copy map it.
bool usdz_export_mesh(const std::vector<double>& vertices,
                      const std::vector<int>& triangles,
                      const std::string& path,
                      const std::string& name = "CyberCadKernel_mesh");

/// Serialize the same mesh to a bare ASCII-USD (.usda) layer at `path` (not zipped).
/// Useful for inspection / debugging and as the building block the USDZ packer uses.
/// Same mesh contract as above. Returns false on I/O failure.
bool usda_export_mesh(const std::vector<double>& vertices,
                      const std::vector<int>& triangles,
                      const std::string& path,
                      const std::string& name = "CyberCadKernel_mesh");

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_USDZ_WRITER_H
