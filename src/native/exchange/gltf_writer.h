// SPDX-License-Identifier: Apache-2.0
//
// gltf_writer.h — native glTF 2.0 exporter for a tessellated triangle mesh
// (MOAT exchange — glTF/USDZ, iPad AR / QuickLook / share / render handoff).
// OCCT-FREE, host-buildable, true millimetres → metres (glTF's linear unit).
//
// The facade tessellates a body through the already-neutral tessellation path
// (IEngine::tessellate → MeshData) and hands the flat POD (fp64 vertices + int
// index triples) to this writer, so the mesh is produced ONCE (no duplicated
// meshing) — the same mesh the STL / tessellate path uses. NO OCCT type is
// referenced; glTF writing is pure native serialization.
//
// glTF 2.0 (Khronos, ISO/IEC 12113) is the runtime asset format the app hands to
// iOS RealityKit / QuickLook / SceneKit and to any web / render pipeline. We emit
// a self-contained single-mesh, single-primitive asset:
//   * one buffer holding, in order, the interleave-free blocks
//       [ indices:uint32 ] [ POSITION:vec3<f32> ] [ NORMAL:vec3<f32> ]
//     each block padded to a 4-byte boundary (glTF accessor alignment rule);
//   * three accessors (indices SCALAR/UNSIGNED_INT, POSITION VEC3/FLOAT with
//     correct min/max bounds, NORMAL VEC3/FLOAT), two bufferViews
//     (ELEMENT_ARRAY_BUFFER for indices, ARRAY_BUFFER for the vertex attributes);
//   * one mesh with one triangle primitive, one node, one scene;
//   * one default metallic-roughness PBR material (mid grey, metal 0 / rough 0.8).
//
// Per-vertex NORMAL is derived from the mesh geometry (area-weighted face-normal
// accumulation, then normalized) — the tessellator's MeshData carries no normals,
// so we compute smooth-shading normals here. This does NOT modify the tessellator.
//
// Two container variants:
//   * .gltf  — a JSON document whose single buffer is a base64 `data:` URI
//              (fully self-contained, no sidecar .bin).
//   * .glb   — the binary container: 12-byte header (magic "glTF", version 2,
//              total length), a JSON chunk (padded with spaces to 4 bytes), and a
//              BIN chunk (padded with zeros to 4 bytes) holding the same buffer.
//
// Coordinate units: the kernel works in millimetres; glTF's convention is metres,
// so positions are scaled by 1e-3. This is the one semantic transform; connectivity
// and bounds (in metres) round-trip exactly under it.
//
// OCCT-FREE. Standard library only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_EXCHANGE_GLTF_WRITER_H
#define CYBERCAD_NATIVE_EXCHANGE_GLTF_WRITER_H

#include <string>
#include <vector>

namespace cybercad::native::exchange {

/// Serialize a flat triangle mesh (POD) to a glTF 2.0 asset at `path`.
///   vertices : x,y,z triplets (fp64, millimetres); length = 3 * vertexCount.
///   triangles: i,j,k index triplets into `vertices`; length = 3 * facetCount.
///   glb      : true  => binary .glb container; false => JSON .gltf (base64 buffer).
///   name     : the mesh/node name (also the asset's generator note); may be empty.
/// Positions are emitted in METRES (millimetres × 1e-3), the glTF linear unit;
/// connectivity is preserved index-for-index. Triangles with an out-of-range index
/// are skipped (matching the STL writer), and only vertices actually referenced are
/// kept — so the exported vertex count is the reachable count. A NULL / empty mesh
/// (no valid triangles) writes NOTHING and returns true (an empty export is a
/// no-op success, mirroring the facade's null-mesh contract).
/// Deterministic: the same mesh + mode produces a byte-identical file.
/// Returns false only on an I/O failure (path empty or file cannot be written).
bool gltf_export_mesh(const std::vector<double>& vertices,
                      const std::vector<int>& triangles,
                      const std::string& path,
                      bool glb,
                      const std::string& name = "CyberCadKernel_mesh");

}  // namespace cybercad::native::exchange

#endif  // CYBERCAD_NATIVE_EXCHANGE_GLTF_WRITER_H
