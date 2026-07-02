#ifndef CYBERCADKERNEL_COMPUTE_METAL_GPU_MESH_POST_H
#define CYBERCADKERNEL_COMPUTE_METAL_GPU_MESH_POST_H

// GPU mesh post-processing (Phase 2, capability gpu-tessellation).
//
// This module computes smooth per-vertex mesh normals as a data-parallel
// `ComputeKind::MeshPostProcess` pass on the Metal backend: it accumulates each
// incident triangle's (angle-weighted) face normal into its vertices and then
// normalizes. It is self-contained fp32 numeric code — it links only the Metal
// backend and Metal/Foundation, never OCCT, and runs on the "Apple iOS simulator
// GPU". The header compiles as plain C++20 (no Objective-C / Metal types leak
// out); the MSL kernel lives in the .mm and is compiled at runtime via
// MetalBackend::compilePipeline.
//
// Division of labour (mirrors the gpu-tessellation spec: "CPU triangulator owns
// topology"):
//   * CPU  — owns all topology/connectivity. computeNormals() builds the
//            vertex->triangle adjacency (CSR) on the CPU before dispatch.
//   * GPU  — owns only the fp32 numeric accumulation: one thread per vertex,
//            gathering its incident face normals in a fixed order.
//
// Determinism: each output vertex is written by exactly one GPU thread (a
// gather, never a scatter), and that thread visits its incident triangles in a
// fixed order (increasing triangle index, from the CSR adjacency). There are no
// cross-thread atomics or races, so repeated GPU runs on the same input are
// bit-identical — satisfying the "Deterministic GPU tessellation" requirement.
//
// Precision: Apple GPUs are fp32-only. All arithmetic here is fp32 by design;
// exact fp64 geometry never flows through this path (enforced upstream by the
// compute-backend precision guard). The GPU result matches the CPU reference
// (computeNormalsCpuReference, same formula and same accumulation order) within
// the documented fp32 tolerance below.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "compute/metal/metal_backend.h"
#include "core/result.h"

namespace cyber::metal {

// A tightly packed fp32 3-vector (12 bytes) matching MSL's `packed_float3`, used
// for both vertex positions and output normals.
struct Float3 {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};
static_assert(sizeof(Float3) == 3 * sizeof(float), "Float3 must be tightly packed");

// A triangle as three vertex indices into the vertex array (CCW winding defines
// the outward normal direction, consistently with the accumulation).
struct TriIndices {
    std::uint32_t v0 = 0;
    std::uint32_t v1 = 0;
    std::uint32_t v2 = 0;
};
static_assert(sizeof(TriIndices) == 3 * sizeof(std::uint32_t),
              "TriIndices must be tightly packed");

// Documented fp32 parity tolerance for GPU-vs-CPU per-vertex normals. Normals
// are unit vectors, so an absolute per-component tolerance is the meaningful
// bound; it is well below any meshing deflection and is the acceptance bar used
// by the on-simulator parity suite.
inline constexpr float kNormalParityAbsTol = 1e-3f;

// Compute smooth per-vertex normals on the GPU.
//
// For every vertex it accumulates, over each incident triangle (in fixed
// triangle-index order), that triangle's angle-weighted face normal
//   weight = interior angle at the vertex, in radians
//   contribution = weight * normalize(cross(e1, e2))
// where (e1, e2) are the two triangle edges emanating from the vertex (kept in
// winding order), then normalizes the sum. Degenerate corners (zero-length edge
// or zero-area face) contribute nothing; a vertex with no non-degenerate
// incidence gets a zero normal.
//
// Returns one normal per input vertex (same count, same order). Fails with an
// Error (never a crash) when: `backend` has no usable device, `vertices` is
// empty, a triangle references an out-of-range vertex, the MSL kernel fails to
// compile, a GPU buffer cannot be allocated, or the dispatch fails. Uses the
// caller's MetalBackend (its device, queue, pipeline cache and shared buffers) —
// it never creates a second device or queue.
Result<std::vector<Float3>> computeNormals(MetalBackend& backend,
                                           const std::vector<Float3>& vertices,
                                           const std::vector<TriIndices>& triangles);

// CPU reference for computeNormals: identical formula and identical (CSR,
// fixed-order) accumulation, computed in fp32 so it is the tight parity oracle
// the GPU result is compared against. Never fails: on an empty mesh it returns
// an empty vector; an out-of-range triangle index is skipped.
std::vector<Float3> computeNormalsCpuReference(
    const std::vector<Float3>& vertices, const std::vector<TriIndices>& triangles);

}  // namespace cyber::metal

#endif  // CYBERCADKERNEL_COMPUTE_METAL_GPU_MESH_POST_H
