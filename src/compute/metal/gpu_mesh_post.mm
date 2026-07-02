// GPU mesh post-processing implementation (Objective-C++, iOS-only).
//
// Compiled only when CYBERCAD_HAS_METAL is set (see CMakeLists.txt), with ARC and
// -framework Metal -framework Foundation. Objective-C / Metal types stay confined
// to this translation unit; the header (gpu_mesh_post.h) trades only plain C++
// types. See that header for the API contract, the CPU/GPU division of labour,
// the determinism guarantee, and the fp32 parity tolerance.
//
// The MSL kernel below is embedded as a C++ string literal and compiled at
// runtime through MetalBackend::compilePipeline (no .metallib precompile step);
// all GPU objects come from the caller's MetalBackend (no second device/queue).

#import <Foundation/Foundation.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include "compute/metal/gpu_mesh_post.h"
#include "compute/metal/metal_backend.h"
#include "core/result.h"

namespace cyber::metal {
namespace {

// Per-vertex smooth-normal kernel. One thread per vertex (a gather): the thread
// visits its incident triangles in the fixed order given by the CSR adjacency
// (built on the CPU) and sums each triangle's angle-weighted face normal, then
// normalizes. Because each output vertex is owned by exactly one thread and the
// visit order is fixed, the result is deterministic (no atomics, no races). The
// arithmetic mirrors accumulateNormal() below exactly for tight GPU/CPU parity.
constexpr const char* kNormalsKernel = R"MSL(
#include <metal_stdlib>
using namespace metal;

static inline float3 vertexAt(device const float* v, uint i) {
    return float3(v[3u * i], v[3u * i + 1u], v[3u * i + 2u]);
}

kernel void cc_vertex_normals(device const float* verts   [[buffer(0)]],  // 3*V
                              device const uint*  tris     [[buffer(1)]],  // 3*T
                              device const uint*  adj      [[buffer(2)]],  // CSR values
                              device const uint*  offsets  [[buffer(3)]],  // CSR offsets (V+1)
                              device float*       normals  [[buffer(4)]],  // 3*V
                              constant uint&      count    [[buffer(5)]],
                              uint gid [[thread_position_in_grid]]) {
    if (gid >= count) {
        return;
    }
    float3 pv = vertexAt(verts, gid);
    float3 acc = float3(0.0);

    uint begin = offsets[gid];
    uint end   = offsets[gid + 1u];
    for (uint k = begin; k < end; ++k) {
        uint t = adj[k];
        uint a = tris[3u * t];
        uint b = tris[3u * t + 1u];
        uint c = tris[3u * t + 2u];
        // The two other corners, kept in winding order so cross() is consistent.
        uint o1, o2;
        if (a == gid)      { o1 = b; o2 = c; }
        else if (b == gid) { o1 = c; o2 = a; }
        else               { o1 = a; o2 = b; }

        float3 e1 = vertexAt(verts, o1) - pv;
        float3 e2 = vertexAt(verts, o2) - pv;
        float3 fn = cross(e1, e2);
        float fnLen = length(fn);
        float l1 = length(e1);
        float l2 = length(e2);
        if (fnLen > 1e-20 && l1 > 1e-20 && l2 > 1e-20) {
            float cosang = clamp(dot(e1, e2) / (l1 * l2), -1.0, 1.0);
            float ang = acos(cosang);
            acc += ang * (fn / fnLen);
        }
    }

    float aLen = length(acc);
    float3 n = aLen > 1e-20 ? acc / aLen : float3(0.0);
    normals[3u * gid]      = n.x;
    normals[3u * gid + 1u] = n.y;
    normals[3u * gid + 2u] = n.z;
}
)MSL";

// Vertex->triangle adjacency in CSR form. A triangle contributes to a vertex's
// list only if all three of its indices are in range, so the kernel's "other
// corner" reads are always valid. Triangles appear in increasing index order in
// each vertex's list — the fixed accumulation order that makes runs reproducible.
struct AdjacencyCsr {
    std::vector<std::uint32_t> offsets;  // size V+1
    std::vector<std::uint32_t> values;   // size = sum of incidences
};

bool triangleInRange(std::size_t vertexCount, const TriIndices& t) {
    return t.v0 < vertexCount && t.v1 < vertexCount && t.v2 < vertexCount;
}

bool allTrianglesInRange(std::size_t vertexCount, const std::vector<TriIndices>& tris) {
    for (const TriIndices& t : tris) {
        if (!triangleInRange(vertexCount, t)) {
            return false;
        }
    }
    return true;
}

AdjacencyCsr buildAdjacency(std::size_t vertexCount, const std::vector<TriIndices>& tris) {
    AdjacencyCsr csr;
    csr.offsets.assign(vertexCount + 1, 0);
    for (const TriIndices& t : tris) {
        if (!triangleInRange(vertexCount, t)) {
            continue;
        }
        ++csr.offsets[t.v0 + 1];
        ++csr.offsets[t.v1 + 1];
        ++csr.offsets[t.v2 + 1];
    }
    for (std::size_t i = 1; i <= vertexCount; ++i) {
        csr.offsets[i] += csr.offsets[i - 1];
    }
    csr.values.assign(csr.offsets[vertexCount], 0);
    std::vector<std::uint32_t> cursor(csr.offsets.begin(), csr.offsets.begin() + vertexCount);
    for (std::size_t t = 0; t < tris.size(); ++t) {
        if (!triangleInRange(vertexCount, tris[t])) {
            continue;
        }
        const auto ti = static_cast<std::uint32_t>(t);
        csr.values[cursor[tris[t].v0]++] = ti;
        csr.values[cursor[tris[t].v1]++] = ti;
        csr.values[cursor[tris[t].v2]++] = ti;
    }
    return csr;
}

// The two triangle corners other than `v`, kept in winding order. Precondition:
// `v` is one of the triangle's corners (guaranteed by the CSR construction).
void otherCorners(std::uint32_t v, const TriIndices& t, std::uint32_t& o1, std::uint32_t& o2) {
    if (t.v0 == v) {
        o1 = t.v1;
        o2 = t.v2;
    } else if (t.v1 == v) {
        o1 = t.v2;
        o2 = t.v0;
    } else {
        o1 = t.v0;
        o2 = t.v1;
    }
}

float length3(const Float3& p) { return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z); }

// CPU mirror of the MSL per-vertex accumulation (same formula, same fixed order).
Float3 accumulateNormal(std::uint32_t v, const AdjacencyCsr& csr,
                        const std::vector<Float3>& verts,
                        const std::vector<TriIndices>& tris) {
    const Float3 pv = verts[v];
    float ax = 0.0f, ay = 0.0f, az = 0.0f;

    for (std::uint32_t k = csr.offsets[v]; k < csr.offsets[v + 1]; ++k) {
        const TriIndices& t = tris[csr.values[k]];
        std::uint32_t o1 = 0, o2 = 0;
        otherCorners(v, t, o1, o2);

        const Float3 e1{verts[o1].x - pv.x, verts[o1].y - pv.y, verts[o1].z - pv.z};
        const Float3 e2{verts[o2].x - pv.x, verts[o2].y - pv.y, verts[o2].z - pv.z};
        const Float3 fn{e1.y * e2.z - e1.z * e2.y, e1.z * e2.x - e1.x * e2.z,
                        e1.x * e2.y - e1.y * e2.x};
        const float fnLen = length3(fn);
        const float l1 = length3(e1);
        const float l2 = length3(e2);
        if (fnLen > 1e-20f && l1 > 1e-20f && l2 > 1e-20f) {
            float cosang = (e1.x * e2.x + e1.y * e2.y + e1.z * e2.z) / (l1 * l2);
            cosang = std::fmin(1.0f, std::fmax(-1.0f, cosang));
            const float ang = std::acos(cosang);
            ax += ang * (fn.x / fnLen);
            ay += ang * (fn.y / fnLen);
            az += ang * (fn.z / fnLen);
        }
    }

    const float aLen = std::sqrt(ax * ax + ay * ay + az * az);
    if (aLen > 1e-20f) {
        return Float3{ax / aLen, ay / aLen, az / aLen};
    }
    return Float3{0.0f, 0.0f, 0.0f};
}

// Allocate a shared buffer initialised from `data` (or a zeroed dummy of one
// element when the vector is empty, so no zero-length allocation is requested).
template <class T>
BufferHandle makeVectorBuffer(MetalBackend& backend, const std::vector<T>& data) {
    if (data.empty()) {
        return backend.makeSharedBuffer(sizeof(T));
    }
    return backend.makeSharedBuffer(data.data(), data.size() * sizeof(T));
}

}  // namespace

std::vector<Float3> computeNormalsCpuReference(const std::vector<Float3>& vertices,
                                               const std::vector<TriIndices>& triangles) {
    if (vertices.empty()) {
        return {};
    }
    const AdjacencyCsr csr = buildAdjacency(vertices.size(), triangles);
    std::vector<Float3> normals(vertices.size());
    for (std::uint32_t v = 0; v < vertices.size(); ++v) {
        normals[v] = accumulateNormal(v, csr, vertices, triangles);
    }
    return normals;
}

Result<std::vector<Float3>> computeNormals(MetalBackend& backend,
                                           const std::vector<Float3>& vertices,
                                           const std::vector<TriIndices>& triangles) {
    if (vertices.empty()) {
        return make_error("computeNormals: empty vertex set");
    }
    if (backend.device() == nullptr) {
        return make_error("computeNormals: Metal backend has no device");
    }
    const std::size_t vertexCount = vertices.size();
    if (!allTrianglesInRange(vertexCount, triangles)) {
        return make_error("computeNormals: triangle references an out-of-range vertex");
    }

    const AdjacencyCsr csr = buildAdjacency(vertexCount, triangles);

    Result<PipelineHandle> pso = backend.compilePipeline(kNormalsKernel, "cc_vertex_normals");
    if (!pso) {
        return pso.error();
    }

    // Float3 and TriIndices are tightly packed, so their vectors map 1:1 onto the
    // flat float/uint layouts the kernel expects — no conversion needed.
    BufferHandle hVerts = makeVectorBuffer(backend, vertices);
    BufferHandle hTris = makeVectorBuffer(backend, triangles);
    BufferHandle hAdj = makeVectorBuffer(backend, csr.values);
    BufferHandle hOffsets = makeVectorBuffer(backend, csr.offsets);
    BufferHandle hNormals = backend.makeSharedBuffer(vertexCount * sizeof(Float3));
    const auto count = static_cast<std::uint32_t>(vertexCount);
    BufferHandle hCount = backend.makeSharedBuffer(&count, sizeof(count));

    if (!hVerts || !hTris || !hAdj || !hOffsets || !hNormals || !hCount) {
        return make_error("computeNormals: shared buffer allocation failed");
    }

    Result<void> dispatched = backend.dispatch(
        pso.value(), {hVerts, hTris, hAdj, hOffsets, hNormals, hCount}, vertexCount);
    if (!dispatched) {
        return dispatched.error();
    }

    const auto* out = static_cast<const Float3*>(backend.bufferContents(hNormals));
    if (out == nullptr) {
        return make_error("computeNormals: could not read GPU output buffer");
    }
    return std::vector<Float3>(out, out + vertexCount);
}

}  // namespace cyber::metal
