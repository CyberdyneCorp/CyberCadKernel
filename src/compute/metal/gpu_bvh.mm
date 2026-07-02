// GPU LBVH build + stackless ray traversal + frustum pick (Objective-C++,
// iOS-only). Compiled only when CYBERCAD_HAS_METAL is set. See gpu_bvh.h for the
// API contract, the pipeline overview, and the determinism / tie-break rules.
//
// All Metal/Objective-C types stay in this translation unit; the header trades
// only plain C++20 and the opaque void* handles from metal_backend.h. The MSL
// kernels are embedded as string literals and compiled at runtime through the
// backend's compilePipeline (no .metallib precompile step).
//
// ── Buffer layouts (fp32; all words are 4 bytes, so C++ and MSL agree without
//    struct-packing ambiguity — every buffer is a flat float/uint/int array) ──
//   triangles : 9 floats / tri   [v0.xyz, v1.xyz, v2.xyz]
//   rays      : 8 floats / ray   [origin.xyz, dir.xyz, tmin, tmax]
//   hits      : 4 words  / ray   [tri(int bits), t, u, v]      (tri via as_type)
//   prim aabb : 6 floats / prim  [min.xyz, max.xyz]
//   nodes     : 9 words  / node  [min.xyz, max.xyz,
//                                 primStart(int), primCount(int), escape(int)]
//               A leaf has primCount > 0 and references prim_index[primStart ..
//               primStart+primCount). An internal node has primCount == 0 and its
//               first child at index+1 (DFS preorder). `escape` is the index of
//               the next node to visit when this node/box is skipped (== the
//               index just past this node's whole subtree), giving a stackless
//               walk with no per-thread stack.

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "compute/metal/gpu_bvh.h"
#include "compute/metal/metal_backend.h"
#include "core/result.h"

namespace cyber::metal {
namespace {

constexpr int kLeafSize = 4;         // max primitives per leaf
constexpr int kMortonBits = 10;      // per axis => 30-bit codes
constexpr float kMortonScale = 1023.0f;  // (1 << kMortonBits) - 1
constexpr int kNodeWords = 9;        // words per linear BVH node
constexpr float kRayTriDetEps = 1e-8f;

// ── Embedded MSL: primitives, Morton, stackless traversal, frustum ──────────
constexpr const char* kBvhMsl = R"MSL(
#include <metal_stdlib>
using namespace metal;

// Möller–Trumbore, two-sided. Mirrors the CPU reference exactly (same op order,
// same epsilon) so GPU and CPU agree at fp32.
inline bool rayTri(float3 o, float3 d, float3 v0, float3 v1, float3 v2,
                   float tmin, float tmax,
                   thread float& tOut, thread float& uOut, thread float& vOut) {
    float3 e1 = v1 - v0;
    float3 e2 = v2 - v0;
    float3 p = cross(d, e2);
    float det = dot(e1, p);
    if (fabs(det) < 1e-8f) return false;
    float inv = 1.0f / det;
    float3 tv = o - v0;
    float u = dot(tv, p) * inv;
    if (u < 0.0f || u > 1.0f) return false;
    float3 q = cross(tv, e1);
    float v = dot(d, q) * inv;
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = dot(e2, q) * inv;
    if (t < tmin || t > tmax) return false;
    tOut = t; uOut = u; vOut = v;
    return true;
}

// Ray/AABB slab test over [tmin, tmax]; returns true if the box is entered.
inline bool hitAabb(float3 mn, float3 mx, float3 o, float3 invd,
                    float tmin, float tmax) {
    float3 t0 = (mn - o) * invd;
    float3 t1 = (mx - o) * invd;
    float3 tsm = min(t0, t1);
    float3 tbg = max(t0, t1);
    float tn = max(max(tsm.x, tsm.y), max(tsm.z, tmin));
    float tf = min(min(tbg.x, tbg.y), min(tbg.z, tmax));
    return tn <= tf;
}

inline uint expandBits(uint v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

kernel void bvh_primitives(device const float* tris     [[buffer(0)]],
                           device float*       aabb      [[buffer(1)]],
                           device float*       centroid  [[buffer(2)]],
                           constant uint&      count     [[buffer(3)]],
                           uint gid [[thread_position_in_grid]]) {
    if (gid >= count) return;
    uint b = gid * 9u;
    float3 v0 = float3(tris[b + 0], tris[b + 1], tris[b + 2]);
    float3 v1 = float3(tris[b + 3], tris[b + 4], tris[b + 5]);
    float3 v2 = float3(tris[b + 6], tris[b + 7], tris[b + 8]);
    float3 mn = min(v0, min(v1, v2));
    float3 mx = max(v0, max(v1, v2));
    float3 c = (v0 + v1 + v2) * (1.0f / 3.0f);
    uint o6 = gid * 6u;
    aabb[o6 + 0] = mn.x; aabb[o6 + 1] = mn.y; aabb[o6 + 2] = mn.z;
    aabb[o6 + 3] = mx.x; aabb[o6 + 4] = mx.y; aabb[o6 + 5] = mx.z;
    uint o3 = gid * 3u;
    centroid[o3 + 0] = c.x; centroid[o3 + 1] = c.y; centroid[o3 + 2] = c.z;
}

kernel void bvh_morton(device const float* centroid  [[buffer(0)]],
                       device uint*        codes     [[buffer(1)]],
                       constant float*     sceneMin   [[buffer(2)]],
                       constant float*     invExtent  [[buffer(3)]],
                       constant uint&      count      [[buffer(4)]],
                       uint gid [[thread_position_in_grid]]) {
    if (gid >= count) return;
    uint o = gid * 3u;
    float nx = clamp((centroid[o + 0] - sceneMin[0]) * invExtent[0], 0.0f, 1.0f);
    float ny = clamp((centroid[o + 1] - sceneMin[1]) * invExtent[1], 0.0f, 1.0f);
    float nz = clamp((centroid[o + 2] - sceneMin[2]) * invExtent[2], 0.0f, 1.0f);
    uint x = min(uint(nx * 1023.0f), 1023u);
    uint y = min(uint(ny * 1023.0f), 1023u);
    uint z = min(uint(nz * 1023.0f), 1023u);
    codes[gid] = (expandBits(x) << 2) | (expandBits(y) << 1) | expandBits(z);
}

kernel void bvh_traverse(device const float* nodes      [[buffer(0)]],
                         device const uint*  primIndex   [[buffer(1)]],
                         device const float* tris        [[buffer(2)]],
                         device const float* rays        [[buffer(3)]],
                         device float*       hits        [[buffer(4)]],
                         constant uint&      nodeCount   [[buffer(5)]],
                         constant uint&      rayCount    [[buffer(6)]],
                         uint gid [[thread_position_in_grid]]) {
    if (gid >= rayCount) return;
    uint rb = gid * 8u;
    float3 o = float3(rays[rb + 0], rays[rb + 1], rays[rb + 2]);
    float3 d = float3(rays[rb + 3], rays[rb + 4], rays[rb + 5]);
    float tmin = rays[rb + 6];
    float tmax = rays[rb + 7];
    float3 invd = 1.0f / d;

    int bestTri = -1;
    float bestT = tmax;
    float bestU = 0.0f, bestV = 0.0f;

    uint i = 0u;
    while (i < nodeCount) {
        uint n = i * 9u;
        float3 mn = float3(nodes[n + 0], nodes[n + 1], nodes[n + 2]);
        float3 mx = float3(nodes[n + 3], nodes[n + 4], nodes[n + 5]);
        int primStart = as_type<int>(nodes[n + 6]);
        int primCount = as_type<int>(nodes[n + 7]);
        int escape    = as_type<int>(nodes[n + 8]);

        if (hitAabb(mn, mx, o, invd, tmin, bestT)) {
            if (primCount > 0) {  // leaf
                for (int k = 0; k < primCount; ++k) {
                    uint p = primIndex[primStart + k];
                    uint tb = p * 9u;
                    float3 a = float3(tris[tb + 0], tris[tb + 1], tris[tb + 2]);
                    float3 b = float3(tris[tb + 3], tris[tb + 4], tris[tb + 5]);
                    float3 c = float3(tris[tb + 6], tris[tb + 7], tris[tb + 8]);
                    float t, u, v;
                    if (rayTri(o, d, a, b, c, tmin, tmax, t, u, v)) {
                        float tie = 1e-4f * max(1.0f, fabs(t));
                        bool accept;
                        if (bestTri < 0)                 accept = true;
                        else if (fabs(t - bestT) <= tie) accept = int(p) < bestTri;
                        else                             accept = t < bestT;
                        if (accept) { bestTri = int(p); bestT = t; bestU = u; bestV = v; }
                    }
                }
                i = uint(escape);
            } else {
                i = i + 1u;  // descend to first child
            }
        } else {
            i = uint(escape);
        }
    }

    uint hb = gid * 4u;
    hits[hb + 0] = as_type<float>(bestTri);
    hits[hb + 1] = bestT;
    hits[hb + 2] = bestU;
    hits[hb + 3] = bestV;
}

kernel void bvh_frustum(device const float* aabb    [[buffer(0)]],
                        device const float* planes  [[buffer(1)]],
                        device uint*        flags    [[buffer(2)]],
                        constant uint&      count    [[buffer(3)]],
                        uint gid [[thread_position_in_grid]]) {
    if (gid >= count) return;
    uint o = gid * 6u;
    float3 mn = float3(aabb[o + 0], aabb[o + 1], aabb[o + 2]);
    float3 mx = float3(aabb[o + 3], aabb[o + 4], aabb[o + 5]);
    bool inside = true;
    for (int p = 0; p < 6 && inside; ++p) {
        float a = planes[p * 4 + 0];
        float b = planes[p * 4 + 1];
        float c = planes[p * 4 + 2];
        float dd = planes[p * 4 + 3];
        float px = (a >= 0.0f) ? mx.x : mn.x;
        float py = (b >= 0.0f) ? mx.y : mn.y;
        float pz = (c >= 0.0f) ? mx.z : mn.z;
        if (a * px + b * py + c * pz + dd < 0.0f) inside = false;
    }
    flags[gid] = inside ? 1u : 0u;
}
)MSL";

// ── Small fp32 CPU mirror of the MSL (same op order / epsilons) ─────────────
struct V3 {
    float x, y, z;
};
V3 sub(V3 a, V3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
float dot3(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross3(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

V3 triVert(const float* tri9, int i) {
    return {tri9[i * 3 + 0], tri9[i * 3 + 1], tri9[i * 3 + 2]};
}

// CPU Möller–Trumbore, byte-for-byte the same math as the MSL rayTri.
bool ray_tri(V3 o, V3 d, V3 v0, V3 v1, V3 v2, float tmin, float tmax, float& tOut,
             float& uOut, float& vOut) {
    V3 e1 = sub(v1, v0);
    V3 e2 = sub(v2, v0);
    V3 p = cross3(d, e2);
    float det = dot3(e1, p);
    if (std::fabs(det) < kRayTriDetEps) return false;
    float inv = 1.0f / det;
    V3 tv = sub(o, v0);
    float u = dot3(tv, p) * inv;
    if (u < 0.0f || u > 1.0f) return false;
    V3 q = cross3(tv, e1);
    float v = dot3(d, q) * inv;
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = dot3(e2, q) * inv;
    if (t < tmin || t > tmax) return false;
    tOut = t; uOut = u; vOut = v;
    return true;
}

// Shared nearest-hit tie-break: closest t wins; a near-tie (within tie_eps) is
// resolved by the lower primitive index. Identical on GPU, CPU, and oracle.
bool accept_hit(int prim, float t, int best_tri, float best_t) {
    if (best_tri < 0) return true;
    const float tie = 1e-4f * std::max(1.0f, std::fabs(t));
    if (std::fabs(t - best_t) <= tie) return prim < best_tri;
    return t < best_t;
}

std::uint32_t expand_bits(std::uint32_t v) {
    v = (v * 0x00010001u) & 0xFF0000FFu;
    v = (v * 0x00000101u) & 0x0F00F00Fu;
    v = (v * 0x00000011u) & 0xC30C30C3u;
    v = (v * 0x00000005u) & 0x49249249u;
    return v;
}

std::uint32_t morton3(float nx, float ny, float nz) {
    auto q = [](float n) {
        float c = std::min(std::max(n, 0.0f), 1.0f) * kMortonScale;
        return std::min(static_cast<std::uint32_t>(c), 1023u);
    };
    return (expand_bits(q(nx)) << 2) | (expand_bits(q(ny)) << 1) | expand_bits(q(nz));
}

// Conservative AABB-vs-frustum test (mirror of the MSL bvh_frustum body).
bool box_in_frustum(const float* aabb6, const Frustum& f) {
    for (int p = 0; p < 6; ++p) {
        const float a = f.planes[p][0], b = f.planes[p][1], c = f.planes[p][2],
                    dd = f.planes[p][3];
        const float px = (a >= 0.0f) ? aabb6[3] : aabb6[0];
        const float py = (b >= 0.0f) ? aabb6[4] : aabb6[1];
        const float pz = (c >= 0.0f) ? aabb6[5] : aabb6[2];
        if (a * px + b * py + c * pz + dd < 0.0f) return false;
    }
    return true;
}

// ── CPU-side hierarchy build (step 4) ───────────────────────────────────────
// Emits a DFS-preorder linear node array with escape indices. Deterministic:
// the input `order` is sorted by (Morton code, primitive index), and median
// splits of that fixed order produce a fixed tree.
class HierarchyBuilder {
public:
    HierarchyBuilder(const std::vector<std::uint32_t>& order, const std::vector<float>& prim_aabb)
        : order_(order), prim_aabb_(prim_aabb) {}

    // Returns the flat node-word array (kNodeWords per node).
    std::vector<std::int32_t> build() {
        nodes_.clear();
        if (!order_.empty()) {
            emit(0, static_cast<int>(order_.size()));
        }
        return std::move(nodes_);
    }

private:
    // Emit the subtree over sorted range [lo, hi); returns the node's index.
    int emit(int lo, int hi) {
        const int self = static_cast<int>(nodes_.size()) / kNodeWords;
        nodes_.resize(nodes_.size() + kNodeWords);  // placeholder slot

        float mn[3] = {kInf, kInf, kInf};
        float mx[3] = {-kInf, -kInf, -kInf};
        int prim_start = 0, prim_count = 0;

        if (hi - lo <= kLeafSize) {
            leaf_bounds(lo, hi, mn, mx);
            prim_start = lo;
            prim_count = hi - lo;
        } else {
            const int mid = (lo + hi) / 2;
            const int left = emit(lo, mid);
            const int right = emit(mid, hi);
            union_child(left, mn, mx);
            union_child(right, mn, mx);
        }

        write_node(self, mn, mx, prim_start, prim_count);
        return self;
    }

    void leaf_bounds(int lo, int hi, float* mn, float* mx) const {
        for (int k = lo; k < hi; ++k) {
            const float* a = &prim_aabb_[order_[k] * 6];
            for (int c = 0; c < 3; ++c) {
                mn[c] = std::min(mn[c], a[c]);
                mx[c] = std::max(mx[c], a[c + 3]);
            }
        }
    }

    void union_child(int child, float* mn, float* mx) const {
        const std::int32_t* w = &nodes_[static_cast<std::size_t>(child) * kNodeWords];
        for (int c = 0; c < 3; ++c) {
            mn[c] = std::min(mn[c], word_f(w[c]));
            mx[c] = std::max(mx[c], word_f(w[c + 3]));
        }
    }

    void write_node(int self, const float* mn, const float* mx, int prim_start, int prim_count) {
        std::int32_t* w = &nodes_[static_cast<std::size_t>(self) * kNodeWords];
        for (int c = 0; c < 3; ++c) {
            w[c] = f_word(mn[c]);
            w[c + 3] = f_word(mx[c]);
        }
        w[6] = prim_start;
        w[7] = prim_count;
        w[8] = static_cast<std::int32_t>(nodes_.size() / kNodeWords);  // escape
    }

    static std::int32_t f_word(float f) {
        std::int32_t v;
        std::memcpy(&v, &f, sizeof(v));
        return v;
    }
    static float word_f(std::int32_t v) {
        float f;
        std::memcpy(&f, &v, sizeof(f));
        return f;
    }

    static constexpr float kInf = std::numeric_limits<float>::infinity();
    const std::vector<std::uint32_t>& order_;
    const std::vector<float>& prim_aabb_;
    std::vector<std::int32_t> nodes_;
};

// Sort primitive indices by (Morton code, index) — the fixed tie-break that makes
// the node array reproducible.
std::vector<std::uint32_t> sorted_order(const std::vector<std::uint32_t>& codes) {
    std::vector<std::uint32_t> order(codes.size());
    for (std::uint32_t i = 0; i < codes.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&codes](std::uint32_t a, std::uint32_t b) {
        if (codes[a] != codes[b]) return codes[a] < codes[b];
        return a < b;
    });
    return order;
}

}  // namespace

// ── Objective-C++ GPU helpers (kept tiny; the heavy code is the MSL above) ───
namespace {

// Read a shared buffer's contents into a typed CPU vector of `count` elements.
template <class T>
std::vector<T> read_back(MetalBackend& be, BufferHandle h, std::size_t count) {
    std::vector<T> out(count);
    const auto* p = static_cast<const T*>(be.bufferContents(h));
    if (p != nullptr) std::memcpy(out.data(), p, count * sizeof(T));
    return out;
}

// GPU path: per-triangle AABB + centroid, returning them on the CPU.
Result<void> gpu_primitives(MetalBackend& be, BufferHandle tri_buf, BufferHandle aabb_buf,
                            std::uint32_t n, std::vector<float>& centroids_out) {
    BufferHandle cbuf = be.makeSharedBuffer(std::size_t(n) * 3 * sizeof(float));
    BufferHandle nbuf = be.makeSharedBuffer(&n, sizeof(n));
    auto pso = be.compilePipeline(kBvhMsl, "bvh_primitives");
    if (!pso) return pso.error();
    auto d = be.dispatch(pso.value(), {tri_buf, aabb_buf, cbuf, nbuf}, n);
    if (!d) return d.error();
    centroids_out = read_back<float>(be, cbuf, std::size_t(n) * 3);
    return Result<void>::ok();
}

// GPU path: 30-bit Morton codes from centroids and scene bounds.
Result<std::vector<std::uint32_t>> gpu_morton(MetalBackend& be, const std::vector<float>& centroids,
                                              const float scene_min[3], const float inv_ext[3],
                                              std::uint32_t n) {
    BufferHandle cbuf = be.makeSharedBuffer(centroids.data(), centroids.size() * sizeof(float));
    BufferHandle codebuf = be.makeSharedBuffer(std::size_t(n) * sizeof(std::uint32_t));
    BufferHandle minbuf = be.makeSharedBuffer(scene_min, 3 * sizeof(float));
    BufferHandle extbuf = be.makeSharedBuffer(inv_ext, 3 * sizeof(float));
    BufferHandle nbuf = be.makeSharedBuffer(&n, sizeof(n));
    auto pso = be.compilePipeline(kBvhMsl, "bvh_morton");
    if (!pso) return pso.error();
    auto d = be.dispatch(pso.value(), {cbuf, codebuf, minbuf, extbuf, nbuf}, n);
    if (!d) return d.error();
    return read_back<std::uint32_t>(be, codebuf, n);
}

}  // namespace

// ── build ────────────────────────────────────────────────────────────────────
Result<GpuBvh> GpuBvh::build(std::shared_ptr<MetalBackend> backend,
                             const std::vector<GpuTriangle>& triangles) {
    if (triangles.empty()) {
        return make_error("GpuBvh::build: empty triangle set");
    }
    const std::uint32_t n = static_cast<std::uint32_t>(triangles.size());

    GpuBvh bvh;
    bvh.backend_ = backend;
    bvh.tri_count_ = n;

    // Flatten triangles (9 floats each) — needed for both GPU buffers and CPU.
    bvh.tri_data_.resize(std::size_t(n) * 9);
    std::memcpy(bvh.tri_data_.data(), triangles.data(), bvh.tri_data_.size() * sizeof(float));

    bvh.prim_aabb_.resize(std::size_t(n) * 6);
    std::vector<float> centroids(std::size_t(n) * 3);

    // Step 1: per-primitive AABB + centroid (GPU when a backend is present).
    if (backend) {
        bvh.tri_buf_ = backend->makeSharedBuffer(bvh.tri_data_.data(),
                                                 bvh.tri_data_.size() * sizeof(float));
        bvh.aabb_buf_ = backend->makeSharedBuffer(bvh.prim_aabb_.size() * sizeof(float));
        if (!bvh.tri_buf_ || !bvh.aabb_buf_) return make_error("GpuBvh::build: buffer alloc failed");
        auto r = gpu_primitives(*backend, bvh.tri_buf_, bvh.aabb_buf_, n, centroids);
        if (!r) return r.error();
        bvh.prim_aabb_ = read_back<float>(*backend, bvh.aabb_buf_, bvh.prim_aabb_.size());
    } else {
        for (std::uint32_t i = 0; i < n; ++i) {
            cpu_primitive(bvh.tri_data_.data() + std::size_t(i) * 9,
                          bvh.prim_aabb_.data() + std::size_t(i) * 6, centroids.data() + std::size_t(i) * 3);
        }
    }

    // Step 2: reduce centroid bounds (serial, deterministic).
    float scene_min[3] = {centroids[0], centroids[1], centroids[2]};
    float scene_max[3] = {centroids[0], centroids[1], centroids[2]};
    for (std::uint32_t i = 1; i < n; ++i) {
        for (int c = 0; c < 3; ++c) {
            scene_min[c] = std::min(scene_min[c], centroids[std::size_t(i) * 3 + c]);
            scene_max[c] = std::max(scene_max[c], centroids[std::size_t(i) * 3 + c]);
        }
    }
    float inv_ext[3];
    for (int c = 0; c < 3; ++c) {
        const float ext = scene_max[c] - scene_min[c];
        inv_ext[c] = ext > 0.0f ? 1.0f / ext : 0.0f;  // degenerate axis => code 0
    }

    // Step 3: Morton codes (GPU or CPU).
    std::vector<std::uint32_t> codes;
    if (backend) {
        auto m = gpu_morton(*backend, centroids, scene_min, inv_ext, n);
        if (!m) return m.error();
        codes = std::move(m).value();
    } else {
        codes.resize(n);
        for (std::uint32_t i = 0; i < n; ++i) {
            codes[i] = morton3((centroids[std::size_t(i) * 3 + 0] - scene_min[0]) * inv_ext[0],
                               (centroids[std::size_t(i) * 3 + 1] - scene_min[1]) * inv_ext[1],
                               (centroids[std::size_t(i) * 3 + 2] - scene_min[2]) * inv_ext[2]);
        }
    }

    // Step 4: deterministic sort + linear hierarchy (CPU).
    bvh.prim_index_ = sorted_order(codes);
    bvh.node_words_ = HierarchyBuilder(bvh.prim_index_, bvh.prim_aabb_).build();
    bvh.node_count_ = bvh.node_words_.size() / kNodeWords;

    // Upload traversal buffers for the GPU query path.
    if (backend) {
        bvh.node_buf_ = backend->makeSharedBuffer(bvh.node_words_.data(),
                                                  bvh.node_words_.size() * sizeof(std::int32_t));
        bvh.prim_index_buf_ = backend->makeSharedBuffer(bvh.prim_index_.data(),
                                                        bvh.prim_index_.size() * sizeof(std::uint32_t));
        if (!bvh.node_buf_ || !bvh.prim_index_buf_) return make_error("GpuBvh::build: node buffer alloc failed");
    }
    return bvh;
}

// ── closestHit ─────────────────────────────────────────────────────────────
Result<std::vector<GpuHit>> GpuBvh::closestHit(const std::vector<GpuRay>& rays) const {
    if (rays.empty()) return std::vector<GpuHit>{};
    return backend_ ? gpu_closest_hit(rays) : std::vector<GpuHit>(cpu_closest_hit(rays));
}

Result<GpuHit> GpuBvh::closestHit(const GpuRay& ray) const {
    auto r = closestHit(std::vector<GpuRay>{ray});
    if (!r) return r.error();
    return r.value().front();
}

// ── frustumPick ────────────────────────────────────────────────────────────
Result<std::vector<std::uint32_t>> GpuBvh::frustumPick(const Frustum& frustum) const {
    return backend_ ? gpu_frustum_pick(frustum) : cpu_frustum_pick(frustum);
}

// ── Brute-force oracles (public; no BVH) ─────────────────────────────────────
std::vector<GpuHit> GpuBvh::closestHitBruteForce(const std::vector<GpuTriangle>& triangles,
                                                 const std::vector<GpuRay>& rays) {
    std::vector<GpuHit> out(rays.size());
    for (std::size_t r = 0; r < rays.size(); ++r) {
        const GpuRay& ray = rays[r];
        const V3 o{ray.origin[0], ray.origin[1], ray.origin[2]};
        const V3 d{ray.dir[0], ray.dir[1], ray.dir[2]};
        GpuHit best;
        for (std::uint32_t i = 0; i < triangles.size(); ++i) {
            const GpuTriangle& t = triangles[i];
            const V3 v0{t.v0[0], t.v0[1], t.v0[2]};
            const V3 v1{t.v1[0], t.v1[1], t.v1[2]};
            const V3 v2{t.v2[0], t.v2[1], t.v2[2]};
            float th, uh, vh;
            if (ray_tri(o, d, v0, v1, v2, ray.tmin, ray.tmax, th, uh, vh) &&
                accept_hit(static_cast<int>(i), th, best.tri, best.t)) {
                best = GpuHit{static_cast<std::int32_t>(i), th, uh, vh};
            }
        }
        out[r] = best;
    }
    return out;
}

std::vector<std::uint32_t> GpuBvh::frustumPickBruteForce(const std::vector<GpuTriangle>& triangles,
                                                         const Frustum& frustum) {
    std::vector<std::uint32_t> picked;
    for (std::uint32_t i = 0; i < triangles.size(); ++i) {
        const GpuTriangle& t = triangles[i];
        float a6[6] = {t.v0[0], t.v0[1], t.v0[2], t.v0[0], t.v0[1], t.v0[2]};
        for (const float* v : {t.v1, t.v2}) {
            for (int c = 0; c < 3; ++c) {
                a6[c] = std::min(a6[c], v[c]);
                a6[c + 3] = std::max(a6[c + 3], v[c]);
            }
        }
        if (box_in_frustum(a6, frustum)) picked.push_back(i);
    }
    return picked;  // already ascending
}

// ── private CPU-fallback + GPU-dispatch members ──────────────────────────────
void GpuBvh::cpu_primitive(const float* tri9, float* aabb6, float* centroid3) {
    for (int c = 0; c < 3; ++c) {
        aabb6[c] = std::min(tri9[c], std::min(tri9[3 + c], tri9[6 + c]));
        aabb6[3 + c] = std::max(tri9[c], std::max(tri9[3 + c], tri9[6 + c]));
        centroid3[c] = (tri9[c] + tri9[3 + c] + tri9[6 + c]) * (1.0f / 3.0f);
    }
}

std::vector<GpuHit> GpuBvh::cpu_closest_hit(const std::vector<GpuRay>& rays) const {
    std::vector<GpuHit> out(rays.size());
    for (std::size_t r = 0; r < rays.size(); ++r) {
        out[r] = traverse_one(rays[r]);
    }
    return out;
}

// Stackless walk of the linear node array — the exact CPU mirror of bvh_traverse.
GpuHit GpuBvh::traverse_one(const GpuRay& ray) const {
    const float o[3] = {ray.origin[0], ray.origin[1], ray.origin[2]};
    const float invd[3] = {1.0f / ray.dir[0], 1.0f / ray.dir[1], 1.0f / ray.dir[2]};
    GpuHit best;
    best.t = ray.tmax;

    std::uint32_t i = 0;
    while (i < node_count_) {
        const std::int32_t* w = &node_words_[std::size_t(i) * kNodeWords];
        float mn[3], mx[3];
        std::memcpy(mn, w, 3 * sizeof(float));
        std::memcpy(mx, w + 3, 3 * sizeof(float));
        const int prim_start = w[6], prim_count = w[7], escape = w[8];

        if (aabb_hit(mn, mx, o, invd, ray.tmin, best.t)) {
            if (prim_count > 0) {
                leaf_test(prim_start, prim_count, ray, best);
                i = static_cast<std::uint32_t>(escape);
            } else {
                i = i + 1;
            }
        } else {
            i = static_cast<std::uint32_t>(escape);
        }
    }
    if (best.tri < 0) best.t = ray.tmax;  // miss sentinel
    return best;
}

void GpuBvh::leaf_test(int prim_start, int prim_count, const GpuRay& ray, GpuHit& best) const {
    const V3 o{ray.origin[0], ray.origin[1], ray.origin[2]};
    const V3 d{ray.dir[0], ray.dir[1], ray.dir[2]};
    for (int k = 0; k < prim_count; ++k) {
        const std::uint32_t p = prim_index_[prim_start + k];
        const float* tri9 = &tri_data_[std::size_t(p) * 9];
        float th, uh, vh;
        if (ray_tri(o, d, triVert(tri9, 0), triVert(tri9, 1), triVert(tri9, 2), ray.tmin,
                    ray.tmax, th, uh, vh) &&
            accept_hit(static_cast<int>(p), th, best.tri, best.t)) {
            best = GpuHit{static_cast<std::int32_t>(p), th, uh, vh};
        }
    }
}

bool GpuBvh::aabb_hit(const float* mn, const float* mx, const float* o, const float* invd,
                      float tmin, float tmax) {
    float tn = tmin, tf = tmax;
    for (int c = 0; c < 3; ++c) {
        const float t0 = (mn[c] - o[c]) * invd[c];
        const float t1 = (mx[c] - o[c]) * invd[c];
        tn = std::max(tn, std::min(t0, t1));
        tf = std::min(tf, std::max(t0, t1));
    }
    return tn <= tf;
}

std::vector<std::uint32_t> GpuBvh::cpu_frustum_pick(const Frustum& frustum) const {
    std::vector<std::uint32_t> picked;
    for (std::uint32_t i = 0; i < tri_count_; ++i) {
        if (box_in_frustum(&prim_aabb_[std::size_t(i) * 6], frustum)) picked.push_back(i);
    }
    return picked;
}

Result<std::vector<GpuHit>> GpuBvh::gpu_closest_hit(const std::vector<GpuRay>& rays) const {
    const std::uint32_t m = static_cast<std::uint32_t>(rays.size());
    std::vector<float> ray_data(std::size_t(m) * 8);
    for (std::uint32_t i = 0; i < m; ++i) {
        float* dst = &ray_data[std::size_t(i) * 8];
        std::memcpy(dst, rays[i].origin, 3 * sizeof(float));
        std::memcpy(dst + 3, rays[i].dir, 3 * sizeof(float));
        dst[6] = rays[i].tmin;
        dst[7] = rays[i].tmax;
    }

    MetalBackend& be = *backend_;
    BufferHandle ray_buf = be.makeSharedBuffer(ray_data.data(), ray_data.size() * sizeof(float));
    BufferHandle hit_buf = be.makeSharedBuffer(std::size_t(m) * 4 * sizeof(float));
    const std::uint32_t node_count = static_cast<std::uint32_t>(node_count_);
    BufferHandle nc = be.makeSharedBuffer(&node_count, sizeof(node_count));
    BufferHandle rc = be.makeSharedBuffer(&m, sizeof(m));
    auto pso = be.compilePipeline(kBvhMsl, "bvh_traverse");
    if (!pso) return pso.error();
    auto d = be.dispatch(pso.value(),
                         {node_buf_, prim_index_buf_, tri_buf_, ray_buf, hit_buf, nc, rc}, m);
    if (!d) return d.error();

    const auto* raw = static_cast<const float*>(be.bufferContents(hit_buf));
    if (raw == nullptr) return make_error("GpuBvh::closestHit: null hit buffer");
    std::vector<GpuHit> out(m);
    for (std::uint32_t i = 0; i < m; ++i) {
        const float* h = &raw[std::size_t(i) * 4];
        std::int32_t tri;
        std::memcpy(&tri, &h[0], sizeof(tri));
        out[i] = GpuHit{tri, h[1], h[2], h[3]};
        if (tri < 0) out[i].t = rays[i].tmax;
    }
    return out;
}

Result<std::vector<std::uint32_t>> GpuBvh::gpu_frustum_pick(const Frustum& frustum) const {
    const std::uint32_t n = static_cast<std::uint32_t>(tri_count_);
    MetalBackend& be = *backend_;
    BufferHandle plane_buf = be.makeSharedBuffer(&frustum, sizeof(frustum));  // 24 floats
    BufferHandle flag_buf = be.makeSharedBuffer(std::size_t(n) * sizeof(std::uint32_t));
    BufferHandle nc = be.makeSharedBuffer(&n, sizeof(n));
    auto pso = be.compilePipeline(kBvhMsl, "bvh_frustum");
    if (!pso) return pso.error();
    auto d = be.dispatch(pso.value(), {aabb_buf_, plane_buf, flag_buf, nc}, n);
    if (!d) return d.error();

    const auto flags = read_back<std::uint32_t>(be, flag_buf, n);
    std::vector<std::uint32_t> picked;
    for (std::uint32_t i = 0; i < n; ++i) {
        if (flags[i] != 0) picked.push_back(i);  // ascending order preserved
    }
    return picked;
}

}  // namespace cyber::metal
