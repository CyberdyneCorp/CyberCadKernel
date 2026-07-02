#ifndef CYBERCADKERNEL_COMPUTE_METAL_GPU_BVH_H
#define CYBERCADKERNEL_COMPUTE_METAL_GPU_BVH_H

// GPU LBVH (linear bounding-volume hierarchy) over a triangle set, plus a
// stackless GPU ray-traversal nearest-hit query and a GPU frustum pick — the
// `spatial-acceleration` capability's Metal implementation (Phase 2).
//
// Pipeline (all fp32; Apple GPUs have no fp64):
//   1. GPU kernel computes each triangle's AABB + centroid.
//   2. CPU reduces the centroid bounds (a tiny serial pass — deterministic).
//   3. GPU kernel quantizes each centroid to a 30-bit Morton code.
//   4. CPU sorts primitive indices by (Morton code, primitive index) and emits a
//      linear, DFS-preorder node array with per-node "escape" (skip) indices —
//      the documented CPU-side hierarchy build. Sorting by (code, index) makes
//      the node array bit-for-bit identical across runs (fixed tie-break).
//   5. GPU kernel traverses that linear array stacklessly for the nearest hit of
//      a batch of rays; a second GPU kernel tests per-primitive AABBs against a
//      selection frustum.
//
// Why this split: steps 1/3/5 are the embarrassingly parallel numeric work the
// GPU is for; the sort + hierarchy (step 4) is small, serial, and trivially
// deterministic on the CPU. The spec allows "GPU Morton + a documented build",
// which this is.
//
// This header is plain C++20 (no Metal/Objective-C types leak in; the backend is
// only referenced through the opaque handles in metal_backend.h), so it compiles
// in any translation unit. The implementation lives in gpu_bvh.mm and is built
// only when CYBERCAD_HAS_METAL is set.
//
// Backend routing / CPU fallback: pass a MetalBackend to build() to run on the
// GPU, or a null backend to run the identical fp32 algorithm on the CPU. Both
// paths use the SAME fp32 arithmetic and the SAME tie-break, so a query returns
// the same answer with or without a GPU — and that answer equals the brute-force
// oracle (closestHitBruteForce / frustumPickBruteForce) below.
//
// Determinism / tie-break (documented, applied identically on GPU, CPU, and in
// the brute-force oracle):
//   * Nearest hit: the smallest ray parameter t wins. Two hits whose t agree
//     within tie_eps(t) = 1e-4 * max(1, |t|) are treated as equal distance and
//     the LOWER primitive index wins. tie_eps only absorbs fp32 rounding of
//     genuinely equal-distance hits; distinct hits in a scene must differ by more
//     than tie_eps for GPU/CPU/oracle parity to be exact.
//   * Frustum pick: the result is the ascending-sorted list of primitive indices
//     whose AABB intersects the frustum, so it is independent of GPU thread order.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "compute/metal/metal_backend.h"
#include "core/result.h"

namespace cyber::metal {

// A triangle: three fp32 vertices, tightly packed (matches the GPU buffer layout).
struct GpuTriangle {
    float v0[3];
    float v1[3];
    float v2[3];
};

// A ray: origin + direction (need not be normalized) and the valid t interval
// [tmin, tmax] a hit must fall within.
struct GpuRay {
    float origin[3];
    float dir[3];
    float tmin;
    float tmax;
};

// A nearest-hit result. `tri < 0` means the ray missed every triangle; otherwise
// `tri` is the original (input-order) primitive index, `t` the ray parameter at
// the hit, and (u, v) its barycentric coordinates. The hit point is
// origin + t * dir.
struct GpuHit {
    std::int32_t tri = -1;
    float t = 0.0f;
    float u = 0.0f;
    float v = 0.0f;
};

// A selection frustum as 6 half-space planes. A point (x,y,z) is inside plane p
// when planes[p][0]*x + planes[p][1]*y + planes[p][2]*z + planes[p][3] >= 0. A
// primitive is picked when its AABB is not fully outside any of the 6 planes.
struct Frustum {
    float planes[6][4];
};

// A linear BVH over a triangle set plus its GPU-accelerated queries. Build once
// with build(); query many times with closestHit()/frustumPick(). Copyable — GPU
// buffers are owned by the backend, so copies share them safely.
class GpuBvh {
public:
    // Build the LBVH over `triangles`. When `backend` is non-null the AABB/Morton
    // stages and later queries run on that Metal backend's GPU; when null the
    // identical fp32 algorithm runs on the CPU (the routing/fallback contract).
    // Fails (Error) on an empty triangle set or a GPU step error — never crashes.
    static Result<GpuBvh> build(std::shared_ptr<MetalBackend> backend,
                                const std::vector<GpuTriangle>& triangles);

    // Nearest triangle hit for a batch of rays: result[i] corresponds to rays[i].
    // Runs the stackless traversal on the GPU when this BVH was built with a
    // backend, else on the CPU. Deterministic (see the tie-break above).
    Result<std::vector<GpuHit>> closestHit(const std::vector<GpuRay>& rays) const;

    // Convenience single-ray nearest hit.
    Result<GpuHit> closestHit(const GpuRay& ray) const;

    // Ascending-sorted indices of primitives whose AABB intersects `frustum`.
    Result<std::vector<std::uint32_t>> frustumPick(const Frustum& frustum) const;

    std::size_t triangleCount() const { return tri_count_; }
    std::size_t nodeCount() const { return node_count_; }
    bool onGpu() const { return backend_ != nullptr; }

    // ── CPU brute-force oracles (no BVH) ─────────────────────────────────────
    // Test every triangle with the SAME fp32 arithmetic and tie-break the GPU
    // path uses; the parity test asserts the GPU BVH answer equals these.
    static std::vector<GpuHit> closestHitBruteForce(
        const std::vector<GpuTriangle>& triangles, const std::vector<GpuRay>& rays);
    static std::vector<std::uint32_t> frustumPickBruteForce(
        const std::vector<GpuTriangle>& triangles, const Frustum& frustum);

private:
    GpuBvh() = default;

    // CPU-fallback query path (also the shape the GPU kernels mirror).
    static void cpu_primitive(const float* tri9, float* aabb6, float* centroid3);
    std::vector<GpuHit> cpu_closest_hit(const std::vector<GpuRay>& rays) const;
    GpuHit traverse_one(const GpuRay& ray) const;
    void leaf_test(int prim_start, int prim_count, const GpuRay& ray, GpuHit& best) const;
    static bool aabb_hit(const float* mn, const float* mx, const float* o, const float* invd,
                         float tmin, float tmax);
    std::vector<std::uint32_t> cpu_frustum_pick(const Frustum& frustum) const;

    // GPU dispatch path.
    Result<std::vector<GpuHit>> gpu_closest_hit(const std::vector<GpuRay>& rays) const;
    Result<std::vector<std::uint32_t>> gpu_frustum_pick(const Frustum& frustum) const;

    std::shared_ptr<MetalBackend> backend_;  // null => CPU-only BVH
    std::size_t tri_count_ = 0;
    std::size_t node_count_ = 0;

    // GPU buffers (owned by the backend; non-null only when backend_ is set).
    BufferHandle tri_buf_ = nullptr;       // 9 floats per triangle
    BufferHandle node_buf_ = nullptr;      // 9 words per node (see gpu_bvh.mm)
    BufferHandle prim_index_buf_ = nullptr;  // uint per primitive (Morton order)
    BufferHandle aabb_buf_ = nullptr;      // 6 floats per primitive (min,max)

    // CPU-side copies backing the CPU fallback path and buffer construction.
    std::vector<float> tri_data_;             // 9 * tri_count_
    std::vector<std::int32_t> node_words_;    // 9 * node_count_ (bit-packed)
    std::vector<std::uint32_t> prim_index_;   // sorted primitive indices
    std::vector<float> prim_aabb_;            // 6 * tri_count_
};

}  // namespace cyber::metal

#endif  // CYBERCADKERNEL_COMPUTE_METAL_GPU_BVH_H
