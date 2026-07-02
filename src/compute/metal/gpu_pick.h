#ifndef CYBERCADKERNEL_COMPUTE_METAL_GPU_PICK_H
#define CYBERCADKERNEL_COMPUTE_METAL_GPU_PICK_H

// GPU picking (Phase 2, capability `spatial-acceleration`, `ComputeKind::Picking`).
//
// Given a camera ray (or a selection frustum) and a scene expressed as a triangle
// set, this module returns the picked triangle id + hit point on the Metal GPU.
// It is a self-contained, fp32, data-parallel numeric module:
//
//   * Ray-pick  — Moller-Trumbore ray/triangle intersection is evaluated for every
//     triangle in parallel on the GPU (one thread per (ray, triangle) pair); the
//     nearest hit is then resolved on the CPU with a fixed, documented tie-break so
//     the result is independent of GPU thread ordering.
//   * Frustum-pick — every triangle's fp32 AABB is tested against the 6 frustum
//     planes in parallel; the selected triangle ids are returned as a sorted set.
//
// Determinism: the observable result (nearest-hit id + point, or the sorted pick
// set) is reproducible across runs regardless of GPU scheduling — ray-pick ties in
// distance are broken by lowest triangle index within `kTieEpsilon`, and the
// frustum set is emitted sorted ascending.
//
// Precision: Apple GPUs are fp32-only. Picking is a display/interaction spatial
// query, not exact B-rep modeling, so fp32 is acceptable. The CPU reference oracle
// (`cpuPickReference` / `cpuFrustumReference`) uses the *identical* fp32 arithmetic
// so GPU-vs-CPU parity is exact at fp32 for the triangle-id decision and within
// `kPointTolerance` for the hit point. Exact fp64 modeling is never routed here.
//
// This module operates directly on a triangle set and therefore needs no OCCT and
// no prebuilt acceleration structure. It can be extended to consume a GPU LBVH
// (see `gpu_bvh.h`) as a traversal front-end without changing the API below; the
// brute-force triangle path is also the CPU-fallback / parity oracle.
//
// No `cc_*` ABI is involved: this is an internal C++ module the GPU test suite
// calls directly. The header is plain C++20 (no Objective-C / Metal types leak).

#include <cstdint>
#include <memory>
#include <vector>

#include "compute/metal/metal_backend.h"
#include "core/result.h"

namespace cyber::metal {

// ── Value types (POD, plain C++) ────────────────────────────────────────────

// A 3-component fp32 vector (position or direction).
struct Vec3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

// A single triangle given by its three fp32 vertices.
struct PickTriangle {
    Vec3f v0;
    Vec3f v1;
    Vec3f v2;
};

// The scene to pick against: an ordered triangle set. A triangle's id is its index
// in this vector (that index is what a pick returns).
struct PickScene {
    std::vector<PickTriangle> triangles;
};

// A pick ray. `dir` need not be normalized; the returned `t` is measured in units
// of `dir` (point = origin + t*dir). A hit is accepted only for t in [tmin, tmax].
struct PickRay {
    Vec3f origin;
    Vec3f dir;
    float tmin = 0.0f;
    float tmax = 1.0e30f;
};

// The result of a ray-pick. `primitive < 0` means the ray missed every triangle.
struct PickHit {
    std::int32_t primitive = -1;  // picked triangle id, or -1 on miss
    float t = 0.0f;               // ray parameter at the hit (origin + t*dir)
    Vec3f point;                  // hit point in world space

    bool hit() const { return primitive >= 0; }
};

// A selection frustum as 6 half-spaces. Each plane is (a, b, c, d) with the inside
// half-space defined by a*x + b*y + c*z + d >= 0 (normals point inward). A triangle
// is selected when its AABB intersects the frustum (conservative AABB-level test).
struct PickFrustum {
    // planes[i] = {a, b, c, d}. Order is unconstrained; all six must hold to be
    // inside. Typical order is left/right/bottom/top/near/far but any order works.
    float planes[6][4] = {};
};

// Tie-break epsilon (in ray-parameter units): two hits whose `t` differ by at most
// this are treated as equal-distance and resolved by lowest triangle index.
inline constexpr float kTieEpsilon = 1.0e-5f;

// Documented GPU-vs-CPU hit-point tolerance (world units) for parity assertions.
inline constexpr float kPointTolerance = 1.0e-3f;

// ── CPU brute-force reference oracle ────────────────────────────────────────
// These test every triangle with no acceleration structure, using the same fp32
// arithmetic as the GPU kernels. They are the parity oracle and the CPU-fallback
// implementation used when no Metal backend is available.

// Nearest-hit ray-pick over the whole triangle set (lowest-index tie-break).
PickHit cpuPickReference(const PickRay& ray, const PickScene& scene);

// Sorted (ascending) ids of triangles whose AABB intersects the frustum.
std::vector<std::int32_t> cpuFrustumReference(const PickFrustum& frustum,
                                              const PickScene& scene);

// ── GPU pick engine ─────────────────────────────────────────────────────────

// Runs picking on the Metal GPU using the shared MetalBackend (device, queue,
// runtime-compiled pipelines, unified-memory buffers). Construct via create():
// passing a null backend yields an engine that transparently runs the CPU
// reference path (so callers get identical answers with or without a GPU).
class GpuPick {
public:
    // Build a pick engine bound to `backend` (may be null => CPU-fallback engine).
    // Returns an Error only if pipeline compilation fails on a non-null backend.
    static Result<std::shared_ptr<GpuPick>> create(std::shared_ptr<MetalBackend> backend);

    // True when a live Metal backend drives the queries (false => CPU fallback).
    bool usesGpu() const { return backend_ != nullptr; }

    // Nearest-hit ray-pick. Deterministic: lowest-index tie-break within
    // kTieEpsilon. Matches cpuPickReference on the same scene/ray.
    Result<PickHit> pick(const PickRay& ray, const PickScene& scene);

    // Batched nearest-hit ray-pick: one PickHit per input ray, same ordering.
    Result<std::vector<PickHit>> pick(const std::vector<PickRay>& rays,
                                      const PickScene& scene);

    // Frustum-pick: sorted (ascending) ids of triangles whose AABB intersects the
    // frustum. Matches cpuFrustumReference on the same scene/frustum.
    Result<std::vector<std::int32_t>> pickFrustum(const PickFrustum& frustum,
                                                  const PickScene& scene);

private:
    explicit GpuPick(std::shared_ptr<MetalBackend> backend,
                     PipelineHandle rayPipe,
                     PipelineHandle frustumPipe);

    std::shared_ptr<MetalBackend> backend_;  // null => CPU fallback
    PipelineHandle rayPipe_ = nullptr;
    PipelineHandle frustumPipe_ = nullptr;
};

// Self-test used to gate the module: builds a tiny scene, runs a GPU ray-pick and a
// frustum-pick, and verifies both equal the CPU reference. Returns false (no crash)
// when Metal is unavailable or any step fails.
bool gpu_pick_selftest();

}  // namespace cyber::metal

#endif  // CYBERCADKERNEL_COMPUTE_METAL_GPU_PICK_H
