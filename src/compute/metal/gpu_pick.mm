// GPU picking implementation (Objective-C++, iOS-only).
//
// Compiled only when CYBERCAD_HAS_METAL is set (see CMakeLists.txt), with ARC and
// -framework Metal -framework Foundation. Objective-C / Metal types stay in this
// translation unit; the header (gpu_pick.h) trades only plain C++20 POD types.
//
// Two MSL kernels are embedded as string literals and compiled at runtime through
// the shared MetalBackend (no .metallib precompile step):
//   * cc_pick_ray_batch  — Moller-Trumbore ray/triangle intersection, one thread
//     per (ray, triangle) pair, writing per-pair hit distance (or +INF on miss).
//   * cc_pick_frustum     — per-triangle AABB-vs-frustum test, one thread per
//     triangle, writing a 0/1 selection flag.
// The nearest-hit reduction and the frustum set-sort run on the CPU with a fixed
// tie-break, so results are independent of GPU thread ordering. The exact same
// fp32 arithmetic backs the CPU reference oracle, giving exact-at-fp32 parity.

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include "compute/metal/gpu_pick.h"
#include "compute/metal/metal_backend.h"
#include "core/compute_backend.h"
#include "core/result.h"

namespace cyber::metal {
namespace {

// ── Embedded MSL kernels ────────────────────────────────────────────────────
// Buffers are flat float/uint arrays (not float3) to keep C++/MSL memory layout
// identical and free of vector alignment padding.

constexpr const char* kPickKernels = R"MSL(
#include <metal_stdlib>
using namespace metal;

// Vertices: 9 floats per triangle (v0.xyz, v1.xyz, v2.xyz).
// Rays:     8 floats per ray (origin.xyz, dir.xyz, tmin, tmax).
// params:   {numRays, numTris}.
// out:      numRays*numTris floats; out[r*numTris + t] = hit distance or +INF.
kernel void cc_pick_ray_batch(device const float* verts  [[buffer(0)]],
                              device const float* rays   [[buffer(1)]],
                              device float*       out    [[buffer(2)]],
                              constant uint2&     params [[buffer(3)]],
                              uint gid [[thread_position_in_grid]]) {
    uint numTris = params.y;
    if (numTris == 0u) { return; }
    uint r = gid / numTris;
    if (r >= params.x) { return; }
    uint t = gid - r * numTris;

    uint rb = r * 8u;
    float3 o = float3(rays[rb + 0u], rays[rb + 1u], rays[rb + 2u]);
    float3 d = float3(rays[rb + 3u], rays[rb + 4u], rays[rb + 5u]);
    float tmin = rays[rb + 6u];
    float tmax = rays[rb + 7u];

    uint vb = t * 9u;
    float3 v0 = float3(verts[vb + 0u], verts[vb + 1u], verts[vb + 2u]);
    float3 v1 = float3(verts[vb + 3u], verts[vb + 4u], verts[vb + 5u]);
    float3 v2 = float3(verts[vb + 6u], verts[vb + 7u], verts[vb + 8u]);

    float3 e1 = v1 - v0;
    float3 e2 = v2 - v0;
    float3 pv = cross(d, e2);
    float det = dot(e1, pv);

    float hit = INFINITY;
    if (fabs(det) > 1e-8) {
        float inv = 1.0 / det;
        float3 tv = o - v0;
        float u = dot(tv, pv) * inv;
        if (u >= 0.0 && u <= 1.0) {
            float3 qv = cross(tv, e1);
            float v = dot(d, qv) * inv;
            if (v >= 0.0 && (u + v) <= 1.0) {
                float tt = dot(e2, qv) * inv;
                if (tt >= tmin && tt <= tmax) { hit = tt; }
            }
        }
    }
    out[gid] = hit;
}

// planes: 24 floats (6 planes * {a,b,c,d}); inside is a*x+b*y+c*z+d >= 0.
// out:    numTris uints; 1 if the triangle AABB intersects the frustum, else 0.
kernel void cc_pick_frustum(device const float* verts  [[buffer(0)]],
                            device const float* planes [[buffer(1)]],
                            device uint*        out    [[buffer(2)]],
                            constant uint&      numTris [[buffer(3)]],
                            uint gid [[thread_position_in_grid]]) {
    if (gid >= numTris) { return; }
    uint vb = gid * 9u;
    float3 v0 = float3(verts[vb + 0u], verts[vb + 1u], verts[vb + 2u]);
    float3 v1 = float3(verts[vb + 3u], verts[vb + 4u], verts[vb + 5u]);
    float3 v2 = float3(verts[vb + 6u], verts[vb + 7u], verts[vb + 8u]);
    float3 lo = min(v0, min(v1, v2));
    float3 hi = max(v0, max(v1, v2));

    uint inside = 1u;
    for (uint p = 0u; p < 6u; ++p) {
        uint pb = p * 4u;
        float a = planes[pb + 0u];
        float b = planes[pb + 1u];
        float c = planes[pb + 2u];
        float dd = planes[pb + 3u];
        // Most-positive AABB vertex along the (inward) plane normal.
        float px = (a >= 0.0) ? hi.x : lo.x;
        float py = (b >= 0.0) ? hi.y : lo.y;
        float pz = (c >= 0.0) ? hi.z : lo.z;
        if (a * px + b * py + c * pz + dd < 0.0) { inside = 0u; break; }
    }
    out[gid] = inside;
}
)MSL";

constexpr float kInf = std::numeric_limits<float>::infinity();

// ── fp32 CPU math shared by the reference oracle (mirrors the MSL kernels) ───

struct V3 {
    float x, y, z;
};
inline V3 sub(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline float dot3(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3 cross3(const V3& a, const V3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline V3 toV3(const Vec3f& v) { return {v.x, v.y, v.z}; }

// Moller-Trumbore in fp32; returns hit distance or +INF on a miss. Bit-for-bit the
// same operation sequence as the cc_pick_ray_batch kernel.
float rayTriangle(const PickRay& ray, const PickTriangle& tri) {
    const V3 o = toV3(ray.origin);
    const V3 d = toV3(ray.dir);
    const V3 v0 = toV3(tri.v0);
    const V3 e1 = sub(toV3(tri.v1), v0);
    const V3 e2 = sub(toV3(tri.v2), v0);
    const V3 pv = cross3(d, e2);
    const float det = dot3(e1, pv);
    if (std::fabs(det) <= 1e-8f) {
        return kInf;
    }
    const float inv = 1.0f / det;
    const V3 tv = sub(o, v0);
    const float u = dot3(tv, pv) * inv;
    if (u < 0.0f || u > 1.0f) {
        return kInf;
    }
    const V3 qv = cross3(tv, e1);
    const float v = dot3(d, qv) * inv;
    if (v < 0.0f || (u + v) > 1.0f) {
        return kInf;
    }
    const float tt = dot3(e2, qv) * inv;
    if (tt < ray.tmin || tt > ray.tmax) {
        return kInf;
    }
    return tt;
}

// Deterministic nearest-hit reduction over per-triangle distances: smallest finite
// distance wins; ties within kTieEpsilon go to the lowest triangle index (the scan
// runs ascending, so the earliest index is retained on a near-tie).
PickHit reduceNearest(const std::vector<float>& dist, const PickRay& ray) {
    PickHit best;
    float bestT = kInf;
    for (std::size_t i = 0; i < dist.size(); ++i) {
        const float t = dist[i];
        if (std::isfinite(t) && t < bestT - kTieEpsilon) {
            bestT = t;
            best.primitive = static_cast<std::int32_t>(i);
            best.t = t;
        }
    }
    if (best.primitive >= 0) {
        best.point = {ray.origin.x + bestT * ray.dir.x,
                      ray.origin.y + bestT * ray.dir.y,
                      ray.origin.z + bestT * ray.dir.z};
    }
    return best;
}

// fp32 AABB-vs-frustum test mirroring cc_pick_frustum.
bool frustumIntersectsAabb(const PickFrustum& f, const V3& lo, const V3& hi) {
    for (int p = 0; p < 6; ++p) {
        const float a = f.planes[p][0];
        const float b = f.planes[p][1];
        const float c = f.planes[p][2];
        const float dd = f.planes[p][3];
        const float px = (a >= 0.0f) ? hi.x : lo.x;
        const float py = (b >= 0.0f) ? hi.y : lo.y;
        const float pz = (c >= 0.0f) ? hi.z : lo.z;
        if (a * px + b * py + c * pz + dd < 0.0f) {
            return false;
        }
    }
    return true;
}

// Pack a scene into the flat 9-floats-per-triangle vertex layout the kernels read.
std::vector<float> packVertices(const PickScene& scene) {
    std::vector<float> v;
    v.reserve(scene.triangles.size() * 9);
    for (const PickTriangle& t : scene.triangles) {
        v.insert(v.end(), {t.v0.x, t.v0.y, t.v0.z, t.v1.x, t.v1.y, t.v1.z,
                           t.v2.x, t.v2.y, t.v2.z});
    }
    return v;
}

std::vector<float> packRays(const std::vector<PickRay>& rays) {
    std::vector<float> r;
    r.reserve(rays.size() * 8);
    for (const PickRay& ray : rays) {
        r.insert(r.end(), {ray.origin.x, ray.origin.y, ray.origin.z, ray.dir.x,
                           ray.dir.y, ray.dir.z, ray.tmin, ray.tmax});
    }
    return r;
}

}  // namespace

// ── CPU reference oracle (public) ───────────────────────────────────────────

PickHit cpuPickReference(const PickRay& ray, const PickScene& scene) {
    std::vector<float> dist(scene.triangles.size(), kInf);
    for (std::size_t i = 0; i < scene.triangles.size(); ++i) {
        dist[i] = rayTriangle(ray, scene.triangles[i]);
    }
    return reduceNearest(dist, ray);
}

std::vector<std::int32_t> cpuFrustumReference(const PickFrustum& frustum,
                                              const PickScene& scene) {
    std::vector<std::int32_t> out;
    for (std::size_t i = 0; i < scene.triangles.size(); ++i) {
        const PickTriangle& t = scene.triangles[i];
        const V3 a = toV3(t.v0), b = toV3(t.v1), c = toV3(t.v2);
        const V3 lo{std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}),
                    std::min({a.z, b.z, c.z})};
        const V3 hi{std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}),
                    std::max({a.z, b.z, c.z})};
        if (frustumIntersectsAabb(frustum, lo, hi)) {
            out.push_back(static_cast<std::int32_t>(i));
        }
    }
    return out;  // already ascending (scan order)
}

// ── GpuPick ─────────────────────────────────────────────────────────────────

GpuPick::GpuPick(std::shared_ptr<MetalBackend> backend, PipelineHandle rayPipe,
                 PipelineHandle frustumPipe)
    : backend_(std::move(backend)), rayPipe_(rayPipe), frustumPipe_(frustumPipe) {}

Result<std::shared_ptr<GpuPick>> GpuPick::create(std::shared_ptr<MetalBackend> backend) {
    if (!backend) {
        // CPU-fallback engine: no pipelines, queries route to the reference path.
        return std::shared_ptr<GpuPick>(new GpuPick(nullptr, nullptr, nullptr));
    }
    auto rayPipe = backend->compilePipeline(kPickKernels, "cc_pick_ray_batch");
    if (!rayPipe) {
        return rayPipe.error();
    }
    auto frustumPipe = backend->compilePipeline(kPickKernels, "cc_pick_frustum");
    if (!frustumPipe) {
        return frustumPipe.error();
    }
    return std::shared_ptr<GpuPick>(
        new GpuPick(std::move(backend), rayPipe.value(), frustumPipe.value()));
}

Result<PickHit> GpuPick::pick(const PickRay& ray, const PickScene& scene) {
    auto batch = pick(std::vector<PickRay>{ray}, scene);
    if (!batch) {
        return batch.error();
    }
    return batch.value().front();
}

Result<std::vector<PickHit>> GpuPick::pick(const std::vector<PickRay>& rays,
                                           const PickScene& scene) {
    // CPU fallback (or trivial empty inputs): use the reference oracle directly.
    if (!backend_ || scene.triangles.empty() || rays.empty()) {
        std::vector<PickHit> hits;
        hits.reserve(rays.size());
        for (const PickRay& r : rays) {
            hits.push_back(cpuPickReference(r, scene));
        }
        return hits;
    }

    const std::size_t numTris = scene.triangles.size();
    const std::size_t numRays = rays.size();
    const std::vector<float> verts = packVertices(scene);
    const std::vector<float> rayData = packRays(rays);
    const std::uint32_t params[2] = {static_cast<std::uint32_t>(numRays),
                                     static_cast<std::uint32_t>(numTris)};

    BufferHandle vBuf = backend_->makeSharedBuffer(verts.data(), verts.size() * sizeof(float));
    BufferHandle rBuf = backend_->makeSharedBuffer(rayData.data(), rayData.size() * sizeof(float));
    BufferHandle oBuf = backend_->makeSharedBuffer(numRays * numTris * sizeof(float));
    BufferHandle pBuf = backend_->makeSharedBuffer(params, sizeof(params));
    if (!vBuf || !rBuf || !oBuf || !pBuf) {
        return make_error("gpu_pick: shared buffer allocation failed");
    }

    auto disp = backend_->dispatch(rayPipe_, {vBuf, rBuf, oBuf, pBuf}, numRays * numTris);
    if (!disp) {
        return disp.error();
    }

    const auto* out = static_cast<const float*>(backend_->bufferContents(oBuf));
    if (out == nullptr) {
        return make_error("gpu_pick: null GPU output buffer");
    }

    std::vector<PickHit> hits;
    hits.reserve(numRays);
    std::vector<float> dist(numTris);
    for (std::size_t r = 0; r < numRays; ++r) {
        std::copy_n(out + r * numTris, numTris, dist.begin());
        hits.push_back(reduceNearest(dist, rays[r]));
    }
    return hits;
}

Result<std::vector<std::int32_t>> GpuPick::pickFrustum(const PickFrustum& frustum,
                                                       const PickScene& scene) {
    if (!backend_ || scene.triangles.empty()) {
        return cpuFrustumReference(frustum, scene);
    }

    const std::size_t numTris = scene.triangles.size();
    const std::vector<float> verts = packVertices(scene);
    const std::uint32_t nTris = static_cast<std::uint32_t>(numTris);

    BufferHandle vBuf = backend_->makeSharedBuffer(verts.data(), verts.size() * sizeof(float));
    BufferHandle plBuf = backend_->makeSharedBuffer(&frustum.planes[0][0], 24 * sizeof(float));
    BufferHandle oBuf = backend_->makeSharedBuffer(numTris * sizeof(std::uint32_t));
    BufferHandle nBuf = backend_->makeSharedBuffer(&nTris, sizeof(nTris));
    if (!vBuf || !plBuf || !oBuf || !nBuf) {
        return make_error("gpu_pick: shared buffer allocation failed");
    }

    auto disp = backend_->dispatch(frustumPipe_, {vBuf, plBuf, oBuf, nBuf}, numTris);
    if (!disp) {
        return disp.error();
    }

    const auto* flags = static_cast<const std::uint32_t*>(backend_->bufferContents(oBuf));
    if (flags == nullptr) {
        return make_error("gpu_pick: null GPU output buffer");
    }

    std::vector<std::int32_t> out;
    for (std::size_t i = 0; i < numTris; ++i) {
        if (flags[i] != 0u) {
            out.push_back(static_cast<std::int32_t>(i));  // ascending scan => sorted
        }
    }
    return out;
}

// ── Self-test ───────────────────────────────────────────────────────────────

bool gpu_pick_selftest() {
    auto backend = MetalBackend::create();
    if (!backend) {
        return false;
    }
    auto engine = GpuPick::create(backend);
    if (!engine) {
        return false;
    }

    // Two parallel triangles in front of the origin; the ray must hit the nearer.
    PickScene scene;
    scene.triangles.push_back({{-1, -1, 5}, {1, -1, 5}, {0, 1, 5}});   // far
    scene.triangles.push_back({{-1, -1, 2}, {1, -1, 2}, {0, 1, 2}});   // near
    PickRay ray;
    ray.origin = {0, 0, 0};
    ray.dir = {0, 0, 1};

    auto gpuHit = engine.value()->pick(ray, scene);
    if (!gpuHit) {
        return false;
    }
    const PickHit ref = cpuPickReference(ray, scene);
    const PickHit& g = gpuHit.value();
    if (g.primitive != ref.primitive || ref.primitive != 1) {
        return false;
    }
    if (std::fabs(g.point.z - ref.point.z) > kPointTolerance) {
        return false;
    }

    // Frustum enclosing z in [1,3] must select only the near triangle.
    PickFrustum f{};
    f.planes[0][0] = 1;  f.planes[0][3] = 10;   // x >= -10
    f.planes[1][0] = -1; f.planes[1][3] = 10;   // x <= 10
    f.planes[2][1] = 1;  f.planes[2][3] = 10;   // y >= -10
    f.planes[3][1] = -1; f.planes[3][3] = 10;   // y <= 10
    f.planes[4][2] = 1;  f.planes[4][3] = -1;   // z >= 1
    f.planes[5][2] = -1; f.planes[5][3] = 3;    // z <= 3

    auto gpuSet = engine.value()->pickFrustum(f, scene);
    if (!gpuSet) {
        return false;
    }
    return gpuSet.value() == cpuFrustumReference(f, scene);
}

}  // namespace cyber::metal
