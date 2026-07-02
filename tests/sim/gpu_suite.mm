// Integrated GPU-vs-CPU parity suite for CyberCadKernel Phase 2.
//
// Runs inside a booted iOS simulator via `xcrun simctl spawn` (see
// scripts/run-sim-gpu-suite.sh), where MTLCreateSystemDefaultDevice() returns the
// "Apple iOS simulator GPU" and newLibraryWithSource: compiles MSL at runtime.
// The suite is OCCT-FREE: it links only the Metal GPU modules, their Phase-0
// dependency (CpuComputeBackend / ComputeRegistry, compute_backend.cpp), and
// Metal/Foundation. Every arithmetic path here is fp32 — Apple GPUs have no fp64,
// and the exact fp64 modeling core stays on the CPU (never routed to the GPU).
//
// For each Phase-2 GPU module it dispatches the real Metal kernel and asserts the
// GPU result matches an INDEPENDENT CPU reference within a documented fp32
// tolerance, printing "[GPU] PASS/FAIL <label>" per check. main() prints a final
// "== N passed, M failed ==" line, flushes, and std::_Exit()s (0 iff no failures)
// so a hung GPU teardown can never mask the result.
//
// Module coverage (this TU): surface-eval (Bezier grid vs de Casteljau), BVH +
// closestHit (vs brute force), mesh-post normals (vs CPU reference). Picking lives
// in the sibling gpu_pick_check.mm because gpu_pick.h and gpu_surface_eval.h each
// define a `cyber::metal::Vec3f` and cannot share a translation unit; its checks
// are invoked through run_pick_checks() below.

#import <Foundation/Foundation.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

#include "compute/metal/gpu_bvh.h"
#include "compute/metal/gpu_mesh_post.h"
#include "compute/metal/gpu_surface_eval.h"
#include "compute/metal/metal_backend.h"
#include "core/result.h"
#include "gpu_check.h"

using cyber::metal::MetalBackend;
using gpusuite::Counts;
using gpusuite::gpu_check;

namespace {

// ── surface-eval: bicubic Bezier grid on GPU vs an independent de Casteljau ──
//
// A bicubic Bezier patch is the clamped-knot special case of the Cox-de Boor
// evaluator the GPU kernel runs. We evaluate the same patch two ways — on the GPU
// (evaluateSurfaceGridGPU) and with a from-scratch de Casteljau reference here —
// and require component-wise agreement within the module's documented fp32
// tolerance (approxEqual: |a-b| <= 1e-4 + 1e-4*|b|). Sampling at explicit,
// shared parameters removes any ambiguity about the grid's parameter spacing.

// One de Casteljau step chain over four scalar control values at parameter t.
float bezier1D(float p0, float p1, float p2, float p3, float t) {
    const float q0 = p0 + (p1 - p0) * t;
    const float q1 = p1 + (p2 - p1) * t;
    const float q2 = p2 + (p3 - p2) * t;
    const float r0 = q0 + (q1 - q0) * t;
    const float r1 = q1 + (q2 - q1) * t;
    return r0 + (r1 - r0) * t;
}

// Independent bicubic (degree 3x3) de Casteljau evaluation of the control net at
// (u, v). `cp` is row-major with u as the major axis: cp[i*4 + j], matching
// SurfaceDef's layout. Non-rational (w == 1), so no perspective divide is needed.
cyber::metal::Vec3f deCasteljauBicubic(const std::vector<cyber::metal::ControlPoint>& cp,
                                       float u, float v) {
    // First collapse each u-row across v, then collapse the four results across u.
    float cx[4];
    float cy[4];
    float cz[4];
    for (int i = 0; i < 4; ++i) {
        const auto& a = cp[i * 4 + 0];
        const auto& b = cp[i * 4 + 1];
        const auto& c = cp[i * 4 + 2];
        const auto& d = cp[i * 4 + 3];
        cx[i] = bezier1D(a.x, b.x, c.x, d.x, v);
        cy[i] = bezier1D(a.y, b.y, c.y, d.y, v);
        cz[i] = bezier1D(a.z, b.z, c.z, d.z, v);
    }
    cyber::metal::Vec3f out;
    out.x = bezier1D(cx[0], cx[1], cx[2], cx[3], u);
    out.y = bezier1D(cy[0], cy[1], cy[2], cy[3], u);
    out.z = bezier1D(cz[0], cz[1], cz[2], cz[3], u);
    return out;
}

void run_surface_checks(Counts& c, MetalBackend& be) {
    using namespace cyber::metal;

    // A known, non-planar (saddle-ish) 4x4 control net; w == 1 => polynomial.
    SurfaceDef surf;
    surf.degreeU = 3;
    surf.degreeV = 3;
    surf.numU = 4;
    surf.numV = 4;
    surf.control.resize(16);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            ControlPoint cp;
            cp.x = static_cast<float>(i);
            cp.y = static_cast<float>(j);
            cp.z = (static_cast<float>(i) - 1.5f) * (static_cast<float>(j) - 1.5f) * 0.75f;
            cp.w = 1.0f;
            surf.control[i * 4 + j] = cp;
        }
    }
    surf.knotsU = makeClampedKnots(surf.numU, surf.degreeU);  // [0,0,0,0,1,1,1,1]
    surf.knotsV = makeClampedKnots(surf.numV, surf.degreeV);

    // Sample on an explicit 5x5 grid so the reference uses identical parameters.
    GridRequest req;
    req.gridU = 5;
    req.gridV = 5;
    req.computeNormals = false;
    req.paramsU = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};
    req.paramsV = {0.0f, 0.25f, 0.5f, 0.75f, 1.0f};

    auto gpu = evaluateSurfaceGridGPU(be, surf, req);
    gpu_check(c, gpu.has_value(), "surface-eval: GPU Bezier grid evaluates");
    if (!gpu.has_value()) {
        return;
    }
    const SurfaceGrid& grid = gpu.value();
    gpu_check(c,
              grid.gridU == req.gridU && grid.gridV == req.gridV &&
                  grid.points.size() == static_cast<std::size_t>(req.gridU * req.gridV),
              "surface-eval: grid dimensions match request");

    bool all_match = grid.points.size() == static_cast<std::size_t>(req.gridU * req.gridV);
    for (int i = 0; i < req.gridU && all_match; ++i) {
        for (int j = 0; j < req.gridV && all_match; ++j) {
            const Vec3f ref = deCasteljauBicubic(surf.control, req.paramsU[i], req.paramsV[j]);
            const Vec3f got = grid.points[i * req.gridV + j];
            all_match = approxEqual(got, ref);
        }
    }
    gpu_check(c, all_match,
              "surface-eval: GPU grid matches de Casteljau reference (fp32 rel ~1e-4)");
}

// ── BVH + closestHit: GPU LBVH traversal vs CPU brute-force nearest hit ──────
//
// A deterministic triangle set (five coplanar quadrant-triangles stacked in z,
// plus one off to the side) and a batch of rays exercising nearest-hit selection
// among overlapping candidates and clean misses. The GPU LBVH's closestHit is
// compared against GpuBvh::closestHitBruteForce, which shares the exact fp32
// arithmetic and tie-break, so agreement is expected on triangle id and on t
// within a small fp32 tolerance. Distinct hits are spaced far apart in z so no
// comparison lands inside the documented tie epsilon.

cyber::metal::GpuTriangle makeTri(float ax, float ay, float az, float bx, float by, float bz,
                                  float cx, float cy, float cz) {
    cyber::metal::GpuTriangle t;
    t.v0[0] = ax; t.v0[1] = ay; t.v0[2] = az;
    t.v1[0] = bx; t.v1[1] = by; t.v1[2] = bz;
    t.v2[0] = cx; t.v2[1] = cy; t.v2[2] = cz;
    return t;
}

cyber::metal::GpuRay makeRay(float ox, float oy, float oz, float dx, float dy, float dz) {
    cyber::metal::GpuRay r;
    r.origin[0] = ox; r.origin[1] = oy; r.origin[2] = oz;
    r.dir[0] = dx; r.dir[1] = dy; r.dir[2] = dz;
    r.tmin = 0.0f;
    r.tmax = 1.0e30f;
    return r;
}

void run_bvh_checks(Counts& c, std::shared_ptr<MetalBackend> be) {
    using namespace cyber::metal;

    // Triangles 0..4: lower-left unit triangle (covers x+y<=1) at z = 1..5.
    // Triangle 5: off to the side at z = 1 (never on the shared column of rays).
    std::vector<GpuTriangle> tris;
    for (int k = 1; k <= 5; ++k) {
        const float z = static_cast<float>(k);
        tris.push_back(makeTri(0, 0, z, 1, 0, z, 0, 1, z));
    }
    tris.push_back(makeTri(10, 10, 1, 11, 10, 1, 10, 11, 1));

    // Rays: down through the stack (nearest = z1), up through it (nearest = z5),
    // starting between planes (nearest = z3), a lateral miss, and a ray aimed away.
    std::vector<GpuRay> rays = {
        makeRay(0.3f, 0.3f, -1.0f, 0, 0, 1),   // -> tri 0 (z=1)
        makeRay(0.3f, 0.3f, 10.0f, 0, 0, -1),  // -> tri 4 (z=5)
        makeRay(0.3f, 0.3f, 2.5f, 0, 0, 1),    // -> tri 2 (z=3)
        makeRay(5.0f, 5.0f, -1.0f, 0, 0, 1),   // -> miss
        makeRay(0.3f, 0.3f, -1.0f, 0, 0, -1),  // -> miss (points away)
    };

    auto built = GpuBvh::build(be, tris);
    gpu_check(c, built.has_value(), "bvh: LBVH builds on GPU");
    if (!built.has_value()) {
        return;
    }
    const GpuBvh& bvh = built.value();
    gpu_check(c, bvh.onGpu(), "bvh: query path is bound to the Metal backend");

    auto gpuHits = bvh.closestHit(rays);
    gpu_check(c, gpuHits.has_value(), "bvh: GPU closestHit batch succeeds");
    if (!gpuHits.has_value()) {
        return;
    }
    const std::vector<GpuHit>& g = gpuHits.value();
    const std::vector<GpuHit> ref = GpuBvh::closestHitBruteForce(tris, rays);

    bool ok = g.size() == rays.size() && ref.size() == rays.size();
    for (std::size_t i = 0; i < rays.size() && ok; ++i) {
        const bool sameId = g[i].tri == ref[i].tri;
        const bool sameT = (g[i].tri < 0) ||
                           std::fabs(g[i].t - ref[i].t) <= 1e-4f * std::max(1.0f, std::fabs(ref[i].t));
        ok = sameId && sameT;
    }
    gpu_check(c, ok, "bvh: GPU nearest hit matches brute force (same id + t within fp32 tol)");

    // Sanity on the expected structure of the deterministic scene.
    gpu_check(c,
              g.size() == rays.size() && g[0].tri == 0 && g[1].tri == 4 && g[2].tri == 2 &&
                  g[3].tri < 0 && g[4].tri < 0,
              "bvh: nearest-hit selection resolves the stacked/miss scene as designed");
}

// ── mesh-post normals: GPU per-vertex normals vs CPU reference ───────────────
//
// A closed tetrahedron (every vertex shared by three faces) drives the
// angle-weighted per-vertex normal accumulation. GPU output is compared to
// computeNormalsCpuReference (identical formula + accumulation order); parity is
// asserted both per component and via the normalized dot product (~1) the task
// calls for.

void run_mesh_checks(Counts& c, MetalBackend& be) {
    using namespace cyber::metal;

    // Regular tetrahedron vertices; faces wound consistently.
    std::vector<Float3> verts = {
        {1.0f, 1.0f, 1.0f},
        {-1.0f, -1.0f, 1.0f},
        {-1.0f, 1.0f, -1.0f},
        {1.0f, -1.0f, -1.0f},
    };
    std::vector<TriIndices> tris = {
        {0, 1, 2},
        {0, 3, 1},
        {0, 2, 3},
        {1, 3, 2},
    };

    auto gpu = computeNormals(be, verts, tris);
    gpu_check(c, gpu.has_value(), "mesh-post: GPU per-vertex normals compute");
    if (!gpu.has_value()) {
        return;
    }
    const std::vector<Float3>& g = gpu.value();
    const std::vector<Float3> ref = computeNormalsCpuReference(verts, tris);

    bool sizeOk = g.size() == verts.size() && ref.size() == verts.size();
    gpu_check(c, sizeOk, "mesh-post: one normal per input vertex");

    bool components_ok = sizeOk;
    bool dots_ok = sizeOk;
    for (std::size_t i = 0; i < verts.size() && sizeOk; ++i) {
        const Float3& a = g[i];
        const Float3& b = ref[i];
        // Component agreement within the module's documented normal tolerance.
        components_ok = components_ok &&
                        std::fabs(a.x - b.x) <= kNormalParityAbsTol &&
                        std::fabs(a.y - b.y) <= kNormalParityAbsTol &&
                        std::fabs(a.z - b.z) <= kNormalParityAbsTol;
        // Normalized dot ~ 1 for the (unit) reference/GPU normal pair.
        const float refLen = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
        if (refLen > 0.5f) {  // skip any degenerate (zero) normal
            const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
            dots_ok = dots_ok && std::fabs(dot - 1.0f) <= 1e-3f;
        }
    }
    gpu_check(c, components_ok, "mesh-post: GPU normals match CPU reference per component (fp32)");
    gpu_check(c, dots_ok, "mesh-post: GPU/CPU normal dot product ~ 1 (aligned unit vectors)");
}

}  // namespace

// Defined in gpu_pick_check.mm (isolated because gpu_pick.h and gpu_surface_eval.h
// each declare a cyber::metal::Vec3f and cannot share a translation unit).
namespace gpusuite {
void run_pick_checks(Counts& c, std::shared_ptr<MetalBackend> backend);
}  // namespace gpusuite

int main() {
    @autoreleasepool {
        std::printf("== CyberCadKernel Phase 2 GPU-vs-CPU parity suite ==\n");

        Counts c;
        std::shared_ptr<MetalBackend> be = MetalBackend::create();
        gpu_check(c, be != nullptr, "metal device available on simulator GPU");

        if (be) {
            run_surface_checks(c, *be);
            run_bvh_checks(c, be);
            run_pick_checks(c, be);
            run_mesh_checks(c, *be);
        }

        std::printf("== %d passed, %d failed ==\n", c.passed, c.failed);
        std::fflush(stdout);
        std::_Exit(c.failed == 0 ? 0 : 1);
    }
}
