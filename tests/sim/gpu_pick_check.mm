// Picking parity checks for the Phase-2 GPU-vs-CPU suite.
//
// Isolated in its own translation unit: gpu_pick.h and gpu_surface_eval.h each
// define a `cyber::metal::Vec3f`, so they cannot be included in the same TU. This
// TU includes only gpu_pick.h; gpu_suite.mm owns main() and calls run_pick_checks
// (declared there, defined here) through the shared, Metal-free gpu_check.h.
//
// It runs a few camera rays through GpuPick on the Metal GPU and asserts the
// nearest-hit result equals the CPU reference oracle (cpuPickReference): identical
// triangle id (exact at fp32, lowest-index tie-break) and hit point within the
// module's documented kPointTolerance. All arithmetic is fp32; no OCCT, no fp64.

#import <Foundation/Foundation.h>

#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include "compute/metal/gpu_pick.h"
#include "compute/metal/metal_backend.h"
#include "core/result.h"
#include "gpu_check.h"

namespace gpusuite {

namespace {

cyber::metal::Vec3f v3(float x, float y, float z) {
    cyber::metal::Vec3f v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

cyber::metal::PickTriangle tri(cyber::metal::Vec3f a, cyber::metal::Vec3f b,
                               cyber::metal::Vec3f c) {
    cyber::metal::PickTriangle t;
    t.v0 = a;
    t.v1 = b;
    t.v2 = c;
    return t;
}

cyber::metal::PickRay ray(cyber::metal::Vec3f origin, cyber::metal::Vec3f dir) {
    cyber::metal::PickRay r;
    r.origin = origin;
    r.dir = dir;
    r.tmin = 0.0f;
    r.tmax = 1.0e30f;
    return r;
}

bool pointClose(const cyber::metal::Vec3f& a, const cyber::metal::Vec3f& b) {
    return std::fabs(a.x - b.x) <= cyber::metal::kPointTolerance &&
           std::fabs(a.y - b.y) <= cyber::metal::kPointTolerance &&
           std::fabs(a.z - b.z) <= cyber::metal::kPointTolerance;
}

// Axis-aligned box frustum: inside is xmin<=x<=xmax (etc.). Half-spaces use the
// module's inward-normal convention (a*x+b*y+c*z+d >= 0). d is chosen so the plane
// passes through the box face: e.g. x>=xmin => (1,0,0, -xmin), x<=xmax => (-1,0,0, xmax).
cyber::metal::PickFrustum boxFrustum(float xmin, float xmax, float ymin, float ymax,
                                     float zmin, float zmax) {
    cyber::metal::PickFrustum f{};
    f.planes[0][0] = 1.0f;  f.planes[0][3] = -xmin;  // x >= xmin
    f.planes[1][0] = -1.0f; f.planes[1][3] = xmax;   // x <= xmax
    f.planes[2][1] = 1.0f;  f.planes[2][3] = -ymin;  // y >= ymin
    f.planes[3][1] = -1.0f; f.planes[3][3] = ymax;   // y <= ymax
    f.planes[4][2] = 1.0f;  f.planes[4][3] = -zmin;  // z >= zmin
    f.planes[5][2] = -1.0f; f.planes[5][3] = zmax;   // z <= zmax
    return f;
}

// Byte-identical equality of a ray-pick batch: primitive id, t and hit point are
// compared bit-for-bit (memcmp), not within a tolerance, so any nondeterministic
// bit in the GPU result is caught.
bool hitsByteIdentical(const std::vector<cyber::metal::PickHit>& a,
                       const std::vector<cyber::metal::PickHit>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].primitive != b[i].primitive) {
            return false;
        }
        if (std::memcmp(&a[i].t, &b[i].t, sizeof(float)) != 0) {
            return false;
        }
        if (std::memcmp(&a[i].point, &b[i].point, sizeof(cyber::metal::Vec3f)) != 0) {
            return false;
        }
    }
    return true;
}

}  // namespace

void run_pick_checks(Counts& c, std::shared_ptr<cyber::metal::MetalBackend> backend) {
    using namespace cyber::metal;

    // Deterministic scene: three quadrant-triangles stacked in z (nearest-hit
    // selection) plus one lateral triangle a camera ray cleanly misses.
    PickScene scene;
    scene.triangles = {
        tri(v3(0, 0, 1), v3(1, 0, 1), v3(0, 1, 1)),  // 0: z = 1
        tri(v3(0, 0, 2), v3(1, 0, 2), v3(0, 1, 2)),  // 1: z = 2
        tri(v3(0, 0, 3), v3(1, 0, 3), v3(0, 1, 3)),  // 2: z = 3
        tri(v3(10, 10, 1), v3(11, 10, 1), v3(10, 11, 1)),  // 3: off to the side
    };

    // A few camera rays: nearest through the stack, a mid-stack start, and a miss.
    std::vector<PickRay> rays = {
        ray(v3(0.3f, 0.3f, -1.0f), v3(0, 0, 1)),  // -> tri 0
        ray(v3(0.3f, 0.3f, 2.5f), v3(0, 0, 1)),   // -> tri 2
        ray(v3(5.0f, 5.0f, -1.0f), v3(0, 0, 1)),  // -> miss
    };

    auto eng = GpuPick::create(backend);
    gpu_check(c, eng.has_value(), "pick: GpuPick engine compiles pipelines");
    if (!eng.has_value()) {
        return;
    }
    std::shared_ptr<GpuPick> engine = eng.value();
    gpu_check(c, engine && engine->usesGpu(), "pick: engine is driven by the Metal GPU");

    auto batch = engine->pick(rays, scene);
    gpu_check(c, batch.has_value(), "pick: GPU batched ray-pick succeeds");
    if (!batch.has_value()) {
        return;
    }
    const std::vector<PickHit>& g = batch.value();

    bool ok = g.size() == rays.size();
    for (std::size_t i = 0; i < rays.size() && ok; ++i) {
        const PickHit ref = cpuPickReference(rays[i], scene);
        const bool sameId = g[i].primitive == ref.primitive;
        const bool samePoint = !ref.hit() || pointClose(g[i].point, ref.point);
        ok = sameId && samePoint;
    }
    gpu_check(c, ok, "pick: GPU ray-pick matches CPU reference (same id + point within tol)");

    gpu_check(c,
              g.size() == rays.size() && g[0].primitive == 0 && g[1].primitive == 2 &&
                  !g[2].hit(),
              "pick: nearest-hit / miss resolved as designed for the scene");

    // ── Frustum-pick parity (tasks 3.2, 3.4) ────────────────────────────────
    // The scene's triangle AABBs sit at z=1 (tri 0), z=2 (tri 1), z=3 (tri 2),
    // and x,y~=10 z=1 (tri 3, off to the side). A box [-5,5]^2 x [0.5,2.5] on z
    // encloses tri 0 fully, encloses tri 1 fully, straddles nothing at its faces
    // but excludes tri 2 (z=3 beyond the far face) and tri 3 (x,y=10 beyond the
    // lateral faces) => expected subset {0, 1}.
    const PickFrustum subset = boxFrustum(-5.0f, 5.0f, -5.0f, 5.0f, 0.5f, 2.5f);
    auto subsetGpu = engine->pickFrustum(subset, scene);
    gpu_check(c, subsetGpu.has_value(), "pick: GPU frustum-pick succeeds");
    const std::vector<std::int32_t> subsetRef = cpuFrustumReference(subset, scene);

    // Task 3.4: GPU frustum set EQUALS the CPU brute-force set (same ids).
    gpu_check(c, subsetGpu.has_value() && subsetGpu.value() == subsetRef,
              "pick: GPU frustum set equals CPU reference set (same ids)");
    // Guard the oracle itself against a trivially-true equality: it must be the
    // real, known subset {0, 1}, not e.g. an accidental empty set on both sides.
    gpu_check(c, subsetRef == (std::vector<std::int32_t>{0, 1}),
              "pick: frustum reference selects the known subset {0,1}");

    // Task 3.2: the returned set is emitted sorted ascending (strictly increasing).
    bool sortedAsc = subsetGpu.has_value();
    if (subsetGpu.has_value()) {
        const std::vector<std::int32_t>& s = subsetGpu.value();
        for (std::size_t i = 1; i < s.size(); ++i) {
            sortedAsc = sortedAsc && (s[i - 1] < s[i]);
        }
    }
    gpu_check(c, sortedAsc, "pick: GPU frustum set is sorted ascending");

    // Empty selection: a frustum entirely beyond every triangle (z >= 100) => {}.
    const PickFrustum empty = boxFrustum(-1e6f, 1e6f, -1e6f, 1e6f, 100.0f, 200.0f);
    auto emptyGpu = engine->pickFrustum(empty, scene);
    gpu_check(c,
              emptyGpu.has_value() && emptyGpu.value().empty() &&
                  emptyGpu.value() == cpuFrustumReference(empty, scene),
              "pick: empty-selection frustum returns {} (matches CPU)");

    // All-enclosing: a huge box selects every triangle id in order.
    const PickFrustum all = boxFrustum(-1e6f, 1e6f, -1e6f, 1e6f, -1e6f, 1e6f);
    auto allGpu = engine->pickFrustum(all, scene);
    std::vector<std::int32_t> allIds(scene.triangles.size());
    for (std::size_t i = 0; i < allIds.size(); ++i) {
        allIds[i] = static_cast<std::int32_t>(i);
    }
    gpu_check(c,
              allGpu.has_value() && allGpu.value() == allIds &&
                  allGpu.value() == cpuFrustumReference(all, scene),
              "pick: all-enclosing frustum returns every id");

    // ── Determinism (task 5.1) ──────────────────────────────────────────────
    // Repeat both GPU ops >=8x; every run must be byte-identical to the first
    // (fixed nearest-hit tie-break; sorted frustum set independent of GPU order).
    constexpr int kRepeats = 8;
    bool rayDet = true;
    for (int i = 0; i < kRepeats; ++i) {
        auto again = engine->pick(rays, scene);
        rayDet = rayDet && again.has_value() && hitsByteIdentical(again.value(), g);
    }
    gpu_check(c, rayDet, "pick: GPU ray-pick is byte-identical across 8 runs");

    bool frustumDet = subsetGpu.has_value();
    for (int i = 0; i < kRepeats && frustumDet; ++i) {
        auto again = engine->pickFrustum(subset, scene);
        frustumDet = again.has_value() && again.value() == subsetGpu.value();
    }
    gpu_check(c, frustumDet, "pick: GPU frustum-pick is byte-identical across 8 runs");
}

}  // namespace gpusuite
