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
}

}  // namespace gpusuite
