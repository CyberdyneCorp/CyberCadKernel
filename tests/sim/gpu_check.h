#ifndef CYBERCADKERNEL_TESTS_SIM_GPU_CHECK_H
#define CYBERCADKERNEL_TESTS_SIM_GPU_CHECK_H

// Shared, Metal-free reporting helper for the integrated GPU-vs-CPU parity suite
// (tests/sim/gpu_suite.mm + tests/sim/gpu_pick_check.mm).
//
// Why two translation units share this header: the Phase-2 module headers
// gpu_surface_eval.h and gpu_pick.h each define a distinct `cyber::metal::Vec3f`,
// so they cannot be included in the same TU (redefinition). The suite therefore
// keeps surface-eval / BVH / mesh-post in gpu_suite.mm and isolates the picking
// checks in gpu_pick_check.mm; both report through this common counter so main()
// can print a single "== N passed, M failed ==" line.
//
// This header pulls in no Metal / Objective-C types, so it is safe in any TU.

#include <cstdio>

namespace gpusuite {

// Running pass/fail tally shared across the per-module check functions.
struct Counts {
    int passed = 0;
    int failed = 0;
};

// Emit one "[GPU] PASS <label>" / "[GPU] FAIL <label>" line and update the tally.
inline void gpu_check(Counts& c, bool cond, const char* label) {
    std::printf("[GPU] %s %s\n", cond ? "PASS" : "FAIL", label);
    if (cond) {
        ++c.passed;
    } else {
        ++c.failed;
    }
}

}  // namespace gpusuite

#endif  // CYBERCADKERNEL_TESTS_SIM_GPU_CHECK_H
