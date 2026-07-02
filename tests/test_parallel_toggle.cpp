// Host unit test for the ADDITIVE parallel-control surface (cc_set_parallel /
// cc_parallel_enabled). In the no-OCCT host build the active engine is the stub,
// whose set_parallel is a no-op and parallel_enabled() reports 0. This asserts the
// two entry points are callable, do not crash, never leak an error, and behave as
// the documented no-op in this build. On iOS (OCCT adapter) the same calls drive
// occt::ParallelPolicy and are covered on device.

#include <cstring>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

CC_TEST(parallel_toggle_callable_and_no_op_on_stub) {
    // Host/no-OCCT: no B-rep engine is linked, so parallel is a no-op reporting 0.
    const bool hasBrep = cc_brep_available() != 0;

    // Callable both ways without crashing.
    cc_set_parallel(0);
    cc_set_parallel(1);

    const int enabled = cc_parallel_enabled();
    // Result is a clean 0/1; the guarded call must not record an error.
    CC_CHECK(enabled == 0 || enabled == 1);
    CC_CHECK(std::strcmp(cc_last_error(), "") == 0);

    if (!hasBrep) {
        // Stub engine: the toggle is a no-op and always reports disabled.
        cc_set_parallel(1);
        CC_CHECK_EQ(cc_parallel_enabled(), 0);
        cc_set_parallel(0);
        CC_CHECK_EQ(cc_parallel_enabled(), 0);
    }
}

CC_RUN_ALL()
