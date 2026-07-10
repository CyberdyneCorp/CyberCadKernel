// Regression test — cross-engine body handoff must DECLINE, never SEGFAULT.
//
// THE BUG (reproducible on the raw C ABI, pure ctypes/C — no binding involved):
//   1. cc_set_engine(1)          — switch to the NATIVE engine.
//   2. cc_solid_extrude(box)     — build a body OWNED by the native engine (its
//                                  CCShapeId resolves to a NativeShape holder).
//   3. cc_set_engine(0)          — switch back to the DEFAULT engine (real OCCT in
//                                  the macOS-dylib / sim build; the stub on host).
//   4. cc_step_export(handle) OR cc_mass_properties(handle) on that native handle.
//
// In the OCCT build step 4 used to CRASH (exit 139): occt::unwrap() did an unchecked
// static_pointer_cast<OcctShape> on the native void, read a garbage TopoDS_Shape, and
// OCCT dereferenced the bad handle. The header contract is explicit that a native body
// is NEVER handed to OCCT — so this is an engine-adapter defect. The fix guards
// occt::unwrap() at the single handle-resolution chokepoint (src/engine/occt/
// occt_engine.cpp) using the shared cyber::is_native_shape provenance registry, so a
// foreign native body becomes an HONEST DECLINE (return the failure sentinel +
// cc_last_error) for EVERY body-consuming OCCT op, not a crash.
//
// This suite is ENGINE-AGNOSTIC and runs in BOTH configs:
//   * macOS-dylib / sim (default engine = OCCT): the crash reproduces without the fix;
//     with the fix the ops decline. This is the config that exercised the SIGSEGV.
//   * host (default engine = stub, OCCT-free): the stub already declines every geometry
//     op, so the sequence never crashed there — but the SAME decline contract is
//     asserted, guarding the wiring in the config the CI host can build.
// The symmetric case (an OCCT/stub-built body operated under the native engine) is also
// asserted: the native engine's own isNative() guard forwards it to the fallback, and a
// stub fallback declines — never a crash.

#include <cstring>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

namespace {

// Reset to the default engine after each test so cases stay independent.
struct EngineGuard {
    ~EngineGuard() { cc_set_engine(0); }
};

// A closed CCW unit square → extruded to a 1x1x2 box under whichever engine is active.
CCShapeId build_box() {
    const double profile[] = {0.0, 0.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0};
    return cc_solid_extrude(profile, 4, 2.0);
}

}  // namespace

// A NATIVE-built body, operated under the DEFAULT engine (OCCT where linked, else the
// stub), must NOT crash. It either declines (OCCT: foreign body refused at unwrap; stub:
// unsupported) or — if the fallback happened to serve it — returns a coherent result.
// The load-bearing assertion is simply that we REACH this line: before the fix, step 4
// SIGSEGV'd the whole process under OCCT and this test never returned.
CC_TEST(native_body_under_default_engine_declines_not_segfault) {
    EngineGuard g;

    cc_set_engine(1);  // native engine
    const CCShapeId nativeBody = build_box();
    CC_CHECK(nativeBody != 0);  // the native engine builds the prism natively
    if (nativeBody == 0) {
        std::printf("  last_error=%s\n", cc_last_error());
        return;
    }

    cc_set_engine(0);  // back to the default engine — nativeBody is now FOREIGN to it

    // (a) cc_mass_properties on the foreign native handle: must not crash. Under OCCT
    //     the guarded unwrap declines (valid == 0 + cc_last_error set); under the stub
    //     it is unsupported (valid == 0). Reaching here at all proves no SIGSEGV.
    const CCMassProps mp = cc_mass_properties(nativeBody);
    CC_CHECK_EQ(mp.valid, 0);
    // An honest decline records WHY; the message is non-empty.
    CC_CHECK(std::strcmp(cc_last_error(), "") != 0);

    // (b) cc_step_export on the foreign native handle: must not crash and must report
    //     failure (return 0). Path is a scratch file that is never actually written on
    //     the decline path.
    const int exported = cc_step_export(nativeBody, "cross_engine_guard_scratch.step");
    CC_CHECK_EQ(exported, 0);
    CC_CHECK(std::strcmp(cc_last_error(), "") != 0);

    // (c) A second body-consuming op through a different code path (bounding_box) is
    //     likewise a decline, proving the guard is at the shared handle-resolution
    //     boundary and protects ALL ops, not only the two that surfaced the bug.
    double box6[6] = {0, 0, 0, 0, 0, 0};
    const int bbok = cc_bounding_box(nativeBody, box6);
    CC_CHECK_EQ(bbok, 0);

    cc_shape_release(nativeBody);
}

// SYMMETRIC case: a body built under the DEFAULT engine, then operated under the NATIVE
// engine. The native engine's isNative() guard recognises the foreign body and forwards
// it to its fallback (OCCT serves it; the stub declines). Neither path crashes. On the
// OCCT-free host build the default engine is the stub, so build_box() returns 0 and the
// meaningful assertion is that toggling + the op still never crash.
CC_TEST(default_body_under_native_engine_declines_not_segfault) {
    EngineGuard g;

    cc_set_engine(0);  // default engine (OCCT or stub)
    const CCShapeId defaultBody = build_box();

    cc_set_engine(1);  // native engine — defaultBody is FOREIGN to it

    if (defaultBody == 0) {
        // Host/stub build: the default engine cannot build a solid. Nothing to operate
        // on, but we proved the toggle path is crash-free. (Under OCCT defaultBody != 0.)
        CC_CHECK(cc_active_engine() == 1);
        return;
    }

    // Operate the OCCT-built body under the native engine. isNative() is false for it, so
    // the native engine forwards to its OCCT fallback, which serves it correctly — the
    // point is NO CRASH and a coherent verdict. (mp.valid may be 1 here since OCCT can
    // still read its own body via the fallback; we only require we did not SIGSEGV.)
    const CCMassProps mp = cc_mass_properties(defaultBody);
    CC_CHECK(mp.valid == 0 || mp.valid == 1);  // either honest decline or served — not a crash

    cc_shape_release(defaultBody);
}

CC_RUN_ALL()
