// Guard model + facade error surfacing: a thrown std::exception becomes the
// fallback value and a per-thread cc_last_error message; a stub geometry call
// returns 0/nil and records why; cc_brep_available() is 0 in the no-OCCT build.

#include <stdexcept>
#include <string>
#include <thread>

#include "core/guard.h"
#include "cybercadkernel/cc_kernel.h"
#include "harness.h"

CC_TEST(guard_returns_value_on_success) {
    int v = cyber::guard([]() -> int { return 11; }, -1);
    CC_CHECK_EQ(v, 11);
    CC_CHECK_EQ(std::string(cyber::last_error_cstr()), std::string(""));
}

CC_TEST(guard_catches_exception_and_records_message) {
    int v = cyber::guard([]() -> int { throw std::runtime_error("boom"); }, -1);
    CC_CHECK_EQ(v, -1);
    CC_CHECK_EQ(std::string(cyber::last_error_cstr()), std::string("boom"));
}

CC_TEST(guard_success_clears_previous_error) {
    (void)cyber::guard([]() -> int { throw std::runtime_error("first"); }, 0);
    CC_CHECK_EQ(std::string(cyber::last_error_cstr()), std::string("first"));
    int v = cyber::guard([]() -> int { return 1; }, 0);
    CC_CHECK_EQ(v, 1);
    CC_CHECK_EQ(std::string(cyber::last_error_cstr()), std::string(""));
}

CC_TEST(last_error_is_thread_local) {
    cyber::set_last_error("main-thread");
    std::string other;
    std::thread t([&other]() {
        (void)cyber::guard([]() -> int { throw std::runtime_error("worker"); }, 0);
        other = cyber::last_error_cstr();
    });
    t.join();
    CC_CHECK_EQ(other, std::string("worker"));
    CC_CHECK_EQ(std::string(cyber::last_error_cstr()), std::string("main-thread"));
}

CC_TEST(brep_unavailable_in_stub_build) {
    CC_CHECK_EQ(cc_brep_available(), 0);
}

CC_TEST(stub_geometry_returns_zero_and_sets_error) {
    double square[] = {0, 0, 1, 0, 1, 1, 0, 1};
    CCShapeId id = cc_solid_extrude(square, 4, 10.0);
    CC_CHECK_EQ(id, CCShapeId{0});
    CC_CHECK(std::string(cc_last_error()).size() > 0);  // recorded why it failed
}

CC_TEST(stub_tessellate_returns_empty_mesh) {
    CCMesh mesh = cc_tessellate(123, 0.1);
    CC_CHECK(mesh.vertices == nullptr);
    CC_CHECK_EQ(mesh.vertexCount, 0);
    CC_CHECK(mesh.triangles == nullptr);
    CC_CHECK_EQ(mesh.triangleCount, 0);
    cc_mesh_free(mesh);  // safe on empty
}

CC_TEST(release_unknown_id_is_safe) {
    cc_shape_release(0);
    cc_shape_release(999999);  // must not crash
    CC_CHECK(true);
}

CC_RUN_ALL()
