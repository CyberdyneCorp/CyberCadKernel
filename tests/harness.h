#ifndef CYBERCADKERNEL_TESTS_HARNESS_H
#define CYBERCADKERNEL_TESTS_HARNESS_H

// Tiny self-contained assert-based test harness — no external gtest. A test file
// declares cases with CC_TEST and runs them from main() via CC_RUN_ALL(). A
// failed CC_CHECK prints the location and marks the process exit code non-zero
// so CTest reports the failure.

#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace cctest {

struct Case {
    std::string name;
    std::function<void(bool&)> fn;
};

inline std::vector<Case>& cases() {
    static std::vector<Case> c;
    return c;
}

struct Registrar {
    Registrar(std::string name, std::function<void(bool&)> fn) {
        cases().push_back(Case{std::move(name), std::move(fn)});
    }
};

inline int run_all() {
    int failed = 0;
    for (auto& c : cases()) {
        bool ok = true;
        try {
            c.fn(ok);
        } catch (const std::exception& e) {
            ok = false;
            std::printf("  [threw] %s: %s\n", c.name.c_str(), e.what());
        } catch (...) {
            ok = false;
            std::printf("  [threw] %s: unknown exception\n", c.name.c_str());
        }
        std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", c.name.c_str());
        std::fflush(stdout);
        if (!ok) {
            ++failed;
        }
    }
    std::printf("%zu cases, %d failed\n", cases().size(), failed);
    return failed == 0 ? 0 : 1;
}

}  // namespace cctest

#define CC_TEST(name)                                                             \
    static void cc_test_##name(bool& cc_ok_);                                     \
    static ::cctest::Registrar cc_reg_##name(#name, cc_test_##name);              \
    static void cc_test_##name([[maybe_unused]] bool& cc_ok_)

#define CC_CHECK(cond)                                                            \
    do {                                                                          \
        if (!(cond)) {                                                            \
            cc_ok_ = false;                                                       \
            std::printf("  CHECK failed: %s (%s:%d)\n", #cond, __FILE__, __LINE__); \
        }                                                                         \
    } while (0)

#define CC_CHECK_EQ(a, b)                                                         \
    do {                                                                          \
        if (!((a) == (b))) {                                                      \
            cc_ok_ = false;                                                       \
            std::printf("  CHECK_EQ failed: %s == %s (%s:%d)\n", #a, #b, __FILE__, \
                        __LINE__);                                                \
        }                                                                         \
    } while (0)

#define CC_RUN_ALL()                                                             \
    int main() { return ::cctest::run_all(); }

#endif  // CYBERCADKERNEL_TESTS_HARNESS_H
