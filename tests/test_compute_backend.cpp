// Compute backend: the default CPU backend runs data-parallel work correctly and
// is fp64-capable; the precision guard keeps exact fp64 work off an fp32-only
// (GPU-like) backend even when that backend is active; a registered backend is
// selectable without caller changes.

#include <atomic>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "core/compute_backend.h"
#include "harness.h"

using cyber::ComputeRegistry;
using cyber::CpuComputeBackend;
using cyber::IComputeBackend;
using cyber::ParallelBody;
using cyber::Precision;

namespace {
// A stand-in fp32-only backend (like an Apple GPU: no fp64). Used to prove the
// precision guard refuses to route exact work to it.
class Fp32OnlyBackend final : public IComputeBackend {
public:
    std::string name() const override { return "fp32-fake-gpu"; }
    bool supports_fp64() const override { return false; }
    void parallel_for(std::size_t count, const ParallelBody& body) override {
        for (std::size_t i = 0; i < count; ++i) {
            body(i);
        }
    }
};
}  // namespace

CC_TEST(cpu_backend_is_fp64_capable) {
    CpuComputeBackend cpu;
    CC_CHECK_EQ(cpu.name(), std::string("cpu"));
    CC_CHECK(cpu.supports_fp64());
}

CC_TEST(cpu_parallel_for_covers_every_index) {
    CpuComputeBackend cpu;
    const std::size_t n = 10000;
    std::vector<int> hits(n, 0);
    cpu.parallel_for(n, [&hits](std::size_t i) { hits[i] += 1; });
    long sum = std::accumulate(hits.begin(), hits.end(), 0L);
    CC_CHECK_EQ(sum, static_cast<long>(n));
    for (std::size_t i = 0; i < n; ++i) {
        if (hits[i] != 1) {
            CC_CHECK(false);
            break;
        }
    }
}

CC_TEST(cpu_parallel_for_handles_empty_and_null) {
    CpuComputeBackend cpu;
    std::atomic<int> calls{0};
    cpu.parallel_for(0, [&calls](std::size_t) { calls.fetch_add(1); });
    CC_CHECK_EQ(calls.load(), 0);
    cpu.parallel_for(5, nullptr);  // must not crash
    CC_CHECK(true);
}

CC_TEST(registry_default_active_is_cpu_and_fp64_ok) {
    auto& reg = ComputeRegistry::instance();
    CC_CHECK_EQ(reg.active()->name(), std::string("cpu"));
    auto r = reg.backend_for(Precision::Fp64);
    CC_CHECK(r.has_value());
    CC_CHECK(r.value()->supports_fp64());
}

CC_TEST(precision_guard_keeps_fp64_off_fp32_backend) {
    auto& reg = ComputeRegistry::instance();
    reg.register_backend(std::make_shared<Fp32OnlyBackend>());
    auto activated = reg.set_active("fp32-fake-gpu");
    CC_CHECK(activated.has_value());
    CC_CHECK_EQ(reg.active()->name(), std::string("fp32-fake-gpu"));

    // fp32 work uses the active (fp32) backend...
    auto fp32 = reg.backend_for(Precision::Fp32);
    CC_CHECK(fp32.has_value());
    CC_CHECK_EQ(fp32.value()->name(), std::string("fp32-fake-gpu"));

    // ...but exact fp64 work is NEVER dispatched to it — it falls back to CPU.
    auto fp64 = reg.backend_for(Precision::Fp64);
    CC_CHECK(fp64.has_value());
    CC_CHECK(fp64.value()->supports_fp64());
    CC_CHECK_EQ(fp64.value()->name(), std::string("cpu"));

    // Restore the CPU default for other cases (shared singleton).
    (void)reg.set_active("cpu");
}

CC_TEST(unknown_backend_selection_fails) {
    auto& reg = ComputeRegistry::instance();
    auto r = reg.set_active("does-not-exist");
    CC_CHECK(!r.has_value());
    (void)reg.set_active("cpu");
}

CC_RUN_ALL()
