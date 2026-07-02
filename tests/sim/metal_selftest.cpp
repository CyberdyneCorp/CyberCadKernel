// On-simulator acceptance suite for the Metal compute backend.
//
// Runs inside a booted iOS simulator via `xcrun simctl spawn` (see
// scripts/run-metal-sim.sh), where MTLCreateSystemDefaultDevice() = "Apple iOS
// simulator GPU" and newLibraryWithSource: compiles MSL at runtime. This TU is
// OCCT-free — it links only the Metal backend, its Phase-0 dependencies, and
// Metal/Foundation. It exercises every metal-backend spec scenario end to end.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "compute/metal/metal_backend.h"
#include "core/compute_backend.h"

using cyber::ComputeRegistry;
using cyber::Precision;
using cyber::metal::MetalBackend;

namespace {

int g_failed = 0;

void check(bool cond, const char* label) {
    std::printf("[%s] %s\n", cond ? "PASS" : "FAIL", label);
    if (!cond) {
        ++g_failed;
    }
}

// A saxpy kernel: y = a*x + y. Used for the CPU-parity dispatch check.
constexpr const char* kSaxpy = R"MSL(
#include <metal_stdlib>
using namespace metal;
kernel void saxpy(device const float* x [[buffer(0)]],
                  device float*       y [[buffer(1)]],
                  constant float&     a [[buffer(2)]],
                  uint gid [[thread_position_in_grid]]) {
    y[gid] = a * x[gid] + y[gid];
}
)MSL";

// Deliberately broken MSL: must surface a Result error, not crash.
constexpr const char* kBroken = "kernel void nope( this is not valid MSL {";

void run_device_and_selftest() {
    auto be = MetalBackend::create();
    check(be != nullptr, "device init succeeds on simulator GPU");
    if (!be) {
        return;
    }
    check(be->name() == "metal", "backend reports stable name 'metal'");
    check(!be->supports_fp64(), "supports_fp64() is false");
    check(cyber::metal::metal_backend_selftest(), "built-in add-kernel self-test passes");
}

void run_buffer_roundtrip() {
    auto be = MetalBackend::create();
    if (!be) {
        return;
    }
    const std::size_t n = 256;
    std::vector<float> src(n);
    for (std::size_t i = 0; i < n; ++i) {
        src[i] = static_cast<float>(i) * 0.5f;
    }
    auto h = be->makeSharedBuffer(src.data(), n * sizeof(float));
    check(h != nullptr, "shared buffer allocated");
    const auto* p = static_cast<const float*>(be->bufferContents(h));
    bool same = p != nullptr;
    for (std::size_t i = 0; i < n && same; ++i) {
        same = (p[i] == src[i]);
    }
    check(same, "CPU/GPU shared-buffer round-trip returns written bytes (no copy)");
}

void run_pipeline_cache() {
    auto be = MetalBackend::create();
    if (!be) {
        return;
    }
    auto p1 = be->compilePipeline(kSaxpy, "saxpy");
    check(p1.has_value(), "runtime MSL compile builds a pipeline");
    auto p2 = be->compilePipeline(kSaxpy, "saxpy");
    check(p2.has_value() && p1.value() == p2.value(),
          "identical source reuses the cached pipeline (no recompile)");
    auto bad = be->compilePipeline(kBroken, "nope");
    check(!bad.has_value(), "malformed kernel source returns an error, not a crash");
}

void run_dispatch_parity() {
    auto be = MetalBackend::create();
    if (!be) {
        return;
    }
    const std::size_t n = 4096;
    const float a = 2.5f;
    std::vector<float> x(n), y(n), ref(n);
    for (std::size_t i = 0; i < n; ++i) {
        x[i] = static_cast<float>(i);
        y[i] = static_cast<float>(n - i);
        ref[i] = a * x[i] + y[i];  // CPU reference
    }
    auto hx = be->makeSharedBuffer(x.data(), n * sizeof(float));
    auto hy = be->makeSharedBuffer(y.data(), n * sizeof(float));
    auto ha = be->makeSharedBuffer(&a, sizeof(float));
    auto pso = be->compilePipeline(kSaxpy, "saxpy");
    check(pso.has_value(), "saxpy pipeline compiles");
    if (!pso) {
        return;
    }
    auto d = be->dispatch(pso.value(), {hx, hy, ha}, n);
    check(d.has_value(), "dispatch over [0, N) succeeds");
    const auto* out = static_cast<const float*>(be->bufferContents(hy));
    bool within_tol = out != nullptr;
    for (std::size_t i = 0; i < n && within_tol; ++i) {
        within_tol = std::fabs(out[i] - ref[i]) <= 1e-4f * std::max(1.0f, std::fabs(ref[i]));
    }
    check(within_tol, "elementwise kernel matches CPU reference within fp32 tolerance");
}

void run_precision_boundary() {
    auto be = MetalBackend::create();
    if (!be) {
        return;
    }
    auto pso = be->compilePipeline(kSaxpy, "saxpy");
    if (!pso) {
        return;
    }
    auto x = be->makeSharedBuffer(sizeof(float));
    auto refused = be->dispatch(pso.value(), {x}, 1, 0, Precision::Fp64);
    check(!refused.has_value(), "backend refuses an explicit fp64 dispatch");
}

void run_registry_integration() {
    const bool registered = cyber::metal::registerMetalBackend();
    check(registered, "Metal backend registers with ComputeRegistry");

    auto& reg = ComputeRegistry::instance();
    check(reg.cpu() && reg.cpu()->name() == "cpu", "CPU backend remains the fallback");

    auto act = reg.set_active("metal");
    check(act.has_value(), "Metal backend is selectable as active");

    // fp32 work goes to the active (Metal) backend...
    auto fp32 = reg.backend_for(Precision::Fp32);
    check(fp32.has_value() && fp32.value()->name() == "metal",
          "fp32 work routes to the active Metal backend");

    // ...but exact fp64 work is NEVER routed to Metal — the guard diverts to CPU.
    auto fp64 = reg.backend_for(Precision::Fp64);
    check(fp64.has_value() && fp64.value()->supports_fp64() && fp64.value()->name() == "cpu",
          "precision guard keeps fp64 work on the CPU, never Metal");

    (void)reg.set_active("cpu");  // restore default
}

}  // namespace

int main() {
    std::printf("== metal-backend on-simulator acceptance ==\n");
    run_device_and_selftest();
    run_buffer_roundtrip();
    run_pipeline_cache();
    run_dispatch_parity();
    run_precision_boundary();
    run_registry_integration();
    std::printf("%s (%d failed)\n", g_failed == 0 ? "ALL PASS" : "FAILURES", g_failed);
    return g_failed == 0 ? 0 : 1;
}
