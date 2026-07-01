// Host unit tests for the OCCT-free parts of the parallel-acceleration policy:
// the global toggle + per-op override, the OSD_ThreadPool worker-cap resolution,
// the fine-thread boolean gate classification, and scheduler routing with the
// cancellation-safe boundary. The OCCT application (applyOcctWorkerCap) and the
// engine wiring are iOS-only and covered on device; everything exercised here is
// pure C++20, so it runs in the host build.

#include <stdexcept>
#include <thread>

#include "engine/occt/parallel_policy.h"
#include "harness.h"

using cyber::OperationContext;
using cyber::occt::GateDecision;
using cyber::occt::ParallelPolicy;
using cyber::occt::ParallelPref;
using cyber::occt::runScheduled;
using cyber::occt::ThreadSpec;

namespace {
// Each test resets the shared singleton to defaults so ordering never matters.
void reset(ParallelPolicy& p) {
    p.setEnabled(true);
    p.setWorkerCap(0);
    p.setGateEnabled(true);
    p.setFinePitchMM(1.0);
    p.setMinTurnsToGate(6.0);
    p.setCostBudget(30.0);
}
}  // namespace

CC_TEST(toggle_default_on_with_per_op_override) {
    ParallelPolicy& p = ParallelPolicy::instance();
    reset(p);
    // Default ON, opt-out globally.
    CC_CHECK(p.enabled());
    CC_CHECK(p.parallelFor());  // Default follows the toggle
    p.setEnabled(false);
    CC_CHECK(!p.parallelFor());
    // Per-op override wins over the global toggle, both ways.
    CC_CHECK(p.parallelFor(ParallelPref::Force));    // forced on though global off
    p.setEnabled(true);
    CC_CHECK(!p.parallelFor(ParallelPref::Disable));  // forced serial though global on
    reset(p);
}

CC_TEST(worker_cap_resolution) {
    ParallelPolicy& p = ParallelPolicy::instance();
    reset(p);
    // Auto default is a sensible, >= 1 value derived from hardware concurrency.
    CC_CHECK(p.workerCap() == 0);
    CC_CHECK(p.resolvedWorkerCount() >= 1);
    // An explicit host cap is honoured exactly.
    p.setWorkerCap(2);
    CC_CHECK_EQ(p.resolvedWorkerCount(), 2);
    // A negative cap is clamped to auto.
    p.setWorkerCap(-4);
    CC_CHECK(p.workerCap() == 0);
    CC_CHECK(p.resolvedWorkerCount() >= 1);
    reset(p);
}

CC_TEST(fine_thread_gate_refuses_high_turn_fine_pitch) {
    ParallelPolicy& p = ParallelPolicy::instance();
    reset(p);
    // 20 turns @ 0.5 mm pitch → cost 40 >= budget 30, fine-pitch + high-turn → gated.
    const GateDecision refused = p.evaluateGate(ThreadSpec{20.0, 0.5});
    CC_CHECK(refused.gated);
    CC_CHECK(!refused.reason.empty());
    // A coarse, low-turn thread is within budget → not gated.
    const GateDecision passed = p.evaluateGate(ThreadSpec{3.0, 2.0});
    CC_CHECK(!passed.gated);
    // Same fine thread but the host disabled the gate → not gated (Phase 3 un-gate).
    p.setGateEnabled(false);
    CC_CHECK(!p.evaluateGate(ThreadSpec{20.0, 0.5}).gated);
    reset(p);
}

CC_TEST(fine_thread_gate_boundary_and_tuning) {
    ParallelPolicy& p = ParallelPolicy::instance();
    reset(p);
    // A fine-pitch thread just under the turn threshold is NOT high-turn → passes.
    CC_CHECK(!p.evaluateGate(ThreadSpec{5.0, 0.5}).gated);  // turns 5 < 6
    // Host can tighten the budget so a previously-accepted thread is now gated.
    CC_CHECK(!p.evaluateGate(ThreadSpec{8.0, 1.0}).gated);  // cost 8 < 30
    p.setCostBudget(5.0);
    CC_CHECK(p.evaluateGate(ThreadSpec{8.0, 1.0}).gated);   // cost 8 >= 5 now
    reset(p);
}

CC_TEST(last_gate_decision_is_recorded_for_the_host) {
    ParallelPolicy& p = ParallelPolicy::instance();
    reset(p);
    GateDecision d = p.evaluateGate(ThreadSpec{20.0, 0.5});
    p.recordGateDecision(d);
    CC_CHECK(p.lastGateDecision().gated);
    CC_CHECK(p.lastGateDecision().reason == d.reason);
    reset(p);
}

CC_TEST(run_scheduled_completes_off_thread) {
    // A body runs to completion through the scheduler and its value comes back.
    const std::thread::id caller = std::this_thread::get_id();
    std::thread::id worker{};
    int value = runScheduled([&](OperationContext& ctx) -> int {
        worker = std::this_thread::get_id();
        ctx.report(0.5, "work");
        return 42;
    });
    CC_CHECK_EQ(value, 42);
    CC_CHECK(worker != std::thread::id{});
    CC_CHECK(worker != caller);  // ran off the caller's inline path
}

CC_TEST(run_scheduled_propagates_failure) {
    bool threw = false;
    try {
        runScheduled([](OperationContext&) -> int {
            throw std::runtime_error("boom");
        });
    } catch (const std::exception& e) {
        threw = true;
        CC_CHECK(std::string(e.what()) == "boom");  // message preserved for cc_last_error
    }
    CC_CHECK(threw);
}

CC_RUN_ALL()
