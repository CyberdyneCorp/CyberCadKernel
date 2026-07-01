#ifndef CYBERCADKERNEL_ENGINE_OCCT_PARALLEL_POLICY_H
#define CYBERCADKERNEL_ENGINE_OCCT_PARALLEL_POLICY_H

// Parallel-acceleration policy for the OCCT adapter (Phase 1,
// `accelerate-multicore-occt`). Owns the four knobs the parallel booleans /
// meshing paths consult:
//
//   1. Worker cap        — a host-settable bound on OCCT's OSD_ThreadPool so a
//                          mobile device is never oversubscribed; a sensible
//                          default is derived from hardware concurrency.
//   2. Parallel toggle   — a process-wide "parallel" switch (default ON, opt-out)
//                          with a per-operation override, so a caller can force a
//                          serial run (e.g. the determinism audit) without
//                          disabling parallelism globally.
//   3. Fine-thread gate  — refuses the fuse/cut of a high-turn fine-pitch thread
//                          into a shaft (keeping them as separate bodies) and
//                          surfaces the decision to the host rather than hanging.
//   4. Scheduler routing — runScheduled() runs a long boolean/mesh through the
//                          Phase-0 operation-scheduler (off the caller's inline
//                          path) with the cancellation-safe engine boundary.
//
// IMPORTANT — this header is deliberately OCCT-FREE. It carries only plain C++20
// (atomics, the scheduler, POD structs) so the pure policy logic (toggle, cap
// resolution, gate classification, scheduler routing) is unit-testable on the
// host, even though the OCCT adapter TUs that USE it are iOS-only. The single
// OCCT-touching entry point, applyOcctWorkerCap(), is only DECLARED here and
// DEFINED in parallel_policy.cpp (which includes <OSD_ThreadPool.hxx> and so
// compiles only for iOS). Nothing here includes an OpenCASCADE header.

#include <atomic>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

#include "core/scheduler.h"

namespace cyber {
namespace occt {

// ── Per-operation parallel preference ─────────────────────────────────────────
// The global toggle sets the default; a single call can override it either way.
enum class ParallelPref {
    Default,  // follow the global `parallel` toggle
    Force,    // run parallel regardless of the global toggle
    Disable   // run serial regardless of the global toggle (e.g. determinism audit)
};

// ── Fine-thread gate types ────────────────────────────────────────────────────

// Description of a helical thread body, tagged onto the shape at build time by
// cc_helical_thread / cc_tapered_thread and read by the boolean gate.
struct ThreadSpec {
    double turns = 0.0;
    double pitchMM = 0.0;
};

// The gate's verdict for one boolean, surfaced to the host (via cc_last_error and
// ParallelPolicy::lastGateDecision()).
struct GateDecision {
    bool gated = false;      // true => the boolean was refused, bodies kept separate
    std::string reason;      // human-readable explanation for the host
};

// ── ParallelPolicy — process-wide singleton ───────────────────────────────────
// All setters are host-callable; all getters are cheap atomics so the hot boolean
// / mesh paths can read them per op. Cognitive complexity is low by construction:
// every method is a guard-free accessor or a short classification.
class ParallelPolicy {
public:
    static ParallelPolicy& instance() {
        static ParallelPolicy policy;
        return policy;
    }

    // ── Global parallel toggle (default ON, opt-out) ──────────────────────────
    void setEnabled(bool on) noexcept { enabled_.store(on, std::memory_order_relaxed); }
    bool enabled() const noexcept { return enabled_.load(std::memory_order_relaxed); }

    // Resolve a per-op preference against the global toggle.
    bool parallelFor(ParallelPref pref = ParallelPref::Default) const noexcept {
        switch (pref) {
            case ParallelPref::Force:   return true;
            case ParallelPref::Disable: return false;
            case ParallelPref::Default: break;
        }
        return enabled();
    }

    // ── Worker cap over OSD_ThreadPool ────────────────────────────────────────
    // 0 => auto: derive a sensible default from hardware concurrency. A positive
    // value is an explicit host cap for mobile.
    void setWorkerCap(int maxThreads) noexcept {
        workerCap_.store(maxThreads < 0 ? 0 : maxThreads, std::memory_order_relaxed);
    }
    int workerCap() const noexcept { return workerCap_.load(std::memory_order_relaxed); }

    // Effective worker-thread count (always >= 1). This is the number
    // applyOcctWorkerCap() feeds to OSD_ThreadPool and the size of the scheduler
    // dispatch pool.
    int resolvedWorkerCount() const noexcept {
        const int cap = workerCap();
        if (cap > 0) {
            return cap;
        }
        // Auto default: leave one core for the UI/main thread on small devices.
        unsigned hw = std::thread::hardware_concurrency();
        if (hw <= 1) {
            return 1;
        }
        return static_cast<int>(hw > 1 ? hw - 1 : hw);
    }

    // ── Fine-thread gate ──────────────────────────────────────────────────────
    // The gate is ON by default; Phase 3's robust thread↔shaft boolean removes the
    // need for it, at which point the host disables it. `finePitchMM` /
    // `minTurnsToGate` describe what "fine" and "high-turn" mean; `costBudget` is
    // the acceptable-cost ceiling above which the fuse/cut is refused.
    void setGateEnabled(bool on) noexcept { gateEnabled_.store(on, std::memory_order_relaxed); }
    bool gateEnabled() const noexcept { return gateEnabled_.load(std::memory_order_relaxed); }

    void setFinePitchMM(double mm) noexcept { finePitchMM_.store(mm, std::memory_order_relaxed); }
    double finePitchMM() const noexcept { return finePitchMM_.load(std::memory_order_relaxed); }

    void setMinTurnsToGate(double turns) noexcept {
        minTurnsToGate_.store(turns, std::memory_order_relaxed);
    }
    double minTurnsToGate() const noexcept {
        return minTurnsToGate_.load(std::memory_order_relaxed);
    }

    void setCostBudget(double cost) noexcept { costBudget_.store(cost, std::memory_order_relaxed); }
    double costBudget() const noexcept { return costBudget_.load(std::memory_order_relaxed); }

    // Estimated relative cost of booleaning a thread: turns-per-mm. A boolean's
    // runtime tracks the number of thread crest/root intersection curves, which
    // grows with turns and with 1/pitch — so turns/pitch is a cheap, monotone
    // proxy that lets the gate predict a runaway op WITHOUT running it (the whole
    // point: never hang to find out it was too expensive).
    double estimatedThreadCost(const ThreadSpec& t) const noexcept {
        const double pitch = t.pitchMM > 1.0e-6 ? t.pitchMM : 1.0e-6;
        return t.turns / pitch;
    }

    // Is this thread body a gate candidate at all (fine-pitch AND high-turn)?
    bool isFineThread(const ThreadSpec& t) const noexcept {
        const bool finePitch = t.pitchMM > 0.0 && t.pitchMM <= finePitchMM();
        const bool highTurn = t.turns >= minTurnsToGate();
        return finePitch && highTurn;
    }

    // Decide whether a fuse/cut with this thread operand must be gated.
    GateDecision evaluateGate(const ThreadSpec& t) const {
        GateDecision d;
        if (!gateEnabled()) {
            d.reason = "fine-thread gate: disabled by host";
            return d;  // gated == false
        }
        const double cost = estimatedThreadCost(t);
        if (isFineThread(t) && cost >= costBudget()) {
            d.gated = true;
            d.reason = "fine-thread gate: refused fuse/cut of high-turn fine-pitch thread (turns=" +
                       trimmed(t.turns) + ", pitch=" + trimmed(t.pitchMM) + "mm, cost=" +
                       trimmed(cost) + " >= budget " + trimmed(costBudget()) +
                       "); kept as separate bodies";
            return d;
        }
        d.reason = "fine-thread gate: within budget";
        return d;  // gated == false
    }

    // Last gate decision on THIS thread — the host surface. Recorded on every
    // boolean whose operand is a thread, whether gated or not.
    void recordGateDecision(const GateDecision& d) { lastDecisionTls() = d; }
    const GateDecision& lastGateDecision() const { return lastDecisionTls(); }

private:
    ParallelPolicy() = default;

    // to_string without the fixed 6-decimal noise, so gate messages read cleanly.
    static std::string trimmed(double v) {
        std::string s = std::to_string(v);
        auto dot = s.find('.');
        if (dot == std::string::npos) {
            return s;
        }
        std::size_t last = s.find_last_not_of('0');
        if (last == dot) {
            last = dot - 1;  // drop a trailing ".000000" entirely
        }
        return s.substr(0, last + 1);
    }

    // One thread_local decision slot, shared across TUs via this inline function.
    static GateDecision& lastDecisionTls() {
        thread_local GateDecision decision;
        return decision;
    }

    std::atomic<bool> enabled_{true};
    std::atomic<int> workerCap_{0};          // 0 => auto from hardware concurrency
    std::atomic<bool> gateEnabled_{true};
    std::atomic<double> finePitchMM_{1.0};   // pitch <= 1.0 mm is "fine"
    std::atomic<double> minTurnsToGate_{6.0};
    std::atomic<double> costBudget_{30.0};   // turns/pitch ceiling; tuned on device (§6)
};

// ── OSD_ThreadPool application (OCCT-only; defined in parallel_policy.cpp) ─────
// Bounds OCCT's default thread pool to ParallelPolicy::resolvedWorkerCount() via
// OSD_ThreadPool::DefaultPool()->SetNbDefaultThreadsToLaunch(). Called by the
// boolean / mesh paths right before running a parallel op, so a host cap change
// takes effect on the next operation. It is only declared here; the definition
// lives in the OCCT translation unit, so a host build never needs OCCT to use the
// pure policy above.
void applyOcctWorkerCap();

// ── Scheduler routing with the cancellation-safe boundary ─────────────────────
// The shared dispatch pool for long ops. Sized once from the worker cap; the OCCT
// OSD_ThreadPool cap (applyOcctWorkerCap) is the re-settable bound that actually
// governs OCCT's internal parallelism. An inline function-local static gives a
// single instance across every TU that includes this header.
inline Scheduler& sharedScheduler() {
    static Scheduler pool(static_cast<unsigned>(ParallelPolicy::instance().resolvedWorkerCount()));
    return pool;
}

// Run `body(OperationContext&) -> T` through the operation-scheduler off the
// caller's inline path, honouring the cancellation-safe boundary: the body runs
// to completion on a worker (OCCT's Build is not interruptible), but if the host
// cancelled meanwhile the computed result is discarded and OperationCancelled is
// thrown so the facade guard collapses it to 0/nil with resources reclaimed.
// A Failed body rethrows its recorded message so cc_last_error is preserved.
template <class Fn>
auto runScheduled(Fn&& body) -> std::invoke_result_t<Fn, OperationContext&> {
    using T = std::invoke_result_t<Fn, OperationContext&>;
    static_assert(!std::is_void_v<T>, "scheduled op must return a value");
    Operation<T> op = sharedScheduler().submit(std::forward<Fn>(body));
    const OperationStatus status = op.wait();
    if (status == OperationStatus::Completed) {
        return std::move(*op.result());
    }
    if (status == OperationStatus::Cancelled) {
        throw OperationCancelled{};
    }
    const Error err = op.error();
    throw std::runtime_error(err.message.empty() ? "scheduled operation failed" : err.message);
}

}  // namespace occt
}  // namespace cyber

#endif  // CYBERCADKERNEL_ENGINE_OCCT_PARALLEL_POLICY_H
