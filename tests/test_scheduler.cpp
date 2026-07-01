// Operation scheduler: off-thread submit + progress, cooperative cancellation,
// the cancellation-safe engine boundary (discard a non-interruptible result on
// cancel), the Task<T> coroutine (checkpoint progress/cancel + sync_wait), and
// the synchronous shim.

#include <atomic>
#include <chrono>
#include <thread>

#include "core/scheduler.h"
#include "harness.h"

using namespace std::chrono_literals;
using cyber::OperationCancelled;
using cyber::OperationContext;
using cyber::OperationStatus;
using cyber::ProgressSink;
using cyber::Scheduler;
using cyber::StopSource;
using cyber::Task;

namespace {
void spin_until(const std::atomic<bool>& flag) {
    while (!flag.load()) {
        std::this_thread::yield();
    }
}
}  // namespace

CC_TEST(submit_completes_and_reports_progress) {
    Scheduler sched(2);
    std::atomic<double> last{-1.0};
    auto op = sched.submit(
        [](OperationContext& ctx) -> int {
            ctx.report(0.25, "start");
            ctx.report(1.0, "done");
            return 42;
        },
        [&last](double f, std::string_view) { last.store(f); });

    CC_CHECK_EQ(op.wait(), OperationStatus::Completed);
    auto result = op.result();
    CC_CHECK(result.has_value());
    CC_CHECK_EQ(result.value(), 42);
    CC_CHECK_EQ(last.load(), 1.0);
    CC_CHECK_EQ(op.progress().fraction(), 1.0);
}

CC_TEST(cooperative_cancel_stops_at_checkpoint) {
    Scheduler sched(2);
    std::atomic<bool> started{false};
    auto op = sched.submit([&started](OperationContext& ctx) -> int {
        started.store(true);
        for (;;) {
            ctx.throw_if_cancelled();
            std::this_thread::sleep_for(1ms);
        }
    });
    spin_until(started);
    op.request_cancel();
    CC_CHECK_EQ(op.wait(), OperationStatus::Cancelled);
    CC_CHECK(!op.result().has_value());
}

CC_TEST(noninterruptible_result_discarded_on_cancel) {
    Scheduler sched(2);
    std::atomic<bool> started{false};
    std::atomic<bool> proceed{false};
    // Body ignores cancellation (models a non-interruptible engine call) and
    // computes a value; the scheduler must discard it because cancel was asked.
    auto op = sched.submit([&](OperationContext& ctx) -> int {
        (void)ctx;
        started.store(true);
        while (!proceed.load()) {
            std::this_thread::sleep_for(1ms);
        }
        return 77;
    });
    spin_until(started);
    op.request_cancel();
    proceed.store(true);
    CC_CHECK_EQ(op.wait(), OperationStatus::Cancelled);
    CC_CHECK(!op.result().has_value());  // result reclaimed, not surfaced
}

CC_TEST(failed_body_reports_failed_status) {
    Scheduler sched(1);
    auto op = sched.submit([](OperationContext&) -> int {
        throw std::runtime_error("kaboom");
    });
    CC_CHECK_EQ(op.wait(), OperationStatus::Failed);
    CC_CHECK(!op.result().has_value());
    CC_CHECK_EQ(op.error().message, std::string("kaboom"));
}

CC_TEST(task_coroutine_runs_via_submit_task) {
    Scheduler sched(2);
    auto op = sched.submit_task<int>([](OperationContext& ctx) -> Task<int> {
        co_await ctx.checkpoint(0.5, "half");
        co_return 9;
    });
    CC_CHECK_EQ(op.wait(), OperationStatus::Completed);
    CC_CHECK(op.result().has_value());
    CC_CHECK_EQ(op.result().value(), 9);
}

CC_TEST(task_sync_wait_drives_coroutine) {
    StopSource stop;
    ProgressSink progress;
    OperationContext ctx(stop.get_token(), &progress);
    auto task = [](OperationContext& c) -> Task<int> {
        co_await c.checkpoint(1.0);
        co_return 123;
    }(ctx);
    CC_CHECK_EQ(task.sync_wait(), 123);
}

CC_TEST(task_checkpoint_throws_when_cancelled) {
    StopSource stop;
    stop.request_stop();
    ProgressSink progress;
    OperationContext ctx(stop.get_token(), &progress);
    auto task = [](OperationContext& c) -> Task<int> {
        co_await c.checkpoint(0.5);
        co_return 1;
    }(ctx);
    bool threw = false;
    try {
        (void)task.sync_wait();
    } catch (const OperationCancelled&) {
        threw = true;
    }
    CC_CHECK(threw);
}

CC_TEST(run_sync_shim_runs_inline) {
    Scheduler sched(1);
    int v = sched.run_sync([](OperationContext& ctx) -> int {
        ctx.report(1.0);
        return 5;
    });
    CC_CHECK_EQ(v, 5);
}

CC_RUN_ALL()
