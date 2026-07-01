#ifndef CYBERCADKERNEL_CORE_SCHEDULER_H
#define CYBERCADKERNEL_CORE_SCHEDULER_H

// Operation scheduler: runs kernel operations off the host UI thread on a
// std::thread worker pool, with cooperative cancellation (an in-house StopToken;
// see below) and a 0..1 / staged progress sink.
//
//   * Task<T>            — a C++20 coroutine type for authoring operations; its
//                          body co_awaits OperationContext::checkpoint() to report
//                          progress and honour cancellation, and can be driven
//                          synchronously via sync_wait().
//   * Operation<T>       — the host-side handle: request_cancel / wait / status /
//                          result, plus a progress feed.
//   * Scheduler          — the std::thread pool; submit() (plain callable) and
//                          submit_coroutine() (Task factory) go off-thread,
//                          run_sync() is the synchronous shim.
//
// Cancellation-safe engine boundary: most engine calls (OCCT booleans, meshing)
// are not interruptible mid-computation. The body runs to completion on a worker;
// if the host cancelled meanwhile, the scheduler DISCARDS the computed result,
// reports a Cancelled outcome, and reclaims resources — the caller never sees a
// result it cancelled.

#include <atomic>
#include <condition_variable>
#include <coroutine>
#include <cstddef>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "result.h"

namespace cyber {

// Terminal + transient states of a submitted operation.
enum class OperationStatus { Pending, Running, Completed, Cancelled, Failed };

inline bool is_terminal(OperationStatus s) {
    return s == OperationStatus::Completed || s == OperationStatus::Cancelled ||
           s == OperationStatus::Failed;
}

// Thrown at a cooperative checkpoint when cancellation has been requested.
class OperationCancelled : public std::exception {
public:
    const char* what() const noexcept override { return "operation cancelled"; }
};

// Progress feed: fraction in [0,1] plus an optional stage label. Thread-safe;
// the callback (if any) is invoked from the worker thread.
class ProgressSink {
public:
    using Callback = std::function<void(double fraction, std::string_view stage)>;

    ProgressSink() = default;
    explicit ProgressSink(Callback cb) : callback_(std::move(cb)) {}

    void report(double fraction) { report(fraction, std::string_view{}); }

    void report(double fraction, std::string_view stage) {
        if (fraction < 0.0) fraction = 0.0;
        if (fraction > 1.0) fraction = 1.0;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            fraction_ = fraction;
            if (!stage.empty()) {
                stage_.assign(stage);
            }
        }
        if (callback_) {
            callback_(fraction, stage);
        }
    }

    double fraction() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return fraction_;
    }

    std::string stage() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return stage_;
    }

private:
    mutable std::mutex mutex_;
    Callback callback_;
    double fraction_ = 0.0;
    std::string stage_;
};

// ── Cooperative cancellation token ────────────────────────────────────────────
//
// std::stop_token / std::stop_source (and std::jthread) are gated behind libc++
// availability annotations that mark them unavailable before iOS 14 — they rely
// on the atomic-wait/notify runtime introduced in that OS. Our deployment target
// is iOS 13, and the scheduler only needs a plain request/observe flag (no
// stop_callback, no atomic wait), so we provide a minimal shared-flag token with
// no OS-version dependency. The observable API mirrors the std types we replace:
// StopSource::get_token / request_stop and StopToken::stop_requested.
class StopSource;

class StopToken {
public:
    StopToken() = default;
    bool stop_requested() const noexcept {
        return flag_ && flag_->load(std::memory_order_acquire);
    }

private:
    friend class StopSource;
    explicit StopToken(std::shared_ptr<std::atomic<bool>> flag)
        : flag_(std::move(flag)) {}
    std::shared_ptr<std::atomic<bool>> flag_;
};

class StopSource {
public:
    StopSource() : flag_(std::make_shared<std::atomic<bool>>(false)) {}

    StopToken get_token() const noexcept { return StopToken(flag_); }

    // Returns true if this call transitioned the source into the stop state.
    bool request_stop() noexcept {
        return !flag_->exchange(true, std::memory_order_release);
    }

    bool stop_requested() const noexcept {
        return flag_->load(std::memory_order_acquire);
    }

private:
    std::shared_ptr<std::atomic<bool>> flag_;
};

// Passed to an operation body: exposes cooperative cancellation + progress.
class OperationContext {
public:
    OperationContext(StopToken token, ProgressSink* progress)
        : token_(std::move(token)), progress_(progress) {}

    StopToken token() const { return token_; }
    bool cancelled() const { return token_.stop_requested(); }

    void throw_if_cancelled() const {
        if (token_.stop_requested()) {
            throw OperationCancelled{};
        }
    }

    void report(double fraction, std::string_view stage = {}) {
        if (progress_) {
            progress_->report(fraction, stage);
        }
    }

    // Cooperative checkpoint for a Task<T> body: `co_await ctx.checkpoint(f)`
    // reports progress `f` and throws OperationCancelled if cancellation was
    // requested. It never truly suspends, so a Task drives to completion in one
    // resume (see Task::sync_wait).
    struct Checkpoint {
        OperationContext* ctx;
        double fraction;
        std::string_view stage;
        bool await_ready() const noexcept { return true; }
        void await_suspend(std::coroutine_handle<>) const noexcept {}
        void await_resume() const {
            ctx->report(fraction, stage);
            ctx->throw_if_cancelled();
        }
    };

    Checkpoint checkpoint(double fraction, std::string_view stage = {}) {
        return Checkpoint{this, fraction, stage};
    }

private:
    StopToken token_;
    ProgressSink* progress_;
};

// ── Task<T> coroutine ─────────────────────────────────────────────────────────

template <class T>
class Task {
public:
    struct promise_type {
        std::variant<std::monostate, T, std::exception_ptr> result_{};
        std::coroutine_handle<> continuation_{};

        Task get_return_object() noexcept {
            return Task{std::coroutine_handle<promise_type>::from_promise(*this)};
        }
        std::suspend_always initial_suspend() noexcept { return {}; }

        struct FinalAwaiter {
            bool await_ready() noexcept { return false; }
            std::coroutine_handle<> await_suspend(
                std::coroutine_handle<promise_type> h) noexcept {
                auto cont = h.promise().continuation_;
                return cont ? cont : std::noop_coroutine();
            }
            void await_resume() noexcept {}
        };
        FinalAwaiter final_suspend() noexcept { return {}; }

        void return_value(T value) { result_.template emplace<1>(std::move(value)); }
        void unhandled_exception() { result_.template emplace<2>(std::current_exception()); }
    };

    Task() = default;
    explicit Task(std::coroutine_handle<promise_type> handle) : handle_(handle) {}
    Task(Task&& other) noexcept : handle_(std::exchange(other.handle_, {})) {}
    Task& operator=(Task&& other) noexcept {
        if (this != &other) {
            destroy();
            handle_ = std::exchange(other.handle_, {});
        }
        return *this;
    }
    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;
    ~Task() { destroy(); }

    // Awaitable: lets one Task co_await another.
    bool await_ready() const noexcept { return !handle_ || handle_.done(); }
    std::coroutine_handle<> await_suspend(std::coroutine_handle<> awaiting) noexcept {
        handle_.promise().continuation_ = awaiting;
        return handle_;
    }
    T await_resume() { return take_result(); }

    // Synchronous shim: resume the (lazy) coroutine to completion and return its
    // value, or rethrow the exception it finished with.
    T sync_wait() {
        if (!handle_) {
            throw std::runtime_error("sync_wait on empty Task");
        }
        handle_.resume();
        return take_result();
    }

private:
    void destroy() {
        if (handle_) {
            handle_.destroy();
            handle_ = {};
        }
    }
    T take_result() {
        auto& slot = handle_.promise().result_;
        if (slot.index() == 2) {
            std::rethrow_exception(std::get<2>(slot));
        }
        return std::move(std::get<1>(slot));
    }

    std::coroutine_handle<promise_type> handle_{};
};

// ── Operation<T> handle + shared state ────────────────────────────────────────

template <class T>
struct OperationState {
    std::mutex mutex;
    std::condition_variable cv;
    OperationStatus status = OperationStatus::Pending;
    std::optional<T> value;
    Error error;
};

template <class T>
class Operation {
public:
    Operation(std::shared_ptr<OperationState<T>> state, StopSource stop,
              std::shared_ptr<ProgressSink> progress)
        : state_(std::move(state)), stop_(std::move(stop)), progress_(std::move(progress)) {}

    // Request cooperative cancellation. A checkpoint-aware body stops at its next
    // checkpoint; a non-interruptible body runs to completion but its result is
    // discarded at the boundary.
    void request_cancel() { stop_.request_stop(); }

    // Block until the operation reaches a terminal state; returns that state.
    OperationStatus wait() {
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->cv.wait(lock, [this] { return is_terminal(state_->status); });
        return state_->status;
    }

    OperationStatus status() const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        return state_->status;
    }

    // Result of a Completed operation; nullopt otherwise.
    std::optional<T> result() const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (state_->status == OperationStatus::Completed) {
            return state_->value;
        }
        return std::nullopt;
    }

    Error error() const {
        std::lock_guard<std::mutex> lock(state_->mutex);
        return state_->error;
    }

    ProgressSink& progress() { return *progress_; }

private:
    std::shared_ptr<OperationState<T>> state_;
    StopSource stop_;
    std::shared_ptr<ProgressSink> progress_;
};

// Runs `produce(ctx)` and records the outcome, applying the cancellation-safe
// boundary rule (discard the result if the host cancelled during a
// non-interruptible run).
template <class T, class Produce>
void run_operation(const std::shared_ptr<OperationState<T>>& state, Produce produce,
                   StopToken token, const std::shared_ptr<ProgressSink>& progress) {
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->status = OperationStatus::Running;
    }
    OperationContext ctx(token, progress.get());
    OperationStatus final_status = OperationStatus::Failed;
    std::optional<T> produced;
    Error error;
    try {
        T value = produce(ctx);
        if (token.stop_requested()) {
            final_status = OperationStatus::Cancelled;  // discard `value`
        } else {
            produced.emplace(std::move(value));
            final_status = OperationStatus::Completed;
        }
    } catch (const OperationCancelled&) {
        final_status = OperationStatus::Cancelled;
    } catch (const std::exception& e) {
        error = make_error(e.what());
    } catch (...) {
        error = make_error("unknown error");
    }
    {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->status = final_status;
        state->value = std::move(produced);
        state->error = std::move(error);
    }
    state->cv.notify_all();
}

// ── Scheduler (std::thread pool) ──────────────────────────────────────────────

class Scheduler {
public:
    explicit Scheduler(unsigned workers = 0);  // 0 => hardware_concurrency
    ~Scheduler();

    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;

    // Submit a plain callable `body(OperationContext&) -> T` to run off-thread.
    template <class Fn>
    auto submit(Fn body, ProgressSink::Callback progress_cb = {})
        -> Operation<std::invoke_result_t<Fn, OperationContext&>> {
        using T = std::invoke_result_t<Fn, OperationContext&>;
        static_assert(!std::is_void_v<T>, "operation body must return a value");
        auto state = std::make_shared<OperationState<T>>();
        auto progress = std::make_shared<ProgressSink>(std::move(progress_cb));
        StopSource stop;
        StopToken token = stop.get_token();
        enqueue([state, body = std::move(body), token, progress]() mutable {
            run_operation<T>(state, std::move(body), token, progress);
        });
        return Operation<T>(std::move(state), std::move(stop), std::move(progress));
    }

    // Submit a coroutine factory `factory(OperationContext&) -> Task<T>`. The
    // Task is driven to completion on a worker via sync_wait. T is given
    // explicitly (e.g. sched.submit_task<int>(...)).
    template <class T, class Factory>
    Operation<T> submit_task(Factory factory, ProgressSink::Callback progress_cb = {}) {
        auto state = std::make_shared<OperationState<T>>();
        auto progress = std::make_shared<ProgressSink>(std::move(progress_cb));
        StopSource stop;
        StopToken token = stop.get_token();
        enqueue([state, factory = std::move(factory), token, progress]() mutable {
            run_operation<T>(
                state,
                [&factory](OperationContext& ctx) {
                    Task<T> task = factory(ctx);
                    return task.sync_wait();
                },
                token, progress);
        });
        return Operation<T>(std::move(state), std::move(stop), std::move(progress));
    }

    // Synchronous shim: run the body inline on the caller's thread (no pool, no
    // cancellation) and return its value directly. Exceptions propagate.
    template <class Fn>
    auto run_sync(Fn body) -> std::invoke_result_t<Fn, OperationContext&> {
        StopSource stop;  // never signalled
        ProgressSink progress;
        OperationContext ctx(stop.get_token(), &progress);
        return body(ctx);
    }

    std::size_t worker_count() const { return workers_.size(); }

private:
    void enqueue(std::function<void()> job);
    void worker_loop();

    std::vector<std::thread> workers_;
    std::queue<std::function<void()>> jobs_;
    std::mutex mutex_;
    std::condition_variable cv_;
    bool stopping_ = false;
};

}  // namespace cyber

#endif  // CYBERCADKERNEL_CORE_SCHEDULER_H
