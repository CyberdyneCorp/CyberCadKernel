#include "scheduler.h"

#include <algorithm>

namespace cyber {

Scheduler::Scheduler(unsigned workers) {
    const unsigned count = workers != 0 ? workers
                                        : std::max(1u, std::thread::hardware_concurrency());
    workers_.reserve(count);
    for (unsigned i = 0; i < count; ++i) {
        workers_.emplace_back([this] { worker_loop(); });
    }
}

Scheduler::~Scheduler() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
    }
    cv_.notify_all();
    // Join the workers HERE, while mutex_/cv_ are still alive. std::thread does
    // not auto-join, and destroying a joinable thread calls std::terminate, so
    // join each explicitly before clearing. Doing it here (rather than in member
    // destruction) also guarantees the workers stop touching cv_/mutex_ before
    // those are destroyed (workers_ is declared first, so destroyed last).
    for (std::thread& worker : workers_) {
        if (worker.joinable()) {
            worker.join();
        }
    }
    workers_.clear();
}

void Scheduler::enqueue(std::function<void()> job) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        jobs_.push(std::move(job));
    }
    cv_.notify_one();
}

void Scheduler::worker_loop() {
    for (;;) {
        std::function<void()> job;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
            if (jobs_.empty()) {
                if (stopping_) {
                    return;
                }
                continue;
            }
            job = std::move(jobs_.front());
            jobs_.pop();
        }
        job();
    }
}

}  // namespace cyber
