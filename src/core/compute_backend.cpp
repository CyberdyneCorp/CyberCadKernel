#include "compute_backend.h"

#include <algorithm>
#include <thread>
#include <vector>

namespace cyber {

CpuComputeBackend::CpuComputeBackend(unsigned max_threads)
    : threads_(max_threads != 0 ? max_threads
                                 : std::max(1u, std::thread::hardware_concurrency())) {}

void CpuComputeBackend::parallel_for(std::size_t count, const ParallelBody& body) {
    if (count == 0 || !body) {
        return;
    }
    // Small ranges or single-thread configs run serially to avoid spawn overhead.
    const unsigned workers = std::min<std::size_t>(threads_, count) <= 1
                                 ? 1u
                                 : static_cast<unsigned>(std::min<std::size_t>(threads_, count));
    if (workers == 1) {
        for (std::size_t i = 0; i < count; ++i) {
            body(i);
        }
        return;
    }

    const std::size_t chunk = (count + workers - 1) / workers;
    std::vector<std::thread> pool;
    pool.reserve(workers);
    for (unsigned w = 0; w < workers; ++w) {
        const std::size_t begin = static_cast<std::size_t>(w) * chunk;
        if (begin >= count) {
            break;
        }
        const std::size_t end = std::min(begin + chunk, count);
        pool.emplace_back([&body, begin, end]() {
            for (std::size_t i = begin; i < end; ++i) {
                body(i);
            }
        });
    }
    for (auto& t : pool) {
        t.join();
    }
}

ComputeRegistry::ComputeRegistry() {
    cpu_ = std::make_shared<CpuComputeBackend>();
    backends_.emplace(cpu_->name(), cpu_);
    active_ = cpu_;
}

ComputeRegistry& ComputeRegistry::instance() {
    static ComputeRegistry registry;
    return registry;
}

void ComputeRegistry::register_backend(std::shared_ptr<IComputeBackend> backend) {
    if (!backend) {
        return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    backends_[backend->name()] = std::move(backend);
}

Result<void> ComputeRegistry::set_active(const std::string& name) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = backends_.find(name);
    if (it == backends_.end()) {
        return make_error("unknown compute backend: " + name);
    }
    active_ = it->second;
    return Result<void>::ok();
}

std::shared_ptr<IComputeBackend> ComputeRegistry::active() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return active_;
}

std::shared_ptr<IComputeBackend> ComputeRegistry::cpu() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cpu_;
}

Result<std::shared_ptr<IComputeBackend>> ComputeRegistry::backend_for(Precision precision) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (precision == Precision::Fp32) {
        return active_;
    }
    // Fp64: exact modeling stays on a double-capable backend. Prefer the active
    // one only if it supports fp64; otherwise fall back to the CPU. It is NEVER
    // dispatched to an fp32-only backend.
    if (active_->supports_fp64()) {
        return active_;
    }
    if (cpu_ && cpu_->supports_fp64()) {
        return cpu_;
    }
    return make_error("no fp64-capable compute backend available for exact work");
}

}  // namespace cyber
