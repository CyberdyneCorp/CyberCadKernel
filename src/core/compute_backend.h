#ifndef CYBERCADKERNEL_CORE_COMPUTE_BACKEND_H
#define CYBERCADKERNEL_CORE_COMPUTE_BACKEND_H

// Compute-backend abstraction for data-parallel work (batch surface evaluation,
// BVH build/traversal, picking, mesh post-processing). A default CPU/multi-core
// backend ships here; Metal/CUDA/OpenCL/Vulkan backends register through the same
// interface without changing callers. A precision guard refuses to route exact
// fp64 modeling work to an fp32-only (GPU) backend — CPU stays the source of
// truth.

#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "result.h"

namespace cyber {

// Work classes the backends accelerate. Callers express intent abstractly so the
// active backend can be swapped without touching the call site.
enum class ComputeKind {
    SurfaceEval,      // batch parametric surface evaluation
    BvhBuild,         // acceleration-structure construction
    Picking,          // ray/pick queries against a BVH
    MeshPostProcess,  // normal/smoothing/decimation passes on a mesh
};

// Numeric precision a job needs. Exact modeling is Fp64; display-tolerant,
// data-parallel work is Fp32.
enum class Precision { Fp32, Fp64 };

// A unit of data-parallel work: a body invoked for indices [0, count).
using ParallelBody = std::function<void(std::size_t index)>;

// Interface every compute backend implements. Correctness must not depend on
// which backend is active (only throughput / fp32-tolerance differs).
class IComputeBackend {
public:
    virtual ~IComputeBackend() = default;

    virtual std::string name() const = 0;

    // True if the backend can represent double precision. Apple GPUs cannot, so
    // a Metal backend returns false and the precision guard keeps fp64 on CPU.
    virtual bool supports_fp64() const = 0;

    // Run `body(i)` for i in [0, count). Implementations may parallelize; the
    // observable result must match a serial run.
    virtual void parallel_for(std::size_t count, const ParallelBody& body) = 0;
};

// Default CPU/multi-core backend. Always fp64-capable; present in every build so
// no capability depends on a GPU being available.
class CpuComputeBackend final : public IComputeBackend {
public:
    explicit CpuComputeBackend(unsigned max_threads = 0);  // 0 => hardware_concurrency

    std::string name() const override { return "cpu"; }
    bool supports_fp64() const override { return true; }
    void parallel_for(std::size_t count, const ParallelBody& body) override;

private:
    unsigned threads_;
};

// Runtime registry + active-backend selector and the precision guard. GPU
// backends register here at startup; the facade/engine dispatch through it.
class ComputeRegistry {
public:
    static ComputeRegistry& instance();

    // Register a backend under its name(). The first registered backend (the CPU
    // default) also becomes active until set_active overrides it.
    void register_backend(std::shared_ptr<IComputeBackend> backend);

    // Select the active backend by name. Fails if the name is unknown.
    Result<void> set_active(const std::string& name);

    // The active backend (never null: the CPU default is registered in ctor).
    std::shared_ptr<IComputeBackend> active() const;

    // The CPU (fp64-capable) backend — the fallback for exact work.
    std::shared_ptr<IComputeBackend> cpu() const;

    // Precision guard: pick a backend for `precision`. Fp64 work is only ever
    // routed to an fp64-capable backend (the CPU when the active one is
    // fp32-only); it is NEVER dispatched to an fp32-only GPU backend. Fp32 work
    // uses the active backend.
    Result<std::shared_ptr<IComputeBackend>> backend_for(Precision precision) const;

private:
    ComputeRegistry();

    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::shared_ptr<IComputeBackend>> backends_;
    std::shared_ptr<IComputeBackend> active_;
    std::shared_ptr<IComputeBackend> cpu_;
};

}  // namespace cyber

#endif  // CYBERCADKERNEL_CORE_COMPUTE_BACKEND_H
