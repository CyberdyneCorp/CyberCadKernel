#ifndef CYBERCADKERNEL_COMPUTE_METAL_METAL_BACKEND_H
#define CYBERCADKERNEL_COMPUTE_METAL_METAL_BACKEND_H

// Metal compute backend (iOS-only, built only when CYBERCAD_HAS_METAL is set).
//
// MetalBackend is a drop-in `IComputeBackend` (Phase 0) so callers, the cc_*
// facade, and the engine adapter never change. On top of the portable interface
// it exposes a small GPU primitive API — shared (unified-memory) buffers,
// runtime-compiled MSL pipelines (cached), and index-range dispatch — that the
// sibling Phase-2 GPU modules (tessellation, BVH, picking) build on.
//
// Objective-C / Metal types are kept OUT of this header: every GPU object is
// handed back as an opaque `void*` handle whose lifetime is owned by the backend
// (buffers and pipelines live in the backend's caches). Sibling .mm modules
// bridge the handle back to the concrete `id<MTL...>` inside their own .mm TUs.
// This header therefore compiles as plain C++20 in any translation unit.
//
// Apple GPUs are fp32-only: `supports_fp64()` is false and the backend refuses
// any explicitly fp64-typed dispatch, so the Phase-0 precision guard always keeps
// exact double-precision modeling on the CPU (the source of truth).

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "core/compute_backend.h"
#include "core/result.h"

namespace cyber::metal {

// Opaque, non-owning handles to Metal objects. In a .mm translation unit these
// are the corresponding `id<MTL...>`; portable C++ only passes them around. They
// stay valid for as long as the owning MetalBackend (and its caches) live.
using DeviceHandle = void*;    // id<MTLDevice>
using QueueHandle = void*;     // id<MTLCommandQueue>
using PipelineHandle = void*;  // id<MTLComputePipelineState>
using BufferHandle = void*;    // id<MTLBuffer>

// A Metal-backed IComputeBackend plus the GPU primitives sibling modules call.
// Construct only through create()/makeMetalBackend(): construction fails cleanly
// (returns null) when no Metal device is available, so CPU-only paths are safe.
class MetalBackend final : public IComputeBackend {
public:
    // Acquire the system default Metal device + a command queue. Returns null
    // (no crash) when MTLCreateSystemDefaultDevice() yields no device.
    static std::shared_ptr<MetalBackend> create();

    ~MetalBackend() override;

    MetalBackend(const MetalBackend&) = delete;
    MetalBackend& operator=(const MetalBackend&) = delete;

    // ── IComputeBackend ──────────────────────────────────────────────────────
    std::string name() const override { return "metal"; }
    bool supports_fp64() const override { return false; }
    // Interface conformance: runs body(i) for i in [0, count). Provided so Metal
    // is interchangeable with the CPU backend; the GPU value is the primitives
    // below, which the sibling algorithm modules use directly.
    void parallel_for(std::size_t count, const ParallelBody& body) override;

    // ── Internal GPU API (used by sibling GPU modules) ───────────────────────

    // The shared device + command queue owned by this backend.
    DeviceHandle device() const;
    QueueHandle queue() const;

    // Compile `fnName` out of MSL `mslSource` into a compute pipeline state,
    // caching by a hash of (source, fnName) so repeated dispatches never
    // recompile. A source that fails to compile returns an Error, not a crash.
    Result<PipelineHandle> compilePipeline(const std::string& mslSource,
                                           const std::string& fnName);

    // Allocate a unified-memory (MTLResourceStorageModeShared) buffer the CPU and
    // GPU address without an explicit copy. The byte-filling overload copies
    // `len` bytes from `bytes` into the new shared allocation. Returns nullptr on
    // failure (e.g. len == 0). The buffer is owned by the backend.
    BufferHandle makeSharedBuffer(const void* bytes, std::size_t len);
    BufferHandle makeSharedBuffer(std::size_t len);

    // CPU-visible pointer to a shared buffer's contents (unified memory). Returns
    // nullptr for a null/unknown handle.
    void* bufferContents(BufferHandle handle) const;

    // Encode + dispatch `pipeline` over the index range [0, gridSize): bind each
    // buffer at its slot index, size threadgroups (from the pipeline's
    // maxTotalThreadsPerThreadgroup when threadgroupSize == 0), commit and wait.
    // `precision` is a defensive fp32 boundary — an Fp64 request is refused
    // (Error) rather than silently downcast.
    Result<void> dispatch(PipelineHandle pipeline,
                          const std::vector<BufferHandle>& buffers,
                          std::size_t gridSize,
                          std::size_t threadgroupSize = 0,
                          Precision precision = Precision::Fp32);

private:
    MetalBackend();

    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Factory: build a Metal backend, or nullptr when no Metal device is available
// (leaving the CPU backend as the active/fallback backend). Exposed as the
// portable IComputeBackend type for callers that only need the interface.
std::shared_ptr<IComputeBackend> makeMetalBackend();

// Create the Metal backend (if available) and register it with the Phase-0
// ComputeRegistry. The CPU backend stays registered as the default + fallback.
// Returns true iff a Metal backend was created and registered.
bool registerMetalBackend();

// Self-test used to gate the backend: creates a device, compiles a trivial fp32
// add kernel at runtime, dispatches it over a shared buffer, and verifies the
// result equals the CPU reference. Returns false (no crash) if Metal is
// unavailable or any step fails.
bool metal_backend_selftest();

}  // namespace cyber::metal

#endif  // CYBERCADKERNEL_COMPUTE_METAL_METAL_BACKEND_H
