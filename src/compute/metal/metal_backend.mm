// Metal compute backend implementation (Objective-C++, iOS-only).
//
// Compiled only when CYBERCAD_HAS_METAL is set (see CMakeLists.txt), with ARC and
// -framework Metal -framework Foundation. All Objective-C / Metal types are
// confined to this translation unit; the header trades only opaque void* handles.
// See metal_backend.h for the API contract and the fp32/fp64 boundary rationale.

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "compute/metal/metal_backend.h"
#include "core/compute_backend.h"

namespace cyber::metal {
namespace {

// A trivial fp32 elementwise add kernel, compiled at runtime by the self-test to
// prove device init, buffer round-trip, pipeline compilation, and dispatch.
constexpr const char* kSelfTestKernel = R"MSL(
#include <metal_stdlib>
using namespace metal;
kernel void cc_selftest_add(device const float* a  [[buffer(0)]],
                            device const float* b  [[buffer(1)]],
                            device float*       out [[buffer(2)]],
                            uint gid [[thread_position_in_grid]]) {
    out[gid] = a[gid] + b[gid];
}
)MSL";

// Cache key: fold the kernel source and function name into one hash so identical
// (source, fn) pairs reuse the same compiled pipeline.
std::size_t pipeline_key(const std::string& source, const std::string& fn) {
    std::size_t h = std::hash<std::string>{}(source);
    h ^= std::hash<std::string>{}(fn) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
}

// Build a readable error message from an NSError (may be nil).
Error metal_error(const char* what, NSError* err) {
    std::string msg = what;
    if (err != nil) {
        msg += ": ";
        msg += err.localizedDescription.UTF8String;
    }
    return make_error(std::move(msg));
}

}  // namespace

// Owns the device, queue, and the strong references that keep returned buffer /
// pipeline handles alive. Foundation collections retain their contents, so a
// void* handle stays valid for the lifetime of the backend.
struct MetalBackend::Impl {
    id<MTLDevice> device = nil;
    id<MTLCommandQueue> queue = nil;
    NSMutableArray<id<MTLBuffer>>* buffers = nil;
    NSMutableDictionary<NSNumber*, id<MTLComputePipelineState>>* pipelines = nil;
};

MetalBackend::MetalBackend() : impl_(std::make_unique<Impl>()) {
    impl_->buffers = [NSMutableArray array];
    impl_->pipelines = [NSMutableDictionary dictionary];
}

MetalBackend::~MetalBackend() = default;

std::shared_ptr<MetalBackend> MetalBackend::create() {
    id<MTLDevice> dev = MTLCreateSystemDefaultDevice();
    if (dev == nil) {
        return nullptr;  // No Metal device: caller falls back to the CPU backend.
    }
    id<MTLCommandQueue> q = [dev newCommandQueue];
    if (q == nil) {
        return nullptr;
    }
    std::shared_ptr<MetalBackend> backend(new MetalBackend());
    backend->impl_->device = dev;
    backend->impl_->queue = q;
    return backend;
}

void MetalBackend::parallel_for(std::size_t count, const ParallelBody& body) {
    // Interface conformance: the CPU drives the loop. Data-parallel GPU value
    // comes from the buffer/pipeline/dispatch primitives the sibling modules use.
    if (count == 0 || !body) {
        return;
    }
    for (std::size_t i = 0; i < count; ++i) {
        body(i);
    }
}

DeviceHandle MetalBackend::device() const {
    return (__bridge void*)impl_->device;
}

QueueHandle MetalBackend::queue() const {
    return (__bridge void*)impl_->queue;
}

Result<PipelineHandle> MetalBackend::compilePipeline(const std::string& mslSource,
                                                     const std::string& fnName) {
    NSNumber* key = @(pipeline_key(mslSource, fnName));
    if (id<MTLComputePipelineState> cached = impl_->pipelines[key]) {
        return static_cast<PipelineHandle>((__bridge void*)cached);  // cache hit
    }

    NSError* err = nil;
    NSString* source = [NSString stringWithUTF8String:mslSource.c_str()];
    id<MTLLibrary> lib = [impl_->device newLibraryWithSource:source options:nil error:&err];
    if (lib == nil) {
        return metal_error("MSL compile failed", err);
    }

    id<MTLFunction> func = [lib newFunctionWithName:[NSString stringWithUTF8String:fnName.c_str()]];
    if (func == nil) {
        return make_error("MSL function not found: " + fnName);
    }

    id<MTLComputePipelineState> pso =
        [impl_->device newComputePipelineStateWithFunction:func error:&err];
    if (pso == nil) {
        return metal_error("pipeline creation failed", err);
    }

    impl_->pipelines[key] = pso;
    return static_cast<PipelineHandle>((__bridge void*)pso);
}

BufferHandle MetalBackend::makeSharedBuffer(const void* bytes, std::size_t len) {
    if (len == 0) {
        return nullptr;
    }
    id<MTLBuffer> buf =
        bytes ? [impl_->device newBufferWithBytes:bytes
                                           length:len
                                          options:MTLResourceStorageModeShared]
              : [impl_->device newBufferWithLength:len options:MTLResourceStorageModeShared];
    if (buf == nil) {
        return nullptr;
    }
    [impl_->buffers addObject:buf];  // keep it alive for the backend's lifetime
    return (__bridge void*)buf;
}

BufferHandle MetalBackend::makeSharedBuffer(std::size_t len) {
    return makeSharedBuffer(nullptr, len);
}

void* MetalBackend::bufferContents(BufferHandle handle) const {
    if (handle == nullptr) {
        return nullptr;
    }
    id<MTLBuffer> buf = (__bridge id<MTLBuffer>)handle;
    return [buf contents];
}

Result<void> MetalBackend::dispatch(PipelineHandle pipeline,
                                    const std::vector<BufferHandle>& buffers,
                                    std::size_t gridSize,
                                    std::size_t threadgroupSize,
                                    Precision precision) {
    // Defensive fp32 boundary: never silently downcast exact fp64 work.
    if (precision == Precision::Fp64) {
        return make_error("Metal backend refuses fp64 work (fp32-only GPU)");
    }
    if (pipeline == nullptr) {
        return make_error("dispatch: null pipeline");
    }
    if (gridSize == 0) {
        return Result<void>::ok();
    }

    id<MTLComputePipelineState> pso = (__bridge id<MTLComputePipelineState>)pipeline;
    id<MTLCommandBuffer> cmd = [impl_->queue commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cmd computeCommandEncoder];
    [enc setComputePipelineState:pso];
    for (std::size_t i = 0; i < buffers.size(); ++i) {
        [enc setBuffer:(__bridge id<MTLBuffer>)buffers[i] offset:0 atIndex:i];
    }

    NSUInteger tg = threadgroupSize != 0 ? static_cast<NSUInteger>(threadgroupSize)
                                         : pso.maxTotalThreadsPerThreadgroup;
    tg = std::max<NSUInteger>(1, std::min<NSUInteger>(tg, gridSize));

    // Non-uniform threadgroups: Metal splits the trailing partial group, so the
    // grid need not be a multiple of the threadgroup size.
    [enc dispatchThreads:MTLSizeMake(gridSize, 1, 1)
        threadsPerThreadgroup:MTLSizeMake(tg, 1, 1)];
    [enc endEncoding];
    [cmd commit];
    [cmd waitUntilCompleted];

    if (cmd.status == MTLCommandBufferStatusError) {
        return metal_error("GPU dispatch failed", cmd.error);
    }
    return Result<void>::ok();
}

std::shared_ptr<IComputeBackend> makeMetalBackend() {
    return MetalBackend::create();
}

bool registerMetalBackend() {
    auto backend = MetalBackend::create();
    if (!backend) {
        return false;  // No device: CPU stays the active/fallback backend.
    }
    ComputeRegistry::instance().register_backend(backend);
    return true;
}

bool metal_backend_selftest() {
    auto backend = MetalBackend::create();
    if (!backend) {
        return false;
    }

    constexpr std::size_t kN = 1024;
    std::vector<float> a(kN), b(kN);
    for (std::size_t i = 0; i < kN; ++i) {
        a[i] = static_cast<float>(i);
        b[i] = static_cast<float>(2 * i) + 1.0f;
    }

    const std::size_t bytes = kN * sizeof(float);
    BufferHandle ha = backend->makeSharedBuffer(a.data(), bytes);
    BufferHandle hb = backend->makeSharedBuffer(b.data(), bytes);
    BufferHandle hout = backend->makeSharedBuffer(bytes);
    if (!ha || !hb || !hout) {
        return false;
    }

    auto pso = backend->compilePipeline(kSelfTestKernel, "cc_selftest_add");
    if (!pso) {
        return false;
    }
    if (!backend->dispatch(pso.value(), {ha, hb, hout}, kN)) {
        return false;
    }

    const auto* out = static_cast<const float*>(backend->bufferContents(hout));
    if (out == nullptr) {
        return false;
    }
    for (std::size_t i = 0; i < kN; ++i) {
        const float expect = a[i] + b[i];
        if (std::fabs(out[i] - expect) > 1e-4f * std::max(1.0f, std::fabs(expect))) {
            return false;
        }
    }
    return true;
}

}  // namespace cyber::metal
