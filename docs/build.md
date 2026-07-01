# Building and testing CyberCadKernel

CyberCadKernel is a C++20 static library behind a stable plain-C ABI (`cc_*`).
It builds in two configurations:

- **Host, no OCCT** (`CYBERCAD_HAS_OCCT=OFF`, the default) — fully buildable and
  unit-testable on macOS/Linux with a C++20 compiler. Geometry is served by a
  no-op **stub engine** (`cc_brep_available()` returns 0; geometric `cc_*` return
  0/nil). This is the configuration CI and local development use.
- **iOS, with OCCT** (`CYBERCAD_HAS_OCCT=ON`) — additionally compiles the OCCT
  engine adapter under `src/engine/occt/`. This requires the trimmed OpenCASCADE
  static libraries cross-compiled for arm64 and is **not host-buildable**.

## Repository layout

```
include/cybercadkernel/cc_kernel.h   Public C ABI (57 cc_*, POD structs). Mirrors
                                     the app's KernelBridgeAPI.h byte-for-byte.
src/core/        result.h            Result<T,Error> (in-house; std::expected is C++23)
                 guard.{h,cpp}       Exception->status guard + thread-local last_error
                 shape_registry.*    CCShapeId <-> opaque EngineShape, thread-safe
                 scheduler.{h,cpp}   Task<T> coroutine, jthread pool, cancel, progress
                 compute_backend.*   IComputeBackend, CPU default, precision guard
src/engine/      IEngine.h           Internal engine interface (capability groups)
                 engine_registry.cpp Active-engine selector (OCCT/native coexist)
                 stub/               No-op engine (host build links without OCCT)
                 occt/               OCCT adapter (compiled only when HAS_OCCT=ON)
src/facade/      cc_kernel.cpp       Every cc_*: guard + delegate + registry + buffers
tests/           harness.h           Tiny assert-based CTest harness (no gtest)
                 test_*.cpp          registry / guard / scheduler / compute / ABI
docs/build.md    this file
```

Only `src/engine/occt/*.cpp` may include OpenCASCADE headers; no OCCT (or any
engine) type appears in a public or shared header.

## Host build + test (no OCCT)

The host build uses Homebrew LLVM `clang++` for complete C++20 support
(coroutines, `<stop_token>`, `<jthread>`, `<barrier>`, `<latch>`):

```sh
cmake -S . -B build \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
  -DCYBERCAD_HAS_OCCT=OFF
cmake --build build
cd build && ctest --output-on-failure
```

Expected: `100% tests passed, 0 tests failed out of 5`.

`CMAKE_CXX_COMPILER` is overridable — any compiler with full C++20 library
support works; Apple Clang from Xcode is used for the iOS cross-build.

### ABI contract test

`test_abi` `#include`s both this library's `cc_kernel.h` and the app's reference
`KernelBridgeAPI.h`, and `static_assert`s the size and field offsets of every POD
struct across the two headers, proving binary compatibility. The reference path
defaults to
`/Users/leonardoaraujo/work/cybercad/CyberCad/Kernel/Bridge/KernelBridgeAPI.h`
and is overridable:

```sh
cmake -S . -B build -DCYBERCAD_REFERENCE_ABI_HEADER=/path/to/KernelBridgeAPI.h ...
```

If the reference header is absent, the test degrades to a self-consistency +
link check (the layout comparison is skipped).

## iOS build (with OCCT)

```sh
cmake -S . -B build-ios \
  -DCMAKE_TOOLCHAIN_FILE=<ios-toolchain>.cmake \
  -DCYBERCAD_HAS_OCCT=ON
```

This adds `src/engine/occt/*.cpp` and defines `CYBERCAD_HAS_OCCT=1`. Supply the
OCCT include directories and static libraries via the iOS toolchain (see the app
repo's `docs/occt-build.md`). This configuration will not link on a host without
a matching OCCT.

## Notes

- Tests can be disabled with `-DCYBERCAD_BUILD_TESTS=OFF`.
- The library links `Threads::Threads` (the jthread pool and thread-safe
  registry).
