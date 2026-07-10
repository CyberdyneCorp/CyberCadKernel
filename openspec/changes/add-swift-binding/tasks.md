# Tasks — add-swift-binding

## 1. Packaging
- [ ] 1.1 SwiftPM package `CyberCadKernel` with a binary-target dependency on the iOS/macOS
      xcframework (`scripts/build-xcframework.sh` output) and a C module map over `cc_kernel.h`.
- [ ] 1.2 Package builds for macOS arm64 (dev/test/showcase) and iOS device + simulator.

## 2. Low-level 1:1 layer
- [ ] 2.1 Surface every `cc_*` function + POD struct to Swift via the C module map (nothing hidden).

## 3. Ergonomic Swift API
- [ ] 3.1 `Kernel` + `Shape` object model; handle lifetime auto-released on `deinit`.
- [ ] 3.2 Throwing functions that raise a Swift `Error` from `cc_last_error` (no `0`/`nil` returns).
- [ ] 3.3 Swift value types for POD structs; mesh/point data as `[SIMD3<Float>]` / `Data`.

## 4. Tests
- [ ] 4.1 XCTest suite asserting REAL geometry through Swift (extrude/boolean volumes, mass
      properties, STEP round-trip, tessellation) — the Swift mirror of the Python pytest suite.

## 5. Docs & migration path
- [ ] 5.1 `docs/swift.md` — package integration + the app-migration path (replace
      `KernelBridge.mm` + `BRepShape` bridging with the package).

## 6. Close-out
- [ ] 6.1 `openspec validate --all --strict`; no `cc_*` ABI change; app untouched.
