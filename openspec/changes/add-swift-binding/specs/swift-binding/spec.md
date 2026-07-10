# swift-binding

## ADDED Requirements

### Requirement: SwiftPM package over the xcframework with a low-level 1:1 `cc_*` layer

The kernel SHALL provide a SwiftPM package `CyberCadKernel` that depends on the kernel's
iOS/macOS xcframework and exposes, via a C module map over `cc_kernel.h`, a low-level layer
surfacing every `cc_*` function and POD struct to Swift. Nothing in the C ABI SHALL be hidden
from this layer, so a consumer can always fall through to the raw call, and the binding SHALL
remain valid as the engine behind the facade evolves (it depends only on the stable ABI).

#### Scenario: Every `cc_*` symbol is reachable from Swift

- GIVEN the `CyberCadKernel` SwiftPM package built against the xcframework
- WHEN a Swift consumer imports the low-level module
- THEN every `cc_*` function and POD struct declared in `cc_kernel.h` SHALL be callable/constructible from Swift

#### Scenario: The package builds for macOS and iOS targets

- GIVEN the package and the xcframework (device + simulator + macOS arm64)
- WHEN the package is built for macOS arm64 and for iOS device and simulator
- THEN each target SHALL build and link successfully against the xcframework

### Requirement: Ergonomic Swift API with automatic lifetime and thrown errors

The package SHALL provide an ergonomic Swift API layered over the low-level bindings: a
`Kernel`/`Shape` object model whose handle lifetime is released automatically on `deinit`,
Swift value types for POD structs, mesh/point data exposed as Swift arrays
(`[SIMD3<Float>]` / `Data`), and functions that THROW a Swift `Error` carrying `cc_last_error`
rather than returning a `0`/`nil` sentinel. A caller SHALL NOT need to manage `cc_shape_*`
handle release manually or inspect integer/`nil` return codes to detect failure.

#### Scenario: A shape handle is released automatically

- GIVEN a `Shape` created through the ergonomic API
- WHEN the `Shape` value goes out of scope
- THEN its underlying `cc_*` handle SHALL be released exactly once, with no manual release call and no leak

#### Scenario: A failing operation throws rather than returning a sentinel

- GIVEN an operation that fails in the engine (setting `cc_last_error`)
- WHEN it is invoked through the ergonomic Swift API
- THEN the call SHALL throw a Swift `Error` carrying the `cc_last_error` message, instead of returning `0`/`nil`

### Requirement: XCTest suite asserting real geometry through Swift

The package SHALL ship an XCTest suite that drives the kernel through the Swift API and
asserts REAL geometric properties — extrude/boolean volumes, mass properties, STEP round-trip,
and tessellation — mirroring the desktop Python `pytest` suite, so the Swift binding is proven
against real geometry, not merely compiled.

#### Scenario: The Swift test suite asserts real geometry

- GIVEN the XCTest suite built against a real-engine kernel build
- WHEN it runs box/extrude/boolean/mass-property/STEP/tessellation tests through the Swift API
- THEN the asserted geometric quantities SHALL match the known values (e.g. a unit box volume and surface area), proving the binding drives real geometry

### Requirement: The Swift binding is optional for the app and does not alter the ABI

The Swift binding SHALL be a pure consumer of the `cc_*` ABI and the xcframework and SHALL NOT
require any change to that ABI, to `src/native`, or to the app. The CyberCad app MAY adopt the
package to replace its hand-written `KernelBridge.mm`/`BRepShape` bridge, but adoption SHALL
remain optional — the `ios-app-compat` ABI-parity strategy SHALL keep the app working whether
or not it adopts this binding.

#### Scenario: Introducing the binding changes neither the ABI nor the app

- GIVEN the Swift binding package is added to the kernel repository
- WHEN it is built and tested
- THEN the `cc_*` ABI header and `src/native` SHALL be unchanged, and the CyberCad app SHALL continue to build and run against the kernel with its existing bridge, unaffected by the package's existence
