# ios-app-compat

## ADDED Requirements

### Requirement: The `cc_*` C ABI is the sole iOS integration seam

The kernel SHALL expose exactly one integration boundary to the iOS app: the plain-C `cc_*`
ABI declared in `include/cybercadkernel/cc_kernel.h`, consisting of integer shape handles and
POD structs. No C++ type, engine type, or STL container SHALL appear in that boundary. The iOS
app SHALL be able to consume the kernel entirely through this C surface from its own
Objective-C++/Swift bridge, with no requirement that the kernel ship any higher-level binding.

#### Scenario: The app links the kernel through the C ABI only

- GIVEN the CyberCad iOS app with its own Objective-C++ bridge (`KernelBridge.mm`) over `cc_*`
- WHEN the app links the kernel product and calls the kernel through `cc_kernel.h`
- THEN no C++ or engine type SHALL cross the boundary, and the app SHALL require no kernel-provided Swift or Objective-C API to build against the kernel

### Requirement: The kernel ABI is a verified superset of the app's bridge header

The kernel's shipped ABI (`cc_kernel.h`) SHALL declare every `cc_*` symbol that the app's
`CyberCad/Kernel/Bridge/KernelBridgeAPI.h` declares and its Swift layer calls, so that the app
can link the kernel product with all existing call-sites unchanged. Any `cc_*` symbol the app
actively calls SHALL be present in the kernel ABI; a change that removes or alters the
signature of such a symbol SHALL be treated as an app-breaking change.

#### Scenario: Every app-called symbol resolves against the kernel product

- GIVEN the set of `cc_*` symbols the app's Swift `BRepShape`/`KernelService` actively call
- WHEN the app is linked against the kernel product static library / xcframework
- THEN every such symbol SHALL resolve, with no unresolved-symbol link error, and the app SHALL NOT need to modify any call-site to compile

#### Scenario: A signature change to an app-called symbol is flagged app-breaking

- GIVEN a proposed change to a `cc_*` function that the app calls
- WHEN the change alters that function's signature or removes it
- THEN it SHALL be classified as an app-breaking ABI change requiring an explicit migration path, not a silent break

### Requirement: The kernel ships to iOS as a linkable xcframework

The kernel SHALL be buildable as an iOS xcframework (device + simulator, arm64) that exports
`cc_kernel.h`, so the app can adopt it as a binary product. Adoption SHALL consist of linking
the kernel behind the app's existing `cc_*` call-sites and selecting the engine via
`cc_set_engine`, without rewriting the app's geometry call-sites.

#### Scenario: The app adopts the kernel by linking the xcframework and flipping the engine

- GIVEN the kernel built as an iOS xcframework exporting `cc_kernel.h`
- WHEN the app links it and calls `cc_set_engine` to select the engine
- THEN the app's existing `cc_*` call-sites SHALL drive the kernel unchanged, per the adoption sequence in `docs/APP-ADOPTION-GUIDE.md`

### Requirement: Adoption SHALL require no kernel-side Swift binding

Kernel iOS-app compatibility SHALL be satisfied by the `cc_*` ABI-parity guarantee and the
xcframework alone, with no kernel-provided Swift binding required for adoption. The Swift and
Objective-C++ layer that wraps `cc_*` for the app (`KernelBridge.mm`, `KernelService.swift`,
`PreviewKernel.swift`, the `BRepShape` wrapper) is app-owned code, not a kernel deliverable; a
kernel-provided ergonomic Swift binding is explicitly OUT of scope here and tracked separately
(`add-swift-binding`).

#### Scenario: Adoption succeeds with no kernel-side Swift package

- GIVEN the kernel provides only the `cc_*` C ABI and the xcframework (no Swift package)
- WHEN the app adopts the kernel using its own Swift/Obj-C++ bridge
- THEN adoption SHALL be complete and correct without any kernel-shipped Swift API, and the absence of a kernel Swift binding SHALL NOT block the app
