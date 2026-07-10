# Proposal — add-swift-binding (a first-class iOS/macOS Swift binding over `cc_*`)

## Why

The kernel already ships a stable `cc_*` C ABI and an iOS xcframework, and the CyberCad app
consumes them through its **own** hand-written Objective-C++ bridge (`KernelBridge.mm`) plus
Swift wrappers (`BRepShape`, `KernelService`). That works (see `ios-app-compat`), but it means
every app owns and maintains bridging boilerplate — manual handle lifetime, manual POD-struct
marshalling, `0`/`nil`-return error checking — that is identical in shape to what the desktop
**Python binding** (`add-python-binding`) already automates on its side.

This change proposes the Swift analogue of that Python binding: a **kernel-shipped, ergonomic
Swift package** over `cc_*`, so that the app (and any future Swift consumer, incl. a macOS
tool or Swift test/showcase harness) can drop its hand-written bridge in favour of a
maintained, tested Swift API that tracks the ABI. It is a **future deliverable**, sequenced
after `ios-app-compat` records the current strategy — build it when the app is ready to
migrate off `KernelBridge.mm`.

## What Changes

- A **SwiftPM package** `CyberCadKernel` distributed against the existing iOS/macOS
  xcframework (`scripts/build-xcframework.sh`), exposing two layers:
  - a **low-level 1:1 layer** — every `cc_*` function + POD struct surfaced to Swift (via the
    C module map over `cc_kernel.h`), so nothing is hidden and the binding stays valid as the
    engine behind the facade evolves;
  - an **ergonomic Swift API** — a `Kernel`/`Shape` object model with automatic handle
    lifetime (release on `deinit`), Swift value types for POD structs, mesh/point data as
    `[SIMD3<Float>]` / `Data` / `MTLBuffer`-friendly arrays, and **throwing** functions that
    raise a Swift `Error` from `cc_last_error` instead of returning `0`/`nil`.
- An **XCTest suite** asserting REAL geometry through Swift (extrude/boolean volumes, mass
  properties, STEP round-trip, tessellation) — the Swift mirror of the Python `pytest` suite.
- **Documentation** (`docs/swift.md`) covering package integration and the app-migration path
  (replace `KernelBridge.mm` + the `BRepShape` bridging with the package).

## Relationship to the app

The app CAN adopt this package to replace its bespoke bridge, but is NOT required to — the
`ios-app-compat` ABI-parity strategy keeps the app working with its own bridge regardless.
This binding is an ergonomics + maintenance improvement, not an adoption prerequisite. The app
remains read-only from the kernel's side; migration is an app-side choice.

## Scope

- The `swift-binding` capability: the SwiftPM package (low-level + ergonomic layers), the
  XCTest suite, and `docs/swift.md`. Pure consumer of the `cc_*` ABI + the xcframework.

## Non-goals

- No change to the `cc_*` ABI, `src/native`, or the xcframework packaging beyond what the
  package consumes. No engine or geometry change.
- Does not modify the CyberCad app (adoption is an app-side task, tracked with the app).
- Not a replacement for `ios-app-compat` — that capability still governs the C-ABI seam and
  the parity guarantee; this binding sits on top of it.
