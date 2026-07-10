# Proposal — ios-app-compat (iOS-app compatibility as a first-class, recorded strategy)

## Why

The CyberCad iPad app (`/Users/leonardoaraujo/work/cybercad`) is the kernel's primary
consumer, yet the *strategy* by which the kernel stays usable from that iOS/Swift app has only
ever been recorded implicitly — scattered across `docs/APP-MIGRATION-READINESS.md`,
`docs/APP-ADOPTION-GUIDE.md`, and the `moat-abi-app-parity` change. There is no single spec
that states, as intent, **how iOS-app compatibility is delivered and what guarantees the
kernel owes the app.** This change records that strategy as a first-class capability so it is
durable intent, not tribal knowledge — and so the *absence* of a kernel-side Swift binding is
a deliberate, documented decision rather than an oversight.

The strategy is already true and already working; this change makes it explicit and testable.

## The strategy (what is already the case)

- **The `cc_*` C ABI is the sole iOS integration seam.** The public boundary is plain C
  (integer handles + POD structs — `include/cybercadkernel/cc_kernel.h`). No C++ or engine
  type crosses it. This is byte-identical in shape to the app's own
  `CyberCad/Kernel/Bridge/KernelBridgeAPI.h`, by design.
- **The kernel's ABI is a superset of what the app links.** `moat-abi-app-parity` added the
  six functions the app's Swift `BRepShape`/`KernelService` actively call but the kernel
  header was missing (`cc_loft_circles`, `cc_loft_circle_wire`, `cc_loft_typed`,
  `cc_loft_along_rails`, `cc_shape_solid_count`, `cc_shape_solid_at`), so the app can link the
  kernel product with every call-site unchanged.
- **The app owns its Swift / Objective-C++ layer.** `KernelBridge.mm` (the Obj-C++ bridge
  over `cc_*`), `KernelService.swift`, `PreviewKernel.swift`, and the `BRepShape` wrapper are
  **app code**, not a kernel deliverable. The kernel does not ship a Swift binding today; it
  ships the ABI and the app maps its own Swift onto it.
- **The kernel ships to iOS as an xcframework** (`scripts/build-xcframework.sh` — device +
  simulator, arm64) exposing `cc_kernel.h`. Adoption is: link `libcybercadkernel` behind the
  app's existing `cc_*` call-sites, then flip the engine with `cc_set_engine` — the mechanics
  in `docs/APP-ADOPTION-GUIDE.md`.

## What Changes

- Adds a new capability spec `ios-app-compat` recording the four guarantees above as
  requirements with scenarios, so future ABI changes are checked against "does this keep the
  app linkable and its Swift call-sites unchanged?"
- **No code changes.** This is a documentation-of-intent change; it references existing docs
  and the already-shipped ABI/xcframework. `src/native`, the `cc_*` ABI, and the app are all
  untouched.

## Scope

- The `ios-app-compat` capability spec only. It formalizes the ABI-parity + app-owns-Swift
  strategy that `moat-abi-app-parity` / `APP-ADOPTION-GUIDE.md` already implement.

## Non-goals

- This does NOT add a kernel-side Swift binding — that is the deliberately-separate Phase 2
  (`add-swift-binding`), a future deliverable for when the app wants to drop its hand-written
  bridge. This change records precisely that the current strategy needs no such binding.
- No change to the ABI, the xcframework packaging, or any app code.
