# Proposal — moat-abi-app-parity (ABI parity: the 6 functions the app links but the kernel is missing)

## Why

The #1 adoption blocker in `docs/APP-MIGRATION-READINESS.md` is that the CyberCad app cannot
even LINK the CyberCadKernel product: the app's `KernelBridgeAPI.h` declares — and its Swift
`BRepShape` wrapper actively CALLS — six `cc_*` symbols that are **absent** from the kernel's
shipped ABI (`include/cybercadkernel/cc_kernel.h`). A naïve link of the app against
`libcybercadkernel.a` fails to resolve these symbols, so the "ADOPT" step (replace the app's
inline OCCT bridge with the kernel product, keeping every call-site unchanged) is impossible
until they exist.

The six missing functions are all thin wrappers over machinery the kernel ALREADY ships:

- `cc_loft_circles`, `cc_loft_circle_wire`, `cc_loft_typed`, `cc_loft_along_rails` — loft
  variants that reuse the existing `BRepOffsetAPI_ThruSections` / `MakePipeShell` loft path
  (the same one behind `cc_solid_loft_wires` / `cc_loft_along_rail`), differing only in the
  section wires (true circles, typed arc/spline profiles) and the two-rail guide spine.
- `cc_shape_solid_count`, `cc_shape_solid_at` — enumerate the connected solids of a
  (possibly multi-lump / compound) body, so the app can split a disjoint boolean result into
  independently selectable bodies. Thin wrappers over topology enumeration (the same
  sub-shape machinery behind `cc_subshape_ids`).

The exact signatures are dictated by the app's header (this is the one case where an external
header defines the ABI shape); they are matched precisely so the app links + behaves identically.

## What changes

- **`include/cybercadkernel/cc_kernel.h` (ADDITIVE ONLY):** add the six declarations, byte-for-byte
  signature-matched to the app's `KernelBridgeAPI.h` (param order/types/return convention). No
  existing declaration is touched.
- **`src/facade/cc_kernel.cpp` (additive):** six new entry points, each a guarded delegation to
  the active engine, exactly like every other `cc_*` op. `cc_loft_typed` reuses the existing
  `to_profile_segs` translation.
- **`src/engine/IEngine.h` (additive):** six new virtuals with `engine_unsupported` defaults, so
  the stub / any partial engine inherits an honest decline.
- **`src/engine/occt/*` (the ORACLE + transition fallback):** implement all six to MATCH the
  app's reference `KernelBridge.mm` construction exactly — true-circle `gp_Circ` section wires,
  the typed-wire builder placed on a plane frame, the two-rail `MakePipeShell` with a guide
  auxiliary spine (with the guide-dropped retry), and `TopExp_Explorer(TopAbs_SOLID)` for
  count/at.
- **`src/engine/native/*` (native serves what it can, else honest decline → OCCT):**
  `shape_solid_count` / `shape_solid_at` are NATIVE for a native body (the native `Explorer`
  over `ShapeType::Solid`, matching OCCT's `TopExp_Explorer` count-with-repetition + isSame
  dedup). The four loft variants HONESTLY DECLINE on the native engine during the transition
  (they require true analytic-circle / spline section edges the native ruled-loft substrate
  cannot represent exactly yet) and fall through to the OCCT oracle — never a wrong shape.

## Honest scope / declines

- The four loft wrappers decline natively → OCCT. This is the correct discipline: a native
  ruled loft would face the true circle rim, changing topology (a 64-gon rim instead of one
  circular edge) — a WRONG result vs the app's contract, so we decline instead of faking it.
- `shape_solid_count` / `shape_solid_at` return `0` on an unknown body / out-of-range index
  (index is 0-based, matching the app's `connectedSolids()` loop `0..<count`).
- On the OCCT-free host, the loft wrappers return `0` with an honest error (no OCCT to fall to);
  the solid-enumeration wrappers serve natively on the host.

## Impact

- The app's naïve link resolves ALL of its declared `cc_*` symbols (a link-completeness check
  greps every declaration in the app header against `cc_kernel.h`).
- `src/native/**` stays OCCT-free (comments only).
- The ONLY `include/` change is the six additive declarations; every existing `cc_*` is unchanged.
- The tessellator (`src/native/tessellate/`), `src/native/blend`, and `src/native/boolean` are
  UNTOUCHED (other tracks own them).
- Readiness: the "ADOPT — app cannot link the kernel product (6 missing symbols)" blocker moves
  to RESOLVED.
