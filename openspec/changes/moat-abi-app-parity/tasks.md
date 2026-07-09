# Tasks — moat-abi-app-parity

## 1. ABI surface (additive)
- [x] 1.1 Add the six declarations to `include/cybercadkernel/cc_kernel.h`, signature-matched
      to the app's `KernelBridgeAPI.h` (`cc_loft_circles`, `cc_loft_circle_wire`,
      `cc_loft_typed`, `cc_loft_along_rails`, `cc_shape_solid_count`, `cc_shape_solid_at`).
- [x] 1.2 Confirm no existing declaration changed (`git diff` on the header = pure additions).

## 2. Engine interface
- [x] 2.1 Add six virtuals to `src/engine/IEngine.h` with `engine_unsupported` defaults.

## 3. OCCT engine (oracle + transition fallback)
- [x] 3.1 `loft_circles`, `loft_circle_wire`, `loft_typed`, `loft_along_rails` — match the
      app's `KernelBridge.mm` construction exactly (true `gp_Circ` wires, typed-wire + frame
      placement, two-rail `MakePipeShell` with guide retry).
- [x] 3.2 `shape_solid_count` / `shape_solid_at` — `TopExp_Explorer(TopAbs_SOLID)`,
      index 0-based.

## 4. Native engine
- [x] 4.1 `shape_solid_count` / `shape_solid_at` — NATIVE for a native body via the native
      `Explorer` over `ShapeType::Solid`; OCCT body → fallback.
- [x] 4.2 The four loft variants — honest decline (fallthrough to OCCT) during transition.

## 5. Facade
- [x] 5.1 Six guarded entry points in `src/facade/cc_kernel.cpp` delegating to the active
      engine; `cc_loft_typed` reuses `to_profile_segs`.

## 6. Gate A — host analytic (OCCT-free)
- [x] 6.1 `cc_loft_circles` of two coaxial circles (r, R at z=0/h) → frustum; volume matches
      the closed-form frustum formula `π h (r² + rR + R²)/3` (cylinder when r==R).
- [x] 6.2 `cc_shape_solid_count` / `cc_shape_solid_at` on a known 2-lump compound → exactly 2,
      each extracted solid re-enumerates to 1, and their volumes sum to the whole.
- [x] 6.3 Regression tests for every new function (including honest-decline/degenerate paths).

## 7. Gate B — sim native-vs-OCCT parity (booted simulator)
- [x] 7.1 Parity harness (own `main()` + runner) comparing native-vs-OCCT for the loft
      wrappers (volume/area) and native-vs-OCCT solid enumeration (count + per-solid volume).
- [x] 7.2 `run-sim-suite.sh` SKIP entry for the new harness.

## 8. Link-completeness
- [x] 8.1 Check EVERY `cc_*` declared in the app's `KernelBridgeAPI.h` now exists in
      `cc_kernel.h` (grep/link-check).

## 9. Docs + validate
- [x] 9.1 `docs/APP-MIGRATION-READINESS.md` + `openspec/MOAT-ROADMAP.md`: mark the 6-fn
      ABI-parity link blocker RESOLVED.
- [x] 9.2 `openspec validate moat-abi-app-parity --strict`.
- [x] 9.3 Structural discipline: `git diff src/native` OCCT-free; only-additive header diff;
      tessellator/blend/boolean untouched.
