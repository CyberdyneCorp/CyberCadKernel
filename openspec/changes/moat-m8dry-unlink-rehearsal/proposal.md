# Proposal — moat-m8dry-unlink-rehearsal (MOAT M8, scoped-unlink DRY-RUN)

## Why

`DROP-OCCT-READINESS.md` is a **static, source-read** classification of every OCCT
fall-through (A now-native / B must-go-native / C decline). It asserts — from reading
`native_engine.cpp` — that at unlink the Class-A ops keep serving natively and the
Class-B/C ops become "clean declines". Nothing had ever **run** the kernel in the
post-unlink shape to confirm that. A static audit cannot tell a genuine clean decline
apart from a crash or a silent-wrong result on out-of-envelope input; only executing the
native-only + stub-fallback configuration can.

This change converts the static audit into a **measured** one: a throwaway, non-shipping
build configuration that makes the build's DEFAULT active engine the `NativeEngine` with
the **stub** as its only fallback — the exact wiring the product would have after
`src/engine/occt` is deleted — and runs the full HOST suite plus a per-op decline probe
against it. It is a rehearsal, NOT the unlink: `src/engine/occt` is not deleted and no
shipping default changes.

## What changes

- **New non-shipping CMake option `CYBERCAD_M8_REHEARSAL` (default OFF).** When ON (and
  it is mutually exclusive with `CYBERCAD_HAS_OCCT`), `create_default_engine()` returns
  `NativeEngine(make_native_fallback_engine())` = NativeEngine-over-stub instead of the
  bare stub. Every `cc_*` op — even in a test that never calls `cc_set_engine(1)` — then
  exercises the native-only + stub-fallback path. The override lives in
  `native_engine.cpp` under `#if CYBERCAD_M8_REHEARSAL && !CYBERCAD_HAS_OCCT`; the stub's
  own `create_default_engine()` is guarded out under the same macro to avoid a duplicate.
- **No production default changes.** With the flag OFF (every shipping configuration —
  host stub, iOS OCCT, macOS OCCT dylib) the engine selection is byte-identical to before.
  `src/native/**` stays OCCT-free/additive; the `cc_*` ABI is unchanged.
- **Measured `§6 measured rehearsal` section added to `DROP-OCCT-READINESS.md`** (dated
  2026-07-08) recording the empirical HOST-suite result under the rehearsal config, the
  per-op decline probe, and every static-vs-measured discrepancy.
- **`MOAT-ROADMAP.md` M8 section** updated with the rehearsal findings + the measured
  unlink checklist.

## Impact

- **Measurement only** — no geometry code, no ABI change, no shipping-default change. The
  rehearsal reused the existing native path unmodified; it changed only which engine is
  the *default* under an opt-in flag.
- **Key finding:** under the native-only + stub-fallback default, the full HOST suite
  produces **0 crashes and 0 silent-wrong results**. Every Class-B/C op on a native body
  **cleanly declines** (id=0 + honest `cc_last_error`); every Class-A spine op **serves
  natively**. The only "failing" tests are three that assert the *shipping-default*
  invariant ("default engine = stub / no B-rep / not native"), which the rehearsal
  deliberately inverts — they are sentinel flips, not regressions.
- **Consequence for the unlink:** deleting `src/engine/occt` from the product build today
  would NOT break the build or produce wrong output; it would only convert the 7 Class-B
  ops + 2 Class-C ops from OCCT-served to honest declines. Whether that decline is
  acceptable is a product-scope (M2/M3 breadth) question, not a build blocker.
