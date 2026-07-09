# Design — moat-m8dry-unlink-rehearsal (MOAT M8, scoped-unlink DRY-RUN)

## Goal

Empirically validate the static `DROP-OCCT-READINESS.md` classification by running the
kernel in the post-unlink shape — `NativeEngine` as the default active engine with the
**stub** as its only fallback — WITHOUT deleting `src/engine/occt` or changing any
shipping default. Report, per `cc_*` op, whether it serves natively, cleanly declines, or
misbehaves (crash / silent-wrong).

## The three engine-selection states

The build already had two engine seams:

- `create_default_engine()` — the DEFAULT active engine (what `cc_*` hits with no
  `cc_set_engine(1)`). In the shipping host build it returns the **stub**; in an OCCT
  build it returns `OcctEngine`.
- `make_native_fallback_engine()` — the engine a `NativeEngine` forwards to for ops it
  does not serve. Returns `OcctEngine` under `CYBERCAD_HAS_OCCT`, else the **stub**.

| configuration | default engine | native fallback | reaches OCCT? |
|---|---|---|---|
| shipping host (stub) | stub | stub | no (OCCT not linked) |
| shipping iOS/macOS (OCCT) | OcctEngine | OcctEngine | yes |
| **M8 rehearsal (this change)** | **NativeEngine(stub)** | **stub** | **no — never linked** |

The critical distinction: in the *plain* host build the default engine is the STUB, so a
test that never calls `cc_set_engine(1)` measures the stub, not the native path. The
rehearsal flips the DEFAULT to `NativeEngine`, so the WHOLE suite — and any future app
smoke — exercises the native-only path exactly as the product would after unlink. This is
why the rehearsal is a stronger measurement than the plain host build, even though both
have OCCT unlinked.

## Why a default-engine flip (not deleting occt) is the right rehearsal

`DROP-OCCT-READINESS.md §4` step 4 is "delete `src/engine/occt` so
`make_native_fallback_engine()` / `create_default_engine()` resolve to the stub." The
host build ALREADY excludes the OCCT TUs (`CYBERCAD_HAS_OCCT=OFF` by default), so the
fallback is already the stub — the *only* thing that differs between the plain host build
and the true post-unlink product is which engine is DEFAULT. The rehearsal changes
exactly that one bit, behind an opt-in flag, and leaves `src/engine/occt` present so the
oracle stays available and `main`'s shipping default is provably untouched.

## Implementation

- `CMakeLists.txt`: `option(CYBERCAD_M8_REHEARSAL … OFF)`, a FATAL_ERROR if combined with
  `CYBERCAD_HAS_OCCT` (the rehearsal is the no-OCCT world), and
  `target_compile_definitions(… CYBERCAD_M8_REHEARSAL=1)` when ON.
- `native_engine.cpp`: `#if defined(CYBERCAD_M8_REHEARSAL) && !defined(CYBERCAD_HAS_OCCT)`
  → `create_default_engine()` returns `std::make_shared<NativeEngine>(make_native_fallback_engine())`.
- `stub_engine.cpp`: its `create_default_engine()` is now
  `#if !defined(CYBERCAD_HAS_OCCT) && !defined(CYBERCAD_M8_REHEARSAL)` so exactly one TU
  defines the symbol in each configuration.

## Measurement method

1. **Build:** configure `-DCYBERCAD_M8_REHEARSAL=ON` into a separate `build-m8/`; confirm
   it links with zero OCCT TUs and no `OcctEngine` symbol.
2. **Full HOST suite:** run every CTest under the rehearsal config, per test classified
   PASS-native / clean-decline / FAIL(crash/silent-wrong).
3. **Per-op decline probe:** a throwaway driver (`scratch/m8_probe.cpp`) drives the
   Class-B/C ops on a native body (`cc_fillet_face`, `cc_full_round_fillet[_faces]`,
   `cc_fillet_edges_g2`, `cc_twisted_sweep` with real twist, `cc_loft_along_rail` with a
   curved rail, `cc_thread_apply`, `cc_iges_import/_export`) and spot-checks the Class-A
   spine (`cc_solid_revolve/_loft/_tessellate/_mass_properties`). Each op is reported
   SERVED-NATIVE, CLEAN-DECLINE (id=0 + honest error), or the FINDING cases
   (DECLINE-NO-ERR = id 0 with empty error; SERVED where a wrong/fabricated shape is
   emitted).

## Decline contract being validated

When a `NativeEngine` op returns NULL (out-of-envelope) it forwards to `fallback()`. Under
the rehearsal the fallback is the stub, whose default methods return
`engine_unsupported(op)` — a clean `Error` the facade collapses to a 0/nil handle with a
non-empty `cc_last_error`. Body-consuming ops that hard-decline native bodies
(`CC_NATIVE_BODY_UNSUPPORTED`) return their own honest error directly (they never hand a
native void to the fallback). Either way the observable contract is: **id=0 + non-empty
error, never a crash, never a fabricated shape.**

## Non-goals / discipline

- NOT deleting `src/engine/occt`; NOT changing any shipping default; NOT touching `main`.
- NOT weakening any tolerance or faking a pass — a clean decline is a first-class,
  desired outcome for out-of-envelope input, and is reported as such.
- The rehearsal flag and the probe driver are throwaway measurement artifacts, never
  shipped.
