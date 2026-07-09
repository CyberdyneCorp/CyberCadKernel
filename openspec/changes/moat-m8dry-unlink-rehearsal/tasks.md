# Tasks — moat-m8dry-unlink-rehearsal (MOAT M8, scoped-unlink DRY-RUN)

Order: rehearsal build flag → build native-only + stub-fallback config → run HOST
suite + per-op decline probe → reconcile static-vs-measured → docs. This is a
MEASUREMENT track: no geometry code, no `cc_*` change, no shipping-default change.
The rehearsal flag is throwaway/non-shipping and default OFF.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh host` (exit 0). (iossim substrate not
      needed — the rehearsal is a HOST measurement; the flag is host-buildable.)

## 1. Rehearsal build configuration (non-shipping)

- [x] 1.1 `CMakeLists.txt` — `option(CYBERCAD_M8_REHEARSAL … OFF)` + a FATAL_ERROR
      mutual-exclusion guard with `CYBERCAD_HAS_OCCT`; define `CYBERCAD_M8_REHEARSAL=1`
      on the target when ON. Clearly documented as NON-SHIPPING.
- [x] 1.2 `native_engine.cpp` — under `#if CYBERCAD_M8_REHEARSAL && !CYBERCAD_HAS_OCCT`,
      `create_default_engine()` returns `NativeEngine(make_native_fallback_engine())`
      (NativeEngine over stub). `make_native_fallback_engine()` unchanged.
- [x] 1.3 `stub_engine.cpp` — guard the stub's `create_default_engine()` out under the
      same macro to avoid a duplicate definition.
- [x] 1.4 Baseline build (flag OFF): configure + `cmake --build` + full `ctest` green
      (56/56 pass; the 5 heavy SSI numerics tests pass with an adequate per-test timeout).
      Confirms the shipping default is unchanged on this branch.

## 2. Build the native-only + stub-fallback config

- [x] 2.1 `cmake -S . -B build-m8 -DCYBERCAD_M8_REHEARSAL=ON -DCYBERCAD_HAS_NUMSCI=ON …`
      configures + builds + links clean.
- [x] 2.2 Confirm NO `src/engine/occt` TUs are compiled (`find build-m8 -path *occt* -name *.o`
      empty) and NO `OcctEngine` symbol is referenced in the archive (`nm | grep OcctEngine`
      empty).

## 3. Measure — full HOST suite + per-op decline probe

- [x] 3.1 Run the full HOST suite under the rehearsal config; record per test PASS-native
      / clean-decline / FAIL(crash/silent-wrong). Result: 53 pass, 3 sentinel-flip
      "fails" (`test_guard`, `test_abi`, `test_native_engine` — each asserts the
      shipping-default "stub / no-brep / not-native" invariant the rehearsal inverts),
      0 crashes, 0 silent-wrong.
- [x] 3.2 Per-op decline probe (`scratch/m8_probe.cpp`, throwaway): drive the Class-B/C
      ops on a native body + Class-A spine spot-check. Result: every B/C op CLEAN-DECLINE
      (id=0 + honest error), every A spine op SERVED-NATIVE. Recorded in
      `DROP-OCCT-READINESS.md §6`.

## 4. Reconcile static vs measured

- [x] 4.1 Compare the measured A/B/C behavior against the static classification. Record
      every discrepancy in `DROP-OCCT-READINESS.md §6`. (Finding: no A→crash and no
      A→silent-wrong discrepancies; the static classification held. The only surprises
      are the three shipping-default sentinel tests that the rehearsal-default inverts.)
- [x] 4.2 Produce the measured unlink checklist: the concrete ops that would change
      behavior (B/C → decline) if `src/engine/occt` were deleted today, with each exact
      failure mode. Recorded in `DROP-OCCT-READINESS.md §6` + `MOAT-ROADMAP.md` M8.

## 5. Docs + discipline

- [x] 5.1 `DROP-OCCT-READINESS.md` — add dated `§6 measured rehearsal` section.
- [x] 5.2 `MOAT-ROADMAP.md` — M8 section updated with the rehearsal findings.
- [x] 5.3 Structural check: production default engine on this branch UNCHANGED (rehearsal
      is opt-in via the flag only); `src/native` OCCT-free/additive; `cc_*` unchanged.
