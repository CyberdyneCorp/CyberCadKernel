# engine-adapter

## ADDED Requirements

### Requirement: Non-shipping M8 unlink-rehearsal engine configuration

The build system SHALL provide a NON-SHIPPING, default-OFF option
`CYBERCAD_M8_REHEARSAL` that, when enabled, makes the build's DEFAULT active engine a
`NativeEngine` whose only fallback is the stub — the exact engine wiring the product
would have after `src/engine/occt` is deleted — so that every `cc_*` operation, including
in a test that never calls `cc_set_engine(1)`, exercises the native-only + stub-fallback
path. This option SHALL be mutually exclusive with `CYBERCAD_HAS_OCCT` (the rehearsal is
the no-OCCT configuration) and SHALL NOT change the engine selection of any shipping
configuration when it is OFF. It exists ONLY to MEASURE the scoped-`drop-occt` end state
and SHALL NOT be enabled in any shipped artifact.

Under this configuration, each `cc_*` operation on a native body SHALL exhibit exactly
one of two observable outcomes: it SHALL either SERVE the result natively (a valid
non-zero handle / valid result), or CLEANLY DECLINE — return a 0/nil handle (or
`valid = 0`) with a non-empty `cc_last_error`. It SHALL NEVER crash and SHALL NEVER emit a
fabricated or silently-wrong shape. A native builder that returns NULL SHALL forward to
the stub fallback, whose default method returns the honest "unsupported" error the facade
collapses to a clean decline; a body-consuming op with no native path SHALL return its
own honest error directly and SHALL NOT hand a native void to the fallback.

#### Scenario: The rehearsal build links with no OCCT translation units

- GIVEN the kernel configured with `-DCYBERCAD_M8_REHEARSAL=ON` and `CYBERCAD_HAS_OCCT` OFF
- WHEN the library and test suite are built
- THEN the build SHALL link cleanly with NO `src/engine/occt` translation unit compiled
  AND with no `OcctEngine` symbol referenced in the archive

#### Scenario: The default engine is native with the stub as its only fallback

- GIVEN a rehearsal-configured build with no prior `cc_set_engine` call
- WHEN `cc_active_engine()` and `cc_brep_available()` are queried and a `cc_solid_extrude`
  is invoked on an in-domain profile
- THEN `cc_active_engine()` SHALL report native, `cc_brep_available()` SHALL report
  available, AND the extrude SHALL be SERVED natively (a non-zero handle) without reaching
  any OCCT code

#### Scenario: Every out-of-envelope operation cleanly declines

- GIVEN a rehearsal-configured build and a native body handed to a Class-B op with no
  native path (e.g. `cc_fillet_face`, `cc_thread_apply`) or a dropped Class-C op
  (`cc_iges_import` / `cc_iges_export`), or a native op given genuinely out-of-envelope
  input (e.g. `cc_twisted_sweep` with real twist, `cc_loft_along_rail` with a curved rail)
- WHEN the operation is invoked
- THEN it SHALL return a 0/nil handle with a non-empty `cc_last_error` AND SHALL NOT crash
  AND SHALL NOT emit a shape

### Requirement: Shipping default engine unchanged when the rehearsal option is off

The build SHALL keep every shipping configuration's engine selection byte-identical to
before this change when `CYBERCAD_M8_REHEARSAL` is OFF (host stub, iOS OCCT, macOS OCCT
dylib). The rehearsal option SHALL NOT alter the `cc_*` ABI, and `src/native/**` SHALL
remain OCCT-free and additive.

#### Scenario: Host stub default is unchanged with the flag off

- GIVEN the kernel built with `CYBERCAD_M8_REHEARSAL` OFF (the default) and no OCCT
- WHEN the test suite runs without any `cc_set_engine(1)` call
- THEN the default active engine SHALL be the stub (not native), `cc_brep_available()`
  SHALL report unavailable, and the stub-default sentinel tests SHALL pass exactly as
  before
