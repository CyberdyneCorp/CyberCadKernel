# native-exchange — descope native IGES import (STEP-only)

## MODIFIED Requirements

### Requirement: STEP import and IGES export/import stay OCCT (out of scope, honest)

Native IGES import and export SHALL be DESCOPED (STEP-only interchange): no native IGES
reader or writer SHALL ever be built. `cc_iges_export` and `cc_iges_import` SHALL remain
unconditional fall-throughs to the OCCT engine (`IGESControl_*`) under both engine
settings — the `cc_*` ABI SHALL be preserved (additive-only) — and at `#8 drop-occt` the
`cc_iges_*` entries SHALL be removed/stubbed (return `0`/`nil`), NOT reimplemented
natively. Native STEP import HAS landed as a first slice (the AP203 manifold-solid-brep
subset — see the native-STEP-import requirements), so STEP SHALL be the SOLE native
interchange format; IGES SHALL NOT be a `drop-occt` blocker, and the remaining
`drop-occt` exchange work SHALL be a general STEP/AP242 reader on top of the landed AP203
slice.

#### Scenario: STEP import is native-first-slice, else OCCT (parity)
- GIVEN a STEP file on a booted iOS simulator
- WHEN `cc_step_import(path)` is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN a writer-emitted AP203 manifold-solid-brep subset file SHALL be read by the native reader (self-verified watertight) matching the OCCT `STEPControl_Reader` result within tolerance, and any out-of-scope file SHALL decline to OCCT — never a fabricated shape

#### Scenario: IGES export and import are identical under both engines (parity), pending descope
- GIVEN a solid and an IGES file on a booted iOS simulator
- WHEN `cc_iges_export(body, path)` and `cc_iges_import(path)` are called with the native engine active and with the OCCT default
- THEN the results SHALL be identical under both engines (IGES stays OCCT `IGESControl_*`; the native engine intercepts neither), and at `drop-occt` the `cc_iges_*` entries SHALL be removed/stubbed rather than reimplemented natively

### Requirement: IGES import/export and the STEP writer stay unchanged (out of scope, honest)

IGES import and export SHALL be DESCOPED (STEP-only decision): no native `cc_iges_export`
/ `cc_iges_import` path SHALL be built; `NativeEngine::iges_export` /
`NativeEngine::iges_import` SHALL remain unconditional OCCT fall-throughs, removed/stubbed
at `drop-occt`. This change SHALL NOT modify the native STEP writer (`step_writer.cpp`) or
the tessellator — native STEP import inverts what the writer already produces. Native STEP
import of the writer-emitted AP203 manifold-solid-brep subset SHALL be recognised as the
landed first import slice; a general STEP/AP242 reader + a general-curved kernel are what
still block `#8 drop-occt` — IGES SHALL NOT be on that list.

#### Scenario: IGES import/export are identical under both engines (parity), pending descope
- GIVEN a solid and an IGES file on a booted iOS simulator
- WHEN `cc_iges_export(body, path)` and `cc_iges_import(path)` are called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the results SHALL be identical under both engines (IGES stays OCCT `IGESControl_*`; the native engine intercepts neither) until `drop-occt` removes/stubs them

#### Scenario: The STEP writer and tessellator are byte-for-byte unchanged
- GIVEN this change applied
- WHEN `step_export_native` serializes a native solid and the tessellator meshes it
- THEN their output SHALL be identical to before this change (this descope is documentation-only; it reads/changes no code and does not alter the writer or the tessellator)
