# Proposal — add-native-data-exchange

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability **#7 `native-exchange`** (STEP / IGES) is
sequenced late and honestly flagged as "may stay a thin external dependency longest".
This change lands the ONE piece of exchange that is tractable, exactly verifiable, and
adds real native coverage without faking the hard part: **native STEP export of a
native-built solid**.

Writing STEP is a **serialization** of a B-rep the native kernel already owns — it walks
the native topology (#2) and reads the attached geometry (#1) and emits the corresponding
ISO-10303-21 records. It needs no new geometry algorithm and is deterministic and
closed-form. Crucially it is **exactly verifiable**: the file it writes can be re-read by
OCCT `STEPControl_Reader` and compared against the source solid, so the honesty bar is
mechanical.

**Parsing arbitrary STEP (import) is the huge, long-lived part** — a full ISO-10303-21
tokenizer plus an entity-graph resolver for the whole Part-42 + AP203/AP214 schema
(hundreds of entity types, external refs, healing of ill-formed foreign files) and B-rep
reconstruction. **IGES** is a separate older format with its own writer + parser. Doing
either "natively" now would be a large effort with a high faking risk. So this proposal
scopes STEP export only; STEP import and IGES export/import stay OCCT, labelled honestly.

The result is real: a native-representable solid gets a native, standards-conformant STEP
AP203 file in true millimetres, proven correct by an OCCT round-trip. A solid outside the
representable subset, or a foreign / OCCT-built solid, falls through to OCCT
`STEPControl_Writer` — labelled, verified, never faked. This is the achievable native
ceiling for exchange; it does NOT unblock #8 `drop-occt` (that still needs native import +
IGES + a general-curved kernel).

## What changes

1. **A native STEP writer subtree** (`src/native/exchange/`, OCCT-free, host-buildable).
   New headers (e.g. `step_text.h` for ISO-10303-21 record formatting, `step_entities.h`
   for the Part-42 entity emitters, `step_writer.h` for the B-rep walk + umbrella
   `native_exchange.h`) that take a `topology::Shape` solid and return a STEP AP203 text
   buffer (an empty buffer ⇒ DECLINE / fall through). They include only
   `src/native/math` and `src/native/topology`, never OCCT. The pipeline:
   - **Representability gate.** Accept ONLY a native `Solid` with a `CLOSED_SHELL` (a
     manifold shell of faces) whose faces are `Plane` / `Cylinder` / `Cone` / `Sphere` /
     `BSpline` `FaceSurface`s and whose edges are `Line` / `Circle` / `BSpline`
     `EdgeCurve`s. Anything else (an `Ellipse` edge, an un-down-converted `Bezier`, a
     non-manifold / open shell, a degenerate face) ⇒ empty buffer (OCCT fall-through).
   - **Entity-graph build with deduplication.** Walk the topology once, assigning a
     stable `#n` id to each emitted entity, deduplicating shared leaves (a shared
     `CARTESIAN_POINT`, `DIRECTION`, `VERTEX_POINT`, and a shared `EDGE_CURVE` used by two
     `ORIENTED_EDGE`s of adjacent faces) so the graph mirrors the shared-node native
     B-rep. Emit bottom-up (points/directions → placements → curves/surfaces →
     vertices/edges → loops/bounds → faces → shell → solid) so every `#n` reference is
     already defined (no dangling forward reference).
   - **Part-42 entity emitters.** `CARTESIAN_POINT`, `DIRECTION`, `AXIS2_PLACEMENT_3D`
     (from a native `Ax3` frame), `VERTEX_POINT`; curves `LINE` (point + `VECTOR` /
     `DIRECTION`), `CIRCLE` (placement + radius), `B_SPLINE_CURVE_WITH_KNOTS` (degree,
     control points, knots + multiplicities, rational weights when present); `EDGE_CURVE`
     (start/end `VERTEX_POINT` + curve + same-sense flag), `ORIENTED_EDGE`, `EDGE_LOOP`,
     `FACE_OUTER_BOUND` / `FACE_BOUND`; surfaces `PLANE`, `CYLINDRICAL_SURFACE`,
     `CONICAL_SURFACE`, `SPHERICAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS`;
     `ADVANCED_FACE` (bounds + surface + sense); `CLOSED_SHELL`; `MANIFOLD_SOLID_BREP`.
     All coordinates in **true millimetres**.
   - **Header + context wrapper.** The ISO-10303-21 `HEADER` (`FILE_DESCRIPTION`,
     `FILE_NAME`, `FILE_SCHEMA` naming the AP203 config-controlled-3d schema); the AP203
     product / context wrapper (`APPLICATION_CONTEXT`, `PRODUCT`, `PRODUCT_DEFINITION`,
     the mm `SI_UNIT` length + solid-angle + plane-angle units and
     `GEOMETRIC_REPRESENTATION_CONTEXT` with `UNCERTAINTY_MEASURE_WITH_UNIT`) and the
     `ADVANCED_BREP_SHAPE_REPRESENTATION` binding the `MANIFOLD_SOLID_BREP` to the
     context.
2. **`NativeEngine::step_export` native-else-fallback glue**
   (`src/engine/native/native_engine.cpp`). `step_export` — today an unconditional
   fall-through — becomes: if `body` is a native body, invoke the native writer; if it
   returns a non-empty buffer, write it to `path`, then run the **mandatory OCCT-read
   round-trip self-check** (re-read the file with OCCT `STEPControl_Reader`; compare the
   read shape's volume / bbox / sub-shape counts vs the source native solid within
   tolerance; require the read shape valid). On success return `1`. If the writer DECLINES
   (empty buffer), or the self-check FAILS, or `body` is foreign, DISCARD any native file
   and fall through to OCCT `STEPControl_Writer` (labelled). The OCCT round-trip check and
   fall-through live behind `CYBERCAD_HAS_OCCT`; the native writer itself never sees OCCT.
3. **`step_import`, `iges_export`, `iges_import` unchanged** — they stay `fallback().*`
   (OCCT). No native code path is added for them.

## Non-goals (DEFERRED — stay OCCT, not implemented, not faked)

- **STEP import** (`cc_step_import`) — parsing an arbitrary ISO-10303-21 file (full
  tokenizer + Part-42 + AP203/AP214 entity resolver + foreign-file healing + B-rep
  reconstruction). Stays OCCT `STEPControl_Reader`. This is the large, long-lived part.
- **IGES export / import** (`cc_iges_export` / `cc_iges_import`) — a separate older format
  with its own writer and parser. Stay OCCT `IGESControl_*`.
- **Unrepresentable native solids** — an `Ellipse` edge, an un-down-converted `Bezier`
  surface, a non-manifold / open shell, a degenerate face. The native writer DECLINES;
  OCCT `STEPControl_Writer` fall-through.
- **Foreign / OCCT-built solids** — no native B-rep to walk; OCCT `STEPControl_Writer`
  fall-through.
- **STEP AP242 / PMI / assemblies / colours / names / GD&T** — only a single-solid AP203
  `ADVANCED_BREP_SHAPE_REPRESENTATION` is emitted. Richer AP242 features are out of scope.
- **Unblocking #8 `drop-occt`** — this change does NOT remove the OCCT dependency; STEP
  import + IGES + a general-curved kernel still block it. Reported honestly.

## Impact

- New OCCT-free, host-buildable headers under `src/native/exchange/` (ISO-10303-21 text
  formatting, Part-42 entity emitters, the B-rep walk, the representability gate) under a
  `native_exchange.h` umbrella. New host CTest cases in a new
  `tests/test_native_exchange.cpp` (structural assertions on the emitted buffer, no OCCT)
  + facade cases in `tests/test_native_engine.cpp` (native export success + fall-through).
- `src/engine/native/native_engine.cpp` — `step_export` gains the native-else-fallback
  path + the OCCT-read round-trip self-check. `step_import` / `iges_export` /
  `iges_import` unchanged. `native_engine.h` unchanged (`step_export` signature already
  present).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no**
  `src/facade/cc_kernel.cpp` change (the `cc_step_export` entry already routes through the
  active engine). The `cc_step_export` doc-comment (returns `1` on success) is the
  contract this change implements natively for native-representable solids.
- A new sim parity/round-trip test (`tests/sim/native_exchange_parity.mm` +
  `scripts/run-sim-native-exchange.sh`) with its own `main()`, on the `run-sim-suite.sh`
  SKIP list so the 221-assertion suite count is unchanged.
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the native STEP writer. All existing suites stay green at the
  OCCT default.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** structural unit tests on the
emitted ISO-10303-21 buffer for a native-built solid — correct file framing, an AP203
`FILE_SCHEMA`, a millimetre unit context, exactly one `MANIFOLD_SOLID_BREP` → one
`CLOSED_SHELL`, an `ADVANCED_FACE` per native face with the right surface entity, correct
curve entities, every `#n` reference resolving, mm coordinates — plus DECLINE cases
(`Ellipse` edge / non-manifold / unrepresentable → empty buffer), all with no OCCT;
(b) **sim native-write / OCCT-read round-trip** through the facade (`cc_set_engine(1)`):
export a native solid, re-read the file with OCCT `STEPControl_Reader`, and assert the
round-tripped shape matches the source native solid (volume / bbox / topology within
tolerance, valid `BRepCheck`), with the OCCT `STEPControl_Writer` export
(`cc_set_engine(0)`) as oracle; and assert the fall-through cases (foreign / unrepresentable)
and `cc_step_import` / `cc_iges_*` identical under both engines. Done only when both gates
pass and every existing suite stays green at the OCCT default. Reported honestly as the
export-only native ceiling; STEP import + IGES remain OCCT.
