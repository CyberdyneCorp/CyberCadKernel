# Proposal — moat-m4pmi-ap242-pmi (MOAT M4-tail-3, AP242 PMI recognise+classify+count)

## Why

Today the native STEP reader (`src/native/exchange/step_reader.cpp`) is
schema-independent and **SKIPS** AP242 PMI / GD&T / draughting / annotation
entities entirely. Its disposition scan (`assemblyDisposition`) already keys on
whether a relationship graph reaches a `MANIFOLD_SOLID_BREP`: a
representation-relationship / annotation graph that reaches **no** brep returns
`AsmKind::None` and the geometry falls through to the flat solid path — so an AP242
solid + PMI file imports the **SOLID identically to OCCT**, with the PMI silently
dropped. That is honest, but it discards information the file carries: which
dimensions, geometric tolerances, datums, and notes are present, and what geometry
they attach to.

The blocker to doing more is real and must be stated plainly: **there is no native
PMI data model**. `topo::Shape` carries geometry only; `cc_step_import` returns a
single solid handle. A full GD&T semantic model (tolerance values, zones, modifiers,
datum reference frames, feature-control-frame parsing) is a multi-person-year
capability and is explicitly **out of scope** here. What IS reachable — and what
this change delivers — is a bounded FIRST slice: **recognise + classify + count** the
PMI annotation entities and expose their presence, type, and attachment target as
**additive, read-only native metadata**, without a semantic model and **without
changing the geometry import by a single byte**.

This is genuinely verifiable on both required gates. HOST ANALYTIC: a hand-authored
AP242 fixture with known PMI content (N dimensions, M geometric tolerances, K datums,
J notes, each attached to a known face) — the native scan's per-class counts, types,
and attachment targets must match the fixture's known content exactly, with no OCCT.
SIM: the XDE toolkits (`TKXCAF`/`TKDESTEP`) are already linked in
`scripts/run-sim-native-step-import.sh`, so `STEPCAFControl_Reader` +
`XCAFDoc_DimTolTool` provide an OCCT oracle for the semantic-PMI subset (dimensions,
tolerances, datums) — the native scan must match OCCT XDE on the annotation set the
slice covers; graphical-only notes it does not expose are checked by the host gate.

## What Changes

1. **An additive, read-only PMI scan pass** in `src/native/exchange/step_reader.cpp`
   — a NEW `scanPmi(records) → PmiSummary` walk over the already-parsed Part-21
   record table. It classifies each recognised AP242 PMI keyword into a native
   `PmiClass` (Dimension, GeometricTolerance, Datum, DatumTarget, Note,
   AnnotationGeometry), counts per class, and records each annotation's referenced
   attachment target `#id` (the `SHAPE_ASPECT` / geometric-item / dimensional-
   characteristic it points at, `0` if none). It reads the record table **only** —
   it does NOT touch `Mapper::build()` or the geometry path.

2. **A native PMI metadata type + free function** in the native-exchange public
   surface (`native_exchange.h` / `step_reader.h`): `PmiSummary step_scan_pmi(const
   std::string& path)` and the POD types (`PmiClass`, `PmiAnnotation`, `PmiSummary`).
   OCCT-free, header-declared, body in `step_reader.cpp`.

3. **An additive `cc_*` accessor** — a new `cc_step_pmi_scan(const char* path,
   CCPmiSummary* out)` (integer per-class counts, POD out-struct) and its
   `NativeEngine`/`IEngine` plumbing. The existing `cc_step_import` /
   `step_import_native` signatures and behaviour are **UNCHANGED** — this is purely
   additive; no existing accessor is modified.

4. **Verification.** A host-analytic gate (`clang++ -std=c++20`, no OCCT) asserting
   the scan matches a known-content fixture; a sim parity gate asserting the scan
   matches OCCT XDE (`STEPCAFControl_Reader` + `XCAFDoc_DimTolTool`) on the covered
   semantic-PMI set; and a **byte-identical geometry proof** — the imported solid
   for a PMI-bearing file is bit-for-bit what it is today (the PMI scan is a separate
   pass, so this holds by construction and is proven, not assumed).

## Honest scope / decline boundary

- **In slice:** recognise, classify, count PMI annotation entities; expose per-class
  counts, per-annotation type + keyword, and the referenced attachment `#id`.
- **Out of slice (declared, not faked):** tolerance values / zones / modifiers,
  feature-control-frame semantics, datum reference frames, full GD&T meaning. A
  PMI class the reader cannot classify is counted as `Unknown` (never invented).
- **Decline:** if a PMI construct cannot be faithfully recognised, it is counted as
  `Unknown` or omitted — never fabricated. If the sim XDE oracle is unavailable for a
  class, that class is verified host-analytically. The geometry import ALWAYS stays
  the current honest PMI-skip solid, byte-identical.
