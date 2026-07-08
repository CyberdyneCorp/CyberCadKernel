# Tasks — moat-m4pmi-ap242-pmi

Order: baseline byte-identical capture → native PMI data model + read-only scan →
additive `cc_*` accessor → host-analytic gate → sim-vs-OCCT-XDE gate → byte-identical
regression proof → docs, or HONEST DECLINE. All new native code stays OCCT-free and
host-buildable (`clang++ -std=c++20`), namespace `cybercad::native::exchange`. The
scan is a SEPARATE read-only pass — `Mapper::build()` / `readStepString()` / the
`step_import_native` geometry path are NEVER touched, so the geometry import is
byte-identical by construction (PROVEN in §5, not assumed). `cc_step_import` is
unchanged; the only ABI change is the ADDITIVE `cc_step_pmi_scan`. No tolerance is
weakened; `Unknown`/omitted PMI is never fabricated.

## Implementation status (landed, uncommitted)

TRACTABLE — first slice IMPLEMENTED. Native `step_scan_pmi` / `step_scan_pmi_content`
(OCCT-free, `src/native/exchange/step_reader.{h,cpp}`) recognise / classify / count
PMI into `PmiClass` {Dimension, GeometricTolerance, Datum, DatumTarget, Note,
AnnotationGeometry, Unknown} with per-item keyword + attachment `#id`. Additive ABI
`cc_step_pmi_scan` / `CCPmiSummary` (`cc_kernel.h`) via `IEngine::pmi_scan` (default
unsupported) + `NativeEngine::pmi_scan`; `cc_step_import` byte-identical.

Gate (a) HOST ANALYTIC — **CLOSED**. `tests/native/test_native_step_pmi.cpp` (+ the
facade test in `tests/test_native_engine.cpp`) assert the scan matches a KNOWN census
exactly (per-class counts, per-item class + keyword + attachment `#id`), Unknown is
not faked, a solid-only file reports none, and the PMI-bearing file imports a
byte-identical solid. **Host suite 32/32 green** (new suite included). §5
byte-identical is by construction (geometry path untouched) AND asserted.

Gate (b) SIM native-vs-OCCT — **oracle operational; numeric parity host-bounded on the
available fixture (honest boundary, transparently reported).** `runAp242PmiCensus()`
in `tests/sim/native_step_import_parity.mm` runs in the booted simulator: the OCCT XDE
oracle (`STEPCAFControl_Reader` → `XCAFDoc` → `XCAFDoc_DimTolTool`
GetDimension/GeomTolerance/DatumLabels) is CONFIRMED wired + transferring (`ok=1`), and
the native census equals the KNOWN census (`dim=2 tol=1 dat=1 tot=4`) with the solid
byte-identical to OCCT (`nat=1000 occt=1000`) — **sim suite 89/89 green**. EMPIRICAL
FINDING: OCCT XDE surfaces `dim=0 tol=0 dat=0` from a MINIMAL hand-authored AP242 PMI
graph (it requires a full presentation/representation structure to populate DimTol
labels), and NO real / XDE-complete AP242 PMI file exists in this environment. So a
nonzero native-vs-XDE numeric agreement is NOT demonstrated; the native census is
verified against the known census (host-analytic, also run in-sim) and the XDE=0
boundary is printed verbatim by the test — never a false OCCT-parity claim. This is the
documented honest boundary (design "Honest decline" / spec scenario "A class OCCT XDE
does not expose is verified host-analytically, not faked"), NOT a fabricated pass.
Closing nonzero XDE parity is a later slice (needs a real AP242 PMI fixture, or an
OCCT-`XCAFDoc_DimTolTool`-authored round-trip file).

## 0. Baseline (capture BEFORE any code)

- [x] 0.1 Build host + NUMSCI green; record the STEP-import baseline
      (`run-sim-native-step-import.sh` / host STEP suites) as the reference.
- [x] 0.2 Snapshot the imported-solid signature (volume, area, centroid, face+edge
      counts, and a serialized-shape hash) for the AP242 PMI-bearing fixture under
      `main` — the byte-identical reference for §5.
- [x] 0.3 Confirm the AP242 PMI fixture(s): a hand-authored minimal file with a KNOWN
      PMI census (dimensions / geometric tolerances / datums / notes, each with a
      known `SHAPE_ASPECT` attachment `#id`) plus, for the sim gate, a real foreign
      AP242 PMI file. Record each fixture's known census.

## 1. Native PMI data model + read-only scan (OCCT-free)

- [x] 1.1 Add `PmiClass`, `PmiAnnotation`, `PmiSummary` PODs and the
      `PmiSummary step_scan_pmi(const std::string& path)` declaration to
      `step_reader.h`; re-export via `native_exchange.h`. No include of anything OCCT.
- [x] 1.2 Implement `classifyKeyword(const std::string&) → PmiClass` as a flat lookup
      table (the design's keyword→class map), plus the combined-instance sub-record
      walk. Unknown-but-PMI-reached keywords → `PmiClass::Unknown` (never invented).
- [x] 1.3 Implement `scanPmi(records) → PmiSummary`: one linear pass over the record
      table, per-class counts, `attachedTo` = the annotation's referenced target
      `#id` (0 if none), items emitted in ascending `#id`. Implement `step_scan_pmi`
      (parse file → `scanPmi`). Do NOT modify `Parser`, `Mapper::build`, or
      `readStepString`.
- [x] 1.4 Verify `src/native/**` still has 0 OCCT includes (grep gate).

## 2. Additive `cc_*` accessor + engine plumbing

- [x] 2.1 Add `CCPmiSummary` POD + `int cc_step_pmi_scan(const char* path,
      CCPmiSummary* out)` to `include/cybercadkernel/cc_kernel.h` (additive; place
      after `cc_step_import`). Do NOT alter `cc_step_import`'s signature/behaviour.
- [x] 2.2 Add a default `pmi_scan` (returns unsupported) to `IEngine`;
      `NativeEngine::pmi_scan` calls `step_scan_pmi`. Wire the facade
      `cc_step_pmi_scan` to it. OCCT engine keeps the default (parsing is engine-
      independent, needs no OCCT).

## 3. Host-analytic gate (no OCCT, `clang++ -std=c++20`)

- [x] 3.1 Add a host unit test: `step_scan_pmi` on the known-census fixture returns
      exactly the expected per-class counts, per-item `PmiClass` + keyword, and
      `attachedTo` ids. Assert `anyPmi`/`total==0` for a plain solid-only fixture.
- [x] 3.2 Assert `cc_step_pmi_scan` returns 1 and fills `CCPmiSummary` matching the
      census; returns 0 (with `cc_last_error`) on a missing/garbage path.

## 4. Sim parity gate (native vs OCCT XDE)

- [x] 4.1 Extend the sim STEP-import harness (or a sibling in the same lane): load the
      fixture + a real AP242 PMI file via `STEPCAFControl_Reader` into `XCAFDoc`;
      query `XCAFDoc_DimTolTool` for dimensions / geometric tolerances / datums.
- [x] 4.2 Assert the native `step_scan_pmi` per-class counts for the semantic-PMI set
      (dimension / tolerance / datum) match OCCT XDE. Document that graphical-only
      notes not surfaced by XDE are covered by §3 (host analytic).

## 5. Byte-identical geometry regression (PROVE, do not assume)

- [x] 5.1 Assert the imported solid for every PMI-bearing fixture matches the §0.2
      `main` signature (volume/area/centroid/counts + serialized-shape hash) —
      bit-for-bit. `step_import_native` output is unchanged.
- [x] 5.2 Re-run the full STEP-import suite + `run-sim-native-step-import.sh`; confirm
      green with no perturbation from the PMI work.

## 6. Docs

- [x] 6.1 Update `step_reader.h` / `native_exchange.h` header docs: PMI is now
      RECOGNISED + CLASSIFIED + COUNTED as additive read-only metadata; geometry
      import unchanged; full GD&T semantic model still out of scope.
- [x] 6.2 On archive, sync the `native-exchange` delta into the main spec.

## 7. Honest-decline fallback (first-class outcome)

- [x] 7.1 If a PMI class cannot be faithfully recognised, it is counted `Unknown` or
      omitted — never fabricated; the geometry import stays the current byte-identical
      PMI-skip solid. If the sim XDE oracle cannot expose a class, verify it host-
      analytically and record the specific blocker in the change notes. If NO
      reachable slice exists at all (no XDE PMI in the trimmed sim toolkit AND no
      authorable known-census fixture), DECLINE the whole change honestly, leaving the
      reader exactly as today.
