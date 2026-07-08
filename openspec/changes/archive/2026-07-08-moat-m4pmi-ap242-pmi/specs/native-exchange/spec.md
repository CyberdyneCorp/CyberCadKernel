# native-exchange

## ADDED Requirements

### Requirement: Native AP242 PMI recognise + classify + count as additive read-only metadata

The native STEP reader SHALL provide a read-only scan
(`step_scan_pmi(path) → PmiSummary`, and the additive facade
`cc_step_pmi_scan(path, &CCPmiSummary)`) that RECOGNISES the AP242 PMI / GD&T /
draughting annotation entities in a STEP file, CLASSIFIES each into a native
`PmiClass` (Dimension, GeometricTolerance, Datum, DatumTarget, Note,
AnnotationGeometry, or Unknown), COUNTS them per class, and records for each
annotation its raw STEP keyword and its referenced attachment target `#id` (the
`SHAPE_ASPECT` / geometric-item / dimensional-characteristic it points at, `0` when
none). The scan SHALL be a SEPARATE pass over the parsed Part-21 record table that
does NOT invoke or modify the geometry mapper (`Mapper::build`) — it is pure,
deterministic (annotations in ascending `#id` order), and OCCT-free
(`src/native/**` retains 0 OCCT includes). A keyword reached as PMI but outside the
recognised classification table SHALL be counted `Unknown` and NEVER fabricated into
a specific class; tolerance magnitudes, zones, modifiers, feature-control-frame
semantics, and datum reference frames are explicitly OUT of this slice and SHALL NOT
be invented.

#### Scenario: A recognised PMI census is classified and counted

- GIVEN a STEP file carrying AP242 PMI — dimensions (`DIMENSIONAL_SIZE` /
  `DIMENSIONAL_LOCATION` / `ANGULAR_SIZE`), geometric tolerances
  (`GEOMETRIC_TOLERANCE` and its `*_TOLERANCE` subtypes), datums (`DATUM` /
  `DATUM_FEATURE`), and notes / annotation-occurrence entities
- WHEN `step_scan_pmi` (or `cc_step_pmi_scan`) is called on the file
- THEN each recognised annotation SHALL be classified into the correct `PmiClass`,
  the per-class counts and `total` SHALL reflect the file's PMI content, each item
  SHALL carry its raw keyword and its referenced attachment `#id`, and `anyPmi`
  SHALL be true

#### Scenario: A PMI-adjacent but unclassifiable entity is counted Unknown, not faked

- GIVEN a STEP file whose PMI graph reaches an entity keyword outside the recognised
  classification table
- WHEN the scan classifies that entity
- THEN it SHALL be counted as `PmiClass::Unknown` (its keyword preserved), no
  specific dimension / tolerance / datum class SHALL be fabricated for it, and no
  tolerance value or semantic meaning SHALL be invented

#### Scenario: A solid-only file reports no PMI

- GIVEN a STEP file with a `MANIFOLD_SOLID_BREP` and no PMI / annotation entities
- WHEN `step_scan_pmi` is called
- THEN `anyPmi` SHALL be false, `total` SHALL be 0, and every per-class count SHALL
  be 0

### Requirement: The PMI scan is additive and keeps the geometry import byte-identical

Adding the PMI scan SHALL NOT change the geometry import in any way. The existing
`step_import_native(path) → topo::Shape`, `readStepFile`, and the `cc_step_import`
facade SHALL retain their exact signatures and behaviour, and the imported solid for
ANY file — PMI-bearing or not — SHALL be BYTE-IDENTICAL to the pre-change result. The
new PMI accessor SHALL be additive-only: `cc_step_pmi_scan` / `CCPmiSummary` are NEW
symbols, and no existing `cc_*` accessor SHALL be modified or removed. The AP242
PMI-skip behaviour (a relationship / annotation graph reaching no
`MANIFOLD_SOLID_BREP` is skipped so the solid still imports) SHALL be preserved
unchanged; the PMI is now additionally EXPOSED as metadata, not folded into the
geometry.

#### Scenario: Geometry import is unchanged for a PMI-bearing file

- GIVEN an AP242 file carrying both a `MANIFOLD_SOLID_BREP` and PMI annotations
- WHEN the file is imported via `step_import_native` / `cc_step_import`
- THEN the resulting solid's volume, area, centroid, face and edge counts, and
  serialized-shape hash SHALL be bit-for-bit identical to the pre-change import, and
  the existing STEP-import suite SHALL stay green

#### Scenario: The PMI accessor is purely additive

- GIVEN the change introduces `cc_step_pmi_scan` / `CCPmiSummary`
- WHEN the public ABI is compared to the pre-change ABI
- THEN `cc_step_import` and every other existing `cc_*` symbol SHALL be unchanged,
  and only the new PMI symbols SHALL have been added

### Requirement: PMI scan is verified host-analytically against a known census (no OCCT)

The PMI scan SHALL be verified on the HOST ANALYTIC gate (`clang++ -std=c++20`, no
OCCT linked): against a fixture whose PMI content is KNOWN, the scan's per-class
counts, per-item `PmiClass` + keyword, and per-item `attachedTo` target `#id` SHALL
match the fixture's known census exactly. This gate SHALL cover every class the scan
recognises, including graphical-only notes that an OCCT XDE oracle does not surface.

#### Scenario: Scan matches the hand-authored fixture census exactly

- GIVEN a hand-authored AP242 fixture with a known census (e.g. 2 dimensions, 1
  position tolerance, 1 datum, 1 note, each with a known `SHAPE_ASPECT` attachment
  `#id`)
- WHEN `step_scan_pmi` is run on it with no OCCT present
- THEN the returned counts, per-item classes + keywords, and `attachedTo` ids SHALL
  equal the known census, proving recognition/classification/count without any oracle
  dependency

### Requirement: PMI scan matches OCCT XDE on the covered semantic-PMI set (simulator gate)

Where OCCT exposes PMI, the native scan SHALL be verified against the OCCT oracle on
the SIM gate: the same fixture (and a real foreign AP242 PMI file) loaded via
`STEPCAFControl_Reader` into an `XCAFDoc` document and queried through
`XCAFDoc_DimTolTool` for dimensions, geometric tolerances, and datums. The native
scan's per-class counts for that semantic-PMI set SHALL match OCCT XDE. Classes OCCT
XDE does not surface (free graphical notes) SHALL be covered by the host-analytic
gate and that boundary SHALL be documented — never silently claimed as
OCCT-verified.

#### Scenario: Native counts match OCCT XDE for dimensions, tolerances, and datums

- GIVEN a booted simulator with the XDE toolkits linked (`TKXCAF` / `TKDESTEP`) and
  an AP242 PMI file
- WHEN the file is read by OCCT `STEPCAFControl_Reader` into `XCAFDoc` and its
  dimensions / geometric tolerances / datums are counted via `XCAFDoc_DimTolTool`,
  and independently scanned by the native `step_scan_pmi`
- THEN the native per-class counts for the semantic-PMI set SHALL match the OCCT XDE
  counts

#### Scenario: A class OCCT XDE does not expose is verified host-analytically, not faked

- GIVEN a PMI class (e.g. a free graphical note) that OCCT XDE `XCAFDoc_DimTolTool`
  does not surface
- WHEN that class is verified
- THEN it SHALL be checked against the known-census host gate and the design SHALL
  state that this class is host-verified, not OCCT-verified — no false OCCT-parity
  claim SHALL be made

### Requirement: Honest decline when a PMI slice is unreachable

The reader SHALL DECLINE a PMI scope honestly whenever a reachable slice does not
exist — no native PMI data model can faithfully represent a construct, the trimmed
sim toolkit exposes no XDE PMI, AND no known-census fixture can be authored to verify
it. In that case the reader SHALL report the specific blocker and SHALL leave the
geometry import exactly as today (the PMI-skip solid, byte-identical). No annotation data SHALL be fabricated to
manufacture a passing result, and no tolerance SHALL be weakened to force recognition.

#### Scenario: Unrepresentable PMI is declined, geometry preserved

- GIVEN a PMI construct the first slice cannot faithfully recognise or verify on
  either gate
- WHEN the reader processes the file
- THEN it SHALL count that construct `Unknown` or omit it (never fabricate a class or
  value), the specific blocker SHALL be reported, and the imported solid SHALL remain
  the current byte-identical PMI-skip result
