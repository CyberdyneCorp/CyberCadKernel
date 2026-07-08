# Design — moat-m4pmi-ap242-pmi

## Context

`src/native/exchange/step_reader.cpp` parses a STEP DATA section into a
`std::unordered_map<int, Record>` table (`Parser::parse`), then `Mapper::build()`
reduces it to a native `topo::Shape`. PMI is never modelled: `assemblyDisposition()`
returns `AsmKind::None` for a relationship graph that reaches no
`MANIFOLD_SOLID_BREP`, so an AP242 solid-plus-PMI file imports the solid and drops
the PMI. The public entry points are `readStepFile(path)` / `step_import_native(path)
→ topo::Shape` and the C ABI `cc_step_import(path) → CCShapeId`.

Two hard invariants constrain this change:

- **`src/native/**` stays OCCT-free** (0 OCCT includes). OCCT lives only in
  `src/engine/occt`.
- **The geometry import stays byte-identical.** `cc_step_import` /
  `step_import_native` must return bit-for-bit the same solid they do today for
  every file, PMI-bearing or not.

## Goals / Non-goals

**Goals** — recognise, classify, and count AP242 PMI annotation entities; expose per
class counts + per-annotation type/keyword + attachment target `#id` as additive,
read-only native metadata; verify vs known fixture content (host) and vs OCCT XDE
(sim); keep geometry byte-identical.

**Non-goals** — a GD&T semantic model (tolerance magnitudes, zones, modifiers, FCF
parsing, datum reference frames); folding PMI into `topo::Shape`; any change to
`cc_step_import` behaviour; parsing arbitrary AP242 draughting presentation geometry.

## Key decision: a SEPARATE read-only pass, not a `build()` modification

The PMI scan is a **new pass** over the same `records` table, invoked by a new public
function. `Mapper::build()` and `readStepString()` are **untouched**. This is what
makes the byte-identical geometry guarantee hold *by construction*: no code on the
`step_import_native` path changes, so its output cannot change. The scan is a pure
function of the record table — no I/O, no OCCT, deterministic (annotations emitted in
ascending `#id` order, like `findManifoldBreps`).

## Native data model (OCCT-free, POD)

Declared in `step_reader.h`, re-exported via `native_exchange.h`:

```cpp
enum class PmiClass {
  Dimension,            // DIMENSIONAL_SIZE / _LOCATION, ANGULAR_SIZE / _LOCATION, …
  GeometricTolerance,   // GEOMETRIC_TOLERANCE + subtypes (position, flatness, …)
  Datum,                // DATUM, DATUM_FEATURE, DATUM_SYSTEM, DATUM_REFERENCE
  DatumTarget,          // DATUM_TARGET
  Note,                 // annotation notes / draughting callouts (text PMI)
  AnnotationGeometry,   // ANNOTATION_*_OCCURRENCE, DRAUGHTING_MODEL (graphical PMI)
  Unknown               // recognised as PMI-adjacent but not classifiable — never faked
};

struct PmiAnnotation {
  int       id = 0;          // the entity #id
  PmiClass  cls = PmiClass::Unknown;
  std::string keyword;       // the raw STEP keyword (audit trail)
  long      attachedTo = 0;  // referenced SHAPE_ASPECT / geometric-item / target #id, 0 = none
};

struct PmiSummary {
  std::size_t dimensions = 0, tolerances = 0, datums = 0,
              datumTargets = 0, notes = 0, annotationGeometry = 0,
              unknown = 0, total = 0;
  bool anyPmi = false;
  std::vector<PmiAnnotation> items;  // ascending #id
};

PmiSummary step_scan_pmi(const std::string& path);   // reads file, parses, scans
```

## Classification table (keyword → PmiClass)

The scan matches the record keyword (and, for combined instances, each sub-record
keyword) against a fixed table. Representative AP242 keywords:

- **Dimension:** `DIMENSIONAL_SIZE`, `DIMENSIONAL_LOCATION`,
  `DIMENSIONAL_LOCATION_WITH_PATH`, `ANGULAR_SIZE`, `ANGULAR_LOCATION`,
  `DIMENSIONAL_CHARACTERISTIC_REPRESENTATION`.
- **GeometricTolerance:** `GEOMETRIC_TOLERANCE` and the subtype set —
  `*_TOLERANCE` (position, flatness, straightness, roundness, cylindricity,
  perpendicularity, parallelism, angularity, circular_runout, total_runout,
  line_profile, surface_profile, concentricity, symmetry) and
  `GEOMETRIC_TOLERANCE_WITH_DATUM_REFERENCE`.
- **Datum / DatumTarget:** `DATUM`, `DATUM_FEATURE`, `DATUM_SYSTEM`,
  `DATUM_REFERENCE`, `DATUM_REFERENCE_COMPARTMENT` → Datum; `DATUM_TARGET` →
  DatumTarget.
- **Note / AnnotationGeometry:** `DRAUGHTING_CALLOUT`,
  `ANNOTATION_OCCURRENCE`, `ANNOTATION_CURVE_OCCURRENCE`,
  `ANNOTATION_FILL_AREA_OCCURRENCE`, `ANNOTATION_PLANE`, `DRAUGHTING_MODEL`,
  `TESSELLATED_ANNOTATION_OCCURRENCE`.

A keyword outside the table but reached only through a PMI relationship is counted
`Unknown`. The table is data, easy to extend, and unit-tested against the fixture.

## Attachment resolution (first slice)

`attachedTo` records the referenced target `#id` the annotation points at — for a
`DIMENSIONAL_SIZE` the `SHAPE_ASPECT` (arg ref), for a `GEOMETRIC_TOLERANCE` its
toleranced `SHAPE_ASPECT`, for a `DATUM_FEATURE` its `SHAPE_ASPECT`. The first slice
does NOT resolve the target down to a specific native face/edge (no reverse index
from `SHAPE_ASPECT` to a reconstructed `ADVANCED_FACE` is built yet) — it records the
`#id` faithfully and leaves face-level binding to a later slice. `0` means the
annotation carries no resolvable attachment reference. Nothing is invented.

## C ABI (additive-only)

```c
typedef struct {
  int dimensions, tolerances, datums, datum_targets,
      notes, annotation_geometry, unknown, total;
} CCPmiSummary;

/* Read-only PMI scan of a STEP file. Returns 1 and fills *out on success,
   0 on failure (see cc_last_error). Does NOT import geometry and does NOT
   alter cc_step_import. */
int cc_step_pmi_scan(const char *path, CCPmiSummary *out);
```

`cc_step_import` / `step_import_native` are untouched. `IEngine` gains a default
`pmi_scan` returning unsupported; `NativeEngine::pmi_scan` calls `step_scan_pmi`;
the OCCT engine may leave the default (the native scan is engine-independent parsing
and needs no OCCT).

## Verification strategy (two gates + regression)

1. **HOST ANALYTIC (no OCCT, `clang++ -std=c++20`).** Hand-author a minimal AP242
   fixture near `src/native/exchange` carrying a known PMI census — e.g. 2
   dimensions, 1 position tolerance, 1 datum, 1 note, each with a known
   `SHAPE_ASPECT` attachment `#id`. Assert `step_scan_pmi` returns exactly those
   per-class counts, the right `PmiClass` + keyword per item, and the right
   `attachedTo` ids. Also assert `anyPmi == false` and `total == 0` for a plain
   solid-only fixture.
2. **SIM native-vs-OCCT (`scripts/run-sim-native-step-import.sh` lane, XDE linked).**
   Load the same fixture (and a real foreign AP242 PMI file) with
   `STEPCAFControl_Reader` into an `XCAFDoc` document; query `XCAFDoc_DimTolTool`
   for dimensions / geometric tolerances / datums. Assert the native scan's counts
   for those semantic-PMI classes match OCCT XDE. Classes OCCT XDE does not surface
   (free graphical notes) are covered by gate 1, stated explicitly.
3. **BYTE-IDENTICAL GEOMETRY.** For every PMI-bearing fixture, assert the solid from
   `step_import_native` (volume / area / centroid / face+edge counts, and ideally a
   serialized-shape hash) is identical to `main`. Assert `cc_step_import`'s handle
   result and the existing STEP-import suite stay green.

## Honest decline

If a PMI construct is unrecognised it is counted `Unknown`, never fabricated. If the
sim XDE oracle cannot expose a class, that class is verified only host-analytically
and the design says so. The geometry import is never altered to accommodate PMI.

## Complexity

The scan is a single linear pass with a lookup table — target cognitive complexity
well within the backend budget (≤ 15). `classifyKeyword` is a flat switch/lookup;
`scanPmi` is one loop + sort. No change to `build()` complexity.
