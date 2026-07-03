# add-native-data-exchange

A **NARROW, HONEST slice** of Phase 4 capability **#7 `native-exchange`**
(`openspec/NATIVE-REWRITE.md`). This change makes ONE half of data exchange native:
**STEP EXPORT of a native-built solid**. It writes a native `topology::Shape` solid to a
valid **ISO 10303-21** (STEP AP203) exchange file — the exchange header plus the Part-42
B-rep entity graph in **true millimetres** — with NO OCCT. Everything else in exchange
stays OCCT and is honestly labelled: **STEP import stays OCCT**, **IGES export/import stay
OCCT**, and any solid that cannot be faithfully serialized natively **falls through to
OCCT `STEPControl_Writer`** (labelled, never faked).

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT), and
does NOT emit a STEP file that reads back as a different or invalid solid. The correctness
gate is a **native-write / OCCT-read round-trip**: the native STEP file MUST re-read
through OCCT `STEPControl_Reader` to the SAME solid (volume / bbox / topology within
tolerance).

## Why export-only is the achievable native ceiling (and import + IGES are not)

Writing STEP is a **serialization** of a B-rep the native kernel already owns: enumerate
the topology (#2 `native-topology`), read the attached analytic / NURBS geometry (#1
`native-math`), and emit the corresponding ISO-10303-21 records. It is deterministic,
closed-form, and exactly verifiable (OCCT re-reads it). No new geometry algorithm is
needed — the entity kinds map 1:1 onto the geometry the native kernel already produces
(`Plane` / `Cylinder` / `Cone` / `Sphere` / `BSpline` surfaces; `Line` / `Circle` /
`B_SPLINE_CURVE` curves).

**Parsing arbitrary STEP (import) is the huge part** — a full ISO-10303-21 tokenizer, an
entity-graph resolver for the entire Part-42 + AP203/AP214 schema (hundreds of entity
types, external references, healing of ill-formed files from dozens of foreign CAD
systems), and reconstruction into a valid B-rep with pcurves and shape healing. That is a
large, long-lived effort and is explicitly OUT OF SCOPE — it stays OCCT
`STEPControl_Reader`. **IGES** (an older, looser, entity-numbered format with its own
parser and writer) is likewise OUT OF SCOPE and stays OCCT.

So the honest end state of #7 after this change: **STEP export is native** (verified by an
OCCT round-trip); STEP import + IGES export/import remain OCCT. This is the achievable
native ceiling for exchange. **Full native STEP import + IGES + a general-curved kernel
are what still block #8 `drop-occt`** — this change does NOT unblock it.

## What data exchange is (the `cc_*` contract — unchanged)

```c
/* STEP exchange. cc_step_export returns 1 on success. */
int cc_step_export(CCShapeId body, const char *path);
CCShapeId cc_step_import(const char *path);

/* IGES exchange (millimetres, B-rep mode). cc_iges_export returns 1 on success. */
int cc_iges_export(CCShapeId body, const char *path);
CCShapeId cc_iges_import(const char *path);
```

- **`cc_step_export`** — write `body` to a STEP AP203 file at `path`; returns `1` on
  success, `0` on failure. This is the `cc_*` mirror of OCCT
  `STEPControl_Writer::Transfer` + `Write`. **This change makes it native for
  native-representable solids.**
- **`cc_step_import`** / **`cc_iges_export`** / **`cc_iges_import`** — unchanged, stay
  OCCT. No ABI change.

## Scope (NARROW + HONEST) — `cc_step_export` native for native-representable solids

| Exchange case | Native in this change | Falls through / stays OCCT (honest, labelled) |
|---|---|---|
| **`cc_step_export` of a NATIVE-built solid** whose faces are `Plane` / `Cylinder` / `Cone` / `Sphere` / `BSpline` and whose edges are `Line` / `Circle` / `BSpline` — a manifold `CLOSED_SHELL` | YES — emit ISO-10303-21 header + Part-42 B-rep entity graph in true mm; guarded by the OCCT-read round-trip self-check bar | — |
| A native solid whose geometry is **unrepresentable** in the emitted Part-42 subset (an `Ellipse` edge, a `Bezier` surface not down-converted, a non-manifold / open shell, a degenerate face) | — | YES — the native writer DECLINES; the engine falls through to OCCT `STEPControl_Writer` (labelled, never a faked file) |
| **`cc_step_export` of a FOREIGN / OCCT-built solid** (an OCCT `TopoDS_Shape`, not a native `topology::Shape`) | — | YES — the native writer has no native B-rep to serialize; OCCT `STEPControl_Writer` fall-through |
| A native STEP file that **fails the round-trip self-check** (OCCT re-reads it to a different volume / bbox / topology, or to an invalid shape) | — | YES — the engine **DISCARDS** the native file and falls through to OCCT (never emits a file that reads back wrong) |
| **`cc_step_import`** (parse an arbitrary STEP file) | — | YES — stays OCCT `STEPControl_Reader`. Out of scope (the large parsing effort) |
| **`cc_iges_export`** / **`cc_iges_import`** | — | YES — stay OCCT `IGESControl_*`. Out of scope |

### Why the hard cases stay OCCT (not faked)

- **STEP import.** A production STEP reader is a full tokenizer + a resolver for the whole
  Part-42 + AP203/AP214 entity schema + healing of files from many foreign CAD systems.
  Writing a valid file (a fixed, known entity graph) is a tiny fraction of reading an
  arbitrary one. Import stays OCCT — honest.
- **IGES.** A separate, older format with its own writer and parser and its own quirks.
  Not touched by this change; stays OCCT.
- **Unrepresentable / foreign solids.** The native writer emits ONLY the Part-42 subset
  the native kernel's geometry maps onto. Anything outside that subset (an `Ellipse`
  edge, an un-down-converted `Bezier`, a non-manifold shell) or any non-native body is a
  labelled OCCT fall-through, not a partial / lossy native file.
- **The round-trip self-check is mandatory.** A native STEP file is accepted ONLY if OCCT
  `STEPControl_Reader` re-reads it to the SAME solid (volume / bbox / topology within
  tolerance) and the shape is valid; otherwise it is DISCARDED and the engine falls
  through to OCCT. The kernel NEVER ships a STEP file that reads back as a different or
  invalid solid.

## Method (locked, per NATIVE-REWRITE.md)

CLEAN-ROOM from the STEP standard — **ISO 10303-21** (the Part-21 exchange-file syntax:
`HEADER` / `DATA` sections, `FILE_DESCRIPTION` / `FILE_NAME` / `FILE_SCHEMA`, `#n = ENTITY(...)`
instance records) and **ISO 10303-42** (the integrated geometric & topological
representation resources: `CARTESIAN_POINT`, `DIRECTION`, `AXIS2_PLACEMENT_3D`,
`VERTEX_POINT`, the curves `LINE` / `CIRCLE` / `B_SPLINE_CURVE_WITH_KNOTS`, `EDGE_CURVE`,
`ORIENTED_EDGE`, `EDGE_LOOP`, `FACE_BOUND` / `FACE_OUTER_BOUND`, the surfaces `PLANE` /
`CYLINDRICAL_SURFACE` / `CONICAL_SURFACE` / `SPHERICAL_SURFACE` / `B_SPLINE_SURFACE`,
`ADVANCED_FACE`, `CLOSED_SHELL`, `MANIFOLD_SOLID_BREP`) plus the AP203 product / context
wrapper (`ADVANCED_BREP_SHAPE_REPRESENTATION` + `PRODUCT` / `PRODUCT_DEFINITION` /
`APPLICATION_CONTEXT` / units in mm) — and the `cc_step_export` contract
(`include/cybercadkernel/cc_kernel.h`), with OCCT source
(`/Users/leonardoaraujo/work/OCCT/src`: `STEPControl_Writer`, `STEPCAFControl`,
`GeomToStep_*`, `TopoDSToStep_*`, `RWStepShape_*`) consulted as a **reference ORACLE
only** — to confirm the entity graph shape, the record field order, and the units /
context wrapper — never copied.

## Architecture / OCCT boundary (unchanged from #1–#6)

- The native STEP writer lands under a new **`src/native/exchange/`** subtree (stays
  **OCCT-FREE and host-buildable**, `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no
  OCCT, no simulator). It includes only `src/native/math` and `src/native/topology`
  (walk the shape, read its geometry) and emits a **string / byte buffer** of ISO-10303-21
  text. It references no OCCT / `IEngine` / `EngineShape` type. It reports DECLINE (empty
  buffer) for any solid outside the representable subset.
- `src/engine/native/native_engine.cpp` — `step_export` gains a native-else-fallback
  path: when `body` is a native body whose geometry is representable, serialize natively,
  write the buffer to `path`, and run the **mandatory OCCT-read round-trip self-check**
  (`STEPControl_Reader` re-reads the file; volume / bbox / topology compared vs the source
  native solid within tolerance; the read shape must be valid). If the writer DECLINES or
  the self-check FAILS, the engine falls through to OCCT `STEPControl_Writer` (labelled).
  `step_import`, `iges_export`, `iges_import` are UNCHANGED — they stay
  `fallback().*` (OCCT).
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via `cc_set_engine(1)`),
  so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host structural unit tests** (`clang++ -std=c++20`, no OCCT): the native writer over
   a native-built solid emits a **well-formed ISO-10303-21 buffer** — asserted
   structurally without OCCT: the `ISO-10303-21;` / `HEADER;` / `DATA;` / `ENDSEC;` /
   `END-ISO-10303-21;` framing; a `FILE_SCHEMA` naming the AP203 schema; a millimetre
   `SI_UNIT` context; exactly one `MANIFOLD_SOLID_BREP` referencing one `CLOSED_SHELL`;
   an `ADVANCED_FACE` per native face with the right surface entity kind
   (`PLANE` / `CYLINDRICAL_SURFACE` / `CONICAL_SURFACE` / `SPHERICAL_SURFACE` /
   `B_SPLINE_SURFACE`); `EDGE_CURVE` / `ORIENTED_EDGE` / `EDGE_LOOP` with `LINE` /
   `CIRCLE` / `B_SPLINE_CURVE_WITH_KNOTS` curves; `VERTEX_POINT` / `CARTESIAN_POINT` /
   `DIRECTION` leaves; every `#n` reference resolves (no dangling forward references);
   coordinates in mm. Plus DECLINE cases (an `Ellipse` edge, a non-manifold shell, an
   unrepresentable surface) yielding an empty buffer.
2. **Simulator native-write / OCCT-read round-trip** (the correctness gate): on a booted
   iOS simulator (OCCT linked), a native-built solid is exported with the native engine
   active (`cc_set_engine(1)`), the resulting file is re-read through OCCT
   `STEPControl_Reader`, and the round-tripped shape is compared vs the source native
   solid — **volume / bbox / sub-shape counts / topology within tolerance**, and the read
   shape valid (`BRepCheck`). The SAME solid exported via OCCT `STEPControl_Writer`
   (`cc_set_engine(0)`) is the oracle. The fall-through cases (a foreign / OCCT-built
   solid, an unrepresentable native solid, and a file that would fail the self-check) are
   asserted to produce a valid OCCT-written file identical to `cc_set_engine(0)`
   (fall-through proof). `cc_step_import`, `cc_iges_export`, `cc_iges_import` are asserted
   UNCHANGED (still OCCT under both engine settings). Default restored in teardown; own
   `main()` (on the `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is
   unchanged.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green at the OCCT
default. This is honestly reported as the **export-only native ceiling** for exchange;
STEP import + IGES export/import remain OCCT, and full native import + IGES + a
general-curved kernel are what still block #8 `drop-occt`.
