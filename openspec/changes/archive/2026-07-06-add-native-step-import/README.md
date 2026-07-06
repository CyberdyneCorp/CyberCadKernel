# add-native-step-import

A **NARROW, HONEST slice** of Phase 4 capability **#7 `native-exchange`**
(`openspec/NATIVE-REWRITE.md`) — the FIRST native STEP **import** slice, sitting on the
export slice (`add-native-data-exchange`) and the shape-healing slice
(`add-native-shape-healing`) already landed. It makes **`cc_step_import` native** in
`NativeEngine` for the AP203 `MANIFOLD_SOLID_BREP` subset the native writer (and OCCT)
emit for planar + elementary-quadric + basic B-spline solids: it tokenizes an
ISO-10303-21 (STEP Part 21) file into an entity table, maps the writer's entity set back to
native geometry/topology (the inverse of the export §3 map), drops the periodic-wall seam
artefact, and runs `heal::healShell` to close the file's sub-tolerance gaps into a
watertight native `topology::Shape` solid — with NO OCCT in `src/native/**`. Everything
outside the subset falls through to OCCT and is honestly labelled: **arbitrary / AP242 /
assembly / multi-root / non-manifold / unsupported-surface files DECLINE (NULL) → OCCT
`STEPControl_Reader`**, and **IGES import/export stay OCCT**.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT), does NOT
modify the STEP writer (`step_writer.cpp`) or the tessellator, and **never fabricates
geometry** the file did not describe. The correctness gates are a **host native round-trip**
(export → import reconstructs the SAME solid, OCCT-free) and a **sim vs OCCT** gate
(native-written + foreign OCCT-written STEP import natively and agree with OCCT
`STEPControl_Reader`).

## Why THIS subset is the achievable first import slice (and arbitrary import is not)

The native writer emits a fixed, known entity graph (`src/native/exchange/step_writer.cpp`):
one `MANIFOLD_SOLID_BREP` → one `CLOSED_SHELL` → N `ADVANCED_FACE` over five surface kinds,
bounded by `EDGE_LOOP`s of `ORIENTED_EDGE`→`EDGE_CURVE` over three curve kinds, with
`VERTEX_POINT`/`CARTESIAN_POINT`/`DIRECTION`/`AXIS2_PLACEMENT_3D` leaves and the AP203
product/context wrapper. Reading THAT subset back is the deterministic inverse of the
writer's map — and OCCT `STEPControl_Writer` emits the same subset for the same solids, so a
foreign OCCT-written box/cylinder is in scope too.

**Parsing an arbitrary foreign STEP file is the huge part** — the whole Part-42 + AP203 /
AP214 / AP242 schema (hundreds of entity types), external references, PMI, assemblies,
non-conforming files from many CAD systems, pcurve/sewing reconstruction, and freeform
re-approximation. That stays OCCT `STEPControl_Reader` — honest. This slice reads the
emitted subset and DECLINEs (NULL → OCCT) on anything outside it.

## Scope (NARROW + HONEST)

| Import case | Native in this change | Falls through / stays OCCT (honest, labelled) |
|---|---|---|
| **`cc_step_import` of a NATIVE-written STEP file** (the writer's AP203 manifold-solid-brep subset), or a **FOREIGN OCCT-written STEP** of the same subset (box / cylinder / holed / basic B-spline) | YES — tokenize → map → drop seam → `healShell` → watertight native solid; guarded by the native round-trip + sim-vs-OCCT gates | — |
| A file whose B-rep **`healShell` cannot heal** to a watertight positive-volume solid (gap beyond tolerance, genuinely open shell, non-manifold) | — | YES — reader returns NULL; engine falls through to OCCT `STEPControl_Reader` (tolerance NEVER widened) |
| A file with an **unsupported entity / surface** (`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`, `ELLIPSE`, rational/weighted B-spline), an **assembly / multiple roots**, or **AP242 / PMI** | — | YES — reader DECLINEs (NULL); OCCT `STEPControl_Reader` fall-through (no fabricated geometry) |
| A **non-millimetre** unit context | — | YES — DECLINE (no silent rescale) → OCCT |
| **`cc_iges_export`** / **`cc_iges_import`** | — | YES — stay OCCT `IGESControl_*`. Out of scope |

### Why the hard cases stay OCCT (not faked)

- **Arbitrary STEP import.** A production reader is a full tokenizer + a resolver for the
  whole Part-42 + AP203/AP214/AP242 schema + healing of foreign files. Reading the fixed
  writer-emitted subset is a tiny fraction of that; anything else DECLINEs → OCCT.
- **The heal step is mandatory and honest.** A STEP file writes each face's boundary
  independently at fp precision, so the reconstructed shell is a face soup coincident only
  within tolerance. `heal::healShell` closes it; if it returns `Unhealed`, the reader returns
  NULL and the engine defers to OCCT `ShapeFix` via `STEPControl_Reader`. The tolerance is
  never widened to force a pass.
- **No fabricated geometry.** The reader parses only what is in the file and maps only what
  it genuinely supports; a NULL result is the honest outcome that lets the engine fall
  through — it never invents a solid the file did not describe.

## Method (locked, per NATIVE-REWRITE.md)

CLEAN-ROOM from the STEP standard — **ISO 10303-21** (Part-21 syntax: `HEADER` / `DATA`
sections, `#n = ENTITY(...)` instance records, string/number/enum/list encoding) and
**ISO 10303-42** (the geometry/topology entities) — and the EXACT entity set of
`src/native/exchange/step_writer.cpp` (the writer this reader inverts), with OCCT source
(`STEPControl_Reader`, `RWStep*`, `StepToTopoDS_*`) consulted as a **reference ORACLE +
engine-side fallback only** — never copied.

## Architecture / OCCT boundary (unchanged from #1–#7)

- The native STEP reader lands under `src/native/exchange/` (new `step_parser.h/.cpp` +
  `step_reader.h/.cpp`, extending `native_exchange.h`) — **OCCT-FREE and host-buildable**
  (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). It includes only `src/native/{math,
  topology,heal}`, consumes a path/text, and returns a `topology::Shape` (NULL ⇒ DECLINE).
  It references no OCCT / `IEngine` / `EngineShape`.
- `src/engine/native/native_engine.cpp` — `step_import` gains a native-else-fallback path:
  call `step_import_native`, self-verify (valid watertight solid), wrap as a native body;
  on NULL / failed self-verify, fall through to OCCT `STEPControl_Reader` (labelled).
  `iges_export`, `iges_import`, `step_export`, `step_writer.cpp`, and the tessellator are
  UNCHANGED.
- **No `cc_*` ABI change**; default engine stays OCCT (opt-in via `cc_set_engine(1)`).

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host round-trip** (`clang++ -std=c++20`, no OCCT): a native solid →
   `step_export_native` → `step_import_native` → tessellate reconstructs the SAME solid
   (valid + watertight, volume / bbox / face+edge+vertex counts / topology EXACT) for a box,
   a cylinder / capped cylinder, a holed / typed-profile solid, and a B-spline-face solid;
   plus Part-21 tokenizer unit cases (typed reals, `''` strings, enums, `$`, `*`, nested
   lists, combined instances) and DECLINE cases (unsupported surface, multi-root, rational
   spline, unhealable B-rep) returning NULL.
2. **Simulator sim-vs-OCCT** (OCCT linked): (a) a native-written file imported by the native
   reader and by OCCT `STEPControl_Reader` agree (valid watertight, volume / bbox within
   tol); (b) a FOREIGN OCCT-written STEP (box / cylinder) imports natively and agrees with
   the OCCT re-import; (c) an assembly / unsupported-surface file DECLINEs natively and is
   imported by OCCT identical to `cc_set_engine(0)`. `xcrun simctl list devices booted`
   first; own `main()` on the `run-sim-suite.sh` SKIP list; default restored in teardown.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh`, host CTest, GPU / Phase-3) stays green at the OCCT default with
no regression to the export slice, shape healing, SSI S1–S4, S5 native-pass=6, native blends
+ #6/#7, marching, boolean, construct, tessellation, or phase3. Honestly reported as the
**first native import slice** (the writer-emitted AP203 manifold-solid-brep subset);
arbitrary / AP242 / IGES import remain OCCT, and #8 `drop-occt` stays blocked.
