# Proposal — add-native-step-import

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). The prior exchange slice (`add-native-data-exchange`)
landed native STEP **export** and honestly flagged **STEP import** as "the huge,
long-lived part" left on OCCT `STEPControl_Reader`. This change lands the FIRST native
STEP **import** slice — read an ISO-10303-21 (STEP Part 21) file and reconstruct a native
B-rep solid — **for exactly the AP203 manifold-solid-brep subset the native writer already
emits and OCCT emits for planar + elementary-quadric + basic B-spline solids**. It sits on
the shape-healing slice just landed (`add-native-shape-healing`): the reconstructed B-rep
carries the sub-tolerance gaps STEP data always has (each face's edges are written
independently at fp precision), so import runs `healShell` to close them into a watertight
solid before accepting the result.

Import is **not** the inverse-difficulty of export in general — parsing an *arbitrary*
foreign STEP file (the whole Part-42 + AP203/AP214 schema, hundreds of entity types,
external refs, AP242 PMI, assemblies, non-manifold, healing of ill-formed files) stays a
large long-lived effort. But the **narrow subset the writer emits** — the entity set in
`src/native/exchange/step_writer.cpp` (`CARTESIAN_POINT` / `DIRECTION` /
`AXIS2_PLACEMENT_3D` / `VERTEX_POINT`; `LINE` / `CIRCLE` / `B_SPLINE_CURVE_WITH_KNOTS`;
`PLANE` / `CYLINDRICAL_SURFACE` / `CONICAL_SURFACE` / `SPHERICAL_SURFACE` /
`B_SPLINE_SURFACE_WITH_KNOTS`; `EDGE_CURVE` / `ORIENTED_EDGE` / `EDGE_LOOP` /
`FACE_OUTER_BOUND` / `FACE_BOUND` / `ADVANCED_FACE` / `CLOSED_SHELL` /
`MANIFOLD_SOLID_BREP` / `ADVANCED_BREP_SHAPE_REPRESENTATION`) — **is** tractable, exactly
verifiable, and adds real native coverage. The map back is the mirror of §3 of the export
design.

The result is real and exactly verifiable: a native round-trip (native
`step_export_native` → `step_import_native`) must reconstruct the SAME solid, and a foreign
OCCT-written STEP of the same subset must import natively and match the OCCT
`STEPControl_Reader` re-import. Anything OUTSIDE the subset — an unsupported entity/surface,
an assembly, multiple roots, a non-manifold shell, or a B-rep that `healShell` cannot heal
to a valid watertight solid — returns a NULL / undecided result and the engine falls
through to the OCCT reader (labelled, honest). **We never fabricate geometry the file did
not describe.** This does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES +
a general-curved kernel still block it); it is the honest first native import slice.

## What changes

1. **A native STEP reader subtree** (`src/native/exchange/`, OCCT-free, host-buildable).
   New files (`step_parser.h/.cpp` for the ISO-10303-21 tokenizer + entity table,
   `step_reader.h/.cpp` for the AP203 B-rep entity map + `healShell` post-step, extending
   the `native_exchange.h` umbrella) that take a file path / text buffer and return a
   `topology::Shape` Solid (a NULL Shape ⇒ DECLINE / fall through). They include only
   `src/native/{math,topology,heal}`, never OCCT. The pipeline:
   - **Part 21 tokenizer + entity table.** Lex the DATA-section records
     `#N=ENTITY_NAME(arg,arg,...);` handling: integer refs `#M`; reals (incl. typed forms
     `1.E2`, `1.`); strings `'...'` (embedded `''` doubled); enums `.T.` / `.PLANE.`; lists
     `(...)`; `$` (null / undefined); `*` (derived); and the combined-instance
     `( SUB(...) SUB(...) )` form (units / context). Skip the HEADER. Build a
     `map<int, Record{keyword, args}>` with `Arg` a variant (ref / real / int / string /
     enum / list / null). Ignore unknown-keyword records (they are only reachable if
     nothing in scope references them; a referenced unknown ⇒ DECLINE).
   - **Two-pass mapper (mirror of the writer, §3 of `add-native-data-exchange/design.md`).**
     Pass A resolves leaf geometry, memoised by `#id`: `CARTESIAN_POINT` → `math::Point3`
     (mm); `DIRECTION` → `math::Dir3`; `AXIS2_PLACEMENT_3D` → `math::Ax3`;
     `LINE`/`CIRCLE`/`B_SPLINE_CURVE_WITH_KNOTS` → `topo::EdgeCurve`;
     `PLANE`/`CYLINDRICAL_/CONICAL_/SPHERICAL_SURFACE`/`B_SPLINE_SURFACE_WITH_KNOTS` →
     `topo::FaceSurface` (knots RLE-expanded from `(mults),(knots)`, poles row-major).
     Pass B builds topology following refs: `VERTEX_POINT` → `makeVertex`; `EDGE_CURVE` →
     `makeEdgeWithVertices` (dedup by `#id` — the writer already shares them); `ORIENTED_EDGE`
     → orientation onto the shared edge; `EDGE_LOOP` → `makeWire`; `FACE_OUTER_BOUND`
     (child 0) / `FACE_BOUND` (holes) + `ADVANCED_FACE` sense → `makeFace`; `CLOSED_SHELL`
     → `makeShell`; `MANIFOLD_SOLID_BREP` → `makeShell`/`makeSolid`. The periodic-wall SEAM
     edge the writer synthesises (a straight `LINE` used forward at u=period, reversed at
     u=0) is recognised and dropped from the reconstructed loop (it is a parametric closure
     artefact, not a physical edge).
   - **Root selection.** Find the single `ADVANCED_BREP_SHAPE_REPRESENTATION` →
     `MANIFOLD_SOLID_BREP`. Zero or **more than one** root solid ⇒ DECLINE (assembly /
     multi-root out of scope).
   - **Post-import heal.** Run `heal::healShell(solid, {tolerance})` to close the
     sub-tolerance gaps the independently-written per-face edges carry. `Healed` ⇒ return
     the watertight solid; `Unhealed` ⇒ return NULL (DECLINE → OCCT). The tolerance is
     NEVER widened to force a pass.
2. **`step_import_native(path)` API** (`native_exchange.h` + `step_reader.h`) returning a
   `topo::Shape` (NULL Shape ⇒ DECLINE). Additive; the existing `step_export_native` /
   `step_can_export_native` are unchanged.
3. **`NativeEngine::step_import` native-else-fallback glue**
   (`src/engine/native/native_engine.cpp`). `step_import` — today an unconditional
   `fallback().step_import(path)` — becomes: call `step_import_native(path)`; if it returns
   a non-null solid, **self-verify** it (valid watertight solid, enclosed volume > 0 via the
   native tessellate self-verify already used by heal/boolean) and wrap it as a native
   `EngineShape`; on NULL or a failed self-verify, fall through to OCCT
   `STEPControl_Reader` (labelled). No OCCT is compiled into `src/native/**`; the fallback
   lives on the engine side only.
4. **`iges_import` / `iges_export` / `step_export` behaviour unchanged.** `iges_*` stay
   `fallback().*` (OCCT). `step_export` keeps the export slice already landed. The STEP
   **writer** (`step_writer.cpp`) and the tessellator are NOT modified — import reads what
   the writer already produces.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **Arbitrary / foreign-schema STEP import** — the full Part-42 + AP203/AP214 entity
  schema, external references, non-conforming constructs from foreign CAD systems beyond
  the emitted subset. A referenced entity/keyword or a surface kind we do not model ⇒
  DECLINE → OCCT `STEPControl_Reader`.
- **AP242 / PMI / GD&T / colours / names / assemblies / multiple roots** — a file with more
  than one root solid, a product-structure assembly, or AP242 semantic PMI ⇒ DECLINE → OCCT.
- **Non-manifold / open / unhealable B-reps** — a shell that is non-manifold, or a B-rep
  `healShell` cannot close to a watertight positive-volume solid within tolerance ⇒ NULL
  → OCCT. No solid is invented that the file did not describe.
- **Rational / weighted B-spline curves & surfaces, `Ellipse`/`Bezier`, `TOROIDAL_SURFACE`,
  `SURFACE_OF_REVOLUTION`, and other non-writer-emitted geometry** — out of the subset ⇒
  DECLINE → OCCT.
- **IGES import/export** — a separate older format; stays OCCT `IGESControl_*`.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved
  kernel still block it. Reported honestly.

## Impact

- New OCCT-free, host-buildable files under `src/native/exchange/` (`step_parser.h/.cpp`,
  `step_reader.h/.cpp`) extending the `native_exchange.h` umbrella. New host CTest cases in
  a new `tests/native/test_native_step_reader.cpp` (Part 21 tokenizer unit cases + native
  round-trip: export → import → exact match, no OCCT) plus a facade case in
  `tests/test_native_engine.cpp` (native import success + fall-through). `step_writer.cpp`
  and the tessellator are UNCHANGED.
- `src/engine/native/native_engine.cpp` — `step_import` gains the native-else-fallback path
  + self-verify. `iges_export` / `iges_import` / `step_export` unchanged.
  `native_engine.h` unchanged (`step_import` signature already present).
- **No** `include/cybercadkernel/cc_kernel.h` signature change; **no**
  `src/facade/cc_kernel.cpp` change (`cc_step_import` already routes through the active
  engine). The `cc_step_import` doc-comment (returns a shape id) is the contract this
  change implements natively for the in-scope subset.
- A new sim parity/round-trip test (`tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh`) with its own `main()`, on the `run-sim-suite.sh`
  SKIP list so the suite assertion count is unchanged.
- Behaviour unchanged by default (engine stays OCCT); only callers that call
  `cc_set_engine(1)` see the native STEP reader. All existing suites stay green at the OCCT
  default; the export slice, healing, SSI S1–S4, S5 native-pass=6, native blends + #6/#7,
  marching, boolean, construct, tessellation, phase3 do NOT regress.

## Verification

Two independent gates from `NATIVE-REWRITE.md`:

1. **Host round-trip (OCCT-FREE — the core correctness).** Build a native solid →
   `step_export_native` to a string/file → `step_import_native` back → tessellate → assert
   the imported solid is valid + watertight with **volume / bbox / sub-shape counts /
   topology matching the original EXACTLY** (both ends native). This proves the reader
   inverts the writer. Plus Part-21 tokenizer unit cases (typed reals `1.E2`, strings with
   `''`, enums, `$`, nested lists, combined instances) and DECLINE cases (an unsupported
   surface keyword, a multi-root file, an unhealable B-rep) returning a NULL Shape.
2. **Sim vs OCCT (simulator, OCCT linked).** (a) Import a file the NATIVE writer produced
   with the native reader and with OCCT `STEPControl_Reader`; compare volume / watertight /
   valid within tolerance. (b) FOREIGN STEP: have OCCT `STEPControl_Writer` write a STEP
   file from an OCCT box/cylinder, import it NATIVELY, and compare vs the OCCT re-import —
   proving we read foreign-generated STEP of the subset. Plus the fall-through cases
   (assembly / unsupported surface) asserted identical under both engines.

Done only when both gates pass and every existing suite stays green at the OCCT default.
Reported honestly as the first native import slice (the writer-emitted AP203
manifold-solid-brep subset); arbitrary/AP242/IGES import remain OCCT and #8 `drop-occt`
stays blocked.
