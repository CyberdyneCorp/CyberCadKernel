# Tasks — add-native-step-import (Phase 4 #7 slice — native STEP import)

Order: Part 21 tokenizer + entity table → two-pass STEP→native entity map (mirror
`step_writer.cpp`) → seam-edge drop → post-import `healShell` + root/DECLINE gate → native
reader API → engine native-else-fallback wiring + self-verify → Gate 1 (host round-trip,
no OCCT) → Gate 2 (sim vs OCCT + foreign STEP) → docs. Native code stays OCCT-free +
host-buildable (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*` ABI change.
Default engine stays OCCT. `step_writer.cpp` + the tessellator are NOT modified. `iges_*`
and `step_export` stay unchanged.

## 1. Part 21 tokenizer + entity table (`step_parser.h/.cpp`, OCCT-free)

- [x] 1.1 Char-scanner: skip whitespace + `/* */` comments; read the DATA section only
      (skip HEADER up to `DATA;`, stop at `ENDSEC;`).
- [x] 1.2 Value-form sub-parsers producing `Arg{kind, ...}`: ref `#M`, int, real (typed
      forms `1.`, `1.E2`, `-3.5E-07`), string `'...'` (embedded `''` → `'`), enum
      `.NAME.`, list `( ... )` nested, null `$`, derived `*`.
- [x] 1.3 Record parser: `#N = KEYWORD ( args ) ;` → `Record{id, keyword, args}` into a
      `map<int, Record>`; combined-instance `( SUB(...) SUB(...) )` stored with keyword
      `""` + sub-records; malformed record → parse error (surfaced as DECLINE).
- [x] 1.4 Host-buildable, includes only STL + `src/native/{math,topology,heal}`; the
      value-form dispatch documented as the one systems-band function (≤ ~25).

## 2. STEP → native leaf-geometry map — Pass A (`step_reader.cpp`, OCCT-free)

- [x] 2.1 `CARTESIAN_POINT` → `math::Point3` (mm); `DIRECTION` → `math::Dir3`;
      `AXIS2_PLACEMENT_3D` → `math::Ax3` (Y = Z×X, mirroring `worldFrame`; `$` axis/ref →
      default). Memoised by `#id`.
- [x] 2.2 Curves: `LINE`(+`VECTOR`) → `EdgeCurve{Line}`; `CIRCLE` → `EdgeCurve{Circle}`;
      `B_SPLINE_CURVE_WITH_KNOTS` → `EdgeCurve{BSpline}` with knots RLE-expanded from
      `(mults),(knots)` (inverse of `compressKnots`). Rational/weighted wrap → DECLINE.
- [x] 2.3 Surfaces: `PLANE` / `CYLINDRICAL_SURFACE` / `CONICAL_SURFACE` /
      `SPHERICAL_SURFACE` → `FaceSurface` analytic; `B_SPLINE_SURFACE_WITH_KNOTS` →
      `FaceSurface{BSpline}` (degreeU/V, nPolesU/V, poles row-major from the list-of-U-rows,
      knotsU/V RLE-expanded). Any other keyword/kind → DECLINE.

## 3. STEP → native topology map — Pass B (`step_reader.cpp`, OCCT-free)

- [x] 3.1 `VERTEX_POINT` → `makeVertex` (memoised by `#id`); `EDGE_CURVE` →
      `makeEdgeWithVertices` (ONE per `#id`; param range from projecting the two vertices
      onto the resolved curve). Dedup shared edges/vertices by `#id` (writer already shares).
- [x] 3.2 `ORIENTED_EDGE` → the shared edge `.oriented(Forward/Reversed)`; `EDGE_LOOP` →
      `makeWire`; `FACE_OUTER_BOUND` (child 0) / `FACE_BOUND` (holes) + `ADVANCED_FACE` sense
      (`.F.` → `Orientation::Reversed`) → `makeFace`.
- [x] 3.3 `CLOSED_SHELL` → collected faces; `MANIFOLD_SOLID_BREP` → root candidate;
      `ADVANCED_BREP_SHAPE_REPRESENTATION` → the single ROOT. Zero / >1 root → DECLINE.

## 4. Seam-edge drop + post-import heal + DECLINE gate (`step_reader.cpp`)

- [x] 4.1 Detect the writer's periodic-wall SEAM: within one `EDGE_LOOP`, an `EDGE_CURVE`
      referenced by two `ORIENTED_EDGE`s of OPPOSITE sense; drop that pair from the
      reconstructed wire so the native face keeps only its real rim edges (round-trip edge
      count stays exact vs the native source).
- [~] 4.2 Post-import watertightness. DESIGN CHANGE (honest, verified against the heal
      source): `heal::healShell` rebuilds EVERY face as a best-fit PLANE with straight Line
      edges (`face_soup.h` / `assemble_shell.h`), which would PLANARIZE and destroy a
      cylinder/cone/sphere/B-spline face and its volume — so running it inline on a curved
      import is wrong. Instead the reconstruction shares vertex+edge NODES by `#id` exactly
      as the writer shared them, so adjacent faces reference the SAME edge node and the
      solid is watertight BY CONSTRUCTION (the tessellator's shared-edge weld). The ENGINE
      then self-verifies robustly watertight + volume>0 (task 6.1) and, on ANY failure,
      declines to OCCT — never a planarized or fabricated solid. Tolerance never widened.
- [x] 4.3 DECLINE (NULL Shape) on: unknown/unsupported referenced keyword or surface/curve
      kind; rational-spline wrap; non-mm unit context; malformed/dangling record; >1 root;
      `Unhealed`. Never return a partial / invented solid.

## 5. Native reader API (`step_reader.h` + `native_exchange.h`, OCCT-free umbrella)

- [x] 5.1 `step_import_native(const std::string& path)` → `topo::Shape` (NULL ⇒ DECLINE)
      and a `step_import_native_string(text)` overload for host tests. Extend
      `native_exchange.h`; do NOT touch `step_export_native` / `step_writer.*`.
- [x] 5.2 Confirm the whole `src/native/exchange/` subtree compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic` with NO OCCT
      and NO simulator (clean, no warnings).

## 6. Engine wiring + self-verify (`src/engine/native/native_engine.cpp`)

- [x] 6.1 `step_import`: call `step_import_native(path)`; on a non-null solid, self-verify
      it (valid watertight solid, enclosed volume > 0 via the native tessellate self-verify)
      and wrap as a native `EngineShape`; return the tracked handle.
- [x] 6.2 On NULL or failed self-verify, fall through to OCCT `STEPControl_Reader`
      (labelled). No native void handed to OCCT; the fallback re-reads the FILE.
- [x] 6.3 Leave `iges_export`, `iges_import`, `step_export` UNCHANGED. Confirm
      `src/native/**` still has ZERO OCCT includes/symbols (grep gate).

## 7. Gate 1 — host round-trip unit tests (no OCCT)

- [x] 7.1 `tests/native/test_native_step_reader.cpp` tokenizer cases: typed reals
      (`1.`/`1.E2`/`-3.5E-07`), string with `''`, enums, `$`, `*`, nested lists, a combined
      instance; a malformed record → parse error.
- [x] 7.2 Round-trip EXACT: native box (planar) → `step_export_native` →
      `step_import_native` → tessellate → valid + watertight, volume / bbox / face+edge+
      vertex counts / topology match the original EXACTLY.
- [x] 7.3 Round-trip EXACT: native cylinder + capped cylinder (elementary quadric) and a
      holed / typed-profile solid — volume/topology exact (seam dropped → edge count exact).
- [ ] 7.4 Round-trip: a B-spline-face solid — DEFERRED, honestly. The reader's
      `B_SPLINE_CURVE_WITH_KNOTS` / `B_SPLINE_SURFACE_WITH_KNOTS` mapping (degree, row-major
      poles, RLE-expanded knots, non-rational gate) is IMPLEMENTED and reviewed, but NO
      `cc_solid_*` construct path (nor the writer's fixtures) emits a WATERTIGHT B-spline-FACE
      solid — the scratchpad hand-built single B-spline face is an open shell, not a closed
      solid, so it has no volume to round-trip. Fixturing an artificial watertight
      bspline-face solid would be fabricated coverage; left unchecked until a native builder
      (loft/sweep freeform) produces one. Not required for the box/quadric round-trip that is
      this slice's stated goal.
- [x] 7.5 DECLINE cases → NULL Shape (in `test_native_step_reader`): `TOROIDAL_SURFACE`
      (unsupported surface keyword), a two-root file (assembly), a non-mm `SI_UNIT`, a
      malformed record (unterminated string), and empty / no-DATA input — all return NULL so
      the engine falls to OCCT. (Rational-spline + unknown-curve declines are enforced in the
      mapper's `bsplineCurve`/`bsplineSurface`/`curve` guards — knot-length + non-rational
      checks — but are not separately fixtured here; no facade path emits them.)
- [x] 7.6 Facade case in `tests/test_native_engine.cpp`
      (`native_step_import_reads_native_file`): native `cc_step_import` of a native-written
      file returns a valid body; an unsupported file falls through (OCCT).
- [x] 7.7 Wire into host CTest (`test_native_step_reader`); all existing native suites green.

## 8. Gate 2 — sim vs OCCT + foreign STEP (the correctness gate)

- [x] 8.1 `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh`
      via the `cc_*` facade under `cc_set_engine(1)`; `xcrun simctl list devices booted`.
- [x] 8.2 Import a NATIVE-writer file with the native reader and with OCCT
      `STEPControl_Reader`; assert both valid watertight solids, volume / bbox within tol.
- [x] 8.3 FOREIGN STEP: OCCT `STEPControl_Writer` writes a box/cylinder STEP from an OCCT
      solid; import it NATIVELY and compare vs the OCCT re-import (volume / watertight / valid
      within tol) — proving we read foreign-generated STEP of the subset.
- [~] 8.4 Fall-through: DECLINE is covered exhaustively by the HOST Gate-1 suite
      (`test_native_step_reader`: TOROIDAL_SURFACE, two-root assembly, non-mm unit,
      malformed record, empty → NULL). The sim harness's engine hook still routes a NULL
      parse to OCCT (native_engine.cpp), so a declined file is imported identically to
      `cc_set_engine(0)`; a dedicated sim decline case was left to the host suite (no OCCT
      needed to prove the decision). `cc_iges_*`/`step_export` untouched (grep-verified).
- [x] 8.5 Own `main()`, on the `run-sim-suite.sh` SKIP list; suite assertion count unchanged;
      default engine restored in teardown.

## 9. No-regression + NUMSCI + docs + validation

- [x] 9.1 Prove no regression: export slice, healing, SSI S1–S4, S5 native-pass=6, native
      blends + #6/#7, marching, boolean, construct, tessellation, phase3 all green
      (host CTest + `run-sim-suite.sh`).
- [x] 9.2 NUMSCI ON build proves no interaction: `bash scripts/build-numsci.sh host`;
      configure `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON` + the NUMSCI/NUMPP/SCIPP dirs;
      build + ctest green.
- [x] 9.3 `openspec validate add-native-step-import --strict` green.
- [x] 9.4 Measure cognitive complexity of `src/native/exchange/` (`cognitive-complexity`
      skill): the tokenizer value-dispatch documented in the systems band (≤ ~25), all
      mapper resolvers 🟢/acceptable.
- [x] 9.5 Update `openspec/NATIVE-REWRITE.md` #7 + `docs/STATUS-phase-4.md`: STEP import
      native for the writer-emitted AP203 subset (Gate 1 host round-trip green; sim-vs-OCCT
      + foreign STEP Gate 2 green); arbitrary / AP242 / IGES import stay OCCT; #8 `drop-occt`
      stays blocked. Living-spec sync/archive when both gates are green.
