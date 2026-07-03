# Tasks — add-native-data-exchange (Phase 4 #7 slice — native STEP export)

Order: ISO-10303-21 text formatting → Part-42 entity emitters + `#n` id/dedup → topology
walk + representability gate → header/units/context wrapper → native writer API →
engine native-else-fallback wiring + OCCT-read round-trip self-check → Gate 1 (host
structural) → Gate 2 (sim native-write / OCCT-read round-trip) → docs. Native code stays
OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*`
ABI change. Default engine stays OCCT. STEP import + IGES export/import stay OCCT.

IMPLEMENTATION NOTE: the module is factored as `step_writer.h/.cpp` (text formatting +
Part-42 emitters + topology walk, all in one TU) + the `native_exchange.h` umbrella,
rather than the separate `step_text.h` / `step_entities.h` files the original plan
sketched — the same responsibilities, one cohesive OCCT-free translation unit. All
writer functions came out 🟢 Excellent (≤ 7 cognitive complexity), so the systems-band
walk the plan anticipated did not materialise (the walk is split into small
vertex/curve/edge/loop/surface/face methods).

## 1. ISO-10303-21 text formatting (`step_writer.cpp`, OCCT-free)

- [x] 1.1 Real-number formatting to ISO-10303-21 lexical rules (a decimal point always
      present, full fp64 precision via `%.17g`, no locale dependence); string, enum
      (`.T.` / `.F.` / `.UNSPECIFIED.`), and list `( ... )` formatting.
- [x] 1.2 An entity-record writer (`DataWriter::emit`): `#n = KEYWORD(args);` with a
      monotonic `#n` allocator.
- [x] 1.3 Host-buildable, includes only STL + `src/native/{topology,math}`; cognitive
      complexity 🟢.

## 2. Part-42 entity emitters + id dedup (`step_writer.cpp`, OCCT-free)

- [x] 2.1 Leaf emitters with value-dedup: `CARTESIAN_POINT`, `DIRECTION`,
      `AXIS2_PLACEMENT_3D` (from `math::Ax3`), `VERTEX_POINT`.
- [x] 2.2 Curve emitters: `LINE` (+`VECTOR`), `CIRCLE`, `B_SPLINE_CURVE_WITH_KNOTS`
      (degree, poles, knots+multiplicities via RLE-compress of the flat knot vector).
      Rational (weighted) spline is out of scope → DECLINE (canSerialize false).
- [x] 2.3 Surface emitters: `PLANE`, `CYLINDRICAL_SURFACE`, `CONICAL_SURFACE`,
      `SPHERICAL_SURFACE`, `B_SPLINE_SURFACE_WITH_KNOTS`. Rational surface out of scope.
- [x] 2.4 Topology emitters: `EDGE_CURVE`, `ORIENTED_EDGE`, `EDGE_LOOP`,
      `FACE_OUTER_BOUND` / `FACE_BOUND`, `ADVANCED_FACE`, `CLOSED_SHELL`,
      `MANIFOLD_SOLID_BREP`.
- [x] 2.5 A dedup map so shared leaves emit ONE record: `CARTESIAN_POINT` / `DIRECTION`
      value-deduped; `VERTEX_POINT` deduped by its `CARTESIAN_POINT` id; `EDGE_CURVE`
      deduped GEOMETRICALLY (unordered endpoint-vertex pair + curve signature) so the two
      adjacent faces of a physical edge — which carry DISTINCT edge nodes because the
      native builders defer edge-node sharing (#4) — share ONE `EDGE_CURVE`, welding a
      manifold `CLOSED_SHELL`. Each emitter ≤ 7 cognitive complexity.

## 3. Topology walk + representability gate (`step_writer.h`, OCCT-free)

- [x] 3.1 Representability gate (`canSerialize`): accept ONLY a native `Solid`/`Shell`
      (or Compound/CompSolid) whose faces are `Plane`/`Cylinder`/`Cone`/`Sphere`/`BSpline`
      and whose edges are `Line`/`Circle`/`BSpline` (knotted, non-rational); else DECLINE.
- [x] 3.2 Bottom-up walk (points/directions → placements → curves/surfaces →
      vertices/edges → loops/bounds → faces → shell → solid), assigning `#n` ids so every
      reference is defined before use (no dangling forward reference — asserted by the
      host test's contiguous-ascending-id check).
- [x] 3.3 Map native `Orientation` → `ORIENTED_EDGE` / `ADVANCED_FACE` sense flags;
      native face outer wire (child 0) → `FACE_OUTER_BOUND`, holes → `FACE_BOUND`.
- [x] 3.4 Read geometry via `src/native/topology` accessors (`curveOf` / `surfaceOf` /
      `pointOf`), coordinates in true mm, world-placed via each shape's `Location`. The
      walk stayed 🟢 (split into small methods), not the anticipated systems-band function.

## 4. Header + units + AP203 context wrapper (`step_writer.cpp`)

- [x] 4.1 Emit the ISO-10303-21 `HEADER` (`FILE_DESCRIPTION`, `FILE_NAME`, `FILE_SCHEMA`
      naming the AP203 `CONFIG_CONTROL_DESIGN` schema).
- [x] 4.2 Emit the mm unit context: `SI_UNIT(.MILLI.,.METRE.)` length + radian plane-angle
      + steradian solid-angle, `UNCERTAINTY_MEASURE_WITH_UNIT`, and the combined
      `GEOMETRIC_REPRESENTATION_CONTEXT` / `GLOBAL_UNIT_ASSIGNED_CONTEXT`.
- [x] 4.3 Emit the AP203 product spine (`APPLICATION_CONTEXT`,
      `APPLICATION_PROTOCOL_DEFINITION`, `PRODUCT`, `PRODUCT_DEFINITION_FORMATION_WITH_
      SPECIFIED_SOURCE`, `PRODUCT_DEFINITION`, `PRODUCT_DEFINITION_SHAPE`,
      `SHAPE_DEFINITION_REPRESENTATION`, `PRODUCT_RELATED_PRODUCT_CATEGORY`) and the
      `ADVANCED_BREP_SHAPE_REPRESENTATION` binding `MANIFOLD_SOLID_BREP` to the context.

## 5. Native writer API (`native_exchange.h`, OCCT-free umbrella)

- [x] 5.1 Entries `step_export_native(solid, path)` (→ bool) + `step_can_export_native`
      (→ bool) + `writeStepString(solid)` (→ buffer, for tests). No OCCT / engine types.
- [x] 5.2 Confirmed the whole `src/native/exchange/` subtree compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic` with NO
      OCCT and NO simulator (clean, no warnings).

## 6. Engine wiring + OCCT-read round-trip self-check (`src/engine/native/native_engine.cpp`)

- [x] 6.1 `step_export`: if `isNative(body)` and `step_can_export_native`, call
      `step_export_native` on the native `topology::Shape`, writing `path`; returns success.
- [x] 6.2 Mandatory OCCT-read round-trip self-check (behind `CYBERCAD_HAS_OCCT`): re-read
      the written file with `STEPControl_Reader`, require a valid single-solid shape,
      compare volume / bbox / sub-shape counts vs the source. Realised at the sim gate
      (Gate 2) where OCCT is linked — the host has no OCCT to self-check against; entity
      arg orders were also cross-checked against the OCCT `RWStep*` writer modules. GREEN:
      native-written → OCCT re-read round-trips to the SAME solid — box vol relV 2.27e-16
      / area rel 1.89e-16 / centroidΔ 0 / bbox maxCornerΔ 1.00e-07, faces 6→6 edges 24→24;
      cylinder relV 1.27e-03 / area rel 5.97e-04 / bbox 1.00e-07, faces 9→9 edges 30→30;
      holed-plate relV 2.90e-04 / area rel 1.09e-04 / bbox 1.00e-07, faces 7→7, edges
      28→30 (within tol — sewn periodic-wall seam, matching OCCT's own writer). Writer
      parity native-vs-OCCT (both re-read): box/cylinder/holed-plate relV ≤ 4.70e-15,
      relA ≤ 6.48e-15, bboxΔ 0.
- [x] 6.3 On DECLINE / out-of-scope native body, DISCARD (no file) and return an honest
      error (never a faked file, never a native void handed to OCCT); a foreign
      (OCCT-built) body falls through to OCCT `STEPControl_Writer`.
- [x] 6.4 Return success (`Result<void>{}` → facade `1`) on native success, matching the
      `cc_step_export` contract.
- [x] 6.5 Left `step_import`, `iges_export`, `iges_import` UNCHANGED (still `fallback().*`,
      OCCT).

## 7. Gate 1 — host structural unit tests (no OCCT)

- [x] 7.1 `tests/native/test_native_step_writer.cpp`: a native box / cylinder emits a
      buffer with correct framing (`ISO-10303-21;` … `END-ISO-10303-21;`), an AP203
      `FILE_SCHEMA`, a millimetre `SI_UNIT`.
- [x] 7.2 Assert exactly one `MANIFOLD_SOLID_BREP` → one `CLOSED_SHELL`; an `ADVANCED_FACE`
      per native face with the right surface entity (box → 6 `PLANE`; cylinder →
      `CYLINDRICAL_SURFACE` + `PLANE` caps).
- [x] 7.3 Assert curve entities (`LINE` / `CIRCLE`) via `EDGE_CURVE` / `ORIENTED_EDGE` /
      `EDGE_LOOP` (box → 12 shared `EDGE_CURVE`, 24 `ORIENTED_EDGE`), and `VERTEX_POINT`
      (box → 8) / `CARTESIAN_POINT` / `DIRECTION` leaves.
- [x] 7.4 Assert every `#n` line is a well-formed contiguous ascending
      `#n = ENTITY(...);` (no dangling forward reference) and coordinates are STEP REALs.
- [x] 7.5 DECLINE cases → `canSerialize` false: a null shape, a bare vertex (non-solid).
- [x] 7.6 Facade case in `tests/test_native_engine.cpp`
      (`native_step_export_writes_valid_ap203_file`): native `cc_step_export` returns `1`
      for a representable native box and writes a file with the ISO magic +
      `MANIFOLD_SOLID_BREP` + mm `SI_UNIT`.
- [x] 7.7 Wired into host CTest (`test_native_step_writer`); all existing native suites
      green (host CTest 20/20).

## 8. Gate 2 — sim native-write / OCCT-read round-trip (the correctness gate)

- [x] 8.1 `tests/sim/native_step_parity.mm` + `scripts/run-sim-native-step.sh` through the
      `cc_*` facade under `cc_set_engine(1)`: export a native-built solid, re-read the file
      with OCCT `STEPControl_Reader`. GREEN — box / cylinder / holed-plate all re-read to a
      valid solid.
- [x] 8.2 Round-tripped shape matches the source native solid — volume / bbox / sub-shape
      counts / topology within tolerance. GREEN — box EXACT (vol 1000, 6 faces, 24 edges);
      cylinder vol rel 1.27e-3 / 9 faces; holed-plate vol rel 2.90e-4 / 7 faces; a sewn
      periodic wall gains one seam edge the deferred-sharing native source omits (native 28
      → re-read 30, matching OCCT's own writer — accepted as a bounded superset). Two writer
      bugs fixed to reach this: EDGE_LOOP/ADVANCED_FACE stray `'',''` arg (rejected → empty
      solid) and the missing full-turn-wall SEAM edge (read back zero-area → leaky solid);
      both pinned by host regression tests.
- [x] 8.3 Fall-through cases GREEN — a foreign / OCCT-built solid forwards to
      `STEPControl_Writer` (writer-parity native vs OCCT rel ≤ 4.7e-15); an out-of-scope
      native solid returns a clean error, never a native void handed to OCCT.
- [x] 8.4 `cc_step_import`, `cc_iges_export`, `cc_iges_import` UNCHANGED (still OCCT) under
      both engine settings.
- [x] 8.5 Own `main()`, on the `run-sim-suite.sh` SKIP list; 221-assertion count unchanged
      (`run-sim-suite.sh` re-verified 221/221).

## 9. Docs + validation

- [x] 9.1 `openspec validate add-native-data-exchange --strict` green.
- [x] 9.2 Measured cognitive complexity of `src/native/exchange/` (`cognitive-complexity`
      skill): all functions 🟢 Excellent (worst `geometrySupported` 7), NO systems-band
      function.
- [x] 9.3 Updated `openspec/NATIVE-REWRITE.md` #7 status + `docs/STATUS-phase-4.md`:
      STEP export native (Gate 1 green; OCCT round-trip is Gate 2 on the sim); STEP import
      + IGES stay OCCT; #8 `drop-occt` stays blocked. Living-spec sync/archive when both
      gates are green.
