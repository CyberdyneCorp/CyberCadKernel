# Tasks — add-native-step-assemblies (Phase 4 #7 — placed assembly import)

Widen the WORKING native STEP reader (`src/native/exchange/step_reader.{h,cpp}`) from a FLAT
multi-solid `Compound` to a PLACED `Compound`: parse the STEP assembly transform tree, compose
each component's RIGID placement into a native `topology::Location`, apply it per component
`MANIFOLD_SOLID_BREP` via `Shape::located()`, and confirm AP214/AP242 `FILE_SCHEMA` acceptance.
Map ONLY onto placements/geometry the file carries; otherwise DECLINE (NULL → OCCT). Native
code stays OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No
`cc_*` ABI change. Default engine stays OCCT. `step_writer.cpp` + the tessellator are NOT
modified. `iges_*` / `step_export` stay unchanged. NEVER invent a placement or a solid.

## 1. Confirm the exact OCCT-emitted assembly entities (grounding, before coding)

- [x] 1.1 Author a 2-box rigid assembly with OCCT (`STEPControl_Writer` on a `TopoDS_Compound`
      of two `BRepBuilderAPI_Transform`-placed boxes with `STEPControl_AsIs`, and/or
      `STEPCAFControl_Writer` on an XCAF assembly doc) and Diagnose the emitted DATA section.
      Record the EXACT keyword strings + arg orders for `NEXT_ASSEMBLY_USAGE_OCCURRENCE`,
      `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`, `REPRESENTATION_RELATIONSHIP` +
      `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`, `ITEM_DEFINED_TRANSFORMATION`, and/or
      `MAPPED_ITEM` + `REPRESENTATION_MAP` + `REPRESENTATION_MAP`'s child-rep link. Confirm which
      form OCCT emits for a flat placed multi-body compound (Form A §2.1 vs Form B §2.2).
- [x] 1.2 Confirm the linkage from each component representation to its
      `MANIFOLD_SOLID_BREP`(s) (the rep `items` list / `SHAPE_REPRESENTATION_RELATIONSHIP`
      chain) so `brepsOfRep` can resolve it structurally (by refs, not by name).

## 2. Rigid transform helpers (`step_reader.cpp` + `math`)

- [x] 2.1 `frameToWorld(const math::Ax3&) → math::Transform`: affine `{linear = [X|Y|Z]
      columns, translation = origin}` from an orthonormal `Ax3` (the reader already builds the
      `Ax3` via `axis2placement`).
- [x] 2.2 `isRigid(const math::Transform&)`: orthonormal linear part (`M·Mᵀ ≈ I`, tol 1e-9)
      AND det ≈ +1 (a proper rotation, not a mirror/scale/shear). A read-only predicate; NO
      change to existing `math` behaviour. Non-rigid → the caller DECLINES.

## 3. Assembly-structure parse (`step_reader.cpp`, new mapper members)

- [x] 3.1 `itemDefinedTransform(id) → optional<Transform>`: read
      `ITEM_DEFINED_TRANSFORMATION('',desc,#from,#to)`; resolve both `AXIS2_PLACEMENT_3D`
      frames via `axis2placement`; `T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`. Malformed /
      missing frame / singular inverse → `nullopt`.
- [x] 3.2 `mappedItem(id) / representationMap(id) → optional<(childRepId, Transform)>`: read
      `MAPPED_ITEM('',#map,#target)` + `REPRESENTATION_MAP(#origin,#childRep)`;
      `T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹`. Malformed → `nullopt`.
- [x] 3.3 `nextAssemblyPlacements() → map<int childRepId, Transform>`: walk the confirmed
      linkage (§2.1/§2.2 per 1.1) to associate each component representation with its composed
      rigid `Transform`. Structural resolution (refs only). Any unresolved link → set `fail_`.
- [x] 3.4 `brepsOfRep(repId) → vector<int>`: the root `MANIFOLD_SOLID_BREP` ids belonging to a
      component representation (its `items` / `SHAPE_REPRESENTATION_RELATIONSHIP` chain per 1.2),
      in ascending #id order (deterministic).

## 4. Placed compound in `build()` (`step_reader.cpp`)

- [x] 4.1 `assembly() → topo::Shape`: for each `(childRepId, T)` in `nextAssemblyPlacements()`
      (sorted by repId), require `isRigid(T)` (else return NULL); for each `brepsOfRep(childRepId)`
      map via the EXISTING `mapManifoldBrep` (unchanged, local coords) and push
      `solid.located(Location{T})`. Require every root brep placed exactly once
      (`placed.size() == findManifoldBreps().size()`, none unplaced/duplicated) else NULL. One
      placed component → return the placed Solid; ≥2 → `ShapeBuilder::makeCompound(placed)`.
      NEVER default a component to identity; NEVER partial-import.
- [x] 4.2 In `build()`: replace `if (hasNestedAssembly()) return {};` with
      `if (hasNestedAssembly()) return assembly();`. The FLAT multi-solid path and the
      single-solid path are UNCHANGED (byte-for-byte) — only a present transform tree takes the
      new branch (which previously returned NULL, so no accepting path regresses).

## 5. AP214 / AP242 schema-header acceptance (confirm + pin, no reader logic change)

- [x] 5.1 Confirm the reader never gates on `FILE_SCHEMA` (it enters at `DATA;`), so an AP214
      (`AUTOMOTIVE_DESIGN`) / AP242 header with in-slice entities already imports. NO logic
      change. Update `step_reader.h` doc-comment to state schema-independence explicitly (so a
      future reader does not add a schema gate).

## 6. Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`)

- [x] 6.1 `step_import_native` signature unchanged (returns one `topo::Shape`, now possibly a
      PLACED `Compound`). Update the doc-comment: placed single-level rigid assemblies +
      schema-independence + the still-declined set (non-rigid/scaled, deep-nested, out-of-slice
      component, PMI, non-mm).
- [x] 6.2 Confirm `src/native/exchange/` still compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO
      simulator (clean). Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 7. Engine hook + OCCT fallback (`src/engine/native/native_engine.cpp`) — logic unchanged

- [x] 7.1 Confirm `step_import` still calls `step_import_native` then `robustlyWatertightImport`
      (which already per-member-verifies a Compound) — a PLACED compound is a Compound of placed
      solids; each placed solid self-verifies watertight (a rigid transform preserves it). No
      new engine gate. Any NULL parse or leaky member → OCCT `STEPControl_Reader` re-reads the
      SAME file. `iges_*` / `step_export` untouched.

## 8. Gate 1 — host unit / decline `tests/native/test_native_step_reader.cpp`

- [x] 8.1 Placement composition: an `ITEM_DEFINED_TRANSFORMATION('',$,#from,#to)` with a known
      frame delta composes to the expected `Location` (rotation `Mat3` + translation `Vec3`
      checked against the delta).
- [x] 8.2 Placed compound: a two-component transform-tree buffer → `Compound` of two `Solid`s
      whose per-solid centroids sit at the composed WORLD placements (not the origin), count 2,
      each valid.
- [x] 8.3 Rigid gate: a scaled / mirrored placement DECLINES (NULL).
- [x] 8.4 Out-of-slice component (`TOROIDAL_SURFACE` in one component) DECLINES (NULL) — no
      partial import. Repurpose `decline_transformed_assembly_returns_null` to a NON-rigid /
      out-of-slice case (the rigid assembly now IMPORTS).
- [x] 8.5 Uncomposable structure (a root brep reached by no placement) DECLINES (NULL).
- [x] 8.6 Schema: an AP214 / AP242 `FILE_SCHEMA` header with in-slice entities imports.
- [x] 8.7 No regression: the FLAT multi-solid, single-solid, quadric, and bspline-face
      round-trip cases STILL pass. Wire into host CTest; all existing native suites green.

## 9. Gate 2 — sim vs OCCT + foreign assembly `tests/sim/native_step_import_parity.mm`

- [x] 9.1 Extend the harness + `scripts/run-sim-native-step-import.sh`; `xcrun simctl list
      devices booted` first; own `main()`, on the `run-sim-suite.sh` SKIP list; default engine
      restored in teardown (suite assertion count unchanged).
- [x] 9.2 (A) OCCT authors a 2-component rigid assembly (two boxes at distinct placements via
      the transform tree); native `cc_step_import` (engine 1) → placed `Compound`; OCCT
      `STEPControl_Reader` re-imports; assert same solid COUNT, same TOTAL volume (rel tol),
      per-solid bbox + centroid/placement within tol.
- [x] 9.3 (B) 3+-component / nested assembly if tractable from the authoring path; else document
      that (B) reduces to (A) repeated (honest).
- [x] 9.4 (C) unsupported assembly: OCCT authors an assembly with a component out of slice (or a
      scaled instance); native `cc_step_import` DECLINES → OCCT and matches `cc_set_engine(0)`.
- [x] 9.5 AP214 header: OCCT authors an AP214 file with in-slice entities; native import matches
      OCCT re-import.

## 10. No-regression + NUMSCI + complexity + docs + validation

- [x] 10.1 No regression: prior import slices (flat multi-solid + bspline-face round-trip, sim
      `[NIMPORT]` 28/28), STEP export, healing, SSI S1–S5, native blends + #6/#7, marching,
      boolean, construct, tessellation, phase3 — all green (host CTest + `run-sim-suite.sh`).
- [x] 10.2 NUMSCI ON build proves no interaction: `bash scripts/build-numsci.sh host`; configure
      `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON` + the NUMSCI/NUMPP/SCIPP dirs; build + ctest.
- [x] 10.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`itemDefinedTransform`, `mappedItem`/`representationMap`, `nextAssemblyPlacements`,
      `brepsOfRep`, `assembly`, `isRigid`, `frameToWorld`, `build`) all acceptable for the
      parser/systems band; none pushed to a higher band.
- [x] 10.4 `openspec validate add-native-step-assemblies --strict` green.
- [x] 10.5 Update `openspec/NATIVE-REWRITE.md` #7 + `docs/STATUS-phase-4.md`: native STEP import
      now covers rigid single-level PLACED assemblies (placed compound) + AP214/AP242 header
      acceptance; non-rigid / deep-nested / out-of-slice / PMI assemblies stay OCCT; #8
      `drop-occt` stays blocked. Living-spec sync/archive when the gates are green.
