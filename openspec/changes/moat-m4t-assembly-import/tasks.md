# Tasks — moat-m4t-assembly-import

## 1. Reader: Form-B MAPPED_ITEM / REPRESENTATION_MAP dispatch

- [x] 1.1 `assemblyDisposition()`: add `MappedItem` verdict — a `MAPPED_ITEM` whose
  `REPRESENTATION_MAP` reaches exactly one `MANIFOLD_SOLID_BREP` returns `MappedItem`
  (else the existing Form-B `Decline`); a mixed Form-A + Form-B file declines.
- [x] 1.2 `representationMapBrep(#repMap)` helper: resolve
  `REPRESENTATION_MAP(#origin, #mappedRep)` → the single brep in `#mappedRep`'s item list
  (0 if none / >1 → ambiguous).
- [x] 1.3 `mappedItemPlacement(#mappedItem)` → `{Transform, mirror}` or nullopt:
  `T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹` for an `AXIS2_PLACEMENT_3D` target,
  or `cartesianOperator(target)` for a `CARTESIAN_TRANSFORMATION_OPERATOR_3D` target,
  classified by `classifyPlacement` (rigid / uniform-scale / mirror else decline).
- [x] 1.4 `mappedAssembly()`: for each `MAPPED_ITEM` (ascending #id, deterministic), resolve
  the shared brep + placement, map the shared brep ONCE (memoized by brep #id), locate it,
  complement faces on mirror, collect a placed `Compound`. Decline the whole file on any
  unreadable / non-conformal / ambiguous instance.
- [x] 1.5 `build()`: dispatch `AsmKind::MappedItem → mappedAssembly()`.
- [x] 1.6 Update the `step_reader.h` scope comment + the `assembly()` Form-B decline note.

## 2. Structural discipline

- [x] 2.1 `mapManifoldBrep` + all leaf-geometry mappers + `step_writer.*` byte-frozen
  (only `assemblyDisposition` / `build` / new Form-B members added).
- [x] 2.2 `src/native/**` OCCT-free; no `cc_*` ABI change; `native_engine.cpp` unchanged.

## 3. Gate (a) HOST-analytic (`tests/native/test_native_step_reader.cpp`, OCCT-free)

- [x] 3.1 A REPRESENTATION_MAP over a shared box instanced by N MAPPED_ITEMs at known
  translations → an N-solid Compound; each instance volume = box volume, each world bbox =
  box translated by its target frame (closed form).
- [x] 3.2 A rotated MAPPED_ITEM target composes exactly (rigid, det≈+1): the instance world
  bbox matches the rotated placement.
- [x] 3.3 Decline: a MAPPED_ITEM whose mapped representation carries no brep; a lone
  REPRESENTATION_MAP with no MAPPED_ITEM; a mixed Form-A + Form-B file → NULL.

## 4. Gate (b) SIM parity vs OCCT (`tests/sim/native_step_mapped_item_parity.mm` + runner)

- [x] 4.1 Read a MAPPED_ITEM-instanced STEP file natively AND through OCCT
  `STEPControl_Reader`; compare the set of solids (count + per-solid vol / area / centroid /
  bbox / face + edge counts) native-vs-OCCT within tolerance.
- [x] 4.2 New `.mm` with own `main()`, wired into `run-sim-suite.sh` as a SKIP-guarded entry.

## 5. Finalize

- [x] 5.1 `openspec validate moat-m4t-assembly-import --strict` passes.
- [x] 5.2 Update `openspec/MOAT-ROADMAP.md` M4 status + `openspec/DROP-OCCT-READINESS.md`
  `step_import` row.
- [x] 5.3 `scripts/build-numsci.sh host` + `iossim` exit 0; `ctest` green; structural check.
