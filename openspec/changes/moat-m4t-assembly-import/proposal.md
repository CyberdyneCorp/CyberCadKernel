# Proposal — moat-m4t-assembly-import (MOAT M4 import tail: MAPPED_ITEM / REPRESENTATION_MAP)

## Why

The native STEP reader (`step_import_native`) already admits AP203/AP214/AP242 solids,
B-spline surfaces + curves (rational, trimmed), full-periodic sphere/torus, revolved
profiles, and **Form-A** assemblies (`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` →
`REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` / `ITEM_DEFINED_TRANSFORMATION`, single-
and multi-level nested, rigid / uniform-scale / mirror). The one product-structure gap that
remains a hard **decline → OCCT** is the STANDARD AP242 assembly-reuse mechanism:
**`MAPPED_ITEM` / `REPRESENTATION_MAP`** (ISO 10303-42 "Form-B"). A foreign system that
instances ONE shared representation many times (a fastener library, a repeated feature)
emits one `REPRESENTATION_MAP` over the shared `SHAPE_REPRESENTATION` and one `MAPPED_ITEM`
per instance, each carrying its own target `AXIS2_PLACEMENT_3D`. Today every such file falls
through to `STEPControl_Reader`.

The placement math is IDENTICAL to the Form-A path already landed:
`T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹`, classified conformal by the SAME
`classifyPlacement` gate, applied through the SAME `Shape::located(math::Transform)`
substrate (M-TX), with the SAME mirror orientation-compensation. This change adds the
Form-B dispatch as a bounded, additive slice: resolve each `MAPPED_ITEM`'s shared brep
through its `REPRESENTATION_MAP`, map that brep ONCE (memoized), and emit one located
instance per `MAPPED_ITEM` as a placed `Compound`. PMI *semantics* remain a measured
decline (the existing `pmi_scan` census is unchanged; GD&T semantic modelling is out of
scope and never fabricated).

## What Changes

- **Form-B `MAPPED_ITEM` / `REPRESENTATION_MAP` admission.** `assemblyDisposition()` gains a
  `MappedItem` verdict: a `MAPPED_ITEM` whose `REPRESENTATION_MAP` reaches exactly one
  `MANIFOLD_SOLID_BREP` dispatches to a new `mappedAssembly()` instead of declining. Each
  `MAPPED_ITEM(name, #repMap, #target)` resolves the shared brep from
  `#repMap = REPRESENTATION_MAP(#origin, #mappedRep)` (with `#mappedRep` a
  `SHAPE_REPRESENTATION` listing the brep), computes the instance transform
  `T = frameToWorld(target) ∘ frameToWorld(origin)⁻¹`, and locates the shared solid at `T`.
  The `#target` may be an `AXIS2_PLACEMENT_3D` (rigid) OR a
  `CARTESIAN_TRANSFORMATION_OPERATOR_3D` (uniform-scale / mirror — resolved via the existing
  `cartesianOperator`); `classifyPlacement` gates conformality; a mirror complements the
  instance's faces. The shared brep is mapped ONCE and re-instanced through the shared node.
- **Honest decline preserved and sharpened.** A `MAPPED_ITEM` whose `REPRESENTATION_MAP` /
  origin / target is unreadable, whose mapped representation contains ≠ 1 brep, or whose
  composed transform is non-conformal (shear / non-uniform scale / singular), declines the
  WHOLE file → OCCT (never a mis-placed or partial assembly). A file that mixes Form-A
  (CDSR-reaching-brep) AND Form-B is out of this bounded slice → decline. A lone
  `REPRESENTATION_MAP` with no `MAPPED_ITEM` instancing it → decline.
- **PMI semantics stay a decline.** No GD&T / tolerance-zone / feature-control-frame model is
  built; `pmi_scan` (census only) is byte-unchanged. The measured reason is recorded.

## Impact

- Affected specs: `native-exchange` (import requirement + scenarios — MODIFIED delta).
- Affected code: `src/native/exchange/step_reader.{h,cpp}` (additive Form-B path;
  `mapManifoldBrep` / leaf-geometry mappers and the writer are byte-frozen). No `cc_*` ABI
  change; `native_engine.cpp` `step_import` dispatch unchanged (still native-else-OCCT,
  self-verified). `src/native/**` stays OCCT-free.
- Tests: `tests/native/test_native_step_reader.cpp` (host-analytic MAPPED_ITEM instancing +
  decline cases) and `tests/sim/native_step_mapped_item_parity.mm` (SIM parity vs OCCT
  `STEPControl_Reader`).
