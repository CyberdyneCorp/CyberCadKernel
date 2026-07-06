# add-native-step-assemblies

A **bounded, honest breadth slice** of Phase 4 capability **#7 `native-exchange`**
(`openspec/NATIVE-REWRITE.md`), sitting on the working native STEP import reader
(`add-native-step-import` → `widen-native-step-import`, both archived). Those slices import a
**flat** multi-solid file (several co-equal root `MANIFOLD_SOLID_BREP` at world coordinates,
no transform tree) as a native `Compound`, but **DECLINE the moment they see an assembly
transform tree** (`hasNestedAssembly()` returns true → NULL → OCCT). A host test
`decline_transformed_assembly_returns_null` pins that decline today.

This change **widens native import to TRANSFORMED ASSEMBLIES**: it parses the STEP assembly
transform structure (`NEXT_ASSEMBLY_USAGE_OCCURRENCE` /
`(REPRESENTATION_RELATIONSHIP + REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION)` +
`ITEM_DEFINED_TRANSFORMATION`, and/or `MAPPED_ITEM` + `REPRESENTATION_MAP`), composes each
component's rigid placement (rotation + translation, from the `AXIS2_PLACEMENT_3D` pair) into
a native `topology::Location` (`math::Transform`), applies it per referenced
`MANIFOLD_SOLID_BREP` via `Shape::located()`, and returns a **PLACED `Compound`**. The native
topology already supports a placed sub-shape — a located node resolves world coordinates by
composing its `Location` down the graph (`accessors.h`), so the engine's existing per-member
`robustlyWatertightImport` self-verify accepts a placed solid unchanged and **no tessellator
change is needed**. It also **accepts AP214 / AP242 `FILE_SCHEMA` headers** (the reader never
gated on the schema string, so this is confirmed + regression-pinned, not newly added).

Honest scope: **rigid-transform assemblies of in-slice solids** only. A non-rigid / scaled /
mirrored transform, a component whose geometry is out of the import slice
(`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`, rational B-spline, …), a placement chain the
reader cannot compose, or any placed solid that fails the watertight self-verify → **NULL →
OCCT `STEPControl_Reader`**. No placement or solid the file did not describe is ever invented;
no tolerance is weakened. It does NOT change the `cc_*` ABI, the default engine (stays OCCT),
`step_writer.cpp`, or the tessellator, and does NOT unblock #8 `drop-occt` (a general
STEP/AP242 reader + IGES + a general-curved kernel still block it).

The correctness gate is **sim vs OCCT**: OCCT `STEPControl_Writer` (or `STEPCAFControl_Writer`)
authors a 2-component assembly (two boxes at distinct rigid placements via the transform
tree); native import returns a placed `Compound`; it is compared vs OCCT `STEPControl_Reader`
re-import — same solid count, same total volume, per-solid bbox/placement within tolerance.
Plus host unit cases (placement composition + honest declines) and a NUMSCI-ON no-interaction
build. Every prior suite stays green at the OCCT default.
