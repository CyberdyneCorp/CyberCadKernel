# Proposal — add-native-step-assemblies

## Why

The native STEP import reader (`add-native-step-import` → `widen-native-step-import`, both
archived) tokenizes an ISO-10303-21 (Part-21) file and maps the in-slice B-rep subset to a
native `topology::Shape`: a `Solid` for one root `MANIFOLD_SOLID_BREP`, or a **flat**
`Compound` for several co-equal roots at world coordinates. But it **DECLINES the moment it
sees an assembly transform tree**: `Mapper::hasNestedAssembly()` returns true whenever any of
`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `MAPPED_ITEM`, `REPRESENTATION_RELATIONSHIP[_WITH_TRANSFORMATION]`,
`SHAPE_REPRESENTATION_RELATIONSHIP`, `ITEM_DEFINED_TRANSFORMATION`, or
`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` appears — the whole file returns NULL → OCCT
(`decline_transformed_assembly_returns_null` pins this). That is correct-but-narrow: it never
mis-places a solid, but it hands every real CAD assembly (a product placed by a transform
tree) straight to OCCT.

This slice **widens native import to a TRANSFORMED ASSEMBLY of in-slice solids**. The pieces
are already present and honest to compose:

- **The transform is real and in the file.** A STEP assembly places each component
  representation into its parent by a **rigid** `ITEM_DEFINED_TRANSFORMATION` — an ordered
  `AXIS2_PLACEMENT_3D` pair (a "from" datum and a "to" datum) whose delta is a
  rotation + translation — carried by a `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`
  (linked to the instance via `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` +
  `NEXT_ASSEMBLY_USAGE_OCCURRENCE`), and/or by a `MAPPED_ITEM` referencing a
  `REPRESENTATION_MAP` (a `mapping_origin` `AXIS2_PLACEMENT_3D` + a mapped representation). We
  parse the placement **that is written**; we do not invent one.

- **The native topology already models a placed sub-shape.** `topology::Location` wraps a
  `math::Transform{Mat3 linear, Vec3 translation}` (an exact rigid map when `linear` is a
  rotation), `Shape::located(Location)` composes it onto a shared node, and `accessors.h`
  resolves world coordinates by baking the composed Location down the graph — exactly
  `TopLoc_Location` semantics. So a placed component solid tessellates and self-verifies
  watertight **transparently** (a rotation/translation preserves the watertight 2-manifold);
  **the tessellator is not touched**.

- **The engine self-verify already accepts a Compound.** `robustlyWatertightImport`
  (`native_engine.cpp`) explores a `Compound`'s solids and requires EACH `robustlyWatertight`.
  A placed solid is still a solid; a leaky/mis-composed member fails the check → OCCT
  fall-through. No new engine gate is needed beyond calling the existing one.

- **AP214 / AP242 headers already parse.** The reader NEVER gated on the `FILE_SCHEMA`
  header string — it enters at `DATA;` and gates on entities + the mm unit context. So an
  `AUTOMOTIVE_DESIGN` (AP214) / `AP242` file whose entities are all in slice already imports.
  This change **confirms and regression-pins** that (adds a header assertion + a foreign-AP214
  fixture), rather than claiming a new capability that isn't there.

So the slice **replaces the blanket transform-tree decline with a placement composition**:
parse the assembly structure, compose each component's rigid `Location`, apply it to the
component's mapped `MANIFOLD_SOLID_BREP`(s), and build a **placed `Compound`** — DECLINING (→
OCCT) any file whose transform is non-rigid/scaled/mirrored, whose structure the reader cannot
compose, whose component geometry is out of the import slice, or whose placed solid fails the
watertight self-verify. It is **not** a general product-structure importer (deep multi-level
nesting, external part refs, and PMI stay OCCT); it is a bounded widening covering the
single-level rigid assemblies OCCT authors for a placed multi-body product.

This does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved
kernel still block it). It is an additive breadth widening of the working import slice.

## What changes

1. **Assembly-structure parse (`step_reader.cpp`, new mapper members).** Add an
   `assembly()` pass that, when a transform tree is present, resolves the component
   placements instead of declining:
   - **`itemDefinedTransform(id)`** — read `ITEM_DEFINED_TRANSFORMATION('',desc,#from,#to)`
     as the ordered `AXIS2_PLACEMENT_3D` pair `(#from, #to)`; the rigid placement is the map
     that takes the `#from` frame onto the `#to` frame: `T = Ax3(to) ∘ Ax3(from)⁻¹`, built
     from the frames' orthonormal rotation `Mat3` + origin translation (both frames are
     `math::Ax3` via the existing `axis2placement`). A frame that is not orthonormal, or a
     resulting `linear` part that is not a rotation (‖det − 1‖ > tol, or `Mat3·Mat3ᵀ ≠ I` — a
     scale/shear/mirror), → DECLINE.
   - **`representationMap(id)` / `mappedItem(id)`** — read `MAPPED_ITEM('',#map,#target)` +
     `REPRESENTATION_MAP(#origin,#rep)`: the placement is `Ax3(target) ∘ Ax3(origin)⁻¹`
     composed with the mapped representation's own transform (identity for a plain mapped
     solid). Same rigid-only gate.
   - **`nextAssemblyPlacements()`** — walk `NEXT_ASSEMBLY_USAGE_OCCURRENCE` /
     `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` / `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION`
     to associate each **component representation** (the `SHAPE_REPRESENTATION` /
     `ADVANCED_BREP_SHAPE_REPRESENTATION` carrying the component's `MANIFOLD_SOLID_BREP`s) with
     its composed rigid `Location`. The exact entity strings + linkage are confirmed against
     an OCCT-authored fixture Diagnose (task 1.1) before the arms are written.
2. **Placed compound in `build()` (`step_reader.cpp`).** When `hasNestedAssembly()` is true,
   instead of returning NULL, call the assembly pass. For each component representation:
   map its root `MANIFOLD_SOLID_BREP`(s) via the existing `mapManifoldBrep` (unchanged, builds
   the solid at its local coordinates), then `solid.located(Location{T})` with the composed
   rigid transform `T`, and collect the placed solids. Return
   `ShapeBuilder::makeCompound(placedSolids)`. If the assembly structure cannot be composed
   into rigid placements for every component (unknown linkage, missing frame, non-rigid
   transform, an unmapped component, zero components) → return NULL (DECLINE, never a partial
   or identity-placed import). The FLAT multi-solid path (no transform tree) and the
   single-solid path are **unchanged** (byte-for-byte).
3. **Rigid-only guard (`step_reader.cpp`, `math`).** A small `isRigid(const math::Transform&)`
   predicate (linear part orthonormal + det ≈ +1) gates every composed placement; a scaled /
   sheared / mirrored transform DECLINES → OCCT (the native `Location` would apply the linear
   part faithfully, but a non-rigid placement is out of the honest slice and could change a
   solid's volume, so it is declined rather than applied).
4. **AP214/AP242 header acceptance (confirm + pin).** No reader change (the reader never
   rejected a schema); add a host assertion that an `AUTOMOTIVE_DESIGN` / `AP242_...`
   `FILE_SCHEMA` header with in-slice entities imports, and a sim foreign-AP214 fixture. The
   doc-comment in `step_reader.h` is updated to state schema-independence explicitly.
5. **Engine hook + OCCT fallback (`native_engine.cpp`) — unchanged logic, wider input.**
   `step_import` still calls `step_import_native` then `robustlyWatertightImport` (which
   already per-member-verifies a Compound). A placed compound is a Compound of placed solids;
   each placed solid self-verifies watertight (a rigid transform preserves it). Any NULL
   parse or any leaky member → OCCT `STEPControl_Reader` re-reads the SAME file. `iges_*` /
   `step_export` untouched.
6. **healShell per placed solid.** As today, the shared-node reconstruction is watertight by
   construction and `readStepString` deliberately does NOT planarize via `healShell` (that
   would destroy curved faces). The `heal::healShell` deferral is unchanged; a placed solid
   that still leaves a gap fails the engine self-verify → OCCT. (No change to the healing
   policy; the spec restates it for the placed case.)
7. **Verification** — extend `scripts/run-sim-native-step-import.sh` +
   `tests/sim/native_step_import_parity.mm` with (A) an OCCT-authored 2-component **assembly**
   (two boxes at distinct rigid placements via the transform tree) → native placed-compound
   import vs OCCT re-import (count / total volume / per-solid bbox+placement within tol); (B) a
   3+-component or nested assembly if tractable, else documented; (C) an UNSUPPORTED assembly
   (a component out of slice, or a non-rigid/scaled transform) → honest NULL → OCCT; plus an
   AP214 foreign-header fixture. Host CTest gains the placement-composition unit cases +
   honest-decline cases; the `decline_transformed_assembly_returns_null` test is REPLACED by a
   placed-import test for the rigid case and RETAINED (renamed) for the non-rigid/out-of-slice
   case.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **Non-rigid / scaled / mirrored placement transforms** — a `Location` would apply the
  linear part, but a non-rigid instance is out of the honest slice (it can change a solid's
  volume / handedness); DECLINE → OCCT. Only rotation + translation is composed.
- **Deep (multi-level) nested product structure, external part references, shared
  sub-assemblies with their own transform sub-trees** — this slice composes the
  single-level component placements OCCT authors for a placed multi-body product; a
  genuinely recursive assembly the reader cannot fully compose → DECLINE → OCCT.
- **Out-of-slice component geometry** (`TOROIDAL_SURFACE`, `SURFACE_OF_REVOLUTION`,
  `TRIMMED_CURVE`, rational/weighted B-splines, `BEZIER`) inside any component → the whole
  assembly DECLINES → OCCT (no partial import).
- **PMI / GD&T / colours / names / validation properties** → DECLINE → OCCT.
- **Non-mm units, non-manifold / unhealable component B-reps** → DECLINE → OCCT (unchanged
  gates).
- **Inventing a placement or a solid** — only transforms/solids present in the file are
  composed; a structure that cannot be composed to rigid placements for every component
  DECLINES rather than defaulting a component to identity or fabricating geometry.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved
  kernel still block it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — new mapper members
  (`itemDefinedTransform`, `representationMap`/`mappedItem`, `nextAssemblyPlacements`,
  `assembly()`, `isRigid`) + `build()` calls the assembly pass instead of declining when a
  transform tree is present; the FLAT/single-solid paths are unchanged. `step_reader.h` /
  `native_exchange.h` — the `step_import_native` contract now documents placed assemblies
  (signature unchanged: still one `topo::Shape`, now possibly a placed `Compound`) + explicit
  schema-independence. OCCT-free, host-buildable. `step_writer.cpp` and the tessellator are
  NOT modified.
- `src/native/math/**` — a small `isRigid(Transform)` predicate (or inline in the reader) —
  no behavioural change to existing math.
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport`
  already per-member-verifies a Compound; a placed compound is a Compound). `iges_*` /
  `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — placement-composition unit cases (an
  `ITEM_DEFINED_TRANSFORMATION` frame pair → the expected rigid `Location`; a placed
  two-component buffer → a `Compound` whose two solids sit at the composed placements); a
  non-rigid/scaled transform → NULL; an out-of-slice component → NULL; an AP214/AP242 header
  with in-slice entities → imports; the FLAT multi-solid + single-solid + prior round-trips
  STILL pass.
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — the (A)
  assembly parity, (C) unsupported-assembly fall-through, and AP214 header cases. Own
  `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine
  stays OCCT. The prior import slices (flat multi-solid + bspline-face round-trip, sim
  `[NIMPORT]` 28/28), STEP export, healing, SSI S1–S5, blends/#6/#7, phase3 do NOT regress.

## Verification

1. **Host unit (OCCT-free).** An `ITEM_DEFINED_TRANSFORMATION` frame pair composes to the
   expected rigid `Location` (rotation `Mat3` + translation `Vec3` checked against the frame
   delta); a two-component transform-tree buffer imports as a `Compound` whose two solids sit
   at the composed placements (per-solid centroid at the expected world position); a non-rigid
   / scaled transform DECLINES (NULL); an out-of-slice component DECLINES (NULL); an
   AP214/AP242 header with in-slice entities imports; the FLAT/single-solid/round-trip cases
   are unchanged.
2. **Sim vs OCCT (simulator, OCCT linked).** OCCT `STEPControl_Writer` /
   `STEPCAFControl_Writer` authors a 2-component assembly (two boxes at distinct rigid
   placements via the transform tree); native `cc_step_import` (engine 1) returns a placed
   `Compound`; OCCT `STEPControl_Reader` re-imports the same file; the two agree on solid
   **count**, **total volume**, and per-solid **bbox + placement** within tolerance. An
   UNSUPPORTED assembly DECLINES natively and imports via OCCT identical to `cc_set_engine(0)`.
   A foreign AP214 file with in-slice entities imports natively and matches OCCT.

Done only when the relevant gates pass and every existing suite stays green at the OCCT
default. Reported honestly: this adds **rigid single-level assembly** import (placed compound)
+ **AP214/AP242 header acceptance** (confirmed, was already schema-independent); non-rigid /
deep-nested / out-of-slice / PMI assemblies stay OCCT; arbitrary / AP242-general / IGES import
remain OCCT and #8 `drop-occt` stays blocked.
