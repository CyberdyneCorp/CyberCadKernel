# Proposal — moat-m4t-nested-assemblies

## Why

The native STEP import reader composes a **single-level** placed assembly: OCCT's
`STEPControl_Writer` emits, per placed component, a `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` (CDSR)
→ `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` + `ITEM_DEFINED_TRANSFORMATION` (IDT) whose
`AXIS2_PLACEMENT_3D` FROM/TO pair yields one rigid / uniform-scale / mirror transform, and the reader
places each leaf `MANIFOLD_SOLID_BREP` by the ONE CDSR whose child shape-representation *directly*
reaches it (`Mapper::assembly()` in `src/native/exchange/step_reader.cpp`). That is correct for a flat
placed product but it has **no notion of an ancestor placement**.

A **nested** assembly — a component that is ITSELF an assembly (`STEPCAFControl_Writer` on a
multi-level XCAF document) — expresses the tree as a CHAIN of CDSR relationships: the leaf part's
shape-representation is placed into a SUB-assembly shape-representation by `T₂`, and that sub-assembly
is placed into the ROOT shape-representation by `T₁`. The leaf's true world placement is the
composition `W = T₁ ∘ T₂`. The landed reader, walking only the single CDSR that directly reaches the
leaf, applies `T₂` alone — the intermediate CDSR (whose child representation reaches no brep directly)
is currently SKIPPED as a PMI/annotation relationship. So a genuine 2-level tree is **mis-placed**
(the leaf lands at its sub-assembly-local position, missing `T₁`); the landed spec's intent is that
deep-nested files DECLINE → OCCT, but the completeness gate does not reliably reject a leaf-only
2-level chain. Either way, nested assemblies are not correctly imported today.

This slice — MOAT **M4-tail**, FIRST slice — closes that with the SIMPLEST reachable case: a
**2-level nested rigid / conformal assembly**. It replaces the per-brep single-transform association
with a **relationship-chain walk**: for each leaf brep, find its owning shape-representation, walk the
CDSR parent edges to a UNIQUE root, and compose the per-level conformal transforms into ONE world
`Location`. The composed transform is classified ONCE by the LANDED `classifyPlacement` and applied by
the LANDED `Shape::located()` + mirror compensation. A single-level chain (length 1) composes to
EXACTLY today's placement, so the slice is a **strict superset** — the landed single-level path stays
byte-identical.

The pieces are already present and honest to compose:

- **The transforms are real and in the file.** Each level's `T` is read by the SAME landed helpers
  (`itemDefinedTransform`, `cartesianOperator`, `frameToWorld`, `resolveOperator`). Composing two
  conformal maps yields a conformal map, so `classifyPlacement` (Gram = k²·I; det sign) still gates
  rigid / uniform-scale / mirror on the COMPOSED `W`. We parse the placement the file carries; we
  invent none.
- **The native topology already models a placed sub-shape.** `Shape::located(Location)` composes a
  `math::Transform` onto a node and the accessors resolve world coordinates by baking the Location
  down the graph — one composed `W` per leaf tessellates + self-verifies watertight unchanged. **The
  tessellator is not touched.**
- **The engine self-verify already accepts a placed Compound.** `robustlyWatertightImport`
  per-member-verifies each solid; a nested placed leaf is still a rigidly / conformally located solid.

We hand OCCT anything the walk cannot resolve to a clean forest of conformal placements: a
`MAPPED_ITEM` / `REPRESENTATION_MAP` (Form-B — this slice extends the Form-A/CDSR path only), a
cyclic / ambiguous / dangling chain, or a non-conformal composed transform → NULL → OCCT
`STEPControl_Reader`. OCCT stays the ORACLE; an honest DECLINE is a first-class outcome.

## What changes

1. **Relationship-chain model (`step_reader.cpp`, new mapper members).** Build a parent-edge map
   `childSr → (parentSr, opId)` from every combined CDSR instance via the landed
   `relationshipAndTransform`. For each leaf `MANIFOLD_SOLID_BREP`, find its owning shape-representation
   (the SR whose `represented_items` list contains it) and **walk the parent edges to a unique root**,
   composing `W = T_root ∘ … ∘ T_leaf` (each `T` via the landed `resolveOperator`). Classify `W` once
   via the landed `classifyPlacement`; apply `Shape::located(Location{W})` + mirror compensation.
2. **`assembly()` generalized (`step_reader.cpp`).** Replace the current "place each brep by the ONE
   CDSR that directly reaches it" loop with the chain walk. A length-1 chain reproduces today's result
   exactly (proven byte-identical). The FLAT / single-solid paths and the disposition scan
   (`assemblyDisposition`) are unchanged; `MAPPED_ITEM` / `REPRESENTATION_MAP` still DECLINE.
3. **Robustness gates (`step_reader.cpp`).** The walk DECLINEs (NULL → OCCT) on: a cycle
   (visited-set guard); an ambiguous chain (a representation with two parent edges, or a leaf placed
   more than once); a dangling parent reference; a leaf whose chain does not terminate at a unique
   root; a non-conformal composed `W` (the landed classifier). No leaf is placed at a partial /
   identity location; no non-conformal transform is applied.
4. **Verification.** Host unit / analytic cases in `tests/native/test_native_step_reader.cpp` (the
   composed `W` vs an independent matrix multiplication; a 2-level nested buffer → placed centroids;
   single-level byte-identical; the decline cases). Sim parity in
   `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh`: a foreign
   `STEPCAFControl_Writer` 2-level nested assembly → native placed-compound vs OCCT re-import
   (count / per-solid volume / bbox / centroid); a `MAPPED_ITEM` / non-conformal / cyclic file →
   honest NULL → OCCT.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT; `src/native/**`
stays OCCT-free.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **`MAPPED_ITEM` / `REPRESENTATION_MAP` (Form-B) instances** — this slice extends the Form-A / CDSR
  chain only; a mapped-item instance still DECLINEs (the alternative first-slice target; deferred).
- **Non-conformal (non-uniform-scale / shear) composed transforms** — the landed classifier gates
  conformality on `W`; a sheared composition DECLINEs.
- **Cyclic / ambiguous / dangling relationship graphs** — a shared sub-assembly with more than one
  placement into the same tree, a cycle, or a missing reference DECLINEs (no partial import).
- **Out-of-slice component geometry** (`TOROIDAL_SURFACE` face that leaves a gap, etc.) inside any
  component → the whole assembly DECLINEs per the landed gates.
- **PMI / GD&T / colours / names, non-mm units, non-manifold / unhealable component B-reps** →
  DECLINE → OCCT (unchanged gates).
- **Inventing a placement or a solid**, weakening any tolerance, or committing dead code.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved kernel still
  block it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — new mapper members (parent-edge map, per-leaf chain walk +
  cycle / ambiguity / root gates); `assembly()` composes the chain instead of the single direct CDSR.
  OCCT-free, host-buildable. `step_writer.cpp` and the tessellator are NOT modified.
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport` already
  per-member-verifies a placed Compound). `iges_*` / `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — nested-composition analytic cases + decline cases; the
  single-level / flat / round-trip cases STILL pass (byte-identical).
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — the nested
  sim-vs-OCCT parity + the honest-decline fall-through. Own `main()`, on the `run-sim-suite.sh` SKIP
  list; default engine restored in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine stays
  OCCT. The prior import slices, STEP export, healing, SSI S1–S5, blends/#6/#7, GPU/Phase-3 do NOT
  regress.

## Verification

1. **HOST ANALYTIC gate (no OCCT).** For an OCCT-free nested buffer, each leaf's composed world
   `Location` equals an INDEPENDENT matrix multiplication of the file's frame-pair transforms, and
   each placed leaf's world centroid sits at that placement; a single-level buffer composes to the
   byte-identical landed placement; `MAPPED_ITEM` / cyclic / ambiguous / dangling / non-conformal
   buffers return NULL.
2. **SIM native-vs-OCCT gate (simulator, OCCT linked).** A foreign `STEPCAFControl_Writer` 2-level
   nested rigid assembly imports natively (engine 1) as a placed `Compound`; OCCT
   `STEPControl_Reader` re-imports the same file; the two agree on solid **count**, per-solid
   **volume** / **bbox** / **centroid**, and TOTAL volume within tolerance. A `MAPPED_ITEM` /
   non-conformal / cyclic file DECLINEs natively and imports via OCCT identical to `cc_set_engine(0)`.

Done only when both gates pass and every existing suite stays green at the OCCT default. Reported
honestly: this adds **2-level (depth-general) nested rigid / conformal assembly** import; `MAPPED_ITEM`
/ non-conformal / cyclic assemblies and general AP242 / IGES import remain OCCT and #8 `drop-occt`
stays blocked. If robust multi-level composition proves NOT reachable as a bounded slice (the nested
CDSR linkage OCCT authors is not the clean parent-forest this design assumes), the change DECLINEs
honestly with the measured gap (the diagnosed graph shape) rather than shipping a mis-composed import.
