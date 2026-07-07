# Tasks — moat-m4t-nested-assemblies (MOAT M4-tail, first slice)

Order: DIAGNOSE the real nested CDSR graph → additive chain-walk composition → robustness /
decline gates → HOST ANALYTIC gate → SIM native-vs-OCCT gate → zero-regression proof → docs, or
HONEST DECLINE. All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`),
namespace `cybercad::native::exchange`. No `cc_*` ABI change. OCCT stays the ORACLE and the honest
fallback; a correct DECLINE (`MAPPED_ITEM` / non-conformal / cyclic → OCCT) is a first-class outcome.
No tolerance is weakened; no dead code is committed.

## 0. Substrate

- [ ] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`; export
  `CYBERCAD_NUMSCI_DIR=$PWD/build-numsci/iossim` (or `/host`) before building / running.
- [ ] 0.2 Capture the baseline: host CTest + `scripts/run-sim-suite.sh` (221/221) + the STEP-import
  parity suite GREEN on this worktree BEFORE any change (the regression baseline).

## 1. DIAGNOSE the nested CDSR graph (gate the slice)

- [ ] 1.1 On the simulator, author a 2-level nested rigid assembly with `STEPCAFControl_Writer`
  (leaf part in a sub-assembly, sub-assembly in the top assembly, distinct rigid transforms). Dump
  the DATA graph: every `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`,
  `REPRESENTATION_RELATIONSHIP[_WITH_TRANSFORMATION]`, `ITEM_DEFINED_TRANSFORMATION`,
  `SHAPE_REPRESENTATION`, and `NEXT_ASSEMBLY_USAGE_OCCURRENCE`.
- [ ] 1.2 Confirm the linkage this design assumes: each placing CDSR carries `rep_1 = childSr`,
  `rep_2 = parentSr`, and the leaf brep sits in the deepest child SR's `represented_items`. Confirm
  the graph is a clean parent-forest (each `childSr` has ≤ 1 parent; the leaf reaches a unique root).
- [ ] 1.3 **DECISION POINT.** If the linkage is a clean conformal parent-forest → proceed. If it is
  ambiguous / shared / routed differently than `REPRESENTATION_RELATIONSHIP.rep_2` → record the
  measured graph shape and DECLINE the slice honestly (the gap), rather than shipping a
  mis-composition.

## 2. Chain-walk composition (`step_reader.cpp`, additive)

- [ ] 2.1 Add `parentEdges()` — build `childSr → (parentSr, opId)` from every combined CDSR instance
  (extend / sibling `relationshipAndTransform` to also read `REPRESENTATION_RELATIONSHIP.rep_2`).
  DECLINE if any `childSr` has two distinct parent edges (ambiguous / shared sub-assembly).
- [ ] 2.2 Add `ownerSr(brep)` — the shape-representation whose `represented_items` contains `brep`;
  DECLINE if ≠ 1 owner.
- [ ] 2.3 Add `composeChain(leafBrep)` — walk `ownerSr(leafBrep)` up `parentEdges` to a UNIQUE root,
  composing `W = T_root ∘ … ∘ T_leaf` (each `T` via the landed `resolveOperator`; visited-set cycle
  guard). Return `nullopt` (→ DECLINE) on a cycle, a dangling edge, or a non-terminating chain.
- [ ] 2.4 Refactor `assembly()` to place each leaf by `composeChain(leaf)` and
  `classifyPlacement(W)` (rigid / uniform-scale / mirror; else DECLINE), applying
  `located(Location{W})` + `reversedShape()` for a mirror. A **length-1 chain SHALL reproduce the
  landed single-level placement exactly** — PROVE byte-identical (§5.1). Keep `assemblyDisposition`,
  the flat / single-solid paths, and the `MAPPED_ITEM` / `REPRESENTATION_MAP` DECLINE unchanged.

## 3. Robustness / decline gates

- [ ] 3.1 Cycle → DECLINE (visited-set). Ambiguous (two parents for one `childSr`, or a leaf reached
  twice) → DECLINE. Dangling / missing parent reference → DECLINE. Non-conformal composed `W`
  (`classifyPlacement` nullopt) → DECLINE. No leaf placed at a partial / identity location.
- [ ] 3.2 Confirm `src/native/**` has ZERO OCCT includes/symbols after the change
  (`grep -rE 'occt|OCCT|TopoDS|BRep|STEPControl' src/native/ | wc -l` → 0 new).

## 4. HOST ANALYTIC gate (no OCCT) — `tests/native/test_native_step_reader.cpp`

- [ ] 4.1 Build an OCCT-free 2-level nested buffer with KNOWN `T₁`, `T₂`; assert the leaf's composed
  world `Location` equals an INDEPENDENT `Mat3`/`Vec3` multiply of the two frame-pair transforms, and
  the placed leaf's world centroid equals `W · centroid_local`.
- [ ] 4.2 Assert a SINGLE-level buffer composes to the byte-identical landed placement.
- [ ] 4.3 Assert DECLINE (NULL) for: a `MAPPED_ITEM` buffer, a cyclic graph, an ambiguous
  (two-parent) graph, a dangling reference, and a non-conformal composed transform.

## 5. SIM native-vs-OCCT gate — `tests/sim/native_step_import_parity.mm` + `run-sim-native-step-import.sh`

- [ ] 5.1 Foreign `STEPCAFControl_Writer` 2-level nested rigid assembly → native `cc_step_import`
  (engine 1) placed Compound vs OCCT `STEPControl_Reader` re-import: equal solid COUNT, per-solid
  VOLUME, per-solid BBOX, per-solid CENTROID, TOTAL volume within tolerance.
- [ ] 5.2 A `MAPPED_ITEM` / non-conformal / cyclic file → native NULL (DECLINE) → OCCT import
  identical to `cc_set_engine(0)` (honest fall-through, no fabricated placement).
- [ ] 5.3 The parity test restores the OCCT default in teardown and carries its own `main()` (on the
  `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

## 6. Zero-regression proof

- [ ] 6.1 Host CTest + `scripts/run-sim-suite.sh` (221/221) + GPU / Phase-3 GREEN at the OCCT default;
  DIFF vs the §0.2 baseline (no delta except the additive nested cases).
- [ ] 6.2 The landed single-level / scaled / mirrored / flat / round-trip import assertions unchanged
  (the walk is a strict superset).
- [ ] 6.3 Cognitive complexity of the new / refactored mapper helpers measured with the
  cognitive-complexity skill and within the parsers band (≤ 25–35); flagged if irreducible.

## 7. Docs / honesty

- [ ] 7.1 Update `step_reader.h` doc-comment: the reader composes single- OR multi-level (nested)
  rigid / uniform-scale / mirror assemblies via the CDSR relationship chain; `MAPPED_ITEM` /
  non-conformal / cyclic / ambiguous DECLINE → OCCT.
- [ ] 7.2 `openspec validate moat-m4t-nested-assemblies --strict` passes; update the MOAT roadmap
  M4-tail line (nested landed; `MAPPED_ITEM` remains the deferred Form-B tail). Report honestly what
  landed vs what stays OCCT; `drop-occt` stays blocked.
