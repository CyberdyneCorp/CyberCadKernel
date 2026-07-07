# Design — moat-m4t-nested-assemblies

## Context

`src/native/exchange/step_reader.cpp` (`Mapper`) reconstructs a native `topology::Shape` from an
ISO-10303-21 file with no OCCT. `Mapper::build()` runs `assemblyDisposition()`:

- **Compose** — a `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` (CDSR) whose child shape-representation
  reaches a `MANIFOLD_SOLID_BREP` → `assembly()`.
- **Decline** — a `MAPPED_ITEM` / `REPRESENTATION_MAP`, or a lone `NEXT_ASSEMBLY_USAGE_OCCURRENCE`
  with no composable placement → NULL → OCCT.
- **None** — flat multi-solid / single-solid / PMI-only → the flat path.

The landed `assembly()` (verbatim behaviour):

1. Seed every root `MANIFOLD_SOLID_BREP` at identity.
2. For each CDSR: `relationshipAndTransform(arg[0])` → `{childSr, opId}`; `brepOfRepresentation(childSr)`
   → the ONE brep directly in that child representation's `represented_items`; `resolveOperator(opId)`
   → `(T, mirrorFlag)` (rigid / uniform-scale / mirror via `classifyPlacement`, else decline). Record
   `placement[brep] = T`.
3. Completeness gate: `placedCount ≥ 1 && (rootCount − placedCount) ≤ 1` (the root component carries no
   CDSR and stays identity).
4. Map each brep by `mapManifoldBrep`, apply `located(Location{T})` + `reversedShape()` if mirrored.

The composition is keyed **per brep by the single CDSR that directly reaches it**. It has no concept of
a parent chain, so a nested tree (leaf placed in a sub-assembly placed in the root) applies only the
leaf's own `T₂`; the intermediate CDSR (`childSr` = the sub-assembly SR, which reaches no brep
directly) is skipped by the `brep == 0 → continue` arm. Result: a mis-placed leaf (or, when the gate
happens to reject, a decline). Either way, nested is not correctly imported.

## Goals / non-goals

- **Goal.** Correctly compose a **2-level (depth-general) nested rigid / conformal assembly** into a
  placed `Compound`, verified vs OCCT, with the landed single-level / flat / single-solid paths
  byte-identical.
- **Non-goal.** `MAPPED_ITEM` / `REPRESENTATION_MAP` (Form-B), non-conformal composed transforms,
  cyclic / ambiguous / dangling graphs, out-of-slice geometry, PMI, IGES, `drop-occt`. All DECLINE →
  OCCT.

## Decision — a leaf→root relationship-chain walk (Form A / CDSR)

Model the CDSR graph as **parent edges over shape-representations**, not per-brep placements:

```
parentEdge : childSr → (parentSr, opId)      // one entry per placing CDSR
```

built by iterating every combined CDSR instance: `relationshipAndTransform(arg[0])` gives
`{childSr, opId}`; `childSr` is `REPRESENTATION_RELATIONSHIP.rep_1` (arg[2]) and the parent is
`rep_2` (arg[3]). (The landed `relationshipAndTransform` already extracts `childSr` + `opId`; this
slice ALSO reads `rep_2` as the parent — a small additive extension of that helper, or a sibling that
returns the triple `{childSr, parentSr, opId}`.)

Per leaf `MANIFOLD_SOLID_BREP`:

1. `ownerSr(brep)` — the shape-representation whose `represented_items` contains `brep` (scan SRs via
   the landed `representationItems`). If none / more than one → DECLINE (ambiguous ownership).
2. **Walk** `sr = ownerSr(brep)` following `parentEdge[sr]`, composing
   `W ← resolveOperator(opId) ∘ W` (start `W = identity`; each level's `T` is applied on the LEFT so
   the root-most transform ends outermost: `W = T_root ∘ … ∘ T_leaf`). Maintain a **visited set**;
   revisiting an SR → DECLINE (cycle). A `sr` with NO `parentEdge` entry is the ROOT → stop.
3. Track a per-SR `mirror` parity by XOR-ing each level's `mirrorFlag`; but the AUTHORITATIVE
   classification is `classifyPlacement(W)` on the COMPOSED transform (product of conformal maps is
   conformal), which yields the single rigid / uniform-scale / mirror verdict and the final mirror
   flag. A non-conformal `W` → DECLINE.
4. Place: `mapManifoldBrep(brep).located(Location{W})`, then `reversedShape()` iff `W` is a mirror.

**Ambiguity gate.** Each `childSr` SHALL have AT MOST ONE `parentEdge` (a representation placed into
two parents = a shared sub-assembly instanced twice, which needs per-instance world transforms this
slice does not model) → DECLINE. Each leaf brep SHALL be owned by exactly one SR and reached by one
chain → DECLINE otherwise. No leaf is placed twice.

**Why this is a strict superset (single-level byte-identical).** In a single-level file each leaf's
owner SR is placed DIRECTLY into the root by one CDSR: the chain has length 1, `W = T` — exactly the
landed `placement[brep]`. A root leaf (owner SR = the root SR, no parent edge) stays identity — exactly
the landed unplaced root. A flat file (no CDSR) → every chain length 0 → identity → flat Compound. So
for every file the landed path handles, the walk yields the identical placed shape. This is asserted
directly (a single-level buffer through both code paths, or a snapshot vs `main`).

### Alternatives considered

- **MAPPED_ITEM first slice.** The task allows either. Rejected as the FIRST slice: Form-B is a wholly
  new parse path (`MAPPED_ITEM` + `REPRESENTATION_MAP` + `mapping_source/target` AXIS2 pair), whereas
  the nested Form-A walk REUSES every landed helper (`relationshipAndTransform`, `resolveOperator`,
  `frameToWorld`, `classifyPlacement`, `located`, mirror compensation) and only adds graph traversal.
  Nested is the smaller, lower-risk, higher-reuse slice. `MAPPED_ITEM` stays a documented DECLINE.
- **Recursively placing sub-assembly SRs.** Equivalent to the leaf-up walk but needs a placed-subtree
  cache; the leaf-up walk is simpler (one composition per leaf) and matches how the reader already
  enumerates leaf breps. A memoized `worldOf(sr)` (SR → composed `W`) makes it linear if depth grows;
  optional for the 2-level slice.

## Correctness gates (OCCT is the oracle; never removed)

- **(a) HOST ANALYTIC — no OCCT.** A host test builds a nested buffer with KNOWN `T₁`, `T₂` and asserts
  the leaf's composed world `Location` equals an INDEPENDENT `Mat3`/`Vec3` multiplication of the two
  frame-pair transforms, and the placed leaf's world centroid equals `W · centroid_local`. This proves
  the composition without any engine.
- **(b) SIM native-vs-OCCT.** On a booted simulator, `STEPCAFControl_Writer` authors a 2-level nested
  XCAF assembly; native `cc_step_import` (engine 1) returns a placed Compound; OCCT `STEPControl_Reader`
  re-imports the SAME file; assert equal solid COUNT, per-solid VOLUME, per-solid BBOX, per-solid
  CENTROID (hence TOTAL volume) within tolerance.

## Self-verify → OCCT fallback (honest decline)

`step_import_native` returns NULL — the engine falls through to OCCT `STEPControl_Reader` re-reading
the SAME file — whenever the walk cannot resolve a clean forest of conformal placements: a
`MAPPED_ITEM` / `REPRESENTATION_MAP`; a cycle (visited-set hit); an ambiguous chain (a `childSr` with
two parents, a brep owned by ≠1 SR, a leaf reached twice); a dangling parent reference; a leaf whose
chain does not terminate at a unique root; a non-conformal composed `W`; or any landed decline
(out-of-slice geometry, non-mm units, unhealable B-rep). The engine additionally self-verifies each
placed leaf watertight with the correct POSITIVE enclosed volume (a mirror whose normals point inward
fails → OCCT). **Never a mis-placed or fabricated solid.**

## Risks

- **The nested CDSR linkage OCCT authors may not be the clean parent-forest this design assumes**
  (e.g. `STEPCAFControl_Writer` may share one sub-assembly SR across instances, or route the parent
  through `NEXT_ASSEMBLY_USAGE_OCCURRENCE` rather than `REPRESENTATION_RELATIONSHIP.rep_2`). MITIGATION:
  Task 1 DIAGNOSES a real OCCT-authored 2-level file (dumps the CDSR / relationship graph) BEFORE the
  walk is written; if the linkage is ambiguous / shared, the slice DECLINEs honestly with the measured
  graph shape rather than shipping a mis-composition.
- **Composition order / handedness.** A left-vs-right multiply error mis-places the leaf. MITIGATION:
  gate (a) pins the exact `W = T₁ ∘ T₂` against an independent multiply; gate (b) pins it vs OCCT.
- **Single-level regression.** MITIGATION: the length-1 chain is asserted byte-identical (host) and the
  full existing suite stays green.
- **Depth generality vs bounded verification.** The walk is depth-general but only 2 levels are
  verified vs OCCT. MITIGATION: the cycle + unique-root gates make an unresolvable deeper chain DECLINE
  rather than mis-compose; deeper parity is a follow-up, not a silent claim.

## Complexity

The chain walk + gates are added as small mapper helpers (`parentEdges()`, `ownerSr()`,
`composeChain()`), each with a bounded loop + visited-set; target cognitive complexity within the
parsers/compilers band (≤ 25–35) per the project rule, measured with the cognitive-complexity skill and
kept green as `assembly()` is refactored.
