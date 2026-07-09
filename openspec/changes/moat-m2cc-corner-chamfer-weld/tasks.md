# Tasks — moat-m2cc-corner-chamfer-weld (MOAT M2 convex-corner chamfer weld)

Order: diagnose the sequential chamfer corner blocker → up-front-planes corner weld
verb → mandatory watertight + shrink self-verify → oracle-match scope (2-edge dihedral
corner matches OCCT; triple corner declines) → host analytic gate (closed-form volume +
declines) → additive engine candidate → sim native-vs-OCCT gate → byte-freeze + zero-
regression proof → OpenSpec. All new native code stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::blend`. No `cc_*` ABI change. The
change is strictly ADDITIVE: `chamfer_edges`, `fillet_edges`, `full_round`,
`fillet_face`, M0 `SolidMesher`, and every landed weld path stay BYTE-IDENTICAL. No
tolerance is weakened; a correct decline is a first-class outcome.

## STATUS

LANDED (both gates). `chamfer_corner` welds a 2-adjacent-edge convex corner (and a
single edge, and a non-orthogonal corner) watertight at the EXACT inclusion-exclusion
closed-form volume; the byte-frozen sequential `chamfer_edges` still DECLINES the same
adjacent set. The TRIPLE corner (≥3 edges at one vertex) DECLINES with
`TripleCornerOracleGap` because OCCT's `MakeChamfer` trims more there than the
half-space corner (measured on the sim: OCCT 985.667 vs 985.75). Routed through the
existing `cc_chamfer_edges` as an additive engine candidate (no ABI change).

## 1. Diagnose the blocker
- [x] Confirm sequential `chamfer_edges` returns NULL on `{x,y}` and `{x,y,z}` at a box
      corner (the shared corner lost after the first cut) and lands single / opposite
      pairs.

## 2. Corner weld verb (additive)
- [x] `corner_chamfer_weld.h` `chamfer_corner`: resolve every edge + chamfer plane up
      front on the ORIGINAL soup, then apply all clips; corner facet from the exposed
      rings + `assembleSolid` T-junction repair. Consumes `detail::chamferPlane` /
      `detail::applyCut` / `PlanarModel` byte-identical.
- [x] Mandatory self-verify: watertight AND `0 < V < V(original)`.
- [x] Triple-corner guard: DECLINE `TripleCornerOracleGap` when ≥3 picked edges share
      one vertex (OCCT-mismatch), matching the DIHEDRAL corner (≤2 per vertex) only.
- [x] Typed measured declines: BadInput / NonPlanarSolid / EdgeNotFound / NotConvexEdge
      / CutFailed / AssembleFailed / NotWatertight / VolumeInconsistent.

## 3. Engine wiring (additive, no ABI change)
- [x] `NativeEngine::chamfer_edges` tries `nblend::chamfer_corner` as candidate 1b after
      the sequential planar chamfer, gated by the same shrink self-verify.
- [x] `native_blend.h` includes the new header.

## 4. Gate A — host analytic (no OCCT)
- [x] `tests/native/test_native_corner_chamfer.cpp` (6/6): single-edge exact,
      2-adjacent exact, 2-edge volume sweep, triple-corner decline, non-orthogonal
      corner, and the out-of-domain declines. Registered in CMakeLists (always-on
      native suite).

## 5. Gate B — sim native-vs-OCCT
- [x] `native_blend_parity.mm` `chamfer-corner2` case: native == OCCT
      `BRepFilletAPI_MakeChamfer` to fp64 (990.333, watertight, native active).
- [x] `runTripleCornerGuard`: the 3-edge triple corner declines to id 0 under native.
- [x] `scripts/run-sim-native-blend.sh` gains `TKHLR` (pre-existing missing toolkit
      surfaced by relinking). 20 passed / 0 failed on the booted simulator.

## 6. Discipline + docs
- [x] Byte-freeze proof: `git diff` shows `src/native/tessellate/**`,
      `chamfer_edges.h`, `fillet_edges.h` byte-identical vs HEAD; new header OCCT-free.
- [x] Zero-regression: `test_native_blend` (31), `test_native_analytic_fillet` (5),
      `test_native_engine` (44) all pass unchanged.
- [x] OpenSpec change validated `--strict`.
