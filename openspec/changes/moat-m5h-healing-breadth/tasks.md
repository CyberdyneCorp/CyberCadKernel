# Tasks — moat-m5h-healing-breadth (MOAT M5 tail — short-edge merge)

Order: explore landed healer → pick defect class → land opt-in default-OFF pass →
host-analytic gate → SIM parity gate vs OCCT → openspec validate → discipline check →
commit. STRICTLY ADDITIVE to `src/native/heal/**`; OCCT-free; `cc_*` unchanged; weld
tolerance never widened; tessellator untouched.

## 0. Substrate + exploration

- [x] 0.1 Build numsci host + iossim (`scripts/build-numsci.sh host`, `iossim`); configure
      the kernel build (Unix Makefiles, `-DCYBERCAD_HAS_NUMSCI=ON`, NumPP/SciPP by absolute
      path). Establish the baseline: `test_native_heal` 23/23 green.
- [x] 0.2 Read the landed healer (`heal.cpp`/`heal.h`, `degenerate.h`, `gap_bridge.h`,
      `cap_hole.h`, `tolerant_sew.h`, `vertex_unify.h`, `self_verify.h`, `assemble_shell.h`)
      and the M5 roadmap section. Confirm the short-edge defect is UNHANDLED and distinct
      from `dropZeroLengthSides` (≤tol only) and `gap_bridge` (cross-face seam only).

## 1. Land the opt-in short-edge collapse pass (default-OFF)

- [x] 1.1 New `src/native/heal/short_edge.h` (header-only, OCCT-free): `collapseShortEdges`
      + `detail::collapseLoop` + `detail::distanceToLine`, with the five-layer bound
      (opt-in merge length, ¼·neighbour local-feature cap, collinearity within tol, loop
      stays ≥ 3 corners, deferral to the UNCHANGED self-verify) documented in the header.
- [x] 1.2 `HealOptions.shortEdgeMergeLen` appended LAST (default 0.0) so every existing
      positional aggregate-init is byte-identical; `HealMetrics.nCollapsedShortEdges` +
      `maxCollapsedShortEdge` added additively (`heal_result.h`).
- [x] 1.3 Wire into `heal.cpp` BEFORE the first sew, dead-guarded by `shortEdgeMergeLen >
      0.0`; add the include to `native_heal.h` + document pass #8 in its header.

## 2. Gate 1 — HOST analytic (no OCCT)

- [x] 2.1 Deliberately-defected fixture `cubeTopShortEdge(seg)`: a unit cube whose +Z face
      carries a COLLINEAR short edge of length `seg` on the c4→c5 boundary run.
- [x] 2.2 `heal_short_edge_collapse_heals`: with `shortEdgeMergeLen = 1e-2` the pass
      collapses the micro-edge → watertight unit cube `V = 1.0`, `nCollapsedShortEdges > 0`,
      `maxCollapsedShortEdge ≈ seg`, residual 0.
- [x] 2.3 `heal_short_edge_default_off_declines`: flag OFF ⇒ honest decline
      (`GapBeyondTolerance`, residual = `seg`), nothing collapsed, input UNCHANGED.
- [x] 2.4 `heal_short_edge_feature_cap_refuses`: merge length > seg but ¼·neighbour < seg ⇒
      the cap (not the caller) refuses.
- [x] 2.5 `heal_short_edge_non_collinear_declines`: a short edge that turns a real corner is
      NOT collapsed (collinearity layer refuses), input unchanged.
- [x] 2.6 `heal_short_edge_collapse_loop_layer`: unit-drive `collapseLoop` (collinear split
      collapses to a square; off-line notch survives).
- [x] 2.7 Full `test_native_heal` 28/28 green (23 landed unchanged + 5 new); full host
      `ctest` green (no cross-suite regression).

## 3. Gate 2 — SIM parity vs OCCT (booted simulator)

- [x] 3.1 `native_heal_parity.mm`: `cubeTopShortEdgeSoup` + `nativePolyFace` /
      `nativeShortEdgeSoup` (native six-face soup with the split hexagon) + `occtShortEdgeSoup`
      (the SAME soup as an OCCT compound with the 6-vertex top polygon).
- [x] 3.2 In-scope check: native collapse (mergeLen ≥ seg) matches OCCT `sewAndFix` at a
      tolerance ≥ seg — both watertight `V ≈ 1`, `nCollapsedShortEdges > 0`.
- [x] 3.3 Equal-or-more-conservative check: flag OFF ⇒ native declines
      (`GapBeyondTolerance`, nothing collapsed) while OCCT aggressively closes; assert native
      is NEVER a wrong repair and OCCT's closure is the same honest unit cube.
- [x] 3.4 `run-sim-native-heal.sh` 12/12 green (10 landed + 2 new); add `native_heal_parity.mm`
      to `run-sim-suite.sh` SKIP (own `main()` + own runner).

## 4. Finalize

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M5 status (short-edge slice landed, both gates,
      declined tail).
- [x] 4.2 `openspec validate moat-m5h-healing-breadth --strict` until valid.
- [x] 4.3 Discipline check: `git diff src/native` OCCT-free & additive; `cc_*` unchanged;
      existing heal callers byte-identical with the flag OFF; tessellator untouched.
- [x] 4.4 Commit to branch `moat-m5h` (concise technical message, no AI mention). Do NOT push.

## Declined (honest asymptotic tail → OCCT `ShapeFix`)

- Short edge that turns a REAL (non-collinear) corner (would change the boundary).
- Short edge whose removal needs the neighbour face re-projected.
- Pcurve reconstruction, self-intersecting-wire repair, arbitrary broken industrial B-rep.
