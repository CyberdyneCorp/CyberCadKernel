# Tasks — moat-m5h-collinear-vertex (MOAT M5 tail — collinear-vertex removal)

Order: explore landed healer → pick defect class → land opt-in default-OFF pass →
host-analytic gate → SIM parity gate vs OCCT → openspec validate → discipline check →
commit. STRICTLY ADDITIVE to `src/native/heal/**`; OCCT-free; `cc_*` unchanged; weld
tolerance never widened; tessellator untouched.

## 0. Substrate + exploration

- [x] 0.1 Reuse the prebuilt numsci (host); configure kernel build (Unix Makefiles,
      `-DCYBERCAD_HAS_NUMSCI=ON`, NumPP/SciPP by absolute path). Baseline
      `test_native_heal` 28/28 green.
- [x] 0.2 Read the landed healer (`heal.cpp`, `degenerate.h`, `gap_bridge.h`, `cap_hole.h`,
      `short_edge.h`, `tolerant_sew.h`, `vertex_unify.h`) and the M5 roadmap section.
      Confirm the single-collinear-vertex defect is UNHANDLED and distinct from
      `dropZeroLengthSides` (≤tol only), `gap_bridge` (cross-face seam only), and
      `short_edge` (¼·neighbour micro-edge, removes two corners, `collapseLoop` length band
      rejects a full-length `B→C`).

## 1. Land the opt-in collinear-vertex removal pass (default-OFF)

- [x] 1.1 New `src/native/heal/collinear_vert.h` (header-only, OCCT-free):
      `removeCollinearVertices` + `detail::removeLoopVerts` + `detail::perpAndParam`, with
      the four-layer bound (opt-in flag, collinearity within tol AND strict-between
      projection `0<t<1`, loop stays `≥ 3` corners, deferral to the UNCHANGED self-verify)
      documented in the header. NO length parameter — exact collinearity is the sole
      criterion.
- [x] 1.2 `HealOptions.removeCollinearVerts` appended LAST (default false) so every existing
      positional aggregate-init is byte-identical; `HealMetrics.nRemovedCollinearVerts` +
      `maxCollinearVertDev` added additively (`heal_result.h`).
- [x] 1.3 Wire into `heal.cpp` BEFORE the first sew, dead-guarded by `removeCollinearVerts`;
      add the include to `native_heal.h` + document pass #9 in its header.

## 2. Gate 1 — HOST analytic (no OCCT)

- [x] 2.1 Deliberately-defected fixture `cubeTopCollinearVert(t, off)`: a unit cube whose +Z
      face carries ONE extra vertex at parameter `t` on the c4→c5 run, offset `off`
      perpendicular in-plane (off==0 ⇒ collinear T-vertex; off>0 ⇒ real corner).
- [x] 2.2 `heal_collinear_vert_removal_heals`: with `removeCollinearVerts=true` the pass
      removes the redundant vertex → watertight unit cube `V = 1.0`,
      `nRemovedCollinearVerts > 0`, `maxCollinearVertDev ≤ tol`, `nCollapsedShortEdges == 0`,
      residual 0.
- [x] 2.3 `heal_collinear_vert_default_off_declines`: flag OFF ⇒ honest decline
      (`GapBeyondTolerance`, residual > tol), nothing removed, input UNCHANGED.
- [x] 2.4 `heal_collinear_vert_real_corner_declines`: a vertex 0.1 off the line (a real
      corner) is NOT removed even with the flag ON (collinearity layer refuses), input
      unchanged.
- [x] 2.5 `heal_collinear_vert_remove_loop_layer`: unit-drive `removeLoopVerts` (collinear
      vertex removed; off-line notch kept; backtracking spur `t∉(0,1)` kept).
- [x] 2.6 Full `test_native_heal` 32/32 green (28 landed unchanged + 4 new).

## 3. Gate 2 — SIM parity vs OCCT (booted simulator)

- [x] 3.1 `native_heal_parity.mm`: `cubeTopCollinearVertSoup` + `nativeCollinearVertSoup`
      (native six-face soup with the pentagon) + `occtCollinearVertSoup` (the SAME soup as
      an OCCT compound with the 5-vertex top polygon).
- [x] 3.2 In-scope check: native removal (flag ON) matches OCCT `sewAndFix` — both watertight
      `V ≈ 1`, `nRemovedCollinearVerts > 0`.
- [x] 3.3 Equal-or-more-conservative check: flag OFF ⇒ native declines (`GapBeyondTolerance`,
      nothing removed) while OCCT aggressively closes; assert native is NEVER a wrong repair
      and OCCT's closure is the same honest unit cube.
- [x] 3.4 `run-sim-native-heal.sh` 14/14 green (12 landed + 2 new).

## 4. Finalize

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M5 status (collinear-vertex slice landed, both
      gates, declined tail).
- [x] 4.2 `openspec validate moat-m5h-collinear-vertex --strict` until valid.
- [x] 4.3 Discipline check: `git diff src/native` OCCT-free & additive; `cc_*` unchanged;
      existing heal callers byte-identical with the flag OFF; tessellator untouched.
- [x] 4.4 Commit to branch `moat-seq` (concise technical message, gate numbers, no AI
      mention). Do NOT push.

## Declined (honest asymptotic tail → OCCT `ShapeFix`)

- Vertex that turns a REAL (non-collinear) corner (would change the boundary).
- Vertex whose removal needs the neighbour face re-projected.
- Pcurve reconstruction, self-intersecting-wire repair, arbitrary broken industrial B-rep.
