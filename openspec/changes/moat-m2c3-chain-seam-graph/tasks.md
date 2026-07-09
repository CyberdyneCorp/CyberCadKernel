# Tasks — moat-m2c3-chain-seam-graph (MOAT M2 blocker #4, ≥3-seam)

Order: diagnose reachability on the real fixture (no OCCT) → chain seam-graph builder
(three-cutting-face set + iso-classify + chain order + two analytic junctions + arc
clip/join) → host analytic gate → sim native-vs-OCCT parity gate → zero-regression proof
→ docs, OR HONEST DECLINE at the sharpest reachable level. All new native code stays
OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::boolean`. No `cc_*` ABI change. The change is strictly ADDITIVE: B1
`recogniseFreeformSolid`, B2 `splitFace`/`splitFaceSmoothTrim`, M0 `SolidMesher`, M1
`traceWallSeam`, `buildSeamGraph`, `junction_split.h`, `multi_face_weld.h`, and the
analytic `recogniseCurvedSolid`/`classifyPoint` stay BYTE-IDENTICAL. No tolerance is
weakened; a correct decline is a first-class outcome; no seam-graph stub.

## STATUS — LANDED (both gates green)

DIAGNOSE (host-analytic, on the real fixture, no OCCT — grounding for reachability):
- [x] **Pose confirmed.** `A` = `first_freeform_boolean_fixture` bowl-lidded prism; `B` =
  the edge box `x ∈ [−0.15, 0.15], y ∈ [0.0, 0.8], z ∈ [−0.6, 0.2]`. The pole-straddle
  predicate reports EXACTLY three cutting faces (`x = −0.15`, `x = +0.15`, `y = 0`); the
  other three faces contain `A`.
- [x] **Iso + junctions confirmed.** The arcs are iso-parametric — `u = 0.350`,
  `u = 0.650` (parallel ends), `v = 0.500` (middle). The two analytic junctions
  `(0.35, 0.50) → (−0.15, 0, 0.009)` and `(0.65, 0.50) → (0.15, 0, 0.009)` lie on BOTH
  adjacent planes with residual ~5e-13 ≪ weldTol = 1.1e-7.
- [x] **Closed-form strip oracle confirmed.** The removed strip `A ∩ {−0.15 ≤ x ≤ 0.15,
  y ≥ 0}` integrates to a strictly-interior discriminating fraction of `V(A)`; the
  partition + union identities hold to 1e-12.

BUILD (additive):
- [x] `src/native/boolean/seam_graph_chain.h`: `ChainSeamGraph`, `ChainSeamDecline`,
  `buildChainSeamGraph`, with `sgcdetail::{findThreeCuttingFaces, traceArcs, orderChain,
  computeJunctions, joinChain}` helpers (backend cognitive-complexity band). OCCT-free,
  header-only, additive sibling of `seam_graph.h`.

GATE A (host analytic, no OCCT) — `tests/native/test_native_chain_seam.cpp` (5/5):
- [x] chain graph builds: three distinct cutting faces, two-parallel-ends + one-middle,
  canonical order, both junctions on both planes (residual < 1e-9), `chainSeam` bent with
  `J1`, `J2` as exact interior vertices, join gap < 0.02·diag.
- [x] middle arc spans both junctions; ends each fix one junction coordinate.
- [x] strip oracle partition + union identities.
- [x] two-face corner box DECLINES `NotThreeCuttingFaces`; non-freeform operand declines.
- [x] `tests/native/chain_seam_fixture.h`: edge box (reuses `buildCornerBox`) + strip
  oracle (reuses `polyVolume`, `clipHalf`).

GATE B (sim native-vs-OCCT, booted iOS sim) — `tests/sim/native_chain_seam_parity.mm`
(9/9), runner `scripts/run-sim-native-chain-seam.sh`:
- [x] every native arc node on OCCT bowl surface (≤ 1e-4, measured 1.4e-15) and on its
  cutting plane (measured 1.1e-12).
- [x] both junctions are exact triple points (on surface + both planes, ~1e-13).
- [x] `BRepAlgoAPI_Section(B, bowlFace)` is a connected 3-edge chain with both native
  junctions on it (~1e-13).
- [x] two-face corner box declines `NotThreeCuttingFaces`.

ZERO-REGRESSION:
- [x] `src/native/**` byte-identical (only the new header added); `include/` diff empty
  (no `cc_*` ABI change); tessellator untouched.
- [x] host suites green: `test_native_tessellate` 17/17, `test_native_multi_seam` 8/8,
  `test_native_curved_wall_cut` 12/12, `test_native_two_operand_freeform_boolean` 6/6,
  `test_native_first_freeform_boolean` 5/5, `test_native_smooth_trim_split` 7/7.

DOCS:
- [x] roadmap M2 blocker #4 note updated with the landed enabler + the sharpened next
  blocker (the 2-junction weld).

HONEST NEXT BLOCKER (out of this slice): the 2-junction WALL SPLIT + strip WELD — a
generalization of `junction_split.h` (two interior valence-3 vertices instead of one) and
`multi_face_weld.h` (a strip removal region + its box caps) — is the sharpened next
target. It stays inside the robust planar-cutter regime (flat caps, analytic junctions)
and has both the closed-form strip oracle (landed here) and an OCCT `BRepAlgoAPI_*`
parity oracle, so it is verifiable without faking.
