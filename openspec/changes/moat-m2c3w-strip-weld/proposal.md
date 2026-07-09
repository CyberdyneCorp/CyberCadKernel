# Proposal — moat-m2c3w-strip-weld (MOAT M2 blocker #4, ≥3-seam WELD)

## Why

The landed chain seam-GRAPH builder (`src/native/boolean/seam_graph_chain.h`,
`buildChainSeamGraph`, change `moat-m2c3-chain-seam-graph`) assembles the three-arc,
two-junction seam CHAIN for the edge-straddling box pose but records the WELD as its
sharpened next blocker:

> "The 2-junction WALL SPLIT + strip weld (a generalization of `junction_split.h` /
> `multi_face_weld.h` to two interior valence-3 vertices) is the SHARPENED next blocker,
> tracked after this enabler."

This change lands exactly that weld: it consumes the landed chain graph and produces a
watertight two-operand result solid for the STRIP removal `A ∩ {x0 ≤ x ≤ x1, y ≥ y0}`,
for CUT (`A − B`) and COMMON (`A ∩ B`). It is the direct ≥3-seam analogue of the landed
corner-clip weld (`multi_face_weld.h`, change `moat-m2w-multiface-weld`): the corner clip
handled a removed QUADRANT + ONE interior junction + TWO box caps; this handles a removed
STRIP + TWO interior junctions + THREE box caps, staying entirely in the ROBUST
planar-cutter regime (flat box caps, iso-parametric arcs, ANALYTIC junctions, exact
curve-crossing analytic-face clips). It does NOT touch the byte-frozen M0 tessellator and
does NOT reintroduce the curved↔curved closed-seam weld fragility that gates blocker
#1/#3.

Two additive header-only verbs compose the weld:

- **`strip_split.h` (`splitFaceStrip`)** — the TWO-junction wall split, the additive
  sibling of the landed one-junction `splitFaceJunction`. Where the byte-frozen B2
  `splitFace` reaches the two boundary crossings but DECLINES the bent chain seam (its
  fixed-density reflatten shortcuts BOTH interior kinks; on the edge pose the two crossings
  even land on ONE boundary edge, collapsing its whole-vertex boundary-arc walk), the strip
  verb resolves both kinks by CONSTRUCTION: it introduces `J1`, `J2` as EXACT shared
  valence-3 vertices, building the seam as THREE straight-in-UV edges (E→J1, J1→J2, J2→X),
  and adds a full-ring boundary wrap for the same-edge crossing. Each straight-in-UV part
  reflattens to MACHINE PRECISION under the SAME strict rebuild tolerance B2 uses (never
  weakened). The wall partitions into the removed STRIP sub-face + the survivor.
- **`multi_face_strip_weld.h` (`multiFaceStripClip`)** — the STRIP weld, the additive
  sibling of `multiFaceCornerClip`. It clips A's analytic faces to the op's keep region
  (the flat bottom via an exact straight-edge notch reroute; the CURVED-topped walls via
  the byte-frozen exact-crossing `cutAnalyticFace`, composed twice for the back wall the
  strip cuts with BOTH parallel x-planes), synthesises the THREE box CAP faces sharing the
  three seam-arc segments with the wall sub-face, welds the shell and self-verifies
  (watertight + a consistent op-volume bound).

The reachability + correctness were MEASURED against the real fixture geometry: the strip
split tiles to the closed-form UV strip area (0.09) to 1e-12; the welded CUT/COMMON solids
are watertight (Euler χ = 2) with enclosed volume matching the closed-form strip oracle to
the curved-tessellation band and converging from above. OCCT remains the oracle: the sim
gate compares each native op against `BRepAlgoAPI_Cut/Common` on volume, area, watertight,
topology, bbox, one-sided Hausdorff and a 4000+-point classify batch — all with FIXED,
never-widened tolerances.

## What Changes

- ADD `src/native/boolean/strip_split.h` (header-only, OCCT-free): the `StripFaceSplit`
  value type + `splitFaceStrip(face, chainSeam, J1uv, J1_3d, J2uv, J2_3d, SplitOptions)`,
  an ADDITIVE SIBLING of `splitFaceJunction`. Reuses the byte-frozen B2 `detail::` +
  `jsdetail::` primitives verbatim; the only new steps are the three-seam-edge wire and the
  full-ring boundary wrap for the same-edge crossing.
- ADD `src/native/boolean/multi_face_strip_weld.h` (header-only, OCCT-free): `StripWeldOp`,
  `StripWeldDecline`/`StripWeldReport`, `multiFaceStripClip(A, g, ss, op, deflection,
  StripWeldReport*)`, an ADDITIVE SIBLING of `multiFaceCornerClip`. Reuses the byte-frozen
  `hscdetail::` (`cutAnalyticFace`, `orderLoop`, `planarFaceFromLoop`, `Piece`, `signedDist`),
  `mfwdetail::notchedBoxFace`, `assemble.h`, and the M0 `SolidMesher`/`isWatertight`/
  `enclosedVolume` — edits none.
- ADD to `tests/native/chain_seam_fixture.h`: the closed-form UV strip-area oracle
  (`uvStripArea`).
- ADD to `tests/native/test_native_chain_seam.cpp` (host GATE a): the strip-split-lands-
  where-B2-declines proof, the watertight CUT/COMMON-at-closed-form-volume proof, and the
  volume-converges-across-deflection proof.
- ADD `tests/sim/native_chain_seam_weld_parity.mm` +
  `scripts/run-sim-native-chain-seam-weld.sh` (sim GATE b): native-vs-OCCT parity of the
  strip weld CUT/COMMON.
- No `include/` change (no `cc_*` ABI surface). No existing `src/native/**` file edited;
  the M0 tessellator, B2 `splitFace`, `junction_split.h`, `seam_graph_chain.h`,
  `multi_face_weld.h` are byte-identical.

## Scope + honest boundary

This change lands the two-junction WALL SPLIT and the STRIP weld for CUT and COMMON — the
sharpened blocker the chain-graph change recorded. FUSE (`A ∪ B`) of the strip pose is NOT
yet reachable: it needs a box cutting face NOTCHED at BOTH junction columns simultaneously
(the middle plane `y = y0` is attached to the cap along the J1 AND J2 vertical columns,
beyond the single-attach `notchedBoxFace`), so `multiFaceStripClip` HONESTLY returns NULL
for FUSE (its self-verify rejects the non-watertight shell) → OCCT. This is the SHARPENED
next blocker (a two-attach-column box notch), asserted in both gates as the fallback
contract. Anything outside the reachable strip pose is a MEASURED decline (nullopt / NULL),
never a fudge, never a partial/leaky solid, never a widened tolerance.
