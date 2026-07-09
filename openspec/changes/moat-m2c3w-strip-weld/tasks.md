# Tasks — moat-m2c3w-strip-weld

## 1. Two-junction wall split (`strip_split.h`)
- [x] 1.1 Add `StripFaceSplit`/`StripSplitResult`/`StripSplitDecline` + `splitFaceStrip`,
      the additive sibling of `splitFaceJunction`, reusing the byte-frozen B2 `detail::` +
      `jsdetail::` primitives verbatim.
- [x] 1.2 Locate BOTH junctions `J1`, `J2` as EXACT interior nodes of the clipped chain
      chord; build THREE straight-in-UV seam edges (E→J1, J1→J2, J2→X).
- [x] 1.3 Handle the same-boundary-edge crossing (strip pokes through one edge): a
      full-ring boundary wrap for the survivor sub-loop (`buildStripWire`), disambiguated
      by the on-edge parameter.
- [x] 1.4 Self-verify with the SAME strict B2 tolerances (tiling gap, simple sub-loops,
      bit-identical shared seam, rebuild reflatten to machine precision) — never weakened.

## 2. Strip weld (`multi_face_strip_weld.h`)
- [x] 2.1 Add `StripWeldOp`/`StripWeldDecline`/`StripWeldReport` + `multiFaceStripClip`,
      the additive sibling of `multiFaceCornerClip`.
- [x] 2.2 Clip A's analytic faces to the op keep region: the FLAT bottom via an exact
      straight-edge notch reroute (`rerouteStripSurvivor`, edge-crossing notch); the
      CURVED-topped walls via the byte-frozen exact-crossing `cutAnalyticFace`, composed
      twice (`cutWallBetweenX`) for the back wall the strip cuts with both x-planes.
- [x] 2.3 Synthesise the THREE box CAP faces (left/middle/right), sharing the three seam
      arc segments with the wall strip sub-face and the vertical junction columns.
- [x] 2.4 Mandatory self-verify: mesh the welded result, require watertight + a consistent
      op-volume bound; NULL → OCCT on any failure (no leaky/wrong-volume solid).
- [x] 2.5 FUSE honest decline: the two-attach-column box notch is not yet reachable, so
      FUSE returns NULL (self-verify rejects) → OCCT.

## 3. Host gate (a) — closed-form, no OCCT
- [x] 3.1 Add the closed-form UV strip-area oracle (`uvStripArea`) to the fixture.
- [x] 3.2 Prove the strip split lands where byte-frozen B2 declines (TilingGap), tiling to
      the UV strip area to 1e-12 with machine-precision rebuild residual.
- [x] 3.3 Prove the welded CUT/COMMON solids are watertight at the closed-form strip
      volumes (≤ 2% at deflection 0.01) and converge monotonically as deflection tightens.

## 4. Sim gate (b) — native-vs-OCCT on the booted simulator
- [x] 4.1 Add `native_chain_seam_weld_parity.mm` + `run-sim-native-chain-seam-weld.sh`.
- [x] 4.2 Compare native CUT/COMMON vs `BRepAlgoAPI_Cut/Common` on volume, area,
      watertight, Euler χ, bbox, one-sided Hausdorff, and a classify batch (zero crisp
      IN↔OUT disagreements) at two deflections — all with fixed tolerances.
- [x] 4.3 Assert the FUSE fallback contract (NULL → OCCT).

## 5. Discipline
- [x] 5.1 `src/native/**` OCCT-free (0 OCCT includes/symbols); no `cc_*` ABI change.
- [x] 5.2 Additive only — no existing `src/native/**` file edited; the byte-frozen
      tessellator, B2, `junction_split.h`, `seam_graph_chain.h`, `multi_face_weld.h`
      untouched.
- [x] 5.3 Full native suite green (zero regression); `openspec validate --strict`.
