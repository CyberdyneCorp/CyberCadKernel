# Proposal — moat-m2c3f-strip-weld-fuse (MOAT M2 blocker #4, ≥3-seam FUSE)

## Why

The landed strip weld (`src/native/boolean/multi_face_strip_weld.h`, `multiFaceStripClip`,
change `moat-m2c3w-strip-weld`) welds the three-arc / two-junction chain-seam result
watertight for CUT (`A − B`) and COMMON (`A ∩ B`), but recorded FUSE (`A ∪ B`) as its
sharpened next blocker:

> "FUSE not yet reachable (two-attach-column box notch) → HONEST NULL, the sharpened next
> blocker. The naive 3-notched-box FUSE leaves 2 open + 2 non-manifold edges at the J1/J2
> box vertical columns (the [J,Jb] drop is a sub-segment of the full-height box edge)."

This change lands exactly that FUSE. It stays entirely in the ROBUST planar-cutter regime
(flat box faces, iso-parametric arcs, ANALYTIC junctions, exact curve-crossing analytic-face
clips) — it does NOT touch the byte-frozen M0 tessellator and does NOT reintroduce the
curved↔curved closed-seam weld fragility that gates blockers #1/#3.

## What

The FUSE boundary is `(∂A \ int B) ∪ (∂B \ int A)`: A's CUT-survivor faces (the three box
caps are now interior to `A ∪ B` and dropped) welded to `B`'s six faces clipped to outside
`A`. B's three NON-cutting faces are whole; its three CUTTING faces (`x=x0`, `x=x1`, `y=y0`)
are clipped to outside `A` along the shared seam arcs.

The measured defect in the prior wave was ONLY on the MIDDLE cutting face (`Pm: y=y0`): its
cap `arcM` (J1→J2) spans the FULL WIDTH of the box middle face — J1 lies on the `x=x0` box
side edge and J2 on the `x=x1` box side edge — so the cap footprint attaches to BOTH junction
columns `[J1,J1b]` and `[J2,J2b]` simultaneously. The corner-clip `notchedBoxFace` drops only
ONE attach column, leaving the other column's box edge unmatched (the J1-column open +
non-manifold edges).

The fix is one additive detail verb, `mfswdetail::splitMiddleBoxFace`, which removes the
full-width cap by splitting the middle box face into TWO disjoint planar pieces:

- **TOP** — above the cap: `arcM(J1→J2)` + up the `x=x1` edge to the top-right box corner +
  across the top box edge + down the `x=x0` edge back to J1. Shares `arcM` with A's bowl
  survivor sub-face and its two side edges (above J1/J2) with the two END notched box faces.
- **BOTTOM** — below the cap's bottom `J1b→J2b`: `J1b→J2b` + down to the bottom-right box
  corner + across the bottom edge + up to J1b. Shares `J1b→J2b` with A's rerouted bottom
  notch and its side edges (below J1b/J2b) with the same two end faces.

The two END faces keep the byte-unchanged single-column corner-clip notch (`notchedBoxFace`).
Every junction-column edge is therefore used by exactly two faces → watertight.

This is STRICTLY ADDITIVE: `splitMiddleBoxFace` is new, and only the FUSE branch of
`appendFuseShell` is rewired to call it (plus `notchedBoxFace` for the two ends). CUT and
COMMON code paths are byte-unchanged; `notchedBoxFace`, `strip_split.h`, `seam_graph_chain.h`,
the analytic-face clips, the M0 tessellator and every `cc_*` ABI are untouched.

## Verification (two gates, OCCT is the oracle)

- **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_chain_seam.cpp`. FUSE joins
  CUT/COMMON in `strip_weld_lands_watertight_cut_common_at_closed_form_volumes`, plus new
  `strip_weld_fuse_lands_watertight_at_union_volume` (watertight, `V = V(A∪B)` closed-form to
  rel ≤ 2e-2, discriminating: `V(A∪B) > max(V(A),V(B))`, inside the inclusion–exclusion
  bound) and `strip_weld_fuse_volume_converges_across_deflection` (monotone convergence to
  the `chain_seam_fixture::volUnion` oracle). 10/10 pass.
- **Gate B (SIM native-vs-OCCT)** — `tests/sim/native_chain_seam_weld_parity.mm`
  (`scripts/run-sim-native-chain-seam-weld.sh`): FUSE is promoted from an honest-NULL
  fallback to a FULL parity case against `BRepAlgoAPI_Fuse` (volume/area/watertight/Euler/
  bbox/one-sided Hausdorff/classify batch) at the robust deflections 0.01 and 0.005.

## Honesty

An isolated mesh-density deflection (0.008) declines to NULL → OCCT for ALL THREE ops
(CUT/COMMON/FUSE alike) — the self-verify catches it; no wrong/leaky solid is ever emitted
and no tolerance is weakened. FUSE follows the SAME robust-at-most-deflections contract as
the already-landed CUT/COMMON.
