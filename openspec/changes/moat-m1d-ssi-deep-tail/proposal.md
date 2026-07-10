# Proposal — moat-m1d-ssi-deep-tail (MOAT M1 SSI deep tail: extend robust near-tangent crossing into the grazing regime)

## Why

M1c promoted the off-axis analytic-quadric intersection tail to verified and named the remaining
SSI moat core as the deep tail: **S4-c general near-tangent breadth** — GRAZING crossings where the
transversality sine (the angle between the two surface normals along the intersection) dips below
the band the marcher can currently cross.

The shipped S4-c crossing corrector (change `add-native-ssi-s4c-near-tangent-marching`) crosses a
near-tangent graze by freezing ONE reference tangent `t★` (the last-good pre-band tangent) as BOTH
the crossability anchor AND the fixed-plane advance direction for the whole crossing. Host probing
of the offset-cylinder-grazes-sphere family (unit sphere ∩ cylinder R=0.4 shifted +x) measured the
exact breadth boundary of that fixed-plane crossing:

| offset dx | r+dx | min transversality sine | shipped S4-c |
|-----------|------|-------------------------|--------------|
| 0.585     | 0.985| 0.173                   | CROSSES (1 closed loop) |
| 0.590     | 0.990| 0.141                   | **DEFERS** (nearTangentGaps=1) |
| 0.593     | 0.993| 0.118                   | DEFERS |
| 0.595     | 0.995| 0.100                   | DEFERS |

The floor is ≈ **0.17**. Below it the corrector fails with `c.ok == false` — the curve TURNS
materially through the tighter pinch, so the frozen `t★` plane slices the curve far from the guess
and the two-surface Newton corrector cannot land. This is an HONEST defer, but a RESOLVABLE one: the
graze is a genuine single transversal branch (band-minimum sine well above the 0.3·enter floor, no
steep sine collapse) — the marcher is just using a stale plane.

## What

- **Adaptive crossing re-anchoring (`src/native/ssi/marching.cpp`, ADDITIVE, default OFF).** A new
  `MarchOptions::adaptiveCrossReanchor` (with a `reanchorBlend` weight) lets the S4-c crossing
  corrector RE-ANCHOR its fixed-plane advance direction toward the LOCAL intersection tangent
  (`normalize(nA×nB)`, continuity-oriented) as it steps, so the advance plane FOLLOWS the curve's
  turn through a tighter graze instead of slicing it far from the guess. Three coupled refinements,
  all gated behind the flag:
  1. the advance plane normal blends the local curve tangent with the frozen `t★`;
  2. per-step progress is measured along the actual step direction (the curve advances along its
     own tangent), guarded by an anti-orbit total-arc cap so a non-traversing orbit still
     terminates and defers;
  3. hand-back to the normal S3 march happens once transversality recovers above the ENTER
     threshold (where S3 is comfortable) with a two-node stability requirement, rather than the
     higher exit-hysteresis threshold a WIDE graze may never reach between pinches.

  The HONESTY anchors are UNCHANGED: crossability is still decided against the frozen `t★` (the
  band-minimum floor, the steep-sine-collapse witness, the per-step ≥60° branch-flip guard), every
  node is still verified on BOTH surfaces at the SAME `onSurfTol`, and a genuine tangency / branch
  (sine → 0) still defers. It only follows the curve's turn — it never widens a tolerance and never
  fabricates a point.

- **Measured breadth extension.** With re-anchoring on, the robust-crossing floor drops from a
  transversality sine of ≈ **0.17 to ≈ 0.14** — the dx=0.590 (r+dx=0.990, minSine≈0.141) pose that
  the shipped corrector declines is now traced to a full closed loop (every node on both surfaces
  ≤ 1e-9, arc length matching the tolerance-below-dip ground truth). The dx=0.593–0.595 poses
  (minSine ≈ 0.12–0.10) remain an HONEST DECLINE even with re-anchoring on — the near-tangent band
  is then so wide (a large fraction of the loop is near-tangent) that the curve-following crossing
  cannot recover to a transversal stretch within budget, so it discards and defers rather than
  fabricate. That sharpened floor is the new measured next blocker.

- **Honest-decline preservation.** The default path (flag off) is BYTE-IDENTICAL to the shipped
  S4-c crossing for every prior case. A true tangency / branch (equal orthogonal cylinders, tangent
  sphere/cylinder equator) still defers with the flag on. No closed form is added.

## Impact

- `src/native/ssi/` — ADDITIVE: two default-off `MarchOptions` fields + the re-anchor branch inside
  the S4-c `crossNearTangent` crossing loop (`marching.cpp`, `marching.h`). No existing behaviour
  changes (flag defaults off). `git diff src/native` touches only `ssi/marching.{h,cpp}`.
- `src/native/**` — OCCT-free and additive; tessellator (`tessellate/`), boolean (`boolean/`), and
  blend layers UNTOUCHED.
- `cc_*` ABI — **unchanged** (SSI is internal; asserted at the C++ boundary, no facade entry point).
- Tests — additive: 2 new host cases (deep graze crossed with re-anchoring; honest decline below
  the extended floor) + 2 new sim parity cases (existing cases frozen).
- Spec — one new requirement in `native-ssi` (S4-c near-tangent breadth extended via adaptive
  re-anchoring, with the honest floor).
- Roadmap — M1 status updated: the near-tangent breadth floor extended (0.17 → 0.14), with the
  sharpened next blocker (wide-band grazes below ≈ 0.12 and coincident/overlapping freeform).
