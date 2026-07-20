# Design — moat-m1d-ssi-deep-tail

## Root cause (measured, host-probed)

The shipped S4-c crossing corrector (`crossNearTangent`) crosses a near-tangent graze with a
FIXED-plane cut: it freezes the last-good pre-band tangent `t★` and, for the whole band, advances
along `t★` while the two-surface Newton corrector re-projects onto both surfaces holding
`dot(pa − prev, t★) = h`. The crossability decision (band-minimum sine floor, steep-sine-collapse
witness, per-step ≥60° branch-flip guard) is also anchored on `t★`.

Sweeping the offset-cylinder-grazes-sphere family toward tangency (unit sphere ∩ cyl R=0.4 offset
+x by dx; tangency at r+dx = 1.0) measured the fixed-plane floor:

| dx    | r+dx  | min transversality sine | shipped S4-c | root-cause at defer |
|-------|-------|-------------------------|--------------|---------------------|
| 0.585 | 0.985 | 0.173                   | crosses      | —                   |
| 0.590 | 0.990 | 0.141                   | DEFERS       | `c.ok==false` (corrector cannot land) |
| 0.593 | 0.993 | 0.118                   | DEFERS       | same                |
| 0.595 | 0.995 | 0.100                   | DEFERS       | same                |

At dx=0.590 the defer is NOT the band-min floor (measured band-min 0.141 ≫ the 0.075 = 0.3·0.25
floor) and NOT a steep collapse (stall sine 0.237 vs last-good 0.255, ratio 0.93 ≫ 0.25). It is the
CORRECTOR failing to converge (`c.ok == false`) after the step shrinks to `minStep`: the curve
turns materially through the tighter pinch, so the frozen `t★` plane slices the true curve far from
the linearized guess and the 4-equation Newton solve (3 for `pa − pb = 0`, 1 for the `t★` advance)
cannot land a point on both surfaces. A resolvable single-branch graze, declined by a stale plane.

## The fix — adaptive re-anchoring (three coupled refinements, all gated)

All three live inside the crossing loop and activate ONLY under `adaptiveCrossReanchor`; with the
flag off the loop is byte-identical to the shipped fixed-plane crossing.

### 1. Advance plane follows the curve
The advance direction `stepDir` is re-anchored toward the LOCAL intersection tangent
`t_local = normalize(nA × nB)` at the current node, continuity-oriented toward the crossing way,
blended with `t★` by `reanchorBlend` (default 0.5). Inside a CROSSABLE graze the pair sine stays
bounded above the floor, so `t_local` is a well-defined curve direction (a raw node secant was
tried first and REJECTED — too noisy at the fine crossing step). The advance plane
`dot(pa − prev, stepDir) = h` stays transverse to the turning curve, so the corrector keeps
landing on both surfaces deeper into the band.

### 2. Progress measured along the step; anti-orbit arc cap
Because the curve legitimately turns, per-step progress is measured along the actual `stepDir`
(the curve advances along its own tangent) rather than along the frozen `t★` — otherwise a
legitimately turning graze reads as a stall (`advance < 0.25·h`) exactly where `t★` and the local
tangent diverge. To keep this honest (a curve-following step could in principle orbit rather than
traverse) a total-arc cap bounds the crossing: a crossable graze traverses in a bounded arc, so a
run that exceeds `max(hCrossCap, loopClose·h0)·crossMaxSteps` is orbiting → discard + defer. It is
a TERMINATION SAFETY, never a correctness gate — far-side recovery + the on-both-surfaces / floor /
branch-flip guards decide the crossing.

### 3. Hand-back at the ENTER threshold, with stability
A WIDE graze has TRANSVERSAL stretches between pinches, but for the tighter poses those stretches
top out only modestly above the ENTER threshold (below the shipped 1.5·enter exit hysteresis). The
shipped exit test (`sine ≥ bandExitSin`) therefore never fires and the crossing runs out its step
budget mid-loop. On the reanchor path the hand-back to normal S3 fires at `bandEnterSin` (where S3
itself is comfortable) with a two-consecutive-recovered-node stability requirement (a single lucky
sample can't trigger a premature exit); if S3 dips near-tangent again it simply re-enters the
crossing. The recovered region is handed to the SAME S3 corrector that runs everywhere else — no
tolerance is weakened. To reach those recovery stretches within budget the reanchor step is allowed
to GROW back toward the normal `maxStep` while sine ≥ enter and SHRINK to the fine cap near a pinch,
so the fine steps are spent WHERE the graze actually is.

### The per-node crossability guard on the reanchor path
`crossNodeCrossable` keeps the sine floor and the per-step ≥60° branch-flip guard unchanged. Only
the "U-turn vs the FROZEN `t★`" test (`dot(rawNew, t★) ≤ 0`) is relaxed to continuity against the
PREVIOUS raw tangent, because across a tighter graze the curve legitimately turns MORE than 90° away
from the stale entry `t★` while remaining a single smooth branch. A genuine branch still trips the
per-step turn guard (a branch point flips the raw tangent sharply in ONE step), the sine floor, or
the anti-orbit arc cap.

## Measured breadth extension (numbers)
- Robust-crossing floor: min transversality sine ≈ **0.17 → ≈ 0.14**.
- dx=0.590 (r+dx=0.990, minSine≈0.141): shipped DEFERS; reanchor crosses to ONE closed loop,
  every node on both surfaces ≤ 8e-11, arc length within ~4% of the tolerance-below-dip ground
  truth (chord underestimate through the graze — same signature as the shipped dx=0.585 case).
- dx=0.593 (minSine≈0.118) and dx=0.595 (minSine≈0.100): HONEST DECLINE even with reanchor on —
  the near-tangent band is too wide to recover within budget; the marcher discards and defers.

## Why additive + non-regressing
- Both new `MarchOptions` fields default off; every reanchor-specific branch is guarded by
  `t.adaptiveCrossReanchor`. With the defaults the crossing loop, the per-node guard, the step
  control, and the hand-back threshold are byte-identical to the shipped S4-c crossing → every
  already-passing case (all host SSI suites, all prior sim cases) is unchanged, proven by re-running
  them green.
- No tolerance is widened. The reanchor uses the SAME `onSurfTol` for every node; a candidate that
  does not land on both surfaces is discarded, never fabricated. A genuine tangency / branch still
  defers.

## Alternatives rejected
- **Raw node-secant re-anchoring** — tried first; too noisy at the fine crossing step (the secant
  of two near-coincident nodes deep in the band points nearly tangent), destabilizing even the
  shipped dx=0.585 case. Replaced with the local `nA×nB` tangent.
- **Lowering the band-min floor / widening tangentSinTol** — rejected: that would weaken the
  crossable gate and risk crossing a true tangency. The floor is untouched.
- **Pushing into the wide-band regime (minSine < 0.12)** — declined: the crossing cannot recover to
  a transversal stretch within budget, so forcing it would fabricate. Left as the honest decline.
