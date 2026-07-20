# Proposal — moat-m1e-ssi-wide-band (MOAT M1 SSI wide band: remove the 90°-turn reversal trap in the S4-c re-anchored crossing)

## Why

M1d shipped adaptive crossing re-anchoring and recorded a robust-crossing floor of a minimum
transversality sine of ≈ **0.14**, with the poses below it (dx = 0.593–0.595, minSine ≈ 0.12–0.10)
documented as an HONEST DECLINE explained by a WIDE near-tangent band: "the near-tangent band is so
wide (a large fraction of the loop is near-tangent) that the curve-following crossing cannot recover
to a transversal stretch within budget".

**That explanation is wrong, and a five-probe adversarially-verified diagnosis refuted it three
independent ways.** The declines are a DEFECT, not a limit:

- **The band is not wide.** Closed-form, verified against the kernel's own adapter normals to 1.5e-15
  over 20 000 samples: the sub-threshold arc is **6.76% / 7.93% / 8.69% / 9.27%** of the loop at
  dx = 0.585 / 0.590 / 0.593 / 0.595. Nine percent is not "a large fraction", and it grows only 1.37×
  across a binary cross → decline flip.
- **The budget is not binding.** Raising `crossMaxSteps` 256 → 16384 (64×) changes nothing: still an
  exit on budget, with arc growing exactly linearly. The anti-orbit arc cap is derived FROM the step
  budget, so it scales in lockstep and can never fire.
- **The declining runs never traverse.** Node positions read from the emitted vector show a **period-2
  (dx=0.593) / period-4 (dx=0.595) spatial limit cycle**: consecutive displacements are exact
  negatives (normalized dot −1.00), two points ~1.5e-2 apart revisited for 235 consecutive steps,
  arc 3.86 for net transport 0.21 — 5.6% efficiency. It oscillates; it does not crawl.

**The actual mechanism is a 90°-turn reversal trap on a frozen reference direction.**
`crossNearTangent` freezes `dirStar = t★` at band entry and uses that stale vector for TWO half-space
decisions — the local tangent's SIGN (`marching.cpp:552`) and its ADOPTION GATE (`:553`). Both
degenerate at the same point: when the curve's tangent has turned 90° from `t★`. Past it the sign
test flips the true FORWARD tangent to BACKWARD, the step retreats, one step back the turn is inside
90° again, and it steps forward — self-sustaining.

An analytic predictor derived with **no marcher involved** confirms it: the turn accumulated by the
time the sine recovers to the band-enter threshold is 72.3° / 83.8° / 90.5° / 94.8° at the four
poses, predicting the 90° crossover at **dx★ = 0.592787**. The measured shipped boundary from a 1e-4
sweep is **dx = 0.5926 crosses / 0.5927 declines** — agreement to 1e-4. It also predicts an
observable it was not fitted to: the parked sine at dx=0.593, 0.22999 analytic vs 0.230063 measured.

**So the M1d "floor ≈ 0.14 minSine" was an ARTEFACT of the trap, and minSine was never the governing
quantity.** The real shipped boundary is an ANGULAR one.

## What

- **Incremental orientation (`src/native/ssi/marching.cpp`, ADDITIVE, default OFF).** A new
  `MarchOptions::reanchorIncrementalOrientation`, consulted only under `adaptiveCrossReanchor`,
  re-references BOTH half-space tests to the PREVIOUS ACCEPTED step direction (seeded to `t★`).
  Continuity is monotone across an arbitrarily large accumulated turn. **Both tests must move
  together** — re-referencing only the sign test was tested and FAILS, merely relocating the decline
  from budget-exhaustion to step-floor collapse.

- **Net-transport termination guard** (same path, default-off with it). Bounds arc spent against net
  displacement from band entry at 4×. The shipped per-step `advanced` test is structurally blind here
  — it measures progress along the very direction the step was taken in, which the corrector's fourth
  residual pins to the requested step (measured `advance/h = 1.000000000` on EVERY accepted step,
  including every reversing one). Measured separation: **17.8 / 20.4** at the failing poses vs **1.19**
  at the crossing pose — a >14× gap, so the threshold is not delicate.

- **Measured breadth extension.** dx = 0.593 / 0.595 / 0.597 now cross to ONE closed loop, every node
  on both surfaces to **1.31e-11 / 1.16e-11 / 2.39e-11**. The new floor is **minSine ≈ 0.077**, and
  the new limiter is `minCrossSine = 0.075` — **the DESIGNED honesty tolerance**, reached at the
  band-minimum gate with ZERO crossing nodes emitted. That is where a floor should sit.

- **Honest-decline preservation.** Flag off is byte-identical for every prior case (including the
  M1d decline test at dx=0.595, kept UNMODIFIED). dx = 0.5975 / 0.598 still decline with the flag on;
  true tangency still defers; poses past tangency are unchanged.

## Impact

- `src/native/ssi/` — ADDITIVE: one default-off `MarchOptions` field + the orientation reference and
  net-transport guard inside `crossNearTangent` (`marching.{h,cpp}`). No existing behaviour changes.
- `cc_*` ABI — **unchanged** (SSI is internal).
- Tests — additive: 2 host cases (wide-band crossed + flag-off/inert byte-identity), 2 host parity
  cases. All prior cases frozen.
- Gates — host Gate A **24/24**; host Gate B **21/0** (19 prior frozen + 2 new).
- Spec — one new requirement in `native-ssi`.
- Roadmap — M1 status CORRECTED: the wide-band explanation is retracted as measurably false.

## NEWLY EXPOSED — the next blocker, found by the parity gate

Gate B at dx = 0.597 **failed** at `onCurve = 6.00e-04` against a 5e-4 tolerance, and the cause is
NOT the march. Measured off-surface deviation, corrected nodes vs the fitted convenience B-spline:

| dx | nodes | fitted spline |
|----|-------|---------------|
| 0.593 | 1.31e-11 | 2.97e-05 |
| 0.595 | 1.16e-11 | 3.36e-05 |
| 0.597 | 2.39e-11 | 6.99e-05 |

The fit is six orders of magnitude worse than the march it interpolates. The one-shot
densify-and-refit never fires because its target is `scale·2e-4` ≈ **2.2e-3** — four times LOOSER
than the parity gate's on-curve tolerance. `onCurveTol` was NOT widened to absorb this; dx = 0.597 is
excluded from Gate B and documented, while Gate A still covers its crossing. **The fitted-curve
resolution at extreme graze curvature is the next slice.**
