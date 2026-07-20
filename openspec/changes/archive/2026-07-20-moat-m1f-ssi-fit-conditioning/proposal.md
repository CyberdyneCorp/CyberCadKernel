# Proposal — moat-m1f-ssi-fit-conditioning (MOAT M1 SSI fit: stop the densify refit reaching interpolation)

## Why

M1e restored the wide-band grazing crossings but had to EXCLUDE dx = 0.597 from the native-vs-OCCT
parity gate: the crossing was correct (every corrected node on both surfaces to 2.39e-11) while the
fitted convenience B-spline missed the OCCT locus by **6.00e-04** against a 5e-4 tolerance. The
tolerance was deliberately NOT widened. This change fixes the fit and restores the pose on merit.

The obvious remedy — tighten the densify trigger — is a TRAP, and shipping it alone makes things
worse. Two defects interact:

1. **The refit pole target reached interpolation.** `target = std::min(m, kDensifyMaxPoles)` with
   `kDensifyMaxPoles = 200` resolves to `m` for any loop with m ≤ 200 — exactly the moderate-node
   high-curvature regime the refit exists to serve (the graze family is m = 193…195). At
   `nPoles == m` the system is square and interpolating and the clamped-uniform knot vector over a
   chord-length parametrization degenerates: the curve rides every node exactly while oscillating
   between them. Measured at dx = 0.597: a 195-pole refit reports `maxFitError` **3.639e-06** —
   apparently excellent — while its true off-surface deviation is **4.990e-01**.

2. **The trigger was looser than the verification budget.** `fitTarget = scale · 2e-4` = 2.203e-03
   at this family's scale 11.014390, ≈ 4× LOOSER than the 5e-4 on-curve budget the parity gate
   enforces. The failing pose's `maxFitError` (6.690100e-04) sat 3.3× UNDER the trigger, so the
   refit never fired and the 64-pole fit shipped.

Tightening (2) WITHOUT fixing (1) was measured end-to-end on the real gate: **Gate B 21 passed /
2 failed** — dx = 0.597 degrades 6.00e-04 → **1.85e-01**, and the currently-green dx = 0.595
REGRESSES 1.73e-04 → 1.90e-01. Two verifiers reproduced this independently.

**A knot-vector remedy was tried and REJECTED.** Replacing `clampedUniformKnots` with
`math::approxKnots` (Piegl–Tiller 9.68) looked like a free 4.8× win at 64 poles. On the real gate it
breaks `skew cyl unequal` — two clean orthogonal-cylinder transversal loops with NO near-tangency —
driving onSurf 7.39e-07 → **1.25e-06** against its own declared 1e-6 tolerance, while perturbing
16–18 of 23 gate lines and deepening the conditioning cliff it was meant to cure (1700× worse than
uniform knots at 160 poles). The knot vector is NOT touched.

## What

- **Conditioning guard (`marching.cpp`).** `target = std::min(m * 2 / 3, kDensifyMaxPoles)`. Keeps
  the pole/node ratio clear of the measured degeneration onset (≈ 0.88–0.95). The cap must be
  RELATIVE: a flat 112 or 128 starves `march_densify_refit_high_curvature_loop`, whose loop
  genuinely needs all 200 (verified twice). For m ≥ 300 this is a literal NO-OP, so every refit
  firing in the gates today is unchanged apart from three that drop 6–11% in pole count — a saving
  in an O(m·poles²) solve.

- **Between-node accept test (`marching.cpp`).** The existing `maxFitError` comparison is ANDed with
  a node-midpoint deviation check. The at-node metric cannot see the failure mode by construction.
  Discriminating power measured ≈ 1700×: a healthy 130-pole candidate reads 2.29e-04 at midpoints,
  a blown-up one 3.97e-01. (A pole-envelope / bounding-box variant was tried and REFUTED — it is
  anti-correlated in the band that matters.)

- **Trigger (`marching.cpp` + `MarchOptions::fitDensifyTargetScale`, default 2e-5).** Threads
  between the two measured populations: ordinary loops fit to ~5e-07 (worst ordinary 2.8e-05, still
  8× under) and stay byte-identical; near-tangent loops at 2e-04…7e-04 now trip it. 2e-6 is the
  measured cliff where ordinary loops start buying 1.1–1.3 s refits.

- **Always-on, with an escape hatch.** The conditioning guard is a latent-bug fix that is a no-op
  for every refit in either gate today — gating it behind a flag would ship the hazard armed. The
  trigger does change behaviour on loops that trip it, so it is exposed as a `MarchOptions` field a
  caller can restore. Both gates were run end-to-end and NO line regresses.

## Impact

- `src/native/ssi/` — `ssi/marching.{h,cpp}` only; OCCT-free; `cc_*` unchanged.
- Gates — host Gate A **25/25** (24 prior + 1 new regression case); host Gate B **22/0** with
  dx = 0.597 RESTORED. Exactly 2 of 22 lines move, both improving 21× and 82×; the other 20 are
  byte-identical. ssi 11/11, seeding 11/11, exact_fuzz 147 agreed / 0 disagreed.
- Cost — 0 of 16 ordinary analytic poses fire the refit at 2e-5. On freeform corpora the honest
  numbers diverge and both are reported: one verifier measured whole-Gate-A wall **+17.8%**, another
  measured fit-stage aggregate **+0.29%** with whole-gate wall inside baseline run-to-run spread.
  **Treat +17.8% as the pessimistic bound.**

## Measured effect (real OCCT parity metric, tolerance untouched at 5e-4)

| case | before | after |
|------|--------|-------|
| wide-band 0.597 onCurve | 6.00e-04 **FAIL** | **2.30e-05** PASS |
| wide-band 0.595 onCurve | 1.73e-04 | 8.06e-06 |
| wide-band 0.593 | 1.44e-04 | unchanged (does not fire) |
| other 20 Gate B lines | — | byte-identical |

## Caveat

`scale · 2e-5` still misses dx = 0.590 (1.45e-04) and dx = 0.593 (1.98e-04), which stay at 64 poles.
dx = 0.593 therefore passes at 1.44e-04, only 3.5× under budget. That margin is accepted;
tightening further crosses toward the 2e-6 cliff.
