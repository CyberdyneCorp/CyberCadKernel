# Tasks — moat-m1d-ssi-deep-tail

## 1. Diagnose (host, OCCT-free)
- [x] 1.1 Sweep the offset-cylinder-grazes-sphere family (unit sphere ∩ cyl R=0.4 shifted +x)
  toward tangency and MEASURE, per pose, the minimum transversality sine along the closed-form
  ground-truth loop and whether the shipped S4-c crossing crosses or defers.
- [x] 1.2 Pin the shipped floor at min transversality sine ≈ 0.17 (dx=0.585 crosses; dx=0.590,
  minSine≈0.141, defers) and identify the ROOT CAUSE via instrumentation: at dx=0.590 the crossing
  defers with `c.ok == false` (corrector cannot land) — the frozen `t★` plane slices the turning
  curve far from the guess, NOT the band-min floor (0.141 ≫ 0.075) and NOT a steep collapse.

## 2. Adaptive crossing re-anchoring (src/native/ssi, additive, default-off)
- [x] 2.1 Add `MarchOptions::adaptiveCrossReanchor` (+ `reanchorBlend`), both DEFAULT OFF; resolve
  into `Tuned`.
- [x] 2.2 In `crossNearTangent` (`marching.cpp`), when the flag is set, re-anchor the advance
  direction toward the local intersection tangent (blended with `t★`), measure per-step progress
  along the actual step direction, add an anti-orbit total-arc cap, and hand back to S3 at the
  ENTER threshold with a two-node stability requirement. Keep the crossability decision (band-min
  floor, steep-collapse, per-step ≥60° branch-flip) against the frozen `t★`.
- [x] 2.3 Prove non-regression: every prior SSI host suite passes UNCHANGED with the flag default
  off (ssi 11, seeding 9, s4_classification 22, s4e 7, s4f 6, boolean 4, curved_boolean 11,
  marching 19→21 with the new cases).

## 3. Gate A — host self-consistency (`test_native_ssi_marching.cpp`)
- [x] 3.1 `march_deep_near_tangent_reanchor_crossed_s4c` — dx=0.590 (minSine≈0.141): reanchor OFF
  DEFERS (nearTangentGaps==1, no curve); reanchor ON crosses to ONE closed loop, nearTangentGaps==0,
  nearTangentCrossed≥1, every node on both surfaces ≤ 1e-9, arc length within a step-bounded window
  of the tolerance-below-dip ground truth.
- [x] 3.2 `march_deep_near_tangent_reanchor_honest_decline_s4c` — dx=0.595 (minSine≈0.100): EVEN
  WITH reanchor ON the marcher honestly declines (nearTangentCrossed==0, no Closed loop,
  nearTangentGaps≥1), while a ground-truth loop still exists (traced with tol below the dip).

## 4. Gate B — sim native-vs-OCCT parity (`native_ssi_marching_parity.mm`)
- [x] 4.1 `pairDeepNearTangentReanchorS4c` (domain-un-clipped SphericalSurface/CylindricalSurface —
  both finite by construction here) — reanchor OFF declines, reanchor ON crosses to ONE closed loop
  whose densely-sampled nodes all lie on the OCCT `GeomAPI_IntSS` locus AND on both surfaces within
  tol (crossResid ≤ onSurfTol).
- [x] 4.2 `pairDeepNearTangentHonestDeclineS4c` — below the extended floor native declines
  (nearTangentCrossed==0, no Closed loop, nearTangentGaps≥1) while OCCT reports a locus.
- [ ] 4.3 Run `run-sim-native-ssi-marching` on the booted simulator; confirm all prior cases frozen
  and the two new cases pass.

## 5. Structural + finalize
- [x] 5.1 `git diff src/native` OCCT-free and additive; tessellator / boolean / blend UNTOUCHED;
  `cc_*` unchanged.
- [x] 5.2 Update `openspec/MOAT-ROADMAP.md` M1 status: near-tangent breadth floor extended
  (0.17 → 0.14) with the sharpened next blocker.
- [x] 5.3 `openspec validate --strict moat-m1d-ssi-deep-tail`.
