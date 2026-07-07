# Proposal — moat-m5-healing-robustness

## Why

MOAT stage **M5** (`openspec/MOAT-ROADMAP.md`) attacks the shape-healing robustness
that the landed first slice (`native-healing`) deliberately deferred. Today
`cybercad::native::heal::healShell` heals the *coincident-within-tolerance /
degenerate / orientation* family exactly (tolerant sew + vertex/tolerance unify +
degenerate removal + orientation flood-fill), and it **declines everything harder** —
routing it to the OCCT `ShapeFix` oracle. Its single clearest decline is
`heal.cpp`'s beyond-tolerance branch:

```cpp
if (sr.boundaryEdges > 0) {
  const UnhealedReason why =
      sr.maxResidualGap > tol ? UnhealedReason::GapBeyondTolerance
                              : UnhealedReason::OpenShell;
  return unhealed(shape, why, sr.maxResidualGap, m);
}
```

Any face soup whose shared boundary is pulled apart by even a hair **more** than the
weld tolerance (`tolerance < gap`) leaves a boundary edge and is declined
`GapBeyondTolerance`, even when the two sides are an obvious near-miss seam that OCCT
`BRepBuilderAPI_Sewing` closes trivially at a slightly larger sewing tolerance. This
is the M5 roadmap's option (a): **beyond-tolerance gap bridging** — "a gap slightly
above the current weld tol, closed by a tolerance-widening pass with a bounded
budget."

This change adds exactly that ONE bounded slice: an **opt-in, budget-bounded
near-miss gap-bridging pass**, self-verified against the same watertight + positive
volume gate, matched to OCCT `BRepBuilderAPI_Sewing` at the same tolerance — with an
**honest out** for any gap beyond the budget and an explicit **asymptotic-tail
caveat** for the arbitrary-broken-B-rep remainder that stays OCCT's moat.

The non-negotiable disciplines carry over unchanged: `src/native/**` stays
**OCCT-free** (OCCT is the sim oracle only); the `cc_*` ABI is untouched (healing is
internal, no entry point); the tessellator is untouched; and the tolerance is
**never** auto-weakened beyond the stated, explicit budget — a residual past the
budget is reported, not faked.

## What Changes

1. **Opt-in budget on `HealOptions`.** Add `double gapBridgeBudget = 0.0` (model
   units). **Default `0.0` disables bridging**, so `healShell` behaves byte-identically
   to the landed slice — the four existing `run-sim-native-heal.sh` fixtures and every
   host test are unchanged (no regression). When `> 0` it is the maximum gap, *beyond*
   `tolerance`, the healer may bridge.

2. **A bounded near-miss gap-bridging pass** (`src/native/heal/gap_bridge.h`,
   OCCT-free, header-only). After the primary weld at `tolerance` leaves boundary
   corners unpaired, the pass welds an unpaired corner to its **mutual-nearest**
   unpaired partner **on a different face**, but ONLY when the gap is within the
   effective bound `min(gapBridgeBudget, kLocalFeatureFraction · localEdgeLen)` — a
   hard **local-feature-size cap** (a fraction of the shortest incident edge) so a
   bridge can never collapse geometry comparable to a real feature. The re-welded
   soup then re-runs orientation + the **mandatory self-verify**; a candidate that
   does not tessellate watertight with positive volume is still `Unhealed`.

3. **Honest out + measured band.** A new additive `UnhealedReason::GapBeyondBudget`
   distinguishes a gap that exceeds the budget/feature cap from the in-tolerance
   `OpenShell`. New additive `HealMetrics` fields `int nBridgedGaps` and
   `double maxBridgedGap` report how many corners were bridged and the largest bridged
   gap (honestly `≤` the effective bound); `maxResidualGap` still carries any surviving
   gap when the heal declines.

4. **OCCT oracle parity + honest decline.** The sim gate runs OCCT
   `BRepBuilderAPI_Sewing` at sewing tolerance `≈ gapBridgeBudget`: the in-band fixture
   heals watertight in both with matching volume; the out-of-budget fixture is
   `Unhealed{GapBeyondBudget}` natively while OCCT at that tolerance leaves it open
   (parity of decline). The host gate proves both, plus the default-off no-op.

**Additive only.** No `cc_*` entry point, signature, or POD changes; no tessellator
change; `HealOptions` / `HealMetrics` / `UnhealedReason` are internal C++ types
extended additively. `src/native/**` includes no OCCT header.

## Impact

- **Specs:** `native-healing` — two ADDED requirements (the bounded bridging pass; the
  metrics + `GapBeyondBudget` honest-out) and one MODIFIED requirement (carve the
  opt-in bounded band out of the "beyond-tolerance repair is out of scope" clause and
  add the OCCT-sewing-at-budget parity to the verification gate).
- **Code:** additive `src/native/heal/gap_bridge.h`; small additive fields on
  `heal_result.h` (`HealOptions.gapBridgeBudget`, `HealMetrics.nBridgedGaps` /
  `.maxBridgedGap`, `UnhealedReason::GapBeyondBudget`); one new guarded branch in
  `heal.cpp` (runs only when `gapBridgeBudget > 0` and boundary edges survive).
- **Tests:** a 5th host fixture pair (in-band heals / out-of-budget declines / default-off
  no-op) in the `native-healing` host suite; a matching pair in
  `tests/sim/native_heal_parity.mm` vs OCCT sewing at the budget.
- **No impact:** `cc_*` ABI, the tessellator, the CyberCad app, and the four landed
  heal fixtures (bridging is default-off).
