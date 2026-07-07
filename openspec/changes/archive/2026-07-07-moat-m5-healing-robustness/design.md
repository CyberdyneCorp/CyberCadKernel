# Design — moat-m5-healing-robustness (bounded beyond-tolerance gap bridging)

## Context

MOAT stage **M5** is shape-healing robustness beyond the landed slice, framed by the
roadmap as *bounded first slices with an asymptotic tail*. The landed
`cybercad::native::heal` module (clean-room, OCCT-free) heals the
coincident-within-tolerance / degenerate / orientation family and **honestly declines**
everything else to the OCCT `ShapeFix` oracle. This change picks **one** concrete
declined defect and heals it robustly under a stated bound.

**Chosen defect (roadmap option a): beyond-tolerance near-miss gap.** `heal.cpp`
declines any soup whose primary weld at `tolerance` leaves a boundary edge with
`maxResidualGap > tolerance` (reason `GapBeyondTolerance`). The commonest real instance
is a **seam near-miss**: two faces that *should* share an edge but whose corners a STEP
exporter (or a prior native op) wrote a hair farther apart than the caller's weld
tolerance. OCCT `BRepBuilderAPI_Sewing` closes these by sewing at a *larger* tolerance;
we earn the same win natively under an **explicit, bounded, opt-in budget**, never by
silently widening the primary weld.

Why this defect first: it is the healer's single clearest decline, it is **bounded**
(a near-miss band, not arbitrary topology repair), it maps one-to-one onto the OCCT
sewing oracle for parity, and it composes with the existing pipeline as a *second weld
pass* — no new geometry kernel, just a guarded, capped re-weld of the corners the first
pass left unpaired.

## Goals / Non-Goals

**Goals**
- An **opt-in** (`HealOptions.gapBridgeBudget > 0`) bridging pass that closes a
  boundary near-miss whose gap lies in the band `(tolerance, budget]`, subject to a
  hard local-feature-size cap, producing a watertight + valid solid that matches OCCT
  `BRepBuilderAPI_Sewing` at the same tolerance.
- **Default-off**: `gapBridgeBudget == 0.0` ⇒ the pass is a no-op ⇒ `healShell` is
  byte-identical to the landed slice ⇒ the four `run-sim-native-heal.sh` fixtures and
  every host test pass unchanged.
- An **honest out**: a gap beyond the effective bound stays `Unhealed`
  (`GapBeyondBudget`) with the measured residual; the input is returned unchanged;
  nothing is faked.
- The **mandatory self-verify** gate is unchanged and still authoritative: a bridged
  candidate that does not tessellate watertight with positive volume is discarded.

**Non-Goals (stay OCCT `ShapeFix`'s moat — the asymptotic tail)**
- Bridging a gap `>` the budget or `>` the local-feature cap (a real hole / a missing
  face / geometry-scale separation).
- Any repair that is not a near-miss seam: pcurve reconstruction, self-intersecting-wire
  repair, freeform re-approximation, non-manifold (3+-face-edge) repair, T-junction
  splitting — all remain declined exactly as today.
- Auto-widening the *primary* weld tolerance, or bridging without an explicit budget.
- Any `cc_*` ABI change; any tessellator change; any OCCT include under `src/native/**`.

## The bound (why it is safe, and provably not a fabricated closure)

A bridge is legitimate only when it welds two corners that geometrically *are* the same
seam point written slightly apart — never two distinct features. Three layers enforce
this; a bridge that fails any layer is refused and the heal declines honestly.

1. **Explicit opt-in budget.** `gapBridgeBudget` (model units) caps the absolute gap the
   pass may close, *beyond* `tolerance`. It is a separate, caller-supplied bound; the
   primary weld tolerance is never touched. `budget == 0` ⇒ no bridging.

2. **Local-feature-size cap.** For each candidate corner the effective bridge tolerance
   is `min(gapBridgeBudget, kLocalFeatureFraction · localEdgeLen)`, where `localEdgeLen`
   is the shortest edge incident to that corner and `kLocalFeatureFraction = 0.25`
   (documented constant). A gap comparable to a real edge can therefore **never** be
   bridged — the pass only closes seams narrower than a quarter of the smallest local
   feature. This is the geometric guarantee that bridging cannot collapse distinct
   topology, independent of how large a caller sets `budget`.

3. **Mutual-nearest, cross-face, non-manifold-safe matching.** Only corners left
   **unpaired** by the primary weld are eligible. A corner `a` may bridge to corner `b`
   only when: (i) `b` is `a`'s nearest eligible corner AND `a` is `b`'s nearest
   (a **mutual** nearest pair — deterministic, no corner welded twice); (ii) `b` lies on
   a **different face** than `a` (bridging a seam between faces, never folding one face
   onto itself); and (iii) merging them introduces **no** edge referenced by 3+ faces
   (bridging that would create a non-manifold edge is refused → `Unhealed{NonManifold}`).

4. **Mandatory self-verify (unchanged).** After bridging + orientation flood-fill, the
   candidate must tessellate watertight with positive enclosed volume across the
   deflection ladder (`self_verify.h`). Failure ⇒ `Unhealed{SelfVerifyFailed}`, input
   unchanged. The healer never trusts its own bookkeeping to claim a closure.

**Honest residual.** If any boundary corner has no in-bound mutual partner, or its gap
exceeds the effective bound, the heal declines `GapBeyondBudget` with
`maxResidualGap = ` the largest surviving gap. The bridged band is reported separately:
`nBridgedGaps` (corners closed) and `maxBridgedGap` (largest gap closed, `≤` the
effective bound).

## Asymptotic-tail caveat

This slice closes only the **bounded near-miss band** `(tolerance,
min(budget, ¼·localFeature)]` via a stable mutual-nearest matching. Arbitrary
beyond-tolerance repair — real holes needing a synthesized face, gaps at geometry scale,
one-sided/greedy matchings, non-seam defects — remains the decades-deep OCCT `ShapeFix`
moat and is **declined honestly**. Per `MOAT-ROADMAP.md` M5, a first robust slice is
bounded; completeness against arbitrary broken industrial B-rep is **asymptotic** —
re-earned only incrementally, never claimed here.

## Module shape

```
src/native/heal/
  gap_bridge.h    // NEW: bounded mutual-nearest bridging of unpaired boundary corners
  heal_result.h   // + HealOptions.gapBridgeBudget; + HealMetrics.nBridgedGaps/maxBridgedGap;
                  //   + UnhealedReason::GapBeyondBudget   (all ADDITIVE, internal C++)
  heal.cpp        // + one guarded branch: when boundaryEdges survive AND budget>0,
                  //   run bridgeGaps(), re-tally, re-orient, re-verify; else unchanged
  tolerant_sew.h  // reused: the primary weld + residual measurement (unchanged)
  self_verify.h   // reused: the mandatory watertight + positive-volume gate (unchanged)
```

`gap_bridge.h` consumes only `src/native/{math,topology}` + the sew types; it takes the
`SewResult`'s unpaired boundary corners (keyed by the same shared-vertex-node identity
the `EdgePool` uses), computes mutual-nearest cross-face pairs within the effective
bound, and rewrites the affected faces' shared-vertex loops so the re-sew shares those
edge nodes. It adds no OCCT include and no `cc_*` surface.

### `heal.cpp` control flow (additive branch only)

```cpp
if (sr.boundaryEdges > 0) {
  if (opts.gapBridgeBudget > 0.0) {
    BridgeResult br = bridgeGaps(sr, clean, tol, opts.gapBridgeBudget);  // NEW
    if (br.applied) {
      m.nBridgedGaps = br.nBridged;
      m.maxBridgedGap = br.maxBridged;
      sr = br.sew;                 // re-welded soup, boundary edges recomputed
    }
  }
  if (sr.boundaryEdges > 0) {      // still open after (optional) bridging
    const UnhealedReason why =
        opts.gapBridgeBudget > 0.0 && sr.maxResidualGap > opts.gapBridgeBudget
            ? UnhealedReason::GapBeyondBudget
            : (sr.maxResidualGap > tol ? UnhealedReason::GapBeyondTolerance
                                       : UnhealedReason::OpenShell);
    return unhealed(shape, why, sr.maxResidualGap, m);
  }
}
// falls into the UNCHANGED orient + assemble + self-verify tail
```

With `gapBridgeBudget == 0.0` this is the identical code path as today — the new block
is dead-guarded off, so the landed 4/4 is untouched.

## Verification

**Host gate (no OCCT).** New fixture pair added to the `native-healing` host suite:
- *In-band heals.* A cube face soup with one seam pulled apart by `g` with
  `tolerance < g ≤ budget` and `g < ¼·edge`. `healShell(soup, {tol, budget})` returns
  `Healed`, watertight + valid, `nBridgedGaps > 0`, `maxBridgedGap ≈ g`, correct volume.
- *Out-of-budget declines.* The same soup with `g > budget` returns
  `Unhealed{GapBeyondBudget}`, `maxResidualGap ≈ g`, input unchanged.
- *Default-off no-op.* The in-band soup with `budget == 0` still returns `Unhealed`
  (proving no primary-weld weakening and no landed-slice regression).
- *Feature-cap guard.* A soup where `budget > ¼·edge` but a bridge would collapse a real
  edge is refused (declined, not welded) — the cap, not the caller's budget, governs.

**Sim native-vs-OCCT gate.** Matching pair in `tests/sim/native_heal_parity.mm`:
OCCT `BRepBuilderAPI_Sewing` at sewing tolerance `≈ budget` on the same soup. In-band:
both watertight, volume within tol at the `cybercad::native::heal` C++ boundary (no
`cc_*` call). Out-of-budget: native `Unhealed{GapBeyondBudget}` while OCCT at that
tolerance leaves the shell open → parity of decline; the engine falls through to
`ShapeFix`.

**Regression.** `run-sim-native-heal.sh` still 4/4 (bridging default-off); the numsci
host regression and CTest suites unchanged.

## Cognitive complexity

`bridgeGaps` is the one dense function (systems band, target `≤ 25`): a corner-index
build, the mutual-nearest scan, and the non-manifold check. Each is factored into a
named helper (`effectiveBound`, `mutualNearest`, `wouldBeNonManifold`, `rewriteLoops`)
so the orchestrator stays a flat guarded pipeline; the `heal.cpp` branch adds `< 5`.

## Alternatives considered

- **Widen the primary weld tolerance directly.** Rejected — it violates the
  never-weaken-tolerance discipline and would silently reclassify legitimate holes as
  closures. The bridging pass is a *separate, explicitly-bounded, opt-in* second pass
  that leaves the primary weld semantics intact.
- **Greedy nearest-neighbour matching.** Rejected — order-dependent and can weld a
  corner to a non-partner. Mutual-nearest is deterministic and symmetric.
- **Sliver-triangle repair / self-intersecting-wire repair (options b/c).** Deferred —
  each is a larger slice; option (a) is the healer's clearest single decline with a
  clean OCCT-sewing oracle, so it is the right first M5 slice.
