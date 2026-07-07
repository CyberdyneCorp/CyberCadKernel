# Tasks — moat-m5-healing-robustness

Order: additive result-type fields → the bounded bridging pass → the guarded
`heal.cpp` branch → host tests → sim OCCT-parity → docs. All new code stays
**OCCT-free** and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::heal`. **No `cc_*` ABI change** (healing is internal). The
tessellator is **not** modified. The primary weld tolerance is **never** widened; the
bridging budget is opt-in (default `0.0`) so the landed slice is byte-identically
preserved. Keep `bridgeGaps` in the systems band (`≤ 25`) via named helpers.

## 1. Additive result types (`src/native/heal/heal_result.h`)

- [x] 1.1 Add `double gapBridgeBudget = 0.0;` to `HealOptions` with the doc contract:
      model units, the max gap *beyond* `tolerance` the healer may bridge; `0.0`
      disables bridging (behaviour identical to the landed slice); never widens the
      primary weld.
- [x] 1.2 Add `int nBridgedGaps = 0;` and `double maxBridgedGap = 0.0;` to
      `HealMetrics` (corners closed by the bridging pass; largest gap bridged — honestly
      `≤` the effective bound).
- [x] 1.3 Append `GapBeyondBudget` to `UnhealedReason` (a gap that exceeds the
      budget / local-feature cap — distinct from the in-tolerance `OpenShell` and the
      no-budget `GapBeyondTolerance`). Enum value APPENDED (additive; internal C++).

## 2. Bounded bridging pass (`src/native/heal/gap_bridge.h`, NEW, header-only)

- [x] 2.1 SPDX `Apache-2.0`; OCCT-free; consumes `native/{math,topology}` + the sew
      types only. File header states the bound + the honest-out + the asymptotic-tail
      caveat. (Consumes the FaceLoop soup type; the pass rewrites the soup and re-uses
      the existing `sew()` unchanged, so it needs no SewResult internals.)
- [x] 2.2 `effectiveBound(corner, budget)` → `min(budget, kLocalFeatureFraction *
      shortestIncidentEdgeLen(corner))`, `kLocalFeatureFraction = 0.25` (documented
      constant). Only gaps in `(tol, effectiveBound]` are bridgeable.
- [x] 2.3 Deterministic cross-face matching: an unpaired corner snaps onto its nearest
      DIFFERENT-face corner when that partner is within-tol paired (established
      geometry); a symmetric seam whose two unpaired sides are mutually-nearest snaps
      both to their midpoint. Ambiguous (unpaired, non-mutual) is left unbridged. No
      corner is moved twice; decisions read the original geometry (order-independent).
- [~] 2.4 `wouldBeNonManifold` — NOT implemented as a separate pre-check. A bridge that
      would create a non-manifold edge fails the MANDATORY self-verify gate (watertight
      requires every mesh edge used by exactly two triangles → `Unhealed{SelfVerifyFailed}`),
      which is already authoritative. A standalone predicate would be untested/dead code on
      every fixture (the diagnosis flagged `NonManifold` as never-emitted); relying on
      self-verify is the honest choice and matches "self-verify unchanged, still authoritative".
- [x] 2.5 `bridgeGaps(soup, tol, budget) -> BridgeResult{ std::vector<FaceLoop> soup;
      bool applied; int nBridged; double maxBridged; }` — rewrite the bridged corners in
      the soup + refresh Newell normals; heal.cpp re-runs `sew()` on it (which recomputes
      `boundaryEdges` / `maxResidualGap`). Cognitive complexity kept in-band via `detail`
      helpers (`flattenCorners`, `effectiveBound`, `nearestCrossFace`, `isPaired`).

## 3. Guarded orchestration branch (`src/native/heal/heal.cpp`)

- [x] 3.1 In the `sr.boundaryEdges > 0` block, when `opts.gapBridgeBudget > 0.0` run
      `bridgeGaps(...)`, record `m.nBridgedGaps` / `m.maxBridgedGap`, and adopt the
      re-welded `SewResult` (`sr = sew(br.soup, tol)`). When `budget == 0.0` the block is
      unchanged (dead-guarded).
- [x] 3.2 If boundary edges STILL survive, decline with `GapBeyondBudget` (whenever
      `budget>0` and `maxResidualGap > tol` — the effective, feature-capped bound
      governs, so a feature-cap refusal reports `GapBeyondBudget` too), else the existing
      `GapBeyondTolerance` / `OpenShell` reasons. Input returned UNCHANGED.
- [x] 3.3 On a successful bridge fall through to the UNCHANGED orient + assemble +
      **mandatory self-verify** tail; a bridged candidate that fails self-verify is
      `Unhealed{SelfVerifyFailed}` (no faked closure).

## 4. Host gate (no OCCT) — `native-healing` host suite

- [x] 4.1 In-band fixture: cube soup, one seam gap `g` with `tol < g ≤ budget` and
      `g < ¼·edge` → `Healed`, watertight + valid, `nBridgedGaps > 0`,
      `maxBridgedGap ≈ g`, correct volume.
- [x] 4.2 Out-of-budget fixture: same soup with `g > budget` →
      `Unhealed{GapBeyondBudget}`, `maxResidualGap ≈ g`, input unchanged.
- [x] 4.3 Default-off fixture: the in-band soup with `budget == 0.0` still `Unhealed`
      (proves no primary-weld weakening; no landed-slice regression).
- [x] 4.4 Feature-cap fixture: `budget > ¼·edge` but a bridge would collapse a real
      edge → refused (declined, not welded) — the cap governs, not the caller's budget.

## 5. Sim native-vs-OCCT parity — `tests/sim/native_heal_parity.mm`

- [x] 5.1 In-band pair: OCCT `BRepBuilderAPI_Sewing` at sewing tolerance `≈ budget` on
      the same soup → both watertight, volume within tol, compared at the
      `cybercad::native::heal` C++ boundary (no `cc_*` call).
- [x] 5.2 Out-of-budget pair: native `Unhealed{GapBeyondBudget}` while OCCT at that
      tolerance leaves the shell open → parity of decline; engine falls through to
      `ShapeFix`.
- [x] 5.3 Confirm `run-sim-native-heal.sh` still passes the four landed fixtures
      (bridging default-off) — total unchanged + the new pair.

## 6. Docs

- [x] 6.1 Update `src/native/heal/native_heal.h` scope block: add the opt-in
      bounded bridging band to WHAT-THIS-HEALS, keep the asymptotic-tail caveat.
- [x] 6.2 Tick the M5 first-slice line in `openspec/MOAT-ROADMAP.md` (bounded slice
      landed; the beyond-budget / arbitrary-broken remainder stays asymptotic).

## 7. Validate & archive

- [x] 7.1 `openspec validate moat-m5-healing-robustness --strict`.
- [x] 7.2 After merge, `openspec archive moat-m5-healing-robustness`.
