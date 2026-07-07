# Proposal — moat-m7t-guided-sweep-orientation (MOAT M7-tail)

## Why

M7a **honest-declined** the orientation-constraining guided sweep. Its recorded reason
was that "OCCT's guide surface is NOT a per-station rigid in-plane rotation": a native
"rigid guide-aimed up-axis" reproduced OCCT's swept volume (`volRel ~4e-4`) but was
spatially ~7% wrong (`bboxΔ 0.54`), invariant under station densification — a law
mismatch, not a sampling artifact. The lesson banked was that a native builder whose
self-verify checks only volume/watertightness would WRONGLY ACCEPT that spatially-wrong
solid.

This change is the **M7-tail retry**. It does NOT re-attempt the discredited rigid
per-station-rotation law (that was a measured trap). Instead it targets the law that
`BRepOffsetAPI_MakePipeShell::SetMode(guideWire)` (single-arg, **NoContact**) actually
realizes, reverse-engineered from the OCCT oracle source
(`GeomFill_GuideTrihedronPlan::D0` + `GeomFill_LocationGuide::D0`, `rotation == false`
path). It scopes the FIRST tractable slice — a **straight spine** — and gates acceptance
on a **spatial (bbox/Hausdorff)** metric, not volume alone, so the M7a trap cannot recur.

### Diagnosis — what OCCT's `SetMode(guideWire)` NoContact law actually is

Read from the oracle source (not assumed). For a spine parameter with spine point `P`
and spine Frenet tangent `T`, `GeomFill_GuideTrihedronPlan::D0` computes:

- `Pprime` = the guide point lying in the plane through `P` **perpendicular to `T`**,
  found by root-finding the guide parameter `W` (`GeomFill_PlanFunc`). This is the
  **perpendicular-plane correspondence** — NOT a guide-parameter-fraction correspondence.
- `Normal N = normalize(Pprime − P)` — the section's aim axis points from the spine to
  that guide point.
- `BiNormal B = T × N`.

`GeomFill_LocationGuide::D0` (the `rotation == false` branch taken by NoContact
`SetMode(guide)`) then places the section with translation `V = P` and rotation
`M = cols(N, B, T)` — a **pure per-station rigid frame, no scaling, no per-station angle
root-find** (the angle root-find and homothety are the *contact*-mode branch only; the
guide BSpline approximation is `#ifdef DRAW` only). So the section's local `(x, y)` map to
`(N, B)` and the profile is placed rigidly at each station.

**The M7a trap, precisely.** OCCT's law IS a per-station rigid frame after all — but the
frame's in-plane angle is set by the **perpendicular-plane** correspondence (which guide
point the aim axis targets), not by the guide-parameter-fraction "aim" the M7a prototype
used. On a rotating/offset guide those two correspondences select different guide points,
so the section is rigidly rotated to a different in-plane angle at each station — which
preserves cross-section area (hence near-correct swept volume) but shifts the solid in
space (the `bboxΔ 0.54`). M7a's "it is not a per-station rigid rotation" framing was the
misdiagnosis; the corrected law is a per-station rigid frame with the CORRECT correspondence.

### Host-analytic evidence (GATE a, no OCCT) — measured this session

A native build of the reverse-engineered law on a straight `+Z` spine (host unit tests,
NO OCCT linked; `tests/native/test_native_sweep.cpp`, 21/21) confirms:

- **Offset (non-rotating) guide** → constant `N` → identity frame → 2-station prism:
  volume `80.0` EXACT vs `4ab·H` (`relErr 0`), 6 faces, watertight, bbox `x[-2,2] y[-1,1]
  z[0,10]` exact.
- **Rotating (90°) guide** → watertight at every deflection; swept volume converges to
  within `3e-2` of `4ab·H` (the ruled-band discretization deficit against the continuous
  guide, shrinking with deflection); **the bbox grows to the rotated-rectangle union
  half-extent `√5 ≈ 2.2361` in BOTH x and y (to `5e-3`)** — the section orientation is
  demonstrably guide-driven, NOT the axis-aligned `[-2,2]×[-1,1]` a fixed/wrong frame gives.
- **Spatially load-bearing**: the rotating footprint (`√5`) differs from the orientation-
  blind axis-aligned footprint (`x=2, y=1`) — so a bbox gate is meaningful and a volume-only
  gate (the M7a mistake) would be blind to the orientation.

*(The prior tail summary's `2.2e-16 rad` angle match, exact `4.0` quadratic-convergence
ratio, and `2.16` bbox spread were from a probe not reproduced in this session and are
superseded by the measured figures above — no fabricated numbers are carried forward.)*

Gate (a) passes: the law is closed-form-correct AND its native solid is watertight and
spatially correct on the straight-spine slice. GATE (b) — the on-simulator native-vs-OCCT
parity — is the **binding acceptance gate** and **PASSES** (19/19,
`tests/sim/native_sweep_parity.mm`): native matches `MakePipeShell + SetMode(guide,
CE=false, NoContact)` on volume/area/watertight/topology AND on **bbox corner delta**
(offset `1.0e-07`, rotating `2.49e-04`, both `< 5e-2`). The native path ships guarded by a
watertight self-verify + OCCT fallback; a curved-spine / degenerate guide declines to OCCT.

## What Changes

1. **A NEW ADDITIVE facade `cc_guided_orient_sweep`** in `include/cybercadkernel/cc_kernel.h`
   + the C ABI: `cc_guided_orient_sweep(profileXY, profileCount, pathXYZ, pathCount,
   guideXYZ, guideCount)`. This is DISTINCT from the existing scale-splay `cc_guided_sweep`
   (which the M7a trap explicitly forbids repurposing): the new entry constrains section
   **orientation** by the guide (NoContact, no scaling), matching `MakePipeShell +
   SetMode(guideWire)`. Purely additive — no existing signature, POD layout, or enum
   changes; `cc_guided_sweep` and every shipped op stay byte-identical.

2. **A real OCCT oracle** `OcctEngine::guided_orient_sweep` in `src/engine/occt` built on
   `BRepOffsetAPI_MakePipeShell` + `SetMode(guideWire)` (single-arg, NoContact) — the
   ORACLE the native builder self-verifies against and the FALLBACK target. OCCT stays
   confined to `src/engine/occt`.

3. **An OCCT-free native builder** under `src/native/construct` that, for a **straight
   spine only**, samples the per-station rigid frame via the perpendicular-plane
   correspondence (`N = normalize(Pprime − P)`, `B = T × N`, no scaling) and assembles the
   ruled tube via the existing station→band assembler. `src/native/**` keeps **0 OCCT
   includes** and references no `IEngine`/`EngineShape`/OCCT type.

4. **Engine dispatch + SPATIAL self-verify** in `NativeEngine::guided_orient_sweep`: keep
   the native result ONLY IF it is watertight, volume-positive, AND matches the OCCT oracle
   within a **bbox/Hausdorff** tolerance (not volume alone); otherwise forward the SAME
   arguments to the OCCT oracle and return that solid. A non-straight spine, a guide whose
   perpendicular-plane intersection is empty/ambiguous, or any spatial-tolerance miss →
   NULL → OCCT. The `IEngine` base gets a default returning `engine_unsupported`.

5. **Two verification gates**: HOST analytic (`tests/native/`, the closed-form volume +
   orientation probe, NO OCCT) and SIM native-vs-OCCT parity
   (`tests/sim/native_sweep_parity.mm`, `cc_guided_orient_sweep` vs `MakePipeShell +
   SetMode(guideWire)` on volume/area/watertight/topology **AND bbox/Hausdorff**, plus
   honest-decline fixtures that must delegate to OCCT).

6. **Docs**: this delta adds the `cc_guided_orient_sweep` requirement to `native-construction`;
   `MOAT-ROADMAP.md` M7 records the sharpened diagnosis (the true law + the M7a
   correspondence trap) and the straight-spine-slice outcome.

## Impact

- Affected specs: `native-construction` (this delta — one ADDED requirement).
- Affected code (all additive; no behavior change to any existing op):
  `include/cybercadkernel/cc_kernel.h` (+ C ABI), `src/facade/cc_kernel.cpp`,
  `src/engine/IEngine.h` (default), `src/engine/native/*` (native builder wiring + SPATIAL
  self-verify), `src/engine/occt/*` (new `MakePipeShell + SetMode(guideWire)` oracle),
  `src/native/construct/*` (straight-spine guide-frame builder), `tests/native/*` and
  `tests/sim/native_sweep_parity.mm`. No existing `src/native/**` op and no tessellator
  changed.
- OCCT remains the oracle and the fallback; it is never removed. `cc_guided_sweep` (the
  scale-splay op) stays byte-identical. If the on-sim SPATIAL gate is not met, the native
  path DECLINES and the change ships as a measured decline with the OCCT path unchanged —
  a first-class outcome.
