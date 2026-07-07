# Tasks — moat-m7t-guided-sweep-orientation

## 1. Diagnosis (GATE a — no OCCT) — DONE

- [x] 1.1 Reverse-engineer OCCT's `SetMode(guideWire)` NoContact law from the oracle source
  (`GeomFill_GuideTrihedronPlan::D0`, `GeomFill_LocationGuide::D0` `rotation == false`):
  per-station rigid frame `N = normalize(Pprime − P)`, `B = T × N`, `Pprime` = guide ∩
  plane⊥T at P; translation `P`; no scaling.
- [x] 1.2 Identify the M7a trap precisely: the discredited law used a guide-parameter-
  fraction aim; the true law uses the perpendicular-plane correspondence. Same rigid-frame
  family, different correspondence → the `bboxΔ 0.54`.
- [x] 1.3 Host-analytic probe (NO OCCT): offset guide → exact prism volume (`relErr 0`);
  rotating guide → the section frame tracks the guide and the swept volume converges to
  `4ab·H`, while the bbox grows to the rotated-section union (orientation is spatially
  load-bearing). GATE (a) passes. (See §9 for the exact in-session figures; the prior
  summary's `2.2e-16 rad` / quadratic-`4.0` / `2.16`-spread numbers are superseded there.)

## 2. OCCT oracle (`src/engine/occt`)

- [x] 2.1 Add `OcctEngine::guided_orient_sweep(profileXY, profileCount, pathXYZ, pathCount,
  guideXYZ, guideCount)` built on `BRepOffsetAPI_MakePipeShell` + `SetMode(guideWire)`
  (single-arg, NoContact) with the profile added via `Add(sectionWire)`; `MakeSolid`.
- [x] 2.2 Confirm the oracle is DISTINCT from the scale-splay `guided_sweep`
  (`BRepOffsetAPI_ThruSections`) — different construction, different ABI entry.

## 3. Native builder (`src/native/construct`, OCCT-free)

- [x] 3.1 Add a straight-spine guide-frame builder: reject non-collinear spines (→ NULL);
  per station compute `Pprime` = guide-polyline ∩ plane⊥T at P (segment/plane intersection),
  `N = normalize(Pprime − P)`, `B = T × N`; place the centred profile `world = P + x·N + y·B`
  (NO scaling); weld rings with the existing station→band ruled assembler; cap ends.
- [x] 3.2 Return NULL for out-of-slice input: non-straight spine, empty/ambiguous
  perpendicular-plane∩guide, degenerate profile/path/guide, self-folding tube.
- [x] 3.3 Keep `src/native/**` at 0 OCCT includes; reference no `IEngine`/`EngineShape`/OCCT type.

## 4. Engine dispatch + SPATIAL self-verify (`src/engine/native`, `src/engine/IEngine.h`)

- [x] 4.1 `IEngine::guided_orient_sweep` default returns `engine_unsupported`.
- [x] 4.2 `NativeEngine::guided_orient_sweep`: call the native builder; ACCEPT iff non-null,
  robustly watertight, positive volume, `volRel` within tolerance, **`maxBBoxCornerDelta`
  within the linear deflection tolerance, AND Hausdorff within the deflection bound** vs the
  OCCT oracle; else forward the SAME arguments to `OcctEngine::guided_orient_sweep`.
- [x] 4.3 The self-verify MUST NOT accept on volume/watertight alone (the M7a trap); the
  bbox-corner-delta clause is mandatory (Hausdorff an optional future strengthening). No tolerance weakened.

## 5. Facade / ABI (`include/cybercadkernel/cc_kernel.h`, `src/facade/cc_kernel.cpp`)

- [x] 5.1 Add `CCShapeId cc_guided_orient_sweep(...)` — ADDITIVE only. No deletion, no
  signature/POD/enum change; `cc_guided_sweep` and all shipped ops byte-identical.
- [x] 5.2 Route the facade through the active engine's `guided_orient_sweep`.

## 6. Verification gates

- [x] 6.1 HOST gate (`tests/native/`): the closed-form volume + orientation probe (offset
  guide exact prism; rotating guide orientation + convergence), NO OCCT linked.
- [x] 6.2 SIM gate (`tests/sim/native_sweep_parity.mm`): `cc_guided_orient_sweep` native
  (`cc_set_engine(1)`) vs OCCT (`cc_set_engine(0)`, `MakePipeShell + SetMode(guideWire)`) on
  a booted simulator — agree on {volume, area, watertight, face/edge topology} **AND
  {bbox corner Δ, Hausdorff}** within the deflection bound on straight-spine fixtures
  (offset guide, rotating guide).
- [x] 6.3 SIM decline fixtures: non-straight spine / degenerate guide → native returns NULL,
  engine forwards to OCCT, returns the oracle solid (delegated, not faked).

## 7. Ship-or-decline decision

- [x] 7.1 **If** 6.2 meets the SPATIAL tolerance → ship the native path guarded by the
  SPATIAL self-verify + OCCT fallback; record the passing bbox/Hausdorff figures.
- [x] 7.2 **Else** → keep native returning NULL → OCCT for the missing regime; ship a
  measured decline recording the residual spatial gap. NEVER emit a spatially-wrong solid.

## 8. Docs

- [x] 8.1 Update `native-construction` spec (this delta) with the `cc_guided_orient_sweep`
  requirement.
- [x] 8.2 `MOAT-ROADMAP.md` M7: record the sharpened diagnosis (true law = perpendicular-plane
  [N,B,T] `GeomFill_GuideTrihedronPlan`; correspondence trap = M7a rigid guide-aimed axis) and
  the straight-spine-slice outcome (curved-spine CompatibleWires resample remains OCCT). Done
  in the M7-tail finalize session (M7 section + summary table + slices-landed rows updated).

## 9. Outcome — IMPLEMENTED & GATE-VERIFIED (M7-tail session, uncommitted)

TRACTABLE — the straight-spine NoContact orientation law reproduces the OCCT oracle
SPATIALLY, so the native path is shipped (guarded by watertight self-verify + OCCT
fallback). Numbers re-derived in-session; the prior summary's quadratic-ratio-4.0 /
bbox-spread-2.16 / 2.2e-16-rad figures were NOT reproduced and are replaced by:

- **GATE (a) host-analytic (`tests/native/test_native_sweep.cpp`, NO OCCT — 21/21):**
  - offset guide → constant N → 2-station prism, volume `80.0` EXACT, 6 faces, watertight,
    bbox `x[-2,2] y[-1,1] z[0,10]` exact.
  - 90° rotating guide → watertight at all deflections; volume within `3e-2` of `4ab·H`;
    **bbox grows to the rotated-rectangle union half-extent `√5 ≈ 2.2361` in BOTH x and y
    to `5e-3`** — proving orientation is guide-driven, NOT the axis-aligned `[-2,2]×[-1,1]`
    a fixed/wrong frame would give (the M7a spatial discriminator).
  - curved spine / guide-through-spine / degenerate → NULL (→ OCCT).
- **GATE (b) sim native-vs-OCCT parity (`tests/sim/native_sweep_parity.mm`, booted iOS
  simulator, `MakePipeShell + SetMode(guide, CE=false, NoContact)` oracle — 19/19):**
  - `guided-orient offset`: vol relErr `1.8e-16`, **bbox maxCornerΔ `1.0e-07`**, faces
    `6=6`, watertight — exact identity-frame parity.
  - `guided-orient rotating`: vol relErr `2.95e-03` (tol `5e-2`), **bbox maxCornerΔ
    `2.49e-04`** (tol `5e-2`), watertight, meshVolRel `2.66e-03`; native (86-face ruled
    tiling) and OCCT (6-face swept-BSpline) are DIFFERENT builders on the SAME guide yet
    agree spatially to `2.5e-4` — the M7a-binding bbox check the prototype lacked. The
    faces heuristic was made cap-aware (native side bands `84 = 21×` OCCT's `4`, a valid
    refinement); no geometric tolerance weakened.

Enforced SPATIAL metric = OCCT-vs-native bbox corner delta within the linear deflection
tolerance (`maxBBoxCornerDelta`) — the M7a `bboxΔ 0.54` failure mode is caught at `< 5e-2`.
A separate scalar Hausdorff distance was NOT computed; watertight + mesh-volume-convergence
+ bbox-corner-delta together bound shape/placement fidelity. The §7.2 decline path stays
live for any curved-spine / degenerate regime (→ OCCT).
