# Tasks — moat-m0w-closed-seam-weld (MOAT M0 tessellator weld for the closed-inner-seam annulus)

Order: reproduce (done) → guarded seam pinning in the M0 tessellator → byte-identical hash
proof for every existing mesh → closed-seam ladder regression test → host-analytic ladder
gate → sim native-vs-OCCT parity gate → run both gates → docs → commit. All native code
stays OCCT-free and host-buildable (`clang++ -std=c++20`). The `cc_*` ABI is additive-only.
The change is strictly topology-guarded and ADDITIVE: every existing surface kind and edge
meshes BYTE-IDENTICAL; the ONLY meshes allowed to change are the previously-failing
closed-seam cases (non-watertight → watertight). No global tolerance is widened; a case
that still can't weld DECLINES (non-watertight → NULL → OCCT) — a correct decline is a
first-class outcome.

## STATUS — LANDED (closed inner seam); curved RIM is a separate documented blocker

The closed inner seam shared by the curved sub-face and the flat cap now welds watertight
at every deflection. Root cause was: the M0 tessellator re-evaluated the curved sub-face's
seam-boundary vertices through its OWN bowl surface (`S_bowl(pcurve(t))`, which bulges off
the straight degree-1 chord) instead of pinning them to the seam edge's canonical 3D chord
points; the FINE regime diverged (399 of ~4788 near-seam vertices coincided at `d=0.004`).
The fix is a topology-guarded canonical-seam pin (`SeamPins`). The curved-wall COMMON
(`KeepSide::Above`) additionally welds the freeform bowl's OUTER curved RIM to the flat top
lid — a SEPARATE, pre-existing curved-edge fragility (a free-form face subdivides the shared
rim beyond a flat neighbour's need, plus a coarse-regime near-degenerate sliver) — so COMMON
still honestly declines at the fine end; that decline is now isolated to the rim, not the
seam.

## 1. Reproduce + diagnose (done)

- [x] 1.1 Confirm the byte-frozen baseline: `test_native_curved_wall_cut` CUT robust, COMMON
  asserted as a MEASURED decline via `curved_wall_common_rim_weld_fragility_is_measured_decline`.
- [x] 1.2 Measure the FINE divergence (only 399 of ~4788 near-seam vertices coincide at
  deflection `0.004`; the curved sub-face bulges off the straight chord, the cap sits on it).
- [x] 1.3 Localize the root cause to boundary-vertex evaluation in
  `src/native/tessellate/face_mesher.h` (`BoundaryAnchors` snap fires only within
  `kSnapEps = 1e-6`, far tighter than the bowl bulge).
- [x] 1.4 Separate the two failure surfaces by z-band: the closed SEAM (fixable here) vs. the
  curved RIM (pre-existing, distinct — present in the bare operand at fine deflection).

## 2. Implement the guarded seam pinning

- [x] 2.1 `edge_mesher.h`: add `detail::isSeamChord` — a 2-pole degree-1 curve, the exact
  shape `smooth_trim_split` / `curved_wall_cut` lay as one segment of a closed interior seam.
- [x] 2.2 `face_mesher.h`: add `SeamPins` (a UV → canonical-3D map) and `recordSeamChordPins`
  — for a seam-chord edge, pin the diverging (curved-sub-face) boundary samples EXACTLY to
  the edge's canonical 3D chord points `d.points` by UV correspondence, so both sub-faces
  emit BIT-IDENTICAL seam points. `evaluatePoints` / `structuredGrid` consult the pin before
  surface evaluation.
- [x] 2.3 Guard: the pin fires ONLY on a seam chord that genuinely diverges (`> kSnapEps`); a
  flat cap's chord samples equal `S_plane(pcurve)` and are never pinned. No analytic
  primitive, no genuinely-curved shared edge (a cylinder cap↔side circle), and no straight
  `Line` edge is a seam chord — so none is pinned. No global weld/snap tolerance widened.
- [x] 2.4 Cognitive complexity kept in band via the small `isSeamChord` / `SeamPins` /
  `recordSeamChordPins` helpers.

## 3. Byte-identical proof (the acceptance battery)

- [x] 3.1 FNV hash over `{vertexCount, triangleCount, vertices, triangles, watertight, area,
  volume}` for a meshed solid; battery = box + bump-capped cylinder (CURVED shared seam) +
  small variant + rational-BSpline cap + bowl operand + mid-wall operand, over six
  deflections `{0.05, 0.02, 0.01, 0.006, 0.004, 0.002}` = 36 (shape,deflection) hashes.
- [x] 3.2 Baseline (git stash) vs changed: **36/36 hashes BYTE-IDENTICAL** — the guarded
  seam-chord branch is unreachable for every existing mesh.
- [x] 3.3 The closed-seam CUT/COMMON solids (which carry the seam) change from non-watertight
  → watertight, as intended; nothing else changes.

## 4. Closed-seam ladder regression test

- [x] 4.1 Add `curved_wall_closed_seam_welds_watertight_fine_deflection`: the dome CUT
  (disk ∪ flat cap sharing the closed seam) welds watertight (Euler `χ = 2`) at the closed
  form at `d = 0.004` — the fine deflection that DECLINED (`NotWatertight`) before the pin.
- [x] 4.2 Extend `midwall_cut_below_watertight_converges_to_closed_form` to assert the ANNULAR
  cap (inner hole = closed seam) welds watertight and converges across its full ladder
  `{0.02, 0.012, 0.008, 0.005, 0.0025}` (0.62% → 0.10%).
- [x] 4.3 Keep the honest-decline test `curved_wall_common_rim_weld_fragility_is_measured_decline`:
  COMMON's remaining fine-deflection decline is now isolated to the curved RIM (not the
  seam) and stays non-watertight → NULL → OCCT, never a leaky solid.

## 5. Host GATE (a) — analytic, no OCCT

- [x] 5.1 Mid-wall ANNULAR-cap CUT (the canonical annulus: outer = wall chords, inner hole =
  the closed seam) welds watertight (Euler `χ = 2`) at the closed-form cap volume across its
  full ladder, converging 0.62% → 0.10%, built with NO OCCT.
- [x] 5.2 Dome CUT closed seam welds watertight at the fine `d = 0.004` (baseline declined).
  Full host `ctest`: 58/58 GREEN.

## 6. Sim GATE (b) — native-vs-OCCT parity

- [x] 6.1 The SAME bowl-cup operand reconstructed in OCCT (`Geom_BezierSurface` bowl + planar
  lid, sewn, outward-oriented) cut by `BRepAlgoAPI_Common` (keep-half box) on the booted iOS
  simulator (`scripts/run-sim-native-curved-wall-cut.sh`).
- [x] 6.2 The native CUT/COMMON matches OCCT (`BRepGProp`) on volume, area, watertightness,
  Euler `χ = 2`, bbox, and one-sided Hausdorff at all asserted deflections — **45/45 PASS**,
  INCLUDING the newly-added fine `CUT d = 0.004` (native watertight, volume rel 4.0%, area
  0.5%, Hausdorff 3.4e-8) that declined pre-pin; COMMON fine decline stays honest NULL.

## 7. Run both gates + discipline proof

- [x] 7.1 Full host `ctest` GREEN (58/58), including the byte-identical battery and the
  closed-seam ladder regression test.
- [x] 7.2 `git diff src/native` proves 0 OCCT includes gained and the `cc_*` ABI is untouched.
- [x] 7.3 Both gates run: host analytic (byte-identical + closed-seam ladder) + sim
  native-vs-OCCT parity (45/45) on the booted simulator.

## 8. Docs + commit

- [x] 8.1 Update `openspec/MOAT-ROADMAP.md`: closed-seam weld landed (both gates); the
  `moat-m2cw-curved-wall-cut` COMMON blocker now isolated to the curved RIM (the seam is no
  longer a blocker), with gate numbers.
- [x] 8.2 Commit the guarded seam pinning + the byte-identical battery + the closed-seam
  ladder test + the sim fine-deflection parity.
