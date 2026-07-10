# Design — moat-vsweep-variable-section

## Op semantics

`cc_variable_sweep(profileA_XY, aCount, profileB_XY, bCount, spineXYZ, spineCount,
guideXYZ, guideCount) → CCShapeId`

Sweep a section that MORPHS from profile A (at the spine START) to profile B (at the spine
END) along the 3D polyline spine. Each station at arc-length fraction `f ∈ [0,1]` places
the section `interpolate(A,B,f)` (vertex `k` of A paired to vertex `k` of B — hence A and
B SHARE a vertex count) in the section frame, then scales it uniformly by the guide splay
`scale(f) = dist(spine(f), guide(f)) / dist(spine(0), guide(0))`. A NULL guide or
`guideCount < 2` means NO guide (`scale ≡ 1`), which reduces EXACTLY to `cc_loft_along_rail`.

This is a strict SUPERSET:
- `cc_loft_along_rail` = `cc_variable_sweep` with no guide.
- `cc_guided_sweep` = `cc_variable_sweep` with A == B (constant section) + a guide.
- The genuinely NEW capability = A ≠ B **and** a guide together (a morphing, guide-steered
  boss), which no landed op offers.

## Frame law (matches the landed ops + the OCCT oracle)

- **Straight spine** → the perpendicular frame `uDir = t̂ × ref`, `vDir = t̂ × uDir`
  (`ref = +Z` unless `|t̂·Z| > 0.95` then `+X`) — the SAME `perpendicularFrame` law the
  landed `loft_along_rail` and its OCCT oracle use. Held CONSTANT along the (single-
  tangent) straight spine.
- **Smooth-curved spine** → the double-reflection rotation-minimizing frame (`rmfFrames`,
  Wang–Jüttler–Zheng–Yang) — the SAME zero-torsion transport the landed
  `build_curved_rail_loft` uses. The spine is densified by arc length so each ruled band's
  tangent turn stays under `kMaxBandTurn`, so the tube welds watertight.

The morphed+scaled ring at station `s` is `origin + uDir·((Aₖ.x + (Bₖ.x−Aₖ.x)f)·sc) +
vDir·((Aₖ.y + (Bₖ.y−Aₖ.y)f)·sc)`, tiled by the landed `assembleRingTube` (one bilinear
ruled band per (morph edge × station segment) + two planar caps, oriented outward).

## Native scope vs honest decline

NATIVE (self-verified robustly watertight + positive enclosed volume):
- No guide + straight spine → the exact `loft_along_rail` perpendicular ruled morph. A
  circle→circle radius morph is a truncated cone; a constant section is a prism.
- No guide + smooth-curved (any 3D) spine → the RMF morph (`build_curved_rail_loft`).
- With guide + straight or smooth-curved PLANAR spine → the RMF/perp guide-scaled morph
  tube (`build_variable_sweep_tube`), when it welds watertight and does not self-fold.

DECLINE → NULL Shape → OCCT `MakePipeShell` (multi-section):
- Mismatched section vertex counts (`aCount != bCount`) — the ruled morph pairs `k→k`.
- A NON-PLANAR **guided** spine — the guide-scaled morph on a non-planar spine uses the
  genuine corrected-Frenet law OCCT applies, which the RMF frame does not reproduce, so we
  defer rather than ship a solid that disagrees with the oracle.
- A coincident guide start (`d0 ≈ 0`), a non-positive station scale (a collapsing guide),
  a degenerate profile / spine (< 3 profile pts / < 2 spine pts), or a self-folding morph
  (a rim overtaking the spine step, caught by `sectionSweepUnsafe`).

The engine runs the MANDATORY self-verify (`robustlyWatertight` + `watertightVolume > 0`)
and DISCARDS any candidate that fails → OCCT. The native builder never fakes a wrong shape
and never hands a native void to OCCT.

## Two-gate verification

- **Gate (a) HOST-analytic** (`tests/native/test_native_vsweep.cpp`, OCCT-free): a
  circle(r0)→circle(r1) morph along a straight spine of length `H` is a truncated cone,
  volume `πH/3·(r0² + r0·r1 + r1²)` (matched via the exact polygon-frustum prismatoid
  volume + the smooth-cone closed form within the polygon-vs-circle bound); a constant
  section is a prism (`area·H`, exact); a guide-scaled square is a frustum (prismatoid
  volume, exact); a curved-arc morph is watertight + consistently oriented + Euler χ = 2 +
  volume-stable across deflections; plus the honest declines (mismatched counts, coincident
  guide, collapsing guide, degenerate input).
- **Gate (b) SIM parity** (`tests/sim/native_vsweep_parity.mm`, `run-sim-native-vsweep.sh`,
  on the SKIP list): native vs OCCT `BRepOffsetAPI_MakePipeShell` (multi-section) +
  `BRepGProp` on a booted iOS simulator — volume / area / centroid / bbox / watertight,
  with the non-planar guided morph asserted as a verified OCCT fall-through.

## Reuse map (byte-frozen landed code)

`build_loft_along_rail`, `build_curved_rail_loft`, `perpendicularFrame` (as the new
`perpFrame`, matching the OCCT helper), `rmfFrames`, `stationTangents`,
`resamplePolylineByArcLength`, `centredProfile`, `sectionSweepUnsafe`, `assembleRingTube`,
`spineIsStraight`, `spineIsPlanar`, `cleanPath`, `kMaxBandTurn`, `kMaxDensifyStations`.
