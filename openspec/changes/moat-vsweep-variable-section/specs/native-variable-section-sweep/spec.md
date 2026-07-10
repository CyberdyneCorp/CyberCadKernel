# native-variable-section-sweep

## ADDED Requirements

### Requirement: Native variable-section / guide+spine sweep (morph A→B along a spine, optionally guide-scaled)

The kernel SHALL provide a native, **OCCT-free** variable-section sweep op
`cc_variable_sweep(profileA_XY, aCount, profileB_XY, bCount, spineXYZ, spineCount,
guideXYZ, guideCount)` that sweeps a section MORPHING from profile A (at the spine start)
to profile B (at the spine end) along the 3D polyline spine, each station at arc-length
fraction `f ∈ [0,1]` placing the linearly interpolated section `interpolate(A,B,f)` in the
section frame, OPTIONALLY scaled uniformly by the guide splay
`scale(f) = dist(spine(f), guide(f)) / dist(spine(0), guide(0))`. A NULL guide or
`guideCount < 2` SHALL mean NO guide (`scale ≡ 1`), reducing EXACTLY to `cc_loft_along_rail`.

Profile A and profile B SHALL share a vertex count (`aCount == bCount ≥ 3`, the ruled
morph pairing vertex `k` of A to vertex `k` of B) and the spine SHALL have `≥ 2` points.

The op SHALL be served NATIVELY when the spine is straight OR smooth-curved and the tube
welds watertight:
- a STRAIGHT spine SHALL be framed by the perpendicular frame `uDir = t̂ × ref`,
  `vDir = t̂ × uDir` (`ref = +Z` unless `|t̂·Z| > 0.95` then `+X`) held constant — the SAME
  law the landed `loft_along_rail` and its OCCT oracle use; with NO guide the native path
  SHALL forward to the landed `build_loft_along_rail` BYTE-IDENTICALLY;
- a SMOOTH-CURVED spine SHALL be framed by the double-reflection rotation-minimizing frame
  (`rmfFrames`), the spine densified by arc length so each ruled band's tangent turn stays
  under the landed weld bound `kMaxBandTurn`;
- each station's morphed+scaled ring SHALL be tiled into a watertight ruled tube by the
  landed `assembleRingTube` (one bilinear ruled band per (morph edge × station segment) +
  two planar caps, oriented outward), guarded against self-intersection by the landed
  `sectionSweepUnsafe`.

The result SHALL be accepted ONLY when the engine self-verify passes: robustly watertight
(`robustlyWatertight`, a deflection ladder) with a positive enclosed volume. Any candidate
that fails SHALL be DISCARDED → OCCT. The native builder SHALL consume the landed
`build_loft_along_rail`, `build_curved_rail_loft`, `rmfFrames`, `stationTangents`,
`resamplePolylineByArcLength`, `centredProfile`, `sectionSweepUnsafe`, `assembleRingTube`,
`spineIsStraight`, `spineIsPlanar`, and `cleanPath` BYTE-IDENTICALLY, adding no change to
the landed sweep/loft substrate, and SHALL keep the tessellator, boolean, blend, analysis,
and exchange modules UNTOUCHED.

Anything outside this arm SHALL return a NULL Shape (→ OCCT `BRepOffsetAPI_MakePipeShell`
multi-section): mismatched section vertex counts (`aCount != bCount`), a NON-PLANAR
**guided** spine (the genuine corrected-Frenet law the RMF frame does not reproduce), a
coincident guide start (`d0 ≈ 0`), a non-positive station scale (a collapsing guide), a
degenerate profile / spine, or a self-folding morph (a rim overtaking the spine step). The
engine SHALL NEVER hand a native void to OCCT and SHALL NEVER widen a tolerance to accept a
folded solid. `src/native/**` SHALL stay OCCT-FREE.

#### Scenario: Circle→circle radius morph along a straight spine (truncated cone)

- **WHEN** `cc_variable_sweep` morphs a circle of radius `r0` into a circle of radius `r1`
  (equal vertex count polygons) along a straight spine of length `H`, no guide, under the
  native engine
- **THEN** the result is a watertight, consistently-oriented single lump (Euler χ = 2)
  whose enclosed volume equals the truncated-cone closed form `πH/3·(r0² + r0·r1 + r1²)`
  (within the polygon-vs-circle bound; exactly the polygon prismatoid volume)

#### Scenario: Constant section, no guide (plain ruled sweep)

- **WHEN** `cc_variable_sweep` morphs a profile to ITSELF (A == B) along a straight spine
  of length `H` with no guide under the native engine
- **THEN** the result is the plain ruled sweep — a prism of the exact volume `area·H`,
  watertight, matching `cc_loft_along_rail` (a constant section is a prism)

#### Scenario: Guide-scaled square (square frustum)

- **WHEN** `cc_variable_sweep` sweeps a constant square along a straight spine with a guide
  splaying linearly so the section scales `1 → k` under the native engine
- **THEN** the result is a square frustum whose volume equals the prismatoid closed form
  `H/3·(A_start + A_end + √(A_start·A_end))`, watertight and consistently oriented

#### Scenario: Native-vs-OCCT parity on the simulator

- **WHEN** the same variable sweeps (circle→circle straight and smooth-arc, constant
  square, guide-scaled square) run under the native engine and against the OCCT oracle
  `BRepOffsetAPI_MakePipeShell` (multi-section) + `BRepGProp` on a booted iOS simulator
- **THEN** the native and OCCT results agree on volume, area, centroid, bounding box, and
  watertightness within fixed tolerances (exact for the straight morph, tessellation-
  bounded for the curved morph)

#### Scenario: Honest decline outside the native arm

- **WHEN** the section vertex counts differ, the guided spine is non-planar, the guide
  starts on the spine, the guide scale collapses to zero, the input is degenerate, or the
  morph would self-fold
- **THEN** the native builder returns a NULL Shape and the engine falls back to OCCT
  `BRepOffsetAPI_MakePipeShell` — never a wrong, leaky, or self-overlapping solid

The `cc_*` ABI change SHALL be ADDITIVE (a new `cc_variable_sweep` entry only); no existing
signature changes.
