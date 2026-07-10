# Proposal — moat-vsweep-variable-section (variable-section / guide+spine sweep)

## Why

VARIABLE-SECTION SWEEP — sweeping a profile that MORPHS between section shapes along a
spine while optionally being STEERED by a guide rail — is the Shapr3D-style "shaping boss"
and the SolidWorks/Fusion "swept boss with guide + profile morph" that the app will need
for any tapering duct, blended handle, transition fitting, or organically-varying feature.
No `cc_*` entry expresses it today: `cc_solid_sweep` transports a CONSTANT section,
`cc_loft_along_rail` morphs A→B along a rail but has NO scale/guide law, and
`cc_guided_sweep` guide-scales a CONSTANT section but does NOT morph. The combination —
a section that both CHANGES SHAPE (A→B) and is guide-STEERED (scaled by the guide splay) —
has no facade op, so the app's variable-section path can only reach OCCT
`BRepOffsetAPI_MakePipeShell` (multi-section).

The tractable case is a bounded **assembly of the landed sweep/loft substrate**, not new
math: each station's section is the linear interpolation of the two end profiles placed by
the SAME frame the landed ops use (perpendicular frame on a straight spine — the
`loft_along_rail` law; rotation-minimizing frame on a smooth-curved spine — the
`build_curved_rail_loft` law), scaled uniformly by the guide splay
`dist(spine(f),guide(f)) / dist(spine(0),guide(0))` (the landed `guided_sweep` law), and
the per-station rings are tiled by the landed `assembleRingTube` and self-verified.

## What changes

- **New builder `build_variable_sweep` in `src/native/construct/sweep.h`** (OCCT-free,
  header-only) — morphs profile A (spine start) → profile B (spine end) along the spine,
  each station = `interpolate(A,B,f)·guideScale(f)` placed by the perpendicular
  (straight) / RMF (curved) frame, tiled into a watertight ruled tube by the landed
  `assembleRingTube` + self-fold guard (`sectionSweepUnsafe`). WITH NO GUIDE it reuses the
  landed `build_loft_along_rail` morph BYTE-IDENTICALLY (straight = perpendicular ruled
  loft, curved = RMF morph). A guide runs the new `build_variable_sweep_tube` (RMF/perp
  guide-scaled morph). Reuses the landed `rmfFrames`, `stationTangents`,
  `resamplePolylineByArcLength`, `centredProfile`, `sectionSweepUnsafe`, `assembleRingTube`.
- **Additive `cc_variable_sweep` in the C ABI** — `cc_variable_sweep(profileA_XY, aCount,
  profileB_XY, bCount, spineXYZ, spineCount, guideXYZ, guideCount)`, styled like the landed
  `cc_loft_along_rail` / `cc_guided_sweep`. `IEngine::variable_sweep` default
  `engine_unsupported`; facade wiring in `src/facade/cc_kernel.cpp`. No existing `cc_*`
  signature changes.
- **`NativeEngine::variable_sweep`** serves the native morph (self-verified robustly
  watertight + positive enclosed volume, else honest decline → OCCT); **`OcctEngine::
  variable_sweep`** is the `BRepOffsetAPI_MakePipeShell` MULTI-SECTION oracle (sample the
  spine, place a perpendicular-framed morphed+scaled section wire at each station, Add them
  all, sweep along the spine).

## Impact

- The landed sweep/loft substrate (`build_sweep`, `build_loft_along_rail`,
  `build_curved_rail_loft`, `build_guided_sweep`, `rmfFrames`, `assembleRingTube`) stays
  **byte-frozen** — the new builder only CALLS it; the no-guide path forwards to
  `build_loft_along_rail` unchanged.
- `src/native/**` stays OCCT-FREE (descriptive comments only).
- Tessellator, boolean, blend, analysis, and exchange tracks are UNTOUCHED. The
  concurrent sheet-metal track's new `sheetmetal` module is untouched.
- A NON-PLANAR guided spine, mismatched section vertex counts, a coincident guide start, a
  non-positive guide scale, or a self-folding morph are honest-declined → OCCT
  `MakePipeShell`; never a wrong / leaky / self-overlapping solid, never a widened
  tolerance.
