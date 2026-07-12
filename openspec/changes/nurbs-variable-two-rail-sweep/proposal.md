# Proposal — nurbs-variable-two-rail-sweep (NURBS roadmap Layer 6)

## Why

The Layer-6 swept-surface module (`src/native/math/bspline_sweep.{h,cpp}`) landed the
translational-exact sweep, the general rotation-minimizing-frame (RMF) sweep, the rational sweeps,
and the exact rational `sweepRotational` (revolve, A7.1). Its scope note lists two capabilities as
explicit residuals: a **variable-section sweep** (the profile morphs — scales and/or twists — as it
travels the spine) and a **two-rail sweep** (the section is fit between two rail curves, anchored to
touch both at every station). These are the standard BRepFill-class surfacing primitives behind
tapered pipes, twisted extrusions, and rail-driven lofts. This change lands both, additively.

They are worth building now because each has an **airtight, closed-form oracle** reusing machinery
that already exists (the RMF placement, the affine `transform.h`, and the just-landed `skinSurface` /
`skinRationalSurface`):

- **Constant-section degenerate** — a variable sweep with unit scale and zero twist reproduces the
  existing RMF sweep EXACTLY (machine precision), so the new path is a strict superset.
- **Linear-taper cone** — a rational circle scaled linearly along a straight spine is an EXACT
  rational cone frustum (every station iso a true circle at radius `s(t)·R`).
- **Two-rail on-rails** — two parallel straight rails give a planar ruled strip and two diverging
  rails an exact linear taper, with the section's anchor endpoints lying ON both rails at every
  station (machine precision).

Degenerate rails (crossing / zero-length chord), coincident anchors, non-positive scale, and rational
input on the non-rational routines all honest-decline.

## What

Additive routines in the existing `src/native/math/bspline_sweep.{h,cpp}` (namespace
`cybercad::native::math`, `CYBERCAD_HAS_NUMSCI`-gated, OCCT-free). No existing declaration or signature
changes — the `cc_*` ABI and the current sweep API are byte-unchanged.

1. **`sweepVariable`** (non-rational) and **`sweepRationalVariable`** (rational section) — sweep the
   section along the trajectory while applying a per-station **scale** (uniform, about the section
   origin) and **twist** (rotation about the local sweep axis) to the RMF-transported section, then
   **skin** the placed sections. Scale/twist are supplied as sampled fields (one value per station);
   analytic laws are sampled by the caller. A uniform scale + rotation is a similarity, so the
   rational variant preserves the section's weights exactly.

2. **`sweepTwoRail`** (non-rational) and **`sweepRationalTwoRail`** (rational section) — the section
   carries two **anchor** pole indices; at each station the section is scaled so its anchor chord
   matches the rail-to-rail chord `|rail1(t) − rail0(t)|`, oriented so the anchor chord aligns with
   the rail chord (remaining spin removed by an RMF along the rail-midpoint spine), and translated so
   anchor0 lands on `rail0(t)`; the placed sections are skinned. The placement is a similarity
   (weights preserved for the rational variant).

A host-analytic gate `tests/native/test_native_nurbs_vsweep.cpp` proves the oracles above, wired into
`CMakeLists.txt` under the `CYBERCAD_HAS_NUMSCI` block beside the base sweep gate.

## Impact

- **Affected specs**: `native-math` (ADDED requirements for the variable-section and two-rail sweeps).
- **Affected code**: `src/native/math/bspline_sweep.{h,cpp}` (additive), `CMakeLists.txt` (new test
  registration), new `tests/native/test_native_nurbs_vsweep.cpp`.
- **ABI**: `cc_*` facade byte-unchanged (no ABI header touched; math-layer-only additions).
- **OCCT**: `src/native` stays OCCT-free (0 OCCT/Geom/BRep/TK code or includes in changed files).
- **Residuals**: exact GeomFill/BRepFill-class *continuous* variable/two-rail sweeps (the skin of a
  finite station set is an approximation refining with station count), and multi-section morph laws
  beyond scale+twist, remain documented residuals.
