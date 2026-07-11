# Proposal — nurbs-swept-surface (NURBS roadmap Layer 6)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 1 (the
exact-NURBS *geometry kernel*, `src/native/math/bspline_ops.{h,cpp}`), Layer 7 (fitting /
approximation, `src/native/math/bspline_fit.{h,cpp}`), and the first Layer-6 slice — **skinning /
lofting** (`src/native/math/bspline_skin.{h,cpp}`, a surface interpolated *through* given sections)
— are landed. This change adds the second Layer-6 slice: **swept surfaces** — given a **section**
(profile) B-spline curve and a **trajectory** (spine), construct a tensor-product B-spline surface
that sweeps the section along the trajectory. This is the core freeform-surfacing primitive behind
pipes, extrusions, mouldings and swept solids. Where skinning interpolates a surface *through*
sections the caller already has, sweeping *generates* the sections by moving one profile along a
spine, then skins them.

This slice is worth building **now** because it (a) is small and well-bounded (*The NURBS Book*
§10.4), (b) is built entirely on machinery that already exists — the Layer-1 data types, the affine
`transform.h` moving frame, and the just-landed Layer-6 `skinSurface` — and (c) has a **uniquely
airtight base case**: the *translational* sweep (a section swept along a straight vector) is the
EXACT tensor product of the section with a degree-1 path, so every iso-curve is the section
translated — a machine-exact, fitting-free oracle. The general sweep then reuses the skin's
containment oracle.

## What

A new OCCT-free module `src/native/math/bspline_sweep.{h,cpp}` (namespace `cybercad::native::math`,
beside `bspline_ops` / `bspline_fit` / `bspline_skin`), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`,
because the general sweep composes `skinSurface`, whose V-interpolation solves through the numsci
facade). It reuses the Layer-1 `BsplineCurveData` (section / trajectory in) / `BsplineSurfaceData`
(surface out) types. **Non-rational section and trajectory only** (all weights = 1); rational
sweeps are an explicit residual.

From *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 10 §10.4:

1. **Translational sweep** (`sweepTranslational`, the EXACT closed-form case) — sweep the section
   along a straight vector `sweep`. The result is the tensor product of the section curve (U,
   degree `p`, the section's knots, its `N` poles) with a degree-1 two-pole path (V, poles `0` and
   `sweep`): the surface net is `pole(i,0) = section.poles[i]`, `pole(i,1) = section.poles[i] +
   sweep`. Every iso-curve `S(·, v)` is EXACTLY the section translated by `v·sweep` — no fitting, no
   solve, machine-exact. This is the extrusion / tabulated-cylinder special case.
2. **General sweep** (`sweepAlongTrajectory`) — place the section at `K` stations sampled evenly
   along the trajectory (spine) B-spline. At each station compute a **rotation-minimizing moving
   frame** (double-reflection method, Wang, Jüttler, Zheng & Liu, ACM TOG 2008) so the profile does
   not spin about the spine (unlike a Frenet frame, which is undefined at inflections and spins with
   torsion). Transform a copy of the section by each station's rigid frame (translate its origin to
   the station point; rotate its plane normal onto the trajectory tangent within the
   rotation-minimizing frame), then SKIN the `K` transformed sections via the Layer-6 `skinSurface`.
   Non-rational.

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_sweep.cpp`:
  1. **Translational exactness (the base-case oracle)** — the swept surface's iso-curve at each
     station `v` reproduces the section TRANSLATED by `v·sweep` POINTWISE to ~1e-12 (achieved
     ~1e-15). Exact case, no fitting.
  2. **Station containment (general)** — the swept surface contains each TRANSFORMED section at its
     station parameter to ~1e-8 (achieved ~1e-15); the skin's containment oracle carries through the
     transform-then-skin composition.
  3. **Known-surface checks** — a straight section swept translationally reproduces the closed-form
     bilinear ruled patch `S(u,v) = A + u·(B−A) + v·sweep` to ~1e-12; a circle-approx section swept
     translationally is a cylinder-like patch whose every height iso reproduces the section profile
     to ~1e-12.
  4. **Anti-twist frame sanity** — for a straight spine the rotation-minimizing frame keeps the
     section orientation fixed (no spurious spin), so the general straight sweep reproduces the
     translational sweep pointwise (~1e-8, achieved ~1e-15); on a curved spine each station iso
     preserves the section arc length (rigid frame).
  5. **Degenerate guards** — fewer than two stations, coincident-trajectory / null sweep, rational
     section or trajectory, and malformed sections decline honestly (`ok=false`, no crash).
- **SIM native-vs-OCCT parity** — OPTIONAL cross-check against OCCT `BRepOffsetAPI_MakePipe` /
  `GeomFill` (a separate track; HOST is primary and sufficient).

## Scope

- Adds `src/native/math/bspline_sweep.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob. Added to `native_math.h`.
- Adds `tests/native/test_native_nurbs_sweep.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_skin`.
- Only `#include`s `bspline_ops.h` (Layer 1), `bspline_skin.h` (Layer 6), `bspline.h` (evaluators),
  and `transform.h` (moving frame) — it does NOT modify them.
- **`cc_*` ABI unchanged.** Layer 6 is an internal geometry-algorithm library; its consumers are
  later surfacing features, not the app today. No ABI is added until a consumer needs it —
  consistent with the demand-driven policy.

## Non-goals

- **No rational / weighted sweep** — a rational section/trajectory is materially harder and is an
  explicit residual for a later slice. This module sweeps non-rational inputs only and never
  fabricates weights.
- **No rotational (revolved) sweep** — a profile revolved about an axis is exact-rational and a
  distinct construction; it remains a residual.
- **No exact GeomFill/BRepFill-class sweep** — the general sweep skins a finite set of station
  placements (an approximation of the continuous sweep that improves with station count), not an
  analytically exact swept surface. Exact-continuous sweeps remain a residual.
- **No variable section** — a section that morphs (scales/rotates by law, or interpolates between
  end profiles) along the spine remains a residual; this module sweeps one rigid section.
- No error-driven adaptive station selection; no automatic degree/knot selection; no new `cc_*` ABI;
  no change to STEP admission, the tessellator, or any evaluator signature.
