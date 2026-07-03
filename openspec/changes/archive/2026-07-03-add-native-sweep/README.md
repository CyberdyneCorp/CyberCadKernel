# add-native-sweep

Phase 4 capability **#4b Tier C — sweep / pipe-shell**. The next honest increment
of native construction after Tier A (holed / typed-profile extrudes + typed-profile
revolve) and Tier B (2-section ruled loft). This change moves **`cc_solid_sweep`**
native for the two tractable cases and keeps the genuinely hard pipe-shell / guide
cases as honest, labelled OCCT fall-through.

It does NOT change the `cc_*` ABI, does NOT change the default engine (stays OCCT),
and does NOT fake any sub-case — anything not genuinely native + verified falls
through to OCCT in `NativeEngine` and is documented as such.

## What a sweep is (the `cc_solid_sweep` contract)

`cc_solid_sweep(profileXY, profileCount, pathXYZ, pathCount)` sweeps a **closed 2D
profile** — `(x,y)` pairs, centred on the profile centroid, lying in its own local
XY plane — along a **3D path** (`(x,y,z)` triples). The profile is placed
**perpendicular to the path at the path's start** (its local +Z tangent to the path)
and carried along the spine, generating a swept ("pipe") solid capped at both ends.
This is the `cc_*` mirror of OCCT `BRepOffsetAPI_MakePipe` (the simple constant-frame
pipe) / `BRepOffsetAPI_MakePipeShell` (the general guided pipe-shell).

## Scope (Tier C) — `cc_solid_sweep` native for the tractable cases

| Sweep case | Native in this change | Falls through to OCCT (honest, labelled) |
|---|---|---|
| **Straight path** (single segment, or all path points collinear) | YES — the sweep is a **directional extrude** of the profile along the path vector; the swept solid's volume is **EXACT** (`profileArea · pathLength`) | — |
| **Smooth curved path** (C¹, gentle curvature) | YES — a **rotation-minimizing frame (RMF / parallel-transport)** carries the profile frame along a discretized spine with **minimal twist**; one **swept surface** (Bézier/BSpline `FaceSurface`) per profile edge + two end caps → a watertight native B-rep, **deflection-bounded** vs OCCT `MakePipe` | — |
| **Tight-curvature / self-intersecting sweep** (curvature radius < profile extent → the swept tube folds on itself) | — | YES — the builder **detects** the fold (a guard, see design.md) and returns a NULL `Shape`; `NativeEngine` falls through to OCCT (never faked) |
| `cc_twisted_sweep` (sweep + accumulating twist / end-scale) | **attempted IF the RMF sweep works** — the RMF frame is post-rotated by an accumulating `twistRadians·s` and scaled by `lerp(1, scaleEnd, s)`; the same tight-curvature / self-intersection guard applies | YES when the base RMF sweep is not applicable, or the guard trips |
| `cc_guided_sweep` (profile guided by an auxiliary guide curve) | — | YES — a genuine pipe-shell/guide problem; **labelled, verified fall-through**, not faked |
| `cc_loft_along_rail` (two profiles skinned along a rail) | — | YES — a genuine pipe-shell/rail problem; **labelled, verified fall-through**, not faked |

### Why the hard cases fall through (not faked)

- **Tight-curvature / self-intersecting sweeps.** When the spine's curvature radius
  drops below the profile's radial extent, the swept surface self-intersects (the
  inner wall of a bend crosses itself). Producing a valid B-rep here requires
  trimming self-intersections — the pipe-shell robustness problem OCCT's BOPAlgo /
  `BRepOffsetAPI_MakePipeShell` solves. Tier C **guards** it (max path curvature vs
  profile radius) and defers rather than emit a self-intersecting (invalid) solid.
- **`cc_guided_sweep` / `cc_loft_along_rail`.** A guide/rail re-orients or re-scales
  the profile per the auxiliary curve (`BRepOffsetAPI_MakePipeShell` with a guide, or
  `BRepFill` along a rail). This is a strictly harder, separately verifiable
  correspondence problem than a rotation-minimizing pipe; Tier C leaves them as
  honest OCCT fall-through and documents it, rather than approximate the guide.
- **`cc_twisted_sweep`** is delivered **only as a strict extension of the working RMF
  sweep** (post-compose an accumulating rotation + scale onto the transported frame).
  If the base RMF sweep is not applicable for the given path (straight-only, or the
  curvature guard trips), the twisted sweep falls through too — no separate faked path.

## Method (locked, per NATIVE-REWRITE.md)

Clean-room from the `cc_solid_sweep` contract semantics (the doc-comments in
`include/cybercadkernel/cc_kernel.h`) and computational-geometry first principles
(the **double-reflection rotation-minimizing frame** of Wang–Jüttler–Zheng–Wang,
2008; parallel transport; ruled/swept surface tiling), with OCCT source
(`/Users/leonardoaraujo/work/OCCT/src`: `BRepOffsetAPI_MakePipe`,
`BRepOffsetAPI_MakePipeShell`, `BRepFill_PipeShell`, `GeomFill_CorrectedFrenet`)
consulted as a **reference oracle only** — never copied. The profile frame is
transported with minimal twist (RMF), the swept surfaces are real native-math
`Bezier`/`BSpline` `FaceSurface`s, and the ends are capped with the profile faces.

## Architecture / OCCT boundary (unchanged from #4 / #4b Tier A–B)

- A new native builder lives under `src/native/construct/sweep.h` and stays
  **OCCT-FREE and host-buildable** (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`,
  no OCCT, no simulator); it includes only `src/native/math` + `src/native/topology`
  + `src/native/tessellate` + the existing `src/native/construct/` assemblers and
  returns a `topology::Shape`. A **swept surface** helper is added to
  `src/native/math` (a `SweptSurface`, or Bézier/BSpline poles from the transported
  profile) if the analytic `FaceSurface` kinds are insufficient.
- `src/engine/native/native_engine.{h,cpp}` — `solid_sweep` (currently a pure
  `fallback()` delegation) becomes native-else-fallback; a NULL native result (a
  straight/curved case the builder declined, or a tight-curvature guard trip) falls
  through to the fallback with no interception. `twisted_sweep` becomes native-else-
  fallback under the same RMF path; `guided_sweep` and `loft_along_rail` stay pure
  fall-through (labelled). OCCT stays behind `CYBERCAD_HAS_OCCT`; the native builder
  never sees OCCT.
- **No `cc_*` ABI change**; the default engine stays OCCT (opt-in via
  `cc_set_engine(1)`), so every existing suite is unchanged unless it opts in.

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT): built native B-rep +
   its native tessellation.
   - **Straight sweep = extrude** — a square profile swept along a straight path of
     length `L` is watertight (`boundaryEdgeCount == 0`) with **exact** volume
     `profileArea · L` and the expected face count (N side faces + 2 caps).
   - **Curved sweep** — a circular-ish profile swept along a gentle arc spine is
     watertight, and its volume converges to `profileArea · spineArcLength` within a
     deflection bound (Pappus check for the centroid-on-spine assumption).
   - **RMF twist minimality** — the transported frame's accumulated twist over a
     planar arc is ~0 (double-reflection RMF property), asserted analytically.
   - **Guards / fall-through** — a tight-curvature path (curvature radius < profile
     radius), a self-intersecting spine, and a degenerate profile/path each return a
     NULL `Shape`.
2. **Simulator native-vs-OCCT parity through the facade**: the SAME `cc_solid_sweep`
   (and `cc_twisted_sweep`) calls issued native (`cc_set_engine(1)`) vs OCCT
   (default), compared on mass properties / bbox / sub-shape counts / watertight
   tessellation within a documented fp64 (straight: exact) / deflection (curved)
   tolerance vs `BRepOffsetAPI_MakePipe`. The fall-through cases (tight curvature,
   `cc_guided_sweep`, `cc_loft_along_rail`) asserted **identical** under both engines
   (fall-through proof). Default restored in teardown.

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green at the
OCCT default.
