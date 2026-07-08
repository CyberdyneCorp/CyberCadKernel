# Proposal — moat-gs-measure-curvature (MOAT M-GS, GS3 + GS4)

## Why

The app's measurement / analysis features (dimensioning, inspection, the "measure"
and "curvature comb" tools) need two exact geometric SERVICES that the native kernel
does not yet expose: the **minimum distance / angle between two B-rep entities**
(GS3) and the **differential curvature of a surface or edge at a parameter point**
(GS4). Today the app can only get these from OCCT (`BRepExtrema_DistShapeShape`,
`BRepLProp_SLProps` / `GeomLProp`), which keeps the LGPL engine on the critical path
of a read-only analysis feature and blocks the native-only build.

Both are **closed-form math on geometry the native kernel already evaluates
exactly**. The landed native substrate supplies everything needed:

- `src/native/math/elementary.h` + `torus.h` — analytic `Plane` / `Cylinder` /
  `Cone` / `Sphere` / `Torus` with `value(u,v)` and `normal(u,v)`;
- `src/native/math/bspline.h` — `surfaceDerivs` / `nurbsSurfaceDerivs` (to 2nd
  order), `curveDerivs` / `nurbsCurveDerivs`, `surfaceNormal`, rational-aware;
- `src/native/numerics/closest_point.h` — the OCCT-`Extrema`-parity typed
  projection layer (curve/surface point projection with Newton polish), ALREADY
  verified 22/22 vs OCCT `Extrema` in `add-native-numerics`;
- `src/native/topology/shape.h` — `EdgeCurve` / `FaceSurface` kinds and the
  `Shape` graph the facade resolves sub-shape ids against.

This is a **LOW-RISK slice**: closed-form differential geometry (first/second
fundamental forms; the point-to-analytic and line-to-line distance formulae) layered
on an evaluator that is already OCCT-verified. It is scoped to the **analytic +
simple-NURBS core** — the cases where the minimizer is provably robust. The
genuinely-hard cases (minimum distance to a freeform *trimmed* patch where the
seeded Newton minimizer is not robust to a global optimum; near-parametric-
singularity curvature such as a sphere pole) are an **HONEST DECLINE**, not a
guessed number. A measured decline is a first-class outcome; a wrong measurement is
never emitted.

## What Changes

1. **A new header-only, OCCT-free module `src/native/analysis/`** in namespace
   `cybercad::native::analysis`, consuming the landed NURBS/topology/numerics eval:
   - **`distance.h` (GS3)** — minimum distance between an entity pair
     (point·point, point·edge, point·face, edge·edge, edge·face, face·face). For
     **analytic** pairs it returns the closed form (point-to-point / -line /
     -circle / -plane / -cylinder / -sphere; line-to-line incl. parallel and skew);
     for pairs involving a **NURBS** curve/surface it seeds on a coarse sample of
     the parameter domain and refines each seed with the existing
     `numerics/closest_point` Newton polish, returning the global minimum and the
     two witness points. It returns the distance AND both closest points.
   - **`angle.h` (GS3)** — angle between two linear/planar entities (line·line,
     plane·plane, line·plane) from the entities' direction / normal vectors,
     closed-form, in `[0, π/2]` for unsigned pairs (line·line, line·plane) and
     `[0, π]` for oriented plane·plane, degenerate-input guarded.
   - **`curvature.h` (GS4)** — at a surface `(u,v)`: Gaussian `K`, mean `H`, and
     principal `k1 ≥ k2` from the **first fundamental form** `(E,F,G)` and **second
     fundamental form** `(L,M,N)`, where `E,F,G` come from `S_u,S_v` and `L,M,N`
     from `S_uu,S_uv,S_vv` projected on the unit normal — via
     `math::surfaceDerivs` / `nurbsSurfaceDerivs` for NURBS and the closed form for
     each analytic kind (plane `K=H=0`; sphere `K=1/R²`, `H=1/R`, `k1=k2=1/R`;
     cylinder `K=0`, `k1=1/R`, `k2=0`, `H=1/(2R)`; cone `K=0`; torus the known
     `K = cos v / (r (R + r cos v))`, `H = (R + 2 r cos v)/(2 r (R + r cos v))`).
     At an edge parameter `t`: curvature `κ = ‖C′×C″‖ / ‖C′‖³` (line `0`, circle
     `1/R`) via `math::curveDerivs` / `nurbsCurveDerivs`.
2. **An additive `cc_*` facade** in `include/cybercadkernel/cc_kernel.h` +
   `src/engine` dispatch — new accessors only, existing `cc_*` byte-for-byte
   unchanged:
   - `int cc_measure_distance(CCShapeId a, int subKindA, int subIdA, CCShapeId b,
     int subKindB, int subIdB, double *out7)` — `out7 =
     [distance, p1x,p1y,p1z, p2x,p2y,p2z]`; returns `1` on success, `0` on an
     honest decline (with `cc_last_error` set).
   - `int cc_measure_angle(CCShapeId a, int subKindA, int subIdA, CCShapeId b,
     int subKindB, int subIdB, double *outRadians)`.
   - `int cc_surface_curvature(CCShapeId body, int faceId, double u, double v,
     double *out4)` — `out4 = [K, H, k1, k2]`.
   - `int cc_edge_curvature(CCShapeId body, int edgeId, double t, double *outKappa)`.
3. **Two verification gates, no weakened tolerances.**
   - **HOST ANALYTIC (no OCCT):** every result matches the closed form on known
     analytic entities — point/line/circle/plane distances, line·line angle, sphere
     `K=1/R²` / cylinder `K=0,H=1/(2R)` / plane `0` / torus known / edge circle
     `1/R`.
   - **SIM native-vs-OCCT:** on a booted iOS simulator the native distance matches
     `BRepExtrema_DistShapeShape` (min distance + witness points) and the native
     curvature matches `BRepLProp_SLProps` / `GeomLProp_SLProps` (Gaussian, mean,
     principal) within a scale-relative tolerance.
4. **Honest decline is wired end-to-end.** For an entity pair or configuration this
   slice does not robustly handle — minimum distance to a genuinely-trimmed freeform
   patch whose seeded minimizer cannot certify the global optimum; angle between
   non-linear/non-planar entities; curvature where the first fundamental form is
   degenerate (`EG − F² ≈ 0`, a parametric singularity such as a sphere pole) — the
   facade returns `0` and sets `cc_last_error`; it NEVER returns a wrong number.

## Capabilities

### Added Capabilities

- `native-analysis`: NEW capability — exact minimum distance and angle between
  B-rep entity pairs (GS3), and Gaussian / mean / principal surface curvature and
  edge curvature at a parameter point (GS4), computed OCCT-free on the native
  NURBS/analytic evaluators, verified against closed-form host oracles and OCCT
  (`BRepExtrema_DistShapeShape`, `BRepLProp_SLProps` / `GeomLProp`), with an honest
  decline for the non-robust freeform / parametric-singularity cases.

## Impact

- **NEW** `src/native/analysis/{distance,angle,curvature}.h` — header-only, namespace
  `cybercad::native::analysis`, **zero OCCT includes**. Consumes only
  `math/*`, `numerics/closest_point.h`, `topology/shape.h`.
- **`include/cybercadkernel/cc_kernel.h`** — four ADDITIVE prototypes + two POD-free
  out-array contracts; the ABI contract test (`CC_KERNEL_NO_PROTOTYPES`) confirms no
  existing signature or struct changes.
- **`src/engine`** — additive dispatch that resolves the sub-shape ids to native
  `EdgeCurve` / `FaceSurface`, calls `analysis::*`, and marshals the out-arrays; the
  OCCT oracle path lives ONLY in the sim test harness, never in `src/native/**`.
- **Cognitive complexity** — the distance dispatcher delegates per-pair to small
  closed-form helpers and one seed-and-refine helper (systems band ≤ 25 for the
  irreducible seed loop, flagged); curvature/angle helpers ≤ 12.
- **Determinism** — the seed-and-refine NURBS minimizer uses a fixed sample grid and
  deterministic tie-break, so results are reproducible.
- **Out of scope (declines, documented not faked):** minimum distance to a
  genuinely-trimmed freeform patch where the global optimum is not certifiable by the
  seeded minimizer; curvature at a parametric singularity (`EG−F² ≈ 0`); signed /
  directional curvature combs; distance between whole solids' full boundary BVH
  (this slice is per-named-entity-pair). No `cc_*` ABI break; no CyberCad app change;
  no OCCT linked into `src/native/**`.
