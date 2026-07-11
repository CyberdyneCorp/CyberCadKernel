# Proposal — trimmed-nurbs-brep-model (NURBS roadmap Layer 8)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 8 is the
**trimmed-NURBS B-rep data model + pcurve robustness** — the foundational prerequisite for the
eventual Layer-3 exact-NURBS B-rep boolean (BOPAlgo-class). A trimmed NURBS **face** is a
surface `S(u,v)` bounded by **trimming loops** in the surface's `(u,v)` parameter plane; each
loop is an ordered sequence of **pcurves** (2-D curves). The load-bearing robustness property is
**pcurve fidelity**: the 3-D edge curve `C(t)` must equal `S(pcurve(t))` (the surface evaluated
along the pcurve) within tolerance — without it a NURBS B-rep boolean's shared seams crack.

This slice is worth building **now** because it is (a) small and well-bounded — a data-model +
robustness layer, not a heavy algorithm; (b) built entirely on machinery that already exists —
the kernel ALREADY admits B-spline surfaces + trims from STEP (`shape.h` `FaceSurface`
`Kind::BSpline`; `step_reader.cpp` reads trims), and Layer-7 fitting (`bspline_fit`) + the
numerics projection (`closest_point_on_surface`) are landed; and (c) **uniquely airtight to
verify** — pcurve fidelity has a machine-precision oracle (an exact iso-curve), and pcurve
construction reconstructs a KNOWN `(u,v)` path (the strongest oracle available).

## What

A new OCCT-free module `src/native/topology/trimmed_nurbs.{h,cpp}` (namespace
`cybercad::native::topology`), a **robust operations layer ON TOP of the existing `shape.h` data
model** — it adds NOTHING to `shape.h` and changes no existing `FaceSurface`/`PCurve`/`step_reader`
consumer (least-invasive: the existing `FaceSurface` + `PCurve` + `EdgePCurve` already ARE the
storage; this module provides the robust operations a boolean needs and that the mesher-oriented
`tessellate/trim.h` deliberately does not promise). It provides:

1. **Data model** — `TrimmedNurbsFace` = surface (`FaceSurface`) + `Location` + outer trim loop +
   inner (hole) loops, each loop an ordered list of `PcurveSegment` (a `PCurve` + `[first,last]`
   range + orientation) in `(u,v)`. Buildable from an existing topology face `Shape` (reusing its
   stored pcurves) or assembled directly. Stores enough to answer "is `(u,v)` inside?".

2. **Point-in-trimmed-region** classification — `classify(u,v) → {In, Out, OnBoundary, Unknown}`
   by even-odd ray-cast in PARAMETER space, keeping the kept region inside the outer loop and
   outside every hole. On-edge points → `OnBoundary`; a point in a hole → `Out`; empty / open /
   self-touching / degenerate loops → **`Unknown`** (honest decline, not a fabricated verdict).

3. **Pcurve fidelity guard** — `pcurveFidelity(S, C, p, …)` verifies `S(p(t)) == C(t)` on a dense
   sample within a **scale-relative** tolerance and reports the max deviation; a deliberately-wrong
   pcurve (not on `S`) is DETECTED (large deviation flagged), never passed. And **pcurve
   construction** (numsci-gated) — `constructPcurve` builds a pcurve for a 3-D edge lying on `S`
   by projecting sampled points to `(u,v)` via `numerics::closest_point_on_surface` and fitting a
   2-D B-spline via `bspline_fit`, then round-trip-verifies fidelity.

The always-on legs (data model, classify, fidelity) use only `src/native/math` +
`src/native/topology`. `constructPcurve` is compiled only under `CYBERCAD_HAS_NUMSCI` (it depends
on the numerics facade + `bspline_fit`, exactly like `src/native/math/bspline_fit.cpp`); with the
guard OFF that one function is simply absent and everything else builds and tests without it.

## Scope / residuals

This is a data-model + robustness slice, not a full tolerant B-rep. Documented residuals (declined
honestly, never faked):
- General **tolerant-topology HEALING** — auto-closing gapped loops, resolving self-touching pinch
  points into a valid topology, snapping near-coincident pcurves — is a residual; those cases are
  declined `Unknown`, not silently repaired.
- `constructPcurve` is **non-rational** (`bspline_fit` is non-rational); a rational edge is fitted
  as a non-rational approximation and its true deviation is reported (never a widened tolerance).
- The Layer-3 exact-NURBS B-rep boolean this feeds is out of scope here.

`cc_*` unchanged; `src/native` stays OCCT-free; `src/native/math`, `ssi`, `blend`, `boolean` are
untouched (only `#include`d). No existing `FaceSurface`/`step_reader` consumer changes.
