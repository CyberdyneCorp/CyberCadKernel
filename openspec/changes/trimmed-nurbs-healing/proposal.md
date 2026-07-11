# Proposal — trimmed-nurbs-healing (NURBS roadmap Layer 8, healing extension)

## Why

`trimmed-nurbs-brep-model` landed the Layer-8 trimmed-NURBS data model + `classify()` +
pcurve fidelity/construction, but it explicitly DECLINED (`Unknown`) any loop that was only
*near*-valid: a loop with a small gap between consecutive pcurve endpoints, two pcurves that
should share a boundary but sit a hair apart, or a loop that self-touches at a point. Those are
the everyday defects a STEP-read or boolean-produced trimmed face carries. Declining all of them
throws away otherwise-usable faces.

This change adds a **bounded, region-preserving tolerant-topology HEALING** pass so common
near-valid loops become valid — while a genuinely broken loop still declines honestly. Healing is
the difference between a data model that only accepts pristine input and one a real B-rep boolean
(Layer 3) can feed off of.

## What

Extend `src/native/topology/trimmed_nurbs.{h,cpp}` (least-invasive: no `shape.h` change, no `cc_*`
change, `src/native` stays OCCT-free, only existing math is `#include`d) with:

1. **`healLoop()`** — the healing primitive over a flattened loop polyline (+ its segment-join
   gaps):
   - **GAP CLOSING** — consecutive pcurve endpoints (including the closing join) within a
     scale-relative `gapTol` but not coincident are SNAPPED to their midpoint (the loop is welded
     closed).
   - **NEAR-COINCIDENT PCURVE SNAP** — the same weld makes two near-coincident pcurve endpoints
     share a single boundary vertex.
   - **LARGE-GAP decline** — a segment join whose gap exceeds `gapTol` is a genuine gap: the loop
     is DECLINED (`largeGap`), never force-welded into a fabricated region.
   - **PINCH detection** — a repeated non-adjacent vertex (self-touch) is DETECTED (`pinch`).
   Reports what was healed via `HealReport` (healed / changed / gapsClosed / maxGapClosed /
   pinch / largeGap / residualGap / tolerance).

2. **`healTrimLoop()`** — flatten a `TrimLoop` (join-gap-aware) + heal, for diagnosis.

3. **`classify()` runs healing** — new `ClassifyOptions::heal` (default on) + `healGapTol`. Each
   loop is healed before the raycast; a heal failure (large gap / pinch / degeneracy) declines
   `Unknown` exactly as before, so no previously-accepted input regresses and no previously-declined
   broken loop is now force-accepted.

4. **PINCH-SPLITTING** — `splitAtPinch()` / `splitTrimLoopAtPinch()` (returning a `SplitReport`)
   resolve a CLEAN 2-way pinch (a figure-eight self-touching at exactly one vertex) into TWO
   independent sub-loops that share only the pinch vertex. This is **region-preserving**: the two
   disjoint lobes' union classifies every point IDENTICALLY to the original pinched loop. A 3+-way /
   crossing pinch is `ambiguous` and DECLINED honestly (never force-split). `classify()` exposes it
   as an OPT-IN option `ClassifyOptions::splitPinch` (default OFF, so a pinch still declines
   `Unknown` unless the caller asks to split); with it ON a cleanly-splittable pinched outer loop
   classifies via the union of its two sub-loops.

Every heal is **SCALE-RELATIVE** and **REGION-PRESERVING**: a weld moves a vertex by at most
`gapTol/2`, so no interior/exterior point farther than that band from the boundary can change its
`In`/`Out` verdict — a heal never flips the region.

## Scope / residuals

- Healing covers the common near-valid cases (small gaps, near-coincident vertices, pinch
  detection) PLUS clean 2-way pinch **splitting** (a figure-eight → two region-preserving
  sub-loops). **General non-manifold repair** — resolving a 3+-way / crossing pinch, healing across
  surface seams, re-parametrizing a badly-drifting pcurve — remains a documented residual (declined
  honestly as `ambiguous` / `Unknown`, never fabricated).
- The **rational residual** (`constructPcurve` fits non-rationally and reports the true deviation)
  is unchanged by this change.

`cc_*` unchanged; `src/native` stays OCCT-free; `src/native/math`, `ssi`, `blend`, `boolean` are
untouched (only `#include`d). No existing `FaceSurface`/`PCurve`/`step_reader` consumer changes.
