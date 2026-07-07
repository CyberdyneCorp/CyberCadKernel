# Design — add-native-wrap-emboss-breadth (#7 wrap-emboss, breadth tracks)

## Context

The archived `add-native-wrap-emboss` slice built ONE native case behind
`cc_wrap_emboss(body, faceId, profileXY, count, height, boss)`: a RAISED
RECTANGULAR pad on a CYLINDER lateral face (`boss = 1`). The builder
(`src/native/feature/wrap_emboss.h`) projects the footprint onto the cylinder
(`u = px/R`, `v = py + vMid`), builds the raised pad as a cap-and-side set (outer
cap at `R + height`, two axial radial-plane walls, two circumferential planar
strips), retiles the base wall over the FULL turn with the footprint WINDOW
removed, and welds the whole embossed solid as one deflection-bounded planar-facet
soup through the boolean `assembleSolid`. `NativeEngine::wrap_emboss` runs a
mandatory `wrapEmbossVerified` (watertight + volume grows by ≈ footprint-area ×
height) and DISCARDS a bad result → OCCT `cc_wrap_emboss`.

This change WIDENS that path along three independent tracks. The method stays
**clean-room** (the OCCT `cc_wrap_emboss` result is the verification ORACLE only,
never copied) and every track keeps the same discipline: build → mandatory
self-verify → OCCT fallback, NULL for anything not robustly buildable.

The key reusable primitives already in the header:
- `cylinderWall(solid, faceId)` → frame + radius + axial extent of a Cylinder
  lateral face (T3 adds a `coneWall` sibling).
- `USamples` / `uSamples` / `sagittaSteps` — the shared full-turn angular sample
  sequence with the window arc as an exact prefix, so every seam welds.
- `tileWallWithWindow` — the base wall over the full turn minus a RECTANGULAR
  window (T2 generalises the window to a polygon; T1 reuses it verbatim).
- `emitOuterCap` / `emitAxialWalls` / `emitCircWalls` / `emitEndCap` / `emitTri`
  / `emitFlat` — the deflection-bounded facet emitters, all sharing the window
  u-samples.
- `ringPoint(ax, rad, u, v)` — a point on a cylinder frame at radius `rad`,
  angle `u`, axial `v` (T3 adds a `conePoint`).

## Goals / Non-Goals

**Goals**
- **T1 DEBOSS**: a native RECESSED rectangular pocket on a cylinder lateral face
  (`boss = 0`), watertight, volume reduced by ≈ footprint-area × depth.
- **T2 NON-RECTANGULAR**: a native raised emboss for an arbitrary CLOSED N-vertex
  SIMPLE polygon footprint on a cylinder (convex robustly; simple non-convex
  gated by the self-verify).
- **T3 CONE base**: a native raised rectangular emboss on a CONE lateral face
  (normal-offset parallel-cone cap), as a narrow slice with an honest decline.
- Generalise `wrapEmbossVerified` to a SIGNED (grow/shrink) check using the true
  shoelace / surface-metric footprint area; keep the NULL → OCCT fallback.
- All behind the unchanged `cc_wrap_emboss` ABI; OCCT oracle for everything else.

**Non-Goals (return NULL → OCCT, never faked)**
- SPHERE and general freeform (BSpline/Bezier) base faces — T3 declines them.
- Self-intersecting / non-simple polygons, a polygon that wraps `> 2π`, or a
  footprint that exceeds the face's axial extent.
- Deboss whose `depth ≥ R` (floor would reach / cross the axis).
- Any combination whose weld cannot close watertight or whose volume change is
  outside the plausible band — the self-verify discards it.

## T1 — DEBOSS (recessed rectangular pocket, highest confidence)

**Geometry.** The raised pad pushes the footprint window OUT to `rOut = R + height`
and closes the base wall around it; deboss pushes it IN to `rFloor = R − depth` and
closes the base wall around a POCKET. The decomposition is identical with two
sign changes:

1. **Radial target.** Introduce a signed pad radius:
   `rTarget = boss ? R + height : R − depth`, where `depth = height` (the ABI's
   `depth` parameter; positive). Guard `depth < R − ε` so the floor stays strictly
   off the axis (else NULL → OCCT).
2. **Cap.** The raised OUTER cap (cylinder patch at `R + height`, outward normal
   `+radial`) becomes the pocket FLOOR (cylinder patch at `R − depth`, outward
   normal still `+radial` — the floor faces radially OUTWARD toward the pocket
   opening, away from the solid material beneath it at radius `< R − depth`).
3. **Side walls.** The axial and circumferential walls now span `ρ ∈ [R − depth, R]`
   and their outward normals FLIP to point INTO the pocket (toward the window
   interior) — the solid material is now OUTSIDE the window in the wall band.
4. **Base wall.** `tileWallWithWindow` (full turn minus the footprint window) and
   the two end caps are UNCHANGED — the solid outside the window is identical for
   boss and deboss.

The emit helpers already take a target outward-normal hint and orient each facet
by `dot(normal, outward)`; T1 supplies the flipped hints and the inward `rTarget`.
The whole soup still shares the window u-samples, so it welds watertight via
`assembleSolid` exactly as the raised pad does. Volume DECREASES by ≈ the wrapped
footprint area × depth.

**Confidence.** Highest — the pocket is the existing build with a sign flip; the
only genuinely new logic is the guard `depth < R` and the self-verify's shrink
branch.

## T2 — NON-RECTANGULAR polygon footprint

**Footprint.** Replace `rectFootprint` (4 axis-aligned corners) with a general
`polyFootprint`: `count ≥ 3` corners forming a CLOSED SIMPLE polygon in `(px, py)`
with positive shoelace area and no self-intersection. Project each corner to the
cylinder by the SAME map (`u = px/R`, `v = py + vMid`). An edge between two corners
that differ in BOTH `px` and `py` becomes a HELICAL curve on the cylinder (both
`u` and `v` vary); a purely-`px` edge is a circumferential arc, a purely-`py` edge
an axial segment.

**Cap.** Triangulate the polygon in `(u, v)` space — a fan for a convex polygon,
ear-clipping (`O(n²)`, robust for simple polygons) for a non-convex one. Each
`(u, v)` triangle maps to the cylinder at `R + height`; because only `u` carries
curvature (v is straight on a cylinder), subdivide each triangle's `u`-extent so
the angular sagitta `R(1 − cos Δu/2) ≤ deflection`, then map corners with
`ringPoint(ax, R + height, u, v)` and split each cell into planar triangles.

**Side walls.** One ruled strip per polygon EDGE, from `R` to `R + height`,
following the edge curve. A helical edge is tiled into deflection-bounded chords
(sagitta bound on its `u`-span); each chord's quad is split into two planar
triangles with the outward normal pointing away from the polygon interior.

**Base wall retiling (the hard part).** The base wall is a rectangle in the
`(u, v)` chart over the full turn `[uAnchor, uAnchor + 2π] × [vLo, vHi]` with the
POLYGON window removed. This is a polygon-with-rectangular-outer-boundary-and-
polygon-hole triangulation:
- **Convex window** — decompose the wall into the band above / below the window's
  `v`-range plus left / right strips beside it, fanning each strip to the polygon
  boundary vertices. Robust.
- **Simple non-convex window** (e.g. L-shape) — a constrained triangulation
  (bridge the outer rectangle to the hole and ear-clip the resulting simple
  polygon) that shares EVERY polygon-boundary vertex with the cap's inner edge and
  the side walls' R-side edge, so the seam welds. If the constrained triangulation
  cannot close (degenerate / near-self-intersecting projection), return NULL.

Every part reuses the SAME angular sample sequence along shared arcs, so the cap
inner edge, the side-wall R-edge, and the wall-window boundary coincide vertex-for-
vertex → `assembleSolid` welds watertight.

**Confidence.** Convex polygons build robustly; simple non-convex (L-shape) is
attempted and accepted ONLY if the mandatory self-verify (watertight + volume
grows by the SHOELACE area × height) passes — else NULL → OCCT. Self-intersecting
or `> 2π` footprints decline immediately.

## T3 — FREEFORM base: CONE lateral face (narrow slice / honest decline)

**Why cone, not sphere.** A cone lateral face parametrises as
`S(u, v) = O + (R₀ + v·sinα)(cos u·X + sin u·Y) + v·cosα·Z`. Its outward normal is
constant-angle to the axis (tilted by the half-angle α), and — critically — the
PARALLEL surface at normal offset `h` is again a coaxial cone with the SAME
half-angle and reference radius `R₀ + h/cosα` (an exact elementary surface). So the
raised cap stays an elementary cone patch and the cap-and-side + shared-vertex weld
transfers almost verbatim from the cylinder, with three adjustments:
1. `conePoint(frame, R₀, α, u, v)` replaces `ringPoint`; the local ring radius is
   `R(v) = R₀ + v·sinα`.
2. The pad is offset along the surface NORMAL `n̂(u) = cosα·(cos u·X + sin u·Y) −
   sinα·Z` (NOT purely radial); the cap corners are `S(u,v) + h·n̂`.
3. The sagitta bound uses the LOCAL ring radius `R(v)`; the base wall retiling,
   end structures (the cone's base disk stays a full disk, the apex is left
   untouched), and the window removal mirror the cylinder helpers.

A SPHERE lateral face (`R·cos v(cos u·X + sin u·Y) + R·sin v·Z`) is deferred: its
parallel surface is a concentric sphere (clean), but the base solid is the full
ball with a polar singularity at `v = ±π/2`, and the wrapped-footprint area varies
with latitude — the retiling seam and area-band are materially heavier. T3 declines
sphere → OCCT.

**Honest decline (no dead code).** T3 attempts the cone slice for real; the
mandatory self-verify (watertight + volume grows by the CONE-surface wrapped
footprint area × height) gates it. If the cone weld cannot be made watertight with
the correct volume for the target inputs, T3 returns NULL → OCCT for ALL cone
cases and the change RECORDS that honest decline with the measured gap — rather
than shipping a cone builder that is never accepted (which would be dead code). The
proposal ships whichever of {cone narrow slice, full honest decline} the sim
verify supports; the decision is recorded in tasks §6.

## Engine self-verify → OCCT fallback (generalised)

`wrapEmbossVerified(result, original, profileXY, count, height, boss)` (the `boss`
argument is ADDED) generalises the existing guard:

1. **Watertight + positive.** `watertightVolume(result) > 0` (else reject).
2. **Signed change.** With `vo = watertightVolume(original)`:
   - `boss != 0` (emboss): require `vr > vo` and `|vr − (vo + A·h)| ≤ tol`.
   - `boss == 0` (deboss): require `vr < vo` and `|vr − (vo − A·h)| ≤ tol`.
   where `h` is the ABI `height`/`depth` and `A` is the wrapped footprint area.
3. **Footprint area `A`.** SHOELACE polygon area of the `(px, py)` profile
   (`A = ½|Σ (xᵢ·yᵢ₊₁ − xᵢ₊₁·yᵢ)|`) — EXACT for the rectangle (matches the
   current bbox result) and correct for any polygon. For a CONE base, `A` is the
   wrapped footprint area on the cone surface (the local metric scales `px` by
   `R(v)/R₀`); a deflection-bounded band absorbs the curvature.
4. **Tolerance.** `tol = max(1e-2 · expected, floor)` — deflection-bounded, the
   SAME 1%-relative band the curved boolean guards use. NEVER weakened.

NULL builder OR a failing self-verify ⇒ OCCT `cc_wrap_emboss`. The engine SHALL
NEVER emit an unverified, leaky, or wrong-signed embossed solid.

## Module shape

```
src/native/feature/wrap_emboss.h        // EXTEND (OCCT-free, header-only):
  polyFootprint (T2)                     //   N-vertex simple polygon + shoelace
  coneWall / conePoint (T3)              //   cone lateral face + normal offset
  tileWallWithPolyWindow (T2)            //   wall minus polygon window (constrained)
  buildDebossedCylinder (T1)             //   inward pocket (rTarget = R − depth)
  wrap_emboss(...) dispatch              //   boss / rect|poly / cyl|cone routing
src/engine/native/native_engine.cpp
  wrapEmbossVerified(..., boss)          // GENERALISE: signed + shoelace/metric area
```

`NativeEngine::wrap_emboss` is otherwise unchanged: native body → native builder →
generalised self-verify → OCCT fallback.

## Cognitive complexity

The dispatch stays flat guard-clauses (backend band ≤ 15). Each track is isolated:
T1 is the existing build with signed offset + flipped hints; T2's ear-clip and
constrained wall triangulation are self-contained helpers (systems band 25–35,
documented Visitor-style tiling); T3's cone helpers mirror the cylinder ones. The
self-verify is a signed branch over one area computation. Measure with the
`cognitive-complexity` skill; split any tiling helper that exceeds ~35.

## Verification model (two gates)

- **Host (no OCCT; both default and NUMSCI configs):** extend
  `tests/native/test_native_wrap_emboss.cpp`:
  - **T1** — deboss a rectangular pocket on a native cylinder; assert watertight
    (`boundaryEdgeCount == 0`) and volume `= |cyl| − footArea·depth` within the
    deflection bound; assert `depth ≥ R` returns NULL.
  - **T2** — emboss a hexagon and an L-shape; assert watertight and volume
    `= |cyl| + shoelaceArea·height`; assert a self-intersecting polygon returns
    NULL.
  - **T3** — emboss a rectangle on a native cone lateral face; assert watertight
    and volume `= |cone| + coneWrappedArea·height` (if the cone slice ships);
    assert a sphere base returns NULL. If T3 is a full decline, assert the cone
    builder returns NULL and the call defers.
  - **control** — the raised-rectangular-pad-on-cylinder case is unchanged.
- **Sim native-vs-OCCT parity** (`scripts/run-sim-native-wrap-emboss.sh` +
  `tests/sim/native_wrap_emboss_parity.mm`, EXTENDED): on a booted simulator,
  compute each track's `cc_wrap_emboss` under the native engine (`cc_set_engine(1)`)
  and under OCCT (`cc_set_engine(0)`); compare volume / surface area /
  watertightness / `BRepCheck_Analyzer::IsValid` within the deflection-bounded
  tolerance and RECORD the measured deltas. Confirm no regression: the raised-
  rectangular-pad-on-cylinder control (existing 6/6), the OCCT wrap-emboss (#290,
  `run-sim-phase3-suite.sh`), the native curved fillet, and `run-sim-suite.sh` all
  stay green.

## Decisions

- **Deboss first (T1).** A pure sign flip on the proven raised-pad build — the
  lowest-risk breadth and the one the roadmap names next.
- **Shoelace area in the self-verify.** The bbox area is only correct for a
  rectangle; T2 requires the true polygon area so a non-rectangular emboss is
  gated on its actual added material. It also keeps the rectangle exact.
- **Cone via normal-offset parallel cone.** The cone's parallel surface is a clean
  coaxial cone, so the elementary cap-and-side + shared-vertex weld transfers; the
  sphere's full-ball + polar seam does not, so sphere declines.
- **Honest decline for the hard edges.** Self-intersecting polygons, sphere/
  freeform bases, over-2π / off-end footprints, and `depth ≥ R` all return NULL →
  OCCT. T3 may collapse to a full decline; the sim verify decides and the outcome
  is recorded — no dead code either way.

## Risks / Trade-offs

- **T2 non-convex robustness.** The constrained wall-minus-polygon triangulation
  is the riskiest T2 piece; convex polygons are robust, non-convex is gated by the
  self-verify. If ear-clipping produces slivers that fail watertightness, the
  result is discarded → OCCT — never a leaky pad.
- **T3 cone-area band.** The cone's wrapped footprint area varies with `v`; the
  self-verify uses a deflection-bounded band. A borderline cone (large half-angle,
  footprint near the apex) returns NULL → OCCT rather than a fragile accept.
- **T3 may not land as a slice.** If the cone weld can't be made watertight, T3 is
  an honest decline; the design's cylinder/deboss/polygon value (T1 + T2) stands
  independently.
- **Slice narrowness vs honesty.** Sphere, general freeform bases, and dense/self-
  intersecting profiles remain OCCT. Accepted — faking a wrap-emboss is forbidden;
  the measured OCCT-fallback gap is reported, never masked.
