# native-drafting Specification

## Purpose
TBD - created by archiving change moat-hlr-hidden-line-removal. Update Purpose after archive.
## Requirements
### Requirement: Orthographic hidden-line removal over an analytic/polyhedral solid

The native drafting service SHALL perform **orthographic (parallel) hidden-line
removal** for the analytic/polyhedral core (box / prism / cylinder / cone /
sphere and simple combinations), producing from a set of world-space straight
edges and a triangle occluder mesh two DISJOINT sets of 2D drawing-plane
segments — **visible** and **hidden** — via `projectOrthographic`. It SHALL be
**OCCT-FREE** (`src/native/drafting/**` includes only `src/native/math`; zero
OCCT includes) and header-only. Each edge SHALL be projected onto the drawing
plane whose orthonormal basis is `right = normalize(viewDir × up)`,
`trueUp = right × viewDir`, with a world point mapping to `(P·right, P·trueUp)`.
A projected edge sample SHALL be classified **HIDDEN** when, and only when, a
surface of the solid lies strictly nearer the parallel viewpoint along that line
of sight (a ray from the sample toward the viewpoint crosses an occluder triangle
at a strictly positive distance, after the sample is nudged toward the camera by
`surfaceOffset` to discount the edge's own coplanar adjacent faces); otherwise it
SHALL be **VISIBLE**. The union of the emitted visible and hidden segments SHALL
cover the whole projected edge with no gaps and no overlaps, so total projected
length is conserved across the visible/hidden split. The service SHALL NEVER emit
a wrong classification (a hidden edge as visible or a visible edge as hidden) for
an input it accepts.

#### Scenario: A box from an isometric corner yields 9 visible + 3 hidden segments (host, no OCCT)

- GIVEN a unit box (half-extent 1, centred at the origin) as 8 corners, its 12 straight edges, and its 12-triangle boundary occluder
- AND an orthographic view down an isometric corner (`viewDir = normalize(-1,-1,-1)`, up hint `(0,0,1)`)
- WHEN `projectOrthographic(...)` is called
- THEN the result SHALL contain EXACTLY 9 visible segments AND EXACTLY 3 hidden segments AND the 3 hidden segments SHALL share one common endpoint (the projection of the occluded far corner) within `1e-7`

#### Scenario: With no occluder every edge is fully visible (host, no OCCT)

- GIVEN the same box's 12 edges and an EMPTY occluder (no triangles)
- WHEN `projectOrthographic(...)` is called with the same isometric view
- THEN the result SHALL contain EXACTLY 12 visible segments AND 0 hidden segments

#### Scenario: Total projected length is conserved across the visible/hidden split (host, no OCCT)

- GIVEN the box, its edges, its occluder and the isometric view
- WHEN `projectOrthographic(...)` is called
- THEN the sum of the visible segment lengths plus the hidden segment lengths SHALL equal the sum of the raw projected lengths of all 12 edges within `1e-6`

### Requirement: Edge splitting at visibility transitions

`projectOrthographic` SHALL classify each edge along its length (sampling at cell
MIDPOINTS, i.e. strict edge interiors, never at the exact endpoints, so that
silhouette-corner ambiguity cannot produce spurious slivers) and SHALL SPLIT an
edge at every transition between visible and hidden. The split parameter SHALL be
refined by bisection to `transitionTol`, and the resulting segments SHALL be
disjoint and together span the full edge. A partially occluded edge SHALL
therefore emit both a visible and a hidden portion, each meeting exactly at the
occlusion boundary. Degenerate (zero-length) projected segments SHALL be dropped
using `dropSegmentLen`.

#### Scenario: An edge half-covered by a nearer face splits into one visible + one hidden segment (host, no OCCT)

- GIVEN a straight edge from `(-5,0,0)` to `(5,0,0)` viewed down `-Z` (projection `= (x,y)`)
- AND an occluder quad at `z = +1` (nearer the camera) covering `x ∈ [0,6], y ∈ [-1,1]`
- WHEN `projectOrthographic(...)` is called
- THEN the result SHALL contain EXACTLY 1 visible segment (the `x ∈ [-5,0]` half) AND EXACTLY 1 hidden segment (the `x ∈ [0,5]` half) AND both SHALL meet at the coverage boundary `x = 0` within `1e-6`

### Requirement: Honest decline for cases the native path cannot classify correctly

The drafting service SHALL return an honest decline (an error / unsupported
status) — rather than emit a possibly-wrong drawing — for any configuration it
cannot robustly classify. In this slice the service TRACES the closed-form
silhouette of a **cylinder** and a **sphere** face (emitted through the shared
occlusion/split path), and the DECLINED cases SHALL include: the silhouette of a
**cone** (apex singularity + flank/base handling), a **torus** (a quartic,
self-occluding silhouette), and any **freeform** (B-spline/Bézier/NURBS) face; a
**partial/trimmed quadric** whose silhouette generator leaves the face's `(u,v)`
trim window (ambiguous outline); an **axis-parallel-degenerate** cylinder view
within the near-threshold band; a **view direction along an edge** (degenerate
projection); and any sample whose occlusion is within tolerance of ambiguous
(grazing / coincident faces). A decline SHALL NEVER be silently converted into a
visible-or-hidden guess or a fabricated/mesh-edge outline, and accepting an input
SHALL imply its classification is correct to the stated tolerances.

#### Scenario: A cone, torus or freeform silhouette is declined, not approximated

- GIVEN a request whose visible outline is the curved SILHOUETTE of a CONE, a TORUS, or a FREEFORM (B-spline/Bézier/NURBS) face
- WHEN the drafting service is asked to produce that outline in this slice
- THEN it SHALL DECLINE (report the silhouette case as unsupported) AND SHALL NOT emit mesh-edge segments approximating the true curved outline

#### Scenario: A cylinder or sphere silhouette is traced, not declined

- GIVEN a solid whose outline includes the silhouette of a CYLINDER or a SPHERE face traceable in this slice
- WHEN hidden-line removal is requested for it
- THEN the service SHALL TRACE the closed-form silhouette and classify it through the shared occlusion/split path, rather than decline

#### Scenario: A partial quadric with an out-of-window silhouette is declined honestly

- GIVEN a trimmed cylinder whose silhouette generator falls outside the face's `(u,v)` trim window, so the outline is ambiguous
- WHEN hidden-line removal is requested for it
- THEN the service SHALL DECLINE rather than emit a clipped or extrapolated outline or a visible/hidden guess

### Requirement: Two-gate verification against the analytic answer and OCCT

The native HLR result SHALL be verified against two independent oracles. GATE (a)
HOST ANALYTIC (no OCCT) SHALL check, with plain `clang++ -std=c++20`, that known
solids from known views match the closed-form answer — the polyhedral cases (the
box isometric-corner `9 visible + 3 hidden`, the no-occluder `12 visible`, the
half-occluded edge split, projected-length conservation) AND the quadric-silhouette
cases (a sphere projecting to a circle of radius `r`, fully visible, length `2πr`;
a cylinder side-on giving two visible generator segments of length `h` separated
by `2r` plus split cap arcs; an axis-parallel cylinder emitting no generators).
GATE (b) SIM native-vs-OCCT SHALL, on a booted iOS simulator, match OCCT
`HLRBRep_Algo` / `HLRBRep_HLRToShape` visible and hidden compounds on segment
**count**, **total projected length**, and **endpoint positions** within
tolerance for the polyhedral solids AND for a cylinder and a sphere, while a cone
and a torus SHALL be asserted DECLINED, not compared. Only the simulator parity
harness SHALL link OCCT; the native drafting library (including `silhouette.h`)
SHALL remain OCCT-free.

#### Scenario: The host analytic gate passes with no OCCT and no simulator

- GIVEN the always-on host test `tests/native/test_native_drafting.cpp` built with plain `clang++ -std=c++20` (no OCCT, no simulator)
- WHEN it is run
- THEN all cases SHALL pass — the polyhedral box isometric-corner `9 visible + 3 hidden`, the no-occluder `12 visible / 0 hidden`, the half-occluded edge split, projected-length conservation, the sphere circle (radius `r`, fully visible, length `2πr`), the cylinder side-on (two visible generators length `h` separated by `2r` + split cap arcs), and the axis-parallel cylinder (no generators)

#### Scenario: The simulator gate matches OCCT HLRBRep_Algo on polyhedral, cylinder and sphere solids

- GIVEN the simulator harness `tests/sim/native_hlr_parity.mm` driving `cc_hlr_project` on a booted iOS simulator under BOTH engines — `cc_set_engine(0)` (OCCT `HLRBRep_Algo` oracle) and `cc_set_engine(1)` (the OCCT-free native core) — for the SAME solids (box iso + oblique, triangle prism, non-convex L-prism, a cylinder side-on + oblique, and a sphere) built identically under each engine
- WHEN each solid is projected both ways and the native visible/hidden 2D segment sets are compared against the OCCT sets
- THEN the native result SHALL match the OCCT oracle on visible COUNT, hidden COUNT, total projected LENGTH (relative tolerance ≤ `1e-4`), and endpoint PARTITION (every native segment covered by an OCCT segment of the same visible/hidden class within `1e-5`, and vice versa) AND a cone AND a torus solid SHALL be asserted DECLINED, not compared — with NO tolerance widening to mask a divergence

#### Scenario: The native library links no OCCT

- GIVEN the `src/native/drafting/**` sources (including the new `silhouette.h`)
- WHEN they are scanned for OCCT usage AND compiled for host and for the arm64 iOS simulator
- THEN they SHALL contain ZERO OCCT includes (including only `src/native/math`) AND SHALL compile for both targets, so the drafting core is OCCT-free on the ABI-relevant platforms

### Requirement: Additive cc_* accessor for drawing projection

The native HLR core SHALL be exposed to the app behind the existing ADDITIVE
plain-C accessor `cc_hlr_project(body, viewDir, up, opts) -> CCDrawing` that
returns the **visible** and **hidden** 2D drawing-plane edge-segment sets (flat
arrays owned by the caller and released with `cc_drawing_free`). This slice
EXTENDS its COVERAGE — a cylinder or sphere body now returns a populated
`CCDrawing` carrying its traced silhouette outline classified visible/hidden —
WITHOUT changing the accessor's signature or its PODs (`CCDrawing`,
`CCDrawingSegment`, `CCHlrOptions`). The accessor and its PODs SHALL remain
**additive only**: no existing `cc_*` signature SHALL change, and no engine or
C++ type SHALL cross the facade. When the requested drawing contains a case
declined in this slice (a cone/torus/freeform silhouette, or a partial quadric
with an out-of-window silhouette) the accessor SHALL return an empty `CCDrawing`
with `cc_last_error` set, and SHALL NOT return a partial or wrong drawing.

#### Scenario: The accessor signature is unchanged while cylinder/sphere coverage is added

- GIVEN the existing `cc_hlr_project` signature and its PODs
- WHEN cylinder + sphere silhouette coverage is added to the native engine behind it
- THEN every existing `cc_*` signature SHALL be unchanged (COVERAGE extension only) AND `cc_hlr_project` SHALL return a `CCDrawing` carrying separate visible and hidden 2D segment arrays owned by the caller and freed by `cc_drawing_free`

#### Scenario: A cylinder or sphere body returns a populated drawing

- GIVEN a `body` that is a cylinder or a sphere whose silhouette is traceable in this slice
- WHEN `cc_hlr_project(...)` is called under the native engine
- THEN it SHALL return a NON-empty `CCDrawing` whose visible/hidden segment sets include the traced silhouette outline classified through the shared occlusion/split path, with no `cc_last_error` set

#### Scenario: A declined drawing returns empty with an error, never a wrong drawing

- GIVEN a `body` whose drawing requires a case declined in this slice (a cone/torus/freeform silhouette, or a partial quadric with an ambiguous out-of-window silhouette)
- WHEN `cc_hlr_project(...)` is called
- THEN it SHALL return an empty `CCDrawing` (null/zero segment arrays) with `cc_last_error` set AND SHALL NOT return a partial drawing or a wrong visible/hidden classification

### Requirement: Quadric-face silhouette tracing for orthographic HLR (cylinder + sphere)

The native drafting service SHALL trace the **closed-form silhouette locus** of a
**cylinder** and a **sphere** analytic face — the curve on the face where the
outward surface normal is perpendicular to the view direction (`n · viewDir = 0`)
— and emit it as first-class world-space projected polyline(s) that are fed
through the SAME occlusion + visibility-split path (`projectOrthographic`) as
topological edges, so the visible/hidden classification is reused verbatim. This
SHALL be provided by an additive OCCT-FREE helper
`src/native/drafting/silhouette.h` (`cybercad::native::drafting`, header-only,
including ONLY `native/math`); the polyhedral `orthographic_hlr.h` code path and
its parameters SHALL remain BYTE-IDENTICAL for polyhedral inputs.

- For a **sphere** (centre `C`, radius `r`) the silhouette SHALL be the great
  circle in the plane through `C` perpendicular to `viewDir`, `P(t) = C +
  r·(cos t·right + sin t·trueUp)`, whose orthographic projection is an exact
  circle of radius `r` centred at the projection of `C`.
- For a **cylinder** (axis `A`, radius `r`, finite height `h`) the silhouette
  SHALL be the two diametrically opposite generator segments at
  `θ* = atan2(−(X·viewDir), (Y·viewDir))` and `θ*+π`, each swept over the face's
  axial (`v`) range `s ∈ [0,h]`.

The helper SHALL return an honest "not traceable" status (`traced=false`) — and
the request SHALL DECLINE, NEVER emitting a guessed or clipped polyline — for:
an **axis-parallel-degenerate** cylinder view within the near-threshold band; a
**partial/trimmed quadric** whose silhouette generator leaves the face's `(u,v)`
trim window; and a **cone, torus, or freeform** (`BSpline`/`Bézier`) face. When
the view looks exactly down a cylinder's axis the helper SHALL emit **no**
generators (the cap edge is the outline) rather than a near-degenerate line —
a correct special case, not a decline. The service SHALL NEVER emit
mesh-edge segments approximating a curved outline in place of the closed-form
silhouette.

#### Scenario: A sphere projects to a circle of radius r, fully visible (host, no OCCT)

- GIVEN a sphere of radius `r` centred at `C` and an arbitrary orthographic view
- WHEN its silhouette is traced and projected
- THEN the outline SHALL be a CLOSED circle of radius `r` centred at the projection of `C` within `1e-9`, it SHALL be classified FULLY VISIBLE with 0 hidden segments, AND the total projected outline length SHALL equal `2πr` within `1e-6`

#### Scenario: A cylinder side-on yields two visible generator lines plus split cap arcs (host, no OCCT)

- GIVEN a cylinder of radius `r` and height `h` viewed with its axis PERPENDICULAR to `viewDir` (side-on)
- WHEN its silhouette generators are traced and projected together with its two circular cap edges
- THEN the result SHALL contain EXACTLY two VISIBLE generator segments, each of projected length `h`, separated by `2r` within `1e-9`, AND each cap circle SHALL split into a visible front arc and a hidden back arc, AND total projected length (generators + cap arcs) SHALL be conserved across the visible/hidden split within `1e-6`

#### Scenario: A view down the cylinder axis emits no generators, not a degenerate line (host, no OCCT)

- GIVEN a cylinder viewed with `viewDir` parallel to its axis (`‖viewDir − (viewDir·A)A‖` below the axis-parallel tolerance)
- WHEN its silhouette is traced
- THEN the helper SHALL emit NO generator segments (the cap edge carries the outline) rather than a near-degenerate generator, AND this SHALL NOT be reported as a decline

#### Scenario: A partial quadric whose generator leaves the trim window is declined, not clipped

- GIVEN a trimmed cylinder (a sector / fillet flank) whose silhouette generator `θ*` falls OUTSIDE the face's `(u,v)` trim window
- WHEN its silhouette is traced
- THEN the helper SHALL return `traced=false` and the request SHALL DECLINE, AND SHALL NOT emit a clipped or extrapolated generator

