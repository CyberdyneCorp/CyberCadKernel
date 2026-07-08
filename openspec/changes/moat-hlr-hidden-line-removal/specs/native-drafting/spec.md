# native-drafting

## ADDED Requirements

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
cannot robustly classify. In this slice the DECLINED cases SHALL include: the
**silhouette outline of a curved face** (cylinder/cone/sphere generators and
tangent end-ellipse arcs) that is not represented by a projected topological
edge; **freeform** (B-spline/Bézier/NURBS) faces; a **view direction along an
edge** (degenerate projection); and any sample whose occlusion is within
tolerance of ambiguous (grazing / coincident faces). A decline SHALL NEVER be
silently converted into a visible-or-hidden guess, and accepting an input SHALL
imply its classification is correct to the stated tolerances.

#### Scenario: A curved-surface silhouette outline is declined, not approximated

- GIVEN a request whose visible outline is the curved SILHOUETTE of a cylinder/cone/sphere (where `n·viewDir = 0`) rather than a projected topological edge
- WHEN the drafting service is asked to produce that outline in this slice
- THEN it SHALL DECLINE (report the silhouette case as unsupported) AND SHALL NOT emit mesh-edge segments approximating the true curved outline

#### Scenario: A freeform face is declined honestly

- GIVEN a solid bounded by a freeform (B-spline / Bézier / NURBS) face
- WHEN hidden-line removal is requested for it
- THEN the service SHALL DECLINE (report freeform faces as unsupported) rather than emit a visible/hidden classification for the freeform silhouette

### Requirement: Two-gate verification against the analytic answer and OCCT

The native HLR result SHALL be verified against two independent oracles. GATE (a)
HOST ANALYTIC (no OCCT) SHALL check, with plain `clang++ -std=c++20`, that known
solids from known views match the closed-form answer (the box isometric-corner
`9 visible + 3 hidden`, the no-occluder `12 visible`, the half-occluded edge
split, and projected-length conservation). GATE (b) SIM native-vs-OCCT SHALL, on
a booted iOS simulator, match OCCT `HLRBRep_Algo` / `HLRBRep_HLRToShape` visible
and hidden compounds on segment **count**, **total projected length**, and
**endpoint positions** within tolerance. Only the simulator parity harness SHALL
link OCCT; the native drafting library SHALL remain OCCT-free.

#### Scenario: The host analytic gate passes with no OCCT and no simulator

- GIVEN the always-on host test `tests/native/test_native_drafting.cpp` built with plain `clang++ -std=c++20` (no OCCT, no simulator)
- WHEN it is run
- THEN all cases SHALL pass — the box isometric-corner `9 visible + 3 hidden`, the no-occluder `12 visible / 0 hidden` baseline, the half-occluded edge split, and projected-length conservation

#### Scenario: The native library links no OCCT

- GIVEN the `src/native/drafting/**` sources
- WHEN they are scanned for OCCT usage AND compiled for host and for the arm64 iOS simulator
- THEN they SHALL contain ZERO OCCT includes (including only `src/native/math`) AND SHALL compile for both targets, so the drafting core is OCCT-free on the ABI-relevant platforms

### Requirement: Additive cc_* accessor for drawing projection

The native HLR core SHALL be exposed to the app behind a NEW, ADDITIVE plain-C
accessor `cc_hlr_project(body, viewDir, up, opts) -> CCDrawing` that returns the
**visible** and **hidden** 2D drawing-plane edge-segment sets (flat arrays owned
by the caller and released with `cc_drawing_free`), so the app's
`DrawingProjector` / `ProjectEdges` / `ProjectBody` path can consume native HLR
in place of OCCT's visible/hidden compounds. The accessor and its PODs
(`CCDrawing`, `CCDrawingSegment`, `CCHlrOptions`) SHALL be **additive only**: no
existing `cc_*` signature SHALL change, and no engine or C++ type SHALL cross the
facade. When the requested drawing contains a declined case (a curved silhouette
or a freeform face) the accessor SHALL return an empty `CCDrawing` with
`cc_last_error` set, and SHALL NOT return a partial or wrong drawing.

#### Scenario: The accessor is additive and returns visible/hidden segment sets

- GIVEN the existing `cc_*` ABI
- WHEN `cc_hlr_project(...)` and its PODs are added
- THEN every existing `cc_*` signature SHALL be unchanged AND `cc_hlr_project` SHALL return a `CCDrawing` carrying separate visible and hidden 2D segment arrays owned by the caller and freed by `cc_drawing_free`

#### Scenario: A declined drawing returns empty with an error, never a wrong drawing

- GIVEN a `body` whose drawing requires a case declined in this slice (a curved silhouette outline or a freeform face)
- WHEN `cc_hlr_project(...)` is called
- THEN it SHALL return an empty `CCDrawing` (null/zero segment arrays) with `cc_last_error` set AND SHALL NOT return a partial drawing or a wrong visible/hidden classification
