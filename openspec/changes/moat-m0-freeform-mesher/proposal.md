# Proposal — moat-m0-freeform-mesher (MOAT M0, the keystone)

## Why

Every bounded native slice this project shipped declined at ONE recurring blocker:
a **general foreign trimmed B-spline/NURBS surface patch cannot be tessellated
watertight**. Our OWN B-spline faces mesh fine — they are bare-periodic
`VERTEX_LOOP` faces (revolution / extrude) that the mesher routes through the
**structured-grid** path, which carries interior curvature sampling. A *foreign*
patch is different: it has a genuinely **trimmed `EDGE_LOOP`** bound whose edges
carry **pcurves**, so it fails the full-parametric-rectangle test and is routed to
the **ear-clip** path — and that path **adds NO interior sample points** (verified:
`uv_triangulate.h` triangulates only the boundary polygon; its own comment says
faces "that need INTERIOR curvature sampling are meshed by the … structured-grid
path"). For a *planar* trimmed face that is exact (flat). For a *curved* trimmed
patch the interior is unsampled, so the web of boundary chords is off-surface, the
chord deflection is unbounded and the enclosed volume is wrong — the mesh fails the
engine's watertight/volume self-verify and the whole import **declines to OCCT**.

This single gap sits under **freeform booleans (M2)**, **freeform blends /
wrap-emboss (M3)**, and **foreign-B-spline STEP import (M4)**. It is stage **M0** —
the keystone — and unblocks the most downstream work per unit effort
(`openspec/MOAT-ROADMAP.md`). It is exactly the gap the foreign rational-B-spline
STEP slice hit (`src/native/exchange/step_reader.cpp`).

A general trimmed-NURBS mesher is a ~1.5–3 person-year capability; this change is
the **FIRST bounded slice**, not the general mesher. It lands ONE concrete foreign
trimmed B-spline/NURBS face — a pcurve-bounded `EDGE_LOOP` over a B-spline surface
whose boundary pcurves are reconstructable faithfully (line / circle / ellipse in
`(u,v)`, or a supplied B-spline pcurve) — meshed **watertight** into a solid that
matches OCCT `BRepMesh`. Where the foreign patch's pcurves cannot be reconstructed
faithfully, or the watertight self-verify fails, the reader keeps the **honest
decline** (the patch stays OCCT). A correct decline is a first-class, expected
outcome; a non-watertight mesh is never emitted and no dead code is written.

## What Changes

1. **An additive trimmed-freeform mesh branch** in `src/native/tessellate/
   face_mesher.h`, taken ONLY for a `Kind::BSpline` / `Kind::Bezier` face that is
   **genuinely trimmed** (real `EDGE_LOOP`, fails `isFullRectangle`). Today such a
   face routes to `earClipMesh` and produces an off-surface, non-watertight web that
   the engine discards — i.e. it is **currently a decline**, reachable by NO passing
   test. The new branch:
   - flattens the `EDGE_LOOP` to a UV boundary polygon at the **shared per-edge
     fractions** (the existing STAGE-2 `flattenWireShared` machinery, so the two
     faces of every shared edge place BIT-IDENTICAL boundary points — the weld
     contract) and records the canonical seam anchors;
   - samples the surface **INTERIOR** of the trim region on a curvature-driven UV
     grid (existing `worstCurvature` / `divisionsFor` sizing on the patch), keeping
     only points **inside the outer loop and outside every hole** (existing
     `UVRegion::inside` even-odd test);
   - triangulates **boundary + interior** with a **boundary-constrained** UV
     triangulation that PRESERVES every boundary segment (so the boundary edges
     remain exactly the shared-edge samples) and fills the interior; every vertex is
     `S(u,v)` on the true surface, boundary vertices snapped to their anchors.
   The existing **planar** ear-clip path and **structured-grid** path are UNTOUCHED.
2. **A constrained interior triangulator** in `src/native/tessellate/
   uv_triangulate.h` (a new entry point that accepts interior Steiner points and
   preserves the boundary loops), leaving the existing `triangulatePolygon`
   ear-clip entry point byte-identical for the planar callers.
3. **Reader admission of ONE foreign trimmed B-spline face** in
   `src/native/exchange/step_reader.cpp`: a `B_SPLINE_SURFACE_WITH_KNOTS` (incl. the
   rational `weights` already evaluable by `math::nurbsSurface*`) bounded by a real
   `EDGE_LOOP` is built as a native trimmed `Kind::BSpline` face **only when** a
   faithful pcurve can be reconstructed for every boundary edge (endpoint projection
   by the existing `projectBSplineUV` surface inversion for straight-in-`(u,v)`
   edges; a densified-sample fit that VERIFIES `S(pcurve(t)) = C_edge(t)` within a
   scale-relative tolerance for a curved boundary). If ANY edge's pcurve cannot be
   reconstructed faithfully, the face **DECLINES → OCCT** (unchanged precedent).
4. **The honest-out is preserved end-to-end.** The engine's mandatory watertight +
   volume self-verify remains the final arbiter: a native trimmed-freeform solid
   that is not watertight or whose volume/area does not match the OCCT oracle is
   DISCARDED → OCCT. No tolerance is weakened; `src/native/**` stays OCCT-free; the
   `cc_*` ABI is unchanged (this is internal mesher + reader behaviour).

## Capabilities

### Modified Capabilities

- `native-tessellation`: ADDS a watertight mesh path for a **genuinely trimmed
  freeform (`Kind::BSpline` / `Kind::Bezier`) face** — a pcurve-bounded `EDGE_LOOP`
  over a curved patch — via boundary-constrained interior curvature sampling, proven
  **byte-identical** for every existing surface kind and mesh path, with the honest
  decline retained when watertightness cannot be met additively.
- `native-exchange`: ADDS admission of ONE foreign **trimmed** `B_SPLINE_SURFACE_
  WITH_KNOTS` face (rational or not) whose boundary pcurves reconstruct faithfully,
  meshed watertight by the new tessellator path; DECLINES → OCCT otherwise.

## Impact

- `src/native/tessellate/face_mesher.h` — a NEW guarded branch `trimmedFreeformMesh`
  dispatched only for a curved genuinely-trimmed face; existing `structuredGrid`,
  `earClipMesh`, `axisSamples`, anchor snapping untouched. Cognitive complexity kept
  in the systems band (the driver delegates to boundary / interior-sample /
  constrained-triangulate helpers).
- `src/native/tessellate/uv_triangulate.h` — a NEW `triangulateConstrained(pts,
  loops, interior)` entry point; existing `triangulatePolygon` unchanged.
- `src/native/tessellate/trim.h` — reused as-is (`UVRegion::inside`, pcurve
  sampling); no signature change.
- `src/native/exchange/step_reader.cpp` — `advancedFace` / `pcurveFor` gain a
  faithful **B-spline-surface** pcurve arm + reconstruction guard; the non-faithful
  path keeps the existing `decline()`.
- **Zero-regression discipline (mandatory).** Because the new mesh branch is
  reachable ONLY by faces that TODAY decline, no passing mesh can change; this is
  PROVEN, not assumed: every existing surface kind and the FULL
  tessellation-sensitive suite (`run-sim-suite` 221/221, curved-fillet 23/23,
  curved-chamfer 18/18, curved-boolean native-pass=18, wrap-emboss 14/14, loft,
  phase3 70/70, STEP import 77/77) MUST mesh **byte-identically** (same triangle
  counts, watertight status, enclosed volumes). If ANY differs, the mesher change is
  reverted and the foreign patch keeps the OCCT decline.
- **Out of scope (declines, documented not faked):** general curved pcurves that do
  not reconstruct faithfully; self-intersecting trim wires; beyond-tolerance boundary
  gaps (that is M5 healing); multi-patch freeform booleans (M2) and blends (M3),
  which merely CONSUME this mesher. No `cc_*` ABI change; no CyberCad app change; no
  OCCT linked into `src/native/**`.
