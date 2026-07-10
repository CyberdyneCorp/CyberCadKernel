# Proposal — moat-nfill-surface-patch (MOAT surface — bounded N-sided fill / surface patch)

## Why

FILLING an N-sided hole — a boundary loop of edges — with a smooth surface patch is the
surfacing operation every professional CAD (BRepFill_Filling in OCCT, "Fill" / "Boundary
surface" / "N-sided patch" in SolidWorks / Fusion / CATIA) ships and the CyberCad app
needs to complete an open boundary into a watertight body. No `cc_*` entry exists yet, so
the app's fill path can only reach OCCT `BRepOffsetAPI_MakeFilling` / `BRepFill_Filling`.

The landed heal `capPlanarHole` (`src/native/heal/cap_hole.h`) already closes ONE simple
PLANAR hole exactly (a planar cap face on the hole's shared nodes). It DECLINES a hole
whose boundary is NOT coplanar. The next bounded surfacing slice is to fill a **non-planar
3-6-sided analytic/polyline boundary loop** with a smooth interpolating patch.

## THE SCOPE BOUND (non-negotiable — this is NOT a general NURBS kernel)

The boundary loop is composed of ANALYTIC / POLYLINE edges (straight segments + circular
arcs). The fill patch is a **Coons / Gregory-style transfinite interpolant that is
EVALUATED to a TESSELLATED triangle-grid mesh** interpolating the boundary — NOT stored as
a general trimmed-NURBS surface. The output is a mesh/solid-completing patch, NOT a
NURBS B-rep face. No general NURBS surface representation is added; no NURBS evaluator is
built beyond what a Coons/Gregory patch over the given boundary curves needs. A case that
needs true NURBS surfacing (non-analytic boundary, G2 continuity, >6-sided degenerate,
self-intersecting patch, highly non-planar boundary that will not weld) is HONEST-DECLINED
→ OCCT `BRepFill_Filling`. This bound is what keeps the whole campaign from becoming a
general NURBS kernel.

## What changes

- **New OCCT-free module `src/native/surface/`** —
  - `ngon_fill.h` — `surface::fillNGon(boundary, opts, &decline)` evaluates a bilinearly-
    blended **Coons patch** (N=3-4 via a discrete Coons interpolant; N=5-6 via a Gregory-
    style convex-combination of side Coons patches over a mean-value parameterization) to a
    `tess::Mesh` triangle grid of density `gridN`. The four/N boundary curves are the
    input analytic edges (straight segments + circular arcs) sampled EXACTLY. Boundary rows
    of the grid reuse the boundary samples BIT-EXACTLY, so the patch shares the boundary
    points. `NGonDecline` (NonAnalyticBoundary / TooManySides / DegenerateBoundary /
    SelfIntersecting / NotConverged).
  - `fill_solid.h` — `surface::fillHoleSolid(openShell, opts, &decline)` finds the single
    free-boundary loop of an OPEN shell (the same used-once-edge boundary graph as
    `cap_hole.h`), builds the fill patch, welds it to the shell's existing faces on shared
    boundary points via the boolean `assembleSolid(std::vector<Polygon>)` substrate, and
    SELF-VERIFIES (watertight closed 2-manifold, positive enclosed volume). Planar boundary
    ⇒ the patch is the planar face (fan) restoring the hole exactly; non-planar ⇒ the
    tessellated Coons patch. Reuses `heal::self_verify` semantics.
- **Additive `cc_fill_ngon` in the C ABI** — signature-styled like the landed
  `cc_variable_sweep` / `cc_sheet_base_flange` point-array ops. Takes an explicit closed
  boundary (corner XYZ + per-corner edge kind: straight or arc-through-a-mid-point) and a
  grid density; returns a **mesh-backed patch body** (the tessellated surface), served
  through `cc_mass_properties` (area) / `cc_bounding_box` / `cc_tessellate` /
  `cc_face_meshes` like an imported mesh body. `NativeEngine::fill_ngon` builds the patch
  natively (self-verified boundary-coincidence, else honest decline);
  `OcctEngine::fill_ngon` is the `BRepFill_Filling` oracle. No existing `cc_*` signature
  changes.

## Impact

- Heal `cap_hole.h` and M0-M7 remain **byte-frozen**; the new module is an additive
  `src/native/surface/` sibling that REUSES `heal::self_verify`, `boolean::assembleSolid`,
  `boolean::Polygon`, and the boundary-graph idea from `cap_hole.h` (re-implemented locally
  so heal stays untouched).
- `src/native/**` stays OCCT-FREE (descriptive comments only).
- Tessellator, boolean, construct, blend, analysis, exchange, sheetmetal are UNTOUCHED.
- Non-analytic boundary / >6 sides / self-intersecting patch / highly non-planar boundary
  that will not weld watertight / G2 request are honest-declined (measured reason) → OCCT;
  never a wrong / leaky / self-intersecting patch, never a widened tolerance.
