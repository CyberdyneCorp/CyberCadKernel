# Proposal — moat-hlrc-curved-silhouette (quadric-face silhouette HLR: cylinder + sphere)

## Why

The landed orthographic HLR core (`src/native/drafting/orthographic_hlr.h` +
`cc_hlr_project`) is two-gate complete for the **POLYHEDRAL** case: it projects
straight topological edges, ray-casts each sample against the M0 triangle
occluder, splits at visibility transitions, and matches OCCT `HLRBRep_Algo` on
box / prism / L-prism (13/13 sim parity). It **DECLINES every curved-faced
body** with a single blanket guard in `NativeEngine::hlr_project`
(`native_engine.cpp` ~L1498): any face whose surface `kind != Kind::Plane`
returns `"curved/freeform silhouette declined (polyhedral core only in this
slice)"`.

That guard is correct but blunt. A cylinder/cone/sphere solid draws with an
**outline that is not a topological edge at all** — it is a true **silhouette
curve**, the locus on the curved face where the surface normal is perpendicular
to the view direction (`n · viewDir = 0`). No projected edge represents it, so
the polyhedral path has nothing to draw there and (correctly) refuses to guess.
The consequence: a cylinder, a cone and a sphere — the three most common
mechanical primitives — produce **no native drawing at all**, forcing every
curved body back to OCCT and blocking the drop-OCCT trajectory for drafting.

The silhouette of a **quadric** face, however, is **closed-form**:

- **Sphere** (centre `C`, radius `r`): the silhouette is the great circle in the
  plane through `C` perpendicular to `viewDir`. Under orthographic projection it
  is a circle of radius `r` centred at the projection of `C` — the entire
  outline of a sphere, and its only outline (a full sphere carries no
  topological edge).
- **Cylinder** (axis `A`, radius `r`, finite height `h`): the silhouette is the
  **two generator lines** parallel to `A`, tangent to the view — the two points
  on the rim where the radial normal is perpendicular to `viewDir`, swept along
  the axis over the cylinder's height. Together with the two circular cap edges
  (already topological), this is the familiar side-on outline: two straight
  outline lines + a visible and a hidden cap ellipse arc.

These loci can be computed exactly and emitted as first-class projected
polylines that feed the **existing** occlusion + visibility-split path
unchanged. The visible/hidden classification then "just works": for a single
convex quadric the silhouette is the outermost rim and is fully visible, the far
cap arc is occluded by the near lateral surface and comes out hidden — exactly
the closed-form answer and exactly what OCCT `HLRBRep_Algo` produces.

This is the first bounded slice of curved HLR. It lands the **two cleanest
quadrics — cylinder and sphere**. Cone (apex singularity + flank generators),
torus (a quartic, self-occluding silhouette), a **partial/trimmed quadric whose
generator leaves the face's trim window**, and freeform (B-spline/Bézier) faces
**stay honestly DECLINED** this wave. A correct decline is a first-class,
expected outcome; a fabricated outline or a wrong visible/hidden classification
is never emitted.

## What Changes

1. **A new OCCT-free silhouette helper** `src/native/drafting/silhouette.h`
   (namespace `cybercad::native::drafting`, header-only, includes only
   `native/math`). Given an analytic `FaceSurface` (`Cylinder` or `Sphere`), a
   finite parametric/axial extent, and an `OrthographicView`, it returns the
   silhouette as **world-space polylines** (a cylinder → two straight generator
   segments clipped to the height; a sphere → the great-circle polyline
   discretized to a caller-chosen chord bound). It returns an **honest
   "not traceable" status** — never a polyline — for a cone, a torus, a
   freeform surface, an axis-parallel-degenerate cylinder view, or a partial
   quadric whose silhouette generator falls outside the face's trim window.

2. **`orthographic_hlr.h` stays BYTE-IDENTICAL** for polyhedral inputs. The
   silhouette polylines are fed to the **existing** `projectOrthographic` as
   ordinary world edges, so occlusion ray-casting, bisection splitting and the
   visible/hidden partition are reused verbatim. No change to the polyhedral
   code path, its params, or its 13/13 parity.

3. **`NativeEngine::hlr_project` admits cylinder + sphere.** The blanket
   non-planar decline is replaced by a per-face router: a `Plane` face
   contributes its topological edges (unchanged); a `Cylinder`/`Sphere` face
   additionally contributes its traced silhouette polylines; **any other curved
   kind (Cone, Torus, BSpline, Bezier), or a quadric whose silhouette the helper
   cannot trace, still DECLINES the whole request** with an error. The occluder
   mesh, edge de-duplication and the `projectOrthographic` call are otherwise
   unchanged.

4. **`cc_hlr_project` signature and PODs are UNCHANGED** — this is purely a
   COVERAGE extension of the existing accessor. A cylinder/sphere body now
   returns a populated `CCDrawing`; a cone/torus/freeform body still returns an
   empty `CCDrawing` with `cc_last_error` set. `src/native/**` remains OCCT-free
   (0 OCCT includes); OCCT appears only in the sim parity oracle.

5. **Two new gates.** GATE (a) HOST ANALYTIC (`clang++ -std=c++20`, no OCCT):
   a sphere projects to a circle of radius `r` (fully visible, total length
   `2πr`); a cylinder side-on yields two visible generator segments of length
   `h` separated by `2r` plus the cap-arc visible/hidden split. GATE (b) SIM:
   `cc_hlr_project` under the native engine matches OCCT `HLRBRep_Algo` on
   visible/hidden **count**, **total projected length**, and **endpoint
   partition** within tolerance for cylinder and sphere; a cone/torus body is
   asserted DECLINED, not compared — with no tolerance widening.

## Impact

- **Specs:** `native-drafting` — ADD a quadric-silhouette requirement; MODIFY
  the honest-decline, two-gate-verification, and additive-accessor requirements
  to move cylinder/sphere from DECLINED to SUPPORTED while keeping
  cone/torus/partial-quadric/freeform declined.
- **Code:** new `src/native/drafting/silhouette.h`; a per-face router edit in
  `src/engine/native/native_engine.cpp::hlr_project`. No `cc_*` signature
  change; `orthographic_hlr.h` byte-identical for polyhedral inputs.
- **Tests:** `tests/native/test_native_drafting.cpp` gains the sphere-circle and
  cylinder-side-on host cases; the sim parity harness gains cylinder + sphere vs
  `HLRBRep_Algo` and a cone/torus decline assertion.
- **Risk / non-goals:** cone, torus, partial-quadric-with-ambiguous-silhouette
  and freeform silhouettes are explicitly out of scope and remain declined; no
  section module is touched (sibling GS2 workflow owns `src/native/section`).
