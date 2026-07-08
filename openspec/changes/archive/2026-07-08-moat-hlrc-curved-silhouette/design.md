# Design — moat-hlrc-curved-silhouette (quadric silhouette HLR)

Trace the **closed-form silhouette locus** (`n · viewDir = 0`) of a **cylinder**
and a **sphere** face, emit it as world-space polylines, and feed those through
the **existing** `projectOrthographic` occlusion + visibility-split path so the
visible/hidden classification is reused verbatim. Everything else — cone, torus,
partial/trimmed quadric with an out-of-window silhouette, and freeform — keeps
the honest decline. Clean-room math (elementary quadric geometry); OCCT
`HLRBRep_Algo` is the **oracle only**, never linked by `src/native`.

## 0. What the landed HLR already does (the substrate — verified in source)

`orthographic_hlr.h::projectOrthographic(edgeVertices, edges, occ, view, prm)`:

- builds the drawing-plane basis `right = normalize(viewDir × up)`,
  `trueUp = right × viewDir` (`detail::makeBasis`);
- for each world edge, samples at **cell midpoints** (strict interiors), calls
  `detail::occluded(sample, viewDir, occ, prm)` — a Möller–Trumbore ray from the
  sample nudged toward the camera by `surfaceOffset`, HIDDEN iff any occluder
  triangle is crossed at `t > rayEps`;
- **bisects** to `transitionTol` at each visible→hidden transition and emits
  disjoint projected `Segment2D`s whose union covers the whole edge.

`NativeEngine::hlr_project` (`native_engine.cpp` ~L1479) wraps it: validates the
view, **declines any non-planar face** (the blunt guard we replace), meshes the
M0 occluder at the caller's deflection, discretizes + **de-duplicates**
topological edges (endpoint-quantized key — a box's 24 edge nodes collapse to
12), then calls `projectOrthographic`.

The occluder is the **inscribed** faceted mesh: every facet chords *under* the
true curved surface. This is the load-bearing fact for correctness (§3).

## 1. The silhouette math (closed form)

A point on a face is on the silhouette iff the outward surface normal is
perpendicular to the view direction: `n(u,v) · d = 0`, `d = viewDir` (unit).

### 1a. Sphere — `FaceSurface{Kind::Sphere, frame, radius r}`

Centre `C = frame.origin`. The normal at a surface point `P` is radial,
`n = (P − C)/r`, so `n · d = 0` ⇔ `(P − C) ⟂ d`. The silhouette is the **great
circle** of radius `r` in the plane through `C` with normal `d`:

```
P(t) = C + r·(cos t · e1 + sin t · e2),   t ∈ [0, 2π)
```

where `{e1, e2}` is any orthonormal basis of the plane `⟂ d` — take
`e1 = right`, `e2 = trueUp` (the drawing-plane basis). Because that plane is
**perpendicular to `d`**, the orthographic projection of `P(t)` is
`project(C) + r·(cos t, sin t)` — an exact **circle of radius `r`** in drawing
coordinates. Closed form for GATE (a): projected outline is a circle radius `r`,
length `2πr`, fully visible.

Emit: the great circle discretized to `nSeg` chords sized by a chord-height
bound `δ` (`nSeg ≥ π/√(2δ/r)` for sagitta ≤ δ), as a closed world polyline. A
full untrimmed sphere has **no topological edge**, so this is its only outline.

### 1b. Cylinder — `FaceSurface{Kind::Cylinder, frame, radius r}`, height h

Axis `A = frame.z`, radial basis `X = frame.x`, `Y = frame.y`. A lateral point
at angle `θ`, height `s∈[0,h]`: normal `n(θ) = cos θ·X + sin θ·Y` (independent of
`s`). `n · d = 0` ⇒

```
cos θ (X·d) + sin θ (Y·d) = 0   ⇒   θ* = atan2( −(X·d), (Y·d) )  and θ*+π
```

giving the **two diametrically opposite generators**. Each generator is the
straight segment `P(s) = C + r(cos θ*·X + sin θ*·Y) + s·A`, `s∈[0,h]`, where the
axial extent `[0,h]` is the face's parametric `v`-range (cap-to-cap). Two
straight world segments — fed as ordinary edges.

**Degenerate view (axis-parallel).** When `|A · d| → 1` the view looks down the
axis: `X·d` and `Y·d` both → 0, `θ*` is ill-conditioned and the lateral
silhouette **collapses onto the cap circle**, which is already a topological
edge. Guard: if `‖d − (d·A)A‖ < kSil` (the view direction has no component in
the cylinder's cross-section plane), emit **no generators** — the cap edge
carries the outline. This is a correct special case, not a guess. Right at the
threshold band the helper returns "not traceable" and the request declines
rather than emit a near-degenerate generator.

**Trim-window guard (partial quadric).** `θ*` and `θ*+π` must lie inside the
face's actual angular trim window and the axial segment inside its `v`-range.
For a full cylinder (seam-closed, capped) both generators are in-window → traced.
For a **partial** cylinder (a fillet flank, a trimmed sector) a generator that
falls outside the `[uMin,uMax]×[vMin,vMax]` window has an **ambiguous** outline
(the silhouette may run off the face onto an adjacent face) → the helper returns
"not traceable" and the whole request declines. No clipped-generator guess.

### 1c. Out of scope this wave (return "not traceable" → decline)

- **Cone**: two flank generators `θ*` exist, but the **apex** is a normal
  singularity and the outline mixes the flank generators with the base ellipse
  in a way that needs apex-visibility handling — deferred.
- **Torus**: `n · d = 0` is a **quartic** in the tube angle with up to four
  real branches; the silhouette self-occludes (inner vs outer) — deferred.
- **Freeform** (`BSpline`/`Bezier`): no closed form; a marching/tracing solver is
  a separate capability — deferred.

## 2. The silhouette helper API (`src/native/drafting/silhouette.h`)

```cpp
namespace cybercad::native::drafting {

struct SilhouetteParams { double chordDeflection = 0.05; double axisParallelTol = 1e-6; };

struct SilhouetteResult {
  bool traced = false;                                   // false ⇒ DECLINE this face
  std::vector<std::vector<math::Point3>> polylines;      // world-space outline curve(s)
};

// Trace the analytic silhouette of ONE face for `view`. Returns traced=false
// (never a guessed polyline) for cone/torus/freeform, an axis-parallel-degenerate
// cylinder, or a partial quadric whose generator leaves the trim window.
SilhouetteResult traceSilhouette(const FaceSurfaceView& face, const OrthographicView& view,
                                 const SilhouetteParams& prm = {});
}
```

`FaceSurfaceView` is a tiny OCCT-free adapter carrying the surface kind, frame,
radius, and the face's `(u,v)` trim window + orientation (built by the engine
from `ntopo::FaceSurface`; the helper never sees a topology type). The helper is
pure math over `native/math` — no occluder, no projection state.

`orthographic_hlr.h` is **untouched**; the silhouette polylines are appended to
`edgeVertices`/`edges` before the single existing `projectOrthographic` call.

## 3. Why the classification "just works" (correctness)

For a **single convex** quadric the silhouette rim is the outermost boundary in
its view direction, so every silhouette sample is the nearest surface along its
line of sight → VISIBLE. Robustness against the **inscribed** occluder: a true
silhouette point projects to the extreme boundary, while the faceted occluder is
inscribed *inside* the true outline, so the silhouette sample's ray toward the
camera passes **outside** the occluder's projected shadow → not spuriously
occluded → correctly visible. The `surfaceOffset` nudge (reused unchanged)
discounts self-grazing.

The **far cap arc** (a topological circular edge) lies on the true surface but
projects *inside* the outline; the near lateral facets are nearer the camera
there → the existing ray-cast marks it HIDDEN, and bisection splits the cap
circle into its visible front arc + hidden back arc. This reproduces the
closed-form "one visible + one hidden cap ellipse arc" **with no new occlusion
code** — the only additions are the generator/great-circle polylines.

**Tolerance honesty.** The inscribed-facet chord error near the silhouette must
not misclassify cap-arc samples. The occluder deflection is **caller-chosen**
(`opts.deflection`, never silently defaulted) and the sim gate uses a tight
deflection; if a body cannot be classified within tolerance at the chosen
deflection the request declines rather than emit a doubtful split. No tolerance
is weakened anywhere.

## 4. Engine router (`NativeEngine::hlr_project`)

Replace the blanket non-planar decline with a per-face pass:

```
for each face F:
  if F.surface.kind == Plane:                       # unchanged contribution
      (its topological edges already flow through the existing edge loop)
  elif F.surface.kind == Cylinder or Sphere:
      r = traceSilhouette(view(F), view, prm)
      if not r.traced: return make_error("hlr_project: quadric silhouette not traceable (declined)")
      append r.polylines to the edge set
  else:                                              # Cone/Torus/BSpline/Bezier
      return make_error("hlr_project: curved/freeform silhouette declined (cone/torus/freeform)")
```

Topological curved edges (the cap circles) already flow through the existing
`cache.discretize` + de-dup loop; they were unreachable only because the old
guard rejected the body up front. Everything after (occluder build, dedup,
`projectOrthographic`, `DrawingData` assembly) is unchanged. `cc_hlr_project`
and its PODs are untouched — additive COVERAGE only.

## 5. Verification

- **GATE (a) HOST ANALYTIC** (`tests/native/test_native_drafting.cpp`, no OCCT):
  - Sphere r at C, arbitrary view → outline is a closed circle radius `r`
    centred at `project(C)` within `1e-9`; fully visible; total length `2πr`;
    0 hidden.
  - Cylinder r,h, axis ⟂ view (side-on) → exactly two visible generator
    segments of length `h` separated by `2r`; cap arcs split into a visible +
    a hidden arc; total projected length conserved.
  - Axis-parallel cylinder view → no generators emitted, cap edge is the
    outline (not a decline); the near-threshold band declines.
  - Cone / torus / freeform / partial-quadric-out-of-window → `traced=false`,
    request declines with the documented error.
- **GATE (b) SIM native-vs-OCCT** (`tests/sim/native_hlr_parity.mm`): cylinder +
  sphere under `cc_set_engine(1)` match `HLRBRep_Algo` (`cc_set_engine(0)`) on
  visible COUNT, hidden COUNT, total projected LENGTH (rel ≤ `1e-4`), and
  endpoint PARTITION (each native segment covered by an OCCT segment of the same
  class within `1e-5`, and vice versa). A cone/torus body is asserted DECLINED,
  not compared. No tolerance widening.
- **OCCT-free proof**: `src/native/drafting/**` (incl. new `silhouette.h`)
  scanned for OCCT includes → zero; compiles host + arm64 iOS simulator.

## 6. Cognitive complexity

`traceSilhouette` dispatches by surface kind to two small closed-form emitters
(`sphereGreatCircle`, `cylinderGenerators`) plus the degenerate/trim guards;
each helper targets the geometry band (≤ 25) with the quadric math flagged. The
engine router adds one `switch` over face kind (≤ 15). `orthographic_hlr.h` is
unchanged.

## 7. Alternatives considered

- **Tessellate the curved face densely and draw mesh silhouette edges** —
  rejected: it emits a fabricated polyline approximating the outline (exactly
  the "mesh-edge segments approximating the true curved outline" the landed spec
  forbids) and its classification is deflection-dependent, not closed-form.
- **A general marching silhouette tracer for all surfaces** — rejected for this
  wave: cone/torus/freeform need a robust `n·d=0` continuation solver with
  branch/self-occlusion handling; landing it now would risk a wrong outline. The
  honest decline keeps those on OCCT until a dedicated slice.
