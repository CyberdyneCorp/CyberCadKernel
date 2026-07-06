# Design — add-native-curved-chamfer (#6 curved blends, circular-rim cone-frustum chamfer)

## Context

The native CURVED FILLET (`src/native/blend/curved_fillet.h`, `curved_fillet_edge` →
`buildFilletedCylinder`) rounds the rim where a cylinder LATERAL face meets a COAXIAL
PLANAR CAP in a CONVEX dihedral: a ball of radius `r` rolled on the outside of the crease
inserts a G1-TANGENT quarter-TORUS canal (major `Rc − r`, minor `r`), trimmed to the two
TANGENT circles torus∩cylinder (`Rc`, at `H − s·r`) and torus∩plane (`Rc − r`, at `H`),
and REMOVES material.

The native CHAMFER (`src/native/blend/chamfer_edges.h`, `chamfer_edges`) is native only
for a PLANAR-dihedral edge: it slices the convex corner off with the single PLANE through
the two setback lines and re-welds. A CURVED (circular) edge falls through to OCCT.

This design lands the first CURVED chamfer: the SAME convex cylinder↔cap rim the curved
fillet handles, but cutting a FLAT BEVEL instead of a rounded arc. A chamfer with a
SYMMETRIC distance `d` sets each face back by `d` and bridges the two setback lines with a
straight bevel. For a CIRCULAR rim the two setback lines become two setback CIRCLES and
the straight bevel becomes a **CONE FRUSTUM** (a ruled truncated cone). Unlike the fillet
(a curved torus arc, G1-tangent), the chamfer is a straight bevel that meets each face at
the chamfer ANGLE — **C0, NOT G1**. Asserting tangency here would be geometrically wrong.

The construction is the curved-fillet builder with the torus quarter-tube replaced by a
single straight frustum band and the two TANGENT circles replaced by the two SETBACK
circles. Everything else — the `detail::facesOnRim` + `detail::rimGeom` classifier, the
`math::Ax3` cylinder frame, the `sagittaSteps` angular budget, the `ringPoint` sampler,
the planar-triangle weld through the boolean `assembleSolid` — is reused.

The method is **clean-room**: derive the cone frustum + its two setback circles in closed
form; OCCT `BRepFilletAPI_MakeChamfer` is the verification ORACLE only, never copied.

## Goals / Non-Goals

**Goals**
- A native, OCCT-free SYMMETRIC-distance chamfer on a CONVEX circular crease (a cylinder
  lateral face meeting a coaxial PLANAR cap in a CONVEX dihedral), both setback circles
  staying inside their faces.
- Chamfer surface = a CONE FRUSTUM (straight bevel band) between the two SETBACK circles
  (cylinder seam at `Rc`, axial `H − s·d`; cap seam at `Rc − d`, `z = H`), meeting each
  face at the chamfer angle **C0 (NOT G1)**, welded watertight through `assembleSolid`.
- The chamfer REMOVES material: the assembled solid's volume is strictly LESS than the
  body's, exactly `|body| − π·d²·(Rc − d/3)`, gated by the engine's
  `blendResultVerified(..., wantGrow=false)` (SHRINK) branch.
- Behind the unchanged `cc_chamfer_edges` ABI; OCCT fallback for the rest.

**Non-Goals (return NULL → OCCT, never faked)**
- ASYMMETRIC two-distance chamfer (`d1` on the wall, `d2` on the cap, or a distance +
  angle) — the frustum would be an oblique cone; this slice is the SYMMETRIC distance
  only (equal setback on both faces, 45° bevel).
- NON-circular curved creases (cone↔plane rim, sphere rims, ellipse/spline creases, a
  non-coaxial or tilted plane) — deferred. (A PLANAR-dihedral edge is already handled by
  the planar `chamfer_edges`; this slice adds only the circular rim.)
- CONCAVE circular rim (cylinder↔larger coaxial plane — a stepped-shaft base rim) —
  deferred (the frustum would ADD material; not in this slice).
- **cylinder↔cylinder (curved↔curved) chamfer** (two curved faces) — deferred.
- Any freeform (NURBS/Bézier/B-spline) adjacent face — deferred.
- Near-degenerate distance: `Rc ≤ d` (the cap circle `Rc − d ≤ 0` collapses to a point /
  crosses the axis) or the wall shorter than `d` (the cylinder seam leaves the wall) —
  NULL → OCCT.
- Multi-edge selection (`edgeCount ≠ 1`) — this slice handles a single circular rim.

## The circular-rim cone-frustum chamfer geometry (clean-room)

Take a cylinder of radius `Rc`, axis `A` (frame Z), capped by a coaxial planar CAP at
axial height `H` (cap normal ∥ `A`), the rim the CIRCLE of radius `Rc` at `H`, the
dihedral CONVEX (air fills the corner). Let `s = ±1` be the axial sign from the far end
toward the cap (read from the geometry exactly as `buildFilletedCylinder` reads `capH` vs
`farH`). A SYMMETRIC chamfer of distance `d` sets BOTH faces back by `d`:

- the cylinder wall set back AXIALLY by `d` → the **cylinder seam** circle: radius `Rc`,
  axial `H − s·d` (a circle ON the cylinder wall by construction, `radius = Rc` exact);
- the cap set back RADIALLY by `d` → the **cap seam** circle: radius `Rc − d`, axial `H`
  (a circle ON the cap plane by construction, `axial = H` exact).

The bevel bridges the two setback circles with a STRAIGHT line in every meridian
half-plane — from `(Rc, H − s·d)` to `(Rc − d, H)` in `(radius, axial)` coordinates.
Revolving that straight line about `A` sweeps a **CONE FRUSTUM** (a ruled truncated cone),
the larger circle (radius `Rc`, at `H − s·d`) on the wall side and the smaller circle
(radius `Rc − d`, at `H`) on the cap side:

```
radius(u, τ) = Rc − d·τ ,          axial(u, τ) = (H − s·d) + s·d·τ ,   τ ∈ [0, 1]
point(u, τ)  = ringPoint(A, radius(u, τ), u, axial(u, τ)) ,            u ∈ [0, 2π)
```

- `τ = 0` → radius `Rc`, axial `H − s·d`  → the CYLINDER seam circle.
- `τ = 1` → radius `Rc − d`, axial `H`     → the CAP seam circle.

The meridian slant direction is `(Δradius, Δaxial) = (−d, s·d)` — EQUAL magnitude, so for
the symmetric distance the slant makes **45°** with the axial (cylinder-wall) direction
and **45°** with the radial (cap) direction. The frustum is a single straight step in
`τ`: no meridian subdivision is needed (only the angular `u` is faceted).

### The removed volume (exact, closed form — Pappus)

The chamfer removes the sharp-corner ring ABOVE the bevel. In the meridian `(radius,
axial)` plane that region is the RIGHT TRIANGLE with vertices

```
P0 = (Rc,     H)        the sharp corner (right angle here)
P1 = (Rc,     H − s·d)  the cylinder setback point
P2 = (Rc − d, H)        the cap setback point
```

legs `d × d`, area `A_tri = d²/2`, centroid radial `R̄ = (Rc + Rc + (Rc − d))/3 = Rc −
d/3`. By Pappus the revolved removed volume is EXACT:

```
V_removed = 2π · R̄ · A_tri = 2π · (Rc − d/3) · (d²/2) = π · d² · (Rc − d/3)
```

Cross-check (corner ring minus frustum): the full `d × d` corner square revolved is
`V_sq = 2π(Rc − d/2)d² = π d²(2Rc − d)`; the KEPT frustum wedge (the other triangle, below
the bevel, centroid radial `Rc − 2d/3`) is `V_keep = π d²(Rc − 2d/3)`; `V_sq − V_keep =
π d²(Rc − d/3)` — the same closed form. The chamfered solid's volume is `|cyl| −
π d²(Rc − d/3)`. (For reference the FILLET removes `π·(Rc − d + 2d/3·…)` less — a chamfer
of the same setback removes MORE than a fillet, since the flat bevel cuts inside the
rounded arc: triangle area `d²/2` vs the fillet's `d²(1 − π/4) ≈ 0.215 d²`.)

### The bevel is C0, NOT G1 (the load-bearing correctness statement)

The frustum outward normal in the meridian plane is PERPENDICULAR to the slant `(−d,
s·d)`, oriented away from the axis and toward the removed corner: `n_m ∝ (s, 1)` in
`(radial, axial)` — i.e. equal radial and axial components, `n_m = (radial + s·axial)/√2`
in 3D at azimuth `u`:

```
n_frustum(u) = ( x̂·cos u + ŷ·sin u )·(1/√2)  +  ẑ·( s/√2 )
```

- At the **cylinder seam** (`τ = 0`): the cylinder normal is RADIAL,
  `n_cyl = x̂·cos u + ŷ·sin u`. Then `cos∠ = n_frustum · n_cyl = 1/√2 ≈ 0.70710678` — NOT
  `1`. The frustum is NOT tangent to the wall; it meets it at a 45° bevel (a C0 crease).
- At the **cap seam** (`τ = 1`): the cap normal is AXIAL, `n_cap = s·ẑ`. Then
  `cos∠ = n_frustum · n_cap = 1/√2` — again NOT `1`. C0, 45° bevel.

So the builder SELF-VERIFIES the CORRECT bevel geometry — each seam ON its neighbour
surface (`radius = Rc` at the wall seam; `axial = H` at the cap seam) AND the frustum
normal at the CHAMFER ANGLE to each face (`cos = 1/√2` for the symmetric distance) — and
explicitly asserts it is NOT tangent (`cos ≠ 1`). This is the deliberate INVERSE of the
fillet's G1 assertion: a chamfer is a straight bevel, so tangency would be WRONG.

(General note for the honest record: for a hypothetical asymmetric chamfer the bevel angle
would be `atan2(d_cap, d_wall)` and the two `cos` values would differ; this slice ships
ONLY the symmetric `d`, so both are `1/√2`. Asymmetric → OCCT.)

## Convex / symmetric-distance classification (how the builder knows)

The pick is a CIRCLE shared by ONE `Cylinder` lateral face and ONE coaxial `Plane` CAP
whose normal ∥ axis, meeting CONVEX — the EXACT config the curved fillet classifies
(`detail::facesOnRim` + `detail::rimGeom`). The chamfer builder reuses those verbatim: if
the edge is not that convex cylinder↔cap rim it returns NULL (the planar chamfer, or OCCT,
owns the rest). The ONLY numeric input is the single chamfer distance `d` (symmetric);
there is no separate asymmetric / concave / cyl-cyl / non-circular path in this slice.

## Module shape

```
src/native/blend/
  curved_chamfer.h         // NEW. curved_chamfer_edge(...) mirrors
                           //   curved_fillet_edge(...); reuses detail::facesOnRim,
                           //   cylinderInfo, rimGeom, sagittaSteps, ringPoint via
                           //   #include "native/blend/curved_fillet.h". The only new
                           //   code is buildChamferedCylinder(const RimGeom&, double d,
                           //   double defl) — the fillet's buildFilletedCylinder with
                           //   the torus quarter-tube replaced by one straight frustum
                           //   band and the tangent circles by the setback circles.
  curved_fillet.h          // REUSED (classifier + helpers); no change.
  chamfer_edges.h          // REUSED (planar chamfer stays the first native attempt).
  blend_geom.h             // REUSED — facePlane, signedDist.
  native_blend.h           // add #include "native/blend/curved_chamfer.h".
```

`curved_chamfer_edge(const topo::Shape&, const int* edgeIds, int edgeCount, double
distance, double deflection)` returns the chamfered solid or a NULL Shape. It reuses the
curved fillet classifier (`facesOnRim`, `cylinderInfo`, `facePlane`, `rimGeom`) and the
`sagittaSteps` / `ringPoint` / `emit*` facet helpers; the only new geometry is the
frustum band + the setback circles.

## Build pipeline (concrete)

1. **Classify the picked edge.** Reuse `detail::facesOnRim` + `detail::rimGeom`: resolve
   the 1-based edge id to a `Circle` `EdgeCurve` of radius `Rc` coaxial with a `Cylinder`
   face (radius `Rc`, axis `A`) and a `Plane` CAP whose normal ∥ `A`, meeting CONVEX.
   Require exactly one of each at the rim. Else return NULL.
2. **Precondition guard.** Require `edgeCount == 1`, `distance = d > kBlendEps`, the cap
   circle real (`Rc − d > eps`, so the frustum does not collapse to a point / cross the
   axis), and the far end beyond the cylinder seam `s·((H − s·d) − hFar) > 0` (the wall
   covers the axial setback). Any failure ⇒ NULL → OCCT. (NO ring-torus guard `Rc ≥ 2d`
   is needed — the frustum only requires `Rc − d > 0`.)
3. **Facet budget.** `N` angular quads from `sagittaSteps(Rc, 2π, defl)` (the frustum
   band, the wall, and both caps share the SAME `N` samples ⇒ coincident seam vertices).
   The frustum is a SINGLE meridian step (`M = 1`, straight bevel) — no minor
   subdivision.
4. **Tile the cone-frustum band.** For each of `N` angular sectors emit ONE quad
   `(Rc, u0, H−s·d) → (Rc, u1, H−s·d) → (Rc−d, u1, H) → (Rc−d, u0, H)`, each split into
   two exactly-planar triangles carrying their own geometric normal (a frustum quad is
   not coplanar — exactly the split the fillet builder uses for the torus quads). The
   `τ = 0` edge is the cylinder seam (radius `Rc`); the `τ = 1` edge is the cap seam
   (radius `Rc − d`).
5. **Rebuild the neighbours to the setback circles.** The cylinder wall from the far end
   up to the cylinder seam (`H − s·d`) as `N` quads; the far cap as a full `N`-gon disk
   (radius `Rc`, outward `−capNormal`); the TRIMMED cap as an `N`-gon disk (radius
   `Rc − d` at `H`, outward `capNormal`). All share the `N` samples.
6. **Verify the bevel (on-surface + chamfer angle, C0).** Assert every cylinder-seam
   station has `radius = Rc` (on the wall) and every cap-seam station has `axial = H` (on
   the cap) — exact by construction — AND the frustum normal makes the chamfer angle with
   each face normal (`cos = 1/√2` for the symmetric `d`), explicitly NOT tangent
   (`cos ≠ 1`). On any failure return NULL.
7. **Weld watertight.** Feed the facets to the boolean `assembleSolid` (weld +
   triangulate) → a native `Solid`.
8. **Return** the solid to `NativeEngine::chamfer_edges`, which runs
   `blendResultVerified(result, body, wantGrow=false)` — the SHRINK branch.

Cognitive-complexity: the classifier + guard is flat guard-clauses (backend band ≤15);
the frustum band is a single isolated loop with ONE sagitta bound (simpler than the
fillet's two — no minor loop), sharing the fillet builder's helpers; the seam derivations
are closed-form circles (systems band).

## Engine wiring (the load-bearing change)

`NativeEngine::chamfer_edges` changes from a single native (planar) attempt to a
native-planar → native-curved → error dispatch mirroring how `fillet_edges` dispatches
its planar → curved → concave candidates:

```cpp
ShapeResult NativeEngine::chamfer_edges(EngineShape body, const int* e, int ec, double d) {
  if (!isNative(body)) return fallback().chamfer_edges(body, e, ec, d);
  const auto* h = static_cast<const NativeShape*>(body.get());

  // 1) PLANAR chamfer (convex planar-dihedral edge) — slices the corner off; SHRINK.
  ntopo::Shape result = nblend::chamfer_edges(h->shape, e, ec, d);
  if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
    return track(wrapNative(std::move(result)));

  // 2) CURVED circular chamfer (convex cylinder ↔ coaxial cap rim, cone-frustum bevel) —
  //    C0 flat bevel, REMOVES material, verified SHRINK.
  result = nblend::curved_chamfer_edge(h->shape, e, ec, d);
  if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
    return track(wrapNative(std::move(result)));

  // 3) Neither native slice built a verified watertight result → honest error (the OCCT
  //    engine serves the call — never forward a native void to OCCT).
  return make_error(
    "native chamfer_edges: no verified watertight result for this native body "
    "(asymmetric / non-circular / concave / curved-curved / tilted / freeform / "
    "Rc<=d → OCCT-only)");
}
```

A chamfer REMOVES material, so BOTH candidates use the SAME `wantGrow=false` SHRINK guard
(`0 < Vr < Vo`) — identical to today's planar-chamfer guard. The curved attempt is TRIED
ONLY AFTER the planar chamfer returns NULL (a circular rim is not a planar-dihedral edge,
so the planar builder declines it first), so the planar chamfer path is byte-identical to
today. A NULL result or a failed self-verify DISCARDS the candidate; because a native body
cannot be forwarded to OCCT (OCCT would misread the native void), the honest outcome is an
error, and the shipping parity path serves the call from the OCCT engine
(`cc_set_engine(0)`). No new guard type, no weakened tolerance.

## Verification model (two gates)

- **Host (no OCCT):** build a native capped cylinder on the host; chamfer its top rim
  natively with `curved_chamfer_edge(d)`; assert (a) every cylinder-seam station lies on
  the cylinder (`radius = Rc`) and every cap-seam station lies on the cap (`axial = H`),
  within tol; (b) the mesh is watertight (`boundaryEdgeCount == 0`) across the deflection
  ladder; (c) the enclosed volume equals `|body| − π·d²·(Rc − d/3)` within the
  tessellation deflection bound (the EXACT Pappus removed volume); (d) the bevel is C0 at
  the chamfer angle — the frustum normal makes `cos = 1/√2` with the cylinder normal at
  the wall seam and `1/√2` with the cap normal at the cap seam, and is explicitly NOT
  tangent (`cos ≠ 1` — a chamfer, not a fillet); and (e) a chamfer of setback `d` removes
  MORE than the constant fillet of radius `d` on the same rim (`V_removed^chamfer >
  V_removed^fillet`, the flat bevel cuts inside the rounded arc — no sign confusion).
  Built default + NUMSCI-ON.
- **Sim native-vs-OCCT parity** (`scripts/run-sim-native-curved-chamfer.sh` + its `.mm`,
  modelled on `run-sim-native-curved-fillet.sh`): on a booted simulator, chamfer the SAME
  cylinder top rim with the native engine (`cc_set_engine(1)`) and with OCCT
  (`cc_set_engine(0)`); OCCT builds the chamfer with `BRepFilletAPI_MakeChamfer` +
  `Add(distance, edge)` (symmetric); the harness compares volume / surface area /
  watertightness / presence of the cone-frustum bevel face and asserts the bevel is a
  FLAT frustum at the chamfer angle (C0, NOT a G1 torus arc), for (A) `d = 1.0` and
  (B) a second distance. Because the symmetric chamfer IS EXACTLY a cone frustum, the
  native↔OCCT volume/area gap is bounded ONLY by the angular tessellation deflection
  (a TIGHT bound, e.g. rel ≤ 2e-2, ≈ deflection — not a loosened curved-parity band): the
  frustum is the exact chamfer surface, not an approximation. A fixture whose measured gap
  exceeds the bound is declared out of slice (NULL → OCCT), the gap REPORTED — never passed
  with a loosened bound. Confirm no regression: the curved + variable FILLET cases in
  `run-sim-native-curved-fillet.sh` stay green (23/23), `run-sim-native-blend.sh` (16/16,
  including the PLANAR chamfer control) + `run-sim-suite.sh` stay green.

## Decisions

- **Reuse the curved-fillet classifier + weld; swap the torus canal for a cone frustum.**
  The chamfer shares the fillet's `facesOnRim` / `rimGeom` classifier, `math::Ax3` frame,
  `sagittaSteps` budget, `ringPoint` sampler, and planar-triangle weld — the ONLY new
  geometry is the straight frustum band (one meridian step) and the two setback circles.
  Minimal new code, minimal regression surface.
- **Trim to the SETBACK circles, not the TANGENT circles.** The fillet trims to the
  torus∩surface tangent circles (`Rc` at `H − s·r`, `Rc − r` at `H`); the chamfer trims
  to the SETBACK circles at the SAME radii/heights (`Rc` at `H − s·d`, `Rc − d` at `H`) —
  numerically the same loci for `r = d`, but the band between them is a STRAIGHT frustum,
  not a curved arc, so the interior surface and the removed volume differ.
- **C0 at the chamfer angle — NEVER G1.** The self-verify asserts the frustum meets each
  face at the chamfer angle (`cos = 1/√2` for symmetric `d`), explicitly NOT tangent. A
  chamfer is a flat bevel; asserting tangency (as the fillet does) would be WRONG. This is
  the deliberate inverse of the fillet's G1 assertion.
- **`wantGrow=false` (SHRINK) — a chamfer removes material.** Same guard the planar
  chamfer uses; no new guard type. The candidate is tried after the planar chamfer, both
  under the identical SHRINK self-verify.
- **Exact frustum, tight parity.** The symmetric chamfer surface IS a cone frustum
  exactly, so the native↔OCCT gap is only the angular deflection — a genuine strength, not
  a loosened tolerance. Out-of-slice (asymmetric / non-circular / concave / cyl-cyl /
  `Rc ≤ d`) → OCCT, gap REPORTED.

## Risks / Trade-offs

- **Degenerate cap circle.** As `d → Rc` the cap circle `Rc − d → 0` and the frustum
  degenerates to a full cone tip; the builder guards `Rc − d > eps` and returns NULL →
  OCCT below it, rather than emit a near-singular facet fan. Bounded and honest.
- **Seam-inside-face tolerance.** Deciding "the cylinder seam stays on the wall"
  (`s·((H − s·d) − hFar) > 0`) uses scale-derived tolerances; a borderline config (the
  wall barely longer than `d`) returns NULL → OCCT rather than a fragile patch, matching
  the roadmap's degeneracy stance.
- **Slice narrowness vs honesty.** Only the SYMMETRIC-distance convex cylinder↔coaxial-cap
  circular rim lands; asymmetric distances, non-circular creases, concave rims, cyl↔cyl
  chamfers, and freeform faces defer. Accepted — the alternative is faking a curved
  chamfer, which the project forbids. The measured OCCT-fallback gap is REPORTED, never
  masked.
- **Wrong-blend-type confusion guard.** Because the frustum and the torus share the same
  two trim circles, a volume-only self-verify could in principle accept a torus where a
  frustum is expected (or vice-versa). The bevel-angle self-verify (`cos = 1/√2`, C0, NOT
  `cos = 1`) is the discriminator: the curved-chamfer builder emits a frustum and asserts
  the C0 chamfer angle; the fillet builder emits a torus and asserts G1 — so the two can
  never be confused, and the host test checks the removed volume equals the frustum's
  `π d²(Rc − d/3)` (distinct from the fillet's), never the torus'.
