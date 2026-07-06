# Design — add-native-variable-fillet (#6 curved blends, variable-radius circular rim)

## Context

The native CONSTANT-radius CONVEX circular fillet (`src/native/blend/curved_fillet.h`,
`curved_fillet_edge` → `buildFilletedCylinder`) rounds the rim where a cylinder LATERAL
face meets a COAXIAL PLANAR CAP in a CONVEX dihedral. A ball of FIXED radius `r` rolled
on the outside of that crease has its centre at cylindrical radius `Rc − r`, axial
`H − r`; the canal is a coaxial TORUS (major `Rc − r`, minor `r`); the two tangent seams
are the CIRCLES torus∩cylinder (`Rc`) and torus∩plane (`Rc − r`); and the fillet REMOVES
material (`0 < Vr < Vo`). Both that slice and its concave sibling deferred a VARIABLE
radius to OCCT.

This design lands the first VARIABLE slice: the SAME convex cylinder↔cap rim, but with
the ball radius varying LINEARLY around the rim. `cc_fillet_edges_variable(body,
edgeIds, count, r1, r2)` today forwards every input to OCCT
`BRepFilletAPI_MakeFillet` (evolved-radius law). We add a native path for the linear law
`r(θ) = r1 + (r2 − r1)·θ/2π`, `θ ∈ [0, 2π)`, the rim angle measured from a fixed seam
start (`r = r1` at `θ = 0`, `r → r2` as `θ → 2π`).

Because `r` now depends on `θ`, the constant builder's single TORUS becomes a
one-parameter FAMILY of meridian arcs — a **swept variable-radius canal**. The
construction is the constant builder with `r` promoted from a scalar to `r(θ)`,
evaluated per angular STATION. Everything else — the `math::Ax3` cylinder frame, the
`sagittaSteps` facet budget, the meridian-arc cross-section, the planar-triangle weld
through the boolean `assembleSolid` — is reused.

The method is **clean-room**: derive the swept canal + its two non-circular seams in
closed form; OCCT `BRepFilletAPI_MakeFillet` (variable) is the verification ORACLE only,
never copied.

## Goals / Non-Goals

**Goals**
- A native, OCCT-free VARIABLE-radius CONVEX circular-crease fillet: a cylinder lateral
  face meeting a coaxial PLANAR cap in a CONVEX dihedral, with a LINEAR radius law
  `r(θ) = r1 + (r2 − r1)·θ/2π`, both seams staying inside their faces at every station.
- Blend = a swept variable-radius canal (a per-station meridian arc of the local
  `r(θ)`), trimmed to its two NON-circular varying-radius seams (cylinder seam at `Rc`,
  varying axial `H − s·r(θ)`; cap seam at varying radius `Rc − r(θ)`, `z = H`),
  G1-tangent to both faces at EVERY station, welded watertight through `assembleSolid`.
- The fillet REMOVES material: the assembled solid's volume is strictly LESS than the
  body's, gated by the engine's `blendResultVerified(..., wantGrow=false)` branch.
- Behind the unchanged `cc_fillet_edges_variable` ABI; OCCT fallback for the rest.

**Non-Goals (return NULL → OCCT, never faked)**
- NON-linear radius laws (a quadratic / spline evolution, per-vertex laws around a
  multi-vertex edge) — this slice is the LINEAR law only.
- CONCAVE variable rim (cylinder↔larger plane) — the concave canal with a varying radius
  is deferred (the constant concave slice remains constant-only).
- **cylinder↔cylinder canal variable fillet** (two curved faces; a general variable
  pipe/canal surface, not a cyl↔plane rim) — deferred.
- Non-circular creases (cone↔plane rim, sphere rims, ellipse/spline creases, a
  non-coaxial or tilted plane) — deferred.
- Any freeform (NURBS/Bézier/B-spline) adjacent face — deferred.
- Near-degenerate radii (a seam that leaves its face at some station — the cap radius
  `Rc − r(θ) ≤ 0`, or the wall shorter than `max(r1,r2)`, or `Rc < 2·max(r1,r2)` so the
  swept centre curve crosses the axis) — NULL → OCCT.
- A radius GRADIENT so steep that the meridian-arc canal leaves the curved-parity
  tolerance vs OCCT's evolved-law envelope — NULL → OCCT, the measured gap REPORTED.

## The VARIABLE-radius circular-crease rolling-ball geometry (clean-room)

Take a cylinder of radius `Rc`, axis `A` (frame Z), capped by a coaxial planar CAP at
axial height `H` (cap normal ∥ `A`), the rim the CIRCLE of radius `Rc` at `H`, the
dihedral CONVEX (air fills the corner). Let `s = ±1` be the axial sign from the far end
toward the cap (read from the geometry exactly as `buildFilletedCylinder` reads
`capH` vs `farH`). Define the LINEAR radius law around the rim:

```
r(θ) = r1 + (r2 − r1)·θ/2π ,   θ ∈ [0, 2π)
```

A ball of radius `r(θ)` rolled into the convex corner at rim angle `θ` stays tangent to:

- the cylinder (touch the lateral surface from OUTSIDE ⇒ centre at cylindrical radius
  `Rc − r(θ)`), and
- the cap (touch from the material side ⇒ centre at axial `H − s·r(θ)`).

So the **ball-centre locus** is the SWEPT curve `C(θ) = (radius Rc − r(θ), axial
H − s·r(θ))` at rim angle `θ` — NOT a circle (both its radius and its height vary with
`θ`). At each station the ball cross-section in the meridian half-plane at angle `θ` is
a circular arc of radius `r(θ)` from the cylinder-seam point `(Rc, H − s·r(θ))` to the
cap-seam point `(Rc − r(θ), H)`, centred at `C(θ)`:

```
radius(θ, v) = (Rc − r(θ)) + r(θ)·cos v          v ∈ [0, π/2]
axial (θ, v) = (H − s·r(θ)) + s·r(θ)·sin v
```

- `v = 0`  → radius `Rc`, axial `H − s·r(θ)`, normal RADIAL → the CYLINDER seam.
- `v = π/2`→ radius `Rc − r(θ)`, axial `H`,   normal AXIAL(+s) → the CAP seam.

The swept surface `P(θ, v)` (revolve each meridian arc to its angle `θ` about `A`) is
the variable-radius canal — the constant builder's torus with `r → r(θ)`.

### The two trim seams (non-circular, closed form, exact)

- **cylinder seam** (`v = 0`): the curve at radius `Rc` (EXACTLY on the cylinder by
  construction) with axial height `H − s·r(θ)` varying LINEARLY with `θ` — a helix-like
  curve on the cylinder wall (a straight line in the wall's `(θ, z)` unrolling). The
  constant case's CIRCLE (`z = H − s·r` fixed) is the `r1 = r2` degenerate.
- **cap seam** (`v = π/2`): the curve in the plane `z = H` (EXACTLY on the cap by
  construction) at radius `Rc − r(θ)` varying LINEARLY with `θ` — an Archimedean-spiral
  arc in the cap plane. The constant case's CIRCLE (radius `Rc − r`) is the `r1 = r2`
  degenerate.

Both seams lie on their neighbour surface exactly (the wall seam has `radius = Rc`; the
cap seam has `axial = H`), so the on-surface check is exact — no solver, no NUMSCI. This
is the varying-radius generalisation of the constant slice's SSI-S1 circle seams.

### G1-tangency at both seams holds for ANY radius law (the load-bearing lemma)

The blend surface tangent plane at a seam is spanned by `∂P/∂v` and `∂P/∂θ`. Compute
`∂/∂v` of the meridian arc:

```
∂radius/∂v = −r(θ)·sin v ,   ∂axial/∂v = s·r(θ)·cos v
```

- At **`v = 0`**: `∂radius/∂v = 0`, `∂axial/∂v = s·r(θ)` → `∂P/∂v` is PURELY AXIAL. And
  `∂P/∂θ` at `v = 0` is `∂/∂θ (Rc·cos θ, Rc·sin θ, H − s·r(θ))` = a CIRCUMFERENTIAL
  vector plus an AXIAL component `−s·r'(θ)` — both lie in the cylinder's tangent plane
  (which contains the axial and circumferential directions). So the tangent plane is the
  cylinder's tangent plane and the blend normal is RADIAL == the cylinder normal.
- At **`v = π/2`**: `∂radius/∂v = −r(θ)`, `∂axial/∂v = 0` → `∂P/∂v` is PURELY RADIAL
  in the plane `z = H`. And `∂P/∂θ` at `v = π/2` lies in the plane `z = H` (the whole
  seam is in that plane). So the tangent plane is the cap plane and the blend normal is
  AXIAL == the cap normal.

Both conclusions are INDEPENDENT of `r(θ)` and `r'(θ)` — they use only `∂radius/∂v = 0`
at `v = 0` and `∂axial/∂v = 0` at `v = π/2`, which hold for any `r`. So the swept
meridian-arc canal is G1-tangent to the cylinder at `v = 0` and to the cap at `v = π/2`
for EVERY linear (indeed any differentiable) law, at EVERY station. The builder asserts
`cos = 1` at both seams at every station; any failure ⇒ NULL → OCCT.

### Honest caveat: interior envelope vs OCCT's evolved law

The EXACT rolling-ball envelope of the sphere family `{ centre C(θ), radius r(θ) }` has
a CHARACTERISTIC circle that is TILTED out of the meridian plane by the radius gradient
(`tan(tilt) = r'(θ)/‖C'(θ)‖`); OCCT's evolved-law fillet builds that tilted envelope.
The meridian-arc canal here is UPRIGHT (each cross-section in its meridian plane). The
two agree exactly at the seams (proved above, G1) and in the constant limit
(`r' = 0`); in the interior they differ by `O(r')`. For a MODERATE linear law (e.g.
`r1 = 1.0, r2 = 2.0` over `2π`, `r' = (r2−r1)/2π ≈ 0.16`) the deviation is within the
tessellation deflection + curved-parity band. The design does NOT claim the interior is
bit-identical to OCCT; it claims a well-defined G1 variable-radius blend of the correct
LOCAL radius at each meridian, VERIFIED (a) watertight + volume-shrinking by the engine
guard, (b) against a closed-form swept-volume bound on the host, and (c) against OCCT
within the curved-parity tolerance on the sim. Any `(r1, r2, Rc)` whose measured gap
exceeds the tolerance is OUT OF SLICE → NULL → OCCT, with the gap REPORTED. No tolerance
is ever weakened to pass.

## Convex / linear-law classification (how the builder knows)

The pick is a CIRCLE shared by ONE `Cylinder` lateral face and ONE coaxial `Plane` CAP
whose normal ∥ axis, meeting CONVEX — the EXACT config the constant convex builder
classifies (`detail::facesOnRim` + `detail::rimGeom`). The variable builder reuses those
verbatim: if the edge is not that convex cylinder↔cap rim it returns NULL (the concave
builder, or OCCT, owns the rest). The ONLY additional inputs are the two radii `r1, r2`
defining the linear law; there is no separate concave / cyl-cyl / non-circular path in
this slice.

## Module shape

```
src/native/blend/
  curved_fillet.h          // extend: add the VARIABLE builder next to the constant
                           //   convex + concave ones. New entry
                           //   `variable_fillet_edge(...)` mirrors
                           //   `curved_fillet_edge(...)`; shared helpers
                           //   (facesOnRim, cylinderInfo, rimGeom, sagittaSteps,
                           //   ringPoint, emit/emitTri/emitQuad) are reused unchanged.
  blend_geom.h             // REUSED — facePlane, signedDist.
  native_blend.h           // already includes curved_fillet.h; no change.
```

`variable_fillet_edge(const topo::Shape&, const int* edgeIds, int edgeCount, double r1,
double r2, double deflection)` returns the filleted solid or a NULL Shape. It reuses the
constant convex classifier (`facesOnRim`, `cylinderInfo`, `facePlane`, `rimGeom`) and
the `sagittaSteps` / `ringPoint` / `emit*` facet helpers; the only new code is
`buildVariableFillet(const RimGeom&, double r1, double r2, double defl)` — the
constant `buildFilletedCylinder` with `r` promoted to a per-station `r(θ)` and the two
seam rings replaced by their varying-radius station loops.

## Build pipeline (concrete)

1. **Classify the picked edge.** Reuse `detail::facesOnRim` + `detail::rimGeom`: resolve
   the 1-based edge id to a `Circle` `EdgeCurve` of radius `Rc` coaxial with a `Cylinder`
   face (radius `Rc`, axis `A`) and a `Plane` CAP whose normal ∥ `A`, meeting CONVEX.
   Require exactly one of each at the rim. Else return NULL.
2. **Precondition guard.** Require `edgeCount == 1`, `r1 > kBlendEps`, `r2 > kBlendEps`,
   and both seams inside their faces at EVERY station — since `r(θ)` is monotone in `θ`
   the extremes are `min(r1,r2)` and `max(r1,r2)`, so it suffices to guard with
   `rmax = max(r1,r2)`: `Rc ≥ 2·rmax` (the swept centre curve never reaches the axis;
   the cap radius `Rc − r(θ) ≥ Rc − rmax > 0`), and the far end beyond the wall seam
   `s·(hSeam(rmax) − hFar) > 0`. Any failure ⇒ NULL → OCCT. (`r1 == r2` is allowed — it
   reproduces the constant torus; the constant path still owns constant `cc_fillet_edges`.)
3. **Facet budget.** `N` angular stations from `max(sagittaSteps(Rc, 2π, defl),
   sagittaSteps(rmax, 2π, defl))` PLUS a gradient term so the seam-height / seam-radius
   step per station stays under `defl` (`N ≥ ⌈|r2 − r1| / defl⌉`); `M` minor steps from
   `sagittaSteps(rmax, π/2, defl)`. Wall / canal / cap share the SAME `N` samples ⇒
   coincident seam vertices.
4. **Tile the swept canal.** For each station `i` (`θ_i = 2π·i/N`) compute `r(θ_i)` and
   the meridian arc; emit the `θ × v` grid as `N·M` planar TRIANGLE pairs (each quad
   split into two exactly-planar triangles carrying their own geometric normal, exactly
   as the constant builder does — a curved quad is not coplanar). At `v = 0` the ring is
   at radius `Rc`, axial `H − s·r(θ_i)` (the wall seam); at `v = π/2` at radius
   `Rc − r(θ_i)`, axial `H` (the cap seam).
5. **Rebuild the neighbours to the non-circular seams.** The cylinder wall from the far
   end up to the `v = 0` seam loop (per-station height `H − s·r(θ_i)`); the far cap as
   a full `N`-gon disk (radius `Rc`); the TRIMMED cap as the region of the plane `z = H`
   OUTSIDE the `v = π/2` seam loop up to the cap's outer boundary — realised as an
   `N`-station strip between the seam loop (radius `Rc − r(θ_i)`) and the cap rim
   (radius `Rc`), so the varying-radius cap seam welds to the canal. All share the `N`
   samples.
6. **Verify seams (on surface by construction + G1).** Assert every wall-seam station
   has `radius = Rc` and every cap-seam station has `axial = H` (exact by construction),
   and assert G1 `cos = 1` at both seams at every station (`∂P/∂v` axial at `v=0`,
   radial-in-plane at `v=π/2`). On any failure return NULL.
7. **Weld watertight.** Feed the facets to the boolean `assembleSolid` (weld +
   triangulate) → a native `Solid`.
8. **Return** the solid to `NativeEngine::fillet_edges_variable`, which runs
   `blendResultVerified(result, body, wantGrow=false)` — the SHRINK branch.

Cognitive-complexity: the classifier + guard is flat guard-clauses (backend band ≤15);
the canal tiling is an isolated loop with two documented sagitta bounds plus the gradient
term (systems band), sharing the constant builder's helpers; the seam derivations are
closed-form.

## Engine wiring (the load-bearing change)

`NativeEngine::fillet_edges_variable` changes from a pure OCCT fall-through to a
native-first dispatch mirroring the constant `fillet_edges` convex candidate:

```cpp
ShapeResult NativeEngine::fillet_edges_variable(EngineShape body, const int* e, int ec,
                                                double r1, double r2) {
  if (!isNative(body)) return fallback().fillet_edges_variable(body, e, ec, r1, r2);
  const auto* h = static_cast<const NativeShape*>(body.get());

  // VARIABLE convex circular rim (cylinder ↔ coaxial cap, linear r(θ)) — REMOVES
  // material, verified SHRINK. NULL builder / failed self-verify → honest error
  // (the OCCT engine serves the call — never forward a native void to OCCT).
  ntopo::Shape result = nblend::variable_fillet_edge(h->shape, e, ec, r1, r2);
  if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
    return track(wrapNative(std::move(result)));

  return make_error(
    "native fillet_edges_variable: no verified watertight result for this native body "
    "(non-linear law / non-circular / concave / cyl-cyl canal / tilted / freeform / "
    "gradient out of parity tolerance → OCCT-only)");
}
```

A convex variable fillet REMOVES material, so the guard is the SAME `wantGrow=false`
SHRINK branch the constant convex fillet uses (`0 < Vr < Vo`). A NULL result or a failed
self-verify DISCARDS the candidate; because a native body cannot be forwarded to OCCT
(OCCT would misread the native void), the honest outcome is an error, and the shipping
parity path serves the call from the OCCT engine (`cc_set_engine(0)`) — exactly how the
constant `fillet_edges` treats an unbuildable native rim. No new guard type, no weakened
tolerance.

## Verification model (two gates)

- **Host (no OCCT):** build a native capped cylinder on the host; fillet its top rim
  natively with `variable_fillet_edge(r1, r2)`; assert (a) every wall-seam station lies
  on the cylinder (`radius = Rc`) and on the swept canal, and every cap-seam station
  lies on the cap (`axial = H`) and on the canal, within tol; (b) the mesh is watertight
  (`boundaryEdgeCount == 0`) across the deflection ladder; (c) the enclosed volume equals
  `|body| − V_removed` within the tessellation deflection bound, where `V_removed` is the
  closed-form SWEPT removed rim-band — the meridian corner area `A(θ) = r(θ)²(1 − π/4)`
  integrated around the rim at the local seam radius (a Pappus-with-varying-radius
  integral, evaluated in closed form for the linear law); (d) the canal normal is
  G1-tangent (`cos = 1`) to the cylinder at `v = 0` and the cap at `v = π/2` at every
  station; and (e) the `r1 = r2` degenerate reproduces the constant convex torus volume
  (no regression / no sign confusion). Built default + NUMSCI-ON.
- **Sim native-vs-OCCT parity** (`scripts/run-sim-native-curved-fillet.sh` extended +
  its `.mm`): on a booted simulator, fillet the SAME cylinder top rim with the native
  engine (`cc_set_engine(1)`) and with OCCT (`cc_set_engine(0)`); OCCT builds the
  variable fillet with the evolved law — `BRepFilletAPI_MakeFillet` with `SetRadius(r1)`
  at one rim vertex and `SetRadius(r2)` at the other (or an `EvolvedRadius` law) — and
  the harness compares volume / surface area / watertightness / presence of the variable
  blend face and G1-tangency at the two varying-radius seams within the curved-parity
  tolerance, for (A) `r1 = 1.0, r2 = 2.0` and (B) a second `(r1, r2)` pair. The measured
  gap is REPORTED; a fixture whose gap exceeds the tolerance is declared out of slice
  (NULL → OCCT) rather than passed with a loosened bound. Confirm no regression: the
  CONSTANT convex + concave rim cases in the SAME script stay green (15/15),
  `run-sim-native-blend.sh` (16/16) + `run-sim-suite.sh` stay green.

## Decisions

- **Reuse the constant convex builder; promote `r` to `r(θ)`.** The variable case is the
  constant case with the scalar `r` read PER STATION as `r(θ) = r1 + (r2−r1)·θ/2π`. It
  shares the `math::Ax3` frame, `facesOnRim` / `rimGeom` classifier, `sagittaSteps`,
  `ringPoint`, and the planar-triangle weld — no new geometry engine, minimal new code,
  minimal regression surface.
- **`wantGrow=false` (SHRINK) — a convex variable fillet removes material.** Same guard
  the constant convex fillet uses; no new guard type. The concave-variable case (which
  would grow) is DEFERRED, so the variable path only ever verifies SHRINK.
- **Seams on their surfaces by construction; G1 proven for any law.** The wall seam has
  `radius = Rc` and the cap seam has `axial = H` exactly, so the on-surface check is
  exact; G1 holds analytically at every station because `∂radius/∂v = 0` at `v = 0` and
  `∂axial/∂v = 0` at `v = π/2`, independent of `r'(θ)`.
- **Guard with `rmax = max(r1,r2)`.** `r(θ)` is monotone, so guarding the worst-case
  station with `rmax` (`Rc ≥ 2·rmax`, cap radius `Rc − rmax > 0`, wall length ≥ `rmax`)
  keeps EVERY station inside its face.
- **Meridian-arc canal, gated against OCCT.** The interior is the upright meridian-arc
  canal, not OCCT's tilted evolved envelope; the two agree at the seams and in the
  constant limit and are gated by the host swept-volume bound + the sim curved-parity
  tolerance. Out-of-tolerance gradients are out of slice → OCCT, gap REPORTED.

## Risks / Trade-offs

- **Interior envelope approximation.** The meridian-arc canal differs from OCCT's
  evolved-law envelope by `O(r')` in the interior. Bounded by the host swept-volume test
  and the sim parity tolerance; a steep-gradient fixture that exceeds it is declared out
  of slice (NULL → OCCT), the gap REPORTED — never masked with a weakened tolerance.
- **Non-circular seam weld.** The wall / cap seams are per-station loops (varying height
  / radius); the wall, canal, and trimmed-cap strips must share the SAME `N` station
  samples so the loops weld with coincident vertices. If the cap's outer boundary is not
  available natively the builder returns NULL → OCCT. Bounded and honest.
- **Seam-inside-face tolerance.** Deciding "seam within the face at every station" uses
  `rmax` and scale-derived tolerances; borderline configs (`Rc` barely `≥ 2·rmax`,
  cap radius near zero at the large-`r` end) return NULL → OCCT rather than a fragile
  patch, matching the roadmap's degeneracy stance.
- **Slice narrowness vs honesty.** Only the LINEAR-law convex cylinder↔coaxial-cap
  circular rim lands; non-linear laws, concave-variable, cyl↔cyl-canal, non-circular,
  and freeform defer. Accepted — the alternative is faking a variable blend, which the
  project forbids. The measured OCCT-fallback gap is REPORTED, never masked.
