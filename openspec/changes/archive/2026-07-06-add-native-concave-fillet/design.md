# Design — add-native-concave-fillet (#6 curved blends, concave circular rim)

## Context

The native CONVEX circular fillet (`src/native/blend/curved_fillet.h`,
`curved_fillet_edge`) rounds the rim where a cylinder LATERAL face meets a COAXIAL
PLANAR CAP in a CONVEX dihedral. A ball of radius `r` rolled on the OUTSIDE of that
convex crease has its centre at cylindrical radius `Rc − r`, axial `H − r`; the canal
surface is a coaxial TORUS (major `Rc − r`, minor `r`); the two tangent seams are the
circles torus∩cylinder (radius `Rc`) and torus∩plane (radius `Rc − r`); and the
fillet REMOVES material (`0 < Vr < Vo`). That slice deferred the CONCAVE rim to OCCT.

This design lands the concave rim — the inside fillet. A CONCAVE circular rim is where
a cylinder (a BOSS) meets a LARGER planar face in a CONCAVE dihedral: the base rim of
a boss on a plate, or the bottom rim of a blind hole. The construction is the convex
builder with THREE signs flipped: the ball-centre offset (`+r` not `−r`), the material
side the quarter-tube fills, and the volume change (grow not shrink). Everything else —
the `math::Torus` frame, the `sagittaSteps` facet budget, the planar-facet weld through
the boolean `assembleSolid` — is reused unchanged.

The method is **clean-room**: derive the concave torus + seam circles in closed form;
OCCT `BRepFilletAPI_MakeFillet` is the verification ORACLE only, never copied.

## Goals / Non-Goals

**Goals**
- A native, OCCT-free CONCAVE circular-crease fillet: a cylinder lateral face meeting a
  LARGER coaxial plane in a CONCAVE dihedral (boss base rim; blind-hole bottom rim),
  constant radius `r`, both seams staying inside their faces.
- Blend = a coaxial TORUS canal-surface patch (major `Rc + r`, minor `r`) on the
  MATERIAL side, trimmed to its two CIRCLE seams (torus∩cylinder at `Rc` + torus∩plane
  at `Rc + r`), G1-tangent to both faces, welded watertight through `assembleSolid`.
- The fillet ADDS material: the assembled solid's volume is strictly GREATER than the
  body's, gated by the engine's `blendResultVerified(..., wantGrow=true)` branch.
- Behind the unchanged `cc_fillet_edges` ABI; OCCT fallback for the rest.

**Non-Goals (return NULL → OCCT, never faked)**
- CONVEX circular rim (cylinder↔coaxial cap) — the EXISTING convex slice; untouched.
- Variable-radius fillet on the rim (that is `fillet_edges_variable`, already OCCT).
- **cylinder↔cylinder canal fillet** (two curved faces; the canal is a general
  pipe/canal surface, not a torus) — deferred.
- Non-circular concave creases (cone↔plane rim, sphere rims, ellipse/spline creases,
  a non-coaxial or tilted plane) — deferred.
- Any freeform (NURBS/Bézier/B-spline) adjacent face — deferred.
- Near-degenerate `r` (a seam that leaves its face — the plane annulus is not large
  enough for the `Rc + r` seam, or the wall is shorter than `r`) — NULL → OCCT.

## The CONCAVE circular-crease rolling-ball geometry (clean-room)

Take a cylinder (boss) of radius `Rc`, axis `A` (frame Z), standing on a LARGER planar
face at axial height `H` (the plane normal ∥ `A`, and the plane extends WELL beyond
`Rc`). The rim is the CIRCLE of radius `Rc` at `H`. The dihedral between the cylinder
wall and the plane is CONCAVE (measured through the material, the interior angle is
< 180°; material fills the corner). A ball of radius `r` rolled INTO that concave
corner stays tangent to:

- the cylinder (touch the lateral surface from OUTSIDE the wall ⇒ ball centre at
  cylindrical radius `Rc + r` — the offset sign FLIPS vs the convex `Rc − r`), and
- the plane (touch from the material side ⇒ ball centre at axial offset `+r` INTO the
  material, i.e. `H + r`; convex was `H − r`).

So the **ball-centre locus** is the CIRCLE coaxial with `A`, radius `Rc + r`, at axial
`H + r`. Revolving the ball's cross-section (radius `r`) about `A` sweeps the **torus**:

```
T = Torus{ frame: {origin = axisPoint(H + r), Z = A},
           majorRadius R_t = Rc + r,
           minorRadius r_t = r }
```

Because `R_t = Rc + r > r` always, the ring-torus guard (`Rc ≥ 2r`) of the convex case
is UNNECESSARY here — the concave torus is always a valid ring torus. What remains is
the SEAM-inside-face guard: the `Rc + r` plane seam must fit inside the larger plane's
extent, and the `v=0` wall seam (at `H + r`) must be within the wall's axial length.

### The two trim seams (SSI S1 — closed form, exact)

- **torus ∩ cylinder** (coaxial): the ball touches the cylinder along the CIRCLE of
  radius `Rc` at axial `H + r`. On the torus this is the meridian point at minor angle
  `v = 0` — the INNER equator (radius `R_t − r = Rc`), because the tube now bulges
  OUTWARD from the axis (major `Rc + r`) and its innermost meridian sits at `Rc`. This
  is the coaxial cyl∩(surface-of-revolution) S1 branch at `ρ = Rc`.
- **torus ∩ plane** (axis-perpendicular): the ball touches the plane `z = H` along the
  CIRCLE of radius `R_t = Rc + r` in that plane — the `v = π/2` ring (the tube-centre
  circle projected onto the plane). This is the axis-perpendicular plane∩torus S1
  concentric-circle family.

Both seams lie on the torus BY CONSTRUCTION (we evaluate the torus at `v=0` / `v=π/2`),
so the on-torus check is exact. G1-tangency: at `v=0` the torus normal is RADIAL (== the
cylinder outward normal), at `v=π/2` the torus normal is AXIAL (== the plane normal);
both asserted `cos = 1` analytically.

### The concave quarter-tube (the material side)

The convex builder sweeps `v∈[0,π/2]` with `radius(v) = R + r·cos(v)`, `axialOffset(v)
= s·r·sin(v)` (a tube that shrinks in radius and rises toward the cap). The concave
builder sweeps the OTHER quadrant of the same tube — the one that faces the material
corner:

```
radius(v)      = R_t − r·cos(v) = (Rc + r) − r·cos(v)
axialOffset(v) = H + r − s·r·sin(v)          (v grows from the wall toward the plane)
```

- `v = 0`  → radius `Rc`, axial `H + r`, normal RADIAL(+outward) → the CYLINDER seam.
- `v = π/2`→ radius `Rc + r`, axial `H`,   normal AXIAL(toward the plane) → the PLANE seam.

The quarter-tube is CONCAVE (it curves into the corner), so it ADDS the material that a
sharp inside corner lacks. `s` is the sign of the axial direction from the plane toward
the boss body (so the ball sits on the boss side of the plane); it is read from the
geometry exactly as the convex builder reads `capH` vs `farH`.

### Convex vs concave classification (how the builder knows)

The pick is a CIRCLE shared by ONE `Cylinder` face and ONE `Plane` face whose normal
∥ axis. What distinguishes convex from concave is whether the plane extends INSIDE or
OUTSIDE the cylinder radius at the rim:

- **CONVEX (existing):** the plane is a CAP — it covers the disc of radius `≤ Rc`
  (the cylinder's own end face). The material is on ONE side of the plane and INSIDE
  the cylinder. The rolling ball sits OUTSIDE the convex corner (`Rc − r`, `H − r`).
- **CONCAVE (this change):** the plane is LARGER than the cylinder — it extends to
  radius `> Rc` around the rim (a plate the boss stands on, or the flat bottom of a
  blind hole with the wall rising from it). Material fills the corner; the rolling ball
  seats on the material side (`Rc + r`, `H + r`).

The classifier decides by probing the plane's face extent (the larger-plane annulus
reaches beyond `Rc`) AND the material side (a point just outside the wall at the plane
height is INSIDE the solid for concave, OUTSIDE for convex). Both probes use only
native topology + point-in-solid; no OCCT. If the config is ambiguous or is the convex
cap case, the concave builder returns NULL (the convex builder or OCCT handles it).

## Module shape

```
src/native/blend/
  curved_fillet.h          // extend: add the CONCAVE classifier + builder next to
                           //   the convex one. New entry `concave_fillet_edge(...)`
                           //   mirrors `curved_fillet_edge(...)`; shared helpers
                           //   (cylinderInfo, circleOf, sagittaSteps, ringPoint,
                           //   emit/emitTri/emitQuad) are reused unchanged.
  blend_geom.h             // REUSED — facePlane, signedDist, point-in-solid probe.
  native_blend.h           // already includes curved_fillet.h; no change.
```

`concave_fillet_edge(const topo::Shape&, const int* edgeIds, int edgeCount, double r,
double deflection)` returns the filleted solid or a NULL Shape. It reuses
`detail::cylinderInfo`, `detail::circleOf`, `detail::sagittaSteps`,
`detail::ringPoint`, and the `emit/emitTri/emitQuad` facet helpers; the only new code
is the concave classifier (`facesOnConcaveRim`), the concave `rimGeom` (material-side
sign), and the concave `buildConcaveFillet` (the flipped offsets + the annulus plane).

## Build pipeline (concrete)

1. **Classify the picked edge.** Resolve the 1-based edge id to a `Circle` `EdgeCurve`
   of radius `Rc` coaxial with a `Cylinder` face (radius `Rc`, axis `A`) and a `Plane`
   face whose normal ∥ `A`. Require exactly one of each at the rim (as the convex
   builder does). Then require the CONCAVE signature: the plane extends beyond `Rc`
   AND the corner is on the material side (point-in-solid probe just outside the wall
   at the plane height). Else return NULL.
2. **Precondition guard.** Require `r > kBlendEps` and both seams inside their faces:
   the plane annulus reaches radius `≥ Rc + r`, and the wall length covers axial
   `H + r` (the `v=0` seam). No ring-torus guard (always satisfied). Any failure ⇒
   NULL → OCCT.
3. **Build the concave torus** `T` (major `Rc + r`, minor `r`, coaxial, tube-centre at
   `H + r`) via the `math::Torus` frame (evaluated directly, matching the convex path).
4. **Seam circles (closed form).** `seamCyl` = torus∩cylinder = circle radius `Rc` at
   `H + r` (the `v=0` inner-equator ring); `seamPlane` = torus∩plane = circle radius
   `Rc + r` in the plane `z = H` (the `v=π/2` ring). Analytically identical to SSI-S1
   coaxial / plane-torus.
5. **Verify seams (on torus by construction + G1).** The seams are evaluated ON the
   torus, so they lie on it exactly; assert G1 (torus normal == cylinder radial at
   `v=0`; == plane normal at `v=π/2`, `cos = 1`). On any failure return NULL.
6. **Rebuild + tile.** Rebuild the region as a planar-facet soup sharing the SAME `N`
   angular samples: the cylinder wall up to the `v=0` seam (`H + r`); the concave torus
   quarter-tube `v∈[0,π/2] × u` into `N·M` facets (each quad → 2 triangles, exactly
   planar); the LARGER plane as an ANNULUS whose inner boundary is the `v=π/2` seam
   (radius `Rc + r`) and outer boundary is the plane's existing outer loop; and the
   rest of the boss/plate faces unchanged. Rim vertices are SHARED so the shell closes.
7. **Weld watertight.** Feed the facets to the boolean `assembleSolid` (weld +
   triangulate) → a native `Solid`.
8. **Return** the solid to `NativeEngine::fillet_edges`, which runs
   `blendResultVerified(result, body, wantGrow=true)` — the CONCAVE (grow) branch.

Cognitive-complexity: the classifier + guard is flat guard-clauses (backend band ≤15);
the torus tiling is an isolated loop with two documented sagitta bounds (systems band),
shared with the convex builder; the seam derivations are closed-form.

## Self-verify sign per convex / concave (the load-bearing change)

`blendResultVerified(result, original, wantGrow)` (in `native_engine.cpp`) ALREADY
encodes both signs:

- `wantGrow == false` → `0 < Vr < Vo − tol` (chamfer / CONVEX fillet / shell /
  offset-shrink).
- `wantGrow == true`  → `Vr > Vo + tol` (offset-face GROW — and now the CONCAVE
  fillet).

The convex fillet REMOVES material, the concave fillet ADDS material — so the two
cannot share one inequality; the sign MUST be selected per case. The dispatch selects
it by WHICH builder produced the candidate:

```cpp
ShapeResult NativeEngine::fillet_edges(EngineShape body, const int* e, int ec, double r) {
  if (!isNative(body)) return fallback().fillet_edges(body, e, ec, r);
  const auto* h = static_cast<const NativeShape*>(body.get());

  // 1. Planar dihedral (tangent cylinder) — verified SHRINK.
  ntopo::Shape result = nblend::fillet_edges(h->shape, e, ec, r);
  if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
    return track(wrapNative(std::move(result)));

  // 2. CONVEX circular rim (cylinder ↔ coaxial cap) — verified SHRINK.
  result = nblend::curved_fillet_edge(h->shape, e, ec, r);
  if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/false))
    return track(wrapNative(std::move(result)));

  // 3. CONCAVE circular rim (cylinder ↔ larger plane) — verified GROW.
  result = nblend::concave_fillet_edge(h->shape, e, ec, r);
  if (!result.isNull() && blendResultVerified(result, h->shape, /*wantGrow=*/true))
    return track(wrapNative(std::move(result)));

  return make_error("native fillet_edges: no verified watertight result … → OCCT-only");
}
```

Each candidate is checked with ITS OWN correctly-signed guard; a convex candidate can
never pass the grow test and a concave candidate can never pass the shrink test, so the
sign can't be spoofed. Because the concave and convex classifiers are mutually
exclusive (cap vs larger-plane + material-side probe), at most one curved builder
returns non-NULL for a given rim. NULL from all three OR a failed self-verify → the
honest error / OCCT fallback. No new guard type, no weakened tolerance.

## Verification model (two gates)

- **Host (no OCCT):** build a native boss-on-plate solid on the host (a cylinder fused
  onto a larger slab) and a blind-hole solid (a slab with a cylindrical pocket); fillet
  the concave base / bottom rim natively; assert (a) both seam circles lie on the
  concave torus and on the cylinder / plane within tol, (b) the mesh is watertight
  (`boundaryEdgeCount == 0`) across the deflection ladder, (c) the enclosed volume
  equals `|body| + V_fill` within the tessellation deflection bound, where `V_fill` is
  the ADDED concave rim-band volume (a closed-form Pappus solid of revolution:
  the square corner `r²` minus the quarter-disc `πr²/4`, that area's centroid at radius
  `Rc + r − c` revolved about the axis), and (d) the torus-patch normal is G1-tangent
  to each neighbour along its seam (`cos = 1`). Also assert the CONVEX rim control
  still SHRINKS (no regression / no sign confusion). Built default + NUMSCI-ON.
- **Sim native-vs-OCCT parity** (`scripts/run-sim-native-curved-fillet.sh` extended +
  its `.mm`): on a booted simulator, fillet the SAME concave rim with the native engine
  (`cc_set_engine(1)`) and with OCCT (`cc_set_engine(0)`); compare volume / surface
  area / watertightness / presence of the toroidal blend face and G1-tangency at the
  two seams against OCCT `BRepFilletAPI_MakeFillet` within the curved-parity tolerance,
  for (A) a boss-on-plate base rim and (B) a blind-hole bottom rim. Confirm no
  regression: the CONVEX rim cases in the SAME script stay green (9/9), and
  `run-sim-native-blend.sh` (16/16) + `run-sim-suite.sh` stay green.

## Decisions

- **Reuse the convex builder; flip three signs.** The concave case is the convex case
  with the offset sign (`Rc + r` / `H + r`), the swept material side, and the volume
  change flipped. It shares `math::Torus`, `sagittaSteps`, and the planar-facet weld —
  no new geometry engine, minimal new code, minimal regression surface.
- **`wantGrow=true` for concave, `false` for convex — selected by builder.** The two
  fillets have opposite volume signs; the guard already supports both, and the dispatch
  picks the sign by which builder produced the candidate. Mutually-exclusive
  classifiers mean at most one curved builder fires, so the sign is unambiguous.
- **No ring-torus guard for concave.** `R_t = Rc + r > r` always holds, so the concave
  torus is always a valid ring torus; only the SEAM-inside-face guard remains.
- **Seams on the torus by construction.** As in the convex slice, the seams are
  evaluated on the torus (`v=0` / `v=π/2`), so the on-torus check is exact; only the
  G1-tangency to the neighbour surfaces needs asserting.
- **Larger plane trimmed to an ANNULUS.** The concave seam (`Rc + r`) is INSIDE the
  larger plane, so the plane is rebuilt as an annulus (inner radius `Rc + r`, outer =
  the existing loop) rather than the shrunken disc the convex cap uses. The annulus and
  the quarter-tube share the `v=π/2` ring vertices, welding watertight.

## Risks / Trade-offs

- **Classifier robustness (convex vs concave).** The cap-vs-larger-plane +
  material-side probe must be reliable. Borderline configs (plane exactly at `Rc`,
  non-coaxial plane, tilted plane) return NULL → OCCT rather than a mis-signed blend.
  Accepted — the alternative is a wrong sign, which the self-verify would reject anyway,
  but the classifier declines early and honestly.
- **Annulus weld.** Trimming the larger plane to an annulus (a face with an inner hole)
  through the planar-facet soup needs the inner ring to share the `v=π/2` seam
  vertices; if the plane's outer loop is not available natively the builder returns NULL
  → OCCT. Bounded and honest.
- **Seam-inside-face tolerance.** Deciding "seam within the face" (plane annulus ≥
  `Rc + r`, wall length ≥ `r`) uses scale-derived tolerances; borderline cases return
  NULL → OCCT rather than a fragile patch, matching the roadmap's degeneracy stance.
- **Slice narrowness vs honesty.** Only the concave cylinder↔larger-coaxial-plane
  circular rim lands; cyl↔cyl-canal, non-circular, variable, and freeform defer.
  Accepted — the alternative is faking a curved blend, which the project forbids. The
  measured OCCT-fallback gap is REPORTED, never masked.
