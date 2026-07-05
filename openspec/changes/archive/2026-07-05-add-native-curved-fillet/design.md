# Design — add-native-curved-fillet (#6 curved blends, first slice)

## Context

The native planar fillet (`src/native/blend/fillet_edges.h`) implements the
rolling-ball construction for a STRAIGHT convex dihedral: a ball of radius `r`
rolled into the crease traces a CYLINDER (axis ∥ crease), seated tangent to both
planes, and the sharp edge is replaced by the cylinder arc between the two tangent
lines `T_i = C + r·n_i`, G1-tangent to both faces. Curved-face inputs return NULL →
OCCT.

`ROADMAP.md` #6 (curved blends) and `SSI-ROADMAP.md` (§Sequencing: `S5 → #6
blends`) place the FIRST curved slice directly on the finished stack: a
constant-radius rolling-ball fillet on a **CIRCULAR crease**. The canal surface of
a circular crease is a **TORUS**, and the two trim seams are **CIRCLES** already
covered by native SSI **S1** (torus∩cylinder coaxial, torus∩plane axis-perp). This
design derives that construction and gates it with the engine's EXISTING blend
self-verify → OCCT fallback.

The method is **clean-room**: derive the torus + seam circles in closed form; OCCT
`BRepFilletAPI_MakeFillet` is the verification ORACLE only, never copied.

## Goals / Non-Goals

**Goals**
- A native, OCCT-free circular-crease fillet: cylinder-lateral↔planar-cap rim,
  constant radius `r`, CONVEX crease, `r` small enough that both tangent circles
  stay inside their faces.
- Blend = a TORUS canal-surface patch (major `R−r`, minor `r`, coaxial), trimmed to
  its two CIRCLE seams (torus∩cylinder + torus∩plane, native SSI S1), G1-tangent to
  both faces, welded watertight through the boolean `assembleSolid`.
- Every seam circle **self-verified** on BOTH surfaces before use; the assembled
  solid gated by the engine's EXISTING watertight + volume-reducing guard.
- Behind the unchanged `cc_fillet_edges` ABI; OCCT fallback for the rest.

**Non-Goals (return NULL → OCCT, never faked)**
- CONCAVE circular rim (an internal fillet; the ball centre circle is `R + r` and
  the tangency/self-intersection guard differs) — deferred.
- Variable-radius fillet on the rim (that is `fillet_edges_variable`, already OCCT).
- **cylinder↔cylinder canal fillet** (two curved faces; the canal surface is not a
  torus but a general canal / pipe surface) — deferred.
- Non-circular curved creases (cap not planar, cone↔plane rim, sphere rims,
  ellipse/spline creases) — deferred.
- Any freeform (NURBS/Bézier/B-spline) adjacent face — deferred.
- Near-tangent / near-degenerate `r` (r ≥ R, r so large a seam leaves its face, or
  the two seams collide) — return NULL → OCCT.

## The circular-crease rolling-ball geometry (clean-room)

Take a native cylinder solid: lateral `Cylinder` face of radius `R`, axis `A`
(frame Z), and a planar `Cap` at axial height `z_cap` with outward normal ∥ `A`
(say `+Z` for the top cap). The rim is the CIRCLE of radius `R` at `z_cap`. A ball
of radius `r` rolled along the OUTSIDE of this convex rim stays tangent to:

- the cylinder (touch the lateral surface from outside ⇒ ball centre at cylindrical
  radius `R − r`), and
- the cap plane (touch from above ⇒ ball centre at axial offset `r` BELOW the cap,
  i.e. `z_cap − r`, on the material side).

So the **ball-centre locus** is the CIRCLE coaxial with `A`, radius `R − r`, at
axial height `z_cap − r`. Revolving the ball's cross-section (radius `r`) about `A`
sweeps the **torus**:

```
T = Torus{ frame: {origin = axisPoint(z_cap − r), Z = A},
           majorRadius R_t = R − r,
           minorRadius r_t = r }
```

This is exactly a native REVOLVE of an off-axis circular arc (the ball cross-section
at distance `R − r` from the axis) → `math::Torus` / `build_revolution_torus`
(`residuals.h`). It is a proper RING torus iff `R_t = R − r > 0` and `R_t ≥ r_t`,
i.e. `R ≥ 2r` — a precondition the builder checks (else NULL → OCCT).

### The two trim seams (native SSI S1 — closed form, exact)

- **torus ∩ cylinder** (coaxial, `quadric_pairs.h`): the ball touches the cylinder
  along the CIRCLE of radius `R` at axial `z_cap − r` (the torus outer equator
  meridian tangent to `ρ = R`). Native SSI's coaxial cyl∩(surface-of-revolution)
  family returns this circle; we take the branch at `ρ = R`.
- **torus ∩ plane** (axis-perpendicular, `plane_torus.h`): the ball touches the cap
  plane `z = z_cap` along the CIRCLE of radius `R_t = R − r` in that plane (the
  axis-perpendicular plane∩torus concentric-circle family — the tangent plane
  touches the torus top at its centre-circle radius).

Both seams are TANGENT (single tangent circle), so the blend is G1 across each seam:
the torus and the cylinder share a tangent plane along the cylinder seam (both
tangent to the common tangent plane whose normal is radial); the torus and the cap
share a tangent plane along the cap seam. G1-tangency at the two seams is asserted
in the verification gate by comparing the torus normal to the neighbouring face
normal along each seam.

## Module shape

```
src/native/blend/
  fillet_circular_edge.h   // NEW — the circular cylinder↔cap rolling-ball fillet
  fillet_edges.h           // existing planar dispatch; add a curved-crease branch
                           //   that delegates to fillet_circular_edge.h when the
                           //   picked edge is a circle on a Cylinder+Plane pair
  blend_geom.h             // REUSED — id→geometry, half-space clip, PlanarModel;
                           //   extend the model to carry the ONE curved (cylinder)
                           //   face + the cap plane for this pair
```

The existing `nblend::fillet_edges(shape, edgeIds, ec, r)` entry (called by
`NativeEngine::fillet_edges`) keeps its signature. Internally it first tries the
planar-dihedral path; if the picked edge resolves to a CIRCLE shared by a
`Cylinder` face and a `Plane` face, it delegates to the new
`filletCircularEdge(...)`. Anything else → the existing NULL return.

## Build pipeline (concrete)

1. **Classify the picked edge.** Resolve the 1-based edge id to its two incident
   faces. Require exactly two: one `FaceSurface`-kind `Cylinder` (radius `R`, axis
   `A`) and one `FaceSurface`-kind `Plane` whose normal ∥ `A` (a cap). Require the
   edge to be a `Circle` `EdgeCurve` of radius `R` coaxial with `A` (the true rim,
   not a sampled polyline). Require CONVEX crease (cap outward normal and cylinder
   outward normal open away from the material). Else return NULL.
2. **Precondition guard.** Require `r > 0` and `R ≥ 2r + ε` (ring torus, seam stays
   on the cylinder), and both computed seam circles lie strictly inside their faces'
   parameter extents (the cap disc radius ≥ `R − r`; the cylinder axial extent
   covers `z_cap − r`). Else return NULL → OCCT.
3. **Build the torus** `T` (major `R − r`, minor `r`, coaxial, at `z_cap − r`) via
   the native revolve / `math::Torus`.
4. **Seam circles via SSI S1.** `seamCyl = ssi::torusCylinderCoaxial(T, cyl)` (the
   `ρ = R` branch); `seamCap = ssi::planeTorus(capPlane, T)` (the axis-perpendicular
   `R − r` circle). Both returned as native `Circle`.
5. **Self-verify seams.** Sample each seam circle; assert every sample lies on the
   torus AND on the neighbouring original surface (cylinder / cap plane) within a
   scale-derived tol. On failure return NULL (never a wrong seam).
6. **Trim faces.** Trim the cylinder lateral face back to `seamCyl` (drop the rim
   band between `seamCyl` and the old rim) and the cap face back to `seamCap` (shrink
   the cap disc to radius `R − r`), reusing the `blend_geom.h` clip discipline
   adapted to a circular trim boundary.
7. **Tile the torus patch.** Sweep the quarter-tube of `T` between `seamCap`
   (minor angle `v = π/2`, on the cap) and `seamCyl` (minor angle `v = 0`, outer
   equator) — a ring of quad strips: `N_major` segments around `u ∈ [0, 2π)`
   (sagitta of `R_t` bounded by deflection) × `N_minor` steps over `v` (sagitta of
   `r` bounded by deflection). Rim vertices are SHARED with the two trimmed faces
   (canonical seam points) so the shell closes.
8. **Weld watertight.** Feed trimmed faces + patch strips to the boolean
   `assembleSolid` (weld + T-junction repair + triangulate) → a native `Solid`.
9. **Return** the solid to `NativeEngine::fillet_edges`, which runs the EXISTING
   `blendResultVerified(result, body, wantGrow=false)` guard.

Cognitive-complexity: the classifier + guard is flat guard-clauses (backend band
≤15); the torus tiling is an isolated loop with two documented sagitta bounds
(systems band); the seam derivations live in SSI S1 already.

## Self-verify → OCCT fallback (unchanged engine discipline)

The engine's `blendResultVerified` (watertight closed 2-manifold across the mesher's
deflection ladder AND a convex fillet strictly REDUCES volume, `0 < Vr < Vo`) is
ALREADY applied to the `fillet_edges` dispatch. The circular-rim result flows
through it unchanged: a NULL builder result OR a guard failure → OCCT
`BRepFilletAPI_MakeFillet`. No new guard, no weakened tolerance. This is the #6
instance of the project's mandatory self-verify → OCCT-fallback rule.

## Verification model (two gates)

- **Host (no OCCT):** build a native cylinder solid on the host; fillet the top rim
  natively; assert (a) both seam circles lie on the torus and on the
  cylinder/cap within tol, (b) the mesh is watertight (`boundaryEdgeCount == 0`)
  across the deflection ladder, (c) the enclosed volume equals `|body| − V_wedge`
  within the tessellation deflection bound, where `V_wedge` is the removed rim-band
  volume (annular corner minus torus quarter-tube — a closed-form Pappus figure), and
  (d) the torus-patch normal is G1-tangent to each neighbour along its seam.
- **Sim native-vs-OCCT parity** (`scripts/run-sim-native-curved-fillet.sh`, modelled
  on `run-sim-native-blend.sh`): on a booted simulator, fillet the SAME rim with the
  native engine (`cc_set_engine(1)`) and with OCCT (`cc_set_engine(0)`); compare
  volume / surface area / watertightness / sub-shape presence of the torus blend face
  and G1-tangency at the two seams against OCCT `BRepFilletAPI_MakeFillet` within the
  curved-parity tolerance. Also confirm no regression: `run-sim-native-blend.sh`
  (planar fillet/chamfer/offset/shell) and `run-sim-suite.sh` stay green.

## Decisions

- **Torus, not a generic canal surface.** For a CIRCULAR crease against a coaxial
  cylinder + axis-normal plane the canal surface is EXACTLY a coaxial torus; we build
  it directly with `math::Torus` + native revolve rather than a general offset/pipe.
  This keeps both seams in the closed-form SSI S1 family.
- **Seams from SSI S1, not marching.** torus∩cylinder(coaxial) and torus∩plane
  (axis-perp) are S1 closed-form; using S1 keeps the slice solver-free (NUMSCI OFF)
  and exact, and reuses verified code.
- **Reuse the engine guard as-is.** A convex fillet reduces volume; the existing
  `blendResultVerified` already encodes `0 < Vr < Vo`. No new self-verify path.
- **Faceted torus patch.** The patch is tiled into deflection-bounded quads (two
  sagitta bounds, one per curvature) exactly like the planar fillet's arc facets, so
  the result welds and meshes through the SAME `assembleSolid`; deflection-bounded vs
  the OCCT toroidal blend face.

## Risks / Trade-offs

- **Slice narrowness vs honesty.** Only the convex cylinder↔cap circular rim with
  `R ≥ 2r` lands; everything else defers. Accepted — the alternative is faking a
  curved blend, which the project forbids.
- **Two-curvature facet budget.** A torus patch needs facets in both `u` and `v`;
  the strip count can grow. Bounded by the deflection ladder and capped like the
  planar arc; if the cap is hit the result still self-verifies watertight or falls
  through.
- **Seam-inside-face tolerance.** Deciding `R ≥ 2r` and "seam within the face" uses
  scale-derived tolerances; borderline (near-tangent, `r` near `R/2`) configurations
  return NULL → OCCT rather than a fragile patch, matching the roadmap's degeneracy
  stance.
