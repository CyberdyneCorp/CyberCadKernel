# Tasks — add-native-concave-fillet (#6 curved blends, concave circular rim)

Verification levels: **host** = OCCT-free host CTest (seam-on-both-surfaces +
watertight + closed-form ADDED-volume + G1-tangency at the two seams); **sim** =
native-vs-OCCT `BRepFilletAPI_MakeFillet` parity on a booted simulator
(volume / area / watertight / G1 at the seams within the curved-parity tol). No
`cc_*` signature is added — the concave-rim path lands behind the existing
`cc_fillet_edges`.

> Mechanism note (mirrors the convex slice, three signs flipped): the concave builder
> lives beside the convex one in `src/native/blend/curved_fillet.h` (new
> `concave_fillet_edge(...)`), reusing `cylinderInfo`, `circleOf`, `sagittaSteps`,
> `ringPoint`, and the `emit/emitTri/emitQuad` facet helpers. It rebuilds the region
> as ONE deflection-bounded planar-facet soup — wall up to the `v=0` seam (`H+r`), the
> CONCAVE torus quarter-tube (major `Rc+r`, minor `r`, tube-centre at `H+r`), and the
> LARGER plane as an ANNULUS with inner radius `Rc+r` — all sharing the same `N`
> angular samples, welded via the boolean `assembleSolid`. The two seams are closed
> form (torus∩cylinder → circle `Rc` at `H+r`; torus∩plane → circle `Rc+r` at `H`),
> analytically identical to SSI-S1 — no solver, no NUMSCI.

## 1. Classify the CONCAVE cylinder↔larger-plane crease
- [x] 1.1 `curved_fillet.h` gains `facesOnConcaveRim`: resolve a CIRCULAR edge
  (`EdgeCurve::Kind::Circle`, `circleOf`) and, by GEOMETRY, the sole coaxial
  `Cylinder` face at the rim radius (reuse `isRimCylinder`) and the sole `Plane` face
  through the rim with normal ∥ axis (reuse `isRimCap`). (**host**)
- [x] 1.2 CONCAVE signature test: the plane extends BEYOND `Rc` (larger-plane, not a
  cap) AND the corner is on the material side (point-in-solid probe just outside the
  wall at the plane height is INSIDE). The convex cap case ⇒ NULL (convex builder /
  OCCT handles it). Exposes `Rc`, axis `Ax3`, plane height `H`, material-side sign,
  plane outer loop (`concaveRimGeom`). (**host**)
- [x] 1.3 Precondition guard: single picked edge, `r > kBlendEps`, both seams inside
  their faces (plane annulus reaches `≥ Rc+r`; wall length covers `H+r`). No
  ring-torus guard (`Rc+r > r` always). Any failure ⇒ NULL (→ OCCT). (**host**)

## 2. Concave torus canal-surface patch (offset-sign FLIP)
- [x] 2.1 The coaxial canal torus (major `R_t = Rc+r`, minor `r`, tube-centre circle
  at axial `H+r`) is evaluated directly (matching `math::Torus`) — the offset sign is
  `+r` (convex was `−r`). (**host**)
- [x] 2.2 Tiles the CONCAVE quarter-tube (`v∈[0,π/2]`, `radius(v)=Rc+r−r·cos v`,
  `axial(v)=H+r−s·r·sin v`) × full `u` into `N·M` facets — `N` from the major sagitta
  (`Rc+r`), `M` from the minor sagitta (`r`), both ≤ deflection (`sagittaSteps`
  reused). Each quad → 2 TRIANGLES (planar). Wall/torus/annulus share the SAME `N`
  samples ⇒ coincident seam vertices. (**host**)

## 3. Trim seams (closed form, exact — solver-free, NUMSCI-OFF)
- [x] 3.1 **torus ∩ cylinder (coaxial)** → tangent CIRCLE radius `Rc` at `H+r` (the
  `v=0` INNER-equator ring, because the tube bulges outward from the axis); the wall
  is rebuilt only up to it. (**host** + **sim**)
- [x] 3.2 **torus ∩ plane (axis-perpendicular)** → tangent CIRCLE radius `Rc+r` in the
  plane `z=H` (the `v=π/2` ring); the LARGER plane is rebuilt as an ANNULUS with that
  inner radius. (**host** + **sim**)
- [x] 3.3 Seams are constructed ON the torus by evaluation, so they lie on it by
  construction; G1 tangency (torus normal == cylinder radial at `v=0`, == plane normal
  at `v=π/2`) is verified analytically: `cos = 1.000000000000`. (**host**)

## 4. Rebuild + insert + weld watertight (material side, annulus plane)
- [x] 4.1 The wall is rebuilt to the `v=0` seam (`H+r`); the LARGER plane to an ANNULUS
  (inner `Rc+r`, outer = the plane's existing loop); the CONCAVE quarter-tube fills the
  corner (planar-facet rings + torus triangles, deflection-bounded). (**host**)
- [x] 4.2 All facets (wall + concave torus + annulus plane + the rest of the boss/plate
  faces, sharing canonical seam vertices) feed the boolean `assembleSolid` → a native
  `Solid`; the torus patch closes the shell G1-tangent to both faces. Watertight
  (`isWatertight`), 0 boundary edges. (**host**)

## 5. Engine self-verify SIGN per convex / concave (the load-bearing change)
- [x] 5.1 `NativeEngine::fillet_edges` tries planar (verify SHRINK, `wantGrow=false`),
  then CONVEX `curved_fillet_edge` (verify SHRINK), then CONCAVE
  `concave_fillet_edge` (verify GROW, `wantGrow=true`); the first candidate that passes
  ITS correctly-signed `blendResultVerified` wins. (**host**)
- [x] 5.2 The CONCAVE result flows through `blendResultVerified(result, body,
  wantGrow=true)` (watertight + `Vr > Vo`) — the SAME grow branch `offset_face` grow
  uses; NULL builder OR guard failure ⇒ OCCT `BRepFilletAPI_MakeFillet`. No new guard,
  no weakened tolerance. Confirm the CONVEX control still verifies with `wantGrow=false`
  (no sign confusion — a convex candidate can never pass grow, concave never passes
  shrink). (**host**)
- [x] 5.3 Out-of-slice inputs still return NULL (asserted in a
  `concave_fillet_scope_defers` test): CONVEX cap rim (→ convex builder), variable
  radius, cyl↔cyl canal, non-circular / cone / sphere / tilted / non-coaxial plane,
  freeform, ≠2-face edge, seam-leaves-face, `r≤0`, multi-edge. (**host** + **sim**)

## 6. Verification (two gates)
- [x] 6.1 Host suite (no OCCT): `test_native_blend.cpp` adds
  `concave_fillet_boss_on_plate_watertight_volume_grown` (watertight + volume GROWN vs
  sharp + matches the closed-form ADDED rim-band, rel ≤ 5e-3),
  `concave_fillet_g1_tangent_at_both_seams` (analytic G1 cos=1 at both seams + seam
  radii Rc / Rc+r), and `concave_fillet_scope_defers` (out-of-slice → NULL, incl. the
  blind-hole bottom rim which is DEFERRED to OCCT this slice). Built default +
  NUMSCI-ON. (**host**)
- [x] 6.2 Sim parity: extend `scripts/run-sim-native-curved-fillet.sh` +
  `tests/sim/native_curved_fillet_parity.mm` — native vs OCCT `BRepFilletAPI` on (A) a
  boss-on-plate base rim over a small Rc/r sweep (2 fixtures × 3 checks): vol rel ≤
  3.8e-3, area rel ≤ 2.1e-3, watertight, G1 `cos=1.0` at both seams, `grew=1`. (B) the
  blind-hole bottom rim is DEFERRED to OCCT this slice (different concave config).
  (**sim**)
- [x] 6.3 No regression: the CONVEX rim cases in the SAME script stay green (9/9);
  `run-sim-native-blend.sh` 16/16; native booleans + SSI + curved-boolean + healing +
  import host suites green under NUMSCI + default. (**sim** + **host**)
- [x] 6.4 `openspec validate add-native-concave-fillet --strict` green. (**host**)

## Deferred (NOT in this slice — honest NULL → OCCT)

- [ ] **Variable-radius concave rim** fillet → `fillet_edges_variable` (already OCCT).
- [ ] **cylinder ↔ cylinder concave canal fillet** (two curved faces; general
  canal/pipe surface, not a torus) → OCCT.
- [ ] **Non-circular concave creases** (cone↔plane rim, sphere rims, ellipse/spline
  creases, non-coaxial or tilted plane) → OCCT.
- [ ] **Any freeform (NURBS/Bézier/B-spline) adjacent face** → OCCT.
- [ ] **Near-degenerate `r`** (a seam leaving its face — plane annulus < `Rc+r`, or
  wall shorter than `r`) → NULL → OCCT — never faked with a weakened tolerance.
