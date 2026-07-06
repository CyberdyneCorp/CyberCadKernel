# Tasks — add-native-curved-chamfer (#6 curved blends, circular-rim cone-frustum chamfer)

Verification levels: **host** = OCCT-free host CTest (both setback circles on their
surfaces + watertight + exact closed-form Pappus removed volume `π·d²·(Rc − d/3)` + the
C0 chamfer-angle bevel geometry, NOT G1); **sim** = native-vs-OCCT
`BRepFilletAPI_MakeChamfer` (`Add(distance, edge)`, symmetric) parity on a booted
simulator (volume / area / watertight / flat cone-frustum bevel at the chamfer angle
within the deflection bound). No `cc_*` signature is added — the symmetric curved circular
chamfer path lands behind the existing `cc_chamfer_edges`.

> Mechanism note (mirrors the curved fillet, torus canal → cone frustum): the curved
> chamfer builder lives in a NEW header `src/native/blend/curved_chamfer.h` (new
> `curved_chamfer_edge(...)`) that `#include`s `curved_fillet.h` to reuse
> `detail::facesOnRim`, `cylinderInfo`, `rimGeom`, `sagittaSteps`, `ringPoint`, and the
> `emit/emitTri/emitQuad` facet helpers. It rebuilds the capped-cylinder region as ONE
> deflection-bounded planar-triangle soup — far cap, wall up to the cylinder seam
> (`H − s·d`), the straight cone-FRUSTUM bevel band (`Rc @ H−s·d → Rc−d @ H`, ONE meridian
> step), and the cap trimmed to the cap seam (`Rc − d @ H`) — all sharing the same `N`
> angular samples, welded via the boolean `assembleSolid`. The two seams are closed-form
> CIRCLES (cylinder seam radius `Rc` at axial `H − s·d`; cap seam radius `Rc − d` at
> `z = H`) — no solver, no NUMSCI. `native_blend.h` gains the new `#include`.

## 1. Classify the CONVEX cylinder↔cap crease + the symmetric distance
- [x] 1.1 `curved_chamfer_edge` reuses `detail::facesOnRim` + `detail::rimGeom` to resolve
  a CIRCULAR edge (`EdgeCurve::Kind::Circle`, `circleOf`) shared by the sole coaxial
  `Cylinder` face at the rim radius (`isRimCylinder`) and the sole coaxial `Plane` CAP
  through the rim with normal ∥ axis (`isRimCap`), meeting CONVEX. Not that config ⇒ NULL
  (planar chamfer / OCCT owns it). (**host**)
- [x] 1.2 Read the SYMMETRIC chamfer distance `d` from `distance`, with `s` = axial sign
  toward the cap from `rimGeom` (`capH` vs `farH`). The setbacks are: cylinder seam radius
  `Rc`, axial `H − s·d`; cap seam radius `Rc − d`, axial `H`. (**host**)
- [x] 1.3 Precondition guard: single picked edge, `d > kBlendEps`, the cap circle real
  (`Rc − d > eps` — the frustum does not collapse to a point / cross the axis), and the far
  end beyond the cylinder seam `s·((H − s·d) − hFar) > 0` (the wall covers the axial
  setback). NO ring-torus guard `Rc ≥ 2d`. Any failure ⇒ NULL (→ OCCT). (**host**)

## 2. Cone-frustum bevel band (straight, single meridian step)
- [x] 2.1 The bevel is the straight meridian line `(Rc, H − s·d) → (Rc − d, H)`, i.e.
  `radius(τ) = Rc − d·τ`, `axial(τ) = (H − s·d) + s·d·τ`, `τ ∈ [0, 1]`; revolved about the
  axis it is a CONE FRUSTUM. NO minor subdivision (`M = 1`, straight bevel). (**host**)
- [x] 2.2 Tile the band into `N` angular quads from `sagittaSteps(Rc, 2π, defl)`; each quad
  `(Rc,u0,H−s·d)→(Rc,u1,H−s·d)→(Rc−d,u1,H)→(Rc−d,u0,H)` → 2 exactly-planar TRIANGLES
  carrying their own geometric normal (a frustum quad is not coplanar — the SAME split the
  torus quads use). Wall / frustum / caps share the SAME `N` samples ⇒ coincident seam
  vertices. (**host**)

## 3. Trim seams (setback CIRCLES, closed form, exact — solver-free, NUMSCI-OFF)
- [x] 3.1 **cylinder seam** (`τ = 0`) → the CIRCLE at radius `Rc` (exact on the cylinder)
  at axial `H − s·d`; the wall is rebuilt only up to it. (**host** + **sim**)
- [x] 3.2 **cap seam** (`τ = 1`) → the CIRCLE at radius `Rc − d` in the plane `z = H`
  (exact on the cap); the cap is rebuilt as the disk of radius `Rc − d`. (**host** +
  **sim**)
- [x] 3.3 Seams are constructed ON their neighbour surfaces by evaluation (cylinder seam
  `radius = Rc`; cap seam `axial = H`), so they lie on them by construction; the bevel
  geometry is verified analytically — the frustum meets the wall and the cap at the CHAMFER
  ANGLE, **C0 NOT G1**: the frustum normal makes `cos = 1/√2` (≈ 0.70710678) with the
  cylinder radial normal at `τ = 0` and `cos = 1/√2` with the cap axial normal at `τ = 1`,
  and is explicitly asserted NOT tangent (`cos ≠ 1`). Asserting G1 would be WRONG for a
  chamfer. (**host**)

## 4. Rebuild + insert + weld watertight (cone frustum, setback circles)
- [x] 4.1 The far cap is a full `N`-gon disk (radius `Rc`, outward `−capNormal`); the wall
  is rebuilt from the far end to the cylinder seam (radius `Rc`, axial `H − s·d`); the cap
  is rebuilt as the TRIMMED disk (radius `Rc − d` at `H`, outward `capNormal`); the
  cone-FRUSTUM band fills between the two setback circles (planar-triangle grid,
  deflection-bounded). (**host**)
- [x] 4.2 All facets (far cap + wall + frustum band + trimmed cap, sharing canonical seam
  vertices) feed the boolean `assembleSolid` → a native `Solid`; the frustum patch closes
  the shell C0 (at the chamfer angle) to both faces. Watertight (`isWatertight`), 0
  boundary edges. (**host**)

## 5. Engine wiring behind `cc_chamfer_edges` (native planar → native curved → SHRINK self-verify)
- [x] 5.1 `NativeEngine::chamfer_edges` changes from a single native (planar) attempt to a
  native-planar → native-curved → error dispatch: try `nblend::chamfer_edges(...)` (planar)
  first; if NULL or unverified, try `nblend::curved_chamfer_edge(h->shape, e, ec, d)`; each
  candidate accepted ONLY through `blendResultVerified(result, body, wantGrow=false)`
  (watertight + `0 < Vr < Vo`). A chamfer REMOVES material, so BOTH use the SAME SHRINK
  branch. (**host**)
- [x] 5.2 A NULL builder result OR a failed self-verify from BOTH slices DISCARDS the
  candidate; because a native body cannot be forwarded to OCCT (OCCT would misread the
  native void), the engine returns an honest error and the shipping parity path serves the
  call from the OCCT engine — exactly how `fillet_edges` treats an unbuildable native rim.
  The planar chamfer path stays byte-identical (a circular rim declines the planar builder
  first). No new guard, no weakened tolerance. (**host**)
- [x] 5.3 Out-of-slice inputs still return NULL (asserted in a
  `curved_chamfer_scope_defers` test): ASYMMETRIC two-distance chamfer, CONCAVE circular
  rim, cyl↔cyl (curved↔curved) rim, non-circular / cone / sphere / ellipse / spline /
  tilted / non-coaxial plane, freeform, ≠2-face edge, `Rc ≤ d` (cap circle collapses),
  wall shorter than `d`, and multi-edge. (**host** + **sim**)

## 6. Verification (two gates)
- [x] 6.1 Host suite (no OCCT): `test_native_blend.cpp` adds
  `curved_chamfer_cylinder_cap_watertight_volume` (watertight + volume SHRUNK vs sharp +
  matches the EXACT closed-form Pappus removed volume `π·d²·(Rc − d/3)`, rel ≤ 6e-3),
  `curved_chamfer_second_distance` (fixture B, watertight + closed-form),
  `curved_chamfer_bevel_c0_not_g1` (frustum normal `cos = 1/√2` to the wall at `τ=0` and to
  the cap at `τ=1`, explicitly NOT tangent `cos ≠ 1`; seam loci: cylinder radius `Rc`, cap
  radius `Rc − d`), `curved_chamfer_removes_more_than_fillet` (`V_removed^chamfer >
  V_removed^fillet` for setback = radius = `d`), and `curved_chamfer_scope_defers`
  (out-of-slice → NULL). Built default + NUMSCI-ON. (**host**)
- [x] 6.2 Sim parity: add `scripts/run-sim-native-curved-chamfer.sh` +
  `tests/sim/native_curved_chamfer_parity.mm` (modelled on the curved-fillet harness) —
  `cc_chamfer_edges` under the native engine vs OCCT `BRepFilletAPI_MakeChamfer` +
  `Add(distance, edge)` (symmetric) on a cylinder top rim over 2 distance fixtures
  ((A) `d = 1.0`, (B) `d = 1.5`). HARD native gates: watertight, mesh↔B-rep vol rel ≤ 2e-2,
  `shrank = 1`, native vol vs the closed-form Pappus removed volume rel ≤ 1e-2, bevel is a
  FLAT cone frustum at the chamfer angle (`cos = 1/√2`, C0) NOT a G1 torus arc. Because the
  symmetric chamfer IS EXACTLY a cone frustum, the native↔OCCT parity is TIGHT (bounded by
  the angular deflection, ≈ 2e-2) — not a loosened curved-parity band; a fixture beyond tol
  is declared out of slice (NULL → OCCT), the gap REPORTED. (**sim**)
- [x] 6.3 No regression: the PLANAR chamfer control + planar blends in
  `run-sim-native-blend.sh` stay green (16/16); the constant + variable curved FILLET cases
  in `run-sim-native-curved-fillet.sh` stay green (23/23); native booleans + SSI +
  curved-boolean + healing + import host suites green under NUMSCI + default. (**sim** +
  **host**)
- [x] 6.4 `openspec validate add-native-curved-chamfer --strict` green. (**host**)

## Deferred (NOT in this slice — honest NULL → OCCT)

- [ ] **Asymmetric two-distance chamfer** (`d1` on the wall, `d2` on the cap, or a
  distance + angle — an oblique cone frustum) → OCCT.
- [ ] **Concave circular chamfer** (a boss cylinder ↔ a larger coaxial plane — the frustum
  would ADD material) → OCCT.
- [ ] **cylinder ↔ cylinder (curved↔curved) chamfer** (two curved faces) → OCCT.
- [ ] **Non-circular curved creases** (cone↔plane rim, sphere rims, ellipse/spline creases,
  non-coaxial or tilted plane) → OCCT.
- [ ] **Any freeform (NURBS/Bézier/B-spline) adjacent face** → OCCT.
- [ ] **Near-degenerate distance** (`Rc ≤ d` so the cap circle `Rc − d ≤ 0`, or the wall
  shorter than `d`) → NULL → OCCT — never faked with a weakened tolerance; the measured gap
  is REPORTED.
