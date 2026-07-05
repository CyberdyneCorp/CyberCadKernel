# Tasks — add-native-curved-fillet (#6 curved blends, first slice)

Verification levels: **host** = OCCT-free host CTest (seam-on-both-surfaces +
watertight + closed-form removed-volume + G1-tangency at the two seams); **sim** =
native-vs-OCCT `BRepFilletAPI_MakeFillet` parity on a booted simulator
(volume / area / watertight / G1 at the seams within the curved-parity tol). No
`cc_*` signature is added — the circular-rim path lands behind the existing
`cc_fillet_edges`.

> Implementation landed in `src/native/blend/curved_fillet.h` (OCCT-free, header-only,
> included by `native_blend.h`) + the engine dispatch in
> `NativeEngine::fillet_edges`. Mechanism note (equivalent geometry, simpler build):
> because the whole native blend pipeline welds + meshes PLANAR polygons, the slice
> rebuilds the filleted capped cylinder as ONE deflection-bounded planar-facet soup
> (wall + torus quarter-tube + trimmed cap + far cap, all sharing the same `N`
> angular samples) and welds it with the boolean `assembleSolid` — rather than
> revolving a torus B-spline and clipping the existing faces. The two tangent seams
> are computed in closed form directly (torus∩cylinder → circle radius `Rc` at
> `H∓r`; torus∩plane → circle radius `Rc−r` at `H`) — analytically identical to the
> SSI-S1 coaxial/plane-torus families — so no solver and no NUMSCI are needed.

## 1. Classify the circular cylinder↔cap crease
- [x] 1.1 `curved_fillet.h` resolves a CIRCULAR edge (`EdgeCurve::Kind::Circle`,
  `circleOf`) and, by GEOMETRY (each face owns its own edge nodes, so identity can't
  pair them), the sole coaxial `Cylinder` face at the rim radius (`isRimCylinder`)
  and the sole `Plane` cap through the rim with normal ∥ axis (`isRimCap`). Exposes
  `Rc`, axis `Ax3`, `capH`, `farH`, cap normal (`rimGeom`). (**host** ✓)
- [x] 1.2 Precondition guard: single picked edge, `r > kBlendEps`, ring torus
  `R = Rc−r ≥ r` (⇔ `Rc ≥ 2r`), wall→torus seam strictly inside the wall
  (`s·(hSeam−hFar) > 0`). Any failure ⇒ NULL (→ OCCT). (**host** ✓)
- [x] 1.3 `NativeEngine::fillet_edges` tries the planar-dihedral path first; on its
  NULL it delegates to `nblend::curved_fillet_edge(...)`; keeps the NULL-else. (**host** ✓)

## 2. Torus canal-surface patch
- [x] 2.1 The coaxial canal torus (major `R=Rc−r`, minor `r`, tube-centre circle at
  axial `H∓r`) is evaluated directly (matching `math::Torus`); non-ring (`R < r`)
  ⇒ NULL. (**host** ✓)
- [x] 2.2 Tiles the quarter-tube (`v∈[0,π/2]`) × full `u` into `N·M` facets — `N`
  from the major-radius sagitta (`Rc`), `M` from the minor-radius sagitta (`r`), both
  ≤ deflection (`sagittaSteps`). Each quad is split into two TRIANGLES (a torus quad's
  4 corners are non-coplanar; triangles stay exactly planar so the facets weld). The
  wall/torus/cap share the SAME `N` samples ⇒ coincident seam vertices. (**host** ✓)

## 3. Trim seams (closed form, exact — solver-free, NUMSCI-OFF)
- [x] 3.1 **torus ∩ cylinder (coaxial)** → tangent CIRCLE radius `Rc` at `H∓r` (the
  `v=0` outer-equator ring); the wall is rebuilt only up to it. (**host** + **sim** ✓)
- [x] 3.2 **torus ∩ plane (axis-perpendicular)** → tangent CIRCLE radius `Rc−r` in
  the cap plane (the `v=π/2` ring); the cap is rebuilt as a disk of that radius.
  (**host** + **sim** ✓)
- [x] 3.3 Seams are constructed ON the torus by evaluation, so they lie on the torus
  by construction; G1 tangency (torus normal == cylinder radial at `v=0`, == cap
  axial at `v=π/2`) is verified analytically: `cos = 1.000000000000`. (**host** ✓)

## 4. Trim + insert + weld watertight
- [x] 4.1 The wall is rebuilt to `hSeam` and the cap to radius `Rc−r`; the far cap is
  a full disk (planar-facet rings, deflection-bounded). (**host** ✓)
- [x] 4.2 All facets (wall + torus + both caps, sharing canonical seam vertices) feed
  the boolean `assembleSolid` → a native `Solid`; the torus patch closes the shell
  G1-tangent to both faces. Watertight (`isWatertight`), 0 boundary edges. (**host** ✓)

## 5. Engine self-verify → OCCT fallback (existing guard, unchanged)
- [x] 5.1 The circular-rim result flows through the EXISTING
  `blendResultVerified(result, body, wantGrow=false)` (watertight + `0 < Vr < Vo`) in
  `NativeEngine::fillet_edges`; NULL builder OR guard failure ⇒ OCCT
  `BRepFilletAPI_MakeFillet`. No new guard, no weakened tolerance. (**host** ✓)
- [x] 5.2 Curved-non-rim / concave / ≠cyl-cap / non-circular / `Rc<2r` / multi-edge /
  `r≤0` inputs still return NULL (asserted in `curved_fillet_scope_defers`); the
  revolve-built SEGMENTED rim (3 patches) also declines — the existing
  `fillet-curved-edge` fallback stays exact-OCCT. (**host** + **sim** ✓)

## 6. Verification (two gates)
- [x] 6.1 Host suite (no OCCT): `test_native_blend.cpp` adds
  `curved_fillet_cylinder_cap_watertight_volume_reduced` (watertight + volume reduced
  vs sharp + matches the closed-form solid-of-revolution `πRc²(h−r)+πr[R²+2Rr·π/4+
  r²·2/3]` to the deflection bound, rel ≤ 5e-3), `curved_fillet_scope_defers`, and
  `curved_fillet_both_rims_and_engine_dispatch`. Built default + NUMSCI-ON. (**host** ✓)
- [x] 6.2 Sim parity: `scripts/run-sim-native-curved-fillet.sh` +
  `tests/sim/native_curved_fillet_parity.mm` — native vs OCCT `BRepFilletAPI` on the
  cylinder↔cap rim over Rc∈{5,4,6}: vol rel ≤ 3.8e-3, area rel ≤ 2.1e-3, watertight,
  G1 `cos=1.0` at both seams. 9/9 pass. (**sim** ✓)
- [x] 6.3 No regression: `run-sim-native-blend.sh` 16/16 (incl. the curved fallback
  still exact-OCCT), native booleans + SSI + curved-boolean host suites 33/33 under
  NUMSCI, default 26/26. (**sim** + **host** ✓)
- [x] 6.4 `openspec validate add-native-curved-fillet --strict` green. (**host** ✓)

## Deferred (NOT in this slice — honest NULL → OCCT)

- [ ] **Concave circular rim** (internal fillet; ball-centre circle `R+r`, different
  self-intersection guard) → OCCT.
- [ ] **Variable-radius rim** fillet → `fillet_edges_variable` (already OCCT).
- [ ] **cylinder ↔ cylinder canal fillet** (two curved faces; general canal/pipe
  surface, not a torus) → OCCT.
- [ ] **Non-circular curved creases** (cone↔plane rim, sphere rims, ellipse/spline
  creases, non-coaxial cap) → OCCT.
- [ ] **Any freeform (NURBS/Bézier/B-spline) adjacent face** → OCCT.
- [ ] **Near-degenerate `r`** (`r ≥ R/2`, seam leaves its face, seams collide) →
  NULL → OCCT — never faked with a weakened tolerance.
