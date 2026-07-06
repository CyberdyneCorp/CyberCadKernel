# Tasks — add-native-variable-fillet (#6 curved blends, variable-radius circular rim)

Verification levels: **host** = OCCT-free host CTest (both non-circular seams on their
surfaces + watertight + closed-form SWEPT removed-volume + G1-tangency at both seams at
every station); **sim** = native-vs-OCCT `BRepFilletAPI_MakeFillet` (variable / evolved
law) parity on a booted simulator (volume / area / watertight / G1 at the two
varying-radius seams within the curved-parity tol). No `cc_*` signature is added — the
linear-radius convex rim path lands behind the existing `cc_fillet_edges_variable`.

> Mechanism note (mirrors the constant convex slice, `r` promoted to `r(θ)`): the
> variable builder lives beside the constant convex + concave ones in
> `src/native/blend/curved_fillet.h` (new `variable_fillet_edge(...)`), reusing
> `facesOnRim`, `cylinderInfo`, `rimGeom`, `sagittaSteps`, `ringPoint`, and the
> `emit/emitTri/emitQuad` facet helpers. It rebuilds the capped-cylinder region as ONE
> deflection-bounded planar-triangle soup — far cap, wall up to the (non-circular) `v=0`
> seam, the SWEPT variable canal (`r(θ) = r1 + (r2−r1)·θ/2π`, per-station meridian arc),
> and the cap trimmed to the (non-circular) `v=π/2` seam — all sharing the same `N`
> angular stations, welded via the boolean `assembleSolid`. The two seams are closed
> form per-station (wall seam radius `Rc` at axial `H − s·r(θ)`; cap seam radius
> `Rc − r(θ)` at `z = H`) — no solver, no NUMSCI.

## 1. Classify the CONVEX cylinder↔cap crease + the linear law
- [x] 1.1 `variable_fillet_edge` reuses `detail::facesOnRim` + `detail::rimGeom` to
  resolve a CIRCULAR edge (`EdgeCurve::Kind::Circle`, `circleOf`) shared by the sole
  coaxial `Cylinder` face at the rim radius (`isRimCylinder`) and the sole coaxial
  `Plane` CAP through the rim with normal ∥ axis (`isRimCap`), meeting CONVEX. Not that
  config ⇒ NULL (concave builder / OCCT owns it). (**host**)
- [x] 1.2 Read the linear law `r(θ) = r1 + (r2 − r1)·θ/2π`, `θ ∈ [0, 2π)` from `(r1, r2)`
  (`r = r1` at `θ = 0`, `r → r2` at `θ → 2π`), with `s` = axial sign toward the cap from
  `rimGeom` (`capH` vs `farH`). (**host**)
- [x] 1.3 Precondition guard: single picked edge, `r1 > kBlendEps`, `r2 > kBlendEps`,
  and every station inside its faces via `rmax = max(r1,r2)`: `Rc ≥ 2·rmax` (swept centre
  curve never reaches the axis; cap radius `Rc − r(θ) > 0`), and the far end beyond the
  worst-case wall seam `s·(hSeam(rmax) − hFar) > 0`. Any failure ⇒ NULL (→ OCCT).
  (**host**)

## 2. Swept variable-radius canal patch (`r` → `r(θ)`)
- [x] 2.1 Per angular station `θ_i = 2π·i/N` evaluate `r(θ_i)` and the meridian arc
  centre `(Rc − r(θ_i), H − s·r(θ_i))`; the arc `v∈[0,π/2]` is
  `radius(θ,v) = (Rc − r(θ)) + r(θ)·cos v`, `axial(θ,v) = (H − s·r(θ)) + s·r(θ)·sin v`.
  (**host**)
- [x] 2.2 Tile the `θ × v` grid into `N·M` facets — `N` angular stations from
  `max(sagittaSteps(Rc,2π), sagittaSteps(rmax,2π))` PLUS a gradient term
  `⌈|r2−r1|/defl⌉` (so the per-station seam step ≤ defl), `M` minor from
  `sagittaSteps(rmax, π/2)`, all ≤ deflection. Each quad → 2 TRIANGLES (planar), carrying
  their own geometric normal. Wall / canal / cap share the SAME `N` stations ⇒ coincident
  seam vertices. (**host**)

## 3. Trim seams (non-circular, closed form, exact — solver-free, NUMSCI-OFF)
- [x] 3.1 **cylinder seam** (`v = 0`) → the curve at radius `Rc` (exact on the cylinder)
  with axial `H − s·r(θ)` varying LINEARLY (a line in the wall's `(θ,z)` unrolling); the
  wall is rebuilt only up to it. (**host** + **sim**)
- [x] 3.2 **cap seam** (`v = π/2`) → the curve in the plane `z = H` (exact on the cap) at
  radius `Rc − r(θ)` varying LINEARLY (an Archimedean-spiral arc); the cap is rebuilt as
  an `N`-station strip between that seam loop and the cap rim (radius `Rc`). (**host** +
  **sim**)
- [x] 3.3 Seams are constructed ON their neighbour surfaces by evaluation (wall seam
  `radius = Rc`; cap seam `axial = H`), so they lie on them by construction; G1 tangency
  is verified analytically at EVERY station — `∂radius/∂v = 0` at `v=0` (normal radial ==
  cylinder) and `∂axial/∂v = 0` at `v=π/2` (normal axial == cap): `cos = 1.000000000000`,
  independent of `r'(θ)`. (**host**)

## 4. Rebuild + insert + weld watertight (swept canal, non-circular seams)
- [x] 4.1 The far cap is a full `N`-gon disk (radius `Rc`); the wall is rebuilt from the
  far end to the (non-circular) `v=0` seam loop (per-station axial `H − s·r(θ_i)`); the
  cap is rebuilt as the `N`-station strip between the (non-circular) `v=π/2` seam loop
  (radius `Rc − r(θ_i)`) and the cap rim; the SWEPT canal fills between the seams
  (planar-triangle grid, deflection-bounded). (**host**)
- [x] 4.2 All facets (far cap + wall + variable canal + trimmed-cap strip, sharing
  canonical seam vertices) feed the boolean `assembleSolid` → a native `Solid`; the canal
  patch closes the shell G1-tangent to both faces. Watertight (`isWatertight`), 0 boundary
  edges. (**host**)

## 5. Engine wiring behind `cc_fillet_edges_variable` (native-first + SHRINK self-verify)
- [x] 5.1 `NativeEngine::fillet_edges_variable` changes from a pure OCCT fall-through to:
  if `body` is native, call `nblend::variable_fillet_edge(h->shape, e, ec, r1, r2)` and
  accept it ONLY through `blendResultVerified(result, body, wantGrow=false)` (watertight
  + `0 < Vr < Vo`); else forward. A convex variable fillet REMOVES material, so the guard
  is the SAME SHRINK branch the constant convex fillet uses. (**host**)
- [x] 5.2 A NULL builder result OR a failed self-verify DISCARDS the candidate; because a
  native body cannot be forwarded to OCCT (OCCT would misread the native void), the engine
  returns an honest error and the shipping parity path serves the call from the OCCT
  engine — exactly how the constant `fillet_edges` treats an unbuildable native rim. No
  new guard, no weakened tolerance. (**host**)
- [x] 5.3 Out-of-slice inputs still return NULL (asserted in a
  `variable_fillet_scope_defers` test): NON-linear law, CONCAVE variable rim, cyl↔cyl
  canal, non-circular / cone / sphere / tilted / non-coaxial plane, freeform, ≠2-face
  edge, seam-leaves-face (`Rc < 2·rmax`), `r1≤0` / `r2≤0`, multi-edge, and a gradient
  beyond the parity tolerance. (**host** + **sim**)

## 6. Verification (two gates)
- [x] 6.1 Host suite (no OCCT): `test_native_blend.cpp` adds
  `variable_fillet_cylinder_cap_watertight_volume_between` (watertight + volume SHRUNK vs
  sharp + BRACKETED between the two constant-radius fillets `v(r2)<v<v(r1)` + matches the
  closed-form SWEPT removed rim-band, rel ≤ 6e-3),
  `variable_fillet_second_fixture_and_reversed` (fixture B + the reversed law `r1>r2`, both
  watertight + closed-form),
  `variable_fillet_reduces_to_constant_when_r1_eq_r2` (`r1 = r2` reproduces the constant
  torus volume, rel ≤ 1e-9), `variable_fillet_g1_tangent_at_both_seams` (analytic G1 cos=1
  at both seams at every station + seam loci: wall radius `Rc` / cap radius `Rc − r(θ)`),
  and `variable_fillet_scope_defers` (out-of-slice → NULL). Built default + NUMSCI-ON.
  (**host**)
- [x] 6.2 Sim parity: extend `scripts/run-sim-native-curved-fillet.sh` +
  `tests/sim/native_curved_fillet_parity.mm` — `cc_fillet_edges_variable` under the native
  engine vs OCCT `BRepFilletAPI_MakeFillet::Add(r1,r2,edge)` (EVOLVED law) on a cylinder
  top rim over 2 `(r1,r2)` fixtures ((A) `r1=1.0, r2=2.0`, (B) `r1=0.75, r2=2.25`). HARD
  native gates: watertight, mesh↔B-rep vol rel ≤ 2e-2, `shrank=1`, and native vol vs the
  closed-form SWEPT removed volume rel ≤ 1.5e-2 (relX ≈ 3e-3 measured), G1 `cos=1.0` at
  both seams. The native↔OCCT-evolved parity is a SEPARATE, LOOSER report (rel ≤ 6e-2;
  ≈1.3e-2 measured) because the upright meridian-arc canal differs from OCCT's tilted
  evolved envelope by O(r') in the interior (agrees at both seams + the `r1=r2` limit) —
  the gap is REPORTED honestly, never hidden behind a loosened HARD bound; a fixture beyond
  tol is declared out of slice (NULL → OCCT). (**sim**)
- [x] 6.3 No regression: the CONSTANT convex + concave rim cases in the SAME script stay
  green (15/15); `run-sim-native-blend.sh` 16/16; native booleans + SSI + curved-boolean +
  healing + import host suites green under NUMSCI + default. (**sim** + **host**)
- [x] 6.4 `openspec validate add-native-variable-fillet --strict` green. (**host**)

## Deferred (NOT in this slice — honest NULL → OCCT)

- [ ] **Non-linear radius laws** (quadratic / spline evolution, per-vertex multi-vertex
  laws) → OCCT.
- [ ] **Concave variable rim** (boss cylinder ↔ larger plane, varying radius) → OCCT.
- [ ] **cylinder ↔ cylinder canal variable fillet** (two curved faces; a general variable
  pipe/canal surface) → OCCT.
- [ ] **Non-circular variable creases** (cone↔plane rim, sphere rims, ellipse/spline
  creases, non-coaxial or tilted plane) → OCCT.
- [ ] **Any freeform (NURBS/Bézier/B-spline) adjacent face** → OCCT.
- [ ] **Near-degenerate / steep-gradient radii** (a seam leaving its face — `Rc < 2·rmax`
  or cap radius `Rc − rmax ≤ 0`; or a gradient beyond the curved-parity tolerance) →
  NULL → OCCT — never faked with a weakened tolerance; the measured gap is REPORTED.
