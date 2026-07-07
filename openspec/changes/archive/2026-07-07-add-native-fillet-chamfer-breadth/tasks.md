# Tasks — add-native-fillet-chamfer-breadth (#6 curved blends, off-the-circle breadth)

## IMPLEMENTATION STATUS (honest, per-track)

- **T1 — asymmetric two-distance chamfer: LANDED, EXACT.** Native
  `buildChamferedCylinderAsym` + `curved_chamfer_edge_asym` (oblique cone frustum, C0 at two
  angles), additive `cc_chamfer_edges_asym` facade + `IEngine::chamfer_edges_asym` (OCCT
  `MakeChamfer::Add(d1,d2,edge,face)` override) + `NativeEngine` override (SHRINK self-verify).
  Host: 4 new tests pass (watertight, volume `|body|−π·d1·d2·(Rc−d2/3)`, two-angle C0, scope
  defers). iOS-sim parity: 18/18 (9 symmetric controls + 9 new asymmetric), native == OCCT to
  relO≈2.6e-3, watertight, C0. Symmetric chamfer BYTE-IDENTICAL (asym `d1=d2` wrapper).
- **T2 — elliptical-crease fillet: HONEST DECLINE (no dead code).** BLOCKER (measured): a
  native elliptical fillet needs a native body with a true `Cylinder` face + OBLIQUE `Plane`
  face + `Ellipse` edge. No OCCT-FREE constructor yields that topology — native booleans are
  planar-faced only; the SSI curved boolean recognizes only quadric↔quadric pairs (cyl↔cyl /
  sphere↔sphere / cone↔cyl), NOT a cylinder cut by an oblique half-space. Oblique cuts are
  OCCT-built ⇒ the body is never a `NativeShape` ⇒ the elliptical path is UNREACHABLE natively;
  a builder would be untestable dead code. Documented OCCT-fallthrough in
  `NativeEngine::fillet_edges`; OCCT ref `Rc=5,H=10,60°,r=1` → filleted `383.454285`.
- **T3 — cyl↔cyl-canal fillet: HONEST DECLINE (no dead code).** BLOCKER: for equal-radius
  perpendicular cylinders the two crease loops CROSS at the two Steinmetz poles; a single
  swept-`r`-circle canal cannot close that corner blend watertight and G1 fails at the crossing
  (a model gap, not a tolerance). Documented OCCT-fallthrough; OCCT ref `Rc=3,L=20,r=0.5`
  COMMON → `143.179260`, Δ `−0.820740`.

Only T1 tasks below are checked as DONE; T2/T3 tasks are marked DECLINED with the blocker.


Verification levels: **host** = OCCT-free host CTest (seams/spines on their surfaces +
watertight + correct SHRINK volume + the right continuity — chamfer C0 at the per-seam
bevel angle, fillet G1 at both contact curves); **sim** = native-vs-OCCT
`BRepFilletAPI_MakeChamfer` (`Add(d1, d2, edge, face)`) / `BRepFilletAPI_MakeFillet`
parity on a booted simulator (volume / area / watertight / continuity within the bound).
`cc_fillet_edges` / `cc_chamfer_edges` keep their signatures; T1 adds the ADDITIVE
`cc_chamfer_edges_asym`; T2/T3 land behind the unchanged `cc_fillet_edges`.

> Mechanism note: T1 generalizes `src/native/blend/curved_chamfer.h` (`buildChamferedCylinder`
> → `buildChamferedCylinderAsym(g, d1, d2, defl)`, symmetric = `d1 = d2`) — an OBLIQUE cone
> frustum between the setback circles `(Rc, H − s·d1)` and `(Rc − d2, H)`, C0 at the two
> DIFFERENT bevel angles, removed volume `π·d1·d2·(Rc − d2/3)`. T2 adds
> `src/native/blend/elliptical_fillet.h` — the cylinder↔oblique-plane ELLIPSE crease (SSI S1
> `plane_conics`), a closed-form ellipse spine + two contact ellipses, and a swept `r`-circle
> canal (G1 at both contacts), NUMSCI-OFF. T3 adds `src/native/blend/cylcyl_fillet.h`
> (NUMSCI-gated) — the cyl↔cyl marching crease + spine + swept canal — RETAINED only if the
> narrow slice self-verifies, else an honest decline (no dead code). All welded via the
> boolean `assembleSolid`; `native_blend.h` gains the new `#include`s.

## T1 — Asymmetric two-distance chamfer (highest confidence)

### 1. Additive facade + engine entry
- [x] 1.1 Add `CCShapeId cc_chamfer_edges_asym(CCShapeId body, const int* edgeIds, int
  edgeCount, double distance1, double distance2)` to `include/cybercadkernel/cc_kernel.h`
  and `src/facade/cc_kernel.cpp`, mirroring `cc_chamfer_edges` (same registry lookup +
  exception-to-status guard). The existing `cc_chamfer_edges` is BYTE-UNCHANGED. (**host**)
- [x] 1.2 Add `IEngine::chamfer_edges_asym(EngineShape, const int*, int, double d1, double
  d2)` (default `engine_unsupported("chamfer_edges_asym")`); OCCT override →
  `BRepFilletAPI_MakeChamfer` + `Add(d1, d2, edge, face)` (`face` = the cylinder wall, so
  `d1` is the axial wall setback); `NativeEngine` override → native T1. (**host** + **sim**)

### 2. Oblique cone-frustum builder (C0, two bevel angles)
- [x] 2.1 Promote `detail::buildChamferedCylinder(g, d, defl)` to
  `buildChamferedCylinderAsym(g, d1, d2, defl)`: cylinder seam radius `Rc` at axial
  `H − s·d1`; cap seam radius `Rc − d2` at `H`; the bevel band `radius(τ) = Rc − d2·τ`,
  `axial(τ) = (H − s·d1) + s·d1·τ`, `τ ∈ [0,1]` — an OBLIQUE frustum, ONE meridian step.
  Keep the symmetric entry as `buildChamferedCylinderAsym(g, d, d, defl)`. (**host**)
- [x] 2.2 Bevel normal `n_frustum(u) = radial(u)·d1 + axial·(s·d2)` (normalized); tile the
  band into `N` angular quads (`sagittaSteps(Rc, 2π, defl)`), each → 2 exactly-planar
  triangles; wall / frustum / caps share the SAME `N` samples ⇒ coincident seam vertices.
  (**host**)
- [x] 2.3 `curved_chamfer_edge_asym(solid, edgeIds, edgeCount, d1, d2, deflection)` reuses
  `detail::facesOnRim` + `cylinderInfo` + `rimGeom` (convex circular cylinder↔coaxial-cap
  rim); guards `edgeCount == 1`, `d1 > kBlendEps`, `d2 > kBlendEps`, `Rc − d2 > eps`, wall
  covers `H − s·d1`. Not that config / any degeneracy ⇒ NULL → OCCT. (**host**)

### 3. Self-verify (C0 at the two DIFFERENT bevel angles) + volume
- [x] 3.1 Cylinder seam ON the wall (`radius = Rc`), cap seam ON the cap (`axial = H`) by
  construction; the frustum normal makes `cos = d1/√(d1²+d2²)` with the cylinder radial
  normal at `τ = 0` and `cos = d2/√(d1²+d2²)` with the cap axial normal at `τ = 1`, both
  explicitly `≠ 1` (C0, NOT G1). (**host**)
- [x] 3.2 `NativeEngine::chamfer_edges_asym` accepts ONLY through
  `blendResultVerified(result, body, wantGrow=false)` (watertight + `0 < Vr < Vo`); removed
  volume `π·d1·d2·(Rc − d2/3)`; `d1 = d2` reproduces the symmetric `π·d²·(Rc − d/3)`. NULL /
  unverified ⇒ honest error → OCCT. (**host**)

## T2 — Non-circular (elliptical) crease fillet (medium confidence, narrow slice)

### 4. Elliptical-crease classification (cylinder ∩ oblique plane)
- [~] 4.1 `elliptical_fillet_edge` recognizes a CIRCULAR-cylinder lateral face + an OBLIQUE
  `Plane` face meeting at an `Ellipse` `EdgeCurve`; obtain the crease ellipse (centre, axes,
  semi-minor `Rc`, semi-major `Rc/sinθ`) from the SSI S1
  `intersectPlaneCylinder(plane, cylinder)`. Not an oblique-plane ellipse (axis-⟂ circle /
  axis-∥ lines / non-ellipse / concave / freeform) ⇒ NULL → OCCT. (**host**)
- [~] 4.2 Decline guard: `r < ρ_min = Rc·sinθ` (with a scale margin — else the canal
  self-intersects on the concave side), `θ` bounded away from `0` and `90°`, `edgeCount == 1`.
  Any failure ⇒ NULL → OCCT. (**host**)

### 5. Ellipse spine + contact ellipses (closed form, solver-free, NUMSCI-OFF)
- [~] 5.1 Spine `S` = `intersectPlaneCylinder(plane shifted `r` along the outward normal,
  cylinder radius `Rc − r`)` — a plane∩cylinder ELLIPSE (semi-minor `Rc − r`). (**host**)
- [~] 5.2 Cylinder-contact ellipse `C_cyl` (foot on the cylinder, radius `Rc`) and
  plane-contact ellipse `C_pl` (`= S − r·n_out`, on the plane) — closed form, ON their faces
  by construction. (**host** + **sim**)

### 6. Swept `r`-circle canal + G1 self-verify + weld
- [~] 6.1 For `N` spine stations (`sagittaSteps` on the spine ellipse arc), build the
  characteristic `r`-circle in the plane NORMAL to the spine tangent `T = S'(t)`; emit the
  `r`-arc from the cylinder-contact foot to the plane-contact foot (`M` meridian steps),
  tiled into planar triangles sharing `N` stations with the rebuilt wall + trimmed oblique
  cap. (**host**)
- [~] 6.2 G1 self-verify: the canal normal at the cylinder-contact foot equals the cylinder
  radial normal (`cos = 1`) and at the plane-contact foot equals the plane normal
  (`cos = 1`); the two contact ellipses lie on their faces (radius `Rc`; on the plane). Any
  station failing ⇒ NULL. Weld via `assembleSolid`; watertight (0 boundary edges). (**host**)
- [~] 6.3 Wire into `NativeEngine::fillet_edges` AFTER the circular convex/concave
  candidates: `nblend::elliptical_fillet_edge(h->shape, e, ec, r)` accepted ONLY through
  `blendResultVerified(..., wantGrow=false)` (SHRINK — a convex fillet removes material). The
  circular candidates stay byte-identical (tried first). (**host**)

## T3 — Cylinder↔cylinder canal fillet (narrow slice OR honest decline — no dead code)

### 7. Marching crease + spine (NUMSCI-gated)
- [~] 7.1 `cylcyl_fillet_edge` (under `CYBERCAD_HAS_NUMSCI`) recognizes the CURVED↔CURVED
  crease between two `Cylinder` faces; obtain the crease curve from the SSI marching
  `trace_intersection(cyl1, cyl2)` (transversal, `Closed`). Restrict to the ROBUST slice:
  equal radii `Rc1 = Rc2`, PERPENDICULAR axes, `r` safely below the crease min curvature.
  Outside the slice / no NUMSCI / non-closing trace ⇒ NULL → OCCT. (**host**, NUMSCI-ON)
- [~] 7.2 Spine `S` = marching `trace_intersection(offset-cyl1 (Rc − r), offset-cyl2
  (Rc − r))`; fit to a B-spline `WLine`; the two cylinder-contact curves are the feet on
  `cyl1` / `cyl2`. (**host**, NUMSCI-ON)

### 8. Swept canal + G1 + the honest gate
- [~] 8.1 Sweep the `r`-circle canal in the spine-normal planes (as T2, on a general spine),
  trim between the two cylinder-contact curves, weld the saddle sub-regions via
  `assembleSolid`. G1 self-verify: the canal normal at each cylinder-contact foot equals that
  cylinder's radial normal (`cos = 1`). (**host**, NUMSCI-ON)
- [~] 8.2 HONEST GATE: wire into `NativeEngine::fillet_edges` (NUMSCI-guarded) under
  `blendResultVerified(..., wantGrow=false)` ONLY IF the narrow slice self-verifies
  watertight + G1 + SHRINK + OCCT parity on its fixture. If it does NOT build robustly
  (saddle weld fails / envelope self-intersects / trace does not close), DO NOT retain an
  always-NULL builder — cyl↔cyl fillet stays a documented OCCT-fallthrough, the measured gap
  REPORTED. No dead code. (**host** + **sim**)

## 9. Engine dispatch + out-of-slice defers
- [ ] 9.1 `NativeEngine::fillet_edges` = planar → curved-convex → curved-concave →
  elliptical (T2) → [cyl↔cyl (T3) iff landed] → honest error; each candidate under the SAME
  `blendResultVerified(wantGrow=false)` SHRINK guard. `chamfer_edges` (symmetric) UNCHANGED;
  new `chamfer_edges_asym` override for T1. (**host**)
- [ ] 9.2 Out-of-slice inputs return NULL (asserted in `*_scope_defers` tests): T1 —
  concave / cyl↔cyl / non-circular / tilted / freeform / `Rc ≤ d2` / wall < `d1` / multi-edge
  chamfer; T2 — axis-⟂ (circle) / axis-∥ (lines) / non-ellipse / concave / freeform /
  `r ≥ Rc·sinθ`; T3 — unequal-radius / non-orthogonal / branched crease / no-NUMSCI. (**host**
  + **sim**)

## 10. Verification (two gates)
- [ ] 10.1 Host suite (no OCCT), `test_native_blend.cpp` adds: **T1**
  `asym_chamfer_oblique_frustum_watertight_volume` (watertight + volume `|body| − π·d1·d2·
  (Rc − d2/3)`, rel ≤ 6e-3), `asym_chamfer_two_bevel_angles_c0` (`cos = d1/√(d1²+d2²)` at the
  wall, `d2/√(d1²+d2²)` at the cap, both `≠ 1`), `asym_chamfer_symmetric_special_case`
  (`d1 = d2` → the symmetric removed volume), `asym_chamfer_scope_defers`. **T2**
  `elliptical_fillet_oblique_plane_watertight_shrink` (spine + contact ellipses on-surface +
  watertight + SHRINK), `elliptical_fillet_g1_both_contacts` (`cos = 1` at both),
  `elliptical_fillet_curvature_bound_defers` (`r ≥ Rc·sinθ` → NULL), `elliptical_fillet_
  scope_defers`. **T3** (NUMSCI-ON) `cylcyl_fillet_narrow_slice_watertight_g1` OR
  `cylcyl_fillet_declines` (documented). Built default + NUMSCI-ON. (**host**)
- [ ] 10.2 Sim parity: extend `scripts/run-sim-native-curved-chamfer.sh` +
  `tests/sim/native_curved_chamfer_parity.mm` with T1 (`cc_chamfer_edges_asym(d1, d2)` vs
  OCCT `BRepFilletAPI_MakeChamfer::Add(d1, d2, edge, face)`, ≥ 2 `d1 ≠ d2` fixtures, TIGHT
  bound); extend `scripts/run-sim-native-curved-fillet.sh` +
  `tests/sim/native_curved_fillet_parity.mm` with T2 (`cc_fillet_edges(r)` vs
  `BRepFilletAPI_MakeFillet` on the oblique-plane ellipse rim, ≥ 1 fixture) and T3 (the
  cyl↔cyl rim iff landed, else decline-parity fall-through). HARD native gates: watertight,
  mesh↔B-rep vol within bound, `shrank = 1`, the right continuity. A fixture beyond tol →
  out of slice (NULL → OCCT), gap REPORTED. (**sim**)
- [ ] 10.3 No regression: `run-sim-native-curved-chamfer.sh` (9/9) +
  `run-sim-native-curved-fillet.sh` (23/23) + `run-sim-native-blend.sh` (16/16) stay green;
  native booleans + SSI + curved-boolean + healing + import host suites green under NUMSCI +
  default; `run-sim-suite.sh` unchanged count. (**sim** + **host**)
- [ ] 10.4 `openspec validate add-native-fillet-chamfer-breadth --strict` green. (**host**)

## Deferred (NOT in this batch — honest NULL → OCCT, gap REPORTED)

- [ ] **Asymmetric chamfer on a NON-circular / concave / cyl↔cyl rim** → OCCT (T1 is the
  convex circular cylinder↔cap rim only).
- [ ] **Non-circular fillet beyond the oblique-plane ELLIPSE** (cone↔plane, sphere rims,
  spline creases, concave elliptical rim, `r ≥ Rc·sinθ`) → OCCT.
- [ ] **Cyl↔cyl fillet beyond the equal-radius orthogonal slice** (unequal radii,
  non-orthogonal axes, branched / self-intersecting crease) → OCCT — and, if the narrow
  slice itself is not robustly buildable, the WHOLE T3 track is an honest decline (no dead
  code), the measured gap REPORTED.
- [ ] **Any freeform (NURBS/Bézier/B-spline) adjacent face** → OCCT.
