# Tasks — moat-gs-measure-curvature (MOAT M-GS, GS3 + GS4)

Order: substrate build → GS4 curvature (simplest) → GS3 angle → GS3 distance →
additive facade → host analytic gate → sim native-vs-OCCT gate → docs, or HONEST
DECLINE. All new native code is header-only, OCCT-free, host-buildable (`clang++
-std=c++20`), namespace `cybercad::native::analysis`. `cc_*` additive-only; no
existing signature changes. No tolerance weakened; a measured decline (freeform-
trimmed minimizer / parametric singularity) is a first-class outcome.

**STATUS (implement phase):** GS3+GS4 landed (`src/native/analysis/`, header-only,
OCCT-free) behind the additive `cc_measure_*` / `cc_*_curvature` facade. GATE A
(host, no OCCT) green: `test_native_analysis` 47/0 + `test_native_analysis_facade`
5/0. GATE B (sim, native-vs-OCCT) green: `native_analysis_parity` 21/0 vs
`GeomLProp_SLProps`/`GeomLProp_CLProps`/`BRepExtrema_DistShapeShape` on identical
geometry. 7.2 (cognitive-complexity pass — DONE: 15 fns, avg 3.6, worst
`distSegSeg`=11 🟡, all within the systems/frontend bands) and 7.3 (roadmap doc —
DONE: GS3/GS4 marked NATIVE landed with declines documented) are complete.
Deliberately DECLINED / deferred as honest, first-class outcomes: 3.3 (constrained
boundary restart — curved-face freeform distance is DECLINED at the facade instead
of returning a non-certifiable guess), 4.3 (dedicated ABI byte-diff test —
additive-only proven by `git diff`: 0 removed lines, only the 4 new prototypes
appended), 6.3 (declines are asserted in GATE A host, not re-run on the sim).

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR` to `build-numsci/host` (host gate) / `/iossim`
      (sim gate).
- [x] 0.2 Confirm the consumed evaluators build and are on the OCCT-verified path:
      `math::surfaceDerivs`/`nurbsSurfaceDerivs` (maxDeriv=2), `curveDerivs`/
      `nurbsCurveDerivs`, `numerics::polishCurveParam`/`polishSurfaceParam`,
      `math::elementary`/`torus` value+normal.

## 1. GS4 curvature (`src/native/analysis/curvature.h`, OCCT-free)

- [x] 1.1 `surfaceCurvature(FaceSurface, u, v) → {K,H,k1,k2} | decline`: analytic arm
      (plane 0; sphere `1/R²`,`1/R`; cylinder `0`,`1/(2R)`,`{1/R,0}`; cone; torus
      known formulae) + NURBS arm via first/second fundamental forms from
      `(nurbs)surfaceDerivs` maxDeriv=2.
- [x] 1.2 DECLINE guard: `EG−F² ≤ ε·max(E,G)²` (parametric singularity) → decline;
      cone within `ε` of apex → decline.
- [x] 1.3 `edgeCurvature(EdgeCurve, t) → κ`: line 0, circle `1/R`, ellipse closed
      form, NURBS `‖C′×C″‖/‖C′‖³` via `(nurbs)curveDerivs` maxDeriv=2; `‖C′‖≤ε` →
      decline.
- [x] 1.4 `k1≥k2` ordering + `H` sign under the outward-normal convention.

## 2. GS3 angle (`src/native/analysis/angle.h`, OCCT-free)

- [x] 2.1 `angle(A,B) → θ | decline`: line·line `acos(|d_a·d_b|)`∈[0,π/2];
      plane·plane `acos(clamp(n_a·n_b))`∈[0,π]; line·plane `asin(|d·n|)`∈[0,π/2].
- [x] 2.2 DECLINE for any non-line/non-plane entity or degenerate direction.

## 3. GS3 distance (`src/native/analysis/distance.h`, OCCT-free)

- [x] 3.1 Closed-form analytic·analytic cells: point·{point,line/segment,circle,
      plane,cylinder,sphere}; line·line (parallel + skew, feet clamped to ranges).
- [x] 3.2 Seed-and-refine for any NURBS curve/surface: deterministic coarse sample
      (degree×span grid) → per-seed project onto the other entity → alternate
      `numerics/closest_point` Newton polish to a converged witness pair; return
      global min + both closest points.
- [ ] 3.3 Trim / range awareness: face witness inside `UVRegion::inside`; edge
      witness in param range; constrained boundary restart when the unconstrained
      optimum is outside the trim.
- [x] 3.4 DECLINE: genuinely-trimmed freeform patch whose global optimum is not
      certifiable (multiple comparably-deep basins / non-converging boundary
      restart) → return decline, never a guessed minimum.

## 4. Additive `cc_*` facade (`include/cybercadkernel/cc_kernel.h` + `src/engine`)

- [x] 4.1 Add prototypes: `cc_measure_distance` (→ `out7 = [d,p1,p2]`),
      `cc_measure_angle` (→ radians), `cc_surface_curvature` (→ `out4 = [K,H,k1,k2]`),
      `cc_edge_curvature` (→ κ); return `1` on success / `0` on decline +
      `cc_last_error`.
- [x] 4.2 Engine dispatch resolves `(subKind,subId)` to the native `EdgeCurve`/
      `FaceSurface`/vertex, calls `analysis::*`, marshals out-arrays; flips
      curvature sign for a `Reversed` face.
- [ ] 4.3 ABI contract test (`CC_KERNEL_NO_PROTOTYPES`): every pre-existing struct
      and signature byte-identical; only the four new prototypes appear.

## 5. GATE A — HOST ANALYTIC (no OCCT, closed-form)

- [x] 5.1 Distance: point·line, point·circle, point·plane, line·line (parallel +
      skew) vs hand-derived closed forms, `1e-9`.
- [x] 5.2 Curvature: sphere `K=1/R²`, cylinder `K=0 ∧ H=1/(2R)`, plane `0`, torus
      `K=cos v/(r(R+r cos v))`, edge circle `κ=1/R`, `1e-9`.
- [x] 5.3 Angle: line·line, plane·plane, line·plane vs closed form.
- [x] 5.4 Decline: parametric-singularity curvature + non-line/plane angle assert a
      clean decline (not a number).

## 6. GATE B — SIM native-vs-OCCT (booted iOS simulator)

- [x] 6.1 `cc_measure_distance` vs `BRepExtrema_DistShapeShape` (min distance +
      witness points) over analytic + simple-NURBS pairs, scale-relative tol.
- [x] 6.2 `cc_surface_curvature` vs `BRepLProp_SLProps` (Gaussian/mean/principal) and
      `cc_edge_curvature` vs `GeomLProp` over analytic + simple-NURBS fixtures.
- [ ] 6.3 Declined fixtures (trimmed-freeform distance / singular curvature) assert a
      clean decline, not a compared number.

## 7. Zero-regression + docs

- [x] 7.1 `src/native/**` stays OCCT-free (0 OCCT includes — grep gate); existing
      `cc_*` unchanged; full native + host suites green vs `main`.
- [x] 7.2 Cognitive complexity: distance seed loop ≤ 25 (systems band, flagged);
      curvature/angle helpers ≤ 12. Measured: 15 fns, avg 3.6, worst `distSegSeg`=11
      (🟡, within band); all curvature/angle helpers ≤ 7.
- [x] 7.3 Update `openspec/MOAT-ROADMAP.md` GS3/GS4 status; document the declined
      cases (freeform-trimmed minimizer, parametric singularities) as expected, not
      faked.
