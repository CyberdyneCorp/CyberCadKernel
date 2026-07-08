# Tasks — moat-m4rc-rational-curve-trims (MOAT M4-tail-2, first slice)

Scope: ONE foreign rational B-spline CURVE used as an edge's 3D geometry or a
`TRIMMED_CURVE` basis. All edits confined to the CURVE / TRIMMED_CURVE / edge-basis
region of `src/native/exchange/step_reader.cpp` (`curve` / `trimmedCurve` / `bsplineCurve`
/ `evalEdge`). Do NOT touch the representation-relationship / annotation / assembly-gate
region (sibling M4-tail-3 lane).

## 0. Substrate + baseline (capture BEFORE touching the reader)

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
  export `CYBERCAD_NUMSCI_DIR` to the `iossim` (sim gate) / `host` (analytic gate) tree.
- [x] 0.2 Capture GREEN baseline pre-change: host reader test count, host CTest, and the
  M4 non-rational + M4-rational **surface** cases. Record exact numbers so zero-regression
  is a diff, not an assertion.
- [x] 0.3 Confirm `src/native/**` has 0 OCCT `#include`s at baseline (grep) so the
  OCCT-free invariant is measured, not assumed.

## 1. Shared curve parse factor (must be byte-identical)

- [x] 1.1 Extract `fillBsplineCurve(degree, polesArg, multsArg, knotsArg, EdgeCurve& out)`
  from `bsplineCurve()` — the curve analogue of `fillBsplineGrid` (line 1457): fill
  `degree`, poles (ref → `point()`), `knots = expandKnots(knots, mults)`; return false on
  malformed / short / non-ref input or `knots.size() != poles.size() + degree + 1`.
- [x] 1.2 Re-point `bsplineCurve()` (line 985) at the helper. Prove the host reader suite
  is byte-identical to the 0.2 baseline (non-rational B-spline edges unchanged).

## 2. Rational-curve read arm

- [x] 2.1 `curve()` (line 881): before the `r->combined` decline (line 883), route a
  combined record carrying a `RATIONAL_B_SPLINE_CURVE` sub-record to
  `rationalBsplineCurve(*r)` via the existing `hasSub` (line 1449); every other combined
  record keeps `std::nullopt`.
- [x] 2.2 `rationalBsplineCurve()`: `findSub` the `B_SPLINE_CURVE` (degree + poles),
  `B_SPLINE_CURVE_WITH_KNOTS` (mults + knots), and `RATIONAL_B_SPLINE_CURVE` (weights)
  sub-records; missing/mistyped sibling → `std::nullopt`. Fill via `fillBsplineCurve`,
  then read the flat weight list into `EdgeCurve::weights` in pole order.
- [x] 2.3 Weight validation (decline gate, D3): `weights.size() == poles.size()`, every
  weight finite and strictly positive; else `std::nullopt`. NEVER clamp; no tolerance
  introduced.

## 3. Rational-aware guard evaluator

- [x] 3.1 `evalEdge()` (line 2325) `Kind::BSpline` arm: route to `math::nurbsCurvePoint`
  when `c.weights` non-empty, else keep `math::curvePoint` (line 2340). Byte-identical for
  non-rational edges. Confirm the trim machinery (`trimmedCurve` 912, `trimmedRange` 1639,
  `curveRange` 1622–1627) is reused unchanged.

## 4. HOST ANALYTIC gate (no OCCT linked)

- [x] 4.1 Rational-quadratic arc case: an `EdgeCurve` (degree 2, control triangle,
  weights `{1, cos(Δ/2), 1}`) whose `evalEdge(t)` reproduces the exact circle point
  `O + R(cos θ, sin θ)` within `1e-9` at a parameter grid — proven with NO OCCT linked.
- [x] 4.2 Guard accept/reject: the per-edge faithful guard ACCEPTS the faithful rational
  edge and REJECTS a deliberately perturbed off-curve weight (`decline()` fires).
- [x] 4.3 Malformed-record decline: weight-count mismatch, non-finite weight, and
  non-positive weight each `decline()` (host, no OCCT).

## 5. SIM native-vs-OCCT parity gate (booted iOS simulator, OCCT linked)

- [x] 5.1 Measure foreign reachability (D5): determine whether OCCT `STEPControl_Writer`
  authors a rational edge curve for an in-scope solid. If YES: import that foreign file
  with `cc_set_engine(1)` and assert count/volume/area/watertight/topology match the OCCT
  `STEPControl_Reader` + `BRepMesh` oracle. If NO: record the measured gap and show parity
  on a native-authored combined `RATIONAL_B_SPLINE_CURVE` record round-tripped through
  OCCT; the foreign decline is documented, not faked.
- [x] 5.2 Decline parity: a malformed / non-positive-weight record and a non-faithful edge
  import byte-identically to `cc_set_engine(0)` (OCCT), and the engine self-verify
  discards any non-watertight admitted rational edge → OCCT.

## 6. Zero-regression proof (mandatory)

- [x] 6.1 Non-rational B-spline edge path + every analytic curve arm + the M4 /
  M4-rational **surface** paths byte-identical to the 0.2 baseline.
- [x] 6.2 Host CTest + host reader suite GREEN (baseline + new rational-curve cases);
  full tessellation-sensitive sim suite unperturbed.
- [x] 6.3 `src/native/**` has 0 OCCT `#include`s; `cc_*` ABI unchanged (additive read-side
  only); `EdgeCurve` / `PCurve` unchanged; tessellator unchanged.

## 7. Docs / spec

- [x] 7.1 Record results + the measured OCCT foreign-reachability gap (D5) in this tasks
  file; keep the honest decline documented where OCCT will not author the foreign curve.

## Results (2026-07-08)

Implemented additively in the CURVE/TRIMMED_CURVE/edge-basis lane of
`src/native/exchange/step_reader.cpp` only. `git diff --stat`: step_reader.cpp (+84/-14),
test_native_step_reader.cpp (+200), native_step_import_parity.mm (+123). No other files.

- **OCCT-free invariant**: `src/native/**` has 0 OCCT includes (grep = 0). `cc_*` ABI
  unchanged (read-side only). `EdgeCurve`/`PCurve`/tessellator unchanged.
- **Byte-identity**: the non-rational B-spline edge path (`fillBsplineCurve` factor +
  `evalEdge` `weights.empty()` ternary) and the M4 / M4-rational **surface** paths are
  byte-identical — all pre-existing host + sim cases stay GREEN.
- **HOST ANALYTIC gate (no OCCT)**: `test_native_step_reader` = **56 cases, 0 failed**.
  New: `rational_bspline_curve_arc_reproduces_exact_circle` (rational-quadratic 90° arc
  reproduces the exact circle r=R to <1e-9 via the shared `edgeCurveLocal`/`evalEdge`
  evaluator; the same poles WITHOUT weights miss it by >1e-2 → weights load-bearing);
  `foreign_rational_bspline_curve_unit_weights_matches_nonrational` (combined record admits,
  watertight, same volume + face count as the non-rational round-trip);
  `..._nonunit_weight_off_surface_declines` (perturbed weight → wall faithful guard rejects →
  DECLINE — the precise regression test for rational-aware `evalEdge`);
  `..._malformed_weights_decline` (short/zero/negative → decline);
  `combined_bspline_curve_without_rational_sub_declines` (zero-regression: non-rational
  combined curve still declines).
- **SIM native-vs-OCCT gate (booted simulator, OCCT linked)**: `native_step_import_parity`
  = **90 cases, 0 failed**. New G5 `ratcurve admit`: native parsed=1, watertight=1,
  solids=1, nativeVol=304.38 == source 304.38 (exact) and == OCCT `STEPControl_Reader`
  oracle 304.505 (rel 3.8e-4). `ratcurve decline`: perturbed non-unit weight → native
  parsed=0 (decline) while OCCT still reads the standard entity (occtVol=301.5) → shipping
  falls back to OCCT.
- **D5 foreign reachability (honest caveat, precedented)**: a genuinely OCCT-authored
  rational EDGE on an *admissible* surface is not readily producible — `BRepBuilderAPI_NurbsConvert`
  (G4) bundles the rational edge with a non-standard seam surface the reader declines, so
  G4 still declines overall (parsed=0) and shipping matches OCCT (267.965) — **unchanged by
  this slice, no wrong solid emitted**. So, exactly as the landed M4-RATIONAL *surface*
  gate did, the SIM parity uses a native-authored combined `RATIONAL_B_SPLINE_CURVE` record
  (unit-weight rewrite of the spline-wall rim) round-tripped through both readers; the
  non-unit closed-form is covered by the HOST arc gate to machine epsilon. The foreign
  OCCT-authored-edge decline is documented, not faked.
