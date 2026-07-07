# Design — moat-m4r-rational-bspline-step

## Context

M4 admits a foreign non-rational `B_SPLINE_SURFACE_WITH_KNOTS` face. The reader
already:

- parses **combined** Part-21 records `( SUB(..) SUB(..) .. )` into
  `Record::subs` (tokenizer, `readRecord()`), and scans sub-records by keyword in
  several places (e.g. `relationshipAndTransform()` for assembly relationships);
- carries `FaceSurface::weights` (`empty ⇒ non-rational`) through to the mesher;
- evaluates the faithful-reconstruction guard through `bsplineSurfaceValue()`,
  which is **already rational-aware** — it calls `math::nurbsSurfacePoint(...,
  {weights.data(), weights.size()}, ...)` when `weights.size() == poles.size()`;
- meshes a trimmed rational `Kind::BSpline` face via the M0 trimmed-freeform branch,
  which evaluates vertices with `math::nurbsSurfacePoint` when `weights` is non-empty.

The only missing link is the **read** of surface weights. A rational surface never
arrives as the bare `B_SPLINE_SURFACE_WITH_KNOTS` keyword; OCCT emits it as a
combined instance, which `surface()` rejects at `if (!r || r->combined) return
std::nullopt;`.

## Goals / Non-Goals

**Goals**
- Parse the combined `RATIONAL_B_SPLINE_SURFACE` record and populate
  `FaceSurface::weights` for ONE foreign rational B-spline surface face.
- Keep the non-rational keyword path and every downstream stage byte-identical.
- Honest decline on any malformed / mismatched / non-positive-weight rational
  record or non-faithful boundary.

**Non-Goals**
- No general NURBS importer, no rational-curve read, no periodic/seamed rational
  surface close, no multi-patch rational assemblies.
- No tessellator change; no `cc_*` ABI change; no OCCT in `src/native/**`.

## Decisions

### D1 — Dispatch on the combined record, reuse existing sub-record machinery

`surface()` currently early-returns on `r->combined`. Add, BEFORE that guard, a
check: if the combined instance carries a `RATIONAL_B_SPLINE_SURFACE` sub-record
(and the required `B_SPLINE_SURFACE` + `B_SPLINE_SURFACE_WITH_KNOTS` siblings),
dispatch to `rationalBsplineSurface(*r)`. Any other combined surface record keeps
the `nullopt` decline. This mirrors the sub-record scan already used by
`relationshipAndTransform()`, so no tokenizer change and no new record type.

```
std::optional<topo::FaceSurface> surface(long id) {
  const Record* r = rec(id);
  if (!r) return std::nullopt;
  if (r->combined) {
    if (hasSub(*r, "RATIONAL_B_SPLINE_SURFACE")) return rationalBsplineSurface(*r);
    return std::nullopt;                 // any other combined surface: decline
  }
  ... existing keyword dispatch (byte-identical) ...
}
```

### D2 — Field layout of the combined rational record

In the combined form the fields are **split across sub-records** (unlike the single
`B_SPLINE_SURFACE_WITH_KNOTS` keyword whose args are all in one record):

- `B_SPLINE_SURFACE(degU, degV, ((#pole)…), form, uClosed, vClosed, selfInt)` —
  degrees, the row-major pole grid (U outer), and the closure flags.
- `B_SPLINE_SURFACE_WITH_KNOTS((uMults), (vMults), (uKnots), (vKnots), spec)` —
  the RLE knot data only (expanded by the existing `expandKnots`).
- `RATIONAL_B_SPLINE_SURFACE(((w)…))` — the weight grid, SAME shape / order as the
  pole grid.

`rationalBsplineSurface()` reads degrees + poles from the first, knots from the
second, weights from the third. To avoid duplicating the pole/knot parse (and to
keep `bsplineSurface()` byte-identical), factor a shared helper
`fillBsplineGrid(degU, degV, polesArg, uMults, vMults, uKnots, vKnots, out&)` that
both the keyword path and the rational path call with the args pulled from their
respective (single or split) records.

### D3 — Weight-grid validation (the decline gate)

`FaceSurface::weights` is filled ONLY when the weight grid is well-formed:

- outer length `== nPolesU`, every inner row length `== nPolesV` (non-ragged);
- total count `== poles.size()`;
- every weight finite and **strictly positive** (a zero/negative weight is not a
  valid NURBS surface → decline, never clamped).

Row-major order matches the poles so `weights[i*nPolesV + j]` pairs with
`poles[i*nPolesV + j]`, exactly what `math::nurbsSurfacePoint` expects. Any failure
→ `std::nullopt` (decline → OCCT). No tolerance is introduced or weakened here; this
is a structural well-formedness check.

### D4 — Reuse the faithful guard and mesher unchanged

Once `weights` is populated, the existing per-edge `bsplinePCurveFor` pcurve arm and
the `S_face(pcurve(t)) = C_edge(t)` guard run unmodified. Because
`bsplineSurfaceValue()` already switches to `math::nurbsSurfacePoint` when `weights`
is present, the guard measures the gap on the TRUE rational surface. An admitted face
flows into the landed M0 trimmed-freeform mesher (rational eval) and then the engine's
mandatory watertight + volume/area self-verify. A non-watertight / off-volume native
result is DISCARDED → OCCT.

### D5 — Non-rational surfaces whose weights are all 1

A rational record whose weights are all exactly `1.0` denotes a non-rational surface.
We still store the (all-ones) `weights`; `math::nurbsSurfacePoint` with unit weights
equals the non-rational `surfacePoint`, so the geometry and guard are identical. We do
NOT special-case "all ones" — storing them is simplest, correct, and avoids a hidden
branch. (The non-rational **keyword** path continues to store no weights and take the
polynomial evaluator, byte-identical.)

## Verification strategy (the two gates)

- **HOST ANALYTIC (no OCCT linked).** Build a rational B-spline patch whose exact
  geometry is closed-form — a rational-quadratic patch that reproduces an exact
  cylindrical/spherical section (weights `{1, 1/√2, 1, …}` for the 90° arc). Assert
  (1) `surfaceValue` reproduces the closed-form surface point within `1e-9` for a grid
  of `(u,v)`; (2) the guard ACCEPTS the faithful trimmed face and REJECTS a perturbed
  off-surface variant; (3) the meshed solid is watertight with the closed-form enclosed
  volume / area within the deflection tolerance. This proves the parsed rational surface
  reproduces an independent value with NO OCCT symbol linked.
- **SIM native-vs-OCCT (booted iOS simulator, OCCT linked).** A FOREIGN OCCT-authored
  solid whose face is a rational B-spline surface (e.g. an OCCT NURBS-converted cylinder
  / sphere / trimmed patch written by `STEPControl_Writer` as the combined
  `RATIONAL_B_SPLINE_SURFACE` record) imports NATIVELY under `cc_set_engine(1)` and
  matches `STEPControl_Reader` + `BRepMesh_IncrementalMesh` on solid count, volume,
  surface area, watertightness, and sub-shape topology. A malformed / non-faithful
  rational patch DECLINES natively and imports via OCCT identical to `cc_set_engine(0)`.
  The parity test restores the OCCT default in teardown and carries its own `main()` (on
  the `run-sim-suite.sh` SKIP list) so the suite assertion count is unchanged.

## Risks / honest-decline triggers

- **No genuinely-foreign fixture reachable.** If no non-fabricated OCCT-authored
  rational-surface STEP fixture can be produced (OCCT may emit analytic surfaces for
  primitives), the sim parity gate is closed only against a real rational patch — else
  the slice DECLINES honestly with the measured gap, and the host analytic gate stands
  as the closed-form proof.
- **Guard rejects the reconstructed pcurve on curvature the M0 mesher cannot close
  watertight.** Then the face declines → OCCT (first-class outcome); the measured gap
  is reported.
- If a clean read-side arm that keeps the non-rational path byte-identical AND meshes
  the rational face watertight cannot be achieved, the arm is reverted and the rational
  surface keeps the honest OCCT decline (an OCCT-imported patch loses nothing).

## Cognitive complexity

`surface()` gains one combined-record branch; `rationalBsplineSurface()` delegates to
`hasSub`, `findSub`, `fillBsplineGrid`, and a small weight-grid reader — each a
straight-line helper, keeping every function within the parser band.
