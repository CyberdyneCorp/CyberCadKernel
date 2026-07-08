# Proposal — moat-m4rc-rational-curve-trims (MOAT M4-tail-2, first slice)

## Why

M4-rational (`moat-m4r`) landed native STEP admission of a foreign **rational**
B-spline **surface** face: `surface()` routes the combined Part-21 record carrying a
`RATIONAL_B_SPLINE_SURFACE` sub-record to `rationalBsplineSurface()`, which reads the
surface weights into `FaceSurface::weights` and — guarded by the rational-aware
per-edge `S_face(pcurve(t)) = C_edge(t)` reconstruction guard — hands the trimmed
`Kind::BSpline` face to the landed M0 mesher, verified watertight vs OCCT `BRepMesh`.

But the **curve/edge** basis path was left non-rational. A foreign **rational B-spline
CURVE** used as an edge's 3D geometry or as a `TRIMMED_CURVE` basis still DECLINES to
OCCT today. Three concrete facts in `src/native/exchange/step_reader.cpp`:

- `curve()` (line 881) rejects EVERY **combined** Part-21 record at its first guard —
  `if (!r || r->combined) { return std::nullopt; }` (line 883). OCCT `STEPControl_Writer`
  emits a rational curve as a combined instance
  `( BOUNDED_CURVE() B_SPLINE_CURVE(deg, (#pole…), form, closed, selfInt)
  B_SPLINE_CURVE_WITH_KNOTS((mults), (knots), spec)
  RATIONAL_B_SPLINE_CURVE((weight…)) CURVE() REPRESENTATION_ITEM('') GEOMETRIC_REPRESENTATION_ITEM() )`
  — never a bare `B_SPLINE_CURVE_WITH_KNOTS` keyword. So it never reaches a builder;
  the edge (and the whole solid) declines. The code comment at line 901 records this:
  "Any other curve keyword (RATIONAL_B_SPLINE_*, …) is out of scope."
- `bsplineCurve()` (line 985) reads the **non-rational** keyword form only. It leaves
  `EdgeCurve::weights` **empty** and enforces the non-rational knot invariant
  `knots.size() == poles.size() + degree + 1`.
- The reader's own faithful-guard curve evaluator `evalEdge()` (line 2325) evaluates a
  `Kind::BSpline` edge with `math::curvePoint` **unconditionally** (line 2340) — the
  non-rational routine — so the guard would mis-evaluate a rational edge even once
  `weights` is populated.

Everything **else downstream is already rational-ready** and MUST NOT be touched:

- `EdgeCurve` (`topology/shape.h:190`) and `PCurve` (`shape.h:203`) already carry
  `std::vector<double> weights` (`empty ⇒ non-rational`).
- The tessellator's edge evaluator `edgeCurveLocal()` (`tessellate/edge_mesher.h:114`)
  **already** routes to `math::nurbsCurvePoint(...weights...)` when `weights` is
  non-empty (lines 137–141), exactly as the M4-rational surface story relied on
  `edgeCurveLocal`/`nurbsSurfacePoint`. So the native mesher meshes a rational edge the
  moment `weights` is populated — the tessellator is NOT modified.
- `math::nurbsCurvePoint` (`math/bspline.h:77`) already exists.
- The B-spline trim machinery — `trimmedCurve()` (line 912), `trimmedRange()`
  (line 1639), `curveRange()`'s B-spline arm (lines 1622–1627) — selects the covered
  knot sub-domain from the two `PARAMETER_VALUE` trims and is kind-agnostic; a rational
  basis reuses it unchanged.

So this slice is a **read-side change**, mirroring M4-rational one level down (curve
instead of surface): parse the combined `RATIONAL_B_SPLINE_CURVE` record, populate
`EdgeCurve::weights`, make the ONE non-rational guard evaluator (`evalEdge`)
rational-aware, and let the already-landed trim machinery + tessellator + engine
self-verify do the rest. This is the FIRST bounded slice — ONE foreign rational
B-spline curve as edge/trim geometry — not a general NURBS-curve importer. Where the
combined record is malformed, the weight count does not equal the pole count, a weight
is non-finite or non-positive, or the faithful guard fails, the reader keeps the
**honest decline** (the edge stays OCCT). A correct decline is a first-class, expected
outcome; no wrong or leaky solid is emitted and no dead code is written. If a real
foreign file authoring a rational edge curve is not robustly producible from OCCT, the
measured reachability gap is recorded and the decline stands (see design D5).

## What Changes

1. **A rational-curve arm in `curve()`** (`src/native/exchange/step_reader.cpp`). Before
   the existing `if (!r || r->combined) return nullptr` decline, route a **combined**
   record carrying a `RATIONAL_B_SPLINE_CURVE` sub-record (alongside a `B_SPLINE_CURVE`
   and a `B_SPLINE_CURVE_WITH_KNOTS` sub-record) to a new `rationalBsplineCurve()`
   builder — reusing the existing `hasSub` / `findSub` sub-record scan (lines 1444–1449)
   that `rationalBsplineSurface()` already uses. Every OTHER combined record keeps its
   current decline; every non-combined keyword path is byte-identical.
2. **`rationalBsplineCurve()`**: gather `degree` + the pole list from the
   `B_SPLINE_CURVE` sub-record and the RLE `(mults, knots)` from the
   `B_SPLINE_CURVE_WITH_KNOTS` sub-record (the same fields `bsplineCurve()` reads, only
   **split across two sub-records** in the combined form), then read the `(weight…)`
   list from the `RATIONAL_B_SPLINE_CURVE` sub-record into `EdgeCurve::weights` in the
   SAME order as the poles. Validate: `knots.size() == poles.size() + degree + 1`,
   `weights.size() == poles.size()`, and every weight finite and strictly positive;
   otherwise `std::nullopt` (decline). The shared pole/knot parse is factored into a
   helper so the non-rational `bsplineCurve()` keyword path produces a byte-identical
   `EdgeCurve`.
3. **Make the guard evaluator rational-aware.** `evalEdge()` (line 2325) SHALL route the
   `Kind::BSpline` case to `math::nurbsCurvePoint` when `c.weights` is non-empty and to
   the existing `math::curvePoint` otherwise — mirroring `edgeCurveLocal` and the
   rational-aware surface evaluator. Byte-identical for every non-rational edge
   (`weights` empty).
4. **No change downstream of the read.** The populated `weights` flow unmodified through
   the trim machinery (`trimmedCurve` / `trimmedRange` / `curveRange`, unchanged), the
   tessellator's already-rational `edgeCurveLocal`, and the engine's mandatory
   watertight + volume/area self-verify. A native solid that is not watertight or
   off-volume is DISCARDED → OCCT.
5. **The honest-out is preserved end-to-end.** A malformed / mismatched / non-positive-
   weight combined record, or an edge whose faithful guard fails, `decline()`s → OCCT
   (unchanged precedent). No tolerance is weakened; `src/native/**` stays OCCT-free; the
   tessellator is NOT modified; `EdgeCurve` / `PCurve` are unchanged (`weights` already
   exists); the `cc_*` ABI is unchanged (additive read-side behaviour only).

## Capabilities

### Modified Capabilities

- `native-exchange`: ADDS admission of ONE foreign **rational** `RATIONAL_B_SPLINE_CURVE`
  combined-record curve used as an edge's 3D geometry or a `TRIMMED_CURVE` basis —
  reading curve `weights` into `EdgeCurve::weights`, making the reader's faithful-guard
  evaluator rational-aware, and meshing watertight through the landed trim machinery +
  M0 mesher — with the honest decline retained on any malformed / non-faithful /
  non-reachable rational curve.

## Impact

- `src/native/exchange/step_reader.cpp` — `curve()` gains a combined-record arm; a new
  `rationalBsplineCurve()` builder; the pole/knot parse shared with `bsplineCurve()` is
  factored so the non-rational keyword path is byte-identical; `evalEdge()` gains a
  rational route (weights-empty ⇒ byte-identical). Cognitive complexity kept in the
  parser band (each sub-record reader is a small helper).
- `src/native/topology/shape.h` — UNCHANGED (`EdgeCurve::weights` / `PCurve::weights`
  already exist).
- Tessellator (`tessellate/edge_mesher.h`, `face_mesher.h`) — UNCHANGED (`edgeCurveLocal`
  already routes rational edge curves to `math::nurbsCurvePoint`); zero-regression is
  structural, not merely asserted.
- **Zero-regression discipline.** The rational-curve arm is reachable ONLY by a combined
  `RATIONAL_B_SPLINE_CURVE` record, which TODAY declines (`curve()` returns `nullptr` on
  `r->combined`, line 883). No non-rational keyword path, no analytic curve, no existing
  mesh can change. This is PROVEN: every existing STEP round-trip and the full
  tessellation-sensitive suite MUST stay byte-identical; if any differs the rational arm
  is reverted and the rational curve keeps the OCCT decline. The M4 non-rational and
  M4-rational **surface** paths MUST stay byte-identical.
- **Out of scope (declines, documented not faked):** periodic/closed rational curves that
  need a seam close beyond the landed trim clamp; rational PCURVEs authored in the 2D
  parameter plane (the reader synthesises its own analytic pcurves — the STEP pcurves are
  ignored, per `curve()`'s SURFACE_CURVE unwrap); any rational **surface** widening
  beyond M4-rational; multi-curve rational assemblies. No `cc_*` ABI change; no CyberCad
  app change; no OCCT linked into `src/native/**`.
