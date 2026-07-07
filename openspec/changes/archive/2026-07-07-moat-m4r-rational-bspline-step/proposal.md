# Proposal — moat-m4r-rational-bspline-step (MOAT M4-rational, first slice)

## Why

M4 landed native STEP admission of a foreign **non-rational**
`B_SPLINE_SURFACE_WITH_KNOTS` face: `surface()` routes that keyword to
`bsplineSurface()`, which parses degree/poles/knots and — guarded by the
per-edge `S_face(pcurve(t)) = C_edge(t)` faithful-reconstruction guard — hands the
trimmed `Kind::BSpline` face to the landed M0 trimmed-freeform mesher, verified
watertight vs OCCT `BRepMesh`.

But the reader does **not** read surface weights, so a genuine **rational** NURBS
surface declines to OCCT today. Two concrete facts in
`src/native/exchange/step_reader.cpp`:

- `surface()` (~line 918) returns `std::nullopt` for any **combined** Part-21
  record (`r->combined`). A rational B-spline surface is emitted by OCCT
  `STEPControl_Writer` as a combined instance
  `( BOUNDED_SURFACE() B_SPLINE_SURFACE(degU,degV,((#pts)),form,uClosed,vClosed,selfInt)
  B_SPLINE_SURFACE_WITH_KNOTS((uMults),(vMults),(uKnots),(vKnots),spec)
  GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((weights))) …SURFACE() )`
  — never a bare `B_SPLINE_SURFACE_WITH_KNOTS` keyword. So it never reaches
  `bsplineSurface()`; the whole face (and file) declines.
- `bsplineSurface()` (~line 1342) leaves `FaceSurface::weights` **empty**
  (`weights` is populated today only by the internal rational revolution builder).

Everything **downstream of the read is already rational-ready** and MUST NOT be
touched:

- `FaceSurface` (`topology/shape.h`) already carries `std::vector<double> weights`
  (`empty ⇒ non-rational`).
- The reader's guard evaluator `bsplineSurfaceValue()` (~line 2271) already routes
  to `math::nurbsSurfacePoint(...weights...)` when `weights` is non-empty, so the
  `S_face(pcurve(t)) = C_edge(t)` guard is rational-aware the moment `weights` is
  populated.
- The M0 trimmed-freeform mesh branch (`native-tessellation`) already evaluates
  vertices with `math::nurbsSurfacePoint / nurbsSurfaceDerivs` when `weights` is
  non-empty (proven in the M0 host analytic gate).

So the entire M4-rational slice is a **read-side change**: parse the combined
`RATIONAL_B_SPLINE_SURFACE` record, populate `FaceSurface::weights`, and let the
already-landed guard + mesher + engine self-verify do the rest. This is the FIRST
bounded slice — ONE foreign rational B-spline surface face — not a general NURBS
importer. Where the rational record is malformed, the weight grid does not match
the pole grid, a weight is non-positive, or any boundary edge fails the faithful
guard, the reader keeps the **honest decline** (the patch stays OCCT). A correct
decline is a first-class, expected outcome; no wrong or leaky solid is emitted and
no dead code is written.

## What Changes

1. **A rational-surface arm in `surface()`** (`src/native/exchange/step_reader.cpp`).
   When the surface record is a **combined** instance carrying a
   `RATIONAL_B_SPLINE_SURFACE` sub-record alongside a `B_SPLINE_SURFACE` and a
   `B_SPLINE_SURFACE_WITH_KNOTS` sub-record, dispatch to a new
   `rationalBsplineSurface()` builder. This reuses the existing combined-instance
   machinery already used for rational curves / assembly relationships (`Record::subs`,
   the sub-record scan pattern in `relationshipAndTransform()`), so no new tokenizer
   is added. Every non-combined keyword path is byte-identical.
2. **`rationalBsplineSurface()`**: gather `degU,degV,poles,form,closedU,closedV`
   from the `B_SPLINE_SURFACE` sub-record and the `(uMults,vMults,uKnots,vKnots)`
   from the `B_SPLINE_SURFACE_WITH_KNOTS` sub-record (the same fields
   `bsplineSurface()` reads, only **split across two sub-records** in the combined
   form), then read the `((weights))` grid from the `RATIONAL_B_SPLINE_SURFACE`
   sub-record into `FaceSurface::weights` in the SAME row-major (U-outer/V-inner)
   order as the poles. Validate: same `nPolesU × nPolesV` cardinality as the pole
   grid, non-ragged rows, and every weight finite and strictly positive; otherwise
   `std::nullopt` (decline). Factor the shared pole/knot parsing so
   `bsplineSurface()` (non-rational keyword path) stays byte-identical.
3. **No change downstream of the read.** The populated `weights` flow unmodified
   through the existing faithful-reconstruction guard (rational-aware via
   `math::nurbsSurfacePoint`), the existing `bsplinePCurveFor` pcurve arm, the
   landed M0 trimmed-freeform mesher, and the engine's mandatory watertight +
   volume/area self-verify. A native rational face that is not watertight or
   off-volume is DISCARDED → OCCT.
4. **The honest-out is preserved end-to-end.** A malformed / mismatched / non-
   positive-weight rational record, or a face whose boundary pcurve fails the guard,
   `decline()`s → OCCT (unchanged precedent). No tolerance is weakened;
   `src/native/**` stays OCCT-free; the native tessellator is NOT modified; the
   `cc_*` ABI is unchanged (additive read-side behaviour only).

## Capabilities

### Modified Capabilities

- `native-exchange`: ADDS admission of ONE foreign **rational**
  `RATIONAL_B_SPLINE_SURFACE` combined-record surface face — reading surface
  `weights` into `FaceSurface::weights` and meshing watertight through the landed
  guard + M0 mesher — with the honest decline retained on any malformed / non-
  faithful rational patch.

## Impact

- `src/native/exchange/step_reader.cpp` — `surface()` gains a combined-record
  arm; a new `rationalBsplineSurface()` builder; the pole/knot parsing shared with
  `bsplineSurface()` is factored so the non-rational keyword path is byte-identical.
  Cognitive complexity kept in the systems/parser band (each sub-record reader is a
  small helper).
- `src/native/topology/shape.h` — UNCHANGED (`FaceSurface::weights` already exists).
- Tessellator (`face_mesher.h`, `trim.h`, `surface_eval.h`) — UNCHANGED (the M0
  rational mesh path already consumes `weights`); zero-regression is therefore
  structural, not merely asserted.
- **Zero-regression discipline.** The rational arm is reachable ONLY by a combined
  `RATIONAL_B_SPLINE_SURFACE` record, which TODAY declines (`surface()` returns
  `nullopt` on `r->combined`). No non-rational keyword path, no analytic surface, no
  existing mesh can change. This is PROVEN: every existing STEP round-trip and the
  full tessellation-sensitive suite MUST stay byte-identical; if any differs the
  rational arm is reverted and the rational patch keeps the OCCT decline.
- **Out of scope (declines, documented not faked):** periodic/seamed rational
  surfaces that need a seam/pole close beyond the landed guard; multi-patch rational
  assemblies; rational surfaces whose boundary pcurve does not reconstruct
  faithfully; any rational curve read (still out of scope — `curve()` line ~809).
  No `cc_*` ABI change; no CyberCad app change; no OCCT linked into `src/native/**`.
