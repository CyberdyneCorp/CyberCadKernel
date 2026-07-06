# Proposal — widen-native-step-import

## Why

The first native STEP **import** slice (`add-native-step-import`, archived) landed a
working ISO-10303-21 (Part-21) tokenizer + a two-pass AP203 `MANIFOLD_SOLID_BREP` mapper
behind `cc_step_import` (`src/native/exchange/step_reader.{h,cpp}`), OCCT-free, with an
engine-side OCCT `STEPControl_Reader` fallback (`src/engine/native/native_engine.cpp`). It
is deliberately narrow: it maps the exact entity set the native writer emits (planar +
elementary-quadric + basic non-rational B-spline solids, a SINGLE root solid, mm units) and
DECLINEs everything else to OCCT. This change **widens** that working reader along three
**independent, honestly-gated** breadth tracks — each maps ONLY onto native geometry that
genuinely exists, and DECLINEs (returns NULL → OCCT) otherwise. It is **not** a general
AP242 parser; the large long-lived arbitrary-import effort stays OCCT.

- **T1 — Entity coverage: `ELLIPSE` curve + `TOROIDAL_SURFACE` surface.** The reader today
  DECLINEs both. `EdgeCurve::Kind::Ellipse` is a **genuine native curve kind**
  (`src/native/topology/shape.h`: `radius` = major, `minorRadius` = minor) that the
  tessellator already evaluates (`edge_mesher.h::edgeCurveLocal` case `K::Ellipse`) and
  trims (`trim.h`). So T1 **maps `ELLIPSE` → `EdgeCurve::Kind::Ellipse`** (a real widening).
  `TOROIDAL_SURFACE`, by contrast, has **NO native `FaceSurface::Kind`** — the surface kinds
  are `{Plane, Cylinder, Cone, Sphere, BSpline, Bezier}` and the tessellator's
  `surface_eval.h` has no torus arm; the native `math::Torus` (`math/torus.h`) is only
  emitted by construction as a *rational* B-spline patch, and the reader (and this change)
  DECLINE rational B-splines and MUST NOT modify the tessellator. Therefore **T1 keeps
  `TOROIDAL_SURFACE` a DECLINE → OCCT and reports that honestly** — forcing a wrong map
  (e.g. planarizing or a rational patch that cannot self-verify) is prohibited. The native
  writer emits neither entity (an ellipse edge falls out of `curve()`'s switch to the
  `canSerialize` guard; there is no torus emitter), so the honest test is a **FOREIGN
  OCCT-authored** STEP with an elliptical edge (native import vs OCCT re-import).

- **T2 — Multi-solid import.** `findSingleManifoldBrep` DECLINEs the moment it sees a second
  `MANIFOLD_SOLID_BREP` (blanket assembly-decline). This change lifts that into a
  **multi-solid import**: map ALL root `MANIFOLD_SOLID_BREP`s to native solids and return a
  `ShapeBuilder::makeCompound({solid0, solid1, …})`. The engine self-verifies EACH member
  solid robustly watertight with positive volume before accepting the compound; if any member
  fails, the whole import DECLINEs → OCCT (never a partial import). NESTED product-structure
  assemblies (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, mapped-item transforms, external refs) stay
  out of scope → OCCT. Verified vs OCCT re-import per-solid (volume / watertight / count).

- **T3 — B-spline-FACE round-trip (the deferred import task 7.4).** The reader's
  `B_SPLINE_SURFACE_WITH_KNOTS` / `B_SPLINE_CURVE_WITH_KNOTS` mapping is implemented and
  reviewed, but 7.4 was left unchecked because **no non-fabricated native construct path was
  known to emit a WATERTIGHT B-spline-FACE solid** (a lone hand-built spline face is an open
  shell, no volume to round-trip). T3 **investigates whether such a fixture is now genuinely
  constructible** from an existing native op (loft / sweep / revolve producing
  `FaceSurface::Kind::BSpline` faces on a CLOSED solid the writer serialises via
  `B_SPLINE_SURFACE_WITH_KNOTS`). **IF** one exists, close 7.4 with a host round-trip
  (native export → native import EXACT: degrees, row-major poles, RLE-expanded knots, volume
  within analytic tolerance). **IF NONE exists**, report that honestly and leave 7.4 skipped
  — **do NOT fabricate a bspline-face fixture** and do NOT modify the writer to synthesize
  one that is not native-built.

This does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved
kernel still block it). It is a bounded breadth widening of the working import slice.

## What changes

1. **T1a — `ELLIPSE` → native ellipse edge** (`step_reader.cpp`, Pass A `curve()`). Add an
   `ELLIPSE('',#axis2placement,semiAxis1,semiAxis2)` arm inverse to a `CIRCLE`: resolve the
   `AXIS2_PLACEMENT_3D` frame, set `EdgeCurve::Kind::Ellipse` with `radius = semiAxis1`
   (major, along frame X), `minorRadius = semiAxis2` (along frame Y). The `EDGE_CURVE` param
   range is projected onto the ellipse exactly as it is for a circle (existing
   vertex-projection path). A degenerate ellipse (either semi-axis ≤ 0, or NaN) DECLINEs.
2. **T1b — `TOROIDAL_SURFACE` stays a DECLINE, documented.** No `FaceSurface::Kind::Torus`
   exists and the tessellator must not be modified, so `surface()` continues to return
   `std::nullopt` for `TOROIDAL_SURFACE` (→ NULL → OCCT). The DECLINE comment names the
   reason (no native surface kind + tessellator immutability) so it is not mistaken for an
   oversight. The `math::Torus` struct is untouched.
3. **T2 — multi-solid compound** (`step_reader.cpp`). Replace `findSingleManifoldBrep`'s
   `>1 → decline` with a `findManifoldBreps` that collects ALL root
   `MANIFOLD_SOLID_BREP` ids (a root = a `MANIFOLD_SOLID_BREP` reachable from an
   `ADVANCED_BREP_SHAPE_REPRESENTATION`, not nested under an assembly usage). `build()` maps
   each via the existing `mapManifoldBrep`; one solid → return the solid as today (zero
   behaviour change); ≥2 → `ShapeBuilder::makeCompound(solids)`. Any nested-assembly entity
   (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, `MAPPED_ITEM`, `REPRESENTATION_RELATIONSHIP*`) present
   → DECLINE (out of scope). The engine (`native_engine.cpp`) self-verifies: for a Solid, the
   existing `robustlyWatertight`; for a Compound, EACH child solid must be `robustlyWatertight`
   (a new small `robustlyWatertightMulti` helper) — any failure → OCCT fall-through.
4. **T3 — bspline-face round-trip IF constructible** (`tests/native/test_native_step_reader.cpp`,
   task 7.4). Attempt to build a watertight B-spline-face solid from an existing native op; if
   found, add a host round-trip test (export → import EXACT). If not, document the honest skip
   in tasks.md and the spec note. NO writer change and NO fabricated fixture.
5. **Verification** — extend `scripts/run-sim-native-step-import.sh` +
   `tests/sim/native_step_import_parity.mm` with (T1) a FOREIGN OCCT-authored elliptical-edge
   solid imported natively vs OCCT re-import; (T2) a FOREIGN OCCT-authored 2-solid file
   imported natively as a compound vs OCCT re-import (per-solid volume / watertight / count);
   plus a `TOROIDAL_SURFACE` FOREIGN file asserted to DECLINE natively and import via OCCT
   identical to `cc_set_engine(0)`. Host CTest gains the T1 ELLIPSE round-trip-style DECLINE/
   accept cases and (if constructible) the T3 bspline-face round-trip.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **`TOROIDAL_SURFACE` as a native surface** — no native `FaceSurface::Kind::Torus` exists
  and the tessellator (which this change must not modify) has no torus arm, so a torus face
  cannot be self-verified natively. It stays a DECLINE → OCCT. (If a future change adds a
  native Torus surface kind + tessellator arm behind its own gate, T1b can be revisited.)
- **`SURFACE_OF_REVOLUTION`, `TRIMMED_CURVE`, rational / weighted B-splines, `BEZIER`** — out
  of the writer subset and (revolution/trimmed) with no direct native kind → DECLINE → OCCT.
- **Nested product-structure assemblies** — `NEXT_ASSEMBLY_USAGE_OCCURRENCE`, mapped-item /
  `REPRESENTATION_RELATIONSHIP` transforms, external references → DECLINE → OCCT. T2 handles
  only multiple co-equal root solids in one representation (a compound), not a transform tree.
- **AP242 / PMI / GD&T / colours / names** → DECLINE → OCCT.
- **Non-mm units, non-manifold / unhealable B-reps** → DECLINE → OCCT (unchanged gates).
- **A fabricated B-spline-face fixture (T3)** — if no native op emits a watertight
  bspline-face solid, T3 is honestly skipped; no fixture is fabricated and the writer is not
  modified to synthesize one.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved
  kernel still block it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — T1a `ELLIPSE` arm in `curve()`; T2
  `findManifoldBreps` + compound assembly in `build()` (replacing the `>1` decline) + the
  nested-assembly DECLINE guard. `step_reader.h` / `native_exchange.h` — the
  `step_import_native` contract now documents ellipse edges + multi-solid compounds
  (signature unchanged: still returns one `topo::Shape`, now possibly a Compound). OCCT-free,
  host-buildable. `step_writer.cpp` and the tessellator are NOT modified.
- `src/engine/native/native_engine.cpp` — `step_import` self-verify handles a Compound
  (per-member `robustlyWatertight`); Solid path unchanged. `iges_*` / `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — T1 ELLIPSE accept + `TOROIDAL_SURFACE` DECLINE
  cases; T2 two-root → compound accept + nested-assembly DECLINE; T3 bspline-face round-trip
  if constructible (else documented skip).
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — T1
  foreign ellipse-edge parity, T2 foreign 2-solid compound parity, `TOROIDAL_SURFACE`
  fall-through. Own `main()`; on the `run-sim-suite.sh` SKIP list; default engine restored in
  teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine
  stays OCCT. The prior import slice (host round-trip, sim `[NIMPORT]` parity), STEP export,
  healing, SSI S1–S4, S5 native-pass, blends/#6/#7, phase3 do NOT regress.

## Verification

1. **Host round-trip / unit (OCCT-free).** T1: an `ELLIPSE` `EDGE_CURVE` in an in-scope entity
   graph maps to `EdgeCurve::Kind::Ellipse` (major/minor from the two semi-axes) and
   reconstructs; `TOROIDAL_SURFACE` DECLINEs (NULL). T2: a two-root buffer imports as a
   Compound of two solids; a nested-assembly buffer DECLINEs (NULL). T3 (if constructible): a
   native bspline-face solid round-trips export → import EXACT (degrees, poles, knots, volume
   within analytic tol); else the skip is recorded.
2. **Sim vs OCCT (simulator, OCCT linked).** T1: OCCT `STEPControl_Writer` authors a solid
   with an elliptical edge (foreign — the native writer emits none); native import vs OCCT
   re-import agree (volume / watertight / valid within tol). T2: OCCT authors a 2-solid STEP;
   native import returns a compound whose per-solid volume / watertight / count match the OCCT
   re-import. `TOROIDAL_SURFACE` foreign file DECLINEs natively and imports via OCCT identical
   to `cc_set_engine(0)`.

Done only when the relevant gates pass and every existing suite stays green at the OCCT
default. Reported honestly: T1 adds ELLIPSE (torus stays OCCT — no native kind); T2 adds
multi-solid compounds (nested assemblies stay OCCT); T3 closes the bspline-face round-trip
only if a non-fabricated fixture exists, else is skipped. Arbitrary / AP242 / IGES import
remain OCCT and #8 `drop-occt` stays blocked.
