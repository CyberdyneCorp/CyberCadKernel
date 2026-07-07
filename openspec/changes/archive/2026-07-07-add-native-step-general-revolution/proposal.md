# Proposal — add-native-step-general-revolution

## Why

The native STEP import reader (`add-native-step-import` → `widen-native-step-import` →
`add-native-step-assemblies` → `add-native-step-scaled-ap242` → `add-native-step-general-surfaces` →
`add-native-step-revolution-quadrics` → `add-native-step-torus`, all archived) tokenizes an ISO-10303-21
(Part-21) file and maps the in-slice B-rep subset to a native `topology::Shape`. The
`add-native-step-torus` slice closed **T1** of the revolution family — an OFF-AXIS `CIRCLE` / arc and the
direct `TOROIDAL_SURFACE` keyword now import onto a native `FaceSurface::Kind::Torus` (watertight, through
the already-proven sphere bare-periodic path; the tessellator's `face_mesher.h` was NOT touched). That
slice explicitly **DEFERRED T2** — an `ELLIPSE` or `B_SPLINE_CURVE_WITH_KNOTS` generatrix
`SURFACE_OF_REVOLUTION` — as "a larger, higher-blast-radius rational-tensor-B-spline reconstruction",
keeping the honest `default → nullopt` DECLINE → OCCT.

This change attacks that LAST revolution gap — **T2 only** — behind an HONEST watertight gate.

The exact surface a profile curve `P(v)` sweeps by a full revolution about an axis is a **rational
tensor-product B-spline surface**: the revolution's **standard rational-quadratic full circle** in `u`
(control net revolved at the standard knot angles `0, π/2, π, 3π/2, 2π` with the standard rational weights
`1, 1/√2, 1, 1/√2, 1`) tensored with the profile's own representation in `v`. The DECISIVE question the
torus slice named — **does the native substrate actually carry this?** — is answered YES in the source:

- **`FaceSurface::Kind::BSpline` carries `weights`** (`topology/shape.h` ~L224: `std::vector<double>
  weights; ///< empty ⇒ non-rational`). The kind is **rational-capable**; a revolved surface is
  representable.
- **The math substrate meshes rational NURBS surfaces** — `math::nurbsSurfacePoint` /
  `math::nurbsSurfaceDerivs` (`math/bspline.h` ~L113/L120) exist and are already wired into the
  tessellator: `tessellate/surface_eval.h` (~L163–172, L216–220) dispatches `Kind::BSpline` to the
  **rational** evaluators whenever `weights` is non-empty. So a rational B-spline surface **evaluates**
  through the existing, unmodified mesh path.

So the reconstruction is *representable* and *evaluable* today. What is NOT yet proven — and is the genuine
blast-radius the torus slice flagged — is **watertight meshing of a *periodic* revolved rational B-spline
face**: the existing rational-B-spline mesh path is exercised only by **non-periodic, pole-free**
freeform faces (e.g. the spline-wall prism). A full revolution is **`u`-periodic** (a seam at `u=0≡2π`),
and if the profile **touches the axis** its end **collapses to a degenerate pole**. Unlike the analytic
`Sphere` / `Torus`, a `Kind::BSpline` face has **no bare-periodic / canonical-seam-anchor reconstruction
path** in the mesher — a freeform face is meshed as an open `(u,v)` grid. Whether that grid welds the
`u`-seam and closes an axis pole watertight is the pivotal, must-prove gate.

Therefore this change is gated exactly like the torus T1 gate, but on the T2 axis:

- **T2 lands ONLY if the revolved rational B-spline face self-verifies watertight through the EXISTING
  mesh path.** A faithfully-representable revolution (the `u`-seam welds; a profile endpoint on the axis
  closes as a clean pole) is built natively as `Kind::BSpline` (rational) and gated by the engine
  `robustlyWatertightImport` self-verify.
- **T2 keeps the OCCT decline otherwise — with NO dead code and NO tessellator perturbation.** If the
  periodic rational B-spline reconstructs non-watertight (a leaky `u`-seam, an unclosable axis pole), OR
  if closing it would require modifying the mesher in a way that is not STRICTLY ADDITIVE + PROVEN
  byte-identical for every existing mesh (the torus slice set this precedent — `face_mesher.h` was left
  pristine), the reader keeps the `default → nullopt` DECLINE → OCCT. The ellipse / B-spline revolution
  already imports fine via OCCT — nothing is lost. **No approximate / non-rational surface is ever forced;
  no unreachable reconstruction is committed.**

This maps ONLY geometry the file exactly describes, VERIFIED faithful (sampled profile points lie on the
reconstructed surface) before emission, then gated by the watertight self-verify. It does NOT weaken any
tolerance. It does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved kernel
still block it); it is an additive breadth widening onto the existing rational `Kind::BSpline`.

## What changes

1. **Reader general-revolution mapping (`exchange/step_reader.cpp`).** `surfaceOfRevolution`'s
   `Ellipse` / `BSpline`-profile branch (currently `return std::nullopt; // …→ general revolution → DECLINE`
   at ~L987) gains a call to a new `revolvedProfileBSpline(profile, L, A)`.
2. **`revolvedProfileBSpline(profile, L, A) → optional<FaceSurface>`.** Builds the EXACT rational
   tensor-product B-spline:
   - **`u` direction** — the standard revolution full circle: `degreeU = 2`, **9** control poles,
     rational weights `{1, 1/√2, 1, 1/√2, 1, 1/√2, 1, 1/√2, 1}` (the four in-between corner poles carry
     `cos(45°) = 1/√2`; the five on-circle poles at the quadrant angles carry `1`), `knotsU` the standard
     `{0,0,0, π/2,π/2, π,π, 3π/2,3π/2, 2π,2π,2π}`.
   - **`v` direction** — the profile's own representation: an `ELLIPSE` (or `CIRCLE`) is promoted to its
     exact rational-quadratic B-spline; a non-rational `B_SPLINE_CURVE_WITH_KNOTS` is used directly
     (`degreeV`, `knotsV`, `w^v_j = 1`).
   - **Tensor product** — pole `P_ij = C_i(Q_j)` places the `i`-th revolution-circle control point at the
     `j`-th profile pole `Q_j`'s axial height and radius; weight `w_ij = w^u_i · w^v_j`. Emit
     `FaceSurface{Kind::BSpline, degreeU=2, degreeV=deg(profile), nPolesU=9, nPolesV=n_profile, poles,
     weights, knotsU, knotsV}` — a surface the tessellator ALREADY evaluates (`nurbsSurfacePoint`).
   - Isolated in its own function with small helpers (`revolutionCirclePoles`, `promoteProfileToNurbs`) so
     cognitive complexity stays in the parser/systems band.
3. **Faithful-reduction guard (`exchange/step_reader.cpp`).** `profileOnSurface(profile, surface)` samples
   the profile `P(v_k)` and asserts each lies on the reconstructed surface at `u=0` within a scale-relative
   tolerance; a mismatch → `nullopt` (DECLINE). Mirrors the existing `lineOnSurface` / `circleOnSurface`
   guards.
4. **HONEST WATERTIGHT GATE (`exchange/step_reader.cpp` face reconstruction + engine self-verify).** The
   revolved B-spline face is emitted ONLY when the reconstructed solid self-verifies watertight through the
   existing `robustlyWatertightImport`. If the `u`-seam leaks or a profile-endpoint axis pole cannot close
   through the existing rational-B-spline mesh path, the reader keeps `nullopt` (DECLINE → OCCT). No pole
   is fabricated; no seam is force-welded.
5. **Tessellator — VERIFY-ONLY, preferred UNCHANGED.** The rational B-spline mesh path already exists
   (`surface_eval.h` dispatches `nurbsSurfacePoint` / `nurbsSurfaceDerivs`). This change adds NO mesher
   code UNLESS a periodic-seam / axis-pole close for a `Kind::BSpline` face is genuinely required AND can be
   added STRICTLY ADDITIVELY with a PROVEN zero-regression on every existing mesh (the torus precedent). If
   it would perturb existing tessellation, T2 keeps the honest DECLINE instead.
6. **Reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`).** `step_import_native` signature
   unchanged; doc-comment updated: an ellipse / B-spline `SURFACE_OF_REVOLUTION` imports onto a native
   rational `Kind::BSpline` when faithfully representable + self-verifying, else DECLINE → OCCT. OCCT-free,
   host-buildable.
7. **Engine hook + OCCT fallback (`native_engine.cpp`) — unchanged logic, wider input.** `step_import`
   still calls `step_import_native` then `robustlyWatertightImport` (per-member for a Compound). A revolved
   B-spline face self-verifies exactly as any face; any NULL parse or leaky result → OCCT re-reads the SAME
   file. `iges_*` / `step_export` untouched.
8. **STEP writer UNCHANGED (OCCT-authored fixtures).** The verification fixtures are OCCT-authored (a real
   ellipse-profile / B-spline-profile `SURFACE_OF_REVOLUTION` solid) so `step_writer.cpp` need NOT emit a
   revolved surface. The writer stays byte-unchanged.
9. **Verification** — extend `tests/native/test_native_step_reader.cpp` +
   `scripts/run-sim-native-step-import.sh` + `tests/sim/native_step_import_parity.mm`: OCCT authors an
   ELLIPSE-profile revolution (a spheroid of revolution) and, if OCCT emits one, a B-spline-profile
   revolution solid; native import vs OCCT re-import agree on solid count / volume / watertight / bbox IF T2
   lands; if T2 stays DECLINED the test asserts the honest OCCT fallback matches `cc_set_engine(0)`. IF and
   ONLY IF the mesher was touched, the full tessellation-sensitive sim set is re-run to prove zero
   regression.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **T2 dropped if not faithfully representable + watertight.** A revolution whose reconstructed periodic
  rational B-spline self-verifies non-watertight (a leaky `u`-seam, an unclosable profile-endpoint axis
  pole), or which would require perturbing the existing tessellation to close, keeps its OCCT decline. No
  dead reconstruction code is committed.
- **A directly-authored arbitrary rational (weighted) B-spline SURFACE, a `SURFACE_OF_LINEAR_EXTRUSION`,
  `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`, or any general swept / bounded /
  offset surface** — out of this slice; DECLINE → OCCT.
- **A skew oblique line (hyperboloid)** — still DECLINE (unchanged).
- **A rational (weighted) `B_SPLINE_CURVE_WITH_KNOTS` profile** — the reader's `bsplineCurve` arm parses
  **non-rational** curves only (`step_reader.cpp` ~L905/L906), so a weighted-profile revolution is declined
  at the curve level upstream; only ELLIPSE / CIRCLE / non-rational B-spline profiles reach
  `revolvedProfileBSpline`. Widening the curve reader to rational is out of scope.
- **Inventing a curve, a surface, or a solid** — only geometry the file describes is mapped, verified to
  pass through the profile; any revolution that cannot be represented faithfully AND self-verified DECLINES.
  No tolerance is weakened.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved kernel still block
  it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — `surfaceOfRevolution` ellipse/bspline arm → `revolvedProfileBSpline`;
  the `revolutionCirclePoles` / `promoteProfileToNurbs` helpers; the `profileOnSurface` guard; the
  watertight-gated face reconstruction. `step_reader.h` / `native_exchange.h` doc-comments updated.
  OCCT-free, host-buildable.
- `src/native/topology/shape.h` — **no change**; `Kind::BSpline` + `weights` already carry a rational
  surface.
- `src/native/math/**` — **no change**; `nurbsSurfacePoint` / `nurbsSurfaceDerivs` already exist and are
  reused.
- `src/native/tessellate/**` — **preferred no change**; the rational B-spline mesh path already exists.
  Touched ONLY if a periodic-seam / axis-pole close is genuinely required AND provably additive +
  byte-identical; else T2 keeps the DECLINE.
- `src/native/exchange/step_writer.cpp` — **unchanged** (OCCT-authored fixtures).
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport` self-verifies
  every member). `iges_*` / `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — ellipse / B-spline revolution reduction/decline; every prior
  round-trip STILL passes.
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — the general-revolution
  parity / honest-decline case. Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored
  in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine stays OCCT.
  The landed STEP import (sim `[NIMPORT]` 69/69), STEP export, healing, SSI S1–S5, blends / #6 / #7,
  curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3, and the full tessellation-sensitive
  set do NOT regress.

## Verification

1. **Host unit (OCCT-free).** An ellipse (and, if authorable, a B-spline) `SURFACE_OF_REVOLUTION` maps to a
   rational `Kind::BSpline`, VERIFIED the profile lies on the surface; the reconstructed solid is valid +
   watertight and matches the analytic spheroid volume/bbox IF T2 lands, else the reduction is host-verified
   and the end-to-end path DECLINES (NULL). The on-axis-circle → Sphere, off-axis-circle → Torus, line →
   cylinder/cone/plane, trimmed-curve, quadric, bspline-face, and assembly round-trips are unchanged.
2. **Sim vs OCCT (simulator, OCCT linked).** OCCT `STEPControl_Writer` authors an ELLIPSE-profile revolution
   (a spheroid) and, if emitted, a B-spline-profile revolution solid; native `cc_step_import` (engine 1)
   imports each; OCCT `STEPControl_Reader` re-imports the same file; a track that LANDS agrees on solid
   count / volume / watertight / bbox within tolerance; a track that stays DECLINED imports via OCCT
   identical to `cc_set_engine(0)`.
3. **Tessellation zero-regression PROOF — ONLY IF the mesher was touched.** If (and only if) a periodic-seam
   / axis-pole close required an additive mesher branch, `run-sim-suite`, curved-fillet, curved-chamfer,
   curved-boolean, wrap-emboss, and phase3 all stay green with **identical** triangle counts, watertight
   status, and volumes for every existing face. If the mesher was NOT touched (preferred), existing
   tessellation is byte-identical by construction.

Done only when the relevant gate passes and every existing suite stays green at the OCCT default. Reported
honestly: T2 either adds ellipse / B-spline revolution import onto a native **rational `Kind::BSpline`**
surface (watertight-gated, no tessellator perturbation) — OR keeps the OCCT decline if not faithfully
representable + watertight. Arbitrary / AP242-general / IGES import remain OCCT and #8 `drop-occt` stays
blocked.
