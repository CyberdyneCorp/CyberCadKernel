# Proposal — add-native-step-torus

## Why

The native STEP import reader (`add-native-step-import` → `widen-native-step-import` →
`add-native-step-assemblies` → `add-native-step-scaled-ap242` → `add-native-step-general-surfaces` →
`add-native-step-revolution-quadrics`, all archived) tokenizes an ISO-10303-21 (Part-21) file and maps
the in-slice B-rep subset to a native `topology::Shape`. The last slice widened the
`SURFACE_OF_REVOLUTION` arm to the four **analytic-quadric** reductions the reader already authors for
the direct keyword — a `LINE` **parallel** → `Cylinder`, **oblique-meeting** → `Cone`, **perpendicular**
→ `Plane`, an **on-axis `CIRCLE`** → `Sphere`. What still DECLINES → OCCT is the **LAST STEP-surface
gap** in the revolution family, and it splits into exactly two tracks:

- **(T1) An OFF-AXIS `CIRCLE` / arc generatrix, and the direct `TOROIDAL_SURFACE` keyword → a TORUS.**
  `revolvedCircle`'s off-axis branch (`step_reader.cpp` ~L1063) DECLINES because there is **no**
  `FaceSurface::Kind::Torus` — the enum is `{Plane,Cylinder,Cone,Sphere,BSpline,Bezier}`. `TOROIDAL_SURFACE`
  is not even a `surface()` keyword arm (it hits the default `nullopt`). Yet `math::Torus` (`math/torus.h`,
  value/dU/dV/normal, major+minor radius, OCCT-`ElSLib`-parametrized) **already exists** and is used by
  SSI. The blocker is purely topological + tessellation: no native surface **kind** carries a torus, and
  the tessellator has no torus mesh path.

- **(T2) An `ELLIPSE` or `B_SPLINE_CURVE_WITH_KNOTS` generatrix → a general revolved surface.** The exact
  surface a profile curve sweeps about an axis is a **rational tensor-product B-spline** (the revolution's
  rational-quadratic circle in `u` ⊗ the profile curve in `v`). `FaceSurface::Kind::BSpline` **already
  exists** and the tessellator already meshes rational B-spline faces (`nurbsSurfacePoint` /
  `nurbsSurfaceDerivs`). The reader's `bsplineSurface` arm, however, only accepts **non-rational**
  direct-keyword surfaces and authors **no** revolved surface; `surfaceOfRevolution` sends every
  `Ellipse` / `BSpline` profile to the `default` DECLINE.

This slice attacks BOTH tracks, each behind an **HONEST per-track gate** — because the risk profile is
categorically higher than the quadric slice, which reused fully-verified existing kinds and left the
tessellator PRISTINE. Here T1 requires a **new native surface kind** and a **new tessellator mesh path**,
and the tessellator has been kept byte-stable all along precisely because a mesher change is
high-blast-radius (a prior change fixed one case and broke five). So the discipline is load-bearing:

- **T1 is ADDITIVE-ONLY and PROVE-BYTE-IDENTICAL or it does not land.** The new `Kind::Torus` is appended
  to the enum; a new `minorRadius` field is added (default `0.0`, so every existing kind is untouched);
  the torus mesh path is a **new branch** that does not touch the Plane/Cylinder/Cone/Sphere/BSpline/Bezier
  paths; and the change ships ONLY if the full tessellation-sensitive sim set proves the existing meshes
  are unchanged (same triangle counts, same watertight, same volumes across every existing suite). If a
  clean additive torus path cannot be achieved without perturbing existing tessellation, **T1 keeps the
  current OCCT decline** (torus already imports fine via OCCT — nothing is lost) and reports why.

- **T2 lands ONLY if the revolved rational B-spline reconstructs watertight; else it keeps the OCCT
  decline.** A faithfully-representable revolution (a closed rational B-spline, `u`-periodic, whose profile
  endpoints either close on the axis as clean poles or bound a genuine open zone) is built natively and
  gated by the engine watertight self-verify. A revolution that is not faithfully representable, or that
  self-verifies non-watertight (a leaky periodic seam, an unhandled pole), **DECLINES → OCCT** — never
  a mangled or approximate surface.

Both tracks map ONLY geometry the file exactly describes, each verified faithful (the circle lies on the
torus; the profile lies on the reconstructed B-spline) before it is emitted, then gated by the existing
`robustlyWatertightImport` self-verify. Neither track weakens a tolerance. This does NOT unblock #8
`drop-occt` (a general STEP/AP242 reader + IGES + a general-curved kernel still block it); it is an
additive breadth widening onto one new native kind (torus) and one existing kind (rational B-spline
revolution).

## What changes

1. **New additive `FaceSurface::Kind::Torus` + `minorRadius` field (`topology/shape.h`).** `Kind` gains
   `Torus` **appended** to `{Plane,Cylinder,Cone,Sphere,BSpline,Bezier}` (no existing enumerator's value
   changes). `FaceSurface` gains `double minorRadius = 0.0;` — the tube radius; the existing `radius`
   carries the **major** radius (axis→tube-centre), matching `math::Torus{majorRadius=radius,
   minorRadius}`. Every existing kind leaves `minorRadius` at its default, so their in-memory layout
   semantics and all existing round-trips are byte-unchanged.
2. **Additive torus arm in the surface evaluator (`tessellate/surface_eval.h`).** A new `detail::asTorus`
   and new `case K::Torus` in `localValue` / `localD1` / `bounds` (natural bounds `u∈[0,2π]`, `v∈[0,2π]`
   — periodic in BOTH directions, NO poles), delegating to `math::Torus`. The existing cases are
   byte-unchanged; the switch simply gains one arm.
3. **Additive torus MESH PATH in the face mesher (`tessellate/face_mesher.h`).** A torus face is
   classified as an **analytic-curved** surface (like Cylinder/Cone/Sphere — NOT freeform, NOT planar), so
   it flows through the EXISTING periodic-analytic grid + canonical-seam-anchor machinery with a full-turn
   grid in each direction. The additive work is a **doubly-periodic seam weld with NO pole**: the `u`-seam
   (`u=0≡2π`) and the `v`-seam (`v=0≡2π`) each weld via the existing shared-edge / canonical-anchor path
   the sphere's longitude seam already uses (simpler than a sphere — a ring torus has no degenerate pole
   vertex). NO existing (Plane/Cylinder/Cone/Sphere/BSpline/Bezier) branch is modified; the change is a new
   guarded branch, PROVEN byte-identical for existing meshes by the zero-regression gate. **If this cannot
   be done additively + watertight, T1 is dropped and the torus DECLINE is kept.**
4. **Reader torus mapping (`exchange/step_reader.cpp`) — two entry points.**
   - **Direct keyword**: `surface()` gains a `TOROIDAL_SURFACE` arm → `toroidalSurface(r)`, an
     `AXIS2_PLACEMENT_3D` + `majorRadius` + `minorRadius` builder (parallel to `placedSurface(K::Sphere,1)`
     but with two radii) → `Kind::Torus`.
   - **Revolution**: `revolvedCircle`'s off-axis branch (currently DECLINE) builds a `Kind::Torus`
     (`radius` = ⊥-distance(centre, axis) = major, `minorRadius` = the circle radius), frame on the axis,
     VERIFIED (the circle lies on the torus tube) before returning. An on-axis circle still reduces to
     `Sphere` (unchanged); a circle whose plane does not contain the tube correctly stays a torus/decline
     per the geometry.
   - **`projectUV` / `surfaceValue` / `pcurveFor`** gain torus arms: `u = atan2(ly,lx)`,
     `v = atan2(lz, hypot(lx,ly) − R)`; the faithful-reduction guard's `surfaceValue` gains `K::Torus`;
     `pcurveFor` synthesises the analytic rim/meridian pcurves (a `v`-const boundary is a `u`-circle, a
     `u`-const boundary is a `v`-circle) and the doubly-periodic seam pcurves the doubly-periodic face
     reconstruction needs.
   - **Full-torus face reconstruction**: unlike a sphere (a VERTEX_LOOP degenerate-pole bound), a full
     OCCT torus face is doubly periodic with NO pole and is bounded by seam edges. Task 1 diagnoses the
     exact emitted DATA (VERTEX_LOOP? seam EDGE_LOOP? two seams?) and the reconstruction closes it
     watertight ADDITIVELY (a doubly-periodic bare-surface face, analogous to the sphere bare-surface
     path) — **or, if that cannot be closed watertight without a writer/tessellator change beyond the
     additive budget, T1 keeps the OCCT decline**.
5. **Reader general-revolution mapping (T2, `exchange/step_reader.cpp`).** `surfaceOfRevolution`'s
   `Ellipse` / `BSpline`-profile branch gains a `revolvedProfileBSpline(profile, axis)` that builds the
   EXACT rational tensor-product B-spline `FaceSurface` (`Kind::BSpline`, rational): the `u` direction is
   the revolution's rational-quadratic full circle (weights `{1, cos(Δ/2), 1, …}` per the standard
   NURBS-circle representation), the `v` direction is the profile curve's own representation (a circle /
   ellipse promoted to its rational-quadratic B-spline; a `B_SPLINE_CURVE_WITH_KNOTS` used directly). The
   surface is emitted ONLY when the profile is faithfully representable AND the reconstructed face
   self-verifies watertight; otherwise `nullopt` (DECLINE → OCCT). No tolerance is weakened.
6. **Faithful-reduction guard (`exchange/step_reader.cpp`).** T1's `torusOnSurface` (four tube-quadrant
   points of the generatrix circle lie on the candidate torus within a scale-relative tolerance) and T2's
   `profileOnSurface` (sampled profile points lie on the reconstructed B-spline) run BEFORE the surface is
   returned; a failed check → `nullopt`. The engine `robustlyWatertightImport` self-verify remains the
   final arbiter. No tolerance widened.
7. **Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`).** `step_import_native`
   signature unchanged; doc-comment updated: a `TOROIDAL_SURFACE` face and an off-axis-circle
   `SURFACE_OF_REVOLUTION` import onto native `Kind::Torus` (when additive + self-verifying), and an
   ellipse / B-spline revolution onto a native rational `Kind::BSpline` (when faithfully representable +
   self-verifying); otherwise DECLINE → OCCT. OCCT-free, host-buildable.
8. **Engine hook + OCCT fallback (`native_engine.cpp`) — unchanged logic, wider input.** `step_import`
   still calls `step_import_native` then `robustlyWatertightImport` (per-member for a Compound). A torus /
   revolved-B-spline face self-verifies exactly as any analytic face; any NULL parse or leaky result → OCCT
   `STEPControl_Reader` re-reads the SAME file. `iges_*` / `step_export` untouched.
9. **STEP writer policy (prefer UNCHANGED; OCCT-authored fixtures).** The verification fixtures are
   **OCCT-authored** (a real `TOROIDAL_SURFACE` / off-axis-circle revolution and a general-revolution
   solid) so `step_writer.cpp` need NOT emit a torus for the gate. The writer is modified ONLY if a native
   round-trip fixture genuinely needs it (then `isPeriodicSurfaceKind` gains `Torus` additively and a torus
   `TOROIDAL_SURFACE` emitter mirrors the sphere path) — else it stays byte-unchanged.
10. **Verification** — extend `tests/native/test_native_step_reader.cpp` +
    `scripts/run-sim-native-step-import.sh` + `tests/sim/native_step_import_parity.mm`: OCCT authors (A) a
    **TORUS** solid (off-axis-circle revolution AND/OR direct `TOROIDAL_SURFACE`) and (B) a
    **general-revolution** (ellipse profile) solid; native import vs OCCT re-import agree on solid
    count / volume / watertight / bbox for any track that LANDS; for any track that stays DECLINED the test
    asserts the honest OCCT fallback matches `cc_set_engine(0)`. **CRITICAL**: the FULL
    tessellation-sensitive sim set (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean,
    wrap-emboss, phase3) is run to PROVE the new torus mesh path did not perturb existing tessellation.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **T1 dropped if not cleanly additive + byte-identical.** If the torus mesh path or the doubly-periodic
  torus face reconstruction cannot be added without perturbing existing tessellation or without a
  watertight close, the `TOROIDAL_SURFACE` / off-axis-circle revolution **keeps its current OCCT decline**.
  Reported plainly, not forced.
- **T2 dropped if not faithfully representable + watertight.** A revolution whose exact surface is not a
  faithfully representable rational B-spline, or whose reconstructed periodic B-spline self-verifies
  non-watertight (a leaky `u`-seam, an unhandled profile-endpoint pole), **keeps its OCCT decline**.
- **A `SURFACE_OF_LINEAR_EXTRUSION`, `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`,
  `CURVE_BOUNDED_SURFACE`, or any general swept / bounded / offset surface** — out of this slice; DECLINE
  → OCCT.
- **A skew oblique line (hyperboloid) / a directly-authored arbitrary rational B-spline surface** — still
  DECLINE (unchanged by this slice).
- **Inventing a curve, a surface, or a solid** — only geometry the file describes is mapped, each
  reduction verified to pass through the profile; any revolution that cannot be represented faithfully AND
  self-verified DECLINES. No tolerance is weakened.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved kernel still block
  it. Reported honestly.

## Impact

- `src/native/topology/shape.h` — `FaceSurface::Kind` gains `Torus` (appended); `FaceSurface` gains
  `double minorRadius = 0.0`. Additive; every existing kind byte-unchanged.
- `src/native/tessellate/surface_eval.h` — additive `K::Torus` arms (`asTorus`, `localValue`, `localD1`,
  `bounds`) delegating to `math::Torus`; existing arms byte-unchanged.
- `src/native/tessellate/face_mesher.h` — a new additive doubly-periodic torus mesh branch (no pole); the
  Plane/Cylinder/Cone/Sphere/BSpline/Bezier paths are NOT touched. Gated by the byte-identical proof.
- `src/native/exchange/step_reader.cpp` — `surface()` gains `TOROIDAL_SURFACE` → `toroidalSurface`;
  `revolvedCircle` off-axis → `Torus`; `surfaceOfRevolution` ellipse/bspline → `revolvedProfileBSpline`
  (T2); `projectUV` / `surfaceValue` / `pcurveFor` gain torus arms; the doubly-periodic torus face
  reconstruction; the `torusOnSurface` / `profileOnSurface` guards. `step_reader.h` /
  `native_exchange.h` doc-comments updated. OCCT-free, host-buildable.
- `src/native/math/**` — no behavioural change; `math::Torus` already exists and is reused.
- `src/native/exchange/step_writer.cpp` — **preferred unchanged** (OCCT-authored fixtures); modified only
  if a native torus round-trip fixture genuinely needs it (then `isPeriodicSurfaceKind` + a torus emitter,
  additively).
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport` self-verifies
  every member). `iges_*` / `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — torus (direct + off-axis-circle) reduction/decline;
  general-revolution reduction/decline; every prior round-trip STILL passes.
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — the torus and
  general-revolution parity / honest-decline cases. Own `main()`, on the `run-sim-suite.sh` SKIP list;
  default engine restored in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine stays OCCT.
  The landed STEP import (sim `[NIMPORT]` 69/69), STEP export, healing, SSI S1–S5, blends / #6 / #7,
  curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3, and the full tessellation-sensitive
  set do NOT regress.

## Verification

1. **Host unit (OCCT-free).** T1: a `TOROIDAL_SURFACE` face and an off-axis-circle `SURFACE_OF_REVOLUTION`
   map to `Kind::Torus` (major = axis distance, minor = circle radius), VERIFIED the circle lies on the
   torus; the reconstructed solid is valid + watertight IF T1 lands, else the reduction is host-verified
   and the end-to-end path DECLINES honestly. T2: an ellipse / B-spline revolution maps to a rational
   `Kind::BSpline`, VERIFIED the profile lies on the surface, watertight IF T2 lands, else DECLINE. The
   on-axis-circle → Sphere, line → cylinder/cone/plane, trimmed-curve, quadric, bspline-face, and assembly
   round-trips are unchanged.
2. **Sim vs OCCT (simulator, OCCT linked).** OCCT `STEPControl_Writer` authors (A) a TORUS solid
   (off-axis-circle revolution / `TOROIDAL_SURFACE`) and (B) a general-revolution (ellipse profile) solid;
   native `cc_step_import` (engine 1) imports each; OCCT `STEPControl_Reader` re-imports the same file; a
   track that LANDS agrees on solid count / volume / watertight / bbox within tolerance; a track that stays
   DECLINED imports via OCCT identical to `cc_set_engine(0)`.
3. **Tessellation zero-regression PROOF (booted sim).** `run-sim-suite`, curved-fillet, curved-chamfer,
   curved-boolean, wrap-emboss, and phase3 all stay green with **identical** triangle counts, watertight
   status, and volumes for every existing sphere / cylinder / cone / plane / B-spline face — the additive
   torus mesh path perturbs NOTHING.

Done only when the relevant gates pass and every existing suite stays green at the OCCT default. Reported
honestly per track: T1 adds `TOROIDAL_SURFACE` / off-axis-circle-revolution import onto a **new native
`Kind::Torus`** with an additive, byte-identical-proven torus mesh path — OR keeps the OCCT decline if the
additive bar is not met; T2 adds ellipse / B-spline revolution import onto a native **rational
`Kind::BSpline`** — OR keeps the OCCT decline if not faithfully representable + watertight. Arbitrary /
AP242-general / IGES import remain OCCT and #8 `drop-occt` stays blocked.
