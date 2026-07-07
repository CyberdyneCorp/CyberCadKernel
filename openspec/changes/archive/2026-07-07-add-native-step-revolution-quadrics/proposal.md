# Proposal — add-native-step-revolution-quadrics

## Why

The native STEP import reader (`add-native-step-import` → `widen-native-step-import` →
`add-native-step-assemblies` → `add-native-step-scaled-ap242` → `add-native-step-general-surfaces`,
all archived) tokenizes an ISO-10303-21 (Part-21) file and maps the in-slice B-rep subset to a native
`topology::Shape`. The last slice taught the `surface()` dispatcher a `SURFACE_OF_REVOLUTION` arm, but
it LANDED **narrowed to one case**: a straight `LINE` generatrix **parallel** to the revolution axis →
an exact native `Cylinder` (via `revolvedLine`). **Every other revolution DECLINES → OCCT** — including
three cases that reduce, in closed form, to a native `FaceSurface` kind the reader **already builds**
for the analytic-keyword surfaces:

1. **A `LINE` generatrix OBLIQUE to the axis** (crossing it at an angle) is exactly a **native
   `Cone`** — half-angle = the line-axis angle, apex at the axis intersection. `FaceSurface::Kind::Cone`
   EXISTS; `surface()` already builds it for `CONICAL_SURFACE` (`placedSurface(K::Cone, 2)`), and the
   writer round-trips it watertight (`coneSurface` emits `CONICAL_SURFACE`). Yet `revolvedLine`
   declines an oblique line — the prior slice deferred it, noting only that its *apex-carrying*
   reconstruction had not been proven watertight.

2. **A `LINE` generatrix PERPENDICULAR to the axis** is exactly a **native `Plane`** (a flat
   annulus / disk face). `FaceSurface::Kind::Plane` EXISTS; `surface()` already builds it for `PLANE`.

3. **A `CIRCLE` / arc generatrix whose CENTRE lies ON the axis** (revolved about a diameter) is exactly
   a **native `Sphere`** — radius = the circle radius. `FaceSurface::Kind::Sphere` EXISTS; `surface()`
   already builds it for `SPHERICAL_SURFACE`, and the writer round-trips it (`SPHERICAL_SURFACE`).
   Yet `surfaceOfRevolution` rejects every non-`LINE` profile before it is even classified
   (`profile->kind != Line → nullopt`).

These are not new geometry: each revolved generatrix **IS** the native quadric the reader already
authors for the direct analytic keyword — the same `Cone` / `Plane` / `Sphere` `FaceSurface`, the same
tessellator path, the same writer round-trip. The reader is declining native geometry it fully
represents, merely because it arrived through the `SURFACE_OF_REVOLUTION` door rather than the
`CONICAL_SURFACE` / `PLANE` / `SPHERICAL_SURFACE` door.

This slice **extends the `SURFACE_OF_REVOLUTION` arm to those three analytic-quadric reductions**,
mapping ONLY onto native kinds that genuinely exist AND self-verify watertight, and keeping the honest
DECLINE (NULL → OCCT, exactly like `TOROIDAL_SURFACE`) for every revolution with **no faithful native
kind**:

- **(R1) LINE oblique to the axis → native `Cone`.** Classify the line-axis angle. When the line's
  support is **coplanar with and intersects** the axis, the revolution is an exact cone: apex at the
  intersection, half-angle = ∠(line, axis). Build the native `Cone` `FaceSurface` (frame on the axis,
  reference radius at the frame origin per the native `S(u,v)=O+(R+v·sinα)(cos u·X+sin u·Y)+v·cosα·Z`
  convention, `semiAngle` = the half-angle) — byte-identical to a `CONICAL_SURFACE`-keyword face.
- **(R2) LINE perpendicular to the axis → native `Plane`.** When ∠(line, axis) = 90°, the revolution
  is a flat annulus in the plane through the line normal to the axis: a native `Plane` `FaceSurface`
  (frame at the foot on the axis, Z = the axis direction) — byte-identical to a `PLANE`-keyword face.
- **(R3) CIRCLE / arc, centre ON the axis, plane containing the axis → native `Sphere`.** When the
  circle centre lies on the axis and the circle plane contains the axis direction (revolution about a
  diameter), the revolution is an exact sphere: a native `Sphere` `FaceSurface` (centre = the circle
  centre, radius = the circle radius) — byte-identical to a `SPHERICAL_SURFACE`-keyword face.
- **(R4) Honest DECLINE (like `TOROIDAL_SURFACE`)** — every revolution with no faithful native kind:
  a **CIRCLE / arc whose centre is OFF the axis** (→ a **torus**; there is no `FaceSurface::Kind::Torus`,
  kept consistent with the landed `TOROIDAL_SURFACE` decline); an **ELLIPSE** or
  **`B_SPLINE_CURVE_WITH_KNOTS`** generatrix (→ a general revolved surface; the reader authors no
  revolved-B-spline surface); a **skew** oblique line whose support does NOT meet the axis (→ a
  **hyperboloid of one sheet**, no native kind); a **degenerate axis** (zero direction, or the line
  ON the axis); a circle whose plane does **not** contain the axis; or ANY reduced face that fails the
  engine's **watertight self-verify**. NULL → OCCT.

Each reduction is **verified faithful before it is emitted** — the resulting analytic surface must pass
through the profile (the line lies on the cone/plane, the circle lies on the sphere) within a
scale-relative tolerance — and is then gated by the existing engine watertight self-verify. If either
check fails, the reader DECLINES rather than forcing a wrong native kind.

This does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved kernel
still block it). It is an additive breadth widening of the working revolution arm onto native kinds
that already exist.

## What changes

1. **`revolvedLine` gains the oblique (Cone) and perpendicular (Plane) arms
   (`step_reader.cpp`).** Today it maps ONLY the parallel case (|cos∠| ≈ 1 → Cylinder) and declines
   everything else. It gains:
   - **R2 (perpendicular)** — when `|cos∠(line, axis)| ≈ 0`: build a native `Plane` `FaceSurface`,
     frame at the foot of the line's point on the axis, Z = the axis direction, X = the radial
     direction. (Verify the line is genuinely ⟂: every point of the line is equidistant along the
     axis, i.e. the line lies in a single plane normal to the axis.)
   - **R1 (oblique)** — otherwise (line neither ∥ nor ⟂): require the line's support to be **coplanar
     with the axis and to intersect it** at an apex `Q` (else it is a **skew** line → hyperboloid →
     DECLINE). Build a native `Cone` `FaceSurface`: frame on the axis (origin `O` = the foot of the
     line's reference point on the axis, `Z` = the axis direction oriented toward the opening),
     `radius` = ⊥-distance from that reference point to the axis (the reference radius at `O`, matching
     the native cone `v=0` convention), `semiAngle` = ∠(line, axis) folded to `(0, π/2)`. **Verify** the
     line lies on the cone (the reduction passes through the profile) before returning it.
2. **`surfaceOfRevolution` gains the CIRCLE → Sphere / torus classification
   (`step_reader.cpp`).** The current early `profile->kind != Line → nullopt` guard is replaced by a
   dispatch: a `Line` profile → `revolvedLine` (now R1/R2/Cylinder); a `Circle` profile →
   `revolvedCircle` (R3 / R4); any other kind (`Ellipse` / `BSpline` / `Bezier`) → `nullopt` (R4
   DECLINE). `revolvedCircle(circle, axisPoint, axisDir) → optional<FaceSurface>` reduces:
   - **R3 (Sphere)** — when the circle centre lies ON the axis (⊥-distance ≈ 0) AND the circle plane
     contains the axis direction (the axis is a diameter of the swept sphere): build a native `Sphere`
     `FaceSurface` (centre = circle centre, radius = circle radius). **Verify** the circle lies on the
     sphere before returning.
   - **R4 (DECLINE)** — a centre OFF the axis (a **torus**, no native kind), or a circle plane that
     does not contain the axis (degenerate / non-spherical), or any other configuration → `nullopt`.
3. **Faithful-reduction guard (`step_reader.cpp`).** Each R1/R2/R3 builder computes the candidate
   analytic surface and checks the generatrix's defining points lie on it within a scale-relative
   tolerance (the line's two support points on the cone/plane; the circle's centre + a rim point on
   the sphere) BEFORE returning; a failed check → `nullopt` (never a mis-fit face). The existing engine
   `robustlyWatertightImport` self-verify remains the final arbiter — any reduced face that leaves a
   gap → DECLINE → OCCT. No tolerance is widened.
4. **Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`).**
   `step_import_native` signature unchanged. Doc-comment updated: a `SURFACE_OF_REVOLUTION` face
   imports onto the exact native surface that represents it — `Cylinder` (line ∥ axis), `Cone` (line
   oblique, meeting the axis), `Plane` (line ⟂ axis), or `Sphere` (on-axis circle / arc) — when
   representable + self-verifying, else DECLINE (like `TOROIDAL_SURFACE`); torus / hyperboloid /
   ellipse- / B-spline-profile revolutions stay OCCT. OCCT-free, host-buildable.
5. **Engine hook + OCCT fallback (`native_engine.cpp`) — unchanged logic, wider input.**
   `step_import` still calls `step_import_native` then `robustlyWatertightImport` (per-member for a
   Compound). A reduced cone / plane / sphere revolved face self-verifies exactly as any analytic-
   keyword face; any NULL parse or leaky / non-watertight result → OCCT `STEPControl_Reader` re-reads
   the SAME file. `iges_*` / `step_export` untouched.
6. **Tessellator / STEP writer policy (unchanged).** No tessellator change (the native `Cone` /
   `Plane` / `Sphere` evaluation + pcurve synthesis already exist and are verified via the writer
   round-trip). `step_writer.cpp` is NOT modified. A revolved face that leaves a gap fails the
   self-verify → OCCT.
7. **Verification** — extend `tests/native/test_native_step_reader.cpp` +
   `scripts/run-sim-native-step-import.sh` + `tests/sim/native_step_import_parity.mm` with
   OCCT-authored fixtures: (A) a **CONE** solid (an oblique-line `SURFACE_OF_REVOLUTION`) → native
   import vs OCCT re-import (count / volume / watertight / bbox); (B) a revolved-to-**PLANE** face (a
   perpendicular-line revolution, e.g. the flat cap of a turned solid) → native vs OCCT parity; (C) a
   **SPHERE** solid (a semicircle `SURFACE_OF_REVOLUTION` about its diameter) → native vs OCCT parity;
   (D) an **OFFSET-circle (torus)** revolution and (E) an **ellipse / B-spline** revolution → honest
   DECLINE, asserting the OCCT fallback matches `cc_set_engine(0)`. Host CTest gains the R1 (cone),
   R2 (plane), R3 (sphere) reduction cases, the R4 torus + ellipse/B-spline + skew-line declines, and
   the existing parallel-line → cylinder case STILL passes.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **A `TOROIDAL_SURFACE`, or a `SURFACE_OF_REVOLUTION` of an off-axis circular arc (a torus)** — no
  native `FaceSurface::Kind::Torus`; kept an honest DECLINE (R4) consistent with the landed
  `TOROIDAL_SURFACE` decline. Not faked through the revolution path.
- **An `ELLIPSE` or `B_SPLINE_CURVE_WITH_KNOTS` generatrix (a general revolved surface)** — the exact
  surface is a rational tensor-product B-spline; the reader authors no revolved-B-spline surface in
  this slice (a candidate future slice). DECLINE → OCCT.
- **A skew oblique line (support does NOT meet the axis) → a hyperboloid of one sheet** — no native
  kind; DECLINE → OCCT (distinguished from the cone case by the coplanar-and-intersecting test).
- **A general swept / bounded / offset / pipe surface** (`SURFACE_OF_LINEAR_EXTRUSION`,
  `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`, …) — out of this slice;
  DECLINE → OCCT.
- **Inventing a curve, a surface, or a solid** — only geometry the file describes is mapped; each
  reduction is the EXACT analytic quadric the revolved generatrix defines, verified to pass through the
  profile; any revolution that cannot be represented faithfully AND self-verified DECLINES rather than
  being forced onto a wrong native kind. No tolerance is weakened.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved kernel still
  block it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — `revolvedLine` gains the R1 (oblique → Cone) + R2
  (perpendicular → Plane) arms with the coplanar-intersecting-axis + faithful-reduction guards;
  `surfaceOfRevolution` gains the profile-kind dispatch to a new `revolvedCircle` (R3 on-axis →
  Sphere / R4 off-axis torus DECLINE); ELLIPSE / B-spline / skew-line / degenerate cases DECLINE. The
  parallel-line → cylinder branch and every existing leaf builder are byte-unchanged for files without
  an oblique/perpendicular line or a circle revolution. `step_reader.h` / `native_exchange.h`
  doc-comments updated. OCCT-free, host-buildable. `step_writer.cpp` and the tessellator are NOT
  modified.
- `src/native/topology/**` — no new primitive; `FaceSurface::Kind::{Cone,Plane,Sphere}` +
  `radius` / `semiAngle` already exist and round-trip through the writer.
- `src/native/math/**` — no behavioural change; the reduction uses existing `Ax3` / point-line /
  point-plane geometry and the existing analytic `Cone` / `Plane` / `Sphere` parametrizations
  (`elementary.h`).
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport`
  self-verifies every member). `iges_*` / `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — R1 (oblique line → Cone), R2 (perpendicular line →
  Plane), R3 (on-axis circle → Sphere) reductions; R4 off-axis-circle (torus), ellipse / B-spline,
  and skew-line declines; the parallel-line → cylinder and all prior round-trips STILL pass.
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — the (A) cone,
  (B) plane, (C) sphere parity cases + the (D) torus and (E) ellipse/B-spline honest-decline cases.
  Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine stays
  OCCT. The prior import slices (sim `[NIMPORT]` 53/53 incl. trimmed-curve + revolution-cylinder +
  honest torus/general declines), STEP export, healing, SSI S1–S5, blends/#6/#7, phase3 do NOT
  regress.

## Verification

1. **Host unit (OCCT-free).** `surfaceOfRevolution` reduces an **oblique** line-profile revolution to
   `Cone` (apex on the axis, half-angle = the line-axis angle), a **perpendicular** line-profile
   revolution to `Plane`, and an **on-axis circle / arc** revolution to `Sphere` (radius = the circle
   radius); each reconstructed solid is valid + watertight and matches the equivalent
   `CONICAL_SURFACE` / `PLANE` / `SPHERICAL_SURFACE`-keyword solid. It DECLINES (NULL) an **off-axis
   circle** (torus), an **ellipse / B-spline** profile, and a **skew** oblique line (hyperboloid); the
   **parallel** line → cylinder case and the single / flat / placed / AP242 / trimmed-curve round-trips
   are unchanged.
2. **Sim vs OCCT (simulator, OCCT linked).** OCCT `STEPControl_Writer` authors (A) a CONE solid
   (oblique-line revolution), (B) a solid with a revolved-to-PLANE face (perpendicular-line
   revolution), (C) a SPHERE solid (semicircle revolution about its diameter), (D) a torus (off-axis
   circle) revolution, (E) an ellipse / B-spline revolution; native `cc_step_import` (engine 1) imports
   each; OCCT `STEPControl_Reader` re-imports the same file; (A)/(B)/(C) agree on solid **count**,
   **volume**, **watertight**, and **bbox** within tolerance; (D)/(E) DECLINE natively and import via
   OCCT identical to `cc_set_engine(0)`.

Done only when the relevant gates pass and every existing suite stays green at the OCCT default.
Reported honestly: this adds **`SURFACE_OF_REVOLUTION`-face import onto native `Cone` (oblique line),
`Plane` (perpendicular line), and `Sphere` (on-axis circle)** — extending the landed cylinder
reduction — with a faithful-reduction + watertight self-verify gate; a **torus** revolution, an
**ellipse / B-spline** revolution, a **skew-line hyperboloid**, arbitrary directly-authored surfaces,
and every previously-declined construct stay OCCT; arbitrary / AP242-general / IGES import remain OCCT
and #8 `drop-occt` stays blocked.
