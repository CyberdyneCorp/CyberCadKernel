# Proposal — add-native-step-general-surfaces

## Why

The native STEP import reader (`add-native-step-import` → `widen-native-step-import` →
`add-native-step-assemblies` → `add-native-step-scaled-ap242`, all archived) tokenizes an
ISO-10303-21 (Part-21) file and maps the in-slice B-rep subset to a native `topology::Shape`. It
now imports single-solid, flat multi-solid, and rigid / uniform-scale / mirror single-level
placed assemblies, and it tolerates AP242 PMI. But its leaf-geometry dispatchers still **DECLINE
two whole families of general surfaces the format routinely carries**, so any file that uses them
declines the WHOLE file → OCCT:

1. **A `TRIMMED_CURVE` edge.** `curve(#id)` (the 3D-curve dispatcher) accepts only bare `LINE`,
   `CIRCLE`, `ELLIPSE`, and `B_SPLINE_CURVE_WITH_KNOTS` (plus the `SURFACE_CURVE` / `SEAM_CURVE` /
   `INTERSECTION_CURVE` wrappers it already unwraps). A `TRIMMED_CURVE('',#basis,(trim1),(trim2),
   sense,master)` — a basis curve explicitly bounded between two trim points / parameters — falls
   through to the `return std::nullopt` at the end of `curve()` and declines (the code comment even
   names it: *"Any other curve keyword (TRIMMED_CURVE, RATIONAL_B_SPLINE_*, …) is out of scope."*).
   Yet the native `Edge` **is** *"a trimmed 3D curve bounded by two vertices"* over a stored
   `[first,last]` parameter range (`shape.h` `firstParam()`/`lastParam()`) — the `TRIMMED_CURVE`'s
   basis is one of the curve kinds the reader already builds, and its two trims are exactly the
   parameter bounds the native trimmed edge already models. The reader is declining a curve the
   native topology represents natively.

2. **A `SURFACE_OF_REVOLUTION` face.** `surface(#id)` accepts `PLANE`, `CYLINDRICAL_SURFACE`,
   `SPHERICAL_SURFACE`, `CONICAL_SURFACE`, and `B_SPLINE_SURFACE_WITH_KNOTS` (non-rational);
   anything else — including `SURFACE_OF_REVOLUTION` (a profile curve revolved about an axis — a
   turned / grooved / filleted profile) — falls through to `return std::nullopt` and declines,
   exactly as `TOROIDAL_SURFACE` does. **Some** revolutions are exactly a native surface the
   reader already builds (a line revolved about a parallel axis is a cylinder; a line meeting the
   axis is a cone; an on-axis circular arc is a sphere), and — a fact verified in the tessellator —
   the native `FaceSurface` carries a `weights` field and `surface_eval.h` / `edge_mesher.h` /
   `gpu_sample.h` GENUINELY evaluate **rational (weighted) NURBS** surfaces + curves
   (`nurbsSurfacePoint` / `nurbsSurfaceDerivs` / `nurbsCurvePoint`), so the exact rational
   tensor-product B-spline form of a revolved in-slice generatrix is representable + meshable by
   the un-modified tessellator. But **other** revolutions have **no** faithful native kind (an
   off-axis circular arc is a **torus**, and there is no `FaceSurface::Kind::Torus` — the same
   geometry the landed `TOROIDAL_SURFACE` decline honestly refuses).

This slice **widens native import along the two bounded, honest general-surface tracks the reader
declines** — mapping ONLY onto native geometry that genuinely exists, and DECLINING honestly (NULL
→ OCCT, exactly like `TOROIDAL_SURFACE`) wherever no native kind represents the entity faithfully:

- **(T1) `TRIMMED_CURVE` → the native trimmed edge.** `curve()` gains a `TRIMMED_CURVE` arm that
  resolves the **basis** curve (recursively — `LINE` / `CIRCLE` / `ELLIPSE` /
  `B_SPLINE_CURVE_WITH_KNOTS`, including a basis reached through the existing `SURFACE_CURVE`
  wrapper) and reads the **two trims**. The basis is the native `EdgeCurve`; the trims set the
  edge's `[first,last]` parameter range. The trims may be `PARAMETER_VALUE(x)` and / or a
  `CARTESIAN_POINT`; when a `PARAMETER_VALUE` pair is present (and `master_representation` is
  `.PARAMETER.`) the reader uses it — honoring `sense_agreement` — to set the EXACT range,
  including an arc that sweeps clockwise or through more than π that the current vertex-only
  `curveRange` heuristic (which assumes the writer's CCW-forward, < 2π convention) cannot recover;
  when only point trims are present the reader falls back to the existing vertex-derived range. A
  `TRIMMED_CURVE` whose basis is itself out of slice (a rational / unsupported curve) or whose
  trims are inconsistent / degenerate still **DECLINES → OCCT**.

- **(T2) `SURFACE_OF_REVOLUTION` → the native surface that represents it, else honest DECLINE.**
  `surface()` gains a `SURFACE_OF_REVOLUTION` arm that resolves the **axis**
  (`AXIS1_PLACEMENT` — origin + one direction) and the **profile** curve (reusing `curve()`,
  including T1's `TRIMMED_CURVE`), then chooses ONE of:
  - **(T2a) Exact analytic reduction** — when the profile + axis degenerate in closed form to a
    quadric the reader already builds: a **LINE** parallel to the axis → **Cylinder**; a **LINE**
    whose extension meets the axis → **Cone**; a **LINE** perpendicular to the axis → **Plane**
    (planar annulus); an on-axis **CIRCLE / circular arc** (center on the axis, plane containing
    it) → **Sphere**. The reduced face is built with the EXISTING analytic `FaceSurface` + the
    reader's existing pcurve synthesis. This is fully faithful — the revolved quadric **is** that
    native quadric.
  - **(T2b) Exact rational revolved B-spline** — when the profile is an in-slice generatrix that
    does NOT reduce to a quadric (a general `B_SPLINE_CURVE_WITH_KNOTS` or `ELLIPSE` profile — a
    turned / profiled face), the exact surface of revolution is a **rational tensor-product
    B-spline** (V = the profile, U = a rational-quadratic circle; the standard Piegl–Tiller
    `MakeRevolvedSurf` construction — an EXACT identity, not an approximation). The native
    `FaceSurface::Kind::BSpline` + `weights` + the tessellator's `nurbsSurfacePoint` GENUINELY
    represent + mesh it. The reader builds it, synthesizes the revolved-plane pcurves, and **keeps
    it ONLY IF** the assembled solid **self-verifies watertight** (and, on the sim gate, matches
    OCCT within tolerance); otherwise it **DECLINES → OCCT**.
  - **(T2c) Honest DECLINE (like `TOROIDAL_SURFACE`)** — an **off-axis circular arc** profile (the
    revolved surface is a **TORUS**: no native `FaceSurface::Kind::Torus`; kept a DECLINE for
    consistency with the landed `TOROIDAL_SURFACE` decline — we do NOT add a torus through the
    revolution back door while declining it through `TOROIDAL_SURFACE`); a **rational /
    self-intersecting / out-of-slice** profile; a **degenerate axis**; or ANY revolved surface
    (T2a or T2b) whose reconstructed face fails the watertight self-verify. NULL → OCCT.

So the slice **teaches the two leaf dispatchers two honest general-surface arms**: a `TRIMMED_CURVE`
maps onto the native trimmed edge (T1), and a `SURFACE_OF_REVOLUTION` maps onto the exact native
surface that represents it — analytic quadric (T2a) or exact rational revolved B-spline (T2b) — with
a hard watertight self-verify gate, DECLINING honestly (T2c) wherever no native kind is faithful.
It is **not** a general-surface reader: a directly-authored arbitrary rational surface, a torus, a
non-uniform swept / bounded / offset surface, and every previously-declined out-of-slice construct
stay DECLINE → OCCT.

This does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved kernel
still block it). It is an additive breadth widening of the working import slice.

## What changes

> **AS LANDED (honest reconciliation — the plan below was the design intent; this is what the code
> actually does, and what the living spec + docs claim).** The narrower reality was reached
> because the deferred cases could not pass the engine's watertight self-verify, so they DECLINE
> honestly rather than being forced:
> - **T1 (LANDED, narrowed).** `TRIMMED_CURVE` is now **accepted** (it declined before): the basis
>   (`LINE`/`CIRCLE`/`ELLIPSE`/`B_SPLINE_CURVE_WITH_KNOTS`) is unwrapped onto the native edge. The two
>   `PARAMETER_VALUE` trims drive `[first,last]` **only for a B-spline basis** (the covered knot
>   sub-domain, clamped); for an analytic basis the vertex-derived range is kept (trims redundant).
>   `sense_agreement` / `master_representation` are **not** consulted (no reliable OCCT fixture needed
>   them for a watertight round-trip). Items 1–2 below landed in this narrowed form.
> - **T2 (LANDED, narrowed to the cylinder case only).** `SURFACE_OF_REVOLUTION` is now accepted
>   **only** for a straight `LINE` generatrix **parallel** to the axis → an exact native `Cylinder`
>   (item 4's line-∥-axis branch, via `revolvedLine`). Item 3 (`axis1placement`) landed.
> - **DEFERRED → honest DECLINE → OCCT (NOT implemented):** the T2a **cone / plane / sphere**
>   reductions (the reader's apex-carrying cone does not round-trip watertight — a pre-existing gap),
>   the entire **T2b exact rational revolved B-spline** construction, and the item-5 **T2b pcurve
>   synthesis**. There is no `reduceToQuadric` and no `revolveToRationalBSpline` in the code — a
>   non-parallel line or any non-line profile returns `nullopt`. These stay DECLINE → OCCT exactly
>   like the landed `TOROIDAL_SURFACE`, and are a candidate future slice. The living spec + STATUS
>   docs claim ONLY the LANDED behavior above.

1. **`TRIMMED_CURVE` arm in `curve()` (`step_reader.cpp`).** Add, before the final `nullopt`, a
   `TRIMMED_CURVE` branch: `trimmedCurve(const Record&) → optional<EdgeCurve>` that resolves the
   basis curve (arg[1], recursively via `curve()`), returning the native `EdgeCurve` unchanged; the
   trims (arg[2], arg[3]) + `sense_agreement` (arg[4]) + `master_representation` (arg[5]) are
   surfaced to the edge builder via a small side-channel so `edgeCurve()` can set the exact
   `[first,last]`.
2. **Trim-driven edge range (`step_reader.cpp`).** `edgeCurve()` / `curveRange()` gain a
   trim-override: when the EDGE_CURVE's 3D curve resolves through a `TRIMMED_CURVE` carrying a
   `PARAMETER_VALUE` pair (with `master_representation = .PARAMETER.`), the edge's `[first,last]`
   comes from those trims (honoring `sense_agreement`), overriding the vertex-only heuristic; a
   point-only trim falls back to the existing vertex-derived range. The basis-curve **geometry** is
   unchanged; only the bounding parameters come from the trims. The native edge already stores an
   arbitrary `[first,last]` — no new topology.
3. **`AXIS1_PLACEMENT` resolver (`step_reader.cpp`).** `axis1placement(id) → optional<pair<Point3,
   Dir3>>` reads `AXIS1_PLACEMENT('',#origin,#axis)` (origin + one direction; `$` axis defaults to
   +Z per schema) — the axis a `SURFACE_OF_REVOLUTION` revolves about. Mirrors `axis2placement`.
4. **`SURFACE_OF_REVOLUTION` arm in `surface()` (`step_reader.cpp`).** Add, before the final
   `nullopt`, a `SURFACE_OF_REVOLUTION` branch: `surfaceOfRevolution(const Record&) →
   optional<FaceSurface>` reading `SURFACE_OF_REVOLUTION('',#profile,#axis1)`. It resolves the axis
   (3) + profile (`curve()`), then:
   - **T2a** — classifies the profile+axis against the closed-form quadric conditions (line ∥ axis
     → Cylinder; line meeting axis → Cone; line ⟂ axis → Plane; on-axis arc → Sphere) and returns
     the exact analytic `FaceSurface`;
   - **T2b** — for an in-slice non-degenerate generatrix, constructs the exact rational
     tensor-product B-spline `FaceSurface` (`Kind::BSpline`, `weights` set, degU = 2 rational
     circle, degV = profile degree — an exact `MakeRevolvedSurf`) and returns it;
   - **T2c** — an off-axis arc (torus), a rational / out-of-slice profile, or a degenerate axis →
     `nullopt` (DECLINE). The watertight self-verify (existing engine gate) is the final arbiter
     for T2a/T2b: a reconstructed revolved face that does not self-verify → DECLINE → OCCT.
5. **pcurve synthesis for a revolved face (`step_reader.cpp`).** The reader synthesizes its own
   analytic pcurves per face surface (`pcurveFor` / `angularURef`). T2a reuses the analytic-quadric
   pcurve synthesis unchanged. T2b synthesizes pcurves in the revolved (u = circle parameter,
   v = profile parameter) plane; if a faithful pcurve cannot be synthesized for a T2b face, that
   face → DECLINE → OCCT (never a wrong pcurve → never a wrong mesh). No tessellator change.
6. **Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`).**
   `step_import_native` signature unchanged. Doc-comment updated: `TRIMMED_CURVE` edges import onto
   the native trimmed edge; `SURFACE_OF_REVOLUTION` faces import onto the exact native surface
   (analytic quadric or exact rational revolved B-spline) when representable + self-verifying, else
   DECLINE (like `TOROIDAL_SURFACE`); torus / arbitrary-rational / out-of-slice stay OCCT.
   OCCT-free, host-buildable.
7. **Engine hook + OCCT fallback (`native_engine.cpp`) — unchanged logic, wider input.**
   `step_import` still calls `step_import_native` then `robustlyWatertightImport` (per-member for a
   Compound). A `TRIMMED_CURVE` edge and a reduced / revolved-B-spline face self-verify exactly as
   any other; any NULL parse or leaky / non-watertight result → OCCT `STEPControl_Reader` re-reads
   the SAME file. `iges_*` / `step_export` untouched.
8. **healShell / tessellator policy (unchanged).** No tessellator change (its rational-NURBS path
   already exists and is verified). `readStepString` still does not planarize via `healShell`. A
   revolved face that leaves a gap fails the self-verify → OCCT.
9. **Verification** — extend `scripts/run-sim-native-step-import.sh` +
   `tests/sim/native_step_import_parity.mm` with OCCT-authored fixtures: (A) a solid with a
   `TRIMMED_CURVE` edge (an arc trimmed by parameters that sweeps the "hard" way) →
   native-import vs OCCT re-import (count / volume / watertight / bbox); (B) a **turned** solid
   whose lateral face is a `SURFACE_OF_REVOLUTION` reducing to a **cylinder / cone** (T2a) →
   native vs OCCT parity; (C) a **turned** solid whose face is a `SURFACE_OF_REVOLUTION` of a
   **B-spline / ellipse** profile (T2b) → either native parity (if it self-verifies) or honest
   fall-through; (D) a **grooved** solid whose face is a `SURFACE_OF_REVOLUTION` of an **off-axis
   arc** (a torus, T2c) → honest NULL → OCCT, asserting the fallback matches
   `cc_set_engine(0)`. Host CTest gains the `TRIMMED_CURVE` unwrap + trim-range cases, the T2a
   reduction cases, the T2b build-or-decline case, and the T2c torus decline.

Additive throughout; the `cc_*` ABI never changes; the default engine stays OCCT.

## Non-goals (DEFERRED — return NULL → OCCT, not implemented, not faked)

- **A directly-authored arbitrary rational (weighted) `*_B_SPLINE_SURFACE` / `*_B_SPLINE_CURVE`
  face or edge** — the reader authors `weights` ONLY for the EXACT revolved-B-spline construction
  (T2b), verified watertight. It does NOT accept a foreign, arbitrary rational B-spline surface /
  curve; that stays DECLINE → OCCT (a separate future slice).
- **A `TOROIDAL_SURFACE`, or a `SURFACE_OF_REVOLUTION` of an off-axis circular arc (a torus)** —
  no native `FaceSurface::Kind::Torus`; kept an honest DECLINE (T2c) consistent with the landed
  `TOROIDAL_SURFACE` decline. Not faked through the revolution path.
- **A general swept / bounded / offset / pipe surface** (`SURFACE_OF_LINEAR_EXTRUSION`,
  `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`, …) — out of this slice;
  DECLINE → OCCT.
- **A `TRIMMED_CURVE` whose basis is out of slice** (rational / unsupported curve kind), or whose
  trims are inconsistent / degenerate — DECLINE → OCCT.
- **Inventing a curve, a surface, a trim, or a solid** — only geometry the file describes is
  mapped; the revolved-surface construction is the EXACT mathematical revolution (Piegl–Tiller),
  never an approximation or a substitute; any revolved / trimmed entity that cannot be represented
  faithfully AND self-verified DECLINES rather than being forced onto a wrong native kind.
- **Unblocking #8 `drop-occt`** — a general STEP/AP242 reader + IGES + a general-curved kernel
  still block it. Reported honestly.

## Impact

- `src/native/exchange/step_reader.cpp` — `curve()` gains a `TRIMMED_CURVE` arm
  (`trimmedCurve` + trim-driven `[first,last]` in `edgeCurve`/`curveRange`); `surface()` gains a
  `SURFACE_OF_REVOLUTION` arm (`surfaceOfRevolution` = T2a analytic reduction | T2b exact rational
  revolved B-spline | T2c DECLINE) + `axis1placement`; T2b pcurve synthesis. The single-solid /
  flat / placed-assembly / AP242 paths are UNCHANGED for files without these entities.
  `step_reader.h` / `native_exchange.h` doc-comments updated. OCCT-free, host-buildable.
  `step_writer.cpp` and the tessellator are NOT modified.
- `src/native/topology/**` — no new primitive; the native trimmed `Edge` (`firstParam`/`lastParam`)
  and `FaceSurface::Kind::{Cylinder,Cone,Plane,Sphere,BSpline}` + `weights` already exist.
- `src/native/math/**` — no behavioural change; the exact revolution construction uses existing
  `Ax3` / NURBS types; the tessellator's rational-NURBS evaluation already exists (verified).
- `src/engine/native/native_engine.cpp` — **unchanged logic** (`robustlyWatertightImport`
  self-verifies every member). `iges_*` / `step_export` unchanged.
- `tests/native/test_native_step_reader.cpp` — `TRIMMED_CURVE` unwrap → basis `EdgeCurve` + exact
  trim range (incl. a CW / > π arc); T2a reductions (line→cylinder/cone, on-axis-arc→sphere); T2b
  build-or-decline (a B-spline-profile revolution self-verifies or declines); T2c off-axis-arc
  (torus) DECLINE; the single / flat / placed / AP242 round-trips STILL pass.
- `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh` — the (A)
  trimmed-edge, (B) cylinder/cone-reduction, (C) B-spline-profile, (D) torus-decline cases. Own
  `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown.
- **No** `cc_kernel.h` / `cc_kernel.cpp` change; the `cc_*` ABI is unchanged; default engine stays
  OCCT. The prior import slices (flat multi-solid + bspline-face + rigid / uniform-scale / mirror
  assemblies + AP242, sim `[NIMPORT]` 41/41), STEP export, healing, SSI S1–S5, blends/#6/#7,
  phase3 do NOT regress.

## Verification

1. **Host unit (OCCT-free).** `curve()` unwraps a `TRIMMED_CURVE` to its basis `EdgeCurve` and the
   edge takes the trims' `[first,last]` (a parameter-trimmed arc that sweeps CW / > π ranges
   correctly, which the vertex-only heuristic would mis-range); `surface()` reduces a line-profile
   revolution to `Cylinder` / `Cone`, an on-axis-arc revolution to `Sphere` (T2a), builds an exact
   rational revolved B-spline for a B-spline-profile revolution and keeps it iff it self-verifies
   watertight (T2b), and DECLINES an off-axis-arc (torus) revolution and an out-of-slice / rational
   profile (T2c, NULL); the single / flat / placed / AP242 / round-trip cases are unchanged.
2. **Sim vs OCCT (simulator, OCCT linked).** OCCT `STEPControl_Writer` authors (A) a solid with a
   `TRIMMED_CURVE` edge, (B) a turned solid whose lateral face is a cylinder/cone
   `SURFACE_OF_REVOLUTION`, (C) a turned solid whose face is a B-spline / ellipse-profile
   `SURFACE_OF_REVOLUTION`, (D) a grooved solid whose face is an off-axis-arc (torus)
   `SURFACE_OF_REVOLUTION`; native `cc_step_import` (engine 1) imports each; OCCT
   `STEPControl_Reader` re-imports the same file; (A)/(B) and (C, when it self-verifies) agree on
   solid **count**, **volume**, **watertight**, and **bbox** within tolerance; (D) — and (C) when
   it cannot self-verify — DECLINES natively and imports via OCCT identical to `cc_set_engine(0)`.

Done only when the relevant gates pass and every existing suite stays green at the OCCT default.
Reported honestly: this adds **`TRIMMED_CURVE`-edge import** (onto the native trimmed edge) +
**`SURFACE_OF_REVOLUTION`-face import** (onto the exact native surface that represents it — analytic
quadric or exact rational revolved B-spline — with a watertight self-verify gate); a **torus**
revolution, an arbitrary directly-authored rational surface, out-of-slice swept/bounded surfaces,
and every previously-declined construct stay OCCT; arbitrary / AP242-general / IGES import remain
OCCT and #8 `drop-occt` stays blocked.
