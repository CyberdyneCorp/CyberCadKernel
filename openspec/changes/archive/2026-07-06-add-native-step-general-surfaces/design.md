# Design — add-native-step-general-surfaces

Widen the WORKING native STEP reader (`src/native/exchange/step_reader.{h,cpp}`, the archived
`add-native-step-import` → `widen-native-step-import` → `add-native-step-assemblies` →
`add-native-step-scaled-ap242` slices) to import two general-surface families it currently
DECLINES: **(T1)** a `TRIMMED_CURVE` edge (a basis curve bounded by two trims) and **(T2)** a
`SURFACE_OF_REVOLUTION` face (a profile revolved about an axis). Map ONLY onto native geometry that
GENUINELY EXISTS and self-verifies; DECLINE (NULL → OCCT, exactly like the landed `TOROIDAL_SURFACE`
decline) wherever no native kind is faithful. Clean-room from ISO 10303-21/-42/-43 + the existing
reader; OCCT (`STEPControl_Reader` / `STEPControl_Writer`) is the ORACLE + fixture-author + fallback
only. NOT a general-surface reader.

## 0. What the reader already does (the substrate this widens)

Two passes over a `map<#id, Record>`: Pass A resolves leaf geometry, Pass B builds topology. The
two leaf dispatchers are thin keyword switches that fall through to `nullopt` (→ `decline()` at the
call site) on any unrecognised keyword:

```cpp
std::optional<topo::EdgeCurve> curve(long id) {           // step_reader.cpp ~L775
  ... if (SURFACE_CURVE|SEAM_CURVE|INTERSECTION_CURVE) return curve(arg[1]);  // unwrap
  if (LINE)  return lineCurve(*r);
  if (CIRCLE) return circleCurve(*r);
  if (ELLIPSE) return ellipseCurve(*r);
  if (B_SPLINE_CURVE_WITH_KNOTS) return bsplineCurve(*r);
  return std::nullopt;   // TRIMMED_CURVE, RATIONAL_B_SPLINE_*, … out of scope  ← T1 adds here
}
std::optional<topo::FaceSurface> surface(long id) {       // step_reader.cpp ~L868
  if (PLANE) ...; if (CYLINDRICAL_SURFACE) ...; if (SPHERICAL_SURFACE) ...;
  if (CONICAL_SURFACE) ...; if (B_SPLINE_SURFACE_WITH_KNOTS) return bsplineSurface(*r);
  return std::nullopt;   // SURFACE_OF_REVOLUTION, TOROIDAL_SURFACE, … out of scope  ← T2 adds here
}
```

An `EDGE_CURVE('',#v0,#v1,#curve,sense)` builds ONE shared native `Edge` per `#id`: it resolves the
3D curve via `curve()`, then computes the parameter range with `curveRange(cv, p0, p1)` from the two
vertex endpoints, and calls `ShapeBuilder::makeEdgeWithVertices(cc, first, last, verts)`. The engine
(`native_engine.cpp::step_import`) keeps the result native ONLY when `robustlyWatertightImport`
passes (per-member for a Compound); else OCCT.

**Three exact facts this design leans on (verified in the source):**

- The native `Edge` **is a trimmed 3D curve bounded by two vertices** over a stored `[first,last]`
  range — `shape.h`: `Shape::Type::Edge` = *"A trimmed 3D curve bounded by two vertices"*, with
  `firstParam()` / `lastParam()` accessors and a `makeEdgeWithVertices(curve, first, last, verts)`
  builder. So a `TRIMMED_CURVE` (basis + two trim parameters) maps onto the EXISTING native trimmed
  edge with **no new topology** — the basis is the `EdgeCurve`, the trims are `[first,last]`.
- The native `FaceSurface` carries a `weights` field (`shape.h`: *"empty ⇒ non-rational"*) AND the
  tessellator evaluates **rational NURBS** end-to-end: `surface_eval.h::localValue`/`localD1` call
  `math::nurbsSurfacePoint` / `nurbsSurfaceDerivs` with `{s_.weights…}` when weights are present;
  `edge_mesher.h` calls `math::nurbsCurvePoint`; `gpu_sample.h` reads `s.weights`. The math
  (`bspline.h`) has full rational curve+surface point+derivative routines. So a **rational
  tensor-product B-spline** surface — the EXACT form of a surface of revolution — is genuinely
  representable + meshable by the un-modified tessellator. Only the *reader* has never authored a
  rational face (policy, no fixture).
- `math::Ax3` / the analytic `Cylinder` / `Cone` / `Sphere` structs (`elementary.h`) have the exact
  parametrizations the tessellator uses (`S=O+R(cos u·X+sin u·Y)+v·Z` etc.). A degenerate revolution
  reduces to one of these **exactly** — no approximation.

## 1. The native capability this maps onto (checked)

| STEP concept | Native construct | Exists? | Evidence |
|---|---|---|---|
| A `TRIMMED_CURVE` (basis + two trims) on an edge | `Edge` = trimmed `EdgeCurve` over `[first,last]` via `makeEdgeWithVertices` | **YES** | `shape.h` `Edge` doc + `firstParam`/`lastParam`; basis is an existing `EdgeCurve` kind |
| A revolved LINE ∥ / meeting / ⟂ axis | `FaceSurface::Kind::{Cylinder,Cone,Plane}` | **YES** | `elementary.h` exact quadric params; reader already builds them + synthesizes their pcurves |
| A revolved on-axis circular arc | `FaceSurface::Kind::Sphere` | **YES** | `elementary.h` `Sphere` param; on-axis arc IS a sphere zone |
| A revolved in-slice generatrix (general profile) | `FaceSurface::Kind::BSpline` + `weights` (rational) | **YES (T2b, self-verify-gated)** | `surface_eval.h`/`edge_mesher.h`/`gpu_sample.h` + `bspline.h` evaluate rational NURBS; revolution = exact rational tensor B-spline (Piegl–Tiller A8.1) |
| A revolved OFF-axis circular arc (a torus) | — (no `FaceSurface::Kind::Torus`) | **NO → DECLINE (T2c)** | same geometry the landed `TOROIDAL_SURFACE` decline refuses |
| A directly-authored arbitrary rational surface | — (reader authors weights only for the exact revolution) | **NO → DECLINE** | not this slice; a foreign arbitrary rational surface stays OCCT |

The native trimmed edge + analytic quadrics + the (verified) rational-NURBS tessellator path are the
substrate; the new work is (a) a `TRIMMED_CURVE` unwrap + trim-driven range, (b) a
`SURFACE_OF_REVOLUTION` classifier that reduces to a quadric / builds the exact rational revolution /
declines, all gated by the existing watertight self-verify.

## 2. T1 — `TRIMMED_CURVE` → the native trimmed edge

### 2.1 The entity (ISO 10303-42)

```
#tc = TRIMMED_CURVE('', #basis_curve, (trim_1), (trim_2), sense_agreement, master_representation);
```

- `#basis_curve` — a `LINE` / `CIRCLE` / `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` (or a curve reached
  through the existing `SURFACE_CURVE` wrapper). Resolved recursively by `curve()` — reuses every
  existing builder unchanged.
- `(trim_1)` / `(trim_2)` — each a SET of one or two `trimming_select`s: a `CARTESIAN_POINT` and / or
  a `PARAMETER_VALUE(x)` (a bare REAL tagged by position). At least one; often both.
- `sense_agreement` — `.T.` ⇒ the trim runs from `trim_1` to `trim_2` in the basis curve's natural
  parameter direction; `.F.` ⇒ reversed.
- `master_representation` — `.PARAMETER.` / `.CARTESIAN.` / `.UNSPECIFIED.` — which trim form is
  authoritative when both are present.

### 2.2 The mapping

The basis curve **is** the native `EdgeCurve` (geometry unchanged). The trims are the edge's
`[first,last]`:

- `trimmedCurve(const Record& r) → optional<EdgeCurve>` resolves `curve(r.args[1].ref)` and returns
  it. It also records, keyed by the `TRIMMED_CURVE` `#id` (or threaded through a small
  `TrimSpec{ has_params, t0, t1, senseForward, masterIsParameter }` returned alongside), the trim
  parameters / points + sense so `edgeCurve()` can apply them.
- `edgeCurve()` today calls `curveRange(cv, p0, p1)` (vertex-only). The change: if the 3D curve
  resolved through a `TRIMMED_CURVE` with a `PARAMETER_VALUE` pair and `master_representation`
  is `.PARAMETER.` (or `.UNSPECIFIED.` with only params present), the edge range is `[t0,t1]` taken
  from the trims (swapped when `sense_agreement = .F.`), matched to the curve kind's parametrization
  (for a `CIRCLE`/`ELLIPSE` the STEP parameter is the angle in **degrees or radians per the unit
  context** — resolved against the mm/radian unit gate; for a B-spline it is the knot parameter).
  Otherwise (point-only trims, or `master = .CARTESIAN.`) the existing vertex-derived `curveRange`
  is kept — the trim points coincide with the edge's vertices, so the result is identical to today.

**Why the trim override matters (load-bearing).** `curveRange` for a `CIRCLE`/`ELLIPSE` assumes the
writer's convention: *"writer arcs sweep forward/CCW … keep a1 ahead of a0 … while (a1 <= a0) a1 +=
2π"* — it cannot recover an arc that sweeps **clockwise** or through **more than π** from the two
endpoints alone. A foreign `TRIMMED_CURVE` states the exact `[t0,t1]`; honoring it is the difference
between a correctly- and an incorrectly-swept arc (a wrong sweep → a self-intersecting / non-
watertight face → the self-verify would catch it and fall to OCCT, so the override is what lets these
files import NATIVELY rather than always bouncing to OCCT).

### 2.3 T1 honest decline

- basis curve out of slice (a rational / unsupported curve kind) → `curve()` already returns
  `nullopt` → DECLINE.
- degenerate / inconsistent trims (`t0 == t1`, a param outside the curve's valid range, a
  point-trim that matches no vertex) → DECLINE (never a zero-length or fabricated-range edge).

No new topology; the native `Edge` already stores an arbitrary `[first,last]`.

## 3. T2 — `SURFACE_OF_REVOLUTION` → the native surface that represents it, else DECLINE

### 3.1 The entity (ISO 10303-42)

```
#sor = SURFACE_OF_REVOLUTION('', #profile_curve, #axis1);
#axis1 = AXIS1_PLACEMENT('', #origin_point, #axis_direction);   // origin + ONE direction ($⇒+Z)
```

`axis1placement(id) → optional<pair<Point3,Dir3>>` mirrors `axis2placement` (a new tiny builder).
The profile is resolved with `curve()` (incl. T1's `TRIMMED_CURVE`). The reader then chooses the
representation by MEASURING the profile+axis — never by trusting a keyword.

### 3.2 T2a — exact analytic reduction (the degenerate revolutions)

Let the axis be `(A, d̂)` (point + unit direction) and consider the profile:

| Profile | Geometric condition | Native surface | Params |
|---|---|---|---|
| `LINE` (dir `ê`, point `P`) | `ê ∥ d̂` (parallel) | **Cylinder** | radius = ⊥-distance(P, axis); frame = `Ax3(A, d̂, ⊥-dir)` |
| `LINE` | `ê` not ∥, not ⟂; the line's support meets the axis at apex `Q` | **Cone** | apex `Q`; `semiAngle = ∠(ê, d̂)`; reference radius per the native `Cone` convention at `v=0` |
| `LINE` | `ê ⟂ d̂` (perpendicular) | **Plane** (planar annulus) | plane through `P` with normal `d̂` |
| `CIRCLE` / arc (center `C`, radius `ρ`, plane containing `d̂`) | `C` lies ON the axis | **Sphere** | center `C`, radius `ρ` |

Each condition is an exact algebraic test (dot / cross / point-line distance, scale-relative
tolerance). A match builds the analytic `FaceSurface` with the EXISTING `placedSurface` machinery
and the EXISTING analytic pcurve synthesis (`pcurveFor` / `angularURef`), because the reduced surface
is byte-identical to a natively-authored `CYLINDRICAL_SURFACE` / `CONICAL_SURFACE` / `PLANE` /
`SPHERICAL_SURFACE`. Fully faithful, lowest risk. (OCCT typically emits the analytic keyword directly,
so T2a mainly matters for foreign / hand-authored files that use `SURFACE_OF_REVOLUTION` for a
quadric.)

### 3.3 T2b — exact rational revolved B-spline (the general turned profile)

For an in-slice generatrix that does NOT reduce to a quadric — a general
`B_SPLINE_CURVE_WITH_KNOTS` or an `ELLIPSE` profile (a turned / profiled face) — the surface of
revolution is EXACTLY a rational tensor-product B-spline (Piegl & Tiller, *The NURBS Book*, Algorithm
A8.1 `MakeRevolvedSurf`):

- **U direction** (around the axis) — a **rational quadratic** B-spline circle: degree 2, `2m+1`
  poles for a sweep in `m` quadrant-spans (9 poles / knots `{0,0,0,¼,¼,½,½,¾,¾,1,1,1}` for a full
  turn), weights `{1, √2⁄2, 1, √2⁄2, …}`. This is the EXACT circle, not a polygonal approximation.
- **V direction** (along the profile) — the profile's own degree / knots / poles (a `B_SPLINE`
  profile passes through directly; an `ELLIPSE` profile is first expressed as its exact rational
  quadratic B-spline).
- **poles / weights** — each profile pole is swept to the ring of `2m+1` rational-circle poles at
  its distance from the axis; the surface weight grid is the outer product of the circle weights and
  the profile weights.

The result populates `FaceSurface{ kind = BSpline, degreeU = 2, degreeV = profileDeg, nPolesU,
nPolesV, poles (row-major), weights, knotsU, knotsV }` — precisely what `surface_eval.h`'s
`K::BSpline` + non-empty-`weights` branch feeds to `math::nurbsSurfacePoint`. **This is an exact
identity, not an approximation** (a revolved NURBS *is* this NURBS), so it is NOT "fabricated
geometry" — it is the same surface the file describes, in the native kernel's representation.

**pcurve synthesis (the real risk).** The reader synthesizes analytic pcurves per face surface; for a
T2b revolved surface the boundary pcurves live in the `(u = circle parameter, v = profile parameter)`
plane — the profile-end circles are `v = const` lines, the seam / profile edges are `u = const`
lines. `pcurveFor` is extended to emit these axis-aligned pcurve lines for a revolved face. If a
faithful pcurve cannot be synthesized for a given T2b face, that face → DECLINE (never a wrong
pcurve). The **watertight self-verify** (existing engine gate, and OCCT parity on the sim gate) is
the FINAL arbiter: a T2b face that does not self-verify → DECLINE → OCCT. So T2b can only ever import
a revolved face that is provably correct; otherwise it falls through exactly like TOROIDAL.

### 3.4 T2c — honest DECLINE (like `TOROIDAL_SURFACE`)

`surfaceOfRevolution` returns `nullopt` (→ DECLINE → OCCT) when:

- the profile is a **CIRCLE / arc whose center is OFF the axis** — the revolved surface is a
  **TORUS**. There is no `FaceSurface::Kind::Torus`; a torus IS an exact rational biquadratic B-spline,
  but we keep it a DECLINE for **cross-route consistency** with the landed `TOROIDAL_SURFACE` decline
  (we do not add a torus through the revolution back door while `surface()` declines
  `TOROIDAL_SURFACE`; a future slice can add a native torus kind and unify both routes).
- the profile is **rational / self-intersecting / out of slice**, or the **axis is degenerate**
  (zero direction, or intersecting the profile so the revolution self-intersects).
- ANY T2a / T2b reconstructed face fails the **watertight self-verify** (or, on the sim gate, OCCT
  parity). No tolerance is widened to force a pass.

## 4. `surface()` / `curve()` — the behavioural changes

```cpp
std::optional<topo::EdgeCurve> curve(long id) {
  ... existing wrappers + LINE/CIRCLE/ELLIPSE/B_SPLINE ...
  if (r->keyword == "TRIMMED_CURVE") return trimmedCurve(*r);   // T1 — resolve basis, record trims
  return std::nullopt;
}

std::optional<topo::FaceSurface> surface(long id) {
  ... existing PLANE/CYLINDRICAL/SPHERICAL/CONICAL/B_SPLINE_SURFACE ...
  if (r->keyword == "SURFACE_OF_REVOLUTION") return surfaceOfRevolution(*r);  // T2
  return std::nullopt;   // TOROIDAL_SURFACE etc. still DECLINE
}

std::optional<topo::FaceSurface> surfaceOfRevolution(const Record& r) {
  auto axis = axis1placement(r.args[2].ref);   // (A, d̂)
  auto prof = curve(r.args[1].ref);             // reuse the curve dispatcher (incl. TRIMMED_CURVE)
  if (!axis || !prof) return std::nullopt;                       // degenerate axis / out-of-slice
  if (auto q = reduceToQuadric(*prof, *axis)) return q;          // T2a — exact analytic
  if (isOffAxisArc(*prof, *axis)) return std::nullopt;           // T2c — TORUS → DECLINE
  if (auto s = revolveToRationalBSpline(*prof, *axis)) return s; // T2b — exact rational revolution
  return std::nullopt;                                           // T2c — else DECLINE
}
```

The single-solid / flat / placed / AP242 paths and every existing leaf builder are byte-unchanged
for files without a `TRIMMED_CURVE` / `SURFACE_OF_REVOLUTION`.

## 5. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • curve():  TRIMMED_CURVE → basis EdgeCurve + trim-driven [first,last]  (T1)
        │     │     • surface(): SURFACE_OF_REVOLUTION →
        │     │         reduceToQuadric  → Cylinder/Cone/Plane/Sphere        (T2a, exact)
        │     │         revolveToRationalBSpline → exact rational tensor NURBS (T2b, self-verify-gated)
        │     │         off-axis arc (torus) / rational / out-of-slice / degenerate → NULL (T2c)
        │     │     • single / flat / placed / AP242 paths unchanged
        │     ├─ NULL → OCCT fall-through
        │     └─ Solid/Compound → robustlyWatertightImport (per-member, volume>0) → wrap native
        │             └─ any member fails (incl. a wrongly-revolved / mis-trimmed face) → OCCT
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```

`src/native/**` stays OCCT-free (grep-gated). `step_writer.cpp` and the tessellator are NOT
modified (the tessellator's rational-NURBS path already exists + is verified). No `cc_*` ABI change;
default engine stays OCCT (opt-in `cc_set_engine(1)`).

## 6. Honest scope (what still DECLINEs → OCCT)

- **A `TOROIDAL_SURFACE`, or a `SURFACE_OF_REVOLUTION` of an off-axis arc (a torus)** — no native
  torus kind; honest DECLINE (T2c), consistent with the landed `TOROIDAL_SURFACE` decline.
- **A directly-authored arbitrary rational surface / curve** — the reader authors `weights` only for
  the EXACT revolved-B-spline construction (T2b, self-verified); a foreign arbitrary rational surface
  stays DECLINE.
- **General swept / bounded / offset surfaces** (`SURFACE_OF_LINEAR_EXTRUSION`,
  `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`, …) — DECLINE.
- **A `TRIMMED_CURVE` over an out-of-slice basis, or with degenerate / inconsistent trims** — DECLINE.
- **Any T2a / T2b face that fails the watertight self-verify** — DECLINE (never widened to pass).
- **IGES** import/export — stay OCCT `IGESControl_*`. **#8 `drop-occt`** stays blocked.

## 7. Cognitive complexity

`trimmedCurve` is a small ref-resolving builder (≤ ~8, mirrors `lineCurve`). The trim-range override
in `edgeCurve`/`curveRange` adds a few guarded branches (≤ ~12). `axis1placement` mirrors
`axis2placement` (≤ ~6). `surfaceOfRevolution` is a dispatcher (≤ ~8); `reduceToQuadric` is the
densest — four exact geometric classifications with a shared frame build (target ≤ ~20, parser/systems
band; each quadric arm isolated in a helper if it grows). `revolveToRationalBSpline` is the exact
`MakeRevolvedSurf` construction (a bounded loop over profile poles building the pole/weight/knot grid;
≤ ~20). All measured with the `cognitive-complexity` skill before archive; nothing pushed to a higher
band.

## 8. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - **T1 trimmed curve** — a `TRIMMED_CURVE` over a `LINE` / `CIRCLE` unwraps to the basis
    `EdgeCurve`; the edge's `[first,last]` comes from the trim `PARAMETER_VALUE`s, ranging a
    clockwise / `> π` arc CORRECTLY (a case the vertex-only `curveRange` would mis-range); a
    point-only trim reproduces the vertex-derived range; a `TRIMMED_CURVE` over a rational basis, or
    with `t0==t1`, DECLINES.
  - **T2a reduction** — a `SURFACE_OF_REVOLUTION` of a LINE ∥ axis → `Cylinder`(radius); of a LINE
    meeting the axis → `Cone`(semiAngle); of a LINE ⟂ axis → `Plane`; of an on-axis arc → `Sphere`;
    each reconstructed solid valid + watertight, matching the equivalent analytic-keyword solid.
  - **T2b build-or-decline** — a `SURFACE_OF_REVOLUTION` of a `B_SPLINE_CURVE_WITH_KNOTS` / `ELLIPSE`
    profile builds a `Kind::BSpline` `FaceSurface` with `weights` set (degU = 2); the assembled solid
    is kept iff it self-verifies watertight, else DECLINES (NULL).
  - **T2c torus decline** — a `SURFACE_OF_REVOLUTION` of an OFF-axis circular arc DECLINES (NULL),
    exactly like `TOROIDAL_SURFACE`.
  - **No regression** — the single-solid, flat multi-solid, placed rigid / uniform-scale / mirror
    assembly, AP242, quadric, and bspline-face round-trip cases STILL pass.
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - **(A) trimmed-edge parity** — OCCT authors a solid whose one edge is a `TRIMMED_CURVE` (a
    parameter-trimmed arc); native `cc_step_import` imports it; OCCT re-imports; assert same solid
    **count / volume / watertight / bbox** within tolerance.
  - **(B) cylinder/cone-reduction parity** — OCCT authors a turned solid whose lateral face is a
    `SURFACE_OF_REVOLUTION` reducing to a cylinder / cone (T2a); native import vs OCCT re-import
    agree on count / volume / watertight / bbox.
  - **(C) B-spline-profile revolution** — OCCT authors a turned solid whose face is a
    `SURFACE_OF_REVOLUTION` of a B-spline / ellipse profile (T2b); native import EITHER matches OCCT
    (count / volume / watertight / bbox) when it self-verifies, OR DECLINES → OCCT identical to
    `cc_set_engine(0)` — both outcomes asserted honest.
  - **(D) torus-groove decline** — OCCT authors a grooved solid whose face is a
    `SURFACE_OF_REVOLUTION` of an off-axis arc (a torus, T2c); native `cc_step_import` DECLINES → OCCT
    and matches `cc_set_engine(0)`.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown so the
    suite assertion count is unchanged.
- **Done** only when the relevant gates are green and every existing suite stays green at the OCCT
  default (prior import slices host round-trip + sim `[NIMPORT]` 41/41 incl. rigid / uniform-scale /
  mirror assemblies + AP242, STEP export, healing, SSI S1–S5, native blends + #6/#7, marching,
  boolean, construct, tessellation, phase3 do NOT regress). Honestly reported: this adds
  `TRIMMED_CURVE`-edge import (onto the native trimmed edge) + `SURFACE_OF_REVOLUTION`-face import
  (onto the exact native surface — analytic quadric or exact rational revolved B-spline — with a
  watertight self-verify gate); a torus revolution, an arbitrary rational surface, out-of-slice
  swept/bounded surfaces stay OCCT; #8 `drop-occt` stays blocked.
