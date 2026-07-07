# Design — add-native-step-revolution-quadrics

Extend the WORKING native STEP reader's `SURFACE_OF_REVOLUTION` arm
(`src/native/exchange/step_reader.cpp`, landed by `add-native-step-general-surfaces`) — which today
maps ONLY a straight `LINE` generatrix **parallel** to the axis → an exact native `Cylinder` — to the
other **analytic-quadric** revolution cases, each reducing to a native `FaceSurface` kind the reader
ALREADY builds for the direct analytic keyword: a `LINE` **oblique** to the axis → `Cone`, a `LINE`
**perpendicular** → `Plane`, an on-axis **CIRCLE / arc** → `Sphere`. Map ONLY onto native geometry that
GENUINELY EXISTS, is VERIFIED to pass through the profile, and SELF-VERIFIES watertight; DECLINE
(NULL → OCCT, exactly like the landed `TOROIDAL_SURFACE` decline) wherever no native kind is faithful
(a torus, a general revolved surface, a hyperboloid). Clean-room from ISO 10303-42/-43 + the existing
reader; OCCT (`STEPControl_Reader` / `STEPControl_Writer`) is the ORACLE + fixture-author + fallback
only. NOT a general-surface reader.

## 0. What the reader already does (the substrate this widens)

`surface(#id)` is a thin keyword switch; the `SURFACE_OF_REVOLUTION` arm added by the prior slice
resolves the axis + profile then reduces ONLY the parallel-line case:

```cpp
std::optional<topo::FaceSurface> surface(long id) {                 // step_reader.cpp ~L914
  if (PLANE) return placedSurface(*r, K::Plane, 0);
  if (CYLINDRICAL_SURFACE) return placedSurface(*r, K::Cylinder, 1);
  if (SPHERICAL_SURFACE)  return placedSurface(*r, K::Sphere, 1);
  if (CONICAL_SURFACE)    return placedSurface(*r, K::Cone, 2);      // Cone ALREADY built here
  if (B_SPLINE_SURFACE_WITH_KNOTS) return bsplineSurface(*r);
  if (SURFACE_OF_REVOLUTION) return surfaceOfRevolution(*r);
  return std::nullopt;                                              // TOROIDAL_SURFACE … DECLINE
}

std::optional<topo::FaceSurface> surfaceOfRevolution(const Record& r) {   // ~L949
  const auto profile = curve(r.args[1].ref);
  const auto ax      = axis1placement(r.args[2].ref);              // (A, d̂), $ ⇒ +Z
  if (!profile || !ax) return std::nullopt;
  if (profile->kind != topo::EdgeCurve::Kind::Line) return std::nullopt;  // ← R3/R4 replaces
  return revolvedLine(*profile, ax->first, ax->second);           // ← R1/R2 extend
}

std::optional<topo::FaceSurface> revolvedLine(const EdgeCurve& line, Point3 L, Dir3 A) {  // ~L973
  const Dir3 D = line.frame.x;                                     // line direction
  if (std::fabs(std::fabs(dot(A, D)) - 1.0) > 1e-7) return std::nullopt;  // not ∥ → DECLINE  ← R1/R2
  const Point3 P = line.frame.origin;
  const Point3 foot = L + A·dot(P−L, A);                           // foot of P on the axis
  const double r = norm(P − foot);                                 // ⊥-distance
  if (r < 1e-9) return std::nullopt;                               // line ON the axis
  FaceSurface s; s.kind = Cylinder; s.frame = Ax3::fromAxisAndRef(foot, A, {P−foot}); s.radius = r;
  return s;
}
```

**Three exact facts this design leans on (verified in the source):**

- `FaceSurface::Kind::{Plane,Cylinder,Cone,Sphere}` already exist (`shape.h`), the reader already
  builds all four for the direct analytic keyword (`placedSurface`), and the STEP **writer** emits
  `PLANE` / `CYLINDRICAL_SURFACE` / `CONICAL_SURFACE` / `SPHERICAL_SURFACE` and round-trips them
  watertight (`step_writer.cpp` `coneSurface`, `SPHERICAL_SURFACE`, …). So a revolved generatrix that
  reduces to one of these maps onto a kind the tessellator + writer already handle end-to-end — **no
  new topology, no tessellator change**.
- The native `Cone` parametrization is `S(u,v) = O + (R + v·sinα)(cos u·X + sin u·Y) + v·cosα·Z`
  (`elementary.h`), where `O` is the frame origin, `R` the **reference radius at v=0** (NOT necessarily
  the apex), and `α` the half-angle. So a cone reduction does **not** require placing the frame at the
  apex singular point — it places `O` at the foot of the generatrix's reference point on the axis with
  `R` = the ⊥-distance there, exactly as the writer authors a truncated (frustum) cone that round-trips
  watertight. This is why the prior slice's "apex-carrying reconstruction not watertight" concern is
  avoided: the frame is a regular point on the axis, not the apex.
- The native `Sphere` parametrization is `S(u,v) = O + R·cos v(cos u·X + sin u·Y) + R·sin v·Z`
  (`elementary.h`) — a circle of radius `R` centred at `O` revolved about the frame Z axis. An on-axis
  generatrix circle (centre on the axis, plane containing the axis) IS exactly this sphere zone.

## 1. The native capability this maps onto (checked)

| STEP concept (revolution) | Geometric reduction | Native construct | Exists? | Evidence |
|---|---|---|---|---|
| `LINE` ∥ axis | Cylinder (landed) | `FaceSurface::Kind::Cylinder` | **YES (landed)** | `revolvedLine` parallel branch |
| `LINE` oblique, support meets axis at apex Q | **Cone** (apex Q, α = ∠(line,axis)) | `FaceSurface::Kind::Cone` | **YES (R1)** | `placedSurface(K::Cone)` + writer `coneSurface` round-trip |
| `LINE` ⟂ axis | **Plane** (flat annulus normal to axis) | `FaceSurface::Kind::Plane` | **YES (R2)** | `placedSurface(K::Plane)` |
| `CIRCLE` / arc, centre ON axis, plane ∋ axis | **Sphere** (centre, radius = circle radius) | `FaceSurface::Kind::Sphere` | **YES (R3)** | `placedSurface(K::Sphere)` + writer `SPHERICAL_SURFACE` round-trip |
| `CIRCLE` / arc, centre OFF axis | Torus | — (no `Kind::Torus`) | **NO → DECLINE (R4)** | same geometry the `TOROIDAL_SURFACE` decline refuses |
| `LINE` skew (does NOT meet axis) | Hyperboloid of one sheet | — (no ruled-quadric kind) | **NO → DECLINE (R4)** | not representable natively |
| `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` | General revolved (rational tensor B-spline) | — (reader authors no revolved B-spline) | **NO → DECLINE (R4)** | out of this slice (future slice) |

The analytic quadrics + the (verified) writer/tessellator round-trip are the substrate; the new work
is (a) two extra arms in `revolvedLine` (oblique → Cone, perpendicular → Plane) with a
coplanar-and-intersecting-axis test that separates a cone from a hyperboloid, (b) a `revolvedCircle`
classifier (on-axis → Sphere, off-axis → torus DECLINE), (c) a faithful-reduction guard, all gated by
the existing watertight self-verify.

## 2. The entity (ISO 10303-42) — unchanged from the landed arm

```
#sor   = SURFACE_OF_REVOLUTION('', #profile_curve, #axis1);
#axis1 = AXIS1_PLACEMENT('', #origin_point, #axis_direction);   // origin + ONE direction ($ ⇒ +Z)
```

`axis1placement(id) → optional<pair<Point3,Dir3>>` already exists. The profile is resolved with
`curve()` (a `LINE` / `CIRCLE` / `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS`, including a `TRIMMED_CURVE`
basis). The reader chooses the representation by **MEASURING** the profile + axis — never by trusting a
keyword.

## 3. R1 / R2 — `revolvedLine` gains the oblique (Cone) and perpendicular (Plane) arms

Let the axis be `(L, Â)` (point + unit direction) and the line be point `P`, unit direction `D̂`
(`line.frame.origin`, `line.frame.x`). Let `c = |Â·D̂|` (|cos∠(axis, line)|).

- **`c ≈ 1` → Cylinder** (LANDED, unchanged). Radius = ⊥-distance(P, axis).
- **`c ≈ 0` → Plane (R2).** The line is perpendicular to the axis, so every point revolves in a single
  plane normal to `Â`. Build `FaceSurface{ kind = Plane, frame = Ax3(foot(P), Â, radial(P)) }` where
  `foot(P) = L + Â·(P−L)·Â` and `radial(P) = P − foot(P)`. **Verify** both line-support points share
  the same axial coordinate `(·−L)·Â` (they lie in one normal plane) before returning; else DECLINE.
- **Otherwise `0 < c < 1` → Cone (R1).** The generatrix is oblique. The revolution is a cone **iff the
  line's support and the axis are coplanar and intersect** at an apex `Q`; if they are **skew**, the
  revolution is a hyperboloid of one sheet — NO native kind → DECLINE. Test:
  1. Compute the common perpendicular between the two lines (support line `(P, D̂)`, axis `(L, Â)`).
     If its length > scale-relative tol → **skew** → DECLINE (R4 hyperboloid).
  2. Else find the intersection apex `Q` (solve `P + s·D̂ = L + t·Â`).
  3. Build the native `Cone`: pick the frame origin `O` = `foot(P)` (a regular point on the axis, NOT
     the apex), `R` = ⊥-distance(P, axis) (the reference radius at `O`), `Z` = `Â` oriented so the cone
     opens in the direction the generatrix recedes from the apex, `X` = `radial(P)`; `semiAngle` =
     `acos(c)` folded into `(0, π/2)` — the angle between the generatrix and the axis. **Verify** a
     second point of the line lies on the resulting cone (the reduction passes through the profile)
     before returning; else DECLINE.

Both R1 and R2 reuse the EXISTING analytic `FaceSurface` machinery and the reader's existing analytic
pcurve synthesis (`pcurveFor` for a Cone / Plane face), so the reduced face is byte-identical to a
`CONICAL_SURFACE` / `PLANE`-keyword face. The frustum cone (apex outside the face's v-range) is the
realistic OCCT-authored fixture and round-trips watertight; a face whose bound reaches the apex (a
degenerate collapsed seam) fails the watertight self-verify and DECLINES → OCCT — honest, not forced.

## 4. R3 / R4 — `revolvedCircle`: on-axis circle → Sphere, else torus DECLINE

`surfaceOfRevolution` replaces its `profile->kind != Line → nullopt` guard with a dispatch:

```cpp
switch (profile->kind) {
  case Line:   return revolvedLine(*profile, ax->first, ax->second);   // R1/R2/Cylinder
  case Circle: return revolvedCircle(*profile, ax->first, ax->second); // R3/R4
  default:     return std::nullopt;                                    // Ellipse/BSpline → R4 DECLINE
}
```

`revolvedCircle(circle, L, Â)` reads the circle centre `C` (`circle.frame.origin`), radius `ρ`
(`circle.radius`), and plane normal `N̂` (`circle.frame.z`):

- **R3 (Sphere)** — iff `C` lies ON the axis (⊥-distance(C, axis) ≤ tol) AND the circle plane
  **contains the axis direction** (`|N̂·Â| ≤ tol`, i.e. the axis is a diameter of the circle's plane):
  the circle revolved about a diameter through its centre is an EXACT sphere. Build
  `FaceSurface{ kind = Sphere, frame = Ax3(C, Â, ref), radius = ρ }`. **Verify** the circle lies on the
  sphere: `|C − C| = 0` (centre coincides) and a rim point `C + ρ·X_circle` is at distance `ρ` from
  `C` (trivially true) AND lies in a plane containing the axis — i.e. re-confirm `|N̂·Â| ≤ tol`. Return
  it; the reconstructed face is byte-identical to a `SPHERICAL_SURFACE`-keyword sphere zone.
- **R4 (DECLINE)** — `C` OFF the axis (⊥-distance > tol): the revolution is a **TORUS** (major radius =
  ⊥-distance, minor radius = `ρ`). There is no `FaceSurface::Kind::Torus`; `nullopt` (DECLINE),
  consistent with the landed `TOROIDAL_SURFACE` decline — no torus through the revolution back door.
  Also DECLINE when the circle plane does NOT contain the axis (`|N̂·Â| > tol`, a non-spherical /
  degenerate revolution).

An `Ellipse` / `BSpline` / `Bezier` profile never reaches `revolvedCircle` — it hits the `default`
DECLINE (its exact surface is a rational revolved B-spline the reader does not author in this slice).

## 5. The faithful-reduction guard (never a mis-fit face)

Every R1/R2/R3 builder VERIFIES the generatrix lies on the candidate analytic surface within a
scale-relative tolerance BEFORE returning — the load-bearing honesty check the GOAL demands
("the resulting analytic surface must pass through the profile"):

- **Cone (R1)** — both endpoints of the line project onto the cone's `S(u,v)` locus (correct radius at
  their axial coordinate: `R + v·sinα` equals their measured ⊥-distance).
- **Plane (R2)** — both line-support points share one axial coordinate (lie in the plane normal to the
  axis).
- **Sphere (R3)** — the circle centre coincides with the sphere centre and the circle plane contains
  the axis (so every circle point is at distance `ρ` from `C`).

A failed check → `nullopt` (DECLINE). Then the engine `robustlyWatertightImport` self-verify is the
FINAL arbiter: any reduced face that leaves a gap → DECLINE → OCCT. **No tolerance is widened to force
a pass**; the honest residual is a fall-through to OCCT, never a mangled surface.

## 6. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • surface(): SURFACE_OF_REVOLUTION →
        │     │         revolvedLine:   line ∥ → Cylinder (landed) | ⟂ → Plane (R2) | oblique-meets → Cone (R1)
        │     │         revolvedCircle: on-axis circle → Sphere (R3) | off-axis → torus DECLINE (R4)
        │     │         ellipse / B-spline / skew line / degenerate → NULL (R4)
        │     │     • single / flat / placed / AP242 / trimmed-curve paths unchanged
        │     ├─ NULL → OCCT fall-through
        │     └─ Solid/Compound → robustlyWatertightImport (per-member, volume>0) → wrap native
        │             └─ any member fails (incl. an apex-reaching cone / mis-fit face) → OCCT
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```

`src/native/**` stays OCCT-free (grep-gated). `step_writer.cpp` and the tessellator are NOT modified
(the native Cone / Plane / Sphere evaluation + writer round-trip already exist + are verified). No
`cc_*` ABI change; default engine stays OCCT (opt-in `cc_set_engine(1)`).

## 7. Honest scope (what still DECLINEs → OCCT)

- **A `TOROIDAL_SURFACE`, or a `SURFACE_OF_REVOLUTION` of an off-axis circular arc (a torus)** — no
  native torus kind; honest DECLINE (R4), consistent with the landed `TOROIDAL_SURFACE` decline.
- **An `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` generatrix (a general revolved surface)** — the reader
  authors no revolved rational B-spline in this slice; DECLINE → OCCT (a candidate future slice).
- **A skew oblique line (support does not meet the axis) → a hyperboloid of one sheet** — no native
  kind; DECLINE, distinguished from the cone case by the coplanar-and-intersecting test.
- **General swept / bounded / offset surfaces** (`SURFACE_OF_LINEAR_EXTRUSION`,
  `RECTANGULAR_TRIMMED_SURFACE`, `OFFSET_SURFACE`, `CURVE_BOUNDED_SURFACE`, …) — DECLINE.
- **Any reduced Cone / Plane / Sphere face that fails the faithful-reduction guard or the watertight
  self-verify** (e.g. an apex-reaching cone with a degenerate collapsed seam) — DECLINE (never widened
  to pass).
- **IGES** import/export — stay OCCT `IGESControl_*`. **#8 `drop-occt`** stays blocked.

## 8. Cognitive complexity

The two new `revolvedLine` arms add a few guarded branches to an existing ≤ ~12 function (the
common-perpendicular / apex-intersection test is the densest piece — isolated in a small
`lineMeetsAxis(P, D̂, L, Â) → optional<apex>` helper, ≤ ~10). `revolvedCircle` mirrors `revolvedLine`
(a centre-on-axis + plane-contains-axis classification, ≤ ~10). The `surfaceOfRevolution` dispatch
is a 3-way switch (≤ ~6). The faithful-reduction guard is a small per-kind point-on-surface check
(≤ ~8). All measured with the `cognitive-complexity` skill before archive; nothing pushed to a higher
band (parser/systems 25–35).

## 9. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - **R1 cone** — a `SURFACE_OF_REVOLUTION` of a `LINE` oblique to (and meeting) the axis → a
    `Cone`(apex on axis, semiAngle = ∠(line,axis)); the reconstructed frustum solid is valid +
    watertight and matches the `CONICAL_SURFACE`-keyword-equivalent solid.
  - **R2 plane** — a `SURFACE_OF_REVOLUTION` of a `LINE` ⟂ axis → a `Plane` (flat annulus); the
    reconstructed face is valid + watertight and matches the `PLANE`-keyword-equivalent face.
  - **R3 sphere** — a `SURFACE_OF_REVOLUTION` of a semicircle whose centre is on the axis and whose
    plane contains the axis → a `Sphere`(radius = circle radius); the reconstructed solid is valid +
    watertight and matches the `SPHERICAL_SURFACE`-keyword-equivalent solid.
  - **R4 declines** — an OFF-axis circle (torus), an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` profile,
    and a SKEW oblique line (hyperboloid) each DECLINE (NULL), like `TOROIDAL_SURFACE`.
  - **No regression** — the parallel-line → cylinder case, and the single-solid, flat multi-solid,
    placed rigid / uniform-scale / mirror assembly, AP242, trimmed-curve, quadric, and bspline-face
    round-trip cases STILL pass.
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - **(A) cone parity** — OCCT authors a CONE solid (oblique-line `SURFACE_OF_REVOLUTION`); native
    `cc_step_import` imports it; OCCT re-imports; assert same solid **count / volume / watertight /
    bbox** within tolerance.
  - **(B) plane parity** — OCCT authors a solid with a revolved-to-PLANE face (perpendicular-line
    revolution); native import vs OCCT re-import agree on count / volume / watertight / bbox.
  - **(C) sphere parity** — OCCT authors a SPHERE solid (semicircle revolution about its diameter);
    native import vs OCCT re-import agree on count / volume / watertight / bbox.
  - **(D) torus decline** — OCCT authors a torus (off-axis circle) `SURFACE_OF_REVOLUTION`; native
    `cc_step_import` DECLINES → OCCT and matches `cc_set_engine(0)`.
  - **(E) ellipse / B-spline decline** — OCCT authors an ellipse- or B-spline-profile
    `SURFACE_OF_REVOLUTION`; native DECLINES → OCCT and matches `cc_set_engine(0)`.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown so the
    suite assertion count is unchanged.
- **Done** only when the relevant gates are green and every existing suite stays green at the OCCT
  default (prior import slices host round-trip + sim `[NIMPORT]` 53/53 incl. trimmed-curve +
  revolution-cylinder + honest torus/general declines, STEP export, healing, SSI S1–S5, native blends
  + #6/#7, marching, boolean, construct, tessellation, phase3 do NOT regress). Honestly reported:
  this adds `SURFACE_OF_REVOLUTION`-face import onto native `Cone` (oblique line meeting the axis),
  `Plane` (perpendicular line), and `Sphere` (on-axis circle) — extending the landed cylinder
  reduction — with a faithful-reduction + watertight self-verify gate; a torus revolution, an
  ellipse / B-spline revolution, a skew-line hyperboloid, and arbitrary surfaces stay OCCT; #8
  `drop-occt` stays blocked.
