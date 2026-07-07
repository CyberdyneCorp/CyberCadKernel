# Design — add-native-step-torus

Close the LAST STEP-surface gap in the revolution family — two tracks, each behind an **honest per-track
gate**. **T1**: an OFF-AXIS `CIRCLE` / arc `SURFACE_OF_REVOLUTION`, and the direct `TOROIDAL_SURFACE`
keyword, → a **new native `FaceSurface::Kind::Torus`** (topology + an ADDITIVE tessellator torus mesh
path + STEP mapping). **T2**: an `ELLIPSE` / `B_SPLINE_CURVE_WITH_KNOTS` generatrix `SURFACE_OF_REVOLUTION`
→ a native **rational `Kind::BSpline`** surface (the exact revolved rational tensor-product B-spline), or
an honest DECLINE. Clean-room from ISO 10303-42/-43 + the existing reader; OCCT (`STEPControl_Reader` /
`STEPControl_Writer`) is the ORACLE + fixture-author + fallback only. Map ONLY geometry the file exactly
describes; VERIFY each reduction passes through the profile; DECLINE (NULL → OCCT) wherever the additive +
watertight bar is not met.

## 0. What the reader already does (the substrate this widens)

`add-native-step-revolution-quadrics` (archived) left `surfaceOfRevolution` dispatching by profile kind
and `revolvedCircle` reducing an ON-axis circle → `Sphere`, DECLINING an off-axis circle:

```cpp
std::optional<topo::FaceSurface> surfaceOfRevolution(const Record& r) {        // step_reader.cpp ~L958
  const auto profile = curve(r.args[1].ref);
  const auto ax = axis1placement(r.args[2].ref);
  if (!profile || !ax) return std::nullopt;
  using K = topo::EdgeCurve::Kind;
  if (profile->kind == K::Line)   return revolvedLine(*profile, ax->first, ax->second);
  if (profile->kind == K::Circle) return revolvedCircle(*profile, ax->first, ax->second);
  return std::nullopt;   // ← T2 replaces: Ellipse / BSpline → revolvedProfileBSpline
}

std::optional<topo::FaceSurface> revolvedCircle(const EdgeCurve& circle, Point3 L, Dir3 A) {  // ~L1048
  const double R = circle.radius;
  const Point3 C = circle.frame.origin;
  const Point3 footC = L + A·dot(C−L, A);
  if (distance(C, footC) > tol) return std::nullopt;                // ← T1 replaces: off-axis → Torus
  if (fabs(dot(circle.frame.z, A)) > tol) return std::nullopt;      // plane ∦ axis
  FaceSurface s; s.kind = Sphere; s.frame = Ax3::fromAxisAndRef(C, A, circle.frame.z); s.radius = R;
  if (!circleOnSurface(circle, s)) return std::nullopt;
  return s;
}

std::optional<topo::FaceSurface> surface(long id) {                            // ~L913
  if (PLANE) …; if (CYLINDRICAL_SURFACE) …; if (SPHERICAL_SURFACE) …;
  if (CONICAL_SURFACE) …; if (B_SPLINE_SURFACE_WITH_KNOTS) …;
  if (SURFACE_OF_REVOLUTION) return surfaceOfRevolution(*r);
  return std::nullopt;   // ← T1 adds: TOROIDAL_SURFACE → toroidalSurface(*r)
}
```

**Exact facts this design leans on (verified in the source):**

- `FaceSurface::Kind` is `{Plane,Cylinder,Cone,Sphere,BSpline,Bezier}` (`shape.h` ~L210) — **NO Torus**.
  `FaceSurface` carries `radius` (Cyl/Sphere radius, Cone reference radius) and `semiAngle` (Cone) — **no
  second radius**. A torus needs BOTH a major and a minor radius.
- `math::Torus` (`math/torus.h`) ALREADY exists: `value/dU/dV/normal`, `majorRadius R`, `minorRadius r`,
  OCCT-`ElSLib`-parametrized `S(u,v)=O+(R+r·cos v)(cos u·X+sin u·Y)+r·sin v·Z`, `u,v∈[0,2π)`. Used by SSI.
  So the MATH is done; only the topology **kind** + the tessellator path + the STEP mapping are missing.
- The tessellator's `SurfaceEvaluator` (`surface_eval.h`) is the ONE place the `FaceSurface` variant is
  switched (localValue / localD1 / bounds). The face mesher (`face_mesher.h`) classifies a face as
  **freeform** (`Bezier`/`BSpline`), **planar** (`Plane`), or **analytic-curved** (the rest —
  Cylinder/Cone/Sphere) and meshes each surface through the SAME `SurfaceEvaluator`-driven grid, welding
  seams via canonical anchors; the sphere's longitude seam + poles are handled by this path. The
  tessellator has been kept PRISTINE all session (a mesher change is high-blast-radius).
- The reader synthesises analytic pcurves per face (`pcurveFor` → `projectUV` inverse, `surfaceValue`
  forward) since STEP carries no pcurve; a FULL periodic surface with a VERTEX_LOOP (degenerate-pole) bound
  is reconstructed as a **bare-surface face** (null outer wire) for `Kind::Sphere` only (`step_reader.cpp`
  ~L1435), meshed watertight over natural bounds.
- The tessellator ALREADY meshes **rational** B-spline faces (`nurbsSurfacePoint` / `nurbsSurfaceDerivs`
  in `surface_eval.h`); only the READER's `bsplineSurface` restricts to non-rational, and no code authors a
  revolved B-spline surface.

## 1. The native capability this maps onto (checked)

| STEP concept (this slice) | Geometric reduction | Native construct | Exists? | Track |
|---|---|---|---|---|
| `CIRCLE`/arc, centre OFF axis | Torus (R=axis dist, r=circle radius) | `FaceSurface::Kind::Torus` (NEW) | **NO → build (T1)** | new kind + mesh path |
| `TOROIDAL_SURFACE` keyword | Torus (major, minor radii) | `FaceSurface::Kind::Torus` (NEW) | **NO → build (T1)** | direct keyword arm |
| `ELLIPSE` / `B_SPLINE` generatrix | Revolved rational tensor B-spline | `FaceSurface::Kind::BSpline` (rational) | **YES (T2)** | reader authors surface |
| `CIRCLE`, centre ON axis | Sphere (landed) | `FaceSurface::Kind::Sphere` | YES (landed) | unchanged |
| `LINE` ∥/oblique/⟂ (landed) | Cylinder/Cone/Plane | existing | YES (landed) | unchanged |
| skew line | Hyperboloid | — | NO → DECLINE | unchanged |

## 2. T1 — new additive `Kind::Torus`

### 2.1 Topology (`shape.h`) — additive, zero-perturbation
- `enum class Kind : uint8_t { Plane, Cylinder, Cone, Sphere, BSpline, Bezier, Torus };` — `Torus`
  **appended** so no existing enumerator's ordinal changes.
- `double minorRadius = 0.0;` added to `FaceSurface`. A torus uses `radius` = **major** radius R and
  `minorRadius` = **minor** radius r → `math::Torus{frame, radius, minorRadius}`. Every existing kind
  leaves `minorRadius` at its default; their meaning of `radius`/`semiAngle` is unchanged. This is the
  ONLY struct change and it is purely additive.

### 2.2 Surface evaluator (`surface_eval.h`) — additive switch arms
```cpp
inline math::Torus asTorus(const topo::FaceSurface& s) noexcept {
  return math::Torus{s.frame, s.radius, s.minorRadius};
}
// localValue:  case K::Torus: return detail::asTorus(s_).value(u, v);
// localD1:     case K::Torus: { auto t = detail::asTorus(s_); return {t.value(u,v), t.dU(u,v), t.dV(u,v)}; }
// bounds:      case K::Torus: return {0.0, kTwoPi, 0.0, kTwoPi};   // periodic in BOTH u and v, NO poles
```
The existing arms are byte-unchanged; `curvatureMagnitude` (a central-difference of `localValue`) works
for the torus with no change (it already handles any kind through `localValue`).

### 2.3 Face mesher (`face_mesher.h`) — additive doubly-periodic path, NO pole
A torus is classified as analytic-curved (`k != Bezier && k != BSpline && k != Plane`), so it inherits the
EXISTING periodic-analytic grid + canonical-seam-anchor machinery unchanged. The additive work is that a
ring torus is periodic in BOTH directions with **NO degenerate pole** (unlike a sphere's two poles):
- The full-turn grid spans `u∈[0,2π]`, `v∈[0,2π]`; the `u=0≡2π` seam and the `v=0≡2π` seam each weld by
  the SAME canonical-anchor / shared-edge path the sphere longitude seam uses — but with two seams and no
  pole collapse, which is STRICTLY SIMPLER than the sphere (no degenerate-triangle fan at a pole).
- The mesh branch is a NEW guarded arm; it does NOT modify the Plane/Cylinder/Cone/Sphere/BSpline/Bezier
  code. The **zero-regression proof** (Gate 3) is the gate: every existing sphere/cyl/cone/plane/bspline
  mesh must be byte-identical (same tri counts, same watertight, same volumes) across the full
  tessellation-sensitive sim set.
- **Honest-out**: if a clean additive doubly-periodic seam weld cannot be achieved without perturbing an
  existing path (e.g. the seam-anchor machinery needs a shared change that alters sphere meshes), **T1 is
  dropped** and the torus DECLINE is kept — torus already imports via OCCT, nothing is lost.

### 2.4 Reader (`step_reader.cpp`) — mapping + projection + reconstruction
- **`surface()`**: add `if (r->keyword == "TOROIDAL_SURFACE") return toroidalSurface(*r);`.
  `toroidalSurface` reads `AXIS2_PLACEMENT_3D` + two trailing reals (major, minor) → `Kind::Torus`
  (mirrors `placedSurface(K::Sphere,1)` with `nRadii=2`, mapping arg[2]→`radius`, arg[3]→`minorRadius`).
- **`revolvedCircle`** off-axis branch: replace the `distance(C,footC)>tol → nullopt` decline with a torus
  build — `R_major = distance(C, footC)` (⊥-distance centre→axis), `r_minor = circle.radius`, frame origin
  = `footC` on the axis, Z = axis, X ref = the radial direction to `C`. Require the circle plane to CONTAIN
  the axis normal appropriately for a standard ring torus (the generatrix circle lies in a plane through
  the axis — the meridian plane); VERIFY via `torusOnSurface` before returning. A circle whose plane does
  not admit a ring torus → DECLINE.
- **`projectUV`**: `case K::Torus: { u = atan2(ly,lx); rc = hypot(lx,ly); v = atan2(lz, rc − s.radius); return {u,v}; }`.
- **`surfaceValue`** (guard forward-eval): `case K::Torus: return math::Torus{s.frame, s.radius, s.minorRadius}.value(u,v);`.
- **`pcurveFor`**: a torus `v`-const boundary (a rim, the full major circle) is a straight `u`-line in
  (u,v); a `u`-const boundary (a meridian tube circle) is a straight `v`-line; the doubly-periodic seams
  are the `u=0` and `v=0` lines — synthesised by projecting endpoints exactly as the sphere/cylinder arms
  do. Add the torus arms alongside the existing kinds.
- **Full-torus face reconstruction**: Task 1 empirically diagnoses how OCCT bounds a full `TOROIDAL_SURFACE`
  advanced face (a torus has NO pole, so it is NOT a single VERTEX_LOOP like a sphere — it is bounded by
  seam EDGE_LOOP(s)). The reconstruction closes it watertight ADDITIVELY: either the seam edges reconstruct
  a normal trimmed periodic face, or — if OCCT emits a bare doubly-periodic face — a doubly-periodic
  bare-surface face path analogous to the sphere bare-surface path (extended to two seams, no pole). If
  neither closes watertight within the additive budget (no writer/tessellator change beyond the additive
  torus mesh path), **T1 keeps the OCCT decline** and reports it.
- **`torusOnSurface` guard**: four tube-quadrant points of the generatrix circle
  (`C + r(cos t·Xc + sin t·Yc)`, `t∈{0,π/2,π,3π/2}`) each lie on the candidate torus within a
  scale-relative tolerance (`pointOnSurface` via `projectUV`+`surfaceValue`); else DECLINE.

## 3. T2 — ellipse / B-spline revolution → native rational `Kind::BSpline`

The surface a profile curve `P(v)` sweeps by full revolution about an axis is the EXACT rational
tensor-product B-spline whose `u` direction is the **standard NURBS full circle** (a periodic /
clamped rational quadratic, control polygon the regular polygon of the swept radius, weights
`{1, w, 1, w, …}`, `w = cos(Δ/2)` per segment) and whose `v` direction is the profile's own
representation:
- a **`CIRCLE` / `ELLIPSE`** profile is first promoted to its exact rational-quadratic B-spline in `v`
  (an ellipse IS a rational quadratic; a circle is the `w=cos(Δ/2)` case);
- a **`B_SPLINE_CURVE_WITH_KNOTS`** profile is used directly in `v` (rational if it carries weights,
  else non-rational).
The tensor product's poles are `P_ij = C_i(v_j)` placed on the revolution circle at each profile pole,
weights `w_ij = w^u_i · w^v_j`. This is a rational `FaceSurface{Kind::BSpline, degreeU=2, degreeV=deg(profile),
poles, weights, knotsU (circle), knotsV (profile)}` — which the tessellator ALREADY meshes
(`nurbsSurfacePoint`). `revolvedProfileBSpline(profile, L, A)` builds it.

**Watertightness (the honest gate):**
- The `u` seam (`u=0≡2π`) must weld. The standard NURBS full-circle representation closes exactly at the
  seam; the reconstructed face reuses the periodic-B-spline seam weld the tessellator applies to any closed
  B-spline face. If it self-verifies non-watertight → DECLINE.
- If the profile **touches the axis** (an endpoint at ⊥-distance 0), that end of the revolution collapses
  to a **pole** (like a sphere/cone apex). If the pole cannot be closed watertight through the existing
  B-spline mesh path → DECLINE (no pole fabrication).
- `profileOnSurface` guard: sampled profile points `P(v_k)` lie on the reconstructed surface at `u=0`
  within a scale-relative tolerance before returning.

**Honest-out**: T2 emits the surface ONLY when representable AND self-verifying; otherwise the existing
`default → nullopt` DECLINE stays. An `Ellipse`/`BSpline` profile that revolves to a non-watertight or
non-representable surface remains OCCT — exactly the current behaviour, kept honest.

## 4. The entity (ISO 10303-42)

```
#tor   = TOROIDAL_SURFACE('', #axis2placement, major_radius, minor_radius);   // T1 direct
#sor   = SURFACE_OF_REVOLUTION('', #profile_curve, #axis1);                    // T1 off-axis circle / T2 profile
#axis1 = AXIS1_PLACEMENT('', #origin_point, #axis_direction);
```
`toroidalSurface` validates the two-real shape exactly as `placedSurface` does for a cone's two reals.

## 5. The faithful-reduction guard (never a mis-fit face)

`torusOnSurface` (T1) and `profileOnSurface` (T2) re-evaluate the candidate surface through the
generatrix's defining points and DECLINE on a scale-relative mismatch — the "never fabricate geometry"
gate. Then `robustlyWatertightImport` is the FINAL arbiter: any reconstructed face that leaves a gap →
DECLINE → OCCT. **No tolerance is widened**; the honest residual is a fall-through to OCCT.

## 6. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • surface(): TOROIDAL_SURFACE → toroidalSurface → Kind::Torus (T1)
        │     │     • surfaceOfRevolution: off-axis circle → Torus (T1) | ellipse/bspline → rational BSpline (T2)
        │     │                            on-axis circle → Sphere | line → cyl/cone/plane (landed, unchanged)
        │     │     • non-representable / non-watertight / out-of-scope → NULL (DECLINE)
        │     ├─ NULL → OCCT fall-through
        │     └─ Solid/Compound → robustlyWatertightImport (per-member, volume>0) → wrap native
        │             └─ any member fails (leaky torus / revolved-B-spline) → OCCT
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```
`src/native/**` stays OCCT-free (grep-gated). The tessellator changes are ADDITIVE-ONLY (a new torus mesh
branch) and PROVEN byte-identical for existing meshes; `step_writer.cpp` is preferred unchanged
(OCCT-authored fixtures). No `cc_*` ABI change; default engine stays OCCT (opt-in `cc_set_engine(1)`).

## 7. Honest scope (per-track outcomes)

- **T1 lands** iff `Kind::Torus` + the additive doubly-periodic torus mesh path + the torus face
  reconstruction close watertight AND the full tessellation-sensitive set is byte-identical. **Else T1
  keeps the OCCT decline** for `TOROIDAL_SURFACE` + off-axis-circle revolution.
- **T2 lands** iff the revolved rational B-spline is faithfully representable AND self-verifies watertight.
  **Else T2 keeps the OCCT decline** for ellipse / B-spline revolution.
- Unchanged declines: hyperboloid (skew line), directly-authored arbitrary rational B-spline surface,
  general swept / bounded / offset surfaces, IGES. **#8 `drop-occt`** stays blocked.

## 8. Cognitive complexity

`toroidalSurface` mirrors `placedSurface` (≤ ~6). The `revolvedCircle` off-axis torus arm adds a small
guarded branch (≤ ~10). `revolvedProfileBSpline` (T2) is the densest piece (circle×profile tensor
assembly) — isolated in its own function with helpers for the NURBS-circle poles/weights and the profile
promotion, targeted ≤ ~20 (parser/systems band). The `projectUV`/`surfaceValue`/`pcurveFor` torus arms are
one case each (≤ ~4). The torus mesh branch in `face_mesher.h` is a guarded arm ≤ ~12. All measured with
the `cognitive-complexity` skill before archive; nothing pushed past the parser/systems band.

## 9. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - **T1 torus** — a `TOROIDAL_SURFACE` face and an off-axis-circle `SURFACE_OF_REVOLUTION` map to
    `Kind::Torus` (major = axis distance, minor = circle radius), VERIFIED the circle lies on the torus; if
    T1 lands the reconstructed solid is valid + watertight and matches the OCCT-torus volume/bbox; else the
    reduction is host-asserted and the end-to-end declines.
  - **T2 general revolution** — an ellipse / B-spline `SURFACE_OF_REVOLUTION` maps to a rational
    `Kind::BSpline`, VERIFIED the profile lies on the surface; watertight if T2 lands, else DECLINE.
  - **No regression** — on-axis circle → Sphere, line → cylinder/cone/plane, trimmed-curve, quadric,
    bspline-face, single/flat/placed-assembly, AP242 round-trips STILL pass; `minorRadius`-default keeps
    every existing kind byte-identical.
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - **(A) torus** — OCCT authors a TORUS solid (off-axis-circle revolution / `TOROIDAL_SURFACE`); if T1
    lands, native import matches the OCCT re-import (count / volume / watertight / bbox); else native
    DECLINES → OCCT and matches `cc_set_engine(0)`.
  - **(B) general revolution** — OCCT authors an ellipse-profile revolution; if T2 lands, native import
    matches OCCT re-import; else DECLINES → OCCT and matches `cc_set_engine(0)`.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown.
- **Gate 3 (tessellation zero-regression PROOF, booted sim)** — the CRITICAL gate for the additive torus
  mesh path: `run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, and phase3 all
  green with **identical** triangle counts, watertight status, and volumes for every existing sphere /
  cylinder / cone / plane / B-spline face. If ANY existing mesh changes, T1's mesh path is not additive →
  drop T1, keep the torus DECLINE.
- **Done** only when the relevant gates are green and every existing suite stays green at the OCCT default
  (landed import sim `[NIMPORT]` 69/69, STEP export, healing, SSI S1–S5, native blends + #6/#7,
  marching, boolean, construct, tessellation, curved-fillet/chamfer/boolean, wrap-emboss, phase3 do NOT
  regress). Reported honestly per track: T1 either adds a native torus (with a byte-identical-proven mesh
  path) or keeps the OCCT decline; T2 either adds a native revolved rational B-spline or keeps the OCCT
  decline. #8 `drop-occt` stays blocked.
