# Design — add-native-step-general-revolution

Close the LAST `SURFACE_OF_REVOLUTION` gap the `add-native-step-torus` slice deferred: an `ELLIPSE` /
`B_SPLINE_CURVE_WITH_KNOTS` generatrix `SURFACE_OF_REVOLUTION` (**T2**) → a native **rational
`FaceSurface::Kind::BSpline`** surface — the exact revolved rational tensor-product B-spline — behind an
HONEST watertight gate, or an honest DECLINE with NO dead code and NO tessellator perturbation. Clean-room
from ISO 10303-42/-43 + Piegl & Tiller (A7.1 `MakeRevolvedSurface`) + the existing reader; OCCT
(`STEPControl_Reader` / `STEPControl_Writer`) is the ORACLE + fixture-author + fallback only. Map ONLY
geometry the file exactly describes; VERIFY the reduction passes through the profile; DECLINE (NULL → OCCT)
wherever the watertight bar is not met.

## 0. What the reader already does (the substrate this widens)

`add-native-step-revolution-quadrics` + `add-native-step-torus` (archived) left `surfaceOfRevolution`
dispatching by profile kind — `Line → revolvedLine` (cyl/cone/plane), `Circle → revolvedCircle`
(on-axis → Sphere, off-axis → Torus), and an `Ellipse` / `BSpline` profile falling through to DECLINE:

```cpp
std::optional<topo::FaceSurface> surfaceOfRevolution(const Record& r) {        // step_reader.cpp ~L979
  const auto profile = curve(r.args[1].ref);
  const auto ax = axis1placement(r.args[2].ref);
  if (!profile || !ax) return std::nullopt;
  using K = topo::EdgeCurve::Kind;
  if (profile->kind == K::Line)   return revolvedLine(*profile, ax->first, ax->second);
  if (profile->kind == K::Circle) return revolvedCircle(*profile, ax->first, ax->second);
  return std::nullopt;   // ← T2 replaces: Ellipse / BSpline → revolvedProfileBSpline
}
```

**Exact facts this design leans on (verified in the source):**

- `FaceSurface::Kind` is `{Plane,Cylinder,Cone,Sphere,BSpline,Bezier,Torus}` (`shape.h` ~L210) and
  `FaceSurface` carries `int degreeU,degreeV; int nPolesU,nPolesV; std::vector<Point3> poles;
  std::vector<double> weights; std::vector<double> knotsU,knotsV;` (~L219–226). **`weights` empty ⇒
  non-rational** — so `Kind::BSpline` is RATIONAL-capable. **No topology change is needed.**
- `math::nurbsSurfacePoint` / `math::nurbsSurfaceDerivs` (`math/bspline.h` ~L113/L120) exist and
  `tessellate/surface_eval.h` (~L163–172, L216–220) ALREADY dispatches `Kind::BSpline` to the RATIONAL
  evaluators when `weights` is non-empty. **A rational B-spline surface EVALUATES through the existing,
  unmodified mesh path.** This is the pivotal check — and it PASSES on the evaluation substrate.
- The reader's `curve()` parses `LINE`, `CIRCLE`, `ELLIPSE`, and **non-rational** `B_SPLINE_CURVE_WITH_KNOTS`
  (`bsplineCurve`, ~L891, with the `knots.size() == poles + degree + 1` non-rational guard at ~L906).
  So the profiles that can reach `revolvedProfileBSpline` are `ELLIPSE`, `CIRCLE` (already reduced), and a
  **non-rational** B-spline — i.e. `w^v_j = 1` for every B-spline profile pole; an ellipse promotes to a
  KNOWN rational quadratic.
- The reader synthesises analytic pcurves per face; a FULL periodic analytic surface with a VERTEX_LOOP /
  all-seam bound is reconstructed as a **bare-surface face** (NULL outer wire) for `Kind::Sphere` and
  `Kind::Torus` — but there is **no bare-periodic path for `Kind::BSpline`**: a freeform face meshes as an
  open `(u,v)` grid. This is the crux (see §3).

## 1. The native capability this maps onto (checked)

| STEP concept (this slice) | Geometric reduction | Native construct | Exists? | Gate |
|---|---|---|---|---|
| `ELLIPSE` generatrix | Revolved rational tensor B-spline (spheroid) | `FaceSurface::Kind::BSpline` (rational) | **YES — eval path exists** | watertight self-verify |
| non-rational `B_SPLINE` generatrix | Revolved rational tensor B-spline | `FaceSurface::Kind::BSpline` (rational) | **YES — eval path exists** | watertight self-verify |
| rational (weighted) `B_SPLINE` generatrix | — (curve reader is non-rational only) | — | NO → DECLINE upstream | unchanged |
| `CIRCLE` on/off axis, `LINE` ∥/oblique/⟂ | Sphere/Torus/Cyl/Cone/Plane (landed) | existing | YES (landed) | unchanged |
| skew line | Hyperboloid | — | NO → DECLINE | unchanged |

## 2. The exact revolved rational tensor-product B-spline (Piegl & Tiller A7.1)

Revolving a profile curve `P(v)` (with control poles `Q_j`, weights `ω_j`, degree `p_v`, knots `V`) a full
`2π` about an axis `(L, unit A)` yields the tensor-product NURBS surface

```
        Σ_i Σ_j  N_{i,2}(u) · N_{j,p_v}(v) · w_ij · P_ij
S(u,v) = ───────────────────────────────────────────────
        Σ_i Σ_j  N_{i,2}(u) · N_{j,p_v}(v) · w_ij
```

whose **`u` direction is the standard rational-quadratic full circle** and whose **`v` direction is the
profile**.

### 2.1 The `u` circle (standard revolution knots + weights)
- `degreeU = 2`, **9** control poles `P_0…P_8`.
- Standard rational weights over the 9 poles: `{1, 1/√2, 1, 1/√2, 1, 1/√2, 1, 1/√2, 1}` — the five
  **on-circle** poles (at the quadrant angles) carry weight `1`; the four in-between **corner** poles carry
  `cos(Δ/2) = cos(45°) = 1/√2` (`Δ = π/2` per quadrant segment). *(This is the task's stated schedule: the
  distinct quadrant nodes `0, π/2, π, 3π/2, 2π` carry weight `1`; the corner poles between them carry
  `1/√2` — combined into the 9-pole weight vector above.)*
- Standard knot vector `knotsU = {0,0,0, π/2,π/2, π,π, 3π/2,3π/2, 2π,2π,2π}` (12 knots = 9 poles + 2 + 1).

### 2.2 The revolution-circle poles at each profile pole (`revolutionCirclePoles`)
For a profile pole `Q_j`, decompose it in the axis frame into an axial height `h_j = A·(Q_j − L)` and a
radial vector `ρ_j = (Q_j − L) − h_j·A` of length `r_j = |ρ_j|`, with radial unit `X_j = ρ_j / r_j` (for
`r_j > 0`) and `Y_j = A × X_j`. The 9 revolution-circle control poles about the axis foot
`O_j = L + h_j·A` are the classic circle control polygon:

```
on-circle pole at angle θ_k (k = 0..4, θ = 0,π/2,π,3π/2,2π):  O_j + r_j·(cos θ_k·X_j + sin θ_k·Y_j)
corner pole between θ_k and θ_{k+1} (diagonal 45°):           O_j + (r_j/cos45°)·(cos φ·X_j + sin φ·Y_j)
```

with the corner at the bisector angle `φ = θ_k + 45°` and "radius" `r_j·√2` (so the corner pole's weight
`1/√2` restores the true circle). A pole with `r_j = 0` (a profile point **on the axis**) degenerates all 9
poles to `O_j` — a **pole singularity** in the surface (see §3).

### 2.3 The profile in `v` (`promoteProfileToNurbs`)
- an **`ELLIPSE`** (semi-axes `a, b`, frame `X,Y`) is promoted to its exact rational-quadratic B-spline —
  the SAME 9-pole / `{1,1/√2,…}` structure as the circle but scaled `a` in `X`, `b` in `Y` (a circle is the
  `a=b` case); an ELLIPTIC ARC uses the covered sub-arc's poles/knots;
- a **non-rational `B_SPLINE_CURVE_WITH_KNOTS`** is used directly: `degreeV = degree`, `knotsV = knots`,
  poles `= poles`, `w^v_j = 1`.

### 2.4 Assembly (`revolvedProfileBSpline`)
```
degreeU = 2;  degreeV = p_v;  nPolesU = 9;  nPolesV = n_v;
poles[i*nPolesV + j]   = revolutionCirclePoles(Q_j)[i]        // row-major, U outer (matches gridOf/bsplineSurface)
weights[i*nPolesV + j] = w^u_i · ω_j
knotsU = {0,0,0, π/2,π/2, π,π, 3π/2,3π/2, 2π,2π,2π};  knotsV = V;
FaceSurface{ Kind::BSpline, degreeU, degreeV, nPolesU, nPolesV, poles, weights, knotsU, knotsV }
```
Row-major `U`-outer matches `detail::gridOf` (`surface_eval.h` ~L140) and the reader's `bsplineSurface`
pole layout — so the tessellator's existing `nurbsSurfacePoint` evaluates it with no adaptation.

## 3. Watertightness — the pivotal, must-prove gate (the honest crux)

A full revolution is **`u`-periodic** (a seam at `u=0≡2π`) and, if the profile touches the axis (some
`r_j=0`), has a **degenerate pole**. Unlike `Sphere` / `Torus` — which the reader reconstructs as
analytic **bare-periodic** faces welded by the mesher's canonical-seam-anchor + pole machinery — a
`Kind::BSpline` face is meshed by the **freeform** grid path, which today is exercised ONLY on
**non-periodic, pole-free** faces. The open questions this change MUST answer empirically (Task 1):

1. **Does the `u`-seam weld watertight?** The standard NURBS full-circle closes exactly at `u=0≡2π`
   (`P_8 = P_0`, `w_8 = w_0`), so the evaluated surface points coincide at the seam. Whether the mesher's
   freeform grid emits coincident seam vertices that WELD (shared edge) or leaves a T-gap is the gate.
2. **Does a profile-endpoint axis pole close?** A spheroid's ellipse profile with an endpoint on the axis
   collapses that `v`-row to a single point — a degenerate-triangle fan. Whether the freeform path collapses
   it watertight (like the sphere pole) or leaves a hole is the gate.

**Outcomes, honest per branch:**
- **If the existing freeform path already welds the seam + closes the pole watertight** (proven by the
  engine self-verify + a host volume/watertight assertion), T2 LANDS with **zero tessellator change** — the
  ideal, mirroring the torus slice that landed through an already-proven path.
- **If it does NOT, but a periodic-seam / pole close can be added STRICTLY ADDITIVELY to the mesher with a
  PROVEN byte-identical result for every existing mesh** (the `add-native-step-torus` bar), T2 may land
  through that additive branch — gated by the full tessellation-sensitive zero-regression proof.
- **Otherwise T2 keeps the OCCT DECLINE** — `revolvedProfileBSpline` returns `nullopt`, no reconstruction
  code is left reachable-but-broken, and the ellipse / B-spline revolution imports via OCCT (fine, nothing
  lost). **No dead code; no perturbed tessellation; no weakened tolerance.**

`profileOnSurface` (sampled profile points `P(v_k)` lie on `S(0, v_k)` within a scale-relative tolerance)
is the pre-emission faithful-reduction guard; `robustlyWatertightImport` is the FINAL arbiter. Neither
tolerance is widened.

## 4. The entity (ISO 10303-42)

```
#sor    = SURFACE_OF_REVOLUTION('', #profile_curve, #axis1);
#axis1  = AXIS1_PLACEMENT('', #origin_point, #axis_direction);      // origin + one dir; $ ⇒ +Z
#profile= ELLIPSE('', #position, semi_axis_1, semi_axis_2);          // or B_SPLINE_CURVE_WITH_KNOTS(...)
```
`axis1placement` (already present) resolves `(origin, axis)`; `curve()` (already present) resolves the
profile to an `EdgeCurve` of kind `Ellipse` / `BSpline`.

## 5. Architecture / OCCT boundary (unchanged)

```
cc_step_import (facade, unchanged)
   └─ active engine
        ├─ NativeEngine::step_import(path)
        │     ├─ step_import_native(path)  (src/native/exchange, OCCT-FREE) → topo::Shape
        │     │     • surfaceOfRevolution: ellipse/bspline → revolvedProfileBSpline → rational Kind::BSpline
        │     │                            (on-axis circle → Sphere | off-axis circle → Torus |
        │     │                             line → cyl/cone/plane — all landed, unchanged)
        │     │     • non-representable / non-watertight / out-of-scope → NULL (DECLINE)
        │     ├─ NULL → OCCT fall-through
        │     └─ Solid/Compound → robustlyWatertightImport (per-member, volume>0) → wrap native
        │             └─ leaky revolved-B-spline (seam/pole gap) → OCCT
        └─ OCCT STEPControl_Reader (fallback + oracle + fixture author)
   cc_iges_* / cc_step_export → unchanged
```
`src/native/**` stays OCCT-free (grep-gated). The tessellator is preferred UNCHANGED (the rational
B-spline eval path already exists); any mesher change is ADDITIVE-ONLY + PROVEN byte-identical or T2
declines. `step_writer.cpp` is unchanged (OCCT-authored fixtures). No `cc_*` ABI change; default engine
stays OCCT (opt-in `cc_set_engine(1)`).

## 6. Honest scope (outcomes)

- **T2 lands** iff the revolved rational B-spline is faithfully representable AND self-verifies watertight
  through the existing (or provably additive) mesh path. **Else T2 keeps the OCCT decline** for ellipse /
  B-spline revolution — no dead code, no perturbed tessellation.
- Unchanged declines: hyperboloid (skew line), rational (weighted) B-spline profile (curve reader is
  non-rational), directly-authored arbitrary rational B-spline surface, general swept / bounded / offset
  surfaces, IGES. **#8 `drop-occt`** stays blocked.

## 7. Cognitive complexity

`revolvedProfileBSpline` is the densest piece (circle × profile tensor assembly) — isolated in its own
function with `revolutionCirclePoles` (the 9-pole circle polygon at one profile pole) and
`promoteProfileToNurbs` (ellipse → rational quadratic; B-spline pass-through) extracted, targeted ≤ ~20
(parser/systems band). `profileOnSurface` mirrors `circleOnSurface` (≤ ~8). The `surfaceOfRevolution`
dispatch adds one branch (≤ ~6). All measured with the `cognitive-complexity` skill before archive; nothing
pushed past the parser/systems band.

## 8. Verification plan

- **Gate 1 (host, OCCT-free)** — `tests/native/test_native_step_reader.cpp`:
  - an ELLIPSE (and, if authorable, a non-rational B-spline) `SURFACE_OF_REVOLUTION` maps to a rational
    `Kind::BSpline`, VERIFIED the profile lies on the surface (`profileOnSurface`); IF T2 lands the
    reconstructed solid is valid + watertight and matches the analytic spheroid volume/bbox; ELSE the
    reduction is host-asserted and the end-to-end path DECLINES (NULL).
  - **No regression** — on-axis circle → Sphere, off-axis circle → Torus, line → cylinder/cone/plane,
    trimmed-curve, quadric, bspline-face, single/flat/placed-assembly, AP242 round-trips STILL pass.
- **Gate 2 (sim vs OCCT, OCCT linked)** — `tests/sim/native_step_import_parity.mm` +
  `scripts/run-sim-native-step-import.sh` via `cc_*` under `cc_set_engine(1)`;
  `xcrun simctl list devices booted` first:
  - OCCT authors an ELLIPSE-profile revolution (a spheroid) and, if emitted, a B-spline-profile revolution;
    if T2 lands, native import matches the OCCT re-import (count / volume / watertight / bbox); else native
    DECLINES → OCCT and matches `cc_set_engine(0)`.
  - Own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in teardown.
- **Gate 3 (tessellation zero-regression PROOF, booted sim) — ONLY IF the mesher was touched.** If a
  periodic-seam / axis-pole close required an additive mesher branch: `run-sim-suite`, curved-fillet,
  curved-chamfer, curved-boolean, wrap-emboss, and phase3 all green with **identical** triangle counts,
  watertight status, and volumes for every existing face. If the mesher was NOT touched (preferred),
  existing tessellation is byte-identical by construction and this gate is a no-op re-run for assurance.
- **Done** only when the relevant gate is green and every existing suite stays green at the OCCT default
  (landed import sim `[NIMPORT]` 69/69, STEP export, healing, SSI S1–S5, native blends + #6/#7, marching,
  boolean, construct, tessellation, curved-fillet/chamfer/boolean, wrap-emboss, phase3 do NOT regress).
  Reported honestly: T2 either adds a native revolved rational B-spline or keeps the OCCT decline. #8
  `drop-occt` stays blocked.
