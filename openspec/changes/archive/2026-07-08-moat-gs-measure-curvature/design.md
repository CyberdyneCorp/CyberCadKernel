# Design — moat-gs-measure-curvature (MOAT M-GS, GS3 + GS4)

Expose two exact, OCCT-free analysis SERVICES on the landed native evaluators:
**minimum distance / angle between B-rep entities** (GS3) and **surface / edge
curvature at a parameter point** (GS4). Clean-room differential geometry (do Carmo;
Piegl & Tiller for the NURBS derivative evaluation, already native in
`math/bspline.h`); OCCT (`BRepExtrema_DistShapeShape`, `BRepLProp_SLProps`,
`GeomLProp_SLProps`) is the **oracle only**, in the sim harness — never linked into
`src/native/**`.

## 0. The substrate this consumes (verified in source)

- `math/elementary.h` — `Plane`/`Cylinder`/`Cone`/`Sphere` with `value(u,v)`,
  `normal(u,v)`; `math/torus.h::Torus::value(u,v)`. Analytic frames are `Ax3`.
- `math/bspline.h` — `surfaceDerivs(...,maxDeriv,out)` / `nurbsSurfaceDerivs(...)`
  fill a row-major `(maxDeriv+1)²` grid of `Vec3` with `∂^(k+l)S/∂u^k∂v^l`;
  `curveDerivs` / `nurbsCurveDerivs`; `surfaceNormal` (rational-aware).
- `numerics/closest_point.h` — `polishCurveParam` / `polishSurfaceParam` (Newton on
  `½‖·−T‖²`, clamped to the domain) and the typed `CurveClosest` / `SurfaceClosest`
  projection results; OCCT-`Extrema`-verified 22/22 in `add-native-numerics`.
- `topology/shape.h` — `EdgeCurve{kind,frame,radius,minorRadius,degree,poles,
  weights,knots}`, `FaceSurface{kind,frame,radius,semiAngle,minorRadius,degreeU/V,
  nPolesU/V,poles,weights,knotsU/V}`, `Shape` graph + sub-shape ids.

Nothing here needs new numeric primitives — GS3/GS4 are algebra on the above.

## 1. GS4 curvature (the simplest, land first)

### Surface curvature `analysis::surfaceCurvature(FaceSurface, u, v) → {K,H,k1,k2} | decline`

Two arms, both exact:

- **Analytic closed form** (no derivative evaluation, exact):
  - `Plane`: `K=0, H=0, k1=k2=0`.
  - `Sphere` (radius `R`): `K=1/R²`, `H=1/R` (sign by outward normal convention),
    `k1=k2=1/R`.
  - `Cylinder` (radius `R`): `K=0`, principal `{1/R, 0}`, `H=1/(2R)`.
  - `Cone` (half-angle `α`, distance `d` from apex along axis): `K=0`, one principal
    `0`, the other `cos α /(d)`-family; `H` = half that. Declines within `ε` of the
    apex (`d→0`, curvature blows up).
  - `Torus` (major `R`, minor `r`): `K = cos v /(r (R + r cos v))`,
    `H = (R + 2 r cos v)/(2 r (R + r cos v))`; principal `{1/r, cos v/(R+r cos v)}`.
- **NURBS arm** (`Kind::BSpline`/`Bezier`) via the fundamental forms:
  1. `surfaceDerivs`/`nurbsSurfaceDerivs` with `maxDeriv=2` → `S_u,S_v,S_uu,S_uv,S_vv`.
  2. First form `E=S_u·S_u, F=S_u·S_v, G=S_v·S_v`; `W = EG−F²`.
     **If `W ≤ ε·max(E,G)²` → DECLINE** (parametric singularity — e.g. a pole).
  3. Unit normal `n = (S_u×S_v)/‖S_u×S_v‖`.
  4. Second form `L=S_uu·n, M=S_uv·n, N=S_vv·n`.
  5. `K=(LN−M²)/W`, `H=(EN−2FM+GL)/(2W)`, `k1,k2 = H ± √(max(0,H²−K))`.

The `k1≥k2` ordering and the sign of `H` follow the outward-normal convention so the
result matches `BRepLProp_SLProps` (which uses the face normal). A `Reversed` face
flips `n`, hence the sign of `H`, `k1`, `k2` (and leaves `K` unchanged) — handled at
the facade where orientation is known.

### Edge curvature `analysis::edgeCurvature(EdgeCurve, t) → κ`

- `Line`: `0`. `Circle` (radius `R`): `1/R`. `Ellipse`: `‖C′×C″‖/‖C′‖³` closed form.
- `BSpline`/`Bezier`: `curveDerivs`/`nurbsCurveDerivs` `maxDeriv=2` →
  `κ = ‖C′×C″‖ / ‖C′‖³`. **If `‖C′‖ ≤ ε` → DECLINE** (a stationary/cusp point).

## 2. GS3 distance `analysis::minDistance(EntityA, EntityB) → {d, p1, p2} | decline`

Entity = a resolved `Point3` (vertex), `EdgeCurve` (+ param range), or `FaceSurface`
(+ trim). Dispatch by the two kinds; each cell is either a closed form or a
seed-and-refine:

- **Closed form (analytic·analytic):**
  - point·point, point·line/segment (clamp to range), point·circle, point·plane,
    point·cylinder, point·sphere;
  - line·line: parallel → point-to-line; skew → the common-perpendicular formula,
    each foot clamped to its segment range (clamped pairs re-minimized on the
    boundary, exactly as `DistShapeShape` reports the *bounded* distance).
- **Seed-and-refine (any NURBS curve/surface involved):** evaluate the NURBS entity
  on a coarse deterministic parameter sample (grid sized by degree × span), project
  each sample onto the OTHER entity (closed form when the other is analytic, else its
  own coarse sample), keep the best seed, then alternate `numerics/closest_point`
  Newton polish on each side until the witness pair converges. Return the global
  minimum + both witness points.
- **Trim awareness:** a witness on a face must lie inside the face's trim region
  (the existing even-odd `UVRegion::inside` test); a witness on an edge must lie in
  its param range. If the unconstrained optimum is outside the trim, the minimizer
  restarts constrained to the boundary loop.

**Decline** when the pair involves a genuinely-trimmed freeform patch and the seeded
minimizer cannot certify the global optimum (multiple comparably-deep basins within
the seed resolution, or a boundary-constrained restart that does not converge) — the
service returns "decline", never a guessed minimum.

## 3. GS3 angle `analysis::angle(EntityA, EntityB) → θ | decline`

- line·line: `θ = acos(|d_a·d_b|)` ∈ `[0,π/2]` (unsigned; parallel → 0).
- plane·plane: `θ = acos(clamp(n_a·n_b,−1,1))` ∈ `[0,π]` (oriented normals);
  the app may take the `π−θ` supplement for the dihedral it wants.
- line·plane: `θ = asin(|d·n|)` ∈ `[0,π/2]` (angle to the plane, 0 = in-plane).
- Any entity that is not a `Line`/`Plane` (a general curve or curved surface has no
  single angle) → DECLINE. Degenerate direction (`‖d‖≈0`) → DECLINE.

## 4. Facade + engine dispatch

Four ADDITIVE `cc_*` prototypes (see proposal). `subKind` selects VERTEX/EDGE/FACE;
`subId` is the existing sub-shape id (`cc_subshape_ids` numbering). The engine
resolves the id to the native leaf geometry, calls `analysis::*`, and marshals the
out-array. Return `1` = success, `0` = honest decline with `cc_last_error` set. The
POD/ABI contract test (`CC_KERNEL_NO_PROTOTYPES`) proves every pre-existing struct
and signature is byte-identical.

## 5. Verification (two gates, no weakened tolerances)

- **GATE A — HOST ANALYTIC (no OCCT):** closed-form oracles compiled `clang++
  -std=c++20`: point/line/circle/plane distances hand-derived; line·line angle;
  sphere `K=1/R²`, cylinder `K=0 ∧ H=1/(2R)`, plane `0`, torus
  `K=cos v/(r(R+r cos v))`, edge circle `κ=1/R`. Tolerance `1e-9` (scale-relative).
- **GATE B — SIM native-vs-OCCT:** on a booted iOS simulator, native
  `cc_measure_distance` vs `BRepExtrema_DistShapeShape` (min distance AND witness
  points), native `cc_surface_curvature` vs `BRepLProp_SLProps` (Gaussian/mean/
  principal), native `cc_edge_curvature` vs `GeomLProp` — over analytic + simple-
  NURBS fixtures. Scale-relative tolerance; the declined fixtures assert a clean
  decline, not a compared number.

## 6. Risks / honest declines

- **Freeform-trimmed global minimum** is the genuine hard case; the seeded minimizer
  is robust for smooth simple patches but not certifiable on a wavy trimmed patch —
  those DECLINE. This is the expected boundary of this LOW-RISK slice.
- **Parametric singularities** (sphere pole, cone apex) give `EG−F²→0`; curvature is
  undefined in that chart → DECLINE rather than emit a blown-up number.
- **Orientation sign:** `H`/`k1`/`k2` sign depends on the face normal; the facade
  flips for a `Reversed` face so the SIM gate matches OCCT's face-normal convention.
