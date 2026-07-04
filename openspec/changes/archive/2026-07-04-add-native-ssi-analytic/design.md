# Design — add-native-ssi-analytic (SSI Stage S1)

## Context

`SSI-ROADMAP.md` stages surface-surface intersection analytic-first. **S1** is the
closed-form slice: for the elementary surfaces the kernel already evaluates
(`src/native/math/`: `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus` — each giving
`value` / `dU` / `dV` / `normal`), the intersection of two of them is, for a
bounded family of pairs and placements, an **exact conic** (line, circle, ellipse,
parabola, hyperbola) or the roots of a **low-degree planar polynomial**
(plane∩torus quartic). No seeding, no marching, no generic Newton — that is S2–S4.

The method is **locked to CLEAN-ROOM**: derive each curve from the closed-form
geometry and `SSI-ROADMAP.md`, using OCCT (`IntPatch` ImpImp, `GeomAPI_IntSS`)
strictly as a verification **oracle**, never copied.

SSI is an **internal** capability. It is consumed by native booleans/blends (S5),
not exposed on the `cc_*` C ABI. Therefore it is verified at the SSI-function level
(native curves vs OCCT `GeomAPI_IntSS`) — the same discipline as native-math and
native-topology parity — with **no ABI change**.

## Goals / Non-Goals

**Goals**
- A native `cybercad::native::ssi` module: a pair-dispatch + closed-form handlers
  for the elementary conic family, returning native curve(s) on both surfaces.
- Every returned curve **self-verified** (sampled points on both surfaces within
  tol) before it is handed back.
- A typed **NOT-ANALYTIC** result (with a reason enum) as the honest deferral seam.
- OCCT-free; substrate (`native-numerics` polynomial roots) only for plane∩torus.

**Non-Goals (deferred to S2/S3/S4/OCCT — never faked here)**
- General/skew cylinder∩cylinder (quartic space curve), general cone∩cone, general
  (non-coaxial) cone∩cylinder.
- Torus∩curved (anything beyond plane∩torus), any NURBS/Bézier/B-spline/freeform.
- Near-tangent / coincident robustness (the S4 moat).
- Marching, seeding, B-spline fitting of a walked polyline (S2/S3).
- Any `cc_*` facade entry point or ABI change.

## Module shape

```
src/native/ssi/
  native_ssi.h        // aggregate public header + namespace doc
  ssi_result.h        // IntersectionCurve variant + NotAnalytic{reason} + result type
  dispatch.h/.cpp     // classify(pair, placement) → handler | NOT_ANALYTIC
  plane_pairs.h/.cpp  // plane∩{plane,sphere,cylinder,cone}
  plane_torus.cpp     // plane∩torus (planar quartic via native-numerics)  [CYBERCAD_HAS_NUMSCI]
  quadric_pairs.h/.cpp// sphere∩sphere, sphere∩cyl(coax), sphere∩cone(coax),
                      // cyl∩cyl(coax/parallel), cyl∩cone(coax)
  verify.h            // sample-and-check "lies on both surfaces" self-verify
```

Returned curves reuse native-math primitives. Where a first-class native `Circle`
/ `Ellipse` / `Line` curve type does not yet exist, S1 defines a minimal analytic
curve descriptor (center, axes, radii / direction, parameter range) that (a)
evaluates to points for self-verify and (b) maps 1:1 onto the OCCT `Geom_Line` /
`Geom_Circle` / `Geom_Ellipse` / `Geom_Parabola` / `Geom_Hyperbola` the parity
harness compares against.

## Result type

```cpp
enum class CurveKind { Line, Circle, Ellipse, Parabola, Hyperbola, Point };

struct IntersectionCurve {           // one analytic branch, on BOTH surfaces
  CurveKind kind;
  math::Ax3  placement;              // frame the conic is expressed in
  double     r1 = 0, r2 = 0;         // radius / semi-axes / focal params by kind
  math::Vec3 dir{};                  // line direction (Line) / axis (Parabola…)
  // parameter range + a value(t) evaluator for self-verify
};

enum class NotAnalyticReason {
  OutOfScopePair,      // e.g. skew cyl∩cyl, cone∩cone, torus∩curved
  FreeformSurface,     // any NURBS/Bezier/B-spline operand
  NonCoaxialQuadric,   // pair only handled coaxial/parallel in S1
  NearTangentOrCoincident,
  SelfVerifyFailed,    // a closed-form branch failed the on-both-surfaces check
  ObliquePlaneTorus,   // plane∩torus that does not reduce to a closed-form family
};

struct SsiResult {
  std::vector<IntersectionCurve> curves;  // empty ⇒ no intersection (still analytic)
  bool analytic = true;                    // false ⇒ see `reason`, DEFER to S2/S3/OCCT
  NotAnalyticReason reason{};
};
```

`analytic == false` is the contract with S5 / callers: **do not** trust
`curves`; route this pair to the marching tracer or OCCT. `analytic == true` with
empty `curves` means a proven "no intersection".

## Dispatch / classification

`classify(A, B)` is symmetric (it canonicalizes operand order) and returns the
handler or a NOT-ANALYTIC reason:

1. If either operand is freeform → `FreeformSurface`.
2. plane∩plane / plane∩sphere / plane∩cylinder / plane∩cone → always closed-form.
3. plane∩torus → the in-plane quartic handler (may itself return
   `ObliquePlaneTorus` if it does not reduce to a closed-form family).
4. sphere∩sphere → closed-form.
5. sphere∩cylinder, sphere∩cone, cylinder∩cone, cylinder∩cylinder → check relative
   placement: **coaxial** (and, for cyl∩cyl, **parallel**) → closed-form; otherwise
   `NonCoaxialQuadric`.
6. Any pair whose closed-form branch detects a near-tangent / coincident
   configuration → `NearTangentOrCoincident`.
7. Anything else → `OutOfScopePair`.

Cognitive-complexity target: dispatch is a flat classification (guard clauses +
a pair-type switch), kept in the backend band (≤15). Each closed-form handler is
isolated and small; the plane∩cone conic-type split and the plane∩torus quartic
are the most involved and are documented inline (systems band).

## Closed-form derivations (clean-room)

- **plane∩plane:** direction `d = n₁ × n₂`; a point via solving the 2×2 in-plane
  system. `|d|≈0` ⇒ parallel (coincident if the offset matches, else none).
- **plane∩sphere:** signed distance `h` of the center to the plane. `|h|>R` none;
  `|h|=R` tangent point; else circle of radius `√(R²−h²)` centered at the foot of
  the center on the plane, in the plane frame.
- **plane∩cylinder:** angle θ between the plane normal and the cylinder axis.
  Axis ⟂ plane (plane normal ∥ axis) ⇒ circle of the cylinder radius. Axis ∥ plane
  ⇒ 0/1/2 lines parallel to the axis by the axis-to-plane distance vs R. Oblique ⇒
  ellipse (semi-minor `R`, semi-major `R/cos θ`).
- **plane∩cone:** classic conic section by the angle of the plane vs the cone
  half-angle α through the apex: circle (⟂ axis), ellipse, parabola (plane
  parallel to a generator), hyperbola, or degenerate (apex point / line pair)
  when the plane passes through the apex.
- **plane∩torus:** substitute the plane into the torus implicit
  `(x²+y²+z²+R²−r²)² = 4R²(x²+y²)` → a **planar quartic**; solve with the
  `native-numerics` polynomial-root substrate. Handle the special families
  (axis-perpendicular plane → concentric circles; plane through the axis →
  two off-axis circles; the Villarceau plane → two circles). The general oblique
  plane returns the quartic curve if it factors into a closed-form family,
  otherwise `ObliquePlaneTorus` (deferred).
- **sphere∩sphere:** distance `d` between centers. `d>R₁+R₂` or `d<|R₁−R₂|` none;
  equality tangent point; else a circle in the plane ⟂ the center line at the
  known offset, radius from the two-sphere formula.
- **sphere∩cylinder (coaxial):** in the axial coordinate, the cylinder is
  `ρ=R_cyl`; intersect with the sphere `ρ²+ (z−z₀)² = R_sph²` → 0/1/2 circles at
  `z = z₀ ± √(R_sph²−R_cyl²)`, each of radius `R_cyl`.
- **sphere∩cone (coaxial):** same axial reduction against `ρ = z·tan α` → 0/1/2
  circles at the solved heights.
- **cylinder∩cylinder (coaxial/parallel):** coaxial equal R ⇒ coincident (report as
  such, `analytic` but degenerate); coaxial unequal ⇒ none; parallel equal R at
  center distance `2R`? tangent line; parallel with `|Δ|<2R`? the axial reduction
  gives two lines parallel to the axes; otherwise none.
- **cylinder∩cone (coaxial):** solve `R_cyl = z·tan α` for the height(s) → circle(s)
  of radius `R_cyl` at those heights.

## Self-verification (mandatory)

Before returning any `IntersectionCurve`, `verify.h` samples it (dense enough for
the curve kind) and asserts every sample satisfies both surfaces' implicit within
a tolerance derived from the operands' scale. On failure the whole result is
downgraded to `SsiResult{ analytic=false, reason=SelfVerifyFailed }` — S1 never
emits an unverified curve. This is the S1 instance of the roadmap's mandatory
**self-verify → OCCT fallback**.

## Verification model (two gates, per SSI-ROADMAP §Verification model)

- **Host analytic (no OCCT):** for each supported pair with a known conic, assert
  the returned curve kind + parameters match the closed form AND every sampled
  point lies on both surfaces within tol. Assert each out-of-scope pair returns
  `analytic == false` with the right reason.
- **Sim native-vs-OCCT parity:** build the same operands as OCCT `Geom_*Surface`,
  run `GeomAPI_IntSS`, and compare the native curve(s) against OCCT's (kind,
  sampled-point distance, count of branches) within tol — the internal parity gate
  used for native-math / native-topology. No `cc_*` call; SSI is compared at its
  own C++ boundary.

## Decisions

- **Symmetric dispatch.** Operand order is canonicalized so `classify(A,B)` and
  `classify(B,A)` route identically; handlers assume canonical order.
- **`analytic=false` is data, not an error.** NOT-ANALYTIC is a normal,
  first-class outcome (the deferral seam), not an exception.
- **Substrate only where required.** Only plane∩torus needs the polynomial-root
  solver, so only that TU is under `CYBERCAD_HAS_NUMSCI`; every other handler is
  solver-free and builds without NumPP/SciPP.
- **Coaxial/parallel-only quadric pairs in S1.** Skew quadric∩quadric is a genuine
  space quartic → S2/S3, so S1 returns `NonCoaxialQuadric` rather than approximate.

## Risks / Trade-offs

- **Family completeness vs honesty.** The closed-form family is finite; the
  dispatch + self-verify guarantee that anything outside it (or numerically
  unsafe) defers rather than lies. Accepted.
- **Placement classification tolerance.** Deciding coaxial/parallel/perpendicular
  uses angle/offset tolerances; borderline cases (near-tangent) route to
  `NearTangentOrCoincident` and defer, matching the roadmap's degeneracy stance.
- **Curve descriptor vs a full native curve library.** S1 introduces a minimal
  conic descriptor rather than a full `Geom`-equivalent curve library; it is
  sufficient for self-verify + OCCT parity and can be widened when S5 needs it.
