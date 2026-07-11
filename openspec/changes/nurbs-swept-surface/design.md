# Design — nurbs-swept-surface

## Placement & conventions

New module `src/native/math/bspline_sweep.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_ops.{h,cpp}`, `bspline_fit.{h,cpp}`, and `bspline_skin.{h,cpp}`. Reuses `math::Point3` /
`Vec3` / `Dir3` (`native/math/vec.h`), the evaluators (`curveDerivs` from `bspline.h`, for the spine
point + tangent), the affine `Mat3` / `Transform` (`native/math/transform.h`, for the moving-frame
placement), the **Layer-6 `skinSurface`** (`bspline_skin.h`, to skin the placed sections), and the
**Layer-1 data types** `BsplineCurveData` (section / trajectory in) / `BsplineSurfaceData` (surface
out). OCCT-free, fp64, deterministic. Added to the `native_math.h` aggregator.

**numsci gate.** The general sweep composes `skinSurface`, whose V-interpolation solves through the
numsci facade (`numerics::lin_solve`), so the whole `.cpp` is under `CYBERCAD_HAS_NUMSCI`, exactly
like `bspline_skin.cpp`: the header declares everything; with the guard OFF the implementation TU is
inert and the functions are absent. `CYBERCAD_HAS_NUMSCI` is defined library-wide
(`target_compile_definitions(cybercadkernel PRIVATE CYBERCAD_HAS_NUMSCI=1)`), so `bspline_sweep.cpp`
— though in the default `src/native` glob — sees it when the option is ON. (The translational sweep
is itself solve-free — pure tensor product — but is kept in the same guarded TU for a single clean
compilation unit.)

Conventions match the rest of the kernel: **flat clamped knot vectors** (degree+1 end multiplicity,
length `nPoles + degree + 1`); **row-major, U-outer** surface poles `pole(i,j) = poles[i*nPolesV +
j]`; **non-rational** (weights empty).

## Translational sweep (§10.4, the exact base case)

`sweepTranslational(section, sweep)` returns the EXACT extrusion of `section` along the straight
vector `sweep`. There is no fitting and no solve — the swept surface is the tensor product of the
section (U) with a degree-1 two-pole path (V):

- **U direction** = the section: `degreeU = section.degree`, `knotsU = section.knots`,
  `nPolesU = N = section.poles.size()`.
- **V direction** = the straight path: `degreeV = 1`, `knotsV = {0,0,1,1}`, `nPolesV = 2`.
- **Net** (row-major, U-outer): `pole(i,0) = section.poles[i]`, `pole(i,1) = section.poles[i] +
  sweep`.

Because a degree-1 B-spline on `{0,0,1,1}` is the straight interpolation `P(v) = (1−v)·P₀ + v·P₁`,
the tensor-product surface is `S(u,v) = C(u) + v·sweep` where `C` is the section — so the iso-curve
`S(·,v)` is EXACTLY the section translated by `v·sweep`, machine-exact for every `v`. `vParams =
{0,1}` (the surface reproduces the section at `v=0` and section+sweep at `v=1`).

**Guards:** rational section (non-empty weights) → decline; malformed section (degree < 1, empty
poles, or `knots.size() ≠ poles + degree + 1`) → decline; **null sweep** (`|sweep|` below tolerance
— a zero-length extrusion has no surface) → decline. All return `ok=false` without crashing.

## General sweep (§10.4, transform-then-skin)

`sweepAlongTrajectory(section, trajectory, sectionNormal, stations, degreeV)`:

1. **Sample the spine.** Take `stations` parameters evenly across the trajectory's clamped domain
   `[a,b] = [knots.front(), knots.back()]`. At each, `curveDerivs(...,1,...)` gives the point `Pₖ`
   and (from the first derivative) the unit tangent `Tₖ`. A stationary spine point (null first
   derivative) → decline.
2. **Coincident-trajectory guard.** If the total sampled path length `Σ |Pₖ − Pₖ₋₁|` is below
   tolerance there is no path to sweep along → decline (mirrors the skin's coincident-sections
   decline).
3. **Rotation-minimizing frame (anti-twist).** Compute reference normals `rₖ` along the spine by the
   **double-reflection method** (Wang, Jüttler, Zheng & Liu, "Computation of Rotation Minimizing
   Frames", ACM TOG 2008). Seed `r₀` = the component of `sectionNormal` orthogonal to `T₀`
   (normalized; fall back to a stable orthonormal seed if `sectionNormal ∥ T₀`). Propagate each
   `rₖ₊₁` by two reflections (across the plane bisecting `Pₖ,Pₖ₊₁`, then across the plane bisecting
   the reflected tangent and `Tₖ₊₁`), then re-orthogonalize against `Tₖ₊₁` and normalize. The result
   is a frame that rotates by the MINIMUM amount consistent with following the tangent — it has NO
   torsion-driven spin, the standard anti-twist choice for pipe/tube sweeps.
4. **Place the section at each station.** Build the section reference basis `(u₀, v₀, n₀)` with
   `n₀ = sectionNormal` and `(u₀,v₀)` a stable orthonormal span of the section plane. Build each
   station basis `(uₖ, vₖ, wₖ)` with `wₖ = Tₖ`, `uₖ = rₖ`, `vₖ = wₖ × uₖ`. The placement rotation
   `Rₖ = [uₖ vₖ wₖ]·[u₀ v₀ n₀]ᵀ` maps the section basis onto the station basis; the affine placement
   is `v' = Rₖ·v + Pₖ` (rotate about the section origin, translate the origin to the station point).
   Transform a copy of the section's poles by this rigid map.
5. **Skin.** Call `skinSurface(placed, degreeV)` on the `stations` transformed sections. The result
   is returned verbatim: `surface` = the skin's tensor-product surface, `vParams` = the skin's
   station parameters. A downstream skin failure (e.g. coincident placed sections) → decline.

**Why transform-then-skin.** The skin already carries an airtight containment oracle (its iso-curve
at `v_k` equals section `k` exactly). Since each placed section is an exact rigid image of the
section, the swept surface contains each placed section at its station parameter — the sweep inherits
a machine-precision oracle for free, and never reinvents surface interpolation.

**Frame choice, restated for the record.** The Frenet frame is rejected because it is undefined at
inflection points (zero curvature) and spins with the spine's torsion, which would twist the swept
profile. The rotation-minimizing frame is defined everywhere the tangent is defined and introduces
no torsion-driven rotation, so a constant profile sweeps without twist — exactly what pipe/tube/
moulding sweeps require. For a straight spine the RMF is constant, so the general sweep collapses to
the translational sweep (verified in the gate).

## Guards (honest declines, never a crash)

`sweepTranslational`: rational section; malformed section; null sweep vector.
`sweepAlongTrajectory`: `stations < 2`; rational section OR trajectory; malformed section OR
trajectory; degenerate spine domain (`b ≤ a`); stationary spine point (null tangent);
coincident-trajectory points (no path); downstream skin failure. Every guard returns
`SweepResult{ok=false}` and allocates nothing surprising.

## Residuals (documented, never faked)

Rational / weighted sweep; rotational (revolved) sweep (exact-rational, distinct construction);
exact GeomFill/BRepFill-class continuous sweep (the general sweep skins a finite station set — an
approximation that improves with `stations`); variable section (a profile that morphs along the
spine). All recorded in `docs/NURBS-SCOPE.md` Layer-6 row.

## Test strategy

Host-analytic, OCCT-free, numsci-gated (mirrors `test_native_nurbs_skin`): translational exactness
(dense stations, closed-form translate reference, ~1e-12 target / ~1e-15 achieved); the two
known-surface closed-form patches (ruled bilinear, cylinder-like profile); general-sweep station
containment on a straight spine (placement is a pure translation there, so the reference placed
sections are closed-form); anti-twist equality between the straight general sweep and the
translational sweep; a curved-spine arc-length invariant (a rigid frame preserves the section length
at every station); the full degenerate-guard matrix.
