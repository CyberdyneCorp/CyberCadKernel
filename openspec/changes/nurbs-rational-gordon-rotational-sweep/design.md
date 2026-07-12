# Design — nurbs-rational-gordon-rotational-sweep

## Placement & conventions

Additive extensions to the existing Layer-6 modules `src/native/math/bspline_gordon.{h,cpp}` and
`bspline_sweep.{h,cpp}`, namespace `cybercad::native::math`. No new files: the shared library
globs the module `.cpp` sources, and the host gates
(`tests/native/test_native_nurbs_gordon.cpp` / `test_native_nurbs_sweep.cpp`) already link the
library. Reuse `math::Point3` / `Vec3` / `Dir3` (`vec.h`), the evaluators (`nurbsCurvePoint`,
`nurbsSurfacePoint`, `curvePoint`, `findSpan`, `basisFuns` from `bspline.h`), the **rational-aware
Layer-1 surface ops** (`elevateDegreeSurface` / `refineKnotSurface`, which run on the homogeneous
R⁴ net so weights ride through exactly), the **Layer-6 rational compatibility**
(`makeRationalSectionsCompatible` from `bspline_skin.h`), and `transform.h` (`Mat3::rotation`,
though the revolve builds arc poles directly). OCCT-free, fp64, deterministic.

**numsci gate.** The rational Gordon family/grid interpolations solve collocation systems through
`numerics::lin_solve`, so `bspline_gordon.cpp` stays entirely under `CYBERCAD_HAS_NUMSCI`. The
rotational sweep is itself solve-free (pure tensor product of the profile with a rational arc) but
lives in the same guarded `bspline_sweep.cpp` TU for one clean compilation unit. Both headers
declare everything; with the guard OFF the implementation TUs are inert and the functions absent.

Conventions match the rest of the kernel: flat clamped knot vectors; row-major, U-outer surface
poles `pole(i,j) = poles[i*nPolesV + j]`; a **non-empty `weights` vector ⇒ rational** (one weight
per pole).

## Rational Gordon — boolean sum in homogeneous R⁴

The non-rational Gordon surface is the boolean sum `G = S_u ⊕ S_v ⊖ T` of two skins and a grid
tensor interpolant, brought to a common basis then `poles(G) = poles(S_u) + poles(S_v) − poles(T)`.
The rational version carries the whole construction in HOMOGENEOUS space: a rational
curve/surface is its net of `(w·P, w) ∈ R⁴` control points, and the four coordinates
`(wx, wy, wz, w)` are interpolated by the SAME collocation matrix as the non-rational path, then
projected back to `(pole, weight)`. This is exactly the rational-skin / rational-fit lift.

The subtlety unique to Gordon: the boolean sum only cancels on the network curves if the three
summands agree at the grid in HOMOGENEOUS space, not merely Euclidean space. When a u-curve `C_k`
and a v-curve `D_l` intersect at `Q_{k,l}`, their rational evaluators give the same Euclidean
point but generally different homogeneous values `w·Q` (their local weights differ). So the
rational boolean sum requires the network to be **homogeneously consistent**: `C_k^w(u_l)` must
equal `D_l^w(v_k)` at the grid. `gordonRationalSurface` builds the homogeneous grid from the
u-curves and DECLINES if the two families disagree homogeneously (checked against tol) — an honest
precondition. A network extracted from a KNOWN rational surface is homogeneously consistent by
construction, which is the airtight test oracle.

After the three homogeneous summands are interpolated and brought to a common basis with the
rational-aware `unifyDirection`, the net is combined `homog(G) = homog(S_u) + homog(S_v) −
homog(T)` per pole and projected back. A projected non-positive weight clears `ok` (never divide
by ≤ 0). `verifyRationalNetwork` is the rational analogue of `verifyNetwork` (grid checked with
`nurbsCurvePoint`).

## Rotational (revolved) sweep — exact rational surface of revolution (A7.1)

Revolving a profile about an axis by a signed `angle` is the tensor product of the profile (U)
with a rational circular arc (V). Following *The NURBS Book* §8.5 / A7.1:

- Split the angle into `narcs = ceil(|angle|/90°)` segments each spanning `Δθ = angle/narcs`
  (≤90°). The V direction is degree 2 with `2·narcs + 1` poles over a clamped knot vector whose
  interior breakpoints (at `j/narcs`) have multiplicity 2 — the standard piecewise rational arc.
- For each profile control point `P`, let `foot` be its projection onto the axis and `r =
  ‖P − foot‖` its radius. Build the in-plane frame `e0 = radial/r`, `e1 = axis × e0`. Per segment
  the ON-arc pole sits at angle `seg·Δθ` (arc weight 1) and the BETWEEN pole at the segment's
  mid-angle at radius `r/cos(Δθ/2)` (arc weight `cos(Δθ/2)` — the tangent-intersection pole).
- The surface weight is the SEPARABLE product `wProfile · wArc`. This separability is what makes
  the revolve exact: the z-coordinate and axis-distance of any surface point are then
  `v`-independent (the arc-weight factor cancels between numerator and denominator), so the
  revolve preserves the profile's shape at every angle.

**On-axis profile point (r ≈ 0):** the ON and BETWEEN poles keep the POSITION `P` (it stays on the
axis under revolution), but the weight STILL follows the arc pattern `wProfile · {1, cos(Δθ/2),
…}`. Forcing weight 1 there (the naive choice) breaks the separable weight structure and warps the
revolve away from the analytic surface — the bug the sphere oracle catches. This is the one
non-obvious detail of the construction.

The finished surface is always rational (the arc weights alone make it so, even for a non-rational
profile). U carries the profile's degree/knots/poles and its weights when rational. `vParams =
{0, 1}` (the domain endpoints; the arc in between is the exact rational sweep of the profile
through the swept angle).

## Oracles

- **Rational Gordon containment** — extract a u/v RATIONAL iso-curve network from a KNOWN rational
  surface (a rational revolved patch, rational in BOTH directions), build the rational Gordon
  surface, confirm it reproduces every rational network curve pointwise (~7e-16) and the grid
  points lie on the surface.
- **EXACT revolve** — the strongest oracle: a straight offset segment revolved 360° is an exact
  cylinder, a tilted segment a cone/frustum, a rational semicircle a sphere — each matching the
  analytic surface of revolution (radius / height / distance-from-center) pointwise to ~1e-15. A
  faceted or weight-warped revolve fails these immediately. Partial-angle and >180° (multi-segment)
  revolves add coverage.
- **Honest declines** — inconsistent / non-rational / degenerate networks and zero-angle /
  on-axis / null-axis / malformed / non-positive-weight profiles decline (`ok = false`, no crash,
  no faked surface).

## Alternatives considered

- **Skin K rotated station copies (like the general sweep)** — rejected: a skinned station
  approximation is NOT the exact analytic surface of revolution (it only contains the stations),
  and would fail the exact cylinder/cone/sphere oracles. The A7.1 rational-arc tensor product is
  the exact construction.
- **Euclidean-only network consistency for rational Gordon** — rejected: the boolean sum does not
  cancel unless the summands agree in homogeneous space at the grid, so the homogeneous-consistency
  precondition is required and checked honestly.
