# native-math

## ADDED Requirements

### Requirement: Exact analytic-primitive to rational NURBS construction

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/analytic_nurbs.{h,cpp}`, guarded under `CYBERCAD_HAS_NUMSCI`),
constructors that convert an analytic primitive into an EXACT rational NURBS
representation (Piegl & Tiller *The NURBS Book* Chapter 7): `circleToNurbs`,
`arcToNurbs`, `ellipseToNurbs`, `lineToNurbs` for curves and `planeToNurbs`,
`cylinderToNurbs`, `coneToNurbs`, `sphereToNurbs`, `torusToNurbs` for surfaces. A full
circle SHALL be a piecewise rational-quadratic B-spline of four quarter-circle segments
with alternating weights `1, cos(45°), 1, …`; an arc SHALL be split into segments of
sweep ≤ 90° each with middle weight `cos(halfSweep)`. Quadric and torus surfaces SHALL
be built as exact rational surfaces of revolution. The representation SHALL be EXACT —
the curve/surface traces the true primitive, it does not sample it.

#### Scenario: Points on a constructed primitive lie on the true primitive exactly

- GIVEN an analytic circle, cylinder, sphere, or torus
- WHEN it is converted with the corresponding `*ToNurbs` constructor and the resulting rational NURBS is evaluated on a dense parameter grid
- THEN every evaluated point SHALL satisfy the primitive's implicit equation to ≤ 1e-13 (rational quadratic is exact, not an approximation)

### Requirement: Rational-NURBS to analytic-primitive recognition with algebraic exactness

The module SHALL provide `recognizeCurve` and `recognizeSurface` that detect whether a
rational NURBS curve/surface is EXACTLY a line/circle/arc/ellipse or plane/cylinder/
cone/sphere and recover its parameters, otherwise reporting "general". Recognition MAY
use the primitive-fit machinery (`primitive_fit.h`) on sampled points as a candidate
generator, but acceptance SHALL require an ALGEBRAIC exactness certificate at the
control-net level — the control points (curves) or the surface numerator polynomial
`p̃ᵀ Q p̃` on a dense grid (surfaces) SHALL satisfy the candidate primitive's equation to
≤ 1e-12. A candidate with a small RMS but a non-vanishing algebraic certificate SHALL be
rejected as "general"; a tolerance SHALL NEVER be widened to force a primitive.

#### Scenario: Round-trip recovers the primitive exactly

- GIVEN an analytic primitive (circle, arc, ellipse, line, plane, cylinder, cone, or sphere)
- WHEN it is converted to a rational NURBS and passed back through `recognizeCurve`/`recognizeSurface`
- THEN the recognized kind SHALL match the original and the recovered parameters (center/radius/axis/plane/half-angle, as applicable) SHALL agree to ≤ 1e-12

#### Scenario: A freeform NURBS is not mistaken for a primitive

- GIVEN a genuinely freeform rational NURBS (e.g. a bicubic bump surface) OR a non-uniform-weight almost-circle that is not actually a circle
- WHEN it is passed through `recognizeSurface`/`recognizeCurve`
- THEN the result SHALL be "general", NOT a spurious primitive, because the control-net algebraic certificate does not vanish
