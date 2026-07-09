# Design — moat-gs1c-curved-hlr

## Silhouette geometry

### Cone / cone-frustum

The native revolve represents a slanted profile segment as a `Kind::Cone`
`FaceSurface` with `frame` (origin on the axis, `Z`=axis), `radius` = the cone
reference radius at axial height `h = 0`, and `semiAngle` `α` = the half-angle.
The elementary parametrization is `r(v) = radius + v·sinα`, axial `h = v·cosα`,
so eliminating `v`:

    r(h) = radius + h·tanα        (rim radius at axial height h)

The outward normal is constant along a ruling:

    n(u) = cosα·(cos u·X + sin u·Y) − sinα·Z

Silhouette condition `n·d = 0`:

    cosα·(cos u·Xd + sin u·Yd) = sinα·Zd
    ⇒ hypot(Xd,Yd)·cos(u − φ) = tanα·Zd,   φ = atan2(Yd, Xd)
    ⇒ cos(u − φ) = (sinα·Zd) / (cosα·hypot(Xd,Yd)) = rhs

Two rulings at `u = φ ± acos(rhs)`. Each is a straight 2-point polyline between
`conePoint(hMin)` and `conePoint(hMax)`, where the trim `[hMin,hMax]` is the
face's axial extent measured `dot(edgePoint − origin, Z)` (axial HEIGHT, matched
to the `r(h)` formula, NOT the slant parameter).

**Declines** (never a fabricated ruling):
- view parallel to axis (`hypot(Xd,Yd) ≈ 0`): the whole side is silhouette;
- view end-on (`|rhs| > 1`): the cone is seen along a ruling, no lateral
  contour ⟂ view exists.

A full cone (apex end, `r = 0` at one trim end) is handled uniformly: the ruling
simply runs from the apex point to the rim.

### Torus

A `Kind::Torus` face has `frame` (origin = torus centre, `Z`=axis), `radius` =
MAJOR radius `R`, `minorRadius` = tube radius `r`. Parametrization (matching
`math::Torus` / OCCT ElSLib):

    S(u,v) = O + (R + r·cos v)·(cos u·X + sin u·Y) + r·sin v·Z
    n(u,v) = cos v·(cos u·X + sin u·Y) + sin v·Z

Silhouette condition `n·d = 0`, per major angle `u`:

    cos v·P(u) + sin v·Zd = 0,   P(u) = cos u·Xd + sin u·Yd
    ⇒ v*(u) = atan2(−P(u), Zd)   (and the antipodal branch v*+π)

Sweeping `u ∈ [0,2π)` traces the two closed turning contours (the outer and inner
limbs). Discretized in `u` to the chord-sagitta bound sized for the widest limb
(radius `R + r`).

**Decline**: view down the axis (`hypot(Xd,Yd) ≈ 0`) makes `P(u) ≈ 0` for all
`u`, so the turning contour collapses onto the two rim circles — a boundary case
declined rather than emitted as a near-degenerate contour.

## Why a revolve-built torus still declines (sharpened reason)

The construction path (`residuals.h build_revolution_profile_spline`) emits an
off-axis-arc revolve as **rational-quadratic B-spline surface-of-revolution
bands** — `Kind::BSpline` faces, not `Kind::Torus`. The analytic tracer cannot
robustly recover `(R, r, frame)` from B-spline poles, so those faces hit the
freeform decline. Only a `Kind::Torus` face (produced today by the STEP reader)
carries the analytic parameters and is traced. The decline message names this
explicitly so the map stays honest.

## Reuse of the landed occlusion path

Both new generators emit world-space polylines identical in shape to the
cylinder/sphere output, so the existing engine machinery applies unchanged:
per-outline endpoint dedup across angular sector faces, self-verify that every
emitted silhouette point lies on the occluder boundary (rejecting a partial
quadric / off-surface contour), the convex-limb self-grazing surfaceOffset cure,
and `projectOrthographic` classification + splitting. No tessellator change.

## Structural discipline

- `silhouette.h` includes only `native/math` (vec + elementary); OCCT-free,
  header-only, `clang++ -std=c++20`.
- Polyhedral (planar-only) inputs keep `curvedFaces` empty, so every curved-only
  branch (smooth-edge suppression, silhouette augmentation, offset cure) is
  skipped and their HLR output is byte-identical.
- No `cc_*` symbol added or changed.
