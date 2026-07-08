# Design — moat-dm3-replace-face (MOAT M-DM, DM3 + DM4)

## DM3: general replace_face = derive-target-plane + reuse DM2

`cc_replace_face(body, faceId, offset, tiltDeg)` (per the OCCT adapter it mirrors)
builds a new plane from the picked planar face's own plane `(o, n̂_F)`:

    p' = o + n̂_F · offset             (translate the origin along the outward normal)
    n' = Rot(faceXaxis, tiltDeg) · n̂_F  (tilt the normal about the face X-axis)

then trims the solid to `(p', n')` with a half-space cut. The **pure-offset**
sub-case (`tiltDeg = 0`) reduces to `n' = n̂_F`, `p' = o + n̂_F·offset` — a pure
parallel push (offset > 0, grow) / pull (offset < 0, trim). That is exactly DM2's
parallel branch, so `replaceFaceOffsetTilt` reads the picked face
(`rfdetail::readPickedFace`), forms `tp = centroid + n̂_F·offset`, and calls the
byte-frozen DM2 `replaceFaceToPlane(solid, faceId, tp, n̂_F)`. The DM2 verb owns the
grow/trim dispatch, the watertight self-verify, and the closed-form
`V' = V₀ + A_F·offset` oracle — DM3 inherits all of it.

**Why tilt declines.** A non-zero `tiltDeg` rotates about `gp_Pln::XAxis`, the OCCT
surface-PARAMETRIZATION X-axis of the face. For a native B-rep body that axis is a
foreign convention we do not reproduce, so the resulting solid could not be proven
equal to the OCCT reference on the SIM parity gate. Rather than tilt about a
native-chosen axis (which would silently diverge from OCCT), DM3 honestly declines
`tiltDeg ≠ 0` to OCCT. This lands the tractable, both-gate-verifiable slice and
leaves a sharp, measured blocker — never a wrong or convention-mismatched solid.

## DM4: closed-form normal projection onto an analytic surface

`projectPointOnFace(solid, faceId, p)` reads the face surface world-placed
(`topology::surfaceOf` + a Location-placed frame) and computes the
foot-of-perpendicular in closed form:

- **Plane** `(o, n̂)`: `s = (p − o)·n̂`; `foot = p − s·n̂`; `distance = |s|`.
- **Cylinder** (axis `o + t·â`, radius `R`): split `v = p − o` into axial `hâ` and
  radial `ρ`; `foot = o + hâ + (R/ρ)·radial`; `distance = |ρ − R|`.
- **Sphere** (centre `c`, radius `R`): `foot = c + (R/ρ)(p − c)`; `distance = |ρ − R|`.

This is projection onto the face's INFINITE analytic surface — the exact behaviour
of OCCT `GeomAPI_ProjectPointOnSurf` on the untrimmed `Geom_Surface`, which the SIM
gate uses as the oracle. Cone / torus / freeform are out of the analytic slice and
DECLINE (`NonAnalyticFace`). A point ON the cylinder axis / AT the sphere centre has
a whole-circle / whole-sphere foot — no single closed-form answer — and DECLINES
(`Ambiguous`). The engine reports the decline and falls to OCCT; it never fabricates
a foot.

## ABI

`CCProjection { double footX, footY, footZ; double distance; int valid; }` +
`CCProjection cc_project_point_on_face(CCShapeId body, int faceId, double px, double py,
double pz)`. `valid = 0` on an honest decline (with `cc_last_error` set). The
internal `ProjectionData` POD carries the same fields through `IEngine`.

## Two gates (OCCT is the oracle)

- **(a) HOST analytic (no OCCT):** DM3 asserts the closed-form volume `V₀ + A_F·offset`
  (fp-exact for a box; off-axis face too) on the watertight re-solve; DM4 asserts the
  closed-form foot + distance for plane / cylinder / sphere and the honest declines.
- **(b) SIM native-vs-OCCT (booted simulator):** DM3 vs the OCCT plane-cut-and-extend
  move-face oracle (volume / area / watertight / Euler χ / bbox), including an off-axis
  fixture whose oracle is the same cut rigidly rotated; DM4 vs
  `GeomAPI_ProjectPointOnSurf` on the matching untrimmed `Geom_Surface` (foot coords +
  distance to machine precision).

## Non-goals

Tilted retarget (foreign X-axis convention), non-planar picked faces, non-all-planar
solids, cone / torus / freeform projection, and ambiguous projection poses all remain
OCCT — declined with a measured reason, never faked.
