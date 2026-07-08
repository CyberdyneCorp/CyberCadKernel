# native-directmodel

## ADDED Requirements

### Requirement: Native general `cc_replace_face` retargets a planar face by a pure normal offset

The engine SHALL compute `cc_replace_face(body, faceId, offset, tiltDeg)` — retarget
the planar face `faceId` by translating it `offset` along its outward normal and tilting
it `tiltDeg` about the face's parametric X-axis, trimming `body` to the new plane —
NATIVELY, without OCCT, when `body` is a native all-planar solid, the picked face is an
identifiable planar polygon, AND `tiltDeg` is (within a tight tolerance) zero. For the
pure-offset case the engine SHALL derive the target plane `(o + n̂_F·offset, n̂_F)` from
the picked face's own outward plane `(o, n̂_F)` and re-solve the neighbours by invoking
the landed DM2 `replaceFaceToPlane(body, faceId, tp, n̂_F)` verb, CONSUMED byte-identical,
which owns the grow/trim dispatch and the watertight self-verify.

The returned solid SHALL be a native `topology::Shape` of type `Solid`, watertight
(closed 2-manifold), single-lump (Euler χ = 2), with the picked face moved onto the
target plane and enclosed volume equal to the closed form `V₀ + A_F·offset` within the
tessellation deflection tolerance. This native path SHALL remain OCCT-free and SHALL NOT
change the `cc_*` ABI (DM3 supplies only a native path behind the existing facade seam).

A non-zero `tiltDeg` (a rotation about OCCT's face-parametrization X-axis — a foreign
convention the native engine does not reproduce), a non-planar picked face, a solid with
any curved neighbour, and an unverifiable/no-op offset SHALL be honestly DECLINED: the
native op SHALL return a NULL result with a measured reason and the engine SHALL fall
through to OCCT. The engine SHALL NEVER hand a native void to OCCT and SHALL NEVER emit
an unverified or convention-mismatched solid.

#### Scenario: A pure normal offset of a planar face yields the closed-form watertight solid (host)

- GIVEN a native all-planar solid `B` with a planar face `F` of area `A_F` and enclosed volume `V₀`, and a signed `offset`, with the native engine active and no OCCT
- WHEN `cc_replace_face(B, id(F), offset, 0)` is invoked
- THEN the returned solid SHALL be a watertight single-lump `Solid` with `F` on the plane `(o + n̂_F·offset, n̂_F)` AND its enclosed volume SHALL equal `V₀ + A_F·offset` within the deflection tolerance

#### Scenario: A non-axis-aligned planar face offsets correctly (host)

- GIVEN a native all-planar solid rotated so a chosen planar face `F` has a non-axis-aligned outward normal, with the native engine active and no OCCT
- WHEN `cc_replace_face(B, id(F), offset, 0)` is invoked
- THEN the returned solid SHALL be watertight AND its enclosed volume SHALL equal `V₀ + A_F·offset` (the offset is invariant under the rigid pose)

#### Scenario: A native offset move-face matches the OCCT move-face oracle (sim)

- GIVEN a native box and the OCCT plane-cut-and-extend move-face reference for the same face and offset, on a booted simulator
- WHEN both compute the offset retarget
- THEN the native result SHALL match the OCCT reference on volume, area, watertightness, topology (Euler χ = 2, one solid) and per-axis bbox within fixed tolerances

#### Scenario: A tilted or non-planar retarget is honestly declined (host)

- GIVEN a native solid and either a non-zero `tiltDeg`, a curved neighbour, or a non-planar picked face, with the native engine active and no OCCT
- WHEN `cc_replace_face` is invoked
- THEN the native op SHALL return a NULL result with a measured decline reason AND SHALL NOT emit a solid (the engine falls through to OCCT)

### Requirement: Native `cc_project_point_on_face` computes the closed-form foot on an analytic surface

The engine SHALL compute `cc_project_point_on_face(body, faceId, px, py, pz)` — the
foot-of-perpendicular of the point `(px,py,pz)` on the underlying analytic surface of
face `faceId`, plus the minimum point→surface distance — NATIVELY, without OCCT, when
`body` is a native body and the face surface is a plane, cylinder, or sphere. The engine
SHALL read the face surface world-placed and compute the foot in closed form: for a
plane, drop the normal component (`foot = p − ((p−o)·n̂)n̂`); for a cylinder, push the
radial component of `p−o` onto the radius; for a sphere, push `p−c` onto the radius. The
projection SHALL be onto the face's untrimmed (infinite) analytic surface, matching OCCT
`GeomAPI_ProjectPointOnSurf` on the untrimmed `Geom_Surface`.

The result SHALL be returned via the additive `CCProjection` struct
(`footX, footY, footZ, distance, valid`) behind the additive `cc_project_point_on_face`
entry; no existing `cc_*` signature SHALL change. On a native body the OCCT adapter is
never reached; on an OCCT body the call forwards to the `GeomAPI_ProjectPointOnSurf`
oracle.

A cone / torus / freeform (BSpline/Bezier) face, and an AMBIGUOUS pose where the foot is
not a single point — a point on the cylinder axis or at the sphere centre — SHALL be
honestly DECLINED: `cc_project_point_on_face` SHALL return `valid = 0` with `cc_last_error`
set, and the native op SHALL never fabricate a foot.

#### Scenario: A point projects onto a plane / cylinder / sphere face in closed form (host)

- GIVEN a native box, cylinder, and sphere and a query point off each analytic face, with the native engine active and no OCCT
- WHEN `cc_project_point_on_face` is invoked for the plane / cylinder / sphere face
- THEN the returned foot SHALL equal the closed-form foot-of-perpendicular AND the distance SHALL equal the closed-form min distance AND `valid` SHALL be 1

#### Scenario: The native foot matches GeomAPI_ProjectPointOnSurf (sim)

- GIVEN a native plane / cylinder / sphere fixture and the matching untrimmed OCCT `Geom_Surface`, on a booted simulator
- WHEN the native projection and OCCT `GeomAPI_ProjectPointOnSurf` project the same point
- THEN the native foot coordinates and distance SHALL match the OCCT nearest point and lower distance to machine precision

#### Scenario: A non-analytic or ambiguous projection is honestly declined (host)

- GIVEN a cone face, or a point on a cylinder axis / at a sphere centre, with the native engine active and no OCCT
- WHEN `cc_project_point_on_face` is invoked
- THEN the call SHALL return `valid = 0` with a measured reason AND SHALL NOT return a fabricated foot
