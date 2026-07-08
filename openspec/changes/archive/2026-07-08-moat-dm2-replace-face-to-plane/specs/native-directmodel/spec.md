# native-directmodel

## ADDED Requirements

### Requirement: Native `cc_replace_face_to_plane` re-solves one planar face of a reachable convex planar solid onto a target plane

The engine SHALL compute `cc_replace_face_to_plane(body, faceId, px,py,pz, nx,ny,nz)` —
retarget the single planar face `faceId` of `body` onto the target plane `P_t` through
`(px,py,pz)` with normal `normalize(nx,ny,nz)`, re-solving the adjacent PLANAR faces so
the solid stays watertight (the app's push/pull "move a face to a target plane") —
NATIVELY, without OCCT, when `body` is a **convex planar polyhedron** in the reachable
native domain and the move keeps the result a bounded convex 2-manifold with the SAME
face adjacency (each original neighbour still contributes one planar face; the picked
face now lies on `P_t`). The engine SHALL read the picked face's outward normal `n_F`
and plane from the native topology (READ-ONLY), reject a non-planar picked face, and
dispatch by the average signed displacement `d̄ = δ(centroid(F))` with `δ(p) =
((P_t.o − p)·n)/(n·n_F)`:

- **Pure trim** (`d̄ < 0`, the target plane keeps the body bulk and removes only an
  outward slab from every neighbour) SHALL be re-solved by the landed DM1 verb
  `boolean::splitByPlane(operand, P_t.o, P_t.n, keepPositive = bulk-side)`, CONSUMED
  byte-identical.
- **Pure grow, parallel** (`d̄ > 0`, `n ∥ n_F`) SHALL be re-solved by fusing the slab
  formed by extruding the picked face to `P_t`:
  `boolean::boolean_solid(operand, build_prism(faceLoop(F), n_F·d), Op::Fuse)`, then
  healed/welded.
- **General convex re-solve** (a tilted `P_t`, or a mixed grow/trim on a convex
  polyhedron) SHALL be re-solved by **grow-then-trim** — two already-gated ops, NOT an
  N-cut half-space chain: first extend the operand outward past the target plane's reach
  by fusing a parallel slab `grown = boolean::boolean_solid(operand,
  build_prism(faceLoop(F), n_F·G), Op::Fuse)`, then a SINGLE tilted cut to the target
  plane keeping the bulk `boolean::splitByPlane(grown, P_t.o, −P_t.n, keepPositive =
  bulk-side)`, then healed/welded. A pure-trim tilt (the moved face stays inside the
  operand everywhere) SHALL skip the grow and be a single cut.

The returned solid SHALL be a native `topology::Shape` of type `Solid`, watertight
(closed 2-manifold, every edge shared by exactly two faces) with positive enclosed
volume, a single lump (Euler χ = 2), with the moved face lying on `P_t` and the original
face count preserved. Moving the face by the signed displacement SHALL change the
enclosed volume by the CLOSED-FORM `ΔV = A_face · d̄` (for a parallel offset of an
axis-aligned box: fp-exact `A_face · d`; for a tilt about an in-face axis through the
centroid: the wedge integral `A_face · d̄`). This native path SHALL remain OCCT-free —
the consumed verbs (`splitByPlane`, `boolean_solid`, `build_prism`, the heal weld)
reference no OCCT — and the `cc_*` ABI SHALL be unchanged (DM2 supplies only a native
path behind the existing facade seam).

#### Scenario: An axis-aligned box face pushed to a parallel plane grows by the closed-form volume (host)

- GIVEN an axis-aligned native box `B` of face area `A` and one planar face, with the native engine active (`cc_set_engine(1)`) and no OCCT
- WHEN `cc_replace_face_to_plane(B, faceId, P_t)` retargets that face to a parallel plane at signed normal distance `d > 0` (a push)
- THEN the result SHALL be a watertight single-lump `Solid` with the moved face on `P_t` AND the original face count preserved AND its enclosed volume SHALL equal `|B| + A·d` fp-exact

#### Scenario: An axis-aligned box face pulled to a parallel plane shrinks by the closed-form volume (host)

- GIVEN an axis-aligned native box `B` of face area `A`, with the native engine active and no OCCT
- WHEN `cc_replace_face_to_plane(B, faceId, P_t)` retargets that face to a parallel plane at signed normal distance `d < 0` (a pull that keeps the body bulk)
- THEN the result SHALL be a watertight single-lump `Solid` with the moved face on `P_t` AND its enclosed volume SHALL equal `|B| + A·d` fp-exact (a shrink) AND the pull SHALL be the DM1 `splitByPlane` bulk-side piece

#### Scenario: An axis-aligned box face retargeted to a tilted plane re-solves to a watertight convex polytope (host)

- GIVEN an axis-aligned native box `B` of face area `A` and one planar face, with the native engine active and no OCCT
- WHEN `cc_replace_face_to_plane(B, faceId, P_t)` retargets that face to a plane tilted about an in-face axis through the face centroid (the neighbours re-solve to the new incidence)
- THEN the result SHALL be a watertight single-lump convex `Solid` with the moved face on `P_t` AND the original face count preserved AND its enclosed volume SHALL equal `|B| + A·d̄` within the deflection tolerance, where `d̄` is the centroid displacement

#### Scenario: A verified native move-face result is read back by the native paths (host)

- GIVEN a native `cc_replace_face_to_plane` result that PASSES the self-verify (watertight 2-manifold, positive volume, moved face on the target plane, single lump)
- WHEN its mass properties, bounding box, sub-shape ids, and tessellation are queried
- THEN they SHALL be served by the native body-consuming paths with no OCCT fallback call

### Requirement: Mandatory re-solve self-verify (discard and fall through to OCCT)

The engine SHALL accept a native `cc_replace_face_to_plane` result as native ONLY when
it PASSES a mandatory self-verify: the candidate SHALL be a **closed watertight
2-manifold** with **positive enclosed volume**, the **moved face's samples SHALL lie on
the target plane** (`|n·(p − P_t.o)| ≤ tol`), it SHALL be a **single lump** (Euler χ =
2, the move did not sever the body), and the **face adjacency SHALL be preserved** (the
resolved face count equals the original — no neighbour inverted or removed, no new face
introduced), measured by the engine's existing `watertightVolume` /
`tessellate::isWatertight` audit over the mesher's deflection ladder. If the candidate
is NULL (a consumed verb declined) OR fails ANY check, the engine SHALL DISCARD it and
return EXACTLY the OCCT fallback (`OcctEngine::replace_face_to_plane`) result for that
call. The engine SHALL NEVER emit an unverified, leaky, inverted, wrong, or multi-lump
solid, and SHALL NEVER hand a native void to OCCT (a move-face operand is always
OCCT-reconstructible, so OCCT is the true fall-through).

#### Scenario: A non-watertight, inverted, multi-lump, or NULL candidate is discarded (host)

- GIVEN a native re-solve candidate that is NULL (a consumed verb declined) OR open / non-manifold / zero-volume / multi-lump / with a neighbour inverted or removed, with the native engine active
- WHEN the re-solve self-verify is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL fall through to `OcctEngine::replace_face_to_plane`, emitting NO leaky, wrong, or inverted solid

#### Scenario: The native move-face result matches the OCCT oracle on a booted simulator (sim)

- GIVEN a reachable fixture (axis-aligned box face moved to a parallel or tilted target plane) on a booted iOS simulator with OCCT available as the oracle
- WHEN the native `cc_replace_face_to_plane` result is compared against the OCCT move-face oracle (`OcctEngine::replace_face_to_plane` for a trim; the equivalent OCCT plane-cut-and-extend / box of new extents for a grow) for the same body, face, and target plane
- THEN the volume, area, watertightness (closed 2-manifold), topology (Euler χ = 2, single solid), and bounding box SHALL match within the landed direct-modeling tolerances, with no tolerance widened

### Requirement: Curved-neighbour, topology-changing, non-convex, degenerate, and foreign move-face cases fall through to OCCT

The native `replace_face_to_plane` branch SHALL DECLINE (return a NULL result → the OCCT
fallback) for any case outside the reachable convex-planar domain, each labelled and
verified as a fall-through and NEVER faked: (1) a **curved neighbour** — a face adjacent
to the picked face is a `Cylinder`/`Cone`/`Sphere`/`BSpline`/`Bezier`, which the planar
re-solve cannot retrim to the moved plane; (2) a **topology-changing target plane** — a
plane that clips PAST an adjacent face (fully removes a neighbour), inverts a neighbour,
severs the body into more than one lump, or introduces a new face; (3) a **non-convex
operand** — a concave planar solid the half-space intersection would not reproduce; (4)
a **degenerate** target — coincident with the picked face (`|d̄| ≤ tol`, a no-op) or
leaving a zero-volume sliver / empty solid; (5) a **non-planar picked face**; (6) a
**foreign / mesh-only** body with no native B-rep to re-solve. When the native engine is
active, each such case SHALL produce EXACTLY the result of the same call under the OCCT
engine (`cc_set_engine(0)`), proving fall-through with no native interception. The change
SHALL NOT fake, stub-out, or partially implement any declined case; a correct decline is
a first-class, measured outcome.

#### Scenario: A curved neighbour declines to OCCT (host + parity)

- GIVEN a native solid whose picked planar face is adjacent to a cylindrical / conical / spherical / spline face, with the native engine active
- WHEN `cc_replace_face_to_plane` is invoked
- THEN the native branch SHALL return a NULL result (the curved side cannot be retrimmed to the moved plane) AND the result SHALL be identical to invoking the same call under `cc_set_engine(0)` (the OCCT oracle), proving fall-through

#### Scenario: A topology-changing or non-convex move declines to OCCT (host)

- GIVEN a target plane that severs the solid into more than one lump, inverts / removes / adds a neighbour face, or a non-convex operand whose half-space intersection would not reproduce the concavity, with the native engine active
- WHEN `cc_replace_face_to_plane` is invoked
- THEN the native branch SHALL return a NULL result (rather than emit a wrong, leaky, inverted, or multi-lump solid) AND the engine SHALL fall through to `OcctEngine::replace_face_to_plane`

#### Scenario: A degenerate, non-planar-face, or foreign move declines to OCCT (host)

- GIVEN a target plane coincident with the picked face (a no-op) or leaving a zero-volume sliver, OR a non-planar picked face, OR a foreign / mesh-only body with no native B-rep, with the native engine active
- WHEN `cc_replace_face_to_plane` is invoked
- THEN the native branch SHALL return a NULL result AND the engine SHALL fall through to `OcctEngine::replace_face_to_plane`, emitting no wrong or leaky solid
