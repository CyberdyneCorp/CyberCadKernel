# Design — moat-m2c-freeform-point-in-solid (MOAT M2c / B3, first slice)

## Context

`src/native/boolean/ssi_boolean.h:247` `classifyPoint(const CurvedSolid&, p, tol)`
answers point-in-solid ONLY for an analytic `CurvedSolid` — a solid whose curved
wall folds to ONE elementary surface (cylinder/sphere/cone) plus planar caps, using
the closed-form radial/spherical/conic signed distance ∧ cap half-spaces. The moment
an operand carries a `Kind::BSpline`/`Kind::Bezier` face, `recogniseCurvedSolid`
returns `nullopt` and there is no membership test — B3.

The landed MOAT M0 mesher (`src/native/tessellate/face_mesher.h`
`trimmedFreeformMesh`, assembled by `SolidMesher::mesh` in `solid_mesher.h`)
produces a **watertight boundary triangle `Mesh`** (`mesh.h`: `vertices` fp64 +
`triangles` 0-based, wound CCW outward) for a genuinely-trimmed freeform-walled
solid. A watertight, outward-wound triangulation is precisely the substrate a
ray-cast / winding membership test consumes. This slice builds that test.

## Goals / Non-goals

**Goals**
- A native, OCCT-free `IN`/`OUT`/`ON`/`UNKNOWN` classifier for a query point against
  the M0 boundary mesh of a **single freeform-face solid**, robust for points
  **comfortably away from the ON band**.
- Prove it (host) against closed-form truth on a freeform-walled solid whose
  inside/outside is analytically known, and (sim) against OCCT
  `BRepClass3d_SolidClassifier` on N random points.
- Never emit a wrong classification silently: near-ON/near-tangent → honest `ON`,
  ambiguous ray consensus → honest `UNKNOWN`.

**Non-goals (this slice)**
- Robust ON-boundary / near-tangent classification (the asymptotic tail) — points in
  the band resolve to `ON`/`UNKNOWN` by design.
- Multi-freeform-face solids, thin walls, self-intersecting or non-watertight input.
- Wiring the classifier into the S5 boolean assembler, or B1/B2/B4/B5.
- Any change to `classifyPoint`, `recogniseCurvedSolid`, or the tessellator.

## Decisions

### D1 — Ray-cast odd/even crossings over winding number
Both are valid; ray parity is chosen because it needs only a watertight surface (no
consistent global orientation lookup beyond winding) and maps directly to the
`BRepClass3d_SolidClassifier` oracle's IN/OUT. Each ray is intersected with every
boundary triangle by **Möller–Trumbore** (branch-light, fp64). Odd forward
crossings ⇒ inside; even ⇒ outside. Winding-number is kept as an optional
cross-check for the host gate but is not the shipping path.

### D2 — Multi-ray consensus for degeneracy robustness
A single ray can graze a shared edge or vertex (two adjacent triangles report a
crossing at the same `t`, double-counting parity), or run parallel to a face. The
classifier casts a small budget of independent, **non-axis-aligned, mutually
non-parallel** directions (e.g. a fixed low-discrepancy set), and:
- discards any ray whose crossings include a near-degenerate hit (barycentric
  coordinate within `edgeEps` of an edge, or `|ray·triNormal|` below a grazing
  threshold) — that ray is UNUSABLE, not miscounted;
- takes the **majority parity** across usable rays. If usable rays disagree, or
  fewer than a quorum remain usable, the verdict is `UNKNOWN` (decline this point)
  — never a coin-flip.
For points comfortably away from the ON band (this slice's domain) rays are
overwhelmingly clean and consensus is unanimous; the machinery exists so an
occasional grazing ray declines instead of lying.

### D3 — Principled ON-band from min-distance-to-surface
Before/alongside ray casting, compute `dmin = min over triangles of
pointTriangleDistance(p, tri)`. If `dmin ≤ band`, the verdict is `ON`. The band is
**scale-relative and never weakened**: it is sized to the mesh's own fidelity —
`band = max(absTol, relTol·diag) + meshDeflection`, where `diag` is the solid's
bounding-box diagonal and `meshDeflection` the deflection the M0 mesh was built at
(the meshed surface is only within `deflection` of the true surface, so a point
within `deflection` of a triangle cannot be crisply classified). This is the honest
ON: the point is within the mesh's resolution of the boundary. Away from the band,
`dmin` is large and the verdict is the ray parity.

### D4 — Consume M0, never rewrite it
The classifier takes a `const tessellate::Mesh&` (already welded/watertight) plus the
solid's bbox/deflection metadata. The caller obtains it via the existing
`tessellate::SolidMesher::mesh(shape)`. No tessellator code is touched; the classifier
is a pure read-only consumer. (The mesh's watertightness is a PRECONDITION the caller
asserts via the existing `mesh.h` `isWatertight`; a non-watertight mesh is out of
scope → `UNKNOWN`.)

### D5 — Host-analytic gate: a freeform wall with closed-form truth
To test IN/OUT against ground truth with NO OCCT, build a freeform-walled solid whose
membership is known analytically. The construction (`src/native/construct`) can
revolve/extrude a B-spline profile; a B-spline wall that EXACTLY coincides with a
cylinder of radius `r` (a straight, axis-parallel B-spline generatrix) yields a
`Kind::BSpline` face whose true inside/outside is the closed-form cylinder half-space
`‖radial‖ < r` ∧ cap half-spaces. The test meshes this solid with M0, then asserts
`classify(p)` matches `sign(cylinderHalfSpace(p) ∧ caps)` on sample points chosen to
sit comfortably away from the ON band (well inside, well outside). This exercises the
freeform mesh path (the face IS `Kind::BSpline`, routed through `trimmedFreeformMesh`)
while keeping an analytic oracle. If a B-spline-that-is-a-cylinder is not faithfully
constructible/meshable, a genuinely-curved B-spline wall whose enclosed region has a
closed-form membership (e.g. an extruded convex B-spline profile with an analytic
point-in-2D-region test extruded along z) is the fallback fixture.

### D6 — Sim parity gate: OCCT `BRepClass3d_SolidClassifier`
On a booted simulator with OCCT linked, build the SAME trimmed-freeform-walled solid,
mesh with M0, and for N random points in a bbox-enlarged sampling box compare
`classify(p)` to `BRepClass3d_SolidClassifier(occtSolid).State(p, tol)` mapped
`TopAbs_IN→IN`, `TopAbs_OUT→OUT`, `TopAbs_ON→ON`. A point OCCT calls `ON`, or that
the native classifier calls `ON`/`UNKNOWN`, is counted as agreement when the other
side is within the band (the tolerance band, not a weakened tolerance). Any crisp
IN↔OUT disagreement is a FAILURE (0 silent wrong tolerated). The pass criterion is
recorded as `N passed / 0 crisp-disagreements / k in-band-or-declined`.

## Algorithm (per query point)

```
classify(mesh, bbox, deflection, p):
  band = max(absTol, relTol * diag(bbox)) + deflection
  dmin = min_tri pointTriangleDistance(p, tri)
  if dmin <= band: return ON
  usable = 0; insideVotes = 0
  for dir in rayDirections:            # fixed non-axis, mutually non-parallel set
    (parity, degenerate) = rayParity(mesh, p, dir)
    if degenerate: continue            # grazed an edge/vertex → discard ray
    usable += 1; insideVotes += parity # parity in {0,1}
  if usable < quorum: return UNKNOWN
  if insideVotes == 0: return OUT
  if insideVotes == usable: return IN
  return UNKNOWN                        # rays disagree → honest decline
```

`rayParity` counts Möller–Trumbore forward hits (`t > eps`), flagging `degenerate`
when any hit's barycentric coord is within `edgeEps` of an edge or `|dir·n|` is below
the grazing threshold.

## Complexity

The per-point classifier is a linear scan over triangles (crossing count +
min-distance) times a small fixed ray budget — cognitive complexity target: backend
band (≤15). The Möller–Trumbore kernel and point-triangle-distance kernel are small,
isolated, individually testable pure functions. No spatial acceleration in this slice
(fixture solids are small); a BVH is a later optimisation, out of scope.

## Risks / Open questions

- **R1 — M0 mesh watertightness at the sampling scale.** If the chosen fixture's M0
  mesh has any residual open seam, ray parity is unreliable. Mitigated by asserting
  `isWatertight` as a precondition and by D5 choosing a fixture M0 already meshes
  watertight (a cylinder-coincident or convex-extrude B-spline wall). If not
  achievable → DECLINE with the measured open-edge count.
- **R2 — band vs OCCT tol calibration.** The native band and OCCT's `State` tol must
  be reconcilable so in-band points are not spuriously counted as crisp
  disagreements. Recorded explicitly in the sim gate; never widened to hide a real
  IN↔OUT error.
- **R3 — sampling into the band by chance.** Random sim points may land in the band;
  those are counted as in-band-or-declined, not failures — only crisp IN↔OUT
  mismatches fail. Comfortably-away points are the graded correctness signal.
