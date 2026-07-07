# native-booleans

## ADDED Requirements

### Requirement: Native point-in-freeform-solid membership by ray-cast over the M0 boundary mesh

The native boolean library SHALL provide an OCCT-free, header-only classifier
`classifyPointInMesh(boundary, bbox, meshDeflection, p, tol)` that, given the
**watertight boundary triangle mesh** of a solid whose boundary includes a genuinely
trimmed freeform (`Kind::BSpline` / `Kind::Bezier`) face — the mesh produced by the
landed MOAT M0 tessellator (`src/native/tessellate` `SolidMesher::mesh` /
`trimmedFreeformMesh`, CONSUMED read-only, NOT rewritten) — and an arbitrary query
point `p`, returns one of `IN`, `OUT`, `ON`, or `UNKNOWN`. The classifier SHALL:

- decide inside/outside by **odd/even ray crossings** — casting a ray from `p` and
  counting forward triangle intersections (Möller–Trumbore, fp64); an ODD count ⇒
  `IN`, an EVEN count ⇒ `OUT`;
- use **multi-ray consensus**: cast a fixed set of independent, non-axis-aligned,
  mutually non-parallel ray directions, DISCARD any ray with a near-degenerate
  crossing (a hit within an edge/vertex tolerance, or a grazing ray nearly parallel
  to a face), and take the MAJORITY parity of the usable rays; if fewer than a quorum
  of rays are usable, or the usable rays DISAGREE, return `UNKNOWN` (an honest
  decline) rather than a guessed IN/OUT;
- report `ON` when the minimum distance from `p` to any boundary triangle is within a
  **scale-relative ON-band** `band = max(absTol, relTol·diag(bbox)) + meshDeflection`
  — the point is within the mesh's own deflection + tolerance resolution of the
  boundary and cannot be crisply classified;
- treat a **non-watertight** input mesh as out of scope and return `UNKNOWN` (never a
  fabricated verdict).

This classifier SHALL be strictly ADDITIVE: it SHALL NOT modify the existing analytic
`classifyPoint` (`src/native/boolean/ssi_boolean.h`) or `recogniseCurvedSolid`, SHALL
NOT modify `src/native/tessellate/**`, SHALL introduce NO `cc_*` ABI surface, and
SHALL add ZERO OCCT includes under `src/native/**`. It SHALL NEVER emit a wrong
classification silently: near-boundary / near-tangent points resolve to `ON`, and
ambiguous ray consensus resolves to `UNKNOWN`. This first slice targets the SIMPLEST
reachable case — a **single freeform-face solid**, with correctness graded on points
**comfortably away from the ON band**; robust ON-boundary / near-tangent
classification is an explicitly deferred asymptotic tail.

#### Scenario: Points comfortably inside/outside a freeform-walled solid classify IN/OUT against analytic truth (host)

- GIVEN a freeform-walled solid whose curved boundary is a `Kind::BSpline` face whose inside/outside is KNOWN in closed form (e.g. a B-spline wall coincident with a cylinder of radius `r`, or a convex-extruded-B-spline profile), built on the host with NO OCCT and meshed watertight by the M0 `SolidMesher`
- WHEN `classifyPointInMesh` is evaluated at sample points placed COMFORTABLY away from the ON band (well inside and well outside the closed-form region)
- THEN every verdict SHALL equal the closed-form analytic truth (`IN` for interior points, `OUT` for exterior points), with NO crisp verdict emitted for any point inside the band

#### Scenario: A point within the deflection+tolerance band resolves to ON, not a crisp IN/OUT (host)

- GIVEN the same freeform-walled solid meshed at deflection `d`, and a query point whose minimum distance to the boundary mesh is within `band = max(absTol, relTol·diag) + d`
- WHEN `classifyPointInMesh` is evaluated at that point
- THEN the verdict SHALL be `ON` — the point is within the mesh's own resolution of the surface — and SHALL NOT be reported as a fabricated crisp `IN` or `OUT`

#### Scenario: An ambiguous or grazing ray arrangement declines to UNKNOWN rather than guessing (host)

- GIVEN a query point comfortably away from the boundary but positioned so that a candidate ray grazes a shared edge/vertex or runs nearly parallel to a face
- WHEN `classifyPointInMesh` casts its ray set and cannot reach a usable-ray quorum with agreeing parity
- THEN the verdict SHALL be `UNKNOWN` (an honest decline), and NO wrong crisp classification SHALL be emitted (0 silent wrong)

#### Scenario: The native classifier agrees with OCCT BRepClass3d_SolidClassifier on N random points (sim)

- GIVEN a trimmed-freeform-walled solid built BOTH natively and as an OCCT solid on a booted iOS simulator with OCCT linked, the native one meshed watertight by the M0 `SolidMesher`
- WHEN `classifyPointInMesh` is evaluated at N random points in a bounding-box-enlarged sampling box and compared to `BRepClass3d_SolidClassifier(occt).State(p, tol)` (mapping `TopAbs_IN→IN`, `TopAbs_OUT→OUT`, `TopAbs_ON→ON`)
- THEN there SHALL be ZERO crisp `IN`↔`OUT` disagreements; a point one side calls `ON`/`UNKNOWN` and the other places within the ON band SHALL count as agreement (the tolerance band, NOT a weakened tolerance); and the run SHALL be recorded as `N passed / 0 crisp-disagreements / k in-band-or-declined`

### Requirement: Point-in-freeform-solid membership self-verifies or DECLINES with a measured gap

The point-in-freeform-solid classifier SHALL be accepted as a landed capability ONLY
when BOTH mandatory gates pass with zero regression: (a) the HOST-ANALYTIC gate — on
a freeform-walled solid whose membership is analytically known, the native verdicts
match the closed-form truth on points comfortably away from the ON band with NO OCCT
linked; and (b) the SIM native-vs-OCCT gate — the native verdicts agree with
`BRepClass3d_SolidClassifier` on a batch of N random points within the tolerance
band, with ZERO crisp IN↔OUT disagreements. If EITHER gate cannot be met robustly —
the M0 mesh of the chosen fixture is not watertight enough, ray consensus is
unreliable away from the ON band, or crisp disagreements remain — the change SHALL
DECLINE HONESTLY: it SHALL record the MEASURED gap (which gate, how many of N
disagreed and by how much, or the residual open-edge count), SHALL leave the existing
analytic `classifyPoint` path UNCHANGED, and SHALL NOT land the classifier as a
passing capability. No tolerance SHALL be weakened to force a pass, OCCT SHALL remain
the oracle (never removed), and NO wrong classification SHALL be emitted at any point.
An honest decline with a specific measured blocker is a first-class, expected outcome.

#### Scenario: Both gates green with zero regression lands the classifier (success)

- GIVEN the host-analytic gate and the sim OCCT-parity gate for the single-freeform-face fixture
- WHEN both are run and the existing analytic `classifyPoint`/`recogniseCurvedSolid` and the tessellator are proven byte-identical vs `main`, with zero OCCT includes under `src/native/**` and no `cc_*` ABI change
- THEN the classifier SHALL be recorded as landed (first slice of B3), with the fixture, N, and the ON band documented in `openspec/MOAT-ROADMAP.md`

#### Scenario: An unreachable gate is reported as an honest decline, not a weakened pass (decline)

- GIVEN robust membership cannot be achieved for the trimmed-freeform fixture (e.g. the M0 mesh retains open edges, or crisp IN↔OUT disagreements against OCCT remain)
- WHEN the gates are evaluated
- THEN the change SHALL report the MEASURED gap (the failing gate and its quantified shortfall), SHALL leave the analytic `classifyPoint` path as-is, SHALL NOT weaken any tolerance, and SHALL NOT emit any wrong classification — the decline is the recorded outcome
