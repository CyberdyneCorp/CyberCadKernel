# native-ssi

Add the two SSI Stage **S4 CLASSIFICATION LAYERS** (`openspec/SSI-ROADMAP.md` S4 — the
moat) on top of the shipped S1 analytic / S2 seeding / S3 marching stack:

- **S4-a** — a native, OCCT-free **coincident / overlapping-surface detector** that
  returns a TYPED `CoincidentRegion` (`FullSurfaceSame` | `OverlapSubRegion{param
  bounds on each surface}`) — generalising the existing analytic `Coincident` detection
  across the elementary families and ADDING seeded-path patch-agreement detection — or
  `Undecided` when a region cannot be robustly delimited.
- **S4-b** — a native **typed tangent-contact classifier** that types a contact as
  exactly one of `TransversalOnly` | `TangentPoint` | `TangentCurve` |
  `NearTangentTransversal`, emitting the point/curve where determinable, replacing the
  blunt `SeedSet.deferredTangent` counter with a typed `TangentContact` per dropped
  region, and typing the S3 `nearTangentGaps` hand-off.

This change is **DETECTION + CLASSIFICATION ONLY**. It does NOT march through a tangency
and does NOT fabricate a curve across a degeneracy — that is **S4-c**, explicitly OUT OF
SCOPE. A `NearTangentTransversal` is classified and handed on (an S4-c gap → OCCT),
never traced through. The tracer is UNCHANGED except for an additive typed stop reason.
Native classification returns **undecided/empty → OCCT** on every non-robust decision;
the ENGINE owns the OCCT fallback + self-verify. SSI is INTERNAL: **no `cc_*` ABI
change**; `src/native/**` stays OCCT-free; the seeded-path parts are compiled under
`CYBERCAD_HAS_NUMSCI` (like S2/S3).

## ADDED Requirements

### Requirement: Native typed coincident / overlapping-surface detection (S4-a)

The kernel SHALL provide a native, **OCCT-free** coincident-surface detector in
`cybercad::native::ssi` that returns a TYPED `CoincidentRegion` describing the shared
locus of two native surfaces, classified as exactly one of `FullSurfaceSame` (the two
surfaces are the same locus), `OverlapSubRegion` (they coincide on a delimited parameter
sub-region, carrying the param bounds on EACH surface), `None` (not coincident), or
`Undecided` (coincidence suspected but the region cannot be robustly delimited).

On the **analytic path**, the detector SHALL decide `FullSurfaceSame` in closed form
from the surface frames and sizes for ALL elementary families — same plane (same normal
up to sign AND same signed offset), coaxial-equal cylinder, coaxial-equal cone (same
apex, collinear axis, equal half-angle), same sphere (same centre, equal radius), same
torus (same centre and axis, equal major and minor radius) — generalising the existing
`IntersectionStatus::Coincident` detection into a complete, consistent family while
keeping the shipped same-sphere and coaxial-equal-cylinder `Coincident` results
bit-identical. On the **seeded path** (`CYBERCAD_HAS_NUMSCI`), the detector SHALL detect
that two general/quadric surfaces coincide over a PATCH by verifying that BOTH the
on-both-surfaces point residual and the surface normals agree over a sampled sub-region
(not merely at isolated seeds), delimit the agreement boundary in parameter space, and
return `OverlapSubRegion` with those bounds — SUPPRESSING spurious seeds and marching
inside it. When the overlap boundary cannot be robustly delimited (partial agreement,
fuzzy boundary, ambiguous domain-edge touch) the detector SHALL return `Undecided` (→
OCCT) and SHALL NOT fabricate a region. `src/native/**` SHALL NOT link OCCT; no `cc_*`
entry point, signature, or POD struct SHALL be added or changed.

#### Scenario: analytic identical elementary surfaces classify FullSurfaceSame

- GIVEN two native elementary surfaces of the same kind occupying the same locus — same
  plane, coaxial-equal cylinder, coaxial-equal cone, same sphere, or same torus
- WHEN the S4-a detector classifies the pair
- THEN it SHALL return `CoincidentRegion` of kind `FullSurfaceSame`
- AND `intersect_surfaces` SHALL continue to report `IntersectionStatus::Coincident` for
  those pairs, now backed by the typed region, with the previously-shipped same-sphere
  and coaxial-equal-cylinder results unchanged

#### Scenario: a near-miss of an identical pair classifies None, not a false coincidence

- GIVEN a pair that is close to identical but shifted, rotated, or resized beyond the
  linear/angular tolerance (e.g. two spheres with slightly different radii, or two
  planes with a small offset)
- WHEN the S4-a detector classifies the pair
- THEN it SHALL return `CoincidentRegion` of kind `None` — it SHALL NOT report a
  coincidence for a pair that is not the same locus within tolerance

#### Scenario: two coincident freeform patches yield a delimited OverlapSubRegion

- GIVEN two freeform (Bézier / B-spline / NURBS) surface patches that coincide over a
  sub-rectangle of their parameter domains, with `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-a seeded detector runs over the candidate region
- THEN it SHALL verify that both the on-both residual and the normals agree across the
  sampled sub-region, delimit the agreement boundary, and return `CoincidentRegion` of
  kind `OverlapSubRegion` carrying the parameter bounds on each surface
- AND it SHALL suppress spurious seeds and marching inside the delimited region

#### Scenario: an undelimitable overlap returns Undecided, not a guessed region

- GIVEN two surfaces whose coincident overlap boundary is partial or fuzzy, or touches a
  domain edge ambiguously, so the region cannot be robustly delimited
- WHEN the S4-a seeded detector attempts to delimit it
- THEN it SHALL return `CoincidentRegion` of kind `Undecided` (the engine falls back to
  OCCT, reported) — it SHALL NOT fabricate a region boundary

### Requirement: Native typed tangent-contact classification (S4-b)

The kernel SHALL provide a native, **OCCT-free** tangent-contact classifier in
`cybercad::native::ssi` that, given a surface pair (analytic) or a seeded solution where
‖n₁ × n₂‖ < `SeedOptions.tangentSinTol`, returns a TYPED `TangentContact` classifying
the contact as exactly one of `TransversalOnly` (no tangency — the normal path handles
it), `TangentPoint` (an isolated 0-dimensional contact — the point SHALL be emitted),
`TangentCurve` (the surfaces are tangent along a whole curve — the curve SHALL be
emitted where closed-form, else its existence flagged with the contact locus),
`NearTangentTransversal` (the surfaces graze but CROSS — an S4-c gap handed on to OCCT,
NOT traced through here), or `Undecided` (the local jet is ambiguous — → OCCT).

On the **analytic path**, tangent configurations SHALL be classified in closed form —
sphere∩sphere at `d = R₁+R₂` (external) or `d = |R₁−R₂|` (internal) → `TangentPoint`
carrying the centre-line contact point; coaxial sphere∩cylinder / sphere∩cone tangent
equator and a plane tangent along a cylinder ruling → `TangentCurve` carrying the tangent
circle / line; a plane tangent to a sphere → `TangentPoint`. Analytic tangency SHALL NOT
return `NearTangentTransversal` or `Undecided`. On the **seeded path**
(`CYBERCAD_HAS_NUMSCI`), the classifier SHALL type the contact by the LOCAL DIFFERENTIAL
GEOMETRY — the sign/rank structure of the relative second fundamental form (relative
normal curvature) in the shared tangent plane: sign-definite → `TangentPoint`; rank-1
(one near-zero eigenvalue) → `TangentCurve`; indefinite (grazes and crosses) →
`NearTangentTransversal`; within the model-scale-derived curvature-noise band →
`Undecided`. The classifier SHALL replace the blunt `SeedSet.deferredTangent`
increment with a typed `TangentContact` recorded per dropped near-tangent region, and
SHALL KEEP the `deferredTangent` integer as a compatibility summary count. It SHALL NOT
fabricate a seed for any tangent contact, SHALL NOT trace through a
`NearTangentTransversal` (that is S4-c → OCCT), and SHALL NOT hand-tune or weaken the
curvature band to force a verdict. `src/native/**` SHALL NOT link OCCT; no `cc_*` entry
point SHALL be added or changed.

#### Scenario: analytic tangent spheres classify TangentPoint and emit the point

- GIVEN two spheres at centre distance `d = R₁ + R₂` (externally tangent) or `d =
  |R₁ − R₂|` (internally tangent)
- WHEN the S4-b analytic classifier classifies the pair
- THEN it SHALL return `TangentContact` of type `TangentPoint` carrying the contact
  point, which SHALL lie on BOTH spheres within tolerance

#### Scenario: analytic surfaces tangent along a curve classify TangentCurve and emit the curve

- GIVEN a coaxial sphere∩cylinder tangent at its equator (cylinder radius equal to the
  sphere radius), or a plane tangent along a cylinder ruling
- WHEN the S4-b analytic classifier classifies the pair
- THEN it SHALL return `TangentContact` of type `TangentCurve` carrying the tangent
  curve (the equator circle / the ruling line), which SHALL lie on BOTH surfaces within
  tolerance

#### Scenario: a seeded near-tangent solution is typed by its relative normal curvature

- GIVEN a seeded refine solution with ‖n₁ × n₂‖ < `tangentSinTol`, with
  `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-b seeded classifier evaluates the relative second fundamental form at the
  contact
- THEN it SHALL return `TangentPoint` when the form is sign-definite, `TangentCurve` when
  it is rank-1, `NearTangentTransversal` when it is indefinite, and `Undecided` when it
  is within the curvature-noise band
- AND the typed `TangentContact` SHALL be recorded on the `SeedSet` while
  `deferredTangent` is kept as a compatibility summary count, with NO seed fabricated

#### Scenario: a near-tangent transversal is classified and handed on, never traced through

- GIVEN a seeded near-tangent contact whose relative second fundamental form is
  indefinite (the surfaces graze but cross)
- WHEN the S4-b classifier types it
- THEN it SHALL return `TangentContact` of type `NearTangentTransversal` and hand it on
  as an S4-c gap (→ OCCT) — it SHALL NOT step through the tangency and SHALL NOT
  fabricate any intersection-curve points across it

#### Scenario: an ambiguous local jet returns Undecided, not a guessed type

- GIVEN a seeded near-tangent contact whose relative second fundamental form is within
  the model-scale-derived curvature-noise band (the type is not robustly decidable)
- WHEN the S4-b classifier types it
- THEN it SHALL return `TangentContact` of type `Undecided` (the engine falls back to
  OCCT, reported) — it SHALL NOT guess `TangentPoint`, `TangentCurve`, or
  `NearTangentTransversal`

### Requirement: The S3 near-tangent hand-off carries the typed contact (additive)

The S3 marching tracer (`src/native/ssi/marching.h`) SHALL carry the typed
`TangentContact` classification of WHY a march stopped at a tangency, additively,
WITHOUT changing the tracer's transversal-only stepping. A `WLine` whose `status` is
`TraceStatus::NearTangent` SHALL carry an optional `TangentContact` stop reason (computed
by the S4-b seeded classifier at the stop point); `TraceSet.nearTangentGaps` SHALL be
unchanged as a count. The tracer SHALL still stop AT the tangency and march only up to it
— it SHALL NOT step through the tangency (that is S4-c, out of scope). No `cc_*` entry
point SHALL be added or changed, and the marching entry points SHALL remain compiled
under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a march that stops at a tangency reports the typed reason

- GIVEN a transversal march that runs into a near-tangent region and stops with `status
  == TraceStatus::NearTangent`
- WHEN the tracer finalises the `WLine`
- THEN the `WLine` SHALL carry a typed `TangentContact` stop reason classifying the
  tangency (point / curve / near-tangent transversal), and `TraceSet.nearTangentGaps`
  SHALL still count that gap
- AND the number of traced nodes SHALL be unchanged from the pre-typing behaviour — no
  points SHALL have been fabricated past the tangency
