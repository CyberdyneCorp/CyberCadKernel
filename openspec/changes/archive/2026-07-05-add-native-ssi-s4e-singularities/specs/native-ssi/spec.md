# native-ssi

Add the FIRST SSI Stage **S4-e CHART-SINGULARITY** slice (`openspec/SSI-ROADMAP.md` S4 — the
moat, degeneracy robustness) on top of the shipped S1 analytic / S2 seeding / S3 marching
stack, the S4-a/S4-b classification layers, the S4-c near-tangent marching-through slice, and
the S4-d branch-point slice:

- **S4-e** — a native, OCCT-free capability that, when a marched intersection curve crosses a
  SINGLE SURFACE's own PARAMETRIC SINGULARITY (a sphere parametric pole `v = ±π/2` where `‖dU‖`
  collapses to zero because of the `cos v` factor, or a cone apex where the signed radius
  `R₀ + v·sin α = 0` so `‖dU‖` collapses) — where the 3D point and the surface NORMAL are
  well-defined but the `(u,v)` CHART degenerates and the S3 marcher TRUNCATES (a half-traced
  great circle; an apex step-collapse) — DETECTS the chart collapse via a SINGLE-surface
  Jacobian rank-drop, STEPS across the singular band with a point-based / chart-clamped
  corrector (using the finite point + normal), and RESUMES the normal `(u,v)` march on the far
  side, reporting a chart-singularities-crossed count.

This change is **additive** to the S3/S4-c/S4-d marcher. The transversal S3 trace, the S4-c
crossable-graze crossing, and the S4-d branch-point trace are UNCHANGED — the chart machinery
engages ONLY at a detected single-surface chart collapse (a NEW, independent `‖dU‖`-vs-`‖dV‖`
witness, DISTINCT from the S4-c pair transversality sine and the S4-d locus tangent flip). A
chart singularity that cannot be robustly CROSSED (the far-side node does not re-project on both
surfaces within tolerance; the crossing makes no far-side progress) STILL STOPS + defers → OCCT.
A genuine DOMAIN boundary `v`-edge (a finite cap — no `‖dU‖` collapse) STILL exits as a boundary,
and a genuine curve CUSP endpoint (the point-based step cannot continue) STILL ends. The tracer
NEVER fabricates a point across a singularity and NEVER weakens a tolerance; a singularity it
cannot cross is an honestly reported gap. SSI is INTERNAL: **no `cc_*` ABI change**;
`src/native/**` stays OCCT-free; the S4-e parts are compiled under `CYBERCAD_HAS_NUMSCI` (like
S2/S3/S4-c/S4-d). General / freeform parametric singularities, edge / higher-order cusps, and
self-intersection completeness (S4-f) remain OUT OF SCOPE.

## ADDED Requirements

### Requirement: Native chart-singularity detection and point-based crossing (S4-e)

The kernel SHALL provide a native, **OCCT-free** chart-singularity capability in
`cybercad::native::ssi` that, when the S3/S4-c/S4-d marcher reaches a point where a SINGLE
surface's own `(u,v)` PARAMETRIZATION is singular (a sphere parametric pole where `‖dU‖`
collapses to zero, or a cone apex where the signed radius and hence `‖dU‖` collapses), while the
3D point and the surface normal remain well-defined, DETECTS the chart collapse, STEPS across the
singular band, and RESUMES the normal march — rather than truncating at the singularity. The
capability SHALL DETECT the singularity via a SINGLE-surface Jacobian rank-drop — `‖dU‖`
collapsing relative to `‖dV‖` and the model scale on one surface while that surface's normal
stays finite — and this detection SHALL be INDEPENDENT of the S4-c pair transversality sine
`‖n₁ × n₂‖` (which need not collapse at a pole) and the S4-d locus tangent flip. It SHALL STEP
across the singular band with a POINT-BASED corrector that does NOT depend on the degenerate
`dU` (the fixed-plane cut whose residuals use only the surface point and the last-good tangent),
mapping the far-side chart coordinates back LOOSELY — at a sphere pole pinning the arbitrary
longitude from the continuity of the incoming arc and clamping the pole latitude, and at a cone
apex treating the apex as a single 3D point the curve passes through. It SHALL enter the band
with a fine step so the singularity is resolved rather than leapt, and SHALL resume the normal
`(u,v)` march once `‖dU‖` recovers on both surfaces. The chart singularities the marcher steps
across and verifies SHALL be reported (a per-`TraceSet` chart-singularities-crossed count and a
per-`WLine` crossed count) and SHALL NOT be counted in `nearTangentGaps`. Outside a detected
chart singularity the transversal S3 trace, the S4-c crossable-graze crossing, and the S4-d
branch-point trace SHALL be unchanged. `src/native/**` SHALL NOT link OCCT; no `cc_*` entry
point, signature, or POD struct SHALL be added or changed; the marching entry points SHALL remain
compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the sphere-pole great circle is now fully traced

- GIVEN a unit sphere and a plane through the sphere's axis (the plane `y = 0`), whose
  intersection is a great circle passing through BOTH sphere parametric poles (`v = ±π/2`, where
  `‖dU‖` collapses), forced through marching, whose S3 trace today stops at the pole as a
  `BoundaryExit` covering only a single pole-to-pole meridian (arc length ≈ π, half the closed
  loop), with `CYBERCAD_HAS_NUMSCI` built and chart-singularity handling enabled
- WHEN the S4-e marcher traces the pair
- THEN it SHALL DETECT the chart collapse at each pole, STEP across it with the point-based
  corrector (pinning the outgoing longitude from arc continuity), and RESUME the march on the
  opposite meridian — assembling the FULL closed great circle (arc length ≈ `2π` within the
  deflection tolerance, both meridians visited, both poles crossed), reporting at least two chart
  singularities crossed and yielding `nearTangentGaps == 0` for the traced curve
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the cone-apex line is now crossed

- GIVEN a double cone whose apex is at the origin (reference radius zero at the apex) and a plane
  through the apex (the plane `y = 0`), whose intersection is a line crossing the apex and
  spanning both nappes, forced through marching, whose S3 trace today stalls just short of the
  apex (`‖dU‖ → 0` collapses the parameter step, exhausting the node budget) and never reaches
  the far nappe, with chart-singularity handling enabled
- WHEN the S4-e marcher traces the pair
- THEN it SHALL DETECT the chart collapse at the apex, treat the apex as a single 3D point,
  STEP across it with the point-based corrector, and RESUME the march on the far nappe — tracing
  the full apex-crossing line spanning both nappes in a BOUNDED node count, reporting at least
  one chart singularity crossed
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the transversal march, the S4-c graze, and the S4-d branch trace are unchanged

- GIVEN a transversal surface pair whose intersection never reaches a chart singularity, a
  surface pair whose intersection is a `NearTangentTransversal` single-branch graze the S4-c
  slice marches through, and the Steinmetz bicylinder whose intersection self-crosses at two
  branch points the S4-d slice localizes and routes
- WHEN the S4-e marcher traces them
- THEN the transversal traced curve SHALL be identical to the S3 result (same nodes, same status,
  same counts), the graze SHALL still be MARCHED THROUGH by the S4-c crossing, and the Steinmetz
  SHALL still be assembled by the S4-d branch machinery (`branchPoints == 2`) — each with zero
  chart singularities crossed, because the chart machinery SHALL engage ONLY at a detected
  single-surface chart collapse

### Requirement: Chart-singularity honesty — genuine boundaries exit, cusps end, unresolved singularities defer (S4-e)

The S4-e marcher SHALL step across a `v`-edge only when it is a genuine PARAMETRIC SINGULARITY —
a sphere pole or cone apex where `‖dU‖` collapses while the surface normal stays finite. When the
`v`-edge is a genuine DOMAIN BOUNDARY (a finite surface's cap edge, where `‖dU‖` does NOT
collapse), the marcher SHALL let the curve EXIT as a boundary exit and SHALL NOT attempt a
crossing. When the intersection curve has a genuine CUSP endpoint (the curve velocity collapses
while both surfaces' charts are regular and the point-based step cannot continue through it), the
marcher SHALL let the curve END there and SHALL NOT fabricate a continuation. When a chart
singularity's far-side node cannot be re-projected onto both surfaces within `onSurfTol`, when the
crossing makes no real far-side progress, or when a pole's continuity-pinned outgoing node fails
the on-both-surfaces verification, the marcher SHALL DISCARD the crossing arc, STOP, record the
typed stop reason, count the region in `nearTangentGaps`, and DEFER it (→ OCCT) — reporting the
measured gap. The marcher SHALL NEVER fabricate a point across a singularity and SHALL NEVER
weaken a tolerance to force a crossing; a singularity that cannot be robustly crossed SHALL remain
an honestly reported gap.

#### Scenario: a genuine domain boundary still exits, never fabricating a crossing

- GIVEN a finite surface whose intersection curve runs to a real domain-boundary `v`-edge (a
  finite cylinder's cap edge, where `‖dU‖` does NOT collapse and there is no surface beyond the
  edge)
- WHEN the S4-e marcher reaches the edge
- THEN it SHALL let the curve EXIT as a boundary exit, reporting zero chart singularities crossed
- AND it SHALL NOT fabricate any curve node beyond the boundary

#### Scenario: a chart singularity that cannot be robustly crossed defers honestly

- GIVEN a detected chart singularity where either the far-side node does not re-project onto both
  surfaces within `onSurfTol`, the crossing makes no real far-side progress, or a pole's
  continuity-pinned outgoing node fails the on-both-surfaces verification
- WHEN the S4-e marcher attempts to step across it
- THEN it SHALL DISCARD the crossing arc, STOP at the singularity, record the typed stop reason,
  and count the region in `nearTangentGaps` (deferred → OCCT) — it SHALL NOT fabricate a point
  across the singularity and SHALL NOT weaken a tolerance to force the crossing
