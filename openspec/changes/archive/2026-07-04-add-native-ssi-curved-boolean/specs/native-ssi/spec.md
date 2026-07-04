# native-ssi

Declare the shipped S3 `TraceSet` (one `WLine` per transversal branch, each node
carrying `(u1,v1,u2,v2)` on both surfaces + a fitted B-spline + a per-curve
`TraceStatus`, with `nearTangentGaps` the S4 seam) as the **consumed input contract for
S5-a curved booleans** (`openspec/SSI-ROADMAP.md` S5). Each transversal `WLine` splits a
curved face; `nearTangentGaps > 0` is the honest boundary the S5-a boolean respects by
declining (→ OCCT) rather than consuming a non-transversal trace. Internal capability:
**no `cc_*` ABI change**.

## ADDED Requirements

### Requirement: The S3 TraceSet is the consumed input contract for S5-a curved booleans

The S3 `cybercad::native::ssi` `TraceSet` SHALL be the **input contract consumed by the
native S5-a curved boolean** (`src/native/boolean`): for a transversal elementary curved
face pair, the boolean SHALL obtain the `TraceSet` and use each transversal `WLine` — its
per-node `(u1,v1,u2,v2)` on both surfaces (the UV split track) and its fitted B-spline
(the shared seam edge) — to split the curved faces. The S5-a boolean SHALL consume a
`TraceSet` ONLY when it is fully transversal — `nearTangentGaps == 0` and every consumed
`WLine.status` is `Closed` or `BoundaryExit`; a `TraceSet` with `nearTangentGaps > 0`, or
any `NearTangent` / `Failed` WLine, SHALL be treated as the honest **S4 fallback
boundary** and SHALL NOT be consumed (the boolean declines → OCCT). The tracer SHALL NOT
change to serve this consumption — the contract is the already-shipped S3 output; no
`cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI module
SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like the S3 tracer).

#### Scenario: a fully-transversal TraceSet is consumed to split a curved face
- GIVEN a transversal elementary curved face pair whose S3 `TraceSet` has
  `nearTangentGaps == 0` and every WLine `Closed` or `BoundaryExit`
- WHEN the S5-a curved boolean consumes the `TraceSet`
- THEN it SHALL split each curved face along its WLine's `(u,v)` track and use the WLine's
  fitted B-spline as the shared seam edge, with every seam node on both surfaces ≤ tol
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: a non-transversal TraceSet is the S4 fallback boundary, not consumed
- GIVEN an intersecting curved face pair whose S3 `TraceSet` reports
  `nearTangentGaps > 0` (or a consumed WLine is `NearTangent` / `Failed`)
- WHEN the S5-a curved boolean inspects the `TraceSet`
- THEN it SHALL decline to consume the trace (the honest S4 seam) and the boolean SHALL
  fall back to OCCT, reported — never splitting a face on a truncated or fabricated seam
