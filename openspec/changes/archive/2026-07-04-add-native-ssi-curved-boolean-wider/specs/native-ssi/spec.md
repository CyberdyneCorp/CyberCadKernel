# native-ssi

Declare the single **closed** transversal `WLine` (one seam circle,
`nearTangentGaps == 0`, `status == Closed`) as the consumed input contract for the S5-c
sphere∩sphere curved boolean (`openspec/SSI-ROADMAP.md` S5), alongside the two-rim-seam
contract the archived S5-a change already declared for the through-drill cylinder pair.
The tracer is UNCHANGED — this is the already-shipped S3 output; `nearTangentGaps > 0`
(tangent / coincident spheres) remains the honest S4 boundary the boolean respects by
declining (→ OCCT). Internal capability: **no `cc_*` ABI change**.

## ADDED Requirements

### Requirement: The single closed TraceSet seam is the consumed input contract for the S5-c sphere∩sphere common

The S3 `cybercad::native::ssi` `TraceSet` SHALL be the input contract consumed by the
native S5-c sphere∩sphere common (`src/native/boolean/ssi_boolean.cpp buildLensCommon`):
for a transversal sphere∩sphere pair, the boolean SHALL obtain the `TraceSet` and use the
SINGLE `Closed` `WLine` — its per-node `(u1,v1,u2,v2)` on both spheres (the seam-circle
track used to fan each spherical cap) and its shared 3D nodes (the seam vertices both caps
weld on through the shared `VertexPool`) — to assemble the two-cap lens. The S5-c boolean
SHALL consume the `TraceSet` ONLY when it is fully transversal — `nearTangentGaps == 0`
and the consumed `WLine.status` is `Closed`; a `TraceSet` with `nearTangentGaps > 0`, a
`NearTangent` / `Failed` WLine, or a seam that spans a full sphere (coincident) SHALL be
treated as the honest S4 fallback boundary and SHALL NOT be consumed (the boolean declines
→ OCCT). The tracer SHALL NOT change to serve this consumption — the contract is the
already-shipped S3 output; no `cc_*` entry point, signature, or POD struct SHALL be added
or changed, and the SSI module SHALL remain OCCT-free and compiled under
`CYBERCAD_HAS_NUMSCI` (like the S3 tracer).

#### Scenario: a single closed TraceSet seam is consumed to weld the lens
- GIVEN a transversal sphere∩sphere pair whose S3 `TraceSet` has `nearTangentGaps == 0`
  and exactly one `Closed` WLine
- WHEN the S5-c curved boolean consumes the `TraceSet`
- THEN it SHALL fan each spherical cap from its pole to the WLine's shared 3D seam nodes,
  with every seam node on both spheres ≤ tol, and the two caps SHALL weld along that single
  seam
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: a tangent / coincident sphere TraceSet is the S4 boundary, not consumed
- GIVEN a sphere∩sphere pair whose S3 `TraceSet` reports `nearTangentGaps > 0` (tangent
  spheres) or whose single WLine is `NearTangent` / spans a full sphere (coincident)
- WHEN the S5-c curved boolean inspects the `TraceSet`
- THEN it SHALL decline to consume the trace (the honest S4 seam) and the boolean SHALL
  fall back to OCCT, reported — never welding a lens on a truncated or fabricated seam
