# native-ssi

Extend the S5-c input contract from the sphere∩sphere COMMON to ALL THREE transversal
sphere∩sphere boolean ops. The SAME single closed seam circle already consumed by
`buildLensCommon` is the input to the new FUSE (`buildLensFuse`) and CUT (`buildLensCut`)
builders: the seam splits each sphere into an inner and an outer cap, and COMMON / FUSE /
CUT differ only in which caps survive and their orientation — not in the trace. The tracer
does not change; the near-tangent fallback boundary is unchanged.

## MODIFIED Requirements

### Requirement: The single closed TraceSet seam is the consumed input contract for the S5-c sphere∩sphere common

The S3 `cybercad::native::ssi` `TraceSet` SHALL be the input contract consumed by ALL THREE
native S5-c sphere∩sphere booleans (`src/native/boolean/ssi_boolean.cpp`): `buildLensCommon`
(COMMON), `buildLensFuse` (FUSE), and `buildLensCut` (CUT). For a transversal sphere∩sphere
pair, each boolean SHALL obtain the `TraceSet` and use the SINGLE `Closed` `WLine` — its
per-node `(u1,v1,u2,v2)` on both spheres (the seam-circle track used to fan each spherical
cap) and its shared 3D nodes (the seam vertices the caps weld on through the shared
`VertexPool`). The SAME seam SHALL split each sphere into an INNER cap (apex nearest the
other centre) and an OUTER cap (apex at the far pole); the ops differ ONLY in cap selection
and orientation: COMMON assembles the two INNER caps (the lens), FUSE the two OUTER caps
(the peanut), and CUT the OUTER cap of the minuend plus the INNER cap of the subtrahend
REVERSED (the scooped cavity). Each boolean SHALL consume the `TraceSet` ONLY when it is
fully transversal — `nearTangentGaps == 0` and the consumed `WLine.status` is `Closed`; a
`TraceSet` with `nearTangentGaps > 0`, a `NearTangent` / `Failed` WLine, or a seam that
spans a full sphere (coincident) SHALL be treated as the honest S4 fallback boundary and
SHALL NOT be consumed by any of the three ops (the boolean declines → OCCT). The tracer
SHALL NOT change to serve this consumption — the contract is the already-shipped S3 output;
no `cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI
module SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like the S3 tracer).

#### Scenario: a single closed TraceSet seam is consumed to weld the lens
- GIVEN a transversal sphere∩sphere pair whose S3 `TraceSet` has `nearTangentGaps == 0`
  and exactly one `Closed` WLine
- WHEN the S5-c curved boolean consumes the `TraceSet`
- THEN it SHALL fan each spherical cap from its pole to the WLine's shared 3D seam nodes,
  with every seam node on both spheres ≤ tol, and the two caps SHALL weld along that single
  seam
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: the one closed seam feeds fuse and cut as it already feeds common
- GIVEN a transversal sphere∩sphere pair whose S3 `TraceSet` is one `Closed` WLine with
  `nearTangentGaps == 0` and `(u1,v1,u2,v2)` per node
- WHEN the native boolean path runs `Op::Fuse` or `Op::Cut`
- THEN the SAME single seam SHALL be decimated once (`decimateSeam` + `seamNodeTarget`) and
  shared by the two caps the op selects (two OUTER caps for FUSE; OUTER + reversed INNER for
  CUT), so the caps weld watertight along the ONE seam
- AND no additional trace, re-trace, or seam SHALL be required beyond the single closed
  circle S3 already produces

#### Scenario: a tangent / coincident sphere TraceSet is the S4 boundary, not consumed
- GIVEN a sphere∩sphere pair whose S3 `TraceSet` reports `nearTangentGaps > 0` (tangent
  spheres) or whose single WLine is `NearTangent` / spans a full sphere (coincident)
- WHEN any of the S5-c curved booleans (COMMON, FUSE, CUT) inspects the `TraceSet`
- THEN it SHALL decline to consume the trace (the honest S4 seam) and the boolean SHALL
  fall back to OCCT, reported — never welding on a truncated or fabricated seam
