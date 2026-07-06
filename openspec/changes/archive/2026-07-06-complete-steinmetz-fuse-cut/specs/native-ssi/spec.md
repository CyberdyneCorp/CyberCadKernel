# native-ssi

Extend the S5-d input contract from the Steinmetz bicylinder COMMON to ALL THREE Steinmetz
boolean ops. The SAME resolved branched `TraceSet` already consumed by `buildSteinmetzCommon`
(`branchPoints == 2`, four `BranchArc` arms, `nearTangentGaps == 0`) is the input to the new
FUSE (`buildSteinmetzFuse`) and CUT (`buildSteinmetzCut`) builders: the four arcs split each
cylinder wall into inside/outside regions, and COMMON / FUSE / CUT differ only in which regions
survive, their orientation, and (FUSE/CUT) the original disc end caps — not in the trace. The
tracer does not change; the branch-point re-trace and the non-Steinmetz fallback boundary are
unchanged.

## MODIFIED Requirements

### Requirement: The S4-d branched TraceSet is the consumed input contract for the S5-d Steinmetz common

The S4-d `cybercad::native::ssi` branched `TraceSet` SHALL be the input contract consumed by ALL
THREE native S5-d Steinmetz-family branched curved booleans
(`src/native/boolean/ssi_boolean.cpp`): `buildSteinmetzCommon` (COMMON), `buildSteinmetzFuse`
(FUSE), and `buildSteinmetzCut` (CUT). For an equal-radius orthogonal crossing cylinder pair,
each boolean SHALL obtain the branched `TraceSet` by tracing with
`MarchOptions.enableBranchPoints = true` and use its two `BranchNode`s (the localized branch
points, each on both cylinders within `onSurfTol`) and its four `BranchArc` `WLine`s — each arm's
per-node `(u1,v1,u2,v2)` on both cylinders (the arc track used to split each wall into lune
patches) and its shared 3D nodes (the seam vertices the patches weld on, plus the two shared
branch-point vertices). The SAME four arcs SHALL split each cylinder wall into an INSIDE region
(the lune, inside the other cylinder) and an OUTSIDE region; the ops differ ONLY in fragment
selection, orientation, and cap handling: COMMON assembles the four INSIDE lunes (the
bicylinder), FUSE the four OUTSIDE lune walls plus both cylinders' two original disc end caps
(the outer envelope), and CUT the OUTSIDE walls plus disc caps of the minuend plus the INSIDE
lunes of the subtrahend REVERSED (the carved channel). Each boolean SHALL consume the branched
`TraceSet` ONLY when it is fully resolved — `nearTangentGaps == 0`, `branchPoints == 2` with
`branchNodes.size() == 2`, exactly FOUR `WLine`s all of `status == BranchArc`, every arm on both
cylinders within `onSurfTol`, and the two branch nodes connecting all four arms. A branched
`TraceSet` with `nearTangentGaps > 0`, `branchPoints != 2`, a WLine set that is not four
`BranchArc` arms, or arms that do not meet at the two branch nodes SHALL be treated as the honest
fallback boundary and SHALL NOT be consumed by any of the three ops (the boolean declines →
OCCT). The branched re-trace SHALL be entered ONLY after the DEFAULT (unbranched) trace has
declined AND the Steinmetz pre-gate (both cylinders, equal radii, orthogonal crossing axes)
matches, so the single-seam S3 transversal trace the S5-a/b/c paths consume is UNCHANGED. The
tracer SHALL NOT change to serve this consumption — the contract is the already-shipped S4-d
output; no `cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI
module SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like the S3/S4-d tracer).

#### Scenario: the branched Steinmetz TraceSet is consumed to weld the four lunes

- GIVEN an equal-radius orthogonal crossing cylinder pair whose S4-d branched `TraceSet`
  (`enableBranchPoints = true`) has `nearTangentGaps == 0`, `branchPoints == 2`, and exactly
  four `BranchArc` arms meeting at the two branch nodes
- WHEN the S5-d curved boolean consumes the branched `TraceSet`
- THEN it SHALL split each cylinder wall along its two arcs into the inside-the-other lune
  patches, with every arc node on both cylinders within `onSurfTol`, and the four lune patches
  SHALL weld along the four arcs and the two shared branch-point vertices
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: the four arcs feed fuse and cut as they already feed common

- GIVEN an equal-radius orthogonal crossing cylinder pair whose S4-d branched `TraceSet` is
  fully resolved (`branchPoints == 2`, four `BranchArc` arms, `nearTangentGaps == 0`) with
  `(u1,v1,u2,v2)` per node
- WHEN the native boolean path runs `Op::Fuse` or `Op::Cut`
- THEN the SAME four arcs SHALL be oriented and pole-axis-resampled ONCE (the shared prologue) and
  shared by the fragments the op selects (both cylinders' OUTSIDE lunes plus their disc end caps
  for FUSE; the minuend's OUTSIDE lunes plus its caps plus the subtrahend's INSIDE lunes REVERSED
  for CUT), so the fragments weld watertight along the four arcs and the two shared branch-point
  poles
- AND no additional trace, re-trace, or arc SHALL be required beyond the four `BranchArc` arms
  the S4-d re-trace already produces

#### Scenario: a non-Steinmetz or unresolved branched TraceSet is the fallback boundary, not consumed

- GIVEN a branched `TraceSet` that reports `nearTangentGaps > 0` (an arm the S4-d marcher could
  not resolve), or `branchPoints != 2`, or a WLine set that is not exactly four `BranchArc`
  arms, or arms that do not meet at the two branch nodes
- WHEN any of the S5-d curved booleans (COMMON, FUSE, CUT) inspects the branched `TraceSet`
- THEN it SHALL decline to consume the trace (the honest fallback boundary) and the boolean
  SHALL fall back to OCCT, reported — never welding a shell on a truncated, mismatched, or
  fabricated branched structure

#### Scenario: the default single-seam trace is unchanged for non-Steinmetz pairs

- GIVEN a transversal surface pair whose DEFAULT (unbranched) trace is a clean single-seam
  transversal (a through-drill cylinder pair or a sphere-lens pair) OR any pair the Steinmetz
  pre-gate does not match
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume the DEFAULT `TraceSet` (branch points OFF) exactly as the single-seam
  S5-a/b/c paths do, and SHALL NOT enter the branched re-trace — the branch machinery engages
  ONLY on the declined edge when the Steinmetz pre-gate matches
