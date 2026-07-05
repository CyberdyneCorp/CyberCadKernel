# native-ssi

Declare the S4-d **branched** `TraceSet` — two localized branch points connecting four
`BranchArc` arms, fully resolved (`nearTangentGaps == 0`), obtained by tracing with
`MarchOptions.enableBranchPoints = true` — as the consumed input contract for the S5-d
Steinmetz-family branched curved boolean (`openspec/SSI-ROADMAP.md` S5), alongside the
single-seam contracts the archived S5-a/b/c changes already declared (the two-rim through-drill
seam and the single closed sphere-lens seam). The tracer is UNCHANGED — this is the
already-shipped S4-d output; a branched trace that is NOT this exact resolved Steinmetz
structure (`nearTangentGaps > 0`, `branchPoints != 2`, or a WLine set that is not four
`BranchArc` arms) remains the honest boundary the boolean respects by declining (→ OCCT).
Internal capability: **no `cc_*` ABI change**.

## ADDED Requirements

### Requirement: The S4-d branched TraceSet is the consumed input contract for the S5-d Steinmetz common

The S4-d `cybercad::native::ssi` branched `TraceSet` SHALL be the input contract consumed by
the native S5-d Steinmetz-family branched curved boolean
(`src/native/boolean/ssi_boolean.cpp buildSteinmetzCommon` and the fuse/cut builders): for an
equal-radius orthogonal crossing cylinder pair, the boolean SHALL obtain the branched
`TraceSet` by tracing with `MarchOptions.enableBranchPoints = true` and use its two
`BranchNode`s (the localized branch points, each on both cylinders within `onSurfTol`) and its
four `BranchArc` `WLine`s — each arm's per-node `(u1,v1,u2,v2)` on both cylinders (the arc track
used to split each wall into lune patches) and its shared 3D nodes (the seam vertices the lune
patches weld on, plus the two shared branch-point vertices) — to assemble the four-lune COMMON
(and the fuse/cut fragment selections). The S5-d boolean SHALL consume the branched `TraceSet`
ONLY when it is fully resolved — `nearTangentGaps == 0`, `branchPoints == 2` with
`branchNodes.size() == 2`, exactly FOUR `WLine`s all of `status == BranchArc`, every arm on
both cylinders within `onSurfTol`, and the two branch nodes connecting all four arms. A branched
`TraceSet` with `nearTangentGaps > 0`, `branchPoints != 2`, a WLine set that is not four
`BranchArc` arms, or arms that do not meet at the two branch nodes SHALL be treated as the
honest fallback boundary and SHALL NOT be consumed (the boolean declines → OCCT). The branched
re-trace SHALL be entered ONLY after the DEFAULT (unbranched) trace has declined AND the
Steinmetz pre-gate (both cylinders, equal radii, orthogonal crossing axes) matches, so the
single-seam S3 transversal trace the S5-a/b/c paths consume is UNCHANGED. The tracer SHALL NOT
change to serve this consumption — the contract is the already-shipped S4-d output; no `cc_*`
entry point, signature, or POD struct SHALL be added or changed, and the SSI module SHALL remain
OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like the S3/S4-d tracer).

#### Scenario: the branched Steinmetz TraceSet is consumed to weld the four lunes

- GIVEN an equal-radius orthogonal crossing cylinder pair whose S4-d branched `TraceSet`
  (`enableBranchPoints = true`) has `nearTangentGaps == 0`, `branchPoints == 2`, and exactly
  four `BranchArc` arms meeting at the two branch nodes
- WHEN the S5-d curved boolean consumes the branched `TraceSet`
- THEN it SHALL split each cylinder wall along its two arcs into the inside-the-other lune
  patches, with every arc node on both cylinders within `onSurfTol`, and the four lune patches
  SHALL weld along the four arcs and the two shared branch-point vertices
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: a non-Steinmetz or unresolved branched TraceSet is the fallback boundary, not consumed

- GIVEN a branched `TraceSet` that reports `nearTangentGaps > 0` (an arm the S4-d marcher could
  not resolve), or `branchPoints != 2`, or a WLine set that is not exactly four `BranchArc`
  arms, or arms that do not meet at the two branch nodes
- WHEN the S5-d curved boolean inspects the branched `TraceSet`
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
