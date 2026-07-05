## Why

`SSI-ROADMAP.md` S5 is **the payoff** — general curved booleans driven by the S3/S4
`TraceSet`. S5-a landed the through-drill cylinder∩cylinder COMMON; S5-b/S5-c (archived
`add-native-ssi-curved-boolean-wider`) added the through-drill FUSE/CUT and the
sphere∩sphere lens COMMON. Every S5 slice so far consumes a **single-seam-per-loop
transversal** trace — the assembler gate in `src/native/boolean/ssi_boolean.cpp`
explicitly DECLINES anything branched:

```cpp
if (trace.nearTangentGaps > 0) return {};   // a branch traced up to a tangent → S4
if (trace.branchPoints > 0) return {};       // S4-d self-crossing → OCCT
... if (w.status == BranchArc) return {};     // arm WLine → OCCT
```

That decline is exactly the **Steinmetz bicylinder** — two equal-radius cylinders whose
axes cross orthogonally. It is the canonical curved-boolean payoff because its COMMON (the
bicylinder) has an EXACT analytic volume `16 R³ / 3` — a **free ground-truth oracle** on
top of OCCT `BRepAlgoAPI`. S4-d (`add-native-ssi-s4d-branch-points`, archived) already does
the hard tracing work: with `MarchOptions.enableBranchPoints = true` the marcher LOCALIZES
both branch points, ENUMERATES the tangent-cone arms, ROUTES them, and returns a fully
resolved branched `TraceSet`. Diagnosed on the current host marcher (two equal R=1
orthogonal cylinders, axes Z and X through the origin, extent z,x ∈ [−3,3]):

| Field | Value | Meaning |
|---|---|---|
| `branchPoints` | **2** | the two saddles, localized at `(0, ±1, 0)` (`branchSine ≈ 1.7e-7`, on both surfaces ≤ 1e-12) |
| `lines` | **4**, all `status == BranchArc` | the four branch-to-branch elliptical arms (branchIds 0, 10000, 10001, 10002) |
| `nearTangentGaps` | **0** | nothing deferred — the structure is fully traced |
| per-arm `(u1,v1,u2,v2)` | present, `onSurfResidual ≤ 2e-9` | each arm tracks the seam on BOTH cylinders |
| each arm's ends | both at `(0, ±1, 0)` | every arm runs branch-to-branch |

So the GAP is again the **assembler**, not tracing: S4-d already delivers the branched
trace; `ssi_boolean_solid` throws it away. This change adds the **branched-trace S5
assembler** — the S5-d slice — that consumes that branched `TraceSet` and builds the native
Steinmetz-family COMMON (and FUSE/CUT where tractable), verified against both the exact
`16 R³ / 3` analytic and OCCT `BRepAlgoAPI`, declining honestly (NULL → OCCT) for anything
outside the recognised Steinmetz family.

## What Changes

- **(S5-d) Turn on the branched trace, ONLY for the recognised Steinmetz family.** In
  `ssi_boolean_solid`, when both operands are recognised curved solids, RE-TRACE with
  `MarchOptions.enableBranchPoints = true` when the default (unbranched) trace comes back
  as an unresolved branch signal (`nearTangentGaps > 0` with no usable seams) AND the pair
  passes the cheap Steinmetz pre-gate (both cylinders, near-equal radii, axes ~orthogonal
  and crossing). The default single-seam paths (S5-a/b/c: through-drill cyl∩cyl,
  sphere∩sphere lens) are UNCHANGED — the branched re-trace is entered only when the
  unbranched trace itself declined and the pre-gate matches, so no existing S5 pass changes
  its trace.
- **(S5-d) Recognise the Steinmetz-family branched trace.** A NEW gate accepts the trace
  ONLY when: both operands are `Cylinder`; radii equal within tolerance; axes orthogonal
  (`|â·b̂| ≤ tol`) and crossing (the axis lines meet within tolerance); `branchPoints == 2`;
  exactly **four** `BranchArc` arms; every arm on both cylinders ≤ `onSurfTol`; and the two
  `BranchNode`s connect all four arms (`armLineIds` covers the four branchIds at each B).
  Anything else — unequal radii, non-orthogonal or non-crossing axes, a different branch
  count, an unresolved gap (`nearTangentGaps > 0`), a cylinder∩non-cylinder branched pair —
  returns NULL → OCCT, reported.
- **(S5-d) Split each cylinder wall along its arcs into the inside-the-other lune
  patch(es).** On each cylinder the two arms lying on THAT cylinder bound the region INSIDE
  the other cylinder. Split the wall along its two arcs and keep the fragment(s) whose
  interior sample passes the op's survival rule (`classifyPoint` against the other solid):
  **COMMON** keeps inside∩inside → the four curved lune patches; **CUT `A − B`** keeps
  A-outside-B plus B-inside-A reversed; **FUSE `A ∪ B`** keeps each wall's outside-the-other
  fragment plus the cylinders' end caps. The SAME planar set algebra as the planar
  `booleanPolygons` and the S5-a/b `classifyPoint` selection.
- **(S5-d) Weld the surviving patches into ONE watertight shell, sharing the arc seams AND
  the two branch-point vertices.** Reuse the S5-a planar-facet weld discipline: every
  seam-adjacent patch emits PLANAR-TRIANGLE facets through the EXACT traced arc nodes drawn
  from ONE shared `VertexPool`, and the two branch-point vertices `(0, ±1, 0)` are pooled
  ONCE so all four arcs meet there with no crack. Because both sides of every shared arc
  draw the same pooled nodes, and the branch points are a single shared vertex, the shell
  welds watertight (the S5-a lesson — the tessellator is NOT touched).
- **Engine self-verify — no new generic oracle needed; the Steinmetz oracle already
  exists.** The ENGINE's existing `ssiCurvedBooleanVerified` Steinmetz special oracle
  (`native_engine.cpp`, op == common, equal-radius orthogonal cylinders) already knows the
  `16 R³ / 3` closed form; it now has a NATIVE candidate to verify instead of always
  falling to OCCT. FUSE/CUT are caught by the EXISTING generic set-algebra guard
  (`Vr ≈ va + vb − vc` / `va − vc`, `vc` = the native branched COMMON). A candidate that is
  open, non-manifold, or off-volume is DISCARDED → OCCT. No new engine oracle is added; the
  single-seam S5-a/b/c guards are untouched.
- **Honest scope.** ONLY the recognised Steinmetz family ships: equal-R orthogonal crossing
  cylinders, `branchPoints == 2`, four `BranchArc` arms, fully resolved
  (`nearTangentGaps == 0`). General branched booleans — unequal-R or non-orthogonal
  branched cylinder pairs, cylinder∩sphere/cone self-crossings, three-plus branch points,
  any pair whose branched trace is not this exact structure — still DECLINE → OCCT. FUSE/CUT
  ship ONLY if they assemble a watertight, correct-volume shell; whichever does not verify
  falls back to OCCT and is REPORTED with the measured gap. Nothing is faked, hand-tuned, or
  shipped unverified; a tolerance is never weakened to force a pass.

**No `cc_*` ABI change.** Invoked behind the existing `cc_boolean` op codes through the same
`boolean_solid` entry. `src/native/**` stays OCCT-free; the branched assembler path is
compiled under `CYBERCAD_HAS_NUMSCI` (it consumes the S4-d branched tracer), exactly like
S5-a/b/c. Additive only — the single-seam S5-a/b/c paths and their tests are unchanged, and
the S4-d tracer is consumed unchanged (this change adds NO tracer code).

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-booleans capability (the S5 SSI-curve-
driven curved boolean) with the branched-trace Steinmetz-family assembler, and EXTENDS
native-ssi by declaring the S4-d branched TraceSet (branchNodes + four BranchArc arms) as a
consumed S5-d input contract. -->

### Modified Capabilities
- `native-booleans`: extend the S5 SSI-curve-driven curved boolean with (S5-d) a NEW
  **branched-trace assembler** that consumes the S4-d branched `TraceSet` (two branch nodes
  + four `BranchArc` arms) for the **Steinmetz family** (equal-R orthogonal crossing
  cylinders), SPLITS each cylinder wall along its arcs into the inside-the-other lune
  patches, SELECTS the surviving fragments per op (COMMON → four lunes; FUSE/CUT analogous),
  and WELDS them into ONE watertight shell SHARING the arc seams and the two branch-point
  vertices. COMMON is verified against the EXACT `16 R³ / 3` analytic (the existing engine
  Steinmetz oracle) and OCCT `BRepAlgoAPI_Common`; FUSE/CUT against the existing generic
  set-algebra guard and OCCT `BRepAlgoAPI_{Fuse,Cut}`. Out-of-family branched pairs DECLINE
  → OCCT, reported not faked. No `cc_*` change; the single-seam S5-a/b/c paths are unchanged.
- `native-ssi`: declare the S4-d branched `TraceSet` — `branchPoints == 2`,
  `branchNodes.size() == 2` connecting four `BranchArc` arms, `nearTangentGaps == 0`,
  obtained with `MarchOptions.enableBranchPoints = true` — as the consumed input contract for
  the S5-d Steinmetz-family branched boolean. Each arm's per-node `(u1,v1,u2,v2)` tracks the
  arc on BOTH cylinders and its shared 3D nodes (plus the two shared branch-point vertices)
  are what the lune patches weld on. The tracer is UNCHANGED; a branched trace that is NOT
  this exact resolved Steinmetz structure (`nearTangentGaps > 0`, ≠ 2 branch points, ≠ 4
  arms) is the honest boundary the boolean declines on → OCCT.

## Impact

- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry point,
  signature, or POD struct change. Additive only.
- **Build**: edits `src/native/boolean/ssi_boolean.cpp` only (a Steinmetz pre-gate + the
  branched re-trace opt-in in `ssi_boolean_solid`; a branched-trace recognition gate; a
  `buildSteinmetzCommon` and, if tractable, `buildSteinmetzFuse` / `buildSteinmetzCut`
  assembler reusing `wallSurface`, the planar-facet seam helpers, `VertexPool`,
  `classifyPoint`). No new files strictly required. Compiled under `CYBERCAD_HAS_NUMSCI`.
  `src/native/**` stays OCCT-free. **NO change to `src/native/tessellate`** (the S5-a
  lesson: a tessellator change fixes one case and breaks others — every watertight fix is
  assembler-side, using the S5-a planar-facet weld). **NO change to `src/native/ssi`** (the
  S4-d tracer is consumed unchanged). **NO change to the CyberCad app.**
- **Verification**: two gates. **Host (no OCCT)** — extend
  `tests/native/test_native_ssi_curved_boolean.cpp`: the equal-R orthogonal Steinmetz pair
  re-traces branched, `ssi_boolean_solid(A, B, Common)` is non-NULL, watertight, and its
  enclosed volume equals the EXACT `16 R³ / 3` within the curved-face deflection band; every
  arc node on BOTH cylinders ≤ tol and the two branch-point vertices are a single shared
  node; FUSE = `2·vol(cyl) − 16R³/3` and CUT = `vol(cyl) − 16R³/3` within the band (if
  shipped); an UNEQUAL-radius or NON-orthogonal branched pair returns NULL (deferred). No
  OCCT linked; no tolerance weakened. **Sim native-vs-OCCT** — the existing
  `scripts/run-sim-native-ssi-curved-boolean.sh` + `tests/sim/native_ssi_curved_boolean_parity.mm`
  ALREADY carry the `cyl=cyl(steinmetz)` COMMON/FUSE/CUT cases (today native NULL → OCCT,
  volO common = 5.3333, fuse = 32.366, cut = 13.516); after this change they must become
  NATIVE passes (watertight, volume + area within tol vs `BRepAlgoAPI_{Common,Fuse,Cut}`).
  Report per-op the native-vs-OCCT deltas and the count still deferred to OCCT.
- **Roadmap**: advances `SSI-ROADMAP.md` S5 from the single-seam slices (S5-a/b/c) to the
  FIRST BRANCHED-TRACE slice (S5-d) — the Steinmetz bicylinder, the canonical payoff, now
  native and verified against its exact `16 R³ / 3` oracle and OCCT. **Explicitly a first
  branched slice:** general / freeform branch points and non-Steinmetz branched booleans
  remain the tail and still decline → OCCT.
- **Risk (honest)**: (a) the lune split can mis-classify a patch whose interior sample is
  near the arc — mitigated by sampling at the patch centroid (well inside), the
  `classifyPoint` ON-band abort, and the engine correct-volume guard DISCARD → OCCT; (b) the
  four arcs must weld exactly at the two branch points — mitigated by pooling each branch
  vertex ONCE and taking every arc's shared 3D nodes from the SAME `VertexPool`, with the
  engine watertight check as the backstop; (c) an arm's per-cylinder `(u,v)` track may wrap
  the seam periodically — mitigated by the S5-a `nearU`/`unwrapRim` folding the assembler
  already uses; (d) FUSE/CUT may leave a hairline gap at a re-trimmed cap — caught by the
  watertight guard → OCCT, and FUSE/CUT are shipped ONLY if they verify (COMMON is the
  guaranteed slice). Whatever does not verify falls back to OCCT and is reported with the
  measured gap; no case is faked, stubbed, or hand-tuned to pass.
