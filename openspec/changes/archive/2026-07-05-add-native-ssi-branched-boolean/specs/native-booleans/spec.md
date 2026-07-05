# native-booleans

Extend the S5 SSI-curve-driven curved boolean (`src/native/boolean/ssi_boolean.{h,cpp}`,
`openspec/SSI-ROADMAP.md` S5) with the **S5-d branched-trace assembler** — the branched
analogue of the single-seam S5-a/b/c slices — driven by the already-shipped S4-d branched
`ssi::TraceSet` (two branch nodes + four `BranchArc` arms, obtained with
`MarchOptions.enableBranchPoints = true`) and guarded by the engine's EXISTING oracles → OCCT
fallback:

- **S5-d** — native **Common** (`A ∩ B`), and **Fuse** (`A ∪ B`) / **Cut** (`A − B`) where
  they verify, for the **Steinmetz family** (two equal-radius cylinders whose axes cross
  orthogonally), by splitting each cylinder wall along its two arcs into the inside-the-other
  lune patches, selecting the surviving fragments per the op set algebra, and welding them into
  ONE watertight shell that SHARES the four arc seams and the two branch-point vertices.

The Steinmetz COMMON is verified against the EXACT `16 R³ / 3` analytic (the engine's existing
Steinmetz oracle) and OCCT `BRepAlgoAPI_Common`; FUSE/CUT against the existing generic
set-algebra guard and OCCT `BRepAlgoAPI_{Fuse,Cut}`. Everything outside the recognised
Steinmetz family DECLINES (NULL → OCCT); a candidate the self-verify rejects is DISCARDED →
OCCT, reported. Internal: **no `cc_*` ABI change** — invoked behind the existing `cc_boolean`
op codes. `src/native/**` stays OCCT-free; the branched assembler is compiled under
`CYBERCAD_HAS_NUMSCI`. No change to `src/native/tessellate`, to `src/native/ssi` (the S4-d
tracer is consumed unchanged), or to the S5-a/b/c single-seam paths.

## ADDED Requirements

### Requirement: SSI-driven native Common for the Steinmetz-family branched pair

The native boolean library SHALL compute `cc_boolean(a, b, 2)` (common, `A ∩ B`) NATIVELY for
the **Steinmetz family** — two `Cylinder` curved solids of equal radius (`|rA − rB|` within
tolerance) whose axes are orthogonal (`|â · b̂|` within tolerance) and cross (the axis lines
meet within tolerance) — via a NEW branched-trace assembler (`buildSteinmetzCommon`). The
assembler SHALL consume the S4-d branched `ssi::TraceSet` obtained with
`MarchOptions.enableBranchPoints = true`, and SHALL recognise the Steinmetz structure ONLY
when the trace is fully resolved: `nearTangentGaps == 0`, `branchPoints == 2` with
`branchNodes.size() == 2`, EXACTLY FOUR WLines all of `status == BranchArc`, every arm on both
cylinders within `onSurfTol`, and the two branch nodes connecting all four arms (each arm's two
endpoints coincide with the two branch-node points). The assembler SHALL:

- **Split.** On each cylinder the two arms lying on that cylinder bound the region of its wall
  INSIDE the other cylinder; SPLIT the wall along its two arcs into the candidate lune patches,
  each emitted as a strip of PLANAR triangles between its two arcs (walked branch-to-branch in
  lockstep), every interior sample placed ON the analytic cylinder and its `(u,v)` folded
  contiguous around the patch centroid so no ±2π wrap corrupts it.
- **Select (COMMON rule).** KEEP each lune patch ONLY IF its centroid sample is INSIDE the
  OTHER cylinder (`classifyPoint(other, centroid) == inside`) — the four inside∩inside patches
  ARE the bicylinder boundary. A centroid robustly ON the other cylinder SHALL abort the native
  path → NULL → OCCT, never a guessed side.
- **Weld.** Emit every seam-adjacent patch as PLANAR-TRIANGLE facets through the EXACT traced
  arc nodes drawn from ONE shared `VertexPool`, with the two branch-point vertices pooled ONCE
  so all four arcs meet there with no crack; both sides of every shared arc draw the SAME
  pooled vertices, so the four lune patches weld watertight along the four arcs and the two
  branch points (the S5-a planar-facet weld discipline). Facet normals SHALL be oriented
  outward (the cylinder's outward radial).

The result SHALL be a native `topology::Shape` of type `Solid`, watertight (every edge shared
by exactly two faces), with every arc node on BOTH cylinders within tolerance and the two
branch-point vertices a single shared node, whose enclosed volume equals the EXACT Steinmetz
bicylinder value `16 R³ / 3` within the curved-face tessellation deflection tolerance. The
assembler SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type, and
SHALL be compiled under `CYBERCAD_HAS_NUMSCI`. No `cc_*` entry point, signature, or POD struct
SHALL be added or changed, and the single-seam S5-a/b/c paths SHALL be unchanged.

#### Scenario: The Steinmetz common matches the exact analytic bicylinder volume (host)

- GIVEN two equal-radius cylinders (radius `R`) whose axes cross orthogonally at the origin,
  built as native curved solids on the host with no OCCT, and their S4-d branched `TraceSet`
  (obtained with `enableBranchPoints = true`: `branchPoints == 2`, four `BranchArc` arms,
  `nearTangentGaps == 0`)
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces) with
  every arc node on both cylinders within tolerance and the two branch-point vertices a single
  shared node AND its enclosed volume SHALL equal the exact bicylinder `16 R³ / 3` within the
  curved-face deflection tolerance

#### Scenario: The four lune patches weld at the shared arcs and the two branch points (host)

- GIVEN the Steinmetz pair and its S4-d branched `TraceSet` (four `BranchArc` arms meeting at
  two branch nodes)
- WHEN `buildSteinmetzCommon` splits each cylinder wall along its arcs, selects the four
  inside-the-other lune patches, and welds them
- THEN the four lune patches SHALL be assembled from the SAME traced arc nodes drawn from ONE
  shared `VertexPool` with the two branch-point vertices pooled ONCE, so all four arcs weld at
  the two branch points and both sides of every arc weld along the arc, with no new tracing and
  no hand-matched per-primitive result builder

### Requirement: SSI-driven native Fuse and Cut for the Steinmetz-family branched pair

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse (`A ∪ B`),
`op = 1` cut (`A − B`) — NATIVELY for the Steinmetz family (the SAME recognised branched trace
the COMMON path consumes) WHEN it can assemble a watertight, correct-volume shell, reusing the
SAME four arcs, the SAME shared `VertexPool` weld (arcs + the two branch-point vertices), and
the SAME planar-triangle facet discipline as the COMMON, and selecting the surviving fragments
per the op's face-survival rule — the SAME set algebra as the planar path:

- **Cut `A − B`**: A's wall OUTSIDE B + A's two end caps + B's inside-A lune patches REVERSED
  (the tunnel wall); A's inside-B lune patches SHALL be dropped. The shared arcs weld the
  reversed B patches to A's outside wall.
- **Fuse `A ∪ B`**: each cylinder's OUTSIDE-the-other wall + both cylinders' end caps; both
  cylinders' inside-the-other lune patches SHALL be dropped (now interior to the union). The
  shared arcs weld the two outer walls.

Fragment survival SHALL be decided by the S5-a curved point-in-solid test (`classifyPoint`) at
an interior sample; a sample robustly ON the other solid, a non-Steinmetz branched trace, or a
weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The result SHALL be a native
`topology::Shape` of type `Solid`, watertight (every edge shared by exactly two faces), whose
enclosed volume equals the exact set-algebra value for the op within a relative tolerance sized
to the curved-face tessellation deflection: `Vr ≈ vol(A) + vol(B) − vol(A ∩ B)` (fuse) or
`Vr ≈ vol(A) − vol(A ∩ B)` (cut), where `vol(A ∩ B)` is the native Steinmetz COMMON. A builder
that cannot robustly assemble a watertight, correct-volume shell SHALL return NULL (→ OCCT),
reported — the COMMON is the guaranteed slice. The builder SHALL remain OCCT-free and be
compiled under `CYBERCAD_HAS_NUMSCI`; no `cc_*` entry point SHALL be added or changed, and the
single-seam S5-a/b/c paths SHALL be unchanged.

#### Scenario: The Steinmetz cut removes the bicylinder with the correct volume (host)

- GIVEN the equal-R orthogonal Steinmetz pair built as native curved solids on the host with no
  OCCT, and its S4-d branched `TraceSet`
- WHEN `cc_boolean(A, B, 1)` (cut) is computed and tessellated and the builder assembles a
  watertight shell
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces) AND its
  enclosed volume SHALL equal `vol(A) − 16 R³ / 3` within the curved-face deflection tolerance;
  a builder that cannot assemble a watertight, correct-volume shell SHALL return NULL (→ OCCT)

#### Scenario: The Steinmetz fuse welds both cylinders with the correct volume (host)

- GIVEN the same equal-R orthogonal Steinmetz pair built as native curved solids on the host
- WHEN `cc_boolean(A, B, 0)` (fuse) is computed and tessellated and the builder assembles a
  watertight shell
- THEN the result SHALL be a watertight closed 2-manifold `Solid` AND its enclosed volume SHALL
  equal `vol(A) + vol(B) − 16 R³ / 3` within the curved-face deflection tolerance AND SHALL
  satisfy `fuse ≥ max(vol(A), vol(B))`; a builder that cannot assemble a watertight,
  correct-volume shell SHALL return NULL (→ OCCT)

### Requirement: The Steinmetz branched curved boolean is guarded by the existing engine self-verify

The engine SHALL accept a native S5-d Steinmetz COMMON / FUSE / CUT result as native ONLY when
it PASSES the EXISTING mandatory self-verify: a closed watertight 2-manifold with the correct
volume. The Steinmetz COMMON SHALL be verified by the engine's EXISTING
`ssiCurvedBooleanVerified` Steinmetz oracle (op == common, equal-radius orthogonal cylinders —
the `16 R³ / 3` closed form), which previously always found a NULL native candidate and fell to
OCCT and now verifies the branched native candidate's watertight volume against `16 R³ / 3`.
The Steinmetz FUSE / CUT SHALL be verified by the EXISTING generic set-algebra guard
(`Vr ≈ va + vb − vc` / `va − vc`, `vc` = the native Steinmetz COMMON volume). NO new engine
oracle SHALL be added, and the single-seam S5-a/b/c guards SHALL remain untouched and SHALL NOT
fire for the branched Steinmetz case. If the self-verify fails, the engine SHALL DISCARD the
native result and fall through to OCCT `BRepAlgoAPI` (OCCT operand) or report an honest error
(both operands native voids). The engine SHALL NEVER emit an unverified, leaky, or wrong
Steinmetz curved boolean.

#### Scenario: A bad Steinmetz branched candidate is discarded (host)

- GIVEN a native S5-d Steinmetz COMMON / FUSE / CUT candidate that is open / non-manifold OR
  whose enclosed volume is outside the deflection-sized band for its op, built on the host
- WHEN the existing engine self-verify (the Steinmetz `16 R³/3` oracle for common; the generic
  set-algebra guard for fuse/cut) is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL NOT emit a leaky or wrong
  curved solid (a native-native case reports an honest error; an OCCT-operand case falls through
  to OCCT)

#### Scenario: A verified Steinmetz common passes the existing oracle natively (host)

- GIVEN a native Steinmetz COMMON whose watertight volume matches the exact `16 R³ / 3` within
  the deflection band
- WHEN the existing `ssiCurvedBooleanVerified` Steinmetz oracle is applied
- THEN the oracle SHALL accept the candidate AND it SHALL be served natively with no OCCT
  fallback call AND no new engine oracle SHALL have been added

### Requirement: Out-of-family branched curved pairs fall through to OCCT

The S5-d branched assembler SHALL DECLINE (return a NULL `Shape`) for any branched curved pair
outside the recognised Steinmetz family: (1) **unequal-radius, non-orthogonal, or non-crossing
cylinder pairs** (the Steinmetz pre-gate rejects); (2) **cylinder∩sphere, cylinder∩cone,
cone∩cone, or freeform self-crossings**; (3) branched traces with `nearTangentGaps > 0` (an arm
the S4-d marcher could not resolve), `branchPoints != 2`, or a WLine count / status that is not
exactly four `BranchArc` arms. The branched re-trace SHALL be entered ONLY when the DEFAULT
(unbranched) trace declined AND the Steinmetz pre-gate matches, so no single-seam S5-a/b/c pass
re-traces or changes its result. When either operand is an OCCT body, each declined case SHALL
produce EXACTLY the OCCT `BRepAlgoAPI` fallback result; when both operands are native voids OCCT
cannot read, the engine SHALL report an honest error. The change SHALL NOT fake, stub-out,
hand-tune, or partially implement any deferred case; a branched trace that is not the exact
resolved Steinmetz structure SHALL remain the honest fallback boundary, not an error.

#### Scenario: An unequal-radius or non-orthogonal branched cylinder pair declines to OCCT (host)

- GIVEN two cylinders whose axes cross but with UNEQUAL radii, OR two equal-radius cylinders
  whose axes are NOT orthogonal, with the native engine active
- WHEN `cc_boolean(A, B, op)` is invoked for any op
- THEN the S5-d branched assembler SHALL return a NULL `Shape` (the Steinmetz pre-gate / the
  recognition gate rejects) AND (with an OCCT operand) the result SHALL be identical to invoking
  the same call with the OCCT engine active, proving fall-through with no native interception

#### Scenario: A single-seam S5-a/b/c pair never enters the branched re-trace (host)

- GIVEN a through-drill cylinder∩cylinder pair (unequal radii, two rim seams) and a transversal
  sphere∩sphere pair (one closed seam), each with the native engine active
- WHEN each is dispatched
- THEN each SHALL be handled by its single-seam S5-a/b/c builder on the DEFAULT trace with no
  branched re-trace, and the S5-d branched path SHALL NOT fire (its pre-gate requires equal-R
  orthogonal cylinders), with no cross-contamination and no hand-matched per-primitive builder
