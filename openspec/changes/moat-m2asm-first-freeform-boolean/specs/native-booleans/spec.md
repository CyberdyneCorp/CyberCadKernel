# native-booleans

## ADDED Requirements

### Requirement: B4 analytic-face split and cross-section cap-synthesis weld verb for a planar half-space cut

The native boolean library SHALL provide an OCCT-free, header-only **B4 verb** (in
`src/native/boolean/half_space_cut.h`) that, given a `FreeformOperand` admitted by B1,
a cutting `Plane` `P`, a keep side, and the B2 seam chord for the operand's freeform
wall, SPLITS each analytic (planar) boundary face `P` crosses, SYNTHESISES one
cross-section cap on `P`, and WELDS the survivors into a watertight `Solid` — or
DECLINES (returns a NULL `Shape`) with a measured blocker. It SHALL be a strictly
ADDITIVE sibling: it SHALL NOT modify B1/B2/B3, the M0 mesher, the M1 tracer, or the
analytic `recogniseCurvedSolid`/`classifyPoint`.

- **Analytic-face split.** For each `AnalyticHalfSpace` face, the verb SHALL keep the
  face WHOLE when its outer loop lies entirely on the keep side, DROP it when entirely
  on the discard side, and otherwise SPLIT it along its exact `Face ∩ P` line: it SHALL
  require EXACTLY TWO boundary-edge crossings, record each crossing as (edge id, curve
  parameter), and rebuild the keep-side sub-face over the parent `Plane` surface with
  the parent's per-edge pcurves carried verbatim. A crossing that is tangent, at a
  vertex, or that does not yield exactly two crossings SHALL DECLINE.
- **Cross-section cap synthesis.** The verb SHALL assemble ONE ordered closed loop on
  `P` from the B2 seam chord spliced to the straight crossing segments of the split
  analytic faces, SHALL verify the loop is SIMPLE (no non-adjacent-edge intersection)
  and CLOSED, and SHALL build ONE `Plane`-surface cap face bounded by that loop
  oriented OUTWARD (cap normal facing the discard side). The seam edge SHALL be built
  ONCE and SHARED bit-exactly with the kept freeform sub-face; each straight cap edge
  SHALL be SHARED with its split analytic face's crossing edge. A non-simple or open
  section loop SHALL DECLINE.
- **Weld.** The verb SHALL weld the kept freeform sub-face, the kept analytic sub-faces
  and whole faces, and the section cap into one shell → `Solid`, welding coincident
  corners to shared vertices (the `assemble.h` `VertexPool` discipline). A shell that
  cannot close SHALL DECLINE.

The verb SHALL remain OCCT-free (zero OCCT includes), SHALL introduce no `cc_*` ABI
surface, and SHALL NOT weaken any tolerance to force a result. Its per-function
cognitive complexity SHALL stay within the backend band (helpers for per-face split,
section-loop assembly, and weld).

#### Scenario: The plane splits the crossed analytic faces and caps the section (host, no OCCT)

- GIVEN the bowl-lidded convex-quad prism operand admitted by `recogniseFreeformSolid`, the cutting plane `P: x = 0` keeping `x ≤ 0`, and the B2 seam chord for the bowl top, built on the host with NO OCCT
- WHEN the B4 verb runs
- THEN it SHALL split exactly the two vertical walls and the bottom face `P` crosses (each into a keep and a discard sub-face along `Face ∩ P`, with the two crossings recorded as (edge, param)), keep the wall entirely on the keep side and drop the wall entirely on the discard side whole, and synthesise ONE outward-oriented `Plane` section cap whose boundary is the B2 seam chord spliced to the three straight crossing segments — a simple, closed loop

#### Scenario: A degenerate cut configuration DECLINES to a NULL Shape (host)

- GIVEN a configuration outside the verb's envelope — a plane that misses the operand, a crossing that is tangent or lands on a vertex, an analytic face with other than two crossings, a section loop that is not simple or not closed, or a weld that cannot close — built on the host with NO OCCT
- WHEN the B4 verb runs
- THEN it SHALL return a NULL `Shape` identifying the measured blocker, SHALL NOT emit a partial, overlapping, or leaky solid, and SHALL NOT weaken a tolerance to force a result

#### Scenario: The section cap and freeform top share the seam edge bit-exactly (host)

- GIVEN a B4 result on the bowl-lidded prism CUT
- WHEN its shell is inspected
- THEN the seam edge SHALL be a SINGLE shared edge between the kept freeform sub-face and the section cap (built once, opposite orientation), each straight cap edge SHALL be shared with its split analytic face's crossing edge, and the shell SHALL be watertight (every edge shared by exactly two faces)

### Requirement: First native freeform↔analytic-half-space CUT composed from B1→M1→B2→B4→B3→M0 with mandatory self-verify

The native boolean library SHALL compute the FIRST end-to-end freeform boolean — a
single-freeform-wall operand (the bowl-lidded convex-quad prism) CUT by ONE analytic
PLANAR half-space — composed ENTIRELY from the landed M2 verbs consumed UNCHANGED,
guarded by a mandatory OCCT-free self-verify. The assembler SHALL: (1) admit the
operand with `recogniseFreeformSolid`[B1]; (2) trace the freeform wall ∩ plane seam as
a single M1 `WLine`[M1] with `points.size() ≥ 2` and status `Closed`/`BoundaryExit`;
(3) split the freeform wall along the seam with `splitFace`[B2]; (4) split the crossed
analytic faces and synthesise the cross-section cap with the B4 verb; (5) CONFIRM the
keep-side selection with `classifyPointInMesh`[B3] against the PRE-cut operand's M0
boundary mesh — every kept sub-face's interior centroid SHALL classify `In` AND lie on
the keep side of the plane; (6) weld the survivors watertight[M0]. Before returning,
the assembler SHALL run the MANDATORY self-verify: the welded result SHALL be
WATERTIGHT (every edge shared by exactly two faces) AND its enclosed volume SHALL match
the closed-form CUT value `∫∫_{Q ∩ keep} (h0 + a·(x² + y²)) dA` within a scale-relative
tolerance sized to the M0 deflection, and a result that FAILS SHALL be DISCARDED
(NULL `Shape` → OCCT).

The assembler SHALL DECLINE (NULL `Shape` → OCCT) whenever any verb declines — B1
`nullopt`, an M1 seam that is not one clean chord, a B2 `SplitDecline`, a B4 decline,
or a B3 `On`/`Unknown` verdict or a wrong-side survivor — and SHALL NEVER emit a
partial, overlapping, leaky, or wrong-volume solid, nor GUESS a fragment's membership.
If the end-to-end CUT is not robustly reachable this wave, the change SHALL LAND the B4
verb proven in isolation and HONEST-DECLINE the assembly, recording which verb declined
and the measured remaining gap. The consumed B1/B2/B3/M0/M1 subsystems and the analytic
`recogniseCurvedSolid`/`classifyPoint` paths SHALL remain byte-identical; no tolerance
SHALL be weakened to force an assembly; OCCT SHALL remain the oracle and the fallback.

#### Scenario: The bowl-lidded quad prism CUT by a plane assembles watertight with the closed-form volume (host, analytic — no OCCT)

- GIVEN the bowl-lidded convex-quad prism operand and the cutting plane `x = 0` keeping `x ≤ 0`, whose clipped-region volume `∫∫_{Q ∩ {x≤0}} (h0 + a·(x² + y²)) dA` is KNOWN in closed form, on a host build with NO OCCT linked
- WHEN the assembler recognises the operand[B1], traces the seam[M1], splits the wall[B2], runs the analytic-face split + section-cap synthesis[B4], confirms the survivors[B3], welds[M0], and runs the mandatory self-verify
- THEN it SHALL return a watertight `Solid` (every edge shared by exactly two faces) whose enclosed volume equals the closed-form CUT value within the scale-relative tolerance — the five verbs compose into the first correct freeform boolean

#### Scenario: A composition that cannot complete robustly DECLINES to OCCT, B4 still landing (host)

- GIVEN a freeform↔analytic CUT case in which a verb declines (no clean single-chord M1 seam, a B2 `SplitDecline`, a B4 decline, or a survivor centroid B3 resolves to `On`/`Unknown` or on the wrong side) OR the welded result fails the watertight/volume self-verify
- WHEN the assembler is evaluated
- THEN it SHALL return a NULL `Shape` (→ OCCT) reporting which verb declined and the measured gap, SHALL NOT emit any partial/overlapping/leaky/wrong-volume solid, SHALL NOT guess any fragment's membership, and SHALL leave the B4 verb landed (proven by its isolated host tests) and the consumed subsystems byte-identical — the honest decline is a first-class outcome

### Requirement: First freeform CUT parity with OCCT BRepAlgoAPI_Cut through the facade (simulator gate)

When the first freeform↔analytic CUT assembles, it SHALL be verified on a booted iOS
simulator (OCCT linked) against the OCCT oracle: the same operand and cutting plane
SHALL be built BOTH natively and as OCCT shapes, and the native
`freeformHalfSpaceCut` result SHALL match `BRepAlgoAPI_Cut` (measured via `BRepGProp`)
on ENCLOSED VOLUME and SURFACE AREA within a scale-relative tolerance, SHALL be
WATERTIGHT (closed shell), SHALL agree on topology counts, and a batch of query points
SHALL agree with `BRepClass3d_SolidClassifier` on the native result with ZERO crisp
IN↔OUT disagreements. The engine's mandatory watertight + volume self-verify SHALL
DISCARD any native result that is not watertight or whose volume does not match the
OCCT oracle, falling through to `BRepAlgoAPI_Cut` so a wrong/leaky solid is NEVER
emitted. OCCT SHALL be referenced ONLY in the simulator proof harness / `src/engine/occt`;
the assembler and its inputs SHALL remain OCCT-free. The change SHALL NOT add any
`cc_*` entry point, and the count of any deferred cases SHALL be reported, not hidden.

#### Scenario: The native freeform CUT matches BRepAlgoAPI_Cut within tolerance (sim, parity)

- GIVEN the bowl-lidded convex-quad prism operand and the cutting plane built BOTH natively and as OCCT shapes on a booted simulator with OCCT linked
- WHEN the native `freeformHalfSpaceCut` produces a result and it is compared against `BRepAlgoAPI_Cut` via `BRepGProp` (volume/area) and topology counts, and against `BRepClass3d_SolidClassifier` on a query-point batch
- THEN the native volume/area SHALL match the OCCT oracle within the scale-relative tolerance, the native result SHALL be watertight with agreeing topology counts, and there SHALL be ZERO crisp IN↔OUT classification disagreements

#### Scenario: A non-watertight or wrong-volume native result is discarded to OCCT (sim)

- GIVEN a native first-CUT result that is deliberately perturbed to be non-watertight or wrong-volume on a booted simulator with OCCT linked
- WHEN the engine's mandatory watertight + volume self-verify is evaluated
- THEN the native result SHALL be DISCARDED and the boolean SHALL fall through to `BRepAlgoAPI_Cut` (OCCT), no wrong/leaky solid SHALL be emitted, and no tolerance SHALL be weakened to force the native path

### Requirement: The first freeform boolean is strictly additive and defers the B2 smooth-trim generalisation

Landing the B4 verb and the first freeform↔analytic CUT SHALL leave the consumed M2
substrate byte-identical: B1 `recogniseFreeformSolid` (`boolean/freeform_operand.h`),
B2 `splitFace` (`boolean/face_split.h`), B3 `classifyPointInMesh`
(`boolean/freeform_membership.h`), the M0 `SolidMesher` (`tessellate/solid_mesher.h`),
the M1 `WLine` tracer (`ssi/marching.h`), and the analytic `recogniseCurvedSolid`/
`classifyPoint` (`ssi_boolean.h`) SHALL be UNCHANGED and consumed WITHOUT modification.
The addition SHALL introduce no `cc_*` ABI change and SHALL keep `src/native/**` free of
OCCT includes. The change SHALL EXPLICITLY DEFER the **B2 smooth-trim (closed / circular
freeform wall) generalisation** (B1's blocker (i)): the bowl-lidded convex-quad prism
operand SIDESTEPS it by presenting a freeform wall whose outer loop is the convex
straight-edged quadrilateral B2 already splits, and NO closed/circular-wall split path
SHALL be written or stubbed in this change. The existing curved-boolean and freeform
substrate test suites SHALL pass with counts unchanged from the pre-change baseline.

#### Scenario: The M2 substrate and analytic paths are byte-identical after B4 is added (host + sim)

- GIVEN the B1/B2/B3/M0/M1 headers and the analytic `recogniseCurvedSolid`/`classifyPoint`, together with the curved-boolean, face-split, freeform-membership, and SSI-marching suites, before and after `half_space_cut.h` is added
- WHEN each is exercised at the same inputs and compared against the pre-change baseline
- THEN the six consumed subsystems SHALL be byte-identical (unchanged source, unchanged suite counts), there SHALL be zero OCCT includes under `src/native/**`, and no `cc_*` signature or POD layout SHALL have changed

#### Scenario: The B2 smooth-trim generalisation is deferred, not stubbed (host)

- GIVEN this change scoped to the bowl-lidded convex-quad prism operand (whose freeform wall has a convex straight-edged outer loop) and the B4 verb
- WHEN the change is inspected for closed/circular-freeform-wall handling
- THEN it SHALL contain NO closed/circular-wall split path (stubbed or otherwise), the operand SHALL sidestep blocker (i) within B2's shipped envelope, and the B2 smooth-trim generalisation SHALL be recorded as the deferred NEXT enabler with its measured blocker — not faked or partially implemented
