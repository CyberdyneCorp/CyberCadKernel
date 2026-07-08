# native-booleans

## ADDED Requirements

### Requirement: Two-operand inter-solid seam set for the freeform↔finite-box single-curved-seam pose, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only inter-solid
seam-set builder (`src/native/boolean/inter_solid_seam.h`) that, GIVEN operand `A` (a
`recogniseFreeformSolid`-admitted bowl-lidded convex-quad prism) and operand `B` (a
FINITE axis-aligned analytic box whose every face is a `Plane`), assembles the
intersection boundary of `A` and `B` for the SIMPLEST reachable pose: one in which
exactly ONE face of `B` slices `A`'s Bézier bowl wall in a single clean transversal
seam and `B`'s remaining faces cross only `A`'s PLANAR faces. The builder SHALL
compute (1) the single **curved seam** `S_curve` = (`B`'s cutting-plane) ∩ (`A`'s
Bézier wall) by driving the EXISTING `ssi::trace_intersection` / `traceWallSeam`
machinery UNCHANGED (requiring `points.size() ≥ 2` and status `Closed`/`BoundaryExit`),
and (2) each **straight seam** `S_lin,i` = (a `B` face) ∩ (an `A` planar face) as the
plane∩plane line clipped to both face polygons by the SAME analytic clip the planar
boolean uses. It SHALL splice `S_curve` and the `{S_lin,i}` into ONE ordered loop and
VERIFY it is SIMPLE (no non-adjacent-edge intersection) and CLOSED, recording each
crossing as `(faceId, edgeParam)` so the split step replays it.

`buildInterSolidSeam(A, B)` SHALL return a typed DECLINE — carrying the measured
blocker — and NO partial seam when: the operands' AABBs do not overlap (`NoOverlap`);
zero or more than ONE `B` face crosses the freeform wall (`NotSingleCurvedCut` — the
honest scope boundary of this single-curved-seam slice); the curved seam is not one
well-formed transversal chord (`SeamUnusable`); or the spliced loop is not simple and
closed (`SeamNotClosed`). The builder SHALL remain OCCT-free, SHALL introduce no `cc_*`
ABI surface, SHALL NOT weaken any tolerance to force a loop, and SHALL keep its
per-function cognitive complexity within the backend band via per-pair helpers.

#### Scenario: The finite box's one planar face cuts the freeform wall and the seam set closes into one simple loop (host, no OCCT)

- GIVEN the bowl-lidded convex-quad prism `A` and a finite axis-aligned box `B` positioned so exactly one `B` face slices `A`'s Bézier wall and `B`'s other faces cross only `A`'s planar walls/bottom, built on the host with NO OCCT
- WHEN `buildInterSolidSeam(A, B)` traces the single curved seam via the existing `traceWallSeam` machinery and computes the plane∩plane straight seams
- THEN it SHALL return ONE ordered, simple, closed intersection loop of the curved seam spliced to the straight seam segments, with each crossing recorded as `(faceId, edgeParam)`, and NO partial seam

#### Scenario: A pose outside the single-curved-seam envelope DECLINES with a measured blocker (host)

- GIVEN a pose in which the box misses `A`, or zero/more-than-one box face crosses the freeform wall, or the curved seam is not one clean transversal chord, or the spliced loop is not simple/closed, built on the host with NO OCCT
- WHEN `buildInterSolidSeam(A, B)` runs
- THEN it SHALL return a typed DECLINE (`NoOverlap` / `NotSingleCurvedCut` / `SeamUnusable` / `SeamNotClosed`) identifying the blocker, SHALL emit NO partial seam, and SHALL NOT weaken a tolerance to force a loop

### Requirement: Two-operand face split of BOTH operands along the inter-solid seam set, or DECLINE

The native boolean library SHALL split BOTH operands' crossed faces along the
inter-solid seam set: `A`'s Bézier bowl wall along `S_curve` via B2 `splitFace`
(`boolean/face_split.h`) consumed UNCHANGED into two tiling sub-faces, and EACH crossed
PLANAR face of `A` and of `B` along its straight seam segment `S_lin,i` via the landed
analytic-face-split primitive (`boolean/half_space_cut.h`, `hscdetail`) consumed
UNCHANGED into keep/discard sub-faces with EXACTLY two recorded crossings. Each seam
edge SHALL be built ONCE and SHARED bit-exactly between the two sub-faces that meet
along it (`A`'s Bézier sub-face ↔ `B`'s cut-face sub-face for `S_curve`; the two planar
sub-faces for each `S_lin,i`) — the watertight weld contract. Uncrossed faces SHALL pass
through whole. A split that is tangent, at a vertex, or that does not yield exactly two
crossings SHALL DECLINE (`SplitDecline`), emitting no partial split. The consumed B2 and
analytic-face-split primitives SHALL remain BYTE-IDENTICAL; the split step SHALL add no
`cc_*` ABI surface and weaken no tolerance.

#### Scenario: Both operands' crossed faces split along the seam set with bit-exact shared seam edges (host, no OCCT)

- GIVEN the closed inter-solid seam set from `buildInterSolidSeam(A, B)` on the host with NO OCCT
- WHEN the split step runs B2 `splitFace` on `A`'s Bézier wall along `S_curve` and the landed analytic-face-split on every crossed planar face of `A` and `B` along its `S_lin,i`
- THEN each crossed face SHALL split into the correct sub-faces with exactly-two-crossings recorded, each seam edge SHALL be a SINGLE shared edge (built once, opposite orientation) between its two incident sub-faces, and every uncrossed face SHALL pass through whole

#### Scenario: A degenerate crossing DECLINES without emitting a partial split (host)

- GIVEN a crossing that is tangent, lands on a vertex, or does not yield exactly two crossings on some crossed face, built on the host with NO OCCT
- WHEN the split step runs
- THEN it SHALL return `SplitDecline` identifying the offending face, SHALL emit NO partial or overlapping split, and SHALL leave B2 and the analytic-face-split primitive byte-identical

### Requirement: Two-operand shell classification by B3 membership (A-in/A-out, B-in/B-out), or DECLINE

The native boolean library SHALL classify every face fragment of the split operands as
inside or outside the OTHER solid using B3 `classifyPointInMesh`
(`boolean/freeform_membership.h`) consumed UNCHANGED. It SHALL mesh `A` and `B`
(pre-split) ONCE each with the M0 `SolidMesher`, then classify each `A` fragment's
interior centroid against `B`'s boundary mesh (→ `A`-in / `A`-out) and each `B`
fragment's interior centroid against `A`'s boundary mesh (→ `B`-in / `B`-out). A
fragment whose centroid classifies `On` or `Unknown`, or lands in the membership
ON-band, SHALL DECLINE (`ClassifyAmbiguous`) — a fragment's membership SHALL NEVER be
guessed. The consumed B3 and M0 paths SHALL remain BYTE-IDENTICAL; the classify step
SHALL add no `cc_*` ABI surface and weaken no tolerance.

#### Scenario: Every fragment of both operands classifies crisply in/out the other solid (host, no OCCT)

- GIVEN the split fragments of `A` and `B` and the pre-split M0 boundary meshes `meshA`, `meshB`, built on the host with NO OCCT
- WHEN each `A` fragment centroid is classified against `meshB` and each `B` fragment centroid against `meshA` via B3 `classifyPointInMesh`
- THEN every fragment SHALL resolve crisply to `A`-in / `A`-out / `B`-in / `B`-out with no `On`/`Unknown` verdict, partitioning both shells for the per-op survivor selection

#### Scenario: An ambiguous fragment membership DECLINES rather than guess (host)

- GIVEN a fragment whose interior centroid classifies `On`/`Unknown` or lands in the ON-band against the other solid's mesh, built on the host with NO OCCT
- WHEN the classify step runs
- THEN it SHALL return `ClassifyAmbiguous` identifying the fragment, SHALL NOT guess its membership, and SHALL leave B3 and M0 byte-identical

### Requirement: First two-operand freeform↔finite-box FUSE with mandatory self-verify, or HONEST DECLINE

The native boolean library SHALL compute the FIRST two-operand freeform boolean —
`freeformBooleanTwoOperand(A, B, op)` in `src/native/boolean/two_operand.h` — that FUSES
the bowl-lidded convex-quad prism `A` with a finite axis-aligned analytic box `B` over
the single-curved-seam pose, composed ENTIRELY from the landed verbs consumed unchanged:
(1) `buildInterSolidSeam`[seam]; (2) the two-operand split of BOTH operands[B2 + analytic
split]; (3) the two-operand shell classification[B3]; (4) the per-op weld selecting the
survivor set — FUSE = `A`-out ∪ `B`-out, CUT = `A`-out ∪ (`B`-in reversed), COMMON =
`A`-in ∪ `B`-in — welded via the `assemble.h` `VertexPool` into one shell → `Solid`.
Before returning, the assembler SHALL run the MANDATORY self-verify: the welded result
SHALL be WATERTIGHT (every edge shared by exactly two faces) AND its enclosed volume
SHALL match the INDEPENDENT closed-form op value (for FUSE, `V(A ∪ B) = V(A) + V(B) −
V(A ∩ B)`) within a scale-relative tolerance sized to the M0 deflection; a result that
FAILS SHALL be DISCARDED (NULL `Shape` → OCCT).

The assembler SHALL DECLINE (NULL `Shape` → OCCT) whenever any verb declines — a
`buildInterSolidSeam` decline, a `SplitDecline`, a `ClassifyAmbiguous`, a `WeldOpen`, or
a self-verify failure — recording WHICH verb declined and the measured gap, and SHALL
NEVER emit a partial, overlapping, leaky, or wrong-volume solid nor guess a fragment's
membership. If the full FUSE weld is not robustly reachable this wave (the known
shared-curved-edge deflection fragility), the change SHALL LAND the provable subset — the
inter-solid seam-set trace and the two-operand split/classify verbs proven in isolation —
and HONEST-DECLINE the weld, naming the M0 shared-curved-edge single-sampling fix as the
next enabler. The consumed B1/B2/B3/M0/M1 subsystems, the analytic
`recogniseCurvedSolid`/`classifyPoint`, and the landed `freeformHalfSpaceCut` CUT/COMMON
path SHALL remain BYTE-IDENTICAL; no `cc_*` ABI SHALL be added; `src/native/**` SHALL keep
zero OCCT includes; no tolerance SHALL be weakened; OCCT SHALL remain the oracle and the
fallback. NO FUSE stub SHALL be written.

#### Scenario: The bowl-lidded prism FUSED with a finite box assembles watertight at the closed-form union volume (host, analytic — no OCCT)

- GIVEN operand `A` (bowl-lidded convex-quad prism) and a finite axis-aligned box `B` in the single-curved-seam pose, whose union volume `V(A) + V(B) − V(A ∩ B)` is KNOWN in closed form, on a host build with NO OCCT linked and a deflection at which the shared curved seam welds coincident
- WHEN `freeformBooleanTwoOperand(A, B, FUSE)` composes seam-set → split-both → classify-both → per-op weld → mandatory self-verify
- THEN it SHALL return a watertight `Solid` (every edge shared by exactly two faces) whose enclosed volume equals the closed-form union value within the scale-relative deflection band — the first correct two-operand freeform boolean

#### Scenario: A composition that cannot complete robustly DECLINES to OCCT with the provable subset landed (host)

- GIVEN a two-operand FUSE case in which a verb declines (`buildInterSolidSeam` decline, `SplitDecline`, `ClassifyAmbiguous`, `WeldOpen`) OR the welded result fails the watertight/volume self-verify
- WHEN the assembler is evaluated
- THEN it SHALL return a NULL `Shape` (→ OCCT) reporting which verb declined and the measured gap, SHALL NOT emit any partial/overlapping/leaky/wrong-volume solid, SHALL NOT guess any fragment's membership, and SHALL leave the inter-solid seam-set trace and the two-operand split/classify verbs landed (proven by their isolated host tests) — the honest decline is a first-class outcome

### Requirement: Two-operand CUT and COMMON as the complementary survivor-sets of the FUSE machinery

The native boolean library SHALL reach two-operand **CUT** (`A − B`) and **COMMON**
(`A ∩ B`) through the SAME `freeformBooleanTwoOperand` machinery with NO new geometry
verb, by flipping only the per-op survivor selection and seam orientation: CUT = `A`-out
∪ (`B`-in reversed), COMMON = `A`-in ∪ `B`-in — versus FUSE = `A`-out ∪ `B`-out. The
seam-set builder, the two-operand split, the shell classifier, and the weld SHALL be
reused; only the survivor set and the reversed-face orientation SHALL differ. Each op's
result SHALL be gated on its OWN independent closed-form volume — CUT on `V(A) − V(A ∩ B)`,
COMMON on `V(A ∩ B)` — under the mandatory watertight + volume self-verify, and SHALL
DISCARD → OCCT on failure. The consumed verbs, B1/B2/B3/M0/M1, the analytic paths, and the
landed single-operand `freeformHalfSpaceCut` CUT/COMMON SHALL remain BYTE-IDENTICAL; no
`cc_*` ABI SHALL be added; `src/native/**` SHALL keep zero OCCT includes; no tolerance
SHALL be weakened.

#### Scenario: Two-operand CUT and COMMON reuse the FUSE machinery with flipped survivor sets (host, no OCCT)

- GIVEN operand `A` and the finite box `B` in the single-curved-seam pose on a host build with NO OCCT, at a both-sides-weld deflection
- WHEN `freeformBooleanTwoOperand(A, B, CUT)` and `(A, B, COMMON)` run, flipping only the survivor selection and reversed-face orientation
- THEN CUT SHALL weld a watertight `Solid` at `V(A) − V(A ∩ B)` and COMMON at `V(A ∩ B)`, each within the scale-relative band, with the seam-set / split / classify / weld verbs and the landed single-operand CUT/COMMON path byte-identical

### Requirement: Host-analytic closed-form two-operand volume oracle (no mesher, no OCCT)

The native boolean library SHALL provide a mesh-free, OCCT-free closed-form oracle for
the two-operand op volumes: `V(A)` from the exact per-triangle quadratic-moment bowl
integrand `∫∫_Q (h0 + a·(x² + y²)) dA` (the landed CUT/COMMON oracle), `V(B)` the box
volume, and `V(A ∩ B)` the closed-form clip of the bowl integrand over the box footprint.
The identities `V(A ∪ B) = V(A) + V(B) − V(A ∩ B)`, `V(A − B) = V(A) − V(A ∩ B)`, and
`V(A ∩ B) + V(A − B) = V(A)` SHALL hold to machine precision, independent of any mesh.
This oracle SHALL be the PRIMARY two-operand correctness check (the HOST-ANALYTIC gate);
when a two-operand solid welds watertight at a deflection, its MESH enclosed volume SHALL
also match the corresponding closed-form value within the doubled deflection band. The
oracle SHALL depend on NO mesher and NO OCCT.

#### Scenario: The closed-form two-operand identities close to machine precision (host, no OCCT)

- GIVEN `V(A)`, `V(B)`, and `V(A ∩ B)` evaluated by the exact closed-form oracle with NO OCCT
- WHEN the union / difference / partition identities are checked
- THEN `|V(A ∪ B) − (V(A) + V(B) − V(A ∩ B))| ≤ 1e-12`, `|V(A ∩ B) + V(A − B) − V(A)| ≤ 1e-12`, and `0 < V(A ∩ B) < V(A)` SHALL hold — FUSE, CUT, and COMMON are exactly related independent of any mesh

#### Scenario: A watertight two-operand mesh matches its closed-form volume (host)

- GIVEN a deflection at which `freeformBooleanTwoOperand(A, B, FUSE)` welds watertight on a host build with NO OCCT
- WHEN the result is meshed and its enclosed volume compared to `V(A ∪ B)`
- THEN the mesh SHALL be watertight and its volume SHALL equal the closed-form union value within the doubled scale-relative deflection band — the mesh-level check confirms the closed-form gate

### Requirement: First two-operand freeform FUSE parity with OCCT BRepAlgoAPI_Fuse (simulator gate, spatial)

When the first two-operand freeform FUSE assembles, it SHALL be verified on a booted iOS
simulator (OCCT linked) against the OCCT oracle: operand `A` and the finite box `B` SHALL
be built BOTH natively and as OCCT shapes, and the native `freeformBooleanTwoOperand(A, B,
FUSE)` result SHALL match `BRepAlgoAPI_Fuse` (measured via `BRepGProp`) on ENCLOSED VOLUME
and SURFACE AREA within a scale-relative tolerance, SHALL be WATERTIGHT (closed shell),
SHALL agree on TOPOLOGY counts (faces / edges / vertices in the reachable envelope), and
SHALL match on a **spatial BBOX** (not volume-only); additionally a batch of query points
SHALL agree with `BRepClass3d_SolidClassifier` on the native result with ZERO crisp IN↔OUT
disagreements. The corresponding CUT and COMMON SHALL match `BRepAlgoAPI_Cut` /
`BRepAlgoAPI_Common` under the same spatial criteria. The engine's mandatory watertight +
volume self-verify SHALL DISCARD any native result that is not watertight or whose volume
does not match the OCCT oracle, falling through to the OCCT boolean so a wrong/leaky solid
is NEVER emitted. OCCT SHALL be referenced ONLY in the simulator proof harness /
`src/engine/occt`; the assembler and its inputs SHALL remain OCCT-free. The change SHALL
add no `cc_*` entry point, and the count of any deferred cases SHALL be reported, not hidden.

#### Scenario: The native two-operand FUSE matches BRepAlgoAPI_Fuse spatially (sim, parity)

- GIVEN operand `A` and the finite box `B` built BOTH natively and as OCCT shapes on a booted simulator with OCCT linked
- WHEN the native `freeformBooleanTwoOperand(A, B, FUSE)` produces a result and it is compared to `BRepAlgoAPI_Fuse` via `BRepGProp` (volume / area), topology counts, spatial BBOX, and `BRepClass3d_SolidClassifier` on a query-point batch
- THEN the native volume / area SHALL match the OCCT oracle within tolerance, the native result SHALL be watertight with agreeing topology counts and a matching spatial BBOX, and there SHALL be ZERO crisp IN↔OUT classification disagreements

#### Scenario: A non-watertight or wrong-volume two-operand result is discarded to OCCT (sim)

- GIVEN a native two-operand FUSE result deliberately perturbed to be non-watertight or wrong-volume on a booted simulator with OCCT linked
- WHEN the engine's mandatory watertight + volume self-verify is evaluated
- THEN the native result SHALL be DISCARDED and the boolean SHALL fall through to `BRepAlgoAPI_Fuse` (OCCT), no wrong/leaky solid SHALL be emitted, and no tolerance SHALL be weakened to force the native path

### Requirement: The two-operand freeform boolean is strictly additive and keeps the landed single-operand path byte-identical

Landing the two-operand freeform boolean SHALL leave the consumed substrate
byte-identical: B1 `recogniseFreeformSolid` (`boolean/freeform_operand.h`), B2 `splitFace`
(`boolean/face_split.h`), B3 `classifyPointInMesh` (`boolean/freeform_membership.h`), the
M0 `SolidMesher` (`tessellate/solid_mesher.h`), the M1 `WLine` tracer (`ssi/marching.h`),
the analytic `recogniseCurvedSolid`/`classifyPoint` (`ssi_boolean.h`), and the landed
single-operand `freeformHalfSpaceCut` CUT/COMMON (`boolean/half_space_cut.h`) SHALL be
UNCHANGED and consumed WITHOUT modification. The new code SHALL live in NEW headers
(`inter_solid_seam.h`, `two_operand.h`) and SHALL reuse the landed analytic-face-split and
`assemble.h` weld primitives unchanged. The addition SHALL introduce no `cc_*` ABI change
and SHALL keep `src/native/**` free of OCCT includes. Any M1 SSI touch SHALL be strictly
ADDITIVE with every prior seeding/marching control BYTE-FROZEN and proven inert on all
existing callers before use. The existing native-booleans and native-ssi suites SHALL pass
with counts unchanged from the pre-change baseline.

#### Scenario: The landed single-operand path and consumed subsystems are byte-identical after the two-operand verb is added (host + sim)

- GIVEN the B1/B2/B3/M0/M1 headers, the analytic `recogniseCurvedSolid`/`classifyPoint`, and the landed `freeformHalfSpaceCut` CUT/COMMON, together with the native-booleans and native-ssi suites, before and after this change
- WHEN each is exercised at the same inputs and compared against the pre-change baseline
- THEN the consumed subsystems and the landed single-operand CUT/COMMON path SHALL be byte-identical (unchanged source, unchanged suite counts), there SHALL be zero OCCT includes under `src/native/**`, and no `cc_*` signature or POD layout SHALL have changed

#### Scenario: Any M1 SSI touch is additive with prior controls byte-frozen (host)

- GIVEN this change and any additive helper it adds to the M1 marcher for full-seam coverage
- WHEN the M1 seeding/marching controls are exercised on every existing SSI caller before the helper is used
- THEN every prior seeding/marching output SHALL be byte-identical to the pre-change baseline (the helper is proven inert), and the addition SHALL be a new defaulted option that changes no existing caller

### Requirement: Honest decline with the next sharpened blocker is a first-class two-operand outcome

The change SHALL treat an honest decline carrying the next sharpened blocker as a
first-class outcome: if the general single-curved-seam FUSE weld is not robustly
reachable this wave, the change SHALL land whatever piece is PROVABLE and return the
NEXT sharpened blocker honestly, choosing the sharpest reachable level: (1) full FUSE self-verified watertight at
the closed-form union volume and matching `BRepAlgoAPI_Fuse` in sim; else (2) a constrained
FUSE at the deflection(s) where the shared curved seam welds coincident, the other
deflections declining, naming the M0 shared-curved-edge single-sampling fix as the
deflection-independence enabler; else (3) the inter-solid seam-set trace plus the
two-operand split and classify verbs proven in ISOLATION (the seam loop closes, both
operands split, every fragment classifies crisply), the weld declined with its measured
gap; else (4) a sharpened seam-graph-assembly decline with the failing measurement. Every
level SHALL emit either a self-verified-correct solid OR a NULL `Shape` → OCCT; NO partial,
overlapping, leaky, or wrong-volume solid, NO guessed membership, NO FUSE stub, and NO
weakened tolerance SHALL be shipped. The reached level and the measured next blocker SHALL
be recorded, not hidden.

#### Scenario: The provable subset lands and the weld is honestly declined with the next enabler (host)

- GIVEN a wave in which the full FUSE weld is not robustly reachable (the shared-curved-edge deflection fragility) but the seam-set trace and the two-operand split/classify verbs are provable in isolation
- WHEN the change is landed
- THEN it SHALL land `buildInterSolidSeam` + the two-operand split + the shell classifier proven by isolated host tests (the seam loop closes, both operands split, every fragment classifies crisply), SHALL DECLINE the weld with the measured gap, SHALL name the M0 shared-curved-edge single-sampling fix as the next enabler, and SHALL ship NO FUSE stub and NO weakened tolerance — a first-class honest outcome

#### Scenario: No two-operand path emits a leak at any deflection (host, no OCCT)

- GIVEN operand `A` and the finite box `B`, swept over a range of deflections for FUSE / CUT / COMMON on a host build with NO OCCT
- WHEN each `freeformBooleanTwoOperand` result is meshed and audited
- THEN EVERY result SHALL be `isNull()` OR watertight (a non-watertight or wrong-volume solid SHALL NEVER be returned), and the deflections at which each op declines SHALL be attributable to the shared-curved-edge weld mismatch — the honest-decline discipline holds at 100% of the sweep
