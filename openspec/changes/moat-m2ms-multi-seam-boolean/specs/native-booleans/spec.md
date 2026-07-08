# native-booleans

## ADDED Requirements

### Requirement: Two-operand seam-GRAPH builder for the freeform↔finite-box two-cutting-face junction pose, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only seam-graph builder
(`src/native/boolean/seam_graph.h`) that, GIVEN operand `A` (a
`recogniseFreeformSolid`-admitted bowl-lidded convex-quad prism) and operand `B` (a
FINITE axis-aligned analytic box whose every face is a `Plane`), assembles the
inter-solid intersection as a **seam graph** for the FIRST multi-seam pose: one in
which EXACTLY TWO adjacent faces of `B` slice `A`'s Bézier bowl wall in two distinct
curved arcs that MEET at a shared **junction vertex** on the wall, and `B`'s remaining
faces contain `A`. The builder SHALL (1) identify the cutting-face SET via the pole-hull
straddle predicate reused UNCHANGED, requiring exactly two adjacent cutting faces; (2)
compute the junction vertex `J` where the two cutting planes' shared box edge pierces
the wall (the point on the Bézier surface where both cutting-plane signed distances
vanish); (3) trace EACH curved arc by driving the EXISTING `traceWallSeam` /
`ssi::trace_intersection` machinery UNCHANGED (requiring `points.size() ≥ 2` and status
`Closed`/`BoundaryExit`); (4) CLIP each full chord at `J` and JOIN the two `boundary→J`
sub-arcs into ONE ordered boundary-to-boundary polyline bent at `J`; and (5) compute each
straight seam `S_lin,i` = (a cutting face) ∩ (an `A` planar face) by the SAME analytic
clip the planar boolean uses.

`buildSeamGraph(A, B)` SHALL return a typed DECLINE — carrying the measured blocker —
and NO partial graph when: `B` is not all-planar (`NotPlanarB`); the AABBs do not overlap
(`NoOverlap`); the number of cutting faces is not exactly two adjacent faces
(`NotTwoCuttingFaces` — the honest scope boundary of this two-seam slice); a non-cutting
face does not contain `A` (`NotContained`); the junction `J` does not converge inside the
trimmed wall (`JunctionUnusable`); an arc is not one well-formed transversal chord
(`SeamUnusable`); or the two sub-arcs do not join coincident at `J` within the weld
tolerance (`JunctionNotJoined`). The builder SHALL remain OCCT-free, SHALL introduce no
`cc_*` ABI surface, SHALL NOT weaken any tolerance to force a junction or a join, SHALL
consume the landed `inter_solid_seam.h` primitives (`planeStraddlesWall`,
`aabbInsidePlane`, `tracePlaneOf`, `wallWorldPoles`) and every `hscdetail::` primitive
BYTE-IDENTICAL, and SHALL keep its per-function cognitive complexity within the backend
band via per-arc / per-junction helpers.

#### Scenario: Two adjacent box faces cut the wall and the two arcs join at the junction into one bent seam (host, no OCCT)

- GIVEN the bowl-lidded convex-quad prism `A` and a finite axis-aligned box `B` straddling one corner of `A` so exactly two adjacent `B` faces slice `A`'s Bézier wall and the other faces contain `A`, built on the host with NO OCCT
- WHEN `buildSeamGraph(A, B)` identifies the two cutting faces, computes the junction vertex `J`, traces each arc via the existing `traceWallSeam` machinery, and clips-and-joins the two `boundary→J` sub-arcs
- THEN it SHALL return a seam graph whose two curved arcs meet coincident at `J` (within the weld tolerance) forming ONE ordered boundary-to-boundary bent polyline, together with the straight seam segments, and NO partial graph

#### Scenario: A pose outside the two-cutting-face junction envelope DECLINES with a measured blocker (host)

- GIVEN a pose in which `B` misses `A`, or the number of wall-cutting faces is not exactly two adjacent faces, or a non-cutting face does not contain `A`, or the junction `J` does not converge on the wall, or an arc is not a clean transversal chord, or the two sub-arcs do not join at `J`, built on the host with NO OCCT
- WHEN `buildSeamGraph(A, B)` runs
- THEN it SHALL return a typed DECLINE (`NotPlanarB` / `NoOverlap` / `NotTwoCuttingFaces` / `NotContained` / `JunctionUnusable` / `SeamUnusable` / `JunctionNotJoined`) identifying the blocker, SHALL emit NO partial graph, and SHALL NOT weaken a tolerance to force a junction or a join

### Requirement: Junction-joined two-operand face split of BOTH operands along the seam graph, or DECLINE

The native boolean library SHALL split BOTH operands' crossed faces along the seam
graph: `A`'s Bézier bowl wall along the junction-joined bent seam via B2 `splitFace`
(`boolean/face_split.h`) consumed UNCHANGED in a SINGLE call into the corner sub-face and
the L-shaped survivor sub-face (the junction vertex `J` being an interior vertex of the
boundary-to-boundary bent seam), and EACH crossed PLANAR face of `A` and of `B` along its
straight seam segment via the landed analytic-face-split primitive
(`boolean/half_space_cut.h`, `hscdetail`) consumed UNCHANGED into keep/discard sub-faces
with EXACTLY two recorded crossings. Each seam edge (each arc segment and each straight
chord) SHALL be built ONCE via `edgeFromPiece` and SHARED bit-exactly between the two
sub-faces that meet along it, and the junction vertex `J` SHALL be built ONCE as a single
shared vertex referenced by every sub-face incident to it — the watertight weld contract.
Uncrossed faces SHALL pass through whole. A split that is tangent, at a vertex, or that
does not yield exactly two crossings SHALL DECLINE (`SplitDecline`), emitting no partial
split. The consumed B2 and analytic-face-split primitives SHALL remain BYTE-IDENTICAL;
the split step SHALL add no `cc_*` ABI surface and weaken no tolerance.

#### Scenario: The wall splits into corner + L-survivor along the bent seam with a single shared junction vertex (host, no OCCT)

- GIVEN the seam graph from `buildSeamGraph(A, B)` on the host with NO OCCT
- WHEN B2 `splitFace` runs ONCE on `A`'s Bézier wall along the junction-joined bent seam and the landed analytic-face-split runs on every crossed planar face of `A` and `B`
- THEN the wall SHALL split into the corner sub-face and the L-shaped survivor sub-face with the junction vertex `J` a single shared interior vertex, each seam edge SHALL be a SINGLE shared edge between its two incident sub-faces, and every uncrossed face SHALL pass through whole

#### Scenario: A degenerate crossing DECLINES without emitting a partial split (host)

- GIVEN a crossing that is tangent, lands on a vertex, or does not yield exactly two crossings on some crossed face, built on the host with NO OCCT
- WHEN the split step runs
- THEN it SHALL return `SplitDecline` identifying the offending face, SHALL emit NO partial or overlapping split, and SHALL leave B2 and the analytic-face-split primitive byte-identical

### Requirement: Seam-graph shell classification by B3 membership (A-in/A-out, B-in/B-out), or DECLINE

The native boolean library SHALL classify every face fragment of the split operands as
inside or outside the OTHER solid using B3 `classifyPointInMesh`
(`boolean/freeform_membership.h`) consumed UNCHANGED. It SHALL mesh `A` and `B`
(pre-split) ONCE each with the M0 `SolidMesher`, then classify each `A` fragment's
interior centroid against `B`'s boundary mesh (→ `A`-in / `A`-out) and each `B`
fragment's interior centroid against `A`'s boundary mesh (→ `B`-in / `B`-out). A fragment
whose centroid classifies `On` or `Unknown`, or lands in the membership ON-band, SHALL
DECLINE (`ClassifyAmbiguous`) — a fragment's membership SHALL NEVER be guessed. The
consumed B3 and M0 paths SHALL remain BYTE-IDENTICAL; the classify step SHALL add no
`cc_*` ABI surface and weaken no tolerance.

#### Scenario: Every fragment of both operands classifies crisply in/out the other solid (host, no OCCT)

- GIVEN the split fragments of `A` and `B` and the pre-split M0 boundary meshes `meshA`, `meshB`, built on the host with NO OCCT
- WHEN each `A` fragment centroid is classified against `meshB` and each `B` fragment centroid against `meshA` via B3 `classifyPointInMesh`
- THEN every fragment SHALL resolve crisply to `A`-in / `A`-out / `B`-in / `B`-out with no `On`/`Unknown` verdict, partitioning both shells for the per-op survivor selection

#### Scenario: An ambiguous fragment membership DECLINES rather than guess (host)

- GIVEN a fragment whose interior centroid classifies `On`/`Unknown` or lands in the ON-band against the other solid's mesh, built on the host with NO OCCT
- WHEN the classify step runs
- THEN it SHALL return `ClassifyAmbiguous` identifying the fragment, SHALL NOT guess its membership, and SHALL leave B3 and M0 byte-identical

### Requirement: First multi-seam freeform↔finite-box FUSE with mandatory self-verify, or HONEST DECLINE

The native boolean library SHALL compute the FIRST multi-seam (seam-graph) two-operand
freeform boolean — `freeformBooleanMultiSeam(A, B, op)` in
`src/native/boolean/multi_seam.h` — that FUSES the bowl-lidded convex-quad prism `A` with
a finite axis-aligned analytic box `B` over the two-cutting-face junction pose, composed
ENTIRELY from the landed verbs consumed unchanged: (1) `buildSeamGraph`[seam graph]; (2)
the junction-joined two-operand split of BOTH operands[B2 + analytic split]; (3) the
two-operand shell classification[B3]; (4) the per-op weld selecting the survivor set —
FUSE = `A`-L-survivor ∪ `B`-out ∪ the two corner-notched cutting sub-faces — welded via
the `assemble.h` `VertexPool` into one shell → `Solid` with the junction vertex `J` a
single valence-3 shared vertex. Before returning, the assembler SHALL run the MANDATORY
self-verify: the welded result SHALL be WATERTIGHT (every edge shared by exactly two
faces) AND its enclosed volume SHALL match the INDEPENDENT closed-form op value (for FUSE,
`V(A ∪ B) = V(A) + V(B) − V(A ∩ B)`, the corner-clip union) within a scale-relative
tolerance sized to the M0 deflection; a result that FAILS SHALL be DISCARDED (NULL `Shape`
→ OCCT).

The assembler SHALL DECLINE (NULL `Shape` → OCCT) whenever any verb declines — a
`buildSeamGraph` decline, a `SplitDecline`, a `ClassifyAmbiguous`, a `WeldOpen`, or a
self-verify failure — recording WHICH verb declined and the measured gap, and SHALL NEVER
emit a partial, overlapping, leaky, or wrong-volume solid nor guess a fragment's
membership. If the full seam-graph weld is not robustly reachable this wave (the sharpened
junction-vertex weld coincidence), the change SHALL LAND the provable subset — the
seam-graph builder, the junction-vertex computation, and the junction-joined split proven
in isolation — and HONEST-DECLINE the weld, naming the valence-3 junction single-sampling
weld as the next enabler. The consumed B1/B2/B3/M0/M1 subsystems, the analytic
`recogniseCurvedSolid`/`classifyPoint`, and the landed single-seam `inter_solid_seam.h` /
`two_operand.h` path SHALL remain BYTE-IDENTICAL; no `cc_*` ABI SHALL be added;
`src/native/**` SHALL keep zero OCCT includes; no tolerance SHALL be weakened; OCCT SHALL
remain the oracle and the fallback. NO seam-graph stub SHALL be written.

#### Scenario: The bowl-lidded prism FUSED with a corner box assembles watertight at the closed-form union volume (host, analytic — no OCCT)

- GIVEN operand `A` (bowl-lidded convex-quad prism) and a finite axis-aligned box `B` straddling one corner in the two-cutting-face junction pose, whose union volume `V(A) + V(B) − V(A ∩ B)` is KNOWN in closed form, on a host build with NO OCCT and a deflection at which the junction and the two arcs weld coincident
- WHEN `freeformBooleanMultiSeam(A, B, FUSE)` composes seam-graph → junction-joined split → classify → per-op weld → mandatory self-verify
- THEN it SHALL return a watertight `Solid` (every edge shared by exactly two faces, including the valence-3 junction vertex `J`) whose enclosed volume equals the closed-form union value within the scale-relative deflection band — the first correct multi-seam freeform boolean

#### Scenario: A composition that cannot complete robustly DECLINES to OCCT with the provable subset landed (host)

- GIVEN a multi-seam FUSE case in which a verb declines (`buildSeamGraph` decline, `SplitDecline`, `ClassifyAmbiguous`, `WeldOpen`) OR the welded result fails the watertight/volume self-verify
- WHEN the assembler is evaluated
- THEN it SHALL return a NULL `Shape` (→ OCCT) reporting which verb declined and the measured gap, SHALL NOT emit any partial/overlapping/leaky/wrong-volume solid, SHALL NOT guess any fragment's membership, and SHALL leave the seam-graph builder and the junction-joined split proven by their isolated host tests — the honest decline is a first-class outcome

### Requirement: Multi-seam CUT and COMMON as genuine seam-graph welds (not a half-space reduction)

The native boolean library SHALL reach multi-seam **CUT** (`A − B`, an L-shaped
bowl-topped solid) and **COMMON** (`A ∩ B`, the corner solid) through the SAME
`freeformBooleanMultiSeam` machinery with NO new geometry verb, by flipping only the
per-op survivor selection and seam orientation: CUT = `A`-L-survivor ∪ (the two cutting
sub-faces reversed as the corner cavity walls), COMMON = `A`-corner sub-face ∪ the two
cutting sub-faces — versus FUSE = `A`-L-survivor ∪ `B`-out ∪ the two cutting sub-faces.
Because `B` removes a re-entrant quadrant CORNER of `A`, CUT and COMMON SHALL NOT reduce
to the single-operand `freeformHalfSpaceCut` (the removed region is not a half-space of
`A`); they SHALL be assembled by the seam-graph weld. Each op's result SHALL be gated on
its OWN independent closed-form volume — CUT on `V(A) − V(A ∩ B)`, COMMON on `V(A ∩ B)` —
under the mandatory watertight + volume self-verify, and SHALL DISCARD → OCCT on failure.
The consumed verbs, B1/B2/B3/M0/M1, the analytic paths, and the landed single-operand and
single-seam CUT/COMMON paths SHALL remain BYTE-IDENTICAL; no `cc_*` ABI SHALL be added;
`src/native/**` SHALL keep zero OCCT includes; no tolerance SHALL be weakened.

#### Scenario: Multi-seam CUT and COMMON reuse the FUSE machinery with flipped survivor sets (host, no OCCT)

- GIVEN operand `A` and the corner box `B` in the two-cutting-face junction pose on a host build with NO OCCT, at a junction-weld deflection
- WHEN `freeformBooleanMultiSeam(A, B, CUT)` and `(A, B, COMMON)` run, flipping only the survivor selection and reversed-face orientation
- THEN CUT SHALL weld a watertight L-shaped `Solid` at `V(A) − V(A ∩ B)` and COMMON a watertight corner `Solid` at `V(A ∩ B)`, each within the scale-relative band, WITHOUT delegating to `freeformHalfSpaceCut`, with the seam-graph / split / classify / weld verbs and the landed single-operand and single-seam CUT/COMMON paths byte-identical

### Requirement: Host-analytic closed-form multi-seam corner-volume oracle (no mesher, no OCCT)

The native boolean library SHALL provide a mesh-free, OCCT-free closed-form oracle for
the multi-seam op volumes: `V(A)` from the exact per-triangle quadratic-moment bowl
integrand `∫∫_Q (h0 + a·(x² + y²)) dA` (the landed oracle), `V(B)` the box volume, and
`V(A ∩ B)` the closed-form clip of the bowl integrand over the **corner footprint**
`Q ∩ {x ≥ 0, y ≥ 0}`. The identities `V(A ∪ B) = V(A) + V(B) − V(A ∩ B)`,
`V(A − B) = V(A) − V(A ∩ B)`, and `V(A ∩ B) + V(A − B) = V(A)` SHALL hold to machine
precision, independent of any mesh. This oracle SHALL be the PRIMARY multi-seam
correctness check (the HOST-ANALYTIC gate); when a multi-seam solid welds watertight at a
deflection, its MESH enclosed volume SHALL also match the corresponding closed-form value
within the doubled deflection band. The oracle SHALL depend on NO mesher and NO OCCT.

#### Scenario: The closed-form multi-seam identities close to machine precision (host, no OCCT)

- GIVEN `V(A)`, `V(B)`, and `V(A ∩ B)` (corner clip) evaluated by the exact closed-form oracle with NO OCCT
- WHEN the union / difference / partition identities are checked
- THEN `|V(A ∪ B) − (V(A) + V(B) − V(A ∩ B))| ≤ 1e-12`, `|V(A ∩ B) + V(A − B) − V(A)| ≤ 1e-12`, and `0 < V(A ∩ B) < V(A)` SHALL hold — FUSE, CUT, and COMMON are exactly related independent of any mesh

#### Scenario: A watertight multi-seam mesh matches its closed-form corner volume (host)

- GIVEN a deflection at which `freeformBooleanMultiSeam(A, B, FUSE)` welds watertight on a host build with NO OCCT
- WHEN the result is meshed and its enclosed volume compared to `V(A ∪ B)`
- THEN the mesh SHALL be watertight and its volume SHALL equal the closed-form union value within the doubled scale-relative deflection band — the mesh-level check confirms the closed-form gate

### Requirement: First multi-seam freeform FUSE/CUT/COMMON parity with OCCT BRepAlgoAPI (simulator gate, spatial)

When the first multi-seam freeform boolean assembles, it SHALL be verified on a booted
iOS simulator (OCCT linked) against the OCCT oracle: operand `A` and the corner box `B`
SHALL be built BOTH natively and as OCCT shapes, and the native
`freeformBooleanMultiSeam(A, B, op)` result SHALL match `BRepAlgoAPI_Fuse` / `_Cut` /
`_Common` (measured via `BRepGProp`) on ENCLOSED VOLUME and SURFACE AREA within a
scale-relative tolerance, SHALL be WATERTIGHT (closed shell), SHALL agree on TOPOLOGY
counts (faces / edges / vertices in the reachable envelope), and SHALL match on a
**spatial BBOX** (not volume-only); additionally a batch of query points SHALL agree with
`BRepClass3d_SolidClassifier` on the native result with ZERO crisp IN↔OUT disagreements.
The engine's mandatory watertight + volume self-verify SHALL DISCARD any native result
that is not watertight or whose volume does not match the OCCT oracle, falling through to
the OCCT boolean so a wrong/leaky solid is NEVER emitted. OCCT SHALL be referenced ONLY in
the simulator proof harness / `src/engine/occt`; the assembler and its inputs SHALL remain
OCCT-free. The change SHALL add no `cc_*` entry point, and the count of any deferred cases
SHALL be reported, not hidden.

#### Scenario: The native multi-seam FUSE matches BRepAlgoAPI_Fuse spatially (sim, parity)

- GIVEN operand `A` and the corner box `B` built BOTH natively and as OCCT shapes on a booted simulator with OCCT linked
- WHEN the native `freeformBooleanMultiSeam(A, B, FUSE)` produces a result and it is compared to `BRepAlgoAPI_Fuse` via `BRepGProp` (volume / area), topology counts, spatial BBOX, and `BRepClass3d_SolidClassifier` on a query-point batch
- THEN the native volume / area SHALL match the OCCT oracle within tolerance, the native result SHALL be watertight with agreeing topology counts and a matching spatial BBOX, and there SHALL be ZERO crisp IN↔OUT classification disagreements

#### Scenario: A non-watertight or wrong-volume multi-seam result is discarded to OCCT (sim)

- GIVEN a native multi-seam result deliberately perturbed to be non-watertight or wrong-volume on a booted simulator with OCCT linked
- WHEN the engine's mandatory watertight + volume self-verify is evaluated
- THEN the native result SHALL be DISCARDED and the boolean SHALL fall through to `BRepAlgoAPI_*` (OCCT), no wrong/leaky solid SHALL be emitted, and no tolerance SHALL be weakened to force the native path

### Requirement: The multi-seam freeform boolean is strictly additive and keeps the landed single-seam path byte-identical

Landing the multi-seam freeform boolean SHALL leave the consumed substrate byte-identical:
B1 `recogniseFreeformSolid` (`boolean/freeform_operand.h`), B2 `splitFace`
(`boolean/face_split.h`), B3 `classifyPointInMesh` (`boolean/freeform_membership.h`), the
M0 `SolidMesher` (`tessellate/solid_mesher.h`), the M1 `traceWallSeam` /
`ssi::trace_intersection` (`ssi/marching.h`), the analytic
`recogniseCurvedSolid`/`classifyPoint` (`ssi_boolean.h`), the landed single-operand
`freeformHalfSpaceCut` (`boolean/half_space_cut.h`), and the landed single-seam
two-operand path (`boolean/inter_solid_seam.h`, `boolean/two_operand.h`) SHALL be UNCHANGED
and consumed WITHOUT modification. The new code SHALL live in NEW headers
(`seam_graph.h`, `multi_seam.h`) and SHALL reuse the landed analytic-face-split,
`inter_solid_seam.h` primitives, and `assemble.h` weld primitives unchanged. The addition
SHALL introduce no `cc_*` ABI change and SHALL keep `src/native/**` free of OCCT includes.
Any M1 SSI touch SHALL be strictly ADDITIVE with every prior seeding/marching control
BYTE-FROZEN and proven inert on all existing callers before use. The existing
native-booleans and native-ssi suites SHALL pass with counts unchanged from the pre-change
baseline.

#### Scenario: The landed single-seam path and consumed subsystems are byte-identical after the multi-seam verb is added (host + sim)

- GIVEN the B1/B2/B3/M0/M1 headers, the analytic `recogniseCurvedSolid`/`classifyPoint`, the landed `freeformHalfSpaceCut`, and the landed single-seam `inter_solid_seam.h` / `two_operand.h` path, together with the native-booleans and native-ssi suites, before and after this change
- WHEN each is exercised at the same inputs and compared against the pre-change baseline
- THEN the consumed subsystems and the landed single-operand and single-seam paths SHALL be byte-identical (unchanged source, unchanged suite counts), there SHALL be zero OCCT includes under `src/native/**`, and no `cc_*` signature or POD layout SHALL have changed

#### Scenario: Any M1 SSI touch is additive with prior controls byte-frozen (host)

- GIVEN this change and any additive helper it adds to the M1 marcher for junction-aware arc trimming
- WHEN the M1 seeding/marching controls are exercised on every existing SSI caller before the helper is used
- THEN every prior seeding/marching output SHALL be byte-identical to the pre-change baseline (the helper is proven inert), and the addition SHALL be a new defaulted option that changes no existing caller

### Requirement: Honest decline with the next sharpened blocker is a first-class multi-seam outcome

The change SHALL treat an honest decline carrying the next sharpened blocker as a
first-class outcome: if the full two-cutting-face junction weld is not robustly reachable
this wave, the change SHALL land whatever piece is PROVABLE and return the NEXT sharpened
blocker honestly, choosing the sharpest reachable level: (1) full multi-seam FUSE + CUT +
COMMON self-verified watertight at the closed-form corner volumes and matching
`BRepAlgoAPI_*` in sim; else (2) a constrained multi-seam FUSE at the deflection(s) where
the junction and the two arcs weld coincident, the other deflections declining, naming the
valence-3 junction single-sampling weld as the deflection-independence enabler; else (3)
the seam-graph builder plus the junction-vertex computation and the junction-joined split
proven in ISOLATION (the two arcs join at `J` to `weldTol`, the bent seam splits the wall
into corner + L-survivor, every fragment classifies crisply), the weld declined with its
measured gap; else (4) a sharpened junction-computation decline (`J` convergence or
arc-clip coincidence) with the failing measurement. Every level SHALL emit either a
self-verified-correct solid OR a NULL `Shape` → OCCT; NO partial, overlapping, leaky, or
wrong-volume solid, NO guessed membership, NO seam-graph stub, and NO weakened tolerance
SHALL be shipped. The reached level and the measured next blocker (the general
`≥ 3`-cutting-face / multi-junction / branch-point graph) SHALL be recorded, not hidden.

#### Scenario: The provable subset lands and the weld is honestly declined with the next enabler (host)

- GIVEN a wave in which the full junction weld is not robustly reachable (the junction-vertex weld coincidence) but the seam-graph builder, the junction computation, and the junction-joined split are provable in isolation
- WHEN the change is landed
- THEN it SHALL land `buildSeamGraph` + the junction-vertex computation + the junction-joined B2 split proven by isolated host tests (the two arcs join at `J`, the wall splits into corner + L-survivor, every fragment classifies crisply), SHALL DECLINE the weld with the measured gap, SHALL name the valence-3 junction single-sampling weld and the general `≥ 3`-seam / branch-point graph as the next enablers, and SHALL ship NO seam-graph stub and NO weakened tolerance — a first-class honest outcome

#### Scenario: No multi-seam path emits a leak at any deflection (host, no OCCT)

- GIVEN operand `A` and the corner box `B`, swept over a range of deflections for FUSE / CUT / COMMON on a host build with NO OCCT
- WHEN each `freeformBooleanMultiSeam` result is meshed and audited
- THEN EVERY result SHALL be `isNull()` OR watertight (a non-watertight or wrong-volume solid SHALL NEVER be returned), and the deflections at which each op declines SHALL be attributable to the junction-vertex weld coincidence — the honest-decline discipline holds at 100% of the sweep
