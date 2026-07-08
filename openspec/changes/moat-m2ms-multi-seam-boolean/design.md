# Design — moat-m2ms-multi-seam-boolean (MOAT M2-multiseam)

## 0. What the substrate already supplies (verified in source)

- **The landed single-seam path** (`boolean/inter_solid_seam.h` +
  `two_operand.h`) provides `buildInterSolidSeam(A, B)` and
  `freeformBooleanTwoOperand(A, B, op)` for the pose with EXACTLY ONE cutting face and
  full containment. CONSUMED byte-identical. Its `isdetail::` primitives —
  `wallWorldPoles`, `planeStraddlesWall`, `aabbInsidePlane`, `tracePlaneOf`,
  `splitOperandA` — are reused; the seam-graph builder generalises the caller
  (`findCuttingFace` `nCut == 1` → a cutting-face SET), not the primitives.
- **B1 `recogniseFreeformSolid`** (`boolean/freeform_operand.h`) admits `A` as a
  `FreeformOperand{ solid, faces[], freeform[], analytic[], bbox, watertight }` with one
  freeform bowl face and world-placed outward normals. CONSUMED unchanged.
- **The all-planar box `B`** is admitted by the existing planar-solid path
  (`isAllPlanar` + `extractPolygons`); the corner box is six single-quad `Plane` faces.
  CONSUMED unchanged; no new admission gate.
- **M1 `traceWallSeam`** (`half_space_cut.h`) drives `ssi::trace_intersection` for one
  Bézier-wall ∩ plane chord over a param box from the operand AABB. CONSUMED unchanged,
  called ONCE PER cutting plane.
- **B2 `splitFace(face, WLine)`** (`boolean/face_split.h`) splits ONE trimmed freeform
  face along ONE boundary-to-boundary seam polyline into two tiling sub-faces. CONSUMED
  unchanged, called ONCE on the junction-joined bent seam.
- **B4 analytic-face split + `hscdetail::` primitives** (`half_space_cut.h`):
  `cutAnalyticFace`, `orderLoop`, `edgeFromPiece`, `seamChord3d`, `signedDist`,
  `onKeepSide`, `Piece`, `planarFaceFromLoop`. REUSED unchanged for the straight chords
  and per-arc loop assembly.
- **B3 `classifyPointInMesh`** (`boolean/freeform_membership.h`) and **M0
  `SolidMesher`/`isWatertight`** (`tessellate/solid_mesher.h`). CONSUMED unchanged for
  fragment membership and the pre-cut meshes + final self-verify.
- **`assemble.h` `VertexPool` + `triangulatePolygonToFaces`** weld coincident corners to
  shared vertices. REUSED unchanged; the junction `J` is welded as ONE shared vertex.

## 1. The configuration that makes the seam graph minimal but genuine

The general multi-seam blocker is "several curved seams assembled into a consistent
intersection GRAPH with junction vertices." We pick the SMALLEST pose that forces a
real junction WITHOUT any new curved SSI:

- `A` = the bowl-lidded convex-quad prism (the landed operand; `V(A) = 0.196310`).
- `B` = a finite axis-aligned box straddling ONE corner of `A`: `x ∈ [0, 0.8]`,
  `y ∈ [0, 0.6]`, `z ∈ [−0.6, 0.2]` (`V(B) = 0.384000`), so **TWO** of its faces —
  `x = 0` and `y = 0` — slice `A`'s Bézier wall, their supporting planes share the box
  vertical edge `x = 0, y = 0`, and the other four faces contain `A`.

Then the inter-solid intersection boundary on the wall is:

- TWO **curved arcs** `arcA = {x = 0} ∩ wall` and `arcB = {y = 0} ∩ wall`, each traced
  by the EXISTING `traceWallSeam` (byte-unchanged), each a portion of the full chord.
- ONE **junction vertex** `J = (0, 0, 0)` where the box's vertical edge pierces the
  wall — the shared endpoint of `arcA` and `arcB`.
- Straight chords `S_lin,i` where the two cutting planes cross `A`'s planar walls/bottom
  and each other (the box's own edge), computed by the SAME `cutAnalyticFace` clip.

`B` removes exactly the quadrant `A ∩ {x ≥ 0, y ≥ 0}` (`V = 0.051275`). So, closed-form
and OCCT-free (DIAGNOSED):

- `V(A ∩ B) = ∫∫_{Q ∩ {x ≥ 0, y ≥ 0}} (H0 + a(x² + y²)) dA = 0.051275`
- `V(A − B) = V(A) − V(A ∩ B) = 0.145035`  (the L-shaped bowl-topped survivor)
- `V(A ∪ B) = V(B) + V(A − B) = 0.529035`

Crucially the removed region is a re-entrant CORNER, so `A − B` is **L-shaped** and
does NOT reduce to a half-space of `A` — the single-seam CUT/COMMON theorem no longer
applies. This is what makes it a genuine multi-seam slice rather than a re-pose.

## 2. Seam-graph builder (`seam_graph.h`)

`buildSeamGraph(FreeformOperand A, PlanarSolid B) → SeamGraph | Decline`:

1. `bPolys = extractPolygons(B)`; DECLINE `NotPlanarB` if empty, `NoOverlap` if the
   AABBs miss.
2. **Cutting-face SET.** Reusing `planeStraddlesWall` on the wall poles, collect every
   `B` face that straddles the wall. This slice REQUIRES exactly TWO
   (`cutIdx = {i, j}`); DECLINE `NotTwoCuttingFaces` on `≠ 2`. Reusing `aabbInsidePlane`,
   require every OTHER `B` face contains `A` (DECLINE `NotContained`).
3. **Shared box edge + junction.** The two cutting planes `Pi`, `Pj` must share a `B`
   edge (adjacent box faces); their intersection LINE is the box vertical edge. Compute
   its piercing point `J` on the wall by solving `signedDist(Pi, ·) = signedDist(Pj, ·) = 0`
   on the Bézier surface (a 2-D Newton/bisection on the wall param, seeded from the
   pole-hull straddle) — DECLINE `JunctionUnusable` if it does not converge inside the
   trimmed wall.
4. **Per-arc trace.** For each `Pk`, `arck = traceWallSeam(A, wall.fs, tracePlaneOf(Pk))`
   (UNCHANGED); require `points.size() ≥ 2` and `Closed`/`BoundaryExit`, else
   `SeamUnusable`. CLIP each full chord at `J` (drop the portion on the far side of the
   OTHER cutting plane), keeping the `boundary→J` sub-arc.
5. **Junction join.** Concatenate the clipped `arcA (boundary→J)` and `arcB (J→boundary)`
   into ONE `WLine` bent at `J` (`J` shared bit-exactly). Verify the join is coincident
   at `J` within `weldTol`, else `JunctionNotJoined`.
6. Return `SeamGraph{ bPolys, cutIdx, tracePlanes, junction J, jointSeam, per-plane
   straight chords }`; NEVER emit a partial graph.

Per-arc / per-junction helpers keep each function within the backend cognitive band.

## 3. Junction-joined freeform split (`multi_seam.h` split step)

`arcA(boundary→J)` bent to `arcB(J→boundary)` is ONE boundary-to-boundary polyline with
`J` an interior vertex, so B2 `splitFace(wall.face, jointSeam)` UNCHANGED splits the wall
in a SINGLE call into (a) the corner sub-face (over `Q ∩ {x ≥ 0, y ≥ 0}`) and (b) the
L-shaped survivor sub-face. The keep side is picked by the sub-face trim centroid (the
landed `splitOperandA` logic). Each planar `A`/`B` face crossed by a cutting plane is
split by `cutAnalyticFace` UNCHANGED. Every seam edge (each arc segment and each straight
chord) is built ONCE via `edgeFromPiece` and shared bit-exactly between its two incident
sub-faces — the weld contract. `J` is built ONCE as a shared vertex.

## 4. Seam-graph shell classifier (classify step)

Mesh `A` and `B` (pre-split) with M0 once each → `meshA`, `meshB`. Confirm (never guess)
via B3 `classifyPointInMesh` at each fragment centroid: the L-survivor wall sub-face and
`A`'s outer planar keeps are OUTSIDE `B`; `B`'s non-cutting faces are OUTSIDE `A`. An
`On`/`Unknown`/wrong-side verdict DECLINES `ClassifyAmbiguous`. `faceCentroid3d` from the
landed `two_operand.h` is reused.

## 5. Per-op seam-graph weld (weld step) and the survivor sets

| op     | survivors                                                         |
|--------|------------------------------------------------------------------|
| FUSE   | `A`-L-survivor (no cap) ∪ `B` non-cutting WHOLE ∪ two corner-notched cutting sub-faces |
| CUT    | `A`-L-survivor ∪ (the two cutting sub-faces reversed, as the corner cavity walls)      |
| COMMON | `A`-corner sub-face ∪ the two cutting sub-faces (the corner solid)                     |

Each cutting face `Pk` contributes a sub-face that is its box rectangle minus the corner
notch bounded by `arck` and the straight chords — the single-seam annulus generalised so
its hole boundary is `arck` PLUS the junction edge to `J`. The two cutting sub-faces and
the wall corner sub-face all reference `J` as a valence-3 shared vertex, welded by
`assemble.h` `VertexPool`. A shell that cannot close DECLINES `WeldOpen`. Unlike the
single-seam slice, CUT/COMMON do NOT delegate to `freeformHalfSpaceCut` — the L-shape /
corner is not a half-space of `A`.

## 6. Mandatory self-verify → OCCT fallback (the load-bearing gate)

Admit the welded result ONLY if BOTH hold:

1. **Watertight** — `isWatertight(SolidMesher::mesh(result))`: every edge shared by
   exactly two triangles (the junction `J` is the sharpest test — three sub-faces meet
   there and their edges to `J` must coincide bit-exactly).
2. **Independent closed-form volume** — the enclosed volume matches, within a
   scale-relative deflection band, the mesh-free closed-form value: `V(A ∪ B) = 0.529035`
   (FUSE), `V(A − B) = 0.145035` (CUT), `V(A ∩ B) = 0.051275` (COMMON), computed by the
   landed exact per-triangle quadratic-moment bowl integrand clipped to the corner
   footprint. This is the HOST-ANALYTIC gate — NO OCCT.

A result that fails EITHER is DISCARDED → NULL `Shape` → OCCT `BRepAlgoAPI_*`. No
tolerance weakened; a non-watertight or wrong-volume solid is NEVER emitted.

## 7. SIM native-vs-OCCT parity gate (spatial, not volume-only)

On a booted simulator (OCCT linked), `A` and the corner box `B` are built BOTH natively
and as OCCT shapes; `freeformBooleanMultiSeam(A, B, FUSE)` is compared to
`BRepAlgoAPI_Fuse` on enclosed VOLUME and AREA (`BRepGProp`), WATERTIGHTNESS, TOPOLOGY
counts, a **spatial BBOX** match, and a query-point batch agreeing with
`BRepClass3d_SolidClassifier` with ZERO crisp IN↔OUT disagreements — likewise CUT/COMMON
vs `BRepAlgoAPI_Cut`/`_Common`. OCCT appears ONLY in the harness / `src/engine/occt`.

## 8. The M1 additive extension (only if required)

Per-arc `traceWallSeam` covers each chord with NO new SSI; junction clipping is a
host-side param clip on the returned `WLine`, not an SSI change. IF junction-aware
trimming benefits from a marcher stop-plane, it is added ADDITIVELY behind a new option
field defaulted to current behaviour, every prior seeding/marching control byte-frozen,
proven inert on all existing callers before use.

## 9. Honest-decline hierarchy (a first-class outcome)

If the full seam-graph weld is not robustly reachable this wave, land the provable
subset and DECLINE the rest with the measured next blocker, in this order:

1. **Full multi-seam FUSE + CUT + COMMON** (best) — assembled `A ⊙ B` self-verifies
   watertight at the closed-form corner volume and matches `BRepAlgoAPI_*` in sim.
2. **Constrained multi-seam FUSE** — at the deflection(s) where the junction `J` and the
   two arcs weld coincident; other deflections DECLINE, naming the junction-vertex
   single-sampling weld as the enabler for deflection-independence.
3. **Seam-graph + junction + split only** — land `buildSeamGraph` (two arcs, junction
   `J` computed, arcs clipped and joined) and the junction-joined B2 split proven in
   ISOLATION (the corner loop closes, `J` is consistent to `weldTol`, the bent seam
   splits the wall into corner + L-survivor), and DECLINE the weld with the measured gap.
4. **Sharpened decline** — if even the two-arc graph does not close robustly, return the
   junction-computation blocker (`J` convergence / arc-clip coincidence) with the failing
   measurement.

Every level emits either a self-verified-correct solid or a NULL `Shape` → OCCT. No
partial, overlapping, leaky, or wrong-volume solid; no guessed membership; no seam-graph
stub; no weakened tolerance.

## 10. Alternatives considered

- **General `≥ 3`-cutting-face / multi-junction / branch-point SSI graph** — the correct
  end state, but multi-person-year; deferred. This slice picks the two-face, one-junction
  corner pose precisely to force ONE junction while keeping the graph a single bent chain
  that landed B2 splits in one call.
- **Split the wall by each arc separately (two B2 calls)** — rejected as unnecessary: the
  junction-joined bent seam is a single valid boundary-to-boundary B2 seam, so ONE
  `splitFace` yields corner + L-survivor with `J` as a shared interior polyline vertex,
  keeping B2 byte-identical and avoiding a second split's re-trim ambiguity.
- **Reduce CUT/COMMON to `freeformHalfSpaceCut`** (as the single-seam slice did) —
  IMPOSSIBLE here: the removed corner is not a half-space of `A`; CUT is L-shaped. CUT and
  COMMON MUST be genuine seam-graph welds.
- **Volume-only sim parity** — rejected by the discipline: parity is spatial (BBOX + point
  classification + topology), so a coincidentally-equal-volume wrong shape cannot pass.
- **Ship a defer-only seam-graph stub** — forbidden. An honest decline carrying the next
  blocker is the floor; a stub is not.
