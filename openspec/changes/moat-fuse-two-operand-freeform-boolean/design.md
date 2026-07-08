# Design — moat-fuse-two-operand-freeform-boolean (MOAT M2-FUSE)

## 0. What the substrate already supplies (verified in source)

- **B1 `recogniseFreeformSolid`** (`boolean/freeform_operand.h`) admits ONE
  bowl-lidded convex-quad prism: `FreeformOperand{ solid, faces[], freeform[],
  analytic[], bbox, watertight }`, each `OperandFace` world-placed with resolved
  outward normal. CONSUMED unchanged for operand `A`.
- **The all-planar box `B`** is a native `topology::Shape` whose every face is a
  `Plane`. It is already recognised by the planar-solid path (`native_boolean.h` /
  `polygon.h` / `split_plane.h`) that computes plane-faced fuse/cut/common. CONSUMED
  unchanged for operand `B`; no new admission gate.
- **M1 `ssi::trace_intersection(A, B)`** (`ssi/marching.h`) traces a surface∩surface
  seam as `WLine`s with `TraceStatus` (`Closed` / `BoundaryExit` are well-formed).
  `half_space_cut.h::traceWallSeam` already drives it for a Bézier-wall ∩ plane seam
  over a param box derived from the operand AABB. CONSUMED unchanged.
- **B2 `splitFace(face, WLine seam)`** (`boolean/face_split.h`) splits ONE trimmed
  freeform face along ONE seam into two tiling sub-faces (`SplitResult`). CONSUMED
  unchanged for `A`'s Bézier wall.
- **B4 analytic-face split + section cap** (`half_space_cut.h`, `hscdetail`) splits a
  PLANAR face along its `Face ∩ plane` line into keep/discard sub-faces with exactly
  two recorded crossings, and welds via `assemble.h` `VertexPool`. Its per-face split
  primitive is REUSED for the plane∩plane straight seams; its single-cap synthesis is
  NOT (a two-operand FUSE has no single planar cap).
- **B3 `classifyPointInMesh(mesh, bbox, p)`** (`boolean/freeform_membership.h`)
  ray-casts a point against an M0 boundary mesh → `In` / `Out` / `On` / `Unknown`.
  CONSUMED unchanged for fragment centroids.
- **M0 `SolidMesher::mesh`** (`tessellate/solid_mesher.h`) meshes a shape watertight;
  `isWatertight` audits it. CONSUMED unchanged for the pre-cut operand meshes (B3
  input) and the final self-verify.

## 1. The configuration that collapses multi-seam SSI to one reused seam

The general FUSE blocker is "several plane∩(bowl+walls) seams assembled into a
consistent intersection graph." We pick the SIMPLEST pose that makes the graph
tractable WITHOUT new curved SSI:

- `A` = the bowl-lidded convex-quad prism (Bézier bowl top + four planar quad walls +
  planar bottom), the same operand the landed CUT/COMMON already admit.
- `B` = a finite **axis-aligned box** placed so that exactly ONE of its six planar
  faces (call it `Pcut`) slices through `A`'s Bézier bowl wall in a **single clean
  transversal seam**, and `B`'s other faces cross only `A`'s PLANAR faces.

Then the inter-solid intersection boundary is:

- ONE **curved seam** `S_curve = Pcut ∩ (A's Bézier wall)` — traced by the EXISTING
  `traceWallSeam` machinery (the plane is `Pcut`'s supporting plane; the param box is
  the union of the two operands' overlap AABB). This is byte-for-byte the seam the
  landed CUT already produces.
- Several **straight seams** `S_lin,i = (a B face) ∩ (an A planar face)` — each a
  plane∩plane line clipped to both face polygons, computed analytically by the SAME
  clip the planar boolean already uses. No curved SSI.

`S_curve` and the `S_lin,i` splice end-to-end into ONE closed intersection loop that
bounds the overlap region `A ∩ B`. That closed loop is exactly what a two-operand
boolean needs, and it is reachable with the landed verbs.

## 2. Inter-solid seam-set builder (`inter_solid_seam.h`)

`buildInterSolidSeam(FreeformOperand A, PlanarSolid B) → InterSolidSeam | Decline`:

1. Compute the overlap AABB `A.bbox ∩ B.bbox`; DECLINE `NoOverlap` if empty.
2. Identify `Pcut` = the unique `B` face whose supporting plane crosses `A`'s Bézier
   wall (its plane separates the wall's pole hull). DECLINE `NotSingleCurvedCut` if
   zero or more than one `B` face crosses the freeform wall — that keeps this slice
   to ONE curved seam (the honest scope boundary).
3. Trace `S_curve = traceWallSeam(A, wall.fs, Pcut.plane)`; require
   `points.size() ≥ 2` and status `Closed`/`BoundaryExit`, else DECLINE `SeamUnusable`.
4. For each `(B face, A planar face)` pair whose polygons overlap, compute the
   clipped plane∩plane segment `S_lin,i`; collect them.
5. Splice `S_curve` + `{S_lin,i}` into ONE ordered loop; verify it is SIMPLE (no
   non-adjacent-edge intersection) and CLOSED. DECLINE `SeamNotClosed` otherwise.

The builder returns typed crossings `(faceId, edgeParam)` per split so the split
driver replays them; it NEVER emits a partial seam.

## 3. Two-operand split of BOTH operands (`two_operand.h` split step)

- `A`'s Bézier wall → B2 `splitFace(wall.face, S_curve)` UNCHANGED → two sub-faces.
- Each crossed PLANAR face of `A` and of `B` → the landed analytic-face-split
  primitive along its `S_lin,i` line → keep/discard sub-faces, exactly-two-crossings
  verified. A tangent, at-vertex, or wrong-count crossing DECLINES `SplitDecline`.
- Uncrossed faces pass through whole.

The seam edges are built ONCE and shared bit-exactly between the two faces that meet
along them (`A`'s Bézier sub-face ↔ `B`'s `Pcut` sub-face for `S_curve`; the two
planar sub-faces for each `S_lin,i`), the weld contract.

## 4. Two-operand shell classifier (classify step)

Mesh `A` and `B` (pre-split) with M0 once each → `meshA`, `meshB`. For every fragment:

- fragment of `A` → `classifyPointInMesh(meshB, B.bbox, centroid)` → `A`-in / `A`-out
- fragment of `B` → `classifyPointInMesh(meshA, A.bbox, centroid)` → `B`-in / `B`-out

An `On`/`Unknown` verdict, or a fragment centroid that lands in the ON-band, DECLINES
`ClassifyAmbiguous` — membership is NEVER guessed.

## 5. Per-op weld (weld step) and the survivor sets

| op     | survivors                                        | seam orientation |
|--------|--------------------------------------------------|------------------|
| FUSE   | `A`-out ∪ `B`-out                                | outward          |
| CUT    | `A`-out ∪ (`B`-in **reversed**)                  | `B` faces flipped inward-facing become the cavity wall |
| COMMON | `A`-in ∪ `B`-in                                  | outward          |

Weld the survivor faces into one shell via `assemble.h` `VertexPool` (coincident
corners → shared vertices; the shared seam edges already coincide bit-exactly). A
shell that cannot close DECLINES `WeldOpen`.

## 6. Mandatory self-verify → OCCT fallback (the load-bearing gate)

The welded result is admitted ONLY if BOTH hold:

1. **Watertight** — `isWatertight(SolidMesher::mesh(result))`: every edge shared by
   exactly two faces.
2. **Independent closed-form volume** — the enclosed volume matches, within a
   scale-relative deflection band, the mesh-free closed-form value for the op:
   `V(A ∪ B) = V(A) + V(B) − V(A ∩ B)` for FUSE (and the corresponding
   `V(A) − V(A ∩ B)` for CUT, `V(A ∩ B)` for COMMON), where `V(A)` uses the landed
   exact per-triangle quadratic-moment bowl oracle, `V(B)` is the box volume, and
   `V(A ∩ B)` is the closed-form clip of the bowl integrand over the box footprint.
   This is the HOST-ANALYTIC gate — NO OCCT.

A result that fails EITHER is DISCARDED → NULL `Shape` → OCCT `BRepAlgoAPI_Fuse`.
No tolerance is weakened; a non-watertight or wrong-volume solid is NEVER emitted.

## 7. SIM native-vs-OCCT parity gate (spatial, not volume-only)

On a booted iOS simulator with OCCT linked, `A` and `B` are built BOTH natively and
as OCCT shapes; the native `freeformBooleanTwoOperand(A, B, FUSE)` is compared to
`BRepAlgoAPI_Fuse` on: enclosed VOLUME and surface AREA (`BRepGProp`); WATERTIGHTNESS
(closed shell); TOPOLOGY counts (faces / edges / vertices in the reachable envelope);
and a **spatial BBOX** match (not volume-only) plus a query-point batch agreeing with
`BRepClass3d_SolidClassifier` with ZERO crisp IN↔OUT disagreements. OCCT appears ONLY
in the proof harness / `src/engine/occt`.

## 8. The M1 additive extension (only if required)

The single reused curved seam needs NO new SSI — `traceWallSeam` already covers it.
IF full-seam coverage over the two-operand param box needs a helper (e.g. a
param-box union so the marcher does not exit the box early), it is added ADDITIVELY
to `ssi/marching.h` behind a new option field defaulted to the current behaviour, so
every prior seeding/marching control and every existing SSI parity figure
(`native-ssi` suites) stays byte-identical. Prior controls are byte-frozen; the
addition is proven inert on all existing callers before it is used.

## 9. Honest-decline hierarchy (a first-class outcome)

If the full FUSE weld is not robustly reachable this wave, land the provable subset
and DECLINE the rest with the measured next blocker, in this order of preference:

1. **Full FUSE** (best) — the assembled `A ∪ B` self-verifies watertight at the
   closed-form volume and matches `BRepAlgoAPI_Fuse` in sim.
2. **Constrained FUSE** — FUSE at the specific deflection(s) where the shared curved
   seam welds coincident; the other deflections DECLINE (the landed CUT/COMMON
   fragility, now two-operand). Land it, name the M0 shared-curved-edge
   single-sampling fix as the enabler for deflection-independence.
3. **Seam + split/classify only** — land `buildInterSolidSeam` + the two-operand
   split and classify verbs proven in ISOLATION (host analytic: the seam loop closes,
   both operands split, every fragment classifies crisply), and DECLINE the weld with
   the measured gap.
4. **Sharpened decline** — if even the seam-set does not close robustly for the box
   pose, return the seam-graph-assembly blocker with the failing measurement.

Every level emits either a self-verified-correct solid or a NULL `Shape` → OCCT.
No partial, overlapping, leaky, or wrong-volume solid; no guessed membership; no FUSE
stub; no weakened tolerance.

## 10. Alternatives considered

- **General multi-curved-seam SSI graph** — the correct end state, but multi-person-
  year; deferred. This slice picks the single-curved-seam box pose precisely to avoid
  it.
- **Reuse the single-cap B4 synthesis for FUSE** — rejected: FUSE has no single planar
  section cap; it welds survivor faces from BOTH operands. The B4 per-face split
  primitive is reused; its cap synthesis is not.
- **Volume-only sim parity** — rejected by the discipline: parity is spatial (BBOX +
  point classification + topology), so a coincidentally-equal-volume wrong shape cannot
  pass.
- **Ship a defer-only FUSE stub** — forbidden. An honest decline carrying the next
  blocker is the floor; a stub is not.
