# Design — moat-m2asm-first-freeform-boolean

## 1. Diagnosis: the shortest path to ONE verified freeform boolean

B1's assembly decline pinned two blockers. This section shows which is reducible.

### D1.1 Blocker (i) is sidesteppable — pick the operand B2 already splits

B2 `splitFace` requires the freeform face to have a **convex, straight-edged outer
loop** cut by **one seam chord that enters through one boundary edge and exits through
another** (`SplitDecline::NoOuterLoop` / `CrossingsNot2` otherwise). The
bump-capped-cylinder keystone from B1 fails because its freeform wall wraps 360°, so
its trim is a smooth CLOSED loop — no open outer chain to cross twice.

But B2 already SHIPS a passing fixture that is exactly the shape we need
(`tests/native/face_split_fixture.h`): a degree-2 Bézier **bowl** patch
`z = a·((u−½)² + (v−½)²)`, with `x = u−½`, `y = v−½`, genuinely TRIMMED by a **convex
quadrilateral** whose four edges carry Line pcurves and exact degree-2 Bézier 3-D
curves, cut by the plane `x = 0` into a clean chord `u ≡ ½` (entry on the bottom quad
edge, exit on the top quad edge). B1 admits this patch as `Freeform` (its trim is not
the full parametric rectangle → `TrimStatus::Genuine`).

**Sidestep operand — the bowl-lidded convex-quad prism.** Wrap that patch into a
watertight solid `A`:

- **Top (freeform):** the bowl Bézier patch, trimmed by the convex quad `Q`.
- **Four side walls (analytic, Plane):** over each quad edge `e_k`, a vertical planar
  wall bounded by the shared top Bézier edge (from the bowl), two vertical Line edges,
  and a straight bottom Line edge. Each wall is genuinely PLANAR: a bowl edge over a
  straight UV segment has `(x(t), y(t))` linear and `z(t)` quadratic, so the 3-D curve
  lies in the **vertical plane** through that `(x,y)` segment; the wall's other three
  edges lie in that same vertical plane.
- **Bottom (analytic, Plane):** the flat quad `Q` at `z = 0`.

Every boundary edge is shared by exactly two faces (each quad edge is shared by the
bowl and one wall; each vertical edge by two adjacent walls; each bottom edge by the
bottom and one wall), so B1's edge-incidence audit passes and `recogniseFreeformSolid`
admits `A` with ONE freeform face (the bowl) whose outer loop is the convex quad B2
splits. **Blocker (i) is sidestepped; the B2 smooth-trim generalisation is deferred.**

### D1.2 Blocker (ii) is irreducible — B4 is the enabler

A half-space cut by plane `P` of ANY solid crosses that solid's OTHER faces and leaves
an open section that must be capped. Concretely, cutting `A` with `P: x = 0` (keep
`x ≤ 0`):

- The **freeform top** is split by B2 into `faceIn` (`x ≥ 0`) and `faceOut` (`x ≤ 0`);
  keep `faceOut`.
- `P` crosses the two side walls over quad edges `e0` (`u`: .15→.85) and `e2`
  (`u`: .80→.20) — both straddle `u = ½` — and the **bottom** quad face. Each must be
  split along its `Face ∩ P` line into keep (`x ≤ 0`) and discard (`x ≥ 0`) halves.
- The walls over `e1` (`u ≈ .8`) and `e3` (`u ≈ .17`) do NOT straddle `u = ½`: `e1` is
  discarded whole, `e3` is kept whole.
- A NEW **cross-section cap** on `P` closes the solid, bounded by the seam parabola
  `z = a·y²` (the `wall ∩ P` curve, from B2) on top, the two vertical crossing segments
  on the split `e0`/`e2` walls, and the bottom crossing segment.

Neither an analytic-face splitter nor a section-cap synthesiser is a landed verb.
**B4 is the single irreducible enabler for the first freeform boolean.**

### D1.3 Why CUT, not COMMON/FUSE, and why one plane, not a box

CUT by a single planar half-space is the minimum that still exercises the full arc
(recognise→trace→split→cap→classify→weld→verify). COMMON/FUSE and a box (multi-plane)
cutter each add cross-section caps on several planes and inter-cap welds — deferred.
The closed-form volume of the single-plane CUT is a polynomial (D4), giving a clean
host-analytic oracle.

## 2. The B4 verb — `half_space_cut.h`

`std::optional<topo::Shape> halfSpaceCut(const FreeformOperand& op, const math::Plane&
P, KeepSide side, const ssi::WLine& seam, HalfSpaceCutDecline* why)`, returning the
welded keep-side `Solid`, or `nullopt` with a measured blocker.

### D2.1 Analytic-face split (per analytic face)

For each `OperandFace` of role `AnalyticHalfSpace`:

- Flatten its outer loop (the `tess::buildRegion` / pcurve path B2 uses) and evaluate
  the signed distance of each boundary vertex to `P` (`d = n_P · (v − o_P)`).
- If all `d` are on the keep side (beyond a scale-relative band) → keep the face whole.
- If all on the discard side → drop it.
- If the loop crosses `P`: find the EXACTLY TWO boundary edges whose endpoints straddle
  `P`, interpolate the crossing point on each (edge id + curve parameter recorded),
  split the loop into the keep sub-loop and discard sub-loop by the chord between the
  two crossings, and rebuild the keep sub-face over the parent's `Plane` surface with
  the parent's per-edge Line pcurves carried verbatim (the `face_split.h restrictEdge`
  discipline). A crossing that is tangent, at a vertex, or yields ≠ 2 crossings →
  DECLINE (`AnalyticCrossingNot2` / `CrossingAtVertex`).

The two crossing points per split face are the section cap's straight-segment corners;
they are welded (via the shared `VertexPool`) with the neighbour wall's crossing point
so the cap and the walls share vertices.

### D2.2 Cross-section cap synthesis on `P`

- Project every crossing point and every seam-chord node onto `P`'s 2-D frame
  (`u = x_P · (w − o_P)`, `v = y_P · (w − o_P)`).
- Assemble ONE ordered closed loop: the B2 seam chord (endpoints = the freeform wall's
  two crossings, which coincide with the split-wall crossings on `e0`/`e2`) spliced to
  the straight crossing segments of the split analytic faces, walked so the loop is
  simple and closed. Verify simplicity (no self-intersection, `segmentsCross` on
  non-adjacent edges) and closure (last node == first node within weld tolerance) —
  else DECLINE (`SectionLoopNotSimple` / `SectionLoopOpen`).
- Build the cap `Face` over a `Plane` surface whose frame `z = ±n_P` (oriented so the
  cap normal faces the DISCARD side — outward for the keep-side solid). The seam edge
  is built ONCE (degree-1..2 pcurve on `P` through the seam nodes; 3-D curve = the seam
  points, identical to B2's seam edge) and SHARED with the kept freeform sub-face so
  the cap↔top weld is bit-exact; the straight cap edges are Line edges SHARED with the
  split walls' crossing edges.

### D2.3 Weld → Solid

Collect: kept `faceOut` (freeform), kept analytic sub-faces + whole faces, the section
cap. Weld coincident corners to shared vertices with the `assemble.h VertexPool` (cell
size `kWeldTol`), wrap the faces in a Shell → Solid. Fewer than four faces, or a shell
that does not close, → DECLINE (`WeldOpen`).

## 3. Composition — the first-CUT assembler

`std::optional<topo::Shape> freeformHalfSpaceCut(const topo::Shape& operand, const
math::Plane& P, KeepSide side)`:

1. `op = recogniseFreeformSolid(operand)` — else NULL (B1 decline).
2. Require exactly ONE freeform face (this slice); build its `SurfaceAdapter` and the
   plane `SurfaceAdapter`, and `seam = ` the single M1 `WLine` from
   `ssi::trace_intersection(wall, P)` with `points.size() ≥ 2` and status
   `Closed`/`BoundaryExit`; else NULL (M1 decline).
3. `SplitResult sr = splitFace(wall, seam)` — else NULL (B2 `SplitDecline`).
4. `cut = halfSpaceCut(op, P, side, sr.split->seam)` — else NULL (B4 decline).
5. **B3 confirmation.** Mesh the PRE-cut operand with M0; for the kept freeform
   sub-face's interior centroid and each kept analytic sub-face's interior centroid,
   `classifyPointInMesh` MUST be `In` (interior to the operand) and their side of `P`
   MUST be the keep side; any `On`/`Unknown`, or a survivor on the wrong side, → NULL.
   (B3 confirms the selection rather than re-deriving it — the split geometry already
   knows the side; B3 is the independent cross-check that no fragment was mis-kept.)
6. **Self-verify (mandatory).** Mesh `cut` with M0; require `isWatertight` and enclosed
   volume within a scale-relative tolerance of the closed-form value (D4). Fail →
   DISCARD (NULL → OCCT).

## 4. Host-analytic volume oracle (no OCCT)

The operand `A = {(x,y,z): (x,y) ∈ Q, 0 ≤ z ≤ h0 + a(x²+y²)}` (base height `h0 > 0`
so the lid clears the base). The CUT keeps `x ≤ 0`:
`V = ∫∫_{Q ∩ {x ≤ 0}} (h0 + a·(x² + y²)) dA`. `Q ∩ {x ≤ 0}` is a convex polygon; the
integrand is a degree-2 polynomial, so `V` is evaluated in CLOSED FORM by triangulating
the clipped region and summing the exact per-triangle polynomial moments
(`∫∫ 1`, `∫∫ x²`, `∫∫ y²`). This is the host gate's oracle — independent of the mesher
and of OCCT.

## 5. Honesty, tolerances, and the fall-through discipline

- Every stage returns NULL on decline; the assembler NEVER emits a partial, leaky, or
  wrong-volume solid, and NEVER guesses a fragment's side (a B3 `On`/`Unknown` is a
  decline, not a coin-flip).
- No tolerance is weakened to pass: the self-verify volume tolerance is scale-relative
  and sized to the M0 deflection (curved bowl top), NOT tuned to admit a wrong result;
  the section-loop simplicity/closure tests are bit/geometry predicates, not fudge.
- The B2 smooth-trim generalisation (blocker i) is DEFERRED, not stubbed: no
  closed/circular-wall path is written; the bowl-lidded-prism operand keeps this change
  inside B2's shipped envelope. If B4 lands but the composition does not robustly
  assemble this wave, B4 is proven in isolation (its own host tests) and the assembly
  HONEST-DECLINES with the next sharpened blocker.

## 6. Additivity, complexity, gates

- `half_space_cut.h` is a NEW header; B1/B2/B3/M0/M1 and the analytic
  `recogniseCurvedSolid`/`classifyPoint` are consumed byte-identical. No `cc_*` change;
  0 OCCT includes under `src/native/**`.
- Cognitive complexity: the B4 driver delegates per-face split, section-loop assembly,
  and weld to free-function helpers (the `assemble.h`/`face_split.h` pattern), keeping
  each function within the backend band (≤ 15).
- **HOST ANALYTIC gate** (no OCCT): §4 oracle + watertight audit on the bowl-lidded
  prism CUT; the decline battery (plane misses / tangent crossing / non-simple section
  loop / open weld) returns NULL. **SIM native-vs-OCCT gate**: match `BRepAlgoAPI_Cut`
  on volume/area/watertightness/topology + `BRepClass3d_SolidClassifier` point-batch
  agreement, extending the existing curved-boolean sim harness.
