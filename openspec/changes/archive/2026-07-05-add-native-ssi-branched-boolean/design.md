# Design — add-native-ssi-branched-boolean (SSI Stage S5-d)

## Context

S5-a/b/c (archived) shipped the SSI-curve-driven split→classify→select→weld pipeline in
`src/native/boolean/ssi_boolean.{h,cpp}` (OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated) for
**single-seam-per-loop transversal** traces: through-drill cyl∩cyl COMMON/FUSE/CUT and the
sphere∩sphere lens COMMON. The `ssi_boolean_solid` gate DECLINES every BRANCHED trace:

```cpp
const ssi::TraceSet trace = ssi::trace_intersection(adA, adB);  // default: branch points OFF
if (trace.nearTangentGaps > 0) return {};   // a branch traced up to a tangent → S4
if (trace.branchPoints > 0) return {};       // S4-d self-crossing → OCCT
for (const ssi::WLine& w : trace.lines)
  if (w.status == ... || w.status == BranchArc) return {};   // arm WLine → OCCT
```

The declined case is the **Steinmetz bicylinder** — two equal-radius cylinders whose axes
cross orthogonally. S4-d (`add-native-ssi-s4d-branch-points`, archived) already localizes
both branch points, enumerates the tangent-cone arms, routes them, and returns a fully
resolved branched `TraceSet` — but ONLY when the marcher is called with
`MarchOptions.enableBranchPoints = true` (default `false`, so every existing S5 pass is
byte-identical). This change is the **assembler** for that branched trace: recognise the
Steinmetz family, split each cylinder wall along its arcs into the inside-the-other lune
patches, select per op, and weld into one watertight shell sharing the arc seams and the two
branch-point vertices. OCCT `BRepAlgoAPI` + the exact `16 R³ / 3` analytic are the
verification ORACLES only; the assembler is clean-room and OCCT-free.

### Diagnosed branched trace (host, `CYBERCAD_HAS_NUMSCI` ON, R=1, axes Z & X, z,x∈[−3,3])

`ssi::trace_intersection(adA, adB, {}, {.enableBranchPoints = true})` returns:

```
lines = 4   tracedBranches = 4   nearTangentGaps = 0   branchPoints = 2   routedArms = 3
branchNodes = 2
  B[0] point = (0, -1, 0)  sine = 1.72e-07  onSurf = 6.1e-13  arms = {0, 10000, 10001, 10002}
  B[1] point = (0, +1, 0)  sine = 1.88e-07  onSurf = 8.9e-13  arms = {0, 10000, 10001, 10002}
  L[0] branchId=0     BranchArc  167 pts  onSurf 6.1e-10  u1[-1.57,1.58] v1[0,1]   u2[0,π]  v2[0,1]
  L[1] branchId=10000 BranchArc 1026 pts  onSurf 2.0e-09  u1[-1.57,1.59] v1[-1,0]  u2[π,2π] v2[0,1]
  L[2] branchId=10001 BranchArc  173 pts  onSurf 8.2e-10  u1[-4.73,-1.57] v1[0,1]  u2[0,π]  v2[-1,0]
  L[3] branchId=10002 BranchArc  173 pts  onSurf 8.2e-10  u1[1.57,4.73] v1[-1,0]   u2[-π,0] v2[-1,0]
```

Every arm runs branch-to-branch between `(0, +1, 0)` and `(0, −1, 0)`; every node is on both
cylinders (`onSurfResidual ≤ 2e-9`). On cylinder A (axis Z, `v` = axial Z) every arm's `v1`
stays within `[−1, 1]` — the |z| ≤ R band, exactly the region of A INSIDE cylinder B; the
same holds for B. This is the structural fact the split/select uses.

### Analytic ground truth (host, no OCCT)

Steinmetz **COMMON** (bicylinder) `= 16 R³ / 3` (R=1 → **5.33333**). For finite cylinders of
radius R and axial extent `h` (here 2·3 = 6, `vol_cyl = π R² h = 6π ≈ 18.84956`):
**FUSE** `= 2·vol_cyl − 16R³/3 ≈ 32.36578`; **CUT** `= vol_cyl − 16R³/3 ≈ 13.51622`. These
match the sim harness's recorded OCCT values (`volO common = 5.3333`, `fuse = 32.366`,
`cut = 13.516`), so both the analytic and OCCT oracles agree.

## Goals / Non-Goals

**Goals**
- (S5-d) Compute native `common` (`A ∩ B`) for the **Steinmetz family** (equal-R orthogonal
  crossing cylinders) from the S4-d branched `TraceSet`: split each cylinder wall along its
  two arcs into the inside-the-other lune patch(es), keep the four lunes, and weld them into
  one watertight `Solid` sharing the four arc seams and the two branch-point vertices. Return
  a watertight curved-faced `Solid` or NULL → OCCT.
- (S5-d) Compute native `fuse` / `cut` for the same family WHERE they assemble a watertight,
  correct-volume shell (the complementary fragment selection over the same arcs + the
  cylinders' end caps). Ship ONLY if verified; else NULL → OCCT.
- Keep the ENGINE self-verify unchanged: the EXISTING Steinmetz oracle
  (`ssiCurvedBooleanVerified`, `16 R³ / 3`) verifies COMMON; the EXISTING generic
  set-algebra guard verifies FUSE/CUT.
- Report native-vs-OCCT parity for the Steinmetz COMMON/FUSE/CUT; call out what still
  declines.

**Non-Goals (deferred — never faked here)**
- **General / non-Steinmetz branched pairs** — unequal-R or non-orthogonal or non-crossing
  branched cylinder pairs; cylinder∩sphere / cylinder∩cone / cone∩cone self-crossings; any
  pair with ≠ 2 branch points or ≠ 4 arms; any branched trace with `nearTangentGaps > 0`
  (an arm the S4-d marcher could not resolve). All DECLINE → OCCT.
- **The single-seam S5-a/b/c paths, their trace, their gate.** The branched re-trace is
  entered ONLY when the DEFAULT unbranched trace declined AND the Steinmetz pre-gate matches,
  so no existing S5 pass changes its trace or result.
- Any change to `src/native/tessellate` (the S5-a lesson), to `src/native/ssi` (the S4-d
  tracer is consumed unchanged), to the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the engine oracles, or the S5-a/b/c builders.

## Module shape

```
src/native/boolean/ssi_boolean.cpp   [CYBERCAD_HAS_NUMSCI]
  buildCommon / buildCut / buildFuse / buildLensCommon   // S5-a/b/c — UNCHANGED
  steinmetzPreGate(csA, csB)          // NEW: cheap cyl/cyl, ~equal R, ~orthogonal crossing axes
  recogniseSteinmetzTrace(trace, ...) // NEW: 2 branch nodes + 4 BranchArc arms, all on both cyls
  buildSteinmetzCommon(A, B, arcs, branchPts)  // NEW S5-d: the four lune patches, welded
  buildSteinmetzCut  (A, B, arcs, branchPts)   // NEW S5-d (if tractable): A−B
  buildSteinmetzFuse (A, B, arcs, branchPts)   // NEW S5-d (if tractable): A∪B
  ssi_boolean_solid(a, b, op)         // dispatch: default trace → S5-a/b/c; else, if the
                                      //   Steinmetz pre-gate matches, RE-TRACE branched and
                                      //   route to the buildSteinmetz* builders; else NULL
```

Reuses (no new files needed): `VertexPool` (`assemble.h`), `wallSurface`, `pushPlanarTri`,
`radialOut`, `nearU` / `unwrapRim` (periodic (u,v) folding), `classifyPoint`,
`recogniseCurvedSolid`, `kCapSagitta`, `seamNodeTarget` / `decimateSeam` — all already in
`ssi_boolean.{h,cpp}`. OCCT-free.

## Pipeline

### Dispatch (`ssi_boolean_solid`) — extended, still short + linear

The gate + DEFAULT trace are UNCHANGED (recognise both operands as elementary curved
solids; obtain the default `TraceSet` with branch points OFF). The single-seam S5-a/b/c
paths run FIRST exactly as today. The branched path is entered ONLY on the existing decline
edge:

```
csA, csB = recogniseCurvedSolid(a), recogniseCurvedSolid(b)     // unchanged
trace = trace_intersection(adA, adB)                            // unchanged (branch pts OFF)
if trace is a clean single-seam transversal:                    // unchanged
    → S5-a/b/c buildCommon / buildFuse / buildCut / buildLensCommon
else if trace.nearTangentGaps > 0 (declined) AND steinmetzPreGate(csA, csB):   // NEW
    bt = trace_intersection(adA, adB, {}, {.enableBranchPoints = true})        // S4-d branched
    if recogniseSteinmetzTrace(bt): → buildSteinmetz{Common,Cut,Fuse}(...)     // S5-d
else:
    return {}   // → OCCT
```

`steinmetzPreGate` is the cheap discriminator (both `CurvedKind::Cylinder`,
`|rA − rB| ≤ tol·max(rA,rB)`, axis directions orthogonal `|â·b̂| ≤ tol`, axis LINES cross
within tol) — it gates the extra branched trace so no non-Steinmetz pair pays for it and no
existing S5 pass re-traces. `recogniseSteinmetzTrace` is the honest structural gate (below).
The dispatcher stays a short table; the geometry lives in the flagged builders.

### Steinmetz-family recognition gate (`recogniseSteinmetzTrace`)

Accept the branched `TraceSet` ONLY when ALL hold (else NULL → OCCT):

1. `nearTangentGaps == 0` — the branched structure is FULLY resolved (no arm deferred).
2. `branchPoints == 2` and `branchNodes.size() == 2` — the two saddles.
3. Exactly **four** WLines, ALL `status == BranchArc` — the four branch-to-branch arcs.
4. Each arm's `onSurfResidual ≤ onSurfTol` — every arc node on BOTH cylinders.
5. Each `BranchNode.armLineIds` connects all four arms (the four branchIds), and each arm's
   two endpoints coincide (≤ `branchMergeFrac·scale`) with the two branch-node points — the
   arcs actually meet at the two shared branch vertices.

The pre-gate already pinned equal-R orthogonal crossing cylinders, so the structural gate
just confirms the trace has the Steinmetz shape. A branched trace failing ANY of (1)–(5) —
a third branch point, a deferred arm, a non-arc WLine, arcs not meeting at the two nodes — is
out of scope → NULL → OCCT, never assembled.

### Wall split into inside-the-other lune patches

Each cylinder carries TWO of the four arcs (its `(u,v)` track is the one on THAT cylinder).
The two arcs on a cylinder bound the region of that cylinder's wall INSIDE the other cylinder
— on the Z-axis cylinder A the arcs stay in `v1 ∈ [−R, R]` (the |z| ≤ R band), so the region
they enclose (containing the azimuths facing cylinder B, i.e. u near 0 and near π) is the
part of A inside B. There are TWO such lune patches per cylinder (one on each side, around
u≈0 and u≈π), so FOUR lune patches total — the Steinmetz COMMON boundary.

To SPLIT robustly and periodically-safely, build each lune patch as a **radially/parametric
fan bounded by its two arcs and the two shared branch-point vertices**, sampled ON the
analytic cylinder:

- A lune patch is bounded by two arcs (each a branch-to-branch `BranchArc`, sharing the two
  branch vertices) that together form a closed loop on the cylinder. Emit the patch as a
  strip of PLANAR triangles between the two arcs, walking both arcs branch-to-branch in
  lockstep by arc-length fraction (like `appendTubeBand`'s rim pairing but between two arcs
  meeting at shared endpoints), every interior sample placed ON the analytic cylinder via
  `cs.point(u, v)` at the interpolated `(u,v)`, and the two branch vertices pooled ONCE.
- Periodic safety: fold each arc's `u` track contiguous with `nearU` around the patch's
  centroid azimuth (the S5-a discipline) so the |z|≤R band's u-span does not wrap ±2π.

Which two arcs bound which lune is decided by the `(u,v)` tracks: group the two arcs on each
cylinder by the side (u-window) they occupy; the patch's interior azimuth (u near the group
mean) selects the SURVIVING side per op.

### Select per op (the set algebra, `classifyPoint`)

Sample each candidate lune patch at its centroid (well inside the patch, on the cylinder
surface) and classify against the OTHER solid:

- **COMMON `A ∩ B`** — keep the four lune patches whose centroid is INSIDE the other cylinder
  (`classifyPoint(other, centroid) == +1`). These four inside∩inside patches ARE the
  bicylinder boundary. (An ON-band centroid → abort → NULL → OCCT.)
- **CUT `A − B`** — keep A's wall OUTSIDE B + B's inside-A lune patches REVERSED (the tunnel
  wall) + A's two end caps; drop A's inside-B lunes. The shared arcs weld the reversed B
  patches to A's outside wall.
- **FUSE `A ∪ B`** — keep each cylinder's OUTSIDE-the-other wall + both cylinders' end caps;
  drop both cylinders' inside-the-other lunes (now interior). The shared arcs weld the two
  outer walls.

Selection is the SAME rule as the planar `booleanPolygons` and the S5-a/b `classifyPoint`
selection — COMMON is the guaranteed slice (four patches, exact `16 R³/3`); FUSE/CUT ship
ONLY if their fragment weld verifies watertight + correct-volume.

### Weld (planar facets, shared arcs + shared branch vertices)

The S5-a watertight discipline applies verbatim:

- ONE `VertexPool` for the whole shell. Every arc's shared 3D nodes are pooled, so a patch on
  either side of an arc draws the SAME vertices → the arc seam welds.
- The two branch-point vertices `(0, ±R, 0)` are pooled ONCE (via the pool's coincident-vertex
  weld); all four arcs terminate at those two pooled vertices, so the four patches meet there
  with no crack.
- Every seam-adjacent patch emits PLANAR-TRIANGLE facets through the pooled arc/branch nodes
  (no analytic surface face on a shared seam — its structured-grid mesh would inject interior
  u-lines and open a T-junction, the exact S5-a failure). Interior fan/strip nodes are unique
  to a patch (no neighbour) so they introduce no T-junction. Facet normals oriented outward
  (the cylinder's outward radial for a lune; ±axis for an end cap) via `pushPlanarTri` /
  `radialOut`.
- `makeShell(patches) → makeSolid`. Return the candidate; the ENGINE self-verifies.

## Curved point-in-solid classification (reused, unchanged)

Reuses S5-a's `classifyPoint(CurvedSolid, P, tol)` unchanged: signed distance to the curved
wall (cylinder radial slab) + cap half-spaces; `+1` inside, `−1` outside, `0` ON. An ON
verdict aborts the native path → NULL → OCCT, never a guessed side. Used to select the
inside-vs-outside lune patches per op.

## Self-verify → OCCT fallback (ENGINE — NO new oracle)

`ssi_boolean_solid` returns the candidate `Solid` or NULL; the ENGINE decides shippability,
exactly as for S5-a/b/c. **No engine code change is required:**

- **S5-d COMMON** is caught by the EXISTING `ssiCurvedBooleanVerified` **Steinmetz oracle**
  (`native_engine.cpp`, op == common, equal-radius orthogonal cylinders — the `16 R³/3`
  closed form). Today that oracle always finds a NULL native candidate and falls to OCCT;
  now it verifies the branched native candidate's watertight volume against `16 R³/3` and
  DISCARDS a mismatch → OCCT. This is the intended consumer of the oracle that S4-d/S5-d were
  building toward — no oracle change, it finally has a candidate.
- **S5-d FUSE/CUT** are caught by the EXISTING generic set-algebra guard
  (`Vr ≈ va + vb − vc` / `va − vc`, `vc` = the native branched COMMON), DISCARDING a bad
  candidate → OCCT.

The single-seam S5-a/b/c guards are untouched and do not fire for the branched Steinmetz
case (that pre-gate requires equal-R orthogonal cylinders, which the through-drill /
sphere-lens gates reject). The library stays OCCT-free.

## Steinmetz-vs-deferred scope (honest)

| Configuration | behaviour |
|---|---|
| Equal-R orthogonal crossing cylinders, `branchPoints == 2`, 4 `BranchArc` arms, `nearTangentGaps == 0`, **Common** | `buildSteinmetzCommon` (S5-d) → native `Solid` (verified vs `16 R³/3` + OCCT, or discard → OCCT) |
| Same family, **Cut / Fuse** | `buildSteinmetzCut` / `buildSteinmetzFuse` (S5-d) → native `Solid` if it verifies; else NULL → OCCT |
| Through-drill cyl∩cyl (unequal R), any op | S5-a/b `buildCommon`/`buildCut`/`buildFuse` (single-seam, unchanged) |
| Transversal sphere∩sphere, **Common** | S5-c `buildLensCommon` (single-seam, unchanged) |
| Unequal-R / non-orthogonal / non-crossing branched cylinder pair | pre-gate rejects → NULL → OCCT |
| Branched trace with `nearTangentGaps > 0`, ≠ 2 branch points, or ≠ 4 arms | `recogniseSteinmetzTrace` rejects → NULL → OCCT |
| cyl∩sphere / cyl∩cone / cone∩cone self-crossings, freeform branched | pre-gate / recognition rejects → NULL → OCCT |
| Self-verify (watertight or volume) fails on a candidate | ENGINE DISCARDS → OCCT, reported |

## Verification model (two gates)

- **Host (no OCCT), analytic oracle.** Extend
  `tests/native/test_native_ssi_curved_boolean.cpp`, under `CYBERCAD_HAS_NUMSCI`:
  - **S5-d Common:** the equal-R (R=1) orthogonal Steinmetz pair (axes Z & X through the
    origin) re-traces branched (`branchPoints == 2`, four `BranchArc` arms,
    `nearTangentGaps == 0`); `ssi_boolean_solid(A, B, Common)` is non-NULL, watertight
    (`watertightMeshVolume > 0`, every edge shared by exactly two faces), and its enclosed
    volume equals the EXACT `16 R³ / 3 = 5.33333` within the curved-face deflection band
    (~1%); every arc node on BOTH cylinders ≤ tol; the two branch-point vertices are a single
    shared node (all four arcs weld there).
  - **S5-d Fuse/Cut (if shipped):** volume equals `2·vol_cyl − 16R³/3` and
    `vol_cyl − 16R³/3` within the band, each watertight; monotone invariants hold.
  - **Deferrals:** an UNEQUAL-radius orthogonal cylinder branched pair and a NON-orthogonal
    equal-R cylinder pair return NULL (pre-gate / recognition decline). The single-seam
    S5-a/b/c tests stay green unchanged.
  - Full CTest green NUMSCI ON and OFF (the S5-d assertions correctly absent with NUMSCI
    off, like the S5-a/b/c tests). No OCCT linked; no tolerance weakened.
- **Sim native-vs-OCCT — `BRepAlgoAPI` parity.** The existing
  `tests/sim/native_ssi_curved_boolean_parity.mm` +
  `scripts/run-sim-native-ssi-curved-boolean.sh` ALREADY carry the `cyl=cyl(steinmetz)`
  COMMON/FUSE/CUT cases (today native NULL → OCCT). After this change they must become
  NATIVE passes: the native branched result matches `BRepAlgoAPI_{Common,Fuse,Cut}` in
  volume, surface area, watertight closed shell, and `BRepCheck` validity within tol. Update
  the harness comments so the Steinmetz pair is recorded as a native pass (not the honest
  fall-back it records today); report per-op the native-vs-OCCT deltas and the count still
  deferred to OCCT (out-of-family branched pairs). Parity is a REPORTED figure; whatever does
  not pass falls back to OCCT and is reported with the measured gap.

## Decisions

- **Consume the S4-d branched trace; add NO tracer code.** S4-d already localizes the two
  branch points and routes the four arms; S5-d only assembles them. The branched trace is
  obtained by RE-CALLING `trace_intersection` with `enableBranchPoints = true` on the
  Steinmetz pre-gate edge — the tracer is untouched, and no default S5 trace changes.
- **Pre-gate before the branched re-trace.** The extra branched trace is only run when the
  default trace declined AND the pair is cheaply Steinmetz-shaped (both cylinders, ~equal R,
  ~orthogonal crossing axes). This keeps every non-Steinmetz pair and every existing S5 pass
  on their unchanged single trace, and bounds the cost.
- **Split into lune patches by the arcs, weld on shared arcs + branch vertices.** The four
  arcs already carry the exact seam on both cylinders; the assembler reuses the S5-a
  planar-facet weld and the `VertexPool` coincident-vertex weld so the four lunes share the
  arcs AND the two branch points — the branched analogue of the S5-a two-rim weld. This is why
  it is an assembler slice, not new tracing.
- **COMMON is the guaranteed slice; FUSE/CUT ship only if they verify.** COMMON's four lunes
  have the clean exact `16 R³/3` oracle and the simplest weld; FUSE/CUT add the cylinders'
  end caps + the outside-wall fragments — shipped only when the watertight + set-algebra guard
  accepts, else NULL → OCCT (honest, never faked).
- **No new engine oracle.** The Steinmetz `16 R³/3` oracle already exists in the engine and
  was waiting for a native candidate; the generic set-algebra guard already covers FUSE/CUT.
  Adding an oracle would be redundant and risk masking a real weld defect.
- **Planar facets on every seam-adjacent face; tessellator untouched.** The S5-a watertight
  lesson applies verbatim — any face sharing an arc or a branch vertex emits planar-triangle
  facets through the shared pooled nodes.

## Risks / Trade-offs

- **Lune centroid mis-classification.** A patch whose centroid sits near an arc could
  mis-select. Mitigation: sample at the patch centroid (well inside), the `classifyPoint`
  ON-band abort → NULL, and the engine correct-volume guard DISCARD → OCCT. Never shipped
  wrong.
- **Four-arc weld at the branch points.** If the two branch vertices are not pooled as ONE
  shared vertex, the four patches crack there. Mitigation: pool each branch point ONCE and
  take every arc's shared 3D nodes from the SAME `VertexPool`; the engine watertight check is
  the backstop → OCCT.
- **Periodic (u,v) wrap on a lune's u-span.** The |z|≤R band spans a wide azimuth window that
  can wrap ±2π. Mitigation: the S5-a `nearU` / `unwrapRim` contiguous folding the assembler
  already uses.
- **FUSE/CUT re-trim hairline gap.** The cylinders' end caps + outside-wall fragments can
  leave a hairline seam mismatch. Mitigation: emit them as planar facets through the shared
  pooled arc nodes; ship FUSE/CUT ONLY if the watertight + set-algebra guard accepts, else
  NULL → OCCT. COMMON is the guaranteed native slice.
- **Branched re-trace cost.** The branched trace is heavier (one arm has ~1000 nodes).
  Mitigation: it runs ONLY on the Steinmetz pre-gate edge (rare), and the assembler decimates
  dense arcs to the `seamNodeTarget` node budget (the S5-c discipline) before welding.
