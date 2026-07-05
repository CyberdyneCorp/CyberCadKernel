# Design — add-native-shape-healing (first native shape-healing slice)

## Context

`NATIVE-REWRITE.md` names shape healing (#4) and native STEP/IGES import (#3) as two
of the tracks (with SSI + curved booleans) still gating `drop-occt`. They are
coupled: a native STEP parser produces a **face soup** — a bag of trimmed faces
whose shared boundaries are *geometrically* coincident (within the exporter's write
tolerance) but *topologically* independent (each face carries its own copy of the
shared edge / vertex). That is not a valid B-rep: the rest of the kernel
(`tessellate` / mass / `query` / `boolean`) needs a **connected, consistently
oriented, watertight shell/solid** with shared edges and vertices. Healing bridges
that gap. So healing is the **gating foundation** import stands on, and it is worth
landing first, independently, verified against OCCT.

This slice heals the **tractable 80%**: the *coincident-within-tolerance /
degenerate / orientation* defect family. These are the defects a spatial hash +
closed-form geometry repair **exactly** — no re-approximation, no research-grade
tolerance chasing. Everything harder (pcurve reconstruction, self-intersecting-wire
repair, bridging a gap that is genuinely beyond tolerance) is OUT OF SCOPE and
defers to OCCT `ShapeFix`, honestly reported.

The method is **CLEAN-ROOM**: derive the sew/weld/orient logic from first principles
and standard B-rep references (Mäntylä; Hoffmann; the tessellator's shared-edge weld
and the boolean `VertexPool`), using OCCT (`BRepBuilderAPI_Sewing`, `ShapeFix_Shell`,
`ShapeFix_Solid`) strictly as a verification **oracle**, never copied.

Healing is an **INTERNAL** capability, invoked by the engine (after a future native
import, or to repair a native boolean/loft residual), NOT exposed on the `cc_*` C
ABI. It is therefore verified at the healing-function level (native vs OCCT sewing /
ShapeFix) — the same discipline as native-ssi / native-math / native-topology — with
**no ABI change**.

## Goals / Non-Goals

**Goals**
- A native `cybercad::native::heal` module: given a `topology::Shape` (face soup /
  malformed shell) + a tolerance, run the four sub-operations and return a repaired
  watertight shell/solid OR an honest UNHEALED result.
- Four sub-operations: **tolerant sewing**, **vertex/tolerance unification**,
  **degenerate removal**, **orientation fix**.
- Every heal **self-verified** watertight + valid + all-merges-within-tolerance
  before it is kept — never a claimed-but-false closure.
- A typed **UNHEALED** result (with the measured residual gap) as the honest
  deferral seam, feeding the engine's OCCT fallback.
- OCCT-free; no numeric substrate needed (pure spatial hash + closed-form geometry),
  so it builds identically under no-OCCT and `CYBERCAD_HAS_NUMSCI`.

**Non-Goals (deferred to OCCT `ShapeFix` — never faked here)**
- Bridging a gap that is genuinely **beyond tolerance** (a real hole in the model).
- A **genuinely open** shell that cannot close within tolerance.
- **Missing pcurve reconstruction** (re-projecting a 3D edge onto a face's surface).
- **Self-intersecting-wire** repair.
- General non-coincident / non-degenerate industrial B-rep repair; freeform-surface
  re-approximation.
- Any `cc_*` facade entry point or ABI change; any tessellator change.

## Module shape

```
src/native/heal/
  native_heal.h       // aggregate public header + namespace doc + HONEST-SCOPE caveat
  heal_result.h       // HealResult (metrics + healed/watertight/valid) + UnhealedReason
  vertex_unify.h      // generalized VertexPool: quantized spatial hash → shared Vertex
  tolerant_sew.h      // edge-pair coincidence match + merge into one shared edge
  degenerate.h        // zero-length-edge + sliver/zero-area-face detection & removal
  orient.h            // flood-fill outward orientation across shared edges
  self_verify.h       // tessellate candidate → isWatertight + enclosed-volume sign check
  heal.h/.cpp         // top-level healShell(shape, opts) orchestrating the four steps
```

Reads/rebuilds the B-rep via `topology::` (`ShapeBuilder`, `accessors.h`
`pointOf/curveOf/surfaceOf`, `explore.h`). Self-verify tessellates via
`tessellate::` (`isWatertight`, `enclosedVolume`). Reuses the coincidence-hash idea
of `boolean/assemble.h` `VertexPool` and `exchange/stl_reader.cpp`'s grid weld.

## Result type

```cpp
enum class HealStatus {
  Healed,      // watertight + valid; caller uses `shape`
  Unhealed,    // out-of-scope / gap > tol / still open → DEFER to OCCT ShapeFix
};

enum class UnhealedReason {
  GapBeyondTolerance,   // a shared boundary is farther apart than `tolerance`
  OpenShell,            // still has boundary edges after sewing (a real hole)
  NonManifold,          // an edge is shared by 3+ faces (bad input, not a soup)
  SelfVerifyFailed,     // candidate did not tessellate watertight / volume sign wrong
  OutOfScope,           // missing pcurve, self-intersecting wire, freeform re-approx
};

struct HealMetrics {
  bool   watertight = false;      // self-verified closed 2-manifold mesh
  bool   valid = false;           // consistent outward orientation (enclosed vol > 0)
  int    nMergedVerts = 0;        // near-coincident vertices unified
  int    nMergedEdges = 0;        // coincident-within-tol edge pairs stitched
  int    nDroppedDegenerate = 0;  // zero-length edges + sliver/zero-area faces removed
  int    nFlipped = 0;            // faces re-oriented by the flood-fill
  double maxResidualGap = 0.0;    // largest surviving boundary gap (0 ⇒ fully closed)
};

struct HealResult {
  HealStatus     status = HealStatus::Unhealed;
  UnhealedReason reason{};        // meaningful only when status == Unhealed
  topology::Shape shape;          // Healed ⇒ the repaired shell/solid; Unhealed ⇒ input UNCHANGED
  HealMetrics    metrics;
};

struct HealOptions {
  double tolerance = math::kLinearTolerance;  // the SEW/WELD tolerance — never weakened to pass
};
```

`status == Unhealed` is the contract with the engine: **do not** trust a closure;
route the ORIGINAL shape to OCCT `BRepBuilderAPI_Sewing` + `ShapeFix`. `status ==
Healed` guarantees the returned shell/solid self-verified watertight + valid.

## Algorithm

`healShell(shape, opts)` runs the four sub-operations in dependency order, then
self-verifies. All tolerances derive from `opts.tolerance` (the caller's stated sew
tolerance); NONE is silently widened.

### (a) Collect faces + boundary edges/vertices
Explore the input for all `Face`s; for each, collect its wire edges and the edges'
endpoint vertices with world-placed points (`accessors.h`). This is the working set
the next steps rebuild — the input graph is immutable, so healing constructs a NEW
graph with shared nodes.

### (b) Vertex / tolerance unification (generalized VertexPool)
Hash every boundary vertex position by its quantized cell (cell side = `tolerance`,
nearest-cell rounding) to a single shared `topology::Vertex` node — the
`boolean/assemble.h` `VertexPool`, generalized to arbitrary B-rep input (not just
axis-aligned boolean corners). Near-coincident vertices (‖Δ‖ ≤ `tolerance`) collapse
to one node; `nMergedVerts` counts the reduction. This is the prerequisite for edge
merging: two edges can only be "the same" if their endpoints already unified.

### (c) Tolerant sewing (edge-pair coincidence → one shared edge)
Two edges from two different faces are **coincident within tolerance** — and thus one
shared edge — iff (i) their endpoint vertices unified to the SAME two shared vertices
(after step b), AND (ii) their curves agree along the span (midpoint proximity +
same curve kind/geometry within `tolerance`). For each such pair, replace both
faces' references with ONE shared edge node (built once, referenced by both faces'
rebuilt wires). An edge that finds NO within-tolerance partner remains a **boundary
edge** — a candidate hole. `nMergedEdges` counts the stitched pairs. **Honesty core:**
an edge is merged ONLY on within-tolerance coincidence; a near-miss BEYOND
`tolerance` is left unstitched (it becomes the measured residual, not a faked
closure).

### (d) Degenerate removal
Drop **zero-length edges** (`‖v1 − v0‖ < tolerance`; both endpoints unified to one
vertex) and **sliver / zero-area faces** (planar/parametric area < `tolerance²`, or a
face whose wire collapses to < 3 distinct vertices after unification). Rebuild the
affected wires/faces without the removed elements. `nDroppedDegenerate` counts them.
A face that degenerates entirely is removed from the shell.

### (e) Orientation fix (flood-fill outward)
Build face adjacency across the now-shared edges. Flood-fill orientation: pick a seed
face, then for each shared edge require the two incident faces to traverse it in
OPPOSITE directions (the manifold consistency rule), flipping a neighbour when they
agree. Seed the global sign from a face whose outward direction is unambiguous, and
CONFIRM by the sign of the enclosed volume of the resulting closed mesh — if the
enclosed volume is negative, flip the whole shell. `nFlipped` counts re-orientations.

### (f) Assemble + self-verify (mandatory)
Wrap the sewn, de-degenerated, consistently-oriented faces in a `Shell`, and a
`Solid` if closed. Then `self_verify.h` tessellates the candidate
(`tessellate::SolidMesher` → `Mesh`) and asserts:
- `tessellate::isWatertight(mesh)` — every undirected edge used by exactly two
  triangles (closed 2-manifold, no boundary, no T-junction), AND
- `tessellate::enclosedVolume(mesh) > 0` — consistent outward orientation, AND
- every recorded merge was within `tolerance` (max merge residual ≤ tolerance).

If all hold ⇒ `HealResult{ Healed, shape = solid, metrics{ watertight=true,
valid=true, maxResidualGap = <max surviving boundary gap = 0> } }`. If the shell is
still open (any boundary edge with no within-tolerance partner) ⇒ `HealResult{
Unhealed, reason = OpenShell | GapBeyondTolerance, shape = INPUT UNCHANGED, metrics{
watertight=false, maxResidualGap = <measured largest surviving gap> } }`. **Never**
returns `watertight=true` unless the mesh actually closed.

## Engine-internal native-heal hook + OCCT fallback

`src/engine/native/` gets an internal `tryNativeHeal(shape, tol)` on the native
engine path, wired the SAME way every native op is (native → self-verify → OCCT
fallback), and reached internally (NOT via `cc_*`):

1. Run `heal::healShell(shape, {tol})`.
2. If `status == Healed` (self-verified watertight + valid) ⇒ keep the native result.
3. Else (`Unhealed`, any reason) ⇒ fall through to the OCCT adapter
   (`CYBERCAD_HAS_OCCT`): `BRepBuilderAPI_Sewing` on the face soup, then
   `ShapeFix_Shell` / `ShapeFix_Solid`. The OCCT path stays entirely in
   `src/engine/occt/` — `src/native/**` never includes an OCCT header.

No `cc_*` entry point is added or changed; the hook is a private code path (like the
SSI hook), so every existing suite and the public ABI are untouched.

## Verification model (two gates, per NATIVE-REWRITE §Verification model)

- **Host (no OCCT):** build deliberately-broken fixtures in the native model and heal
  them; assert the four sub-operations fire with the expected counts, the result is
  watertight + valid with the expected enclosed volume, and the UN-healable fixture
  returns `Unhealed` with a non-zero measured `maxResidualGap`. No OCCT linked.
- **Sim native-vs-OCCT parity:** rebuild the same fixtures for OCCT
  (`TopoDS_Compound` of faces), run `BRepBuilderAPI_Sewing` + `ShapeFix_Shell` /
  `ShapeFix_Solid`, and compare: same watertight/closed shell, same `IsValid` solid,
  same volume (`BRepGProp`) within tol. On the un-healable fixture, assert the native
  UNHEALED verdict matches OCCT leaving it open / needing more (OCCT sewing also does
  NOT close it within the same tolerance). Compared at the `cybercad::native::heal`
  C++ boundary — no `cc_*` call; SSI-style internal parity.

## Fixtures (deliberately broken — one per defect + an un-healable)

- **soup-cube** — six planar faces of a unit cube, each with its OWN copies of the 12
  shared edges + 8 corners, all coincident WITHIN tol but topologically independent.
  Native heal ⇒ watertight solid, V = 1, `nMergedEdges = 12`, `nMergedVerts` collapses
  the corner copies. (tolerant sew + vertex unify.)
- **subtol-gap-cube** — the soup-cube with adjacent faces pulled apart by 0.4·tol
  (still within tol). Heals; residual gap ≤ tol; watertight.
- **near-coincident-verts** — a face soup with corner copies scattered within tol.
  `nMergedVerts` reduces to the true corner count; watertight.
- **degenerate-edge** — a face whose wire has a zero-length edge (two coincident
  corners). `nDroppedDegenerate ≥ 1`; the rebuilt face is valid; watertight solid.
- **sliver-face** — a soup with an extra near-zero-area sliver face inserted. Dropped;
  watertight solid, correct volume.
- **flipped-face** — the soup-cube with one face wound inward. `nFlipped = 1`;
  enclosed volume > 0 after the fix; watertight.
- **UN-healable open-cube (gap > tol)** — the soup-cube with one face pulled 5·tol
  away (a real hole beyond tolerance). Native ⇒ `Unhealed`,
  `reason = GapBeyondTolerance | OpenShell`, `maxResidualGap ≈ 5·tol`, shape
  UNCHANGED. Sim: OCCT sewing at the same tolerance ALSO leaves it open — native
  UNHEALED matches OCCT "needs more".

## Decisions

- **Tolerance is the caller's, never widened.** `HealOptions.tolerance` is the single
  sew/weld tolerance; every merge/degeneracy test uses it (or `tolerance²` for area);
  the healer NEVER auto-relaxes it to force a pass. A defect that needs a larger
  tolerance is reported UNHEALED with the measured residual.
- **`Unhealed` is data, not an error.** UNHEALED is a normal first-class outcome (the
  deferral seam), not an exception — exactly like SSI's `NotAnalytic`.
- **Self-verify via the existing tessellator.** Watertightness is decided by the
  SAME `tessellate::isWatertight` the boolean assembler self-verifies with (shared-edge
  weld ⇒ every edge used twice), and orientation by `enclosedVolume > 0` — no new
  verification machinery, no tessellator change.
- **Reuse, don't re-implement, the weld.** Vertex unification IS the `VertexPool`
  spatial hash generalized; edge merging is built on top of it. This keeps the new
  code small and consistent with the boolean/tessellate weld already trusted.
- **Input is returned UNCHANGED on Unhealed.** The healer never partially mutates the
  caller's shape; on any non-heal it hands back the original so the engine's OCCT
  fallback sees the pristine soup.

## Risks / Trade-offs

- **Family completeness vs honesty (asymptotic, like SSI S4-f).** The in-scope defect
  family (coincident-within-tolerance / degenerate / orientation) is finite and
  healed exactly; the self-verify + UNHEALED seam guarantee that anything outside it
  — a gap beyond tolerance, a missing pcurve, a self-intersecting wire, a genuinely
  open shell — DEFERS to OCCT rather than fakes a closure. Completeness is a
  **measured win over OCCT on the in-scope fixtures, not a guarantee** on arbitrary
  broken industrial B-rep. Baked into the header docs, the tests, and the spec.
- **Coincidence tolerance for edges.** Deciding "same edge" uses endpoint-unification
  + midpoint/curve proximity within `tolerance`; a borderline pair beyond tolerance is
  left as a boundary edge (measured residual), matching the "never fake a closure"
  stance rather than a hand-tuned merge.
- **Orientation seed ambiguity.** The flood-fill needs a consistent seed; the
  enclosed-volume sign check is the global tie-breaker (flip the whole shell if
  negative), so a wrong seed cannot produce an inward-facing "valid" solid.
- **Curved-face healing depth.** The first slice sews and orients faces of any
  surface kind the topology model carries, but the degenerate/sliver-area test and
  the tolerant-sew curve-proximity check are simplest and most exact for
  planar/analytic faces; freeform re-approximation stays OUT OF SCOPE (UNHEALED →
  OCCT). This can be widened when a native import needs it, exactly as SSI widened
  its curve descriptor.
