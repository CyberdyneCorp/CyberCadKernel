# Proposal — moat-m2b-freeform-face-split (MOAT M2b / B2, first slice)

## Why

The general freeform boolean (M2) is blocked on THREE nameable substrate
subsystems the analytic S5 assembler never needed
(`openspec/MOAT-ROADMAP.md`, "M2 substrate — B1/B2/B3"). This change lands the
FIRST SLICE of **B2 — WLine freeform face-split**: there is today NO path to
partition a trimmed freeform face along the M1-traced seam. `recogniseCurvedSolid`
declines every freeform operand (B1), and even if it did not, nothing splits a
freeform face into in/out fragments — only analytic caps/walls are assembled. So a
freeform ∪/−/∩ has no in-face to hand to the welder; the whole operation honestly
declines to OCCT.

The M1 tracer already produces the seam as DATA: a `WLine` whose `points` are
`WLinePoint` nodes each carrying `(u1,v1)` on surface A and `(u2,v2)` on surface B
(`src/native/ssi/marching.h`). The M0 mesher already meshes a genuinely-trimmed
freeform `EDGE_LOOP` face watertight (`FaceMesher::trimmedFreeformMesh`,
`triangulateConstrained`). What is MISSING is the purely-2D combinatorial step
BETWEEN them: take one trimmed freeform face's outer loop in `(u,v)` and the seam
polyline `(u1,v1)` and cut the trimmed domain into TWO closed sub-loops (in / out of
the cutter), producing two genuinely-trimmed sub-faces that TILE the original.

A general freeform face partition (arbitrary seam count, holes, tangential grazes,
seam re-entry, branch points) is a multi-person-year capability. This change is the
SIMPLEST reachable case, NOT the general splitter: ONE trimmed freeform face with a
convex outer `EDGE_LOOP` and ONE seam chord that CLEANLY crosses it — the seam
enters the trimmed domain through one boundary edge and exits through another, with
no tangency and no re-entry. Anything outside that envelope DECLINES (returns a NULL
split), so the boolean keeps its honest OCCT fall-through. A correct decline with a
measured blocker is a first-class, expected outcome; a leaky or overlapping split is
NEVER emitted.

## What Changes

1. **A new header-only, OCCT-free 2D face-split subsystem** in
   `src/native/boolean/` (with a small seam-clip helper in `src/native/ssi/` if it
   fits the marcher's data model). It consumes — never rewrites — the landed M0 and
   M1 code:
   - INPUT: one trimmed freeform `Face` (`FaceSurface` `Kind::BSpline`/`Bezier` with
     a real outer-wire `EDGE_LOOP`), and the M1 `WLine` seam whose `(u1,v1)` nodes
     are the seam pcurve on THIS face's `(u,v)` domain (surface A = this face).
   - Flatten the outer `EDGE_LOOP` to a UV boundary polygon at the SAME shared
     per-edge fractions the mesher uses (`buildBoundaryLoops`), so a sub-face's
     boundary points stay BIT-IDENTICAL to the parent's (the weld contract).
   - Clip the seam polyline to the trimmed UV domain: find where it ENTERS and EXITS
     the outer polygon (segment/boundary crossings), keeping the interior chord.
   - Partition: walk the outer boundary from the entry crossing to the exit crossing
     one way for sub-loop L1, the other way for sub-loop L2, splicing the shared seam
     chord (reversed) into each. Result: two closed UV sub-loops sharing the seam
     exactly, tiling the parent with no overlap and no gap.
   - Rebuild each sub-loop as a genuinely-trimmed sub-`Face` over the SAME
     `FaceSurface` node: parent boundary arcs keep their existing 3D edges/pcurves;
     the seam becomes a new shared edge whose pcurve is the `(u1,v1)` polyline
     (`PCurve` on the face surface) and whose 3D curve is the `WLine`'s fitted curve.
2. **A mandatory self-verify → decline gate** (no OCCT): the two sub-faces are
   accepted ONLY when the seam cleanly enters and exits the trimmed domain, each
   sub-region is non-degenerate (positive UV area, simple loop), the seam is the
   EXACT shared boundary of both, and the two sub-face UV areas SUM to the parent's
   within a scale-relative tolerance. If any check fails — seam does not cleanly
   cross, a sub-region degenerates, areas do not tile — the splitter returns NULL
   (decline). NEVER a wrong/leaky split.
3. **NO change to the tessellator or the tracer** beyond strictly-additive consumption.
   The M0 `FaceMesher` meshes each sub-face as-is; the M1 `WLine` is read, not
   modified. The `cc_*` ABI is unchanged (this is internal substrate, not yet wired
   into any `cc_*` call — that integration is M2a/B1). `src/native/**` stays
   OCCT-free (0 OCCT includes).

## Impact

- **Affected specs:** `native-booleans` (ADDED requirements — the freeform
  face-split substrate + its self-verify/decline gate + the two proof gates). No
  existing requirement is modified: the boolean-level decline behaviour is unchanged
  until M2a wires this in.
- **Affected code (new, additive, OCCT-free):** `src/native/boolean/` (new
  face-split header + host/sim tests), optionally a seam-clip helper in
  `src/native/ssi/`. Consumes `src/native/topology/shape.h`,
  `src/native/ssi/marching.h` (M1), `src/native/tessellate/face_mesher.h` +
  `uv_triangulate.h` (M0). No edit to the tessellator or tracer.
- **Proof — two gates.** (a) HOST ANALYTIC (no OCCT linked): the two sub-faces TILE
  the original — UV areas sum to the parent within tolerance, the seam is the shared
  boundary of both exactly, no overlap/gap. (b) SIM native-vs-OCCT (booted iOS
  simulator, OCCT linked): each sub-face meshes WATERTIGHT via the M0 mesher and the
  union of the two sub-faces reproduces the parent face's area/topology, cross-checked
  against the OCCT `BRepMesh` area of each sub-face.
- **Out of scope (explicit, deferred):** multiple seams, seams that re-enter,
  tangential grazes, branch points, holes in the trimmed domain, concave outer loops
  where a chord exits and re-enters, and the boolean-level wiring
  (`recogniseCurvedSolid` admission is M2a/B1; point-in-solid labelling is M2c/B3).
  Each such case DECLINES with a measured blocker.
