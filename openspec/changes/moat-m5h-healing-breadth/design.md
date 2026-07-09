# Design — moat-m5h-healing-breadth (MOAT M5 tail — short-edge merge)

## Context

The native healer rebuilds a face soup onto shared vertex/edge nodes and self-verifies.
A face is read as an ordered world-corner loop (`face_soup.h`); corners are unified into
shared vertex nodes (`vertex_unify.h`, single-cell spatial hash at `tolerance`); an edge
node is shared per unordered vertex-node pair (`tolerant_sew.h`, `EdgePool`); a side used
by exactly one face is a *boundary* edge. A closed manifold has ZERO boundary edges. The
pipeline then flood-fills consistent orientation (`orient.h`), assembles a solid
(`assemble_shell.h`), and self-verifies watertight + positive enclosed volume
(`self_verify.h`). Three opt-in tail passes (gap bridging, single- and multi-hole planar
capping) each rewrite the soup / append a cap and re-sew, all default-OFF.

## The defect and why the landed passes decline it

A STEP exporter / mesh→B-rep conversion can split a boundary vertex on an
otherwise-straight wire run into two distinct vertices a tiny distance `seg` apart,
inserting a NON-zero micro-edge `B→C`. When `tolerance < seg` (e.g. `seg = 5e-3`,
`tolerance = 1e-4`):

- `degenerate.h::dropZeroLengthSides` does NOT remove it — that pass drops only
  consecutive corners within `tolerance`.
- `vertex_unify` keeps `B` and `C` as distinct nodes (they are `> tolerance` apart from
  each other and from `A`/`D`).
- The face's boundary run is `A→B→C→D` (edges `{A,B}`, `{B,C}`, `{C,D}`), but the
  NEIGHBOUR face's matching span is one straight edge `{A,D}`. None of `{A,B}`, `{B,C}`,
  `{C,D}`, `{A,D}` is shared ⇒ four surviving boundary edges ⇒ `Unhealed` (honestly, with
  `maxResidualGap = seg` reported by the landed `residualGap`).

`gap_bridge.h` cannot help: it only snaps *cross-face* unpaired corners, and here `B`/`C`
belong to a single face — moving them to a partner would fold the face, not close a seam.

## The pass

`collapseShortEdges(soup, tol, mergeLen)` (`short_edge.h`) sweeps each face loop once. For
a side `B→C` with wire neighbours `A = prev(B)` and `D = next(C)` it collapses the side —
removing BOTH `B` and `C`, so the run becomes the straight span `A→D` — iff ALL hold:

1. **Opt-in merge length.** `mergeLen > 0` (else the pass is a dead-guarded no-op).
2. **Bounded band + local-feature cap.** `tol < |B−C| ≤ min(mergeLen, ¼·min(|A−B|,|C−D|))`.
   The `¼·neighbour` cap (`kShortEdgeFeatureFraction = 0.25`, the same constant as
   `gap_bridge.h`) is the geometric guarantee that an edge comparable to a real feature is
   NEVER removed, whatever the caller sets `mergeLen` to.
3. **Collinearity within tolerance.** Both `B` and `C` lie within `tol` of the straight
   line `A→D` (perpendicular distance). A short edge that TURNS a real corner (`B` or `C`
   off the `A-D` line) is left in place — the pass only removes a *redundant* split, and
   removing it restores the EXACT straight span the neighbour already carries.
4. **Loop stays a polygon.** Collapsing must leave `≥ 3` corners.
5. **Mandatory self-verify (unchanged).** After collapse + re-sew the candidate must still
   tessellate watertight with positive enclosed volume; else `Unhealed{SelfVerifyFailed}`.

### Why remove BOTH endpoints (not merge to a midpoint)

Merging `B` and `C` to a single interior vertex `M` would leave `M` still on the line but
still absent from the neighbour face → the run would remain unshared. The neighbour face
carries the span `{A,D}` with NO interior vertex, so the only restorative operation that
lets the sew share the run is to drop the micro-edge entirely, leaving `A→D`. Because both
`B`/`C` are collinear (layer 3), dropping them does not change the face's boundary
geometry — it restores the polygon the exporter should have written.

### Pipeline placement

The pass rewrites per-face corner loops, so it runs on the working soup BEFORE the first
`sew(work, tol)` in `heal.cpp`, guarded by `opts.shortEdgeMergeLen > 0.0`. With the default
`0.0` the block is never entered and `work` stays `== clean` — the sew, bridging, capping,
orientation, and self-verify all see byte-identical input, so every existing caller is
byte-identical.

## Determinism and honesty

The sweep is left-to-right and marks disjoint windows (a collapse advances past `D` so a
corner is never consumed twice), so the result is order-independent for a given loop. No
tolerance is widened: `mergeLen` is a SEPARATE caller-supplied bound above `tolerance`, and
the primary weld still runs at `tolerance`. No new `UnhealedReason` is added — a short edge
outside the bound stays the honest landed decline (`OpenShell` / `GapBeyondTolerance`).

## Oracle and the equal-or-more-conservative contract

Host gate: the defected cube is analytically still a unit cube; collapsing the collinear
micro-edge restores `V = 1.0` and watertightness with no re-approximation (closed form).

SIM gate: OCCT `BRepBuilderAPI_Sewing` at a tolerance `≥ seg` merges the split verts via
its own small-edge handling and `ShapeFix` closes the solid — the OCCT analogue. Native
collapse matches it (`V ≈ 1`). With the flag OFF native declines; OCCT is observed to
close the collinear split even at a nominal tolerance `< seg` (its sewer computes its own
effective tolerance — it is AGGRESSIVE). Native DEFERRING where OCCT aggressively repairs
is native being MORE CONSERVATIVE — the intended contract — not a failure: native never
emits a different watertight solid than the truth, and the opt-in flag recovers exactly the
same win when the caller asks for it.

## Alternatives considered

- **Merge-to-midpoint** (keep one interior vertex): rejected — leaves the run unshared (see
  above).
- **Widen the weld tolerance to `seg`**: rejected — violates the non-negotiable "never widen
  the weld tolerance"; would also merge unrelated geometry within `seg`.
- **A cross-face variant of `gap_bridge`**: rejected — the defect is within one face; a
  cross-face pass is the wrong tool and risks folding a face.

## Deferred (asymptotic tail → OCCT `ShapeFix`)

Short edge that turns a real (non-collinear) corner; a collapse needing the neighbour face
re-projected; pcurve reconstruction; self-intersecting-wire repair; arbitrary broken
industrial B-rep. All declined honestly with the input unchanged.
