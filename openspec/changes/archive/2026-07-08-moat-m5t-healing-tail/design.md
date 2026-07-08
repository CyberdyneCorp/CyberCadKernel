# Design — moat-m5t-healing-tail

## Diagnosis: which tail defect is the bounded slice?

The landed healer declines three next-defect classes. The moat discipline requires
the ONE that is a bounded opt-in slice with a clean 1:1 OCCT `ShapeFix` oracle and a
closed-form host oracle; the other two are the arbitrary-broken-B-rep asymptotic tail
and DECLINE honestly.

| Candidate | Host analytic oracle | 1:1 OCCT oracle | Delta over landed code | Verdict |
| --- | --- | --- | --- | --- |
| Multi-hole planar cap (≥ 2 disjoint simple planar holes) | **Closed-form**: cube missing two opposite faces → `V = 1.0` | **Clean**: one `MakeFace(gp_Pln, wire)` per hole + `ShapeFix` | **Small**: `traceSingleLoop` → `traceAllLoops`; per-loop layers UNCHANGED | **CHOSEN** |
| Non-planar / curved missing-face cap | None (arbitrary curved hole has no closed-form volume) | Blurry (`BRepFill` / `GeomPlate` surface re-approximation) | Large (support-surface fit + freeform patch) | Declined — OCCT moat |
| Self-intersecting-wire repair | None (no closed-form acceptance) | Blurry (reorder/split heuristics) | Large (wire untangling) | Declined — OCCT moat |

### Feasibility spike (over the landed worktree, host, no OCCT)

A throwaway driver reused the landed `extractSoup` / `removeDegenerate` / `sew` /
`boundaryGraph` / `bestFitPlane` / `maxPlaneDeviation` / `isSimplePolygon` /
`makeConsistent` / `assembleSolid` / `verify`, added only a `traceAllLoops`, and
capped a unit cube missing its two Z faces. Result:

```
after sew: faces=4 boundaryEdges=8 residual=0
traced 2 disjoint boundary loop(s)
synthesized 2 cap face(s), worstPlanarityDev=0
after cap re-sew: boundaryEdges=0
self-verify: watertight=1 signedVolume=1
tess: watertight=1 enclosedVolume=1.000000000000 (expect 1.0)
SPIKE PASS: two opposite planar holes capped to V=1.0
```

The slice is reachable with a localized additive change; the self-verify closes the
loop exactly as it does for the single-hole cap.

## Approach

### The `traceAllLoops` generalization (cap_hole.h, additive)

The landed `traceSingleLoop` already requires every boundary vertex to have exactly
two incident boundary edges and traces ONE cycle, rejecting a branching boundary or a
second disjoint loop. The generalization keeps the degree-2 precondition (a branching vertex, degree ≠ 2,
still declines the whole set — the SAME invariant `traceSingleLoop` enforces) and
simply repeats the walk from each unvisited vertex, collecting ALL disjoint cycles.
(MEASURED NOTE: two ADJACENT missing faces on a cube do NOT produce a degree-4 branch —
they orphan the two exclusively-shared corners, so the sew reports a residual gap and
declines at the earlier honest-out with `GapBeyondTolerance`; the degree ≠ 2 guard is
still the correct defensive invariant for a genuinely branching boundary.)

- ANY boundary vertex with degree ≠ 2 ⇒ return empty (decline whole).
- Walk each unvisited component into a closed cycle; a component that does not close
  ⇒ return empty (decline whole).
- Return the vector of cycles (each a simple loop of shared vertex nodes).

`capAllPlanarHoles(sr, tol)` then runs the UNCHANGED per-loop layers on each cycle
(named `capAllPlanarHoles`, not `capPlanarHoles`, to avoid colliding with the existing
`HealOptions.capPlanarHoles` bool):

1. **Planarity within tolerance** (`bestFitPlane` + `maxPlaneDeviation ≤ tol`).
2. **Simple polygon** (`isSimplePolygon` on the loop projected to its best-fit plane).

If EVERY loop passes, it emits one cap `FaceLoop` per loop (corners on the loop's
EXISTING shared vertex nodes + winding-coherent Newell normal). If ANY loop fails —
non-planar or self-intersecting — it declines the WHOLE set (no partial closure). The
landed single-loop `capPlanarHole` and all four detail helpers are byte-for-byte
untouched; `capAllPlanarHoles` is new, additive code that reuses them.

### The heal.cpp branch (additive, opt-in-guarded)

```cpp
if (sr.boundaryEdges > 0 && opts.capMultiplePlanarHoles) {
  const MultiCapResult caps = capAllPlanarHoles(sr, tol);
  if (!caps.declined && !caps.caps.empty()) {
    m.nCappedFaces = static_cast<int>(caps.caps.size());
    m.maxCapPlanarityDev = caps.planarityDev;
    for (const FaceLoop& c : caps.caps) work.push_back(c);
    sr = sew(work, tol);           // UNCHANGED re-sew
    m.nMergedVerts = sr.mergedVerts;
    m.nMergedEdges = sr.mergedEdges;
  }
}
// the LANDED single-hole branch, now guarded so it runs ONLY when multi is off:
if (sr.boundaryEdges > 0 && opts.capPlanarHoles && !opts.capMultiplePlanarHoles) {
  ... // byte-identical to the landed slice for every existing caller
}
```

The landed single-hole branch's guard gains `&& !opts.capMultiplePlanarHoles` so it is
entered ONLY when `capMultiplePlanarHoles == false` — which, being a new default-false
field, is unchanged for every existing caller, preserving byte-identical behavior for
existing caller and every landed test (including `heal_cap_two_holes_declines`). The
rest of `heal.cpp` — the surviving-boundary honest-out, orientation flood-fill, global
outward sign flip, assemble, and mandatory self-verify — is UNCHANGED and remains the
authoritative closure check.

Because `capMultiplePlanarHoles` is a superset of the single-hole cap (it caps one
hole too), a caller may set it alone; the landed `capPlanarHoles` flag stays for the
strictly-single-hole contract already specified.

## Honesty & invariants

- **Tolerance never widened.** Only loops coplanar within `tolerance` are capped;
  non-planar loops decline. The primary weld tolerance is untouched.
- **Self-verify authoritative.** A capped candidate is reported `Healed` ONLY after
  the UNCHANGED `isWatertight` + `enclosedVolume > 0` self-verify; otherwise
  `Unhealed{SelfVerifyFailed}`, input unchanged.
- **No partial closure.** If any loop in the set is out of bound, the WHOLE shell
  declines `Unhealed{OpenShell}` with the input UNCHANGED — never some-holes-capped.
- **Additive-only.** No `cc_*` entry point / signature / POD change; no new
  `UnhealedReason`; the tessellator is not modified; `src/native/**` stays OCCT-free.

## Alternatives considered

- **Reuse `capPlanarHoles` (the existing bool) to cap all holes.** Rejected: it would
  change `heal_cap_two_holes_declines` (a landed byte-identical guarantee). A new
  default-off flag is required.
- **Cap the cappable subset and leave the rest open.** Rejected: that emits a shell
  still carrying boundary edges; self-verify would fail anyway, and a partial rewrite
  muddies the honest-decline contract. Decline the whole set instead.
- **A `maxCappedHoles` integer knob.** Deferred: a boolean is the minimal surface for
  this slice; a count-limited variant can be added later if a caller needs it.
