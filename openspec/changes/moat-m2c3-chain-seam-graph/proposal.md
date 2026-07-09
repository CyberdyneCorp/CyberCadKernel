# Proposal — moat-m2c3-chain-seam-graph (MOAT M2 blocker #4, ≥3-seam)

## Why

The landed M2-multiseam builder (`src/native/boolean/seam_graph.h`,
`buildSeamGraph`) assembles the inter-solid wall boundary for exactly the CORNER-box
pose: **two** adjacent `B` faces slice operand `A`'s Bézier wall in two arcs that meet
at **one** junction vertex `J`, joined into one bent boundary→`J`→boundary seam. The
M2 roadmap names the next blocker explicitly:

> "General ≥3-seam / branch-point seam graph — beyond the 2-cutting-face corner-clip."

This change lands the first tractable step of that blocker: a box `B` positioned so
that **three** consecutive faces slice `A`'s Bézier wall, producing a CHAIN of three
arcs meeting at **two** interior junctions:

```
    boundary → J1 → J2 → boundary
```

Two of the cutting faces are the PARALLEL end planes (`x = x0`, `x = x1`), each an
iso-`u` arc; the third is the ORTHOGONAL middle plane (`y = y0`), an iso-`v` arc bounded
by BOTH junctions. `B` removes the STRIP `A ∩ {x0 ≤ x ≤ x1, y ≥ y0}`.

This generalizes the landed one-junction builder along the single dimension the roadmap
named — the number of seams — while staying entirely inside the ROBUST planar-cutter
regime the landed multi-face weld already proved watertight (flat box caps, iso-parametric
arcs, ANALYTIC junctions). It does NOT touch the byte-frozen M0 tessellator and does NOT
reintroduce the curved↔curved closed-seam weld fragility that gates blocker #1/#3 (which
remain blocked on a separately-gated topology-aware tessellator weld).

The reachability was MEASURED against the real fixture geometry BEFORE any code was
written: for the box `x ∈ [−0.15, 0.15], y ∈ [0.0, 0.8]`, exactly three faces straddle the
wall; the three traced arcs are iso-parametric (`u = 0.350`, `u = 0.650`, `v = 0.500`); and
the two analytic junctions `(0.35, 0.50) → (−0.15, 0, 0.009)` and `(0.65, 0.50) →
(0.15, 0, 0.009)` land on BOTH their adjacent cutting planes with residual ~5e-13 ≪
weldTol. OCCT remains the oracle: the sim gate grounds every native arc node and both
junctions against OCCT's exact bowl surface, cutting planes, and `BRepAlgoAPI_Section`.

## What Changes

- ADD `src/native/boolean/seam_graph_chain.h` (header-only, OCCT-free): the
  `ChainSeamGraph` value type + `buildChainSeamGraph(A, B, ChainSeamDecline*)`, an
  ADDITIVE SIBLING of `buildSeamGraph`. It (1) detects the three-cutting-face SET and
  the containing faces; (2) traces + iso-classifies each arc (reusing `traceWallSeam` /
  `arcIsoParam` byte-identical); (3) resolves the chain order (two parallel ends + one
  orthogonal middle); (4) computes both analytic junctions `J1 = wall(u(end0), v(mid))`,
  `J2 = wall(u(end1), v(mid))`, verifying each inside the trimmed wall and on both
  adjacent planes; (5) clips + joins the three arcs into one bent
  boundary→`J1`→`J2`→boundary `chainSeam` with `J1`, `J2` as exact interior vertices.
- ADD `tests/native/chain_seam_fixture.h` + `tests/native/test_native_chain_seam.cpp`
  (host GATE a, closed-form): the edge-straddling box fixture + the strip-clip volume
  oracle + the graph-closes proof.
- ADD `tests/sim/native_chain_seam_parity.mm` + `scripts/run-sim-native-chain-seam.sh`
  (sim GATE b, native-vs-OCCT): grounds the native chain graph against OCCT.
- No `include/` change (no `cc_*` ABI surface). No `src/native/**` file edited; the M0
  tessellator, B2 `splitFace`/`splitFaceSmoothTrim`, `buildSeamGraph`, `junction_split.h`
  and `multi_face_weld.h` are byte-identical.

## Scope + honest boundary

This change lands the GRAPH BUILDER and its standalone closed-graph oracle — the direct
analogue of the landed `buildSeamGraph`, which itself landed the graph in isolation and
recorded the WELD as its next blocker. The 2-junction WALL SPLIT + strip weld (a
generalization of `junction_split.h` / `multi_face_weld.h` to two interior valence-3
vertices) is the SHARPENED next blocker, tracked after this enabler. Anything outside the
reachable three-face / two-junction pose is a MEASURED `ChainSeamDecline` (nullopt),
never a fudge, never a partial graph, never a widened tolerance.
