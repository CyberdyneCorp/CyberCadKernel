# Proposal — moat-a2-ssi-slab-prune (MOAT M1 A2: an oriented separating-slab prune for near-parallel descents)

## Why

`subdivide` (`ssi/seeding.cpp`) prunes a candidate box pair with an AXIS-ALIGNED test,
`aabbDisjoint`. On a near-parallel pair whose separation runs along no coordinate axis, that test
can never fire: the two AABBs overlap at every depth, the descent enumerates the whole 4D box
product, and every leaf is handed to `refineRegion`.

Measured on a disjoint bicubic dish pair at dz = 1e-3: **1 835 481 candidates in 533 s**, and all of
it waste — `converged = 0`, `offBranch = 1 835 481`, `seeds = 0`. The pair does not intersect
anywhere. `refineRegion` runs **once per candidate region**, not once per branch (the comment
claiming otherwise is corrected at `seeding.cpp:900`), at 205–227 µs each, ≈ 65 % of wall with
`least_squares` 99.9 % of that.

The A2 coincidence certificate (`patchGapBound` + root precondition) already collapsed the
COINCIDENT case, 1 835 481 → 144 candidates. It does nothing here, and cannot: it is a sound UPPER
bound certifying agreement, whereas skipping a crossing-free subtree needs a LOWER bound. The
expensive band is `dz ∈ [0, ~2e-2]` while the coincidence-detectable band is `dz ≤ 2.9e-7` — five
orders of magnitude that are provably not coincident.

**Two obvious prefilters are dead and must not be retried.** The AABB test is a provable no-op —
`subdivide` already runs `aabbDisjoint` before emitting. `patchGapBound` is the wrong direction, as
above.

**The same predicate is DISQUALIFIED at the refine site.** `refineRegion` clamps into the FULL
domain, so it is effectively a global solve: **97.8 % of accepted refines converge outside their own
candidate box**. Filtering there drops 25 324 / 85 678 / 49 284 seeds on the transversal controls.
The prune is sound inside `subdivide` because a descendant's param boxes are contained in its
parent's, and for no other reason.

## What

- **`slabSeparated` (`ssi/patch_gap.h`).** An oriented separating-slab witness. Take the EXACT
  de Casteljau sub-nets over the two param sub-boxes, project both onto a unit direction `n`, and
  prune when the projected intervals are separated by more than `gap` — mirroring `aabbDisjoint`'s
  slack and its strict `>`, so a pair separated by exactly `gap` stays a candidate.

- **Call site (`ssi/seeding.cpp`).** Immediately after the existing `aabbDisjoint` test in
  `subdivide`, gated on `bothFreeform && A.hasBezierNet && B.hasBezierNet` — the same eligibility as
  the coincidence certificate, WITHOUT its root precondition, which is specific to the coincidence
  direction. An elementary, rational or multi-span operand keeps `hasBezierNet == false` and takes
  the byte-identical unchanged path.

- **Direction.** A's midpoint normal, evaluated per node. Soundness does NOT depend on this choice —
  a poor `n` merely fails to separate — so it is a heuristic for REACH only. `n` is normalized
  inside the predicate rather than trusted from the caller, and a degenerate direction returns false.

## Soundness

By the convex-hull property, `S_A(boxA)` lies inside the hull of its sub-net and likewise for B.
Projection onto `n` is linear, so it maps each hull into the closed interval spanned by that net's
projected poles. Two separated intervals therefore PROVE that every point of `S_A(boxA)` is further
than `gap` from every point of `S_B(boxB)` — measured along `n`, hence a fortiori in 3D. Descendant
param boxes are contained in the parent's, so a parent-level proof kills only crossing-free subtrees.

Unlike `patchGapBound` this does NOT require equal degrees: each hull bounds its own surface, so no
correspondence between the two nets is involved.

The argument in the code is CONTAINMENT, deliberately not an argument from precedent —
"box-locality is already trusted inside subdivide" would equally license the refine-site filter
that is correctly rejected above.

## Why the midpoint normal, and the limit that follows from it

For a vertically offset pair, projecting on **z** requires the cell's z-extent to fall below `dz` —
FIRST order in cell size, so it never fires at reachable depths. Projecting on the **surface normal**
makes the patch's own extent SECOND order (≈ κh²/2) while the offset still contributes `dz·(n·ẑ)` in
full. At the shipped 1/64 leaf that is ≈ 3e-4 against dz = 1e-3, which is exactly why the target
pose collapses.

The corollary is the honest limit: **the prune fires only while `κh²/2 < dz`.** Below ~3e-4 on these
operands no reachable cell size separates and the pair falls back to the full 4D product. This
BOUNDS the near-parallel band on cost; it does not close it.

## Impact

- `src/native/ssi/` — `patch_gap.h` (+ `slabSeparated`) and `seeding.cpp` (call site + gate) only.
  OCCT-free, substrate-free, `cc_*` unchanged. No tolerance widened, no node fabricated.

| pose | candidates | wall | seeds |
|---|---|---|---|
| disjoint dz=1e-3 (target) | 1 835 481 → **0** | 533 s → **0.056 s** | 0 → 0 |
| disjoint dz=1e-2 | 683 017 → **0** | 149 s → 0.009 s | 0 → 0 |
| dish × tilt s=0.5 | 58 591 → 7 453 | 7.31 → 0.59 s | 1 → 1 |
| dish × plane z=0.30 | 38 128 → 12 767 | 3.55 → 1.01 s | 1 → 1 |
| dish × plane z=0.50 | 26 628 → 11 221 | 2.12 → 0.88 s | 4 → 4 |
| carton × carton ph=0.10 | 482 292 → 13 935 | — | 4 → 4 |
| carton × tilt s=0.30 | 326 439 → 19 558 | — | 1 → 1 |
| coincident dz=0 | 144 → 144 | unchanged (certificate short-circuits) | 0 → 0 |

- **Verified against a SAVED BASELINE ARCHIVE** — the same binary with the gate forced false — not
  against inference: **seeds, branches and coincidence verdicts are identical on all 34 poses of the
  co-resident multi-loop family and all 11 bench poses.** Only the candidate count, which is pure
  cost, moves.

- Gates — host Gate A green across six suites: marching 26/0, s4_classification 22/0, seeding 16/16,
  ssi 11/11, s4f 7/7, patch_gap 10/10.

## Caveat — the one sweep that would have shown the limit was trimmed

The `wave × wave` family sweep was cut from eight poses to two for runtime. The two retained
(dz = 1e-3, 3.2e-3) are precisely the ones ABOVE the `κh²/2 < dz` firing threshold; the six dropped
are where the prune does nothing. Their 1 891 467 → 0 is real but is **NOT evidence about the
sub-3e-4 band**, and the table above should not be read as covering it.
