# Tasks — moat-a2-ssi-slab-prune

## 1. Diagnose (host)
- [x] 1.1 Reproduce the pathology: disjoint dish pair at dz = 1e-3 gives 1 835 481 candidates in
  533 s with `converged = 0`, `offBranch = 1 835 481`, `seeds = 0` — the pair does not intersect.
- [x] 1.2 Establish that `refineRegion` runs once per CANDIDATE, not once per branch (205–227 µs
  each, ≈ 65 % of wall); correct the false comment at `seeding.cpp:900`.
- [x] 1.3 Rule out the AABB prefilter: `subdivide` already runs `aabbDisjoint` before emitting, so
  it is a provable no-op.
- [x] 1.4 Rule out `patchGapBound`: it is a sound UPPER bound certifying agreement, where skipping a
  crossing-free subtree needs a LOWER bound. Wrong direction, not a tuning problem.
- [x] 1.5 Disqualify the refine SITE: `refineRegion` clamps into the full domain, 97.8 % of accepted
  refines converge outside their own candidate box, and filtering there drops 25 324 / 85 678 /
  49 284 seeds on the transversal controls.

## 2. Implement (src/native/ssi)
- [x] 2.1 `slabSeparated` in `patch_gap.h` — exact de Casteljau sub-nets projected onto one
  direction; strict `>` against `gap`, mirroring `aabbDisjoint`.
- [x] 2.2 Normalize `n` inside the predicate rather than trusting the caller; refuse (return false)
  on empty nets or a degenerate direction.
- [x] 2.3 Call site in `subdivide` immediately after `aabbDisjoint`, gated on
  `bothFreeform && A.hasBezierNet && B.hasBezierNet`; direction = A's midpoint normal.
- [x] 2.4 Write the CONTAINMENT argument in the comment, explicitly NOT an argument from precedent —
  the precedent form would equally license the refine-site filter rejected in 1.5.

## 3. Gate A — host self-consistency
- [x] 3.1 NEW `slab_projection_contains_the_surface_over_the_subbox` — the hull-containment lemma
  the prune rests on, over random nets, sub-boxes and directions. 27 040 surface samples,
  0 outside the sub-net hull.
- [x] 3.2 NEW `slab_separation_is_never_claimed_without_a_real_gap` — a claimed separation must
  survive dense sampling of both pieces. 51 separations claimed, 0 refuted.
  **Anti-vacuity:** asserts `fired > 20`; this guard caught the first version of the test, which
  exercised the predicate exactly ONCE because full-domain random boxes are too large for these
  close pairs' hulls to separate. Cells are now bounded to 2–12 % of the domain, the depths the
  prune actually runs at.
- [x] 3.3 NEW `slab_prunes_the_tilted_pair_the_aabb_test_cannot` — the motivating pose; also asserts
  a non-unit direction at 1000× length does NOT scale into a false separation.
- [x] 3.4 NEW `slab_refuses_what_it_cannot_prove` — empty nets, degenerate direction; and that
  UNEQUAL degrees are fine here, unlike `patchGapBound`.
- [x] 3.5 NEW regression case `seed_slab_prune_kills_the_near_parallel_descent_without_losing_a_crossing`
  in `test_native_ssi_seeding.cpp` — pins the target pose AND transversal controls whose seed and
  branch counts must not move. The second half is the load-bearing half: an over-aggressive prune
  presents as a FASTER run with FEWER seeds, which reads as an improvement.
- [x] 3.6 Suites green: marching 26/0, s4_classification 22/0, seeding 16/16, ssi 11/11, s4f 7/7,
  patch_gap 10/10.

## 4. Corpus differential — the property that matters
- [x] 4.1 Build a SAVED BASELINE ARCHIVE (same binary, gate forced false) and verify each archive
  against a known discriminator before comparing, so a mislabelled build cannot be mistaken for a
  measurement.
- [x] 4.2 Co-resident multi-loop family, 34 poses: seeds, branches and coincidence verdicts
  **identical**; candidates fall up to 35× (carton × carton ph=0.10, 482 292 → 13 935).
- [x] 4.3 Bench, 11 poses: same — identical verdicts, target pose 1 835 481 → 0 and 533 s → 0.056 s.
- [x] 4.4 Record that the `wave × wave` sweep was TRIMMED to the two poses above the firing
  threshold, so its 1 891 467 → 0 is not evidence about the sub-3e-4 band.

## 5. Gate B — native-vs-OCCT parity
- [x] 5.1 Host parity `native_ssi_marching_parity` green.
- [ ] 5.2 Run `run-sim-native-ssi-marching` on the booted simulator. **(macOS-only — left for the
  Mac.)** Expect MORE movement than the one benign line originally predicted: this changes candidate
  sets on TRANSVERSAL poses too, not only disjoint ones. A line that moves while staying orders
  under tolerance is expected; a changed SEED or BRANCH count is not, and would be the real signal.

## 6. Structural + finalize
- [x] 6.1 Diff confined to `ssi/patch_gap.h` + `ssi/seeding.cpp`; OCCT-free; `cc_*` unchanged; no
  tolerance widened; no node fabricated; elementary / rational / multi-span operands byte-identical.
- [x] 6.2 Update `openspec/MOAT-ROADMAP.md` §M1 A2 with the measured table and the `κh²/2 < dz`
  limit.
- [x] 6.3 `openspec validate --strict moat-a2-ssi-slab-prune`.
