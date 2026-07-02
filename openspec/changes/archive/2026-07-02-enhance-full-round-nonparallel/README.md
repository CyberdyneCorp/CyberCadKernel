# enhance-full-round-nonparallel

Generalize the native rolling-ball full round
(`src/engine/occt/occt_full_round_fillet.cpp`) from the (anti-)PARALLEL planar-wall
special case to the general NON-PARALLEL planar-dihedral case: build the tangent
cylinder from the two neighbour planes directly — axis direction
`normalize(n1 × n2)`, axis located on the interior dihedral bisector at
perpendicular distance `r` from BOTH planes — so the middle strip face is consumed
into a blend that is G1-tangent to both non-parallel neighbours. The parallel path
stays working (it is the special case where `n1 × n2` runs along the strip and the
bisector is the mid-plane). Truly curved (non-planar) neighbours stay out of scope
and keep the valid standard-fillet fallback, recorded deferred with the measured
gap. Additive/internal only — no `cc_*` signature or POD-struct change
(`cc_full_round_fillet` / `cc_full_round_fillet_faces` already exist). Verified by a
new non-parallel-rib check in `tests/sim/checks_full_round_fillet.cpp` on the
iOS simulator via `scripts/run-sim-phase3-suite.sh`; the host stub build stays a
safe no-op.
