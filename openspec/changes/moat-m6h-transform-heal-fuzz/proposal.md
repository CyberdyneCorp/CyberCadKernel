# Proposal — moat-m6h-transform-heal-fuzz (MOAT M6-breadth-8, the EIGHTH domain)

## Why

The MOAT completeness bar (dropping OCCT is gated by *proven* native correctness) has
seven landed differential-fuzzing domains — curved booleans
(`native_boolean_fuzz.mm`), STEP round-trip (`native_step_import_fuzz.mm`),
construction/loft/sweep (`native_construct_fuzz.mm`), blends
(`native_blend_fuzz.mm`), mesh mass-properties (`native_mass_props_fuzz.mm`),
wrap/emboss (`native_wrap_emboss_fuzz.mm`), and their shared harness machinery. The
**rigid/similarity TRANSFORM layer** — the native path behind the CyberCad app's
translate / rotate / scale / mirror / place tools (`cc_translate_shape`,
`cc_rotate_shape_about`, `cc_scale_shape`, `cc_mirror_shape`, `cc_place_on_frame`) —
has curated parity coverage but **no seeded differential fuzzer** that drives *random
compositions* of those transforms on random valid solids and classifies every trial.
A transform that silently corrupts a volume, moves a centroid to the wrong place,
drops a face, or fails to flip handedness on a mirror would be a **silent wrong
result the user places into their model**, and nothing today searches the input space
for one. This change extends the completeness bar to that EIGHTH native domain.

Between the two candidate domains — TRANSFORM CHAINS and HEALING — transform chains is
the **cleaner** domain and is the one this change lands. Its correctness has a
*pristine closed-form arbiter*: a chain of translate / rotate / uniform-scale / mirror
is a SIMILARITY, and for any solid the transformed volume, area, and centroid, the
topology count, and the handedness are exact closed forms of the base solid and the
composed map (§ design). Healing (broken-soup repair vs OCCT `ShapeFix`/`Sewing`) has
no closed-form ground truth — its "correct" output is a heuristic agreement, so a
disagreement cannot be attributed without a third oracle. Transform chains gives a
third, engine-independent arbiter for free; healing does not. (Healing already has a
curated parity harness, `native_heal_parity.mm`; a *fuzzer* for it is left as future
work and documented as an honest deferral, not faked.)

The native transform is a **placement** (verified in source):

- `topology::Shape::located(Location{math::Transform})`
  (`src/native/topology/shape.h`) composes the affine onto the shape's `Location`;
  the tessellator (`src/native/tessellate/surface_eval.h`, `edge_mesher.h`)
  **world-places every sample through that Location** and the explorer
  (`src/native/topology/explore.h`) composes the location down the graph so each
  meshed face is world-placed. Meshing the located solid therefore yields the
  transformed geometry — the exact path the app's transform tools exercise.
- `math::Transform` (`src/native/math/transform.h`) is a clean-room affine
  `v' = L·v + t` modelling `gp_Trsf`: rotation (Rodrigues), uniform scale, mirror
  (`det < 0`), composition and inversion in closed form. This is the SYSTEM UNDER
  TEST.

Because a chain of translate / rotate / **uniform**-scale / mirror is a similarity
`L = S·Q` (`Q` orthonormal, `S = Π` uniform-scale factors), the transformed measures
are EXACT closed forms of the base: `volume' = S³·volume`, `area' = S²·area`,
`centroid' = L·centroid + t`, topology counts are INVARIANT, and the signed enclosed
volume's sign equals `sign(base)·(−1)^#mirrors` (a mirror must **flip handedness yet
leave a valid, watertight, positive-|volume| solid**). Uniform scale ONLY keeps the
area closed form exact (an anisotropic scale has no simple closed-form area) — an
HONEST SCOPE choice, and `cc_scale_shape` is uniform by contract.

## What Changes

1. **A new transform-chain differential fuzzer**
   `tests/sim/native_transform_fuzz.mm` (iOS-simulator, OCCT linked), reusing the
   landed harness machinery and analytic-arbiter pattern of `native_mass_props_fuzz.mm`.
   Per trial it:
   - **deterministically generates** a random-but-VALID base solid (`BOX`,
     `NGON_PRISM`, `CYLINDER`, `SPHERE`, coaxial `LOFT`) and a random CHAIN (length
     1–4) of `TRANSLATE` / `ROTATE`(any axis) / `USCALE`(uniform) / `MIRROR`(any
     plane) via a splitmix64/xoshiro256** stream keyed ONLY by an explicit `FUZZ_SEED`
     (argv/env, fixed default) — NO clock, NO `rand()`; same seed → byte-identical
     batch. A sparse tail appends a singular (zero-scale) op as a DECLINE-exerciser.
   - **applies the chain three independent ways**: the native `math::Transform`
     composed and driven through `Shape::located()` + the tessellator (SYSTEM UNDER
     TEST); the OCCT oracle `BRepBuilderAPI_Transform` with the SAME composed
     `gp_Trsf`, measured by `BRepGProp`; and a THIRD engine-independent closed-form
     affine computed with plain fp64 in the harness (the PRIMARY arbiter).
   - **classifies** each trial into EXACTLY ONE of AGREED / HONESTLY-DECLINED /
     DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED at a FIXED, deflection-matched
     tolerance (never widened), asserting volume/area/centroid against the analytic
     similarity image AND topology invariance AND mirror-handedness. Prints a
     per-family / per-op-kind coverage summary; exits 0 IFF the bar holds. Any
     DISAGREE/ORACLE-INACCURATE prints seed + case index + base/chain descriptor +
     all measurements.
2. **A runner** `scripts/run-sim-native-transform-fuzz.sh` mirroring
   `run-sim-native-mass-props-fuzz.sh` (runs ≥2 seeds by default), and the new `.mm`
   added to the `run-sim-suite.sh` SKIP list (fuzzers run under their own runner).
3. **Nothing in `src/native/**` or `src/engine/**` changes.** The native transform +
   tessellate path is the SYSTEM UNDER TEST and stays OCCT-free and untouched; the
   `cc_*` ABI is unchanged. If the fuzzer surfaces a real native disagreement it is
   reported with its seed — not silenced.

## Capabilities

### Modified Capabilities

- `native-verification`: ADDS the eighth differential-fuzzing domain — a
  transform-chain harness that drives random compositions of translate / rotate /
  uniform-scale / mirror through the native `located()` + tessellate path and OCCT
  `BRepBuilderAPI_Transform`, arbitrated by a THIRD engine-independent closed-form
  similarity ground truth at a deflection-matched fixed tolerance, with topology
  invariance, mirror handedness, and the singular-transform decline as first-class
  honest scope.

## Impact

- `tests/sim/native_transform_fuzz.mm` — NEW test harness (infrastructure).
- `scripts/run-sim-native-transform-fuzz.sh` — NEW runner.
- `scripts/run-sim-suite.sh` — the new `.mm` added to the SKIP list (one line).
- **Zero production-code change.** `src/native/**` stays OCCT-free and untouched;
  `src/engine/**` unchanged; the `cc_*` ABI is unchanged. No tolerance is weakened; no
  result is silently capped or dropped.
- **Out of scope / declined (documented, not faked):** the HEALING domain — a
  broken-soup healer has no closed-form ground truth (its correctness is a heuristic
  agreement with OCCT `ShapeFix`/`Sewing`), so a *differential fuzzer* cannot attribute
  a disagreement without a third oracle it does not have; healing keeps its curated
  parity harness (`native_heal_parity.mm`) and a fuzzer for it is a future change.
  ANISOTROPIC (non-uniform) scale is out of scope for the area arbiter (no closed
  form; `cc_scale_shape` is uniform anyway). A singular (zero-scale) transform is an
  honest decline (collapsed solid → no valid mesh), never a wrong result.
