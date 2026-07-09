# Proposal — moat-m6k-transformed-boolean-fuzz (MOAT M6-breadth-11, the ELEVENTH domain)

## Why

The MOAT completeness bar (dropping OCCT is gated by *proven* native correctness) has
ten landed differential-fuzzing domains — curved booleans (`native_boolean_fuzz.mm`),
STEP round-trip (`native_step_import_fuzz.mm`), construction/loft/sweep
(`native_construct_fuzz.mm`), blends (`native_blend_fuzz.mm`), wrap/emboss
(`native_wrap_emboss_fuzz.mm`), mesh mass-properties (`native_mass_props_fuzz.mm`),
geometry services (`native_geometry_services_fuzz.mm`), rigid/similarity transforms
(`native_transform_fuzz.mm`), reference/datum geometry
(`native_reference_geometry_fuzz.mm`), and direct-modeling
(`native_directmodel_fuzz.mm`). Every one of these fuzzes a SINGLE domain in isolation.

The roadmap's M6 REMAINING candidate list explicitly names **boolean-of-transformed-
operands** as a domain that "single-domain fuzzers miss": a bug in the INTERACTION
between a transform and a boolean. This change closes exactly that gap — the ELEVENTH
native domain — and is the highest-value remaining pick, for three reasons:

- **It is an INTERACTION no landed fuzzer can reach.** `native_boolean_fuzz` booleans
  operands built AXIS-ALIGNED in canonical world frames (identity `Location`).
  `native_transform_fuzz` transforms a SINGLE solid and measures it. NEITHER composes the
  two. But the native planar BSP-CSG boolean (`NativeEngine::boolean_op` →
  `cybercad::native::boolean::boolean_solid`) receives each operand's `topology::Shape`
  WITH whatever solid-level `Location` a transform baked onto it (`cc_translate_shape` /
  `cc_rotate_shape_about` / `cc_mirror_shape` produce a `Shape::located(math::Transform)`).
  The polygon extraction the BSP consumes (`src/native/boolean/polygon.h`) must FOLD that
  solid-level `Location` into every face's world polygon AND world normal. A `Location`
  the extractor drops, mis-composes, or mis-orients (e.g. a MIRROR's handedness flip not
  propagated to the outward normal) yields a boolean on the WRONG operand geometry — a
  silent-wrong result the single-domain fuzzers structurally cannot surface.

- **It drives the SHIPPING PATH under both engines.** Like `native_directmodel_fuzz` /
  `native_hlr_parity`, this harness drives the public `cc_*` facade under BOTH
  `cc_set_engine(1)` (NativeEngine, the OCCT-free planar BSP-CSG) and `cc_set_engine(0)`
  (OCCT adapter `BRepAlgoAPI_{Fuse,Cut,Common}`, the oracle), verifying the exact ABI +
  engine-toggle path the app's move/rotate/mirror-then-boolean workflows use.

- **It has a PRISTINE, engine-independent closed-form arbiter.** A rigid transform `T`
  commutes with a boolean and preserves volume + area: `T(A) ∘ T(B) == T(A ∘ B)`, so the
  TRANSFORMED-boolean's enclosed volume must EXACTLY equal the untransformed boolean's
  volume, `|RT|_vol == |R0|_vol`, with NO oracle needed. Because every operand family is
  an all-PLANAR prism (the native boolean's exact domain), the native mesh is exact and
  this invariant holds to machine epsilon — a large-margin separator between a correct
  composition and any real `Location` mis-handling.

The native planar boolean is verified in source to be OCCT-free
(`src/native/boolean/*`, a clean-room BSP-CSG that self-verifies watertightness + the
set-algebra volume and DISCARDS any result that fails). It is native ONLY for all-planar
polyhedra, so all operand families here are planar prisms — keeping the native path
EXERCISED, not declined. (The shared `NativeEngine` TU pulls in NUMSCI-guarded freeform
paths, so the harness links the substrate even though the planar BSP itself does not need
it.)

## What Changes

1. **A new transformed-boolean differential fuzzer**
   `tests/sim/native_transformed_boolean_fuzz.mm` (iOS-simulator, whole kernel + OCCT
   linked, under `-DCYBERCAD_HAS_NUMSCI`), reusing the landed harness machinery
   (splitmix64/xoshiro256** RNG, coverage tally). Per trial it:
   - **deterministically generates** two VALID all-planar operands (a `BOX`, an `NGON`
     prism, or a concave `LSHAPE` prism) positioned for a clean transversal overlap, one
     boolean op (`FUSE` / `CUT` / `COMMON`), and one random rigid transform `T`
     (`IDENTITY` / `TRANSLATE` / `ROTATE`-about-axis / `MIRROR`-through-plane), via an RNG
     keyed ONLY by an explicit `FUZZ_SEED` (argv/env, fixed default) — NO clock, NO
     `rand()`; same seed → byte-identical batch. The first `T_COUNT` trials force each
     transform kind for coverage.
   - **runs the composed op under BOTH engines** — the baseline `R0 = boolean(A, B, op)`
     and the transformed `RT = boolean(T(A), T(B), op)` — and compares by
     `cc_mass_properties` (a shape is ALWAYS measured under the engine that built it),
     against a THIRD engine-independent CLOSED-FORM arbiter in plain fp64:
     - **PRIMARY (rigid invariant)** — `|RT|_vol == |R0|_vol` to a TIGHT tolerance (a
       rigid `T` preserves volume; boolean commutes with `T`), plus the area invariant to
       a meshed-facet-weld bound. This is the true correctness signal.
     - **SECONDARY (engine agreement)** — native `RT` vs OCCT `RT` volume + area at the
       fixed relative tolerance `native_boolean_fuzz` proved (`2e-2`).
   - **classifies** each trial into EXACTLY ONE of AGREED / HONESTLY-DECLINED (native
     planar BSP scopes out a near-tangent/degenerate composed op → OCCT ships) /
     BOTH-DECLINED (empty `COMMON` etc.) / ORACLE_UNRELIABLE (OCCT breaks its OWN rigid
     invariant while native upholds it — native vindicated, gated off) / DISAGREED (a real
     finding). Prints a per-family / per-op / per-transform coverage summary; exits 0 IFF
     the bar holds. Any DISAGREED / ORACLE_UNRELIABLE prints seed + case index + full
     operand + transform descriptor.
2. **A runner** `scripts/run-sim-native-transformed-boolean-fuzz.sh` mirroring
   `run-sim-native-directmodel-fuzz.sh` (whole kernel + OCCT + the numsci substrate the
   shared `NativeEngine` TU needs). It runs ≥2 seeds by default and fails if any seed
   fails; the new `.mm` is added to the `run-sim-suite.sh` SKIP list.
3. **Nothing in `src/native/**` or `src/engine/**` changes.** The native
   transform + boolean + `cc_*` facade path is the SYSTEM UNDER TEST and stays
   byte-unchanged; the `cc_*` ABI is unchanged. If the fuzzer surfaces a real native
   disagreement it is reported with its seed — not silenced.

## Capabilities

### Modified Capabilities

- `native-verification`: ADDS the eleventh differential-fuzzing domain — a
  transformed-boolean harness that composes a rigid transform WITH a boolean (random
  rigid-placed all-planar operands, `FUSE`/`CUT`/`COMMON`, under BOTH engines via
  `cc_set_engine`) to catch `Location`-composition bugs the single-domain boolean and
  transform fuzzers structurally cannot reach, arbitrated by a THIRD engine-independent
  closed-form rigid invariant (`|T(A)∘T(B)|_vol == |A∘B|_vol`, exact for planar operands)
  plus native-vs-OCCT engine agreement at the sibling-proven tolerance, with the native
  planar BSP's scoped near-tangent/degenerate declines as first-class honest scope.

## Impact

- `tests/sim/native_transformed_boolean_fuzz.mm` — NEW test harness (infrastructure).
- `scripts/run-sim-native-transformed-boolean-fuzz.sh` — NEW runner.
- `scripts/run-sim-suite.sh` — the new `.mm` added to the SKIP list (one line).
- `openspec/MOAT-ROADMAP.md` — M6 breadth row updated (×10 → ×11).
- **Zero production-code change.** `src/native/**`, `src/engine/**`, `include/**`, and the
  `cc_*` ABI stay BYTE-UNCHANGED. No tolerance is weakened; no result is silently capped
  or dropped. The PRIMARY closed-form VOLUME invariant is held tight (machine epsilon);
  the native-vs-OCCT SECONDARY bound reuses `native_boolean_fuzz`'s proven `2e-2` and is
  never widened to launder a real disagreement.
- **Verdict:** across ≥2 seeds at N=96 (>60), DISAGREED = 0 and ORACLE_UNRELIABLE = 0,
  with full family / op / transform coverage — the native transform+boolean composition
  upholds the rigid volume invariant to ≤1e-15 on every trial and agrees with OCCT well
  inside `2e-2`.
- **Out of scope / declined (documented, not faked):** curved operands (cylinder / sphere
  / cone) are OUT of the native planar boolean domain and OUT of this harness (they would
  fall through to OCCT and never exercise the native path). NON-rigid transforms (SCALE)
  are out of scope for the rigid-invariant arbiter (a uniform scale changes volume by
  `s³`, breaking `|RT|==|R0|`); this slice fuzzes the rigid group (translate / rotate /
  mirror) where the closed-form invariant is exact. Near-tangent / degenerate composed
  ops the native planar BSP self-verify rejects route to HONESTLY-DECLINED (native NULL →
  OCCT), NEVER a DISAGREE.
