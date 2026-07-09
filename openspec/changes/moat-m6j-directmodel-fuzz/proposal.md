# Proposal — moat-m6j-directmodel-fuzz (MOAT M6-breadth-10, the TENTH domain)

## Why

The MOAT completeness bar (dropping OCCT is gated by *proven* native correctness) has
nine landed differential-fuzzing domains — curved booleans (`native_boolean_fuzz.mm`),
STEP round-trip (`native_step_import_fuzz.mm`), construction/loft/sweep
(`native_construct_fuzz.mm`), blends (`native_blend_fuzz.mm`), wrap/emboss
(`native_wrap_emboss_fuzz.mm`), mesh mass-properties (`native_mass_props_fuzz.mm`),
geometry services (`native_geometry_services_fuzz.mm`), rigid/similarity transforms
(`native_transform_fuzz.mm`), and reference/datum geometry
(`native_reference_geometry_fuzz.mm`). The **DIRECT-MODELING layer** — the native path
behind the CyberCad app's direct-edit tools (`cc_split_plane`, `cc_replace_face`
parallel cap offset, `cc_project_point_on_face`) — landed as M-DM with CURATED per-op
parity harnesses (`native_split_plane_parity.mm`, `native_replace_face_parity.mm`,
`native_dm3_dm4_parity.mm`) but has **no seeded differential fuzzer** that drives
*random valid solids* through those ops under BOTH engines and classifies every trial.

A direct-model op that silently produces a wrong-volume solid (a plane split keeping the
wrong material, a cap offset of the wrong amount) or a wrong foot-of-perpendicular is a
**silent wrong edit the user's downstream model is built on**, and nothing today searches
the input space for one. This change extends the completeness bar to that TENTH native
domain. Among the roadmap's REMAINING candidate domains (direct-modeling, section-curve,
HLR, boolean-of-transformed-operands), **direct-modeling** is the highest-value gap:

- **Highest-value gap:** the roadmap M6 REMAINING list explicitly names "direct-modeling
  / section curves"; the three DM ops each have *per-op* parity but no *fuzz* domain —
  exactly the "no fuzz domain yet" the roadmap flags.
- **Drives the SHIPPING PATH under both engines.** Unlike the eight internal-C++ fuzzers
  (which call the native C++ layer directly and use an OCCT oracle slice), this harness —
  like `native_hlr_parity.mm` — drives the public `cc_*` facade under BOTH
  `cc_set_engine(1)` (NativeEngine, the OCCT-free DM core) and `cc_set_engine(0)` (OCCT
  adapter, the oracle), so it verifies the exact ABI + engine-toggle path the app uses.
- **Pristine closed-form arbiter for each op.** Each DM op on an ANALYTIC primitive has
  an EXACT closed form: an axis-aligned plane split of a box/prism has an exact
  keep-volume; a parallel cap offset of a constant-cross-section solid changes volume by
  exactly `capArea·offset`; a point projected onto a planar or cylindrical face has an
  exact foot. This THIRD, engine-independent ground truth attributes a native-vs-OCCT gap
  instead of reflexively blaming either engine — and it SURFACES an OCCT-facade
  limitation the curated parity tests hide (see below).

The native direct-modeling services are verified in source to be OCCT-free: split_plane
/ replace_face compose `src/native/boolean/split_plane.h` (`freeformHalfSpaceCut` — the
seam trace requires the `CYBERCAD_HAS_NUMSCI` substrate, so the harness links it), and
`src/native/directmodel/project.h` serves plane/cylinder/sphere faces in closed form and
honestly declines cone-lateral / freeform / ambiguous poses.

## What Changes

1. **A new direct-modeling differential fuzzer** `tests/sim/native_directmodel_fuzz.mm`
   (iOS-simulator, whole kernel + OCCT linked, under `-DCYBERCAD_HAS_NUMSCI`), reusing
   the landed harness machinery (splitmix64/xoshiro256** RNG, coverage tally). Per trial
   it:
   - **deterministically generates** a random-but-VALID base solid (`BOX` / `NGON`
     prism, `CYLINDER`, `CONE` frustum) built IDENTICALLY under both engines via
     `cc_solid_extrude` / `cc_solid_revolve`, and one random direct-model op, via an RNG
     keyed ONLY by an explicit `FUZZ_SEED` (argv/env, fixed default) — NO clock, NO
     `rand()`; same seed → byte-identical batch.
   - **runs the op under BOTH engines** (`cc_set_engine(0)` OCCT oracle, `cc_set_engine(1)`
     NativeEngine) and compares the two results by `cc_mass_properties` / `cc_bounding_box`
     (a shape is ALWAYS measured under the engine that built it), AND against a THIRD
     engine-independent CLOSED-FORM arbiter in plain fp64:
     - **SPLIT** (`cc_split_plane`, axis-aligned OR oblique, random keep side): native vs
       OCCT keep-side volume/area/bbox; an exact half-space keep-volume for an
       axis-aligned box/prism; and a PARTITION-CLOSURE `V(keep+) + V(keep−) == V(whole)`
       arbiter (native cut both ways) for every family.
     - **OFFSET** (`cc_replace_face` of a planar cap by a signed distance): native vs OCCT
       result volume/area/bbox; an exact `ΔV == capArea·offset` arbiter for the
       constant-cross-section families (BOX/NGON/CYLINDER).
     - **PROJECT** (`cc_project_point_on_face` of a random exterior point): native vs OCCT
       foot + distance; an exact planar-face / cylinder-radial foot arbiter.
   - **classifies** each op trial into EXACTLY ONE of AGREED / HONESTLY-DECLINED /
     DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED / ORACLE_UNRELIABLE at a FIXED tight
     tolerance (never widened). Prints a per-family / per-op coverage summary; exits 0 IFF
     the bar holds. Any DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE prints seed +
     case index + base descriptor.
2. **A runner** `scripts/run-sim-native-directmodel-fuzz.sh` mirroring
   `run-sim-native-hlr.sh` (whole kernel + OCCT) and `run-sim-native-split-plane.sh` (the
   numsci substrate the native split path needs). It runs ≥2 seeds by default and fails
   if any seed fails; the new `.mm` is added to the `run-sim-suite.sh` SKIP list.
3. **Nothing in `src/native/**` or `src/engine/**` changes.** The native direct-modeling
   + `cc_*` facade path is the SYSTEM UNDER TEST and stays byte-unchanged; the `cc_*` ABI
   is unchanged. If the fuzzer surfaces a real native disagreement it is reported with its
   seed — not silenced.

## Capabilities

### Modified Capabilities

- `native-verification`: ADDS the tenth differential-fuzzing domain — a direct-modeling
  harness that drives random valid solids through the native `cc_*` direct-edit ops
  (`cc_split_plane` / `cc_replace_face` cap offset / `cc_project_point_on_face`) under
  BOTH engines via `cc_set_engine`, arbitrated by a THIRD engine-independent closed-form
  ground truth (exact half-space keep-volume + partition closure; `capArea·offset`
  volume; planar/cylindrical foot) at a fixed tolerance, with the native DM slice's
  scoped declines (cone-lateral projection; cone cap offset; out-of-scope split configs)
  as first-class honest scope.

## Impact

- `tests/sim/native_directmodel_fuzz.mm` — NEW test harness (infrastructure).
- `scripts/run-sim-native-directmodel-fuzz.sh` — NEW runner.
- `scripts/run-sim-suite.sh` — the new `.mm` added to the SKIP list (one line).
- `openspec/MOAT-ROADMAP.md` — M6 breadth row updated (×9 → ×10).
- **Zero production-code change.** `src/native/**`, `src/engine/**`, `include/**`, and the
  `cc_*` ABI stay BYTE-UNCHANGED. No tolerance is weakened; no result is silently capped
  or dropped.
- **A surfaced OCCT-facade limitation (documented, not faked).** `cc_replace_face`'s OCCT
  adapter implements a cap move as a half-space `BRepAlgoAPI_Cut` — it can only TRIM
  (offset < 0), never GROW (offset > 0): a grow leaves the OCCT solid un-grown while the
  NativeEngine genuinely extends it and matches the exact `capArea·offset` math. The
  harness therefore generates BOTH signs — a TRIM is an AGREE (native == OCCT == math);
  a GROW is classified ORACLE-INACCURATE (native vindicated by exact math, OCCT the
  outlier), NEVER a bar failure. The curated `native_dm3_dm4_parity.mm` hides this by
  reconstructing an over-tall OCCT box; the fuzzer, driving the raw facade, exposes it.
- **Out of scope / declined (documented, not faked):** cone-lateral `cc_project_point_on_face`
  is a FIRST-CLASS native decline (the native slice serves plane/cylinder/sphere; a cone
  lateral declines → OCCT is the fallback oracle, counted HONESTLY-DECLINED). A cone cap
  `cc_replace_face` and out-of-scope `cc_split_plane` configurations that the NativeEngine
  declines route to HONESTLY-DECLINED (native NULL → OCCT), NEVER a DISAGREE. SCALE /
  MIRROR bases, non-planar cap offsets, and tilted (non-parallel) face replacements are
  out of scope for this first DM slice (an HONEST SCOPE choice).
