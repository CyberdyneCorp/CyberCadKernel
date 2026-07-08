# Proposal — moat-mpf-mass-properties-fuzz (MOAT M6-breadth-6, the SIXTH domain)

## Why

The MOAT completeness bar (drop-OCCT is gated by *proven* native correctness) has
five landed differential-fuzzing domains — curved booleans
(`native_boolean_fuzz.mm`), STEP round-trip (`native_step_import_fuzz.mm`),
construction/loft/sweep (`native_construct_fuzz.mm`), blends
(`native_blend_fuzz.mm`), and their shared harness machinery. The **mass-properties
query layer** — the native path behind the CyberCad app's MassReadout / Inertia /
Measure panels (`cc_mass_properties`, `cc_principal_moments`) — has curated parity
coverage (`native_tessellation_parity.mm` checks mesh area/volume against
`BRepGProp`) but **no seeded differential fuzzer** that drives *random valid solids*
through the native mass path and OCCT and classifies every trial. A wrong volume,
area, or centroid on a valid solid would be a **silent wrong answer the user reads
off the screen**, and nothing today searches the input space for one. This change
extends the completeness bar to that SIXTH native domain.

The native mass path is **mesh-based** and its shape is load-bearing for the design
(verified in `src/engine/native/native_engine.cpp`):

- `NativeEngine::mass_properties` tessellates the B-rep at a fixed
  `kPropertyDeflection = 0.005`, then computes `area = surfaceArea(mesh)`,
  `volume = |enclosedVolume(mesh)|` (divergence theorem), and the centroid from the
  same signed-tetra decomposition. `valid = isWatertight(mesh) && volume > 0` — a
  non-watertight mesh yields **no valid mass** (an honest decline), never a wrong
  number.
- `NativeEngine::principal_moments` is `CC_NATIVE_BODY_UNSUPPORTED` for a native
  body: it **delegates to the OCCT fallback**. The native path therefore has **no
  independent inertia answer** — the inertia/principal-moments dimension is a
  first-class HONEST NATIVE DECLINE, not a fuzzable native computation. This
  boundary is documented, not papered over.

Because the native answer is a *mesh discretisation* of an exact solid while OCCT
`BRepGProp` measures the *exact B-rep*, a native-vs-OCCT gap on a **curved** family
(cylinder / sphere / cone) is dominated by the tessellation chord error, not by a
native fault. So native-vs-OCCT alone cannot attribute a disagreement. The harness
therefore carries a **closed-form analytic arbiter** as the PRIMARY correctness
oracle for every family that has one (box / prism / cylinder / cone / sphere exact
volume + area + centroid + inertia; loft prismatoid-band volume; revolution via
Pappus), and the native-mesh-vs-exact tolerance is **matched to the deflection
bound** (the tessellator's own convergence guarantee at `kPropertyDeflection`),
never widened. A planar family (box / prism / straight loft) meshes exactly and is
held to a tight tolerance; a curved family is held to the deflection-derived bound.

## What Changes

1. **A new mass-properties differential fuzzer** `tests/sim/native_mass_props_fuzz.mm`
   (iOS-simulator, OCCT linked), reusing the landed harness machinery and
   analytic-arbiter pattern of `native_construct_fuzz.mm` /
   `native_blend_fuzz.mm`. Per trial it:
   - **deterministically generates** a random-but-VALID solid from the native
     mass families via a splitmix64/xoshiro256** stream keyed ONLY by an explicit
     `FUZZ_SEED` (argv/env, fixed default) — NO clock, NO `rand()`; same seed →
     byte-identical batch. Families: `BOX` (rectangular prism), `NGON_PRISM`
     (regular-n-gon extrude), `CYLINDER`, `CONE`/`FRUSTUM`, `SPHERE` (revolves),
     `LOFT` (coaxial n-gon prismatoid), `REVOLVE` (arbitrary axial polygon → Pappus),
     plus sparse out-of-scope DECLINE-exercisers (a degenerate/self-touching profile)
     that yield no valid mass.
   - **measures the same solid two ways**: the native mass path
     (`cc_mass_properties` / `cc_principal_moments` under the native engine, called
     directly) AND OCCT `BRepGProp::VolumeProperties` + `SurfaceProperties` +
     `GProp_PrincipalProps`; and computes the **closed-form analytic** volume, area,
     centroid, and (where exact) principal moments.
   - **classifies** each trial into EXACTLY ONE of AGREED / HONESTLY-DECLINED /
     DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED at a FIXED, deflection-matched
     tolerance (never widened). Prints a per-family coverage summary; exits 0 IFF
     the bar holds (DISAGREED = 0). Any DISAGREE/ORACLE-INACCURATE prints
     seed + case index + family/param tuple + all measurements.
2. **A runner** `scripts/run-sim-native-mass-props-fuzz.sh` mirroring
   `run-sim-native-construct-fuzz.sh`, and the new `.mm` added to the
   `run-sim-suite.sh` SKIP list (fuzzers run under their own runner, like the other
   four).
3. **Nothing in `src/native/**` or `src/engine/**` changes.** The native
   mass-properties path is the SYSTEM UNDER TEST and stays OCCT-free and untouched;
   the `cc_*` ABI is unchanged. If the fuzzer surfaces a real native disagreement it
   is reported with its seed — not silenced.

## Capabilities

### Modified Capabilities

- `native-verification`: ADDS the sixth differential-fuzzing domain — a
  mass-properties harness that drives random valid solids through the native
  mesh-based mass path and OCCT `BRepGProp`, arbitrated by a closed-form analytic
  ground truth at a deflection-matched fixed tolerance, with the inertia native
  decline and the mesh-vs-exact deflection boundary logged as first-class honest
  scope.

## Impact

- `tests/sim/native_mass_props_fuzz.mm` — NEW test harness (infrastructure).
- `scripts/run-sim-native-mass-props-fuzz.sh` — NEW runner.
- `scripts/run-sim-suite.sh` — the new `.mm` added to the SKIP list (one line).
- **Zero production-code change.** `src/native/**` stays OCCT-free and untouched;
  `src/engine/**` unchanged; the `cc_*` ABI is unchanged. No tolerance is weakened;
  no result is silently capped or dropped.
- **Out of scope / declined (documented, not faked):** native principal
  moments / inertia (the native path delegates to OCCT — no independent native
  answer, so it is an honest decline, not a differential); families with no clean
  closed-form arbiter and no exact OCCT oracle (e.g. a twisted freeform loft is
  covered for volume only via the prismatoid band, with its curved-side inertia left
  declined); any solid whose native mesh is not watertight (no valid mass → honest
  decline).
