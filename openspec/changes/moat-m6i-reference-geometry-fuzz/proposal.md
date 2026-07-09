# Proposal — moat-m6i-reference-geometry-fuzz (MOAT M6-breadth-9, the NINTH domain)

## Why

The MOAT completeness bar (dropping OCCT is gated by *proven* native correctness) has
eight landed differential-fuzzing domains — curved booleans (`native_boolean_fuzz.mm`),
STEP round-trip (`native_step_import_fuzz.mm`), construction/loft/sweep
(`native_construct_fuzz.mm`), blends (`native_blend_fuzz.mm`), wrap/emboss
(`native_wrap_emboss_fuzz.mm`), mesh mass-properties (`native_mass_props_fuzz.mm`),
geometry services (`native_geometry_services_fuzz.mm`), and rigid/similarity transforms
(`native_transform_fuzz.mm`). The **REFERENCE / DATUM GEOMETRY + TOPOLOGY-READ layer** —
the native path behind the CyberCad app's datum/reference tools (`cc_face_axis`,
`cc_ref_axis_from_face`, `cc_ref_plane_from_face`, `cc_ref_axis_from_edge`,
`cc_tangent_chain`, `cc_outer_rim_chain`, `cc_offset_face_boundary`) — landed as M-REF
with a CURATED per-op parity harness (`native_reference_parity.mm`) but has **no seeded
differential fuzzer** that drives *random valid solids at random poses* through those
queries and classifies every trial.

A reference query that silently returns a wrong axis, a mis-oriented datum plane, an
edge line pointing the wrong way, or an offset polygon of the wrong area is a **silent
wrong datum the user builds their model on** (a mate, a section, a pattern axis), and
nothing today searches the input space for one. This change extends the completeness
bar to that NINTH native domain.

Among the task's candidate domains (direct-modeling, reference-geometry, section-curve),
**reference-geometry** is the highest-value gap and the cleanest fuzz setup:

- **Highest-value gap:** M-REF just landed (22 app sites, a MUST-GO-NATIVE Class-B
  bucket) with *per-op* parity but no *fuzz* domain — exactly the "no fuzz domain yet"
  the roadmap flags. Direct-modeling ops rely on `CYBERCAD_HAS_NUMSCI` substrate
  (freeform seam trace) and have narrower closed-form coverage; section-curve
  (`BRepAlgoAPI_Section`) has no native `cc_*` producer to fuzz against a native path.
- **Pristine closed-form arbiter:** each reference datum of an ANALYTIC primitive is an
  EXACT closed form (a cylinder/cone axis, a planar cap normal, a straight-edge line, a
  polygon inward-offset area). Under a RIGID pose (rotate + translate, no scale/mirror)
  every datum transforms EXACTLY, so the harness has a THIRD, engine-independent ground
  truth that attributes a native-vs-OCCT gap instead of reflexively blaming either.

The native reference services are **read-only queries over the placed B-rep** (verified
in source): `src/native/reference/reference.h` (OCCT-FREE, header-only) computes every
datum by exact fp64 vector math from the shared-node `topology::Shape` graph, baking the
sub-shape `Location` into the result exactly as `BRep_Tool` bakes `TopLoc_Location`. The
harness builds a random VALID base solid via the OCCT-FREE native builders
(`src/native/construct`), applies a random rigid pose via
`Shape::located(math::Transform)`, and drives every reference op on the posed native
solid — the exact path the app's datum tools exercise.

## What Changes

1. **A new reference-geometry differential fuzzer**
   `tests/sim/native_reference_geometry_fuzz.mm` (iOS-simulator, OCCT linked), reusing
   the landed harness machinery (splitmix64/xoshiro256** RNG, coverage tally, OCCT
   oracle block) from the sibling fuzzers. Per trial it:
   - **deterministically generates** a random-but-VALID base solid (`BOX` / `NGON`
     prism, `CYLINDER`, `CONE` frustum) and a random RIGID pose (rotate about a random
     unit axis + translate) via an RNG keyed ONLY by an explicit `FUZZ_SEED` (argv/env,
     fixed default) — NO clock, NO `rand()`; same seed → byte-identical batch.
   - **drives each reference op three independent ways**: the native `ref::` service on
     the posed native solid (SYSTEM UNDER TEST); the OCCT oracle
     (`gp_Cylinder`/`gp_Cone::Axis`, `gp_Pln`, `gp_Lin`, `BRepOffsetAPI_MakeOffset`,
     `BRepTools::OuterWire`, `BRepAdaptor_Curve::D1` tangent) on the posed OCCT analog;
     and a THIRD engine-independent closed-form analytic arbiter (the known construction
     datum transformed by the SAME pose with plain fp64 affine) as the PRIMARY oracle.
   - **classifies** each op trial into EXACTLY ONE of AGREED / HONESTLY-DECLINED /
     DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED / ORACLE_UNRELIABLE at a FIXED tight
     rigid tolerance (never widened). Prints a per-family / per-op coverage summary;
     exits 0 IFF the bar holds. Any DISAGREE / ORACLE-INACCURATE / ORACLE_UNRELIABLE
     prints seed + case index + base descriptor.
2. **A runner** `scripts/run-sim-native-reference-geometry-fuzz.sh` mirroring
   `run-sim-native-reference.sh` / `run-sim-native-transform-fuzz.sh` (runs ≥2 seeds by
   default, fails if any seed fails), and the new `.mm` added to the `run-sim-suite.sh`
   SKIP list (fuzzers run under their own runner).
3. **Nothing in `src/native/**` or `src/engine/**` changes.** The native reference +
   `located()` path is the SYSTEM UNDER TEST and stays OCCT-free and untouched; the
   `cc_*` ABI is unchanged. If the fuzzer surfaces a real native disagreement it is
   reported with its seed — not silenced.

## Capabilities

### Modified Capabilities

- `native-verification`: ADDS the ninth differential-fuzzing domain — a
  reference/datum-geometry harness that drives random valid solids at random rigid poses
  through the native reference services (`faceAxis` / `refAxisFromFace` /
  `refPlaneFromFace` / `refAxisFromEdge` / `tangentChain` / `outerRimChain` /
  `offsetFaceBoundary`) and the OCCT topology-query oracle, arbitrated by a THIRD
  engine-independent closed-form datum ground truth at a fixed tight rigid tolerance,
  with the reference.h scoped declines (circular-cap offset, freeform edge in a tangent
  walk) as first-class honest scope.

## Impact

- `tests/sim/native_reference_geometry_fuzz.mm` — NEW test harness (infrastructure).
- `scripts/run-sim-native-reference-geometry-fuzz.sh` — NEW runner.
- `scripts/run-sim-suite.sh` — the new `.mm` added to the SKIP list (one line).
- **Zero production-code change.** `src/native/**` stays OCCT-free and untouched;
  `src/engine/**` unchanged; the `cc_*` ABI is unchanged. No tolerance is weakened; no
  result is silently capped or dropped.
- **Out of scope / declined (documented, not faked):** SCALE / MIRROR poses are out of
  scope (a rigid pose keeps every datum an exact transform of the closed form; a
  similarity would still work for axes/normals but complicates the offset-area arbiter —
  the rigid restriction is an HONEST SCOPE choice). A circular cap offset and a freeform
  edge in a tangent walk are FIRST-CLASS declines that reference.h returns and that the
  closed form confirms have no closed-form datum (matched → HONESTLY-DECLINED, never a
  DISAGREE). The circular-cap `outerRimChain` is arbitrated STRUCTURALLY (the rim id set
  equals the native cap face's own outer wire, confirmed as a circular boundary by the
  OCCT circle oracle) because the native periodic revolution cap stores its rim as
  arc edges with periodic seam vertices — a legitimate representational difference from
  OCCT's single seam edge, not a datum defect.
