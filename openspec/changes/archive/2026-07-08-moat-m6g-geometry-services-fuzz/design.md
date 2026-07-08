# Design — moat-m6g-geometry-services-fuzz (MOAT M6-breadth-7)

## Context

This extends the landed M6 differential-fuzzing bar (boolean / import /
construction / blend / wrap-emboss / mass-properties) to a SEVENTH native domain:
the geometry-services (GS) analysis/section/drafting layer. OCCT is the ORACLE.
The harness is infrastructure — `src/native/**` stays OCCT-FREE and UNTOUCHED,
`cc_*` is unchanged, and no production behavior changes. The invariant across all
seven domains is identical: seeded deterministic random VALID inputs → BOTH the
native service AND its OCCT oracle → a five-way classification at a FIXED
tolerance, with a closed-form analytic value as the PRIMARY arbiter wherever one
exists, and DISAGREE == 0 as the bar.

## Goals / Non-goals

**Goals.** Cover the reachable GS services (GS3 distance, GS4 curvature, GS2
section, GS1 HLR, GS5 inertia, GS6 validity) with a clean OCCT oracle, INCLUDING
oblique/tilted regimes; prove DISAGREE == 0 across ≥2 seeds with real per-service
coverage; classify each documented native decline as an HONEST-NATIVE-DECLINE, not
a DISAGREE; surface any real native disagreement prominently with its seed.

**Non-goals.** No new geometry capability; no edit to any `src/native/**` service
(including no fix to the `ssi/plane_conics` oblique bug — that is a separate
upstream workflow); no `cc_*` change; no tolerance widening or silent batch cap.

## Per-service oracle + arbiter map

| GS | Native service (OCCT-free)                       | OCCT oracle                         | Closed-form arbiter (primary)                                  | Oblique/tilted regime the fuzzer MUST hit |
|----|--------------------------------------------------|-------------------------------------|---------------------------------------------------------------|-------------------------------------------|
| GS3 distance  | `analysis/distance.h` / `measure_distance` | `BRepExtrema_DistShapeShape`  | point·{point,segment,line,plane,circle}, segment·segment (skew) exact min distance | skew (non-coplanar) segment pairs, offset/tilted line-plane |
| GS4 curvature | `analysis/curvature.h` / `surface_curvature`,`edge_curvature` | `GeomLProp_SLProps` / `BRepLProp_SLProps` | sphere K=1/R², cylinder K=0 H=1/2R, cone, torus; circle κ=1/R | tilted analytic faces + NURBS sampled at interior (u,v) |
| GS2 section   | `section/section.h` / `sectionByPlane`     | `BRepAlgoAPI_Section` + `BRepGProp` | box=rect, cyl⟂=circle πR², cyl‖=rect 2RH, sphere great-circle | OBLIQUE cut planes (the plane_conics exemplar → decline) |
| GS1 HLR       | `drafting/orthographic_hlr.h`              | `HLRBRep_Algo`                | box from isometric corner = 9 visible + 3 hidden segments     | oblique/isometric view directions, not axis-on |
| GS5 inertia   | `analysis/inertia.h`                       | `GProp_PrincipalProps`        | box/prism/cyl/sphere exact tensor + principal moments         | arbitrarily-rotated solids (principal axes off world axes) |
| GS6 validity  | `analysis/validity.h`                      | `BRepCheck_Analyzer::IsValid` | valid vs broken ground truth by construction                  | tilted broken solids (hole / flipped face / self-intersect) |

The closed-form arbiter is authoritative over OCCT when it exists: a native
result is exonerated (ORACLE-INACCURATE) only when it POSITIVELY matches exact
math while OCCT is the outlier. An unreliable oracle is never laundered into a
pass.

## Decisions

### D1 — One harness, per-service sub-generators, shared classifier

A single `.mm` hosts one deterministic RNG stream and one five-way classifier;
each GS service contributes a sub-generator (its family + oblique-regime sampler),
a native evaluator (calls the OCCT-free service directly), an OCCT evaluator, and a
closed-form arbiter. This mirrors the multi-family structure of
`native_mass_props_fuzz.mm` and keeps the AGREE/DECLINE/DISAGREE/ORACLE-INACCURATE/
BOTH-DECLINE semantics identical across services, so the bar reads the same for all
seven M6 domains.

### D2 — Determinism: splitmix64 → xoshiro256**, keyed only by FUZZ_SEED

Same generator discipline as the landed fuzzers: no wall clock, `rand()`, pid,
address, or thread scheduling. Byte-identical batch for a given `FUZZ_SEED` + batch
size on any machine. Each per-service sub-generator draws from the same stream in a
fixed order so a seed reproduces every service's inputs.

### D3 — Fixed, per-service tolerance matched to the source of error, never widened

- **Exact analytic services (GS3 closed-form cells, GS4 analytic charts):** tight
  tolerance (native computes exact closed form; OCCT is exact) — e.g. `1e-9`.
- **Mesh-derived services (GS5 inertia, GS6 validity, GS2 cap area, GS4 on a
  meshed NURBS):** the tolerance is the tessellation deflection-convergence bound
  at the property deflection (`rel ≈ C·deflection/featureSize`), the SAME
  discipline `native_mass_props_fuzz.mm` codifies — matched to the deflection,
  NEVER loosened past it. A planar family is held to the tight exact-meshing
  tolerance; a curved family to its deflection bound.
- **GS1 HLR:** segment visible/hidden classification + total visible/hidden length
  vs OCCT `HLRBRep_Algo`, at a fixed projection tolerance.

No per-service tolerance is a tunable escape hatch; each is derived from the
service's error source and printed in the summary.

### D4 — Oblique regimes are mandatory coverage, and a documented decline is honest

The generator MUST reach the tilted/oblique regime for every service, because
axis-aligned-only sampling is exactly what let the `ssi/plane_conics`
oblique-cylinder bug hide. Where the native service documents a decline on an
oblique sub-domain (GS2 oblique cylinder cut, GS4 singular chart, GS1 curved
silhouette), that trial is classified HONEST-NATIVE-DECLINE — a first-class,
logged outcome, NOT a DISAGREE and NOT skipped. The harness asserts the decline is
HONEST (native returned its typed decline AND the trial genuinely lies in the
documented out-of-scope sub-domain), so a decline can never mask a wrong answer.

### D5 — GS6 validity needs BROKEN inputs, generated deterministically

Unlike the other services, GS6's interesting signal is on INVALID solids. The
generator emits valid solids AND deterministically-broken variants (drop a face →
hole, flip one face's winding → orientation defect, translate a sub-mesh into
itself → self-intersection) with a KNOWN ground-truth verdict, and the classifier
compares native `validity` against BOTH OCCT `BRepCheck_Analyzer` AND the
construction-time ground truth. A coplanar-overlap self-intersection that GS6
marks `selfIntersectionCertified == false` is an HONEST out-of-scope verdict, not
a DISAGREE.

### D6 — Surface real disagreements, do not paper over them

If a trial classifies DISAGREE — native returns a confident answer that the
oracle+closed-form contradict — the harness FAILS the bar and prints the seed,
service, case index, input tuple, and native/OCCT/analytic values so it is
reproducible. A surfaced GS-service bug is the harness doing its job and is
reported prominently (exactly as the workflow would want the `plane_conics`-class
bug reported), never silenced by widening a tolerance or reclassifying as a
decline.

## Risks / trade-offs

- **A service may not be cleanly fuzzable against a clean oracle** (e.g. GS1 HLR
  segment identity is sensitive to OCCT's edge splitting). Mitigation: cover the
  cleanly-fuzzable services with real per-service coverage across ≥2 seeds and
  DOCUMENT any service reduced to a coarse invariant (total visible/hidden length
  rather than per-segment identity) or deferred — a subset with honest coverage is
  the bar, not an all-or-nothing gate.
- **Deflection-matched tolerances could hide a small real fault.** Mitigation: the
  closed-form analytic arbiter (exact for both planar and curved families) is the
  primary correctness oracle, so a native-vs-OCCT gap is attributed to mesh error
  only when native POSITIVELY matches exact math.

## Migration / rollout

Additive test infrastructure only. New `.mm` + new runner; the `.mm` is added to
the `run-sim-suite.sh` SKIP list (run by its dedicated runner, like every other
`native_*_fuzz.mm`). No production code, `cc_*` API, or `src/native/**` service
changes. Nothing to roll back beyond deleting two new files and one SKIP-list line.
