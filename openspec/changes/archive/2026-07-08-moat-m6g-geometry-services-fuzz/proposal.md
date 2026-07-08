# Proposal — moat-m6g-geometry-services-fuzz (MOAT M6-breadth-7)

## Why

The MOAT M6 completeness bar — a deterministic, seeded, native-vs-OCCT
DIFFERENTIAL fuzzer classifying random VALID inputs as AGREE /
honest-native-DECLINE / DISAGREE / ORACLE-INACCURATE / BOTH-DECLINE at a FIXED
(never-widened) tolerance — currently spans SIX native domains: curved booleans
(`native_boolean_fuzz.mm`), STEP round-trip import (`native_step_import_fuzz.mm`),
loft/sweep construction (`native_construct_fuzz.mm`), fillet/chamfer/offset/shell
blends (`native_blend_fuzz.mm`), wrap-emboss pads/pockets (`native_wrap_emboss_fuzz.mm`),
and mesh mass-properties (`native_mass_props_fuzz.mm`).

The **geometry-services (GS) layer** just landed in this worktree is NOT yet under
that bar. These are the OCCT-free read-only analysis/query services the CyberCad
app's Measure / Curvature / Section / Drawing / Inertia / Check panels read:

- **GS3 distance** — `src/native/analysis/distance.h`
  (`NativeEngine::measure_distance`): minimum distance + witness points between two
  B-rep entities. Oracle: OCCT `BRepExtrema_DistShapeShape`.
- **GS4 curvature** — `src/native/analysis/curvature.h`
  (`NativeEngine::surface_curvature` / `edge_curvature`): Gaussian/mean/principal
  surface curvature and edge curvature κ. Oracle: OCCT `GeomLProp_SLProps` /
  `BRepLProp_SLProps`.
- **GS2 section** — `src/native/section/section.h` (`sectionByPlane`): closed
  section loops of a cut plane through a solid. Oracle: OCCT `BRepAlgoAPI_Section`
  (edge length + loop count + closed-ness) and `BRepGProp` (cap area).
- **GS1 HLR** — `src/native/drafting/orthographic_hlr.h`: orthographic hidden-line
  removal over a polyhedral occluder. Oracle: OCCT `HLRBRep_Algo`.
- **GS5 inertia** — `src/native/analysis/inertia.h`: volume, centroid, inertia
  tensor, principal moments/axes of a solid from its M0 mesh. Oracle: OCCT
  `GProp_PrincipalProps`.
- **GS6 validity** — `src/native/analysis/validity.h`: structural-validity report
  (finite / closed-2-manifold / oriented / non-degenerate / no-self-intersection)
  over a solid's M0 mesh. Oracle: OCCT `BRepCheck_Analyzer::IsValid`.

Each service already carries a documented HONEST-DECLINE scope (a non-convergent
freeform witness pair for GS3, a parametric-singularity chart for GS4, an oblique
cylinder cut for GS2, a curved silhouette for GS1, a non-watertight mesh for GS5,
a coplanar-overlap pair for GS6). Without a seeded differential fuzzer that
covers the **oblique / tilted** regimes — not only axis-aligned inputs — a silent
wrong answer in one of these services can ship undetected. This is EXACTLY the
harness that would have caught the `ssi/plane_conics` oblique-cylinder semi-major
bug that GS2 documents and routes around: a fuzzer that sampled tilted cut planes
against a cylinder would have surfaced the inverted `R/|sinθ|` ellipse
immediately. GS2 turns that regime into an HONEST DECLINE today; the fuzzer must
still cover it, prove the decline is honest (not a silently-wrong answer), and
stand ready to re-classify it AGREE the moment the upstream `ssi/` fix lands.

## What Changes

1. **A new iOS-simulator differential-fuzz harness `tests/sim/native_geometry_services_fuzz.mm`**
   (Objective-C++), extending the six landed M6 fuzzers to a SEVENTH domain — the
   GS analysis/section/drafting services. It is TEST INFRASTRUCTURE: it adds NO
   geometry capability, edits NO `src/native/**` (which stays OCCT-FREE), and
   changes NO `cc_*` signature. It drives the OCCT-free native GS services
   DIRECTLY and the OCCT oracle on the same random valid inputs, and classifies
   each trial into the fixed five-way bucket at a fixed tolerance.

2. **A deterministic, explicitly-seeded generator** (splitmix64 seeding a
   xoshiro256** stream, keyed only by an argv/env `FUZZ_SEED`) producing plain
   parameter POD consumed identically by the native service and the OCCT oracle
   builder. It samples per-service inputs across the **recognised** families AND —
   decisively — across **oblique / tilted** regimes: entity pairs in general
   position for GS3, tilted analytic + NURBS faces sampled at interior `(u,v)` for
   GS4, OBLIQUE and axis-aligned cut planes for GS2, random view directions
   (isometric / oblique, not only axis-on) for GS1, arbitrarily-rotated solids for
   GS5, and valid + deliberately-BROKEN (hole / flipped-face / self-intersecting)
   solids for GS6.

3. **A five-way classifier per service at a FIXED tolerance:** AGREE (native and
   OCCT — and the closed-form analytic arbiter where one exists — match within
   tol) / HONEST-NATIVE-DECLINE (native returns its documented typed decline while
   OCCT answers — first-class, logged, not a bar failure) / DISAGREE (native
   returns a confident answer that is WRONG per the oracle/closed-form — the fault
   the harness exists to catch, FAILS the bar) / ORACLE-INACCURATE (native matches
   exact closed-form math while OCCT is the outlier — logged, not a native fault) /
   BOTH-DECLINE (neither engine produced a comparable answer — logged, not a
   failure). No tolerance is EVER widened per-service; a curved/mesh service is
   held to its deflection-matched bound, never to an arbitrary loosened value.

4. **A new runner `scripts/run-sim-native-geometry-services.sh`** (mirroring the
   existing `run-sim-native-*` runners) that boots the simulator, builds and runs
   the harness over at least two distinct seeds, and the new `.mm` is added to the
   `run-sim-suite.sh` SKIP list (like every other `native_*_fuzz.mm`, which are
   run by their dedicated runners, not the aggregate suite).

5. **A `native-verification` spec delta** capturing the GS-services fuzz domain:
   the deterministic generator (incl. oblique/tilted coverage), the dual
   native+OCCT evaluation with a closed-form arbiter, the five-way classifier, the
   oblique-regime coverage requirement (the `plane_conics` exemplar), and the
   coverage-summary / zero-silent-wrong-answer bar across ≥2 seeds.

## Impact

- **New files:** `tests/sim/native_geometry_services_fuzz.mm`,
  `scripts/run-sim-native-geometry-services.sh`.
- **Edited (test infra only):** `scripts/run-sim-suite.sh` (add the new `.mm` to
  the SKIP list). `src/native/**` UNTOUCHED and OCCT-FREE; no `cc_*` change; no
  production behavior change.
- **Spec:** `native-verification` gains the GS-services differential-fuzz
  requirements (ADDED; siblings of the existing boolean / import / construction /
  blend / wrap-emboss / mass-properties fuzz requirements).

## Honest scope (declines are first-class, logged, never bar failures)

- **GS2 section — OBLIQUE cylinder cut:** GS2 DECLINES it because the upstream
  `ssi/plane_conics.h` `intersectPlaneCylinder` returns an inverted oblique-ellipse
  semi-major (`R/|sinθ|` instead of `R/|cosθ|`). The fuzzer COVERS this regime and
  classifies it HONEST-NATIVE-DECLINE (native declines; OCCT answers) — NOT a
  DISAGREE, and NOT skipped. If a future `ssi/` fix makes GS2 answer, the same
  trial re-classifies AGREE with no harness change.
- **GS4 curvature — parametric singularity:** a sphere pole / cone apex chart or a
  cusp curve point where native honestly declines (`EG−F² ≤ ε`).
- **GS3 distance — non-convergent freeform witness pair:** a freeform-trimmed pair
  the alternating minimizer cannot certify → native declines.
- **GS1 HLR — curved silhouette:** cylinder/cone/sphere outline tracing is out of
  the polyhedral-core slice → declined (polyhedral occluder covered).
- **GS5 inertia — non-watertight mesh:** no enclosed inertia is defined → declined.
- **GS6 validity — coplanar-overlap pair:** `selfIntersectionCertified == false`
  (transversal predicate cannot decide) → out-of-scope verdict, not a false clean.

If any GS service proves not cleanly fuzzable against a clean OCCT oracle within
this slice, it is documented as such and the cleanly-fuzzable services are
covered; a subset with real per-service coverage across ≥2 seeds is the bar, not
an all-or-nothing gate.
