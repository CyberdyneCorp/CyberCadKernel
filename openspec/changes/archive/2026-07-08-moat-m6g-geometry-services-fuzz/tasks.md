# Tasks — moat-m6g-geometry-services-fuzz (MOAT M6-breadth-7)

Order: substrate build → deterministic generator (incl. oblique/tilted) →
per-service dual native+OCCT evaluators + closed-form arbiters → five-way
classifier → coverage summary + zero-DISAGREE bar → runner + SKIP-list → docs.
The harness is TEST INFRASTRUCTURE: it edits NO `src/native/**` (OCCT-FREE,
UNTOUCHED), changes NO `cc_*` signature, adds NO geometry capability. OCCT is the
ORACLE. No tolerance is EVER widened; an HONEST-NATIVE-DECLINE is first-class; a
DISAGREE fails the bar and is reported with its seed.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR=/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/moat-m6g/build-numsci/iossim`.
- [x] 0.2 Confirm the GS services build and are on the verified native path
      (OCCT-free, header-only): `analysis/distance.h`, `analysis/curvature.h`,
      `analysis/inertia.h`, `analysis/validity.h`, `section/section.h`,
      `drafting/orthographic_hlr.h` — and their `NativeEngine` entry points
      (`measure_distance`, `surface_curvature`/`edge_curvature`, `mass_properties`).
- [x] 0.3 Confirm the OCCT oracle headers are reachable in the sim harness:
      `BRepExtrema_DistShapeShape`, `GeomLProp_SLProps`/`BRepLProp_SLProps`,
      `BRepAlgoAPI_Section` + `BRepGProp`, `HLRBRep_Algo`, `GProp_PrincipalProps`,
      `BRepCheck_Analyzer`.

## 1. Deterministic seeded generator (OCCT-free parameter POD)

- [x] 1.1 splitmix64 → xoshiro256** stream keyed ONLY by argv/env `FUZZ_SEED`
      (fixed default); no wall clock / `rand()` / pid / address / thread schedule.
- [x] 1.2 Assert byte-identical batch for a fixed `FUZZ_SEED` + batch size across
      two runs (self-check at startup).
- [x] 1.3 Per-service sub-generators drawing from the shared stream in a FIXED
      order, each emitting a valid in-scope tuple AND labelled oblique/tilted and
      out-of-scope DECLINE-exerciser tuples:
  - [x] 1.3a GS3: entity pairs (vertex/edge/face) incl. SKEW segment pairs and
        offset/tilted line-plane pairs in general position.
  - [x] 1.3b GS4: analytic faces (plane/sphere/cylinder/cone/torus) + NURBS faces,
        TILTED, sampled at interior `(u,v)`; edges at parameter `t`; plus
        singular-chart (pole/apex/cusp) decline-exercisers.
  - [x] 1.3c GS2: cut planes through solids incl. OBLIQUE planes and, decisively,
        the OBLIQUE CYLINDER cut (the `plane_conics` exemplar) as a labelled
        expected-decline tuple.
  - [x] 1.3d GS1: polyhedral/quadric solids with random view directions incl.
        oblique/isometric (not axis-on); curved-silhouette decline-exercisers.
  - [x] 1.3e GS5: solids (box/prism/cyl/cone/sphere) under random ROTATION so
        principal axes are off the world axes; non-watertight decline-exercisers.
  - [x] 1.3f GS6: valid solids AND deterministically-BROKEN variants (hole /
        flipped-face / self-intersection) with a KNOWN ground-truth verdict.

## 2. Dual native + OCCT evaluation, one input, closed-form arbiter

- [x] 2.1 Native evaluator per service: call the OCCT-free service DIRECTLY; a
      typed decline is observed as a decline (NOT silently forwarded to OCCT).
- [x] 2.2 OCCT oracle evaluator per service: build the geometrically IDENTICAL
      shape and run the mapped OCCT oracle (§ design map D-table).
- [x] 2.3 Closed-form analytic arbiter per family that has one (GS3 exact min
      distance; GS4 sphere/cyl/cone/torus/circle curvature; GS2 box/cyl/sphere
      section area; GS1 box-isometric 9-visible/3-hidden; GS5 exact tensor;
      GS6 construction-time verdict).
- [x] 2.4 Assert native and OCCT operated on the SAME geometry (same tuple), and
      `src/native/**` referenced NO OCCT type in the native evaluation path.

## 3. Five-way classifier at a FIXED per-service tolerance

- [x] 3.1 Per-service tolerance matched to the error source (§ design D3): tight
      for exact-analytic; deflection-convergence bound for mesh-derived; fixed
      projection tol for HLR. NEVER widened.
- [x] 3.2 Classify each trial into EXACTLY ONE of: AGREE / HONEST-NATIVE-DECLINE /
      DISAGREE / ORACLE-INACCURATE / BOTH-DECLINE, with the closed-form arbiter
      authoritative over OCCT where it exists.
- [x] 3.3 An oblique/tilted decline in a documented out-of-scope sub-domain (GS2
      oblique cylinder, GS4 singular chart, GS1 curved silhouette, GS5 non-
      watertight, GS6 coplanar overlap) classifies HONEST-NATIVE-DECLINE — assert
      the decline is honest (typed decline AND trial genuinely out-of-scope), NOT
      a DISAGREE, NOT skipped.
- [x] 3.4 A DISAGREE (native confident but wrong per oracle + closed form) FAILS
      the bar and prints seed, service, case index, input tuple, and
      native/OCCT/analytic values so it is reproducible. Do NOT paper over it.

## 4. Coverage summary + zero-silent-wrong-answer bar

- [x] 4.1 Print per-service counts of AGREE / HONEST-NATIVE-DECLINE / DISAGREE /
      ORACLE-INACCURATE / BOTH-DECLINE.
- [x] 4.2 Exit 0 IFF DISAGREE == 0 across the batch, with real per-service
      coverage (each covered GS service has ≥1 AGREE trial, incl. an oblique/tilted
      trial where the service accepts it) proven across ≥2 distinct seeds.
- [x] 4.3 Log honest scope explicitly: the GS2 oblique-cylinder `plane_conics`
      decline, each service's documented decline sub-domain, the mesh-vs-exact
      deflection boundary, and any capped/skipped trial. No silent cap or drop.
- [x] 4.4 If a GS service is not cleanly fuzzable within this slice, document it
      and cover the cleanly-fuzzable subset (a subset with honest per-service
      coverage across ≥2 seeds is the bar).

## 5. Runner, suite wiring, docs

- [x] 5.1 Add `scripts/run-sim-native-geometry-services.sh` (mirror an existing
      `run-sim-native-*.sh`): boot the sim, build + run the `.mm` over ≥2 seeds.
- [x] 5.2 Add `native_geometry_services_fuzz.mm` to the `run-sim-suite.sh` SKIP
      list (run by its dedicated runner, like every other `native_*_fuzz.mm`).
- [x] 5.3 Run over ≥2 seeds; confirm DISAGREE == 0 and per-service coverage; if a
      real GS-service bug is surfaced, report it prominently with its seed.
- [x] 5.4 Update `openspec/specs` via archive once implemented; note the seventh
      M6 domain in the MOAT roadmap.
