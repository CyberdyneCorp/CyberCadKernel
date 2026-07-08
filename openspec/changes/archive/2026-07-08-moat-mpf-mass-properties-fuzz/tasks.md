# Tasks — moat-mpf-mass-properties-fuzz (MOAT M6-breadth-6)

Order: build substrate → reuse harness machinery → family generators + analytic
arbiters → dual native/OCCT measurement → five-way classifier → coverage bar over ≥2
seeds → runner + suite SKIP + docs, or HONEST DECLINE. NO change to `src/native/**`
or `src/engine/**` (system under test); no `cc_*` ABI change; no tolerance weakened.

## 0. Substrate

- [x] 0.1 Build the OCCT-linked simulator numsci: `bash scripts/build-numsci.sh
      iossim && bash scripts/build-numsci.sh host`, then export
      `CYBERCAD_NUMSCI_DIR=<worktree>/build-numsci/iossim`.
- [x] 0.2 Confirm in source the native mass path shape used by the design:
      `NativeEngine::mass_properties` is mesh-based at `kPropertyDeflection = 0.005`
      and `NativeEngine::principal_moments` delegates to OCCT for a native body
      (`CC_NATIVE_BODY_UNSUPPORTED`). Record the exact line refs.

## 1. Harness skeleton (reuse the landed machinery)

- [x] 1.1 Create `tests/sim/native_mass_props_fuzz.mm` copying the RNG helper
      (splitmix64 → xoshiro256**, `FUZZ_SEED` from argv/env, fixed default), the
      coverage-tally struct, and the classifier printer from
      `native_construct_fuzz.mm`.
- [x] 1.2 Copy the OCCT `BRepGProp::VolumeProperties` + `SurfaceProperties` +
      `GProp_PrincipalProps` measurement block; add a centroid extractor
      (`GProp_GProps::CentreOfMass`).

## 2. Family generators + analytic arbiters (§3 of design)

- [x] 2.1 BOX, NGON_PRISM (planar, exact volume/area/centroid/inertia; tight tol).
- [x] 2.2 CYLINDER, CONE/FRUSTUM, SPHERE (revolves; exact closed form; deflection tol).
- [x] 2.3 LOFT (coaxial n-gon prismatoid band volume + trapezoid lateral area).
- [x] 2.4 REVOLVE (arbitrary axial polygon; Pappus volume + lateral area).
- [x] 2.5 Sparse DECLINE-exercisers (zero-height / self-touching profile).
- [x] 2.6 Each generator emits ONE parameter POD consumed identically by the native
      builder and the OCCT builder; assert determinism (same seed → byte-identical).

## 3. Dual measurement + five-way classifier

- [x] 3.1 Per trial: build native (via `cc_*` under the native engine), measure
      `cc_mass_properties` (volume/area/centroid, `valid`); build OCCT, measure
      `BRepGProp`; compute the analytic ground truth.
- [x] 3.2 Classify AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE /
      BOTH-DECLINED at the per-family deflection-matched tolerance (planar tight,
      curved deflection-derived); never widened.
- [x] 3.3 Record the inertia dimension as an HONEST NATIVE DECLINE for every native
      body; run the closed-form-vs-OCCT inertia check only as oracle-trust telemetry
      (logged, never a bar input).
- [x] 3.4 Any DISAGREE / ORACLE-INACCURATE prints seed + case index + family/param
      tuple + native/OCCT/analytic triple.

## 4. Coverage bar

- [x] 4.1 Print the per-family AGREED / DECLINED / DISAGREED / ORACLE-INACCURATE /
      BOTH-DECLINED summary plus the honest-scope block (native inertia decline; the
      per-family deflection tolerance rationale); log any capped/skipped trial.
- [x] 4.2 Exit 0 IFF `DISAGREED == 0` with each of BOX / NGON_PRISM / CYLINDER /
      CONE / SPHERE / LOFT / REVOLVE having ≥1 AGREED trial.
- [x] 4.3 Run over **≥2 distinct seeds**; prove DISAGREED == 0 with real per-family
      coverage on both.

## 5. Runner + suite wiring + docs

- [x] 5.1 Add `scripts/run-sim-native-mass-props-fuzz.sh` mirroring
      `run-sim-native-construct-fuzz.sh` (boot sim, build the one `.mm`, run over the
      chosen seeds, propagate exit code).
- [x] 5.2 Add `native_mass_props_fuzz.mm` to the `run-sim-suite.sh` SKIP list (it runs
      under its own runner, like the other four fuzzers).
- [x] 5.3 Record the run results (seeds, per-family counts, bar outcome) in the change
      before archiving; update `MOAT-ROADMAP.md` to mark the sixth domain landed.

## 6. Honest-decline gate

- [x] 6.1 If a family has no clean oracle (no closed form AND OCCT unreliable), the
      family is DECLINED and documented — not forced into a pass with a widened
      tolerance.
- [x] 6.2 If the fuzzer surfaces a real native DISAGREE, report it with the seed;
      do NOT silence it, do NOT weaken the tolerance to hide it — a fix is a separate
      change carrying its own regression trial.

## Results (IMPLEMENT phase — landed, uncommitted in worktree)

Deliverables: `tests/sim/native_mass_props_fuzz.mm`,
`scripts/run-sim-native-mass-props-fuzz.sh`, `run-sim-suite.sh` SKIP entry added.
`src/native/**`, `src/engine/**`, `include/**` and the `cc_*` ABI are UNTOUCHED — the
harness REPRODUCES `NativeEngine::mass_properties` (SolidMesher @ kPropertyDeflection=0.005
→ surfaceArea/|enclosedVolume|/signed-tetra centroid/watertight-validity) rather than
modifying it. No tolerance weakened; planar tol=1e-6 (exact), curved tol=5·deflection/
featureSize (deflection convergence bound, never widened past the mesh).

Runs on a booted iOS simulator (arm64), three distinct seeds — **DISAGREED=0 and
ORACLE_UNRELIABLE=0 on every seed**, each of the seven AGREE families with ≥1 AGREED:

| seed | N | AGREED | HONESTLY-DECLINED | BOTH-DECLINED | DISAGREED | ORACLE_UNRELIABLE |
|---|---|---|---|---|---|---|
| 0x6D3A11C05B | 120 | 110 | 7 | 3 | 0 | 0 |
| 0xA17F3C92D5E | 160 | 145 | 9 | 6 | 0 | 0 |
| 0xDEADBEEF01 | 140 | 127 | 4 | 9 | 0 | 0 |

Per-family AGREE coverage (seed 0x6D3A11C05B): BOX 14, NGON 13, CYLINDER 19,
CONE 13 (apex) + 7 DECLINED (frustum), SPHERE 16, LOFT 22, REVOLVE 13. Planar
families exact (dV/dA ~1e-15); curved families deflection-bounded (dV ~2e-3..9e-3,
worst sphere ~1.56e-2 at R=1, under the matched tol). Determinism verified: same seed
twice → byte-identical [FUZZ] batch.

BOUNDARY (reported, not papered over): the CONE **FRUSTUM** sub-family (r1>0) does NOT
weld watertight at deflection 0.005 (cap⟷slant seam) → native valid=false → first-class
HONEST DECLINE (OCCT's cone is valid and ships). The CONE **apex** sub-family (r1=0)
DOES weld → real AGREE coverage. INERTIA is an HONEST NATIVE DECLINE for every body
(`NativeEngine::principal_moments` = `CC_NATIVE_BODY_UNSUPPORTED`); OCCT
`GProp_PrincipalProps` moments logged as oracle-trust telemetry only, never a bar input.

No native DISAGREE surfaced. Do NOT commit (per workflow) — changes left uncommitted.
