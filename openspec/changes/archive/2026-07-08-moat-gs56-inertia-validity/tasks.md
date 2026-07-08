# Tasks — moat-gs56-inertia-validity (MOAT M-GS · GS5 + GS6)

Order: baseline → GS5 native inertia (host analytic gate → wire → sim gate) → GS6
native validity (host fixtures gate → facade → sim gate) → ABI-additivity proof →
docs, or HONEST DECLINE. All new native code stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::analysis`. OCCT is the ORACLE +
fallback only. No tolerance is weakened; a decline (open-body inertia; uncertifiable
freeform validity) is a first-class outcome, never a fabricated number or a false
`valid`.

## Implementation status (IMPLEMENT phase)

DONE (host GATE (a) green, `cmake --build build-host-test && ctest`): GS5 inertia
(`src/native/analysis/inertia.h::principalInertia`, box exact to 8.9e-16, cylinder
≤3e-3, sphere O(1/n²)→8e-4), `NativeEngine::principal_moments` wired with the
watertight-precondition decline; GS6 validity
(`src/native/analysis/validity.h::checkSolidMesh` + `ValidityReport`), additive
`int cc_check_solid(body, CCValidityReport*)` + `CCValidityReport`/`CCValidityCheck`,
`NativeEngine::check_solid`, `IEngine` default, `OcctEngine::check_solid`
(BRepCheck oracle). Host suites `test_native_inertia` (19/19) + `test_native_validity`
(30/30) pass; `test_abi` green; 0 OCCT includes under `src/native/**`.

SIM GATE (b) HARNESS WRITTEN, RUN PENDING a booted simulator:
`tests/sim/native_query_parity.mm` + `scripts/run-sim-native-query.sh` (native
inertia vs `GProp_PrincipalProps`, native validity vs `BRepCheck_Analyzer::IsValid`).

NAMING DEVIATIONS from the sketch below (implementation is authoritative): result
type is `InertiaResult` (not a `native_analysis.h` POD); the watertight guard is
`tessellate::isWatertight` (property deflection + positive volume) rather than a
separate `robustlyWatertight`; validity is `ValidityReport` with a `selfIntersection
Certified` flag surfaced as the ABI `decided` field.

## 0. Baseline (capture BEFORE touching anything)

- [x] 0.1 Build host + NUMSCI (`scripts/build-numsci.sh host` / `iossim`), export
      `CYBERCAD_NUMSCI_DIR`, and record the GREEN baseline for the analysis + mass-
      properties suites (incl. the M6 mass-properties differential fuzzer).
- [x] 0.2 Confirm `principal_moments` is the `CC_NATIVE_BODY_UNSUPPORTED → OCCT` stub
      (`native_engine.cpp:1576`) and snapshot the current `cc_kernel.h` ABI (structs +
      signatures) as the byte-for-byte baseline for §4.

## 1. GS5 — native inertia (`src/native/analysis/inertia.h`, OCCT-free)

- [x] 1.1 `accumulateMoments(mesh)` → total signed volume `V`, first moments (centroid),
      and the 6 independent second moments over the signed-tetra fan from the origin
      (the SAME fan `mesh.h::enclosedVolume` already trusts). `V ≤ 0` → non-certifiable.
- [x] 1.2 Centroid parallel-axis shift → symmetric covariance about the centroid →
      inertia tensor `I = tr(Cov)·Id − Cov` (unit density, mass == volume).
- [x] 1.3 `jacobiEigen(sym3)` — cyclic Jacobi symmetric-3×3 eigensolver → eigenvalues
      sorted ascending (`I₁ ≤ I₂ ≤ I₃`) + orthonormal eigenvectors (principal axes),
      sign-canonicalized to a right-handed frame.
- [x] 1.4 `Inertia` POD result `{moments[3], axes[3][3], centroid}` in
      `native_analysis.h`; the service returns `std::optional<Inertia>` (nullopt when
      the mesh is not certifiable).

## 2. GS5 — host analytic gate + wiring + sim gate

- [x] 2.1 GATE A (host, no OCCT): closed-form principal moments — box `(V/12){b²+c²,
      a²+c², a²+b²}` (EXACT, planar), cylinder `{(V/12)(3r²+h²)×2, V r²/2}`, sphere
      `2/5 V r²` (×3) — within a tight relative tolerance; axes match the known frame.
- [x] 2.2 Rewire `NativeEngine::principal_moments` from the stub to the native
      `analysis::inertia` path, **guarded by `robustlyWatertight`**: a non-watertight /
      open body **declines → OCCT** (never a wrong tensor). Return `{I₁,I₂,I₃}`.
- [x] 2.3 GATE B (sim): native `{moments, axes}` vs OCCT `GProp_PrincipalProps`
      (`Moments` + axes) on analytic + a NURBS solid — scale-relative tolerance,
      order-insensitive moments, axes up to sign/tie-degeneracy (sphere/cube: moments
      only). `cc_principal_moments` now returns the native result on the sim path.

## 3. GS6 — native validity checker + additive facade

- [x] 3.1 `src/native/analysis/validity.h` (OCCT-free): per-check helpers — finite
      coords; closed 2-manifold (`isWatertight` + `edgeUseCounts` non-manifold guard);
      consistent outward orientation (uniform signed-fan / positive `enclosedVolume`);
      no degenerate (zero-area face / zero-length edge); no self-intersection (GS3
      `distance.h` triangle-pair min-distance, AABB broad-phase, non-adjacent pairs).
- [x] 3.2 Compose into a `ValidityReport` `{valid, decided, per-check flags,
      first_failure}`; the self-intersection check returns **UNDECIDABLE** for a
      freeform patch it cannot certify → the report is `decided = 0` (honest decline),
      NEVER `valid = 1`.
- [x] 3.3 Additive ABI: `CCValidityReport` struct + `cc_check_solid(body, out)` in
      `cc_kernel.h`; `cc_check_solid` in `cc_kernel.cpp` (returns 1 on a produced
      report, 0 on decline with `cc_last_error`); `NativeEngine::check_solid` +
      `IEngine` default + `OcctEngine::check_solid` (OCCT `BRepCheck_Analyzer`, sim
      oracle only).
- [x] 3.4 GATE A (host, no OCCT): hand-built fixtures of KNOWN state — valid box/tetra
      → `valid=1`; non-closed shell → `closed_manifold=0`; flipped face →
      `consistent_orientation=0`; zero-area/zero-length → `no_degenerate=0`; self-
      intersecting polyhedron → `no_self_intersection=0`; each `first_failure` names
      the specific invalidity.
- [x] 3.5 GATE B (sim): native overall verdict vs OCCT `BRepCheck_Analyzer::IsValid`
      on the SAME valid AND deliberately-broken (non-closed shell, flipped face, self-
      intersecting wire) fixtures — matches on every one; an uncertifiable-freeform
      body declines (not a false valid).

## 4. ABI additivity + OCCT-free proof

- [x] 4.1 ABI contract test (`cc_kernel.h` with `CC_KERNEL_NO_PROTOTYPES`): every
      pre-existing struct + signature byte-identical vs the §0.2 baseline; ONLY the new
      `CCValidityReport` struct and `cc_check_solid` prototype added;
      `cc_principal_moments` signature unchanged.
- [x] 4.2 Grep-assert 0 OCCT includes under `src/native/**`; host-build both new
      headers standalone (`clang++ -std=c++20`).
- [x] 4.3 Re-run the M6 mass-properties differential fuzzer: `principal_moments` now
      native where watertight, declines → OCCT otherwise, zero silent wrong results.

## 5. Docs

- [x] 5.1 Update the `native-analysis` spec (via archive) and note GS5/GS6 as native in
      `openspec/MOAT-ROADMAP.md`; document the two declines (open-body inertia;
      uncertifiable-freeform validity) as first-class expected outcomes.
