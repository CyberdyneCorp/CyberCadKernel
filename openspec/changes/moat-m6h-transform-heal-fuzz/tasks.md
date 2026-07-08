# Tasks — moat-m6h-transform-heal-fuzz (MOAT M6-breadth-8)

Order: build substrate → reuse harness machinery → base-family + transform-chain
generators → three-way (native / OCCT / analytic) driving → five-way classifier →
coverage bar over ≥2 seeds → runner + suite SKIP + docs, or HONEST DECLINE. NO change
to `src/native/**` or `src/engine/**` (system under test); no `cc_*` ABI change; no
tolerance weakened.

## 0. Substrate

- [x] 0.1 Build the OCCT-linked simulator numsci: `bash scripts/build-numsci.sh
      iossim && bash scripts/build-numsci.sh host`, then export
      `CYBERCAD_NUMSCI_DIR=<worktree>/build-numsci/iossim`.
- [x] 0.2 Confirm in source the native transform path: `Shape::located(Location{
      math::Transform})` (`src/native/topology/shape.h`) is a placement; the
      tessellator world-places through the Location (`surface_eval.h`, `edge_mesher.h`)
      and the Explorer composes location down the graph (`explore.h`); `math::Transform`
      (`src/native/math/transform.h`) is the affine under test (`det<0` = mirror).

## 1. Domain choice (transform chains vs healing)

- [x] 1.1 Pick the cleaner domain: TRANSFORM CHAINS (closed-form similarity arbiter)
      over HEALING (no closed-form ground truth). Document the deferral of a healing
      fuzzer (its curated parity harness `native_heal_parity.mm` stays).

## 2. Harness skeleton (reuse the landed machinery)

- [x] 2.1 Create `tests/sim/native_transform_fuzz.mm` copying the RNG helper
      (splitmix64 → xoshiro256**, `FUZZ_SEED` from argv/env, fixed default), the
      coverage-tally struct, the OCCT `BRepGProp` measurement block, and the classifier
      printer from `native_mass_props_fuzz.mm`.
- [x] 2.2 Reuse the base-family generators + exact closed forms (BOX / NGON prism /
      CYLINDER / SPHERE / coaxial LOFT) and the native/OCCT builders.

## 3. Transform-chain generators + independent analytic affine

- [x] 3.1 Op kinds TRANSLATE / ROTATE(any axis, centre, angle) / USCALE(uniform) /
      MIRROR(any plane); chain length 1–4; force each kind singly in an initial block
      for guaranteed coverage; sparse singular (zero-scale) DECLINE-exerciser tail.
- [x] 3.2 Build each op THREE ways in lock-step: native `math::Transform`
      (`composedWith`), OCCT `gp_Trsf` (`Multiplied`), and an engine-independent fp64
      `Aff` (`affCompose`); track the product `S` and mirror parity.
- [x] 3.3 Analytic similarity image: `vol'=S³V₀`, `area'=S²A₀`, `centroid'=L·C₀+t`,
      topology invariant, signed-vol sign = `sign(base)·(−1)^#mirrors`.
- [x] 3.4 Determinism: same seed → byte-identical `[FUZZ]` batch (verified by
      re-running and diffing).

## 4. Three-way measurement + five-way classifier

- [x] 4.1 Native: `baseShape.located(Location{nativeXf})` → `SolidMesher` @
      `kPropertyDeflection`; measure signed+abs volume, area, centroid, face/edge/vertex
      counts.
- [x] 4.2 OCCT: `BRepBuilderAPI_Transform(occBase, occtTrsf, copy)` → `BRepGProp`.
- [x] 4.3 Classify AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE /
      BOTH-DECLINED / ORACLE_UNRELIABLE at the per-family deflection-matched tolerance
      (planar tight, curved deflection-derived at the scaled feature size); never
      widened. AGREE requires topology invariance AND handedness parity.
- [x] 4.4 Any DISAGREE / ORACLE-INACCURATE prints seed + case index + base/chain
      descriptor + native/OCCT/analytic measurements.

## 5. Coverage bar

- [x] 5.1 Print the per-base-family + per-op-kind AGREED coverage, the mirror
      handedness-flip-confirmed count, and the honest-scope block; log any
      capped/skipped trial.
- [x] 5.2 Exit 0 IFF `DISAGREED == 0 AND ORACLE_UNRELIABLE == 0` with each base family
      (BOX / NGON / CYLINDER / SPHERE / LOFT) and each op kind
      (TRANSLATE / ROTATE / USCALE / MIRROR) having ≥1 AGREED and the mirror flip
      confirmed ≥1.
- [x] 5.3 Run over **≥2 distinct seeds**; prove the bar on both.

## 6. Runner + suite wiring + docs

- [x] 6.1 Add `scripts/run-sim-native-transform-fuzz.sh` mirroring
      `run-sim-native-mass-props-fuzz.sh` (boot sim, build the one `.mm`, run over ≥2
      default seeds, propagate exit code).
- [x] 6.2 Add `native_transform_fuzz.mm` to the `run-sim-suite.sh` SKIP list.
- [x] 6.3 Record the run results (seeds, per-family/op-kind counts, bar outcome) below.

## 7. Honest-decline gate

- [x] 7.1 HEALING is DECLINED as a fuzz domain (no closed-form ground truth) and
      documented — not forced. ANISOTROPIC scale is out of the area arbiter's scope
      (no closed form); a singular transform is an HONEST DECLINE.
- [x] 7.2 If the fuzzer surfaces a real native DISAGREE, report it with the seed; do
      NOT silence it, do NOT weaken the tolerance to hide it.

## Results (IMPLEMENT phase — landed, uncommitted in worktree)

Deliverables: `tests/sim/native_transform_fuzz.mm`,
`scripts/run-sim-native-transform-fuzz.sh`, `run-sim-suite.sh` SKIP entry added.
`src/native/**`, `src/engine/**`, `include/**` and the `cc_*` ABI are UNTOUCHED — the
harness DRIVES the native `Shape::located(math::Transform)` + `SolidMesher` @
`kPropertyDeflection=0.005` path (signed/abs enclosed volume + surfaceArea +
signed-tetra centroid + watertight + Explorer face/edge/vertex counts) rather than
modifying it. No tolerance weakened; planar tol=1e-6 (exact under a similarity),
curved tol=5·deflection/(featureSize·min(1,S)) (deflection convergence bound at the
scaled world feature size, never widened past the mesh).

Runs on a booted iOS simulator (arm64), two distinct seeds, N=60 — **DISAGREED=0 and
ORACLE_UNRELIABLE=0 on both seeds**, each of the five base families AND each of the
four transform kinds with ≥1 AGREED, and the mirror handedness-flip positively
confirmed:

| seed | N | AGREED | HONESTLY-DECLINED | BOTH-DECLINED | DISAGREED | ORACLE_UNRELIABLE | mirror-flip-confirmed |
|---|---|---|---|---|---|---|---|
| 0x7A5C0FFEE2 | 60 | 58 | 0 | 2 | 0 | 0 | 20 |
| 0xB19D0C2A77 | 60 | 58 | 0 | 2 | 0 | 0 | 22 |

Per-base-family AGREE coverage — seed 0x7A5C0FFEE2: BOX 11, NGON 9, CYLINDER 11,
SPHERE 10, LOFT 17; seed 0xB19D0C2A77: BOX 15, NGON 11, CYLINDER 8, SPHERE 10, LOFT 14.
Per-op-kind AGREE coverage — seed 0x7A5C0FFEE2: TRANSLATE 28, ROTATE 30, USCALE 23,
MIRROR 25; seed 0xB19D0C2A77: TRANSLATE 35, ROTATE 27, USCALE 28, MIRROR 28. Planar
bases (BOX/NGON/LOFT) reproduce the solid EXACTLY under a similarity (dV/dA ~1e-16,
machine-tight, even for chains with scale S up to ~3 and odd/even mirror counts);
curved bases (CYLINDER/SPHERE) are deflection-bounded (dV ~2e-3..1.3e-2 under the
matched tol). Topology counts are INVARIANT across every transform
(`topo(f/e/v==base)`). Mirror HANDEDNESS: every odd-mirror chain flipped the signed
enclosed-volume sign while staying a valid watertight positive-|volume| solid
(`hand=1`). Determinism verified: same seed twice → byte-identical `[FUZZ]` batch.

BOUNDARY / ORACLE FINDING (reported, not papered over): a singular (zero-scale)
transform collapses the native solid to a non-watertight degenerate → the NATIVE path
declines it CLEANLY and quickly (confirmed by stage-localised instrumentation: the
native mesh completes, returning non-watertight). OCCT, however, HANGS: feeding
`BRepBuilderAPI_Transform` a zero-scale (non-invertible) `gp_Trsf` on a CURVED base
(reproduced on a CYLINDER at seed 0x7A5C0FFEE2 case 46) does not return — an OCCT
oracle pathology on a degenerate placement, NOT a native fault. Because a zero-scale is
not a valid invertible placement, the harness does NOT hand the singular transform to
OCCT (gated on `ch.singular`); the case is classified BOTH-DECLINED (native declines +
oracle unable). This is an honest ORACLE-LIMITATION decline, logged in the harness
comment and here — the native transform layer is vindicated (it declines the degenerate
input without hanging). HEALING is DECLINED as a fuzz domain (no closed-form ground
truth) and left to its curated parity harness (`native_heal_parity.mm`). ANISOTROPIC
scale is out of the area arbiter's scope (uniform scale only; `cc_scale_shape` is
uniform).

No native DISAGREE surfaced; the only anomaly found is the OCCT zero-scale-transform
hang (oracle-side, handled). Do NOT commit (per workflow) — changes left uncommitted.
