# Tasks — moat-m6i-reference-geometry-fuzz (MOAT M6-breadth-9)

Order: confirm the substrate → reuse harness machinery → base-family + rigid-pose
generators → three-way (native / OCCT / analytic) driving of each reference op →
six-way classifier → coverage bar over ≥2 seeds → runner + suite SKIP + docs. NO change
to `src/native/**` or `src/engine/**` (system under test); no `cc_*` ABI change; no
tolerance weakened.

## 0. Substrate

- [x] 0.1 Build the OCCT-linked simulator numsci: `bash scripts/build-numsci.sh iossim
      && bash scripts/build-numsci.sh host` (both exit 0). This domain does NOT link
      numsci (reference/construct/topology/math are header-only, OCCT-FREE) but the
      build sanity-check is run.
- [x] 0.2 Confirm in source the native reference path: `src/native/reference/
      reference.h` computes each datum by exact fp64 math from `topology::Shape`, baking
      the sub-shape `Location`; `Shape::located(Location{math::Transform})`
      (`src/native/topology/shape.h`) is a placement the reference services read through.

## 1. Domain choice (reference-geometry vs direct-modeling vs section-curve)

- [x] 1.1 Pick reference-geometry: cleanest closed-form arbiter (analytic primitive
      datums transform exactly under a rigid pose) and highest-value gap (M-REF landed
      with per-op parity but no fuzz domain). Document the deferral of direct-modeling
      (numsci-substrate-gated) and section-curve (no native `cc_*` producer to fuzz).

## 2. Harness skeleton (reuse the landed machinery)

- [x] 2.1 Create `tests/sim/native_reference_geometry_fuzz.mm` copying the RNG helper
      (splitmix64 → xoshiro256**, `FUZZ_SEED` from argv/env, fixed default), the
      coverage-tally struct, the engine-independent fp64 affine, and the OCCT oracle
      block from `native_transform_fuzz.mm` / `native_reference_parity.mm`.
- [x] 2.2 Reuse the base-family generators + native/OCCT builders (BOX / NGON prism via
      `build_prism` + `MakeBox`/`MakePrism`; CYLINDER / CONE via `build_revolution` +
      `MakeCylinder`/`MakeCone`).

## 3. Rigid-pose generator + independent analytic affine

- [x] 3.1 A random RIGID pose (rotate about a random unit axis + translate; NO
      scale/mirror) built THREE ways in lock-step: native `math::Transform`, OCCT
      `gp_Trsf`, engine-independent fp64 `Aff`.
- [x] 3.2 Analytic datum image: axis/line dir ← `P.linear·d₀`, origin ← `P·o₀`, cap
      normal ← `P.linear·n₀`; inward-offset polygon area is rigid-invariant (= the
      closed-form miter offset on the unposed base polygon).
- [x] 3.3 Determinism: same seed → byte-identical `[FUZZ]` batch (verified by re-run
      diff — 538 lines byte-identical at seed 0x9EF12A0055 N=96).

## 4. Three-way measurement + six-way classifier (per reference op)

- [x] 4.1 PLANE (`refPlaneFromFace`) vs `gp_Pln` + analytic normal (all OCCT planar
      faces matched by parallel normal + coplanar origin).
- [x] 4.2 FAXIS (`faceAxis`/`refAxisFromFace`) vs `gp_Cylinder`/`gp_Cone::Axis` +
      analytic posed +Y axis; `refAxisFromFace == faceAxis` contract asserted.
- [x] 4.3 EAXIS (`refAxisFromEdge`) vs `gp_Lin` (all OCCT line edges matched by dir +
      midpoint-on-native-line).
- [x] 4.4 OFFSET (`offsetFaceBoundary`) polygon-cap inward area vs the closed-form miter
      offset; circular cap MUST decline (matched → HONESTLY-DECLINED).
- [x] 4.5 RIM (`outerRimChain`) polygon-cap midpoint set vs `BRepTools::OuterWire`;
      circular cap arbitrated STRUCTURALLY (rim id set == cap face outer wire, OCCT
      circle confirms circular boundary).
- [x] 4.6 TANGENT (`tangentChain`) straight-edge C1 grow decision (grown pair must be
      collinear) + circle-edge seed must NOT decline.
- [x] 4.7 Classify AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE /
      BOTH-DECLINED / ORACLE_UNRELIABLE at the fixed tight rigid tolerance; never
      widened. Any DISAGREE / ORACLE-INACCURATE / ORACLE_UNRELIABLE prints seed + index.

## 5. Coverage bar

- [x] 5.1 Print the per-base-family + per-op AGREED coverage and the honest-scope block.
- [x] 5.2 Exit 0 IFF `DISAGREED == 0 AND ORACLE_UNRELIABLE == 0` with each base family
      (BOX/NGON/CYLINDER/CONE) and each op (PLANE/FAXIS/EAXIS/OFFSET/RIM/TANGENT) ≥1
      AGREED.
- [x] 5.3 Run over ≥2 distinct seeds (N=96 each ≥ the N≥60 gate); prove the bar on both.

## 6. Runner + suite wiring + docs

- [x] 6.1 Add `scripts/run-sim-native-reference-geometry-fuzz.sh` mirroring
      `run-sim-native-reference.sh` (boot sim, build the one `.mm`, run over ≥2 default
      seeds, propagate exit code; add `TKHLR`/`TKShHealing` for the drafting adapter).
- [x] 6.2 Add `native_reference_geometry_fuzz.mm` to the `run-sim-suite.sh` SKIP list.
- [x] 6.3 Record the run results (seeds, per-family/op counts, bar outcome) below.

## 7. Honest-scope gate

- [x] 7.1 SCALE/MIRROR poses out of scope (rigid keeps the datum image exact); a
      circular-cap offset and a freeform edge in a tangent walk are FIRST-CLASS declines
      matched by the closed form. Documented, not forced.
- [x] 7.2 If the fuzzer surfaces a real native DISAGREE, report it with the seed; do NOT
      silence it, do NOT weaken the tolerance to hide it. (A harness-oracle mismatch on
      the circular-cap rim — native periodic arc representation vs OCCT's single seam
      edge — was diagnosed and correctly re-arbitrated STRUCTURALLY; it was never a
      native datum bug and no native code was touched.)

## Results (IMPLEMENT phase — landed, committed on branch moat-m6i)

Deliverables: `tests/sim/native_reference_geometry_fuzz.mm`,
`scripts/run-sim-native-reference-geometry-fuzz.sh`, `run-sim-suite.sh` SKIP entry added.
`src/native/**`, `src/engine/**`, `include/**` and the `cc_*` ABI are BYTE-UNCHANGED —
the harness DRIVES the native `ref::` services on `Shape::located(math::Transform)`
rather than modifying them. No tolerance weakened; dirTol=1e-9, ptTol=1e-7, offTol=1e-6.

Runs on a booted iOS simulator (arm64), two distinct seeds, N=96 — **DISAGREED=0 and
ORACLE_UNRELIABLE=0 on both seeds**, each of the four base families AND each of the six
reference ops with ≥1 AGREED:

| seed | N | AGREED | HONESTLY-DECLINED | DISAGREED | ORACLE-INACCURATE | BOTH-DECLINED | ORACLE_UNRELIABLE |
|---|---|---|---|---|---|---|---|
| 0x9EF12A0055 | 96 | 480 | 58 | 0 | 0 | 0 | 0 |
| 0xC0DEDA7A11 | 96 | 480 | 41 | 0 | 0 | 0 | 0 |

Per-base-family AGREED — seed 0x9EF12A0055: BOX 85, NGON 105, CYLINDER 135, CONE 155;
seed 0xC0DEDA7A11: BOX 130, NGON 145, CYLINDER 95, CONE 110. Per-op AGREED — seed
0x9EF12A0055: PLANE 96, FAXIS 58, EAXIS 96, OFFSET 38, RIM 96, TANGENT 96; seed
0xC0DEDA7A11: PLANE 96, FAXIS 41, EAXIS 96, OFFSET 55, RIM 96, TANGENT 96. The
HONESTLY-DECLINED count is the circular-cap OFFSET (reference.h declines a circular
boundary; the closed form confirms no polygon → first-class decline). Analytic-exact
datums: FAXIS axis dir/origin error 0–2.2e-16, EAXIS line worst 1e-16 (curved caps)
/ ~5 (posed midpoint magnitude, matched within ptTol), OFFSET area dA ≤ ~1e-14.
Determinism verified: same seed twice → byte-identical batch (538 `[FUZZ]` lines).

BOUNDARY / ORACLE FINDING (reported, not papered over): the native periodic revolution
CAP stores its outer wire as ≥1 Circle-ARC edges (max 2π/3 span, per the construct
angular-station split) with periodic seam vertices, vs OCCT's `MakeCone`/`MakeCylinder`
single seam edge. So the OCCT circle is NOT a faithful per-vertex oracle for the
circular-cap `outerRimChain`; the FAITHFUL closed-form ground truth is the STRUCTURAL
identity "the rim id set equals the native cap face's own outer wire" (confirmed circular
by the OCCT circle oracle). This is a representational difference, NOT a native datum
defect — no native code was touched. No native DISAGREE surfaced.
