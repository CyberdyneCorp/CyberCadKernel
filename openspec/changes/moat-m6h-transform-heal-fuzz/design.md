# Design — moat-m6h-transform-heal-fuzz (MOAT M6-breadth-8)

Extend the differential-fuzzing completeness bar to an EIGHTH native domain — the
**rigid/similarity TRANSFORM layer** the CyberCad app's translate / rotate / scale /
mirror / place tools read (`cc_translate_shape`, `cc_rotate_shape_about`,
`cc_scale_shape`, `cc_mirror_shape`, `cc_place_on_frame`). This is INFRASTRUCTURE: a
seeded harness, not a geometry capability. OCCT `BRepBuilderAPI_Transform` +
`BRepGProp` is the ORACLE; a THIRD, engine-independent closed-form similarity image is
the PRIMARY arbiter; the bar is ZERO silent wrong transforms over a seeded batch; an
HONEST DECLINE (singular transform → no valid mesh) is first-class. `src/native/**`
and `src/engine/**` are UNTOUCHED — they are the system under test.

## 0. Why transform chains, not healing (the domain choice)

The task allowed either TRANSFORM CHAINS or HEALING; the diagnosis picks transform
chains because it has a **closed-form analytic arbiter** and healing does not:

- A chain of translate / rotate / uniform-scale / mirror is a SIMILARITY. Its effect
  on ANY solid's volume / area / centroid / topology / handedness is an EXACT closed
  form of the base solid and the composed map — a third oracle independent of BOTH
  the native engine and OCCT. That is the pristine differential-fuzzing setup: the
  arbiter attributes a native-vs-OCCT gap instead of reflexively blaming either.
- Healing (broken-soup repair) has NO closed-form ground truth. Its "correct" output
  is a *heuristic* agreement with OCCT `ShapeFix`/`Sewing`; a native-vs-OCCT
  disagreement cannot be attributed without a third oracle that does not exist. A
  fuzzer with only two heuristic engines and no arbiter would launder either engine's
  quirk into a verdict. Healing already has a curated parity harness
  (`native_heal_parity.mm`, `heal::healShell` vs `cyber::occt::sewAndFix` on fixed
  broken fixtures); a *fuzzer* for it is deferred honestly, not faked.

## 1. The substrate (verified in source, not assumed)

`src/native/topology/shape.h`:

- `Shape::located(const Location& loc)` returns a copy whose `Location` is
  `loc.composedWith(existing)` — the transform is a placement, not a rebuild of the
  geometry node.
- `Location` wraps `math::Transform` with an identity fast-path (mirrors
  `TopLoc_Location`).

`src/native/math/transform.h` — the SYSTEM UNDER TEST:

- `Transform` = affine `v' = L·v + t` (a 3×3 linear part + translation, exactly
  `gp_Trsf`'s model). `Mat3::rotation` is Rodrigues; `Transform::scaleOf` is uniform
  (or per-axis) scale about a centre; `composedWith` and `inverse` are closed form;
  `determinant() < 0 ⇔ isMirrored()`. There is no mirror factory, so the harness
  builds the reflection linear part `L = I − 2·n·nᵀ`, `t = 2(p·n)·n` directly and
  feeds it to `Transform{L, t}`.

`src/native/tessellate/*` and `src/native/topology/explore.h`:

- `SurfaceEvaluator` / `edge_mesher` world-place every sample through the face's
  `Location`; the `Explorer` composes the location down the graph
  (`.located(s.location())`). So `SolidMesher{}.mesh(shape.located(loc))` meshes the
  TRANSFORMED solid — the same path the app's transform tools drive.

`src/native/tessellate/mesh.h`: `surfaceArea` (`Σ ½|(b−a)×(c−a)|`), `enclosedVolume`
(SIGNED divergence-theorem tetra sum — its SIGN is the handedness signal),
`isWatertight`. The harness reproduces the mass path (mesh @ `kPropertyDeflection =
0.005`) exactly as `native_mass_props_fuzz.mm` does.

## 2. The closed-form similarity arbiter (PRIMARY)

For a similarity `L = S·Q` (`Q` orthonormal, `det Q = ±1`; `S = Π` uniform-scale
factors) applied to a solid with exact base `V₀, A₀, C₀`:

| quantity | closed form | why |
|---|---|---|
| volume | `V' = S³·V₀` | `|det L| = S³` |
| area | `A' = S²·A₀` | orthonormal `Q` preserves area; uniform `S` scales it by `S²` |
| centroid | `C' = L·C₀ + t` | affine image of the base centroid (exact) |
| topology | `nf/ne/nv unchanged` | a placement adds/drops no sub-shape |
| handedness | `sign(V'_signed) = sign(V₀_signed)·(−1)^#mirrors` | each mirror flips orientation; the solid stays watertight and positive-|volume| |

The base `V₀, A₀, C₀` are themselves exact closed forms: BOX / NGON prism / coaxial
n-gon LOFT are planar and mesh EXACTLY (tight tol ~1e-6); CYLINDER / SPHERE are curved
and mesh to the deflection bound. The similarity image is computed with a plain fp64
affine (`Aff`) in the harness — NOT `math::Transform`, NOT `gp_Trsf` — so it is a
genuinely third, independent computation. This matters: if the native
`math::Transform` composition had a bug, the native mesh would diverge from the
independent analytic image → DISAGREED, even though the OCCT path might share a
convention.

Only UNIFORM scale is used: an anisotropic scale has no simple closed-form area for a
general solid, and `cc_scale_shape` is uniform by contract. Restricting to uniform
scale is an HONEST SCOPE choice that keeps the area arbiter exact — not a weakened
tolerance.

## 3. The three independent drivings of one chain

Each op is appended to three cumulative representations in lock-step
(`applyTranslate/Rotate/Scale/Mirror`):

- **native** `math::Transform`: `opXf.composedWith(cum)` (argument applied first) →
  driven via `baseShape.located(Location{cum})` + `SolidMesher`.
- **OCCT** `gp_Trsf`: `opT.Multiplied(cum)` → `BRepBuilderAPI_Transform(occBase, cum,
  copy=true)`, measured by `BRepGProp::VolumeProperties` + `SurfaceProperties` +
  `CentreOfMass`.
- **analytic** `Aff` (plain fp64): `affCompose(op, cum)` → applied to the exact base
  `C₀`; volume/area scaled by `S³`/`S²`.

The composition order is identical across all three (op₁ first, then op₂…). Mirror
count parity and the product `S` are tracked for the handedness and volume/area
factors.

## 4. The five-way classifier (mirrors the landed siblings)

| bucket | native valid | native vs analytic image | vs OCCT | verdict |
|---|---|---|---|---|
| AGREED | 1 (watertight, |vol|>0) | match: vol/area/centroid + topology invariant + handedness | match | pass |
| HONESTLY-DECLINED | 0 (collapsed / non-watertight) | — | OCCT valid | pass, logged |
| DISAGREED | 1 | **mismatch** (vol / area / centroid / topology / handedness) | — | **FAIL** |
| ORACLE-INACCURATE | 1 | match | mismatch | pass, logged (OCCT outlier) |
| BOTH-DECLINED | 0 | — | OCCT also invalid | pass, logged |
| ORACLE_UNRELIABLE | — | — | core-case OCCT ≠ closed form | investigate |

`natMatchesAnalytic` = watertight ∧ `|vol|>0` ∧ topology counts equal the base ∧
signed-vol sign equals `sign(base)·mirrorParity` ∧ `relDiff(|vol|, S³V₀) < tol` ∧
`relDiff(area, S²A₀) < tol` ∧ `centErr < centTol`. The handedness check is thus a
first-class part of the AGREE gate: a native mirror that FAILS to flip the signed-vol
sign, or collapses the solid, is a DISAGREE.

**The singular-transform decline (an ORACLE finding, handled honestly).** The
zero-scale DECLINE-exerciser localised a real ORACLE pathology: the NATIVE path meshes
the collapsed solid and returns non-watertight quickly (a clean decline), but OCCT
`BRepBuilderAPI_Transform` fed a zero-scale (non-invertible) `gp_Trsf` on a CURVED base
(reproduced on a CYLINDER, seed `0x7A5C0FFEE2` case 46) **hangs** — it does not return.
Because a zero-scale is not a valid invertible placement, the harness does NOT hand a
singular transform to OCCT (gated on the chain's `singular` flag); the native side
already declines it, and the case is classified BOTH-DECLINED. This is reported as an
honest ORACLE-LIMITATION (in the harness comment and the tasks Results), NOT a native
fault: the native transform layer is vindicated by declining the degenerate input
without hanging. The finding was confirmed with stage-localised stderr instrumentation
(native-mesh-done vs occt-done markers), then the instrumentation was removed.

## 5. Tolerances (fixed, deflection-matched, never widened)

- **planar** base (BOX / NGON / LOFT): the mesh reproduces the solid EXACTLY under a
  similarity → tight `1e-6` (observed dV/dA ~1e-16).
- **curved** base (CYLINDER / SPHERE): the deflection convergence bound at the SCALED
  world feature size, `tol = kCurveC·deflection/(featureSize·min(1,S))` — the
  mesher's own guarantee, never an arbitrary widened value. `centTol =
  tol·max(charLen·S, 1)`.
- **oracle** trust: OCCT must match the closed form to `1e-6` (it is an exact B-rep);
  a core-case OCCT that does not is ORACLE_UNRELIABLE, never laundered into a pass.

## 6. Determinism, coverage, and the bar

- RNG: splitmix64 seeds a xoshiro256** stream from an explicit `FUZZ_SEED`
  (argv[1]/env, fixed default). No clock, no `rand()`. Same seed → byte-identical
  batch (verified by re-running and diffing the `[FUZZ]` lines).
- Coverage: the first `2·#kinds` cases force each op KIND singly (guaranteed
  per-kind coverage); a sparse `i%23==0` tail forces a singular decline-exerciser;
  the rest are random chains of length 1–4.
- Bar: exit 0 IFF `DISAGREED == 0` AND `ORACLE_UNRELIABLE == 0`, with each base
  family (BOX / NGON / CYLINDER / SPHERE / LOFT) and each op kind
  (TRANSLATE / ROTATE / USCALE / MIRROR) having ≥1 AGREED trial, and the mirror
  HANDEDNESS-FLIP positively confirmed ≥1 time — proven over **≥2 seeds**. Any
  DISAGREE / ORACLE-INACCURATE prints seed + case index + base/chain descriptor +
  native/OCCT/analytic measurements.
- No silent caps: a capped or skipped trial is logged; the honest-scope block
  (uniform-scale-only, planar-vs-curved tolerance rationale, the singular decline) is
  always printed.

## 7. What this change deliberately does NOT do

- Does NOT touch `src/native/**` or `src/engine/**` — the native transform path is the
  system under test; a surfaced disagreement is REPORTED with its seed, not fixed here
  (a real native-fault fix would be its own change with a regression trial).
- Does NOT fuzz HEALING — no closed-form ground truth; deferred honestly (its curated
  parity harness stays).
- Does NOT fuzz ANISOTROPIC scale — no closed-form area; `cc_scale_shape` is uniform.
- Does NOT widen any tolerance: each family's tolerance is its tessellation
  convergence bound (planar → machine-tight; curved → deflection-derived).

## 8. Reuse (no new machinery invented)

The RNG helper, the AGREE/DECLINE/DISAGREE/ORACLE-INACCURATE/BOTH-DECLINE printer, the
per-family coverage tally, the base-family generators + closed forms, and the OCCT
`BRepGProp` measurement block are lifted from `native_mass_props_fuzz.mm`. This harness
adds the transform-chain generator, the three-way composed driving, the independent
`Aff` similarity arbiter, the topology-invariance and handedness checks, and the
per-op-kind coverage tally.
