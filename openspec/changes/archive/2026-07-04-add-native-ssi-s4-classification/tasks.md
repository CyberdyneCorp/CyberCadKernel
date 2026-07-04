# Tasks — add-native-ssi-s4-classification (SSI Stage S4-a + S4-b)

Verification levels: **host** = OCCT-free host CTest
(`tests/native/test_native_ssi_s4_classification.cpp`) — analytic closed-form
classification (no substrate) + seeded classification (NUMSCI); **sim** = native-vs-OCCT
classification parity (`tests/sim/native_ssi_s4_classification_parity.mm` +
`scripts/run-sim-native-ssi-s4.sh`) vs `IntAna_QuadQuadGeo` / `IntAna_ResultType`
(analytic) and `IntPatch` / `GeomAPI_IntSS` (seeded). SSI is INTERNAL — **no `cc_*`
entry point is added or changed**. The seeded-path parts are compiled under
**`CYBERCAD_HAS_NUMSCI`** (like S2/S3); the analytic parts need no substrate.
`src/native/**` stays OCCT-free. **No change to `src/native/tessellate`** and **no
marching THROUGH a tangency** (S4-c is out of scope — `NearTangentTransversal` is
classified and handed on, never traced).

## 1. Typed result headers  [host, no substrate]
- [x] 1.1 Add `src/native/ssi/coincidence.h` (header-only, OCCT-free, substrate-free):
  `enum class CoincidenceKind { None, FullSurfaceSame, OverlapSubRegion, Undecided }`
  and `struct CoincidentRegion { CoincidenceKind kind; ParamBox regionA, regionB; }`
  with factories (`::none()`, `::fullSurfaceSame()`, `::overlap(a,b)`, `::undecided()`).
  Uses `native/math` + `native/ssi/patch_bounds.h` (ParamBox) only. (**host**)
- [x] 1.2 Add `src/native/ssi/tangent_contact.h` (header-only, OCCT-free): `enum class
  ContactType { TransversalOnly, TangentPoint, TangentCurve, NearTangentTransversal,
  Undecided }` and `struct TangentContact { ContactType type; Point3 point;
  std::optional<IntersectionCurve> curve; double crossingSine; }` with factories. Uses
  `native/ssi/curve.h` (IntersectionCurve). (**host**)

## 2. S4-a analytic — generalise coincident (FullSurfaceSame) detection  [host, no substrate]
- [x] 2.1 Add closed-form `FullSurfaceSame` predicates covering ALL elementary families,
  each from the surface frame + sizes: plane/plane (same normal ± + same offset),
  cyl/cyl (collinear axes + equal radius — FOLD the existing coaxial-equal case),
  cone/cone (same apex + collinear axis + equal half-angle), sphere/sphere (same centre
  + equal radius — FOLD the existing same-sphere case), torus/torus (same centre+axis +
  equal major + equal minor). Return `CoincidentRegion{FullSurfaceSame}`. (**host**)
- [x] 2.2 Wire into dispatch: add `classify_degeneracy(A,B)` (sibling to
  `intersect_surfaces`) returning `{CoincidentRegion, TangentContact}` for the pair;
  `intersect_surfaces` keeps returning `IntersectionStatus::Coincident` for the
  FullSurfaceSame pairs (status UNCHANGED, now backed by the typed region). The shipped
  same-sphere / coaxial-equal-cyl `Coincident` cases stay bit-identical. (**host** ✓
  regression) 
- [x] 2.3 Host check: each FullSurfaceSame fixture classifies coincident; a
  shifted/rotated/resized near-miss of each classifies `None` (no false coincidence).
  (**host**)

## 3. S4-b analytic — typed tangent-contact classification  [host, no substrate]
- [x] 3.1 Analytic classifier in `tangent_contact.h` / `quadric_pairs` retyping: sphere∩
  sphere `d=R₁+R₂` (external) and `d=|R₁−R₂|` (internal) → `TangentPoint` carrying the
  centre-line contact point (reuse `intersectSphereSphere`'s `makePoint`). (**host**)
- [x] 3.2 Coaxial sphere∩cylinder / sphere∩cone tangent equator (`disc==0`, `Rc==Rs`) →
  `TangentCurve` carrying the single tangent Circle (retype the existing single-circle
  result); plane tangent to a cylinder (parallel, touching) → `TangentCurve` with the
  ruling Line; plane tangent to a sphere → `TangentPoint`. (**host**)
- [x] 3.3 Host check: each analytic tangent fixture yields the right `ContactType` and
  the emitted point/curve lies on BOTH surfaces ≤ tol; analytic tangency NEVER returns
  `NearTangentTransversal` / `Undecided`. (**host**)

## 4. S4-a seeded — coincident-patch detector  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 In the S2 pass (`seeding.cpp`), add a patch-agreement test on a candidate
  region: grid-sample; a sample AGREES iff on-both residual ≤ `onSurfTol` AND ‖n_A×n_B‖
  ≤ `tangentSinTol`. If the whole grid agrees, GROW the region (edge bisection) to the
  agreement boundary and, if it closes robustly on both surfaces, emit
  `CoincidentRegion{OverlapSubRegion, regionA, regionB}` and SUPPRESS seeds/march inside
  it. Fuzzy/partial boundary or ambiguous domain-edge touch → `Undecided`. (**host**)
- [x] 4.2 Extend `SeedSet` (`seed.h`): add `std::vector<CoincidentRegion>
  coincidentRegions;` (populated by 4.1). Keep all existing fields. (**host**)
- [x] 4.3 Host check (NUMSCI): two identical freeform patches overlapping on a
  sub-rectangle → one `OverlapSubRegion` whose delimited bounds match the constructed
  overlap within tol, no spurious seeds inside; a partial/fuzzy overlap → `Undecided`
  (no fabricated region). (**host**)

## 5. S4-b seeded — differential-geometry tangent classifier  [CYBERCAD_HAS_NUMSCI]
- [x] 5.1 Replace `refineRegion`'s bare `nearTangent = true; return false;` (the
  `sinAngle < tangentSinTol` branch) with a `TangentContact` built from the relative
  second fundamental form `H = II_A − II_B` in the shared tangent-plane basis (analytic
  second derivatives for elementary operands; scale-derived finite-difference jet of
  `SurfaceAdapter.point` otherwise): sign-definite → `TangentPoint` (+point); rank-1 →
  `TangentCurve` (+locus/null-dir); indefinite → `NearTangentTransversal` (S4-c gap,
  handed on, NOT traced); within the curvature-noise band → `Undecided`. No seed
  fabricated for any. (**host**)
- [x] 5.2 Extend `SeedSet` (`seed.h`): add `std::vector<TangentContact> tangentContacts;`
  (one per dropped near-tangent region) and KEEP `int deferredTangent` as a compatibility
  summary count (still `++`-ed alongside pushing the typed contact). (**host**)
- [x] 5.3 Host check (NUMSCI): internally-tangent spheres via the seeded path →
  `TangentPoint`; a cylinder tangent along a plane → `TangentCurve`; a shallow grazing
  crossing (indefinite relative II) → `NearTangentTransversal` (handed on, verified NOT
  traced — no WLine fabricated through it); a near-degenerate jet → `Undecided`. The
  curvature band is model-scale-derived, NEVER hand-tuned to force a verdict. (**host**)

## 6. Marching — type the stop reason (additive, still stops at the tangency)  [CYBERCAD_HAS_NUMSCI]
- [x] 6.1 Extend `marching.h` `WLine`: add `std::optional<TangentContact> stopReason;`
  set (via the 5.1 seeded classifier at the stop point) when `status ==
  TraceStatus::NearTangent`. `TraceSet.nearTangentGaps` (count) UNCHANGED; the typed
  contacts additionally retained for reporting. The tracer STILL stops at the tangency —
  it does NOT step through (S4-c). (**host**)
- [x] 6.2 Host check (NUMSCI): a march that stops at a tangency carries a typed
  `stopReason`; the node count is unchanged from the pre-typing behaviour (purely
  additive — no extra points traced past the tangency). (**host**)

## 7. Undecided → OCCT rule (honest, everywhere)
- [x] 7.1 Confirm `src/native/**` NEVER links OCCT and returns `Undecided`/`None`/empty
  on every non-robust classification (ambiguous jet, undelimitable overlap,
  within-noise near-tangent). Document the engine-owned OCCT fallback + self-verify
  contract in the `coincidence.h` / `tangent_contact.h` headers and the `native-ssi`
  namespace docs. (**host** ✓ Undecided fixtures return no fabricated verdict)

## 8. Verification (two gates)
- [x] 8.1 Host suite `tests/native/test_native_ssi_s4_classification.cpp`: S4-a analytic
  FullSurfaceSame + near-miss `None`; S4-a seeded `OverlapSubRegion` + `Undecided`; S4-b
  analytic `TangentPoint` / `TangentCurve` (point/curve on both surfaces ≤ tol); S4-b
  seeded `TangentPoint` / `TangentCurve` / `NearTangentTransversal` / `Undecided`;
  marching typed `stopReason`. Full CTest green NUMSCI ON and OFF (seeded assertions
  absent with NUMSCI off). No OCCT; no tolerance weakened. (**host**)
- [x] 8.2 Sim parity `tests/sim/native_ssi_s4_classification_parity.mm` +
  `scripts/run-sim-native-ssi-s4.sh` (modelled on `run-sim-native-ssi-seeding-parity.sh`):
  analytic pairs vs `IntAna_QuadQuadGeo` / `IntAna_ResultType` (`FullSurfaceSame`↔`Same`,
  `TangentPoint`↔`Point`, `TangentCurve`↔tangent `Line`/`Circle`, `None`↔`Empty`/proper
  transversal); seeded pairs vs `IntPatch` / `GeomAPI_IntSS` (coincident/tangent
  restriction agreement); report per-pair classification agreement + the count still
  deferred to S4-c → OCCT; run via `xcrun simctl spawn <booted udid>`. (**sim**)
- [ ] 8.3 `openspec validate add-native-ssi-s4-classification --strict` green; update
  `SSI-ROADMAP.md` S4 (S4-a coincident-region + S4-b typed tangent-contact landed at the
  bar with measured classification agreement; S4-c marching-through-tangency stays the
  tail), and `ROADMAP.md` / `NATIVE-REWRITE.md` / `README.md` where they cite S4.

## Deferred to S4-c / later (NOT in this change — honest)

- [ ] **S4-c: marching THROUGH a tangency / fabricating a curve across a degeneracy** —
  near-tangent stepping, higher-order predictor, branch-point splitting,
  self-intersection resolution. `NearTangentTransversal` is classified and handed on →
  OCCT; NEVER traced through here. The hard core of the moat.
- [ ] **Consuming `CoincidentRegion` / `TangentCurve` in a curved boolean** (overlap
  handling, tangent-seam trimming) → a later S5 stage.
- [ ] **Non-robustly-decidable classifications** (ambiguous local jet, undelimitable
  overlap, within-noise near-tangent) → `Undecided` → OCCT (engine self-verify),
  reported with the measured gap, never faked.
