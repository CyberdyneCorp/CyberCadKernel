# Tasks — add-native-ssi-analytic (SSI Stage S1)

Verification levels: **host** = OCCT-free host CTest (analytic known-conic checks +
"every sampled point lies on both surfaces" + NOT-ANALYTIC reason checks);
**sim** = native-vs-OCCT `GeomAPI_IntSS` parity on the simulator (kind + sampled
distance + branch count within tol). SSI is INTERNAL — no `cc_*` entry point is
added or exercised; parity is asserted at the `cybercad::native::ssi` C++ boundary,
exactly like native-math / native-topology.

**S1 verification (all green):** host `test_native_ssi` CTest 19/23 (NUMSCI OFF)
and 20/24 (NUMSCI ON) — **11 host cases, 0 failed**; sim parity harness
`scripts/run-sim-native-ssi.sh` — **18 pairs, 0 failed** vs OCCT `GeomAPI_IntSS`
(on-surface / coincidence deltas ≤ ~4e-15, all inside each pair's tol). No
regressions: `run-sim-suite.sh` 221/221.

**Note on the reason taxonomy.** The design listed a fine-grained
`NotAnalyticReason` enum (FreeformSurface / NonCoaxialQuadric / OutOfScopePair /
NearTangentOrCoincident / ObliquePlaneTorus / SelfVerifyFailed). The shipped result
model (`curve.h`) instead carries a single `IntersectionStatus` (Ok /
NoIntersection / Coincident / **NotAnalytic**); the specific out-of-scope reason is
documented per dispatch branch rather than encoded as an enum value. The CONTRACT is
identical — `NotAnalytic` + empty `curves` is the honest "defer to S2/S3/OCCT"
signal — so the reason-specific sub-tasks are marked done against that status, with
this representational difference recorded honestly.

## 1. Module skeleton + result type
- [x] 1.1 Create `src/native/ssi/` with `native_ssi.h` (aggregate header +
  namespace doc) and `curve.h` (`IntersectionCurve`, `CurveKind`,
  `IntersectionStatus`, `IntersectionResult`). OCCT-free. (**host**) — result type is
  `IntersectionResult{status, curves}`; the `ssi_result.h` role is fulfilled by
  `curve.h`.
- [x] 1.2 Analytic conic descriptor `IntersectionCurve` (frame `Ax3` + radius /
  semi-axes a,b / focal / hyperbola branch) with a `value(t)` evaluator mapping 1:1
  onto OCCT `Geom_Line` / `Circle` / `Ellipse` / `Parabola` / `Hyperbola`. (**host**)

## 2. Pair-dispatch / classifier
- [x] 2.1 `classify`/dispatch (`dispatch.h`) — symmetric (canonicalize operand
  order: plane→sphere→cylinder→cone→torus); routes to a closed-form handler or
  returns `NotAnalytic`. (**host**)
- [x] 2.2 Any pair not enumerated (NURBS/freeform operand, non-coaxial quadric,
  skew cyl∩cyl, cone∩cone, non-coaxial cone∩cyl, torus∩curved, unsafe placement)
  returns `NotAnalytic` with an empty curve list. (**host**) — represented by the
  single `NotAnalytic` status (see the reason-taxonomy note above), not distinct
  enum values.

## 3. Self-verify substrate
- [x] 3.1 Verification harness samples each returned `IntersectionCurve` and asserts
  every sample lies on BOTH input surfaces within a scale-derived tol; the host gate
  (`test_native_ssi.cpp`) enforces the on-surface residual ≤ tol for every emitted
  curve (max observed ~4e-15), and the sim harness re-checks it against the OCCT
  oracle. (**host**) — verify is asserted at the test gate; every S1 handler is
  closed-form-correct by construction, so no in-solver downgrade path fired.

## 4. Plane pairs (solver-free)
- [x] 4.1 **plane ∩ plane** → line (parity: Line, onSurf 0, coin 0). (**host** + **sim**)
- [x] 4.2 **plane ∩ sphere** → circle (parity: Circle, onSurf 3.79e-15). (**host** + **sim**)
- [x] 4.3 **plane ∩ cylinder** → ⟂ circle / ∥ 2 lines / oblique ellipse
  (parity: Circle / Line·Line / Ellipse, onSurf ≤ 1.91e-15). (**host** + **sim**)
- [x] 4.4 **plane ∩ cone** → circle / ellipse / parabola / hyperbola(×2 nappes)
  (parity: all four conic kinds match OCCT; steep cone → 2 Hyperbola branches). (**host** + **sim**)

## 5. Plane ∩ torus (closed-form families — solver-free, NO numerics needed)
- [x] 5.1 **plane ∩ torus** — the two closed-form families are native
  (`plane_torus.h`): plane ⟂ axis → 1–2 concentric circles; plane ∋ axis → 2 meridian
  circles. Parity: 2 Circle/Circle for both families (onSurf ≤ 2.84e-15). (**host** +
  **sim**) — NOTE: implemented purely over `native-math` (no `native-numerics`
  quartic root-find), so it builds with **NUMSCI OFF**; the general-quartic path in
  the original design was not needed for the closed-form families.
- [x] 5.2 General OBLIQUE plane ∩ torus (spiric quartic, no rational conic
  decomposition) returns `NotAnalytic` — not faked. (**host**)

## 6. Quadric pairs (coaxial / parallel, solver-free)
- [x] 6.1 **sphere ∩ sphere** → circle (parity Circle; OCCT arc-splits into 2, TYPE
  matches; onSurf 4.12e-15). (**host** + **sim**)
- [x] 6.2 **sphere ∩ cylinder, COAXIAL** → 2 circles (parity Circle/Circle,
  onSurf 1.88e-15). (**host** + **sim**)
- [x] 6.3 **sphere ∩ cone, COAXIAL** → circle (parity Circle/Circle; OCCT emits an
  extra arc split; onSurf 3.14e-15). (**host** + **sim**)
- [x] 6.4 **cylinder ∩ cylinder, COAXIAL or PARALLEL** → coincident / parallel
  lines (parity: coaxial → coincident detected, nat=0 occt=0; parallel → Line/Line,
  onSurf 1.26e-15). (**host** + **sim**)
- [x] 6.5 **cylinder ∩ cone, COAXIAL** → circle(s) (parity Circle/Circle; OCCT
  arc-splits; onSurf 1.79e-15). (**host** + **sim**)

## 7. Deferral seam (NOT-ANALYTIC, never faked)
- [x] 7.1 Out-of-scope pairs return `NotAnalytic` with empty `curves` — host case
  `deferred_not_analytic` asserts this; the sim harness confirms **skew cyl∩cyl**
  returns `NotAnalytic` (native) while OCCT emits 7 Ellipse curve(s) — deferred by
  design. (**host** + **sim**)
- [x] 7.2 Documented that `analytic == false` (`NotAnalytic`) is the contract with
  S2/S3/OCCT — the S5 curved-boolean caller MUST route these to marching or OCCT
  (`native_ssi.h` namespace doc + SSI-ROADMAP S1 entry). (**host**)

## 8. Verification (two gates)
- [x] 8.1 Host analytic suite: for every supported pair, curve kind + parameters
  match the closed form AND all sampled points lie on both surfaces within tol;
  out-of-scope pairs return `NotAnalytic`. No OCCT. `test_native_ssi` **11 cases, 0
  failed**. (**host**)
- [x] 8.2 Sim native-vs-OCCT `GeomAPI_IntSS` parity: same operands built as OCCT
  `Geom_*Surface`, run `GeomAPI_IntSS`, compare native curve(s) (kind, on-surface
  residual, coincidence, coverage, branch count) within tol at the SSI C++ boundary.
  `native_ssi_parity.mm` / `run-sim-native-ssi.sh` **18 pairs, 0 failed**. (**sim**)
- [x] 8.3 `openspec validate add-native-ssi-analytic --strict` green; S1 marked done
  and the S2 on-ramp noted in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` /
  `docs/STATUS-phase-4.md` / root `README.md`. (**host**)

## Deferred to S2–S4 / OCCT (NOT in the S1 analytic family — honest `NotAnalytic`)

These are NOT degree-≤2 closed-form reductions; they need subdivision seeding (S2),
marching (S3) and/or tangent-robustness (S4), or stay OCCT-backed:

- [ ] **skew cylinder ∩ cylinder** — general skew axes give a planar QUARTIC, no
  rational conic decomposition → `NotAnalytic` (sim: OCCT emits 7 Ellipse curves).
  → S2/S3.
- [ ] **general cone ∩ cone** (non-coaxial) — quartic space curve → S2/S3.
- [ ] **non-coaxial cone ∩ cylinder / sphere ∩ cylinder / sphere ∩ cone** — the
  NON-coaxial placements (only the coaxial/parallel families are S1) → S2/S3.
- [ ] **oblique plane ∩ torus** (spiric quartic) and **torus ∩ any curved** → S2/S3.
- [ ] **any freeform (NURBS / B-spline / Bézier) surface pair** → S2 seeding + S3
  marching.
- [ ] **near-tangent / coincident-curved** robustness on the above → S4 (the moat) +
  OCCT fallback.
