# Design — add-native-ssi-s4-classification (SSI Stage S4-a + S4-b)

## Context

The native SSI stack (S1 analytic → S2 seeding → S3 marching) is transversal-first
and hits the **S4 boundary** (`SSI-ROADMAP.md` S4 — tangent / degeneracy robustness,
"the moat") with two blunt hand-offs:

1. **`SeedSet.deferredTangent` is a bare integer.** `seeding.cpp refineRegion` drops a
   refine whose solution is near-tangent (‖n₁×n₂‖ < `SeedOptions.tangentSinTol`,
   default 1e-3) and merely does `++out.deferredTangent`. All degeneracy types collapse
   to one count — the downstream cannot distinguish an isolated tangent point, a
   full-curve tangency, or a near-tangent transversal.
2. **Coincidence is analytic-only and incomplete.** `quadric_pairs.h` returns
   `IntersectionStatus::Coincident` for same-sphere and coaxial-equal cylinder, and
   `CurveKind::Point` for tangent sphere∩sphere (`d=R₁+R₂`). It is a bare status/flag
   (no typed region), it does not cover every analytic family (same plane, coaxial-equal
   cone, same torus), and there is NO coincidence detection on the seeded path — two
   coincident freeform patches yield spurious seeds/marches.

This change adds the two **classification layers** that type the S4 boundary. It is
**detection + classification ONLY** — it does not step through a tangency and does not
fabricate a curve across a degeneracy (that is **S4-c**, out of scope). The method is
clean-room; OCCT (`IntAna_QuadQuadGeo` / `IntPatch` / `GeomAPI_IntSS`) is the
verification ORACLE only.

## Goals / Non-Goals

**Goals**
- (S4-a) A native, OCCT-free `CoincidentRegion` result (`FullSurfaceSame` |
  `OverlapSubRegion{param bounds on A and B}`), populated by BOTH the analytic path
  (generalised closed-form full-surface-same detection across the elementary families)
  and the seeded path (patch-agreement detection delimiting an overlap sub-region), or
  `Undecided` → OCCT when a region cannot be robustly delimited.
- (S4-b) A native `TangentContact` result typed as exactly one of `TransversalOnly` /
  `TangentPoint` / `TangentCurve` / `NearTangentTransversal`, populated by the analytic
  (closed-form) and seeded (local differential-geometry) paths, emitting the point/curve
  where determinable, returning `Undecided` → OCCT when the local jet is ambiguous, and
  handing `NearTangentTransversal` on to S4-c → OCCT WITHOUT tracing through it.
- Type the S3 hand-off: `TraceStatus::NearTangent` gains a typed `TangentContact`
  companion (WHY the march stopped), and `SeedSet.deferredTangent` gains a per-region
  typed `TangentContact` list; the integer counter stays as a compatibility summary.
- Report native-vs-OCCT classification agreement; call out what still defers to S4-c.

**Non-Goals (deferred — never faked here)**
- **S4-c: marching THROUGH a tangency / fabricating a curve across a degeneracy.**
  A `NearTangentTransversal` is classified and handed on → OCCT; it is NEVER traced
  through in this change. Near-tangent STEPPING, higher-order predictors, branch-point
  splitting, self-intersection resolution → S4-c + OCCT. This is the hard core.
- **Building a boolean / trimmed face from a `CoincidentRegion` or `TangentCurve`.**
  This change EMITS the typed result; consuming it in a curved boolean (overlap
  handling, tangent-seam trimming) is a later S5 stage.
- **Any change to `src/native/tessellate`, the `cc_*` ABI, or the S1 transversal
  curve handlers themselves.** The analytic transversal outputs (Line/Circle/…) are
  unchanged; S4-a/b only add the coincident/tangent TYPING alongside them.
- **Guessing a region or a contact type.** Ambiguous → `Undecided`/empty → OCCT.

## Module shape

```
src/native/ssi/coincidence.h        [header-only, OCCT-free, substrate-free]
  enum class CoincidenceKind { None, FullSurfaceSame, OverlapSubRegion, Undecided }
  struct CoincidentRegion { kind; ParamBox regionA, regionB; /* bounds when OverlapSubRegion */ }
  // analytic closed-form full-surface-same predicates (same plane / coaxial-equal
  // cyl / coaxial-equal cone / same sphere / same torus) → CoincidentRegion

src/native/ssi/tangent_contact.h    [header-only, OCCT-free, substrate-free]
  enum class ContactType { TransversalOnly, TangentPoint, TangentCurve,
                           NearTangentTransversal, Undecided }
  struct TangentContact {
    ContactType type;
    Point3 point;                     // set for TangentPoint
    std::optional<IntersectionCurve> curve;  // set for TangentCurve when closed-form
    double crossingSine;              // ‖n₁×n₂‖ at the contact (the witness)
  }
  // analytic closed-form classifier (sphere∩sphere d=R₁+R₂; coaxial tangent circle;
  // plane tangent to quadric) → TangentContact

src/native/ssi/dispatch.h           [extend — attach the typed results to S1 families]
  // intersect_surfaces stays the transversal router; a sibling
  // classify_degeneracy(A,B) returns {CoincidentRegion, TangentContact} for the pair
  // (analytic, closed form). Coincident status now carries the CoincidentRegion.

src/native/ssi/seeding.{h,cpp}      [CYBERCAD_HAS_NUMSCI — the seeded classifiers]
  // refineRegion near-tangent branch: instead of a bare ++deferredTangent, build a
  // TangentContact by local differential geometry and record it.
  // a NEW coincident-patch detector run before/within seeding: sample the candidate
  // region on both surfaces; if points AND normals agree over the whole patch, emit a
  // CoincidentRegion(OverlapSubRegion) and suppress spurious seeds there.

src/native/ssi/seed.h               [extend — carry the typed results]
  // SeedSet gains: std::vector<TangentContact> tangentContacts;  (typed dropped regions)
  //               std::vector<CoincidentRegion> coincidentRegions;
  //               int deferredTangent;  // kept: compatibility summary count

src/native/ssi/marching.h           [extend — type the stop reason, additive]
  // WLine gains: std::optional<TangentContact> stopReason;  (set when status==NearTangent)
  // TraceSet keeps nearTangentGaps (count) + gains the typed contacts it stopped on.
```

`src/native/**` stays OCCT-free. The analytic types + predicates are header-only and
substrate-free (they need only `src/native/math`). The seeded classifiers live in
`seeding.cpp` under `CYBERCAD_HAS_NUMSCI` (they run within the refine/seed pass).

## S4-a — Coincident / overlapping-surface detection + typed region

### Typed result

```
CoincidenceKind::None            — surfaces are not coincident (transversal / disjoint)
CoincidenceKind::FullSurfaceSame — the two surfaces are the SAME locus (analytic)
CoincidenceKind::OverlapSubRegion— they coincide on a delimited param sub-region
CoincidenceKind::Undecided       — coincidence suspected but not robustly delimited → OCCT
```

`CoincidentRegion` carries `regionA` / `regionB` (`ParamBox` on each surface) for
`OverlapSubRegion`; for `FullSurfaceSame` the region is each surface's full domain.

### Analytic path (closed form — generalise the existing detection)

The existing `IntersectionStatus::Coincident` in `quadric_pairs.h` (same-sphere,
coaxial-equal cylinder) is preserved and GENERALISED to a complete, consistent family,
each decided from the surface frames + sizes:

| Pair | FullSurfaceSame test (closed form) |
|---|---|
| plane / plane | same normal (± sign) AND same signed offset |
| cylinder / cylinder | collinear axes AND equal radii (the existing coaxial-equal case, folded) |
| cone / cone | same apex AND collinear axes AND equal half-angle |
| sphere / sphere | same centre AND equal radius (the existing same-sphere case, folded) |
| torus / torus | same centre + axis AND equal major AND equal minor radius |

Each returns `CoincidentRegion{FullSurfaceSame}` and the dispatcher reports
`IntersectionStatus::Coincident` (unchanged status; now carrying the typed region).
No analytic pair produces an `OverlapSubRegion` (an infinite elementary surface is
either the same locus or not); `OverlapSubRegion` is the SEEDED-path outcome (bounded
freeform patches).

### Seeded path (patch agreement — delimit the overlap sub-region)

Within the S2 pass, BEFORE accepting isolated seeds on a candidate region, run a
**patch-agreement test**: sample the candidate region on a small grid; at each sample
project the A-point onto B and check (i) the on-both residual ‖A.point − B.point‖ ≤ tol
AND (ii) the normals agree ‖n_A × n_B‖ ≤ `tangentSinTol` (surfaces locally identical,
not just crossing). If the WHOLE grid agrees, the patch is a coincident overlap:

1. **Grow** the agreeing region outward in param space (bisection on each domain edge)
   until an edge sample stops agreeing → the delimited `regionA` / `regionB` bounds.
2. If the grown region closes to a well-defined `ParamBox` on both surfaces (every
   boundary sample agrees inside, disagrees just outside, within a tolerance band),
   emit `CoincidentRegion{OverlapSubRegion, regionA, regionB}` and SUPPRESS seeding /
   marching inside it (no spurious seeds, no spurious march).
3. If the boundary cannot be delimited robustly (agreement is partial, the boundary is
   fuzzy, or the region touches a domain edge ambiguously) → `Undecided` → OCCT. Never
   a guessed rectangle.

## S4-b — Tangent-contact CLASSIFICATION

### Typed result

```
ContactType::TransversalOnly        — no tangency (‖n₁×n₂‖ ≥ tol); normal path handles it
ContactType::TangentPoint           — isolated 0-dim contact; `point` emitted
ContactType::TangentCurve           — tangent along a whole curve; `curve` emitted if closed-form,
                                      else the type + seed locus flag the curve's existence
ContactType::NearTangentTransversal — grazes but CROSSES → S4-c gap, handed on → OCCT (NOT traced)
ContactType::Undecided              — local jet ambiguous → OCCT
```

The classifier is TOTAL over the contact space so it never silently drops a case; the
honest exits are `NearTangentTransversal` (a real S4-c gap) and `Undecided`.

### Analytic path (closed form — start here)

Decidable directly from the family:
- **sphere ∩ sphere, `d = R₁+R₂` (external) or `d = |R₁−R₂|` (internal):** the existing
  `CurveKind::Point` — classify as `TangentPoint`, emit the contact point (on the
  centre line, already computed by `intersectSphereSphere`), verified on both spheres.
- **coaxial sphere ∩ cylinder / cone tangent (the `disc == 0` equator, `Rc == Rs`):**
  the surfaces touch along a whole circle → `TangentCurve`, emit that Circle
  (`intersectSphereCylinderCoaxial` already returns the single tangent circle — retype
  it as a tangent curve rather than a transversal intersection).
- **plane tangent to sphere / cylinder / cone (the degenerate conic-section cases —
  tangent point on a sphere, tangent ruling line on a cylinder):** `TangentPoint`
  (plane∩sphere at distance `= R`) or `TangentCurve` (plane parallel to and touching a
  cylinder → one ruling line). Emit the point/line from the existing degenerate arms of
  the plane∩quadric handlers.

Analytic tangency is exact, so it NEVER returns `NearTangentTransversal` or `Undecided`
— those are seeded-path outcomes.

### Seeded path (local differential geometry)

At a refine solution with ‖n₁×n₂‖ < `tangentSinTol` (the current `nearTangent` branch),
the surfaces are tangent (share a tangent plane) at that point. Classify by the
**relative second fundamental form** — the difference of the two surfaces' shape as
seen in the shared tangent plane (relative normal curvature). Concretely, in the shared
tangent-plane basis (u,w), form the relative Hessian `H = II_A − II_B` (each surface's
second fundamental form evaluated from its analytic/numerical second derivatives at the
contact):

- `H` sign-definite (both eigenvalues same sign, magnitudes above a curvature-noise
  band) ⇒ the surfaces touch and separate on all sides ⇒ **`TangentPoint`** (isolated
  contact); emit the contact point.
- `H` has exactly one near-zero eigenvalue (rank-1, the other well above noise) ⇒ the
  contact is tangent along the null direction ⇒ **`TangentCurve`**; flag the curve
  (seed locus + null direction; the full curve is an S3/analytic emission where
  available, else flagged for OCCT).
- `H` indefinite (eigenvalues of opposite sign, both above noise) ⇒ the surfaces cross
  through the tangent plane ⇒ **`NearTangentTransversal`** — a grazing crossing → S4-c
  gap, handed on → OCCT, NOT traced through here.
- `H` near-zero in BOTH eigenvalues, or within the noise band so the verdict is not
  robust ⇒ **`Undecided`** → OCCT. Report, do not guess.

The curvature-noise band scales with the model scale and the numerical second-derivative
step (documented, tolerance-scaled — NEVER hand-tuned to force a verdict). The second
derivatives come from the analytic surfaces where the operand is elementary, and from a
finite-difference jet of the `SurfaceAdapter.point` otherwise (the same evaluator the
refine uses).

### The `deferredTangent` replacement

`refineRegion`'s `if (sinAngle < opts.tangentSinTol) { nearTangent = true; return false; }`
becomes: classify the contact into a `TangentContact`, push it onto
`SeedSet.tangentContacts`, and keep `++deferredTangent` as a compatibility summary
count. A `TangentPoint` additionally carries its emitted point; a `TangentCurve` its
locus; a `NearTangentTransversal` / `Undecided` records the type only (still an S4-c/OCCT
gap). No seed is fabricated for any of them.

## The undecided → OCCT rule (honest, everywhere)

Native classification returns `Undecided` / empty / `None` whenever it cannot robustly
decide:
- an ambiguous local jet (curvature within the noise band),
- a coincident overlap whose boundary cannot be delimited,
- a near-tangent whose transversal-vs-tangent verdict is within noise.

`src/native/**` NEVER links OCCT and NEVER guesses. The ENGINE owns the OCCT fallback +
self-verify: it consumes the typed result, and on `Undecided`/empty it falls back to
OCCT and reports the measured gap. A correct "undecided → OCCT" is a first-class outcome
(the same "deferral-is-data" stance S1's `NotAnalytic` and S2's `deferredTangent` take);
a fabricated classification is a task failure.

## Marching hand-off (typed, additive — still stops at the tangency)

`marching.h` gains a typed companion to the existing honest stop:
- `WLine.stopReason` (`std::optional<TangentContact>`) set when `status == NearTangent`
  — the classification of WHY the march stopped (point / curve / near-tangent
  transversal), computed at the stop point by the same seeded classifier.
- `TraceSet.nearTangentGaps` (count) is unchanged; the typed contacts are additionally
  retained for reporting.

The tracer STILL stops at the tangency and marches only UP TO it — it does NOT step
through (S4-c). This is purely additive typing of the already-honest S3 gap.

## Classification-vs-deferred scope (honest)

| Configuration | classification | consumed by |
|---|---|---|
| Analytic same plane / coaxial-eq cyl / coaxial-eq cone / same sphere / same torus | `CoincidentRegion{FullSurfaceSame}` | downstream (typed), no march |
| Coincident freeform patch, boundary delimitable | `CoincidentRegion{OverlapSubRegion}` | downstream (typed), seeds suppressed |
| Coincident freeform, boundary NOT delimitable | `Undecided` | → OCCT |
| sphere∩sphere `d=R₁+R₂` / `d=|R₁−R₂|` | `TangentContact{TangentPoint}` + point | downstream (typed) |
| coaxial sphere∩cyl/cone tangent circle; plane tangent along a cyl ruling | `TangentContact{TangentCurve}` + curve | downstream (typed) |
| Seeded near-tangent, relative II sign-definite | `TangentContact{TangentPoint}` + point | downstream (typed) |
| Seeded near-tangent, relative II rank-1 | `TangentContact{TangentCurve}` + locus | downstream (typed) |
| Seeded near-tangent, relative II indefinite (grazes + crosses) | `TangentContact{NearTangentTransversal}` | → **S4-c** → OCCT (NOT traced) |
| Seeded near-tangent, jet within noise band | `TangentContact{Undecided}` | → OCCT |
| S3 march stops at a tangency | `WLine.stopReason` typed | reported; march stops (NOT through) |
| Marching THROUGH a tangency / fabricating a curve across a degeneracy | — | **S4-c, OUT OF SCOPE** → OCCT |

## Verification model (two gates)

- **Host (no OCCT), analytic + constructed oracles.** New
  `tests/native/test_native_ssi_s4_classification.cpp`:
  - **S4-a analytic:** same plane, coaxial-equal cylinder, coaxial-equal cone, same
    sphere, same torus → `CoincidentRegion{FullSurfaceSame}`; a shifted/rotated
    near-miss of each → `None` (not a false coincidence). The existing same-sphere /
    coaxial-cyl `IntersectionStatus::Coincident` cases stay green (regression).
  - **S4-a seeded** (NUMSCI): two identical Bézier/B-spline patches offset to overlap on
    a sub-rectangle → `CoincidentRegion{OverlapSubRegion}` whose delimited bounds match
    the constructed overlap within tol; a partial/fuzzy overlap → `Undecided`.
  - **S4-b analytic:** sphere∩sphere `d=R₁+R₂` → `TangentPoint`, point on both spheres
    ≤ tol; coaxial sphere∩cylinder tangent equator → `TangentCurve` with the circle on
    both surfaces ≤ tol; plane tangent to a cylinder → `TangentCurve` (ruling line).
  - **S4-b seeded** (NUMSCI): two spheres nudged to internal tangency via the seeded
    path → `TangentPoint`; a cylinder resting tangent along a plane → `TangentCurve`;
    a shallow grazing crossing (indefinite relative II) → `NearTangentTransversal`
    (handed on, NOT traced); a near-degenerate jet → `Undecided`.
  - Full CTest green NUMSCI ON and OFF (the seeded assertions absent with NUMSCI off,
    like the S2/S3 tests). No OCCT linked; no tolerance weakened.
- **Sim native-vs-OCCT — IntAna classification parity.** New
  `tests/sim/native_ssi_s4_classification_parity.mm` + `scripts/run-sim-native-ssi-s4.sh`
  (modelled on `run-sim-native-ssi-seeding-parity.sh`): for each fixture assert the
  native classification AGREES with OCCT —
  - analytic pairs vs `IntAna_QuadQuadGeo` / `IntAna_ResultType`: our `FullSurfaceSame`
    ↔ OCCT `Same`; our `TangentPoint` ↔ OCCT `Point`; our `TangentCurve` ↔ OCCT
    `Line` / `Circle` (a tangent line/circle); `None` where OCCT is `Empty` / a proper
    transversal `Circle`/`Ellipse`;
  - seeded pairs vs `IntPatch` / `GeomAPI_IntSS`: coincident overlap ↔ OCCT reporting a
    coincident/same restriction; tangent point/curve ↔ OCCT tangent restriction;
  report per-pair classification agreement + the count still deferred to S4-c → OCCT.
  Parity is a REPORTED figure; whatever the native side leaves `Undecided` falls back to
  OCCT and is reported with the measured gap. Run via `xcrun simctl spawn <booted udid>`.

## Decisions

- **Type the boundary, don't march it.** The whole change is DETECTION + CLASSIFICATION.
  `NearTangentTransversal` is a first-class classified outcome handed to S4-c → OCCT; we
  deliberately do NOT step through a tangency here (that is the hard S4-c core). This is
  the honest, tractable slice of the moat.
- **`CoincidentRegion` and `TangentContact` as typed results, not flags.** Downstream
  (booleans, S5) needs to know WHICH degeneracy it hit. A bare `deferredTangent` int and
  a bare `Coincident` status lose that; typed results carry the point/curve/region the
  consumer needs and make the undecided case explicit.
- **Analytic first, then seeded.** Analytic tangency/coincidence is exact and cheap —
  it seeds the classifier's ground truth and covers the elementary families with no
  substrate. The seeded differential-geometry classifier is the general (freeform)
  extension, gated behind the `Undecided` band so it never guesses.
- **Relative second fundamental form for the seeded tangent type.** The point-vs-curve
  -vs-crossing distinction IS the sign/rank structure of the relative normal curvature —
  the mathematically correct local invariant. Using the local jet (not a global search)
  keeps it cheap and per-contact; the noise band + `Undecided` keep it honest.
- **Reuse the existing tolerance seams.** `tangentSinTol` (the near-tangent gate) is
  reused unchanged; the coincidence residual reuses the seed `onSurfTol`; the curvature
  band is model-scale-derived like the other S2/S3 tolerances. No new hand-tuned
  constant, no weakened tolerance.
- **Additive, ABI-stable, tessellator-untouched.** New header types + additive fields on
  `SeedSet` / `WLine`; no `cc_*` change; `src/native/tessellate` untouched; seeded parts
  under `CYBERCAD_HAS_NUMSCI` exactly like S2/S3.

## Risks / Trade-offs

- **Seeded overlap-boundary delimitation.** A high-curvature coincident patch can be
  under/over-grown. Mitigation: emit `OverlapSubRegion` only when the boundary closes
  robustly (inside agrees, just-outside disagrees within the band); else `Undecided` →
  OCCT. The sim gate measures the delimited bounds vs OCCT. Never a guessed rectangle.
- **Point-vs-crossing misread near a degenerate relative II.** When the relative second
  fundamental form is near-degenerate the type is genuinely ambiguous. Mitigation: the
  `Undecided` noise band (report, don't guess) + the engine self-verify. Accepted — a
  correct `Undecided → OCCT` is a fine outcome.
- **Generalising analytic `Coincident` must not regress.** The shipped same-sphere /
  coaxial-cyl detection is pinned green; the new families (plane, cone, torus) are added
  additively and cross-checked vs OCCT `Same`. Accepted.
- **Second-derivative jet cost/accuracy for freeform.** The finite-difference jet adds a
  few extra `point` evaluations per near-tangent contact and carries a truncation error.
  Mitigation: analytic second derivatives for elementary operands; a scale-derived step
  for freeform; the `Undecided` band as backstop. Accepted.
