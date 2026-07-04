## Why

`SSI-ROADMAP.md` S4 is **the moat** — tangent / degeneracy robustness, OCCT's
decades of tuning. It is explicitly *progressively hardened*; whatever is not robust
falls back to OCCT and is reported with the measured gap. Today the native SSI stack
hits the S4 boundary with two **blunt, information-losing** hand-offs:

- **Seeded path** (`src/native/ssi/seeding.cpp`): a refine that lands near-tangent
  (‖n₁×n₂‖ < `SeedOptions.tangentSinTol`, default 1e-3) is dropped and merely
  **counted** in `SeedSet.deferredTangent`. Downstream sees a bare integer — it
  cannot tell an isolated tangent POINT (a real 0-dim contact worth emitting) from a
  tangent CURVE (the surfaces coincide differentially along a whole seam) from a
  near-tangent TRANSVERSAL that just grazes-and-crosses. Every one is thrown into the
  same counter → OCCT.
- **Coincident detection** exists only in the ANALYTIC path
  (`quadric_pairs.h` / `dispatch.h`: same-sphere, coaxial-equal cylinder →
  `IntersectionStatus::Coincident`; tangent sphere∩sphere at `d=R₁+R₂` →
  `CurveKind::Point`). It is (a) NOT generalised across the analytic families (same
  plane, coaxial-equal cone, same torus are not all folded), and (b) entirely ABSENT
  on the seeded path — two general/quadric surfaces that coincide on a REGION produce
  spurious seeds / a spurious march instead of a typed coincident-region result.

Both gaps lose classification the downstream (booleans, S5) needs. This change adds
the **two CLASSIFICATION LAYERS** that type the S4 boundary honestly — it does NOT
march through a tangency and does NOT fabricate a curve across a degeneracy.

**CRITICAL SCOPE BOUND.** This change is **DETECTION + CLASSIFICATION only**. It does
NOT trace through a tangency and does NOT fabricate an intersection curve across a
degeneracy — that is **S4-c** (the marching-through-tangency core), explicitly OUT OF
SCOPE here. A `NearTangentTransversal` is *classified as such and handed on* (still an
S4-c gap → OCCT), never traced through.

## What Changes

- **(S4-a) Coincident / overlapping-surface detection + a TYPED coincident-region
  result.** Add a `CoincidentRegion` result (the shared locus/region descriptor:
  `FullSurfaceSame`, or an `OverlapSubRegion` with its param bounds on each surface),
  and detect coincidence on BOTH paths:
  - **Analytic** — generalise the existing `IntersectionStatus::Coincident` detection
    across the closed-form families so it is complete and consistent: same plane,
    coaxial-equal cylinder, coaxial-equal cone, same sphere, same torus all fold to a
    `CoincidentRegion` (`FullSurfaceSame`), decided in closed form from the surface
    frames + sizes. The existing `Coincident` status is preserved; the change is that
    it now carries the typed region rather than a bare flag.
  - **Seeded** — detect when two general/quadric surfaces coincide over a PATCH
    (normals AND points agree over a sampled sub-region, not just at isolated seeds)
    and return a `CoincidentRegion` (`OverlapSubRegion` with the delimited param
    bounds) instead of spurious seeds / a spurious march. If the region cannot be
    robustly delimited (partial agreement, ambiguous boundary), return **undecided →
    OCCT**, honestly — never a guessed region.
- **(S4-b) Tangent-contact CLASSIFICATION — a typed classifier replacing the blunt
  `deferredTangent`.** Add a `TangentContact` result that types a contact as exactly
  one of:
  - **`TransversalOnly`** — no tangency at the solution (‖n₁×n₂‖ ≥ tol); the normal
    seeded/analytic path handles it (this arm exists so the classifier is total).
  - **`TangentPoint`** — an isolated 0-dim contact (e.g. spheres touching at
    `d=R₁+R₂`); **emit the point**.
  - **`TangentCurve`** — the surfaces are tangent along a whole curve (e.g. a cylinder
    tangent to a plane along a ruling line); **emit / flag the curve** where the
    analytic family gives it in closed form, else flag its existence with the seed
    locus.
  - **`NearTangentTransversal`** — grazes but CROSSES (a transversal crossing whose
    normals are near-parallel but the contact is not a true tangency); **classified
    and handed to S4-c → OCCT**, never traced through here.
  Populate it from BOTH paths: **analytic** tangent configs are decidable in closed
  form (start there — the sphere∩sphere `d=R₁+R₂` `Point`, the coaxial-tangent-circle
  cases, plane-tangent-to-quadric); **seeded** solutions (‖n₁×n₂‖ < `tangentSinTol`)
  classify by LOCAL DIFFERENTIAL GEOMETRY (the rank / sign structure of the relative
  second fundamental form — relative normal curvature at the contact), returning
  **undecided → OCCT** when the local jet is ambiguous. The seeding pass replaces the
  bare `++deferredTangent` with a typed `TangentContact` recorded per dropped region;
  the integer counter is kept as a compatibility summary.
- **The undecided → OCCT rule (honest, everywhere).** Native classification returns a
  NULL/empty/`Undecided` result whenever it cannot robustly decide — an ambiguous
  local jet, a coincident region whose boundary cannot be delimited, a near-tangent
  whose transversal-vs-tangent verdict is within noise. The ENGINE owns the OCCT
  fallback + self-verify; `src/native/**` stays OCCT-free. A correct
  "deferred/undecided → OCCT" is a fine, first-class outcome; a fabricated
  classification is not.
- **Marching consumes the typed reason (additive).** `marching.h`'s
  `TraceStatus::NearTangent` / `TraceSet.nearTangentGaps` gain a typed companion: a
  march that stops at a tangency records the `TangentContact` classification of WHY it
  stopped (point / curve / near-tangent transversal), so S3's honest gap is now typed
  rather than a bare count. The tracer still stops at the tangency — it does NOT march
  through (that is S4-c).

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-ssi capability with the two S4
CLASSIFICATION LAYERS (coincident-region detection + typed tangent-contact
classification). It adds no new capability spec and no cc_* ABI. -->

### Modified Capabilities
- `native-ssi`: add (S4-a) a native, OCCT-free **coincident / overlapping-surface
  detector** returning a TYPED `CoincidentRegion` (`FullSurfaceSame` |
  `OverlapSubRegion{param bounds}`) on both the analytic and the seeded paths, and
  (S4-b) a native **typed tangent-contact classifier** returning a `TangentContact`
  (`TransversalOnly` | `TangentPoint` | `TangentCurve` | `NearTangentTransversal`)
  that replaces the blunt `SeedSet.deferredTangent` counter with a typed contact per
  dropped region and types the S3 `nearTangentGaps`. Both return **undecided/empty →
  OCCT** whenever a robust decision is not available (the engine owns the fallback +
  self-verify). Marching-through-a-tangency (S4-c) is **explicitly out of scope**:
  `NearTangentTransversal` is classified and handed on, never traced through. No
  `cc_*` ABI change; `src/native/**` stays OCCT-free; the seeded-path parts are
  compiled under `CYBERCAD_HAS_NUMSCI` like S2/S3.

## Impact

- **ABI**: none. SSI is INTERNAL — no `cc_*` entry point, signature, or POD struct
  change. Additive only; the tessellator (`src/native/tessellate`) and the CyberCad
  app are untouched.
- **Build**: adds `src/native/ssi/coincidence.h` (the `CoincidentRegion` type +
  analytic full-surface-same predicates — OCCT-free, substrate-free, header-only) and
  `src/native/ssi/tangent_contact.h` (the `TangentContact` type + the analytic
  classifier — header-only) and the seeded-path classifiers in
  `src/native/ssi/seeding.cpp` (the patch-agreement coincidence detector + the
  differential-geometry tangent classifier at a near-tangent refine), compiled under
  `CYBERCAD_HAS_NUMSCI` like the S2 refine. Extends `dispatch.h` /
  `quadric_pairs.h` to attach the typed `CoincidentRegion` / `TangentContact` to the
  analytic families, and `seed.h` / `marching.h` to carry the typed results. No change
  to `src/native/tessellate`; the analytic parts need no substrate.
- **Verification**: two gates. **Host (no OCCT)** — a new
  `tests/native/test_native_ssi_s4_classification.cpp`: analytic closed-form
  classification (same-plane/coaxial-cyl/coaxial-cone/same-sphere/same-torus →
  `CoincidentRegion::FullSurfaceSame`; sphere∩sphere `d=R₁+R₂` → `TangentPoint` with
  the emitted point on both spheres ≤ tol; cylinder-tangent-plane → `TangentCurve`
  with the ruling line on both surfaces; a grazing pair whose local jet is transversal
  → `NearTangentTransversal`); the seeded patch-coincidence detector delimiting an
  `OverlapSubRegion` on two coincident freeform patches; and an ambiguous jet returning
  **`Undecided`** (no fabricated verdict). Full CTest green NUMSCI ON and OFF (the
  seeded assertions correctly absent with NUMSCI off, like S2/S3). No OCCT linked; no
  tolerance weakened. **Sim native-vs-OCCT** — a new
  `tests/sim/native_ssi_s4_classification_parity.mm` +
  `scripts/run-sim-native-ssi-s4.sh` (modelled on
  `run-sim-native-ssi-seeding-parity.sh`): assert the native coincident/tangent
  CLASSIFICATION agrees with OCCT — the analytic pairs vs `IntAna_QuadQuadGeo` /
  `IntAna_ResultType` (`Same` / `Empty` / `Point` / `Line` / `Circle`), the seeded
  pairs vs `IntPatch` / `GeomAPI_IntSS`; report per-pair classification agreement and
  the count still deferred to S4-c → OCCT. `xcrun simctl list devices booted`.
- **Roadmap**: advances `SSI-ROADMAP.md` S4 from a blunt counter to two typed
  classification layers (coincident-region + tangent-contact type), the honest first
  hardening of the moat. **S4-c (marching through a tangency / fabricating a curve
  across a degeneracy) remains the tail** — this change only DETECTS and CLASSIFIES;
  it hands `NearTangentTransversal` on unchanged.
- **Risk (honest)**: (a) the seeded patch-coincidence detector can under- or
  over-delimit an overlap boundary on a high-curvature patch — mitigated by returning
  `Undecided → OCCT` unless the region closes robustly, and by the sim parity gate
  measuring the boundary vs OCCT; (b) the differential-geometry tangent classifier can
  misread `TangentPoint` vs `NearTangentTransversal` when the relative second
  fundamental form is near-degenerate — mitigated by the `Undecided` band (report,
  don't guess) and the engine self-verify; (c) generalising analytic `Coincident`
  must not regress the shipped same-sphere / coaxial-cyl detection — pinned by keeping
  those exact cases green and adding the new families additively. Whatever does not
  classify robustly returns undecided → OCCT and is reported with the measured gap; no
  case is faked, hand-tuned, or weakened to pass.
