## Why

`SSI-ROADMAP.md` S5 is the payoff — general curved booleans driven by the S3/S4/S1 seam trace.
The native assembler now covers three surface families: through-drill **cyl∩cyl** (S5-a/b), the
**sphere∩sphere** lens (S5-c), and the branched **Steinmetz** bicylinder (S5-d) — all three ops
each, for a native-pass of **12** in the sim parity harness. The next surface family on the
roadmap is the **CONE**.

`recogniseCurvedSolid` ALREADY folds a `CurvedKind::Cone` face (frame, reference radius,
`semiAngle`) into a `CurvedSolid` and builds a cone `SurfaceAdapter` (`makeConeAdapter`), and the
S1 analytic layer (`src/native/ssi/quadric_pairs.h`) ALREADY closed-forms the seam of a **coaxial
cone∩cylinder** (`intersectCylinderConeCoaxial`) and a **coaxial cone∩sphere**
(`intersectSphereConeCoaxial`) as ANALYTIC CIRCLES — the cleanest possible SSI trace (a single
full-turn ring, `nearTangentGaps == 0`, no branch points). So recognition AND tracing are already
in place for cone pairs. The GAP is again the **assembler**: `buildCommon`/`buildFuse`/`buildCut`
(through-drill cyl), `buildLensCommon`/`Fuse`/`Cut` (sphere), and `buildSteinmetz*` (branched)
NONE handle a cone face — a coaxial cone∩cylinder seam is a single closed circle on both walls,
which `buildCommon` declines (it needs two rim seams) and `buildLensCommon` declines (it needs two
Sphere operands), so today the pair falls through to OCCT `BRepAlgoAPI_Common`.

The **analytically-cleanest** cone boolean — the target of this first cone slice — is a
**coaxial cone(frustum)∩cylinder COMMON**: the two coaxial walls cross at ONE circle (where the
cone's cross-section radius equals the cylinder radius), and the enclosed COMMON is a solid of
revolution whose radius profile is the pointwise MINIMUM of the two walls — a **frustum band**
welded to a **cylinder-segment band** along the seam circle, capped by two discs. That volume has
a **CLOSED FORM** (`V_frustum + V_cyl-segment`), giving a **DUAL oracle**: OCCT
`BRepAlgoAPI_Common` AND the exact analytic value. The seam circle does NOT pass through the cone
apex, so this slice avoids the S4-e apex chart singularity entirely (the frustum is gated
apex-free within its extent).

Harder cone pairs — TRANSVERSAL (non-coaxial) cone∩cylinder (a genuine quartic space curve),
APEX-CROSSING seams (S4-e territory), and cone∩cone — are NOT in scope and DECLINE honestly →
NULL → OCCT, exactly as they do today.

## What Changes

The geometry (a frustum cone A and a coaxial cylinder B sharing one axis, overlapping so their
walls cross at a single circle):

- Let the cone's cross-section radius along the axis be `r_c(h) = R0 + (h − h0)·tanα` (apex-free:
  `r_c(h) > 0` over the whole frustum extent), the cylinder radius `Rc`, and the two solids'
  axial overlap `[hBot, hTop] = [max(coneBottom, cylBottom), min(coneTop, cylTop)]`. The walls
  cross at the seam height `h*` where `r_c(h*) = Rc` (the S1 analytic circle), `h*` strictly
  inside the overlap. On one side of `h*` the cone is the tighter (smaller-radius) wall; on the
  other the cylinder is.
- **COMMON** `A ∩ B` (new, `buildConeCylCommon`) = the min-radius-profile solid of revolution over
  `[hBot, hTop]`: the **CONE band** where `r_c ≤ Rc` (kept because it classifies INSIDE the
  cylinder) welded along the seam circle to the **CYLINDER band** where `Rc ≤ r_c` (kept because
  it classifies INSIDE the cone), closed by a bottom disc cap at `hBot` and a top disc cap at
  `hTop` (each the terminal disc of whichever band ends there). Volume closed form:
  `V(A ∩ B) = V_frustum(r(hBot) → Rc) + π Rc²·(hTop − h*)`, where
  `V_frustum(ra → rb over Δh) = (π Δh / 3)(ra² + ra·rb + rb²)`.

Concretely:

- **Add `buildConeCylCommon(A, B, seam)`** in `src/native/boolean/ssi_boolean.cpp` — the coaxial
  cone∩cylinder COMMON assembler. It (1) GATES: exactly one operand `Cone` + one coaxial
  `Cylinder`, a SINGLE closed full-circle seam on BOTH walls, the frustum apex-free within its
  extent (`r_c(v) > margin` for all `v` — else NULL → OCCT, S4-e apex territory), and the seam
  `h*` strictly interior to BOTH extents (a transversal circle, not a cap-edge tangent); (2)
  resamples the traced seam circle into ONE pooled full-turn ring (`nu` samples) shared by both
  bands; (3) emits the CONE band from the cone-side terminal to the seam and the CYLINDER band
  from the seam to the cylinder-side terminal, each as a planar-facet ring strip drawn through a
  shared `VertexPool` so the two bands weld along the seam ring; each band is KEPT only if its
  interior sample classifies strictly INSIDE the other solid (`classifyPoint == 1`); (4) closes
  with `appendDiskCap` at `hBot` (outward −axis) and `hTop` (outward +axis), the terminal rims
  pooled with the bands; (5) `makeShell → makeSolid`, or NULL on any decline.
- **Add `appendConeBand`** — the cone analogue of the cylinder-band strip: a ring-by-ring planar-
  facet frustum band between two axial stations, outward radial normal (the cone's outward wall
  normal, folded by `cosα`), sharing its terminal rim rings through the `VertexPool`. Mirrors the
  existing `appendTubeBand`/`appendDiskCap` planar-facet discipline. The cylinder band reuses the
  existing cylinder strip helper.
- **Dispatch** (`ssi_boolean_solid`, `Op::Common`): after `buildCommon` (through-drill, declines a
  single seam) and before/after `buildLensCommon` (declines a non-sphere operand), try
  `buildConeCylCommon`. Recognition, tracing, and the other builders are UNCHANGED — only the
  COMMON dispatch grows one arm. `Op::Fuse` / `Op::Cut` for cone pairs are NOT wired → NULL → OCCT
  (deferred; the analytically-clean COMMON is the guaranteed first slice).
- **Engine self-verify — a NEW analytic oracle, mirroring the Steinmetz `16 R³/3` oracle.** Extend
  `ssiCurvedBooleanVerified` (`src/engine/native/native_engine.cpp`, the `op == 2` COMMON branch)
  to recognise the coaxial cone∩cylinder configuration and return the closed-form expected volume
  `V_frustum(r(hBot) → Rc) + π Rc²·(hTop − h*)`; the guard accepts the native result iff it is
  watertight AND its mesh volume matches that closed form within the deflection-bounded tolerance
  — a mis-selected band, a mis-placed cap, or a hairline seam gap yields the wrong volume, FAILS
  the guard, and is DISCARDED → OCCT. The engine (not the library) owns the analytic oracle, so
  `src/native/**` stays OCCT-free.
- **(Optional secondary) `buildConeSphereCommon`** — the coaxial cone∩sphere COMMON, gated to the
  single-crossing config (the S1 `intersectSphereConeCoaxial` returning exactly ONE valid circle
  within both extents; the two-circle case DECLINES → OCCT). Same assembler shape (frustum band +
  spherical-segment band welded on the seam circle), closed form `V_frustum + V_spherical-segment`
  as the analytic oracle. Included only if it lands watertight in the same verified envelope.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-booleans SSI curved-boolean op-set to the CONE
surface family (the coaxial cone∩cylinder COMMON, optionally cone∩sphere COMMON), reusing the S1
analytic circle seam, the shared-pool planar-facet weld, and the engine self-verify already
established for the cyl / sphere / Steinmetz families. -->

### Modified Capabilities
- `native-booleans`: add native **coaxial cone∩cylinder COMMON** (and optionally coaxial
  cone∩sphere COMMON) — the min-radius-profile solid of revolution (a frustum band welded to a
  cylinder-segment / spherical-segment band along the single S1 analytic seam circle, closed by
  two disc caps), selected by the inside-the-other rule (`classifyPoint`), welded watertight
  through one `VertexPool`, and gated by a NEW closed-form analytic oracle in the engine
  self-verify (`V_frustum + V_cyl-segment`) → OCCT fallback. The cone builder DECLINES (NULL →
  OCCT) outside its verified envelope: an apex-crossing / apex-in-extent frustum, a non-coaxial
  (transversal) pair, a cap-edge-tangent seam, a two-circle cone∩sphere crossing, cone∩cone, and
  cone FUSE/CUT all remain the OCCT boundary. The cyl / sphere / Steinmetz builders and all their
  ops stay byte-identical. No `cc_*` change; compiled under `CYBERCAD_HAS_NUMSCI`;
  `src/native/**` stays OCCT-free.
- `native-ssi`: re-affirm the S1 analytic coaxial cone∩cylinder / cone∩sphere circle seam
  (`intersectCylinderConeCoaxial` / `intersectSphereConeCoaxial`) as the consumed input contract
  for the new S5 cone COMMON assembler — a SINGLE closed full-turn circle at the wall-crossing
  height, `nearTangentGaps == 0`, `branchPoints == 0`, full-circle on both walls, NOT passing
  through the cone apex — and the honest decline boundary: a transversal (non-coaxial) cone pair
  (a quartic space curve, not analytic here), a two-circle coaxial crossing, or an apex-crossing
  seam is NOT consumed → OCCT.

## Impact

- **Scope (one analytically-clean surface pair).** This change adds ONLY the coaxial cone∩cylinder
  COMMON (optionally cone∩sphere COMMON) — the seam is a single analytic circle and the volume is
  closed-form. It does NOT touch the existing cyl / sphere / Steinmetz builders or their ops, and
  it does NOT attempt transversal cone∩cylinder, apex-crossing seams, cone∩cone, or cone
  FUSE/CUT — those keep their existing decline → OCCT behaviour.
- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry point,
  signature, or POD struct changes. Additive only.
- **Build**: extends `src/native/boolean/ssi_boolean.cpp` (add `buildConeCylCommon`,
  `appendConeBand`; extend the `Op::Common` dispatch) and `src/engine/native/native_engine.cpp`
  (extend `ssiCurvedBooleanVerified` with the coaxial-cone closed-form oracle). Reuses
  `recogniseCurvedSolid`, `classifyPoint`, `appendDiskCap`, the cylinder strip helper,
  `pushPlanarTri`, `VertexPool`, `toSeam`, and the S1 `intersectCylinderConeCoaxial` seam — all
  already present. No new files. Compiled under `CYBERCAD_HAS_NUMSCI`. `src/native/**` stays
  OCCT-free. No change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the
  analytic `curved.h`, or the cyl / sphere / Steinmetz builders.
- **Verification (two gates, dual oracle, no weakened tolerance).**
  - **Host (no OCCT)** — the closed-form min-profile volume:
    `V(A ∩ B) = V_frustum(r(hBot) → Rc) + π Rc²·(hTop − h*)` with
    `V_frustum(ra → rb over Δh) = (π Δh/3)(ra² + ra·rb + rb²)`. The host suite asserts the
    assembled shell is watertight (`boundaryEdgeCount == 0`, every edge shared by exactly two
    faces), the enclosed volume matches the closed form within the deflection-sized band, every
    seam-ring node lies on BOTH walls ≤ tol, the seam ring is pooled ONCE (both bands share it),
    and an apex-in-extent / transversal / cap-tangent / two-circle fixture returns NULL
    (deferred). Green with NUMSCI on AND off (the cone path correctly absent off).
  - **Sim native-vs-OCCT** — extend `scripts/run-sim-native-ssi-curved-boolean.sh` +
    `tests/sim/native_ssi_curved_boolean_parity.mm` with a coaxial cone∩cylinder pair (the
    harness already has `makeCone`/`occtCone` and `makeCyl`/`occtCyl`); its COMMON becomes a
    NATIVE pass vs `BRepAlgoAPI_Common` (volume + surface area + watertight closed shell + valid
    shape), raising **native-pass 12 → 13** (14 if cone∩sphere COMMON also lands). FUSE/CUT for
    the cone pair stay honest fall-backs with the measured gap reported. Do NOT regress the 12
    existing native passes.
- **Roadmap**: opens the CONE surface family in `SSI-ROADMAP.md` S5 (coaxial cone∩cylinder COMMON
  native; transversal / apex / cone∩cone the tail). Records the measured deltas in
  `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md`.
- **Risk (honest)**: a band misclassification, a mis-placed cap, or a hairline seam-ring gap
  yields a self-intersecting or wrong-volume shell — caught by the engine's mandatory watertight +
  closed-form-volume self-verify, which DISCARDS the candidate → OCCT. An apex-crossing or
  transversal seam is refused up front (gate) → OCCT. If the cone COMMON cannot be built
  watertight with the correct closed-form volume, the builder returns NULL → OCCT and the measured
  gap is reported. No case is faked, stubbed, or tolerance-weakened. The cyl / sphere / Steinmetz
  paths stay byte-identical (native-pass 12 not regressed).
