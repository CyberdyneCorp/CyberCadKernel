## Why

`SSI-ROADMAP.md` S5 is the payoff — general curved booleans driven by the S3/S4/S1 seam trace.
The CONE surface family opened with the analytically-cleanest cone boolean: the archived
`add-native-cone-boolean` (S5-e COMMON) + `complete-cone-cyl-fuse-cut` (FUSE + CUT) made the
**coaxial cone(frustum)∩cylinder** op-set 3/3 native (sim `native-pass = 15`). All three share the
SAME S1 analytic circle seam (`intersectCylinderConeCoaxial`, `nearTangentGaps == 0`,
`branchPoints == 0`): the shared `coneCylSetup` prologue folds the pair, cross-checks the analytic
crossing `s*` against the traced seam, and the split machinery (`appendRevolvedBand` +
`appendDiskCap` + `appendAnnulusCap` + one `VertexPool`) welds the surviving bands. COMMON =
min-radius inside bands, FUSE = max-radius outer bands, CUT = A outer wall + reversed B inside band
— each verified against a DUAL oracle (the analytic closed form AND OCCT `BRepAlgoAPI`).

The NEXT cone-involving pair after coaxial cone∩cylinder is the **coaxial cone∩sphere** family, and
it is ALSO analytically clean. A coaxial cone and sphere (sphere centre ON the cone axis) meet along
ONE analytic CIRCLE seam (SSI S1 `intersectSphereConeCoaxial` — a QUADRATIC in the cone parameter,
so up to two circles; the single-crossing configuration where the sphere sits on the FRUSTUM side
gives exactly ONE circle that does NOT cross the cone apex). That is the analytically-clean case:
the seam is one closed circle, the overlap is a solid of revolution, and the COMMON volume has a
closed form (a cone-frustum band welded to a spherical segment/cap split at the seam circle). Today
the dispatcher's `Op::Common` arm runs `buildCommon` (through-drill, declines a single seam) →
`buildLensCommon` (declines a non-sphere operand) → `buildConeCylCommon` (declines a sphere operand)
and returns NULL; `Op::Fuse` / `Op::Cut` similarly fall through. So the coaxial cone∩sphere family
is **0/3 native** — COMMON, FUSE, and CUT all fall back to OCCT `BRepAlgoAPI_{Common,Fuse,Cut}`
(honest, verified fall-backs — but fall-backs). The archived `add-native-cone-boolean` even drafted
an OPTIONAL `buildConeSphereCommon` requirement that did not land.

The GAP is again the **assembler**, not recognition or tracing: `recogniseCurvedSolid` +
`intersectSphereConeCoaxial` already fold the pair and closed-form the seam circle. And the two
sides of the seam are each already covered by an existing builder family: the CONE side is the
cone-wall split from `buildConeCylCommon` (`appendRevolvedBand` + `appendDiskCap`), and the SPHERE
side is the spherical-cap fragment from the sphere-lens builders (`appendSphereCap`, with its
inner/outer apex and reversed-normal flags). This change assembles the two: the coaxial cone∩sphere
family becomes **3/3 native** (COMMON + FUSE + CUT), mirroring the archived
`complete-steinmetz-fuse-cut`, `complete-sphere-sphere-fuse-cut`, and `complete-cone-cyl-fuse-cut`.

## What Changes

The geometry (a coaxial frustum cone A and a sphere B whose centre lies ON the cone axis,
positioned on the FRUSTUM side so their walls cross at a SINGLE circle at `s*`; measure axial
station `s` along the cone axis from the cone origin; `r_c(s) = R0 + s·tanα` is the cone
cross-section radius, `r_s(s) = √(Rs² − (s − s_c)²)` the sphere cross-section radius about the
on-axis sphere centre at station `s_c`; the seam at `s*` where `r_c(s*) = r_s(s*)` splits each wall
into an INSIDE-the-other band and an OUTSIDE band; the cone carries its two disc end caps, the
sphere has no flat cap but its wall closes at its two on-axis poles at `s_c ± Rs`):

- **COMMON** `A ∩ B` (new, `buildConeSphereCommon`) = the min-cross-section solid of revolution over
  the overlap: the CONE wall band on the `r_c ≤ r_s` side (inside the sphere) welded along the seam
  circle to the SPHERICAL SEGMENT on the `r_s ≤ r_c` side (the sphere cap inside the cone, closing
  to the sphere pole that lies inside the cone), plus the cone end-cap disc that bounds the overlap
  (the cone terminal disc inside the sphere). Volume = `V_frustum(cone-tighter sub-band) +
  V_spherical-segment(sphere-tighter sub-band)` — a closed form.
- **FUSE** `A ∪ B` (new, `buildConeSphereFuse`) = the max-cross-section outer profile: the SPHERE
  OUTER cap (the part of the sphere outside the cone, closing to the FAR pole) welded at the SAME
  seam circle to the CONE OUTER wall band (the part of the cone outside the sphere), closed by the
  cone terminal disc(s) that bound the union. Volume = `V(A) + V(B) − V(A ∩ B)` (a GROW).
- **CUT** `A − B` (new, `buildConeSphereCut`, `A` the CONE minuend) = A's OUTER wall (outside B) +
  A's terminal disc cap(s) outside B, joined to B's INSIDE cap emitted REVERSED (inward radial
  normal, the spherical dimple bounding the carved cavity, pinching to the seam circle). Volume =
  `V(A) − V(A ∩ B)` (a SHRINK). Unlike the cone∩cylinder CUT, the cone∩sphere single-crossing CUT is
  CONNECTED (one closed component — a frustum with a spherical dimple).

Concretely, reusing the S5-e seam/split/weld AND the sphere-lens cap builder
(`recogniseCurvedSolid`, `sameAxis`, `intersectSphereConeCoaxial`, the shared `VertexPool`, the
pooled seam ring, `appendRevolvedBand`, `appendDiskCap`, `appendSphereCap`, `classifyPoint`):

- **Add the shared prologue `coneSphereSetup(A, B, seams)`** (mirroring `coneCylSetup`) — the gate
  (one `Cone` + one `Sphere`, the sphere centre ON the cone axis, a single closed full-circle seam,
  apex-free frustum), the single interior crossing `s*` from the `intersectSphereConeCoaxial`
  quadratic (declines when TWO roots fall inside both extents, or the seam crosses the apex, or `s*`
  is not strictly interior), the analytic-vs-traced seam cross-check (centroid height `s*`, mean
  radius `r_c(s*)`), the azimuth resolution `N`, the shared frame `(O, ẑ, X, Y)`, the `rCone(s)` and
  `ring(r, s)` functors, the two sphere poles CLASSIFIED against the cone (inner = the pole inside
  the cone, outer = the pole outside the cone), and ONE canonical pooled seam ring at `(r_c(s*), s*)`
  emitted as a `Seam` so BOTH the cone band and the spherical cap weld on the IDENTICAL seam nodes.
- **Add `buildConeSphereCommon`** — cone wall band (inside sphere, `appendRevolvedBand` outward) +
  the pooled seam ring + the sphere INNER cap (inside cone, `appendSphereCap`, inner apex, outward)
  + the cone terminal disc inside the sphere (`appendDiskCap`). Survival: the cone band mid-sample
  INSIDE the sphere AND the sphere cap apex INSIDE the cone (`classifyPoint == 1`). Volume =
  `V_frustum + V_spherical-segment`.
- **Add `buildConeSphereFuse`** — the sphere OUTER cap (outside cone, `appendSphereCap`, outer apex,
  outward) + the pooled seam ring + the cone OUTER wall band (outside sphere, `appendRevolvedBand`
  outward) + the cone terminal disc bounding the union. Survival: the far pole OUTSIDE the cone AND
  the cone outer band mid-sample OUTSIDE the sphere (`classifyPoint == -1`). Volume =
  `V(A) + V(B) − V(A ∩ B)`.
- **Add `buildConeSphereCut`** — `A` is the CONE minuend (order-sensitive, matches
  `BRepAlgoAPI_Cut(a, b)`). A's OUTER wall band (outside sphere, outward) + A's terminal disc
  cap(s) outside B + the sphere INNER cap emitted REVERSED (`appendSphereCap`, inner apex,
  reversed = inward — the spherical dimple bounding the cavity, pinching to the seam ring). Survival:
  A's outer band OUTSIDE B AND the reversed sphere cap's near-pole INSIDE A. A sphere-minuend
  (`sphere − cone`) declines → OCCT. Volume = `V(A) − V(A ∩ B)`.
- **Driver dispatch** (`ssi_boolean_solid`): `Op::Common` → `buildConeSphereCommon` (after
  `buildCommon` / `buildLensCommon` / `buildConeCylCommon` decline), `Op::Fuse` →
  `buildConeSphereFuse` (after `buildFuse` / `buildLensFuse` / `buildConeCylFuse` decline),
  `Op::Cut` → `buildConeSphereCut` (after `buildCut` / `buildLensCut` / `buildConeCylCut` decline).
  Recognition, trace, the transversality gate, and every other builder are UNCHANGED — each op arm
  grows one final call. A non-(cone + coaxial-sphere) pair keeps its existing path.
- **Engine self-verify — ONE new analytic oracle arm, correct per-op sign.** Add a coaxial
  cone∩sphere COMMON arm to `ssiCurvedBooleanVerified` (`native_engine.cpp`), mirroring the existing
  Steinmetz (`16 r³/3`) and cone∩cylinder (`V_frustum + V_frustum`) arms: for a recognised coaxial
  cone∩sphere pair with a single interior crossing, `expected = V_frustum(cone-tighter sub-band) +
  V_spherical-segment(sphere-tighter sub-band)` — the CLOSED FORM. FUSE / CUT need NO new arm: the
  `op == 2`-only analytic oracle does not intercept them, so the generic `booleanResultVerified`
  runs `vc = watertightVolume(boolean_solid(a, b, Op::Common))` = the native `buildConeSphereCommon`
  = `V(A ∩ B)`, and `expected = va + vb − vc` (fuse, GROWS) / `va − vc` (cut, SHRINKS). A
  mis-selected band, a mis-oriented reversed cap, or a hairline seam gap yields the wrong volume,
  FAILS the guard, and is DISCARDED → OCCT, never faked.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-booleans + native-ssi capabilities with the next
cone-involving pair (coaxial cone∩sphere COMMON + FUSE + CUT), reusing the S1 analytic circle seam
(`intersectSphereConeCoaxial`), the shared-pool planar-facet weld, the cone-wall split
(`appendRevolvedBand` / `appendDiskCap`) and the spherical-cap fragment (`appendSphereCap`) already
established for the cyl / sphere / Steinmetz / cone∩cylinder families, plus one new analytic
self-verify oracle arm mirroring the existing Steinmetz and cone∩cylinder COMMON arms. -->

### Modified Capabilities
- `native-booleans`: open the coaxial cone∩sphere op-set — add native **Common** (the min-
  cross-section overlap: the cone wall band inside the sphere welded along the single S1 seam circle
  to the spherical segment inside the cone + the cone terminal disc bounding the overlap), **Fuse**
  (the max-cross-section union: the sphere outer cap outside the cone welded to the cone outer wall
  outside the sphere + the cone terminal disc), and **Cut** (`A − B`, cone minuend: A's outer wall +
  A's caps outside B + the sphere inner cap emitted REVERSED, a connected frustum-with-spherical-
  dimple), all welded along the SAME single S1 analytic circle seam with the shared-pool planar-
  facet discipline (cone side `appendRevolvedBand` / `appendDiskCap`, sphere side `appendSphereCap`).
  COMMON is guarded by a NEW analytic closed-form oracle arm (`V_frustum + V_spherical-segment`) in
  `ssiCurvedBooleanVerified`; FUSE / CUT by the EXISTING generic set-algebra self-verify (fuse
  grows, cut shrinks, vs the native cone∩sphere COMMON `V(A ∩ B)`) → OCCT fallback. All three
  DECLINE (NULL → OCCT) outside the verified envelope: a TWO-circle crossing (the sphere passes
  fully through the cone), an apex-crossing / apex-in-extent frustum, a non-coaxial (transversal)
  pair (a quartic space curve), a sphere-minuend cut, cone∩cone, and every other pair. No `cc_*`
  change; compiled under `CYBERCAD_HAS_NUMSCI`; `src/native/**` stays OCCT-free.
- `native-ssi`: consume the S1 analytic coaxial cone∩sphere circle seam
  (`intersectSphereConeCoaxial`) as the input contract for ALL THREE cone∩sphere ops — the SINGLE
  closed full-turn circle at `s*` (from the QUADRATIC's single in-extent root) splits each coaxial
  wall into an inside/outside band; COMMON / FUSE / CUT differ only in WHICH bands survive, their
  orientation, and the cone caps. The tracer does not change; a TWO-circle crossing, the transversal
  (non-coaxial) cone∩sphere pair (a quartic space curve, `notAnalytic`), and the apex-crossing seam
  remain the honest decline boundary → OCCT.

## Impact

- **Scope (bounded new pair).** This change ONLY opens the coaxial cone∩sphere op-set in its
  single-crossing configuration (the sphere on the frustum side, ONE seam circle). All other pairs
  (through-drill cyl∩cyl, sphere∩sphere lens, Steinmetz bicylinder, coaxial cone∩cylinder, cone∩cone,
  transversal cone∩sphere, the two-circle cone∩sphere crossing, freeform) are UNCHANGED and keep
  their existing native/decline behaviour.
- **ABI**: none. Invoked behind the existing `cc_boolean` op codes; no `cc_*` entry point,
  signature, or POD struct changes. Additive only.
- **Build**: extends `src/native/boolean/ssi_boolean.cpp` (add `coneSphereSetup`, `buildConeSphere
  Common` / `Fuse` / `Cut`, extend the three dispatch arms) and `src/engine/native/native_engine.cpp`
  (add ONE coaxial cone∩sphere COMMON analytic arm to `ssiCurvedBooleanVerified`, mirroring the
  Steinmetz and cone∩cylinder arms). Reuses `recogniseCurvedSolid`, `sameAxis`, `classifyPoint`,
  `appendRevolvedBand`, `appendDiskCap`, `appendSphereCap`, `decimateSeam`/`seamNodeTarget`,
  `pushPlanarTri`, `VertexPool`, and the S1 `intersectSphereConeCoaxial` seam — all already present.
  No new files. Compiled under `CYBERCAD_HAS_NUMSCI`. `src/native/**` stays OCCT-free. No change to
  `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic `curved.h`, the cyl /
  sphere / Steinmetz / cone∩cylinder builders, or the generic set-algebra self-verify.
- **Verification (two gates, dual oracle, no weakened tolerance).**
  - **Host (no OCCT)** — inclusion-exclusion on the exact closed-form cone∩sphere common
    `V(A ∩ B) = V_frustum(r_c(sLo) → r_c(s*)) + V_spherical-segment(s* → pole)` and the operand
    volumes (`V(cone frustum) = (π Δh/3)(r0² + r0·r1 + r1²)`, `V(sphere) = 4/3·π Rs³`):
    `FUSE = V(A) + V(B) − V(A ∩ B)`, `CUT(A,B) = V(A) − V(A ∩ B)`. The host suite asserts the
    assembled shell is watertight (`boundaryEdgeCount == 0`, every edge shared by exactly two faces),
    the enclosed volume matches the closed form within the deflection-sized band, every seam-ring
    node lies on BOTH walls ≤ tol, and the seam ring is pooled ONCE. Two-circle / apex-in-extent /
    transversal / sphere-minuend fixtures return NULL (deferred). Green with NUMSCI on AND off (the
    cone∩sphere path correctly absent off).
  - **Sim native-vs-OCCT** — add a coaxial cone∩sphere COMMON/FUSE/CUT fixture to
    `scripts/run-sim-native-ssi-curved-boolean.sh` / `tests/sim/native_ssi_curved_boolean_parity.mm`
    and compare native vs OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (volume + surface area + watertight
    closed shell + valid shape). Reference fixture: cone `r_c(s) = 0.5 + 0.5s` over `[0,4]`, sphere
    centre on-axis at the cone origin, `Rs = 2` → single crossing `s* ≈ 1.5436`, seam radius
    `≈ 1.2718`, `V(A) ≈ 32.463`, `V(B) ≈ 33.510`, `V(A ∩ B) ≈ 5.256`, `FUSE ≈ 60.718`,
    `CUT(cone − sphere) ≈ 27.207`. The three ops become NATIVE passes, raising **native-pass 15 →
    18**. Do NOT regress the 15 existing native passes. Any pair whose self-verify does not pass stays
    an honest fall-back with the measured gap reported.
- **Roadmap**: opens `SSI-ROADMAP.md` S5 coaxial cone∩sphere to 3/3 native (COMMON + FUSE + CUT).
  Records the measured deltas in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md`.
- **Risk (honest)**: a COMMON band/cap misclassification, a FUSE outer-cap error, a CUT reversed-cap
  orientation error, or a hairline seam-ring gap yields a self-intersecting or wrong-signed shell —
  caught by the mandatory watertight + correct-volume self-verify (COMMON vs the NEW closed-form
  oracle; fuse grows / cut shrinks vs the native cone∩sphere COMMON), which DISCARDS the candidate →
  OCCT. A two-circle crossing, an apex-crossing / apex-in-extent frustum, a transversal seam, or a
  sphere-minuend cut is refused up front (gate) → OCCT. If any op cannot be built watertight with the
  correct volume, the builder returns NULL → OCCT and the measured gap is reported. No case is faked,
  stubbed, or tolerance-weakened. The cyl / sphere / Steinmetz / cone∩cylinder paths stay byte-
  identical (native-pass 15 not regressed).
