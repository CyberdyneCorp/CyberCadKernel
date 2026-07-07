# Tasks — add-native-cone-sphere-boolean (SSI Stage S5-f: coaxial cone∩sphere COMMON/FUSE/CUT)

Verification levels: **host** = OCCT-free host CTest — analytic inclusion-exclusion on the exact
cone∩sphere common (`V(A ∩ B) = V_frustum(r_c(sLo) → r_c(s*)) + V_spherical-segment(s* → pole)`,
`V_frustum(ra → rb over Δh) = (π Δh/3)(ra² + ra·rb + rb²)`, `V_seg(s1 → s2) = π[Rs²(s2 − s1) −
((s2 − s_c)³ − (s1 − s_c)³)/3]`) and the operand volumes (`V(cone frustum)`, `V(sphere) =
4/3·π Rs³`): `FUSE = V(A) + V(B) − V(A ∩ B)`, `CUT(A,B) = V(A) − V(A ∩ B)`; watertight
(`boundaryEdgeCount == 0`, every edge shared by exactly two faces), correct set-algebra volume,
every seam-ring node on BOTH walls ≤ tol, the seam ring pooled ONCE; two-circle / apex-in-extent /
transversal / sphere-minuend fixtures → NULL (deferred, no native solid). **sim** = native-vs-OCCT
`BRepAlgoAPI_{Common,Fuse,Cut}` parity on a booted iOS simulator (volume, surface area, watertight
closed shell, valid shape) via `tests/sim/native_ssi_curved_boolean_parity.mm` /
`scripts/run-sim-native-ssi-curved-boolean.sh` — native-pass 15 → 18. Invoked behind the existing
`cc_boolean` op codes — **no `cc_*` entry point is added or changed**; asserted at the
`cybercad::native::boolean` C++ boundary. Compiled under **`CYBERCAD_HAS_NUMSCI`**. `src/native/**`
stays OCCT-free.

## 1. Shared gate/seam prologue `coneSphereSetup`  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 Add `coneSphereSetup(A, B, seams) -> ConeSphereSetup` mirroring `coneCylSetup`: the gate
  (one `Cone` + one `Sphere`, the sphere centre ON the cone axis via `distancePointLine`/`sameAxis`,
  a single closed full-circle seam, non-degenerate `tanα`, apex-free frustum), the
  `intersectSphereConeCoaxial` quadratic re-solved in the cone's `s`-coordinate requiring EXACTLY
  ONE root `s*` strictly interior to both extents (declines a TWO-in-extent-root crossing and an
  apex crossing), the analytic-vs-traced seam cross-check (centroid height `s*`, mean radius
  `r_c(s*)`), the azimuth resolution `N` (seam-chord sagitta ≤ `kCapSagitta`), the shared frame
  `(O, ẑ, X, Y)`, the `rCone(s)` / `rSph(s)` / `ring(r, s)` / `wallPoint(r, s)` functors, the two
  sphere poles (`s_c ± Rs`) CLASSIFIED against the cone into `innerPole` (inside) / `outerPole`
  (outside), and ONE canonical pooled seam ring (`ring(r_c(s*), s*)`) returned as a `Seam` so BOTH
  the cone band and `appendSphereCap` weld on the identical nodes. Returns `ok == false` on any
  decline. (**host**)

## 2. `buildConeSphereCommon` — cone band + seam + sphere inner cap + cone disc  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 Assemble the min-cross-section overlap: the cone wall band on the cone-tighter side
  (`appendRevolvedBand`, outward, kept iff its mid-sample `classifyPoint(sphere, mid) == 1`), the
  cone terminal disc inside the sphere (`appendDiskCap`, `±ẑ`), and the sphere INNER cap
  (`appendSphereCap`, inner apex = `innerPole`, `reversed = false`, kept iff
  `classifyPoint(cone, innerPole) == 1`), all sharing the pooled seam ring. `makeShell → makeSolid`;
  too few faces → `{}`. Measured volume = `V_frustum + V_spherical-segment` within the deflection
  band (reference fixture ≈ `5.256`), watertight. (**host** ✓)

## 3. `buildConeSphereFuse` — sphere outer cap + seam + cone outer band + disc  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Assemble the max-cross-section union: the sphere OUTER cap (`appendSphereCap`, outer apex
  = `outerPole`, `reversed = false`, kept iff `classifyPoint(cone, outerPole) == -1`), the cone
  OUTER wall band on the cone-outside-sphere side (`appendRevolvedBand`, outward, kept iff its
  mid-sample `classifyPoint(sphere, mid) == -1`), and the cone terminal disc bounding the union
  (`appendDiskCap`), all sharing the pooled seam ring. `makeShell → makeSolid`; too few faces →
  `{}`. Measured volume = `V(A) + V(B) − V(A ∩ B)` within the deflection band (reference fixture ≈
  `60.718`), watertight, a GROW. (**host** ✓)

## 4. `buildConeSphereCut` — cone outer band + disc + reversed sphere inner cap  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 `A` is the CONE minuend (order-sensitive, matches `BRepAlgoAPI_Cut(a, b)`; `&A != s.cone`
  → `{}` so `sphere − cone` declines → OCCT). Assemble A's OUTER wall band (outside sphere,
  `appendRevolvedBand`, outward, kept iff `classifyPoint(sphere, mid) == -1`), A's terminal disc
  cap(s) outside B (`appendDiskCap`), and the sphere INNER cap emitted REVERSED (`appendSphereCap`,
  inner apex = `innerPole`, `reversed = true` — inward normal, the dimple bounding the cavity,
  pinching to the pooled seam ring, kept iff `classifyPoint(A, innerPole) == 1`). `makeShell →
  makeSolid` (ONE connected component). Measured volume = `V(A) − V(A ∩ B)` within the deflection
  band (reference fixture ≈ `27.207`), watertight, a SHRINK. (**host** ✓)

## 5. Driver dispatch  [CYBERCAD_HAS_NUMSCI]
- [x] 5.1 In `ssi_boolean_solid`: `Op::Common` arm → after `buildCommon` / `buildLensCommon` /
  `buildConeCylCommon` decline, `return buildConeSphereCommon(*csA, *csB, seams)`; `Op::Fuse` arm →
  after `buildConeCylFuse` decline, `return buildConeSphereFuse(*csA, *csB, seams)`; `Op::Cut` arm →
  after `buildConeCylCut` decline, `return buildConeSphereCut(*csA, *csB, seams)`. Recognition,
  trace, the transversality gate, and every other builder UNCHANGED; a non-(cone + coaxial-sphere)
  pair keeps its existing path (no regression). (**host**)

## 6. Engine self-verify — new COMMON arm + per-op sign (fuse grows, cut shrinks)
- [x] 6.1 Add a coaxial cone∩sphere COMMON arm to `ssiCurvedBooleanVerified` (`native_engine.cpp`),
  alongside the Steinmetz and cone∩cylinder arms: for a recognised coaxial cone∩sphere pair (sphere
  centre on the cone axis, single interior crossing `s*`), `expected = V_frustum(cone-tighter
  sub-band) + V_spherical-segment(sphere-tighter sub-band)`; `vr = watertightVolume(result)`; accept
  iff watertight AND `|vr − expected| ≤ max(1e-2·expected, 1e-6)`. Returns `{}` (not applicable) for
  `op != 2` and for non-cone∩sphere pairs → no interception of other families. (**host**)
- [x] 6.2 CONFIRM the generic `booleanResultVerified` computes `expected = va + vb − vc` (fuse) /
  `va − vc` (cut) with `vc =` native `buildConeSphereCommon` (`= V(A ∩ B)`), so FUSE grows
  (`Vr > max(VA, VB)`) and CUT shrinks (`Vr < VA`) against the native cone∩sphere common. A
  mis-selected band, a mis-oriented reversed cap, or a hairline seam-ring gap yields the wrong
  volume, FAILS the guard, and is DISCARDED → OCCT — the engine never emits an unverified cone∩
  sphere boolean. (**host** ✓ wrong-volume candidate discarded)

## 7. Honest scope — deferrals (never faked)
- [x] 7.1 TWO-circle coaxial cone∩sphere crossings (the sphere passes fully through the cone / spans
  the apex), apex-crossing / apex-in-extent frustums, TRANSVERSAL (non-coaxial) cone∩sphere (a
  quartic space curve — `intersectSphereConeCoaxial` returns `notAnalytic`), a `sphere − cone` CUT
  (sphere minuend), and cone∩cone (any op) → NULL → OCCT, documented in the `ssi_boolean.cpp`
  cone∩sphere header block. The cyl / sphere / Steinmetz / cone∩cylinder builders, the through-drill
  / lens / cone∩cylinder paths, and the generic self-verify UNCHANGED. (**host** ✓ docs + NULL
  fixtures)

## 8. Verification (two gates, dual oracle, no weakened tolerance)
- [x] 8.1 Host suite (extend `test_native_ssi_curved_boolean` or the S5 test): the reference coaxial
  cone∩sphere pair COMMON + FUSE + CUT → watertight, volume matches the analytic inclusion-exclusion
  closed form within the deflection band, seam-ring nodes on both walls ≤ tol, seam ring pooled
  once; two-circle / apex-in-extent / transversal / sphere-minuend → NULL. Full CTest green NUMSCI
  on AND off (cone∩sphere tests absent with NUMSCI off). Existing goldens (cyl / sphere / Steinmetz
  / cone∩cylinder) unchanged. (**host**)
- [x] 8.2 Sim: `scripts/run-sim-native-ssi-curved-boolean.sh` on a booted simulator
  (`xcrun simctl list devices booted`) — add the `cone=sphere(coax)` pair; its COMMON + FUSE + CUT
  become native passes vs `BRepAlgoAPI_{Common,Fuse,Cut}` (volume ≈ `5.256` common / `60.718` fuse /
  `27.207` cut, surface area, watertight closed shell, valid shape); native-pass **15 → 18**. Do NOT
  regress the 15 existing native passes (through-drill cyl∩cyl + sphere∩sphere + Steinmetz +
  cone∩cylinder COMMON/FUSE/CUT). Any pair whose self-verify does not pass stays an honest fall-back
  with the measured gap reported — no tolerance weakened. (**sim**)
- [x] 8.3 `openspec validate add-native-cone-sphere-boolean --strict` green; note the coaxial
  cone∩sphere op-set now 3/3 native in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with
  the measured deltas. Confirm no SSI / blend / heal / import / marching / phase3 suite regresses.
  (**host** + **sim**)

## Out of scope (NOT in this change — honest)
- [x] TWO-circle coaxial cone∩sphere crossings, apex-crossing seams, frustum extents including the
  apex, TRANSVERSAL (non-coaxial) cone∩sphere (a quartic space curve), a `sphere − cone` CUT
  (sphere minuend) → OCCT.
- [x] cone∩cone (any op) → UNCHANGED, NULL → OCCT.
- [x] Any other curved-curved family (through-drill cyl∩cyl, sphere∩sphere, Steinmetz, coaxial
  cone∩cylinder) → UNCHANGED, existing native/decline behaviour.
- [x] Freeform (NURBS / Bézier) operand faces → OCCT.
- [x] Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the cyl / sphere / Steinmetz / cone∩cylinder builders, or the generic set-algebra
  self-verify.
