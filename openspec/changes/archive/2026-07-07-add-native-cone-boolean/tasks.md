# Tasks — add-native-cone-boolean (SSI Stage S5 — CONE surface family, coaxial COMMON)

Verification levels: **host** = OCCT-free host CTest — the closed-form min-radius-profile volume
`V(A ∩ B) = V_frustum(r(hBot) → Rc) + π Rc²·(hTop − h*)` with
`V_frustum(ra → rb over Δh) = (π Δh/3)(ra² + ra·rb + rb²)`; watertight (`boundaryEdgeCount == 0`,
every edge shared by exactly two faces), correct closed-form volume, every seam-ring node on BOTH
walls ≤ tol, the seam ring pooled ONCE; apex-in-extent / transversal / cap-tangent / two-circle
fixtures → NULL (deferred, no native solid). **sim** = native-vs-OCCT `BRepAlgoAPI_Common` parity
on a booted iOS simulator (volume, surface area, watertight closed shell, valid shape) via
`tests/sim/native_ssi_curved_boolean_parity.mm` / `scripts/run-sim-native-ssi-curved-boolean.sh`
— native-pass 12 → 13 (14 if cone∩sphere COMMON lands). Invoked behind the existing `cc_boolean`
op codes — **no `cc_*` entry point is added or changed**; asserted at the
`cybercad::native::boolean` C++ boundary. Compiled under **`CYBERCAD_HAS_NUMSCI`**.
`src/native/**` stays OCCT-free.

## 1. Gate + seam contract  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 Confirm (no edit) `recogniseCurvedSolid` folds a frustum solid into a `CurvedSolid` of
  `kind == Cone` (frame, `radius = R0`, `semiAngle`, axial `vLo/vHi`, planar `capPlanes`), and
  that `classifyPoint` already scores the cone wall half-space; document the axial-height vs
  intrinsic-v relation (`h = v·cosα`) in the `ssi_boolean.cpp` cone header block. (**host**)
- [x] 1.2 Confirm (no edit) `intersectCylinderConeCoaxial` emits the single physical seam circle
  (radius `Rc`, centred at `h*` where `r_c(h*) = Rc`) for a coaxial frustum∩cylinder, and that
  `ssi_boolean_solid`'s transversality gate passes it through as ONE clean WLine
  (`nearTangentGaps == 0`, `branchPoints == 0`, `status` traced/analytic). (**host**)
- [x] 1.3 `buildConeCylCommon` up-front GATE: exactly one `Cone` + one `Cylinder`, coaxial
  (`sameAxis`), EXACTLY ONE closed full-circle seam on BOTH walls, frustum apex-free over its
  extent (`r_c(v) > margin ∀ v`), seam `h*` strictly inside `(hBot, hTop)`. Any miss → return `{}`
  (→ OCCT), documented in the cone header block. (**host** — apex-in-extent / transversal /
  cap-tangent fixtures → NULL)

## 2. `appendConeBand` — the frustum ring strip  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 `appendConeBand(cone, hFrom, hTo, seamRing, pool, faces)` — emit a planar-facet frustum
  band between axial stations `hFrom..hTo` as ring strips: the seam-side terminal ring IS the
  shared pooled `seamRing` (welds to the cylinder band), the far terminal ring a fresh pooled ring
  (welds to a cap); `nu` (u-count) matches the seam ring; station count from the slant-length /
  sagitta bound; each facet's outward normal the TRUE cone normal (`radial·cosα − ẑ·sinα`). (**host**)
- [x] 2.2 `appendConeBand` returns `false` (→ NULL upstream) if a station falls on the apex
  (`r_c ≤ margin`) or the band is degenerate — a frustum whose extent reaches the apex declines →
  OCCT, never faked. (**host** ✓ explicit up-front guard)
- [x] 2.3 `appendCylBand(cyl, hFrom, hTo, seamRing, pool, faces)` — the constant-radius cylinder
  band (pure-radial normal), seam ring as one terminal, a fresh pooled ring the other (reuse /
  factor from `appendTubeBand`). (**host**)

## 3. `buildConeCylCommon` — frustum band + cyl band + two caps  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Gate (task 1.3) → resample the traced seam circle into `nu` nodes pooled ONCE (the
  shared `seamRing`); compute `[hBot, hTop]` overlap (empty → `{}`); read which band lives on
  which side of `h*` from `classifyPoint` (widening vs narrowing cone, not hard-coded). (**host**)
- [x] 3.2 CONE band: `appendConeBand(cone, <cone-side terminal h>, h*, seamRing, …)`, kept iff its
  mid-sample classifies INSIDE the cylinder (`classifyPoint(cyl, mid) == 1`; ON `== 0` → `{}`).
  CYLINDER band: `appendCylBand(cyl, h*, <cyl-side terminal h>, seamRing, …)`, kept iff its
  mid-sample classifies INSIDE the cone (`classifyPoint(cone, mid) == 1`; ON `== 0` → `{}`).
  (**host**)
- [x] 3.3 Two caps: `appendDiskCap` at `hBot` (`−ẑ` outward) and `hTop` (`+ẑ` outward), terminal
  rims pooled with the bands. `makeShell → makeSolid`; too few faces / non-watertight → `{}`.
  Measured volume matches `V_frustum(r(hBot) → Rc) + π Rc²·(hTop − h*)` within the deflection band
  (reference fixture ≈ `9.1315`). (**host** ✓)

## 4. Driver dispatch  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 In `ssi_boolean_solid`'s `Op::Common` arm: after `buildCommon` (through-drill, declines
  a single seam) and `buildLensCommon` (declines a non-sphere operand), try `buildConeCylCommon`
  (then optionally `buildConeSphereCommon`). Recognition, trace, transversality gate, and the
  through-drill / lens / Steinmetz builders UNCHANGED; a non-cone pair keeps its existing path (no
  regression). `Op::Fuse` / `Op::Cut` NOT wired for cone pairs → NULL → OCCT. (**host**)

## 5. Engine self-verify — the coaxial-cone analytic oracle
- [x] 5.1 Extend `ssiCurvedBooleanVerified` (`native_engine.cpp`, `op == 2` branch): recognise one
  `Cone` + one coaxial `Cylinder` (apex-free, single interior seam), compute the closed-form
  `expected = (π·Δcone/3)(rBot² + rBot·Rc + Rc²) + π·Rc²·(hTop − h*)`, accept iff `watertight` AND
  `|vr − expected| ≤ max(1e-2·expected, 1e-6)`; else `matched = false` → DISCARD → OCCT. Mirrors
  the Steinmetz `16 R³/3` oracle; the oracle lives in the ENGINE (library stays OCCT-free).
  (**host**)
- [x] 5.2 A mis-selected band, a mis-placed cap, or a hairline seam-ring gap yields the wrong
  volume, FAILS the guard, and is DISCARDED → OCCT — the engine never emits an unverified cone
  common. (**host** ✓ wrong-volume candidate discarded)

## 6. (Optional) `buildConeSphereCommon` — coaxial cone∩sphere COMMON  [CYBERCAD_HAS_NUMSCI]
- [ ] 6.1 Gate to the SINGLE-crossing config (`intersectSphereConeCoaxial` returns exactly ONE
  circle inside both extents; the two-circle case → `{}` → OCCT). Assemble the frustum band welded
  to the spherical-segment band along the seam circle, closed by the terminal caps. Closed-form
  oracle `V = V_frustum + V_spherical-segment` (add the sphere branch to
  `ssiCurvedBooleanVerified`). Shipped ONLY if watertight in the verified envelope; else deferred →
  OCCT with the measured gap reported. (**host** + **sim**)

## 7. Honest scope — deferrals (never faked)
- [x] 7.1 Apex-crossing / apex-in-extent frustums, TRANSVERSAL (non-coaxial) cone∩cylinder /
  cone∩sphere (quartic space curve — `intersectCylinderConeCoaxial` returns `notAnalytic`), the
  two-circle coaxial cone∩sphere crossing, cone∩cone (any config), and cone FUSE/CUT → NULL →
  OCCT, documented in the `ssi_boolean.cpp` cone header block. The cyl / sphere / Steinmetz builders
  and the through-drill / lens paths UNCHANGED. (**host** ✓ docs + NULL fixtures)

## 8. Verification (two gates, dual oracle, no weakened tolerance)
- [x] 8.1 Host suite (extend `test_native_ssi_curved_boolean`): the reference coaxial cone∩cylinder
  pair COMMON → watertight, volume matches the closed form within the deflection band, seam-ring
  nodes on both walls ≤ tol, seam ring pooled once; apex-in-extent / transversal / cap-tangent /
  two-circle → NULL. Full CTest green NUMSCI on AND off (cone tests absent with NUMSCI off). (**host**)
- [x] 8.2 Sim: `scripts/run-sim-native-ssi-curved-boolean.sh` on a booted simulator
  (`xcrun simctl list devices booted`) — the coaxial cone∩cylinder pair's COMMON becomes a native
  pass vs `BRepAlgoAPI_Common` (volume ≈ `9.1315`, surface area, watertight closed shell, valid
  shape); native-pass **12 → 13** (14 if cone∩sphere COMMON lands). Do NOT regress the 12 existing
  native passes (through-drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT + Steinmetz
  COMMON/FUSE/CUT). Any pair whose self-verify does not pass stays an honest fall-back with the
  measured gap reported — no tolerance weakened. (**sim**)
- [x] 8.3 `openspec validate add-native-cone-boolean --strict` green; record the CONE family opening
  in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with the measured deltas. Confirm no SSI
  / blend / heal / import / marching / phase3 suite regresses. (**host** + **sim**)

## Out of scope (NOT in this change — honest)
- [ ] Cone FUSE / CUT (any pair) → UNCHANGED, NULL → OCCT.
- [ ] TRANSVERSAL (non-coaxial) cone∩cylinder / cone∩sphere, apex-crossing seams, frustum extents
  including the apex, the two-circle coaxial cone∩sphere crossing, cone∩cone → OCCT.
- [ ] Any other curved-curved family (through-drill cyl∩cyl, sphere∩sphere, Steinmetz) → UNCHANGED.
- [ ] Freeform (NURBS / Bézier) operand faces → OCCT.
- [ ] Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, or the cyl / sphere / Steinmetz builders.
