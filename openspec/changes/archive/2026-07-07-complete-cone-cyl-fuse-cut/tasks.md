# Tasks — complete-cone-cyl-fuse-cut (SSI Stage S5-e completion)

Verification levels: **host** = OCCT-free host CTest — analytic inclusion-exclusion on the exact
cone∩cylinder common (`V(A ∩ B) = V_frustum(r(sLo) → Rc) + π Rc²·(sHi − s*)`,
`V_frustum(ra → rb over Δh) = (π Δh/3)(ra² + ra·rb + rb²)`) and the operand volumes
(`V(cone frustum) = (π Δh/3)(r0² + r0·r1 + r1²)`, `V(cyl) = π Rc² L`):
`FUSE = V(A) + V(B) − V(A ∩ B)`, `CUT(A,B) = V(A) − V(A ∩ B)`; watertight (`boundaryEdgeCount == 0`,
every edge shared by exactly two faces), correct set-algebra volume (summed over components for a
disconnected CUT), every seam-ring node on BOTH walls ≤ tol, the seam ring pooled ONCE; apex-in-
extent / transversal / cap-tangent fixtures → NULL (deferred, no native solid). **sim** = native-
vs-OCCT `BRepAlgoAPI_{Fuse,Cut}` parity on a booted iOS simulator (volume, surface area, watertight
closed shell, valid shape) via `tests/sim/native_ssi_curved_boolean_parity.mm` /
`scripts/run-sim-native-ssi-curved-boolean.sh` — native-pass 13 → 15. Invoked behind the existing
`cc_boolean` op codes — **no `cc_*` entry point is added or changed**; asserted at the
`cybercad::native::boolean` C++ boundary. Compiled under **`CYBERCAD_HAS_NUMSCI`**.
`src/native/**` stays OCCT-free.

## 1. Factor the shared gate/seam prologue  [CYBERCAD_HAS_NUMSCI]
- [ ] 1.1 Extract `coneCylSetup(A, B, seams) -> ConeCylSetup` from `buildConeCylCommon`'s inline
  prologue: the gate (one `Cone` + one `Cylinder`, coaxial via `ssidetail::sameAxis`, a single
  closed full-circle seam, non-degenerate `tanα`, apex-free frustum), the cylinder `s`-extent in
  the cone's `s`-coordinate (sign-corrected for antiparallel axes), the overlap `[sLo, sHi]`, the
  single interior crossing `s* = (Rc − R0)/tanα` (declines apex/edge crossings), the analytic-vs-
  traced seam cross-check (centroid height `s*`, mean radius `Rc`), the azimuth resolution `N`, the
  shared frame `(O, ẑ, X, Y)`, the `rCone(s)` and `ring(r, s)` functors, and the pooled seam ring
  at `(Rc, s*)`. Returns `ok == false` on any decline. (**host**)
- [ ] 1.2 `buildConeCylCommon` calls `coneCylSetup` and produces the BYTE-IDENTICAL two-inside-band
  + two-disc-cap shell — COMMON native pass does not regress (sim volN unchanged vs baseline
  `19.107`, watertight). (**host** ✓ COMMON regression golden unchanged)

## 2. `appendAnnulusCap` — the flat washer helper  [CYBERCAD_HAS_NUMSCI]
- [ ] 2.1 `appendAnnulusCap(ringIn, ringOut, axialOutward, pool, faces)` — emit a flat annular ring
  between two coaxial same-station rings (inner + outer radius, `N` u-aligned nodes each) as planar
  facets with a FIXED axial `±ẑ` outward normal, both rings drawn through the shared pool so the
  washer welds to the inner-edge and outer-edge rings. Mirrors `appendRevolvedBand` but forces the
  normal AXIAL. A degenerate (equal-radius) annulus is skipped (walls meet directly). (**host**)

## 3. `buildConeCylFuse` — max-radius outer profile + caps + annuli  [CYBERCAD_HAS_NUMSCI]
- [ ] 3.1 Mirror `buildConeCylCommon`: `coneCylSetup(A, B, seams)` (SAME frame + `s*` + seam ring).
  Build the ordered profile stations from the sorted `{coneLo, cylLo, s*, coneHi, cylHi}` restricted
  to the union extent `[min(coneLo,cylLo), max(coneHi,cylHi)]`; at each station the OUTER radius =
  max of the present operands' radii, reusing the pooled seam ring at `s*`. (**host**)
- [ ] 3.2 Emit the outer wall bands (`appendRevolvedBand`, outward radial), each KEPT iff its mid-
  sample classifies strictly OUTSIDE the other solid (`classifyPoint(other, mid) == -1`; ON `== 0`
  → `{}`); the annular step caps (`appendAnnulusCap`, `±ẑ`) where an end-cap disc protrudes; and the
  two terminal disc caps (`appendDiskCap`, min-s `−ẑ`, max-s `+ẑ`). `makeShell → makeSolid`; too few
  faces → `{}`. Measured volume = `V(A) + V(B) − V(A ∩ B)` within the deflection band (reference
  fixture ≈ `41.626`), watertight, a GROW. (**host** ✓)

## 4. `buildConeCylCut` — A outer + A caps + reversed B inside  [CYBERCAD_HAS_NUMSCI]
- [ ] 4.1 Mirror `buildConeCylCommon`; `A` is the minuend (CUT is order-sensitive, matches
  `BRepAlgoAPI_Cut(a, b)`); SAME `coneCylSetup(A, B, seams)`. Emit A's OUTER wall bands (outside B,
  `appendRevolvedBand` outward, kept iff mid-sample `classifyPoint(B, mid) == -1`), A's terminal
  disc cap(s) outside B (`appendDiskCap ±ẑ`), and A's cap-annulus where A's disc extends past B
  (`appendAnnulusCap ±ẑ`). (**host**)
- [ ] 4.2 Emit B's INSIDE-A wall band REVERSED (`appendRevolvedBand` with the INWARD radial
  reference — bounds the carved cavity, pinching to the pooled seam ring), kept iff its mid-sample
  classifies strictly INSIDE A (`classifyPoint(A, mid) == 1`; ON `== 0` → `{}`); and B's end-cap
  disc inside A REVERSED (`appendDiskCap` with the opposite axial normal — the cavity floor/ceiling).
  `makeShell → makeSolid` (one shell may carry two disjoint closed components, e.g. the fixture's
  end-frustum + conical washer). Measured volume = `V(A) − V(A ∩ B)` within the deflection band
  (reference fixture ≈ `13.352`, summed over both components), watertight per component, a SHRINK.
  (**host** ✓)

## 5. Driver dispatch  [CYBERCAD_HAS_NUMSCI]
- [ ] 5.1 In `ssi_boolean_solid`: `Op::Fuse` arm → after `buildFuse` (through-drill) and
  `buildLensFuse` (sphere lens) decline, `return buildConeCylFuse(*csA, *csB, seams)`; `Op::Cut`
  arm → after `buildCut` and `buildLensCut` decline, `return buildConeCylCut(*csA, *csB, seams)`,
  mirroring the existing `Op::Common` → `buildConeCylCommon`. Recognition, trace, the transversality
  gate, and every other builder UNCHANGED; a non-(cone+coaxial-cylinder) pair keeps its existing
  path (no regression). (**host**)

## 6. Engine self-verify — per-op sign (fuse grows, cut shrinks)
- [ ] 6.1 CONFIRM (no edit) `ssiCurvedBooleanVerified` returns `{}` for `op != 2`, so the cone∩
  cylinder analytic closed-form oracle does NOT intercept fuse/cut; the generic
  `booleanResultVerified` computes `expected = va + vb − vc` (fuse) / `va − vc` (cut) with `vc =`
  native `buildConeCylCommon` (`= V(A ∩ B)`), so FUSE grows (`Vr > max(VA, VB)`) and CUT shrinks
  (`Vr < VA`) against the native cone∩cylinder common. (**host**)
- [ ] 6.2 A mis-selected band, a mis-oriented reversed fragment, a mis-placed annular cap, or a
  hairline seam-ring gap yields the wrong volume, FAILS the guard, and is DISCARDED → OCCT — the
  engine never emits an unverified cone∩cylinder fuse/cut. (**host** ✓ wrong-volume candidate
  discarded)

## 7. Honest scope — deferrals (never faked)
- [ ] 7.1 Apex-crossing / apex-in-extent frustums, TRANSVERSAL (non-coaxial) cone∩cylinder (quartic
  space curve — `intersectCylinderConeCoaxial` returns `notAnalytic`), cap-edge-tangent seams,
  coaxial cone∩sphere (any op), and cone∩cone (any op) → NULL → OCCT, documented in the
  `ssi_boolean.cpp` cone header block. The cyl / sphere / Steinmetz builders, the through-drill /
  lens paths, and `buildConeCylCommon` UNCHANGED. (**host** ✓ docs + NULL fixtures)

## 8. Verification (two gates, dual oracle, no weakened tolerance)
- [ ] 8.1 Host suite (extend `test_native_ssi_curved_boolean` or the S5-e test): the reference
  coaxial cone∩cylinder pair FUSE + CUT → watertight, volume matches the analytic inclusion-
  exclusion closed form within the deflection band, seam-ring nodes on both walls ≤ tol, seam ring
  pooled once; apex-in-extent / transversal / cap-tangent → NULL. Full CTest green NUMSCI on AND off
  (cone fuse/cut tests absent with NUMSCI off). COMMON regression golden unchanged (byte-identical).
  (**host**)
- [ ] 8.2 Sim: `scripts/run-sim-native-ssi-curved-boolean.sh` on a booted simulator
  (`xcrun simctl list devices booted`) — the `cone=cyl(coax)` pair's FUSE + CUT become native passes
  vs `BRepAlgoAPI_{Fuse,Cut}` (volume ≈ `41.626` fuse / `≈ 13.352` cut, surface area, watertight
  closed shell, valid shape); native-pass **13 → 15**. Do NOT regress the 13 existing native passes
  (through-drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT + Steinmetz
  COMMON/FUSE/CUT + cone∩cylinder COMMON). Any pair whose self-verify does not pass stays an honest
  fall-back with the measured gap reported — no tolerance weakened. (**sim**)
- [ ] 8.3 `openspec validate complete-cone-cyl-fuse-cut --strict` green; note the coaxial cone∩
  cylinder op-set now 3/3 native in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with the
  measured deltas. Confirm no SSI / blend / heal / import / marching / phase3 suite regresses.
  (**host** + **sim**)

## Out of scope (NOT in this change — honest)
- [ ] Coaxial cone∩sphere (any op) and cone∩cone (any op) → UNCHANGED, NULL → OCCT.
- [ ] TRANSVERSAL (non-coaxial) cone∩cylinder, apex-crossing seams, frustum extents including the
  apex, cap-edge-tangent seams → OCCT.
- [ ] Any other curved-curved family (through-drill cyl∩cyl, sphere∩sphere, Steinmetz) → UNCHANGED,
  existing native/decline behaviour.
- [ ] Freeform (NURBS / Bézier) operand faces → OCCT.
- [ ] Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the cyl / sphere / Steinmetz builders, `buildConeCylCommon`, or the engine self-verify.
