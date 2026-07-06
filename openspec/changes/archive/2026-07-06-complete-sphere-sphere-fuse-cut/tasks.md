# Tasks — complete-sphere-sphere-fuse-cut (SSI Stage S5-c completion)

Verification levels: **host** = OCCT-free host CTest — analytic spherical-cap volumes
(`V_cap = π h² (3R − h) / 3`, `lens = V_cap(A)+V_cap(B)`, `FUSE = V(A)+V(B)−lens`,
`CUT(A,B) = V(A)−lens`), watertight (`boundaryEdgeCount == 0`, every edge shared by exactly
two faces), correct set-algebra volume, seam nodes on BOTH spheres ≤ tol; tangent /
degenerate fixtures → NULL (deferred, no native solid). **sim** = native-vs-OCCT
`BRepAlgoAPI_{Fuse,Cut}` parity on a booted iOS simulator (volume, surface area, watertight
closed shell, valid shape) via `tests/sim/native_ssi_curved_boolean_parity.mm` /
`scripts/run-sim-native-ssi-curved-boolean.sh` — native-pass 6 → 8. Invoked behind the
existing `cc_boolean` op codes — **no `cc_*` entry point is added or changed**; asserted at
the `cybercad::native::boolean` C++ boundary. Compiled under **`CYBERCAD_HAS_NUMSCI`**.
`src/native/**` stays OCCT-free.

## 1. Generalise `appendSphereCap` (inner/outer apex + reversed normal)  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 Add two defaulted bools to `appendSphereCap`: `bool outer = false`
  (apexDir = `outer ? −unit(otherCentre−centre) : +unit(...)`, the far pole vs the near
  apex) and `bool reversed = false` (facet outward reference = `reversed ? (centre−ctr) :
  (ctr−centre)`, flips the whole cap's normal via `pushPlanarTri`'s reference arg). The
  ring/slerp/fan loop and the `r==rings → seam.pts[k]` outer ring are UNCHANGED. (**host**)
- [x] 1.2 Confirm `buildLensCommon`'s two calls use the defaults (`outer=false,
  reversed=false`) so COMMON output is BYTE-IDENTICAL (no volume/area/vertex change vs
  main). (**host** ✓ COMMON regression golden unchanged)

## 2. `buildLensFuse` — two OUTER caps  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 Mirror `buildLensCommon`: gate (`seams.size()==1`, closed, ≥4 pts, both Sphere,
  `|cA−cB|>eps`); `capSeam = decimateSeam(seam, seamNodeTarget(seam))` — the SAME shared
  seam so the two caps weld. (**host**)
- [x] 2.2 Survival = each far pole OUTSIDE the other solid: `classifyPoint(B, outerApexA) !=
  1` AND `classifyPoint(A, outerApexB) != 1` (a far pole INSIDE the other = containment/
  tangent → NULL → OCCT, never guessed). (**host** ✓ containment fixture → NULL)
- [x] 2.3 Two `appendSphereCap(..., outer=true, reversed=false)` (OUTER-A, OUTER-B,
  outward), `ringsFor` sizing each from its true apex→seam polar angle; `makeShell →
  makeSolid`; `<4 faces → NULL`. Volume = `V(A)+V(B)−lens`. (**host**)

## 3. `buildLensCut` — OUTER-A + reversed INNER-B  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Mirror `buildLensCommon`; A is the minuend (CUT is order-sensitive, matches
  `BRepAlgoAPI_Cut(a,b)`); SAME `capSeam`. (**host**)
- [x] 3.2 Survival = outer apex of A OUTSIDE B AND inner apex of B INSIDE A (else NULL →
  OCCT). `appendSphereCap(A, cB, ..., outer=true, reversed=false)` (OUTER-A, outward) +
  `appendSphereCap(B, cA, ..., outer=false, reversed=true)` (INNER-B, INWARD — bounds the
  cavity). `makeShell → makeSolid`; `<4 faces → NULL`. Volume = `V(A)−lens`. (**host**)

## 4. Driver dispatch  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 In `ssi_boolean_solid`'s `switch(op)`: `Op::Fuse` → try through-drill `buildFuse`
  (NULL for 1 seam) then `buildLensFuse`; `Op::Cut` → try through-drill `buildCut` (NULL for
  1 seam) then `buildLensCut` — mirroring the existing `Op::Common` dispatch. Gate + trace
  UNCHANGED; the through-drill S5-b fuse/cut path is untouched (still tried first). (**host**)

## 5. Engine self-verify — per-op sign (fuse grows, cut shrinks)
- [x] 5.1 CONFIRM (no edit) the generic guard `booleanResultVerified` already computes
  `expected = va+vb−vc` (fuse) / `va−vc` (cut) with `vc =` native `buildLensCommon`, so
  FUSE grows (`Vr > max(VA,VB)`) and CUT shrinks (`Vr < VA`) against the native lens. The
  Steinmetz oracle (`op==2` equal-R ⟂ cyl) does NOT intercept sphere fuse/cut. (**host**)
- [x] 5.2 A mis-welded / wrong-signed candidate FAILS the guard and is DISCARDED → OCCT —
  the engine never emits an unverified sphere fuse/cut. (**host** ✓ wrong-volume candidate
  discarded)

## 6. Honest scope — deferrals (never faked)
- [x] 6.1 Tangent / coincident / near-containment spheres (`nearTangentGaps > 0`, or an apex
  landing on the wrong side of the other sphere for the op's rule) → NULL → OCCT, documented
  in the `ssi_boolean.cpp` sphere-lens header block. Other curved-curved families and the
  through-drill / Steinmetz paths UNCHANGED. (**host** ✓ docs + NULL fixtures)

## 7. Verification (two gates, dual oracle, no weakened tolerance)
- [x] 7.1 Host suite (extend `test_native_ssi_curved_boolean` or the S5-c test): equal- and
  unequal-radius overlapping sphere pairs, FUSE + CUT → watertight, volume matches the
  analytic spherical-cap closed form within the deflection band, seam nodes on both spheres
  ≤ tol; tangent/degenerate → NULL. Full CTest green NUMSCI on AND off (sphere fuse/cut
  tests absent with NUMSCI off). (**host**)
- [x] 7.2 Sim: `scripts/run-sim-native-ssi-curved-boolean.sh` on a booted simulator — the
  two sphere-lens pairs' FUSE + CUT become native passes vs `BRepAlgoAPI_{Fuse,Cut}`
  (volume, surface area, watertight closed shell, valid shape); native-pass **6 → 8**. Do
  NOT regress the 6 existing native passes (drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere
  COMMON + Steinmetz COMMON). Any pair whose self-verify does not pass stays an honest
  fall-back with the measured gap reported — no tolerance weakened. (**sim**)
- [x] 7.3 `openspec validate complete-sphere-sphere-fuse-cut --strict` green; note the
  sphere∩sphere op-set now 3/3 native in `SSI-ROADMAP.md` / `ROADMAP.md` /
  `NATIVE-REWRITE.md` with the measured deltas. Confirm no SSI / blend / heal / import /
  marching suite regresses. (**host** + **sim**)

## Out of scope (NOT in this change — honest)
- [ ] Any other curved-curved family (cyl∩cyl through-drill / Steinmetz FUSE/CUT beyond
  what already ships, sphere/cone∩box, cyl∩cone, cyl∩sphere, cone∩cone, oblique cyl∩cyl) →
  UNCHANGED, existing native/decline behaviour.
- [ ] Near-tangent / coincident / branch-point pairs (`nearTangentGaps > 0`) → S4 + OCCT.
- [ ] Freeform (NURBS / Bézier) operand faces → OCCT.
- [ ] Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the
  analytic `curved.h`, the through-drill `buildCommon/Fuse/Cut`, or `buildLensCommon`.
