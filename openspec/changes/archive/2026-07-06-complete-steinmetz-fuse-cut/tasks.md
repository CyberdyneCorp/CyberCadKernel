# Tasks — complete-steinmetz-fuse-cut (SSI Stage S5-d completion)

Verification levels: **host** = OCCT-free host CTest — analytic inclusion-exclusion on the
exact bicylinder common (`V(A ∩ B) = 16 R³/3`) and cylinder volumes (`V = π R² L`):
`FUSE = π R²(L_A + L_B) − 16 R³/3`, `CUT(A,B) = π R² L_A − 16 R³/3`; watertight
(`boundaryEdgeCount == 0`, every edge shared by exactly two faces), correct set-algebra volume,
every arc node on BOTH cylinders ≤ tol, the two poles pooled ONCE; short-cylinder / non-Steinmetz
fixtures → NULL (deferred, no native solid). **sim** = native-vs-OCCT `BRepAlgoAPI_{Fuse,Cut}`
parity on a booted iOS simulator (volume, surface area, watertight closed shell, valid shape) via
`tests/sim/native_ssi_curved_boolean_parity.mm` / `scripts/run-sim-native-ssi-curved-boolean.sh`
— native-pass 10 → 12. Invoked behind the existing `cc_boolean` op codes — **no `cc_*` entry
point is added or changed**; asserted at the `cybercad::native::boolean` C++ boundary. Compiled
under **`CYBERCAD_HAS_NUMSCI`**. `src/native/**` stays OCCT-free.

## 1. Factor the shared arc/pole prologue  [CYBERCAD_HAS_NUMSCI]
- [x] 1.1 Extracted `orientResampleArcs(refR, st) -> std::vector<BranchArcData>` from
  `buildSteinmetzCommon`'s inline prologue: `axisVec = pole1 − pole0`, axisLen guard, `nn` from
  the arc-sagitta bound clamped `[24,180]`, cosine `tvals`, `orientArc` → `resampleArcByAxis` per
  arc, snap grid endpoints to the exact shared poles. Returns the four resampled arcs (empty on a
  degenerate pole axis). (**host**)
- [x] 1.2 `buildSteinmetzCommon` calls `orientResampleArcs` and produces the BYTE-IDENTICAL
  four-inside-lune shell — COMMON native pass does not regress (sim volN=5.3287 unchanged vs
  baseline; dV=8.75e-04). (**host** ✓ COMMON regression golden unchanged)

## 2. OUTSIDE wall + two disc caps  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 IMPLEMENTATION NOTE — the OUTSIDE-the-other wall is NOT a simple two-arc lune: it wraps
  around and reaches the caps at `v=vLo/vHi` (the whole wall MINUS the two inside lunes). So it is
  assembled EXACTLY like the through-drill fat wall — `appendHoledWall(cs, mouth0, mouth1, …)`
  with the two INSIDE lunes as the two mouths (each a closed (u,v) loop built by `luneMouthSeam`
  concatenating its two arcs pole0→pole1→pole0) — then the two disc caps via the existing
  `appendDiskCap` fanned onto the wall's shared rim rings (`−axis` at vLo, `+axis` at vHi). This is
  MORE correct than the "outside-lune patch" sketch (which would drop the cap-adjacent full-ring
  bands and mis-volume). Packaged in `appendSteinmetzOuterWall(cs, arcs, which, pool, faces)`.
  (**host**)
- [x] 2.2 `appendSteinmetzOuterWall` returns false (→ NULL upstream) if a cap plane falls within
  the seam band (mouth v-extent reaches `vLo/vHi` — a short cylinder, out of scope) so a
  short-cylinder Steinmetz declines → OCCT, never faked. (**host** ✓ explicit up-front guard)

## 3. `buildSteinmetzFuse` — both outside walls + four caps  [CYBERCAD_HAS_NUMSCI]
- [x] 3.1 Mirror `buildSteinmetzCommon`: `orientResampleArcs(A, st)` (SAME arcs). (**host**)
- [x] 3.2 `appendSteinmetzOuterWall(A, arcs, LuneCyl::A, …)` + `appendSteinmetzOuterWall(B, arcs,
  LuneCyl::B, …)`: each emits that cylinder's OUTSIDE wall (full wall minus its two inside lunes,
  via `appendHoledWall` with the two inside lunes as mouths) + its two disc caps, on the SHARED
  pooled arc nodes so the two walls weld along the four arcs. (**host**)
- [x] 3.3 `makeShell → makeSolid`; too few faces / short-cylinder → NULL. Measured volume =
  32.385 (analytic `V(A)+V(B)−16R³/3` = 32.366, dRel = 5.9e-4), watertight. (**host** ✓)

## 4. `buildSteinmetzCut` — A outside + A caps + B inside REVERSED  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 Mirror `buildSteinmetzCommon`; A is the minuend (CUT is order-sensitive, matches
  `BRepAlgoAPI_Cut(a,b)`); SAME `orientResampleArcs(A, st)` arcs. (**host**)
- [x] 4.2 Emit A's OUTSIDE wall + A's two disc caps via `appendSteinmetzOuterWall(A, arcs,
  LuneCyl::A, …)`; then B's two INSIDE-A lunes REVERSED `appendLunePatch(B, LuneCyl::B, …,
  outwardSign=-1.0)` (INWARD — bounds the carved channel), kept iff their centroid classifies
  INSIDE A (`classifyPoint(A, centroid) == 1`; an ON verdict `== 0` → NULL → OCCT), all sharing
  the four arcs + two poles. `makeShell → makeSolid`; too few faces → NULL. Measured volume =
  13.526 (analytic `V(A)−16R³/3` = 13.516, dRel = 7.4e-4), watertight. (**host** ✓)

## 5. Driver dispatch  [CYBERCAD_HAS_NUMSCI]
- [x] 5.1 In `tryBranchedSteinmetz`'s `op` switch: `Op::Fuse` → `buildSteinmetzFuse`, `Op::Cut` →
  `buildSteinmetzCut`, mirroring the existing `Op::Common` → `buildSteinmetzCommon`. Pre-gate +
  branch-enabled re-trace + `recogniseSteinmetzTrace` UNCHANGED; every single-seam S5-a/b/c pass
  keeps its default trace (no re-trace, no regression). (**host**)

## 6. Engine self-verify — per-op sign (fuse grows, cut shrinks)
- [x] 6.1 CONFIRM (no edit) `ssiCurvedBooleanVerified` returns `{}` for `op != 2`, so the
  Steinmetz analytic `16 R³/3` oracle does NOT intercept fuse/cut; the generic
  `booleanResultVerified` computes `expected = va+vb−vc` (fuse) / `va−vc` (cut) with `vc =` native
  `buildSteinmetzCommon` (`= 16 R³/3`), so FUSE grows (`Vr > max(VA,VB)`) and CUT shrinks
  (`Vr < VA`) against the native bicylinder. (**host**)
- [x] 6.2 A mis-selected / mis-oriented / non-watertight candidate FAILS the guard and is
  DISCARDED → OCCT — the engine never emits an unverified Steinmetz fuse/cut. (**host** ✓
  wrong-volume candidate discarded)

## 7. Honest scope — deferrals (never faked)
- [x] 7.1 Short cylinders whose end caps clip the seam band, non-equal-radius / non-orthogonal /
  non-crossing pairs (`steinmetzPreGate` false), and unresolved branched traces
  (`recogniseSteinmetzTrace` nullopt) → NULL → OCCT, documented in the `ssi_boolean.cpp`
  Steinmetz header block. Other curved-curved families and the through-drill / sphere-lens paths
  UNCHANGED. (**host** ✓ docs + NULL fixtures)

## 8. Verification (two gates, dual oracle, no weakened tolerance)
- [x] 8.1 Host suite (extend `test_native_ssi_curved_boolean` or the S5-d test): the equal-R
  orthogonal Steinmetz pair, FUSE + CUT → watertight, volume matches the analytic
  inclusion-exclusion closed form within the deflection band, arc nodes on both cylinders ≤ tol,
  poles pooled once; short-cylinder / non-Steinmetz → NULL. Full CTest green NUMSCI on AND off
  (Steinmetz fuse/cut tests absent with NUMSCI off). (**host**)
- [x] 8.2 Sim: `scripts/run-sim-native-ssi-curved-boolean.sh` on a booted simulator
  (`xcrun simctl list devices booted`) — the `cyl=cyl(steinmetz)` pair's FUSE + CUT become native
  passes vs `BRepAlgoAPI_{Fuse,Cut}` (volume `≈ 32.366` fuse / `≈ 13.516` cut, surface area,
  watertight closed shell, valid shape); native-pass **10 → 12**. Do NOT regress the 10 existing
  native passes (through-drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT + Steinmetz
  COMMON). Any pair whose self-verify does not pass stays an honest fall-back with the measured
  gap reported — no tolerance weakened. (**sim**)
- [x] 8.3 `openspec validate complete-steinmetz-fuse-cut --strict` green; note the Steinmetz
  op-set now 3/3 native in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` with the measured
  deltas. Confirm no SSI / blend / heal / import / marching / phase3 suite regresses. (**host** +
  **sim**)

## Out of scope (NOT in this change — honest)
- [ ] Any other curved-curved family (through-drill cyl∩cyl FUSE/CUT beyond what already ships,
  sphere∩sphere, sphere/cone∩box, cyl∩cone, cyl∩sphere, cone∩cone, oblique cyl∩cyl) → UNCHANGED,
  existing native/decline behaviour.
- [ ] Short cylinders (end caps within the seam band), non-equal-radius / non-orthogonal /
  non-crossing cylinder pairs, unresolved branched traces → OCCT.
- [ ] Freeform (NURBS / Bézier) operand faces → OCCT.
- [ ] Any change to `src/native/tessellate`, the `cc_*` ABI, the planar BSP-CSG, the analytic
  `curved.h`, the through-drill / sphere-lens builders, or `buildSteinmetzCommon`.
