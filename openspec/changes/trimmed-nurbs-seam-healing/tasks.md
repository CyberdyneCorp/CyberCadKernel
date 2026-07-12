# Tasks — trimmed-nurbs-seam-healing

## Implementation

- [x] Add `SurfacePeriod` + `surfacePeriod()` to `trimmed_nurbs.h`/`.cpp`: Cylinder/Cone/Sphere
      periodic in u (2π); Torus in both; Plane/BSpline/Bezier non-periodic (honest residual).
- [x] Add `SeamHealReport` + `loopCrossesSeam()` / `healSeamLoop()` / `healTrimLoopSeam()`
      (declarations + contract docs in the header).
- [x] Implement seam detection + unwrap in `trimmed_nurbs.cpp`:
  - [x] `isSeamJump()` — a consecutive u-jump within tol of ±one full period.
  - [x] `loopCrossesSeam()` — any consecutive step (incl. the closing edge) is a ~period jump.
  - [x] `loopTangentToSeam()` — a vertex ON the seam whose neighbours sit on the SAME side (a
        touch-without-cross) → the ambiguous seam-tangent case.
  - [x] `healSeamLoop()` — decline a seam-tangent graze; unwrap the u by ∓period at each jump;
        gate on the closing residual being an integer number of periods; classify FULL WRAP
        (u-span ≥ ~period) vs FINITE STRADDLING (u-span < period, unwrapped loop must be simple);
        honest `ambiguous` decline otherwise.
- [x] Add `classifySeam()` — a strict NO-OP superset of `classify()` (non-periodic surface or a
      loop not touching the seam defers to `classify` byte-for-byte); for a seam-crossing outer
      loop, reduce the query u into the unwrapped window and classify by the v-band (full wrap) or
      the raycast (straddling); seam-heal holes the same way; decline `Unknown` on ambiguity.

## Tests (regression, airtight oracles — `test_native_trimmed_nurbs_seam`)

- [x] Periodicity detection: Cylinder/Cone/Sphere periodic in u; Torus in both; Plane / BSpline
      non-periodic (no-op).
- [x] FULL WRAP: a band wrapping the whole u-period heals into one seam loop (`fullWrap`, u-span
      ≈ 2π); a point whose v is inside the band classifies `In` for EVERY u, outside → `Out`, on
      the band edge → `OnBoundary`.
- [x] HALF WRAP: a finite region straddling the seam heals into one simple loop (winding 0); every
      interior/exterior probe classifies IDENTICALLY to the reference region.
- [x] NON-CROSSING NO-OP: a loop inside one period → `classifySeam == classify` byte-for-byte.
- [x] NON-PERIODIC NO-OP: on a Plane, `classifySeam == classify` for every probe; `healTrimLoopSeam`
      is an echo (healed, not crossing).
- [x] AMBIGUOUS DECLINE: a seam-tangent loop (touch-without-cross) is `ambiguous` / not healed, and
      `classifySeam` returns `Unknown` — never a fabricated full band.

## Verification

- [x] `src/native` stays OCCT-free (0 OCCT/BRep/Geom/TK refs in changed files).
- [x] `cc_*` ABI byte-unchanged (additive only); seam verbs are new, seam behaviour opt-in.
- [x] `test_native_trimmed_nurbs` (existing) + `test_native_trimmed_nurbs_seam` (new) pass; full
      `ctest` green.
- [x] `openspec validate --all` passes.
- [x] Update the L3 readiness / roadmap doc: seam-healing residual now CLOSED for the analytic-
      periodic family; closed free-form seam-period detection remains the explicit residual.
