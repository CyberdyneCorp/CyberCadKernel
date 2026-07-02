# Tasks — enhance-full-round-nonparallel

Verification levels: **host** = the no-OCCT / no-Metal host build
(`/opt/homebrew/opt/llvm/bin/clang++`, `CYBERCAD_HAS_OCCT` OFF) compiles and its
CTest stays green (the stub full round is a safe no-op); **ios-sim-build** = the
kernel + Phase-3 suite compile from source for `arm64-apple-ios16.0-simulator`
with `-DCYBERCAD_HAS_OCCT=1` (Metal off) via `scripts/run-sim-phase3-suite.sh`;
**ios-sim-run** = that suite runs on the booted simulator and every `[PASS]`/`[FAIL]`
holds (deferred cases do NOT fail the suite) — this is the acceptance bar for the
non-parallel full round.

## 1. Planar-dihedral tangent-cylinder solver
- [x] 1.1 Extract a `planarDihedralAxis(info, /*out*/ gp_Pnt& c0, gp_Dir& d, double& r)`
  helper in `src/engine/occt/occt_full_round_fillet.cpp`: compute
  `d = normalize(n1 × n2)`, solve the 3×3 linear system for the centre `c0` on the
  interior bisector at perpendicular distance `r` from BOTH neighbour planes, and
  derive `r` from the seam-edge offset / strip half-width so both branches share one
  expression. Orient the bisector toward the body centroid (valley, not ridge). (**host** compiles under the OCCT guard; logic verified at **ios-sim-run**)
- [x] 1.2 Anti-parallel branch: when `|n1 × n2|` is below `kCreaseFloor`, take
  `d = edgeDirection(eL)` and `c0 = seamMid − r·nMiddleOut` (the existing parallel
  derivation) so the two branches are one code path with a single tangent-geometry
  source of truth. (**ios-sim-run**)

## 2. Widen eligibility to non-parallel planar walls
- [x] 2.1 Update `rollingBallEligible`: still require both neighbours PLANAR and
  `width`/`seamLen` above the floor; accept the NON-PARALLEL dihedral branch when
  `|n1 × n2| >= kCreaseFloor`, and keep the `n1·n2 < -0.98` (anti-)parallel branch.
  Curved neighbours still return false → fallback. (**ios-sim-run**)
- [x] 2.2 Confirm the parallel-rib fixture still takes the native path with the
  SAME eligibility outcome (no regression). (**ios-sim-run**)

## 3. Generalize the carve
- [x] 3.1 Rework `buildRollingBall` to consume `(c0, d, r)` from
  `planarDihedralAxis`: cylinder along `d` centred to span `seamLen`; corner slab
  in the generalized frame (`X = d`, `Z` = the mean/outward of `n1,n2`) so it covers
  the whole strip top for a splayed wall; cut `body − (slab − cylinder)`. Gate on
  `BRepCheck_Analyzer::IsValid`. (**ios-sim-run**)
- [x] 3.2 On an invalid boolean or a degenerate crease, fall through to the
  existing `fallbackEdgeFillet` (unchanged). (**ios-sim-run**)

## 4. On-simulator non-parallel-rib check (real properties, never trivially true)
- [x] 4.1 Add a NON-PARALLEL planar-dihedral fixture to
  `tests/sim/checks_full_round_fillet.cpp` (a rib whose two side walls meet at a
  real dihedral angle, e.g. a symmetric draft-angled rib, distinct from the existing
  splayed-trapezoid CURVED-fallback fixture). Run
  `cc_full_round_fillet_faces(body, left, middle, right)`. (**ios-sim-build**)
- [x] 4.2 Assert REAL properties on success: `cc_mass_properties(out).valid == 1`
  and positive volume (valid + watertight); the flat top at the original strip
  height is GONE (`flat_top_remains == false`, i.e. middle face consumed);
  `find_blend_cylinder` succeeds and the cylinder axis direction matches
  `normalize(n1 × n2)` (the strip direction) within tolerance. (**ios-sim-run**)
- [x] 4.3 Assert G1 tangency to BOTH neighbours for the dihedral case: the blend
  axis is at perpendicular distance `r` from BOTH neighbour planes, and the blend
  normal at each seam contact equals the neighbour outward normal within
  `cos(1°)`. (**ios-sim-run**)
- [x] 4.4 If the dihedral blend cannot be built for a sub-case, assert the returned
  shape is a VALID fallback and `ctx.defer(...)` with the measured tangency gap /
  dihedral angle — never a trivially-true check, never a faked G1/consumption pass. (**ios-sim-run**)
- [x] 4.5 Keep the existing parallel-rib checks green with unchanged expected
  numbers (`720 + 20π` volume, axis `x=0, dir ±z`), and keep the CURVED-neighbour
  (splayed) case as a valid-fallback + deferred check. (**ios-sim-run**)

## 5. ABI / build guards stay intact
- [x] 5.1 No `cc_*` signature or POD-struct change; `tests/test_abi.cpp` unchanged
  and passing. (**host**)
- [x] 5.2 The new construction is entirely under `#ifdef CYBERCAD_HAS_OCCT`; the
  host stub build stays a safe no-op (`cc_full_round_fillet*` return `0` without
  crashing) and host CTest stays green. (**host**)
- [x] 5.3 `scripts/run-sim-phase3-suite.sh` builds from current source and runs
  green (exit 0; deferred cases do not fail). (**ios-sim-run**)

## 6. Docs / spec
- [x] 6.1 Update the module header comment in
  `src/engine/occt/occt_full_round_fillet.cpp` to describe the planar-dihedral
  generalization (axis along `n1 × n2`, centre on the interior bisector) and that
  curved neighbours remain deferred. (**host**)
- [x] 6.2 On completion, sync the `full-round-fillet` delta into the living spec and
  archive this change. (**host**)
