# Tasks — moat-m2cw-curved-wall-cut (MOAT M2 curved-wall freeform CUT/COMMON)

Order: baseline capture → bowl-cup fixture + closed-form oracles → `curvedWallHalfSpaceCut`
verb (recognise → trace → smooth-trim split → wall split → circular cap synth → M0 weld
→ watertight+volume self-verify) → host analytic gate → sim native-vs-OCCT gate →
byte-freeze proof → zero-regression proof → docs, OR HONEST DECLINE at the sharpest
reachable level. All new native code stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::boolean`. No `cc_*` ABI change. The
change is strictly ADDITIVE: `splitFaceSmoothTrim`, B2 `splitFace`, `splitFaceJunction`,
`freeformHalfSpaceCut`, the recognisers, M0 `SolidMesher`, M1 `traceWallSeam`, and every
landed weld path stay BYTE-IDENTICAL. No tolerance is weakened; a correct decline is a
first-class outcome; no weld stub.

## STATUS

LANDED (BOTH gates). `curvedWallHalfSpaceCut` welds a steep Bézier bowl-cup cut by a
horizontal plane (CLOSED CIRCULAR seam, the real S3 trace) into the CUT (Below) and
COMMON (Above) keep sides. The CUT is robust — watertight (Euler χ=2) and monotone-
converging to the closed form `π·ρ²·c/2` across a resonance-free deflection sweep; COMMON
welds watertight at its robust deflection at `V(full)−V(z≤c)`. The COMMON annulus↔lid rim
weld is deflection-fragile and honestly DECLINES to NULL away from the robust band — the
sharpened next blocker. Host gate 8/8, sim native-vs-OCCT gate 37/37.

## 1. Baseline + fixture

- [x] 1.1 Capture the byte-frozen substrate baseline: `test_native_smooth_trim_split`,
  `test_native_face_split`, `test_native_first_freeform_boolean`,
  `test_native_multi_seam`, `test_native_two_operand_freeform_boolean` GREEN on the
  branch base; record they stay GREEN after this change.
- [x] 1.2 Add `tests/native/curved_wall_cut_fixture.h`: a steep (a=2.0) degree-2 Bézier
  bowl trimmed by a rim CIRCLE (radius R in (u,v)) + a flat top-lid disk on z=a·R², the
  horizontal cutter plane z=c (c=a·ρ², ρ<R), the real `closedSeamWLine()`, and the
  closed-form oracles `fullVolume = π·a·R⁴/2`, `cutVolume = π·ρ²·c/2`, `commonVolume`.

## 2. The verb

- [x] 2.1 `src/native/boolean/curved_wall_cut.h` `curvedWallHalfSpaceCut` composes
  recognise[B1] → trace[M1] (byte-unchanged `traceWallSeam`) → `splitFaceSmoothTrim`[B2]
  → analytic wall split[B4 `cutAnalyticFace`] → flat circular cap synth → M0 weld.
- [x] 2.2 Mandatory self-verify: mesh (M0), require WATERTIGHT AND a positive enclosed
  volume; ANY decline → NULL (→ OCCT). Typed `CurvedWallCutDecline`.
- [x] 2.3 Keep the driver's cognitive complexity in the backend band by delegating the
  recognise/trace/split prologue, the keep-side pick, and the analytic-face collection
  to `cwcdetail::` helpers (driver 10, all helpers ≤ 9).

## 3. Host GATE (a) — analytic, no OCCT

- [x] 3.1 Operand B1-admits with exactly one freeform wall; seam is a closed circle
  interior to the rim trim.
- [x] 3.2 CUT (Below) welds watertight (χ=2) at the closed form and CONVERGES
  monotonically across a resonance-free deflection sweep.
- [x] 3.3 COMMON (Above) welds watertight (χ=2) at `V(full)−V(z≤c)` at its robust
  deflection; the closed-form partition identity is exact.
- [x] 3.4 The COMMON rim-weld fragility is a MEASURED decline (NULL, `NotWatertight`).
- [x] 3.5 Non-cutting plane + non-operand DECLINE to NULL.

## 4. Sim GATE (b) — native-vs-OCCT

- [x] 4.1 `tests/sim/native_curved_wall_cut_parity.mm` + run script
  `scripts/run-sim-native-curved-wall-cut.sh`; SKIP entry in `run-sim-suite.sh`.
- [x] 4.2 OCCT reconstructs the SAME bowl-cup operand (Geom_BezierSurface bowl + planar
  lid, sewn, outward-oriented) and cuts by `BRepAlgoAPI_Common` (keep-half box).
- [x] 4.3 CUT (Below, 3 deflections) + COMMON (Above, robust deflection) match OCCT on
  volume / area / watertight / Euler χ=2 / bbox / one-sided Hausdorff, with fixed bands.
- [x] 4.4 The COMMON rim-weld fragility declines to NULL in the sim harness too.

## 5. Byte-freeze + zero-regression proof

- [x] 5.1 `git diff` on `smooth_trim_split.h` / `face_split.h` / `half_space_cut.h` /
  `two_operand.h` / `multi_face_weld.h` / `seam_graph.h` / `junction_split.h` /
  `freeform_operand.h` / the tessellator = EMPTY (byte-frozen).
- [x] 5.2 `src/native` has 0 new OCCT includes; 0 `cc_*` change.
- [x] 5.3 Full host `ctest` GREEN (57/57), including the new suite; the new header
  `-fsyntax-only` clean under the iossim toolchain.

## 6. Docs

- [x] 6.1 Update `openspec/MOAT-ROADMAP.md` M2 status with the landed curved-wall
  CUT/COMMON, both gate numbers, and the sharpened next blocker.
