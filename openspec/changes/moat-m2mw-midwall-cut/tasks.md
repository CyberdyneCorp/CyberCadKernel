# Tasks — moat-m2mw-midwall-cut (MOAT M2 walled-bowl MID-WALL freeform CUT)

Order: baseline capture → steep walled-bowl fixture + closed-form oracle → annular-cap
enabler in `curvedWallHalfSpaceCut` → host analytic gate → sim native-vs-OCCT gate →
byte-freeze + zero-regression proof → docs. All new native code stays OCCT-free and
host-buildable (`clang++ -std=c++20`), namespace `cybercad::native::boolean`. No `cc_*`
ABI change. Strictly ADDITIVE: the dome (disk-cap) path stays behaviourally identical;
`smooth_trim_split.h`, B2 `splitFace`, `half_space_cut.h` (`cutAnalyticFace`/`orderLoop`/
`loopSimple`/`edgeFromPiece`), `freeform_operand.h`, M0/M1 stay BYTE-IDENTICAL. No
tolerance weakened; a correct decline is first-class.

## STATUS

LANDED (BOTH gates). `curvedWallHalfSpaceCut` now welds the MID-WALL pose — a steep
Bézier bowl over a convex quad + 4 planar walls + a flat base cut by a horizontal plane
whose seam is a closed interior circle AND which splits the 4 walls — into the CUT
(Below) keep side with an ANNULAR cross-section cap (outer = wall-section polygon, inner
HOLE = the seam). Watertight (Euler χ=2), 7-face topology, monotone-converging to the
closed form `(H0+c)·A_Q − c·π·ρ²/2`. Host gate 12/12; sim native-vs-OCCT gate 28/28.

## 1. Baseline + fixture

- [x] 1.1 Capture the byte-frozen substrate baseline: `test_native_curved_wall_cut`
  (8/8 dome), `test_native_smooth_trim_split`, `test_native_face_split`,
  `test_native_first_freeform_boolean`, `test_native_tessellate` GREEN before the change.
- [x] 1.2 Add `tests/native/walled_bowl_midwall_fixture.h`: a STEEP (a=2.0) degree-2
  Bézier bowl over the convex quad (reused from `face_split_fixture`) + 4 planar walls
  + a flat base (the `first_freeform_boolean_fixture` construction, shared vertices/edges
  ⇒ watertight, B1-admissible), the horizontal cutter plane `z = c` with c < a·d_e² (so
  the seam circle is interior AND each wall is split), and the closed-form oracles
  `fullVolume` (the exact bowl-prism polynomial), `cutVolume = (H0+c)·A_Q − c·π·ρ²/2`.

## 2. The annular-cap enabler

- [x] 2.1 `collectKeptAnalyticFaces` records each SPLIT wall's `Face ∩ P` chord
  (`AnalyticCut::cross0/cross1`) into a `capChords` list; the dome pose leaves it empty.
- [x] 2.2 `synthAnnularCap` chains `capChords` into the OUTER loop (`orderLoop` +
  `loopSimple`), builds the freeform seam as the HOLE wire (opposite winding), pins the
  cap orientation off the plane `frame.z` so its normal faces the DISCARD side
  deterministically (winding-independent), and returns a planar face-with-hole.
- [x] 2.3 The driver branches: `capChords.empty()` → the byte-identical single-disk
  `synthCircularCap`; else → `synthAnnularCap`. A null annular cap (chords do not chain
  simple-closed) → `AnalyticCutFailed`. The mandatory M0 watertight + positive-volume
  self-verify is unchanged.
- [x] 2.4 Keep the driver + helpers within the backend cognitive-complexity band.

## 3. Host GATE (a) — analytic, no OCCT

- [x] 3.1 The operand B1-admits with one freeform bowl + 5 analytic faces (4 walls +
  base); the cut height is strictly below the interior-circle bound (seam interior).
- [x] 3.2 The MID-WALL CUT (Below) yields a 7-face solid (disk + 4 wall trapezoids +
  base + ANNULAR cap) that welds watertight (χ=2) and CONVERGES monotonically to the
  closed form across a deflection sweep.
- [x] 3.3 The removed cap volume matches the bowl-cap closed form (a SUBSTANTIAL,
  discriminating cut — asserts the −c·π·ρ²/2 annular-hole term).
- [x] 3.4 A non-cutting plane DECLINES to NULL.
- [x] 3.5 The 8 landed dome cases stay GREEN (disk-cap path byte-behaviour preserved).

## 4. Sim GATE (b) — native-vs-OCCT

- [x] 4.1 `tests/sim/native_walled_bowl_midwall_parity.mm` + run script
  `scripts/run-sim-native-walled-bowl-midwall.sh`.
- [x] 4.2 OCCT reconstructs the SAME operand (Geom_BezierSurface bowl + 4 planar walls +
  base, sewn, outward-oriented) and cuts by `BRepAlgoAPI_Cut` (keep-half box, z≤c).
- [x] 4.3 CUT (Below, 3 deflections) matches OCCT on volume / area / watertight / Euler
  χ=2 / annular 7-face topology / bbox / one-sided Hausdorff, with fixed bands; OCCT cut
  is cross-checked against the closed form (rel 1.3e-07).

## 5. Byte-freeze + zero-regression proof

- [x] 5.1 `include/` diff EMPTY (no `cc_*` change); only `curved_wall_cut.h` changes in
  `src/native`; `smooth_trim_split.h` / `face_split.h` / `half_space_cut.h` /
  `freeform_operand.h` / the tessellator BYTE-IDENTICAL.
- [x] 5.2 `src/native` has 0 new OCCT includes.
- [x] 5.3 Boolean + tessellate host suites GREEN with zero regression
  (`test_native_curved_wall_cut` 12/12, `test_native_tessellate` 17/17,
  `smooth_trim_split`/`face_split`/`first_freeform_boolean`/`two_operand`/`multi_seam`/
  `freeform_operand`/`ssi_boolean`/`curved_boolean`/`boolean` all GREEN).

## 6. Docs

- [x] 6.1 Update `openspec/MOAT-ROADMAP.md` M2 status with the landed MID-WALL CUT, both
  gate numbers, and the sharpened next blocker.
