# Tasks — nurbs-variable-two-rail-sweep

## 1. Variable-section sweep (scale + twist along the spine)
- [x] 1.1 Declare `sweepVariable` / `sweepRationalVariable` in `bspline_sweep.h` (section, trajectory,
      section normal, sampled `scales[]` + `twists[]`, stations, V degree).
- [x] 1.2 Implement a shared `placeSectionsVariable` helper: RMF-transport the section to each
      station, compose an in-plane scale (about the origin) + twist (about the section normal) BEFORE
      the rigid frame — a similarity that preserves weights exactly.
- [x] 1.3 Non-rational `sweepVariable` skins the placed sections; rational `sweepRationalVariable`
      rational-skins them. Honest guards: `<2` stations, wrong-size / non-positive scale field,
      rational input on the non-rational routine (and vice-versa), degenerate trajectory.

## 2. Two-rail sweep (section anchored between two rails)
- [x] 2.1 Declare `sweepTwoRail` / `sweepRationalTwoRail` in `bspline_sweep.h` (section, rail0, rail1,
      section normal, anchor0/anchor1 pole indices, stations, V degree).
- [x] 2.2 Implement a shared `placeSectionsTwoRail` helper: per station scale = rail chord / anchor
      chord, orient the anchor chord onto the rail chord with an RMF along the rail-midpoint spine
      (anti-twist), translate so anchor0 rides rail0(t). Similarity ⇒ weights preserved.
- [x] 2.3 Non-rational `sweepTwoRail` skins; rational `sweepRationalTwoRail` rational-skins. Honest
      guards: `<2` stations, out-of-range / equal anchors, coincident section anchors, DEGENERATE
      rails (zero rail chord at a station — crossing / coincident rails), degenerate midpoint spine,
      rational input on the non-rational routine (and rational rails).

## 3. Host-analytic gate — airtight oracles
- [x] 3.1 `tests/native/test_native_nurbs_vsweep.cpp`, wired into `CMakeLists.txt` (CYBERCAD_TESTS
      list + `_SRC` map + `CYBERCAD_HAS_NUMSCI` compile-definitions block), mirroring the base gate.
- [x] 3.2 Constant-section degenerate: scales≡1, twists≡0 reproduces `sweepAlongTrajectory` and
      `sweepRationalAlongTrajectory` pointwise (≤1e-12, achieved 0.0 / machine precision).
- [x] 3.3 Linear-taper: rational circle + straight spine + linear scale = EXACT rational cone
      frustum (surface contains each scaled circle; every station iso a true circle at radius
      `s(t)·R`, ≤1e-9, achieved ~1e-15). Non-rational taper + pure-twist containment (≤1e-8).
- [x] 3.4 Two-rail: parallel rails → planar ruled strip matches closed-form; diverging rails → exact
      linear taper; section anchor isos lie ON both rails at every station (≤1e-9, achieved ~1e-16).
      Rational two-rail anchors ride the rails (≤1e-8).
- [x] 3.5 Degenerate guards decline honestly (no crash): crossing / coincident rails, coincident
      anchors, bad anchor indices, wrong-size / non-positive scale fields, rational/non-rational
      routing.

## 4. Close-out
- [x] 4.1 Build `build-numsci/host`, configure the kernel with `-DCYBERCAD_HAS_NUMSCI=ON`, build the
      new test target, run it (45/45 checks pass), and confirm zero regression across the native
      math / NURBS surfacing test group (24/24).
- [x] 4.2 `openspec validate --strict` passes; `cc_*` ABI byte-unchanged (no ABI header touched);
      `src/native` stays OCCT-free; base sweep API unchanged (additive only).
- [ ] 4.3 OPTIONAL follow-up: SIM native-vs-OCCT parity (`BRepOffsetAPI_MakePipeShell` with a law
      / two guide rails). HOST is primary and sufficient; separate track.
