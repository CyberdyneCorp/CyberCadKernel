# Tasks — moat-m2cf-chamfer-face (MOAT M2 full-face chamfer weld)

Order: diagnose the missing chamfer twin of `fillet_face` → full-face verb assembling
the landed corner weld → convex-edge fit guard reuse → host analytic gate (closed-form
volume + declines) → sim native-vs-OCCT gate → byte-freeze + zero-regression proof →
OpenSpec. All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`),
namespace `cybercad::native::blend`. No `cc_*` ABI change, no engine-glue change. The
change is strictly ADDITIVE: `chamfer_edges`, `corner_chamfer_weld`, `fillet_edges`,
`fillet_face`, `full_round`, the M0 tessellator, and every landed weld path stay
BYTE-IDENTICAL. No tolerance is weakened; a correct decline is a first-class outcome.

## STATUS

LANDED (both gates). `chamfer_face` chamfers every convex planar-dihedral edge bounding
a picked planar face by assembling the landed `chamfer_corner` over the face's
bounding-edge loop. On a box a face's loop is four 2-edge DIHEDRAL corners (never a
triple), so it welds watertight AND matches OCCT `BRepFilletAPI_MakeChamfer` to fp64. No
`cc_*` ABI or engine-glue change (the verb is arbitrated at the native-blend layer by the
two gates).

## 1. Diagnose the gap
- [x] Confirm `fillet_face` exists but has no chamfer twin, because the sequential
      `chamfer_edges` declines a corner-sharing face-edge loop; and that a single face's
      loop is only 2-edge dihedral corners (never a triple), so `chamfer_corner` welds it
      exactly.

## 2. Full-face verb (additive)
- [x] `chamfer_face.h` `chamfer_face`: guard all-planar solid + planar face; collect the
      convex planar-dihedral bounding edges via the SAME `detail::filletArc` fit guard
      `fillet_face` uses; call byte-frozen `chamfer_corner` and return its result.
- [x] Typed measured declines: BadInput / NonPlanarSolid / NonPlanarFace / NoConvexEdges
      / WeldFailed. Consumes `blend_geom.h`, `fillet_edges.h`, `corner_chamfer_weld.h`
      byte-identical.
- [x] `native_blend.h` includes the new header.

## 3. Gate A — host analytic (no OCCT)
- [x] `tests/native/test_native_chamfer_face.cpp` (5/5): top face exact
      (`L³ − (2d²L − 4d³/3)` = 959.5), all six cube faces exact (981.333 at d=1), setback
      sweep, non-orthogonal triangular-prism top face (watertight + shrink), and the
      out-of-domain declines. Registered in CMakeLists (always-on native suite).

## 4. Gate B — sim native-vs-OCCT
- [x] `tests/sim/native_chamfer_face_parity.mm`: for every cube face and a setback sweep,
      native `chamfer_face` volume == OCCT `BRepFilletAPI_MakeChamfer` (all face edges) ==
      closed form to fp64, native watertight; plus the oversized honest decline. 11
      passed / 0 failed on the booted simulator.
- [x] `scripts/run-sim-native-chamfer-face.sh` (models run-sim-native-blend.sh; links the
      whole kernel + OCCT; native headers on the `-I src` path).

## 5. Discipline + docs
- [x] Byte-freeze proof: `git diff` shows `src/native/tessellate/**`, `chamfer_edges.h`,
      `corner_chamfer_weld.h`, `fillet_edges.h`, `fillet_face.h` byte-identical vs HEAD;
      the new header is OCCT-free; no `cc_*` ABI or engine change.
- [x] Zero-regression: the native blend / corner-chamfer / analytic-fillet host suites
      pass unchanged.
- [x] OpenSpec change validated `--strict`.
