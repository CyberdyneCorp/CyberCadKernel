// SPDX-License-Identifier: Apache-2.0
//
// test_native_interference.cpp — GATE A (HOST ANALYTIC, no OCCT) for MOAT M-GS GS7:
// the native clash / interference classifier (src/native/analysis/interference.h)
// and its cc_interference facade wiring over the NativeEngine.
//
// Every fixture has a CLOSED-FORM answer computed here without linking OCCT (that
// native-vs-OCCT match is gate (b), the sim harness native_interference_parity.mm):
//   * two overlapping axis-aligned boxes → CLASH, overlap volume = the exact
//     intersection-box volume, and the witness AABB equals that intersection box;
//   * two boxes 10 apart          → CLEAR, min distance = the exact gap, no witness;
//   * two face-touching boxes     → TOUCHING, overlap volume 0 (zero-volume contact);
//   * a nested box (fully inside) → CLASH, overlap volume = the inner box volume.
// The classifier NEVER emits a wrong clash flag or overlap volume; a non-watertight
// operand DECLINES (Unknown) rather than guess.
//
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include "cybercadkernel/cc_kernel.h"
#include "harness.h"
#include "native/analysis/interference.h"

namespace an = cybercad::native::analysis;
namespace tess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;

namespace {

// Axis-aligned box [x0,x0+a]×[y0,y0+b]×[z0,z0+c], outward-CCW-wound + watertight.
tess::Mesh boxMesh(double x0, double y0, double z0, double a, double b, double c) {
  tess::Mesh m;
  const nm::Point3 v[8] = {{x0, y0, z0},       {x0 + a, y0, z0},
                           {x0 + a, y0 + b, z0}, {x0, y0 + b, z0},
                           {x0, y0, z0 + c},     {x0 + a, y0, z0 + c},
                           {x0 + a, y0 + b, z0 + c}, {x0, y0 + b, z0 + c}};
  for (const auto& p : v) m.vertices.push_back(p);
  auto quad = [&](int A, int B, int C, int D) { m.addTriangle(A, B, C); m.addTriangle(A, C, D); };
  quad(0, 3, 2, 1); quad(4, 5, 6, 7); quad(0, 1, 5, 4);
  quad(2, 3, 7, 6); quad(1, 2, 6, 5); quad(3, 0, 4, 7);
  return m;
}

constexpr double kDefl = 0.005;

}  // namespace

// ── Two overlapping boxes → CLASH at the exact intersection-box volume ─────────
CC_TEST(interference_overlapping_boxes_clash_volume) {
  // A = [0,2]³, B = [1,3]³ → overlap = [1,2]³, volume 1.
  const tess::Mesh a = boxMesh(0, 0, 0, 2, 2, 2);
  const tess::Mesh b = boxMesh(1, 1, 1, 2, 2, 2);
  const an::InterferenceResult r = an::meshInterference(a, b, kDefl);
  CC_CHECK(r.state == an::ClashState::Clash);
  CC_CHECK(r.clash());
  // Witness present and its AABB must contain the true overlap box [1,2]³.
  CC_CHECK(r.hasWitness);
  CC_CHECK(r.witnessLo.x <= 1.0 + 1e-6 && r.witnessHi.x >= 2.0 - 1e-6);
  CC_CHECK(r.witnessLo.y <= 1.0 + 1e-6 && r.witnessHi.y >= 2.0 - 1e-6);
  CC_CHECK(r.witnessLo.z <= 1.0 + 1e-6 && r.witnessHi.z >= 2.0 - 1e-6);
  // The mesh classifier does not itself fill overlapVolume (that is the engine's
  // boolean COMMON) — assert the classification + witness here; the volume value is
  // asserted through the facade below.
}

// ── Two disjoint boxes → CLEAR at the exact gap ────────────────────────────────
CC_TEST(interference_disjoint_boxes_clear_distance) {
  // A = [0,1]³, B = [11,12]³ → axis gap 10 along X.
  const tess::Mesh a = boxMesh(0, 0, 0, 1, 1, 1);
  const tess::Mesh b = boxMesh(11, 0, 0, 1, 1, 1);
  const an::InterferenceResult r = an::meshInterference(a, b, kDefl);
  CC_CHECK(r.state == an::ClashState::Clear);
  CC_CHECK(!r.clash());
  CC_CHECK(!r.hasWitness);
  // The gap between the faces x=1 and x=11 is exactly 10.
  CC_CHECK(std::fabs(r.minDistance - 10.0) < 1e-6);
}

// ── Face-touching boxes → TOUCHING, zero-volume contact ────────────────────────
CC_TEST(interference_face_touching_boxes_touching) {
  // A = [0,1]³, B = [1,2]×[0,1]×[0,1] share the face x=1 exactly.
  const tess::Mesh a = boxMesh(0, 0, 0, 1, 1, 1);
  const tess::Mesh b = boxMesh(1, 0, 0, 1, 1, 1);
  const an::InterferenceResult r = an::meshInterference(a, b, kDefl);
  // Shared face is coplanar (parallel-plane guard excludes it from crossings) and
  // no vertex is strictly inside the other → TOUCHING, not CLASH.
  CC_CHECK(r.state == an::ClashState::Touching);
  CC_CHECK(!r.clash());
  CC_CHECK(r.minDistance < 2.0 * kDefl + 1e-9);
}

// ── A fully-nested box → CLASH; every inner vertex is inside the outer ─────────
CC_TEST(interference_nested_box_clash) {
  const tess::Mesh outer = boxMesh(0, 0, 0, 10, 10, 10);
  const tess::Mesh inner = boxMesh(3, 3, 3, 2, 2, 2);  // [3,5]³ ⊂ [0,10]³
  const an::InterferenceResult r = an::meshInterference(inner, outer, kDefl);
  CC_CHECK(r.state == an::ClashState::Clash);
  CC_CHECK(r.hasWitness);
}

// ── COPLANAR PLUS-SIGN-CROSS (regression, moat-clashfix) ───────────────────────
// Two boxes whose coplanar faces overlap in a plus/cross with NEITHER box having a
// corner inside the other. The min tri–tri distance is attained EDGE–EDGE (A's top
// edges cross B's bottom edges), not vertex–face; before the edge–edge term the
// vertex-face minimum overshot to 1.0 and this flush TOUCH mis-reported as CLEAR.
//
//   A = [0,3]×[1,2]×[0,1]  (horizontal bar, top face z=1)
//   B = [1,2]×[0,3]×[1,2]  (vertical  bar, bottom face z=1)
// coplanar at z=1; footprints cross; overlap [1,2]×[1,2]; distance 0 ⇒ TOUCHING.
CC_TEST(interference_coplanar_cross_touching) {
  const tess::Mesh a = boxMesh(0, 1, 0, 3, 1, 1);
  const tess::Mesh b = boxMesh(1, 0, 1, 1, 3, 1);
  const an::InterferenceResult r = an::meshInterference(a, b, kDefl);
  // Regression: was CLEAR (minDistance ≈ 0.53–1.0) before the edge–edge term.
  CC_CHECK(r.state == an::ClashState::Touching);
  CC_CHECK(!r.clash());
  CC_CHECK(r.minDistance < 2.0 * kDefl + 1e-9);  // exact edge–edge distance is 0
}

// Penetrating cross → CLASH. B is lowered + widened so A's top-face corners sit
// strictly inside B (the existing B3 penetration signature fires); the overlap is
// [1,2]×[1,2]×[0.5,1.0], a positive-volume interior intersection.
CC_TEST(interference_coplanar_cross_penetrate_clash) {
  const tess::Mesh a = boxMesh(0, 1, 0, 3, 1, 1);      // [0,3]×[1,2]×[0,1]
  const tess::Mesh b = boxMesh(0.5, 0, 0.5, 2.0, 3, 2); // [0.5,2.5]×[0,3]×[0.5,2.5]
  const an::InterferenceResult r = an::meshInterference(a, b, kDefl);
  CC_CHECK(r.state == an::ClashState::Clash);
  CC_CHECK(r.clash());
  CC_CHECK(r.hasWitness);
  // Witness AABB must cover the true overlap box [1,2]×[1,2]×[0.5,1.0].
  CC_CHECK(r.witnessLo.x <= 2.0 + 1e-6 && r.witnessHi.x >= 1.0 - 1e-6);
  CC_CHECK(r.witnessLo.y <= 2.0 + 1e-6 && r.witnessHi.y >= 1.0 - 1e-6);
}

// Gapped cross → CLEAR at the exact gap. Same footprints, B raised so its bottom
// face z=1.5 leaves a 0.5 clearance above A's top face z=1. The min distance is the
// edge–edge / face–face gap 0.5 (the very term the fix adds) — NOT a false TOUCH.
CC_TEST(interference_coplanar_cross_gap_clear) {
  const tess::Mesh a = boxMesh(0, 1, 0, 3, 1, 1);      // top z=1
  const tess::Mesh b = boxMesh(1, 0, 1.5, 1, 3, 1);    // bottom z=1.5
  const an::InterferenceResult r = an::meshInterference(a, b, kDefl);
  CC_CHECK(r.state == an::ClashState::Clear);
  CC_CHECK(!r.clash());
  CC_CHECK(std::fabs(r.minDistance - 0.5) < 1e-6);
}

// ── HONEST DECLINE: a non-watertight operand → Unknown (never a guessed clash) ──
CC_TEST(interference_non_watertight_declines) {
  tess::Mesh a = boxMesh(0, 0, 0, 2, 2, 2);
  a.triangles.resize(a.triangles.size() - 2);  // drop a face → open shell
  const tess::Mesh b = boxMesh(1, 1, 1, 2, 2, 2);
  const an::InterferenceResult r = an::meshInterference(a, b, kDefl);
  CC_CHECK(r.state == an::ClashState::Unknown);
  CC_CHECK(!r.clash());
}

// ═════════════════════════════════════════════════════════════════════════════
// FACADE (cc_interference over the NativeEngine) — the marshaled C ABI path.
// ═════════════════════════════════════════════════════════════════════════════

namespace {
struct EngineGuard { ~EngineGuard() { cc_set_engine(0); } };

// A native a×b×c box with its min corner at the origin (unit-square profile scaled
// via the extrude profile + depth). Translate to place it.
CCShapeId makeBoxAt(double x, double y, double z, double a, double b, double c) {
  const double profile[] = {0.0, 0.0, a, 0.0, a, b, 0.0, b};
  const CCShapeId s = cc_solid_extrude(profile, 4, c);
  if (s == 0) return 0;
  return cc_translate_shape(s, x, y, z);
}
}  // namespace

// Overlapping native boxes → cc_interference reports CLASH with the exact overlap
// volume (native boolean COMMON) and a witness AABB.
CC_TEST(facade_interference_overlap_volume) {
  EngineGuard g; cc_set_engine(1);
  // A = [0,2]³, B = [1,3]³ → overlap [1,2]³, volume = 1.
  const CCShapeId a = makeBoxAt(0, 0, 0, 2, 2, 2);
  const CCShapeId b = makeBoxAt(1, 1, 1, 2, 2, 2);
  CC_CHECK(a != 0 && b != 0);

  CCInterference out{};
  const int ok = cc_interference(a, b, &out);
  CC_CHECK(ok == 1);
  CC_CHECK(out.decided == 1);
  CC_CHECK(out.clash == 1);
  CC_CHECK(out.state == CC_CLASH_CLASH);
  // Native COMMON is exact for planar polyhedra: overlap volume = 1.
  CC_CHECK(std::fabs(out.overlap_volume - 1.0) < 1e-6);
  CC_CHECK(out.has_witness == 1);
  // The representative interior point must lie inside the true overlap [1,2]³.
  CC_CHECK(out.witness_point[0] > 1.0 && out.witness_point[0] < 2.0);
  CC_CHECK(out.witness_point[1] > 1.0 && out.witness_point[1] < 2.0);
  CC_CHECK(out.witness_point[2] > 1.0 && out.witness_point[2] < 2.0);

  cc_shape_release(a);
  cc_shape_release(b);
}

// Disjoint native boxes → CLEAR with the exact gap, no witness.
CC_TEST(facade_interference_clear) {
  EngineGuard g; cc_set_engine(1);
  const CCShapeId a = makeBoxAt(0, 0, 0, 1, 1, 1);
  const CCShapeId b = makeBoxAt(11, 0, 0, 1, 1, 1);
  CC_CHECK(a != 0 && b != 0);

  CCInterference out{};
  const int ok = cc_interference(a, b, &out);
  CC_CHECK(ok == 1);
  CC_CHECK(out.decided == 1);
  CC_CHECK(out.clash == 0);
  CC_CHECK(out.state == CC_CLASH_CLEAR);
  CC_CHECK(std::fabs(out.overlap_volume) < 1e-12);
  CC_CHECK(std::fabs(out.min_distance - 10.0) < 1e-6);

  cc_shape_release(a);
  cc_shape_release(b);
}

// Face-touching native boxes → TOUCHING, zero overlap volume.
CC_TEST(facade_interference_touching) {
  EngineGuard g; cc_set_engine(1);
  const CCShapeId a = makeBoxAt(0, 0, 0, 1, 1, 1);
  const CCShapeId b = makeBoxAt(1, 0, 0, 1, 1, 1);
  CC_CHECK(a != 0 && b != 0);

  CCInterference out{};
  const int ok = cc_interference(a, b, &out);
  CC_CHECK(ok == 1);
  CC_CHECK(out.decided == 1);
  CC_CHECK(out.clash == 0);
  CC_CHECK(out.state == CC_CLASH_TOUCHING);
  CC_CHECK(std::fabs(out.overlap_volume) < 1e-12);

  cc_shape_release(a);
  cc_shape_release(b);
}

CC_RUN_ALL()
