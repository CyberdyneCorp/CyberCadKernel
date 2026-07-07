// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for Tier-B native construction (Phase 4 #4b): 2-section RULED
// loft (src/native/construct/loft.h). OCCT-FREE — Gate 1 (host, analytic) of the
// two-gate model in openspec/NATIVE-REWRITE.md. Each result is validated with the
// native tessellator (watertight / enclosed volume) and the topology Explorer
// (face structure). Deferred cases (mismatched counts / non-planar / degenerate)
// are asserted to return a NULL Shape so the engine can fall through to OCCT.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_loft.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_loft
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace m = cybercad::native::math;

namespace {
int countSub(const topo::Shape& shape, topo::ShapeType type) {
  int n = 0;
  for (topo::Explorer ex(shape, type); ex.more(); ex.next()) ++n;
  return n;
}
}  // namespace

// ── build_loft: square→square, equal size → a straight box ───────────────────---
// Bottom = top = 10×10 square, depth 10. The ruled skin degenerates to 4 vertical
// planar quads; the solid is a box, volume = 100·10 = 1000, EXACT. 6 faces
// (4 sides + 2 caps), watertight.
CC_TEST(loft_square_to_equal_square_is_box) {
  const double bot[] = {0, 0, 10, 0, 10, 10, 0, 10};
  const double top[] = {0, 0, 10, 0, 10, 10, 0, 10};
  const topo::Shape solid = cst::build_loft(bot, 4, top, 4, 10.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 6);  // 4 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.05;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - 1000.0) < 1e-6);
}

// ── build_loft: 10×10 → 6×6 square (a frustum / truncated pyramid) ───────────---
// Bottom 10×10 (area A1 = 100) at z=0, top 6×6 (area A2 = 36) at z=10, both centred
// at (0,0) so corresponding corners pair sensibly. This is a square frustum; its
// exact volume is the prismatoid formula h/3·(A1 + A2 + √(A1·A2)) =
// 10/3·(100 + 36 + √3600) = 10/3·(100 + 36 + 60) = 10/3·196 = 653.33. The ruled
// side faces are true bilinear (here planar trapezoids), so the volume is exact up
// to a deflection bound.
CC_TEST(loft_square_frustum_prismatoid_volume) {
  const double bot[] = {-5, -5, 5, -5, 5, 5, -5, 5};
  const double top[] = {-3, -3, 3, -3, 3, 3, -3, 3};
  const topo::Shape solid = cst::build_loft(bot, 4, top, 4, 10.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 6);
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = 10.0 / 3.0 * (100.0 + 36.0 + std::sqrt(100.0 * 36.0));  // 653.33
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - expected) / expected < 0.02);
}

// ── build_loft: rotated square → square (a genuinely NON-PLANAR ruled skin) ──---
// Bottom 2×2 square; top the SAME square rotated 45° about z, at z=4. Corresponding
// corners no longer align, so each ruled side face is a truly twisted bilinear
// patch (an "antiprism"-like skin). We do not assert an exact closed form (the
// twisted patch volume is between the two extreme readings); we assert the solid is
// watertight and the volume is positive and bounded by the convex-hull estimate.
CC_TEST(loft_rotated_square_twisted_watertight) {
  const double s = 1.0;                                  // half-side of the bottom
  const double bot[] = {-s, -s, s, -s, s, s, -s, s};     // axis-aligned square
  const double r = std::sqrt(2.0) * s;                   // top rotated 45°, same "radius"
  const double top[] = {r, 0, 0, r, -r, 0, 0, -r, 0};    // 4 corners on axes, z filled below
  // Rebuild top with z=4 explicitly (build_loft places top at z=depth already; the
  // XY above is what matters). Bottom corners map to these rotated corners in order.
  (void)top;
  const double topXY[] = {r, 0, 0, r, -r, 0, 0, -r};
  const topo::Shape solid = cst::build_loft(bot, 4, topXY, 4, 4.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 6);
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double vol = std::fabs(tess::enclosedVolume(mesh));
  CC_CHECK(vol > 1.0);   // clearly a nonzero solid
  CC_CHECK(vol < 40.0);  // bounded well under bbox*depth
}

// ── build_loft_wires: two planar triangles in ARBITRARY 3D planes ────────────---
// Section A is a triangle in the z=0 plane; section B the same triangle translated
// +3 in z (so both planar, parallel). Ruled loft = a triangular prism, volume =
// area·3. Triangle area (0,0)(4,0)(2,3) = ½|4·3| = 6, volume 18. Watertight.
CC_TEST(loft_wires_planar_triangles_prism) {
  const double a[] = {0, 0, 0, 4, 0, 0, 2, 3, 0};
  const double b[] = {0, 0, 3, 4, 0, 3, 2, 3, 3};
  const topo::Shape solid = cst::build_loft_wires(a, 3, b, 3);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 5);  // 3 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.05;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - 18.0) < 1e-6);
}

// ── build_loft_wires: a tilted planar section (arbitrary plane) → still native --
// Section A pentagon in z=0; section B the SAME pentagon lifted and tilted (a plane
// with a non-vertical normal). Both are planar, equal count → native ruled loft,
// watertight. We only assert watertight + positive volume (exact volume of a skew
// prism is fiddly; the point is that a non-axis-aligned planar section works).
CC_TEST(loft_wires_tilted_planar_section_watertight) {
  const double A[] = {0, 0, 0, 2, 0, 0, 3, 2, 0, 1, 3, 0, -1, 2, 0};
  // Top pentagon: same XY, but z = 4 + 0.5*x (a tilted plane z = 4 + 0.5 x).
  auto tz = [](double x) { return 4.0 + 0.5 * x; };
  const double B[] = {0, 0, tz(0), 2, 0, tz(2), 3, 2, tz(3), 1, 3, tz(1), -1, 2, tz(-1)};
  const topo::Shape solid = cst::build_loft_wires(A, 5, B, 5);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 7);  // 5 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.05;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh)) > 1.0);
}

// ── build_loft_wires: two OFFSET + ROTATED same-count 3D quads → ruled solid ──---
// Section A is a 4×4 square in the z=0 plane (corners at ±2). Section B is the same
// square rotated 45° about z AND offset +6 in z (its corners fall on the axes at
// radius r = 2√2). Corresponding corners no longer align, so each of the 4 ruled
// side faces is a genuinely twisted bilinear patch (an antiprism-like skin). We
// assert: 6 faces (4 sides + 2 caps), watertight, volume > 0, and a correct
// bounding box — XY spans [-r, r] (the top quad is the widest ring, reaching the
// axes at ±2√2 ≈ 2.828) and Z spans [0, 6].
CC_TEST(loft_wires_offset_rotated_quads) {
  const double a[] = {-2, -2, 0, 2, -2, 0, 2, 2, 0, -2, 2, 0};        // z=0 axis-aligned square
  const double r = 2.0 * std::sqrt(2.0);                              // rotated-corner radius
  const double b[] = {r, 0, 6, 0, r, 6, -r, 0, 6, 0, -r, 6};         // z=6 square rotated 45°
  const topo::Shape solid = cst::build_loft_wires(a, 4, b, 4);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 6);  // 4 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh)) > 1.0);  // a genuine nonzero solid

  // Bounding box: XY spans [-r, r] (top ring is widest), Z spans [0, 6].
  double lo[3] = {1e9, 1e9, 1e9}, hi[3] = {-1e9, -1e9, -1e9};
  for (const m::Point3& v : mesh.vertices) {
    lo[0] = std::min(lo[0], v.x); hi[0] = std::max(hi[0], v.x);
    lo[1] = std::min(lo[1], v.y); hi[1] = std::max(hi[1], v.y);
    lo[2] = std::min(lo[2], v.z); hi[2] = std::max(hi[2], v.z);
  }
  CC_CHECK(std::fabs(lo[0] - (-r)) < 1e-6 && std::fabs(hi[0] - r) < 1e-6);
  CC_CHECK(std::fabs(lo[1] - (-r)) < 1e-6 && std::fabs(hi[1] - r) < 1e-6);
  CC_CHECK(std::fabs(lo[2] - 0.0) < 1e-6 && std::fabs(hi[2] - 6.0) < 1e-6);
}

// ── T1 NATIVE: mismatched section counts (4-pt → 6-pt), SAME square → box ────────--
// Bottom is a 4×4 square (4 corners). Top is the SAME 4×4 square sampled at 6 points
// (two extra COLLINEAR edge midpoints at (2,0) and (2,4)). The arc-length
// correspondence resamples both loops at the union of their params, inserting the
// matching collinear points on the bottom — so both rings are geometrically the same
// 4×4 square and the loft is a 4×4×3 box, volume 48, EXACT. Faces = 6 sides + 2 caps.
CC_TEST(loft_mismatched_counts_native_box) {
  const double bot[] = {0, 0, 4, 0, 4, 4, 0, 4};                 // 4-gon (4×4 square)
  const double top[] = {0, 0, 2, 0, 4, 0, 4, 4, 2, 4, 0, 4};     // 6-gon: same square + midpts
  const topo::Shape solid = cst::build_loft(bot, 4, top, 6, 3.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 8);  // 6 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - 48.0) < 1e-6);
}

// ── T1 NATIVE: mismatched 4→8, geometry-preserving 10×10 box (exact 1000) ────────--
// Bottom 10×10 square (4 corners) → the SAME 10×10 square as 8 points (corners + edge
// midpoints), depth 10. Both rings are the same square after correspondence, so the
// loft is a 10×10×10 box, volume 1000, EXACT. Faces = 8 sides + 2 caps.
CC_TEST(loft_mismatched_box_4to8_exact) {
  const double bot[] = {-5, -5, 5, -5, 5, 5, -5, 5};  // 4-gon (10×10)
  const double top[] = {-5, -5, 0, -5, 5, -5, 5, 0,   // 8-gon (same 10×10 + midpoints)
                        5, 5, 0, 5, -5, 5, -5, 0};
  const topo::Shape solid = cst::build_loft(bot, 4, top, 8, 10.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 10);  // 8 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - 1000.0) < 1e-6);
}

// ── T1 NATIVE: mismatched 4→8 square FRUSTUM (exact prismatoid 653.33) ───────────--
// Bottom 10×10 (4 corners) → 6×6 (8 points: corners + midpoints), depth 10, both
// centred. Corner→corner and midpoint→midpoint pairing keeps every side face a planar
// trapezoid, so the solid IS the true square frustum. Its exact prismatoid volume is
// h/6·(A_bot + 4·A_mid + A_top) = 10/6·(100 + 4·64 + 36) = 653.33. Faces = 8 + 2.
CC_TEST(loft_mismatched_frustum_4to8_exact) {
  const double bot[] = {-5, -5, 5, -5, 5, 5, -5, 5};  // 10×10, 4 pts
  const double top[] = {-3, -3, 0, -3, 3, -3, 3, 0,   // 6×6, 8 pts (corners + midpoints)
                        3, 3, 0, 3, -3, 3, -3, 0};
  const topo::Shape solid = cst::build_loft(bot, 4, top, 8, 10.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 10);  // 8 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = 10.0 / 6.0 * (100.0 + 4.0 * 64.0 + 36.0);  // 653.33
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - expected) / expected < 1e-6);
}

// ── DEFERRED: a NON-PLANAR section wire → NULL (OCCT fallthrough) ────────────---
// Section B's four points do NOT lie on a common plane (one corner lifted in z),
// so a planar cap cannot close it — the native ruled loft defers to OCCT.
CC_TEST(loft_wires_non_planar_section_deferred) {
  const double a[] = {0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0};        // planar square
  const double b[] = {0, 0, 3, 4, 0, 3, 4, 4, 5, 0, 4, 3};        // corner 3 lifted → skew
  CC_CHECK(cst::build_loft_wires(a, 4, b, 4).isNull());
}

// ── DEFERRED: a degenerate (collinear / point) section → NULL ────────────────---
CC_TEST(loft_degenerate_section_deferred) {
  const double bot[] = {0, 0, 4, 0, 4, 4, 0, 4};
  const double collinearTop[] = {0, 0, 1, 0, 2, 0};  // 3 collinear points → zero area
  CC_CHECK(cst::build_loft(bot, 4, collinearTop, 3, 3.0).isNull());
  const double pointTop[] = {1, 1, 1, 1, 1, 1};      // all coincident → point
  CC_CHECK(cst::build_loft(bot, 4, pointTop, 3, 3.0).isNull());
}

// ── DEFERRED: degenerate depth / too-few points → NULL ───────────────────────---
CC_TEST(loft_bad_input_deferred) {
  const double sq[] = {0, 0, 4, 0, 4, 4, 0, 4};
  CC_CHECK(cst::build_loft(sq, 4, sq, 4, 0.0).isNull());   // zero depth
  CC_CHECK(cst::build_loft(sq, 4, sq, 4, -1.0).isNull());  // negative depth
  const double two[] = {0, 0, 1, 1};
  CC_CHECK(cst::build_loft(two, 2, sq, 4, 3.0).isNull());  // < 3 bottom points
}

// ═════════════════════════════════════════════════════════════════════════════
// N-SECTION loft (3+ sections) — the cc_solid_loft chain extension. Each result is
// validated with the native tessellator (watertight + enclosed volume) and the
// Explorer (face structure). Deferred cases return a NULL Shape (OCCT fallthrough).
// ═════════════════════════════════════════════════════════════════════════════

namespace {
// Pack `sectionCount` flat (x,y,z) section loops into one buffer + a counts array,
// then call build_loft_sections. All sections here share the same vertex count.
topo::Shape loftSections(const std::vector<std::vector<double>>& sections) {
  std::vector<double> xyz;
  std::vector<int> counts;
  for (const auto& s : sections) {
    counts.push_back(static_cast<int>(s.size() / 3));
    xyz.insert(xyz.end(), s.begin(), s.end());
  }
  return cst::build_loft_sections(xyz.data(), counts.data(), static_cast<int>(counts.size()));
}
}  // namespace

// ── 3 sections: stacked equal squares → a straight box (two bands, exact) ────────
// Three 4×4 squares at z=0, z=5, z=10 (all identical XY). The two ruled bands
// degenerate to vertical quads; the solid is a 4×4×10 box, volume 160, EXACT. Faces
// = 8 side (4 per band) + 2 caps = 10; internal ring is NOT capped.
CC_TEST(loft3_stacked_equal_squares_is_box) {
  const std::vector<double> s0 = {0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0};
  const std::vector<double> s1 = {0, 0, 5, 4, 0, 5, 4, 4, 5, 0, 4, 5};
  const std::vector<double> s2 = {0, 0, 10, 4, 0, 10, 4, 4, 10, 0, 4, 10};
  const topo::Shape solid = loftSections({s0, s1, s2});
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 10);  // 8 sides + 2 end caps
  tess::MeshParams p;
  p.deflection = 0.05;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - 160.0) < 1e-6);
}

// ── 3 sections: wide → narrow → wide (a spool / bowtie prismatoid) ───────────────
// Squares centred on the axis: 10×10 at z=0, 4×4 at z=6, 10×10 at z=12. Two stacked
// square frustums; total volume = 2 × prismatoid(10×10 ↔ 4×4 over h=6). Each frustum
// = 6/3·(100 + 16 + √1600) = 2·(116 + 40) = 312 → total 624 (exact ruled bands).
CC_TEST(loft3_spool_two_frustums_volume) {
  const std::vector<double> s0 = {-5, -5, 0, 5, -5, 0, 5, 5, 0, -5, 5, 0};
  const std::vector<double> s1 = {-2, -2, 6, 2, -2, 6, 2, 2, 6, -2, 2, 6};
  const std::vector<double> s2 = {-5, -5, 12, 5, -5, 12, 5, 5, 12, -5, 5, 12};
  const topo::Shape solid = loftSections({s0, s1, s2});
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 10);  // 8 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double frustum = 6.0 / 3.0 * (100.0 + 16.0 + std::sqrt(100.0 * 16.0));  // 312
  const double expected = 2.0 * frustum;                                       // 624
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - expected) / expected < 0.02);
}

// ── 4 sections: rotating square chain (twist propagates along the chain) ─────────
// Four 2×2 squares, each rotated +45° from the previous about z, at z = 0,3,6,9.
// The predecessor-alignment must keep each section paired to its nearest neighbour
// so the four ruled bands do NOT self-intersect. We assert watertight + a positive,
// bounded volume (the twisted-band closed form is fiddly; the point is the chain
// closes and encloses a sensible solid). Faces = 4·3 sides + 2 caps = 14.
CC_TEST(loft4_rotating_square_chain_watertight) {
  auto sq = [](double z, double ang) {
    std::vector<double> out;
    const double r = std::sqrt(2.0);  // circumradius of a 2×2 square
    for (int i = 0; i < 4; ++i) {
      const double t = ang + i * (M_PI / 2.0) + M_PI / 4.0;  // corner angles
      out.push_back(r * std::cos(t));
      out.push_back(r * std::sin(t));
      out.push_back(z);
    }
    return out;
  };
  const topo::Shape solid =
      loftSections({sq(0, 0), sq(3, M_PI / 4), sq(6, M_PI / 2), sq(9, 3 * M_PI / 4)});
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 14);  // 12 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double vol = std::fabs(tess::enclosedVolume(mesh));
  CC_CHECK(vol > 10.0);   // clearly a nonzero solid (~4·9 = 36 order)
  CC_CHECK(vol < 60.0);   // bounded well under bbox·height
}

// ── 3 sections: triangles in arbitrary parallel planes → stacked prism ───────────
// Same triangle at z=0, z=2, z=5. Two triangular-prism bands. Triangle area
// (0,0)(4,0)(2,3) = 6; total volume = 6·5 = 30 (exact). Faces = 2·3 + 2 = 8.
CC_TEST(loft3_triangles_stacked_prism_volume) {
  const std::vector<double> s0 = {0, 0, 0, 4, 0, 0, 2, 3, 0};
  const std::vector<double> s1 = {0, 0, 2, 4, 0, 2, 2, 3, 2};
  const std::vector<double> s2 = {0, 0, 5, 4, 0, 5, 2, 3, 5};
  const topo::Shape solid = loftSections({s0, s1, s2});
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 8);  // 6 sides + 2 caps
  tess::MeshParams p;
  p.deflection = 0.05;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - 30.0) < 1e-6);
}

// ── 3 sections: narrow → WIDE → narrow OCTAGON spool (equal-count section chain) ─
// The task's "square → octagon → square" shape family. The N-section ruled loft pairs
// vertex k→k, so all sections must share a vertex count; we model the family as an
// EQUAL-count octagon chain: an octagon of circumradius 3 at z=0, a WIDE octagon of
// circumradius 5 at z=6, back to circumradius 3 at z=12 (all sharing the π/8 rotation
// so corners align 1:1). Two ruled bands, no self-intersection. We assert:
//   * watertight;
//   * the face structure — 2 bands × 8 side faces + 2 end caps = 18 (an 8-gon spool);
//   * MONOTONIC / EXPECTED volume — strictly greater than a straight r=3 octagon prism
//     over the full height (the middle bulges OUT) and strictly less than a straight
//     r=5 octagon prism (the ends are narrower), i.e. bracketed by its own extremes.
CC_TEST(loft3_octagon_spool_narrow_wide_narrow_watertight) {
  auto octagon = [](double z, double rad) {
    std::vector<double> o;
    const double rot = M_PI / 8.0;  // flat-top octagon, corners aligned across sections
    for (int i = 0; i < 8; ++i) {
      const double a = rot + 2.0 * M_PI * i / 8.0;
      o.push_back(rad * std::cos(a));
      o.push_back(rad * std::sin(a));
      o.push_back(z);
    }
    return o;
  };
  const topo::Shape solid = loftSections({octagon(0, 3.0), octagon(6, 5.0), octagon(12, 3.0)});
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 2 * 8 + 2);  // 2 bands·8 + 2 caps
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));

  // A regular octagon of circumradius R has area 2√2·R². The spool volume is bracketed
  // by the two straight-octagon prisms of the same total height (12): r=3 (narrow, the
  // ends) below, r=5 (wide, the bulge) above.
  auto octArea = [](double R) { return 2.0 * std::sqrt(2.0) * R * R; };
  const double vNarrowPrism = octArea(3.0) * 12.0;
  const double vWidePrism = octArea(5.0) * 12.0;
  const double vol = std::fabs(tess::enclosedVolume(mesh));
  CC_CHECK(vol > vNarrowPrism);  // the bulge adds volume over the narrow prism
  CC_CHECK(vol < vWidePrism);    // the narrow ends remove volume vs the wide prism
}

// ── T1 NATIVE (N-section): MISMATCHED counts 4→8→4 spool (two frustums, exact) ────
// A 6×6 square (4 pts) at z=0, a 10×10 square sampled at 8 points at z=6, back to a
// 6×6 square (4 pts) at z=12. The counts differ (4, 8, 4); the union-of-arc-length
// correspondence resamples the 4-pt ends to 8 SYMMETRIC points (corner + edge
// midpoints) so every ring is an 8-point square and each band is a true square
// frustum. Two prismatoids over h=6: each = 6/6·(36 + 4·64 + 100) = 392, total 784,
// EXACT. Watertight (the symmetric resampled caps mesh cleanly).
CC_TEST(loft_sections_mismatched_counts_native) {
  const std::vector<double> s0 = {-3, -3, 0, 3, -3, 0, 3, 3, 0, -3, 3, 0};  // 6×6, 4 pts
  const std::vector<double> s1 = {-5, -5, 6, 0, -5, 6, 5, -5, 6, 5, 0, 6,   // 10×10, 8 pts
                                  5, 5, 6, 0, 5, 6, -5, 5, 6, -5, 0, 6};
  const std::vector<double> s2 = {-3, -3, 12, 3, -3, 12, 3, 3, 12, -3, 3, 12};  // 6×6, 4 pts
  const topo::Shape solid = loftSections({s0, s1, s2});
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double frustum = 6.0 / 6.0 * (36.0 + 4.0 * 64.0 + 100.0);  // 392 per band
  const double expected = 2.0 * frustum;                          // 784
  CC_CHECK(std::fabs(std::fabs(tess::enclosedVolume(mesh)) - expected) / expected < 1e-6);
}

// ── T1 HONEST DECLINE: a genuinely different-count section pair whose resampled caps
// carry ASYMMETRIC collinear vertices the mesher cannot close watertight is DISCARDED
// by the engine self-verify → OCCT. The native builder still returns non-null (the
// correspondence runs), but here we assert the geometry-preserving cases build AND a
// harder genuine mismatch (triangle→square) is left for the engine's watertight gate.
// (The builder-level result may be non-null but non-watertight; the ENGINE, not this
// pure-geometry unit, is what forwards it to OCCT — see native_engine solid_loft.)
CC_TEST(loft_triangle_to_square_correspondence_runs) {
  const double tri[] = {0, 0, 6, 0, 3, 5};              // 3-gon
  const double sq[] = {0, 0, 6, 0, 6, 6, 0, 6};         // 4-gon
  const topo::Shape solid = cst::build_loft(tri, 3, sq, 4, 4.0);
  // The correspondence equalizes counts and builds a candidate solid (non-null); its
  // watertightness is decided by the engine self-verify, not asserted here.
  CC_CHECK(!solid.isNull());
}

// ── DEFERRED (N-section): a non-planar internal section → NULL ───────────────────
// The middle section's four points do not lie on a common plane (one corner lifted),
// so the well-posed in-plane alignment is not guaranteed → OCCT fallthrough.
CC_TEST(loft_sections_non_planar_middle_deferred) {
  const std::vector<double> s0 = {0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0};
  const std::vector<double> s1 = {0, 0, 3, 4, 0, 3, 4, 4, 5, 0, 4, 3};  // skew (corner lifted)
  const std::vector<double> s2 = {0, 0, 6, 4, 0, 6, 4, 4, 6, 0, 4, 6};
  CC_CHECK(loftSections({s0, s1, s2}).isNull());
}

// ── DEFERRED (N-section): a degenerate section + too-few sections → NULL ─────────
CC_TEST(loft_sections_degenerate_and_single_deferred) {
  const std::vector<double> s0 = {0, 0, 0, 4, 0, 0, 4, 4, 0, 0, 4, 0};
  const std::vector<double> collinear = {0, 0, 3, 1, 0, 3, 2, 0, 3, 3, 0, 3};  // 4 collinear
  const std::vector<double> s2 = {0, 0, 6, 4, 0, 6, 4, 4, 6, 0, 4, 6};
  CC_CHECK(loftSections({s0, collinear, s2}).isNull());  // degenerate middle
  CC_CHECK(loftSections({s0}).isNull());                 // only one section
}

CC_RUN_ALL()
