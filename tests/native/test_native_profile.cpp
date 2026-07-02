// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for Tier-A native construction (Phase 4 #4b): typed profiles,
// holed extrude, and typed-profile revolve (src/native/construct/profile.h).
// OCCT-FREE — Gate 1 (host, analytic) of the two-gate model in
// openspec/NATIVE-REWRITE.md. Each result is validated with the native tessellator
// (watertight / enclosed volume / area) and the topology Explorer (surface kinds).
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_profile.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_profile
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace m = cybercad::native::math;

namespace {
constexpr double kPi = 3.14159265358979323846;

struct SurfaceTally {
  int plane = 0, cylinder = 0, cone = 0, sphere = 0, other = 0;
};
SurfaceTally classifyFaces(const topo::Shape& shape) {
  SurfaceTally t;
  for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next()) {
    const auto s = topo::surfaceOf(ex.current());
    if (!s) { ++t.other; continue; }
    switch (s->surface->kind) {
      case topo::FaceSurface::Kind::Plane: ++t.plane; break;
      case topo::FaceSurface::Kind::Cylinder: ++t.cylinder; break;
      case topo::FaceSurface::Kind::Cone: ++t.cone; break;
      case topo::FaceSurface::Kind::Sphere: ++t.sphere; break;
      default: ++t.other; break;
    }
  }
  return t;
}
int countSub(const topo::Shape& shape, topo::ShapeType type) {
  int n = 0;
  for (topo::Explorer ex(shape, type); ex.more(); ex.next()) ++n;
  return n;
}
bool isZCap(const topo::Shape& f) {
  const auto s = topo::surfaceOf(f);
  if (!s || s->surface->kind != topo::FaceSurface::Kind::Plane) return false;
  return std::fabs(std::fabs(s->surface->frame.z.z()) - 1.0) <= 1e-9;  // normal ±Z
}
// Count the planar ±Z cap faces that carry exactly TWO wires (one outer + one
// hole) — i.e. the hole is a real inner loop on each cap (2 wires per cap).
int planarCapsWithOneHole(const topo::Shape& shape) {
  int n = 0;
  for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next()) {
    const topo::Shape& f = ex.current();
    if (!isZCap(f)) continue;
    if (countSub(f, topo::ShapeType::Wire) == 2) ++n;  // outer wire + 1 hole wire
  }
  return n;
}
}  // namespace

// ── build_prism_with_holes: square with a circular through-hole ──────────────---
// A 10×10 outer square, one circular hole (centre (5,5), r=2), extruded depth 4.
// Volume = (100 − π·4)·4 = 400 − 16π. The hole keeps a TRUE circle edge, so a
// Cylinder wall appears; watertight with the exact-minus-curvature volume.
CC_TEST(prism_circular_hole_volume_watertight) {
  const double outer[] = {0, 0, 10, 0, 10, 10, 0, 10};
  std::vector<cst::CircleHole> holes = {{5.0, 5.0, 2.0}};
  const topo::Shape solid = cst::build_prism_with_holes(outer, 4, holes, {}, 4.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  const SurfaceTally st = classifyFaces(solid);
  CC_CHECK(st.cylinder >= 1);  // the circular hole wall is a true cylinder
  CC_CHECK(st.plane >= 6);     // 2 caps + 4 outer walls

  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));

  const double expected = (100.0 - kPi * 4.0) * 4.0;
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.02);
}

// ── build_prism_with_holes: square with a square polygon hole ────────────────---
// 10×10 outer, one 2×2 square hole centred at (5,5), depth 3. Volume =
// (100 − 4)·3 = 288, exact (all planar). Watertight; 12 faces (2 caps + 4 outer
// walls + 4 hole walls).
CC_TEST(prism_polygon_hole_volume_watertight) {
  const double outer[] = {0, 0, 10, 0, 10, 10, 0, 10};
  std::vector<std::vector<m::Point3>> poly = {
      {{4, 4, 0}, {6, 4, 0}, {6, 6, 0}, {4, 6, 0}}};
  const topo::Shape solid = cst::build_prism_with_holes(outer, 4, {}, poly, 3.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  CC_CHECK_EQ(countSub(solid, topo::ShapeType::Face), 10);  // 2 caps + 4 outer + 4 hole walls
  tess::MeshParams p;
  p.deflection = 0.2;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - 288.0) < 1e-6);
}

// ── build_prism_with_holes: MULTIPLE circular holes ─────────────────────────---
// A 20×10 outer rectangle with TWO circular holes (r=1.5 at (5,5), r=2 at (15,5)),
// depth 3. Volume = (200 − π·1.5² − π·2²)·3. Exercises the multi-hole cap
// triangulation (rightmost-first, visibility-checked bridging in uv_triangulate.h):
// both holes must be subtracted AND the mesh watertight.
CC_TEST(prism_two_circular_holes_watertight) {
  const double outer[] = {0, 0, 20, 0, 20, 10, 0, 10};
  std::vector<cst::CircleHole> holes = {{5.0, 5.0, 1.5}, {15.0, 5.0, 2.0}};
  const topo::Shape solid = cst::build_prism_with_holes(outer, 4, holes, {}, 3.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;
  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = (200.0 - kPi * 1.5 * 1.5 - kPi * 2.0 * 2.0) * 3.0;
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.01);
}

// ── build_prism_profile: a typed profile with a circular AND a polygon hole ──---
// A 20×10 rectangle (4 line segments) with one circular hole and one square hole,
// depth 4 → volume (200 − π·1.5² − 16)·4. Both holes subtract; watertight.
CC_TEST(profile_circle_and_polygon_hole) {
  std::vector<cst::ProfileSegment> segs(4);
  const double sq[4][4] = {{0, 0, 20, 0}, {20, 0, 20, 10}, {20, 10, 0, 10}, {0, 10, 0, 0}};
  for (int i = 0; i < 4; ++i) {
    segs[i].kind = 0;
    segs[i].x0 = sq[i][0]; segs[i].y0 = sq[i][1];
    segs[i].x1 = sq[i][2]; segs[i].y1 = sq[i][3];
  }
  std::vector<cst::CircleHole> circ = {{5.0, 5.0, 1.5}};
  std::vector<std::vector<m::Point3>> poly = {{{13, 3, 0}, {17, 3, 0}, {17, 7, 0}, {13, 7, 0}}};
  const topo::Shape solid = cst::build_prism_profile(segs, circ, poly, 4.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;
  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = (200.0 - kPi * 1.5 * 1.5 - 16.0) * 4.0;
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.01);
}

// ── build_revolution_profile: partial (90°) line-profile turn ────────────────---
// A rectangle [x∈[1,2], y∈[0,3]] revolved only π/2 → a quarter tube, volume
// π(2²−1²)·3 / 4 = 9π/4. Two planar meridian caps close the opening; watertight.
CC_TEST(revolve_profile_partial_turn) {
  const double pts[4][2] = {{1, 0}, {2, 0}, {2, 3}, {1, 3}};
  std::vector<cst::ProfileSegment> segs(4);
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    segs[i].kind = 0;
    segs[i].x0 = pts[i][0]; segs[i].y0 = pts[i][1];
    segs[i].x1 = pts[j][0]; segs[i].y1 = pts[j][1];
  }
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  const topo::Shape solid = cst::build_revolution_profile(segs, yAxis, kPi / 2.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = kPi * (4.0 - 1.0) * 3.0 / 4.0;  // 9π/4
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.02);
}

// ── build_prism_profile: a full-circle profile (kind 2) → a solid cylinder ────---
// A single kind-2 circle segment (centre origin, r=3) extruded depth 5 → a solid
// cylinder, volume πr²h = 45π. TRUE circle edges: two caps (each with one circle
// wire) + one cylinder wall. Watertight within the deflection bound.
CC_TEST(profile_full_circle_extrude_cylinder) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 2;
  segs[0].cx = 0; segs[0].cy = 0; segs[0].r = 3.0;
  const topo::Shape solid = cst::build_prism_profile(segs, {}, {}, 5.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  const SurfaceTally st = classifyFaces(solid);
  CC_CHECK_EQ(st.cylinder, 1);  // one true cylinder wall
  CC_CHECK_EQ(st.plane, 2);     // two circular caps

  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = kPi * 9.0 * 5.0;  // 45π
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - expected) / expected < 0.02);
}

// ── build_prism_profile: a D-shape (line + arc) typed outer profile ──────────---
// A half-disc "D": one straight edge (0,2)→(0,-2) and one semicircle arc (r=2,
// centre origin, from -π/2 to +π/2 on the +x side), extruded depth 3. The arc
// becomes a TRUE Circle cap edge + Cylinder side wall(s) (NOT a chord polyline);
// area = πr²/2 = 2π, so volume = 2π·3 = 6π. Watertight; the arc side classifies to
// Cylinder (a semicircle spans exactly π ⇒ ONE bounded, non-periodic Cylinder patch,
// matching OCCT's single cylindrical face — the extrude split threshold is π, not the
// revolve's 120°; see assembleTypedProfileSolid), the caps + straight wall to Plane.
CC_TEST(profile_dshape_line_arc_true_cylinder) {
  std::vector<cst::ProfileSegment> segs(2);
  segs[0].kind = 0;  // straight diameter edge (0,2) → (0,-2)
  segs[0].x0 = 0; segs[0].y0 = 2; segs[0].x1 = 0; segs[0].y1 = -2;
  segs[1].kind = 1;  // semicircle arc back (0,-2) → (0,2) on the +x side
  segs[1].cx = 0; segs[1].cy = 0; segs[1].r = 2.0;
  segs[1].x0 = 0; segs[1].y0 = -2; segs[1].x1 = 0; segs[1].y1 = 2;
  segs[1].a0 = -kPi / 2.0; segs[1].a1 = kPi / 2.0;
  const topo::Shape solid = cst::build_prism_profile(segs, {}, {}, 3.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  // The arc side wall is a real Cylinder (a TRUE Circle edge extruded), not planar.
  const SurfaceTally st = classifyFaces(solid);
  CC_CHECK(st.cylinder >= 1);
  CC_CHECK(st.plane >= 3);  // 2 caps + 1 straight wall

  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = 2.0 * kPi * 3.0;  // (πr²/2)·depth = 6π
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - expected) / expected < 0.02);
}

// ── build_prism_profile: kind-3 spline is DEFERRED (returns NULL) ────────────---
CC_TEST(profile_spline_deferred_null) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 3;
  segs[0].ptOffset = 0; segs[0].ptCount = 4;
  CC_CHECK(cst::build_prism_profile(segs, {}, {}, 2.0).isNull());
}

// ── build_revolution_profile: line profile → cylinder (parity with #4 path) ──---
// A rectangle profile [x∈[1,2], y∈[0,3]] as 4 line segments, full 2π about the Y
// axis → a tube, volume π(2²−1²)·3 = 9π. Watertight.
CC_TEST(revolve_profile_line_tube) {
  const double pts[4][2] = {{1, 0}, {2, 0}, {2, 3}, {1, 3}};
  std::vector<cst::ProfileSegment> segs(4);
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    segs[i].kind = 0;
    segs[i].x0 = pts[i][0]; segs[i].y0 = pts[i][1];
    segs[i].x1 = pts[j][0]; segs[i].y1 = pts[j][1];
  }
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  const topo::Shape solid = cst::build_revolution_profile(segs, yAxis, 2.0 * kPi);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;
  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = kPi * (4.0 - 1.0) * 3.0;  // 9π
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - expected) / expected < 0.02);
}

// ── build_revolution_profile: on-axis arc → a SPHERE ─────────────────────────---
// A half-circle arc of radius R=3 centred at the origin ON the Y axis, from the
// south pole (0,-3) to the north pole (0,3), revolved full 2π → a full sphere of
// radius 3, volume (4/3)πR³ = 36π. Confirms the Sphere-band classification + volume.
CC_TEST(revolve_profile_arc_sphere_volume) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1;
  segs[0].cx = 0; segs[0].cy = 0; segs[0].r = 3.0;
  // Arc endpoints in the profile plane: (x=0,y=-3) south, (x=0,y=+3)? These lie ON
  // the axis (r=0). Use the meridian at x≥0: a half great-circle from (0,-3) up
  // through (3,0) to (0,3). a0=-π/2 → a1=+π/2 in the (x=r, y=h) plane.
  segs[0].x0 = 0.0; segs[0].y0 = -3.0;
  segs[0].x1 = 0.0; segs[0].y1 = 3.0;
  segs[0].a0 = -kPi / 2.0; segs[0].a1 = kPi / 2.0;
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  const topo::Shape solid = cst::build_revolution_profile(segs, yAxis, 2.0 * kPi);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  const SurfaceTally st = classifyFaces(solid);
  CC_CHECK(st.sphere >= 1);
  CC_CHECK_EQ(st.cone + st.other, 0);

  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));
  const double expected = (4.0 / 3.0) * kPi * 27.0;  // 36π
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - expected) / expected < 0.03);
}

// ── build_revolution_profile: off-axis arc (Torus) is DEFERRED (NULL) ────────---
CC_TEST(revolve_profile_offaxis_arc_deferred_null) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1;
  segs[0].cx = 5.0; segs[0].cy = 0.0; segs[0].r = 1.0;  // centre off the Y axis
  segs[0].x0 = 6.0; segs[0].y0 = 0.0;
  segs[0].x1 = 5.0; segs[0].y1 = 1.0;
  segs[0].a0 = 0.0; segs[0].a1 = kPi / 2.0;
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  CC_CHECK(cst::build_revolution_profile(segs, yAxis, 2.0 * kPi).isNull());
}

// ── build_revolution_profile: spline segment DEFERRED (NULL) ─────────────────---
CC_TEST(revolve_profile_spline_deferred_null) {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 3;
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  CC_CHECK(cst::build_revolution_profile(segs, yAxis, 2.0 * kPi).isNull());
}

// ── build_prism_with_holes: 20×20 square, central circular hole r=5, depth 10 ─---
// The task's canonical Tier-A case: volume = (400 − π·25)·10, watertight, AND the
// hole is a REAL inner loop — each of the two planar ±Z caps carries exactly TWO
// wires (one outer + one hole), not a merged/bridged single boundary.
CC_TEST(prism_central_circle_hole_two_wires_per_cap) {
  const double outer[] = {0, 0, 20, 0, 20, 20, 0, 20};
  std::vector<cst::CircleHole> holes = {{10.0, 10.0, 5.0}};
  const topo::Shape solid = cst::build_prism_with_holes(outer, 4, holes, {}, 10.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  // Topology: exactly two planar caps, each with 2 wires (outer + the hole loop).
  CC_CHECK_EQ(planarCapsWithOneHole(solid), 2);
  const SurfaceTally st = classifyFaces(solid);
  CC_CHECK(st.cylinder >= 1);  // hole kept as a TRUE circle → cylinder wall

  tess::MeshParams p;
  p.deflection = 0.01;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));

  const double expected = (400.0 - kPi * 25.0) * 10.0;  // (400 − 25π)·10
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.01);
}

// ── build_prism_profile: a rectangle with ONE rounded (arc) corner ───────────---
// A 10×6 rectangle whose top-right corner is replaced by a quarter-circle fillet of
// radius r=2. Typed profile = 4 straight edges + 1 quarter arc (5 typed segments).
// Analytic top-face area = full rect − corner square + quarter disc
//   = 60 − r² + π r²/4 = 60 − 4 + π. Extruded depth 3 → volume = (56 + π)·3.
// The arc is ONE curved edge on each cap (a TRUE Circle edge), NOT a sampled
// polyline: the cap outer wire has exactly 5 edges (4 line + 1 arc), and the arc
// side wall classifies to Cylinder. If it were sampled the edge count would balloon.
CC_TEST(profile_rounded_rect_one_arc_edge) {
  const double r = 2.0;
  // Corners: A(0,0) B(10,0) then up to the fillet start (10,4), quarter arc to
  // (8,6), then across the top to (0,6), and down the left back to A.
  // Fillet-corner circle centre = (8,4), radius r; arc from angle 0 (at (10,4)) to
  // π/2 (at (8,6)).
  std::vector<cst::ProfileSegment> segs(5);
  auto line = [&](int i, double x0, double y0, double x1, double y1) {
    segs[i].kind = 0; segs[i].x0 = x0; segs[i].y0 = y0; segs[i].x1 = x1; segs[i].y1 = y1;
  };
  line(0, 0, 0, 10, 0);   // bottom
  line(1, 10, 0, 10, 4);  // right up to fillet start
  segs[2].kind = 1;       // quarter-circle fillet, centre (8,4) r=2, 0 → π/2
  segs[2].cx = 8; segs[2].cy = 4; segs[2].r = r;
  segs[2].x0 = 10; segs[2].y0 = 4; segs[2].x1 = 8; segs[2].y1 = 6;
  segs[2].a0 = 0.0; segs[2].a1 = kPi / 2.0;
  line(3, 8, 6, 0, 6);    // top across
  line(4, 0, 6, 0, 0);    // left down (closes)

  const topo::Shape solid = cst::build_prism_profile(segs, {}, {}, 3.0);
  CC_CHECK(!solid.isNull());
  if (solid.isNull()) return;

  // The arc is ONE curved edge per cap: each planar cap's outer wire = 5 edges
  // (4 line + 1 arc), matching the typed segment count — not a sampled polygon.
  // No holes here → each cap has exactly one wire, so the face's total edge count
  // equals the outer-loop edge count.
  int capEdgeCount = -1;
  for (topo::Explorer ex(solid, topo::ShapeType::Face); ex.more(); ex.next()) {
    const topo::Shape& f = ex.current();
    if (!isZCap(f)) continue;
    capEdgeCount = countSub(f, topo::ShapeType::Edge);
    break;
  }
  CC_CHECK_EQ(capEdgeCount, 5);  // 4 line + 1 arc, NOT a sampled polyline
  const SurfaceTally st = classifyFaces(solid);
  CC_CHECK(st.cylinder >= 1);  // the fillet arc extrudes to a TRUE Cylinder wall

  tess::MeshParams p;
  p.deflection = 0.005;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(solid);
  CC_CHECK(tess::isWatertight(mesh));

  const double area = 60.0 - r * r + kPi * r * r / 4.0;  // 56 + π
  const double expected = area * 3.0;
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.01);
}

CC_RUN_ALL()
