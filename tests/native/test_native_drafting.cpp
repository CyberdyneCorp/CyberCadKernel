// SPDX-License-Identifier: Apache-2.0
//
// Host analytic tests for the native drafting HLR core (src/native/drafting,
// capability `native-drafting`). OCCT-FREE — this is Gate (a) HOST ANALYTIC of
// the two-gate verification model: the code compiles and unit-tests with
// clang++ -std=c++20, no OCCT, no simulator. Gate (b) native-vs-OCCT
// HLRBRep_Algo parity on the sim is a separate harness.
//
// The canonical analytic result: a box viewed down an isometric CORNER shows 3
// faces — 9 edges VISIBLE (3 at the near corner + the 6-edge silhouette
// outline) and 3 edges HIDDEN (the three meeting at the occluded far corner).
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_drafting.cpp \
//     -I src -I tests -o test_native_drafting && ./test_native_drafting
//
#include "native/drafting/native_drafting.h"

#include "harness.h"

#include <array>
#include <cmath>
#include <vector>

namespace {

namespace draft = cybercad::native::drafting;
namespace math = cybercad::native::math;

// Unit box (half-extent 1) centred at the origin. Corners indexed by sign bits:
//   0:(-,-,-) 1:(+,-,-) 2:(+,+,-) 3:(-,+,-) 4:(-,-,+) 5:(+,-,+) 6:(+,+,+) 7:(-,+,+)
std::vector<math::Point3> boxCorners() {
  return {{-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
          {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1}};
}

// 12 edges of the box (index pairs into boxCorners()).
std::vector<draft::EdgeIndices> boxEdges() {
  return {{0, 1}, {1, 2}, {2, 3}, {3, 0},   // bottom
          {4, 5}, {5, 6}, {6, 7}, {7, 4},   // top
          {0, 4}, {1, 5}, {2, 6}, {3, 7}};  // verticals
}

// 12 triangles (2 per face); winding is irrelevant to occlusion.
std::vector<std::array<std::uint32_t, 3>> boxTriangles() {
  auto quad = [](std::vector<std::array<std::uint32_t, 3>>& t, std::uint32_t a, std::uint32_t b,
                 std::uint32_t c, std::uint32_t d) {
    t.push_back({a, b, c});
    t.push_back({a, c, d});
  };
  std::vector<std::array<std::uint32_t, 3>> t;
  quad(t, 0, 1, 2, 3);  // -Z
  quad(t, 4, 5, 6, 7);  // +Z
  quad(t, 0, 1, 5, 4);  // -Y
  quad(t, 3, 2, 6, 7);  // +Y
  quad(t, 0, 3, 7, 4);  // -X
  quad(t, 1, 2, 6, 5);  // +X
  return t;
}

bool nearlyEq(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// True if the two 2D points coincide within tol.
bool ptEq(double ax, double ay, double bx, double by, double tol) {
  return nearlyEq(ax, bx, tol) && nearlyEq(ay, by, tol);
}

}  // namespace

// A box from an isometric corner: exactly 9 visible + 3 hidden segments, and the
// 3 hidden segments meet at a single common point (the projection of the hidden
// far corner).
CC_TEST(box_isometric_corner_9_visible_3_hidden) {
  const auto verts = boxCorners();
  const auto edges = boxEdges();
  const auto tris = boxTriangles();

  draft::Occluder occ{&verts, &tris};
  draft::OrthographicView view{math::Dir3{-1, -1, -1}, math::Dir3{0, 0, 1}};

  const draft::HlrResult r = draft::projectOrthographic(verts, edges, occ, view);

  CC_CHECK_EQ(r.visible.size(), static_cast<std::size_t>(9));
  CC_CHECK_EQ(r.hidden.size(), static_cast<std::size_t>(3));

  // The 3 hidden edges are the ones meeting at the hidden far corner (index 0),
  // so all three share one endpoint. Find a point present as an endpoint of all
  // three hidden segments.
  if (r.hidden.size() == 3) {
    const auto& h = r.hidden;
    auto touches = [&](double x, double y) {
      int n = 0;
      for (const auto& s : h)
        if (ptEq(s.ax, s.ay, x, y, 1e-7) || ptEq(s.bx, s.by, x, y, 1e-7)) ++n;
      return n;
    };
    // Every endpoint of the first hidden segment is a candidate common corner.
    const bool sharedCorner =
        touches(h[0].ax, h[0].ay) == 3 || touches(h[0].bx, h[0].by) == 3;
    CC_CHECK(sharedCorner);
  }

  // Conservation: total projected length is invariant under the visible/hidden
  // split (no length created or destroyed by classification).
  auto seglen = [](const draft::Segment2D& s) {
    return std::hypot(s.bx - s.ax, s.by - s.ay);
  };
  double split = 0.0;
  for (const auto& s : r.visible) split += seglen(s);
  for (const auto& s : r.hidden) split += seglen(s);
  double raw = 0.0;  // sum of raw projected edge lengths
  for (const auto& e : edges) {
    const math::Point3 a = verts[e[0]], b = verts[e[1]];
    // project both endpoints with the same basis the algorithm uses
    const math::Vec3 vd = view.viewDir.vec();
    const math::Dir3 right{cross(vd, view.up.vec())};
    const math::Vec3 rr = right.vec();
    const math::Vec3 tu = cross(rr, vd);
    const double ax = dot(a.asVec(), rr), ay = dot(a.asVec(), tu);
    const double bx = dot(b.asVec(), rr), by = dot(b.asVec(), tu);
    raw += std::hypot(bx - ax, by - ay);
  }
  CC_CHECK(nearlyEq(split, raw, 1e-6));
}

// With no occluder every edge is fully visible.
CC_TEST(box_no_occluder_all_visible) {
  const auto verts = boxCorners();
  const auto edges = boxEdges();
  std::vector<std::array<std::uint32_t, 3>> noTris;  // empty
  draft::Occluder occ{&verts, &noTris};
  draft::OrthographicView view{math::Dir3{-1, -1, -1}, math::Dir3{0, 0, 1}};

  const draft::HlrResult r = draft::projectOrthographic(verts, edges, occ, view);
  CC_CHECK_EQ(r.visible.size(), static_cast<std::size_t>(12));
  CC_CHECK_EQ(r.hidden.size(), static_cast<std::size_t>(0));
}

// An edge half-covered by a nearer face SPLITS into one visible + one hidden
// segment exactly at the coverage boundary (proves edge splitting).
CC_TEST(edge_splits_at_visibility_transition) {
  // Edge along X at z=0, from x=-5 to x=+5.
  std::vector<math::Point3> verts = {{-5, 0, 0}, {5, 0, 0},
                                     // occluder quad at z=+1 (nearer the camera),
                                     // covering x in [0,6], y in [-1,1]:
                                     {0, -1, 1}, {6, -1, 1}, {6, 1, 1}, {0, 1, 1}};
  std::vector<draft::EdgeIndices> edges = {{0, 1}};
  std::vector<std::array<std::uint32_t, 3>> tris = {{2, 3, 4}, {2, 4, 5}};
  draft::Occluder occ{&verts, &tris};
  draft::OrthographicView view{math::Dir3{0, 0, -1}, math::Dir3{0, 1, 0}};  // proj = (x,y)

  const draft::HlrResult r = draft::projectOrthographic(verts, edges, occ, view);
  CC_CHECK_EQ(r.visible.size(), static_cast<std::size_t>(1));
  CC_CHECK_EQ(r.hidden.size(), static_cast<std::size_t>(1));
  if (r.visible.size() == 1 && r.hidden.size() == 1) {
    // Visible half is x in [-5,0]; hidden half is x in [0,5]; split at x=0.
    const auto& vseg = r.visible[0];
    const auto& hseg = r.hidden[0];
    CC_CHECK(nearlyEq(std::min(vseg.ax, vseg.bx), -5.0, 1e-9));
    CC_CHECK(nearlyEq(std::max(hseg.ax, hseg.bx), 5.0, 1e-9));
    // Shared split endpoint at x=0 (within refinement tolerance).
    const double vSplit = std::max(vseg.ax, vseg.bx);
    const double hSplit = std::min(hseg.ax, hseg.bx);
    CC_CHECK(nearlyEq(vSplit, 0.0, 1e-6));
    CC_CHECK(nearlyEq(hSplit, 0.0, 1e-6));
  }
}

CC_RUN_ALL()
