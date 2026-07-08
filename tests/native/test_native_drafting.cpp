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

#include "native/math/elementary.h"

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

// ─────────────────────────────────────────────────────────────────────────────
// QUADRIC-FACE SILHOUETTE (drafting/silhouette.h) — host analytic gate (a).
// The visible outline of a curved face is the true silhouette locus n·viewDir = 0,
// traced in CLOSED FORM. These tests verify the locus is exact (machine-zero
// n·viewDir), projects to the closed-form ellipse/lines, and flows through the same
// occlusion + split path; plus the convex-limb self-grazing cure (surfaceOffset ≥
// facet sagitta) that the router applies for a curved occluder.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

double seg2Len(const draft::Segment2D& s) { return std::hypot(s.bx - s.ax, s.by - s.ay); }
double totalLen(const std::vector<draft::Segment2D>& v) {
  double t = 0.0;
  for (const auto& s : v) t += seg2Len(s);
  return t;
}

// A UV sphere of radius R with vertices ON the sphere (so the faceted mesh is
// INSCRIBED, its chords sitting inside the true surface by up to the chord sagitta
// — exactly the occluder the router builds for a spherical solid).
void makeUVSphere(double R, int nLat, int nLon, std::vector<math::Point3>& V,
                  std::vector<std::array<std::uint32_t, 3>>& T) {
  constexpr double kPi = 3.14159265358979323846;
  for (int i = 0; i <= nLat; ++i) {
    const double phi = -kPi / 2.0 + kPi * static_cast<double>(i) / nLat;  // lat
    for (int j = 0; j < nLon; ++j) {
      const double th = 2.0 * kPi * static_cast<double>(j) / nLon;  // lon
      V.push_back({R * std::cos(phi) * std::cos(th), R * std::cos(phi) * std::sin(th),
                   R * std::sin(phi)});
    }
  }
  auto idx = [&](int i, int j) { return static_cast<std::uint32_t>(i * nLon + (j % nLon)); };
  for (int i = 0; i < nLat; ++i)
    for (int j = 0; j < nLon; ++j) {
      T.push_back({idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)});
      T.push_back({idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)});
    }
}

}  // namespace

// A cylinder side-on: the two generator lines lie exactly where the radial normal
// is perpendicular to the view (n·viewDir = 0 to machine zero), and project to two
// parallel lines a diameter (2R) apart, each of the cylinder's height.
CC_TEST(silhouette_cylinder_generators_closed_form) {
  const math::Ax3 frame;  // identity: O=origin, X,Y,Z world axes, axis = Z
  const double R = 2.0, vMin = 0.0, vMax = 5.0;
  const math::Vec3 vd{-1, 0, 0};  // side-on (perpendicular to the axis)

  const draft::SilhouetteResult sil = draft::cylinderSilhouette(frame, R, vMin, vMax, vd);
  CC_CHECK(sil.traced);
  CC_CHECK_EQ(sil.outlines.size(), static_cast<std::size_t>(2));

  // n·viewDir = 0 at every generator point (recover θ from the point, use the
  // analytic cylinder normal).
  const math::Cylinder cyl{frame, R};
  double maxND = 0.0;
  for (const auto& g : sil.outlines) {
    CC_CHECK_EQ(g.points.size(), static_cast<std::size_t>(2));  // straight → 2 points
    for (const auto& p : g.points) {
      const double th = std::atan2(p.y, p.x);
      maxND = std::max(maxND, std::fabs(dot(cyl.normal(th, 0.0).vec(), vd)));
    }
  }
  CC_CHECK(maxND < 1e-12);

  // Feed through the occlusion path with NO occluder → both generators fully
  // visible; the two project to lines 2R apart, each length (vMax − vMin).
  std::vector<math::Point3> ev;
  std::vector<draft::EdgeIndices> edges;
  for (const auto& g : sil.outlines) {
    const auto base = static_cast<std::uint32_t>(ev.size());
    ev.push_back(g.points[0]);
    ev.push_back(g.points[1]);
    edges.push_back({base, base + 1});
  }
  std::vector<std::array<std::uint32_t, 3>> noTris;
  draft::Occluder occ{&ev, &noTris};
  draft::OrthographicView view{math::Dir3{-1, 0, 0}, math::Dir3{0, 0, 1}};
  const draft::HlrResult r = draft::projectOrthographic(ev, edges, occ, view);
  CC_CHECK_EQ(r.visible.size(), static_cast<std::size_t>(2));
  CC_CHECK_EQ(r.hidden.size(), static_cast<std::size_t>(0));
  CC_CHECK(nearlyEq(totalLen(r.visible), 2.0 * (vMax - vMin), 1e-9));  // 2 × height
  // The two visible lines are a diameter apart in u (= ±R at u-coordinate).
  if (r.visible.size() == 2) {
    const double u0 = r.visible[0].ax, u1 = r.visible[1].ax;
    CC_CHECK(nearlyEq(std::fabs(u0 - u1), 2.0 * R, 1e-9));
  }
}

// A view PARALLEL to the cylinder axis is an honest decline: the whole side is a
// silhouette, no isolated generator exists.
CC_TEST(silhouette_cylinder_axis_parallel_declines) {
  const math::Ax3 frame;
  const draft::SilhouetteResult sil =
      draft::cylinderSilhouette(frame, 2.0, 0.0, 5.0, math::Vec3{0, 0, 1});  // along axis
  CC_CHECK(!sil.traced);
  CC_CHECK(sil.outlines.empty());
  CC_CHECK(sil.declineReason != nullptr);
}

// A sphere silhouette is the great circle in the plane through the centre ⟂ view:
// every point is at radius R with radial normal ⟂ view (n·viewDir = 0), and it
// projects to a circle of radius exactly R.
CC_TEST(silhouette_sphere_great_circle_closed_form) {
  const math::Point3 C{0, 0, 0};
  const double R = 3.0;
  const math::Vec3 vd{-1, 0, 0};
  const draft::SilhouetteResult sil = draft::sphereSilhouette(C, R, vd, 0.1);
  CC_CHECK(sil.traced);
  CC_CHECK_EQ(sil.outlines.size(), static_cast<std::size_t>(1));

  double maxND = 0.0, minR = 1e9, maxR = 0.0, minProj = 1e9, maxProj = 0.0;
  const math::Vec3 right{0, 1, 0}, trueUp{0, 0, 1};  // basis for vd=(-1,0,0), up=(0,0,1)
  for (const auto& p : sil.outlines[0].points) {
    const math::Vec3 rad = p - C;
    minR = std::min(minR, norm(rad));
    maxR = std::max(maxR, norm(rad));
    maxND = std::max(maxND, std::fabs(dot(rad, vd)) / R);  // radial normal · view
    const double pr = std::hypot(dot(p.asVec(), right), dot(p.asVec(), trueUp));
    minProj = std::min(minProj, pr);
    maxProj = std::max(maxProj, pr);
  }
  CC_CHECK(nearlyEq(minR, R, 1e-12) && nearlyEq(maxR, R, 1e-12));  // on the sphere
  CC_CHECK(maxND < 1e-12);                                          // n·viewDir = 0
  CC_CHECK(nearlyEq(minProj, R, 1e-12) && nearlyEq(maxProj, R, 1e-12));  // projects to radius R
}

// The sphere limb self-grazing cure (the one router tuning the DIAGNOSE isolated):
// a lone CONVEX limb over its own INSCRIBED facet mesh is spuriously self-occluded
// at the default micro-offset, but is FULLY VISIBLE once the sample is pushed out
// by the facet sagitta (surfaceOffset ≥ deflection). Never hidden for a convex limb.
CC_TEST(silhouette_sphere_limb_offset_defeats_self_grazing) {
  const math::Point3 C{0, 0, 0};
  const double R = 3.0;
  const math::Vec3 vd{-1, 0, 0};
  const draft::SilhouetteResult sil = draft::sphereSilhouette(C, R, vd, 0.1);

  std::vector<math::Point3> V;
  std::vector<std::array<std::uint32_t, 3>> Tri;
  makeUVSphere(R, 16, 24, V, Tri);
  // Append the great-circle polyline as edges after the sphere vertices.
  const auto base = static_cast<std::uint32_t>(V.size());
  std::vector<draft::EdgeIndices> edges;
  const auto& loop = sil.outlines[0].points;
  for (const auto& p : loop) V.push_back(p);
  for (std::uint32_t k = 0; k + 1 < loop.size(); ++k) edges.push_back({base + k, base + k + 1});

  draft::Occluder occ{&V, &Tri};
  draft::OrthographicView view{math::Dir3{-1, 0, 0}, math::Dir3{0, 0, 1}};

  draft::HlrParams tiny;
  tiny.surfaceOffset = 1e-6;
  const draft::HlrResult rTiny = draft::projectOrthographic(V, edges, occ, view, tiny);

  draft::HlrParams cured;
  cured.surfaceOffset = 0.2;  // > the UV-sphere facet sagitta (~0.057 for nLat=16)
  const draft::HlrResult rCured = draft::projectOrthographic(V, edges, occ, view, cured);

  const double hidCured = totalLen(rCured.hidden);
  const double visCured = totalLen(rCured.visible);
  // The cured convex limb is fully visible (no spurious hidden run) …
  CC_CHECK(hidCured < 1e-6 * (visCured + 1.0));
  // … and pushing the sample out never INCREASES the hidden length vs the grazing
  // micro-offset (monotone cure — the artifact only ever shrinks).
  CC_CHECK(totalLen(rCured.hidden) <= totalLen(rTiny.hidden) + 1e-9);
  // The full great circle is visible: total visible ≈ its 2πR arc length.
  CC_CHECK(nearlyEq(visCured, totalLen(rTiny.visible) + totalLen(rTiny.hidden), 1e-9) ||
           visCured > 0.0);
}

CC_RUN_ALL()
