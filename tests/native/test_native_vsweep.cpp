// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native VARIABLE-SECTION / guide+spine sweep (moat-vsweep):
// cc_variable_sweep → construct/sweep.h build_variable_sweep. OCCT-FREE — Gate 1 (host,
// analytic / closed-form) of the two-gate model. A section MORPHS from profile A (spine
// start) to profile B (spine end) along the spine, each station = interpolate(A,B,f) placed
// by the perpendicular (straight) / RMF (curved) frame, OPTIONALLY guide-SCALED.
//
// The analytic oracle where a closed form exists:
//   * circle(r0)→circle(r1) morph along a STRAIGHT spine of length H = a TRUNCATED CONE
//     (frustum), volume V = πH/3·(r0² + r0·r1 + r1²) (a polygon-approximated circle
//     converges to this as the vertex count grows — measured as a relative-error bound).
//   * a CONSTANT section (A == B, no guide) = the plain ruled sweep (a prism of area·H).
//   * a guide-SCALED constant square section along a straight spine, guide splaying
//     linearly from d0 to k·d0, is a square frustum with the known prismatoid volume.
// Each result is validated with the native tessellator: watertight, consistently oriented,
// Euler characteristic χ = V−E+F = 2 (genus-0), and the closed-form volume. Multi-station
// convergence is asserted (a finer curved spine stays watertight, volume stable). The hard
// declines (mismatched section counts, self-folding morph, coincident guide) return a NULL
// Shape so the engine falls through to OCCT.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_vsweep.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_vsweep
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <set>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace m = cybercad::native::math;

namespace {
constexpr double kPi = 3.14159265358979323846;

int countSub(const topo::Shape& shape, topo::ShapeType type) {
  int n = 0;
  for (topo::Explorer ex(shape, type); ex.more(); ex.next()) ++n;
  return n;
}

// A regular N-gon of radius r centred at the origin, as flat (x,y) pairs. Its area is
// (N/2)·r²·sin(2π/N), which → πr² as N grows.
std::vector<double> circlePoly(double r, int n) {
  std::vector<double> p;
  p.reserve(static_cast<std::size_t>(n) * 2);
  for (int i = 0; i < n; ++i) {
    const double th = 2.0 * kPi * i / n;
    p.push_back(r * std::cos(th));
    p.push_back(r * std::sin(th));
  }
  return p;
}
double polyArea(double r, int n) { return 0.5 * n * r * r * std::sin(2.0 * kPi / n); }

// Weld coincident vertices (the mesher duplicates per-face seam vertices) and return the
// Euler characteristic χ = V − E + F of the welded closed triangle mesh. For a genus-0
// watertight consistently-oriented solid χ == 2.
int eulerChar(const tess::Mesh& mesh) {
  auto key = [](const m::Point3& p) {
    auto q = [](double v) { return static_cast<long long>(std::llround(v * 1e6)); };
    return std::make_tuple(q(p.x), q(p.y), q(p.z));
  };
  std::map<std::tuple<long long, long long, long long>, int> weld;
  std::vector<int> remap(mesh.vertices.size());
  for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
    auto k = key(mesh.vertices[i]);
    auto it = weld.find(k);
    if (it == weld.end()) it = weld.emplace(k, static_cast<int>(weld.size())).first;
    remap[i] = it->second;
  }
  const int V = static_cast<int>(weld.size());
  std::set<std::pair<int, int>> edges;
  for (const auto& t : mesh.triangles) {
    int a = remap[t.a], b = remap[t.b], c = remap[t.c];
    auto add = [&](int x, int y) { edges.insert({std::min(x, y), std::max(x, y)}); };
    add(a, b);
    add(b, c);
    add(c, a);
  }
  const int E = static_cast<int>(edges.size());
  const int F = static_cast<int>(mesh.triangles.size());
  return V - E + F;
}

// Validate a candidate variable-sweep solid: watertight + consistently oriented + χ==2 at
// a small deflection, returning the enclosed volume.
bool validSolid(const topo::Shape& s, double defl, double& vol) {
  if (s.isNull()) return false;
  tess::MeshParams p;
  p.deflection = defl;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(s);
  vol = std::fabs(tess::enclosedVolume(mesh));
  return tess::isWatertight(mesh) && tess::isConsistentlyOriented(mesh) && eulerChar(mesh) == 2;
}
}  // namespace

// ── ANALYTIC: circle(r0)→circle(r1) morph along a straight spine = truncated cone ──
// r0 = 5, r1 = 2, straight spine length H = 12 along +Z, no guide. The frustum volume is
// V = πH/3·(r0²+r0·r1+r1²). The polygon-approximated circle converges to this; with the
// polygon AREA substituted for πr² the match is exact for a linear (frustum) morph, so we
// use the prismatoid volume with the exact polygon areas as the tight oracle and also check
// the smooth-circle frustum within a coarse-polygon bound.
CC_TEST(vsweep_circle_to_circle_straight_is_truncated_cone) {
  const int N = 96;
  const double r0 = 5.0, r1 = 2.0, H = 12.0;
  const std::vector<double> A = circlePoly(r0, N);
  const std::vector<double> B = circlePoly(r1, N);
  const double spine[] = {0, 0, 0, 0, 0, H};
  const topo::Shape s = cst::build_variable_sweep(A.data(), N, B.data(), N, spine, 2, nullptr, 0);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;

  double vol = 0.0;
  CC_CHECK(validSolid(s, 0.02, vol));

  // Tight oracle: prismatoid volume of the linear morph between the two polygon caps,
  // H/3·(A0 + A1 + √(A0·A1)) with A0/A1 the exact polygon areas — the frustum a linear
  // ruled morph produces exactly.
  const double A0 = polyArea(r0, N), A1 = polyArea(r1, N);
  const double vExactPoly = H / 3.0 * (A0 + A1 + std::sqrt(A0 * A1));
  CC_CHECK(std::fabs(vol - vExactPoly) / vExactPoly < 1e-4);

  // Smooth-circle frustum closed form, matched within the N=96 polygon-vs-circle bound.
  const double vCone = kPi * H / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
  CC_CHECK(std::fabs(vol - vCone) / vCone < 2e-3);
}

// ── CONSTANT section (A==B, no guide) = the plain ruled sweep (a prism) ─────────────
// A 6×6 square (half-extent 3, area 36) morphed to itself along a straight +Z spine of
// length 10 is a 6×6×10 box, volume 360, EXACT. Confirms the no-guide path reduces to
// loft_along_rail / the plain sweep (a constant section is a prism).
CC_TEST(vsweep_constant_section_no_guide_is_prism) {
  const double sq[] = {-3, -3, 3, -3, 3, 3, -3, 3};
  const double spine[] = {0, 0, 0, 0, 0, 10};
  const topo::Shape s = cst::build_variable_sweep(sq, 4, sq, 4, spine, 2, nullptr, 0);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 6);  // 4 sides + 2 caps (a box)
  double vol = 0.0;
  CC_CHECK(validSolid(s, 0.02, vol));
  CC_CHECK(std::fabs(vol - 360.0) < 1e-6);  // 36 area × 10 height
}

// ── GUIDE-SCALED constant square = a square frustum (prismatoid volume) ──────────────
// A 2×2 square (half-extent 1) swept along the +Z spine [0..10], guide a straight line
// offset in +X: guide-start at x=4 (d0=4), guide-end at x=8. The splay scale grows linearly
// 1→2, so the square grows 2×2 → 4×4 — a square frustum. Its volume is the prismatoid
// H/3·(A0 + A1 + √(A0A1)) = 10/3·(4 + 16 + 8) = 280/3.
CC_TEST(vsweep_guide_scaled_square_frustum) {
  const double sq[] = {-1, -1, 1, -1, 1, 1, -1, 1};
  const double spine[] = {0, 0, 0, 0, 0, 10};
  const double guide[] = {4, 0, 0, 8, 0, 10};  // offset in +X, splays 4→8 as z 0→10
  const topo::Shape s = cst::build_variable_sweep(sq, 4, sq, 4, spine, 2, guide, 2);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  double vol = 0.0;
  CC_CHECK(validSolid(s, 0.02, vol));
  const double vExact = 10.0 / 3.0 * (4.0 + 16.0 + std::sqrt(4.0 * 16.0));  // 280/3
  CC_CHECK(std::fabs(vol - vExact) / vExact < 1e-6);
}

// ── MULTI-STATION CONVERGENCE: a smooth-arc spine morph stays watertight, volume stable ─
// A circle(r0)→circle(r1) morph along a smooth quarter-arc spine (radius R = 40 ≫ profile,
// XZ plane). The curved builder densifies to a bounded per-band turn; the solid must be
// watertight + consistently oriented + χ==2 at two deflections, and the volume must be
// close to the Pappus estimate of the mean section area × arc length (a convergence check,
// not an exact closed form on a curved morph).
CC_TEST(vsweep_curved_arc_morph_watertight_stable) {
  const int N = 48;
  const double r0 = 4.0, r1 = 2.0, R = 40.0;
  const std::vector<double> A = circlePoly(r0, N);
  const std::vector<double> B = circlePoly(r1, N);
  std::vector<double> spine;
  const int NS = 16;
  for (int k = 0; k <= NS; ++k) {
    const double th = (kPi / 2.0) * k / NS;
    spine.push_back(R * std::cos(th));
    spine.push_back(0.0);
    spine.push_back(R * std::sin(th));
  }
  const topo::Shape s =
      cst::build_variable_sweep(A.data(), N, B.data(), N, spine.data(), NS + 1, nullptr, 0);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  double v1 = 0.0, v2 = 0.0;
  CC_CHECK(validSolid(s, 0.05, v1));
  CC_CHECK(validSolid(s, 0.02, v2));
  CC_CHECK(std::fabs(v1 - v2) / v2 < 5e-3);  // volume stable across deflections

  // Convergence SANITY (not a closed form on a curved shrinking morph): the volume must be
  // in the ballpark of the mean-section-area × arc length. A shrinking section along an arc
  // deviates from this simple estimate (the wider start sweeps more), so a generous 10%
  // band — the point of this case is watertight+χ=2 stability, checked above.
  const double meanArea = 0.5 * (polyArea(r0, N) + polyArea(r1, N));
  const double arcLen = R * (kPi / 2.0);
  CC_CHECK(std::fabs(v2 - meanArea * arcLen) / (meanArea * arcLen) < 0.10);
}

// ── HONEST DECLINE: mismatched section vertex counts → NULL (OCCT MakePipeShell) ─────
CC_TEST(vsweep_mismatched_counts_declines) {
  const std::vector<double> A = circlePoly(5.0, 12);
  const std::vector<double> B = circlePoly(2.0, 8);  // different count
  const double spine[] = {0, 0, 0, 0, 0, 10};
  const topo::Shape s = cst::build_variable_sweep(A.data(), 12, B.data(), 8, spine, 2, nullptr, 0);
  CC_CHECK(s.isNull());  // ruled morph pairs vertex k→k → declines
}

// ── HONEST DECLINE: coincident guide start (d0 ≈ 0) → NULL ───────────────────────────
CC_TEST(vsweep_coincident_guide_declines) {
  const double sq[] = {-1, -1, 1, -1, 1, 1, -1, 1};
  const double spine[] = {0, 0, 0, 0, 0, 10};
  const double guide[] = {0, 0, 0, 0, 0, 10};  // guide starts ON the spine → d0=0
  const topo::Shape s = cst::build_variable_sweep(sq, 4, sq, 4, spine, 2, guide, 2);
  CC_CHECK(s.isNull());
}

// ── HONEST DECLINE: self-folding morph (a section collapses to a point) → NULL ───────
// A guide that collapses toward the spine at the far end drives the scale to 0 there, a
// non-positive/collapsing section the guard rejects.
CC_TEST(vsweep_collapsing_guide_declines) {
  const double sq[] = {-1, -1, 1, -1, 1, 1, -1, 1};
  const double spine[] = {0, 0, 0, 0, 0, 10};
  const double guide[] = {4, 0, 0, 0, 0, 10};  // splay 4 → 0 (scale 1 → 0)
  const topo::Shape s = cst::build_variable_sweep(sq, 4, sq, 4, spine, 2, guide, 2);
  CC_CHECK(s.isNull());  // scale → 0 station → collapse guard → OCCT
}

// ── DEGENERATE input guards → NULL ───────────────────────────────────────────────────
CC_TEST(vsweep_degenerate_input_declines) {
  const double sq[] = {-1, -1, 1, -1, 1, 1, -1, 1};
  const double spine[] = {0, 0, 0, 0, 0, 10};
  CC_CHECK(cst::build_variable_sweep(nullptr, 4, sq, 4, spine, 2, nullptr, 0).isNull());
  CC_CHECK(cst::build_variable_sweep(sq, 2, sq, 2, spine, 2, nullptr, 0).isNull());  // <3 pts
  CC_CHECK(cst::build_variable_sweep(sq, 4, sq, 4, spine, 1, nullptr, 0).isNull());  // <2 spine
}

CC_RUN_ALL()
