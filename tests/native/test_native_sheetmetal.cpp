// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native SHEET-METAL library (MOAT M-SM, first slice).
// OCCT-FREE — this is Gate (a) (host, closed-form) of the two-gate model. OCCT core
// has NO sheet-metal module, so there is NO OCCT oracle: the ARBITER is CLOSED FORM.
// The suite exercises the native builders DIRECTLY (sheetmetal::baseFlange /
// edgeFlange / unfold → topology::Shape) and validates each result with the native
// TESSELLATOR (watertight / χ=2 / consistently oriented / enclosed volume) against
// the analytic closed form. Nothing here links OCCT.
//
// WHAT IS ASSERTED (matches the delivered first slice, see sheetmetal.h):
//   * base flange of a rectangle → watertight, χ=2, oriented, volume = area·thickness;
//   * a single edge flange (cylindrical bend + planar wall) off the straight +X rim →
//     watertight, χ=2, oriented, volume = base + ½θ((r+t)²−r²)·W + h·t·W (bend meshes
//     to deflection, so the volume converges from below within a deflection-set band);
//   * the UNFOLD develops to devLength = baseRun + BA + h with BA = θ(r + k·t), the
//     footprint area = baseArea + BA·W + flangeArea, and fold→unfold is AREA-INVARIANT;
//   * the honest declines (bad parameter / non-straight bend line / non-recognised
//     base / self-collision) return a NULL Shape with a MEASURED reason — never a wrong
//     or self-intersecting solid.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_sheetmetal.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o test_native_sheetmetal
//
#include "native/sheetmetal/sheetmetal.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace sm   = cybercad::native::sheetmetal;
namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace m    = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Mesh audit bundle: watertight, consistently oriented, χ=2, and the enclosed volume.
struct Audit {
  bool watertight = false;
  bool oriented = false;
  long chi = 0;
  double volume = 0.0;
};
Audit audit(const topo::Shape& s, double defl = 0.005) {
  tess::MeshParams mp;
  mp.deflection = defl;
  const tess::Mesh mesh = tess::SolidMesher(mp).mesh(s);
  Audit a;
  a.watertight = tess::isWatertight(mesh);
  a.oriented = tess::isConsistentlyOriented(mesh);
  a.chi = cybercad::native::directmodel::rfdetail::eulerChar(mesh);
  a.volume = std::fabs(tess::enclosedVolume(mesh));
  return a;
}

// Resolve the flangeable straight +X rim edge id of a rectangular base flange.
int findRim(const topo::Shape& base, double L, double W, double t) {
  const topo::ShapeMap map = topo::mapShapes(base, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= map.size(); ++i)
    if (sm::efdetail::isFlangeableRim(base, static_cast<int>(i), sm::BasePlate{L, W, t}))
      return static_cast<int>(i);
  return -1;
}

}  // namespace

// ── Base flange: a flat sheet = profile · thickness ─────────────────────────────
CC_TEST(base_flange_rectangle_volume) {
  const double rect[8] = {0, 0, 40, 0, 40, 20, 0, 20};  // 40×20
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape base = sm::baseFlange(rect, 4, 2.0, &why);
  CC_CHECK(!base.isNull());
  CC_CHECK(why == sm::SheetMetalDecline::Ok);
  const Audit a = audit(base);
  CC_CHECK(a.watertight);
  CC_CHECK(a.oriented);
  CC_CHECK_EQ(a.chi, 2L);
  CC_CHECK(std::fabs(a.volume - 40.0 * 20.0 * 2.0) < 1e-6);  // exact (planar)
}

CC_TEST(base_flange_L_profile_volume) {
  // An L-shaped (concave rectilinear) footprint: area = 30·10 + 10·10 = 400.
  const double lshape[12] = {0, 0, 30, 0, 30, 10, 10, 10, 10, 20, 0, 20};
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape base = sm::baseFlange(lshape, 6, 1.5, &why);
  CC_CHECK(!base.isNull());
  const Audit a = audit(base);
  CC_CHECK(a.watertight);
  CC_CHECK_EQ(a.chi, 2L);
  CC_CHECK(std::fabs(a.volume - 400.0 * 1.5) < 1e-6);
}

CC_TEST(base_flange_declines_degenerate) {
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const double tri[6] = {0, 0, 10, 0, 5, 5};
  // thickness ≤ 0 → BadThickness.
  CC_CHECK(sm::baseFlange(tri, 3, 0.0, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::BadThickness);
  // < 3 points → BadProfile.
  CC_CHECK(sm::baseFlange(tri, 2, 1.0, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::BadProfile);
}

// ── Edge flange: base + cylindrical bend + planar wall, one watertight solid ────
CC_TEST(edge_flange_90deg_closed_form_volume) {
  const double L = 40, W = 20, t = 2, r = 3, h = 15, th = kPi / 2;
  const double rect[8] = {0, 0, L, 0, L, W, 0, W};
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape base = sm::baseFlange(rect, 4, t, &why);
  const int rim = findRim(base, L, W, t);
  CC_CHECK(rim > 0);

  sm::FoldRecord fold{};
  const topo::Shape folded = sm::edgeFlange(base, rim, h, r, th, &why, &fold);
  CC_CHECK(!folded.isNull());
  CC_CHECK(why == sm::SheetMetalDecline::Ok);
  CC_CHECK(fold.valid);

  const double ro = r + t;
  const double expected = L * W * t + 0.5 * th * (ro * ro - r * r) * W + h * t * W;
  const Audit a = audit(folded);
  CC_CHECK(a.watertight);
  CC_CHECK(a.oriented);
  CC_CHECK_EQ(a.chi, 2L);
  // Converges from below; a fine-deflection bend lands well inside 1%.
  CC_CHECK(a.volume <= expected + 1e-6);
  CC_CHECK(std::fabs(a.volume - expected) < 0.01 * expected);
}

CC_TEST(edge_flange_variants_build) {
  const double L = 30, W = 12, t = 1.5;
  const double rect[8] = {0, 0, L, 0, L, W, 0, W};
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape base = sm::baseFlange(rect, 4, t, &why);
  const int rim = findRim(base, L, W, t);
  CC_CHECK(rim > 0);
  // A sharp (r=0) bend, a tight r=1 bend, and a 120° bend all build watertight.
  for (auto [r, th] : std::vector<std::pair<double, double>>{
           {0.0, kPi / 2}, {1.0, kPi / 2}, {3.0, 2.0 * kPi / 3.0}}) {
    const topo::Shape f = sm::edgeFlange(base, rim, 8.0, r, th, &why);
    CC_CHECK(!f.isNull());
    const Audit a = audit(f);
    CC_CHECK(a.watertight);
    CC_CHECK_EQ(a.chi, 2L);
  }
}

CC_TEST(edge_flange_honest_declines) {
  const double L = 40, W = 20, t = 2;
  const double rect[8] = {0, 0, L, 0, L, W, 0, W};
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape base = sm::baseFlange(rect, 4, t, &why);
  const int rim = findRim(base, L, W, t);
  CC_CHECK(rim > 0);

  // angle = 0 → BadParam.
  CC_CHECK(sm::edgeFlange(base, rim, 15, 3, 0.0, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::BadParam);
  // negative radius → BadParam.
  CC_CHECK(sm::edgeFlange(base, rim, 15, -1, kPi / 2, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::BadParam);
  // edge id out of range → EdgeNotFound.
  CC_CHECK(sm::edgeFlange(base, 9999, 15, 3, kPi / 2, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::EdgeNotFound);
  // a non-rim edge (base bottom, id 1) → NotSingleBendPart (recognised, not the rim).
  CC_CHECK(sm::edgeFlange(base, 1, 15, 3, kPi / 2, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::NotSingleBendPart);
}

// ── Unfold: flat pattern, k-factor bend allowance, area invariant ───────────────
CC_TEST(unfold_bend_allowance_and_area) {
  const double L = 40, W = 20, t = 2, r = 3, h = 15, th = kPi / 2, k = 0.44;
  sm::FlatPattern fp{};
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape blank = sm::unfold(L, W, t, r, th, h, k, &fp, &why);
  CC_CHECK(!blank.isNull());
  CC_CHECK(why == sm::SheetMetalDecline::Ok);

  const double BA = th * (r + k * t);
  CC_CHECK(std::fabs(fp.bendAllowance - BA) < 1e-9);
  CC_CHECK(std::fabs(fp.devLength - (L + BA + h)) < 1e-9);
  // Area invariant: developed footprint = base + bend developed × W + flange.
  const double expectedArea = L * W + BA * W + h * W;
  CC_CHECK(std::fabs(fp.area - expectedArea) < 1e-9);
  // The blank is a flat sheet of that footprint (planar ⇒ mesh exact).
  const Audit a = audit(blank);
  CC_CHECK(a.watertight);
  CC_CHECK_EQ(a.chi, 2L);
  CC_CHECK(std::fabs(a.volume - expectedArea * t) < 1e-6);
}

CC_TEST(fold_then_unfold_area_invariant) {
  // Build a fold, recover its FoldRecord, unfold it, and assert the developed area is
  // exactly the closed-form sum — the fold→unfold round-trip invariant.
  const double L = 50, W = 25, t = 1.0, r = 2.0, h = 20.0, th = kPi / 2, k = 0.4;
  const double rect[8] = {0, 0, L, 0, L, W, 0, W};
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape base = sm::baseFlange(rect, 4, t, &why);
  const int rim = findRim(base, L, W, t);
  CC_CHECK(rim > 0);
  sm::FoldRecord fold{};
  const topo::Shape folded = sm::edgeFlange(base, rim, h, r, th, &why, &fold);
  CC_CHECK(!folded.isNull());
  CC_CHECK(fold.valid);

  sm::FlatPattern fp{};
  const topo::Shape blank = sm::unfold(fold, k, &fp, &why);
  CC_CHECK(!blank.isNull());
  const double BA = th * (r + k * t);
  const double expectedArea = L * W + BA * W + h * W;
  CC_CHECK(std::fabs(fp.area - expectedArea) < 1e-9);
  // The flat blank encloses exactly footprint·thickness (planar, exact).
  const Audit a = audit(blank);
  CC_CHECK(std::fabs(a.volume - expectedArea * t) < 1e-6);
}

CC_TEST(unfold_declines_invalid_fold) {
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  // An unrecognised (invalid) fold record → NotSingleBendPart.
  const sm::FoldRecord bad{};
  CC_CHECK(sm::unfold(bad, 0.44, nullptr, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::NotSingleBendPart);
  // kFactor out of [0,1] on a valid record → BadParam.
  const sm::FoldRecord good{true, 40, 20, 2, 3, kPi / 2, 15};
  CC_CHECK(sm::unfold(good, 1.5, nullptr, &why).isNull());
  CC_CHECK(why == sm::SheetMetalDecline::BadParam);
}

CC_RUN_ALL();
