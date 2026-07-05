// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native WRAP-EMBOSS slice (Phase 4, capability #7
// `native-wrap-emboss`). OCCT-FREE — Gate 1 (host, analytic) of the two-gate model:
// the native builder compiles and unit-tests with clang++ -std=c++20, no OCCT, no
// simulator, no cc_* facade.
//
// The builder is exercised DIRECTLY on a native capped cylinder and validated the way
// the ENGINE does: native TESSELLATOR watertightness + a volume GROWN by the analytic
// wrapped-footprint area × height (emboss RAISES a pad). A result that fails these is one
// the engine DISCARDS and falls through to OCCT, so the tests assert the HONEST
// native/fallthrough split (deboss / non-rectangular / non-cylindrical / off-end → NULL).
//
// Build (standalone):
//   /opt/homebrew/opt/llvm/bin/clang++ -std=c++20 tests/native/test_native_wrap_emboss.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_wrap_emboss && /tmp/test_native_wrap_emboss
//
#include "native/construct/native_construct.h"
#include "native/feature/wrap_emboss.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace feat = cybercad::native::feature;

namespace {

// A capped solid cylinder: full-circle profile radius Rc extruded to height h. Faces:
// bottom cap (−Z), top cap (+Z), one Cylinder wall. Axis +Z, z ∈ [0,h].
topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;  // full circle
  seg.cx = 0;
  seg.cy = 0;
  seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}

// The 1-based id of the single Cylinder lateral face.
int cylFaceId(const topo::Shape& s) {
  const topo::ShapeMap m = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= m.size(); ++i) {
    const auto surf = topo::surfaceOf(m.shape(static_cast<int>(i)));
    if (surf && surf->surface->kind == topo::FaceSurface::Kind::Cylinder) return static_cast<int>(i);
  }
  return 0;
}

// The 1-based id of a planar cap face (for the negative "not a cylinder" test).
int capFaceId(const topo::Shape& s) {
  const topo::ShapeMap m = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= m.size(); ++i) {
    const auto surf = topo::surfaceOf(m.shape(static_cast<int>(i)));
    if (surf && surf->surface->kind == topo::FaceSurface::Kind::Plane) return static_cast<int>(i);
  }
  return 0;
}

// Watertight enclosed volume at a fine deflection; sets `wt`.
double vol(const topo::Shape& s, bool& wt) {
  if (s.isNull()) { wt = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.005;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

// An axis-aligned rectangle profile in (px,py): arc-length width aw, axial height ah,
// centred at (0,0). 4 corners CCW.
std::vector<double> rect(double aw, double ah) {
  return {-aw / 2, -ah / 2, aw / 2, -ah / 2, aw / 2, ah / 2, -aw / 2, ah / 2};
}

}  // namespace

CC_TEST(wrap_emboss_rectangular_pad_watertight_volume_grown) {
  // Rc=10, h=20 cylinder; emboss a 6(arc)×8(axial) rectangular pad, height 2. Watertight,
  // volume GROWN by ≈ footprint area × height (48 × 2 = 96), to the deflection bound.
  const double Rc = 10.0, h = 20.0, height = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int fid = cylFaceId(cyl);
  CC_CHECK(fid != 0);
  const std::vector<double> prof = rect(6.0, 8.0);
  topo::Shape e = feat::wrap_emboss(cyl, fid, prof.data(), 4, height, 1, 0.01);
  bool wt = false;
  const double v = vol(e, wt);
  CC_CHECK(!e.isNull());
  CC_CHECK(wt);       // pad walls + outer cap + windowed base wall weld watertight
  CC_CHECK(v > v0);   // an emboss (boss) GROWS the volume
  // Expected growth = wrapped footprint area × height. Because px is already arc-length,
  // the wrapped area equals the flat profile area 6×8 = 48 (independent of Rc).
  const double expected = v0 + 6.0 * 8.0 * height;
  CC_CHECK(std::fabs(v - expected) <= 1e-2 * expected);  // deflection-bounded facet mesh
}

CC_TEST(wrap_emboss_area_independent_of_radius) {
  // The wrapped footprint area is arc-length × axial, independent of Rc, so the volume
  // growth is the same on two different-radius cylinders for the same profile+height.
  const double height = 1.5;
  const std::vector<double> prof = rect(4.0, 5.0);
  const double growWant = 4.0 * 5.0 * height;
  for (double Rc : {5.0, 8.0, 12.0}) {
    topo::Shape cyl = cappedCylinder(Rc, 20.0);
    bool wt0 = false;
    const double v0 = vol(cyl, wt0);
    const int fid = cylFaceId(cyl);
    topo::Shape e = feat::wrap_emboss(cyl, fid, prof.data(), 4, height, 1, 0.01);
    bool wt = false;
    const double v = vol(e, wt);
    CC_CHECK(!e.isNull());
    CC_CHECK(wt);
    CC_CHECK(std::fabs((v - v0) - growWant) <= 1e-2 * (v0 + growWant));
  }
}

CC_TEST(wrap_emboss_scope_defers) {
  const double Rc = 10.0, h = 20.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int fid = cylFaceId(cyl);
  const int cap = capFaceId(cyl);
  const std::vector<double> prof = rect(6.0, 8.0);

  // Deboss (boss=0) → NULL (this slice only raises a pad).
  CC_CHECK(feat::wrap_emboss(cyl, fid, prof.data(), 4, 2.0, 0, 0.01).isNull());
  // A planar cap face is not a cylinder lateral face → NULL.
  CC_CHECK(feat::wrap_emboss(cyl, cap, prof.data(), 4, 2.0, 1, 0.01).isNull());
  // A 3-corner (triangle) profile is not a rectangle → NULL.
  const std::vector<double> tri = {-3, -4, 3, -4, 0, 4};
  CC_CHECK(feat::wrap_emboss(cyl, fid, tri.data(), 3, 2.0, 1, 0.01).isNull());
  // A slanted 4-corner loop (not axis-aligned in (px,py)) → NULL.
  const std::vector<double> slant = {-3, -4, 3, -2, 3, 4, -3, 2};
  CC_CHECK(feat::wrap_emboss(cyl, fid, slant.data(), 4, 2.0, 1, 0.01).isNull());
  // A footprint whose axial span (30) runs off the wall (h=20) → NULL.
  const std::vector<double> tall = rect(6.0, 30.0);
  CC_CHECK(feat::wrap_emboss(cyl, fid, tall.data(), 4, 2.0, 1, 0.01).isNull());
  // An arc span ≥ full turn (width 80, Rc=10 → 8 rad > 2π) → NULL.
  const std::vector<double> wide = rect(80.0, 8.0);
  CC_CHECK(feat::wrap_emboss(cyl, fid, wide.data(), 4, 2.0, 1, 0.01).isNull());
  // Non-positive height → NULL.
  CC_CHECK(feat::wrap_emboss(cyl, fid, prof.data(), 4, 0.0, 1, 0.01).isNull());
  CC_CHECK(feat::wrap_emboss(cyl, fid, prof.data(), 4, -2.0, 1, 0.01).isNull());
}

CC_TEST(wrap_emboss_offcentre_footprint) {
  // A footprint offset in py (not straddling u=0 symmetrically) still welds watertight
  // and grows by the same area — exercises the anchored-at-uMin angular grid + the axial
  // window sitting off the wall middle.
  const double Rc = 8.0, h = 24.0, height = 2.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int fid = cylFaceId(cyl);
  // arc-length px in [1, 7] (offset from 0), axial py in [-2, 6].
  const std::vector<double> prof = {1, -2, 7, -2, 7, 6, 1, 6};
  topo::Shape e = feat::wrap_emboss(cyl, fid, prof.data(), 4, height, 1, 0.01);
  bool wt = false;
  const double v = vol(e, wt);
  CC_CHECK(!e.isNull());
  CC_CHECK(wt);
  const double expected = v0 + 6.0 * 8.0 * height;  // (7−1)×(6−(−2)) = 48
  CC_CHECK(std::fabs(v - expected) <= 1e-2 * expected);
}

CC_RUN_ALL();
