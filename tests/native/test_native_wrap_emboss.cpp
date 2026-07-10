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

// A regular hexagon profile (centre-to-vertex a) centred at (0,0), CCW. Shoelace area
// = 3√3/2 · a².
std::vector<double> hexagon(double a) {
  const double s = a * 0.8660254037844386;  // a·sin60
  const double h = a * 0.5;                  // a·cos60
  return {a, 0, h, s, -h, s, -a, 0, -h, -s, h, -s};
}

// A SPHERE-CAP dome about the +Y axis (like the curved_shell test): one coaxial Sphere
// wall (radius R, centre at the origin) closed at the pole (0,R), cut by ONE axis-normal cap
// plane at y=capOff. Meridian: base disc (0,capOff)→(rimBase,capOff), then arc to (0,R).
topo::Shape sphereCapDome(double R, double capOff) {
  const double rimBase = std::sqrt(R * R - capOff * capOff);
  cst::ProfileSegment base;
  base.kind = 0;
  base.x0 = 0; base.y0 = capOff; base.x1 = rimBase; base.y1 = capOff;
  cst::ProfileSegment arc;
  arc.kind = 1;
  arc.x0 = rimBase; arc.y0 = capOff; arc.x1 = 0; arc.y1 = R; arc.cx = 0; arc.cy = 0; arc.r = R;
  return cst::build_revolution_profile({base, arc}, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}

// The 1-based id of the (first) Sphere wall face.
int sphereFaceId(const topo::Shape& s) {
  const topo::ShapeMap m = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= m.size(); ++i) {
    const auto surf = topo::surfaceOf(m.shape(static_cast<int>(i)));
    if (surf && surf->surface->kind == topo::FaceSurface::Kind::Sphere) return static_cast<int>(i);
  }
  return 0;
}

// The exact shell-sector volume delta of a pole boss: 2π(1−cosφ0)·((R+h)³−R³)/3.
double poleBossDelta(double R, double h, double phi0) {
  return 2.0 * M_PI * (1.0 - std::cos(phi0)) * (std::pow(R + h, 3) - std::pow(R, 3)) / 3.0;
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

CC_TEST(wrap_emboss_deboss_rectangular_pocket_watertight_volume_reduced) {
  // T1 — a recessed rectangular pocket (boss=0): watertight, volume SHRUNK by ≈ footprint
  // area × depth (48 × 2 = 96) to the deflection bound. The MIRROR of the raised pad.
  const double Rc = 10.0, h = 20.0, depth = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int fid = cylFaceId(cyl);
  const std::vector<double> prof = rect(6.0, 8.0);
  topo::Shape e = feat::wrap_emboss(cyl, fid, prof.data(), 4, depth, 0, 0.01);
  bool wt = false;
  const double v = vol(e, wt);
  CC_CHECK(!e.isNull());
  CC_CHECK(wt);        // pocket floor + inward walls + windowed base wall weld watertight
  CC_CHECK(v < v0);    // a deboss (cut) SHRINKS the volume
  const double expected = v0 - 6.0 * 8.0 * depth;
  CC_CHECK(std::fabs(v - expected) <= 1e-2 * expected);
}

CC_TEST(wrap_emboss_hexagon_pad_watertight_volume_grown) {
  // T2 — a raised regular-hexagon pad (a=5): watertight, volume GROWN by ≈ shoelace area
  // (3√3/2·25 = 64.9519) × height. Exercises the non-rectangular polygon footprint.
  const double Rc = 10.0, h = 20.0, height = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int fid = cylFaceId(cyl);
  const std::vector<double> hexp = hexagon(5.0);
  topo::Shape e = feat::wrap_emboss(cyl, fid, hexp.data(), 6, height, 1, 0.01);
  bool wt = false;
  const double v = vol(e, wt);
  CC_CHECK(!e.isNull());
  CC_CHECK(wt);
  CC_CHECK(v > v0);
  const double area = 3.0 * std::sqrt(3.0) / 2.0 * 25.0;  // 64.9519
  const double expected = v0 + area * height;
  CC_CHECK(std::fabs(v - expected) <= 1e-2 * expected);
}

CC_TEST(wrap_emboss_hexagon_pocket_watertight_volume_reduced) {
  // T2 + T1 — a recessed regular-hexagon pocket (boss=0): watertight, volume SHRUNK.
  const double Rc = 10.0, h = 20.0, depth = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int fid = cylFaceId(cyl);
  const std::vector<double> hexp = hexagon(5.0);
  topo::Shape e = feat::wrap_emboss(cyl, fid, hexp.data(), 6, depth, 0, 0.01);
  bool wt = false;
  const double v = vol(e, wt);
  CC_CHECK(!e.isNull());
  CC_CHECK(wt);
  CC_CHECK(v < v0);
  const double area = 3.0 * std::sqrt(3.0) / 2.0 * 25.0;
  const double expected = v0 - area * depth;
  CC_CHECK(std::fabs(v - expected) <= 1e-2 * expected);
}

CC_TEST(wrap_emboss_scope_defers) {
  const double Rc = 10.0, h = 20.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int fid = cylFaceId(cyl);
  const int cap = capFaceId(cyl);
  const std::vector<double> prof = rect(6.0, 8.0);

  // A planar cap face is not a cylinder lateral face → NULL (T3 freeform base declines).
  CC_CHECK(feat::wrap_emboss(cyl, cap, prof.data(), 4, 2.0, 1, 0.01).isNull());
  // A footprint whose axial span (30) runs off the wall (h=20) → NULL.
  const std::vector<double> tall = rect(6.0, 30.0);
  CC_CHECK(feat::wrap_emboss(cyl, fid, tall.data(), 4, 2.0, 1, 0.01).isNull());
  // An arc span ≥ full turn (width 80, Rc=10 → 8 rad > 2π) → NULL.
  const std::vector<double> wide = rect(80.0, 8.0);
  CC_CHECK(feat::wrap_emboss(cyl, fid, wide.data(), 4, 2.0, 1, 0.01).isNull());
  // A deboss depth ≥ the radius (12 > 10) → NULL.
  CC_CHECK(feat::wrap_emboss(cyl, fid, prof.data(), 4, 12.0, 0, 0.01).isNull());
  // A self-intersecting (pentagram) 5-corner loop → NULL. Five outer points traversed in
  // star order so non-adjacent edges cross; not a bbox rectangle, so it reaches the
  // polygon path's simple-loop guard.
  const std::vector<double> star = {0, 5, 2.939, -4.045, -4.755, 1.545, 4.755, 1.545, -2.939, -4.045};
  CC_CHECK(feat::wrap_emboss(cyl, fid, star.data(), 5, 2.0, 1, 0.01).isNull());
  // A degenerate 2-point profile → NULL.
  const std::vector<double> two = {-3, -4, 3, 4};
  CC_CHECK(feat::wrap_emboss(cyl, fid, two.data(), 2, 2.0, 1, 0.01).isNull());
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

// ── F5 FREEFORM (curved) base: sphere-cap pole boss ─────────────────────────────────

CC_TEST(wrap_emboss_sphere_pole_boss_watertight_closed_form) {
  // A hemisphere dome (R=10, cap at y=0). Emboss a circular pole boss whose profile in-radius
  // ρ=3 (a 6×6 square in arc-length) → φ0=ρ/R=0.3 rad, raised height=2. Watertight, volume
  // GROWN by the EXACT spherical-shell-sector delta 2π(1−cosφ0)·((R+h)³−R³)/3.
  const double R = 10.0, height = 2.0;
  topo::Shape dome = sphereCapDome(R, 0.0);
  bool wt0 = false;
  const double v0 = vol(dome, wt0);
  CC_CHECK(wt0);
  const int fid = sphereFaceId(dome);
  CC_CHECK(fid != 0);
  const std::vector<double> prof = rect(6.0, 6.0);  // in-radius ρ = 3
  topo::Shape e = feat::wrap_emboss(dome, fid, prof.data(), 4, height, 1, 0.005);
  bool wt = false;
  const double v = vol(e, wt);
  CC_CHECK(!e.isNull());
  CC_CHECK(wt);
  CC_CHECK(v > v0);  // a raised pole boss GROWS the volume
  const double expected = v0 + poleBossDelta(R, height, 3.0 / R);
  CC_CHECK(std::fabs(v - expected) <= 1.5e-2 * expected);  // deflection-bounded curved mesh
}

CC_TEST(wrap_emboss_sphere_pole_boss_offcenter_caps_closed_form) {
  // The pole/cap orientation must be resolved GEOMETRICALLY, not from the revolve cap's
  // frame normal (which can point inward). A DEEP dome (cap below centre, φcap>π/2) and a
  // SHALLOW cap (cap above centre, φcap<π/2) both emboss to base + the exact shell sector —
  // this locks the pole-direction bug that a centre-cut (capOff=0) hemisphere masks.
  struct Case { double R, capOff, rho, height; };
  const Case cases[] = {{12.0, -2.0, 2.5, 1.5}, {8.0, 1.0, 2.0, 1.0}};
  for (const Case& cs : cases) {
    topo::Shape dome = sphereCapDome(cs.R, cs.capOff);
    bool wt0 = false;
    const double v0 = vol(dome, wt0);
    CC_CHECK(wt0);
    const int fid = sphereFaceId(dome);
    const std::vector<double> prof = rect(2.0 * cs.rho, 2.0 * cs.rho);
    topo::Shape e = feat::wrap_emboss(dome, fid, prof.data(), 4, cs.height, 1, 0.005);
    bool wt = false;
    const double v = vol(e, wt);
    CC_CHECK(!e.isNull());
    CC_CHECK(wt);
    CC_CHECK(v > v0);
    const double expected = v0 + poleBossDelta(cs.R, cs.height, cs.rho / cs.R);
    CC_CHECK(std::fabs(v - expected) <= 1.5e-2 * expected);
  }
}

CC_TEST(wrap_emboss_sphere_pole_boss_delta_helper_matches) {
  // The exposed closed-form helper equals the independent formula and is R-consistent.
  const double R = 12.0, height = 1.5;
  topo::Shape dome = sphereCapDome(R, -2.0);  // a deep dome (cap below centre)
  const int fid = sphereFaceId(dome);
  const std::vector<double> prof = rect(5.0, 5.0);  // ρ = 2.5
  const double d = feat::spherePoleBossVolumeDelta(dome, fid, prof.data(), 4, height);
  CC_CHECK(d > 0.0);
  CC_CHECK(std::fabs(d - poleBossDelta(R, height, 2.5 / R)) <= 1e-9 * d);
}

CC_TEST(wrap_emboss_sphere_scope_defers) {
  const double R = 10.0;
  topo::Shape dome = sphereCapDome(R, 0.0);
  const int fid = sphereFaceId(dome);
  // A DEBOSS pole cap on a sphere is out of scope → NULL (only raised boss is native).
  const std::vector<double> prof = rect(6.0, 6.0);
  CC_CHECK(feat::wrap_emboss(dome, fid, prof.data(), 4, 2.0, 0, 0.01).isNull());
  // A footprint whose pole cap reaches the rim (ρ ≈ R·π/2 → φ0 ≥ φcap) → NULL.
  const std::vector<double> huge = rect(40.0, 40.0);  // ρ=20 → φ0=2 rad > π/2
  CC_CHECK(feat::wrap_emboss(dome, fid, huge.data(), 4, 2.0, 1, 0.01).isNull());
  // The delta helper returns 0 for an out-of-scope boss (rim-reaching φ0).
  CC_CHECK(feat::spherePoleBossVolumeDelta(dome, fid, huge.data(), 4, 2.0) == 0.0);
  // A spherical ZONE (two cap planes) is not a pure sphere-cap dome → NULL.
  // (Built as a band: caps at y=−3 and y=+3.) The recogniser rejects two distinct caps.
  cst::ProfileSegment b0; b0.kind = 0; b0.x0 = 0; b0.y0 = -3; b0.x1 = std::sqrt(R*R-9); b0.y1 = -3;
  cst::ProfileSegment ar; ar.kind = 1; ar.x0 = std::sqrt(R*R-9); ar.y0 = -3; ar.x1 = std::sqrt(R*R-9); ar.y1 = 3; ar.cx = 0; ar.cy = 0; ar.r = R;
  cst::ProfileSegment t0; t0.kind = 0; t0.x0 = std::sqrt(R*R-9); t0.y0 = 3; t0.x1 = 0; t0.y1 = 3;
  topo::Shape zone = cst::build_revolution_profile({b0, ar, t0}, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int zf = sphereFaceId(zone);
  if (zf != 0)
    CC_CHECK(feat::wrap_emboss(zone, zf, prof.data(), 4, 2.0, 1, 0.01).isNull());
}

CC_RUN_ALL();
