// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native CURVED-SHELL slice (MOAT M3, `moat-m3cs-curved-shell`).
// OCCT-FREE — Gate (a) of the two-gate model: the curved shell compiles and unit-tests
// with clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
//   * curved_shell — hollow a capped CYLINDER or capped CONE FRUSTUM to a uniform wall
//     `t`, opening ONE planar cap. The curved wall offsets inward analytically (cylinder
//     Rc→Rc−t; cone reference radius →−t/cosσ). The wall volume has a CLOSED FORM:
//       cylinder:  V = π·Rc²·H − π·(Rc−t)²·(H−t)          (open one end)
//       frustum:   V = V_outer_frustum − V_inner_frustum  (inner shifted in by t/cosσ,
//                      one end open so the inner runs kept-inner-face → open end).
//
// Each result is validated the way the ENGINE does — native tessellator watertightness
// + a SHRINK volume sign — plus the closed-form analytic oracle, and the out-of-scope
// bodies (stepped shaft, both caps removed, wall picked, thickness too large) DECLINE
// (NULL → OCCT). No tolerance is weakened; a measured decline is a first-class outcome.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_curved_shell.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_curved_shell && /tmp/test_native_curved_shell
//
#include "native/blend/native_blend.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace blend = cybercad::native::blend;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace nmath = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

double vol(const topo::Shape& s, bool& wt) {
  if (s.isNull()) { wt = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.004;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

bool nearRel(double got, double want, double rel = 3e-3, double abs = 1e-6) {
  return std::fabs(got - want) <= std::max(rel * std::fabs(want), abs);
}

// A capped solid cylinder about +Z (radius Rc, height h): bottom cap z=0, top cap z=h,
// one Cylinder wall.
topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;  // full circle
  seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}

// A capped cone frustum about the +Y axis (build_revolution revolves about the profile
// Y axis): base radius Rb at y=0, top radius Rt at y=H. Meridian (0,0)→(Rb,0)→(Rt,H)→(0,H).
topo::Shape cappedFrustum(double Rb, double Rt, double H) {
  const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
  return cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * kPi);
}

// The planar cap face whose plane is normal to `axis` at the point `axis·coord`. A revolve
// splits a cap into several angular sub-faces; any one of them selects that whole cap.
// 0 if none.
int capFaceAt(const topo::Shape& s, const nmath::Vec3& axis, double coord) {
  const nmath::Vec3 az = nmath::Dir3{axis}.vec();
  const nmath::Point3 on{az.x * coord, az.y * coord, az.z * coord};
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = blend::facePlane(s, static_cast<int>(i));
    if (!pl) continue;
    if (std::fabs(std::fabs(nmath::dot(pl->normal, az)) - 1.0) > 1e-6) continue;
    if (std::fabs(blend::signedDist(*pl, on)) < 1e-6) return static_cast<int>(i);
  }
  return 0;
}
int capFaceAtZ(const topo::Shape& s, double z) { return capFaceAt(s, {0, 0, 1}, z); }
int capFaceAtY(const topo::Shape& s, double y) { return capFaceAt(s, {0, 1, 0}, y); }

// The (first) curved wall face — a Cylinder or Cone. 0 if none.
int curvedWallFace(const topo::Shape& s) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto surf = topo::surfaceOf(map.shape(static_cast<int>(i)));
    if (!surf) continue;
    if (surf->surface->kind == topo::FaceSurface::Kind::Cylinder ||
        surf->surface->kind == topo::FaceSurface::Kind::Cone)
      return static_cast<int>(i);
  }
  return 0;
}

}  // namespace

// ── CYLINDER shell ────────────────────────────────────────────────────────────────

CC_TEST(cylinder_shell_open_top_closed_form) {
  // A cup: Rc=5, H=10, wall t=1, top (z=H) open. Wall volume:
  //   V = π·Rc²·H − π·(Rc−t)²·(H−t) = π·25·10 − π·16·9 = π(250 − 144) = 106π.
  const double Rc = 5.0, H = 10.0, t = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, H);
  const int top = capFaceAtZ(cyl, H);
  CC_CHECK(top != 0);
  const int ids[1] = {top};
  topo::Shape sh = blend::curved_shell(cyl, ids, 1, t, 0.004);
  CC_CHECK(!sh.isNull());
  bool wt = false;
  const double v = vol(sh, wt);
  CC_CHECK(wt);
  const double expected = kPi * (Rc * Rc * H - (Rc - t) * (Rc - t) * (H - t));
  CC_CHECK(nearRel(v, expected, 3e-3));
  CC_CHECK(v < kPi * Rc * Rc * H);  // SHRINK
}

CC_TEST(cylinder_shell_open_bottom_symmetry) {
  // Opening the BOTTOM cap gives the same wall volume by symmetry (t inward from the
  // kept top). Same 106π for Rc=5,H=10,t=1.
  const double Rc = 5.0, H = 10.0, t = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, H);
  const int bottom = capFaceAtZ(cyl, 0.0);
  CC_CHECK(bottom != 0);
  const int ids[1] = {bottom};
  topo::Shape sh = blend::curved_shell(cyl, ids, 1, t, 0.004);
  CC_CHECK(!sh.isNull());
  bool wt = false;
  const double v = vol(sh, wt);
  CC_CHECK(wt);
  const double expected = kPi * (Rc * Rc * H - (Rc - t) * (Rc - t) * (H - t));
  CC_CHECK(nearRel(v, expected, 3e-3));
}

CC_TEST(cylinder_shell_converges) {
  // The faceted wall volume converges to the closed form as deflection → 0.
  const double Rc = 6.0, H = 12.0, t = 1.5;
  const double expected = kPi * (Rc * Rc * H - (Rc - t) * (Rc - t) * (H - t));
  topo::Shape cyl = cappedCylinder(Rc, H);
  const int top = capFaceAtZ(cyl, H);
  const int ids[1] = {top};
  double prevErr = 1e9;
  for (double defl : {0.02, 0.008, 0.004, 0.002}) {
    topo::Shape sh = blend::curved_shell(cyl, ids, 1, t, defl);
    CC_CHECK(!sh.isNull());
    bool wt = false;
    const double v = vol(sh, wt);
    CC_CHECK(wt);
    const double err = std::fabs(v - expected);
    CC_CHECK(err <= prevErr + 1e-6);  // error shrinks (or holds) as deflection refines
    prevErr = err;
  }
}

// ── CONE FRUSTUM shell ──────────────────────────────────────────────────────────---

CC_TEST(frustum_shell_open_top_closed_form) {
  // A tapered bushing: base Rb=6, top Rt=4, H=10, wall t=1, top open. The inner surface
  // is the same cone shifted inward by t/cosσ perpendicular. σ: tanσ = (Rt−Rb)/H.
  //   Outer frustum volume (full, base→top): (πH/3)(Rb²+Rb·Rt+Rt²).
  //   Inner frustum: the cavity runs from the KEPT inner face (z=t) to the open end (z=H).
  //     inner radius at z: Rw(z) − t/cosσ, with Rw(z)=Rb + z·tanσ.
  //   Cavity volume = ∫_t^H π·(Rw(z)−t/cosσ)² dz.
  const double Rb = 6.0, Rt = 4.0, H = 10.0, t = 1.0;
  const double tanS = (Rt - Rb) / H;
  const double cosS = 1.0 / std::sqrt(1.0 + tanS * tanS);
  const double inset = t / cosS;
  topo::Shape fr = cappedFrustum(Rb, Rt, H);
  const int top = capFaceAtY(fr, H);
  CC_CHECK(top != 0);
  const int ids[1] = {top};
  topo::Shape sh = blend::curved_shell(fr, ids, 1, t, 0.004);
  CC_CHECK(!sh.isNull());
  bool wt = false;
  const double v = vol(sh, wt);
  CC_CHECK(wt);
  // Outer solid frustum volume.
  const double vOuter = (kPi * H / 3.0) * (Rb * Rb + Rb * Rt + Rt * Rt);
  // Cavity: numerically integrate the inner frustum from z=t to z=H (closed form below).
  // ∫ π (a + b z)² dz with a=Rb−inset, b=tanS over [t,H]:
  const double a = Rb - inset, b = tanS;
  auto F = [&](double z) {
    // antiderivative of (a+bz)² = a²z + a b z² + b² z³/3
    return a * a * z + a * b * z * z + b * b * z * z * z / 3.0;
  };
  const double vCav = kPi * (F(H) - F(t));
  const double expected = vOuter - vCav;
  CC_CHECK(nearRel(v, expected, 5e-3));
  CC_CHECK(v < vOuter);  // SHRINK
}

// ── DECLINES (measured, first-class) ───────────────────────────────────────────────

CC_TEST(curved_shell_declines_out_of_scope) {
  const double Rc = 5.0, H = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, H);
  const int top = capFaceAtZ(cyl, H);
  const int bottom = capFaceAtZ(cyl, 0.0);
  const int wall = curvedWallFace(cyl);
  CC_CHECK(top != 0 && bottom != 0 && wall != 0);

  // Zero / negative thickness → NULL.
  int ids1[1] = {top};
  CC_CHECK(blend::curved_shell(cyl, ids1, 1, 0.0, 0.004).isNull());

  // BOTH caps removed → NULL (not the single-open-cap tube).
  int both[2] = {top, bottom};
  CC_CHECK(blend::curved_shell(cyl, both, 2, 1.0, 0.004).isNull());

  // Picking the CURVED WALL (non-planar face) → NULL.
  int idsWall[1] = {wall};
  CC_CHECK(blend::curved_shell(cyl, idsWall, 1, 1.0, 0.004).isNull());

  // No faces removed → NULL.
  CC_CHECK(blend::curved_shell(cyl, nullptr, 0, 1.0, 0.004).isNull());

  // Thickness ≥ radius (cavity collapses, Ri ≤ 0) → NULL.
  CC_CHECK(blend::curved_shell(cyl, ids1, 1, Rc + 0.1, 0.004).isNull());

  // Thickness ≥ height (H ≤ t) → NULL.
  CC_CHECK(blend::curved_shell(cyl, ids1, 1, H + 0.1, 0.004).isNull());

  // A planar box (no curved wall) → NULL (the PLANAR shell owns that; curved declines).
  const double p[] = {0, 0, 4, 0, 4, 4, 0, 4};
  topo::Shape boxS = cst::build_prism(p, 4, 4.0);
  const topo::ShapeMap bmap = topo::mapShapes(boxS, topo::ShapeType::Face);
  int aTop = 0;
  for (std::size_t i = 1; i <= bmap.size(); ++i) {
    const auto pl = blend::facePlane(boxS, static_cast<int>(i));
    if (pl && nmath::dot(pl->normal, nmath::Dir3{{0, 0, 1}}.vec()) > 0.999) { aTop = static_cast<int>(i); break; }
  }
  CC_CHECK(aTop != 0);
  int idsBox[1] = {aTop};
  CC_CHECK(blend::curved_shell(boxS, idsBox, 1, 0.5, 0.004).isNull());
}

CC_TEST(curved_shell_declines_stepped_shaft) {
  // A stepped shaft (two coaxial cylinders + a shoulder) is NOT a pure capped cylinder →
  // NULL (multi-cylinder wall). Revolve a stepped meridian about +Z.
  const double prof[] = {0, 0, 6, 0, 6, 3, 4, 3, 4, 9, 0, 9};
  topo::Shape shaft = cst::build_revolution(prof, 6, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * kPi);
  CC_CHECK(!shaft.isNull());
  const int top = capFaceAtY(shaft, 9.0);
  // Even if a top cap is found, the multi-cylinder body must be rejected.
  int ids[1] = {top != 0 ? top : 1};
  CC_CHECK(blend::curved_shell(shaft, ids, 1, 0.5, 0.004).isNull());
}

CC_RUN_ALL()
