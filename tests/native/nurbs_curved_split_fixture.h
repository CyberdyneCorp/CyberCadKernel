// SPDX-License-Identifier: Apache-2.0
//
// nurbs_curved_split_fixture.h — the reachable proof fixture for NURBS roadmap
// LAYER 3, SLICE 2 (L3-S2): a genuine NURBS-walled bowl cut by an ANALYTIC CURVED
// face (a SPHERE), and its closed-form lens-volume oracle. OCCT-FREE; requires
// CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// This extends nurbs_plane_split_fixture.h (L3-S1, a NURBS bowl cut by a PLANE) with
// a CURVED cutter. The bowl wall is the SAME genuine B-spline (`Kind::BSpline`,
// degree 2, clamped knots {0,0,0,1,1,1}) paraboloid z = a·(x²+y²) — but the cutter is
// now a genuine analytic SPHERE solid (built by the native construct library's
// `build_revolution_profile` of a semicircle arc, translated onto the axis), so L3-S2
// exercises the curved-NURBS↔analytic-CURVED sew (the stage-5 residual the readiness
// doc named), not the curved↔FLAT one L3-S1 lands.
//
// ── GEOMETRY (a clean single-seam lens, exactly closed-form) ────────────────────
// The paraboloid patch maps (u,v)∈[0,1]² to x=(u−½)·2H, y=(v−½)·2H with HALF-WIDTH
// H=0.35, so the untrimmed patch reaches r_max=H·√2≈0.495. A sphere of radius Rs=0.6
// centred on the axis at height zc (below) meets the paraboloid in TWO coaxial circles:
// the INNER r=ρ=0.25 (the seam we want) and an OUTER r≈0.598. The outer circle is OFF
// the patch (0.598 > 0.495), so the SSI trace sees ONLY the clean inner circle — a
// CLOSED interior loop the tracer lands (like L3-S1's plane circle), sidestepping the
// two-branch confusion that a bigger patch would introduce. The bowl is trimmed by a
// rim circle of radius R=0.30 in x (ρ<R<H), so the seam ρ=0.25 is strictly interior to
// the trimmed wall.
//
//   sphere: centre (0,0,zc), zc = a·ρ² + √(Rs²−ρ²); radius Rs. Its LOWER surface
//           z_G(r) = zc − √(Rs²−r²) passes through the seam (ρ, a·ρ²) and bulges DOWN
//           to apex zc−Rs > 0 at r=0 (above the paraboloid vertex, below the lid), so
//           the region {r≤ρ, a·r² ≤ z ≤ z_G(r)} is a genuine closed LENS.
//
// ── THE ORACLE (exact closed form for the lens) ─────────────────────────────────
// KeepSide::Below (CUT — keep the disk sub-face OUTSIDE the sphere: its wall centroid
// r≈0 has ‖·‖>Rs) yields the LENS {r≤ρ, paraboloid ≤ z ≤ sphere-lower} — the disk
// paraboloid sub-face (bottom) sealed by the SPHERICAL cap (top) along the ONE seam:
//
//   V_lens = 2π[ zc·ρ²/2 − a·ρ⁴/4 ] − (2π/3)[ Rs³ − (Rs²−ρ²)^{3/2} ]
//
// (∫₀^ρ [zc − √(Rs²−r²) − a·r²]·2πr dr, the analytic lens volume). The native curved
// cap is a deflection-bounded planar-triangle fan on the true sphere, so the meshed
// volume CONVERGES to V_lens as the mesh refines (O(deflection) faceting — measured
// 7%→0.9% as defl halves 0.004→0.0005). KeepSide::Above (COMMON — keep the annulus
// INSIDE the sphere) is the complementary piece; its watertight+positive-volume weld is
// verified but it has no simple closed form (kept as the second keep side, not a
// closed-form gate).
//
#ifndef CYBERCAD_TESTS_NATIVE_NURBS_CURVED_SPLIT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_NURBS_CURVED_SPLIT_FIXTURE_H

#include "native/construct/native_construct.h"
#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace nurbs_curved_split_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;
namespace cst = cybercad::native::construct;

inline constexpr double kA = 2.0;                          // bowl amplitude (deep cup)
inline constexpr double kH = 0.35;                         // paraboloid patch half-width (x=(u-½)·2H)
inline constexpr double kR = 0.30;                         // rim radius in x (ρ < R < H)
inline constexpr double kRho = 0.25;                       // seam radius in x (the inner circle)
inline constexpr double kRs = 0.6;                         // sphere cutter radius
inline constexpr double kRimZ = kA * kR * kR;              // lid height z = a·R²
inline constexpr double kPi = 3.14159265358979323846;
inline const double kZc = kA * kRho * kRho + std::sqrt(kRs * kRs - kRho * kRho);  // sphere centre z

// ── the bowl surface as a genuine NURBS (Kind::BSpline degree 2) ─────────────────
// Separable quadratic z = a·(x²+y²), x=(u−½)·2H, y=(v−½)·2H over [0,1]². A clamped
// degree-2 B-spline with 3 poles reproduces the quadratic EXACTLY, so z=a·(x²+y²) is
// represented to machine eps (the closed-form lens oracle is exact on THIS surface).
inline std::vector<fmath::Point3> bowlPoles() {
  const double xc[3] = {-kH, 0.0, kH};
  const double zc[3] = {kA * kH * kH, -kA * kH * kH, kA * kH * kH};  // a·H²·[1,−1,1]
  std::vector<fmath::Point3> poles;
  poles.reserve(9);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) poles.push_back(fmath::Point3{xc[i], xc[j], zc[i] + zc[j]});
  return poles;
}
inline std::vector<double> bowlKnots() { return {0, 0, 0, 1, 1, 1}; }  // clamped degree-2

inline topo::FaceSurface bowlSurface() {
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::BSpline;   // ← genuine NURBS, not Bézier
  s.degreeU = 2;
  s.degreeV = 2;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.poles = bowlPoles();
  s.knotsU = bowlKnots();
  s.knotsV = bowlKnots();
  return s;  // s.weights empty ⇒ non-rational NURBS
}

// ── closed-form lens volume oracle (no OCCT) ─────────────────────────────────────
// V_lens = 2π[zc·ρ²/2 − a·ρ⁴/4] − (2π/3)[Rs³ − (Rs²−ρ²)^{3/2}]  (≈ 0.00682994).
inline double lensVolume() {
  const double sphCap = (kRs * kRs * kRs - std::pow(kRs * kRs - kRho * kRho, 1.5)) / 3.0;
  return 2.0 * kPi * (kZc * kRho * kRho / 2.0 - kA * std::pow(kRho, 4) / 4.0) - 2.0 * kPi * sphCap;
}

// ── the analytic SPHERE cutter (native construct; recognisable curved solid) ─────
// A genuine sphere: revolve a semicircle arc (centre on the revolve axis, radius Rs)
// through 2π, then translate the centre from the origin onto the axis at height zc.
// build_revolution_profile emits real Kind::Sphere faces; ssi's recogniseCurvedSolid
// folds them into one CurvedSolid (kind=Sphere, radius=Rs, centre=(0,0,zc), no caps).
inline topo::Shape buildSphereCutter() {
  cst::ProfileSegment arc{};
  arc.kind = 1;  // arc
  arc.cx = 0.0;
  arc.cy = 0.0;
  arc.r = kRs;
  arc.x0 = 0.0;
  arc.y0 = -kRs;
  arc.x1 = 0.0;
  arc.y1 = kRs;
  arc.a0 = -kPi / 2.0;
  arc.a1 = kPi / 2.0;
  cst::ProfileSegment axisSeg{};
  axisSeg.kind = 0;  // the meridian axis segment closing the profile
  axisSeg.x0 = 0.0;
  axisSeg.y0 = kRs;
  axisSeg.x1 = 0.0;
  axisSeg.y1 = -kRs;
  const std::vector<cst::ProfileSegment> segs = {arc, axisSeg};
  const topo::Shape sphere =
      cst::build_revolution_profile(segs, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * kPi);
  return sphere.located(
      topo::Location{fmath::Transform::translationOf(fmath::Vec3{0, 0, kZc})});
}

// The rim circle in the bowl's (u,v): radius R_uv = R/(2H) about (½,½), CCW.
inline std::vector<fmath::Point3> rimUV() {
  const double Ruv = kR / (2.0 * kH);
  const int kRimSegs = 64;
  std::vector<fmath::Point3> uv;
  uv.reserve(kRimSegs);
  for (int k = 0; k < kRimSegs; ++k) {
    const double th = 2.0 * kPi * static_cast<double>(k) / kRimSegs;
    uv.push_back(fmath::Point3{0.5 + Ruv * std::cos(th), 0.5 + Ruv * std::sin(th), 0.0});
  }
  return uv;
}

// ── the bowl-cup operand: NURBS bowl wall (trimmed by the rim) + flat top-lid ─────
struct Operand {
  topo::Shape wall;   ///< the trimmed NURBS bowl wall
  topo::Shape base;   ///< the flat top-lid Plane face
  topo::Shape solid;  ///< the full watertight bowl-cup (for oracle meshing)
};

inline Operand buildOperand() {
  const topo::FaceSurface bowl = bowlSurface();
  tess::SurfaceEvaluator eval(bowl, topo::Location{});
  const std::vector<fmath::Point3> uv = rimUV();
  const int n = static_cast<int>(uv.size());

  std::vector<fmath::Point3> rim3d(n);
  std::vector<topo::Shape> vRim(n);
  for (int k = 0; k < n; ++k) {
    rim3d[k] = eval.value(uv[k].x, uv[k].y);
    vRim[k] = topo::ShapeBuilder::makeVertex(rim3d[k]);
  }

  std::vector<topo::Shape> rimEdges(n);
  std::vector<fmath::Point3> rimCtrl(n);
  for (int k = 0; k < n; ++k) {
    const int k1 = (k + 1) % n;
    const fmath::Point3 mid{(uv[k].x + uv[k1].x) * 0.5, (uv[k].y + uv[k1].y) * 0.5, 0.0};
    const fmath::Point3 Sm = eval.value(mid.x, mid.y);
    rimCtrl[k] = fmath::Point3{2 * Sm.x - 0.5 * (rim3d[k].x + rim3d[k1].x),
                               2 * Sm.y - 0.5 * (rim3d[k].y + rim3d[k1].y),
                               2 * Sm.z - 0.5 * (rim3d[k].z + rim3d[k1].z)};
    topo::EdgeCurve c{};
    c.kind = topo::EdgeCurve::Kind::Bezier;
    c.degree = 2;
    c.poles = {rim3d[k], rimCtrl[k], rim3d[k1]};
    rimEdges[k] = topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, vRim[k], vRim[k1]);
  }

  // bowl wall (NURBS), circular UV trim. Reversed ⇒ outward normal points down/out.
  topo::Shape wall;
  {
    const topo::Shape node = topo::ShapeBuilder::makeFace(bowl, topo::Shape{});
    std::vector<topo::Shape> we;
    we.reserve(n);
    for (int k = 0; k < n; ++k) {
      const int k1 = (k + 1) % n;
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = fmath::Point3{uv[k].x, uv[k].y, 0.0};
      pc.dir2d = fmath::Vec3{uv[k1].x - uv[k].x, uv[k1].y - uv[k].y, 0.0};
      we.push_back(topo::ShapeBuilder::addPCurve(rimEdges[k], node.tshape(), pc));
    }
    wall = topo::ShapeBuilder::makeFace(bowl, topo::ShapeBuilder::makeWire(std::move(we)), {},
                                        topo::Orientation::Reversed);
  }

  // top lid (plane z = a·R²), bounded by the SAME rim edges reversed.
  topo::Shape base;
  {
    topo::FaceSurface pl{};
    pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = fmath::Point3{0, 0, kRimZ};
    pl.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
    pl.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
    pl.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
    const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
    std::vector<topo::Shape> we;
    we.reserve(n);
    for (int k = n - 1; k >= 0; --k) {
      const int k1 = (k + 1) % n;
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::BSpline;
      pc.degree = 2;
      pc.poles2d = {fmath::Point3{rim3d[k].x, rim3d[k].y, 0.0},
                    fmath::Point3{rimCtrl[k].x, rimCtrl[k].y, 0.0},
                    fmath::Point3{rim3d[k1].x, rim3d[k1].y, 0.0}};
      pc.knots = {0, 0, 0, 1, 1, 1};
      we.push_back(topo::ShapeBuilder::addPCurve(rimEdges[k], node.tshape(), pc).reversedShape());
    }
    base = topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)), {},
                                        topo::Orientation::Forward);
  }

  std::vector<topo::Shape> faces = {wall, base};
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  Operand op;
  op.wall = wall;
  op.base = base;
  op.solid = topo::ShapeBuilder::makeSolid({shell});
  return op;
}

}  // namespace nurbs_curved_split_fixture

#endif  // CYBERCAD_TESTS_NATIVE_NURBS_CURVED_SPLIT_FIXTURE_H
