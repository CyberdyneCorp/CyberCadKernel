// SPDX-License-Identifier: Apache-2.0
//
// nurbs_plane_split_fixture.h — the reachable proof fixture for NURBS roadmap
// LAYER 3, SLICE 1 (L3-S1): a genuine NURBS-walled bowl-cup cut by a horizontal
// plane, and its closed-form volume oracles. OCCT-FREE; requires CYBERCAD_HAS_NUMSCI
// for the M1 seam trace.
//
// This mirrors curved_wall_cut_fixture.h EXCEPT the freeform wall is a genuine
// **B-spline** FaceSurface (`Kind::BSpline`, degree 2, clamped knots {0,0,0,1,1,1})
// rather than a Bézier patch — so L3-S1 exercises the exact-NURBS operand path
// (makeBSplineAdapter / nurbsSurfacePoint), not the Bézier one. A clamped degree-2
// B-spline with 3 poles over [0,1] REPRODUCES the SAME separable quadratic the Bézier
// bowl carries, so z = a·(x²+y²) is represented EXACTLY and the closed-form volume
// oracle π·a·R⁴/2 is unchanged (the surface is bit-equal; only the FaceSurface KIND
// and the SSI adapter differ — the whole point of the slice).
//
// Operand ("bowl cup") = { x²+y² ≤ R², a·(x²+y²) ≤ z ≤ a·R² }:
//   * BOWL WALL (NURBS, Kind::BSpline degree 2): z = a·(x²+y²), x=u−½, y=v−½, trimmed
//     by a CIRCLE of radius R in (u,v) centred at (½,½). Opens UP (min z=0 at centre,
//     rim z=a·R² at R).
//   * TOP LID (Plane): the flat disk at z=a·R² bounded by the SAME rim circle.
// Two faces sharing the rim ⇒ a watertight bowl-cup whose sole NURBS wall is cut by
// the horizontal plane z=c along a CLOSED interior circular seam.
//
// Cutter: horizontal plane z=c, 0<c<a·R², normal +z:
//   * KeepSide::Below (CUT)   — keep the cup below z=c → V = π·ρ²·c/2, ρ=√(c/a).
//   * KeepSide::Above (COMMON)— keep the bowl above z=c + the lid → V = V(full)−that.
//   V(full) = π·a·R⁴/2.
//
#ifndef CYBERCAD_TESTS_NATIVE_NURBS_PLANE_SPLIT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_NURBS_PLANE_SPLIT_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace nurbs_plane_split_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;

inline constexpr double kA = 2.0;                          // steep bowl amplitude (deep cup)
inline constexpr double kR = 0.35;                         // rim radius in (u,v)
inline constexpr double kRimZ = kA * kR * kR;              // top-lid height z = a·R² (= 0.245)
inline constexpr double kRho = 0.25;                       // seam radius in (u,v)
inline constexpr double kCutZ = kA * kRho * kRho;          // cut plane z = c = a·ρ² (= 0.125)
inline constexpr int kRimSegs = 64;
inline constexpr double kPi = 3.14159265358979323846;

// ── the STEEP bowl surface as a genuine NURBS (Kind::BSpline degree 2) ───────────
// Separable quadratic z = a·(x²+y²) with x=u−½, y=v−½ over [0,1]²: the same 3×3 pole
// grid the Bézier bowl uses, carried by a clamped degree-2 B-spline (knots {0,0,0,1,1,1}),
// which is bit-equal to the Bézier over one span but a genuine NURBS FaceSurface.
inline std::vector<fmath::Point3> bowlPoles() {
  const double xc[3] = {-0.5, 0.0, 0.5};
  const double zc[3] = {0.25 * kA, -0.25 * kA, 0.25 * kA};  // 0.25a(1−2t)² = a(t−½)²
  std::vector<fmath::Point3> poles;
  poles.reserve(9);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) poles.push_back(fmath::Point3{xc[i], xc[j], zc[i] + zc[j]});
  return poles;
}
inline std::vector<double> bowlKnots() { return {0, 0, 0, 1, 1, 1}; }  // clamped degree-2, one span

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
  // s.weights left empty ⇒ non-rational NURBS (the "non-rational first" slice).
  return s;
}

// ── closed-form volume oracles (no OCCT) ─────────────────────────────────────────
inline double fullVolume()   { return kPi * kA * kR * kR * kR * kR / 2.0; }  // π·a·R⁴/2
inline double cutVolume()    { return kPi * kRho * kRho * kCutZ / 2.0; }     // π·ρ²·c/2 (z ≤ c)
inline double commonVolume() { return fullVolume() - cutVolume(); }          // z ≥ c

// The cutter plane z = c with normal +z.
inline fmath::Plane cutPlane() {
  fmath::Ax3 fr;
  fr.origin = fmath::Point3{0.0, 0.0, kCutZ};
  fr.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  fr.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  fr.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  return fmath::Plane{fr};
}

// The rim circle in the bowl's (u,v): (½ + R·cosθ, ½ + R·sinθ), CCW.
inline std::vector<fmath::Point3> rimUV() {
  std::vector<fmath::Point3> uv;
  uv.reserve(kRimSegs);
  for (int k = 0; k < kRimSegs; ++k) {
    const double th = 2.0 * kPi * static_cast<double>(k) / kRimSegs;
    uv.push_back(fmath::Point3{0.5 + kR * std::cos(th), 0.5 + kR * std::sin(th), 0.0});
  }
  return uv;
}

// ── the bowl-cup operand: NURBS bowl wall (trimmed by the rim) + flat top-lid ─────
// Returns the wall face and the base (lid) face separately (L3-S1 consumes them as the
// two operand faces). Together they form a watertight closed shell.
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

  // rim edges as degree-2 Bézier 3-D curves (the bowl edge over a straight UV chord is
  // exactly degree 2), built ONCE and shared by the bowl wall + the top lid.
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

}  // namespace nurbs_plane_split_fixture

#endif  // CYBERCAD_TESTS_NATIVE_NURBS_PLANE_SPLIT_FIXTURE_H
