// SPDX-License-Identifier: Apache-2.0
//
// first_freeform_boolean_fixture.h — the ONE reachable proof fixture for MOAT
// M2-assembly / B4: the bowl-lidded convex-quad PRISM operand and its closed-form
// volume oracle, shared by the host analytic gate and the sim native-vs-OCCT gate.
// OCCT-FREE.
//
// Operand A = { (x,y,z) : (x,y) ∈ Q, −H0 ≤ z ≤ a·(x²+y²) }:
//   * TOP (freeform): the B2 degree-2 Bézier "bowl" patch z = a·(x²+y²), x=u−½,
//     y=v−½, genuinely TRIMMED by the convex quad Q (the B2 face_split fixture patch,
//     UNSHIFTED so the fixture's real M1 seam trace is reused verbatim).
//   * FOUR SIDE WALLS (analytic, Plane): each quad edge extruded down to z = −H0.
//     A bowl edge over a straight UV segment is (x,y) linear × z quadratic, so its 3-D
//     curve lies in the VERTICAL plane through that segment ⇒ each wall is PLANAR.
//   * BOTTOM (analytic, Plane): the flat quad Q at z = −H0.
//
// The half-space cut is the plane x = 0 (keep x ≤ 0). The closed-form CUT volume is
// the polynomial ∫∫_{Q ∩ {x≤0}} (H0 + a(x²+y²)) dA — a no-OCCT oracle.
//
#ifndef CYBERCAD_TESTS_NATIVE_FIRST_FREEFORM_BOOLEAN_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_FIRST_FREEFORM_BOOLEAN_FIXTURE_H

#include "native/face_split_fixture.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <array>
#include <cmath>
#include <vector>

namespace first_freeform_boolean_fixture {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;
namespace fx = face_split_fixture;

inline constexpr double kA  = fx::kBowlA;  // bowl amplitude (0.4)
inline constexpr double kH0 = 0.5;         // base depth: bottom quad at z = −H0

// The cutter plane x = 0 with normal +x (keep the x ≤ 0 half).
inline fmath::Plane cutPlane() {
  fmath::Ax3 fr;
  fr.origin = fmath::Point3{0, 0, 0};
  fr.x = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  fr.y = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  fr.z = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  return fmath::Plane{fr};
}

// ── closed-form volume oracle: ∫∫_poly (H0 + a(x²+y²)) dA over a convex polygon ──
struct P2 { double x, y; };
inline double polyVolume(const std::vector<P2>& poly) {
  double V = 0.0;
  for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
    const P2 a = poly[0], b = poly[i], c = poly[i + 1];
    const double Aabs = std::fabs(0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)));
    const double Ix2 = Aabs / 6.0 * (a.x*a.x + b.x*b.x + c.x*c.x + a.x*b.x + b.x*c.x + c.x*a.x);
    const double Iy2 = Aabs / 6.0 * (a.y*a.y + b.y*b.y + c.y*c.y + a.y*b.y + b.y*c.y + c.y*a.y);
    V += kH0 * Aabs + kA * (Ix2 + Iy2);
  }
  return V;
}
inline std::vector<P2> quadXY() {
  const auto& q = fx::quadUV();
  std::vector<P2> out;
  for (int k = 0; k < 4; ++k) out.push_back({q[k].x - 0.5, q[k].y - 0.5});
  return out;
}
inline std::vector<P2> clipXle0(const std::vector<P2>& in) {
  std::vector<P2> out;
  const int n = static_cast<int>(in.size());
  for (int i = 0; i < n; ++i) {
    const P2 a = in[i], b = in[(i + 1) % n];
    const bool ai = a.x <= 0, bi = b.x <= 0;
    if (ai) out.push_back(a);
    if (ai != bi) { const double t = (0.0 - a.x) / (b.x - a.x); out.push_back({0.0, a.y + t * (b.y - a.y)}); }
  }
  return out;
}
inline double fullVolume() { return polyVolume(quadXY()); }
inline double cutVolume()  { return polyVolume(clipXle0(quadXY())); }

// ── the operand solid ───────────────────────────────────────────────────────────
inline topo::Shape buildOperand() {
  const topo::FaceSurface bowl = fx::bowlSurface();
  tess::SurfaceEvaluator eval(bowl, topo::Location{});
  const auto& q = fx::quadUV();

  std::array<fmath::Point3, 4> T, B, ctrl;
  std::array<topo::Shape, 4> vT, vB;
  for (int k = 0; k < 4; ++k) {
    T[k] = eval.value(q[k].x, q[k].y);
    B[k] = fmath::Point3{T[k].x, T[k].y, -kH0};
    vT[k] = topo::ShapeBuilder::makeVertex(T[k]);
    vB[k] = topo::ShapeBuilder::makeVertex(B[k]);
  }
  std::array<topo::Shape, 4> TE, VE, BE;
  for (int k = 0; k < 4; ++k) {
    const fmath::Point3 a = q[k], b = q[(k + 1) % 4], m{(a.x + b.x) * .5, (a.y + b.y) * .5, 0};
    const fmath::Point3 S0 = T[k], S1 = T[(k + 1) % 4], Sm = eval.value(m.x, m.y);
    ctrl[k] = fmath::Point3{2 * Sm.x - .5 * (S0.x + S1.x), 2 * Sm.y - .5 * (S0.y + S1.y),
                            2 * Sm.z - .5 * (S0.z + S1.z)};
    topo::EdgeCurve c{}; c.kind = topo::EdgeCurve::Kind::Bezier; c.degree = 2; c.poles = {S0, ctrl[k], S1};
    TE[k] = topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, vT[k], vT[(k + 1) % 4]);
  }
  for (int k = 0; k < 4; ++k) {
    topo::EdgeCurve c{}; c.kind = topo::EdgeCurve::Kind::Line; c.frame.origin = T[k];
    fmath::Vec3 d = B[k] - T[k]; c.frame.x = fmath::Dir3{d};
    VE[k] = topo::ShapeBuilder::makeEdge(c, 0.0, fmath::norm(d), vT[k], vB[k]);
  }
  for (int k = 0; k < 4; ++k) {
    topo::EdgeCurve c{}; c.kind = topo::EdgeCurve::Kind::Line; c.frame.origin = B[k];
    fmath::Vec3 d = B[(k + 1) % 4] - B[k]; c.frame.x = fmath::Dir3{d};
    BE[k] = topo::ShapeBuilder::makeEdge(c, 0.0, fmath::norm(d), vB[k], vB[(k + 1) % 4]);
  }

  std::vector<topo::Shape> faces;
  // bowl top (freeform)
  {
    const topo::Shape node = topo::ShapeBuilder::makeFace(bowl, topo::Shape{});
    std::vector<topo::Shape> we;
    for (int k = 0; k < 4; ++k) {
      topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = fmath::Point3{q[k].x, q[k].y, 0.0};
      pc.dir2d = fmath::Vec3{q[(k + 1) % 4].x - q[k].x, q[(k + 1) % 4].y - q[k].y, 0.0};
      we.push_back(topo::ShapeBuilder::addPCurve(TE[k], node.tshape(), pc));
    }
    faces.push_back(topo::ShapeBuilder::makeFace(bowl, topo::ShapeBuilder::makeWire(std::move(we)), {},
                                                 topo::Orientation::Forward));
  }
  // four planar walls
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    const fmath::Vec3 segd = B[k1] - B[k];
    const double L = fmath::norm(segd);
    const fmath::Vec3 seg = segd / L, up{0, 0, 1}, outN{seg.y, -seg.x, 0.0};
    topo::FaceSurface pl{}; pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = B[k]; pl.frame.x = fmath::Dir3{seg}; pl.frame.y = fmath::Dir3{up}; pl.frame.z = fmath::Dir3{outN};
    const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
    auto uvOf = [&](const fmath::Point3& P) { fmath::Vec3 d = P - B[k];
      return fmath::Point3{fmath::dot(d, seg), fmath::dot(d, up), 0.0}; };
    std::vector<topo::Shape> we;
    { topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line; pc.origin2d = uvOf(B[k]);
      fmath::Point3 e = uvOf(B[k1]); pc.dir2d = fmath::Vec3{(e.x - pc.origin2d.x) / L, (e.y - pc.origin2d.y) / L, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(BE[k], node.tshape(), pc)); }
    { const double Lv = kH0 + T[k1].z; topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = uvOf(T[k1]); fmath::Point3 e = uvOf(B[k1]);
      pc.dir2d = fmath::Vec3{(e.x - pc.origin2d.x) / Lv, (e.y - pc.origin2d.y) / Lv, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(VE[k1], node.tshape(), pc).reversedShape()); }
    { topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::BSpline; pc.degree = 2;
      pc.poles2d = {uvOf(T[k]), uvOf(ctrl[k]), uvOf(T[k1])}; pc.knots = {0, 0, 0, 1, 1, 1};
      we.push_back(topo::ShapeBuilder::addPCurve(TE[k], node.tshape(), pc).reversedShape()); }
    { const double Lv = kH0 + T[k].z; topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = uvOf(T[k]); fmath::Point3 e = uvOf(B[k]);
      pc.dir2d = fmath::Vec3{(e.x - pc.origin2d.x) / Lv, (e.y - pc.origin2d.y) / Lv, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(VE[k], node.tshape(), pc)); }
    faces.push_back(topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)), {},
                                                 topo::Orientation::Forward));
  }
  // bottom (plane z=−H0, frame z=+z, Reversed ⇒ outward −z)
  {
    topo::FaceSurface pl{}; pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = fmath::Point3{0, 0, -kH0}; pl.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
    pl.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}}; pl.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
    const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
    std::vector<topo::Shape> we;
    for (int k = 0; k < 4; ++k) {
      topo::PCurve pc{}; pc.kind = topo::EdgeCurve::Kind::Line; pc.origin2d = fmath::Point3{B[k].x, B[k].y, 0};
      const double L = fmath::norm(B[(k + 1) % 4] - B[k]);
      pc.dir2d = fmath::Vec3{(B[(k + 1) % 4].x - B[k].x) / L, (B[(k + 1) % 4].y - B[k].y) / L, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(BE[k], node.tshape(), pc));
    }
    faces.push_back(topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)), {},
                                                 topo::Orientation::Reversed));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

}  // namespace first_freeform_boolean_fixture

#endif  // CYBERCAD_TESTS_NATIVE_FIRST_FREEFORM_BOOLEAN_FIXTURE_H
