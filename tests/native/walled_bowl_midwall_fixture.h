// SPDX-License-Identifier: Apache-2.0
//
// walled_bowl_midwall_fixture.h — the reachable proof fixture for MOAT M2 curved-wall
// freeform half-space CUT in the "walled bowl / dome cut MID-WALL" pose: a bowl-lidded
// convex-quad PRISM (a degree-2 Bézier bowl over a convex quad + 4 PLANAR side walls +
// a flat base) cut by a HORIZONTAL plane z = c whose seam on the freeform bowl is a
// CLOSED interior CIRCLE AND which genuinely crosses the 4 analytic side walls (the
// analytic Split fires). Shared by the host analytic gate + the sim native-vs-OCCT
// gate. OCCT-FREE; requires CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// ── WHY THIS COMPLETES THE MID-WALL SLICE ─────────────────────────────────────────
// The landed dome-cut CUT (`curved_wall_cut_fixture`) cuts a bowl-cup whose ONLY other
// face (the flat top lid) is entirely on the keep side, so NO analytic split fires and
// the cross-section cap is a simple DISK. Here the cut plane crosses the freeform bowl
// (closed interior circle) AND the 4 planar walls, so:
//   * every wall is SPLIT (`hscdetail::cutAnalyticFace` → Kind::Split, 2 crossings);
//   * the cross-section cap is an ANNULUS — OUTER = the 4 wall-section chords, inner
//     HOLE = the freeform seam circle.
// This exercises the `curvedWallHalfSpaceCut` MID-WALL (analytic-split + annular-cap)
// path, distinct from the dome-pose disk cap.
//
// ── THE OPERAND ───────────────────────────────────────────────────────────────────
// A = { (x,y) ∈ Q, −H0 ≤ z ≤ a·(x²+y²) }, with a STEEP amplitude a = 2.0 (so the cut
// removes a SUBSTANTIAL fraction — a discriminating volume oracle), H0 = 0.5, Q the
// convex quad in the bowl's (u,v) with x = u−½, y = v−½. Built exactly like
// `first_freeform_boolean_fixture::buildOperand` (shared vertices/edges ⇒ watertight,
// B1-admissible) but with the steep bowl. The bowl min is z = 0 at the axis (x,y)=(0,0)
// (interior to Q); the walls span z ∈ [−H0, edge-height].
//
// ── THE CUT ───────────────────────────────────────────────────────────────────────
// Horizontal plane z = c, normal +z, KeepSide::Below (CUT keeps z ≤ c). c is chosen
// STRICTLY below every top-edge height, which is EXACTLY the condition that the bowl
// seam circle r = ρ = √(c/a) is interior to Q (both ⟺ c < a·d_e², d_e = the min
// axis-to-quad-edge distance): so the top Bézier arcs stay ENTIRELY above the plane
// (dropped whole) while the two vertical edges of each wall straddle it (2 crossings ⇒
// Split). The kept solid = the freeform disk (r ≤ ρ) + the 4 lower wall trapezoids +
// the flat base + the flat ANNULAR cap on z = c.
//
// ── CLOSED-FORM VOLUME ORACLE (no OCCT) ───────────────────────────────────────────
// V(full) = ∫∫_Q (H0 + a·(x²+y²)) dA   (the exact bowl-prism polynomial integral).
// V(z ≤ c) = ∫∫_Q (H0 + min(c, a·r²)) dA = (H0 + c)·A_Q − c·π·ρ²/2,  ρ = √(c/a).
// (∫∫_{r≤ρ} a·r² dA = a·π·ρ⁴/2 = c·π·ρ²/2; the r>ρ column is capped at c.) A_Q is the
// quad area (shoelace). ρ < d_e keeps the disk inside Q so the integral split is exact.
// Unit-checked in the host gate; no OCCT.
//
#ifndef CYBERCAD_TESTS_NATIVE_WALLED_BOWL_MIDWALL_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_WALLED_BOWL_MIDWALL_FIXTURE_H

#include "native/face_split_fixture.h"        // quadUV (the convex trim, reused)
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <array>
#include <cmath>
#include <vector>

namespace walled_bowl_midwall_fixture {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;
namespace fx = face_split_fixture;

inline constexpr double kA = 2.0;    // STEEP bowl amplitude (discriminating cut volume)
inline constexpr double kH0 = 0.5;   // base depth: bottom quad at z = −H0
inline constexpr double kPi = 3.14159265358979323846;

// Cut height: STRICTLY below every top-edge min so the seam circle is interior to Q.
// d_e (min axis-to-edge distance) ≈ 0.274 ⇒ a·d_e² ≈ 0.150; c = 0.10 ⇒ ρ = √(0.05) ≈
// 0.224 < d_e (comfortably interior). At a = 2.0 the cut removes ≈ 30% of the solid.
inline constexpr double kCutZ = 0.10;
inline double rho() { return std::sqrt(kCutZ / kA); }  // seam-circle radius in (x,y)

// ── the STEEP bowl surface (separable degree-2 Bézier for z = a·(x²+y²)) ─────────
inline std::vector<fmath::Point3> bowlPoles() {
  const double xc[3] = {-0.5, 0.0, 0.5};
  const double zc[3] = {0.25 * kA, -0.25 * kA, 0.25 * kA};  // 0.25a(1−2t)² = a(t−½)²
  std::vector<fmath::Point3> poles;
  poles.reserve(9);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) poles.push_back(fmath::Point3{xc[i], xc[j], zc[i] + zc[j]});
  return poles;
}
inline topo::FaceSurface bowlSurface() {
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Bezier;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.poles = bowlPoles();
  return s;
}

// The cutter plane z = c with normal +z.
inline fmath::Plane cutPlane() {
  fmath::Ax3 fr;
  fr.origin = fmath::Point3{0.0, 0.0, kCutZ};
  fr.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  fr.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  fr.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  return fmath::Plane{fr};
}

// The convex quad in (x,y) = (u−½, v−½).
inline std::vector<fmath::Point3> quadXY() {
  const auto& q = fx::quadUV();
  std::vector<fmath::Point3> out;
  out.reserve(4);
  for (int k = 0; k < 4; ++k) out.push_back(fmath::Point3{q[k].x - 0.5, q[k].y - 0.5, 0.0});
  return out;
}
inline double quadArea() {
  const std::vector<fmath::Point3> q = quadXY();
  double a = 0.0;
  for (std::size_t i = 0, j = q.size() - 1; i < q.size(); j = i++)
    a += q[j].x * q[i].y - q[i].x * q[j].y;
  return std::fabs(0.5 * a);
}
inline double minEdgeDistance() {
  const std::vector<fmath::Point3> q = quadXY();
  double best = 1e30;
  for (std::size_t i = 0, j = q.size() - 1; i < q.size(); j = i++) {
    const fmath::Point3 a = q[j], b = q[i];
    const double dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy;
    double t = L2 > 0 ? -(a.x * dx + a.y * dy) / L2 : 0.0;  // foot of perp from origin
    t = std::max(0.0, std::min(1.0, t));
    const double px = a.x + t * dx, py = a.y + t * dy;
    best = std::min(best, std::hypot(px, py));
  }
  return best;
}

// ── closed-form volume oracles (no OCCT) ─────────────────────────────────────────
// ∫∫_poly (H0 + a·(x²+y²)) dA over a convex polygon (triangle fan, exact quadratic
// second-moments per triangle). The FULL bowl-prism volume.
inline double polyVolume(const std::vector<fmath::Point3>& poly) {
  double V = 0.0;
  for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
    const fmath::Point3 a = poly[0], b = poly[i], c = poly[i + 1];
    const double Aabs =
        std::fabs(0.5 * ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)));
    const double Ix2 =
        Aabs / 6.0 * (a.x * a.x + b.x * b.x + c.x * c.x + a.x * b.x + b.x * c.x + c.x * a.x);
    const double Iy2 =
        Aabs / 6.0 * (a.y * a.y + b.y * b.y + c.y * c.y + a.y * b.y + b.y * c.y + c.y * a.y);
    V += kH0 * Aabs + kA * (Ix2 + Iy2);
  }
  return V;
}
inline double fullVolume() { return polyVolume(quadXY()); }
inline double cutVolume() {  // V(z ≤ c)
  const double r = rho();
  return (kH0 + kCutZ) * quadArea() - kCutZ * kPi * r * r / 2.0;
}
inline double commonVolume() { return fullVolume() - cutVolume(); }

// ── the bowl-lidded prism operand (STEEP bowl top + 4 planar walls + flat base) ──
// Byte-for-byte the `first_freeform_boolean_fixture::buildOperand` construction with
// the steep bowl surface: shared vertices/edges make it watertight + B1-admissible.
inline topo::Shape buildOperand() {
  const topo::FaceSurface bowl = bowlSurface();
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
    topo::EdgeCurve c{};
    c.kind = topo::EdgeCurve::Kind::Bezier;
    c.degree = 2;
    c.poles = {S0, ctrl[k], S1};
    TE[k] = topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, vT[k], vT[(k + 1) % 4]);
  }
  for (int k = 0; k < 4; ++k) {
    topo::EdgeCurve c{};
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = T[k];
    fmath::Vec3 d = B[k] - T[k];
    c.frame.x = fmath::Dir3{d};
    VE[k] = topo::ShapeBuilder::makeEdge(c, 0.0, fmath::norm(d), vT[k], vB[k]);
  }
  for (int k = 0; k < 4; ++k) {
    topo::EdgeCurve c{};
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = B[k];
    fmath::Vec3 d = B[(k + 1) % 4] - B[k];
    c.frame.x = fmath::Dir3{d};
    BE[k] = topo::ShapeBuilder::makeEdge(c, 0.0, fmath::norm(d), vB[k], vB[(k + 1) % 4]);
  }

  std::vector<topo::Shape> faces;
  // bowl top (freeform), convex-quad UV trim, Forward.
  {
    const topo::Shape node = topo::ShapeBuilder::makeFace(bowl, topo::Shape{});
    std::vector<topo::Shape> we;
    for (int k = 0; k < 4; ++k) {
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = fmath::Point3{q[k].x, q[k].y, 0.0};
      pc.dir2d = fmath::Vec3{q[(k + 1) % 4].x - q[k].x, q[(k + 1) % 4].y - q[k].y, 0.0};
      we.push_back(topo::ShapeBuilder::addPCurve(TE[k], node.tshape(), pc));
    }
    faces.push_back(topo::ShapeBuilder::makeFace(bowl, topo::ShapeBuilder::makeWire(std::move(we)),
                                                 {}, topo::Orientation::Forward));
  }
  // four planar walls
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    const fmath::Vec3 segd = B[k1] - B[k];
    const double L = fmath::norm(segd);
    const fmath::Vec3 seg = segd / L, up{0, 0, 1}, outN{seg.y, -seg.x, 0.0};
    topo::FaceSurface pl{};
    pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = B[k];
    pl.frame.x = fmath::Dir3{seg};
    pl.frame.y = fmath::Dir3{up};
    pl.frame.z = fmath::Dir3{outN};
    const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
    auto uvOf = [&](const fmath::Point3& P) {
      fmath::Vec3 d = P - B[k];
      return fmath::Point3{fmath::dot(d, seg), fmath::dot(d, up), 0.0};
    };
    std::vector<topo::Shape> we;
    {
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = uvOf(B[k]);
      fmath::Point3 e = uvOf(B[k1]);
      pc.dir2d = fmath::Vec3{(e.x - pc.origin2d.x) / L, (e.y - pc.origin2d.y) / L, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(BE[k], node.tshape(), pc));
    }
    {
      const double Lv = kH0 + T[k1].z;
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = uvOf(T[k1]);
      fmath::Point3 e = uvOf(B[k1]);
      pc.dir2d = fmath::Vec3{(e.x - pc.origin2d.x) / Lv, (e.y - pc.origin2d.y) / Lv, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(VE[k1], node.tshape(), pc).reversedShape());
    }
    {
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::BSpline;
      pc.degree = 2;
      pc.poles2d = {uvOf(T[k]), uvOf(ctrl[k]), uvOf(T[k1])};
      pc.knots = {0, 0, 0, 1, 1, 1};
      we.push_back(topo::ShapeBuilder::addPCurve(TE[k], node.tshape(), pc).reversedShape());
    }
    {
      const double Lv = kH0 + T[k].z;
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = uvOf(T[k]);
      fmath::Point3 e = uvOf(B[k]);
      pc.dir2d = fmath::Vec3{(e.x - pc.origin2d.x) / Lv, (e.y - pc.origin2d.y) / Lv, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(VE[k], node.tshape(), pc));
    }
    faces.push_back(topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)),
                                                 {}, topo::Orientation::Forward));
  }
  // bottom (plane z=−H0, frame z=+z, Reversed ⇒ outward −z)
  {
    topo::FaceSurface pl{};
    pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = fmath::Point3{0, 0, -kH0};
    pl.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
    pl.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
    pl.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
    const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
    std::vector<topo::Shape> we;
    for (int k = 0; k < 4; ++k) {
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = fmath::Point3{B[k].x, B[k].y, 0};
      const double L = fmath::norm(B[(k + 1) % 4] - B[k]);
      pc.dir2d = fmath::Vec3{(B[(k + 1) % 4].x - B[k].x) / L, (B[(k + 1) % 4].y - B[k].y) / L, 0};
      we.push_back(topo::ShapeBuilder::addPCurve(BE[k], node.tshape(), pc));
    }
    faces.push_back(topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)),
                                                 {}, topo::Orientation::Reversed));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

}  // namespace walled_bowl_midwall_fixture

#endif  // CYBERCAD_TESTS_NATIVE_WALLED_BOWL_MIDWALL_FIXTURE_H
