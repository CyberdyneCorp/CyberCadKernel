// SPDX-License-Identifier: Apache-2.0
//
// curved_wall_cut_fixture.h — the reachable proof fixture for MOAT M2 curved-wall
// freeform half-space CUT / COMMON: the dome/bowl-cut-by-a-horizontal-plane pose whose
// `wall ∩ plane` seam is a CLOSED CIRCLE, and its closed-form volume oracles. Shared by
// the host analytic gate and the sim native-vs-OCCT gate. OCCT-FREE; requires
// CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// Operand (a "bowl cup") = { (x,y,z) : x²+y² ≤ R², a·(x²+y²) ≤ z ≤ a·R² }:
//   * BOWL (freeform): a degree-2 Bézier "bowl" patch — z = a·(x²+y²), x=u−½, y=v−½ —
//     the SAME separable-quadratic family the B2 / smooth-trim fixtures use, but with a
//     STEEPER amplitude a so the cut leaves two well-conditioned pieces (not a sliver).
//     Genuinely TRIMMED by a CIRCLE of radius R in the bowl's own (u,v) centred at
//     (½,½). The bowl opens UP: its min is z=0 at the centre, its rim is z=a·R² at R.
//   * TOP LID (analytic, Plane): the flat disk at z = a·R², bounded by the SAME rim
//     circle — the bowl-cup's flat top (Reversed bowl face + Forward lid ⇒ outward-
//     consistent closed shell; validated by B1's watertight edge-incidence audit).
// Two faces sharing the rim circle ⇒ a watertight bowl-cup solid whose SOLE freeform
// wall is cut by a horizontal plane along a CLOSED interior circular seam (the case
// byte-frozen B2 `splitFace` DECLINES; `splitFaceSmoothTrim` resolves).
//
// Cutter: the HORIZONTAL plane z = c with 0 < c < a·R², normal +z:
//   * KeepSide::Below (CUT)   — remove the bowl above z=c → keep the cup r ≤ ρ + a
//     flat circular cap at z=c. Closed form V(z≤c) = π·ρ²·c/2, ρ = √(c/a).
//   * KeepSide::Above (COMMON)— keep the bowl above z=c (the annulus) + the top lid +
//     a flat circular cap at z=c. V(z≥c) = V(full) − V(z≤c).
//   Full bowl-cup volume V(full) = ∫₀^R (a·R² − a·r²)·2πr dr = π·a·R⁴/2.
//
// The seam is the REAL M1 WLine from ssi::trace_intersection(bowl, plane z=c). Closed
// forms are unit-checked in the host gate; no OCCT.
//
#ifndef CYBERCAD_TESTS_NATIVE_CURVED_WALL_CUT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_CURVED_WALL_CUT_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <array>
#include <cmath>
#include <vector>

namespace curved_wall_cut_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;

inline constexpr double kA = 2.0;          // STEEP bowl amplitude (deep cup — both cut pieces are substantial)
inline constexpr double kR = 0.35;         // rim radius in the bowl's (u,v) (rim x,y ∈ ±0.35)
inline constexpr double kRimZ = kA * kR * kR;              // top-lid height z = a·R²  (= 0.245)
inline constexpr double kRho = 0.25;       // circular-seam radius in (u,v)
inline constexpr double kCutZ = kA * kRho * kRho;          // cut plane z = c = a·ρ² (= 0.125, 0 < c < a·R²)
inline constexpr int kRimSegs = 64;        // rim circle polygon resolution
inline constexpr double kPi = 3.14159265358979323846;

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
inline ssi::SurfaceAdapter bowlAdapter() { return ssi::makeBezierAdapter(bowlPoles(), 3, 3); }

// ── closed-form volume oracles (no OCCT) ─────────────────────────────────────────
inline double fullVolume()   { return kPi * kA * kR * kR * kR * kR / 2.0; }     // π·a·R⁴/2
inline double cutVolume()    { return kPi * kRho * kRho * kCutZ / 2.0; }        // π·ρ²·c/2 (z ≤ c)
inline double commonVolume() { return fullVolume() - cutVolume(); }             // z ≥ c

// The cutter plane z = c with normal +z.
inline fmath::Plane cutPlane() {
  fmath::Ax3 fr;
  fr.origin = fmath::Point3{0.0, 0.0, kCutZ};
  fr.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  fr.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  fr.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  return fmath::Plane{fr};
}

// The rim circle in the bowl's (u,v): (½ + R·cosθ, ½ + R·sinθ), CCW, kRimSegs samples.
inline std::vector<fmath::Point3> rimUV() {
  std::vector<fmath::Point3> uv;
  uv.reserve(kRimSegs);
  for (int k = 0; k < kRimSegs; ++k) {
    const double th = 2.0 * kPi * static_cast<double>(k) / kRimSegs;
    uv.push_back(fmath::Point3{0.5 + kR * std::cos(th), 0.5 + kR * std::sin(th), 0.0});
  }
  return uv;
}

// The real M1 seam: trace bowl ∩ {z=c} → the single CLOSED WLine (circle radius ρ).
inline ssi::WLine closedSeamWLine() {
  const ssi::SurfaceAdapter A = bowlAdapter();
  fmath::Plane pl{};
  pl.pos.origin = fmath::Point3{0.0, 0.0, kCutZ};
  const ssi::SurfaceAdapter B = ssi::makePlaneAdapter(pl, ssi::ParamBox{-0.6, 0.6, -0.6, 0.6});
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  const ssi::WLine* best = nullptr;
  for (const ssi::WLine& w : tr.lines) {
    if (w.points.size() < 3) continue;
    if (w.isClosed()) return w;
    if (!best || w.points.size() > best->points.size()) best = &w;
  }
  return best ? *best : ssi::WLine{};
}

// ── the bowl-cup operand solid (bowl freeform + flat top-lid disk) ───────────────
inline topo::Shape buildOperand() {
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
  // exactly degree-2), built ONCE and shared by the bowl face + the top lid.
  std::vector<topo::Shape> rimEdges(n);
  std::vector<fmath::Point3> rimCtrl(n);
  for (int k = 0; k < n; ++k) {
    const int k1 = (k + 1) % n;
    const fmath::Point3 m{(uv[k].x + uv[k1].x) * 0.5, (uv[k].y + uv[k1].y) * 0.5, 0.0};
    const fmath::Point3 Sm = eval.value(m.x, m.y);
    rimCtrl[k] = fmath::Point3{2 * Sm.x - 0.5 * (rim3d[k].x + rim3d[k1].x),
                               2 * Sm.y - 0.5 * (rim3d[k].y + rim3d[k1].y),
                               2 * Sm.z - 0.5 * (rim3d[k].z + rim3d[k1].z)};
    topo::EdgeCurve c{};
    c.kind = topo::EdgeCurve::Kind::Bezier;
    c.degree = 2;
    c.poles = {rim3d[k], rimCtrl[k], rim3d[k1]};
    rimEdges[k] = topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, vRim[k], vRim[k1]);
  }

  std::vector<topo::Shape> faces;

  // bowl (freeform), circular UV trim. Reversed ⇒ outward normal points DOWN/out (the
  // bowl is the cup's underside); the whole-solid orientation is a consistent closed
  // shell, which B1's watertight audit and the M0 mesher confirm.
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
    faces.push_back(topo::ShapeBuilder::makeFace(bowl, topo::ShapeBuilder::makeWire(std::move(we)), {},
                                                 topo::Orientation::Reversed));
  }

  // top lid (plane z = a·R²). Bounded by the SAME rim edges, reversed order + orientation
  // so the lid loop winds opposite the bowl loop ⇒ shared boundary.
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
    faces.push_back(topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)), {},
                                                 topo::Orientation::Forward));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

}  // namespace curved_wall_cut_fixture

#endif  // CYBERCAD_TESTS_NATIVE_CURVED_WALL_CUT_FIXTURE_H
