// SPDX-License-Identifier: Apache-2.0
//
// freeform_freeform_cut_fixture.h — the reachable proof fixture for MOAT M2
// freeform↔freeform CLOSED-SEAM CUT / COMMON: two coaxial paraboloid bowl-cups whose
// two CURVED walls intersect in ONE CLOSED curved seam, and its closed-form volume
// oracles. Shared by the host analytic gate and the sim native-vs-OCCT gate. OCCT-FREE;
// requires CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// ── The two operands (both freeform bowl-cups) ───────────────────────────────────
//   * A = the "up bowl-cup": a degree-2 Bézier bowl z_A = a·(x²+y²) (opens UP, apex at
//     z=0) trimmed by a rim CIRCLE of radius R, closed by a flat TOP LID at z = a·R².
//     Solid A = { a·r² ≤ z ≤ a·R², r ≤ R }.  V(A) = ∫₀^R (a·R²−a·r²)·2πr dr = π·a·R⁴/2.
//   * B = the "down dome-cup": a degree-2 Bézier dome z_B = H − a·(x²+y²) (opens DOWN,
//     apex at z=H) trimmed by the SAME rim radius R, closed by a flat BOTTOM LID at
//     z = H − a·R². Solid B = { H − a·R² ≤ z ≤ H − a·r², r ≤ R }. B is A mirrored in z
//     about z = H/2. Its bottom lid (z = H − a·R²) sits BELOW A's apex, its dome apex
//     (z = H) sits BELOW A's lid (z = a·R²), so B slices A only through the two curved
//     walls, cleanly.
//
// ── The shared CLOSED curved seam ────────────────────────────────────────────────
// A's bowl and B's dome meet where a·r² = H − a·r² ⟺ r² = H/(2a) ⟺ r = ρ = √(H/(2a)),
// a CLOSED CIRCLE at height z* = a·ρ² = H/2. With ρ < R the seam is interior to both
// rim trims. This is the shared closed curved seam BOTH walls are split along — the
// case both sides of the seam are CURVED (the M0w closed-seam pin welds it watertight).
//
// ── Closed-form volume oracles (no OCCT) ─────────────────────────────────────────
// The lens A ∩ B (the COMMON) is bounded below by A's bowl, above by B's dome, over the
// disk r ≤ ρ:  V(A∩B) = ∫₀^ρ ((H−a·r²) − a·r²)·2πr dr = π·H·ρ² − π·a·ρ⁴ = π·H²/(4a)
// (using ρ² = H/(2a)).  The CUT A − B = V(A) − V(A∩B).
//
#ifndef CYBERCAD_TESTS_NATIVE_FREEFORM_FREEFORM_CUT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_FREEFORM_FREEFORM_CUT_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace freeform_freeform_cut_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;

inline constexpr double kA = 2.0;      // bowl amplitude (steep, well-conditioned pieces)
inline constexpr double kR = 0.35;     // rim radius in each bowl's (u,v) (rim x,y ∈ ±0.35)
inline constexpr double kH = 0.16;     // B's dome apex height (⇒ seam ρ = √(H/2a) = 0.2)
inline constexpr int kRimSegs = 64;    // rim circle polygon resolution
inline constexpr double kPi = 3.14159265358979323846;

inline double rho() { return std::sqrt(kH / (2.0 * kA)); }        // seam radius (= 0.20)
inline double seamZ() { return kH / 2.0; }                        // seam height z* = H/2

// ── closed-form volume oracles (no OCCT) ─────────────────────────────────────────
inline double volA()      { return kPi * kA * kR * kR * kR * kR / 2.0; }   // π·a·R⁴/2
inline double volCommon() { return kPi * kH * kH / (4.0 * kA); }           // lens A∩B = π·H²/(4a)
inline double volCut()    { return volA() - volCommon(); }                 // A − B

// ── A's UP bowl surface (separable degree-2 Bézier for z = a·(x²+y²)) ─────────────
inline std::vector<fmath::Point3> upBowlPoles() {
  const double xc[3] = {-0.5, 0.0, 0.5};
  const double zc[3] = {0.25 * kA, -0.25 * kA, 0.25 * kA};  // 0.25a(1−2t)² = a(t−½)²
  std::vector<fmath::Point3> poles;
  poles.reserve(9);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) poles.push_back(fmath::Point3{xc[i], xc[j], zc[i] + zc[j]});
  return poles;
}
// ── B's DOWN dome surface: z = H − a·(x²+y²) (A's poles mirrored: z ↦ H − z) ───────
inline std::vector<fmath::Point3> downDomePoles() {
  std::vector<fmath::Point3> poles = upBowlPoles();
  for (fmath::Point3& p : poles) p.z = kH - p.z;
  return poles;
}

inline topo::FaceSurface bezierSurface(const std::vector<fmath::Point3>& poles) {
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Bezier;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.poles = poles;
  return s;
}
inline ssi::SurfaceAdapter upBowlAdapter() { return ssi::makeBezierAdapter(upBowlPoles(), 3, 3); }
inline ssi::SurfaceAdapter downDomeAdapter() { return ssi::makeBezierAdapter(downDomePoles(), 3, 3); }

// The real M1 seam: trace A's bowl ∩ B's dome → the single CLOSED WLine (circle ρ).
inline ssi::WLine closedSeamWLine() {
  const ssi::TraceSet tr = ssi::trace_intersection(upBowlAdapter(), downDomeAdapter());
  const ssi::WLine* best = nullptr;
  for (const ssi::WLine& w : tr.lines) {
    if (w.points.size() < 3) continue;
    if (w.isClosed()) return w;
    if (!best || w.points.size() > best->points.size()) best = &w;
  }
  return best ? *best : ssi::WLine{};
}

// The rim circle in a bowl's (u,v): (½ + R·cosθ, ½ + R·sinθ), CCW, kRimSegs samples.
inline std::vector<fmath::Point3> rimUV() {
  std::vector<fmath::Point3> uv;
  uv.reserve(kRimSegs);
  for (int k = 0; k < kRimSegs; ++k) {
    const double th = 2.0 * kPi * static_cast<double>(k) / kRimSegs;
    uv.push_back(fmath::Point3{0.5 + kR * std::cos(th), 0.5 + kR * std::sin(th), 0.0});
  }
  return uv;
}

// ── Build a bowl-cup solid: a freeform bowl (circular UV trim) closed by a flat lid at
// z = lidZ. `bowlDown` mirrors the winding so the outward normal is consistent. ───────
inline topo::Shape buildCup(const std::vector<fmath::Point3>& poles, double lidZ) {
  const topo::FaceSurface bowl = bezierSurface(poles);
  tess::SurfaceEvaluator eval(bowl, topo::Location{});
  const std::vector<fmath::Point3> uv = rimUV();
  const int n = static_cast<int>(uv.size());

  std::vector<fmath::Point3> rim3d(n);
  std::vector<topo::Shape> vRim(n);
  for (int k = 0; k < n; ++k) {
    rim3d[k] = eval.value(uv[k].x, uv[k].y);
    vRim[k] = topo::ShapeBuilder::makeVertex(rim3d[k]);
  }

  // rim edges as degree-2 Bézier 3-D curves (bowl edge over a straight UV chord is
  // exactly degree-2), built ONCE and shared by the bowl face + the lid.
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

  // bowl (freeform), circular UV trim. Reversed ⇒ outward normal points away from the
  // solid interior; the whole-solid orientation is a consistent closed shell (B1 audit).
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
    faces.push_back(topo::ShapeBuilder::makeFace(bowl, topo::ShapeBuilder::makeWire(std::move(we)),
                                                 {}, topo::Orientation::Reversed));
  }

  // lid (plane z = lidZ). Bounded by the SAME rim edges, reversed order + orientation so
  // the lid loop winds opposite the bowl loop ⇒ shared boundary, watertight.
  {
    topo::FaceSurface pl{};
    pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = fmath::Point3{0, 0, lidZ};
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
    faces.push_back(topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(we)),
                                                 {}, topo::Orientation::Forward));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// A = the up bowl-cup (lid at z = a·R²); B = the down dome-cup (lid at z = H − a·R²).
inline topo::Shape buildA() { return buildCup(upBowlPoles(), kA * kR * kR); }
inline topo::Shape buildB() { return buildCup(downDomePoles(), kH - kA * kR * kR); }

}  // namespace freeform_freeform_cut_fixture

#endif  // CYBERCAD_TESTS_NATIVE_FREEFORM_FREEFORM_CUT_FIXTURE_H
