// SPDX-License-Identifier: Apache-2.0
//
// nurbs_freeform_split_fixture.h — the reachable proof fixture for NURBS roadmap
// LAYER 3, SLICE 3 (L3-S3): TWO genuine-NURBS-walled bowl-cups whose curved NURBS
// walls meet in ONE CLOSED curved seam (the general freeform↔freeform pose, BOTH
// operands arbitrary NURBS), and its closed-form lens-volume oracle. OCCT-FREE;
// requires CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// This is `freeform_freeform_cut_fixture.h` (two coaxial paraboloid bowl-cups) with the
// two curved walls upgraded from `Kind::Bezier` to genuine `Kind::BSpline` (NURBS,
// clamped degree-2 knots {0,0,0,1,1,1}) — so L3-S3 exercises the curved-NURBS↔curved-
// NURBS sew (both operands arbitrary NURBS), the stage-5 deep-tail wall the readiness
// doc named, rather than the Bézier↔Bézier case `freeform_freeform_cut.h` owns.
//
// ── The two operands (both genuine-NURBS bowl-cups) ─────────────────────────────
//   * F = the "up bowl-cup": a degree-2 CLAMPED B-SPLINE bowl z_F = a·(x²+y²) (opens UP,
//     apex at z=0), trimmed by a rim CIRCLE of radius R, closed by a flat TOP LID at
//     z = a·R². Solid F = { a·r² ≤ z ≤ a·R², r ≤ R }.  V(F) = π·a·R⁴/2.
//   * G = the "down dome-cup": a degree-2 CLAMPED B-SPLINE dome z_G = H − a·(x²+y²)
//     (opens DOWN, apex at z=H), trimmed by the SAME rim, closed by a flat BOTTOM LID at
//     z = H − a·R². G is F mirrored in z about z = H/2; its bottom lid sits BELOW F's apex
//     and its apex BELOW F's lid, so G slices F ONLY through the two curved NURBS walls.
//
// A clamped degree-2 B-spline with 3 poles reproduces a quadratic EXACTLY, so each wall
// represents z = a·(x²+y²) (resp. H − a·(x²+y²)) to machine eps — the closed-form lens
// oracle is exact on THESE surfaces (not a fit). Both surfaces are `Kind::BSpline`.
//
// ── The shared CLOSED curved seam ──────────────────────────────────────────────
// F's bowl and G's dome meet where a·r² = H − a·r² ⟺ r = ρ = √(H/(2a)), a CLOSED CIRCLE
// at z* = H/2. With ρ < R the seam is interior to both rim trims — the shared closed
// curved seam BOTH NURBS walls are split along (both sides CURVED NURBS sub-faces).
//
// ── Closed-form volume oracles (no OCCT) ────────────────────────────────────────
// The lens F ∩ G (the COMMON) over the disk r ≤ ρ:
//   V(F∩G) = ∫₀^ρ ((H−a·r²) − a·r²)·2πr dr = π·H·ρ² − π·a·ρ⁴ = π·H²/(4a).
// The CUT F − G = V(F) − V(F∩G) (out of the L3-S3 envelope — deferred, apex-ambiguous).
//
#ifndef CYBERCAD_TESTS_NATIVE_NURBS_FREEFORM_SPLIT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_NURBS_FREEFORM_SPLIT_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace nurbs_freeform_split_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;

inline constexpr double kA = 2.0;      // bowl amplitude (steep, well-conditioned pieces)
inline constexpr double kR = 0.35;     // rim radius in each bowl's (u,v) (rim x,y ∈ ±0.35)
inline constexpr double kH = 0.16;     // G's dome apex height (⇒ seam ρ = √(H/2a) = 0.2)
inline constexpr int kRimSegs = 64;    // rim circle polygon resolution
inline constexpr double kPi = 3.14159265358979323846;

inline double rho() { return std::sqrt(kH / (2.0 * kA)); }  // seam radius (= 0.20)
inline double seamZ() { return kH / 2.0; }                  // seam height z* = H/2

// ── closed-form volume oracles (no OCCT) ────────────────────────────────────────
inline double volF()      { return kPi * kA * kR * kR * kR * kR / 2.0; }   // π·a·R⁴/2
inline double volCommon() { return kPi * kH * kH / (4.0 * kA); }           // lens F∩G = π·H²/(4a)
inline double volCut()    { return volF() - volCommon(); }                 // F − G

// ── F's UP bowl poles (separable degree-2 for z = a·(x²+y²) over [0,1]²) ──────────
// Same pole layout as the Bézier fixture; the surface KIND is BSpline (clamped knots).
inline std::vector<fmath::Point3> upBowlPoles() {
  const double xc[3] = {-0.5, 0.0, 0.5};
  const double zc[3] = {0.25 * kA, -0.25 * kA, 0.25 * kA};  // 0.25a(1−2t)² = a(t−½)²
  std::vector<fmath::Point3> poles;
  poles.reserve(9);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) poles.push_back(fmath::Point3{xc[i], xc[j], zc[i] + zc[j]});
  return poles;
}
// ── G's DOWN dome poles: z = H − a·(x²+y²) (F's poles mirrored: z ↦ H − z) ─────────
inline std::vector<fmath::Point3> downDomePoles() {
  std::vector<fmath::Point3> poles = upBowlPoles();
  for (fmath::Point3& p : poles) p.z = kH - p.z;
  return poles;
}
inline std::vector<double> bowlKnots() { return {0, 0, 0, 1, 1, 1}; }  // clamped degree-2

// ── the bowl/dome surface as a genuine NURBS (Kind::BSpline degree 2) ─────────────
inline topo::FaceSurface bsplineSurface(const std::vector<fmath::Point3>& poles) {
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::BSpline;   // ← genuine NURBS, not Bézier
  s.degreeU = 2;
  s.degreeV = 2;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.poles = poles;
  s.knotsU = bowlKnots();
  s.knotsV = bowlKnots();
  return s;  // s.weights empty ⇒ non-rational NURBS
}

// The NURBS operand front-end adapters (BSpline degree-2, clamped knots).
inline ssi::SurfaceAdapter upBowlAdapter() {
  return ssi::makeBSplineAdapter(2, 2, upBowlPoles(), 3, 3, bowlKnots(), bowlKnots());
}
inline ssi::SurfaceAdapter downDomeAdapter() {
  return ssi::makeBSplineAdapter(2, 2, downDomePoles(), 3, 3, bowlKnots(), bowlKnots());
}

// The real M1 seam: trace F's bowl ∩ G's dome → the single CLOSED WLine (circle ρ).
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

// ── Build a bowl-cup solid: a NURBS bowl (circular UV trim) closed by a flat lid at
// z = lidZ. Mirrors freeform_freeform_cut_fixture::buildCup, but the wall is BSpline. ──
inline topo::Shape buildCup(const std::vector<fmath::Point3>& poles, double lidZ) {
  const topo::FaceSurface bowl = bsplineSurface(poles);
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

  // bowl (NURBS), circular UV trim. Reversed ⇒ outward normal points away from the
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

  // lid (plane z = lidZ), bounded by the SAME rim edges reversed ⇒ shared boundary.
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

// F = the up bowl-cup (lid at z = a·R²); G = the down dome-cup (lid at z = H − a·R²).
inline topo::Shape buildF() { return buildCup(upBowlPoles(), kA * kR * kR); }
inline topo::Shape buildG() { return buildCup(downDomePoles(), kH - kA * kR * kR); }

}  // namespace nurbs_freeform_split_fixture

#endif  // CYBERCAD_TESTS_NATIVE_NURBS_FREEFORM_SPLIT_FIXTURE_H
