// SPDX-License-Identifier: Apache-2.0
//
// face_split_fixture.h — the ONE reachable proof fixture for MOAT M2b / B2
// (freeform face-split), shared by the host tiling gate and the sim native-vs-OCCT
// gate. OCCT-FREE; requires CYBERCAD_HAS_NUMSCI for the M1 seam trace.
//
// Parent: a gently-curved 3×3 Bézier "bowl" patch z = a·((u−½)² + (v−½)²) with
// x = u−½, y = v−½ (so the (u,v) plane and the (x,y) plane coincide). It is
// genuinely TRIMMED by a CONVEX quadrilateral outer EDGE_LOOP whose four edges carry
// Line pcurves (UV) and exact degree-2 Bézier 3D curves (the bowl's z is separable
// quadratic, so S along a straight UV segment is exactly a degree-2 curve).
//
// Cutter: the plane x = 0 → the seam is the chord u ≡ ½ across the trimmed domain,
// entering the bottom quad edge and exiting the top one (one clean entry + exit).
//
// The seam is the REAL M1 WLine from ssi::trace_intersection(bowl, plane) — the
// tracer is CONSUMED, not synthesised.
//
#ifndef CYBERCAD_TESTS_NATIVE_FACE_SPLIT_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_FACE_SPLIT_FIXTURE_H

#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/native_topology.h"

#include <array>
#include <vector>

namespace face_split_fixture {

namespace topo = cybercad::native::topology;
namespace ssi = cybercad::native::ssi;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;

inline constexpr double kBowlA = 0.4;  // gentle bowl amplitude

// The convex quad trim in (u,v) (DIAGNOSE fixture), traced CCW.
inline const std::array<fmath::Point3, 4>& quadUV() {
  static const std::array<fmath::Point3, 4> q{fmath::Point3{0.15, 0.20, 0.0},
                                              fmath::Point3{0.85, 0.25, 0.0},
                                              fmath::Point3{0.80, 0.82, 0.0},
                                              fmath::Point3{0.20, 0.78, 0.0}};
  return q;
}

// 3×3 Bézier control net (row-major, U outer): x depends on U, y on V, z separable
// quadratic. zc = {¼,−¼,¼}·a reproduces a·(param−½)² as a degree-2 Bernstein.
inline std::vector<fmath::Point3> bowlPoles() {
  const double xc[3] = {-0.5, 0.0, 0.5};
  const double zc[3] = {0.25 * kBowlA, -0.25 * kBowlA, 0.25 * kBowlA};
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

// A native SurfaceAdapter for the bowl (for the S2/S3 tracer).
inline ssi::SurfaceAdapter bowlAdapter() { return ssi::makeBezierAdapter(bowlPoles(), 3, 3); }

// The cutter plane x = 0 as a SurfaceAdapter (frame z = +x, param (u,v) → (0,u,v)).
inline ssi::SurfaceAdapter cutterAdapter() {
  const fmath::Ax3 fr{fmath::Point3{0, 0, 0}, fmath::Dir3{0, 1, 0}, fmath::Dir3{0, 0, 1},
                      fmath::Dir3{1, 0, 0}};
  fmath::Plane pl{fr};
  return ssi::makePlaneAdapter(pl, ssi::ParamBox{-0.6, 0.6, -0.2, 0.4});
}

// The genuinely-trimmed parent face over the bowl surface, bounded by the convex quad.
inline topo::Shape parentFace() {
  const topo::FaceSurface surf = bowlSurface();
  const topo::Shape f0 = topo::ShapeBuilder::makeFace(surf, topo::Shape{});
  tess::SurfaceEvaluator eval(surf, topo::Location{});
  const auto& q = quadUV();

  std::vector<topo::Shape> wireEdges;
  for (int k = 0; k < 4; ++k) {
    const fmath::Point3 a = q[k];
    const fmath::Point3 b = q[(k + 1) % 4];
    const fmath::Point3 mid{(a.x + b.x) * 0.5, (a.y + b.y) * 0.5, 0.0};
    const fmath::Point3 S0 = eval.value(a.x, a.y);
    const fmath::Point3 S1 = eval.value(b.x, b.y);
    const fmath::Point3 Sm = eval.value(mid.x, mid.y);
    // Degree-2 Bézier control point that makes the curve pass through Sm at t=½.
    const fmath::Point3 ctrl{2.0 * Sm.x - 0.5 * (S0.x + S1.x), 2.0 * Sm.y - 0.5 * (S0.y + S1.y),
                             2.0 * Sm.z - 0.5 * (S0.z + S1.z)};
    topo::EdgeCurve c3d{};
    c3d.kind = topo::EdgeCurve::Kind::Bezier;
    c3d.degree = 2;
    c3d.poles = {S0, ctrl, S1};
    auto v0 = topo::ShapeBuilder::makeVertex(S0);
    auto v1 = topo::ShapeBuilder::makeVertex(S1);
    topo::Shape e = topo::ShapeBuilder::makeEdge(c3d, 0.0, 1.0, v0, v1);
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = fmath::Point3{a.x, a.y, 0.0};
    pc.dir2d = fmath::Vec3{b.x - a.x, b.y - a.y, 0.0};
    wireEdges.push_back(topo::ShapeBuilder::addPCurve(e, f0.tshape(), pc));
  }
  return topo::ShapeBuilder::makeFace(surf, topo::ShapeBuilder::makeWire(std::move(wireEdges)));
}

// The real M1 seam: trace the bowl ∩ cutter intersection and return the single WLine.
// Requires CYBERCAD_HAS_NUMSCI (the S3 corrector). Returns an empty WLine on failure.
inline ssi::WLine seamWLine() {
  const ssi::SurfaceAdapter A = bowlAdapter();
  const ssi::SurfaceAdapter B = cutterAdapter();
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  for (const ssi::WLine& w : tr.lines)
    if (w.points.size() >= 2) return w;
  return ssi::WLine{};
}

}  // namespace face_split_fixture

#endif  // CYBERCAD_TESTS_NATIVE_FACE_SPLIT_FIXTURE_H
