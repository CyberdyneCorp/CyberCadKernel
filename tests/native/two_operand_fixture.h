// SPDX-License-Identifier: Apache-2.0
//
// two_operand_fixture.h — the reachable proof pose for MOAT M2-FUSE (the FIRST
// two-operand freeform boolean), shared by the host analytic gate and the sim
// native-vs-OCCT gate. OCCT-FREE.
//
// Operand A = the bowl-lidded convex-quad prism of first_freeform_boolean_fixture.
// Operand B = a FINITE axis-aligned box x∈[0,0.8], y∈[−0.6,0.6], z∈[−0.6,0.2] built as
// SIX single-quad Plane faces (one face per side, outward normals) — so exactly ONE of
// its faces (x=0) slices A's Bézier wall (the single-curved-cut pose) and B fully
// contains A's x≥0 material. Then, closed-form and OCCT-free:
//   * V(A ∪ B) = V(B) + V(A ∩ {x≤0})            (disjoint add; B ⊇ A ∩ {x≥0})
//   * V(A − B) = V(A ∩ {x≤0})                    (A outside B)
//   * V(A ∩ B) = V(A) − V(A ∩ {x≤0})             (A inside B = A ∩ {x≥0} ⊂ B)
//
#ifndef CYBERCAD_TESTS_NATIVE_TWO_OPERAND_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_TWO_OPERAND_FIXTURE_H

#include "native/first_freeform_boolean_fixture.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <array>
#include <vector>

namespace two_operand_fixture {

namespace topo = cybercad::native::topology;
namespace tmath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;

inline constexpr double kX0 = 0.0, kX1 = 0.8;
inline constexpr double kY0 = -0.6, kY1 = 0.6;
inline constexpr double kZ0 = -0.6, kZ1 = 0.2;

inline double boxVolume() { return (kX1 - kX0) * (kY1 - kY0) * (kZ1 - kZ0); }
inline double unionVolume() { return boxVolume() + ffx::cutVolume(); }   // A ∪ B
inline double cutVolume() { return ffx::cutVolume(); }                    // A − B
inline double commonVolume() { return ffx::fullVolume() - ffx::cutVolume(); }  // A ∩ B

// One single-quad Plane face: corners CCW as seen from OUTSIDE, `outward` its normal.
// Four Line edges (+ Line pcurves) so the face is a valid, meshable planar quad and
// `extractPolygons` yields ONE polygon per box side (not a triangulated pair).
inline topo::Shape quadFace(const std::array<tmath::Point3, 4>& c, const tmath::Vec3& outward) {
  const tmath::Vec3 xd = c[1] - c[0];
  topo::FaceSurface pl{};
  pl.kind = topo::FaceSurface::Kind::Plane;
  pl.frame.origin = c[0];
  pl.frame.x = tmath::Dir3{xd};
  pl.frame.z = tmath::Dir3{outward};
  pl.frame.y = tmath::Dir3{tmath::cross(outward, xd)};
  const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});

  auto uv = [&](const tmath::Point3& p) {
    const tmath::Vec3 d = p - pl.frame.origin;
    return tmath::Point3{tmath::dot(d, pl.frame.x.vec()), tmath::dot(d, pl.frame.y.vec()), 0.0};
  };
  std::array<topo::Shape, 4> v;
  for (int k = 0; k < 4; ++k) v[k] = topo::ShapeBuilder::makeVertex(c[k]);
  std::vector<topo::Shape> edges;
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    const tmath::Vec3 d = c[k1] - c[k];
    const double L = tmath::norm(d);
    topo::EdgeCurve ec{};
    ec.kind = topo::EdgeCurve::Kind::Line;
    ec.frame.origin = c[k];
    ec.frame.x = tmath::Dir3{d};
    const topo::Shape e = topo::ShapeBuilder::makeEdge(ec, 0.0, L, v[k], v[k1]);
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = uv(c[k]);
    const tmath::Point3 e1 = uv(c[k1]);
    pc.dir2d = tmath::Vec3{(e1.x - pc.origin2d.x) / L, (e1.y - pc.origin2d.y) / L, 0.0};
    edges.push_back(topo::ShapeBuilder::addPCurve(e, node.tshape(), pc));
  }
  return topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(edges)), {},
                                      topo::Orientation::Forward);
}

// The finite box B as a six-quad-face solid (outward normals; one shell).
inline topo::Shape buildBoxB() {
  auto p = [](double x, double y, double z) { return tmath::Point3{x, y, z}; };
  std::vector<topo::Shape> faces;
  // −x (Pcut), +x, −y, +y, −z, +z — corners CCW seen from outside.
  faces.push_back(quadFace({p(kX0, kY0, kZ0), p(kX0, kY0, kZ1), p(kX0, kY1, kZ1), p(kX0, kY1, kZ0)}, {-1, 0, 0}));
  faces.push_back(quadFace({p(kX1, kY0, kZ0), p(kX1, kY1, kZ0), p(kX1, kY1, kZ1), p(kX1, kY0, kZ1)}, {1, 0, 0}));
  faces.push_back(quadFace({p(kX0, kY0, kZ0), p(kX1, kY0, kZ0), p(kX1, kY0, kZ1), p(kX0, kY0, kZ1)}, {0, -1, 0}));
  faces.push_back(quadFace({p(kX0, kY1, kZ0), p(kX0, kY1, kZ1), p(kX1, kY1, kZ1), p(kX1, kY1, kZ0)}, {0, 1, 0}));
  faces.push_back(quadFace({p(kX0, kY0, kZ0), p(kX0, kY1, kZ0), p(kX1, kY1, kZ0), p(kX1, kY0, kZ0)}, {0, 0, -1}));
  faces.push_back(quadFace({p(kX0, kY0, kZ1), p(kX1, kY0, kZ1), p(kX1, kY1, kZ1), p(kX0, kY1, kZ1)}, {0, 0, 1}));
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

}  // namespace two_operand_fixture

#endif  // CYBERCAD_TESTS_NATIVE_TWO_OPERAND_FIXTURE_H
