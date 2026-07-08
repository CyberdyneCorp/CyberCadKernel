// SPDX-License-Identifier: Apache-2.0
//
// multi_seam_fixture.h — the ONE reachable proof fixture for MOAT M2-multiseam: the
// corner box `B` that slices `A`'s Bézier wall with TWO adjacent faces (a seam graph
// with one junction), and the closed-form CORNER-CLIP volume oracle. Shared by the host
// analytic gate and (later) the sim native-vs-OCCT gate. OCCT-FREE.
//
// `A` = the bowl-lidded convex-quad prism (`first_freeform_boolean_fixture`).
// `B` = the axis-aligned box `x ∈ [0, 0.8], y ∈ [0, 0.6], z ∈ [−0.6, 0.2]`, straddling
// one corner of `A`, so its `x = 0` and `y = 0` faces each slice `A`'s wall and the
// other four faces contain `A`. `B` removes exactly the quadrant `A ∩ {x ≥ 0, y ≥ 0}`.
//
// The oracle integrates the bowl integrand `H0 + a(x² + y²)` over the quad `Q` clipped
// to a half-plane, so `V(A ∩ B)`, `V(A − B)`, `V(A ∪ B)` are mesh-free / OCCT-free.
//
#ifndef CYBERCAD_TESTS_NATIVE_MULTI_SEAM_FIXTURE_H
#define CYBERCAD_TESTS_NATIVE_MULTI_SEAM_FIXTURE_H

#include "native/first_freeform_boolean_fixture.h"
#include "native/topology/native_topology.h"

#include <array>
#include <cmath>
#include <vector>

namespace multi_seam_fixture {

namespace topo = cybercad::native::topology;
namespace fmath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;

// ── the corner box B (six single-quad Plane faces, outward normals) ───────────────
inline topo::Shape quadFace(const std::array<fmath::Point3, 4>& c, const fmath::Vec3& outward) {
  const fmath::Vec3 xd = c[1] - c[0];
  topo::FaceSurface pl{};
  pl.kind = topo::FaceSurface::Kind::Plane;
  pl.frame.origin = c[0];
  pl.frame.x = fmath::Dir3{xd};
  pl.frame.z = fmath::Dir3{outward};
  pl.frame.y = fmath::Dir3{fmath::cross(outward, xd)};
  const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
  auto uv = [&](const fmath::Point3& p) {
    const fmath::Vec3 d = p - pl.frame.origin;
    return fmath::Point3{fmath::dot(d, pl.frame.x.vec()), fmath::dot(d, pl.frame.y.vec()), 0.0};
  };
  std::array<topo::Shape, 4> v;
  for (int k = 0; k < 4; ++k) v[k] = topo::ShapeBuilder::makeVertex(c[k]);
  std::vector<topo::Shape> edges;
  for (int k = 0; k < 4; ++k) {
    const int k1 = (k + 1) % 4;
    const fmath::Vec3 d = c[k1] - c[k];
    const double L = fmath::norm(d);
    topo::EdgeCurve ec{};
    ec.kind = topo::EdgeCurve::Kind::Line;
    ec.frame.origin = c[k];
    ec.frame.x = fmath::Dir3{d};
    topo::Shape e = topo::ShapeBuilder::makeEdge(ec, 0.0, L, v[k], v[k1]);
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = uv(c[k]);
    const fmath::Point3 e1 = uv(c[k1]);
    pc.dir2d = fmath::Vec3{(e1.x - pc.origin2d.x) / L, (e1.y - pc.origin2d.y) / L, 0.0};
    edges.push_back(topo::ShapeBuilder::addPCurve(e, node.tshape(), pc));
  }
  return topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(edges)), {},
                                      topo::Orientation::Forward);
}

inline topo::Shape buildCornerBox(double X0, double X1, double Y0, double Y1, double Z0, double Z1) {
  auto p = [](double x, double y, double z) { return fmath::Point3{x, y, z}; };
  std::vector<topo::Shape> f;
  f.push_back(quadFace({p(X0, Y0, Z0), p(X0, Y0, Z1), p(X0, Y1, Z1), p(X0, Y1, Z0)}, {-1, 0, 0}));
  f.push_back(quadFace({p(X1, Y0, Z0), p(X1, Y1, Z0), p(X1, Y1, Z1), p(X1, Y0, Z1)}, {1, 0, 0}));
  f.push_back(quadFace({p(X0, Y0, Z0), p(X1, Y0, Z0), p(X1, Y0, Z1), p(X0, Y0, Z1)}, {0, -1, 0}));
  f.push_back(quadFace({p(X0, Y1, Z0), p(X0, Y1, Z1), p(X1, Y1, Z1), p(X1, Y1, Z0)}, {0, 1, 0}));
  f.push_back(quadFace({p(X0, Y0, Z0), p(X0, Y1, Z0), p(X1, Y1, Z0), p(X1, Y0, Z0)}, {0, 0, -1}));
  f.push_back(quadFace({p(X0, Y0, Z1), p(X1, Y0, Z1), p(X1, Y1, Z1), p(X0, Y1, Z1)}, {0, 0, 1}));
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(f))});
}

/// The canonical corner box of design §1.
inline topo::Shape cornerBox() { return buildCornerBox(0.0, 0.8, 0.0, 0.6, -0.6, 0.2); }
inline constexpr double kBoxVolume = 0.8 * 0.6 * 0.8;  // V(B) = 0.384

// ── closed-form corner-clip volume oracle (mesh-free, no OCCT) ────────────────────
using ffx::P2;

/// Sutherland–Hodgman clip of a convex polygon to a half-plane. `axis`==0 clips on x,
/// ==1 on y; `keepGreater` keeps the `≥ 0` side.
inline std::vector<P2> clipHalf(const std::vector<P2>& in, int axis, bool keepGreater) {
  std::vector<P2> out;
  const int n = static_cast<int>(in.size());
  auto coord = [&](const P2& q) { return axis == 0 ? q.x : q.y; };
  auto inside = [&](const P2& q) { return keepGreater ? coord(q) >= 0.0 : coord(q) <= 0.0; };
  for (int i = 0; i < n; ++i) {
    const P2 a = in[i], b = in[(i + 1) % n];
    const bool ai = inside(a), bi = inside(b);
    if (ai) out.push_back(a);
    if (ai != bi) {
      const double ca = coord(a), cb = coord(b);
      const double t = (0.0 - ca) / (cb - ca);
      out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
    }
  }
  return out;
}

inline double volFull() { return ffx::polyVolume(ffx::quadXY()); }
/// V(A ∩ B): the bowl integrand over `Q ∩ {x ≥ 0, y ≥ 0}` (the removed corner quadrant).
inline double volCommon() {
  const std::vector<P2> q = clipHalf(clipHalf(ffx::quadXY(), 0, true), 1, true);
  return q.size() >= 3 ? ffx::polyVolume(q) : 0.0;
}
inline double volCut() { return volFull() - volCommon(); }              // V(A − B), L-shaped
inline double volUnion() { return kBoxVolume + volCut(); }              // V(A ∪ B)

// ── closed-form UV corner-area oracle (the removed quadrant's wall projection) ────
// The bowl wall is parameterised over UV = [0,1]², trimmed to the footprint quad Q(uv).
// The removed corner `A ∩ {x≥0, y≥0}` maps to `Q(uv) ∩ {u≥½, v≥½}` (x=u−½, y=v−½), so
// its UV-domain area is the shoelace of that clipped polygon — the ground truth for the
// junction-aware wall split's corner sub-face UV area. Mesh-free, OCCT-free.
inline double uvCornerArea() {
  struct Q2 { double u, v; };
  std::vector<Q2> poly;
  for (const auto& c : ffx::fx::quadUV()) poly.push_back({c.x, c.y});
  auto clip = [](std::vector<Q2> in, int axis, double val) {
    std::vector<Q2> out;
    const int n = static_cast<int>(in.size());
    auto coord = [&](const Q2& q) { return axis == 0 ? q.u : q.v; };
    for (int i = 0; i < n; ++i) {
      const Q2 a = in[i], b = in[(i + 1) % n];
      const bool ai = coord(a) >= val, bi = coord(b) >= val;
      if (ai) out.push_back(a);
      if (ai != bi) {
        const double ca = coord(a), cb = coord(b), t = (val - ca) / (cb - ca);
        out.push_back({a.u + t * (b.u - a.u), a.v + t * (b.v - a.v)});
      }
    }
    return out;
  };
  const std::vector<Q2> c = clip(clip(poly, 0, 0.5), 1, 0.5);
  double A = 0.0;
  const int n = static_cast<int>(c.size());
  for (int i = 0, j = n - 1; i < n; j = i++) A += c[j].u * c[i].v - c[i].u * c[j].v;
  return std::fabs(0.5 * A);
}

/// World footprint straddles BOTH cutting planes (x=0 and y=0) — the proof the box cut is
/// genuinely MULTI-FACE (the bottom quad + side walls over Q, not only the bowl wall).
inline bool footprintStraddlesBothPlanes() {
  bool xNeg = false, xPos = false, yNeg = false, yPos = false;
  for (const P2& p : ffx::quadXY()) {
    xNeg = xNeg || p.x < 0; xPos = xPos || p.x > 0;
    yNeg = yNeg || p.y < 0; yPos = yPos || p.y > 0;
  }
  return xNeg && xPos && yNeg && yPos;
}

}  // namespace multi_seam_fixture

#endif  // CYBERCAD_TESTS_NATIVE_MULTI_SEAM_FIXTURE_H
