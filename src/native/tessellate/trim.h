// SPDX-License-Identifier: Apache-2.0
//
// trim.h — UV-space trimming of a face against its wires.
//
// A face is a trimmed surface: an outer wire bounds the kept region and each
// inner (hole) wire bounds a removed region, all expressed in the surface's
// (u,v) parameter plane by the edges' PCURVES (topology::PCurve, obtained via
// BRep_Tool-equivalent pcurveOf). This header:
//
//   * flattens each wire into a polyline of (u,v) points by sampling every
//     edge's pcurve (analytic line/circle/ellipse, or free-form poly) — the
//     wire discretization used both for the outer boundary and holes;
//   * point-in-polygon (even-odd ray cast) so a grid sample can be classified
//     as inside the outer loop and outside every hole loop;
//   * a UVRegion aggregate = outer polygon + hole polygons + their bounding box,
//     the object the face mesher queries per grid vertex / triangle.
//
// The trimming rule (the mesher applies it): keep a UV point iff it is inside
// the outer polygon AND outside every hole polygon. A triangle is kept iff its
// centroid passes that test (a cheap, robust-enough classifier for an adaptive
// grid; near-boundary triangles are handled by sampling the boundary itself in
// the mesher so the silhouette stays within deflection).
//
// OCCT-FREE. Uses src/native/math + src/native/topology. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_TRIM_H
#define CYBERCAD_NATIVE_TESSELLATE_TRIM_H

#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace cybercad::native::tessellate {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

/// A point in the surface parameter plane.
struct UV {
  double u = 0.0;
  double v = 0.0;
};

/// A closed polygon in UV (wire flattened to a polyline; implicitly closed).
using UVPolygon = std::vector<UV>;

/// Axis-aligned UV bounding box.
struct UVBox {
  double uMin = 0.0, uMax = 0.0, vMin = 0.0, vMax = 0.0;
  bool valid = false;
  void expand(const UV& p) noexcept {
    if (!valid) { uMin = uMax = p.u; vMin = vMax = p.v; valid = true; return; }
    uMin = std::min(uMin, p.u); uMax = std::max(uMax, p.u);
    vMin = std::min(vMin, p.v); vMax = std::max(vMax, p.v);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// pcurve sampling — evaluate a topology::PCurve at parameter t (2D).
//
// PCurve mirrors EdgeCurve but in the (u,v) plane: analytic Line/Circle/Ellipse
// use origin2d + dir2d + radii; free-form kinds use poles2d (as (u,v,0)). This
// is a small, self-contained evaluator (the wire flattener drives it).
// ─────────────────────────────────────────────────────────────────────────────
// Evaluate a pcurve. `t` is the true curve parameter (for analytic kinds); `frac`
// is the position within the edge's [first,last] range as a fraction in [0,1]
// (used to index the free-form pole poly, whose sample spacing is uniform in the
// param range — see the harness/kernel that emits poles2d). Analytic kinds ignore
// `frac`; the poly fallback ignores `t`.
inline UV pcurveValue(const topo::PCurve& c, double t, double frac) noexcept {
  using K = topo::EdgeCurve::Kind;
  switch (c.kind) {
    case K::Line:
      return {c.origin2d.x + c.dir2d.x * t, c.origin2d.y + c.dir2d.y * t};
    case K::Circle: {
      // origin2d = center; dir2d.x = radius; (u,v) = center + R(cos t, sin t).
      const double r = c.dir2d.x;
      return {c.origin2d.x + r * std::cos(t), c.origin2d.y + r * std::sin(t)};
    }
    case K::Ellipse: {
      const double a = c.dir2d.x, b = c.dir2d.y;  // major, minor radii
      return {c.origin2d.x + a * std::cos(t), c.origin2d.y + b * std::sin(t)};
    }
    case K::Bezier:
    case K::BSpline:
    default: {
      // Free-form: linear-interpolate the poly of 2D poles by the [0,1] fraction
      // across the edge's param range (poles are stored uniformly over that range).
      if (c.poles2d.empty()) return {c.origin2d.x, c.origin2d.y};
      if (c.poles2d.size() == 1) return {c.poles2d[0].x, c.poles2d[0].y};
      const double f = frac <= 0.0 ? 0.0 : (frac >= 1.0 ? 1.0 : frac);
      const double scaled = f * static_cast<double>(c.poles2d.size() - 1);
      const auto i = static_cast<std::size_t>(scaled);
      const std::size_t j = std::min(i + 1, c.poles2d.size() - 1);
      const double a = scaled - static_cast<double>(i);
      return {c.poles2d[i].x * (1 - a) + c.poles2d[j].x * a,
              c.poles2d[i].y * (1 - a) + c.poles2d[j].y * a};
    }
  }
}

// Resolve the pcurve of `edge` to use on `face`. Prefer the exact match keyed on
// the face node (topo::pcurveOf ≡ BRep_Tool::CurveOnSurface). As a fallback, if
// the edge carries exactly ONE pcurve, use it: an edge whose pcurve was laid onto
// a sibling face node that shares this face's surface (a common consequence of
// building the wire before the final face node exists, and of bridged data where
// each edge has a single pcurve) is unambiguous, so trimming still gets the right
// UV boundary. When an edge legitimately carries pcurves on several faces the
// exact key resolves it, so this fallback never mis-picks.
inline const topo::PCurve* pcurveForFace(const topo::Shape& edge, const topo::Shape& face) noexcept {
  if (const topo::PCurve* exact = topo::pcurveOf(edge, face)) return exact;
  if (edge.isNull() || edge.type() != topo::ShapeType::Edge) return nullptr;
  const auto& pcs = edge.tshape()->pcurves();
  return pcs.size() == 1 ? &pcs.front().curve : nullptr;
}

// ─────────────────────────────────────────────────────────────────────────────
// Wire flattening — sample a wire's edge pcurves into a UV polyline.
//
// For each edge of the wire (in stored order) the edge's pcurve on THIS face is
// evaluated at `segsPerEdge+1` params across the edge's [first,last] range and
// appended (skipping the duplicate join vertex). Edge orientation within the
// wire flips the sampling direction so the polygon is traced consistently.
// ─────────────────────────────────────────────────────────────────────────────
// Sample one edge's pcurve into `poly`. Points run across the edge's [first,last]
// range, in the wire's traversal direction (reversed edges sample backward). The
// first sample is skipped when `poly` is non-empty, to avoid duplicating the
// vertex shared with the previous edge.
inline void appendEdgeSamples(UVPolygon& poly, const topo::Shape& edge, const topo::PCurve& pc,
                              int segsPerEdge) {
  const auto rr = topo::rangeOf(edge);
  const double first = rr ? rr->first : 0.0;
  const double last = rr ? rr->last : 1.0;
  const bool reversed = edge.orientation() == topo::Orientation::Reversed;
  const int startI = poly.empty() ? 0 : 1;
  for (int i = startI; i <= segsPerEdge; ++i) {
    const double a = static_cast<double>(i) / segsPerEdge;
    const double t = reversed ? last + (first - last) * a : first + (last - first) * a;
    const double frac = reversed ? 1.0 - a : a;  // fraction along the stored poly
    poly.push_back(pcurveValue(pc, t, frac));
  }
}

inline UVPolygon flattenWire(const topo::Shape& wire, const topo::Shape& face,
                             int segsPerEdge = 24) {
  UVPolygon poly;
  if (wire.isNull() || wire.type() != topo::ShapeType::Wire) return poly;
  for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
    const topo::Shape& edge = ex.current();
    if (const topo::PCurve* pc = pcurveForFace(edge, face))
      appendEdgeSamples(poly, edge, *pc, segsPerEdge);
  }
  return poly;
}

// ── Shared-edge sampling (STAGE 2 boundary from STAGE 1 discretization) ───────
//
// Sample one edge's pcurve at a GIVEN list of parameter FRACTIONS (from the shared
// EdgeCache) rather than an independent per-edge count. `fracs` are in the edge's
// own forward [first,last] direction; when the wire traverses the edge Reversed we
// walk them backwards so the wire is traced consistently. This is the hook that
// makes two faces sharing an edge place boundary vertices at IDENTICAL 3D points
// (each maps the same fraction through its own pcurve → S_face(pcurve(f)) =
// C_edge(f)). The first sample is skipped when `poly` is non-empty (shared join
// vertex with the previous edge).
inline void appendEdgeSamplesAtFracs(UVPolygon& poly, const topo::Shape& edge,
                                     const topo::PCurve& pc, const std::vector<double>& fracs) {
  const auto rr = topo::rangeOf(edge);
  const double first = rr ? rr->first : 0.0;
  const double last = rr ? rr->last : 1.0;
  const bool reversed = edge.orientation() == topo::Orientation::Reversed;
  const std::size_t startI = poly.empty() ? 0 : 1;
  const std::size_t m = fracs.size();
  for (std::size_t i = startI; i < m; ++i) {
    // Walk the shared fractions in wire-traversal order.
    const double f = reversed ? fracs[m - 1 - i] : fracs[i];
    const double t = first + (last - first) * f;
    poly.push_back(pcurveValue(pc, t, f));
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Point-in-polygon — even-odd (ray-casting) rule. Robust for simple polygons;
// boundary points may classify either way (acceptable — the mesher samples the
// boundary explicitly, so a borderline centroid test never drops a needed
// triangle beyond deflection).
// ─────────────────────────────────────────────────────────────────────────────
inline bool pointInPolygon(const UVPolygon& poly, const UV& p) noexcept {
  const std::size_t n = poly.size();
  if (n < 3) return false;
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const UV& a = poly[i];
    const UV& b = poly[j];
    const bool straddles = (a.v > p.v) != (b.v > p.v);
    if (straddles) {
      const double xCross = (b.u - a.u) * (p.v - a.v) / (b.v - a.v) + a.u;
      if (p.u < xCross) inside = !inside;
    }
  }
  return inside;
}

// ─────────────────────────────────────────────────────────────────────────────
// UVRegion — the trimmed domain of a face: the outer polygon, the hole polygons
// and the overall UV bounding box. classifyInside() is the mesher's keep test.
// ─────────────────────────────────────────────────────────────────────────────
struct UVRegion {
  UVPolygon outer;
  std::vector<UVPolygon> holes;
  UVBox box;

  bool hasOuter() const noexcept { return outer.size() >= 3; }

  /// Keep rule: inside the outer loop AND outside every hole loop.
  bool inside(const UV& p) const noexcept {
    if (!pointInPolygon(outer, p)) return false;
    for (const UVPolygon& h : holes)
      if (pointInPolygon(h, p)) return false;
    return true;
  }

  /// True when the outer loop is (within `tol`) the domain bounding rectangle and
  /// there are no holes — i.e. the face is NOT actually trimmed (its wire is the
  /// four sides of the parametric box, as for a full primitive: a box planar face,
  /// a full cylindrical/spherical face). In that case the mesher keeps the whole
  /// grid — no triangle is dropped and adjacent faces' boundary grid lines coincide
  /// exactly, so welding yields a watertight mesh. A genuinely trimmed face (holes,
  /// or a non-rectangular silhouette) fails this test and goes through inside().
  bool isFullRectangle(double tol) const noexcept {
    if (!holes.empty() || !hasOuter() || !box.valid) return false;
    const double du = box.uMax - box.uMin;
    const double dv = box.vMax - box.vMin;
    const double eu = std::max(du, 1.0) * tol;
    const double ev = std::max(dv, 1.0) * tol;
    // Every outer-loop vertex must lie on the bounding-box border (within tol).
    for (const UV& p : outer) {
      const bool onU = std::fabs(p.u - box.uMin) <= eu || std::fabs(p.u - box.uMax) <= eu;
      const bool onV = std::fabs(p.v - box.vMin) <= ev || std::fabs(p.v - box.vMax) <= ev;
      if (!onU && !onV) return false;  // an interior vertex ⇒ genuinely trimmed
    }
    return true;
  }
};

/// Build a face's UVRegion from its wires (child 0 = outer, rest = holes),
/// flattening each via its pcurves. If the outer wire has no usable pcurves the
/// region is left empty (hasOuter()==false) and the mesher falls back to the
/// surface's natural UV bounds.
inline UVRegion buildRegion(const topo::Shape& face, int segsPerEdge = 24) {
  UVRegion region;
  if (face.isNull() || face.type() != topo::ShapeType::Face) return region;
  const auto& wires = face.tshape()->children();
  for (std::size_t i = 0; i < wires.size(); ++i) {
    UVPolygon poly = flattenWire(wires[i], face, segsPerEdge);
    if (i == 0)
      region.outer = std::move(poly);
    else if (poly.size() >= 3)
      region.holes.push_back(std::move(poly));
  }
  for (const UV& p : region.outer) region.box.expand(p);
  return region;
}

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_TRIM_H
