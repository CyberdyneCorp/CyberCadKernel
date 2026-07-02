// SPDX-License-Identifier: Apache-2.0
//
// accessors.h — read geometry off topology nodes (BRep_Tool equivalent).
//
// These free functions mirror BRep_Tool: given a Shape, hand back the geometry
// it carries, applying the shape's Location so results are WORLD-placed (BRep_Tool
// does the same by baking TopLoc_Location into the returned geometry). They are
// pure reads — the topology graph is immutable.
//
//   pointOf(vertex)          → BRep_Tool::Pnt
//   toleranceOf(shape)       → BRep_Tool::Tolerance
//   curveOf(edge)            → BRep_Tool::Curve  (curve + [first,last] range)
//   rangeOf(edge)            → BRep_Tool::Range
//   surfaceOf(face)          → BRep_Tool::Surface
//   pcurveOf(edge, face)     → BRep_Tool::CurveOnSurface (2D pcurve on the face)
//
// OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TOPOLOGY_ACCESSORS_H
#define CYBERCAD_NATIVE_TOPOLOGY_ACCESSORS_H

#include "native/topology/shape.h"

#include <optional>

namespace cybercad::native::topology {

/// Tolerance stored on a shape's node (linear tolerance in model units).
inline double toleranceOf(const Shape& s) noexcept {
  return s.isNull() ? math::kLinearTolerance : s.tshape()->tolerance();
}

// ── Vertex ──────────────────────────────────────────────────────────────────

/// World-space point of a vertex (local point transformed by the shape's
/// Location). Returns nullopt if the shape is not a vertex.
inline std::optional<math::Point3> pointOf(const Shape& vertex) noexcept {
  if (vertex.isNull() || vertex.type() != ShapeType::Vertex) return std::nullopt;
  const auto* p = std::get_if<math::Point3>(&vertex.tshape()->geometry());
  if (!p) return std::nullopt;
  const Location& loc = vertex.location();
  return loc.isIdentity() ? *p : loc.transform().applyToPoint(*p);
}

// ── Edge ──────────────────────────────────────────────────────────────────--

/// Parameter range [first, last] of an edge (unaffected by location).
struct ParamRange {
  double first = 0.0;
  double last = 0.0;
};
inline std::optional<ParamRange> rangeOf(const Shape& edge) noexcept {
  if (edge.isNull() || edge.type() != ShapeType::Edge) return std::nullopt;
  return ParamRange{edge.tshape()->firstParam(), edge.tshape()->lastParam()};
}

/// The 3D curve of an edge plus its parameter range. The curve is returned in
/// LOCAL coordinates together with the edge's Location so the caller can place
/// it (BRep_Tool bakes the location into the returned Geom_Curve; we keep them
/// separate to avoid copying free-form pole arrays here — apply `location` when
/// evaluating). Returns nullopt if the edge has no attached curve.
struct EdgeCurveResult {
  const EdgeCurve* curve = nullptr;
  Location location{};
  double first = 0.0;
  double last = 0.0;
};
inline std::optional<EdgeCurveResult> curveOf(const Shape& edge) noexcept {
  if (edge.isNull() || edge.type() != ShapeType::Edge) return std::nullopt;
  const auto* c = std::get_if<EdgeCurve>(&edge.tshape()->geometry());
  if (!c) return std::nullopt;
  return EdgeCurveResult{c, edge.location(), edge.tshape()->firstParam(),
                         edge.tshape()->lastParam()};
}

// ── Face ──────────────────────────────────────────────────────────────────--

/// The surface of a face plus its Location (see EdgeCurveResult note on why the
/// location is returned rather than baked). Returns nullopt if not a face.
struct FaceSurfaceResult {
  const FaceSurface* surface = nullptr;
  Location location{};
};
inline std::optional<FaceSurfaceResult> surfaceOf(const Shape& face) noexcept {
  if (face.isNull() || face.type() != ShapeType::Face) return std::nullopt;
  const auto* srf = std::get_if<FaceSurface>(&face.tshape()->geometry());
  if (!srf) return std::nullopt;
  return FaceSurfaceResult{srf, face.location()};
}

// ── Edge-on-face pcurve ───────────────────────────────────────────────────--

/// The 2D pcurve of `edge` on the surface of `face`, or nullopt if the edge
/// carries no pcurve for that face's surface node. Matches
/// BRep_Tool::CurveOnSurface. Matching is by the face's SURFACE node identity.
inline const PCurve* pcurveOf(const Shape& edge, const Shape& face) noexcept {
  if (edge.isNull() || face.isNull()) return nullptr;
  if (edge.type() != ShapeType::Edge || face.type() != ShapeType::Face) return nullptr;
  const TShape* faceNode = face.tshape().get();
  for (const auto& pc : edge.tshape()->pcurves())
    if (pc.face.get() == faceNode) return &pc.curve;
  return nullptr;
}

}  // namespace cybercad::native::topology

#endif  // CYBERCAD_NATIVE_TOPOLOGY_ACCESSORS_H
