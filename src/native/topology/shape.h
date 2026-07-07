// SPDX-License-Identifier: Apache-2.0
//
// shape.h — native B-rep topology data model (Vertex/Edge/Wire/Face/Shell/
//           Solid/Compound) as a shared-node graph with orientation + location.
//
// Clean-room implementation from first principles and standard B-rep references
// (Mäntylä, *An Introduction to Solid Modeling*; Hoffmann, *Geometric & Solid
// Modeling*). Conventions are aligned with OCCT's TopoDS/TopAbs model so that a
// native shape can be compared against the OCCT oracle on the simulator, but no
// OCCT header is included and nothing is copied verbatim:
//
//   * A Shape is a lightweight VALUE HANDLE = (shared TShape node, Orientation,
//     Location). Many Shapes can reference the SAME underlying TShape — this is
//     how a cube's two adjacent faces share one edge node. Mirrors TopoDS_Shape
//     = (TopoDS_TShape, TopAbs_Orientation, TopLoc_Location).
//   * A TShape is the shared, reference-counted node. It owns the ordered list
//     of child Shapes (each child carries its own orientation/location relative
//     to the parent) and, for leaf types, the geometry payload. Mirrors
//     TopoDS_TShape + its sub-class hierarchy, collapsed into one node type with
//     a variant payload for readability.
//   * Orientation is Forward/Reversed/Internal/External with OCCT's exact
//     Compose/Reverse/Complement algebra (see Orientation below).
//   * ShapeType ordering is COMPOUND > COMPSOLID > SOLID > SHELL > FACE > WIRE >
//     EDGE > VERTEX (most→least complex), matching TopAbs_ShapeEnum so explorer
//     ordering and "avoid" logic agree with TopExp.
//
// Identity semantics (see Shape):
//   * isSameGeometry / isPartner — same underlying TShape node (ignores location
//     and orientation). TopoDS IsPartner.
//   * isSame                     — same TShape AND same Location (ignores
//                                   orientation). TopoDS IsSame; the relation
//                                   MapShapes/explorers dedup on by default.
//   * isEqual                    — same TShape, Location AND Orientation.
//                                   TopoDS IsEqual / Is-identical handle.
//
// OCCT-FREE: includes only src/native/math. Compiles with
//   clang++ -std=c++20
//
#ifndef CYBERCAD_NATIVE_TOPOLOGY_SHAPE_H
#define CYBERCAD_NATIVE_TOPOLOGY_SHAPE_H

#include "native/math/native_math.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

namespace cybercad::native::topology {

namespace math = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// ShapeType — the topological kind, ordered most-complex → least-complex.
//
// The numeric order matters: explorers and the "avoid" filter compare
// complexity by this enum's value, exactly as TopAbs_ShapeEnum does. Do not
// reorder without updating explore.h.
// ─────────────────────────────────────────────────────────────────────────────
enum class ShapeType : std::uint8_t {
  Compound = 0,   ///< A heterogeneous group of any shapes below.
  CompSolid = 1,  ///< A set of solids sharing faces.
  Solid = 2,      ///< A region of 3D space bounded by shells.
  Shell = 3,      ///< A set of faces connected along edges (open or closed).
  Face = 4,       ///< A trimmed surface bounded by wires (one outer + holes).
  Wire = 5,       ///< A connected sequence of edges.
  Edge = 6,       ///< A trimmed 3D curve bounded by two vertices.
  Vertex = 7,     ///< A point in space.
};

/// True if `a` is a strictly more-complex kind than `b` (COMPOUND is the most
/// complex). Used by the explorer's "avoid" rule.
constexpr bool isMoreComplex(ShapeType a, ShapeType b) noexcept {
  return static_cast<std::uint8_t>(a) < static_cast<std::uint8_t>(b);
}

// ─────────────────────────────────────────────────────────────────────────────
// Orientation — relation of a sub-shape to the region it bounds.
//
// Forward/Reversed select which side of the boundary is "material"; Internal /
// External mark embedded shapes (e.g. a seam edge inside a face) that do not
// bound the region. The algebra below matches TopAbs exactly so cumulative
// orientation through the graph agrees with the oracle.
// ─────────────────────────────────────────────────────────────────────────────
enum class Orientation : std::uint8_t {
  Forward = 0,
  Reversed = 1,
  Internal = 2,
  External = 3,
};

/// Reverse: swap the two material sides (Forward↔Reversed; Internal/External
/// unchanged). Mirrors TopAbs::Reverse.
constexpr Orientation reversed(Orientation o) noexcept {
  constexpr Orientation table[4] = {Orientation::Reversed, Orientation::Forward,
                                    Orientation::Internal, Orientation::External};
  return table[static_cast<int>(o)];
}

/// Complement: reverse the inside/outside status of every side
/// (Forward↔Reversed, Internal↔External). Mirrors TopAbs::Complement.
constexpr Orientation complemented(Orientation o) noexcept {
  constexpr Orientation table[4] = {Orientation::Reversed, Orientation::Forward,
                                    Orientation::External, Orientation::Internal};
  return table[static_cast<int>(o)];
}

/// Compose the orientation of an outer shape (`outer`) with that of a sub-shape
/// (`inner`) to get the sub-shape's cumulative orientation. NON-symmetric.
/// Mirrors TopAbs::Compose (the table indexed [inner][outer]).
constexpr Orientation composed(Orientation outer, Orientation inner) noexcept {
  // table[inner][outer]
  constexpr Orientation table[4][4] = {
      {Orientation::Forward, Orientation::Reversed, Orientation::Internal, Orientation::External},
      {Orientation::Reversed, Orientation::Forward, Orientation::Internal, Orientation::External},
      {Orientation::Internal, Orientation::Internal, Orientation::Internal, Orientation::Internal},
      {Orientation::External, Orientation::External, Orientation::External, Orientation::External}};
  return table[static_cast<int>(inner)][static_cast<int>(outer)];
}

// ─────────────────────────────────────────────────────────────────────────────
// Location — a placement applied to a shared node, so one geometry node can be
// instanced at several positions. Wraps math::Transform with an identity
// fast-path, mirroring TopLoc_Location (which stores elementary datums; we store
// the composed affine transform directly — simpler and exact for fp64).
// ─────────────────────────────────────────────────────────────────────────────
class Location {
 public:
  constexpr Location() noexcept = default;  ///< identity
  explicit Location(const math::Transform& t) noexcept : xf_(t), identity_(false) {}

  constexpr bool isIdentity() const noexcept { return identity_; }
  constexpr const math::Transform& transform() const noexcept { return xf_; }

  /// this ∘ other — apply `other` first, then `this` (transform composition).
  Location composedWith(const Location& other) const noexcept {
    if (identity_) return other;
    if (other.identity_) return *this;
    return Location{xf_.composedWith(other.xf_)};
  }

  /// Inverse placement, or nullopt if the linear part is singular.
  std::optional<Location> inverse() const noexcept {
    if (identity_) return Location{};
    if (auto inv = xf_.inverse()) return Location{*inv};
    return std::nullopt;
  }

  /// Exact equality of the placement (identity compares equal to an identity
  /// transform). Kept exact (bitwise on fp64) to match handle-identity intent.
  bool operator==(const Location& o) const noexcept {
    if (identity_ && o.identity_) return true;
    const math::Transform& a = xf_;
    const math::Transform& b = o.xf_;
    for (std::size_t i = 0; i < 3; ++i)
      for (std::size_t j = 0; j < 3; ++j)
        if (a.linear()(i, j) != b.linear()(i, j)) return false;
    return a.translation() == b.translation();
  }
  bool operator!=(const Location& o) const noexcept { return !(*this == o); }

 private:
  math::Transform xf_{};
  bool identity_ = true;
};

// ─────────────────────────────────────────────────────────────────────────────
// Geometry payloads for leaf/face nodes. Held by the TShape node so that the
// geometry is SHARED with the node (a located instance re-uses the same curve /
// surface and applies its Location when the caller reads world coordinates).
// ─────────────────────────────────────────────────────────────────────────────

/// A 3D curve attached to an edge. Only the analytic/simple forms the native
/// kernel produces so far are modelled; NURBS store their defining arrays so an
/// edge is self-describing without a separate geometry table.
struct EdgeCurve {
  enum class Kind : std::uint8_t { Line, Circle, Ellipse, BSpline, Bezier } kind = Kind::Line;

  // Analytic forms use an origin frame + radii; free-form forms use the arrays.
  math::Ax3 frame{};        ///< placement (Line: origin+X = direction)
  double radius = 0.0;      ///< Circle radius / Ellipse major radius
  double minorRadius = 0.0; ///< Ellipse minor radius

  // Free-form (BSpline/Bezier) definition. Empty for analytic kinds.
  int degree = 0;
  std::vector<math::Point3> poles;
  std::vector<double> weights;  ///< empty ⇒ non-rational
  std::vector<double> knots;    ///< flat knot vector (BSpline only)
};

/// A 2D parametric curve (pcurve) of an edge on a particular face's surface.
/// Represented in the surface's (u,v) parameter plane. Same kinds as EdgeCurve
/// but 2D; we store 2D poles as (u,v,0) Point3 for reuse of the math routines.
struct PCurve {
  EdgeCurve::Kind kind = EdgeCurve::Kind::Line;
  math::Point3 origin2d{};   ///< (u0,v0,0)
  math::Vec3 dir2d{1, 0, 0}; ///< (du,dv,0) for a line
  int degree = 0;
  std::vector<math::Point3> poles2d;  ///< (u,v,0)
  std::vector<double> weights;
  std::vector<double> knots;
};

/// A surface attached to a face. Analytic surfaces reuse the math elementary
/// types; free-form surfaces store their defining grid.
struct FaceSurface {
  enum class Kind : std::uint8_t { Plane, Cylinder, Cone, Sphere, BSpline, Bezier, Torus } kind =
      Kind::Plane;

  math::Ax3 frame{};        ///< placement for analytic surfaces
  double radius = 0.0;      ///< Cylinder/Sphere radius, Cone reference radius, Torus MAJOR radius
  double semiAngle = 0.0;   ///< Cone half-angle
  double minorRadius = 0.0; ///< Torus MINOR (tube) radius; 0 for every other kind

  // Free-form definition (row-major, U outer). Empty for analytic kinds.
  int degreeU = 0;
  int degreeV = 0;
  int nPolesU = 0;
  int nPolesV = 0;
  std::vector<math::Point3> poles;
  std::vector<double> weights;   ///< empty ⇒ non-rational
  std::vector<double> knotsU;
  std::vector<double> knotsV;
};

// Forward declaration.
class TShape;

// ─────────────────────────────────────────────────────────────────────────────
// Shape — the value handle. Cheap to copy (a shared_ptr + two small fields).
// ─────────────────────────────────────────────────────────────────────────────
class Shape {
 public:
  Shape() noexcept = default;
  Shape(std::shared_ptr<const TShape> node, Orientation o, Location loc) noexcept
      : node_(std::move(node)), orient_(o), loc_(std::move(loc)) {}

  bool isNull() const noexcept { return node_ == nullptr; }
  const std::shared_ptr<const TShape>& tshape() const noexcept { return node_; }

  ShapeType type() const noexcept;  // defined after TShape

  Orientation orientation() const noexcept { return orient_; }
  const Location& location() const noexcept { return loc_; }

  /// Return a copy with a different orientation (node/location shared).
  Shape oriented(Orientation o) const noexcept { return Shape{node_, o, loc_}; }
  /// Return a copy with the orientation reversed.
  Shape reversedShape() const noexcept { return Shape{node_, reversed(orient_), loc_}; }
  /// Return a copy relocated by `loc` composed onto the current location.
  Shape located(const Location& loc) const noexcept {
    return Shape{node_, orient_, loc.composedWith(loc_)};
  }

  // ── Identity relations ─────────────────────────────────────────────────────

  /// Same underlying geometry node, regardless of placement/orientation.
  /// (TopoDS IsPartner.)
  bool isSameGeometry(const Shape& o) const noexcept { return node_ == o.node_; }
  bool isPartner(const Shape& o) const noexcept { return isSameGeometry(o); }

  /// Same node AND same placement (orientation ignored). This is the relation
  /// explorers/MapShapes dedup on. (TopoDS IsSame.)
  bool isSame(const Shape& o) const noexcept { return node_ == o.node_ && loc_ == o.loc_; }

  /// Same node, placement AND orientation. (TopoDS IsEqual.)
  bool isEqual(const Shape& o) const noexcept { return isSame(o) && orient_ == o.orient_; }
  bool operator==(const Shape& o) const noexcept { return isEqual(o); }
  bool operator!=(const Shape& o) const noexcept { return !isEqual(o); }

 private:
  std::shared_ptr<const TShape> node_;
  Orientation orient_ = Orientation::Forward;
  Location loc_{};
};

// ─────────────────────────────────────────────────────────────────────────────
// TShape — the shared node. Immutable once built (built via the Builder helpers
// in native_topology.h). Holds the ordered children and the leaf geometry.
//
// Children carry their own orientation/location relative to THIS node; world
// placement is obtained by composing parent locations down the graph.
// ─────────────────────────────────────────────────────────────────────────────
class TShape {
 public:
  using GeometryPayload = std::variant<std::monostate,
                                       math::Point3,  // Vertex position (local)
                                       EdgeCurve,     // Edge 3D curve
                                       FaceSurface>;  // Face surface

  explicit TShape(ShapeType t) noexcept : type_(t) {}

  ShapeType type() const noexcept { return type_; }

  /// Ordered sub-shapes. For a Face, index 0 is the outer wire and the rest are
  /// hole wires (see native_topology.h Builder). For a Wire the edges are in
  /// connection order.
  const std::vector<Shape>& children() const noexcept { return children_; }

  const GeometryPayload& geometry() const noexcept { return geom_; }

  double tolerance() const noexcept { return tolerance_; }

  // ── Edge parameter range (valid only when type() == Edge) ──────────────────
  double firstParam() const noexcept { return first_; }
  double lastParam() const noexcept { return last_; }

  // ── Per-face pcurves of an edge (parallel to the edge's faces) ─────────────
  // Each entry pairs a face TShape (the surface the pcurve lives on) with the
  // 2D curve of THIS edge on that face. Only populated for Edge nodes that have
  // been laid onto faces.
  struct EdgePCurve {
    std::shared_ptr<const TShape> face;  // the face surface node this pcurve is on
    PCurve curve;
  };
  const std::vector<EdgePCurve>& pcurves() const noexcept { return pcurves_; }

  // Face wire counts: number of hole (inner) wires = children - 1 when present.
  bool hasOuterWire() const noexcept { return type_ == ShapeType::Face && !children_.empty(); }

 private:
  friend class ShapeBuilder;  // the only writer (see native_topology.h)

  ShapeType type_;
  std::vector<Shape> children_;
  GeometryPayload geom_{};
  double tolerance_ = math::kLinearTolerance;
  double first_ = 0.0;  // edge parameter range
  double last_ = 0.0;
  std::vector<EdgePCurve> pcurves_;
};

inline ShapeType Shape::type() const noexcept {
  return node_ ? node_->type() : ShapeType::Compound;
}

}  // namespace cybercad::native::topology

#endif  // CYBERCAD_NATIVE_TOPOLOGY_SHAPE_H
