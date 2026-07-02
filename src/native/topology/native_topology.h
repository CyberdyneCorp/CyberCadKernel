// SPDX-License-Identifier: Apache-2.0
//
// native_topology.h — public aggregate header for the native B-rep topology
// library (Phase 4, capability #2 `native-topology`).
//
// Clean-room, OCCT-FREE data model + traversal for solid-modelling topology,
// built on the native math foundation (src/native/math). Provides:
//   * shape.h      — ShapeType, Orientation algebra, Location, Shape handle,
//                    TShape shared node, and the geometry payloads
//                    (Vertex point / Edge curve+pcurves / Face surface).
//   * explore.h    — Explorer (deterministic DFS), ShapeMap (stable 1-based
//                    ids), AncestryMap (sub-shape → parents).
//   * accessors.h  — BRep_Tool-equivalent geometry reads.
//   * ShapeBuilder — the ONLY writer: assembles immutable TShape nodes into a
//                    valid B-rep (vertices → edges → wires → faces → shells →
//                    solids → compounds), the standard bottom-up construction
//                    order from Mäntylä.
//
// Conventions mirror OCCT TopoDS/TopExp/BRep_Tool for oracle comparison on the
// simulator; no OCCT header is included. Compiles with clang++ -std=c++20.
//
// -- ORIENTATION / LOCATION MODEL (must match the oracle) ----------------------
// A Shape is (shared TShape node, Orientation, Location). Sharing the node is
// how adjacent faces share an edge. Cumulative orientation down the graph uses
// composed(); world placement composes Locations. This is exactly the
// TopoDS_Shape triple; see shape.h for the identity relations (isSame / isEqual
// / isSameGeometry) that MapShapes and the explorer dedup on.
//
#ifndef CYBERCAD_NATIVE_TOPOLOGY_H
#define CYBERCAD_NATIVE_TOPOLOGY_H

#include "native/topology/shape.h"
#include "native/topology/explore.h"
#include "native/topology/accessors.h"

#include <memory>
#include <utility>
#include <vector>

namespace cybercad::native::topology {

// ─────────────────────────────────────────────────────────────────────────────
// ShapeBuilder — bottom-up construction of immutable topology nodes.
//
// Each factory allocates a fresh shared TShape node and wraps it in a Forward,
// identity-located Shape handle. Sub-shapes are stored with THEIR handle's
// orientation/location (so the caller controls sharing/orientation). The builder
// is the sole friend of TShape's private setters, keeping nodes immutable after
// construction.
//
// Complexity note: each factory is a short, linear assembler (cognitive
// complexity ≤ ~5); no systems-band functions here — the traversal/eval
// complexity lives in explore.h and the math library.
// ─────────────────────────────────────────────────────────────────────────────
class ShapeBuilder {
 public:
  // ── Vertex ─────────────────────────────────────────────────────────────---
  static Shape makeVertex(const math::Point3& p, double tol = math::kLinearTolerance) {
    auto node = std::make_shared<TShape>(ShapeType::Vertex);
    node->geom_ = p;
    node->tolerance_ = tol;
    return wrap(std::move(node));
  }

  // ── Edge ───────────────────────────────────────────────────────────────---
  // An edge references its curve, its parameter range, and its two bounding
  // vertices (v0 = start at `first`, v1 = end at `last`). The start vertex is
  // stored Forward, the end vertex Reversed — the TopoDS convention that lets an
  // edge's orientation flip its vertices' roles. Either vertex may be null for
  // an unbounded/degenerate edge.
  static Shape makeEdge(const EdgeCurve& curve, double first, double last,
                        const Shape& v0, const Shape& v1,
                        double tol = math::kLinearTolerance) {
    auto node = std::make_shared<TShape>(ShapeType::Edge);
    node->geom_ = curve;
    node->first_ = first;
    node->last_ = last;
    node->tolerance_ = tol;
    if (!v0.isNull()) node->children_.push_back(v0.oriented(Orientation::Forward));
    if (!v1.isNull()) node->children_.push_back(v1.oriented(Orientation::Reversed));
    return wrap(std::move(node));
  }

  // General edge factory: stores the given boundary vertices IN ORDER, each
  // KEEPING its own orientation (unlike makeEdge, which forces v0=Forward /
  // v1=Reversed by position). This is the faithful form for edges whose vertex
  // list order and orientations are dictated elsewhere — e.g. edges rebuilt from
  // an external B-rep or produced by a boolean, where the start vertex is not
  // guaranteed to be stored first. Preserving order matters because traversal
  // emits sub-vertices in stored order, so the enumeration must reproduce the
  // source's vertex ordering exactly.
  static Shape makeEdgeWithVertices(const EdgeCurve& curve, double first, double last,
                                    std::vector<Shape> vertices,
                                    double tol = math::kLinearTolerance) {
    auto node = std::make_shared<TShape>(ShapeType::Edge);
    node->geom_ = curve;
    node->first_ = first;
    node->last_ = last;
    node->tolerance_ = tol;
    for (Shape& v : vertices)
      if (!v.isNull()) node->children_.push_back(std::move(v));
    return wrap(std::move(node));
  }

  /// Attach a pcurve of `edge` onto `faceSurfaceNode` (the surface node the
  /// pcurve lives on). Returns a NEW edge node that shares the curve/vertices
  /// but adds the pcurve (nodes are immutable). Typically the same edge node is
  /// laid onto two adjacent faces via two calls, sharing the 3D curve.
  static Shape addPCurve(const Shape& edge, const std::shared_ptr<const TShape>& faceSurfaceNode,
                         const PCurve& pcurve) {
    auto node = cloneEdgeNode(edge);
    node->pcurves_.push_back(TShape::EdgePCurve{faceSurfaceNode, pcurve});
    return Shape{std::move(node), edge.orientation(), edge.location()};
  }

  // ── Wire ───────────────────────────────────────────────────────────────---
  // Edges are stored in connection order, each with its own orientation (which
  // direction the edge is traversed within the wire).
  static Shape makeWire(std::vector<Shape> edges) {
    return makeContainer(ShapeType::Wire, std::move(edges));
  }

  // ── Face ───────────────────────────────────────────────────────────────---
  // Boundary wires: `outer` first (index 0), then hole wires. The face carries
  // its surface + orientation (Forward ⇒ surface normal is the material-outward
  // normal). Hole wires bound the removed regions.
  static Shape makeFace(const FaceSurface& surface, const Shape& outer,
                        std::vector<Shape> holes = {},
                        Orientation orient = Orientation::Forward,
                        double tol = math::kLinearTolerance) {
    auto node = std::make_shared<TShape>(ShapeType::Face);
    node->geom_ = surface;
    node->tolerance_ = tol;
    if (!outer.isNull()) node->children_.push_back(outer);
    for (Shape& h : holes)
      if (!h.isNull()) node->children_.push_back(std::move(h));
    return Shape{std::move(node), orient, Location{}};
  }

  // ── Shell / Solid / CompSolid / Compound ───────────────────────────────---
  static Shape makeShell(std::vector<Shape> faces) {
    return makeContainer(ShapeType::Shell, std::move(faces));
  }
  static Shape makeSolid(std::vector<Shape> shells) {
    return makeContainer(ShapeType::Solid, std::move(shells));
  }
  static Shape makeCompSolid(std::vector<Shape> solids) {
    return makeContainer(ShapeType::CompSolid, std::move(solids));
  }
  static Shape makeCompound(std::vector<Shape> members) {
    return makeContainer(ShapeType::Compound, std::move(members));
  }

 private:
  static Shape wrap(std::shared_ptr<TShape> node) {
    return Shape{std::const_pointer_cast<const TShape>(std::move(node)), Orientation::Forward,
                 Location{}};
  }

  static Shape makeContainer(ShapeType type, std::vector<Shape> children) {
    auto node = std::make_shared<TShape>(type);
    node->children_ = std::move(children);
    return wrap(std::move(node));
  }

  // Copy an edge node's data so a pcurve can be appended immutably.
  static std::shared_ptr<TShape> cloneEdgeNode(const Shape& edge) {
    const TShape& src = *edge.tshape();
    auto node = std::make_shared<TShape>(ShapeType::Edge);
    node->geom_ = src.geometry();
    node->first_ = src.firstParam();
    node->last_ = src.lastParam();
    node->tolerance_ = src.tolerance();
    node->children_ = src.children();
    node->pcurves_ = src.pcurves();
    return node;
  }
};

}  // namespace cybercad::native::topology

#endif  // CYBERCAD_NATIVE_TOPOLOGY_H
