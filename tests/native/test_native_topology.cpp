// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native B-rep topology library (Phase 4, capability #2
// `native-topology`). OCCT-FREE — this is Gate 1 (host, analytic) of the
// two-gate verification model in openspec/NATIVE-REWRITE.md: the native code
// compiles and unit-tests with clang++ -std=c++20, with no OCCT and no
// simulator. Gate 2 (native-vs-OCCT parity on the sim) is a separate suite.
//
// harness.h style: cases are declared with CC_TEST and run from CC_RUN_ALL().
//
// The tests hand-build canonical topologies with native-math geometry and
// assert INVARIANTS of the data model + traversal (no OCCT):
//   * Euler-Poincaré: a closed box solid (shared edges) satisfies V-E+F = 2
//     (genus 0); a solid with one through-hole/handle satisfies V-E+F = 0
//     (genus 1).
//   * Manifold connectivity: every edge of a closed solid is shared by exactly
//     2 faces (via the AncestryMap edge → faces).
//   * Sub-shape ids are stable, 1-based, unique, and deterministic across
//     repeated MapShapes runs.
//   * Orientation algebra + reversing a face reverses its contribution; the
//     explorer visits the expected counts per type.
//   * Accessors return the attached geometry (edge curve+range, face surface,
//     vertex point) for a built shape.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_topology.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_topology
//
#include "native/topology/native_topology.h"

#include "harness.h"

#include <array>
#include <vector>

using namespace cybercad::native::topology;
namespace m = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// Builders for canonical topologies with SHARED edges/vertices.
//
// A valid closed B-rep shares each edge between exactly two faces. The helpers
// below build a box (and a box with a through-hole) from a shared pool of
// vertices and edges so the Euler-Poincaré and manifold invariants hold — this
// is the standard bottom-up construction order (Mäntylä): vertices → edges →
// wires → faces → shell → solid.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// A straight-line edge between two (shared) vertices, degree-1 param range.
Shape lineEdge(const Shape& a, const Shape& b) {
  EdgeCurve line{};
  line.kind = EdgeCurve::Kind::Line;
  return ShapeBuilder::makeEdge(line, 0.0, 1.0, a, b);
}

// A planar quad face from four edges (already sharing vertices). Each edge is
// inserted with the orientation needed to traverse the loop consistently; the
// caller supplies the four edges in loop order. The shared EDGE nodes are what
// make adjacent faces meet along one edge.
Shape quadFace(const std::array<Shape, 4>& loop, Orientation orient = Orientation::Forward) {
  FaceSurface plane{};
  plane.kind = FaceSurface::Kind::Plane;
  Shape wire = ShapeBuilder::makeWire({loop[0], loop[1], loop[2], loop[3]});
  return ShapeBuilder::makeFace(plane, wire, /*holes=*/{}, orient);
}

// ── A closed box solid with SHARED edges (12 edges, 8 vertices, 6 faces). ──────
//
// Corner naming: bit i = coordinate along axis i (x,y,z). The 12 cube edges are
// the pairs of corners differing in exactly one bit. Every edge node is built
// once and reused by the two faces adjacent to it, so the ancestry map reports
// exactly two faces per edge and Euler-Poincaré gives V-E+F = 2.
struct Box {
  std::array<Shape, 8> v;   // vertices, indexed by corner bits
  std::array<Shape, 12> e;  // shared edges
  std::array<Shape, 6> f;   // faces
  Shape solid;
};

Box makeBox() {
  Box b;
  // 8 corners of the unit cube (index = x + 2y + 4z).
  for (int i = 0; i < 8; ++i) {
    const double x = (i & 1) ? 1 : 0;
    const double y = (i & 2) ? 1 : 0;
    const double z = (i & 4) ? 1 : 0;
    b.v[static_cast<std::size_t>(i)] = ShapeBuilder::makeVertex({x, y, z});
  }
  auto V = [&](int i) -> const Shape& { return b.v[static_cast<std::size_t>(i)]; };

  // 12 edges, each built once from its two corner vertices. Index scheme is
  // arbitrary but fixed; a small table keeps it readable.
  const std::array<std::array<int, 2>, 12> ends{{
      {0, 1}, {2, 3}, {4, 5}, {6, 7},  // x-parallel
      {0, 2}, {1, 3}, {4, 6}, {5, 7},  // y-parallel
      {0, 4}, {1, 5}, {2, 6}, {3, 7},  // z-parallel
  }};
  for (int i = 0; i < 12; ++i)
    b.e[static_cast<std::size_t>(i)] = lineEdge(V(ends[i][0]), V(ends[i][1]));
  auto E = [&](int i) -> const Shape& { return b.e[static_cast<std::size_t>(i)]; };

  // 6 faces. Each face's loop is 4 shared edges; a reversed edge is used where
  // the loop traverses it backwards (so the same edge NODE is shared, only the
  // handle orientation differs — TopoDS convention).
  auto R = [](const Shape& s) { return s.reversedShape(); };

  // z=0 (bottom): 0-1-3-2 ; edges 0,5,1(rev),4(rev)
  b.f[0] = quadFace({E(0), E(5), R(E(1)), R(E(4))});
  // z=1 (top): 4-5-7-6 ; edges 2,7,3(rev),6(rev)
  b.f[1] = quadFace({E(2), E(7), R(E(3)), R(E(6))});
  // y=0 (front): 0-1-5-4 ; edges 0,9,2(rev),8(rev)
  b.f[2] = quadFace({E(0), E(9), R(E(2)), R(E(8))});
  // y=1 (back): 2-3-7-6 ; edges 1,11,3(rev),10(rev)
  b.f[3] = quadFace({E(1), E(11), R(E(3)), R(E(10))});
  // x=0 (left): 0-2-6-4 ; edges 4,10,6(rev),8(rev)
  b.f[4] = quadFace({E(4), E(10), R(E(6)), R(E(8))});
  // x=1 (right): 1-3-7-5 ; edges 5,11,7(rev),9(rev)
  b.f[5] = quadFace({E(5), E(11), R(E(7)), R(E(9))});

  Shape shell = ShapeBuilder::makeShell({b.f[0], b.f[1], b.f[2], b.f[3], b.f[4], b.f[5]});
  b.solid = ShapeBuilder::makeSolid({shell});
  return b;
}

// Count distinct sub-shapes of a type in a shape (deduped by isSame).
long long countOf(const Shape& s, ShapeType t) {
  return static_cast<long long>(mapShapes(s, t).size());
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Orientation algebra (must match TopAbs Reverse / Complement / Compose tables).
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(orientation_algebra) {
  CC_CHECK(reversed(Orientation::Forward) == Orientation::Reversed);
  CC_CHECK(reversed(Orientation::Reversed) == Orientation::Forward);
  CC_CHECK(reversed(Orientation::Internal) == Orientation::Internal);
  CC_CHECK(reversed(Orientation::External) == Orientation::External);

  CC_CHECK(complemented(Orientation::Forward) == Orientation::Reversed);
  CC_CHECK(complemented(Orientation::Internal) == Orientation::External);
  CC_CHECK(complemented(Orientation::External) == Orientation::Internal);

  // Compose is non-symmetric; a few oracle-matching entries.
  CC_CHECK(composed(Orientation::Forward, Orientation::Forward) == Orientation::Forward);
  CC_CHECK(composed(Orientation::Reversed, Orientation::Reversed) == Orientation::Forward);
  CC_CHECK(composed(Orientation::Forward, Orientation::Reversed) == Orientation::Reversed);
  CC_CHECK(composed(Orientation::Reversed, Orientation::Forward) == Orientation::Reversed);
  CC_CHECK(composed(Orientation::Internal, Orientation::Forward) == Orientation::Internal);
}

// ─────────────────────────────────────────────────────────────────────────────
// ShapeType complexity ordering (drives explorer avoid + traversal).
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(shapetype_ordering) {
  CC_CHECK(isMoreComplex(ShapeType::Compound, ShapeType::Solid));
  CC_CHECK(isMoreComplex(ShapeType::Solid, ShapeType::Face));
  CC_CHECK(isMoreComplex(ShapeType::Face, ShapeType::Edge));
  CC_CHECK(isMoreComplex(ShapeType::Edge, ShapeType::Vertex));
  CC_CHECK(!isMoreComplex(ShapeType::Edge, ShapeType::Face));
  CC_CHECK(!isMoreComplex(ShapeType::Vertex, ShapeType::Vertex));
}

// ─────────────────────────────────────────────────────────────────────────────
// Accessors: vertex point (+location), edge curve/range/vertices, face surface.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(accessor_vertex_point_and_location) {
  Shape v = ShapeBuilder::makeVertex({1, 2, 3});
  auto p = pointOf(v);
  CC_CHECK(p.has_value());
  CC_CHECK(p->x == 1 && p->y == 2 && p->z == 3);

  // A located instance shares the node but reports the WORLD point.
  m::Transform t = m::Transform::translationOf({10, 0, 0});
  Shape vl = v.located(Location{t});
  auto pl = pointOf(vl);
  CC_CHECK(pl.has_value());
  CC_CHECK(pl->x == 11 && pl->y == 2 && pl->z == 3);
  CC_CHECK(v.isSameGeometry(vl));  // same node
  CC_CHECK(!v.isSame(vl));         // different location

  // A non-vertex returns nullopt.
  CC_CHECK(!pointOf(lineEdge(v, v)).has_value());
}

CC_TEST(accessor_edge_curve_range_vertices) {
  Shape a = ShapeBuilder::makeVertex({0, 0, 0});
  Shape b = ShapeBuilder::makeVertex({1, 0, 0});
  EdgeCurve line{};
  line.kind = EdgeCurve::Kind::Line;
  Shape e = ShapeBuilder::makeEdge(line, 0.0, 2.5, a, b);

  auto r = rangeOf(e);
  CC_CHECK(r.has_value());
  CC_CHECK(r->first == 0.0 && r->last == 2.5);

  auto c = curveOf(e);
  CC_CHECK(c.has_value());
  CC_CHECK(c->curve->kind == EdgeCurve::Kind::Line);
  CC_CHECK(c->first == 0.0 && c->last == 2.5);

  // Two bounding vertices: start Forward, end Reversed (TopoDS convention).
  CC_CHECK_EQ(static_cast<long long>(e.tshape()->children().size()), 2);
  CC_CHECK(e.tshape()->children()[0].orientation() == Orientation::Forward);
  CC_CHECK(e.tshape()->children()[1].orientation() == Orientation::Reversed);
}

// makeEdgeWithVertices is the general (non-forcing) edge factory: it stores the
// given boundary vertices IN ORDER, each keeping its OWN orientation. This is
// what a faithful bridge from an external B-rep needs — makeEdge, by contrast,
// forces v0=Forward/v1=Reversed by position and would both reorder and relabel
// vertices whose source stores the Reversed vertex first. Regression guard for
// the native-vs-OCCT topology parity harness (vertex MapShapes id order +
// per-edge vertex orientation).
CC_TEST(make_edge_with_vertices_preserves_order_and_orientation) {
  Shape a = ShapeBuilder::makeVertex({0, 0, 0});
  Shape b = ShapeBuilder::makeVertex({1, 0, 0});
  EdgeCurve line{};
  line.kind = EdgeCurve::Kind::Line;

  // Deliberately store the Reversed vertex FIRST and the Forward vertex second —
  // the opposite of makeEdge's forced convention.
  Shape e = ShapeBuilder::makeEdgeWithVertices(
      line, 0.0, 2.5, {b.oriented(Orientation::Reversed), a.oriented(Orientation::Forward)});

  CC_CHECK_EQ(static_cast<long long>(e.tshape()->children().size()), 2);
  // Order preserved: child 0 is the vertex we passed first (b), child 1 is a.
  CC_CHECK(e.tshape()->children()[0].isSameGeometry(b));
  CC_CHECK(e.tshape()->children()[1].isSameGeometry(a));
  // Orientation preserved verbatim (NOT forced Forward/Reversed by position).
  CC_CHECK(e.tshape()->children()[0].orientation() == Orientation::Reversed);
  CC_CHECK(e.tshape()->children()[1].orientation() == Orientation::Forward);

  // Curve/range still read back correctly.
  auto c = curveOf(e);
  CC_CHECK(c.has_value());
  CC_CHECK(c->first == 0.0 && c->last == 2.5);

  // Null vertices are skipped, not stored.
  Shape e2 = ShapeBuilder::makeEdgeWithVertices(line, 0.0, 1.0, {a, Shape{}, b});
  CC_CHECK_EQ(static_cast<long long>(e2.tshape()->children().size()), 2);
}

CC_TEST(accessor_face_surface_and_pcurve) {
  Shape v = ShapeBuilder::makeVertex({0, 0, 0});
  Shape e = lineEdge(v, v);
  FaceSurface plane{};
  plane.kind = FaceSurface::Kind::Plane;
  Shape face = ShapeBuilder::makeFace(plane, ShapeBuilder::makeWire({e}));

  auto s = surfaceOf(face);
  CC_CHECK(s.has_value());
  CC_CHECK(s->surface->kind == FaceSurface::Kind::Plane);
  CC_CHECK(face.tshape()->hasOuterWire());
  CC_CHECK(face.tshape()->children()[0].type() == ShapeType::Wire);

  // pcurve attaches immutably: the new edge carries it, the original does not.
  PCurve pc{};
  pc.kind = EdgeCurve::Kind::Line;
  Shape e2 = ShapeBuilder::addPCurve(e, face.tshape(), pc);
  CC_CHECK(pcurveOf(e2, face) != nullptr);
  CC_CHECK(pcurveOf(e, face) == nullptr);
}

// ─────────────────────────────────────────────────────────────────────────────
// Explorer visits the expected count per type on the closed box.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(explorer_counts_per_type) {
  Box b = makeBox();
  CC_CHECK_EQ(countOf(b.solid, ShapeType::Solid), 1);
  CC_CHECK_EQ(countOf(b.solid, ShapeType::Shell), 1);
  CC_CHECK_EQ(countOf(b.solid, ShapeType::Face), 6);
  CC_CHECK_EQ(countOf(b.solid, ShapeType::Wire), 6);   // one outer wire per face
  CC_CHECK_EQ(countOf(b.solid, ShapeType::Edge), 12);  // SHARED edges
  CC_CHECK_EQ(countOf(b.solid, ShapeType::Vertex), 8); // SHARED vertices

  // Explorer 'avoid': no vertex lies outside an edge (all vertices are under
  // edges), so avoiding Edge yields zero vertices.
  Explorer vNoEdge(b.solid, ShapeType::Vertex, ShapeType::Edge);
  CC_CHECK_EQ(static_cast<long long>(vNoEdge.all().size()), 0);

  // Avoiding a less-complex type has no effect (matches TopExp): avoiding Vertex
  // while finding Edge still returns all edges.
  Explorer eAvoidV(b.solid, ShapeType::Edge, ShapeType::Vertex);
  CC_CHECK_EQ(static_cast<long long>(eAvoidV.all().size()), 12);
}

// ─────────────────────────────────────────────────────────────────────────────
// Euler-Poincaré: closed box (genus 0) satisfies V - E + F = 2.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(euler_poincare_box_genus0) {
  Box b = makeBox();
  const long long V = countOf(b.solid, ShapeType::Vertex);
  const long long E = countOf(b.solid, ShapeType::Edge);
  const long long F = countOf(b.solid, ShapeType::Face);
  CC_CHECK_EQ(V, 8);
  CC_CHECK_EQ(E, 12);
  CC_CHECK_EQ(F, 6);
  CC_CHECK_EQ(V - E + F, 2);  // χ = 2 - 2·genus, genus 0
}

// ─────────────────────────────────────────────────────────────────────────────
// Manifold connectivity: every edge of the closed box is shared by exactly two
// faces (edge → faces ancestry).
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(manifold_edge_shared_by_two_faces) {
  Box b = makeBox();
  AncestryMap ef(b.solid, ShapeType::Edge, ShapeType::Face);
  const ShapeMap& edges = ef.subShapes();
  CC_CHECK_EQ(static_cast<long long>(edges.size()), 12);

  bool allTwo = true;
  for (int id = 1; id <= static_cast<int>(edges.size()); ++id) {
    if (ef.parentsByIndex(id).size() != 2) allTwo = false;
  }
  CC_CHECK(allTwo);  // manifold: each edge bounds exactly two faces

  // Spot-check a specific shared edge from the box directly.
  CC_CHECK_EQ(static_cast<long long>(ef.parentsOf(b.e[0]).size()), 2);
}

// ─────────────────────────────────────────────────────────────────────────────
// Vertex → face ancestry on the box: each corner is shared by exactly 3 faces.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(vertex_shared_by_three_faces) {
  Box b = makeBox();
  AncestryMap vf(b.solid, ShapeType::Vertex, ShapeType::Face);
  const ShapeMap& verts = vf.subShapes();
  CC_CHECK_EQ(static_cast<long long>(verts.size()), 8);
  bool allThree = true;
  for (int id = 1; id <= static_cast<int>(verts.size()); ++id)
    if (vf.parentsByIndex(id).size() != 3) allThree = false;
  CC_CHECK(allThree);
}

// ─────────────────────────────────────────────────────────────────────────────
// Sub-shape ids: 1-based, unique, and DETERMINISTIC across repeated MapShapes
// runs on the same shape.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(shape_ids_stable_unique_deterministic) {
  Box b = makeBox();

  ShapeMap run1 = mapShapes(b.solid, ShapeType::Face);
  ShapeMap run2 = mapShapes(b.solid, ShapeType::Face);
  CC_CHECK_EQ(static_cast<long long>(run1.size()), 6);
  CC_CHECK_EQ(static_cast<long long>(run2.size()), 6);

  // 1-based and self-consistent: findIndex(shape(i)) == i for all i.
  bool stable = true;
  for (int i = 1; i <= 6; ++i)
    if (run1.findIndex(run1.shape(i)) != i) stable = false;
  CC_CHECK(stable);

  // Deterministic across runs: id i refers to the SAME face (isSame) in both.
  bool deterministic = true;
  for (int i = 1; i <= 6; ++i)
    if (!run1.shape(i).isSame(run2.shape(i))) deterministic = false;
  CC_CHECK(deterministic);

  // Unique: adding an already-present face returns its existing id, no growth.
  const long long before = static_cast<long long>(run1.size());
  const int reAdd = run1.add(run1.shape(3));
  CC_CHECK_EQ(reAdd, 3);
  CC_CHECK_EQ(static_cast<long long>(run1.size()), before);

  // Determinism also holds for edges and vertices.
  ShapeMap e1 = mapShapes(b.solid, ShapeType::Edge);
  ShapeMap e2 = mapShapes(b.solid, ShapeType::Edge);
  bool edgesDet = e1.size() == e2.size();
  for (int i = 1; edgesDet && i <= static_cast<int>(e1.size()); ++i)
    if (!e1.shape(i).isSame(e2.shape(i))) edgesDet = false;
  CC_CHECK(edgesDet);
}

// ─────────────────────────────────────────────────────────────────────────────
// Orientation: reversing a face reverses its contribution. A reversed face is
// isSame (same node+location) as the forward one but NOT isEqual (orientation
// differs), and its cumulative orientation flips — exactly how a face's material
// side is selected in the boundary.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(orientation_reversing_a_face) {
  Box b = makeBox();
  const Shape& forwardFace = b.f[0];
  Shape reversedFace = forwardFace.reversedShape();

  CC_CHECK(forwardFace.orientation() == Orientation::Forward);
  CC_CHECK(reversedFace.orientation() == Orientation::Reversed);
  CC_CHECK(forwardFace.isSame(reversedFace));    // same node + location
  CC_CHECK(!forwardFace.isEqual(reversedFace));  // orientation differs
  CC_CHECK(forwardFace.isSameGeometry(reversedFace));

  // Reversing twice returns to Forward (involution).
  CC_CHECK(reversedFace.reversedShape().orientation() == Orientation::Forward);

  // The reversed face contributes the OPPOSITE cumulative orientation to its
  // edges. Compose the face orientation with a Forward child edge: Forward
  // parent keeps it Forward, Reversed parent flips it to Reversed.
  CC_CHECK(composed(forwardFace.orientation(), Orientation::Forward) == Orientation::Forward);
  CC_CHECK(composed(reversedFace.orientation(), Orientation::Forward) == Orientation::Reversed);
}

// ─────────────────────────────────────────────────────────────────────────────
// Euler-Poincaré: a solid with one through-hole (a handle, genus 1) satisfies
// V - E + F = 0.
//
// The model is a "picture-frame": a box with a square prismatic hole punched all
// the way through. Its closed manifold boundary is a torus (genus 1). Built from
// a SHARED pool of vertices and edges (exactly like the box), so the explorer's
// deduped counts ARE the Euler numbers — V=16, E=32, F=16 → χ = V-E+F = 0.
//
// Two nested square rings (outer + inner hole) at z=0 and z=1: 8 vertices per
// level = 16. Edges = 8 lower-ring + 8 upper-ring + 8 verticals + 8 cap-radials
// = 32. Faces = 4 outer walls + 4 inner (hole) walls + 4 lower-cap annulus quads
// + 4 upper-cap annulus quads = 16.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(euler_poincare_handle_genus1) {
  // 16 shared vertices: index [level*8 + slot], slot 0..3 outer, 4..7 inner.
  auto vId = [](int level, int slot) { return static_cast<std::size_t>(level * 8 + slot); };
  std::array<Shape, 16> v;
  const double outer[4][2] = {{-2, -2}, {2, -2}, {2, 2}, {-2, 2}};
  const double inner[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
  for (int lvl = 0; lvl < 2; ++lvl) {
    const double z = lvl;
    for (int i = 0; i < 4; ++i) v[vId(lvl, i)] = ShapeBuilder::makeVertex({outer[i][0], outer[i][1], z});
    for (int i = 0; i < 4; ++i) v[vId(lvl, 4 + i)] = ShapeBuilder::makeVertex({inner[i][0], inner[i][1], z});
  }
  auto V = [&](int lvl, int slot) -> const Shape& { return v[vId(lvl, slot)]; };

  // 32 shared edges, built once and reused by adjacent faces.
  std::vector<Shape> edges;
  auto edge = [&](const Shape& a, const Shape& b) {
    edges.push_back(lineEdge(a, b));
    return edges.back();
  };
  // Ring edges (outer + inner) on each level: 16.
  std::array<std::array<Shape, 8>, 2> ringE;  // [level][slot] = edge slot→slot+1
  for (int lvl = 0; lvl < 2; ++lvl) {
    for (int i = 0; i < 4; ++i)
      ringE[static_cast<std::size_t>(lvl)][static_cast<std::size_t>(i)] = edge(V(lvl, i), V(lvl, (i + 1) % 4));
    for (int i = 0; i < 4; ++i)
      ringE[static_cast<std::size_t>(lvl)][static_cast<std::size_t>(4 + i)] =
          edge(V(lvl, 4 + i), V(lvl, 4 + (i + 1) % 4));
  }
  // Verticals lo→hi for all 8 slots: 8.
  std::array<Shape, 8> vert;
  for (int i = 0; i < 8; ++i) vert[static_cast<std::size_t>(i)] = edge(V(0, i), V(1, i));
  // Cap radials outer[i]→inner[i] on each level: 8.
  std::array<std::array<Shape, 4>, 2> radial;
  for (int lvl = 0; lvl < 2; ++lvl)
    for (int i = 0; i < 4; ++i)
      radial[static_cast<std::size_t>(lvl)][static_cast<std::size_t>(i)] = edge(V(lvl, i), V(lvl, 4 + i));

  // 16 faces from the shared edges. Orientation is not under test here, so each
  // face's wire reuses the shared edge NODES (reversed where traversed backward).
  FaceSurface plane{};
  plane.kind = FaceSurface::Kind::Plane;
  std::vector<Shape> faces;
  auto R = [](const Shape& s) { return s.reversedShape(); };
  auto face4 = [&](const Shape& a, const Shape& b, const Shape& c, const Shape& d) {
    faces.push_back(ShapeBuilder::makeFace(plane, ShapeBuilder::makeWire({a, b, c, d})));
  };
  auto ring = [&](int lvl, int i) -> const Shape& { return ringE[static_cast<std::size_t>(lvl)][static_cast<std::size_t>(i)]; };
  auto rad = [&](int lvl, int i) -> const Shape& { return radial[static_cast<std::size_t>(lvl)][static_cast<std::size_t>(i)]; };

  for (int i = 0; i < 4; ++i) {  // 4 outer walls: lower-ring, vertical, upper-ring, vertical
    face4(ring(0, i), vert[static_cast<std::size_t>((i + 1) % 4)], R(ring(1, i)), R(vert[static_cast<std::size_t>(i)]));
  }
  for (int i = 0; i < 4; ++i) {  // 4 inner (hole) walls
    face4(ring(0, 4 + i), vert[static_cast<std::size_t>(4 + (i + 1) % 4)], R(ring(1, 4 + i)),
          R(vert[static_cast<std::size_t>(4 + i)]));
  }
  for (int i = 0; i < 4; ++i) {  // 4 lower-cap annulus quads
    face4(ring(0, i), rad(0, (i + 1) % 4), R(ring(0, 4 + i)), R(rad(0, i)));
  }
  for (int i = 0; i < 4; ++i) {  // 4 upper-cap annulus quads
    face4(ring(1, i), rad(1, (i + 1) % 4), R(ring(1, 4 + i)), R(rad(1, i)));
  }

  Shape shell = ShapeBuilder::makeShell(faces);
  Shape solid = ShapeBuilder::makeSolid({shell});

  // Deduped counts ARE the Euler numbers (shared vertices + edges).
  const long long nV = countOf(solid, ShapeType::Vertex);
  const long long nE = countOf(solid, ShapeType::Edge);
  const long long nF = countOf(solid, ShapeType::Face);
  CC_CHECK_EQ(nV, 16);
  CC_CHECK_EQ(nE, 32);
  CC_CHECK_EQ(nF, 16);
  CC_CHECK_EQ(nV - nE + nF, 0);              // χ = 2 - 2·genus → genus 1
  CC_CHECK_EQ(static_cast<long long>(edges.size()), 32);

  // Manifold: every edge of the handle is still shared by exactly two faces.
  AncestryMap ef(solid, ShapeType::Edge, ShapeType::Face);
  bool allTwo = true;
  for (int id = 1; id <= static_cast<int>(ef.subShapes().size()); ++id)
    if (ef.parentsByIndex(id).size() != 2) allTwo = false;
  CC_CHECK(allTwo);
}

CC_RUN_ALL()
