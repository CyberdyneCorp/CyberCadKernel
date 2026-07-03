// SPDX-License-Identifier: Apache-2.0
//
// step_writer.cpp — native ISO-10303-21 (STEP AP203) writer implementation.
//
// See step_writer.h for scope + the clean-room provenance note. This file walks
// the native topology graph face-by-face, assigns #ids to the Part-42 entity
// instances, deduplicates the leaf geometry (CARTESIAN_POINT / DIRECTION /
// VERTEX_POINT / EDGE_CURVE) that adjacent faces share, and streams the HEADER +
// DATA sections to text.
//
// OCCT-FREE. clang++ -std=c++20.

#include "native/exchange/step_writer.h"

#include "native/math/native_math.h"
#include "native/topology/accessors.h"
#include "native/topology/explore.h"
#include "native/topology/shape.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace cybercad::native::exchange {

namespace math = cybercad::native::math;

namespace {

// ── World placement helpers ─────────────────────────────────────────────────
// A Shape carried out of the Explorer already has its world Location baked in.
// Geometry accessors return LOCAL geometry + that Location; these helpers place a
// point / direction / frame into world coordinates so the emitted STEP is in true
// world mm regardless of any Location on the solid.

math::Point3 worldPoint(const topo::Location& loc, const math::Point3& p) {
  return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
}
math::Dir3 worldDir(const topo::Location& loc, const math::Dir3& d) {
  return loc.isIdentity() ? d : loc.transform().applyToDir(d);
}
math::Ax3 worldFrame(const topo::Location& loc, const math::Ax3& f) {
  if (loc.isIdentity()) return f;
  return math::Ax3{worldPoint(loc, f.origin), worldDir(loc, f.x), worldDir(loc, f.y),
                   worldDir(loc, f.z)};
}

// ── Number formatting ───────────────────────────────────────────────────────
// STEP reals are C-locale decimals that MUST contain a '.' (a bare integer is a
// STEP integer, not a real). 17 significant digits round-trips fp64 exactly.
std::string real(double v) {
  if (v == 0.0) v = 0.0;  // normalise -0.0 → 0.0
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.17g", v);
  std::string s(buf);
  // Ensure a decimal point / exponent so the token parses as a REAL.
  if (s.find('.') == std::string::npos && s.find('e') == std::string::npos &&
      s.find('E') == std::string::npos && s.find("inf") == std::string::npos &&
      s.find("nan") == std::string::npos)
    s += ".";
  return s;
}

// STEP string literal: single-quoted, embedded quotes doubled.
std::string str(const std::string& s) {
  std::string out = "'";
  for (char c : s) {
    if (c == '\'') out += "''";
    else out += c;
  }
  out += "'";
  return out;
}

// ── The DATA-section writer: assigns ids and streams entity instances ────────
// One instance per emit() call, returning its #id. Dedup maps key the shared leaf
// geometry so a cube's shared vertices/points/edges are written once.
class DataWriter {
 public:
  // Append an entity body (e.g. "CARTESIAN_POINT('',(0.,0.,0.))") and return its
  // freshly-assigned #id.
  int emit(const std::string& body) {
    const int id = nextId_++;
    lines_.push_back("#" + std::to_string(id) + " = " + body + ";");
    return id;
  }

  const std::vector<std::string>& lines() const noexcept { return lines_; }

  // Deduplicated CARTESIAN_POINT. Points are keyed on their rounded coordinates so
  // two faces meeting at a shared vertex reference the same #.
  int point(const math::Point3& p) {
    const std::string key = coordKey(p.x, p.y, p.z);
    if (auto it = pointCache_.find(key); it != pointCache_.end()) return it->second;
    const int id = emit("CARTESIAN_POINT('',(" + real(p.x) + "," + real(p.y) + "," + real(p.z) +
                        "))");
    pointCache_.emplace(key, id);
    return id;
  }

  // Deduplicated DIRECTION.
  int direction(const math::Dir3& d) {
    const std::string key = coordKey(d.x(), d.y(), d.z());
    if (auto it = dirCache_.find(key); it != dirCache_.end()) return it->second;
    const int id = emit("DIRECTION('',(" + real(d.x()) + "," + real(d.y()) + "," + real(d.z()) +
                        "))");
    dirCache_.emplace(key, id);
    return id;
  }

  // AXIS2_PLACEMENT_3D from a world frame: location + axis (Z) + ref_direction (X).
  int placement(const math::Ax3& f) {
    const int loc = point(f.origin);
    const int axis = direction(f.z);
    const int ref = direction(f.x);
    return emit("AXIS2_PLACEMENT_3D('',#" + std::to_string(loc) + ",#" + std::to_string(axis) +
                ",#" + std::to_string(ref) + ")");
  }

 private:
  // Round to a stable grid so fp noise below the modelling tolerance does not
  // defeat point/direction sharing. 1e-9 mm is far below any real feature size.
  static std::string coordKey(double x, double y, double z) {
    auto q = [](double v) {
      double r = std::round(v / 1e-9) * 1e-9;
      if (r == 0.0) r = 0.0;
      char b[32];
      std::snprintf(b, sizeof(b), "%.12g", r);
      return std::string(b);
    };
    return q(x) + "|" + q(y) + "|" + q(z);
  }

  int nextId_ = 1;
  std::vector<std::string> lines_;
  std::unordered_map<std::string, int> pointCache_;
  std::unordered_map<std::string, int> dirCache_;
};

// ── Geometry-kind capability check ───────────────────────────────────────────
bool curveKindSupported(topo::EdgeCurve::Kind k) {
  using K = topo::EdgeCurve::Kind;
  return k == K::Line || k == K::Circle || k == K::BSpline;
}
bool surfaceKindSupported(topo::FaceSurface::Kind k) {
  using K = topo::FaceSurface::Kind;
  return k == K::Plane || k == K::Cylinder || k == K::Cone || k == K::Sphere || k == K::BSpline;
}

// ── Flat knot vector → (distinct knots, multiplicities) for *_WITH_KNOTS ──────
struct KnotData {
  std::vector<double> values;
  std::vector<int> mults;
};
KnotData compressKnots(const std::vector<double>& flat) {
  KnotData out;
  for (double u : flat) {
    if (!out.values.empty() && std::abs(u - out.values.back()) <= 1e-12) {
      ++out.mults.back();
    } else {
      out.values.push_back(u);
      out.mults.push_back(1);
    }
  }
  return out;
}

// A list of #ids as "(#a,#b,#c)".
std::string idList(const std::vector<int>& ids) {
  std::string s = "(";
  for (std::size_t i = 0; i < ids.size(); ++i) {
    if (i) s += ",";
    s += "#" + std::to_string(ids[i]);
  }
  s += ")";
  return s;
}

// A STEP list "(a,b,c)" of the elements of `v`, each rendered by `fmt`.
template <class T, class Fmt>
std::string listOf(const std::vector<T>& v, Fmt fmt) {
  std::string s = "(";
  for (std::size_t i = 0; i < v.size(); ++i) {
    if (i) s += ",";
    s += fmt(v[i]);
  }
  return s + ")";
}
std::string multList(const std::vector<int>& m) {
  return listOf(m, [](int x) { return std::to_string(x); });
}
std::string knotList(const std::vector<double>& k) {
  return listOf(k, [](double x) { return real(x); });
}

// ─────────────────────────────────────────────────────────────────────────────
// StepBuilder — the walk. Emits the geometry graph for one solid.
//
// Cognitive complexity note (systems band, openspec/NATIVE-REWRITE.md): the
// per-face / per-edge / per-curve dispatch is an irreducible switch over the
// geometry kinds. Each method stays small and single-purpose (≤ ~12); the class
// splits the walk into vertex / curve / edge / loop / surface / face methods so no
// single function carries the whole graph.
// ─────────────────────────────────────────────────────────────────────────────
class StepBuilder {
 public:
  explicit StepBuilder(DataWriter& w) : w_(w) {}

  // VERTEX_POINT for a (world-placed) vertex, deduplicated GEOMETRICALLY by its
  // CARTESIAN_POINT id. The native builders may emit distinct vertex NODES at the
  // same physical corner (per-patch vertices, NATIVE-REWRITE.md #4); collapsing them
  // to one VERTEX_POINT is what lets the edge-sharing key (below) merge the two
  // faces' coincident edges into one manifold EDGE_CURVE. w_.point() already dedups
  // coincident coordinates, so keying the VERTEX_POINT on that #id is exact.
  int vertex(const topo::Shape& v) {
    const auto p = topo::pointOf(v);  // pointOf already applies the vertex Location
    return vertexForPoint(w_.point(p ? *p : math::Point3{}));
  }

  // VERTEX_POINT for an already-emitted CARTESIAN_POINT #id, deduplicated on it.
  // (Shared with the seam builder, which manufactures vertices from raw points.)
  int vertexForPoint(int pointId) {
    if (auto it = vertexCache_.find(pointId); it != vertexCache_.end()) return it->second;
    const int id = w_.emit("VERTEX_POINT('',#" + std::to_string(pointId) + ")");
    vertexCache_.emplace(pointId, id);
    return id;
  }

  // The 3D curve entity (LINE / CIRCLE / B_SPLINE_CURVE_WITH_KNOTS) of an edge.
  int curve(const topo::EdgeCurve& c, const topo::Location& loc) {
    using K = topo::EdgeCurve::Kind;
    switch (c.kind) {
      case K::Line:   return lineCurve(c, loc);
      case K::Circle: return circleCurve(c, loc);
      case K::BSpline: return bsplineCurve(c, loc);
      default:        return 0;  // guarded by canSerialize
    }
  }

  // EDGE_CURVE (with its two VERTEX_POINTs + 3D curve). Deduplicated GEOMETRICALLY:
  // the native builders emit PER-FACE edges (edge-node sharing is deferred — see
  // NATIVE-REWRITE.md #4), so the two faces meeting at a physical edge carry two
  // distinct edge NODES that coincide in space. STEP wants ONE EDGE_CURVE shared by
  // both ADVANCED_FACEs to form a properly-sewn manifold CLOSED_SHELL (so the file
  // re-reads through OCCT as a valid solid, not a heap of coincident faces). We
  // therefore key the cache on the physical edge — its (unordered) endpoint vertex
  // ids plus a curve-geometry signature — and reuse the same EDGE_CURVE for the
  // second face. The ORIENTED_EDGE (emitted per wire) still records each face's
  // traversal sense, so the shared edge is used forward on one face, reversed on the
  // other, exactly as a manifold shell requires.
  int edgeCurve(const topo::Shape& edge) {
    const auto cr = topo::curveOf(edge);
    // Bounding vertices in stored order (start=Forward, end=Reversed by convention).
    int v0 = 0, v1 = 0;
    const auto& kids = edge.tshape()->children();
    if (!kids.empty()) v0 = vertex(kids.front().located(edge.location()));
    if (kids.size() >= 2) v1 = vertex(kids.back().located(edge.location()));
    if (v1 == 0) v1 = v0;  // degenerate/closed edge — same vertex both ends

    const std::string gkey = edgeGeomKey(v0, v1, cr ? cr->curve : nullptr, cr ? cr->location
                                                                             : topo::Location{});
    if (auto it = edgeGeomCache_.find(gkey); it != edgeGeomCache_.end()) return it->second;

    const int crv = cr ? curve(*cr->curve, cr->location) : 0;
    const int id = w_.emit("EDGE_CURVE('',#" + std::to_string(v0) + ",#" + std::to_string(v1) +
                           ",#" + std::to_string(crv) + ",.T.)");
    edgeGeomCache_.emplace(gkey, id);
    return id;
  }

  // ORIENTED_EDGE wrapping an EDGE_CURVE with the edge's orientation in the wire.
  int orientedEdge(const topo::Shape& edge) {
    const int ec = edgeCurve(edge);
    const bool forward = edge.orientation() == topo::Orientation::Forward;
    return w_.emit(std::string("ORIENTED_EDGE('',*,*,#") + std::to_string(ec) + "," +
                   (forward ? ".T." : ".F.") + ")");
  }

  // EDGE_LOOP for a wire (the wire's edges, in order, each as an ORIENTED_EDGE).
  // `periodicSurface` is true when the wire bounds a full-turn periodic analytic
  // surface (cylinder/cone/sphere), which needs a SEAM edge (see wallLoopWithSeam).
  int edgeLoop(const topo::Shape& wire, bool periodicSurface = false) {
    std::vector<topo::Shape> placedEdges;
    for (const topo::Shape& e : wire.tshape()->children()) {
      // Compose the wire's placement/orientation onto the edge (world-placed).
      placedEdges.push_back(
          e.located(wire.location())
              .oriented(topo::composed(wire.orientation(), e.orientation())));
    }

    // A full-turn periodic wall (e.g. a cylindrical hole wall) reaches the writer as
    // a loop of two CLOSED full-circle rim edges with NO seam. A periodic STEP
    // surface trimmed to its full parametric period is only a valid bounded face if
    // the loop carries a SEAM edge (a curve appearing once at u=0 and once at
    // u=period). Without it OCCT's reader cannot close the periodic region and reads
    // the face back with zero area (an invalid, leaky solid). We therefore synthesise
    // the seam here from the two rim circles' seam vertices.
    if (periodicSurface) {
      if (const int seamLoop = wallLoopWithSeam(placedEdges); seamLoop != 0) return seamLoop;
    }

    std::vector<int> oriented;
    oriented.reserve(placedEdges.size());
    for (const topo::Shape& e : placedEdges) oriented.push_back(orientedEdge(e));
    return w_.emit("EDGE_LOOP(''," + idList(oriented) + ")");
  }

  // If `edges` is a full-turn periodic wall loop — exactly two CLOSED full-circle
  // edges (start vertex == end vertex) at two heights — emit the STEP EDGE_LOOP with
  // the required SEAM edge and return its #id; otherwise return 0 (caller emits the
  // plain loop). The seam is a straight LINE joining the two rims' seam vertices,
  // referenced once forward and once reversed so the periodic face is closed at both
  // u=0 and u=period, exactly as STEPControl_Writer emits a cylindrical hole wall.
  int wallLoopWithSeam(const std::vector<topo::Shape>& edges) {
    if (edges.size() != 2) return 0;
    for (const topo::Shape& e : edges)
      if (!isClosedCircle(e)) return 0;

    // Seam runs from the first rim's seam vertex to the second rim's seam vertex.
    const auto p0 = seamPoint(edges[0]);
    const auto p1 = seamPoint(edges[1]);
    if (!p0 || !p1) return 0;

    const int v0 = w_.point(*p0);
    const int v1 = w_.point(*p1);
    const int vp0 = vertexForPoint(v0);
    const int vp1 = vertexForPoint(v1);
    const int seam = seamEdgeCurve(*p0, *p1, vp0, vp1);

    // Loop: circle0, seam(forward), circle1, seam(reversed).
    const int oe0 = orientedEdge(edges[0]);
    const int seamF = w_.emit("ORIENTED_EDGE('',*,*,#" + std::to_string(seam) + ",.T.)");
    const int oe1 = orientedEdge(edges[1]);
    const int seamR = w_.emit("ORIENTED_EDGE('',*,*,#" + std::to_string(seam) + ",.F.)");
    return w_.emit("EDGE_LOOP(''," + idList({oe0, seamF, oe1, seamR}) + ")");
  }

  // The surface entity (PLANE / CYLINDRICAL / CONICAL / SPHERICAL / B_SPLINE) of a
  // face, world-placed.
  int surface(const topo::FaceSurface& s, const topo::Location& loc) {
    using K = topo::FaceSurface::Kind;
    switch (s.kind) {
      case K::Plane:    return w_.emit("PLANE('',#" + std::to_string(placement(s, loc)) + ")");
      case K::Cylinder: return w_.emit("CYLINDRICAL_SURFACE('',#" +
                                       std::to_string(placement(s, loc)) + "," + real(s.radius) +
                                       ")");
      case K::Cone:     return coneSurface(s, loc);
      case K::Sphere:   return w_.emit("SPHERICAL_SURFACE('',#" +
                                       std::to_string(placement(s, loc)) + "," + real(s.radius) +
                                       ")");
      case K::BSpline:  return bsplineSurface(s, loc);
      default:          return 0;  // guarded by canSerialize
    }
  }

  // ADVANCED_FACE: outer FACE_OUTER_BOUND + inner FACE_BOUNDs + the surface.
  int advancedFace(const topo::Shape& face) {
    const auto sr = topo::surfaceOf(face);
    const int srf = sr ? surface(*sr->surface, sr->location) : 0;
    // A full-turn cylinder / cone / sphere wall needs a seam edge in its loop.
    const bool periodic = sr && sr->surface && isPeriodicSurfaceKind(sr->surface->kind);

    std::vector<int> bounds;
    const auto& wires = face.tshape()->children();
    for (std::size_t i = 0; i < wires.size(); ++i) {
      const topo::Shape wire = wires[i]
                                   .located(face.location())
                                   .oriented(topo::composed(face.orientation(), wires[i].orientation()));
      const int loop = edgeLoop(wire, periodic);
      const char* kind = (i == 0) ? "FACE_OUTER_BOUND" : "FACE_BOUND";
      bounds.push_back(
          w_.emit(std::string(kind) + "('',#" + std::to_string(loop) + ",.T.)"));
    }
    const bool sameSense = face.orientation() == topo::Orientation::Forward;
    return w_.emit("ADVANCED_FACE(''," + idList(bounds) + ",#" + std::to_string(srf) + "," +
                   (sameSense ? ".T." : ".F.") + ")");
  }

 private:
  DataWriter& w_;

  std::unordered_map<int, int> vertexCache_;  // CARTESIAN_POINT id → VERTEX_POINT id
  std::unordered_map<std::string, int> edgeGeomCache_;

  // A geometric identity key for a PHYSICAL edge: the unordered pair of endpoint
  // vertex ids (already coordinate-deduped) plus a curve signature so two DIFFERENT
  // curves between the same two vertices (a chord vs an arc) do not collapse. Lines
  // are fully identified by their endpoints; circles/arcs add radius + centre; a
  // spline adds its first interior pole (endpoints already pin the ends).
  static std::string edgeGeomKey(int v0, int v1, const topo::EdgeCurve* c,
                                 const topo::Location& loc) {
    const int lo = v0 < v1 ? v0 : v1;
    const int hi = v0 < v1 ? v1 : v0;
    std::string k = std::to_string(lo) + "_" + std::to_string(hi) + "_";
    if (!c) return k + "null";
    using K = topo::EdgeCurve::Kind;
    switch (c->kind) {
      case K::Line: return k + "L";
      case K::Circle: case K::Ellipse: {
        const math::Ax3 f = worldFrame(loc, c->frame);
        char b[96];
        std::snprintf(b, sizeof(b), "C%.9g@%.6g,%.6g,%.6g", c->radius, f.origin.x, f.origin.y,
                      f.origin.z);
        return k + b;
      }
      case K::BSpline: case K::Bezier: {
        char b[96];
        const math::Point3 p = c->poles.empty()
                                   ? math::Point3{}
                                   : worldPoint(loc, c->poles[c->poles.size() / 2]);
        std::snprintf(b, sizeof(b), "S%d#%zu@%.6g,%.6g,%.6g", c->degree, c->poles.size(), p.x, p.y,
                      p.z);
        return k + b;
      }
    }
    return k;
  }

  // AXIS2_PLACEMENT_3D from an analytic surface's world frame.
  int placement(const topo::FaceSurface& s, const topo::Location& loc) {
    return w_.placement(worldFrame(loc, s.frame));
  }

  // ── Seam support for full-turn periodic walls ────────────────────────────────
  // A periodic analytic surface (cylinder/cone/sphere) trimmed to its full
  // parametric period needs a seam edge in its EDGE_LOOP (see wallLoopWithSeam).
  static bool isPeriodicSurfaceKind(topo::FaceSurface::Kind k) {
    using K = topo::FaceSurface::Kind;
    return k == K::Cylinder || k == K::Cone || k == K::Sphere;
  }

  // A CLOSED full-circle edge = a Circle-kind edge whose two bounding vertices are
  // the SAME physical point (start == end, the native rim seam vertex).
  static bool isClosedCircle(const topo::Shape& edge) {
    const auto cr = topo::curveOf(edge);
    if (!cr || !cr->curve || cr->curve->kind != topo::EdgeCurve::Kind::Circle) return false;
    const auto& kids = edge.tshape()->children();
    if (kids.empty()) return false;
    const auto a = topo::pointOf(kids.front().located(edge.location()));
    const auto b = topo::pointOf(kids.back().located(edge.location()));
    if (!a || !b) return false;
    return math::distance(*a, *b) <= 1e-9;
  }

  // The world seam point of a closed rim circle edge (its shared start/end vertex).
  static std::optional<math::Point3> seamPoint(const topo::Shape& edge) {
    const auto& kids = edge.tshape()->children();
    if (kids.empty()) return std::nullopt;
    return topo::pointOf(kids.front().located(edge.location()));
  }

  // A straight seam EDGE_CURVE between the two rim seam vertices, deduplicated so the
  // one physical seam is shared (used forward at u=period, reversed at u=0).
  int seamEdgeCurve(const math::Point3& a, const math::Point3& b, int va, int vb) {
    const std::string key = "SEAM_" + std::to_string(va < vb ? va : vb) + "_" +
                            std::to_string(va < vb ? vb : va);
    if (auto it = edgeGeomCache_.find(key); it != edgeGeomCache_.end()) return it->second;
    const math::Vec3 dv = b - a;
    const double len = math::norm(dv);
    const math::Dir3 dir = len > 0.0 ? math::Dir3{dv} : math::Dir3{0, 0, 1};
    const int pnt = w_.point(a);
    const int dirId = w_.direction(dir);
    const int vec = w_.emit("VECTOR('',#" + std::to_string(dirId) + ",1.)");
    const int line = w_.emit("LINE('',#" + std::to_string(pnt) + ",#" + std::to_string(vec) + ")");
    const int ec = w_.emit("EDGE_CURVE('',#" + std::to_string(va) + ",#" + std::to_string(vb) +
                           ",#" + std::to_string(line) + ",.T.)");
    edgeGeomCache_.emplace(key, ec);
    return ec;
  }

  // ── Curve emitters ─────────────────────────────────────────────────────────
  // LINE = point + VECTOR(direction, magnitude). The native Line frame stores the
  // origin + X = direction; magnitude 1 keeps the parametrisation unit-speed-ish
  // (the EDGE_CURVE vertices trim it, so the exact magnitude is not load-bearing).
  int lineCurve(const topo::EdgeCurve& c, const topo::Location& loc) {
    const math::Ax3 f = worldFrame(loc, c.frame);
    const int pnt = w_.point(f.origin);
    const int dir = w_.direction(f.x);
    const int vec = w_.emit("VECTOR('',#" + std::to_string(dir) + ",1.)");
    return w_.emit("LINE('',#" + std::to_string(pnt) + ",#" + std::to_string(vec) + ")");
  }

  int circleCurve(const topo::EdgeCurve& c, const topo::Location& loc) {
    const int plc = w_.placement(worldFrame(loc, c.frame));
    return w_.emit("CIRCLE('',#" + std::to_string(plc) + "," + real(c.radius) + ")");
  }

  // B_SPLINE_CURVE_WITH_KNOTS. Non-rational only (the native kernel's spline edges
  // carry no weights); a rational spline would need
  // RATIONAL_B_SPLINE_CURVE — out of the current native scope (canSerialize keeps
  // it native only when weights are empty; a weighted spline falls back to OCCT).
  int bsplineCurve(const topo::EdgeCurve& c, const topo::Location& loc) {
    std::vector<int> poleIds;
    poleIds.reserve(c.poles.size());
    for (const math::Point3& p : c.poles) poleIds.push_back(w_.point(worldPoint(loc, p)));
    const KnotData kd = compressKnots(c.knots);
    return w_.emit("B_SPLINE_CURVE_WITH_KNOTS(''," + std::to_string(c.degree) + "," +
                   idList(poleIds) + ",.UNSPECIFIED.,.F.,.F.," + multList(kd.mults) + "," +
                   knotList(kd.values) + ",.UNSPECIFIED.)");
  }

  // ── Surface emitters ─────────────────────────────────────────────────────────
  int coneSurface(const topo::FaceSurface& s, const topo::Location& loc) {
    return w_.emit("CONICAL_SURFACE('',#" + std::to_string(placement(s, loc)) + "," +
                   real(s.radius) + "," + real(s.semiAngle) + ")");
  }

  // The control-point net as a STEP list-of-U-rows each a list of V CARTESIAN_POINT
  // #ids. Poles are stored row-major (U outer, V inner), matching STEP's ordering.
  std::string poleNet(const topo::FaceSurface& s, const topo::Location& loc) {
    std::string grid = "(";
    for (int i = 0; i < s.nPolesU; ++i) {
      if (i) grid += ",";
      std::vector<int> row;
      row.reserve(static_cast<std::size_t>(s.nPolesV));
      for (int j = 0; j < s.nPolesV; ++j) {
        const std::size_t idx = static_cast<std::size_t>(i) * s.nPolesV + j;
        row.push_back(w_.point(worldPoint(loc, s.poles[idx])));
      }
      grid += idList(row);
    }
    return grid + ")";
  }

  // B_SPLINE_SURFACE_WITH_KNOTS (non-rational — see bsplineCurve on the weighted
  // fall-back). Poles row-major (U outer, V inner).
  int bsplineSurface(const topo::FaceSurface& s, const topo::Location& loc) {
    const std::string grid = poleNet(s, loc);
    const KnotData ku = compressKnots(s.knotsU);
    const KnotData kv = compressKnots(s.knotsV);
    return w_.emit("B_SPLINE_SURFACE_WITH_KNOTS(''," + std::to_string(s.degreeU) + "," +
                   std::to_string(s.degreeV) + "," + grid + ",.UNSPECIFIED.,.F.,.F.,.F.," +
                   multList(ku.mults) + "," + multList(kv.mults) + "," + knotList(ku.values) +
                   "," + knotList(kv.values) + ",.UNSPECIFIED.)");
  }
};

// ── canSerialize: every face surface + every edge curve must be in-scope ─────

// One face's surface is serialisable: an attached surface of a supported kind, and
// (for BSpline) a knotted — not Bezier — form.
bool faceSurfaceInScope(const topo::Shape& face) {
  const auto sr = topo::surfaceOf(face);
  if (!sr || !sr->surface) return false;
  if (!surfaceKindSupported(sr->surface->kind)) return false;
  // A Bezier-as-BSpline surface has no knot arrays → no *_WITH_KNOTS form.
  return !(sr->surface->kind == topo::FaceSurface::Kind::BSpline && sr->surface->knotsU.empty());
}

// One edge's curve is serialisable: an attached curve of a supported kind, and (for
// BSpline) a knotted, non-rational form.
bool edgeCurveInScope(const topo::Shape& edge) {
  const auto cr = topo::curveOf(edge);
  if (!cr || !cr->curve) return false;
  if (!curveKindSupported(cr->curve->kind)) return false;
  if (cr->curve->kind != topo::EdgeCurve::Kind::BSpline) return true;
  // A weighted (rational) spline / a Bezier-as-BSpline is out of scope.
  return !cr->curve->knots.empty() && cr->curve->weights.empty();
}

bool geometrySupported(const topo::Shape& solid) {
  bool anyFace = false;
  for (topo::Explorer fx(solid, topo::ShapeType::Face); fx.more(); fx.next()) {
    anyFace = true;
    if (!faceSurfaceInScope(fx.current())) return false;
  }
  bool anyEdge = false;
  for (topo::Explorer ex(solid, topo::ShapeType::Edge); ex.more(); ex.next()) {
    anyEdge = true;
    if (!edgeCurveInScope(ex.current())) return false;
  }
  return anyFace && anyEdge;
}

// The solid whose shells we serialise: accept a Solid (walk its shells) or a bare
// Shell (wrap it as the sole shell). Returns null if neither.
bool isSolidOrShell(const topo::Shape& s) {
  return !s.isNull() &&
         (s.type() == topo::ShapeType::Solid || s.type() == topo::ShapeType::Shell ||
          s.type() == topo::ShapeType::Compound || s.type() == topo::ShapeType::CompSolid);
}

// ── HEADER + product/context boilerplate ──────────────────────────────────────
// A minimal, conformant AP203 wrapper: the manifold BREP is placed in an
// (ADVANCED_)BREP_SHAPE_REPRESENTATION sharing a geometric context whose SI units
// are millimetre + radian + steradian, tied to a PRODUCT_DEFINITION_SHAPE. Mirrors
// the entity set STEPControl_Writer emits for STEPControl_AsIs, hand-written.
void writeHeader(std::ostream& os, const std::string& productName) {
  os << "ISO-10303-21;\n";
  os << "HEADER;\n";
  os << "FILE_DESCRIPTION(('CyberCadKernel native STEP AP203 export'),'2;1');\n";
  os << "FILE_NAME(" << str(productName + ".step")
     << ",'',(''),(''),'CyberCadKernel','CyberCadKernel native writer','');\n";
  os << "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\n";
  os << "ENDSEC;\n";
}

// Emit the product/context wrapper around the MANIFOLD_SOLID_BREP #brepId and
// return nothing (the wrapper's ids are internal). Uses the same DataWriter id
// stream so all references stay consistent.
void writeWrapper(DataWriter& w, int brepId, const std::string& productName) {
  // Units: mm (SI length prefix MILLI + METRE), radian, steradian.
  const int lenUnit = w.emit(
      "( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
  const int angUnit = w.emit(
      "( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
  const int solUnit = w.emit(
      "( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
  const int uncertainty =
      w.emit("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#" +
             std::to_string(lenUnit) + ",'distance_accuracy_value','confusion accuracy')");
  const int geomCtx = w.emit(
      "( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
      "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#" +
      std::to_string(uncertainty) + ")) GLOBAL_UNIT_ASSIGNED_CONTEXT((#" +
      std::to_string(lenUnit) + ",#" + std::to_string(angUnit) + ",#" + std::to_string(solUnit) +
      ")) REPRESENTATION_CONTEXT('Context #1','3D Context with UNIT and UNCERTAINTY') )");

  // The BREP shape representation carries the solid in the mm context.
  const int shapeRep = w.emit("ADVANCED_BREP_SHAPE_REPRESENTATION('',(#" +
                              std::to_string(brepId) + "),#" + std::to_string(geomCtx) + ")");

  // Product / definition chain (AP203 config-control-design).
  const int appCtx = w.emit(
      "APPLICATION_CONTEXT('configuration controlled 3D designs of "
      "mechanical parts and assemblies')");
  const int appProto = w.emit(
      "APPLICATION_PROTOCOL_DEFINITION('international standard',"
      "'config_control_design',1994,#" +
      std::to_string(appCtx) + ")");
  (void)appProto;
  const int prodCtx = w.emit("PRODUCT_CONTEXT('',#" + std::to_string(appCtx) + ",'mechanical')");
  const int product = w.emit("PRODUCT(" + str(productName) + "," + str(productName) + ",'',(#" +
                             std::to_string(prodCtx) + "))");
  const int prodDefForm = w.emit(
      "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#" + std::to_string(product) +
      ",.NOT_KNOWN.)");
  const int prodDefCtx =
      w.emit("PRODUCT_DEFINITION_CONTEXT('part definition',#" + std::to_string(appCtx) + ",'design')");
  const int prodDef = w.emit("PRODUCT_DEFINITION('design','',#" + std::to_string(prodDefForm) +
                             ",#" + std::to_string(prodDefCtx) + ")");
  const int prodDefShape =
      w.emit("PRODUCT_DEFINITION_SHAPE('','',#" + std::to_string(prodDef) + ")");
  w.emit("SHAPE_DEFINITION_REPRESENTATION(#" + std::to_string(prodDefShape) + ",#" +
         std::to_string(shapeRep) + ")");

  // Product category (as STEPControl emits, keeps AP203 readers happy).
  const int prodRelCat =
      w.emit("PRODUCT_RELATED_PRODUCT_CATEGORY('part','',(#" + std::to_string(product) + "))");
  (void)prodRelCat;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API.
// ─────────────────────────────────────────────────────────────────────────────

bool canSerialize(const topo::Shape& solid) {
  if (!isSolidOrShell(solid)) return false;
  return geometrySupported(solid);
}

std::string writeStepString(const topo::Shape& solid, const std::string& productName) {
  if (!canSerialize(solid)) return {};

  DataWriter w;
  StepBuilder b(w);

  // Walk each shell in the solid; emit its faces → CLOSED_SHELL; then wrap the
  // shells in a MANIFOLD_SOLID_BREP (one shell → the outer shell of the solid;
  // the current native solids are single-shell, but the loop handles more).
  std::vector<int> shellIds;
  for (topo::Explorer shx(solid, topo::ShapeType::Shell); shx.more(); shx.next()) {
    std::vector<int> faceIds;
    // Walk the shell's faces IN NODE ORDER (not the dedup Explorer) so each face is
    // emitted once with its own bounds; a face is shared by at most one shell here.
    for (topo::Explorer fx(shx.current(), topo::ShapeType::Face); fx.more(); fx.next())
      faceIds.push_back(b.advancedFace(fx.current()));
    shellIds.push_back(w.emit("CLOSED_SHELL(''," + idList(faceIds) + ")"));
  }
  if (shellIds.empty()) return {};

  // MANIFOLD_SOLID_BREP references the OUTER shell (first). Any further shells are
  // voids; the native single-shell solids have exactly one.
  const int brepId =
      w.emit("MANIFOLD_SOLID_BREP(" + str(productName) + ",#" + std::to_string(shellIds.front()) +
             ")");

  writeWrapper(w, brepId, productName);

  std::ostringstream os;
  writeHeader(os, productName);
  os << "DATA;\n";
  for (const std::string& line : w.lines()) os << line << "\n";
  os << "ENDSEC;\n";
  os << "END-ISO-10303-21;\n";
  return os.str();
}

bool writeStepFile(const topo::Shape& solid, const std::string& path,
                   const std::string& productName) {
  const std::string content = writeStepString(solid, productName);
  if (content.empty()) return false;
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out) return false;
  out << content;
  return static_cast<bool>(out);
}

}  // namespace cybercad::native::exchange
