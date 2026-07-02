// SPDX-License-Identifier: Apache-2.0
//
// native_topology_parity.mm — native-topology vs OCCT-oracle parity harness
//                             (iOS simulator).
//
// Phase 4 capability #2 (`native-topology`), simulator verification gate 2 (see
// openspec/NATIVE-REWRITE.md). The native B-rep topology library
// (src/native/topology/*, an OCCT-FREE clean-room data model + traversal built
// on src/native/math) is compared, on IDENTICAL shapes, against the OCCT model
// it mirrors (TopoDS / TopExp / TopTools / BRep_Tool).
//
// ── What lives WHERE (the OCCT-free boundary) ────────────────────────────────
// The native library never sees OCCT. The *bridge* that walks a real
// TopoDS_Shape into the native model (occtToNative below) is TEST-ONLY and lives
// HERE, in the harness, which links OCCT. It reads geometry/orientation/location
// off OCCT via BRep_Tool + the adaptors and rebuilds an equivalent native graph
// through ShapeBuilder — nothing under src/native gains an OCCT dependency.
//
// ── Sharing / ordering strategy (why the enumerations line up) ───────────────
// OCCT expresses adjacency by SHARING one TShape node between parents (a cube's
// two adjacent faces reference the same edge node). The native model shares the
// same way (one native TShape referenced by several Shape handles). The bridge
// therefore keys native nodes on the OCCT TShape pointer + TopLoc_Location, so a
// shared OCCT sub-shape becomes a shared native node. Because the native
// Explorer / ShapeMap dedup by isSame (node+location) with the SAME pre-order
// DFS and the SAME enum ordering (ShapeType 0..7 == TopAbs COMPOUND..VERTEX),
// native MapShapes reproduces TopExp::MapShapes id-for-id. A per-comparison
// correspondence table (OCCT sub-shape → native Shape) records the mapping made
// during the walk so id N on the OCCT side can be checked against id N on the
// native side.
//
// ── Comparisons (per shape: box, cylinder, filleted box) ─────────────────────
//   1. sub-shape COUNTS per type (V/E/F/wire/shell/solid) == TopExp counts.
//   2. MapShapes ID ORDER: native ShapeMap[i] corresponds to OCCT
//      IndexedMap[i] for every i, and the sizes match (== TopExp::MapShapes).
//   3. ANCESTRY: for each edge, native edge->faces (AncestryMap) equals OCCT's
//      TopExp ancestry (IndexedDataMapOfShapeListOfShape), mapped through ids.
//   4. ACCESSORS (tight fp64 tol): vertex points == BRep_Tool::Pnt; edge curve
//      endpoints (C(first)/C(last)) + [first,last] == BRep_Tool; face surface
//      type + eval at a sample (u,v) == BRep_Tool surface.
//   5. ORIENTATION flags per sub-shape match (native == OCCT, all types).
//
// Output: [NTOPO] PASS/FAIL lines with counts + max accessor error, then a final
// "== N passed, M failed ==". Flushes and std::_Exit (OCCT static teardown in
// the trimmed static build is not exit-clean — same rationale as the other sim
// harnesses; every handle here is stack/RAII-scoped).
//
// Build (see scripts/run-sim-native-topology.sh):
//   -DCYBERCAD_HAS_OCCT  -std=c++20  for arm64-apple-ios-simulator, linking the
//   ModelingData/ModelingAlgorithms slice of OCCT (TKBRep/TKPrim/TKFillet/...).

#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_topology_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

// ── OCCT oracle headers ──────────────────────────────────────────────────────
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopTools_ListIteratorOfListOfShape.hxx>
#include <TopLoc_Location.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Pnt.hxx>

namespace nt = cybercad::native::topology;
namespace nm = cybercad::native::math;

// ═════════════════════════════════════════════════════════════════════════════
// Result accounting.  One "case" = one shape × one comparison family.
// ═════════════════════════════════════════════════════════════════════════════
static int g_pass = 0;
static int g_fail = 0;

// A single PASS/FAIL line.  `detail` carries counts / max accessor error.
static void report(const char* shape, const char* what, bool ok,
                   const std::string& detail) {
  std::printf("[NTOPO] %-16s %-22s %s  %s\n", shape, what, ok ? "PASS" : "FAIL",
              detail.c_str());
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// ═════════════════════════════════════════════════════════════════════════════
// Enum bridges — native ShapeType/Orientation are DEFINED to match TopAbs
// (Compound=0..Vertex=7, Forward=0..External=3), so the maps are the identity.
// Made explicit here so a future reorder is caught at compile time.
// ═════════════════════════════════════════════════════════════════════════════
static nt::ShapeType toNativeType(TopAbs_ShapeEnum t) {
  switch (t) {
    case TopAbs_COMPOUND:  return nt::ShapeType::Compound;
    case TopAbs_COMPSOLID: return nt::ShapeType::CompSolid;
    case TopAbs_SOLID:     return nt::ShapeType::Solid;
    case TopAbs_SHELL:     return nt::ShapeType::Shell;
    case TopAbs_FACE:      return nt::ShapeType::Face;
    case TopAbs_WIRE:      return nt::ShapeType::Wire;
    case TopAbs_EDGE:      return nt::ShapeType::Edge;
    default:               return nt::ShapeType::Vertex;
  }
}
static nt::Orientation toNativeOrient(TopAbs_Orientation o) {
  switch (o) {
    case TopAbs_FORWARD:  return nt::Orientation::Forward;
    case TopAbs_REVERSED: return nt::Orientation::Reversed;
    case TopAbs_INTERNAL: return nt::Orientation::Internal;
    default:              return nt::Orientation::External;
  }
}

// gp → native converters.
static nm::Point3 toPt(const gp_Pnt& p) { return {p.X(), p.Y(), p.Z()}; }
static nm::Dir3   toDir(const gp_Dir& d) { return {d.X(), d.Y(), d.Z()}; }

// Build a native Ax3 from an OCCT placement (gp_Ax3 or gp_Ax2 → origin+X+Y+Z).
template <class GpAx>
static nm::Ax3 toAx3(const GpAx& ax) {
  return nm::Ax3{toPt(ax.Location()), toDir(ax.XDirection()), toDir(ax.YDirection()),
                 toDir(ax.Direction())};
}

// The OCCT world transform of a located shape, as a native Transform.
static nm::Transform toTransform(const TopLoc_Location& loc) {
  const gp_Trsf t = loc.Transformation();
  const nm::Mat3 m{t.Value(1, 1), t.Value(1, 2), t.Value(1, 3),
                   t.Value(2, 1), t.Value(2, 2), t.Value(2, 3),
                   t.Value(3, 1), t.Value(3, 2), t.Value(3, 3)};
  return nm::Transform{m, nm::Vec3{t.Value(1, 4), t.Value(2, 4), t.Value(3, 4)}};
}
static nt::Location toLocation(const TopLoc_Location& loc) {
  if (loc.IsIdentity()) return nt::Location{};
  return nt::Location{toTransform(loc)};
}

// ═════════════════════════════════════════════════════════════════════════════
// Bridge: OCCT TopoDS_Shape → native Shape.
//
// A recursive walk that mirrors OCCT's node SHARING: native TShape nodes are
// memoised on the OCCT TShape pointer + TopLoc_Location key, so a sub-shape
// shared by two parents in OCCT becomes one shared native node (identical
// explorer/map dedup behaviour). Geometry for the leaf-ish types (vertex point,
// edge curve, face surface) is read off OCCT via BRep_Tool / the adaptors and
// rebuilt as the native payload.
//
// The walk also fills `corr`: OCCT-TShape*+loc → the native Shape handle it was
// bridged to (orientation-independent), the correspondence used to line up ids.
//
// Cognitive complexity is kept low by delegating each leaf's geometry to a
// dedicated builder (buildEdgeCurve / buildFaceSurface) and driving containers
// through one generic child loop.
// ═════════════════════════════════════════════════════════════════════════════
class OcctBridge {
 public:
  // The correspondence key: shared node identity + placement (matches native
  // isSame and OCCT IsSame). Orientation is intentionally excluded.
  struct Key {
    const void* node;
    TopLoc_Location loc;
    bool operator==(const Key& o) const {
      return node == o.node && loc.IsEqual(o.loc);
    }
  };
  struct KeyHash {
    std::size_t operator()(const Key& k) const {
      // Node pointer is the primary discriminator; location collisions are
      // resolved by Key::operator== (loc.IsEqual) in the bucket. Keeping the
      // hash node-only avoids depending on TopLoc_Location's HashCode signature
      // (which differs across OCCT versions).
      return std::hash<const void*>{}(k.node);
    }
  };

  // Bridge `s` and everything below it; return the native handle for `s`.
  nt::Shape bridge(const TopoDS_Shape& s) { return build(s); }

  // OCCT sub-shape → native Shape (node+location; orientation stripped). Used to
  // translate OCCT ids into the native handle they correspond to.
  //
  // The lookup key uses `s.Location()`, i.e. the WORLD-cumulative location the
  // caller's explorer (TopExp) reports, so it lines up with how the native
  // Explorer world-places its handles.
  const nt::Shape* correspond(const TopoDS_Shape& s) const {
    auto it = corr_.find(keyOf(s));
    return it == corr_.end() ? nullptr : &it->second;
  }

 private:
  static Key keyOf(const TopoDS_Shape& s) {
    return Key{s.TShape().get(), s.Location()};
  }

  // Record the WORLD-placed correspondence for `world` (orientation-independent
  // handle; native Explorer/ShapeMap dedup by node+location, so this handle only
  // needs the world node + world location).
  void remember(const TopoDS_Shape& world, const nt::Shape& node) {
    corr_.emplace(keyOf(world),
                  nt::Shape{node.tshape(), nt::Orientation::Forward, toLocation(world.Location())});
  }

  // ── The two-orientation/location model of the bridge ─────────────────────────
  //
  // A TopoDS_Shape carries a LOCAL orientation+location relative to its parent;
  // the native model stores children with those LOCAL values and composes them
  // down the graph during traversal (exactly like OCCT's own graph +
  // TopoDS_Iterator with cumOri/cumLoc). The comparison, however, drives OCCT's
  // TopExp explorers which report each sub-shape with its WORLD-cumulative
  // orientation/location. So the bridge must:
  //   * STORE each child with its LOCAL orientation/location (non-cumulative
  //     iteration) — otherwise the native traversal composes the parent's
  //     orientation/location a SECOND time (double-application: e.g. a face's
  //     Reversed flag applied twice to its wire, or a fillet sub-shape's location
  //     applied twice, giving a 30-unit point error);
  //   * KEY the correspondence table on the WORLD-cumulative shape, so
  //     correspond() matches what the comparison's explorer produces.
  //
  // `build(world)` therefore keys/memoises on `world` (cumulative) but recurses
  // over children read NON-cumulatively, composing world context itself only for
  // the recursive key.

  // Compose parent world orientation/location onto a child's local values to get
  // the child's world shape (used purely as the recursion/correspondence key).
  static TopoDS_Shape worldOf(const TopoDS_Shape& parentWorld, const TopoDS_Shape& localChild) {
    TopoDS_Shape w = localChild;
    w.Orientation(TopAbs::Compose(parentWorld.Orientation(), localChild.Orientation()));
    if (!parentWorld.Location().IsIdentity()) w.Move(parentWorld.Location(), false);
    return w;
  }

  // Memoised construction of the SHARED native node for the geometry of `world`.
  // Returns a handle carrying `world`'s WORLD orientation+location (so a caller
  // can store it, or re-wrap it with local values). Records the correspondence.
  nt::Shape build(const TopoDS_Shape& world) {
    const Key k = keyOf(world);
    if (auto it = nodeCache_.find(k); it != nodeCache_.end())
      return oriented(it->second, world);  // shared node, this instance's placement

    nt::Shape node = construct(world);      // recurses over children
    nodeCache_.emplace(k, node);
    remember(world, node);                  // world-placed correspondence
    return oriented(node, world);
  }

  // Re-wrap a base native handle with the given OCCT shape's orientation and
  // location (world when called on a world shape; local when called on a local
  // child). Both bases coincide because native composition mirrors OCCT's.
  static nt::Shape oriented(const nt::Shape& base, const TopoDS_Shape& s) {
    return nt::Shape{base.tshape(), toNativeOrient(s.Orientation()), toLocation(s.Location())};
  }

  // Build the base native handle for one OCCT shape (its own node; the caller
  // applies orientation/location via oriented()).
  nt::Shape construct(const TopoDS_Shape& world) {
    switch (world.ShapeType()) {
      case TopAbs_VERTEX: return buildVertex(TopoDS::Vertex(world));
      case TopAbs_EDGE:   return buildEdge(TopoDS::Edge(world), world);
      case TopAbs_FACE:   return buildFace(TopoDS::Face(world), world);
      case TopAbs_WIRE:   return container(world, nt::ShapeType::Wire);
      case TopAbs_SHELL:  return container(world, nt::ShapeType::Shell);
      case TopAbs_SOLID:  return container(world, nt::ShapeType::Solid);
      case TopAbs_COMPSOLID: return container(world, nt::ShapeType::CompSolid);
      default:            return container(world, nt::ShapeType::Compound);
    }
  }

  // Bridge one LOCAL child of `parentWorld`: recurse on its world shape (to fill
  // the correspondence table with world-cumulative keys) to obtain the shared
  // node, then re-wrap that node with the child's LOCAL orientation/location for
  // storage in the parent node.
  nt::Shape bridgeLocalChild(const TopoDS_Shape& parentWorld, const TopoDS_Shape& localChild) {
    const nt::Shape world = build(worldOf(parentWorld, localChild));
    return oriented(world, localChild);  // local orientation/location for storage
  }

  // ── Leaves ─────────────────────────────────────────────────────────────────
  nt::Shape buildVertex(const TopoDS_Vertex& v) {
    const gp_Pnt p = BRep_Tool::Pnt(v);
    return nt::ShapeBuilder::makeVertex(toPt(p), BRep_Tool::Tolerance(v));
  }

  nt::Shape buildEdge(const TopoDS_Edge& e, const TopoDS_Shape& world) {
    // Bounding vertices read NON-cumulatively so each carries its LOCAL
    // orientation, and stored IN OCCT ITERATION ORDER with those orientations via
    // makeEdgeWithVertices. This reproduces OCCT's edge exactly on both axes:
    //   * ORDER — the native Explorer emits an edge's vertices in stored order,
    //     so the first-encountered vertex (hence its MapShapes id) matches OCCT
    //     only if we keep OCCT's child order (a Forward-first / Reversed-second
    //     forcing would swap the ids of edges OCCT stores Reversed-first);
    //   * ORIENTATION — each vertex keeps its own OCCT Forward/Reversed flag.
    std::vector<nt::Shape> verts;
    for (TopoDS_Iterator it(e, /*cumOri=*/false, /*cumLoc=*/false); it.More(); it.Next()) {
      if (it.Value().ShapeType() != TopAbs_VERTEX) continue;
      verts.push_back(bridgeLocalChild(world, it.Value()));
    }
    double first = 0.0, last = 0.0;
    nt::EdgeCurve curve = buildEdgeCurve(e, first, last);
    return nt::ShapeBuilder::makeEdgeWithVertices(curve, first, last, std::move(verts),
                                                  BRep_Tool::Tolerance(e));
  }

  nt::Shape buildFace(const TopoDS_Face& f, const TopoDS_Shape& world) {
    nt::FaceSurface srf = buildFaceSurface(f);
    // Wires read NON-cumulatively (local orientation/location); first sub-wire is
    // the outer, the rest are holes (OCCT order). The face node carries the
    // face's LOCAL orientation; native traversal composes it onto the wires.
    nt::Shape outer;
    std::vector<nt::Shape> holes;
    for (TopoDS_Iterator it(f, /*cumOri=*/false, /*cumLoc=*/false); it.More(); it.Next()) {
      if (it.Value().ShapeType() != TopAbs_WIRE) continue;
      nt::Shape w = bridgeLocalChild(world, it.Value());
      if (outer.isNull()) outer = w; else holes.push_back(w);
    }
    return nt::ShapeBuilder::makeFace(srf, outer, std::move(holes),
                                      toNativeOrient(f.Orientation()), BRep_Tool::Tolerance(f));
  }

  // ── Containers (wire/shell/solid/compsolid/compound) ─────────────────────────
  nt::Shape container(const TopoDS_Shape& world, nt::ShapeType type) {
    std::vector<nt::Shape> kids;
    for (TopoDS_Iterator it(world, /*cumOri=*/false, /*cumLoc=*/false); it.More(); it.Next())
      kids.push_back(bridgeLocalChild(world, it.Value()));
    switch (type) {
      case nt::ShapeType::Wire:      return nt::ShapeBuilder::makeWire(std::move(kids));
      case nt::ShapeType::Shell:     return nt::ShapeBuilder::makeShell(std::move(kids));
      case nt::ShapeType::Solid:     return nt::ShapeBuilder::makeSolid(std::move(kids));
      case nt::ShapeType::CompSolid: return nt::ShapeBuilder::makeCompSolid(std::move(kids));
      default:                       return nt::ShapeBuilder::makeCompound(std::move(kids));
    }
  }

  // ── Geometry extraction ──────────────────────────────────────────────────────
  // Edge curve as a native payload; also returns the [first,last] range. The
  // analytic kinds (Line/Circle/Ellipse) are modelled exactly. Anything else
  // (OCCT GeomAbs_BSplineCurve / GeomAbs_OtherCurve — e.g. a fillet's toroidal
  // blend edges) is recorded as the non-analytic BSpline kind WITHOUT populating
  // an analytic frame; the accessor comparison then SKIPS its endpoint check
  // (endpoint parity is only meaningful for the analytic kinds this bridge
  // evaluates). Marking these BSpline rather than a placeholder Line is what lets
  // compareEdges skip them — a placeholder Line would be mistaken for a real line
  // and compared against OCCT's actual curve, giving a spurious large error.
  static nt::EdgeCurve buildEdgeCurve(const TopoDS_Edge& e, double& first, double& last) {
    nt::EdgeCurve c;
    BRepAdaptor_Curve ad(e);
    first = ad.FirstParameter();
    last = ad.LastParameter();
    switch (ad.GetType()) {
      case GeomAbs_Line: {
        const gp_Lin l = ad.Line();
        c.kind = nt::EdgeCurve::Kind::Line;
        c.frame = nm::Ax3{toPt(l.Location()), toDir(l.Direction()), {}, toDir(l.Direction())};
        break;
      }
      case GeomAbs_Circle: {
        const gp_Circ ci = ad.Circle();
        c.kind = nt::EdgeCurve::Kind::Circle;
        c.frame = toAx3(ci.Position());
        c.radius = ci.Radius();
        break;
      }
      case GeomAbs_Ellipse: {
        const gp_Elips el = ad.Ellipse();
        c.kind = nt::EdgeCurve::Kind::Ellipse;
        c.frame = toAx3(el.Position());
        c.radius = el.MajorRadius();
        c.minorRadius = el.MinorRadius();
        break;
      }
      default:
        // Non-analytic (BSpline/Other). Mark as BSpline so compareEdges skips the
        // analytic endpoint check; no analytic frame is populated.
        c.kind = nt::EdgeCurve::Kind::BSpline;
        break;
    }
    return c;
  }

  static nt::FaceSurface buildFaceSurface(const TopoDS_Face& f) {
    nt::FaceSurface s;
    BRepAdaptor_Surface ad(f);
    switch (ad.GetType()) {
      case GeomAbs_Plane:
        s.kind = nt::FaceSurface::Kind::Plane;
        s.frame = toAx3(ad.Plane().Position());
        break;
      case GeomAbs_Cylinder:
        s.kind = nt::FaceSurface::Kind::Cylinder;
        s.frame = toAx3(ad.Cylinder().Position());
        s.radius = ad.Cylinder().Radius();
        break;
      case GeomAbs_Cone:
        s.kind = nt::FaceSurface::Kind::Cone;
        s.frame = toAx3(ad.Cone().Position());
        s.radius = ad.Cone().RefRadius();
        s.semiAngle = ad.Cone().SemiAngle();
        break;
      case GeomAbs_Sphere:
        s.kind = nt::FaceSurface::Kind::Sphere;
        s.frame = toAx3(ad.Sphere().Position());
        s.radius = ad.Sphere().Radius();
        break;
      default:
        s.kind = nt::FaceSurface::Kind::Plane;  // placeholder; eval checked via OCCT
        break;
    }
    return s;
  }

  std::unordered_map<Key, nt::Shape, KeyHash> nodeCache_;  // shared-node memo
  std::unordered_map<Key, nt::Shape, KeyHash> corr_;       // OCCT sub-shape → native
};

// ═════════════════════════════════════════════════════════════════════════════
// Native analytic evaluators — mirror ElSLib / the adaptor Value(). Used ONLY by
// the accessor comparison to evaluate the native payload at a sample; the native
// library itself exposes math::Plane/Cylinder/... which we drive here.
// ═════════════════════════════════════════════════════════════════════════════
static nm::Point3 nativeSurfaceValue(const nt::FaceSurface& s, const nt::Location& loc,
                                     double u, double v) {
  nm::Point3 p;
  switch (s.kind) {
    case nt::FaceSurface::Kind::Plane:    p = nm::Plane{s.frame}.value(u, v); break;
    case nt::FaceSurface::Kind::Cylinder: p = nm::Cylinder{s.frame, s.radius}.value(u, v); break;
    case nt::FaceSurface::Kind::Cone:     p = nm::Cone{s.frame, s.radius, s.semiAngle}.value(u, v); break;
    case nt::FaceSurface::Kind::Sphere:   p = nm::Sphere{s.frame, s.radius}.value(u, v); break;
    default:                              p = nm::Plane{s.frame}.value(u, v); break;
  }
  return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
}

// Native edge curve evaluated at parameter t (world-placed). Only Line/Circle/
// Ellipse are analytic here — enough for the box/cylinder/fillet edge set.
static nm::Point3 nativeCurveValue(const nt::EdgeCurve& c, const nt::Location& loc, double t) {
  nm::Point3 p;
  switch (c.kind) {
    case nt::EdgeCurve::Kind::Line:
      p = c.frame.origin + c.frame.z.vec() * t;
      break;
    case nt::EdgeCurve::Kind::Circle:
      p = nm::frameCombine(c.frame, c.radius * std::cos(t), c.radius * std::sin(t), 0.0);
      break;
    case nt::EdgeCurve::Kind::Ellipse:
      p = nm::frameCombine(c.frame, c.radius * std::cos(t), c.minorRadius * std::sin(t), 0.0);
      break;
    default:
      p = c.frame.origin;
      break;
  }
  return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
}

static double ptErr(const nm::Point3& p, const gp_Pnt& q) {
  return std::max({std::fabs(p.x - q.X()), std::fabs(p.y - q.Y()), std::fabs(p.z - q.Z())});
}

// ═════════════════════════════════════════════════════════════════════════════
// COMPARISON 1 + 2 — sub-shape counts and MapShapes id order/count per type.
//
// For each topological kind we build OCCT's TopExp::MapShapes and the native
// mapShapes, then require: same size, and for every 1-based id the native
// ShapeMap entry corresponds (via the bridge table) to OCCT's IndexedMap entry.
// ═════════════════════════════════════════════════════════════════════════════
struct TypePair {
  const char* label;
  TopAbs_ShapeEnum occt;
  nt::ShapeType native;
};
static const TypePair kTypes[] = {
    {"vertex", TopAbs_VERTEX, nt::ShapeType::Vertex},
    {"edge",   TopAbs_EDGE,   nt::ShapeType::Edge},
    {"wire",   TopAbs_WIRE,   nt::ShapeType::Wire},
    {"face",   TopAbs_FACE,   nt::ShapeType::Face},
    {"shell",  TopAbs_SHELL,  nt::ShapeType::Shell},
    {"solid",  TopAbs_SOLID,  nt::ShapeType::Solid},
};

static void compareCountsAndOrder(const char* name, const TopoDS_Shape& occt,
                                  const nt::Shape& native, OcctBridge& bridge) {
  bool countsOk = true, orderOk = true;
  std::string counts;
  char buf[64];

  for (const TypePair& tp : kTypes) {
    TopTools_IndexedMapOfShape omap;
    TopExp::MapShapes(occt, tp.occt, omap);
    nt::ShapeMap nmap = nt::mapShapes(native, tp.native);

    const int on = omap.Extent();
    const std::size_t nn = nmap.size();
    std::snprintf(buf, sizeof(buf), "%s=%d ", tp.label, on);
    counts += buf;
    if (static_cast<std::size_t>(on) != nn) countsOk = false;

    // Id order: OCCT IndexedMap is 1-based in TopExp::MapShapes order; the native
    // ShapeMap is 1-based in native explorer order. Require entry-by-entry
    // correspondence (the native id equals the OCCT id for the same sub-shape).
    for (int i = 1; i <= on; ++i) {
      const TopoDS_Shape& os = omap.FindKey(i);
      const nt::Shape* corr = bridge.correspond(os);
      if (!corr) { orderOk = false; break; }
      const int nid = nmap.findIndex(*corr);
      if (nid != i) { orderOk = false; break; }
    }
  }

  report(name, "counts", countsOk, counts);
  report(name, "mapshapes-order", orderOk,
         orderOk ? "native id == TopExp id, all types"
                 : "native/OCCT id mismatch");
}

// ═════════════════════════════════════════════════════════════════════════════
// COMPARISON 3 — edge→face ancestry.
//
// OCCT: TopExp::MapShapesAndUniqueAncestors(shape, EDGE, FACE, M,
//       useOrientation=false). Native: AncestryMap(native, Edge, Face).
// For each edge, the SET of parent faces must match. We compare through the
// bridge correspondence: translate each OCCT parent face to its native handle
// and require the native parent set to contain exactly the same faces (by
// isSame), same cardinality.
//
// UNIQUE (deduped) ancestors, deliberately — a documented convention difference
// vs the plain TopExp::MapShapesAndAncestors. The native AncestryMap keeps a
// SET of parents (appendUnique, deduped by isSame), and the native Explorer
// visits each sub-shape once (deduped by isSame). The plain OCCT variant appends
// an ancestor once PER OCCURRENCE, so a cylinder's seam edge (which appears twice
// in the cylindrical face's wire) would list that face twice — the native
// "each sub-shape/parent seen once" model corresponds to the *Unique* variant,
// not the raw one. Matching against MapShapesAndUniqueAncestors(useOrientation=
// false) compares like-for-like rather than weakening the assertion.
// ═════════════════════════════════════════════════════════════════════════════
static void compareAncestry(const char* name, const TopoDS_Shape& occt,
                            const nt::Shape& native, OcctBridge& bridge) {
  TopTools_IndexedDataMapOfShapeListOfShape oanc;
  TopExp::MapShapesAndUniqueAncestors(occt, TopAbs_EDGE, TopAbs_FACE, oanc,
                                      /*useOrientation=*/false);
  nt::AncestryMap nanc(native, nt::ShapeType::Edge, nt::ShapeType::Face);

  bool ok = true;
  int edges = 0;
  int worstEdge = -1;
  std::size_t worstNative = 0, worstOcct = 0;

  for (int i = 1; i <= oanc.Extent() && ok; ++i) {
    ++edges;
    const TopoDS_Shape& oedge = oanc.FindKey(i);
    const TopTools_ListOfShape& ofaces = oanc.FindFromIndex(i);

    const nt::Shape* nedge = bridge.correspond(oedge);
    if (!nedge) { ok = false; worstEdge = i; break; }
    const std::vector<nt::Shape>& nfaces = nanc.parentsOf(*nedge);

    // Same count.
    if (static_cast<std::size_t>(ofaces.Extent()) != nfaces.size()) {
      ok = false; worstEdge = i;
      worstNative = nfaces.size();
      worstOcct = static_cast<std::size_t>(ofaces.Extent());
      break;
    }
    // Every OCCT parent face must be present (by isSame) in the native set.
    for (TopTools_ListIteratorOfListOfShape it(ofaces); it.More() && ok; it.Next()) {
      const nt::Shape* nface = bridge.correspond(it.Value());
      bool found = false;
      if (nface)
        for (const nt::Shape& nf : nfaces)
          if (nf.isSame(*nface)) { found = true; break; }
      if (!found) { ok = false; worstEdge = i; }
    }
  }

  char buf[96];
  if (ok)
    std::snprintf(buf, sizeof(buf), "edges=%d edge->faces sets match", edges);
  else
    std::snprintf(buf, sizeof(buf), "mismatch at edge #%d (native=%zu occt=%zu)",
                  worstEdge, worstNative, worstOcct);
  report(name, "ancestry-edge-faces", ok, buf);
}

// ═════════════════════════════════════════════════════════════════════════════
// COMPARISON 4 — accessors (tight fp64 tolerance).
//
//   Vertices: native pointOf == BRep_Tool::Pnt.
//   Edges:    native curve endpoints C(first)/C(last) and [first,last] range
//             == the OCCT edge sampled at its own first/last params.
//   Faces:    native surface type == OCCT surface type, and native eval at a
//             sample (u,v) == OCCT BRepAdaptor_Surface.Value(u,v).
// ═════════════════════════════════════════════════════════════════════════════
static const double kAccTol = 1e-9;

static void compareVertices(const TopoDS_Shape& occt, const OcctBridge& b, double& maxErr) {
  TopTools_IndexedMapOfShape vmap;
  TopExp::MapShapes(occt, TopAbs_VERTEX, vmap);
  for (int i = 1; i <= vmap.Extent(); ++i) {
    const TopoDS_Vertex v = TopoDS::Vertex(vmap.FindKey(i));
    const nt::Shape* nv = b.correspond(v);
    if (!nv) { maxErr = 1e9; continue; }
    if (auto p = nt::pointOf(*nv)) maxErr = std::max(maxErr, ptErr(*p, BRep_Tool::Pnt(v)));
    else maxErr = 1e9;
  }
}

static void compareEdges(const TopoDS_Shape& occt, const OcctBridge& b, double& maxErr) {
  TopTools_IndexedMapOfShape emap;
  TopExp::MapShapes(occt, TopAbs_EDGE, emap);
  for (int i = 1; i <= emap.Extent(); ++i) {
    const TopoDS_Edge e = TopoDS::Edge(emap.FindKey(i));
    const nt::Shape* ne = b.correspond(e);
    if (!ne) { maxErr = 1e9; continue; }
    auto cr = nt::curveOf(*ne);
    auto rr = nt::rangeOf(*ne);
    if (!cr || !rr) { maxErr = 1e9; continue; }

    BRepAdaptor_Curve ad(e);
    // Range parity.
    maxErr = std::max({maxErr, std::fabs(rr->first - ad.FirstParameter()),
                       std::fabs(rr->last - ad.LastParameter())});
    // Endpoint parity: native curve value at first/last vs OCCT curve value.
    // Skip non-analytic native curves (placeholder) — their endpoints are only
    // meaningful for the analytic kinds this bridge models.
    if (cr->curve->kind == nt::EdgeCurve::Kind::Line ||
        cr->curve->kind == nt::EdgeCurve::Kind::Circle ||
        cr->curve->kind == nt::EdgeCurve::Kind::Ellipse) {
      const nm::Point3 nf = nativeCurveValue(*cr->curve, cr->location, cr->first);
      const nm::Point3 nl = nativeCurveValue(*cr->curve, cr->location, cr->last);
      maxErr = std::max({maxErr, ptErr(nf, ad.Value(ad.FirstParameter())),
                         ptErr(nl, ad.Value(ad.LastParameter()))});
    }
  }
}

static void compareFaces(const TopoDS_Shape& occt, const OcctBridge& b,
                         double& maxErr, bool& typeOk) {
  TopTools_IndexedMapOfShape fmap;
  TopExp::MapShapes(occt, TopAbs_FACE, fmap);
  for (int i = 1; i <= fmap.Extent(); ++i) {
    const TopoDS_Face f = TopoDS::Face(fmap.FindKey(i));
    const nt::Shape* nf = b.correspond(f);
    if (!nf) { maxErr = 1e9; typeOk = false; continue; }
    auto sr = nt::surfaceOf(*nf);
    if (!sr) { maxErr = 1e9; typeOk = false; continue; }

    BRepAdaptor_Surface ad(f);
    // Type parity for the analytic surfaces this bridge models.
    const GeomAbs_SurfaceType ot = ad.GetType();
    const bool analytic = ot == GeomAbs_Plane || ot == GeomAbs_Cylinder ||
                          ot == GeomAbs_Cone || ot == GeomAbs_Sphere;
    if (analytic) {
      const bool match =
          (ot == GeomAbs_Plane    && sr->surface->kind == nt::FaceSurface::Kind::Plane) ||
          (ot == GeomAbs_Cylinder && sr->surface->kind == nt::FaceSurface::Kind::Cylinder) ||
          (ot == GeomAbs_Cone     && sr->surface->kind == nt::FaceSurface::Kind::Cone) ||
          (ot == GeomAbs_Sphere   && sr->surface->kind == nt::FaceSurface::Kind::Sphere);
      if (!match) typeOk = false;
      // Eval parity at a sample (u,v) inside the face's parametric range.
      const double u = 0.5 * (ad.FirstUParameter() + ad.LastUParameter());
      const double v = 0.5 * (ad.FirstVParameter() + ad.LastVParameter());
      const nm::Point3 np = nativeSurfaceValue(*sr->surface, sr->location, u, v);
      maxErr = std::max(maxErr, ptErr(np, ad.Value(u, v)));
    }
  }
}

static void compareAccessors(const char* name, const TopoDS_Shape& occt,
                             const nt::Shape& native, OcctBridge& bridge) {
  double maxErr = 0.0;
  bool typeOk = true;

  // All three reuse the shared bridge, whose correspondence table was populated
  // by the initial walk in runShape().
  compareVertices(occt, bridge, maxErr);
  compareEdges(occt, bridge, maxErr);
  compareFaces(occt, bridge, maxErr, typeOk);

  char buf[96];
  std::snprintf(buf, sizeof(buf), "maxErr=%.3e tol=%.1e surfType=%s", maxErr, kAccTol,
                typeOk ? "match" : "MISMATCH");
  report(name, "accessors", maxErr <= kAccTol && typeOk, buf);
}

// ═════════════════════════════════════════════════════════════════════════════
// COMPARISON 5 — orientation flags per sub-shape.
//
// For every sub-shape of every type, the native handle at the same MapShapes id
// must carry the same orientation as OCCT. MapShapes/IndexedMap store the FIRST
// occurrence orientation; we compare that (deduped-by-isSame) handle's flag.
// ═════════════════════════════════════════════════════════════════════════════
static void compareOrientation(const char* name, const TopoDS_Shape& occt,
                               const nt::Shape& native, OcctBridge& bridge) {
  bool ok = true;
  int checked = 0;
  const char* worst = "";

  for (const TypePair& tp : kTypes) {
    TopTools_IndexedMapOfShape omap;
    TopExp::MapShapes(occt, tp.occt, omap);
    nt::ShapeMap nmap = nt::mapShapes(native, tp.native);
    for (int i = 1; i <= omap.Extent() && ok; ++i) {
      ++checked;
      const nt::Orientation want = toNativeOrient(omap.FindKey(i).Orientation());
      const int nid = nmap.findIndex(*bridge.correspond(omap.FindKey(i)));
      if (nid == 0) { ok = false; worst = tp.label; break; }
      if (nmap.shape(nid).orientation() != want) { ok = false; worst = tp.label; }
    }
  }

  char buf[80];
  if (ok) std::snprintf(buf, sizeof(buf), "%d sub-shapes, flags match", checked);
  else    std::snprintf(buf, sizeof(buf), "orientation mismatch on a %s", worst);
  report(name, "orientation", ok, buf);
}

// ═════════════════════════════════════════════════════════════════════════════
// Per-shape driver: bridge once, run all five comparison families.
// ═════════════════════════════════════════════════════════════════════════════
static void runShape(const char* name, const TopoDS_Shape& occt) {
  OcctBridge bridge;
  nt::Shape native = bridge.bridge(occt);

  compareCountsAndOrder(name, occt, native, bridge);
  compareAncestry(name, occt, native, bridge);
  compareAccessors(name, occt, native, bridge);
  compareOrientation(name, occt, native, bridge);
}

int main() {
  std::printf("== native-topology vs OCCT-oracle parity ==\n");
  std::fflush(stdout);

  // Shape 1 — a box (6 planar faces, 12 edges, 8 vertices).
  {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    runShape("box", box);
  }
  // Shape 2 — a cylinder (2 planar caps + 1 cylindrical face + a seam edge).
  {
    TopoDS_Shape cyl = BRepPrimAPI_MakeCylinder(5.0, 12.0).Shape();
    runShape("cylinder", cyl);
  }
  // Shape 3 — a filleted box (adds cylindrical + spherical blend faces and the
  // richest edge/vertex ancestry of the three).
  {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    BRepFilletAPI_MakeFillet mf(box);
    for (TopExp_Explorer ex(box, TopAbs_EDGE); ex.More(); ex.Next())
      mf.Add(2.0, TopoDS::Edge(ex.Current()));
    TopoDS_Shape fil = mf.Shape();
    runShape("filleted-box", fil);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);

  // OCCT static teardown in the trimmed static build is not exit-clean (same
  // rationale as native_math_parity / parity_bench): every handle here is
  // stack/RAII-scoped, so exit without running C++ static destructors and report
  // the true result.
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
