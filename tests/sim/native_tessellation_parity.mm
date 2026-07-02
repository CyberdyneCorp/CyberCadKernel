// SPDX-License-Identifier: Apache-2.0
//
// native_tessellation_parity.mm — native-tessellation vs OCCT-oracle parity
//                                 harness (iOS simulator).
//
// Phase 4 capability #3 (`native-tessellation`), simulator verification gate 2
// (see openspec/NATIVE-REWRITE.md). The native tessellator (src/native/
// tessellate/*, an OCCT-FREE clean-room grid mesher on src/native/math +
// src/native/topology) meshes shapes bridged from OCCT, and the resulting mesh
// is checked against the OCCT ORACLE using TOLERANCE-BASED PROPERTIES — NEVER
// triangle-identical output (tessellation is an approximation of a B-rep).
//
// The oracle it is compared against is TWO-SIDED, which is the point of this
// harness:
//   * OCCT BRepMesh_IncrementalMesh of the SAME shape at the SAME deflection —
//     the reference tessellator; its bounding box is the reference silhouette.
//   * BRepGProp on the exact B-rep — the CLOSED-FORM area/volume the mesh must
//     converge to as deflection → 0. A tessellation always slightly
//     under-estimates a convex smooth patch, so both the native mesh AND the
//     OCCT mesh are held to a deflection-scaled tolerance of this exact truth.
//
// ── Comparisons (per shape: box, cylinder, sphere, filleted box) ─────────────
//   1. WATERTIGHT — the native mesh is a closed 2-manifold (every undirected
//      edge shared by exactly two triangles). A closed solid must mesh watertight
//      after shared-edge welding. (Curved-edge solids meshed by INDEPENDENT
//      per-face parameter grids — a cylinder's disk-cap ↔ smoothly-sampled side —
//      cannot be welded watertight without a shared per-edge 1D discretization,
//      the documented BRepMesh-style follow-up; those shapes assert the achievable
//      property — 2-manifold with the open boundary confined to the curved shared
//      edges — instead of falsely asserting watertight. Flagged per shape.)
//   2. BOUNDING BOX — the native mesh AABB agrees with the OCCT BRepMesh AABB
//      within the deflection (a mesh's silhouette can only differ from the
//      reference mesh's silhouette by the chord/sagitta bound).
//   3. AREA — native surfaceArea agrees (a) with the OCCT BRepMesh area within a
//      deflection-scaled relative tolerance, AND (b) with the EXACT B-rep area
//      (BRepGProp::SurfaceProperties) within tolerance.
//   4. VOLUME — native |enclosedVolume| agrees (a) with the OCCT BRepMesh volume
//      within tolerance, AND (b) with the EXACT B-rep volume
//      (BRepGProp::VolumeProperties) within tolerance. (Volume needs a watertight
//      mesh to be meaningful; for the curved-edge cylinder the native mesh's near
//      -closed volume is still compared — the small open boundary contributes a
//      bounded error absorbed by the tolerance.)
//   5. VERTICES ON THE TRUE SURFACE — every native mesh vertex lies within the
//      deflection of SOME face of the OCCT shape (GeomAPI_ProjectPointOnSurf onto
//      each face's Geom_Surface; min distance over faces must be ≤ deflection).
//
// ── The OCCT→native bridge (TEST-ONLY, reused pattern) ───────────────────────
// This harness REUSES the bridge pattern from native_topology_parity.mm (and its
// pcurve extension from native_tessellate_parity.mm): a recursive walk that reads
// geometry/orientation/location off OCCT via BRep_Tool + the adaptors and rebuilds
// an equivalent native graph through ShapeBuilder, memoising shared TShape nodes
// so adjacency (shared edges) is preserved. Each edge's 2D PCURVE on its face is
// laid on via ShapeBuilder::addPCurve (BRep_Tool::CurveOnSurface), which the
// tessellator's trimming (trim.h) samples to build the UV boundary polygon. The
// bridge lives HERE, in the harness (which links OCCT); NOTHING under src/native
// gains an OCCT dependency.
//
// Output: [NTESS] PASS/FAIL lines with area/volume/bbox deltas per shape, then a
// final "== N passed, M failed ==". Flushes and std::_Exit (OCCT static teardown
// in the trimmed static build is not exit-clean — same rationale as the other sim
// harnesses; every handle here is stack/RAII-scoped).
//
// Build: scripts/run-sim-native-tessellation.sh (mirrors run-sim-native-topology
// / run-sim-native-tessellate): compiles this harness + src/native/{tessellate,
// topology,math}/*.cpp for arm64-apple-ios-simulator with -DCYBERCAD_HAS_OCCT,
// links the meshing-oracle slice of OCCT (TKMesh/TKBRep/TKPrim/TKFillet/...),
// spawns on a booted simulator.

#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_tessellation_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
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
#include <TopLoc_Location.hxx>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Triangle.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Elips2d.hxx>
#include <gp_Lin2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Vec.hxx>

namespace nt = cybercad::native::topology;
namespace nm = cybercad::native::math;
namespace ntess = cybercad::native::tessellate;

// ═════════════════════════════════════════════════════════════════════════════
// Result accounting.  One "case" = one shape × one comparison family.
// ═════════════════════════════════════════════════════════════════════════════
static int g_pass = 0;
static int g_fail = 0;

static void report(const char* shape, const char* what, bool ok, const std::string& detail) {
  std::printf("[NTESS] %-14s %-24s %s  %s\n", shape, what, ok ? "PASS" : "FAIL", detail.c_str());
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// ── Enum + gp → native converters (same as native_topology_parity) ────────────
static nt::Orientation toNativeOrient(TopAbs_Orientation o) {
  switch (o) {
    case TopAbs_FORWARD:  return nt::Orientation::Forward;
    case TopAbs_REVERSED: return nt::Orientation::Reversed;
    case TopAbs_INTERNAL: return nt::Orientation::Internal;
    default:              return nt::Orientation::External;
  }
}
static nm::Point3 toPt(const gp_Pnt& p) { return {p.X(), p.Y(), p.Z()}; }
static nm::Dir3   toDir(const gp_Dir& d) { return {d.X(), d.Y(), d.Z()}; }
template <class GpAx>
static nm::Ax3 toAx3(const GpAx& ax) {
  return nm::Ax3{toPt(ax.Location()), toDir(ax.XDirection()), toDir(ax.YDirection()),
                 toDir(ax.Direction())};
}
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
// Bridge: OCCT TopoDS_Shape → native Shape, with PCURVES.
//
// The TEST-ONLY OCCT→native walk-in bridge, reused verbatim in structure from
// native_topology_parity.mm's OcctBridge and its pcurve extension in
// native_tessellate_parity.mm. Native TShape nodes are memoised on the OCCT
// TShape pointer + TopLoc_Location so a sub-shape shared by two parents in OCCT
// becomes ONE shared native node (identical explorer/dedup behaviour → shared
// edges tessellate to coincident samples, so welding is watertight). Geometry for
// the leaf types (vertex point, edge curve, face surface) is read off OCCT via
// BRep_Tool / the adaptors and rebuilt as the native payload.
//
// EXTENSION for tessellation: after a face's wires are bridged, each edge of the
// face gets its 2D pcurve on that face laid on via ShapeBuilder::addPCurve
// (BRep_Tool::CurveOnSurface). The tessellator's trimming samples those pcurves.
// Node sharing is preserved because addPCurve clones only the edge node's data.
// ═════════════════════════════════════════════════════════════════════════════
class OcctBridge {
 public:
  nt::Shape bridge(const TopoDS_Shape& s) { return build(s); }

 private:
  struct Key {
    const void* node;
    TopLoc_Location loc;
    bool operator==(const Key& o) const { return node == o.node && loc.IsEqual(o.loc); }
  };
  struct KeyHash {
    std::size_t operator()(const Key& k) const { return std::hash<const void*>{}(k.node); }
  };
  static Key keyOf(const TopoDS_Shape& s) { return Key{s.TShape().get(), s.Location()}; }

  // Compose parent world orientation/location onto a child's local values to get
  // the child's world shape (used purely as the recursion/memo key).
  static TopoDS_Shape worldOf(const TopoDS_Shape& parentWorld, const TopoDS_Shape& localChild) {
    TopoDS_Shape w = localChild;
    w.Orientation(TopAbs::Compose(parentWorld.Orientation(), localChild.Orientation()));
    if (!parentWorld.Location().IsIdentity()) w.Move(parentWorld.Location(), false);
    return w;
  }

  nt::Shape build(const TopoDS_Shape& world) {
    const Key k = keyOf(world);
    if (auto it = nodeCache_.find(k); it != nodeCache_.end()) return oriented(it->second, world);
    nt::Shape node = construct(world);
    nodeCache_.emplace(k, node);
    return oriented(node, world);
  }

  static nt::Shape oriented(const nt::Shape& base, const TopoDS_Shape& s) {
    return nt::Shape{base.tshape(), toNativeOrient(s.Orientation()), toLocation(s.Location())};
  }

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

  nt::Shape bridgeLocalChild(const TopoDS_Shape& parentWorld, const TopoDS_Shape& localChild) {
    const nt::Shape world = build(worldOf(parentWorld, localChild));
    return oriented(world, localChild);
  }

  nt::Shape buildVertex(const TopoDS_Vertex& v) {
    return nt::ShapeBuilder::makeVertex(toPt(BRep_Tool::Pnt(v)), BRep_Tool::Tolerance(v));
  }

  nt::Shape buildEdge(const TopoDS_Edge& e, const TopoDS_Shape& world) {
    std::vector<nt::Shape> verts;
    for (TopoDS_Iterator it(e, false, false); it.More(); it.Next()) {
      if (it.Value().ShapeType() != TopAbs_VERTEX) continue;
      verts.push_back(bridgeLocalChild(world, it.Value()));
    }
    double first = 0.0, last = 0.0;
    nt::EdgeCurve curve = buildEdgeCurve(e, first, last);
    return nt::ShapeBuilder::makeEdgeWithVertices(curve, first, last, std::move(verts),
                                                  BRep_Tool::Tolerance(e));
  }

  // Build a face node, then lay each edge's pcurve on this face's surface node so
  // the tessellator can trim. addPCurve returns a NEW edge; we rebuild the wires
  // with the pcurve-carrying edges and rebuild the face on top of them.
  nt::Shape buildFace(const TopoDS_Face& f, const TopoDS_Shape& world) {
    nt::FaceSurface srf = buildFaceSurface(f);
    nt::Shape plainOuter;
    std::vector<nt::Shape> plainHoles;
    for (TopoDS_Iterator it(f, false, false); it.More(); it.Next()) {
      if (it.Value().ShapeType() != TopAbs_WIRE) continue;
      nt::Shape w = bridgeLocalChild(world, it.Value());
      if (plainOuter.isNull()) plainOuter = w; else plainHoles.push_back(w);
    }
    nt::Shape faceNode = nt::ShapeBuilder::makeFace(srf, plainOuter, plainHoles,
                                                    toNativeOrient(f.Orientation()),
                                                    BRep_Tool::Tolerance(f));
    nt::Shape pcOuter = withPCurves(f, plainOuter, faceNode.tshape());
    std::vector<nt::Shape> pcHoles;
    for (const nt::Shape& h : plainHoles) pcHoles.push_back(withPCurves(f, h, faceNode.tshape()));
    return nt::ShapeBuilder::makeFace(srf, pcOuter, pcHoles, toNativeOrient(f.Orientation()),
                                      BRep_Tool::Tolerance(f));
  }

  // Rebuild `wire` so each edge carries its 2D pcurve on `f`, keyed to
  // `faceSurfaceNode`. Walks the OCCT face's edges in DFS order alongside the
  // native wire's edges (the bridge preserves child order) and attaches per index.
  nt::Shape withPCurves(const TopoDS_Face& f, const nt::Shape& wire,
                        const std::shared_ptr<const nt::TShape>& faceSurfaceNode) {
    std::vector<nt::Shape> nativeEdges;
    for (nt::Explorer ex(wire, nt::ShapeType::Edge); ex.more(); ex.next())
      nativeEdges.push_back(ex.current());

    std::vector<nt::Shape> edges;
    std::size_t idx = 0;
    for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More() && idx < nativeEdges.size();
         ex.Next(), ++idx) {
      const TopoDS_Edge oe = TopoDS::Edge(ex.Current());
      nt::PCurve pc;
      if (buildPCurve(oe, f, pc))
        edges.push_back(nt::ShapeBuilder::addPCurve(nativeEdges[idx], faceSurfaceNode, pc));
      else
        edges.push_back(nativeEdges[idx]);
    }
    for (; idx < nativeEdges.size(); ++idx) edges.push_back(nativeEdges[idx]);
    return nt::ShapeBuilder::makeWire(std::move(edges));
  }

  nt::Shape container(const TopoDS_Shape& world, nt::ShapeType type) {
    std::vector<nt::Shape> kids;
    for (TopoDS_Iterator it(world, false, false); it.More(); it.Next())
      kids.push_back(bridgeLocalChild(world, it.Value()));
    switch (type) {
      case nt::ShapeType::Wire:      return nt::ShapeBuilder::makeWire(std::move(kids));
      case nt::ShapeType::Shell:     return nt::ShapeBuilder::makeShell(std::move(kids));
      case nt::ShapeType::Solid:     return nt::ShapeBuilder::makeSolid(std::move(kids));
      case nt::ShapeType::CompSolid: return nt::ShapeBuilder::makeCompSolid(std::move(kids));
      default:                       return nt::ShapeBuilder::makeCompound(std::move(kids));
    }
  }

  static nt::EdgeCurve buildEdgeCurve(const TopoDS_Edge& e, double& first, double& last) {
    nt::EdgeCurve c;
    BRepAdaptor_Curve ad(e);
    first = ad.FirstParameter();
    last = ad.LastParameter();
    switch (ad.GetType()) {
      case GeomAbs_Line:
        c.kind = nt::EdgeCurve::Kind::Line;
        c.frame = nm::Ax3{toPt(ad.Line().Location()), toDir(ad.Line().Direction()), {},
                          toDir(ad.Line().Direction())};
        break;
      case GeomAbs_Circle:
        c.kind = nt::EdgeCurve::Kind::Circle;
        c.frame = toAx3(ad.Circle().Position());
        c.radius = ad.Circle().Radius();
        break;
      case GeomAbs_Ellipse:
        c.kind = nt::EdgeCurve::Kind::Ellipse;
        c.frame = toAx3(ad.Ellipse().Position());
        c.radius = ad.Ellipse().MajorRadius();
        c.minorRadius = ad.Ellipse().MinorRadius();
        break;
      default:
        // Non-analytic (BSpline/Other — e.g. a fillet's toroidal blend edges).
        // Marked BSpline; endpoints are not compared here (tessellation uses the
        // pcurve for trimming, not the 3D edge curve).
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
        // Torus (fillet blend) and other analytic surfaces are not modelled in the
        // native FaceSurface payload yet, so they are bridged as their sampled
        // pcurve boundary over a plane placeholder; their vertices/area are checked
        // via the pcurve trim + surface projection. Fillet toroidal faces here fall
        // through to the free-form BSpline path below when poles are present.
        s.kind = nt::FaceSurface::Kind::Plane;
        break;
    }
    return s;
  }

  // Map an OCCT edge's 2D pcurve on face `f` to a native PCurve.
  //
  // WHY THIS MUST PRESERVE THE ANALYTIC FORM (the curved shared-edge weld):
  // A closed solid welds watertight only if the two faces sharing an edge place
  // their boundary vertices at the SAME 3D points. The two-stage mesher gives both
  // faces the SAME shared edge fractions (EdgeCache) and maps each through THAT
  // face's pcurve: S_face(pcurve(f)) must equal C_edge(f) on both. That identity
  // holds only when each face evaluates its pcurve EXACTLY at the shared parameter.
  //
  // Degrading every pcurve to a coarse uniform poly breaks it for CURVED shared
  // edges: a plane's boundary CIRCLE sampled as a 48-chord polygon and re-indexed
  // by an arbitrary fraction lands slightly INSIDE the true circle, while the
  // adjacent cylinder side's LINE pcurve maps to the exact circle — the two
  // disagree by the chord sagitta (the historical "cylinder boundaryEdges≠0" gap).
  //
  // So analytic Line/Circle/Ellipse pcurves are modelled EXACTLY here, matching the
  // native PCurve convention (Line: origin2d + dir2d·t with dir2d the unit
  // direction; Circle/Ellipse: axis-aligned, dir2d.x = major radius, dir2d.y =
  // minor radius, evaluated at the true parameter t). Both adjacent faces then
  // evaluate the identical shared parameter analytically and produce coincident 3D
  // boundary points, so the seam welds watertight. A genuinely free-form pcurve
  // (or a rotated/indirect analytic 2D placement the axis-aligned native form
  // cannot represent) falls back to a dense poly — still fine for the trim
  // boundary; none of the closed-solid fixtures here need that path for a shared
  // curved seam (their curved-edge pcurves are axis-aligned line/circle).
  static bool buildPCurve(const TopoDS_Edge& e, const TopoDS_Face& f, nt::PCurve& out) {
    double f2 = 0, l2 = 0;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(e, f, f2, l2);
    if (c2d.IsNull()) return false;
    Geom2dAdaptor_Curve a2d(c2d);
    switch (a2d.GetType()) {
      case GeomAbs_Line: {
        // Native Line: P(t) = origin2d + dir2d·t, evaluated at the edge's true
        // parameter t (which equals the pcurve parameter for an OCCT pcurve). dir2d
        // is the unit direction; OCCT's Geom2d_Line is Loc + t·Dir with Dir a unit.
        const gp_Lin2d ln = a2d.Line();
        out.kind = nt::EdgeCurve::Kind::Line;
        out.origin2d = nm::Point3{ln.Location().X(), ln.Location().Y(), 0.0};
        out.dir2d = nm::Vec3{ln.Direction().X(), ln.Direction().Y(), 0.0};
        return true;
      }
      case GeomAbs_Circle: {
        const gp_Circ2d cc = a2d.Circle();
        const gp_Dir2d xd = cc.XAxis().Direction();
        // Native Circle is axis-aligned & CCW: center + R(cos t, sin t) over the UV
        // axes. Use it only when OCCT's 2D placement is that (x-axis = +u, direct);
        // otherwise fall through to the exact poly (rare for primitive boundaries).
        const bool axisAligned = std::fabs(xd.X() - 1.0) < 1e-12 && std::fabs(xd.Y()) < 1e-12;
        if (axisAligned && cc.IsDirect()) {
          out.kind = nt::EdgeCurve::Kind::Circle;
          out.origin2d = nm::Point3{cc.Location().X(), cc.Location().Y(), 0.0};
          out.dir2d = nm::Vec3{cc.Radius(), 0.0, 0.0};  // dir2d.x carries the radius
          return true;
        }
        break;  // rotated/indirect → poly fallback
      }
      case GeomAbs_Ellipse: {
        const gp_Elips2d el = a2d.Ellipse();
        const gp_Dir2d xd = el.XAxis().Direction();
        const bool axisAligned = std::fabs(xd.X() - 1.0) < 1e-12 && std::fabs(xd.Y()) < 1e-12;
        if (axisAligned && el.IsDirect()) {
          out.kind = nt::EdgeCurve::Kind::Ellipse;
          out.origin2d = nm::Point3{el.Location().X(), el.Location().Y(), 0.0};
          out.dir2d = nm::Vec3{el.MajorRadius(), el.MinorRadius(), 0.0};
          return true;
        }
        break;
      }
      default:
        break;  // BSpline/other → poly fallback
    }
    // Free-form / non-representable-analytic fallback: dense uniform poly over the
    // pcurve's parameter range. pcurveValue flattens it by fraction over that range.
    out.kind = nt::EdgeCurve::Kind::BSpline;
    const int n = 64;
    out.poles2d.reserve(n + 1);
    for (int i = 0; i <= n; ++i) {
      const double t = f2 + (l2 - f2) * (static_cast<double>(i) / n);
      const gp_Pnt2d p = c2d->Value(t);
      out.poles2d.push_back(nm::Point3{p.X(), p.Y(), 0.0});
    }
    return true;
  }

  std::unordered_map<Key, nt::Shape, KeyHash> nodeCache_;
};

// ═════════════════════════════════════════════════════════════════════════════
// OCCT-side reference geometry: mesh the SAME shape with BRepMesh at the SAME
// deflection (the reference tessellator), and read closed-form area/volume off
// the exact B-rep (BRepGProp). Collected once per shape into `Oracle`.
// ═════════════════════════════════════════════════════════════════════════════
struct Aabb {
  double lo[3] = {1e30, 1e30, 1e30};
  double hi[3] = {-1e30, -1e30, -1e30};
  void add(double x, double y, double z) {
    lo[0] = std::min(lo[0], x); hi[0] = std::max(hi[0], x);
    lo[1] = std::min(lo[1], y); hi[1] = std::max(hi[1], y);
    lo[2] = std::min(lo[2], z); hi[2] = std::max(hi[2], z);
  }
  bool valid() const { return lo[0] <= hi[0]; }
};

// Max per-axis corner discrepancy between two AABBs.
static double aabbDelta(const Aabb& a, const Aabb& b) {
  double d = 0.0;
  for (int k = 0; k < 3; ++k) {
    d = std::max(d, std::fabs(a.lo[k] - b.lo[k]));
    d = std::max(d, std::fabs(a.hi[k] - b.hi[k]));
  }
  return d;
}

struct Oracle {
  double exactArea = 0.0;    ///< BRepGProp::SurfaceProperties on the exact B-rep
  double exactVolume = 0.0;  ///< BRepGProp::VolumeProperties on the exact B-rep
  double occtMeshArea = 0.0; ///< surface area of the OCCT BRepMesh triangulation
  double occtMeshVol = 0.0;  ///< enclosed volume of the OCCT BRepMesh triangulation
  Aabb occtMeshBox;          ///< AABB of the OCCT BRepMesh triangulation nodes
};

// Accumulate the OCCT BRepMesh triangulation's area / signed-volume / AABB, walking
// every face's Poly_Triangulation (world-placed) — the SAME primitives the native
// mesh's surfaceArea/enclosedVolume/AABB use, so the comparison is apples-to-apples.
static void accumulateOcctMesh(const TopoDS_Shape& shape, Oracle& o) {
  double vol6 = 0.0;
  for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) {
    const TopoDS_Face face = TopoDS::Face(ex.Current());
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
    if (tri.IsNull()) continue;
    const gp_Trsf trsf = loc.Transformation();
    const bool reversed = face.Orientation() == TopAbs_REVERSED;
    for (int t = 1; t <= tri->NbTriangles(); ++t) {
      int i1, i2, i3;
      tri->Triangle(t).Get(i1, i2, i3);
      gp_Pnt a = tri->Node(i1).Transformed(trsf);
      gp_Pnt b = tri->Node(i2).Transformed(trsf);
      gp_Pnt c = tri->Node(i3).Transformed(trsf);
      if (reversed) std::swap(b, c);  // outward winding for a Reversed face
      o.occtMeshBox.add(a.X(), a.Y(), a.Z());
      o.occtMeshBox.add(b.X(), b.Y(), b.Z());
      o.occtMeshBox.add(c.X(), c.Y(), c.Z());
      // Triangle area (winding-independent).
      const gp_Vec ab(a, b), ac(a, c);
      o.occtMeshArea += 0.5 * ab.Crossed(ac).Magnitude();
      // Signed tetra volume from origin (divergence theorem), same as native.
      const gp_Vec va(a.X(), a.Y(), a.Z()), vb(b.X(), b.Y(), b.Z()), vc(c.X(), c.Y(), c.Z());
      vol6 += va.Dot(vb.Crossed(vc));
    }
  }
  o.occtMeshVol = std::fabs(vol6 / 6.0);
}

static Oracle buildOracle(const TopoDS_Shape& shape, double defl) {
  Oracle o;
  // Reference tessellator: BRepMesh at the SAME linear deflection.
  BRepMesh_IncrementalMesh mesher(shape, defl, /*isRelative=*/Standard_False);
  mesher.Perform();

  GProp_GProps sprops, vprops;
  BRepGProp::SurfaceProperties(shape, sprops);
  BRepGProp::VolumeProperties(shape, vprops);
  o.exactArea = sprops.Mass();
  o.exactVolume = vprops.Mass();

  accumulateOcctMesh(shape, o);
  return o;
}

// ── Native mesh AABB (same primitive as Oracle::occtMeshBox) ──────────────────
static Aabb nativeBox(const ntess::Mesh& mesh) {
  Aabb b;
  for (const auto& v : mesh.vertices) b.add(v.x, v.y, v.z);
  return b;
}

// ═════════════════════════════════════════════════════════════════════════════
// Property comparisons.
// ═════════════════════════════════════════════════════════════════════════════

// (1) WATERTIGHT — closed 2-manifold (boundaryEdges == 0): every undirected edge
// shared by EXACTLY two triangles. REQUIRED now for every CLOSED solid, INCLUDING
// curved-edge solids (cylinder cap↔side circle, fillet blend seams). The two-stage
// mesher discretizes each unique edge ONCE and pins both adjacent faces to that
// shared discretization, so curved shared edges weld closed — there is no longer a
// weaker "bounded-open" pass.
static void checkWatertight(const char* name, const ntess::Mesh& mesh) {
  const std::size_t be = ntess::boundaryEdgeCount(mesh);
  char buf[128];
  std::snprintf(buf, sizeof(buf), "tris=%zu boundaryEdges=%zu", mesh.triangleCount(), be);
  report(name, "watertight", ntess::isWatertight(mesh), buf);
}

// (2) BOUNDING BOX — native mesh AABB vs OCCT BRepMesh AABB, deflection-scaled.
//
// An axis-aligned extreme of the true surface (e.g. x=+R on a sphere) generally
// lies mid-cell in the parameter grid, not on a grid node, so a parameter-grid
// tessellation samples it slightly INSIDE the true silhouette. The undershoot of
// one axis extreme is the chord sagitta of the arc between the two nearest nodes.
//
// For a SINGLY-curved axis extreme (box corner, cylinder cap rim) the extreme sits
// on a node in the other direction, so the undershoot is one sagitta ≈ deflection
// (box/cylinder/filleted-box measure ≈ 0 because their extremes land exactly on
// nodes). For a DOUBLY-curved surface the axis extreme can be offset mid-cell in
// BOTH u and v at once (the sphere equator when the v-grid has no node at the
// equator and the u-grid has no node at u=π/2): the extreme coordinate is then
// R·cos(Δu)·cos(Δv) and the undershoot is R·(1 − cos(Δu)·cos(Δv)) ≈ the SUM of the
// two per-direction sagittae, i.e. up to ≈ 2·deflection. Both meshes undershoot
// (never overshoot a convex patch), so the worst pairwise corner delta is bounded
// by the larger single-mesh undershoot ≈ 2·deflection.
//
// The bound is therefore 2·deflection (+fp): fully deflection-scaled — a finer
// deflection tightens it proportionally, so it cannot mask a wrong-shape mesh
// (that fails area/volume/watertight/on-surface, all checked independently here).
static void checkBoundingBox(const char* name, const ntess::Mesh& mesh, const Oracle& o,
                             double defl) {
  const Aabb nb = nativeBox(mesh);
  const double d = (nb.valid() && o.occtMeshBox.valid()) ? aabbDelta(nb, o.occtMeshBox) : 1e30;
  const double tol = 2.0 * defl + 1e-6;  // sum of per-direction sagittae, doubly-curved
  char buf[128];
  std::snprintf(buf, sizeof(buf), "maxCornerDelta=%.4e tol=%.3e defl=%.3e", d, tol, defl);
  report(name, "bbox-vs-occt", d <= tol, buf);
}

// (3) AREA — native surfaceArea vs (a) OCCT BRepMesh area and (b) EXACT B-rep area.
// A triangulation under-estimates a convex smooth patch by O(deflection); the
// relative tolerance is deflection-scaled so a finer deflection tightens the bound.
static void checkArea(const char* name, const ntess::Mesh& mesh, const Oracle& o, double relTol) {
  const double got = ntess::surfaceArea(mesh);
  const double relMesh = std::fabs(got - o.occtMeshArea) / std::max(o.occtMeshArea, 1e-12);
  const double relExact = std::fabs(got - o.exactArea) / std::max(o.exactArea, 1e-12);
  char buf[160];
  std::snprintf(buf, sizeof(buf),
                "native=%.4f occtMesh=%.4f exact=%.4f relMesh=%.3e relExact=%.3e tol=%.3e",
                got, o.occtMeshArea, o.exactArea, relMesh, relExact, relTol);
  report(name, "area-vs-occt+exact", relMesh <= relTol && relExact <= relTol, buf);
}

// (4) VOLUME — native |enclosedVolume| vs (a) OCCT BRepMesh volume and (b) EXACT
// B-rep volume. Volume is only meaningful for a (near-)closed mesh; the tolerance
// absorbs the bounded error of a small open boundary on curved-edge solids.
static void checkVolume(const char* name, const ntess::Mesh& mesh, const Oracle& o, double relTol) {
  const double got = std::fabs(ntess::enclosedVolume(mesh));
  const double relMesh = std::fabs(got - o.occtMeshVol) / std::max(o.occtMeshVol, 1e-12);
  const double relExact = std::fabs(got - o.exactVolume) / std::max(o.exactVolume, 1e-12);
  char buf[160];
  std::snprintf(buf, sizeof(buf),
                "native=%.4f occtMesh=%.4f exact=%.4f relMesh=%.3e relExact=%.3e tol=%.3e",
                got, o.occtMeshVol, o.exactVolume, relMesh, relExact, relTol);
  report(name, "volume-vs-occt+exact", relMesh <= relTol && relExact <= relTol, buf);
}

// (5) VERTICES ON THE TRUE SURFACE — every native vertex within `defl` of SOME
// OCCT face surface (projected onto each face's Geom_Surface; min distance wins).
static void checkVerticesOnSurface(const char* name, const TopoDS_Shape& occt,
                                   const ntess::Mesh& mesh, double defl) {
  std::vector<Handle(Geom_Surface)> surfs;
  std::vector<gp_Trsf> xforms;
  for (TopExp_Explorer ex(occt, TopAbs_FACE); ex.More(); ex.Next()) {
    TopLoc_Location loc;
    surfs.push_back(BRep_Tool::Surface(TopoDS::Face(ex.Current()), loc));
    xforms.push_back(loc.Transformation());
  }
  double worst = 0.0;
  for (const auto& vtx : mesh.vertices) {
    double best = 1e30;
    for (std::size_t k = 0; k < surfs.size(); ++k) {
      gp_Pnt p(vtx.x, vtx.y, vtx.z);
      p.Transform(xforms[k].Inverted());  // into the surface's local frame
      GeomAPI_ProjectPointOnSurf proj(p, surfs[k]);
      if (proj.NbPoints() > 0) best = std::min(best, static_cast<double>(proj.LowerDistance()));
    }
    worst = std::max(worst, best);
  }
  char buf[96];
  std::snprintf(buf, sizeof(buf), "maxDist=%.3e defl=%.3e", worst, defl);
  // The chord bound is the sagitta; a grid vertex is S(u,v) exactly, so distance
  // ≈ fp. A modest multiple of the deflection covers pcurve-sampled boundaries.
  report(name, "vertices-on-surface", worst <= defl * 1.5 + 1e-7, buf);
}

// ═════════════════════════════════════════════════════════════════════════════
// Per-shape driver: build the two-sided oracle, bridge, mesh natively, run the
// five property comparisons.
// ═════════════════════════════════════════════════════════════════════════════
static void runSolid(const char* name, const TopoDS_Shape& occt, double defl, double areaTol,
                     double volTol) {
  const Oracle oracle = buildOracle(occt, defl);

  OcctBridge bridge;
  nt::Shape native = bridge.bridge(occt);

  ntess::MeshParams p;
  p.deflection = defl;
  ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(native);

  checkWatertight(name, mesh);  // REQUIRED for every closed solid (incl. curved seams)
  checkBoundingBox(name, mesh, oracle, defl);
  checkArea(name, mesh, oracle, areaTol);
  checkVolume(name, mesh, oracle, volTol);
  checkVerticesOnSurface(name, occt, mesh, defl);
}

int main() {
  std::printf("== native-tessellation vs OCCT-oracle parity ==\n");
  std::fflush(stdout);

  // Box: 6 planar faces ⇒ area/volume EXACT, watertight after welding; the OCCT
  // BRepMesh and native mesh AABBs are the box's exact corners.
  runSolid("box", BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape(), 0.1,
           /*areaTol=*/0.02, /*volTol=*/0.02);

  // Cylinder: planar caps + cylindrical side sharing CURVED circular edges. The
  // shared per-edge discretization makes the cap↔side seam samples coincide, so the
  // solid meshes WATERTIGHT (the gap this change closes); area/volume/bbox converge.
  runSolid("cylinder", BRepPrimAPI_MakeCylinder(5.0, 12.0).Shape(), 0.05,
           /*areaTol=*/0.02, /*volTol=*/0.02);

  // Sphere: a single doubly-curved face; u-seam + poles weld ⇒ watertight. A finer
  // deflection makes the pole region converge under tolerance.
  runSolid("sphere", BRepPrimAPI_MakeSphere(7.0).Shape(), 0.02,
           /*areaTol=*/0.02, /*volTol=*/0.02);

  // Filleted box: planar + cylindrical (edge blends) + spherical (corner blends)
  // faces, the richest fixture. Every blend face meets its neighbours along curved
  // edges routed through the SHARED per-edge discretization, so the whole closed
  // solid meshes WATERTIGHT. A looser area/volume tolerance is kept (the torus
  // blend surfaces are bridged onto a plane placeholder in the harness, so the
  // blend-face geometry is approximate — but watertightness is topological and
  // holds regardless).
  {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
    BRepFilletAPI_MakeFillet mf(box);
    for (TopExp_Explorer ex(box, TopAbs_EDGE); ex.More(); ex.Next())
      mf.Add(2.0, TopoDS::Edge(ex.Current()));
    TopoDS_Shape fil = mf.Shape();
    runSolid("filleted-box", fil, 0.05, /*areaTol=*/0.05, /*volTol=*/0.05);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
