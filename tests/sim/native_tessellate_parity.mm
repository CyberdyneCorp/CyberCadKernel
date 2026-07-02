// SPDX-License-Identifier: Apache-2.0
//
// native_tessellate_parity.mm — native-tessellation vs OCCT-oracle parity
//                               harness (iOS simulator).
//
// Phase 4 capability #3 (`native-tessellation`), simulator verification gate 2
// (see openspec/NATIVE-REWRITE.md). The native tessellator (src/native/
// tessellate/*, an OCCT-FREE clean-room mesher on src/native/math +
// src/native/topology) meshes shapes that are bridged from OCCT, and the mesh is
// checked against the OCCT ORACLE via TOLERANCE-BASED PROPERTIES — never
// triangle-identical output (tessellation is an approximation):
//
//   1. VERTICES ON THE TRUE SURFACE — every native mesh vertex projects onto the
//      OCCT face surface within the deflection bound (GeomAPI_ProjectPointOnSurf
//      over each face's Geom_Surface; a vertex must be within `deflection` of
//      SOME face of the shape).
//   2. AREA CONVERGENCE — native surfaceArea within a deflection-derived relative
//      tolerance of BRepGProp::SurfaceProperties on the OCCT shape.
//   3. VOLUME CONVERGENCE — for a closed solid, native |enclosedVolume| within
//      tolerance of BRepGProp::VolumeProperties.
//   4. WATERTIGHT — the solid mesh is a closed 2-manifold (every interior edge
//      shared by exactly two triangles), the property a closed solid must have.
//
// ── The OCCT→native bridge (TEST-ONLY, reused pattern) ───────────────────────
// This harness REUSES the bridge pattern from native_topology_parity.mm: a
// recursive walk that reads geometry/orientation/location off OCCT via BRep_Tool
// + the adaptors and rebuilds an equivalent native graph through ShapeBuilder,
// memoising shared TShape nodes so adjacency (shared edges) is preserved. The
// bridge lives HERE, in the harness (which links OCCT); nothing under src/native
// gains an OCCT dependency. It is EXTENDED here to also lay each edge's PCURVE on
// its faces (BRep_Tool::CurveOnSurface → native PCurve), which the tessellator's
// trimming needs — the topology parity harness did not require pcurves, this one
// does.
//
// Output: [NTESS] PASS/FAIL lines, then "== N passed, M failed ==". Flushes and
// std::_Exit (OCCT static teardown in the trimmed static build is not exit-clean
// — same rationale as the other sim harnesses; every handle here is RAII-scoped).
//
// Build: scripts/run-sim-native-tessellate.sh (mirrors run-sim-native-topology).

#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_tessellate_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
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
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Geom_Surface.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>

namespace nt = cybercad::native::topology;
namespace nm = cybercad::native::math;
namespace ntess = cybercad::native::tessellate;

// ═════════════════════════════════════════════════════════════════════════════
// Result accounting.
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
// Same shared-node walk as native_topology_parity's OcctBridge, EXTENDED so that
// after a face's wires are bridged, each edge of the face gets its 2D pcurve on
// that face laid on via ShapeBuilder::addPCurve (BRep_Tool::CurveOnSurface). The
// tessellator's trimming (trim.h) samples those pcurves to build the UV boundary
// polygon; without them the mesher falls back to the surface's natural bounds,
// which for the analytic primitives here (box faces, cylinder/sphere) still spans
// the face — but we lay them so trimming is exercised faithfully.
//
// Because native TShape nodes are immutable, the pcurve is attached by REBUILDING
// the edge node (addPCurve returns a new edge) and rebuilding the wire/face with
// the pcurve-carrying edges. To keep node sharing across faces, we build faces in
// two passes: first bridge the plain graph (shared nodes), then re-derive faces
// with pcurves. For this harness a simpler, faithful route is used: since each
// bridged edge is laid onto the ≤2 faces that reference it and the tessellator's
// pcurveForFace fallback accepts an edge's single pcurve, we attach the pcurve of
// the CURRENT face while bridging that face's edges. Analytic parametrizations of
// OCCT and the native math library agree (verified in native_topology parity), so
// the 2D pcurve params line up.
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
    // First bridge wires (shared nodes) as-is to get the plain face node — its
    // TShape identity is the pcurve key.
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
    // Rebuild each wire with pcurve-carrying edges keyed to faceNode.
    nt::Shape pcOuter = withPCurves(f, plainOuter, faceNode.tshape());
    std::vector<nt::Shape> pcHoles;
    for (const nt::Shape& h : plainHoles) pcHoles.push_back(withPCurves(f, h, faceNode.tshape()));
    return nt::ShapeBuilder::makeFace(srf, pcOuter, pcHoles, toNativeOrient(f.Orientation()),
                                      BRep_Tool::Tolerance(f));
  }

  // Rebuild `wire` so each edge carries its 2D pcurve on `f`, keyed to
  // `faceSurfaceNode`. Reads BRep_Tool::CurveOnSurface per edge; maps the OCCT
  // Geom2d curve to a native PCurve (analytic line/circle/ellipse; else sampled
  // poly). Edge/vertex node SHARING is preserved because addPCurve clones only
  // the edge node's data (children stay shared).
  nt::Shape withPCurves(const TopoDS_Face& f, const nt::Shape& wire,
                        const std::shared_ptr<const nt::TShape>& faceSurfaceNode) {
    std::vector<nt::Shape> edges;
    // Walk the OCCT wire's edges in order alongside the native wire's edges. We
    // rely on same-order traversal (the bridge preserves child order).
    std::vector<nt::Shape> nativeEdges;
    for (nt::Explorer ex(wire, nt::ShapeType::Edge); ex.more(); ex.next())
      nativeEdges.push_back(ex.current());

    // Collect the OCCT wire that corresponds — re-find it by matching the face's
    // wires. Simpler: iterate the face's edges (with their pcurves) in the same
    // DFS order the native explorer used; attach per index.
    std::size_t idx = 0;
    for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More() && idx < nativeEdges.size(); ex.Next(), ++idx) {
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
        s.kind = nt::FaceSurface::Kind::Plane;
        break;
    }
    return s;
  }

  // Map an OCCT edge's 2D pcurve on face `f` to a native PCurve. Analytic pcurves
  // (line/circle/ellipse in the (u,v) plane) are modelled exactly; anything else
  // is sampled to a poly (poles2d) so trimming still gets a boundary. The pcurve
  // is evaluated in the surface's parameter domain, matching how the native
  // surface is parametrized (validated by native_topology parity).
  static bool buildPCurve(const TopoDS_Edge& e, const TopoDS_Face& f, nt::PCurve& out) {
    double f2 = 0, l2 = 0;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(e, f, f2, l2);
    if (c2d.IsNull()) return false;
    // Sample the 2D pcurve into a poly; robust for any pcurve kind. The
    // tessellator flattens polys by arclength fraction, so we store enough
    // samples for the boundary. Params run [f2,l2] over the poly's [0,1].
    out.kind = nt::EdgeCurve::Kind::BSpline;  // treat as free-form poly
    const int n = 32;
    out.poles2d.reserve(n + 1);
    for (int i = 0; i <= n; ++i) {
      const double t = f2 + (l2 - f2) * (static_cast<double>(i) / n);
      gp_Pnt2d p = c2d->Value(t);
      out.poles2d.push_back(nm::Point3{p.X(), p.Y(), 0.0});
    }
    return true;
  }

  std::unordered_map<Key, nt::Shape, KeyHash> nodeCache_;
};

// ═════════════════════════════════════════════════════════════════════════════
// Property comparisons against the OCCT oracle.
// ═════════════════════════════════════════════════════════════════════════════

// (1) Every native mesh vertex is within `defl` of SOME OCCT face surface.
static void checkVerticesOnSurface(const char* name, const TopoDS_Shape& occt,
                                   const ntess::Mesh& mesh, double defl) {
  // Collect each face's Geom_Surface + a projector. Projecting onto every face and
  // taking the min distance is O(V·F) but fine for these small fixtures.
  std::vector<Handle(Geom_Surface)> surfs;
  std::vector<gp_Trsf> xforms;
  for (TopExp_Explorer ex(occt, TopAbs_FACE); ex.More(); ex.Next()) {
    TopLoc_Location loc;
    Handle(Geom_Surface) s = BRep_Tool::Surface(TopoDS::Face(ex.Current()), loc);
    surfs.push_back(s);
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
  // Allow a small multiple of the deflection (chord error is measured to the
  // surface; the bound is the sagitta, and projection distance ≤ that plus fp).
  report(name, "vertices-on-surface", worst <= defl * 1.5 + 1e-7, buf);
}

// (2) Native surface area vs BRepGProp::SurfaceProperties.
static void checkArea(const char* name, const TopoDS_Shape& occt, const ntess::Mesh& mesh,
                      double relTol) {
  GProp_GProps props;
  BRepGProp::SurfaceProperties(occt, props);
  const double truth = props.Mass();
  const double got = ntess::surfaceArea(mesh);
  const double rel = std::fabs(got - truth) / std::max(truth, 1e-12);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "native=%.4f occt=%.4f rel=%.3e tol=%.3e", got, truth, rel, relTol);
  report(name, "area-convergence", rel <= relTol, buf);
}

// (3) Native enclosed volume vs BRepGProp::VolumeProperties (closed solids).
static void checkVolume(const char* name, const TopoDS_Shape& occt, const ntess::Mesh& mesh,
                        double relTol) {
  GProp_GProps props;
  BRepGProp::VolumeProperties(occt, props);
  const double truth = props.Mass();
  const double got = std::fabs(ntess::enclosedVolume(mesh));
  const double rel = std::fabs(got - truth) / std::max(truth, 1e-12);
  char buf[96];
  std::snprintf(buf, sizeof(buf), "native=%.4f occt=%.4f rel=%.3e tol=%.3e", got, truth, rel, relTol);
  report(name, "volume-convergence", rel <= relTol, buf);
}

// (4) Watertight (closed 2-manifold): every undirected edge shared by EXACTLY two
// triangles (boundaryEdges == 0). This is now REQUIRED for every CLOSED solid —
// including CURVED shared edges (a cylinder's circular cap↔side seam). The mesher
// discretizes each unique edge ONCE (STAGE 1) and pins both adjacent faces to that
// shared discretization (STAGE 2), so the seam samples coincide and weld closed;
// there is no longer a weaker "bounded-open" pass for curved-edge solids.
static void checkWatertight(const char* name, const ntess::Mesh& mesh) {
  const std::size_t be = ntess::boundaryEdgeCount(mesh);
  char buf[112];
  std::snprintf(buf, sizeof(buf), "tris=%zu boundaryEdges=%zu", mesh.triangleCount(), be);
  report(name, "watertight", ntess::isWatertight(mesh), buf);
}

// ═════════════════════════════════════════════════════════════════════════════
// Per-shape driver: bridge, mesh natively, run the four property checks.
// ═════════════════════════════════════════════════════════════════════════════
static void runSolid(const char* name, const TopoDS_Shape& occt, double defl, double volTol) {
  OcctBridge bridge;
  nt::Shape native = bridge.bridge(occt);

  ntess::MeshParams p;
  p.deflection = defl;
  ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(native);

  checkVerticesOnSurface(name, occt, mesh, defl);
  checkArea(name, occt, mesh, /*relTol=*/0.02);
  checkVolume(name, occt, mesh, volTol);
  checkWatertight(name, mesh);  // REQUIRED for every closed solid (incl. curved seams)
}

int main() {
  std::printf("== native-tessellation vs OCCT-oracle parity ==\n");
  std::fflush(stdout);

  // Box: 6 planar faces ⇒ area/volume EXACT, watertight after welding.
  runSolid("box", BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape(), 0.1, /*volTol=*/0.02);
  // Cylinder: planar caps + cylindrical side sharing CURVED circular edges. The
  // shared per-edge discretization makes the cap↔side seam samples coincide, so the
  // solid now meshes WATERTIGHT (the gap this change closes); area/volume converge.
  runSolid("cylinder", BRepPrimAPI_MakeCylinder(5.0, 12.0).Shape(), 0.05, /*volTol=*/0.02);
  // Sphere: doubly-curved single face; u-seam + poles welded ⇒ watertight. A finer
  // deflection is used so the pole-region volume converges under 2%.
  runSolid("sphere", BRepPrimAPI_MakeSphere(7.0).Shape(), 0.02, /*volTol=*/0.02);

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
