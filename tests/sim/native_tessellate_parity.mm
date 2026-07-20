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
//   0. BRIDGE PCURVE FIDELITY — before any mesh property is judged, the pcurves the
//      test bridge feeds the mesher are audited against the oracle:
//      S_face(pcurve(t)) == C_edge(t) to 1e-9. A bridge defect otherwise surfaces as
//      a mesh-property failure and reads as a mesher bug (it did — see buildPCurve).
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
#include <Geom2dAdaptor_Curve.hxx>
#include <gp_Ax22d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Elips2d.hxx>
#include <gp_Lin2d.hxx>
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
// polygon. The pcurves are REQUIRED, not a fidelity nicety: with no usable pcurve
// the mesher falls back to the surface's natural UV bounds, which do NOT span these
// faces — measured, a box bridged without resolvable pcurves meshes to area 6.0
// against the true 2200.0.
//
// ── WHAT THIS BRIDGE DOES NOT PRESERVE: EDGE-NODE SHARING ────────────────────
// Native TShape nodes are immutable, so a pcurve is attached by REBUILDING the edge
// node (addPCurve returns a new edge) and rebuilding the wire/face on top of it.
// Since each face lays its OWN pcurve, each face ends up with its OWN clone of every
// edge it uses, and edge-node sharing across faces is LOST. Measured on the fixtures
// below (AncestryMap over the bridged graph vs TopExp::MapShapesAndAncestors):
//
//   box       OCCT 12 edges x 2 faces  ->  bridged 24 edge nodes x 1 face
//   cylinder  OCCT  3 edges x 2 faces  ->  bridged  5 edge nodes x 1 face
//
// This is INHERENT, not an oversight to be tidied up later. A pcurve is keyed by the
// FACE NODE POINTER it lies on (topo::pcurveOf compares tshape().get()), and a face
// node cannot exist before the edges it contains — so one shared edge node cannot
// carry pcurves keyed to its own incident final face nodes. Building each edge once
// with both pcurves keyed to the pre-pcurve face nodes was implemented and measured:
// pcurveForFace then resolves NEITHER key (the exact match misses and the
// single-pcurve fallback does not apply to two), the mesher falls back to natural UV
// bounds, and the box's area collapses from 2200 to 6.0. Restoring sharing requires a
// kernel-side change to how pcurves are keyed, not a harness change.
//
// Nor does the mesher need the sharing: it welds curved seams carried on SEPARATE
// nodes via edge_mesher's canonical endpoint+midpoint fallback. Measured — the
// kernel's own construct::build_revolution cylinder (r=5, h=12) has 9 faces and 30
// edge nodes, EVERY one incident to a single face, and meshes with 0 boundary edges.
// So do not read a watertight failure here as "the bridge lost sharing"; check
// bridge-pcurve-fidelity (check 0) first, which is what actually broke.
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
  // poly).
  //
  // ⚠ EDGE-NODE SHARING IS **NOT** PRESERVED HERE. `addPCurve` returns a NEW edge
  // node, and a pcurve is keyed by the face-surface node it lies on, so an edge
  // shared by two faces necessarily yields two distinct nodes — one per face.
  // Measured on the box fixture: 24 edge nodes, each referenced by exactly 1 face,
  // i.e. ZERO sharing. See the header block above for why that is a property of
  // this BRIDGE and not of the kernel mesher, and why the obvious "build each edge
  // node once" fix is structurally unavailable (a face node cannot pre-exist its
  // own edges).
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

  std::unordered_map<Key, nt::Shape, KeyHash> nodeCache_;

 public:
  // Map an OCCT edge's 2D pcurve on face `f` to a native PCurve. Public because
  // checkPCurveFidelity() below audits exactly what this produces.
  //
  // ── ANALYTIC FIRST, AND WHY IT IS LOAD-BEARING ──────────────────────────────
  // topo::PCurve's free-form kind stores bare 2D poles with NO knot vector, and
  // tessellate::pcurveValue() then LERPS them (trim.h, the poles2d fallback). What
  // the mesher traces is therefore the CHORD POLYGON of the pcurve, not the pcurve.
  // On a pcurve that is STRAIGHT in (u,v) the chords are exact; on a CURVED one the
  // polygon is inset by the sagitta.
  //
  // That asymmetry is what used to open this harness's cylinder. Measured on
  // BRepPrimAPI_MakeCylinder(5,12) with the previous unconditional 32-pole poly: the
  // cap's seam ring landed at r = 4.9797 (its pcurve is a CIRCLE in the plane's
  // (u,v)) while the side face's ring landed at r = 5.0 exactly (its pcurve is a LINE
  // in (theta,h), so chords are exact). Both rings had 23 points at matching angles
  // but sat 2.0e-2 apart — orders of magnitude beyond any weld tolerance — so the
  // solid meshed with 84 open boundary edges, 42 at each cap seam. Read off the
  // watertight assertion alone that looks exactly like a mesher leak on cylinders.
  // It was this function.
  //
  // So: transcribe the analytic forms exactly and keep the poly only for genuinely
  // free-form pcurves, where no exact target exists.
  static bool buildPCurve(const TopoDS_Edge& e, const TopoDS_Face& f, nt::PCurve& out) {
    double f2 = 0, l2 = 0;
    Handle(Geom2d_Curve) c2d = BRep_Tool::CurveOnSurface(e, f, f2, l2);
    if (c2d.IsNull()) return false;

    // pcurveValue() drives an ANALYTIC pcurve with the EDGE's 3-D parameter, so the
    // analytic transcription is only valid when the 2D and 3D ranges coincide. When
    // they do not, the poly path is still correct: it is indexed by the [0,1]
    // fraction across the range, which is parametrisation-independent.
    BRepAdaptor_Curve ad3(e);
    const double a3 = ad3.FirstParameter(), b3 = ad3.LastParameter();
    const double rangeTol = 1e-9 * (1.0 + std::fabs(a3) + std::fabs(b3));
    if (std::fabs(f2 - a3) <= rangeTol && std::fabs(l2 - b3) <= rangeTol &&
        buildAnalyticPCurve(Geom2dAdaptor_Curve(c2d, f2, l2), out))
      return true;

    out = nt::PCurve{};
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

 private:
  // Term-for-term transcription into the analytic forms pcurveValue() evaluates:
  //   Line    -> origin2d + t * dir2d
  //   Circle  -> origin2d + R * (cos t, sin t)         [R read from dir2d.x]
  //   Ellipse -> origin2d + (a cos t, b sin t)         [a,b from dir2d.x/.y]
  // topo::PCurve carries no 2D frame for the conics, so their axes are IMPLICITLY
  // +u/+v. A rotated or reflected OCCT conic would be traced at the wrong phase and
  // silently produce a plausible-looking but wrong boundary, so it is rejected here
  // and falls through to the sampled poly, which has no frame assumption.
  static bool buildAnalyticPCurve(const Geom2dAdaptor_Curve& ad, nt::PCurve& out) {
    const auto axesAreUV = [](const gp_Ax22d& pos) {
      return std::fabs(pos.XDirection().X() - 1.0) <= 1e-12 &&
             std::fabs(pos.XDirection().Y()) <= 1e-12 &&
             std::fabs(pos.YDirection().Y() - 1.0) <= 1e-12 &&
             std::fabs(pos.YDirection().X()) <= 1e-12;
    };
    out = nt::PCurve{};
    switch (ad.GetType()) {
      case GeomAbs_Line: {
        const gp_Lin2d l = ad.Line();
        out.kind = nt::EdgeCurve::Kind::Line;
        out.origin2d = nm::Point3{l.Location().X(), l.Location().Y(), 0.0};
        out.dir2d = nm::Vec3{l.Direction().X(), l.Direction().Y(), 0.0};
        return true;
      }
      case GeomAbs_Circle: {
        const gp_Circ2d c = ad.Circle();
        if (!axesAreUV(c.Position())) return false;
        out.kind = nt::EdgeCurve::Kind::Circle;
        out.origin2d = nm::Point3{c.Location().X(), c.Location().Y(), 0.0};
        out.dir2d = nm::Vec3{c.Radius(), 0.0, 0.0};
        return true;
      }
      case GeomAbs_Ellipse: {
        const gp_Elips2d el = ad.Ellipse();
        if (!axesAreUV(el.Axis())) return false;
        out.kind = nt::EdgeCurve::Kind::Ellipse;
        out.origin2d = nm::Point3{el.Location().X(), el.Location().Y(), 0.0};
        out.dir2d = nm::Vec3{el.MajorRadius(), el.MinorRadius(), 0.0};
        return true;
      }
      default:
        return false;
    }
  }
};

// ═════════════════════════════════════════════════════════════════════════════
// Property comparisons against the OCCT oracle.
// ═════════════════════════════════════════════════════════════════════════════

// (0) BRIDGE-CONSTRUCTION FIDELITY — the pcurve `OcctBridge::buildPCurve` PRODUCES
// must reproduce the edge's 3-D curve through the face surface:
//
//     S_face( pcurve(t) ) == C_edge(t)   for all t in [first,last]
//
// That identity IS the seam-weld contract: two faces meeting at an edge only weld
// if each independently lands on the same 3-D points, and each gets there through
// its own pcurve. A pcurve that is merely CLOSE traces a boundary that is close to
// the edge and welds nowhere.
//
// ⚠ SCOPE — read this before trusting a PASS. This audits the CONSTRUCTION PATH, not
// the delivery. It re-invokes the static `OcctBridge::buildPCurve` on each (edge,
// face) pair and never inspects the bridged `nt::Shape` the mesher is actually handed.
// So it proves the recipe is right; it does NOT prove the recipe reached the mesher.
// Verified deliberately: breaking pcurve ATTACHMENT badly enough to collapse the box's
// measured area from 2200 to 6.0 still leaves this check PASSING on all three fixtures.
// Attachment is covered downstream, by watertight + area + volume; this check exists to
// localise a CONSTRUCTION regression, which is what previously surfaced three checks
// later as `cylinder watertight FAIL` and read as a mesher leak.
//
// Tolerance is 1e-9 (an exact transcription, not an approximation) — the old 32-pole
// poly measured 2.0e-2 on the cylinder cap, so the deviation bound has real
// discriminating power rather than being satisfied by construction.
static void checkPCurveFidelity(const char* name, const TopoDS_Shape& occt,
                                int expectedPCurves) {
  double worst = 0.0;
  int pcurves = 0, samples = 0;
  for (TopExp_Explorer fx(occt, TopAbs_FACE); fx.More(); fx.Next()) {
    const TopoDS_Face f = TopoDS::Face(fx.Current());
    TopLoc_Location sloc;
    Handle(Geom_Surface) srf = BRep_Tool::Surface(f, sloc);
    if (srf.IsNull()) continue;
    for (TopExp_Explorer ex(f, TopAbs_EDGE); ex.More(); ex.Next()) {
      const TopoDS_Edge e = TopoDS::Edge(ex.Current());
      nt::PCurve pc;
      if (!OcctBridge::buildPCurve(e, f, pc)) continue;
      ++pcurves;
      BRepAdaptor_Curve ad3(e);
      const double a = ad3.FirstParameter(), b = ad3.LastParameter();
      const int n = 64;
      for (int i = 0; i <= n; ++i) {
        const double frac = static_cast<double>(i) / n;
        const double t = a + (b - a) * frac;
        const ntess::UV uv = ntess::pcurveValue(pc, t, frac);
        gp_Pnt onSurface = srf->Value(uv.u, uv.v);
        onSurface.Transform(sloc.Transformation());
        worst = std::max(worst, onSurface.Distance(ad3.Value(t)));
        ++samples;
      }
    }
  }
  // COVERAGE IS ASSERTED AGAINST AN EXACT PER-FIXTURE COUNT, not a floor. A bridge
  // change that stopped emitting pcurves would otherwise leave `worst` at 0.0 and pass
  // vacuously — and a floor does not close that: `samples == 65 * pcurves` by
  // construction above, so `pcurves > 0 && samples >= 64` reduces to `pcurves > 0`, and
  // auditing 1 of the box's 24 pcurves would still have passed. The exact count (box 24,
  // cylinder 6, sphere 4 — each face's edges, counted per face, so a shared edge counts
  // once per face it bounds) fails if even one pcurve stops being produced.
  const bool exercised = (pcurves == expectedPCurves) && (samples == 65 * expectedPCurves);
  char buf[160];
  std::snprintf(buf, sizeof(buf), "maxDev=%.3e pcurves=%d (want %d) samples=%d", worst, pcurves,
                expectedPCurves, samples);
  report(name, "bridge-pcurve-fidelity", exercised && worst <= 1e-9, buf);
}

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
// triangles (boundaryEdges == 0). REQUIRED for every CLOSED solid — including the
// CURVED cap↔side seam of a cylinder; there is no weaker "bounded-open" pass.
//
// Note what actually makes the seam weld HERE. The mesher's per-TShape shared
// discretization never fires in this harness, because the bridge hands each face its
// own edge node (see the bridge header). What welds these seams is edge_mesher's
// canonical curved-edge fallback, which matches separate nodes by quantized endpoints
// plus a 3-D midpoint and gives both faces bit-identical samples. That fallback can
// only work if both faces reach the same 3-D curve in the first place, which is what
// check 0 (bridge-pcurve-fidelity) guarantees.
static void checkWatertight(const char* name, const ntess::Mesh& mesh) {
  const std::size_t be = ntess::boundaryEdgeCount(mesh);
  char buf[112];
  std::snprintf(buf, sizeof(buf), "tris=%zu boundaryEdges=%zu", mesh.triangleCount(), be);
  report(name, "watertight", ntess::isWatertight(mesh), buf);
}

// ═════════════════════════════════════════════════════════════════════════════
// Per-shape driver: bridge, mesh natively, run the four property checks.
// ═════════════════════════════════════════════════════════════════════════════
static void runSolid(const char* name, const TopoDS_Shape& occt, double defl, double volTol,
                     int expectedPCurves) {
  OcctBridge bridge;
  nt::Shape native = bridge.bridge(occt);

  ntess::MeshParams p;
  p.deflection = defl;
  ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(native);

  // Audit the bridge's CONSTRUCTION path before judging the mesher (see the scope note).
  checkPCurveFidelity(name, occt, expectedPCurves);
  checkVerticesOnSurface(name, occt, mesh, defl);
  checkArea(name, occt, mesh, /*relTol=*/0.02);
  checkVolume(name, occt, mesh, volTol);
  checkWatertight(name, mesh);  // REQUIRED for every closed solid (incl. curved seams)
}

int main() {
  std::printf("== native-tessellation vs OCCT-oracle parity ==\n");
  std::fflush(stdout);

  // Box: 6 planar faces ⇒ area/volume EXACT, watertight after welding.
  runSolid("box", BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape(), 0.1, /*volTol=*/0.02,
           /*expectedPCurves=*/24);
  // Cylinder: planar caps + cylindrical side sharing CURVED circular edges. The
  // shared per-edge discretization makes the cap↔side seam samples coincide, so the
  // solid now meshes WATERTIGHT (the gap this change closes); area/volume converge.
  runSolid("cylinder", BRepPrimAPI_MakeCylinder(5.0, 12.0).Shape(), 0.05, /*volTol=*/0.02,
           /*expectedPCurves=*/6);
  // Sphere: doubly-curved single face; u-seam + poles welded ⇒ watertight. A finer
  // deflection is used so the pole-region volume converges under 2%.
  runSolid("sphere", BRepPrimAPI_MakeSphere(7.0).Shape(), 0.02, /*volTol=*/0.02,
           /*expectedPCurves=*/4);

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
