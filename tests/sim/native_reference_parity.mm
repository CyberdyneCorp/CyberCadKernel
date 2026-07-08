// SPDX-License-Identifier: Apache-2.0
//
// native_reference_parity.mm — MOAT M-REF native-vs-OCCT parity harness (iOS
// simulator). SIM GATE (b) of the two-gate discipline; gate (a) is the OCCT-free
// host suite tests/native/test_native_reference.cpp.
//
// The native OCCT-FREE, header-only reference services (src/native/reference/
// reference.h) are asserted against the OCCT ORACLE on IDENTICAL primitives built
// on both sides:
//   * refPlaneFromFace   vs  gp_Pln (BRepAdaptor_Surface, outward normal flipped
//                            for a REVERSED face) — normal equal, origins coplanar.
//   * faceAxis / refAxisFromFace  vs  gp_Cylinder::Axis — origin-on-axis + dir ∥.
//   * refAxisFromEdge    vs  gp_Lin (BRepAdaptor_Curve) — origin-on-line + dir ∥.
//   * outerRimChain      vs  BRepTools::OuterWire — the loop vertex SET matches.
//   * offsetFaceBoundary vs  BRepOffsetAPI_MakeOffset (inward, sharp corners) —
//                            enclosed area + bbox extents match.
//   * tangentChain       vs  BRepAdaptor_Curve::D1 tangent oracle — native's C1
//                            grow/stop decision agrees with OCCT's |t1·t2| test.
//
// Matching is GEOMETRIC (faces by normal, edges by midpoint), so it is robust to
// native-vs-OCCT sub-shape id ordering. OCCT-DEPENDENT; compiled ONLY by
// scripts/run-sim-native-reference.sh; carries its own main(); std::_Exit to skip
// the non-exit-clean OCCT static teardown (same as native_query_parity).
//
#include "native/reference/reference.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_reference_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

namespace ref = cybercad::native::reference;
namespace topo = cybercad::native::topology;
namespace nm = cybercad::native::math;

using nm::Ax3;
using nm::Dir3;
using nm::Point3;
using nm::Vec3;
using topo::EdgeCurve;
using topo::FaceSurface;
using topo::Shape;
using topo::ShapeBuilder;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, bool ok) {
  std::printf("[MREF] %-46s %s\n", name, ok ? "PASS" : "FAIL");
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

static constexpr double kPi = 3.14159265358979323846;

// ── native fixtures (welded quads / box / cylinder) ───────────────────────────
static Shape lineEdge(const Shape& v0, const Shape& v1) {
  const Point3 a = *topo::pointOf(v0), b = *topo::pointOf(v1);
  EdgeCurve c; c.kind = EdgeCurve::Kind::Line;
  c.frame = Ax3{a, Dir3{b - a}, Dir3{}, Dir3{}};
  return ShapeBuilder::makeEdge(c, 0.0, nm::norm(b - a), v0, v1);
}
static Shape circleWire(const Point3& ctr, double R) {
  EdgeCurve c; c.kind = EdgeCurve::Kind::Circle;
  c.frame = Ax3::fromAxisAndRef(ctr, Dir3{0, 0, 1}, Dir3{1, 0, 0});
  c.radius = R;
  return ShapeBuilder::makeWire({ShapeBuilder::makeEdge(c, 0, 2 * kPi, Shape{}, Shape{})});
}
static Shape weldedQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3,
                        const Dir3& n) {
  Shape v0 = ShapeBuilder::makeVertex(p0), v1 = ShapeBuilder::makeVertex(p1);
  Shape v2 = ShapeBuilder::makeVertex(p2), v3 = ShapeBuilder::makeVertex(p3);
  FaceSurface s; s.kind = FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, n, Dir3{p1 - p0});
  return ShapeBuilder::makeFace(
      s, ShapeBuilder::makeWire({lineEdge(v0, v1), lineEdge(v1, v2), lineEdge(v2, v3),
                                 lineEdge(v3, v0)}));
}
static Shape nativeBox(double lx, double ly, double lz) {
  const Point3 a{0, 0, 0}, b{lx, 0, 0}, c{lx, ly, 0}, d{0, ly, 0};
  const Point3 e{0, 0, lz}, f{lx, 0, lz}, g{lx, ly, lz}, h{0, ly, lz};
  std::vector<Shape> faces = {
      weldedQuad(a, d, c, b, Dir3{0, 0, -1}), weldedQuad(e, f, g, h, Dir3{0, 0, 1}),
      weldedQuad(a, b, f, e, Dir3{0, -1, 0}), weldedQuad(d, h, g, c, Dir3{0, 1, 0}),
      weldedQuad(a, e, h, d, Dir3{-1, 0, 0}), weldedQuad(b, c, g, f, Dir3{1, 0, 0})};
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell(std::move(faces))});
}
static Shape nativeCylinder(double R, double H) {
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, -1}, Dir3{1, 0, 0});
  Shape bottom = ShapeBuilder::makeFace(bs, circleWire({0, 0, 0}, R));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0, 0, H}, Dir3{0, 0, 1}, Dir3{1, 0, 0});
  Shape top = ShapeBuilder::makeFace(ts, circleWire({0, 0, H}, R));
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cylinder;
  ls.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0});
  ls.radius = R;
  Shape lat = ShapeBuilder::makeFace(
      ls, ShapeBuilder::makeWire({ShapeBuilder::makeEdge(
              [R] { EdgeCurve c; c.kind = EdgeCurve::Kind::Circle;
                    c.frame = Ax3::fromAxisAndRef(Point3{0, 0, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0});
                    c.radius = R; return c; }(), 0, 2 * kPi, Shape{}, Shape{})}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lat})});
}
static Shape faceOf(const Shape& s, int id) {
  return topo::mapShapes(s, topo::ShapeType::Face).shape(id);
}

// ── geometry helpers ──────────────────────────────────────────────────────────
static bool vclose(double ax, double ay, double az, double bx, double by, double bz,
                   double t = 1e-7) {
  return std::fabs(ax - bx) <= t && std::fabs(ay - by) <= t && std::fabs(az - bz) <= t;
}

// ── refPlaneFromFace vs gp_Pln (per box face, matched by outward normal) ───────
static void checkBoxPlanes() {
  const Shape box = nativeBox(2, 3, 4);
  const TopoDS_Shape occt = BRepPrimAPI_MakeBox(2, 3, 4).Shape();
  int matched = 0;
  bool allOk = true;
  for (TopExp_Explorer ex(occt, TopAbs_FACE); ex.More(); ex.Next()) {
    const TopoDS_Face f = TopoDS::Face(ex.Current());
    BRepAdaptor_Surface surf(f);
    gp_Dir on = surf.Plane().Axis().Direction();
    if (f.Orientation() == TopAbs_REVERSED) on.Reverse();
    // find the native box face with the same outward normal
    for (int i = 1; i <= 6; ++i) {
      auto p = ref::refPlaneFromFace(faceOf(box, i));
      if (!p) continue;
      if (!vclose((*p)[3], (*p)[4], (*p)[5], on.X(), on.Y(), on.Z(), 1e-9)) continue;
      // origins must be coplanar: (o_native - o_occt)·n ≈ 0
      Standard_Real u0, u1, v0, v1; BRepTools::UVBounds(f, u0, u1, v0, v1);
      const gp_Pnt oo = surf.Value(0.5 * (u0 + u1), 0.5 * (v0 + v1));
      const double dx = (*p)[0] - oo.X(), dy = (*p)[1] - oo.Y(), dz = (*p)[2] - oo.Z();
      const double resid = dx * on.X() + dy * on.Y() + dz * on.Z();
      if (std::fabs(resid) > 1e-9) allOk = false;
      ++matched;
      break;
    }
  }
  report("box face datum planes vs gp_Pln (6/6)", allOk && matched == 6);
}

// ── faceAxis / refAxisFromFace vs gp_Cylinder axis ─────────────────────────────
static void checkCylinderAxis() {
  const double R = 1.5, H = 5.0;
  const Shape cyl = nativeCylinder(R, H);
  const TopoDS_Shape occt = BRepPrimAPI_MakeCylinder(R, H).Shape();
  gp_Ax1 oax; bool found = false;
  for (TopExp_Explorer ex(occt, TopAbs_FACE); ex.More(); ex.Next()) {
    BRepAdaptor_Surface surf(TopoDS::Face(ex.Current()));
    if (surf.GetType() == GeomAbs_Cylinder) { oax = surf.Cylinder().Axis(); found = true; break; }
  }
  auto na = ref::faceAxis(faceOf(cyl, 3));
  bool ok = found && na.has_value();
  if (ok) {
    const gp_Dir od = oax.Direction();
    ok = std::fabs(std::fabs((*na)[3] * od.X() + (*na)[4] * od.Y() + (*na)[5] * od.Z()) - 1.0) <= 1e-9;
    // native origin lies on the OCCT axis: cross((o_n - o_occt), dir) ≈ 0
    const gp_Pnt op = oax.Location();
    const double wx = (*na)[0] - op.X(), wy = (*na)[1] - op.Y(), wz = (*na)[2] - op.Z();
    const double cx = wy * od.Z() - wz * od.Y(), cy = wz * od.X() - wx * od.Z(),
                 cz = wx * od.Y() - wy * od.X();
    ok = ok && std::sqrt(cx * cx + cy * cy + cz * cz) <= 1e-7;
  }
  report("cylinder face axis vs gp_Cylinder::Axis", ok);
  auto na2 = ref::refAxisFromFace(faceOf(cyl, 3));
  report("refAxisFromFace == faceAxis (cyl)", na && na2 && *na == *na2);
}

// ── refAxisFromEdge vs gp_Lin (box edges, matched by midpoint) ─────────────────
static void checkEdgeAxes() {
  const Shape box = nativeBox(2, 3, 4);
  const topo::ShapeMap emap = topo::mapShapes(box, topo::ShapeType::Edge);
  const TopoDS_Shape occt = BRepPrimAPI_MakeBox(2, 3, 4).Shape();
  TopTools_IndexedMapOfShape oem;  // dedup shared edges (TopExp visits each twice)
  TopExp::MapShapes(occt, TopAbs_EDGE, oem);
  int checked = 0; bool allOk = true;
  for (int oi = 1; oi <= oem.Extent(); ++oi) {
    BRepAdaptor_Curve cu(TopoDS::Edge(oem.FindKey(oi)));
    if (cu.GetType() != GeomAbs_Line) continue;
    const gp_Lin ol = cu.Line();
    const gp_Pnt mid = cu.Value(0.5 * (cu.FirstParameter() + cu.LastParameter()));
    const gp_Dir od = ol.Direction();
    // find a native line edge whose axis direction is parallel AND whose line
    // passes through the OCCT edge midpoint.
    bool oneOk = false;
    for (std::size_t i = 1; i <= emap.size(); ++i) {
      auto a = ref::refAxisFromEdge(emap.shape(static_cast<int>(i)));
      if (!a) continue;
      if (std::fabs(std::fabs((*a)[3] * od.X() + (*a)[4] * od.Y() + (*a)[5] * od.Z()) - 1.0) > 1e-9)
        continue;
      const double wx = mid.X() - (*a)[0], wy = mid.Y() - (*a)[1], wz = mid.Z() - (*a)[2];
      const double cx = wy * (*a)[5] - wz * (*a)[4], cy = wz * (*a)[3] - wx * (*a)[5],
                   cz = wx * (*a)[4] - wy * (*a)[3];
      if (std::sqrt(cx * cx + cy * cy + cz * cz) <= 1e-7) { oneOk = true; break; }
    }
    if (!oneOk) allOk = false;
    ++checked;
  }
  report("box edge axes vs gp_Lin (12/12)", allOk && checked == 12);
}

// ── outerRimChain vs BRepTools::OuterWire (loop vertex set) ─────────────────────
static void checkOuterRim() {
  // Native welded quad cap [0,4]² and the OCCT equivalent planar face.
  Shape face = weldedQuad({0, 0, 0}, {4, 0, 0}, {4, 4, 0}, {0, 4, 0}, Dir3{0, 0, 1});
  auto rim = ref::outerRimChain(face, {1});
  // native rim edge midpoints
  const topo::ShapeMap emap = topo::mapShapes(face, topo::ShapeType::Edge);
  std::vector<std::array<double, 3>> nmid;
  for (int id : rim) {
    std::vector<Point3> vp;
    for (topo::Explorer vex(emap.shape(id), topo::ShapeType::Vertex); vex.more(); vex.next())
      vp.push_back(*topo::pointOf(vex.current()));
    if (vp.size() == 2)
      nmid.push_back({0.5 * (vp[0].x + vp[1].x), 0.5 * (vp[0].y + vp[1].y),
                      0.5 * (vp[0].z + vp[1].z)});
  }
  // OCCT outer wire of the same rectangle face
  BRepBuilderAPI_MakePolygon poly(gp_Pnt(0, 0, 0), gp_Pnt(4, 0, 0), gp_Pnt(4, 4, 0),
                                  gp_Pnt(0, 4, 0), Standard_True);
  TopoDS_Face of = BRepBuilderAPI_MakeFace(poly.Wire()).Face();
  const TopoDS_Wire ow = BRepTools::OuterWire(of);
  int occtN = 0; bool allMatched = true;
  for (BRepTools_WireExplorer we(ow); we.More(); we.Next()) {
    BRepAdaptor_Curve cu(we.Current());
    const gp_Pnt m = cu.Value(0.5 * (cu.FirstParameter() + cu.LastParameter()));
    bool m1 = false;
    for (auto& nmp : nmid) if (vclose(nmp[0], nmp[1], nmp[2], m.X(), m.Y(), m.Z())) m1 = true;
    if (!m1) allMatched = false;
    ++occtN;
  }
  report("outer rim edges vs BRepTools::OuterWire", allMatched && (int)nmid.size() == occtN && occtN == 4);
}

// ── offsetFaceBoundary vs BRepOffsetAPI_MakeOffset (inward rectangle) ──────────
static void polyAreaBBox(const std::vector<double>& xyz, double& area, double bb[4]) {
  const int n = (int)xyz.size() / 3;
  area = 0.0; bb[0] = bb[2] = 1e300; bb[1] = bb[3] = -1e300;
  for (int i = 0; i < n; ++i) {
    const double x = xyz[3 * i], y = xyz[3 * i + 1];
    const int j = (i + 1) % n;
    area += x * xyz[3 * j + 1] - xyz[3 * j] * y;
    bb[0] = std::min(bb[0], x); bb[1] = std::max(bb[1], x);
    bb[2] = std::min(bb[2], y); bb[3] = std::max(bb[3], y);
  }
  area = std::fabs(area) * 0.5;
}
static void checkOffset() {
  // Native rectangle [0,10]×[0,6] offset inward by 1 → [1,9]×[1,5].
  Shape face = weldedQuad({0, 0, 0}, {10, 0, 0}, {10, 6, 0}, {0, 6, 0}, Dir3{0, 0, 1});
  auto npts = ref::offsetFaceBoundary(face, 1.0);
  if (!npts) { report("offset boundary vs MakeOffset (inward rect)", false); return; }
  double narea, nbb[4]; polyAreaBBox(*npts, narea, nbb);

  // OCCT: offset the same rectangle wire; pick the INWARD (smaller-area) result.
  BRepBuilderAPI_MakePolygon poly(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0), gp_Pnt(10, 6, 0),
                                  gp_Pnt(0, 6, 0), Standard_True);
  const TopoDS_Wire w = poly.Wire();
  double best = 1e300; std::vector<double> obest;
  for (double d : {1.0, -1.0}) {
    BRepOffsetAPI_MakeOffset mko(w, GeomAbs_Arc);
    mko.Perform(d);
    if (!mko.IsDone()) continue;
    const TopoDS_Shape res = mko.Shape();
    TopoDS_Wire ow;
    if (res.ShapeType() == TopAbs_WIRE) ow = TopoDS::Wire(res);
    else { TopExp_Explorer e(res, TopAbs_WIRE); if (e.More()) ow = TopoDS::Wire(e.Current()); }
    if (ow.IsNull()) continue;
    std::vector<double> pts;
    for (BRepTools_WireExplorer we(ow); we.More(); we.Next()) {
      const gp_Pnt p = BRepAdaptor_Curve(we.Current()).Value(
          BRepAdaptor_Curve(we.Current()).FirstParameter());
      pts.push_back(p.X()); pts.push_back(p.Y()); pts.push_back(p.Z());
    }
    if (pts.size() < 9) continue;
    double a, bb[4]; polyAreaBBox(pts, a, bb);
    if (a < best) { best = a; obest = pts; }
  }
  double oarea, obb[4]; polyAreaBBox(obest, oarea, obb);
  const bool ok = std::fabs(narea - oarea) <= 1e-6 && std::fabs(narea - 32.0) <= 1e-6 &&
                  std::fabs(nbb[0] - obb[0]) <= 1e-7 && std::fabs(nbb[1] - obb[1]) <= 1e-7 &&
                  std::fabs(nbb[2] - obb[2]) <= 1e-7 && std::fabs(nbb[3] - obb[3]) <= 1e-7;
  std::printf("       native area=%.9g bbox[%.6g,%.6g,%.6g,%.6g]  occt area=%.9g bbox[%.6g,%.6g,%.6g,%.6g]\n",
              narea, nbb[0], nbb[1], nbb[2], nbb[3], oarea, obb[0], obb[1], obb[2], obb[3]);
  report("offset boundary vs MakeOffset (inward rect)", ok);
}

// ── tangentChain vs OCCT D1 tangent oracle ─────────────────────────────────────
static double occtEdgeTangentDot(const TopoDS_Edge& e1, const TopoDS_Edge& e2, const gp_Pnt& at) {
  auto tan = [&](const TopoDS_Edge& e) {
    BRepAdaptor_Curve c(e);
    // parameter nearest `at` among the endpoints
    const double t0 = c.FirstParameter(), t1 = c.LastParameter();
    gp_Pnt p0 = c.Value(t0), p1 = c.Value(t1);
    const double t = (p0.Distance(at) <= p1.Distance(at)) ? t0 : t1;
    gp_Pnt p; gp_Vec d; c.D1(t, p, d);
    return gp_Dir(d);
  };
  const gp_Dir a = tan(e1), b = tan(e2);
  return std::fabs(a.Dot(b));
}
static void checkTangent() {
  // Collinear pair sharing (1,0,0): native grows; OCCT D1 confirms |t·t|≈1 ≥ .966.
  {
    Shape v0 = ShapeBuilder::makeVertex({0, 0, 0}), v1 = ShapeBuilder::makeVertex({1, 0, 0}),
          v2 = ShapeBuilder::makeVertex({2, 0, 0});
    Shape wire = ShapeBuilder::makeWire({lineEdge(v0, v1), lineEdge(v1, v2)});
    auto ch = ref::tangentChain(wire, {1});
    TopoDS_Edge oe1 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(1, 0, 0)).Edge();
    TopoDS_Edge oe2 = BRepBuilderAPI_MakeEdge(gp_Pnt(1, 0, 0), gp_Pnt(2, 0, 0)).Edge();
    const double d = occtEdgeTangentDot(oe1, oe2, gp_Pnt(1, 0, 0));
    report("tangent grows ⇔ OCCT C1 (collinear)", ch && ch->size() == 2 && d >= 0.966);
  }
  // Right-angle pair: native stops; OCCT D1 confirms |t·t|≈0 < .966.
  {
    Shape v0 = ShapeBuilder::makeVertex({0, 0, 0}), v1 = ShapeBuilder::makeVertex({1, 0, 0}),
          v2 = ShapeBuilder::makeVertex({1, 1, 0});
    Shape wire = ShapeBuilder::makeWire({lineEdge(v0, v1), lineEdge(v1, v2)});
    auto ch = ref::tangentChain(wire, {1});
    TopoDS_Edge oe1 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(1, 0, 0)).Edge();
    TopoDS_Edge oe2 = BRepBuilderAPI_MakeEdge(gp_Pnt(1, 0, 0), gp_Pnt(1, 1, 0)).Edge();
    const double d = occtEdgeTangentDot(oe1, oe2, gp_Pnt(1, 0, 0));
    report("tangent stops ⇔ OCCT non-C1 (corner)", ch && ch->size() == 1 && d < 0.966);
  }
}

int main() {
  std::printf("== MOAT M-REF native-vs-OCCT reference/topology parity (sim gate b) ==\n");
  std::fflush(stdout);
  checkBoxPlanes();
  checkCylinderAxis();
  checkEdgeAxes();
  checkOuterRim();
  checkOffset();
  checkTangent();
  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
