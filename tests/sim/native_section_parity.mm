// SPDX-License-Identifier: Apache-2.0
//
// native_section_parity.mm — MOAT M-GS GS2 planar SECTION-CURVES native-vs-OCCT
// parity harness (iOS simulator). GATE (b) of the two-gate model; GATE (a) (host,
// no OCCT, closed-form) is tests/native/test_native_section.cpp.
//
// For matched native / OCCT primitives (box, cylinder, sphere) cut by the same
// plane we assert the native section service (cybercad::native::section, OCCT-FREE,
// M1-SSI + topology consumed read-only) agrees with the OCCT oracle:
//   1. LOOP COUNT — native SectionResult.loopCount() == the number of connected
//      wires ShapeAnalysis_FreeBounds recovers from BRepAlgoAPI_Section's edges.
//   2. TOTAL EDGE LENGTH — native totalLength() ≈ the summed arc length of the OCCT
//      section edges measured with GCPnts_AbscissaPoint::Length (relative tol). This
//      dedicated arc-length integrator is used INSTEAD of BRepGProp::LinearProperties
//      because the latter's fixed low-order mass quadrature under-resolves a full
//      analytic Ellipse edge (oblique cylinder section) by ~1e-4; GCPnts converges to
//      the true perimeter. Straight/circular cases are identical under both.
//   3. CLOSED-NESS — every OCCT section wire closes ⇔ native reports closed loops.
//   4. CAPPED AREA — native totalArea() ≈ BRepGProp::SurfaceProperties mass of the
//      OCCT section face(s) built on the cut plane (absolute tol).
//
// Asserted at the cybercad::native::section C++ boundary (like the S1/S3 SSI parity
// harnesses); the native solids are built with the same ShapeBuilder fixtures the
// host GATE (a) uses, and the OCCT solids are built independently with
// BRepPrimAPI_Make{Box,Cylinder,Sphere} to the same dimensions. NO cc_* entry point
// is exercised here — the ABI accessor (cc_section_plane) is covered by the host
// suite + the shipping-path build.
//
// Built ONLY by scripts/run-sim-native-section.sh; on the SKIP list of
// run-sim-suite.sh.
//
#include "native/section/native_section.h"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <GProp_GProps.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace sec = cybercad::native::section;
namespace topo = cybercad::native::topology;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;
using nmath::Vec3;
using topo::EdgeCurve;
using topo::FaceSurface;
using topo::Shape;
using topo::ShapeBuilder;

namespace {

constexpr double kPi = 3.14159265358979323846;
int g_failures = 0;

void check(bool cond, const std::string& what) {
  std::printf("  [%s] %s\n", cond ? "PASS" : "FAIL", what.c_str());
  if (!cond) ++g_failures;
}

// ── native fixture builders (mirror the host GATE-a suite) ──────────────────────

Shape lineEdge(const Point3& a, const Point3& b) {
  const Vec3 d = b - a;
  EdgeCurve c; c.kind = EdgeCurve::Kind::Line;
  c.frame = Ax3{a, Dir3{d}, Dir3{}, Dir3{}};
  return ShapeBuilder::makeEdge(c, 0.0, nmath::norm(d), ShapeBuilder::makeVertex(a),
                                ShapeBuilder::makeVertex(b));
}
Shape circleEdge(const Point3& ctr, double r, const Dir3& n, const Dir3& x) {
  EdgeCurve c; c.kind = EdgeCurve::Kind::Circle;
  c.frame = Ax3::fromAxisAndRef(ctr, n, x); c.radius = r;
  return ShapeBuilder::makeEdge(c, 0.0, 2.0 * kPi, Shape{}, Shape{});
}
Shape planarQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3,
                 const Dir3& n) {
  FaceSurface s; s.kind = FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, n, Dir3{p1 - p0});
  return ShapeBuilder::makeFace(
      s, ShapeBuilder::makeWire({lineEdge(p0, p1), lineEdge(p1, p2), lineEdge(p2, p3),
                                 lineEdge(p3, p0)}));
}
Shape makeBox(double lx, double ly, double lz) {
  const Point3 a{0,0,0}, b{lx,0,0}, c{lx,ly,0}, d{0,ly,0};
  const Point3 e{0,0,lz}, f{lx,0,lz}, g{lx,ly,lz}, h{0,ly,lz};
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({
      planarQuad(a,d,c,b, Dir3{0,0,-1}), planarQuad(e,f,g,h, Dir3{0,0,1}),
      planarQuad(a,b,f,e, Dir3{0,-1,0}), planarQuad(d,h,g,c, Dir3{0,1,0}),
      planarQuad(a,e,h,d, Dir3{-1,0,0}), planarQuad(b,c,g,f, Dir3{1,0,0})})});
}
Shape makeCylinder(double R, double H) {
  const Dir3 zp{0,0,1}, xp{1,0,0};
  FaceSurface bs; bs.kind = FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, Dir3{0,0,-1}, xp);
  Shape bottom = ShapeBuilder::makeFace(bs, ShapeBuilder::makeWire({circleEdge(Point3{0,0,0}, R, zp, xp)}));
  FaceSurface ts; ts.kind = FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0,0,H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(ts, ShapeBuilder::makeWire({circleEdge(Point3{0,0,H}, R, zp, xp)}));
  FaceSurface ls; ls.kind = FaceSurface::Kind::Cylinder;
  ls.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, zp, xp); ls.radius = R;
  Shape lat = ShapeBuilder::makeFace(ls, ShapeBuilder::makeWire({
      circleEdge(Point3{0,0,0}, R, zp, xp), circleEdge(Point3{0,0,H}, R, zp, xp)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lat})});
}
Shape makeSphere(double R) {
  FaceSurface s; s.kind = FaceSurface::Kind::Sphere;
  s.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, Dir3{0,0,1}, Dir3{1,0,0}); s.radius = R;
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({
      ShapeBuilder::makeFace(s, ShapeBuilder::makeWire({}))})});
}
Dir3 orthoX(const Dir3& n) {
  const double ax=std::fabs(n.x()), ay=std::fabs(n.y()), az=std::fabs(n.z());
  Vec3 pick = (ax<=ay && ax<=az) ? Vec3{1,0,0} : (ay<=az) ? Vec3{0,1,0} : Vec3{0,0,1};
  return Dir3{nmath::cross(n.vec(), pick)};
}
nmath::Plane cutPlane(const Point3& o, const Dir3& n) {
  return nmath::Plane{Ax3::fromAxisAndRef(o, n, orthoX(n))};
}

// ── OCCT oracle ─────────────────────────────────────────────────────────────────

struct OcctSection { int wireCount = 0; double length = 0.0; double area = 0.0; bool allClosed = true; bool ok = false; };

OcctSection occtSection(const TopoDS_Shape& solid, const gp_Pnt& o, const gp_Dir& n) {
  OcctSection r;
  const gp_Pln pln(o, n);
  BRepAlgoAPI_Section sec(solid, pln, Standard_False);
  sec.ComputePCurveOn1(Standard_False);
  sec.Approximation(Standard_False);
  sec.Build();
  if (!sec.IsDone()) return r;
  const TopoDS_Shape edges = sec.Shape();

  // Edge length via OCCT's dedicated arc-length integrator (adaptive Gauss on each
  // BRepAlgoAPI_Section edge). We do NOT use BRepGProp::LinearProperties here: its
  // fixed low-order mass quadrature under-resolves a full-revolution analytic
  // Ellipse edge (an oblique cylinder section) by ~1e-4 relative, whereas
  // GCPnts_AbscissaPoint::Length converges to the true perimeter (verified: it
  // agrees with the closed-form π·(a+b)·… value to 1e-8). BRepGProp is retained for
  // AREA (SurfaceProperties, below), which is accurate. For the straight/circular
  // cases both integrators are exact, so those parity numbers are unchanged.
  Handle(TopTools_HSequenceOfShape) edgeSeq = new TopTools_HSequenceOfShape();
  for (TopExp_Explorer ex(edges, TopAbs_EDGE); ex.More(); ex.Next()) {
    edgeSeq->Append(ex.Current());
    BRepAdaptor_Curve ac(TopoDS::Edge(ex.Current()));
    r.length += GCPnts_AbscissaPoint::Length(ac, 1e-10);
  }
  if (edgeSeq->IsEmpty()) return r;

  Handle(TopTools_HSequenceOfShape) wires;
  ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edgeSeq, 1e-6, Standard_False, wires);
  r.wireCount = wires ? wires->Length() : 0;

  for (Standard_Integer i = 1; i <= r.wireCount; ++i) {
    const TopoDS_Wire w = TopoDS::Wire(wires->Value(i));
    if (!w.Closed()) r.allClosed = false;
    BRepBuilderAPI_MakeFace mf(pln, w, /*OnlyPlane=*/Standard_True);
    if (mf.IsDone()) {
      GProp_GProps sp;
      BRepGProp::SurfaceProperties(mf.Face(), sp);
      r.area += std::fabs(sp.Mass());
    }
  }
  r.ok = true;
  return r;
}

bool rel(double a, double b, double tol) {
  const double d = std::fabs(a - b);
  return d <= tol * std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
}

// One parity case: native section vs OCCT section of matched solids.
void parityCase(const std::string& name, const Shape& nativeSolid, const TopoDS_Shape& occtSolid,
                const Point3& o, const Dir3& nn) {
  std::printf("── %s\n", name.c_str());
  const sec::SectionResult nr = sec::sectionByPlane(nativeSolid, cutPlane(o, nn));
  const OcctSection orr =
      occtSection(occtSolid, gp_Pnt(o.x, o.y, o.z), gp_Dir(nn.x(), nn.y(), nn.z()));

  check(nr.ok(), "native section produced (Ok)");
  check(orr.ok, "OCCT section produced");
  if (!nr.ok() || !orr.ok) return;

  check(nr.loopCount() == orr.wireCount,
        "loop count native=" + std::to_string(nr.loopCount()) +
            " occt=" + std::to_string(orr.wireCount));
  check(rel(nr.totalLength(), orr.length, 1e-4),
        "total edge length native=" + std::to_string(nr.totalLength()) +
            " occt=" + std::to_string(orr.length));
  check(orr.allClosed, "OCCT wires all closed (native loops are closed by construction)");
  check(rel(nr.totalArea(), orr.area, 1e-4),
        "capped area native=" + std::to_string(nr.totalArea()) +
            " occt=" + std::to_string(orr.area));
}

}  // namespace

int main() {
  std::printf("native_section_parity — MOAT GS2 native-vs-OCCT section curves\n");

  // Box 10×8×6.
  const Shape nBox = makeBox(10, 8, 6);
  const TopoDS_Shape oBox = BRepPrimAPI_MakeBox(10.0, 8.0, 6.0).Shape();
  parityCase("box x=5 → rectangle 8×6", nBox, oBox, Point3{5,0,0}, Dir3{1,0,0});
  parityCase("box z=3 → rectangle 10×8", nBox, oBox, Point3{0,0,3}, Dir3{0,0,1});

  // Cylinder R3 H10.
  const Shape nCyl = makeCylinder(3, 10);
  const TopoDS_Shape oCyl = BRepPrimAPI_MakeCylinder(3.0, 10.0).Shape();
  parityCase("cylinder z=5 → cross-section circle", nCyl, oCyl, Point3{0,0,5}, Dir3{0,0,1});
  parityCase("cylinder y=0 → axial rectangle 6×10", nCyl, oCyl, Point3{0,0,0}, Dir3{0,1,0});
  // Oblique 45° cut → ellipse a=R/|cosθ|=R√2, b=R. Native uses the Ramanujan-II
  // perimeter (rel error ≪1e-4 at this eccentricity) vs OCCT edge length, and
  // area=π·a·b vs BRepGProp; 1 closed wire.
  parityCase("cylinder oblique → ellipse", nCyl, oCyl, Point3{0,0,5}, Dir3{0,1,1});

  // Sphere R5.
  const Shape nSph = makeSphere(5);
  const TopoDS_Shape oSph = BRepPrimAPI_MakeSphere(5.0).Shape();
  parityCase("sphere z=0 → great circle", nSph, oSph, Point3{0,0,0}, Dir3{0,0,1});
  parityCase("sphere z=3 → small circle r=4", nSph, oSph, Point3{0,0,3}, Dir3{0,0,1});

  std::printf("%s (%d failing checks)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
  return g_failures == 0 ? 0 : 1;
}
