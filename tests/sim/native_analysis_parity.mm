// SPDX-License-Identifier: Apache-2.0
//
// native_analysis_parity.mm — MOAT M-GS GS3/GS4 native-vs-OCCT parity harness
// (iOS simulator). This is GATE (b) of the two-gate discipline; gate (a) is the
// OCCT-free host-analytic suite tests/native/test_native_analysis.cpp.
//
// The native ANALYSIS services (src/native/analysis/*.h, OCCT-FREE, header-only,
// built on the landed native NURBS/topology evaluators) are asserted against the
// OCCT ORACLE on IDENTICAL geometry built on both sides:
//   * GS4 surface curvature K, H, k1/k2  vs  GeomLProp_SLProps
//     (GaussianCurvature / MeanCurvature / Max/MinCurvature).
//   * GS4 edge curvature κ                vs  GeomLProp_CLProps (Curvature).
//   * GS3 minimum distance                vs  BRepExtrema_DistShapeShape (Value),
//     for point/edge/face TopoDS operands built from the same data.
//   * GS3 angle (line·line, plane·plane, line·plane) vs an independent gp_Dir
//     closed form.
//
// CURVATURE SIGN. Gaussian K is orientation-independent, so it is compared SIGNED.
// Mean H and the principals k1/k2 flip sign with the chart normal; the two sides
// choose it by their own parametrisation convention, so those are compared by
// MAGNITUDE (|H|, sorted |k|) — exactly as the host gate does. The facade layer is
// where the OCCT-face-normal sign is pinned (a Reversed-face flip), covered by the
// facade host test; this geometry-level oracle asserts the invariant quantities.
//
// HONEST DECLINE is not re-exercised here (it has no OCCT number to match); the
// host gate owns the nullopt cases. This harness only asserts the cases the
// service RETURNS — the native-vs-OCCT numeric agreement.
//
// OCCT-DEPENDENT + NumSci-DEPENDENT (the freeform distance minimizer rides
// numerics/closest_point). Compiled ONLY by scripts/run-sim-native-analysis.sh;
// carries its own main(); std::_Exit to skip the non-exit-clean OCCT static
// teardown of the trimmed static build (same rationale as native_numerics_parity).

#include "native/analysis/angle.h"
#include "native/analysis/curvature.h"
#include "native/analysis/distance.h"
#include "native/topology/shape.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_analysis_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_analysis_parity requires -DCYBERCAD_HAS_NUMSCI and the NumPP/SciPP substrate"
#endif

// ── OCCT oracle headers ──────────────────────────────────────────────────────
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pln.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BezierCurve.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GeomLProp_CLProps.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopoDS_Shape.hxx>

namespace an = cybercad::native::analysis;
namespace nm = cybercad::native::math;
namespace nt = cybercad::native::topology;

static int g_pass = 0, g_fail = 0;
static constexpr double kTolK = 1e-6;   // Gaussian / mean / principal
static constexpr double kTolD = 1e-6;   // distance (coords O(1..10))
static constexpr double kTolA = 1e-9;   // angle
static constexpr double kTwoPi = 6.283185307179586476925286766559;
static constexpr double kHalfPi = 1.5707963267948966192313216916398;

static nm::Ax3 stdFrame(const nm::Point3& o = {0, 0, 0}) {
  return nm::Ax3{o, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
}
static gp_Ax3 occtStdAx(const nm::Point3& o = {0, 0, 0}) {
  return gp_Ax3(gp_Pnt(o.x, o.y, o.z), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
}

static void report(const char* name, bool ok, double a, double b) {
  std::printf("[MGS] %-40s %s  native=%.10g occt=%.10g  d=%.3e\n", name,
              ok ? "PASS" : "FAIL", a, b, std::fabs(a - b));
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// ═════════════════════════════════════════════════════════════════════════════
// GS4 surface curvature vs GeomLProp_SLProps.
//   nativeK/H/|k| against occ.GaussianCurvature()/MeanCurvature()/Max|Min.
// ═════════════════════════════════════════════════════════════════════════════
static void checkSurface(const char* name, const std::optional<an::SurfaceCurvature>& nat,
                         const Handle(Geom_Surface)& surf, double u, double v) {
  if (!nat) { report(name, false, 0, 0); return; }
  GeomLProp_SLProps p(surf, u, v, 2, 1e-12);
  const bool defined = p.IsCurvatureDefined();
  const double oK = defined ? p.GaussianCurvature() : 0.0;
  const double oH = defined ? p.MeanCurvature() : 0.0;
  const double oMax = defined ? p.MaxCurvature() : 0.0;
  const double oMin = defined ? p.MinCurvature() : 0.0;
  const double nHi = std::max(std::fabs(nat->k1), std::fabs(nat->k2));
  const double nLo = std::min(std::fabs(nat->k1), std::fabs(nat->k2));
  const double oHi = std::max(std::fabs(oMax), std::fabs(oMin));
  const double oLo = std::min(std::fabs(oMax), std::fabs(oMin));
  const bool ok = defined && std::fabs(nat->K - oK) <= kTolK &&
                  std::fabs(std::fabs(nat->H) - std::fabs(oH)) <= kTolK &&
                  std::fabs(nHi - oHi) <= kTolK && std::fabs(nLo - oLo) <= kTolK;
  std::printf("[MGS] %-40s %s  K:%.8g/%.8g |H|:%.8g/%.8g\n", name, ok ? "PASS" : "FAIL",
              nat->K, oK, std::fabs(nat->H), std::fabs(oH));
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

static void groupSurfaceCurvature() {
  // Sphere R=2.
  {
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Sphere; s.frame = stdFrame(); s.radius = 2.0;
    Handle(Geom_SphericalSurface) g = new Geom_SphericalSurface(occtStdAx(), 2.0);
    checkSurface("sphere R=2 curvature", an::surfaceCurvature(s, 1.0, 0.4), g, 1.0, 0.4);
  }
  // Cylinder R=3.
  {
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Cylinder; s.frame = stdFrame(); s.radius = 3.0;
    Handle(Geom_CylindricalSurface) g = new Geom_CylindricalSurface(occtStdAx(), 3.0);
    checkSurface("cylinder R=3 curvature", an::surfaceCurvature(s, 0.7, 1.5), g, 0.7, 1.5);
  }
  // Cone semiAngle=π/6, ref radius 5, at v=0 (ρ=5).
  {
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Cone; s.frame = stdFrame();
    s.radius = 5.0; s.semiAngle = M_PI / 6.0;
    Handle(Geom_ConicalSurface) g = new Geom_ConicalSurface(occtStdAx(), M_PI / 6.0, 5.0);
    checkSurface("cone a=pi/6 R=5 v=0 curvature", an::surfaceCurvature(s, 0.9, 0.0), g, 0.9, 0.0);
  }
  // Torus R=5,r=2 at outer equator v=0 (K=1/14) and top v=π/2 (K=0).
  {
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Torus; s.frame = stdFrame();
    s.radius = 5.0; s.minorRadius = 2.0;
    Handle(Geom_ToroidalSurface) g = new Geom_ToroidalSurface(occtStdAx(), 5.0, 2.0);
    checkSurface("torus outer v=0 curvature", an::surfaceCurvature(s, 1.1, 0.0), g, 1.1, 0.0);
    checkSurface("torus top v=pi/2 curvature", an::surfaceCurvature(s, 0.3, kHalfPi), g, 0.3, kHalfPi);
  }
  // Non-rational B-spline surface (bicubic 4×4, gently undulating). Freeform path.
  {
    const int degU = 3, degV = 3, nU = 4, nV = 4;
    const double zTab[4][4] = {{0.0, 0.5, 0.5, 0.0}, {0.5, 1.5, 1.5, 0.5},
                               {0.5, 1.5, 1.5, 0.5}, {0.0, 0.5, 0.5, 0.0}};
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::BSpline;
    s.degreeU = degU; s.degreeV = degV; s.nPolesU = nU; s.nPolesV = nV;
    for (int i = 0; i < nU; ++i)
      for (int j = 0; j < nV; ++j) s.poles.push_back({double(i), double(j), zTab[i][j]});
    s.knotsU = {0, 0, 0, 0, 1, 1, 1, 1};
    s.knotsV = {0, 0, 0, 0, 1, 1, 1, 1};

    TColgp_Array2OfPnt op(1, nU, 1, nV);
    for (int i = 0; i < nU; ++i)
      for (int j = 0; j < nV; ++j)
        op.SetValue(i + 1, j + 1, gp_Pnt(double(i), double(j), zTab[i][j]));
    TColStd_Array1OfReal uk(1, 2), vk(1, 2); uk.SetValue(1, 0); uk.SetValue(2, 1);
    vk.SetValue(1, 0); vk.SetValue(2, 1);
    TColStd_Array1OfInteger um(1, 2), vm(1, 2);
    um.SetValue(1, 4); um.SetValue(2, 4); vm.SetValue(1, 4); vm.SetValue(2, 4);
    Handle(Geom_BSplineSurface) g =
        new Geom_BSplineSurface(op, uk, vk, um, vm, degU, degV, Standard_False, Standard_False);
    checkSurface("bspline bicubic curvature", an::surfaceCurvature(s, 0.35, 0.6), g, 0.35, 0.6);
    checkSurface("bspline bicubic curvature #2", an::surfaceCurvature(s, 0.7, 0.25), g, 0.7, 0.25);
  }
  // Rational (NURBS) quarter-cylinder R=3: a rational-quadratic arc extruded (deg1).
  {
    const double R = 3.0, w = std::sqrt(2.0) / 2.0;
    const nm::Point3 arc[3] = {{R, 0, 0}, {R, R, 0}, {0, R, 0}};
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::BSpline;
    s.degreeU = 2; s.degreeV = 1; s.nPolesU = 3; s.nPolesV = 2;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 2; ++j) s.poles.push_back({arc[i].x, arc[i].y, j == 0 ? 0.0 : 4.0});
    const double wa[3] = {1.0, w, 1.0};
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 2; ++j) s.weights.push_back(wa[i]);
    s.knotsU = {0, 0, 0, 1, 1, 1};
    s.knotsV = {0, 0, 1, 1};

    TColgp_Array2OfPnt op(1, 3, 1, 2);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 2; ++j)
        op.SetValue(i + 1, j + 1, gp_Pnt(arc[i].x, arc[i].y, j == 0 ? 0.0 : 4.0));
    TColStd_Array1OfReal uk(1, 2), vk(1, 2); uk.SetValue(1, 0); uk.SetValue(2, 1);
    vk.SetValue(1, 0); vk.SetValue(2, 1);
    TColStd_Array1OfInteger um(1, 2), vm(1, 2);
    um.SetValue(1, 3); um.SetValue(2, 3); vm.SetValue(1, 2); vm.SetValue(2, 2);
    // Weights array (row=U index, col=V index).
    TColStd_Array2OfReal W(1, 3, 1, 2);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 2; ++j) W.SetValue(i + 1, j + 1, wa[i]);
    Handle(Geom_BSplineSurface) g =
        new Geom_BSplineSurface(op, W, uk, vk, um, vm, 2, 1, Standard_False, Standard_False);
    checkSurface("nurbs quarter-cyl R=3 curvature", an::surfaceCurvature(s, 0.5, 0.5), g, 0.5, 0.5);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// GS4 edge curvature vs GeomLProp_CLProps.
// ═════════════════════════════════════════════════════════════════════════════
static void checkCurve(const char* name, const std::optional<double>& nat,
                       const Handle(Geom_Curve)& crv, double t) {
  if (!nat) { report(name, false, 0, 0); return; }
  GeomLProp_CLProps p(crv, t, 2, 1e-12);
  const double oK = p.IsTangentDefined() ? p.Curvature() : -1.0;
  report(name, std::fabs(*nat - oK) <= kTolK, *nat, oK);
}

static void groupEdgeCurvature() {
  // Circle R=4.
  {
    nt::EdgeCurve c; c.kind = nt::EdgeCurve::Kind::Circle; c.frame = stdFrame(); c.radius = 4.0;
    Handle(Geom_Circle) g = new Geom_Circle(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 4.0);
    checkCurve("circle R=4 kappa", an::edgeCurvature(c, 1.0), g, 1.0);
  }
  // Ellipse a=3,b=2 at t=0.7.
  {
    nt::EdgeCurve c; c.kind = nt::EdgeCurve::Kind::Ellipse; c.frame = stdFrame();
    c.radius = 3.0; c.minorRadius = 2.0;
    Handle(Geom_Ellipse) g = new Geom_Ellipse(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)), 3.0, 2.0);
    checkCurve("ellipse a=3 b=2 kappa", an::edgeCurvature(c, 0.7), g, 0.7);
  }
  // Non-rational cubic B-spline curve (5 poles).
  {
    const int degree = 3;
    std::vector<nm::Point3> poles = {{0, 0, 0}, {1, 2, 0}, {2, -1, 1}, {3, 2, 0}, {4, 0, 1}};
    const std::vector<double> flat = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
    nt::EdgeCurve c; c.kind = nt::EdgeCurve::Kind::BSpline; c.degree = degree;
    c.poles = poles; c.knots = flat;

    TColgp_Array1OfPnt op(1, 5);
    for (int i = 0; i < 5; ++i) op.SetValue(i + 1, gp_Pnt(poles[i].x, poles[i].y, poles[i].z));
    TColStd_Array1OfReal kn(1, 3); kn.SetValue(1, 0); kn.SetValue(2, 0.5); kn.SetValue(3, 1);
    TColStd_Array1OfInteger mu(1, 3); mu.SetValue(1, 4); mu.SetValue(2, 1); mu.SetValue(3, 4);
    Handle(Geom_BSplineCurve) g = new Geom_BSplineCurve(op, kn, mu, degree, Standard_False);
    checkCurve("bspline cubic kappa", an::edgeCurvature(c, 0.42), g, 0.42);
  }
  // Non-rational degree-2 Bézier parabola.
  {
    nt::EdgeCurve c; c.kind = nt::EdgeCurve::Kind::Bezier; c.degree = 2;
    c.poles = {{0, 0, 0}, {0.5, 0, 0}, {1, 1, 0}};
    TColgp_Array1OfPnt op(1, 3);
    op.SetValue(1, gp_Pnt(0, 0, 0)); op.SetValue(2, gp_Pnt(0.5, 0, 0)); op.SetValue(3, gp_Pnt(1, 1, 0));
    Handle(Geom_BezierCurve) g = new Geom_BezierCurve(op);
    checkCurve("bezier parabola kappa", an::edgeCurvature(c, 0.5), g, 0.5);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// GS3 minimum distance vs BRepExtrema_DistShapeShape.
// ═════════════════════════════════════════════════════════════════════════════
static void checkDist(const char* name, const std::optional<an::DistanceResult>& nat,
                      const TopoDS_Shape& s1, const TopoDS_Shape& s2) {
  if (!nat) { report(name, false, 0, 0); return; }
  BRepExtrema_DistShapeShape ext(s1, s2);
  const bool ok = ext.IsDone() && std::fabs(nat->distance - ext.Value()) <= kTolD;
  report(name, ok, nat->distance, ext.IsDone() ? ext.Value() : -1.0);
}

static void groupDistance() {
  // vertex · vertex.
  {
    an::Entity a = an::Entity::ofVertex({1, 2, 3}), b = an::Entity::ofVertex({4, 6, 3});
    TopoDS_Shape v1 = BRepBuilderAPI_MakeVertex(gp_Pnt(1, 2, 3));
    TopoDS_Shape v2 = BRepBuilderAPI_MakeVertex(gp_Pnt(4, 6, 3));  // d=5
    checkDist("vertex.vertex d=5", an::minDistance(a, b), v1, v2);
  }
  // vertex · line-edge (segment [0,0,0]-[2,0,0], point (5,1,0) → clamp → √10).
  {
    nt::EdgeCurve line; line.kind = nt::EdgeCurve::Kind::Line;
    line.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
    an::Entity a = an::Entity::ofVertex({5, 1, 0});
    an::Entity b = an::Entity::ofEdge(line, 0.0, 2.0);
    TopoDS_Shape v = BRepBuilderAPI_MakeVertex(gp_Pnt(5, 1, 0));
    TopoDS_Shape e = BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(2, 0, 0));
    checkDist("vertex.lineEdge clamp", an::minDistance(a, b), v, e);
  }
  // vertex · plane-face (bounded window), foot inside → orthogonal distance.
  {
    nt::FaceSurface plane; plane.kind = nt::FaceSurface::Kind::Plane; plane.frame = stdFrame();
    an::Entity a = an::Entity::ofVertex({1, 2, 7});
    an::Entity b = an::Entity::ofFace(plane, -10, 10, -10, 10);
    TopoDS_Shape v = BRepBuilderAPI_MakeVertex(gp_Pnt(1, 2, 7));
    Handle(Geom_Plane) gpl = new Geom_Plane(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
    TopoDS_Shape f = BRepBuilderAPI_MakeFace(gpl, -10.0, 10.0, -10.0, 10.0, 1e-9);
    checkDist("vertex.planeFace d=7", an::minDistance(a, b), v, f);
  }
  // vertex · sphere-face R=2, point (10,0,0) → d=8.
  {
    nt::FaceSurface sph; sph.kind = nt::FaceSurface::Kind::Sphere; sph.frame = stdFrame(); sph.radius = 2.0;
    an::Entity a = an::Entity::ofVertex({10, 0, 0});
    an::Entity b = an::Entity::ofFace(sph, 0, kTwoPi, -kHalfPi, kHalfPi);
    TopoDS_Shape v = BRepBuilderAPI_MakeVertex(gp_Pnt(10, 0, 0));
    Handle(Geom_SphericalSurface) gsp = new Geom_SphericalSurface(occtStdAx(), 2.0);
    TopoDS_Shape f = BRepBuilderAPI_MakeFace(gsp, 1e-9);
    checkDist("vertex.sphereFace d=8", an::minDistance(a, b), v, f);
  }
  // edge(line) · edge(line) skew: x-axis seg & y-axis seg at z=3 → d=3.
  {
    nt::EdgeCurve lx; lx.kind = nt::EdgeCurve::Kind::Line;
    lx.frame = nm::Ax3{{-2, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
    nt::EdgeCurve ly; ly.kind = nt::EdgeCurve::Kind::Line;
    ly.frame = nm::Ax3{{0, -2, 3}, nm::Dir3{0, 1, 0}, nm::Dir3{-1, 0, 0}, nm::Dir3{0, 0, 1}};
    an::Entity a = an::Entity::ofEdge(lx, 0, 4), b = an::Entity::ofEdge(ly, 0, 4);
    TopoDS_Shape e1 = BRepBuilderAPI_MakeEdge(gp_Pnt(-2, 0, 0), gp_Pnt(2, 0, 0));
    TopoDS_Shape e2 = BRepBuilderAPI_MakeEdge(gp_Pnt(0, -2, 3), gp_Pnt(0, 2, 3));
    checkDist("edge.edge skew d=3", an::minDistance(a, b), e1, e2);
  }
  // freeform edge · edge: two well-separated non-rational quadratic Béziers (z-gap 5).
  {
    nt::EdgeCurve a; a.kind = nt::EdgeCurve::Kind::Bezier; a.degree = 2;
    a.poles = {{-1, 0, 0}, {0, 1, 0}, {1, 0, 0}};
    nt::EdgeCurve b; b.kind = nt::EdgeCurve::Kind::Bezier; b.degree = 2;
    b.poles = {{-1, 0, 5}, {0, -1, 5}, {1, 0, 5}};
    TColgp_Array1OfPnt pa(1, 3), pb(1, 3);
    pa.SetValue(1, gp_Pnt(-1, 0, 0)); pa.SetValue(2, gp_Pnt(0, 1, 0)); pa.SetValue(3, gp_Pnt(1, 0, 0));
    pb.SetValue(1, gp_Pnt(-1, 0, 5)); pb.SetValue(2, gp_Pnt(0, -1, 5)); pb.SetValue(3, gp_Pnt(1, 0, 5));
    Handle(Geom_BezierCurve) ca = new Geom_BezierCurve(pa), cb = new Geom_BezierCurve(pb);
    TopoDS_Shape e1 = BRepBuilderAPI_MakeEdge(ca);
    TopoDS_Shape e2 = BRepBuilderAPI_MakeEdge(cb);
    checkDist("freeform bezier edge.edge", an::minDistance(an::Entity::ofEdge(a, 0, 1),
                                                           an::Entity::ofEdge(b, 0, 1)), e1, e2);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// GS3 angle vs an independent gp_Dir closed form.
// ═════════════════════════════════════════════════════════════════════════════
static void groupAngle() {
  // line·line at 45°.
  {
    nt::EdgeCurve lx; lx.kind = nt::EdgeCurve::Kind::Line;
    lx.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
    nt::EdgeCurve l45; l45.kind = nt::EdgeCurve::Kind::Line;
    l45.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 1, 0}, nm::Dir3{-1, 1, 0}, nm::Dir3{0, 0, 1}};
    auto nat = an::angle(an::Entity::ofEdge(lx, 0, 1), an::Entity::ofEdge(l45, 0, 1));
    gp_Dir d1(1, 0, 0), d2(1, 1, 0);
    double occ = d1.Angle(d2); occ = std::min(occ, M_PI - occ);  // acute
    report("angle line.line 45", nat && std::fabs(*nat - occ) <= kTolA, nat ? *nat : -1, occ);
  }
  // plane·plane at 90° (oriented normals).
  {
    nt::FaceSurface pz; pz.kind = nt::FaceSurface::Kind::Plane;
    pz.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
    nt::FaceSurface px; px.kind = nt::FaceSurface::Kind::Plane;
    px.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}, nm::Dir3{1, 0, 0}};
    auto nat = an::angle(an::Entity::ofFace(pz, 0, 1, 0, 1), an::Entity::ofFace(px, 0, 1, 0, 1));
    gp_Dir n1(0, 0, 1), n2(1, 0, 0);
    double occ = n1.Angle(n2);
    report("angle plane.plane 90", nat && std::fabs(*nat - occ) <= kTolA, nat ? *nat : -1, occ);
  }
  // line·plane 90° (line along z vs plane z=0).
  {
    nt::FaceSurface pz; pz.kind = nt::FaceSurface::Kind::Plane;
    pz.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
    nt::EdgeCurve lz; lz.kind = nt::EdgeCurve::Kind::Line;
    lz.frame = nm::Ax3{{0, 0, 0}, nm::Dir3{0, 0, 1}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}};
    auto nat = an::angle(an::Entity::ofEdge(lz, 0, 1), an::Entity::ofFace(pz, 0, 1, 0, 1));
    gp_Dir d(0, 0, 1), n(0, 0, 1);
    double occ = std::asin(std::min(1.0, std::fabs(d.Dot(n))));
    report("angle line.plane 90", nat && std::fabs(*nat - occ) <= kTolA, nat ? *nat : -1, occ);
  }
}

int main() {
  std::printf("== MOAT M-GS GS3/GS4 native-vs-OCCT parity (sim gate b) ==\n");
  std::fflush(stdout);
  groupSurfaceCurvature();
  groupEdgeCurvature();
  groupDistance();
  groupAngle();
  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
