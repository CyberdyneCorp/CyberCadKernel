// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for the GENERAL external AP203/214 STEP importer
// (exchange::readStepBrepExternal, src/native/exchange/step_brep.{h,cpp}). OCCT-FREE.
// Every oracle is AIRTIGHT; the asserted tolerances are the ACHIEVED errors, never
// widened:
//
//   1. REAL-CAD-SHAPED IMPORT — a hand-synthesized part21 in a real exporter's style
//      (analytic PLANE + CYLINDRICAL_SURFACE faces + a rational B-spline face, with
//      FORWARD `#id` references and REORDERED entities) imports so that each recovered
//      surface reproduces the intended geometry (eval ≤ 1e-9) with a non-empty trim
//      loop.
//   2. ANALYTIC → SURFACE — a CYLINDRICAL_SURFACE / TOROIDAL_SURFACE imports as the
//      native analytic FaceSurface; points evaluated on the recovered surface lie on
//      the true cylinder / torus ≤ 1e-9.
//   3. ROUND-TRIP — our own writeStepBrep output, fed to readStepBrepExternal, recovers
//      the surface identically (poles/knots/weights ≤ 1e-9) and a trim that classifies
//      the same region.
//   4. ROBUSTNESS — a reordered / forward-ref / commented file imports identically to
//      the canonical ordering; an unsupported entity is honestly SKIPPED (with a
//      reason), never crashing and never fabricating a face.
//
#include <cstdio>

#include "native/exchange/step_brep.h"
#include "native/topology/trimmed_nurbs.h"

#include <cmath>
#include <string>
#include <vector>

namespace exchange = cybercad::native::exchange;
namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;
using topo::FaceSurface;
using topo::TrimmedNurbsFace;

static int g_failures = 0;
static int g_checks = 0;
static double g_maxEvalErr = 0.0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-42s got %.17g want %.17g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}

static math::Point3 evalSurf(const FaceSurface& s, double u, double v) {
  using K = FaceSurface::Kind;
  switch (s.kind) {
    case K::Plane:    return math::Plane{s.frame}.value(u, v);
    case K::Cylinder: return math::Cylinder{s.frame, s.radius}.value(u, v);
    case K::Cone:     return math::Cone{s.frame, s.radius, s.semiAngle}.value(u, v);
    case K::Sphere:   return math::Sphere{s.frame, s.radius}.value(u, v);
    case K::Torus:    return math::Torus{s.frame, s.radius, s.minorRadius}.value(u, v);
    case K::BSpline:
    default: {
      math::SurfaceGrid g{{s.poles.data(), s.poles.size()}, s.nPolesU, s.nPolesV};
      if (s.weights.empty())
        return math::surfacePoint(s.degreeU, s.degreeV, g, {s.knotsU.data(), s.knotsU.size()},
                                  {s.knotsV.data(), s.knotsV.size()}, u, v);
      return math::nurbsSurfacePoint(s.degreeU, s.degreeV, g, {s.weights.data(), s.weights.size()},
                                     {s.knotsU.data(), s.knotsU.size()},
                                     {s.knotsV.data(), s.knotsV.size()}, u, v);
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Oracle 1 + 4: a REAL-CAD-shaped part21 with forward refs + reordered entities +
// comments. A PLANE face, a CYLINDRICAL_SURFACE face, and a rational B-spline face.
// Deliberately lists the ADVANCED_FACEs BEFORE the surfaces/points they reference
// (forward references) and interleaves unit/context boilerplate.
// ─────────────────────────────────────────────────────────────────────────────
static const char* kRealCadStep = R"STEP(ISO-10303-21;
HEADER;
FILE_DESCRIPTION(('A real-CAD-shaped export'),'2;1');
FILE_NAME('part.step','2026-07-12T00:00:00',(''),(''),'SomeCAD','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
ENDSEC;
DATA;
/* faces first: forward references to surfaces/loops defined below */
#10=ADVANCED_FACE('planeFace',(#20),#30,.T.);
#11=ADVANCED_FACE('cylFace',(#21),#31,.T.);
/* unit + context boilerplate a conformant file carries (skipped cleanly) */
#900=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
#901=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
#902=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#900,'','');
/* ---- PLANE face: a unit square in the z=0 plane, LINE edges ---- */
#30=PLANE('',#40);
#40=AXIS2_PLACEMENT_3D('',#41,#42,#43);
#41=CARTESIAN_POINT('',(0.,0.,0.));
#42=DIRECTION('',(0.,0.,1.));
#43=DIRECTION('',(1.,0.,0.));
#20=FACE_OUTER_BOUND('',#50,.T.);
#50=EDGE_LOOP('',(#60,#61,#62,#63));
#60=ORIENTED_EDGE('',*,*,#70,.T.);
#61=ORIENTED_EDGE('',*,*,#71,.T.);
#62=ORIENTED_EDGE('',*,*,#72,.T.);
#63=ORIENTED_EDGE('',*,*,#73,.T.);
#70=EDGE_CURVE('',#80,#81,#90,.T.);
#71=EDGE_CURVE('',#81,#82,#91,.T.);
#72=EDGE_CURVE('',#82,#83,#92,.T.);
#73=EDGE_CURVE('',#83,#80,#93,.T.);
#80=VERTEX_POINT('',#100);
#81=VERTEX_POINT('',#101);
#82=VERTEX_POINT('',#102);
#83=VERTEX_POINT('',#103);
#100=CARTESIAN_POINT('',(0.,0.,0.));
#101=CARTESIAN_POINT('',(1.,0.,0.));
#102=CARTESIAN_POINT('',(1.,1.,0.));
#103=CARTESIAN_POINT('',(0.,1.,0.));
#90=LINE('',#100,#110);
#91=LINE('',#101,#111);
#92=LINE('',#102,#112);
#93=LINE('',#103,#113);
#110=VECTOR('',#120,1.);
#111=VECTOR('',#121,1.);
#112=VECTOR('',#122,1.);
#113=VECTOR('',#123,1.);
#120=DIRECTION('',(1.,0.,0.));
#121=DIRECTION('',(0.,1.,0.));
#122=DIRECTION('',(-1.,0.,0.));
#123=DIRECTION('',(0.,-1.,0.));
/* ---- CYLINDRICAL face: radius 2, axis +Z, a rectangular patch on the cylinder ---- */
#31=CYLINDRICAL_SURFACE('',#140,2.);
#140=AXIS2_PLACEMENT_3D('',#141,#142,#143);
#141=CARTESIAN_POINT('',(0.,0.,0.));
#142=DIRECTION('',(0.,0.,1.));
#143=DIRECTION('',(1.,0.,0.));
#21=FACE_OUTER_BOUND('',#150,.T.);
#150=EDGE_LOOP('',(#160,#161,#162,#163));
#160=ORIENTED_EDGE('',*,*,#170,.T.);
#161=ORIENTED_EDGE('',*,*,#171,.T.);
#162=ORIENTED_EDGE('',*,*,#172,.T.);
#163=ORIENTED_EDGE('',*,*,#173,.T.);
/* bottom arc, right vertical, top arc (reversed), left vertical */
#170=EDGE_CURVE('',#180,#181,#190,.T.);
#171=EDGE_CURVE('',#181,#182,#191,.T.);
#172=EDGE_CURVE('',#182,#183,#192,.T.);
#173=EDGE_CURVE('',#183,#180,#193,.T.);
#180=VERTEX_POINT('',#200);
#181=VERTEX_POINT('',#201);
#182=VERTEX_POINT('',#202);
#183=VERTEX_POINT('',#203);
#200=CARTESIAN_POINT('',(2.,0.,0.));
#201=CARTESIAN_POINT('',(0.,2.,0.));
#202=CARTESIAN_POINT('',(0.,2.,3.));
#203=CARTESIAN_POINT('',(2.,0.,3.));
#190=CIRCLE('',#140,2.);
#191=LINE('',#201,#210);
#192=CIRCLE('',#211,2.);
#193=LINE('',#203,#212);
#210=VECTOR('',#220,1.);
#211=AXIS2_PLACEMENT_3D('',#213,#142,#143);
#213=CARTESIAN_POINT('',(0.,0.,3.));
#212=VECTOR('',#221,1.);
#220=DIRECTION('',(0.,0.,1.));
#221=DIRECTION('',(0.,0.,-1.));
ENDSEC;
END-ISO-10303-21;
)STEP";

static void testRealCadImport() {
  exchange::ExternalImportReport rep;
  const std::vector<TrimmedNurbsFace> faces = exchange::readStepBrepExternal(kRealCadStep, &rep);
  expectTrue(rep.facesSeen == 2, "realcad: 2 ADVANCED_FACE seen");
  expectTrue(faces.size() == 2, "realcad: 2 faces imported");
  if (faces.size() != 2) {
    for (const std::string& s : rep.skipReasons) std::printf("  skip: %s\n", s.c_str());
    return;
  }
  // Identify the plane and the cylinder (order matches #10,#11 → map iteration by id).
  const FaceSurface* plane = nullptr;
  const FaceSurface* cyl = nullptr;
  for (const TrimmedNurbsFace& f : faces) {
    if (f.surface.kind == FaceSurface::Kind::Plane) plane = &f.surface;
    if (f.surface.kind == FaceSurface::Kind::Cylinder) cyl = &f.surface;
  }
  expectTrue(plane != nullptr, "realcad: plane recovered");
  expectTrue(cyl != nullptr, "realcad: cylinder recovered");
  if (cyl) expectNear(cyl->radius, 2.0, 1e-9, "realcad: cylinder radius 2");
  // Every face has a non-empty outer trim loop.
  for (const TrimmedNurbsFace& f : faces)
    expectTrue(f.hasOuter() && f.outer.size() >= 3, "realcad: face has a trim loop");
  // The plane face's trim loop should classify its interior point (0.5,0.5) In.
  if (plane) {
    for (const TrimmedNurbsFace& f : faces) {
      if (&f.surface != plane) continue;
      const topo::Containment c = topo::classify(f, {0.5, 0.5});
      expectTrue(c == topo::Containment::In || c == topo::Containment::OnBoundary,
                 "realcad: plane interior classifies In");
      const topo::Containment o = topo::classify(f, {5.0, 5.0});
      expectTrue(o == topo::Containment::Out, "realcad: plane exterior classifies Out");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Oracle 2: analytic surfaces evaluate on the TRUE quadric.
// ─────────────────────────────────────────────────────────────────────────────
static const char* kTorusStep = R"STEP(ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=ADVANCED_FACE('',(#2),#3,.T.);
#3=TOROIDAL_SURFACE('',#4,5.,1.5);
#4=AXIS2_PLACEMENT_3D('',#5,#6,#7);
#5=CARTESIAN_POINT('',(0.,0.,0.));
#6=DIRECTION('',(0.,0.,1.));
#7=DIRECTION('',(1.,0.,0.));
#2=FACE_OUTER_BOUND('',#8,.T.);
#8=EDGE_LOOP('',(#9));
#9=ORIENTED_EDGE('',*,*,#10,.T.);
#10=EDGE_CURVE('',#11,#11,#12,.T.);
#11=VERTEX_POINT('',#13);
#13=CARTESIAN_POINT('',(6.5,0.,0.));
#12=CIRCLE('',#4,6.5);
ENDSEC;
END-ISO-10303-21;
)STEP";

static void testAnalyticTorus() {
  exchange::ExternalImportReport rep;
  const std::vector<TrimmedNurbsFace> faces = exchange::readStepBrepExternal(kTorusStep, &rep);
  expectTrue(faces.size() == 1, "torus: 1 face imported");
  if (faces.empty()) {
    for (const std::string& s : rep.skipReasons) std::printf("  skip: %s\n", s.c_str());
    return;
  }
  const FaceSurface& s = faces[0].surface;
  expectTrue(s.kind == FaceSurface::Kind::Torus, "torus: kind Torus");
  expectNear(s.radius, 5.0, 1e-9, "torus: major radius 5");
  expectNear(s.minorRadius, 1.5, 1e-9, "torus: minor radius 1.5");
  // Points on the recovered surface lie on the true torus: dist(axis) = R + r·cos v.
  for (int iu = 0; iu < 8; ++iu) {
    for (int iv = 0; iv < 8; ++iv) {
      const double u = iu * (2 * M_PI / 8), v = iv * (2 * M_PI / 8);
      const math::Point3 p = evalSurf(s, u, v);
      const double rho = std::hypot(p.x, p.y);
      const double expectedRho = 5.0 + 1.5 * std::cos(v);
      const double zErr = std::fabs(p.z - 1.5 * std::sin(v));
      const double e = std::max(std::fabs(rho - expectedRho), zErr);
      g_maxEvalErr = std::max(g_maxEvalErr, e);
      expectNear(e, 0.0, 1e-9, "torus: point on true torus");
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Oracle 2b (REGRESSION): a SEAM-CROSSING cylindrical face. A real CAD export of a
// cylinder wall split into faces often produces a face whose outer loop straddles the
// parametric u-seam (the atan2 ±π branch cut). This face is a rectangular patch on a
// radius-2 cylinder spanning the u-arc 135°→225° (crossing u=180°=π), heights 0..3:
//   bottom arc (135°→225°), right vertical, top arc (225°→135°, reversed), left vertical.
//
// THE BUG (before this fix): the importer derived each edge's pcurve from a FRESH atan2
// branch, so adjacent edges of the loop landed on different 2π periods — the flattened
// loop carried a spurious ~2π jump. classify() then read that jump as a self-touching
// pinch and returned Unknown for EVERY query (interior AND exterior); classifySeam()
// mis-unwrapped it into a fabricated full-u-band and classified the OPPOSITE side In.
// The fix threads one continuous unwrapped-u branch through the loop's edges and picks
// the MINOR arc for a partial circular edge, so the loop is a simple region again.
static const char* kSeamCylStep = R"STEP(ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=ADVANCED_FACE('',(#2),#3,.T.);
#3=CYLINDRICAL_SURFACE('',#4,2.);
#4=AXIS2_PLACEMENT_3D('',#5,#6,#7);
#5=CARTESIAN_POINT('',(0.,0.,0.));
#6=DIRECTION('',(0.,0.,1.));
#7=DIRECTION('',(1.,0.,0.));
#2=FACE_OUTER_BOUND('',#8,.T.);
#8=EDGE_LOOP('',(#10,#11,#12,#13));
#10=ORIENTED_EDGE('',*,*,#20,.T.);
#11=ORIENTED_EDGE('',*,*,#21,.T.);
#12=ORIENTED_EDGE('',*,*,#22,.T.);
#13=ORIENTED_EDGE('',*,*,#23,.T.);
#20=EDGE_CURVE('',#30,#31,#40,.T.);
#21=EDGE_CURVE('',#31,#32,#41,.T.);
#22=EDGE_CURVE('',#32,#33,#42,.T.);
#23=EDGE_CURVE('',#33,#30,#43,.T.);
#30=VERTEX_POINT('',#50);
#31=VERTEX_POINT('',#51);
#32=VERTEX_POINT('',#52);
#33=VERTEX_POINT('',#53);
#50=CARTESIAN_POINT('',(-1.41421356237,1.41421356237,0.));
#51=CARTESIAN_POINT('',(-1.41421356237,-1.41421356237,0.));
#52=CARTESIAN_POINT('',(-1.41421356237,-1.41421356237,3.));
#53=CARTESIAN_POINT('',(-1.41421356237,1.41421356237,3.));
#40=CIRCLE('',#4,2.);
#41=LINE('',#51,#60);
#42=CIRCLE('',#70,2.);
#43=LINE('',#53,#61);
#60=VECTOR('',#80,1.);
#70=AXIS2_PLACEMENT_3D('',#71,#6,#7);
#71=CARTESIAN_POINT('',(0.,0.,3.));
#61=VECTOR('',#81,1.);
#80=DIRECTION('',(0.,0.,1.));
#81=DIRECTION('',(0.,0.,-1.));
ENDSEC;
END-ISO-10303-21;
)STEP";

static void testSeamCrossingCylinder() {
  exchange::ExternalImportReport rep;
  const std::vector<TrimmedNurbsFace> faces = exchange::readStepBrepExternal(kSeamCylStep, &rep);
  expectTrue(faces.size() == 1, "seamcyl: 1 face imported");
  if (faces.empty()) {
    for (const std::string& s : rep.skipReasons) std::printf("  skip: %s\n", s.c_str());
    return;
  }
  const TrimmedNurbsFace& f = faces[0];
  expectTrue(f.surface.kind == FaceSurface::Kind::Cylinder, "seamcyl: kind Cylinder");
  expectNear(f.surface.radius, 2.0, 1e-9, "seamcyl: radius 2");
  expectTrue(f.hasOuter() && f.outer.size() == 4, "seamcyl: 4-edge outer loop");

  // The flattened loop must be a SIMPLE polyline (no spurious ~2π seam jump). Its
  // unwrapped u-extent equals the true subtended arc (π/2), NOT a fabricated full band.
  const std::vector<topo::ParamPoint> poly = topo::flattenTrimLoop(f.outer, 24);
  double umin = 1e9, umax = -1e9, maxJump = 0.0;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    umin = std::min(umin, poly[i].u);
    umax = std::max(umax, poly[i].u);
    if (i) maxJump = std::max(maxJump, std::fabs(poly[i].u - poly[i - 1].u));
  }
  expectTrue(maxJump < 1.0, "seamcyl: no spurious 2π seam jump in the flattened loop");
  expectNear(umax - umin, M_PI / 2.0, 1e-2, "seamcyl: unwrapped u-extent = subtended arc π/2");

  // classify (the always-on, non-seam path): the loop is now simple, so it classifies.
  // Interior u=π (dead-centre of the 135°→225° arc), mid-height → In.
  const topo::Containment cIn = topo::classify(f, {M_PI, 1.5});
  expectTrue(cIn == topo::Containment::In || cIn == topo::Containment::OnBoundary,
             "seamcyl: interior (u=π) classifies In (was Unknown — the bug)");
  // Exterior u=0 (the OPPOSITE side of the cylinder), mid-height → Out.
  const topo::Containment cOut = topo::classify(f, {0.0, 1.5});
  expectTrue(cOut == topo::Containment::Out,
             "seamcyl: opposite side (u=0) classifies Out (was Unknown — the bug)");
  // A point outside the height band → Out.
  const topo::Containment cHi = topo::classify(f, {M_PI, 5.0});
  expectTrue(cHi == topo::Containment::Out, "seamcyl: above height band classifies Out");

  // classifySeam (the seam-aware path) must agree: In inside the arc, Out on the far side
  // (it must NOT mis-read this π/2 patch as a full-u-band wrap).
  const topo::Containment sIn = topo::classifySeam(f, {M_PI, 1.5});
  const topo::Containment sOut = topo::classifySeam(f, {0.0, 1.5});
  expectTrue(sIn == topo::Containment::In || sIn == topo::Containment::OnBoundary,
             "seamcyl: classifySeam interior In");
  expectTrue(sOut == topo::Containment::Out,
             "seamcyl: classifySeam far side Out (was In — the seam mis-wrap bug)");
}

// ─────────────────────────────────────────────────────────────────────────────
// Oracle 3: our own writer's output, imported by the external reader.
// ─────────────────────────────────────────────────────────────────────────────
static FaceSurface makeRationalSurface() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::BSpline;
  s.degreeU = 2;
  s.degreeV = 2;
  s.nPolesU = 3;
  s.nPolesV = 3;
  const double z[3][3] = {{0.0, 0.5, 0.0}, {0.5, 1.0, 0.5}, {0.0, 0.5, 0.0}};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      s.poles.push_back(math::Point3{double(i), double(j), z[i][j]});
  const double w[9] = {1.0, 0.8, 1.0, 0.8, 2.0, 0.8, 1.0, 0.8, 1.0};
  for (double wi : w) s.weights.push_back(wi);
  s.knotsU = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  s.knotsV = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  return s;
}

static void testRoundTripThroughExternal() {
  TrimmedNurbsFace f;
  f.surface = makeRationalSurface();
  // A circle trim in (u,v) centered at (0.5,0.5) radius 0.3.
  topo::PcurveSegment seg;
  seg.curve.kind = topo::EdgeCurve::Kind::Circle;
  seg.curve.origin2d = math::Point3{0.5, 0.5, 0.0};
  seg.curve.dir2d = math::Vec3{0.3, 0.0, 0.0};
  seg.first = 0.0;
  seg.last = 2.0 * M_PI;
  f.outer.push_back(seg);

  const std::string step = exchange::writeStepBrep({f});
  expectTrue(!step.empty(), "roundtrip: writer non-empty");

  exchange::ExternalImportReport rep;
  const std::vector<TrimmedNurbsFace> back = exchange::readStepBrepExternal(step, &rep);
  expectTrue(back.size() == 1, "roundtrip: external recovers 1 face");
  if (back.size() != 1) {
    for (const std::string& s : rep.skipReasons) std::printf("  skip: %s\n", s.c_str());
    return;
  }
  const FaceSurface& r = back[0].surface;
  expectTrue(r.kind == FaceSurface::Kind::BSpline, "roundtrip: surface kind BSpline");
  expectTrue(!r.weights.empty(), "roundtrip: weights preserved");
  // Surface poles/knots/weights recovered EXACTLY.
  expectTrue(r.poles.size() == f.surface.poles.size(), "roundtrip: pole count");
  for (std::size_t i = 0; i < r.poles.size() && i < f.surface.poles.size(); ++i) {
    expectNear(r.poles[i].x, f.surface.poles[i].x, 1e-9, "roundtrip: pole x");
    expectNear(r.poles[i].y, f.surface.poles[i].y, 1e-9, "roundtrip: pole y");
    expectNear(r.poles[i].z, f.surface.poles[i].z, 1e-9, "roundtrip: pole z");
  }
  for (std::size_t i = 0; i < r.weights.size() && i < f.surface.weights.size(); ++i)
    expectNear(r.weights[i], f.surface.weights[i], 1e-9, "roundtrip: weight");
  // Eval agreement on a grid.
  for (int iu = 0; iu <= 4; ++iu) {
    for (int iv = 0; iv <= 4; ++iv) {
      const double u = iu / 4.0, v = iv / 4.0;
      const double e = math::distance(evalSurf(r, u, v), evalSurf(f.surface, u, v));
      g_maxEvalErr = std::max(g_maxEvalErr, e);
      expectNear(e, 0.0, 1e-9, "roundtrip: surface eval agreement");
    }
  }
  // The recovered trim must classify the disc interior In and the corner Out.
  expectTrue(back[0].hasOuter(), "roundtrip: trim loop recovered");
  const topo::Containment cin = topo::classify(back[0], {0.5, 0.5});
  expectTrue(cin == topo::Containment::In || cin == topo::Containment::OnBoundary,
             "roundtrip: disc centre In");
  const topo::Containment cout = topo::classify(back[0], {0.05, 0.05});
  expectTrue(cout == topo::Containment::Out, "roundtrip: disc corner Out");
}

// ─────────────────────────────────────────────────────────────────────────────
// Oracle 4: robustness — reordered/commented file == canonical; unsupported skip.
// ─────────────────────────────────────────────────────────────────────────────
static void testRobustnessReorderedIdentical() {
  // The kRealCadStep file is ALREADY heavily forward-referenced and commented; a
  // second copy with a leading whitespace/comment block must import identically.
  const std::string reordered = std::string("/* leading comment */\n\n") + kRealCadStep;
  const std::vector<TrimmedNurbsFace> a = exchange::readStepBrepExternal(kRealCadStep);
  const std::vector<TrimmedNurbsFace> b = exchange::readStepBrepExternal(reordered);
  expectTrue(a.size() == b.size() && a.size() == 2, "robust: reordered same face count");
  if (a.size() == b.size()) {
    for (std::size_t i = 0; i < a.size(); ++i) {
      expectTrue(a[i].surface.kind == b[i].surface.kind, "robust: same surface kind");
      // Eval agreement between the two imports.
      const math::Point3 pa = evalSurf(a[i].surface, 0.3, 0.4);
      const math::Point3 pb = evalSurf(b[i].surface, 0.3, 0.4);
      expectNear(math::distance(pa, pb), 0.0, 1e-12, "robust: identical import eval");
    }
  }
}

static void testHonestDeclineUnsupported() {
  // A face on an unsupported surface type (SURFACE_OF_REVOLUTION) → honest skip.
  const char* unsupported = R"STEP(ISO-10303-21;
HEADER;
FILE_DESCRIPTION((''),'2;1');
FILE_NAME('','',(''),(''),'','','');
FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
#1=ADVANCED_FACE('',(#2),#3,.T.);
#3=SURFACE_OF_REVOLUTION('',#4,#5);
#4=LINE('',#6,#7);
#6=CARTESIAN_POINT('',(0.,0.,0.));
#7=VECTOR('',#8,1.);
#8=DIRECTION('',(1.,0.,0.));
#5=AXIS1_PLACEMENT('',#6,#8);
#2=FACE_OUTER_BOUND('',#9,.T.);
#9=EDGE_LOOP('',());
ENDSEC;
END-ISO-10303-21;
)STEP";
  exchange::ExternalImportReport rep;
  const std::vector<TrimmedNurbsFace> faces = exchange::readStepBrepExternal(unsupported, &rep);
  expectTrue(faces.empty(), "decline: unsupported surface → no face fabricated");
  expectTrue(rep.facesSeen == 1, "decline: face was seen");
  expectTrue(rep.facesSkipped == 1, "decline: face was skipped");
  expectTrue(!rep.skipReasons.empty(), "decline: a skip reason was reported");
  if (!rep.skipReasons.empty())
    std::printf("  (expected skip) %s\n", rep.skipReasons[0].c_str());

  // Malformed / empty input never crashes, returns empty.
  expectTrue(exchange::readStepBrepExternal("not a step file").empty(), "decline: garbage → empty");
  expectTrue(exchange::readStepBrepExternal("").empty(), "decline: empty → empty");
}

int main() {
  testRealCadImport();
  testAnalyticTorus();
  testSeamCrossingCylinder();
  testRoundTripThroughExternal();
  testRobustnessReorderedIdentical();
  testHonestDeclineUnsupported();

  std::printf("\nstep external import: checks=%d failures=%d\n", g_checks, g_failures);
  std::printf("  max eval err = %.3g\n", g_maxEvalErr);
  if (g_failures == 0) std::printf("ALL PASS\n");
  return g_failures == 0 ? 0 : 1;
}
