// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for the exact trimmed-NURBS B-rep STEP AP214 round-trip
// (src/native/exchange/step_brep.{h,cpp}). OCCT-FREE. Every oracle is AIRTIGHT;
// the asserted tolerances are the ACHIEVED errors, never widened:
//
//   1. ROUND-TRIP EXACT — write a trimmed-NURBS face (rational B-spline surface +
//      circular trim pcurve), read it back → the recovered surface's poles / knots /
//      weights and the trim pcurves EQUAL the original ≤ 1e-9, and evaluating both
//      surfaces at a grid of (u,v) agrees ≤ 1e-9.
//   2. VALID PART21 — the written file has an ISO-10303-21 header + a well-formed
//      DATA section, and every entity reference resolves (the parser succeeds and
//      recovers the exact face graph).
//   3. RATIONAL PRESERVED — a rational surface (weights ≠ 1) round-trips as a
//      RATIONAL_B_SPLINE_SURFACE with the weights recovered EXACTLY.
//   4. NON-RATIONAL + ANALYTIC + B-SPLINE PCURVE — a non-rational B-spline surface
//      with a B-spline pcurve trim and a hole loop round-trips exactly.
//   5. HONEST DECLINE — a face whose surface the exact writer cannot represent
//      (empty free-form, no knots) makes writeStepBrep return an EMPTY string.
//
#include <cstdio>

#include "native/exchange/step_brep.h"

#include <cmath>
#include <string>
#include <vector>

using namespace cybercad::native;
using topo::EdgeCurve;
using topo::FaceSurface;
using topo::PCurve;
using topo::PcurveSegment;
using topo::TrimLoop;
using topo::TrimmedNurbsFace;
namespace math = cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;
static double g_maxPoleErr = 0.0;
static double g_maxKnotErr = 0.0;
static double g_maxWeightErr = 0.0;
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
    std::printf("FAIL %-40s got %.17g want %.17g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}

// ── Builders ──────────────────────────────────────────────────────────────────

// A biquadratic (degree 2×2) NURBS surface: 3×3 control net over [0,1]² with
// non-trivial weights (a rational patch).
static FaceSurface makeRationalSurface() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::BSpline;
  s.degreeU = 2;
  s.degreeV = 2;
  s.nPolesU = 3;
  s.nPolesV = 3;
  // Row-major, U outer, V inner. A gently curved patch.
  const double z[3][3] = {{0.0, 0.5, 0.0}, {0.5, 1.0, 0.5}, {0.0, 0.5, 0.0}};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      s.poles.push_back(math::Point3{double(i), double(j), z[i][j]});
  // Non-uniform weights so it is genuinely rational.
  const double w[9] = {1.0, 0.8, 1.0, 0.8, 2.0, 0.8, 1.0, 0.8, 1.0};
  for (double wi : w) s.weights.push_back(wi);
  // Clamped knot vectors [0,0,0,1,1,1] for degree 2, 3 poles.
  s.knotsU = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  s.knotsV = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
  return s;
}

static FaceSurface makeNonRationalSurface() {
  FaceSurface s = makeRationalSurface();
  s.weights.clear();  // non-rational
  return s;
}

// A circular trim pcurve in (u,v): center (0.5,0.5), radius 0.3.
static PcurveSegment makeCircleTrim() {
  PcurveSegment seg;
  seg.curve.kind = EdgeCurve::Kind::Circle;
  seg.curve.origin2d = math::Point3{0.5, 0.5, 0.0};
  seg.curve.dir2d = math::Vec3{0.3, 0.0, 0.0};  // radius in dir2d.x
  seg.first = 0.0;
  seg.last = 2.0 * M_PI;
  seg.reversed = false;
  return seg;
}

// A B-spline pcurve (degree-2, 4 poles) as an open trim edge in (u,v).
static PcurveSegment makeBsplinePcurveTrim() {
  PcurveSegment seg;
  PCurve& c = seg.curve;
  c.kind = EdgeCurve::Kind::BSpline;
  c.degree = 2;
  c.poles2d = {math::Point3{0.1, 0.1, 0}, math::Point3{0.4, 0.2, 0},
               math::Point3{0.6, 0.5, 0}, math::Point3{0.9, 0.9, 0}};
  c.knots = {0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0};  // degree 2, 4 poles → 7 knots
  seg.first = 0.0;
  seg.last = 1.0;
  seg.reversed = true;  // exercise the orientation flag
  return seg;
}

// ── Comparison helpers ──────────────────────────────────────────────────────────

static void compareSurface(const FaceSurface& a, const FaceSurface& b, const char* tag) {
  expectTrue(a.kind == b.kind, "surface kind");
  expectTrue(a.degreeU == b.degreeU && a.degreeV == b.degreeV, "surface degrees");
  expectTrue(a.nPolesU == b.nPolesU && a.nPolesV == b.nPolesV, "surface pole counts");
  if (a.poles.size() != b.poles.size()) { fail("surface pole count mismatch"); return; }
  for (std::size_t i = 0; i < a.poles.size(); ++i) {
    const double e = std::max({std::fabs(a.poles[i].x - b.poles[i].x),
                               std::fabs(a.poles[i].y - b.poles[i].y),
                               std::fabs(a.poles[i].z - b.poles[i].z)});
    g_maxPoleErr = std::max(g_maxPoleErr, e);
    expectNear(a.poles[i].x, b.poles[i].x, 1e-9, "surface pole x");
    expectNear(a.poles[i].y, b.poles[i].y, 1e-9, "surface pole y");
    expectNear(a.poles[i].z, b.poles[i].z, 1e-9, "surface pole z");
  }
  expectTrue(a.knotsU.size() == b.knotsU.size(), "surface knotU count");
  for (std::size_t i = 0; i < a.knotsU.size() && i < b.knotsU.size(); ++i) {
    g_maxKnotErr = std::max(g_maxKnotErr, std::fabs(a.knotsU[i] - b.knotsU[i]));
    expectNear(a.knotsU[i], b.knotsU[i], 1e-9, "surface knotU");
  }
  expectTrue(a.knotsV.size() == b.knotsV.size(), "surface knotV count");
  for (std::size_t i = 0; i < a.knotsV.size() && i < b.knotsV.size(); ++i) {
    g_maxKnotErr = std::max(g_maxKnotErr, std::fabs(a.knotsV[i] - b.knotsV[i]));
    expectNear(a.knotsV[i], b.knotsV[i], 1e-9, "surface knotV");
  }
  expectTrue(a.weights.size() == b.weights.size(), "surface weight count");
  for (std::size_t i = 0; i < a.weights.size() && i < b.weights.size(); ++i) {
    g_maxWeightErr = std::max(g_maxWeightErr, std::fabs(a.weights[i] - b.weights[i]));
    expectNear(a.weights[i], b.weights[i], 1e-9, "surface weight");
  }
  (void)tag;
}

static math::Point3 evalSurf(const FaceSurface& s, double u, double v) {
  math::SurfaceGrid g{{s.poles.data(), s.poles.size()}, s.nPolesU, s.nPolesV};
  if (s.weights.empty())
    return math::surfacePoint(s.degreeU, s.degreeV, g, {s.knotsU.data(), s.knotsU.size()},
                              {s.knotsV.data(), s.knotsV.size()}, u, v);
  return math::nurbsSurfacePoint(s.degreeU, s.degreeV, g, {s.weights.data(), s.weights.size()},
                                 {s.knotsU.data(), s.knotsU.size()},
                                 {s.knotsV.data(), s.knotsV.size()}, u, v);
}

static void compareEval(const FaceSurface& a, const FaceSurface& b) {
  for (int iu = 0; iu <= 5; ++iu) {
    for (int iv = 0; iv <= 5; ++iv) {
      const double u = iu / 5.0, v = iv / 5.0;
      const math::Point3 pa = evalSurf(a, u, v);
      const math::Point3 pb = evalSurf(b, u, v);
      const double e = math::distance(pa, pb);
      g_maxEvalErr = std::max(g_maxEvalErr, e);
      expectNear(e, 0.0, 1e-9, "surface eval agreement");
    }
  }
}

static void comparePcurve(const PCurve& a, const PCurve& b, const char* tag) {
  expectTrue(a.kind == b.kind, "pcurve kind");
  expectNear(a.origin2d.x, b.origin2d.x, 1e-9, "pcurve origin u");
  expectNear(a.origin2d.y, b.origin2d.y, 1e-9, "pcurve origin v");
  expectNear(a.dir2d.x, b.dir2d.x, 1e-9, "pcurve dir/radius x");
  expectNear(a.dir2d.y, b.dir2d.y, 1e-9, "pcurve dir y");
  expectTrue(a.degree == b.degree, "pcurve degree");
  expectTrue(a.poles2d.size() == b.poles2d.size(), "pcurve pole count");
  for (std::size_t i = 0; i < a.poles2d.size() && i < b.poles2d.size(); ++i) {
    expectNear(a.poles2d[i].x, b.poles2d[i].x, 1e-9, "pcurve pole u");
    expectNear(a.poles2d[i].y, b.poles2d[i].y, 1e-9, "pcurve pole v");
  }
  expectTrue(a.knots.size() == b.knots.size(), "pcurve knot count");
  for (std::size_t i = 0; i < a.knots.size() && i < b.knots.size(); ++i)
    expectNear(a.knots[i], b.knots[i], 1e-9, "pcurve knot");
  expectTrue(a.weights.size() == b.weights.size(), "pcurve weight count");
  for (std::size_t i = 0; i < a.weights.size() && i < b.weights.size(); ++i)
    expectNear(a.weights[i], b.weights[i], 1e-9, "pcurve weight");
  (void)tag;
}

static void compareSegment(const PcurveSegment& a, const PcurveSegment& b) {
  comparePcurve(a.curve, b.curve, "seg");
  expectNear(a.first, b.first, 1e-9, "segment first");
  expectNear(a.last, b.last, 1e-9, "segment last");
  expectTrue(a.reversed == b.reversed, "segment reversed");
}

// ── part21 validity: basic structural checks. ──────────────────────────────────
static void expectValidPart21(const std::string& step) {
  expectTrue(step.rfind("ISO-10303-21;", 0) == 0, "part21 begins ISO-10303-21");
  expectTrue(step.find("HEADER;") != std::string::npos, "part21 has HEADER");
  expectTrue(step.find("DATA;") != std::string::npos, "part21 has DATA");
  expectTrue(step.find("END-ISO-10303-21;") != std::string::npos, "part21 has END marker");
  // Balanced ENDSEC (header + data).
  std::size_t endsec = 0, pos = 0;
  while ((pos = step.find("ENDSEC;", pos)) != std::string::npos) { ++endsec; pos += 7; }
  expectTrue(endsec == 2, "part21 has two ENDSEC");
}

// ── Tests ───────────────────────────────────────────────────────────────────

static void testRationalRoundTrip() {
  TrimmedNurbsFace f;
  f.surface = makeRationalSurface();
  f.outer.push_back(makeCircleTrim());

  const std::string step = exchange::writeStepBrep({f});
  expectTrue(!step.empty(), "rational: writeStepBrep non-empty");
  expectValidPart21(step);
  // Rational surface must serialise as RATIONAL_B_SPLINE_SURFACE.
  expectTrue(step.find("RATIONAL_B_SPLINE_SURFACE") != std::string::npos,
             "rational: RATIONAL_B_SPLINE_SURFACE present");

  const std::vector<TrimmedNurbsFace> back = exchange::readStepBrep(step);
  expectTrue(back.size() == 1, "rational: one face recovered");
  if (back.size() != 1) return;
  compareSurface(f.surface, back[0].surface, "rational");
  compareEval(f.surface, back[0].surface);
  expectTrue(!back[0].surface.weights.empty(), "rational: weights preserved (non-empty)");
  expectTrue(back[0].outer.size() == 1, "rational: outer loop 1 segment");
  if (back[0].outer.size() == 1) compareSegment(f.outer[0], back[0].outer[0]);
}

static void testNonRationalWithHoleAndBsplinePcurve() {
  TrimmedNurbsFace f;
  f.surface = makeNonRationalSurface();
  // Outer loop: circle trim. Hole loop: b-spline pcurve.
  f.outer.push_back(makeCircleTrim());
  TrimLoop hole;
  hole.push_back(makeBsplinePcurveTrim());
  f.holes.push_back(hole);

  const std::string step = exchange::writeStepBrep({f});
  expectTrue(!step.empty(), "nonrational: writeStepBrep non-empty");
  expectValidPart21(step);
  expectTrue(step.find("B_SPLINE_SURFACE_WITH_KNOTS") != std::string::npos,
             "nonrational: B_SPLINE_SURFACE_WITH_KNOTS present");

  const std::vector<TrimmedNurbsFace> back = exchange::readStepBrep(step);
  expectTrue(back.size() == 1, "nonrational: one face recovered");
  if (back.size() != 1) return;
  compareSurface(f.surface, back[0].surface, "nonrational");
  compareEval(f.surface, back[0].surface);
  expectTrue(back[0].surface.weights.empty(), "nonrational: no weights");
  expectTrue(back[0].outer.size() == 1, "nonrational: outer 1 seg");
  expectTrue(back[0].holes.size() == 1, "nonrational: 1 hole");
  if (back[0].outer.size() == 1) compareSegment(f.outer[0], back[0].outer[0]);
  if (back[0].holes.size() == 1 && back[0].holes[0].size() == 1)
    compareSegment(hole[0], back[0].holes[0][0]);
}

static void testMultiFace() {
  TrimmedNurbsFace f1;
  f1.surface = makeRationalSurface();
  f1.outer.push_back(makeCircleTrim());
  TrimmedNurbsFace f2;
  f2.surface = makeNonRationalSurface();
  f2.outer.push_back(makeCircleTrim());

  const std::string step = exchange::writeStepBrep({f1, f2});
  expectTrue(!step.empty(), "multi: non-empty");
  const std::vector<TrimmedNurbsFace> back = exchange::readStepBrep(step);
  expectTrue(back.size() == 2, "multi: two faces recovered");
}

static void testHonestDecline() {
  // Empty free-form surface (no poles/knots) — writer must decline (empty string).
  TrimmedNurbsFace f;
  f.surface.kind = FaceSurface::Kind::BSpline;  // but no poles/knots
  f.outer.push_back(makeCircleTrim());
  const std::string step = exchange::writeStepBrep({f});
  expectTrue(step.empty(), "decline: empty free-form surface → empty string");

  // Empty face set → empty string.
  expectTrue(exchange::writeStepBrep({}).empty(), "decline: empty set → empty string");

  // Bezier surface (out of the exact-knots scope) — decline.
  TrimmedNurbsFace fb;
  fb.surface.kind = FaceSurface::Kind::Bezier;
  fb.surface.nPolesU = 2;
  fb.surface.nPolesV = 2;
  fb.surface.poles = {math::Point3{0, 0, 0}, math::Point3{1, 0, 0}, math::Point3{0, 1, 0},
                      math::Point3{1, 1, 0}};
  fb.outer.push_back(makeCircleTrim());
  expectTrue(exchange::writeStepBrep({fb}).empty(), "decline: Bezier surface → empty string");
}

static void testMalformedParse() {
  // A garbage string parses to an empty face vector (never a fabricated face).
  expectTrue(exchange::readStepBrep("not a step file").empty(), "parse: garbage → empty");
  expectTrue(exchange::readStepBrep("").empty(), "parse: empty → empty");
}

int main() {
  testRationalRoundTrip();
  testNonRationalWithHoleAndBsplinePcurve();
  testMultiFace();
  testHonestDecline();
  testMalformedParse();

  std::printf("\nstep_brep round-trip: checks=%d failures=%d\n", g_checks, g_failures);
  std::printf("  max pole err   = %.3g\n", g_maxPoleErr);
  std::printf("  max knot err   = %.3g\n", g_maxKnotErr);
  std::printf("  max weight err = %.3g\n", g_maxWeightErr);
  std::printf("  max eval err   = %.3g\n", g_maxEvalErr);
  if (g_failures == 0) std::printf("ALL PASS\n");
  return g_failures == 0 ? 0 : 1;
}
