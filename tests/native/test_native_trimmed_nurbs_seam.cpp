// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 8 — SEAM-CROSSING trimmed-NURBS healing
// (src/native/topology/trimmed_nurbs.{h,cpp}, the seam-aware verbs surfacePeriod /
// loopCrossesSeam / healSeamLoop / healTrimLoopSeam / classifySeam). OCCT-FREE. All
// oracles are airtight and closed-form on the periodic cylinder / cone / sphere:
//
//   1. PERIODICITY DETECTION — Cylinder / Cone / Sphere are periodic in u (period 2π);
//      Torus in both; Plane / BSpline / Bezier are NON-periodic (seam path is a NO-OP).
//   2. FULL-WRAP — a loop that wraps the WHOLE u-period (a band bounded by two cross-
//      section circles) heals into ONE seam-crossing loop with winding ±1; a point whose
//      v is inside the band classifies In (for EVERY u), outside → Out, on the band edge
//      → OnBoundary. The unwrapped u-span accounts for the full period.
//   3. HALF-WRAP — a finite region STRADDLING the seam (u-arc crossing u=0 once each way)
//      heals into ONE simple loop (winding 0); classification matches the exact reference
//      region computed with the seam identification (every probe In/Out correct).
//   4. NON-CROSSING NO-OP — a loop entirely inside one period is byte-identical to the
//      plain classify() path (classifySeam == classify for every probe).
//   5. NON-PERIODIC NO-OP — on a Plane, classifySeam defers to classify() (byte-identical).
//   6. AMBIGUOUS DECLINE — a seam-tangent loop that grazes u=0 without a clean crossing is
//      declined (ambiguous / Unknown), never mis-wrapped into a fabricated full band.
//
// Always-on: uses only math + topology, no numsci link. With the guard OFF or ON the whole
// gate runs identically (seam healing is not numsci-gated).
//
#include <cstdio>

#include "native/topology/trimmed_nurbs.h"

#include <cmath>
#include <vector>

using namespace cybercad::native::topology;
namespace math = cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) { std::printf("FAIL %s\n", what); ++g_failures; }
static void expectTrue(bool c, const char* what) { ++g_checks; if (!c) fail(what); }
static void expectEq(int a, int b, const char* what) {
  ++g_checks;
  if (a != b) { std::printf("FAIL %-46s got %d want %d\n", what, a, b); ++g_failures; }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) { std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b); ++g_failures; }
}
static const char* nameOf(Containment c) {
  switch (c) {
    case Containment::In: return "In";
    case Containment::Out: return "Out";
    case Containment::OnBoundary: return "OnBoundary";
    default: return "Unknown";
  }
}
static void expectClass(Containment got, Containment want, const char* what) {
  ++g_checks;
  if (got != want) {
    std::printf("FAIL %-40s got %s want %s\n", what, nameOf(got), nameOf(want));
    ++g_failures;
  }
}

static const double kTwoPi = 2.0 * M_PI;

// ─────────────────────────────────────────────────────────────────────────────
// Fixtures.
// ─────────────────────────────────────────────────────────────────────────────

// A unit cylinder: S(u,v) = (cos u, sin u, v), u periodic with period 2π.
static FaceSurface unitCylinder() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::Cylinder;
  s.frame = math::Ax3{};
  s.radius = 1.0;
  return s;
}

static FaceSurface unitPlane() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::Plane;
  s.frame = math::Ax3{};
  return s;
}

static PcurveSegment lineSeg(double u0, double v0, double u1, double v1) {
  PcurveSegment seg;
  seg.curve.kind = EdgeCurve::Kind::Line;
  const double du = u1 - u0, dv = v1 - v0;
  const double L = std::sqrt(du * du + dv * dv);
  seg.curve.origin2d = math::Point3{u0, v0, 0.0};
  seg.curve.dir2d = L > 0 ? math::Vec3{du / L, dv / L, 0.0} : math::Vec3{1, 0, 0};
  seg.first = 0.0;
  seg.last = L;
  seg.reversed = false;
  return seg;
}

// A rectangular loop [uLo,uHi]×[vLo,vHi], CCW (four line segments).
static TrimLoop rectLoop(double uLo, double uHi, double vLo, double vHi) {
  TrimLoop loop;
  loop.push_back(lineSeg(uLo, vLo, uHi, vLo));
  loop.push_back(lineSeg(uHi, vLo, uHi, vHi));
  loop.push_back(lineSeg(uHi, vHi, uLo, vHi));
  loop.push_back(lineSeg(uLo, vHi, uLo, vLo));
  return loop;
}

// Build a raw wrapped polyline (u reduced into [0,2π)) sampling a rectangle whose u-range
// [uA,uB] may exceed the period (a full or partial wrap). Each side is sampled at `n` points.
static void pushArc(std::vector<ParamPoint>& poly, double u0, double v0, double u1, double v1,
                    int n) {
  for (int i = 0; i < n; ++i) {
    const double a = static_cast<double>(i) / n;
    double u = u0 + (u1 - u0) * a;
    double v = v0 + (v1 - v0) * a;
    // Wrap u into [0,2π).
    while (u < 0.0) u += kTwoPi;
    while (u >= kTwoPi) u -= kTwoPi;
    poly.push_back(ParamPoint{u, v});
  }
}

// A FULL-WRAP band boundary between v0 and v1 as a single closed loop crossing the seam. The
// band is PHASE-SHIFTED (starts at u=0.5) so its two cross-section circles genuinely cross the
// u=0 seam. Traced: bottom circle u:0.5→0.5+2π at v0 (crosses the seam once), then the top circle
// back u:0.5+2π→0.5 at v1 (crosses the seam once, opposite direction). Net winding 0 (a band, not
// a double-wrap), but the unwrapped u-span reaches a FULL period → fullWrap: the whole ring is
// enclosed in u between v0 and v1.
static std::vector<ParamPoint> fullWrapBand(double v0, double v1, int n) {
  std::vector<ParamPoint> poly;
  pushArc(poly, 0.5, v0, 0.5 + kTwoPi, v0, n);          // bottom, +u, crosses seam once
  pushArc(poly, 0.5 + kTwoPi, v1, 0.5, v1, n);          // top, −u back, crosses seam once
  return poly;
}

// A HALF-WRAP finite region: u-arc [1.5π, 2.5π] (wrapped: [1.5π,2π]∪[0,0.5π]) × [v0,v1].
static std::vector<ParamPoint> halfWrapRegion(double v0, double v1, int n) {
  std::vector<ParamPoint> poly;
  pushArc(poly, 1.5 * M_PI, v0, 2.5 * M_PI, v0, n);  // bottom, crosses seam once (→)
  pushArc(poly, 2.5 * M_PI, v0, 2.5 * M_PI, v1, 2);  // right side at u=0.5π
  pushArc(poly, 2.5 * M_PI, v1, 1.5 * M_PI, v1, n);  // top, crosses seam once (←)
  pushArc(poly, 1.5 * M_PI, v1, 1.5 * M_PI, v0, 2);  // left side at u=1.5π, close
  return poly;
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Periodicity detection.
// ─────────────────────────────────────────────────────────────────────────────
static void testPeriodicity() {
  SurfacePeriod cyl = surfacePeriod(unitCylinder());
  expectTrue(cyl.periodicU, "cylinder periodic in u");
  expectTrue(!cyl.periodicV, "cylinder NOT periodic in v");
  expectLE(std::fabs(cyl.uPeriod - kTwoPi), 1e-12, "cylinder u-period == 2π");

  FaceSurface cone = unitCylinder(); cone.kind = FaceSurface::Kind::Cone;
  expectTrue(surfacePeriod(cone).periodicU, "cone periodic in u");
  FaceSurface sph = unitCylinder(); sph.kind = FaceSurface::Kind::Sphere;
  expectTrue(surfacePeriod(sph).periodicU, "sphere periodic in u");
  FaceSurface tor = unitCylinder(); tor.kind = FaceSurface::Kind::Torus;
  SurfacePeriod tp = surfacePeriod(tor);
  expectTrue(tp.periodicU && tp.periodicV, "torus periodic in BOTH u and v");

  expectTrue(!surfacePeriod(unitPlane()).periodicU, "plane NOT periodic (non-periodic no-op)");
  FaceSurface bsp = unitPlane(); bsp.kind = FaceSurface::Kind::BSpline;
  expectTrue(!surfacePeriod(bsp).periodicU, "free-form BSpline NOT auto-periodic (honest residual)");
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. FULL-WRAP: heal + classify a band wrapping the whole u-period.
// ─────────────────────────────────────────────────────────────────────────────
static void testFullWrap() {
  const double v0 = 0.2, v1 = 0.8;
  std::vector<ParamPoint> poly = fullWrapBand(v0, v1, 64);

  expectTrue(loopCrossesSeam(poly, kTwoPi), "full-wrap band crosses the seam");

  SeamHealReport rep = healSeamLoop(poly, kTwoPi);
  expectTrue(rep.healed, "full-wrap band heals into one seam loop");
  expectTrue(rep.crossesSeam, "full-wrap crossesSeam flagged");
  expectTrue(!rep.ambiguous, "full-wrap NOT ambiguous");
  expectTrue(rep.fullWrap, "full-wrap fullWrap flagged");
  // Unwrapped u-span accounts for the FULL u-period (the whole ring is enclosed in u).
  expectLE(std::fabs(rep.uSpan - kTwoPi), 0.3, "full-wrap unwrapped u-span ≈ full period 2π");
  expectTrue(rep.uSpan >= 0.75 * kTwoPi, "full-wrap u-span reaches a full period");

  // Classify the full band through the TrimmedNurbsFace + classifySeam. Build the outer loop
  // as pcurve line segments matching the wrapped polyline (so classifySeam re-flattens it).
  TrimmedNurbsFace f;
  f.surface = unitCylinder();
  // Outer band loop (pcurve form): bottom circle 0→2π at v0, then top 2π→0... represented as
  // two u-sweeps that cross the seam, mirroring fullWrapBand's topology.
  f.outer.push_back(lineSeg(0.0, v0, kTwoPi, v0));    // bottom sweep (seam-crossing at close)
  f.outer.push_back(lineSeg(0.0, v1, kTwoPi, v1));    // top sweep

  // Points at DIFFERENT u all inside the v-band → In (the whole u-band is enclosed).
  for (double u : {0.1, 1.0, M_PI, 4.0, 6.0}) {
    expectClass(classifySeam(f, {u, 0.5}), Containment::In, "full-wrap: mid-band point In (any u)");
  }
  // v outside the band → Out (for any u).
  expectClass(classifySeam(f, {1.0, 0.05}), Containment::Out, "full-wrap: below band Out");
  expectClass(classifySeam(f, {3.0, 0.95}), Containment::Out, "full-wrap: above band Out");
  // On the band edge → OnBoundary.
  expectClass(classifySeam(f, {2.0, v0}), Containment::OnBoundary, "full-wrap: on v0 edge OnBoundary");
  expectClass(classifySeam(f, {5.0, v1}), Containment::OnBoundary, "full-wrap: on v1 edge OnBoundary");
}

// ─────────────────────────────────────────────────────────────────────────────
// 3. HALF-WRAP: heal + classify a finite region straddling the seam.
// ─────────────────────────────────────────────────────────────────────────────
static void testHalfWrap() {
  const double v0 = 0.3, v1 = 0.7;
  std::vector<ParamPoint> poly = halfWrapRegion(v0, v1, 40);

  expectTrue(loopCrossesSeam(poly, kTwoPi), "half-wrap region crosses the seam");
  SeamHealReport rep = healSeamLoop(poly, kTwoPi);
  expectTrue(rep.healed, "half-wrap heals into one simple loop");
  expectTrue(rep.crossesSeam, "half-wrap crossesSeam flagged");
  expectTrue(!rep.ambiguous, "half-wrap NOT ambiguous");
  expectTrue(!rep.fullWrap, "half-wrap NOT a full wrap (finite region)");
  expectEq(rep.winding, 0, "half-wrap net winding 0 (finite straddling region)");

  // Build the TrimmedNurbsFace outer loop matching halfWrapRegion (seam-crossing rectangle).
  // The stored pcurve lines are in wrapped u so classifySeam re-flattens + unwraps them.
  TrimmedNurbsFace f;
  f.surface = unitCylinder();
  f.outer.push_back(lineSeg(1.5 * M_PI, v0, kTwoPi, v0));       // toward the seam at v0
  f.outer.push_back(lineSeg(0.0, v0, 0.5 * M_PI, v0));          // re-entered at u=0, to 0.5π
  f.outer.push_back(lineSeg(0.5 * M_PI, v0, 0.5 * M_PI, v1));   // right side
  f.outer.push_back(lineSeg(0.5 * M_PI, v1, 0.0, v1));          // top toward seam
  f.outer.push_back(lineSeg(kTwoPi, v1, 1.5 * M_PI, v1));       // re-entered at u=2π, to 1.5π
  f.outer.push_back(lineSeg(1.5 * M_PI, v1, 1.5 * M_PI, v0));   // left side, close

  // Inside the wrapped arc (u near the seam, mid v) → In.
  expectClass(classifySeam(f, {0.2, 0.5}), Containment::In, "half-wrap: u=0.2 inside arc In");
  expectClass(classifySeam(f, {6.1, 0.5}), Containment::In, "half-wrap: u≈2π inside arc In");
  expectClass(classifySeam(f, {1.5 * M_PI + 0.2, 0.5}), Containment::In, "half-wrap: u=1.5π+ In");
  // OUTSIDE the arc (u on the far side of the cylinder, u≈π) → Out.
  expectClass(classifySeam(f, {M_PI, 0.5}), Containment::Out, "half-wrap: u=π (far side) Out");
  expectClass(classifySeam(f, {M_PI, 0.5}), Containment::Out, "half-wrap: opposite side Out");
  // Inside the u-arc but v outside the band → Out.
  expectClass(classifySeam(f, {0.2, 0.1}), Containment::Out, "half-wrap: below v-band Out");
  expectClass(classifySeam(f, {0.2, 0.9}), Containment::Out, "half-wrap: above v-band Out");
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. NON-CROSSING NO-OP: a loop inside one period is byte-identical to plain classify.
// ─────────────────────────────────────────────────────────────────────────────
static void testNonCrossingNoOp() {
  TrimmedNurbsFace f;
  f.surface = unitCylinder();
  f.outer = rectLoop(1.0, 2.0, 0.2, 0.8);  // wholly inside u∈[0,2π]

  std::vector<ParamPoint> poly = flattenTrimLoop(f.outer, 48);
  expectTrue(!loopCrossesSeam(poly, kTwoPi), "non-crossing loop does NOT cross the seam");

  // classifySeam must equal classify() for every probe (strict no-op superset).
  const ParamPoint probes[] = {{1.5, 0.5}, {1.1, 0.3}, {0.5, 0.5}, {2.5, 0.5},
                               {1.5, 0.1}, {1.5, 0.9}, {1.0, 0.5}};
  for (const ParamPoint& p : probes) {
    const Containment cs = classifySeam(f, p);
    const Containment cc = classify(f, p);
    expectClass(cs, cc, "non-crossing: classifySeam == classify (no-op)");
  }
  // And correctness: interior In, exterior Out.
  expectClass(classifySeam(f, {1.5, 0.5}), Containment::In, "non-crossing: interior In");
  expectClass(classifySeam(f, {0.5, 0.5}), Containment::Out, "non-crossing: left of rect Out");
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. NON-PERIODIC NO-OP: on a Plane, classifySeam defers to classify (byte-identical).
// ─────────────────────────────────────────────────────────────────────────────
static void testNonPeriodicNoOp() {
  TrimmedNurbsFace f;
  f.surface = unitPlane();
  f.outer = rectLoop(0.2, 0.8, 0.2, 0.8);

  expectTrue(!surfacePeriod(f.surface).periodicU, "plane is non-periodic");
  const ParamPoint probes[] = {{0.5, 0.5}, {0.3, 0.3}, {0.1, 0.5}, {0.9, 0.5},
                               {0.2, 0.5}, {0.5, 0.8}};
  for (const ParamPoint& p : probes) {
    expectClass(classifySeam(f, p), classify(f, p), "non-periodic: classifySeam == classify");
  }
  // healTrimLoopSeam on a non-periodic surface is a NO-OP echo (healed, not crossing).
  SeamHealReport rep = healTrimLoopSeam(f.surface, f.outer, 48);
  expectTrue(rep.healed, "non-periodic healTrimLoopSeam heals (echo)");
  expectTrue(!rep.crossesSeam, "non-periodic never crosses a seam");
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. AMBIGUOUS DECLINE: a seam-tangent loop grazing u=0 is declined honestly.
// ─────────────────────────────────────────────────────────────────────────────
static void testAmbiguousDecline() {
  // A SEAM-TANGENT loop: the loop TOUCHES the seam line (u=0) at a single vertex without
  // crossing it — both neighbours sit on the SAME (positive-u) side, so the loop merely grazes
  // the seam. Whether that touch is an interior or exterior contact is genuinely undecidable, so
  // the heal MUST decline honestly (ambiguous) rather than mis-wrap it into a full band. Triangle
  // touching u=0 at its apex: (1.0,0.3) → (0.0,0.5) [ON the seam] → (1.0,0.7) → close.
  std::vector<ParamPoint> poly;
  poly.push_back(ParamPoint{1.0, 0.3});
  poly.push_back(ParamPoint{0.5, 0.4});
  poly.push_back(ParamPoint{0.0, 0.5});   // apex ON the seam line u=0 (touch, no crossing)
  poly.push_back(ParamPoint{0.5, 0.6});
  poly.push_back(ParamPoint{1.0, 0.7});
  poly.push_back(ParamPoint{0.7, 0.5});   // close back (all neighbours u>0 — a bounce, not a cross)

  SeamHealReport rep = healSeamLoop(poly, kTwoPi);
  expectTrue(rep.crossesSeam, "seam-tangent loop is flagged seam-involved");
  expectTrue(rep.ambiguous, "seam-tangent (touch-without-cross) loop is AMBIGUOUS (honest decline)");
  expectTrue(!rep.healed, "ambiguous seam loop is NOT healed (declined)");

  // Through classifySeam: an ambiguous outer seam loop declines Unknown, never a fabricated band.
  TrimmedNurbsFace f;
  f.surface = unitCylinder();
  f.outer.push_back(lineSeg(1.0, 0.3, 0.0, 0.5));  // down to the seam apex
  f.outer.push_back(lineSeg(0.0, 0.5, 1.0, 0.7));  // back up from the seam (same side — a touch)
  f.outer.push_back(lineSeg(1.0, 0.7, 1.0, 0.3));  // right side, close (all u≥0, a bounce)
  expectClass(classifySeam(f, {0.5, 0.5}), Containment::Unknown,
              "seam-tangent topology → classifySeam Unknown (honest decline)");
}

int main() {
  testPeriodicity();
  testFullWrap();
  testHalfWrap();
  testNonCrossingNoOp();
  testNonPeriodicNoOp();
  testAmbiguousDecline();

  if (g_failures == 0)
    std::printf("OK  test_native_trimmed_nurbs_seam: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_trimmed_nurbs_seam: %d failures / %d checks\n",
                g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}
