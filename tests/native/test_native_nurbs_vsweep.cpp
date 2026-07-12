// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — VARIABLE-SECTION and TWO-RAIL swept
// surfaces (src/native/math/bspline_sweep.{h,cpp}: sweepVariable / sweepRationalVariable /
// sweepTwoRail / sweepRationalTwoRail). OCCT-FREE. The oracles are airtight and closed-form:
//
//   1. CONSTANT-SECTION DEGENERATE — sweepVariable with scales≡1, twists≡0 reproduces the
//      existing RMF sweepAlongTrajectory EXACTLY (≤1e-12), and the rational variant reproduces
//      sweepRationalAlongTrajectory. (No scale/twist ⇒ identity in the section plane.)
//   2. LINEAR-TAPER CONE — a rational circle of radius R swept along a straight +Z path with a
//      linear scale s(t)=s0+(s1−s0)·t gives an EXACT rational CONE FRUSTUM: every surface point
//      at station v_k lies at radius s(t_k)·R and the placed section is the circle scaled about
//      its center (≤1e-9). Non-rational (polyline-circle) taper: the placed section is the
//      section scaled about the origin, contained by the surface (skin containment).
//   3. TWO-RAIL SANITY — two parallel straight rails + a segment section reproduce a planar/
//      ruled strip; two rails diverging linearly give a tapered surface; the section anchor
//      endpoints lie ON rail0 and rail1 at EVERY station (≤1e-9).
//   4. DEGENERATE GUARDS — crossing / zero-length-chord rails, coincident anchors, bad
//      anchor indices, wrong-size or non-positive scale fields, and rational input on the
//      non-rational routines all HONEST-DECLINE (ok=false, no crash).
//
// These sweeps compose skinSurface / skinRationalSurface (which solve via numerics::lin_solve),
// so the whole gate is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_sweep). With the guard
// OFF this compiles to a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_ops.h"
#include "native/math/bspline_skin.h"
#include "native/math/bspline_sweep.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-56s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}

// ── Evaluators ─────────────────────────────────────────────────────────────────
static Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}
static Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
}
static Point3 evalRatCurve(const BsplineCurveData& c, double u) {
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}
static Point3 evalRatSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}
static double isoVsCurveMaxDev(const BsplineSurfaceData& s, double v, const BsplineCurveData& c,
                               int nS = 200) {
  double worst = 0.0;
  for (int i = 0; i <= nS; ++i) {
    const double u = static_cast<double>(i) / nS;
    worst = std::max(worst, distance(evalSurface(s, u, v), evalCurve(c, u)));
  }
  return worst;
}
static double ratIsoVsCurveMaxDev(const BsplineSurfaceData& s, double v,
                                  const BsplineCurveData& c, int nS = 200) {
  double worst = 0.0;
  for (int i = 0; i <= nS; ++i) {
    const double u = static_cast<double>(i) / nS;
    worst = std::max(worst, distance(evalRatSurface(s, u, v), evalRatCurve(c, u)));
  }
  return worst;
}

// ── Fixtures (mirror the base sweep gate) ────────────────────────────────────────
static BsplineCurveData ratFullCircleXY(double radius) {
  const double s = std::sqrt(2.0) / 2.0;
  const double R = radius;
  BsplineCurveData c;
  c.degree = 2;
  c.knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
  c.poles = {{R, 0, 0},   {R, R, 0},   {0, R, 0},   {-R, R, 0}, {-R, 0, 0},
             {-R, -R, 0}, {0, -R, 0},  {R, -R, 0},  {R, 0, 0}};
  c.weights = {1, s, 1, s, 1, s, 1, s, 1};
  return c;
}
static BsplineCurveData sectionCubicXY() {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = {0, 0, 0, 0, 0.4, 0.7, 1, 1, 1, 1};
  c.poles = {{0, 0, 0}, {1, 2, 0}, {2, -1, 0}, {4, 1, 0}, {5, 3, 0}, {6, 0, 0}};
  return c;
}
static BsplineCurveData straightTrajectory(const Vec3& from, const Vec3& to) {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = {0, 0, 0, 0, 1, 1, 1, 1};
  for (int i = 0; i < 4; ++i) {
    const double s = static_cast<double>(i) / 3.0;
    const Vec3 p = from + (to - from) * s;
    c.poles.push_back({p.x, p.y, p.z});
  }
  return c;
}
// A degree-1 straight rail from A to B (2 poles on [0,1]).
static BsplineCurveData straightRail(const Point3& A, const Point3& B) {
  BsplineCurveData c;
  c.degree = 1;
  c.knots = {0, 0, 1, 1};
  c.poles = {A, B};
  return c;
}

int main() {
  const double PI = 3.14159265358979323846;

  // ═══ 1. CONSTANT-SECTION DEGENERATE ═════════════════════════════════════════
  // scales≡1, twists≡0 ⇒ sweepVariable == sweepAlongTrajectory pointwise (≤1e-12).
  {
    const BsplineCurveData sec = sectionCubicXY();
    const Vec3 from{0, 0, 0}, to{1, 2, 6};
    const BsplineCurveData traj = straightTrajectory(from, to);
    const Dir3 normal(0, 0, 1);
    const int stations = 8;
    const std::vector<double> ones(stations, 1.0), zeros(stations, 0.0);

    SweepResult base = sweepAlongTrajectory(sec, traj, normal, stations, 3);
    SweepResult var = sweepVariable(sec, traj, normal, ones, zeros, stations, 3);
    expectTrue(base.ok, "base sweepAlongTrajectory ok");
    expectTrue(var.ok, "sweepVariable ok (constant section)");
    expectTrue(var.surface.weights.empty(), "variable sweep non-rational");
    expectTrue(var.vParams.size() == static_cast<std::size_t>(stations), "one v-param per station");

    double worst = 0.0;
    for (int iu = 0; iu <= 24; ++iu)
      for (int iv = 0; iv <= 24; ++iv) {
        const double u = static_cast<double>(iu) / 24.0;
        const double v = static_cast<double>(iv) / 24.0;
        worst = std::max(worst,
                         distance(evalSurface(var.surface, u, v), evalSurface(base.surface, u, v)));
      }
    expectLE(worst, 1e-12, "constant-section: sweepVariable == sweepAlongTrajectory POINTWISE");
    std::printf("INFO constant-section degenerate worst dev = %.3e\n", worst);
  }

  // Rational constant-section degenerate: sweepRationalVariable == sweepRationalAlongTrajectory.
  {
    const BsplineCurveData circ = ratFullCircleXY(1.5);
    const BsplineCurveData traj = straightTrajectory({0, 0, 0}, {0, 0, 5});
    const Dir3 normal(0, 0, 1);
    const int stations = 6;
    const std::vector<double> ones(stations, 1.0), zeros(stations, 0.0);

    SweepResult base = sweepRationalAlongTrajectory(circ, traj, normal, stations, 3);
    SweepResult var = sweepRationalVariable(circ, traj, normal, ones, zeros, stations, 3);
    expectTrue(base.ok && var.ok, "rational constant-section ok");
    expectTrue(!var.surface.weights.empty(), "rational variable sweep IS rational");
    double worst = 0.0;
    for (int iu = 0; iu <= 40; ++iu)
      for (double vk : var.vParams) {
        const double u = static_cast<double>(iu) / 40.0;
        worst = std::max(worst, distance(evalRatSurface(var.surface, u, vk),
                                         evalRatSurface(base.surface, u, vk)));
      }
    expectLE(worst, 1e-12,
             "rational constant-section: sweepRationalVariable == rational RMF sweep");
    std::printf("INFO rational constant-section worst dev = %.3e\n", worst);
  }

  // ═══ 2. LINEAR-TAPER CONE (analytic, exact rational frustum) ═════════════════
  // Rational circle radius R swept straight +Z with linear scale s(t)=s0+(s1−s0)*t.
  // Every station's placed section is the circle scaled about its center → radius s(t_k)*R.
  {
    const double R = 2.0, H = 6.0, s0 = 1.0, s1 = 3.0;
    const BsplineCurveData circ = ratFullCircleXY(R);
    const BsplineCurveData traj = straightTrajectory({0, 0, 0}, {0, 0, H});
    const Dir3 normal(0, 0, 1);
    const int stations = 12;
    std::vector<double> scales(stations), twists(stations, 0.0);
    for (int k = 0; k < stations; ++k) {
      const double t = static_cast<double>(k) / (stations - 1);
      scales[k] = s0 + (s1 - s0) * t;
    }

    SweepResult r = sweepRationalVariable(circ, traj, normal, scales, twists, stations, 3);
    expectTrue(r.ok, "sweepRationalVariable ok (linear taper)");
    expectTrue(!r.surface.weights.empty(), "tapered cone IS rational");
    expectTrue(r.surface.degreeU == 2, "cone U degree == circle degree 2");

    // (i) The surface CONTAINS each scaled rational circle at its station param: the circle
    //     scaled by scales[k] about its center (the origin), translated to z = t_k*H.
    double worstContain = 0.0, worstRad = 0.0;
    for (int k = 0; k < stations; ++k) {
      const double t = static_cast<double>(k) / (stations - 1);
      BsplineCurveData scaled = circ;  // keeps weights (similarity preserves them)
      for (Point3& p : scaled.poles) {
        p.x *= scales[k];
        p.y *= scales[k];
        p.z = t * H;  // section origin at z=0 → placed at station z
      }
      worstContain = std::max(worstContain, ratIsoVsCurveMaxDev(r.surface, r.vParams[k], scaled));
    }
    expectLE(worstContain, 1e-9,
             "linear-taper: surface contains each scaled rational circle (exact rational)");

    // (ii) Analytic frustum radius law: at station v_k every surface point is at radius
    //      scales[k]*R from the Z axis (true circle, not faceted).
    for (int k = 0; k < stations; ++k)
      for (int iu = 0; iu <= 48; ++iu) {
        const double u = static_cast<double>(iu) / 48.0;
        const Point3 p = evalRatSurface(r.surface, u, r.vParams[k]);
        worstRad = std::max(worstRad, std::fabs(std::hypot(p.x, p.y) - scales[k] * R));
      }
    expectLE(worstRad, 1e-9, "linear-taper: each station iso is a TRUE circle at radius s(t)*R");
    std::printf("INFO linear-taper cone contain=%.3e radius err=%.3e\n", worstContain, worstRad);
  }

  // Non-rational taper: the placed section is the polyline section scaled about the origin;
  // the surface contains it (skin containment). Straight spine ⇒ pure scale+translate.
  {
    const BsplineCurveData sec = sectionCubicXY();
    const Vec3 from{0, 0, 0}, to{0, 0, 5};
    const BsplineCurveData traj = straightTrajectory(from, to);
    const Dir3 normal(0, 0, 1);
    const int stations = 8;
    std::vector<double> scales(stations), twists(stations, 0.0);
    for (int k = 0; k < stations; ++k)
      scales[k] = 1.0 + 1.5 * (static_cast<double>(k) / (stations - 1));

    SweepResult r = sweepVariable(sec, traj, normal, scales, twists, stations, 3);
    expectTrue(r.ok, "sweepVariable ok (non-rational taper)");
    double worst = 0.0;
    for (int k = 0; k < stations; ++k) {
      const double t = static_cast<double>(k) / (stations - 1);
      BsplineCurveData scaled = sec;
      for (Point3& p : scaled.poles) {
        p.x *= scales[k];
        p.y *= scales[k];
        p.z = t * 5.0;
      }
      worst = std::max(worst, isoVsCurveMaxDev(r.surface, r.vParams[k], scaled));
    }
    expectLE(worst, 1e-8, "non-rational taper: surface contains each scaled section (containment)");
    std::printf("INFO non-rational taper containment worst dev = %.3e\n", worst);
  }

  // Twist sanity: a pure twist (scales≡1) about a straight +Z spine rotates each placed
  // section by twists[k] in its plane. Check the surface contains the rotated section.
  {
    const BsplineCurveData sec = sectionCubicXY();
    const BsplineCurveData traj = straightTrajectory({0, 0, 0}, {0, 0, 4});
    const Dir3 normal(0, 0, 1);
    const int stations = 8;
    std::vector<double> ones(stations, 1.0), twists(stations);
    for (int k = 0; k < stations; ++k)
      twists[k] = (PI / 2.0) * (static_cast<double>(k) / (stations - 1));  // 0..90°

    SweepResult r = sweepVariable(sec, traj, normal, ones, twists, stations, 3);
    expectTrue(r.ok, "sweepVariable ok (pure twist)");
    double worst = 0.0;
    for (int k = 0; k < stations; ++k) {
      const double t = static_cast<double>(k) / (stations - 1);
      const double c = std::cos(twists[k]), s = std::sin(twists[k]);
      BsplineCurveData rot = sec;
      for (Point3& p : rot.poles) {
        const double x = p.x, y = p.y;
        p.x = c * x - s * y;  // rotate about +Z (the section normal)
        p.y = s * x + c * y;
        p.z = t * 4.0;
      }
      worst = std::max(worst, isoVsCurveMaxDev(r.surface, r.vParams[k], rot));
    }
    expectLE(worst, 1e-8, "pure-twist: surface contains each rotated section (containment)");
    std::printf("INFO pure-twist containment worst dev = %.3e\n", worst);
  }

  // ═══ 3. TWO-RAIL SANITY ═════════════════════════════════════════════════════
  // (a) Two PARALLEL straight rails + a straight segment section → planar/ruled strip, and
  //     the section endpoints ride the rails exactly.
  {
    // Section: a straight segment from anchor0=(0,0,0) to anchor1=(1,0,0) in XY (normal +Z).
    BsplineCurveData seg;
    seg.degree = 1;
    seg.knots = {0, 0, 1, 1};
    seg.poles = {{0, 0, 0}, {1, 0, 0}};
    const Dir3 normal(0, 0, 1);

    // Parallel rails: rail0 along +Z at x=0, rail1 along +Z at x=2 (constant separation 2).
    const BsplineCurveData rail0 = straightRail({0, 0, 0}, {0, 0, 5});
    const BsplineCurveData rail1 = straightRail({2, 0, 0}, {2, 0, 5});

    const int stations = 10;
    SweepResult r = sweepTwoRail(seg, rail0, rail1, normal, 0, 1, stations, 3);
    expectTrue(r.ok, "sweepTwoRail ok (parallel rails)");
    expectTrue(r.surface.weights.empty(), "two-rail sweep non-rational");

    // Anchor iso-curves lie ON the rails: S(0,v) on rail0, S(1,v) on rail1 (anchors at u=0,1).
    double worstR0 = 0.0, worstR1 = 0.0;
    for (int k = 0; k < stations; ++k) {
      const double vk = r.vParams[k];
      const double t = static_cast<double>(k) / (stations - 1);
      const Point3 s0 = evalSurface(r.surface, 0.0, vk);
      const Point3 s1 = evalSurface(r.surface, 1.0, vk);
      worstR0 = std::max(worstR0, distance(s0, evalCurve(rail0, t)));
      worstR1 = std::max(worstR1, distance(s1, evalCurve(rail1, t)));
    }
    expectLE(worstR0, 1e-9, "parallel two-rail: anchor0 iso lies ON rail0 at every station");
    expectLE(worstR1, 1e-9, "parallel two-rail: anchor1 iso lies ON rail1 at every station");

    // The whole strip is PLANAR (y=0) and ruled: S(u,v) = (2u, 0, 5v). Match closed-form.
    double worst = 0.0;
    for (int iu = 0; iu <= 20; ++iu)
      for (int iv = 0; iv <= 20; ++iv) {
        const double u = static_cast<double>(iu) / 20.0;
        const double v = static_cast<double>(iv) / 20.0;
        const Point3 sp = evalSurface(r.surface, u, v);
        worst = std::max(worst, distance(sp, Point3{2.0 * u, 0.0, 5.0 * v}));
      }
    expectLE(worst, 1e-9, "parallel two-rail: planar ruled strip matches closed-form S=(2u,0,5v)");
    std::printf("INFO two-rail parallel rail-on err (%.3e,%.3e) strip err=%.3e\n", worstR0,
                worstR1, worst);
  }

  // (b) Two DIVERGING rails (linear taper): rail0 straight at x=0, rail1 fanning out so the
  //     separation grows linearly. The section endpoints still ride the rails exactly, and the
  //     strip is the exact tapered surface.
  {
    BsplineCurveData seg;
    seg.degree = 1;
    seg.knots = {0, 0, 1, 1};
    seg.poles = {{0, 0, 0}, {1, 0, 0}};  // unit anchor chord
    const Dir3 normal(0, 0, 1);

    // rail0: (0,0,0)→(0,0,6). rail1: (1,0,0)→(4,0,6). Separation grows 1→4 along Z.
    const BsplineCurveData rail0 = straightRail({0, 0, 0}, {0, 0, 6});
    const BsplineCurveData rail1 = straightRail({1, 0, 0}, {4, 0, 6});

    const int stations = 12;
    SweepResult r = sweepTwoRail(seg, rail0, rail1, normal, 0, 1, stations, 3);
    expectTrue(r.ok, "sweepTwoRail ok (diverging rails)");

    double worstR0 = 0.0, worstR1 = 0.0, worstStrip = 0.0;
    for (int k = 0; k < stations; ++k) {
      const double vk = r.vParams[k];
      const double t = static_cast<double>(k) / (stations - 1);
      worstR0 = std::max(worstR0, distance(evalSurface(r.surface, 0.0, vk), evalCurve(rail0, t)));
      worstR1 = std::max(worstR1, distance(evalSurface(r.surface, 1.0, vk), evalCurve(rail1, t)));
    }
    expectLE(worstR0, 1e-9, "diverging two-rail: anchor0 iso ON rail0 at every station");
    expectLE(worstR1, 1e-9, "diverging two-rail: anchor1 iso ON rail1 at every station");

    // Exact tapered surface: at station param v_k the strip interpolates rail0(t)→rail1(t)
    // linearly in u (the segment section is linear), y=0 throughout. Check on-station isos.
    for (int k = 0; k < stations; ++k) {
      const double vk = r.vParams[k];
      const double t = static_cast<double>(k) / (stations - 1);
      const Point3 A = evalCurve(rail0, t), B = evalCurve(rail1, t);
      for (int iu = 0; iu <= 20; ++iu) {
        const double u = static_cast<double>(iu) / 20.0;
        const Point3 sp = evalSurface(r.surface, u, vk);
        const Vec3 exp = A.asVec() + (B - A) * u;
        worstStrip = std::max(worstStrip, distance(sp, Point3{exp.x, exp.y, exp.z}));
      }
    }
    expectLE(worstStrip, 1e-9,
             "diverging two-rail: on-station iso == linear rail0(t)→rail1(t) (exact taper)");
    std::printf("INFO two-rail diverging rail-on err (%.3e,%.3e) taper err=%.3e\n", worstR0,
                worstR1, worstStrip);
  }

  // (c) Rational two-rail: a rational circle anchored between two diverging rails. The placed
  //     sections stay TRUE circles (weights preserved by the similarity); anchors ride rails.
  {
    const double R = 1.0;
    // Anchor the circle's +X pole (index 0) and −X pole (index 4) to the rails. Anchor chord
    // = (R,0,0)→(−R,0,0), length 2R, along −X. The rails span this diameter.
    const BsplineCurveData circ = ratFullCircleXY(R);
    const Dir3 normal(0, 0, 1);
    const BsplineCurveData rail0 = straightRail({R, 0, 0}, {R, 0, 5});         // rides pole 0 (+X)
    const BsplineCurveData rail1 = straightRail({-R, 0, 0}, {-3.0, 0, 5});     // rides pole 4 (−X)

    const int stations = 10;
    SweepResult r = sweepRationalTwoRail(circ, rail0, rail1, normal, 0, 4, stations, 3);
    expectTrue(r.ok, "sweepRationalTwoRail ok");
    expectTrue(!r.surface.weights.empty(), "rational two-rail IS rational");

    double worstR0 = 0.0, worstR1 = 0.0;
    for (int k = 0; k < stations; ++k) {
      const double vk = r.vParams[k];
      const double t = static_cast<double>(k) / (stations - 1);
      // Anchor pole 0 is at circle param u=0; pole 4 is at u=0.5 (the −X point).
      worstR0 = std::max(worstR0, distance(evalRatSurface(r.surface, 0.0, vk), evalCurve(rail0, t)));
      worstR1 = std::max(worstR1, distance(evalRatSurface(r.surface, 0.5, vk), evalCurve(rail1, t)));
    }
    expectLE(worstR0, 1e-8, "rational two-rail: anchor0 (+X pole) iso ON rail0");
    expectLE(worstR1, 1e-8, "rational two-rail: anchor1 (−X pole) iso ON rail1");
    std::printf("INFO rational two-rail rail-on err (%.3e,%.3e)\n", worstR0, worstR1);
  }

  // ═══ 4. DEGENERATE / DECLINE GUARDS ═════════════════════════════════════════
  {
    const BsplineCurveData sec = sectionCubicXY();
    const BsplineCurveData traj = straightTrajectory({0, 0, 0}, {0, 0, 5});
    const Dir3 normal(0, 0, 1);
    const int stations = 8;
    const std::vector<double> ones(stations, 1.0), zeros(stations, 0.0);

    // Variable: wrong-size scale/twist field → decline.
    expectTrue(!sweepVariable(sec, traj, normal, std::vector<double>(stations - 1, 1.0), zeros,
                              stations, 3)
                    .ok,
               "variable declines on wrong-size scale field");
    expectTrue(!sweepVariable(sec, traj, normal, ones, std::vector<double>(stations + 1, 0.0),
                              stations, 3)
                    .ok,
               "variable declines on wrong-size twist field");
    // Non-positive scale → decline.
    std::vector<double> badScale(stations, 1.0);
    badScale[3] = 0.0;
    expectTrue(!sweepVariable(sec, traj, normal, badScale, zeros, stations, 3).ok,
               "variable declines on non-positive scale");
    // < 2 stations → decline.
    expectTrue(!sweepVariable(sec, traj, normal, {1.0}, {0.0}, 1, 3).ok,
               "variable declines with <2 stations");
    // Rational section on the non-rational routine → decline.
    BsplineCurveData ratSec = sec;
    ratSec.weights.assign(ratSec.poles.size(), 1.0);
    expectTrue(!sweepVariable(ratSec, traj, normal, ones, zeros, stations, 3).ok,
               "variable declines on rational section");
    // Non-rational section on the rational routine → decline.
    expectTrue(!sweepRationalVariable(sec, traj, normal, ones, zeros, stations, 3).ok,
               "rational variable declines on non-rational section");
    // Coincident trajectory → decline.
    BsplineCurveData deadTraj = traj;
    for (Point3& p : deadTraj.poles) p = Point3{1, 1, 1};
    expectTrue(!sweepVariable(sec, deadTraj, normal, ones, zeros, stations, 3).ok,
               "variable declines on coincident trajectory");
  }

  // Two-rail degenerate guards.
  {
    BsplineCurveData seg;
    seg.degree = 1;
    seg.knots = {0, 0, 1, 1};
    seg.poles = {{0, 0, 0}, {1, 0, 0}};
    const Dir3 normal(0, 0, 1);
    const BsplineCurveData rail0 = straightRail({0, 0, 0}, {0, 0, 5});
    const BsplineCurveData rail1 = straightRail({2, 0, 0}, {2, 0, 5});

    // CROSSING rails (zero-length chord at a station): rail1 crosses rail0 at mid-height, so at
    // some station |rail1(t)−rail0(t)| ≈ 0 → undefined scale/orientation → honest-decline.
    const BsplineCurveData railX = straightRail({-2, 0, 0}, {2, 0, 5});  // crosses rail0's line
    expectTrue(!sweepTwoRail(seg, rail0, railX, normal, 0, 1, 9, 3).ok,
               "two-rail declines on crossing rails (zero chord at a station)");
    // Fully-coincident rails (identical) → every chord zero → decline.
    expectTrue(!sweepTwoRail(seg, rail0, rail0, normal, 0, 1, 8, 3).ok,
               "two-rail declines on coincident rails");

    // Bad anchor indices → decline.
    expectTrue(!sweepTwoRail(seg, rail0, rail1, normal, 0, 5, 8, 3).ok,
               "two-rail declines on out-of-range anchor");
    expectTrue(!sweepTwoRail(seg, rail0, rail1, normal, 1, 1, 8, 3).ok,
               "two-rail declines on equal anchors");

    // Coincident section anchors (both poles at the same point) → no chord → decline.
    BsplineCurveData deg = seg;
    deg.poles = {{0, 0, 0}, {0, 0, 0}};
    expectTrue(!sweepTwoRail(deg, rail0, rail1, normal, 0, 1, 8, 3).ok,
               "two-rail declines on coincident section anchors");

    // Rational section on the non-rational routine → decline; rational rail → decline.
    BsplineCurveData ratSeg = seg;
    ratSeg.weights.assign(ratSeg.poles.size(), 1.0);
    expectTrue(!sweepTwoRail(ratSeg, rail0, rail1, normal, 0, 1, 8, 3).ok,
               "two-rail declines on rational section");
    BsplineCurveData ratRail = rail0;
    ratRail.weights.assign(ratRail.poles.size(), 1.0);
    expectTrue(!sweepTwoRail(seg, ratRail, rail1, normal, 0, 1, 8, 3).ok,
               "two-rail declines on rational rail");
    // < 2 stations → decline.
    expectTrue(!sweepTwoRail(seg, rail0, rail1, normal, 0, 1, 1, 3).ok,
               "two-rail declines with <2 stations");
  }

  // ── report ──
  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_vsweep: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_vsweep: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_nurbs_vsweep (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
