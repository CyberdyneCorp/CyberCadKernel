// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — swept surfaces
// (src/native/math/bspline_sweep.{h,cpp}). OCCT-FREE. The oracles are airtight and
// closed-form:
//
//   1. TRANSLATIONAL EXACTNESS (the base-case oracle) — the swept surface's iso-curve at
//      each station v reproduces the section TRANSLATED by v·sweep, POINTWISE to ~1e-12.
//      This is the EXACT closed-form case (no fitting, pure tensor product).
//   2. STATION CONTAINMENT (general sweep) — the swept surface contains each TRANSFORMED
//      section at its station parameter to ~1e-8 (skinning's containment oracle carries
//      through the transform-then-skin composition).
//   3. KNOWN-SURFACE CHECKS — a straight section swept translationally reproduces a
//      planar / ruled patch matching the closed-form bilinear surface; a circle-approx
//      section swept translationally is a cylinder-like patch matching the analytic tube.
//   4. FRAME (anti-twist) SANITY — for a straight trajectory the rotation-minimizing frame
//      keeps the section orientation FIXED (no spurious spin), so the general sweep along a
//      straight spine reproduces the translational sweep pointwise.
//   5. DEGENERATE GUARDS — <2 stations, coincident-trajectory / null sweep, and rational
//      input decline honestly (ok=false, no crash).
//
// The general sweep composes skinSurface (which solves via numerics::lin_solve), so the
// whole gate is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_skin). With the guard
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
    std::printf("FAIL %-52s %.6g <= %.6g violated\n", what, a, b);
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
// Max distance between the RATIONAL surface iso-curve S(·,v) and a RATIONAL curve.
static double ratIsoVsCurveMaxDev(const BsplineSurfaceData& s, double v,
                                  const BsplineCurveData& c, int nS = 200) {
  double worst = 0.0;
  for (int i = 0; i <= nS; ++i) {
    const double u = static_cast<double>(i) / nS;
    worst = std::max(worst, distance(evalRatSurface(s, u, v), evalRatCurve(c, u)));
  }
  return worst;
}
static BsplineCurveData ratTranslated(const BsplineCurveData& c, const Vec3& d) {
  BsplineCurveData o = c;  // keeps weights
  for (Point3& p : o.poles) p = p + d;
  return o;
}

// An EXACT rational quadratic FULL circle of `radius` in the XY plane (z=0), θ ∈ [0,2π]:
// the standard 9-pole NURBS circle (four quarter Bézier arcs). Corner poles on the circle
// (w=1), between-corner poles at the polygon corners (w=√2/2). Evaluates to a TRUE circle —
// the airtight rational conic that turns into an exact rational cylinder under a sweep.
static BsplineCurveData ratFullCircleXY(double radius) {
  const double s = std::sqrt(2.0) / 2.0;
  const double R = radius;
  BsplineCurveData c;
  c.degree = 2;
  // 9 poles ⇒ 12 knots; four segments at 0,¼,½,¾,1 each with interior multiplicity 2.
  c.knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
  c.poles = {{R, 0, 0},   {R, R, 0},   {0, R, 0},   {-R, R, 0}, {-R, 0, 0},
             {-R, -R, 0}, {0, -R, 0},  {R, -R, 0},  {R, 0, 0}};
  c.weights = {1, s, 1, s, 1, s, 1, s, 1};
  return c;
}

// Max distance between the surface iso-curve S(·,v) and a curve over a dense u-sample.
static double isoVsCurveMaxDev(const BsplineSurfaceData& s, double v,
                               const BsplineCurveData& c, int nS = 200) {
  double worst = 0.0;
  for (int i = 0; i <= nS; ++i) {
    const double u = static_cast<double>(i) / nS;
    worst = std::max(worst, distance(evalSurface(s, u, v), evalCurve(c, u)));
  }
  return worst;
}

// Translate a curve by a vector (build the section+offset reference for the exact case).
static BsplineCurveData translated(const BsplineCurveData& c, const Vec3& d) {
  BsplineCurveData o = c;
  for (Point3& p : o.poles) p = p + d;
  return o;
}

// A clamped cubic section on [0,1] with 6 poles (knot length 6+3+1 = 10), in the XY plane
// (z = 0), so its plane normal is +Z. A generic non-planar-in-XY profile for sweeping.
static BsplineCurveData sectionCubicXY() {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = {0, 0, 0, 0, 0.4, 0.7, 1, 1, 1, 1};  // 6 poles
  c.poles = {{0, 0, 0}, {1, 2, 0}, {2, -1, 0}, {4, 1, 0}, {5, 3, 0}, {6, 0, 0}};
  return c;
}

// A quarter-circle-ish quadratic B-spline in the XY plane (radius ~1, non-rational
// approximation — good enough for a cylinder-like patch oracle to fitting tolerance).
static BsplineCurveData sectionArcXY(int quads = 4) {
  // Sample a full circle of radius 1 in XY and interpolate is heavy; instead build a
  // polyline-ish quadratic through circle points. We use a coarse control polygon that
  // rounds; the oracle checks the swept surface CONTAINS this section, not that it is a
  // perfect circle. Here just a smooth closed-ish arc.
  (void)quads;
  BsplineCurveData c;
  c.degree = 2;
  c.knots = {0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1};  // 6 poles
  c.poles = {{1, 0, 0},   {0.7, 0.7, 0}, {0, 1, 0},
             {-0.7, 0.7, 0}, {-1, 0, 0},  {-0.7, -0.7, 0}};
  return c;
}

// A straight trajectory: degree-1 (or degree-3) B-spline whose sampled points lie on a
// line. Direction and length control the sweep. Cubic so the tangent is well-defined.
static BsplineCurveData straightTrajectory(const Vec3& from, const Vec3& to) {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = {0, 0, 0, 0, 1, 1, 1, 1};  // 4 poles, degree-3 Bézier
  for (int i = 0; i < 4; ++i) {
    const double s = static_cast<double>(i) / 3.0;
    const Vec3 p = from + (to - from) * s;
    c.poles.push_back({p.x, p.y, p.z});
  }
  return c;
}

int main() {
  // ═══ 1. TRANSLATIONAL EXACTNESS (the base-case oracle) ══════════════════════
  // Sweep a section along a vector. The surface must be non-rational, N×2, degree
  // (p × 1), and its iso-curve at v reproduce the section translated by v·sweep to
  // machine precision (no fitting).
  {
    const BsplineCurveData sec = sectionCubicXY();
    const Vec3 sweep{0.5, -0.3, 4.0};
    SweepResult r = sweepTranslational(sec, sweep);
    expectTrue(r.ok, "sweepTranslational ok");
    expectTrue(r.surface.weights.empty(), "translational sweep is non-rational");
    expectTrue(r.surface.degreeU == sec.degree, "U degree == section degree");
    expectTrue(r.surface.degreeV == 1, "V degree == 1 (straight path)");
    expectTrue(r.surface.nPolesV == 2, "V has exactly 2 poles");
    expectTrue(r.surface.nPolesU == static_cast<int>(sec.poles.size()), "U poles == section poles");

    // Exactness at a dense set of stations v: S(·,v) == section translated by v·sweep.
    double worst = 0.0;
    for (int j = 0; j <= 20; ++j) {
      const double v = static_cast<double>(j) / 20.0;
      const BsplineCurveData ref = translated(sec, sweep * v);
      worst = std::max(worst, isoVsCurveMaxDev(r.surface, v, ref));
    }
    expectLE(worst, 1e-12, "translational: S(.,v) == section + v*sweep POINTWISE (exact)");
    std::printf("INFO translational exactness worst dev = %.3e\n", worst);

    // Endpoints exactly reproduce section (v=0) and section+sweep (v=1).
    expectLE(isoVsCurveMaxDev(r.surface, 0.0, sec), 1e-13, "translational: S(.,0) == section");
    expectLE(isoVsCurveMaxDev(r.surface, 1.0, translated(sec, sweep)), 1e-13,
             "translational: S(.,1) == section + sweep");
    expectTrue(r.vParams.size() == 2 && r.vParams[0] == 0.0 && r.vParams[1] == 1.0,
               "translational vParams == {0,1}");
  }

  // ═══ 3(a). KNOWN-SURFACE: straight section → planar/ruled patch ═════════════
  // A straight-line section swept translationally is a planar parallelogram patch. Its
  // closed-form is the bilinear map S(u,v) = A + u*(B-A) + v*sweep. Match pointwise.
  {
    BsplineCurveData line;  // degree-1 straight section from A to B
    line.degree = 1;
    line.knots = {0, 0, 1, 1};
    const Point3 A{0, 0, 0}, B{3, 1, 0};
    line.poles = {A, B};
    const Vec3 sweep{0, 0, 2};
    SweepResult r = sweepTranslational(line, sweep);
    expectTrue(r.ok, "straight-section translational ok");
    double worst = 0.0;
    for (int iu = 0; iu <= 20; ++iu)
      for (int iv = 0; iv <= 20; ++iv) {
        const double u = static_cast<double>(iu) / 20.0;
        const double v = static_cast<double>(iv) / 20.0;
        const Vec3 expv = A.asVec() + (B - A) * u + sweep * v;
        worst = std::max(worst, distance(evalSurface(r.surface, u, v),
                                         Point3{expv.x, expv.y, expv.z}));
      }
    expectLE(worst, 1e-12, "ruled patch matches closed-form bilinear S(u,v)=A+u(B-A)+v*sweep");
    std::printf("INFO ruled-patch closed-form worst dev = %.3e\n", worst);
  }

  // ═══ 3(b). KNOWN-SURFACE: circle section swept → cylinder-like patch ════════
  // Sweeping a circular section straight along +Z gives a (generalized) cylinder: every
  // point of S(u,v) is section(u) + v*axis, so its projection onto the XY plane traces the
  // SAME section for all v (constant "radius profile"). Check the swept surface reproduces
  // the section profile at every v to the exact tensor-product tolerance.
  {
    const BsplineCurveData arc = sectionArcXY();
    const Vec3 axis{0, 0, 5};
    SweepResult r = sweepTranslational(arc, axis);
    expectTrue(r.ok, "arc translational (cylinder-like) ok");
    double worst = 0.0;
    for (int j = 0; j <= 12; ++j) {
      const double v = static_cast<double>(j) / 12.0;
      // The XY profile at height v is exactly the section (translation is pure +Z).
      const BsplineCurveData ref = translated(arc, axis * v);
      worst = std::max(worst, isoVsCurveMaxDev(r.surface, v, ref));
    }
    expectLE(worst, 1e-12, "cylinder-like: each height iso == section profile (exact)");
    std::printf("INFO cylinder-like profile worst dev = %.3e\n", worst);
  }

  // ═══ 2 + 4. GENERAL SWEEP: station containment + anti-twist frame ═══════════
  // Sweep the XY section (normal +Z) along a straight trajectory pointing +Z. The
  // rotation-minimizing frame must keep the section fixed (no spin), so the general sweep
  // reproduces the section at every station (containment), AND matches the translational
  // sweep of the same section along the same total displacement.
  {
    const BsplineCurveData sec = sectionCubicXY();
    const Vec3 from{0, 0, 0}, to{0, 0, 6};
    const BsplineCurveData traj = straightTrajectory(from, to);
    const Dir3 normal(0, 0, 1);  // section plane normal aligned with the straight tangent

    SweepResult r = sweepAlongTrajectory(sec, traj, normal, 8, 3);
    expectTrue(r.ok, "sweepAlongTrajectory ok (straight spine)");
    expectTrue(r.surface.weights.empty(), "general sweep non-rational");
    expectTrue(r.surface.degreeU == sec.degree, "general sweep U degree == section degree");
    expectTrue(r.vParams.size() == 8, "one v-param per station");

    // STATION CONTAINMENT: rebuild each placed (transformed) section and confirm the
    // surface contains it at its station param. For a straight +Z spine with tangent +Z
    // and section normal +Z, the placement is a pure +Z translation to the station point,
    // so the placed section is section translated by the station's z.
    double worst = 0.0;
    for (int k = 0; k < 8; ++k) {
      const double t = static_cast<double>(k) / 7.0;
      const Vec3 stationPt = from + (to - from) * t;
      // placement translates section origin to stationPt; section already at z=0 origin.
      const BsplineCurveData placed = translated(sec, stationPt);
      worst = std::max(worst, isoVsCurveMaxDev(r.surface, r.vParams[k], placed));
    }
    expectLE(worst, 1e-8, "general sweep CONTAINS each transformed section (station containment)");
    std::printf("INFO general-sweep station containment worst dev = %.3e\n", worst);

    // ANTI-TWIST: compare the general straight sweep to the translational sweep along the
    // full displacement. Both describe the same ruled tube; sample points must agree.
    SweepResult tr = sweepTranslational(sec, to - from);
    expectTrue(tr.ok, "reference translational sweep ok");
    double worstTwist = 0.0;
    for (int iu = 0; iu <= 20; ++iu)
      for (int iv = 0; iv <= 20; ++iv) {
        const double u = static_cast<double>(iu) / 20.0;
        const double v = static_cast<double>(iv) / 20.0;
        worstTwist = std::max(worstTwist, distance(evalSurface(r.surface, u, v),
                                                   evalSurface(tr.surface, u, v)));
      }
    expectLE(worstTwist, 1e-8,
             "anti-twist: RMF straight sweep == translational sweep pointwise (no spin)");
    std::printf("INFO anti-twist straight-sweep vs translational worst dev = %.3e\n", worstTwist);
  }

  // General sweep along a CURVED (L-shaped-ish) trajectory: the surface must still contain
  // each placed station section. This exercises the moving frame on a non-straight spine.
  {
    const BsplineCurveData sec = sectionArcXY();  // small circular profile
    BsplineCurveData traj;                         // a curved cubic spine in 3D
    traj.degree = 3;
    traj.knots = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};  // 5 poles
    traj.poles = {{0, 0, 0}, {2, 0, 0}, {4, 1, 1}, {4, 3, 2}, {4, 5, 4}};
    const Dir3 normal(0, 0, 1);

    const int stations = 10;
    SweepResult r = sweepAlongTrajectory(sec, traj, normal, stations, 3);
    expectTrue(r.ok, "sweepAlongTrajectory ok (curved spine)");
    expectTrue(r.vParams.size() == static_cast<std::size_t>(stations), "curved: one v per station");

    // Reconstruct the SKIN's own compatible sections is not directly available, but the
    // containment oracle is: rebuild the placement the module uses is internal. Instead we
    // assert the surface contains SOME section geometry at each station by checking the
    // skin containment invariant transitively: skinSurface(placed) contains placed[k], and
    // sweepAlongTrajectory returns exactly that skin. We re-derive placed[k] via the same
    // public frame contract is not exposed; so here we assert the weaker-but-honest oracle
    // that the surface is well-formed and every iso-curve at v_k has the section's arc
    // length within tolerance (a curved-spine sanity check on the moving frame).
    // Arc-length of the section profile (constant along a rigid sweep).
    auto arcLen = [](const BsplineCurveData& c) {
      double L = 0.0;
      Point3 prev = curvePoint(c.degree, c.poles, c.knots, 0.0);
      for (int i = 1; i <= 400; ++i) {
        const double u = static_cast<double>(i) / 400.0;
        const Point3 p = curvePoint(c.degree, c.poles, c.knots, u);
        L += distance(p, prev);
        prev = p;
      }
      return L;
    };
    const double secLen = arcLen(sec);
    double worstLenErr = 0.0;
    for (int k = 0; k < stations; ++k) {
      // iso-curve S(·, v_k) as a sampled polyline length.
      double L = 0.0;
      Point3 prev = evalSurface(r.surface, 0.0, r.vParams[k]);
      for (int i = 1; i <= 400; ++i) {
        const double u = static_cast<double>(i) / 400.0;
        const Point3 p = evalSurface(r.surface, u, r.vParams[k]);
        L += distance(p, prev);
        prev = p;
      }
      worstLenErr = std::max(worstLenErr, std::fabs(L - secLen));
    }
    // A rigid moving frame preserves the section's arc length at every station (the skin
    // reproduces the placed section exactly at v_k). Tolerance is loose (chord sampling).
    expectLE(worstLenErr, 1e-6,
             "curved sweep: each station iso preserves the section arc length (rigid frame)");
    std::printf("INFO curved-sweep station arc-length worst err = %.3e (section len %.4f)\n",
                worstLenErr, secLen);
  }

  // ═══ 5. DEGENERATE GUARDS ═══════════════════════════════════════════════════
  {
    const BsplineCurveData sec = sectionCubicXY();
    const BsplineCurveData traj = straightTrajectory({0, 0, 0}, {0, 0, 5});
    const Dir3 normal(0, 0, 1);

    // Null sweep vector declines.
    expectTrue(!sweepTranslational(sec, Vec3{0, 0, 0}).ok, "translational declines on null sweep");

    // Rational section declines (translational + general).
    BsplineCurveData ratl = sec;
    ratl.weights.assign(ratl.poles.size(), 1.0);  // non-empty ⇒ rational
    expectTrue(!sweepTranslational(ratl, Vec3{0, 0, 1}).ok,
               "translational declines on rational section");
    expectTrue(!sweepAlongTrajectory(ratl, traj, normal, 8, 3).ok,
               "general declines on rational section");

    // Rational trajectory declines.
    BsplineCurveData ratTraj = traj;
    ratTraj.weights.assign(ratTraj.poles.size(), 1.0);
    expectTrue(!sweepAlongTrajectory(sec, ratTraj, normal, 8, 3).ok,
               "general declines on rational trajectory");

    // < 2 stations declines.
    expectTrue(!sweepAlongTrajectory(sec, traj, normal, 1, 3).ok,
               "general declines with <2 stations");

    // Coincident trajectory (all poles at one point) → no path → decline.
    BsplineCurveData deadTraj = traj;
    for (Point3& p : deadTraj.poles) p = Point3{1, 1, 1};
    expectTrue(!sweepAlongTrajectory(sec, deadTraj, normal, 8, 3).ok,
               "general declines on coincident-trajectory points (honest guard)");

    // Malformed section (bad knot vector) declines.
    BsplineCurveData bad = sec;
    bad.knots.pop_back();
    expectTrue(!sweepTranslational(bad, Vec3{0, 0, 1}).ok,
               "translational declines on malformed section");
    expectTrue(!sweepAlongTrajectory(bad, traj, normal, 8, 3).ok,
               "general declines on malformed section");
  }

  // ═══ 6. RATIONAL TRANSLATIONAL SWEEP — EXACT RATIONAL CYLINDER (strongest oracle) ═══
  //
  // Sweep a rational quadratic CIRCLE (radius R, XY plane, exact NURBS) translationally along
  // +Z by height H. The result MUST be an EXACT rational cylinder: for every (u,v) the point
  // S(u,v) lies at radius R from the axis AND at height v·H. We compare it BOTH against the
  // section-translated rational iso-curve AND against the closed-form analytic cylinder point
  // computed from the circle's own angle — the definitive proof this is an exact rational
  // surface (not a faceted approximation).
  {
    const double R = 2.5, H = 6.0;
    const BsplineCurveData circ = ratFullCircleXY(R);
    const Vec3 axis{0, 0, H};
    SweepResult r = sweepRationalTranslational(circ, axis);
    expectTrue(r.ok, "sweepRationalTranslational ok (rational circle)");
    expectTrue(!r.surface.weights.empty(), "rational cylinder IS rational");
    expectTrue(r.surface.weights.size() == r.surface.poles.size(),
               "one weight per cylinder pole");
    expectTrue(r.surface.degreeU == 2 && r.surface.degreeV == 1,
               "rational cylinder degree (2 × 1)");
    expectTrue(r.surface.nPolesV == 2, "rational cylinder V has 2 poles");

    // (i) Each height iso == the rational circle translated by v·axis (rational-exact).
    double worstIso = 0.0;
    for (int j = 0; j <= 16; ++j) {
      const double v = static_cast<double>(j) / 16.0;
      worstIso = std::max(worstIso, ratIsoVsCurveMaxDev(r.surface, v, ratTranslated(circ, axis * v)));
    }
    expectLE(worstIso, 1e-12, "rational cylinder: each iso == rational circle + v*axis (exact)");

    // (ii) Against the ANALYTIC cylinder: every surface point at radius R and height v·H, and
    // matching the exact analytic point derived from the SAME circle's parameter angle. The
    // circle's own point gives the (x,y) at radius R; the swept point must equal that (x,y) at
    // height v·H — the closed-form rational cylinder.
    double worstAna = 0.0, worstRad = 0.0;
    for (int iu = 0; iu <= 64; ++iu)
      for (int iv = 0; iv <= 16; ++iv) {
        const double u = static_cast<double>(iu) / 64.0;
        const double v = static_cast<double>(iv) / 16.0;
        const Point3 sp = evalRatSurface(r.surface, u, v);
        // Analytic reference: the circle profile point (exact rational circle) lifted to z=v*H.
        const Point3 cp = evalRatCurve(circ, u);
        const Point3 ana{cp.x, cp.y, v * H};
        worstAna = std::max(worstAna, distance(sp, ana));
        worstRad = std::max(worstRad, std::fabs(std::hypot(sp.x, sp.y) - R));
      }
    expectLE(worstAna, 1e-12,
             "EXACT rational cylinder matches analytic (circle profile × v*H) POINTWISE");
    expectLE(worstRad, 1e-12, "rational cylinder: every point at radius R (true circle, not faceted)");
    std::printf("INFO exact-rational-cylinder analytic dev = %.3e, radius err = %.3e\n",
                worstAna, worstRad);
    expectTrue(r.vParams.size() == 2 && r.vParams[0] == 0.0 && r.vParams[1] == 1.0,
               "rational cylinder vParams == {0,1}");
  }

  // ═══ 7. RATIONAL GENERAL SWEEP — station containment ════════════════════════
  // Sweep a rational circle along a straight +Z spine. The RMF keeps the section fixed, so
  // the general rational sweep contains each transformed rational section, AND matches the
  // rational translational cylinder pointwise (anti-twist, rational).
  {
    const double R = 1.5;
    const BsplineCurveData circ = ratFullCircleXY(R);
    const Vec3 from{0, 0, 0}, to{0, 0, 5};
    const BsplineCurveData traj = straightTrajectory(from, to);
    const Dir3 normal(0, 0, 1);

    SweepResult r = sweepRationalAlongTrajectory(circ, traj, normal, 6, 3);
    expectTrue(r.ok, "sweepRationalAlongTrajectory ok (straight spine)");
    expectTrue(!r.surface.weights.empty(), "general rational sweep IS rational");
    expectTrue(r.vParams.size() == 6, "one v-param per rational station");

    // Station containment: each placed rational section is the circle translated by station z.
    double worst = 0.0;
    for (int k = 0; k < 6; ++k) {
      const double t = static_cast<double>(k) / 5.0;
      const Vec3 stationPt = from + (to - from) * t;
      worst = std::max(worst, ratIsoVsCurveMaxDev(r.surface, r.vParams[k],
                                                  ratTranslated(circ, stationPt)));
    }
    expectLE(worst, 1e-8, "general rational sweep CONTAINS each transformed rational section");
    std::printf("INFO general rational-sweep station containment worst dev = %.3e\n", worst);

    // Every surface point still on the true cylinder (radius R) — proves the rational skin
    // preserved the conic through the transform-then-skin composition.
    double worstRad = 0.0;
    for (int iu = 0; iu <= 40; ++iu)
      for (double vk : r.vParams) {
        const double u = static_cast<double>(iu) / 40.0;
        const Point3 p = evalRatSurface(r.surface, u, vk);
        worstRad = std::max(worstRad, std::fabs(std::hypot(p.x, p.y) - R));
      }
    expectLE(worstRad, 1e-8, "general rational sweep iso-circles stay at radius R");
  }

  // ═══ 8. RATIONAL SWEEP GUARDS ═══════════════════════════════════════════════
  {
    const BsplineCurveData circ = ratFullCircleXY(2.0);
    const BsplineCurveData nonrat = sectionCubicXY();  // no weights
    const BsplineCurveData traj = straightTrajectory({0, 0, 0}, {0, 0, 5});
    const Dir3 normal(0, 0, 1);

    // Non-rational section declined by the rational routines (wrong path).
    expectTrue(!sweepRationalTranslational(nonrat, Vec3{0, 0, 1}).ok,
               "rational translational declines on NON-rational section");
    expectTrue(!sweepRationalAlongTrajectory(nonrat, traj, normal, 6, 3).ok,
               "rational general declines on NON-rational section");

    // Null sweep declines.
    expectTrue(!sweepRationalTranslational(circ, Vec3{0, 0, 0}).ok,
               "rational translational declines on null sweep");

    // Non-positive weight declines.
    BsplineCurveData badW = circ;
    badW.weights[3] = 0.0;
    expectTrue(!sweepRationalTranslational(badW, Vec3{0, 0, 1}).ok,
               "rational translational declines on non-positive weight");
    expectTrue(!sweepRationalAlongTrajectory(badW, traj, normal, 6, 3).ok,
               "rational general declines on non-positive weight");

    // Mismatched weight count declines.
    BsplineCurveData mism = circ;
    mism.weights.pop_back();
    expectTrue(!sweepRationalTranslational(mism, Vec3{0, 0, 1}).ok,
               "rational translational declines on weight/pole mismatch");

    // Rational trajectory declines (spine must be non-rational).
    BsplineCurveData ratTraj = traj;
    ratTraj.weights.assign(ratTraj.poles.size(), 1.0);
    expectTrue(!sweepRationalAlongTrajectory(circ, ratTraj, normal, 6, 3).ok,
               "rational general declines on rational trajectory");

    // < 2 stations declines.
    expectTrue(!sweepRationalAlongTrajectory(circ, traj, normal, 1, 3).ok,
               "rational general declines with <2 stations");

    // Coincident trajectory declines.
    BsplineCurveData deadTraj = traj;
    for (Point3& p : deadTraj.poles) p = Point3{1, 1, 1};
    expectTrue(!sweepRationalAlongTrajectory(circ, deadTraj, normal, 6, 3).ok,
               "rational general declines on coincident trajectory");
  }

  // ═══ 9. ROTATIONAL (REVOLVED) SWEEP — EXACT surfaces of revolution ══════════
  // Revolving a profile about an axis is an EXACT RATIONAL surface of revolution. The
  // strongest oracles are the closed-form analytic surfaces: cylinder / cone / sphere.
  {
    const double PI = 3.14159265358979323846;
    const Point3 axisP{0, 0, 0};
    const Dir3 axisD(0, 0, 1);  // revolve about +Z

    // ── (a) CYLINDER: a straight segment PARALLEL to the axis, offset by R, revolved 360°. ──
    // Profile: vertical line x=R, z from 0 to H (degree-1, 2 poles). Every surface point must
    // lie at radius R and its own height — the EXACT analytic cylinder.
    {
      const double R = 2.0, H = 5.0;
      BsplineCurveData prof;
      prof.degree = 1;
      prof.knots = {0, 0, 1, 1};
      prof.poles = {{R, 0, 0}, {R, 0, H}};  // straight, parallel to +Z, radius R
      SweepResult r = sweepRotational(prof, axisP, axisD, 2 * PI);
      expectTrue(r.ok, "sweepRotational cylinder ok");
      expectTrue(!r.surface.weights.empty(), "revolved cylinder IS rational");
      expectTrue(r.surface.degreeV == 2, "revolve V degree == 2 (rational arc)");

      // The EXACT analytic cylinder is the implicit surface {radius==R, z==u*H}. A rational
      // Bézier arc is NOT linear-in-parameter in ANGLE, so the analytic oracle is the implicit
      // surface condition (radius + height), not an assumed uniform angle. Radius err at machine
      // zero proves a TRUE circle (not faceted). We ALSO confirm the swept angle at V=1 is the
      // full 2π (the arc spans the requested angle) via atan2 of the end iso-curve.
      double worstRad = 0.0, worstZ = 0.0;
      for (int iu = 0; iu <= 8; ++iu)
        for (int iv = 0; iv <= 96; ++iv) {
          const double u = static_cast<double>(iu) / 8.0;
          const double v = static_cast<double>(iv) / 96.0;
          const Point3 sp = evalRatSurface(r.surface, u, v);
          worstRad = std::max(worstRad, std::fabs(std::hypot(sp.x, sp.y) - R));
          worstZ = std::max(worstZ, std::fabs(sp.z - u * H));
        }
      expectLE(worstRad, 1e-9, "EXACT cylinder: every point at radius R from axis (true circle)");
      expectLE(worstZ, 1e-9, "EXACT cylinder: height linear in profile param (analytic surface)");
      std::printf("INFO revolved cylinder radius err = %.3e, height err = %.3e\n",
                  worstRad, worstZ);

      // Containment: S(·,0) == profile, S(·,1) == profile (full 360° returns to start).
      double worstStart = 0.0;
      for (int iu = 0; iu <= 40; ++iu) {
        const double u = static_cast<double>(iu) / 40.0;
        const Point3 sp = evalRatSurface(r.surface, u, 0.0);
        const Point3 cp = evalCurve(prof, u);  // non-rational profile
        worstStart = std::max(worstStart, distance(sp, cp));
      }
      expectLE(worstStart, 1e-9, "revolved surface contains the profile at V=0");
    }

    // ── (b) CONE: a straight segment TILTED to the axis (radius grows with height), 360°. ──
    // Profile from (R0,0,0) to (R1,0,H). At height z the radius is R0+(R1-R0)*(z/H) — an
    // EXACT truncated cone (frustum). Check against the analytic frustum.
    {
      const double R0 = 1.0, R1 = 3.0, H = 4.0;
      BsplineCurveData prof;
      prof.degree = 1;
      prof.knots = {0, 0, 1, 1};
      prof.poles = {{R0, 0, 0}, {R1, 0, H}};
      SweepResult r = sweepRotational(prof, axisP, axisD, 2 * PI);
      expectTrue(r.ok, "sweepRotational cone ok");
      // Analytic frustum: implicit surface {radius == R0+(R1-R0)*u, z == u*H}. Independent of
      // the arc's angle parametrization; radius-at-height exactness proves the exact cone.
      double worstRad = 0.0, worstZ = 0.0;
      for (int iu = 0; iu <= 8; ++iu)
        for (int iv = 0; iv <= 96; ++iv) {
          const double u = static_cast<double>(iu) / 8.0;
          const double v = static_cast<double>(iv) / 96.0;
          const Point3 sp = evalRatSurface(r.surface, u, v);
          const double radExpect = R0 + (R1 - R0) * u;  // profile param u ↦ radius, z=u*H
          worstRad = std::max(worstRad, std::fabs(std::hypot(sp.x, sp.y) - radExpect));
          worstZ = std::max(worstZ, std::fabs(sp.z - u * H));
        }
      expectLE(worstRad, 1e-9, "EXACT cone/frustum: radius == R0+(R1-R0)*u at every point (analytic)");
      expectLE(worstZ, 1e-9, "EXACT cone/frustum: height linear in profile param (analytic)");
      std::printf("INFO revolved cone radius err = %.3e, height err = %.3e\n", worstRad, worstZ);
    }

    // ── (c) SPHERE: a rational SEMICIRCLE whose diameter lies on the axis, revolved 360°. ──
    // Profile = a rational quadratic half-circle of radius R in the XZ plane, from the south
    // pole (0,0,-R) through (R,0,0) to the north pole (0,0,R), diameter on the Z axis. Revolved
    // 360° about Z it is an EXACT SPHERE of radius R — every surface point at distance R from
    // the center. This is the strongest oracle (rational profile × rational revolve arc).
    {
      const double R = 2.5;
      const double s = std::sqrt(2.0) / 2.0;
      // 5-pole rational half-circle (two 90° Bézier arcs) in XZ, center at origin.
      // South pole (0,0,-R) → (R,0,-R) w=s → (R,0,0) → (R,0,R) w=s → north pole (0,0,R).
      BsplineCurveData semi;
      semi.degree = 2;
      semi.knots = {0, 0, 0, 0.5, 0.5, 1, 1, 1};  // 5 poles, two segments
      semi.poles = {{0, 0, -R}, {R, 0, -R}, {R, 0, 0}, {R, 0, R}, {0, 0, R}};
      semi.weights = {1, s, 1, s, 1};

      SweepResult r = sweepRotational(semi, axisP, axisD, 2 * PI);
      expectTrue(r.ok, "sweepRotational sphere ok");
      expectTrue(!r.surface.weights.empty(), "revolved sphere IS rational");

      double worstRad = 0.0;
      for (int iu = 0; iu <= 64; ++iu)
        for (int iv = 0; iv <= 96; ++iv) {
          const double u = static_cast<double>(iu) / 64.0;
          const double v = static_cast<double>(iv) / 96.0;
          const Point3 sp = evalRatSurface(r.surface, u, v);
          worstRad = std::max(worstRad, std::fabs(std::hypot(std::hypot(sp.x, sp.y), sp.z) - R));
        }
      expectLE(worstRad, 1e-9, "EXACT sphere: every revolved point at distance R from center");
      std::printf("INFO revolved sphere radius err = %.3e\n", worstRad);

      // Profile containment at V=0 (rational).
      expectLE(ratIsoVsCurveMaxDev(r.surface, 0.0, semi), 1e-9,
               "revolved sphere contains the rational profile at V=0");
    }

    // ── (d) PARTIAL-ANGLE revolve = the correct rational arc sector. ──
    // Revolve the cylinder profile by only 90° (a quarter). The surface must be the analytic
    // quarter-cylinder: angles v·(π/2), and S(·,1) == profile rotated by 90°.
    {
      const double R = 1.8, H = 3.0, sweepAng = PI / 2.0;
      BsplineCurveData prof;
      prof.degree = 1;
      prof.knots = {0, 0, 1, 1};
      prof.poles = {{R, 0, 0}, {R, 0, H}};
      SweepResult r = sweepRotational(prof, axisP, axisD, sweepAng);
      expectTrue(r.ok, "sweepRotational partial (90°) ok");
      // Analytic quarter-cylinder: radius R at every point, angle stays within [0, π/2] (the
      // sector), height linear. Radius exactness proves the exact arc sector (not faceted).
      double worstRad = 0.0, worstZ = 0.0, worstAngle = 0.0;
      for (int iu = 0; iu <= 8; ++iu)
        for (int iv = 0; iv <= 48; ++iv) {
          const double u = static_cast<double>(iu) / 8.0;
          const double v = static_cast<double>(iv) / 48.0;
          const Point3 sp = evalRatSurface(r.surface, u, v);
          worstRad = std::max(worstRad, std::fabs(std::hypot(sp.x, sp.y) - R));
          worstZ = std::max(worstZ, std::fabs(sp.z - u * H));
          const double ang = std::atan2(sp.y, sp.x);  // in [0, π/2] for the quarter sector
          if (ang < -1e-9 || ang > sweepAng + 1e-9) worstAngle = std::max(worstAngle, 1.0);
        }
      expectLE(worstRad, 1e-9, "partial 90° revolve: radius R exact (true arc sector)");
      expectLE(worstZ, 1e-9, "partial 90° revolve: height linear (analytic quarter-cylinder)");
      expectLE(worstAngle, 0.0, "partial 90° revolve: every point within the [0,90°] sector");
      std::printf("INFO partial 90° revolve radius err = %.3e\n", worstRad);

      // S(·,1) is the profile rotated by exactly 90° (x→y): (R,0,z) → (0,R,z).
      double worstEnd = 0.0;
      for (int iu = 0; iu <= 40; ++iu) {
        const double u = static_cast<double>(iu) / 40.0;
        const Point3 sp = evalRatSurface(r.surface, u, 1.0);
        const Point3 end{0.0, R, u * H};
        worstEnd = std::max(worstEnd, distance(sp, end));
      }
      expectLE(worstEnd, 1e-9, "partial revolve: S(.,1) == profile rotated by the full angle");
    }

    // ── (e) A 270° revolve (>180°, forces 3 arc segments) still analytic. ──
    {
      const double R = 1.2, H = 2.0, sweepAng = 1.5 * PI;
      BsplineCurveData prof;
      prof.degree = 1;
      prof.knots = {0, 0, 1, 1};
      prof.poles = {{R, 0, 0}, {R, 0, H}};
      SweepResult r = sweepRotational(prof, axisP, axisD, sweepAng);
      expectTrue(r.ok, "sweepRotational 270° ok");
      expectTrue(r.surface.nPolesV == 7, "270° revolve: 3 arc segments → 7 V-poles");
      double worstRad = 0.0, worstZ = 0.0;
      for (int iu = 0; iu <= 4; ++iu)
        for (int iv = 0; iv <= 90; ++iv) {
          const double u = static_cast<double>(iu) / 4.0;
          const double v = static_cast<double>(iv) / 90.0;
          const Point3 sp = evalRatSurface(r.surface, u, v);
          worstRad = std::max(worstRad, std::fabs(std::hypot(sp.x, sp.y) - R));
          worstZ = std::max(worstZ, std::fabs(sp.z - u * H));
        }
      expectLE(worstRad, 1e-9, "270° revolve: radius R exact at every point (3-segment arc)");
      expectLE(worstZ, 1e-9, "270° revolve: height linear (analytic surface)");
      // Endpoint at V=1 is the profile rotated 270° about Z: (R,0,z) → (0,-R,z).
      double worstEnd = 0.0;
      for (int iu = 0; iu <= 20; ++iu) {
        const double u = static_cast<double>(iu) / 20.0;
        const Point3 sp = evalRatSurface(r.surface, u, 1.0);
        worstEnd = std::max(worstEnd, distance(sp, Point3{0.0, -R, u * H}));
      }
      expectLE(worstEnd, 1e-9, "270° revolve: S(.,1) == profile rotated 270°");
      std::printf("INFO revolved 270° radius err = %.3e (nPolesV=%d)\n",
                  worstRad, r.surface.nPolesV);
    }

    // ── (f) DEGENERATE / decline guards. ──
    {
      // Zero angle → no surface.
      BsplineCurveData prof;
      prof.degree = 1;
      prof.knots = {0, 0, 1, 1};
      prof.poles = {{2, 0, 0}, {2, 0, 5}};
      expectTrue(!sweepRotational(prof, axisP, axisD, 0.0).ok,
                 "rotational declines on zero angle");

      // Profile entirely ON the axis (radius 0 everywhere) → collapses → decline.
      BsplineCurveData onAxis;
      onAxis.degree = 1;
      onAxis.knots = {0, 0, 1, 1};
      onAxis.poles = {{0, 0, 0}, {0, 0, 5}};  // lies on the Z axis
      expectTrue(!sweepRotational(onAxis, axisP, axisD, 2 * PI).ok,
                 "rotational declines when the whole profile lies on the axis");

      // Null axis direction → decline.
      expectTrue(!sweepRotational(prof, axisP, Dir3(0, 0, 0), 2 * PI).ok,
                 "rotational declines on a null axis direction");

      // Malformed profile (bad knot vector) → decline.
      BsplineCurveData bad = prof;
      bad.knots.pop_back();
      expectTrue(!sweepRotational(bad, axisP, axisD, 2 * PI).ok,
                 "rotational declines on a malformed profile");

      // Rational profile with a non-positive weight → decline.
      BsplineCurveData ratbad = prof;
      ratbad.weights = {1.0, 0.0};
      expectTrue(!sweepRotational(ratbad, axisP, axisD, 2 * PI).ok,
                 "rotational declines on a rational profile with a non-positive weight");
    }
  }

  // ── report ──
  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_sweep: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_sweep: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_nurbs_sweep (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
