// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — skinning / lofting
// (src/native/math/bspline_skin.{h,cpp}). OCCT-FREE. The oracles are airtight and
// closed-form:
//
//   1. SECTION CONTAINMENT (the core oracle) — the skinned surface evaluated at the
//      section parameter v = v_k reproduces section curve k POINTWISE on a dense
//      u-sample to ~1e-8. The surface CONTAINS every input section exactly.
//   2. COMPATIBILITY CORRECTNESS — after raising to the common degree + merging to the
//      union knot vector, every section shares degree + knots + control-point count,
//      and each COMPATIBLE section still equals its ORIGINAL curve pointwise (Layer-1
//      elevate/refine are exact — no geometry drift).
//   3. KNOWN-SURFACE ROUND-TRIP — take a KNOWN tensor-product B-spline surface, extract
//      K iso-curves as sections, skin them → recover a surface matching the original
//      pointwise (the strongest oracle), when the section knots/params align.
//   4. DEGENERATE GUARDS — <2 sections, coincident sections, rational sections, and
//      incompatible-but-recoverable inputs are handled honestly (ok=false, no crash).
//
// The V-interpolation solves linear systems through numerics::lin_solve, so the whole
// gate is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_fit). With the guard OFF
// this compiles to a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_ops.h"
#include "native/math/bspline_skin.h"

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
// Rational (NURBS) evaluators — used for the weighted skinning oracles.
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

// An EXACT rational quadratic HALF-circle (semicircle) of `radius` centered at (cx,cy) in the
// plane z = zPlane, spanning θ ∈ [0, π]. 5 poles, knots {0,0,0,0.5,0.5,1,1,1} (two quarter
// Bézier arcs joined). Middle-of-each-quarter poles carry weight √2/2; the shared/mid pole is
// exact. This evaluates as a TRUE circle (the airtight rational conic).
static BsplineCurveData ratHalfCircle(double radius, double cx, double cy, double zPlane) {
  const double s = std::sqrt(2.0) / 2.0;  // cos(45°) — quarter-arc middle weight
  BsplineCurveData c;
  c.degree = 2;
  c.knots = {0, 0, 0, 0.5, 0.5, 1, 1, 1};  // 5 poles = 8 knots − 2 − 1
  // Quarter-arc control polygons (unit circle), θ: 0→90→180. Corner poles on the circle,
  // between-corner poles at the polygon corners (weight √2/2).
  c.poles = {{cx + radius, cy + 0, zPlane},          // θ=0    (w=1)
             {cx + radius, cy + radius, zPlane},      // corner (w=√2/2)
             {cx + 0, cy + radius, zPlane},           // θ=90   (w=1)
             {cx - radius, cy + radius, zPlane},      // corner (w=√2/2)
             {cx - radius, cy + 0, zPlane}};          // θ=180  (w=1)
  c.weights = {1.0, s, 1.0, s, 1.0};
  return c;
}

// Max pointwise distance between two curves over a dense u-sample of [0,1].
static double curveMaxDev(const BsplineCurveData& a, const BsplineCurveData& b, int nS = 200) {
  double worst = 0.0;
  for (int i = 0; i <= nS; ++i) {
    const double u = static_cast<double>(i) / nS;
    worst = std::max(worst, distance(evalCurve(a, u), evalCurve(b, u)));
  }
  return worst;
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

// A clamped cubic section on [0,1] with 6 poles (knot length 6+3+1 = 10). The z-plane
// and pole offset let us build a family of parallel-ish sections for a loft.
static BsplineCurveData sectionCubic(double zPlane, double xShift) {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = {0, 0, 0, 0, 0.4, 0.7, 1, 1, 1, 1};  // 6 poles
  c.poles = {{0 + xShift, 0, zPlane},
             {1 + xShift, 2, zPlane},
             {2 + xShift, -1, zPlane},
             {4 + xShift, 1, zPlane},
             {5 + xShift, 3, zPlane},
             {6 + xShift, 0, zPlane}};
  return c;
}

// A KNOWN non-rational tensor-product B-spline surface (round-trip oracle source).
static BsplineSurfaceData knownPatch() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 2;
  s.nPolesU = 6;
  s.nPolesV = 5;
  s.knotsU = {0, 0, 0, 0, 0.4, 0.7, 1, 1, 1, 1};  // 6 + 3 + 1 = 10
  s.knotsV = {0, 0, 0, 0.5, 0.75, 1, 1, 1};        // 5 + 2 + 1 = 8
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 5; ++j) {
      const double x = i * 1.2;
      const double y = j * 1.5;
      const double z = std::sin(0.6 * i) + std::cos(0.5 * j) + 0.04 * i * j;
      s.poles.push_back({x, y, z});
    }
  return s;
}

int main() {
  // ═══ 2. COMPATIBILITY CORRECTNESS ═══════════════════════════════════════════
  // Three sections of DIFFERENT degree and DIFFERENT interior knots. After
  // makeSectionsCompatible they must share degree + knots + N, and each compatible
  // section must still equal its ORIGINAL pointwise (Layer-1 ops are exact).
  {
    BsplineCurveData c0;  // degree 2, interior knot 0.5
    c0.degree = 2;
    c0.knots = {0, 0, 0, 0.5, 1, 1, 1};  // 4 poles
    c0.poles = {{0, 0, 0}, {1, 2, 0}, {3, -1, 0}, {4, 1, 0}};

    BsplineCurveData c1;  // degree 3, interior knots 0.3, 0.6
    c1.degree = 3;
    c1.knots = {0, 0, 0, 0, 0.3, 0.6, 1, 1, 1, 1};  // 6 poles
    c1.poles = {{0, 0, 1}, {1, 1, 1}, {2, 2, 1}, {3, 0, 1}, {4, 1, 1}, {5, -1, 1}};

    BsplineCurveData c2 = sectionCubic(2.0, 0.0);  // degree 3, interior 0.4, 0.7

    std::vector<BsplineCurveData> raw = {c0, c1, c2};
    SectionCompatibility comp = makeSectionsCompatible(raw);
    expectTrue(comp.ok, "makeSectionsCompatible ok on mixed degree/knots");
    expectTrue(comp.degree == 3, "common degree is the max (3)");
    expectTrue(comp.sections.size() == 3, "all sections returned");

    // Post-condition: identical degree + knots + control-point count.
    const std::size_t N = comp.sections.front().poles.size();
    bool sameShape = true;
    for (const BsplineCurveData& c : comp.sections) {
      if (c.degree != comp.degree) sameShape = false;
      if (c.poles.size() != N) sameShape = false;
      if (c.knots.size() != comp.knots.size()) sameShape = false;
      else
        for (std::size_t k = 0; k < c.knots.size(); ++k)
          if (std::fabs(c.knots[k] - comp.knots[k]) > 1e-12) sameShape = false;
    }
    expectTrue(sameShape, "compatible sections share degree + knots + N");
    expectTrue(comp.sections.front().weights.empty(), "compatible section non-rational");

    // No geometry drift: each compatible section equals its original pointwise.
    expectLE(curveMaxDev(comp.sections[0], c0), 1e-11, "compat section 0 == original (exact)");
    expectLE(curveMaxDev(comp.sections[1], c1), 1e-11, "compat section 1 == original (exact)");
    expectLE(curveMaxDev(comp.sections[2], c2), 1e-11, "compat section 2 == original (exact)");
  }

  // ═══ 1. SECTION CONTAINMENT (the core oracle) ═══════════════════════════════
  // Skin a family of sections; the surface iso-curve at v = v_k must reproduce the
  // COMPATIBLE section k pointwise on a dense u-sample. (Section k after compat equals
  // original section k, proven above, so the surface contains the original too.)
  {
    std::vector<BsplineCurveData> secs = {
        sectionCubic(0.0, 0.0), sectionCubic(1.0, 0.3),
        sectionCubic(2.0, 0.1), sectionCubic(3.0, -0.2)};
    SkinResult r = skinSurface(secs, 3);
    expectTrue(r.ok, "skinSurface ok (4 cubic sections)");
    expectTrue(r.surface.weights.empty(), "skinned surface is non-rational");
    expectTrue(r.surface.degreeU == 3, "surface U degree == section degree");
    expectTrue(r.vParams.size() == secs.size(), "one v-param per section");

    // Recompute the compatible sections (the surface contains THESE exactly).
    SectionCompatibility comp = makeSectionsCompatible(secs);
    expectTrue(comp.ok, "containment: compat ok");
    double worst = 0.0;
    for (std::size_t k = 0; k < secs.size(); ++k)
      worst = std::max(worst, isoVsCurveMaxDev(r.surface, r.vParams[k], comp.sections[k]));
    expectLE(worst, 1e-8, "surface iso-curve at v_k == section k POINTWISE (containment)");
    std::printf("INFO containment worst dev = %.3e over %zu sections\n", worst, secs.size());

    // The surface also contains the ORIGINAL sections (compat == original).
    double worstOrig = 0.0;
    for (std::size_t k = 0; k < secs.size(); ++k)
      worstOrig = std::max(worstOrig, isoVsCurveMaxDev(r.surface, r.vParams[k], secs[k]));
    expectLE(worstOrig, 1e-8, "surface contains the ORIGINAL sections too");
  }

  // Mixed-degree sections still contained (compat runs inside skin).
  {
    BsplineCurveData a;  // degree 2
    a.degree = 2;
    a.knots = {0, 0, 0, 0.5, 1, 1, 1};
    a.poles = {{0, 0, 0}, {1, 2, 0}, {3, -1, 0}, {4, 1, 0}};
    BsplineCurveData b = sectionCubic(2.0, 0.0);  // degree 3
    BsplineCurveData c = sectionCubic(4.0, 0.5);  // degree 3
    std::vector<BsplineCurveData> secs = {a, b, c};
    SkinResult r = skinSurface(secs, 2);
    expectTrue(r.ok, "skinSurface ok (mixed degree sections)");
    expectTrue(r.surface.degreeU == 3, "mixed-degree loft raises U to max degree");
    SectionCompatibility comp = makeSectionsCompatible(secs);
    double worst = 0.0;
    for (std::size_t k = 0; k < secs.size(); ++k)
      worst = std::max(worst, isoVsCurveMaxDev(r.surface, r.vParams[k], comp.sections[k]));
    expectLE(worst, 1e-8, "mixed-degree loft contains every compatible section");
  }

  // ═══ 3. KNOWN-SURFACE ROUND-TRIP ════════════════════════════════════════════
  //
  // Part (a): extract K iso-curves from a KNOWN surface as sections and skin them. The
  // skinned surface must CONTAIN each extracted iso-curve exactly. This proves the skin
  // reconstructs geometry taken from a real tensor-product surface (not synthetic
  // sections). The FULL-surface pointwise identity does NOT hold in general, because the
  // skin re-parametrizes V by chord length across the control polygons (its own
  // averaging knots), which need not equal the source's V-knots — the classic
  // parametrization confound (the same one the Layer-7 round-trip documents). So part
  // (a) tests exact iso-curve containment, and part (b) tests full-surface IDENTITY via
  // idempotence, where the V-parametrization IS a fixed point by construction.
  {
    const BsplineSurfaceData src = knownPatch();
    const int K = 5;  // == nPolesV of the source
    std::vector<double> vStations(K);
    for (int j = 0; j < K; ++j) {  // interior Greville abscissae of knotsV
      double s = 0.0;
      for (int t = 1; t <= src.degreeV; ++t) s += src.knotsV[j + t];
      vStations[j] = s / src.degreeV;
    }
    std::vector<BsplineCurveData> secs;
    for (int j = 0; j < K; ++j) {
      BsplineCurveData c;  // iso-curve S(u, vStations[j]) as a U-direction B-spline
      c.degree = src.degreeU;
      c.knots = src.knotsU;
      c.poles.resize(src.nPolesU);
      for (int i = 0; i < src.nPolesU; ++i) {  // blend each U-pole row in V
        std::vector<Point3> vrow(src.nPolesV);
        for (int l = 0; l < src.nPolesV; ++l)
          vrow[l] = src.poles[static_cast<std::size_t>(i) * src.nPolesV + l];
        c.poles[i] = curvePoint(src.degreeV, vrow, src.knotsV, vStations[j]);
      }
      secs.push_back(c);
    }

    SkinResult r = skinSurface(secs, src.degreeV);
    expectTrue(r.ok, "round-trip(a): skin of extracted iso-curves ok");
    expectTrue(r.surface.degreeU == src.degreeU, "round-trip(a) U degree matches source");

    SectionCompatibility comp = makeSectionsCompatible(secs);
    double worstSec = 0.0;
    for (int k = 0; k < K; ++k)
      worstSec = std::max(worstSec, isoVsCurveMaxDev(r.surface, r.vParams[k], comp.sections[k]));
    expectLE(worstSec, 1e-8, "round-trip(a): every KNOWN-surface iso-curve contained to 1e-8");
    std::printf("INFO known-surface iso-curve containment worst dev = %.3e\n", worstSec);
  }

  // Part (b): IDEMPOTENCE — the exact full-surface round-trip. Skin a family → S1;
  // extract S1's OWN iso-curves at S1's OWN section parameters v_k → re-skin → S2; then
  // S1 ≡ S2 POINTWISE to machine precision. Here the V-parametrization is a fixed point
  // by construction (the second skin sees the same v_k and reproduces the same averaging
  // knots), so the whole surface — not just the sections — is reconstructed exactly.
  {
    std::vector<BsplineCurveData> secs = {
        sectionCubic(0.0, 0.0), sectionCubic(1.0, 0.4),
        sectionCubic(2.0, 0.1), sectionCubic(3.0, -0.3), sectionCubic(4.0, 0.2)};
    SkinResult s1 = skinSurface(secs, 3);
    expectTrue(s1.ok, "idempotence: S1 skin ok");

    // Re-extract iso-curves from S1 at S1's own section parameters v_k.
    const BsplineSurfaceData& S = s1.surface;
    std::vector<BsplineCurveData> reSecs;
    for (double vk : s1.vParams) {
      BsplineCurveData c;
      c.degree = S.degreeU;
      c.knots = S.knotsU;
      c.poles.resize(S.nPolesU);
      for (int i = 0; i < S.nPolesU; ++i) {
        std::vector<Point3> vrow(S.nPolesV);
        for (int l = 0; l < S.nPolesV; ++l)
          vrow[l] = S.poles[static_cast<std::size_t>(i) * S.nPolesV + l];
        c.poles[i] = curvePoint(S.degreeV, vrow, S.knotsV, vk);
      }
      reSecs.push_back(c);
    }
    SkinResult s2 = skinSurface(reSecs, 3);
    expectTrue(s2.ok, "idempotence: S2 re-skin ok");
    expectTrue(s1.surface.poles.size() == s2.surface.poles.size(),
               "idempotence: identical net size");

    // Control-net identity.
    double worstPole = 0.0;
    for (std::size_t i = 0; i < s1.surface.poles.size(); ++i)
      worstPole = std::max(worstPole, distance(s1.surface.poles[i], s2.surface.poles[i]));
    expectLE(worstPole, 1e-9, "idempotence: recovers identical surface control net to 1e-9");

    // Dense pointwise identity — the strongest full-surface oracle.
    double worstSurf = 0.0;
    for (int iu = 0; iu <= 40; ++iu)
      for (int iv = 0; iv <= 40; ++iv) {
        const double u = static_cast<double>(iu) / 40;
        const double v = static_cast<double>(iv) / 40;
        worstSurf = std::max(worstSurf, distance(evalSurface(s1.surface, u, v),
                                                 evalSurface(s2.surface, u, v)));
      }
    std::printf("INFO idempotence full-surface round-trip worst dev = %.3e\n", worstSurf);
    expectLE(worstSurf, 1e-8, "idempotence recovers the skinned surface POINTWISE to 1e-8");
  }

  // ═══ 4. DEGENERATE GUARDS ═══════════════════════════════════════════════════
  {
    // < 2 sections declines honestly.
    std::vector<BsplineCurveData> one = {sectionCubic(0.0, 0.0)};
    expectTrue(!skinSurface(one).ok, "skin declines with <2 sections");
    std::vector<BsplineCurveData> none;
    expectTrue(!skinSurface(none).ok, "skin declines with 0 sections");

    // Coincident sections (identical) → no V length to normalize → decline.
    std::vector<BsplineCurveData> same = {sectionCubic(1.0, 0.0), sectionCubic(1.0, 0.0),
                                          sectionCubic(1.0, 0.0)};
    expectTrue(!skinSurface(same).ok, "skin declines on coincident sections (honest guard)");

    // Rational section → non-rational scope declines (never fakes rational).
    BsplineCurveData ratl = sectionCubic(0.0, 0.0);
    ratl.weights.assign(ratl.poles.size(), 1.0);  // non-empty ⇒ rational
    std::vector<BsplineCurveData> withRat = {sectionCubic(0.0, 0.0), ratl};
    expectTrue(!makeSectionsCompatible(withRat).ok, "compat declines on rational section");
    expectTrue(!skinSurface(withRat).ok, "skin declines on rational section (non-rational scope)");

    // Incompatible-but-recoverable: two sections that only differ in degree + knots
    // still skin (compat recovers), proving graceful handling, not a crash.
    BsplineCurveData lo;
    lo.degree = 1;
    lo.knots = {0, 0, 1, 1};  // 2 poles, straight line
    lo.poles = {{0, 0, 0}, {6, 0, 0}};
    BsplineCurveData hi = sectionCubic(2.0, 0.0);
    std::vector<BsplineCurveData> recover = {lo, hi};
    SkinResult rr = skinSurface(recover, 1);
    expectTrue(rr.ok, "incompatible-but-recoverable pair skins (compat recovers)");
    SectionCompatibility comp = makeSectionsCompatible(recover);
    if (comp.ok) {
      double worst = 0.0;
      for (std::size_t k = 0; k < recover.size(); ++k)
        worst = std::max(worst, isoVsCurveMaxDev(rr.surface, rr.vParams[k], comp.sections[k]));
      expectLE(worst, 1e-8, "recovered pair: sections still contained exactly");
    }
  }

  // ═══ 5. RATIONAL SKINNING ═══════════════════════════════════════════════════
  //
  // The rational skin lifts every section to homogeneous R⁴, interpolates across the
  // sections there, and projects back to a rational surface. Its oracles mirror the
  // non-rational ones, evaluated as rational NURBS.

  // 5a. RATIONAL CONTAINMENT — skin a stack of EXACT rational semicircles of the SAME
  // radius; the rational surface's iso-curve at v_k reproduces rational section k pointwise.
  {
    std::vector<BsplineCurveData> secs = {
        ratHalfCircle(2.0, 0.0, 0.0, 0.0), ratHalfCircle(2.0, 0.0, 0.0, 1.0),
        ratHalfCircle(2.0, 0.0, 0.0, 2.0), ratHalfCircle(2.0, 0.0, 0.0, 3.0)};
    SkinResult r = skinRationalSurface(secs, 3);
    expectTrue(r.ok, "skinRationalSurface ok (4 rational semicircles)");
    expectTrue(!r.surface.weights.empty(), "rational skin surface IS rational");
    expectTrue(r.surface.weights.size() == r.surface.poles.size(),
               "one weight per surface pole");
    expectTrue(r.surface.degreeU == 2, "rational skin U degree == section degree (2)");
    expectTrue(r.vParams.size() == secs.size(), "one v-param per rational section");

    // Compatible rational sections the surface contains exactly.
    SectionCompatibility comp = makeRationalSectionsCompatible(secs);
    expectTrue(comp.ok, "rational compat ok");
    double worst = 0.0;
    for (std::size_t k = 0; k < secs.size(); ++k)
      worst = std::max(worst, ratIsoVsCurveMaxDev(r.surface, r.vParams[k], comp.sections[k]));
    expectLE(worst, 1e-9, "RATIONAL surface iso at v_k == rational section k POINTWISE");
    std::printf("INFO rational containment worst dev = %.3e over %zu sections\n",
                worst, secs.size());

    // Contains the ORIGINAL rational sections too (compat == original, rational-exact).
    double worstOrig = 0.0;
    for (std::size_t k = 0; k < secs.size(); ++k)
      worstOrig = std::max(worstOrig, ratIsoVsCurveMaxDev(r.surface, r.vParams[k], secs[k]));
    expectLE(worstOrig, 1e-9, "rational skin contains the ORIGINAL rational sections too");

    // Each contained iso-curve is a TRUE circle: every point at radius 2 from the axis.
    double worstRad = 0.0;
    for (double vk : r.vParams)
      for (int i = 0; i <= 100; ++i) {
        const double u = static_cast<double>(i) / 100.0;
        const Point3 p = evalRatSurface(r.surface, u, vk);
        worstRad = std::max(worstRad, std::fabs(std::hypot(p.x, p.y) - 2.0));
      }
    expectLE(worstRad, 1e-9, "rational skin iso-curves are TRUE circles (radius exact)");
    std::printf("INFO rational skin circle-radius worst err = %.3e\n", worstRad);
  }

  // 5b. RATIONAL CONE / FRUSTUM — skin rational circles of DIFFERENT radii (a truncated
  // cone). The surface contains each rational circle exactly, and each iso is a true circle
  // of the prescribed radius. This proves rational skinning across a varying-radius family.
  {
    std::vector<BsplineCurveData> secs = {ratHalfCircle(1.0, 0.0, 0.0, 0.0),
                                          ratHalfCircle(2.0, 0.0, 0.0, 2.0),
                                          ratHalfCircle(3.5, 0.0, 0.0, 4.0)};
    const double radii[] = {1.0, 2.0, 3.5};
    SkinResult r = skinRationalSurface(secs, 2);
    expectTrue(r.ok, "skinRationalSurface ok (rational frustum, 3 radii)");
    expectTrue(!r.surface.weights.empty(), "frustum surface is rational");

    SectionCompatibility comp = makeRationalSectionsCompatible(secs);
    double worst = 0.0, worstRad = 0.0;
    for (std::size_t k = 0; k < secs.size(); ++k) {
      worst = std::max(worst, ratIsoVsCurveMaxDev(r.surface, r.vParams[k], comp.sections[k]));
      for (int i = 0; i <= 100; ++i) {
        const double u = static_cast<double>(i) / 100.0;
        const Point3 p = evalRatSurface(r.surface, u, r.vParams[k]);
        worstRad = std::max(worstRad, std::fabs(std::hypot(p.x, p.y) - radii[k]));
      }
    }
    expectLE(worst, 1e-9, "rational frustum contains each rational circle POINTWISE");
    expectLE(worstRad, 1e-9, "rational frustum: each ring iso is a true circle of its radius");
    std::printf("INFO rational frustum containment %.3e, radius err %.3e\n", worst, worstRad);
  }

  // 5c. RATIONAL DEGENERATE / GUARD checks.
  {
    // Non-rational sections declined by the rational skin (wrong path).
    std::vector<BsplineCurveData> nonrat = {sectionCubic(0.0, 0.0), sectionCubic(1.0, 0.0)};
    expectTrue(!skinRationalSurface(nonrat).ok,
               "rational skin declines on NON-rational sections");
    expectTrue(!makeRationalSectionsCompatible(nonrat).ok,
               "rational compat declines on non-rational sections");

    // Non-positive weight declined honestly.
    BsplineCurveData badW = ratHalfCircle(2.0, 0.0, 0.0, 0.0);
    badW.weights[2] = -0.5;  // negative weight
    std::vector<BsplineCurveData> withBad = {badW, ratHalfCircle(2.0, 0.0, 0.0, 1.0)};
    expectTrue(!skinRationalSurface(withBad).ok,
               "rational skin declines on non-positive weight (honest guard)");
    expectTrue(!makeRationalSectionsCompatible(withBad).ok,
               "rational compat declines on non-positive weight");

    // Mismatched weight count declined.
    BsplineCurveData mism = ratHalfCircle(2.0, 0.0, 0.0, 0.0);
    mism.weights.pop_back();  // one fewer weight than poles
    std::vector<BsplineCurveData> withMism = {mism, ratHalfCircle(2.0, 0.0, 0.0, 1.0)};
    expectTrue(!skinRationalSurface(withMism).ok,
               "rational skin declines on weight/pole count mismatch");

    // < 2 rational sections declines.
    std::vector<BsplineCurveData> one = {ratHalfCircle(2.0, 0.0, 0.0, 0.0)};
    expectTrue(!skinRationalSurface(one).ok, "rational skin declines with <2 sections");

    // Coincident rational sections → no V length → decline.
    std::vector<BsplineCurveData> same = {ratHalfCircle(2.0, 0.0, 0.0, 1.0),
                                          ratHalfCircle(2.0, 0.0, 0.0, 1.0)};
    expectTrue(!skinRationalSurface(same).ok,
               "rational skin declines on coincident rational sections");

    // Mixed-degree RATIONAL sections still compatibilize + skin (rational elevate is exact):
    // a rational circle (degree 2) + a degree-3 rational curve share a domain [0,1].
    BsplineCurveData rat3;  // degree-3 rational curve on [0,1]
    rat3.degree = 3;
    rat3.knots = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};  // 5 poles
    rat3.poles = {{3, 0, 5}, {2, 2, 5}, {0, 3, 5}, {-2, 2, 5}, {-3, 0, 5}};
    rat3.weights = {1.0, 0.8, 1.2, 0.8, 1.0};
    std::vector<BsplineCurveData> mixed = {ratHalfCircle(3.0, 0.0, 0.0, 0.0), rat3};
    SkinResult rmix = skinRationalSurface(mixed, 1);
    expectTrue(rmix.ok, "rational skin ok on mixed-degree rational sections");
    if (rmix.ok) {
      SectionCompatibility comp = makeRationalSectionsCompatible(mixed);
      expectTrue(comp.degree == 3, "mixed rational compat raises to degree 3");
      double worst = 0.0;
      for (std::size_t k = 0; k < mixed.size(); ++k)
        worst = std::max(worst, ratIsoVsCurveMaxDev(rmix.surface, rmix.vParams[k],
                                                    comp.sections[k]));
      expectLE(worst, 1e-9, "mixed-degree rational loft contains every rational section");
    }
  }

  // ── report ──
  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_skin: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_skin: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_nurbs_skin (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
