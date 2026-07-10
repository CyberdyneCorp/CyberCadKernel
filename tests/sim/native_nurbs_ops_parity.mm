// SPDX-License-Identifier: Apache-2.0
//
// native_nurbs_ops_parity.mm — SIM native-vs-OCCT parity harness for the
// exact-NURBS geometry kernel (NURBS roadmap Layer 1). This is GATE (b) of the
// two-gate discipline for `nurbs-exact-geometry-kernel`; gate (a) is the
// OCCT-free host-analytic suite tests/native/test_native_nurbs_ops.cpp.
//
// ─────────────────────────────────────────────────────────────────────────────
// BUILD DEPENDENCY — AUTHORED BUT NOT YET BUILDABLE.
//
//   This harness #includes "native/math/bspline_ops.h" and calls the frozen API
//   from openspec/changes/nurbs-exact-geometry-kernel/design.md. That module
//   (src/native/math/bspline_ops.{h,cpp}) is being implemented CONCURRENTLY in a
//   separate worktree and is NOT present here yet, so this file DOES NOT COMPILE
//   until the module lands. Everything else — the OCCT reference mapping, the
//   flat↔(knots,mults) conversions, the sampled-point oracle, the SKIP wiring —
//   is complete and compile-ready against the frozen signatures. The orchestrator
//   does the build-verify at consolidation (add a scripts/run-sim-native-nurbs-
//   ops.sh runner modelled on run-sim-native-math.sh: it globs src/native/math/
//   *.cpp OCCT-free + this .mm + the TKG3d/TKGeomBase/TKG2d/TKMath/TKernel oracle
//   slice; bspline_ops.cpp is picked up by that glob automatically).
// ─────────────────────────────────────────────────────────────────────────────
//
// WHAT THIS ASSERTS. For each construction op we build the SAME B-spline / NURBS
// curve and surface on BOTH sides — native `BsplineCurveData`/`BsplineSurfaceData`
// and an OCCT `Geom_BSpline*` with knots converted flat→(knots,mults) — apply the
// native op AND the OCCT reference op, then assert they AGREE on:
//   (1) the resulting (degree, poles, weights, knots) after converting OCCT's
//       (knots,mults) back to a flat vector, to a FIXED tolerance, AND
//   (2) a dense sample of evaluated points between native-result and OCCT-result.
// Both non-rational and rational curves+surfaces, varied degree, interior knots,
// and endpoint / full-multiplicity cases are covered.
//
// NATIVE-op → OCCT-reference map (see the report / design.md §"Oracle strategy"):
//   insertKnotCurve      → Geom_BSplineCurve::InsertKnot(U, M)          (Boehm)
//   refineKnotCurve      → Geom_BSplineCurve::InsertKnots(Knots, Mults) (Oslo)
//   removeKnotCurve      → Geom_BSplineCurve::RemoveKnot(Index, M, Tol)
//   elevateDegreeCurve   → Geom_BSplineCurve::IncreaseDegree(deg + t)
//   splitCurve           → two Geom_BSplineCurve::Segment(a,b) sub-intervals
//   decomposeCurveToBezier → GeomConvert_BSplineCurveToBezierCurve (cross-check)
//   insertKnotSurface    → Geom_BSplineSurface::InsertUKnot / InsertVKnot
//   elevateDegreeSurface → Geom_BSplineSurface::IncreaseDegree
//   removeKnotSurface    → Geom_BSplineSurface::RemoveUKnot / RemoveVKnot
//   splitSurface         → Geom_BSplineSurface::Segment on the sub-rectangle
//
// CONVENTION CAVEATS (confirmed against the sim-build OCCT headers):
//   * FLAT ↔ (knots,mults). The kernel stores ONE flat knot vector of length
//     nPoles+degree+1 (multiplicities expanded). OCCT stores SEPARATE distinct
//     `Knots()` + `Multiplicities()`. `flatToKnotsMults` collapses flat→(K,M) on
//     the way in; `occtCurveToFlat` / `occtSurfaceToFlat` expand (K,M)→flat on the
//     way back for the direct knot compare. This two-way conversion is the single
//     most error-prone spot and is exercised on every trial.
//   * RemoveKnot takes an INDEX into the DISTINCT-knots array (1-based), NOT a
//     parameter value. `knotIndexOf` finds it. The native API takes the knot VALUE
//     `u`; the harness maps value→index for the OCCT side.
//   * Segment mutates in place and MAY RELOCATE/RESCALE interior knots as it trims;
//     it also re-clamps ends. So for split/segment we compare EVALUATED POINTS on
//     the sub-domain (the geometry, which is invariant), not the raw knot arrays —
//     the native splitCurve keeps the original parametrisation on each piece, which
//     OCCT's Segment preserves pointwise. Knot-array equality is asserted only for
//     the ops OCCT keeps in the original parametrisation (insert/refine/elevate/
//     remove).
//   * OCCT Pole()/Weight() are 1-based; surface Pole(UIndex,VIndex) is (row=U,col=V)
//     which matches the kernel's row-major U-outer layout pole(i,j)=poles[i*nV+j].
//
// Every trial is classified AGREED / DISAGREED (native wrong) / ORACLE-INACCURATE
// (native more correct than OCCT at a numeric edge — justified inline). Bar:
// DISAGREED == 0. Tolerances are FIXED and never widened.
//
// OCCT-DEPENDENT. Carries its own main(); std::_Exit to skip the non-exit-clean
// OCCT static teardown of the trimmed static build (same rationale as
// native_math_parity / native_analysis_parity).

#include "native/math/bspline.h"       // findSpan/curvePoint/nurbs* evaluators
#include "native/math/bspline_ops.h"   // FROZEN API under test — lands concurrently
#include "native/math/vec.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_nurbs_ops_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif

// ── OCCT oracle headers ──────────────────────────────────────────────────────
#include <gp_Pnt.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierCurve.hxx>
#include <GeomConvert_BSplineCurveToBezierCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

namespace nm = cybercad::native::math;

// ═════════════════════════════════════════════════════════════════════════════
// Trial accounting + classification (AGREED / DISAGREED / ORACLE-INACCURATE).
// ═════════════════════════════════════════════════════════════════════════════
static int g_agree = 0;      // native and OCCT agree within tol
static int g_disagree = 0;   // native disagrees with OCCT — native presumed WRONG (the bar)
static int g_oracle = 0;     // native more correct than OCCT at a numeric edge (justified)

// FIXED tolerances — chosen from the evaluated-magnitude scale; NEVER widened.
static constexpr double kTolKnot = 1e-9;   // knot values (domains are O(1))
static constexpr double kTolPole = 1e-9;   // pole coords (coords O(1..20))
static constexpr double kTolPt   = 1e-9;   // sampled evaluated points

// Error metric: absolute for small magnitudes, relative once |ref| is large, so a
// fixed 1e-9 absolute tolerance is not unfairly strict on large coordinates.
static double relErr(double got, double ref) {
  const double a = std::fabs(got - ref);
  const double m = std::fabs(ref);
  return (m > 1.0) ? a / m : a;
}
static double ptErr(const nm::Point3& p, const gp_Pnt& q) {
  return std::max({relErr(p.x, q.X()), relErr(p.y, q.Y()), relErr(p.z, q.Z())});
}

// One classified trial. `nativeErr` is native-vs-OCCT worst error over the whole
// (knots/poles/weights + sampled points) compare. `oracleInaccurate` is passed
// true (with a justification) only where the native result is provably closer to
// the analytic truth than OCCT at a numeric edge.
static void classify(const char* name, double nativeErr, double tol,
                     bool oracleInaccurate = false, const char* justify = "") {
  const char* verdict;
  if (nativeErr <= tol) { verdict = "AGREED"; ++g_agree; }
  else if (oracleInaccurate) { verdict = "ORACLE-INACCURATE"; ++g_oracle; }
  else { verdict = "DISAGREED"; ++g_disagree; }
  std::printf("[NURBS-OPS] %-46s %-18s err=%.3e tol=%.1e%s%s\n", name, verdict,
              nativeErr, tol, justify[0] ? "  " : "", justify);
  std::fflush(stdout);
}

// ═════════════════════════════════════════════════════════════════════════════
// FLAT ↔ (knots, mults) conversion — the load-bearing convention bridge.
// ═════════════════════════════════════════════════════════════════════════════

// Collapse a FLAT knot vector (knots repeated by multiplicity) → OCCT's distinct
// (knots, mults). Identical to native_math_parity's helper (proven idiom).
static void flatToKnotsMults(const std::vector<double>& flat,
                             std::vector<double>& knots, std::vector<int>& mults) {
  knots.clear();
  mults.clear();
  for (double k : flat) {
    if (!knots.empty() && std::fabs(k - knots.back()) < 1e-15) ++mults.back();
    else { knots.push_back(k); mults.push_back(1); }
  }
}

// Expand OCCT distinct (knots, mults) → a FLAT knot vector (the kernel's rep).
static std::vector<double> expandFlat(const std::vector<double>& knots,
                                      const std::vector<int>& mults) {
  std::vector<double> flat;
  for (std::size_t i = 0; i < knots.size(); ++i)
    for (int m = 0; m < mults[i]; ++m) flat.push_back(knots[i]);
  return flat;
}

// 1-based INDEX (into the distinct-knots array) of the knot value nearest `u`,
// for the OCCT RemoveKnot(Index,...) API which is index-, not value-, addressed.
static int knotIndexOf(const std::vector<double>& knots, double u) {
  int best = 1;
  double bd = 1e300;
  for (int i = 0; i < static_cast<int>(knots.size()); ++i) {
    const double d = std::fabs(knots[i] - u);
    if (d < bd) { bd = d; best = i + 1; }
  }
  return best;
}

static TColStd_Array1OfReal toReal1(const std::vector<double>& v) {
  TColStd_Array1OfReal a(1, static_cast<int>(v.size()));
  for (int i = 0; i < static_cast<int>(v.size()); ++i) a.SetValue(i + 1, v[i]);
  return a;
}
static TColStd_Array1OfInteger toInt1(const std::vector<int>& v) {
  TColStd_Array1OfInteger a(1, static_cast<int>(v.size()));
  for (int i = 0; i < static_cast<int>(v.size()); ++i) a.SetValue(i + 1, v[i]);
  return a;
}
static TColgp_Array1OfPnt toPnt1(const std::vector<nm::Point3>& v) {
  TColgp_Array1OfPnt a(1, static_cast<int>(v.size()));
  for (int i = 0; i < static_cast<int>(v.size()); ++i)
    a.SetValue(i + 1, gp_Pnt(v[i].x, v[i].y, v[i].z));
  return a;
}

// ── native BsplineCurveData → OCCT Geom_BSplineCurve ─────────────────────────
static Handle(Geom_BSplineCurve) buildCurve(const nm::BsplineCurveData& c) {
  std::vector<double> knots;
  std::vector<int> mults;
  flatToKnotsMults(c.knots, knots, mults);
  TColgp_Array1OfPnt occP = toPnt1(c.poles);
  TColStd_Array1OfReal occK = toReal1(knots);
  TColStd_Array1OfInteger occM = toInt1(mults);
  if (c.weights.empty())
    return new Geom_BSplineCurve(occP, occK, occM, c.degree, Standard_False);
  TColStd_Array1OfReal occW = toReal1(c.weights);
  return new Geom_BSplineCurve(occP, occW, occK, occM, c.degree, Standard_False);
}

// ── OCCT Geom_BSplineCurve → native BsplineCurveData (flat) for the knot compare ─
static nm::BsplineCurveData occtCurveToData(const Handle(Geom_BSplineCurve)& oc) {
  nm::BsplineCurveData d;
  d.degree = oc->Degree();
  const int nP = oc->NbPoles(), nK = oc->NbKnots();
  for (int i = 1; i <= nP; ++i) {
    const gp_Pnt p = oc->Pole(i);
    d.poles.push_back({p.X(), p.Y(), p.Z()});
  }
  if (oc->IsRational())
    for (int i = 1; i <= nP; ++i) d.weights.push_back(oc->Weight(i));
  std::vector<double> knots;
  std::vector<int> mults;
  for (int i = 1; i <= nK; ++i) { knots.push_back(oc->Knot(i)); mults.push_back(oc->Multiplicity(i)); }
  d.knots = expandFlat(knots, mults);
  return d;
}

// ── native BsplineSurfaceData → OCCT Geom_BSplineSurface ─────────────────────
static Handle(Geom_BSplineSurface) buildSurface(const nm::BsplineSurfaceData& s) {
  std::vector<double> ku, kv;
  std::vector<int> mu, mv;
  flatToKnotsMults(s.knotsU, ku, mu);
  flatToKnotsMults(s.knotsV, kv, mv);
  TColgp_Array2OfPnt occP(1, s.nPolesU, 1, s.nPolesV);
  for (int i = 0; i < s.nPolesU; ++i)
    for (int j = 0; j < s.nPolesV; ++j) {
      const nm::Point3& p = s.poles[static_cast<std::size_t>(i) * s.nPolesV + j];
      occP.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  TColStd_Array1OfReal occKu = toReal1(ku), occKv = toReal1(kv);
  TColStd_Array1OfInteger occMu = toInt1(mu), occMv = toInt1(mv);
  if (s.weights.empty())
    return new Geom_BSplineSurface(occP, occKu, occKv, occMu, occMv, s.degreeU, s.degreeV,
                                   Standard_False, Standard_False);
  TColStd_Array2OfReal occW(1, s.nPolesU, 1, s.nPolesV);
  for (int i = 0; i < s.nPolesU; ++i)
    for (int j = 0; j < s.nPolesV; ++j)
      occW.SetValue(i + 1, j + 1, s.weights[static_cast<std::size_t>(i) * s.nPolesV + j]);
  return new Geom_BSplineSurface(occP, occW, occKu, occKv, occMu, occMv, s.degreeU, s.degreeV,
                                 Standard_False, Standard_False);
}

// ── OCCT Geom_BSplineSurface → native BsplineSurfaceData (flat) ──────────────
static nm::BsplineSurfaceData occtSurfaceToData(const Handle(Geom_BSplineSurface)& os) {
  nm::BsplineSurfaceData d;
  d.degreeU = os->UDegree();
  d.degreeV = os->VDegree();
  d.nPolesU = os->NbUPoles();
  d.nPolesV = os->NbVPoles();
  for (int i = 1; i <= d.nPolesU; ++i)
    for (int j = 1; j <= d.nPolesV; ++j) {
      const gp_Pnt p = os->Pole(i, j);
      d.poles.push_back({p.X(), p.Y(), p.Z()});
    }
  if (os->IsURational() || os->IsVRational())
    for (int i = 1; i <= d.nPolesU; ++i)
      for (int j = 1; j <= d.nPolesV; ++j) d.weights.push_back(os->Weight(i, j));
  std::vector<double> ku, kv;
  std::vector<int> mu, mv;
  for (int i = 1; i <= os->NbUKnots(); ++i) { ku.push_back(os->UKnot(i)); mu.push_back(os->UMultiplicity(i)); }
  for (int i = 1; i <= os->NbVKnots(); ++i) { kv.push_back(os->VKnot(i)); mv.push_back(os->VMultiplicity(i)); }
  d.knotsU = expandFlat(ku, mu);
  d.knotsV = expandFlat(kv, mv);
  return d;
}

// ═════════════════════════════════════════════════════════════════════════════
// Direct-representation compare: native BsplineCurveData vs OCCT-derived data.
// Returns the worst error over degree/poles/weights/knots. `compareKnots` is
// false for Segment-class ops (OCCT may relocate interior knots — see caveat).
// ═════════════════════════════════════════════════════════════════════════════
static double curveRepErr(const nm::BsplineCurveData& a, const nm::BsplineCurveData& b,
                          bool compareKnots = true) {
  double e = 0.0;
  if (a.degree != b.degree) return 1e9;
  if (a.poles.size() != b.poles.size()) return 1e9;
  for (std::size_t i = 0; i < a.poles.size(); ++i) {
    e = std::max(e, relErr(a.poles[i].x, b.poles[i].x));
    e = std::max(e, relErr(a.poles[i].y, b.poles[i].y));
    e = std::max(e, relErr(a.poles[i].z, b.poles[i].z));
  }
  // Weights: both rational or both non-rational; compare where present.
  if (a.weights.size() == b.weights.size())
    for (std::size_t i = 0; i < a.weights.size(); ++i)
      e = std::max(e, relErr(a.weights[i], b.weights[i]));
  else if (!a.weights.empty() != !b.weights.empty())
    return 1e9;  // rationality mismatch
  if (compareKnots) {
    if (a.knots.size() != b.knots.size()) return 1e9;
    for (std::size_t i = 0; i < a.knots.size(); ++i)
      e = std::max(e, relErr(a.knots[i], b.knots[i]));
  }
  return e;
}

// Dense sampled-point compare of a native curve vs an OCCT curve over [lo,hi].
static double curveSampleErr(const nm::BsplineCurveData& n,
                             const Handle(Geom_BSplineCurve)& oc, double lo, double hi,
                             int samples = 40) {
  double e = 0.0;
  for (int s = 0; s <= samples; ++s) {
    const double u = lo + (hi - lo) * (double(s) / samples);
    nm::Point3 np = n.weights.empty()
                        ? nm::curvePoint(n.degree, n.poles, n.knots, u)
                        : nm::nurbsCurvePoint(n.degree, n.poles, n.weights, n.knots, u);
    gp_Pnt op;
    oc->D0(u, op);
    e = std::max(e, ptErr(np, op));
  }
  return e;
}

static double surfaceRepErr(const nm::BsplineSurfaceData& a, const nm::BsplineSurfaceData& b,
                            bool compareKnots = true) {
  double e = 0.0;
  if (a.degreeU != b.degreeU || a.degreeV != b.degreeV) return 1e9;
  if (a.nPolesU != b.nPolesU || a.nPolesV != b.nPolesV) return 1e9;
  for (std::size_t i = 0; i < a.poles.size() && i < b.poles.size(); ++i) {
    e = std::max(e, relErr(a.poles[i].x, b.poles[i].x));
    e = std::max(e, relErr(a.poles[i].y, b.poles[i].y));
    e = std::max(e, relErr(a.poles[i].z, b.poles[i].z));
  }
  if (a.weights.size() == b.weights.size())
    for (std::size_t i = 0; i < a.weights.size(); ++i)
      e = std::max(e, relErr(a.weights[i], b.weights[i]));
  if (compareKnots) {
    if (a.knotsU.size() != b.knotsU.size() || a.knotsV.size() != b.knotsV.size()) return 1e9;
    for (std::size_t i = 0; i < a.knotsU.size(); ++i) e = std::max(e, relErr(a.knotsU[i], b.knotsU[i]));
    for (std::size_t i = 0; i < a.knotsV.size(); ++i) e = std::max(e, relErr(a.knotsV[i], b.knotsV[i]));
  }
  return e;
}

static double surfaceSampleErr(const nm::BsplineSurfaceData& n,
                               const Handle(Geom_BSplineSurface)& os,
                               double u0, double u1, double v0, double v1, int samples = 12) {
  double e = 0.0;
  nm::SurfaceGrid grid{n.poles, n.nPolesU, n.nPolesV};
  for (int su = 0; su <= samples; ++su)
    for (int sv = 0; sv <= samples; ++sv) {
      const double u = u0 + (u1 - u0) * (double(su) / samples);
      const double v = v0 + (v1 - v0) * (double(sv) / samples);
      nm::Point3 np = n.weights.empty()
                          ? nm::surfacePoint(n.degreeU, n.degreeV, grid, n.knotsU, n.knotsV, u, v)
                          : nm::nurbsSurfacePoint(n.degreeU, n.degreeV, grid, n.weights,
                                                  n.knotsU, n.knotsV, u, v);
      gp_Pnt op;
      os->D0(u, v, op);
      e = std::max(e, ptErr(np, op));
    }
  return e;
}

// ═════════════════════════════════════════════════════════════════════════════
// Fixtures — clamped B-spline / NURBS curves and surfaces on [0,1] domains.
// ═════════════════════════════════════════════════════════════════════════════

// Clamped flat knot vector on [0,1]: (p+1) zeros, interior-1 interior knots evenly
// spaced, (p+1) ones. Length = nPoles + p + 1.
static std::vector<double> clampedFlat(int degree, int interior) {
  std::vector<double> flat;
  for (int i = 0; i <= degree; ++i) flat.push_back(0.0);
  for (int i = 1; i < interior; ++i) flat.push_back(double(i) / interior);
  for (int i = 0; i <= degree; ++i) flat.push_back(1.0);
  return flat;
}

// A cubic non-rational B-spline curve with two interior spans (a knot at 0.5).
static nm::BsplineCurveData fixtureCurveBSpline() {
  nm::BsplineCurveData c;
  c.degree = 3;
  c.poles = {{0, 0, 0}, {1, 2, 0}, {2, -1, 1}, {3, 2, 0}, {4, 0, 1}};
  c.knots = clampedFlat(3, 2);  // {0,0,0,0, 0.5, 1,1,1,1}
  return c;
}

// A quadratic rational NURBS curve — 90° circular arc R=2 (classic w=cos45).
static nm::BsplineCurveData fixtureCurveNurbsArc() {
  nm::BsplineCurveData c;
  c.degree = 2;
  const double R = 2.0;
  c.poles = {{R, 0, 0}, {R, R, 0}, {0, R, 0}};
  c.weights = {1.0, std::sqrt(2.0) / 2.0, 1.0};
  c.knots = {0, 0, 0, 1, 1, 1};
  return c;
}

// A rational NURBS curve with an interior knot (so removal/refine have work to do):
// full circle-ish via two quadratic arcs sharing a knot at 0.5.
static nm::BsplineCurveData fixtureCurveNurbsInterior() {
  nm::BsplineCurveData c;
  c.degree = 2;
  const double w = std::sqrt(2.0) / 2.0;
  c.poles = {{2, 0, 0}, {2, 2, 0}, {0, 2, 0}, {-2, 2, 0}, {-2, 0, 0}};
  c.weights = {1.0, w, 1.0, w, 1.0};
  c.knots = {0, 0, 0, 0.5, 0.5, 1, 1, 1};  // interior knot at 0.5, mult 2
  return c;
}

// A bicubic non-rational B-spline surface, gently undulating (4×4 net, no interior).
static nm::BsplineSurfaceData fixtureSurfBSpline() {
  const int dU = 3, dV = 3, nU = 4, nV = 4;
  const double zTab[4][4] = {{0.0, 0.5, 0.5, 0.0}, {0.5, 1.5, 1.5, 0.5},
                             {0.5, 1.5, 1.5, 0.5}, {0.0, 0.5, 0.5, 0.0}};
  nm::BsplineSurfaceData s;
  s.degreeU = dU; s.degreeV = dV; s.nPolesU = nU; s.nPolesV = nV;
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j) s.poles.push_back({double(i), double(j), zTab[i][j]});
  s.knotsU = clampedFlat(dU, 1);  // {0,0,0,0,1,1,1,1}
  s.knotsV = clampedFlat(dV, 1);
  return s;
}

// A rational NURBS surface — quarter-cylinder R=3: rational-quadratic arc (U) ×
// linear extrude (V).
static nm::BsplineSurfaceData fixtureSurfNurbs() {
  const double R = 3.0, w = std::sqrt(2.0) / 2.0;
  const nm::Point3 arc[3] = {{R, 0, 0}, {R, R, 0}, {0, R, 0}};
  nm::BsplineSurfaceData s;
  s.degreeU = 2; s.degreeV = 1; s.nPolesU = 3; s.nPolesV = 2;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j) s.poles.push_back({arc[i].x, arc[i].y, j == 0 ? 0.0 : 4.0});
  const double wa[3] = {1.0, w, 1.0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j) s.weights.push_back(wa[i]);
  s.knotsU = {0, 0, 0, 1, 1, 1};
  s.knotsV = {0, 0, 1, 1};
  return s;
}

// ═════════════════════════════════════════════════════════════════════════════
// CURVE OPS.
// ═════════════════════════════════════════════════════════════════════════════

// insertKnotCurve → Geom_BSplineCurve::InsertKnot.
static void groupInsertCurve() {
  struct Case { const char* name; nm::BsplineCurveData c; double u; int r; };
  const Case cases[] = {
      {"insert bspline u=0.25 r=1", fixtureCurveBSpline(), 0.25, 1},
      {"insert bspline u=0.5  r=1 (existing knot)", fixtureCurveBSpline(), 0.5, 1},
      {"insert bspline u=0.75 r=2", fixtureCurveBSpline(), 0.75, 2},
      {"insert nurbs-arc u=0.5 r=1", fixtureCurveNurbsArc(), 0.5, 1},
      {"insert nurbs-interior u=0.3 r=1", fixtureCurveNurbsInterior(), 0.3, 1},
      {"insert nurbs-interior u=0.5 r=1 (raise mult to deg)", fixtureCurveNurbsInterior(), 0.5, 1},
  };
  for (const auto& k : cases) {
    nm::BsplineCurveData nres = nm::insertKnotCurve(k.c, k.u, k.r);
    Handle(Geom_BSplineCurve) oc = buildCurve(k.c);
    oc->InsertKnot(k.u, k.r);  // Boehm; default ParametricTolerance/Add
    nm::BsplineCurveData ores = occtCurveToData(oc);
    const double rep = curveRepErr(nres, ores);
    const double smp = curveSampleErr(nres, oc, 0.0, 1.0);
    classify(k.name, std::max(rep, smp), std::max(kTolPole, kTolPt));
  }
}

// refineKnotCurve → Geom_BSplineCurve::InsertKnots (whole-vector, Oslo).
static void groupRefineCurve() {
  struct Case { const char* name; nm::BsplineCurveData c; std::vector<double> nk; };
  const Case cases[] = {
      {"refine bspline {0.2,0.4,0.6,0.8}", fixtureCurveBSpline(), {0.2, 0.4, 0.6, 0.8}},
      {"refine nurbs-arc {0.25,0.5,0.75}", fixtureCurveNurbsArc(), {0.25, 0.5, 0.75}},
      {"refine nurbs-interior {0.25,0.75}", fixtureCurveNurbsInterior(), {0.25, 0.75}},
  };
  for (const auto& k : cases) {
    nm::BsplineCurveData nres = nm::refineKnotCurve(k.c, std::span<const double>(k.nk));
    // OCCT InsertKnots takes distinct new knots + the multiplicity to REACH; here
    // each new value is inserted once, so mult=1 and Add=Standard_True (additive).
    std::vector<double> dk;
    std::vector<int> dm;
    flatToKnotsMults(k.nk, dk, dm);
    Handle(Geom_BSplineCurve) oc = buildCurve(k.c);
    oc->InsertKnots(toReal1(dk), toInt1(dm), 0.0, Standard_True);
    nm::BsplineCurveData ores = occtCurveToData(oc);
    const double rep = curveRepErr(nres, ores);
    const double smp = curveSampleErr(nres, oc, 0.0, 1.0);
    classify(k.name, std::max(rep, smp), std::max(kTolPole, kTolPt));
  }
}

// elevateDegreeCurve → Geom_BSplineCurve::IncreaseDegree.
static void groupElevateCurve() {
  struct Case { const char* name; nm::BsplineCurveData c; int t; };
  const Case cases[] = {
      {"elevate bspline t=1", fixtureCurveBSpline(), 1},
      {"elevate bspline t=2", fixtureCurveBSpline(), 2},
      {"elevate nurbs-arc t=1", fixtureCurveNurbsArc(), 1},
      {"elevate nurbs-interior t=1", fixtureCurveNurbsInterior(), 1},
  };
  for (const auto& k : cases) {
    nm::BsplineCurveData nres = nm::elevateDegreeCurve(k.c, k.t);
    Handle(Geom_BSplineCurve) oc = buildCurve(k.c);
    oc->IncreaseDegree(k.c.degree + k.t);
    nm::BsplineCurveData ores = occtCurveToData(oc);
    const double rep = curveRepErr(nres, ores);
    const double smp = curveSampleErr(nres, oc, 0.0, 1.0);
    classify(k.name, std::max(rep, smp), std::max(kTolPole, kTolPt));
  }
}

// removeKnotCurve → Geom_BSplineCurve::RemoveKnot(Index, M, Tol). Built to be
// removable by inserting first (insert↔remove identity), so both sides succeed.
static void groupRemoveCurve() {
  struct Case { const char* name; nm::BsplineCurveData base; double u; };
  const Case cases[] = {
      {"remove bspline u=0.5 (native interior)", fixtureCurveBSpline(), 0.5},
      {"remove nurbs-interior u=0.5", fixtureCurveNurbsInterior(), 0.5},
  };
  for (const auto& k : cases) {
    // Insert u once on both sides so there is a removable knot, then remove it.
    nm::BsplineCurveData inserted = nm::insertKnotCurve(k.base, k.u, 1);
    nm::KnotRemovalResult nr = nm::removeKnotCurve(inserted, k.u, 1, 1e-9);

    Handle(Geom_BSplineCurve) oc = buildCurve(k.base);
    oc->InsertKnot(k.u, 1);
    // Distinct-knot index of u for the OCCT index-addressed RemoveKnot.
    std::vector<double> dk;
    std::vector<int> dm;
    for (int i = 1; i <= oc->NbKnots(); ++i) { dk.push_back(oc->Knot(i)); dm.push_back(oc->Multiplicity(i)); }
    const int idx = knotIndexOf(dk, k.u);
    const int curMult = dm[idx - 1];
    const bool oRemoved = oc->RemoveKnot(idx, curMult - 1, 1e-9);  // drop mult by 1

    nm::BsplineCurveData ores = occtCurveToData(oc);
    const double rep = curveRepErr(nr.curve, ores);
    const double smp = curveSampleErr(nr.curve, oc, 0.0, 1.0);
    // Native must report at least one removal, matching OCCT's success.
    const double consistency = (nr.removed >= 1) == oRemoved ? 0.0 : 1e9;
    classify(k.name, std::max({rep, smp, consistency}), std::max(kTolPole, kTolPt));
  }
}

// splitCurve → two Geom_BSplineCurve::Segment on the sub-intervals.
// Segment may relocate interior knots (see caveat) → compare EVALUATED POINTS
// on the sub-domain, not the raw knot arrays.
static void groupSplitCurve() {
  struct Case { const char* name; nm::BsplineCurveData c; double u; };
  const Case cases[] = {
      {"split bspline u=0.5", fixtureCurveBSpline(), 0.5},
      {"split bspline u=0.3", fixtureCurveBSpline(), 0.3},
      {"split nurbs-arc u=0.5", fixtureCurveNurbsArc(), 0.5},
      {"split nurbs-interior u=0.5", fixtureCurveNurbsInterior(), 0.5},
  };
  for (const auto& k : cases) {
    nm::CurveSplit ns = nm::splitCurve(k.c, k.u);
    Handle(Geom_BSplineCurve) left = buildCurve(k.c);
    Handle(Geom_BSplineCurve) right = buildCurve(k.c);
    left->Segment(0.0, k.u);
    right->Segment(k.u, 1.0);
    // Native `left` piece is parametrised on [0,u]; `right` on [u,1] (design.md:
    // pieces reconstruct C on their sub-domains). OCCT Segment preserves the point
    // map, so compare native-left(t) vs OCCT-left(t) over [0,u] and similarly right.
    const double el = curveSampleErr(ns.left, left, 0.0, k.u);
    const double er = curveSampleErr(ns.right, right, k.u, 1.0);
    classify(k.name, std::max(el, er), kTolPt);
  }
}

// decomposeCurveToBezier → GeomConvert_BSplineCurveToBezierCurve (segment count +
// each Bézier arc re-evaluates to the source curve on its span).
static void groupDecomposeCurve() {
  struct Case { const char* name; nm::BsplineCurveData c; };
  const Case cases[] = {
      {"decompose bspline (2 spans)", fixtureCurveBSpline()},
      {"decompose nurbs-interior (2 spans)", fixtureCurveNurbsInterior()},
  };
  for (const auto& k : cases) {
    std::vector<nm::BsplineCurveData> segs = nm::decomposeCurveToBezier(k.c);
    Handle(Geom_BSplineCurve) oc = buildCurve(k.c);
    GeomConvert_BSplineCurveToBezierCurve conv(oc);
    const int nArcs = conv.NbArcs();
    // Segment-count agreement + each native segment re-evaluates to source on its
    // own [0,1] Bézier span. Cross-check native segment i's endpoints against the
    // source curve at the corresponding distinct-knot span boundaries.
    double e = (static_cast<int>(segs.size()) == nArcs) ? 0.0 : 1e9;
    // Distinct interior spans of the source define the span boundaries.
    std::vector<double> dk;
    std::vector<int> dm;
    flatToKnotsMults(k.c.knots, dk, dm);
    for (std::size_t si = 0; si + 1 < dk.size() && si < segs.size(); ++si) {
      const double a = dk[si], b = dk[si + 1];
      if (b <= a) continue;
      // Native Bézier segment is clamped on [a,b] (design: re-evaluates to source
      // on its span). Sample the source native curve and the segment on [a,b].
      for (int s = 0; s <= 20; ++s) {
        const double u = a + (b - a) * (double(s) / 20.0);
        nm::Point3 src = k.c.weights.empty()
                             ? nm::curvePoint(k.c.degree, k.c.poles, k.c.knots, u)
                             : nm::nurbsCurvePoint(k.c.degree, k.c.poles, k.c.weights, k.c.knots, u);
        const nm::BsplineCurveData& seg = segs[si];
        nm::Point3 sp = seg.weights.empty()
                            ? nm::curvePoint(seg.degree, seg.poles, seg.knots, u)
                            : nm::nurbsCurvePoint(seg.degree, seg.poles, seg.weights, seg.knots, u);
        e = std::max(e, std::max({relErr(sp.x, src.x), relErr(sp.y, src.y), relErr(sp.z, src.z)}));
      }
    }
    classify(k.name, e, kTolPt);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// SURFACE OPS.
// ═════════════════════════════════════════════════════════════════════════════

// insertKnotSurface → InsertUKnot / InsertVKnot.
static void groupInsertSurface() {
  struct Case { const char* name; nm::BsplineSurfaceData s; nm::ParamDir d; double v; int r; };
  const Case cases[] = {
      {"insert-U bspline surf v=0.5 r=1", fixtureSurfBSpline(), nm::ParamDir::U, 0.5, 1},
      {"insert-V bspline surf v=0.25 r=1", fixtureSurfBSpline(), nm::ParamDir::V, 0.25, 1},
      {"insert-U nurbs surf v=0.5 r=1", fixtureSurfNurbs(), nm::ParamDir::U, 0.5, 1},
      {"insert-V nurbs surf v=0.5 r=1", fixtureSurfNurbs(), nm::ParamDir::V, 0.5, 1},
  };
  for (const auto& k : cases) {
    nm::BsplineSurfaceData nres = nm::insertKnotSurface(k.s, k.d, k.v, k.r);
    Handle(Geom_BSplineSurface) os = buildSurface(k.s);
    if (k.d == nm::ParamDir::U) os->InsertUKnot(k.v, k.r, 0.0, Standard_True);
    else os->InsertVKnot(k.v, k.r, 0.0, Standard_True);
    nm::BsplineSurfaceData ores = occtSurfaceToData(os);
    const double rep = surfaceRepErr(nres, ores);
    const double smp = surfaceSampleErr(nres, os, 0, 1, 0, 1);
    classify(k.name, std::max(rep, smp), std::max(kTolPole, kTolPt));
  }
}

// elevateDegreeSurface → Geom_BSplineSurface::IncreaseDegree (per direction).
static void groupElevateSurface() {
  struct Case { const char* name; nm::BsplineSurfaceData s; nm::ParamDir d; int t; };
  const Case cases[] = {
      {"elevate-U bspline surf t=1", fixtureSurfBSpline(), nm::ParamDir::U, 1},
      {"elevate-V bspline surf t=1", fixtureSurfBSpline(), nm::ParamDir::V, 1},
      {"elevate-U nurbs surf t=1", fixtureSurfNurbs(), nm::ParamDir::U, 1},
      {"elevate-V nurbs surf t=1", fixtureSurfNurbs(), nm::ParamDir::V, 1},
  };
  for (const auto& k : cases) {
    nm::BsplineSurfaceData nres = nm::elevateDegreeSurface(k.s, k.d, k.t);
    Handle(Geom_BSplineSurface) os = buildSurface(k.s);
    if (k.d == nm::ParamDir::U) os->IncreaseDegree(k.s.degreeU + k.t, k.s.degreeV);
    else os->IncreaseDegree(k.s.degreeU, k.s.degreeV + k.t);
    nm::BsplineSurfaceData ores = occtSurfaceToData(os);
    const double rep = surfaceRepErr(nres, ores);
    const double smp = surfaceSampleErr(nres, os, 0, 1, 0, 1);
    classify(k.name, std::max(rep, smp), std::max(kTolPole, kTolPt));
  }
}

// removeKnotSurface → RemoveUKnot / RemoveVKnot. Insert first so it is removable.
static void groupRemoveSurface() {
  struct Case { const char* name; nm::BsplineSurfaceData s; nm::ParamDir d; double v; };
  const Case cases[] = {
      {"remove-U bspline surf v=0.5", fixtureSurfBSpline(), nm::ParamDir::U, 0.5},
      {"remove-V nurbs surf v=0.5", fixtureSurfNurbs(), nm::ParamDir::V, 0.5},
  };
  for (const auto& k : cases) {
    nm::BsplineSurfaceData inserted = nm::insertKnotSurface(k.s, k.d, k.v, 1);
    nm::KnotRemovalResultS nr = nm::removeKnotSurface(inserted, k.d, k.v, 1, 1e-9);

    Handle(Geom_BSplineSurface) os = buildSurface(k.s);
    std::vector<double> dk;
    std::vector<int> dm;
    if (k.d == nm::ParamDir::U) {
      os->InsertUKnot(k.v, 1, 0.0, Standard_True);
      for (int i = 1; i <= os->NbUKnots(); ++i) { dk.push_back(os->UKnot(i)); dm.push_back(os->UMultiplicity(i)); }
      const int idx = knotIndexOf(dk, k.v);
      os->RemoveUKnot(idx, dm[idx - 1] - 1, 1e-9);
    } else {
      os->InsertVKnot(k.v, 1, 0.0, Standard_True);
      for (int i = 1; i <= os->NbVKnots(); ++i) { dk.push_back(os->VKnot(i)); dm.push_back(os->VMultiplicity(i)); }
      const int idx = knotIndexOf(dk, k.v);
      os->RemoveVKnot(idx, dm[idx - 1] - 1, 1e-9);
    }
    nm::BsplineSurfaceData ores = occtSurfaceToData(os);
    const double rep = surfaceRepErr(nr.surface, ores);
    const double smp = surfaceSampleErr(nr.surface, os, 0, 1, 0, 1);
    classify(k.name, std::max(rep, smp), std::max(kTolPole, kTolPt));
  }
}

// splitSurface → Geom_BSplineSurface::Segment on the sub-rectangle. Segment may
// relocate knots (see caveat) → compare EVALUATED POINTS on the sub-domain.
static void groupSplitSurface() {
  struct Case { const char* name; nm::BsplineSurfaceData s; nm::ParamDir d; double v; };
  const Case cases[] = {
      {"split-U bspline surf v=0.5", fixtureSurfBSpline(), nm::ParamDir::U, 0.5},
      {"split-V bspline surf v=0.5", fixtureSurfBSpline(), nm::ParamDir::V, 0.5},
      {"split-U nurbs surf v=0.5", fixtureSurfNurbs(), nm::ParamDir::U, 0.5},
  };
  for (const auto& k : cases) {
    nm::SurfaceSplit ns = nm::splitSurface(k.s, k.d, k.v);
    Handle(Geom_BSplineSurface) lo = buildSurface(k.s);
    Handle(Geom_BSplineSurface) hi = buildSurface(k.s);
    if (k.d == nm::ParamDir::U) {
      lo->Segment(0.0, k.v, 0.0, 1.0);
      hi->Segment(k.v, 1.0, 0.0, 1.0);
      const double el = surfaceSampleErr(ns.low, lo, 0.0, k.v, 0.0, 1.0);
      const double eh = surfaceSampleErr(ns.high, hi, k.v, 1.0, 0.0, 1.0);
      classify(k.name, std::max(el, eh), kTolPt);
    } else {
      lo->Segment(0.0, 1.0, 0.0, k.v);
      hi->Segment(0.0, 1.0, k.v, 1.0);
      const double el = surfaceSampleErr(ns.low, lo, 0.0, 1.0, 0.0, k.v);
      const double eh = surfaceSampleErr(ns.high, hi, 0.0, 1.0, k.v, 1.0);
      classify(k.name, std::max(el, eh), kTolPt);
    }
  }
}

int main() {
  std::printf("== nurbs-exact-geometry-kernel native-vs-OCCT parity (SIM gate b) ==\n");
  std::fflush(stdout);

  groupInsertCurve();
  groupRefineCurve();
  groupElevateCurve();
  groupRemoveCurve();
  groupSplitCurve();
  groupDecomposeCurve();

  groupInsertSurface();
  groupElevateSurface();
  groupRemoveSurface();
  groupSplitSurface();

  std::printf("== AGREED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d ==\n",
              g_agree, g_disagree, g_oracle);
  std::fflush(stdout);
  // Bar: DISAGREED == 0. ORACLE-INACCURATE trials are justified inline and do not
  // fail the gate (native is more correct than OCCT there).
  std::_Exit(g_disagree == 0 ? 0 : 1);
}
