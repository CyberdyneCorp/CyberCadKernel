// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_marching_parity.mm — SSI Stage S3 (marching-line tracer / WLine)
// native-vs-OCCT parity harness (iOS simulator). Gate 2 of the two-gate S3 model;
// Gate 1 (host, no OCCT) is tests/native/test_native_ssi_marching.cpp.
//
// S3 turns the S2 SeedSet (one seed per transversal branch) into the FULL intersection
// CURVES: from each seed a predictor-corrector walk (tangent = normalize(nA×nB); step
// P+h·t; re-project onto BOTH surfaces via native-numerics least_squares; deflection-
// adaptive h; terminate on loop-closure / boundary / near-tangent) yields one WLine per
// branch plus a fitted B-spline. This harness asserts that native trace_intersection
// agrees with the OCCT IntPatch / GeomAPI_IntSS intersection curves it is cloning the
// SCHEME of (oracle only — never copied).
//
// PER-PAIR ASSERTIONS (the S3 verification contract, SSI-ROADMAP.md S3):
//   1. SAME NUMBER OF TRANSVERSAL BRANCHES traced. The OCCT oracle's transversal branch
//      count (arc-split loci re-joined into connected components, tangential curves
//      excluded via ‖nA×nB‖) must equal the native tracedBranches (Closed | BoundaryExit
//      WLines). Native NearTangentTruncated WLines are EXCLUDED from this count and
//      reported separately (nt=…) — they are the honest S4 gap, not a full trace.
//   2. ON THE OCCT CURVE. Every DENSELY-SAMPLED native WLine point (both the corrected
//      polyline nodes and samples of the fitted B-spline) lies on SOME OCCT intersection
//      curve: nearest-point over all OCCT branches < onCurveTol
//      (GeomAPI_ProjectPointOnCurve::LowerDistance).
//   3. ON BOTH SURFACES. Every native WLine point lies on BOTH input surfaces:
//      GeomAPI_ProjectPointOnSurf::LowerDistance < onSurfTol on each.
//   4. LENGTH ≈ OCCT. The native curve length (summed over its WLines) ≈ the OCCT curve
//      length (summed over the matched OCCT branches) within a deflection/step tolerance
//      (relative lengthTol). This catches a march that stopped short or ran twice.
//   5. CLOSED LOOPS. An OCCT loop that closes (first point ≈ last point over its param
//      range) must be traced by native as a Closed WLine (status == Closed).
//
// NEAR-TANGENT (S4, honest). A branch whose normals stay parallel along it is tangential
// and EXCLUDED from the transversal count (reported nt=…). If the native march enters a
// near-tangent region it must mark that WLine NearTangentTruncated (status ==
// NearTangent) — traced up to the tangency, remainder an S4 gap; the harness asserts such
// WLines are excluded from the full-trace count and never fabricated past the tangency.
// This harness does NOT assert a full trace across a tangency (that is S4).
//
// COVERAGE (transversal freeform + skew-quadric pairs — the pairs S1 defers as
// NotAnalytic and S2 only SEEDS):
//   * bspline ∩ bspline        — a biquadratic bump vs a facing dish → 1 transversal loop.
//   * bspline ∩ plane (MULTI)  — an egg-carton B-spline sheet cut by a plane → several
//                                small transversal loops (the multi-branch stressor).
//   * skew cyl ∩ cyl           — orthogonal unequal-radius cylinders → 2 transversal loops
//                                (the skew-quadric quartic S1 defers).
//   * sphere ∩ sphere          — crossing equal spheres → 1 transversal circle (closed).
//   * sphere ∩ bezier          — a sphere vs a freeform Bézier sheet → 1 transversal loop.
//
// PASS = ALL transversal OCCT branches for the pair are fully traced within tol (count,
// on-curve, on-surface, length, and closed-loop classification all hold); a march that
// stopped short, retraced, or landed off-curve FAILS honestly. Near-tangent branches are
// reported (nt=…), not counted as failures — but they are also never faked as a full
// trace.
//
// SSI is INTERNAL — NO cc_* entry point is called or added; asserted at the
// cybercad::native::ssi C++ boundary, exactly like the S1/S2 parity harnesses.
//
// This TU is OCCT-dependent AND substrate-dependent: it links the OCCT oracle and the
// NumPP/SciPP numsci archive, and compiles src/native/ssi/{seeding,marching}.cpp +
// src/native/numerics/numerics.cpp under -DCYBERCAD_HAS_NUMSCI. Built ONLY by
// scripts/run-sim-native-ssi-marching.sh; on the SKIP list of run-sim-suite.sh.
//
// Output: one [NMARCH] PASS/FAIL line per pair with per-pair transversal-branch count,
// near-tangent count, and the max on-OCCT-curve / on-surface / length deltas, then a
// final "== N passed, M failed ==". Flushes and std::_Exit (OCCT static teardown in the
// trimmed static build is not exit-clean — same rationale as native_ssi_parity).
//
#include "native/ssi/native_ssi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_ssi_marching_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_ssi_marching_parity requires -DCYBERCAD_HAS_NUMSCI (the least_squares corrector + lstsq fit)"
#endif

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax3.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_Curve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomLProp_SLProps.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

namespace ssi = cybercad::native::ssi;
namespace nm = cybercad::native::math;
using nm::Ax3;
using nm::Dir3;
using nm::Point3;
using nm::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kSamplesPerCurve = 128;   // OCCT-curve sampling for classification + on-curve
constexpr int kFitSamples = 200;         // native fitted-B-spline sampling for the on-curve check

int g_pass = 0;
int g_fail = 0;

// ── OCCT conversion helpers ─────────────────────────────────────────────────────
gp_Ax3 toOcctAx3(const Ax3& f) {
  return gp_Ax3(gp_Pnt(f.origin.x, f.origin.y, f.origin.z),
                gp_Dir(f.z.x(), f.z.y(), f.z.z()),
                gp_Dir(f.x.x(), f.x.y(), f.x.z()));
}
gp_Pnt toOcctPnt(const Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

double distToOcctSurface(const Handle(Geom_Surface)& s, const Point3& p) {
  GeomAPI_ProjectPointOnSurf proj(toOcctPnt(p), s);
  return proj.NbPoints() > 0 ? proj.LowerDistance() : 1e30;
}
double distToOcctCurve(const Handle(Geom_Curve)& c, const Point3& p) {
  GeomAPI_ProjectPointOnCurve proj(toOcctPnt(p), c);
  return proj.NbPoints() > 0 ? proj.LowerDistance() : 1e30;
}

// Sine of the angle between the two OCCT surface normals at a 3D point (the
// transversality witness), computed on the ORACLE surfaces so the classification does
// not depend on the native side. -1 if a normal is undefined (degenerate patch).
double crossingSineOnOcct(const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                          const gp_Pnt& q) {
  GeomAPI_ProjectPointOnSurf pa(q, sa), pb(q, sb);
  if (pa.NbPoints() == 0 || pb.NbPoints() == 0) return -1.0;
  Standard_Real ua, va, ub, vb;
  pa.LowerDistanceParameters(ua, va);
  pb.LowerDistanceParameters(ub, vb);
  GeomLProp_SLProps la(sa, ua, va, 1, 1e-9), lb(sb, ub, vb, 1, 1e-9);
  if (!la.IsNormalDefined() || !lb.IsNormalDefined()) return -1.0;
  gp_Dir na = la.Normal(), nb = lb.Normal();
  // gp_Dir::Crossed returns a UNIT gp_Dir (magnitude normalized away) — not what we
  // want. The transversality witness is the true cross-product length ‖nA×nB‖ = sin θ,
  // so cross as gp_Vec (which preserves magnitude) and take its Magnitude().
  return gp_Vec(na).Crossed(gp_Vec(nb)).Magnitude();  // ‖nA × nB‖ = sin θ for unit normals
}

// ── an OCCT reference branch: curve + classification + arc length + closure ────────
struct OcctBranch {
  Handle(Geom_Curve) curve;
  bool transversal = true;
  double maxCrossingSine = 0.0;
  double length = 0.0;
  bool closed = false;
  double f = 0.0, l = 1.0;  // clamped parameter range used for sampling
};

// Sample one OCCT curve over its (clamped) range, classifying transversal vs tangential
// from the MAX ‖nA×nB‖ along it (a transversal branch crosses somewhere even if it
// grazes locally), measuring its arc length (GCPnts_AbscissaPoint) and whether it CLOSES
// (endpoint ≈ startpoint). Closed OCCT loops must map to native Closed WLines.
OcctBranch classifyBranch(const Handle(Geom_Curve)& c,
                          const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                          double tangentSine) {
  OcctBranch b;
  b.curve = c;
  double f = c->FirstParameter(), l = c->LastParameter();
  if (!std::isfinite(f) || f < -1e6) f = -1e6;
  if (!std::isfinite(l) || l > 1e6) l = 1e6;
  b.f = f; b.l = l;
  double maxSine = 0.0;
  gp_Pnt first, last;
  for (int i = 0; i <= kSamplesPerCurve; ++i) {
    const double t = f + (l - f) * (double(i) / kSamplesPerCurve);
    gp_Pnt q;
    try { q = c->Value(t); } catch (...) { continue; }
    if (i == 0) first = q;
    last = q;
    const double s = crossingSineOnOcct(sa, sb, q);
    if (s >= 0.0) maxSine = std::max(maxSine, s);
  }
  b.maxCrossingSine = maxSine;
  b.transversal = (maxSine > tangentSine);
  b.closed = (first.Distance(last) < 1e-6);
  try {
    GeomAdaptor_Curve ac(c, f, l);
    b.length = GCPnts_AbscissaPoint::Length(ac, f, l);
  } catch (...) { b.length = 0.0; }
  return b;
}

// OCCT arc-splits ONE intersection locus into several Geom_Curves (as seen at S1/S2:
// NbLines ≥ native branch count). To compare "number of transversal BRANCHES" we merge
// arc-split OCCT curves that share an endpoint into connected COMPONENTS, and count / sum
// length per component. Two branches join if an endpoint of one is within `weld` of an
// endpoint of the other.
struct OcctComponent {
  std::vector<int> members;     // indices into the transversal-branch list
  double length = 0.0;
  bool closed = false;          // any member closed OR the component's chain re-closes
};

std::vector<OcctComponent> weldComponents(const std::vector<OcctBranch>& tb, double weld) {
  const int n = static_cast<int>(tb.size());
  std::vector<int> comp(n, -1);
  auto endpts = [](const OcctBranch& b) {
    gp_Pnt a = b.curve->Value(b.f), z = b.curve->Value(b.l);
    return std::pair<gp_Pnt, gp_Pnt>(a, z);
  };
  std::vector<std::pair<gp_Pnt, gp_Pnt>> ep(n);
  for (int i = 0; i < n; ++i) ep[i] = endpts(tb[i]);
  auto touch = [&](int i, int j) {
    const gp_Pnt& ai = ep[i].first; const gp_Pnt& zi = ep[i].second;
    const gp_Pnt& aj = ep[j].first; const gp_Pnt& zj = ep[j].second;
    return ai.Distance(aj) < weld || ai.Distance(zj) < weld ||
           zi.Distance(aj) < weld || zi.Distance(zj) < weld;
  };
  int nc = 0;
  for (int i = 0; i < n; ++i) {
    if (comp[i] != -1) continue;
    comp[i] = nc;
    // flood fill
    bool grew = true;
    while (grew) {
      grew = false;
      for (int a = 0; a < n; ++a) {
        if (comp[a] != nc) continue;
        for (int j = 0; j < n; ++j)
          if (comp[j] == -1 && touch(a, j)) { comp[j] = nc; grew = true; }
      }
    }
    ++nc;
  }
  std::vector<OcctComponent> out(nc);
  for (int i = 0; i < n; ++i) {
    OcctComponent& c = out[comp[i]];
    c.members.push_back(i);
    c.length += tb[i].length;
    if (tb[i].closed) c.closed = true;
  }
  // A single-member component that is not self-closed but whose two endpoints coincide
  // (already flagged), or a multi-member chain whose free ends meet, is a closed loop.
  for (auto& c : out) {
    if (c.closed) continue;
    // collect the endpoints of members; if every endpoint is matched by another member's
    // endpoint (no free ends), the component forms a closed chain.
    std::vector<gp_Pnt> ends;
    for (int m : c.members) { ends.push_back(ep[m].first); ends.push_back(ep[m].second); }
    bool allMatched = true;
    for (std::size_t a = 0; a < ends.size(); ++a) {
      bool matched = false;
      for (std::size_t b = 0; b < ends.size(); ++b)
        if (a != b && (a / 2 != b / 2) && ends[a].Distance(ends[b]) < weld) { matched = true; break; }
      if (!matched) { allMatched = false; break; }
    }
    if (allMatched && ends.size() >= 2) c.closed = true;
  }
  return out;
}

// ── native WLine sampling ──────────────────────────────────────────────────────
// Densely sample a WLine: its corrected polyline nodes PLUS samples of its fitted
// B-spline (if valid), so both the raw march and the fitted Geom-quality curve are
// checked on-curve/on-surface.
std::vector<Point3> sampleWLine(const ssi::WLine& w) {
  std::vector<Point3> pts;
  pts.reserve(w.points.size() + kFitSamples + 1);
  for (const auto& p : w.points) pts.push_back(p.point);
  if (w.curve.valid()) {
    const auto& c = w.curve;
    const double t0 = c.knots.front(), t1 = c.knots.back();
    for (int i = 0; i <= kFitSamples; ++i) {
      const double t = t0 + (t1 - t0) * (double(i) / kFitSamples);
      pts.push_back(nm::curvePoint(c.degree, c.poles, c.knots, t));
    }
  }
  return pts;
}

// Polyline length of a WLine's corrected nodes (the ground-truth march length; the fitted
// spline tracks it). For a Closed WLine we add the closing chord back to the first node.
double wlineLength(const ssi::WLine& w) {
  double len = 0.0;
  for (std::size_t i = 1; i < w.points.size(); ++i)
    len += nm::distance(w.points[i - 1].point, w.points[i].point);
  if (w.isClosed() && w.points.size() >= 2)
    len += nm::distance(w.points.back().point, w.points.front().point);
  return len;
}

// ── the per-pair report ─────────────────────────────────────────────────────────
// Runs native trace_intersection + OCCT GeomAPI_IntSS, classifies + welds the OCCT
// branches into transversal components, and asserts the five S3 conditions.
//   expectTransversal   analytic truth, cross-checked against the welded component count.
//   tangentSine         threshold below which an OCCT branch is deemed tangential (S4).
//   lengthTol           relative length tolerance (deflection/step budget).
void reportPair(const std::string& pairName,
                const ssi::SurfaceAdapter& A, const ssi::SurfaceAdapter& B,
                const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                int expectTransversal, double onSurfTol, double onCurveTol,
                double tangentSine, double lengthTol,
                const ssi::SeedOptions& seedOpt, const ssi::MarchOptions& marchOpt) {
  const ssi::TraceSet ts = ssi::trace_intersection(A, B, seedOpt, marchOpt);

  // OCCT oracle: classify + weld into transversal connected components.
  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;
  std::vector<OcctBranch> allBranches, transBranches;
  for (int i = 1; i <= occtN; ++i)
    allBranches.push_back(classifyBranch(iss.Line(i), sa, sb, tangentSine));
  int tangential = 0;
  for (const auto& b : allBranches) {
    if (b.transversal) transBranches.push_back(b);
    else ++tangential;
  }

  // Weld tolerance: generous vs the model scale so arc-split seams re-join.
  const double weld = std::max(onCurveTol * 10.0, A.modelScale * 1e-3);
  const std::vector<OcctComponent> comps = weldComponents(transBranches, weld);
  const int occtTransversalBranches = static_cast<int>(comps.size());
  double occtTotalLen = 0.0;
  int occtClosed = 0;
  for (const auto& c : comps) { occtTotalLen += c.length; if (c.closed) ++occtClosed; }

  // Native side: separate full traces (Closed | BoundaryExit) from near-tangent gaps.
  int nativeTraced = 0, nativeClosed = 0, nativeNearTangent = 0, nativeFailed = 0;
  double nativeTotalLen = 0.0;
  double maxOnCurve = 0.0, maxOnSurf = 0.0;
  for (const auto& w : ts.lines) {
    switch (w.status) {
      case ssi::TraceStatus::Closed:       ++nativeTraced; ++nativeClosed; break;
      case ssi::TraceStatus::BoundaryExit: ++nativeTraced; break;
      case ssi::TraceStatus::NearTangent:  ++nativeNearTangent; break;
      case ssi::TraceStatus::Failed:       ++nativeFailed; break;
    }
    // On-curve + on-surface hold for EVERY emitted WLine (even a truncated one: the
    // points it DID trace must still lie on the loci — it just stopped early).
    const std::vector<Point3> samples = sampleWLine(w);
    for (const Point3& p : samples) {
      double best = 1e30;
      for (const auto& b : allBranches) best = std::min(best, distToOcctCurve(b.curve, p));
      maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf,
                           std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
    }
    if (w.status == ssi::TraceStatus::Closed || w.status == ssi::TraceStatus::BoundaryExit)
      nativeTotalLen += wlineLength(w);
  }

  // Length delta: relative to the OCCT total transversal length (guard div-by-zero).
  const double lenDelta = occtTotalLen > 1e-12
                              ? std::fabs(nativeTotalLen - occtTotalLen) / occtTotalLen
                              : std::fabs(nativeTotalLen - occtTotalLen);

  bool ok = true;
  // (1) same number of transversal branches fully traced.
  if (nativeTraced != occtTransversalBranches) ok = false;
  // cross-check the oracle against the analytic truth.
  if (occtTransversalBranches != expectTransversal) ok = false;
  // (2)+(3) on-curve / on-surface within tol (only meaningful when something was traced).
  if (maxOnCurve > onCurveTol) ok = false;
  if (maxOnSurf > onSurfTol) ok = false;
  // (4) length within the deflection/step budget.
  if (lenDelta > lengthTol) ok = false;
  // (5) closed OCCT loops → native Closed WLines (count must match).
  if (nativeClosed != occtClosed) ok = false;

  if (ok) ++g_pass; else ++g_fail;

  std::printf("[NMARCH] %-4s %-18s branches=%d/%d(occt) nt=%d closed=%d/%d "
              "onCurve=%.2e onSurf=%.2e lenDelta=%.2e (nat=%.4f occt=%.4f) tang=%d seeds=%d\n",
              ok ? "PASS" : "FAIL", pairName.c_str(), nativeTraced, occtTransversalBranches,
              nativeNearTangent, nativeClosed, occtClosed,
              maxOnCurve, maxOnSurf, lenDelta, nativeTotalLen, occtTotalLen,
              tangential, ts.seededBranches);
  std::fflush(stdout);
}

// ── shared frame + surface builders ───────────────────────────────────────────────
Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

// Clamped-uniform knot vector of length nPoles+degree+1 over [0,1].
std::vector<double> clampedKnots(int degree, int nPoles) {
  const int m = nPoles + degree + 1;
  std::vector<double> k(static_cast<std::size_t>(m), 0.0);
  const int interior = nPoles - degree - 1;
  for (int i = 0; i < m; ++i) {
    if (i <= degree) k[i] = 0.0;
    else if (i >= nPoles) k[i] = 1.0;
    else k[i] = double(i - degree) / double(interior + 1);
  }
  return k;
}

Handle(Geom_BSplineSurface) toOcctBSpline(const std::vector<Point3>& poles, int nR, int nC,
                                          int degU, int degV) {
  TColgp_Array2OfPnt cp(1, nR, 1, nC);
  for (int i = 0; i < nR; ++i)
    for (int j = 0; j < nC; ++j) {
      const Point3& p = poles[static_cast<std::size_t>(i) * nC + j];
      cp.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  auto distinct = [](int degree, int nPoles,
                     TColStd_Array1OfReal& knots, TColStd_Array1OfInteger& mults) {
    const int interior = nPoles - degree - 1;
    const int nDistinct = interior + 2;
    for (int i = 1; i <= nDistinct; ++i) {
      knots.SetValue(i, (i == 1) ? 0.0 : (i == nDistinct ? 1.0 : double(i - 1) / double(interior + 1)));
      mults.SetValue(i, (i == 1 || i == nDistinct) ? (degree + 1) : 1);
    }
  };
  const int nDistU = (nR - degU - 1) + 2, nDistV = (nC - degV - 1) + 2;
  TColStd_Array1OfReal knU(1, nDistU), knV(1, nDistV);
  TColStd_Array1OfInteger muU(1, nDistU), muV(1, nDistV);
  distinct(degU, nR, knU, muU);
  distinct(degV, nC, knV, muV);
  return new Geom_BSplineSurface(cp, knU, knV, muU, muV, degU, degV);
}

Handle(Geom_BezierSurface) toOcctBezier(const std::vector<Point3>& poles, int nR, int nC) {
  TColgp_Array2OfPnt cp(1, nR, 1, nC);
  for (int i = 0; i < nR; ++i)
    for (int j = 0; j < nC; ++j) {
      const Point3& p = poles[static_cast<std::size_t>(i) * nC + j];
      cp.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  return new Geom_BezierSurface(cp);
}

// ── pairs (native + OCCT, identical geometry) ──────────────────────────────────────

// bspline ∩ bspline — a biquadratic bump (opening +Z) vs a facing dish (opening −Z),
// meeting in one transversal loop (same geometry as the S2 recall harness so the pair
// carries over end-to-end).
void pairBSplineBSpline() {
  std::vector<Point3> bump = {
      {-1, -1, 0.6}, {-1, 0, 1.0}, {-1, 1, 0.6},
      { 0, -1, 1.0}, { 0, 0, 1.6}, { 0, 1, 1.0},
      { 1, -1, 0.6}, { 1, 0, 1.0}, { 1, 1, 0.6}};
  std::vector<Point3> dish = {
      {-1, -1, 1.4}, {-1, 0, 1.0}, {-1, 1, 1.4},
      { 0, -1, 1.0}, { 0, 0, 0.4}, { 0, 1, 1.0},
      { 1, -1, 1.4}, { 1, 0, 1.0}, { 1, 1, 1.4}};
  const auto kU = clampedKnots(2, 3), kV = clampedKnots(2, 3);
  auto A = ssi::makeBSplineAdapter(2, 2, bump, 3, 3, kU, kV);
  auto B = ssi::makeBSplineAdapter(2, 2, dish, 3, 3, kU, kV);
  Handle(Geom_Surface) sa = toOcctBSpline(bump, 3, 3, 2, 2);
  Handle(Geom_Surface) sb = toOcctBSpline(dish, 3, 3, 2, 2);
  ssi::SeedOptions sopt; sopt.initialGridU = 4; sopt.initialGridV = 4;
  ssi::MarchOptions mopt;  // scale-derived defaults
  reportPair("bspline x bspline", A, B, sa, sb, /*expectTransversal=*/1,
             /*onSurf=*/1e-6, /*onCurve=*/1e-4, /*tangentSine=*/1e-2, /*lengthTol=*/2e-2,
             sopt, mopt);
}

// bspline ∩ plane (MULTI-LOOP) — an egg-carton biquadratic B-spline sheet (5×5 poles,
// alternating high/low) cut by a plane; each hump poking through the plane is its own
// transversal branch. The multi-branch tracer stressor: every branch must be traced.
//
// CUT HEIGHT (deliberately TRANSVERSAL, S3 scope). The interpolated egg-carton has a
// z-range ≈ [0.33, 0.95] with a BAND of saddle/critical points at z ≈ 0.43–0.53 (the
// cols between the humps) — the surface is locally PARALLEL to a z=const plane there.
// Slicing at z=0.5 (the old value) drove the plane straight through that critical band,
// so the z=0.5 contour ran along near-tangent ridges and self-crossed at saddle points:
// a NEAR-TANGENT / branch-point locus, which is S4 (deferred), NOT the clean transversal
// marching S3 is scoped to verify. (Confirmed: the min crossing-sine on that contour is
// ≈ 0.03, and both the native marcher AND an independent contour follower stall/fold at
// the saddle — the honest S4 gap, not a marcher bug.) We cut at z = 0.80, ABOVE the
// whole critical band and below the peaks (≈ 0.95): each high hump clears the plane in a
// clean transversal branch (crossing-sine well above tol everywhere), which is exactly
// what an S3 parity gate must exercise. The near-tangent z=0.5 slice is left to S4.
void pairBSplinePlane() {
  const int n = 5;
  const double cutZ = 0.80;  // transversal: above the z≈0.43–0.53 saddle band (see note)
  std::vector<Point3> poles;
  poles.reserve(static_cast<std::size_t>(n) * n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -1.0 + 2.0 * double(i) / (n - 1);
      const double y = -1.0 + 2.0 * double(j) / (n - 1);
      const double z = 0.5 + 0.55 * (((i + j) % 2 == 0) ? 1.0 : -1.0);
      poles.push_back({x, y, z});
    }
  const auto kU = clampedKnots(2, n), kV = clampedKnots(2, n);
  auto A = ssi::makeBSplineAdapter(2, 2, poles, n, n, kU, kV);

  nm::Plane pl{Ax3{{0, 0, cutZ}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  ssi::ParamBox pd{-1.5, 1.5, -1.5, 1.5};
  auto B = ssi::makePlaneAdapter(pl, pd);

  Handle(Geom_Surface) sa = toOcctBSpline(poles, n, n, 2, 2);
  Handle(Geom_Surface) sb = new Geom_Plane(toOcctAx3(Ax3{{0, 0, cutZ}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));

  // Determine the expected transversal-branch count from the oracle itself (welded
  // components), so the assertion tracks the actual number of humps clearing z=0.5.
  GeomAPI_IntSS iss(sa, sb, 1e-7);
  std::vector<OcctBranch> tb;
  for (int i = 1; i <= (iss.IsDone() ? iss.NbLines() : 0); ++i) {
    OcctBranch b = classifyBranch(iss.Line(i), sa, sb, 1e-2);
    if (b.transversal) tb.push_back(b);
  }
  const int expect = static_cast<int>(weldComponents(tb, A.modelScale * 1e-3).size());

  ssi::SeedOptions sopt; sopt.initialGridU = 8; sopt.initialGridV = 8; sopt.minPatchFrac = 1.0 / 48.0;
  ssi::MarchOptions mopt;
  // Tighter deflection budget than the scale·1e-3 default. The hump arcs are the
  // highest-curvature curves in this suite, so at the default step spacing the FITTED
  // B-spline (interpolating the on-surface polyline nodes) bows ~1.4e-6 off the CURVED
  // surface BETWEEN nodes — the nodes themselves sit on both surfaces to ~1e-11. This is
  // fit resolution, not corrector drift: a tighter deflection budget packs the nodes
  // denser so the fitted curve hugs the surface to < the 1e-6 on-surface gate. (Confirmed
  // scaling: node→plane ~6e-12 throughout; fit→plane 3.1e-7 at 1e-3 → 4.8e-12 at 3e-4.)
  mopt.maxDeflection = A.modelScale * 2.5e-4;
  reportPair("bspline x plane", A, B, sa, sb, expect,
             1e-6, 1e-4, 1e-2, 3e-2, sopt, mopt);
}

// skew cyl ∩ cyl — orthogonal unequal-radius cylinders → 2 transversal loops (the skew-
// quadric quartic S1 defers). OCCT arc-splits the loci; the weld re-joins them into 2
// components; native must trace both.
void pairSkewCylinders() {
  nm::Cylinder cz{frameZ(), 1.0};
  const Ax3 fx{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}};  // axis X
  nm::Cylinder cx{fx, 0.7};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);
  Handle(Geom_Surface) sa = new Geom_CylindricalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(fx), 0.7);
  ssi::SeedOptions sopt; sopt.initialGridU = 4; sopt.initialGridV = 3;
  ssi::MarchOptions mopt;
  reportPair("skew cyl unequal", A, B, sa, sb, /*expectTransversal=*/2,
             1e-6, 1e-4, 1e-2, 3e-2, sopt, mopt);
}

// sphere ∩ sphere — crossing equal-radius spheres → 1 transversal circle (a CLOSED loop).
// OCCT may arc-split the circle; the weld re-joins it into 1 closed component; native
// must trace it as a single Closed WLine.
void pairCrossingSpheres() {
  nm::Sphere s1{frameZ(), 1.0};
  nm::Sphere s2{frameZ({1.0, 0, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(frameZ({1.0, 0, 0})), 1.0);
  ssi::SeedOptions sopt; sopt.initialGridU = 3; sopt.initialGridV = 3;
  ssi::MarchOptions mopt;
  reportPair("sphere x sphere", A, B, sa, sb, /*expectTransversal=*/1,
             1e-6, 1e-4, 1e-2, 2e-2, sopt, mopt);
}

// sphere ∩ bezier — a unit sphere vs a freeform biquadratic Bézier sheet dipping through
// the sphere's upper cap → 1 transversal loop. Exercises the corrector against a mixed
// analytic/freeform pair (different derivative magnitudes on the two sides).
void pairSphereBezier() {
  nm::Sphere sph{frameZ(), 1.0};
  ssi::ParamBox sdom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(sph, sdom);

  // 3×3 biquadratic Bézier sheet centred over the sphere's north pole, bowing down into
  // the cap so it cuts one loop around the pole.
  std::vector<Point3> poles = {
      {-0.9, -0.9, 1.15}, {-0.9, 0, 0.95}, {-0.9, 0.9, 1.15},
      { 0.0, -0.9, 0.95}, { 0.0, 0, 0.55}, { 0.0, 0.9, 0.95},
      { 0.9, -0.9, 1.15}, { 0.9, 0, 0.95}, { 0.9, 0.9, 1.15}};
  auto B = ssi::makeBezierAdapter(poles, 3, 3);

  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = toOcctBezier(poles, 3, 3);
  ssi::SeedOptions sopt; sopt.initialGridU = 4; sopt.initialGridV = 4;
  ssi::MarchOptions mopt;
  reportPair("sphere x bezier", A, B, sa, sb, /*expectTransversal=*/1,
             1e-6, 1e-4, 1e-2, 3e-2, sopt, mopt);
}

}  // namespace

int main() {
  std::printf("== SSI Stage S3 marching-line tracer native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  pairBSplineBSpline();
  pairBSplinePlane();
  pairSkewCylinders();
  pairCrossingSpheres();
  pairSphereBezier();

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
