// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_marching_parity.mm — SSI Stage S3 (marching-line tracer / WLine)
// native-vs-OCCT parity harness (iOS simulator). Gate 2 of the two-gate S3 model;
// Gate 1 (host, no OCCT) is tests/native/test_native_ssi_marching.cpp.
//
// Also carries the S4-c / S4-d / S4-e gate-2 cases (same TU): S4-c near-tangent crossing,
// S4-d Steinmetz branch routing, and S4-e CHART SINGULARITIES — a sphere parametric pole
// (v=±π/2) / cone apex (signed radius = 0) crossed by the point-based corrector. The S4-e
// cases FORCE marching (trace_from_seeds with a hand seed) so the analytic pair does not
// short-circuit to a closed form, and assert the fully-traced native curve lies on the OCCT
// GeomAPI_IntSS locus + on both surfaces within tol (Gate-1 host suite is
// tests/native/test_native_ssi_s4e_singularities.cpp).
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
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
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

// ── M1c DOMAIN-CLIPPED ORACLE ─────────────────────────────────────────────────────
// An OCCT Geom_Cylindrical/Conical/SphericalSurface is INFINITE (unbounded height / both
// nappes / the full latitude band); the native adapters are FINITE patches over a ParamBox.
// When an unbounded quadric pierces the other operand more than once along its infinite extent
// GeomAPI_IntSS returns the full multi-loop INFINITE locus — an oracle the finite native trace
// legitimately cannot match. Wrapping the oracle surface in a Geom_RectangularTrimmedSurface
// trimmed to the SAME [u0,u1]×[v0,v1] the native adapter uses makes GeomAPI_IntSS produce the
// SAME finite locus the native trace covers — an apples-to-apples oracle. This is a TEST-HARNESS
// fix only; it does not touch src/native and never widens a tolerance. The native adapters and
// the OCCT surfaces share the (u=angle, v=height/latitude) parameterisation, so the box maps 1:1.
Handle(Geom_Surface) clipOracle(const Handle(Geom_Surface)& s, const ssi::ParamBox& box) {
  return new Geom_RectangularTrimmedSurface(s, box.u0, box.u1, box.v0, box.v1, Standard_True,
                                            Standard_True);
}

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
      case ssi::TraceStatus::BranchArc:    ++nativeTraced; break;  // S4-d branch-to-branch arc
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

// ── M1b BREADTH: general non-coaxial / skew analytic quadric poses vs OCCT ────────────
// The S1 closed-form dispatch defers all of these as NotAnalytic (no elementary closed
// form); the S2 seeder + S3 marcher trace them. These cases verify each pose against
// GeomAPI_IntSS via the shipped reportPair harness (count/type/closed/on-curve/on-surface/
// arc-length). Poses are shared with the host cases (test_native_ssi_marching.cpp) so the
// two gates agree.
//
// ORACLE-SETUP NOTE (which families are DECISIVELY verified here, and the declined tail).
// An OCCT Geom_CylindricalSurface / Geom_ConicalSurface is INFINITE (both cone nappes,
// unbounded height); the native adapters are BOUNDED patches (v ∈ [v0,v1], one nappe). When
// an unbounded quadric can pierce the other operand MORE THAN ONCE along its infinite extent,
// GeomAPI_IntSS returns the full multi-loop INFINITE locus that the finite native trace
// legitimately cannot match without domain-clipped oracle surfaces
// (Geom_RectangularTrimmedSurface). Measured on the sim: cyl∩cone off-axis → oracle arc-length
// ≈ 66 across the unbounded cone; cone∩cone → 2 components on the two nappes; off-axis
// sphere∩cyl → the infinite cylinder pierces the sphere on BOTH sides (2 loops, and at coarse
// seeding the second loop is a SEEDING-RECALL miss). These are the DECLINED tail — the
// sharpened next blocker for this breadth track (clipped-oracle surfaces + seeding recall).
//
// The two families below pair against a FINITE operand whose intersection is a SINGLE loop
// regardless of the infinite extent: (1) skew cyl∩cyl bounded by the smaller finite
// cylinder's single penetration region, and (2) off-axis sphere∩cone where the FINITE sphere
// admits only the near cone nappe once. For these the native finite trace and the OCCT locus
// coincide and the parity is decisive (matching count/type/closed-count, ≈1e-5 deltas).

// Orthonormal Ax3 from origin + main axis (Gram-Schmidt on a non-parallel reference X) —
// the native side; the OCCT side is built with the SAME frame via toOcctAx3.
static Ax3 axFromZ(Point3 o, Vec3 z) {
  const double zn = std::sqrt(z.x * z.x + z.y * z.y + z.z * z.z);
  z = Vec3{z.x / zn, z.y / zn, z.z / zn};
  const Vec3 ref = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 x = nm::cross(ref, z);
  const double xn = std::sqrt(x.x * x.x + x.y * x.y + x.z * x.z);
  x = Vec3{x.x / xn, x.y / xn, x.z / xn};
  const Vec3 y = nm::cross(z, x);
  return Ax3{o, Dir3{x.x, x.y, x.z}, Dir3{y.x, y.y, y.z}, Dir3{z.x, z.y, z.z}};
}

// GENERAL SKEW cyl∩cyl — axes neither parallel nor intersecting (gap 0.4 along +y) AND
// oblique (60° tilt); the smaller cylinder fully penetrates the big one in a SINGLE crossing
// region → ONE connected quartic loop (distinct from the symmetric orthogonal-intersecting
// two-loop case above). The intersection is bounded by the penetration, so the finite native
// trace matches the OCCT locus. A TIGHT deflection budget (like bspline×plane) packs the fit
// nodes dense enough that the fitted B-spline hugs the curved surfaces to < the on-surf gate.
void pairSkewCylindersGeneral() {
  const Ax3 fz = frameZ();
  const Ax3 fx = axFromZ({0, 0.4, 0}, Vec3{std::sin(kPi / 3), 0, std::cos(kPi / 3)});
  nm::Cylinder cz{fz, 1.0};
  nm::Cylinder cx{fx, 0.7};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -3.0, 3.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);
  Handle(Geom_Surface) sa = new Geom_CylindricalSurface(toOcctAx3(fz), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(fx), 0.7);
  ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6;
  ssi::MarchOptions mopt; mopt.maxDeflection = A.modelScale * 2.5e-4;  // dense fit nodes
  reportPair("skew cyl general", A, B, sa, sb, /*expectTransversal=*/1,
             1e-4, 1e-4, 1e-2, 3e-2, sopt, mopt);
}

// OFF-AXIS sphere∩cone — cone axis offset from the sphere centre. The intersection is bounded
// by the FINITE sphere (only the near cone nappe reaches it) → one closed loop; native and the
// OCCT locus coincide. Tight deflection for the on-surf fit gate.
void pairSphereConeOffAxis() {
  const Ax3 fs = frameZ();
  const Ax3 fk = axFromZ({0.3, 0, -2}, Vec3{0.1, 0, 1});
  nm::Sphere sp{fs, 1.0};
  nm::Cone co{fk, 0.05, 0.3};
  ssi::ParamBox ds{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox dk{0.0, 2.0 * kPi, 0.0, 5.0};
  auto A = ssi::makeSphereAdapter(sp, ds);
  auto B = ssi::makeConeAdapter(co, dk);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(fs), 1.0);
  Handle(Geom_Surface) sb = new Geom_ConicalSurface(toOcctAx3(fk), 0.3, 0.05);
  ssi::SeedOptions sopt; sopt.initialGridU = 8; sopt.initialGridV = 8; sopt.minPatchFrac = 1.0 / 48.0;
  ssi::MarchOptions mopt; mopt.maxDeflection = A.modelScale * 2.5e-4;  // dense fit nodes
  reportPair("sphere cone off-axis", A, B, sa, sb, /*expectTransversal=*/1,
             1e-4, 1e-4, 1e-2, 3e-2, sopt, mopt);
}

// ── M1c BREADTH: the M1b-declined tail promoted to verified via the domain-clipped oracle ──
// M1b honestly declined general cone∩cone, off-axis cyl∩cone, and off-axis sphere∩cyl because
// the INFINITE OCCT oracle surfaces returned a multi-loop locus the FINITE native patch could
// not match. M1c wraps each oracle surface in a Geom_RectangularTrimmedSurface trimmed to the
// native patch's ParamBox (clipOracle) so GeomAPI_IntSS produces the SAME finite locus the native
// trace covers. The twice-piercing sphere∩cyl additionally uses the M1c SEEDING-RECALL BUMP
// (completenessCritic + criticTargetedReseed) to seed the SECOND loop. Poses are shared with the
// host cases (test_native_ssi_marching.cpp) so the two gates agree.

// GENERAL cone∩cone — two finite cones with offset apexes + tilted axes → ONE closed loop inside
// both finite patches. Verified against a domain-clipped oracle (both cones trimmed to v∈[0,2.5]).
void pairConeConeGeneral() {
  const Ax3 f1 = frameZ({0, 0, -1.0});
  const Ax3 f2 = axFromZ({0.35, 0, 0.9}, Vec3{0.05, 0, -1});
  nm::Cone c1{f1, 0.05, 0.4};
  nm::Cone c2{f2, 0.05, 0.4};
  ssi::ParamBox d1{0.0, 2.0 * kPi, 0.0, 2.5};
  ssi::ParamBox d2{0.0, 2.0 * kPi, 0.0, 2.5};
  auto A = ssi::makeConeAdapter(c1, d1);
  auto B = ssi::makeConeAdapter(c2, d2);
  Handle(Geom_Surface) sa = clipOracle(new Geom_ConicalSurface(toOcctAx3(f1), 0.4, 0.05), d1);
  Handle(Geom_Surface) sb = clipOracle(new Geom_ConicalSurface(toOcctAx3(f2), 0.4, 0.05), d2);
  ssi::SeedOptions sopt; sopt.initialGridU = 8; sopt.initialGridV = 8; sopt.minPatchFrac = 1.0 / 48.0;
  ssi::MarchOptions mopt; mopt.maxDeflection = A.modelScale * 2.5e-4;
  reportPair("cone cone general", A, B, sa, sb, /*expectTransversal=*/1,
             1e-4, 1e-4, 1e-2, 3e-2, sopt, mopt);
}

// OFF-AXIS cyl∩cone — the intersection RUNS OFF the finite patch boundaries → ONE open
// (BoundaryExit) arc. The domain-clipped oracle (both operands trimmed to their native boxes)
// returns the matching finite arc; reportPair asserts the SAME single non-closed component.
void pairCylConeOffAxis() {
  const Ax3 fcy = frameZ();
  const Ax3 fco = axFromZ({0.2, 0, -0.3}, Vec3{0.12, 0, 1});
  nm::Cylinder cy{fcy, 0.6};
  nm::Cone co{fco, 0.02, 0.45};
  ssi::ParamBox dcy{0.0, 2.0 * kPi, -1.2, 1.2};
  ssi::ParamBox dco{0.0, 2.0 * kPi, 0.0, 2.5};
  auto A = ssi::makeCylinderAdapter(cy, dcy);
  auto B = ssi::makeConeAdapter(co, dco);
  Handle(Geom_Surface) sa = clipOracle(new Geom_CylindricalSurface(toOcctAx3(fcy), 0.6), dcy);
  Handle(Geom_Surface) sb = clipOracle(new Geom_ConicalSurface(toOcctAx3(fco), 0.45, 0.02), dco);
  ssi::SeedOptions sopt; sopt.initialGridU = 8; sopt.initialGridV = 8; sopt.minPatchFrac = 1.0 / 48.0;
  ssi::MarchOptions mopt; mopt.maxDeflection = A.modelScale * 2.5e-4;
  reportPair("cyl cone off-axis", A, B, sa, sb, /*expectTransversal=*/1,
             1e-4, 1e-4, 1e-2, 3e-2, sopt, mopt);
}

// OFF-AXIS sphere∩cyl (TWICE-PIERCING) — a thin cylinder offset from the sphere centre pierces the
// FINITE sphere on BOTH sides → TWO disjoint closed loops. The coarse fixed grid merges the two
// loops into ONE topological cluster (one representative seed) → the fixed-resolution trace finds
// only ONE loop (the M1b recall miss). The M1c SEEDING-RECALL BUMP (completenessCritic +
// criticTargetedReseed) re-seeds the uncovered param cells and recovers the SECOND loop. Verified
// against a domain-clipped oracle (the cylinder trimmed to its finite height) that returns exactly
// the two finite loops. reportPair asserts branches=2, closed=2, both on the OCCT loci + surfaces.
void pairSphereCylTwicePiercing() {
  const Ax3 fs = frameZ();
  const Ax3 fc = axFromZ({0.55, 0, 0}, Vec3{0.05, 0, 1});
  nm::Sphere sp{fs, 1.0};
  nm::Cylinder cy{fc, 0.3};
  ssi::ParamBox ds{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox dcy{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeSphereAdapter(sp, ds);
  auto B = ssi::makeCylinderAdapter(cy, dcy);
  Handle(Geom_Surface) sa = clipOracle(new Geom_SphericalSurface(toOcctAx3(fs), 1.0), ds);
  Handle(Geom_Surface) sb = clipOracle(new Geom_CylindricalSurface(toOcctAx3(fc), 0.3), dcy);
  ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6; sopt.minPatchFrac = 1.0 / 32.0;
  sopt.completenessCritic = true;         // M1c seeding-recall bump
  sopt.criticTargetedReseed = true;       // targeted re-seed of the uncovered second-loop cells
  sopt.criticRefineFactor = 0.6;
  ssi::MarchOptions mopt; mopt.maxDeflection = A.modelScale * 2.5e-4;
  reportPair("sphere cyl twice", A, B, sa, sb, /*expectTransversal=*/2,
             1e-4, 1e-4, 1e-2, 3e-2, sopt, mopt);
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

// ── S4-c: NEAR-TANGENT TRANSVERSAL graze MARCHED THROUGH, vs OCCT ─────────────────
// An offset cylinder (axis +x by 0.585, R=0.4) grazes a unit sphere: the intersection is
// a SINGLE closed loop whose transversality sine dips to ≈0.10 at the grazing pinches but
// the curve genuinely CONTINUES through them. With tangentSinTol raised ABOVE that dip the
// S3 marcher truncates (NearTangent); S4-c recognizes the graze as NearTangentTransversal
// and MARCHES THROUGH it, producing the FULL loop.
//
// PARITY (honest): at the graze OCCT's GeomAPI_IntSS and the native marcher legitimately
// disagree on CONNECTIVITY — OCCT tolerance-splits the loop at the tangency, native crosses
// it as one loop. So this gate asserts the strong, uncontested facts: (a) every densely-
// sampled point of the crossed native curve lies ON the OCCT intersection locus AND on BOTH
// surfaces within tol — i.e. the crossed curve IS the true intersection, not a fabricated
// path; and (b) it was a genuine S4-c crossing (nearTangentGaps → 0, nearTangentCrossed ≥ 1,
// one Closed loop, the crossed arc on both surfaces ≤ onSurfTol) — not an honest truncation.
void pairNearTangentCrossedS4c() {
  nm::Sphere sph{frameZ(), 1.0};
  nm::Cylinder cyl{frameZ({0.585, 0, 0}), 0.4};  // r+dx = 0.985 → near-tangent graze
  ssi::ParamBox sdom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cdom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sph, sdom);
  auto B = ssi::makeCylinderAdapter(cyl, cdom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(frameZ({0.585, 0, 0})), 0.4);

  ssi::SeedOptions sopt; sopt.initialGridU = 4; sopt.initialGridV = 4; sopt.minPatchFrac = 1.0 / 32.0;
  ssi::MarchOptions mopt; mopt.tangentSinTol = 0.25;  // ABOVE the dip: S3 would truncate, S4-c crosses

  const ssi::TraceSet ts = ssi::trace_intersection(A, B, sopt, mopt);

  // OCCT locus (all branches — the graze may split it) for the on-curve oracle.
  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;
  std::vector<OcctBranch> occtBr;
  for (int i = 1; i <= occtN; ++i) occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, 1e-2));

  const double onCurveTol = 5e-4, onSurfTol = 1e-4;
  double maxOnCurve = 0.0, maxOnSurf = 0.0, crossResid = 0.0;
  for (const auto& w : ts.lines) {
    crossResid = std::max(crossResid, w.crossMaxResidual);
    for (const Point3& p : sampleWLine(w)) {
      double best = 1e30;
      for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, p));
      maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf, std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
    }
  }

  const bool ok = ts.nearTangentGaps == 0 && ts.nearTangentCrossed >= 1 &&
                  ts.closedCurves == 1 && occtN > 0 && crossResid <= onSurfTol &&
                  maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s NTgaps=%d crossed=%d closed=%d onCurve=%.2e onSurf=%.2e "
              "crossResid=%.2e occtBr=%d\n",
              ok ? "PASS" : "FAIL", "nt-cross s4c", ts.nearTangentGaps, ts.nearTangentCrossed,
              ts.closedCurves, maxOnCurve, maxOnSurf, crossResid, occtN);
  std::fflush(stdout);
}

// ── S4-c DEEP breadth (M1d): a TIGHTER graze crossed by adaptive re-anchoring, vs OCCT ──
// The same offset cyl∩sphere family, pushed DEEPER: dx = 0.590 (r+dx = 0.990) so the
// transversality sine dips to ≈ 0.141 — BELOW the ≈ 0.17 floor where the shipped fixed-t★
// crossing corrector still converges. With reanchor OFF the DEFAULT S4-c honestly DEFERS
// here (asserted below). With `adaptiveCrossReanchor` the crossing re-anchors its advance
// plane to the local curve tangent and traverses the graze → the FULL closed loop.
//
// PARITY (honest, same contract as the shipped S4-c graze): OCCT tolerance-splits the loop
// at the near-tangency while native crosses it as ONE loop, so the gate asserts the strong
// uncontested facts: every densely-sampled native node lies ON the OCCT locus AND on BOTH
// surfaces within tol (the crossed curve IS the true intersection, not fabricated), it was a
// genuine crossing (nearTangentGaps → 0, nearTangentCrossed ≥ 1, one Closed loop), AND the
// DEFAULT (reanchor off) still declines this deeper graze (the breadth extension is real).
void pairDeepNearTangentReanchorS4c() {
  nm::Sphere sph{frameZ(), 1.0};
  nm::Cylinder cyl{frameZ({0.590, 0, 0}), 0.4};  // r+dx = 0.990 → deeper graze, minSine ≈ 0.141
  ssi::ParamBox sdom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cdom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sph, sdom);
  auto B = ssi::makeCylinderAdapter(cyl, cdom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(frameZ({0.590, 0, 0})), 0.4);

  ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6; sopt.minPatchFrac = 1.0 / 64.0;

  // DEFAULT S4-c (reanchor OFF): the deeper graze is below the fixed-plane floor → HONEST DEFER.
  ssi::MarchOptions moOff; moOff.tangentSinTol = 0.25;
  const ssi::TraceSet tsOff = ssi::trace_intersection(A, B, sopt, moOff);
  const bool declineOff = tsOff.nearTangentGaps >= 1 && tsOff.nearTangentCrossed == 0 &&
                          tsOff.closedCurves == 0;

  // M1d adaptive re-anchoring ON: crosses the tighter graze → full closed loop.
  ssi::MarchOptions moOn; moOn.tangentSinTol = 0.25;
  moOn.adaptiveCrossReanchor = true; moOn.reanchorBlend = 0.5;
  const ssi::TraceSet ts = ssi::trace_intersection(A, B, sopt, moOn);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;
  std::vector<OcctBranch> occtBr;
  for (int i = 1; i <= occtN; ++i) occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, 1e-2));

  const double onCurveTol = 5e-4, onSurfTol = 1e-4;
  double maxOnCurve = 0.0, maxOnSurf = 0.0, crossResid = 0.0;
  for (const auto& w : ts.lines) {
    crossResid = std::max(crossResid, w.crossMaxResidual);
    for (const Point3& p : sampleWLine(w)) {
      double best = 1e30;
      for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, p));
      maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf, std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
    }
  }

  const bool ok = declineOff && ts.nearTangentGaps == 0 && ts.nearTangentCrossed >= 1 &&
                  ts.closedCurves == 1 && occtN > 0 && crossResid <= onSurfTol &&
                  maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s declineOff=%d NTgaps=%d crossed=%d closed=%d onCurve=%.2e "
              "onSurf=%.2e crossResid=%.2e occtBr=%d\n",
              ok ? "PASS" : "FAIL", "deep-nt reanchor", (int)declineOff, ts.nearTangentGaps,
              ts.nearTangentCrossed, ts.closedCurves, maxOnCurve, maxOnSurf, crossResid, occtN);
  std::fflush(stdout);
}

// ── S4-c HONEST DECLINE below the extended floor (M1d) ────────────────────────────
// Pushed FURTHER: dx = 0.595 (r+dx = 0.995), transversality sine ≈ 0.100 — below even the
// adaptive-re-anchoring floor. EVEN WITH reanchor ON the native marcher HONESTLY DECLINES:
// no crossing, no fabricated loop across the knife-edge (nearTangentCrossed == 0, no Closed
// loop, nearTangentGaps ≥ 1 → deferred to OCCT). OCCT (GeomAPI_IntSS) does resolve a locus
// here; the gate asserts native declines while OCCT reports — the honest boundary, measured.
void pairDeepNearTangentHonestDeclineS4c() {
  nm::Sphere sph{frameZ(), 1.0};
  nm::Cylinder cyl{frameZ({0.595, 0, 0}), 0.4};  // r+dx = 0.995 → minSine ≈ 0.100, below the extended floor
  ssi::ParamBox sdom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cdom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sph, sdom);
  auto B = ssi::makeCylinderAdapter(cyl, cdom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(frameZ({0.595, 0, 0})), 0.4);

  ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6; sopt.minPatchFrac = 1.0 / 64.0;
  ssi::MarchOptions moOn; moOn.tangentSinTol = 0.25;
  moOn.adaptiveCrossReanchor = true; moOn.reanchorBlend = 0.5;
  const ssi::TraceSet ts = ssi::trace_intersection(A, B, sopt, moOn);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;

  bool anyFabricatedLoop = false;
  for (const auto& w : ts.lines)
    if (w.nearTangentCrossed != 0 || w.isClosed()) anyFabricatedLoop = true;

  const bool ok = ts.nearTangentCrossed == 0 && !anyFabricatedLoop &&
                  ts.nearTangentGaps >= 1 && occtN > 0;  // native declines; OCCT reports
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s crossed=%d closed=%d NTgaps=%d occtBr=%d (native declines, OCCT reports)\n",
              ok ? "PASS" : "FAIL", "deep-nt decline", ts.nearTangentCrossed, ts.closedCurves,
              ts.nearTangentGaps, occtN);
  std::fflush(stdout);
}

// ── S4-c WIDE-BAND (M1e): incremental orientation vs OCCT ─────────────────────────────
// The M1d re-anchor oriented the local tangent AND gated its adoption against the FROZEN t★.
// Both degenerate once the curve's tangent turns 90° from t★, flipping forward to backward and
// trapping the march in a 2-cycle that burns the node budget without traversing. That produced
// an honest decline, but a SPURIOUS one — the poses are genuinely crossable.
// `reanchorIncrementalOrientation` re-references both tests to the previous accepted step
// direction. This gate is the INDEPENDENT check the host suite structurally cannot make: the
// newly-crossed loops are self-consistent by construction (they satisfy the same corrector that
// produced them), so they must be confronted with a foreign kernel before they are believed.
//
// SCOPE — why dx = 0.595 and not 0.597. The CROSSING succeeds at 0.597 too (Gate A covers it:
// one closed loop, every corrected node on both surfaces to 2.39e-11). It is excluded HERE
// because of the FITTED CURVE, not the march. `sampleWLine` samples the convenience B-spline as
// well as the nodes, and that fit is targeted at `scale·2e-4` ≈ 2.2e-3 — four times LOOSER than
// this gate's 5e-4 on-curve tolerance — so the one-shot densify-and-refit never fires. Measured
// off-surface deviation, nodes vs fit: 1.31e-11 / 2.97e-05 at dx=0.593, 1.16e-11 / 3.36e-05 at
// 0.595, 2.39e-11 / 6.99e-05 at 0.597 — the fit is six orders of magnitude worse than the march
// it interpolates, and at 0.597 it drags the OCCT distance to 6.00e-04, over tolerance.
// The honest reading: the marched curve is right, the cubic fit under-resolves the extreme
// curvature at the graze. Widening onCurveTol here would hide a real defect, so the gate keeps
// its tolerance and this pose stays out until the fit is fixed. That is the next slice.
void pairWideBandIncrementalOrientationS4c() {
  for (const double dx : {0.595}) {
    nm::Sphere sph{frameZ(), 1.0};
    nm::Cylinder cyl{frameZ({dx, 0, 0}), 0.4};
    ssi::ParamBox sdom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
    ssi::ParamBox cdom{0.0, 2.0 * kPi, -1.5, 1.5};
    auto A = ssi::makeSphereAdapter(sph, sdom);
    auto B = ssi::makeCylinderAdapter(cyl, cdom);
    Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
    Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(frameZ({dx, 0, 0})), 0.4);

    ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6; sopt.minPatchFrac = 1.0 / 64.0;

    // M1d re-anchor WITHOUT incremental orientation: trapped → declines (the defect).
    ssi::MarchOptions moOff; moOff.tangentSinTol = 0.25;
    moOff.adaptiveCrossReanchor = true; moOff.reanchorBlend = 0.5;
    const ssi::TraceSet tsOff = ssi::trace_intersection(A, B, sopt, moOff);
    const bool declineOff = tsOff.nearTangentCrossed == 0 && tsOff.closedCurves == 0;

    // M1e ON: crosses to one closed loop.
    ssi::MarchOptions moOn = moOff;
    moOn.reanchorIncrementalOrientation = true;
    const ssi::TraceSet ts = ssi::trace_intersection(A, B, sopt, moOn);

    GeomAPI_IntSS iss(sa, sb, 1e-7);
    const int occtN = iss.IsDone() ? iss.NbLines() : 0;
    std::vector<OcctBranch> occtBr;
    for (int i = 1; i <= occtN; ++i) occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, 1e-2));

    const double onCurveTol = 5e-4, onSurfTol = 1e-4;
    double maxOnCurve = 0.0, maxOnSurf = 0.0, crossResid = 0.0;
    for (const auto& w : ts.lines) {
      crossResid = std::max(crossResid, w.crossMaxResidual);
      for (const Point3& p : sampleWLine(w)) {
        double best = 1e30;
        for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, p));
        maxOnCurve = std::max(maxOnCurve, best);
        maxOnSurf = std::max(maxOnSurf, std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
      }
    }

    const bool ok = declineOff && ts.nearTangentGaps == 0 && ts.nearTangentCrossed >= 1 &&
                    ts.closedCurves == 1 && occtN > 0 && crossResid <= onSurfTol &&
                    maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol;
    if (ok) ++g_pass; else ++g_fail;
    char label[32];
    std::snprintf(label, sizeof(label), "wide-band %.3f", dx);
    std::printf("[NMARCH] %-4s %-18s declineOff=%d NTgaps=%d crossed=%d closed=%d onCurve=%.2e "
                "onSurf=%.2e crossResid=%.2e occtBr=%d\n",
                ok ? "PASS" : "FAIL", label, (int)declineOff, ts.nearTangentGaps,
                ts.nearTangentCrossed, ts.closedCurves, maxOnCurve, maxOnSurf, crossResid, occtN);
    std::fflush(stdout);
  }
}

// ── S4-c HONEST DECLINE below the M1e floor (minCrossSine) ────────────────────────────
// dx = 0.598: the band minimum sine falls under `minCrossSine` = 0.075, the DESIGNED honesty
// tolerance. Even with incremental orientation the crossing refuses at the band-minimum gate.
// This is where the floor SHOULD sit — a declared tolerance, not an artefact of a stale vector.
void pairWideBandHonestDeclineS4c() {
  nm::Sphere sph{frameZ(), 1.0};
  nm::Cylinder cyl{frameZ({0.598, 0, 0}), 0.4};  // bandMin ≈ 0.063 < minCrossSine 0.075
  ssi::ParamBox sdom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cdom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sph, sdom);
  auto B = ssi::makeCylinderAdapter(cyl, cdom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(frameZ({0.598, 0, 0})), 0.4);

  ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6; sopt.minPatchFrac = 1.0 / 64.0;
  ssi::MarchOptions moOn; moOn.tangentSinTol = 0.25;
  moOn.adaptiveCrossReanchor = true; moOn.reanchorBlend = 0.5;
  moOn.reanchorIncrementalOrientation = true;
  const ssi::TraceSet ts = ssi::trace_intersection(A, B, sopt, moOn);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;

  bool anyFabricatedLoop = false;
  for (const auto& w : ts.lines)
    if (w.nearTangentCrossed != 0 || w.isClosed()) anyFabricatedLoop = true;

  const bool ok = ts.nearTangentCrossed == 0 && !anyFabricatedLoop &&
                  ts.nearTangentGaps >= 1 && occtN > 0;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s crossed=%d closed=%d NTgaps=%d occtBr=%d (below minCrossSine → honest)\n",
              ok ? "PASS" : "FAIL", "wide-band decline", ts.nearTangentCrossed, ts.closedCurves,
              ts.nearTangentGaps, occtN);
  std::fflush(stdout);
}

// ── S4-c: a GENUINE tangency the marcher reaches must STILL STOP (not be crossed) ─────
// Two equal cylinders crossing at 90° (axes Z and X, both R=1) meet TANGENTIALLY at the
// saddle points — a BRANCH crossing (S4-d), where the intersection self-crosses and the
// transversality sine → 0. S4-c's crossable gate must REFUSE it: no fabricated crossing
// (nearTangentCrossed == 0), the region honestly reported as a near-tangent gap
// (nearTangentGaps ≥ 1) — deferred to OCCT, never faked. (OCCT resolves the figure-8; the
// native honest outcome is to decline, which S5 turns into an OCCT fallback.)
void pairEqualCylindersDefer() {
  nm::Cylinder cz{frameZ(), 1.0};
  nm::Cylinder cx{Ax3{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}}, 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -3.0, 3.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);

  const ssi::TraceSet ts = ssi::trace_intersection(A, B);
  const bool ok = ts.nearTangentCrossed == 0 && ts.nearTangentGaps >= 1 &&
                  ts.closedCurves == 0;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s NTgaps=%d crossed=%d closed=%d (branch saddle → defer)\n",
              ok ? "PASS" : "FAIL", "eq-cyl defer", ts.nearTangentGaps,
              ts.nearTangentCrossed, ts.closedCurves);
  std::fflush(stdout);
}

// ── S4-d: the STEINMETZ branch points are LOCALIZED + the arms ROUTED, vs OCCT ────────
// The SAME two equal orthogonal cylinders as pairEqualCylindersDefer (which stays a control
// showing the DEFAULT still defers). Here we enable branch points: the native tracer must
// LOCALIZE both branch points (0,±1,0) and ROUTE the arms so the FULL multi-arm intersection
// (the two ellipses in planes x=±z) is traced. Verified against the OCCT oracle:
//   * branchPoints == 2, each on BOTH OCCT surfaces AND on the OCCT locus within tol;
//   * every native arc node lies on the OCCT locus (nearest OCCT branch) AND on both
//     surfaces within tol — no fabricated points;
//   * the native branch points match the analytic/OCCT saddles (0,±1,0);
//   * nearTangentGaps == 0 (all arcs resolved to branch junctions), tracedBranches ≥ 4.
// (OCCT tolerance-splits the figure-8 into its own arc set; the gate asserts the uncontested
// facts — the native curve lies on OCCT's locus + surfaces, and both branch points match.)
void pairEqualCylindersBranchS4d() {
  nm::Cylinder cz{frameZ(), 1.0};
  nm::Cylinder cx{Ax3{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}}, 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);

  Handle(Geom_Surface) sa = new Geom_CylindricalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb =
      new Geom_CylindricalSurface(toOcctAx3(Ax3{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}}), 1.0);

  ssi::MarchOptions mo;
  mo.enableBranchPoints = true;
  const ssi::TraceSet ts = ssi::trace_intersection(A, B, {}, mo);

  const double onCurveTol = 5e-4, onSurfTol = 5e-4, bpTol = 5e-3;

  // OCCT locus branches (for the on-locus check).
  GeomAPI_IntSS iss(sa, sb, 1e-7);
  std::vector<OcctBranch> occtBr;
  if (iss.IsDone())
    for (int i = 1; i <= iss.NbLines(); ++i)
      occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, /*tangentSine=*/1e-2));

  // Every native arc node on the OCCT locus AND on both surfaces.
  double maxOnCurve = 0.0, maxOnSurf = 0.0;
  for (const auto& w : ts.lines)
    for (const auto& n : w.points) {
      double best = 1e30;
      for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, n.point));
      if (!occtBr.empty()) maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf,
                           std::max(distToOcctSurface(sa, n.point), distToOcctSurface(sb, n.point)));
    }

  // Both branch points localized at (0,±1,0), on both OCCT surfaces and on the OCCT locus.
  bool sawPlus = false, sawMinus = false, bpOnLocus = true, bpOnSurf = true;
  for (const auto& bn : ts.branchNodes) {
    if (nm::distance(bn.point, Point3{0, 1, 0}) < bpTol) sawPlus = true;
    if (nm::distance(bn.point, Point3{0, -1, 0}) < bpTol) sawMinus = true;
    if (distToOcctSurface(sa, bn.point) > onSurfTol || distToOcctSurface(sb, bn.point) > onSurfTol)
      bpOnSurf = false;
    double best = 1e30;
    for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, bn.point));
    if (!occtBr.empty() && best > onCurveTol) bpOnLocus = false;
  }

  const bool ok = ts.branchPoints == 2 && ts.nearTangentGaps == 0 && ts.tracedBranches >= 4 &&
                  sawPlus && sawMinus && bpOnSurf && bpOnLocus &&
                  maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s branchPts=%d NTgaps=%d traced=%d arms=%d onCurve=%.2e "
              "onSurf=%.2e occtBr=%d (steinmetz localized + routed)\n",
              ok ? "PASS" : "FAIL", "eq-cyl s4d", ts.branchPoints, ts.nearTangentGaps,
              ts.tracedBranches, ts.routedArms, maxOnCurve, maxOnSurf, (int)occtBr.size());
  std::fflush(stdout);
}

// ── S4-d GENERAL/FREEFORM open-arm branch point, vs OCCT ──────────────────────────────
// A bicubic B-spline SADDLE z = 0.15·(x²−y²) (a genuine FREEFORM operand, NOT a quadric)
// TANGENT to a plane placed THROUGH its saddle point (the surface value at the patch centre,
// z ≈ 0.2449 — NOT z=0, where the two hyperbola branches are DISJOINT). The intersection locus
// self-crosses at ONE branch point with FOUR arms radiating OPEN to the patch boundary. The
// native tracer must LOCALIZE the branch point, ENUMERATE + ROUTE the four arms, and RECLASSIFY
// each OPEN arm (branch-to-boundary) as a resolved BranchArc → branchPoints==1, nearTangentGaps==0.
// Verified against the OCCT GeomAPI_IntSS oracle on the SAME Geom_BSplineSurface ∩ Geom_Plane:
//   * every native arc node lies on the OCCT locus (nearest OCCT branch) AND on both surfaces;
//   * the localized branch point lies on both OCCT surfaces AND on the OCCT locus, at the saddle;
//   * branchPoints==1, nearTangentGaps==0, tracedBranches≥4 (the four open arms).
void pairFreeformSaddleBranchS4d() {
  const int deg = 3, n = 4;
  const auto k = clampedKnots(deg, n);
  const double xs[4] = {-3, -1, 1, 3};
  const double ys[4] = {-2, -0.7, 0.7, 2};
  std::vector<Point3> saddle;
  saddle.reserve(16);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      saddle.push_back({xs[i], ys[j], 0.15 * (xs[i] * xs[i] - ys[j] * ys[j])});
  auto A = ssi::makeBSplineAdapter(deg, deg, saddle, n, n, k, k);

  // The plane through the B-spline saddle point (the surface value at the patch centre).
  const Point3 saddlePt = A.point(0.5, 0.5);
  const double cutZ = saddlePt.z;
  nm::Plane pl{Ax3{{0, 0, cutZ}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  ssi::ParamBox pd{-4.0, 4.0, -3.0, 3.0};
  auto B = ssi::makePlaneAdapter(pl, pd);

  Handle(Geom_Surface) sa = toOcctBSpline(saddle, n, n, deg, deg);
  Handle(Geom_Surface) sb = new Geom_Plane(toOcctAx3(Ax3{{0, 0, cutZ}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));

  ssi::SeedOptions sopt; sopt.initialGridU = 3; sopt.initialGridV = 3;
  ssi::MarchOptions mo;
  mo.enableBranchPoints = true;
  const ssi::TraceSet ts = ssi::trace_intersection(A, B, sopt, mo);

  const double onCurveTol = 5e-4, onSurfTol = 5e-4, bpTol = 1e-2;

  // OCCT locus branches (for the on-locus check).
  GeomAPI_IntSS iss(sa, sb, 1e-7);
  std::vector<OcctBranch> occtBr;
  if (iss.IsDone())
    for (int i = 1; i <= iss.NbLines(); ++i)
      occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, /*tangentSine=*/1e-2));

  // Every native arc node on the OCCT locus AND on both surfaces.
  double maxOnCurve = 0.0, maxOnSurf = 0.0;
  for (const auto& w : ts.lines)
    for (const auto& nd : w.points) {
      double best = 1e30;
      for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, nd.point));
      if (!occtBr.empty()) maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf,
                           std::max(distToOcctSurface(sa, nd.point), distToOcctSurface(sb, nd.point)));
    }

  // The single branch point localized at the saddle point, on both OCCT surfaces + the OCCT locus.
  bool bpAtSaddle = false, bpOnSurf = true, bpOnLocus = true;
  for (const auto& bn : ts.branchNodes) {
    if (nm::distance(bn.point, Point3{0, 0, cutZ}) < bpTol) bpAtSaddle = true;
    if (distToOcctSurface(sa, bn.point) > onSurfTol || distToOcctSurface(sb, bn.point) > onSurfTol)
      bpOnSurf = false;
    double best = 1e30;
    for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, bn.point));
    if (!occtBr.empty() && best > onCurveTol) bpOnLocus = false;
  }

  const bool ok = ts.branchPoints == 1 && ts.nearTangentGaps == 0 && ts.tracedBranches >= 4 &&
                  bpAtSaddle && bpOnSurf && bpOnLocus &&
                  maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s branchPts=%d NTgaps=%d traced=%d arms=%d onCurve=%.2e "
              "onSurf=%.2e occtBr=%d (freeform saddle open-arm)\n",
              ok ? "PASS" : "FAIL", "saddle s4d-g", ts.branchPoints, ts.nearTangentGaps,
              ts.tracedBranches, ts.routedArms, maxOnCurve, maxOnSurf, (int)occtBr.size());
  std::fflush(stdout);
}

// ── S4-e helpers: force MARCHING through a chart singularity + verify vs OCCT ─────────
// A one-seed SeedSet on hand params (trace_from_seeds forces marching, bypassing S1/S2 so
// the analytic pair does not short-circuit to a closed form and the march actually walks
// through the pole/apex).
ssi::SeedSet handSeed(double u1, double v1, double u2, double v2, const Point3& p) {
  ssi::Seed s;
  s.u1 = u1; s.v1 = v1; s.u2 = u2; s.v2 = v2;
  s.point = p; s.onSurfResidual = 0.0; s.branchId = 0;
  ssi::SeedSet ss;
  ss.seeds.push_back(s);
  ss.candidateRegions = 1;
  ss.refinedAccepted = 1;
  return ss;
}

// OCCT y=0 plane (normal +Y): frame {origin, X=+x, Y=+z, Z(normal)=+y}. Its GeomAPI_IntSS
// with the sphere is the great circle through both poles; with the double cone, the two
// apex-crossing lines. Matches the native planeY0 fixture used in the S4-e host suite.
Handle(Geom_Surface) occtPlaneY0() {
  return new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0)));
}
ssi::SurfaceAdapter nativePlaneY0(const ssi::ParamBox& dom) {
  nm::Plane pl;
  pl.pos = Ax3{Point3{0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 0, 1}, Dir3{0, 1, 0}};
  return ssi::makePlaneAdapter(pl, dom);
}

// ── S4-e (a): SPHERE POLE — the great circle through BOTH poles fully traced vs OCCT ──
// Unit sphere ∩ plane y=0 → the great circle in the x-z plane, which passes through the
// sphere's two PARAMETRIC POLES (v=±π/2). With the chart switch OFF the S3 marcher truncates
// to one meridian at the first pole (spurious BoundaryExit). With it ON S4-e steps across
// both poles with the point-based cut → the FULL closed loop. Verified against OCCT
// GeomAPI_IntSS: every native node on the OCCT locus AND on both OCCT surfaces within tol,
// arc length ≈ the OCCT circle, closed. (The chart singularity is a NATIVE parametrization
// artifact — OCCT's implicit sphere has no pole there — so the strong facts asserted are
// on-locus + on-surface + length + closed, exactly the S3 contract.)
void pairSpherePoleS4e() {
  nm::Sphere sph{frameZ(), 1.0};
  auto A = ssi::makeSphereAdapter(sph, ssi::ParamBox{0.0, 2.0 * kPi, -kPi / 2, kPi / 2});
  auto B = nativePlaneY0(ssi::ParamBox{-2.0, 2.0, -2.0, 2.0});
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = occtPlaneY0();
  auto seeds = handSeed(0.0, 0.0, 1.0, 0.0, Point3{1, 0, 0});  // (1,0,0), away from either pole

  // BEFORE — chart switch OFF: truncates at the first pole (< the full 2π loop).
  ssi::MarchOptions off;
  const ssi::TraceSet before = ssi::trace_from_seeds(A, B, seeds, off);
  double lenBefore = 0.0;
  for (const auto& w : before.lines) lenBefore += wlineLength(w);

  // AFTER — chart switch ON: both poles crossed → the full great circle.
  ssi::MarchOptions on; on.enableChartSingularities = true;
  const ssi::TraceSet ts = ssi::trace_from_seeds(A, B, seeds, on);

  // OCCT locus (the great circle; may be arc-split) for the on-curve oracle + its length.
  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;
  std::vector<OcctBranch> occtBr;
  double occtLen = 0.0;
  for (int i = 1; i <= occtN; ++i) {
    occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, 1e-2));
    occtLen += occtBr.back().length;
  }

  const double onCurveTol = 5e-4, onSurfTol = 1e-4, lengthTol = 3e-2;
  double maxOnCurve = 0.0, maxOnSurf = 0.0, nativeLen = 0.0;
  for (const auto& w : ts.lines) {
    nativeLen += wlineLength(w);
    for (const Point3& p : sampleWLine(w)) {
      double best = 1e30;
      for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, p));
      maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf, std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
    }
  }
  const double lenDelta = occtLen > 1e-12 ? std::fabs(nativeLen - occtLen) / occtLen : nativeLen;

  const bool ok = ts.singularitiesCrossed >= 2 && ts.nearTangentGaps == 0 &&
                  ts.closedCurves == 1 && occtN > 0 && lenBefore < 0.75 * occtLen &&
                  maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol && lenDelta <= lengthTol;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s singX=%d NTgaps=%d closed=%d onCurve=%.2e onSurf=%.2e "
              "lenDelta=%.2e (before=%.4f nat=%.4f occt=%.4f) occtBr=%d\n",
              ok ? "PASS" : "FAIL", "sphere-pole s4e", ts.singularitiesCrossed, ts.nearTangentGaps,
              ts.closedCurves, maxOnCurve, maxOnSurf, lenDelta, lenBefore, nativeLen, occtLen, occtN);
  std::fflush(stdout);
}

// ── S4-e (b): CONE APEX — the apex-crossing line spans BOTH nappes vs OCCT ────────────
// Double cone (R₀=0, α=45°, apex at origin) ∩ plane y=0 → the two lines z=±x through the
// apex. OFF: the S3 marcher step-crawls at the apex (signed radius → 0) and never reaches the
// v<0 nappe. ON: S4-e treats the apex as a pass-through 3D point → both nappes traced in a
// bounded node count. Verified vs OCCT GeomAPI_IntSS: every native node on the OCCT locus AND
// on both surfaces within tol.
void pairConeApexS4e() {
  nm::Cone cone; cone.pos = frameZ(); cone.radius = 0.0; cone.semiAngle = kPi / 4;
  auto A = ssi::makeConeAdapter(cone, ssi::ParamBox{0.0, 2.0 * kPi, -2.0, 2.0});
  auto B = nativePlaneY0(ssi::ParamBox{-3.0, 3.0, -3.0, 3.0});
  Handle(Geom_Surface) sa = new Geom_ConicalSurface(toOcctAx3(frameZ()), kPi / 4, 0.0);
  Handle(Geom_Surface) sb = occtPlaneY0();
  const double vSeed = 1.8, r = vSeed * std::sin(kPi / 4), z = vSeed * std::cos(kPi / 4);
  auto seeds = handSeed(0.0, vSeed, r, z, Point3{r, 0.0, z});

  // BEFORE — chart switch OFF: stalls at the apex, never the far nappe.
  ssi::MarchOptions off;
  const ssi::TraceSet before = ssi::trace_from_seeds(A, B, seeds, off);
  double v1loBefore = 1e9;
  for (const auto& w : before.lines) for (const auto& n : w.points) v1loBefore = std::min(v1loBefore, n.v1);

  // AFTER — chart switch ON: the apex crossed, both nappes.
  ssi::MarchOptions on; on.enableChartSingularities = true;
  const ssi::TraceSet ts = ssi::trace_from_seeds(A, B, seeds, on);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;
  std::vector<OcctBranch> occtBr;
  for (int i = 1; i <= occtN; ++i) occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, 1e-2));

  const double onCurveTol = 5e-4, onSurfTol = 1e-4;
  double maxOnCurve = 0.0, maxOnSurf = 0.0, v1lo = 1e9, v1hi = -1e9;
  int nodeCount = 0;
  for (const auto& w : ts.lines) {
    for (const auto& n : w.points) { v1lo = std::min(v1lo, n.v1); v1hi = std::max(v1hi, n.v1); ++nodeCount; }
    for (const Point3& p : sampleWLine(w)) {
      double best = 1e30;
      for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, p));
      if (!occtBr.empty()) maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf, std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
    }
  }

  const bool ok = ts.singularitiesCrossed >= 1 && ts.nearTangentGaps == 0 && occtN > 0 &&
                  v1loBefore > -0.5 && v1lo < -1.5 && v1hi > 1.5 && nodeCount < 2000 &&
                  maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s singX=%d NTgaps=%d nodes=%d v1=[%.2f,%.2f] (before v1lo=%.2f) "
              "onCurve=%.2e onSurf=%.2e occtBr=%d\n",
              ok ? "PASS" : "FAIL", "cone-apex s4e", ts.singularitiesCrossed, ts.nearTangentGaps,
              nodeCount, v1lo, v1hi, v1loBefore, maxOnCurve, maxOnSurf, occtN);
  std::fflush(stdout);
}

// ── S4-e (c): FREEFORM PARAMETRIC POLE — a NURBS unit sphere crossed vs OCCT ──────────
// A NATIVE rational (NURBS) unit sphere — a surface of revolution whose two v-EDGE control ROWS
// are COLLAPSED to the poles (‖dU‖ → 0, finite point + finite-limit normal) and which carries
// uPeriod == 0 (NO analytic meridian map) — ∩ the plane x = 0 → the great circle through both
// FREEFORM poles. This is the freeform analog of pairSpherePoleS4e: OFF truncates to one meridian
// at the first pole; ON crosses BOTH freeform poles via the point-only far-longitude re-seed
// (freeformChartInvert) and closes the full loop. The OCCT ORACLE is the analytic
// Geom_SphericalSurface (geometrically identical — OCCT has no parametric pole there), so the
// asserted facts are on-locus + on-both-surfaces + closed + arc ≈ the OCCT circle, exactly the
// S3 contract: the native FREEFORM-pole crossing must reproduce the OCCT great circle.
constexpr double kC = 0.70710678118654752440;  // 1/√2 — the NURBS circle mid-weight
ssi::SurfaceAdapter nativeNurbsSphere() {
  const int degU = 2, degV = 2, nU = 9, nV = 5;
  const double px[5] = {0.0, 1.0, 1.0, 1.0, 0.0}, pz[5] = {-1.0, -1.0, 0.0, 1.0, 1.0},
               pw[5] = {1.0, kC, 1.0, kC, 1.0};
  std::vector<Point3> poles(static_cast<std::size_t>(nU) * nV);
  std::vector<double> w(static_cast<std::size_t>(nU) * nV);
  for (int k = 0; k < nU; ++k) {
    const double ang = k * 45.0 * kPi / 180.0;
    const bool corner = (k % 2) == 1;
    const double wc = corner ? kC : 1.0, rs = corner ? (1.0 / kC) : 1.0;
    const double ck = std::cos(ang), sk = std::sin(ang);
    for (int j = 0; j < nV; ++j) {
      const double R = px[j] * rs;
      poles[static_cast<std::size_t>(k) * nV + j] = Point3{R * ck, R * sk, pz[j]};
      w[static_cast<std::size_t>(k) * nV + j] = wc * pw[j];
    }
  }
  std::vector<double> kU{0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4}, kV{0, 0, 0, 1, 1, 2, 2, 2};
  return ssi::makeNurbsAdapter(degU, degV, poles, w, nU, nV, kU, kV);
}
// Native / OCCT plane x = 0 (the yz-plane, normal +X): contains the sphere axis, so ∩ sphere is
// the great circle through both poles. Native frame {origin, X=+y, Y=+z, normal=+x}.
ssi::SurfaceAdapter nativePlaneX0(const ssi::ParamBox& dom) {
  nm::Plane pl;
  pl.pos = Ax3{Point3{0, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}};
  return ssi::makePlaneAdapter(pl, dom);
}
Handle(Geom_Surface) occtPlaneX0() {
  return new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(1, 0, 0), gp_Dir(0, 1, 0)));
}
void pairFreeformPoleS4e() {
  auto A = nativeNurbsSphere();  // v-domain [0,2]; freeform poles at v=0 and v=2
  auto B = nativePlaneX0(ssi::ParamBox{-2.0, 2.0, -2.0, 2.0});
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = occtPlaneX0();
  const Point3 sp = A.point(1.0, 1.0);                     // equator (0,1,0)
  auto seeds = handSeed(1.0, 1.0, sp.y, sp.z, sp);         // plane x=0 params (u=y, v=z)

  // BEFORE — chart switch OFF: truncates at the first freeform pole (< the full 2π loop).
  ssi::MarchOptions off;
  const ssi::TraceSet before = ssi::trace_from_seeds(A, B, seeds, off);
  double lenBefore = 0.0;
  for (const auto& w : before.lines) lenBefore += wlineLength(w);

  // AFTER — chart switch ON: both freeform poles crossed → the full great circle.
  ssi::MarchOptions on; on.enableChartSingularities = true;
  const ssi::TraceSet ts = ssi::trace_from_seeds(A, B, seeds, on);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;
  std::vector<OcctBranch> occtBr;
  double occtLen = 0.0;
  for (int i = 1; i <= occtN; ++i) {
    occtBr.push_back(classifyBranch(iss.Line(i), sa, sb, 1e-2));
    occtLen += occtBr.back().length;
  }

  const double onCurveTol = 5e-4, onSurfTol = 1e-4, lengthTol = 3e-2;
  double maxOnCurve = 0.0, maxOnSurf = 0.0, nativeLen = 0.0;
  for (const auto& w : ts.lines) {
    nativeLen += wlineLength(w);
    for (const Point3& p : sampleWLine(w)) {
      double best = 1e30;
      for (const auto& b : occtBr) best = std::min(best, distToOcctCurve(b.curve, p));
      maxOnCurve = std::max(maxOnCurve, best);
      maxOnSurf = std::max(maxOnSurf, std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
    }
  }
  const double lenDelta = occtLen > 1e-12 ? std::fabs(nativeLen - occtLen) / occtLen : nativeLen;

  const bool ok = ts.singularitiesCrossed >= 2 && ts.nearTangentGaps == 0 &&
                  ts.closedCurves == 1 && occtN > 0 && lenBefore < 0.75 * occtLen &&
                  maxOnCurve <= onCurveTol && maxOnSurf <= onSurfTol && lenDelta <= lengthTol;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[NMARCH] %-4s %-18s singX=%d NTgaps=%d closed=%d onCurve=%.2e onSurf=%.2e "
              "lenDelta=%.2e (before=%.4f nat=%.4f occt=%.4f) occtBr=%d\n",
              ok ? "PASS" : "FAIL", "freeform-pole s4e", ts.singularitiesCrossed, ts.nearTangentGaps,
              ts.closedCurves, maxOnCurve, maxOnSurf, lenDelta, lenBefore, nativeLen, occtLen, occtN);
  std::fflush(stdout);
}

}  // namespace

int main() {
  std::printf("== SSI Stage S3/S4-c/S4-d marching-line tracer native-vs-OCCT parity ==\n");
  std::fflush(stdout);

  pairBSplineBSpline();
  pairBSplinePlane();
  pairSkewCylinders();
  pairSkewCylindersGeneral();    // M1b: general skew cyl∩cyl (gap+oblique) single quartic loop vs OCCT
  pairSphereConeOffAxis();       // M1b: off-axis sphere∩cone (sphere-bounded) vs OCCT
  pairConeConeGeneral();         // M1c: general cone∩cone (clipped oracle) single loop vs OCCT
  pairCylConeOffAxis();          // M1c: off-axis cyl∩cone (clipped oracle) open arc vs OCCT
  pairSphereCylTwicePiercing();  // M1c: off-axis sphere∩cyl twice-piercing (clipped oracle + recall bump) 2 loops vs OCCT
  pairCrossingSpheres();
  pairSphereBezier();
  pairNearTangentCrossedS4c();   // S4-c: graze marched through, full curve vs OCCT
  pairDeepNearTangentReanchorS4c();      // M1d: DEEPER graze crossed by adaptive re-anchoring (default declines) vs OCCT
  pairDeepNearTangentHonestDeclineS4c(); // M1d: below the extended floor, native honestly declines while OCCT reports
  pairWideBandIncrementalOrientationS4c(); // M1e: 90°-turn reversal trap fixed → wide-band grazes crossed vs OCCT
  pairWideBandHonestDeclineS4c();          // M1e: below minCrossSine native declines honestly while OCCT reports
  pairEqualCylindersDefer();     // S4-c: branch saddle still deferred (not crossed) — control
  pairEqualCylindersBranchS4d(); // S4-d: branch points localized + arms routed vs OCCT
  pairFreeformSaddleBranchS4d(); // S4-d-g: FREEFORM saddle open-arm branch localized + routed vs OCCT
  pairSpherePoleS4e();           // S4-e: sphere parametric pole crossed, full great circle vs OCCT
  pairConeApexS4e();             // S4-e: cone apex crossed, both nappes vs OCCT
  pairFreeformPoleS4e();         // S4-e: FREEFORM (NURBS) parametric pole crossed, full circle vs OCCT

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
