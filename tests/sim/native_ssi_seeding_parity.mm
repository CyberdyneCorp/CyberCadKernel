// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_seeding_parity.mm — SSI Stage S2 (subdivision seeding) native-vs-OCCT
// per-branch RECALL parity harness (iOS simulator). Gate 2 of the two-gate S2 model;
// Gate 1 (host, no OCCT) is tests/native/test_native_ssi_seeding.cpp.
//
// This is the STRICTER companion to native_ssi_seeding_recall.mm. Where that harness
// reports recall from deduped seed COUNTS, this one asserts recall PER BRANCH by
// geometry: it takes OCCT GeomAPI_IntSS as the oracle for the TRUE intersection loci
// (each OCCT Geom_Curve = one branch) and, for EVERY transversal OCCT branch, requires
// that the native seeder produced ≥ 1 seed lying ON THAT OCCT CURVE (nearest point on
// the curve < tol, GeomAPI_ProjectPointOnCurve) AND on BOTH surfaces
// (GeomAPI_ProjectPointOnSurf::LowerDistance < tol). Counting alone cannot catch a
// native seed that landed on the WRONG branch or that OCCT arc-split a loop into
// several curves; matching seeds to curves does.
//
// TANGENTIAL branches are EXCLUDED from the recall denominator (S2 is transversal-only;
// tangent seeding ill-conditions the refine → S4). A branch is classified tangential
// when, sampled along the OCCT curve, ‖n₁ × n₂‖ (the sine of the angle between the two
// surface normals) stays below a tangency threshold everywhere on it. Such branches are
// reported separately (tang=…) and never counted against recall — but the harness also
// never fakes a seed for them.
//
// COVERAGE (freeform + non-closed-form pairs S1 defers as NotAnalytic):
//   * bspline ∩ bspline        — two freeform biquadratic B-spline sheets, one loop.
//   * bspline ∩ plane          — a rippled B-spline sheet cut by a plane → MULTI-LOOP
//                                (the completeness stressor: several small transversal
//                                 loops the subdivision must all reach).
//   * skew cyl ∩ cyl           — orthogonal unequal-radius cylinders → 2 transversal
//                                loops (the skew-quadric quartic S1 defers).
//   * sphere ∩ sphere          — crossing equal spheres → 1 transversal circle.
//
// PASS = recall == 100% of the TRANSVERSAL OCCT branches for the pair AND every emitted
// native seed lies on both surfaces within tol. A missed transversal branch (too-shallow
// subdivision) FAILS honestly and prints the miss; it is not hidden behind a seed count.
//
// SSI is INTERNAL — no cc_* entry point is called or added; asserted at the
// cybercad::native::ssi C++ boundary, exactly like the S1 parity harness.
//
// This TU is OCCT-dependent AND substrate-dependent: it links the OCCT oracle and the
// NumPP/SciPP numsci archive, and compiles src/native/ssi/seeding.cpp +
// src/native/numerics/numerics.cpp under -DCYBERCAD_HAS_NUMSCI. Built ONLY by
// scripts/run-sim-native-ssi-seeding.sh; on the SKIP list of run-sim-suite.sh.
//
// Output: one [NSEED] PASS/FAIL line per pair with per-pair transversal-branch recall,
// tangential-branch count, and the worst seed on-both-surfaces residual, then a final
// "== N passed, M failed ==". Flushes and std::_Exit (OCCT static teardown in the
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
#error "native_ssi_seeding_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_ssi_seeding_parity requires -DCYBERCAD_HAS_NUMSCI (the least_squares refine)"
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
#include <Geom_Curve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

namespace ssi = cybercad::native::ssi;
namespace nm = cybercad::native::math;
using nm::Ax3;
using nm::Dir3;
using nm::Point3;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kSamplesPerCurve = 96;

int g_pass = 0;
int g_fail = 0;

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

// Sine of the angle between the two OCCT surface normals at a 3D point, obtained by
// projecting the point onto each surface, reading the (u,v) foot, and taking the
// GeomLProp_SLProps normal there. This is the transversality witness used to classify
// an OCCT branch as transversal vs tangential — computed on the ORACLE surfaces so the
// classification does not depend on the native side. Returns -1 if a normal is
// undefined (degenerate patch) so the caller can treat it conservatively.
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
  const double s = na.Crossed(nb).Magnitude();  // ‖n₁ × n₂‖ for unit normals
  return s;
}

// A classified OCCT branch: its curve handle plus whether it is transversal (part of
// the recall denominator) or tangential (excluded, reported separately, an S4 gap).
struct OcctBranch {
  Handle(Geom_Curve) curve;
  bool transversal = true;
  double maxCrossingSine = 0.0;  // over samples; > tangentSine ⇒ transversal
};

// Sample one OCCT curve over its (clamped) parameter range and decide transversal vs
// tangential from the MAX ‖n₁×n₂‖ along it: a transversal branch crosses somewhere even
// if it grazes locally, so the max being above the tangency threshold means the branch
// is genuinely a crossing locus (S2's target). A branch whose normals stay parallel
// everywhere is tangential (S4).
OcctBranch classifyBranch(const Handle(Geom_Curve)& c,
                          const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                          double tangentSine) {
  OcctBranch b;
  b.curve = c;
  double f = c->FirstParameter(), l = c->LastParameter();
  if (!std::isfinite(f) || f < -1e6) f = -1e6;
  if (!std::isfinite(l) || l > 1e6) l = 1e6;
  double maxSine = 0.0;
  for (int i = 0; i <= kSamplesPerCurve; ++i) {
    const double t = f + (l - f) * (double(i) / kSamplesPerCurve);
    gp_Pnt q;
    try { q = c->Value(t); } catch (...) { continue; }
    const double s = crossingSineOnOcct(sa, sb, q);
    if (s >= 0.0) maxSine = std::max(maxSine, s);
  }
  b.maxCrossingSine = maxSine;
  b.transversal = (maxSine > tangentSine);
  return b;
}

// Report one pair. Runs native seeding + OCCT GeomAPI_IntSS, classifies OCCT branches,
// and asserts PER-BRANCH recall: every transversal OCCT branch must carry ≥ 1 native
// seed that lies on that OCCT curve (< onCurveTol) AND on both surfaces (< onSurfTol).
//   expectTransversalBranches  analytic truth, cross-checked against the classification.
//   tangentSine                threshold below which an OCCT branch is deemed tangential.
void reportPair(const std::string& pairName,
                const ssi::SurfaceAdapter& A, const ssi::SurfaceAdapter& B,
                const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                int expectTransversalBranches, double onSurfTol, double onCurveTol,
                double tangentSine, const ssi::SeedOptions& opt) {
  const ssi::SeedSet ss = ssi::seed_intersection(A, B, opt);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;

  // Classify every OCCT branch as transversal (recall denominator) or tangential (S4).
  std::vector<OcctBranch> branches;
  for (int i = 1; i <= occtN; ++i) branches.push_back(classifyBranch(iss.Line(i), sa, sb, tangentSine));
  int transversal = 0, tangential = 0;
  for (const auto& b : branches) (b.transversal ? transversal : tangential)++;

  // Worst on-both-surfaces residual over the emitted seeds (a seed OFF a surface fails).
  double worstOnSurf = 0.0;
  for (const auto& s : ss.seeds)
    worstOnSurf = std::max(worstOnSurf,
                           std::max(distToOcctSurface(sa, s.point), distToOcctSurface(sb, s.point)));

  // PER-BRANCH recall: a transversal branch is COVERED iff some native seed lies on its
  // OCCT curve (< onCurveTol) and on both surfaces (< onSurfTol).
  int covered = 0;
  for (const auto& b : branches) {
    if (!b.transversal) continue;
    bool hit = false;
    for (const auto& s : ss.seeds) {
      const bool onCurve = distToOcctCurve(b.curve, s.point) < onCurveTol;
      const bool onBoth = distToOcctSurface(sa, s.point) < onSurfTol &&
                          distToOcctSurface(sb, s.point) < onSurfTol;
      if (onCurve && onBoth) { hit = true; break; }
    }
    if (hit) ++covered;
  }
  const double recall = transversal > 0 ? double(covered) / double(transversal) : 1.0;

  bool ok = true;
  if (worstOnSurf > onSurfTol) ok = false;             // a seed OFF a surface → fail
  if (covered < transversal) ok = false;               // missed a transversal branch → fail
  if (transversal != expectTransversalBranches) ok = false;  // oracle disagreed with analytic truth
  if (ok) ++g_pass; else ++g_fail;

  std::printf("[NSEED] %-4s %-20s recall=%.2f (%d/%d transversal) tang=%d nativeSeeds=%d "
              "occtCurves=%d deferredTangent=%d worstOnSurf=%.2e\n",
              ok ? "PASS" : "FAIL", pairName.c_str(), recall, covered, transversal,
              tangential, ss.branchCount(), occtN, ss.deferredTangent, worstOnSurf);
  std::fflush(stdout);
}

// ── shared frame helpers ─────────────────────────────────────────────────────────

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

// Build a clamped uniform knot vector of length nPoles + degree + 1 over [0,1]:
// (degree+1) zeros, interior knots evenly spaced, (degree+1) ones.
std::vector<double> clampedKnots(int degree, int nPoles) {
  const int m = nPoles + degree + 1;
  std::vector<double> k(static_cast<std::size_t>(m), 0.0);
  const int interior = nPoles - degree - 1;  // #interior knots
  for (int i = 0; i < m; ++i) {
    if (i <= degree) k[i] = 0.0;
    else if (i >= nPoles) k[i] = 1.0;
    else k[i] = double(i - degree) / double(interior + 1);
  }
  return k;
}

// OCCT Geom_BSplineSurface from a row-major native pole grid + clamped-uniform knots
// over [0,1]² (identical parametrization to makeBSplineAdapter). `deg` is the degree in
// both directions; `nR`×`nC` poles.
Handle(Geom_BSplineSurface) toOcctBSpline(const std::vector<Point3>& poles, int nR, int nC,
                                          int degU, int degV) {
  TColgp_Array2OfPnt cp(1, nR, 1, nC);
  for (int i = 0; i < nR; ++i)
    for (int j = 0; j < nC; ++j) {
      const Point3& p = poles[static_cast<std::size_t>(i) * nC + j];
      cp.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  // Distinct knots + multiplicities in each direction (clamped-uniform).
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

// ── pair builders (native + OCCT, identical geometry) ──────────────────────────────

// bspline ∩ bspline — two biquadratic (deg 2×2, 3×3 poles) freeform sheets: a bump
// opening +Z and a facing dish opening −Z, overlapping in one transversal loop.
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
  ssi::SeedOptions opt; opt.initialGridU = 4; opt.initialGridV = 4;
  reportPair("bspline x bspline", A, B, sa, sb, /*branches=*/1, 1e-6, 1e-5, 1e-2, opt);
}

// bspline ∩ plane (MULTI-LOOP) — a rippled biquartic B-spline sheet (deg 2×2, 5×5
// poles, alternating high/low poles → an egg-carton) cut by the horizontal plane z=0.5;
// each ripple hump that pokes through the plane is its own transversal loop. The
// completeness stressor: the subdivision must reach every small loop.
void pairBSplinePlane() {
  const int n = 5;
  std::vector<Point3> poles;
  poles.reserve(static_cast<std::size_t>(n) * n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -1.0 + 2.0 * double(i) / (n - 1);
      const double y = -1.0 + 2.0 * double(j) / (n - 1);
      const double z = 0.5 + 0.55 * (((i + j) % 2 == 0) ? 1.0 : -1.0);  // egg-carton
      poles.push_back({x, y, z});
    }
  const auto kU = clampedKnots(2, n), kV = clampedKnots(2, n);
  auto A = ssi::makeBSplineAdapter(2, 2, poles, n, n, kU, kV);

  nm::Plane pl{Ax3{{0, 0, 0.5}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
  ssi::ParamBox pd{-1.5, 1.5, -1.5, 1.5};
  auto B = ssi::makePlaneAdapter(pl, pd);

  Handle(Geom_Surface) sa = toOcctBSpline(poles, n, n, 2, 2);
  Handle(Geom_Surface) sb = new Geom_Plane(toOcctAx3(Ax3{{0, 0, 0.5}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}}));

  // Multi-loop: pre-split finely and subdivide deeper so every small loop is reached.
  ssi::SeedOptions opt; opt.initialGridU = 8; opt.initialGridV = 8; opt.minPatchFrac = 1.0 / 48.0;
  // Denominator is whatever OCCT reports as transversal for this egg-carton (loop count
  // depends on how many humps clear z=0.5); cross-checked below by the classifier, not a
  // hardcoded analytic count — pass expect = the classifier's transversal tally.
  // We assert full recall of THAT set. See note in reportPair (expect == transversal).
  // Determine expected transversal branches from the oracle itself:
  GeomAPI_IntSS iss(sa, sb, 1e-7);
  int transversal = 0;
  for (int i = 1; i <= (iss.IsDone() ? iss.NbLines() : 0); ++i)
    if (classifyBranch(iss.Line(i), sa, sb, 1e-2).transversal) ++transversal;
  reportPair("bspline x plane", A, B, sa, sb, transversal, 1e-6, 1e-5, 1e-2, opt);
}

// skew cyl ∩ cyl — orthogonal unequal-radius cylinders → 2 transversal loops (the
// skew-quadric quartic S1 defers). OCCT may arc-split the loci; the per-branch match
// still requires a native seed on each transversal curve.
void pairSkewCylinders() {
  nm::Cylinder cz{frameZ(), 1.0};
  const Ax3 fx{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}};  // axis X
  nm::Cylinder cx{fx, 0.7};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);
  Handle(Geom_Surface) sa = new Geom_CylindricalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(fx), 0.7);
  ssi::SeedOptions opt; opt.initialGridU = 4; opt.initialGridV = 3;
  reportPair("skew cyl unequal", A, B, sa, sb, /*branches=*/2, 1e-6, 1e-5, 1e-2, opt);
}

// sphere ∩ sphere — crossing equal-radius spheres → 1 transversal circle. OCCT may
// arc-split the circle; per-branch match requires a native seed on the shared circle.
void pairCrossingSpheres() {
  nm::Sphere s1{frameZ(), 1.0};
  nm::Sphere s2{frameZ({1.0, 0, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(frameZ({1.0, 0, 0})), 1.0);
  ssi::SeedOptions opt; opt.initialGridU = 3; opt.initialGridV = 3;
  reportPair("sphere x sphere", A, B, sa, sb, /*branches=*/1, 1e-6, 1e-5, 1e-2, opt);
}

}  // namespace

int main() {
  std::printf("== SSI Stage S2 subdivision-seeding native-vs-OCCT per-branch RECALL ==\n");
  std::fflush(stdout);

  pairBSplineBSpline();
  pairBSplinePlane();
  pairSkewCylinders();
  pairCrossingSpheres();

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
