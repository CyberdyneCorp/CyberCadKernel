// SPDX-License-Identifier: Apache-2.0
//
// native_numerics_parity.mm — native-numerics (closest-point / Extrema) vs OCCT
// parity harness (iOS simulator).
//
// Phase 4 capability #2 (`native-numerics`, numeric-foundations), simulator
// verification gate 2 — the native-vs-OCCT parity pass (openspec/NATIVE-REWRITE.md).
//
// This exercises the native CLOSEST-POINT / projection layer
// (src/native/numerics/closest_point.h — `project_point_to_curve` /
// `project_point_to_surface`), which is built on top of the OCCT-FREE numeric
// facade (src/native/numerics/numerics.{h,cpp}) over the adopted NumPP + SciPP
// substrate (docs/EVAL-numpp-scipp.md, verdict GO-WITH-HARDENING). For a set of
// 3D targets and native geometries — Plane, Cylinder, Sphere, a B-spline surface,
// and a B-spline curve — it computes the native nearest point and compares it to
// OCCT's Extrema (via GeomAPI_ProjectPointOnSurf / GeomAPI_ProjectPointOnCurve,
// which wrap Extrema_ExtPS / Extrema_ExtPC) on IDENTICAL geometry built on both
// sides. Any divergence is a solver/algorithm mismatch, not a modelling one.
//
// For every (geometry, target) case we require, within a tight fp64 tolerance:
//   * SAME nearest distance         (|dNative − dOCCT|)
//   * foot point within tol         (‖Pnative − Pocct‖)
//   * parameter within tol          (curve t; surface (u,v)) — wrap-aware for the
//     periodic elementary surfaces (cylinder u, sphere u), where OCCT and the
//     native search may report the same physical point at u differing by 2π.
//
// Because a parametric distance function can have several equidistant global
// minima (e.g. a target on a cylinder's axis is equidistant to the whole rim),
// the parameter check is SKIPPED for a case only when the foot point is genuinely
// non-unique; those cases still assert distance + a valid foot point on-surface.
//
// This file is OCCT-DEPENDENT (it links the Extrema oracle) AND NumSci-DEPENDENT
// (it compiles src/native/numerics/numerics.cpp under CYBERCAD_HAS_NUMSCI and
// links the NumPP/SciPP archive). It lives under tests/sim, is compiled ONLY by
// scripts/run-sim-native-numerics.sh, and is on the SKIP list of run-sim-suite.sh
// (its own main(); links the geometry-oracle slice of OCCT + the numeric
// substrate, not the whole kernel static lib).
//
// Output: one [NNUM] PASS/FAIL line per case with per-case distance / param
// deltas, then a final "== N passed, M failed ==". Flushes and std::_Exit (OCCT
// static teardown in the trimmed static build is not exit-clean — same rationale
// as native_math_parity / parity_bench).

#include "native/numerics/native_numerics.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_numerics_parity requires -DCYBERCAD_HAS_OCCT and the OCCT Extrema oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_numerics_parity requires -DCYBERCAD_HAS_NUMSCI and the NumPP/SciPP substrate"
#endif

// ── OCCT oracle headers ──────────────────────────────────────────────────────
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Curve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>

namespace nm  = cybercad::native::math;
namespace nn  = cybercad::native::numerics;

// ═════════════════════════════════════════════════════════════════════════════
// Result accounting.
// ═════════════════════════════════════════════════════════════════════════════
static int g_pass = 0;
static int g_fail = 0;

// Tight fp64 tolerances. The distance and foot point are absolute geometric
// quantities (coords O(1..10) here), so an absolute 1e-7 is a strict bar for a
// sampled + BFGS-refined minimum. Parameter tol is looser on a nearly-flat basin
// (a large change in u produces a tiny change in position), so 1e-5.
static constexpr double kTolDist  = 1e-7;
static constexpr double kTolPoint = 1e-7;
static constexpr double kTolParam = 1e-5;
static constexpr double kTwoPi    = 6.283185307179586476925286766559;

static double pointDist(const nm::Point3& p, const gp_Pnt& q) {
  const double dx = p.x - q.X(), dy = p.y - q.Y(), dz = p.z - q.Z();
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Wrap-aware parameter delta: minimal |a − b| modulo `period` (period<=0 → plain).
static double paramDelta(double a, double b, double period) {
  double d = std::fabs(a - b);
  if (period > 0.0) {
    d = std::fmod(d, period);
    if (d > 0.5 * period) d = period - d;
  }
  return d;
}

// One curve case: native project_point_to_* vs the OCCT oracle.
//
// ORACLE COMPLETENESS. GeomAPI_ProjectPointOnCurve wraps Extrema_ExtPC, which
// returns only the INTERIOR PERPENDICULAR feet C'(t)·(C(t)−P)=0 — it does NOT
// enumerate the curve's endpoints as candidate minima (Extrema_ExtPC tracks the
// distance to the range bounds internally as `TrimmedDistance`, but GeomAPI does
// not surface it). On a bounded/clamped curve whose true nearest point is an
// endpoint (no interior perpendicular is closer, and NbPoints() may even be 0),
// the raw GeomAPI LowerDistance is therefore NOT the true nearest — it is a
// farther interior extremum, or undefined. Comparing the native result (which
// correctly considers endpoints) against that incomplete value is an oracle bug,
// not a native error.
//
// So we complete the oracle exactly as a bounded-curve Extrema must: the true
// OCCT-side nearest is the minimum over {all GeomAPI perpendicular feet} ∪
// {C(t0), C(t1)}. `T` is the target; [t0,t1] is the clamped domain; `crv` is the
// same curve handle GeomAPI was built on, evaluated for the endpoint candidates.
static void reportCurve(const char* name, const nm::Point3& T,
                        const nn::CurveClosest& nat,
                        const GeomAPI_ProjectPointOnCurve& occ,
                        const Handle(Geom_Curve)& crv, double t0, double t1,
                        double periodT, bool checkParam) {
  // Assemble the complete OCCT candidate set: perpendicular feet + endpoints.
  double bestDO = std::numeric_limits<double>::max();
  double bestTO = t0;
  gp_Pnt bestPO;
  auto consider = [&](double t) {
    const gp_Pnt p = crv->Value(t);
    const double dx = p.X() - T.x, dy = p.Y() - T.y, dz = p.Z() - T.z;
    const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (d < bestDO) { bestDO = d; bestTO = t; bestPO = p; }
  };
  for (int i = 1; i <= occ.NbPoints(); ++i) consider(occ.Parameter(i));
  consider(t0);
  consider(t1);

  const bool done = nat.success && bestDO < std::numeric_limits<double>::max();
  double dDist = 0.0, dPt = 0.0, dParam = 0.0;
  bool ok = done;
  if (done) {
    dDist  = std::fabs(nat.distance - bestDO);
    dPt    = pointDist(nat.point, bestPO);
    dParam = paramDelta(nat.t, bestTO, periodT);
    ok = (dDist <= kTolDist) && (dPt <= kTolPoint) &&
         (!checkParam || dParam <= kTolParam);
  }
  std::printf("[NNUM] %-26s %s  dDist=%.3e dPoint=%.3e%s%.3e  (t=%.6f)\n",
              name, ok ? "PASS" : "FAIL", dDist, dPt,
              checkParam ? " dParam=" : " dParam(skip)=", dParam, nat.t);
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// Where a surface's U parameter is genuinely UNDEFINED, so comparing it to an oracle's U is
// meaningless rather than informative. This is a property of the SURFACE KIND, not of the
// numeric value of v: on a sphere v = ±π/2 is a pole and every u maps to the same point, but
// on a cylinder or a B-spline patch v = π/2 is an ordinary interior parameter carrying a real
// check. Gating on `std::fabs(v) - π/2` alone would therefore silently disable the U clause
// for any fixture whose v happens to land there — the check must know it is on a sphere.
enum class UDegeneracy {
  None,          ///< U is meaningful everywhere in the sampled domain.
  SphericalPole  ///< U is undefined at v = ±π/2 (and only there).
};

// One surface case: native project_point_to_* vs GeomAPI_ProjectPointOnSurf.
static void reportSurface(const char* name, const nn::SurfaceClosest& nat,
                          const GeomAPI_ProjectPointOnSurf& occ,
                          double periodU, double periodV, bool checkParam,
                          UDegeneracy uDegen = UDegeneracy::None) {
  const bool done = nat.success && occ.NbPoints() > 0;
  double dDist = 0.0, dPt = 0.0, dU = 0.0, dV = 0.0;
  bool atPole = false;
  bool ok = done;
  if (done) {
    const double dO = occ.LowerDistance();
    double uO = 0.0, vO = 0.0;
    occ.LowerDistanceParameters(uO, vO);
    const gp_Pnt pO = occ.NearestPoint();
    dDist = std::fabs(nat.distance - dO);
    dPt   = pointDist(nat.point, pO);
    dU    = paramDelta(nat.u, uO, periodU);
    dV    = paramDelta(nat.v, vO, periodV);
    // At a spherical pole the whole u-circle collapses to one point, so BOTH engines are
    // free to report any longitude and dU carries no information. dDist, dPoint and dV
    // still do, and they remain asserted — the case is narrowed, not disabled.
    constexpr double kHalfPi = 1.5707963267948966;
    atPole = (uDegen == UDegeneracy::SphericalPole) &&
             (std::fabs(std::fabs(nat.v) - kHalfPi) <= 1e-9);
    const bool checkU = checkParam && !atPole;
    ok = (dDist <= kTolDist) && (dPt <= kTolPoint) &&
         (!checkU || dU <= kTolParam) &&
         (!checkParam || dV <= kTolParam);
  }
  std::printf("[NNUM] %-26s %s  dDist=%.3e dPoint=%.3e%s(dU=%.3e%s dV=%.3e)  (u=%.4f v=%.4f)\n",
              name, ok ? "PASS" : "FAIL", dDist, dPt,
              checkParam ? " " : " skip", dU,
              atPole ? " [pole: u undefined, not asserted]" : "", dV, nat.u, nat.v);
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// ═════════════════════════════════════════════════════════════════════════════
// Shared geometry builders (native + OCCT from IDENTICAL data).
// ═════════════════════════════════════════════════════════════════════════════

// Collapse a FLAT knot vector into OCCT's (distinct knots, multiplicities).
static void flatToKnotsMults(const std::vector<double>& flat,
                             std::vector<double>& knots, std::vector<int>& mults) {
  knots.clear(); mults.clear();
  for (double k : flat) {
    if (!knots.empty() && std::fabs(k - knots.back()) < 1e-15) ++mults.back();
    else { knots.push_back(k); mults.push_back(1); }
  }
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

// ═════════════════════════════════════════════════════════════════════════════
// CASE GROUP 1 — Plane.
//
// Native math::Plane S(u,v)=O+uX+vY vs OCCT Geom_Plane (same Ax3). The nearest
// point of a plane is the orthogonal foot; (u,v) is unique, so params ARE checked.
// The native search window is the finite window we hand it (planes are infinite);
// we make it comfortably enclose the foot so the boundary is never the answer.
// ═════════════════════════════════════════════════════════════════════════════
static void groupPlane() {
  // A tilted plane: origin (1,2,3), Z axis (0,0,1) rotated — use an oblique frame.
  const nm::Point3 O{1, 2, 3};
  const nm::Dir3 zAx{0.3, -0.2, 1.0};      // plane normal (will normalize)
  const nm::Dir3 xRef{1.0, 0.0, 0.0};
  const nm::Ax3 fr = nm::Ax3::fromAxisAndRef(O, zAx, xRef);
  const nm::Plane pl{fr};

  gp_Ax3 ax(gp_Pnt(O.x, O.y, O.z), gp_Dir(zAx.x(), zAx.y(), zAx.z()),
            gp_Dir(fr.x.x(), fr.x.y(), fr.x.z()));
  Handle(Geom_Plane) gpl = new Geom_Plane(ax);

  const nm::Point3 targets[] = {{5, 5, 10}, {-3, 4, 0}, {10, -8, 6}, {1, 2, 12}};
  int i = 0;
  for (const nm::Point3& T : targets) {
    // Search window ±40 in each plane parameter around origin — encloses the foot.
    nn::SurfaceClosest nat = nn::project_point_to_surface(T, pl, -40, 40, -40, 40, 24, 24);
    GeomAPI_ProjectPointOnSurf occ(gp_Pnt(T.x, T.y, T.z), gpl,
                                   -40.0, 40.0, -40.0, 40.0);
    std::string nm = "plane#" + std::to_string(i++);
    reportSurface(nm.c_str(), nat, occ, /*periodU*/ 0, /*periodV*/ 0, /*param*/ true);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// CASE GROUP 2 — Cylinder.
//
// Native math::Cylinder (radius R, angular u, axial v) vs OCCT
// Geom_CylindricalSurface (same Ax3, R). u is PERIODIC (2π); we compare u
// wrap-aware and v directly. A target ON the axis is equidistant to the whole rim
// (non-unique foot) — excluded here; all targets are off-axis so the foot is
// unique.
// ═════════════════════════════════════════════════════════════════════════════
static void groupCylinder() {
  const nm::Point3 O{0, 0, 0};
  const nm::Dir3 zAx{0, 0, 1};
  const nm::Dir3 xRef{1, 0, 0};
  const nm::Ax3 fr = nm::Ax3::fromAxisAndRef(O, zAx, xRef);
  const double R = 3.0;
  const nm::Cylinder cy{fr, R};

  gp_Ax3 ax(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
  Handle(Geom_CylindricalSurface) gcy = new Geom_CylindricalSurface(ax, R);

  // Off-axis targets (unique nearest rim point). Axial extent searched v∈[-10,10].
  const nm::Point3 targets[] = {{7, 0, 2}, {0, 5, -3}, {-6, 6, 8}, {2, -2, 0}};
  int i = 0;
  for (const nm::Point3& T : targets) {
    nn::SurfaceClosest nat =
        nn::project_point_to_surface(T, cy, 0.0, kTwoPi, -10.0, 10.0, 32, 24);
    GeomAPI_ProjectPointOnSurf occ(gp_Pnt(T.x, T.y, T.z), gcy,
                                   0.0, kTwoPi, -10.0, 10.0);
    std::string nm = "cylinder#" + std::to_string(i++);
    reportSurface(nm.c_str(), nat, occ, /*periodU*/ kTwoPi, /*periodV*/ 0, /*param*/ true);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// CASE GROUP 3 — Sphere.
//
// Native math::Sphere (u longitude 0..2π, v latitude -π/2..π/2) vs OCCT
// Geom_SphericalSurface (same convention). Both u and v are compared wrap-aware
// (u periodic 2π). A target at the CENTRE is equidistant to the whole sphere
// (non-unique) — excluded; all targets are off-centre.
// ═════════════════════════════════════════════════════════════════════════════
static void groupSphere() {
  const nm::Point3 C{2, 1, -1};
  const nm::Dir3 zAx{0, 0, 1};
  const nm::Dir3 xRef{1, 0, 0};
  const nm::Ax3 fr = nm::Ax3::fromAxisAndRef(C, zAx, xRef);
  const double R = 4.0;
  const nm::Sphere sp{fr, R};

  gp_Ax3 ax(gp_Pnt(C.x, C.y, C.z), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
  Handle(Geom_SphericalSurface) gsp = new Geom_SphericalSurface(ax, R);

  const nm::Point3 targets[] = {{12, 1, -1}, {2, 1, 15}, {-8, -6, 4}, {5, 4, 2}};
  int i = 0;
  for (const nm::Point3& T : targets) {
    nn::SurfaceClosest nat = nn::project_point_to_surface(
        T, sp, 0.0, kTwoPi, -1.5707963267948966, 1.5707963267948966, 32, 24);
    GeomAPI_ProjectPointOnSurf occ(gp_Pnt(T.x, T.y, T.z), gsp);
    std::string nm = "sphere#" + std::to_string(i++);
    // sphere#1's target {2,1,15} sits on the +z axis through the centre, so its foot is the
    // NORTH POLE — u is undefined there. Declared per-surface-kind, so the other three
    // targets (all off-axis) keep their full u check.
    reportSurface(nm.c_str(), nat, occ, /*periodU*/ kTwoPi, /*periodV*/ 0, /*param*/ true,
                  UDegeneracy::SphericalPole);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// CASE GROUP 4 — B-spline SURFACE (bicubic 4×4, matching the EVAL EXP1 shape).
//
// Native surfacePoint / project_point_to_bspline_surface vs OCCT
// Geom_BSplineSurface built from the SAME poles + clamped knots. Interior foot,
// unique — params checked. This is the freeform closest-point the eval flagged
// USABLE-AS-IS (EXP1, err 7.4e-6 vs brute force).
// ═════════════════════════════════════════════════════════════════════════════
static void groupBSplineSurface() {
  const int degU = 3, degV = 3, nU = 4, nV = 4;
  // A gently undulating bicubic patch over an XY grid with varied Z.
  std::vector<nm::Point3> poles;
  poles.reserve(nU * nV);
  const double zTab[4][4] = {
      {0.0, 0.5, 0.5, 0.0},
      {0.5, 1.5, 1.5, 0.5},
      {0.5, 1.5, 1.5, 0.5},
      {0.0, 0.5, 0.5, 0.0}};
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j)
      poles.push_back({double(i), double(j), zTab[i][j]});
  // Clamped uniform knots for a degree-3, 4-pole curve: [0,0,0,0,1,1,1,1].
  const std::vector<double> flat = {0, 0, 0, 0, 1, 1, 1, 1};
  nm::SurfaceGrid grid{poles, nU, nV};

  // OCCT surface from the same poles + (knots,mults).
  std::vector<double> knots; std::vector<int> mults;
  flatToKnotsMults(flat, knots, mults);
  TColgp_Array2OfPnt occPoles(1, nU, 1, nV);
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j) {
      const nm::Point3& p = poles[static_cast<std::size_t>(i) * nV + j];
      occPoles.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  TColStd_Array1OfReal uK = toReal1(knots), vK = toReal1(knots);
  TColStd_Array1OfInteger uM = toInt1(mults), vM = toInt1(mults);
  Handle(Geom_BSplineSurface) gsurf =
      new Geom_BSplineSurface(occPoles, uK, vK, uM, vM, degU, degV,
                              Standard_False, Standard_False);

  // Targets above/around the patch, close enough that the foot is interior.
  const nm::Point3 targets[] = {
      {1.5, 1.5, 5.0}, {0.7, 2.2, 4.0}, {2.4, 0.6, 3.5}, {1.5, 1.5, -3.0}};
  int i = 0;
  for (const nm::Point3& T : targets) {
    nn::SurfaceClosest nat = nn::project_point_to_bspline_surface(
        T, degU, degV, grid, flat, flat, 20, 20);
    GeomAPI_ProjectPointOnSurf occ(gp_Pnt(T.x, T.y, T.z), gsurf);
    std::string nm = "bspline_surf#" + std::to_string(i++);
    reportSurface(nm.c_str(), nat, occ, 0, 0, /*param*/ true);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// CASE GROUP 5 — B-spline CURVE (cubic, 5 poles).
//
// Native curvePoint / project_point_to_bspline_curve vs OCCT Geom_BSplineCurve
// from the SAME poles + clamped knots. Non-periodic (clamped) → t compared
// directly. A wiggly curve is exactly where MULTI-START seeding matters.
// ═════════════════════════════════════════════════════════════════════════════
static void groupBSplineCurve() {
  const int degree = 3;
  std::vector<nm::Point3> poles = {
      {0, 0, 0}, {1, 2, 0}, {2, -1, 1}, {3, 2, 0}, {4, 0, 1}};
  // Clamped knots for degree 3, 5 poles: n+p+2 = 4+3+2 = 9 knots.
  const std::vector<double> flat = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  // Clamped domain [t0,t1] = [knots[degree], knots[end-degree]] = [0, 1].
  const double t0 = flat[static_cast<std::size_t>(degree)];
  const double t1 = flat[flat.size() - 1 - static_cast<std::size_t>(degree)];

  std::vector<double> knots; std::vector<int> mults;
  flatToKnotsMults(flat, knots, mults);
  TColgp_Array1OfPnt occPoles(1, static_cast<int>(poles.size()));
  for (int i = 0; i < static_cast<int>(poles.size()); ++i)
    occPoles.SetValue(i + 1, gp_Pnt(poles[i].x, poles[i].y, poles[i].z));
  TColStd_Array1OfReal occKnots = toReal1(knots);
  TColStd_Array1OfInteger occMults = toInt1(mults);
  Handle(Geom_BSplineCurve) gcur =
      new Geom_BSplineCurve(occPoles, occKnots, occMults, degree, Standard_False);
  Handle(Geom_Curve) gcrv = gcur;

  const nm::Point3 targets[] = {
      {2, 3, 2}, {1, -2, -1}, {3.5, 3, 2}, {2.0, 0.0, -2.0}, {0.0, -1.0, 0.5}};
  int i = 0;
  for (const nm::Point3& T : targets) {
    nn::CurveClosest nat =
        nn::project_point_to_bspline_curve(T, degree, poles, flat, 48);
    GeomAPI_ProjectPointOnCurve occ(gp_Pnt(T.x, T.y, T.z), gcur);
    std::string nm = "bspline_curve#" + std::to_string(i++);
    reportCurve(nm.c_str(), T, nat, occ, gcrv, t0, t1, /*periodT*/ 0, /*param*/ true);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// CASE GROUP 6 — facade evaluator path (numerics.h closest_point_on_surface).
//
// Prove the thin substrate-free facade (not just the typed closest_point.h
// overloads) also matches OCCT: project onto the SAME bicubic patch via a plain
// SurfaceEval lambda. This is the escape-hatch API the kernel calls for any
// geometry without a dedicated overload.
// ═════════════════════════════════════════════════════════════════════════════
static void groupFacadeEval() {
  const int degU = 3, degV = 3, nU = 4, nV = 4;
  std::vector<nm::Point3> poles;
  poles.reserve(nU * nV);
  const double zTab[4][4] = {
      {0.0, 0.5, 0.5, 0.0}, {0.5, 1.5, 1.5, 0.5},
      {0.5, 1.5, 1.5, 0.5}, {0.0, 0.5, 0.5, 0.0}};
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j) poles.push_back({double(i), double(j), zTab[i][j]});
  const std::vector<double> flat = {0, 0, 0, 0, 1, 1, 1, 1};
  nm::SurfaceGrid grid{poles, nU, nV};

  std::vector<double> knots; std::vector<int> mults;
  flatToKnotsMults(flat, knots, mults);
  TColgp_Array2OfPnt occPoles(1, nU, 1, nV);
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j) {
      const nm::Point3& p = poles[static_cast<std::size_t>(i) * nV + j];
      occPoles.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  TColStd_Array1OfReal uK = toReal1(knots), vK = toReal1(knots);
  TColStd_Array1OfInteger uM = toInt1(mults), vM = toInt1(mults);
  Handle(Geom_BSplineSurface) gsurf =
      new Geom_BSplineSurface(occPoles, uK, vK, uM, vM, degU, degV,
                              Standard_False, Standard_False);

  const nm::Point3 T{1.5, 1.5, 5.0};
  nn::SurfaceEval eval = [&](double u, double v) {
    return nm::surfacePoint(degU, degV, grid, flat, flat, u, v);
  };
  nn::SurfaceProjection natF =
      nn::closest_point_on_surface(eval, 0, 1, 0, 1, T, 20, 20);

  GeomAPI_ProjectPointOnSurf occ(gp_Pnt(T.x, T.y, T.z), gsurf);
  // Adapt the facade struct into the SurfaceClosest reporter shape.
  nn::SurfaceClosest sc;
  sc.success = natF.success; sc.u = natF.u; sc.v = natF.v;
  sc.point = natF.point; sc.distance = natF.distance;
  reportSurface("facade_eval_bsurf", sc, occ, 0, 0, /*param*/ true);
}

int main() {
  std::printf("== native-numerics (closest-point / Extrema) vs OCCT parity ==\n");
  std::printf("   substrate: %s\n", nn::substrate_versions().c_str());
  std::fflush(stdout);

  groupPlane();
  groupCylinder();
  groupSphere();
  groupBSplineSurface();
  groupBSplineCurve();
  groupFacadeEval();

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
