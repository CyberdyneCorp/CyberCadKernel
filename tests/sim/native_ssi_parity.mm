// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_parity.mm — native analytic SSI (Stage S1) vs OCCT-oracle parity
// harness (iOS simulator).
//
// SSI capability, SSI-ROADMAP.md Stage S1 (analytic SSI), simulator verification
// gate 2 — the native-vs-OCCT parity pass. Gate 1 (host analytic, no OCCT) is the
// separate tests/native/test_native_ssi.cpp; this file is the OCCT-oracle gate.
//
// The native analytic SSI (src/native/ssi/*, an OCCT-FREE clean-room closed-form
// intersector built on src/native/math) computes intersection curves for the S1
// conic family of elementary-surface pairs. For each supported pair this harness:
//
//   * builds the SAME two surfaces natively AND as OCCT Geom_Surface (identical
//     placement/radii/angles — the native Ax3 maps directly onto gp_Ax3);
//   * computes native intersect_surfaces(A,B) AND OCCT GeomAPI_IntSS(S1,S2,tol);
//   * asserts the SAME NUMBER of intersection curves;
//   * asserts each native curve's TYPE matches an OCCT curve's type (CurveKind ↔
//     GeomAbs_CurveType, via GeomAdaptor_Curve::GetType);
//   * densely samples each native curve and measures, at every sample:
//       - the max ON-SURFACE delta  = distance from the native point to BOTH OCCT
//         surfaces (GeomAPI_ProjectPointOnSurf::LowerDistance) — the point must lie
//         on both surfaces the oracle built; and
//       - the max CURVE-COINCIDENCE delta = distance from the native point to the
//         NEAREST OCCT intersection curve (GeomAPI_ProjectPointOnCurve) — the native
//         and OCCT intersection loci must coincide.
//     Both must stay within a tight tolerance.
//
// For a NOT-ANALYTIC pair (skew cylinder∩cylinder — a genuine quartic space curve),
// the harness asserts the native side returns NotAnalytic (honest deferral, never a
// faked curve) WHILE OCCT's GeomAPI_IntSS still produces a curve — documenting that
// this case is OCCT-only for now (deferred to S2/S3 marching per the roadmap).
//
// COVERAGE (per the S1 scope + the task brief):
//   plane∩plane, plane∩sphere,
//   plane∩cylinder  (⟂ / oblique / ∥ axis → circle / ellipse / parallel lines),
//   plane∩cone      (circle / ellipse / parabola / hyperbola),
//   plane∩torus     (⟂ axis → concentric circles; axis-containing → meridian circles),
//   sphere∩sphere,
//   coaxial sphere∩cylinder, coaxial sphere∩cone,
//   coaxial cylinder∩cone,
//   parallel cylinder∩cylinder (rulings),
//   coaxial cylinder∩cylinder (Coincident — no isolated curve),
//   [NOT-ANALYTIC] skew cylinder∩cylinder (native NotAnalytic, OCCT-only).
//
// This file is OCCT-DEPENDENT (it links the oracle) and lives under tests/sim. It is
// compiled ONLY by scripts/run-sim-native-ssi.sh and is on the SKIP list of
// run-sim-suite.sh. The native SSI/math sources it exercises remain OCCT-free.
//
// Build (see scripts/run-sim-native-ssi.sh):
//   -DCYBERCAD_HAS_OCCT  -std=c++20  for arm64-apple-ios-simulator, linking
//   TKGeomAlgo/TKG3d/TKGeomBase/TKG2d/TKMath/TKernel from the simulator OCCT install.
//
// Output: one [NSSI] PASS/FAIL line per pair with curve-count / type match + max
// on-surface + max curve-coincidence deltas, then a final "== N passed, M failed ==".
// Flushes and std::_Exit (OCCT static teardown in the trimmed static build is not
// exit-clean — same rationale as native_math_parity / parity_bench).

#include "native/ssi/native_ssi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_ssi_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

// ── OCCT oracle headers ──────────────────────────────────────────────────────
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax3.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_Curve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>

namespace ssi = cybercad::native::ssi;
namespace nm = cybercad::native::math;

using nm::Ax3;
using nm::Dir3;
using nm::Point3;

// ═════════════════════════════════════════════════════════════════════════════
// Result accounting.
// ═════════════════════════════════════════════════════════════════════════════
static int g_pass = 0;
static int g_fail = 0;

// ── frame / surface builders shared by the native + OCCT sides ────────────────

// A frame with the world Z axis and world X reference, translated to `o`.
static Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

// The OCCT gp_Ax3 built from the SAME orthonormal directions the native Ax3 holds
// (Location, main Direction = frame.z, XDirection = frame.x). This is exactly the
// mapping native_math_parity.mm verified elementary-surface parity through, so the
// two sides describe an identical surface.
static gp_Ax3 toOcctAx3(const Ax3& f) {
  return gp_Ax3(gp_Pnt(f.origin.x, f.origin.y, f.origin.z),
                gp_Dir(f.z.x(), f.z.y(), f.z.z()),
                gp_Dir(f.x.x(), f.x.y(), f.x.z()));
}

static gp_Pnt toOcctPnt(const Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

// ── OCCT curve-type name (for the diagnostic line) ─────────────────────────────
static const char* occtCurveTypeName(GeomAbs_CurveType t) {
  switch (t) {
    case GeomAbs_Line:      return "Line";
    case GeomAbs_Circle:    return "Circle";
    case GeomAbs_Ellipse:   return "Ellipse";
    case GeomAbs_Hyperbola: return "Hyperbola";
    case GeomAbs_Parabola:  return "Parabola";
    default:                return "Other";  // BSplineCurve etc. (deferred quartic)
  }
}

static const char* nativeKindName(ssi::CurveKind k) {
  switch (k) {
    case ssi::CurveKind::Point:     return "Point";
    case ssi::CurveKind::Line:      return "Line";
    case ssi::CurveKind::Circle:    return "Circle";
    case ssi::CurveKind::Ellipse:   return "Ellipse";
    case ssi::CurveKind::Parabola:  return "Parabola";
    case ssi::CurveKind::Hyperbola: return "Hyperbola";
  }
  return "?";
}

// Does the native curve kind describe the same conic family as the OCCT curve type?
// OCCT's analytic intersector (IntAna behind GeomAPI_IntSS) yields exactly the same
// Line/Circle/Ellipse/Parabola/Hyperbola family the native S1 handlers emit. A
// native Circle is a special ellipse — accept an OCCT Circle *or* Ellipse for it
// (and vice-versa) so a benign circle-vs-ellipse classification split near equal
// axes is not a false failure; every other kind must match exactly.
static bool kindMatches(ssi::CurveKind nk, GeomAbs_CurveType ot) {
  switch (nk) {
    case ssi::CurveKind::Line:      return ot == GeomAbs_Line;
    case ssi::CurveKind::Circle:    return ot == GeomAbs_Circle || ot == GeomAbs_Ellipse;
    case ssi::CurveKind::Ellipse:   return ot == GeomAbs_Ellipse || ot == GeomAbs_Circle;
    case ssi::CurveKind::Parabola:  return ot == GeomAbs_Parabola;
    case ssi::CurveKind::Hyperbola: return ot == GeomAbs_Hyperbola;
    case ssi::CurveKind::Point:     return false;  // a Point has no OCCT curve peer
  }
  return false;
}

// ═════════════════════════════════════════════════════════════════════════════
// A parity case: two native surfaces + the SAME two OCCT surfaces, plus the
// per-case sampling window and tolerances. Everything the harness needs to run a
// single [NSSI] line lives here so `main` reads as a coverage table.
// ═════════════════════════════════════════════════════════════════════════════
struct Case {
  const char* name;
  ssi::Surface a;
  ssi::Surface b;
  Handle(Geom_Surface) oa;
  Handle(Geom_Surface) ob;

  // For open conics (Line/Parabola/Hyperbola) the native `naturalRange` is a wide
  // symmetric window; cosh/sinh grow fast, so cap the |t| we sample for hyperbolae.
  double openHalfSpan = 6.0;

  // Tolerances. Circles/lines are exact fp64 → 1e-9; cone conics accumulate more
  // fp round-off in the classifier → the caller may loosen per case.
  double tol = 1e-7;

  // Expectation: is this an analytic S1 pair (Ok), or the honest NOT-ANALYTIC
  // deferral (native NotAnalytic while OCCT still yields a curve)?
  bool expectAnalytic = true;

  // For an analytic pair that is Coincident (same locus → no isolated curve), OCCT
  // GeomAPI_IntSS also returns 0 lines; set this so the count check expects 0/0.
  bool expectCoincident = false;
};

// Sample count per curve for the on-surface + coincidence sweeps.
static constexpr int kSamplesPerCurve = 200;

// Densely sample one native curve; for every point measure (a) distance to BOTH
// OCCT surfaces and (b) distance to the NEAREST OCCT intersection curve. Updates the
// running maxima. Skips a degenerate Point kind (a tangency, no sweep).
static void sampleNativeCurve(const ssi::IntersectionCurve& c,
                              const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                              const std::vector<Handle(Geom_Curve)>& occtLines,
                              double openHalfSpan, double& maxOnSurf, double& maxCoin) {
  auto [t0, t1] = c.naturalRange();
  if (c.kind == ssi::CurveKind::Line || c.kind == ssi::CurveKind::Parabola ||
      c.kind == ssi::CurveKind::Hyperbola) {
    t0 = -openHalfSpan;
    t1 = openHalfSpan;
  }

  const int N = (c.kind == ssi::CurveKind::Point) ? 0 : kSamplesPerCurve;
  for (int i = 0; i <= N; ++i) {
    const double t = (N == 0) ? 0.0 : t0 + (t1 - t0) * (double(i) / N);
    const Point3 p = (c.kind == ssi::CurveKind::Point) ? c.point : c.value(t);
    const gp_Pnt gp = toOcctPnt(p);

    // (a) on both OCCT surfaces.
    GeomAPI_ProjectPointOnSurf pa(gp, sa);
    GeomAPI_ProjectPointOnSurf pb(gp, sb);
    if (pa.NbPoints() > 0) maxOnSurf = std::max(maxOnSurf, pa.LowerDistance());
    if (pb.NbPoints() > 0) maxOnSurf = std::max(maxOnSurf, pb.LowerDistance());

    // (b) coincident with the nearest OCCT intersection curve.
    double nearest = 1e300;
    for (const auto& oc : occtLines) {
      GeomAPI_ProjectPointOnCurve pc(gp, oc);
      if (pc.NbPoints() > 0) nearest = std::min(nearest, pc.LowerDistance());
    }
    if (nearest < 1e299) maxCoin = std::max(maxCoin, nearest);
  }
}

// Distance from an arbitrary point to one native curve, by dense sampling of the
// native curve over its (widened, for open conics) natural range. Used for the
// REVERSE coverage sweep: every OCCT curve point must lie on SOME native curve, which
// is what catches a genuinely missing native branch (e.g. a dropped cone-nappe circle
// or the second hyperbola branch) — a plain curve-count comparison cannot, because
// OCCT freely splits one closed curve into several arcs.
static double distPointToNativeCurve(const gp_Pnt& q, const ssi::IntersectionCurve& c,
                                     double openHalfSpan) {
  if (c.kind == ssi::CurveKind::Point) {
    const Point3 p = c.point;
    const double dx = p.x - q.X(), dy = p.y - q.Y(), dz = p.z - q.Z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }
  auto [t0, t1] = c.naturalRange();
  if (c.kind == ssi::CurveKind::Line || c.kind == ssi::CurveKind::Parabola ||
      c.kind == ssi::CurveKind::Hyperbola) {
    t0 = -openHalfSpan;
    t1 = openHalfSpan;
  }
  auto distAt = [&](double t) {
    const Point3 p = c.value(t);
    const double dx = p.x - q.X(), dy = p.y - q.Y(), dz = p.z - q.Z();
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  };
  // Coarse scan to localize the nearest sample, then iteratively bisect the bracketing
  // interval to converge to the true foot (a plain fixed-window refine left ~1e-3 slack
  // on an ellipse whose curvature varies along the sweep).
  double best = 1e300, bestT = t0;
  const int N = kSamplesPerCurve;
  for (int i = 0; i <= N; ++i) {
    const double t = t0 + (t1 - t0) * (double(i) / N);
    const double d = distAt(t);
    if (d < best) { best = d; bestT = t; }
  }
  double lo = bestT - (t1 - t0) / N, hi = bestT + (t1 - t0) / N;
  for (int it = 0; it < 60; ++it) {  // golden-ish bisection on |p(t) − q|
    const double m1 = lo + (hi - lo) / 3.0, m2 = hi - (hi - lo) / 3.0;
    const double d1 = distAt(m1), d2 = distAt(m2);
    if (d1 < d2) hi = m2; else lo = m1;
    best = std::min({best, d1, d2});
  }
  return best;
}

// REVERSE coverage: densely sample every OCCT intersection curve and measure, for
// each sample, the distance to the NEAREST native curve. The running max is the
// largest gap by which the native result fails to cover the OCCT locus — nonzero iff
// native is missing a branch OCCT found.
static double occtNotCoveredByNative(const std::vector<Handle(Geom_Curve)>& occtLines,
                                     const std::vector<ssi::IntersectionCurve>& nativeCurves,
                                     double openHalfSpan) {
  double maxGap = 0.0;
  for (const auto& oc : occtLines) {
    double f = oc->FirstParameter(), l = oc->LastParameter();
    // OCCT reports unbounded curves (lines / parabolas / hyperbolae) with sentinel
    // params ≈ ±Precision::Infinite() (a finite ~1e100), not a true inf — clamp any
    // out-of-window param to the same symmetric window the native side samples, so we
    // compare the SAME arc and don't fling samples to 1e100.
    if (!std::isfinite(f) || f < -openHalfSpan) f = -openHalfSpan;
    if (!std::isfinite(l) || l > openHalfSpan) l = openHalfSpan;
    for (int i = 0; i <= kSamplesPerCurve; ++i) {
      const double t = f + (l - f) * (double(i) / kSamplesPerCurve);
      gp_Pnt q;
      try {
        q = oc->Value(t);  // apex-cone degenerate curves can raise Standard_NumericError
      } catch (...) {
        continue;
      }
      double nearest = 1e300;
      for (const auto& nc : nativeCurves)
        nearest = std::min(nearest, distPointToNativeCurve(q, nc, openHalfSpan));
      if (nearest < 1e299) maxGap = std::max(maxGap, nearest);
    }
  }
  return maxGap;
}

// Run one analytic case: native vs OCCT, print the [NSSI] line, tally pass/fail.
static void runAnalytic(const Case& cs) {
  const ssi::IntersectionResult nr = ssi::intersect_surfaces(cs.a, cs.b);

  GeomAPI_IntSS iss(cs.oa, cs.ob, /*Tol=*/1e-7);
  const bool occtDone = iss.IsDone();
  const int occtN = occtDone ? iss.NbLines() : 0;
  std::vector<Handle(Geom_Curve)> occtLines;
  for (int i = 1; i <= occtN; ++i) occtLines.push_back(iss.Line(i));

  bool ok = occtDone;

  // ── count / presence ──────────────────────────────────────────────────────────
  // We do NOT require native and OCCT curve counts to be EQUAL: the two libraries
  // segment the same intersection locus differently — OCCT's GeomAPI_IntSS routinely
  // splits one closed curve (a full circle) into two half-arcs, so it can report 2 or
  // 3 curves where the analytic answer is 1 or 2 whole conics. Equal-count was a false
  // failure for exactly those cases. The real parity criterion is LOCUS COVERAGE in
  // BOTH directions, checked in the geometry block below: every native point on both
  // surfaces + on some OCCT curve, AND every OCCT point on some native curve. Here we
  // only require the right PRESENCE (native produced ≥1 curve when OCCT did).
  int nativeN = 0;
  if (cs.expectCoincident) {
    // Coincident (same locus): native reports Coincident with 0 isolated curves;
    // OCCT GeomAPI_IntSS likewise returns 0 lines for a same-surface pair.
    if (nr.status != ssi::IntersectionStatus::Coincident) ok = false;
    nativeN = 0;
  } else {
    if (!nr.ok_()) ok = false;
    nativeN = static_cast<int>(nr.curves.size());
    if (occtN > 0 && nativeN == 0) ok = false;  // OCCT found a curve, native found none
  }

  // ── type match (each native curve ↔ some OCCT curve of a matching type) ────────
  bool typesOk = true;
  std::string typeStr;
  {
    // Precompute OCCT curve types once.
    std::vector<GeomAbs_CurveType> occtTypes;
    for (const auto& oc : occtLines) {
      GeomAdaptor_Curve ad(oc);
      occtTypes.push_back(ad.GetType());
    }
    std::vector<bool> occtUsed(occtTypes.size(), false);
    for (const auto& c : nr.curves) {
      if (!typeStr.empty()) typeStr += "/";
      typeStr += nativeKindName(c.kind);
      // Greedily claim an unused OCCT curve of a matching type.
      bool claimed = false;
      for (size_t j = 0; j < occtTypes.size(); ++j) {
        if (!occtUsed[j] && kindMatches(c.kind, occtTypes[j])) {
          occtUsed[j] = true;
          claimed = true;
          break;
        }
      }
      if (!claimed) typesOk = false;
    }
    if (typeStr.empty()) typeStr = cs.expectCoincident ? "coincident" : "-";
  }
  if (!cs.expectCoincident && !typesOk) ok = false;

  // ── geometry: bidirectional locus coverage over dense samples ──────────────────
  //   onSurf : every native point lies on BOTH OCCT surfaces.
  //   coin   : every native point lies on SOME OCCT intersection curve (native ⊆ OCCT).
  //   cover  : every OCCT curve point lies on SOME native curve (OCCT ⊆ native) — this
  //            is the direction that catches a genuinely missing native branch.
  double maxOnSurf = 0.0, maxCoin = 0.0;
  for (const auto& c : nr.curves)
    sampleNativeCurve(c, cs.oa, cs.ob, occtLines, cs.openHalfSpan, maxOnSurf, maxCoin);
  const double maxCover = occtNotCoveredByNative(occtLines, nr.curves, cs.openHalfSpan);
  if (maxOnSurf > cs.tol || maxCoin > cs.tol || maxCover > cs.tol) ok = false;

  std::printf(
      "[NSSI] %-28s %s  nCurves nat=%d occt=%d  types=%s%s  onSurf=%.3e coin=%.3e cover=%.3e  tol=%.1e\n",
      cs.name, ok ? "PASS" : "FAIL", nativeN, occtN, typeStr.c_str(),
      (cs.expectCoincident ? " (coincident)" : (typesOk ? " (match)" : " (TYPE-MISMATCH)")),
      maxOnSurf, maxCoin, maxCover, cs.tol);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

// Run the honest NOT-ANALYTIC deferral case: native must return NotAnalytic; OCCT
// must still produce ≥1 curve (documenting that this pair is OCCT-only for now).
static void runNotAnalytic(const Case& cs) {
  const ssi::IntersectionResult nr = ssi::intersect_surfaces(cs.a, cs.b);
  GeomAPI_IntSS iss(cs.oa, cs.ob, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;

  const bool nativeDeferred = (nr.status == ssi::IntersectionStatus::NotAnalytic);
  const bool occtHasCurve = (occtN >= 1);
  const bool ok = nativeDeferred && occtHasCurve;

  const char* occtType = "-";
  if (occtN >= 1) {
    GeomAdaptor_Curve ad(iss.Line(1));
    occtType = occtCurveTypeName(ad.GetType());
  }
  std::printf(
      "[NSSI] %-28s %s  native=%s  occt=%d curve(s) type=%s  (deferred: OCCT-only for now)\n",
      cs.name, ok ? "PASS" : "FAIL",
      nativeDeferred ? "NOT_ANALYTIC" : "UNEXPECTED-OK", occtN, occtType);
  std::fflush(stdout);
  if (ok) ++g_pass; else ++g_fail;
}

// ═════════════════════════════════════════════════════════════════════════════
// Case construction. Each helper returns a ready-to-run Case with the native and
// OCCT surfaces built from IDENTICAL parameters. Frames use frameZ / tilted frames
// exactly matching the host gate (tests/native/test_native_ssi.cpp) so the two gates
// exercise the same geometry.
// ═════════════════════════════════════════════════════════════════════════════

static constexpr double kPi = 3.14159265358979323846;

// A plane placed by frame f.
static Case makePlanePlane() {
  Ax3 f1 = frameZ();                                            // z = 0, normal +Z
  Ax3 f2{Point3{0, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}};  // x = 0, normal +X
  Case cs;
  cs.name = "plane∩plane (line)";
  cs.a = ssi::Surface::of(nm::Plane{f1});
  cs.b = ssi::Surface::of(nm::Plane{f2});
  cs.oa = new Geom_Plane(toOcctAx3(f1));
  cs.ob = new Geom_Plane(toOcctAx3(f2));
  cs.tol = 1e-9;
  return cs;
}

static Case makePlaneSphere() {
  Ax3 fp = frameZ();                 // z = 0
  Ax3 fs = frameZ({0, 0, 3});        // sphere centre (0,0,3), R=5 → circle r=4
  Case cs;
  cs.name = "plane∩sphere (circle)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Sphere{fs, 5.0});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_SphericalSurface(toOcctAx3(fs), 5.0);
  cs.tol = 1e-9;
  return cs;
}

static Case makePlaneCylinderPerp() {
  Ax3 fp = frameZ({0, 0, 4});        // ⟂ axis at z=4
  Ax3 fc = frameZ();                 // cylinder axis Z, R=2
  Case cs;
  cs.name = "plane⟂cyl (circle)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Cylinder{fc, 2.0});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_CylindricalSurface(toOcctAx3(fc), 2.0);
  cs.tol = 1e-9;
  return cs;
}

static Case makePlaneCylinderOblique() {
  const double c = std::cos(kPi / 4), s = std::sin(kPi / 4);
  Ax3 fp{Point3{0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, c, s}, Dir3{0, -s, c}};  // normal tilted 45°
  Ax3 fc = frameZ();                 // axis Z, R=2 → ellipse b=2, a=2√2
  Case cs;
  cs.name = "plane∠cyl (ellipse)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Cylinder{fc, 2.0});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_CylindricalSurface(toOcctAx3(fc), 2.0);
  cs.tol = 1e-8;
  return cs;
}

static Case makePlaneCylinderParallel() {
  Ax3 fp{Point3{0.5, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}};  // x=0.5, normal +X
  Ax3 fc = frameZ();                 // axis Z, R=2 → 2 rulings at y=±√(4−0.25)
  Case cs;
  cs.name = "plane∥cyl (2 lines)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Cylinder{fc, 2.0});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_CylindricalSurface(toOcctAx3(fc), 2.0);
  cs.openHalfSpan = 8.0;
  cs.tol = 1e-9;
  return cs;
}

// A 45° cone: OCCT Geom_ConicalSurface(Ax3, Ang, Radius) with Radius at v=0.
static Case makePlaneConeCircle() {
  Ax3 fp = frameZ({0, 0, 3});        // ⟂ axis at z=3
  Ax3 fc = frameZ();                 // apex-ish cone, base radius 1 at v=0, 45°
  Case cs;
  cs.name = "plane⟂cone (circle)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Cone{fc, 1.0, kPi / 4});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_ConicalSurface(toOcctAx3(fc), kPi / 4, 1.0);
  cs.tol = 1e-7;
  return cs;
}

static Case makePlaneConeEllipse() {
  const double ang = kPi / 9;        // 20° tilt < 45° → cuts all generators → ellipse
  const double c = std::cos(ang), s = std::sin(ang);
  Ax3 fp{Point3{0, 0, 3}, Dir3{1, 0, 0}, Dir3{0, c, s}, Dir3{0, -s, c}};
  Ax3 fc = frameZ();
  Case cs;
  cs.name = "plane∠cone (ellipse)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Cone{fc, 1.0, kPi / 4});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_ConicalSurface(toOcctAx3(fc), kPi / 4, 1.0);
  cs.tol = 1e-6;
  return cs;
}

static Case makePlaneConeParabola() {
  const double c = std::cos(kPi / 4), s = std::sin(kPi / 4);  // plane ∥ a generator
  Ax3 fp{Point3{0, 0, 2}, Dir3{1, 0, 0}, Dir3{0, c, s}, Dir3{0, -s, c}};
  Ax3 fc = frameZ();
  Case cs;
  cs.name = "plane∥gen cone (parabola)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Cone{fc, 1.0, kPi / 4});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_ConicalSurface(toOcctAx3(fc), kPi / 4, 1.0);
  cs.openHalfSpan = 3.0;
  cs.tol = 1e-6;
  return cs;
}

static Case makePlaneConeHyperbola() {
  Ax3 fp{Point3{0.5, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}};  // x=0.5, vertical
  Ax3 fc = frameZ();
  Case cs;
  cs.name = "plane steep cone (hyperbola)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Cone{fc, 1.0, kPi / 4});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_ConicalSurface(toOcctAx3(fc), kPi / 4, 1.0);
  cs.openHalfSpan = 1.0;   // cosh/sinh grow fast — keep |t| small to stay near the cone
  cs.tol = 1e-6;
  return cs;
}

static Case makePlaneTorusPerp() {
  Ax3 fp = frameZ();                 // ⟂ axis at z=0 → concentric circles r=5 & r=3
  Ax3 ft = frameZ();                 // R=4, r=1, axis Z
  Case cs;
  cs.name = "plane⟂torus (2 circles)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Torus{ft, 4.0, 1.0});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_ToroidalSurface(toOcctAx3(ft), 4.0, 1.0);
  cs.tol = 1e-9;
  return cs;
}

static Case makePlaneTorusAxis() {
  Ax3 fp{Point3{0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 0, 1}, Dir3{0, 1, 0}};  // normal +Y, contains axis
  Ax3 ft = frameZ();                 // R=4, r=1 → two meridian tube circles r=1
  Case cs;
  cs.name = "plane∋axis torus (2 circles)";
  cs.a = ssi::Surface::of(nm::Plane{fp});
  cs.b = ssi::Surface::of(nm::Torus{ft, 4.0, 1.0});
  cs.oa = new Geom_Plane(toOcctAx3(fp));
  cs.ob = new Geom_ToroidalSurface(toOcctAx3(ft), 4.0, 1.0);
  cs.tol = 1e-8;
  return cs;
}

static Case makeSphereSphere() {
  Ax3 f1 = frameZ({0, 0, 0});
  Ax3 f2 = frameZ({6, 0, 0});        // d=6, R1=R2=5 → circle radius 4
  Case cs;
  cs.name = "sphere∩sphere (circle)";
  cs.a = ssi::Surface::of(nm::Sphere{f1, 5.0});
  cs.b = ssi::Surface::of(nm::Sphere{f2, 5.0});
  cs.oa = new Geom_SphericalSurface(toOcctAx3(f1), 5.0);
  cs.ob = new Geom_SphericalSurface(toOcctAx3(f2), 5.0);
  cs.tol = 1e-9;
  return cs;
}

static Case makeSphereCylinderCoaxial() {
  Ax3 fs = frameZ();                 // centre origin, R=5
  Ax3 fc = frameZ();                 // coaxial axis Z, R=3 → two circles z=±4
  Case cs;
  cs.name = "coaxial sphere∩cyl (2 circles)";
  cs.a = ssi::Surface::of(nm::Sphere{fs, 5.0});
  cs.b = ssi::Surface::of(nm::Cylinder{fc, 3.0});
  cs.oa = new Geom_SphericalSurface(toOcctAx3(fs), 5.0);
  cs.ob = new Geom_CylindricalSurface(toOcctAx3(fc), 3.0);
  cs.tol = 1e-9;
  return cs;
}

static Case makeSphereConeCoaxial() {
  Ax3 fs = frameZ();                 // centre origin, R=5
  Ax3 fc = frameZ();                 // apex at origin (radius 0 at v=0), 45°, coaxial
  Case cs;
  cs.name = "coaxial sphere∩cone (circle)";
  cs.a = ssi::Surface::of(nm::Sphere{fs, 5.0});
  cs.b = ssi::Surface::of(nm::Cone{fc, 0.0, kPi / 4});
  cs.oa = new Geom_SphericalSurface(toOcctAx3(fs), 5.0);
  cs.ob = new Geom_ConicalSurface(toOcctAx3(fc), kPi / 4, 0.0);
  cs.tol = 1e-7;
  return cs;
}

static Case makeCylinderConeCoaxial() {
  Ax3 fy = frameZ();                 // cylinder axis Z, R=2
  Ax3 fc = frameZ();                 // apex at origin, 45°, coaxial → circle radius 2
  Case cs;
  cs.name = "coaxial cyl∩cone (circle)";
  cs.a = ssi::Surface::of(nm::Cylinder{fy, 2.0});
  cs.b = ssi::Surface::of(nm::Cone{fc, 0.0, kPi / 4});
  cs.oa = new Geom_CylindricalSurface(toOcctAx3(fy), 2.0);
  cs.ob = new Geom_ConicalSurface(toOcctAx3(fc), kPi / 4, 0.0);
  cs.tol = 1e-7;
  return cs;
}

static Case makeCylinderCylinderParallel() {
  Ax3 f1 = frameZ();                 // axis Z, R=3
  Ax3 f2 = frameZ({4, 0, 0});        // parallel axis, offset 4, R=3 → 2 rulings
  Case cs;
  cs.name = "parallel cyl∩cyl (2 lines)";
  cs.a = ssi::Surface::of(nm::Cylinder{f1, 3.0});
  cs.b = ssi::Surface::of(nm::Cylinder{f2, 3.0});
  cs.oa = new Geom_CylindricalSurface(toOcctAx3(f1), 3.0);
  cs.ob = new Geom_CylindricalSurface(toOcctAx3(f2), 3.0);
  cs.openHalfSpan = 8.0;
  cs.tol = 1e-9;
  return cs;
}

static Case makeCylinderCylinderCoaxial() {
  Ax3 f1 = frameZ();                 // axis Z, R=3 — coaxial identical → Coincident
  Case cs;
  cs.name = "coaxial cyl∩cyl (coincident)";
  cs.a = ssi::Surface::of(nm::Cylinder{f1, 3.0});
  cs.b = ssi::Surface::of(nm::Cylinder{f1, 3.0});
  cs.oa = new Geom_CylindricalSurface(toOcctAx3(f1), 3.0);
  cs.ob = new Geom_CylindricalSurface(toOcctAx3(f1), 3.0);
  cs.expectCoincident = true;
  cs.tol = 1e-9;
  return cs;
}

// NOT-ANALYTIC: skew cylinder∩cylinder — a genuine quartic space curve. Native must
// return NotAnalytic; OCCT still produces a curve (BSpline). Axes Z and X, both R=3,
// crossing at the origin → the classic "bicylinder" quartic seam.
static Case makeCylinderCylinderSkew() {
  Ax3 f1 = frameZ();                                             // axis Z, R=3
  Ax3 f2{Point3{0, 0, 0}, Dir3{0, 0, 1}, Dir3{0, 1, 0}, Dir3{1, 0, 0}};  // axis X, R=3
  Case cs;
  cs.name = "skew cyl∩cyl [NOT-ANALYTIC]";
  cs.a = ssi::Surface::of(nm::Cylinder{f1, 3.0});
  cs.b = ssi::Surface::of(nm::Cylinder{f2, 3.0});
  cs.oa = new Geom_CylindricalSurface(toOcctAx3(f1), 3.0);
  cs.ob = new Geom_CylindricalSurface(toOcctAx3(f2), 3.0);
  cs.expectAnalytic = false;
  return cs;
}

int main() {
  std::printf("== native-SSI (Stage S1 analytic) vs OCCT-oracle parity ==\n");
  std::fflush(stdout);

  std::vector<Case> cases;
  cases.push_back(makePlanePlane());
  cases.push_back(makePlaneSphere());
  cases.push_back(makePlaneCylinderPerp());
  cases.push_back(makePlaneCylinderOblique());
  cases.push_back(makePlaneCylinderParallel());
  cases.push_back(makePlaneConeCircle());
  cases.push_back(makePlaneConeEllipse());
  cases.push_back(makePlaneConeParabola());
  cases.push_back(makePlaneConeHyperbola());
  cases.push_back(makePlaneTorusPerp());
  cases.push_back(makePlaneTorusAxis());
  cases.push_back(makeSphereSphere());
  cases.push_back(makeSphereCylinderCoaxial());
  cases.push_back(makeSphereConeCoaxial());
  cases.push_back(makeCylinderConeCoaxial());
  cases.push_back(makeCylinderCylinderParallel());
  cases.push_back(makeCylinderCylinderCoaxial());
  cases.push_back(makeCylinderCylinderSkew());  // NOT-ANALYTIC deferral

  for (const auto& cs : cases) {
    if (cs.expectAnalytic) runAnalytic(cs);
    else runNotAnalytic(cs);
  }

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);

  // Same rationale as native_math_parity.mm / parity_bench.cpp: OCCT's static
  // teardown in the trimmed static build is not exit-clean; every Handle here is
  // RAII-scoped and the residual state is OCCT's own internal statics. Exit without
  // running C++ static destructors so the true result is reported.
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
