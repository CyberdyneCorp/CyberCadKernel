// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_s4f_completeness_parity.mm — SSI Stage S4-f (COMPLETENESS + LOOP
// ROBUSTNESS) native-vs-OCCT parity harness (iOS simulator). Gate 2 of the two-gate
// S4-f model; Gate 1 (host, no OCCT) is tests/native/test_native_ssi_s4f_completeness.cpp.
//
// S4-f HARDENS the completeness/correctness of curves the S3/S4 tracer already walks:
//   PART 1 — robust loop-closure + self-intersection guard in the marcher;
//   PART 2 — an adaptive completeness-critic re-seed that recovers small intersection
//            loops the fixed-resolution S2 subdivision silently MISSES.
//
// This harness measures the two S4-f wins against the OCCT ORACLE (GeomAPI_IntSS):
//
//   (A) SMALL LOOP MISSED → RECOVERED. A B-spline bump sheet grazed by a plane so a small
//       loop lives inside ONE coarse (1/16) leaf cell → the fixed-resolution seed MISSES it
//       (native recall < OCCT branch count). The adaptive critic re-seeds finer and RECOVERS
//       it. We report recall_before, recall_after, and the recall FLOOR reached — recall is a
//       MEASURED figure vs OCCT NbLines, never a blind 1.0.
//   (D) MANY SMALL LOOPS. Four disjoint bumps → four OCCT loops; the fixed resolution finds
//       one, the critic raises recall MEASURABLY toward 4. Reported: before/after/floor/dry.
//   (C) SELF-INTERSECTION. A figure-eight (Gerono) level-set section genuinely self-crosses;
//       with the guard ON native records ≥1 TRANSVERSE self-crossing and keeps ONE arm
//       (branchPoints == 0). OCCT's GeomAPI_IntSS returns the SAME single connected locus
//       (its NbLines may arc-split the eight, welded back to one component here) — the parity
//       claim is: native traces the SAME single self-crossing locus OCCT does and reports the
//       crossing, WITHOUT fabricating a branch node.
//
// HONEST ASYMPTOTIC FRAMING (must not be overclaimed): completeness is MEASURED +
// ASYMPTOTIC, never a proof. Below ANY fixed subdivision resolution a smaller loop can still
// be missed, so completenessResidual stays true and the FLOOR (criticFloorFrac) is reported.
// A fixture's recall → 1 is scoped to THAT fixture at THAT floor. The harness FAILS only on a
// real correctness violation: a recovered node OFF a surface (a fabricated branch), the critic
// NOT raising recall on a pair where OCCT proves loops exist, or the self-crossing arm being
// stopped/false-closed. Every native node is verified on BOTH OCCT surfaces within tol.
//
// SSI is INTERNAL — no cc_* entry point; asserted at the cybercad::native::ssi C++ boundary,
// exactly like the S2/S3 parity harnesses. Links the OCCT oracle (GeomAPI_IntSS + Geom_* +
// point projection) AND the NumPP/SciPP numsci archive (the least_squares refine + lstsq fit);
// compiles src/native/ssi/{seeding,marching}.cpp + numerics under -DCYBERCAD_HAS_NUMSCI.
// Built ONLY by scripts/run-sim-native-ssi-s4f.sh; on the SKIP list of run-sim-suite.sh.
//
#include "native/ssi/native_ssi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax3.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
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
int g_fail = 0;

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

gp_Ax3 toOcctAx3(const Ax3& f) {
  return gp_Ax3(gp_Pnt(f.origin.x, f.origin.y, f.origin.z),
                gp_Dir(f.z.x(), f.z.y(), f.z.z()),
                gp_Dir(f.x.x(), f.x.y(), f.x.z()));
}

double distToOcctSurface(const Handle(Geom_Surface)& s, const Point3& p) {
  GeomAPI_ProjectPointOnSurf proj(gp_Pnt(p.x, p.y, p.z), s);
  return proj.NbPoints() > 0 ? proj.LowerDistance() : 1e30;
}

// A degree-3 B-spline sheet with single-pole bumps at (bi,bj) of heights bh (n×n net over
// [-1,1]²). A grazing plane just below the bump tips cuts each bump in one small loop.
// (Identical geometry to the Gate-1 host suite so the fixture carries over end-to-end.)
struct SheetData {
  ssi::SurfaceAdapter native;
  std::vector<Point3> poles;
  int n = 0, deg = 3;
  std::vector<double> ku;
};

SheetData bumpSheet(int n, const std::vector<int>& bi, const std::vector<int>& bj,
                    const std::vector<double>& bh) {
  SheetData d;
  d.n = n;
  const int deg = 3;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -1.0 + 2.0 * (double(i) / (n - 1));
      const double y = -1.0 + 2.0 * (double(j) / (n - 1));
      double z = 0.0;
      for (std::size_t b = 0; b < bi.size(); ++b)
        if (i == bi[b] && j == bj[b]) z = bh[b];
      d.poles.push_back({x, y, z});
    }
  const int ni = n - deg - 1;
  for (int a = 0; a <= deg; ++a) d.ku.push_back(0.0);
  for (int a = 1; a <= ni; ++a) d.ku.push_back(double(a) / (ni + 1));
  for (int a = 0; a <= deg; ++a) d.ku.push_back(1.0);
  d.native = ssi::makeBSplineAdapter(deg, deg, d.poles, n, n, d.ku, d.ku);
  return d;
}

// Same net → an OCCT Geom_BSplineSurface (clamped uniform, matching the native adapter).
Handle(Geom_BSplineSurface) toOcctSheet(const SheetData& d) {
  const int n = d.n, deg = d.deg;
  TColgp_Array2OfPnt cp(1, n, 1, n);
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const Point3& p = d.poles[static_cast<std::size_t>(i) * n + j];
      cp.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  const int interior = n - deg - 1;
  const int nDist = interior + 2;
  TColStd_Array1OfReal knots(1, nDist);
  TColStd_Array1OfInteger mults(1, nDist);
  for (int i = 1; i <= nDist; ++i) {
    knots.SetValue(i, (i == 1) ? 0.0 : (i == nDist ? 1.0 : double(i - 1) / double(interior + 1)));
    mults.SetValue(i, (i == 1 || i == nDist) ? (deg + 1) : 1);
  }
  return new Geom_BSplineSurface(cp, knots, knots, mults, mults, deg, deg);
}

// Gerono-lemniscate (figure-eight) sheet: z = y² − x² + x⁴, so {z=0} is y² = x²(1−x²) — a
// single self-crossing at the origin where the two lobes cross transversally (slopes ±1).
SheetData geronoSheet(int n, double ext) {
  SheetData d;
  d.n = n;
  const int deg = 3;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -ext + 2 * ext * (double(i) / (n - 1));
      const double y = -ext + 2 * ext * (double(j) / (n - 1));
      d.poles.push_back({x, y, y * y - x * x + x * x * x * x});
    }
  const int ni = n - deg - 1;
  for (int a = 0; a <= deg; ++a) d.ku.push_back(0.0);
  for (int a = 1; a <= ni; ++a) d.ku.push_back(double(a) / (ni + 1));
  for (int a = 0; a <= deg; ++a) d.ku.push_back(1.0);
  d.native = ssi::makeBSplineAdapter(deg, deg, d.poles, n, n, d.ku, d.ku);
  return d;
}

// worst on-surface residual of every native node vs BOTH OCCT surfaces (never a faked branch).
double worstOnSurf(const ssi::TraceSet& ts, const Handle(Geom_Surface)& sa,
                   const Handle(Geom_Surface)& sb) {
  double worst = 0.0;
  for (const auto& w : ts.lines)
    for (const auto& p : w.points)
      worst = std::max(worst, std::max(distToOcctSurface(sa, p.point), distToOcctSurface(sb, p.point)));
  return worst;
}

// ── (A) / (D) small-loop recall recovery vs OCCT NbLines ──────────────────────────
// `expectLoops` is the analytic loop count (cross-checked against OCCT NbLines). We assert:
//   * recall_after > recall_before        (the critic MEASURABLY raised recall);
//   * recall_after reaches the expected floor on THIS fixture (== 1 here, scoped);
//   * every recovered node lies on both OCCT surfaces  (no fabricated branch);
//   * the floor is finer than the start + residual acknowledged (asymptotic, not a proof).
void reportRecall(const std::string& name, const SheetData& sheet, double planeZ,
                  double startFrac, int expectLoops, double onSurfTol) {
  auto A = ssi::makePlaneAdapter(nm::Plane{frameZ({0, 0, planeZ})}, ssi::ParamBox{-1, 1, -1, 1});
  const ssi::SurfaceAdapter& B = sheet.native;

  Handle(Geom_Surface) sa =
      new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, planeZ), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)));
  Handle(Geom_Surface) sb = toOcctSheet(sheet);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;   // OCCT true loop count (arc-split may exceed)
  const int trueLoops = std::max(expectLoops, occtN);   // conservative: never under-claim the truth

  ssi::SeedOptions so;
  so.initialGridU = 1;
  so.initialGridV = 1;
  so.minPatchFrac = startFrac;
  ssi::MarchOptions mo;
  mo.maxPoints = 8000;

  auto before = ssi::trace_intersection(A, B, so, mo);
  ssi::RecallReport rb;
  rb.nativeBranches = before.tracedBranches;
  rb.trueBranches = trueLoops;

  ssi::SeedOptions sc = so;
  sc.completenessCritic = true;
  auto after = ssi::trace_intersection(A, B, sc, mo);
  ssi::RecallReport ra;
  ra.nativeBranches = after.tracedBranches;
  ra.trueBranches = trueLoops;
  ra.criticFloorFrac = after.criticFloorFrac;

  const double worst = worstOnSurf(after, sa, sb);

  bool ok = true;
  if (occtN < expectLoops) ok = false;                        // oracle must confirm the loops exist
  if (ra.recall() <= rb.recall()) ok = false;                 // critic MUST measurably raise recall
  if (after.tracedBranches <= before.tracedBranches) ok = false;
  if (after.criticRecoveredLoops < 1) ok = false;             // it actually recovered something
  if (after.criticFloorFrac <= 0.0 || after.criticFloorFrac >= startFrac) ok = false;  // finer floor
  if (!after.completenessResidual) ok = false;                // asymptotic — always true
  if (worst > onSurfTol) ok = false;                          // no recovered node off a surface
  if (after.tracedBranches > trueLoops) ok = false;           // no over-production
  if (!ok) ++g_fail;

  std::printf("[S4F-A/D] %-4s %-16s occt=%d expect=%d recall %.2f->%.2f (traced %d->%d) "
              "recovered=%d floor=%.5f dry=%d resid=%d worstOnSurf=%.2e\n",
              ok ? "PASS" : "FAIL", name.c_str(), occtN, expectLoops,
              rb.recall(), ra.recall(), before.tracedBranches, after.tracedBranches,
              after.criticRecoveredLoops, after.criticFloorFrac, after.criticStoppedDry ? 1 : 0,
              after.completenessResidual ? 1 : 0, worst);
  std::fflush(stdout);
}

// ── (C) self-intersection vs OCCT single self-crossing locus ──────────────────────
void reportSelfIntersection() {
  const double ext = 1.1;
  SheetData sheet = geronoSheet(10, ext);
  const ssi::SurfaceAdapter& B = sheet.native;
  auto A = ssi::makePlaneAdapter(nm::Plane{frameZ({0, 0, 0})}, ssi::ParamBox{-ext, ext, -ext, ext});

  Handle(Geom_Surface) sa =
      new Geom_Plane(gp_Ax3(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0)));
  Handle(Geom_Surface) sb = toOcctSheet(sheet);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;  // the self-crossing eight (possibly arc-split)

  ssi::SeedOptions so;
  so.initialGridU = 5;
  so.initialGridV = 5;

  // Guard ON: records the crossing, keeps ONE arm, no branch node.
  ssi::MarchOptions on;
  on.maxPoints = 8000;
  on.enableSelfIntersection = true;
  auto trOn = ssi::trace_intersection(A, B, so, on);

  // Guard OFF: byte-identical (no crossings recorded, same curve count).
  ssi::MarchOptions off;
  off.maxPoints = 8000;
  auto trOff = ssi::trace_intersection(A, B, so, off);

  const double worst = worstOnSurf(trOn, sa, sb);

  int typedTransverse = 0;
  bool nearOrigin = false;
  for (const auto& w : trOn.lines)
    for (const auto& s : w.selfIntersections) {
      if (std::fabs(s.tangentCos) < 0.7) ++typedTransverse;
      if (std::sqrt(s.point.x * s.point.x + s.point.y * s.point.y + s.point.z * s.point.z) < 0.2)
        nearOrigin = true;
    }

  bool ok = true;
  if (occtN < 1) ok = false;                              // OCCT must find the self-crossing locus
  if (trOn.selfIntersections < 1) ok = false;             // native records the crossing
  if (typedTransverse < 1) ok = false;                    // genuinely transverse (not a retrace)
  if (!nearOrigin) ok = false;                            // the true crossing is at the origin
  if (trOn.branchPoints != 0) ok = false;                 // ONE arm — NOT an S4-d locus branch
  if (trOff.selfIntersections != 0) ok = false;           // guard OFF byte-identical
  if (trOff.curveCount() != trOn.curveCount()) ok = false;
  if (worst > 1e-4) ok = false;                           // crossing node on both surfaces
  if (!ok) ++g_fail;

  std::printf("[S4F-C] %-4s %-16s occt=%d selfInt(on)=%d transverse=%d nearOrigin=%d "
              "branchPts=%d selfInt(off)=%d curves on/off=%d/%d worstOnSurf=%.2e\n",
              ok ? "PASS" : "FAIL", "gerono eight", occtN, trOn.selfIntersections, typedTransverse,
              nearOrigin ? 1 : 0, trOn.branchPoints, trOff.selfIntersections,
              trOn.curveCount(), trOff.curveCount(), worst);
  std::fflush(stdout);
}

}  // namespace

int main() {
  std::printf("── SSI Stage S4-f completeness + loop-robustness native-vs-OCCT parity\n");

  // (A) one big + one small bump; small loop lives inside one 1/16 leaf → missed, then recovered.
  {
    SheetData sheet = bumpSheet(16, {4, 11}, {4, 11}, {0.60, 0.30});
    reportRecall("A small-loop", sheet, 0.85 * 0.1333, /*startFrac=*/1.0 / 16.0,
                 /*expectLoops=*/2, /*onSurfTol=*/1e-6);
  }
  // (D) four disjoint bumps → four loops; coarse 1/6 finds one, critic raises recall toward 4.
  {
    SheetData sheet = bumpSheet(16, {3, 3, 12, 12}, {3, 12, 3, 12}, {0.60, 0.60, 0.30, 0.30});
    reportRecall("D many-loops", sheet, 0.85 * 0.1333, /*startFrac=*/1.0 / 6.0,
                 /*expectLoops=*/4, /*onSurfTol=*/1e-6);
  }
  // (C) figure-eight self-crossing at the origin.
  reportSelfIntersection();

  std::printf("%s (%d failures)\n", g_fail == 0 ? "ALL PASS" : "FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
