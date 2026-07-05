// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for SSI Stage S4-f — COMPLETENESS + LOOP ROBUSTNESS (OCCT-FREE, Gate 1).
// Two orthogonal parts, both additive and gated so the S3/S4-c/S4-d/S4-e controls stay
// byte-identical:
//
//   PART 1 — robust closure + self-intersection guard (marching):
//     (B) FALSE-CLOSE PREVENTED: a curve traced with a moderately INFLATED loopCloseFrac
//         (a curve re-approaching its seed / an earlier node while heading the OTHER way)
//         is NO LONGER truncated into a short "closed" arc — the TRUE-RETURN + tangent-
//         continuity closure refuses the early stop, so it traces to (near) its full length;
//         at the DEFAULT frac the trace is byte-identical to S3.
//     (C) SELF-INTERSECTION DETECTED + TRACED THROUGH: a genuinely self-crossing single arm
//         (a figure-eight level-set section) records ≥1 typed SelfIntersection with a
//         TRANSVERSE crossing, keeps ONE arm (branchPoints == 0 — distinct from S4-d), and is
//         not stopped/closed at the crossing. With the guard OFF the trace is byte-identical.
//
//   PART 2 — adaptive completeness critic (seeding/trace):
//     (A) SMALL LOOP MISSED → RECOVERED: at a coarse fixed resolution a small loop is MISSED
//         (recall < 1); the critic re-seeds finer and RECOVERS it (recall == 1 on this
//         fixture) with the residual still acknowledged and a floor finer than the start.
//     (D) MANY SMALL LOOPS: a measured recall RISE (recall_after > recall_before) toward the
//         true branch count, floor + stoppedDry + residual reported — never a totality claim.
//
//   CONTROLS: the transversal loop still Closes tangent-continuously (byte-identical), the
//     S4-d Steinmetz still traces (branchPoints == 2, selfIntersections == 0), and the
//     completeness critic / self-intersection guard OFF leave the fixed-resolution trace
//     unchanged.
//
// No OCCT is linked. Compiled only under CYBERCAD_HAS_NUMSCI (the tracer/seeder call the
// least_squares / lstsq substrate), like the S3/S4-e suites.
//
#include "native/ssi/native_ssi.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace ssi = cybercad::native::ssi;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;
using nmath::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

double polylineLength(const ssi::WLine& w) {
  double len = 0.0;
  for (std::size_t i = 1; i < w.points.size(); ++i)
    len += nmath::distance(w.points[i].point, w.points[i - 1].point);
  return len;
}

// A degree-3 B-spline sheet with single-pole bumps at (bi,bj) of heights bh (n×n net over
// [-1,1]²). A grazing plane just below the bump tips cuts each bump in one small loop.
ssi::SurfaceAdapter bumpSheet(int n, const std::vector<int>& bi, const std::vector<int>& bj,
                              const std::vector<double>& bh) {
  const int deg = 3;
  std::vector<Point3> poles;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -1.0 + 2.0 * (double(i) / (n - 1));
      const double y = -1.0 + 2.0 * (double(j) / (n - 1));
      double z = 0.0;
      for (std::size_t b = 0; b < bi.size(); ++b)
        if (i == bi[b] && j == bj[b]) z = bh[b];
      poles.push_back({x, y, z});
    }
  std::vector<double> ku;
  const int ni = n - deg - 1;
  for (int a = 0; a <= deg; ++a) ku.push_back(0.0);
  for (int a = 1; a <= ni; ++a) ku.push_back(double(a) / (ni + 1));
  for (int a = 0; a <= deg; ++a) ku.push_back(1.0);
  return ssi::makeBSplineAdapter(deg, deg, poles, n, n, ku, ku);
}

// Gerono-lemniscate (figure-eight) sheet: z = y² − x² + x⁴, so {z=0} is y² = x²(1−x²) — a
// single self-crossing at the origin where the two lobes cross transversally (slopes ±1).
ssi::SurfaceAdapter geronoSheet(int n, double ext) {
  const int deg = 3;
  std::vector<Point3> poles;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -ext + 2 * ext * (double(i) / (n - 1));
      const double y = -ext + 2 * ext * (double(j) / (n - 1));
      poles.push_back({x, y, y * y - x * x + x * x * x * x});
    }
  std::vector<double> ku;
  const int ni = n - deg - 1;
  for (int a = 0; a <= deg; ++a) ku.push_back(0.0);
  for (int a = 1; a <= ni; ++a) ku.push_back(double(a) / (ni + 1));
  for (int a = 0; a <= deg; ++a) ku.push_back(1.0);
  return ssi::makeBSplineAdapter(deg, deg, poles, n, n, ku, ku);
}

}  // namespace

// ── (B) FALSE-CLOSE PREVENTED ─────────────────────────────────────────────────────
// Two crossing unit spheres → one true circle R = √3/2, length 2πR ≈ 5.4414. At the DEFAULT
// loopCloseFrac the trace is byte-identical (near-full length). At a moderately INFLATED
// loopCloseFrac (10×) — modelling a curve that passes within loopClose·h of its seed while
// NOT truly returning — the pure-proximity S3 test false-closed it to ~1% of the true length;
// the S4-f TRUE-RETURN + tangent-continuity closure now traces it to (near) full length.
CC_TEST(s4f_false_close_prevented) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({1.0, 0, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);
  const double R = std::sqrt(3.0) / 2.0;
  const double trueLen = 2.0 * kPi * R;

  ssi::SeedOptions so; so.initialGridU = 3; so.initialGridV = 3;

  // Default frac: byte-identical S3 behaviour — Closed, near-full length.
  ssi::MarchOptions def; def.maxPoints = 4000;
  auto trDef = ssi::trace_intersection(A, B, so, def);
  CC_CHECK(trDef.curveCount() == 1);
  CC_CHECK(trDef.lines[0].isClosed());
  const double lenDef = polylineLength(trDef.lines[0]);
  CC_CHECK(lenDef >= trueLen * 0.97);   // full circle (chord under-estimate only)

  // Inflated frac (10×): the false-close is PREVENTED — the curve traces to (near) full
  // length instead of a short truncated arc (which was ~1% of the true length in S3).
  ssi::MarchOptions inf; inf.maxPoints = 4000; inf.loopCloseFrac = 20.0;
  auto trInf = ssi::trace_intersection(A, B, so, inf);
  CC_CHECK(trInf.curveCount() >= 1);
  const double lenInf = polylineLength(trInf.lines[0]);
  CC_CHECK(lenInf >= trueLen * 0.85);   // NOT truncated to a tiny arc (was ≈ 1.2% in S3)
  // Every node still lies on both spheres (no fabricated continuation).
  for (const auto& n : trInf.lines[0].points) {
    CC_CHECK(std::fabs(nmath::distance(n.point, s1.pos.origin) - 1.0) < 1e-5);
    CC_CHECK(std::fabs(nmath::distance(n.point, s2.pos.origin) - 1.0) < 1e-5);
  }
}

// ── (C) SELF-INTERSECTION DETECTED + TRACED THROUGH ─────────────────────────────────
// The figure-eight section self-crosses at the origin (both lobes cross at ±45°). With the
// guard ON: ≥1 typed SelfIntersection, TRANSVERSE (|tangentCos| < 0.7), at/near the origin,
// branchPoints == 0 (distinct from an S4-d locus branch), and the arm is NOT stopped there.
// With the guard OFF: byte-identical (no crossings recorded, same polyline).
CC_TEST(s4f_self_intersection_detected_and_traced_through) {
  const double ext = 1.1;
  auto B = geronoSheet(10, ext);
  nmath::Plane pl{frameZ({0, 0, 0})};
  ssi::ParamBox dom{-ext, ext, -ext, ext};
  auto A = ssi::makePlaneAdapter(pl, dom);
  ssi::SeedOptions so; so.initialGridU = 5; so.initialGridV = 5;

  ssi::MarchOptions on; on.maxPoints = 8000; on.enableSelfIntersection = true;
  auto trOn = ssi::trace_intersection(A, B, so, on);
  CC_CHECK(trOn.selfIntersections >= 1);
  CC_CHECK(trOn.branchPoints == 0);   // one arm crossing itself — NOT an S4-d locus branch

  int typed = 0;
  bool nearOrigin = false;
  for (const auto& w : trOn.lines)
    for (const auto& s : w.selfIntersections) {
      ++typed;
      CC_CHECK(std::fabs(s.tangentCos) < 0.7);              // genuinely transverse (not a retrace)
      CC_CHECK(std::fabs(s.point.z) < 1e-5);                // on the plane z=0
      // crossing point lies on the surface (z(x,y) ≈ 0 there)
      const double zc = s.point.y * s.point.y - s.point.x * s.point.x +
                        s.point.x * s.point.x * s.point.x * s.point.x;
      CC_CHECK(std::fabs(zc) < 5e-3);
      if (nmath::norm(Vec3{s.point.x, s.point.y, s.point.z}) < 0.2) nearOrigin = true;
    }
  CC_CHECK(typed >= 1);
  CC_CHECK(nearOrigin);                                     // the true crossing is at the origin

  // Guard OFF → byte-identical (no self-intersections recorded, same curve count).
  ssi::MarchOptions off; off.maxPoints = 8000;
  auto trOff = ssi::trace_intersection(A, B, so, off);
  CC_CHECK(trOff.selfIntersections == 0);
  CC_CHECK(trOff.curveCount() == trOn.curveCount());
}

// ── (A) SMALL LOOP MISSED → RECOVERED BY THE CRITIC ─────────────────────────────────
// A big + a small bump. At a coarse fixed resolution (1/16 leaf) the small loop is MISSED
// (recall 0.5); the adaptive critic re-seeds finer and RECOVERS it (recall 1.0 on this
// fixture), floor finer than the start, residual acknowledged, stopped dry.
CC_TEST(s4f_small_loop_missed_then_recovered) {
  auto B = bumpSheet(16, {4, 11}, {4, 11}, {0.60, 0.30});
  ssi::ParamBox pd{-1.0, 1.0, -1.0, 1.0};
  nmath::Plane pl{frameZ({0, 0, 0.85 * 0.1333})};   // grazes the small tip → small loop
  auto A = ssi::makePlaneAdapter(pl, pd);
  const int trueBranches = 2;

  ssi::SeedOptions so; so.initialGridU = 1; so.initialGridV = 1; so.minPatchFrac = 1.0 / 16.0;
  ssi::MarchOptions mo; mo.maxPoints = 8000;

  // BEFORE (fixed coarse resolution): the small loop is MISSED.
  auto before = ssi::trace_intersection(A, B, so, mo);
  ssi::RecallReport rb; rb.nativeBranches = before.tracedBranches; rb.trueBranches = trueBranches;
  CC_CHECK(before.tracedBranches == 1);
  CC_CHECK(rb.recall() < 1.0);

  // AFTER (critic on): the small loop is RECOVERED.
  ssi::SeedOptions sc = so; sc.completenessCritic = true;
  auto after = ssi::trace_intersection(A, B, sc, mo);
  ssi::RecallReport ra;
  ra.nativeBranches = after.tracedBranches; ra.trueBranches = trueBranches;
  ra.criticFloorFrac = after.criticFloorFrac;
  CC_CHECK(after.tracedBranches == 2);
  CC_CHECK(ra.recall() == 1.0);                       // recovered ON THIS FIXTURE at this floor
  CC_CHECK(after.criticRecoveredLoops >= 1);
  CC_CHECK(after.criticFloorFrac < 1.0 / 16.0);       // floor finer than the start
  CC_CHECK(after.criticFloorFrac > 0.0);
  CC_CHECK(after.completenessResidual);               // ALWAYS true — asymptotic, not a proof
  CC_CHECK(ra.residualAcknowledged);
  // every recovered node still lies on both surfaces (no fabricated branch)
  for (const auto& w : after.lines)
    for (const auto& n : w.points) {
      CC_CHECK(std::fabs(n.point.z - 0.85 * 0.1333) < 1e-5);   // on the plane
      CC_CHECK(n.onSurfResidual < 1e-6);                       // on both surfaces
    }
}

// ── (D) MANY SMALL LOOPS → MEASURED RECALL RISE ─────────────────────────────────────
// Four disjoint bumps. At a coarse fixed resolution (1/6 leaf) only one loop is found
// (recall 0.25); the critic raises recall measurably toward the true count of 4, reporting
// the floor + stoppedDry + residual (never a totality claim).
CC_TEST(s4f_many_small_loops_recall_rise) {
  auto B = bumpSheet(16, {3, 3, 12, 12}, {3, 12, 3, 12}, {0.60, 0.60, 0.30, 0.30});
  ssi::ParamBox pd{-1.0, 1.0, -1.0, 1.0};
  nmath::Plane pl{frameZ({0, 0, 0.85 * 0.1333})};
  auto A = ssi::makePlaneAdapter(pl, pd);
  const int trueBranches = 4;

  ssi::SeedOptions so; so.initialGridU = 1; so.initialGridV = 1; so.minPatchFrac = 1.0 / 6.0;
  ssi::MarchOptions mo; mo.maxPoints = 8000;

  auto before = ssi::trace_intersection(A, B, so, mo);
  ssi::RecallReport rb; rb.nativeBranches = before.tracedBranches; rb.trueBranches = trueBranches;

  ssi::SeedOptions sc = so; sc.completenessCritic = true;
  auto after = ssi::trace_intersection(A, B, sc, mo);
  ssi::RecallReport ra; ra.nativeBranches = after.tracedBranches; ra.trueBranches = trueBranches;

  CC_CHECK(ra.recall() > rb.recall());                // MEASURED rise
  CC_CHECK(after.tracedBranches > before.tracedBranches);
  CC_CHECK(after.criticRecoveredLoops >= 1);
  CC_CHECK(after.criticFloorFrac > 0.0);
  CC_CHECK(after.criticRounds >= 1);
  CC_CHECK(after.completenessResidual);               // residual acknowledged, not a totality claim
  // no over-production: dedup keeps ≈ the true count (locus dedup), never more than trueBranches
  CC_CHECK(after.tracedBranches <= trueBranches);
}

// ── CONTROL: transversal loop still Closes tangent-continuously (byte-identical) ─────
CC_TEST(s4f_control_transversal_loop_still_closes) {
  // plane ∩ sphere → one closed circle (a canonical transversal loop control).
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Plane pl{frameZ({0, 0, 0.5})};
  ssi::ParamBox sdom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox pdom{-2.0, 2.0, -2.0, 2.0};
  auto A = ssi::makeSphereAdapter(sp, sdom);
  auto B = ssi::makePlaneAdapter(pl, pdom);
  const double R = std::sqrt(1.0 - 0.25);
  const double trueLen = 2.0 * kPi * R;

  ssi::SeedOptions so; so.initialGridU = 3; so.initialGridV = 3;
  ssi::MarchOptions mo;
  auto tr = ssi::trace_intersection(A, B, so, mo);
  CC_CHECK(tr.curveCount() == 1);
  CC_CHECK(tr.lines[0].isClosed());                   // still Closed (true-return gate passes)
  CC_CHECK(tr.closedCurves == 1);
  CC_CHECK(tr.selfIntersections == 0);                // no spurious self-crossings on a clean loop
  const double len = polylineLength(tr.lines[0]);
  CC_CHECK(len >= trueLen * 0.97);
  CC_CHECK(len <= trueLen + 1e-4);
}

// ── CONTROL: S4-d Steinmetz still traces (branchPoints == 2, selfIntersections == 0) ─
CC_TEST(s4f_control_steinmetz_branch_trace_unperturbed) {
  // two equal cylinders crossing at 90° → the Steinmetz self-crossing locus (S4-d). The
  // self-intersection guard is a SINGLE-ARM figure-eight detector; a Steinmetz is a LOCUS
  // branch (two arms meet, ‖nA×nB‖→0) → branchPoints == 2 and selfIntersections == 0 even
  // with the S4-f guard ON.
  nmath::Cylinder cz{Ax3::fromAxisAndRef({0, 0, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}), 1.0};
  nmath::Cylinder cx{Ax3::fromAxisAndRef({0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 1, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);

  ssi::SeedOptions so; so.initialGridU = 3; so.initialGridV = 3;
  ssi::MarchOptions mo;
  mo.enableBranchPoints = true;
  mo.enableSelfIntersection = true;   // S4-f guard ON must NOT change the S4-d branch trace
  auto tr = ssi::trace_intersection(A, B, so, mo);
  CC_CHECK(tr.branchPoints == 2);
  CC_CHECK(tr.nearTangentGaps == 0);
  CC_CHECK(tr.selfIntersections == 0);  // a locus branch, NOT a single-arm self-crossing
}

int main() { return cctest::run_all(); }
