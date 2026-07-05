// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for SSI Stage S4-e — CHART SINGULARITIES (sphere parametric pole /
// cone apex crossing), OCCT-FREE (Gate 1). A CHART SINGULARITY is where ONE surface's own
// (u,v) parametrization degenerates (‖dU‖ → 0 at a sphere pole v=±π/2 or a cone apex where
// the signed radius crosses zero) while its 3D point + normal stay FINITE — a REMOVABLE
// coordinate singularity, DISTINCT from the S4-c pair graze (‖n₁×n₂‖→0) and the S4-d locus
// self-crossing. The S3 marcher's single-surface advanceParams 2×2 goes rank-deficient there,
// so it either spuriously BoundaryExits (the pole sits on a non-periodic v edge) or step-
// crawls the node budget (the apex). S4-e detects the single-surface Jacobian rank-drop and
// STEPS ACROSS the singular point with the point-based fixed-plane corrector (which needs only
// the finite point + normal), then resumes the normal march.
//
// The fixtures force MARCHING via trace_from_seeds with a hand seed (analytic pairs skip
// marching via S1's closed-form dispatch, so a marched fixture is required). We assert:
//   (a) SPHERE POLE — unit sphere ∩ plane y=0 (great circle through both poles). With the
//       chart switch OFF the S3 marcher truncates to arcLen ≈ π (one meridian, BoundaryExit);
//       with it ON the FULL closed great circle is traced (arcLen ≈ 2π, Closed, both u=0 and
//       u=π meridians, singularitiesCrossed ≥ 2), every node on both surfaces ≤ tol.
//   (b) CONE APEX — double cone (R₀=0, α=45°) ∩ plane y=0 (apex-crossing line). OFF: stalls
//       at v≈0 burning the node budget. ON: traced ACROSS the apex, both nappes v∈[−2,+2] in
//       a BOUNDED node count, singularitiesCrossed ≥ 1, every node ≤ tol.
//   (c) GENUINE BOUNDARY control — finite cylinder ∩ plane runs to a real v-cap edge (NO ‖dU‖
//       collapse): STILL exits BoundaryExit with singularitiesCrossed == 0 (the chart machinery
//       does NOT fire at a true boundary).
//   (d) REGRESSION — the chart switch OFF is byte-identical to S3/S4-c/S4-d; and with the switch
//       ON the S4-c crossable graze STILL crosses (singularitiesCrossed == 0) and the S4-d
//       Steinmetz STILL traces two branch points (singularitiesCrossed == 0). The chart
//       machinery engages ONLY at a detected single-surface chart collapse.
//
// No OCCT is linked. Compiled only when the marching definitions are available (like
// test_native_ssi_marching); the S4-e result fields + MarchOptions knobs are always visible.
//
#include "native/ssi/native_ssi.h"

#include "harness.h"

#include <cmath>

namespace ssi = cybercad::native::ssi;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;
using nmath::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Plane through the origin whose NORMAL is +Y (so it contains the x and z axes): the y=0
// plane. Its (u,v) parametrize (x, z). Frame {origin, X=+x, Y=+z, Z(axis/normal)=+y}.
ssi::SurfaceAdapter planeY0(const ssi::ParamBox& dom) {
  nmath::Plane pl;
  pl.pos = Ax3{Point3{0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 0, 1}, Dir3{0, 1, 0}};
  return ssi::makePlaneAdapter(pl, dom);
}

double distToSphere(const nmath::Sphere& s, const Point3& x) {
  return std::fabs(nmath::distance(x, s.pos.origin) - s.radius);
}
double distToPlaneY0(const Point3& x) { return std::fabs(x.y); }
// Distance to the double cone R₀=0, α=45°, axis +Z: |sqrt(x²+y²) − |z|| (the cone x²+y²=z²).
double distToDoubleCone45(const Point3& x) {
  return std::fabs(std::hypot(x.x, x.y) - std::fabs(x.z));
}
// Distance to the cylinder radius R about the +Z axis at the origin.
double distToUnitCylZ(const Point3& x, double R) { return std::fabs(std::hypot(x.x, x.y) - R); }

double polylineLength(const ssi::WLine& w) {
  double len = 0.0;
  for (std::size_t i = 1; i < w.points.size(); ++i)
    len += nmath::distance(w.points[i].point, w.points[i - 1].point);
  return len;
}

// A one-seed SeedSet on the given params (forces marching, bypassing S1/S2).
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

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// (a) SPHERE POLE — the great circle through both poles is now FULLY traced.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(s4e_sphere_pole_full_great_circle) {
  nmath::Sphere sp{Ax3{}, 1.0};  // unit sphere, identity frame, centre origin
  auto A = ssi::makeSphereAdapter(sp, ssi::ParamBox{0.0, 2.0 * kPi, -kPi / 2, kPi / 2});
  auto B = planeY0(ssi::ParamBox{-2.0, 2.0, -2.0, 2.0});
  // seed on the great circle away from the pole: (1,0,0) at sphere (u=0,v=0), plane (x=1,z=0).
  auto seeds = handSeed(0.0, 0.0, 1.0, 0.0, Point3{1, 0, 0});
  const double tol = 1e-6;

  // BEFORE — chart switch OFF: the S3 marcher truncates at the pole to one meridian (arc ≈ π).
  ssi::MarchOptions off;
  auto before = ssi::trace_from_seeds(A, B, seeds, off);
  CC_CHECK(before.singularitiesCrossed == 0);
  CC_CHECK(before.curveCount() == 1);
  if (before.curveCount() == 1) {
    const double lenBefore = polylineLength(before.lines[0]);
    CC_CHECK(before.lines[0].status == ssi::TraceStatus::BoundaryExit);  // spurious exit at the pole
    CC_CHECK(lenBefore < 0.75 * (2.0 * kPi));   // only HALF the loop (≈ π)
  }

  // AFTER — chart switch ON: the full closed great circle, crossing BOTH poles.
  ssi::MarchOptions on;
  on.enableChartSingularities = true;
  auto tr = ssi::trace_from_seeds(A, B, seeds, on);

  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.isClosed());                       // full closed loop, not a truncated meridian
  CC_CHECK(tr.closedCurves == 1);
  CC_CHECK(tr.nearTangentGaps == 0);            // the poles were crossed, not deferred
  CC_CHECK(tr.singularitiesCrossed >= 2);       // BOTH poles stepped across
  CC_CHECK(w.chartSingularCrossed >= 2);

  // The full great circle: arc length ≈ 2π (a touch short — the fine crossing chord skips the
  // measure-zero arc AT each pole, never fabricating the pole point), both meridians visited.
  const double len = polylineLength(w);
  CC_CHECK(len <= 2.0 * kPi + 1e-4);            // never longer than the true circumference
  CC_CHECK(len >= 0.90 * (2.0 * kPi));          // the FULL loop (not the truncated π half)
  double u1lo = 1e9, u1hi = -1e9;
  for (const auto& n : w.points) {
    // every node on BOTH surfaces (no fabricated pole-crossing point)
    CC_CHECK(distToSphere(sp, n.point) < tol);
    CC_CHECK(distToPlaneY0(n.point) < tol);
    u1lo = std::min(u1lo, n.u1);
    u1hi = std::max(u1hi, n.u1);
  }
  // both the u=0 and the u=π meridians are visited (the loop crosses BOTH poles).
  CC_CHECK(u1lo < 0.1);
  CC_CHECK(u1hi > kPi - 0.1);
}

// ─────────────────────────────────────────────────────────────────────────────
// (b) CONE APEX — the apex-crossing line now spans both nappes.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(s4e_cone_apex_both_nappes) {
  nmath::Cone cone;
  cone.pos = Ax3{};          // apex at the origin, axis +Z
  cone.radius = 0.0;         // R₀ = 0 → a true double cone, apex at v=0
  cone.semiAngle = kPi / 4;  // 45°
  auto A = ssi::makeConeAdapter(cone, ssi::ParamBox{0.0, 2.0 * kPi, -2.0, 2.0});
  auto B = planeY0(ssi::ParamBox{-3.0, 3.0, -3.0, 3.0});
  const double vSeed = 1.8, r = vSeed * std::sin(kPi / 4), z = vSeed * std::cos(kPi / 4);
  auto seeds = handSeed(0.0, vSeed, r, z, Point3{r, 0.0, z});
  const double tol = 1e-6;

  // BEFORE — chart switch OFF: the marcher step-crawls at the apex, never the far nappe.
  ssi::MarchOptions off;
  auto before = ssi::trace_from_seeds(A, B, seeds, off);
  CC_CHECK(before.singularitiesCrossed == 0);
  if (before.curveCount() == 1) {
    double v1lo = 1e9;
    for (const auto& n : before.lines[0].points) v1lo = std::min(v1lo, n.v1);
    CC_CHECK(v1lo > -0.5);  // stalls just short of the apex — never the v<0 nappe
  }

  // AFTER — chart switch ON: crosses the apex, both nappes in a BOUNDED node count.
  ssi::MarchOptions on;
  on.enableChartSingularities = true;
  auto tr = ssi::trace_from_seeds(A, B, seeds, on);

  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(tr.nearTangentGaps == 0);            // the apex was crossed, not deferred
  CC_CHECK(tr.singularitiesCrossed >= 1);       // the apex stepped across
  CC_CHECK(w.chartSingularCrossed >= 1);
  CC_CHECK(static_cast<int>(w.points.size()) < 2000);  // BOUNDED node count (not the 20042 crawl)

  double v1lo = 1e9, v1hi = -1e9;
  for (const auto& n : w.points) {
    CC_CHECK(distToDoubleCone45(n.point) < tol);
    CC_CHECK(distToPlaneY0(n.point) < tol);
    v1lo = std::min(v1lo, n.v1);
    v1hi = std::max(v1hi, n.v1);
  }
  // BOTH nappes: v runs from ≈ +2 through the apex (v=0) to ≈ −2.
  CC_CHECK(v1lo < -1.5);
  CC_CHECK(v1hi > 1.5);
}

// ─────────────────────────────────────────────────────────────────────────────
// (c) GENUINE BOUNDARY control — a finite cylinder cap v-edge (no chart collapse) STILL exits.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(s4e_cylinder_cap_boundary_still_exits) {
  nmath::Cylinder cyl{Ax3{}, 1.0};  // unit cylinder, axis +Z
  // finite v cap at ±1 (a REAL domain boundary — ‖dU‖ = R stays finite there, no collapse).
  auto A = ssi::makeCylinderAdapter(cyl, ssi::ParamBox{0.0, 2.0 * kPi, -1.0, 1.0});
  auto B = planeY0(ssi::ParamBox{-2.0, 2.0, -2.0, 2.0});
  // plane y=0 ∩ cylinder → the two vertical lines x=±1. Seed on x=+1 at z=0 → (1,0,0).
  auto seeds = handSeed(0.0, 0.0, 1.0, 0.0, Point3{1, 0, 0});
  const double tol = 1e-6;

  ssi::MarchOptions on;
  on.enableChartSingularities = true;  // switch ON — must STILL exit the genuine boundary
  auto tr = ssi::trace_from_seeds(A, B, seeds, on);

  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.status == ssi::TraceStatus::BoundaryExit);  // a real v-cap boundary, not a pole
  CC_CHECK(tr.singularitiesCrossed == 0);                // the chart machinery did NOT fire
  CC_CHECK(w.chartSingularCrossed == 0);
  for (const auto& n : w.points) {
    CC_CHECK(distToUnitCylZ(n.point, 1.0) < tol);
    CC_CHECK(distToPlaneY0(n.point) < tol);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// (d) REGRESSION — the S4-c graze STILL crosses with the chart switch ON (singularitiesCrossed
// == 0): the chart machinery is disjoint from the S4-c pair-graze seam.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(s4e_switch_on_does_not_perturb_s4c_graze) {
  nmath::Sphere sp{Ax3{}, 1.0};
  nmath::Cylinder cy{Ax3{Point3{0.585, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}}, 0.4};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);
  ssi::SeedOptions so;
  so.initialGridU = 4; so.initialGridV = 4; so.minPatchFrac = 1.0 / 32;

  ssi::MarchOptions mo;
  mo.tangentSinTol = 0.25;               // ABOVE the graze dip → S4-c must march THROUGH
  mo.enableChartSingularities = true;    // chart switch ON must NOT change the S4-c crossing
  auto tr = ssi::trace_intersection(A, B, so, mo);

  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;
  CC_CHECK(tr.nearTangentGaps == 0);
  CC_CHECK(tr.nearTangentCrossed >= 1);  // the graze STILL crosses (S4-c)
  CC_CHECK(tr.singularitiesCrossed == 0);// and it is NOT mis-attributed to a chart singularity
  CC_CHECK(tr.lines[0].isClosed());
}

// ─────────────────────────────────────────────────────────────────────────────
// (d) REGRESSION — the S4-d Steinmetz STILL traces two branch points with the chart switch ON
// (singularitiesCrossed == 0): the chart machinery is disjoint from the S4-d locus-flip seam.
// ─────────────────────────────────────────────────────────────────────────────
CC_TEST(s4e_switch_on_does_not_perturb_s4d_steinmetz) {
  nmath::Cylinder cz{Ax3::fromAxisAndRef({0, 0, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}), 1.0};
  nmath::Cylinder cx{Ax3::fromAxisAndRef({0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 1, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);

  ssi::SeedOptions so;
  so.initialGridU = 3; so.initialGridV = 3;
  ssi::MarchOptions mo;
  mo.enableBranchPoints = true;
  mo.enableChartSingularities = true;    // chart switch ON must NOT change the S4-d branch trace
  auto tr = ssi::trace_intersection(A, B, so, mo);

  CC_CHECK(tr.branchPoints == 2);
  CC_CHECK(tr.nearTangentGaps == 0);
  CC_CHECK(tr.singularitiesCrossed == 0);// no chart singularity here — a locus self-crossing
}

int main() { return cctest::run_all(); }
