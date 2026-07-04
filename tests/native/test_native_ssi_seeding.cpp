// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for SSI Stage S2 — subdivision seeding (OCCT-FREE, Gate 1 of the
// two-gate model). For known-branch-count native pairs we assert the S2 CONTRACT:
//   (a) every returned seed lies on BOTH surfaces within tolerance (onSurfResidual);
//   (b) ≥ 1 seed per known TRANSVERSAL branch, and dedup collapses to the expected
//       branch count;
//   (c) each seed carries its (u1,v1,u2,v2) that reproduce the seed point on both
//       surfaces;
//   (d) a near-tangent fixture reports `deferredTangent` (an S4 gap) and emits NO
//       fabricated seed — the honest transversal-only scope.
// No OCCT is linked. This suite is compiled only under CYBERCAD_HAS_NUMSCI (the
// least_squares refine), exactly like test_native_numerics.
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

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

// On-surface residual oracles (closed-form, per elementary kind) — reused from the
// S1 test discipline so the seed-on-both-surfaces check is independent of the SSI
// evaluators that produced the seed.
double distToCylinder(const nmath::Cylinder& c, const Point3& x) {
  const Vec3 w = x - c.pos.origin;
  const double axial = nmath::dot(w, c.pos.z.vec());
  const Vec3 radial = w - c.pos.z.vec() * axial;
  return std::fabs(nmath::norm(radial) - c.radius);
}
double distToSphere(const nmath::Sphere& s, const Point3& x) {
  return std::fabs(nmath::distance(x, s.pos.origin) - s.radius);
}
double distToPlane(const nmath::Plane& p, const Point3& x) {
  return std::fabs(nmath::dot(x - p.pos.origin, p.pos.z.vec()));
}

// A clamped uniform knot vector for `nPoles` control points of `degree`
// (multiplicity degree+1 at each end, evenly spaced interior knots). Length
// nPoles+degree+1 — the flat convention the native B-spline evaluator expects.
std::vector<double> clampedUniformKnots(int degree, int nPoles) {
  std::vector<double> k;
  for (int i = 0; i <= degree; ++i) k.push_back(0.0);
  const int interior = nPoles - degree - 1;
  for (int i = 1; i <= interior; ++i) k.push_back(double(i) / (interior + 1));
  for (int i = 0; i <= degree; ++i) k.push_back(1.0);
  return k;
}

// A "wavy" biquadratic B-spline over the XY grid [-3,3]×[-2,2] with TWO isolated
// positive humps (islands of z>0) rising out of a z=-0.3 valley. The plane z=0 cuts
// each hump as one CLOSED LOOP → exactly two disjoint intersection branches; a plane
// at z<-0.3 or z>0.45 (the surface's z-range) never meets it → zero branches. Degree
// 2 keeps each hump local so the two z>0 islands stay disjoint (verified: two
// separated loops in the sign map). `poles` is returned via `outPoles` so the test can
// reproduce a seed point on the surface independently of the adapter.
ssi::SurfaceAdapter makeWavyBSpline(std::vector<Point3>& outPoles,
                                    std::vector<double>& outKU, std::vector<double>& outKV,
                                    int& degU, int& degV, int& nU, int& nV) {
  degU = 2; degV = 2; nU = 7; nV = 5;
  const double base = -0.3;
  std::vector<std::vector<double>> zc(nU, std::vector<double>(nV, base));
  zc[1][2] = 1.2;  // hump A
  zc[5][2] = 1.2;  // hump B
  outPoles.clear();
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j) {
      const double x = -3.0 + 6.0 * i / (nU - 1);
      const double y = -2.0 + 4.0 * j / (nV - 1);
      outPoles.push_back({x, y, zc[i][j]});
    }
  outKU = clampedUniformKnots(degU, nU);
  outKV = clampedUniformKnots(degV, nV);
  return ssi::makeBSplineAdapter(degU, degV, outPoles, nU, nV, outKU, outKV);
}

// Default options with a coarse initial grid (helps separate distinct loops before
// the recursion) at the default 1/32 resolution.
ssi::SeedOptions defaultOpts() {
  ssi::SeedOptions o;
  o.initialGridU = 3;
  o.initialGridV = 3;
  return o;
}

}  // namespace

// ── skew (orthogonal, unequal-radius) cylinders → 2 transversal loops ────────────
// A thin cylinder (axis X, R=0.7) piercing a fat one (axis Z, R=1) enters and exits,
// giving TWO disjoint closed intersection loops (the non-closed-form quartic S1
// defers as NotAnalytic). S2 must seed BOTH.
CC_TEST(seed_skew_cylinders_two_branches) {
  nmath::Cylinder cz{frameZ(), 1.0};                                    // axis Z
  nmath::Cylinder cx{Ax3{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}}, 0.7};  // axis X
  ssi::ParamBox dom{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() == 2);            // one seed per loop after dedup
  CC_CHECK(ss.deferredTangent == 0);          // both crossings are transversal

  for (const auto& s : ss.seeds) {
    // (a) on both surfaces
    CC_CHECK(distToCylinder(cz, s.point) < 1e-7);
    CC_CHECK(distToCylinder(cx, s.point) < 1e-7);
    CC_CHECK(s.onSurfResidual < 1e-7);
    // (b) transversal (normals not parallel)
    CC_CHECK(s.crossingSine > 0.1);
    // (c) params reproduce the point on both surfaces
    CC_CHECK(nmath::distance(cz.value(s.u1, s.v1), s.point) < 1e-6);
    CC_CHECK(nmath::distance(cx.value(s.u2, s.v2), s.point) < 1e-6);
  }
  // The two loops are on opposite sides (x>0 entry, x<0 exit): distinct branches.
  if (ss.seeds.size() == 2)
    CC_CHECK(ss.seeds[0].point.x * ss.seeds[1].point.x < 0.0);
}

// ── two crossing spheres → 1 transversal circle branch ───────────────────────────
// Non-coaxial sphere∩sphere IS analytic at S1, but routed through S2 as a freeform-
// path smoke test it must find the single intersection circle as ONE branch.
CC_TEST(seed_crossing_spheres_one_branch) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({1.0, 0, 0}), 1.0};  // centres 1 apart, R=1 → intersect in a circle
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() == 1);
  CC_CHECK(ss.deferredTangent == 0);
  for (const auto& s : ss.seeds) {
    CC_CHECK(distToSphere(s1, s.point) < 1e-7);
    CC_CHECK(distToSphere(s2, s.point) < 1e-7);
    CC_CHECK(s.crossingSine > 0.1);
  }
}

// ── sphere piercing a freeform (Bézier) bump patch → 1 transversal loop ──────────
// A biquadratic Bézier patch straddling z≈0 (a shallow bump) crossed by a unit
// sphere: their intersection is a single closed loop. Exercises the freeform
// control-net-hull + sampled bound path and the NURBS/Bézier adapter.
CC_TEST(seed_sphere_bezier_bump_one_branch) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(sp, sd);

  std::vector<Point3> poles = {
      {-2, -2, 0.3}, {-2, 0, 0.0}, {-2, 2, 0.3},
      { 0, -2, 0.0}, { 0, 0, -0.5}, { 0, 2, 0.0},
      { 2, -2, 0.3}, { 2, 0, 0.0}, { 2, 2, 0.3}};
  auto B = ssi::makeBezierAdapter(poles, 3, 3);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() == 1);
  CC_CHECK(ss.deferredTangent == 0);
  for (const auto& s : ss.seeds) {
    CC_CHECK(distToSphere(sp, s.point) < 1e-6);
    // point on the Bézier patch: reproduce via its params
    CC_CHECK(nmath::distance(nmath::bezierSurfacePoint(poles, 3, 3, s.u2, s.v2), s.point) < 1e-6);
    CC_CHECK(s.onSurfResidual < 1e-6);
    CC_CHECK(s.crossingSine > 0.1);
  }
}

// ── parallel disjoint planes → 0 branches (pruned, no false seed) ────────────────
CC_TEST(seed_parallel_planes_no_branch) {
  nmath::Plane p1{frameZ()};
  nmath::Plane p2{frameZ({0, 0, 5})};  // 5 apart, never meet
  ssi::ParamBox dom{-3.0, 3.0, -3.0, 3.0};
  auto A = ssi::makePlaneAdapter(p1, dom);
  auto B = ssi::makePlaneAdapter(p2, dom);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() == 0);
  CC_CHECK(ss.deferredTangent == 0);
  CC_CHECK(ss.candidateRegions == 0);  // disjoint AABBs pruned at the root
}

// ── externally tangent spheres → S4 gap (deferredTangent), NOT a fabricated seed ──
// Two unit spheres 2 apart touch at exactly one point; the intersection is a single
// tangency where the normals are ANTIPARALLEL (‖n₁×n₂‖ → 0). S2 is transversal-only:
// it must NOT emit a seed there and must report it as a deferred-to-S4 gap.
CC_TEST(seed_tangent_spheres_deferred_to_s4) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({2.0, 0, 0}), 1.0};  // touch at (1,0,0)
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() == 0);        // no transversal seed emitted
  CC_CHECK(ss.deferredTangent >= 1);      // the tangency is reported as an S4 gap
}

// ── deeper resolution recovers a small loop shallow resolution misses (recall) ────
// A small Bézier bump crossing the sphere near a pole makes a TINY loop. At a coarse
// resolution the subdivision can miss it (recall < 1); at a finer resolution it is
// recovered (recall rises). This asserts the documented recall/resolution trade-off:
// deeper minPatchFrac finds more, and misses are never faked.
CC_TEST(seed_resolution_recovers_small_loop) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(sp, sd);
  // A small tent bump near (0.6,0,~0.8) on the sphere — a little patch that dips
  // through the sphere surface, making one small loop.
  std::vector<Point3> poles = {
      {0.3, -0.3, 0.9}, {0.3, 0.0, 0.9}, {0.3, 0.3, 0.9},
      {0.6, -0.3, 0.9}, {0.6, 0.0, 0.6}, {0.6, 0.3, 0.9},
      {0.9, -0.3, 0.9}, {0.9, 0.0, 0.9}, {0.9, 0.3, 0.9}};
  auto B = ssi::makeBezierAdapter(poles, 3, 3);

  ssi::SeedOptions coarse = defaultOpts();
  coarse.minPatchFrac = 1.0 / 8.0;
  ssi::SeedOptions fine = defaultOpts();
  fine.minPatchFrac = 1.0 / 48.0;

  auto sc = ssi::seed_intersection(A, B, coarse);
  auto sf = ssi::seed_intersection(A, B, fine);
  // Honest: coarse may miss the loop (recall 0 or partial); fine must recover ≥ coarse.
  CC_CHECK(sf.branchCount() >= sc.branchCount());
  CC_CHECK(sf.branchCount() >= 1);  // at the fine resolution the small loop is found
  // Every emitted seed (either resolution) is genuinely on the sphere — never faked.
  for (const auto& s : sf.seeds) CC_CHECK(distToSphere(sp, s.point) < 1e-6);
}

// ── plane crossing a wavy B-spline in 2 separate loops → 2 branches ──────────────
// The plane z=0 slices each of the two z>0 humps as one closed loop. S2 must dedup to
// EXACTLY two branches, each seed on both the plane and the B-spline surface. This is
// the freeform B-spline adapter path (control-net-hull bound) with a known 2-loop
// structure — the explicit "wavy B-spline, 2 loops" fixture the S2 scope calls for.
CC_TEST(seed_plane_wavy_bspline_two_loops) {
  std::vector<Point3> poles;
  std::vector<double> kU, kV;
  int degU, degV, nU, nV;
  auto B = makeWavyBSpline(poles, kU, kV, degU, degV, nU, nV);

  nmath::Plane pl{frameZ({0, 0, 0})};  // z = 0, cuts both humps
  ssi::ParamBox dom{-3.0, 3.0, -2.0, 2.0};
  auto A = ssi::makePlaneAdapter(pl, dom);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() == 2);       // one seed per loop after topological dedup
  CC_CHECK(ss.deferredTangent == 0);     // both loops are transversal
  nmath::SurfaceGrid grid{poles, nU, nV};
  for (const auto& s : ss.seeds) {
    CC_CHECK(distToPlane(pl, s.point) < 1e-7);
    CC_CHECK(s.onSurfResidual < 1e-7);
    CC_CHECK(s.crossingSine > 0.1);      // transversal crossing
    // params reproduce the point on the B-spline surface (surface B is the 2nd operand)
    const Point3 pb = nmath::surfacePoint(degU, degV, grid, kU, kV, s.u2, s.v2);
    CC_CHECK(nmath::distance(pb, s.point) < 1e-6);
  }
  // The two loops sit on opposite humps (x<0 and x>0) — genuinely distinct branches.
  if (ss.seeds.size() == 2)
    CC_CHECK(ss.seeds[0].point.x * ss.seeds[1].point.x < 0.0);
}

// ── a plane BELOW the wavy B-spline → 0 branches (no false seed) ──────────────────
// The surface z-range is [-0.3, 0.45]; a plane at z = -0.5 lies entirely below it, so
// the AABBs never overlap and the seeder emits no seed (and no tangent deferral).
CC_TEST(seed_plane_below_bspline_no_branch) {
  std::vector<Point3> poles;
  std::vector<double> kU, kV;
  int degU, degV, nU, nV;
  auto B = makeWavyBSpline(poles, kU, kV, degU, degV, nU, nV);

  nmath::Plane pl{frameZ({0, 0, -0.5})};  // below the whole surface
  ssi::ParamBox dom{-3.0, 3.0, -2.0, 2.0};
  auto A = ssi::makePlaneAdapter(pl, dom);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() == 0);
  CC_CHECK(ss.deferredTangent == 0);
  CC_CHECK(ss.candidateRegions == 0);  // disjoint AABBs pruned before any refine
}

// ── B-spline crossing another B-spline transversally → seeds on BOTH surfaces ─────
// A gently sloped bilinear B-spline (z = 0.2·x, expressed as a B-spline patch) slices
// through both humps of the wavy B-spline. The intersection has ≥ 1 transversal branch
// per hump it cuts; S2 must find seeds that ALL lie on both freeform surfaces. This is
// the freeform×freeform path (both operands control-net-hull bounded).
CC_TEST(seed_bspline_x_bspline_transversal) {
  std::vector<Point3> polesB;
  std::vector<double> kUB, kVB;
  int dUB, dVB, nUB, nVB;
  auto B = makeWavyBSpline(polesB, kUB, kVB, dUB, dVB, nUB, nVB);

  // A sloped bilinear (degree 1) B-spline over the same XY footprint: z = 0.2·x.
  const int dU = 1, dV = 1, nu = 2, nv = 2;
  std::vector<Point3> polesA = {
      {-3, -2, -0.6}, {-3, 2, -0.6},
      { 3, -2,  0.6}, { 3, 2,  0.6}};
  std::vector<double> kA = clampedUniformKnots(dU, nu);  // {0,0,1,1}
  auto A = ssi::makeBSplineAdapter(dU, dV, polesA, nu, nv, kA, kA);

  auto ss = ssi::seed_intersection(A, B, defaultOpts());
  CC_CHECK(ss.branchCount() >= 1);       // at least one transversal crossing per hump
  CC_CHECK(ss.refinedAccepted >= ss.branchCount());
  nmath::SurfaceGrid gridA{polesA, nu, nv};
  nmath::SurfaceGrid gridB{polesB, nUB, nVB};
  for (const auto& s : ss.seeds) {
    // on surface A (the sloped patch) and surface B (the wavy humps), via their params
    const Point3 pa = nmath::surfacePoint(dU, dV, gridA, kA, kA, s.u1, s.v1);
    const Point3 pb = nmath::surfacePoint(dUB, dVB, gridB, kUB, kVB, s.u2, s.v2);
    CC_CHECK(nmath::distance(pa, s.point) < 1e-6);
    CC_CHECK(nmath::distance(pb, s.point) < 1e-6);
    CC_CHECK(s.onSurfResidual < 1e-6);
    CC_CHECK(s.crossingSine > 0.05);     // transversal (not tangent) at every seed
  }
}

int main() { return cctest::run_all(); }
