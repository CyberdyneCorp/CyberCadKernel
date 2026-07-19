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

// ── REGRESSION: two co-resident loci in ONE adjacency cluster → both seeded ────────
// A general rational-NURBS ∩ B-spline pose lifted VERBATIM from the freeform SSI fuzzer
// (base seed 0x5715d1ff1275, case 17) — a MEASURED HONESTLY-DECLINED multi-branch case:
// OCCT (GeomAPI_IntSS) finds TWO distinct co-resident intersection loci, but the old S2
// seeder handed the marcher only ONE seed (seeded=1, occtComp=2). The two loci's candidate
// leaves land in ONE param-adjacency cluster (dense wavy nets), and the old per-cluster
// FIFO cap (256) was filled by the FIRST locus's thousands of leaves in candidate-iteration
// order, DROPPING the co-resident locus's later leaves before the distinct-locus split ran
// (cap-starvation). The fix keeps the FULL refined-seed density per cluster (single-linkage,
// now spatially hashed → O(m)), so BOTH loci reach the split and each gets a seed.
//
// The bar is HONEST: each of the two seeds must be a real refined crossing — on BOTH
// surfaces ≤ 1e-7 AND transversal (‖n₁×n₂‖ well above zero) — never a fabricated seed. The
// two seeds sit on genuinely distinct loci (x < 0 vs x > 0), matching OCCT's two lines.
CC_TEST(seed_freeform_two_coresident_loci_recovered) {
  // A: rational NURBS (degU=2, degV=3, 5×6), B: B-spline (degU=degV=3, 4×4).
  const std::vector<Point3> polesA = {
      {-1.21128996,-1.21908255,0.145194143}, {-1.1998045,-0.722431005,0.224019689}, {-1.18528066,-0.240523095,0.155081873},
      {-1.20727003,0.256315325,-0.138305492}, {-1.2354309,0.715989591,-0.200995692}, {-1.20477968,1.22381342,-0.17295903},
      {-0.603062904,-1.20405458,0.249045448}, {-0.630416413,-0.736780525,0.292108207}, {-0.590607468,-0.267882158,0.18571183},
      {-0.569124592,0.208057258,-0.153559289}, {-0.623577718,0.743414313,-0.288390673}, {-0.564966836,1.20348526,-0.183951177},
      {-0.0321369714,-1.17667275,0.0223528906}, {0.0241783425,-0.734134854,-0.0267444078}, {0.000435944203,-0.27465435,0.0401421085},
      {-0.0163486114,0.213345658,0.0355759456}, {0.00226608137,0.697959808,-0.035579903}, {0.0011811433,1.1879855,-0.0126770427},
      {0.610210827,-1.19415663,-0.248624759}, {0.586780427,-0.731082198,-0.346625078}, {0.60464737,-0.208876998,-0.088524004},
      {0.605188924,0.21654637,0.0973368863}, {0.560749299,0.684907247,0.297323106}, {0.570826047,1.1866793,0.182843186},
      {1.20178856,-1.18389511,-0.162002595}, {1.21041922,-0.699763741,-0.217308605}, {1.2066239,-0.235290063,-0.11281356},
      {1.20567144,0.257189991,0.116754358}, {1.19057293,0.716091403,0.277167898}, {1.23408035,1.16662419,0.178675021},
  };
  const std::vector<double> wtsA = {
      0.745691173, 2.32800202, 0.650444672, 1.94652488, 1.79007235, 0.949630341, 1.3192024, 0.415428797,
      1.11515914, 1.28193655, 1.32265724, 2.15411339, 0.811730387, 2.48288942, 1.24483187, 0.454868667,
      1.19505552, 2.03923881, 1.02506307, 1.47794648, 1.61358937, 1.09160037, 2.48373184, 1.07685357,
      2.47985123, 0.419704979, 0.578314701, 2.46531896, 1.27958702, 1.25362873};
  const std::vector<double> kUA = {0, 0, 0, 0.333333333, 0.666666667, 1, 1, 1};
  const std::vector<double> kVA = {0, 0, 0, 0, 0.333333333, 0.666666667, 1, 1, 1, 1};
  const int degUA = 2, degVA = 3, nUA = 5, nVA = 6;

  const std::vector<Point3> polesB = {
      {-1.19880228,-1.17163648,-0.0314569395}, {-1.21302274,-0.425076316,0.129553685}, {-1.18057683,0.36861656,0.07855668},
      {-1.22927937,1.22946816,-0.0118351196}, {-0.371965995,-1.18319903,-0.112030629}, {-0.396434246,-0.390625897,-0.037090016},
      {-0.417584575,0.409947941,-0.00188247391}, {-0.388263411,1.20029292,-0.0328895578}, {0.416179816,-1.22583395,-0.132113178},
      {0.41765547,-0.424009998,0.046062877}, {0.430070828,0.437054778,-0.0428521935}, {0.425333701,1.17950658,-0.0343344577},
      {1.20776824,-1.17688741,-0.0328426964}, {1.23649816,-0.381455868,0.0880619564}, {1.16708503,0.432960939,0.0725515852},
      {1.219396,1.19339302,-0.0403895591},
  };
  const std::vector<double> kUB = {0, 0, 0, 0, 1, 1, 1, 1};
  const std::vector<double> kVB = {0, 0, 0, 0, 1, 1, 1, 1};
  const int degUB = 3, degVB = 3, nUB = 4, nVB = 4;

  auto A = ssi::makeNurbsAdapter(degUA, degVA, polesA, wtsA, nUA, nVA, kUA, kVA);
  auto B = ssi::makeBSplineAdapter(degUB, degVB, polesB, nUB, nVB, kUB, kVB);

  // The fuzzer runs S2 with this grid/leaf; keep it identical so the fixture is the pose.
  ssi::SeedOptions o;
  o.initialGridU = 6;
  o.initialGridV = 6;
  o.minPatchFrac = 1.0 / 32.0;
  auto ss = ssi::seed_intersection(A, B, o);

  // BOTH co-resident loci recovered (was 1 before the cap-starvation fix).
  CC_CHECK(ss.branchCount() == 2);
  CC_CHECK(ss.deferredTangent == 0);  // both loci are transversal — no fabricated tangent

  // Each seed is a GENUINE crossing: on both surfaces ≤ 1e-7 and transversal. Never faked.
  nmath::SurfaceGrid gA{polesA, nUA, nVA};
  nmath::SurfaceGrid gB{polesB, nUB, nVB};
  for (const auto& s : ss.seeds) {
    CC_CHECK(s.onSurfResidual < 1e-7);
    CC_CHECK(s.crossingSine > 0.1);
    const Point3 pa = nmath::nurbsSurfacePoint(degUA, degVA, gA, wtsA, kUA, kVA, s.u1, s.v1);
    const Point3 pb = nmath::surfacePoint(degUB, degVB, gB, kUB, kVB, s.u2, s.v2);
    CC_CHECK(nmath::distance(pa, s.point) < 1e-7);  // on surface A (rational)
    CC_CHECK(nmath::distance(pb, s.point) < 1e-7);  // on surface B
  }
  // The two seeds are on genuinely DISTINCT loci (opposite sides in x), matching OCCT's
  // two lines — not one locus over-split (a duplicate would sit near the other).
  if (ss.seeds.size() == 2)
    CC_CHECK(ss.seeds[0].point.x * ss.seeds[1].point.x < 0.0);
}

// FEATURE-ADAPTIVE INITIAL SUBDIVISION — the idx=43 placement-miss recovery (roadmap
// SSI-SUBDIV). The verbatim freeform-fuzzer pose seed=0x5515D1FF0F0F case 43 (family
// multi-branch): a B-spline (A: degU=2 degV=3, 6x5) cap rational NURBS (B: degU=2 degV=3,
// 6x6) that host THREE co-resident transversal loci. OCCT (GeomAPI_IntSS) finds 3 lines;
// the old S2 uniform leaf produced only 2 clustered candidates -> 2 seeds (a placement miss
// UPSTREAM of clustering — the third locus never got its own clustered candidate at the
// coarse leaf). Feature-adaptive subdivision refines the overlapping (feature) leaves one
// level below the uniform leaf, so the third locus gets its own candidate -> cluster -> seed.
// Regression guard: adaptive OFF => 2 seeds (the old miss), adaptive ON (default) => 3 seeds,
// each a GENUINE on-both-surfaces transversal crossing. Recall-only, DISAGREED-safe.
CC_TEST(seed_freeform_adaptive_subdivision_recovers_third_locus) {
  const std::vector<Point3> polesA = {
      {-1.21747171,-1.19581425,0.0404234124},{-1.22394476,-0.568013867,0.177951072},{-1.20212802,-0.00704172766,-0.0128882773},{-1.16455049,0.63913965,-0.190524012},{-1.20907242,1.1678284,-0.100048088},
      {-0.69437808,-1.19491772,0.137331508},{-0.702030995,-0.635562432,0.541271282},{-0.717979475,-0.0212440509,-0.0118285806},{-0.728313142,0.563135225,-0.495581903},{-0.724875778,1.20918738,-0.11223509},
      {-0.226313637,-1.20284829,0.0848490612},{-0.233764958,-0.616441804,0.285612734},{-0.202500166,0.00655340758,-0.0340870289},{-0.263350588,0.591628856,-0.32908772},{-0.214788328,1.19766228,-0.0557698191},
      {0.274251504,-1.19821973,-0.0785242547},{0.260068915,-0.617159084,-0.321295145},{0.240650532,-0.0191590171,0.0267992762},{0.244400034,0.574770134,0.351639096},{0.211181933,1.16561442,0.0910835631},
      {0.688750703,-1.18807586,-0.16322818},{0.757713441,-0.591585346,-0.511398266},{0.706952744,0.0141144332,-0.00309705817},{0.75148157,0.582230194,0.585586112},{0.688743644,1.23776618,0.10515734},
      {1.22394647,-1.18016884,-0.00594362178},{1.23342871,-0.631672772,-0.0840379363},{1.19898838,0.0290960442,0.00356994585},{1.16509161,0.6155148,0.231641848},{1.23742606,1.16555351,0.0804823041},
  };
  const std::vector<double> kUA = {0,0,0,0.25,0.5,0.75,1,1,1};
  const std::vector<double> kVA = {0,0,0,0,0.5,1,1,1,1};
  const int degUA = 2, degVA = 3, nUA = 6, nVA = 5;

  const std::vector<Point3> polesB = {
      {-1.16430761,-1.17937249,-0.0147624625},{-1.22030726,-0.712158903,0.0640965485},{-1.2125237,-0.266536117,0.132870816},{-1.23063822,0.229748186,0.193287329},{-1.21683875,0.692231599,0.101825002},{-1.20760091,1.21819648,0.0265777438},
      {-0.714165717,-1.20907637,-0.0804500009},{-0.734458071,-0.74175132,0.0234783157},{-0.74609727,-0.212796604,0.0420068429},{-0.755138933,0.202537706,0.0106858884},{-0.744190591,0.713637142,-0.0342293719},{-0.735335206,1.20228443,-0.0875726089},
      {-0.228918842,-1.18118996,-0.0987423564},{-0.243417269,-0.733899789,-0.0435023586},{-0.265567658,-0.264311152,-0.046979612},{-0.26001166,0.259697153,0.00503948117},{-0.279533096,0.744570018,-0.0762688304},{-0.27006795,1.18334064,-0.151188119},
      {0.248298554,-1.17187077,-0.1833745},{0.225243237,-0.744096963,-0.0584970012},{0.260204835,-0.238644718,-0.0419381021},{0.202980282,0.277442681,0.0438023268},{0.271240899,0.74509685,-0.0918828181},{0.27682113,1.20901841,-0.143127858},
      {0.69387993,-1.17022034,-0.0511153818},{0.739499136,-0.697536776,-0.0369703049},{0.71707016,-0.241069073,0.0177233554},{0.687268956,0.255884949,0.0113189055},{0.68850246,0.686339473,-0.047925612},{0.7599325,1.1982825,-0.0436081288},
      {1.23774437,-1.2210962,-0.0125380279},{1.17330841,-0.71608189,0.0441821456},{1.21031965,-0.21570609,0.139604201},{1.18024948,0.273877391,0.145518972},{1.1893667,0.751659509,0.135801938},{1.17076798,1.16170791,-0.0456641623},
  };
  const std::vector<double> wtsB = {
      0.752362917,0.43172777,1.5012661,1.1091207,2.37431704,1.68883422,0.894550815,0.756718124,0.737513747,1.64071124,1.2988382,2.41607334,
      1.14013995,2.10231903,1.02414992,0.841387008,1.21218814,1.84099706,1.90525699,2.19559779,1.65359934,1.89193174,1.77326639,1.67906372,
      0.721094879,1.90497118,1.44038209,1.43976424,1.92020345,2.24487957,1.51411009,0.850511283,1.10507314,1.39493758,1.64299105,1.43417264};
  const std::vector<double> kUB = {0,0,0,0.25,0.5,0.75,1,1,1};
  const std::vector<double> kVB = {0,0,0,0,0.333333333,0.666666667,1,1,1,1};
  const int degUB = 2, degVB = 3, nUB = 6, nVB = 6;

  auto A = ssi::makeBSplineAdapter(degUA, degVA, polesA, nUA, nVA, kUA, kVA);
  auto B = ssi::makeNurbsAdapter(degUB, degVB, polesB, wtsB, nUB, nVB, kUB, kVB);

  // Same grid/leaf the fuzzer runs (the pose is that harness config).
  ssi::SeedOptions o;
  o.initialGridU = 6;
  o.initialGridV = 6;
  o.minPatchFrac = 1.0 / 32.0;

  // Adaptive OFF: the OLD placement miss — the uniform leaf produces only 2 clustered
  // candidates, so the third co-resident locus is never seeded.
  o.adaptiveSubdivision = false;
  auto off = ssi::seed_intersection(A, B, o);
  CC_CHECK(off.branchCount() == 2);

  // Adaptive ON (default): the third locus's overlapping leaf is refined one level finer, so
  // it gets its own candidate -> cluster -> seed. All three loci recovered, none fabricated.
  o.adaptiveSubdivision = true;
  auto on = ssi::seed_intersection(A, B, o);
  CC_CHECK(on.branchCount() == 3);
  CC_CHECK(on.deferredTangent == 0);  // all three loci transversal — no fabricated tangent

  // Every seed is a GENUINE crossing: on both surfaces <= 1e-7 and transversal. Never faked.
  nmath::SurfaceGrid gA{polesA, nUA, nVA};
  nmath::SurfaceGrid gB{polesB, nUB, nVB};
  for (const auto& s : on.seeds) {
    CC_CHECK(s.onSurfResidual < 1e-7);
    CC_CHECK(s.crossingSine > 0.1);
    const Point3 pa = nmath::surfacePoint(degUA, degVA, gA, kUA, kVA, s.u1, s.v1);
    const Point3 pb = nmath::nurbsSurfacePoint(degUB, degVB, gB, wtsB, kUB, kVB, s.u2, s.v2);
    CC_CHECK(nmath::distance(pa, s.point) < 1e-7);  // on surface A (B-spline)
    CC_CHECK(nmath::distance(pb, s.point) < 1e-7);  // on surface B (rational)
  }
  // The three seeds are on genuinely DISTINCT loci (spread across x), not one loop over-split.
  if (on.seeds.size() == 3) {
    double xmin = on.seeds[0].point.x, xmax = xmin;
    for (const auto& s : on.seeds) { xmin = std::min(xmin, s.point.x); xmax = std::max(xmax, s.point.x); }
    CC_CHECK(xmax - xmin > 1.0);  // recovered locus sits well apart (x~-1.1 vs x~+1.2)
  }
}

// ── REGRESSION: a COINCIDENT freeform pair must TERMINATE and decline honestly ────────
//
// Handed two coincident (or overlapping, or sub-tolerance-offset) freeform surfaces, the seeder
// used to HANG rather than decline — measured at the shipped adaptive floor, six of six
// constructed pairs produced no output at all within 1200 s, INCLUDING a genuinely DISJOINT pair
// whose correct answer is the empty set. A hang is not an honest decline.
//
// The cause was `clusterRegions`: on a coincident pair the intersection locus is 2-DIMENSIONAL, so
// no leaf pair anywhere is AABB-disjoint and the candidate pile grows ~4× per halving (measured
// 1 835 481 candidates at the shipped floor). Its all-pairs adjacency loop is O(n²), i.e. ~1.7e12
// pair tests there. It is now a spatial-hash union-find over the SAME predicate, computing the
// SAME components. Measured at the shipped floor on this pair: all-pairs produced no result in
// 600 s; the hashed path returns in ~175 s.
//
// WHAT THIS TEST DOES AND DOES NOT GUARD. It pins the OBSERVABLE contract — a coincident pair
// terminates and returns an honest empty result, never a fabricated seed. It runs at a COARSE
// `adaptiveMinFrac` so the suite stays fast (~1 s); at that size the candidate pile is small
// enough that BOTH the old and new clustering are equally quick (measured 827 ms vs 801 ms), so
// this case does NOT by itself catch a reintroduced O(n²) loop. Reproducing that needs the shipped
// 1/256 floor and ~175 s, which is too slow for this suite. The remaining cost at the shipped
// floor is the DESCENT ITSELF producing 1.8 M candidates — a separate, still-open defect (no
// per-node stop once the AABB prune loses its power on a 2D locus).
CC_TEST(seed_coincident_freeform_terminates_and_declines) {
  // A bicubic Bézier dish, against itself and against near/far copies of itself.
  auto dishPoles = [](double dz) {
    std::vector<Point3> poles;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        const double x = -1.0 + 2.0 * i / 3.0, y = -1.0 + 2.0 * j / 3.0;
        poles.push_back({x, y, dz + 0.35 * (x * x + y * y)});
      }
    return poles;
  };
  const std::vector<double> kn = {0, 0, 0, 0, 1, 1, 1, 1};

  // dz = 0     → exactly coincident (shares a 2D region)
  // dz = 1e-9  → sub-tolerance offset (indistinguishable from coincident)
  // dz = 1e-3  → genuinely DISJOINT above tolerance; correct answer is the empty set, and this
  //              pose hung too, so it is the sharper half of the regression.
  for (const double dz : {0.0, 1e-9, 1e-3}) {
    auto A = ssi::makeBSplineAdapter(3, 3, dishPoles(0.0), 4, 4, kn, kn);
    auto B = ssi::makeBSplineAdapter(3, 3, dishPoles(dz), 4, 4, kn, kn);

    ssi::SeedOptions o;
    o.initialGridU = 4;
    o.initialGridV = 4;
    o.minPatchFrac = 1.0 / 16;
    o.adaptiveMinFrac = 1.0 / 16;  // bound the descent so the suite stays fast — see note above

    const ssi::SeedSet ss = ssi::seed_intersection(A, B, o);

    // TERMINATION is the property under test — reaching this line at all is the regression guard.
    // The verdict must be honest: no seed may be fabricated on a shared region or across a gap.
    CC_CHECK(ss.seeds.empty());
    CC_CHECK(ss.branchCount() == 0);
  }
}

// ── REGRESSION: the B-projection must not have a one-sided dead band at an upper bound ───
//
// `projectOntoB` clamps its residual argument into B's domain, and the substrate's numerical
// Jacobian is a FORWARD difference. At an UPPER bound `x+h` clamps straight back to `x`, so that
// Jacobian column is identically zero and the refine cannot move along it — it returns its seed.
// At a LOWER bound the same step points inward and converges. The failure is therefore ONE-SIDED,
// and because the seeding scan includes the bound exactly, any true match within half a grid step
// of the upper edge starts pinned.
//
// Measured directly on a unit-domain plane patch: every target u >= 0.95 came back at u =
// 1.000000 with residual up to 5.00e-02, while u = 0.010 converged to 1.2e-12 — a dead band of
// span/(2*kScan) = 1/16, exactly as predicted.
//
// The OBSERVABLE consequence, asserted here: a delimited overlap loses a collar of that width on
// each upper edge, so the reported region under-covers the true shared area. On this fixture the
// pre-fix region was [0, 2.8125]^2 = 87.9% of the true [0, 3]^2; the lost 0.1875 in A-parameters
// is 1/16 of B's span carried through the 3:1 parameter ratio between the two surfaces.
CC_TEST(seed_overlap_region_covers_shared_area_to_the_upper_edge) {
  // A: a large plane patch. B: a coplanar Bezier patch covering [0,3]^2 of it — so the shared
  // region is INTERIOR to A and its bounds are genuinely delimitable (not a domain-edge run-out,
  // which would honestly report Undecided instead).
  nmath::Plane pl{frameZ({0, 0, 0})};
  auto A = ssi::makePlaneAdapter(pl, ssi::ParamBox{-1.0, 4.0, -1.0, 4.0});
  std::vector<Point3> poles;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) poles.push_back({3.0 * i / 3.0, 3.0 * j / 3.0, 0.0});
  auto B = ssi::makeBezierAdapter(poles, 4, 4);

  const ssi::SeedSet ss = ssi::seed_intersection(A, B, ssi::SeedOptions{});

  CC_CHECK(ss.coincidentRegions.size() == 1);
  if (ss.coincidentRegions.size() != 1) return;
  const ssi::CoincidentRegion& c = ss.coincidentRegions[0];
  CC_CHECK(c.kind == ssi::CoincidenceKind::OverlapSubRegion);  // delimited, not Undecided
  CC_CHECK(c.isCoincident());

  // The true shared A-region is [0,3] x [0,3]. Require the reported region to reach the upper
  // edges: a returned collar is what the dead band produced (87.9% of area), so 99% bites hard.
  const double area = (c.regionA.u1 - c.regionA.u0) * (c.regionA.v1 - c.regionA.v0);
  CC_CHECK(area >= 0.99 * 9.0);
  CC_CHECK(area <= 1.01 * 9.0);          // and it must not OVER-claim either
  CC_CHECK(c.regionA.u1 >= 3.0 - 1e-3);  // the upper edges specifically
  CC_CHECK(c.regionA.v1 >= 3.0 - 1e-3);
}

int main() { return cctest::run_all(); }
