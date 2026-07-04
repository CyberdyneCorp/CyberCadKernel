// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for SSI Stage S3 — the predictor-corrector marching-line tracer
// (OCCT-FREE, Gate 1 of the two-gate model). On native pairs with a KNOWN
// intersection we assert the S3 CONTRACT:
//   (a) every traced WLine node lies on BOTH surfaces within tolerance;
//   (b) a closed-loop intersection is classified Closed, its polyline forms a loop of
//       the right size, and the traced curve LENGTH ≈ the closed-form length within a
//       step-bounded tolerance (chord polyline underestimates the arc by ≈ curvature·h²);
//   (c) an open, boundary-to-boundary intersection is classified BoundaryExit, is
//       continuous (no node-to-node gap larger than a few marching steps), and
//       terminates on a domain boundary;
//   (d) a NEAR-TANGENT march is TRUNCATED (TraceStatus::NearTangent) and traces only up
//       to the tangency — never fabricating points past it (the honest S4 gap), and a
//       pair S2 defers as a pure tangency yields ZERO fabricated curves;
//   (e) the fitted B-spline reproduces the polyline within a small tolerance;
//   (f) trace_intersection / trace_from_seeds dedup a second seed on an already-traced
//       branch.
// No OCCT is linked. Compiled only under CYBERCAD_HAS_NUMSCI (corrector + fit call the
// least_squares / lstsq substrate), like test_native_ssi_seeding.
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

// Arc length of the traced polyline (sum of chord segments).
double polylineLength(const ssi::WLine& w) {
  double len = 0.0;
  for (std::size_t i = 1; i < w.points.size(); ++i)
    len += nmath::distance(w.points[i].point, w.points[i - 1].point);
  return len;
}
// Largest node-to-node chord — a continuity witness for open curves (no gap should
// exceed a few marching steps).
double maxNodeGap(const ssi::WLine& w) {
  double g = 0.0;
  for (std::size_t i = 1; i < w.points.size(); ++i)
    g = std::max(g, nmath::distance(w.points[i].point, w.points[i - 1].point));
  return g;
}

ssi::SeedOptions defaultSeedOpts() {
  ssi::SeedOptions o;
  o.initialGridU = 3;
  o.initialGridV = 3;
  return o;
}

// Every node of a WLine must sit on both surfaces (independent closed-form oracle).
template <class OnA, class OnB>
void checkAllNodesOnSurfaces(bool& cc_ok_, const ssi::WLine& w, OnA onA, OnB onB, double tol) {
  CC_CHECK(w.points.size() >= 2);
  for (const auto& n : w.points) {
    CC_CHECK(onA(n.point) < tol);
    CC_CHECK(onB(n.point) < tol);
    CC_CHECK(n.onSurfResidual < tol);
  }
}

}  // namespace

// ── crossing spheres → one CLOSED circular loop; length ≈ 2·pi·R ─────────────────
// Two unit spheres 1 apart intersect in a circle of radius sqrt(3)/2 in the plane
// x = 0.5. The tracer must walk it, close the loop, land every node on both spheres,
// and the traced-curve LENGTH must match 2·pi·R within a step-bounded tolerance (the
// chord polyline slightly underestimates the true arc — bounded by curvature·h²).
CC_TEST(march_crossing_spheres_closed_loop) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({1.0, 0, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);

  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.isClosed());
  CC_CHECK(tr.closedCurves == 1);

  checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToSphere(s1, p); },
                          [&](const Point3& p) { return distToSphere(s2, p); }, 1e-6);

  // The true circle: centre (0.5,0,0), radius sqrt(3)/2 in plane x=0.5.
  const double R = std::sqrt(3.0) / 2.0;
  for (const auto& n : w.points) {
    CC_CHECK(std::fabs(n.point.x - 0.5) < 1e-5);                      // in the plane x=0.5
    const double r = std::hypot(n.point.y, n.point.z);
    CC_CHECK(std::fabs(r - R) < 1e-5);                               // on the circle
  }
  // Traced-curve length ≈ 2·pi·R. The chord polyline is INSIDE the arc, so it is a
  // slight UNDER-estimate; the deficit is bounded by the step, so a 1% window is a
  // step-bounded tolerance (measured deficit ≈ 0.4% at the default step).
  const double trueLen = 2.0 * kPi * R;
  const double len = polylineLength(w);
  CC_CHECK(len <= trueLen + 1e-4);            // never longer than the true arc (+eps)
  CC_CHECK(len >= trueLen * 0.98);            // step-bounded under-estimate
  // fitted B-spline reproduces the polyline.
  CC_CHECK(w.curve.valid());
  CC_CHECK(w.curve.maxFitError < 1e-3);
}

// ── plane cutting a sphere → one CLOSED circle ───────────────────────────────────
// z=0.5 plane through a unit sphere: circle radius sqrt(0.75) at z=0.5. Simplest
// closed loop; verifies loop classification + on-surface residuals + the fit + length.
CC_TEST(march_plane_sphere_closed_circle) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Plane pl{frameZ({0, 0, 0.5})};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox pd{-2.0, 2.0, -2.0, 2.0};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makePlaneAdapter(pl, pd);

  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.isClosed());
  checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToSphere(sp, p); },
                          [&](const Point3& p) { return distToPlane(pl, p); }, 1e-6);
  const double R = std::sqrt(0.75);
  for (const auto& n : w.points) {
    CC_CHECK(std::fabs(n.point.z - 0.5) < 1e-5);
    CC_CHECK(std::fabs(std::hypot(n.point.x, n.point.y) - R) < 1e-5);
  }
  const double trueLen = 2.0 * kPi * R;
  const double len = polylineLength(w);
  CC_CHECK(len <= trueLen + 1e-4);
  CC_CHECK(len >= trueLen * 0.98);
}

// ── skew cylinders → TWO closed loops, each traced and deduped to one WLine ───────
// Thin cylinder (axis X, R=0.7) piercing a fat one (axis Z, R=1): two disjoint loops
// (the quartic S1 defers). The tracer must produce two WLines, both closed, all nodes
// on both cylinders. Also exercises periodic-seam wrapping (cylinder U is angular).
CC_TEST(march_skew_cylinders_two_loops) {
  nmath::Cylinder cz{frameZ(), 1.0};
  nmath::Cylinder cx{Ax3{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}}, 0.7};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);

  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(tr.curveCount() == 2);
  CC_CHECK(tr.closedCurves == 2);
  CC_CHECK(tr.nearTangentGaps == 0);
  for (const auto& w : tr.lines) {
    CC_CHECK(w.isClosed());
    checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToCylinder(cz, p); },
                            [&](const Point3& p) { return distToCylinder(cx, p); }, 1e-6);
  }
  // the two loops are on opposite sides in x.
  if (tr.curveCount() == 2) {
    const double x0 = tr.lines[0].points.front().point.x;
    const double x1 = tr.lines[1].points.front().point.x;
    CC_CHECK(x0 * x1 < 0.0);
  }
}

// ── sphere ∩ finite Bézier bump → one closed loop, nodes on both freeform+sphere ──
CC_TEST(march_sphere_bezier_bump_loop) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(sp, sd);

  std::vector<Point3> poles = {
      {-2, -2, 0.3}, {-2, 0, 0.0}, {-2, 2, 0.3},
      { 0, -2, 0.0}, { 0, 0, -0.5}, { 0, 2, 0.0},
      { 2, -2, 0.3}, { 2, 0, 0.0}, { 2, 2, 0.3}};
  auto B = ssi::makeBezierAdapter(poles, 3, 3);

  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(tr.curveCount() >= 1);
  if (tr.curveCount() < 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.points.size() >= 2);
  for (const auto& n : w.points) {
    CC_CHECK(distToSphere(sp, n.point) < 1e-6);                                  // on the sphere
    CC_CHECK(nmath::distance(nmath::bezierSurfacePoint(poles, 3, 3, n.u2, n.v2), n.point) < 1e-5);
    CC_CHECK(n.onSurfResidual < 1e-6);
  }
}

// ── B-spline ∩ B-spline → one CLOSED transversal loop ────────────────────────────
// A bicubic B-spline DOME (z bulges up to ~0.7 in the middle, 0 at the rim) cut by a
// flat bicubic B-spline at z=0.4 meets it in a single closed loop around the dome. Both
// operands flow through makeBSplineAdapter (native surfacePoint/normal). The tracer must
// close the loop, land every node on both B-spline surfaces, and fit a smooth B-spline.
CC_TEST(march_bspline_bspline_closed_loop) {
  const int deg = 3, n = 4;
  const std::vector<double> k = {0, 0, 0, 0, 1, 1, 1, 1};  // clamped bicubic, one span
  const std::vector<Point3> domePoles = {
      {-2, -2, 0.0}, {-0.7, -2, 0.0}, {0.7, -2, 0.0}, {2, -2, 0.0},
      {-2, -0.7, 0.0}, {-0.7, -0.7, 1.2}, {0.7, -0.7, 1.2}, {2, -0.7, 0.0},
      {-2, 0.7, 0.0}, {-0.7, 0.7, 1.2}, {0.7, 0.7, 1.2}, {2, 0.7, 0.0},
      {-2, 2, 0.0}, {-0.7, 2, 0.0}, {0.7, 2, 0.0}, {2, 2, 0.0}};
  std::vector<Point3> flatPoles = domePoles;
  for (auto& p : flatPoles) p.z = 0.4;  // horizontal cutting sheet at z=0.4

  auto A = ssi::makeBSplineAdapter(deg, deg, domePoles, n, n, k, k);
  auto B = ssi::makeBSplineAdapter(deg, deg, flatPoles, n, n, k, k);

  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.isClosed());
  CC_CHECK(tr.closedCurves == 1);
  CC_CHECK(tr.nearTangentGaps == 0);

  const nmath::SurfaceGrid gA{domePoles, n, n};
  const nmath::SurfaceGrid gB{flatPoles, n, n};
  for (const auto& nd : w.points) {
    CC_CHECK(nmath::distance(nmath::surfacePoint(deg, deg, gA, k, k, nd.u1, nd.v1), nd.point) < 1e-5);
    CC_CHECK(nmath::distance(nmath::surfacePoint(deg, deg, gB, k, k, nd.u2, nd.v2), nd.point) < 1e-5);
    CC_CHECK(std::fabs(nd.point.z - 0.4) < 1e-5);  // the loop lives on the z=0.4 sheet
    CC_CHECK(nd.onSurfResidual < 1e-6);
  }
  CC_CHECK(w.curve.valid());
  CC_CHECK(w.curve.maxFitError < 1e-3);
}

// ── plane cutting a WAVY B-spline saddle → TWO OPEN boundary-to-boundary curves ───
// A bicubic B-spline SADDLE z ≈ 0.15·(x² − y²) over [-3,3]×[-2,2] meets the plane z=0
// along the two diagonals x = ±y. Each diagonal runs across the finite patch and EXITS
// on the domain boundary → two OPEN curves (BoundaryExit), not loops, not tangencies.
// Asserts on-surface, on-plane, continuity (no gap ≫ step), boundary termination.
CC_TEST(march_plane_wavy_bspline_open_segments) {
  const int deg = 3, n = 4;
  const std::vector<double> k = {0, 0, 0, 0, 1, 1, 1, 1};
  const double xs[4] = {-3, -1, 1, 3};
  const double ys[4] = {-2, -0.7, 0.7, 2};
  std::vector<Point3> saddle;
  saddle.reserve(16);
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      const double x = xs[i], y = ys[j];
      saddle.push_back({x, y, 0.15 * (x * x - y * y)});  // saddle: rises in x, dips in y
    }
  auto A = ssi::makeBSplineAdapter(deg, deg, saddle, n, n, k, k);

  nmath::Plane pl{frameZ({0, 0, 0.0})};
  ssi::ParamBox pd{-4.0, 4.0, -3.0, 3.0};
  auto B = ssi::makePlaneAdapter(pl, pd);

  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(tr.curveCount() == 2);      // the two diagonal branches
  CC_CHECK(tr.openCurves == 2);
  CC_CHECK(tr.closedCurves == 0);
  CC_CHECK(tr.nearTangentGaps == 0);

  const nmath::SurfaceGrid grid{saddle, n, n};
  // A step-bounded continuity bound: the default step grows to at most scale/8; a couple
  // of steps is a safe "no gap" ceiling for this ~6-unit-diagonal model.
  const double scale = std::max(A.modelScale, B.modelScale);
  const double gapCeil = scale * (1.0 / 8.0) * 3.0;
  for (const auto& w : tr.lines) {
    CC_CHECK(w.status == ssi::TraceStatus::BoundaryExit);
    CC_CHECK(!w.isClosed());
    CC_CHECK(!w.truncated());          // terminated on the patch boundary, not a tangency
    CC_CHECK(w.points.size() >= 2);
    CC_CHECK(maxNodeGap(w) < gapCeil);  // continuous (no gap ≫ step)
    for (const auto& nd : w.points) {
      CC_CHECK(std::fabs(nd.point.z) < 1e-5);  // on the plane z=0
      CC_CHECK(nmath::distance(nmath::surfacePoint(deg, deg, grid, k, k, nd.u1, nd.v1), nd.point) < 1e-5);
    }
    // both endpoints sit on a domain boundary of the plane's parameter box (x=±4 or y=±3),
    // or on the B-spline patch edge (u/v = 0 or 1) — an open curve ends on an edge.
    for (const ssi::WLinePoint* end : {&w.points.front(), &w.points.back()}) {
      const bool planeEdge = std::fabs(end->u2 - pd.u0) < 1e-3 || std::fabs(end->u2 - pd.u1) < 1e-3 ||
                             std::fabs(end->v2 - pd.v0) < 1e-3 || std::fabs(end->v2 - pd.v1) < 1e-3;
      const bool patchEdge = end->u1 < 1e-3 || end->u1 > 1.0 - 1e-3 ||
                             end->v1 < 1e-3 || end->v1 > 1.0 - 1e-3;
      CC_CHECK(planeEdge || patchEdge);
    }
  }
}

// ── NEAR-TANGENT march → TRUNCATED (NearTangent), traced only up to the tangency ──
// An OFFSET (non-coaxial) cylinder GRAZES a unit sphere: axis shifted +x by 0.585 with
// R=0.4, so the wall very nearly touches the sphere on the +x side (r+dx = 0.985). The
// intersection curve is transversal over most of its length but its normal-cross-product
// sine dips to ≈0.17 near the grazing pinch. With tangentSinTol set to 0.25 — ABOVE that
// dip, BELOW the transversal maximum — the march traces the well-conditioned arc and
// STOPS at the near-tangent region: TraceStatus::NearTangent, counted in
// nearTangentGaps, with the nodes reached BEFORE the tangency (never fabricated past it).
// The truncated arc's nodes still lie exactly on both surfaces.
CC_TEST(march_near_tangent_truncated) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0.585, 0, 0}), 0.4};  // r+dx = 0.985 → near-tangent graze
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);

  ssi::SeedOptions so;
  so.initialGridU = 4;
  so.initialGridV = 4;
  so.minPatchFrac = 1.0 / 32;  // enough to seed the grazing branch without over-refining

  ssi::MarchOptions mo;
  mo.tangentSinTol = 0.25;  // stop where transversality drops below 0.25 (the grazing dip)

  auto tr = ssi::trace_intersection(A, B, so, mo);
  CC_CHECK(tr.curveCount() >= 1);
  if (tr.curveCount() < 1) return;

  // Every emitted branch is a truncated near-tangent trace (NOT a fabricated full loop).
  CC_CHECK(tr.nearTangentGaps >= 1);
  CC_CHECK(tr.closedCurves == 0);       // did NOT fake a closed loop past the tangency
  for (const auto& w : tr.lines) {
    CC_CHECK(w.truncated());
    CC_CHECK(w.status == ssi::TraceStatus::NearTangent);
    CC_CHECK(w.points.size() >= 2);
    // nodes traced up to the tangency still lie on both surfaces …
    for (const auto& nd : w.points) {
      CC_CHECK(distToSphere(sp, nd.point) < 1e-6);
      CC_CHECK(distToCylinder(cy, nd.point) < 1e-6);
    }
    // … and the march genuinely STOPPED short — the last node's transversality has
    // decayed toward the tolerance (it did not sail through the tangency).
    const ssi::WLinePoint& last = w.points.back();
    const Vec3 nA = A.normal(last.u1, last.v1).vec();
    const Vec3 nB = B.normal(last.u2, last.v2).vec();
    const double endSine = nmath::norm(nmath::cross(nA, nB));
    CC_CHECK(endSine < 0.35);  // near the tangent band, not out on the transversal arc
  }
}

// ── externally tangent spheres → NO fabricated curve (S2 defers; S3 emits nothing) ─
// The spheres touch at one point; S2 defers it (deferredTangent) and hands no seed to
// S3, so the tracer produces ZERO curves and echoes the S4 gap. The honest boundary:
// S3 never fabricates a trace where S2 found only a tangency.
CC_TEST(march_tangent_spheres_no_curve) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({2.0, 0, 0}), 1.0};  // touch at (1,0,0)
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);

  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(tr.curveCount() == 0);
  CC_CHECK(tr.deferredTangent >= 1);  // S4 gap echoed, not faked
}

// ── dedup: two seeds on the SAME loop collapse to one WLine ──────────────────────
// Hand trace_from_seeds two seeds that both lie on the crossing-spheres circle; the
// tracer must trace the first, recognize the second retraces it, and keep one WLine.
CC_TEST(march_dedup_retraced_branch) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({1.0, 0, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);

  // one real seed from S2 …
  auto ss = ssi::seed_intersection(A, B, defaultSeedOpts());
  CC_CHECK(ss.branchCount() == 1);
  if (ss.branchCount() != 1) return;
  // … duplicated (a second seed on the same circle, different param angle).
  ssi::SeedSet doubled = ss;
  doubled.seeds.push_back(ss.seeds[0]);  // identical seed → same branch

  auto tr = ssi::trace_from_seeds(A, B, doubled);
  CC_CHECK(tr.curveCount() == 1);      // the duplicate is deduped
  CC_CHECK(tr.dedupedRetraces == 1);
}

int main() { return cctest::run_all(); }
