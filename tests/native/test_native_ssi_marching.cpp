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

Vec3 unitVec(Vec3 v) { return v * (1.0 / nmath::norm(v)); }

// Build an orthonormal Ax3 from an origin and a (not-necessarily-unit) main axis.
// The reference X is chosen non-parallel to the axis, then Gram-Schmidt orthonormalised.
Ax3 axFromZ(Point3 o, Vec3 z) {
  z = unitVec(z);
  const Vec3 ref = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  const Vec3 x = unitVec(nmath::cross(ref, z));
  const Vec3 y = nmath::cross(z, x);
  return Ax3{o, Dir3{x.x, x.y, x.z}, Dir3{y.x, y.y, y.z}, Dir3{z.x, z.y, z.z}};
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
// Perpendicular distance from a point to a cone surface: the radial gap between the
// point's actual radius at its axial height and the cone's radius there, scaled by
// cos(semiAngle) to give the true (perpendicular) surface distance.
double distToCone(const nmath::Cone& c, const Point3& x) {
  const Vec3 w = x - c.pos.origin;
  const double axial = nmath::dot(w, c.pos.z.vec());
  const double rad = nmath::norm(w - c.pos.z.vec() * axial);
  const double rAxis = c.radius + axial / std::cos(c.semiAngle) * std::sin(c.semiAngle);
  return std::fabs(rad - rAxis) * std::cos(c.semiAngle);
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

// ── S4-c: NEAR-TANGENT TRANSVERSAL graze → MARCHED THROUGH (full curve, not truncated) ─
// An OFFSET (non-coaxial) cylinder GRAZES a unit sphere: axis shifted +x by 0.585 with
// R=0.4, so the wall very nearly touches the sphere on the +x side (r+dx = 0.985). The
// intersection is a SINGLE closed loop whose normal-cross-product sine dips to ≈0.10 near
// the grazing pinches but the curve GENUINELY CONTINUES through them (transversally). With
// tangentSinTol = 0.25 — ABOVE the dip — the S3 marcher STOPS (this used to be the honest
// `march_near_tangent_truncated` gap). S4-c recognizes the graze as NearTangentTransversal
// and MARCHES THROUGH it with the fixed-plane-cut corrector, producing the FULL loop:
// nearTangentGaps == 0, nearTangentCrossed ≥ 1, every node still exactly on both surfaces,
// and the traced curve matches the one the marcher gets when the tolerance is loosened
// below the dip (the honest ground truth — the crossing did not fabricate a new shape).
CC_TEST(march_near_tangent_crossed_s4c) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0.585, 0, 0}), 0.4};  // r+dx = 0.985 → near-tangent graze
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);

  ssi::SeedOptions so;
  so.initialGridU = 4;
  so.initialGridV = 4;
  so.minPatchFrac = 1.0 / 32;

  // Ground truth: with the tolerance BELOW the dip the S3 marcher already crosses the
  // graze transversally and closes the loop. That is the shape S4-c must reproduce.
  ssi::MarchOptions ctrl;
  ctrl.tangentSinTol = 1e-3;
  auto ref = ssi::trace_intersection(A, B, so, ctrl);
  CC_CHECK(ref.closedCurves == 1);
  CC_CHECK(ref.nearTangentGaps == 0);
  double refLen = 0.0;
  if (ref.curveCount() >= 1) refLen = polylineLength(ref.lines[0]);

  // S4-c: tolerance ABOVE the dip — S3 would truncate; S4-c must march THROUGH.
  ssi::MarchOptions mo;
  mo.tangentSinTol = 0.25;
  auto tr = ssi::trace_intersection(A, B, so, mo);
  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;

  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(tr.nearTangentGaps == 0);          // the graze was crossed, not truncated
  CC_CHECK(tr.nearTangentCrossed >= 1);       // and it is REPORTED as a crossing
  CC_CHECK(w.nearTangentCrossed >= 1);
  CC_CHECK(w.isClosed());                      // full closed loop, not an open truncation
  CC_CHECK(!w.truncated());
  CC_CHECK(w.points.size() >= 2);
  // Every node — including the ones spliced across the graze — lies on BOTH surfaces
  // within the corrector tolerance (never a fabricated point off the geometry).
  for (const auto& nd : w.points) {
    CC_CHECK(distToSphere(sp, nd.point) < 1e-6);
    CC_CHECK(distToCylinder(cy, nd.point) < 1e-6);
  }
  CC_CHECK(w.crossMaxResidual < 1e-6);         // the crossed arc stayed on both surfaces
  // The crossed loop reproduces the ground-truth loop: same closed shape, arc length
  // within a step-bounded window (the crossing takes larger chords through the graze so
  // its polyline underestimates the arc a touch more — a few percent, never longer).
  const double len = polylineLength(w);
  if (refLen > 0.0) {
    CC_CHECK(len <= refLen + 1e-4);            // never longer than the ground-truth arc
    CC_CHECK(len >= refLen * 0.90);            // within a step-bounded under-estimate
  }
}

// ── GENUINE tangency the MARCHER REACHES must STILL STOP (not be crossed) ──────────
// A cylinder (axis Z, R=1) meets a sphere (R=1) centred on that axis at (0,0,0): they are
// tangent ALONG the whole equator circle z=0 (a TangentCurve — the surfaces coincide to
// first order along a 1-D locus, not a transversal crossing). Unlike the sphere/graze
// above, the curve does NOT continue transversally through it. If the seeder hands the
// marcher a near-tangent seed here, S4-c's crossable gate must classify it (TangentCurve /
// not NearTangentTransversal) and REFUSE to cross: no fabricated loop, nearTangentCrossed
// stays 0. Any emitted curve must be an honest NearTangent truncation, never a full loop
// stitched across the tangency.
CC_TEST(march_tangent_curve_not_crossed_s4c) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0, 0, 0}), 1.0};  // coaxial: tangent along the equator z=0
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);

  ssi::SeedOptions so;
  so.initialGridU = 4;
  so.initialGridV = 4;

  ssi::MarchOptions mo;
  mo.tangentSinTol = 0.25;
  auto tr = ssi::trace_intersection(A, B, so, mo);

  // The honesty invariant: NO node is fabricated across the tangency. Either the pair is
  // deferred with no curve, or any emitted curve is a NearTangent truncation — but NEVER
  // a crossing (nearTangentCrossed == 0) and NEVER a fabricated closed loop.
  CC_CHECK(tr.nearTangentCrossed == 0);
  for (const auto& w : tr.lines) {
    CC_CHECK(w.nearTangentCrossed == 0);
    CC_CHECK(!w.isClosed());  // no full loop stitched across the tangent-curve contact
    if (w.stopReason)
      CC_CHECK(w.stopReason->type != ssi::TangentContactType::NearTangentTransversal);
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

// ─────────────────────────────────────────────────────────────────────────────
// S4-d — BRANCH POINTS / self-crossing loci.
// ─────────────────────────────────────────────────────────────────────────────

// STEINMETZ bicylinder: two equal R=1 cylinders, axes Z and X crossing orthogonally at
// the origin. Their intersection is TWO ellipses (planes x=z and x=−z) that CROSS at two
// branch points (0,±1,0). S3+S4-c DEFER there (one NearTangent WLine, a gap). With
// enableBranchPoints the tracer must LOCALIZE both branch points and ROUTE the arms so the
// FULL multi-arm intersection is traced: two branch points at (0,±1,0), all arcs on both
// cylinders, no unresolved near-tangent gaps, no fabricated points.
CC_TEST(march_steinmetz_branch_points_s4d) {
  nmath::Cylinder cz{Ax3::fromAxisAndRef({0, 0, 0}, Dir3{0, 0, 1}, Dir3{1, 0, 0}), 1.0};  // axis Z
  nmath::Cylinder cx{Ax3::fromAxisAndRef({0, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 1, 0}), 1.0};  // axis X
  ssi::ParamBox dom{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);
  const double tol = 1e-5;

  // BEFORE — default (branch points OFF): the marcher defers at the saddle (an S4 gap).
  auto before = ssi::trace_intersection(A, B, defaultSeedOpts());
  CC_CHECK(before.branchPoints == 0);
  CC_CHECK(before.nearTangentGaps >= 1);       // honest defer, no branch handling

  // AFTER — branch points ON: both branch points localized, arms routed, gaps resolved.
  ssi::MarchOptions mo;
  mo.enableBranchPoints = true;
  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts(), mo);

  CC_CHECK(tr.branchPoints == 2);              // the two saddles (0,±1,0)
  CC_CHECK(tr.nearTangentGaps == 0);           // every arc resolved to a branch junction
  CC_CHECK(tr.tracedBranches >= 4);            // 4 arcs (2 per ellipse) assembled
  CC_CHECK(tr.routedArms >= 1);                // at least one arm routed off a branch point

  // Each branch point sits at (0,±1,0) on BOTH cylinders, with a collapsed sine.
  bool sawPlus = false, sawMinus = false;
  for (const auto& bn : tr.branchNodes) {
    CC_CHECK(distToCylinder(cz, bn.point) < tol);
    CC_CHECK(distToCylinder(cx, bn.point) < tol);
    CC_CHECK(bn.branchSine < 1e-3);            // transversality collapsed → 0 at B
    CC_CHECK(bn.armLineIds.size() >= 2);       // a self-crossing: ≥2 arcs meet here
    if (nmath::distance(bn.point, Point3{0, 1, 0}) < 1e-3) sawPlus = true;
    if (nmath::distance(bn.point, Point3{0, -1, 0}) < 1e-3) sawMinus = true;
  }
  CC_CHECK(sawPlus && sawMinus);

  // NO fabricated points: every node of every arc lies on BOTH cylinders.
  for (const auto& w : tr.lines)
    for (const auto& n : w.points) {
      CC_CHECK(distToCylinder(cz, n.point) < tol);
      CC_CHECK(distToCylinder(cx, n.point) < tol);
    }
}

// ISOLATED TANGENT POINT (control): two R=1 spheres at centre distance d = R1+R2 = 2 touch
// at (1,0,0). The relative second form there is SIGN-DEFINITE ⇒ NO real arm directions ⇒
// the S4-d enumerator returns no arms. Even with branch points ENABLED the curve must STILL
// END — zero branch points, zero routed arms, no fabricated arc (S2 already defers it).
CC_TEST(march_tangent_point_never_branches_s4d) {
  nmath::Sphere s1{frameZ({0, 0, 0}), 1.0};
  nmath::Sphere s2{frameZ({2.0, 0, 0}), 1.0};  // touch at (1,0,0)
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);

  ssi::MarchOptions mo;
  mo.enableBranchPoints = true;
  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts(), mo);
  CC_CHECK(tr.branchPoints == 0);              // an isolated tangent point is NOT a branch
  CC_CHECK(tr.routedArms == 0);                // no arms sprouted
  CC_CHECK(tr.curveCount() == 0);              // curve ends (no fabricated arc)
  CC_CHECK(tr.deferredTangent >= 1);           // S4 gap echoed, not faked
}

// ── S4-d GENERAL/FREEFORM OPEN-ARM branch point — a bicubic B-spline SADDLE tangent to a
// plane through its saddle point → an X-shaped self-crossing of the intersection LOCUS.
//
// The B-spline saddle z ≈ 0.15·(x²−y²) is a general (non-quadric, non-Steinmetz) FREEFORM
// patch; its actual saddle point sits at the patch centre (z ≈ 0.2449, ABOVE the z=0 plane of
// march_plane_wavy_bspline_open_segments — where the two hyperbola branches are DISJOINT). A
// plane placed THROUGH the saddle point makes the z=const level set DEGENERATE to the crossing:
// the intersection self-crosses at one branch point with FOUR arms (the two diagonals). The
// transversality sine collapses to 0 there — a genuine tangency.
//
// BEFORE (branch points OFF): the S3/S4-c marcher DEFERS at the saddle — ONE honest S4 gap, no
// branch structure. AFTER (branch points ON): the branch LOCALIZES on the freeform pair (the
// tangent-cone Δ>0 yields four arms), each arm routes branch-to-BOUNDARY on the finite patch,
// and reclassifyBranchArcs recognises the OPEN-ARM topology (one end on the localized branch,
// the other a clean domain boundary) → four resolved BranchArcs, ZERO residual near-tangent
// gaps. Every node stays on BOTH surfaces (never a fabricated point past the degeneracy).
CC_TEST(march_freeform_saddle_branch_open_arms_s4d) {
  const int deg = 3, n = 4;
  const std::vector<double> k = {0, 0, 0, 0, 1, 1, 1, 1};
  const double xs[4] = {-3, -1, 1, 3};
  const double ys[4] = {-2, -0.7, 0.7, 2};
  std::vector<Point3> saddle;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      saddle.push_back({xs[i], ys[j], 0.15 * (xs[i] * xs[i] - ys[j] * ys[j])});
  auto A = ssi::makeBSplineAdapter(deg, deg, saddle, n, n, k, k);
  const nmath::SurfaceGrid grid{saddle, n, n};

  // The B-spline's saddle point is the surface value at the patch centre (symmetric net ⇒
  // centred at (0,0)); the plane THROUGH it forces the tangency.
  const Point3 saddlePt = A.point(0.5, 0.5);
  nmath::Plane pl{frameZ({0, 0, saddlePt.z})};
  ssi::ParamBox pd{-4.0, 4.0, -3.0, 3.0};
  auto B = ssi::makePlaneAdapter(pl, pd);
  const double tol = 1e-5;

  // BEFORE — branch points OFF: DEFER at the saddle (one honest S4 gap, no branch, X truncated).
  ssi::MarchOptions off;
  auto before = ssi::trace_intersection(A, B, defaultSeedOpts(), off);
  CC_CHECK(before.branchPoints == 0);
  CC_CHECK(before.nearTangentGaps == 1);
  CC_CHECK(before.tracedBranches == 0);

  // AFTER — branch points ON: one localized freeform branch, four resolved arms, no gaps.
  ssi::MarchOptions on;
  on.enableBranchPoints = true;
  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts(), on);

  CC_CHECK(tr.branchPoints == 1);       // ONE localized freeform branch point
  CC_CHECK(tr.nearTangentGaps == 0);    // NO residual gap — every arm resolved
  CC_CHECK(tr.tracedBranches == 4);     // the four arms of the X-crossing
  CC_CHECK(tr.openCurves == 4);
  CC_CHECK(tr.curveCount() == 4);

  CC_CHECK(tr.branchNodes.size() == 1);
  if (tr.branchNodes.size() == 1) {
    const ssi::BranchNode& node = tr.branchNodes[0];
    CC_CHECK(node.onSurfResidual < tol);                       // B is on BOTH surfaces
    CC_CHECK(node.branchSine < 1e-3);                          // the transversality collapsed at B
    CC_CHECK(nmath::distance(node.point, saddlePt) < 1e-2);    // B is the saddle point
  }

  // Every arm is a resolved BranchArc: on BOTH surfaces, ONE end on the branch, the other a
  // domain boundary (an OPEN arm — never fabricated past the branch).
  const double mergeR = 1e-3 * std::max(A.modelScale, B.modelScale);
  auto atBranch = [&](const Point3& p) {
    for (const ssi::BranchNode& nd : tr.branchNodes)
      if (nmath::distance(nd.point, p) <= mergeR) return true;
    return false;
  };
  int armsMeetingBranch = 0;
  for (const ssi::WLine& w : tr.lines) {
    CC_CHECK(w.status == ssi::TraceStatus::BranchArc);
    CC_CHECK(w.points.size() >= 2);
    CC_CHECK(w.onSurfResidual < tol);
    for (const ssi::WLinePoint& nd : w.points) {
      CC_CHECK(std::fabs(nd.point.z - saddlePt.z) < tol);      // on the plane
      CC_CHECK(nmath::distance(nmath::surfacePoint(deg, deg, grid, k, k, nd.u1, nd.v1),
                               nd.point) < tol);               // on the B-spline
    }
    const bool frB = atBranch(w.points.front().point);
    const bool bkB = atBranch(w.points.back().point);
    CC_CHECK(frB != bkB);   // EXACTLY one end on the branch (open arm), the other a boundary
    if (frB || bkB) ++armsMeetingBranch;
  }
  CC_CHECK(armsMeetingBranch == 4);
}

// ── S4-d honesty control — a DEFINITE freeform contact NEVER sprouts arms. A bicubic B-spline
// BOWL z = 0.15·(x²+y²) tangent to a plane through its minimum touches at a single point; the
// relative second fundamental form is sign-DEFINITE (tangent-cone Δ≤0), so enumerateArms returns
// NO arms and the curve ENDS. With branch points ON the marcher STILL fabricates nothing —
// distinguishing the transversal saddle self-crossing (arms) from an isolated tangent contact.
CC_TEST(march_freeform_bump_definite_never_branches_s4d) {
  const int deg = 3, n = 4;
  const std::vector<double> k = {0, 0, 0, 0, 1, 1, 1, 1};
  const double xs[4] = {-3, -1, 1, 3};
  const double ys[4] = {-2, -0.7, 0.7, 2};
  std::vector<Point3> bump;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j)
      bump.push_back({xs[i], ys[j], 0.15 * (xs[i] * xs[i] + ys[j] * ys[j])});
  auto A = ssi::makeBSplineAdapter(deg, deg, bump, n, n, k, k);
  const Point3 lo = A.point(0.5, 0.5);       // the bowl minimum
  nmath::Plane pl{frameZ({0, 0, lo.z})};
  auto B = ssi::makePlaneAdapter(pl, ssi::ParamBox{-4.0, 4.0, -3.0, 3.0});

  ssi::MarchOptions on;
  on.enableBranchPoints = true;
  auto tr = ssi::trace_intersection(A, B, defaultSeedOpts(), on);
  CC_CHECK(tr.branchPoints == 0);        // definite contact is NOT a branch
  CC_CHECK(tr.routedArms == 0);          // no arms sprouted
  CC_CHECK(tr.curveCount() == 0);        // the curve ends — no fabricated arc
  CC_CHECK(tr.deferredTangent >= 1);     // honest S4 gap echoed, not faked
}

// ── M1b BREADTH: general non-coaxial / skew analytic quadric poses ───────────────
// The S1 closed-form dispatch defers all of these as NotAnalytic (no elementary closed
// form). The S2 seeder + S3 marcher trace them anyway. These host cases lock the S3
// CONTRACT self-consistently (every node on BOTH surfaces, clean closure, no near-tangent
// gap); Gate B (native_ssi_marching_parity.mm) verifies the SAME poses vs OCCT
// GeomAPI_IntSS. Poses are shared with the sim cases so the two gates agree.
//
// The two families here each pair against a FINITE operand whose intersection is a SINGLE
// loop regardless of the other operand's infinite extent (skew cyl∩cyl bounded by the
// smaller cylinder's single penetration; off-axis sphere∩cone bounded by the finite sphere),
// so the finite native trace matches the (infinite-quadric) OCCT locus on the sim. cone∩cone,
// cyl∩cone off-axis and off-axis sphere∩cyl (the infinite cylinder pierces twice → 2 loops +
// seeding-recall) are the DECLINED tail — see the oracle-setup note in
// native_ssi_marching_parity.mm.

constexpr double kSin60 = 0.86602540378443864676;  // sin 60°
constexpr double kCos60 = 0.5;                      // cos 60°

// GENERAL SKEW cyl∩cyl — axes neither parallel nor intersecting (gap 0.4 along +y) AND
// oblique (60° tilt). The small cylinder fully penetrates the big one in a single
// crossing region → ONE connected quartic loop (unlike the symmetric orthogonal-
// intersecting pose, which is two disjoint loops).
CC_TEST(march_skew_cylinders_general_single_loop) {
  nmath::Cylinder cz{frameZ(), 1.0};                                        // axis +Z, R=1
  nmath::Cylinder cx{axFromZ({0, 0.4, 0}, Vec3{kSin60, 0, kCos60}), 0.7};   // skew, tilt 60°
  ssi::ParamBox dom{0.0, 2.0 * kPi, -3.0, 3.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);
  ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6;

  auto tr = ssi::trace_intersection(A, B, sopt);
  CC_CHECK(tr.curveCount() == 1);
  CC_CHECK(tr.closedCurves == 1);
  CC_CHECK(tr.nearTangentGaps == 0);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.isClosed());
  checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToCylinder(cz, p); },
                          [&](const Point3& p) { return distToCylinder(cx, p); }, 1e-9);
}


// OFF-AXIS sphere∩cone — cone axis offset from the sphere centre. One loop.
CC_TEST(march_sphere_cone_offaxis) {
  nmath::Sphere sp{frameZ(), 1.0};
  nmath::Cone co{axFromZ({0.3, 0, -2}, unitVec(Vec3{0.1, 0, 1})), 0.05, 0.3};
  ssi::ParamBox ds{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox dk{0.0, 2.0 * kPi, 0.0, 5.0};
  auto A = ssi::makeSphereAdapter(sp, ds);
  auto B = ssi::makeConeAdapter(co, dk);
  ssi::SeedOptions sopt; sopt.initialGridU = 8; sopt.initialGridV = 8; sopt.minPatchFrac = 1.0 / 48.0;

  auto tr = ssi::trace_intersection(A, B, sopt);
  CC_CHECK(tr.curveCount() == 1);
  CC_CHECK(tr.closedCurves == 1);
  CC_CHECK(tr.nearTangentGaps == 0);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.isClosed());
  checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToSphere(sp, p); },
                          [&](const Point3& p) { return distToCone(co, p); }, 1e-9);
}

// ── M1c BREADTH: the M1b-declined tail promoted to verified ──────────────────────
// M1b honestly declined general cone∩cone, off-axis cyl∩cone, and off-axis sphere∩cyl:
// the OCCT oracle surfaces are INFINITE while the native adapters are FINITE patches, so an
// unbounded quadric piercing the other operand more than once yielded a multi-loop OCCT locus
// the finite native trace could not match, and the second loop was a seeding-recall miss.
//
// M1c lands them with FINITE, self-consistent poses (Gate A here) that the sim harness verifies
// against a DOMAIN-CLIPPED oracle (Geom_RectangularTrimmedSurface to the native patch bounds).
// These host cases lock the S3 CONTRACT self-consistently (every node on BOTH surfaces ≤ 1e-9,
// clean closure / boundary exit, no near-tangent gap). The twice-piercing sphere∩cyl uses the
// M1c SEEDING-RECALL BUMP (completenessCritic + criticTargetedReseed) to seed the SECOND loop.

// GENERAL cone∩cone — two finite cones with OFFSET apexes and tilted axes, meeting in ONE
// closed loop inside both finite patches (the far nappe / far cone body never reaches the
// bounded height, so it is a single loop the finite native trace matches an equally-clipped
// oracle on).
CC_TEST(march_cone_cone_general_single_loop) {
  nmath::Cone c1{frameZ({0, 0, -1.0}), 0.05, 0.4};
  nmath::Cone c2{axFromZ({0.35, 0, 0.9}, unitVec(Vec3{0.05, 0, -1})), 0.05, 0.4};
  ssi::ParamBox d1{0.0, 2.0 * kPi, 0.0, 2.5};
  ssi::ParamBox d2{0.0, 2.0 * kPi, 0.0, 2.5};
  auto A = ssi::makeConeAdapter(c1, d1);
  auto B = ssi::makeConeAdapter(c2, d2);
  ssi::SeedOptions sopt; sopt.initialGridU = 8; sopt.initialGridV = 8; sopt.minPatchFrac = 1.0 / 48.0;

  auto tr = ssi::trace_intersection(A, B, sopt);
  CC_CHECK(tr.curveCount() == 1);
  CC_CHECK(tr.closedCurves == 1);
  CC_CHECK(tr.nearTangentGaps == 0);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.isClosed());
  checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToCone(c1, p); },
                          [&](const Point3& p) { return distToCone(c2, p); }, 1e-9);
}

// OFF-AXIS cyl∩cone — the cone axis offset + tilted from the cylinder axis; the intersection
// arc RUNS OFF the finite patch boundaries → ONE open (BoundaryExit) arc. The whole arc lies
// on both finite surfaces; the sim verifies it against a domain-clipped oracle.
CC_TEST(march_cyl_cone_offaxis_open_arc) {
  nmath::Cylinder cy{frameZ(), 0.6};
  nmath::Cone co{axFromZ({0.2, 0, -0.3}, unitVec(Vec3{0.12, 0, 1})), 0.02, 0.45};
  ssi::ParamBox dcy{0.0, 2.0 * kPi, -1.2, 1.2};
  ssi::ParamBox dco{0.0, 2.0 * kPi, 0.0, 2.5};
  auto A = ssi::makeCylinderAdapter(cy, dcy);
  auto B = ssi::makeConeAdapter(co, dco);
  ssi::SeedOptions sopt; sopt.initialGridU = 8; sopt.initialGridV = 8; sopt.minPatchFrac = 1.0 / 48.0;

  auto tr = ssi::trace_intersection(A, B, sopt);
  CC_CHECK(tr.curveCount() == 1);
  CC_CHECK(tr.tracedBranches == 1);
  CC_CHECK(tr.nearTangentGaps == 0);
  if (tr.curveCount() != 1) return;
  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(!w.isClosed());  // exits the finite patch boundary
  checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToCylinder(cy, p); },
                          [&](const Point3& p) { return distToCone(co, p); }, 1e-9);
}

// OFF-AXIS sphere∩cyl (TWICE-PIERCING) — a thin cylinder offset from the sphere centre pierces
// the sphere on BOTH sides → TWO disjoint closed loops. At practical seed densities the coarse
// subdivision merges the two loops into ONE topological cluster (one representative seed) so the
// FIXED-resolution trace finds only ONE loop (the M1b decline). The M1c SEEDING-RECALL BUMP —
// completenessCritic + criticTargetedReseed — re-seeds the uncovered param cells and recovers the
// SECOND loop, so BOTH are traced. Default-off flags → no other case changes.
CC_TEST(march_sphere_cyl_twice_piercing_two_loops) {
  nmath::Sphere sp{frameZ(), 1.0};
  nmath::Cylinder cy{axFromZ({0.55, 0, 0}, unitVec(Vec3{0.05, 0, 1})), 0.3};
  ssi::ParamBox ds{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox dcy{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeSphereAdapter(sp, ds);
  auto B = ssi::makeCylinderAdapter(cy, dcy);

  // FIXED-resolution baseline: the coarse grid merges the two loops → only ONE traced.
  ssi::SeedOptions base; base.initialGridU = 6; base.initialGridV = 6; base.minPatchFrac = 1.0 / 32.0;
  auto trBase = ssi::trace_intersection(A, B, base);
  CC_CHECK(trBase.closedCurves == 1);  // the honest recall miss the bump fixes

  // M1c SEEDING-RECALL BUMP: recover the second loop with the targeted critic re-seed.
  ssi::SeedOptions sopt = base;
  sopt.completenessCritic = true;
  sopt.criticTargetedReseed = true;
  sopt.criticRefineFactor = 0.6;
  ssi::MarchOptions mopt; mopt.maxDeflection = A.modelScale * 2.5e-4;
  auto tr = ssi::trace_intersection(A, B, sopt, mopt);
  CC_CHECK(tr.curveCount() == 2);
  CC_CHECK(tr.closedCurves == 2);
  CC_CHECK(tr.nearTangentGaps == 0);
  CC_CHECK(tr.criticRecoveredLoops >= 1);
  for (const ssi::WLine& w : tr.lines) {
    CC_CHECK(w.isClosed());
    checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToSphere(sp, p); },
                            [&](const Point3& p) { return distToCylinder(cy, p); }, 1e-9);
  }
}

// ── S4-c DEEP near-tangent breadth (M1d): adaptive re-anchoring crosses a TIGHTER graze ──
// The same offset cyl∩sphere family as march_near_tangent_crossed_s4c, but pushed DEEPER
// into the near-tangent regime: dx = 0.590 (r+dx = 0.990) so the transversality sine dips to
// ≈ 0.141 at the pinches — BELOW the ≈ 0.17 floor where the shipped fixed-t★ crossing corrector
// still converges. There the frozen-plane corrector fails to land (the curve turns materially
// through the pinch, slicing the fixed plane far from the guess) → the DEFAULT S4-c HONESTLY
// DEFERS (nearTangentGaps == 1, no fabricated curve). With `adaptiveCrossReanchor` the crossing
// re-anchors its advance plane to the LOCAL curve tangent and traverses the graze, producing the
// FULL closed loop — every node still on BOTH surfaces ≤ 1e-9, matching the tolerance-below-dip
// ground truth. This is the measured breadth extension (floor ≈ 0.17 → ≈ 0.14).
CC_TEST(march_deep_near_tangent_reanchor_crossed_s4c) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0.590, 0, 0}), 0.4};  // r+dx = 0.990 → deeper graze, minSine ≈ 0.141
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);

  ssi::SeedOptions so;
  so.initialGridU = 6;
  so.initialGridV = 6;
  so.minPatchFrac = 1.0 / 64;

  // Ground truth: with the tolerance BELOW the dip the S3 marcher closes the loop.
  ssi::MarchOptions ctrl;
  ctrl.tangentSinTol = 1e-4;
  auto ref = ssi::trace_intersection(A, B, so, ctrl);
  CC_CHECK(ref.closedCurves == 1);
  double refLen = 0.0;
  if (ref.curveCount() >= 1) refLen = polylineLength(ref.lines[0]);

  // DEFAULT S4-c (frozen t★, reanchor OFF): the deeper graze is below the fixed-plane floor →
  // HONEST DEFER. No fabricated curve, no crossing.
  ssi::MarchOptions off;
  off.tangentSinTol = 0.25;
  auto trOff = ssi::trace_intersection(A, B, so, off);
  CC_CHECK(trOff.nearTangentGaps == 1);
  CC_CHECK(trOff.nearTangentCrossed == 0);
  CC_CHECK(trOff.closedCurves == 0);

  // M1d DEEP breadth: adaptive re-anchoring crosses the tighter graze → FULL closed loop.
  ssi::MarchOptions on;
  on.tangentSinTol = 0.25;
  on.adaptiveCrossReanchor = true;
  on.reanchorBlend = 0.5;
  auto tr = ssi::trace_intersection(A, B, so, on);
  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;

  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(tr.nearTangentGaps == 0);       // the deeper graze was crossed, not truncated
  CC_CHECK(tr.nearTangentCrossed >= 1);    // and it is REPORTED as a crossing
  CC_CHECK(w.nearTangentCrossed >= 1);
  CC_CHECK(w.isClosed());                  // full closed loop, not an open truncation
  CC_CHECK(!w.truncated());
  CC_CHECK(w.points.size() >= 2);
  // Every node — including those spliced across the tighter graze — lies on BOTH surfaces
  // (never a fabricated point off the geometry).
  for (const auto& nd : w.points) {
    CC_CHECK(distToSphere(sp, nd.point) < 1e-9);
    CC_CHECK(distToCylinder(cy, nd.point) < 1e-9);
  }
  // Reproduces the ground-truth loop: same closed shape, arc length within a step-bounded
  // under-estimate (chord polyline takes larger chords through the graze).
  const double len = polylineLength(w);
  if (refLen > 0.0) {
    CC_CHECK(len <= refLen + 1e-4);        // never longer than the ground-truth arc
    CC_CHECK(len >= refLen * 0.88);        // within a step-bounded under-estimate
  }
}

// ── HONEST DECLINE below the extended floor (M1d) ─────────────────────────────────
// Pushed FURTHER still: dx = 0.595 (r+dx = 0.995), transversality sine dips to ≈ 0.100 — below
// even the ADAPTIVE-re-anchoring floor. The graze is now so wide (a large fraction of the loop
// is near-tangent) that the curve-following crossing cannot recover to a transversal stretch
// within budget. The honest contract: EVEN WITH adaptiveCrossReanchor ON, this defers — NO
// fabricated curve, no crossing across the knife-edge. A ground-truth loop still exists (traced
// only with the tolerance below the dip), so this is a genuine HONEST DECLINE at the sharpened
// floor, not a missing loop.
CC_TEST(march_deep_near_tangent_reanchor_honest_decline_s4c) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0.595, 0, 0}), 0.4};  // r+dx = 0.995 → minSine ≈ 0.100, below the extended floor
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);

  ssi::SeedOptions so;
  so.initialGridU = 6;
  so.initialGridV = 6;
  so.minPatchFrac = 1.0 / 64;

  // A ground-truth loop DOES exist (tolerance below the dip) — so a decline is honest, not a miss.
  ssi::MarchOptions ctrl;
  ctrl.tangentSinTol = 1e-4;
  auto ref = ssi::trace_intersection(A, B, so, ctrl);
  CC_CHECK(ref.closedCurves == 1);

  // Adaptive re-anchoring ON — still HONESTLY DECLINES below the extended floor: no crossing,
  // no fabricated closed loop stitched across the knife-edge.
  ssi::MarchOptions on;
  on.tangentSinTol = 0.25;
  on.adaptiveCrossReanchor = true;
  on.reanchorBlend = 0.5;
  auto tr = ssi::trace_intersection(A, B, so, on);
  CC_CHECK(tr.nearTangentCrossed == 0);   // never crossed the knife-edge
  for (const ssi::WLine& w : tr.lines) {
    CC_CHECK(w.nearTangentCrossed == 0);
    CC_CHECK(!w.isClosed());              // no full loop fabricated across the near-tangency
  }
  CC_CHECK(tr.nearTangentGaps >= 1);       // the honest S4 gap is reported (deferred → OCCT)
}

// ── CONTRACT: a shared 2D locus is distinguishable from "no intersection" ─────────────
//
// A coincident pair shares a REGION rather than meeting in a curve, so it yields no WLines —
// exactly like a pair that misses entirely. The S2 seeder already detects, types and suppresses
// seeds inside such a region, but `trace_intersection` DISCARDED that verdict, leaving the two
// cases field-for-field identical at the S3 contract. A consumer deciding whether a face survives
// a boolean therefore could not tell a shared face from a clear one.
//
// This pins the distinction on three pairs that all differ in exactly that respect.
CC_TEST(trace_reports_coincident_shared_locus) {
  ssi::ParamBox sq{-1, 1, -1, 1};

  // (1) COINCIDENT — two coplanar planes share a 2D region. No curve, but a reported locus.
  //
  // The verdict here is `Undecided`, NOT a confirmed `FullSurfaceSame`: the agreement runs all the
  // way to the domain edge, so the detector cannot delimit the shared region and honestly declines
  // to claim it — `isCoincident()` is deliberately false for `Undecided`. That is the correct
  // answer today and is asserted as "a verdict exists and it is not None", so that tightening the
  // edge-delimitation later (which would promote this to `OverlapSubRegion`) does not spuriously
  // fail this test.
  {
    nmath::Plane p1{frameZ({0, 0, 0})}, p2{frameZ({0, 0, 0})};
    auto A = ssi::makePlaneAdapter(p1, sq);
    auto B = ssi::makePlaneAdapter(p2, sq);
    const ssi::TraceSet ts = ssi::trace_intersection(A, B, ssi::SeedOptions{}, ssi::MarchOptions{});
    CC_CHECK(ts.curveCount() == 0);            // a 2D locus yields no 1D branch
    CC_CHECK(ts.hasCoincidenceVerdict());      // ...but the pair is NOT reported as clear
    CC_CHECK(!ts.coincidentRegions.empty());
    for (const auto& c : ts.coincidentRegions)
      CC_CHECK(c.kind != ssi::CoincidenceKind::None);
  }

  // (2) NO INTERSECTION — parallel planes far apart. Also no curve, and NOT coincident. Before
  // the verdict was propagated this TraceSet was identical to (1).
  {
    nmath::Plane p1{frameZ({0, 0, 0})}, p2{frameZ({0, 0, 5})};
    auto A = ssi::makePlaneAdapter(p1, sq);
    auto B = ssi::makePlaneAdapter(p2, sq);
    const ssi::TraceSet ts = ssi::trace_intersection(A, B, ssi::SeedOptions{}, ssi::MarchOptions{});
    CC_CHECK(ts.curveCount() == 0);
    CC_CHECK(!ts.hasCoincidenceVerdict());        // the discriminator that did not exist before
    CC_CHECK(ts.coincidentRegions.empty());
  }

  // (3) TRANSVERSAL control — a genuine 1D curve must never be reported as a shared region.
  {
    nmath::Sphere s1{frameZ({0, 0, 0}), 1.0}, s2{frameZ({1.0, 0, 0}), 1.0};
    ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
    auto A = ssi::makeSphereAdapter(s1, dom);
    auto B = ssi::makeSphereAdapter(s2, dom);
    const ssi::TraceSet ts = ssi::trace_intersection(A, B, ssi::SeedOptions{}, ssi::MarchOptions{});
    CC_CHECK(ts.curveCount() == 1);
    CC_CHECK(!ts.hasCoincidenceVerdict());
  }
}

// ── M1f REGRESSION: the densify refit must never reach interpolation ──────────────────
//
// The refit pole target was `min(m, kDensifyMaxPoles)`, which for any loop with m ≤ 200 resolved
// to m itself. At nPoles == m the least-squares system is SQUARE and interpolating, and the
// clamped-uniform knot vector over a chord-length parametrization degenerates: the curve rides
// every node exactly while oscillating wildly BETWEEN them. The at-node error metric cannot see
// this by construction — it is sampled at precisely the parameters the fit interpolates. On this
// 195-node graze loop the blown-up fit reported maxFitError 3.6e-06 while its true deviation was
// 4.99e-01, five orders of magnitude worse than the 64-pole fit it replaced.
//
// This pins the invariant DIRECTLY and OCCT-free: the sphere ∩ cylinder locus is analytically
// parametrizable, so the fitted curve's distance to the TRUE locus is checked at node MIDPOINTS —
// between the nodes, where the oscillation lives and where the shipped metric is blind.
CC_TEST(march_densify_refit_never_interpolates_s4c) {
  const double dx = 0.597;
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({dx, 0, 0}), 0.4};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);
  ssi::SeedOptions so;
  so.initialGridU = 6;
  so.initialGridV = 6;
  so.minPatchFrac = 1.0 / 64;

  ssi::MarchOptions on;
  on.tangentSinTol = 0.25;
  on.adaptiveCrossReanchor = true;
  on.reanchorBlend = 0.5;
  on.reanchorIncrementalOrientation = true;
  auto tr = ssi::trace_intersection(A, B, so, on);
  CC_CHECK(tr.curveCount() == 1);
  if (tr.curveCount() != 1) return;

  const ssi::WLine& w = tr.lines[0];
  CC_CHECK(w.curve.valid());
  if (!w.curve.valid()) return;

  // The refit MUST have fired (the whole point of the tightened trigger) yet MUST NOT have
  // reached the node count — the pole target is a fraction of m, never m itself.
  const int m = static_cast<int>(w.points.size());
  const int nPoles = static_cast<int>(w.curve.poles.size());
  CC_CHECK(nPoles > 64);   // the tightened trigger fired
  CC_CHECK(nPoles < m);    // and stopped short of interpolation

  // Sample the fitted curve BETWEEN nodes and require it to stay on BOTH surfaces there. A fit
  // that interpolates the nodes while bowing between them fails here and passes the at-node
  // metric — which is exactly the defect this guards.
  const auto& c = w.curve;
  const double t0 = c.knots.front(), t1 = c.knots.back();
  double worstMid = 0.0;
  for (int i = 0; i < 4000; ++i) {
    const double t = t0 + (t1 - t0) * ((i + 0.5) / 4000.0);
    const Point3 p = nmath::curvePoint(c.degree, c.poles, c.knots, t);
    worstMid = std::max(worstMid, std::max(distToSphere(sp, p), distToCylinder(cy, p)));
  }
  CC_CHECK(worstMid < 5e-4);  // the parity gate's own on-curve budget, asserted OCCT-free
}

// ── M1e WIDE-BAND: incremental orientation crosses the 90°-turn reversal trap ─────────
//
// REGRESSION for a measured DEFECT, not a breadth extension. The M1d re-anchor block resolved
// the local tangent's SIGN and its ADOPTION GATE against the FROZEN t★. Both are half-spaces of
// one stale vector and both go degenerate at the SAME point — when the curve's tangent has
// turned 90° from t★. Past it the sign test flipped the true FORWARD tangent to BACKWARD: the
// step retreated, one step back the turn was inside 90° again, and the march stepped forward —
// a self-sustaining 2-cycle that burned all 256 nodes without traversing (arc 3.86 for net
// transport 0.21). The decline was reported honestly, so nothing was ever fabricated; but it was
// a TRAP, not a limit. `reanchorIncrementalOrientation` re-references BOTH tests to the previous
// accepted step direction (monotone across any accumulated turn), keeping t★ as the honesty
// anchor where it belongs (band-min floor, steep-collapse, ≥60° branch-flip).
//
// The old "floor ≈ 0.14 minSine" was therefore an ARTEFACT of the trap, and minSine was never the
// governing quantity: the shipped boundary is the locus where the turn across the band reaches
// 90°, predicted analytically at dx★ = 0.592787 and measured at dx = 0.5926 crosses / 0.5927
// declines. With the trap removed the limiter becomes `minCrossSine = 0.075` — the DESIGNED
// honesty tolerance — reached at gate C with ZERO crossing nodes emitted.
CC_TEST(march_wide_band_incremental_orientation_s4c) {
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  ssi::SeedOptions so;
  so.initialGridU = 6;
  so.initialGridV = 6;
  so.minPatchFrac = 1.0 / 64;

  // (1) CROSSABLE wide-band poses: the M1d re-anchor alone declines (trapped); incremental
  // orientation traverses each to a FULL closed loop. minSine 0.118 / 0.100 / 0.077.
  for (const double dx : {0.593, 0.595, 0.597}) {
    nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
    nmath::Cylinder cy{frameZ({dx, 0, 0}), 0.4};
    auto A = ssi::makeSphereAdapter(sp, sd);
    auto B = ssi::makeCylinderAdapter(cy, cd);

    // A ground-truth loop exists (tolerance below the dip) — so the M1d decline is a real miss.
    ssi::MarchOptions ctrl;
    ctrl.tangentSinTol = 1e-4;
    auto ref = ssi::trace_intersection(A, B, so, ctrl);
    CC_CHECK(ref.closedCurves == 1);
    const double refLen = ref.curveCount() >= 1 ? polylineLength(ref.lines[0]) : 0.0;

    // M1d re-anchor WITHOUT incremental orientation: trapped → honest decline (the defect).
    ssi::MarchOptions off;
    off.tangentSinTol = 0.25;
    off.adaptiveCrossReanchor = true;
    off.reanchorBlend = 0.5;
    auto trOff = ssi::trace_intersection(A, B, so, off);
    CC_CHECK(trOff.nearTangentCrossed == 0);
    CC_CHECK(trOff.closedCurves == 0);
    CC_CHECK(trOff.nearTangentGaps >= 1);

    // M1e incremental orientation: crosses to ONE closed loop.
    ssi::MarchOptions on = off;
    on.reanchorIncrementalOrientation = true;
    auto tr = ssi::trace_intersection(A, B, so, on);
    CC_CHECK(tr.curveCount() == 1);
    if (tr.curveCount() != 1) continue;

    const ssi::WLine& w = tr.lines[0];
    CC_CHECK(tr.nearTangentGaps == 0);
    CC_CHECK(tr.nearTangentCrossed >= 1);
    CC_CHECK(w.isClosed());
    CC_CHECK(!w.truncated());
    // Every node — including every one spliced across the graze — on BOTH surfaces at the
    // SAME onSurfTol the rest of the march uses. No tolerance is relaxed to buy the crossing.
    checkAllNodesOnSurfaces(cc_ok_, w, [&](const Point3& p) { return distToSphere(sp, p); },
                            [&](const Point3& p) { return distToCylinder(cy, p); }, 1e-9);
    const double len = polylineLength(w);
    CC_CHECK(len <= refLen + 1e-4);   // never longer than the ground-truth arc
    CC_CHECK(len >= refLen * 0.88);   // step-bounded under-estimate
  }

  // (2) HONEST DECLINE below the new floor. bandMin dips under minCrossSine = 0.075, so the
  // crossing refuses at the band-minimum gate — with ZERO crossing nodes emitted. That last
  // assertion is what separates a PRINCIPLED refusal from a burned-budget one: the M1d trap
  // also "declined" at these poses, but only after emitting and discarding 256 orbit nodes.
  for (const double dx : {0.5975, 0.598}) {
    nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
    nmath::Cylinder cy{frameZ({dx, 0, 0}), 0.4};
    auto A = ssi::makeSphereAdapter(sp, sd);
    auto B = ssi::makeCylinderAdapter(cy, cd);

    ssi::MarchOptions ctrl;
    ctrl.tangentSinTol = 1e-4;
    auto ref = ssi::trace_intersection(A, B, so, ctrl);
    CC_CHECK(ref.closedCurves == 1);  // a loop DOES exist → the decline is honest, not a miss

    ssi::MarchOptions on;
    on.tangentSinTol = 0.25;
    on.adaptiveCrossReanchor = true;
    on.reanchorBlend = 0.5;
    on.reanchorIncrementalOrientation = true;
    auto tr = ssi::trace_intersection(A, B, so, on);
    CC_CHECK(tr.nearTangentCrossed == 0);
    CC_CHECK(tr.nearTangentGaps >= 1);
    for (const ssi::WLine& w : tr.lines) {
      CC_CHECK(w.nearTangentCrossed == 0);
      CC_CHECK(!w.isClosed());  // nothing stitched across the knife-edge
    }
  }

  // (3) TRUE TANGENCY still defers with the flag on — the honesty anchor is intact.
  {
    nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
    nmath::Cylinder cy{frameZ({0.600, 0, 0}), 0.4};  // r + dx = 1.0 → tangent
    auto A = ssi::makeSphereAdapter(sp, sd);
    auto B = ssi::makeCylinderAdapter(cy, cd);
    ssi::MarchOptions on;
    on.tangentSinTol = 0.25;
    on.adaptiveCrossReanchor = true;
    on.reanchorBlend = 0.5;
    on.reanchorIncrementalOrientation = true;
    auto tr = ssi::trace_intersection(A, B, so, on);
    CC_CHECK(tr.nearTangentCrossed == 0);
    for (const ssi::WLine& w : tr.lines) CC_CHECK(!w.isClosed());
  }
}

// ── M1e FLAG-OFF BYTE-IDENTITY on a pose the new path changes ─────────────────────────
// The option is default-OFF, so dx = 0.595 must still produce the SHIPPED M1d decline. This is
// the guard that keeps `march_deep_near_tangent_reanchor_honest_decline_s4c` meaningful: the
// wide-band crossing is opt-in, and no caller gets new behaviour by upgrading.
CC_TEST(march_wide_band_flag_off_preserves_m1d_decline_s4c) {
  nmath::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nmath::Cylinder cy{frameZ({0.595, 0, 0}), 0.4};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  ssi::ParamBox cd{0.0, 2.0 * kPi, -1.5, 1.5};
  auto A = ssi::makeSphereAdapter(sp, sd);
  auto B = ssi::makeCylinderAdapter(cy, cd);
  ssi::SeedOptions so;
  so.initialGridU = 6;
  so.initialGridV = 6;
  so.minPatchFrac = 1.0 / 64;

  ssi::MarchOptions dflt;  // reanchorIncrementalOrientation defaults false
  dflt.tangentSinTol = 0.25;
  dflt.adaptiveCrossReanchor = true;
  dflt.reanchorBlend = 0.5;
  CC_CHECK(dflt.reanchorIncrementalOrientation == false);
  auto tr = ssi::trace_intersection(A, B, so, dflt);
  CC_CHECK(tr.nearTangentCrossed == 0);
  CC_CHECK(tr.closedCurves == 0);
  CC_CHECK(tr.nearTangentGaps >= 1);

  // And the option is INERT without adaptiveCrossReanchor: it is AND-ed into the tuned flag, so
  // setting it alone can never alter the frozen-t★ path.
  ssi::MarchOptions bare;
  bare.tangentSinTol = 0.25;
  bare.reanchorIncrementalOrientation = true;  // no adaptiveCrossReanchor
  auto trBare = ssi::trace_intersection(A, B, so, bare);
  ssi::MarchOptions plain;
  plain.tangentSinTol = 0.25;
  auto trPlain = ssi::trace_intersection(A, B, so, plain);
  CC_CHECK(trBare.nearTangentCrossed == trPlain.nearTangentCrossed);
  CC_CHECK(trBare.closedCurves == trPlain.closedCurves);
  CC_CHECK(trBare.nearTangentGaps == trPlain.nearTangentGaps);
  CC_CHECK(trBare.curveCount() == trPlain.curveCount());
}

// ── REGRESSION: DENSIFY-AND-REFIT the fitted curve on a DENSE high-curvature loop ──────
// A general rational-NURBS ∩ B-spline pose lifted VERBATIM from the freeform SSI fuzzer
// (base seed 0x5615d1ff10c2, case 32) — a MEASURED HONESTLY-DECLINED small-loop case whose
// decline was a FIT-DENSITY artifact: the tight high-curvature loop is traced as ~1068
// on-both-surfaces nodes (every node on the true locus), but the least-squares B-spline fit
// at the default 64-pole cap could not follow the loop's curvature and BOWED off the on-locus
// polyline by ~5e-3 (the fitted curve, not the polyline, is what downstream coverage samples,
// so the bow read over the 1e-3 coverage budget). The densify-and-refit follow-up refits at a
// higher (bounded) pole count when the fit under-resolves the nodes, so the convenience curve
// rides the on-locus polyline. This is a pure FIT-QUALITY fix — NO tolerance is widened; more
// poles only pull the fitted curve CLOSER to the already-on-locus nodes, never move a node.
//
// The bar is HONEST: (1) the loop is genuinely dense + high-curvature (>1000 nodes); (2) every
// node is on BOTH surfaces ≤ 1e-6 (the ground truth is untouched); (3) the densify FIRED (the
// fit uses MORE than the 64-pole cap); (4) the resulting fitted curve, densely sampled, rides
// the on-locus node polyline within a tight bound WELL under the 1e-3 coverage budget (the bow
// the fuzzer measured is gone). Nothing here widens the onCurve/onSurf gate.
CC_TEST(march_densify_refit_high_curvature_loop) {
  // A: B-spline (degU=3, degV=2, 5×4).
  const int degAU = 3, degAV = 2, nAU = 5, nAV = 4;
  const std::vector<Point3> polesA = {{-1.2041762101217437,-1.2184680937050816,1.1043575424220369},{-1.185537638405987,-0.38014494483466554,0.5957822103568271},{-1.18010740187799,0.40304862591832458,0.59794289275867052},{-1.2001853758554639,1.2089550877587136,1.0944192470068874},{-0.61320940137210833,-1.201090273857891,0.67198908710055316},{-0.60296573235115314,-0.39444758167468769,0.19119740485794337},{-0.61525612582606692,0.39579177559647449,0.19643837033673009},{-0.58858284348748191,1.2003734839502829,0.67493057194280515},{-0.015532481442894795,-1.1833990631272882,0.53108097147545963},{-0.011052065516317114,-0.38139500394422737,0.058767269953345844},{-0.018003253448602232,0.38474315085196908,0.06436551179334897},{-0.01130027201167509,1.2039779642645638,0.540755360748576},{0.5976479734312472,-1.2171575196981712,0.69321802217522011},{0.59498581996768596,-0.39054156493423148,0.1871687044241386},{0.59888020552337073,0.3963706399983673,0.20689111550327938},{0.61981768926792535,1.198430899857287,0.689873216250111},{1.1922486078242567,-1.1806082058746046,1.0504128184096606},{1.1905270831263075,-0.41408047029921796,0.5900715752523864},{1.2081841627304695,0.41862559163904162,0.61370067394359673},{1.2107898429362014,1.2173372087080006,1.1225925973954927}};
  const std::vector<double> kUA = {0,0,0,0,0.5,1,1,1,1};
  const std::vector<double> kVA = {0,0,0,0.5,1,1,1};

  // B: rational NURBS (degU=3, degV=2, 4×5).
  const int degBU = 3, degBV = 2, nBU = 4, nBV = 5;
  const std::vector<Point3> polesB = {{-1.2108843539576502,-1.1825459159635003,1.2353183121429034},{-1.1926297791063274,-0.58601691140773338,0.74317885705966813},{-1.1818818462240854,-0.0040725523736649637,0.55601817223970373},{-1.2148370812043281,0.59069839119198542,0.77182601447384158},{-1.196637527432781,1.1805643100226988,1.233436852101073},{-0.39382855788641807,-1.2033137081981056,0.66277433667156493},{-0.41633919903378852,-0.59772099086379105,0.16305764599715164},{-0.39149965032746059,0.0021328099999205997,-0.030199520478619075},{-0.41992319395443556,0.61572121224555043,0.14991180844948704},{-0.39282568379174854,1.2072399522656994,0.66420443721182809},{0.41911351970726396,-1.1983491283420222,0.66413221634405795},{0.40246548204497207,-0.59927118635405396,0.15197623571470295},{0.40026123188731882,0.012022116371818421,-0.012743547935508741},{0.41178466751676646,0.58029067004019319,0.14337094132897035},{0.38059472732812322,1.1949847843415273,0.62830987178346476},{1.2053120512454614,-1.2036421600088518,1.2701649874793173},{1.2142059292015448,-0.61808050895734234,0.77567835587962464},{1.2015817831720175,-0.019897876840639941,0.57321677463592469},{1.2073334840078604,0.6180949332563288,0.77624985269131419},{1.1980713968063013,1.216310091982348,1.2587362925951868}};
  const std::vector<double> wtsB = {1.7846924039029641,1.5728765041792359,1.0369128249119424,0.40378889180326355,1.3896097192825165,1.0825362707577295,1.1928789078401725,0.87806398388733031,2.0664276889501463,1.0743668079784259,0.93486293005716237,1.3654385448074935,1.3244880002840893,1.4865953810806807,1.0936465018653696,0.45860089694692535,2.2084261108541203,1.0410340402077738,0.46155052292864407,1.6200227358855492};
  const std::vector<double> kUB = {0,0,0,0,1,1,1,1};
  const std::vector<double> kVB = {0,0,0,0.33333333333333331,0.66666666666666663,1,1,1};

  auto A = ssi::makeBSplineAdapter(degAU, degAV, polesA, nAU, nAV, kUA, kVA);
  auto B = ssi::makeNurbsAdapter(degBU, degBV, polesB, wtsB, nBU, nBV, kUB, kVB);

  ssi::SeedOptions so;
  so.initialGridU = 6;
  so.initialGridV = 6;
  so.minPatchFrac = 1.0 / 32.0;
  auto tr = ssi::trace_intersection(A, B, so);  // default MarchOptions → densify on

  // One dense, high-curvature traced branch (a tight glancing loop that exits the finite
  // patch domain, so it is a BoundaryExit rather than a Closed loop — either is a real traced
  // branch; what matters is that it is DENSE and high-curvature, the fit-density regime).
  CC_CHECK(tr.tracedBranches >= 1);
  const ssi::WLine* loop = nullptr;
  for (const auto& w : tr.lines) {
    const bool traced = w.status == ssi::TraceStatus::Closed ||
                        w.status == ssi::TraceStatus::BoundaryExit;
    if (traced && w.points.size() > 500) { loop = &w; break; }
  }
  CC_CHECK(loop != nullptr);
  if (!loop) return;

  // (1) genuinely DENSE (the fit-density regime: many more nodes than the 64-pole cap).
  CC_CHECK(loop->points.size() > 500);

  // (2) every node on BOTH surfaces (the ground truth is untouched by the fit change).
  const nmath::SurfaceGrid gA{polesA, nAU, nAV};
  const nmath::SurfaceGrid gB{polesB, nBU, nBV};
  for (const auto& nd : loop->points) {
    CC_CHECK(nmath::distance(nmath::surfacePoint(degAU, degAV, gA, kUA, kVA, nd.u1, nd.v1), nd.point) < 1e-6);
    CC_CHECK(nmath::distance(nmath::nurbsSurfacePoint(degBU, degBV, gB, wtsB, kUB, kVB, nd.u2, nd.v2), nd.point) < 1e-6);
    CC_CHECK(nd.onSurfResidual < 1e-6);
  }

  // (3) the densify FIRED — the fit uses MORE than the default 64-pole cap.
  CC_CHECK(loop->curve.valid());
  CC_CHECK(static_cast<int>(loop->curve.poles.size()) > 64);

  // (4) the densified fitted curve RIDES the on-locus node polyline: sampled densely between
  //     the nodes, its worst distance to the node chord is WELL under the 1e-3 coverage budget
  //     (the ~5e-3 bow at 64 poles is gone). NO tolerance widened — this is fit quality only.
  const auto& c = loop->curve;
  const double t0 = c.knots.front(), t1 = c.knots.back();
  const std::size_t nPts = loop->points.size();
  double worstBow = 0.0;
  const int nSamp = 4000;
  for (int i = 0; i <= nSamp; ++i) {
    const double t = t0 + (t1 - t0) * (double(i) / nSamp);
    const Point3 p = nmath::curvePoint(c.degree, c.poles, c.knots, t);
    // nearest node-chord segment.
    double best = 1e30;
    for (std::size_t k = 1; k < nPts; ++k) {
      const Point3& a = loop->points[k - 1].point;
      const Point3& b = loop->points[k].point;
      const Vec3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
      const Vec3 aq{p.x - a.x, p.y - a.y, p.z - a.z};
      const double den = nmath::dot(ab, ab);
      double s = den > 0.0 ? nmath::dot(aq, ab) / den : 0.0;
      s = s < 0.0 ? 0.0 : (s > 1.0 ? 1.0 : s);
      const Point3 pr{a.x + s * ab.x, a.y + s * ab.y, a.z + s * ab.z};
      best = std::min(best, nmath::distance(p, pr));
    }
    worstBow = std::max(worstBow, best);
  }
  CC_CHECK(worstBow < 1e-3);              // densified fit rides the on-locus loop (bow gone)
  CC_CHECK(loop->curve.maxFitError < 1e-3);
}

int main() { return cctest::run_all(); }
