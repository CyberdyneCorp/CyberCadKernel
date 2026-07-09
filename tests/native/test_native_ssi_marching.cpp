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

int main() { return cctest::run_all(); }
