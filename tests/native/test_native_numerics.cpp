// SPDX-License-Identifier: Apache-2.0
//
// test_native_numerics.cpp — host unit tests for the numeric facade
// (src/native/numerics) over NumPP + SciPP. Analytic / known-value assertions,
// no OCCT. Built only when CYBERCAD_HAS_NUMSCI=ON.
//
// Gate 1 for Phase-4 #2: proves scalar_root / solve_system / minimize /
// least_squares / lin_solve / lstsq converge, and that closest_point_on_curve /
// closest_point_on_surface land on the analytic nearest point of native
// geometry (parabola, line, plane, cylinder, sphere, bicubic B-spline).
//
#include <cmath>
#include <cstdio>
#include <span>
#include <vector>

#include "native/numerics/numerics.h"
#include "native/numerics/native_numerics.h"
#include "native/math/bezier.h"
#include "native/math/bspline.h"
#include "native/math/elementary.h"
#include "native/math/torus.h"
#include "native/math/vec.h"

namespace nn = cybercad::native::numerics;
namespace nm = cybercad::native::math;

static int g_pass = 0, g_fail = 0;

static void check(const char* name, bool ok, double got = 0, double want = 0) {
  if (ok) { ++g_pass; std::printf("  PASS %-42s\n", name); }
  else    { ++g_fail; std::printf("  FAIL %-42s got=%.10g want=%.10g\n", name, got, want); }
}
static bool close(double a, double b, double tol = 1e-6) { return std::fabs(a - b) <= tol; }

// ── scalar roots ─────────────────────────────────────────────────────────────
static void test_scalar_roots() {
  std::printf("[scalar roots]\n");
  auto r1 = nn::scalar_root_brentq([](double x) { return x * x - 2.0; }, 0.0, 2.0);
  check("brentq x^2-2 -> sqrt2", r1.converged && close(r1.root, std::sqrt(2.0)), r1.root, std::sqrt(2.0));

  auto r2 = nn::scalar_root_newton([](double x) { return std::cos(x) - x; }, 0.5);
  check("newton cos(x)=x -> Dottie", r2.converged && close(r2.root, 0.7390851332), r2.root, 0.7390851332);
}

// ── solve_system / minimize / least_squares ──────────────────────────────────
static void test_solvers() {
  std::printf("[solvers]\n");
  // fsolve on x^2+y^2=4, x*y=1 near (1.9,0.5).
  auto F = [](const nn::Vector& v) {
    const double x = v[0], y = v[1];
    return nn::Vector{x * x + y * y - 4.0, x * y - 1.0};
  };
  auto s = nn::solve_system(F, {1.9, 0.5});
  const double x = s.x[0], y = s.x[1];
  check("fsolve 2x2 nonlinear", s.success && close(x * x + y * y, 4.0) && close(x * y, 1.0), x * x + y * y, 4.0);

  // BFGS on the Rosenbrock -> (1,1).
  auto rosen = [](const nn::Vector& v) {
    const double a = v[0], b = v[1];
    return (1 - a) * (1 - a) + 100.0 * (b - a * a) * (b - a * a);
  };
  auto m = nn::minimize(rosen, {-1.2, 1.0}, 1e-10, 5000);
  check("BFGS rosenbrock -> (1,1)", m.success && close(m.x[0], 1.0, 1e-3) && close(m.x[1], 1.0, 1e-3), m.x[0], 1.0);

  // least_squares line fit y=2x+1 -> (2,1).
  std::vector<double> xs = {0, 1, 2, 3, 4}, ys = {1, 3, 5, 7, 9};
  auto resid = [&](const nn::Vector& p) {
    nn::Vector r(xs.size());
    for (std::size_t i = 0; i < xs.size(); ++i) r[i] = p[0] * xs[i] + p[1] - ys[i];
    return r;
  };
  auto ls = nn::least_squares(resid, {0.0, 0.0});
  check("least_squares line fit -> (2,1)", ls.success && close(ls.x[0], 2.0) && close(ls.x[1], 1.0), ls.x[0], 2.0);
}

// ── dense linalg ─────────────────────────────────────────────────────────────
static void test_linalg() {
  std::printf("[dense linalg]\n");
  // SPD 3x3 solve, verify residual.
  std::vector<double> A = {4, 1, 1, 1, 3, 0, 1, 0, 2};
  std::vector<double> b = {6, 4, 3};
  auto xr = nn::lin_solve(A, 3, b);
  bool ok = xr.size() == 3;
  double maxr = 0;
  for (int i = 0; ok && i < 3; ++i) {
    double acc = 0;
    for (int j = 0; j < 3; ++j) acc += A[i * 3 + j] * xr[j];
    maxr = std::max(maxr, std::fabs(acc - b[i]));
  }
  check("lin_solve 3x3 SPD residual~0", ok && maxr < 1e-9, maxr, 0.0);

  // Overdetermined 4x2 lstsq: y=2x+1.
  std::vector<double> M = {0, 1, 1, 1, 2, 1, 3, 1};
  std::vector<double> y = {1, 3, 5, 7};
  auto sol = nn::lstsq(M, 4, 2, y);
  check("lstsq 4x2 -> slope 2 intercept 1", sol.size() == 2 && close(sol[0], 2.0) && close(sol[1], 1.0), sol.size() ? sol[0] : 0, 2.0);
}

// ── closest point on native curve ────────────────────────────────────────────
static void test_closest_curve() {
  std::printf("[closest point on curve]\n");
  // Parabola c(t) = (t, t^2, 0), t in [-2,2], target (0, -1, 0) -> nearest at t=0.
  auto parab = [](double t) { return nm::Point3{t, t * t, 0.0}; };
  auto cp = nn::closest_point_on_curve(parab, -2.0, 2.0, nm::Point3{0, -1, 0}, 40);
  check("parabola nearest to (0,-1) -> t=0", cp.success && close(cp.t, 0.0, 1e-4) && close(cp.distance, 1.0, 1e-4), cp.t, 0.0);

  // Line c(t) = (t, 0, 0), t in [0,10], target (3, 4, 0) -> foot t=3, dist 4.
  auto line = [](double t) { return nm::Point3{t, 0.0, 0.0}; };
  auto lp = nn::closest_point_on_curve(line, 0.0, 10.0, nm::Point3{3, 4, 0}, 20);
  check("line nearest to (3,4) -> t=3 d=4", lp.success && close(lp.t, 3.0, 1e-4) && close(lp.distance, 4.0, 1e-4), lp.t, 3.0);
}

// ── closest point on native surfaces ─────────────────────────────────────────
static void test_closest_surface() {
  std::printf("[closest point on surface]\n");
  // Plane z=0 (XY), target (2,3,5) -> foot (2,3,0), distance 5.
  nm::Plane plane;
  plane.pos = nm::Ax3{nm::Point3{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
  auto pl = nn::closest_point_on_surface(
      [&](double u, double v) { return plane.value(u, v); }, -10, 10, -10, 10,
      nm::Point3{2, 3, 5}, 20, 20);
  check("plane nearest to (2,3,5) -> d=5", pl.success && close(pl.distance, 5.0, 1e-4) &&
        close(pl.point.x, 2.0, 1e-3) && close(pl.point.y, 3.0, 1e-3), pl.distance, 5.0);

  // Cylinder radius 2 about Z, target (5,0,1) outside -> nearest on wall dist 3.
  nm::Cylinder cyl;
  cyl.pos = nm::Ax3{nm::Point3{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
  cyl.radius = 2.0;
  auto cy = nn::closest_point_on_surface(
      [&](double u, double v) { return cyl.value(u, v); }, 0, 2 * M_PI, -5, 5,
      nm::Point3{5, 0, 1}, 32, 8);
  check("cylinder R2 nearest to (5,0,1) -> d=3", cy.success && close(cy.distance, 3.0, 1e-3), cy.distance, 3.0);

  // Sphere radius 1 at origin, target (0,0,4) -> nearest north pole, dist 3.
  nm::Sphere sph;
  sph.pos.origin = nm::Point3{0, 0, 0};
  sph.radius = 1.0;
  auto sp = nn::closest_point_on_surface(
      [&](double u, double v) { return sph.value(u, v); }, 0, 2 * M_PI, -M_PI / 2, M_PI / 2,
      nm::Point3{0, 0, 4}, 24, 24);
  check("sphere R1 nearest to (0,0,4) -> d=3", sp.success && close(sp.distance, 3.0, 1e-3), sp.distance, 3.0);
}

// ── closest point on a bicubic B-spline surface (vs dense brute force) ────────
static void test_closest_bspline() {
  std::printf("[closest point on B-spline surface]\n");
  // 4x4 bicubic clamped surface with a central bump (same as the eval probe).
  std::vector<nm::Point3> poles(16);
  std::vector<double> ku = {0, 0, 0, 0, 1, 1, 1, 1};
  std::vector<double> kv = {0, 0, 0, 0, 1, 1, 1, 1};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double z = ((i == 1 || i == 2) && (j == 1 || j == 2)) ? 1.5 : 0.0;
      poles[i * 4 + j] = nm::Point3{double(i), double(j), z};
    }
  nm::SurfaceGrid grid{poles, 4, 4};
  auto eval = [&](double u, double v) { return nm::surfacePoint(3, 3, grid, ku, kv, u, v); };
  nm::Point3 target{2.3, 0.8, 2.6};

  auto sp = nn::closest_point_on_surface(eval, 0, 1, 0, 1, target, 16, 16);

  // Dense brute-force reference distance.
  double bestD = 1e18;
  const int N = 300;
  for (int i = 0; i <= N; ++i)
    for (int j = 0; j <= N; ++j) {
      nm::Point3 p = eval(double(i) / N, double(j) / N);
      double d = nm::distance(p, target);
      if (d < bestD) bestD = d;
    }
  check("bspline nearest matches brute force", sp.success && close(sp.distance, bestD, 1e-3), sp.distance, bestD);
}

// ── typed closest-point layer (closest_point.h) ──────────────────────────────
// The typed project_point_to_* overloads on native geometry, exercising
// endpoint / boundary reporting and the multi-start local-minima set.
static void test_typed_projection() {
  std::printf("[typed projection: closest_point.h]\n");

  // Bézier curve, straight segment (0,0,0)->(4,0,0). Target (3,4,0):
  //   foot at t=0.75 (point (3,0,0)), distance 4, NOT on an endpoint.
  std::vector<nm::Point3> bez = {{0, 0, 0}, {4.0 / 3, 0, 0}, {8.0 / 3, 0, 0}, {4, 0, 0}};
  auto bc = nn::project_point_to_curve(nm::Point3{3, 4, 0}, std::span<const nm::Point3>(bez));
  check("bezier line foot t=0.75 d=4", bc.success && close(bc.t, 0.75, 1e-4) &&
        close(bc.distance, 4.0, 1e-4) && !bc.onEndpoint, bc.t, 0.75);

  // Same Bézier, target beyond the far end (10,0,0): nearest is the t=1 endpoint.
  auto be = nn::project_point_to_curve(nm::Point3{10, 0, 0}, std::span<const nm::Point3>(bez));
  check("bezier beyond-end -> endpoint t=1", be.success && close(be.t, 1.0, 1e-4) &&
        be.onEndpoint, be.t, 1.0);

  // B-spline curve == the same straight segment as a clamped cubic; target on the
  // perpendicular through the midpoint -> foot t=2 (midpoint), interior.
  std::vector<nm::Point3> bp = {{0, 0, 0}, {1, 0, 0}, {3, 0, 0}, {4, 0, 0}};
  std::vector<double> kt = {0, 0, 0, 0, 4, 4, 4, 4};  // clamped cubic, domain [0,4]
  auto bs = nn::project_point_to_bspline_curve(nm::Point3{2, 5, 0}, 3,
              std::span<const nm::Point3>(bp), std::span<const double>(kt));
  check("bspline curve foot at midpoint", bs.success && close(bs.point.y, 0.0, 1e-4) &&
        close(bs.point.x, 2.0, 1e-3) && !bs.onEndpoint, bs.point.x, 2.0);

  // NURBS quarter circle (rational quadratic, R=1 in XY). Target (2,0,0) outside:
  //   nearest on the arc at (1,0,0), distance 1.
  std::vector<nm::Point3> np = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  std::vector<double> nw = {1.0, std::sqrt(2.0) / 2.0, 1.0};
  std::vector<double> nk = {0, 0, 0, 1, 1, 1};  // clamped quadratic, domain [0,1]
  auto nc = nn::project_point_to_nurbs_curve(nm::Point3{2, 0, 0}, 2,
              std::span<const nm::Point3>(np), std::span<const double>(nw),
              std::span<const double>(nk));
  check("nurbs quarter-circle nearest d=1", nc.success && close(nc.distance, 1.0, 1e-4) &&
        close(nc.point.x, 1.0, 1e-3) && close(nc.point.y, 0.0, 1e-3), nc.distance, 1.0);

  // Typed Sphere overload: R=2 at origin, target (0,0,5) -> north pole, d=3, on
  // the v-boundary (latitude = +pi/2).
  nm::Sphere sph; sph.pos.origin = nm::Point3{0, 0, 0}; sph.radius = 2.0;
  auto sp = nn::project_point_to_surface(nm::Point3{0, 0, 5}, sph,
              0, 2 * M_PI, -M_PI / 2, M_PI / 2, 24, 24);
  check("typed sphere -> pole d=3 onBoundary", sp.success && close(sp.distance, 3.0, 1e-3) &&
        sp.onBoundary, sp.distance, 3.0);

  // Typed Cylinder overload: R=2 about Z, target (5,0,1) -> wall d=3, interior.
  nm::Cylinder cyl;
  cyl.pos = nm::Ax3{nm::Point3{0, 0, 0}, nm::Dir3{1, 0, 0}, nm::Dir3{0, 1, 0}, nm::Dir3{0, 0, 1}};
  cyl.radius = 2.0;
  auto cy = nn::project_point_to_surface(nm::Point3{5, 0, 1}, cyl, 0, 2 * M_PI, -5, 5, 32, 8);
  check("typed cylinder wall d=3", cy.success && close(cy.distance, 3.0, 1e-3), cy.distance, 3.0);

  // Typed Torus overload: R=3 r=1 about Z, target on axis at (5,0,0) in the XY
  // plane, beyond the outer equator -> nearest at (4,0,0), distance 1, v-interior.
  nm::Torus tor; tor.majorRadius = 3.0; tor.minorRadius = 1.0;
  auto to = nn::project_point_to_surface(nm::Point3{5, 0, 0}, tor,
              0, 2 * M_PI, 0, 2 * M_PI, 24, 16);
  check("typed torus outer-equator d=1", to.success && close(to.distance, 1.0, 2e-3),
        to.distance, 1.0);
}

// ── multi-start robustness: two symmetric minima on a curve ───────────────────
// A sine curve c(t)=(t, sin t, 0) over [0, 4pi] with the target on the x-axis has
// TWO equally-near troughs (t≈3pi/2 and t≈7pi/2). Multi-start must find both as
// local minima, not swallow one.
static void test_multistart_minima() {
  std::printf("[multi-start local minima]\n");
  // A circle in the XY plane, R=2 centred at origin, parametrised over [0,2pi).
  // Target at the CENTRE (0,0,0): EVERY point is equidistant (d=2), so the grid
  // yields many equal local minima. A single-start refine would report just one;
  // multi-start must surface several. Also verifies the global distance is exact.
  auto circle = [](double t) { return nm::Point3{2 * std::cos(t), 2 * std::sin(t), 0.0}; };
  nm::Point3 target{0, 0, 0};
  auto r = nn::project_point_to_curve(target, circle, 0.0, 2 * M_PI, 48);
  bool globalDist = r.success && close(r.distance, 2.0, 1e-6);
  bool manyMinima = r.extrema.size() >= 2;
  check("circle-from-centre global d=2", globalDist, r.distance, 2.0);
  check("circle-from-centre: >=2 local minima found", manyMinima,
        double(r.extrema.size()), 2.0);
}

int main() {
  std::printf("=== test_native_numerics (%s) ===\n", nn::substrate_versions().c_str());
  test_scalar_roots();
  test_solvers();
  test_linalg();
  test_closest_curve();
  test_closest_surface();
  test_closest_bspline();
  test_typed_projection();
  test_multistart_minima();
  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
