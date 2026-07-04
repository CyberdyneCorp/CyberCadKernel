// SPDX-License-Identifier: Apache-2.0
//
// numerics.h — the kernel-facing numeric facade (Phase-4 #2, numeric-foundations).
//
// A thin, OCCT-FREE wrapper over the adopted numeric substrate NumPP + the SciPP
// optimize/linalg subset (docs/EVAL-numpp-scipp.md, verdict GO-WITH-HARDENING).
// It exposes exactly the routines the kernel needs — scalar root finding, a
// nonlinear system solve, unconstrained minimization, nonlinear least squares,
// dense solve / least-squares — plus native CLOSEST-POINT / projection built on
// top of them (the OCCT `Extrema` on-ramp).
//
//   * scalar_root   — brentq (bracketed) / newton (from a guess)
//   * solve_system  — fsolve: F(x)=0 for a square nonlinear system
//   * minimize      — BFGS unconstrained minimization of f: Rⁿ → R
//   * least_squares — Levenberg–Marquardt min ‖r(x)‖² for r: Rⁿ → Rᵐ
//   * lin_solve     — dense A x = b (NumPP linalg::solve)
//   * lstsq         — dense least-squares min ‖A x − b‖ (NumPP linalg::lstsq)
//   * closest_point_on_curve   — nearest parameter t on a native curve to a 3D point
//   * closest_point_on_surface — nearest (u,v) on a native surface to a 3D point
//
// DESIGN — no leaked substrate types. Callers pass ordinary std::function
// callables over std::vector<double> / native math Point3/Vec3, and receive
// plain structs. NumPP's `ndarray` and SciPP's result types NEVER appear in this
// header, so a TU that includes numerics.h does NOT pull in NumPP/SciPP. The
// implementation (numerics.cpp) is the only place that touches the substrate.
//
// GUARD — the whole module is compiled only when CYBERCAD_HAS_NUMSCI is defined
// (CMake option). The rest of src/native builds and tests without it: nothing
// outside src/native/numerics includes this header. When the guard is OFF this
// header is inert (the declarations remain visible for documentation, but the
// implementation TU is not compiled and the substrate is not linked).
//
// clang++ -std=c++20. fp64, deterministic. OCCT-free; substrate referenced by
// absolute path exactly like OCCT (see scripts/build-numsci.sh).
//
#ifndef CYBERCAD_NATIVE_NUMERICS_H
#define CYBERCAD_NATIVE_NUMERICS_H

#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "native/math/vec.h"

namespace cybercad::native::numerics {

using cybercad::native::math::Point3;
using cybercad::native::math::Vec3;

// A dense real vector in the kernel's own terms — no ndarray leaks here.
using Vector = std::vector<double>;

// Callables the facade accepts. All operate on plain Vector / double.
using ScalarFn = std::function<double(double)>;         // R → R
using ObjFn    = std::function<double(const Vector&)>;  // Rⁿ → R
using VecFn    = std::function<Vector(const Vector&)>;  // Rⁿ → Rᵐ

// ─────────────────────────────────────────────────────────────────────────────
// Generic solver results.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a scalar root solve.
struct RootResult {
  double root = 0.0;
  bool converged = false;
  int iterations = 0;
};

/// Result of a vector solve / minimize / least-squares.
struct SolveResult {
  Vector x;               ///< the solution vector
  double cost = 0.0;      ///< objective value (minimize) or ½‖r‖² (least_squares)
  bool success = false;   ///< converged within tolerance
  int nfev = 0;           ///< function evaluations
};

// ─────────────────────────────────────────────────────────────────────────────
// Scalar root finding.
// ─────────────────────────────────────────────────────────────────────────────

/// Bracketed root of `f` on [a, b] (Brent's method). Requires f(a)·f(b) ≤ 0.
RootResult scalar_root_brentq(const ScalarFn& f, double a, double b,
                              double xtol = 2e-12, int maxiter = 100);

/// Root of `f` near `x0` (Newton–secant). `fprime` optional; secant if omitted.
RootResult scalar_root_newton(const ScalarFn& f, double x0,
                              const ScalarFn& fprime = nullptr,
                              double tol = 1.48e-8, int maxiter = 50);

// ─────────────────────────────────────────────────────────────────────────────
// Nonlinear system / minimization / least squares.
// ─────────────────────────────────────────────────────────────────────────────

/// Solve the square nonlinear system F(x) = 0 (SciPy `fsolve`, MINPACK hybrd).
/// `x0` seeds the iteration; the returned `x` has the same length as `x0`.
SolveResult solve_system(const VecFn& F, const Vector& x0,
                         double xtol = 1.49012e-8, int maxiter = 200);

/// Unconstrained minimization of f: Rⁿ → R by BFGS.
SolveResult minimize(const ObjFn& f, const Vector& x0,
                     double tol = 1e-8, int maxiter = 1000);

/// Nonlinear least squares: min ½‖r(x)‖² by Levenberg–Marquardt.
/// `r` returns an m-vector residual for the n-vector `x` (m ≥ n typical).
SolveResult least_squares(const VecFn& r, const Vector& x0,
                          double ftol = 1e-8, double xtol = 1e-8,
                          int max_nfev = 1000);

// ─────────────────────────────────────────────────────────────────────────────
// Dense linear algebra (NumPP reference engine — no external LAPACK).
// Matrices are passed row-major: A is n×n (lin_solve) or m×n (lstsq).
// ─────────────────────────────────────────────────────────────────────────────

/// Solve the dense linear system A x = b. `a` is n×n row-major, `b` length n.
/// Returns the length-n solution (empty on a singular/ill-formed system).
Vector lin_solve(const Vector& a, int n, const Vector& b);

/// Least-squares solution of A x = b. `a` is m×n row-major, `b` length m.
/// Returns the length-n minimum-norm solution.
Vector lstsq(const Vector& a, int rows, int cols, const Vector& b);

// ─────────────────────────────────────────────────────────────────────────────
// Closest point / projection (native `Extrema` on-ramp).
//
// The curve / surface is supplied as a plain parametric evaluator, so ANY native
// geometry (B-spline / Bézier / NURBS / plane / cylinder / cone / sphere / torus)
// projects through the same code with no substrate types crossing the boundary.
// Robustness comes from MULTI-START seeding: the parameter domain is sampled on a
// coarse grid, the best sample seeds a local minimize, and the best local result
// wins. This mirrors what OCCT Extrema does (sample + local refine).
// ─────────────────────────────────────────────────────────────────────────────

/// A parametric curve C: [t0, t1] → R³.
using CurveEval = std::function<Point3(double t)>;

/// A parametric surface S: [u0,u1]×[v0,v1] → R³.
using SurfaceEval = std::function<Point3(double u, double v)>;

/// Nearest point on a curve to a target.
struct CurveProjection {
  double t = 0.0;         ///< parameter of the nearest point
  Point3 point{};         ///< C(t)
  double distance = 0.0;  ///< ‖C(t) − target‖
  bool success = false;
};

/// Nearest point on a surface to a target.
struct SurfaceProjection {
  double u = 0.0;
  double v = 0.0;
  Point3 point{};         ///< S(u, v)
  double distance = 0.0;  ///< ‖S(u,v) − target‖
  bool success = false;
};

/// Project `target` onto the curve `c` over [t0, t1]. `samples` sets the
/// multi-start grid density (a higher count is more robust for wiggly curves).
CurveProjection closest_point_on_curve(const CurveEval& c, double t0, double t1,
                                       const Point3& target, int samples = 32);

/// Project `target` onto the surface `s` over [u0,u1]×[v0,v1]. `samplesU/V` set
/// the multi-start grid density in each parameter direction.
SurfaceProjection closest_point_on_surface(const SurfaceEval& s,
                                           double u0, double u1,
                                           double v0, double v1,
                                           const Point3& target,
                                           int samplesU = 16, int samplesV = 16);

/// The linked NumPP / SciPP versions (informational; e.g. "NumPP 1.6.0 / SciPP 1.2.0").
std::string substrate_versions();

}  // namespace cybercad::native::numerics

#endif  // CYBERCAD_NATIVE_NUMERICS_H
