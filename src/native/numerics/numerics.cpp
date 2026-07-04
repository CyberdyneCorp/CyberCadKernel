// SPDX-License-Identifier: Apache-2.0
//
// numerics.cpp — implementation of the numeric facade over NumPP + SciPP.
//
// This is the ONLY TU in the kernel that includes NumPP / SciPP. It is compiled
// only when CYBERCAD_HAS_NUMSCI is defined (CMake option); when the option is
// OFF this file is not added to the build, so the substrate is never linked and
// the rest of src/native is unaffected.
//
// The substrate uses NumPy/SciPy conventions: vectors are numpp::ndarray, and
// SciPy-shaped result structs come back from scipp::optimize. We translate at
// this boundary so numerics.h stays free of every substrate type.
//
#if defined(CYBERCAD_HAS_NUMSCI)

#include "native/numerics/numerics.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>

#include "numpp/core/ndarray.hpp"
#include "numpp/linalg/linalg.hpp"
#include "numpp/version.hpp"
#include "scipp/optimize/optimize.hpp"
#include "scipp/version.hpp"

namespace cybercad::native::numerics {
namespace {

namespace op = scipp::optimize;
using numpp::ndarray;

// ── Vector <-> ndarray bridge ────────────────────────────────────────────────
ndarray toND(const Vector& v) {
  ndarray a(numpp::Shape{static_cast<int64_t>(v.size())}, numpp::kFloat64);
  double* p = a.typed_data<double>();
  for (std::size_t i = 0; i < v.size(); ++i) p[i] = v[i];
  return a;
}

Vector toVec(const ndarray& a) {
  ndarray c = a.astype(numpp::kFloat64).ascontiguousarray();
  const double* p = c.typed_data<double>();
  return Vector(p, p + c.size());
}

ndarray toMatrix(const Vector& data, int rows, int cols) {
  ndarray m(numpp::Shape{rows, cols}, numpp::kFloat64);
  double* p = m.typed_data<double>();
  const std::size_t n = static_cast<std::size_t>(rows) * cols;
  for (std::size_t i = 0; i < n && i < data.size(); ++i) p[i] = data[i];
  return m;
}

// Wrap a facade VecFn (Vector→Vector) as a SciPP VecFn (ndarray→ndarray).
op::VecFn wrapVecFn(const VecFn& f) {
  return [f](const ndarray& x) { return toND(f(toVec(x))); };
}

double euclid(const Point3& a, const Point3& b) {
  const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double clampd(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

double dotP(const Point3& a, const Point3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

// fp64 Newton polish of the closest-point parameter on the orthogonality root.
// BFGS pins the distance but can leave the foot point ~1e-6 off tangentially where
// the squared distance is flat; a few damped-Newton steps on g=(S−T)·S' (FD
// derivatives, range-clamped, accept only if the squared distance drops) drive the
// foot point to fp64. Local refinement of an already-correct basin.
double polishCurveT(const CurveEval& c, const Point3& T, double t,
                    double t0, double t1) {
  auto d2 = [&](double x) { const Point3 p = c(clampd(x, t0, t1));
    const double dx=p.x-T.x, dy=p.y-T.y, dz=p.z-T.z; return dx*dx+dy*dy+dz*dz; };
  const double h = std::max(1e-7, 1e-7 * (t1 - t0));
  double bt = clampd(t, t0, t1), bd = d2(bt);
  for (int it = 0; it < 8; ++it) {
    const Point3 cp = c(bt), cph = c(clampd(bt+h,t0,t1)), cmh = c(clampd(bt-h,t0,t1));
    const Point3 d1{(cph.x-cmh.x)/(2*h),(cph.y-cmh.y)/(2*h),(cph.z-cmh.z)/(2*h)};
    const Point3 dd{(cph.x-2*cp.x+cmh.x)/(h*h),(cph.y-2*cp.y+cmh.y)/(h*h),(cph.z-2*cp.z+cmh.z)/(h*h)};
    const Point3 r{cp.x-T.x, cp.y-T.y, cp.z-T.z};
    const double g = dotP(r,d1), gp = dotP(d1,d1)+dotP(r,dd);
    if (std::fabs(gp) < 1e-30) break;
    double step = g/gp; bool imp = false;
    for (int k=0;k<20;++k){ const double nt=clampd(bt-step,t0,t1), nd=d2(nt);
      if (nd<=bd){bt=nt;bd=nd;imp=true;break;} step*=0.5; }
    if (!imp || std::fabs(step) <= 1e-16*(1.0+std::fabs(bt))) break;
  }
  return bt;
}

std::pair<double,double> polishSurfUV(const SurfaceEval& s, const Point3& T,
                                      double u, double v, double u0, double u1,
                                      double v0, double v1) {
  auto d2 = [&](double a, double b){ const Point3 p=s(clampd(a,u0,u1),clampd(b,v0,v1));
    const double dx=p.x-T.x,dy=p.y-T.y,dz=p.z-T.z; return dx*dx+dy*dy+dz*dz; };
  const double hu = std::max(1e-7, 1e-7*(u1-u0)), hv = std::max(1e-7, 1e-7*(v1-v0));
  double bu = clampd(u,u0,u1), bv = clampd(v,v0,v1), bd = d2(bu,bv);
  for (int it=0; it<8; ++it) {
    const Point3 p=s(bu,bv);
    const Point3 pu2=s(clampd(bu+hu,u0,u1),bv), pu0=s(clampd(bu-hu,u0,u1),bv);
    const Point3 pv2=s(bu,clampd(bv+hv,v0,v1)), pv0=s(bu,clampd(bv-hv,v0,v1));
    const Point3 Su{(pu2.x-pu0.x)/(2*hu),(pu2.y-pu0.y)/(2*hu),(pu2.z-pu0.z)/(2*hu)};
    const Point3 Sv{(pv2.x-pv0.x)/(2*hv),(pv2.y-pv0.y)/(2*hv),(pv2.z-pv0.z)/(2*hv)};
    const Point3 Suu{(pu2.x-2*p.x+pu0.x)/(hu*hu),(pu2.y-2*p.y+pu0.y)/(hu*hu),(pu2.z-2*p.z+pu0.z)/(hu*hu)};
    const Point3 Svv{(pv2.x-2*p.x+pv0.x)/(hv*hv),(pv2.y-2*p.y+pv0.y)/(hv*hv),(pv2.z-2*p.z+pv0.z)/(hv*hv)};
    const Point3 puv=s(clampd(bu+hu,u0,u1),clampd(bv+hv,v0,v1));
    const Point3 Suv{(puv.x-pu2.x-pv2.x+p.x)/(hu*hv),(puv.y-pu2.y-pv2.y+p.y)/(hu*hv),(puv.z-pu2.z-pv2.z+p.z)/(hu*hv)};
    const Point3 r{p.x-T.x,p.y-T.y,p.z-T.z};
    const double gu=dotP(r,Su), gv=dotP(r,Sv);
    const double J00=dotP(Su,Su)+dotP(r,Suu), J01=dotP(Su,Sv)+dotP(r,Suv), J11=dotP(Sv,Sv)+dotP(r,Svv);
    const double det=J00*J11-J01*J01;
    if (std::fabs(det) < 1e-30) break;
    double du=(J11*gu-J01*gv)/det, dv=(J00*gv-J01*gu)/det; bool imp=false;
    for (int k=0;k<20;++k){ const double nu=clampd(bu-du,u0,u1), nv=clampd(bv-dv,v0,v1), nd=d2(nu,nv);
      if (nd<=bd){bu=nu;bv=nv;bd=nd;imp=true;break;} du*=0.5; dv*=0.5; }
    if (!imp || (std::fabs(du)<=1e-16*(1.0+std::fabs(bu)) && std::fabs(dv)<=1e-16*(1.0+std::fabs(bv)))) break;
  }
  return {bu,bv};
}

}  // namespace

// ── scalar roots ──────────────────────────────────────────────────────────────
RootResult scalar_root_brentq(const ScalarFn& f, double a, double b,
                              double xtol, int maxiter) {
  RootResult r;
  try {
    r.root = op::brentq(f, a, b, xtol, 8.881784197001252e-16, maxiter);
    r.converged = std::isfinite(r.root) && std::fabs(f(r.root)) < 1e-6;
  } catch (...) {
    r.converged = false;
  }
  return r;
}

RootResult scalar_root_newton(const ScalarFn& f, double x0, const ScalarFn& fprime,
                              double tol, int maxiter) {
  RootResult r;
  try {
    r.root = op::newton(f, x0, fprime, tol, maxiter);
    r.converged = std::isfinite(r.root) && std::fabs(f(r.root)) < 1e-6;
  } catch (...) {
    r.converged = false;
  }
  return r;
}

// ── nonlinear system / minimize / least squares ──────────────────────────────
SolveResult solve_system(const VecFn& F, const Vector& x0, double xtol, int maxiter) {
  SolveResult out;
  out.x = x0;
  try {
    ndarray sol = op::fsolve(wrapVecFn(F), toND(x0), xtol, maxiter);
    out.x = toVec(sol);
    const Vector residual = F(out.x);
    double sq = 0.0;
    for (double e : residual) sq += e * e;
    out.cost = 0.5 * sq;
    out.success = std::sqrt(sq) < 1e-6;
  } catch (...) {
    out.success = false;
  }
  return out;
}

SolveResult minimize(const ObjFn& f, const Vector& x0, double tol, int maxiter) {
  SolveResult out;
  out.x = x0;
  try {
    op::ObjFn obj = [f](const ndarray& x) { return f(toVec(x)); };
    op::OptimizeResult res = op::minimize(obj, toND(x0), "BFGS", tol, maxiter);
    out.x = toVec(res.x);
    out.cost = res.fun;
    out.nfev = res.nfev;
    out.success = res.success;
  } catch (...) {
    out.success = false;
  }
  return out;
}

SolveResult least_squares(const VecFn& r, const Vector& x0, double ftol,
                          double xtol, int max_nfev) {
  SolveResult out;
  out.x = x0;
  try {
    op::LeastSquaresResult res = op::least_squares(wrapVecFn(r), toND(x0), ftol, xtol, max_nfev);
    out.x = toVec(res.x);
    out.cost = res.cost;
    out.nfev = res.nfev;
    out.success = res.success;
  } catch (...) {
    out.success = false;
  }
  return out;
}

// ── dense linear algebra (NumPP) ──────────────────────────────────────────────
Vector lin_solve(const Vector& a, int n, const Vector& b) {
  try {
    ndarray x = numpp::linalg::solve(toMatrix(a, n, n), toND(b));
    return toVec(x);
  } catch (...) {
    return {};
  }
}

Vector lstsq(const Vector& a, int rows, int cols, const Vector& b) {
  try {
    numpp::linalg::LstsqResult res = numpp::linalg::lstsq(toMatrix(a, rows, cols), toND(b));
    return toVec(res.solution);
  } catch (...) {
    return {};
  }
}

// ── closest point / projection (Extrema on-ramp) ─────────────────────────────
//
// Strategy (mirrors OCCT Extrema): sample the parameter domain on a coarse grid
// to find the basin of the global minimum, then refine locally with BFGS on the
// squared distance. Multi-start (keeping every grid seed as a candidate start)
// guards against a single local min swallowing the answer on a wiggly geometry.

CurveProjection closest_point_on_curve(const CurveEval& c, double t0, double t1,
                                       const Point3& target, int samples) {
  CurveProjection best;
  best.distance = std::numeric_limits<double>::max();
  const int n = std::max(2, samples);

  auto dist2 = [&](double t) {
    const Point3 p = c(clampd(t, t0, t1));
    const double dx = p.x - target.x, dy = p.y - target.y, dz = p.z - target.z;
    return dx * dx + dy * dy + dz * dz;
  };

  // Coarse scan: best grid parameter seeds the local refine.
  double seed = t0, seedD2 = std::numeric_limits<double>::max();
  for (int i = 0; i <= n; ++i) {
    const double t = t0 + (t1 - t0) * (double(i) / n);
    const double d2 = dist2(t);
    if (d2 < seedD2) { seedD2 = d2; seed = t; }
  }

  // Local 1-D BFGS on the clamped squared distance from the best seed.
  ObjFn obj = [&](const Vector& x) { return dist2(x[0]); };
  SolveResult m = minimize(obj, {seed}, 1e-12, 200);
  double tm = clampd(m.success ? m.x[0] : seed, t0, t1);
  tm = polishCurveT(c, target, tm, t0, t1);  // fp64 orthogonality polish

  // Take whichever of {refined, seed} is nearer (refine can overshoot a clamp).
  for (double t : {tm, seed}) {
    const Point3 p = c(t);
    const double d = euclid(p, target);
    if (d < best.distance) {
      best.t = t; best.point = p; best.distance = d; best.success = true;
    }
  }
  return best;
}

SurfaceProjection closest_point_on_surface(const SurfaceEval& s,
                                           double u0, double u1,
                                           double v0, double v1,
                                           const Point3& target,
                                           int samplesU, int samplesV) {
  SurfaceProjection best;
  best.distance = std::numeric_limits<double>::max();
  const int nu = std::max(2, samplesU);
  const int nv = std::max(2, samplesV);

  auto dist2 = [&](double u, double v) {
    const Point3 p = s(clampd(u, u0, u1), clampd(v, v0, v1));
    const double dx = p.x - target.x, dy = p.y - target.y, dz = p.z - target.z;
    return dx * dx + dy * dy + dz * dz;
  };

  // Coarse grid scan → best (u,v) seed.
  double su = u0, sv = v0, seedD2 = std::numeric_limits<double>::max();
  for (int i = 0; i <= nu; ++i)
    for (int j = 0; j <= nv; ++j) {
      const double u = u0 + (u1 - u0) * (double(i) / nu);
      const double v = v0 + (v1 - v0) * (double(j) / nv);
      const double d2 = dist2(u, v);
      if (d2 < seedD2) { seedD2 = d2; su = u; sv = v; }
    }

  // Local BFGS on squared distance from the best seed.
  ObjFn obj = [&](const Vector& x) { return dist2(x[0], x[1]); };
  SolveResult m = minimize(obj, {su, sv}, 1e-12, 400);
  double um = clampd(m.success ? m.x[0] : su, u0, u1);
  double vm = clampd(m.success ? m.x[1] : sv, v0, v1);
  std::tie(um, vm) = polishSurfUV(s, target, um, vm, u0, u1, v0, v1);  // fp64 polish

  // Best of {refined, seed}.
  const std::pair<double, double> cands[2] = {{um, vm}, {su, sv}};
  for (auto [u, v] : cands) {
    const Point3 p = s(u, v);
    const double d = euclid(p, target);
    if (d < best.distance) {
      best.u = u; best.v = v; best.point = p; best.distance = d; best.success = true;
    }
  }
  return best;
}

std::string substrate_versions() {
  return std::string("NumPP ") + numpp::version() +
         " / SciPP " + scipp::version_string;
}

}  // namespace cybercad::native::numerics

#endif  // CYBERCAD_HAS_NUMSCI
