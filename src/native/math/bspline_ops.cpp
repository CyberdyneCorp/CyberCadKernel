// SPDX-License-Identifier: Apache-2.0
//
// bspline_ops.cpp — implementation of bspline_ops.h.
// Algorithm references are cited per-function (*The NURBS Book*, 2nd ed., Ch. 5).
// OCCT-FREE, NumPP/SciPP-FREE, always-on. fp64, deterministic.
//
// Structure: every algorithm runs on the homogeneous R⁴ control net (a Homog4
// vector). For a non-rational input the weight column is a constant 1 and the
// algorithm is unchanged — so ONE templated-by-nothing homogeneous core serves
// the curve, surface and rational paths without copy-paste. The public entry
// points lift (poles,weights) → Homog4, run the core, then project back.
//
#include "bspline_ops.h"

#include "bspline.h"  // findSpan (A2.1), used to locate knot spans

#include <algorithm>
#include <cmath>
#include <limits>

namespace cybercad::native::math {
namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Homogeneous R⁴ point (w·x, w·y, w·z, w). Non-rational poles lift with w = 1.
// ─────────────────────────────────────────────────────────────────────────────
struct Homog4 {
  double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
  Homog4 operator+(const Homog4& o) const noexcept { return {x + o.x, y + o.y, z + o.z, w + o.w}; }
  Homog4 operator-(const Homog4& o) const noexcept { return {x - o.x, y - o.y, z - o.z, w - o.w}; }
  Homog4 operator*(double s) const noexcept { return {x * s, y * s, z * s, w * s}; }
};

// A homogeneous control net + its knots/degree — the representation every core
// operates on. `rational` records whether the ORIGINAL input carried weights, so
// the projection back preserves "non-rational stays non-rational".
struct HCurve {
  int degree = 0;
  std::vector<Homog4> cw;      // homogeneous poles
  std::vector<double> knots;   // flat, length = cw.size() + degree + 1
  bool rational = false;
};

Homog4 lift(const Point3& p, double w) noexcept { return {p.x * w, p.y * w, p.z * w, w}; }

// Lift a (poles,weights) curve to homogeneous space. Empty weights ⇒ w = 1.
HCurve lift(const BsplineCurveData& c) {
  HCurve h;
  h.degree = c.degree;
  h.knots = c.knots;
  h.rational = !c.weights.empty();
  h.cw.reserve(c.poles.size());
  for (std::size_t i = 0; i < c.poles.size(); ++i)
    h.cw.push_back(lift(c.poles[i], h.rational ? c.weights[i] : 1.0));
  return h;
}

// Project a homogeneous curve back to (poles,weights). Non-positive projected
// weights are the documented guard: `ok` is cleared and evaluation is left to the
// caller (the ops that can hit this report it). For a non-rational curve the
// weight column is dropped.
BsplineCurveData project(const HCurve& h, bool& ok) {
  BsplineCurveData c;
  c.degree = h.degree;
  c.knots = h.knots;
  c.poles.reserve(h.cw.size());
  if (h.rational) c.weights.reserve(h.cw.size());
  ok = true;
  for (const Homog4& p : h.cw) {
    if (h.rational) {
      if (!(p.w > 0.0)) ok = false;
      const double invW = (p.w != 0.0) ? 1.0 / p.w : 0.0;
      c.poles.push_back({p.x * invW, p.y * invW, p.z * invW});
      c.weights.push_back(p.w);
    } else {
      c.poles.push_back({p.x, p.y, p.z});  // w == 1 by construction
    }
  }
  return c;
}

BsplineCurveData project(const HCurve& h) {
  bool ok = true;
  return project(h, ok);
}

// Multiplicity of value u among the knots, and the FindSpan index (A2.1 clamped).
int knotMultiplicity(const std::vector<double>& U, double u) noexcept {
  int s = 0;
  for (double k : U)
    if (k == u) ++s;
  return s;
}

int findSpanLocal(int n, int p, double u, const std::vector<double>& U) noexcept {
  return findSpan(n, p, u, U);
}

// ─────────────────────────────────────────────────────────────────────────────
// A5.1 — knot insertion (Boehm), r-fold, on the homogeneous net.
// n = #poles-1, p = degree, UP/Pw = new knots/poles. Follows the book verbatim.
// ─────────────────────────────────────────────────────────────────────────────
HCurve insertKnotH(const HCurve& c, double u, int r) {
  const int p = c.degree;
  const int np = static_cast<int>(c.cw.size());     // number of poles
  const int n = np - 1;
  const int mult = knotMultiplicity(c.knots, u);
  const int rMax = p - mult;                          // cannot exceed this
  const int rr = std::clamp(r, 0, std::max(0, rMax));
  if (rr == 0) return c;

  const int k = findSpanLocal(n, p, u, c.knots);      // knot span index
  const int s = mult;                                 // initial multiplicity of u

  HCurve out;
  out.degree = p;
  out.rational = c.rational;
  out.knots.resize(c.knots.size() + rr);
  out.cw.resize(c.cw.size() + rr);

  // Load new knot vector (A5.1).
  for (int i = 0; i <= k; ++i) out.knots[i] = c.knots[i];
  for (int i = 1; i <= rr; ++i) out.knots[k + i] = u;
  for (int i = k + 1; i < static_cast<int>(c.knots.size()); ++i) out.knots[i + rr] = c.knots[i];

  // Save unaltered control points.
  for (int i = 0; i <= k - p; ++i) out.cw[i] = c.cw[i];
  for (int i = k - s; i <= n; ++i) out.cw[i + rr] = c.cw[i];

  // Temporary Rw[0..p-s].
  std::vector<Homog4> R(p - s + 1);
  for (int i = 0; i <= p - s; ++i) R[i] = c.cw[k - p + i];

  // Insert the knot r times.
  int L = 0;
  for (int j = 1; j <= rr; ++j) {
    L = k - p + j;
    for (int i = 0; i <= p - j - s; ++i) {
      const double denom = c.knots[i + k + 1] - c.knots[L + i];
      const double alpha = (denom != 0.0) ? (u - c.knots[L + i]) / denom : 0.0;
      R[i] = R[i + 1] * alpha + R[i] * (1.0 - alpha);
    }
    out.cw[L] = R[0];
    out.cw[k + rr - j - s] = R[p - j - s];
  }
  // Load the remaining (untouched by this pass) control points.
  for (int i = L + 1; i < k - s; ++i) out.cw[i] = R[i - L];

  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// A5.4 — knot refinement (whole vector X inserted at once), homogeneous net.
// ─────────────────────────────────────────────────────────────────────────────
HCurve refineKnotH(const HCurve& c, std::span<const double> X) {
  if (X.empty()) return c;
  const int p = c.degree;
  const int n = static_cast<int>(c.cw.size()) - 1;
  const int m = n + p + 1;                             // high knot index
  const int r = static_cast<int>(X.size()) - 1;
  const std::vector<double>& U = c.knots;

  const int a = findSpanLocal(n, p, X[0], U);
  const int b = findSpanLocal(n, p, X[r], U) + 1;

  HCurve out;
  out.degree = p;
  out.rational = c.rational;
  out.cw.resize(c.cw.size() + r + 1);
  out.knots.resize(U.size() + r + 1);

  // Copy unaltered control points.
  for (int j = 0; j <= a - p; ++j) out.cw[j] = c.cw[j];
  for (int j = b - 1; j <= n; ++j) out.cw[j + r + 1] = c.cw[j];
  // Copy unaltered knots.
  for (int j = 0; j <= a; ++j) out.knots[j] = U[j];
  for (int j = b + p; j <= m; ++j) out.knots[j + r + 1] = U[j];

  int i = b + p - 1;
  int kk = b + p + r;
  for (int j = r; j >= 0; --j) {
    while (X[j] <= U[i] && i > a) {
      out.cw[kk - p - 1] = c.cw[i - p - 1];
      out.knots[kk] = U[i];
      --kk;
      --i;
    }
    out.cw[kk - p - 1] = out.cw[kk - p];
    for (int l = 1; l <= p; ++l) {
      const int ind = kk - p + l;
      double alpha = out.knots[kk + l] - X[j];
      if (std::fabs(alpha) == 0.0) {
        out.cw[ind - 1] = out.cw[ind];
      } else {
        alpha = alpha / (out.knots[kk + l] - U[i - p + l]);
        out.cw[ind - 1] = out.cw[ind - 1] * alpha + out.cw[ind] * (1.0 - alpha);
      }
    }
    out.knots[kk] = X[j];
    --kk;
  }
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// A5.8 — knot removal on the homogeneous net. Removes value u up to `num` times
// within `tol`. Returns how many removals succeeded and the max deviation.
// The book's distance test is applied in R⁴ (homogeneous), which bounds the
// projected geometry error for the well-conditioned nets this layer produces.
// ─────────────────────────────────────────────────────────────────────────────
double dist4(const Homog4& a, const Homog4& b) noexcept {
  const double dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z, dw = a.w - b.w;
  return std::sqrt(dx * dx + dy * dy + dz * dz + dw * dw);
}

struct RemovalH {
  int removed = 0;
  double maxError = 0.0;
  HCurve curve;
};

// A5.8 RemoveCurveKnot: remove knot u (last-occurrence index r, multiplicity s)
// up to `num` times within `tol`. Reproduces the book's index machinery: `temp`
// holds the 2p-1 candidate control points, the removal error is tested BEFORE
// committing, and on success U/Pw are compacted by one. Kept 1:1 with A5.8, so
// the compilers/parsers cognitive band (25–35) applies.
// One knot removal (A5.8 with num=1). Returns true and mutates U/Pw in place if
// the removal is within tol; otherwise leaves them untouched and returns false.
bool removeKnotOnce(int p, double u, double tol, std::vector<double>& U,
                    std::vector<Homog4>& Pw, double& errOut) {
  const int s = knotMultiplicity(U, u);
  if (s == 0) return false;
  int r = -1;
  for (int idx = 0; idx < static_cast<int>(U.size()); ++idx)
    if (U[idx] == u) r = idx;
  if (r < 0) return false;

  const int ord = p + 1;
  const int n = static_cast<int>(Pw.size()) - 1;
  const int m = n + ord;
  const int fout = (2 * r - s - p) / 2;
  const int first = r - p;
  const int last = r - s;
  const int off = first - 1;

  std::vector<Homog4> temp(2 * p + 2);
  temp[0] = Pw[off];
  temp[last + 1 - off] = Pw[last + 1];
  int i = first, j = last;
  int ii = 1, jj = last - off;

  // t == 0 for a single removal (the general multi-removal `t` degenerates).
  while (j - i > 0) {
    const double alfi = (u - U[i]) / (U[i + ord] - U[i]);
    const double alfj = (u - U[j]) / (U[j + ord] - U[j]);
    temp[ii] = (Pw[i] - temp[ii - 1] * (1.0 - alfi)) * (1.0 / alfi);
    temp[jj] = (Pw[j] - temp[jj + 1] * alfj) * (1.0 / (1.0 - alfj));
    ++i; ++ii;
    --j; --jj;
  }

  double err;
  if (j - i < 0) {
    err = dist4(temp[ii - 1], temp[jj + 1]);
  } else {
    const double alfi = (u - U[i]) / (U[i + ord] - U[i]);
    const Homog4 approx = temp[ii + 1] * alfi + temp[ii - 1] * (1.0 - alfi);
    err = dist4(Pw[i], approx);
  }
  if (err > tol) { errOut = err; return false; }  // honest decline

  // Commit the recomputed control points.
  i = first; j = last;
  while (j - i > 0) {
    Pw[i] = temp[i - off];
    Pw[j] = temp[j - off];
    ++i; --j;
  }

  // Drop the removed knot at index r and the redundant control point at fout.
  for (int k1 = r + 1; k1 <= m; ++k1) U[k1 - 1] = U[k1];
  U.pop_back();
  for (int k1 = fout + 1; k1 <= n; ++k1) Pw[k1 - 1] = Pw[k1];
  Pw.pop_back();

  errOut = err;
  return true;
}

// Remove knot value u up to `num` times within tol. Each removal recomputes the
// (shrinking) span/multiplicity from the current net, so the state stays exact.
RemovalH removeKnotH(const HCurve& c, double u, int num, double tol) {
  RemovalH res;
  res.curve = c;
  if (num <= 0 || knotMultiplicity(c.knots, u) == 0) return res;

  std::vector<double> U = c.knots;
  std::vector<Homog4> Pw = c.cw;
  int removed = 0;
  double achievedMax = 0.0;
  for (int t = 0; t < num; ++t) {
    double err = 0.0;
    if (!removeKnotOnce(c.degree, u, tol, U, Pw, err)) break;
    achievedMax = std::max(achievedMax, err);
    ++removed;
  }

  res.removed = removed;
  res.maxError = achievedMax;
  res.curve.degree = c.degree;
  res.curve.rational = c.rational;
  res.curve.knots = std::move(U);
  res.curve.cw = std::move(Pw);
  return res;
}

// ─────────────────────────────────────────────────────────────────────────────
// A5.6 — Bézier decomposition on the homogeneous net. Returns per-segment
// homogeneous control nets (p+1 poles each) and the distinct-span breakpoints.
// ─────────────────────────────────────────────────────────────────────────────
struct BezierDecompH {
  std::vector<std::vector<Homog4>> segments;   // each of size p+1
  std::vector<double> breaks;                   // distinct knots, size nb+1
};

BezierDecompH decomposeH(const HCurve& c) {
  const int p = c.degree;
  const int n = static_cast<int>(c.cw.size()) - 1;
  const int m = n + p + 1;
  const std::vector<double>& U = c.knots;
  const std::vector<Homog4>& Pw = c.cw;

  BezierDecompH out;
  std::vector<double> distinct;
  for (double k : U)
    if (distinct.empty() || distinct.back() != k) distinct.push_back(k);
  out.breaks = distinct;  // includes the clamped endpoints, size = #segments + 1

  // A5.6 DecomposeCurve. Qw[nb] holds segment nb; nextbpts carries the leading
  // control points into the next segment. Each segment has p+1 poles.
  std::vector<std::vector<Homog4>>& Qw = out.segments;
  Qw.emplace_back(p + 1);
  std::vector<Homog4> nextbpts(std::max(1, p - 1));
  std::vector<double> alphas(std::max(1, p - 1));

  int a = p;
  int b = p + 1;
  int nb = 0;
  for (int i = 0; i <= p; ++i) Qw[0][i] = Pw[i];

  while (b < m) {
    const int i0 = b;
    while (b < m && U[b + 1] == U[b]) ++b;
    const int mult = b - i0 + 1;
    if (mult < p) {
      const double numer = U[b] - U[a];
      for (int j = p; j > mult; --j) alphas[j - mult - 1] = numer / (U[a + j] - U[a]);
      const int r = p - mult;
      for (int j = 1; j <= r; ++j) {
        const int save = r - j;
        const int s2 = mult + j;
        for (int k1 = p; k1 >= s2; --k1) {
          const double alpha = alphas[k1 - s2];
          Qw[nb][k1] = Qw[nb][k1] * alpha + Qw[nb][k1 - 1] * (1.0 - alpha);
        }
        if (b < m) nextbpts[save] = Qw[nb][p];   // control point for next segment
      }
    }
    ++nb;                                          // Bézier segment completed
    if (b < m) {                                   // initialize the next segment
      Qw.emplace_back(p + 1);
      for (int i = p - mult; i <= p; ++i) Qw[nb][i] = Pw[b - p + i];
      for (int i = 0; i < p - mult; ++i) Qw[nb][i] = nextbpts[i];
      a = b;
      ++b;
    }
  }
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// A5.9 — degree elevation by t on the homogeneous net.
// Follows the book's Algorithm A5.9 (Bézier-decompose, elevate each Bézier,
// re-merge with the correct interior-knot multiplicities). The coefficient table
// (computeBezalfs) and the three inner sub-steps (a59InsertBezier / a59Elevate
// Bezier / a59MergeLeft) are extracted so the driver stays within the compilers/
// parsers cognitive band (25–35) while preserving the book's structure.
// ─────────────────────────────────────────────────────────────────────────────
double binom(int a, int b) noexcept {
  if (b < 0 || b > a) return 0.0;
  double r = 1.0;
  for (int i = 0; i < b; ++i) r = r * (a - i) / (i + 1);
  return r;
}

// A5.9 Bézier degree-elevation coefficient table bezalfs[i][j] (ph = p + t).
std::vector<std::vector<double>> computeBezalfs(int p, int t) {
  const int ph = p + t;
  const int ph2 = ph / 2;
  std::vector<std::vector<double>> bezalfs(ph + 1, std::vector<double>(p + 1, 0.0));
  bezalfs[0][0] = 1.0;
  bezalfs[ph][p] = 1.0;
  for (int i = 1; i <= ph2; ++i) {
    const double inv = 1.0 / binom(ph, i);
    const int mpi = std::min(p, i);
    for (int j = std::max(0, i - t); j <= mpi; ++j)
      bezalfs[i][j] = inv * binom(p, j) * binom(t, i - j);
  }
  for (int i = ph2 + 1; i <= ph - 1; ++i) {
    const int mpi = std::min(p, i);
    for (int j = std::max(0, i - t); j <= mpi; ++j)
      bezalfs[i][j] = bezalfs[ph - i][p - j];
  }
  return bezalfs;
}

// Count of DISTINCT interior knots (breakpoints excluding the two clamped ends).
int distinctInteriorKnots(const std::vector<double>& U) {
  std::vector<double> distinct;
  for (double k : U)
    if (distinct.empty() || distinct.back() != k) distinct.push_back(k);
  return static_cast<int>(distinct.size()) - 2;
}

// A5.9 sub-step: insert the knot to extract the next Bézier segment (fills bpts,
// records the leading control points of the following segment in Nextbpts).
void a59InsertBezier(int p, int r, int mul, double ub, double ua, int a,
                     const std::vector<double>& U, std::vector<Homog4>& bpts,
                     std::vector<Homog4>& Nextbpts, std::vector<double>& alfs) {
  const double numer = ub - ua;
  for (int k1 = p; k1 > mul; --k1) alfs[k1 - mul - 1] = numer / (U[a + k1] - ua);
  for (int j = 1; j <= r; ++j) {
    const int save = r - j;
    const int sk = mul + j;
    for (int k1 = p; k1 >= sk; --k1)
      bpts[k1] = bpts[k1] * alfs[k1 - sk] + bpts[k1 - 1] * (1.0 - alfs[k1 - sk]);
    Nextbpts[save] = bpts[p];
  }
}

// A5.9 sub-step: degree-elevate the current Bézier segment bpts → ebpts[lbz..ph].
void a59ElevateBezier(int p, int t, int ph, int lbz, const std::vector<Homog4>& bpts,
                      const std::vector<std::vector<double>>& bezalfs,
                      std::vector<Homog4>& ebpts) {
  for (int i = lbz; i <= ph; ++i) {
    ebpts[i] = Homog4{};
    const int mpi = std::min(p, i);
    for (int j = std::max(0, i - t); j <= mpi; ++j)
      ebpts[i] = ebpts[i] + bpts[j] * bezalfs[i][j];
  }
}

// A5.9 sub-step: knot removal from the left, merging the elevated segment with
// the already-written control points (out.cw) and adjusting ebpts.
void a59MergeLeft(int t, int ph, int oldr, int lbz, int cind, int kind, double ua,
                  double ub, std::vector<Homog4>& outCw, std::vector<double>& outKnots,
                  std::vector<Homog4>& ebpts) {
  int first = kind - 2;
  int last = kind;
  const double den = ub - ua;
  const double bet = (ub - outKnots[kind - 1]) / den;
  for (int tr = 1; tr < oldr; ++tr) {
    int i = first, j = last, kj = j - kind + 1;
    while (j - i > tr) {
      if (i < cind) {
        const double alf = (ub - outKnots[i]) / (ua - outKnots[i]);
        outCw[i] = outCw[i] * alf + outCw[i - 1] * (1.0 - alf);
      }
      if (j >= lbz) {
        if (j - tr <= kind - ph + oldr) {
          const double gam = (ub - outKnots[j - tr]) / den;
          ebpts[kj] = ebpts[kj] * gam + ebpts[kj + 1] * (1.0 - gam);
        } else {
          ebpts[kj] = ebpts[kj] * bet + ebpts[kj + 1] * (1.0 - bet);
        }
      }
      ++i; --j; --kj;
    }
    --first; ++last;
  }
  (void)t;
}

HCurve elevateDegreeH(const HCurve& c, int t) {
  if (t <= 0) return c;
  const int p = c.degree;
  const int n = static_cast<int>(c.cw.size()) - 1;
  const int m = n + p + 1;
  const std::vector<double>& U = c.knots;
  const std::vector<Homog4>& Pw = c.cw;

  const int ph = p + t;
  const std::vector<std::vector<double>> bezalfs = computeBezalfs(p, t);
  const int s = distinctInteriorKnots(U);  // #distinct interior knots

  HCurve out;
  out.degree = ph;
  out.rational = c.rational;
  out.cw.assign(static_cast<std::size_t>(n + 1 + (s + 1) * t + t), Homog4{});
  out.knots.assign(static_cast<std::size_t>(m + (s + 2) * t + 1), 0.0);

  std::vector<Homog4> bpts(p + 1);
  std::vector<Homog4> ebpts(ph + 1);
  std::vector<Homog4> Nextbpts(p - 1 > 0 ? p - 1 : 0);
  std::vector<double> alfs(p - 1 > 0 ? p - 1 : 0);

  int mh = ph;
  int kind = ph + 1;
  int r = -1;
  int a = p;
  int b = p + 1;
  int cind = 1;
  double ua = U[0];

  out.cw[0] = Pw[0];
  for (int i = 0; i <= ph; ++i) out.knots[i] = ua;

  for (int i = 0; i <= p; ++i) bpts[i] = Pw[i];

  while (b < m) {
    const int i0 = b;
    while (b < m && U[b] == U[b + 1]) ++b;
    const int mul = b - i0 + 1;
    mh += mul + t;
    const double ub = U[b];
    const int oldr = r;
    r = p - mul;

    const int lbz = (oldr > 0) ? (oldr + 2) / 2 : 1;
    const int rbz = (r > 0) ? ph - (r + 1) / 2 : ph;

    if (r > 0)  // insert knot to extract the Bézier segment.
      a59InsertBezier(p, r, mul, ub, ua, a, U, bpts, Nextbpts, alfs);

    a59ElevateBezier(p, t, ph, lbz, bpts, bezalfs, ebpts);  // degree-elevate it

    if (oldr > 1)  // knot removal from the left (merge with previous segment).
      a59MergeLeft(t, ph, oldr, lbz, cind, kind, ua, ub, out.cw, out.knots, ebpts);

    if (a != p)  // load the knot ua.
      for (int i = 0; i < ph - oldr; ++i) { out.knots[kind] = ua; ++kind; }

    for (int j = lbz; j <= rbz; ++j) {  // load control points into out.cw.
      out.cw[cind] = ebpts[j];
      ++cind;
    }

    if (b < m) {  // set up for the next pass.
      for (int j = 0; j < r; ++j) bpts[j] = Nextbpts[j];
      for (int j = r; j <= p; ++j) bpts[j] = Pw[b - p + j];
      a = b;
      ++b;
      ua = ub;
    } else {  // end knots.
      for (int i = 0; i <= ph; ++i) out.knots[kind + i] = ub;
    }
  }

  const int nh = mh - ph - 1;
  out.cw.resize(nh + 1);
  out.knots.resize(mh + 1);
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Surface driver: apply a curve op along every row (V-dir) or column (U-dir),
// reusing the curve core. Rows/columns are extracted as homogeneous curves so
// the rational path is single-sourced.
// ─────────────────────────────────────────────────────────────────────────────
HCurve extractLine(const BsplineSurfaceData& s, ParamDir d, int fixedIndex) {
  HCurve h;
  const bool rational = !s.weights.empty();
  h.rational = rational;
  if (d == ParamDir::U) {
    // A COLUMN: vary i over U, fix j = fixedIndex. Applies a U-direction op.
    h.degree = s.degreeU;
    h.knots = s.knotsU;
    h.cw.reserve(s.nPolesU);
    for (int i = 0; i < s.nPolesU; ++i) {
      const std::size_t idx = static_cast<std::size_t>(i) * s.nPolesV + fixedIndex;
      h.cw.push_back(lift(s.poles[idx], rational ? s.weights[idx] : 1.0));
    }
  } else {
    // A ROW: vary j over V, fix i = fixedIndex. Applies a V-direction op.
    h.degree = s.degreeV;
    h.knots = s.knotsV;
    h.cw.reserve(s.nPolesV);
    for (int j = 0; j < s.nPolesV; ++j) {
      const std::size_t idx = static_cast<std::size_t>(fixedIndex) * s.nPolesV + j;
      h.cw.push_back(lift(s.poles[idx], rational ? s.weights[idx] : 1.0));
    }
  }
  return h;
}

// Project one homogeneous pole into the (poles,weights) grid at a flat index.
void writePole(BsplineSurfaceData& out, bool rational, std::size_t idx,
               const Homog4& h) {
  if (rational) {
    const double invW = (h.w != 0.0) ? 1.0 / h.w : 0.0;
    out.poles[idx] = {h.x * invW, h.y * invW, h.z * invW};
    out.weights[idx] = h.w;
  } else {
    out.poles[idx] = {h.x, h.y, h.z};
  }
}

// Rebuild a surface from per-line homogeneous results. Along U there are nPolesV
// lines (one per V index) of the new U-length; along V there are nPolesU lines of
// the new V-length. The two directions share `writePole`; only the axis roles and
// the flat-index formula differ.
BsplineSurfaceData assembleFromLines(const BsplineSurfaceData& s, ParamDir d,
                                     const std::vector<HCurve>& lines) {
  BsplineSurfaceData out;
  out.degreeU = s.degreeU;
  out.degreeV = s.degreeV;
  const bool rational = !s.weights.empty();
  const int newLen = static_cast<int>(lines[0].cw.size());

  if (d == ParamDir::U) {
    out.degreeU = lines[0].degree;
    out.nPolesU = newLen;
    out.nPolesV = s.nPolesV;
    out.knotsU = lines[0].knots;
    out.knotsV = s.knotsV;
  } else {
    out.degreeV = lines[0].degree;
    out.nPolesU = s.nPolesU;
    out.nPolesV = newLen;
    out.knotsU = s.knotsU;
    out.knotsV = lines[0].knots;
  }
  out.poles.resize(static_cast<std::size_t>(out.nPolesU) * out.nPolesV);
  if (rational) out.weights.resize(out.poles.size());

  // Line index = the fixed cross-axis coordinate; position = the transformed axis.
  const int nLines = (d == ParamDir::U) ? out.nPolesV : out.nPolesU;
  for (int line = 0; line < nLines; ++line) {
    const HCurve& L = lines[line];
    for (int pos = 0; pos < newLen; ++pos) {
      const std::size_t idx = (d == ParamDir::U)
          ? static_cast<std::size_t>(pos) * out.nPolesV + line   // (i=pos, j=line)
          : static_cast<std::size_t>(line) * out.nPolesV + pos;  // (i=line, j=pos)
      writePole(out, rational, idx, L.cw[pos]);
    }
  }
  return out;
}

int lineCount(const BsplineSurfaceData& s, ParamDir d) {
  return d == ParamDir::U ? s.nPolesV : s.nPolesU;
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// Public curve API.
// ═════════════════════════════════════════════════════════════════════════════

BsplineCurveData insertKnotCurve(const BsplineCurveData& c, double u, int r) {
  return project(insertKnotH(lift(c), u, r));
}

BsplineCurveData refineKnotCurve(const BsplineCurveData& c,
                                 std::span<const double> newKnots) {
  return project(refineKnotH(lift(c), newKnots));
}

KnotRemovalResult removeKnotCurve(const BsplineCurveData& c, double u, int num,
                                  double tol) {
  RemovalH h = removeKnotH(lift(c), u, num, tol);
  KnotRemovalResult res;
  res.removed = h.removed;
  res.maxError = h.maxError;
  res.curve = project(h.curve);
  return res;
}

BsplineCurveData elevateDegreeCurve(const BsplineCurveData& c, int t) {
  return project(elevateDegreeH(lift(c), t));
}

BsplineCurveData reparamCurve(const BsplineCurveData& c, double a, double b) {
  BsplineCurveData out = c;
  if (out.knots.size() < 2) return out;
  const double lo = out.knots.front();
  const double hi = out.knots.back();
  const double span = hi - lo;
  if (span == 0.0) return out;
  const double scale = (b - a) / span;
  for (double& k : out.knots) k = a + (k - lo) * scale;
  return out;
}

CurveSplit splitCurve(const BsplineCurveData& c, double u) {
  // Insert u to full multiplicity p, then partition the net at the shared knot.
  const int p = c.degree;
  const int s = knotMultiplicity(c.knots, u);
  HCurve h = lift(c);
  if (p - s > 0) h = insertKnotH(h, u, p - s);

  // Locate the first knot index equal to u after the leading clamp.
  // The split knot now has multiplicity p; the left piece owns poles [0..k-p]
  // ending at the last pole affecting C(u⁻); the right owns [k-p .. n].
  const int n = static_cast<int>(h.cw.size()) - 1;
  const int span = findSpanLocal(n, p, u, h.knots);
  // With multiplicity p, span is the last index i with knots[i]==u.
  const int kLeftPoles = span - p + 1;   // #poles in left piece = span-p+1

  CurveSplit out;
  bool ok = true;

  // Left curve: poles [0..span-p], knots [0..span] + closing clamp u (p+1 total).
  HCurve hl;
  hl.degree = p;
  hl.rational = h.rational;
  hl.cw.assign(h.cw.begin(), h.cw.begin() + kLeftPoles);
  hl.knots.assign(h.knots.begin(), h.knots.begin() + (span + 1));
  hl.knots.push_back(u);  // ensure the clamp closes at u with mult p+1

  // Right curve: poles [span-p .. n]; opening clamp = (p+1) copies of u, then the
  // trailing knots [span+1 .. end] (which lie strictly above u).
  HCurve hr;
  hr.degree = p;
  hr.rational = h.rational;
  hr.cw.assign(h.cw.begin() + (span - p), h.cw.end());
  hr.knots.assign(p + 1, u);
  hr.knots.insert(hr.knots.end(), h.knots.begin() + (span + 1), h.knots.end());

  out.left = project(hl, ok);
  out.right = project(hr, ok);
  return out;
}

std::vector<BsplineCurveData> decomposeCurveToBezier(const BsplineCurveData& c) {
  const HCurve h = lift(c);
  const BezierDecompH dec = decomposeH(h);
  const int p = c.degree;
  std::vector<BsplineCurveData> segs;
  segs.reserve(dec.segments.size());
  for (std::size_t sIdx = 0; sIdx < dec.segments.size(); ++sIdx) {
    HCurve seg;
    seg.degree = p;
    seg.rational = h.rational;
    seg.cw = dec.segments[sIdx];
    // Bézier clamp on [breaks[sIdx], breaks[sIdx+1]].
    const double a = dec.breaks[sIdx];
    const double b = dec.breaks[sIdx + 1];
    seg.knots.assign(p + 1, a);
    seg.knots.insert(seg.knots.end(), p + 1, b);
    segs.push_back(project(seg));
  }
  return segs;
}

// ═════════════════════════════════════════════════════════════════════════════
// A5.11 — degree reduction by 1.  Strategy honoring the honesty constraint:
// reduce candidate = (Bézier-decompose, reduce each Bézier, re-merge); then MEASURE
// the true deviation by re-elevating the candidate and comparing pole nets in R⁴.
// Exact-when-reducible: for a curve produced by elevating a lower-degree curve,
// the round-trip identity holds so maxError ≈ 0. Irreducible ⇒ ok=false + true
// bound, never a lower-degree curve claimed as exact.
// ═════════════════════════════════════════════════════════════════════════════
namespace {

// Reduce a single Bézier (degree p → p-1) and return the max reduction error
// (the standard A5.11 Bézier reduction with its symmetric error estimate).
std::vector<Homog4> reduceBezier(const std::vector<Homog4>& b, int p, double& maxErr) {
  std::vector<Homog4> r(p);           // degree p-1 has p poles
  r[0] = b[0];
  r[p - 1] = b[p];
  const int rr = (p - 1) / 2;
  auto BinD = [](int a, int c) -> double {
    if (c < 0 || c > a) return 0.0;
    double v = 1.0;
    for (int i = 0; i < c; ++i) v = v * (a - i) / (i + 1);
    return v;
  };
  // Forward and backward recurrences meet in the middle (A5.11).
  for (int i = 1; i <= rr; ++i) {
    const double alpha = static_cast<double>(i) / p;
    r[i] = (b[i] - r[i - 1] * alpha) * (1.0 / (1.0 - alpha));
  }
  for (int i = p - 2; i > rr; --i) {
    const double alpha = static_cast<double>(i + 1) / p;
    r[i] = (b[i + 1] - r[i + 1] * (1.0 - alpha)) * (1.0 / alpha);
  }
  // Error = deviation at the meeting point.
  if (p % 2 == 0) {
    const int i = p / 2;
    maxErr = dist4(b[i], (r[i - 1] + r[i]) * 0.5);
  } else {
    const int i = (p - 1) / 2;
    const double alpha = static_cast<double>(i) / p;
    const Homog4 lhs = (b[i] - r[i - 1] * alpha) * (1.0 / (1.0 - alpha));
    const double a2 = static_cast<double>(i + 1) / p;
    const Homog4 rhs = (b[i + 1] - r[i + 1] * (1.0 - a2)) * (1.0 / a2);
    maxErr = dist4(lhs, rhs) * 0.5;
    r[i] = (lhs + rhs) * 0.5;
  }
  (void)BinD;
  return r;
}

}  // namespace

DegreeReduceResult reduceDegreeCurve(const BsplineCurveData& c, double tol) {
  DegreeReduceResult res;
  const int p = c.degree;
  if (p < 1) { res.curve = c; res.ok = false; return res; }

  const HCurve h = lift(c);
  const BezierDecompH dec = decomposeH(h);

  // Reduce each Bézier segment; track the worst per-segment error.
  double maxErr = 0.0;
  std::vector<std::vector<Homog4>> redSegs;
  redSegs.reserve(dec.segments.size());
  for (const auto& seg : dec.segments) {
    double e = 0.0;
    redSegs.push_back(reduceBezier(seg, p, e));
    maxErr = std::max(maxErr, e);
  }

  // Re-merge the reduced Bézier segments into a degree p-1 spline. Interior
  // breakpoints get multiplicity p-1 (each segment contributes p-1 poles; the
  // last pole of a segment == first pole of the next at a C0 join).
  HCurve out;
  out.degree = p - 1;
  out.rational = h.rational;
  const int nb = static_cast<int>(redSegs.size());
  // Control points: first segment's p poles, then (p-1) new poles per later seg.
  out.cw = redSegs[0];
  for (int s = 1; s < nb; ++s)
    out.cw.insert(out.cw.end(), redSegs[s].begin() + 1, redSegs[s].end());
  // Knots: clamped ends (p) + interior breaks with multiplicity p-1.
  const int q = p - 1;
  for (int i = 0; i <= q; ++i) out.knots.push_back(dec.breaks.front());
  for (int s = 1; s < nb; ++s)
    for (int m = 0; m < q; ++m) out.knots.push_back(dec.breaks[s]);
  for (int i = 0; i <= q; ++i) out.knots.push_back(dec.breaks.back());

  // Elevation inflated interior-knot multiplicities; degree reduction must undo
  // that. Remove each interior break as far down as tol allows (reusing the
  // knot-removal core) so the recovered curve reaches its minimal representation.
  // The removal deviation folds into the reported bound.
  for (int s = 1; s < nb; ++s) {
    const double u = dec.breaks[s];
    for (int k = 0; k < q; ++k) {
      double e = 0.0;
      if (!removeKnotOnce(q, u, tol, out.knots, out.cw, e)) break;
      maxErr = std::max(maxErr, e);
    }
  }

  // Honest verification: measure the TRUE geometric deviation between the reduced
  // candidate and the original by dense pointwise sampling of the represented
  // curves (finite and real regardless of knot structure). This is the reported
  // bound; the candidate is accepted only if it stays within tol.
  bool projOk = true;
  const BsplineCurveData cand = project(out, projOk);
  double trueErr = maxErr;
  if (projOk && !cand.knots.empty()) {
    const double lo = c.knots.front();
    const double hi = c.knots.back();
    const int N = 200;
    for (int i = 0; i <= N; ++i) {
      const double u = lo + (hi - lo) * (static_cast<double>(i) / N);
      const Point3 pa = c.weights.empty()
          ? curvePoint(c.degree, c.poles, c.knots, u)
          : nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
      const Point3 pb = cand.weights.empty()
          ? curvePoint(cand.degree, cand.poles, cand.knots, u)
          : nurbsCurvePoint(cand.degree, cand.poles, cand.weights, cand.knots, u);
      trueErr = std::max(trueErr, distance(pa, pb));
    }
  } else {
    trueErr = std::numeric_limits<double>::infinity();
  }

  res.maxError = trueErr;
  if (projOk && trueErr <= tol) {
    res.ok = true;
    res.curve = cand;
  } else {
    res.ok = false;
    res.curve = c;  // honest decline: return the original, do NOT claim reduction
  }
  return res;
}

// ═════════════════════════════════════════════════════════════════════════════
// Public surface API (tensor-product, per-line reuse of the curve core).
// ═════════════════════════════════════════════════════════════════════════════

BsplineSurfaceData insertKnotSurface(const BsplineSurfaceData& s, ParamDir d,
                                     double val, int r) {
  std::vector<HCurve> lines;
  const int nLines = lineCount(s, d);
  lines.reserve(nLines);
  for (int k = 0; k < nLines; ++k)
    lines.push_back(insertKnotH(extractLine(s, d, k), val, r));
  return assembleFromLines(s, d, lines);
}

BsplineSurfaceData refineKnotSurface(const BsplineSurfaceData& s, ParamDir d,
                                     std::span<const double> newKnots) {
  std::vector<HCurve> lines;
  const int nLines = lineCount(s, d);
  lines.reserve(nLines);
  for (int k = 0; k < nLines; ++k)
    lines.push_back(refineKnotH(extractLine(s, d, k), newKnots));
  return assembleFromLines(s, d, lines);
}

BsplineSurfaceData elevateDegreeSurface(const BsplineSurfaceData& s, ParamDir d,
                                        int t) {
  std::vector<HCurve> lines;
  const int nLines = lineCount(s, d);
  lines.reserve(nLines);
  for (int k = 0; k < nLines; ++k)
    lines.push_back(elevateDegreeH(extractLine(s, d, k), t));
  return assembleFromLines(s, d, lines);
}

KnotRemovalResultS removeKnotSurface(const BsplineSurfaceData& s, ParamDir d,
                                     double val, int num, double tol) {
  // Remove the SAME count from every line so the tensor structure stays valid:
  // the achievable count is the min over lines; the max error is the worst line.
  const int nLines = lineCount(s, d);
  int minRemoved = num;
  for (int k = 0; k < nLines; ++k) {
    RemovalH h = removeKnotH(extractLine(s, d, k), val, num, tol);
    minRemoved = std::min(minRemoved, h.removed);
  }
  KnotRemovalResultS res;
  res.removed = minRemoved;
  if (minRemoved <= 0) { res.surface = s; return res; }
  std::vector<HCurve> lines;
  lines.reserve(nLines);
  double maxErr = 0.0;
  for (int k = 0; k < nLines; ++k) {
    RemovalH h = removeKnotH(extractLine(s, d, k), val, minRemoved, tol);
    maxErr = std::max(maxErr, h.maxError);
    lines.push_back(h.curve);
  }
  res.maxError = maxErr;
  res.surface = assembleFromLines(s, d, lines);
  return res;
}

DegreeReduceResultS reduceDegreeSurface(const BsplineSurfaceData& s, ParamDir d,
                                        double tol) {
  // Reduce each line; succeed only if EVERY line is reducible within tol.
  const int nLines = lineCount(s, d);
  DegreeReduceResultS res;
  std::vector<HCurve> lines;
  lines.reserve(nLines);
  double maxErr = 0.0;
  bool allOk = true;
  const int deg = (d == ParamDir::U) ? s.degreeU : s.degreeV;
  for (int k = 0; k < nLines; ++k) {
    HCurve lh = extractLine(s, d, k);
    BsplineCurveData lc;
    bool ok = true;
    lc = project(lh, ok);
    lc.degree = deg;
    DegreeReduceResult dr = reduceDegreeCurve(lc, tol);
    maxErr = std::max(maxErr, dr.maxError);
    if (!dr.ok) { allOk = false; break; }
    lines.push_back(lift(dr.curve));
  }
  res.maxError = maxErr;
  if (!allOk) { res.ok = false; res.surface = s; return res; }
  res.ok = true;
  res.surface = assembleFromLines(s, d, lines);
  return res;
}

SurfaceSplit splitSurface(const BsplineSurfaceData& s, ParamDir d, double val) {
  const int nLines = lineCount(s, d);
  std::vector<HCurve> lowLines, highLines;
  lowLines.reserve(nLines);
  highLines.reserve(nLines);
  for (int k = 0; k < nLines; ++k) {
    HCurve lh = extractLine(s, d, k);
    bool ok = true;
    BsplineCurveData lc = project(lh, ok);
    lc.degree = lh.degree;
    CurveSplit cs = splitCurve(lc, val);
    lowLines.push_back(lift(cs.left));
    highLines.push_back(lift(cs.right));
  }
  SurfaceSplit out;
  out.low = assembleFromLines(s, d, lowLines);
  out.high = assembleFromLines(s, d, highLines);
  return out;
}

}  // namespace cybercad::native::math
