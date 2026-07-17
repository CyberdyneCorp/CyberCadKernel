// oracle_graze_threeway.cpp — INDEPENDENT ultra-high-precision three-way intersection
// oracle for the SSI near-tangent-graze moat (track SSI-GRAZE).
//
// This is the ONE untried legitimate angle on the PARKED near-tangent curve-divergence
// moat. Three prior adversarial waves (SSI-SMALLLOOP, SSI-MARCH, SSI-TERM) established the
// decline is NOT a seeding gap, NOT marching arc-error, NOT premature termination. At a
// near-tangent graze pinch, native and OCCT each converge to their OWN on-surface root
// differing ~1.8e-3 laterally, just over the fixed onCurve=1e-3 gate. The question left
// open: which root (if either) is closer to the TRUE intersection curve?
//
// METHOD — a THIRD reference independent of both native and OCCT:
//   * Reproduces the EXACT splitmix64→xoshiro256** RNG + surface generator of
//     tests/sim/native_ssi_freeform_fuzz.mm / tests/native/replay_freeform_seed.cpp
//     (verbatim), so it lands on the identical NURBS pose for any (seed, si, idx).
//   * Reconstructs the EXACT NURBS surfaces (native-math surfacePoint / nurbsSurfacePoint)
//     and — crucially — their EXACT ANALYTIC first derivatives (surfaceDerivs /
//     nurbsSurfaceDerivs, Piegl A3.6 / A4.4). No finite differences.
//   * A hand-rolled damped-Newton foot-point projector (its own 4×4 linear solve; NO
//     native-numerics, NO OCCT — a genuinely independent solver) drives, from a REFERENCE
//     world point P_ref (a native or OCCT root), to the TRUE intersection-curve point:
//       find (u1,v1,u2,v2) minimizing ‖A(u1,v1) − B(u2,v2)‖ SUBJECT to the curve MIDPOINT
//       M=(A+B)/2 being the FOOT OF PERPENDICULAR from P_ref (M−P_ref ⟂ curve tangent).
//     The two "on both surfaces" constraints (A−B = 0, 3 eqs collapsed to a 2-DOF surface-
//     pair residual) plus the one foot-of-perpendicular constraint pin all 4 DOFs with a
//     WELL-CONDITIONED Jacobian even where nA×nB → 0 (the perpendicular-foot constraint
//     uses the curve tangent direction only to define the transverse plane; it does NOT
//     divide by the transversality sine the way native's along-t advance residual does).
//     Iterated to ‖A−B‖ ≤ 1e-13·scale and ‖step‖ ≤ 1e-14 — FAR below the tracer's working
//     onSurfTol (scale·1e-7) — so its converged point is the true curve point to ~machine ε.
//
// THREE DISTANCES reported at the pinch node:
//   native-root → true-curve   (foot from the native pinch node)
//   OCCT-root   → true-curve   (foot from the OCCT root, when supplied via ORACLE_OCCT_*)
//   native-root → OCCT-root    (the ~1.8e-3 already known, echoed for cross-check)
// plus the LOCAL CONDITIONING at the true point: transversality sine ‖n̂A×n̂B‖, the
// normal-normal angle, and the 4×4 corrector Jacobian condition number (how ill-posed the
// two-surface Newton is there) — the numbers that decide fixable-deficit vs ill-conditioned-
// fuzzy-band.
//
// Usage:
//   oracle_graze_threeway <baseSeedHex> <si> <idx>
//     [--ref  u1 v1 u2 v2]        reference params on both surfaces (native pinch node)
//     [--refP x y z]              reference world point (alternative to --ref)
//     [--occt u1 v1 u2 v2]        OCCT root params on both surfaces (for OCCT→truth)
//     [--occtP x y z]             OCCT root world point (alternative)
//   Env ORACLE_OCCT_P="x y z" also supplies the OCCT world point.
//
// Build: host, OCCT-FREE, links libcybercadkernel.a (native-math only) — no numsci needed
// for the oracle's own solver (numsci is linked only because the kernel archive references
// it; the oracle itself calls no least_squares).
//
// SPDX-License-Identifier: Apache-2.0

#include "native/math/bspline.h"
#include "native/math/vec.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <vector>

namespace math = cybercad::native::math;
using math::Dir3;
using math::Point3;
using math::Vec3;

// ── RNG (verbatim from native_ssi_freeform_fuzz.mm / replay_freeform_seed.cpp) ────────
struct Rng {
  uint64_t s[4];
  static uint64_t splitmix64(uint64_t& x) {
    uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  explicit Rng(uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
  static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  uint64_t next() {
    const uint64_t r = rotl(s[1] * 5, 7) * 9;
    const uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return r;
  }
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  int irange(int lo, int hi) { return lo + static_cast<int>(next() % static_cast<uint64_t>(hi - lo + 1)); }
};

enum Family { F_TRANSVERSAL, F_TILTED, F_MULTIBRANCH, F_NEAR_TANGENT, F_DISJOINT, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_TRANSVERSAL:  return "transversal";
    case F_TILTED:       return "tilted-sheets";
    case F_MULTIBRANCH:  return "multi-branch";
    case F_NEAR_TANGENT: return "near-tangent";
    case F_DISJOINT:     return "disjoint";
  }
  return "?";
}

struct NurbsSurf {
  int degU = 3, degV = 3;
  int nU = 4, nV = 4;
  std::vector<Point3> poles;
  std::vector<double> weights;
  std::vector<double> knotsU, knotsV;
  bool rational() const { return !weights.empty(); }
};

std::vector<double> clampedFlat(int degree, int nPoles) {
  const int m = nPoles + degree + 1;
  const int interior = nPoles - degree - 1;
  std::vector<double> flat; flat.reserve(static_cast<std::size_t>(m));
  for (int i = 0; i <= degree; ++i) flat.push_back(0.0);
  for (int i = 1; i <= interior; ++i) flat.push_back(double(i) / double(interior + 1));
  for (int i = 0; i <= degree; ++i) flat.push_back(1.0);
  return flat;
}

void maybeRational(NurbsSurf& s, Rng& rng, double p) {
  if (rng.unit() >= p) return;
  s.weights.resize(static_cast<std::size_t>(s.nU) * s.nV);
  for (auto& w : s.weights) w = rng.range(0.4, 2.5);
}

template <class ZBase>
void fillGrid(NurbsSurf& s, Rng& rng, ZBase zbase, double jitter, double xyWobble) {
  s.poles.resize(static_cast<std::size_t>(s.nU) * s.nV);
  for (int i = 0; i < s.nU; ++i)
    for (int j = 0; j < s.nV; ++j) {
      const double fx = (s.nU == 1) ? 0.5 : double(i) / (s.nU - 1);
      const double fy = (s.nV == 1) ? 0.5 : double(j) / (s.nV - 1);
      double x = -1.2 + 2.4 * fx + rng.range(-xyWobble, xyWobble);
      double y = -1.2 + 2.4 * fy + rng.range(-xyWobble, xyWobble);
      double z = zbase(x, y) + rng.range(-jitter, jitter);
      s.poles[static_cast<std::size_t>(i) * s.nV + j] = Point3{x, y, z};
    }
  s.knotsU = clampedFlat(s.degU, s.nU);
  s.knotsV = clampedFlat(s.degV, s.nV);
}

bool buildPair(int family, Rng& rng, NurbsSurf& A, NurbsSurf& B) {
  auto randDeg = [&]() { return rng.irange(2, 3); };
  auto randN = [&](int deg) { return rng.irange(deg + 1, 6); };
  A.degU = randDeg(); A.degV = randDeg();
  A.nU = randN(A.degU); A.nV = randN(A.degV);
  B.degU = randDeg(); B.degV = randDeg();
  B.nU = randN(B.degU); B.nV = randN(B.degV);
  const double ampA = rng.range(0.6, 1.4);
  const double ampB = rng.range(0.6, 1.4);
  switch (family) {
    case F_TRANSVERSAL:
      fillGrid(A, rng, [ampA](double x, double y) { return ampA * (1.0 - 0.5 * (x * x + y * y)); }, 0.12, 0.06);
      fillGrid(B, rng, [ampB](double x, double y) { return 0.9 - ampB * (1.0 - 0.5 * (x * x + y * y)); }, 0.12, 0.06);
      break;
    case F_TILTED: {
      const double sxA = rng.range(-0.6, 0.6), syA = rng.range(-0.6, 0.6);
      const double sxB = rng.range(-0.6, 0.6), syB = rng.range(-0.6, 0.6);
      fillGrid(A, rng, [sxA, syA, ampA](double x, double y) { return sxA * x + syA * y + 0.25 * ampA * std::sin(1.3 * x); }, 0.12, 0.07);
      fillGrid(B, rng, [sxB, syB, ampB](double x, double y) { return 0.15 + sxB * x + syB * y + 0.25 * ampB * std::cos(1.3 * y); }, 0.12, 0.07);
      break;
    }
    case F_MULTIBRANCH: {
      const double f = rng.range(1.6, 2.4);
      fillGrid(A, rng, [f, ampA](double x, double y) { return ampA * 0.5 * std::sin(f * x) * std::sin(f * y); }, 0.05, 0.04);
      fillGrid(B, rng, [ampB](double x, double y) { return 0.0 + 0.08 * ampB * (x * x - y * y); }, 0.05, 0.04);
      break;
    }
    case F_NEAR_TANGENT: {
      const bool tight = rng.unit() < 0.5;
      const double k = rng.range(0.35, 0.55);
      const double dk = tight ? rng.range(0.015, 0.05) : rng.range(0.05, 0.12);
      const double overlap = tight ? rng.range(0.008, 0.03) : rng.range(0.03, 0.10);
      fillGrid(A, rng, [k](double x, double y) { return k * (x * x + y * y); }, 0.015, 0.02);
      fillGrid(B, rng, [k, dk, overlap](double x, double y) { return -overlap + (k + dk) * (x * x + y * y); }, 0.015, 0.02);
      break;
    }
    case F_DISJOINT:
      fillGrid(A, rng, [ampA](double x, double y) { return ampA * 0.3 * (x * x + y * y); }, 0.08, 0.05);
      fillGrid(B, rng, [ampB](double x, double y) { return 5.0 + ampB * 0.3 * (x * x + y * y); }, 0.08, 0.05);
      break;
    default: return false;
  }
  const double pRat = (family == F_DISJOINT) ? 0.0 : 0.5;
  maybeRational(A, rng, pRat);
  maybeRational(B, rng, pRat);
  return true;
}

int pickFamily(Rng& rng) {
  const double fr = rng.unit();
  if (fr < 0.34) return F_TRANSVERSAL;
  if (fr < 0.60) return F_TILTED;
  if (fr < 0.80) return F_MULTIBRANCH;
  if (fr < 0.92) return F_NEAR_TANGENT;
  return F_DISJOINT;
}

double knotLo(const std::vector<double>& k, int deg) { return k[static_cast<std::size_t>(deg)]; }
double knotHi(const std::vector<double>& k, int nPoles) { return k[static_cast<std::size_t>(nPoles)]; }

// ── exact surface evaluation with analytic first derivatives ─────────────────────────
struct SurfEval {
  const NurbsSurf* s = nullptr;
  math::SurfaceGrid grid{};
  void bind(const NurbsSurf& surf) {
    s = &surf;
    grid = math::SurfaceGrid{surf.poles, surf.nU, surf.nV};
  }
  Point3 point(double u, double v) const {
    if (s->rational())
      return math::nurbsSurfacePoint(s->degU, s->degV, grid, s->weights, s->knotsU, s->knotsV, u, v);
    return math::surfacePoint(s->degU, s->degV, grid, s->knotsU, s->knotsV, u, v);
  }
  // Fills P, dU=∂S/∂u, dV=∂S/∂v via exact analytic derivatives (A3.6 / A4.4).
  void eval(double u, double v, Point3& P, Vec3& dU, Vec3& dV) const {
    std::array<Vec3, 4> d{};  // maxDeriv=1 → 2×2 layout: [0]=S,[1]=∂v,[2]=∂u,[3]=∂u∂v
    if (s->rational())
      math::nurbsSurfaceDerivs(s->degU, s->degV, grid, s->weights, s->knotsU, s->knotsV, u, v, 1, d);
    else
      math::surfaceDerivs(s->degU, s->degV, grid, s->knotsU, s->knotsV, u, v, 1, d);
    // out[k*(maxDeriv+1)+l] = ∂^(k+l)S/∂u^k∂v^l ; maxDeriv=1 ⇒ stride 2:
    //   [0]=∂^0 (point), [1]=∂/∂v, [2]=∂/∂u, [3]=∂²/∂u∂v.
    P  = Point3{d[0].x, d[0].y, d[0].z};
    dV = d[1];
    dU = d[2];
  }
  Dir3 normal(double u, double v) const {
    Point3 P; Vec3 dU, dV; eval(u, v, P, dU, dV);
    return Dir3(math::cross(dU, dV));
  }
};

// ── 4×4 dense linear solve (Gaussian elimination, partial pivoting) — the oracle's OWN
//    solver; NO native-numerics, NO OCCT. Returns false if singular. ────────────────────
bool solve4(double A[4][4], double b[4], double x[4]) {
  int idx[4] = {0, 1, 2, 3};
  for (int col = 0; col < 4; ++col) {
    int piv = col; double best = std::fabs(A[col][col]);
    for (int r = col + 1; r < 4; ++r) if (std::fabs(A[r][col]) > best) { best = std::fabs(A[r][col]); piv = r; }
    if (best < 1e-300) return false;
    if (piv != col) { for (int c = 0; c < 4; ++c) std::swap(A[col][c], A[piv][c]); std::swap(b[col], b[piv]); }
    for (int r = col + 1; r < 4; ++r) {
      const double f = A[r][col] / A[col][col];
      for (int c = col; c < 4; ++c) A[r][c] -= f * A[col][c];
      b[r] -= f * b[col];
    }
  }
  for (int r = 3; r >= 0; --r) {
    double s = b[r];
    for (int c = r + 1; c < 4; ++c) s -= A[r][c] * x[c];
    x[r] = s / A[r][r];
  }
  (void)idx;
  return true;
}

// Power-iteration estimate of the 2-norm condition number of a 4×4 matrix J (σ_max/σ_min
// of J via eigenvalues of JᵀJ). Robust enough for a diagnostic conditioning number.
double cond4(const double J[4][4]) {
  double JtJ[4][4] = {};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k) s += J[k][i] * J[k][j];
      JtJ[i][j] = s;
    }
  auto rayleighExtreme = [&](bool wantMax) -> double {
    double v[4] = {1.0, 0.7, 0.3, -0.5};
    double M[4][4];
    if (wantMax) { for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) M[i][j] = JtJ[i][j]; }
    else {
      // shift: (trace·I − JtJ) → largest eigenvalue maps to smallest of JtJ
      double tr = 0.0; for (int i = 0; i < 4; ++i) tr += JtJ[i][i];
      const double sh = tr * 1.0 + 1.0;
      for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) M[i][j] = (i == j ? sh : 0.0) - JtJ[i][j];
    }
    double lam = 0.0;
    for (int it = 0; it < 500; ++it) {
      double w[4] = {};
      for (int i = 0; i < 4; ++i) { double s = 0.0; for (int j = 0; j < 4; ++j) s += M[i][j] * v[j]; w[i] = s; }
      double n = std::sqrt(w[0]*w[0]+w[1]*w[1]+w[2]*w[2]+w[3]*w[3]);
      if (n < 1e-300) break;
      for (int i = 0; i < 4; ++i) v[i] = w[i] / n;
      lam = n;
    }
    return lam;
  };
  const double lmax = rayleighExtreme(true);
  double tr = 0.0; for (int i = 0; i < 4; ++i) tr += JtJ[i][i];
  const double shift = tr * 1.0 + 1.0;
  const double lminShifted = rayleighExtreme(false);
  const double lmin = shift - lminShifted;  // recover smallest eigenvalue of JtJ
  if (lmin <= 0.0) return std::numeric_limits<double>::infinity();
  return std::sqrt(lmax / lmin);
}

// ── the independent true-curve foot-point projector ──────────────────────────────────
//
// Given a reference world point Pref, find the intersection-curve point that is the FOOT
// of perpendicular from Pref. Residual vector r(u1,v1,u2,v2):
//   r0..2 = A(u1,v1) − B(u2,v2)                         (on both surfaces, over-determined; only 2 DOF live)
//   r3    = dot(M − Pref, T)   where M=(A+B)/2, T = unit(dA_along ≈ curve tangent)
// The 4th constraint is the perpendicular-foot condition; we build a SQUARE 4×4 Newton by
// collapsing the 3 A−B residuals against the 4 params with the exact Jacobian
//   ∂(A−B)/∂(u1,v1,u2,v2) = [dU_A, dV_A, −dU_B, −dV_B]
// and appending the perpendicular row ∂r3/∂params. The tangent T is recomputed each step
// from the current normals (T = unit(nA×nB)); where sine→0 the tangent is ill-defined but
// the perpendicular constraint degrades GRACEFULLY (it just becomes a weaker pin) rather
// than dividing by sine as native's dot(A−Pprev,t)−h advance residual does. Damped.
struct FootResult {
  bool ok = false;
  double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
  Point3 P{};             // true curve point (midpoint of the two feet)
  double abResid = 0.0;   // ‖A−B‖ at the solution
  double sine = 0.0;      // ‖n̂A × n̂B‖ (transversality)
  double normalAngleDeg = 0.0;
  double corrCond = 0.0;  // condition number of the two-surface corrector Jacobian
  int iters = 0;
};

FootResult footProject(const SurfEval& A, const SurfEval& B, const Point3& Pref,
                       double u1, double v1, double u2, double v2,
                       double scale, double lo1u, double hi1u, double lo1v, double hi1v,
                       double lo2u, double hi2u, double lo2v, double hi2v) {
  auto clampd = [](double x, double a, double b) { return x < a ? a : (x > b ? b : x); };
  FootResult R;
  const double tolAB = 1e-13 * scale;
  for (int it = 0; it < 200; ++it) {
    Point3 PA, PB; Vec3 dUA, dVA, dUB, dVB;
    A.eval(u1, v1, PA, dUA, dVA);
    B.eval(u2, v2, PB, dUB, dVB);
    const Vec3 gap = PA - PB;
    const Point3 M = PA + (PB - PA) * 0.5;
    const Dir3 nA = Dir3(math::cross(dUA, dVA));
    const Dir3 nB = Dir3(math::cross(dUB, dVB));
    Vec3 Tc = math::cross(nA.vec(), nB.vec());
    const double sineNow = math::norm(Tc);
    Vec3 Tu = sineNow > 1e-300 ? Tc * (1.0 / sineNow) : Vec3{1, 0, 0};
    const double r3 = math::dot(M - Pref, Tu);

    // Jacobian rows.  Params order: u1,v1,u2,v2.
    // Rows 0..2: ∂(A−B)/∂p = [dUA, dVA, −dUB, −dVB].
    // Row 3: ∂(dot(M−Pref,Tu))/∂p ≈ dot(∂M/∂p, Tu)   (freeze Tu — Gauss-Newton, exact enough
    //        for the foot constraint; the curve tangent varies slowly vs the foot motion).
    //        ∂M/∂u1 = 0.5·dUA, ∂M/∂v1 = 0.5·dVA, ∂M/∂u2 = 0.5·dUB, ∂M/∂v2 = 0.5·dVB.
    double J[4][4];
    const Vec3 cols[4] = {dUA, dVA, dUB * -1.0, dVB * -1.0};
    for (int c = 0; c < 4; ++c) { J[0][c] = cols[c].x; J[1][c] = cols[c].y; J[2][c] = cols[c].z; }
    J[3][0] = 0.5 * math::dot(dUA, Tu);
    J[3][1] = 0.5 * math::dot(dVA, Tu);
    J[3][2] = 0.5 * math::dot(dUB, Tu);
    J[3][3] = 0.5 * math::dot(dVB, Tu);

    double b[4] = {-gap.x, -gap.y, -gap.z, -r3};
    double Jw[4][4]; for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) Jw[i][j] = J[i][j];
    double bw[4]; for (int i = 0; i < 4; ++i) bw[i] = b[i];
    double step[4];
    if (!solve4(Jw, bw, step)) {
      // fall back: Levenberg damping on JtJ
      double JtJ[4][4] = {}, Jtb[4] = {};
      for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) { double s = 0; for (int k = 0; k < 4; ++k) s += J[k][i]*J[k][j]; JtJ[i][j] = s; }
      for (int i = 0; i < 4; ++i) { double s = 0; for (int k = 0; k < 4; ++k) s += J[k][i]*b[k]; Jtb[i] = s; }
      for (int i = 0; i < 4; ++i) JtJ[i][i] += 1e-9;
      if (!solve4(JtJ, Jtb, step)) break;
    }
    // damped update
    double damp = 1.0;
    if (std::fabs(step[0]) > 0.5) damp = std::min(damp, 0.5 / std::fabs(step[0]));
    if (std::fabs(step[1]) > 0.5) damp = std::min(damp, 0.5 / std::fabs(step[1]));
    if (std::fabs(step[2]) > 0.5) damp = std::min(damp, 0.5 / std::fabs(step[2]));
    if (std::fabs(step[3]) > 0.5) damp = std::min(damp, 0.5 / std::fabs(step[3]));
    u1 = clampd(u1 + damp * step[0], lo1u, hi1u);
    v1 = clampd(v1 + damp * step[1], lo1v, hi1v);
    u2 = clampd(u2 + damp * step[2], lo2u, hi2u);
    v2 = clampd(v2 + damp * step[3], lo2v, hi2v);

    R.iters = it + 1;
    const double stepNorm = std::fabs(step[0]) + std::fabs(step[1]) + std::fabs(step[2]) + std::fabs(step[3]);
    if (math::norm(gap) <= tolAB && stepNorm < 1e-14) break;
    if (stepNorm < 1e-15) break;
  }
  // final eval
  Point3 PA, PB; Vec3 dUA, dVA, dUB, dVB;
  A.eval(u1, v1, PA, dUA, dVA);
  B.eval(u2, v2, PB, dUB, dVB);
  const Dir3 nA = Dir3(math::cross(dUA, dVA));
  const Dir3 nB = Dir3(math::cross(dUB, dVB));
  const double sine = math::norm(math::cross(nA.vec(), nB.vec()));
  double dotn = math::dot(nA.vec(), nB.vec()); dotn = dotn > 1 ? 1 : (dotn < -1 ? -1 : dotn);
  double J[4][4];
  const Vec3 cols[4] = {dUA, dVA, dUB * -1.0, dVB * -1.0};
  Vec3 Tc = math::cross(nA.vec(), nB.vec());
  const double sn = math::norm(Tc); Vec3 Tu = sn > 1e-300 ? Tc * (1.0 / sn) : Vec3{1, 0, 0};
  for (int c = 0; c < 4; ++c) { J[0][c] = cols[c].x; J[1][c] = cols[c].y; J[2][c] = cols[c].z; }
  J[3][0] = 0.5 * math::dot(dUA, Tu); J[3][1] = 0.5 * math::dot(dVA, Tu);
  J[3][2] = 0.5 * math::dot(dUB, Tu); J[3][3] = 0.5 * math::dot(dVB, Tu);
  R.u1 = u1; R.v1 = v1; R.u2 = u2; R.v2 = v2;
  R.P = PA + (PB - PA) * 0.5;
  R.abResid = math::distance(PA, PB);
  R.sine = sine;
  R.normalAngleDeg = std::acos(std::fabs(dotn)) * 57.29577951308232;
  R.corrCond = cond4(J);
  R.ok = R.abResid <= 1e-9 * scale;
  return R;
}

int main(int argc, char** argv) {
  if (argc < 4) {
    std::fprintf(stderr, "usage: %s <baseSeedHex> <si> <idx> [--ref u1 v1 u2 v2] [--refP x y z] [--occt u1 v1 u2 v2] [--occtP x y z]\n", argv[0]);
    return 2;
  }
  const uint64_t base = std::strtoull(argv[1], nullptr, 0);
  const int siTarget = std::atoi(argv[2]);
  const int idxTarget = std::atoi(argv[3]);

  std::optional<std::array<double, 4>> refParams, occtParams, flankLo, flankHi;
  std::optional<Point3> refP, occtP;
  for (int i = 4; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--ref") && i + 4 < argc) {
      refParams = std::array<double, 4>{atof(argv[i+1]), atof(argv[i+2]), atof(argv[i+3]), atof(argv[i+4])}; i += 4;
    } else if (!std::strcmp(argv[i], "--flankLo") && i + 4 < argc) {
      flankLo = std::array<double, 4>{atof(argv[i+1]), atof(argv[i+2]), atof(argv[i+3]), atof(argv[i+4])}; i += 4;
    } else if (!std::strcmp(argv[i], "--flankHi") && i + 4 < argc) {
      flankHi = std::array<double, 4>{atof(argv[i+1]), atof(argv[i+2]), atof(argv[i+3]), atof(argv[i+4])}; i += 4;
    } else if (!std::strcmp(argv[i], "--refP") && i + 3 < argc) {
      refP = Point3{atof(argv[i+1]), atof(argv[i+2]), atof(argv[i+3])}; i += 3;
    } else if (!std::strcmp(argv[i], "--occt") && i + 4 < argc) {
      occtParams = std::array<double, 4>{atof(argv[i+1]), atof(argv[i+2]), atof(argv[i+3]), atof(argv[i+4])}; i += 4;
    } else if (!std::strcmp(argv[i], "--occtP") && i + 3 < argc) {
      occtP = Point3{atof(argv[i+1]), atof(argv[i+2]), atof(argv[i+3])}; i += 3;
    }
  }
  if (const char* e = std::getenv("ORACLE_OCCT_P")) {
    double x, y, z; if (std::sscanf(e, "%lf %lf %lf", &x, &y, &z) == 3) occtP = Point3{x, y, z};
  }

  const uint64_t thisSeed = base + static_cast<uint64_t>(siTarget) * 0x100000001B3ull;
  Rng rng(thisSeed);
  int family = -1;
  NurbsSurf A, B;
  for (int idx = 0; idx <= idxTarget; ++idx) {
    family = pickFamily(rng);
    A = NurbsSurf{}; B = NurbsSurf{};
    buildPair(family, rng, A, B);
  }

  SurfEval EA, EB; EA.bind(A); EB.bind(B);
  const double loAu = knotLo(A.knotsU, A.degU), hiAu = knotHi(A.knotsU, A.nU);
  const double loAv = knotLo(A.knotsV, A.degV), hiAv = knotHi(A.knotsV, A.nV);
  const double loBu = knotLo(B.knotsU, B.degU), hiBu = knotHi(B.knotsU, B.nU);
  const double loBv = knotLo(B.knotsV, B.degV), hiBv = knotHi(B.knotsV, B.nV);

  // model scale (diag of pole bbox of A) — matches the tracer's scale intent closely enough
  Point3 lo = A.poles[0], hi = A.poles[0];
  for (const auto& p : A.poles) { lo.x=std::min(lo.x,p.x); lo.y=std::min(lo.y,p.y); lo.z=std::min(lo.z,p.z);
                                   hi.x=std::max(hi.x,p.x); hi.y=std::max(hi.y,p.y); hi.z=std::max(hi.z,p.z); }
  const double scale = math::distance(lo, hi);

  std::printf("== ORACLE seed=0x%llx si=%d idx=%d family=%s scale=%.4e ==\n",
              (unsigned long long)thisSeed, siTarget, idxTarget, famName(family), scale);
  std::printf("   A: deg %dx%d poles %dx%d rational=%d  domainA=[%.3f,%.3f]x[%.3f,%.3f]\n",
              A.degU, A.degV, A.nU, A.nV, (int)A.rational(), loAu, hiAu, loAv, hiAv);
  std::printf("   B: deg %dx%d poles %dx%d rational=%d  domainB=[%.3f,%.3f]x[%.3f,%.3f]\n",
              B.degU, B.degV, B.nU, B.nV, (int)B.rational(), loBu, hiBu, loBv, hiBv);

  // Reference world point + seed params.
  Point3 Pref;
  double su1, sv1, su2, sv2;
  if (refParams) {
    su1 = (*refParams)[0]; sv1 = (*refParams)[1]; su2 = (*refParams)[2]; sv2 = (*refParams)[3];
    Point3 pa = EA.point(su1, sv1);
    Pref = refP ? *refP : pa;
  } else if (refP) {
    Pref = *refP;
    // seed params: coarse grid nearest-point on each surface
    auto nearest = [&](const SurfEval& S, double lu, double hu, double lv, double hv, double& bu, double& bv) {
      double best = 1e300; bu = 0.5*(lu+hu); bv = 0.5*(lv+hv);
      for (int i = 0; i <= 40; ++i) for (int j = 0; j <= 40; ++j) {
        double u = lu + (hu-lu)*i/40.0, v = lv + (hv-lv)*j/40.0;
        double d = math::distance(S.point(u,v), Pref);
        if (d < best) { best = d; bu = u; bv = v; }
      }
    };
    nearest(EA, loAu, hiAu, loAv, hiAv, su1, sv1);
    nearest(EB, loBu, hiBu, loBv, hiBv, su2, sv2);
  } else {
    std::fprintf(stderr, "ERROR: need --ref or --refP\n"); return 2;
  }
  std::printf("   REF params uvA=(%.6f,%.6f) uvB=(%.6f,%.6f) Pref=(%.6f,%.6f,%.6f)\n",
              su1, sv1, su2, sv2, Pref.x, Pref.y, Pref.z);

  const FootResult truth = footProject(EA, EB, Pref, su1, sv1, su2, sv2, scale,
                                       loAu, hiAu, loAv, hiAv, loBu, hiBu, loBv, hiBv);
  std::printf("-- TRUE CURVE POINT (independent high-precision foot from native ref) --\n");
  std::printf("   ok=%d iters=%d  uvA=(%.9f,%.9f) uvB=(%.9f,%.9f)\n",
              (int)truth.ok, truth.iters, truth.u1, truth.v1, truth.u2, truth.v2);
  std::printf("   Ptrue=(%.9f,%.9f,%.9f)  abResid=%.3e  sine=%.6e  normalAngle=%.4f deg  corrCond=%.4e\n",
              truth.P.x, truth.P.y, truth.P.z, truth.abResid, truth.sine, truth.normalAngleDeg, truth.corrCond);

  // native-root → true
  const double dNativeTruth = math::distance(Pref, truth.P);
  std::printf("\n=== THREE-WAY DISTANCES @ graze pinch ===\n");
  std::printf("   native-root -> true-curve = %.6e   (%.4f x onCurve[1e-3];  %.4f x scale-onCurve)\n",
              dNativeTruth, dNativeTruth / 1e-3, dNativeTruth / (scale * 1e-3));

  if (occtP || occtParams) {
    Point3 occtWorld;
    double ou1 = su1, ov1 = sv1, ou2 = su2, ov2 = sv2;
    if (occtParams) { ou1=(*occtParams)[0]; ov1=(*occtParams)[1]; ou2=(*occtParams)[2]; ov2=(*occtParams)[3];
                      occtWorld = occtP ? *occtP : EA.point(ou1, ov1); }
    else            { occtWorld = *occtP; }
    // project OCCT root onto the true curve too (independent foot)
    const FootResult truthO = footProject(EA, EB, occtWorld, ou1, ov1, ou2, ov2, scale,
                                          loAu, hiAu, loAv, hiAv, loBu, hiBu, loBv, hiBv);
    const double dOcctTruth = math::distance(occtWorld, truthO.P);
    const double dNativeOcct = math::distance(Pref, occtWorld);
    std::printf("   OCCT-root   -> true-curve = %.6e   (foot ok=%d abResid=%.3e sine=%.4e)\n",
                dOcctTruth, (int)truthO.ok, truthO.abResid, truthO.sine);
    std::printf("   native-root -> OCCT-root  = %.6e\n", dNativeOcct);
    std::printf("\n   VERDICT INPUT: native/truth=%.4e  occt/truth=%.4e  ratio(native/occt)=%.4f\n",
                dNativeTruth, dOcctTruth, dOcctTruth > 0 ? dNativeTruth / dOcctTruth : 0.0);
  } else {
    std::printf("   (OCCT root not supplied — pass --occtP x y z or ORACLE_OCCT_P to complete the three-way)\n");
  }

  // ── CHORD BOW: the true intersection curve's deviation from the straight segment between
  //    the two native nodes flanking the pinch. This is the DISCRETIZATION coverage a coarse
  //    polyline / a fitted-spline segment must approximate through the sharp graze turn —
  //    the source of the sub-onCurve fitted-B-spline bow (roadmap: "~1e-4..1.1e-3"). It is a
  //    FIT-RESOLUTION artifact of the emitted curve, NOT a corrected-node accuracy error.
  if (flankLo && flankHi) {
    FootResult fLo = footProject(EA, EB, EA.point((*flankLo)[0], (*flankLo)[1]),
                                 (*flankLo)[0], (*flankLo)[1], (*flankLo)[2], (*flankLo)[3], scale,
                                 loAu, hiAu, loAv, hiAv, loBu, hiBu, loBv, hiBv);
    FootResult fHi = footProject(EA, EB, EA.point((*flankHi)[0], (*flankHi)[1]),
                                 (*flankHi)[0], (*flankHi)[1], (*flankHi)[2], (*flankHi)[3], scale,
                                 loAu, hiAu, loAv, hiAv, loBu, hiBu, loBv, hiBv);
    const Point3 pLo = fLo.P, pHi = fHi.P;
    const double chordLen = math::distance(pLo, pHi);
    // sample the TRUE curve between the two flanks (foot-projecting interpolated world points)
    // and measure max distance to the straight chord pLo→pHi.
    double maxBow = 0.0;
    const Vec3 chord = pHi - pLo; const double cl2 = math::normSquared(chord);
    for (int i = 1; i < 32; ++i) {
      const double t = double(i) / 32.0;
      // linear param interpolation as the foot seed; foot-project to the true curve
      const double gu1 = (*flankLo)[0] + t*((*flankHi)[0]-(*flankLo)[0]);
      const double gv1 = (*flankLo)[1] + t*((*flankHi)[1]-(*flankLo)[1]);
      const double gu2 = (*flankLo)[2] + t*((*flankHi)[2]-(*flankLo)[2]);
      const double gv2 = (*flankLo)[3] + t*((*flankHi)[3]-(*flankLo)[3]);
      const Point3 seedW = EA.point(gu1, gv1);
      FootResult ft = footProject(EA, EB, seedW, gu1, gv1, gu2, gv2, scale,
                                  loAu, hiAu, loAv, hiAv, loBu, hiBu, loBv, hiBv);
      // distance from the true point to the straight chord
      const Vec3 w = ft.P - pLo;
      const double s = cl2 > 0 ? math::dot(w, chord) / cl2 : 0.0;
      const Point3 proj = pLo + chord * (s < 0 ? 0 : (s > 1 ? 1 : s));
      maxBow = std::max(maxBow, math::distance(ft.P, proj));
    }
    std::printf("\n=== CHORD BOW (fit-resolution coverage through the graze turn) ===\n");
    std::printf("   flank chord length=%.6e   maxTrueCurveBow off chord=%.6e   (%.4f x onCurve[1e-3])\n",
                chordLen, maxBow, maxBow / 1e-3);
  }

  // ── FUZZY-BAND WIDTH: how far can a point drift LATERALLY (perpendicular to the curve
  //    tangent, within the tangent plane of A) while staying within onSurf (scale·1e-6) of
  //    BOTH surfaces? At a near-tangent graze the two surfaces are nearly parallel, so a
  //    point can slide a MEASURABLE lateral distance and still be "on both surfaces" to the
  //    corrector tolerance — the ill-conditioned band. Its half-width is the DECISIVE
  //    conditioning number: if it is ≳ the native↔OCCT ~1.8e-3, then native and OCCT are
  //    both valid points in one fuzzy band (moat confirmed); if it is ≪ that, the divergence
  //    is a genuine accuracy deficit.
  {
    Point3 PA, PB; Vec3 dUA, dVA, dUB, dVB;
    EA.eval(truth.u1, truth.v1, PA, dUA, dVA);
    EB.eval(truth.u2, truth.v2, PB, dUB, dVB);
    const Dir3 nA = Dir3(math::cross(dUA, dVA));
    const Dir3 nB = Dir3(math::cross(dUB, dVB));
    Vec3 T = math::cross(nA.vec(), nB.vec());
    const double tn = math::norm(T); if (tn > 1e-300) T = T * (1.0 / tn);
    // lateral direction: in A's tangent plane, perpendicular to the curve tangent T
    Vec3 lat = math::cross(nA.vec(), T);
    const double ln = math::norm(lat); if (ln > 1e-300) lat = lat * (1.0 / ln);
    const double onSurf = scale * 1e-6;
    // walk laterally from the true point; at each offset, find the max of the two surface
    // distances (project the drifted point back onto each surface by a few Newton steps).
    auto surfDist = [&](const SurfEval& S, const Point3& q, double su, double sv,
                        double lu, double hu, double lv, double hv) -> double {
      double u = su, v = sv;
      for (int it = 0; it < 40; ++it) {
        Point3 P; Vec3 dU, dV; S.eval(u, v, P, dU, dV);
        const Vec3 r = q - P;
        const double a = math::dot(dU, dU), b = math::dot(dU, dV), c = math::dot(dV, dV);
        const double d0 = math::dot(dU, r), d1 = math::dot(dV, r);
        const double det = a*c - b*b; if (std::fabs(det) < 1e-300) break;
        double du = (d0*c - d1*b)/det, dv = (a*d1 - b*d0)/det;
        u += du; v += dv;
        u = u<lu?lu:(u>hu?hu:u); v = v<lv?lv:(v>hv?hv:v);
        if (std::fabs(du)+std::fabs(dv) < 1e-14) break;
      }
      return math::distance(S.point(u, v), q);
    };
    double bandHalf = 0.0;
    for (int sgn = -1; sgn <= 1; sgn += 2) {
      for (double off = scale*1e-8; off <= scale*1e-1; off *= 1.4) {
        const Point3 q = truth.P + lat * (sgn * off);
        const double dA = surfDist(EA, q, truth.u1, truth.v1, loAu, hiAu, loAv, hiAv);
        const double dB = surfDist(EB, q, truth.u2, truth.v2, loBu, hiBu, loBv, hiBv);
        if (std::max(dA, dB) <= onSurf) bandHalf = std::max(bandHalf, off);
        else break;
      }
    }
    std::printf("\n=== FUZZY-BAND WIDTH (ill-conditioning at the graze) ===\n");
    std::printf("   sine=%.4e normalAngle=%.4f deg  onSurf gate=%.3e\n",
                truth.sine, truth.normalAngleDeg, onSurf);
    std::printf("   lateral half-width still within onSurf of BOTH surfaces = %.6e   (full band=%.6e)\n",
                bandHalf, 2.0 * bandHalf);
    std::printf("   => a point can differ laterally by up to %.4e and still be 'on both surfaces' to onSurf.\n",
                2.0 * bandHalf);
  }
  return 0;
}
