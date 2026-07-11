// SPDX-License-Identifier: Apache-2.0
//
// bspline_skin.cpp — NURBS roadmap Layer 6 (skinning / lofting) implementation.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.3 (Algorithm
// A10.3). It COMPOSES Layer 1 (bspline_ops: elevateDegreeCurve / refineKnotCurve, to
// make the sections compatible) and the Layer-7 curve-interpolation machinery (a
// collocation solve through the numsci facade, to interpolate across the sections in
// V). The V interpolation solves linear systems through numerics::lin_solve, so — like
// bspline_fit.cpp — the WHOLE file is under CYBERCAD_HAS_NUMSCI. With the guard OFF the
// TU is inert and the Layer-6 functions are simply absent from the library.
//
#include "native/math/bspline_skin.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"        // findSpan / basisFuns (V-interpolation collocation)
#include "native/math/bspline_ops.h"    // elevateDegreeCurve / refineKnotCurve (compatibility)
#include "native/numerics/numerics.h"   // lin_solve (square V-interpolation solve)

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

namespace cybercad::native::math {
namespace {

using numerics::lin_solve;

constexpr double kKnotEps = 1e-9;  // knot-value coincidence tolerance for the union.

// Number of control points implied by a flat clamped knot vector of given degree.
int nPolesOf(const BsplineCurveData& c) {
  return static_cast<int>(c.knots.size()) - c.degree - 1;
}

// ── Knot union (max multiplicity per distinct value) ──────────────────────────
// Collect every distinct knot value that appears in ANY section, tagged with the
// MAXIMUM multiplicity it reaches across all sections. Assumes all sections have been
// raised to the common degree `p` already (so end-clamp multiplicities match) and
// share the same parameter domain [a,b]. Returns the distinct values sorted ascending.
struct KnotMult { double value; int mult; };

std::vector<KnotMult> knotUnion(std::span<const BsplineCurveData> sections) {
  std::vector<KnotMult> uni;
  for (const BsplineCurveData& c : sections) {
    // Multiplicity of each distinct value within THIS section.
    std::size_t k = 0;
    while (k < c.knots.size()) {
      const double val = c.knots[k];
      int m = 0;
      while (k < c.knots.size() && std::fabs(c.knots[k] - val) <= kKnotEps) { ++m; ++k; }
      // Merge into the union at max multiplicity.
      auto it = std::find_if(uni.begin(), uni.end(),
                             [&](const KnotMult& u) { return std::fabs(u.value - val) <= kKnotEps; });
      if (it == uni.end())
        uni.push_back({val, m});
      else
        it->mult = std::max(it->mult, m);
    }
  }
  std::sort(uni.begin(), uni.end(),
            [](const KnotMult& a, const KnotMult& b) { return a.value < b.value; });
  return uni;
}

// The multiset of knots to INSERT into `c` so its knot vector reaches the union: for
// each distinct union value, (unionMult − existingMult) copies. Existing multiplicity
// is counted with the same tolerance.
std::vector<double> knotsToInsert(const BsplineCurveData& c,
                                  const std::vector<KnotMult>& uni) {
  std::vector<double> ins;
  for (const KnotMult& u : uni) {
    int have = 0;
    for (double kv : c.knots)
      if (std::fabs(kv - u.value) <= kKnotEps) ++have;
    for (int r = have; r < u.mult; ++r) ins.push_back(u.value);
  }
  return ins;
}

// ── Section parameters v_k (chord length across the sections' control polygons) ──
// §10.3: average, over the N control-point indices, the chord-length parameters of the
// polyline {P_i^0, …, P_i^{K−1}} in V. Coincident sections contribute zero length;
// if every section is coincident there is no length to normalize and we return empty.
std::vector<double> sectionParams(const std::vector<BsplineCurveData>& sec, int N) {
  const int K = static_cast<int>(sec.size());
  std::vector<double> acc(K, 0.0);
  int usedRows = 0;
  for (int i = 0; i < N; ++i) {
    // Cumulative chord length of {P_i^k} over k.
    std::vector<double> d(K, 0.0);
    double total = 0.0;
    for (int k = 1; k < K; ++k) {
      total += distance(sec[k].poles[i], sec[k - 1].poles[i]);
      d[k] = total;
    }
    if (!(total > 0.0)) continue;  // this control index is coincident across sections
    for (int k = 0; k < K; ++k) acc[k] += d[k] / total;
    ++usedRows;
  }
  if (usedRows == 0) return {};  // every section coincident — no V length to normalize
  std::vector<double> v(K);
  for (int k = 0; k < K; ++k) v[k] = acc[k] / usedRows;
  v.front() = 0.0;
  v.back() = 1.0;  // pin ends (guard fp drift)
  return v;
}

// ── Averaging knots for the V interpolation (Eq 9.8) ──────────────────────────
std::vector<double> avgKnots(const std::vector<double>& v, int q) {
  const int n = static_cast<int>(v.size()) - 1;  // last index (K−1)
  const int m = n + q + 1;
  std::vector<double> U(m + 1, 0.0);
  for (int i = m - q; i <= m; ++i) U[i] = 1.0;      // clamped tail
  for (int j = 1; j <= n - q; ++j) {                // interior averages
    double s = 0.0;
    for (int i = j; i <= j + q - 1; ++i) s += v[i];
    U[j + q] = s / q;
  }
  return U;
}

// ── Interpolate the V-curves for every control index at once ──────────────────
// Solve the K×K collocation system A·X = B once (A depends only on v_k / V-knots,
// shared by every control index and coordinate). Column layout of the result: for
// control index i and V-control j, out[i][j]. Returns false on a singular solve.
bool interpolateAcrossV(const std::vector<BsplineCurveData>& sec, int N,
                        const std::vector<double>& v, const std::vector<double>& V,
                        int q, std::vector<Point3>& netUouter) {
  const int K = static_cast<int>(sec.size());
  const int lastPole = K - 1;

  // Collocation matrix A(k,j) = N_{j,q}(v_k), row-major K×K.
  std::vector<double> A(static_cast<std::size_t>(K) * K, 0.0);
  std::vector<double> Nb(q + 1);
  for (int k = 0; k < K; ++k) {
    const int span = findSpan(lastPole, q, v[k], V);
    basisFuns(span, v[k], q, V, Nb);
    for (int j = 0; j <= q; ++j)
      A[static_cast<std::size_t>(k) * K + (span - q + j)] = Nb[j];
  }

  // For each control index i, three RHS = the K section poles at index i (x/y/z).
  // Solve gives the K V-control points for column i. Reused matrix A, fresh RHS.
  netUouter.assign(static_cast<std::size_t>(N) * K, Point3{});
  std::vector<double> bx(K), by(K), bz(K);
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < K; ++k) {
      const Point3 p = sec[k].poles[i];
      bx[k] = p.x; by[k] = p.y; bz[k] = p.z;
    }
    const std::vector<double> cx = lin_solve(A, K, bx);
    const std::vector<double> cy = lin_solve(A, K, by);
    const std::vector<double> cz = lin_solve(A, K, bz);
    if (static_cast<int>(cx.size()) != K || static_cast<int>(cy.size()) != K ||
        static_cast<int>(cz.size()) != K)
      return false;  // singular V-collocation
    for (int j = 0; j < K; ++j)
      netUouter[static_cast<std::size_t>(i) * K + j] = {cx[j], cy[j], cz[j]};
  }
  (void)q;
  return true;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Section compatibility (§10.3): raise to common degree, merge to union knots.
// ─────────────────────────────────────────────────────────────────────────────

SectionCompatibility makeSectionsCompatible(std::span<const BsplineCurveData> sections) {
  SectionCompatibility r;
  const int K = static_cast<int>(sections.size());
  if (K < 1) return r;

  // Non-rational scope + basic validity guard.
  int maxDeg = 0;
  for (const BsplineCurveData& c : sections) {
    if (!c.weights.empty()) return r;         // rational section — decline honestly
    if (c.degree < 1 || c.poles.empty()) return r;
    if (static_cast<int>(c.knots.size()) != nPolesOf(c) + c.degree + 1 || nPolesOf(c) < 1)
      return r;                               // malformed knot vector
    maxDeg = std::max(maxDeg, c.degree);
  }

  // Step 1 — raise every section to the common max degree (exact).
  std::vector<BsplineCurveData> raised;
  raised.reserve(K);
  for (const BsplineCurveData& c : sections)
    raised.push_back((c.degree == maxDeg) ? c : elevateDegreeCurve(c, maxDeg - c.degree));

  // Step 2 — merge every section onto the UNION knot vector (exact refinement).
  const std::vector<KnotMult> uni = knotUnion(raised);
  std::vector<BsplineCurveData> compat;
  compat.reserve(K);
  for (const BsplineCurveData& c : raised) {
    const std::vector<double> ins = knotsToInsert(c, uni);
    compat.push_back(ins.empty() ? c : refineKnotCurve(c, ins));
  }

  // Verify the post-condition: identical degree, knot vector, control-point count.
  const std::vector<double> commonKnots = compat.front().knots;
  const int N = nPolesOf(compat.front());
  for (const BsplineCurveData& c : compat) {
    if (c.degree != maxDeg || nPolesOf(c) != N) return r;
    if (c.knots.size() != commonKnots.size()) return r;
    for (std::size_t k = 0; k < commonKnots.size(); ++k)
      if (std::fabs(c.knots[k] - commonKnots[k]) > 1e-7) return r;
  }

  r.ok = true;
  r.degree = maxDeg;
  r.knots = commonKnots;
  r.sections = std::move(compat);
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Skinning / lofting (A10.3).
// ─────────────────────────────────────────────────────────────────────────────

SkinResult skinSurface(std::span<const BsplineCurveData> sections, int degreeV) {
  SkinResult r;
  const int K = static_cast<int>(sections.size());
  if (K < 2) return r;  // need at least two sections to loft between

  // Step 1 — compatibility. Every section now shares degree p, knots, control count N.
  const SectionCompatibility comp = makeSectionsCompatible(sections);
  if (!comp.ok) return r;
  const int p = comp.degree;
  const int N = nPolesOf(comp.sections.front());

  // V degree clamped to K−1 (few sections ⇒ lower-degree loft, always ≥ 1).
  int q = degreeV;
  if (q > K - 1) q = K - 1;
  if (q < 1) q = 1;

  // Step 2 — section parameters v_k (chord length across the control polygons).
  const std::vector<double> v = sectionParams(comp.sections, N);
  if (static_cast<int>(v.size()) != K) return r;  // all-coincident sections — decline
  const std::vector<double> V = avgKnots(v, q);

  // Step 3 — interpolate a V-curve through {P_i^k} for every control index i.
  std::vector<Point3> netUouter;  // row-major, U (i) outer, V (j) inner: [i*K + j]
  if (!interpolateAcrossV(comp.sections, N, v, V, q, netUouter)) return r;

  // Step 4 — assemble the surface. U = section direction (degree p, common section
  // knots, N poles); V = across-sections (degree q, averaging knots, K poles). The net
  // is already row-major U-outer (pole(i,j) = netUouter[i*K + j]).
  r.surface.degreeU = p;
  r.surface.degreeV = q;
  r.surface.nPolesU = N;
  r.surface.nPolesV = K;
  r.surface.knotsU = comp.knots;
  r.surface.knotsV = V;
  r.surface.poles = std::move(netUouter);
  // weights left empty ⇒ non-rational.

  r.vParams = v;
  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI
