// SPDX-License-Identifier: Apache-2.0
//
// test_native_nurbs_ops_fuzz.cpp — HOST differential fuzzer for the exact-NURBS
// geometry kernel (NURBS roadmap Layer 1: bspline_ops). OCCT-FREE, no sim: the
// oracle is the module's OWN represented geometry, sampled through the existing
// evaluators (native/math/bspline.h). This is the OCCT-free host arm; the SIM
// native-vs-OCCT parity leg is tests/sim/native_nurbs_ops_parity.mm.
//
// ─────────────────────────────────────────────────────────────────────────────
// BUILD DEPENDENCY — AUTHORED BUT NOT YET BUILDABLE (by design).
//
//   This fuzzer #includes "native/math/bspline_ops.h" and calls the FROZEN API
//   from openspec/changes/nurbs-exact-geometry-kernel/design.md. That module
//   (src/native/math/bspline_ops.{h,cpp}) is being implemented CONCURRENTLY in a
//   separate worktree and is NOT present in this worktree yet, so this file DOES
//   NOT COMPILE until the module lands. Everything else — the seeded generator,
//   the sampled-point self-oracle, the op-chain driver, the classification and
//   the DISAGREED==0 bar — is complete and compile-ready against the frozen
//   signatures. The orchestrator build-verifies at consolidation.
//
// CMAKE WIRING (deferred to the orchestrator — do NOT edit CMakeLists.txt here).
//   Mirror the always-on test_native_nurbs_ops exactly:
//     1) In the CYBERCAD_TESTS list (CMakeLists.txt ~line 291), add:
//            test_native_nurbs_ops_fuzz
//     2) In the always-on basename→source map (~line 575), add:
//            set(test_native_nurbs_ops_fuzz_SRC
//                "${CMAKE_CURRENT_SOURCE_DIR}/tests/native/test_native_nurbs_ops_fuzz.cpp")
//   No CYBERCAD_HAS_OCCT / CYBERCAD_HAS_NUMSCI guard: this arm is host-only and
//   always-on. The module src/native/math/bspline_ops.{h,cpp} MUST EXIST (compiled
//   into the core lib by the src/native glob) for this target to build/link.
//
// Build (standalone, once the module lands):
//   clang++ -std=c++20 tests/native/test_native_nurbs_ops_fuzz.cpp \
//     src/native/math/bspline.cpp src/native/math/bspline_ops.cpp \
//     -I src -I tests -o test_native_nurbs_ops_fuzz
//
// ── WHAT THIS FUZZES ────────────────────────────────────────────────────────────
// A random-but-seeded stream of VALID B-spline / NURBS curves and tensor-product
// surfaces, run through the bspline_ops construction algorithms — SINGLY and as
// random OP-COMPOSITION CHAINS — against a self-oracle that says the operation
// MUST NOT change the represented geometry. Because insert / refine / elevate /
// split / decompose / reparam are EXACT (they preserve the geometry pointwise),
// any deviation beyond a fixed tolerance is a real defect (DISAGREED). Knot
// removal and degree reduction are the only tolerance-bounded ops: they must be
// HONEST (recover the original on genuinely-reducible inputs; report ok=false /
// removed<num otherwise) — a correct honest decline is AGREED, not DISAGREED.
//
// ── THE ORACLE: the module's own sampled geometry (no OCCT needed) ──────────────
//   * Curve:   dense t-grid over [0,1] via curvePoint / nurbsCurvePoint.
//   * Surface: dense u×v grid over [0,1]² via surfacePoint / nurbsSurfacePoint.
// Before/after each exact op we compare pointwise; a relative deviation beyond
// kExactTol (1e-9, relative to point magnitude) is a DISAGREED. Round-trip ops
// (insert→remove, elevate→reduce) additionally assert the recovered NET matches.
//
// ── DISAGREED / classification (bar = DISAGREED==0 across ≥2 seeds, N≥60/seed) ──
//   AGREED             — exact op preserved geometry (or an honest round-trip held).
//   DISAGREED          — an exact op moved the geometry beyond kExactTol, a round-
//                        trip identity failed, or a structural invariant broke.
//   HONESTLY-DECLINED  — remove / reduce correctly reported it could not proceed
//                        within tol (removed<num or ok=false) and left the geometry
//                        UNCHANGED. This is CORRECT behaviour, never a DISAGREED.
//   ORACLE-INACCURATE  — the fp64 sampled oracle cannot represent a legitimate case
//                        (justified inline). Expected to be rare/absent here since
//                        the oracle IS the same evaluator the geometry is defined by.
// Tolerances are FIXED and NEVER widened to force a case through.
//
#include "native/math/bspline_ops.h"   // FROZEN API under test — lands concurrently
#include "native/math/bspline.h"       // findSpan / curvePoint / nurbs* evaluators (oracle)
#include "native/math/vec.h"           // Point3 / Vec3

#include "harness.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <span>
#include <string>
#include <vector>

namespace nm = cybercad::native::math;

namespace {

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the sibling
//    HOST/SIM fuzzers). Keyed ONLY by an explicit uint64 seed: same seed → byte-
//    identical batch. NO wall clock / rand() / address. ───────────────────────────
struct Rng {
  std::uint64_t s[4];
  static std::uint64_t splitmix64(std::uint64_t& x) {
    std::uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  explicit Rng(std::uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
  static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  std::uint64_t next() {
    const std::uint64_t r = rotl(s[1] * 5, 7) * 9;
    const std::uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return r;
  }
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  std::uint32_t below(std::uint32_t n) { return static_cast<std::uint32_t>(next() % n); }
  int irange(int lo, int hi) { return lo + static_cast<int>(below(static_cast<std::uint32_t>(hi - lo + 1))); }
};

// ═════════════════════════════════════════════════════════════════════════════
// FIXED tolerances — chosen from the evaluated-magnitude scale. NEVER widened.
// ═════════════════════════════════════════════════════════════════════════════
constexpr double kExactTol = 1e-9;   // pointwise deviation allowed on an EXACT op
constexpr double kNetTol   = 1e-9;   // recovered pole/knot/weight after a round-trip
constexpr double kOpTol    = 1e-9;   // tol handed to removeKnot / reduceDegree

// Relative error: absolute for small magnitudes, relative once |ref| is large, so a
// fixed 1e-9 absolute tolerance is not unfairly strict on large coordinates.
double relErr(double got, double ref) {
  const double a = std::fabs(got - ref);
  const double m = std::fabs(ref);
  return (m > 1.0) ? a / m : a;
}
double ptErr(const nm::Point3& a, const nm::Point3& b) {
  return std::max({relErr(a.x, b.x), relErr(a.y, b.y), relErr(a.z, b.z)});
}

// ═════════════════════════════════════════════════════════════════════════════
// Sampled-point self-oracle. A curve/surface is "the same geometry" iff it maps a
// dense parameter grid to the same points. Both evaluated through the module's own
// evaluators — insert/elevate/split/etc. must leave these samples invariant.
// ═════════════════════════════════════════════════════════════════════════════
nm::Point3 evalCurve(const nm::BsplineCurveData& c, double u) {
  return c.weights.empty() ? nm::curvePoint(c.degree, c.poles, c.knots, u)
                           : nm::nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}
nm::Point3 evalSurface(const nm::BsplineSurfaceData& s, double u, double v) {
  const nm::SurfaceGrid grid{s.poles, s.nPolesU, s.nPolesV};
  return s.weights.empty()
             ? nm::surfacePoint(s.degreeU, s.degreeV, grid, s.knotsU, s.knotsV, u, v)
             : nm::nurbsSurfacePoint(s.degreeU, s.degreeV, grid, s.weights, s.knotsU, s.knotsV, u, v);
}

// Domain [lo,hi] of a clamped flat knot vector: first / last knot value.
double loParam(const std::vector<double>& knots) { return knots.front(); }
double hiParam(const std::vector<double>& knots) { return knots.back(); }

// Worst pointwise deviation of curve `b` vs curve `a`, each sampled on ITS OWN
// domain but at the SAME normalized parameter fraction (so an affine reparam maps
// correctly). For a pure exact op with the SAME domain this is the identity compare.
double curveDeviation(const nm::BsplineCurveData& a, const nm::BsplineCurveData& b,
                      int samples = 64) {
  const double aLo = loParam(a.knots), aHi = hiParam(a.knots);
  const double bLo = loParam(b.knots), bHi = hiParam(b.knots);
  double e = 0.0;
  for (int i = 0; i <= samples; ++i) {
    const double f = double(i) / samples;
    e = std::max(e, ptErr(evalCurve(a, aLo + (aHi - aLo) * f), evalCurve(b, bLo + (bHi - bLo) * f)));
  }
  return e;
}
// Curve deviation restricted to a sub-fraction [f0,f1] of a's domain vs the WHOLE
// of b's domain (used to check a split piece reconstructs its slice of the source).
double curveDeviationSlice(const nm::BsplineCurveData& src, double f0, double f1,
                           const nm::BsplineCurveData& piece, int samples = 48) {
  const double sLo = loParam(src.knots), sHi = hiParam(src.knots);
  const double pLo = loParam(piece.knots), pHi = hiParam(piece.knots);
  double e = 0.0;
  for (int i = 0; i <= samples; ++i) {
    const double g = double(i) / samples;          // fraction along the piece
    const double f = f0 + (f1 - f0) * g;           // corresponding fraction along src
    e = std::max(e, ptErr(evalCurve(src, sLo + (sHi - sLo) * f), evalCurve(piece, pLo + (pHi - pLo) * g)));
  }
  return e;
}
double surfaceDeviation(const nm::BsplineSurfaceData& a, const nm::BsplineSurfaceData& b,
                        int samples = 14) {
  const double aUlo = loParam(a.knotsU), aUhi = hiParam(a.knotsU);
  const double aVlo = loParam(a.knotsV), aVhi = hiParam(a.knotsV);
  const double bUlo = loParam(b.knotsU), bUhi = hiParam(b.knotsU);
  const double bVlo = loParam(b.knotsV), bVhi = hiParam(b.knotsV);
  double e = 0.0;
  for (int su = 0; su <= samples; ++su)
    for (int sv = 0; sv <= samples; ++sv) {
      const double fu = double(su) / samples, fv = double(sv) / samples;
      e = std::max(e, ptErr(evalSurface(a, aUlo + (aUhi - aUlo) * fu, aVlo + (aVhi - aVlo) * fv),
                            evalSurface(b, bUlo + (bUhi - bUlo) * fu, bVlo + (bVhi - bVlo) * fv)));
    }
  return e;
}

// Exact NET compare (round-trip identity): degree/poles/weights/knots to kNetTol.
double curveNetErr(const nm::BsplineCurveData& a, const nm::BsplineCurveData& b) {
  if (a.degree != b.degree) return 1e9;
  if (a.poles.size() != b.poles.size()) return 1e9;
  if (a.knots.size() != b.knots.size()) return 1e9;
  if (a.weights.empty() != b.weights.empty()) return 1e9;
  double e = 0.0;
  for (std::size_t i = 0; i < a.poles.size(); ++i) {
    e = std::max(e, relErr(a.poles[i].x, b.poles[i].x));
    e = std::max(e, relErr(a.poles[i].y, b.poles[i].y));
    e = std::max(e, relErr(a.poles[i].z, b.poles[i].z));
  }
  for (std::size_t i = 0; i < a.weights.size() && i < b.weights.size(); ++i)
    e = std::max(e, relErr(a.weights[i], b.weights[i]));
  for (std::size_t i = 0; i < a.knots.size(); ++i)
    e = std::max(e, relErr(a.knots[i], b.knots[i]));
  return e;
}

// ═════════════════════════════════════════════════════════════════════════════
// VALIDITY GUARDS — the generator must ONLY emit valid inputs. Asserted before use.
// ═════════════════════════════════════════════════════════════════════════════
bool curveValid(const nm::BsplineCurveData& c) {
  if (c.degree < 1) return false;
  if (static_cast<int>(c.poles.size()) < c.degree + 1) return false;
  if (c.knots.size() != c.poles.size() + c.degree + 1) return false;             // length invariant
  if (!c.weights.empty() && c.weights.size() != c.poles.size()) return false;
  for (std::size_t i = 1; i < c.knots.size(); ++i)
    if (c.knots[i] < c.knots[i - 1] - 1e-15) return false;                        // non-decreasing
  // Clamped ends: first/last knot repeated degree+1 times.
  for (int i = 1; i <= c.degree; ++i) {
    if (std::fabs(c.knots[i] - c.knots[0]) > 1e-15) return false;
    if (std::fabs(c.knots[c.knots.size() - 1 - i] - c.knots.back()) > 1e-15) return false;
  }
  for (double w : c.weights) if (!(w > 0.0)) return false;                        // positive weights
  return true;
}
bool surfaceValid(const nm::BsplineSurfaceData& s) {
  if (s.degreeU < 1 || s.degreeV < 1) return false;
  if (s.nPolesU < s.degreeU + 1 || s.nPolesV < s.degreeV + 1) return false;
  if (static_cast<int>(s.poles.size()) != s.nPolesU * s.nPolesV) return false;
  if (s.knotsU.size() != static_cast<std::size_t>(s.nPolesU) + s.degreeU + 1) return false;
  if (s.knotsV.size() != static_cast<std::size_t>(s.nPolesV) + s.degreeV + 1) return false;
  if (!s.weights.empty() && static_cast<int>(s.weights.size()) != s.nPolesU * s.nPolesV) return false;
  for (double w : s.weights) if (!(w > 0.0)) return false;
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// RANDOM VALID GENERATORS.
// ═════════════════════════════════════════════════════════════════════════════

// Clamped flat knot vector on [0,1]: (degree+1) zeros, `interior` evenly-spaced
// (optionally jittered) interior knots, (degree+1) ones. Length nPoles+degree+1
// with nPoles = interior + degree + 1.  interior ∈ [0 .. some max].
std::vector<double> clampedFlat(Rng& rng, int degree, int interior, bool jitter) {
  std::vector<double> flat;
  for (int i = 0; i <= degree; ++i) flat.push_back(0.0);
  for (int i = 1; i <= interior; ++i) {
    double t = double(i) / (interior + 1);
    if (jitter) t = std::clamp(t + rng.range(-0.4, 0.4) / (interior + 1), 1e-3, 1.0 - 1e-3);
    flat.push_back(t);
  }
  std::sort(flat.begin() + degree + 1, flat.end());
  for (int i = 0; i <= degree; ++i) flat.push_back(1.0);
  return flat;
}

nm::BsplineCurveData randomCurve(Rng& rng, bool rational) {
  nm::BsplineCurveData c;
  c.degree = rng.irange(1, 5);
  const int interior = rng.irange(0, 4);
  const int nPoles = interior + c.degree + 1;
  for (int i = 0; i < nPoles; ++i)
    c.poles.push_back({rng.range(-10, 10), rng.range(-10, 10), rng.range(-10, 10)});
  c.knots = clampedFlat(rng, c.degree, interior, /*jitter=*/interior > 0 && rng.below(2) == 0);
  if (rational)
    for (int i = 0; i < nPoles; ++i) c.weights.push_back(rng.range(0.2, 5.0));  // strictly positive
  return c;
}

nm::BsplineSurfaceData randomSurface(Rng& rng, bool rational) {
  nm::BsplineSurfaceData s;
  s.degreeU = rng.irange(1, 3);
  s.degreeV = rng.irange(1, 3);
  const int intU = rng.irange(0, 2), intV = rng.irange(0, 2);
  s.nPolesU = intU + s.degreeU + 1;
  s.nPolesV = intV + s.degreeV + 1;
  for (int i = 0; i < s.nPolesU; ++i)
    for (int j = 0; j < s.nPolesV; ++j)
      s.poles.push_back({double(i) + rng.range(-0.3, 0.3), double(j) + rng.range(-0.3, 0.3),
                         rng.range(-3, 3)});
  s.knotsU = clampedFlat(rng, s.degreeU, intU, /*jitter=*/intU > 0 && rng.below(2) == 0);
  s.knotsV = clampedFlat(rng, s.degreeV, intV, /*jitter=*/intV > 0 && rng.below(2) == 0);
  if (rational)
    for (int i = 0; i < s.nPolesU * s.nPolesV; ++i) s.weights.push_back(rng.range(0.2, 5.0));
  return s;
}

// Known rational fixtures — exact circular geometry the generator would rarely hit.
nm::BsplineCurveData fixtureQuarterArc() {  // 90° arc, R=2, w=cos45.
  nm::BsplineCurveData c;
  c.degree = 2;
  c.poles = {{2, 0, 0}, {2, 2, 0}, {0, 2, 0}};
  c.weights = {1.0, std::sqrt(2.0) / 2.0, 1.0};
  c.knots = {0, 0, 0, 1, 1, 1};
  return c;
}
nm::BsplineSurfaceData fixtureQuarterCylinder() {  // rational-quadratic arc (U) × linear (V).
  const double R = 3.0, w = std::sqrt(2.0) / 2.0;
  const nm::Point3 arc[3] = {{R, 0, 0}, {R, R, 0}, {0, R, 0}};
  nm::BsplineSurfaceData s;
  s.degreeU = 2; s.degreeV = 1; s.nPolesU = 3; s.nPolesV = 2;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j) s.poles.push_back({arc[i].x, arc[i].y, j == 0 ? 0.0 : 4.0});
  const double wa[3] = {1.0, w, 1.0};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 2; ++j) s.weights.push_back(wa[i]);
  s.knotsU = {0, 0, 0, 1, 1, 1};
  s.knotsV = {0, 0, 1, 1};
  return s;
}

// Random interior parameter strictly inside (lo,hi), avoiding the very ends.
double randomInterior(Rng& rng, const std::vector<double>& knots) {
  const double lo = loParam(knots), hi = hiParam(knots);
  return rng.range(lo + 0.1 * (hi - lo), hi - 0.1 * (hi - lo));
}
// Multiplicity of value u already present in a flat knot vector.
int multiplicityOf(const std::vector<double>& knots, double u) {
  int m = 0;
  for (double k : knots) if (std::fabs(k - u) < 1e-9) ++m;
  return m;
}
// Distinct interior spans (b>a) of a flat knot vector — the expected Bézier count.
int distinctInteriorSpans(const std::vector<double>& knots) {
  int n = 0;
  for (std::size_t i = 1; i < knots.size(); ++i)
    if (knots[i] - knots[i - 1] > 1e-12) ++n;
  return n;
}

// ═════════════════════════════════════════════════════════════════════════════
// Trial accounting + reporting.
// ═════════════════════════════════════════════════════════════════════════════
enum Bucket { B_INSERT, B_REFINE, B_ELEVATE, B_SPLIT, B_DECOMPOSE, B_REPARAM,
              B_SURF, B_CHAIN, B_ROUNDTRIP, B_HONESTY, B_COUNT };
const char* bucketName(int b) {
  switch (b) {
    case B_INSERT:    return "single/insert";
    case B_REFINE:    return "single/refine";
    case B_ELEVATE:   return "single/elevate";
    case B_SPLIT:     return "single/split";
    case B_DECOMPOSE: return "single/decompose";
    case B_REPARAM:   return "single/reparam";
    case B_SURF:      return "single/surface";
    case B_CHAIN:     return "composed-chain";
    case B_ROUNDTRIP: return "round-trip";
    case B_HONESTY:   return "honesty";
  }
  return "?";
}
int g_agreed = 0, g_disagreed = 0, g_declined = 0, g_oracle = 0;
int g_bA[B_COUNT] = {0}, g_bX[B_COUNT] = {0}, g_bD[B_COUNT] = {0};

void reportDisagree(int bucket, int idx, std::uint64_t seed, const char* what, const char* chain) {
  std::printf("[FUZZ] DISAGREED  bucket=%-15s case=%-4d %s\n"
              "       REPRO seed=0x%llx index=%d op-chain=[%s]\n",
              bucketName(bucket), idx, what, static_cast<unsigned long long>(seed), idx, chain);
  std::fflush(stdout);
}
void agree(int bucket) { ++g_agreed; ++g_bA[bucket]; }
void decline(int bucket) { ++g_declined; ++g_bD[bucket]; }
bool disagree(int bucket, int idx, std::uint64_t seed, const char* what, const char* chain = "") {
  reportDisagree(bucket, idx, seed, what, chain);
  ++g_disagreed; ++g_bX[bucket];
  return false;
}

// ═════════════════════════════════════════════════════════════════════════════
// SINGLE-OP curve trials. Each returns false only on a DISAGREED (real finding).
// ═════════════════════════════════════════════════════════════════════════════
bool trialInsert(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  nm::BsplineCurveData c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_INSERT, idx, seed, "generator emitted invalid curve");
  const double u = randomInterior(rng, c.knots);
  const int room = c.degree - multiplicityOf(c.knots, u);
  const int r = std::max(1, rng.irange(1, std::max(1, room)));
  nm::BsplineCurveData res = nm::insertKnotCurve(c, u, r);
  if (!curveValid(res)) return disagree(B_INSERT, idx, seed, "insert produced invalid curve");
  if (res.poles.size() != c.poles.size() + static_cast<std::size_t>(r))
    return disagree(B_INSERT, idx, seed, "insert did not add exactly r poles");
  const double dev = curveDeviation(c, res);
  if (dev > kExactTol) return disagree(B_INSERT, idx, seed, "insert changed the geometry");
  agree(B_INSERT);
  return true;
}
bool trialRefine(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  nm::BsplineCurveData c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_REFINE, idx, seed, "generator emitted invalid curve");
  const double lo = loParam(c.knots), hi = hiParam(c.knots);
  std::vector<double> nk;
  const int m = rng.irange(1, 5);
  for (int i = 0; i < m; ++i) nk.push_back(rng.range(lo + 0.05 * (hi - lo), hi - 0.05 * (hi - lo)));
  std::sort(nk.begin(), nk.end());
  nm::BsplineCurveData res = nm::refineKnotCurve(c, std::span<const double>(nk));
  if (!curveValid(res)) return disagree(B_REFINE, idx, seed, "refine produced invalid curve");
  if (res.poles.size() != c.poles.size() + nk.size())
    return disagree(B_REFINE, idx, seed, "refine did not add one pole per new knot");
  if (curveDeviation(c, res) > kExactTol)
    return disagree(B_REFINE, idx, seed, "refine changed the geometry");
  agree(B_REFINE);
  return true;
}
bool trialElevate(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  nm::BsplineCurveData c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_ELEVATE, idx, seed, "generator emitted invalid curve");
  const int t = rng.irange(1, 3);
  nm::BsplineCurveData res = nm::elevateDegreeCurve(c, t);
  if (!curveValid(res)) return disagree(B_ELEVATE, idx, seed, "elevate produced invalid curve");
  if (res.degree != c.degree + t)
    return disagree(B_ELEVATE, idx, seed, "elevate did not raise degree by exactly t");
  if (curveDeviation(c, res) > kExactTol)
    return disagree(B_ELEVATE, idx, seed, "elevate changed the geometry");
  agree(B_ELEVATE);
  return true;
}
bool trialSplit(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  nm::BsplineCurveData c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_SPLIT, idx, seed, "generator emitted invalid curve");
  const double lo = loParam(c.knots), hi = hiParam(c.knots);
  const double u = randomInterior(rng, c.knots);
  const double f = (u - lo) / (hi - lo);
  nm::CurveSplit sp = nm::splitCurve(c, u);
  if (!curveValid(sp.left) || !curveValid(sp.right))
    return disagree(B_SPLIT, idx, seed, "split produced an invalid piece");
  if (curveDeviationSlice(c, 0.0, f, sp.left) > kExactTol)
    return disagree(B_SPLIT, idx, seed, "left piece does not reconstruct source slice");
  if (curveDeviationSlice(c, f, 1.0, sp.right) > kExactTol)
    return disagree(B_SPLIT, idx, seed, "right piece does not reconstruct source slice");
  // C0 join: left(end) == right(start) == C(u).
  const nm::Point3 joinL = evalCurve(sp.left, hiParam(sp.left.knots));
  const nm::Point3 joinR = evalCurve(sp.right, loParam(sp.right.knots));
  const nm::Point3 joinC = evalCurve(c, u);
  if (ptErr(joinL, joinC) > kExactTol || ptErr(joinR, joinC) > kExactTol)
    return disagree(B_SPLIT, idx, seed, "split pieces do not share a C0 join at u");
  agree(B_SPLIT);
  return true;
}
bool trialDecompose(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  nm::BsplineCurveData c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_DECOMPOSE, idx, seed, "generator emitted invalid curve");
  std::vector<nm::BsplineCurveData> segs = nm::decomposeCurveToBezier(c);
  const int expect = distinctInteriorSpans(c.knots);
  if (static_cast<int>(segs.size()) != expect)
    return disagree(B_DECOMPOSE, idx, seed, "segment count != distinct interior spans");
  // Each Bézier segment re-evaluates to the source on its corresponding span. The
  // spans partition [lo,hi] left→right in the same order the segments are emitted.
  std::vector<double> distinct;
  for (double k : c.knots)
    if (distinct.empty() || k - distinct.back() > 1e-12) distinct.push_back(k);
  for (std::size_t si = 0; si + 1 < distinct.size() && si < segs.size(); ++si) {
    if (!curveValid(segs[si]))
      return disagree(B_DECOMPOSE, idx, seed, "decompose produced an invalid Bezier segment");
    const double a = distinct[si], b = distinct[si + 1];
    const double lo = loParam(c.knots), hi = hiParam(c.knots);
    const double f0 = (a - lo) / (hi - lo), f1 = (b - lo) / (hi - lo);
    if (curveDeviationSlice(c, f0, f1, segs[si]) > kExactTol)
      return disagree(B_DECOMPOSE, idx, seed, "Bezier segment does not re-evaluate to source on its span");
  }
  agree(B_DECOMPOSE);
  return true;
}
bool trialReparam(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  nm::BsplineCurveData c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_REPARAM, idx, seed, "generator emitted invalid curve");
  const double a = rng.range(-5, 3), b = a + rng.range(0.5, 8.0);  // a<b
  nm::BsplineCurveData res = nm::reparamCurve(c, a, b);
  if (!curveValid(res)) return disagree(B_REPARAM, idx, seed, "reparam produced invalid curve");
  if (std::fabs(loParam(res.knots) - a) > 1e-9 || std::fabs(hiParam(res.knots) - b) > 1e-9)
    return disagree(B_REPARAM, idx, seed, "reparam did not remap knot domain to [a,b]");
  // Poles/weights unchanged: geometry identical up to the affine parameter remap,
  // which curveDeviation compares at matched fractions.
  if (curveDeviation(c, res) > kExactTol)
    return disagree(B_REPARAM, idx, seed, "reparam changed the geometry");
  agree(B_REPARAM);
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// SINGLE-OP surface trials (insert / elevate / split, U and V) — exact-preserving.
// ═════════════════════════════════════════════════════════════════════════════
bool trialSurface(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  // Mix random surfaces with the known quarter-cylinder rational fixture.
  nm::BsplineSurfaceData s = (rational && rng.below(4) == 0) ? fixtureQuarterCylinder()
                                                            : randomSurface(rng, rational);
  if (!surfaceValid(s)) return disagree(B_SURF, idx, seed, "generator emitted invalid surface");
  const nm::ParamDir d = (rng.below(2) == 0) ? nm::ParamDir::U : nm::ParamDir::V;
  const std::vector<double>& kn = (d == nm::ParamDir::U) ? s.knotsU : s.knotsV;
  const double val = randomInterior(rng, kn);
  const int op = rng.irange(0, 2);
  if (op == 0) {  // insert
    nm::BsplineSurfaceData res = nm::insertKnotSurface(s, d, val, 1);
    if (!surfaceValid(res)) return disagree(B_SURF, idx, seed, "surface insert produced invalid surface");
    if (surfaceDeviation(s, res) > kExactTol)
      return disagree(B_SURF, idx, seed, "surface insert changed the geometry");
  } else if (op == 1) {  // elevate
    nm::BsplineSurfaceData res = nm::elevateDegreeSurface(s, d, 1);
    if (!surfaceValid(res)) return disagree(B_SURF, idx, seed, "surface elevate produced invalid surface");
    const int gotDeg = (d == nm::ParamDir::U) ? res.degreeU : res.degreeV;
    const int expDeg = ((d == nm::ParamDir::U) ? s.degreeU : s.degreeV) + 1;
    if (gotDeg != expDeg) return disagree(B_SURF, idx, seed, "surface elevate did not raise degree by 1");
    if (surfaceDeviation(s, res) > kExactTol)
      return disagree(B_SURF, idx, seed, "surface elevate changed the geometry");
  } else {  // split
    nm::SurfaceSplit sp = nm::splitSurface(s, d, val);
    if (!surfaceValid(sp.low) || !surfaceValid(sp.high))
      return disagree(B_SURF, idx, seed, "surface split produced an invalid piece");
    // Each half must reconstruct its slice of the source. Compare on the sub-rect.
    const double lo = loParam(kn), hi = hiParam(kn);
    const double f = (val - lo) / (hi - lo);
    auto sliceDev = [&](const nm::BsplineSurfaceData& piece, double g0, double g1) {
      const int n = 10;
      double e = 0.0;
      const double pUlo = loParam(piece.knotsU), pUhi = hiParam(piece.knotsU);
      const double pVlo = loParam(piece.knotsV), pVhi = hiParam(piece.knotsV);
      const double sUlo = loParam(s.knotsU), sUhi = hiParam(s.knotsU);
      const double sVlo = loParam(s.knotsV), sVhi = hiParam(s.knotsV);
      for (int a = 0; a <= n; ++a)
        for (int b = 0; b <= n; ++b) {
          const double ga = double(a) / n, gb = double(b) / n;
          double pu = pUlo + (pUhi - pUlo) * ga, pv = pVlo + (pVhi - pVlo) * gb;
          // Map the piece fraction back to the source fraction in the split dir.
          double su, sv;
          if (d == nm::ParamDir::U) { su = sUlo + (sUhi - sUlo) * (g0 + (g1 - g0) * ga); sv = sVlo + (sVhi - sVlo) * gb; }
          else                     { su = sUlo + (sUhi - sUlo) * ga; sv = sVlo + (sVhi - sVlo) * (g0 + (g1 - g0) * gb); }
          e = std::max(e, ptErr(evalSurface(s, su, sv), evalSurface(piece, pu, pv)));
        }
      return e;
    };
    if (sliceDev(sp.low, 0.0, f) > kExactTol || sliceDev(sp.high, f, 1.0) > kExactTol)
      return disagree(B_SURF, idx, seed, "surface split piece does not reconstruct its slice");
  }
  agree(B_SURF);
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// COMPOSED-CHAIN trial (the key differentiator): apply a RANDOM SEQUENCE of exact
// ops and assert the composite still preserves the geometry pointwise end-to-end.
// Compared at MATCHED domain fractions, so intervening reparam/split re-scaling of
// the domain is accounted for. Split takes the LEFT piece so the chain continues.
// ═════════════════════════════════════════════════════════════════════════════
bool trialChain(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  const nm::BsplineCurveData orig = randomCurve(rng, rational);
  if (!curveValid(orig)) return disagree(B_CHAIN, idx, seed, "generator emitted invalid curve");
  nm::BsplineCurveData cur = orig;
  // The reference slice [refLo,refHi] (as fractions of orig's domain) that `cur`
  // currently represents — a split narrows it, other ops leave it unchanged.
  double refLo = 0.0, refHi = 1.0;
  const int steps = rng.irange(3, 6);
  std::string chain;
  for (int st = 0; st < steps; ++st) {
    const int op = rng.irange(0, 4);  // 0 insert,1 refine,2 elevate,3 reparam,4 split
    if (op == 0) {
      const double u = randomInterior(rng, cur.knots);
      cur = nm::insertKnotCurve(cur, u, 1);
      chain += "insert ";
    } else if (op == 1) {
      const double lo = loParam(cur.knots), hi = hiParam(cur.knots);
      std::vector<double> nk = {rng.range(lo + 0.1 * (hi - lo), hi - 0.1 * (hi - lo)),
                                rng.range(lo + 0.1 * (hi - lo), hi - 0.1 * (hi - lo))};
      std::sort(nk.begin(), nk.end());
      cur = nm::refineKnotCurve(cur, std::span<const double>(nk));
      chain += "refine ";
    } else if (op == 2) {
      if (cur.degree < 5) { cur = nm::elevateDegreeCurve(cur, 1); chain += "elevate "; }
      else { continue; }
    } else if (op == 3) {
      const double a = rng.range(-3, 2), b = a + rng.range(0.5, 5.0);
      cur = nm::reparamCurve(cur, a, b);
      chain += "reparam ";
    } else {
      const double lo = loParam(cur.knots), hi = hiParam(cur.knots);
      const double u = randomInterior(rng, cur.knots);
      const double f = (u - lo) / (hi - lo);
      nm::CurveSplit sp = nm::splitCurve(cur, u);
      cur = sp.left;                       // keep the left piece, continue chaining
      refHi = refLo + (refHi - refLo) * f; // narrow the reference window
      chain += "split-L ";
    }
    if (!curveValid(cur))
      return disagree(B_CHAIN, idx, seed, "op-chain produced an invalid curve", chain.c_str());
  }
  // End-to-end: `cur` over its whole domain must equal `orig` over [refLo,refHi].
  const double dev = curveDeviationSlice(orig, refLo, refHi, cur);
  if (dev > kExactTol)
    return disagree(B_CHAIN, idx, seed, "op-chain changed the geometry end-to-end", chain.c_str());
  agree(B_CHAIN);
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// ROUND-TRIP trials — insert↔remove and elevate↔reduce identities.
//   insert(u,r) then removeKnot(u,r) recovers the original net (removed==r, err≤tol).
//   elevate(1) then reduce recovers the original degree on a reducible curve.
// A curve that is NOT reducible reports ok=false honestly → HONESTLY-DECLINED.
// ═════════════════════════════════════════════════════════════════════════════
bool trialRoundTrip(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  nm::BsplineCurveData c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_ROUNDTRIP, idx, seed, "generator emitted invalid curve");

  if (rng.below(2) == 0) {
    // insert↔remove: insert a fresh knot r times, then remove it r times.
    const double u = randomInterior(rng, c.knots);
    const int r = rng.irange(1, c.degree);
    nm::BsplineCurveData inserted = nm::insertKnotCurve(c, u, r);
    nm::KnotRemovalResult rr = nm::removeKnotCurve(inserted, u, r, kOpTol);
    if (rr.removed != r)
      return disagree(B_ROUNDTRIP, idx, seed, "insert-then-remove did not remove all r inserted knots");
    if (rr.maxError > kOpTol)
      return disagree(B_ROUNDTRIP, idx, seed, "removeKnot maxError exceeds its own reported tol");
    if (!curveValid(rr.curve))
      return disagree(B_ROUNDTRIP, idx, seed, "removeKnot produced invalid curve");
    if (curveNetErr(c, rr.curve) > kNetTol)
      return disagree(B_ROUNDTRIP, idx, seed, "insert-then-remove did not recover the original net");
    if (curveDeviation(c, rr.curve) > kExactTol)
      return disagree(B_ROUNDTRIP, idx, seed, "insert-then-remove changed the geometry");
    agree(B_ROUNDTRIP);
    return true;
  }

  // elevate↔reduce: elevate by 1 (result is genuinely reducible), then reduce must
  // succeed (ok=true) and recover the original degree and geometry.
  nm::BsplineCurveData elevated = nm::elevateDegreeCurve(c, 1);
  nm::DegreeReduceResult dr = nm::reduceDegreeCurve(elevated, kOpTol);
  if (!dr.ok)
    return disagree(B_ROUNDTRIP, idx, seed, "reduce failed on a genuinely-reducible (just-elevated) curve");
  if (dr.maxError > kOpTol)
    return disagree(B_ROUNDTRIP, idx, seed, "reduce maxError exceeds its own reported tol");
  if (!curveValid(dr.curve))
    return disagree(B_ROUNDTRIP, idx, seed, "reduce produced invalid curve");
  if (dr.curve.degree != c.degree)
    return disagree(B_ROUNDTRIP, idx, seed, "elevate-then-reduce did not recover the original degree");
  if (curveDeviation(c, dr.curve) > kExactTol)
    return disagree(B_ROUNDTRIP, idx, seed, "elevate-then-reduce changed the geometry");
  agree(B_ROUNDTRIP);
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// HONESTY trials — reduce / remove must DECLINE honestly, not falsely "succeed".
//   * reduce on a generic degree-≥2 random curve is (almost surely) NOT exactly
//     reducible: it must report ok=false AND leave the geometry unchanged, OR — in
//     the rare case it IS reducible — report ok=true with maxError≤tol AND a
//     geometry-preserving reduction. Either is HONEST; a false ok=true that MOVED
//     the geometry beyond tol is a DISAGREED.
//   * removeKnot of a knot that cannot be removed within tol must leave the curve
//     unchanged and report removed==0 (HONESTLY-DECLINED).
// ═════════════════════════════════════════════════════════════════════════════
bool trialHonesty(Rng& rng, int idx, std::uint64_t seed, bool rational) {
  if (rng.below(2) == 0) {
    // Generic curve, degree ≥ 2, random poles: almost surely NOT exactly reducible.
    nm::BsplineCurveData c = randomCurve(rng, rational);
    while (c.degree < 2) c = randomCurve(rng, rational);
    if (!curveValid(c)) return disagree(B_HONESTY, idx, seed, "generator emitted invalid curve");
    nm::DegreeReduceResult dr = nm::reduceDegreeCurve(c, kOpTol);
    if (dr.ok) {
      // It CLAIMS reducible: then it must be honest — geometry preserved to tol and
      // degree actually dropped by one. Otherwise the "ok" is a lie (DISAGREED).
      if (!curveValid(dr.curve)) return disagree(B_HONESTY, idx, seed, "reduce ok=true but invalid curve");
      if (dr.curve.degree != c.degree - 1)
        return disagree(B_HONESTY, idx, seed, "reduce ok=true but degree not reduced by one");
      if (curveDeviation(c, dr.curve) > std::max(kExactTol, dr.maxError))
        return disagree(B_HONESTY, idx, seed, "reduce ok=true but geometry moved beyond reported error");
      agree(B_HONESTY);            // a rare genuinely-reducible random curve, handled honestly
      return true;
    }
    // ok=false: the honest decline. The result curve must be the unchanged input
    // (design: SHALL NOT return a differing lower-degree curve presented as exact).
    if (curveValid(dr.curve) && curveNetErr(c, dr.curve) > kNetTol)
      return disagree(B_HONESTY, idx, seed, "reduce ok=false but returned a mutated curve");
    decline(B_HONESTY);
    return true;
  }

  // removeKnot on an ORIGINAL interior knot the net genuinely needs (random curve
  // with an interior knot): removing it usually cannot stay within tol → it must
  // decline (removed==0) and leave the curve unchanged. If it CAN remove (removed>0)
  // it must have stayed within tol AND preserved the geometry.
  nm::BsplineCurveData c = randomCurve(rng, rational);
  // Ensure at least one interior knot to target.
  for (int guard = 0; guard < 8 && distinctInteriorSpans(c.knots) < 2; ++guard)
    c = randomCurve(rng, rational);
  if (!curveValid(c)) return disagree(B_HONESTY, idx, seed, "generator emitted invalid curve");
  // Pick an interior distinct knot value.
  double u = -1.0;
  for (std::size_t i = 1; i < c.knots.size(); ++i)
    if (c.knots[i] > loParam(c.knots) + 1e-9 && c.knots[i] < hiParam(c.knots) - 1e-9) { u = c.knots[i]; break; }
  if (u < 0.0) { decline(B_HONESTY); return true; }  // no interior knot: nothing to try, honest no-op
  nm::KnotRemovalResult rr = nm::removeKnotCurve(c, u, 1, kOpTol);
  if (rr.removed == 0) {
    // Declined: curve must be unchanged.
    if (curveValid(rr.curve) && curveNetErr(c, rr.curve) > kNetTol)
      return disagree(B_HONESTY, idx, seed, "removeKnot removed==0 but mutated the curve");
    decline(B_HONESTY);
    return true;
  }
  // Removed within tol: honest success — geometry must be preserved to the reported error.
  if (!curveValid(rr.curve)) return disagree(B_HONESTY, idx, seed, "removeKnot removed>0 but invalid curve");
  if (rr.maxError > kOpTol)
    return disagree(B_HONESTY, idx, seed, "removeKnot removed>0 but maxError exceeds tol");
  if (curveDeviation(c, rr.curve) > std::max(kExactTol, rr.maxError))
    return disagree(B_HONESTY, idx, seed, "removeKnot removed>0 but geometry moved beyond reported error");
  agree(B_HONESTY);
  return true;
}

// One fuzz trial: pick a family and rationality, dispatch. Families are chosen so
// every bucket gets real coverage across the batch.
void runTrial(Rng& rng, int idx, std::uint64_t seed) {
  const bool rational = (rng.below(2) == 0);
  switch (idx % 10) {
    case 0: (void)trialInsert(rng, idx, seed, rational); break;
    case 1: (void)trialRefine(rng, idx, seed, rational); break;
    case 2: (void)trialElevate(rng, idx, seed, rational); break;
    case 3: (void)trialSplit(rng, idx, seed, rational); break;
    case 4: (void)trialDecompose(rng, idx, seed, rational); break;
    case 5: (void)trialReparam(rng, idx, seed, rational); break;
    case 6: (void)trialSurface(rng, idx, seed, rational); break;
    case 7: (void)trialChain(rng, idx, seed, rational); break;
    case 8: (void)trialRoundTrip(rng, idx, seed, rational); break;
    case 9: (void)trialHonesty(rng, idx, seed, rational); break;
  }
}

}  // namespace

// A single CC_TEST drives the whole seeded batch (the generator IS the test). Two
// default seeds prove seed-independence; FUZZ_SEED / FUZZ_N override for a targeted
// repro. Bar: DISAGREED==0 with real coverage across ≥2 seeds and every bucket ≥1.
CC_TEST(nurbs_ops_differential_fuzz) {
  std::uint64_t seeds[2] = {0x0BEEF5EED11CEull, 0xC0FFEE5A11ADull};
  int N = 90;  // ≥60/seed; 10 buckets → ≥9 of each per seed.
  if (const char* e = std::getenv("FUZZ_SEED")) { seeds[0] = std::strtoull(e, nullptr, 0); seeds[1] = seeds[0]; }
  if (const char* e = std::getenv("FUZZ_N")) { const int n = std::atoi(e); if (n > 0) N = n; }

  std::printf("== NURBS-L1 HOST differential fuzz: bspline_ops over random valid B-spline/NURBS ==\n");
  std::printf("== oracle = the module's OWN sampled geometry; EXACT ops must not move it; kExactTol=%.0e FIXED — NEVER widened ==\n",
              kExactTol);

  const int nSeeds = (seeds[0] == seeds[1]) ? 1 : 2;
  for (int si = 0; si < nSeeds; ++si) {
    Rng rng(seeds[si]);
    std::printf("\n-- seed=0x%llx N=%d --\n", static_cast<unsigned long long>(seeds[si]), N);
    for (int i = 0; i < N; ++i) runTrial(rng, i, seeds[si]);
  }

  std::printf("\n== COVERAGE: AGREED=%d HONESTLY-DECLINED=%d ORACLE-INACCURATE=%d DISAGREED=%d ==\n",
              g_agreed, g_declined, g_oracle, g_disagreed);
  std::printf("   per-bucket [AGREED / HONESTLY-DECLINED / DISAGREED]:\n");
  bool coverage = true;
  for (int b = 0; b < B_COUNT; ++b) {
    std::printf("     %-16s %d / %d / %d\n", bucketName(b), g_bA[b], g_bD[b], g_bX[b]);
    // Every bucket must be exercised (AGREED or, for honesty, a legit DECLINE).
    if (g_bA[b] + g_bD[b] < 1) coverage = false;
  }
  CC_CHECK(nSeeds >= 2);        // seed-independence: at least two seeds unless overridden
  CC_CHECK(coverage);          // every op family exercised at least once
  CC_CHECK_EQ(g_disagreed, 0); // THE BAR: zero geometry-changing exact ops / broken identities
}

CC_RUN_ALL()
