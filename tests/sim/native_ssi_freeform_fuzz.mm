// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_freeform_fuzz.mm — GENERAL FREEFORM SSI differential fuzzer (iOS
// simulator). The empirical instrument for NURBS roadmap Layer 2: general
// NURBS↔NURBS surface-surface intersection.
//
// WHAT IT DOES. It GENERATES random valid NURBS↔NURBS surface pairs (biased toward
// actually intersecting, mostly transversal), runs the NATIVE SSI pipeline (S2
// seed_intersection → S3 trace_intersection, the real marcher) against OCCT
// (GeomAPI_IntSS) as the ORACLE, and classifies EVERY trial into exactly one bucket:
//
//   AGREED             — native traced branch(es) whose nodes all lie on BOTH surfaces
//                        within tol AND whose coverage matches the OCCT locus (native
//                        curves cover the OCCT lines and vice-versa within the traced
//                        set). The honest "native got it right" bucket.
//   HONESTLY-DECLINED  — native declined or returned an incomplete trace (nearTangentGaps
//                        > 0, empty seeds on a case OCCT DOES find an intersection for, or a
//                        NearTangent/Failed status) → this case routes to OCCT. COUNTED,
//                        first-class, NOT a failure. Recorded WITH a reason (near-tangent /
//                        multi-branch / small-loop / no-seed / other) so the decline reasons
//                        aggregate into the S4 work-list. A high decline rate is FINE and
//                        EXPECTED — the bar is DISAGREED==0, not a low decline rate.
//   DISAGREED          — native returned a trace it considered COMPLETE, but a node is NOT
//                        on both surfaces (silent-wrong geometry), OR native claims
//                        transversal completeness (all-Closed/BoundaryExit, no gaps) yet
//                        OCCT finds an intersection native traced NOTHING for, or native
//                        traced a curve OCCT contradicts (an OCCT locus no native curve
//                        covers, while native declared itself complete). This is the failure
//                        the bar FORBIDS.
//   ORACLE-INACCURATE  — native is MORE correct than OCCT at a numeric edge (native's nodes
//                        verify on both surfaces to << tol, but OCCT's line sits off native's
//                        by more than OCCT's own tol). Justified inline; rare.
//
// THE BAR: DISAGREED == 0 across ≥ 2 seeds, N ≥ 40 per seed. Native must NEVER trace a
// curve that isn't on both surfaces, and must never declare completeness while silently
// missing an OCCT locus. A high HONESTLY-DECLINED rate does NOT fail the bar. Exit 0 iff
// DISAGREED == 0. Any DISAGREED prints seed + case index + the two surfaces' defining data
// (degrees, poles, weights, knots) so it reproduces.
//
// FIXED TOLERANCES (stated; NEVER widened to manufacture agreement):
//   * onSurf   = 1e-6   — a native node's max ‖node − surface‖ over BOTH surfaces.
//   * onCurve  = 1e-3   — coverage radius (native↔OCCT nearest-point), the marching-parity
//                         coverage budget (a fitted-B-spline bow between nodes rides ~1e-4).
//   * occtTol  = 1e-7   — the tolerance handed to GeomAPI_IntSS.
//
// DETERMINISM. splitmix64-seeded xoshiro256** stream keyed ONLY by an explicit FUZZ_SEED
// (argv[1] / env) — NO clock, NO rand(), NO address. Same seed → byte-identical run. The
// seed + case index are printed on every DISAGREED so any finding reproduces.
//
// SSI is INTERNAL — NO cc_* entry point is called or added; this is a SIM (native-vs-OCCT)
// test harness. It does NOT modify src/native (which stays OCCT-free). Built ONLY by
// scripts/run-sim-native-ssi-freeform-fuzz.sh; on the SKIP list of run-sim-suite.sh.
//
// This TU is OCCT-dependent AND substrate-dependent: it links the OCCT oracle and the
// NumPP/SciPP numsci archive, and compiles src/native/ssi/{seeding,marching}.cpp +
// src/native/numerics/numerics.cpp under -DCYBERCAD_HAS_NUMSCI. Flushes and std::_Exit
// (OCCT static teardown in the trimmed static build is not exit-clean — same rationale as
// the SSI parity harnesses).
//
#include "native/ssi/native_ssi.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_ssi_freeform_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_ssi_freeform_fuzz requires -DCYBERCAD_HAS_NUMSCI (the least_squares corrector + lstsq fit)"
#endif

#include <gp_Pnt.hxx>
#include <Geom_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Curve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <Extrema_ExtAlgo.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TColStd_Array2OfReal.hxx>

namespace ssi = cybercad::native::ssi;
namespace nm = cybercad::native::math;
using nm::Point3;

namespace {

// ── FIXED tolerances (stated; NEVER widened) ─────────────────────────────────────
constexpr double kOnSurfTol = 1e-6;   // native node on BOTH surfaces
constexpr double kOnCurveTol = 1e-3;  // native↔OCCT curve coverage radius
constexpr double kOcctTol = 1e-7;     // GeomAPI_IntSS tolerance
constexpr int kOcctCurveSamples = 64; // OCCT-line sampling density for coverage
constexpr int kNativeCoverSamples = 32; // native-curve sampling for the reverse coverage
constexpr double kPi = 3.14159265358979323846;

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream (as the siblings) ─────
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
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  int irange(int lo, int hi) { return lo + static_cast<int>(next() % static_cast<uint64_t>(hi - lo + 1)); }
};

// ─────────────────────────────────────────────────────────────────────────────────
// NurbsSurf — a random valid NURBS surface: degrees, pole grid (row-major U-outer),
// optional positive weights (empty ⇒ non-rational), clamped flat knot vectors. This
// is the case's DEFINING DATA (printed verbatim on a DISAGREED so it reproduces).
// ─────────────────────────────────────────────────────────────────────────────────
struct NurbsSurf {
  int degU = 3, degV = 3;
  int nU = 4, nV = 4;                 // poles in each direction
  std::vector<Point3> poles;          // nU*nV, row-major U-outer: poles[i*nV + j]
  std::vector<double> weights;        // empty ⇒ non-rational; else parallel to poles
  std::vector<double> knotsU, knotsV; // clamped flat, length nPoles+deg+1
  bool rational() const { return !weights.empty(); }
};

// Clamped-uniform flat knot vector of length nPoles+degree+1 over [0,1].
std::vector<double> clampedFlat(int degree, int nPoles) {
  const int m = nPoles + degree + 1;
  const int interior = nPoles - degree - 1;  // #interior distinct knots
  std::vector<double> flat;
  flat.reserve(static_cast<std::size_t>(m));
  for (int i = 0; i <= degree; ++i) flat.push_back(0.0);
  for (int i = 1; i <= interior; ++i) flat.push_back(double(i) / double(interior + 1));
  for (int i = 0; i <= degree; ++i) flat.push_back(1.0);
  return flat;
}

// GENERATOR-SANITY (a failure here is a GENERATOR bug, never a native disagreement):
// assert the flat-knot length invariant + clamped ends before the surface is used.
bool validSurf(const NurbsSurf& s) {
  if (s.degU < 1 || s.degV < 1) return false;
  if (s.nU < s.degU + 1 || s.nV < s.degV + 1) return false;
  if (static_cast<int>(s.poles.size()) != s.nU * s.nV) return false;
  if (!s.weights.empty() && static_cast<int>(s.weights.size()) != s.nU * s.nV) return false;
  for (double w : s.weights) if (!(w > 0.0)) return false;
  if (static_cast<int>(s.knotsU.size()) != s.nU + s.degU + 1) return false;
  if (static_cast<int>(s.knotsV.size()) != s.nV + s.degV + 1) return false;
  // clamped ends: first deg+1 knots equal, last deg+1 knots equal, and non-decreasing.
  auto clamped = [](const std::vector<double>& k, int deg) {
    for (int i = 0; i <= deg; ++i)
      if (std::fabs(k[i] - k.front()) > 1e-15 ||
          std::fabs(k[k.size() - 1 - i] - k.back()) > 1e-15)
        return false;
    for (std::size_t i = 1; i < k.size(); ++i)
      if (k[i] < k[i - 1] - 1e-15) return false;
    return k.back() > k.front();
  };
  return clamped(s.knotsU, s.degU) && clamped(s.knotsV, s.degV);
}

// ── case families (bias toward ACTUALLY intersecting, mostly transversal) ──────────
enum Family {
  F_TRANSVERSAL,   // a bump vs a facing dish → a clean transversal loop (usually 1)
  F_TILTED,        // two random freeform sheets tilted through each other → transversal
  F_MULTIBRANCH,   // an egg-carton sheet vs a gently-bowed sheet → several small loops
  F_NEAR_TANGENT,  // two dishes just kissing → a glancing near-tangent pair
  F_DISJOINT,      // vertically separated sheets → empty (control)
  F_COUNT
};
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

// Sprinkle a random weight grid (positive) onto a surface with probability p.
void maybeRational(NurbsSurf& s, Rng& rng, double p) {
  if (rng.unit() >= p) return;
  s.weights.resize(static_cast<std::size_t>(s.nU) * s.nV);
  for (auto& w : s.weights) w = rng.range(0.4, 2.5);
}

// A random pole grid over the bounded box [-1.2,1.2]² in XY, with a per-pole Z given by
// a base height field `zbase(x,y)` plus a small random perturbation `jitter` in Z (and a
// tiny XY wobble so the net is genuinely freeform, not a graph over a regular grid). The
// pole grid is bicubic-ish and VALID by construction (convex-hull-bounded).
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

// Build one random valid NURBS↔NURBS pair for `family`. Returns true and fills A,B.
bool buildPair(int family, Rng& rng, NurbsSurf& A, NurbsSurf& B) {
  auto randDeg = [&]() { return rng.irange(2, 3); };
  auto randN = [&](int deg) { return rng.irange(deg + 1, 6); };  // 3x3..6x6-ish grids

  A.degU = randDeg(); A.degV = randDeg();
  A.nU = randN(A.degU); A.nV = randN(A.degV);
  B.degU = randDeg(); B.degV = randDeg();
  B.nU = randN(B.degU); B.nV = randN(B.degV);

  const double ampA = rng.range(0.6, 1.4);  // bump/dish amplitude
  const double ampB = rng.range(0.6, 1.4);

  switch (family) {
    case F_TRANSVERSAL: {
      // A: a bump opening +Z centred at 0; B: a dish opening −Z, facing it. They cross
      // in a clean transversal loop around the rim where the bump pokes through the dish.
      fillGrid(A, rng, [ampA](double x, double y) { return ampA * (1.0 - 0.5 * (x * x + y * y)); },
               0.12, 0.06);
      fillGrid(B, rng, [ampB](double x, double y) { return 0.9 - ampB * (1.0 - 0.5 * (x * x + y * y)); },
               0.12, 0.06);
      break;
    }
    case F_TILTED: {
      // Two gently curved sheets, each tilted by a random slope through the other, so
      // they cross transversally (usually one open arc or one loop).
      const double sxA = rng.range(-0.6, 0.6), syA = rng.range(-0.6, 0.6);
      const double sxB = rng.range(-0.6, 0.6), syB = rng.range(-0.6, 0.6);
      fillGrid(A, rng, [sxA, syA, ampA](double x, double y) {
        return sxA * x + syA * y + 0.25 * ampA * std::sin(1.3 * x); }, 0.12, 0.07);
      fillGrid(B, rng, [sxB, syB, ampB](double x, double y) {
        return 0.15 + sxB * x + syB * y + 0.25 * ampB * std::cos(1.3 * y); }, 0.12, 0.07);
      break;
    }
    case F_MULTIBRANCH: {
      // A: an egg-carton (alternating high/low) sheet; B: a gently bowed sheet slicing
      // through the humps → several small transversal loops (the multi-branch stressor).
      const double f = rng.range(1.6, 2.4);
      fillGrid(A, rng, [f, ampA](double x, double y) {
        return ampA * 0.5 * std::sin(f * x) * std::sin(f * y); }, 0.05, 0.04);
      fillGrid(B, rng, [ampB](double x, double y) {
        return 0.0 + 0.08 * ampB * (x * x - y * y); }, 0.05, 0.04);
      break;
    }
    case F_NEAR_TANGENT: {
      // A shallow bump sitting in a matching cup of NEARLY-EQUAL curvature: they share a
      // tangent plane over a grazing region and interpenetrate only SHALLOWLY, so the
      // intersection is a small GLANCING loop whose transversality sine dips low — the S4
      // near-tangent stressor. `k` is the shared curvature; `dk` a small curvature mismatch
      // (so a real, tight loop exists); `overlap` a small NEGATIVE gap (shallow penetration).
      // A = k·r²; B = −overlap + (k+dk)·r². Then A−B = overlap − dk·r², which is > 0 near the
      // axis and < 0 beyond r² = overlap/dk → the two paraboloids cross in a SMALL CIRCLE of
      // radius √(overlap/dk). With small overlap and small curvature-mismatch dk the loop is
      // tight and the two surfaces are nearly parallel there (low crossing sine) — a genuine
      // glancing near-tangent contact.
      // A fraction of TIGHTER grazes (small overlap AND small dk → a very small loop where the
      // two paraboloids are nearly parallel, crossing sine near zero) probes the genuine
      // near-tangent STALL; the rest are moderate glancing loops. Never engineered to force a
      // bucket — just a spread across the grazing regime.
      const bool tight = rng.unit() < 0.5;
      const double k = rng.range(0.35, 0.55);
      const double dk = tight ? rng.range(0.015, 0.05) : rng.range(0.05, 0.12);
      const double overlap = tight ? rng.range(0.008, 0.03) : rng.range(0.03, 0.10);
      fillGrid(A, rng, [k](double x, double y) { return k * (x * x + y * y); },
               0.015, 0.02);
      fillGrid(B, rng, [k, dk, overlap](double x, double y) {
        return -overlap + (k + dk) * (x * x + y * y); }, 0.015, 0.02);
      break;
    }
    case F_DISJOINT: {
      // Vertically separated sheets: no intersection (empty control). The gap exceeds the
      // combined amplitude + jitter so the AABBs do not even overlap.
      fillGrid(A, rng, [ampA](double x, double y) { return ampA * 0.3 * (x * x + y * y); },
               0.08, 0.05);
      fillGrid(B, rng, [ampB](double x, double y) { return 5.0 + ampB * 0.3 * (x * x + y * y); },
               0.08, 0.05);
      break;
    }
    default: return false;
  }

  // Rationalize each side independently with moderate probability (disjoint controls
  // stay non-rational — the weights would not change the empty verdict).
  const double pRat = (family == F_DISJOINT) ? 0.0 : 0.5;
  maybeRational(A, rng, pRat);
  maybeRational(B, rng, pRat);
  return validSurf(A) && validSurf(B);
}

// ── native adapter from a NurbsSurf (rational → makeNurbsAdapter, else makeBSpline) ──
ssi::SurfaceAdapter nativeAdapter(const NurbsSurf& s) {
  if (s.rational())
    return ssi::makeNurbsAdapter(s.degU, s.degV, s.poles, s.weights, s.nU, s.nV,
                                 s.knotsU, s.knotsV);
  return ssi::makeBSplineAdapter(s.degU, s.degV, s.poles, s.nU, s.nV, s.knotsU, s.knotsV);
}

// ── OCCT surface from a NurbsSurf (flat → distinct knots + mults; rational uses the
// weights overload). The native/OCCT conventions match native_math_parity: weights are
// row-major U-outer parallel to the poles; OCCT poles/weights are (1..nU, 1..nV). ──
void flatToKnotsMults(const std::vector<double>& flat, std::vector<double>& knots,
                      std::vector<int>& mults) {
  knots.clear(); mults.clear();
  for (double k : flat) {
    if (!knots.empty() && std::fabs(k - knots.back()) < 1e-12) ++mults.back();
    else { knots.push_back(k); mults.push_back(1); }
  }
}

Handle(Geom_BSplineSurface) toOcct(const NurbsSurf& s) {
  std::vector<double> ku, kv; std::vector<int> mu, mv;
  flatToKnotsMults(s.knotsU, ku, mu);
  flatToKnotsMults(s.knotsV, kv, mv);
  TColgp_Array2OfPnt occPoles(1, s.nU, 1, s.nV);
  for (int i = 0; i < s.nU; ++i)
    for (int j = 0; j < s.nV; ++j) {
      const Point3& p = s.poles[static_cast<std::size_t>(i) * s.nV + j];
      occPoles.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  TColStd_Array1OfReal occKu(1, static_cast<int>(ku.size())), occKv(1, static_cast<int>(kv.size()));
  for (int i = 0; i < static_cast<int>(ku.size()); ++i) occKu.SetValue(i + 1, ku[i]);
  for (int i = 0; i < static_cast<int>(kv.size()); ++i) occKv.SetValue(i + 1, kv[i]);
  TColStd_Array1OfInteger occMu(1, static_cast<int>(mu.size())), occMv(1, static_cast<int>(mv.size()));
  for (int i = 0; i < static_cast<int>(mu.size()); ++i) occMu.SetValue(i + 1, mu[i]);
  for (int i = 0; i < static_cast<int>(mv.size()); ++i) occMv.SetValue(i + 1, mv[i]);

  if (s.rational()) {
    TColStd_Array2OfReal occW(1, s.nU, 1, s.nV);
    for (int i = 0; i < s.nU; ++i)
      for (int j = 0; j < s.nV; ++j)
        occW.SetValue(i + 1, j + 1, s.weights[static_cast<std::size_t>(i) * s.nV + j]);
    return new Geom_BSplineSurface(occPoles, occW, occKu, occKv, occMu, occMv, s.degU, s.degV,
                                   Standard_False, Standard_False);
  }
  return new Geom_BSplineSurface(occPoles, occKu, occKv, occMu, occMv, s.degU, s.degV,
                                 Standard_False, Standard_False);
}

// ── OCCT projection helpers ──────────────────────────────────────────────────────
//
// ROBUST surface projection. GeomAPI_ProjectPointOnSurf's default gradient extrema
// (Extrema_ExtAlgo_Grad) can MISS the global nearest point on a wavy freeform NURBS patch
// and return a spuriously LARGE distance for a point that IS on the surface — a well-known
// OCCT search failure, NOT native being wrong (observed: a node on the OCCT intersection
// LINE to 5.7e-8 reporting 1.7e-2 off the surface, which is impossible since the line lies
// on the surface by construction). To measure the TRUE on-surface residual we run BOTH the
// Grad and the Tree (global subdivision) extrema, over the surface's parametric bounds, and
// take the SMALLER distance (the true nearest point is the closer of the two — a larger
// value is always a search miss). This is a harness-side measurement fix; it neither touches
// src/native nor widens the classification tolerance (the 1e-6 gate is unchanged — we only
// make the DISTANCE we compare against it reliable).
double distToOcctSurface(const Handle(Geom_Surface)& s, const Point3& p) {
  const gp_Pnt q(p.x, p.y, p.z);
  Standard_Real u0, u1, v0, v1;
  s->Bounds(u0, u1, v0, v1);
  auto clampInf = [](Standard_Real& a, Standard_Real& b, Standard_Real lo, Standard_Real hi) {
    if (!std::isfinite(a) || a < lo) a = lo;
    if (!std::isfinite(b) || b > hi) b = hi;
  };
  clampInf(u0, u1, -1e6, 1e6);
  clampInf(v0, v1, -1e6, 1e6);
  double best = 1e30;
  for (Extrema_ExtAlgo algo : {Extrema_ExtAlgo_Grad, Extrema_ExtAlgo_Tree}) {
    GeomAPI_ProjectPointOnSurf proj(q, s, u0, u1, v0, v1, algo);
    if (proj.NbPoints() > 0) best = std::min<double>(best, proj.LowerDistance());
  }
  return best;
}
// ROBUST curve distance. GeomAPI_ProjectPointOnCurve only returns ORTHOGONAL feet within the
// curve's param range; when the nearest point is at (or past) an ENDPOINT it can report
// NbPoints()==0, spuriously giving no distance for a node that IS on the curve near its end.
// So we combine the orthogonal projection with a coarse sampling of the curve (including its
// endpoints) and take the smaller — a search miss can only inflate a distance, never shrink
// it. Harness-side measurement fix; the classification tolerance is unchanged.
double distToOcctCurve(const Handle(Geom_Curve)& c, const Point3& p) {
  const gp_Pnt q(p.x, p.y, p.z);
  double best = 1e30;
  GeomAPI_ProjectPointOnCurve proj(q, c);
  if (proj.NbPoints() > 0) best = proj.LowerDistance();
  double f = c->FirstParameter(), l = c->LastParameter();
  if (!std::isfinite(f)) f = -1e6;
  if (!std::isfinite(l)) l = 1e6;
  constexpr int kCurveDistSamples = 48;
  for (int i = 0; i <= kCurveDistSamples; ++i) {
    const double t = f + (l - f) * (double(i) / kCurveDistSamples);
    gp_Pnt s;
    try { s = c->Value(t); } catch (...) { continue; }
    best = std::min(best, q.Distance(s));
  }
  return best;
}

// ── SSI-GRAZE three-way probe: the OCCT curve point nearest a given native world point.
//    Projects `p` onto the OCCT line set (both the analytic projection and a dense sample),
//    returning the nearest OCCT world point + its distance. Used by the DECLINE-DIAG
//    [GRAZE-3WAY] line so an INDEPENDENT host oracle (oracle_graze_threeway) can measure
//    native-root→truth, OCCT-root→truth and native-root→OCCT-root at the graze pinch.
Point3 nearestOcctPoint(const std::vector<Handle(Geom_Curve)>& lines, const Point3& p, double& dOut) {
  const gp_Pnt q(p.x, p.y, p.z);
  double best = 1e30; Point3 bestP = p;
  for (const auto& c : lines) {
    GeomAPI_ProjectPointOnCurve proj(q, c);
    if (proj.NbPoints() > 0 && proj.LowerDistance() < best) {
      best = proj.LowerDistance(); const gp_Pnt s = proj.NearestPoint(); bestP = {s.X(), s.Y(), s.Z()};
    }
    double f = c->FirstParameter(), l = c->LastParameter();
    if (!std::isfinite(f)) f = -1e6;
    if (!std::isfinite(l)) l = 1e6;
    for (int i = 0; i <= 400; ++i) {
      const double t = f + (l - f) * (double(i) / 400.0);
      gp_Pnt s; try { s = c->Value(t); } catch (...) { continue; }
      const double d = q.Distance(s);
      if (d < best) { best = d; bestP = {s.X(), s.Y(), s.Z()}; }
    }
  }
  dOut = best;
  return bestP;
}

// ── native WLine sampling: polyline nodes + fitted-B-spline samples ───────────────
std::vector<Point3> sampleWLine(const ssi::WLine& w) {
  std::vector<Point3> pts;
  for (const auto& p : w.points) pts.push_back(p.point);
  if (w.curve.valid()) {
    const auto& c = w.curve;
    const double t0 = c.knots.front(), t1 = c.knots.back();
    for (int i = 0; i <= kNativeCoverSamples; ++i) {
      const double t = t0 + (t1 - t0) * (double(i) / kNativeCoverSamples);
      pts.push_back(nm::curvePoint(c.degree, c.poles, c.knots, t));
    }
  }
  return pts;
}

// An ORDERED polyline for one WLine — the fitted B-spline densely sampled if valid, else
// the corrected nodes in order. Used for the reverse (OCCT→native) coverage as a point-to-
// SEGMENT distance, so coverage measures "OCCT point near the native CURVE" not "near a
// native SAMPLE VERTEX" (the latter over-reports the gap by up to a half-sample chord — the
// harness bug that made clean transversal loops look like small-loop declines).
std::vector<Point3> orderedPolyline(const ssi::WLine& w) {
  std::vector<Point3> pl;
  if (w.curve.valid()) {
    const auto& c = w.curve;
    const double t0 = c.knots.front(), t1 = c.knots.back();
    const int nSamp = 4 * kNativeCoverSamples;  // dense: segments << the onCurve tol
    for (int i = 0; i <= nSamp; ++i) {
      const double t = t0 + (t1 - t0) * (double(i) / nSamp);
      pl.push_back(nm::curvePoint(c.degree, c.poles, c.knots, t));
    }
  } else {
    for (const auto& p : w.points) pl.push_back(p.point);
  }
  if (w.status == ssi::TraceStatus::Closed && pl.size() >= 2) pl.push_back(pl.front());
  return pl;
}

// Distance from point q to segment [a,b].
double distPointSeg(const Point3& q, const Point3& a, const Point3& b) {
  const nm::Vec3 ab{b.x - a.x, b.y - a.y, b.z - a.z};
  const nm::Vec3 aq{q.x - a.x, q.y - a.y, q.z - a.z};
  const double denom = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
  double t = denom > 0.0 ? (aq.x * ab.x + aq.y * ab.y + aq.z * ab.z) / denom : 0.0;
  t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
  const Point3 proj{a.x + t * ab.x, a.y + t * ab.y, a.z + t * ab.z};
  return nm::distance(q, proj);
}

// Distance from point q to the nearest segment across a set of native polylines.
double distToNativePolylines(const Point3& q, const std::vector<std::vector<Point3>>& pls) {
  double best = 1e30;
  for (const auto& pl : pls)
    for (std::size_t i = 1; i < pl.size(); ++i)
      best = std::min(best, distPointSeg(q, pl[i - 1], pl[i]));
  return best;
}

// ── classification bucket ────────────────────────────────────────────────────────
enum Bucket { B_AGREED, B_DECLINED, B_DISAGREED, B_ORACLE_INACCURATE, B_COUNT };
const char* bucketName(int b) {
  switch (b) {
    case B_AGREED:            return "AGREED";
    case B_DECLINED:          return "HONESTLY-DECLINED";
    case B_DISAGREED:         return "DISAGREED";
    case B_ORACLE_INACCURATE: return "ORACLE-INACCURATE";
  }
  return "?";
}

// decline-reason histogram buckets (the EMPIRICAL S4 work-list).
enum Decline {
  D_NEAR_TANGENT,  // a WLine stopped NearTangent OR nearTangentGaps > 0
  D_MULTI_BRANCH,  // native traced fewer full branches than OCCT's welded component count
  D_SMALL_LOOP,    // native traced NOTHING but OCCT found a (small) closed loop → seeding-recall miss
  D_NO_SEED,       // native produced no seeds at all on a case OCCT DOES intersect
  D_FAILED,        // a WLine came back Failed (corrector could not advance from the seed)
  D_OTHER,         // declined for a reason not classed above
  D_COUNT
};
const char* declineName(int d) {
  switch (d) {
    case D_NEAR_TANGENT: return "near-tangent";
    case D_MULTI_BRANCH: return "multi-branch";
    case D_SMALL_LOOP:   return "small-loop";
    case D_NO_SEED:      return "no-seed";
    case D_FAILED:       return "corrector-failed";
    case D_OTHER:        return "other";
  }
  return "?";
}

// ── OCCT locus: the raw lines + a transversal branch-count via endpoint welding ────
struct Oracle {
  std::vector<Handle(Geom_Curve)> lines;
  int weldedComponents = 0;      // connected components (arc-split loci re-joined)
  bool hasIntersection = false;  // any OCCT line at all
};

// Weld OCCT arc-split lines that share an endpoint into connected components, so the
// native full-branch count can be compared against the number of DISTINCT loci.
int weldedComponentCount(const std::vector<Handle(Geom_Curve)>& lines, double weld) {
  const int n = static_cast<int>(lines.size());
  if (n == 0) return 0;
  std::vector<std::pair<gp_Pnt, gp_Pnt>> ep(n);
  for (int i = 0; i < n; ++i) {
    double f = lines[i]->FirstParameter(), l = lines[i]->LastParameter();
    if (!std::isfinite(f)) f = -1e6;
    if (!std::isfinite(l)) l = 1e6;
    ep[i] = {lines[i]->Value(f), lines[i]->Value(l)};
  }
  std::vector<int> comp(n, -1);
  int nc = 0;
  auto touch = [&](int i, int j) {
    return ep[i].first.Distance(ep[j].first) < weld || ep[i].first.Distance(ep[j].second) < weld ||
           ep[i].second.Distance(ep[j].first) < weld || ep[i].second.Distance(ep[j].second) < weld;
  };
  for (int i = 0; i < n; ++i) {
    if (comp[i] != -1) continue;
    comp[i] = nc;
    bool grew = true;
    while (grew) {
      grew = false;
      for (int a = 0; a < n; ++a) {
        if (comp[a] != nc) continue;
        for (int j = 0; j < n; ++j)
          if (comp[j] == -1 && touch(a, j)) { comp[j] = nc; grew = true; }
      }
    }
    ++nc;
  }
  return nc;
}

Oracle runOracle(const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb, double weld) {
  Oracle o;
  GeomAPI_IntSS iss(sa, sb, kOcctTol);
  const int n = iss.IsDone() ? iss.NbLines() : 0;
  for (int i = 1; i <= n; ++i) o.lines.push_back(iss.Line(i));
  o.hasIntersection = !o.lines.empty();
  o.weldedComponents = weldedComponentCount(o.lines, weld);
  return o;
}

// ── the per-trial classifier ─────────────────────────────────────────────────────
struct TrialResult {
  int bucket = B_AGREED;
  int declineReason = D_OTHER;   // meaningful only when bucket == B_DECLINED
  double maxOnSurf = 0.0;        // worst native-NODE on-both-surfaces residual (silent-wrong gate)
  double maxNativeOnCurve = 0.0; // worst native-NODE → nearest OCCT line (silent-wrong gate)
  double maxFitOnCurve = 0.0;    // worst fitted-spline sample → nearest OCCT line (reported only)
  double maxOcctOnNative = 0.0;  // worst OCCT sample → nearest native curve (coverage)
  int nativeTraced = 0;          // Closed | BoundaryExit | BranchArc
  int nativeGaps = 0;            // NearTangent / Failed WLines + TraceSet.nearTangentGaps
  int occtComponents = 0;
  std::string note;              // ORACLE-INACCURATE / DISAGREED justification

  // ── LOCUS-COVERAGE per-OCCT-line breakdown (the honest decline anatomy) ──
  // OCCT (GeomAPI_IntSS) frequently ARC-SPLITS one intersection locus into several line
  // components. A count/component comparison would mis-score native's single correct loop as
  // a decline whenever OCCT split it into more pieces than native traced. This oracle instead
  // compares LOCI by BIDIRECTIONAL coverage (native ⊆ OCCT via maxNativeOnCurve; OCCT ⊆ native
  // via maxOcctOnNative). These fields make the anatomy of a coverage decline explicit and
  // separate a GENUINE native miss from an OCCT arc-split over-count:
  int occtLinesTotal = 0;             // NbLines (raw OCCT arc-components)
  int occtLinesCoveredByNative = 0;   // OCCT lines native's traced set covers (≤ onCurve)
  int occtLinesUncovByNative = 0;     // OCCT lines native does NOT cover
  int uncovLinesOverCount = 0;        // uncovered lines whose locus IS covered by a SIBLING
                                      //   OCCT line native already covered → arc-split
                                      //   over-count (NOT a native miss; would be AGREED once
                                      //   the sibling is credited)
  int uncovLinesGenuineMiss = 0;      // uncovered lines whose locus NO other OCCT line covers
                                      //   → a genuine distinct locus native missed (real gap)
  double worstGenuineMissLen = 0.0;   // 3D length of the worst genuinely-missed OCCT locus
};

// ── LOCUS-COVERAGE helpers: separate a genuine native miss from an OCCT arc-split ──
//
// GeomAPI_IntSS splits ONE intersection locus into MULTIPLE arc-components. So when native
// correctly traces that locus as a single curve, some OCCT arc-components can sit off native's
// SAMPLE parameterization even though native covers the SAME geometric locus via a sibling arc.
// The oracle compares LOCI, not counts: an OCCT line native does not cover is an over-count
// (not a miss) iff its locus is ALSO occupied by a DIFFERENT OCCT line that native DOES cover.
// These measurements reuse the fixed onCurve tolerance (never widened).

// 3D length of an OCCT line (coarse polyline).
double occtLineLen(const Handle(Geom_Curve)& c) {
  double f = c->FirstParameter(), l = c->LastParameter();
  if (!std::isfinite(f)) f = -1e6;
  if (!std::isfinite(l)) l = 1e6;
  double len = 0.0; gp_Pnt prev;
  for (int i = 0; i <= 32; ++i) {
    const double t = f + (l - f) * (double(i) / 32);
    gp_Pnt q; try { q = c->Value(t); } catch (...) { continue; }
    if (i > 0) len += prev.Distance(q);
    prev = q;
  }
  return len;
}

// Worst distance sampling OCCT line `idx` to the nearest point on the OTHER OCCT lines
// (arc-split-sibling coverage). Small ⇒ this line's locus is redundant with a sibling arc.
double occtLineToOtherOcctLines(const std::vector<Handle(Geom_Curve)>& lines, int idx) {
  const auto& c = lines[static_cast<std::size_t>(idx)];
  double f = c->FirstParameter(), l = c->LastParameter();
  if (!std::isfinite(f)) f = -1e6;
  if (!std::isfinite(l)) l = 1e6;
  double worst = 0.0;
  for (int i = 0; i <= kOcctCurveSamples; ++i) {
    const double t = f + (l - f) * (double(i) / kOcctCurveSamples);
    gp_Pnt q; try { q = c->Value(t); } catch (...) { continue; }
    const Point3 qp{q.X(), q.Y(), q.Z()};
    double best = 1e30;
    for (int j = 0; j < static_cast<int>(lines.size()); ++j) {
      if (j == idx) continue;
      best = std::min(best, distToOcctCurve(lines[static_cast<std::size_t>(j)], qp));
    }
    worst = std::max(worst, best);
  }
  return worst;
}

// Worst distance sampling OCCT line `idx` to the nearest native polyline segment.
double occtLineToNative(const std::vector<Handle(Geom_Curve)>& lines, int idx,
                        const std::vector<std::vector<Point3>>& nativePolys) {
  const auto& c = lines[static_cast<std::size_t>(idx)];
  double f = c->FirstParameter(), l = c->LastParameter();
  if (!std::isfinite(f)) f = -1e6;
  if (!std::isfinite(l)) l = 1e6;
  double worst = 0.0;
  for (int i = 0; i <= kOcctCurveSamples; ++i) {
    const double t = f + (l - f) * (double(i) / kOcctCurveSamples);
    gp_Pnt q; try { q = c->Value(t); } catch (...) { continue; }
    worst = std::max(worst, distToNativePolylines(Point3{q.X(), q.Y(), q.Z()}, nativePolys));
  }
  return worst;
}

// Classify one already-run trial (native TraceSet + OCCT oracle) into exactly one bucket.
TrialResult classify(const ssi::TraceSet& ts,
                     const ssi::SurfaceAdapter& A, const ssi::SurfaceAdapter& B,
                     const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                     const Oracle& oracle) {
  TrialResult r;
  r.occtComponents = oracle.weldedComponents;

  // ── native side: count full traces vs gaps; measure the two DISTINCT residuals.
  //
  // TWO SEPARATE THINGS, TWO SEPARATE TOLERANCES (conflating them was the harness bug):
  //   * NODE on-surface — the corrected marching NODES are native's ACTUAL claim of
  //     "on both surfaces" (the two-surface least_squares corrector drives them there to
  //     ~1e-11). Measured against BOTH OCCT surfaces independently → maxNodeOnSurf. This is
  //     the SILENT-WRONG witness: a node off both surfaces (> onSurf 1e-6) is fabricated
  //     geometry (DISAGREED).
  //   * CURVE on-locus — the FITTED B-spline is a Geom-quality APPROXIMATION through the
  //     nodes; it bows off the surface BETWEEN nodes by a fit-resolution amount (documented
  //     ~1e-4..1e-6, deflection-dependent — NOT a corrector error). So the fitted samples are
  //     NOT held to the node on-surface gate; they are held to the COVERAGE gate onCurve
  //     (1e-3): every native sample (nodes + fit) must lie on SOME OCCT line → maxNatOnCurve.
  //     A native curve genuinely OFF the OCCT locus (> onCurve) is a fabricated curve.
  int nearTangent = 0, failed = 0, traced = 0;
  double maxNodeOnSurf = 0.0, maxNatOnCurve = 0.0, maxFitOnCurve = 0.0;
  bool anyCompleteClaim = false;  // native produced at least one curve it considers complete
  for (const auto& w : ts.lines) {
    switch (w.status) {
      case ssi::TraceStatus::Closed:
      case ssi::TraceStatus::BoundaryExit:
      case ssi::TraceStatus::BranchArc:  ++traced; anyCompleteClaim = true; break;
      case ssi::TraceStatus::NearTangent: ++nearTangent; break;
      case ssi::TraceStatus::Failed:      ++failed; break;
    }
    // NODE on-surface (silent-wrong witness): the corrected nodes only, vs both OCCT surfaces.
    // And NODE on-locus (the on-OCCT-curve silent-wrong witness): the corrected nodes' distance
    // to the nearest OCCT line. Both are native's ACTUAL claim — a node off a surface OR off the
    // OCCT locus is fabricated geometry. The fitted-spline samples are held separately (below).
    for (const auto& node : w.points) {
      const double ds = std::max(distToOcctSurface(sa, node.point), distToOcctSurface(sb, node.point));
      maxNodeOnSurf = std::max(maxNodeOnSurf, ds);
      if (!oracle.lines.empty()) {
        double best = 1e30;
        for (const auto& c : oracle.lines) best = std::min(best, distToOcctCurve(c, node.point));
        maxNatOnCurve = std::max(maxNatOnCurve, best);
      }
    }
    // FIT on-locus (reported only, NOT a DISAGREED gate): the fitted B-spline is a Geom-quality
    // APPROXIMATION through the nodes; it bows off the true curve BETWEEN nodes by a fit-
    // resolution amount (worst on a tight high-curvature glancing loop). That bow is not a
    // corrector error, so it does not decide silent-wrong — it is tracked for reporting.
    if (!oracle.lines.empty() && w.curve.valid()) {
      const auto& c = w.curve;
      const double t0 = c.knots.front(), t1 = c.knots.back();
      for (int i = 0; i <= kNativeCoverSamples; ++i) {
        const double t = t0 + (t1 - t0) * (double(i) / kNativeCoverSamples);
        const Point3 p = nm::curvePoint(c.degree, c.poles, c.knots, t);
        double best = 1e30;
        for (const auto& oc : oracle.lines) best = std::min(best, distToOcctCurve(oc, p));
        maxFitOnCurve = std::max(maxFitOnCurve, best);
      }
    }
  }
  r.nativeTraced = traced;
  r.nativeGaps = nearTangent + failed + ts.nearTangentGaps;
  r.maxOnSurf = maxNodeOnSurf;
  r.maxNativeOnCurve = maxNatOnCurve;
  r.maxFitOnCurve = maxFitOnCurve;

  // ── OCCT → native reverse coverage, PER OCCT LINE (locus-coverage, NOT a count compare).
  //    Sample each OCCT line, take the nearest native CURVE (point-to-segment against the
  //    ordered native polylines, so a sampling-density mismatch does not fake a coverage gap).
  //    For each OCCT line separate three outcomes:
  //      (a) covered by native (≤ onCurve) — native traced this locus;
  //      (b) NOT covered by native, but its locus IS covered by a SIBLING OCCT line native
  //          DID cover (≤ onCurve to the other OCCT lines) — an OCCT ARC-SPLIT OVER-COUNT:
  //          native traced the SAME geometric locus once, OCCT just chopped it into more
  //          arcs. This is credited to native (it is NOT a miss);
  //      (c) NOT covered by native AND its locus is on no other OCCT line either — a GENUINE
  //          distinct locus native missed (a real seeding-recall gap).
  //    `maxOcctOnNat` is the worst reverse residual over the GENUINE-miss lines only (an
  //    over-count sibling arc is not held against native — recognizing geometric equivalence
  //    is a legitimate oracle correction, never a widened tolerance).
  double maxOcctOnNat = 0.0;
  const bool nativeHasCurve = traced > 0;
  if (nativeHasCurve && oracle.hasIntersection) {
    std::vector<std::vector<Point3>> nativePolys;
    for (const auto& w : ts.lines)
      if (w.status == ssi::TraceStatus::Closed || w.status == ssi::TraceStatus::BoundaryExit ||
          w.status == ssi::TraceStatus::BranchArc)
        nativePolys.push_back(orderedPolyline(w));
    r.occtLinesTotal = static_cast<int>(oracle.lines.size());
    for (int i = 0; i < r.occtLinesTotal; ++i) {
      const double onNat = occtLineToNative(oracle.lines, i, nativePolys);
      if (onNat <= kOnCurveTol) { ++r.occtLinesCoveredByNative; continue; }  // (a)
      ++r.occtLinesUncovByNative;
      const double onSibling = (r.occtLinesTotal >= 2)
                                 ? occtLineToOtherOcctLines(oracle.lines, i) : 1e30;
      if (onSibling <= kOnCurveTol) {
        ++r.uncovLinesOverCount;   // (b) arc-split sibling of a covered locus → credit native
      } else {
        ++r.uncovLinesGenuineMiss; // (c) genuine distinct locus native missed
        maxOcctOnNat = std::max(maxOcctOnNat, onNat);
        r.worstGenuineMissLen = std::max(
            r.worstGenuineMissLen, occtLineLen(oracle.lines[static_cast<std::size_t>(i)]));
      }
    }
  }
  r.maxOcctOnNative = maxOcctOnNat;

  // ── DECISION TREE (exactly one bucket) ───────────────────────────────────────────
  //
  // The DISAGREED bar is NARROW and it is about CORRECTNESS OF WHAT NATIVE EMITTED, not
  // about COVERAGE COMPLETENESS. Native's SSI does not claim GLOBAL completeness — recall
  // is a MEASURED figure (TraceSet.completenessResidual is ALWAYS true; the roadmap frames
  // a missed loop as an acknowledged seeding-recall gap, not a wrong answer). So:
  //   * a native NODE off both surfaces, or a native CURVE off the OCCT locus, is DISAGREED
  //     (silent-wrong: native emitted geometry that is not the intersection);
  //   * a MISSING OCCT component while every native curve IS on the locus + on both surfaces
  //     is a DECLINE (multi-branch / small-loop seeding-recall gap) — NOT a disagreement.
  const bool nativeAny = !ts.lines.empty();
  const bool nativeCorrectGeom = (maxNodeOnSurf <= kOnSurfTol) &&
                                 (traced == 0 || maxNatOnCurve <= kOnCurveTol);

  // (1) SILENT-WRONG NODE: a corrected node native emitted is NOT on both surfaces. Native
  //     fabricated off-surface geometry → DISAGREED (the bar forbids it).
  if (nativeAny && maxNodeOnSurf > kOnSurfTol) {
    r.bucket = B_DISAGREED;
    r.note = "native node off both surfaces (nodeOnSurf=" + std::to_string(maxNodeOnSurf) + ")";
    return r;
  }

  // (2) SILENT-WRONG CURVE: a native curve native traced (Closed/BoundaryExit/BranchArc) is
  //     itself OFF the OCCT locus (> onCurve) even though OCCT found that locus. Before
  //     calling DISAGREED, rule out the rare ORACLE-INACCURATE case (native's nodes are on
  //     both surfaces to << tol while OCCT's own nearest line sits OFF a surface by more than
  //     its tol → native is the more correct one).
  if (traced > 0 && oracle.hasIntersection && maxNatOnCurve > kOnCurveTol) {
    double occtOffSurf = 0.0;
    for (const auto& c : oracle.lines) {
      double f = c->FirstParameter(), l = c->LastParameter();
      if (!std::isfinite(f)) f = -1e6;
      if (!std::isfinite(l)) l = 1e6;
      for (int i = 0; i <= kOcctCurveSamples; ++i) {
        const double t = f + (l - f) * (double(i) / kOcctCurveSamples);
        gp_Pnt q;
        try { q = c->Value(t); } catch (...) { continue; }
        const Point3 qp{q.X(), q.Y(), q.Z()};
        occtOffSurf = std::max(occtOffSurf, std::max(distToOcctSurface(sa, qp),
                                                     distToOcctSurface(sb, qp)));
      }
    }
    if (occtOffSurf > 100.0 * kOcctTol && maxNodeOnSurf <= kOnSurfTol) {
      r.bucket = B_ORACLE_INACCURATE;
      r.note = "native curve off OCCT line but OCCT line off surface " +
               std::to_string(occtOffSurf) + " (native nodeOnSurf " +
               std::to_string(maxNodeOnSurf) + ")";
      return r;
    }
    r.bucket = B_DISAGREED;
    r.note = "native traced a curve OFF the OCCT locus (natOnOcct=" +
             std::to_string(maxNatOnCurve) + ", nodeOnSurf=" + std::to_string(maxNodeOnSurf) + ")";
    return r;
  }

  // From here native's emitted geometry (if any) is CORRECT: nodes on both surfaces, curves
  // on the OCCT loci. Remaining questions are COVERAGE (did native find every branch?), which
  // are DECLINE reasons — never DISAGREED.

  // (3) native declined / incomplete (an explicit gap) → HONESTLY-DECLINED, with a reason.
  if (r.nativeGaps > 0) {
    r.bucket = B_DECLINED;
    if (failed > 0 && nearTangent == 0 && ts.nearTangentGaps == 0) r.declineReason = D_FAILED;
    else r.declineReason = D_NEAR_TANGENT;
    return r;
  }

  // (4) OCCT found an intersection but native traced NOTHING complete → decline (seeding
  //     recall / no-seed). no-seed (S2 produced nothing) vs small-loop (seeded, marched
  //     nothing) by the seeded-branch count.
  if (oracle.hasIntersection && traced == 0) {
    r.bucket = B_DECLINED;
    r.declineReason = (ts.seededBranches == 0) ? D_NO_SEED : D_SMALL_LOOP;
    return r;
  }

  // (5) native traced correct curve(s) but left a GENUINE OCCT locus uncovered: some OCCT
  //     point on a line whose locus NO sibling arc covers is far from every native curve
  //     (maxOcctOnNat > onCurve — and maxOcctOnNat is now the GENUINE-miss residual, with
  //     OCCT arc-split over-count siblings already credited to native above). This is the
  //     authoritative LOCUS-COVERAGE signal, NOT a raw component count: an OCCT loop native
  //     covered but OCCT chopped into extra arcs does NOT reach here (its sibling arcs were
  //     credited, maxOcctOnNat stayed 0 → AGREED at (6)). So a decline here is a real
  //     seeding-recall COVERAGE gap (a distinct disjoint loop, or a small loop the grid
  //     merged) — HONESTLY-DECLINED, never a wrong answer. Label multi-branch vs small-loop
  //     by whether MORE THAN ONE distinct locus is genuinely missed (multi-branch) or a
  //     single small loop is (small-loop) — a geometric split, not the arc-component count.
  if (traced > 0 && oracle.hasIntersection && maxOcctOnNat > kOnCurveTol) {
    r.bucket = B_DECLINED;
    // multi-branch: native already traced ≥ 1 locus AND genuinely missed a SEPARATE distinct
    // locus (a co-resident second branch) → the dominant recall frontier. small-loop: only a
    // single small locus is genuinely missed while nothing (or one loop) was otherwise traced.
    const bool multiDistinct =
        r.uncovLinesGenuineMiss >= 1 && r.occtLinesCoveredByNative >= 1;
    r.declineReason = multiDistinct ? D_MULTI_BRANCH : D_SMALL_LOOP;
    return r;
  }

  // (6) native traced correct curve(s) and covered every OCCT LOCUS (arc-split over-counts
  //     credited) → AGREED. This is where an OCCT-arc-split-over-count case lands: native's
  //     single loop covers all of OCCT's split arcs (and vice-versa within tol) → same locus,
  //     AGREED regardless of how many arc-components OCCT emitted.
  if (traced > 0 && oracle.hasIntersection && nativeCorrectGeom) {
    r.bucket = B_AGREED;
    return r;
  }

  // (7) neither side found anything → AGREED (both empty). A DISJOINT control lands here.
  if (!oracle.hasIntersection && !nativeAny) {
    r.bucket = B_AGREED;
    return r;
  }

  // (8) native traced something but OCCT found nothing. Native's curve is on both surfaces
  //     (rules 1-2 already ruled out off-surface/off-locus) → a real curve OCCT (IntSS)
  //     missed → ORACLE-INACCURATE; if native produced only gaps and OCCT is empty, decline.
  if (nativeAny && !oracle.hasIntersection) {
    if (traced > 0 && maxNodeOnSurf <= kOnSurfTol) {
      r.bucket = B_ORACLE_INACCURATE;
      r.note = "native traced an on-both-surfaces curve OCCT (IntSS) returned empty for";
      return r;
    }
    r.bucket = B_DECLINED;  // native only produced gaps and OCCT empty — nothing traced
    r.declineReason = D_OTHER;
    return r;
  }

  // Fallback: treat as declined-other (should be unreachable).
  r.bucket = B_DECLINED;
  r.declineReason = D_OTHER;
  return r;
}

// ── DISAGREED reproduction dump: the two surfaces' defining data ──────────────────
void dumpSurf(const char* tag, const NurbsSurf& s) {
  std::printf("    %s: degU=%d degV=%d nU=%d nV=%d rational=%d\n", tag, s.degU, s.degV,
              s.nU, s.nV, s.rational() ? 1 : 0);
  std::printf("      poles=[");
  for (std::size_t i = 0; i < s.poles.size(); ++i)
    std::printf("%s(%.6g,%.6g,%.6g)", i ? "," : "", s.poles[i].x, s.poles[i].y, s.poles[i].z);
  std::printf("]\n");
  if (s.rational()) {
    std::printf("      weights=[");
    for (std::size_t i = 0; i < s.weights.size(); ++i)
      std::printf("%s%.6g", i ? "," : "", s.weights[i]);
    std::printf("]\n");
  }
  std::printf("      knotsU=[");
  for (std::size_t i = 0; i < s.knotsU.size(); ++i) std::printf("%s%.6g", i ? "," : "", s.knotsU[i]);
  std::printf("] knotsV=[");
  for (std::size_t i = 0; i < s.knotsV.size(); ++i) std::printf("%s%.6g", i ? "," : "", s.knotsV[i]);
  std::printf("]\n");
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x5515D1FF0F0Full;  // fixed default (no clock / rand / address)
  int N = 48;                          // per-seed trial count (≥ 40 required)
  int nSeeds = 2;                      // ≥ 2 seeds required
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (argc > 3) nSeeds = std::atoi(argv[3]);
  else if (const char* e = std::getenv("FUZZ_SEEDS")) nSeeds = std::atoi(e);
  if (N < 40) N = 40;
  if (nSeeds < 2) nSeeds = 2;

  std::printf("== GENERAL FREEFORM SSI differential fuzzer: native NURBS↔NURBS vs OCCT (NURBS L2) ==\n");
  std::printf("== base seed=0x%llx N=%d/seed seeds=%d  FIXED tol: onSurf=%.0e onCurve=%.0e occt=%.0e (NEVER widened) ==\n",
              static_cast<unsigned long long>(seed), N, nSeeds, kOnSurfTol, kOnCurveTol, kOcctTol);
  std::printf("== BAR: DISAGREED==0 (native must never trace a curve not on both surfaces, nor "
              "declare completeness with a silently-missed locus). Decline is FINE. ==\n");
  std::fflush(stdout);

  long bucketTotals[B_COUNT] = {0};
  long declineTotals[D_COUNT] = {0};
  long declineByFamily[F_COUNT][D_COUNT] = {{0}};
  long famTotals[F_COUNT] = {0};
  int totalDisagreed = 0;
  // LOCUS-COVERAGE audit accumulators (the honest anatomy of the residual):
  long agreedOverCountTrials = 0;   // AGREED trials where OCCT arc-split into MORE lines than
                                    //   native traced yet locus-coverage AGREED (the over-count
                                    //   case a count comparison would have mis-declined)
  long declineOverCountLines = 0;   // over-count sibling arcs credited to native across declines
  long declineGenuineMissLines = 0; // genuinely-missed distinct OCCT loci across declines

  for (int si = 0; si < nSeeds; ++si) {
    const uint64_t thisSeed = seed + static_cast<uint64_t>(si) * 0x100000001B3ull;
    Rng rng(thisSeed);
    long seedBucket[B_COUNT] = {0};

    for (int idx = 0; idx < N; ++idx) {
      // Bias toward ACTUALLY intersecting, mostly transversal: transversal/tilted/multi
      // dominate; near-tangent + disjoint are a minority spread.
      int family;
      const double fr = rng.unit();
      if (fr < 0.34) family = F_TRANSVERSAL;
      else if (fr < 0.60) family = F_TILTED;
      else if (fr < 0.80) family = F_MULTIBRANCH;
      else if (fr < 0.92) family = F_NEAR_TANGENT;
      else family = F_DISJOINT;
      ++famTotals[family];

      NurbsSurf A, B;
      if (!buildPair(family, rng, A, B)) {
        // A generator failure is a HARNESS bug, not a native disagreement — report + skip.
        std::printf("[GENFAIL] seed=0x%llx idx=%d family=%s (invalid surface generated)\n",
                    static_cast<unsigned long long>(thisSeed), idx, famName(family));
        std::fflush(stdout);
        continue;
      }

      ssi::SurfaceAdapter na = nativeAdapter(A), nb = nativeAdapter(B);
      Handle(Geom_Surface) sa = toOcct(A), sb = toOcct(B);

      // Native pipeline: S2 seed + S3 trace, the real marcher. A moderate initial grid +
      // finer leaf catches multi-branch loops without special-casing per family.
      ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6; sopt.minPatchFrac = 1.0 / 32.0;
      ssi::MarchOptions mopt;  // scale-derived defaults
      const ssi::TraceSet ts = ssi::trace_intersection(na, nb, sopt, mopt);

      const double weld = std::max(kOnCurveTol * 10.0, na.modelScale * 1e-3);
      const Oracle oracle = runOracle(sa, sb, weld);

      TrialResult r = classify(ts, na, nb, sa, sb, oracle);
      ++bucketTotals[r.bucket];
      ++seedBucket[r.bucket];
      if (r.bucket == B_DECLINED) {
        ++declineTotals[r.declineReason];
        ++declineByFamily[family][r.declineReason];
        declineOverCountLines += r.uncovLinesOverCount;
        declineGenuineMissLines += r.uncovLinesGenuineMiss;
        if (r.declineReason == D_MULTI_BRANCH || r.declineReason == D_SMALL_LOOP)
          std::printf("[DECLINE-DIAG] seed=0x%llx idx=%d fam=%s reason=%s :: traced=%d "
                      "occtLines=%d covByNat=%d uncov=%d overCount=%d genuineMiss=%d "
                      "natOnOcct=%.3e genuineOcctOnNat=%.3e worstMissLen=%.3e\n",
                      static_cast<unsigned long long>(thisSeed), idx, famName(family),
                      declineName(r.declineReason), r.nativeTraced, r.occtLinesTotal,
                      r.occtLinesCoveredByNative, r.occtLinesUncovByNative,
                      r.uncovLinesOverCount, r.uncovLinesGenuineMiss, r.maxNativeOnCurve,
                      r.maxOcctOnNative, r.worstGenuineMissLen);
        std::fflush(stdout);

        // ── SSI-GRAZE THREE-WAY: for the declined trial, find the native NODE whose distance
        //    to the OCCT locus is WORST (native's root at the graze pinch, i.e. where native
        //    and OCCT diverge most), its params on A, and the nearest OCCT world point (OCCT's
        //    root). Print both at full precision so the independent host oracle can measure
        //    native→truth, OCCT→truth and native→OCCT. Also report the worst FITTED-SPLINE
        //    sample's OCCT gap (the discretization-bow candidate). OCCT-free host consumer.
        if (!oracle.lines.empty()) {
          double worstNode = -1.0; Point3 natRoot{}, occtRoot{}; double naU=0, naV=0, nbU=0, nbV=0;
          for (const auto& w : ts.lines) {
            for (const auto& node : w.points) {
              double d; const Point3 op = nearestOcctPoint(oracle.lines, node.point, d);
              if (d > worstNode) { worstNode = d; natRoot = node.point; occtRoot = op;
                                   naU=node.u1; naV=node.v1; nbU=node.u2; nbV=node.v2; }
            }
          }
          double worstFit = -1.0; Point3 fitRoot{}, fitOcct{};
          for (const auto& w : ts.lines) {
            if (!w.curve.valid()) continue;
            const auto& c = w.curve; const double t0=c.knots.front(), t1=c.knots.back();
            for (int i = 0; i <= 128; ++i) {
              const double t = t0 + (t1-t0)*(double(i)/128.0);
              const Point3 p = nm::curvePoint(c.degree, c.poles, c.knots, t);
              double d; const Point3 op = nearestOcctPoint(oracle.lines, p, d);
              if (d > worstFit) { worstFit = d; fitRoot = p; fitOcct = op; }
            }
          }
          std::printf("[GRAZE-3WAY] seed=0x%llx idx=%d :: worstNodeGap=%.6e natRootP=(%.12f,%.12f,%.12f) "
                      "natRootUV=A(%.12f,%.12f)B(%.12f,%.12f) occtRootP=(%.12f,%.12f,%.12f) || "
                      "worstFitGap=%.6e fitRootP=(%.12f,%.12f,%.12f) fitOcctP=(%.12f,%.12f,%.12f)\n",
                      static_cast<unsigned long long>(thisSeed), idx,
                      worstNode, natRoot.x, natRoot.y, natRoot.z, naU, naV, nbU, nbV,
                      occtRoot.x, occtRoot.y, occtRoot.z,
                      worstFit, fitRoot.x, fitRoot.y, fitRoot.z, fitOcct.x, fitOcct.y, fitOcct.z);
          std::fflush(stdout);
        }
      }
      // AGREED yet OCCT emitted MORE arc-components than native traced → the OCCT arc-split
      // over-count the locus-coverage oracle correctly recognizes as the SAME locus (AGREED),
      // where a naive component/branch-COUNT comparison would have wrongly scored a decline.
      if (r.bucket == B_AGREED && r.occtLinesTotal > r.nativeTraced && r.nativeTraced > 0) {
        ++agreedOverCountTrials;
        std::printf("[AGREED-OVERCOUNT] seed=0x%llx idx=%d fam=%s :: traced=%d occtLines=%d "
                    "occtComp=%d occtOnNat=%.3e (OCCT arc-split of one native locus → AGREED)\n",
                    static_cast<unsigned long long>(thisSeed), idx, famName(family),
                    r.nativeTraced, r.occtLinesTotal, r.occtComponents, r.maxOcctOnNative);
        std::fflush(stdout);
      }

      if (r.bucket == B_DISAGREED) {
        ++totalDisagreed;
        std::printf("[DISAGREED] seed=0x%llx idx=%d family=%s :: %s\n",
                    static_cast<unsigned long long>(thisSeed), idx, famName(family), r.note.c_str());
        std::printf("    native: traced=%d gaps=%d seeded=%d nodeOnSurf=%.3e nodeOnOcct=%.3e "
                    "fitOnOcct=%.3e occtOnNat=%.3e occtComp=%d\n", r.nativeTraced, r.nativeGaps,
                    ts.seededBranches, r.maxOnSurf, r.maxNativeOnCurve, r.maxFitOnCurve,
                    r.maxOcctOnNative, r.occtComponents);
        dumpSurf("A", A);
        dumpSurf("B", B);
        std::fflush(stdout);
      } else if (r.bucket == B_ORACLE_INACCURATE) {
        std::printf("[ORACLE-INACCURATE] seed=0x%llx idx=%d family=%s :: %s (onSurf=%.3e)\n",
                    static_cast<unsigned long long>(thisSeed), idx, famName(family), r.note.c_str(),
                    r.maxOnSurf);
        std::fflush(stdout);
      }
    }

    std::printf("-- seed 0x%llx: AGREED=%ld DECLINED=%ld DISAGREED=%ld ORACLE-INACCURATE=%ld (N=%d)\n",
                static_cast<unsigned long long>(thisSeed), seedBucket[B_AGREED], seedBucket[B_DECLINED],
                seedBucket[B_DISAGREED], seedBucket[B_ORACLE_INACCURATE], N);
    std::fflush(stdout);
  }

  // ── COVERAGE SUMMARY ─────────────────────────────────────────────────────────────
  const long total = bucketTotals[B_AGREED] + bucketTotals[B_DECLINED] +
                      bucketTotals[B_DISAGREED] + bucketTotals[B_ORACLE_INACCURATE];
  std::printf("\n== COVERAGE SUMMARY (total trials=%ld) ==\n", total);
  for (int b = 0; b < B_COUNT; ++b)
    std::printf("   %-18s %ld  (%.1f%%)\n", bucketName(b), bucketTotals[b],
                total ? 100.0 * double(bucketTotals[b]) / double(total) : 0.0);

  const long declTotal = bucketTotals[B_DECLINED];
  std::printf("\n== HONESTLY-DECLINED reason histogram (the EMPIRICAL S4 work-list; declined=%ld) ==\n",
              declTotal);
  for (int d = 0; d < D_COUNT; ++d)
    std::printf("   %-18s %ld  (%.1f%% of declines)\n", declineName(d), declineTotals[d],
                declTotal ? 100.0 * double(declineTotals[d]) / double(declTotal) : 0.0);

  std::printf("\n== decline reasons by family (rows: family; cols: %s / %s / %s / %s / %s / %s) ==\n",
              declineName(0), declineName(1), declineName(2), declineName(3), declineName(4), declineName(5));
  for (int f = 0; f < F_COUNT; ++f) {
    std::printf("   %-14s (%ld trials): ", famName(f), famTotals[f]);
    for (int d = 0; d < D_COUNT; ++d) std::printf("%ld ", declineByFamily[f][d]);
    std::printf("\n");
  }

  // ── LOCUS-COVERAGE AUDIT: prove the oracle compares LOCI, not arc-component COUNTS ──
  // agreedOverCountTrials  — AGREED trials where OCCT split one native locus into MORE arcs
  //   than native traced, correctly AGREED (a count comparison would have mis-declined these).
  // declineOverCountLines  — arc-split sibling arcs credited to native inside declines (not miss).
  // declineGenuineMissLines— genuinely distinct OCCT loci native missed inside declines.
  // The residual decline is GENUINE iff declineGenuineMissLines > 0 and declineOverCountLines
  // does NOT drive any decline (a decline with 0 genuine misses would be an over-count artifact).
  std::printf("\n== LOCUS-COVERAGE AUDIT (oracle compares loci, not arc-component counts) ==\n");
  std::printf("   AGREED over-count trials (OCCT arc-split one native locus, AGREED not declined): %ld\n",
              agreedOverCountTrials);
  std::printf("   decline lines credited as OCCT arc-split over-counts (not native misses): %ld\n",
              declineOverCountLines);
  std::printf("   decline lines that are GENUINE distinct loci native missed: %ld\n",
              declineGenuineMissLines);

  std::printf("\n== BAR: DISAGREED==%d  → %s ==\n", totalDisagreed,
              totalDisagreed == 0 ? "PASS (decline is allowed; the bar is DISAGREED==0)" : "FAIL");
  std::fflush(stdout);
  std::_Exit(totalDisagreed == 0 ? 0 : 1);
}
