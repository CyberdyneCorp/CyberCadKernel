// replay_freeform_seed.cpp — OCCT-FREE host replay of the freeform SSI fuzz poses.
//
// Reproduces the EXACT splitmix64→xoshiro256** RNG + surface generator of
// tests/sim/native_ssi_freeform_fuzz.mm (verbatim), advances the stream through
// idx to a target trial, builds the SAME native adapters + SAME SeedOptions the
// sim harness uses (initialGridU=6, initialGridV=6, minPatchFrac=1/32), then runs
// seed_intersection + trace_intersection so the SEED-DIAG instrument reveals WHY a
// locus is/isn't seeded — WITHOUT OCCT (this diagnoses only the native side).
//
// Usage: replay_freeform_seed <baseSeedHex> <N> <si> <idx>
//   e.g. replay_freeform_seed 0x5515D1FF0F0F 48 0 24
// The sim harness per-seed derivation: thisSeed = base + si*0x100000001B3.

#include "native/ssi/native_ssi.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace ssi = cybercad::native::ssi;
using cybercad::native::math::Point3;

// ── RNG (verbatim from native_ssi_freeform_fuzz.mm) ────────────────────────────
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
  return true;  // validSurf omitted (harness-side) — poses here are valid by construction
}

ssi::SurfaceAdapter nativeAdapter(const NurbsSurf& s) {
  if (s.rational())
    return ssi::makeNurbsAdapter(s.degU, s.degV, s.poles, s.weights, s.nU, s.nV, s.knotsU, s.knotsV);
  return ssi::makeBSplineAdapter(s.degU, s.degV, s.poles, s.nU, s.nV, s.knotsU, s.knotsV);
}

int pickFamily(Rng& rng) {
  const double fr = rng.unit();
  if (fr < 0.34) return F_TRANSVERSAL;
  if (fr < 0.60) return F_TILTED;
  if (fr < 0.80) return F_MULTIBRANCH;
  if (fr < 0.92) return F_NEAR_TANGENT;
  return F_DISJOINT;
}

int main(int argc, char** argv) {
  if (argc < 5) { std::fprintf(stderr, "usage: %s <baseSeedHex> <N> <si> <idx>\n", argv[0]); return 2; }
  const uint64_t base = std::strtoull(argv[1], nullptr, 0);
  const int N = std::atoi(argv[2]);
  const int siTarget = std::atoi(argv[3]);
  const int idxTarget = std::atoi(argv[4]);
  (void)N;

  const uint64_t thisSeed = base + static_cast<uint64_t>(siTarget) * 0x100000001B3ull;
  Rng rng(thisSeed);

  int family = -1;
  NurbsSurf A, B;
  for (int idx = 0; idx <= idxTarget; ++idx) {
    family = pickFamily(rng);
    A = NurbsSurf{}; B = NurbsSurf{};
    buildPair(family, rng, A, B);
    if (idx < idxTarget) continue;  // advance RNG only; only the target idx is measured
  }

  std::printf("== REPLAY seed=0x%llx si=%d idx=%d family=%s ==\n",
              (unsigned long long)thisSeed, siTarget, idxTarget, famName(family));
  std::printf("   A: deg %dx%d poles %dx%d rational=%d\n", A.degU, A.degV, A.nU, A.nV, (int)A.rational());
  std::printf("   B: deg %dx%d poles %dx%d rational=%d\n", B.degU, B.degV, B.nU, B.nV, (int)B.rational());

  ssi::SurfaceAdapter na = nativeAdapter(A), nb = nativeAdapter(B);
  std::printf("   modelScale A=%.4e B=%.4e\n", na.modelScale, nb.modelScale);

  // POSE DUMP: emit the verbatim pose as copy-paste-ready C++ for a regression fixture.
  if (std::getenv("REPLAY_DUMP_POSE")) {
    auto dumpPoles = [](const char* nm, const std::vector<Point3>& p) {
      std::printf("  const std::vector<Point3> %s = {\n    ", nm);
      for (std::size_t i = 0; i < p.size(); ++i)
        std::printf("{%.9g,%.9g,%.9g},", p[i].x, p[i].y, p[i].z);
      std::printf("\n  };\n");
    };
    auto dumpW = [](const char* nm, const std::vector<double>& w) {
      std::printf("  const std::vector<double> %s = {", nm);
      for (double x : w) std::printf("%.9g,", x);
      std::printf("};\n");
    };
    std::printf("// ===== POSE DUMP (paste into test_native_ssi_seeding.cpp) =====\n");
    dumpPoles("polesA", A.poles); dumpW("wtsA", A.weights); dumpW("kUA", A.knotsU); dumpW("kVA", A.knotsV);
    std::printf("  const int degUA=%d, degVA=%d, nUA=%d, nVA=%d;\n", A.degU, A.degV, A.nU, A.nV);
    dumpPoles("polesB", B.poles); dumpW("wtsB", B.weights); dumpW("kUB", B.knotsU); dumpW("kVB", B.knotsV);
    std::printf("  const int degUB=%d, degVB=%d, nUB=%d, nVB=%d;\n", B.degU, B.degV, B.nU, B.nV);
    std::printf("// ===== END POSE DUMP =====\n"); std::fflush(stdout);
  }

  ssi::SeedOptions sopt; sopt.initialGridU = 6; sopt.initialGridV = 6; sopt.minPatchFrac = 1.0 / 32.0;
  if (const char* e = std::getenv("REPLAY_GRID"))   { int g = std::atoi(e); if (g>0){ sopt.initialGridU = g; sopt.initialGridV = g; } }
  if (const char* e = std::getenv("REPLAY_LEAF"))   { double v = std::atof(e); if (v>0) sopt.minPatchFrac = v; }
  if (const char* e = std::getenv("REPLAY_BUDGET")) { int b = std::atoi(e); if (b>0) sopt.capRetentionBudget = b; }
  if (std::getenv("REPLAY_NOADAPT")) sopt.adaptiveSubdivision = false;
  std::printf("   cfg: grid=%d leaf=%.5g budget=%d\n", sopt.initialGridU, sopt.minPatchFrac, sopt.capRetentionBudget);

  std::printf("-- seed_intersection (SEED-DIAG below) --\n"); std::fflush(stdout);
  const ssi::SeedSet ss = ssi::seed_intersection(na, nb, sopt);
  std::printf("   SEEDS: n=%zu candidateRegions=%d refinedAccepted=%d deferredTangent=%d\n",
              ss.seeds.size(), ss.candidateRegions, ss.refinedAccepted, ss.deferredTangent);
  for (std::size_t i = 0; i < ss.seeds.size(); ++i) {
    const auto& s = ss.seeds[i];
    std::printf("     seed[%zu] P=(%.5f,%.5f,%.5f) uvA=(%.4f,%.4f) uvB=(%.4f,%.4f) branchId=%d\n",
                i, s.point.x, s.point.y, s.point.z, s.u1, s.v1, s.u2, s.v2, s.branchId);
  }

  const bool seedOnly = std::getenv("REPLAY_SEED_ONLY") != nullptr;
  if (seedOnly) { std::printf("-- (trace skipped: REPLAY_SEED_ONLY) --\n"); return 0; }

  ssi::MarchOptions mopt;
  // Refinement knobs (SSI-TERM/SSI-MARCH): confirm the near-tangent pinch deviation is
  // refinement-INVARIANT (not arc-error) by tightening the deflection / step on the SAME code.
  if (const char* e = std::getenv("MARCH_MAXDEFL")) { double v = std::atof(e); if (v>0) mopt.maxDeflection = na.modelScale * v; }
  if (const char* e = std::getenv("MARCH_MAXSTEP")) { double v = std::atof(e); if (v>0) mopt.maxStep = na.modelScale * v; }
  const ssi::TraceSet ts = ssi::trace_intersection(na, nb, sopt, mopt);
  std::printf("-- trace_intersection --\n");
  std::printf("   TRACE: tracedBranches=%d lines=%zu\n", ts.tracedBranches, ts.lines.size());
  const bool dumpTrace = std::getenv("REPLAY_DUMP_TRACE") != nullptr;
  for (std::size_t i = 0; i < ts.lines.size(); ++i) {
    const auto& w = ts.lines[i];
    int st = static_cast<int>(w.status);
    const char* sn = st==0?"Closed":st==1?"BoundaryExit":st==2?"NearTangent":st==3?"Failed":"BranchArc";
    // polyline arc length (native's covered EXTENT — compare to OCCT locus length).
    double poly = 0.0;
    for (std::size_t k = 1; k < w.points.size(); ++k)
      poly += cybercad::native::math::distance(w.points[k].point, w.points[k-1].point);
    std::printf("     line[%zu] status=%s nodes=%zu polyLen=%.6g onSurfResidual=%.3e nearTangentCrossed=%d branchId=%d\n",
                i, sn, w.points.size(), poly, w.onSurfResidual, w.nearTangentCrossed, w.branchId);
    if (dumpTrace && !w.points.empty()) {
      // FRONT / BACK endpoint params — reveal WHICH surface edge the boundary-exit hit and by how far.
      auto edgeGap = [](double x){ return std::min(std::min(x, 1.0 - x), 1e30); };  // domain [0,1] gap to nearest edge
      const auto& f = w.points.front();
      const auto& b = w.points.back();
      std::printf("       FRONT uvA=(%.6f,%.6f) uvB=(%.6f,%.6f) edgeGapA=%.3e edgeGapB=%.3e\n",
                  f.u1, f.v1, f.u2, f.v2,
                  std::min(edgeGap(f.u1), edgeGap(f.v1)), std::min(edgeGap(f.u2), edgeGap(f.v2)));
      std::printf("       BACK  uvA=(%.6f,%.6f) uvB=(%.6f,%.6f) edgeGapA=%.3e edgeGapB=%.3e\n",
                  b.u1, b.v1, b.u2, b.v2,
                  std::min(edgeGap(b.u1), edgeGap(b.v1)), std::min(edgeGap(b.u2), edgeGap(b.v2)));
      // FULL footprint bbox in (u,v) on both surfaces — does native's arc footprint span the domain?
      double au0=1e30,au1=-1e30,av0=1e30,av1=-1e30,bu0=1e30,bu1=-1e30,bv0=1e30,bv1=-1e30;
      for (const auto& n : w.points) {
        au0=std::min(au0,n.u1); au1=std::max(au1,n.u1); av0=std::min(av0,n.v1); av1=std::max(av1,n.v1);
        bu0=std::min(bu0,n.u2); bu1=std::max(bu1,n.u2); bv0=std::min(bv0,n.v2); bv1=std::max(bv1,n.v2);
      }
      std::printf("       FOOTPRINT A:u[%.4f,%.4f]v[%.4f,%.4f] B:u[%.4f,%.4f]v[%.4f,%.4f]\n",
                  au0,au1,av0,av1,bu0,bu1,bv0,bv1);
      // MIN-SINE + MAX-CHORD + MAX-TURN scan: a near-tangent GRAZE (min transversality sine) is
      // where native's converged locus can diverge laterally from OCCT's. Report the min-sine node
      // + the largest inter-node chord + the sharpest per-node tangent turn (candidate deviation site).
      double minSine = 1e30; std::size_t minSineIdx = 0;
      double maxChord = 0.0;  std::size_t maxChordIdx = 0;
      double maxTurnDeg = 0.0; std::size_t maxTurnIdx = 0;
      for (std::size_t k = 0; k < w.points.size(); ++k) {
        const auto& n = w.points[k];
        const auto nA = na.normal(n.u1, n.v1); const auto nB = nb.normal(n.u2, n.v2);
        const double s = cybercad::native::math::norm(cybercad::native::math::cross(nA.vec(), nB.vec()));
        if (s < minSine) { minSine = s; minSineIdx = k; }
        if (k >= 1) { const double d = cybercad::native::math::distance(n.point, w.points[k-1].point);
                      if (d > maxChord) { maxChord = d; maxChordIdx = k; } }
        if (k >= 1 && k + 1 < w.points.size()) {
          auto d0 = w.points[k].point - w.points[k-1].point;
          auto d1 = w.points[k+1].point - w.points[k].point;
          const double n0 = cybercad::native::math::norm(d0), n1 = cybercad::native::math::norm(d1);
          if (n0 > 1e-12 && n1 > 1e-12) {
            double cs = cybercad::native::math::dot(d0,d1)/(n0*n1); cs = cs>1?1:(cs<-1?-1:cs);
            const double deg = std::acos(cs) * 57.29577951308232;
            if (deg > maxTurnDeg) { maxTurnDeg = deg; maxTurnIdx = k; }
          }
        }
      }
      const double closeGap = cybercad::native::math::distance(w.points.front().point, w.points.back().point);
      std::printf("       SCAN minSine=%.4g @node%zu  maxChord=%.4g @node%zu  maxTurn=%.2fdeg @node%zu  closeGap(front-back)=%.4g\n",
                  minSine, minSineIdx, maxChord, maxChordIdx, maxTurnDeg, maxTurnIdx, closeGap);
      const auto& ms = w.points[minSineIdx];
      std::printf("       MINSINE node uvA=(%.5f,%.5f) uvB=(%.5f,%.5f) P=(%.5f,%.5f,%.5f)\n",
                  ms.u1, ms.v1, ms.u2, ms.v2, ms.point.x, ms.point.y, ms.point.z);
      // SSI-GRAZE pinch neighbourhood — FULL-PRECISION params of the min-sine node and its
      // immediate flanks, so an INDEPENDENT high-precision three-way oracle
      // (oracle_graze_threeway) can project each onto the true curve and measure the
      // chord/fit bow at the graze. Gated by REPLAY_DUMP_PINCH (additive; off → byte-identical).
      if (std::getenv("REPLAY_DUMP_PINCH")) {
        const std::size_t lo = minSineIdx > 0 ? minSineIdx - 1 : minSineIdx;
        const std::size_t hi = minSineIdx + 1 < w.points.size() ? minSineIdx + 1 : minSineIdx;
        for (std::size_t k = lo; k <= hi; ++k) {
          const auto& n = w.points[k];
          std::printf("       PINCH node%zu uvA=(%.12f,%.12f) uvB=(%.12f,%.12f) P=(%.12f,%.12f,%.12f) onSurfRes=%.3e\n",
                      k, n.u1, n.v1, n.u2, n.v2, n.point.x, n.point.y, n.point.z, n.onSurfResidual);
        }
      }
    }
  }
  return 0;
}
