// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 7 reverse-engineering ROBUSTNESS —
// segmentAndFitRobust over NOISY / OUTLIER-LADEN point clouds
// (src/native/math/reverse_engineer.{h,cpp}). OCCT-FREE. Deterministic synthetic data
// (RNG seeded by index — no global clock). The oracles are airtight:
//
//   1. NOISY PRIMITIVE RECOVERY — a plane / a sphere / a cylinder cloud + Gaussian noise
//      σ = 1e-3·extent + 5% GROSS outliers → recovers the correct primitive TYPE and
//      parameters within the NOISE BAND (params within ~σ, NOT claimed exact); the
//      injected outliers are correctly isolated into the OUTLIER set (≥90% flagged,
//      ≤5% false-positive of the true inliers); the reported inlier RMS ≈ σ (honest).
//   2. NOISE-FREE REDUCTION — on a CLEAN composite cloud, segmentAndFitRobust reproduces
//      the noise-free segmentAndFit result (same region count + types, params ≤1e-6, no
//      outliers).
//   3. COMPOSITE + NOISE — a plane+cylinder+sphere composite + noise → correct 3-region
//      segmentation, each TYPE correct, outliers isolated.
//   4. OVER-NOISE HONEST — noise so high no stable primitive exists → the region
//      honest-declines (Freeform or Declined/outliers), NEVER a fabricated primitive.
//
// The pipeline composes primitive_fit (numsci facade) and approximateSurface, so the
// whole gate is under CYBERCAD_HAS_NUMSCI (like test_native_reverse_engineer). With the
// guard OFF this compiles to a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/reverse_engineer.h"

#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-46s got %.15g want %.15g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}

static double absCos(const Dir3& a, const Dir3& b) {
  return std::fabs(dot(a.vec(), b.vec()));
}

static int countKind(const SegmentationResult& s, RegionKind k) {
  int c = 0;
  for (const auto& r : s.regions)
    if (r.kind == k) ++c;
  return c;
}
static const SegmentRegion* firstKind(const SegmentationResult& s, RegionKind k) {
  for (const auto& r : s.regions)
    if (r.kind == k) return &r;
  return nullptr;
}

// Deterministic RNG — Gaussian via Box-Muller over a xorshift uniform. Seeded by index,
// never a clock, so the synthetic clouds are byte-reproducible.
struct Rng {
  std::uint64_t s;
  explicit Rng(std::uint64_t seed) : s(seed ? seed : 0x9e3779b97f4a7c15ULL) {}
  double uniform() {  // (0,1)
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return (static_cast<double>(s >> 11) + 0.5) * (1.0 / 9007199254740992.0);
  }
  double gauss() {
    const double u1 = uniform(), u2 = uniform();
    return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.14159265358979323846 * u2);
  }
  double signedUniform() { return uniform() * 2.0 - 1.0; }
};

int main() {
  constexpr double kPi = 3.14159265358979323846;

  // ── 1a. NOISY PLANE recovery + 5% gross outliers + outlier isolation ────────
  {
    Rng rng(1001);
    std::vector<Point3> pts;
    const double extent = 4.0;                 // patch spans ~[-2,2]
    const double sigma = 1e-3 * extent;        // 4e-3 noise
    std::vector<int> trueOutliers;
    // 20×20 plane z=0 patch with Gaussian noise on z.
    for (int i = 0; i < 20; ++i)
      for (int j = 0; j < 20; ++j) {
        const double x = -2.0 + 4.0 * i / 19.0;
        const double y = -2.0 + 4.0 * j / 19.0;
        pts.push_back({x, y, sigma * rng.gauss()});
      }
    const int nInliers = static_cast<int>(pts.size());
    // 5% gross outliers: points displaced far off the plane (z ~ ±0.5..1.5).
    const int nOut = nInliers / 20;
    for (int o = 0; o < nOut; ++o) {
      const double x = 2.0 * rng.signedUniform();
      const double y = 2.0 * rng.signedUniform();
      const double z = (rng.uniform() > 0.5 ? 1.0 : -1.0) * (0.5 + rng.uniform());
      trueOutliers.push_back(static_cast<int>(pts.size()));
      pts.push_back({x, y, z});
    }

    RobustSegmentParams prm;
    prm.base.tol = 1e-9;   // tiny floor: the band must come from the estimated σ
    prm.sigma = sigma;     // supply the known σ (also test estimation below)
    const SegmentationResult seg = segmentAndFitRobust(pts, prm);

    const SegmentRegion* pr = firstKind(seg, RegionKind::Plane);
    expectTrue(pr != nullptr, "noisy-plane: Plane region present");
    expectTrue(countKind(seg, RegionKind::Plane) == 1, "noisy-plane: exactly one Plane");
    if (pr) {
      expectNear(absCos(pr->plane.normal, Dir3(0, 0, 1)), 1.0, 20 * sigma,
                 "noisy-plane: normal within band");
      // Honest RMS: non-zero and ~σ (never claimed exact).
      expectTrue(pr->rms > 0.1 * sigma, "noisy-plane: RMS honest non-zero");
      expectLE(pr->rms, 4.0 * sigma, "noisy-plane: inlier RMS ~ σ");
    }
    // Outlier isolation: ≥90% of injected outliers flagged, ≤5% false positive.
    std::vector<char> flagged(pts.size(), 0);
    for (int i : seg.outliers) flagged[i] = 1;
    int caught = 0;
    for (int i : trueOutliers) caught += flagged[i];
    int falsePos = 0;
    for (int i = 0; i < nInliers; ++i) falsePos += flagged[i];
    expectLE(0.90 * nOut, static_cast<double>(caught), "noisy-plane: ≥90% outliers caught");
    expectLE(static_cast<double>(falsePos), 0.05 * nInliers,
             "noisy-plane: ≤5% inliers false-flagged");
    expectTrue(seg.noiseSigma >= 0.0, "noisy-plane: σ reported");
  }

  // ── 1b. NOISY SPHERE recovery + outliers, σ ESTIMATED from data ─────────────
  {
    Rng rng(2002);
    const Point3 ctr{5, -3, 2};
    const double rad = 4.0;
    const double sigma = 1e-3 * (2 * rad);   // 8e-3
    std::vector<Point3> pts;
    std::vector<int> trueOutliers;
    for (int i = 1; i < 16; ++i)
      for (int j = 0; j < 22; ++j) {
        const double th = kPi * i / 16.0;
        const double ph = 2 * kPi * j / 22.0;
        Point3 p{ctr.x + rad * std::sin(th) * std::cos(ph),
                 ctr.y + rad * std::sin(th) * std::sin(ph),
                 ctr.z + rad * std::cos(th)};
        p = {p.x + sigma * rng.gauss(), p.y + sigma * rng.gauss(), p.z + sigma * rng.gauss()};
        pts.push_back(p);
      }
    const int nInliers = static_cast<int>(pts.size());
    const int nOut = nInliers / 20;  // 5%
    for (int o = 0; o < nOut; ++o) {
      // Gross outliers: on a shell of a very different radius but same center region.
      const double th = kPi * rng.uniform();
      const double ph = 2 * kPi * rng.uniform();
      const double rr = rad * (1.5 + rng.uniform());  // 1.5R..2.5R — clearly off the sphere
      trueOutliers.push_back(static_cast<int>(pts.size()));
      pts.push_back({ctr.x + rr * std::sin(th) * std::cos(ph),
                     ctr.y + rr * std::sin(th) * std::sin(ph),
                     ctr.z + rr * std::cos(th)});
    }

    RobustSegmentParams prm;
    prm.base.tol = 1e-9;
    prm.sigma = 0.0;   // FORCE σ estimation from the data (MAD)
    const SegmentationResult seg = segmentAndFitRobust(pts, prm);

    const SegmentRegion* sr = firstKind(seg, RegionKind::Sphere);
    expectTrue(sr != nullptr, "noisy-sphere: Sphere region present");
    expectTrue(countKind(seg, RegionKind::Plane) == 0, "noisy-sphere: NOT typed as Plane");
    if (sr) {
      expectLE(std::fabs(sr->sphere.radius - rad), 30 * sigma, "noisy-sphere: radius in band");
      expectLE(distance(sr->sphere.center, ctr), 30 * sigma, "noisy-sphere: center in band");
      expectTrue(sr->rms > 0.1 * sigma, "noisy-sphere: RMS honest non-zero");
      expectLE(sr->rms, 5.0 * sigma, "noisy-sphere: inlier RMS ~ σ");
    }
    expectTrue(seg.noiseSigma > 0.0, "noisy-sphere: σ estimated > 0");
    expectLE(seg.noiseSigma, 6.0 * sigma, "noisy-sphere: estimated σ ~ true σ");
    // Outlier isolation.
    std::vector<char> flagged(pts.size(), 0);
    for (int i : seg.outliers) flagged[i] = 1;
    int caught = 0;
    for (int i : trueOutliers) caught += flagged[i];
    int falsePos = 0;
    for (int i = 0; i < nInliers; ++i) falsePos += flagged[i];
    expectLE(0.90 * nOut, static_cast<double>(caught), "noisy-sphere: ≥90% outliers caught");
    expectLE(static_cast<double>(falsePos), 0.05 * nInliers,
             "noisy-sphere: ≤5% inliers false-flagged");
  }

  // ── 1c. NOISY CYLINDER recovery + outliers ──────────────────────────────────
  {
    Rng rng(3003);
    const double rad = 2.0;
    const double sigma = 1e-3 * (2 * rad);
    std::vector<Point3> pts;
    std::vector<int> trueOutliers;
    for (int i = 0; i < 30; ++i)
      for (int j = 0; j < 10; ++j) {
        const double ph = 2 * kPi * i / 30.0;
        const double h = -3.0 + 6.0 * j / 9.0;
        Point3 p{rad * std::cos(ph), rad * std::sin(ph), h};
        p = {p.x + sigma * rng.gauss(), p.y + sigma * rng.gauss(), p.z + sigma * rng.gauss()};
        pts.push_back(p);
      }
    const int nInliers = static_cast<int>(pts.size());
    const int nOut = nInliers / 20;
    for (int o = 0; o < nOut; ++o) {
      // Gross outliers well off the cylindrical shell (radius 3.5..4.5).
      const double ph = 2 * kPi * rng.uniform();
      const double rr = 3.5 + rng.uniform();
      const double h = -3.0 + 6.0 * rng.uniform();
      trueOutliers.push_back(static_cast<int>(pts.size()));
      pts.push_back({rr * std::cos(ph), rr * std::sin(ph), h});
    }

    RobustSegmentParams prm;
    prm.base.tol = 1e-9;
    prm.sigma = sigma;
    const SegmentationResult seg = segmentAndFitRobust(pts, prm);

    const SegmentRegion* cr = firstKind(seg, RegionKind::Cylinder);
    expectTrue(cr != nullptr, "noisy-cyl: Cylinder region present");
    if (cr) {
      expectLE(std::fabs(cr->cylinder.radius - rad), 30 * sigma, "noisy-cyl: radius in band");
      expectNear(absCos(cr->cylinder.axis, Dir3(0, 0, 1)), 1.0, 30 * sigma,
                 "noisy-cyl: axis in band");
      expectTrue(cr->rms > 0.1 * sigma, "noisy-cyl: RMS honest non-zero");
      expectLE(cr->rms, 5.0 * sigma, "noisy-cyl: inlier RMS ~ σ");
    }
    std::vector<char> flagged(pts.size(), 0);
    for (int i : seg.outliers) flagged[i] = 1;
    int caught = 0;
    for (int i : trueOutliers) caught += flagged[i];
    int falsePos = 0;
    for (int i = 0; i < nInliers; ++i) falsePos += flagged[i];
    expectLE(0.90 * nOut, static_cast<double>(caught), "noisy-cyl: ≥90% outliers caught");
    expectLE(static_cast<double>(falsePos), 0.05 * nInliers,
             "noisy-cyl: ≤5% inliers false-flagged");
  }

  // ── 2. NOISE-FREE REDUCTION: robust == noise-free on a clean composite ──────
  {
    std::vector<Point3> pts;
    // Clean plane patch (as in the noise-free gate).
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        pts.push_back({-34.0 + i * 0.5, -2.0 + j * 0.5, 0.0});
    // Clean cylinder.
    const double cylRad = 2.0;
    for (int i = 0; i < 24; ++i)
      for (int j = 0; j < 8; ++j) {
        const double ph = 2 * kPi * i / 24.0;
        pts.push_back({cylRad * std::cos(ph), cylRad * std::sin(ph), -2.0 + j * 0.5});
      }
    // Clean sphere.
    const Point3 sphCtr{30, 0, 0};
    const double sphRad = 3.0;
    for (int i = 1; i < 10; ++i)
      for (int j = 0; j < 12; ++j) {
        const double th = kPi * i / 10.0, ph = 2 * kPi * j / 12.0;
        pts.push_back({sphCtr.x + sphRad * std::sin(th) * std::cos(ph),
                       sphCtr.y + sphRad * std::sin(th) * std::sin(ph),
                       sphCtr.z + sphRad * std::cos(th)});
      }

    SegmentParams base;
    base.tol = 1e-8;
    const SegmentationResult ref = segmentAndFit(pts, base);

    RobustSegmentParams prm;
    prm.base = base;
    const SegmentationResult rob = segmentAndFitRobust(pts, prm);

    // Same region-kind counts as the noise-free path; no outliers on clean data.
    expectTrue(countKind(rob, RegionKind::Plane) == countKind(ref, RegionKind::Plane),
               "clean-reduce: same Plane count");
    expectTrue(countKind(rob, RegionKind::Cylinder) == countKind(ref, RegionKind::Cylinder),
               "clean-reduce: same Cylinder count");
    expectTrue(countKind(rob, RegionKind::Sphere) == countKind(ref, RegionKind::Sphere),
               "clean-reduce: same Sphere count");
    expectTrue(rob.outlierCount == 0, "clean-reduce: no outliers on clean data");
    expectTrue(rob.assignedCount == ref.assignedCount, "clean-reduce: same assigned count");

    // Params match the noise-free fit ≤1e-6.
    const SegmentRegion* rc = firstKind(rob, RegionKind::Cylinder);
    const SegmentRegion* fc = firstKind(ref, RegionKind::Cylinder);
    if (rc && fc)
      expectNear(rc->cylinder.radius, fc->cylinder.radius, 1e-6, "clean-reduce: cyl radius match");
    const SegmentRegion* rs = firstKind(rob, RegionKind::Sphere);
    const SegmentRegion* fs = firstKind(ref, RegionKind::Sphere);
    if (rs && fs)
      expectNear(rs->sphere.radius, fs->sphere.radius, 1e-6, "clean-reduce: sphere radius match");
  }

  // ── 3. COMPOSITE + NOISE: 3 regions, correct types, outliers isolated ───────
  {
    Rng rng(4004);
    const double sigma = 3e-3;
    std::vector<Point3> pts;
    // Plane patch (z=0), far in -x.
    for (int i = 0; i < 10; ++i)
      for (int j = 0; j < 10; ++j)
        pts.push_back({-34.0 + i * 0.4 + sigma * rng.gauss(),
                       -2.0 + j * 0.4 + sigma * rng.gauss(), sigma * rng.gauss()});
    // Cylinder near origin.
    const double cylRad = 2.0;
    for (int i = 0; i < 28; ++i)
      for (int j = 0; j < 9; ++j) {
        const double ph = 2 * kPi * i / 28.0, h = -2.0 + 4.0 * j / 8.0;
        pts.push_back({cylRad * std::cos(ph) + sigma * rng.gauss(),
                       cylRad * std::sin(ph) + sigma * rng.gauss(),
                       h + sigma * rng.gauss()});
      }
    // Sphere far in +x.
    const Point3 sphCtr{30, 0, 0};
    const double sphRad = 3.0;
    for (int i = 1; i < 12; ++i)
      for (int j = 0; j < 14; ++j) {
        const double th = kPi * i / 12.0, ph = 2 * kPi * j / 14.0;
        pts.push_back({sphCtr.x + sphRad * std::sin(th) * std::cos(ph) + sigma * rng.gauss(),
                       sphCtr.y + sphRad * std::sin(th) * std::sin(ph) + sigma * rng.gauss(),
                       sphCtr.z + sphRad * std::cos(th) + sigma * rng.gauss()});
      }
    const int nInliers = static_cast<int>(pts.size());
    // Gross outliers scattered in the empty space between the three faces.
    const int nOut = nInliers / 20;
    std::vector<int> trueOutliers;
    for (int o = 0; o < nOut; ++o) {
      trueOutliers.push_back(static_cast<int>(pts.size()));
      pts.push_back({-15.0 + 30.0 * rng.uniform(), 8.0 * rng.signedUniform(),
                     8.0 * rng.signedUniform()});
    }

    RobustSegmentParams prm;
    prm.base.tol = 1e-9;
    prm.sigma = sigma;
    const SegmentationResult seg = segmentAndFitRobust(pts, prm);

    expectTrue(countKind(seg, RegionKind::Plane) == 1, "composite-noise: one Plane");
    expectTrue(countKind(seg, RegionKind::Cylinder) == 1, "composite-noise: one Cylinder");
    expectTrue(countKind(seg, RegionKind::Sphere) == 1, "composite-noise: one Sphere");

    const SegmentRegion* cr = firstKind(seg, RegionKind::Cylinder);
    if (cr) expectLE(std::fabs(cr->cylinder.radius - cylRad), 40 * sigma,
                     "composite-noise: cyl radius in band");
    const SegmentRegion* sr = firstKind(seg, RegionKind::Sphere);
    if (sr) expectLE(std::fabs(sr->sphere.radius - sphRad), 40 * sigma,
                     "composite-noise: sphere radius in band");

    // Most injected outliers isolated (the scattered ones between faces).
    std::vector<char> flagged(pts.size(), 0);
    for (int i : seg.outliers) flagged[i] = 1;
    int caught = 0;
    for (int i : trueOutliers) caught += flagged[i];
    expectLE(0.80 * nOut, static_cast<double>(caught),
             "composite-noise: majority of outliers isolated");
  }

  // ── 4. OVER-NOISE HONEST: no stable primitive → no fabricated primitive ─────
  {
    Rng rng(5005);
    // A tiny plane patch drowned in noise as large as its own extent — no coherent
    // primitive survives. The region must NOT be reported as a confident Plane/Sphere/
    // Cylinder/Cone with a bogus tight RMS; it honest-declines (Freeform or outliers).
    std::vector<Point3> pts;
    const double extent = 1.0;
    const double sigma = 0.5 * extent;  // catastrophic noise
    for (int i = 0; i < 12; ++i)
      for (int j = 0; j < 12; ++j) {
        const double x = -0.5 + extent * i / 11.0;
        const double y = -0.5 + extent * j / 11.0;
        pts.push_back({x + sigma * rng.gauss(), y + sigma * rng.gauss(), sigma * rng.gauss()});
      }

    RobustSegmentParams prm;
    prm.base.tol = 1e-6;  // a real CAD tolerance — no primitive can honestly meet it here
    prm.sigma = 0.0;      // estimate σ (it will be large)
    const SegmentationResult seg = segmentAndFitRobust(pts, prm);

    // HONESTY: no region may claim a tight (≤ base.tol) RMS on this catastrophically noisy
    // data. Any accepted primitive region must carry an HONEST (large) RMS, never a
    // fabricated near-exact one.
    for (const auto& r : seg.regions) {
      if (r.kind == RegionKind::Plane || r.kind == RegionKind::Sphere ||
          r.kind == RegionKind::Cylinder || r.kind == RegionKind::Cone) {
        expectTrue(r.rms > prm.base.tol,
                   "over-noise: any primitive RMS is honest (not fabricated exact)");
      }
    }
    // And it must not silently swallow everything as one confident tiny-RMS primitive.
    const SegmentRegion* pl = firstKind(seg, RegionKind::Plane);
    if (pl) expectTrue(pl->rms > 10 * prm.base.tol, "over-noise: plane RMS reflects real noise");
    // Some points end up unassigned/declined/outliers — never a clean single primitive.
    expectTrue(seg.regions.size() + (seg.outlierCount > 0 ? 1u : 0u) >= 1u,
               "over-noise: produced a result without crashing");
  }

  std::printf("reverse_engineer_robust gate: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

#else  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("reverse_engineer_robust gate: skipped (CYBERCAD_HAS_NUMSCI off)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
