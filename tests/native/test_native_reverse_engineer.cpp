// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 7 reverse-engineering PIPELINE —
// segment a raw composite point cloud into typed regions and fit each
// (src/native/math/reverse_engineer.{h,cpp}). OCCT-FREE. The oracles are airtight and
// closed-form:
//
//   1. COMPOSITE RECOVERY (noise-free) — a synthetic cloud from a KNOWN composite
//      (plane + cylinder + sphere trio, disjoint in space) segments into exactly THREE
//      regions, each classified to the CORRECT primitive TYPE with recovered params
//      ≤1e-6, and the inliers correctly partitioned (each region's points really lie on
//      its primitive).
//   2. SINGLE-PRIMITIVE — a pure cylinder cloud → exactly ONE Cylinder region, radius /
//      axis recovered to ≤1e-9.
//   3. FREEFORM REGION — a bicubic-bump grid → exactly ONE Freeform region (not a
//      spurious primitive), fitted patch RMS ≤ tol.
//   4. DISCRIMINATION / NOISE — a sphere region is NOT mis-typed as a plane; small noise
//      still yields the correct type and the reported RMS is the TRUE (non-zero)
//      residual, never claimed exact.
//   5. HONEST-DECLINE — a tiny/degenerate cloud that fits no primitive and cannot form a
//      stable freeform patch is reported Declined (never forced to a wrong primitive).
//
// The pipeline composes primitive_fit (numsci facade) and approximateSurface, so the
// whole gate is under CYBERCAD_HAS_NUMSCI (like test_native_primitive_fit). With the
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

// Count regions of a given kind.
static int countKind(const SegmentationResult& s, RegionKind k) {
  int c = 0;
  for (const auto& r : s.regions)
    if (r.kind == k) ++c;
  return c;
}
// First region of a given kind (nullptr if none).
static const SegmentRegion* firstKind(const SegmentationResult& s, RegionKind k) {
  for (const auto& r : s.regions)
    if (r.kind == k) return &r;
  return nullptr;
}

struct Rng {
  std::uint64_t s;
  explicit Rng(std::uint64_t seed) : s(seed ? seed : 0x9e3779b97f4a7c15ULL) {}
  double next() {  // uniform (-1, 1)
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return (static_cast<double>(s >> 11) * (1.0 / 9007199254740992.0)) * 2.0 - 1.0;
  }
};

int main() {
  constexpr double kPi = 3.14159265358979323846;

  // ── 1. COMPOSITE RECOVERY: plane + cylinder + sphere trio ───────────────────
  // Three primitives placed far apart in space so region growing separates them
  // cleanly. Each is sampled on a dense structured grid.
  {
    std::vector<Point3> pts;

    // (a) PLANE region near x≈-30: z = 0 plane patch (normal +Z), 8×8 grid.
    for (int i = 0; i < 8; ++i)
      for (int j = 0; j < 8; ++j)
        pts.push_back({-34.0 + i * 0.5, -2.0 + j * 0.5, 0.0});
    const int planeCount = static_cast<int>(pts.size());

    // (b) CYLINDER region near origin: axis +Z through (0,0,·), radius 2.0.
    const double cylRad = 2.0;
    for (int i = 0; i < 24; ++i)
      for (int j = 0; j < 8; ++j) {
        const double ph = 2 * kPi * i / 24.0;
        const double h = -2.0 + j * 0.5;
        pts.push_back({cylRad * std::cos(ph), cylRad * std::sin(ph), h});
      }
    const int cylCount = static_cast<int>(pts.size()) - planeCount;

    // (c) SPHERE region near x≈+30: center (30,0,0), radius 3.0.
    const Point3 sphCtr{30, 0, 0};
    const double sphRad = 3.0;
    for (int i = 1; i < 10; ++i)
      for (int j = 0; j < 12; ++j) {
        const double th = kPi * i / 10.0;
        const double ph = 2 * kPi * j / 12.0;
        pts.push_back({sphCtr.x + sphRad * std::sin(th) * std::cos(ph),
                       sphCtr.y + sphRad * std::sin(th) * std::sin(ph),
                       sphCtr.z + sphRad * std::cos(th)});
      }
    const int sphCount = static_cast<int>(pts.size()) - planeCount - cylCount;

    SegmentParams prm;
    prm.tol = 1e-8;
    const SegmentationResult seg = segmentAndFit(pts, prm);

    // Exactly three non-declined regions, one of each expected type.
    expectTrue(countKind(seg, RegionKind::Plane) == 1, "composite: exactly one Plane");
    expectTrue(countKind(seg, RegionKind::Cylinder) == 1, "composite: exactly one Cylinder");
    expectTrue(countKind(seg, RegionKind::Sphere) == 1, "composite: exactly one Sphere");
    expectTrue(seg.declinedCount == 0, "composite: nothing declined");
    expectTrue(seg.assignedCount == static_cast<int>(pts.size()),
               "composite: all points assigned");

    // Inlier partition sizes match the seeded counts.
    const SegmentRegion* pr = firstKind(seg, RegionKind::Plane);
    const SegmentRegion* cr = firstKind(seg, RegionKind::Cylinder);
    const SegmentRegion* sr = firstKind(seg, RegionKind::Sphere);
    expectTrue(pr && static_cast<int>(pr->inliers.size()) == planeCount,
               "composite: plane inlier count");
    expectTrue(cr && static_cast<int>(cr->inliers.size()) == cylCount,
               "composite: cylinder inlier count");
    expectTrue(sr && static_cast<int>(sr->inliers.size()) == sphCount,
               "composite: sphere inlier count");

    // Recovered params ≤ 1e-6 (noise-free).
    if (pr) {
      expectNear(absCos(pr->plane.normal, Dir3(0, 0, 1)), 1.0, 1e-6, "composite: plane normal");
      expectLE(pr->rms, 1e-6, "composite: plane RMS");
    }
    if (cr) {
      expectNear(cr->cylinder.radius, cylRad, 1e-6, "composite: cylinder radius");
      expectNear(absCos(cr->cylinder.axis, Dir3(0, 0, 1)), 1.0, 1e-6, "composite: cylinder axis");
      expectLE(cr->rms, 1e-6, "composite: cylinder RMS");
    }
    if (sr) {
      expectNear(distance(sr->sphere.center, sphCtr), 0.0, 1e-6, "composite: sphere center");
      expectNear(sr->sphere.radius, sphRad, 1e-6, "composite: sphere radius");
      expectLE(sr->rms, 1e-6, "composite: sphere RMS");
    }

    // Inliers really lie on their primitive: verify plane inliers have z≈0, cylinder
    // inliers have radius≈cylRad, sphere inliers are ≈sphRad from sphCtr.
    if (pr)
      for (int idx : pr->inliers)
        expectLE(std::fabs(pts[idx].z), 1e-6, "composite: plane inlier on plane");
    if (cr)
      for (int idx : cr->inliers) {
        const double rr = std::sqrt(pts[idx].x * pts[idx].x + pts[idx].y * pts[idx].y);
        expectLE(std::fabs(rr - cylRad), 1e-6, "composite: cyl inlier on cylinder");
      }
    if (sr)
      for (int idx : sr->inliers)
        expectLE(std::fabs(distance(pts[idx], sphCtr) - sphRad), 1e-6,
                 "composite: sphere inlier on sphere");
  }

  // ── 2. SINGLE-PRIMITIVE: pure cylinder → ONE Cylinder region ────────────────
  {
    const Point3 a0{1, 1, 0};
    Vec3 axis{1, 2, 2};
    axis = axis / std::sqrt(dot(axis, axis));
    const double rad = 2.5;
    Vec3 t = (std::fabs(axis.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 e1 = cross(axis, t); e1 = e1 / std::sqrt(dot(e1, e1));
    Vec3 e2 = cross(axis, e1);
    std::vector<Point3> pts;
    for (int i = 0; i < 24; ++i)
      for (int j = 0; j < 8; ++j) {
        const double ph = 2 * kPi * i / 24.0;
        const double h = -4 + j;
        pts.push_back(a0 + e1 * (rad * std::cos(ph)) + e2 * (rad * std::sin(ph)) + axis * h);
      }
    SegmentParams prm;
    prm.tol = 1e-8;
    const SegmentationResult seg = segmentAndFit(pts, prm);
    expectTrue(countKind(seg, RegionKind::Cylinder) == 1, "single: exactly one Cylinder");
    expectTrue(seg.regions.size() == 1, "single: exactly one region total");
    const SegmentRegion* cr = firstKind(seg, RegionKind::Cylinder);
    expectTrue(cr && static_cast<int>(cr->inliers.size()) == static_cast<int>(pts.size()),
               "single: all points in the cylinder region");
    if (cr) {
      expectNear(cr->cylinder.radius, rad, 1e-9, "single: cylinder radius ≤1e-9");
      expectNear(absCos(cr->cylinder.axis, Dir3(axis)), 1.0, 1e-9, "single: cylinder axis ≤1e-9");
    }
  }

  // ── 3. FREEFORM REGION: bicubic bump → ONE Freeform region ──────────────────
  {
    std::vector<Point3> pts;
    const int N = 12;
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < N; ++j) {
        const double x = -1 + 2.0 * i / (N - 1);
        const double y = -1 + 2.0 * j / (N - 1);
        // Genuinely non-primitive smooth height field (cubic saddle + gentle bump).
        const double z = 0.3 * (x * x * x - x) + 0.2 * (y * y * y) + 0.25 * x * y;
        pts.push_back({x, y, z});
      }
    SegmentParams prm;
    prm.tol = 1e-2;  // loose absolute tol: no primitive fits this, freeform patch must.
    const SegmentationResult seg = segmentAndFit(pts, prm);
    expectTrue(countKind(seg, RegionKind::Freeform) == 1, "freeform: exactly one Freeform");
    expectTrue(countKind(seg, RegionKind::Plane) == 0, "freeform: no spurious Plane");
    expectTrue(countKind(seg, RegionKind::Sphere) == 0, "freeform: no spurious Sphere");
    expectTrue(countKind(seg, RegionKind::Cylinder) == 0, "freeform: no spurious Cylinder");
    expectTrue(countKind(seg, RegionKind::Cone) == 0, "freeform: no spurious Cone");
    const SegmentRegion* fr = firstKind(seg, RegionKind::Freeform);
    expectTrue(fr != nullptr, "freeform: region present");
    if (fr) {
      expectLE(fr->rms, prm.tol, "freeform: patch RMS ≤ tol");
      expectTrue(!fr->surface.poles.empty(), "freeform: patch has poles");
    }
  }

  // ── 4. DISCRIMINATION / NOISE: sphere not mis-typed; RMS honest ─────────────
  {
    const Point3 ctr{4, 4, -2};
    const double rad = 5.0;
    const double sigma = 1e-3;
    Rng rng(2024);
    std::vector<Point3> pts;
    for (int i = 1; i < 12; ++i)
      for (int j = 0; j < 16; ++j) {
        const double th = kPi * i / 12.0;
        const double ph = 2 * kPi * j / 16.0;
        Point3 p{ctr.x + rad * std::sin(th) * std::cos(ph),
                 ctr.y + rad * std::sin(th) * std::sin(ph),
                 ctr.z + rad * std::cos(th)};
        p = {p.x + sigma * rng.next(), p.y + sigma * rng.next(), p.z + sigma * rng.next()};
        pts.push_back(p);
      }
    SegmentParams prm;
    prm.tol = 1e-2;  // loose enough to admit the noisy sphere as a Sphere.
    const SegmentationResult seg = segmentAndFit(pts, prm);
    const SegmentRegion* sr = firstKind(seg, RegionKind::Sphere);
    expectTrue(sr != nullptr, "noise: sphere region present");
    expectTrue(countKind(seg, RegionKind::Plane) == 0, "noise: sphere NOT typed as Plane");
    if (sr) {
      // Honesty: reported RMS is the TRUE non-zero residual (~ noise), never claimed 0.
      expectTrue(sr->rms > 0.1 * sigma, "noise: sphere RMS reported honestly (non-zero)");
      expectLE(sr->rms, 5 * sigma, "noise: sphere RMS ~ noise band");
      expectLE(std::fabs(sr->sphere.radius - rad), 20 * sigma, "noise: sphere radius in band");
    }
  }

  // ── 5. HONEST-DECLINE: tiny/degenerate cloud ───────────────────────────────
  {
    // Four scattered points — below any primitive minimum and not a lattice.
    std::vector<Point3> pts{{0, 0, 0}, {1, 0, 0.3}, {0, 1, -0.2}, {1, 1, 0.7}};
    const SegmentationResult seg = segmentAndFit(pts);
    // Nothing is forced to a primitive; the points are honestly declined.
    expectTrue(countKind(seg, RegionKind::Plane) == 0, "decline: no forced Plane");
    expectTrue(countKind(seg, RegionKind::Sphere) == 0, "decline: no forced Sphere");
    expectTrue(countKind(seg, RegionKind::Cylinder) == 0, "decline: no forced Cylinder");
    expectTrue(countKind(seg, RegionKind::Cone) == 0, "decline: no forced Cone");
    expectTrue(seg.declinedCount == static_cast<int>(pts.size()), "decline: all declined");
  }

  // ── 6. WIDE-CONE DISCRIMINATION (regression) — a narrow-band wide cone is a CONE,
  //       not a bogus sphere ─────────────────────────────────────────────────────
  // A machined countersink / chamfer scanned over a limited height band is a WIDE
  // half-angle cone frustum. At a realistic (loose) scan tolerance such a band is ALSO
  // fit — badly, as a huge-radius sphere — within the relative tolerance, and because a
  // sphere is "simpler" than a cone the old simplicity-first discrimination returned the
  // SPHERE (a mis-classification: wrong type, nonsense radius). The fix keeps simple-on-
  // ties but rejects a simpler primitive that is DECISIVELY worse than a within-tol cone.
  // Here the cone fits ~machine-exact while the best sphere is ~1e-3 relative — no tie —
  // so the region MUST be a Cone, with the true apex / axis / half-angle recovered.
  {
    const Point3 apex{2, -1, 3};
    Vec3 axis{1, 2, 2};
    axis = axis / std::sqrt(dot(axis, axis));  // tilted (not covariance-aligned)
    const double alpha = 55.0 * kPi / 180.0;   // WIDE half-angle
    Vec3 t = (std::fabs(axis.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 e1 = cross(axis, t);
    e1 = e1 / std::sqrt(dot(e1, e1));
    Vec3 e2 = cross(axis, e1);
    std::vector<Point3> pts;
    // Narrow height band h ∈ [3, 4] (a limited frustum, as a real scan of a countersink).
    for (int i = 0; i < 30; ++i)
      for (int j = 0; j < 8; ++j) {
        const double ph = 2 * kPi * i / 30.0;
        const double h = 3.0 + 1.0 * j / 7.0;
        const double rho = h * std::tan(alpha);
        pts.push_back(apex + axis * h + (e1 * std::cos(ph) + e2 * std::sin(ph)) * rho);
      }

    // A LOOSE (mm-scale-scanner) tolerance — the regime where the bogus sphere used to
    // sneak under the relative tolerance and win on simplicity.
    SegmentParams prm;
    prm.tol = 1e-2;
    const SegmentationResult seg = segmentAndFit(pts, prm);

    expectTrue(countKind(seg, RegionKind::Cone) == 1, "wide-cone: exactly one Cone");
    expectTrue(countKind(seg, RegionKind::Sphere) == 0, "wide-cone: NOT mis-typed as Sphere");
    const SegmentRegion* cr = firstKind(seg, RegionKind::Cone);
    expectTrue(cr != nullptr, "wide-cone: Cone region present");
    if (cr) {
      expectNear(absCos(cr->cone.axis, Dir3(axis)), 1.0, 1e-6, "wide-cone: axis recovered");
      expectNear(cr->cone.halfAngle, alpha, 1e-6, "wide-cone: half-angle recovered");
      expectNear(distance(cr->cone.apex, apex), 0.0, 1e-6, "wide-cone: apex recovered");
      expectLE(cr->rms, 1e-6, "wide-cone: RMS ~0 (a true cone fit, not a widened sphere)");
      expectTrue(static_cast<int>(cr->inliers.size()) == static_cast<int>(pts.size()),
                 "wide-cone: whole band owned by the one cone");
    }
  }

  std::printf("reverse_engineer gate: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

#else  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("reverse_engineer gate: skipped (CYBERCAD_HAS_NUMSCI off)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
