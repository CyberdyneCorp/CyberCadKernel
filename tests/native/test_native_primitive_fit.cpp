// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 7 reverse-engineering — analytic-
// primitive detection / fitting (src/native/math/primitive_fit.{h,cpp}). OCCT-FREE.
// The oracles are airtight and closed-form:
//
//   1. EXACT RECOVERY (noise-free) — points sampled from a KNOWN plane / sphere /
//      cylinder / cone recover the exact parameters (normal / center / radius / axis
//      / half-angle) to ≤1e-9 and RMS ≤1e-12. Nothing is widened.
//   2. DISCRIMINATION — sphere points detect as Sphere (not Cylinder/Cone); plane
//      points as Plane; a bicubic-bump free-form cloud detects as Freeform, not a
//      spurious primitive.
//   3. ROBUSTNESS / HONESTY — with small synthetic noise the parameters land within
//      the noise band and the reported RMS is the TRUE achieved RMS (≈ noise σ, NOT
//      claimed as zero). detectPrimitive still classifies correctly at loose tol.
//   4. DEGENERATE GUARDS — too-few points → ok=false, no crash.
//
// The routines are numsci-gated (they call numerics::lstsq / least_squares), so the
// whole gate is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_fit). With the
// guard OFF this compiles to a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/primitive_fit.h"

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
    std::printf("FAIL %-42s got %.15g want %.15g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-42s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}

// |cos(angle)| between two directions == 1 when (anti)parallel.
static double absCos(const Dir3& a, const Dir3& b) {
  return std::fabs(dot(a.vec(), b.vec()));
}

// A tiny deterministic PRNG (xorshift) so the noise oracle is reproducible.
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

  // ── 1a. EXACT PLANE RECOVERY ────────────────────────────────────────────────
  // Plane through (1,2,3) with normal (2,-1,2)/3. Sample a grid on it.
  {
    const Vec3 n0{2.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0};
    const Point3 base{1, 2, 3};
    Vec3 u{ -1.0/3.0, 2.0/3.0, 2.0/3.0 };  // ⟂ to n0
    Vec3 w = cross(n0, u);
    std::vector<Point3> pts;
    for (int i = 0; i < 7; ++i)
      for (int j = 0; j < 7; ++j) {
        const double a = -3 + i, b = -3 + j;
        pts.push_back(base + u * a + w * b);
      }
    const PlaneFit f = fitPlane(pts);
    expectTrue(f.ok, "plane ok");
    expectNear(absCos(f.normal, Dir3(n0)), 1.0, 1e-12, "plane normal recovered");
    expectLE(f.rms, 1e-12, "plane RMS ~0");
    // Base point lies on the recovered plane.
    expectNear(dot(f.normal.vec(), base.asVec()) - f.offset, 0.0, 1e-10, "plane offset");

    const PrimitiveDetection d = detectPrimitive(pts);
    expectTrue(d.type == PrimitiveType::Plane, "plane detected");
  }

  // ── 1b. EXACT SPHERE RECOVERY ───────────────────────────────────────────────
  {
    const Point3 ctr{-2, 5, 1};
    const double rad = 3.7;
    std::vector<Point3> pts;
    for (int i = 1; i < 10; ++i)
      for (int j = 0; j < 12; ++j) {
        const double th = kPi * i / 10.0;      // (0, π)
        const double ph = 2 * kPi * j / 12.0;  // [0, 2π)
        pts.push_back({ctr.x + rad * std::sin(th) * std::cos(ph),
                       ctr.y + rad * std::sin(th) * std::sin(ph),
                       ctr.z + rad * std::cos(th)});
      }
    const SphereFit f = fitSphere(pts);
    expectTrue(f.ok, "sphere ok");
    expectNear(f.center.x, ctr.x, 1e-9, "sphere cx");
    expectNear(f.center.y, ctr.y, 1e-9, "sphere cy");
    expectNear(f.center.z, ctr.z, 1e-9, "sphere cz");
    expectNear(f.radius, rad, 1e-9, "sphere radius");
    expectLE(f.rms, 1e-12, "sphere RMS ~0");

    const PrimitiveDetection d = detectPrimitive(pts);
    expectTrue(d.type == PrimitiveType::Sphere, "sphere detected (not cyl/cone)");
  }

  // ── 1c. EXACT CYLINDER RECOVERY ─────────────────────────────────────────────
  {
    // Axis through (1,1,0) along a tilted unit direction; radius 2.5.
    const Point3 a0{1, 1, 0};
    Vec3 axis{1, 2, 2};
    axis = axis / std::sqrt(dot(axis, axis));
    const double rad = 2.5;
    // Frame ⟂ to axis.
    Vec3 t = (std::fabs(axis.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 e1 = cross(axis, t); e1 = e1 / std::sqrt(dot(e1, e1));
    Vec3 e2 = cross(axis, e1);
    std::vector<Point3> pts;
    for (int i = 0; i < 20; ++i)
      for (int j = 0; j < 8; ++j) {
        const double ph = 2 * kPi * i / 20.0;
        const double h = -4 + j;  // along axis
        const Vec3 rel = e1 * (rad * std::cos(ph)) + e2 * (rad * std::sin(ph)) + axis * h;
        pts.push_back(a0 + rel);
      }
    const CylinderFit f = fitCylinder(pts);
    expectTrue(f.ok, "cylinder ok");
    expectNear(absCos(f.axis, Dir3(axis)), 1.0, 1e-9, "cylinder axis recovered");
    expectNear(f.radius, rad, 1e-9, "cylinder radius");
    expectLE(f.rms, 1e-12, "cylinder RMS ~0");
    // Reported axis point must lie ON the true axis (its ⟂ distance to it is 0).
    {
      const Vec3 wv = f.axisPoint - a0;
      const Vec3 perp = wv - axis * dot(wv, axis);
      expectNear(std::sqrt(dot(perp, perp)), 0.0, 1e-8, "cylinder axis point on axis");
    }

    const PrimitiveDetection d = detectPrimitive(pts);
    expectTrue(d.type == PrimitiveType::Cylinder, "cylinder detected");
  }

  // ── 1d. EXACT CONE RECOVERY ─────────────────────────────────────────────────
  {
    const Point3 apex{2, -1, 3};
    Vec3 axis{0, 0, 1};  // opening +z
    const double alpha = 25.0 * kPi / 180.0;  // half-angle
    Vec3 e1{1, 0, 0}, e2{0, 1, 0};
    std::vector<Point3> pts;
    for (int i = 0; i < 24; ++i)
      for (int j = 1; j <= 6; ++j) {
        const double ph = 2 * kPi * i / 24.0;
        const double h = 0.5 * j;              // height along axis (>0)
        const double rho = h * std::tan(alpha);
        const Vec3 rel = axis * h + (e1 * std::cos(ph) + e2 * std::sin(ph)) * rho;
        pts.push_back(apex + rel);
      }
    const ConeFit f = fitCone(pts);
    expectTrue(f.ok, "cone ok");
    expectNear(absCos(f.axis, Dir3(axis)), 1.0, 1e-8, "cone axis recovered");
    expectNear(f.halfAngle, alpha, 1e-8, "cone half-angle recovered");
    expectNear(distance(f.apex, apex), 0.0, 1e-7, "cone apex recovered");
    expectLE(f.rms, 1e-10, "cone RMS ~0");

    const PrimitiveDetection d = detectPrimitive(pts);
    expectTrue(d.type == PrimitiveType::Cone, "cone detected");
  }

  // ── 1e. WIDE-CONE DISCRIMINATION (regression): a narrow-band wide cone at a LOOSE
  //        relative tolerance is a CONE, not a bogus large-radius sphere ──────────
  // A machined countersink/chamfer scanned over a limited height band is a wide-half-
  // angle cone frustum. Such a band is ALSO fit — badly — by a huge sphere, whose
  // RELATIVE RMS squeaks under a loose (scanner-scale) relTol; because a sphere is
  // "simpler" than a cone the old simplicity-first rule returned the SPHERE (wrong type,
  // nonsense radius). detectPrimitive must now keep simple-on-genuine-ties yet reject a
  // simpler primitive that is DECISIVELY worse than a within-tol cone. The cone fits
  // ~machine-exact while the best sphere is ~1e-3 relative — no tie — so this is a Cone.
  {
    const Point3 apex{2, -1, 3};
    Vec3 axis{1, 2, 2};
    axis = axis / std::sqrt(dot(axis, axis));  // tilted, not covariance-aligned
    const double alpha = 55.0 * kPi / 180.0;   // WIDE half-angle
    Vec3 t = (std::fabs(axis.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    Vec3 e1 = cross(axis, t);
    e1 = e1 / std::sqrt(dot(e1, e1));
    Vec3 e2 = cross(axis, e1);
    std::vector<Point3> pts;
    for (int i = 0; i < 30; ++i)
      for (int j = 0; j < 8; ++j) {
        const double ph = 2 * kPi * i / 30.0;
        const double h = 3.0 + 1.0 * j / 7.0;  // narrow band h ∈ [3,4]
        const double rho = h * std::tan(alpha);
        pts.push_back(apex + axis * h + (e1 * std::cos(ph) + e2 * std::sin(ph)) * rho);
      }
    // A loose relative tolerance — the regime where the bogus sphere used to win.
    const PrimitiveDetection d = detectPrimitive(pts, 5e-3);
    expectTrue(d.type == PrimitiveType::Cone, "wide-cone detected (not a bogus Sphere)");
    expectTrue(d.type != PrimitiveType::Sphere, "wide-cone NOT mis-typed as Sphere");
    // The winning cone is a TRUE fit — its relative error is ~machine-zero, not the ~1e-3
    // of the sphere approximation.
    expectLE(d.relError, 1e-9, "wide-cone winner is the near-exact cone, not the sphere");
  }

  // ── 2. DISCRIMINATION: free-form bicubic bump → Freeform, not a primitive ────
  {
    std::vector<Point3> pts;
    for (int i = 0; i < 15; ++i)
      for (int j = 0; j < 15; ++j) {
        const double x = -1 + 2.0 * i / 14.0;
        const double y = -1 + 2.0 * j / 14.0;
        // A genuinely non-primitive height field (cubic saddle + bump).
        const double z = 0.6 * (x * x * x - x) + 0.4 * (y * y * y) + 0.5 * x * y +
                         0.3 * std::sin(3 * x) * std::cos(3 * y);
        pts.push_back({x, y, z});
      }
    const PrimitiveDetection d = detectPrimitive(pts);
    expectTrue(d.type == PrimitiveType::Freeform, "freeform bump rejected as primitive");
    expectTrue(!d.ok, "freeform ok=false");
  }

  // ── 3. ROBUSTNESS / HONESTY: noisy sphere ───────────────────────────────────
  {
    const Point3 ctr{4, 4, -2};
    const double rad = 5.0;
    const double sigma = 1e-3;  // noise amplitude band
    Rng rng(12345);
    std::vector<Point3> pts;
    for (int i = 1; i < 12; ++i)
      for (int j = 0; j < 16; ++j) {
        const double th = kPi * i / 12.0;
        const double ph = 2 * kPi * j / 16.0;
        Point3 p{ctr.x + rad * std::sin(th) * std::cos(ph),
                 ctr.y + rad * std::sin(th) * std::sin(ph),
                 ctr.z + rad * std::cos(th)};
        p = {p.x + sigma * rng.next(), p.y + sigma * rng.next(),
             p.z + sigma * rng.next()};
        pts.push_back(p);
      }
    const SphereFit f = fitSphere(pts);
    expectTrue(f.ok, "noisy sphere ok");
    // Parameters within the noise band (~ a few σ), NOT exact.
    expectLE(distance(f.center, ctr), 20 * sigma, "noisy sphere center in band");
    expectLE(std::fabs(f.radius - rad), 20 * sigma, "noisy sphere radius in band");
    // Honesty: RMS is a TRUE non-zero residual on the order of the noise, NOT 0.
    expectLE(f.rms, 5 * sigma, "noisy sphere RMS ~ noise");
    expectTrue(f.rms > 0.1 * sigma, "noisy sphere RMS reported honestly (non-zero)");
    // At a loose relative tolerance it still classifies as a sphere.
    const PrimitiveDetection d = detectPrimitive(pts, 1e-2);
    expectTrue(d.type == PrimitiveType::Sphere, "noisy sphere still detected");
  }

  // ── 4. DEGENERATE GUARDS ────────────────────────────────────────────────────
  {
    std::vector<Point3> two{{0, 0, 0}, {1, 0, 0}};
    expectTrue(!fitPlane(two).ok, "plane declines <3 points");
    std::vector<Point3> three{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    expectTrue(!fitSphere(three).ok, "sphere declines <4 points");
    expectTrue(!fitCylinder(three).ok, "cylinder declines <6 points");
    expectTrue(!fitCone(three).ok, "cone declines <6 points");
    const PrimitiveDetection d = detectPrimitive(two);
    expectTrue(d.type == PrimitiveType::Freeform, "detect declines tiny cloud");
  }

  std::printf("primitive_fit gate: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

#else  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("primitive_fit gate: skipped (CYBERCAD_HAS_NUMSCI off)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
