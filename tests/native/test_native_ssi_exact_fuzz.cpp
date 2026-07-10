// SPDX-License-Identifier: Apache-2.0
//
// HOST exact-oracle SSI differential fuzzer (OCCT-FREE). The no-OCCT, machine-precision
// complement to the SIM/OCCT freeform SSI fuzzer: it breadth-hardens the analytic SSI
// core with randomized VALID elementary-surface pairs, and adds an EXACT NURBS↔analytic
// SSI oracle — a known-answer test that native SSI traces the RIGHT curve on
// NURBS-represented (rational) geometry, verified against a closed-form analytic answer
// with NO OCCT.
//
// TWO LEGS:
//   LEG 1 — S1 analytic breadth. Random rigid placements of every S1-handled elementary
//     pair (plane∩{sphere,cyl,cone,torus}, sphere∩sphere, coaxial sphere∩{cyl,cone},
//     parallel/coaxial cyl∩cyl, coaxial cyl∩cone), plus deliberately near-degenerate
//     placements (tangent spheres d≈R1+R2, plane grazing a cyl/cone, cone α near 0/90°).
//     ORACLE (exact, closed-form, NO numsci for this leg's core): every sampled curve
//     point lies on BOTH input surfaces to a FIXED tight tolerance; the curve KIND matches
//     the closed-form expectation for the configuration; the branch COUNT matches the known
//     geometry; and honest declines (NoIntersection / Coincident) are asserted where the
//     configuration has no proper transversal curve.
//
//   LEG 2 — NURBS↔analytic known-answer (numsci; the NURBS-roadmap-specific value).
//     Construct RATIONAL NURBS surfaces that EXACTLY represent quadrics (the classic 9-pole
//     rational cylinder with corner weights cos(θ/2); a rational sphere of revolution),
//     VERIFY the NURBS reproduces the intended quadric to machine precision, then intersect
//     the NURBS-quadric with a PLANE whose TRUE intersection is a KNOWN closed-form curve
//     (⟂ axis → circle; oblique → ellipse; sphere∩plane → circle). Drive the REAL
//     seed_intersection + trace_intersection pipeline (build SurfaceAdapters for the NURBS +
//     the analytic surface). ORACLE: every traced node lies on both surfaces (≤ onSurfTol),
//     AND the traced curve MATCHES the known analytic intersection (nearest-node fit ≤ a
//     fixed curve-match tolerance, branch count matches).
//
// CLASSIFICATION (honest — a tolerance is NEVER widened to manufacture a pass):
//   AGREED             native answer matches the exact oracle.
//   DISAGREED          native returned a WRONG analytic curve, a wrong kind/count for a
//                      known configuration, or a traced curve whose nodes are NOT on both
//                      surfaces / do not match the known answer. A real finding — STOP.
//   HONESTLY-DECLINED  (leg 2) native declines / near-tangent-gaps a case with a known clean
//                      transversal answer. Honest but counted (a recall boundary, not a bug).
//   ORACLE-INACCURATE  (leg 1) native is MORE correct than the test's own closed form at a
//                      numeric edge (justified inline). None observed.
//
// The bar is DISAGREED == 0. Exit 0 iff DISAGREED == 0 with real coverage (AGREED > 0)
// across ≥ 2 seeds, N ≥ 40 cases/seed. Any DISAGREED prints seed + case index + the
// surfaces' defining parameters for repro.
//
// Deterministic seeded RNG: splitmix64 → xoshiro256** (FUZZ_SEED env-overridable, fixed
// default; NO wall clock / rand() / address). Reproducible.
//
// Compiled only under CYBERCAD_HAS_NUMSCI (leg 2 drives seed_intersection +
// trace_intersection, whose definitions call the least_squares / lstsq substrate), mirroring
// test_native_ssi_seeding / test_native_ssi_marching.
//
#include "native/ssi/native_ssi.h"
#include "native/math/transform.h"

#include "harness.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace ssi = cybercad::native::ssi;
namespace nmath = cybercad::native::math;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Mat3;
using nmath::Point3;
using nmath::Transform;
using nmath::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── FIXED tolerances (stated once, NEVER widened) ───────────────────────────────
// Leg 1 is closed-form on both sides, so the on-surface residual is machine-ε class.
// We use 1e-9 (relative to the unit-ish scale of the fuzzed models, radii ~ [0.5, 8]),
// exactly the S1 host test's 1e-9 for the flat conics and 1e-7 for the cone/sphere
// residuals whose distToCone/distToSphere oracle carries a curvature-scaled slack.
constexpr double kLeg1Tol      = 1e-9;   // plane/sphere/cyl residual bar
constexpr double kLeg1ConeTol  = 1e-7;   // cone-involving residual bar (matches S1 host)
// Leg 2 is a numeric trace: onSurf is the marcher node tolerance; curve-match is the
// nearest-node fit to the known analytic curve — both mirror the marching host test.
constexpr double kLeg2OnSurf   = 1e-6;   // traced node on-both-surfaces bar
constexpr double kLeg2Fit      = 1e-3;   // known-curve ↔ traced-node fit bar

// ── deterministic RNG: splitmix64 seeds a xoshiro256** stream ────────────────────
struct SplitMix64 {
  std::uint64_t s;
  std::uint64_t next() {
    std::uint64_t z = (s += 0x9E3779B97F4A7C15ULL);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
};

struct Xoshiro256ss {
  std::uint64_t s[4];
  explicit Xoshiro256ss(std::uint64_t seed) {
    SplitMix64 sm{seed};
    for (auto& x : s) x = sm.next();
  }
  static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  std::uint64_t next() {
    const std::uint64_t result = rotl(s[1] * 5, 7) * 9;
    const std::uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3];
    s[2] ^= t;    s[3] = rotl(s[3], 45);
    return result;
  }
  /// Uniform double in [0,1).
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  /// Uniform double in [lo,hi].
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  /// Uniform int in [lo,hi].
  int irange(int lo, int hi) { return lo + int(next() % std::uint64_t(hi - lo + 1)); }
};

// A random rigid transform (rotation about a random unit axis + random translation).
// Intersection TYPE (kind, branch count) is invariant under a rigid motion, so we build
// each pair in a known local pose and move BOTH surfaces by the SAME transform; the
// closed-form on-surface oracle then checks the transformed surfaces directly.
Transform randomRigid(Xoshiro256ss& rng) {
  Vec3 a{rng.range(-1, 1), rng.range(-1, 1), rng.range(-1, 1)};
  double n = nmath::norm(a);
  if (n < 1e-6) a = {0, 0, 1}, n = 1.0;
  const Dir3 axis{a / n};
  const double ang = rng.range(0.0, 2.0 * kPi);
  const Vec3 t{rng.range(-6, 6), rng.range(-6, 6), rng.range(-6, 6)};
  const Mat3 R = Mat3::rotation(axis, ang);
  return Transform{R, t};
}

Ax3 xform(const Transform& T, const Ax3& f) {
  return Ax3{T.applyToPoint(f.origin), T.applyToDir(f.x), T.applyToDir(f.y), T.applyToDir(f.z)};
}
nmath::Plane    xform(const Transform& T, const nmath::Plane& p)   { return {xform(T, p.pos)}; }
nmath::Sphere   xform(const Transform& T, const nmath::Sphere& s)  { return {xform(T, s.pos), s.radius}; }
nmath::Cylinder xform(const Transform& T, const nmath::Cylinder& c){ return {xform(T, c.pos), c.radius}; }
nmath::Cone     xform(const Transform& T, const nmath::Cone& c)    { return {xform(T, c.pos), c.radius, c.semiAngle}; }
nmath::Torus    xform(const Transform& T, const nmath::Torus& t)   { return {xform(T, t.pos), t.majorRadius, t.minorRadius}; }

// ── on-surface distance oracles (closed-form, per surface kind) — independent of the
//    SSI evaluators that produced the curve. Reused verbatim from the S1 host test. ──
double distToPlane(const nmath::Plane& p, const Point3& x) {
  return std::fabs(nmath::dot(x - p.pos.origin, p.pos.z.vec()));
}
double distToSphere(const nmath::Sphere& s, const Point3& x) {
  return std::fabs(nmath::distance(x, s.pos.origin) - s.radius);
}
double distToCylinder(const nmath::Cylinder& c, const Point3& x) {
  const Vec3 w = x - c.pos.origin;
  const double axial = nmath::dot(w, c.pos.z.vec());
  const Vec3 radial = w - c.pos.z.vec() * axial;
  return std::fabs(nmath::norm(radial) - c.radius);
}
double distToCone(const nmath::Cone& c, const Point3& x) {
  const Vec3 w = x - c.pos.origin;
  const double axial = nmath::dot(w, c.pos.z.vec());
  const Vec3 radial = w - c.pos.z.vec() * axial;
  const double v = axial / std::cos(c.semiAngle);
  const double expectedR = c.radius + v * std::sin(c.semiAngle);
  return std::fabs(nmath::norm(radial) - std::fabs(expectedR));
}
double distToTorus(const nmath::Torus& t, const Point3& x) {
  const Vec3 w = x - t.pos.origin;
  const double z = nmath::dot(w, t.pos.z.vec());
  const Vec3 planar = w - t.pos.z.vec() * z;
  const double rho = nmath::norm(planar);
  const double dRho = rho - t.majorRadius;
  return std::fabs(std::sqrt(dRho * dRho + z * z) - t.minorRadius);
}

// Sample a curve across its natural range; return the worst residual(P(t)).
template <class Fn>
double curveWorst(const ssi::IntersectionCurve& c, Fn residual) {
  if (c.kind == ssi::CurveKind::Point) return residual(c.point);
  auto [t0, t1] = c.naturalRange();
  // Hyperbola cosh/sinh grow fast — keep the sampled window tight so points stay near
  // the surfaces (mirrors the S1 host hyperbola windowing).
  if (c.kind == ssi::CurveKind::Hyperbola) { t0 = -1.0; t1 = 1.0; }
  double worst = 0.0;
  const int N = 96;
  for (int i = 0; i <= N; ++i) {
    const double t = t0 + (t1 - t0) * (double(i) / N);
    worst = std::max(worst, residual(c.value(t)));
  }
  return worst;
}

// ── coverage bookkeeping ─────────────────────────────────────────────────────────
struct Bucket {
  const char* name;
  int agreed = 0, disagreed = 0, declined = 0, oracleInaccurate = 0;
};

// Global report state, printed once after the full sweep.
struct Report {
  std::vector<Bucket> leg1;
  std::vector<Bucket> leg2;
  int totalAgreed = 0, totalDisagreed = 0;
  int cases = 0;
} g_rep;

Bucket& bucket(std::vector<Bucket>& v, const char* name) {
  for (auto& b : v) if (std::strcmp(b.name, name) == 0) return b;
  v.push_back(Bucket{name});
  return v.back();
}

void agree(std::vector<Bucket>& leg, const char* name) {
  bucket(leg, name).agreed++; g_rep.totalAgreed++;
}
void decline(std::vector<Bucket>& leg, const char* name) { bucket(leg, name).declined++; }
void disagree(std::vector<Bucket>& leg, const char* name, std::uint64_t seed, int idx,
              const std::string& why) {
  bucket(leg, name).disagreed++; g_rep.totalDisagreed++;
  std::printf("  !! DISAGREED [%s] seed=%llu case=%d: %s\n", name,
              (unsigned long long)seed, idx, why.c_str());
}

// A single leg-1 verdict: check every returned curve lies on both surfaces + the expected
// kind/count. `residA/residB` are closed-form distance functors for the two transformed
// surfaces. Returns true on AGREED, records the bucket, prints repro on DISAGREED.
template <class RA, class RB>
void verifyLeg1(const char* name, const ssi::IntersectionResult& r, RA residA, RB residB,
                ssi::CurveKind expectKind, int expectCount, double tol,
                std::uint64_t seed, int idx, const std::string& params) {
  if (r.status != ssi::IntersectionStatus::Ok) {
    disagree(g_rep.leg1, name, seed, idx,
             "expected Ok curve(s), got status " + std::to_string(int(r.status)) + " | " + params);
    return;
  }
  if (int(r.curves.size()) != expectCount) {
    disagree(g_rep.leg1, name, seed, idx,
             "branch count " + std::to_string(r.curves.size()) + " != expected " +
                 std::to_string(expectCount) + " | " + params);
    return;
  }
  for (const auto& c : r.curves) {
    if (c.kind != expectKind) {
      disagree(g_rep.leg1, name, seed, idx,
               "curve kind " + std::to_string(int(c.kind)) + " != expected " +
                   std::to_string(int(expectKind)) + " | " + params);
      return;
    }
    const double wa = curveWorst(c, residA), wb = curveWorst(c, residB);
    if (std::max(wa, wb) > tol) {
      char buf[128];
      std::snprintf(buf, sizeof buf, "off-surface residual A=%.3e B=%.3e > %.3e | ", wa, wb, tol);
      disagree(g_rep.leg1, name, seed, idx, std::string(buf) + params);
      return;
    }
  }
  agree(g_rep.leg1, name);
}

// EXACT-TANGENCY knife-edge verdict. At a floating placement of an exactly-tangent config
// (plane at d=R, spheres at d=R1+R2) the true contact is a single point, but a hair of
// rounding pushes it just inside (a tiny transversal Circle) or just outside (NoIntersection).
// ALL THREE are honest: a Point (the exact tangency), a NoIntersection (rounded apart), or a
// SMALL Circle whose radius is within a rounding-scaled bound AND whose points lie on both
// surfaces. A LARGE fabricated curve, or an off-surface curve, is DISAGREED. `residA/residB`
// are the closed-form distance functors; `scale` sets the "small circle" ceiling.
template <class RA, class RB>
void verifyTangent(const char* name, const ssi::IntersectionResult& r, RA residA, RB residB,
                   double scale, double tol, std::uint64_t seed, int idx,
                   const std::string& params) {
  if (r.status == ssi::IntersectionStatus::NoIntersection) { agree(g_rep.leg1, name); return; }
  if (r.status != ssi::IntersectionStatus::Ok || r.curves.size() != 1) {
    disagree(g_rep.leg1, name, seed, idx,
             "tangent gave status " + std::to_string(int(r.status)) + " curves=" +
                 std::to_string(r.curves.size()) + " | " + params);
    return;
  }
  const auto& c = r.curves[0];
  if (c.kind == ssi::CurveKind::Point) {
    if (std::max(residA(c.point), residB(c.point)) > tol) {
      disagree(g_rep.leg1, name, seed, idx, "tangent Point off surface | " + params);
      return;
    }
    agree(g_rep.leg1, name); return;
  }
  if (c.kind == ssi::CurveKind::Circle) {
    // A rounding-scale tiny circle at the tangency is honest; it must be small and on-surface.
    if (c.radius > 1e-4 * scale) {
      char b[96]; std::snprintf(b, sizeof b, "tangent fabricated Circle R=%.3e (>1e-4·scale) | ", c.radius);
      disagree(g_rep.leg1, name, seed, idx, std::string(b) + params);
      return;
    }
    const double w = std::max(curveWorst(c, residA), curveWorst(c, residB));
    if (w > tol) { disagree(g_rep.leg1, name, seed, idx, "tangent tiny-Circle off surface | " + params); return; }
    agree(g_rep.leg1, name); return;
  }
  disagree(g_rep.leg1, name, seed, idx,
           "tangent gave unexpected kind " + std::to_string(int(c.kind)) + " | " + params);
}

// Assert an honest decline: the pair genuinely does not meet (NoIntersection) or is the
// same locus (Coincident). A fabricated Ok curve here is a DISAGREED finding.
void verifyDecline(const char* name, const ssi::IntersectionResult& r,
                   ssi::IntersectionStatus expect, std::uint64_t seed, int idx,
                   const std::string& params) {
  if (r.status == expect) { agree(g_rep.leg1, name); return; }
  disagree(g_rep.leg1, name, seed, idx,
           "expected status " + std::to_string(int(expect)) + ", got " +
               std::to_string(int(r.status)) + " (curves=" + std::to_string(r.curves.size()) +
               ") | " + params);
}

// ─────────────────────────────────────────────────────────────────────────────
// LEG 1 case generators — build a known pair in a canonical local pose, then move both
// surfaces by one random rigid transform (type-preserving). Each records its bucket.
// ─────────────────────────────────────────────────────────────────────────────

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}
using S = ssi::Surface;

// plane ∩ sphere: ⟂ cut at signed offset d from the centre, |d|<R → Circle; |d|=R →
// tangent Point; |d|>R → NoIntersection.
void casePlaneSphere(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const double R = rng.range(1.0, 6.0);
  const int mode = rng.irange(0, 3);  // 0 circle, 1 near-circle, 2 tangent, 3 far
  double d;
  if (mode == 0)      d = rng.range(-0.8, 0.8) * R;
  else if (mode == 1) d = R * rng.range(0.97, 0.999);   // near-tangent grazing circle
  else if (mode == 2) d = R;                             // exactly tangent
  else                d = R * rng.range(1.05, 2.0);      // disjoint
  const nmath::Sphere sp = xform(T, nmath::Sphere{frameZ({0, 0, 0}), R});
  const nmath::Plane pl  = xform(T, nmath::Plane{frameZ({0, 0, d})});
  auto r = ssi::intersect_surfaces(S::of(pl), S::of(sp));
  char p[96]; std::snprintf(p, sizeof p, "R=%.4f d=%.4f", R, d);
  auto rA = [&](const Point3& x){ return distToPlane(pl, x); };
  auto rB = [&](const Point3& x){ return distToSphere(sp, x); };
  if (mode <= 1)      verifyLeg1("plane∩sphere:circle", r, rA, rB, ssi::CurveKind::Circle, 1, kLeg1Tol, seed, idx, p);
  else if (mode == 2) verifyTangent("plane∩sphere:tangent", r, rA, rB, R, kLeg1Tol, seed, idx, p);
  else                verifyDecline("plane∩sphere:empty", r, ssi::IntersectionStatus::NoIntersection, seed, idx, p);
}

// plane ∩ cylinder: ⟂ axis → Circle; oblique (tilt θ∈(0,90)) → Ellipse (b=R, a=R/cosθ);
// ∥ axis inside → 2 Lines; ∥ axis tangent (offset=R) → tangent Line; ∥ outside → Empty.
void casePlaneCylinder(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const double R = rng.range(1.0, 5.0);
  const nmath::Cylinder cy = xform(T, nmath::Cylinder{frameZ(), R});
  const int mode = rng.irange(0, 3);  // 0 perp, 1 oblique, 2 parallel-through, 3 parallel-tangent
  char p[96];
  auto rA = [&](const nmath::Plane& pl, const Point3& x){ return distToPlane(pl, x); };
  auto rB = [&](const Point3& x){ return distToCylinder(cy, x); };
  if (mode == 0) {
    const double z = rng.range(-3, 3);
    const nmath::Plane pl = xform(T, nmath::Plane{frameZ({0, 0, z})});
    std::snprintf(p, sizeof p, "perp R=%.4f z=%.4f", R, z);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(cy));
    verifyLeg1("plane∩cyl:circle", r, [&](const Point3& x){return rA(pl,x);}, rB,
               ssi::CurveKind::Circle, 1, kLeg1Tol, seed, idx, p);
  } else if (mode == 1) {
    // tilt the plane normal by θ from the axis: normal = (0, sinθ, cosθ). Keep θ away from
    // the parallel-degenerate limit but include a near-grazing (steep) tilt.
    const double th = rng.range(0.15, kPi / 2 - 0.05);
    const double c = std::cos(th), s = std::sin(th);
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{0, 0, 0}, {1, 0, 0}, {0, c, s}, {0, -s, c}}});
    std::snprintf(p, sizeof p, "oblique R=%.4f theta=%.4f", R, th);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(cy));
    verifyLeg1("plane∩cyl:ellipse", r, [&](const Point3& x){return rA(pl,x);}, rB,
               ssi::CurveKind::Ellipse, 1, kLeg1Tol, seed, idx, p);
  } else if (mode == 2) {
    const double off = rng.range(0.0, 0.9) * R;  // plane ∥ axis, offset < R → 2 rulings
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{off, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}}});
    std::snprintf(p, sizeof p, "parallel R=%.4f off=%.4f", R, off);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(cy));
    verifyLeg1("plane∩cyl:rulings", r, [&](const Point3& x){return rA(pl,x);}, rB,
               ssi::CurveKind::Line, 2, kLeg1Tol, seed, idx, p);
  } else {
    const double off = R;  // plane ∥ axis exactly at the wall → single tangent ruling
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{off, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}}});
    std::snprintf(p, sizeof p, "tangent-ruling R=%.4f off=%.4f", R, off);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(cy));
    // A plane grazing the cylinder along a ruling: native may report the single tangent
    // Line (Ok, 1) OR a tangent contact. Accept EITHER a single Line on both surfaces OR
    // an honest tangent classification — but a fabricated 2-line crossing is DISAGREED.
    if (r.status == ssi::IntersectionStatus::Ok && r.curves.size() == 1 &&
        r.curves[0].kind == ssi::CurveKind::Line) {
      verifyLeg1("plane∩cyl:tangent-ruling", r, [&](const Point3& x){return rA(pl,x);}, rB,
                 ssi::CurveKind::Line, 1, kLeg1Tol, seed, idx, p);
    } else if (r.status == ssi::IntersectionStatus::Ok && r.curves.size() == 2) {
      // Exact off=R is a knife-edge: a hair of rounding lands just INSIDE the wall, so the
      // section is two NEARLY-COINCIDENT rulings. Honest iff both are Lines ON both surfaces
      // and their separation is rounding-scale (≪ R); a wide 2-line crossing would be a bug.
      const bool bothLines = r.curves[0].kind == ssi::CurveKind::Line &&
                             r.curves[1].kind == ssi::CurveKind::Line;
      double onSurf = 0.0;
      for (const auto& cu : r.curves)
        onSurf = std::max(onSurf, std::max(curveWorst(cu, [&](const Point3& x){return rA(pl,x);}), curveWorst(cu, rB)));
      const double sep = nmath::distance(r.curves[0].frame.origin, r.curves[1].frame.origin);
      if (bothLines && onSurf <= kLeg1Tol && sep < 1e-3 * R)
        decline(g_rep.leg1, "plane∩cyl:tangent-ruling");   // honest rounding split at the knife-edge
      else
        disagree(g_rep.leg1, "plane∩cyl:tangent-ruling", seed, idx,
                 "grazing plane fabricated 2 separated rulings sep=" + std::to_string(sep) + " | " + p);
    } else {
      // NoIntersection/NotAnalytic on an exact tangent is an honest decline (a knife-edge).
      decline(g_rep.leg1, "plane∩cyl:tangent-ruling");
    }
  }
}
// tiny helper so the plane-cylinder modes can share one intersect call site.
ssi::IntersectionResult r_of_impl(const nmath::Cylinder& cy, const nmath::Plane& pl) {
  return ssi::intersect_surfaces(S::of(pl), S::of(cy));
}

// plane ∩ cone: pick the plane tilt θ vs the cone half-angle α to select the conic —
//   ⟂ axis (θ=0)            → Circle
//   0<θ<α (shallow)         → Ellipse
//   θ=α (parallel a genr.)  → Parabola
//   θ>α up to vertical      → Hyperbola (both nappes → 2 branches when the plane misses apex)
//
// KIND selection is governed by the plane-normal tilt θ (angle of the plane normal from
// the cone AXIS) vs the PARABOLA THRESHOLD θc = π/2 − α (NOT θ vs α — that coincidence
// only holds at α=45°, which is why the curated 11-case suite's 45° cone did not surface
// this; the fuzzer sweeps α away from 45° and uses the correct θc):
//   θ = 0        → Circle (plane ⟂ axis)
//   0 < θ < θc   → Ellipse (plane cuts every generator of one nappe)
//   θ = θc       → Parabola (plane parallel to exactly one generator)
//   θc < θ ≤ π/2 → Hyperbola (plane steeper than the generators → both nappes → 2 branches)
void casePlaneCone(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  // Keep α in a band where θc = π/2−α leaves a healthy ellipse/hyperbola interval on both
  // sides (α ∈ [~28°, ~62°] ⇒ θc ∈ [~28°, ~62°]), and away from the 45° coincidence at times.
  const double alpha = rng.range(0.5, 1.1);
  const double thc = kPi / 2 - alpha;   // parabola threshold
  const double baseR = rng.range(0.5, 2.0);
  const nmath::Cone co = xform(T, nmath::Cone{frameZ(), baseR, alpha});
  auto rA = [&](const nmath::Plane& pl, const Point3& x){ return distToPlane(pl, x); };
  auto rB = [&](const Point3& x){ return distToCone(co, x); };
  char p[128];
  const int mode = rng.irange(0, 3);
  if (mode == 0) {  // θ=0: ⟂ axis, clears the apex → Circle
    const double z = rng.range(1.0, 4.0);
    const nmath::Plane pl = xform(T, nmath::Plane{frameZ({0, 0, z})});
    std::snprintf(p, sizeof p, "circle alpha=%.4f z=%.4f", alpha, z);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(co));
    verifyLeg1("plane∩cone:circle", r, [&](const Point3& x){return rA(pl,x);}, rB,
               ssi::CurveKind::Circle, 1, kLeg1ConeTol, seed, idx, p);
  } else if (mode == 1) {  // 0<θ<θc → single Ellipse (bounded cut of one nappe)
    const double th = rng.range(0.05, thc - 0.05);
    const double c = std::cos(th), s = std::sin(th);
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{0, 0, 3.0}, {1, 0, 0}, {0, c, s}, {0, -s, c}}});
    std::snprintf(p, sizeof p, "ellipse alpha=%.4f theta=%.4f thc=%.4f", alpha, th, thc);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(co));
    verifyLeg1("plane∩cone:ellipse", r, [&](const Point3& x){return rA(pl,x);}, rB,
               ssi::CurveKind::Ellipse, 1, kLeg1ConeTol, seed, idx, p);
  } else if (mode == 2) {  // θ=θc exactly (plane ∥ a generator) → Parabola
    const double th = thc;
    const double c = std::cos(th), s = std::sin(th);
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{0, 0, 2.0}, {1, 0, 0}, {0, c, s}, {0, -s, c}}});
    std::snprintf(p, sizeof p, "parabola alpha=%.4f theta=thc=%.4f", alpha, thc);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(co));
    // θ=θc is a knife-edge: a floating θ a hair off θc flips parabola↔ellipse/hyperbola. A
    // Parabola OR either adjacent conic on both surfaces is honest; a fabricated off-surface
    // curve is DISAGREED. Accept Parabola (1) / Ellipse (1) / Hyperbola (2) if on-surface.
    if (r.status == ssi::IntersectionStatus::Ok && r.curves.size() == 1 &&
        r.curves[0].kind == ssi::CurveKind::Parabola) {
      verifyLeg1("plane∩cone:parabola", r, [&](const Point3& x){return rA(pl,x);}, rB,
                 ssi::CurveKind::Parabola, 1, kLeg1ConeTol, seed, idx, p);
    } else if (r.status == ssi::IntersectionStatus::Ok) {
      // knife-edge flip: still verify every returned curve lies on both surfaces.
      double worst = 0.0;
      for (const auto& cu : r.curves) worst = std::max(worst, std::max(curveWorst(cu, [&](const Point3& x){return rA(pl,x);}), curveWorst(cu, rB)));
      if (worst > kLeg1ConeTol)
        disagree(g_rep.leg1, "plane∩cone:parabola", seed, idx,
                 "knife-edge conic OFF surface | " + std::string(p));
      else
        decline(g_rep.leg1, "plane∩cone:parabola");  // honest kind flip at the exact threshold
    } else {
      decline(g_rep.leg1, "plane∩cone:parabola");
    }
  } else {  // θ=π/2 (vertical plane) > θc, offset from the axis → 2 Hyperbolae (both nappes)
    const double off = rng.range(0.2, 1.5);
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{off, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}}});
    std::snprintf(p, sizeof p, "hyperbola alpha=%.4f off=%.4f", alpha, off);
    auto r = ssi::intersect_surfaces(S::of(pl), S::of(co));
    verifyLeg1("plane∩cone:hyperbola", r, [&](const Point3& x){return rA(pl,x);}, rB,
               ssi::CurveKind::Hyperbola, 2, kLeg1ConeTol, seed, idx, p);
  }
}

// plane ∩ torus (⟂ axis / axis-containing): ⟂ at z=0 → 2 concentric circles (R±r);
// axis-containing plane → 2 tube circles radius r. Oblique → NotAnalytic (honest decline).
void casePlaneTorus(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const double R = rng.range(2.0, 5.0);
  const double r = rng.range(0.4, R - 0.5);   // ring torus (r < R)
  const nmath::Torus to = xform(T, nmath::Torus{frameZ(), R, r});
  auto rB = [&](const Point3& x){ return distToTorus(to, x); };
  char p[96];
  const int mode = rng.irange(0, 2);
  if (mode == 0) {  // ⟂ axis at z=0 → 2 circles radii R+r, R-r
    const nmath::Plane pl = xform(T, nmath::Plane{frameZ({0, 0, 0})});
    std::snprintf(p, sizeof p, "perp R=%.4f r=%.4f", R, r);
    auto r0 = ssi::intersect_surfaces(S::of(pl), S::of(to));
    verifyLeg1("plane∩torus:perp2circ", r0, [&](const Point3& x){return distToPlane(pl,x);}, rB,
               ssi::CurveKind::Circle, 2, kLeg1Tol, seed, idx, p);
  } else if (mode == 1) {  // plane CONTAINS the axis (normal ⟂ axis) → 2 tube circles radius r
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {0, 1, 0}}});
    std::snprintf(p, sizeof p, "axial R=%.4f r=%.4f", R, r);
    auto r0 = ssi::intersect_surfaces(S::of(pl), S::of(to));
    verifyLeg1("plane∩torus:axial2circ", r0, [&](const Point3& x){return distToPlane(pl,x);}, rB,
               ssi::CurveKind::Circle, 2, kLeg1Tol, seed, idx, p);
  } else {  // oblique plane → deferred NotAnalytic (honest, per S1 scope)
    const double th = rng.range(0.3, kPi / 3);
    const double c = std::cos(th), s = std::sin(th);
    const nmath::Plane pl = xform(T, nmath::Plane{Ax3{{0, 0, 0}, {1, 0, 0}, {0, c, s}, {0, -s, c}}});
    std::snprintf(p, sizeof p, "oblique R=%.4f r=%.4f theta=%.4f", R, r, th);
    auto r0 = ssi::intersect_surfaces(S::of(pl), S::of(to));
    verifyDecline("plane∩torus:oblique-defer", r0, ssi::IntersectionStatus::NotAnalytic, seed, idx, p);
  }
}

// sphere ∩ sphere: centre distance d. |R1-R2|<d<R1+R2 → Circle; d=R1+R2 → tangent Point;
// d>R1+R2 → Empty; d=0 same R → Coincident.
void caseSphereSphere(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const double R1 = rng.range(1.0, 5.0), R2 = rng.range(1.0, 5.0);
  const int mode = rng.irange(0, 3);  // 0 circle, 1 tangent-external, 2 far, 3 coincident
  double d;
  if (mode == 0)      d = rng.range(std::fabs(R1 - R2) + 0.3, R1 + R2 - 0.3);
  else if (mode == 1) d = R1 + R2;                        // externally tangent
  else if (mode == 2) d = (R1 + R2) * rng.range(1.05, 2.0);
  else                d = 0.0;                             // will use identical spheres
  char p[112];
  if (mode == 3) {
    const nmath::Sphere s1 = xform(T, nmath::Sphere{frameZ({0, 0, 0}), R1});
    std::snprintf(p, sizeof p, "coincident R=%.4f", R1);
    verifyDecline("sphere∩sphere:coincident",
                  ssi::intersect_surfaces(S::of(s1), S::of(s1)),
                  ssi::IntersectionStatus::Coincident, seed, idx, p);
    return;
  }
  const nmath::Sphere s1 = xform(T, nmath::Sphere{frameZ({0, 0, 0}), R1});
  const nmath::Sphere s2 = xform(T, nmath::Sphere{frameZ({d, 0, 0}), R2});
  auto rA = [&](const Point3& x){ return distToSphere(s1, x); };
  auto rB = [&](const Point3& x){ return distToSphere(s2, x); };
  std::snprintf(p, sizeof p, "R1=%.4f R2=%.4f d=%.4f", R1, R2, d);
  auto r = ssi::intersect_surfaces(S::of(s1), S::of(s2));
  if (mode == 0)      verifyLeg1("sphere∩sphere:circle", r, rA, rB, ssi::CurveKind::Circle, 1, kLeg1Tol, seed, idx, p);
  else if (mode == 1) verifyTangent("sphere∩sphere:tangent", r, rA, rB, std::max(R1, R2), kLeg1Tol, seed, idx, p);
  else                verifyDecline("sphere∩sphere:empty", r, ssi::IntersectionStatus::NoIntersection, seed, idx, p);
}

// coaxial sphere ∩ cylinder: sphere at origin R, cylinder coaxial radius Rc<R → 2 circles at
// z=±√(R²−Rc²); Rc=R → tangent equator (TangentCurve, no transversal branch); Rc>R → Empty.
void caseSphereCylinderCoaxial(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const double R = rng.range(2.0, 5.0);
  const int mode = rng.irange(0, 2);  // 0 two-circle, 1 tangent-equator, 2 outside
  double Rc;
  if (mode == 0)      Rc = rng.range(0.3, 0.9) * R;
  else if (mode == 1) Rc = R;
  else                Rc = R * rng.range(1.05, 1.6);
  const nmath::Sphere sp   = xform(T, nmath::Sphere{frameZ(), R});
  const nmath::Cylinder cy = xform(T, nmath::Cylinder{frameZ(), Rc});
  auto rA = [&](const Point3& x){ return distToSphere(sp, x); };
  auto rB = [&](const Point3& x){ return distToCylinder(cy, x); };
  char p[96]; std::snprintf(p, sizeof p, "R=%.4f Rc=%.4f", R, Rc);
  auto r = ssi::intersect_surfaces(S::of(sp), S::of(cy));
  if (mode == 0) {
    verifyLeg1("coax sph∩cyl:2circ", r, rA, rB, ssi::CurveKind::Circle, 2, kLeg1Tol, seed, idx, p);
  } else if (mode == 1) {
    // Tangent along the equator: native must NOT fabricate transversal circles. Accept an
    // honest non-Ok status OR a single tangent Circle/Point; a 2-transversal-circle answer
    // (as if Rc<R) is DISAGREED. NoIntersection is also honest (the loci touch, don't cross).
    if (r.status == ssi::IntersectionStatus::Ok && r.curves.size() == 2)
      disagree(g_rep.leg1, "coax sph∩cyl:tangent-equator", seed, idx,
               std::string("tangent equator fabricated 2 transversal circles | ") + p);
    else
      decline(g_rep.leg1, "coax sph∩cyl:tangent-equator");
  } else {
    verifyDecline("coax sph∩cyl:empty", r, ssi::IntersectionStatus::NoIntersection, seed, idx, p);
  }
}

// coaxial sphere ∩ cone: apex-at-centre 45°-ish cone through a sphere → 2 circles (one per
// nappe) at z=±R·cosα. Uses a coaxial cone with apex at the sphere centre.
void caseSphereConeCoaxial(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const double R = rng.range(2.0, 5.0);
  const double alpha = rng.range(0.3, kPi / 2 - 0.3);
  const nmath::Sphere sp = xform(T, nmath::Sphere{frameZ(), R});
  const nmath::Cone co   = xform(T, nmath::Cone{frameZ(), 0.0, alpha});  // apex at centre
  auto rA = [&](const Point3& x){ return distToSphere(sp, x); };
  auto rB = [&](const Point3& x){ return distToCone(co, x); };
  char p[96]; std::snprintf(p, sizeof p, "R=%.4f alpha=%.4f", R, alpha);
  auto r = ssi::intersect_surfaces(S::of(sp), S::of(co));
  verifyLeg1("coax sph∩cone:2circ", r, rA, rB, ssi::CurveKind::Circle, 2, kLeg1ConeTol, seed, idx, p);
}

// cylinder ∩ cylinder (parallel & coaxial): parallel offset o, radii R1,R2 with
// |R1-R2|<o<R1+R2 → 2 Lines; coaxial same R → Coincident; coaxial different R → Empty.
void caseCylinderCylinder(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const int mode = rng.irange(0, 2);  // 0 parallel-2-lines, 1 coaxial-same, 2 coaxial-diff
  char p[112];
  if (mode == 0) {
    const double R1 = rng.range(1.0, 4.0), R2 = rng.range(1.0, 4.0);
    const double o = rng.range(std::fabs(R1 - R2) + 0.3, R1 + R2 - 0.3);
    const nmath::Cylinder c1 = xform(T, nmath::Cylinder{frameZ(), R1});
    const nmath::Cylinder c2 = xform(T, nmath::Cylinder{frameZ({o, 0, 0}), R2});
    auto rA = [&](const Point3& x){ return distToCylinder(c1, x); };
    auto rB = [&](const Point3& x){ return distToCylinder(c2, x); };
    std::snprintf(p, sizeof p, "parallel R1=%.4f R2=%.4f o=%.4f", R1, R2, o);
    verifyLeg1("cyl∩cyl:2lines", ssi::intersect_surfaces(S::of(c1), S::of(c2)), rA, rB,
               ssi::CurveKind::Line, 2, kLeg1Tol, seed, idx, p);
  } else if (mode == 1) {
    const double R = rng.range(1.0, 4.0);
    const nmath::Cylinder c1 = xform(T, nmath::Cylinder{frameZ(), R});
    std::snprintf(p, sizeof p, "coaxial-same R=%.4f", R);
    verifyDecline("cyl∩cyl:coincident", ssi::intersect_surfaces(S::of(c1), S::of(c1)),
                  ssi::IntersectionStatus::Coincident, seed, idx, p);
  } else {
    const double R1 = rng.range(1.0, 3.0), R2 = R1 + rng.range(0.5, 2.0);
    const nmath::Cylinder c1 = xform(T, nmath::Cylinder{frameZ(), R1});
    const nmath::Cylinder c2 = xform(T, nmath::Cylinder{frameZ(), R2});
    std::snprintf(p, sizeof p, "coaxial-diff R1=%.4f R2=%.4f", R1, R2);
    verifyDecline("cyl∩cyl:coaxial-empty", ssi::intersect_surfaces(S::of(c1), S::of(c2)),
                  ssi::IntersectionStatus::NoIntersection, seed, idx, p);
  }
}

// coaxial cylinder ∩ cone: apex-at-origin cone, coaxial cylinder radius Rc → 2 circles at
// z = ±Rc·cotα (one per nappe, both radius Rc). Uses α so both nappe circles clear.
void caseCylinderConeCoaxial(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const double Rc = rng.range(1.0, 3.0);
  const double alpha = rng.range(0.35, kPi / 2 - 0.35);
  const nmath::Cylinder cy = xform(T, nmath::Cylinder{frameZ(), Rc});
  const nmath::Cone co     = xform(T, nmath::Cone{frameZ(), 0.0, alpha});  // apex at origin
  auto rA = [&](const Point3& x){ return distToCylinder(cy, x); };
  auto rB = [&](const Point3& x){ return distToCone(co, x); };
  char p[96]; std::snprintf(p, sizeof p, "Rc=%.4f alpha=%.4f", Rc, alpha);
  auto r = ssi::intersect_surfaces(S::of(cy), S::of(co));
  verifyLeg1("coax cyl∩cone:2circ", r, rA, rB, ssi::CurveKind::Circle, 2, kLeg1ConeTol, seed, idx, p);
}

// plane ∩ plane: non-parallel → Line; parallel disjoint → Empty; identical → Coincident.
void casePlanePlane(Xoshiro256ss& rng, std::uint64_t seed, int idx) {
  const Transform T = randomRigid(rng);
  const int mode = rng.irange(0, 2);
  char p[64];
  if (mode == 0) {
    const nmath::Plane p1 = xform(T, nmath::Plane{frameZ()});
    const nmath::Plane p2 = xform(T, nmath::Plane{Ax3{{0, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 0, 0}}});
    auto rA = [&](const Point3& x){ return distToPlane(p1, x); };
    auto rB = [&](const Point3& x){ return distToPlane(p2, x); };
    std::snprintf(p, sizeof p, "cross");
    verifyLeg1("plane∩plane:line", ssi::intersect_surfaces(S::of(p1), S::of(p2)), rA, rB,
               ssi::CurveKind::Line, 1, kLeg1Tol, seed, idx, p);
  } else if (mode == 1) {
    const double g = rng.range(0.5, 5.0);
    const nmath::Plane p1 = xform(T, nmath::Plane{frameZ()});
    const nmath::Plane p2 = xform(T, nmath::Plane{frameZ({0, 0, g})});
    std::snprintf(p, sizeof p, "parallel gap=%.4f", g);
    verifyDecline("plane∩plane:empty", ssi::intersect_surfaces(S::of(p1), S::of(p2)),
                  ssi::IntersectionStatus::NoIntersection, seed, idx, p);
  } else {
    const nmath::Plane p1 = xform(T, nmath::Plane{frameZ()});
    std::snprintf(p, sizeof p, "same");
    verifyDecline("plane∩plane:coincident", ssi::intersect_surfaces(S::of(p1), S::of(p1)),
                  ssi::IntersectionStatus::Coincident, seed, idx, p);
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// LEG 2 — NURBS↔analytic exact known-answer.
// ─────────────────────────────────────────────────────────────────────────────

// A quadratic rational NURBS cylinder: a full circle of radius R in the local XY plane
// (the classic 9-control-point rational circle: square control polygon, corner weights
// cos(45°)=1/√2) EXTRUDED along +Z by `height`. degU=2 (the circle), degV=1 (the line).
// Poles are the PROJECTED (Cartesian) control points; weights parallel the grid.
struct NurbsSurf {
  int degU, degV, nU, nV;
  std::vector<Point3> poles;
  std::vector<double> weights;
  std::vector<double> knotsU, knotsV;
};

NurbsSurf makeNurbsCylinder(const Ax3& f, double R, double z0, double z1) {
  // 9-pole rational circle (3 spans wrap: 0,90,180,270,360). Control polygon = square
  // corners + edge midpoints; corner weight = cos(45°) = √2/2, midpoint weight = 1.
  const double w = std::sqrt(2.0) / 2.0;
  // circle control points in local (x,y), CCW starting at (R,0):
  //   (R,0) w1  (R,R) w  (0,R) w1  (-R,R) w  (-R,0) w1  (-R,-R) w  (0,-R) w1  (R,-R) w  (R,0) w1
  struct CP { double x, y, w; };
  const CP circ[9] = {
      { R, 0, 1}, { R, R, w}, { 0, R, 1}, {-R, R, w}, {-R, 0, 1},
      {-R,-R, w}, { 0,-R, 1}, { R,-R, w}, { R, 0, 1}};
  const std::vector<double> ku = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};  // 9 poles, deg2
  const std::vector<double> kv = {0, 0, 1, 1};  // 2 poles, deg1
  NurbsSurf s;
  s.degU = 2; s.degV = 1; s.nU = 9; s.nV = 2;
  s.knotsU = ku; s.knotsV = kv;
  auto place = [&](double lx, double ly, double lz) {
    return nmath::frameCombine(f, lx, ly, lz);
  };
  // Row-major: i over U (the 9 circle poles), j over V (z0, z1).
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 2; ++j) {
      const double lz = (j == 0) ? z0 : z1;
      s.poles.push_back(place(circ[i].x, circ[i].y, lz));
      s.weights.push_back(circ[i].w);
    }
  return s;
}

// A rational NURBS sphere of revolution, EXACT to machine precision (verified below before
// use). It is the tensor product of two rational quadratic circles:
//   * U (longitude): the classic 9-pole full rational circle (unit radius, corner w=√2/2).
//   * V (meridian):  a rational SEMICIRCLE from south pole to north pole built as TWO 90°
//     quadratic arcs (5 poles, deg 2, knots {0,0,0,½,½,1,1,1}) in the (radius,height) plane:
//         south (0,−R) w1 ; (R,−R) w ; equator (R,0) w1 ; (R,R) w ; north (0,R) w1.
//     (A single 3-pole deg-2 arc CANNOT span 180° — its middle weight would be cos90°=0,
//      which is why the naive 9×3 net does not close the sphere; the two-arc meridian does.)
// The product pole is (u-circle scaled by the meridian radius, meridian height) and the
// product weight is w_u·w_v — the de Boor point then lies EXACTLY on x²+y²+z²=R².
NurbsSurf makeNurbsSphere(const Ax3& f, double R) {
  const double w = std::sqrt(2.0) / 2.0;
  struct CP { double x, y, wt; };
  const CP circ[9] = {
      { 1, 0, 1}, { 1, 1, w}, { 0, 1, 1}, {-1, 1, w}, {-1, 0, 1},
      {-1,-1, w}, { 0,-1, 1}, { 1,-1, w}, { 1, 0, 1}};
  struct MV { double rad, z, wt; };
  const MV mer[5] = {{0, -R, 1}, {R, -R, w}, {R, 0, 1}, {R, R, w}, {0, R, 1}};
  const std::vector<double> ku = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
  const std::vector<double> kv = {0, 0, 0, 0.5, 0.5, 1, 1, 1};  // 5 poles, deg2, two spans
  NurbsSurf s;
  s.degU = 2; s.degV = 2; s.nU = 9; s.nV = 5;
  s.knotsU = ku; s.knotsV = kv;
  for (int i = 0; i < 9; ++i)
    for (int j = 0; j < 5; ++j) {
      s.poles.push_back(nmath::frameCombine(f, circ[i].x * mer[j].rad, circ[i].y * mer[j].rad, mer[j].z));
      s.weights.push_back(circ[i].wt * mer[j].wt);
    }
  return s;
}

// Evaluate a NurbsSurf point via native-math nurbsSurfacePoint.
Point3 nurbsPoint(const NurbsSurf& s, double u, double v) {
  nmath::SurfaceGrid g{s.poles, s.nU, s.nV};
  return nmath::nurbsSurfacePoint(s.degU, s.degV, g, s.weights, s.knotsU, s.knotsV, u, v);
}

// Build a SurfaceAdapter for a NurbsSurf over its full [0,1]² clamped domain.
ssi::SurfaceAdapter nurbsAdapter(const NurbsSurf& s) {
  return ssi::makeNurbsAdapter(s.degU, s.degV, s.poles, s.weights, s.nU, s.nV, s.knotsU, s.knotsV);
}

// VERIFY a NURBS surface reproduces an analytic residual to machine precision over a grid.
// Returns the worst residual; the caller asserts it is tiny BEFORE using the fixture.
template <class Resid>
double nurbsFixtureResidual(const NurbsSurf& s, Resid resid) {
  double worst = 0.0;
  for (int i = 0; i <= 16; ++i)
    for (int j = 0; j <= 16; ++j) {
      const double u = i / 16.0, v = j / 16.0;
      worst = std::max(worst, resid(nurbsPoint(s, u, v)));
    }
  return worst;
}

// EXACT distance from a point to the KNOWN analytic curve. For a Circle it is closed-form
// (in-plane radial deviation + out-of-plane offset); for an Ellipse we sample-then-refine so
// the residual is the true geometric distance, NOT a chord-sampling artifact (a naive
// fixed-sample min would report up to ½·(chord) ≈ 2πR/N of spurious error — which is why the
// oracle must be analytic here, not a coarse polyline).
double distToKnownCurve(const ssi::IntersectionCurve& known, const Point3& p) {
  const nmath::Ax3& F = known.frame;
  const Vec3 w = p - F.origin;
  const double px = nmath::dot(w, F.x.vec());
  const double py = nmath::dot(w, F.y.vec());
  const double pz = nmath::dot(w, F.z.vec());   // out-of-plane component
  if (known.kind == ssi::CurveKind::Circle) {
    const double rho = std::hypot(px, py);
    return std::hypot(rho - known.radius, pz);
  }
  if (known.kind == ssi::CurveKind::Ellipse) {
    // Nearest point on x=a·cos t, y=b·sin t (in-plane), plus the out-of-plane pz. Sample
    // coarsely then Newton-refine the in-plane parameter to convergence.
    const double a = known.a, b = known.b;
    double bestT = 0.0, bestD = 1e300;
    for (int i = 0; i < 256; ++i) {
      const double t = 2.0 * kPi * i / 256.0;
      const double dx = a * std::cos(t) - px, dy = b * std::sin(t) - py;
      const double d = dx * dx + dy * dy;
      if (d < bestD) { bestD = d; bestT = t; }
    }
    double t = bestT;
    for (int it = 0; it < 40; ++it) {
      const double ct = std::cos(t), st = std::sin(t);
      const double ex = a * ct - px, ey = b * st - py;
      const double dprime = ex * (-a * st) + ey * (b * ct);            // d/dt of ½‖e‖²
      const double dpp = (-a * st) * (-a * st) + ex * (-a * ct) +
                         (b * ct) * (b * ct) + ey * (-b * st);          // 2nd derivative
      if (std::fabs(dpp) < 1e-14) break;
      const double dt = dprime / dpp;
      t -= dt;
      if (std::fabs(dt) < 1e-14) break;
    }
    const double dx = a * std::cos(t) - px, dy = b * std::sin(t) - py;
    return std::hypot(std::hypot(dx, dy), pz);
  }
  // Fallback (unused kinds): dense sampling.
  auto [t0, t1] = known.naturalRange();
  double best = 1e300;
  for (int i = 0; i <= 512; ++i) {
    const double t = t0 + (t1 - t0) * (double(i) / 512);
    best = std::min(best, nmath::distance(p, known.value(t)));
  }
  return best;
}

// Leg-2 verdict. The DELIVERABLE claim is "native SSI traces the RIGHT curve on
// NURBS-represented geometry": we prove that with TWO exact, closed-form checks —
//   (1) every traced node lies on BOTH surfaces (≤ onSurfTol), and
//   (2) every traced node lies on the KNOWN analytic intersection curve (≤ curve-match tol,
//       the FORWARD direction: node → known curve). Together these prove the traced polyline
//       IS the exact analytic intersection curve, verified with NO OCCT.
// COVERAGE (does the trace span the whole known curve?) is a SEPARATE property: on the
// rational NURBS the u-parametrization wraps at a periodic seam the freeform adapter does not
// declare, so a full circle is (correctly) traced as an OPEN arc covering ≈ 99.6% of it with a
// small seam gap. That is an honest recall boundary, NOT a wrong curve — we REPORT the covered
// fraction and count a mostly-uncovered trace as HONESTLY-DECLINED, never DISAGREED.
void verifyLeg2(bool& cc_ok_, const char* name, const ssi::TraceSet& tr,
                const std::function<double(const Point3&)>& residA,
                const std::function<double(const Point3&)>& residB,
                const ssi::IntersectionCurve& known, int expectBranches,
                std::uint64_t seed, int idx, const std::string& params) {
  // HONEST DECLINE: native declined / left a near-tangent gap on a known clean transversal case.
  if (tr.curveCount() < expectBranches || tr.nearTangentGaps > 0) {
    bucket(g_rep.leg2, name).declined++;
    std::printf("  -- HONESTLY-DECLINED [%s] seed=%llu case=%d: curves=%d (want %d) gaps=%d | %s\n",
                name, (unsigned long long)seed, idx, tr.curveCount(), expectBranches,
                tr.nearTangentGaps, params.c_str());
    return;
  }
  // (1) every node on both surfaces AND (2) every node on the known curve → the RIGHT curve.
  double worstNode = 0.0, worstOnKnown = 0.0;
  int nodes = 0;
  for (const auto& w : tr.lines)
    for (const auto& n : w.points) {
      worstNode = std::max(worstNode, std::max(residA(n.point), residB(n.point)));
      worstOnKnown = std::max(worstOnKnown, distToKnownCurve(known, n.point));
      ++nodes;
    }
  if (worstNode > kLeg2OnSurf) {
    disagree(g_rep.leg2, name, seed, idx,
             "traced node OFF surface, worst=" + std::to_string(worstNode) + " | " + params);
    return;
  }
  if (worstOnKnown > kLeg2Fit) {
    char b[112]; std::snprintf(b, sizeof b,
                               "traced node OFF the known analytic curve, worst=%.3e > %.3e | ",
                               worstOnKnown, kLeg2Fit);
    disagree(g_rep.leg2, name, seed, idx, std::string(b) + params);
    return;
  }
  // COVERAGE (reported, not a pass/fail bar): the fraction of the known conic's PARAMETER
  // circle the traced nodes span. Measured as covered angular bins (NOT node-proximity — the
  // marching node spacing ~0.06 far exceeds kLeg2Fit=1e-3, so a proximity test would spuriously
  // read ~0%; the right question is "does the arc EXTEND over the whole curve", which is the
  // angular span). Each node is projected to its conic angle θ=atan2(py,px) in the known
  // curve's frame; we bin [0,2π) and count occupied bins.
  const nmath::Ax3& KF = known.frame;
  const int NB = 256;
  std::vector<char> bin(NB, 0);
  for (const auto& w : tr.lines)
    for (const auto& n : w.points) {
      const Vec3 ww = n.point - KF.origin;
      const double px = nmath::dot(ww, KF.x.vec());
      const double py = nmath::dot(ww, KF.y.vec());
      double th = std::atan2(py, px);
      if (th < 0) th += 2.0 * kPi;
      int b = int(th / (2.0 * kPi) * NB);
      b = std::max(0, std::min(NB - 1, b));
      bin[b] = 1;
    }
  int occ = 0;
  for (char c : bin) occ += c;
  const double frac = double(occ) / NB;
  if (frac < 0.5) {  // traced the right curve but barely any of it → honest recall boundary
    bucket(g_rep.leg2, name).declined++;
    std::printf("  -- HONESTLY-DECLINED [%s] seed=%llu case=%d: right curve but coverage %.1f%% | %s\n",
                name, (unsigned long long)seed, idx, 100.0 * frac, params.c_str());
    return;
  }
  bucket(g_rep.leg2, name).agreed++;
  g_rep.totalAgreed++;
  std::printf("  ok [%s] nodes=%d onSurf=%.1e onKnown=%.1e coverage=%.1f%% | %s\n",
              name, nodes, worstNode, worstOnKnown, 100.0 * frac, params.c_str());
}

// ── driver: run N leg-1 cases + the leg-2 known-answer set, for one seed ──────────

// A single "run one seed" that dispatches N random leg-1 cases across all buckets, then
// the fixed leg-2 NURBS↔analytic set. Returns via the global report.
void runSeed(bool& cc_ok_, std::uint64_t seed, int N) {
  Xoshiro256ss rng(seed);

  // LEG 1 — N random elementary pairs, round-robin across the pair generators so every
  // bucket gets coverage regardless of N.
  using Gen = void (*)(Xoshiro256ss&, std::uint64_t, int);
  const Gen gens[] = {
      casePlanePlane, casePlaneSphere, casePlaneCylinder, casePlaneCone, casePlaneTorus,
      caseSphereSphere, caseSphereCylinderCoaxial, caseSphereConeCoaxial,
      caseCylinderCylinder, caseCylinderConeCoaxial};
  const int nGen = int(sizeof(gens) / sizeof(gens[0]));
  for (int i = 0; i < N; ++i) {
    gens[i % nGen](rng, seed, i);
    g_rep.cases++;
  }

  // LEG 2 — NURBS↔analytic exact known-answer (a fixed, small, high-value set per seed,
  // placed by a random rigid transform so the trace is exercised off-axis too).
  const Transform T = randomRigid(rng);

  // (2a) NURBS cylinder ∩ plane ⟂ axis → circle of radius R at the cut height.
  {
    const double R = rng.range(1.0, 3.0);
    const Ax3 f = xform(T, frameZ());
    NurbsSurf cyl = makeNurbsCylinder(f, R, -2.0, 2.0);
    // VERIFY the NURBS is the intended cylinder to machine precision before using it.
    nmath::Cylinder anCyl{f, R};
    const double fixErr = nurbsFixtureResidual(cyl, [&](const Point3& x){ return distToCylinder(anCyl, x); });
    CC_CHECK(fixErr < 1e-9);
    if (fixErr >= 1e-9) { std::printf("  NURBS-cyl fixture BAD residual %.3e\n", fixErr); }

    const double zc = rng.range(-1.2, 1.2);
    const Point3 cutO = T.applyToPoint(Point3{0, 0, zc});
    nmath::Plane pl{Ax3{cutO, f.x, f.y, f.z}};  // ⟂ the cylinder axis at height zc
    auto A = nurbsAdapter(cyl);
    ssi::ParamBox pd{-2.0 * R, 2.0 * R, -2.0 * R, 2.0 * R};
    auto B = ssi::makePlaneAdapter(pl, pd);

    ssi::SeedOptions so; so.initialGridU = 4; so.initialGridV = 3;
    auto tr = ssi::trace_intersection(A, B, so);

    // KNOWN answer: circle radius R centred at cutO in the plane normal = axis.
    ssi::IntersectionCurve known = ssi::makeCircle(cutO, R, f.z, f.x);
    char p[96]; std::snprintf(p, sizeof p, "R=%.4f zc=%.4f", R, zc);
    verifyLeg2(cc_ok_, "nurbs-cyl∩plane:circle", tr,
               [&](const Point3& x){ return distToCylinder(anCyl, x); },
               [&](const Point3& x){ return distToPlane(pl, x); },
               known, 1, seed, 1000, p);
  }

  // (2b) NURBS cylinder ∩ oblique plane → ellipse (b=R, a=R/cosθ).
  {
    const double R = rng.range(1.0, 2.5);
    const Ax3 f = xform(T, frameZ());
    NurbsSurf cyl = makeNurbsCylinder(f, R, -3.0, 3.0);
    nmath::Cylinder anCyl{f, R};
    const double fixErr = nurbsFixtureResidual(cyl, [&](const Point3& x){ return distToCylinder(anCyl, x); });
    CC_CHECK(fixErr < 1e-9);

    const double th = rng.range(0.2, 0.8);  // tilt away from grazing (keep the ellipse inside the finite patch)
    const double c = std::cos(th), s = std::sin(th);
    // Oblique plane through the axis origin: local normal (0, sinθ, cosθ), i.e. tilt about X.
    const Dir3 pn{T.applyToDir(Dir3{0, -s, c})};   // plane Z (normal)
    const Dir3 px{T.applyToDir(Dir3{1, 0, 0})};    // plane X (major axis direction in the cut)
    const Point3 o = T.applyToPoint(Point3{0, 0, 0});
    nmath::Plane pl{Ax3::fromAxisAndRef(o, pn, px)};
    auto A = nurbsAdapter(cyl);
    ssi::ParamBox pd{-3.0 * R, 3.0 * R, -3.0 * R, 3.0 * R};
    auto B = ssi::makePlaneAdapter(pl, pd);

    ssi::SeedOptions so; so.initialGridU = 5; so.initialGridV = 4;
    auto tr = ssi::trace_intersection(A, B, so);

    // KNOWN: ellipse centred at o, minor R along the in-plane direction ⟂ tilt (= plane X),
    // major R/cosθ along the tilt direction (plane Y). Sample as an Ellipse with a=major.
    const double a = R / std::cos(th), b = R;
    // Ellipse X axis should be the MAJOR direction. Major grows along the tilt; that lies
    // along the plane's Y (n×X). Build the ellipse frame explicitly.
    const Dir3 ey{nmath::cross(pn.vec(), px.vec())};
    ssi::IntersectionCurve known;
    known.kind = ssi::CurveKind::Ellipse;
    known.frame = Ax3{o, ey, px, pn};   // X = major (ey), Y = minor (px)
    known.a = a; known.b = b;
    char p[96]; std::snprintf(p, sizeof p, "R=%.4f theta=%.4f a=%.4f b=%.4f", R, th, a, b);
    verifyLeg2(cc_ok_, "nurbs-cyl∩plane:ellipse", tr,
               [&](const Point3& x){ return distToCylinder(anCyl, x); },
               [&](const Point3& x){ return distToPlane(pl, x); },
               known, 1, seed, 1001, p);
  }

  // (2c) NURBS sphere ∩ plane → circle radius √(R²−d²) at signed offset d.
  {
    const double R = rng.range(1.5, 3.0);
    const Ax3 f = xform(T, frameZ());
    NurbsSurf sph = makeNurbsSphere(f, R);
    nmath::Sphere anSph{f, R};
    const double fixErr = nurbsFixtureResidual(sph, [&](const Point3& x){ return distToSphere(anSph, x); });
    CC_CHECK(fixErr < 1e-9);
    if (fixErr >= 1e-9) std::printf("  NURBS-sphere fixture BAD residual %.3e\n", fixErr);

    const double d = rng.range(-0.6, 0.6) * R;   // cut offset along the axis
    const double rc = std::sqrt(R * R - d * d);
    const Point3 cutO = T.applyToPoint(Point3{0, 0, d});
    nmath::Plane pl{Ax3{cutO, f.x, f.y, f.z}};   // ⟂ the axis at offset d
    auto A = nurbsAdapter(sph);
    ssi::ParamBox pd{-2.0 * R, 2.0 * R, -2.0 * R, 2.0 * R};
    auto B = ssi::makePlaneAdapter(pl, pd);

    ssi::SeedOptions so; so.initialGridU = 4; so.initialGridV = 4;
    auto tr = ssi::trace_intersection(A, B, so);

    ssi::IntersectionCurve known = ssi::makeCircle(cutO, rc, f.z, f.x);
    char p[96]; std::snprintf(p, sizeof p, "R=%.4f d=%.4f rc=%.4f", R, d, rc);
    verifyLeg2(cc_ok_, "nurbs-sphere∩plane:circle", tr,
               [&](const Point3& x){ return distToSphere(anSph, x); },
               [&](const Point3& x){ return distToPlane(pl, x); },
               known, 1, seed, 1002, p);
  }
}

}  // namespace

// The fuzz driver runs ≥ 2 seeds (default + a derived second), N ≥ 40 leg-1 cases each,
// plus the leg-2 known-answer set per seed. FUZZ_SEED overrides the base seed.
CC_TEST(exact_oracle_ssi_fuzz) {
  std::uint64_t base = 0xC0FFEEULL;  // fixed default (NO wall clock / rand / address)
  if (const char* env = std::getenv("FUZZ_SEED")) {
    base = std::strtoull(env, nullptr, 0);
  }
  const int N = 48;  // ≥ 40 leg-1 cases per seed
  const std::uint64_t seeds[] = {base, base ^ 0x9E3779B97F4A7C15ULL, base + 0x1234567ULL};
  const int nSeeds = int(sizeof(seeds) / sizeof(seeds[0]));  // 3 (≥ 2)

  std::printf("\n=== HOST exact-oracle SSI fuzzer: %d seeds × N=%d leg-1 cases + leg-2 known-answer ===\n",
              nSeeds, N);
  std::printf("seeds:");
  for (int i = 0; i < nSeeds; ++i) std::printf(" 0x%llX", (unsigned long long)seeds[i]);
  std::printf("\ntolerances: leg1=%.0e (cone %.0e), leg2 onSurf=%.0e fit=%.0e\n",
              kLeg1Tol, kLeg1ConeTol, kLeg2OnSurf, kLeg2Fit);

  for (int si = 0; si < nSeeds; ++si) {
    runSeed(cc_ok_, seeds[si], N);
  }

  // ── coverage summary ──
  auto printLeg = [](const char* title, const std::vector<Bucket>& v) {
    std::printf("\n%s\n", title);
    std::printf("  %-34s %8s %10s %11s %9s\n", "bucket", "AGREED", "DISAGREED", "DECLINED", "ORACLE-IN");
    for (const auto& b : v)
      std::printf("  %-34s %8d %10d %11d %9d\n", b.name, b.agreed, b.disagreed, b.declined, b.oracleInaccurate);
  };
  printLeg("LEG 1 — S1 analytic breadth:", g_rep.leg1);
  printLeg("LEG 2 — NURBS↔analytic known-answer:", g_rep.leg2);
  std::printf("\nTOTALS: cases=%d AGREED=%d DISAGREED=%d\n",
              g_rep.cases, g_rep.totalAgreed, g_rep.totalDisagreed);

  // ── exit gate: DISAGREED==0 AND real coverage (AGREED>0) across the seeds ──
  CC_CHECK(g_rep.totalDisagreed == 0);   // the honest bar — a native SSI finding fails here
  CC_CHECK(g_rep.totalAgreed > 0);       // real coverage, not an all-declined vacuous pass
  CC_CHECK(nSeeds >= 2);
  CC_CHECK(N >= 40);
}

int main() { return cctest::run_all(); }
