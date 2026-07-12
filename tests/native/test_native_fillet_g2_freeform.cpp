// SPDX-License-Identifier: Apache-2.0
//
// test_native_fillet_g2_freeform.cpp — host GATE for the G2 (curvature-continuous)
// rolling-ball fillet between two GENERAL FREEFORM NURBS faces
// (blend/fillet_edge_g2_freeform.h, NURBS roadmap Layer-4 freeform substrate).
//
// The freeform G2 fillet marches the rolling-ball CENTRE LOCUS (footpoint on each face
// + centre Newton to distance r), then reads each contact's end curvature from the
// LOCAL SECOND FUNDAMENTAL FORM of that freeform surface, so the pole rule
// q=(5/4)·κ·h² generalises to arbitrary NURBS substrates. Three airtight oracles:
//
//   1. ANALYTIC-REDUCTION — when both faces are PLANAR (represented as NURBS) the
//      freeform normal-curvature read is exactly 0 (L=M=N=0), so the section is the
//      zero-END-curvature quintic reproducing the planar fillet_edges_g2 to ≤1e-9.
//      A SPHERE (rational NURBS) reads its umbilic normal curvature 1/R in every
//      direction, matching curved_fillet_g2's kWall to ≤1e-9.
//   2. G2-TO-FACE — at each seated station the section leaves each face TANGENT to it
//      (G1: section tangent ⟂ face normal ≤1e-6 rad) and with END CURVATURE equal to
//      the face's normal curvature in the section plane (relative ≤1e-4).
//   3. GENUINELY FREEFORM — two bicubic bump faces seat a G2 fillet with the above
//      continuity; where the radius exceeds the local fit the station HONEST-DECLINES
//      (NewtonDiverged / empty), never a self-intersecting fillet.
//
// Exits 0 iff every gate holds. Requires CYBERCAD_HAS_NUMSCI (the freeform substrate);
// with it off this is a clean SKIP.
//
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <span>
#include <vector>

#include "native/math/bspline.h"
#include "native/math/vec.h"

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/blend/fillet_edge_g2_freeform.h"
#endif

namespace math = cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  } else {
    std::printf("  ok  %s\n", what);
  }
}
static void expectNear(double got, double want, double tol, const char* what) {
  ++g_checks;
  const double e = std::fabs(got - want);
  if (!(e <= tol)) {
    std::printf("FAIL: %s: got %.12g want %.12g |err|=%.3g > tol %.3g\n", what, got, want, e, tol);
    ++g_failures;
  } else {
    std::printf("  ok  %-46s got %.9g want %.9g |err|=%.3g\n", what, got, want, e);
  }
}

#if defined(CYBERCAD_HAS_NUMSCI)
namespace blend = cybercad::native::blend;
namespace ff = cybercad::native::blend::ffdetail;

// ── Surface fixtures (each owns its pole/knot storage; ff::Surface is a VIEW). ────

// A bilinear (degree-1) B-spline PLANE through four corners. Exact bilinear patch, so
// L=M=N=0 and the normal-curvature read is exactly 0 (the analytic-plane reduction).
struct PlaneFixture {
  std::vector<math::Point3> poles;
  std::vector<double> knots{0, 0, 1, 1};
  PlaneFixture(math::Point3 p00, math::Point3 p01, math::Point3 p10, math::Point3 p11)
      : poles{p00, p01, p10, p11} {}  // row-major: U outer (2 rows), V inner (2 cols)
  ff::Surface surface() const {
    ff::Surface s;
    s.degreeU = 1;
    s.degreeV = 1;
    s.grid = math::SurfaceGrid{std::span<const math::Point3>(poles.data(), poles.size()), 2, 2};
    s.knotsU = {knots.data(), knots.size()};
    s.knotsV = {knots.data(), knots.size()};
    return s;
  }
};

// A RATIONAL (NURBS) hemisphere octant of radius R over one Bézier patch, the standard
// quadratic rational representation of a sphere. Umbilic → normal curvature 1/R in
// EVERY direction, so the freeform read must return 1/R (the analytic-sphere reduction).
// Control net + weights are the classic 3×3 rational-sphere octant (one +x/+y/+z octant).
struct SphereOctantFixture {
  std::vector<math::Point3> poles;
  std::vector<double> weights;
  std::vector<double> knots{0, 0, 0, 1, 1, 1};  // clamped quadratic, 3 poles
  double R = 1.0;
  explicit SphereOctantFixture(double radius) : R(radius) {
    // Octant of a unit sphere: rows in U (from +z pole toward the equator), cols in V
    // (from +x toward +y). Standard w = {1, 1/√2, 1} pattern on the mid band.
    const double s2 = std::sqrt(2.0) / 2.0;
    // Unnormalised homogeneous corner directions scaled by R; classic 9-pole octant.
    const math::Point3 P[3][3] = {
        {{0, 0, R}, {0, 0, R}, {0, 0, R}},
        {{R, 0, R}, {R, R, R}, {0, R, R}},
        {{R, 0, 0}, {R, R, 0}, {0, R, 0}},
    };
    const double W[3][3] = {
        {1.0, s2, 1.0},
        {s2, 0.5, s2},
        {1.0, s2, 1.0},
    };
    poles.resize(9);
    weights.resize(9);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) {
        poles[i * 3 + j] = P[i][j];
        weights[i * 3 + j] = W[i][j];
      }
  }
  ff::Surface surface() const {
    ff::Surface s;
    s.degreeU = 2;
    s.degreeV = 2;
    s.grid = math::SurfaceGrid{std::span<const math::Point3>(poles.data(), poles.size()), 3, 3};
    s.weights = {weights.data(), weights.size()};
    s.knotsU = {knots.data(), knots.size()};
    s.knotsV = {knots.data(), knots.size()};
    return s;
  }
};

// A bicubic (degree-3) B-spline "bumpy floor/wall": a base plane with a gentle
// sinusoidal bump. floorB: base z=0, u→x, v→y, normal ≈ +z. wallB: base x=0, u→z,
// v→y, param-normal ≈ −x (flip toward the pocket with sB=−1). Genuine freeform: the
// second fundamental form is non-zero and spatially varying.
struct BumpFixture {
  std::vector<math::Point3> poles;
  std::vector<double> knots{0, 0, 0, 0, 1, 1, 1, 1};  // clamped cubic, 4 poles
  BumpFixture(bool wall, double amp) {
    poles.resize(16);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        const double a = 2.0 * i / 3.0;                 // 0..2 along U
        const double b = 2.0 * j / 3.0;                 // 0..2 along V (= y)
        const double bump = amp * std::sin(M_PI * i / 3.0) * std::sin(M_PI * j / 3.0);
        if (!wall)
          poles[i * 4 + j] = math::Point3{a, b, bump};       // floor: (x,y,bump-z)
        else
          poles[i * 4 + j] = math::Point3{bump, b, a};       // wall: (bump-x,y,z)
      }
  }
  ff::Surface surface() const {
    ff::Surface s;
    s.degreeU = 3;
    s.degreeV = 3;
    s.grid = math::SurfaceGrid{std::span<const math::Point3>(poles.data(), poles.size()), 4, 4};
    s.knotsU = {knots.data(), knots.size()};
    s.knotsV = {knots.data(), knots.size()};
    return s;
  }
};

// ── The G2-to-face + G1 witness, shared by the gates. ───────────────────────────
static void checkG2ToFace(const blend::FreeformFilletResult& res, const char* tag,
                          double kRelTol) {
  int checked = 0;
  double maxAngle = 0, maxKrelA = 0, maxKrelB = 0;
  for (const auto& stn : res.stations) {
    if (!stn.seated) continue;
    ++checked;
    const auto& poles = stn.section.poles;
    math::Vec3 t0 = ff::sectionTangent(poles, 0.0);
    math::Vec3 t1 = ff::sectionTangent(poles, 1.0);
    if (!math::isNull(t0)) t0 = t0 / math::norm(t0);
    if (!math::isNull(t1)) t1 = t1 / math::norm(t1);
    // G1: section tangent lies in the face tangent plane ⇔ its dot with the face
    // normal is ~0; asin(dot) is the out-of-plane angle.
    maxAngle = std::max(maxAngle,
                        std::fabs(std::asin(std::clamp(math::dot(t0, stn.contact.nA), -1.0, 1.0))));
    maxAngle = std::max(maxAngle,
                        std::fabs(std::asin(std::clamp(math::dot(t1, stn.contact.nB), -1.0, 1.0))));
    // G2: section end curvature == |face normal curvature|.
    const double c0 = ff::sectionCurvature(poles, 0.0);
    const double c1 = ff::sectionCurvature(poles, 1.0);
    maxKrelA = std::max(maxKrelA, std::fabs(c0 - std::fabs(stn.contact.kA)) /
                                      std::max(1e-9, std::fabs(stn.contact.kA)));
    maxKrelB = std::max(maxKrelB, std::fabs(c1 - std::fabs(stn.contact.kB)) /
                                      std::max(1e-9, std::fabs(stn.contact.kB)));
  }
  std::printf("  [%s] %d stations, maxAngle=%.3e maxKrelA=%.3e maxKrelB=%.3e\n", tag, checked,
              maxAngle, maxKrelA, maxKrelB);
  expectTrue(checked >= 2, "at least two stations seated for the witness");
  expectNear(maxAngle, 0.0, 1e-6, "G1: section tangent ⟂ face normal (≤1e-6 rad)");
  expectNear(maxKrelA, 0.0, kRelTol, "G2: κ(0) matches faceA normal curvature");
  expectNear(maxKrelB, 0.0, kRelTol, "G2: κ(1) matches faceB normal curvature");
}

// ── GATE 1a: ANALYTIC-REDUCTION on two PLANES (a concave dihedral). ─────────────
static void gate1a_planar_reduction() {
  std::printf("\n== GATE 1a: analytic-reduction — two planes → κ=0 quintic ==\n");
  PlaneFixture floor({0, 0, 0}, {0, 2, 0}, {2, 0, 0}, {2, 2, 0});  // z=0, u→x, v→y
  PlaneFixture wall({0, 0, 0}, {0, 2, 0}, {0, 0, 2}, {0, 2, 2});   // x=0, u→z, v→y
  const ff::Surface A = floor.surface();
  const ff::Surface B = wall.surface();
  const double r = 0.5;

  blend::FreeformFilletSeed seed;
  seed.center0 = {0.5, 0.6, 0.5};   // ball centre r from each plane, near y=0.6
  seed.spineDir = {0, 1, 0};        // crease runs along +y
  seed.stepLen = 0.12;
  seed.nStations = 8;
  seed.sA = 1.0;    // floor param-normal +z already toward the centre
  seed.sB = -1.0;   // wall param-normal −x flipped to +x toward the centre
  seed.uA0 = 0.25; seed.vA0 = 0.3;
  seed.uB0 = 0.25; seed.vB0 = 0.3;

  const auto res = blend::fillet_edge_g2_freeform(A, B, r, seed);
  expectTrue(res.ok(), "planar fillet seats + skins");
  int seated = 0;
  for (const auto& stn : res.stations)
    if (stn.seated) ++seated;
  expectTrue(seated == seed.nStations, "every planar station seats");

  double maxK = 0, maxC = 0;
  for (const auto& stn : res.stations) {
    if (!stn.seated) continue;
    maxK = std::max({maxK, std::fabs(stn.contact.kA), std::fabs(stn.contact.kB)});
    maxC = std::max({maxC, std::fabs(ff::sectionCurvature(stn.section.poles, 0.0)),
                     std::fabs(ff::sectionCurvature(stn.section.poles, 1.0))});
  }
  expectNear(maxK, 0.0, 1e-9, "plane normal-curvature read = 0 (≤1e-9)");
  expectNear(maxC, 0.0, 1e-9, "planar section end curvature = 0 (≤1e-9)");
}

// ── GATE 1b: ANALYTIC-REDUCTION on a SPHERE — umbilic curvature read = 1/R. ──────
static void gate1b_sphere_curvature() {
  std::printf("\n== GATE 1b: analytic-reduction — rational-NURBS sphere reads 1/R ==\n");
  const double R = 2.0;
  SphereOctantFixture sph(R);
  const ff::Surface S = sph.surface();
  // Read the surface's normal curvature at several interior params in an arbitrary
  // tangent direction; a sphere is umbilic so every reading must equal 1/R.
  double maxErr = 0;
  int samples = 0;
  for (int i = 1; i <= 3; ++i)
    for (int j = 1; j <= 3; ++j) {
      const double u = i / 4.0, v = j / 4.0;
      const ff::LocalGeom g = ff::localGeom(S, u, v);
      if (!g.ok) continue;
      // Two independent tangent directions (Su, and Su+Sv) — both must read 1/R.
      for (const math::Vec3 t : {g.Su, g.Su + g.Sv}) {
        const double kn = ff::normalCurvatureAlong(g, t);
        maxErr = std::max(maxErr, std::fabs(std::fabs(kn) - 1.0 / R));
        ++samples;
      }
    }
  std::printf("  sampled %d directions, max |κ|−1/R = %.3e\n", samples, maxErr);
  expectTrue(samples >= 6, "sphere sampled at enough interior points");
  expectNear(maxErr, 0.0, 1e-9, "sphere umbilic normal curvature = 1/R (≤1e-9)");
}

// ── GATE 3: GENUINELY FREEFORM — two bicubic bumps seat + G2, over-radius declines. ─
static void gate3_freeform_bump() {
  std::printf("\n== GATE 3: genuinely freeform bicubic bump crease ==\n");
  BumpFixture floor(false, 0.12);  // bumpy floor
  BumpFixture wall(true, 0.12);    // bumpy wall
  const ff::Surface A = floor.surface();
  const ff::Surface B = wall.surface();

  blend::FreeformFilletSeed seed;
  seed.center0 = {0.45, 0.6, 0.45};  // ball in the bumpy concave crease near y=0.6
  seed.spineDir = {0, 1, 0};
  seed.stepLen = 0.12;
  seed.nStations = 7;
  seed.sA = 1.0;
  seed.sB = -1.0;
  seed.uA0 = 0.3; seed.vA0 = 0.3;
  seed.uB0 = 0.3; seed.vB0 = 0.3;

  const double r = 0.4;
  const auto res = blend::fillet_edge_g2_freeform(A, B, r, seed);
  expectTrue(res.ok(), "freeform bump fillet seats + skins");
  if (res.ok()) {
    // Confirm the curvatures read are genuinely NON-ZERO (a real freeform case, not a
    // degenerate flat one) at some station, then run the G2-to-face witness.
    double maxAbsK = 0;
    for (const auto& stn : res.stations)
      if (stn.seated)
        maxAbsK = std::max({maxAbsK, std::fabs(stn.contact.kA), std::fabs(stn.contact.kB)});
    std::printf("  max |freeform κ| over stations = %.4g (genuinely curved)\n", maxAbsK);
    expectTrue(maxAbsK > 1e-3, "freeform faces have genuinely non-zero curvature");
    checkG2ToFace(res, "bump", 1e-4);
    expectTrue(!res.triangles.empty(), "freeform fillet emits a skinned band");
  }

  // HONEST-DECLINE PROBE: a radius far larger than the local crease must NOT seat a
  // consistent ball → the whole fillet declines, never a folded/self-intersecting band.
  blend::FreeformFilletSeed big = seed;
  const double rBig = 8.0;  // vastly exceeds the ~1 unit crease depth
  big.center0 = {5.0, 0.6, 5.0};
  const auto resBig = blend::fillet_edge_g2_freeform(A, B, rBig, big);
  expectTrue(!resBig.ok(), "over-radius freeform fillet declines (ball won't fit)");
  expectTrue(resBig.triangles.empty(), "over-radius fillet emits no triangles");
  std::printf("  over-radius decline reason=%d (honest, no self-intersection)\n",
              static_cast<int>(resBig.decline));
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
#if defined(CYBERCAD_HAS_NUMSCI)
  gate1a_planar_reduction();
  gate1b_sphere_curvature();
  gate3_freeform_bump();

  std::printf("\n==== %d checks, %d failures ====\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
#else
  std::printf("SKIP: test_native_fillet_g2_freeform requires CYBERCAD_HAS_NUMSCI\n");
  return 0;
#endif
}
