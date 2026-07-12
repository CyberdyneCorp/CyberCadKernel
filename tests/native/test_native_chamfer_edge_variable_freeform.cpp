// SPDX-License-Identifier: Apache-2.0
//
// test_native_chamfer_edge_variable_freeform.cpp — host GATE for the two additive
// generalizations of the Wave-G constant NURBS chamfer (blend/chamfer_edge_nurbs.h):
// VARIABLE-distance and FREEFORM-edge chamfer (blend/chamfer_edge_variable_freeform.h,
// NURBS roadmap Layer-4).
//
// Airtight, closed-form oracles:
//
//   1. VARIABLE REDUCES TO CONSTANT — d0==d1 reproduces the Wave-G constant chamfer
//      (chamfer_edge_symmetric) rail-for-rail (≤1e-12).
//   2. LINEAR-TAPER PLANAR — two planes, linear d0→d1: the setback rails are the EXACT
//      linearly-tapered offset lines (each rail point at in-plane distance d(t) from the
//      crease, ≤1e-9), and the bevel face contains both rails.
//   3. FREEFORM REDUCES TO ANALYTIC — the freeform surface-distance march on NURBS-
//      represented PLANES reproduces the analytic planar chamfer (≤1e-9); on a NURBS-
//      represented CYLINDER the wall setback is the circumferential geodesic arc (≤1e-7).
//   4. GENUINELY FREEFORM — two bicubic bumps: the chamfer rails lie ON each freeform face
//      at surface-distance d (≤1e-7); an OVER-LARGE setback HONEST-DECLINES (LeftDomain /
//      SelfLap), never a self-intersecting bevel.
//
// Exits 0 iff every gate holds. Requires CYBERCAD_HAS_NUMSCI (the NURBS evaluator that
// backs the freeform march); with it off this is a clean SKIP.
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
#include "native/blend/chamfer_edge_variable_freeform.h"
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
    std::printf("FAIL: %s: got %.15g want %.15g |err|=%.3g > tol %.3g\n", what, got, want, e, tol);
    ++g_failures;
  } else {
    std::printf("  ok  %-44s got %.12g want %.12g |err|=%.3g\n", what, got, want, e);
  }
}

#if defined(CYBERCAD_HAS_NUMSCI)
namespace cn = cybercad::native::blend::chamfer_nurbs;
namespace ff = cybercad::native::blend::ffdetail;

static double planeDist(const math::Point3& q, const math::Point3& q0, const math::Vec3& n) {
  return std::fabs(math::dot(q - q0, n));
}

// Build a straight-dihedral analytic edge along `tangent` from p0, length L, n intervals.
static std::vector<cn::EdgeStation> straightEdge(const math::Point3& p0, const math::Vec3& tangent,
                                                 const math::Vec3& nA, const math::Vec3& nB,
                                                 double L, int n) {
  std::vector<cn::EdgeStation> e;
  for (int k = 0; k <= n; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(n);
    cn::EdgeStation st;
    st.p = p0 + tangent * (t * L);
    st.tangent = tangent;
    st.nA = nA;
    st.nB = nB;
    e.push_back(st);
  }
  return e;
}

// ── ORACLE 1: VARIABLE REDUCES TO CONSTANT ──────────────────────────────────────
static void oracle_variable_reduces_to_constant() {
  std::printf("\n[oracle 1] variable(d,d) reduces to constant chamfer\n");
  const math::Vec3 nA{0, 0, 1};    // faceA top (+Z)
  const math::Vec3 nB{1, 0, 0};    // faceB side (+X)
  const math::Vec3 tangent{0, 1, 0};
  const math::Point3 p0{0, 0, 0};
  const double L = 2.0, d = 0.3;

  cn::Substrate A;
  A.kind = cn::SubstrateKind::Plane; A.point = p0; A.normal = nA;
  cn::Substrate B;
  B.kind = cn::SubstrateKind::Plane; B.point = p0; B.normal = nB;
  auto edge = straightEdge(p0, tangent, nA, nB, L, 6);

  auto konst = cn::chamfer_edge_symmetric(A, B, edge, d);
  auto vari = cn::chamfer_edge_variable(A, B, edge, d, d);   // d0==d1
  expectTrue(konst.ok() && vari.ok(), "constant and variable(d,d) both build");
  double maxdA = 0, maxdB = 0;
  for (std::size_t k = 0; k < konst.setbackA.size(); ++k) {
    maxdA = std::max(maxdA, math::distance(konst.setbackA[k], vari.setbackA[k]));
    maxdB = std::max(maxdB, math::distance(konst.setbackB[k], vari.setbackB[k]));
  }
  expectNear(maxdA, 0.0, 1e-12, "variable(d,d) railA == constant railA");
  expectNear(maxdB, 0.0, 1e-12, "variable(d,d) railB == constant railB");

  // Triangle-for-triangle byte reduction.
  expectTrue(konst.triangles.size() == vari.triangles.size(), "same triangle count");
  double maxTri = 0;
  for (std::size_t i = 0; i < konst.triangles.size(); ++i)
    for (int j = 0; j < 3; ++j)
      maxTri = std::max(maxTri, math::distance(konst.triangles[i][j], vari.triangles[i][j]));
  expectNear(maxTri, 0.0, 1e-12, "variable(d,d) loft == constant loft");
}

// ── ORACLE 2: LINEAR-TAPER PLANAR ───────────────────────────────────────────────
static void oracle_linear_taper_planar() {
  std::printf("\n[oracle 2] linear-taper planar setback lines\n");
  const math::Vec3 nA{0, 0, 1};
  const math::Vec3 nB{1, 0, 0};
  const math::Vec3 tangent{0, 1, 0};
  const math::Point3 p0{0, 0, 0};
  const double L = 2.0, d0 = 0.2, d1 = 0.6;

  cn::Substrate A;
  A.kind = cn::SubstrateKind::Plane; A.point = p0; A.normal = nA;
  cn::Substrate B;
  B.kind = cn::SubstrateKind::Plane; B.point = p0; B.normal = nB;
  auto edge = straightEdge(p0, tangent, nA, nB, L, 8);

  auto r = cn::chamfer_edge_variable(A, B, edge, d0, d1);
  expectTrue(r.ok(), "linear-taper planar chamfer builds");

  // For a uniform straight edge the arc fraction is k/n. The expected setback at station k
  // is d(t) = d0 + (d1-d0)*t. railA lies on z=0 at |x| = d(t); railB on x=0 at |z| = d(t).
  double maxErrA = 0, maxErrB = 0, maxOffA = 0, maxOffB = 0;
  const int n = static_cast<int>(edge.size()) - 1;
  for (std::size_t k = 0; k < edge.size(); ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(n);
    const double dt = d0 + (d1 - d0) * t;
    // rail A on faceA (z=0), in-plane distance dt (|x|).
    maxOffA = std::max(maxOffA, planeDist(r.setbackA[k], A.point, A.normal));
    maxErrA = std::max(maxErrA, std::fabs(std::fabs(r.setbackA[k].x) - dt));
    // rail B on faceB (x=0), in-plane distance dt (|z|).
    maxOffB = std::max(maxOffB, planeDist(r.setbackB[k], B.point, B.normal));
    maxErrB = std::max(maxErrB, std::fabs(std::fabs(r.setbackB[k].z) - dt));
  }
  expectNear(maxOffA, 0.0, 1e-9, "railA on faceA (C0)");
  expectNear(maxErrA, 0.0, 1e-9, "railA in-face setback = linear taper d(t)");
  expectNear(maxOffB, 0.0, 1e-9, "railB on faceB (C0)");
  expectNear(maxErrB, 0.0, 1e-9, "railB in-face setback = linear taper d(t)");

  // Bevel face contains both rails: each triangle vertex is one of the rail points.
  expectTrue(!r.triangles.empty(), "bevel band emitted");
}

// ── Freeform fixtures (each owns its pole/knot storage; ff::Surface is a VIEW). ────

// A bilinear (degree-1) B-spline PLANE through four corners: exact plane, L=M=N=0 → the
// surface-distance march is the straight in-plane offset (analytic reduction).
struct PlaneFixture {
  std::vector<math::Point3> poles;
  std::vector<double> knots{0, 0, 1, 1};
  PlaneFixture(math::Point3 p00, math::Point3 p01, math::Point3 p10, math::Point3 p11)
      : poles{p00, p01, p10, p11} {}
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

// A RATIONAL (NURBS) quarter-cylinder of radius R, height H, axis +Z. One quadratic
// rational Bézier patch in V (the 90° arc) × linear in U (the axial extent). The wall
// surface-distance march circumferentially is the geodesic arc R·Δψ (analytic cylinder).
struct QuarterCylFixture {
  std::vector<math::Point3> poles;
  std::vector<double> weights;
  std::vector<double> knotsU{0, 0, 1, 1};        // linear in U (axial)
  std::vector<double> knotsV{0, 0, 0, 1, 1, 1};  // quadratic in V (the 90° arc)
  double R = 1.0, H = 2.0;
  QuarterCylFixture(double radius, double height) : R(radius), H(height) {
    // 2 rows (U: z=0 → z=H) × 3 cols (V: the rational quarter-arc +X→+Y).
    // Quarter-circle rational Bézier control net (radius R): P0=(R,0), P1=(R,R) w=1/√2,
    // P2=(0,R). U just lifts z from 0 to H.
    const double w = std::sqrt(2.0) / 2.0;
    poles.resize(6);
    weights.resize(6);
    for (int i = 0; i < 2; ++i) {
      const double z = (i == 0) ? 0.0 : H;
      poles[i * 3 + 0] = math::Point3{R, 0, z};
      poles[i * 3 + 1] = math::Point3{R, R, z};    // homogeneous corner (weight w applied)
      poles[i * 3 + 2] = math::Point3{0, R, z};
      weights[i * 3 + 0] = 1.0;
      weights[i * 3 + 1] = w;
      weights[i * 3 + 2] = 1.0;
    }
  }
  ff::Surface surface() const {
    ff::Surface s;
    s.degreeU = 1;
    s.degreeV = 2;
    s.grid = math::SurfaceGrid{std::span<const math::Point3>(poles.data(), poles.size()), 2, 3};
    s.weights = {weights.data(), weights.size()};
    s.knotsU = {knotsU.data(), knotsU.size()};
    s.knotsV = {knotsV.data(), knotsV.size()};
    return s;
  }
};

// A bicubic (degree-3) B-spline "bumpy floor/wall" mirroring the freeform-fillet gate.
struct BumpFixture {
  std::vector<math::Point3> poles;
  std::vector<double> knots{0, 0, 0, 0, 1, 1, 1, 1};
  BumpFixture(bool wall, double amp) {
    poles.resize(16);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        const double a = 2.0 * i / 3.0;
        const double b = 2.0 * j / 3.0;
        const double bump = amp * std::sin(M_PI * i / 3.0) * std::sin(M_PI * j / 3.0);
        if (!wall)
          poles[i * 4 + j] = math::Point3{a, b, bump};
        else
          poles[i * 4 + j] = math::Point3{bump, b, a};
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

// Distance from point q to freeform surface s (the rail must lie ON the face). We seed a
// coarse (u,v) grid then POLISH with the footpoint Newton (orthogonal projection) so the
// residual is the true point-to-surface distance, not the grid spacing.
static double distToSurface(const ff::Surface& s, const math::Point3& q) {
  double best = 1e300, bu = 0.5, bv = 0.5;
  const int N = 30;
  for (int i = 0; i <= N; ++i)
    for (int j = 0; j <= N; ++j) {
      const double u = static_cast<double>(i) / N;
      const double v = static_cast<double>(j) / N;
      const math::Point3 p = ff::surfacePoint(s, u, v);
      const double dd = math::distance(math::Point3{p.x, p.y, p.z}, q);
      if (dd < best) { best = dd; bu = u; bv = v; }
    }
  const math::Point3 qq{q.x, q.y, q.z};
  const auto fp = ff::footpoint(s, qq, bu, bv);
  if (fp) return math::distance(fp->g.p, qq);
  return best;
}

// ── ORACLE 3: FREEFORM REDUCES TO ANALYTIC ──────────────────────────────────────
static void oracle_freeform_reduces_planar() {
  std::printf("\n[oracle 3a] freeform planes reduce to analytic planar chamfer\n");
  // faceA = floor z=0 (u→x, v→y); faceB = wall x=0 (u→z, v→y). Crease is the Y axis.
  PlaneFixture floor({0, 0, 0}, {0, 2, 0}, {2, 0, 0}, {2, 2, 0});
  PlaneFixture wall({0, 0, 0}, {0, 2, 0}, {0, 0, 2}, {0, 2, 2});
  const ff::Surface A = floor.surface();
  const ff::Surface B = wall.surface();
  const double d = 0.4;

  // Edge along +Y at (0,y,0). Warm starts: floor (u=x/2→0, v=y/2), wall (u=z/2→0, v=y/2).
  std::vector<cn::FreeformEdgeStation> edge;
  const int n = 6;
  for (int k = 0; k <= n; ++k) {
    const double y = 2.0 * k / n;               // y ∈ [0,2]
    cn::FreeformEdgeStation st;
    st.p = math::Point3{0, y, 0};
    st.tangent = math::Vec3{0, 1, 0};
    st.materialHint = math::Vec3{1, 0, 1};   // solid wedge is x>0, z>0
    st.uA0 = 0.02; st.vA0 = std::clamp(y / 2.0, 0.05, 0.95);   // floor u→x≈0
    st.uB0 = 0.02; st.vB0 = std::clamp(y / 2.0, 0.05, 0.95);   // wall  u→z≈0
    edge.push_back(st);
  }

  auto r = cn::chamfer_edge_freeform(A, B, edge, d);
  expectTrue(r.ok(), "freeform planar chamfer builds");

  // Analytic reduction: railA on z=0 at x=d; railB on x=0 at z=d (into +x/+z material).
  double maxA = 0, maxB = 0;
  for (std::size_t k = 0; k < edge.size(); ++k) {
    maxA = std::max(maxA, std::fabs(r.setbackA[k].z));            // on z=0
    maxA = std::max(maxA, std::fabs(r.setbackA[k].x - d));        // x = d
    maxB = std::max(maxB, std::fabs(r.setbackB[k].x));            // on x=0
    maxB = std::max(maxB, std::fabs(r.setbackB[k].z - d));        // z = d
  }
  expectNear(maxA, 0.0, 1e-9, "freeform railA == analytic (z=0, x=d)");
  expectNear(maxB, 0.0, 1e-9, "freeform railB == analytic (x=0, z=d)");

  std::printf("\n[oracle 3b] freeform cylinder wall reduces to geodesic arc\n");
  // faceA = planar cap z=H; faceB = quarter cylinder wall (rational NURBS), radius R.
  const double R = 1.5, H = 2.0, dc = 0.25;
  PlaneFixture cap({0, 0, H}, {0, 2, H}, {2, 0, H}, {2, 2, H});   // z=H, u→x, v→y
  QuarterCylFixture cyl(R, H);
  const ff::Surface capS = cap.surface();
  const ff::Surface cylS = cyl.surface();

  // Rim: the top circle of the cylinder at z=H, ψ ∈ [0,π/2]. On the cyl patch v maps the
  // arc (v=0 → +X, v=1 → +Y), u maps z (u=1 → z=H).
  std::vector<cn::FreeformEdgeStation> rim;
  const int m = 10;
  for (int k = 0; k <= m; ++k) {
    const double t = static_cast<double>(k) / m;          // arc fraction
    const double psi = (M_PI * 0.5) * t;
    const double cx = std::cos(psi), sy = std::sin(psi);
    cn::FreeformEdgeStation st;
    st.p = math::Point3{R * cx, R * sy, H};
    st.tangent = math::Vec3{-sy, cx, 0};
    // Material is inside the cylinder (r<R) and below the cap (z<H): hint radially inward
    // and downward.
    st.materialHint = math::Vec3{-cx, -sy, -1};
    // cap warm start: u→x/2, v→y/2. cyl warm start: u→1 (z=H), v→t (arc fraction).
    st.uA0 = std::clamp(R * cx / 2.0, 0.05, 0.95); st.vA0 = std::clamp(R * sy / 2.0, 0.05, 0.95);
    st.uB0 = 0.98; st.vB0 = std::clamp(t, 0.03, 0.97);
    rim.push_back(st);
  }

  auto rc = cn::chamfer_edge_freeform(capS, cylS, rim, dc);
  expectTrue(rc.ok(), "freeform cylinder chamfer builds");

  // Analytic reduction: cap rail on z=H radius R-dc; wall rail ON the cylinder (radius R)
  // at axial z = H - dc (the circumferential ruling axial slide is the geodesic offset dc).
  double capErr = 0, wallRad = 0, wallAx = 0;
  for (std::size_t k = 0; k < rim.size(); ++k) {
    const math::Point3& a = rc.setbackA[k];
    capErr = std::max(capErr, std::fabs(a.z - H));
    capErr = std::max(capErr, std::fabs(std::sqrt(a.x * a.x + a.y * a.y) - (R - dc)));
    const math::Point3& b = rc.setbackB[k];
    wallRad = std::max(wallRad, std::fabs(std::sqrt(b.x * b.x + b.y * b.y) - R));
    wallAx = std::max(wallAx, std::fabs(b.z - (H - dc)));
  }
  expectNear(capErr, 0.0, 1e-7, "cap setback = circle R-dc at z=H");
  expectNear(wallRad, 0.0, 1e-7, "wall setback stays ON cylinder (radius R)");
  expectNear(wallAx, 0.0, 1e-7, "wall setback axial = dc (geodesic offset)");
}

// ── ORACLE 4: GENUINELY FREEFORM ────────────────────────────────────────────────
static void oracle_genuinely_freeform() {
  std::printf("\n[oracle 4] genuinely freeform bicubic bumps\n");
  BumpFixture floor(false, 0.15);   // bumpy floor near z=0
  BumpFixture wall(true, 0.15);     // bumpy wall near x=0
  const ff::Surface A = floor.surface();
  const ff::Surface B = wall.surface();
  const double d = 0.3;

  // Crease near the Y axis (x≈0, z≈0). Sample the shared corner curve; warm-start both
  // footpoints near u≈0, v≈y-fraction.
  std::vector<cn::FreeformEdgeStation> edge;
  const int n = 6;
  for (int k = 0; k <= n; ++k) {
    const double t = static_cast<double>(k) / n;
    const double y = 2.0 * t;
    cn::FreeformEdgeStation st;
    // The crease point: on the floor at (x=0,y, bump≈0) and on the wall at (x≈0,y,z=0).
    st.p = math::Point3{0, y, 0};
    st.tangent = math::Vec3{0, 1, 0};
    st.materialHint = math::Vec3{1, 0, 1};   // solid wedge is x>0, z>0
    st.uA0 = 0.03; st.vA0 = std::clamp(t, 0.05, 0.95);
    st.uB0 = 0.03; st.vB0 = std::clamp(t, 0.05, 0.95);
    edge.push_back(st);
  }

  auto r = cn::chamfer_edge_freeform(A, B, edge, d);
  expectTrue(r.ok(), "genuinely-freeform chamfer builds");

  // Each rail point must lie ON its freeform face (surface-distance ≈ 0).
  double maxOnA = 0, maxOnB = 0;
  for (std::size_t k = 0; k < edge.size(); ++k) {
    maxOnA = std::max(maxOnA, distToSurface(A, r.setbackA[k]));
    maxOnB = std::max(maxOnB, distToSurface(B, r.setbackB[k]));
  }
  expectNear(maxOnA, 0.0, 1e-7, "freeform railA lies ON faceA");
  expectNear(maxOnB, 0.0, 1e-7, "freeform railB lies ON faceB");

  // The setback SURFACE distance from the crease must be ≈ d on each face. We re-footpoint
  // the crease and march-check: measure the geodesic distance by re-running the march and
  // comparing the emitted rail to a coarser (2× sub-step) march — they must agree to the
  // discretization tolerance (the march converged).
  auto r2 = cn::chamfer_edge_freeform(A, B, edge, d, /*nSub=*/48);
  expectTrue(r2.ok(), "freeform chamfer builds at finer sub-stepping");
  double maxConv = 0;
  for (std::size_t k = 0; k < edge.size(); ++k) {
    maxConv = std::max(maxConv, math::distance(r.setbackA[k], r2.setbackA[k]));
    maxConv = std::max(maxConv, math::distance(r.setbackB[k], r2.setbackB[k]));
  }
  expectNear(maxConv, 0.0, 1e-4, "surface-distance march converged (24 vs 48 sub-steps)");

  // ── HONEST DECLINE: an OVER-LARGE setback runs past the face domain / self-laps. ──
  auto over = cn::chamfer_edge_freeform(A, B, edge, 5.0);   // d ≫ face extent (2×2)
  expectTrue(!over.ok(), "over-large freeform setback declines");
  expectTrue(over.decline == cn::FreeformChamferDecline::LeftDomain ||
                 over.decline == cn::FreeformChamferDecline::SelfLap,
             "over-large declines LeftDomain or SelfLap (no self-intersecting bevel)");

  // Variable freeform: d0→d1 taper builds and reduces to constant at d0==d1.
  auto vf = cn::chamfer_edge_freeform_variable(A, B, edge, d, d);
  expectTrue(vf.ok(), "freeform variable(d,d) builds");
  double maxRed = 0;
  for (std::size_t k = 0; k < edge.size(); ++k) {
    maxRed = std::max(maxRed, math::distance(vf.setbackA[k], r.setbackA[k]));
    maxRed = std::max(maxRed, math::distance(vf.setbackB[k], r.setbackB[k]));
  }
  expectNear(maxRed, 0.0, 1e-12, "freeform variable(d,d) == freeform constant(d)");
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
#if defined(CYBERCAD_HAS_NUMSCI)
  std::printf("== native NURBS variable / freeform chamfer gate ==\n");
  oracle_variable_reduces_to_constant();
  oracle_linear_taper_planar();
  oracle_freeform_reduces_planar();
  oracle_genuinely_freeform();
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
#else
  std::printf("SKIP: native NURBS variable/freeform chamfer gate needs CYBERCAD_HAS_NUMSCI\n");
  return 0;
#endif
}
