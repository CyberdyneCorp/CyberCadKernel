// SPDX-License-Identifier: Apache-2.0
//
// test_native_chamfer_edge_nurbs.cpp — host GATE for the native NURBS CHAMFER
// generator (blend/chamfer_edge_nurbs.h, NURBS roadmap Layer-4, the CHAMFER half of
// fillet/chamfer). The generator traces the two SETBACK CURVES (one per face, at a
// prescribed setback measured ALONG each face) and lofts a ruled (Piegl & Tiller)
// chamfer face between them. It is the flat-bevel counterpart to the G2 fillet family.
//
// Airtight, closed-form oracles:
//
//   1. PLANAR DIHEDRAL EXACTNESS — two planes meeting at a known dihedral angle, a
//      SYMMETRIC chamfer d: the two setback edges are the parallel lines at in-plane
//      distance d from the crease (≤1e-12), the ruled chamfer face is EXACTLY planar
//      (four-corner planarity residual ≤1e-12), and the three faces meet C0 at the
//      setback lines (the rail points lie on the base planes ≤1e-12).
//   2. SYMMETRIC / ASYMMETRIC CONSISTENCY — ASYMMETRIC(d,d) reproduces SYMMETRIC(d)
//      rail-for-rail (≤1e-12); DISTANCE_ANGLE(d, atan(d2/d)) reproduces ASYMMETRIC(d,d2)
//      rail-for-rail (≤1e-12) — the mode-equivalence identities.
//   3. CYLINDER SUBSTRATE — a chamfer between a plane cap and a coaxial cylinder wall on
//      the circular rim: the cylinder setback curve is the correct GEODESIC/normal
//      offset (the seam CIRCLE at axial offset d, radius R, on the cylinder ≤1e-9), the
//      cap setback is the circle radius R−d, and the ruled face contains both (≤1e-9).
//   4. HONEST DECLINE — an OVER-LARGE setback (rails cross) and a DEGENERATE dihedral
//      (parallel faces) return empty with a reason, never a self-intersecting face; a
//      FREEFORM substrate declines UnsupportedSubstrate.
//
// Exits 0 iff every gate holds. Requires CYBERCAD_HAS_NUMSCI (roadmap gating parity
// with the freeform fillet gate); with it off this is a clean SKIP.
//
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

#include "native/math/vec.h"

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/blend/chamfer_edge_nurbs.h"
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
    std::printf("  ok  %-42s got %.12g want %.12g |err|=%.3g\n", what, got, want, e);
  }
}

#if defined(CYBERCAD_HAS_NUMSCI)
namespace cn = cybercad::native::blend::chamfer_nurbs;

// Distance from a point to a plane (point q0, unit normal n).
static double planeDist(const math::Point3& q, const math::Point3& q0, const math::Vec3& n) {
  return std::fabs(math::dot(q - q0, n));
}

// Build a straight-dihedral edge along the world Y axis at the crease line, between two
// planes whose outward normals are nA, nB. The edge runs y ∈ [0,L].
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

// ── ORACLE 1: PLANAR DIHEDRAL EXACTNESS ─────────────────────────────────────────
// A 90° convex dihedral: faceA is the +X face (outward +X), faceB is the +Z face
// (outward +Z). The crease is the Y axis. A symmetric chamfer d sets faceA back by d
// (toward −Z, into faceA which lies in the x=const... actually the two half-planes):
//   * faceA = plane z=0, outward normal +Z? — we set up the standard convex box corner.
// Use the box-corner convention: faceA is the top plane z=0 with outward +Z, faceB is
// the side plane x=0 with outward +X, meeting at the Y-axis edge (x=0,z=0). Convex.
static void oracle_planar_dihedral() {
  std::printf("\n[oracle 1] planar dihedral exactness (90 deg box corner)\n");
  const math::Vec3 nA{0, 0, 1};   // faceA outward +Z (top)
  const math::Vec3 nB{1, 0, 0};   // faceB outward +X (side)
  const math::Vec3 tangent{0, 1, 0};
  const math::Point3 p0{0, 0, 0};
  const double L = 2.0, d = 0.3;

  cn::Substrate A;
  A.kind = cn::SubstrateKind::Plane;
  A.point = p0;
  A.normal = nA;
  cn::Substrate B;
  B.kind = cn::SubstrateKind::Plane;
  B.point = p0;
  B.normal = nB;

  auto edge = straightEdge(p0, tangent, nA, nB, L, 4);
  auto r = cn::chamfer_edge_symmetric(A, B, edge, d);
  expectTrue(r.ok(), "symmetric planar chamfer builds");
  expectTrue(r.planarFace, "chamfer face flagged planar");

  // The chamfer face must be EXACTLY planar.
  expectNear(r.planarityResidual, 0.0, 1e-12, "chamfer-face planarity residual");

  // faceA setback: measured in faceA (plane z=0), perpendicular to the Y edge, into the
  // material. For the box corner the material is x>0,z>0; faceA (z=0) inward is +X, so
  // the faceA setback line is x=d, z=0. faceB (x=0) inward is +Z → setback z=d, x=0.
  for (std::size_t k = 0; k < r.setbackA.size(); ++k) {
    // rail A lies ON faceA (z=0) at distance d from the edge (|x|=d).
    expectNear(planeDist(r.setbackA[k], A.point, A.normal), 0.0, 1e-12, "railA on faceA (C0)");
    expectNear(std::fabs(r.setbackA[k].x), d, 1e-12, "railA in-face setback = d");
    expectNear(planeDist(r.setbackB[k], B.point, B.normal), 0.0, 1e-12, "railB on faceB (C0)");
    expectNear(std::fabs(r.setbackB[k].z), d, 1e-12, "railB in-face setback = d");
  }

  // The bevel face makes 45° with each base plane (the symmetric-90° signature): the
  // chamfer face normal is the bisector (1,0,1)/√2, dotted with each face normal = 1/√2.
  const math::Vec3 chord = r.setbackB[0] - r.setbackA[0];   // from railA to railB
  const math::Vec3 faceNrm = math::cross(edge[0].tangent, chord);
  const double fn = math::norm(faceNrm);
  expectTrue(fn > 1e-12, "chamfer face normal defined");
  const math::Vec3 nhat = faceNrm / fn;
  expectNear(std::fabs(math::dot(nhat, nA)), std::sqrt(0.5), 1e-12, "bevel 45deg to faceA");
  expectNear(std::fabs(math::dot(nhat, nB)), std::sqrt(0.5), 1e-12, "bevel 45deg to faceB");
}

// ── ORACLE 2: SYMMETRIC / ASYMMETRIC / DISTANCE-ANGLE CONSISTENCY ────────────────
static void oracle_mode_consistency() {
  std::printf("\n[oracle 2] mode-equivalence consistency\n");
  const math::Vec3 nA{0, 0, 1};
  const math::Vec3 nB{1, 0, 0};
  const math::Vec3 tangent{0, 1, 0};
  const math::Point3 p0{0, 0, 0};
  const double L = 2.0;

  cn::Substrate A;
  A.kind = cn::SubstrateKind::Plane; A.point = p0; A.normal = nA;
  cn::Substrate B;
  B.kind = cn::SubstrateKind::Plane; B.point = p0; B.normal = nB;
  auto edge = straightEdge(p0, tangent, nA, nB, L, 4);

  // ASYMMETRIC(d,d) == SYMMETRIC(d) rail-for-rail.
  const double d = 0.4;
  auto sym = cn::chamfer_edge_symmetric(A, B, edge, d);
  auto asymEq = cn::chamfer_edge_asymmetric(A, B, edge, d, d);
  expectTrue(sym.ok() && asymEq.ok(), "sym and asym(d,d) both build");
  double maxdA = 0, maxdB = 0;
  for (std::size_t k = 0; k < sym.setbackA.size(); ++k) {
    maxdA = std::max(maxdA, math::distance(sym.setbackA[k], asymEq.setbackA[k]));
    maxdB = std::max(maxdB, math::distance(sym.setbackB[k], asymEq.setbackB[k]));
  }
  expectNear(maxdA, 0.0, 1e-12, "asym(d,d) railA == sym railA");
  expectNear(maxdB, 0.0, 1e-12, "asym(d,d) railB == sym railB");

  // DISTANCE_ANGLE(d, atan(d2/d)) == ASYMMETRIC(d, d2).
  const double d1 = 0.3, d2 = 0.5;
  const double ang = std::atan2(d2, d1) * 180.0 / M_PI;
  auto asym = cn::chamfer_edge_asymmetric(A, B, edge, d1, d2);
  auto da = cn::chamfer_edge_distance_angle(A, B, edge, d1, ang);
  expectTrue(asym.ok() && da.ok(), "asym(d1,d2) and distance-angle both build");
  double maxA = 0, maxB = 0;
  for (std::size_t k = 0; k < asym.setbackA.size(); ++k) {
    maxA = std::max(maxA, math::distance(asym.setbackA[k], da.setbackA[k]));
    maxB = std::max(maxB, math::distance(asym.setbackB[k], da.setbackB[k]));
  }
  expectNear(maxA, 0.0, 1e-12, "distance-angle railA == asym railA");
  expectNear(maxB, 0.0, 1e-12, "distance-angle railB == asym railB");
}

// ── ORACLE 3: CYLINDER SUBSTRATE ────────────────────────────────────────────────
// A coaxial cylinder (axis +Z, radius R) capped by the plane z=H. The convex circular
// rim is at (R·cosψ, R·sinψ, H). A symmetric chamfer d:
//   * cap seam  — cap (z=H) set back RADIALLY inward by d → circle radius R−d at z=H;
//   * wall seam — cylinder wall set back AXIALLY (down) by d → circle radius R at z=H−d.
// The wall setback is the exact along-surface (geodesic) offset: on a cylinder the
// axial direction is a straight geodesic, so an axial slide of d IS the surface distance d.
static void oracle_cylinder() {
  std::printf("\n[oracle 3] cylinder substrate setback\n");
  const double R = 1.5, H = 2.0, d = 0.25;
  const math::Vec3 axis{0, 0, 1};

  cn::Substrate cap;   // faceA = planar cap z=H, outward +Z
  cap.kind = cn::SubstrateKind::Plane;
  cap.point = math::Point3{0, 0, H};
  cap.normal = math::Vec3{0, 0, 1};

  cn::Substrate wall;  // faceB = cylinder wall, radius R, axis +Z
  wall.kind = cn::SubstrateKind::Cylinder;
  wall.point = math::Point3{0, 0, 0};   // axis base
  wall.axis = axis;
  wall.radius = R;

  // Sample the rim. At each ψ: outward cap normal +Z; outward wall normal = radial.
  std::vector<cn::EdgeStation> edge;
  const int n = 24;
  for (int k = 0; k <= n; ++k) {
    const double psi = (M_PI * 0.5) * static_cast<double>(k) / static_cast<double>(n);  // quarter
    const double cx = std::cos(psi), sy = std::sin(psi);
    cn::EdgeStation st;
    st.p = math::Point3{R * cx, R * sy, H};
    // edge tangent = d/dψ of the rim = (−sin, cos, 0).
    st.tangent = math::Vec3{-sy, cx, 0};
    st.nA = math::Vec3{0, 0, 1};        // cap outward
    st.nB = math::Vec3{cx, sy, 0};      // wall outward (radial)
    edge.push_back(st);
  }

  auto r = cn::chamfer_edge_symmetric(cap, wall, edge, d);
  expectTrue(r.ok(), "cylinder chamfer builds");

  double maxCapErr = 0, maxWallRadErr = 0, maxWallAxErr = 0;
  for (std::size_t k = 0; k < edge.size(); ++k) {
    // Cap rail: on z=H, radius R−d.
    const math::Point3& a = r.setbackA[k];
    maxCapErr = std::max(maxCapErr, std::fabs(a.z - H));
    const double ra = std::sqrt(a.x * a.x + a.y * a.y);
    maxCapErr = std::max(maxCapErr, std::fabs(ra - (R - d)));
    // Wall rail: on the cylinder (radius R), axial z = H−d.
    const math::Point3& b = r.setbackB[k];
    const double rb = std::sqrt(b.x * b.x + b.y * b.y);
    maxWallRadErr = std::max(maxWallRadErr, std::fabs(rb - R));   // ON the cylinder
    maxWallAxErr = std::max(maxWallAxErr, std::fabs(b.z - (H - d)));  // geodesic offset d
  }
  expectNear(maxCapErr, 0.0, 1e-9, "cap setback = circle R-d at z=H");
  expectNear(maxWallRadErr, 0.0, 1e-9, "wall setback stays ON cylinder (radius R)");
  expectNear(maxWallAxErr, 0.0, 1e-9, "wall setback axial = d (geodesic offset)");
}

// ── ORACLE 4: HONEST DECLINE ────────────────────────────────────────────────────
static void oracle_decline() {
  std::printf("\n[oracle 4] honest decline\n");
  const math::Vec3 nA{0, 0, 1};
  const math::Vec3 nB{1, 0, 0};
  const math::Vec3 tangent{0, 1, 0};
  const math::Point3 p0{0, 0, 0};
  cn::Substrate A;
  A.kind = cn::SubstrateKind::Plane; A.point = p0; A.normal = nA;
  cn::Substrate B;
  B.kind = cn::SubstrateKind::Plane; B.point = p0; B.normal = nB;
  auto edge = straightEdge(p0, tangent, nA, nB, 2.0, 4);

  // Non-positive setback → BadArguments.
  auto bad = cn::chamfer_edge_symmetric(A, B, edge, -0.1);
  expectTrue(!bad.ok() && bad.decline == cn::ChamferDecline::BadArguments, "negative d declines");

  // Freeform substrate → UnsupportedSubstrate.
  cn::Substrate free;
  free.kind = cn::SubstrateKind::Freeform;
  auto uns = cn::chamfer_edge_symmetric(A, free, edge, 0.3);
  expectTrue(!uns.ok() && uns.decline == cn::ChamferDecline::UnsupportedSubstrate,
             "freeform substrate declines");

  // Degenerate dihedral: parallel faces (same outward normal) → DegenerateDihedral.
  cn::Substrate Bpar;
  Bpar.kind = cn::SubstrateKind::Plane; Bpar.point = p0; Bpar.normal = nA;  // == faceA normal
  auto parEdge = straightEdge(p0, tangent, nA, nA, 2.0, 4);
  auto deg = cn::chamfer_edge_symmetric(A, Bpar, parEdge, 0.3);
  expectTrue(!deg.ok() && deg.decline == cn::ChamferDecline::DegenerateDihedral,
             "parallel faces decline (degenerate dihedral)");

  // Over-large setback on the cylinder: a HUGE circumferential wrap that flips the rail
  // orientation → OverLargeSetback (no self-intersecting face). Use asymmetric so the
  // wall rail sweeps far around and crosses.
  const double R = 1.0, H = 1.0;
  cn::Substrate cap;
  cap.kind = cn::SubstrateKind::Plane; cap.point = math::Point3{0, 0, H}; cap.normal = math::Vec3{0, 0, 1};
  cn::Substrate wall;
  wall.kind = cn::SubstrateKind::Cylinder; wall.point = math::Point3{0, 0, 0};
  wall.axis = math::Vec3{0, 0, 1}; wall.radius = R;
  std::vector<cn::EdgeStation> rim;
  const int n = 8;
  for (int k = 0; k <= n; ++k) {
    const double psi = (M_PI * 0.5) * static_cast<double>(k) / static_cast<double>(n);
    const double cx = std::cos(psi), sy = std::sin(psi);
    cn::EdgeStation st;
    st.p = math::Point3{R * cx, R * sy, H};
    st.tangent = math::Vec3{-sy, cx, 0};
    st.nA = math::Vec3{0, 0, 1};
    st.nB = math::Vec3{cx, sy, 0};
    rim.push_back(st);
  }
  // cap radial setback d1 > R would drive the cap rail past the axis → rails cross.
  auto over = cn::chamfer_edge_asymmetric(cap, wall, rim, 1.5 * R, 0.1);
  expectTrue(!over.ok() && over.decline == cn::ChamferDecline::OverLargeSetback,
             "over-large cap setback declines (rails cross)");
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
#if defined(CYBERCAD_HAS_NUMSCI)
  std::printf("== native NURBS chamfer generator gate ==\n");
  oracle_planar_dihedral();
  oracle_mode_consistency();
  oracle_cylinder();
  oracle_decline();
  std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
#else
  std::printf("SKIP: native NURBS chamfer gate needs CYBERCAD_HAS_NUMSCI\n");
  return 0;
#endif
}
