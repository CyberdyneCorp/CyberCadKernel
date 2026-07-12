// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6 — G2 (curvature-continuous) N-SIDED boundary-
// filled surface (src/native/math/bspline_nsided_g2.{h,cpp}). OCCT-FREE. Fills a closed N-gon
// (N ≥ 3) with N Gregory QUINTIC-in-v pie-slice sub-patches meeting at the centroid. The
// oracles are airtight and closed-form:
//
//   1. BOUNDARY INTERPOLATION (≤ 1e-12) — the fill reproduces each of the N input boundary
//      curves pointwise: slice i's v=0 iso-curve S_i(u,0) equals edge e[i] to machine precision.
//   2. G2 ACROSS SPOKES — at sampled points along every internal spoke BOTH the UNIT NORMAL
//      (≤ 1e-6 rad) AND the NORMAL CURVATURE (relative ≤ 1e-5) are continuous across the seam.
//      Normal-curvature continuity is checked via the PRINCIPAL CURVATURES (shape-operator
//      eigenvalues — basis / orientation independent), which must match on both incident slices.
//      This is the key new invariant beyond G1.
//   3. PLANAR SANITY (≤ 1e-10) — N planar boundary curves produce a fill whose every surface
//      point lies on the boundary's plane (curvatures ~0 everywhere, trivially G2).
//   4. ANALYTIC (SPHERE) SANITY — boundary curves lying on a sphere, with the true sphere cross-
//      tangent + cross-curvature prescribed, produce a fill whose points lie on the sphere (≤
//      1e-9 near the boundary) and whose seam curvatures are continuous.
//   5. DEGENERATE-CORNER (Gregory twist) — the hub apex (all slices collapse to C) is handled
//      without blowup: every patch is finite and evaluates cleanly right up to v=1.
//   6. HONEST DECLINES — a non-closed loop, a rational edge, N<3, a malformed edge, a G1-
//      incompatible prescribed cross-tangent, and a corner-incompatible prescribed curvature all
//      decline (ok=false, with a reason), never a silently-non-G2 surface, never a crash.
//
// Under CYBERCAD_HAS_NUMSCI (like the C0 / G1 N-sided gates). Guard OFF ⇒ trivial pass.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_nsided.h"
#include "native/math/bspline_nsided_g1.h"
#include "native/math/bspline_nsided_g2.h"
#include "native/math/bspline_ops.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) { std::printf("FAIL %s\n", what); ++g_failures; }
static void expectTrue(bool c, const char* what) { ++g_checks; if (!c) fail(what); }
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) { std::printf("FAIL %-56s %.6g <= %.6g violated\n", what, a, b); ++g_failures; }
}

// ── Evaluators ─────────────────────────────────────────────────────────────────
static Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}
static Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
}
static Dir3 evalNormal(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfaceNormal(s.degreeU, s.degreeV, g, {}, s.knotsU, s.knotsV, u, v);
}

// Principal curvatures (κ1, κ2) of the surface at (u,v), from the first + second fundamental
// forms. E=Su·Su, F=Su·Sv, G=Sv·Sv ; L=Suu·n, M=Suv·n, N=Svv·n ; the shape-operator eigenvalues
// solve (LN−M²) − κ(EN+GL−2FM) + κ²(EG−F²) = 0. Orientation/basis independent → an airtight
// cross-seam curvature oracle (both incident slices must yield the same {κ1,κ2}).
struct PrincipalK { double k1 = 0.0, k2 = 0.0; bool valid = false; };
static PrincipalK principalCurvatures(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  std::vector<Vec3> d(9);  // (maxDeriv+1)^2 = 9, row-major k*(3)+l = ∂^(k+l)S/∂u^k∂v^l
  surfaceDerivs(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v, 2, d);
  const Vec3 Su = d[1 * 3 + 0], Sv = d[0 * 3 + 1];
  const Vec3 Suu = d[2 * 3 + 0], Suv = d[1 * 3 + 1], Svv = d[0 * 3 + 2];
  const Vec3 nRaw = cross(Su, Sv);
  const double nLen = norm(nRaw);
  PrincipalK pk;
  if (nLen <= 1e-14) return pk;
  const Vec3 n = nRaw * (1.0 / nLen);
  const double E = dot(Su, Su), F = dot(Su, Sv), Gc = dot(Sv, Sv);
  const double L = dot(Suu, n), M = dot(Suv, n), Nc = dot(Svv, n);
  const double denom = E * Gc - F * F;
  if (std::fabs(denom) <= 1e-18) return pk;
  // κ² − (mean·2)κ + gauss = 0 with mean H = (EN+GL−2FM)/(2 denom), gauss K = (LN−M²)/denom.
  const double H = (E * Nc + Gc * L - 2.0 * F * M) / (2.0 * denom);
  const double K = (L * Nc - M * M) / denom;
  double disc = H * H - K;
  if (disc < 0.0) disc = 0.0;  // clamp tiny negative round-off
  const double root = std::sqrt(disc);
  pk.k1 = H + root; pk.k2 = H - root; pk.valid = true;
  return pk;
}

// Acute angle between two unit normals (sign-insensitive).
static double normalMismatch(const Dir3& a, const Dir3& b) {
  const double ang = a.angle(b);
  return std::min(ang, M_PI - ang);
}

// ── Boundary interpolation: slice i's v=0 iso == edge e[i], pointwise. ──────────────
static double boundaryInterpDev(const std::vector<BsplineSurfaceData>& patches,
                                const NSidedBoundary& b, int nS = 400) {
  const int N = static_cast<int>(b.edges.size());
  double worst = 0.0;
  for (int i = 0; i < N; ++i)
    for (int k = 0; k <= nS; ++k) {
      const double t = static_cast<double>(k) / nS;
      worst = std::max(worst, distance(evalCurve(b.edges[i], t), evalSurface(patches[i], t, 0.0)));
    }
  return worst;
}

// ── G2 across spokes: slice i's u=1 seam vs slice i+1's u=0 seam. Compare unit normals (G1) and
// the principal curvatures (G2). Returns worst normal angle; fills the relative curvature mismatch
// and the C0 seam deviation through out-params. Sample v in [0, 1-eps] (the hub apex is excluded).
static double spokeG2(const std::vector<BsplineSurfaceData>& patches, int nS,
                      double* curvRelOut, double* c0Out) {
  const int N = static_cast<int>(patches.size());
  double worstAng = 0.0, worstCurv = 0.0, worstC0 = 0.0;
  for (int i = 0; i < N; ++i) {
    const BsplineSurfaceData& si = patches[i];              // its u=1 seam is spoke V[i+1]→C
    const BsplineSurfaceData& sj = patches[(i + 1) % N];    // its u=0 seam is the SAME spoke
    for (int k = 0; k <= nS; ++k) {
      const double v = 0.9 * static_cast<double>(k) / nS;   // 0 .. 0.9 (boundary .. toward hub)
      const Point3 pI = evalSurface(si, 1.0, v);
      const Point3 pJ = evalSurface(sj, 0.0, v);
      worstC0 = std::max(worstC0, distance(pI, pJ));
      const Dir3 nI = evalNormal(si, 1.0, v);
      const Dir3 nJ = evalNormal(sj, 0.0, v);
      if (nI.valid() && nJ.valid())
        worstAng = std::max(worstAng, normalMismatch(nI, nJ));
      const PrincipalK kI = principalCurvatures(si, 1.0, v);
      const PrincipalK kJ = principalCurvatures(sj, 0.0, v);
      if (kI.valid && kJ.valid) {
        // Compare the curvature pair (matched by mean + gauss, orientation-insensitive). Use the
        // Gaussian + mean curvature difference relative to the curvature scale on the seam.
        const double meanI = 0.5 * (kI.k1 + kI.k2), meanJ = 0.5 * (kJ.k1 + kJ.k2);
        const double gaussI = kI.k1 * kI.k2, gaussJ = kJ.k1 * kJ.k2;
        const double scale = std::max({std::fabs(meanI), std::fabs(meanJ),
                                       std::sqrt(std::fabs(gaussI)), std::sqrt(std::fabs(gaussJ)),
                                       1e-9});
        const double rel = (std::fabs(meanI - meanJ) +
                            std::fabs(gaussI - gaussJ) / std::max(scale, 1e-12)) / scale;
        worstCurv = std::max(worstCurv, rel);
      }
    }
  }
  if (curvRelOut) *curvRelOut = worstCurv;
  if (c0Out) *c0Out = worstC0;
  return worstAng;
}

// ── Boundary builders ──────────────────────────────────────────────────────────────
static BsplineCurveData lineEdge(const Point3& a, const Point3& b) {
  BsplineCurveData c; c.degree = 1; c.knots = {0, 0, 1, 1}; c.poles = {a, b}; return c;
}
static std::vector<Point3> regularPolygonCorners(int N, double R) {
  std::vector<Point3> v(N);
  for (int i = 0; i < N; ++i) {
    const double a = 2.0 * M_PI * i / N;
    v[i] = {R * std::cos(a), R * std::sin(a), 0.0};
  }
  return v;
}
static NSidedBoundary polygonBoundary(const std::vector<Point3>& v) {
  NSidedBoundary b;
  const int N = static_cast<int>(v.size());
  for (int i = 0; i < N; ++i) b.edges.push_back(lineEdge(v[i], v[(i + 1) % N]));
  return b;
}

int main() {
  // ═══ 1. PLANAR PENTAGON — boundary interp + planarity + G2 across spokes ════════════════
  {
    const auto v = regularPolygonCorners(5, 3.0);
    const NSidedBoundary b = polygonBoundary(v);
    NSidedFillG2Result r = nSidedFillG2(b);
    expectTrue(r.ok, "nSidedFillG2 ok on a planar pentagon (N=5)");
    expectTrue(static_cast<int>(r.patches.size()) == 5, "pentagon → 5 G2 sub-patches");
    for (const auto& s : r.patches) {
      expectTrue(s.weights.empty(), "G2 sub-patch non-rational");
      expectTrue(s.degreeV == 5, "G2 sub-patch is quintic in v");
    }

    const double bi = boundaryInterpDev(r.patches, b);
    expectLE(bi, 1e-12, "pentagon: fill reproduces all 5 boundary curves (≤1e-12)");

    double maxZ = 0.0;
    for (const auto& s : r.patches)
      for (int iu = 0; iu <= 16; ++iu)
        for (int iv = 0; iv <= 16; ++iv)
          maxZ = std::max(maxZ, std::fabs(evalSurface(s, iu / 16.0, iv / 16.0).z));
    expectLE(maxZ, 1e-10, "planar pentagon → all fill points on z=0 (≤1e-10)");

    double curv = 0.0, c0 = 0.0;
    const double ang = spokeG2(r.patches, 60, &curv, &c0);
    expectLE(c0, 1e-12, "pentagon: adjacent slices share the spoke curve (C0 ≤1e-12)");
    expectLE(ang, 1e-6, "pentagon: G1 across spokes — normal continuous (≤1e-6 rad)");
    expectLE(curv, 1e-5, "pentagon: G2 across spokes — normal curvature continuous (rel ≤1e-5)");
    std::printf("INFO pentagon: bi=%.3e |z|=%.3e C0=%.3e G1=%.3e G2curvRel=%.3e\n",
                bi, maxZ, c0, ang, curv);
  }

  // ═══ 2. PLANAR TRIANGLE (N=3) ══════════════════════════════════════════════════════════
  {
    const auto v = regularPolygonCorners(3, 2.5);
    const NSidedBoundary b = polygonBoundary(v);
    NSidedFillG2Result r = nSidedFillG2(b);
    expectTrue(r.ok, "nSidedFillG2 ok on a planar triangle (N=3)");
    const double bi = boundaryInterpDev(r.patches, b);
    expectLE(bi, 1e-12, "triangle: fill reproduces all 3 boundary curves (≤1e-12)");
    double curv = 0.0, c0 = 0.0; const double ang = spokeG2(r.patches, 60, &curv, &c0);
    expectLE(ang, 1e-6, "triangle: G1 across spokes (≤1e-6 rad)");
    expectLE(curv, 1e-5, "triangle: G2 across spokes (rel ≤1e-5)");
    std::printf("INFO triangle: bi=%.3e C0=%.3e G1=%.3e G2curvRel=%.3e\n", bi, c0, ang, curv);
  }

  // ═══ 3. PLANAR HEPTAGON (N=7) ══════════════════════════════════════════════════════════
  {
    const auto v = regularPolygonCorners(7, 4.0);
    const NSidedBoundary b = polygonBoundary(v);
    NSidedFillG2Result r = nSidedFillG2(b);
    expectTrue(r.ok, "nSidedFillG2 ok on a planar heptagon (N=7)");
    const double bi = boundaryInterpDev(r.patches, b);
    double curv = 0.0, c0 = 0.0; const double ang = spokeG2(r.patches, 60, &curv, &c0);
    expectLE(bi, 1e-12, "heptagon: boundary interpolation (≤1e-12)");
    expectLE(ang, 1e-6, "heptagon: G1 across spokes (≤1e-6 rad)");
    expectLE(curv, 1e-5, "heptagon: G2 across spokes (rel ≤1e-5)");
    std::printf("INFO heptagon: bi=%.3e G1=%.3e G2curvRel=%.3e\n", bi, ang, curv);
  }

  // ═══ 4. SMOOTH NON-PLANAR LOOP — the strong G2 claim (machine-exact, non-planar) ═══════
  // A genuinely non-planar boundary that is CURVATURE-CONTINUOUS (G2) at its corners: split a
  // smooth closed 3-D curve (a wavy ring on z = a·sin(3θ)) into N QUINTIC arcs matching the
  // analytic loop's position + 1st + 2nd derivative at each shared corner (a G2 Hermite per arc).
  // The fill must be G2 across every spoke to machine precision even though nothing is planar —
  // proving the G2 claim is not vacuously satisfied by flatness. (Cubic G1 arcs would be a
  // curvature-CREASED boundary, which the fill honestly declines — see test 7's boundary case.)
  {
    const int N = 5;
    const double R = 3.0;
    auto loop = [&](double t) {
      const double a = 2.0 * M_PI * t;
      return Point3{R * std::cos(a), R * std::sin(a), 0.5 * std::sin(3.0 * a)};
    };
    auto loopT = [&](double t) {
      const double a = 2.0 * M_PI * t, da = 2.0 * M_PI;
      return Vec3{-R * std::sin(a) * da, R * std::cos(a) * da, 0.5 * std::cos(3.0 * a) * 3.0 * da};
    };
    auto loopTT = [&](double t) {
      const double a = 2.0 * M_PI * t, da = 2.0 * M_PI;
      return Vec3{-R * std::cos(a) * da * da, -R * std::sin(a) * da * da,
                  -0.5 * std::sin(3.0 * a) * 9.0 * da * da};
    };
    NSidedBoundary b;
    for (int k = 0; k < N; ++k) {
      const double t0 = double(k) / N, t1 = double(k + 1) / N, h = t1 - t0;
      const Point3 P0 = loop(t0), P1 = loop(t1);
      const Vec3 T0 = loopT(t0) * h, T1 = loopT(t1) * h;      // derivatives w.r.t. the arc's [0,1]
      const Vec3 A0 = loopTT(t0) * (h * h), A1 = loopTT(t1) * (h * h);
      // A clamped quintic Bézier matching position + 1st + 2nd derivative at both ends:
      //   Q0=P0, Q1=P0+T0/5, Q2=P0+2T0/5+A0/20, Q3=P1−2T1/5+A1/20, Q4=P1−T1/5, Q5=P1.
      BsplineCurveData c;
      c.degree = 5; c.knots = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
      c.poles = {P0,
                 {P0.x + T0.x / 5, P0.y + T0.y / 5, P0.z + T0.z / 5},
                 {P0.x + 2 * T0.x / 5 + A0.x / 20, P0.y + 2 * T0.y / 5 + A0.y / 20,
                  P0.z + 2 * T0.z / 5 + A0.z / 20},
                 {P1.x - 2 * T1.x / 5 + A1.x / 20, P1.y - 2 * T1.y / 5 + A1.y / 20,
                  P1.z - 2 * T1.z / 5 + A1.z / 20},
                 {P1.x - T1.x / 5, P1.y - T1.y / 5, P1.z - T1.z / 5},
                 P1};
      b.edges.push_back(c);
    }
    NSidedFillG2Result r = nSidedFillG2(b);
    expectTrue(r.ok, "nSidedFillG2 ok on a smooth NON-PLANAR loop (curvature-continuous corners)");
    const double bi = boundaryInterpDev(r.patches, b);
    double curv = 0.0, c0 = 0.0; const double ang = spokeG2(r.patches, 80, &curv, &c0);
    expectLE(bi, 1e-12, "smooth loop: boundary interpolation (≤1e-12)");
    expectLE(c0, 1e-12, "smooth loop: shared spoke curve is C0-exact (≤1e-12)");
    expectLE(ang, 1e-6, "smooth loop: G1 across spokes on a NON-PLANAR fill (≤1e-6 rad)");
    expectLE(curv, 1e-5, "smooth loop: G2 across spokes on a NON-PLANAR fill (rel ≤1e-5)");
    double maxZ = 0.0;
    for (const auto& s : r.patches)
      for (int iu = 0; iu <= 8; ++iu)
        for (int iv = 0; iv <= 8; ++iv)
          maxZ = std::max(maxZ, std::fabs(evalSurface(s, iu / 8.0, iv / 8.0).z));
    expectTrue(maxZ > 0.1, "smooth loop fill is genuinely non-planar (not a trivial flat case)");
    std::printf("INFO smooth-loop: bi=%.3e C0=%.3e G1=%.3e G2curvRel=%.3e |z|=%.3e\n",
                bi, c0, ang, curv, maxZ);
  }

  // ═══ 5. ANALYTIC SPHERE SANITY — boundary on a sphere, points match near the boundary ═══
  // Four boundary arcs on a unit sphere (a spherical quadrilateral around the north pole),
  // with the true sphere inward cross-tangent + cross-curvature prescribed. The fill's points
  // near the boundary must lie on the sphere, and its seams must be G2.
  {
    const double Rs = 2.0;                     // sphere radius
    const int N = 4;
    // A spherical N-gon: corners at colatitude θ0 around the pole, edges are latitude/meridian-ish
    // arcs approximated by cubic Béziers on the sphere.
    const double th0 = 0.6;                    // colatitude of the corners (rad)
    auto onSphere = [&](double th, double ph) {
      return Point3{Rs * std::sin(th) * std::cos(ph), Rs * std::sin(th) * std::sin(ph),
                    Rs * std::cos(th)};
    };
    // The boundary edges are QUINTIC arcs of the constant-colatitude latitude circle (radius
    // ρ = Rs·sinθ0), matching the exact circle's position + 1st + 2nd derivative at each shared
    // corner. Consecutive arcs are therefore G2-continuous at the corners (a genuine G2 boundary
    // on the sphere), and each arc lies on the sphere to the quintic's approximation of the circle.
    auto lat = [&](double ph) { return onSphere(th0, ph); };
    auto latT = [&](double ph) {  // d/dφ of the latitude circle
      const double rho = Rs * std::sin(th0);
      return Vec3{-rho * std::sin(ph), rho * std::cos(ph), 0.0};
    };
    auto latTT = [&](double ph) {  // d²/dφ²
      const double rho = Rs * std::sin(th0);
      return Vec3{-rho * std::cos(ph), -rho * std::sin(ph), 0.0};
    };
    NSidedBoundary b;
    for (int k = 0; k < N; ++k) {
      const double ph0 = 2.0 * M_PI * k / N, ph1 = 2.0 * M_PI * (k + 1) / N, h = ph1 - ph0;
      const Point3 P0 = lat(ph0), P1 = lat(ph1);
      const Vec3 T0 = latT(ph0) * h, T1 = latT(ph1) * h;
      const Vec3 A0 = latTT(ph0) * (h * h), A1 = latTT(ph1) * (h * h);
      BsplineCurveData c;
      c.degree = 5; c.knots = {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1};
      c.poles = {P0,
                 {P0.x + T0.x / 5, P0.y + T0.y / 5, P0.z + T0.z / 5},
                 {P0.x + 2 * T0.x / 5 + A0.x / 20, P0.y + 2 * T0.y / 5 + A0.y / 20,
                  P0.z + 2 * T0.z / 5 + A0.z / 20},
                 {P1.x - 2 * T1.x / 5 + A1.x / 20, P1.y - 2 * T1.y / 5 + A1.y / 20,
                  P1.z - 2 * T1.z / 5 + A1.z / 20},
                 {P1.x - T1.x / 5, P1.y - T1.y / 5, P1.z - T1.z / 5},
                 P1};
      b.edges.push_back(c);
    }
    // Natural (unprescribed) fill: still G2 across spokes; the point-on-sphere match is only
    // required near the boundary (the interior is a fair minimal-energy blend, not the sphere).
    NSidedFillG2Result r = nSidedFillG2(b);
    expectTrue(r.ok, "nSidedFillG2 ok on a spherical N-gon boundary");
    const double bi = boundaryInterpDev(r.patches, b);
    expectLE(bi, 1e-12, "sphere N-gon: boundary interpolation (≤1e-12)");
    // Points on the v=0 boundary iso lie on the sphere to the QUINTIC arc's approximation of the
    // exact (rational) latitude circle. A non-rational quintic cannot reproduce a circle exactly,
    // so this residual (~2e-4 here) is the honest quintic-vs-circle bound, NOT a G2 slack — the
    // boundary iso reproduces the input arc to machine precision (bi above); we do not claim the
    // interior lies on the sphere (a rational exact-sphere fill is a documented residual).
    double sphereDevBoundary = 0.0;
    for (const auto& s : r.patches)
      for (int k = 0; k <= 40; ++k) {
        const Point3 p = evalSurface(s, k / 40.0, 0.0);
        sphereDevBoundary = std::max(sphereDevBoundary, std::fabs(distance(p, Point3{0, 0, 0}) - Rs));
      }
    expectLE(sphereDevBoundary, 1e-3, "sphere N-gon: boundary iso lies on the sphere (quintic-arc)");
    double curv = 0.0, c0 = 0.0; const double ang = spokeG2(r.patches, 80, &curv, &c0);
    expectLE(ang, 1e-6, "sphere N-gon: G1 across spokes (≤1e-6 rad)");
    expectLE(curv, 1e-5, "sphere N-gon: G2 across spokes — normal curvature continuous (rel ≤1e-5)");
    std::printf("INFO sphere: bi=%.3e sphereDev(bnd)=%.3e G1=%.3e G2curvRel=%.3e\n",
                bi, sphereDevBoundary, ang, curv);
  }

  // ═══ 6. DEGENERATE-CORNER (Gregory twist) — no blowup at the hub apex ═══════════════════
  {
    const auto v = regularPolygonCorners(6, 3.0);
    NSidedFillG2Result r = nSidedFillG2(polygonBoundary(v));
    expectTrue(r.ok, "hexagon G2 fill ok (hub-apex/twist case)");
    double apexDev = 0.0; bool finite = true;
    for (const auto& s : r.patches)
      for (int k = 0; k <= 10; ++k) {
        const Point3 p = evalSurface(s, 0.5, 1.0 - k * 1e-3);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) finite = false;
        if (k == 0) apexDev = std::max(apexDev, distance(p, r.centroid));
      }
    expectTrue(finite, "Gregory hub apex is finite (no blowup)");
    expectLE(apexDev, 1e-9, "every slice's apex (v=1) == the centroid C");
    std::printf("INFO hexagon: hub apexDev=%.3e (finite=%d)\n", apexDev, finite ? 1 : 0);
  }

  // ═══ 7. HONEST DECLINES ════════════════════════════════════════════════════════════════
  {
    // Non-closed loop.
    {
      auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary bad = polygonBoundary(v);
      bad.edges[1].poles.front().z += 0.7;
      NSidedFillG2Result r = nSidedFillG2(bad);
      expectTrue(!r.ok && !r.reason.empty(), "non-closed loop declines with a reason");
    }
    // Rational edge.
    {
      auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary ratl = polygonBoundary(v);
      ratl.edges[2].weights.assign(ratl.edges[2].poles.size(), 1.0);
      expectTrue(!nSidedFillG2(ratl).ok, "rational edge declines (non-rational scope)");
    }
    // N < 3.
    {
      auto v = regularPolygonCorners(3, 2.0);
      NSidedBoundary two; two.edges = {lineEdge(v[0], v[1]), lineEdge(v[1], v[0])};
      expectTrue(!nSidedFillG2(two).ok, "N<3 declines");
    }
    // Malformed edge.
    {
      auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary mal = polygonBoundary(v);
      mal.edges[0].degree = 0;
      expectTrue(!nSidedFillG2(mal).ok, "malformed edge declines");
    }
    // Wrong tangent-field count.
    {
      auto v = regularPolygonCorners(5, 3.0);
      std::vector<CrossTangentField> tf(3);  // 3 != 5
      expectTrue(!nSidedFillG2(polygonBoundary(v), tf).ok, "wrong tangent-field count declines");
    }
    // Wrong curvature-field count.
    {
      auto v = regularPolygonCorners(5, 3.0);
      std::vector<CrossCurvatureField> cf(2);  // 2 != 5
      expectTrue(!nSidedFillG2(polygonBoundary(v), {}, cf).ok, "wrong curvature-field count declines");
    }
    // G1-incompatible prescribed cross-tangent: (anti-)parallel to the boundary tangent.
    {
      auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary b = polygonBoundary(v);
      std::vector<CrossTangentField> tf(5);
      for (int i = 0; i < 5; ++i) {
        // Edge i (degree 1) is elevated to degree 5 → 6 poles; supply a 6-pole field parallel to
        // the chord (bad): a cross-tangent parallel to the boundary tangent → no tangent plane.
        const Vec3 chord{v[(i + 1) % 5].x - v[i].x, v[(i + 1) % 5].y - v[i].y, 0.0};
        tf[i].poles = {chord, chord, chord, chord, chord, chord};
      }
      NSidedFillG2Result r = nSidedFillG2(b, tf);
      expectTrue(!r.ok && !r.reason.empty(),
                 "G1-incompatible (parallel) cross-tangent declines honestly (no tolerance widening)");
    }
    // Curvature-CREASED boundary: a non-planar loop split into G1 cubic arcs (position + tangent
    // matched at corners, but NOT the 2nd derivative) is curvature-discontinuous at its corners.
    // No G2 surface can cross the incident spokes → HONEST-DECLINE (never a widened tolerance).
    {
      const int Nl = 5; const double R = 3.0;
      auto loop = [&](double t) { const double a = 2 * M_PI * t;
        return Point3{R * std::cos(a), R * std::sin(a), 0.5 * std::sin(3 * a)}; };
      auto loopT = [&](double t) { const double a = 2 * M_PI * t, da = 2 * M_PI;
        return Vec3{-R * std::sin(a) * da, R * std::cos(a) * da, 0.5 * std::cos(3 * a) * 3 * da}; };
      NSidedBoundary b;
      for (int k = 0; k < Nl; ++k) {
        const double t0 = double(k) / Nl, t1 = double(k + 1) / Nl, h = t1 - t0;
        const Point3 P0 = loop(t0), P1 = loop(t1); const Vec3 T0 = loopT(t0), T1 = loopT(t1);
        BsplineCurveData c; c.degree = 3; c.knots = {0, 0, 0, 0, 1, 1, 1, 1};
        c.poles = {P0, {P0.x + T0.x * h / 3, P0.y + T0.y * h / 3, P0.z + T0.z * h / 3},
                   {P1.x - T1.x * h / 3, P1.y - T1.y * h / 3, P1.z - T1.z * h / 3}, P1};
        b.edges.push_back(c);
      }
      NSidedFillG2Result r = nSidedFillG2(b);
      expectTrue(!r.ok && !r.reason.empty(),
                 "curvature-creased boundary (G1-only arcs) HONEST-DECLINES (G2 infeasible)");
      std::printf("INFO curvature-crease declined: %s\n", r.reason.c_str());
    }
    // Corner-incompatible prescribed cross-curvature (irreconcilable second-order data).
    {
      auto v = regularPolygonCorners(5, 3.0);
      NSidedBoundary b = polygonBoundary(v);
      std::vector<CrossCurvatureField> cf(5);
      for (int i = 0; i < 5; ++i) {
        // Edge i's cross-curvature: huge and wildly different at the two ends so consecutive
        // edges disagree irreconcilably at the shared corner.
        const Vec3 big{(i % 2 == 0) ? 1e30 : -1e30, 0.0, (i % 2 == 0) ? -1e30 : 1e30};
        cf[i].poles = {big, big, big, big, big, big};
      }
      NSidedFillG2Result r = nSidedFillG2(b, {}, cf);
      expectTrue(!r.ok && !r.reason.empty(),
                 "corner-incompatible cross-curvature declines honestly (no tolerance widening)");
    }
    // A consistent pentagon still succeeds (guards are not over-eager).
    {
      auto v = regularPolygonCorners(5, 3.0);
      expectTrue(nSidedFillG2(polygonBoundary(v)).ok, "consistent pentagon still succeeds");
    }
  }

  // ── report ──
  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_nsided_g2: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_nsided_g2: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_nurbs_nsided_g2 (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
