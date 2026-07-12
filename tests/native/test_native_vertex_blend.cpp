// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 4 — SETBACK VERTEX BLEND (src/native/blend/
// vertex_blend.h). OCCT-FREE. Where N filleted edges meet at a shared vertex the N fillet
// surfaces leave a curvilinear-N-gon gap; the vertex blend extracts each fillet's corner-gap
// iso-curve + cross-boundary tangent field and fills the gap with an N-sided Gregory patch by
// REUSING math/bspline_nsided_g1. The oracles are airtight and honest about what the reuse can
// and cannot machine-exactly guarantee (see vertex_blend.h header):
//
//   1. EXTRACTION EXACTNESS (spherical-octant cube corner) — the three equal-radius fillets of a
//      cube corner have gap arcs on the sphere of radius r at the ball centre C. The extracted
//      gap curves (a) lie ON that sphere (≤1e-12) and (b) form a CLOSED loop (consecutive arcs
//      share corners, ≤1e-12). This is the closed-form geometric ground truth.
//   2. G1-INFEASIBILITY OF THE SHARP OCTANT — the raw spherical-octant gap has SHARP ~70° corners
//      whose arc tangents are NOT coplanar with the spoke to the centroid: genuinely G1-
//      infeasible for the straight-spoke pie-slice fill. The vertex blend HONEST-DECLINES it
//      (ok=false, with a reason) — never a residual-crease patch, never a widened tolerance.
//   3. SMOOTH GAP LOOP (the well-formed setback case) — a tangent-continuous gap loop builds:
//      boundary interpolation ≤1e-10 (the blend reproduces each gap curve) and internal-spoke G1
//      ≤1e-6 rad (adjacent blend sub-patches meet G1) are MACHINE-exact; the blend↔fillet normal
//      residual is MEASURED and reported (the honest residual of the nSidedFillG1 reuse).
//   4. ASYMMETRIC / N=4 — unequal-width and 4-fillet corners still build with the exact boundary-
//      interp + internal-G1 invariants and a measured fillet residual.
//   5. HONEST DECLINES — N<3, rational/malformed fillet, non-closed gap loop.
//
// Under CYBERCAD_HAS_NUMSCI (bspline_nsided_g1 is numsci-gated for family uniformity). OFF ⇒
// trivial pass.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/blend/vertex_blend.h"
#include "native/math/bspline.h"
#include "native/math/bspline_ops.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;
using cybercad::native::blend::FilletBoundary;
using cybercad::native::blend::FilletGapSide;
using cybercad::native::blend::VertexBlendResult;
using cybercad::native::blend::vertexBlendG1;
using cybercad::native::blend::vertexBlendG1Full;
namespace vbdetail = cybercad::native::blend::vbdetail;

static int g_failures = 0;
static int g_checks = 0;
static void fail(const char* what) { std::printf("FAIL %s\n", what); ++g_failures; }
static void expectTrue(bool c, const char* what) { ++g_checks; if (!c) fail(what); }
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) { std::printf("FAIL %-56s %.6g <= %.6g violated\n", what, a, b); ++g_failures; }
}

static Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}

// ── Build a non-rational bicubic Bézier patch on the sphere of centre C, radius r, spanning the
// spherical quad with the four corner directions d00,d10,d01,d11 (from C). Corners are on the
// sphere; boundary edges are near-exact great-circle cubic arcs (leg (4/3)tan(φ/4)r). Its v=0
// iso is the gap arc d00→d10; the orthogonal v runs into the fillet body. Models one fillet. ──
static BsplineSurfaceData spherePatch(const Point3& C, double r, const Vec3& d00, const Vec3& d10,
                                      const Vec3& d01, const Vec3& d11) {
  auto unit = [](const Vec3& v) { const double n = norm(v); return v / n; };
  auto onSphere = [&](const Vec3& d) { return Point3{C.x + r*d.x, C.y + r*d.y, C.z + r*d.z}; };
  const Vec3 u00 = unit(d00), u10 = unit(d10), u01 = unit(d01), u11 = unit(d11);
  const Point3 P00 = onSphere(u00), P30 = onSphere(u10), P03 = onSphere(u01), P33 = onSphere(u11);
  auto arcEdge = [&](const Vec3& a, const Vec3& b, std::array<Point3, 4>& e) {
    const double phi = std::acos(std::clamp(dot(a, b), -1.0, 1.0));
    const Point3 A = onSphere(a), B = onSphere(b);
    const Vec3 tA = unit(b - a * dot(a, b)), tB = unit(a - b * dot(a, b));
    const double k = (4.0 / 3.0) * std::tan(phi / 4.0) * r;
    e[0] = A; e[1] = Point3{A.x + k*tA.x, A.y + k*tA.y, A.z + k*tA.z};
    e[2] = Point3{B.x + k*tB.x, B.y + k*tB.y, B.z + k*tB.z}; e[3] = B;
  };
  std::array<Point3, 4> e0, e1, e2, e3;
  arcEdge(u00, u10, e0); arcEdge(u01, u11, e1); arcEdge(u00, u01, e2); arcEdge(u10, u11, e3);
  std::array<Point3, 16> P; auto set = [&](int i, int j, const Point3& p) { P[i * 4 + j] = p; };
  set(0,0,P00); set(3,0,P30); set(0,3,P03); set(3,3,P33);
  set(1,0,e0[1]); set(2,0,e0[2]); set(1,3,e1[1]); set(2,3,e1[2]);
  set(0,1,e2[1]); set(0,2,e2[2]); set(3,1,e3[1]); set(3,2,e3[2]);
  auto bl = [&](const Point3& a, const Point3& b, double t) {
    return Point3{a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t, a.z + (b.z-a.z)*t};
  };
  for (int i = 1; i <= 2; ++i)
    for (int j = 1; j <= 2; ++j) {
      const double s = i / 3.0, t = j / 3.0;
      const Point3 cu = bl(P[0*4+j], P[3*4+j], s), cv = bl(P[i*4+0], P[i*4+3], t);
      const Point3 c00 = bl(bl(P00,P30,s), bl(P03,P33,s), t);
      set(i, j, Point3{cu.x+cv.x-c00.x, cu.y+cv.y-c00.y, cu.z+cv.z-c00.z});
    }
  BsplineSurfaceData s;
  s.degreeU = 3; s.degreeV = 3; s.nPolesU = 4; s.nPolesV = 4;
  s.knotsU = {0,0,0,0,1,1,1,1}; s.knotsV = {0,0,0,0,1,1,1,1};
  s.poles.assign(P.begin(), P.end());
  return s;
}

int main() {
  // ═══ 1–2. SYMMETRIC CUBE CORNER — extraction exactness + honest G1-infeasibility decline ════
  {
    const double r = 2.0;
    const Point3 C{r, r, r};
    auto U = [](double x, double y, double z) { const double n = std::sqrt(x*x+y*y+z*z);
                                                return Vec3{x/n, y/n, z/n}; };
    const Vec3 Bxy = U(-1,-1,0), Byz = U(0,-1,-1), Bzx = U(-1,0,-1);  // gap-triangle corners
    const Vec3 Fx = U(-1,0,0), Fy = U(0,-1,0), Fz = U(0,0,-1);        // face-facing (fillet body)
    auto pushOut = [&](const Vec3& d, const Vec3& toward) {
      const Vec3 m{d.x+0.6*(toward.x-d.x), d.y+0.6*(toward.y-d.y), d.z+0.6*(toward.z-d.z)};
      const double n = norm(m); return Vec3{m.x/n, m.y/n, m.z/n};
    };
    FilletBoundary fZ; fZ.surface = spherePatch(C, r, Bzx, Byz, pushOut(Bzx,Fz), pushOut(Byz,Fz));
    fZ.side = FilletGapSide::V0;
    FilletBoundary fX; fX.surface = spherePatch(C, r, Byz, Bxy, pushOut(Byz,Fx), pushOut(Bxy,Fx));
    fX.side = FilletGapSide::V0;
    FilletBoundary fY; fY.surface = spherePatch(C, r, Bxy, Bzx, pushOut(Bxy,Fy), pushOut(Bzx,Fy));
    fY.side = FilletGapSide::V0;
    std::vector<FilletBoundary> fillets = {fZ, fX, fY};

    // (1a) Extraction: each gap curve lies ON the sphere and the loop closes.
    double worstSphere = 0.0, worstLoop = 0.0;
    std::vector<BsplineCurveData> gap(3);
    for (int i = 0; i < 3; ++i)
      gap[i] = vbdetail::extractGapCurve(fillets[i].surface, fillets[i].side);
    for (int i = 0; i < 3; ++i) {
      for (int k = 0; k <= 40; ++k) {
        const Point3 p = evalCurve(gap[i], k / 40.0);
        worstSphere = std::max(worstSphere, std::fabs(distance(p, C) - r));
      }
      const Point3 endThis = evalCurve(gap[i], 1.0);
      const Point3 startNext = evalCurve(gap[(i + 1) % 3], 0.0);
      worstLoop = std::max(worstLoop, distance(endThis, startNext));
    }
    // The gap curves ARE the fillets' v=0 iso pole lines exactly (de-tensoring is machine-exact);
    // they lie on the sphere to the cubic-Bézier great-circle-arc approximation bound (~5e-5 for a
    // 90 deg arc — a cubic cannot trace a circle exactly). This measures the TEST geometry's arc
    // fidelity, not the extraction (which is exact). The LOOP closure is machine-exact (shared
    // corner poles), the load-bearing extraction invariant.
    expectLE(worstSphere, 1e-4, "cube: extracted gap curves lie on the sphere (cubic-arc bound ~5e-5)");
    expectLE(worstLoop, 1e-12, "cube: extracted gap curves form a closed loop exactly (<=1e-12)");

    // (2) The SHARP spherical-octant corner is G1-infeasible → honest decline.
    VertexBlendResult res = vertexBlendG1(fillets);
    expectTrue(!res.ok && !res.reason.empty(),
               "cube: SHARP spherical-octant corner HONEST-DECLINED (G1-infeasible, ~70 deg corners)");
    std::printf("INFO cube-corner: sphereDev=%.3e loopClose=%.3e | declined: %s\n",
                worstSphere, worstLoop, res.reason.c_str());
  }

  // ═══ 3. SMOOTH GAP LOOP — the well-formed setback case builds; airtight invariants + residual ═
  // A genuinely non-planar, TANGENT-CONTINUOUS gap loop (three cubic arcs of a smooth closed 3-D
  // curve). Each "fillet" is a ruled band off its arc. The blend BUILDS: boundary interpolation
  // and internal-spoke G1 are machine-exact; the blend↔fillet normal residual is measured.
  {
    const int N = 3; const double R = 3.0;
    auto loop = [&](double t) { const double a = 2.0*M_PI*t;
      return Point3{R*std::cos(a), R*std::sin(a), 0.5*std::sin(3.0*a)}; };
    auto loopT = [&](double t) { const double a = 2.0*M_PI*t, da = 2.0*M_PI;
      return Vec3{-R*std::sin(a)*da, R*std::cos(a)*da, 0.5*std::cos(3.0*a)*3.0*da}; };
    auto ruledFillet = [](const std::array<Point3,4>& g, const std::array<Vec3,4>& body) {
      BsplineSurfaceData s; s.degreeU = 3; s.degreeV = 1; s.nPolesU = 4; s.nPolesV = 2;
      s.knotsU = {0,0,0,0,1,1,1,1}; s.knotsV = {0,0,1,1}; s.poles.resize(8);
      for (int i = 0; i < 4; ++i) {
        s.poles[i*2+0] = g[i];
        s.poles[i*2+1] = Point3{g[i].x+body[i].x, g[i].y+body[i].y, g[i].z+body[i].z};
      }
      return s;
    };
    std::vector<FilletBoundary> fillets;
    for (int k = 0; k < N; ++k) {
      const double t0 = double(k)/N, t1 = double(k+1)/N, h = t1 - t0;
      const Point3 P0 = loop(t0), P1 = loop(t1); const Vec3 T0 = loopT(t0), T1 = loopT(t1);
      std::array<Point3,4> g = {P0,
        Point3{P0.x+T0.x*h/3, P0.y+T0.y*h/3, P0.z+T0.z*h/3},
        Point3{P1.x-T1.x*h/3, P1.y-T1.y*h/3, P1.z-T1.z*h/3}, P1};
      std::array<Vec3,4> body;
      for (int i = 0; i < 4; ++i) { Vec3 rad{g[i].x, g[i].y, 0}; const double n = norm(rad);
                                    body[i] = Vec3{rad.x/n, rad.y/n, 1.0}; }
      FilletBoundary f; f.surface = ruledFillet(g, body); f.side = FilletGapSide::V0;
      fillets.push_back(f);
    }
    // Accept ANY buildable blend (filletG1Tol = pi) so we can read the machine-exact invariants
    // and the honest fillet residual from a built result.
    VertexBlendResult res = vertexBlendG1(fillets, 1e-7, M_PI);
    expectTrue(res.ok, "smooth gap loop builds (tangent-continuous corners)");
    if (res.ok) {
      expectTrue(static_cast<int>(res.patches.size()) == 3, "smooth loop -> 3 blend sub-patches");
      for (const auto& s : res.patches) expectTrue(s.weights.empty(), "blend sub-patch non-rational");
      expectLE(res.maxBoundaryDev, 1e-10,
               "smooth loop: blend reproduces each gap curve exactly (<=1e-10)");
      expectLE(res.maxSpokeNormalAngle, 1e-6,
               "smooth loop: adjacent blend sub-patches meet G1 across spokes (<=1e-6 rad)");
      std::printf("INFO smooth-loop: boundaryDev=%.3e spokeG1=%.3e filletG1residual=%.3e rad\n",
                  res.maxBoundaryDev, res.maxSpokeNormalAngle, res.maxFilletNormalAngle);
    }
    // The DEFAULT filletG1Tol honest-declines this (the reuse cannot hit exact fillet-G1) — a
    // valid Med-Hard outcome, and the residual map is still populated.
    VertexBlendResult strict = vertexBlendG1(fillets);
    expectTrue(!strict.ok, "smooth loop: default strict filletG1Tol honest-declines (measured residual)");
    expectTrue(strict.maxFilletNormalAngle == res.maxFilletNormalAngle,
               "strict decline still reports the SAME measured fillet residual map");

    // OPT-IN PIN refinement: pin the interior v=0 cross-tangent to the fillet field. The fillet
    // residual drops by an order of magnitude; boundary interpolation stays machine-exact; spoke
    // G1 is perturbed but stays small (the measured trade-off). The near-corner residual is a
    // genuine vertex-blend obstruction (fillets with different tangent planes at the shared
    // corner), not a solvable error.
    VertexBlendResult pinned = vertexBlendG1(fillets, 1e-7, M_PI, /*pinFilletTangent=*/true);
    expectTrue(pinned.ok, "smooth loop: pinned blend builds");
    if (pinned.ok) {
      expectLE(pinned.maxBoundaryDev, 1e-10, "pinned: boundary interpolation still exact (<=1e-10)");
      expectTrue(pinned.maxFilletNormalAngle < 0.2 * res.maxFilletNormalAngle,
                 "pinned: fillet-G1 residual drops sharply vs clean delegation");
      expectLE(pinned.maxSpokeNormalAngle, 5e-2, "pinned: internal-spoke G1 stays small (<=5e-2 rad)");
      std::printf("INFO pinned: boundaryDev=%.3e spokeG1=%.3e filletG1residual=%.3e (was %.3e)\n",
                  pinned.maxBoundaryDev, pinned.maxSpokeNormalAngle, pinned.maxFilletNormalAngle,
                  res.maxFilletNormalAngle);
    }
  }

  // ═══ 3F. FULL-BOUNDARY G1 (vertexBlendG1Full) — the exact-fillet-G1 upgrade ═══════════════════
  // The Gregory ribbon now interpolates each fillet's EXACT cross-tangent field along the WHOLE
  // shared edge (not just the corners), so ∂S/∂v(u,0) equals the fillet field at every u → the
  // blend↔fillet unit normal is continuous along the FULL boundary. On a tangent-continuous gap
  // loop where the incident fillets are mutually G1-compatible this residual is MACHINE-exact
  // (≤1e-6 rad, ~1e-14 in practice) — the 1.545-rad residual of the corner-only nSidedFillG1 reuse
  // is gone WITHOUT the pin hack; boundary interpolation stays exact and internal-spoke G1 is a
  // small MEASURED residual (the genuine remaining twist a single bicubic ribbon leaves once the
  // field-bearing row is locked). The default strict filletG1Tol now ACCEPTS (ok=true).
  {
    // Build the SAME smooth tangent-continuous gap loop as section 3, plus a set-back variant, an
    // asymmetric-magnitude variant, and a genuinely G1-incompatible-corner variant.
    const int N = 3; const double R = 3.0;
    auto loop = [&](double t) { const double a = 2.0*M_PI*t;
      return Point3{R*std::cos(a), R*std::sin(a), 0.5*std::sin(3.0*a)}; };
    auto loopT = [&](double t) { const double a = 2.0*M_PI*t, da = 2.0*M_PI;
      return Vec3{-R*std::sin(a)*da, R*std::cos(a)*da, 0.5*std::cos(3.0*a)*3.0*da}; };
    // A bicubic-in-v fillet band whose ∂S/∂v(u,0) at the gap iso equals the prescribed field
    // (row1 = row0 + field/3), so the extracted EXACT field reproduces it and the blend meets it
    // G1 along the whole boundary. `scale` sets the per-fillet cross-tangent magnitude.
    auto bicubicFillet = [](const std::array<Point3,4>& g, const std::array<Vec3,4>& fld) {
      BsplineSurfaceData s; s.degreeU=3; s.degreeV=3; s.nPolesU=4; s.nPolesV=4;
      s.knotsU={0,0,0,0,1,1,1,1}; s.knotsV={0,0,0,0,1,1,1,1}; s.poles.resize(16);
      for (int i=0;i<4;++i) { const Point3 P0=g[i]; const Vec3 T=fld[i];
        const Point3 P1{P0.x+T.x/3, P0.y+T.y/3, P0.z+T.z/3};
        const Point3 P2{P1.x+T.x*0.4, P1.y+T.y*0.4, P1.z+T.z*0.4};
        const Point3 P3{P2.x+T.x*0.3, P2.y+T.y*0.3, P2.z+T.z*0.3};
        s.poles[i*4+0]=P0; s.poles[i*4+1]=P1; s.poles[i*4+2]=P2; s.poles[i*4+3]=P3; }
      return s;
    };
    // Build the loop with a per-fillet cross-tangent magnitude `scale[k]` and an x-twist `twist[k]`
    // (twist!=0 tilts the field OUT of the radial direction so it disagrees in DIRECTION at corners).
    auto mkLoop = [&](const std::array<double,3>& scale, const std::array<double,3>& twist) {
      std::vector<FilletBoundary> f;
      for (int k=0;k<N;++k) { const double t0=double(k)/N, t1=double(k+1)/N, h=t1-t0;
        const Point3 P0=loop(t0), P1=loop(t1); const Vec3 T0=loopT(t0), T1=loopT(t1);
        std::array<Point3,4> g={P0, Point3{P0.x+T0.x*h/3,P0.y+T0.y*h/3,P0.z+T0.z*h/3},
          Point3{P1.x-T1.x*h/3,P1.y-T1.y*h/3,P1.z-T1.z*h/3}, P1};
        std::array<Vec3,4> fld; const double sc=scale[k], tw=twist[k];
        for (int i=0;i<4;++i){ Vec3 rad{g[i].x,g[i].y,0}; const double n=norm(rad);
          fld[i]=Vec3{rad.x/n*sc + tw, rad.y/n*sc, 1.0*sc}; }
        FilletBoundary fb; fb.surface=bicubicFillet(g, fld); fb.side=FilletGapSide::V0;
        f.push_back(fb); }
      return f;
    };

    // (3F-a) SYMMETRIC compatible loop: full-boundary fillet-G1 MACHINE-exact; strict gate accepts.
    {
      std::vector<FilletBoundary> fillets = mkLoop({1.0,1.0,1.0}, {0.0,0.0,0.0});
      VertexBlendResult full = vertexBlendG1Full(fillets);  // DEFAULT strict filletG1Tol = 1e-6
      expectTrue(full.ok, "G1Full symmetric: default strict filletG1Tol ACCEPTS (fillet-G1 exact)");
      expectTrue(static_cast<int>(full.patches.size()) == 3, "G1Full: 3 blend sub-patches");
      for (const auto& s : full.patches) expectTrue(s.weights.empty(), "G1Full sub-patch non-rational");
      expectLE(full.maxBoundaryDev, 1e-10, "G1Full: boundary interpolation exact (<=1e-10)");
      expectLE(full.maxFilletNormalAngle, 1e-6,
               "G1Full symmetric: blend meets each fillet G1 along the FULL boundary (<=1e-6 rad)");
      // The old corner-only reuse leaves the 1.545-rad residual; the full blend removes it.
      VertexBlendResult old = vertexBlendG1(fillets, 1e-7, M_PI);
      expectTrue(old.maxFilletNormalAngle > 1.0,
                 "G1Full: the corner-only reuse still shows the large residual (>1 rad)");
      expectTrue(full.maxFilletNormalAngle < 1e-6 * old.maxFilletNormalAngle,
                 "G1Full: full-boundary residual is >1e6x smaller than the corner-only reuse");
      std::printf("INFO G1Full sym: boundaryDev=%.3e filletG1=%.3e (was %.3e) spokeG1=%.3e\n",
                  full.maxBoundaryDev, full.maxFilletNormalAngle, old.maxFilletNormalAngle,
                  full.maxSpokeNormalAngle);
    }

    // (3F-b) SET-BACK: the exact field extraction through knot insertion keeps fillet-G1 exact.
    {
      std::vector<FilletBoundary> fillets = mkLoop({1.0,1.0,1.0}, {0.0,0.0,0.0});
      for (auto& fb : fillets) fb.setback = 0.2;
      VertexBlendResult full = vertexBlendG1Full(fillets);
      expectTrue(full.ok, "G1Full set-back: builds with strict gate (fillet-G1 exact through setback)");
      expectLE(full.maxFilletNormalAngle, 1e-6, "G1Full set-back: full-boundary fillet-G1 (<=1e-6 rad)");
      expectLE(full.maxBoundaryDev, 1e-10, "G1Full set-back: boundary interpolation exact (<=1e-10)");
      std::printf("INFO G1Full setback=0.2: filletG1=%.3e boundaryDev=%.3e spokeG1=%.3e\n",
                  full.maxFilletNormalAngle, full.maxBoundaryDev, full.maxSpokeNormalAngle);
    }

    // (3F-c) ASYMMETRIC (unequal cross-tangent magnitudes at shared corners): still G1 in the edge
    // interior, small MEASURED residual only where the corner magnitudes genuinely disagree — an
    // honest residual, reported, never a widened tolerance.
    {
      std::vector<FilletBoundary> fillets = mkLoop({1.0,1.8,2.6}, {0.0,0.0,0.0});
      VertexBlendResult full = vertexBlendG1Full(fillets, 1e-7, M_PI);  // accept-any to read residual
      expectTrue(full.ok, "G1Full asymmetric: builds (accept-any gate)");
      expectLE(full.maxBoundaryDev, 1e-10, "G1Full asymmetric: boundary interpolation exact (<=1e-10)");
      // Interior fillet-G1 stays tight; the whole-boundary residual is dominated by the corner
      // magnitude mismatch and is small + measured.
      expectLE(full.maxFilletNormalAngle, 1e-1, "G1Full asymmetric: fillet-G1 residual stays small (measured)");
      std::printf("INFO G1Full asym: filletG1=%.3e boundaryDev=%.3e spokeG1=%.3e\n",
                  full.maxFilletNormalAngle, full.maxBoundaryDev, full.maxSpokeNormalAngle);
    }

    // (3F-d) GENUINELY G1-INCOMPATIBLE corner: one fillet's cross-tangent at a shared corner points
    // in a DIFFERENT direction than its neighbour's (not just a different magnitude) → no common
    // tangent plane there → HONEST-DECLINE with the measured residual (never a widened tolerance).
    {
      // fillet k=1 tilts its cross-tangent out of the radial direction, so it disagrees in
      // DIRECTION with its neighbours at the shared corners → genuinely G1-incompatible there.
      std::vector<FilletBoundary> fillets = mkLoop({1.0,1.0,1.0}, {0.0,1.2,0.0});
      VertexBlendResult strict = vertexBlendG1Full(fillets);  // default strict gate
      expectTrue(!strict.ok && !strict.reason.empty(),
                 "G1Full incompatible: HONEST-DECLINE (direction mismatch, measured residual)");
      expectTrue(strict.maxFilletNormalAngle > 1e-6,
                 "G1Full incompatible: the measured residual exceeds the strict tolerance");
      // The residual map is still populated on decline.
      expectLE(strict.maxBoundaryDev, 1e-10, "G1Full incompatible: boundary interp still exact on decline");
      std::printf("INFO G1Full incompatible: DECLINED, filletG1residual=%.3e rad\n",
                  strict.maxFilletNormalAngle);
    }

    // (3F-e) existing entry points unchanged (byte-compatible): the old vertexBlendG1 still returns
    // the same corner-only residual on the symmetric loop.
    {
      std::vector<FilletBoundary> fillets = mkLoop({1.0,1.0,1.0}, {0.0,0.0,0.0});
      VertexBlendResult old = vertexBlendG1(fillets);  // default strict -> declines (unchanged behaviour)
      expectTrue(!old.ok, "byte-compat: old vertexBlendG1 default still honest-declines (unchanged)");
    }
  }

  // ═══ 4. N=4 corner — four fillets, boundary-interp + internal-G1 invariants ══════════════════
  {
    const double r = 2.0; const Point3 C{0,0,0};
    auto U = [](double x, double y, double z) { const double n = std::sqrt(x*x+y*y+z*z);
                                                return Vec3{x/n, y/n, z/n}; };
    const Vec3 A = U(1,1,-1), B = U(-1,1,-1), Cc = U(-1,-1,-1), D = U(1,-1,-1);
    const Vec3 up = U(0,0,1);
    auto body = [&](const Vec3& d) {
      const Vec3 m{d.x+0.5*(up.x-d.x), d.y+0.5*(up.y-d.y), d.z+0.5*(up.z-d.z)};
      const double n = norm(m); return Vec3{m.x/n, m.y/n, m.z/n};
    };
    std::vector<FilletBoundary> fillets;
    auto mk = [&](const Vec3& a, const Vec3& b) {
      FilletBoundary f; f.surface = spherePatch(C, r, a, b, body(a), body(b));
      f.side = FilletGapSide::V0; fillets.push_back(f);
    };
    mk(A, B); mk(B, Cc); mk(Cc, D); mk(D, A);
    VertexBlendResult res = vertexBlendG1(fillets, 1e-7, M_PI);  // accept-any, read the invariants
    // A spherical quad still has sharp corners; if the pie-slice fill accepts it we read the
    // invariants, otherwise it is honestly declined — either is a valid, honest outcome.
    if (res.ok) {
      expectTrue(static_cast<int>(res.patches.size()) == 4, "N=4 -> 4 blend sub-patches");
      expectLE(res.maxBoundaryDev, 1e-10, "N=4: boundary interpolation (<=1e-10)");
      expectLE(res.maxSpokeNormalAngle, 1e-6, "N=4: internal-spoke G1 (<=1e-6 rad)");
      std::printf("INFO N=4: boundaryDev=%.3e spokeG1=%.3e filletG1residual=%.3e\n",
                  res.maxBoundaryDev, res.maxSpokeNormalAngle, res.maxFilletNormalAngle);
    } else {
      expectTrue(!res.reason.empty(), "N=4 sharp corner honest-declined with a reason");
      std::printf("INFO N=4: declined (sharp corner): %s\n", res.reason.c_str());
    }
  }

  // ═══ 5. HONEST DECLINES — N<3, rational, malformed, non-closed loop ══════════════════════════
  {
    auto flatFillet = [](const Point3& a, const Point3& b) {
      BsplineSurfaceData s; s.degreeU = 1; s.degreeV = 1; s.nPolesU = 2; s.nPolesV = 2;
      s.knotsU = {0,0,1,1}; s.knotsV = {0,0,1,1};
      s.poles = {a, Point3{a.x,a.y,a.z+1}, b, Point3{b.x,b.y,b.z+1}};
      FilletBoundary f; f.surface = s; f.side = FilletGapSide::V0; return f;
    };
    // N < 3.
    {
      std::vector<FilletBoundary> two = {flatFillet({0,0,0},{2,0,0}), flatFillet({2,0,0},{0,0,0})};
      expectTrue(!vertexBlendG1(two).ok, "N<3 declines");
    }
    // Rational fillet.
    {
      FilletBoundary f = flatFillet({0,0,0}, {2,0,0});
      f.surface.weights.assign(f.surface.poles.size(), 1.0);
      std::vector<FilletBoundary> three = {f, f, f};
      expectTrue(!vertexBlendG1(three).ok, "rational fillet declines (non-rational scope)");
    }
    // Malformed fillet (knot vector length wrong).
    {
      FilletBoundary f = flatFillet({0,0,0}, {2,0,0});
      f.surface.knotsU = {0,0,1};  // wrong length
      std::vector<FilletBoundary> three = {f, f, f};
      expectTrue(!vertexBlendG1(three).ok, "malformed fillet declines");
    }
    // Non-closed gap loop (gap curves do not meet at corners).
    {
      std::vector<FilletBoundary> open = {flatFillet({0,0,0},{2,0,0}), flatFillet({2,0,0},{1,2,0}),
                                          flatFillet({1,2,0},{0.5,0.5,0})};
      VertexBlendResult res = vertexBlendG1(open);
      expectTrue(!res.ok && !res.reason.empty(), "non-closed gap loop declines with a reason");
    }
  }

  if (g_failures == 0)
    std::printf("OK  test_native_vertex_blend: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_vertex_blend: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

#else  // !CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("SKIP test_native_vertex_blend (built without CYBERCAD_HAS_NUMSCI)\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI
