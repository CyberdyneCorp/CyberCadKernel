// SPDX-License-Identifier: Apache-2.0
//
// native_math_parity.mm — native-math vs OCCT-oracle parity harness (iOS sim).
//
// Phase 4 capability #1 (`native-math`), simulator verification gate 2 (see
// openspec/NATIVE-REWRITE.md). The native math library (src/native/math/*, an
// OCCT-FREE clean-room implementation from *The NURBS Book* + gp_* conventions)
// is compared, for many sampled inputs, against the real OCCT algorithms it was
// modelled on:
//
//   * Transform     — native Transform  vs  gp_Trsf
//   * B-spline curve — native curvePoint/curveDerivs  vs  Geom_BSplineCurve D0/D1/D2
//   * NURBS curve   — native nurbsCurve*  vs  rational Geom_BSplineCurve
//   * B-spline surf — native surfacePoint/derivs/normal  vs  Geom_BSplineSurface
//   * NURBS surf    — native nurbsSurface*  vs  rational Geom_BSplineSurface
//   * Elementary    — native Plane/Cylinder/Cone/Sphere  vs  ElSLib
//
// The native side and the OCCT side are built from IDENTICAL data (same poles,
// flat knots, weights, placements) so any divergence is a numeric/algorithm
// mismatch, not a modelling difference. Agreement is required to a tight fp64
// tolerance (absolute 1e-9, relative for large magnitudes).
//
// This file is OCCT-DEPENDENT (it links the oracle) and lives under tests/sim.
// It is compiled ONLY by scripts/run-sim-native-math.sh and is on the SKIP list
// of run-sim-suite.sh. The native math sources it exercises remain OCCT-free.
//
// Build (see scripts/run-sim-native-math.sh):
//   -DCYBERCAD_HAS_OCCT  -std=c++20  for arm64-apple-ios-simulator, linking
//   TKMath/TKG3d/TKGeomBase/TKG2d/TKernel from the simulator OCCT install.
//
// Output: one [NMATH] line per group with the max observed error, then a final
// "== N passed, M failed ==". Flushes and std::_Exit (OCCT static teardown in
// the trimmed static build is not exit-clean — same rationale as parity_bench).

#include "native/math/native_math.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_math_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

// ── OCCT oracle headers ──────────────────────────────────────────────────────
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Trsf.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array2OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_BSplineSurface.hxx>
#include <ElSLib.hxx>

namespace nm = cybercad::native::math;

// ═════════════════════════════════════════════════════════════════════════════
// Result accounting.
// ═════════════════════════════════════════════════════════════════════════════
static int g_pass = 0;
static int g_fail = 0;

// One group = one geometric primitive. Accumulates the worst error seen across
// all sampled inputs and reports a single PASS/FAIL line for the group.
struct Group {
  const char* name;
  double tol;
  double maxErr = 0.0;
  long samples = 0;
  const char* worstLabel = "";

  explicit Group(const char* n, double t) : name(n), tol(t) {}

  // Record one comparison. `err` is already the error metric (abs or relative).
  void observe(double err, const char* label = "") {
    ++samples;
    if (err > maxErr) { maxErr = err; worstLabel = label; }
  }

  void report() {
    const bool ok = maxErr <= tol;
    std::printf("[NMATH] %-22s %s  maxErr=%.3e  tol=%.1e  (%ld samples)%s%s\n",
                name, ok ? "PASS" : "FAIL", maxErr, tol, samples,
                (!ok && worstLabel[0]) ? "  worst=" : "",
                (!ok) ? worstLabel : "");
    if (ok) ++g_pass; else ++g_fail;
    std::fflush(stdout);
  }
};

// Error metric: absolute for small magnitudes, relative once |ref| is large, so
// that a 1e-9 absolute tolerance is not unfairly strict on coordinates in the
// thousands. mag ~= the reference magnitude the error is measured against.
static double relErr(double got, double ref) {
  const double a = std::fabs(got - ref);
  const double m = std::fabs(ref);
  return (m > 1.0) ? a / m : a;
}
static double ptErr(const nm::Point3& p, const gp_Pnt& q) {
  return std::max({relErr(p.x, q.X()), relErr(p.y, q.Y()), relErr(p.z, q.Z())});
}
static double vecErr(const nm::Vec3& v, const gp_Vec& w) {
  return std::max({relErr(v.x, w.X()), relErr(v.y, w.Y()), relErr(v.z, w.Z())});
}
static double dirErr(const nm::Dir3& d, const gp_Dir& e) {
  // Normal orientation may differ in sign between the two derivations; compare
  // as an unsigned direction (the surfaces are identical, only the outward
  // convention could flip). Error = distance to the nearer of ±e.
  const double s = d.x() * e.X() + d.y() * e.Y() + d.z() * e.Z();
  const double sign = (s < 0.0) ? -1.0 : 1.0;
  return std::max({std::fabs(d.x() - sign * e.X()),
                   std::fabs(d.y() - sign * e.Y()),
                   std::fabs(d.z() - sign * e.Z())});
}

// ═════════════════════════════════════════════════════════════════════════════
// Helpers: convert native flat data → OCCT (poles / knots+mults / weights).
// ═════════════════════════════════════════════════════════════════════════════

// Collapse a FLAT knot vector (knots repeated by multiplicity) into OCCT's
// (distinct knots, multiplicities) representation.
static void flatToKnotsMults(const std::vector<double>& flat,
                             std::vector<double>& knots,
                             std::vector<int>& mults) {
  knots.clear();
  mults.clear();
  for (double k : flat) {
    if (!knots.empty() && std::fabs(k - knots.back()) < 1e-15) {
      ++mults.back();
    } else {
      knots.push_back(k);
      mults.push_back(1);
    }
  }
}

static TColStd_Array1OfReal toReal1(const std::vector<double>& v) {
  TColStd_Array1OfReal a(1, static_cast<int>(v.size()));
  for (int i = 0; i < static_cast<int>(v.size()); ++i) a.SetValue(i + 1, v[i]);
  return a;
}
static TColStd_Array1OfInteger toInt1(const std::vector<int>& v) {
  TColStd_Array1OfInteger a(1, static_cast<int>(v.size()));
  for (int i = 0; i < static_cast<int>(v.size()); ++i) a.SetValue(i + 1, v[i]);
  return a;
}
static TColgp_Array1OfPnt toPnt1(const std::vector<nm::Point3>& v) {
  TColgp_Array1OfPnt a(1, static_cast<int>(v.size()));
  for (int i = 0; i < static_cast<int>(v.size()); ++i)
    a.SetValue(i + 1, gp_Pnt(v[i].x, v[i].y, v[i].z));
  return a;
}

// Build an OCCT Geom_BSplineCurve (rational if weights non-empty) from native data.
static Handle(Geom_BSplineCurve) buildCurve(int degree,
                                            const std::vector<nm::Point3>& poles,
                                            const std::vector<double>& flatKnots,
                                            const std::vector<double>& weights) {
  std::vector<double> knots;
  std::vector<int> mults;
  flatToKnotsMults(flatKnots, knots, mults);
  TColgp_Array1OfPnt occPoles = toPnt1(poles);
  TColStd_Array1OfReal occKnots = toReal1(knots);
  TColStd_Array1OfInteger occMults = toInt1(mults);
  if (weights.empty()) {
    return new Geom_BSplineCurve(occPoles, occKnots, occMults, degree, Standard_False);
  }
  TColStd_Array1OfReal occW = toReal1(weights);
  return new Geom_BSplineCurve(occPoles, occW, occKnots, occMults, degree, Standard_False);
}

// ═════════════════════════════════════════════════════════════════════════════
// GROUP 1 — Transform vs gp_Trsf.
//
// gp_Trsf represents rigid motions + uniform scale + mirror as a decomposed
// scale·rotation·translation (SetValues orthogonalizes, so it does NOT model a
// general affine losslessly — non-uniform scale is therefore excluded here and
// verified only by the host unit tests). We drive BOTH sides from the same
// semantic primitives so the comparison is exact.
// ═════════════════════════════════════════════════════════════════════════════
static void groupTransform(std::mt19937_64& rng) {
  Group gPt("transform-point", 1e-9);
  Group gVec("transform-vector", 1e-9);
  Group gDir("transform-dir", 1e-9);

  std::uniform_real_distribution<double> coord(-100.0, 100.0);
  std::uniform_real_distribution<double> ang(-3.14159, 3.14159);
  std::uniform_real_distribution<double> scl(0.2, 5.0);
  std::uniform_int_distribution<int> kind(0, 3);  // 0 trans 1 rot 2 uscale 3 mirror

  for (int iter = 0; iter < 400; ++iter) {
    nm::Transform nt;
    gp_Trsf ot;

    switch (kind(rng)) {
      case 0: {  // translation
        nm::Vec3 t{coord(rng), coord(rng), coord(rng)};
        nt = nm::Transform::translationOf(t);
        ot.SetTranslation(gp_Vec(t.x, t.y, t.z));
        break;
      }
      case 1: {  // rotation about a random axis through a random center
        nm::Point3 c{coord(rng), coord(rng), coord(rng)};
        nm::Dir3 ax{coord(rng), coord(rng), coord(rng)};
        if (!ax.valid()) ax = nm::Dir3{0, 0, 1};
        const double a = ang(rng);
        nt = nm::Transform::rotationOf(c, ax, a);
        ot.SetRotation(gp_Ax1(gp_Pnt(c.x, c.y, c.z), gp_Dir(ax.x(), ax.y(), ax.z())), a);
        break;
      }
      case 2: {  // uniform scale about a random center
        nm::Point3 c{coord(rng), coord(rng), coord(rng)};
        const double s = scl(rng);
        nt = nm::Transform::scaleOf(c, s);
        ot.SetScale(gp_Pnt(c.x, c.y, c.z), s);
        break;
      }
      default: {  // mirror about a point
        nm::Point3 c{coord(rng), coord(rng), coord(rng)};
        // gp_Trsf point-mirror == uniform scale by -1 about the point.
        nt = nm::Transform::scaleOf(c, -1.0);
        ot.SetMirror(gp_Pnt(c.x, c.y, c.z));
        break;
      }
    }

    // Points, free vectors and directions through the same map.
    for (int k = 0; k < 4; ++k) {
      nm::Point3 p{coord(rng), coord(rng), coord(rng)};
      gp_Pnt op(p.x, p.y, p.z);
      op.Transform(ot);
      gPt.observe(ptErr(nt.applyToPoint(p), op), "point");

      nm::Vec3 v{coord(rng), coord(rng), coord(rng)};
      gp_Vec ov(v.x, v.y, v.z);
      ov.Transform(ot);
      gVec.observe(vecErr(nt.applyToVector(v), ov), "vector");

      nm::Dir3 d{coord(rng), coord(rng), coord(rng)};
      if (d.valid()) {
        gp_Dir od(d.x(), d.y(), d.z());
        od.Transform(ot);
        gDir.observe(dirErr(nt.applyToDir(d), od), "dir");
      }
    }
  }
  gPt.report();
  gVec.report();
  gDir.report();
}

// ═════════════════════════════════════════════════════════════════════════════
// GROUP 2 — B-spline curve (non-rational) vs Geom_BSplineCurve D0/D1/D2.
// ═════════════════════════════════════════════════════════════════════════════
static void sampleCurveGroup(std::mt19937_64& rng, bool rational) {
  const char* nP = rational ? "nurbs-curve-point" : "bspline-curve-point";
  const char* nD1 = rational ? "nurbs-curve-D1" : "bspline-curve-D1";
  const char* nD2 = rational ? "nurbs-curve-D2" : "bspline-curve-D2";
  Group gP(nP, 1e-9), gD1(nD1, 1e-8), gD2(nD2, 1e-7);

  std::uniform_real_distribution<double> coord(-20.0, 20.0);
  std::uniform_real_distribution<double> wdist(0.3, 3.0);
  std::uniform_int_distribution<int> degDist(1, 3);

  for (int iter = 0; iter < 120; ++iter) {
    const int degree = degDist(rng);
    // Clamped curve: interior span count 1..3 → n+1 = degree + 1 + interiorSpans.
    std::uniform_int_distribution<int> spanDist(1, 3);
    const int interior = spanDist(rng);
    const int nPoles = degree + interior;  // #control points
    if (nPoles < degree + 1) continue;

    std::vector<nm::Point3> poles(nPoles);
    for (auto& p : poles) p = nm::Point3{coord(rng), coord(rng), coord(rng)};

    std::vector<double> weights;
    if (rational) {
      weights.resize(nPoles);
      for (auto& w : weights) w = wdist(rng);
    }

    // Clamped flat knot vector on [0,1]: (degree+1) zeros, interior-1 interior
    // knots evenly spaced, (degree+1) ones. Length = nPoles + degree + 1.
    std::vector<double> flat;
    for (int i = 0; i <= degree; ++i) flat.push_back(0.0);
    for (int i = 1; i < interior; ++i) flat.push_back(double(i) / interior);
    for (int i = 0; i <= degree; ++i) flat.push_back(1.0);

    Handle(Geom_BSplineCurve) oc = buildCurve(degree, poles, flat, weights);

    for (int s = 0; s <= 10; ++s) {
      const double u = double(s) / 10.0;  // domain is exactly [0,1]

      // Point.
      nm::Point3 np = rational ? nm::nurbsCurvePoint(degree, poles, weights, flat, u)
                               : nm::curvePoint(degree, poles, flat, u);
      gp_Pnt op;
      oc->D0(u, op);
      gP.observe(ptErr(np, op), "curvePoint");

      // Derivatives 0..2.
      std::vector<nm::Vec3> nd(3);
      if (rational)
        nm::nurbsCurveDerivs(degree, poles, weights, flat, u, 2, nd);
      else
        nm::curveDerivs(degree, poles, flat, u, 2, nd);
      gp_Pnt p2;
      gp_Vec v1, v2;
      oc->D2(u, p2, v1, v2);
      gD1.observe(vecErr(nd[1], v1), "D1");
      gD2.observe(vecErr(nd[2], v2), "D2");
    }
  }
  gP.report();
  gD1.report();
  gD2.report();
}

// ═════════════════════════════════════════════════════════════════════════════
// GROUP 3 — B-spline / NURBS surface vs Geom_BSplineSurface D0/D1 + normal.
// ═════════════════════════════════════════════════════════════════════════════
static void sampleSurfaceGroup(std::mt19937_64& rng, bool rational) {
  const char* nP = rational ? "nurbs-surface-point" : "bspline-surface-point";
  const char* nDu = rational ? "nurbs-surface-dU" : "bspline-surface-dU";
  const char* nDv = rational ? "nurbs-surface-dV" : "bspline-surface-dV";
  const char* nN = rational ? "nurbs-surface-normal" : "bspline-surface-normal";
  Group gP(nP, 1e-9), gDu(nDu, 1e-8), gDv(nDv, 1e-8), gN(nN, 1e-8);

  std::uniform_real_distribution<double> coord(-15.0, 15.0);
  std::uniform_real_distribution<double> wdist(0.4, 2.5);
  std::uniform_int_distribution<int> degDist(1, 3);

  auto clampedFlat = [](int degree, int interior) {
    std::vector<double> flat;
    for (int i = 0; i <= degree; ++i) flat.push_back(0.0);
    for (int i = 1; i < interior; ++i) flat.push_back(double(i) / interior);
    for (int i = 0; i <= degree; ++i) flat.push_back(1.0);
    return flat;
  };

  for (int iter = 0; iter < 80; ++iter) {
    const int du = degDist(rng), dv = degDist(rng);
    std::uniform_int_distribution<int> spanDist(1, 2);
    const int iu = spanDist(rng), iv = spanDist(rng);
    const int nU = du + iu, nV = dv + iv;  // poles in each direction

    std::vector<nm::Point3> poles(static_cast<std::size_t>(nU) * nV);
    for (int i = 0; i < nU; ++i)
      for (int j = 0; j < nV; ++j)
        poles[static_cast<std::size_t>(i) * nV + j] =
            nm::Point3{coord(rng), coord(rng), coord(rng)};

    std::vector<double> weights;
    if (rational) {
      weights.resize(poles.size());
      for (auto& w : weights) w = wdist(rng);
    }

    std::vector<double> flatU = clampedFlat(du, iu);
    std::vector<double> flatV = clampedFlat(dv, iv);
    nm::SurfaceGrid grid{poles, nU, nV};

    // OCCT surface: TColgp_Array2OfPnt is (row=U 1..nU, col=V 1..nV).
    std::vector<double> ku, kv;
    std::vector<int> mu, mv;
    flatToKnotsMults(flatU, ku, mu);
    flatToKnotsMults(flatV, kv, mv);
    TColgp_Array2OfPnt occPoles(1, nU, 1, nV);
    for (int i = 0; i < nU; ++i)
      for (int j = 0; j < nV; ++j) {
        const nm::Point3& p = poles[static_cast<std::size_t>(i) * nV + j];
        occPoles.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
      }
    TColStd_Array1OfReal occKu = toReal1(ku), occKv = toReal1(kv);
    TColStd_Array1OfInteger occMu = toInt1(mu), occMv = toInt1(mv);

    Handle(Geom_BSplineSurface) os;
    if (rational) {
      TColStd_Array2OfReal occW(1, nU, 1, nV);
      for (int i = 0; i < nU; ++i)
        for (int j = 0; j < nV; ++j)
          occW.SetValue(i + 1, j + 1, weights[static_cast<std::size_t>(i) * nV + j]);
      os = new Geom_BSplineSurface(occPoles, occW, occKu, occKv, occMu, occMv, du, dv,
                                   Standard_False, Standard_False);
    } else {
      os = new Geom_BSplineSurface(occPoles, occKu, occKv, occMu, occMv, du, dv,
                                   Standard_False, Standard_False);
    }

    for (int su = 0; su <= 5; ++su) {
      for (int sv = 0; sv <= 5; ++sv) {
        const double u = double(su) / 5.0, v = double(sv) / 5.0;

        // Point.
        nm::Point3 np = rational
            ? nm::nurbsSurfacePoint(du, dv, grid, weights, flatU, flatV, u, v)
            : nm::surfacePoint(du, dv, grid, flatU, flatV, u, v);
        gp_Pnt op;
        gp_Vec ov1u, ov1v;
        os->D1(u, v, op, ov1u, ov1v);
        gP.observe(ptErr(np, op), "surfacePoint");

        // First derivatives ∂/∂u, ∂/∂v. Layout: out[k*(maxDeriv+1)+l].
        const int md = 1;
        std::vector<nm::Vec3> nd(static_cast<std::size_t>((md + 1) * (md + 1)));
        if (rational)
          nm::nurbsSurfaceDerivs(du, dv, grid, weights, flatU, flatV, u, v, md, nd);
        else
          nm::surfaceDerivs(du, dv, grid, flatU, flatV, u, v, md, nd);
        const nm::Vec3 dU = nd[1 * (md + 1) + 0];  // ∂/∂u
        const nm::Vec3 dV = nd[0 * (md + 1) + 1];  // ∂/∂v
        gDu.observe(vecErr(dU, ov1u), "dU");
        gDv.observe(vecErr(dV, ov1v), "dV");

        // Unit normal (compare as unsigned direction against ov1u×ov1v).
        nm::Dir3 nn = nm::surfaceNormal(du, dv, grid, weights, flatU, flatV, u, v);
        gp_Vec ocN = ov1u.Crossed(ov1v);
        if (ocN.Magnitude() > 1e-9) {
          gp_Dir od(ocN);
          gN.observe(dirErr(nn, od), "normal");
        }
      }
    }
  }
  gP.report();
  gDu.report();
  gDv.report();
  gN.report();
}

// ═════════════════════════════════════════════════════════════════════════════
// GROUP 4 — Elementary surfaces vs ElSLib.
//
// Native Ax3 (origin + orthonormal X,Y,Z) maps directly onto gp_Ax3
// (Location, XDirection, Direction). Both sides use the SAME frame, radius and
// (u,v), and the parametrizations are verified identical against ElSLib.cxx.
// ═════════════════════════════════════════════════════════════════════════════
static void groupElementary(std::mt19937_64& rng) {
  Group gPlane("elem-plane", 1e-9);
  Group gCyl("elem-cylinder", 1e-9);
  Group gCylN("elem-cylinder-normal", 1e-9);
  Group gCone("elem-cone", 1e-9);
  Group gConeN("elem-cone-normal", 1e-8);
  Group gSph("elem-sphere", 1e-9);
  Group gSphN("elem-sphere-normal", 1e-9);

  std::uniform_real_distribution<double> coord(-30.0, 30.0);
  std::uniform_real_distribution<double> rad(0.5, 12.0);
  std::uniform_real_distribution<double> uu(0.0, 6.28318530717958648);
  std::uniform_real_distribution<double> vv(-3.0, 3.0);
  std::uniform_real_distribution<double> lat(-1.4, 1.4);  // sphere latitude
  std::uniform_real_distribution<double> ang(0.05, 1.2);  // cone semi-angle

  for (int iter = 0; iter < 300; ++iter) {
    // Random right-handed frame.
    nm::Point3 o{coord(rng), coord(rng), coord(rng)};
    nm::Dir3 axis{coord(rng), coord(rng), coord(rng)};
    if (!axis.valid()) axis = nm::Dir3{0, 0, 1};
    nm::Dir3 xref{coord(rng), coord(rng), coord(rng)};
    if (!xref.valid()) xref = nm::Dir3{1, 0, 0};
    nm::Ax3 frame = nm::Ax3::fromAxisAndRef(o, axis, xref);

    // Matching gp_Ax3 built from the SAME orthonormal directions.
    gp_Ax3 opos(gp_Pnt(o.x, o.y, o.z),
                gp_Dir(frame.z.x(), frame.z.y(), frame.z.z()),
                gp_Dir(frame.x.x(), frame.x.y(), frame.x.z()));

    const double R = rad(rng);
    const double u = uu(rng);

    // Plane.
    {
      const double a = coord(rng), b = coord(rng);
      nm::Plane pl{frame};
      gp_Pnt op = ElSLib::PlaneValue(a, b, opos);
      gPlane.observe(ptErr(pl.value(a, b), op), "plane");
    }
    // Cylinder + normal.
    {
      const double v = vv(rng);
      nm::Cylinder cy{frame, R};
      gp_Pnt op = ElSLib::CylinderValue(u, v, opos, R);
      gCyl.observe(ptErr(cy.value(u, v), op), "cylinder");
      gp_Vec d1u, d1v;
      gp_Pnt tmp;
      ElSLib::CylinderD1(u, v, opos, R, tmp, d1u, d1v);
      gp_Vec on = d1u.Crossed(d1v);
      if (on.Magnitude() > 1e-9) gCylN.observe(dirErr(cy.normal(u, v), gp_Dir(on)), "cyl-normal");
    }
    // Cone + normal.
    {
      const double v = vv(rng);
      const double sa = ang(rng);
      nm::Cone co{frame, R, sa};
      gp_Pnt op = ElSLib::ConeValue(u, v, opos, R, sa);
      gCone.observe(ptErr(co.value(u, v), op), "cone");
      gp_Vec d1u, d1v;
      gp_Pnt tmp;
      ElSLib::ConeD1(u, v, opos, R, sa, tmp, d1u, d1v);
      gp_Vec on = d1u.Crossed(d1v);
      if (on.Magnitude() > 1e-9) gConeN.observe(dirErr(co.normal(u, v), gp_Dir(on)), "cone-normal");
    }
    // Sphere + normal.
    {
      const double v = lat(rng);
      nm::Sphere sp{frame, R};
      gp_Pnt op = ElSLib::SphereValue(u, v, opos, R);
      gSph.observe(ptErr(sp.value(u, v), op), "sphere");
      gp_Vec d1u, d1v;
      gp_Pnt tmp;
      ElSLib::SphereD1(u, v, opos, R, tmp, d1u, d1v);
      gp_Vec on = d1u.Crossed(d1v);
      if (on.Magnitude() > 1e-9) gSphN.observe(dirErr(sp.normal(u, v), gp_Dir(on)), "sph-normal");
    }
  }
  gPlane.report();
  gCyl.report();
  gCylN.report();
  gCone.report();
  gConeN.report();
  gSph.report();
  gSphN.report();
}

int main() {
  std::printf("== native-math vs OCCT-oracle parity ==\n");
  std::fflush(stdout);

  std::mt19937_64 rng(0xC0FFEEu);  // fixed seed → deterministic sampling

  groupTransform(rng);
  sampleCurveGroup(rng, /*rational=*/false);
  sampleCurveGroup(rng, /*rational=*/true);
  sampleSurfaceGroup(rng, /*rational=*/false);
  sampleSurfaceGroup(rng, /*rational=*/true);
  groupElementary(rng);

  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);

  // Same rationale as parity_bench.cpp: OCCT's static teardown in the trimmed
  // static build is not exit-clean; every handle here is stack/RAII-scoped and
  // the residual state is OCCT's own internal statics, not a harness defect.
  // Exit without running C++ static destructors so the true result is reported.
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
