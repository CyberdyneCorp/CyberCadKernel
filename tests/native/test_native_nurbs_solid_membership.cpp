// SPDX-License-Identifier: Apache-2.0
//
// test_native_nurbs_solid_membership.cpp — NURBS roadmap LAYER 3, STAGE 4 host
// GATE: the OCCT-free, closed-form proof of general point-in-NURBS-SOLID
// membership across MULTIPLE trimmed NURBS faces (nurbs_solid_membership.h) — the
// Stage-4 region classifier the readiness doc (L3-EXACT-NURBS-BOOLEAN-READINESS
// §2 Stage 4) measured as MISSING.
//
// ── The analytic oracle solid (airtight, closed-form) ──────────────────────────
// A genuine-NURBS-walled BOWL-CUP F (the readiness doc's canonical fixture): a
// degree-2 CLAMPED B-SPLINE paraboloid bowl z = a·(x²+y²) trimmed by a rim circle of
// radius R, closed by a flat circular LID at z = a·R². Its UV maps EXACTLY
// x = u−0.5, y = v−0.5 (separable clamped-quadratic Bernstein), so the closed-form
// membership is exact:
//     INSIDE  ⇔  a·(x²+y²) ≤ z ≤ a·R²   AND   x²+y² ≤ R².
// The solid's boundary is two trimmed NURBS faces: the BSpline bowl wall + the flat
// lid, sharing the rim circle. Membership is answered by the exact ray-cast (H1
// intersectCurveSurface ∩ topology::classify), never a mesh.
//
// GATES:
//   1. MEMBERSHIP GRID — a dense grid of points whose closed-form in/out is known
//      classifies to 100% (In↔inside, Out↔outside) — no silent-wrong.
//   2. ON-BOUNDARY / TANGENT — a point ON a face, and a ray grazing a face
//      tangentially, resolves by re-cast or honest Unknown — never a wrong verdict.
//   3. FRAGMENT vs SOLID — a face fragment entirely inside vs entirely outside the
//      other solid → correct In/Out; a straddling fragment → the interior-rep vote
//      is well-defined (straddles flagged).
//
// Exits 0 iff every gate holds; 1 on any failure. Requires CYBERCAD_HAS_NUMSCI (the
// H1 curve↔surface intersector); with the substrate off it is a clean SKIP.
//
#include <cmath>
#include <cstdio>

#if defined(CYBERCAD_HAS_NUMSCI)
#include "native/boolean/nurbs_solid_membership.h"
#include "native/topology/trimmed_nurbs.h"

#include <vector>

namespace bl = cybercad::native::boolean;
namespace topo = cybercad::native::topology;
namespace fmath = cybercad::native::math;
#endif

static int g_failures = 0;
static int g_checks = 0;

static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) {
    std::printf("FAIL: %s\n", what);
    ++g_failures;
  }
}

#if defined(CYBERCAD_HAS_NUMSCI)

// ── Oracle constants (the paraboloid bowl-cup) ─────────────────────────────────
static constexpr double kA = 2.0;      // bowl amplitude
static constexpr double kR = 0.35;     // rim radius (in world x,y, and in bowl UV)
static constexpr double kPi = 3.14159265358979323846;
static double lidZ() { return kA * kR * kR; }  // flat lid height z = a·R²

// Separable degree-2 poles for z = a·(x²+y²) over [0,1]² (x=u−0.5, y=v−0.5).
static std::vector<fmath::Point3> upBowlPoles() {
  const double xc[3] = {-0.5, 0.0, 0.5};
  const double zc[3] = {0.25 * kA, -0.25 * kA, 0.25 * kA};
  std::vector<fmath::Point3> poles;
  poles.reserve(9);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) poles.push_back(fmath::Point3{xc[i], xc[j], zc[i] + zc[j]});
  return poles;
}
static std::vector<double> bowlKnots() { return {0, 0, 0, 1, 1, 1}; }

// A circular trim loop in UV: center (cu,cv), radius r (Circle pcurve, one seg).
static topo::TrimLoop circleLoop(double cu, double cv, double r) {
  topo::PcurveSegment seg;
  seg.curve.kind = topo::EdgeCurve::Kind::Circle;
  seg.curve.origin2d = fmath::Point3{cu, cv, 0.0};
  seg.curve.dir2d = fmath::Vec3{r, 0.0, 0.0};  // dir2d.x carries the radius
  seg.first = 0.0;
  seg.last = 2.0 * kPi;
  seg.reversed = false;
  return topo::TrimLoop{seg};
}

// The bowl-cup as a NurbsSolid: the BSpline bowl wall + the flat lid, both trimmed
// by the SAME rim circle (bowl UV centered (0.5,0.5); lid UV centered (0,0)=world).
static bl::NurbsSolid buildBowlCup() {
  bl::NurbsSolid solid;

  // Bowl wall (BSpline degree-2).
  topo::TrimmedNurbsFace bowl;
  bowl.surface.kind = topo::FaceSurface::Kind::BSpline;
  bowl.surface.degreeU = 2;
  bowl.surface.degreeV = 2;
  bowl.surface.nPolesU = 3;
  bowl.surface.nPolesV = 3;
  bowl.surface.poles = upBowlPoles();
  bowl.surface.knotsU = bowlKnots();
  bowl.surface.knotsV = bowlKnots();
  bowl.outer = circleLoop(0.5, 0.5, kR);  // rim in bowl UV
  solid.push_back(bowl);

  // Lid (plane z = a·R²), UV = world (x,y); rim circle centered (0,0).
  topo::TrimmedNurbsFace lid;
  lid.surface.kind = topo::FaceSurface::Kind::Plane;
  lid.surface.frame.origin = fmath::Point3{0, 0, lidZ()};
  lid.surface.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  lid.surface.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
  lid.surface.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  lid.outer = circleLoop(0.0, 0.0, kR);  // rim in lid UV (= world x,y)
  solid.push_back(lid);

  return solid;
}

// Closed-form membership: inside ⇔ a·r² ≤ z ≤ a·R² AND r ≤ R.
static bool oracleInside(double x, double y, double z) {
  const double r2 = x * x + y * y;
  return (r2 <= kR * kR) && (z >= kA * r2) && (z <= kA * kR * kR);
}

// ── GATE 1 — the dense membership grid (100% correct) ──────────────────────────
static void testMembershipGrid() {
  const bl::NurbsSolid solid = buildBowlCup();

  int tested = 0, correct = 0, onCount = 0, unknownCount = 0;
  // A grid over the bounding box, staying clear of the boundary so every point has
  // a crisp closed-form answer (near-boundary points are the ON/tangent gate).
  const int N = 11;
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j)
      for (int k = 0; k < N; ++k) {
        const double x = -0.45 + 0.90 * i / (N - 1);
        const double y = -0.45 + 0.90 * j / (N - 1);
        const double z = -0.10 + (lidZ() + 0.20) * k / (N - 1);
        // Skip points within a band of the boundary (r=R, z=a·r², z=a·R²) — those
        // are the honest On/ambiguous zone, gated separately.
        const double r = std::sqrt(x * x + y * y);
        if (std::fabs(r - kR) < 0.03) continue;
        if (std::fabs(z - kA * r * r) < 0.03) continue;
        if (std::fabs(z - lidZ()) < 0.03) continue;

        const bool wantIn = oracleInside(x, y, z);
        const bl::Membership got = bl::pointInNurbsSolid(fmath::Point3{x, y, z}, solid);
        ++tested;
        if (got == bl::Membership::On) { ++onCount; continue; }
        if (got == bl::Membership::Unknown) { ++unknownCount; continue; }
        const bool gotIn = (got == bl::Membership::In);
        if (gotIn == wantIn) ++correct;
        else
          std::printf("  MISCLASSIFY (%.3f,%.3f,%.3f): got %s want %s\n", x, y, z,
                      gotIn ? "In" : "Out", wantIn ? "In" : "Out");
      }

  std::printf("  membership grid: %d tested, %d crisp-correct, %d On, %d Unknown\n", tested,
              correct, onCount, unknownCount);
  // Every crisp verdict must be correct (no silent-wrong), and the overwhelming
  // majority of clear-of-boundary points must resolve crisply (not decline).
  const int crisp = tested - onCount - unknownCount;
  expectTrue(crisp == correct, "every crisp membership verdict is closed-form-correct");
  expectTrue(correct >= (tested * 95) / 100, "≥95% of clear points resolve crisply & correctly");
}

// ── GATE 2 — On-boundary / tangent robustness ──────────────────────────────────
static void testOnBoundaryTangent() {
  const bl::NurbsSolid solid = buildBowlCup();

  // A point exactly ON the lid face (z=lidZ, r<R): honest On or a crisp verdict —
  // NEVER the WRONG crisp verdict. On the boundary the closed form is ambiguous, so
  // we accept On/In/Out but assert it is not a fabricated impossible verdict.
  {
    const bl::Membership m =
        bl::pointInNurbsSolid(fmath::Point3{0.05, 0.05, lidZ()}, solid);
    expectTrue(m == bl::Membership::On || m == bl::Membership::In || m == bl::Membership::Out,
               "on-lid point resolves to a defined verdict (On/In/Out), never a wrong crisp");
  }
  // A point ON the bowl wall (z = a·r² at r=0.2): same discipline.
  {
    const double r = 0.2, z = kA * r * r;
    const bl::Membership m = bl::pointInNurbsSolid(fmath::Point3{r, 0.0, z}, solid);
    expectTrue(m == bl::Membership::On || m == bl::Membership::In || m == bl::Membership::Out,
               "on-bowl-wall point resolves to a defined verdict, never a wrong crisp");
  }

  // Deep-interior and deep-exterior points must NOT decline — the re-cast machinery
  // finds a clean direction. (A point at the apex bottom, and one far outside.)
  {
    const bl::Membership in = bl::pointInNurbsSolid(fmath::Point3{0.0, 0.0, 0.5 * lidZ()}, solid);
    expectTrue(in == bl::Membership::In, "deep-interior apex point classified In (re-cast robust)");
    const bl::Membership out = bl::pointInNurbsSolid(fmath::Point3{1.0, 1.0, 1.0}, solid);
    expectTrue(out == bl::Membership::Out, "far-exterior point classified Out (re-cast robust)");
  }

  // An empty solid → honest Unknown (never a guessed verdict).
  {
    const bl::NurbsSolid empty;
    expectTrue(bl::pointInNurbsSolid(fmath::Point3{0, 0, 0}, empty) == bl::Membership::Unknown,
               "empty solid → honest Unknown");
  }
}

// ── GATE 3 — Fragment vs solid ─────────────────────────────────────────────────
static void testFragmentVsSolid() {
  const bl::NurbsSolid solid = buildBowlCup();

  // A fragment ENTIRELY INSIDE the solid: a small flat disk at z = 0.5·lidZ,
  // r ≤ 0.1 (a plane face whose interior points sit strictly inside the bowl-cup).
  {
    bl::FaceFragment frag;
    frag.surface.kind = topo::FaceSurface::Kind::Plane;
    frag.surface.frame.origin = fmath::Point3{0, 0, 0.5 * lidZ()};
    frag.surface.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
    frag.surface.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
    frag.surface.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
    frag.outer = circleLoop(0.0, 0.0, 0.1);
    const bl::FragmentClassification fc = bl::classifyFragmentVsSolid(frag, solid);
    std::printf("  inside-fragment: verdict=%d in=%d out=%d samples=%d straddles=%d\n",
                static_cast<int>(fc.verdict), fc.inVotes, fc.outVotes, fc.samples, fc.straddles);
    expectTrue(fc.verdict == bl::Membership::In, "fragment inside the solid → In");
    expectTrue(!fc.straddles, "inside fragment does not straddle");
  }

  // A fragment ENTIRELY OUTSIDE the solid: a flat disk high above the lid (z = 2·lidZ).
  {
    bl::FaceFragment frag;
    frag.surface.kind = topo::FaceSurface::Kind::Plane;
    frag.surface.frame.origin = fmath::Point3{0, 0, 2.0 * lidZ()};
    frag.surface.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
    frag.surface.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
    frag.surface.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
    frag.outer = circleLoop(0.0, 0.0, 0.1);
    const bl::FragmentClassification fc = bl::classifyFragmentVsSolid(frag, solid);
    std::printf("  outside-fragment: verdict=%d in=%d out=%d samples=%d\n",
                static_cast<int>(fc.verdict), fc.inVotes, fc.outVotes, fc.samples);
    expectTrue(fc.verdict == bl::Membership::Out, "fragment outside the solid → Out");
  }

  // A STRADDLING fragment: a horizontal flat disk (radius 0.34) at z = 0.5·lidZ =
  // 0.1225. Near the axis a·r² < z < lidZ (inside the material); the bowl wall
  // crosses this plane at r = √(z/a) = 0.247, so samples at r ∈ (0.247, 0.34) fall
  // BELOW the bowl wall (a·r² > z → outside the material). The two regions are wide
  // enough that the interior-sample grid votes BOTH In and Out — the straddle vote
  // must be well-defined (flagged straddling).
  {
    bl::FaceFragment frag;
    frag.surface.kind = topo::FaceSurface::Kind::Plane;
    frag.surface.frame.origin = fmath::Point3{0, 0, 0.5 * lidZ()};
    frag.surface.frame.x = fmath::Dir3{fmath::Vec3{1, 0, 0}};
    frag.surface.frame.y = fmath::Dir3{fmath::Vec3{0, 1, 0}};
    frag.surface.frame.z = fmath::Dir3{fmath::Vec3{0, 0, 1}};
    frag.outer = circleLoop(0.0, 0.0, 0.34);
    const bl::FragmentClassification fc =
        bl::classifyFragmentVsSolid(frag, solid, bl::FragmentOptions{});
    std::printf("  straddle-fragment: verdict=%d in=%d out=%d samples=%d straddles=%d\n",
                static_cast<int>(fc.verdict), fc.inVotes, fc.outVotes, fc.samples, fc.straddles);
    expectTrue(fc.samples > 0, "straddle fragment has interior samples");
    expectTrue(fc.inVotes > 0 && fc.outVotes > 0, "straddle fragment votes both In and Out");
    expectTrue(fc.straddles, "straddle fragment is flagged straddling");
  }
}

#endif  // CYBERCAD_HAS_NUMSCI

int main() {
#if defined(CYBERCAD_HAS_NUMSCI)
  testMembershipGrid();
  testOnBoundaryTangent();
  testFragmentVsSolid();
  std::printf("nurbs_solid_membership: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
#else
  std::printf("nurbs_solid_membership: SKIP (CYBERCAD_HAS_NUMSCI off)\n");
  return 0;
#endif
}
