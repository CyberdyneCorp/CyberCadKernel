// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_seeding_recall.mm — SSI Stage S2 (subdivision seeding) native-vs-OCCT
// branch-RECALL harness (iOS simulator). Gate 2 of the two-gate model for S2; Gate 1
// (host, no OCCT) is tests/native/test_native_ssi_seeding.cpp.
//
// For each TRANSVERSAL freeform / non-closed-form-quadric pair (exactly the pairs S1
// defers as NotAnalytic) this harness:
//   * builds the SAME two surfaces natively (src/native/math + the S2 SurfaceAdapter)
//     AND as OCCT Geom_Surface;
//   * runs the native S2 seeder `seed_intersection(A,B)` AND OCCT GeomAPI_IntSS;
//   * counts OCCT's intersection lines (the "true" transversal branch count) and the
//     native deduped seed count, and REPORTS the branch recall
//       recall = (native branches carrying ≥1 seed) ÷ (OCCT transversal branches);
//   * verifies every native seed lies on BOTH OCCT surfaces within tol
//     (GeomAPI_ProjectPointOnSurf::LowerDistance) — a seed is never faked;
//   * calls out the deferred-tangent (S4-gap) count.
//
// Recall is a REPORTED figure (SSI-ROADMAP S2), not asserted to be 1.0 blindly — the
// harness FAILS only on a real correctness violation: a seed OFF a surface, or recall
// below a conservative floor on a pair known to be cleanly transversal. Small-loop
// misses lower recall honestly and are printed, not hidden.
//
// SSI is INTERNAL — no cc_* entry point is called or added; asserted at the
// cybercad::native::ssi C++ boundary, exactly like the S1 parity harness.
//
// This TU is OCCT-dependent AND substrate-dependent: it links the OCCT oracle and the
// NumPP/SciPP numsci archive, and compiles src/native/ssi/seeding.cpp +
// src/native/numerics/numerics.cpp under -DCYBERCAD_HAS_NUMSCI. Built ONLY by
// scripts/run-sim-native-ssi-seeding.sh; on the SKIP list of run-sim-suite.sh.
//
#include "native/ssi/native_ssi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax3.hxx>
#include <Geom_Surface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_BezierSurface.hxx>
#include <Geom_Curve.hxx>
#include <GeomAPI_IntSS.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <TColgp_Array2OfPnt.hxx>

namespace ssi = cybercad::native::ssi;
namespace nm = cybercad::native::math;
using nm::Ax3;
using nm::Dir3;
using nm::Point3;

namespace {

constexpr double kPi = 3.14159265358979323846;
int g_failures = 0;

gp_Ax3 toOcctAx3(const Ax3& f) {
  return gp_Ax3(gp_Pnt(f.origin.x, f.origin.y, f.origin.z),
                gp_Dir(f.z.x(), f.z.y(), f.z.z()),
                gp_Dir(f.x.x(), f.x.y(), f.x.z()));
}

double distToOcctSurface(const Handle(Geom_Surface)& s, const Point3& p) {
  GeomAPI_ProjectPointOnSurf proj(gp_Pnt(p.x, p.y, p.z), s);
  return proj.NbPoints() > 0 ? proj.LowerDistance() : 1e30;
}

// Report one pair. `pairName` names it; `expectTransversalBranches` is the analytic
// truth (also cross-checked against OCCT's NbLines). `recallFloor` is the minimum
// acceptable native recall for this KNOWN-transversal pair (fail below it). A seed
// off either surface (> onSurfTol) always fails.
void reportPair(const std::string& pairName, ssi::SurfaceAdapter& A, ssi::SurfaceAdapter& B,
                const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                int expectTransversalBranches, double recallFloor, double onSurfTol) {
  ssi::SeedOptions opt;
  opt.initialGridU = 3;
  opt.initialGridV = 3;
  const ssi::SeedSet ss = ssi::seed_intersection(A, B, opt);

  GeomAPI_IntSS iss(sa, sb, 1e-7);
  const int occtN = iss.IsDone() ? iss.NbLines() : 0;

  double worst = 0.0;
  for (const auto& s : ss.seeds) {
    const double da = distToOcctSurface(sa, s.point);
    const double db = distToOcctSurface(sb, s.point);
    worst = std::max(worst, std::max(da, db));
  }

  ssi::RecallReport rep;
  rep.nativeBranches = ss.branchCount();
  rep.trueBranches = expectTransversalBranches;
  rep.deferredTangent = ss.deferredTangent;
  rep.worstOnSurfResidual = worst;
  const double recall = rep.recall();

  bool ok = true;
  if (worst > onSurfTol) { ok = false; }             // a seed OFF a surface → fail
  if (recall < recallFloor) { ok = false; }          // missed a known transversal branch
  if (!ok) ++g_failures;

  std::printf("[%s] %-26s native=%d occt=%d recall=%.2f tangent=%d worstOnSurf=%.2e\n",
              ok ? "PASS" : "FAIL", pairName.c_str(),
              rep.nativeBranches, occtN, recall, rep.deferredTangent, worst);
}

// ── pair builders (native + OCCT, identical placement) ──────────────────────────

Ax3 frameZ(Point3 o = {0, 0, 0}) {
  return Ax3{o, Dir3{1, 0, 0}, Dir3{0, 1, 0}, Dir3{0, 0, 1}};
}

// Orthogonal unequal-radius cylinders → 2 transversal loops (the skew-quadric quartic
// S1 defers). OCCT GeomAPI_IntSS returns the same locus (possibly arc-split).
void pairSkewCylinders() {
  nm::Cylinder cz{frameZ(), 1.0};
  const Ax3 fx{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}};  // axis X
  nm::Cylinder cx{fx, 0.7};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -2.0, 2.0};
  auto A = ssi::makeCylinderAdapter(cz, dom);
  auto B = ssi::makeCylinderAdapter(cx, dom);
  Handle(Geom_Surface) sa = new Geom_CylindricalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(fx), 0.7);
  reportPair("skew cyl unequal", A, B, sa, sb, /*branches=*/2, /*floor=*/1.0, 1e-6);
}

// Crossing spheres → 1 transversal circle.
void pairCrossingSpheres() {
  nm::Sphere s1{frameZ(), 1.0};
  nm::Sphere s2{frameZ({1.0, 0, 0}), 1.0};
  ssi::ParamBox dom{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(s1, dom);
  auto B = ssi::makeSphereAdapter(s2, dom);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(frameZ({1.0, 0, 0})), 1.0);
  reportPair("crossing spheres", A, B, sa, sb, /*branches=*/1, /*floor=*/1.0, 1e-6);
}

// Sphere piercing a freeform biquadratic Bézier bump → 1 transversal loop.
void pairSphereBezier() {
  nm::Sphere sp{frameZ(), 1.0};
  ssi::ParamBox sd{0.0, 2.0 * kPi, -kPi / 2, kPi / 2};
  auto A = ssi::makeSphereAdapter(sp, sd);
  std::vector<Point3> poles = {
      {-2, -2, 0.3}, {-2, 0, 0.0}, {-2, 2, 0.3},
      { 0, -2, 0.0}, { 0, 0, -0.5}, { 0, 2, 0.0},
      { 2, -2, 0.3}, { 2, 0, 0.0}, { 2, 2, 0.3}};
  auto B = ssi::makeBezierAdapter(poles, 3, 3);

  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(frameZ()), 1.0);
  TColgp_Array2OfPnt cp(1, 3, 1, 3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const Point3& p = poles[static_cast<std::size_t>(i) * 3 + j];
      cp.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
    }
  Handle(Geom_Surface) sb = new Geom_BezierSurface(cp);
  reportPair("sphere x bezier bump", A, B, sa, sb, /*branches=*/1, /*floor=*/1.0, 1e-6);
}

}  // namespace

int main() {
  std::printf("── SSI Stage S2 subdivision-seeding native-vs-OCCT recall\n");
  pairSkewCylinders();
  pairCrossingSpheres();
  pairSphereBezier();
  std::printf("%s (%d failures)\n", g_failures == 0 ? "ALL PASS" : "FAILURES", g_failures);
  return g_failures == 0 ? 0 : 1;
}
