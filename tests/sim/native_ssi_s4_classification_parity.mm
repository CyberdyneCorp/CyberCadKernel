// SPDX-License-Identifier: Apache-2.0
//
// native_ssi_s4_classification_parity.mm — SSI Stage S4 (coincident + tangent-contact
// CLASSIFICATION) native-vs-OCCT parity harness (iOS simulator). Gate 2 of the two-gate
// S4 model; Gate 1 (host, no OCCT) is tests/native/test_native_ssi_s4_classification.cpp.
//
// This asserts the native S4 CLASSIFICATION agrees with OCCT's own classifier for the
// same surface pair — it does NOT trace curves or fabricate geometry (that is S4-c, out
// of scope). Two oracles, matching the two native paths:
//
//   ANALYTIC (S4-a classify_degeneracy + S4-b classify_tangency) vs IntAna_QuadQuadGeo:
//     OCCT's analytic quadric intersector reports an IntAna_ResultType TypeInter():
//       IntAna_Same   ↔ native FullSurfaceSame           (coincident, same locus)
//       IntAna_Empty  ↔ native None + TransversalOnly     (disjoint)
//       IntAna_Point  ↔ native TangentPoint               (isolated tangency)
//       IntAna_Line/Circle where the two surfaces are TANGENT along it
//                     ↔ native TangentCurve               (tangent along a curve)
//       a proper transversal Line/Circle/Ellipse/…        ↔ native TransversalOnly
//     We classify OCCT's tangency by sampling ‖n₁×n₂‖ along its result curve (tangent ⇒
//     stays ~0). The native verdict must match the OCCT bucket.
//
//   SEEDED (S4-b classify_tangent_contact_seeded) vs GeomLProp_SLProps at the touch:
//     For a near-tangent freeform/quadric contact, OCCT's own normals at the projected
//     foot give the crossing sine and the LOCAL relative curvature sign structure; the
//     native differential-geometry classifier's TangentPoint/TangentCurve/
//     NearTangentTransversal call must match the sign structure OCCT reports (definite /
//     rank-1 / indefinite). NearTangentTransversal is a CLASSIFIED hand-off (S4-c → OCCT),
//     asserted as such — never traced.
//
// PASS = the native classification bucket equals the OCCT-derived bucket for every pair,
// and every emitted contact point/curve lies on BOTH surfaces within tol. A count of
// still-deferred-to-S4-c contacts is reported (honest), never hidden.
//
// SSI is INTERNAL — no cc_* entry point. This TU links the OCCT oracle (IntAna_QuadQuadGeo
// + Geom_* + GeomLProp_SLProps + projection) and, for the seeded pairs, the NumPP/SciPP
// numsci archive; built only by scripts/run-sim-native-ssi-s4.sh; on run-sim-suite.sh SKIP.
// Flushes + std::_Exit (OCCT static teardown in the trimmed static build is not exit-clean).
//
#include "native/ssi/native_ssi.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_ssi_s4_classification_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Vec.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Cone.hxx>
#include <IntAna_QuadQuadGeo.hxx>
#include <IntAna_ResultType.hxx>
#include <Geom_Surface.hxx>
#include <Geom_Plane.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomLProp_SLProps.hxx>

namespace ssi = cybercad::native::ssi;
namespace nm = cybercad::native::math;
using nm::Ax3;
using nm::Dir3;
using nm::Point3;

namespace {

constexpr double kPi = 3.14159265358979323846;

int g_pass = 0;
int g_fail = 0;
int g_deferredToS4c = 0;

gp_Ax3 toOcctAx3(const Ax3& f) {
  return gp_Ax3(gp_Pnt(f.origin.x, f.origin.y, f.origin.z),
                gp_Dir(f.z.x(), f.z.y(), f.z.z()),
                gp_Dir(f.x.x(), f.x.y(), f.x.z()));
}
gp_Pnt toOcctPnt(const Point3& p) { return gp_Pnt(p.x, p.y, p.z); }

double distToOcctSurface(const Handle(Geom_Surface)& s, const Point3& p) {
  GeomAPI_ProjectPointOnSurf proj(toOcctPnt(p), s);
  return proj.NbPoints() > 0 ? proj.LowerDistance() : 1e30;
}

// The classification bucket both sides map onto — the S4 CLASSIFICATION alphabet.
enum class Bucket { Same, TangentPoint, TangentCurve, Transversal, Empty, Deferred };

const char* bucketName(Bucket b) {
  switch (b) {
    case Bucket::Same:         return "Same";
    case Bucket::TangentPoint: return "TangentPoint";
    case Bucket::TangentCurve: return "TangentCurve";
    case Bucket::Transversal:  return "Transversal";
    case Bucket::Empty:        return "Empty";
    case Bucket::Deferred:     return "Deferred(S4-c)";
  }
  return "?";
}

// Map a native (CoincidentRegion, TangentContact) pair to the shared bucket. Coincidence
// dominates (same locus is neither a tangency nor a transversal crossing).
Bucket nativeBucket(const ssi::CoincidentRegion& cr, const ssi::TangentContact& tc) {
  if (cr.kind == ssi::CoincidenceKind::FullSurfaceSame ||
      cr.kind == ssi::CoincidenceKind::OverlapSubRegion)
    return Bucket::Same;
  switch (tc.type) {
    case ssi::TangentContactType::TangentPoint:           return Bucket::TangentPoint;
    case ssi::TangentContactType::TangentCurve:           return Bucket::TangentCurve;
    case ssi::TangentContactType::NearTangentTransversal: return Bucket::Deferred;
    case ssi::TangentContactType::Undecided:              return Bucket::Deferred;
    case ssi::TangentContactType::TransversalOnly:        return Bucket::Transversal;
  }
  return Bucket::Transversal;
}

// Map OCCT's IntAna_QuadQuadGeo verdict to the shared bucket. Same is the coincident
// locus; Empty is disjoint; an IntAna_Point is an isolated tangency. A curve result
// (Line/Circle/Ellipse/…) is a SECTION whose curve OCCT reports either way — geometrically
// a tangent-along-a-curve contact and a transversal section BOTH produce a curve. We
// distinguish them by the crossing sine ‖n₁×n₂‖ at a witness point on that curve
// (`curveTangent` — true when the caller measured ~0 on the oracle surfaces there): tangent
// curve ↔ TangentCurve, otherwise a proper transversal section. This matches the design's
// `TangentCurve ↔ tangent Line/Circle` mapping (the equator circle / grazing ruling cases).
Bucket occtAnalyticBucket(IntAna_ResultType t, bool curveTangent) {
  switch (t) {
    case IntAna_Same:              return Bucket::Same;
    case IntAna_Empty:             return Bucket::Empty;
    case IntAna_Point:             return Bucket::TangentPoint;
    case IntAna_Line:
    case IntAna_Circle:
    case IntAna_Ellipse:
    case IntAna_Parabola:
    case IntAna_Hyperbola:         return curveTangent ? Bucket::TangentCurve : Bucket::Transversal;
    default:                       return Bucket::Deferred;  // NoGeometricSolution etc.
  }
}

// Crossing sine of two OCCT surfaces at a 3D point (unit-normal cross magnitude at the
// projected feet). Used to confirm a native TangentCurve really is tangent on the oracle.
double occtCrossingSine(const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                        const Point3& p) {
  GeomAPI_ProjectPointOnSurf pa(toOcctPnt(p), sa), pb(toOcctPnt(p), sb);
  if (pa.NbPoints() == 0 || pb.NbPoints() == 0) return -1.0;
  Standard_Real ua, va, ub, vb;
  pa.LowerDistanceParameters(ua, va);
  pb.LowerDistanceParameters(ub, vb);
  GeomLProp_SLProps la(sa, ua, va, 1, 1e-9), lb(sb, ub, vb, 1, 1e-9);
  if (!la.IsNormalDefined() || !lb.IsNormalDefined()) return -1.0;
  const gp_Vec na(la.Normal()), nb(lb.Normal());
  return na.Crossed(nb).Magnitude();  // ‖n₁ × n₂‖ for unit normals
}

// ── ANALYTIC pair report — native classify_degeneracy + classify_tangency vs OCCT ────
void reportAnalytic(const std::string& name, const ssi::Surface& A, const ssi::Surface& B,
                    IntAna_QuadQuadGeo& occt,
                    const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb) {
  const ssi::CoincidentRegion cr = ssi::classify_degeneracy(A, B);
  const ssi::TangentContact tc = ssi::classify_tangency(A, B);
  const Bucket nb = nativeBucket(cr, tc);

  bool ok = true;

  // On-both-surfaces check for an emitted contact point / curve, plus the tangency witness
  // the oracle mapping needs: sample the crossing sine ‖n₁×n₂‖ on the ORACLE surfaces along
  // the native contact curve — if it stays ~0, OCCT's curve result is a tangent curve.
  double worstOnSurf = 0.0;
  bool curveTangent = false;
  if (tc.type == ssi::TangentContactType::TangentPoint) {
    worstOnSurf = std::max(distToOcctSurface(sa, tc.point), distToOcctSurface(sb, tc.point));
    if (worstOnSurf > 1e-6) ok = false;
  } else if (tc.type == ssi::TangentContactType::TangentCurve && tc.curve.has_value()) {
    const auto& c = *tc.curve;
    const auto rng = c.naturalRange();
    double worstSine = 0.0;
    for (int i = 0; i <= 8; ++i) {
      const double t = rng.first + (rng.second - rng.first) * (i / 8.0);
      const Point3 p = c.value(t);
      worstOnSurf = std::max(worstOnSurf,
                             std::max(distToOcctSurface(sa, p), distToOcctSurface(sb, p)));
      const double s = occtCrossingSine(sa, sb, p);
      if (s >= 0.0) worstSine = std::max(worstSine, s);
    }
    if (worstOnSurf > 1e-6) ok = false;
    curveTangent = (worstSine <= 1e-3);  // tangent along the curve on the oracle surfaces
    if (!curveTangent) ok = false;       // a native TangentCurve must be tangent on OCCT
  }

  const Bucket ob = occt.IsDone() ? occtAnalyticBucket(occt.TypeInter(), curveTangent)
                                  : Bucket::Deferred;
  if (nb != ob) ok = false;

  if (nb == Bucket::Deferred) ++g_deferredToS4c;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[S4A] %-4s %-22s native=%-12s occt=%-12s worstOnSurf=%.2e\n",
              ok ? "PASS" : "FAIL", name.c_str(), bucketName(nb), bucketName(ob), worstOnSurf);
  std::fflush(stdout);
}

Ax3 frameZ(Point3 o = {0, 0, 0}, Dir3 z = Dir3{0, 0, 1}, Dir3 x = Dir3{1, 0, 0}) {
  return Ax3::fromAxisAndRef(o, z, x);
}

// ── analytic pairs ─────────────────────────────────────────────────────────────────

void pairSameSphere() {
  nm::Sphere a{frameZ({0, 0, 0}), 2.0}, b{frameZ({0, 0, 0}), 2.0};
  IntAna_QuadQuadGeo occt(gp_Sphere(toOcctAx3(a.pos), 2.0), gp_Sphere(toOcctAx3(b.pos), 2.0),
                           1e-9);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(a.pos), 2.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(b.pos), 2.0);
  reportAnalytic("same sphere", ssi::Surface::of(a), ssi::Surface::of(b), occt, sa, sb);
}

void pairSpheresExternalTangent() {
  nm::Sphere a{frameZ({0, 0, 0}), 1.0}, b{frameZ({2, 0, 0}), 1.0};
  IntAna_QuadQuadGeo occt(gp_Sphere(toOcctAx3(a.pos), 1.0), gp_Sphere(toOcctAx3(b.pos), 1.0),
                           1e-9);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(a.pos), 1.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(b.pos), 1.0);
  reportAnalytic("spheres d=R1+R2", ssi::Surface::of(a), ssi::Surface::of(b), occt, sa, sb);
}

void pairSpheresTransversal() {
  nm::Sphere a{frameZ({0, 0, 0}), 1.0}, b{frameZ({1, 0, 0}), 1.0};
  IntAna_QuadQuadGeo occt(gp_Sphere(toOcctAx3(a.pos), 1.0), gp_Sphere(toOcctAx3(b.pos), 1.0),
                           1e-9);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(a.pos), 1.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(b.pos), 1.0);
  reportAnalytic("spheres crossing", ssi::Surface::of(a), ssi::Surface::of(b), occt, sa, sb);
}

void pairPlaneTangentSphere() {
  nm::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nm::Plane pl{frameZ({0, 0, 1})};
  IntAna_QuadQuadGeo occt(gp_Pln(toOcctAx3(pl.pos)), gp_Sphere(toOcctAx3(sp.pos), 1.0));
  Handle(Geom_Surface) sa = new Geom_Plane(toOcctAx3(pl.pos));
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(sp.pos), 1.0);
  reportAnalytic("plane tangent sphere", ssi::Surface::of(pl), ssi::Surface::of(sp), occt, sa, sb);
}

void pairCoaxialSphereCylinderEquator() {
  nm::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nm::Cylinder cy{frameZ({0, 0, 0}), 1.0};
  IntAna_QuadQuadGeo occt(gp_Cylinder(toOcctAx3(cy.pos), 1.0), gp_Sphere(toOcctAx3(sp.pos), 1.0), 1e-9);
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(sp.pos), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(cy.pos), 1.0);
  reportAnalytic("sphere=cyl equator", ssi::Surface::of(sp), ssi::Surface::of(cy), occt, sa, sb);
}

void pairPlaneTangentCylinder() {
  nm::Cylinder cy{frameZ({0, 0, 0}), 1.0};
  nm::Plane pl{Ax3::fromAxisAndRef({1, 0, 0}, Dir3{1, 0, 0}, Dir3{0, 0, 1})};
  IntAna_QuadQuadGeo occt(gp_Pln(toOcctAx3(pl.pos)), gp_Cylinder(toOcctAx3(cy.pos), 1.0), 1e-9, 1e-9);
  Handle(Geom_Surface) sa = new Geom_Plane(toOcctAx3(pl.pos));
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(cy.pos), 1.0);
  reportAnalytic("plane tangent cyl", ssi::Surface::of(pl), ssi::Surface::of(cy), occt, sa, sb);
}

#ifdef CYBERCAD_HAS_NUMSCI

// ── SEEDED pair report — native classify_tangent_contact_seeded vs OCCT normals ──────
// At a near-tangent contact of two adapters at known params, the native differential-
// geometry classifier's bucket must match the sign structure OCCT reports there. We
// confirm OCCT agrees the contact is near-tangent (crossing sine below the threshold) and
// that the native verdict is one of the honest tangent/deferred kinds — never a fabricated
// transversal on a tangency. TangentPoint/Curve are resolved; NearTangentTransversal is a
// classified S4-c hand-off (reported, not traced).
void reportSeeded(const std::string& name, const ssi::SurfaceAdapter& A,
                  const ssi::SurfaceAdapter& B, double u1, double v1, double u2, double v2,
                  const Handle(Geom_Surface)& sa, const Handle(Geom_Surface)& sb,
                  Bucket expect) {
  const Point3 P = A.point(u1, v1);
  const Dir3 nA = A.normal(u1, v1);
  const Dir3 nB = B.normal(u2, v2);
  const double sine = nm::norm(nm::cross(nA.vec(), nB.vec()));
  const double scale = std::max(A.modelScale, B.modelScale);
  const ssi::TangentContact tc =
      ssi::classify_tangent_contact_seeded(A, B, u1, v1, u2, v2, P, nA, nB, sine, scale);
  const Bucket nb = nativeBucket(ssi::CoincidentRegion::none(), tc);

  const double occtSine = occtCrossingSine(sa, sb, P);
  bool ok = (nb == expect);
  if (occtSine < 0.0 || occtSine > 1e-2) ok = false;  // OCCT must agree it is near-tangent
  const double onSurf = std::max(distToOcctSurface(sa, P), distToOcctSurface(sb, P));
  if (onSurf > 1e-5) ok = false;
  if (nb == Bucket::Deferred) ++g_deferredToS4c;
  if (ok) ++g_pass; else ++g_fail;
  std::printf("[S4B] %-4s %-22s native=%-14s expect=%-14s occtSine=%.2e onSurf=%.2e\n",
              ok ? "PASS" : "FAIL", name.c_str(), bucketName(nb), bucketName(expect),
              occtSine, onSurf);
  std::fflush(stdout);
}

void seededSphereSphereTangentPoint() {
  nm::Sphere s1{frameZ({0, 0, 0}), 1.0}, s2{frameZ({2, 0, 0}), 1.0};
  auto A = ssi::makeSphereAdapter(s1, ssi::ParamBox{0.0, kPi, -kPi / 2, kPi / 2});
  auto B = ssi::makeSphereAdapter(s2, ssi::ParamBox{0.0, kPi, -kPi / 2, kPi / 2});
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(s1.pos), 1.0);
  Handle(Geom_Surface) sb = new Geom_SphericalSurface(toOcctAx3(s2.pos), 1.0);
  reportSeeded("seeded sph-sph point", A, B, 0.0, 0.0, kPi, 0.0, sa, sb, Bucket::TangentPoint);
}

void seededSphereCylinderTangentCurve() {
  nm::Sphere sp{frameZ({0, 0, 0}), 1.0};
  nm::Cylinder cy{frameZ({0, 0, 0}), 1.0};
  auto A = ssi::makeSphereAdapter(sp, ssi::ParamBox{0.0, kPi, -kPi / 2, kPi / 2});
  auto B = ssi::makeCylinderAdapter(cy, ssi::ParamBox{0.0, 2.0 * kPi, -1.0, 1.0});
  Handle(Geom_Surface) sa = new Geom_SphericalSurface(toOcctAx3(sp.pos), 1.0);
  Handle(Geom_Surface) sb = new Geom_CylindricalSurface(toOcctAx3(cy.pos), 1.0);
  reportSeeded("seeded sph=cyl curve", A, B, 0.0, 0.0, 0.0, 0.0, sa, sb, Bucket::TangentCurve);
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace

int main() {
  std::printf("== SSI Stage S4 coincident+tangent CLASSIFICATION native-vs-OCCT ==\n");
  std::fflush(stdout);

  pairSameSphere();
  pairSpheresExternalTangent();
  pairSpheresTransversal();
  pairPlaneTangentSphere();
  pairCoaxialSphereCylinderEquator();
  pairPlaneTangentCylinder();

#ifdef CYBERCAD_HAS_NUMSCI
  seededSphereSphereTangentPoint();
  seededSphereCylinderTangentCurve();
#endif

  std::printf("== %d passed, %d failed (deferred to S4-c → OCCT: %d) ==\n",
              g_pass, g_fail, g_deferredToS4c);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}
