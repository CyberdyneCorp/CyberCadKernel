// SPDX-License-Identifier: Apache-2.0
//
// native_geometry_services_fuzz.mm — MOAT M6-breadth-7 (the COMPLETENESS BAR, SEVENTH
// domain): a GEOMETRY-SERVICES differential-fuzzing harness (iOS simulator) for the
// OCCT-free read-only analysis/section/drafting services the CyberCad app's Measure /
// Curvature / Section / Drawing / Inertia / Check panels read.
//
// This extends the six landed M6 differential fuzzers — native_boolean_fuzz.mm (curved
// booleans), native_step_import_fuzz.mm (STEP round-trip), native_construct_fuzz.mm
// (loft/sweep), native_blend_fuzz.mm (fillet/chamfer/offset/shell), native_wrap_emboss_fuzz.mm
// (pads/pockets) and native_mass_props_fuzz.mm (mesh mass-properties) — to a SEVENTH native
// domain: the geometry-services (GS) layer. Like its siblings it is INFRASTRUCTURE (a seeded
// harness, not a geometry capability): OCCT is the ORACLE, a CLOSED-FORM analytic value is the
// PRIMARY arbiter wherever one exists, the bar is ZERO silent wrong answers over a seeded batch
// and an HONEST DECLINE is first-class. It edits NO src/native/** (which stays OCCT-FREE and
// UNTOUCHED), changes NO cc_* signature, and adds NO geometry capability.
//
// ── THE SEVEN-WAY REACHABLE GS LAYER (each service driven DIRECTLY, OCCT-FREE) ──────────
//   GS3 distance   analysis/distance.h    an::minDistance         vs BRepExtrema_DistShapeShape
//   GS4 curvature  analysis/curvature.h   an::surfaceCurvature    vs GeomLProp_SLProps
//   GS2 section    section/section.h      sec::sectionByPlane     vs BRepAlgoAPI_Section+BRepGProp
//   GS5 inertia    analysis/inertia.h     an::principalInertia    vs GProp_PrincipalProps
//   GS6 validity   analysis/validity.h    an::checkSolidMesh      vs construction ground-truth
//                                                                    (+ BRepCheck on the valid case)
//   GS1 HLR        drafting/orthographic_hlr.h  drafting::projectOrthographic  vs the box-corner
//                                                                    closed form (9 visible+3 hidden)
//
// ── OBLIQUE / TILTED COVERAGE IS MANDATORY ──────────────────────────────────────────────
// Axis-aligned-only sampling is EXACTLY what let the ssi/plane_conics oblique-cylinder bug
// hide (intersectPlaneCylinder returns semi-major R/|sinθ| instead of R/|cosθ|). Every service
// therefore samples its tilted/oblique regime: skew (non-coplanar) segment pairs + tilted
// point·plane/circle for GS3; tilted analytic faces + a rotated NURBS patch for GS4; an OBLIQUE
// cut plane on a box AND the OBLIQUE CYLINDER cut (the plane_conics exemplar) for GS2; arbitrarily
// rotated solids for GS5; tilted broken solids for GS6; oblique/isometric view directions for GS1.
//
// ── THE plane_conics EXEMPLAR (HONEST-NATIVE-DECLINE, auto-reclassifies AGREE on the fix) ─
// The GS2 OBLIQUE CYLINDER cut is the fault this whole bar exists to catch. GS2 DECLINES it today
// (section.h routes around the upstream ssi/plane_conics inverted oblique-ellipse semi-major). The
// fuzzer COVERS the regime, classifies it HONEST-NATIVE-DECLINE (native declines; OCCT answers) —
// NOT a DISAGREE, NOT skipped — and asserts the decline is HONEST (typed Declined + genuinely the
// oblique-cylinder sub-domain). If a future ssi/ fix makes GS2 answer, the same trial re-classifies
// AGREE with no harness change (native-vs-OCCT + closed form take over automatically).
//
// ── THE FIVE-WAY CLASSIFIER at a FIXED (never-widened) tolerance ─────────────────────────
//   AGREE             native (and the closed-form arbiter where one exists) match the oracle in tol.
//   HONEST-NATIVE-DECLINE  native returns its documented typed decline while OCCT answers — first-
//                     class, logged, NOT a bar failure.
//   DISAGREE          native returns a CONFIDENT answer that is WRONG per oracle+closed-form — the
//                     fault the harness exists to catch (FAILS the bar, printed with its seed).
//   ORACLE-INACCURATE native matches exact closed-form math while OCCT is the outlier — logged.
//   BOTH-DECLINE      neither engine produced a comparable answer (degenerate input) — logged.
// No per-service tolerance is EVER widened: exact-analytic families use a tight tol; the planar
// section/inertia meshes reproduce the solid EXACTLY (tight); a curved family is held to its
// source-of-error bound. The closed-form analytic is authoritative over OCCT where it exists.
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────────
// Exit 0 IFF DISAGREE == 0 across the batch, with real per-service coverage (each covered service
// has ≥1 AGREE incl. an oblique/tilted trial where the service accepts it), proven over ≥2 seeds.
// The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed
// → byte-identical batch (splitmix64 → xoshiro256**, verbatim from the siblings).
//
// OCCT-DEPENDENT + NumSci-DEPENDENT (distance.h rides numerics/closest_point at compile time). Built
// ONLY by scripts/run-sim-native-geometry-services.sh; on run-sim-suite.sh's SKIP list (own main(),
// std::_Exit — OCCT static teardown of the trimmed static build is not exit-clean, same rationale as
// the siblings). src/native / src/engine stay UNTOUCHED — additive test/sim code only.
//
#include "native/analysis/curvature.h"
#include "native/analysis/distance.h"
#include "native/analysis/inertia.h"
#include "native/analysis/validity.h"
#include "native/drafting/orthographic_hlr.h"
#include "native/section/native_section.h"
#include "native/tessellate/mesh.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_geometry_services_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_geometry_services_fuzz requires -DCYBERCAD_HAS_NUMSCI (distance.h rides numerics)"
#endif

// ── OCCT oracle headers ──────────────────────────────────────────────────────────────────
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Surface.hxx>
#include <GeomLProp_SLProps.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PrincipalProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <ShapeAnalysis_FreeBounds.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_HSequenceOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <Standard_Failure.hxx>

namespace an   = cybercad::native::analysis;
namespace sec  = cybercad::native::section;
namespace draft= cybercad::native::drafting;
namespace ntess= cybercad::native::tessellate;
namespace ntopo= cybercad::native::topology;
namespace nm   = cybercad::native::math;
namespace nt   = cybercad::native::topology;

using nm::Ax3;
using nm::Dir3;
using nm::Point3;
using nm::Vec3;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kHalfPi = 1.5707963267948966192313216916398;

// Fixed, per-service, source-of-error tolerances — NEVER widened.
constexpr double kTolDist = 1e-6;   // GS3 closed-form min distance (coords O(1..30))
constexpr double kTolCurv = 1e-6;   // GS4 Gaussian/mean/principal curvature
constexpr double kTolSecRel = 1e-4; // GS2 section length/area (planar exact; OCCT approx wires)
constexpr double kTolInertRel = 1e-6; // GS5 planar box mesh reproduces the solid EXACTLY
constexpr double kTolOracle = 1e-6; // OCCT must match the closed form this tight (oracle-trust)

// ── deterministic RNG: splitmix64 seed → xoshiro256** (VERBATIM from the landed fuzzers).
//    Keyed ONLY by an explicit uint64 seed. No clock, no rand(): same seed → batch. ──────
struct Rng {
  uint64_t s[4];
  static uint64_t splitmix64(uint64_t& x) {
    uint64_t z = (x += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
  }
  explicit Rng(uint64_t seed) { for (auto& v : s) v = splitmix64(seed); }
  static uint64_t rotl(uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }
  uint64_t next() {
    const uint64_t r = rotl(s[1] * 5, 7) * 9;
    const uint64_t t = s[1] << 17;
    s[2] ^= s[0]; s[3] ^= s[1]; s[1] ^= s[2]; s[0] ^= s[3]; s[2] ^= t;
    s[3] = rotl(s[3], 45);
    return r;
  }
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
  double sym(double m) { return range(-m, m); }  // uniform in [-m, m]
};

// ── small linear-algebra helpers (fp64) ──────────────────────────────────────────────────
struct Rot { double m[3][3]; };  // columns are the rotated basis vectors

Rot identityRot() { return Rot{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}; }

Vec3 randDir(Rng& r) {
  for (int guard = 0; guard < 64; ++guard) {
    Vec3 v{r.sym(1.0), r.sym(1.0), r.sym(1.0)};
    const double n = nm::norm(v);
    if (n > 0.3) return v * (1.0 / n);
  }
  return Vec3{0, 0, 1};
}

// A random rotation as an orthonormal basis (Gram-Schmidt on two random dirs).
Rot randRot(Rng& r) {
  const Vec3 z = randDir(r);
  Vec3 x;
  for (int guard = 0; guard < 64; ++guard) {
    const Vec3 c = randDir(r);
    x = c - z * nm::dot(c, z);
    if (nm::norm(x) > 0.3) { x = x * (1.0 / nm::norm(x)); break; }
  }
  const Vec3 y = nm::cross(z, x);
  return Rot{{{x.x, y.x, z.x}, {x.y, y.y, z.y}, {x.z, y.z, z.z}}};
}

Vec3 apply(const Rot& R, const Vec3& v) {
  return Vec3{R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
             R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
             R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z};
}
Point3 applyP(const Rot& R, const Point3& p, const Vec3& t) {
  const Vec3 w = apply(R, p.asVec());
  return Point3{w.x + t.x, w.y + t.y, w.z + t.z};
}
gp_Trsf occtTrsf(const Rot& R, const Vec3& t) {
  gp_Trsf tr;
  tr.SetValues(R.m[0][0], R.m[0][1], R.m[0][2], t.x,
               R.m[1][0], R.m[1][1], R.m[1][2], t.y,
               R.m[2][0], R.m[2][1], R.m[2][2], t.z);
  return tr;
}

double relDiff(double a, double b) {
  return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : std::fabs(a - b);
}
bool relOk(double a, double b, double tol) {
  return std::fabs(a - b) <= tol * std::max(1.0, std::max(std::fabs(a), std::fabs(b)));
}

// ── the five-way verdict + per-service accounting ─────────────────────────────────────────
enum Verdict { AGREE, DECLINE, DISAGREE, ORACLE_INACCURATE, BOTH_DECLINE };

enum Svc { GS3, GS4, GS2, GS5, GS6, GS1, SVC_COUNT };
const char* svcName(int s) {
  switch (s) {
    case GS3: return "GS3 distance";
    case GS4: return "GS4 curvature";
    case GS2: return "GS2 section";
    case GS5: return "GS5 inertia";
    case GS6: return "GS6 validity";
    case GS1: return "GS1 HLR";
  }
  return "?";
}

int g_agree[SVC_COUNT] = {0}, g_decline[SVC_COUNT] = {0}, g_disagree[SVC_COUNT] = {0};
int g_oracleInacc[SVC_COUNT] = {0}, g_bothDecl[SVC_COUNT] = {0};
int g_agreeOblique[SVC_COUNT] = {0};  // AGREE trials that were the oblique/tilted regime

uint64_t g_seed = 0;

void record(int svc, Verdict v, bool oblique, int idx, const std::string& tag, const std::string& detail) {
  const char* vn = "?";
  switch (v) {
    case AGREE: ++g_agree[svc]; if (oblique) ++g_agreeOblique[svc]; vn = "AGREE"; break;
    case DECLINE: ++g_decline[svc]; vn = "HONEST-NATIVE-DECLINE"; break;
    case DISAGREE: ++g_disagree[svc]; vn = "DISAGREE"; break;
    case ORACLE_INACCURATE: ++g_oracleInacc[svc]; vn = "ORACLE-INACCURATE"; break;
    case BOTH_DECLINE: ++g_bothDecl[svc]; vn = "BOTH-DECLINE"; break;
  }
  if (v == DISAGREE) {
    std::printf("[GSFZ] %-22s case=%d %-13s %-10s %s\n"
                "       REPRO seed=0x%llx service=%s index=%d  %s\n",
                vn, idx, svcName(svc), tag.c_str(), detail.c_str(),
                static_cast<unsigned long long>(g_seed), svcName(svc), idx, detail.c_str());
  } else {
    std::printf("[GSFZ] %-22s case=%d %-13s %-10s%s %s\n", vn, idx, svcName(svc), tag.c_str(),
                oblique ? " [oblique]" : "", detail.c_str());
  }
  std::fflush(stdout);
}

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[256]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

// ═════════════════════════════════════════════════════════════════════════════════════════
// GS3 distance — an::minDistance vs BRepExtrema_DistShapeShape, closed-form arbiter.
// ═════════════════════════════════════════════════════════════════════════════════════════
void trialGS3(Rng& r, int idx) {
  // Sub-families: 0 point·point, 1 point·segment(tilted), 2 point·plane(tilted),
  // 3 point·circle(tilted), 4 segment·segment SKEW (the oblique regime). All closed-form.
  const int fam = static_cast<int>(r.below(5));
  const bool oblique = (fam == 4) || (fam == 2) || (fam == 3);
  double nativeD = -1.0, occtD = -1.0, analyticD = -1.0;
  bool nativeOk = false, occtOk = false;
  std::string tag = "pt.pt";

  if (fam == 0) {
    tag = "pt.pt";
    const Point3 a{r.sym(10), r.sym(10), r.sym(10)}, b{r.sym(10), r.sym(10), r.sym(10)};
    analyticD = nm::distance(a, b);
    auto nr = an::minDistance(an::Entity::ofVertex(a), an::Entity::ofVertex(b));
    nativeOk = nr.has_value(); if (nativeOk) nativeD = nr->distance;
    BRepExtrema_DistShapeShape ext(BRepBuilderAPI_MakeVertex(gp_Pnt(a.x, a.y, a.z)),
                                   BRepBuilderAPI_MakeVertex(gp_Pnt(b.x, b.y, b.z)));
    occtOk = ext.IsDone(); if (occtOk) occtD = ext.Value();
  } else if (fam == 1) {
    tag = "pt.seg";
    const Point3 p{r.sym(8), r.sym(8), r.sym(8)};
    const Vec3 dir = randDir(r);
    const double L = r.range(3.0, 8.0);
    const Point3 a{r.sym(6), r.sym(6), r.sym(6)};
    const Point3 b = a + dir * L;
    analyticD = an::distPointSegment(p, a, b).distance;  // reference closed form
    nt::EdgeCurve line; line.kind = nt::EdgeCurve::Kind::Line;
    line.frame = Ax3{a, Dir3{dir}, Dir3{}, Dir3{}};  // a Line curve's direction is frame.x
    auto nr = an::minDistance(an::Entity::ofVertex(p), an::Entity::ofEdge(line, 0.0, L));
    nativeOk = nr.has_value(); if (nativeOk) nativeD = nr->distance;
    BRepExtrema_DistShapeShape ext(BRepBuilderAPI_MakeVertex(gp_Pnt(p.x, p.y, p.z)),
        BRepBuilderAPI_MakeEdge(gp_Pnt(a.x, a.y, a.z), gp_Pnt(b.x, b.y, b.z)));
    occtOk = ext.IsDone(); if (occtOk) occtD = ext.Value();
  } else if (fam == 2) {
    tag = "pt.plane";
    const Point3 o{r.sym(3), r.sym(3), r.sym(3)};
    const Vec3 n = randDir(r);               // TILTED normal (oblique)
    const Point3 p = o + n * r.range(1.0, 9.0) +
                     randDir(r) * r.range(-4.0, 4.0);  // in-plane offset keeps foot interior
    analyticD = std::fabs(nm::dot(p - o, n));
    nt::FaceSurface plane; plane.kind = nt::FaceSurface::Kind::Plane;
    plane.frame = Ax3::fromAxisAndRef(o, Dir3{n}, Dir3{randDir(r)});
    auto nr = an::minDistance(an::Entity::ofVertex(p),
                              an::Entity::ofFace(plane, -60, 60, -60, 60));
    nativeOk = nr.has_value(); if (nativeOk) nativeD = nr->distance;
    Handle(Geom_Plane) gpl = new Geom_Plane(gp_Pnt(o.x, o.y, o.z), gp_Dir(n.x, n.y, n.z));
    TopoDS_Shape f = BRepBuilderAPI_MakeFace(gpl, -60.0, 60.0, -60.0, 60.0, 1e-9);
    BRepExtrema_DistShapeShape ext(BRepBuilderAPI_MakeVertex(gp_Pnt(p.x, p.y, p.z)), f);
    occtOk = ext.IsDone(); if (occtOk) occtD = ext.Value();
  } else if (fam == 3) {
    tag = "pt.circle";
    const Point3 c{r.sym(4), r.sym(4), r.sym(4)};
    const double R = r.range(1.5, 5.0);
    const Vec3 z = randDir(r);                // TILTED circle axis (oblique)
    const Ax3 fr = Ax3::fromAxisAndRef(c, Dir3{z}, Dir3{randDir(r)});
    const Point3 p{r.sym(10), r.sym(10), r.sym(10)};
    analyticD = an::distPointCircle(p, c, fr.x.vec(), fr.y.vec(), fr.z.vec(), R).distance;
    nt::EdgeCurve circ; circ.kind = nt::EdgeCurve::Kind::Circle; circ.frame = fr; circ.radius = R;
    auto nr = an::minDistance(an::Entity::ofVertex(p), an::Entity::ofEdge(circ, 0.0, kTwoPi));
    nativeOk = nr.has_value(); if (nativeOk) nativeD = nr->distance;
    Handle(Geom_Circle) gc = new Geom_Circle(
        gp_Ax2(gp_Pnt(c.x, c.y, c.z), gp_Dir(z.x, z.y, z.z),
               gp_Dir(fr.x.vec().x, fr.x.vec().y, fr.x.vec().z)), R);
    BRepExtrema_DistShapeShape ext(BRepBuilderAPI_MakeVertex(gp_Pnt(p.x, p.y, p.z)),
                                   BRepBuilderAPI_MakeEdge(gc));
    occtOk = ext.IsDone(); if (occtOk) occtD = ext.Value();
  } else {
    // SKEW segments with a KNOWN common-perpendicular gap d (the oblique regime).
    tag = "seg.seg-skew";
    Vec3 u = randDir(r), v = randDir(r);
    for (int guard = 0; guard < 32 && nm::norm(nm::cross(u, v)) < 0.25; ++guard) v = randDir(r);
    const Vec3 w = nm::cross(u, v) * (1.0 / nm::norm(nm::cross(u, v)));  // ⟂ both u and v
    const double d = r.range(1.0, 7.0), L = r.range(4.0, 8.0);
    const Point3 originShift{r.sym(5), r.sym(5), r.sym(5)};
    const Point3 aStart = originShift + u * (-L);          // A: center=originShift, dir u
    const Point3 aEnd   = originShift + u * (L);
    const Point3 bCtr   = originShift + w * d;              // B offset by exactly d·w
    const Point3 bStart = bCtr + v * (-L);
    const Point3 bEnd   = bCtr + v * (L);
    analyticD = d;                                          // exact by construction
    nt::EdgeCurve la; la.kind = nt::EdgeCurve::Kind::Line;
    la.frame = Ax3{aStart, Dir3{u}, Dir3{}, Dir3{}};  // a Line curve's direction is frame.x
    nt::EdgeCurve lb; lb.kind = nt::EdgeCurve::Kind::Line;
    lb.frame = Ax3{bStart, Dir3{v}, Dir3{}, Dir3{}};
    auto nr = an::minDistance(an::Entity::ofEdge(la, 0.0, 2 * L), an::Entity::ofEdge(lb, 0.0, 2 * L));
    nativeOk = nr.has_value(); if (nativeOk) nativeD = nr->distance;
    BRepExtrema_DistShapeShape ext(
        BRepBuilderAPI_MakeEdge(gp_Pnt(aStart.x, aStart.y, aStart.z), gp_Pnt(aEnd.x, aEnd.y, aEnd.z)),
        BRepBuilderAPI_MakeEdge(gp_Pnt(bStart.x, bStart.y, bStart.z), gp_Pnt(bEnd.x, bEnd.y, bEnd.z)));
    occtOk = ext.IsDone(); if (occtOk) occtD = ext.Value();
  }

  const std::string detail = fmt("dN=%.9g dOCCT=%.9g dAnalytic=%.9g", nativeD, occtD, analyticD);
  const bool natMatchesA = nativeOk && std::fabs(nativeD - analyticD) <= kTolDist;
  const bool occtMatchesA = occtOk && std::fabs(occtD - analyticD) <= kTolDist;
  if (!nativeOk && !occtOk) { record(GS3, BOTH_DECLINE, oblique, idx, tag, detail); return; }
  if (!nativeOk) { record(GS3, DECLINE, oblique, idx, tag, detail); return; }
  if (natMatchesA) {
    if (occtMatchesA) record(GS3, AGREE, oblique, idx, tag, detail);
    else record(GS3, ORACLE_INACCURATE, oblique, idx, tag, detail);
  } else {
    record(GS3, DISAGREE, oblique, idx, tag, detail);
  }
}

// ═════════════════════════════════════════════════════════════════════════════════════════
// GS4 curvature — an::surfaceCurvature vs GeomLProp_SLProps, analytic arbiter.
// Compares signed Gaussian K, |mean H|, and sorted |principal|. Analytic families use a tight
// closed form (rotation-invariant, so a TILTED frame is genuine oblique coverage); a random
// NURBS patch (rotated poles) is the freeform oblique regime with OCCT as the sole oracle.
// ═════════════════════════════════════════════════════════════════════════════════════════
struct Curv { double K, absH, hi, lo; bool ok; };
Curv fromNative(const std::optional<an::SurfaceCurvature>& c) {
  if (!c) return {0, 0, 0, 0, false};
  const double hi = std::max(std::fabs(c->k1), std::fabs(c->k2));
  const double lo = std::min(std::fabs(c->k1), std::fabs(c->k2));
  return {c->K, std::fabs(c->H), hi, lo, true};
}
Curv fromOcct(const Handle(Geom_Surface)& s, double u, double v) {
  GeomLProp_SLProps p(s, u, v, 2, 1e-12);
  if (!p.IsCurvatureDefined()) return {0, 0, 0, 0, false};
  const double hi = std::max(std::fabs(p.MaxCurvature()), std::fabs(p.MinCurvature()));
  const double lo = std::min(std::fabs(p.MaxCurvature()), std::fabs(p.MinCurvature()));
  return {p.GaussianCurvature(), std::fabs(p.MeanCurvature()), hi, lo, true};
}
bool curvClose(const Curv& a, const Curv& b, double tol) {
  return a.ok && b.ok && std::fabs(a.K - b.K) <= tol && std::fabs(a.absH - b.absH) <= tol &&
         std::fabs(a.hi - b.hi) <= tol && std::fabs(a.lo - b.lo) <= tol;
}

void trialGS4(Rng& r, int idx) {
  const int fam = static_cast<int>(r.below(5));  // 0 sphere,1 cyl,2 cone,3 torus,4 NURBS
  const bool oblique = true;                     // every GS4 trial rides a tilted frame/rotation
  const Rot R = randRot(r);
  const Point3 o{r.sym(3), r.sym(3), r.sym(3)};
  const Vec3 zaxis = apply(R, Vec3{0, 0, 1}), xaxis = apply(R, Vec3{1, 0, 0});
  const gp_Ax3 gax(gp_Pnt(o.x, o.y, o.z), gp_Dir(zaxis.x, zaxis.y, zaxis.z),
                   gp_Dir(xaxis.x, xaxis.y, xaxis.z));
  const Ax3 nax = Ax3::fromAxisAndRef(o, Dir3{zaxis}, Dir3{xaxis});

  Curv nat{}, occ{}, analytic{}; bool haveAnalytic = false;
  std::string tag; double un = 0, vn = 0, uo = 0, vo = 0;

  if (fam == 0) {
    tag = "sphere";
    const double Rr = r.range(1.0, 5.0), c = 1.0 / Rr;
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Sphere; s.frame = nax; s.radius = Rr;
    un = r.range(0.2, 1.2); vn = r.range(-1.0, 1.0); uo = r.range(0.2, 1.2); vo = r.range(-1.0, 1.0);
    nat = fromNative(an::surfaceCurvature(s, un, vn));
    Handle(Geom_SphericalSurface) g = new Geom_SphericalSurface(gax, Rr);
    occ = fromOcct(g, uo, vo);
    analytic = {c * c, c, c, c, true}; haveAnalytic = true;
  } else if (fam == 1) {
    tag = "cylinder";
    const double Rr = r.range(1.0, 5.0), c = 1.0 / Rr;
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Cylinder; s.frame = nax; s.radius = Rr;
    un = r.range(0.2, 1.2); vn = r.range(-2.0, 2.0); uo = r.range(0.2, 1.2); vo = r.range(-2.0, 2.0);
    nat = fromNative(an::surfaceCurvature(s, un, vn));
    Handle(Geom_CylindricalSurface) g = new Geom_CylindricalSurface(gax, Rr);
    occ = fromOcct(g, uo, vo);
    analytic = {0.0, c * 0.5, c, 0.0, true}; haveAnalytic = true;
  } else if (fam == 2) {
    tag = "cone-v0";  // sampled at v=0 (ρ=R), the parity-proven aligned point
    const double Rr = r.range(2.0, 6.0), a = r.range(0.15, 0.8);
    const double kc = std::cos(a) / Rr;
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Cone; s.frame = nax; s.radius = Rr; s.semiAngle = a;
    un = r.range(0.2, 1.2); uo = un;
    nat = fromNative(an::surfaceCurvature(s, un, 0.0));
    Handle(Geom_ConicalSurface) g = new Geom_ConicalSurface(gax, a, Rr);
    occ = fromOcct(g, uo, 0.0);
    analytic = {0.0, kc * 0.5, kc, 0.0, true}; haveAnalytic = true;
  } else if (fam == 3) {
    tag = "torus";
    const double Rr = r.range(3.0, 6.0), rr = r.range(1.0, 2.0);
    const bool top = r.unit() < 0.5;
    const double vv = top ? kHalfPi : 0.0;
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Torus; s.frame = nax; s.radius = Rr; s.minorRadius = rr;
    un = r.range(0.2, 1.2); uo = un;
    nat = fromNative(an::surfaceCurvature(s, un, vv));
    Handle(Geom_ToroidalSurface) g = new Geom_ToroidalSurface(gax, Rr, rr);
    occ = fromOcct(g, uo, vv);
    const double kMinor = 1.0 / rr;
    const double kMajor = std::cos(vv) / (Rr + rr * std::cos(vv));
    const double hi = std::max(kMinor, kMajor), lo = std::min(kMinor, kMajor);
    analytic = {kMinor * kMajor, std::fabs(0.5 * (kMinor + kMajor)), std::fabs(hi), std::fabs(lo), true};
    haveAnalytic = true;
  } else {
    // Random gentle bicubic NURBS-free B-spline patch, ROTATED (freeform oblique regime).
    tag = "bspline";
    const int nU = 4, nV = 4, degU = 3, degV = 3;
    double z[4][4];
    for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) z[i][j] = r.range(-0.6, 0.6);
    nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::BSpline;
    s.degreeU = degU; s.degreeV = degV; s.nPolesU = nU; s.nPolesV = nV;
    TColgp_Array2OfPnt op(1, nU, 1, nV);
    for (int i = 0; i < nU; ++i)
      for (int j = 0; j < nV; ++j) {
        const Point3 p = applyP(R, Point3{double(i), double(j), z[i][j]}, o.asVec());
        s.poles.push_back(p);
        op.SetValue(i + 1, j + 1, gp_Pnt(p.x, p.y, p.z));
      }
    s.knotsU = {0, 0, 0, 0, 1, 1, 1, 1};
    s.knotsV = {0, 0, 0, 0, 1, 1, 1, 1};
    TColStd_Array1OfReal uk(1, 2), vk(1, 2); uk.SetValue(1, 0); uk.SetValue(2, 1);
    vk.SetValue(1, 0); vk.SetValue(2, 1);
    TColStd_Array1OfInteger um(1, 2), vm(1, 2);
    um.SetValue(1, 4); um.SetValue(2, 4); vm.SetValue(1, 4); vm.SetValue(2, 4);
    Handle(Geom_BSplineSurface) g =
        new Geom_BSplineSurface(op, uk, vk, um, vm, degU, degV, Standard_False, Standard_False);
    un = r.range(0.2, 0.8); vn = r.range(0.2, 0.8); uo = un; vo = vn;  // SAME (u,v) on identical patch
    nat = fromNative(an::surfaceCurvature(s, un, vn));
    occ = fromOcct(g, uo, vo);
    haveAnalytic = false;
  }

  const std::string detail =
      fmt("K:N=%.7g/O=%.7g |H|:N=%.7g/O=%.7g", nat.K, occ.K, nat.absH, occ.absH);
  if (!nat.ok && !occ.ok) { record(GS4, BOTH_DECLINE, oblique, idx, tag, detail); return; }
  if (!nat.ok) { record(GS4, DECLINE, oblique, idx, tag, detail); return; }

  if (haveAnalytic) {
    const bool natA = curvClose(nat, analytic, kTolCurv);
    const bool occA = curvClose(occ, analytic, kTolOracle);
    if (natA && occA) record(GS4, AGREE, oblique, idx, tag, detail);
    else if (natA && occ.ok) record(GS4, ORACLE_INACCURATE, oblique, idx, tag, detail);
    else if (natA) record(GS4, AGREE, oblique, idx, tag, detail);  // native = exact math, OCCT declined
    else record(GS4, DISAGREE, oblique, idx, tag, detail);
  } else {
    // No closed form: OCCT is the sole oracle for the freeform patch.
    if (!occ.ok) { record(GS4, DECLINE, oblique, idx, tag, detail); return; }
    if (curvClose(nat, occ, kTolCurv)) record(GS4, AGREE, oblique, idx, tag, detail);
    else record(GS4, DISAGREE, oblique, idx, tag, detail);
  }
}

// ═════════════════════════════════════════════════════════════════════════════════════════
// GS2 section — sec::sectionByPlane vs BRepAlgoAPI_Section (+ BRepGProp), closed-form arbiter.
// Native fixtures mirror native_section_parity; OCCT primitives are matched. The OBLIQUE box cut
// is a native-vs-OCCT differential; the OBLIQUE CYLINDER cut is the plane_conics HONEST-DECLINE.
// ═════════════════════════════════════════════════════════════════════════════════════════
using ntopo::ShapeBuilder;
using ntopo::Shape;

Shape lineEdge(const Point3& a, const Point3& b) {
  const Vec3 d = b - a;
  nt::EdgeCurve c; c.kind = nt::EdgeCurve::Kind::Line;
  c.frame = Ax3{a, Dir3{d}, Dir3{}, Dir3{}};
  return ShapeBuilder::makeEdge(c, 0.0, nm::norm(d), ShapeBuilder::makeVertex(a),
                                ShapeBuilder::makeVertex(b));
}
Shape circleEdge(const Point3& ctr, double rad, const Dir3& n, const Dir3& x) {
  nt::EdgeCurve c; c.kind = nt::EdgeCurve::Kind::Circle;
  c.frame = Ax3::fromAxisAndRef(ctr, n, x); c.radius = rad;
  return ShapeBuilder::makeEdge(c, 0.0, kTwoPi, Shape{}, Shape{});
}
Shape planarQuad(const Point3& p0, const Point3& p1, const Point3& p2, const Point3& p3, const Dir3& n) {
  nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Plane;
  s.frame = Ax3::fromAxisAndRef(p0, n, Dir3{p1 - p0});
  return ShapeBuilder::makeFace(s, ShapeBuilder::makeWire(
      {lineEdge(p0, p1), lineEdge(p1, p2), lineEdge(p2, p3), lineEdge(p3, p0)}));
}
Shape makeBox(double lx, double ly, double lz) {
  const Point3 a{0,0,0}, b{lx,0,0}, c{lx,ly,0}, d{0,ly,0};
  const Point3 e{0,0,lz}, f{lx,0,lz}, g{lx,ly,lz}, h{0,ly,lz};
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({
      planarQuad(a,d,c,b, Dir3{0,0,-1}), planarQuad(e,f,g,h, Dir3{0,0,1}),
      planarQuad(a,b,f,e, Dir3{0,-1,0}), planarQuad(d,h,g,c, Dir3{0,1,0}),
      planarQuad(a,e,h,d, Dir3{-1,0,0}), planarQuad(b,c,g,f, Dir3{1,0,0})})});
}
Shape makeCyl(double Rr, double H) {
  const Dir3 zp{0,0,1}, xp{1,0,0};
  nt::FaceSurface bs; bs.kind = nt::FaceSurface::Kind::Plane;
  bs.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, Dir3{0,0,-1}, xp);
  Shape bottom = ShapeBuilder::makeFace(bs, ShapeBuilder::makeWire({circleEdge(Point3{0,0,0}, Rr, zp, xp)}));
  nt::FaceSurface ts; ts.kind = nt::FaceSurface::Kind::Plane;
  ts.frame = Ax3::fromAxisAndRef(Point3{0,0,H}, zp, xp);
  Shape top = ShapeBuilder::makeFace(ts, ShapeBuilder::makeWire({circleEdge(Point3{0,0,H}, Rr, zp, xp)}));
  nt::FaceSurface ls; ls.kind = nt::FaceSurface::Kind::Cylinder;
  ls.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, zp, xp); ls.radius = Rr;
  Shape lat = ShapeBuilder::makeFace(ls, ShapeBuilder::makeWire({
      circleEdge(Point3{0,0,0}, Rr, zp, xp), circleEdge(Point3{0,0,H}, Rr, zp, xp)}));
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({bottom, top, lat})});
}
Shape makeSphere(double Rr) {
  nt::FaceSurface s; s.kind = nt::FaceSurface::Kind::Sphere;
  s.frame = Ax3::fromAxisAndRef(Point3{0,0,0}, Dir3{0,0,1}, Dir3{1,0,0}); s.radius = Rr;
  return ShapeBuilder::makeSolid({ShapeBuilder::makeShell({
      ShapeBuilder::makeFace(s, ShapeBuilder::makeWire({}))})});
}
Dir3 orthoX(const Dir3& n) {
  const double ax=std::fabs(n.vec().x), ay=std::fabs(n.vec().y), az=std::fabs(n.vec().z);
  Vec3 pick = (ax<=ay && ax<=az) ? Vec3{1,0,0} : (ay<=az) ? Vec3{0,1,0} : Vec3{0,0,1};
  return Dir3{nm::cross(n.vec(), pick)};
}
nm::Plane cutPlane(const Point3& o, const Dir3& n) {
  return nm::Plane{Ax3::fromAxisAndRef(o, n, orthoX(n))};
}

struct OcctSec { int wireCount = 0; double length = 0, area = 0; bool allClosed = true, ok = false; };
OcctSec occtSection(const TopoDS_Shape& solid, const gp_Pnt& o, const gp_Dir& n) {
  OcctSec r;
  try {
    const gp_Pln pln(o, n);
    BRepAlgoAPI_Section sc(solid, pln, Standard_False);
    sc.ComputePCurveOn1(Standard_False); sc.Approximation(Standard_False); sc.Build();
    if (!sc.IsDone()) return r;
    const TopoDS_Shape edges = sc.Shape();
    GProp_GProps lp; BRepGProp::LinearProperties(edges, lp); r.length = lp.Mass();
    Handle(TopTools_HSequenceOfShape) edgeSeq = new TopTools_HSequenceOfShape();
    for (TopExp_Explorer ex(edges, TopAbs_EDGE); ex.More(); ex.Next()) edgeSeq->Append(ex.Current());
    if (edgeSeq->IsEmpty()) return r;
    Handle(TopTools_HSequenceOfShape) wires;
    ShapeAnalysis_FreeBounds::ConnectEdgesToWires(edgeSeq, 1e-6, Standard_False, wires);
    r.wireCount = wires ? wires->Length() : 0;
    for (Standard_Integer i = 1; i <= r.wireCount; ++i) {
      const TopoDS_Wire w = TopoDS::Wire(wires->Value(i));
      if (!w.Closed()) r.allClosed = false;
      BRepBuilderAPI_MakeFace mf(pln, w, Standard_True);
      if (mf.IsDone()) { GProp_GProps sp; BRepGProp::SurfaceProperties(mf.Face(), sp); r.area += std::fabs(sp.Mass()); }
    }
    r.ok = true;
  } catch (const Standard_Failure&) { r.ok = false; }
  return r;
}

void trialGS2(Rng& r, int idx) {
  const int fam = static_cast<int>(r.below(6));
  // 0 box axis, 1 cyl perp(circle), 2 cyl axial(rect), 3 sphere(circle),
  // 4 box OBLIQUE (native-vs-OCCT), 5 cyl OBLIQUE (plane_conics HONEST-DECLINE)
  const bool oblique = (fam == 4) || (fam == 5);
  Shape nativeSolid; TopoDS_Shape occtSolid;
  Point3 po; Dir3 nn; std::string tag;
  bool haveClosed = false; double cfLen = 0, cfArea = 0; int cfLoops = 1;

  if (fam == 0) {
    tag = "box-axis";
    const double lx = r.range(4, 12), ly = r.range(4, 12), lz = r.range(4, 12);
    nativeSolid = makeBox(lx, ly, lz); occtSolid = BRepPrimAPI_MakeBox(lx, ly, lz).Shape();
    po = Point3{0, 0, r.range(0.2 * lz, 0.8 * lz)}; nn = Dir3{0, 0, 1};
    haveClosed = true; cfArea = lx * ly; cfLen = 2 * (lx + ly); cfLoops = 1;
  } else if (fam == 1) {
    tag = "cyl-perp";
    const double Rr = r.range(2, 5), H = r.range(6, 12);
    nativeSolid = makeCyl(Rr, H); occtSolid = BRepPrimAPI_MakeCylinder(Rr, H).Shape();
    po = Point3{0, 0, r.range(0.2 * H, 0.8 * H)}; nn = Dir3{0, 0, 1};
    haveClosed = true; cfArea = kPi * Rr * Rr; cfLen = kTwoPi * Rr; cfLoops = 1;
  } else if (fam == 2) {
    tag = "cyl-axial";
    const double Rr = r.range(2, 5), H = r.range(6, 12);
    nativeSolid = makeCyl(Rr, H); occtSolid = BRepPrimAPI_MakeCylinder(Rr, H).Shape();
    po = Point3{0, 0, 0}; nn = Dir3{0, 1, 0};
    haveClosed = true; cfArea = 2 * Rr * H; cfLen = 2 * (2 * Rr + H); cfLoops = 1;
  } else if (fam == 3) {
    tag = "sphere";
    const double Rr = r.range(3, 7); const double zc = r.range(-0.5 * Rr, 0.5 * Rr);
    nativeSolid = makeSphere(Rr); occtSolid = BRepPrimAPI_MakeSphere(Rr).Shape();
    po = Point3{0, 0, zc}; nn = Dir3{0, 0, 1};
    const double rc = std::sqrt(Rr * Rr - zc * zc);
    haveClosed = true; cfArea = kPi * rc * rc; cfLen = kTwoPi * rc; cfLoops = 1;
  } else if (fam == 4) {
    tag = "box-oblique";
    // A cube cut by a balanced-normal plane through the centre → a HEXAGON crossing all six
    // faces (each face contributes exactly one segment), the clean oblique regime GS2 accepts.
    const double s = r.range(6, 12);
    nativeSolid = makeBox(s, s, s); occtSolid = BRepPrimAPI_MakeBox(s, s, s).Shape();
    po = Point3{s / 2, s / 2, s / 2};
    const double sx = r.unit() < 0.5 ? -1 : 1, sy = r.unit() < 0.5 ? -1 : 1;
    nn = Dir3{sx * r.range(0.6, 1.0), sy * r.range(0.6, 1.0), r.range(0.6, 1.0)};
    haveClosed = false;  // no simple closed form for an oblique box hexagon → OCCT oracle
  } else {
    tag = "cyl-oblique";  // the plane_conics exemplar → HONEST-NATIVE-DECLINE
    const double Rr = r.range(2, 5), H = r.range(8, 14);
    nativeSolid = makeCyl(Rr, H); occtSolid = BRepPrimAPI_MakeCylinder(Rr, H).Shape();
    po = Point3{0, 0, H / 2};
    nn = Dir3{0.0, r.range(0.4, 0.9), 1.0};  // tilted off the axis → oblique ellipse
    haveClosed = false;
  }

  const sec::SectionResult nr = sec::sectionByPlane(nativeSolid, cutPlane(po, nn));
  const OcctSec orr = occtSection(occtSolid, gp_Pnt(po.x, po.y, po.z),
                                  gp_Dir(nn.vec().x, nn.vec().y, nn.vec().z));

  if (fam == 5) {
    // Expected HONEST decline: native routes around the ssi oblique-cylinder defect.
    const bool declined = (nr.status == sec::SectionStatus::Declined);
    const std::string detail = "native=" + std::string(declined ? "DECLINED" : "OK") +
        " reason='" + nr.reason + "' | OCCT loops=" + std::to_string(orr.wireCount) +
        " len=" + fmt("%.5g", orr.length);
    if (declined && orr.ok) { record(GS2, DECLINE, oblique, idx, tag, detail); return; }
    if (!declined && nr.ok() && orr.ok) {
      // ssi/ fix has landed → native now answers: verify it against OCCT (auto-reclassify).
      const bool ok = nr.loopCount() == orr.wireCount && relOk(nr.totalLength(), orr.length, kTolSecRel);
      record(GS2, ok ? AGREE : DISAGREE, oblique, idx, tag, detail); return;
    }
    record(GS2, orr.ok ? DECLINE : BOTH_DECLINE, oblique, idx, tag, detail); return;
  }

  const std::string detail = fmt("loopsN=%.0f loopsO=%.0f lenN=%.6g lenO=%.6g",
      double(nr.loopCount()), double(orr.wireCount), nr.totalLength(), orr.length) +
      fmt(" areaN=%.6g areaO=%.6g", nr.totalArea(), orr.area);

  if (!nr.ok() && !orr.ok) { record(GS2, BOTH_DECLINE, oblique, idx, tag, detail); return; }
  if (!nr.ok()) { record(GS2, DECLINE, oblique, idx, tag, detail); return; }
  if (!orr.ok) { record(GS2, ORACLE_INACCURATE, oblique, idx, tag, detail); return; }

  const bool loopOk = nr.loopCount() == orr.wireCount;
  const bool lenVsOcct = relOk(nr.totalLength(), orr.length, kTolSecRel);
  const bool areaVsOcct = relOk(nr.totalArea(), orr.area, kTolSecRel);

  if (haveClosed) {
    const bool natVsCF = relOk(nr.totalLength(), cfLen, kTolSecRel) &&
                         relOk(nr.totalArea(), cfArea, kTolSecRel) && nr.loopCount() == cfLoops;
    const bool occVsCF = relOk(orr.length, cfLen, kTolSecRel) && relOk(orr.area, cfArea, kTolSecRel);
    if (natVsCF && occVsCF && loopOk) record(GS2, AGREE, oblique, idx, tag, detail);
    else if (natVsCF) record(GS2, ORACLE_INACCURATE, oblique, idx, tag, detail);  // native=CF, OCCT off
    else record(GS2, DISAGREE, oblique, idx, tag, detail);
  } else {
    // Oblique box: no closed form → native must agree with the OCCT oracle.
    if (loopOk && lenVsOcct && areaVsOcct) record(GS2, AGREE, oblique, idx, tag, detail);
    else record(GS2, DISAGREE, oblique, idx, tag, detail);
  }
}

// ═════════════════════════════════════════════════════════════════════════════════════════
// GS5 inertia — an::principalInertia(mesh) vs GProp_PrincipalProps, exact box tensor arbiter.
// A watertight box mesh under a RANDOM rotation+translation (principal axes off the world axes).
// Principal moments are rotation-INVARIANT → the closed form is exact regardless of the tilt.
// ═════════════════════════════════════════════════════════════════════════════════════════
ntess::Mesh boxMesh(double w, double d, double h, const Rot& R, const Vec3& t) {
  const Point3 loc[8] = {{0,0,0},{w,0,0},{w,d,0},{0,d,0},{0,0,h},{w,0,h},{w,d,h},{0,d,h}};
  ntess::Mesh m;
  for (const Point3& p : loc) m.vertices.push_back(applyP(R, p, t));
  // 12 outward-CCW triangles (same winding convention as OCCT box).
  const int f[12][3] = {
      {0,3,2},{0,2,1},   // z=0 (normal -z)
      {4,5,6},{4,6,7},   // z=h (normal +z)
      {0,1,5},{0,5,4},   // y=0 (normal -y)
      {3,7,6},{3,6,2},   // y=d (normal +y)
      {0,4,7},{0,7,3},   // x=0 (normal -x)
      {1,2,6},{1,6,5}};  // x=w (normal +x)
  for (auto& tr : f) m.addTriangle(tr[0], tr[1], tr[2]);
  return m;
}

void trialGS5(Rng& r, int idx) {
  const bool oblique = r.unit() < 0.75;  // most trials rotated; some axis-aligned for the baseline
  const double w = r.range(1.5, 5), d = r.range(1.5, 5), h = r.range(1.5, 5);
  const Rot R = oblique ? randRot(r) : identityRot();
  const Vec3 t{r.sym(6), r.sym(6), r.sym(6)};
  const ntess::Mesh mesh = boxMesh(w, d, h, R, t);

  const auto ni = an::principalInertia(mesh);
  const double V = w * d * h;
  std::array<double, 3> aM = {V * (d * d + h * h) / 12.0, V * (w * w + h * h) / 12.0,
                              V * (w * w + d * d) / 12.0};
  std::sort(aM.begin(), aM.end());
  const Point3 cLocal{w / 2, d / 2, h / 2};
  const Point3 aC = applyP(R, cLocal, t);

  // OCCT: same box, same placement, principal props (moments rotation-invariant).
  double oV = -1, oI0 = 0, oI1 = 0, oI2 = 0, oCx = 0, oCy = 0, oCz = 0; bool oOk = false;
  try {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(w, d, h).Shape();
    BRepBuilderAPI_Transform xf(box, occtTrsf(R, t), Standard_True);
    const TopoDS_Shape placed = xf.Shape();
    GProp_GProps vg; BRepGProp::VolumeProperties(placed, vg);
    oV = std::fabs(vg.Mass());
    const gp_Pnt g = vg.CentreOfMass(); oCx = g.X(); oCy = g.Y(); oCz = g.Z();
    GProp_PrincipalProps pp = vg.PrincipalProperties();
    pp.Moments(oI0, oI1, oI2);
    double om[3] = {oI0, oI1, oI2}; std::sort(om, om + 3); oI0 = om[0]; oI1 = om[1]; oI2 = om[2];
    oOk = true;
  } catch (const Standard_Failure&) { oOk = false; }

  if (!ni) {
    const std::string detail = "native principalInertia DECLINED (mesh not watertight?)";
    record(GS5, oOk ? DECLINE : BOTH_DECLINE, oblique, idx, "box", detail); return;
  }
  std::array<double, 3> nM = ni->moments; std::sort(nM.begin(), nM.end());
  const double cErr = std::sqrt((ni->centroid.x - aC.x) * (ni->centroid.x - aC.x) +
                                (ni->centroid.y - aC.y) * (ni->centroid.y - aC.y) +
                                (ni->centroid.z - aC.z) * (ni->centroid.z - aC.z));
  const double centTol = kTolInertRel * std::max({w, d, h, nm::norm(t), 1.0});
  const bool natVsA = relDiff(ni->volume, V) < kTolInertRel &&
                      relDiff(nM[0], aM[0]) < kTolInertRel && relDiff(nM[1], aM[1]) < kTolInertRel &&
                      relDiff(nM[2], aM[2]) < kTolInertRel && cErr < centTol;
  const bool occVsA = oOk && relDiff(oV, V) < kTolOracle &&
                      relDiff(oI0, aM[0]) < 1e-4 && relDiff(oI2, aM[2]) < 1e-4;

  const std::string detail = fmt("V:N=%.7g/A=%.7g I1:N=%.7g/A=%.7g", ni->volume, V, nM[0], aM[0]) +
      fmt(" I3:N=%.7g/A=%.7g cErr=%.2e", nM[2], aM[2], cErr);
  if (natVsA && occVsA) record(GS5, AGREE, oblique, idx, "box", detail);
  else if (natVsA && oOk) record(GS5, ORACLE_INACCURATE, oblique, idx, "box", detail);
  else if (natVsA) record(GS5, AGREE, oblique, idx, "box", detail);
  else record(GS5, DISAGREE, oblique, idx, "box", detail);
}

// ═════════════════════════════════════════════════════════════════════════════════════════
// GS6 validity — an::checkSolidMesh(mesh) vs construction-time ground truth (+ BRepCheck on the
// valid case). The interesting signal is on INVALID solids: native must REJECT a broken mesh.
// A false clean (native calls a broken solid valid) is the silent-wrong-answer this catches.
// Broken meshes are tilted (oblique). OCCT BRepCheck is the oracle on the valid case only —
// on the closure/orientation defects BRepCheck operates on a different B-rep notion, so the
// construction ground truth is authoritative there (documented; not a DISAGREE if OCCT differs).
// ═════════════════════════════════════════════════════════════════════════════════════════
void trialGS6(Rng& r, int idx) {
  const int fam = static_cast<int>(r.below(4));  // 0 valid, 1 hole, 2 flipped-face, 3 non-finite
  const bool oblique = r.unit() < 0.75;
  const double w = r.range(1.5, 5), d = r.range(1.5, 5), h = r.range(1.5, 5);
  const Rot R = oblique ? randRot(r) : identityRot();
  const Vec3 t{r.sym(4), r.sym(4), r.sym(4)};
  ntess::Mesh mesh = boxMesh(w, d, h, R, t);
  const bool gtValid = (fam == 0);
  std::string tag;

  if (fam == 1) {                          // HOLE: drop one triangle → not watertight
    tag = "hole"; mesh.triangles.pop_back();
  } else if (fam == 2) {                    // FLIPPED: reverse one triangle's winding → not oriented
    tag = "flipped";
    ntess::Triangle& tr = mesh.triangles[static_cast<std::size_t>(r.below(
        static_cast<uint32_t>(mesh.triangles.size())))];
    std::swap(tr.b, tr.c);
  } else if (fam == 3) {                    // NON-FINITE coordinate
    tag = "non-finite";
    mesh.vertices[static_cast<std::size_t>(r.below(8))].x =
        std::numeric_limits<double>::quiet_NaN();
  } else {
    tag = "valid";
  }

  const an::ValidityReport rep = an::checkSolidMesh(mesh);
  const bool natValid = rep.valid();

  if (gtValid) {
    // Valid solid: OCCT MakeBox IsValid is the oracle; native must certify valid.
    bool occtValid = false;
    try {
      TopoDS_Shape box = BRepPrimAPI_MakeBox(w, d, h).Shape();
      BRepBuilderAPI_Transform xf(box, occtTrsf(R, t), Standard_True);
      BRepCheck_Analyzer an(xf.Shape()); occtValid = an.IsValid();
    } catch (const Standard_Failure&) { occtValid = false; }
    const std::string detail = fmt("natValid=%.0f occtValid=%.0f", double(natValid), double(occtValid));
    if (natValid && occtValid) record(GS6, AGREE, oblique, idx, tag, detail);
    else if (natValid) record(GS6, ORACLE_INACCURATE, oblique, idx, tag, detail);
    else record(GS6, DISAGREE, oblique, idx, tag, detail);  // native false-negative on a valid solid
  } else {
    // Broken solid: ground truth = invalid. Native MUST reject it (valid()==false).
    const std::string detail = std::string("GT=invalid natValid=") + (natValid ? "1" : "0") +
        " reason='" + rep.reason() + "'";
    if (!natValid) record(GS6, AGREE, oblique, idx, tag, detail);          // native correctly rejects
    else record(GS6, DISAGREE, oblique, idx, tag, detail);                 // SILENT FALSE-CLEAN
  }
}

// ═════════════════════════════════════════════════════════════════════════════════════════
// GS1 HLR — drafting::projectOrthographic on a box occluder, closed-form arbiter.
// A convex box seen from a GENERAL-POSITION (oblique/isometric) corner view has exactly 9 visible
// + 3 hidden edge segments. The OCCT HLRBRep_Algo per-segment differential lives in the facade
// parity harness native_hlr_parity.mm; this slice holds the OCCT-free core to its closed form
// under oblique views (per the design's documented mitigation for HLR segment-identity fragility).
// ═════════════════════════════════════════════════════════════════════════════════════════
void trialGS1(Rng& r, int idx) {
  const double w = r.range(2, 6), d = r.range(2, 6), h = r.range(2, 6);
  // Box occluder vertices + the 12 edges.
  std::vector<Point3> verts = {{0,0,0},{w,0,0},{w,d,0},{0,d,0},{0,0,h},{w,0,h},{w,d,h},{0,d,h}};
  std::vector<std::array<std::uint32_t,3>> tris = {
      {0,3,2},{0,2,1},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
      {3,7,6},{3,6,2},{0,4,7},{0,7,3},{1,2,6},{1,6,5}};
  std::vector<draft::EdgeIndices> edges = {
      {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};

  // General-position oblique/isometric view: all components bounded away from 0 (so no face is
  // edge-on and the 9/3 partition is unambiguous), random signs.
  const double sx = r.unit() < 0.5 ? -1 : 1, sy = r.unit() < 0.5 ? -1 : 1, sz = r.unit() < 0.5 ? -1 : 1;
  Vec3 vd{sx * r.range(0.4, 1.0), sy * r.range(0.4, 1.0), sz * r.range(0.4, 1.0)};
  vd = vd * (1.0 / nm::norm(vd));
  Vec3 up{0, 0, 1};
  if (std::fabs(nm::dot(vd, up)) > 0.9) up = Vec3{0, 1, 0};  // keep up ∦ viewDir

  draft::Occluder occ; occ.vertices = &verts; occ.triangles = &tris;
  draft::OrthographicView view; view.viewDir = Dir3{vd}; view.up = Dir3{up};
  draft::HlrParams prm;  // defaults: 64 samples/edge, exact polyhedral projection
  const draft::HlrResult res = draft::projectOrthographic(verts, edges, occ, view, prm);

  const int vis = static_cast<int>(res.visible.size());
  const int hid = static_cast<int>(res.hidden.size());
  const std::string detail = fmt("visible=%.0f hidden=%.0f (closed form: 9 visible + 3 hidden)",
                                 double(vis), double(hid));
  // Closed-form arbiter: convex box from a general corner view → 9 visible + 3 hidden.
  if (vis == 9 && hid == 3) record(GS1, AGREE, /*oblique*/ true, idx, "box-iso", detail);
  else record(GS1, DISAGREE, /*oblique*/ true, idx, "box-iso", detail);
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x6E5A11C07Dull;
  int rounds = 40;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) rounds = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) rounds = std::atoi(e);
  if (rounds <= 0) rounds = 40;
  g_seed = seed;

  std::printf("== M6-breadth-7 differential-fuzz: native GEOMETRY-SERVICES vs OCCT ==\n");
  std::printf("== seed=0x%llx rounds=%d services={GS3 dist, GS4 curv, GS2 sect, GS5 inertia, GS6 valid, GS1 HLR} ==\n",
              static_cast<unsigned long long>(seed), rounds);
  std::printf("== fixed tol: dist=%.0e curv=%.0e sectRel=%.0e inertRel=%.0e (never widened); closed-form arbiter primary ==\n",
              kTolDist, kTolCurv, kTolSecRel, kTolInertRel);

  // Determinism self-check (task 1.2): a fixed seed reproduces the stream byte-for-byte.
  { Rng a(seed), b(seed); bool same = true;
    for (int i = 0; i < 64; ++i) if (a.next() != b.next()) { same = false; break; }
    std::printf("== determinism self-check: %s (splitmix64 -> xoshiro256**, keyed only by FUZZ_SEED) ==\n",
                same ? "PASS" : "FAIL");
    if (!same) { std::fflush(stdout); std::_Exit(2); } }
  std::fflush(stdout);

  Rng rng(seed);
  int idx = 0;
  // Per-service sub-generators draw from the shared stream in a FIXED order each round, so a seed
  // reproduces every service's inputs and every service gets real per-round coverage.
  for (int round = 0; round < rounds; ++round) {
    trialGS3(rng, idx++);
    trialGS4(rng, idx++);
    trialGS2(rng, idx++);
    trialGS5(rng, idx++);
    trialGS6(rng, idx++);
    trialGS1(rng, idx++);
  }

  // ── coverage summary ─────────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx rounds=%d) ==\n",
              static_cast<unsigned long long>(seed), rounds);
  std::printf("   per-service [AGREE(oblique)/HONEST-DECLINE/DISAGREE/ORACLE-INACC/BOTH-DECLINE]:\n");
  int totalDisagree = 0;
  bool coverage = true;
  for (int s = 0; s < SVC_COUNT; ++s) {
    std::printf("     %-15s %d(%d)/%d/%d/%d/%d\n", svcName(s), g_agree[s], g_agreeOblique[s],
                g_decline[s], g_disagree[s], g_oracleInacc[s], g_bothDecl[s]);
    totalDisagree += g_disagree[s];
    if (g_agree[s] < 1 || g_agreeOblique[s] < 1) coverage = false;  // ≥1 AGREE incl. an oblique trial
  }
  std::printf("   HONEST-SCOPE: GS2 OBLIQUE-CYLINDER cut is the ssi/plane_conics exemplar → HONEST-NATIVE-DECLINE\n"
              "                 (native routes around the inverted oblique-ellipse semi-major; OCCT answers). Auto-\n"
              "                 reclassifies AGREE once the upstream ssi/ fix lands. GS6 broken-solid arbiter is the\n"
              "                 construction ground truth (OCCT BRepCheck is the oracle on the VALID case only; on\n"
              "                 closure/orientation defects it tests a different B-rep notion — documented, not a DISAGREE).\n"
              "                 GS1 OCCT-HLRBRep per-segment differential lives in native_hlr_parity.mm; this slice holds\n"
              "                 the OCCT-free HLR core to its box-corner closed form (9 visible + 3 hidden) under oblique views.\n");

  const bool bar = (totalDisagree == 0 && coverage);
  std::printf("== M6-breadth-7 BAR: %s (DISAGREE=%d must be 0; per-service AGREE+oblique coverage=%s) ==\n",
              bar ? "PASS — zero silent wrong answers across the GS layer" : "FAIL",
              totalDisagree, coverage ? "complete" : "INCOMPLETE");
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
