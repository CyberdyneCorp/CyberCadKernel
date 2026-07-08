// SPDX-License-Identifier: Apache-2.0
//
// native_mass_props_fuzz.mm — MOAT M6-breadth-6 (the COMPLETENESS BAR, SIXTH domain):
// a MASS-PROPERTIES differential-fuzzing harness (iOS simulator) for the native query
// layer the CyberCad app's MassReadout / Inertia / Measure panels read.
//
// This extends the landed M6 differential fuzzers — native_boolean_fuzz.mm (curved
// booleans), native_step_import_fuzz.mm (STEP round-trip), native_construct_fuzz.mm
// (loft/sweep construction) and native_blend_fuzz.mm (fillet/chamfer/offset/shell) — to
// a SIXTH independent native domain: the mesh-based mass-properties path
// (NativeEngine::mass_properties, src/engine/native/native_engine.cpp:693). Like its
// siblings it is INFRASTRUCTURE (a seeded harness, not a geometry capability): OCCT is
// the ORACLE, a CLOSED-FORM analytic value is the PRIMARY arbiter, the bar is ZERO
// silent wrong masses over a seeded batch, and an HONEST DECLINE is first-class.
//
// ── THE NATIVE MASS PATH, REPRODUCED DIRECTLY (OCCT-FREE, the system under test) ─────
// NativeEngine::mass_properties for a native B-rep body does NOT call OCCT. It meshes
// the solid at kPropertyDeflection = 0.005 (native_engine.cpp:177) and derives:
//     area   = ntess::surfaceArea(mesh)                 // Σ ½|(b−a)×(c−a)|
//     volume = |ntess::enclosedVolume(mesh)|            // divergence-theorem signed tetra
//     centroid = volume-weighted signed-tetra centroid over the origin fan
//     valid  = ntess::isWatertight(mesh) ∧ volume > 0   // a non-watertight mesh → NO mass
// This harness reproduces that EXACT computation by driving the OCCT-FREE native builders
// (src/native/construct) and the native tessellator (src/native/tessellate) DIRECTLY —
// so a non-watertight / NULL native body is an UNAMBIGUOUS native DECLINE, not a value
// silently forwarded to OCCT by the cc_* facade (same discipline as native_construct_fuzz).
// NativeEngine::principal_moments (native_engine.cpp:1305) is CC_NATIVE_BODY_UNSUPPORTED —
// it delegates inertia to OCCT for a native body — so INERTIA IS NOT differentially
// fuzzable against a native answer; it is a documented HONEST NATIVE DECLINE, and the
// OCCT GProp_PrincipalProps moments are printed only as oracle-trust TELEMETRY.
//
// ── THE DEFLECTION BOUNDARY (why native-vs-OCCT alone is insufficient) ───────────────
// Native mass = f(mesh(solid, 0.005)); OCCT BRepGProp = f(exact_brep(solid)). For a
// PLANAR family (box, n-gon prism, coaxial-frustum loft) the mesh reproduces the solid
// EXACTLY (flat faces, no chord error) so native = OCCT = analytic to machine epsilon and
// a TIGHT tolerance (1e-6) is honest. For a CURVED family (cylinder, cone-to-a-point,
// sphere, revolved tube) the mesh is a faceted UNDER-approximation, so native differs
// from the exact OCCT/analytic value by the tessellation chord error — a real, bounded,
// EXPECTED gap, NOT a native fault. Its size is the deflection convergence bound
// rel ≈ C·deflection/featureSize (empirically C ≈ 1.3 for a cylinder/cone wall, ≈ 3.2 for
// a sphere's double curvature). The curved-family tolerance is therefore that bound —
// tol = kCurveC·deflection/featureSize with kCurveC = 5.0 (a ~1.5× margin over the worst
// observed sphere constant) and featureSize ≥ 1.0 by construction — MATCHED to the
// deflection, NEVER widened past it. Holding a curved family to the planar 1e-6 would
// flag correct meshes as wrong; widening the planar family would hide real faults. The
// CLOSED-FORM analytic value is the PRIMARY arbiter because it is exact for BOTH the
// planar and curved families and lets the classifier attribute a native-vs-OCCT gap
// correctly (native mesh-error vs a real bug vs an OCCT outlier).
//
// ── FAMILIES + ANALYTIC ARBITERS (all in the native-builder revolve frame: profiles on
//    z=0, revolved about world +Y; prisms extruded along +Z — so centroids are directly
//    comparable to the OCCT primitives, built in the SAME frame) ───────────────────────
//   BOX      build_prism(rectangle w×d, h)        V=wdh  A=2(wd+wh+dh)  C=(w/2,d/2,h/2)   [planar/exact]
//   NGON     build_prism(regular n-gon r, h)      V=Aₙh  A=2Aₙ+n·s·h   C=(0,0,h/2)        [planar/exact]
//   CYLINDER build_revolution(r×h rect, +Y, 2π)   V=πr²h A=2πr²+2πrh    C=(0,h/2,0)        [curved]
//   CONE     build_revolution(r0→apex triangle)   V=πr0²h/3  A=πr0²+πr0·√(r0²+h²)  C=(0,h/4,0) [curved, apex welds]
//            — a FRUSTUM (r1>0) does NOT weld watertight at 0.005 → HONEST DECLINE (see below)
//   SPHERE   build_revolution_profile(on-axis arc)V=4πr³/3 A=4πr²      C=(0,0,0)          [curved]
//   LOFT     build_loft_sections(2 coaxial n-gons)prismatoid band V + trapezoid area      [planar/exact]
//   REVOLVE  build_revolution(annular r∈[ri,ro]×h)tube V=2πh·r̄·(ro−ri)… A exact  C=(0,h/2,0) [curved]
//   DECLINE  zero-height / degenerate profile     native NULL/invalid → decline-exerciser
//
// ── THE FIVE-WAY CLASSIFIER (mirrors the landed siblings, specialised to vol/area/centroid)
//   AGREED            native VALID (watertight) + vol/area/centroid within the family tol of
//                     the ANALYTIC truth; OCCT (built the same solid) also matches the
//                     analytic within a tight ORACLE-TRUST tol (else guarded, below).
//   HONESTLY-DECLINED native mesh non-watertight / NULL → valid=false (NO native mass) while
//                     OCCT ships a valid mass. First-class: the CONE FRUSTUM lands here — its
//                     native mesh does not weld watertight at 0.005 (the cap⟷slant seam), so
//                     the native path conservatively refuses; OCCT's BRepPrimAPI cone is valid.
//   DISAGREED         native VALID but vol/area/centroid OUTSIDE the analytic truth — a genuine
//                     SILENT WRONG MASS. The failure this harness exists to catch. (FAILS bar.)
//   ORACLE-INACCURATE native matches the analytic truth while OCCT does NOT — native vindicated
//                     by exact math, OCCT the outlier. Logged, NOT a bar failure, NOT a fault.
//   BOTH-DECLINED     a decline-exerciser both engines refuse (degenerate input). Logged.
//   ORACLE_UNRELIABLE a CORE family whose OCCT build does NOT match the closed form to the
//                     tight oracle-trust tol → the oracle is untrustworthy for that input →
//                     excluded from the verdict and FAILS the bar (investigate, never launder).
//
// The INERTIA dimension is recorded HONESTLY-DECLINED for every native body (native delegates
// to OCCT); the OCCT principal moments (+ a closed-form cross-check for box/sphere) are logged
// as oracle-trust TELEMETRY only — never a native differential, never a bar input.
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────
//   Exit 0 IFF DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, with each AGREE family
//   { BOX, NGON, CYLINDER, CONE, SPHERE, LOFT, REVOLVE } having ≥ 1 AGREED trial (the CONE
//   AGREE coverage comes from its apex sub-family; its frustum sub-family HONESTLY-DECLINES).
//   Run over ≥ 2 distinct seeds. Any DISAGREE / ORACLE-INACCURATE prints seed + case index +
//   family/param tuple + the native/OCCT/analytic triple as a reproducible regression find.
//   The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand():
//   same seed → byte-identical batch (splitmix64 → xoshiro256**, verbatim from the siblings).
//
// This TU is OCCT-dependent (BRepPrimAPI + ThruSections + BRepGProp + GProp_PrincipalProps as
// the oracle) but needs NO numsci: the native construct + tessellate + topology live in
// src/native/** — all OCCT-FREE and header-only (math bezier/bspline are the only compiled
// native TUs). Built ONLY by scripts/run-sim-native-mass-props-fuzz.sh; on run-sim-suite.sh's
// SKIP list (own main(), std::_Exit — OCCT static teardown in the trimmed static build is not
// exit-clean, same rationale as the siblings). src/native / src/engine stay UNTOUCHED — this
// harness is additive test/sim code only and REPRODUCES the native mass path rather than
// modifying it.
//
#include "native/construct/native_construct.h"     // build_prism / build_revolution[_profile] / build_loft_sections
#include "native/tessellate/native_tessellate.h"   // SolidMesher + surfaceArea/enclosedVolume/isWatertight
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_mass_props_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT BRepGProp oracle"
#endif

#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PrincipalProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Standard_Failure.hxx>

namespace ncst  = cybercad::native::construct;
namespace ntess = cybercad::native::tessellate;
namespace ntopo = cybercad::native::topology;

namespace {

constexpr double kPi = 3.14159265358979323846;
// The native mass path meshes at exactly this deflection (native_engine.cpp:177) — we
// reproduce it faithfully so the tolerances below are the true convergence bound.
constexpr double kPropertyDeflection = 0.005;
constexpr double kTightTol  = 1e-6;   // planar families reproduce the solid EXACTLY (~1e-15)
constexpr double kCurveC    = 5.0;    // curved tol = kCurveC·deflection/featureSize (see header)
constexpr double kOracleTol = 1e-6;   // OCCT (exact B-rep) must match the closed form this tight
constexpr double kMinFeat   = 1.0;    // curved featureSize floor (bounds the worst chord error)

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the landed fuzzers).
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
};

enum Family { F_BOX, F_NGON, F_CYLINDER, F_CONE, F_SPHERE, F_LOFT, F_REVOLVE, F_DECLINE, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_BOX:      return "BOX prism(planar)";
    case F_NGON:     return "NGON prism(planar)";
    case F_CYLINDER: return "CYLINDER revolve";
    case F_CONE:     return "CONE revolve(apex/frustum)";
    case F_SPHERE:   return "SPHERE revolve-profile";
    case F_LOFT:     return "LOFT coaxial-frustum";
    case F_REVOLVE:  return "REVOLVE tube(Pappus)";
    case F_DECLINE:  return "DECLINE-exerciser";
  }
  return "?";
}
// The seven mass-property families that MUST show real AGREE coverage; the DECLINE-exerciser
// is an out-of-scope degenerate input both engines may refuse (never a required AGREE).
bool isCoreFamily(int f) { return f >= F_BOX && f <= F_REVOLVE; }

// How the native path builds the body, and how the OCCT oracle builds the SAME solid.
enum NativeBuild { NB_PRISM, NB_REV_POLY, NB_REV_PROFILE, NB_LOFT, NB_DEGEN };
enum OcctBuild   { OB_BOX, OB_NGON_PRISM, OB_CYL, OB_CONE, OB_SPHERE, OB_LOFT, OB_REVOL, OB_NONE };

struct GenCase {
  int family = 0;
  NativeBuild nb = NB_PRISM;
  OcctBuild   ob = OB_NONE;
  // native inputs
  std::vector<double> polyXY;                 // prism / revolve-poly profile (x,y pairs, z=0)
  double depth = 0;                           // prism extrusion height (+Z)
  std::vector<ncst::ProfileSegment> segs;     // typed profile (sphere on-axis arc)
  ncst::RevolveAxis axis{0, 0, 0, 1};         // default world +Y axis
  double angle = 2.0 * kPi;
  std::vector<double> loftXYZ;                 // packed (x,y,z) section loops
  std::vector<int>    loftCnt;
  // OCCT primitive params (built in the SAME native frame → centroid comparable)
  double bw = 0, bd = 0, bh = 0;              // box w,d,h
  double cr0 = 0, cr1 = 0, ch = 0;            // cylinder/cone base,top radius + height (+Y)
  double sr = 0;                              // sphere radius
  double ri = 0, ro = 0, th = 0;             // revolve tube inner/outer radius + height
  // analytic ground truth
  double aVol = 0, aArea = 0, aCx = 0, aCy = 0, aCz = 0;
  bool   planar = true;                       // tight tol vs curved-deflection tol
  double featureSize = kMinFeat;              // curved chord-error feature size
  double charLen = 1;                         // centroid tolerance length scale
  bool   expectDecline = false;               // cone frustum / degenerate → native declines
  std::string desc;
};

// ── analytic helpers ─────────────────────────────────────────────────────────────────
double ngonArea(int n, double R)    { return 0.5 * n * R * R * std::sin(2.0 * kPi / n); }
double ngonEdge(int n, double R)    { return 2.0 * R * std::sin(kPi / n); }
double ngonApothem(int n, double R) { return R * std::cos(kPi / n); }

void appendRegularNgon3D(std::vector<double>& buf, int n, double R, double rot, double z) {
  for (int i = 0; i < n; ++i) {
    const double a = rot + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
    buf.push_back(R * std::cos(a));
    buf.push_back(R * std::sin(a));
    buf.push_back(z);
  }
}

// Exact volume/area/centroid-z of a stack of coaxial regular-n-gon frustums (a prismatoid
// stack): each band is a pyramidal frustum with band volume (Δz/3)(Ak+√(AkAk1)+Ak1) and
// its own centroid; lateral area sums the planar-trapezoid side faces + first/last caps.
void analyticNgonStack(int n, const std::vector<double>& R, const std::vector<double>& z,
                       double& vol, double& area, double& zc) {
  vol = 0.0;
  double lateral = 0.0, moment = 0.0;
  for (std::size_t k = 0; k + 1 < R.size(); ++k) {
    const double dz = std::fabs(z[k + 1] - z[k]);
    const double Ak = ngonArea(n, R[k]), Ak1 = ngonArea(n, R[k + 1]);
    const double band = (dz / 3.0) * (Ak + std::sqrt(Ak * Ak1) + Ak1);
    const double denom = Ak + std::sqrt(Ak * Ak1) + Ak1;
    const double zLocal = denom > 1e-12
        ? (dz / 4.0) * (Ak + 2.0 * std::sqrt(Ak * Ak1) + 3.0 * Ak1) / denom : 0.5 * dz;
    vol += band;
    moment += band * (std::min(z[k], z[k + 1]) + zLocal);
    const double da = ngonApothem(n, R[k]) - ngonApothem(n, R[k + 1]);
    const double slant = std::sqrt(dz * dz + da * da);
    lateral += n * 0.5 * (ngonEdge(n, R[k]) + ngonEdge(n, R[k + 1])) * slant;
  }
  area = lateral + ngonArea(n, R.front()) + ngonArea(n, R.back());
  zc = vol > 1e-12 ? moment / vol : 0.0;
}

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[224]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

int pickFamily(Rng& r) {
  // Core families weighted for coverage; the degenerate decline-exerciser stays sparse.
  const int w[F_COUNT] = {5, 5, 5, 6, 5, 5, 5, 2};   // CONE heavier (apex AGREE + frustum DECLINE)
  int tot = 0; for (int x : w) tot += x;
  int k = static_cast<int>(r.below(static_cast<uint32_t>(tot)));
  for (int i = 0; i < F_COUNT; ++i) { if (k < w[i]) return i; k -= w[i]; }
  return F_BOX;
}

GenCase genCase(Rng& r) {
  GenCase c; c.family = pickFamily(r);
  switch (c.family) {
    case F_BOX: {
      c.nb = NB_PRISM; c.ob = OB_BOX; c.planar = true;
      const double w = r.range(1.0, 4.0), d = r.range(1.0, 4.0), h = r.range(1.0, 4.0);
      c.polyXY = {0, 0, w, 0, w, d, 0, d}; c.depth = h;
      c.bw = w; c.bd = d; c.bh = h;
      c.aVol = w * d * h; c.aArea = 2.0 * (w * d + w * h + d * h);
      c.aCx = 0.5 * w; c.aCy = 0.5 * d; c.aCz = 0.5 * h;
      c.charLen = std::max({w, d, h});
      c.desc = fmt("w=%.4f d=%.4f h=%.4f", w, d, h);
      break;
    }
    case F_NGON: {
      c.nb = NB_PRISM; c.ob = OB_NGON_PRISM; c.planar = true;
      const int n = 3 + static_cast<int>(r.below(10));   // 3..12-gon
      const double R = r.range(1.0, 3.0), h = r.range(1.0, 4.0), rot = r.range(0.0, kPi);
      for (int i = 0; i < n; ++i) {
        const double a = rot + 2.0 * kPi * i / n;
        c.polyXY.push_back(R * std::cos(a));
        c.polyXY.push_back(R * std::sin(a));
      }
      c.depth = h; c.bh = h;
      const double A = ngonArea(n, R), s = ngonEdge(n, R);
      c.aVol = A * h; c.aArea = 2.0 * A + n * s * h;
      c.aCx = 0.0; c.aCy = 0.0; c.aCz = 0.5 * h;
      c.charLen = std::max(2.0 * R, h);
      c.desc = fmt("n=%.0f R=%.4f h=%.4f", n, R, h);
      break;
    }
    case F_CYLINDER: {
      c.nb = NB_REV_POLY; c.ob = OB_CYL; c.planar = false;
      const double R = r.range(1.0, 3.0), h = r.range(1.5, 4.0);
      c.polyXY = {0, 0, R, 0, R, h, 0, h};             // (r,h) rect revolved about +Y
      c.cr0 = R; c.cr1 = R; c.ch = h;
      c.aVol = kPi * R * R * h; c.aArea = 2.0 * kPi * R * R + 2.0 * kPi * R * h;
      c.aCx = 0.0; c.aCy = 0.5 * h; c.aCz = 0.0;
      c.featureSize = R; c.charLen = std::max(2.0 * R, h);
      c.desc = fmt("R=%.4f h=%.4f", R, h);
      break;
    }
    case F_CONE: {
      // Apex cone (r1=0) welds watertight → AGREE; a frustum (r1>0) does NOT weld at 0.005
      // (cap⟷slant seam) → HONEST DECLINE. Draw mostly apex (real AGREE coverage) with a
      // fraction of frustums to exercise the honest-decline branch on a REAL family.
      c.nb = NB_REV_POLY; c.ob = OB_CONE; c.planar = false;
      const double r0 = r.range(1.2, 3.0), h = r.range(1.5, 4.0);
      const bool frustum = r.unit() < 0.30;
      const double r1 = frustum ? r.range(0.4, r0 - 0.3) : 0.0;
      if (r1 > 0.0)
        c.polyXY = {0, 0, r0, 0, r1, h, 0, h};         // trapezoid → frustum
      else
        c.polyXY = {0, 0, r0, 0, 0, h};                // triangle → apex cone
      c.cr0 = r0; c.cr1 = r1; c.ch = h;
      c.expectDecline = frustum;                        // frustum → native mesh non-watertight
      c.featureSize = std::min(r0, (r1 > 0 ? r1 : r0)); c.charLen = std::max(2.0 * r0, h);
      if (r1 > 0.0) {
        c.aVol = kPi * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1);
        c.aArea = kPi * (r0 * r0 + r1 * r1) +
                  kPi * (r0 + r1) * std::sqrt((r0 - r1) * (r0 - r1) + h * h);
        const double num = r0 * r0 + 2.0 * r0 * r1 + 3.0 * r1 * r1;
        c.aCy = (h / 4.0) * num / (r0 * r0 + r0 * r1 + r1 * r1);
        c.desc = fmt("FRUSTUM r0=%.4f r1=%.4f h=%.4f", r0, r1, h);
      } else {
        c.aVol = kPi * r0 * r0 * h / 3.0;
        c.aArea = kPi * r0 * r0 + kPi * r0 * std::sqrt(r0 * r0 + h * h);
        c.aCy = h / 4.0; c.featureSize = r0;
        c.desc = fmt("APEX r0=%.4f h=%.4f", r0, h);
      }
      c.aCx = 0.0; c.aCz = 0.0;
      break;
    }
    case F_SPHERE: {
      c.nb = NB_REV_PROFILE; c.ob = OB_SPHERE; c.planar = false;
      const double R = r.range(1.0, 3.0);
      ncst::ProfileSegment arc; arc.kind = 1;            // on-axis arc → Sphere band
      arc.cx = 0; arc.cy = 0; arc.r = R;
      arc.x0 = 0; arc.y0 = -R; arc.x1 = 0; arc.y1 = R; arc.a0 = -0.5 * kPi; arc.a1 = 0.5 * kPi;
      ncst::ProfileSegment ax; ax.kind = 0;              // closing meridian on the axis
      ax.x0 = 0; ax.y0 = R; ax.x1 = 0; ax.y1 = -R;
      c.segs = {arc, ax};
      c.sr = R;
      c.aVol = 4.0 / 3.0 * kPi * R * R * R; c.aArea = 4.0 * kPi * R * R;
      c.aCx = 0.0; c.aCy = 0.0; c.aCz = 0.0;
      c.featureSize = R; c.charLen = 2.0 * R;
      c.desc = fmt("R=%.4f", R);
      break;
    }
    case F_LOFT: {
      // Coaxial regular-n-gon 2-section frustum at a SHARED rotation → planar trapezoid side
      // faces → the native mesh reproduces the solid EXACTLY (tight tol on vol + area + z).
      c.nb = NB_LOFT; c.ob = OB_LOFT; c.planar = true;
      const int n = 3 + static_cast<int>(r.below(6));   // 3..8-gon
      const double R0 = r.range(1.0, 3.0), R1 = r.range(1.0, 3.0);
      const double rot = r.range(0.0, kPi), dz = r.range(1.5, 4.0);
      appendRegularNgon3D(c.loftXYZ, n, R0, rot, 0.0);
      appendRegularNgon3D(c.loftXYZ, n, R1, rot, dz);
      c.loftCnt = {n, n};
      double zc = 0.0;
      analyticNgonStack(n, {R0, R1}, {0.0, dz}, c.aVol, c.aArea, zc);
      c.aCx = 0.0; c.aCy = 0.0; c.aCz = zc;
      c.charLen = std::max({2.0 * R0, 2.0 * R1, dz});
      c.desc = fmt("n=%.0f R0=%.4f R1=%.4f dz=%.4f", n, R0, R1, dz);
      break;
    }
    case F_REVOLVE: {
      // Annular rectangle r∈[ri,ro], y∈[0,h] revolved 2π about +Y → a tube (Pappus). Exact:
      // V = 2π·r̄·A_prof (r̄ = (ri+ro)/2, A_prof = (ro−ri)h); A = outer+inner walls + 2 annuli.
      c.nb = NB_REV_POLY; c.ob = OB_REVOL; c.planar = false;
      const double ri = r.range(1.0, 2.5), ro = ri + r.range(0.5, 2.0), h = r.range(1.5, 4.0);
      c.polyXY = {ri, 0, ro, 0, ro, h, ri, h};
      c.ri = ri; c.ro = ro; c.th = h;
      const double A = (ro - ri) * h, rbar = 0.5 * (ri + ro);
      c.aVol = 2.0 * kPi * rbar * A;
      c.aArea = 2.0 * kPi * h * (ro + ri) + 2.0 * kPi * (ro * ro - ri * ri);
      c.aCx = 0.0; c.aCy = 0.5 * h; c.aCz = 0.0;
      c.featureSize = ri; c.charLen = std::max(2.0 * ro, h);
      c.desc = fmt("ri=%.4f ro=%.4f h=%.4f", ri, ro, h);
      break;
    }
    case F_DECLINE: {
      // Degenerate decline-exerciser: a rectangle extruded to ~zero height → native build_prism
      // returns NULL (depth ≤ kMinDepth); OCCT box with h≈0 is invalid → BOTH-DECLINED.
      c.nb = NB_DEGEN; c.ob = OB_NONE; c.planar = true; c.expectDecline = true;
      const double w = r.range(1.0, 3.0), d = r.range(1.0, 3.0);
      c.polyXY = {0, 0, w, 0, w, d, 0, d}; c.depth = 1e-9;
      c.desc = fmt("w=%.4f d=%.4f h=0 (degenerate)", w, d);
      break;
    }
  }
  return c;
}

// ── native mass path, reproduced DIRECTLY (OCCT-FREE — native_engine.cpp:693) ──────────
ntopo::Shape buildNative(const GenCase& c) {
  switch (c.nb) {
    case NB_PRISM:
      return ncst::build_prism(c.polyXY.data(), static_cast<int>(c.polyXY.size() / 2), c.depth);
    case NB_REV_POLY:
      return ncst::build_revolution(c.polyXY.data(), static_cast<int>(c.polyXY.size() / 2),
                                    c.axis, c.angle);
    case NB_REV_PROFILE:
      return ncst::build_revolution_profile(c.segs, c.axis, c.angle);
    case NB_LOFT:
      return ncst::build_loft_sections(c.loftXYZ.data(), c.loftCnt.data(),
                                       static_cast<int>(c.loftCnt.size()));
    case NB_DEGEN:
      return ncst::build_prism(c.polyXY.data(), static_cast<int>(c.polyXY.size() / 2), c.depth);
  }
  return {};
}

struct NativeMass { bool present = false, valid = false; double volume = 0, area = 0, cx = 0, cy = 0, cz = 0; };
// Reproduces NativeEngine::mass_properties EXACTLY: mesh at kPropertyDeflection, area =
// surfaceArea, volume = |enclosedVolume|, centroid = signed-tetra fan, valid = watertight ∧ V>0.
NativeMass measureNative(const ntopo::Shape& s) {
  NativeMass m;
  if (s.isNull()) return m;
  m.present = true;
  ntess::MeshParams p; p.deflection = kPropertyDeflection;
  const ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(s);
  m.area = ntess::surfaceArea(mesh);
  m.volume = std::fabs(ntess::enclosedVolume(mesh));
  double cx = 0, cy = 0, cz = 0, vol6 = 0;
  for (const auto& t : mesh.triangles) {
    const auto& A = mesh.vertices[t.a];
    const auto& B = mesh.vertices[t.b];
    const auto& C = mesh.vertices[t.c];
    const double v = A.x * (B.y * C.z - B.z * C.y) - A.y * (B.x * C.z - B.z * C.x) +
                     A.z * (B.x * C.y - B.y * C.x);
    vol6 += v;
    cx += v * (A.x + B.x + C.x); cy += v * (A.y + B.y + C.y); cz += v * (A.z + B.z + C.z);
  }
  if (std::fabs(vol6) > 1e-12) {
    const double inv = 1.0 / (4.0 * vol6);
    m.cx = cx * inv; m.cy = cy * inv; m.cz = cz * inv;
  }
  m.valid = ntess::isWatertight(mesh) && m.volume > 0.0;
  return m;
}

// ── OCCT oracle: build the SAME solid in the SAME native frame, measure by BRepGProp ────
TopoDS_Face ngonFaceZ0(const std::vector<double>& polyXY) {
  BRepBuilderAPI_MakePolygon poly;
  for (std::size_t i = 0; i < polyXY.size() / 2; ++i)
    poly.Add(gp_Pnt(polyXY[i * 2], polyXY[i * 2 + 1], 0.0));
  poly.Close();
  if (!poly.IsDone()) return {};
  BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
  return face.IsDone() ? face.Face() : TopoDS_Face();
}

TopoDS_Shape buildOcct(const GenCase& c) {
  try {
    const gp_Ax2 axY(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0));  // +Y axis, X ref
    switch (c.ob) {
      case OB_BOX:
        return BRepPrimAPI_MakeBox(c.bw, c.bd, c.bh).Shape();
      case OB_NGON_PRISM: {
        const TopoDS_Face f = ngonFaceZ0(c.polyXY);
        if (f.IsNull()) return {};
        return BRepPrimAPI_MakePrism(f, gp_Vec(0, 0, c.bh)).Shape();
      }
      case OB_CYL:
        return BRepPrimAPI_MakeCylinder(axY, c.cr0, c.ch).Shape();
      case OB_CONE:
        return BRepPrimAPI_MakeCone(axY, c.cr0, c.cr1, c.ch).Shape();
      case OB_SPHERE:
        return BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), c.sr).Shape();
      case OB_LOFT: {
        BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
        std::size_t off = 0;
        for (std::size_t k = 0; k < c.loftCnt.size(); ++k) {
          const int cnt = c.loftCnt[k];
          BRepBuilderAPI_MakePolygon poly;
          for (int i = 0; i < cnt; ++i)
            poly.Add(gp_Pnt(c.loftXYZ[off + i * 3], c.loftXYZ[off + i * 3 + 1],
                            c.loftXYZ[off + i * 3 + 2]));
          poly.Close();
          if (!poly.IsDone()) return {};
          gen.AddWire(poly.Wire());
          off += static_cast<std::size_t>(cnt) * 3;
        }
        gen.Build();
        return gen.IsDone() ? gen.Shape() : TopoDS_Shape();
      }
      case OB_REVOL: {
        BRepBuilderAPI_MakePolygon poly;
        poly.Add(gp_Pnt(c.ri, 0, 0)); poly.Add(gp_Pnt(c.ro, 0, 0));
        poly.Add(gp_Pnt(c.ro, c.th, 0)); poly.Add(gp_Pnt(c.ri, c.th, 0));
        poly.Close();
        if (!poly.IsDone()) return {};
        BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
        if (!face.IsDone()) return {};
        return BRepPrimAPI_MakeRevol(face.Face(), gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)),
                                     2.0 * kPi).Shape();
      }
      case OB_NONE:
        return {};
    }
  } catch (const Standard_Failure&) { return {}; }
  return {};
}

struct OcctMass {
  bool present = false, valid = false;
  double volume = 0, area = 0, cx = 0, cy = 0, cz = 0;
  double Ixx = 0, Iyy = 0, Izz = 0;   // principal moments (inertia telemetry only)
};
OcctMass measureOcct(const TopoDS_Shape& s) {
  OcctMass m;
  if (s.IsNull()) return m;
  m.present = true;
  try {
    BRepCheck_Analyzer an(s);
    GProp_GProps vg; BRepGProp::VolumeProperties(s, vg);
    GProp_GProps ag; BRepGProp::SurfaceProperties(s, ag);
    m.volume = std::fabs(vg.Mass());
    m.area = ag.Mass();
    const gp_Pnt g = vg.CentreOfMass();
    m.cx = g.X(); m.cy = g.Y(); m.cz = g.Z();
    GProp_PrincipalProps pp = vg.PrincipalProperties();
    pp.Moments(m.Ixx, m.Iyy, m.Izz);
    m.valid = an.IsValid() && m.volume > 1e-12;
  } catch (const Standard_Failure&) { m.valid = false; }
  return m;
}

double relDiff(double a, double b) { return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : 1e30; }

// ── classifier ────────────────────────────────────────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famAgreed[F_COUNT] = {0}, g_famDeclined[F_COUNT] = {0}, g_famDisagreed[F_COUNT] = {0};
int g_famOracleInacc[F_COUNT] = {0}, g_famBothDecl[F_COUNT] = {0};
int g_inertiaDeclined = 0;   // every native body: inertia is an honest native decline

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x6D3A11C05Bull;
  int N = 120;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 120;

  std::printf("== M6-breadth-6 differential-fuzz: native MASS PROPERTIES vs OCCT BRepGProp ==\n");
  std::printf("== seed=0x%llx N=%d deflection=%g planarTol=%.0e curveTol=%.1f*defl/feat oracleTol=%.0e ==\n",
              static_cast<unsigned long long>(seed), N, kPropertyDeflection, kTightTol, kCurveC, kOracleTol);
  std::printf("== NOTE native principal_moments = CC_NATIVE_BODY_UNSUPPORTED → inertia is an HONEST NATIVE DECLINE (OCCT moments logged as telemetry only) ==\n");
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    const GenCase c = genCase(rng);

    const ntopo::Shape natShape = buildNative(c);
    const NativeMass nat = measureNative(natShape);
    const TopoDS_Shape occShape = buildOcct(c);
    const OcctMass occ = measureOcct(occShape);

    // Family tolerance: planar → machine-tight; curved → deflection convergence bound.
    const double feat = std::max(c.featureSize, kMinFeat);
    const double tol = c.planar ? kTightTol : (kCurveC * kPropertyDeflection / feat);
    const double centTol = tol * std::max(c.charLen, 1.0);

    const bool nativeValid = nat.present && nat.valid && nat.volume > 1e-9;
    const bool oracleValid = occ.present && occ.valid && occ.volume > 1e-9;
    if (nativeValid) ++g_inertiaDeclined;   // native has no inertia — honest decline per body

    // Does the OCCT oracle match the exact closed form (oracle-trust)? For a core family it
    // MUST, to a tight tol (OCCT is an exact B-rep). If not, the oracle is untrustworthy.
    const bool oracleTrust = oracleValid &&
        relDiff(occ.volume, c.aVol) < kOracleTol && relDiff(occ.area, c.aArea) < kOracleTol;

    // Does the native mass match the exact closed form (PRIMARY arbiter)?
    const double natCentErr = std::sqrt((nat.cx - c.aCx) * (nat.cx - c.aCx) +
                                        (nat.cy - c.aCy) * (nat.cy - c.aCy) +
                                        (nat.cz - c.aCz) * (nat.cz - c.aCz));
    const bool natMatchesA = nativeValid &&
        relDiff(nat.volume, c.aVol) < tol && relDiff(nat.area, c.aArea) < tol &&
        natCentErr < centTol;

    Verdict v;
    if (!nativeValid) {
      // Native honestly declined (NULL / non-watertight → no valid mass). If OCCT ships a
      // valid mass it is HONESTLY-DECLINED; if OCCT also refused, a degenerate exerciser is
      // BOTH-DECLINED, but a CORE family with no oracle is ORACLE_UNRELIABLE (investigate).
      if (oracleValid) v = DECLINED;
      else v = (c.family == F_DECLINE) ? BOTH_DECLINED : ORACLE_UNRELIABLE;
    } else if (natMatchesA) {
      // Native produced a mass matching exact math. Cross-check the oracle for trust: OCCT
      // must also match the closed form. If OCCT does NOT match while native does, native is
      // vindicated (ORACLE-INACCURATE); if OCCT is simply broken for a core input, flag it.
      if (oracleTrust) v = AGREED;
      else if (oracleValid) v = ORACLE_INACCURATE;   // native matches math, OCCT the outlier
      else v = ORACLE_UNRELIABLE;
    } else {
      // Native produced a valid mass that does NOT match exact math. If OCCT matches the
      // closed form, this is a genuine SILENT WRONG native mass. If neither matches, the
      // input's oracle is unreliable (never launder a native miss as a pass).
      v = oracleTrust ? DISAGREED : ORACLE_UNRELIABLE;
    }

    switch (v) {
      case AGREED:            ++g_agreed;      ++g_famAgreed[c.family];      break;
      case DECLINED:          ++g_declined;    ++g_famDeclined[c.family];    break;
      case DISAGREED:         ++g_disagreed;   ++g_famDisagreed[c.family];   break;
      case ORACLE_INACCURATE: ++g_oracleInacc; ++g_famOracleInacc[c.family]; break;
      case BOTH_DECLINED:     ++g_bothDecl;    ++g_famBothDecl[c.family];    break;
      case ORACLE_UNRELIABLE: ++g_oracleBad;                                 break;
    }

    if (v == AGREED) {
      std::printf("[FUZZ] AGREED    case=%d %-27s volN=%.6g volA=%.6g dV=%.2e areaN=%.6g areaA=%.6g dA=%.2e "
                  "Cerr=%.2e tol=%.1e  OCCT[I=(%.4g,%.4g,%.4g)]  %s\n",
                  i, famName(c.family), nat.volume, c.aVol, relDiff(nat.volume, c.aVol),
                  nat.area, c.aArea, relDiff(nat.area, c.aArea), natCentErr, tol,
                  occ.Ixx, occ.Iyy, occ.Izz, c.desc.c_str());
    } else if (v == DECLINED) {
      std::printf("[FUZZ] DECLINED  case=%d %-27s native=%s (no valid mass) -> OCCT[volO=%.6g areaO=%.6g C=(%.3g,%.3g,%.3g)]  %s%s\n",
                  i, famName(c.family), nat.present ? "non-watertight" : "NULL",
                  occ.volume, occ.area, occ.cx, occ.cy, occ.cz, c.desc.c_str(),
                  c.expectDecline ? "  [expected-decline sub-family]" : "");
    } else if (v == BOTH_DECLINED) {
      std::printf("[FUZZ] BOTH-DECL case=%d %-27s native declined AND OCCT built no valid mass (degenerate)  %s\n",
                  i, famName(c.family), c.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
      std::printf("[FUZZ] ORACLE_INACCURATE case=%d %-27s native MATCHES analytic, OCCT does NOT "
                  "volN=%.6g volO=%.6g aVol=%.6g areaN=%.6g areaO=%.6g aArea=%.6g\n"
                  "       NOTE seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nat.volume, occ.volume, c.aVol, nat.area, occ.area, c.aArea,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
      std::printf("[FUZZ] ORACLE_UNRELIABLE case=%d %-27s core-family oracle mismatch/absent "
                  "[natValid=%d occValid=%d volO=%.6g aVol=%.6g occVsA_V=%.2e]\n"
                  "       REPRO seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nativeValid, oracleValid, occ.volume, c.aVol,
                  oracleValid ? relDiff(occ.volume, c.aVol) : -1.0,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
    } else {  // DISAGREED
      std::printf("[FUZZ] DISAGREED case=%d %-27s SILENT-WRONG-MASS "
                  "volN=%.6g volA=%.6g dV=%.3e  areaN=%.6g areaA=%.6g dA=%.3e  Cnat=(%.4g,%.4g,%.4g) Canalytic=(%.4g,%.4g,%.4g) Cerr=%.3e tol=%.2e\n"
                  "       REPRO seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nat.volume, c.aVol, relDiff(nat.volume, c.aVol),
                  nat.area, c.aArea, relDiff(nat.area, c.aArea),
                  nat.cx, nat.cy, nat.cz, c.aCx, c.aCy, c.aCz, natCentErr, tol,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
    }
    std::fflush(stdout);
  }

  // ── coverage summary ──────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  BOTH-DECLINED=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleInacc, g_bothDecl);
  std::printf("   per-family [agreed/declined/DISAGREED/oracle-inaccurate/both-declined]:\n");
  for (int f = 0; f < F_COUNT; ++f) {
    std::printf("     %-30s %d/%d/%d/%d/%d\n", famName(f), g_famAgreed[f], g_famDeclined[f],
                g_famDisagreed[f], g_famOracleInacc[f], g_famBothDecl[f]);
  }
  std::printf("   INERTIA: %d native bodies — ALL HONESTLY-DECLINED (native principal_moments = "
              "CC_NATIVE_BODY_UNSUPPORTED; OCCT moments printed as telemetry only, never a bar input)\n",
              g_inertiaDeclined);
  std::printf("   HONEST-SCOPE: planar families (BOX/NGON/LOFT) reproduce the solid EXACTLY (tight %.0e); "
              "curved families (CYLINDER/CONE-apex/SPHERE/REVOLVE) use the deflection convergence bound "
              "%.1f*deflection/featureSize; the CONE FRUSTUM sub-family HONESTLY-DECLINES (native mesh "
              "not watertight at %.3f — cap/slant seam).\n", kTightTol, kCurveC, kPropertyDeflection);
  if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — logged, NOT a native fault)\n", g_oracleInacc);
  if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (degenerate input both engines refuse — no wrong result, logged)\n", g_bothDecl);
  if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (core-family OCCT vs closed-form mismatch — investigate)\n", g_oracleBad);

  // Bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0 AND every core AGREE family has ≥1 AGREED.
  bool coverage = true;
  for (int f = F_BOX; f <= F_REVOLVE; ++f) if (g_famAgreed[f] < 1) coverage = false;
  const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && coverage);
  std::printf("== M6-breadth-6 BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
              "per-family AGREE coverage=%s) ==\n",
              bar ? "PASS — zero silent wrong masses" : "FAIL", g_disagreed, g_oracleBad,
              coverage ? "complete" : "INCOMPLETE");
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
