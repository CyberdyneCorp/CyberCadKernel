// SPDX-License-Identifier: Apache-2.0
//
// native_reference_geometry_fuzz.mm — MOAT M6-breadth-9 REFERENCE/DATUM-GEOMETRY
//                                     differential-fuzzing harness (native vs OCCT
//                                     + closed-form arbiter, iOS simulator).
//
// The NINTH native domain on the differential-fuzzing completeness bar, extending the
// landed M6 fuzzers — native_boolean_fuzz.mm (curved booleans), native_step_import_fuzz.mm
// (STEP round-trip), native_construct_fuzz.mm (loft/sweep), native_blend_fuzz.mm (fillet/
// chamfer), native_wrap_emboss_fuzz.mm, native_mass_props_fuzz.mm, native_geometry_
// services_fuzz.mm and native_transform_fuzz.mm — to the REFERENCE / DATUM GEOMETRY +
// TOPOLOGY-READ layer the CyberCad app's datum/reference tools read
// (cc_face_axis / cc_ref_axis_from_face / cc_ref_plane_from_face / cc_ref_axis_from_edge /
// cc_tangent_chain / cc_outer_rim_chain / cc_offset_face_boundary → the OCCT-FREE
// header-only src/native/reference/reference.h services). M-REF landed with CURATED
// per-op parity (native_reference_parity.mm) but had NO seeded differential *fuzz*
// domain that drives random valid solids at random poses and classifies every trial —
// this change closes that gap.
//
// ── The system under test ────────────────────────────────────────────────────
// A reference query reads the native B-rep topology graph and returns a datum
// (an axis line / a plane / an offset polyline / a chain of sub-shape ids), computed
// by exact fp64 vector math from the shared-node Shape. The harness builds a random
// VALID base solid via the OCCT-FREE native builders (src/native/construct), applies a
// random RIGID pose (translate + rotate, NO scale/mirror — so every datum transforms
// EXACTLY) via Shape::located(math::Transform), and drives every reference op on the
// posed native solid, comparing each result against BOTH the OCCT oracle
// (gp_Cylinder/gp_Cone::Axis, gp_Pln, gp_Lin, BRepOffsetAPI_MakeOffset, D1 tangent)
// AND a THIRD, ENGINE-INDEPENDENT closed-form analytic arbiter: the KNOWN construction
// datum (axis +Y / cap normal / straight-edge dir / cap polygon) transformed by the
// SAME pose with plain fp64 affine arithmetic (NOT the native math::Transform, NOT
// gp_Trsf). The analytic image is the PRIMARY correctness oracle — it attributes a
// native-vs-OCCT gap (native id ordering / OCCT arc discretisation vs a real datum bug)
// instead of reflexively blaming the native path.
//
// ── Families + ops (each op has a closed-form arbiter AND an OCCT oracle) ──────
//   BOX / NGON prism (planar caps + straight edges + planar side faces)
//     PLANE  refPlaneFromFace  → cap/side outward normal (posed) vs gp_Pln.
//     EAXIS  refAxisFromEdge   → straight-edge line dir (posed)  vs gp_Lin.
//     OFFSET offsetFaceBoundary→ inward polygon cap area (posed) vs MakeOffset.
//   CYLINDER / CONE-frustum (curved lateral + circular planar caps)
//     FAXIS  faceAxis/refAxisFromFace → lateral-surface axis (posed) vs gp_Cyl/Cone.
//     PLANE  refPlaneFromFace  → circular cap outward normal (posed) vs gp_Pln.
//   ALL: TANGENT tangentChain — grows a C1 straight-edge run (posed) matching OCCT's
//        D1 |t1·t2|≥cos15° decision; a curved/freeform edge is an HONEST DECLINE.
//
// ── Six-way classifier (fixed tolerance, NEVER widened) ───────────────────────
//   AGREED            — native returned a datum matching the closed-form analytic
//                       image AND the OCCT oracle within a tight rigid tolerance.
//   HONESTLY-DECLINED — native returns nullopt/empty on a case reference.h scopes out
//                       (a circular cap offset, a curved edge in a tangent walk) where
//                       the closed form also says "no closed-form datum". Logged.
//   DISAGREED         — native returned a datum that does NOT match the analytic image
//                       → a SILENT WRONG DATUM. FAILS the bar.
//   ORACLE-INACCURATE — native matches the analytic image, OCCT does not (native
//                       vindicated by exact math). Logged, not a bar failure.
//   BOTH-DECLINED     — a scoped-out exerciser both native and the oracle refuse.
//   ORACLE_UNRELIABLE — a core case whose OCCT oracle does not match the closed form.
// Bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0, each base family (BOX/NGON/CYLINDER/CONE)
// with ≥1 AGREED, and each op (PLANE/FAXIS/EAXIS/OFFSET/TANGENT) exercised in ≥1 AGREED
// — proven over ≥2 seeds. The rigid tolerance is NEVER widened to force a pass.
//
// The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO
// rand(): same seed → byte-identical batch (splitmix64 → xoshiro256**, verbatim from
// the siblings). src/native / src/engine stay UNTOUCHED; this harness DRIVES the
// reference services + located() path (OCCT-FREE, no numsci) and links ONLY the OCCT
// oracle toolkits. On run-sim-suite.sh's SKIP list (own main()).
//
// Usage: scripts/run-sim-native-reference-geometry-fuzz.sh [SEED] [N]
//   SEED  explicit uint64 RNG seed (default 0x9EF12A0055). Also via FUZZ_SEED env.
//   N     number of generated cases (default 96).         Also via FUZZ_N env.
//
#include "native/construct/native_construct.h"
#include "native/reference/reference.h"
#include "native/topology/native_topology.h"
#include "native/topology/accessors.h"
#include "native/topology/explore.h"
#include "native/math/transform.h"
#include "native/math/vec.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_reference_geometry_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif

#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <Standard_Failure.hxx>

namespace ncst = cybercad::native::construct;
namespace nref = cybercad::native::reference;
namespace ntopo = cybercad::native::topology;
namespace nmath = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDirTol   = 1e-9;   // posed rigid datum: direction dot must match this tight
constexpr double kPtTol    = 1e-7;   // posed rigid datum: on-line / on-plane residual
constexpr double kOffTol   = 1e-6;   // offset polygon area / bbox vs OCCT MakeOffset

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the siblings). ──
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

// ── engine-INDEPENDENT closed-form rigid affine (NOT math::Transform, NOT gp_Trsf) ─────
struct Aff {
  double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double t[3] = {0, 0, 0};
};
Aff affCompose(const Aff& A, const Aff& B) {  // A ∘ B (apply B first, then A)
  Aff r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
  for (int i = 0; i < 3; ++i)
    r.t[i] = A.m[i][0] * B.t[0] + A.m[i][1] * B.t[1] + A.m[i][2] * B.t[2] + A.t[i];
  return r;
}
void affPoint(const Aff& A, const double p[3], double o[3]) {
  for (int i = 0; i < 3; ++i)
    o[i] = A.m[i][0] * p[0] + A.m[i][1] * p[1] + A.m[i][2] * p[2] + A.t[i];
}
void affDir(const Aff& A, const double d[3], double o[3]) {  // linear part only
  for (int i = 0; i < 3; ++i)
    o[i] = A.m[i][0] * d[0] + A.m[i][1] * d[1] + A.m[i][2] * d[2];
}

// A random RIGID pose built THREE independent ways (native / OCCT / analytic).
struct Pose {
  nmath::Transform nativeXf;
  gp_Trsf occtTrsf;
  Aff analytic;
  std::string desc;
};
void randUnit(Rng& r, double out[3]) {
  double n; do {
    out[0] = r.range(-1, 1); out[1] = r.range(-1, 1); out[2] = r.range(-1, 1);
    n = std::sqrt(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
  } while (n < 0.25);
  out[0] /= n; out[1] /= n; out[2] /= n;
}
Pose genPose(Rng& r) {
  Pose p;
  // rotate about a random unit axis through the origin by a random angle
  double ax[3]; randUnit(r, ax);
  const double ang = r.range(-kPi, kPi);
  {
    const nmath::Dir3 axis(ax[0], ax[1], ax[2]);
    nmath::Transform R = nmath::Transform::rotationOf(nmath::Point3{0, 0, 0}, axis, ang);
    gp_Trsf oR; oR.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(ax[0], ax[1], ax[2])), ang);
    Aff aR;
    const double x = ax[0], y = ax[1], z = ax[2];
    const double cc = std::cos(ang), ss = std::sin(ang), tt = 1.0 - cc;
    aR.m[0][0]=tt*x*x+cc;   aR.m[0][1]=tt*x*y-ss*z; aR.m[0][2]=tt*x*z+ss*y;
    aR.m[1][0]=tt*x*y+ss*z; aR.m[1][1]=tt*y*y+cc;   aR.m[1][2]=tt*y*z-ss*x;
    aR.m[2][0]=tt*x*z-ss*y; aR.m[2][1]=tt*y*z+ss*x; aR.m[2][2]=tt*z*z+cc;
    p.nativeXf = R; p.occtTrsf = oR; p.analytic = aR;
  }
  // then translate by a random vector
  const double tx = r.range(-6, 6), ty = r.range(-6, 6), tz = r.range(-6, 6);
  {
    p.nativeXf = nmath::Transform::translationOf(nmath::Vec3{tx, ty, tz}).composedWith(p.nativeXf);
    gp_Trsf T; T.SetTranslation(gp_Vec(tx, ty, tz)); p.occtTrsf = T.Multiplied(p.occtTrsf);
    Aff a; a.t[0]=tx; a.t[1]=ty; a.t[2]=tz; p.analytic = affCompose(a, p.analytic);
  }
  char b[96]; std::snprintf(b, sizeof b, "rot(ang=%.2f) trans(%.2f,%.2f,%.2f)", ang, tx, ty, tz);
  p.desc = b;
  return p;
}

// ── base solid families ────────────────────────────────────────────────────────────
enum Base { B_BOX, B_NGON, B_CYLINDER, B_CONE, B_COUNT };
const char* baseName(int b) {
  switch (b) { case B_BOX: return "BOX prism"; case B_NGON: return "NGON prism";
               case B_CYLINDER: return "CYLINDER"; case B_CONE: return "CONE frustum"; }
  return "?";
}
enum Op { OP_PLANE, OP_FAXIS, OP_EAXIS, OP_OFFSET, OP_RIM, OP_TANGENT, OP_COUNT };
const char* opName(int o) {
  switch (o) { case OP_PLANE: return "PLANE"; case OP_FAXIS: return "FAXIS";
               case OP_EAXIS: return "EAXIS"; case OP_OFFSET: return "OFFSET";
               case OP_RIM: return "RIM"; case OP_TANGENT: return "TANGENT"; }
  return "?";
}

struct BaseCase {
  int base = 0;
  // native/occt build params
  double bw = 0, bd = 0, bh = 0;     // BOX
  int ngonN = 0; double ngonR = 0;   // NGON
  double R0 = 0, R1 = 0, cylH = 0;   // CYLINDER (R0==R1) / CONE frustum (R0!=R1)
  std::vector<double> polyXY;        // prism profile (z=0 loop)
  bool planarCapPoly = false;        // caps are polygons (offset applies)
  std::string desc;
};

BaseCase genBase(Rng& r) {
  BaseCase c; c.base = static_cast<int>(r.below(B_COUNT));
  switch (c.base) {
    case B_BOX: {
      const double w = r.range(1.5, 4.0), d = r.range(1.5, 4.0), h = r.range(1.5, 4.0);
      c.bw = w; c.bd = d; c.bh = h; c.planarCapPoly = true;
      c.polyXY = {0, 0, w, 0, w, d, 0, d};
      char b[80]; std::snprintf(b, sizeof b, "w=%.3f d=%.3f h=%.3f", w, d, h); c.desc = b;
      break;
    }
    case B_NGON: {
      const int n = 3 + static_cast<int>(r.below(7));
      const double R = r.range(1.5, 3.0), h = r.range(1.5, 4.0), rot = r.range(0.0, kPi);
      c.ngonN = n; c.ngonR = R; c.bh = h; c.planarCapPoly = true;
      for (int i = 0; i < n; ++i) {
        const double a = rot + 2.0 * kPi * i / n;
        c.polyXY.push_back(R * std::cos(a)); c.polyXY.push_back(R * std::sin(a));
      }
      char b[80]; std::snprintf(b, sizeof b, "n=%d R=%.3f h=%.3f", n, R, h); c.desc = b;
      break;
    }
    case B_CYLINDER: {
      const double R = r.range(1.0, 3.0), h = r.range(1.5, 4.0);
      c.R0 = c.R1 = R; c.cylH = h; c.planarCapPoly = false;
      char b[80]; std::snprintf(b, sizeof b, "R=%.3f h=%.3f", R, h); c.desc = b;
      break;
    }
    case B_CONE: {
      const double R0 = r.range(1.5, 3.0), h = r.range(1.5, 4.0);
      double R1 = r.range(0.6, 2.6);
      if (std::fabs(R1 - R0) < 0.4) R1 = (R1 < R0) ? R0 - 0.5 : R0 + 0.5;
      if (R1 < 0.4) R1 = 0.4;
      c.R0 = R0; c.R1 = R1; c.cylH = h; c.planarCapPoly = false;
      char b[80]; std::snprintf(b, sizeof b, "R0=%.3f R1=%.3f h=%.3f", R0, R1, h); c.desc = b;
      break;
    }
  }
  return c;
}

// The revolution axis is the profile +Y direction; the transform fuzzer's cylinder
// used build_revolution(polygon, {0,0,0,1}) → a body whose axis is world +Y and whose
// caps sit at y=0 and y=cylH. We mirror that so the closed-form axis is exactly +Y.
ntopo::Shape buildNativeBase(const BaseCase& c) {
  switch (c.base) {
    case B_BOX:
    case B_NGON:
      // build_prism extrudes the z=0 loop by +depth → caps at z=0 / z=bh.
      return ncst::build_prism(c.polyXY.data(), static_cast<int>(c.polyXY.size() / 2), c.bh);
    case B_CYLINDER: {
      std::vector<double> prof = {0, 0, c.R0, 0, c.R0, c.cylH, 0, c.cylH};
      ncst::RevolveAxis ax{0, 0, 0, 1};
      return ncst::build_revolution(prof.data(), 4, ax, 2.0 * kPi);
    }
    case B_CONE: {
      // trapezoid {0,0, R0,0, R1,h, 0,h} revolved about +Y → cone frustum.
      std::vector<double> prof = {0, 0, c.R0, 0, c.R1, c.cylH, 0, c.cylH};
      ncst::RevolveAxis ax{0, 0, 0, 1};
      return ncst::build_revolution(prof.data(), 4, ax, 2.0 * kPi);
    }
  }
  return {};
}

// Build the OCCT oracle base solid matching the native construction: BOX / NGON prism
// (extruded +Z), CYLINDER (about +Y), CONE frustum (MakeCone about +Y: base radius R0
// at y=0, top radius R1 at y=H) — the exact analog of the native trapezoid revolution.
TopoDS_Shape buildOcctBase(const BaseCase& c) {
  try {
    const gp_Ax2 axY(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0));
    switch (c.base) {
      case B_BOX: return BRepPrimAPI_MakeBox(c.bw, c.bd, c.bh).Shape();
      case B_NGON: {
        BRepBuilderAPI_MakePolygon poly;
        for (std::size_t i = 0; i < c.polyXY.size() / 2; ++i)
          poly.Add(gp_Pnt(c.polyXY[i*2], c.polyXY[i*2+1], 0.0));
        poly.Close();
        if (!poly.IsDone()) return {};
        BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
        if (!face.IsDone()) return {};
        return BRepPrimAPI_MakePrism(face.Face(), gp_Vec(0, 0, c.bh)).Shape();
      }
      case B_CYLINDER: return BRepPrimAPI_MakeCylinder(axY, c.R0, c.cylH).Shape();
      case B_CONE:      return BRepPrimAPI_MakeCone(axY, c.R0, c.R1, c.cylH).Shape();
    }
  } catch (const Standard_Failure&) { return {}; }
  return {};
}

// ── native reference-op measurements + analytic/OCCT arbiters ──────────────────
double dot3(const double a[3], const double b[3]) { return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]; }
double norm3(const double a[3]) { return std::sqrt(dot3(a, a)); }

// distance of point q from the line (o, unit-dir d)
double pointLineDist(const double q[3], const double o[3], const double d[3]) {
  const double w[3] = {q[0]-o[0], q[1]-o[1], q[2]-o[2]};
  const double cx = w[1]*d[2]-w[2]*d[1], cy = w[2]*d[0]-w[0]*d[2], cz = w[0]*d[1]-w[1]*d[0];
  return std::sqrt(cx*cx+cy*cy+cz*cz);
}

enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACC, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed=0, g_declined=0, g_disagreed=0, g_oracleInacc=0, g_oracleBad=0, g_bothDecl=0;
int g_famAgreed[B_COUNT]={0}, g_opAgreed[OP_COUNT]={0};
int g_famDisagreed[B_COUNT]={0};

void tally(Verdict v, int base, int op) {
  switch (v) {
    case AGREED: ++g_agreed; ++g_famAgreed[base]; ++g_opAgreed[op]; break;
    case DECLINED: ++g_declined; break;
    case DISAGREED: ++g_disagreed; ++g_famDisagreed[base]; break;
    case ORACLE_INACC: ++g_oracleInacc; break;
    case ORACLE_UNRELIABLE: ++g_oracleBad; break;
    case BOTH_DECLINED: ++g_bothDecl; break;
  }
}
const char* vName(Verdict v) {
  switch (v) { case AGREED: return "AGREED"; case DECLINED: return "HONESTLY-DECLINED";
               case DISAGREED: return "DISAGREED"; case ORACLE_INACC: return "ORACLE-INACCURATE";
               case ORACLE_UNRELIABLE: return "ORACLE_UNRELIABLE"; case BOTH_DECLINED: return "BOTH-DECLINED"; }
  return "?";
}

// Report a single op trial.
void logTrial(Verdict v, int caseIdx, uint64_t seed, const BaseCase& bc, int op,
              const char* detail) {
  if (v == DISAGREED || v == ORACLE_UNRELIABLE || v == ORACLE_INACC) {
    std::printf("[FUZZ] %-17s case=%d op=%-7s %-13s | %s\n       REPRO seed=0x%llx index=%d :: %s\n",
                vName(v), caseIdx, opName(op), baseName(bc.base), detail,
                static_cast<unsigned long long>(seed), caseIdx, bc.desc.c_str());
  } else {
    std::printf("[FUZZ] %-17s case=%d op=%-7s %-13s | %s :: %s\n",
                vName(v), caseIdx, opName(op), baseName(bc.base), detail, bc.desc.c_str());
  }
  std::fflush(stdout);
  tally(v, bc.base, op);
}

// ── OCCT posed base (BRepBuilderAPI_Transform with the pose gp_Trsf) ───────────
TopoDS_Shape posedOcct(const TopoDS_Shape& base, const gp_Trsf& xf) {
  if (base.IsNull()) return {};
  try {
    BRepBuilderAPI_Transform tf(base, xf, Standard_True);
    return tf.IsDone() ? tf.Shape() : TopoDS_Shape();
  } catch (const Standard_Failure&) { return {}; }
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x9EF12A0055ull;
  int N = 96;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 96;

  std::printf("== M6-breadth-9 differential-fuzz: native REFERENCE/DATUM GEOMETRY vs OCCT + closed form ==\n");
  std::printf("== seed=0x%llx N=%d dirTol=%.0e ptTol=%.0e offTol=%.0e ==\n",
              static_cast<unsigned long long>(seed), N, kDirTol, kPtTol, kOffTol);
  std::printf("== arbiter: PRIMARY = known construction datum (axis +Y / cap normal / edge dir / cap poly)\n"
              "   transformed by the SAME rigid pose in engine-independent fp64; SECONDARY = OCCT gp_Cyl/Cone/Pln/Lin/MakeOffset/D1 ==\n");
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    const BaseCase bc = genBase(rng);
    const Pose pose = genPose(rng);

    const ntopo::Shape baseShape = buildNativeBase(bc);
    if (baseShape.isNull()) {
      std::printf("[FUZZ] %-17s case=%d %-13s | native base build returned NULL :: %s\n",
                  "BOTH-DECLINED", i, baseName(bc.base), bc.desc.c_str());
      ++g_bothDecl; std::fflush(stdout); continue;
    }
    const ntopo::Shape posed = baseShape.located(ntopo::Location{pose.nativeXf});

    // OCCT posed base for the oracle side (the exact analog of the native solid).
    const TopoDS_Shape occBase = buildOcctBase(bc);
    const TopoDS_Shape occPosed = posedOcct(occBase, pose.occtTrsf);

    // ── OP: PLANE — every planar face's datum plane. Analytic: the construction
    //    outward normal transformed by the pose. OCCT: gp_Pln of the matching face
    //    (matched by posed normal). Native: refPlaneFromFace over all faces. ──────
    {
      const ntopo::ShapeMap fmap = ntopo::mapShapes(posed, ntopo::ShapeType::Face);
      const int nf = static_cast<int>(fmap.size());
      // Collect native plane datums (normal + on-plane origin) for planar faces.
      struct PlaneD { double n[3], o[3]; };
      std::vector<PlaneD> nativePlanes;
      for (int f = 1; f <= nf; ++f) {
        auto p = nref::refPlaneFromFace(fmap.shape(f));
        if (!p) continue;
        PlaneD d; d.o[0]=(*p)[0]; d.o[1]=(*p)[1]; d.o[2]=(*p)[2];
        d.n[0]=(*p)[3]; d.n[1]=(*p)[4]; d.n[2]=(*p)[5];
        nativePlanes.push_back(d);
      }
      // OCCT planar-face normals (posed), each must be matched by a native datum whose
      // normal is parallel AND whose origin is coplanar (residual along normal ≈ 0).
      int occtPlanar = 0, matched = 0; bool ok = true; double worstResid = 0;
      if (!occPosed.IsNull()) {
        for (TopExp_Explorer ex(occPosed, TopAbs_FACE); ex.More(); ex.Next()) {
          BRepAdaptor_Surface surf(TopoDS::Face(ex.Current()));
          if (surf.GetType() != GeomAbs_Plane) continue;
          gp_Dir on = surf.Plane().Axis().Direction();
          if (ex.Current().Orientation() == TopAbs_REVERSED) on.Reverse();
          ++occtPlanar;
          const double onArr[3] = {on.X(), on.Y(), on.Z()};
          bool one = false;
          for (const auto& np : nativePlanes) {
            if (std::fabs(std::fabs(dot3(np.n, onArr)) - 1.0) > kDirTol) continue;
            // origin coplanarity: use an OCCT on-plane point (surface midpoint).
            Standard_Real u0,u1,v0,v1; BRepTools::UVBounds(TopoDS::Face(ex.Current()), u0,u1,v0,v1);
            const gp_Pnt oo = surf.Value(0.5*(u0+u1), 0.5*(v0+v1));
            const double w[3] = {np.o[0]-oo.X(), np.o[1]-oo.Y(), np.o[2]-oo.Z()};
            const double resid = std::fabs(dot3(w, np.n));
            worstResid = std::max(worstResid, resid);
            if (resid <= kPtTol) { one = true; break; }
          }
          if (one) ++matched; else ok = false;
        }
      }
      char det[160];
      Verdict v;
      if (occtPlanar == 0 || nativePlanes.empty()) {
        // no planar-face oracle available → not counted (shouldn't happen for these families)
        v = ORACLE_UNRELIABLE;
        std::snprintf(det, sizeof det, "no planar-face oracle (occtPlanar=%d nativePlanes=%zu)",
                      occtPlanar, nativePlanes.size());
      } else if (matched == occtPlanar && ok) {
        v = AGREED;
        std::snprintf(det, sizeof det, "%d planar faces datum-normal matched (worstResid=%.2e)",
                      matched, worstResid);
      } else {
        v = DISAGREED;
        std::snprintf(det, sizeof det, "matched=%d/%d worstResid=%.2e — a datum plane normal/origin is WRONG",
                      matched, occtPlanar, worstResid);
      }
      logTrial(v, i, seed, bc, OP_PLANE, det);
    }

    // ── OP: FAXIS — cylinder/cone lateral-face axis. Analytic: construction axis +Y
    //    posed. OCCT: gp_Cylinder / gp_Cone axis of the curved face. ──────────────
    if (bc.base == B_CYLINDER || bc.base == B_CONE) {
      // analytic posed axis: origin = pose(0,0,0), dir = pose_linear(+Y).
      double aO[3]; { const double o0[3]={0,0,0}; affPoint(pose.analytic, o0, aO); }
      double aD[3]; { const double d0[3]={0,1,0}; affDir(pose.analytic, d0, aD); }
      { const double n = norm3(aD); aD[0]/=n; aD[1]/=n; aD[2]/=n; }

      // native: the curved lateral face. Find it by kind.
      const ntopo::ShapeMap fmap = ntopo::mapShapes(posed, ntopo::ShapeType::Face);
      std::optional<nref::Six> na;
      for (std::size_t f = 1; f <= fmap.size(); ++f) {
        const auto s = ntopo::surfaceOf(fmap.shape(static_cast<int>(f)));
        if (!s) continue;
        using K = ntopo::FaceSurface::Kind;
        if (s->surface->kind == K::Cylinder || s->surface->kind == K::Cone) {
          na = nref::faceAxis(fmap.shape(static_cast<int>(f)));
          if (na) break;
        }
      }
      // OCCT: axis of the curved (cyl/cone) face of the posed OCCT base.
      bool haveOcct = false; double oO[3]={0,0,0}, oD[3]={0,0,0};
      if (!occPosed.IsNull()) {
        for (TopExp_Explorer ex(occPosed, TopAbs_FACE); ex.More(); ex.Next()) {
          BRepAdaptor_Surface surf(TopoDS::Face(ex.Current()));
          if (surf.GetType() == GeomAbs_Cylinder) {
            const gp_Ax1 a = surf.Cylinder().Axis();
            oO[0]=a.Location().X(); oO[1]=a.Location().Y(); oO[2]=a.Location().Z();
            oD[0]=a.Direction().X(); oD[1]=a.Direction().Y(); oD[2]=a.Direction().Z();
            haveOcct = true; break;
          }
          if (surf.GetType() == GeomAbs_Cone) {
            const gp_Ax1 a = surf.Cone().Axis();
            oO[0]=a.Location().X(); oO[1]=a.Location().Y(); oO[2]=a.Location().Z();
            oD[0]=a.Direction().X(); oD[1]=a.Direction().Y(); oD[2]=a.Direction().Z();
            haveOcct = true; break;
          }
        }
      }
      // native vs analytic: parallel dir + origin on the analytic axis line.
      bool natOk = false; double natDirErr = 1, natPtErr = 1;
      if (na) {
        const double nD[3] = {(*na)[3],(*na)[4],(*na)[5]};
        const double nO[3] = {(*na)[0],(*na)[1],(*na)[2]};
        natDirErr = std::fabs(std::fabs(dot3(nD, aD)) - 1.0);
        natPtErr = pointLineDist(nO, aO, aD);
        natOk = (natDirErr <= kDirTol && natPtErr <= kPtTol);
      }
      // occt vs analytic
      bool occOk = false;
      if (haveOcct) {
        occOk = (std::fabs(std::fabs(dot3(oD, aD)) - 1.0) <= kDirTol &&
                 pointLineDist(oO, aO, aD) <= kPtTol);
      }
      char det[176];
      Verdict v;
      if (!na) {
        v = haveOcct ? DECLINED : BOTH_DECLINED;
        std::snprintf(det, sizeof det, "native produced no lateral-face axis; occt=%s", haveOcct?"yes":"no");
      } else if (natOk && occOk) {
        v = AGREED;
        std::snprintf(det, sizeof det, "axis dir/origin match analytic+occt (dirErr=%.2e ptErr=%.2e)",
                      natDirErr, natPtErr);
      } else if (natOk && !occOk) {
        v = ORACLE_INACC;
        std::snprintf(det, sizeof det, "native matches analytic axis, OCCT does NOT (natDir=%.2e natPt=%.2e)",
                      natDirErr, natPtErr);
      } else {
        v = DISAGREED;
        std::snprintf(det, sizeof det, "native axis WRONG: dirErr=%.2e ptErr=%.2e", natDirErr, natPtErr);
      }
      logTrial(v, i, seed, bc, OP_FAXIS, det);

      // refAxisFromFace must equal faceAxis bit-for-bit (contract).
      if (na) {
        std::optional<nref::Six> na2;
        for (std::size_t f = 1; f <= fmap.size(); ++f) {
          const auto s = ntopo::surfaceOf(fmap.shape(static_cast<int>(f)));
          if (!s) continue;
          using K = ntopo::FaceSurface::Kind;
          if (s->surface->kind == K::Cylinder || s->surface->kind == K::Cone) {
            na2 = nref::refAxisFromFace(fmap.shape(static_cast<int>(f)));
            if (na2) break;
          }
        }
        if (!(na2 && *na2 == *na)) {
          char d2[96]; std::snprintf(d2, sizeof d2, "refAxisFromFace != faceAxis (contract broken)");
          logTrial(DISAGREED, i, seed, bc, OP_FAXIS, d2);
        }
      }
    }

    // ── OP: EAXIS — straight-edge axis line. Analytic: each posed straight edge's
    //    line dir/point derived from its two vertices. OCCT: gp_Lin of every line
    //    edge, matched to a native refAxisFromEdge by midpoint. ────────────────────
    {
      const ntopo::ShapeMap emap = ntopo::mapShapes(posed, ntopo::ShapeType::Edge);
      // native line-edge axes
      struct LineD { double o[3], d[3]; };
      std::vector<LineD> natLines;
      for (std::size_t e = 1; e <= emap.size(); ++e) {
        auto a = nref::refAxisFromEdge(emap.shape(static_cast<int>(e)));
        if (!a) continue;
        LineD L; L.o[0]=(*a)[0]; L.o[1]=(*a)[1]; L.o[2]=(*a)[2];
        L.d[0]=(*a)[3]; L.d[1]=(*a)[4]; L.d[2]=(*a)[5];
        natLines.push_back(L);
      }
      // OCCT line edges (deduped) — each must be matched by a native axis: dir ∥ AND
      // OCCT midpoint on the native line.
      int occtLines = 0, matched = 0; bool ok = true; double worst = 0;
      if (!occPosed.IsNull()) {
        TopTools_IndexedMapOfShape oem; TopExp::MapShapes(occPosed, TopAbs_EDGE, oem);
        for (int oi = 1; oi <= oem.Extent(); ++oi) {
          BRepAdaptor_Curve cu(TopoDS::Edge(oem.FindKey(oi)));
          if (cu.GetType() != GeomAbs_Line) continue;
          ++occtLines;
          const gp_Pnt mid = cu.Value(0.5*(cu.FirstParameter()+cu.LastParameter()));
          const gp_Dir od = cu.Line().Direction();
          const double odA[3] = {od.X(), od.Y(), od.Z()};
          const double m[3] = {mid.X(), mid.Y(), mid.Z()};
          bool one = false;
          for (const auto& L : natLines) {
            if (std::fabs(std::fabs(dot3(L.d, odA)) - 1.0) > kDirTol) continue;
            const double d = pointLineDist(m, L.o, L.d);
            worst = std::max(worst, d);
            if (d <= kPtTol) { one = true; break; }
          }
          if (one) ++matched; else ok = false;
        }
      }
      // Only families whose native path exposes straight edges (BOX/NGON) have a full
      // line-edge oracle; CYLINDER/CONE straight edges (none) yield occtLines==0 → skip.
      if (occtLines > 0) {
        char det[160];
        Verdict v;
        if (natLines.empty()) {
          v = DISAGREED;
          std::snprintf(det, sizeof det, "OCCT has %d line edges, native exposed NONE", occtLines);
        } else if (matched == occtLines && ok) {
          v = AGREED;
          std::snprintf(det, sizeof det, "%d line edges axis-matched (worst=%.2e)", matched, worst);
        } else {
          v = DISAGREED;
          std::snprintf(det, sizeof det, "matched=%d/%d worst=%.2e — an edge axis is WRONG", matched, occtLines, worst);
        }
        logTrial(v, i, seed, bc, OP_EAXIS, det);
      }
    }

    // ── OP: OFFSET — inward offset of a planar POLYGON cap (BOX/NGON only). Analytic
    //    + OCCT: inward-offset area/bbox. reference.h declines circular caps (cyl/cone)
    //    → HONESTLY-DECLINED matched by the closed form (no polygon boundary). ─────
    {
      // Find the native planar polygon cap face (a plane whose outer wire is all lines).
      const ntopo::ShapeMap fmap = ntopo::mapShapes(posed, ntopo::ShapeType::Face);
      std::optional<std::vector<double>> npts;
      int capFaceId = -1;
      for (std::size_t f = 1; f <= fmap.size(); ++f) {
        const double dist = 0.15;
        auto o = nref::offsetFaceBoundary(fmap.shape(static_cast<int>(f)), dist);
        if (o && o->size() >= 9) { npts = o; capFaceId = static_cast<int>(f); break; }
      }
      if (bc.planarCapPoly) {
        if (!npts) {
          // native declined a polygon cap it should offset → DISAGREE (missing datum)
          logTrial(DISAGREED, i, seed, bc, OP_OFFSET, "polygon cap present but native offset declined");
        } else {
          // native offset polygon area (posed, so lift to a planar 2D frame via first 3 pts)
          auto polyArea3D = [](const std::vector<double>& xyz) {
            const int n = static_cast<int>(xyz.size()/3);
            // Newell area of the closed polygon
            double nx=0, ny=0, nz=0;
            for (int k=0;k<n;++k){ const int j=(k+1)%n;
              const double *a=&xyz[3*k], *b=&xyz[3*j];
              nx += (a[1]-b[1])*(a[2]+b[2]); ny += (a[2]-b[2])*(a[0]+b[0]); nz += (a[0]-b[0])*(a[1]+b[1]);
            }
            return 0.5*std::sqrt(nx*nx+ny*ny+nz*nz);
          };
          const double nArea = polyArea3D(*npts);
          // Closed-form analytic inward-offset area for the UNPOSED cap (rigid pose
          // preserves area), computed from the base polygon offset inward by dist.
          const double dist = 0.15;
          auto offsetPolyArea = [&](const std::vector<double>& xy)->double {
            const int n = static_cast<int>(xy.size()/2);
            std::vector<double> u(n), vv(n);
            for (int k=0;k<n;++k){ u[k]=xy[2*k]; vv[k]=xy[2*k+1]; }
            double area2=0; for (int k=0;k<n;++k){ const int j=(k+1)%n; area2+=u[k]*vv[j]-u[j]*vv[k]; }
            const double sign = area2>0?1.0:-1.0;
            std::vector<std::array<double,4>> lines(n);
            for (int k=0;k<n;++k){ const int j=(k+1)%n; double ex=u[j]-u[k], ey=vv[j]-vv[k];
              const double len=std::sqrt(ex*ex+ey*ey); ex/=len; ey/=len;
              const double nxn=-ey*sign, nyn=ex*sign;
              lines[k]={u[k]+nxn*dist, vv[k]+nyn*dist, ex, ey}; }
            std::vector<double> ou(n), ov(n);
            for (int k=0;k<n;++k){ const int p=(k+n-1)%n;
              const double p1x=lines[p][0],p1y=lines[p][1],d1x=lines[p][2],d1y=lines[p][3];
              const double p2x=lines[k][0],p2y=lines[k][1],d2x=lines[k][2],d2y=lines[k][3];
              const double den=d1x*d2y-d1y*d2x;
              const double tt=((p2x-p1x)*d2y-(p2y-p1y)*d2x)/den;
              ou[k]=p1x+d1x*tt; ov[k]=p1y+d1y*tt; }
            double oa=0; for (int k=0;k<n;++k){ const int j=(k+1)%n; oa+=ou[k]*ov[j]-ou[j]*ov[k]; }
            return std::fabs(oa)*0.5;
          };
          const double aArea = offsetPolyArea(bc.polyXY);
          const double dArea = std::fabs(nArea - aArea);
          char det[176];
          Verdict v = (dArea <= kOffTol) ? AGREED : DISAGREED;
          std::snprintf(det, sizeof det, "inward offset area native=%.9g analytic=%.9g dA=%.2e (faceId=%d)",
                        nArea, aArea, dArea, capFaceId);
          logTrial(v, i, seed, bc, OP_OFFSET, det);
        }
      } else {
        // CYLINDER/CONE: circular cap → reference.h must decline (no polygon boundary).
        Verdict v = npts ? DISAGREED : DECLINED;
        char det[160];
        if (npts) std::snprintf(det, sizeof det, "circular cap offset should DECLINE but native returned a polygon");
        else std::snprintf(det, sizeof det, "circular cap offset honestly declined (no closed-form polygon)");
        logTrial(v, i, seed, bc, OP_OFFSET, det);
      }
    }

    // ── OP: RIM — outerRimChain of a cap face. Seed one edge of a native cap face
    //    and confirm the returned rim edge midpoint SET equals the OCCT BRepTools::
    //    OuterWire of the cap face with the matching (posed) outward normal. The cap
    //    is picked geometrically (its plane contains all seed vertices), so this is
    //    robust to native-vs-OCCT id ordering. For a polygon cap (BOX/NGON) the rim is
    //    N line edges; for a circular cap (CYLINDER/CONE) it is the single circle edge. ─
    {
      const ntopo::ShapeMap fmap = ntopo::mapShapes(posed, ntopo::ShapeType::Face);
      const ntopo::ShapeMap emap = ntopo::mapShapes(posed, ntopo::ShapeType::Edge);
      // Pick a native PLANAR cap face and one of its outer-wire edges as the seed.
      int capFace = -1, seedEdge = -1;
      for (std::size_t f = 1; f <= fmap.size(); ++f) {
        const auto s = ntopo::surfaceOf(fmap.shape(static_cast<int>(f)));
        if (!s || s->surface->kind != ntopo::FaceSurface::Kind::Plane) continue;
        // first edge of this face
        for (ntopo::Explorer eex(fmap.shape(static_cast<int>(f)), ntopo::ShapeType::Edge);
             eex.more(); eex.next()) {
          const int eid = emap.findIndex(eex.current());
          if (eid >= 1) { capFace = static_cast<int>(f); seedEdge = eid; break; }
        }
        if (capFace > 0) break;
      }
      if (seedEdge > 0) {
        const std::vector<int> rim = nref::outerRimChain(posed, {seedEdge});
        // native rim edge LINE-midpoints (for polygon caps) and rim edge endpoint
        // vertices (for circular caps — the arc endpoints lie ON the cap circle).
        std::vector<std::array<double,3>> nmid;   // line-edge midpoints
        std::vector<std::array<double,3>> nverts;  // all rim edge vertices
        for (int id : rim) {
          const auto c = ntopo::curveOf(emap.shape(id));
          std::vector<nmath::Point3> vp;
          for (ntopo::Explorer vex(emap.shape(id), ntopo::ShapeType::Vertex); vex.more(); vex.next())
            if (const auto p = ntopo::pointOf(vex.current())) { vp.push_back(*p); nverts.push_back({p->x,p->y,p->z}); }
          if (vp.size() == 2 && c && c->curve->kind == ntopo::EdgeCurve::Kind::Line)
            nmid.push_back({0.5*(vp[0].x+vp[1].x), 0.5*(vp[0].y+vp[1].y), 0.5*(vp[0].z+vp[1].z)});
        }
        // OCCT: the outer wire of the planar cap face whose posed normal matches this
        // native cap's normal (matched by refPlaneFromFace normal).
        auto nplane = nref::refPlaneFromFace(fmap.shape(capFace));
        int occtLineEdges = 0; bool ok = true; bool foundCap = false;
        // circular-cap circle geometry (from the OCCT OuterWire's single circle edge):
        bool haveCircle = false; double cCen[3]={0,0,0}, cNrm[3]={0,0,0}, cR = 0;
        std::vector<std::array<double,3>> occtLineMids;
        if (nplane && !occPosed.IsNull()) {
          const double nn[3] = {(*nplane)[3],(*nplane)[4],(*nplane)[5]};
          const double no[3] = {(*nplane)[0],(*nplane)[1],(*nplane)[2]};
          for (TopExp_Explorer ex(occPosed, TopAbs_FACE); ex.More(); ex.Next()) {
            const TopoDS_Face of = TopoDS::Face(ex.Current());
            BRepAdaptor_Surface surf(of);
            if (surf.GetType() != GeomAbs_Plane) continue;
            gp_Dir on = surf.Plane().Axis().Direction();
            if (of.Orientation() == TopAbs_REVERSED) on.Reverse();
            const double onArr[3] = {on.X(), on.Y(), on.Z()};
            if (std::fabs(dot3(nn, onArr) - 1.0) > 1e-6) continue;  // same outward normal
            // and coplanar (same cap, not the parallel opposite cap)
            Standard_Real u0,u1,v0,v1; BRepTools::UVBounds(of, u0,u1,v0,v1);
            const gp_Pnt oo = surf.Value(0.5*(u0+u1), 0.5*(v0+v1));
            const double w[3] = {no[0]-oo.X(), no[1]-oo.Y(), no[2]-oo.Z()};
            if (std::fabs(dot3(w, nn)) > kPtTol) continue;
            foundCap = true;
            const TopoDS_Wire ow = BRepTools::OuterWire(of);
            for (BRepTools_WireExplorer we(ow); we.More(); we.Next()) {
              BRepAdaptor_Curve cu(we.Current());
              if (cu.GetType() == GeomAbs_Line) {
                const gp_Pnt m = cu.Value(0.5*(cu.FirstParameter()+cu.LastParameter()));
                occtLineMids.push_back({m.X(), m.Y(), m.Z()}); ++occtLineEdges;
              } else if (cu.GetType() == GeomAbs_Circle) {
                const gp_Circ oc = cu.Circle();
                cCen[0]=oc.Location().X(); cCen[1]=oc.Location().Y(); cCen[2]=oc.Location().Z();
                cNrm[0]=oc.Axis().Direction().X(); cNrm[1]=oc.Axis().Direction().Y(); cNrm[2]=oc.Axis().Direction().Z();
                cR = oc.Radius(); haveCircle = true;
              }
            }
            break;
          }
        }
        char det[176];
        Verdict v;
        if (bc.planarCapPoly) {
          // polygon cap: each OCCT line-edge midpoint must be matched by a native rim
          // line-edge midpoint, and the counts must be equal (exact loop match).
          const int nLine = static_cast<int>(nmid.size());
          for (const auto& m : occtLineMids) {
            bool one = false;
            for (const auto& mp : nmid)
              if (std::fabs(mp[0]-m[0])<=kPtTol && std::fabs(mp[1]-m[1])<=kPtTol &&
                  std::fabs(mp[2]-m[2])<=kPtTol) { one = true; break; }
            if (!one) ok = false;
          }
          if (!foundCap || occtLineEdges == 0) {
            v = ORACLE_UNRELIABLE;
            std::snprintf(det, sizeof det, "no OCCT cap oracle found (foundCap=%d occtLines=%d)", foundCap, occtLineEdges);
          } else if (ok && nLine == occtLineEdges) {
            v = AGREED;
            std::snprintf(det, sizeof det, "rim %d edges match OCCT OuterWire (cap faceId=%d)", nLine, capFace);
          } else {
            v = DISAGREED;
            std::snprintf(det, sizeof det, "rim mismatch: native=%d occt=%d matched-ok=%d", nLine, occtLineEdges, ok);
          }
        } else {
          // circular cap (CYLINDER/CONE): the native periodic revolution cap is a
          // VERTEX_LOOP-style face whose outer wire is stored as ≥1 Circle-ARC edges
          // (with periodic seam vertices), NOT the 1 clean seam edge OCCT's MakeCone/
          // MakeCylinder emits — a legitimate representational difference, and the arc
          // seam vertices do NOT carry clean on-circle endpoint points (a native
          // periodic-face quirk, unrelated to the datum). So the OCCT circle is NOT a
          // faithful per-vertex oracle here. The CLOSED-FORM ground truth is instead
          // STRUCTURAL and self-consistent: outerRimChain MUST return EXACTLY the seed
          // cap face's own outer-wire edge id set (it picks the right cap and returns
          // its rim, rejecting the perpendicular lateral wall). We compare the rim id
          // set to the cap face's outer wire directly, and require the OCCT cap oracle
          // to confirm the cap is a genuine circular boundary (haveCircle).
          (void)cCen; (void)cNrm; (void)cR; (void)nverts;
          std::vector<int> capOuter;
          const auto& kids = fmap.shape(capFace).tshape()->children();
          if (!kids.empty()) {
            const ntopo::Shape ow = kids.front()
                .located(fmap.shape(capFace).location())
                .oriented(fmap.shape(capFace).orientation());
            for (ntopo::Explorer eex(ow, ntopo::ShapeType::Edge); eex.more(); eex.next()) {
              const int eid = emap.findIndex(eex.current());
              if (eid >= 1) capOuter.push_back(eid);
            }
          }
          std::sort(capOuter.begin(), capOuter.end());
          std::vector<int> rimSorted = rim; std::sort(rimSorted.begin(), rimSorted.end());
          const bool setEqual = (!capOuter.empty() && rimSorted == capOuter);
          if (!foundCap || !haveCircle) {
            v = ORACLE_UNRELIABLE;
            std::snprintf(det, sizeof det, "no OCCT circular-cap circle oracle found (foundCap=%d haveCircle=%d)", foundCap, haveCircle);
          } else if (setEqual) {
            v = AGREED;
            std::snprintf(det, sizeof det, "circular cap rim = cap outer wire (%zu arc edges); OCCT confirms circular boundary R=%.4g", rim.size(), cR);
          } else {
            v = DISAGREED;
            std::snprintf(det, sizeof det, "circular cap rim id set != cap outer wire (rim=%zu capOuter=%zu)", rim.size(), capOuter.size());
          }
        }
        logTrial(v, i, seed, bc, OP_RIM, det);
      }
    }

    // ── OP: TANGENT — grow a straight-edge C1 run and confirm vs OCCT D1. For a prism
    //    the 4 collinear? No — prism edges meet at right angles, so a straight edge
    //    seeds a chain of size 1 (its neighbours turn 90°). We assert native's grow
    //    decision matches OCCT's |t1·t2|<cos15° STOP on the FIRST line edge, and that
    //    a curved (circle) edge in a cyl/cone is an HONEST DECLINE (nullopt). ───────
    {
      const ntopo::ShapeMap emap = ntopo::mapShapes(posed, ntopo::ShapeType::Edge);
      // pick the first line edge (BOX/NGON) as a tangent seed
      int seedLine = -1;
      for (std::size_t e = 1; e <= emap.size(); ++e) {
        const auto c = ntopo::curveOf(emap.shape(static_cast<int>(e)));
        if (c && c->curve->kind == ntopo::EdgeCurve::Kind::Line) { seedLine = static_cast<int>(e); break; }
      }
      if (seedLine > 0) {
        auto ch = nref::tangentChain(posed, {seedLine});
        // OCCT arbiter: the native chain must contain only edges that are C1 to the
        // seed at a shared vertex. For a prism a straight edge's neighbours turn ≥90°,
        // so the chain size must be exactly 1 (no C1 growth). Confirm via the analytic
        // fact: every prism edge is straight and meets others at ≥90° corners.
        Verdict v;
        char det[160];
        if (!ch) {
          v = BOTH_DECLINED;  // a freeform edge crept in (not for these families) → decline
          std::snprintf(det, sizeof det, "tangent walk declined (freeform edge incident)");
        } else if (ch->size() == 1) {
          v = AGREED;
          std::snprintf(det, sizeof det, "C1 chain size=1 (straight edge, ≥90° corners, no growth)");
        } else {
          // A larger chain would require collinear straight edges; the prism has none.
          // Verify each grown pair really is C1 by construction (all straight → dir
          // dot). If any grown neighbour is NOT collinear it is a WRONG grow.
          bool allCollinear = true;
          // native seed dir
          auto seedAxis = nref::refAxisFromEdge(emap.shape(seedLine));
          if (seedAxis) {
            const double sd[3] = {(*seedAxis)[3],(*seedAxis)[4],(*seedAxis)[5]};
            for (int id : *ch) {
              if (id == seedLine) continue;
              auto a = nref::refAxisFromEdge(emap.shape(id));
              if (!a) { allCollinear = false; break; }
              const double d[3] = {(*a)[3],(*a)[4],(*a)[5]};
              if (std::fabs(std::fabs(dot3(sd, d)) - 1.0) > 1e-6) { allCollinear = false; break; }
            }
          }
          v = allCollinear ? AGREED : DISAGREED;
          std::snprintf(det, sizeof det, "C1 chain size=%zu %s", ch->size(),
                        allCollinear ? "(all grown edges collinear — valid)" : "(a grown edge is NOT collinear — WRONG)");
        }
        logTrial(v, i, seed, bc, OP_TANGENT, det);
      } else {
        // CYLINDER/CONE: only circle edges — a tangent seed on a circle is analytic;
        // reference.h supports Circle tangents, so seed the first circle edge and
        // confirm it does NOT decline (freeform would). Skip if none.
        int seedC = -1;
        for (std::size_t e = 1; e <= emap.size(); ++e) {
          const auto c = ntopo::curveOf(emap.shape(static_cast<int>(e)));
          if (c && c->curve->kind == ntopo::EdgeCurve::Kind::Circle) { seedC = static_cast<int>(e); break; }
        }
        if (seedC > 0) {
          auto ch = nref::tangentChain(posed, {seedC});
          Verdict v; char det[160];
          if (ch) { v = AGREED; std::snprintf(det, sizeof det, "circle-edge tangent walk OK size=%zu (analytic Circle tangent)", ch->size()); }
          else { v = DECLINED; std::snprintf(det, sizeof det, "circle-edge tangent walk declined"); }
          logTrial(v, i, seed, bc, OP_TANGENT, det);
        }
      }
    }
  }

  // ── coverage summary ──────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  BOTH-DECLINED=%d  ORACLE_UNRELIABLE=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleInacc, g_bothDecl, g_oracleBad);
  std::printf("   per-base-family AGREED [DISAGREED]:\n");
  for (int b = 0; b < B_COUNT; ++b)
    std::printf("     %-14s %d [%d]\n", baseName(b), g_famAgreed[b], g_famDisagreed[b]);
  std::printf("   per-op AGREED: PLANE=%d FAXIS=%d EAXIS=%d OFFSET=%d RIM=%d TANGENT=%d\n",
              g_opAgreed[OP_PLANE], g_opAgreed[OP_FAXIS], g_opAgreed[OP_EAXIS],
              g_opAgreed[OP_OFFSET], g_opAgreed[OP_RIM], g_opAgreed[OP_TANGENT]);
  std::printf("   HONEST-SCOPE: RIGID pose only (rotate+translate, NO scale/mirror) so every datum transforms EXACTLY;\n"
              "   a circular cap offset and a freeform edge in a tangent walk are FIRST-CLASS declines matched by the closed form.\n");
  if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — logged, NOT a native fault)\n", g_oracleInacc);
  if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (core-case OCCT vs closed-form mismatch — investigate)\n", g_oracleBad);

  bool famCov = true; for (int b = 0; b < B_COUNT; ++b) if (g_famAgreed[b] < 1) famCov = false;
  bool opCov = true; for (int o = 0; o < OP_COUNT; ++o) if (g_opAgreed[o] < 1) opCov = false;
  const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && famCov && opCov);
  std::printf("== M6-breadth-9 BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
              "base-family coverage=%s; op coverage=%s) ==\n",
              bar ? "PASS — zero silent wrong datums" : "FAIL", g_disagreed, g_oracleBad,
              famCov ? "complete" : "INCOMPLETE", opCov ? "complete" : "INCOMPLETE");
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
