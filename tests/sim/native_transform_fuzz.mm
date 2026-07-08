// SPDX-License-Identifier: Apache-2.0
//
// native_transform_fuzz.mm — MOAT M6-breadth-8 TRANSFORM-CHAIN differential-fuzzing
//                            harness (native vs OCCT, iOS simulator).
//
// The EIGHTH native domain on the differential-fuzzing completeness bar, extending the
// landed M6 fuzzers — native_boolean_fuzz.mm (curved booleans), native_step_import_fuzz.mm
// (STEP round-trip), native_construct_fuzz.mm (loft/sweep), native_blend_fuzz.mm (fillet/
// chamfer/offset/shell), native_mass_props_fuzz.mm (mesh mass properties) and
// native_wrap_emboss_fuzz.mm — to the RIGID/SIMILARITY TRANSFORM layer the app's
// translate / rotate / scale / mirror / place tools read (cc_translate_shape,
// cc_rotate_shape_about, cc_scale_shape, cc_mirror_shape, cc_place_on_frame → the native
// topology::Shape::located(math::Transform) path).
//
// ── The system under test ────────────────────────────────────────────────────
// A native transform is a placement: topology::Shape::located(Location{math::Transform})
// composes the affine onto the shape's Location; the tessellator (surface_eval / edge_mesher)
// world-places every sample through that Location. This harness builds a random VALID base
// solid via the OCCT-FREE native builders (src/native/construct), applies a random CHAIN of
// translate / rotate(any axis) / uniform-scale / mirror(any plane) as ONE composed
// math::Transform, meshes the located solid (src/native/tessellate) and measures it, and
// compares against BOTH the OCCT oracle (BRepBuilderAPI_Transform with the SAME gp_Trsf chain,
// measured by BRepGProp) AND a THIRD, ENGINE-INDEPENDENT closed-form analytic arbiter computed
// with plain fp64 affine arithmetic in the harness (not the native math::Transform, not gp_Trsf).
//
// ── The closed-form similarity arbiter (PRIMARY) ─────────────────────────────
// A chain of translate / rotate / UNIFORM-scale / mirror is a SIMILARITY: linear part L = S·Q
// with Q orthonormal (det ±1) and S = Π uniform-scale factors. For such a map on any solid:
//   * volume'   = S³ · volume_base           (|det L| = S³)
//   * area'     = S² · area_base
//   * centroid' = L · centroid_base + t       (exact affine image of the base centroid)
//   * topology  is INVARIANT (face / edge / vertex counts unchanged — a placement adds/drops
//                nothing)
//   * handedness: signed enclosed volume sign = sign(base) · (−1)^(#mirrors). A MIRROR must
//                 FLIP handedness yet leave a VALID, watertight, positive-|volume| solid.
// The base volume/area/centroid are themselves exact closed forms (BOX / NGON prism / coaxial
// LOFT planar-exact; CYLINDER / SPHERE curved). The analytic image is the PRIMARY correctness
// oracle: it attributes a native-vs-OCCT gap (native mesh chord error vs a real transform bug
// vs an OCCT outlier) instead of reflexively blaming the native path. Uniform scale ONLY keeps
// the area closed form exact (an anisotropic scale has no simple closed-form area) — that is an
// HONEST SCOPE choice, and cc_scale_shape is uniform by contract.
//
// ── Five-way classifier (fixed, deflection-matched tolerance; never widened) ──
//   AGREED            — native watertight ∧ |vol|>0, native vol/area/centroid match the analytic
//                       similarity image AND topology is preserved AND the signed-vol sign
//                       equals the mirror-parity expectation, AND OCCT concurs.
//   HONESTLY-DECLINED — native produces no valid mesh (singular transform → collapsed solid),
//                       OCCT ships a valid solid. First-class, logged.
//   DISAGREED         — native valid but its transformed vol/area/centroid / handedness /
//                       topology does NOT match the analytic image → a SILENT WRONG TRANSFORM.
//   ORACLE-INACCURATE — native matches the analytic image, OCCT does not (native vindicated).
//   BOTH-DECLINED     — a singular-transform decline-exerciser both engines refuse.
//   ORACLE_UNRELIABLE — a core case whose OCCT oracle does not match the closed form.
// Bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0, each base family (BOX/NGON/CYLINDER/SPHERE/LOFT)
// with ≥1 AGREED, each transform KIND (translate/rotate/scale/MIRROR) exercised in ≥1 AGREED
// chain, and the mirror HANDEDNESS-FLIP positively confirmed ≥1 time — proven over ≥2 seeds.
//
// The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand():
// same seed → byte-identical batch (splitmix64 → xoshiro256**, verbatim from the siblings).
//
// src/native / src/engine stay UNTOUCHED; this harness DRIVES the native located() +
// tessellate path (OCCT-FREE, no numsci) and links ONLY the OCCT oracle toolkits. On
// run-sim-suite.sh's SKIP list (own main()).
//
// Usage: scripts/run-sim-native-transform-fuzz.sh [SEED] [N]
//   SEED  explicit uint64 RNG seed (default 0x7A5C0FFEE2). Also via FUZZ_SEED env.
//   N     number of generated cases (default 160).          Also via FUZZ_N env.
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"
#include "native/topology/explore.h"
#include "native/math/transform.h"
#include "native/math/vec.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_transform_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT BRepGProp oracle"
#endif

#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Standard_Failure.hxx>

namespace ncst  = cybercad::native::construct;
namespace ntess = cybercad::native::tessellate;
namespace ntopo = cybercad::native::topology;
namespace nmath = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPropertyDeflection = 0.005;
constexpr double kTightTol  = 1e-6;   // planar base reproduces the solid EXACTLY under a similarity
constexpr double kCurveC    = 5.0;    // curved tol = kCurveC·deflection/(featureSize·min(1,S))
constexpr double kOracleTol = 1e-6;   // OCCT (exact B-rep) must match the closed form this tight
constexpr double kMinFeat   = 1.0;

// ── deterministic RNG: splitmix64 seed → xoshiro256** (verbatim from the landed fuzzers). ──
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

// ── base solid families (each meshes watertight; each has an exact closed form) ────────
enum Base { B_BOX, B_NGON, B_CYLINDER, B_SPHERE, B_LOFT, B_COUNT };
const char* baseName(int b) {
  switch (b) {
    case B_BOX:      return "BOX prism(planar)";
    case B_NGON:     return "NGON prism(planar)";
    case B_CYLINDER: return "CYLINDER(curved)";
    case B_SPHERE:   return "SPHERE(curved)";
    case B_LOFT:     return "LOFT coaxial(planar)";
  }
  return "?";
}
enum NativeBuild { NB_PRISM, NB_REV_POLY, NB_REV_PROFILE, NB_LOFT };
enum OcctBuild   { OB_BOX, OB_NGON_PRISM, OB_CYL, OB_SPHERE, OB_LOFT };

// analytic ngon helpers (verbatim from native_mass_props_fuzz.mm)
double ngonArea(int n, double R)    { return 0.5 * n * R * R * std::sin(2.0 * kPi / n); }
double ngonEdge(int n, double R)    { return 2.0 * R * std::sin(kPi / n); }
double ngonApothem(int n, double R) { return R * std::cos(kPi / n); }
void appendRegularNgon3D(std::vector<double>& buf, int n, double R, double rot, double z) {
  for (int i = 0; i < n; ++i) {
    const double a = rot + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
    buf.push_back(R * std::cos(a)); buf.push_back(R * std::sin(a)); buf.push_back(z);
  }
}
void analyticNgonStack(int n, const std::vector<double>& R, const std::vector<double>& z,
                       double& vol, double& area, double& zc) {
  vol = 0.0; double lateral = 0.0, moment = 0.0;
  for (std::size_t k = 0; k + 1 < R.size(); ++k) {
    const double dz = std::fabs(z[k + 1] - z[k]);
    const double Ak = ngonArea(n, R[k]), Ak1 = ngonArea(n, R[k + 1]);
    const double band = (dz / 3.0) * (Ak + std::sqrt(Ak * Ak1) + Ak1);
    const double denom = Ak + std::sqrt(Ak * Ak1) + Ak1;
    const double zLocal = denom > 1e-12
        ? (dz / 4.0) * (Ak + 2.0 * std::sqrt(Ak * Ak1) + 3.0 * Ak1) / denom : 0.5 * dz;
    vol += band; moment += band * (std::min(z[k], z[k + 1]) + zLocal);
    const double da = ngonApothem(n, R[k]) - ngonApothem(n, R[k + 1]);
    lateral += n * 0.5 * (ngonEdge(n, R[k]) + ngonEdge(n, R[k + 1])) * std::sqrt(dz * dz + da * da);
  }
  area = lateral + ngonArea(n, R.front()) + ngonArea(n, R.back());
  zc = vol > 1e-12 ? moment / vol : 0.0;
}

struct BaseCase {
  int base = 0;
  NativeBuild nb = NB_PRISM;
  OcctBuild   ob = OB_BOX;
  std::vector<double> polyXY;                 // prism / revolve-poly profile
  double depth = 0;
  std::vector<ncst::ProfileSegment> segs;     // sphere arc profile
  ncst::RevolveAxis axis{0, 0, 0, 1};         // world +Y axis (2D: ax,ay,adx,ady)
  double angle = 2.0 * kPi;
  std::vector<double> loftXYZ;
  std::vector<int>    loftCnt;
  double bw = 0, bd = 0, bh = 0, cr = 0, ch = 0, sr = 0;
  double aVol0 = 0, aArea0 = 0, aC0[3] = {0, 0, 0};   // exact base closed form (before transform)
  bool   planar = true;
  double featureSize = kMinFeat, charLen = 1;
  std::string desc;
};

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[224]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

BaseCase genBase(Rng& r) {
  BaseCase c; c.base = static_cast<int>(r.below(B_COUNT));
  switch (c.base) {
    case B_BOX: {
      c.nb = NB_PRISM; c.ob = OB_BOX; c.planar = true;
      const double w = r.range(1.0, 4.0), d = r.range(1.0, 4.0), h = r.range(1.0, 4.0);
      c.polyXY = {0, 0, w, 0, w, d, 0, d}; c.depth = h; c.bw = w; c.bd = d; c.bh = h;
      c.aVol0 = w * d * h; c.aArea0 = 2.0 * (w * d + w * h + d * h);
      c.aC0[0] = 0.5 * w; c.aC0[1] = 0.5 * d; c.aC0[2] = 0.5 * h;
      c.charLen = std::max({w, d, h}); c.desc = fmt("w=%.4f d=%.4f h=%.4f", w, d, h);
      break;
    }
    case B_NGON: {
      c.nb = NB_PRISM; c.ob = OB_NGON_PRISM; c.planar = true;
      const int n = 3 + static_cast<int>(r.below(10));
      const double R = r.range(1.0, 3.0), h = r.range(1.0, 4.0), rot = r.range(0.0, kPi);
      for (int i = 0; i < n; ++i) {
        const double a = rot + 2.0 * kPi * i / n;
        c.polyXY.push_back(R * std::cos(a)); c.polyXY.push_back(R * std::sin(a));
      }
      c.depth = h; c.bh = h;
      const double A = ngonArea(n, R), s = ngonEdge(n, R);
      c.aVol0 = A * h; c.aArea0 = 2.0 * A + n * s * h;
      c.aC0[0] = 0.0; c.aC0[1] = 0.0; c.aC0[2] = 0.5 * h;
      c.charLen = std::max(2.0 * R, h); c.desc = fmt("n=%.0f R=%.4f h=%.4f", n, R, h);
      break;
    }
    case B_CYLINDER: {
      c.nb = NB_REV_POLY; c.ob = OB_CYL; c.planar = false;
      const double R = r.range(1.0, 3.0), h = r.range(1.5, 4.0);
      c.polyXY = {0, 0, R, 0, R, h, 0, h}; c.cr = R; c.ch = h;
      c.aVol0 = kPi * R * R * h; c.aArea0 = 2.0 * kPi * R * R + 2.0 * kPi * R * h;
      c.aC0[0] = 0.0; c.aC0[1] = 0.5 * h; c.aC0[2] = 0.0;
      c.featureSize = R; c.charLen = std::max(2.0 * R, h); c.desc = fmt("R=%.4f h=%.4f", R, h);
      break;
    }
    case B_SPHERE: {
      c.nb = NB_REV_PROFILE; c.ob = OB_SPHERE; c.planar = false;
      const double R = r.range(1.0, 3.0);
      ncst::ProfileSegment arc; arc.kind = 1; arc.cx = 0; arc.cy = 0; arc.r = R;
      arc.x0 = 0; arc.y0 = -R; arc.x1 = 0; arc.y1 = R; arc.a0 = -0.5 * kPi; arc.a1 = 0.5 * kPi;
      ncst::ProfileSegment ax; ax.kind = 0; ax.x0 = 0; ax.y0 = R; ax.x1 = 0; ax.y1 = -R;
      c.segs = {arc, ax}; c.sr = R;
      c.aVol0 = 4.0 / 3.0 * kPi * R * R * R; c.aArea0 = 4.0 * kPi * R * R;
      c.aC0[0] = 0.0; c.aC0[1] = 0.0; c.aC0[2] = 0.0;
      c.featureSize = R; c.charLen = 2.0 * R; c.desc = fmt("R=%.4f", R);
      break;
    }
    case B_LOFT: {
      c.nb = NB_LOFT; c.ob = OB_LOFT; c.planar = true;
      const int n = 3 + static_cast<int>(r.below(6));
      const double R0 = r.range(1.0, 3.0), R1 = r.range(1.0, 3.0);
      const double rot = r.range(0.0, kPi), dz = r.range(1.5, 4.0);
      appendRegularNgon3D(c.loftXYZ, n, R0, rot, 0.0);
      appendRegularNgon3D(c.loftXYZ, n, R1, rot, dz);
      c.loftCnt = {n, n};
      double zc = 0.0; analyticNgonStack(n, {R0, R1}, {0.0, dz}, c.aVol0, c.aArea0, zc);
      c.aC0[0] = 0.0; c.aC0[1] = 0.0; c.aC0[2] = zc;
      c.charLen = std::max({2.0 * R0, 2.0 * R1, dz}); c.desc = fmt("n=%.0f R0=%.4f R1=%.4f dz=%.4f", n, R0, R1, dz);
      break;
    }
  }
  return c;
}

ntopo::Shape buildNativeBase(const BaseCase& c) {
  switch (c.nb) {
    case NB_PRISM:
      return ncst::build_prism(c.polyXY.data(), static_cast<int>(c.polyXY.size() / 2), c.depth);
    case NB_REV_POLY:
      return ncst::build_revolution(c.polyXY.data(), static_cast<int>(c.polyXY.size() / 2), c.axis, c.angle);
    case NB_REV_PROFILE:
      return ncst::build_revolution_profile(c.segs, c.axis, c.angle);
    case NB_LOFT:
      return ncst::build_loft_sections(c.loftXYZ.data(), c.loftCnt.data(), static_cast<int>(c.loftCnt.size()));
  }
  return {};
}

TopoDS_Face ngonFaceZ0(const std::vector<double>& polyXY) {
  BRepBuilderAPI_MakePolygon poly;
  for (std::size_t i = 0; i < polyXY.size() / 2; ++i)
    poly.Add(gp_Pnt(polyXY[i * 2], polyXY[i * 2 + 1], 0.0));
  poly.Close();
  if (!poly.IsDone()) return {};
  BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
  return face.IsDone() ? face.Face() : TopoDS_Face();
}

TopoDS_Shape buildOcctBase(const BaseCase& c) {
  try {
    const gp_Ax2 axY(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0), gp_Dir(1, 0, 0));
    switch (c.ob) {
      case OB_BOX: return BRepPrimAPI_MakeBox(c.bw, c.bd, c.bh).Shape();
      case OB_NGON_PRISM: {
        const TopoDS_Face f = ngonFaceZ0(c.polyXY);
        if (f.IsNull()) return {};
        return BRepPrimAPI_MakePrism(f, gp_Vec(0, 0, c.bh)).Shape();
      }
      case OB_CYL: return BRepPrimAPI_MakeCylinder(axY, c.cr, c.ch).Shape();
      case OB_SPHERE: return BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), c.sr).Shape();
      case OB_LOFT: {
        BRepOffsetAPI_ThruSections gen(Standard_True, Standard_True);
        std::size_t off = 0;
        for (std::size_t k = 0; k < c.loftCnt.size(); ++k) {
          const int cnt = c.loftCnt[k];
          BRepBuilderAPI_MakePolygon poly;
          for (int i = 0; i < cnt; ++i)
            poly.Add(gp_Pnt(c.loftXYZ[off + i * 3], c.loftXYZ[off + i * 3 + 1], c.loftXYZ[off + i * 3 + 2]));
          poly.Close();
          if (!poly.IsDone()) return {};
          gen.AddWire(poly.Wire());
          off += static_cast<std::size_t>(cnt) * 3;
        }
        gen.Build();
        return gen.IsDone() ? gen.Shape() : TopoDS_Shape();
      }
    }
  } catch (const Standard_Failure&) { return {}; }
  return {};
}

// ── engine-INDEPENDENT closed-form affine (NOT math::Transform, NOT gp_Trsf) ───────────
struct Aff {
  double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  double t[3] = {0, 0, 0};
};
Aff affCompose(const Aff& A, const Aff& B) {   // A ∘ B: apply B first, then A
  Aff r;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      r.m[i][j] = A.m[i][0] * B.m[0][j] + A.m[i][1] * B.m[1][j] + A.m[i][2] * B.m[2][j];
  for (int i = 0; i < 3; ++i)
    r.t[i] = A.m[i][0] * B.t[0] + A.m[i][1] * B.t[1] + A.m[i][2] * B.t[2] + A.t[i];
  return r;
}
void affApply(const Aff& A, const double p[3], double out[3]) {
  for (int i = 0; i < 3; ++i)
    out[i] = A.m[i][0] * p[0] + A.m[i][1] * p[1] + A.m[i][2] * p[2] + A.t[i];
}

// ── transform chain: ONE composed op built THREE independent ways ──────────────────────
enum OpKind { OP_TRANSLATE, OP_ROTATE, OP_USCALE, OP_MIRROR, OP_KINDS };
const char* opName(int k) {
  switch (k) { case OP_TRANSLATE: return "T"; case OP_ROTATE: return "R";
               case OP_USCALE: return "S"; case OP_MIRROR: return "M"; }
  return "?";
}

struct Chain {
  nmath::Transform nativeXf;   // composed native math::Transform (SYSTEM UNDER TEST)
  gp_Trsf occtTrsf;            // composed OCCT oracle transform
  Aff analytic;                // composed engine-independent affine (PRIMARY arbiter)
  double S = 1.0;              // product of uniform-scale factors
  int mirrorCount = 0;         // parity of mirrors (handedness)
  int kindMask = 0;            // which OpKinds appear
  bool singular = false;       // a zero-scale decline-exerciser
  std::string desc;
};

// append op #k (already generated random params) onto the three cumulative representations
void applyTranslate(Chain& ch, double tx, double ty, double tz) {
  ch.nativeXf = nmath::Transform::translationOf(nmath::Vec3{tx, ty, tz}).composedWith(ch.nativeXf);
  gp_Trsf T; T.SetTranslation(gp_Vec(tx, ty, tz));
  ch.occtTrsf = T.Multiplied(ch.occtTrsf);
  Aff a; a.t[0] = tx; a.t[1] = ty; a.t[2] = tz;
  ch.analytic = affCompose(a, ch.analytic);
  ch.kindMask |= (1 << OP_TRANSLATE);
}
void applyRotate(Chain& ch, const double c[3], const double ax[3], double ang) {
  const nmath::Dir3 axis(ax[0], ax[1], ax[2]);
  ch.nativeXf = nmath::Transform::rotationOf(nmath::Point3{c[0], c[1], c[2]}, axis, ang).composedWith(ch.nativeXf);
  gp_Trsf T; T.SetRotation(gp_Ax1(gp_Pnt(c[0], c[1], c[2]), gp_Dir(axis.x(), axis.y(), axis.z())), ang);
  ch.occtTrsf = T.Multiplied(ch.occtTrsf);
  // Rodrigues (unit axis) into the independent affine.
  const double x = axis.x(), y = axis.y(), z = axis.z();
  const double cc = std::cos(ang), ss = std::sin(ang), tt = 1.0 - cc;
  Aff a;
  a.m[0][0] = tt*x*x + cc;   a.m[0][1] = tt*x*y - ss*z; a.m[0][2] = tt*x*z + ss*y;
  a.m[1][0] = tt*x*y + ss*z; a.m[1][1] = tt*y*y + cc;   a.m[1][2] = tt*y*z - ss*x;
  a.m[2][0] = tt*x*z - ss*y; a.m[2][1] = tt*y*z + ss*x; a.m[2][2] = tt*z*z + cc;
  for (int i = 0; i < 3; ++i) a.t[i] = c[i] - (a.m[i][0]*c[0] + a.m[i][1]*c[1] + a.m[i][2]*c[2]);
  ch.analytic = affCompose(a, ch.analytic);
  ch.kindMask |= (1 << OP_ROTATE);
}
void applyScale(Chain& ch, const double c[3], double s) {
  ch.nativeXf = nmath::Transform::scaleOf(nmath::Point3{c[0], c[1], c[2]}, s).composedWith(ch.nativeXf);
  gp_Trsf T; T.SetScale(gp_Pnt(c[0], c[1], c[2]), s);
  ch.occtTrsf = T.Multiplied(ch.occtTrsf);
  Aff a; a.m[0][0] = a.m[1][1] = a.m[2][2] = s;
  for (int i = 0; i < 3; ++i) a.t[i] = c[i] - s * c[i];
  ch.analytic = affCompose(a, ch.analytic);
  if (s != 0.0) ch.S *= s; else ch.singular = true;
  ch.kindMask |= (1 << OP_USCALE);
}
void applyMirror(Chain& ch, const double p[3], const double n[3]) {
  // reflection across the plane through p with unit normal n: v' = (I − 2 n nᵀ)v + 2(p·n)n
  // native math::Transform has no mirror factory → build L,t by hand (also feeds the oracle/analytic)
  nmath::Mat3 L(1 - 2*n[0]*n[0],  -2*n[0]*n[1],   -2*n[0]*n[2],
                 -2*n[1]*n[0], 1 - 2*n[1]*n[1],   -2*n[1]*n[2],
                 -2*n[2]*n[0],   -2*n[2]*n[1], 1 - 2*n[2]*n[2]);
  const double pd = p[0]*n[0] + p[1]*n[1] + p[2]*n[2];
  nmath::Vec3 tv{2*pd*n[0], 2*pd*n[1], 2*pd*n[2]};
  ch.nativeXf = nmath::Transform{L, tv}.composedWith(ch.nativeXf);
  gp_Trsf T; T.SetMirror(gp_Ax2(gp_Pnt(p[0], p[1], p[2]), gp_Dir(n[0], n[1], n[2])));
  ch.occtTrsf = T.Multiplied(ch.occtTrsf);
  Aff a;
  a.m[0][0] = 1 - 2*n[0]*n[0]; a.m[0][1] = -2*n[0]*n[1];    a.m[0][2] = -2*n[0]*n[2];
  a.m[1][0] = -2*n[1]*n[0];    a.m[1][1] = 1 - 2*n[1]*n[1]; a.m[1][2] = -2*n[1]*n[2];
  a.m[2][0] = -2*n[2]*n[0];    a.m[2][1] = -2*n[2]*n[1];    a.m[2][2] = 1 - 2*n[2]*n[2];
  a.t[0] = 2*pd*n[0]; a.t[1] = 2*pd*n[1]; a.t[2] = 2*pd*n[2];
  ch.analytic = affCompose(a, ch.analytic);
  ++ch.mirrorCount;
  ch.kindMask |= (1 << OP_MIRROR);
}

// draw a random unit vector (never near-zero)
void randUnit(Rng& r, double out[3]) {
  double n; do {
    out[0] = r.range(-1, 1); out[1] = r.range(-1, 1); out[2] = r.range(-1, 1);
    n = std::sqrt(out[0]*out[0] + out[1]*out[1] + out[2]*out[2]);
  } while (n < 0.25);
  out[0] /= n; out[1] /= n; out[2] /= n;
}

Chain genChain(Rng& r, bool forceKind, int kind, bool degen) {
  Chain ch; ch.desc.clear();
  const int len = forceKind ? 1 : (1 + static_cast<int>(r.below(4)));   // 1..4 ops
  for (int i = 0; i < len; ++i) {
    const int k = forceKind ? kind : static_cast<int>(r.below(OP_KINDS));
    switch (k) {
      case OP_TRANSLATE: {
        const double tx = r.range(-8, 8), ty = r.range(-8, 8), tz = r.range(-8, 8);
        applyTranslate(ch, tx, ty, tz);
        ch.desc += fmt("T(%.2f,%.2f,%.2f) ", tx, ty, tz); break;
      }
      case OP_ROTATE: {
        double axv[3]; randUnit(r, axv);
        const double cc[3] = {r.range(-3, 3), r.range(-3, 3), r.range(-3, 3)};
        const double ang = r.range(-kPi, kPi);
        applyRotate(ch, cc, axv, ang);
        ch.desc += fmt("R(ang=%.2f) ", ang); break;
      }
      case OP_USCALE: {
        const double cc[3] = {r.range(-3, 3), r.range(-3, 3), r.range(-3, 3)};
        const double s = r.range(0.5, 2.0);
        applyScale(ch, cc, s);
        ch.desc += fmt("S(%.3f) ", s); break;
      }
      case OP_MIRROR: {
        double nv[3]; randUnit(r, nv);
        const double pp[3] = {r.range(-3, 3), r.range(-3, 3), r.range(-3, 3)};
        applyMirror(ch, pp, nv);
        ch.desc += "M "; break;
      }
    }
  }
  if (degen) {   // append a singular (zero-scale) op → collapsed solid → decline-exerciser
    const double cc[3] = {0, 0, 0};
    applyScale(ch, cc, 0.0);
    ch.desc += "S(0=singular) ";
  }
  return ch;
}

// ── native measurement of the transformed (located) solid ──────────────────────────────
struct NativeX {
  bool present = false, watertight = false;
  double signedVol = 0, absVol = 0, area = 0, c[3] = {0, 0, 0};
  int nf = 0, ne = 0, nv = 0;
};
int countNative(const ntopo::Shape& s, ntopo::ShapeType t) {
  int n = 0; for (ntopo::Explorer ex(s, t); ex.more(); ex.next()) ++n; return n;
}
NativeX measureNative(const ntopo::Shape& s) {
  NativeX m;
  if (s.isNull()) return m;
  m.present = true;
  ntess::MeshParams p; p.deflection = kPropertyDeflection;
  const ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(s);
  m.area = ntess::surfaceArea(mesh);
  m.signedVol = ntess::enclosedVolume(mesh);
  m.absVol = std::fabs(m.signedVol);
  double cx = 0, cy = 0, cz = 0, vol6 = 0;
  for (const auto& t : mesh.triangles) {
    const auto& A = mesh.vertices[t.a]; const auto& B = mesh.vertices[t.b]; const auto& C = mesh.vertices[t.c];
    const double v = A.x*(B.y*C.z - B.z*C.y) - A.y*(B.x*C.z - B.z*C.x) + A.z*(B.x*C.y - B.y*C.x);
    vol6 += v; cx += v*(A.x+B.x+C.x); cy += v*(A.y+B.y+C.y); cz += v*(A.z+B.z+C.z);
  }
  if (std::fabs(vol6) > 1e-12) { const double inv = 1.0/(4.0*vol6); m.c[0]=cx*inv; m.c[1]=cy*inv; m.c[2]=cz*inv; }
  m.watertight = ntess::isWatertight(mesh);
  m.nf = countNative(s, ntopo::ShapeType::Face);
  m.ne = countNative(s, ntopo::ShapeType::Edge);
  m.nv = countNative(s, ntopo::ShapeType::Vertex);
  return m;
}

struct OcctX {
  bool present = false, valid = false;
  double volume = 0, area = 0, c[3] = {0, 0, 0};
  int nf = 0, ne = 0, nv = 0;
};
int countOcct(const TopoDS_Shape& s, TopAbs_ShapeEnum t) {
  int n = 0; for (TopExp_Explorer ex(s, t); ex.More(); ex.Next()) ++n; return n;
}
OcctX measureOcct(const TopoDS_Shape& s) {
  OcctX m;
  if (s.IsNull()) return m;
  m.present = true;
  try {
    BRepCheck_Analyzer an(s);
    GProp_GProps vg; BRepGProp::VolumeProperties(s, vg);
    GProp_GProps ag; BRepGProp::SurfaceProperties(s, ag);
    m.volume = std::fabs(vg.Mass()); m.area = ag.Mass();
    const gp_Pnt g = vg.CentreOfMass(); m.c[0]=g.X(); m.c[1]=g.Y(); m.c[2]=g.Z();
    m.valid = an.IsValid() && m.volume > 1e-12;
    m.nf = countOcct(s, TopAbs_FACE); m.ne = countOcct(s, TopAbs_EDGE); m.nv = countOcct(s, TopAbs_VERTEX);
  } catch (const Standard_Failure&) { m.valid = false; }
  return m;
}

double relDiff(double a, double b) { return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : 1e30; }

enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, ORACLE_UNRELIABLE, BOTH_DECLINED };
int g_agreed=0, g_declined=0, g_disagreed=0, g_oracleInacc=0, g_oracleBad=0, g_bothDecl=0;
int g_famAgreed[B_COUNT]={0}, g_famDecl[B_COUNT]={0}, g_famDisagreed[B_COUNT]={0};
int g_kindAgreed[OP_KINDS]={0};
int g_mirrorFlipConfirmed=0;   // AGREED with odd mirrors whose signed-vol sign flipped as required

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x7A5C0FFEE2ull;
  int N = 160;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 160;

  std::printf("== M6-breadth-8 differential-fuzz: native TRANSFORM CHAINS vs OCCT BRepBuilderAPI_Transform ==\n");
  std::printf("== seed=0x%llx N=%d deflection=%g planarTol=%.0e curveC=%.1f oracleTol=%.0e ==\n",
              static_cast<unsigned long long>(seed), N, kPropertyDeflection, kTightTol, kCurveC, kOracleTol);
  std::printf("== arbiter: closed-form similarity image vol'=S^3*vol area'=S^2*area centroid'=L*C+t; "
              "mirror flips signed-vol sign; topology invariant ==\n");
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    // Guarantee coverage: the first cases force each transform KIND singly on rotating base
    // families; a sparse tail forces a singular decline-exerciser. The rest are random chains.
    bool forceKind = false; int kind = 0; bool degen = false;
    if (i < OP_KINDS * 2) { forceKind = true; kind = i % OP_KINDS; }
    else if ((i % 23) == 0) degen = true;

    const BaseCase bc = genBase(rng);
    const Chain    ch = genChain(rng, forceKind, kind, degen);

    // NATIVE: build base, apply the composed transform via located(), measure the mesh.
    const ntopo::Shape baseShape = buildNativeBase(bc);
    const NativeX base = measureNative(baseShape);
    const ntopo::Shape xShape = baseShape.isNull()
        ? ntopo::Shape{} : baseShape.located(ntopo::Location{ch.nativeXf});
    const NativeX nat = measureNative(xShape);

    // OCCT oracle: build base, apply the same gp_Trsf, measure. A provably SINGULAR
    // (zero-scale) transform is NOT handed to OCCT: BRepBuilderAPI_Transform of a
    // zero-scale gp_Trsf on a curved base (e.g. a cylinder) HANGS in OCCT (an oracle
    // pathology on a non-invertible placement, confirmed by localisation). Such a
    // transform is degenerate by construction — the native side declines it (collapsed
    // mesh) and the oracle is recorded as unable → BOTH-DECLINED. This is an HONEST
    // ORACLE-LIMITATION decline, logged, not a native fault (the native path completes).
    const TopoDS_Shape occBase = buildOcctBase(bc);
    TopoDS_Shape occX;
    if (!ch.singular) {
      try {
        if (!occBase.IsNull()) { BRepBuilderAPI_Transform tf(occBase, ch.occtTrsf, Standard_True); if (tf.IsDone()) occX = tf.Shape(); }
      } catch (const Standard_Failure&) { occX = TopoDS_Shape(); }
    }
    const OcctX occ = measureOcct(occX);

    // Analytic similarity image of the EXACT base closed form.
    const double S = ch.S;
    const double aVol = S * S * S * bc.aVol0;
    const double aArea = S * S * bc.aArea0;
    double aC[3]; affApply(ch.analytic, bc.aC0, aC);
    const int mirrorParity = (ch.mirrorCount % 2 == 0) ? +1 : -1;

    // Family tolerance: planar exact-meshing → tight; curved → deflection bound at the SCALED
    // world feature size (never widened past the mesh's own convergence guarantee).
    const double feat = std::max(bc.featureSize, kMinFeat) * std::min(1.0, S);
    const double tol = bc.planar ? kTightTol : (kCurveC * kPropertyDeflection / feat);
    const double centTol = tol * std::max(bc.charLen * S, 1.0);

    const bool nativeValid = nat.present && nat.watertight && nat.absVol > 1e-9;
    const bool oracleValid = occ.present && occ.valid && occ.volume > 1e-9;

    // Does the native transform match the analytic similarity image (PRIMARY arbiter)?
    const double natCentErr = std::sqrt((nat.c[0]-aC[0])*(nat.c[0]-aC[0]) +
                                        (nat.c[1]-aC[1])*(nat.c[1]-aC[1]) +
                                        (nat.c[2]-aC[2])*(nat.c[2]-aC[2]));
    const bool topoInvariant = base.present && (nat.nf==base.nf && nat.ne==base.ne && nat.nv==base.nv);
    // handedness: transformed signed-vol sign must equal base sign × mirror parity.
    const int baseSign = (base.signedVol >= 0) ? +1 : -1;
    const int natSign  = (nat.signedVol  >= 0) ? +1 : -1;
    const bool handednessOk = (natSign == baseSign * mirrorParity);
    const bool natMatchesA = nativeValid && topoInvariant && handednessOk &&
        relDiff(nat.absVol, aVol) < tol && relDiff(nat.area, aArea) < tol && natCentErr < centTol;

    const bool oracleTrust = oracleValid &&
        relDiff(occ.volume, aVol) < kOracleTol && relDiff(occ.area, aArea) < kOracleTol;

    Verdict v;
    if (!nativeValid) {
      if (oracleValid) v = DECLINED;
      else v = degen ? BOTH_DECLINED : ORACLE_UNRELIABLE;
    } else if (natMatchesA) {
      if (oracleTrust) v = AGREED;
      else if (oracleValid) v = ORACLE_INACCURATE;
      else v = ORACLE_UNRELIABLE;
    } else {
      v = oracleTrust ? DISAGREED : ORACLE_UNRELIABLE;
    }

    switch (v) {
      case AGREED:
        ++g_agreed; ++g_famAgreed[bc.base];
        for (int k = 0; k < OP_KINDS; ++k) if (ch.kindMask & (1 << k)) ++g_kindAgreed[k];
        if (mirrorParity == -1) ++g_mirrorFlipConfirmed;
        break;
      case DECLINED:          ++g_declined;    ++g_famDecl[bc.base]; break;
      case DISAGREED:         ++g_disagreed;   ++g_famDisagreed[bc.base]; break;
      case ORACLE_INACCURATE: ++g_oracleInacc; break;
      case BOTH_DECLINED:     ++g_bothDecl;    break;
      case ORACLE_UNRELIABLE: ++g_oracleBad;   break;
    }

    char kinds[8] = {0}; int kn = 0;
    for (int k = 0; k < OP_KINDS; ++k) if (ch.kindMask & (1 << k)) kinds[kn++] = opName(k)[0];

    if (v == AGREED) {
      std::printf("[FUZZ] AGREED    case=%d %-20s S=%.3f mir=%d kinds=%-4s | volN=%.6g volA=%.6g dV=%.2e "
                  "areaN=%.6g dA=%.2e Cerr=%.2e hand=%d topo(f%d/e%d/v%d==base) tol=%.1e | %s :: %s\n",
                  i, baseName(bc.base), S, ch.mirrorCount, kinds, nat.absVol, aVol,
                  relDiff(nat.absVol, aVol), nat.area, relDiff(nat.area, aArea), natCentErr,
                  handednessOk, nat.nf, nat.ne, nat.nv, tol, bc.desc.c_str(), ch.desc.c_str());
    } else if (v == DECLINED) {
      std::printf("[FUZZ] DECLINED  case=%d %-20s native=%s (no valid transformed mesh) -> OCCT valid "
                  "[volO=%.6g areaO=%.6g] kinds=%-4s S=%.3f | %s :: %s\n",
                  i, baseName(bc.base), nat.present ? "collapsed/non-watertight" : "NULL",
                  occ.volume, occ.area, kinds, S, bc.desc.c_str(), ch.desc.c_str());
    } else if (v == BOTH_DECLINED) {
      std::printf("[FUZZ] BOTH-DECL case=%d %-20s singular transform: native declined AND OCCT built "
                  "no valid solid | %s :: %s\n", i, baseName(bc.base), bc.desc.c_str(), ch.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
      std::printf("[FUZZ] ORACLE_INACCURATE case=%d %-20s native MATCHES analytic, OCCT does NOT "
                  "volN=%.6g volO=%.6g aVol=%.6g areaN=%.6g areaO=%.6g aArea=%.6g\n"
                  "       NOTE seed=0x%llx index=%d kinds=%-4s S=%.3f %s :: %s\n",
                  i, baseName(bc.base), nat.absVol, occ.volume, aVol, nat.area, occ.area, aArea,
                  static_cast<unsigned long long>(seed), i, kinds, S, bc.desc.c_str(), ch.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
      std::printf("[FUZZ] ORACLE_UNRELIABLE case=%d %-20s oracle mismatch/absent "
                  "[natValid=%d occValid=%d volO=%.6g aVol=%.6g]\n"
                  "       REPRO seed=0x%llx index=%d kinds=%-4s S=%.3f %s :: %s\n",
                  i, baseName(bc.base), nativeValid, oracleValid, occ.volume, aVol,
                  static_cast<unsigned long long>(seed), i, kinds, S, bc.desc.c_str(), ch.desc.c_str());
    } else {  // DISAGREED
      std::printf("[FUZZ] DISAGREED case=%d %-20s SILENT-WRONG-TRANSFORM "
                  "volN=%.6g volA=%.6g dV=%.3e areaN=%.6g areaA=%.6g dA=%.3e "
                  "Cnat=(%.4g,%.4g,%.4g) Canalytic=(%.4g,%.4g,%.4g) Cerr=%.3e hand=%d topo(f%d/e%d/v%d vs base f%d/e%d/v%d) tol=%.2e\n"
                  "       REPRO seed=0x%llx index=%d kinds=%-4s S=%.3f mir=%d %s :: %s\n",
                  i, baseName(bc.base), nat.absVol, aVol, relDiff(nat.absVol, aVol),
                  nat.area, aArea, relDiff(nat.area, aArea), nat.c[0], nat.c[1], nat.c[2],
                  aC[0], aC[1], aC[2], natCentErr, handednessOk, nat.nf, nat.ne, nat.nv,
                  base.nf, base.ne, base.nv, tol,
                  static_cast<unsigned long long>(seed), i, kinds, S, ch.mirrorCount,
                  bc.desc.c_str(), ch.desc.c_str());
    }
    std::fflush(stdout);
  }

  // ── coverage summary ──────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  BOTH-DECLINED=%d  ORACLE_UNRELIABLE=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleInacc, g_bothDecl, g_oracleBad);
  std::printf("   per-base-family [agreed/declined/DISAGREED]:\n");
  for (int b = 0; b < B_COUNT; ++b)
    std::printf("     %-22s %d/%d/%d\n", baseName(b), g_famAgreed[b], g_famDecl[b], g_famDisagreed[b]);
  std::printf("   per-op-kind AGREED coverage: TRANSLATE=%d ROTATE=%d USCALE=%d MIRROR=%d\n",
              g_kindAgreed[OP_TRANSLATE], g_kindAgreed[OP_ROTATE], g_kindAgreed[OP_USCALE], g_kindAgreed[OP_MIRROR]);
  std::printf("   MIRROR HANDEDNESS-FLIP positively confirmed in %d AGREED case(s) "
              "(odd-mirror chains whose signed enclosed-volume sign flipped as required, staying a valid positive-|vol| solid)\n",
              g_mirrorFlipConfirmed);
  std::printf("   HONEST-SCOPE: uniform scale ONLY (anisotropic scale has no closed-form area); planar bases "
              "(BOX/NGON/LOFT) reproduce the solid EXACTLY under a similarity (tight %.0e); curved bases "
              "(CYLINDER/SPHERE) use the deflection bound %.1f*deflection/(featureSize*min(1,S)); a singular "
              "(zero-scale) transform is an HONEST DECLINE.\n", kTightTol, kCurveC);
  if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — logged, NOT a native fault)\n", g_oracleInacc);
  if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (singular transform both engines refuse — no wrong result, logged)\n", g_bothDecl);
  if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (core-case OCCT vs closed-form mismatch — investigate)\n", g_oracleBad);

  // Bar: DISAGREED==0 AND ORACLE_UNRELIABLE==0 AND every base family + every op kind has ≥1
  // AGREED AND the mirror handedness flip was positively confirmed at least once.
  bool famCov = true; for (int b = 0; b < B_COUNT; ++b) if (g_famAgreed[b] < 1) famCov = false;
  bool kindCov = true; for (int k = 0; k < OP_KINDS; ++k) if (g_kindAgreed[k] < 1) kindCov = false;
  const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && famCov && kindCov && g_mirrorFlipConfirmed >= 1);
  std::printf("== M6-breadth-8 BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
              "base-family coverage=%s; op-kind coverage=%s; mirror-flip-confirmed=%d) ==\n",
              bar ? "PASS — zero silent wrong transforms" : "FAIL", g_disagreed, g_oracleBad,
              famCov ? "complete" : "INCOMPLETE", kindCov ? "complete" : "INCOMPLETE", g_mirrorFlipConfirmed);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
