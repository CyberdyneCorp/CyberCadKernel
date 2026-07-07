// SPDX-License-Identifier: Apache-2.0
//
// native_boolean_fuzz.mm — MOAT M6 (the COMPLETENESS BAR), first slice: a
// DIFFERENTIAL-FUZZING harness for the native curved boolean (iOS simulator).
//
// This is INFRASTRUCTURE — a test harness, not a geometry capability. It turns the
// sibling curated-parity harness (native_ssi_curved_boolean_parity.mm, native-pass=18
// on a handful of HAND-PICKED fixtures) into a SEEDED BATCH of N random-but-VALID
// operand pairs, and asserts the discipline drop-OCCT ultimately needs: over N inputs,
// ZERO SILENT WRONG RESULTS.
//
// ── WHAT IT DOES (per the M6 task) ────────────────────────────────────────────────
//   (1) DETERMINISTICALLY generate random-but-VALID inputs for the curved booleans:
//       random cyl/sphere/cone operands with random-but-valid params drawn inside the
//       RECOGNISED native families (so the native path is EXERCISED, not just declined).
//       The RNG is a splitmix64-seeded xoshiro256** stream keyed ONLY by an explicit
//       FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
//   (2) Run EACH {pair, op} through BOTH the native path (nb::boolean_solid, the
//       OCCT-FREE curved/SSI assembler) AND the OCCT oracle (BRepAlgoAPI_{Fuse,Cut,
//       Common} on the SAME analytic geometry).
//   (3) Classify each trial into EXACTLY ONE of three buckets:
//         AGREED             — native non-null + watertight, vol&area within tol of OCCT.
//         HONESTLY-DECLINED  — native NULL (or non-watertight) → OCCT fallback ships,
//                              and the OCCT oracle is itself a valid closed solid.
//         DISAGREED          — native non-null + watertight but vol/area OUTSIDE tol
//                              (or OCCT empty/invalid while native claims a solid). This
//                              is a SILENT WRONG RESULT — the M6 failure this exists to
//                              catch.
//   (4) Print a coverage summary: seed, N cases, trials, agreed / honestly-declined /
//       DISAGREED, plus per-family and per-op breakdowns. Process exits 0 IFF
//       DISAGREED == 0. Any DISAGREE prints seed + case index + full param tuple +
//       vol/area deltas as a REPRODUCIBLE regression find.
//
// ── FAMILIES (recognised → native EXERCISED, valid-by-construction) ────────────────
//   sphere∩sphere lens (coaxial +Y, transversal)       — S5-c, Fuse/Cut/Common native
//   cone∩sphere (coaxial +Y, single-crossing)           — S5-f, Fuse/Cut/Common native
//   cone∩cyl (coaxial +Y, single crossing)              — S5-e, Common native
//   cyl∩cyl equal-R orthogonal (Steinmetz)              — S5-d, Common native
//   box∩cyl (box radially ⊇ cyl, cyl spans through, Z)  — analytic curved.h, F/C/O native
//   cyl∩cyl unequal-R orthogonal (through-drill)        — S5-a DECLINE PROBE (honest decl.)
// The drill family is a deliberate DECLINE probe: a valid input the classifier must bin
// as HONESTLY-DECLINED, exercising that branch. It is NOT counted as a native pass.
//
// The generator draws each family's params in a tight neighbourhood of the sibling's
// known-good native configs and computes DEPENDENT params so every input is valid by
// construction (transversal seam, single crossing, box⊇cyl, spans-through). A pre-boolean
// OPERAND SELF-CHECK (native-vs-OCCT vol/area/watertight agree) makes any downstream
// DISAGREE attributable to the boolean, not a mismatched input; a self-check miss is
// reported as OPERAND_MISMATCH (a generator bug), never as a native disagreement.
//
// The agreement tolerance is FIXED (relTol 2e-2, native-mesh vs OCCT-exact) and NEVER
// widened to hide a gap; the mesh is tessellated fine (deflection 0.001) so a CORRECT
// native result clears the bar with margin (see kDeflection). A FALLBACK_ORACLE_INVALID
// guard forbids laundering a broken oracle result into a "decline".
//
// This TU is OCCT-dependent AND substrate-dependent (identical slice to the sibling): it
// links the OCCT oracle + the NumPP/SciPP numsci archive, compiling
// src/native/boolean/ssi_boolean.cpp + ssi/{seeding,marching}.cpp + numerics/numerics.cpp
// + math/* under CYBERCAD_HAS_NUMSCI. Built ONLY by
// scripts/run-sim-native-boolean-fuzz.sh; on run-sim-suite.sh's SKIP list. src/native
// stays OCCT-FREE — this harness is additive test/sim code only. Flushes and std::_Exit
// (OCCT static teardown in the trimmed static build is not exit-clean — same rationale as
// the sibling).
//
#include "native/boolean/native_boolean.h"      // nb::boolean_solid / Op
#include "native/construct/native_construct.h"   // build_prism / build_revolution_profile
#include "native/tessellate/native_tessellate.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_boolean_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif
#if !defined(CYBERCAD_HAS_NUMSCI)
#error "native_boolean_fuzz requires -DCYBERCAD_HAS_NUMSCI (the S3 tracer the S5 path consumes)"
#endif

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax2.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shell.hxx>
#include <TopAbs_ShapeEnum.hxx>

namespace nb = cybercad::native::boolean;
namespace ncst = cybercad::native::construct;
namespace ntess = cybercad::native::tessellate;
namespace ntopo = cybercad::native::topology;
namespace nmath = cybercad::native::math;

using nmath::Point3;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream ───────────────────────
// Keyed ONLY by an explicit uint64 seed (argv/env). No clock, no rand(). Same seed →
// byte-identical batch (host probe proved FNV digest identical across runs).
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
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

enum Prim { CYL, SPHERE, CONE, BOX };
enum Family { F_SPH_SPH, F_CONE_SPH, F_CONE_CYL, F_STEINMETZ, F_BOX_CYL, F_DRILL, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_SPH_SPH:    return "sphere=sphere(lens)";
    case F_CONE_SPH:   return "cone=sphere(coax)";
    case F_CONE_CYL:   return "cone=cyl(coax)";
    case F_STEINMETZ:  return "cyl=cyl(steinmetz)";
    case F_BOX_CYL:    return "box x cyl(analytic)";
    case F_DRILL:      return "cyl!=cyl(drill)";
  }
  return "?";
}

// A generated case. primA/primB + per-prim args match the makeXxx / occtXxx signatures
// below directly, so one FuzzCase drives BOTH the native and the OCCT operand.
//   CYL:    p[0]=axis(0=X/1=Y/2=Z) p[1]=c0 p[2]=c1 p[3]=r p[4]=lo p[5]=hi
//   SPHERE: p[0]=r p[1]=cy
//   CONE:   p[0]=r0 p[1]=y0 p[2]=r1 p[3]=y1
//   BOX:    p[0]=x0 p[1]=y0 p[2]=x1 p[3]=y1 p[4]=dz   (z∈[0,dz])
struct FuzzCase {
  int family = 0;
  int primA = 0; double a[6] = {0};
  int primB = 0; double b[6] = {0};
  bool runFuse = false, runCut = false, runCommon = false;
  double relTol = 2e-2;
};

// Weighted family sampling: favour the reliably-native families; the drill family is a
// minority DECLINE probe.
int pickFamily(Rng& r) {
  const int w[F_COUNT] = {5, 4, 4, 4, 5, 2};  // sph, conesph, conecyl, stein, boxcyl, drill
  int tot = 0; for (int x : w) tot += x;
  int k = static_cast<int>(r.below(static_cast<uint32_t>(tot)));
  for (int i = 0; i < F_COUNT; ++i) { if (k < w[i]) return i; k -= w[i]; }
  return F_SPH_SPH;
}

FuzzCase genCase(Rng& r) {
  FuzzCase c; c.relTol = 2e-2; c.family = pickFamily(r);
  switch (c.family) {
    case F_SPH_SPH: {  // makeSphere(r,cy) ×2, coaxial +Y, strictly transversal lens
      const double rA = r.range(0.6, 1.6), rB = r.range(0.6, 1.6);
      const double dmin = std::fabs(rA - rB), dmax = rA + rB;
      const double t = r.range(0.30, 0.70);                 // strictly transversal seam
      const double d = dmin + t * (dmax - dmin);
      c.primA = SPHERE; c.a[0] = rA; c.a[1] = 0.0;
      c.primB = SPHERE; c.b[0] = rB; c.b[1] = d;
      c.runFuse = c.runCut = c.runCommon = true; break;     // S5-c: all three native
    }
    case F_CONE_SPH: {  // makeCone(r0,y0,r1,y1) × makeSphere(Rs,cy), coaxial +Y
      const double slope = r.range(0.45, 0.55), r0 = r.range(0.45, 0.65);
      const double y0 = 0.0, y1 = r.range(3.6, 4.4);
      const double r1 = r0 + slope * (y1 - y0);
      const double Rs = r.range(1.85, 2.15), cy = r.range(-0.15, 0.15);
      c.primA = CONE;   c.a[0] = r0; c.a[1] = y0; c.a[2] = r1; c.a[3] = y1;
      c.primB = SPHERE; c.b[0] = Rs; c.b[1] = cy;
      c.runFuse = c.runCut = c.runCommon = true; break;     // S5-f: all three native
    }
    case F_CONE_CYL: {  // makeCone × makeCyl(axis Y), coaxial single crossing
      const double slope = r.range(0.45, 0.55), r0 = r.range(0.45, 0.65);
      const double y0 = 0.0, y1 = r.range(3.6, 4.4);
      const double r1 = r0 + slope * (y1 - y0);
      const double Rc = r.range(1.2, 1.7);                  // r0 < Rc < r1 → one crossing
      const double ystar = (Rc - r0) / slope;               // crossing height ∈ [1.0, 2.78]
      const double cylLo = r.range(0.6, ystar - 0.2);       // seam strictly inside overlap
      const double cylHi = y1 + r.range(0.5, 1.5);
      c.primA = CONE; c.a[0] = r0; c.a[1] = y0; c.a[2] = r1; c.a[3] = y1;
      c.primB = CYL;  c.b[0] = 1; c.b[1] = 0; c.b[2] = 0; c.b[3] = Rc; c.b[4] = cylLo; c.b[5] = cylHi;
      c.runFuse = false; c.runCut = false; c.runCommon = true; break;  // S5-e: COMMON native
    }
    case F_STEINMETZ: {  // makeCyl equal radii, orthogonal axes through origin
      const double rad = r.range(0.6, 1.4), L = 2.5 * rad + r.range(0.5, 1.5);
      const int pair = static_cast<int>(r.below(3));        // (Z,X)/(Z,Y)/(X,Y)
      int axA = 2, axB = 0;
      if (pair == 1) { axA = 2; axB = 1; } else if (pair == 2) { axA = 0; axB = 1; }
      c.primA = CYL; c.a[0] = axA; c.a[1] = 0; c.a[2] = 0; c.a[3] = rad; c.a[4] = -L; c.a[5] = L;
      c.primB = CYL; c.b[0] = axB; c.b[1] = 0; c.b[2] = 0; c.b[3] = rad; c.b[4] = -L; c.b[5] = L;
      c.runFuse = false; c.runCut = false; c.runCommon = true; break;  // S5-d: COMMON native
    }
    case F_BOX_CYL: {  // analytic curved.h: box radially ⊇ cyl, cyl spans through (axis Z)
      const double rad = r.range(0.3, 1.0);
      const double halfPerp = rad + r.range(0.4, 1.2);      // radiallyInside margin
      const double axHalf = r.range(0.5, 1.5);
      const double dz = 2.0 * axHalf;                       // box z∈[0,dz]
      const double m0 = r.range(0.5, 1.5), m1 = r.range(0.5, 1.5);
      // Box centred on (0,0) in X/Y, z∈[0,dz]; Z cylinder at (0,0) spanning past both caps.
      c.primA = BOX; c.a[0] = -halfPerp; c.a[1] = -halfPerp; c.a[2] = halfPerp; c.a[3] = halfPerp; c.a[4] = dz;
      c.primB = CYL; c.b[0] = 2; c.b[1] = 0; c.b[2] = 0; c.b[3] = rad; c.b[4] = -m0; c.b[5] = dz + m1;
      c.runFuse = true; c.runCut = true; c.runCommon = true; break;  // fuse+common+box−cyl cut native
    }
    case F_DRILL: {  // through-drill: exercises the S5-a transversal path → honest DECLINE
      const double R = r.range(1.5, 2.5), rr = r.range(0.3, 0.7);   // rr < R
      const double L = 3.0;
      c.primA = CYL; c.a[0] = 2; c.a[1] = 0; c.a[2] = 0; c.a[3] = R;  c.a[4] = -L; c.a[5] = L;
      c.primB = CYL; c.b[0] = 0; c.b[1] = 0; c.b[2] = 0; c.b[3] = rr; c.b[4] = -L; c.b[5] = L;
      c.runFuse = c.runCut = c.runCommon = true; break;
    }
  }
  return c;
}

// ── native operand builders (OCCT-free; reused verbatim from the sibling harness) ──
ntopo::Shape makeCyl(int axis, double c0, double c1, double r, double lo, double hi) {
  nb::curved::AABox box{Point3{-1000, -1000, -1000}, Point3{1000, 1000, 1000}};
  return nb::curved::buildCommonSegment(box, nb::curved::AxisCylinder{axis, c0, c1, r, lo, hi});
}
ntopo::Shape makeBox(double x0, double y0, double x1, double y1, double dz) {
  const double rect[] = {x0, y0, x1, y0, x1, y1, x0, y1};
  return ncst::build_prism(rect, 4, dz);
}
ntopo::Shape makeSphere(double r, double cy) {
  ncst::ProfileSegment arc;
  arc.kind = 1;  // arc
  arc.cx = 0.0; arc.cy = cy; arc.r = r;
  arc.x0 = 0.0; arc.y0 = cy - r; arc.x1 = 0.0; arc.y1 = cy + r;
  arc.a0 = -kPi / 2.0; arc.a1 = kPi / 2.0;
  ncst::RevolveAxis ax;
  ax.ax = 0.0; ax.ay = 0.0; ax.adx = 0.0; ax.ady = 1.0;
  return ncst::build_revolution_profile({arc}, ax, 2.0 * kPi);
}
ntopo::Shape makeCone(double r0, double y0, double r1, double y1) {
  ncst::ProfileSegment side; side.kind = 0;
  side.x0 = r0; side.y0 = y0; side.x1 = r1; side.y1 = y1;
  ncst::ProfileSegment topEdge; topEdge.kind = 0;
  topEdge.x0 = r1; topEdge.y0 = y1; topEdge.x1 = 0.0; topEdge.y1 = y1;
  ncst::ProfileSegment axisEdge; axisEdge.kind = 0;
  axisEdge.x0 = 0.0; axisEdge.y0 = y1; axisEdge.x1 = 0.0; axisEdge.y1 = y0;
  ncst::ProfileSegment botEdge; botEdge.kind = 0;
  botEdge.x0 = 0.0; botEdge.y0 = y0; botEdge.x1 = r0; botEdge.y1 = y0;
  ncst::RevolveAxis ax; ax.ax = 0.0; ax.ay = 0.0; ax.adx = 0.0; ax.ady = 1.0;
  return ncst::build_revolution_profile({side, topEdge, axisEdge, botEdge}, ax, 2.0 * kPi);
}

ntopo::Shape makeNative(int prim, const double* p) {
  switch (prim) {
    case CYL:    return makeCyl(static_cast<int>(p[0]), p[1], p[2], p[3], p[4], p[5]);
    case SPHERE: return makeSphere(p[0], p[1]);
    case CONE:   return makeCone(p[0], p[1], p[2], p[3]);
    case BOX:    return makeBox(p[0], p[1], p[2], p[3], p[4]);
  }
  return {};
}

// ── OCCT operand builders (matched to the native geometry, verbatim from sibling) ──
TopoDS_Shape occtCyl(int axis, double c0, double c1, double r, double lo, double hi) {
  const double h = hi - lo;
  gp_Pnt base; gp_Dir dir;
  if (axis == 0)      { base = gp_Pnt(lo, c0, c1); dir = gp_Dir(1, 0, 0); }
  else if (axis == 1) { base = gp_Pnt(c0, lo, c1); dir = gp_Dir(0, 1, 0); }
  else                { base = gp_Pnt(c0, c1, lo); dir = gp_Dir(0, 0, 1); }
  return BRepPrimAPI_MakeCylinder(gp_Ax2(base, dir), r, h).Shape();
}
TopoDS_Shape occtBox(double x0, double y0, double x1, double y1, double dz) {
  return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, 0.0), gp_Pnt(x1, y1, dz)).Shape();
}
TopoDS_Shape occtSphere(double r, double cy) {
  return BRepPrimAPI_MakeSphere(gp_Pnt(0.0, cy, 0.0), r).Shape();
}
TopoDS_Shape occtCone(double r0, double y0, double r1, double y1) {
  return BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(0.0, y0, 0.0), gp_Dir(0, 1, 0)), r0, r1, y1 - y0).Shape();
}

TopoDS_Shape makeOcct(int prim, const double* p) {
  switch (prim) {
    case CYL:    return occtCyl(static_cast<int>(p[0]), p[1], p[2], p[3], p[4], p[5]);
    case SPHERE: return occtSphere(p[0], p[1]);
    case CONE:   return occtCone(p[0], p[1], p[2], p[3]);
    case BOX:    return occtBox(p[0], p[1], p[2], p[3], p[4]);
  }
  return {};
}

// ── measurement (identical machinery to the sibling) ───────────────────────────────
struct NativeMeasure { bool present = false, watertight = false; double volume = 0, area = 0; };
// Mesh deflection. FINER than the sibling's 0.005: measureNative tessellates the native
// solid while measureOcct uses OCCT's EXACT mass properties (BRepGProp). That asymmetry
// leaves an inscribed-facet volume bias that grows ∝ deflection and, for the SMALLEST
// generated curved features (sphere r≈0.6, box∩cyl r≈0.3), approaches the fixed 2e-2
// agreement bar at 0.005. 0.001 pushes a CORRECT native result's bias to ≈2–4e-3 —
// comfortably below 2e-2 — so the bar cleanly separates a correct result from a genuine
// wrong-set (which is off by a large margin regardless of mesh fineness). The 2e-2
// agreement tolerance itself is NEVER widened.
constexpr double kDeflection = 0.001;

NativeMeasure measureNative(const ntopo::Shape& s) {
  NativeMeasure m;
  if (s.isNull()) return m;
  m.present = true;
  ntess::MeshParams p; p.deflection = kDeflection;
  const ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(s);
  m.watertight = ntess::isWatertight(mesh);
  m.volume = std::fabs(ntess::enclosedVolume(mesh));
  m.area = ntess::surfaceArea(mesh);
  return m;
}
struct OcctMeasure { bool valid = false, closedShell = false; double volume = 0, area = 0; };
OcctMeasure measureOcct(const TopoDS_Shape& s) {
  OcctMeasure m;
  if (s.IsNull()) return m;
  BRepCheck_Analyzer an(s);
  m.valid = an.IsValid();
  bool anyShell = false, allClosed = true;
  for (TopExp_Explorer ex(s, TopAbs_SHELL); ex.More(); ex.Next()) {
    anyShell = true;
    const TopoDS_Shell sh = TopoDS::Shell(ex.Current());
    if (!BRep_Tool::IsClosed(sh)) allClosed = false;
  }
  m.closedShell = anyShell && allClosed;
  GProp_GProps vg; BRepGProp::VolumeProperties(s, vg); m.volume = std::fabs(vg.Mass());
  GProp_GProps ag; BRepGProp::SurfaceProperties(s, ag); m.area = ag.Mass();
  return m;
}
TopoDS_Shape occtBoolean(const TopoDS_Shape& a, const TopoDS_Shape& b, nb::Op op) {
  switch (op) {
    case nb::Op::Fuse:   { BRepAlgoAPI_Fuse   f(a, b); return f.IsDone() ? f.Shape() : TopoDS_Shape(); }
    case nb::Op::Cut:    { BRepAlgoAPI_Cut    c(a, b); return c.IsDone() ? c.Shape() : TopoDS_Shape(); }
    case nb::Op::Common: { BRepAlgoAPI_Common c(a, b); return c.IsDone() ? c.Shape() : TopoDS_Shape(); }
  }
  return {};
}
const char* opName(nb::Op op) {
  switch (op) { case nb::Op::Fuse: return "fuse"; case nb::Op::Cut: return "cut"; case nb::Op::Common: return "common"; }
  return "?";
}

// ── the three-way classifier ───────────────────────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, OPERAND_MISMATCH, FALLBACK_ORACLE_INVALID };

// Coverage counters.
int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_badOperand = 0, g_badFallback = 0;
int g_famAgreed[F_COUNT] = {0}, g_famDeclined[F_COUNT] = {0}, g_famDisagreed[F_COUNT] = {0};
struct OpCount { int agreed = 0, declined = 0, disagreed = 0; };
OpCount g_opCount[3];  // Fuse=0, Cut=1, Common=2

std::string tupleStr(const FuzzCase& c) {
  char buf[256];
  std::snprintf(buf, sizeof buf,
                "A(prim=%d)[%.4f %.4f %.4f %.4f %.4f %.4f] B(prim=%d)[%.4f %.4f %.4f %.4f %.4f %.4f]",
                c.primA, c.a[0], c.a[1], c.a[2], c.a[3], c.a[4], c.a[5],
                c.primB, c.b[0], c.b[1], c.b[2], c.b[3], c.b[4], c.b[5]);
  return buf;
}

// Pre-boolean operand self-check: native and OCCT operands must agree (so a downstream
// DISAGREE is attributable to the boolean, not a mismatched input).
struct OperandCheck { bool ok = false; bool present = false, watertight = false, valid = false;
                      double volN = 0, volO = 0, areaN = 0, areaO = 0, dV = 1e30, dA = 1e30; };
OperandCheck operandAgrees(const ntopo::Shape& nat, const TopoDS_Shape& occ, double relTol) {
  const NativeMeasure nm = measureNative(nat);
  const OcctMeasure om = measureOcct(occ);
  OperandCheck r;
  r.present = nm.present; r.watertight = nm.watertight; r.valid = om.valid;
  r.volN = nm.volume; r.volO = om.volume; r.areaN = nm.area; r.areaO = om.area;
  // Same-geometry gate: the native operand must exist with a positive enclosed volume
  // and the OCCT operand must be a valid closed solid, and their volume+area must agree.
  // The raw native operand mesh's watertight FLAG is deliberately NOT gated on: a full-
  // revolution frustum's raw mesh can trip the tessellator's watertight seam heuristic
  // (a raw-operand-mesh artifact) even though its enclosed volume matches OCCT to ~2e-3
  // and the boolean — which runs on the B-rep, not this mesh — yields a watertight
  // result. Volume+area agreement is the true same-solid signal; a genuine generator
  // mismatch is off by a large margin and still caught.
  if (!nm.present || !om.valid || !om.closedShell) return r;
  if (nm.volume <= 1e-9 || om.volume <= 1e-12 || om.area <= 1e-12) return r;
  r.dV = std::fabs(nm.volume - om.volume) / om.volume;
  r.dA = std::fabs(nm.area - om.area) / om.area;
  r.ok = r.dV < relTol && r.dA < relTol;
  return r;
}

}  // namespace

int main(int argc, char** argv) {
  // Seed + N from argv (argv[1]=seed, argv[2]=N) or env (FUZZ_SEED / FUZZ_N). Explicit
  // only — NO clock. Defaults are fixed constants so a bare run is still deterministic.
  uint64_t seed = 0xC0FFEE1234ull;
  int N = 96;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 96;

  std::printf("== M6 differential-fuzz: native curved boolean vs OCCT oracle ==\n");
  std::printf("== seed=0x%llx N=%d relTol=2e-2 deflection=0.001 ==\n",
              static_cast<unsigned long long>(seed), N);
  std::fflush(stdout);

  Rng rng(seed);
  int trials = 0;

  for (int i = 0; i < N; ++i) {
    const FuzzCase c = genCase(rng);

    // Build both operand sets ONCE per case.
    const ntopo::Shape natA = makeNative(c.primA, c.a);
    const ntopo::Shape natB = makeNative(c.primB, c.b);
    const TopoDS_Shape occA = makeOcct(c.primA, c.a);
    const TopoDS_Shape occB = makeOcct(c.primB, c.b);

    // Operand self-check — a mismatch is a GENERATOR bug, never a native disagreement.
    const OperandCheck okA = operandAgrees(natA, occA, c.relTol);
    const OperandCheck okB = operandAgrees(natB, occB, c.relTol);
    if (!okA.ok || !okB.ok) {
      ++g_badOperand;
      std::printf("[FUZZ] OPERAND_MISMATCH case=%d fam=%s\n"
                  "       A ok=%d pres=%d wt=%d valid=%d volN=%.5g volO=%.5g dV=%.2e areaN=%.5g areaO=%.5g dA=%.2e\n"
                  "       B ok=%d pres=%d wt=%d valid=%d volN=%.5g volO=%.5g dV=%.2e areaN=%.5g areaO=%.5g dA=%.2e\n"
                  "       %s\n",
                  i, famName(c.family),
                  okA.ok, okA.present, okA.watertight, okA.valid, okA.volN, okA.volO, okA.dV, okA.areaN, okA.areaO, okA.dA,
                  okB.ok, okB.present, okB.watertight, okB.valid, okB.volN, okB.volO, okB.dV, okB.areaN, okB.areaO, okB.dA,
                  tupleStr(c).c_str());
      std::fflush(stdout);
      continue;
    }

    for (nb::Op op : {nb::Op::Fuse, nb::Op::Cut, nb::Op::Common}) {
      if (op == nb::Op::Fuse   && !c.runFuse)   continue;
      if (op == nb::Op::Cut    && !c.runCut)    continue;
      if (op == nb::Op::Common && !c.runCommon) continue;
      ++trials;
      const int opIdx = static_cast<int>(op);

      const ntopo::Shape nativeRes = nb::boolean_solid(natA, natB, op);
      const NativeMeasure nm = measureNative(nativeRes);
      const TopoDS_Shape occtRes = occtBoolean(occA, occB, op);
      const OcctMeasure om = measureOcct(occtRes);

      const double volRel = (nm.present && om.volume > 1e-12)
                                ? std::fabs(nm.volume - om.volume) / om.volume : 1e30;
      const double areaRel = (nm.present && om.area > 1e-12)
                                 ? std::fabs(nm.area - om.area) / om.area : 1e30;

      Verdict v;
      if (nm.present && nm.watertight) {
        // Native SHIPPED a watertight solid — it MUST match OCCT, else silent-wrong.
        const bool oracleSolid = om.valid && om.volume > 1e-9 && om.area > 1e-9;
        v = (oracleSolid && volRel < c.relTol && areaRel < c.relTol) ? AGREED : DISAGREED;
      } else {
        // Native NULL or non-watertight → OCCT fallback ships. It may NOT launder a
        // broken oracle into a "decline": the shipped OCCT result must be a valid solid.
        const bool shippedOk = om.valid && om.closedShell && om.volume > 1e-9 && om.area > 1e-9;
        v = shippedOk ? DECLINED : FALLBACK_ORACLE_INVALID;
      }

      const char* tag = "?";
      switch (v) {
        case AGREED:   ++g_agreed;   ++g_famAgreed[c.family];   ++g_opCount[opIdx].agreed;   tag = "AGREED";   break;
        case DECLINED: ++g_declined; ++g_famDeclined[c.family]; ++g_opCount[opIdx].declined; tag = "DECLINED"; break;
        case DISAGREED:++g_disagreed;++g_famDisagreed[c.family];++g_opCount[opIdx].disagreed;tag = "DISAGREED";break;
        case FALLBACK_ORACLE_INVALID: ++g_badFallback; tag = "FALLBACK_ORACLE_INVALID"; break;
        case OPERAND_MISMATCH: break;  // handled above
      }

      if (v == AGREED) {
        std::printf("[FUZZ] AGREED    case=%d %-20s %-6s volN=%.5g volO=%.5g dV=%.2e areaN=%.5g areaO=%.5g dA=%.2e\n",
                    i, famName(c.family), opName(op), nm.volume, om.volume, volRel, nm.area, om.area, areaRel);
      } else if (v == DECLINED) {
        std::printf("[FUZZ] DECLINED  case=%d %-20s %-6s native=%s -> OCCT[valid=%d closed=%d volO=%.5g areaO=%.5g]\n",
                    i, famName(c.family), opName(op), nm.present ? "non-watertight" : "NULL",
                    om.valid ? 1 : 0, om.closedShell ? 1 : 0, om.volume, om.area);
      } else if (v == DISAGREED) {
        // The M6 failure — a reproducible regression find.
        std::printf("[FUZZ] DISAGREED case=%d %-20s %-6s SILENT-WRONG-RESULT "
                    "volN=%.6g volO=%.6g dV=%.3e areaN=%.6g areaO=%.6g dA=%.3e wt=%d oracleValid=%d\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(c.family), opName(op), nm.volume, om.volume, volRel,
                    nm.area, om.area, areaRel, nm.watertight ? 1 : 0, om.valid ? 1 : 0,
                    static_cast<unsigned long long>(seed), i, tupleStr(c).c_str());
      } else {  // FALLBACK_ORACLE_INVALID
        std::printf("[FUZZ] FALLBACK_ORACLE_INVALID case=%d %-20s %-6s native declined but OCCT oracle "
                    "invalid/open [valid=%d closed=%d volO=%.5g areaO=%.5g]\n"
                    "       REPRO seed=0x%llx index=%d %s\n",
                    i, famName(c.family), opName(op), om.valid ? 1 : 0, om.closedShell ? 1 : 0,
                    om.volume, om.area, static_cast<unsigned long long>(seed), i, tupleStr(c).c_str());
      }
      std::fflush(stdout);
    }
  }

  // ── coverage summary ────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   trials=%d  AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d\n",
              trials, g_agreed, g_declined, g_disagreed);
  std::printf("   per-family [agreed/declined/DISAGREED]:\n");
  for (int f = 0; f < F_COUNT; ++f) {
    std::printf("     %-20s %d/%d/%d\n", famName(f), g_famAgreed[f], g_famDeclined[f], g_famDisagreed[f]);
  }
  std::printf("   per-op [agreed/declined/DISAGREED]:  fuse %d/%d/%d   cut %d/%d/%d   common %d/%d/%d\n",
              g_opCount[0].agreed, g_opCount[0].declined, g_opCount[0].disagreed,
              g_opCount[1].agreed, g_opCount[1].declined, g_opCount[1].disagreed,
              g_opCount[2].agreed, g_opCount[2].declined, g_opCount[2].disagreed);
  if (g_badOperand)  std::printf("   OPERAND_MISMATCH=%d (generator bug — investigate)\n", g_badOperand);
  if (g_badFallback) std::printf("   FALLBACK_ORACLE_INVALID=%d (broken oracle laundered as decline — investigate)\n", g_badFallback);

  const bool bar = (g_disagreed == 0 && g_badOperand == 0 && g_badFallback == 0);
  std::printf("== M6 BAR: %s (DISAGREED=%d must be 0) ==\n",
              bar ? "PASS — zero silent wrong results" : "FAIL", g_disagreed);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
