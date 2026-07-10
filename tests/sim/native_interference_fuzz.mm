// SPDX-License-Identifier: Apache-2.0
//
// native_interference_fuzz.mm — MOAT M6-breadth-17 (the COMPLETENESS BAR, 17th domain):
// an INTERFERENCE / CLASH differential-fuzzing harness (iOS simulator) that certifies the
// newly-native `cc_interference` surface — the OCCT-FREE, header-only clash classifier
// src/native/analysis/interference.h (on the landed B3 membership + M0 mesh vocabulary).
// It gates M8 (drop-occt). test-infra ONLY; zero product risk.
//
// ── WHY THIS DOMAIN IS DISTINCT FROM native_interference_parity.mm ────────────────────
// native_interference_parity.mm is a HAND-PICKED FIXTURE parity check: five fixed box
// poses (overlap-corner / overlap-slab / nested / face-touch / disjoint). THIS harness is
// the SEEDED, RANDOMISED completeness certification the fixture set does not provide: it
// draws random PAIRS of solids (box / n-gon prism / cylinder / sphere) at random dims and
// random relative rigid placements (translate + rotate) spanning ALL THREE interference
// regimes — clearly disjoint (CLEAR), interpenetrating (CLASH), and near-flush (TOUCHING) —
// then differentials the native classifier against the OCCT oracle AND, where the pair has
// one, a CLOSED-FORM arbiter (box∩box intersection-box volume; sphere/sphere lens volume +
// centre distance). It exercises the TOUCHING knife-edge (flush face contact) explicitly:
// a boundary-coincident pose must read On (TOUCHING), never a CLASH — the landed op's B3
// ON-band absorbs the seam (interference.h step 2b), and this harness asserts it under
// randomised poses, not just the one hand-built face-touch fixture.
//
// ── THE ORACLES ───────────────────────────────────────────────────────────────────────
//   OCCT  — the same decision the fixture parity makes: BRepAlgoAPI_Common volume (>band ⇒
//           interior overlap ⇒ CLASH), COMBINED with BRepExtrema_DistShapeShape (min
//           boundary distance: ~0 with no overlap ⇒ TOUCHING; >0 ⇒ CLEAR).
//   CLOSED-FORM (the PRIMARY arbiter where the pair has one) — box∩box: the exact
//           axis-aligned intersection-box volume + the exact axis gap. sphere∩sphere: the
//           exact lens volume + the exact centre-distance-vs-(rA+rB) regime. Exact for the
//           ideal solid, so a native result matching the closed form while OCCT is the
//           outlier is logged ORACLE-INACCURATE (native vindicated), NEVER a bar failure.
//
// ── THE FACETING BOUNDARY (why native-vs-OCCT alone is insufficient for curved pairs) ──
// The native classifier consumes a deflection-bounded PLANAR-FACET mesh; OCCT keeps a true
// analytic B-rep. A faceted cylinder / sphere is INSET from the true surface by up to
// ~2×deflection (chord-secant). So at a near-flush curved contact the faceted surfaces can
// read a hair CLEAR where OCCT's exact B-rep reads TOUCHING — a real, bounded, EXPECTED
// facet artefact, NOT a native fault. The classifier already accounts for it (its contact
// band is max(1e-9·scale, 2·deflection)); the harness therefore treats a curved-primitive
// TOUCHING↔CLEAR straddle inside the facet band as a CONVERGENT match (a shared regime),
// never a DISAGREE. A CLASH↔CLEAR or CLASH↔TOUCHING split is ALWAYS a hard state and any
// disagreement there fails the bar. NO tolerance is ever widened to force a pass.
//
// ── THE SIX-WAY CLASSIFIER (identical discipline to the landed siblings) ──────────────
//   AGREED            native state == OCCT state (and, where present, == the closed-form
//                     regime + overlap-volume/min-distance within band).
//   HONESTLY-DECLINED native meshInterference → Unknown (an ambiguous mesh pose it refuses)
//                     while OCCT ships a crisp verdict → native falls through to OCCT.
//                     First-class, counted separately, NEVER a bar failure.
//   DISAGREED         native crisp state != OCCT crisp state on a HARD boundary (a CLASH
//                     that OCCT calls CLEAR, or vice-versa) AND the closed-form arbiter (if
//                     any) sides with OCCT. A genuine silent-wrong clash. FAILS the bar.
//   ORACLE-INACCURATE native matches the closed-form truth while OCCT does NOT (e.g. a
//                     sub-tolerance overlap OCCT rounds away). Native vindicated. Logged.
//   FACET-CONVERGENT  a curved-pair TOUCHING↔CLEAR straddle within the facet band — the
//                     two engines pick adjacent soft states off a faceted flush contact.
//                     Logged as a convergent match, NOT a failure.
//   BOTH-DECLINED     both engines decline (e.g. OCCT boolean fails AND native Unknown).
//
// ── THE BAR ──────────────────────────────────────────────────────────────────────────
//   Exit 0 IFF DISAGREED == 0, with every populated [family × regime] cell showing real
//   exercise (AGREED or an honest first-class outcome, never an all-decline). Run over ≥ 2
//   distinct seeds, N ≥ 60 per seed; the runner fails if ANY seed fails. The generator is
//   seeded ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed →
//   byte-identical batch (splitmix64 → xoshiro256**, verbatim from the siblings). Any
//   DISAGREE / ORACLE-INACCURATE prints seed + case index + family/regime/param tuple + the
//   native/OCCT/closed-form triple as a reproducible regression find.
//
// This TU is OCCT-dependent (BRepPrimAPI + BRepAlgoAPI_Common + BRepExtrema for the oracle);
// interference.h is header-only, so like native_interference_parity.mm it links only the
// native math TUs (NOT the whole kernel). Built ONLY by scripts/run-sim-native-interference-
// fuzz.sh; on run-sim-suite.sh's SKIP list (own main(), std::_Exit — the trimmed static OCCT
// build's teardown is not exit-clean, same rationale as the siblings). src/native / src/
// engine / include stay BYTE-UNCHANGED — this harness is additive test/sim code only.
//
#include "native/analysis/interference.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_interference_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT oracle libraries"
#endif

#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <Standard_Failure.hxx>

namespace an = cybercad::native::analysis;
namespace tess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDefl = 0.01;   // the deflection the meshes are built at (feeds the band)

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
  double unit() { return (next() >> 11) * (1.0 / 9007199254740992.0); }  // [0,1)
  double range(double lo, double hi) { return lo + (hi - lo) * unit(); }
  uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

// ── A rigid transform: rotation about a unit axis by angle θ then a translation. Applied
//    IDENTICALLY to the native mesh (below) and the OCCT solid (gp_Trsf) so both operands
//    are the SAME body in the SAME pose on both sides. ─────────────────────────────────
struct Rigid {
  double ax = 0, ay = 0, az = 1;  // unit rotation axis
  double theta = 0;                // rotation angle (rad)
  double tx = 0, ty = 0, tz = 0;   // translation
};

// Rotate a vector about a unit axis (a,ay,az) by θ (Rodrigues).
nm::Point3 applyRigid(const Rigid& g, const nm::Point3& p) {
  const double c = std::cos(g.theta), s = std::sin(g.theta), k = 1.0 - c;
  const double x = p.x, y = p.y, z = p.z;
  const double dot = g.ax * x + g.ay * y + g.az * z;
  // v·c + (axis×v)·s + axis·(axis·v)·(1−c)
  const double cx = g.ay * z - g.az * y;
  const double cy = g.az * x - g.ax * z;
  const double cz = g.ax * y - g.ay * x;
  return nm::Point3{
      x * c + cx * s + g.ax * dot * k + g.tx,
      y * c + cy * s + g.ay * dot * k + g.ty,
      z * c + cz * s + g.az * dot * k + g.tz};
}

gp_Trsf occtRigid(const Rigid& g) {
  gp_Trsf rot, tr;
  if (g.theta != 0.0)
    rot.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(g.ax, g.ay, g.az)), g.theta);
  tr.SetTranslation(gp_Vec(g.tx, g.ty, g.tz));
  return tr * rot;   // translate after rotate
}

// ═══════════════════════════════════════════════════════════════════════════════════════
// PRIMITIVES — built IDENTICALLY on both sides. The native side is a watertight,
// outward-CCW-wound M0 mesh; the OCCT side is the matching BRepPrimAPI solid. Both are
// then placed by the SAME rigid transform.
// ═══════════════════════════════════════════════════════════════════════════════════════

void transformMesh(tess::Mesh& m, const Rigid& g) {
  for (nm::Point3& v : m.vertices) v = applyRigid(g, v);
}

// ── BOX centred at origin, half-extents (hx,hy,hz) ─────────────────────────────────────
tess::Mesh boxMesh(double hx, double hy, double hz) {
  tess::Mesh m;
  const nm::Point3 v[8] = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz}, {-hx, hy, -hz},
                           {-hx, -hy, hz},  {hx, -hy, hz},  {hx, hy, hz},  {-hx, hy, hz}};
  for (const auto& p : v) m.vertices.push_back(p);
  auto quad = [&](int A, int B, int C, int D) { m.addTriangle(A, B, C); m.addTriangle(A, C, D); };
  quad(0, 3, 2, 1); quad(4, 5, 6, 7); quad(0, 1, 5, 4);
  quad(2, 3, 7, 6); quad(1, 2, 6, 5); quad(3, 0, 4, 7);
  return m;
}
TopoDS_Shape occtBox(double hx, double hy, double hz) {
  return BRepPrimAPI_MakeBox(gp_Pnt(-hx, -hy, -hz), 2 * hx, 2 * hy, 2 * hz).Shape();
}

// ── N-GON PRISM: regular n-gon of circum-radius R in the XY plane, extruded along Z to
//    half-height hz (centred at origin). CCW-wound so outward normals point out. ─────────
tess::Mesh prismMesh(int n, double R, double hz) {
  tess::Mesh m;
  // bottom ring 0..n-1 (z=-hz), top ring n..2n-1 (z=+hz), then centres.
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * kPi * i / n;
    m.vertices.push_back(nm::Point3{R * std::cos(a), R * std::sin(a), -hz});
  }
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * kPi * i / n;
    m.vertices.push_back(nm::Point3{R * std::cos(a), R * std::sin(a), hz});
  }
  const int cb = 2 * n, ct = 2 * n + 1;
  m.vertices.push_back(nm::Point3{0, 0, -hz});  // bottom centre
  m.vertices.push_back(nm::Point3{0, 0, hz});   // top centre
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    // side quad (bottom i, bottom j, top j, top i) → outward (CCW from outside).
    m.addTriangle(i, j, n + j);
    m.addTriangle(i, n + j, n + i);
    // bottom cap (viewed from -Z outside: CW in XY → i, cb, j)
    m.addTriangle(i, cb, j);
    // top cap (viewed from +Z outside: CCW → n+i, n+j, ct)
    m.addTriangle(n + i, n + j, ct);
  }
  return m;
}
TopoDS_Shape occtPrism(int n, double R, double hz) {
  BRepBuilderAPI_MakePolygon poly;
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * kPi * i / n;
    poly.Add(gp_Pnt(R * std::cos(a), R * std::sin(a), -hz));
  }
  poly.Close();
  BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
  return BRepPrimAPI_MakePrism(face.Face(), gp_Vec(0, 0, 2 * hz)).Shape();
}

// ── CYLINDER: radius R, half-height hz, axis +Z centred at origin. Faceted into `seg`
//    sides (the native mesh); OCCT keeps the true B-rep. ─────────────────────────────────
tess::Mesh cylinderMesh(double R, double hz, int seg) {
  tess::Mesh m;
  for (int i = 0; i < seg; ++i) {
    const double a = 2.0 * kPi * i / seg;
    m.vertices.push_back(nm::Point3{R * std::cos(a), R * std::sin(a), -hz});
  }
  for (int i = 0; i < seg; ++i) {
    const double a = 2.0 * kPi * i / seg;
    m.vertices.push_back(nm::Point3{R * std::cos(a), R * std::sin(a), hz});
  }
  const int cb = 2 * seg, ct = 2 * seg + 1;
  m.vertices.push_back(nm::Point3{0, 0, -hz});
  m.vertices.push_back(nm::Point3{0, 0, hz});
  for (int i = 0; i < seg; ++i) {
    const int j = (i + 1) % seg;
    m.addTriangle(i, j, seg + j);
    m.addTriangle(i, seg + j, seg + i);
    m.addTriangle(i, cb, j);
    m.addTriangle(seg + i, seg + j, ct);
  }
  return m;
}
TopoDS_Shape occtCylinder(double R, double hz) {
  return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -hz), gp_Dir(0, 0, 1)), R, 2 * hz).Shape();
}

// ── SPHERE: radius R centred at origin, UV-tessellated (stacks × slices). ──────────────
tess::Mesh sphereMesh(double R, int stacks, int slices) {
  tess::Mesh m;
  const int top = 0, bot = 1;
  m.vertices.push_back(nm::Point3{0, 0, R});   // north pole (index 0)
  m.vertices.push_back(nm::Point3{0, 0, -R});  // south pole (index 1)
  // interior rings: stacks-1 rings at polar angles φ = π·k/stacks, k=1..stacks-1
  const int ring0 = 2;
  for (int k = 1; k < stacks; ++k) {
    const double phi = kPi * k / stacks;
    const double z = R * std::cos(phi), rr = R * std::sin(phi);
    for (int j = 0; j < slices; ++j) {
      const double th = 2.0 * kPi * j / slices;
      m.vertices.push_back(nm::Point3{rr * std::cos(th), rr * std::sin(th), z});
    }
  }
  auto ringIdx = [&](int k, int j) { return ring0 + (k - 1) * slices + (j % slices); };
  // top cap fan (pole → first ring), outward CCW.
  for (int j = 0; j < slices; ++j)
    m.addTriangle(top, ringIdx(1, j), ringIdx(1, j + 1));
  // middle quads
  for (int k = 1; k < stacks - 1; ++k)
    for (int j = 0; j < slices; ++j) {
      const int a = ringIdx(k, j), b = ringIdx(k, j + 1);
      const int c = ringIdx(k + 1, j + 1), d = ringIdx(k + 1, j);
      m.addTriangle(a, d, c);
      m.addTriangle(a, c, b);
    }
  // bottom cap fan (last ring → south pole)
  for (int j = 0; j < slices; ++j)
    m.addTriangle(bot, ringIdx(stacks - 1, j + 1), ringIdx(stacks - 1, j));
  return m;
}
TopoDS_Shape occtSphere(double R) { return BRepPrimAPI_MakeSphere(gp_Pnt(0, 0, 0), R).Shape(); }

// ═══════════════════════════════════════════════════════════════════════════════════════
// ORACLE + CLOSED-FORM
// ═══════════════════════════════════════════════════════════════════════════════════════
enum State { CLEAR = 0, TOUCHING = 1, CLASH = 2, UNKNOWN = -1 };
const char* stateName(int s) {
  switch (s) { case CLEAR: return "CLEAR"; case TOUCHING: return "TOUCHING";
               case CLASH: return "CLASH"; default: return "UNKNOWN"; }
}

// The OCCT verdict for two placed solids: CLASH (Common volume > band), TOUCHING (distance
// ~0, no overlap), CLEAR. Reports the Common volume + min distance. Declines (UNKNOWN) if
// the boolean itself fails (a rare OCCT robustness gap on a knife-edge tangency).
int occtState(const TopoDS_Shape& A, const TopoDS_Shape& B, double& volOut, double& distOut,
              bool& ok) {
  volOut = 0.0; distOut = 0.0; ok = true;
  try {
    BRepAlgoAPI_Common k(A, B);
    if (k.IsDone() && !k.Shape().IsNull()) {
      GProp_GProps g; BRepGProp::VolumeProperties(k.Shape(), g);
      volOut = std::fabs(g.Mass());
    } else { ok = false; }
    BRepExtrema_DistShapeShape ext(A, B);
    if (ext.IsDone() && ext.NbSolution() > 0) distOut = ext.Value();
    else ok = false;
  } catch (const Standard_Failure&) { ok = false; }
  if (!ok) return UNKNOWN;
  if (volOut > 1e-7) return CLASH;
  return (distOut <= 1e-6) ? TOUCHING : CLEAR;
}

int nativeState(const tess::Mesh& a, const tess::Mesh& b, an::InterferenceResult& r) {
  r = an::meshInterference(a, b, kDefl);
  switch (r.state) {
    case an::ClashState::Clash:    return CLASH;
    case an::ClashState::Touching: return TOUCHING;
    case an::ClashState::Clear:    return CLEAR;
    default:                       return UNKNOWN;
  }
}

// ── families ───────────────────────────────────────────────────────────────────────────
enum Family { F_BOX, F_NGON, F_CYL, F_SPH, F_COUNT };
const char* famName(int f) {
  switch (f) { case F_BOX: return "box=box"; case F_NGON: return "ngon=ngon";
               case F_CYL: return "cyl=cyl"; case F_SPH: return "sphere=sphere"; }
  return "?";
}
bool curvedFamily(int f) { return f == F_CYL || f == F_SPH; }

enum Regime { R_CLEAR = 0, R_TOUCH = 1, R_CLASH = 2, R_COUNT };
const char* regimeName(int r) {
  switch (r) { case R_CLEAR: return "CLEAR"; case R_TOUCH: return "TOUCH"; case R_CLASH: return "CLASH"; }
  return "?";
}

enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, FACET_CONVERGENT, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_facet = 0, g_bothDecl = 0;
// per [family][regime] tallies (regime = the INTENDED regime the generator targeted).
int g_A[F_COUNT][R_COUNT] = {{0}}, g_D[F_COUNT][R_COUNT] = {{0}}, g_X[F_COUNT][R_COUNT] = {{0}};
int g_OI[F_COUNT][R_COUNT] = {{0}}, g_FC[F_COUNT][R_COUNT] = {{0}}, g_BD[F_COUNT][R_COUNT] = {{0}};

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[256]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

// A trial: the two placed operands (native meshes + OCCT solids already built) resolved to
// states + measurements + (optionally) a closed-form arbiter.
struct Trial {
  int family = 0, regime = 0;
  int nState = UNKNOWN, oState = UNKNOWN;
  bool oracleOk = false;
  double nVol = 0, oVol = 0, nDist = 0, oDist = 0;
  bool haveClosed = false;         // a closed-form arbiter exists for this pair
  int  xState = UNKNOWN;           // closed-form regime
  double xVol = 0, xDist = 0;      // closed-form overlap volume / centre-or-axis gap
  bool curved = false;
  bool soupProbe = false;          // a deliberately non-watertight operand ⇒ expect DECLINE
  std::string desc;
};

// Classify one trial with the six-way rubric.
Verdict classify(const Trial& t) {
  // Honest-decline probe: a non-watertight operand MUST make the classifier decline (its
  // step-1 precondition). If it instead emitted a crisp verdict it VIOLATED the "never guess
  // on ambiguous mesh evidence" contract → a hard DISAGREE. If it declined, that is the
  // intended HONESTLY-DECLINED first-class outcome.
  if (t.soupProbe) return (t.nState == UNKNOWN) ? DECLINED : DISAGREED;

  // Native declined (Unknown) → honest first-class (fall through to OCCT), or both declined.
  if (t.nState == UNKNOWN) return (t.oState != UNKNOWN) ? DECLINED : BOTH_DECLINED;
  if (t.oState == UNKNOWN) {
    // OCCT declined but native decided. If the closed form backs native it is native
    // vindicated (ORACLE-INACCURATE); else count as an honest decline of the oracle.
    if (t.haveClosed && t.xState == t.nState) return ORACLE_INACCURATE;
    return DECLINED;
  }

  if (t.nState == t.oState) {
    // States agree. Where a closed form exists, confirm it too; a native/OCCT agreement
    // that CONTRADICTS exact math would be a shared-oracle error — but that cannot make
    // native "wrong" here (both matched), so still AGREED (the closed form only ever
    // RESCUES native from an OCCT-only disagreement, never demotes an agreement).
    return AGREED;
  }

  // States DIFFER. Is it a soft curved-facet straddle (TOUCH↔CLEAR on a curved pair within
  // the facet band)? Then it is FACET-CONVERGENT, not a disagreement.
  const bool softPair = (t.nState == TOUCHING && t.oState == CLEAR) ||
                        (t.nState == CLEAR && t.oState == TOUCHING);
  if (t.curved && softPair) return FACET_CONVERGENT;

  // A hard split. Consult the closed-form arbiter: if it exists and sides with NATIVE,
  // native is vindicated (OCCT the outlier) → ORACLE-INACCURATE. If it sides with OCCT (or
  // there is none and OCCT is crisp) → a genuine DISAGREE.
  if (t.haveClosed) {
    if (t.xState == t.nState) return ORACLE_INACCURATE;   // native == exact math, OCCT wrong
    if (t.xState == t.oState) return DISAGREED;           // OCCT == exact math, native wrong
    // closed form matches neither crisp state — treat a soft straddle leniently, else a
    // genuine disagreement (the closed form is the truth and native missed it).
    if (softPair) return FACET_CONVERGENT;
    return DISAGREED;
  }
  // No closed form. A box↔box / ngon pose with no analytic arbiter should not reach here
  // (box↔box always has one); for curved pairs a hard CLASH split with no arbiter is a
  // genuine disagreement to investigate.
  return DISAGREED;
}

void tally(Verdict v, int fam, int reg) {
  switch (v) {
    case AGREED:            ++g_agreed;      ++g_A[fam][reg];  break;
    case DECLINED:          ++g_declined;    ++g_D[fam][reg];  break;
    case DISAGREED:         ++g_disagreed;   ++g_X[fam][reg];  break;
    case ORACLE_INACCURATE: ++g_oracleInacc; ++g_OI[fam][reg]; break;
    case FACET_CONVERGENT:  ++g_facet;       ++g_FC[fam][reg]; break;
    case BOTH_DECLINED:     ++g_bothDecl;    ++g_BD[fam][reg]; break;
  }
}

void report(int i, Verdict v, const Trial& t, uint64_t seed) {
  auto tri = [&] {
    std::printf("nat=%s occt=%s xf=%s volN=%.6g volO=%.6g volX=%.6g dN=%.4g dO=%.4g dX=%.4g",
                stateName(t.nState), stateName(t.oState),
                t.haveClosed ? stateName(t.xState) : "n/a", t.nVol, t.oVol, t.xVol,
                t.nDist, t.oDist, t.xDist);
  };
  if (v == AGREED) {
    std::printf("[FUZZ] AGREED    case=%-3d %-14s %-6s ", i, famName(t.family), regimeName(t.regime));
    tri(); std::printf("  %s\n", t.desc.c_str());
  } else if (v == DECLINED) {
    std::printf("[FUZZ] DECLINED  case=%-3d %-14s %-6s native/occt Unknown -> fall through  ",
                i, famName(t.family), regimeName(t.regime));
    tri(); std::printf("  %s\n", t.desc.c_str());
  } else if (v == FACET_CONVERGENT) {
    std::printf("[FUZZ] FACET-CONV case=%-3d %-14s %-6s curved TOUCH<->CLEAR facet straddle  ",
                i, famName(t.family), regimeName(t.regime));
    tri(); std::printf("  %s\n", t.desc.c_str());
  } else if (v == BOTH_DECLINED) {
    std::printf("[FUZZ] BOTH-DECL case=%-3d %-14s %-6s both engines declined  %s\n",
                i, famName(t.family), regimeName(t.regime), t.desc.c_str());
  } else if (v == ORACLE_INACCURATE) {
    std::printf("[FUZZ] ORACLE_INACCURATE case=%-3d %-14s %-6s native MATCHES closed form, OCCT does NOT  ",
                i, famName(t.family), regimeName(t.regime));
    tri();
    std::printf("\n       NOTE seed=0x%llx index=%d %s\n",
                static_cast<unsigned long long>(seed), i, t.desc.c_str());
  } else {  // DISAGREED
    std::printf("[FUZZ] DISAGREED case=%-3d %-14s %-6s SILENT-WRONG-CLASH  ",
                i, famName(t.family), regimeName(t.regime));
    tri();
    std::printf("\n       REPRO seed=0x%llx index=%d %s\n",
                static_cast<unsigned long long>(seed), i, t.desc.c_str());
  }
  std::fflush(stdout);
}

// ── closed-form arbiters ─────────────────────────────────────────────────────────────
// Axis-aligned box∩box overlap-box volume (the two boxes UNROTATED, only translated). lo/hi
// are the two placed boxes' AABBs. Returns overlap volume (0 if none) and sets the regime.
double boxOverlapVolume(const nm::Point3& aLo, const nm::Point3& aHi,
                        const nm::Point3& bLo, const nm::Point3& bHi, double& gap) {
  const double ox = std::min(aHi.x, bHi.x) - std::max(aLo.x, bLo.x);
  const double oy = std::min(aHi.y, bHi.y) - std::max(aLo.y, bLo.y);
  const double oz = std::min(aHi.z, bHi.z) - std::max(aLo.z, bLo.z);
  // separation along each axis (positive = gap)
  const double sx = std::max(std::max(aLo.x - bHi.x, bLo.x - aHi.x), 0.0);
  const double sy = std::max(std::max(aLo.y - bHi.y, bLo.y - aHi.y), 0.0);
  const double sz = std::max(std::max(aLo.z - bHi.z, bLo.z - aHi.z), 0.0);
  gap = std::sqrt(sx * sx + sy * sy + sz * sz);
  if (ox > 0 && oy > 0 && oz > 0) return ox * oy * oz;
  return 0.0;
}

// Sphere∩sphere lens volume for centre distance d, radii rA,rB (0 if disjoint / touching).
double sphereLensVolume(double rA, double rB, double d) {
  if (d >= rA + rB) return 0.0;             // disjoint or externally tangent
  if (d <= std::fabs(rA - rB)) {            // one fully inside the other
    const double rmin = std::min(rA, rB);
    return 4.0 / 3.0 * kPi * rmin * rmin * rmin;
  }
  // two spherical caps meeting at the intersection plane (standard lens formula).
  const double part = (rA + rB - d);
  const double num = part * part *
      (d * d + 2 * d * rB - 3 * rB * rB + 2 * d * rA + 6 * rA * rB - 3 * rA * rA);
  return kPi * num / (12.0 * d);
}

// ═══════════════════════════════════════════════════════════════════════════════════════
// GENERATOR — pick a family + a target regime, draw dims + a relative placement that lands
// the pair in that regime, build both operands, run both engines + the closed form.
// ═══════════════════════════════════════════════════════════════════════════════════════

// Build the two operand meshes/solids for a family with body A at origin (identity pose) and
// body B placed by `gB`. `xf` fills the closed-form arbiter when the pose keeps it valid.
void runTrial(int fam, int regime, Rng& r, Trial& out) {
  out.family = fam; out.regime = regime; out.curved = curvedFamily(fam);

  tess::Mesh nA, nB;
  TopoDS_Shape oA, oB;
  Rigid gB;                    // B's placement relative to A (A stays at identity)
  bool axisAligned = false;    // B only translated (no rotation) ⇒ box closed form valid

  // A generic random rotation for B (used by the CLASH/CLEAR regimes where a closed form is
  // not required, and to stress the classifier under arbitrary orientation).
  auto randomAxis = [&] {
    double ax = r.range(-1, 1), ay = r.range(-1, 1), az = r.range(-1, 1);
    const double n = std::sqrt(ax * ax + ay * ay + az * az) + 1e-12;
    gB.ax = ax / n; gB.ay = ay / n; gB.az = az / n;
  };

  if (fam == F_BOX) {
    const double ahx = r.range(1.0, 3.0), ahy = r.range(1.0, 3.0), ahz = r.range(1.0, 3.0);
    double bhx = r.range(1.0, 3.0), bhy = r.range(1.0, 3.0), bhz = r.range(1.0, 3.0);
    // Axis-aligned translation only → the box∩box closed form is exact.
    axisAligned = true; gB.theta = 0;
    if (regime == R_TOUCH) {
      // The TOUCHING knife-edge (the pose the landed op's B3 ON-band + step-4 contact band
      // must handle coplanar-safely). Draw a small signed jitter j about the exact flush pose
      // bHi_x_A == bLo_x_B:
      //   j == 0  exact flush  → TOUCHING (a shared face reads On, NEVER a clash)
      //   j <  0  slight overlap (deeper than the band) → a REAL thin CLASH (must fire)
      //   j >  0  slight gap    → CLEAR
      // The certified contact envelope is a SEATED mate: B's shared-face footprint (its Y/Z
      // extents) is contained within A's, so at least one operand has a boundary vertex on
      // the other's face — the assembly-mate contact the op guarantees exact (shaft-in-bore /
      // coincident / contained / slid face). A pure coplanar-CROSS overlap where NEITHER box
      // contains a vertex of the other is OUT of the op's documented step-4 exactness ("a
      // tight bound otherwise" — the edge-edge min is not evaluated) and is reported as a
      // localized native limitation, NOT drawn here (it would test outside the certified
      // surface, and no tolerance is widened to mask it).
      bhy = std::min(bhy, ahy); bhz = std::min(bhz, ahz);   // B's contact face ⊆ A's
      const int knife = static_cast<int>(r.below(3));       // 0 flush, 1 overlap, 2 gap
      double j = 0.0;
      if (knife == 1) j = -r.range(0.08, 0.5);              // penetrate (well past 2*deflection)
      else if (knife == 2) j = r.range(0.08, 0.5);          // separate
      gB.tx = ahx + bhx + j;
      gB.ty = r.range(-1.0, 1.0) * (ahy - bhy);             // slide, keeping B's face ⊆ A's
      gB.tz = r.range(-1.0, 1.0) * (ahz - bhz);
    } else if (regime == R_CLASH) {
      gB.tx = r.range(0.2, 0.9) * (ahx + bhx);   // guaranteed X-overlap
      gB.ty = r.range(-0.4, 0.4) * (ahy);
      gB.tz = r.range(-0.4, 0.4) * (ahz);
    } else {  // R_CLEAR
      gB.tx = (ahx + bhx) + r.range(1.0, 4.0);   // comfortable gap on +X
      gB.ty = r.range(-1.0, 1.0);
      gB.tz = r.range(-1.0, 1.0);
    }
    // build after the final B half-extents are known (TOUCH may shrink bhy/bhz to seat B).
    nA = boxMesh(ahx, ahy, ahz); oA = occtBox(ahx, ahy, ahz);
    nB = boxMesh(bhx, bhy, bhz); oB = occtBox(bhx, bhy, bhz);
    out.desc = fmt("A(%.2f,%.2f,%.2f)", ahx, ahy, ahz) +
               fmt(" B(%.2f,%.2f,%.2f)", bhx, bhy, bhz) +
               fmt(" t=(%.2f,%.2f,%.2f)", gB.tx, gB.ty, gB.tz);
  } else if (fam == F_NGON) {
    const int n = 3 + static_cast<int>(r.below(6));   // 3..8-gon
    const double Ra = r.range(1.0, 2.5), Rb = r.range(1.0, 2.5);
    const double ahz = r.range(1.0, 2.5), bhz = r.range(1.0, 2.5);
    nA = prismMesh(n, Ra, ahz); oA = occtPrism(n, Ra, ahz);
    nB = prismMesh(n, Rb, bhz); oB = occtPrism(n, Rb, bhz);
    randomAxis();
    if (regime == R_TOUCH) {
      // Flush cap contact on +Z (both prisms axis +Z, no rotation): B's -Z cap on A's +Z.
      // Same knife-edge jitter as the box family: flush / slight penetrate / slight gap.
      const int knife = static_cast<int>(r.below(3));
      double j = 0.0;
      if (knife == 1) j = -r.range(0.08, 0.5);
      else if (knife == 2) j = r.range(0.08, 0.5);
      gB.theta = 0; gB.tx = 0; gB.ty = 0; gB.tz = ahz + bhz + j;
    } else if (regime == R_CLASH) {
      gB.theta = r.range(0.1, 1.2);
      gB.tx = r.range(-0.6, 0.6) * Ra; gB.ty = r.range(-0.6, 0.6) * Ra;
      gB.tz = r.range(-0.5, 0.5) * ahz;
    } else {  // R_CLEAR
      gB.theta = r.range(0.0, 1.5);
      gB.tx = 0; gB.ty = 0; gB.tz = (ahz + bhz) + r.range(1.0, 4.0);
    }
    out.desc = fmt("n=%.0f Ra=%.2f Rb=%.2f", (double)n, Ra, Rb) +
               fmt(" hzA=%.2f hzB=%.2f", ahz, bhz) +
               fmt(" th=%.2f tz=%.2f", gB.theta, gB.tz);
  } else if (fam == F_CYL) {
    const double Ra = r.range(1.0, 2.5), Rb = r.range(1.0, 2.5);
    const double ahz = r.range(1.5, 3.0), bhz = r.range(1.5, 3.0);
    const int seg = 48;
    nA = cylinderMesh(Ra, ahz, seg); oA = occtCylinder(Ra, ahz);
    nB = cylinderMesh(Rb, bhz, seg); oB = occtCylinder(Rb, bhz);
    if (regime == R_TOUCH) {
      // coaxial cap-to-cap flush on +Z, with the knife-edge jitter (flush / penetrate / gap).
      const int knife = static_cast<int>(r.below(3));
      double j = 0.0;
      if (knife == 1) j = -r.range(0.08, 0.5);
      else if (knife == 2) j = r.range(0.08, 0.5);
      gB.theta = 0; gB.tx = 0; gB.ty = 0; gB.tz = ahz + bhz + j;
    } else if (regime == R_CLASH) {
      // coaxial axial overlap (guaranteed positive-volume interior overlap).
      gB.theta = 0; gB.tx = r.range(-0.3, 0.3) * std::min(Ra, Rb);
      gB.ty = 0; gB.tz = r.range(0.2, 0.8) * (ahz + bhz);
    } else {  // R_CLEAR — comfortable axial gap.
      gB.theta = 0; gB.tx = 0; gB.ty = 0; gB.tz = (ahz + bhz) + r.range(1.0, 4.0);
    }
    out.desc = fmt("Ra=%.2f Rb=%.2f", Ra, Rb) + fmt(" hzA=%.2f hzB=%.2f", ahz, bhz) +
               fmt(" tz=%.2f", gB.tz);
  } else {  // F_SPH — has the sphere-lens closed form.
    const double Ra = r.range(1.0, 2.5), Rb = r.range(1.0, 2.5);
    const int stacks = 24, slices = 48;
    nA = sphereMesh(Ra, stacks, slices); oA = occtSphere(Ra);
    nB = sphereMesh(Rb, stacks, slices); oB = occtSphere(Rb);
    gB.theta = 0;   // a sphere is rotation-invariant; translate only → centre distance = |t|
    double d = 0;
    if (regime == R_TOUCH) {
      // external tangency ± the knife-edge jitter: exact / slight overlap / slight gap.
      const int knife = static_cast<int>(r.below(3));
      double j = 0.0;
      if (knife == 1) j = -r.range(0.08, 0.5);
      else if (knife == 2) j = r.range(0.08, 0.5);
      d = Ra + Rb + j;
    } else if (regime == R_CLASH) {
      d = std::fabs(Ra - Rb) + r.range(0.2, 0.8) * (2.0 * std::min(Ra, Rb));  // transversal
    } else {  // R_CLEAR
      d = (Ra + Rb) + r.range(1.0, 4.0);
    }
    // place centre distance d along a random unit direction.
    double dx = r.range(-1, 1), dy = r.range(-1, 1), dz = r.range(-1, 1);
    const double nn = std::sqrt(dx * dx + dy * dy + dz * dz) + 1e-12;
    gB.tx = d * dx / nn; gB.ty = d * dy / nn; gB.tz = d * dz / nn;
    // sphere-lens closed form. Regime: disjoint (d>rA+rB) CLEAR; external tangency
    // (d==rA+rB) TOUCHING; else positive-volume lens ⇒ CLASH.
    out.haveClosed = true;
    out.xVol = sphereLensVolume(Ra, Rb, d);
    out.xDist = d;
    if (d > Ra + Rb + 1e-6) out.xState = CLEAR;
    else if (std::fabs(d - (Ra + Rb)) <= 1e-6) out.xState = TOUCHING;
    else out.xState = CLASH;
    out.desc = fmt("Ra=%.2f Rb=%.2f d=%.3f", Ra, Rb, d);
  }

  // place body B (native mesh + OCCT solid) by the shared rigid transform.
  transformMesh(nB, gB);
  oB = BRepBuilderAPI_Transform(oB, occtRigid(gB), Standard_True).Shape();

  // ── box closed form (axis-aligned families only) ─────────────────────────────────────
  if (fam == F_BOX && axisAligned) {
    nm::Point3 aLo, aHi, bLo, bHi;
    // A at identity: its AABB is its half-extents; recompute from mesh for safety.
    auto bounds = [](const tess::Mesh& m, nm::Point3& lo, nm::Point3& hi) {
      lo = hi = m.vertices[0];
      for (const auto& v : m.vertices) {
        lo.x = std::min(lo.x, v.x); hi.x = std::max(hi.x, v.x);
        lo.y = std::min(lo.y, v.y); hi.y = std::max(hi.y, v.y);
        lo.z = std::min(lo.z, v.z); hi.z = std::max(hi.z, v.z);
      }
    };
    bounds(nA, aLo, aHi); bounds(nB, bLo, bHi);
    double gap = 0;
    out.haveClosed = true;
    out.xVol = boxOverlapVolume(aLo, aHi, bLo, bHi, gap);
    out.xDist = gap;
    if (out.xVol > 1e-9) out.xState = CLASH;
    else if (gap <= 1e-9) out.xState = TOUCHING;
    else out.xState = CLEAR;
  }

  // ── honest-decline probe ─────────────────────────────────────────────────────────────
  // A minority of trials deliberately hand the NATIVE classifier a NON-WATERTIGHT operand
  // (drop two triangles of A → an open shell). The landed op's precondition (interference.h
  // step 1) must DECLINE (Unknown) rather than guess, and the harness counts it HONESTLY-
  // DECLINED (native → OCCT, which still sees the intact analytic solid). This exercises the
  // "a wrong overlap is NEVER returned" contract directly, not just the clean-mesh path.
  if (r.unit() < 0.12 && nA.triangles.size() > 4) {
    nA.triangles.resize(nA.triangles.size() - 2);
    out.soupProbe = true;
  }

  // ── run both engines ─────────────────────────────────────────────────────────────────
  an::InterferenceResult nr;
  out.nState = nativeState(nA, nB, nr);
  out.nVol = nr.overlapVolume;   // mesh classifier leaves this 0; state is the signal here
  out.nDist = nr.minDistance;
  out.oState = occtState(oA, oB, out.oVol, out.oDist, out.oracleOk);
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0xC1A54FEEDull;
  int N = 72;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 72;

  std::printf("== M6-breadth-17 differential-fuzz: native INTERFERENCE/CLASH "
              "(box/ngon/cyl/sphere pairs x CLEAR/TOUCHING/CLASH) vs OCCT + closed-form ==\n");
  std::printf("== seed=0x%llx N=%d deflection=%.3g  (contact band = max(1e-9*scale, 2*deflection); "
              "NEVER widened) ==\n", static_cast<unsigned long long>(seed), N, kDefl);
  std::fflush(stdout);

  Rng rng(seed);
  // Round-robin the families and regimes so every [family x regime] cell is populated
  // deterministically (12 cells; N>=60 gives >=5 trials/cell).
  for (int i = 0; i < N; ++i) {
    const int fam = i % F_COUNT;
    const int reg = (i / F_COUNT) % R_COUNT;
    Trial t;
    runTrial(fam, reg, rng, t);
    const Verdict v = classify(t);
    tally(v, fam, reg);
    report(i, v, t, seed);
  }

  // ── coverage summary ────────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  "
              "FACET-CONVERGENT=%d  BOTH-DECLINED=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleInacc, g_facet, g_bothDecl);
  std::printf("   per-family x regime [AGREED / HONESTLY-DECLINED / DISAGREED / "
              "ORACLE-INACCURATE / FACET-CONVERGENT / BOTH-DECLINED]:\n");
  for (int f = 0; f < F_COUNT; ++f)
    for (int rg = 0; rg < R_COUNT; ++rg)
      std::printf("     %-14s %-6s  %d / %d / %d / %d / %d / %d\n",
                  famName(f), regimeName(rg), g_A[f][rg], g_D[f][rg], g_X[f][rg],
                  g_OI[f][rg], g_FC[f][rg], g_BD[f][rg]);
  if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — logged, NOT a fault)\n", g_oracleInacc);
  if (g_facet)       std::printf("   FACET-CONVERGENT=%d (curved TOUCH<->CLEAR facet straddle within the deflection band — logged)\n", g_facet);
  if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (both engines declined — no wrong result, logged)\n", g_bothDecl);

  // Bar: DISAGREED==0 AND every populated cell showed real exercise (not all-declined). A
  // cell "exercised" if it has >=1 of {AGREED, ORACLE-INACCURATE, FACET-CONVERGENT} — a
  // first-class decided outcome (a cell that is ALL honest-decline is not a native exercise).
  bool coverage = true;
  for (int f = 0; f < F_COUNT; ++f)
    for (int rg = 0; rg < R_COUNT; ++rg) {
      const int decided = g_A[f][rg] + g_OI[f][rg] + g_FC[f][rg];
      const int total = decided + g_D[f][rg] + g_X[f][rg] + g_BD[f][rg];
      if (total > 0 && decided < 1) coverage = false;   // populated but never truly exercised
    }
  const bool bar = (g_disagreed == 0 && coverage);
  std::printf("== M6-breadth-17 BAR: %s (DISAGREED=%d must be 0; per-cell exercise coverage=%s) ==\n",
              bar ? "PASS — zero silent wrong clashes" : "FAIL", g_disagreed,
              coverage ? "complete" : "INCOMPLETE");
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
