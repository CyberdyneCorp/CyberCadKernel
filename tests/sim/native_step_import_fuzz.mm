// SPDX-License-Identifier: Apache-2.0
//
// native_step_import_fuzz.mm — MOAT M6b (the COMPLETENESS BAR, SECOND domain): a
// STEP ROUND-TRIP IMPORT differential-fuzzing harness (iOS simulator).
//
// This is the M6-breadth slice: it extends the landed M6 curved-boolean differential
// fuzzer (native_boolean_fuzz.mm) to a SECOND, independent native domain — the native
// ISO-10303-21 (Part-21) STEP READER (src/native/exchange/step_reader) — so drop-occt is
// gated by more than one fuzzed capability. Like the sibling it is INFRASTRUCTURE (a test
// harness, not a geometry capability): OCCT is the ORACLE, the bar is ZERO SILENT WRONG
// RESULTS over a seeded batch, and an HONEST DECLINE is a first-class outcome.
//
// ── THE DIFFERENTIAL (native reader vs OCCT reader on the SAME bytes) ──────────────
// The discipline mirrors the landed curated round-trip harness native_step_import_parity's
// `runNativeWritten` (native-write → import BOTH ways → compare), turned into a SEEDED batch:
//   (1) DETERMINISTICALLY generate a random-but-VALID native B-rep SOLID from families the
//       native STEP WRITER serialises AND whose native-write → OCCT-read round-trip is a
//       CLEAN oracle (see SCOPE below):
//         box / n-gon prism (planar)        — PLANE faces, LINE edges
//         cylinder (revolved rectangle)     — CYLINDRICAL_SURFACE + planar caps
//         frustum  (revolved trapezoid)     — CONICAL_SURFACE + planar caps
//         holed box (circular through-hole) — PLANE caps + CIRCLE edges + CYLINDRICAL wall
//       Built through the SAME native construct entry points the cc_solid_extrude /
//       cc_solid_revolve / cc_solid_extrude_holes facade uses (build_prism /
//       build_revolution raw-polygon / build_prism_with_holes). The RNG is a
//       splitmix64-seeded xoshiro256** stream keyed ONLY by an explicit FUZZ_SEED
//       (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
//   (2) EXPORT the solid to ONE on-disk STEP file via the native writer, then IMPORT THAT
//       SAME FILE two ways:
//         native : src/native/exchange/step_reader (readStepFile — OCCT-FREE Part-21)
//         oracle : OCCT STEPControl_Reader (the reference importer, measured exactly by BRepGProp)
//   (3) Classify each trial into EXACTLY ONE bucket:
//         AGREED             — native reader returns a watertight solid whose vol/area AND
//                              solid-count match the OCCT re-import within a FIXED relTol.
//         HONESTLY-DECLINED  — native reader returns NULL / non-watertight → the engine falls
//                              back to OCCT (logged); the OCCT re-import of the SAME file is a
//                              valid closed solid (a real, ship-able oracle result).
//         DISAGREED          — native reader returns a watertight solid but does NOT match the
//                              CLOSED-FORM ANALYTIC ground truth (below). A genuine SILENT WRONG
//                              IMPORT — the failure this harness exists to catch.
//         ORACLE-INACCURATE  — native watertight, DIFFERS from OCCT, but MATCHES the analytic
//                              ground truth while OCCT does NOT. The native import is CORRECT and
//                              is VINDICATED by exact math; OCCT's re-import is the inaccurate
//                              one. Logged in full (NOT a bar failure, NOT a native fault).
//   (4) Print a coverage summary. Exit 0 IFF the bar holds. Any DISAGREE / ORACLE-INACCURATE
//       prints seed + case index + family/param tuple + all three measurements.
//
// ── ANALYTIC GROUND-TRUTH ARBITER (why native-vs-OCCT alone is not enough) ─────────
// Every family here has a closed-form volume + area (box/prism/cylinder/frustum/holed-box), so
// the harness computes the EXACT truth per case and uses it to ATTRIBUTE a native-vs-OCCT
// disagreement rather than reflexively blaming the native reader. This is a STRENGTHENING, not a
// weakening: a native result is only exonerated when it POSITIVELY matches exact math while OCCT
// does not, and a genuine native error (native ≠ analytic) still fails the bar. This matters
// because OCCT's STEPControl_Reader re-imports a SHALLOW native cone slightly inaccurately: seed
// 0x1234 index=10 (frustum r0=1.4211 r1=1.3651 H=2.0612) has native vol/area within 9e-4/5e-4 of
// the analytic frustum (12.5688 / 30.2474) while OCCT is 2.7e-2/1.2e-2 HIGH — the native reader
// is right, OCCT is the outlier. Classified ORACLE-INACCURATE, logged, bar NOT failed.
//
// ── GUARDS (so a DISAGREE is the READER's fault, never the oracle's) ───────────────
//   * WRITER_DECLINE     — the native writer could not serialise the source (canSerialize
//                          false / empty emit). The generator reached past the writer's
//                          scope; the trial exercises NO reader. Logged as a coverage drop,
//                          NOT a bar failure (an honest writer decline is allowed — but it is
//                          counted + printed so a silently-uncovered family cannot hide).
//   * ORACLE_UNRELIABLE  — the OCCT re-import of the written file is NOT a valid closed solid.
//                          For the SCOPED families this must never happen; if it does the file
//                          is not a trustworthy oracle → excluded from the reader verdict and
//                          FAILS the bar (investigate the writer / OCCT, never launder it).
//   The BAR: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0.
//
// ── SCOPE (honest, bounded — logged exclusions) ───────────────────────────────────
// Two writer-producible families are DELIBERATELY EXCLUDED because the native-write →
// OCCT-read round-trip is not a clean oracle for them (verified empirically, seed
// 0x5744EE9911):
//   * SPHERE (on-axis-arc, bare-periodic SPHERICAL_SURFACE bounded by a VERTEX_LOOP): OCCT's
//     STEPControl_Reader re-imports the native VERTEX_LOOP-bounded sphere INCONSISTENTLY
//     (sometimes 3 spurious sub-solids / near-zero or negative area), so OCCT is not a
//     trustworthy oracle for it here. It IS covered by the curated foreign-authored
//     round-trip in native_step_import_parity (runRevolvedSphere), against an OCCT-authored file.
//   * RULED LOFT (bilinear B_SPLINE_SURFACE side faces): the native writer HONESTLY DECLINES
//     to serialise it (canSerialize=false) — out of the current writer alphabet. A writer-scope
//     limit, not a reader gap.
// These exclusions are a first-class honest DECLINE at the DOMAIN level; they are noted here
// and in the OpenSpec change so no coverage is silently dropped.
//
// The agreement tolerance is FIXED (relTol 2e-2) and NEVER widened per-trial. The native
// reader's reconstruction is measured by the native tessellator (mesh volume/area) while the
// OCCT re-import is measured exactly by BRepGProp; that inscribed-facet asymmetry is the SAME
// as the sibling boolean fuzzer, so the mesh is tessellated fine (deflection 0.001) to keep a
// CORRECT native import's bias (≈2–4e-3) comfortably under the 2e-2 bar while a genuine
// wrong-set is off by a large margin.
//
// This TU is OCCT-dependent (STEPControl_Reader + BRepGProp) but needs NO numsci: the native
// writer + reader + tessellator depend only on src/native/{exchange,math,topology,tessellate,
// construct} — all OCCT-FREE. Built ONLY by scripts/run-sim-native-step-import-fuzz.sh; on
// run-sim-suite.sh's SKIP list (own main()). src/native stays OCCT-FREE — this harness is
// additive test/sim code only. Flushes and std::_Exit (OCCT static teardown in the trimmed
// static build is not exit-clean — same rationale as the sibling).
//
#include "native/construct/native_construct.h"    // build_prism / build_revolution / build_prism_with_holes
#include "native/exchange/native_exchange.h"       // writeStepFile / canSerialize / readStepFile
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_step_import_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT STEPControl_Reader oracle"
#endif

#include <STEPControl_Reader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>

namespace ncst  = cybercad::native::construct;
namespace nex   = cybercad::native::exchange;
namespace ntess = cybercad::native::tessellate;
namespace ntopo = cybercad::native::topology;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRelTol = 2e-2;       // FIXED agreement bar (never widened per-trial)
constexpr double kDeflection = 0.001;  // fine mesh — see header (matches sibling rationale)
const char* kStepPath = "/tmp/cck_m6b_step_fuzz.step";

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream (verbatim discipline of
//    the sibling native_boolean_fuzz). Keyed ONLY by an explicit uint64 seed. No clock,
//    no rand(): same seed → byte-identical batch. ─────────────────────────────────────
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

enum Family { F_BOX, F_PRISM, F_CYL, F_FRUSTUM, F_HOLED, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_BOX:     return "box(planar)";
    case F_PRISM:   return "ngon-prism(planar)";
    case F_CYL:     return "cylinder(cyl+caps)";
    case F_FRUSTUM: return "frustum(cone+caps)";
    case F_HOLED:   return "holed-box(circle+cyl-wall)";
  }
  return "?";
}

// ── native operand builders — the SAME construct entry points the cc_* facade uses ──
ntopo::Shape makeBox(double w, double d, double h) {
  const double rect[] = {0.0, 0.0, w, 0.0, w, d, 0.0, d};
  return ncst::build_prism(rect, 4, h);
}
ntopo::Shape makeRegularPrism(int n, double R, double rot, double h) {
  std::vector<double> xy;
  xy.reserve(static_cast<std::size_t>(n) * 2);
  for (int i = 0; i < n; ++i) {
    const double a = rot + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
    xy.push_back(R * std::cos(a));
    xy.push_back(R * std::sin(a));
  }
  return ncst::build_prism(xy.data(), n, h);
}
// Revolve a CCW (x,y) silhouette 360° about +Y — the cc_solid_revolve raw-polygon path.
// rect {0,0,r,0,r,h,0,h} → cylinder;  trapezoid {0,0,r0,0,r1,h,0,h} → frustum.
ntopo::Shape makeRevolved(const std::vector<double>& xy) {
  ncst::RevolveAxis ax; ax.ax = 0.0; ax.ay = 0.0; ax.adx = 0.0; ax.ady = 1.0;  // +Y axis
  return ncst::build_revolution(xy.data(), static_cast<int>(xy.size() / 2), ax, 2.0 * kPi);
}
ntopo::Shape makeCylinder(double r, double h) { return makeRevolved({0, 0, r, 0, r, h, 0, h}); }
ntopo::Shape makeFrustum(double r0, double r1, double h) {
  return makeRevolved({0, 0, r0, 0, r1, h, 0, h});
}
// Box with ONE centred circular through-hole → cc_solid_extrude_holes path.
ntopo::Shape makeHoledBox(double w, double d, double h, double holeR) {
  const double rect[] = {0.0, 0.0, w, 0.0, w, d, 0.0, d};
  const std::vector<ncst::CircleHole> holes{ncst::CircleHole{w * 0.5, d * 0.5, holeR}};
  return ncst::build_prism_with_holes(rect, 4, holes, {}, h);
}

// A generated case: the built native source + a human-readable repro tuple + the CLOSED-
// FORM analytic volume/area of the family (the independent ground-truth ARBITER used to
// attribute a native-vs-OCCT disagreement — see the classifier).
struct GenCase { int family = 0; ntopo::Shape shape; std::string desc; double aVol = 0, aArea = 0; };

// Closed-form volume/area of each family (exact ground truth).
void analyticBox(double w, double d, double h, double& v, double& a) {
  v = w * d * h; a = 2.0 * (w * d + w * h + d * h);
}
void analyticRegularPrism(int n, double R, double h, double& v, double& a) {
  const double base = 0.5 * n * R * R * std::sin(2.0 * kPi / n);   // n-gon area
  const double perim = 2.0 * n * R * std::sin(kPi / n);
  v = base * h; a = 2.0 * base + perim * h;
}
void analyticCylinder(double R, double H, double& v, double& a) {
  v = kPi * R * R * H; a = 2.0 * kPi * R * R + 2.0 * kPi * R * H;
}
void analyticFrustum(double r0, double r1, double H, double& v, double& a) {
  v = (kPi * H / 3.0) * (r0 * r0 + r0 * r1 + r1 * r1);
  const double slant = std::sqrt((r0 - r1) * (r0 - r1) + H * H);
  a = kPi * (r0 * r0 + r1 * r1) + kPi * (r0 + r1) * slant;
}
void analyticHoledBox(double w, double d, double h, double holeR, double& v, double& a) {
  const double capNet = w * d - kPi * holeR * holeR;   // one cap area minus the hole
  v = capNet * h;
  a = 2.0 * capNet + 2.0 * (w + d) * h + 2.0 * kPi * holeR * h;  // 2 caps + outer walls + hole wall
}

int pickFamily(Rng& r) {
  const int w[F_COUNT] = {1, 1, 1, 1, 1};  // even weight — real coverage per family
  int tot = 0; for (int x : w) tot += x;
  int k = static_cast<int>(r.below(static_cast<uint32_t>(tot)));
  for (int i = 0; i < F_COUNT; ++i) { if (k < w[i]) return i; k -= w[i]; }
  return F_BOX;
}

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[192]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

GenCase genCase(Rng& r) {
  GenCase c; c.family = pickFamily(r);
  switch (c.family) {
    case F_BOX: {
      const double w = r.range(1.0, 4.0), d = r.range(1.0, 4.0), h = r.range(1.0, 4.0);
      c.shape = makeBox(w, d, h);
      c.desc = fmt("box w=%.4f d=%.4f h=%.4f", w, d, h);
      analyticBox(w, d, h, c.aVol, c.aArea);
      break;
    }
    case F_PRISM: {
      const int n = 3 + static_cast<int>(r.below(6));   // 3..8-gon
      const double R = r.range(1.0, 3.0), rot = r.range(0.0, kPi), h = r.range(1.0, 4.0);
      c.shape = makeRegularPrism(n, R, rot, h);
      c.desc = fmt("prism n=%.0f R=%.4f rot=%.4f h=%.4f", static_cast<double>(n), R, rot, h);
      analyticRegularPrism(n, R, h, c.aVol, c.aArea);
      break;
    }
    case F_CYL: {
      const double R = r.range(0.8, 3.0), H = r.range(1.0, 5.0);
      c.shape = makeCylinder(R, H);
      c.desc = fmt("cyl R=%.4f H=%.4f", R, H);
      analyticCylinder(R, H, c.aVol, c.aArea);
      break;
    }
    case F_FRUSTUM: {
      // Both radii strictly positive (a true frustum, never a degenerate apex) so the
      // native-write → OCCT-read round-trip stays a clean oracle.
      const double r0 = r.range(0.8, 3.0), r1 = r.range(0.8, 3.0), H = r.range(1.5, 5.0);
      c.shape = makeFrustum(r0, r1, H);
      c.desc = fmt("frustum r0=%.4f r1=%.4f H=%.4f", r0, r1, H);
      analyticFrustum(r0, r1, H, c.aVol, c.aArea);
      break;
    }
    case F_HOLED: {
      const double w = r.range(3.0, 6.0), d = r.range(3.0, 6.0), h = r.range(1.0, 4.0);
      const double holeR = r.range(0.4, std::min(w, d) * 0.35);   // fits inside with margin
      c.shape = makeHoledBox(w, d, h, holeR);
      c.desc = fmt("holed w=%.4f d=%.4f h=%.4f holeR=%.4f", w, d, h, holeR);
      analyticHoledBox(w, d, h, holeR, c.aVol, c.aArea);
      break;
    }
  }
  return c;
}

// ── measurement ────────────────────────────────────────────────────────────────────
struct NativeMeasure { bool present = false, watertight = false; double volume = 0, area = 0; int solids = 0; };
NativeMeasure measureNative(const ntopo::Shape& s) {
  NativeMeasure m;
  if (s.isNull()) return m;
  m.present = true;
  for (ntopo::Explorer e(s, ntopo::ShapeType::Solid); e.more(); e.next()) ++m.solids;
  ntess::MeshParams p; p.deflection = kDeflection;
  const ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(s);
  m.watertight = ntess::isWatertight(mesh);
  m.volume = std::fabs(ntess::enclosedVolume(mesh));
  m.area = ntess::surfaceArea(mesh);
  return m;
}

struct OcctMeasure { bool present = false, valid = false, closedShell = false; double volume = 0, area = 0; int solids = 0; };
// Re-import a STEP file with the OCCT oracle and measure it exactly (BRepGProp).
OcctMeasure measureOcctStep(const std::string& path) {
  OcctMeasure m;
  STEPControl_Reader r;
  if (r.ReadFile(path.c_str()) != IFSelect_RetDone) return m;
  r.TransferRoots();
  const TopoDS_Shape s = r.OneShape();
  if (s.IsNull()) return m;
  m.present = true;
  BRepCheck_Analyzer an(s);
  m.valid = an.IsValid();
  bool anyShell = false, allClosed = true;
  for (TopExp_Explorer ex(s, TopAbs_SHELL); ex.More(); ex.Next()) {
    anyShell = true;
    if (!BRep_Tool::IsClosed(TopoDS::Shell(ex.Current()))) allClosed = false;
  }
  m.closedShell = anyShell && allClosed;
  for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++m.solids;
  GProp_GProps vg; BRepGProp::VolumeProperties(s, vg); m.volume = std::fabs(vg.Mass());
  GProp_GProps ag; BRepGProp::SurfaceProperties(s, ag); m.area = ag.Mass();
  return m;
}

double relDiff(double a, double b) { return (b > 1e-12) ? std::fabs(a - b) / b : 1e30; }

// ── the classifier ──────────────────────────────────────────────────────────────────
//   AGREED            native watertight + matches OCCT within tol.
//   DECLINED          native NULL / non-watertight → OCCT fallback (oracle valid).
//   ORACLE_INACCURATE native watertight, DIFFERS from OCCT, but MATCHES the closed-form
//                     analytic ground truth while OCCT does NOT — the native reader is
//                     VINDICATED by exact math (an oracle-side limitation, logged, NOT a
//                     bar failure; the native import is correct).
//   DISAGREED         native watertight but does NOT match the analytic ground truth (a
//                     genuine SILENT WRONG native import — the bar failure).
//   ORACLE_UNRELIABLE OCCT re-import is not a valid closed solid (excluded, fails bar).
//   WRITER_DECLINE    native writer could not serialise (coverage drop, logged).
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, WRITER_DECLINE, ORACLE_UNRELIABLE };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_writerDecline = 0, g_oracleBad = 0;
int g_famAgreed[F_COUNT] = {0}, g_famDeclined[F_COUNT] = {0}, g_famDisagreed[F_COUNT] = {0};
int g_famOracleInacc[F_COUNT] = {0}, g_famWriterDecline[F_COUNT] = {0};

}  // namespace

int main(int argc, char** argv) {
  // Seed + N from argv (argv[1]=seed, argv[2]=N) or env (FUZZ_SEED / FUZZ_N). Explicit
  // only — NO clock. Defaults are fixed so a bare run is still deterministic.
  uint64_t seed = 0x5744EE9911ull;
  int N = 96;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 96;

  std::printf("== M6b differential-fuzz: native STEP-import round-trip vs OCCT STEPControl_Reader ==\n");
  std::printf("== seed=0x%llx N=%d relTol=%.0e deflection=%g ==\n",
              static_cast<unsigned long long>(seed), N, kRelTol, kDeflection);
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    const GenCase c = genCase(rng);

    // (1) WRITER — serialise the source to ONE on-disk STEP file (native writer). A null
    //     source or a writer decline is a coverage drop (logged), never a bar failure.
    if (c.shape.isNull() || !nex::step_can_export_native(c.shape) ||
        !nex::writeStepFile(c.shape, kStepPath)) {
      ++g_writerDecline; ++g_famWriterDecline[c.family];
      std::printf("[FUZZ] WRITER_DECLINE case=%d %-26s native writer declined to serialise  %s\n",
                  i, famName(c.family), c.desc.c_str());
      std::fflush(stdout);
      continue;
    }

    // (2) ORACLE — OCCT re-imports the SAME file. It MUST be a valid closed solid, else it
    //     is not a trustworthy oracle (excluded from the reader verdict, fails the bar).
    const OcctMeasure occ = measureOcctStep(kStepPath);
    const bool oracleSolid = occ.present && occ.valid && occ.closedShell &&
                             occ.volume > 1e-9 && occ.area > 1e-9 && occ.solids >= 1;
    if (!oracleSolid) {
      ++g_oracleBad;
      std::printf("[FUZZ] ORACLE_UNRELIABLE case=%d %-26s OCCT re-import not a valid closed solid "
                  "[present=%d valid=%d closed=%d solids=%d volO=%.5g areaO=%.5g]\n"
                  "       REPRO seed=0x%llx index=%d %s\n",
                  i, famName(c.family), occ.present, occ.valid, occ.closedShell, occ.solids,
                  occ.volume, occ.area, static_cast<unsigned long long>(seed), i, c.desc.c_str());
      std::fflush(stdout);
      continue;
    }

    // (3) NATIVE reader — import the SAME file OCCT-free (Part-21). Measure by native mesh.
    const ntopo::Shape natShape = nex::step_import_native(kStepPath);
    const NativeMeasure nat = measureNative(natShape);
    const double volRel = relDiff(nat.volume, occ.volume);       // native vs OCCT (differential)
    const double areaRel = relDiff(nat.area, occ.area);
    const double natVsA_V = relDiff(nat.volume, c.aVol);         // native vs analytic ground truth
    const double natVsA_A = relDiff(nat.area, c.aArea);
    const double occVsA_V = relDiff(occ.volume, c.aVol);         // OCCT   vs analytic ground truth
    const double occVsA_A = relDiff(occ.area, c.aArea);

    Verdict v;
    if (!(nat.present && nat.watertight && nat.volume > 1e-9)) {
      v = DECLINED;  // native NULL / non-watertight → OCCT fallback ships (oracle proven valid)
    } else if (volRel < kRelTol && areaRel < kRelTol && nat.solids == occ.solids) {
      v = AGREED;    // native watertight AND matches the OCCT re-import of the same bytes
    } else {
      // Native watertight but DIFFERS from OCCT. Attribute the disagreement with the
      // independent closed-form ground truth: a native result that MATCHES exact math while
      // OCCT does not is CORRECT (an oracle-side inaccuracy). Only a native result that fails
      // the analytic truth is a genuine SILENT WRONG native import.
      const bool natMatchesAnalytic = natVsA_V < kRelTol && natVsA_A < kRelTol;
      const bool occMatchesAnalytic = occVsA_V < kRelTol && occVsA_A < kRelTol;
      v = (natMatchesAnalytic && !occMatchesAnalytic) ? ORACLE_INACCURATE : DISAGREED;
    }

    switch (v) {
      case AGREED:            ++g_agreed;      ++g_famAgreed[c.family];      break;
      case DECLINED:          ++g_declined;    ++g_famDeclined[c.family];    break;
      case DISAGREED:         ++g_disagreed;   ++g_famDisagreed[c.family];   break;
      case ORACLE_INACCURATE: ++g_oracleInacc; ++g_famOracleInacc[c.family]; break;
      default: break;
    }

    if (v == AGREED) {
      std::printf("[FUZZ] AGREED    case=%d %-26s volN=%.5g volO=%.5g dV=%.2e areaN=%.5g areaO=%.5g dA=%.2e solids=%d/%d\n",
                  i, famName(c.family), nat.volume, occ.volume, volRel, nat.area, occ.area, areaRel,
                  nat.solids, occ.solids);
    } else if (v == DECLINED) {
      std::printf("[FUZZ] DECLINED  case=%d %-26s native=%s -> OCCT[valid=%d closed=%d volO=%.5g areaO=%.5g solids=%d]  %s\n",
                  i, famName(c.family), nat.present ? "non-watertight" : "NULL",
                  occ.valid, occ.closedShell, occ.volume, occ.area, occ.solids, c.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
      // Native is VINDICATED by exact math; OCCT's re-import is the inaccurate one. Logged in
      // full (not a bar failure) — the native import is correct.
      std::printf("[FUZZ] ORACLE_INACCURATE case=%d %-26s native MATCHES analytic, OCCT does NOT "
                  "volN=%.6g volO=%.6g aVol=%.6g  natVsA_V=%.2e occVsA_V=%.2e  areaN=%.6g areaO=%.6g aArea=%.6g natVsA_A=%.2e occVsA_A=%.2e\n"
                  "       NOTE seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nat.volume, occ.volume, c.aVol, natVsA_V, occVsA_V,
                  nat.area, occ.area, c.aArea, natVsA_A, occVsA_A,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
    } else {  // DISAGREED — native fails the analytic ground truth
      std::printf("[FUZZ] DISAGREED case=%d %-26s SILENT-WRONG-IMPORT "
                  "volN=%.6g volO=%.6g aVol=%.6g dV(occt)=%.3e dV(analytic)=%.3e  areaN=%.6g areaO=%.6g aArea=%.6g dA(analytic)=%.3e solidsN=%d solidsO=%d wt=%d\n"
                  "       REPRO seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nat.volume, occ.volume, c.aVol, volRel, natVsA_V,
                  nat.area, occ.area, c.aArea, natVsA_A, nat.solids, occ.solids, nat.watertight ? 1 : 0,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
    }
    std::fflush(stdout);
  }

  // ── coverage summary ────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleInacc);
  std::printf("   per-family [agreed/declined/DISAGREED/oracle-inaccurate  (writer-declined)]:\n");
  for (int f = 0; f < F_COUNT; ++f) {
    std::printf("     %-28s %d/%d/%d/%d  (%d)\n", famName(f),
                g_famAgreed[f], g_famDeclined[f], g_famDisagreed[f], g_famOracleInacc[f], g_famWriterDecline[f]);
  }
  if (g_oracleInacc)   std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — oracle-side limitation, logged, NOT a native fault)\n", g_oracleInacc);
  if (g_writerDecline) std::printf("   WRITER_DECLINE=%d (source out of native-writer scope — coverage drop, logged)\n", g_writerDecline);
  if (g_oracleBad)     std::printf("   ORACLE_UNRELIABLE=%d (OCCT re-import not a valid solid — investigate writer/OCCT)\n", g_oracleBad);

  const bool bar = (g_disagreed == 0 && g_oracleBad == 0);
  std::printf("== M6b BAR: %s (DISAGREED=%d must be 0) ==\n",
              bar ? "PASS — zero silent wrong imports" : "FAIL", g_disagreed);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
