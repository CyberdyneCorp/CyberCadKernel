// SPDX-License-Identifier: Apache-2.0
//
// native_construct_fuzz.mm — MOAT M6c (the COMPLETENESS BAR, THIRD domain): a
// CONSTRUCTION differential-fuzzing harness (iOS simulator) for the native swept-solid
// builders — RULED LOFT (loft.h) and CONSTANT-FRAME SWEEP (sweep.h).
//
// This is the M6-breadth-2 slice: it extends the landed M6 curved-boolean differential
// fuzzer (native_boolean_fuzz.mm) and the M6-breadth STEP round-trip fuzzer
// (native_step_import_fuzz.mm) to a THIRD, independent native domain — the OCCT-FREE
// construction library src/native/construct (build_loft_sections / build_sweep) — so
// drop-occt is gated by more than two fuzzed capabilities. Like its siblings it is
// INFRASTRUCTURE (a test harness, not a geometry capability): OCCT is the ORACLE, the bar
// is ZERO SILENT WRONG RESULTS over a seeded batch, and an HONEST DECLINE is first-class.
//
// ── THE DIFFERENTIAL (native builder vs OCCT builder on the SAME inputs) ────────────
// The discipline mirrors the landed curated parity harnesses native_loft_parity /
// native_sweep_parity, turned into a SEEDED batch. Unlike those (which drive the cc_*
// facade with cc_set_engine so a native decline SILENTLY forwards to OCCT — you cannot
// then tell native-handled from fall-through), this harness calls the OCCT-FREE native
// builders DIRECTLY, so a NULL / non-watertight result is an UNAMBIGUOUS native DECLINE,
// and OCCT is invoked SEPARATELY as the reference oracle. Per trial:
//   (1) DETERMINISTICALLY generate a random-but-VALID construction input from the
//       families the native path CLAIMS (see SCOPE):
//         L2 frustum  — 2-section coaxial regular-n-gon frustum (equal count)
//         LN stack    — N-section (3..5) coaxial regular-n-gon prismatoid stack
//         L2 mismatch — 2-section, top vertex-count ≠ bottom (T1 collinear resample)
//         SW prism    — closed planar profile swept along a STRAIGHT 3D path
//       plus two SPARSE out-of-scope DECLINE-exercisers (a non-planar loft section; a
//       non-planar sweep spine) that the native builder honestly returns NULL for.
//       The RNG is a splitmix64-seeded xoshiro256** stream keyed ONLY by an explicit
//       FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
//   (2) BUILD the same input two ways:
//         native : ncst::build_loft_sections / ncst::build_sweep (OCCT-FREE), measured
//                  by the native tessellator (mesh vol/area/watertight/solid-count)
//         oracle : OCCT BRepOffsetAPI_ThruSections (ruled solid) / BRepOffsetAPI_MakePipe
//                  — the SAME construction the cc_* OCCT engine uses — measured exactly
//                  by BRepGProp
//   (3) Classify each trial into EXACTLY ONE bucket:
//         AGREED             — native watertight + vol/area AND solid-count match OCCT
//                              within a FIXED relTol.
//         HONESTLY-DECLINED  — native returns NULL / a non-watertight candidate (the
//                              engine's mandatory self-verify would DISCARD it → OCCT,
//                              logged); the OCCT build of the SAME input is a valid closed
//                              solid (a real, ship-able oracle result).
//         DISAGREED          — native watertight but does NOT match the CLOSED-FORM
//                              ANALYTIC ground truth (below). A genuine SILENT WRONG BUILD
//                              — the failure this harness exists to catch.
//         ORACLE-INACCURATE  — native watertight, DIFFERS from OCCT, but MATCHES the
//                              analytic ground truth while OCCT does NOT. The native build
//                              is CORRECT and is VINDICATED by exact math; OCCT is the
//                              outlier. Logged in full (NOT a bar failure, NOT a native fault).
//         BOTH-DECLINED      — a DECLINE-exerciser where native returned NULL AND OCCT also
//                              did not build a valid solid: neither engine produced a wrong
//                              result, so there is nothing to compare. Logged, NOT a bar
//                              failure (an out-of-scope input both engines refuse).
//   (4) Print a coverage summary. Exit 0 IFF the bar holds. Any DISAGREE / ORACLE-
//       INACCURATE prints seed + case index + family/param tuple + all measurements.
//
// ── ANALYTIC GROUND-TRUTH ARBITER (why native-vs-OCCT alone is not enough) ──────────
// Every AGREE family here has a closed-form volume + surface area:
//   * a PRISMATOID loft between coaxial regular n-gons is a stack of pyramidal frustums —
//     band volume = (Δz/3)·(A(R_k) + √(A(R_k)A(R_{k+1})) + A(R_{k+1})), and because each
//     side face is a planar trapezoid the lateral area is closed-form too;
//   * a PRISM sweep along a straight path is profileArea·pathLength with surface area
//     2·profileArea + profilePerimeter·pathLength.
// The harness computes the EXACT truth per case and uses it to ATTRIBUTE a native-vs-OCCT
// disagreement rather than reflexively blaming the native builder. This is a STRENGTHENING,
// not a weakening: a native result is only exonerated when it POSITIVELY matches exact math
// while OCCT does not, and a genuine native error (native ≠ analytic) still fails the bar.
// (The T1 mismatched-count case carries the analytic of its regular-n-gon OUTLINE — the
// collinear resampled vertices change neither volume nor area.)
//
// ── GUARDS (so a DISAGREE is the BUILDER's fault, never the oracle's) ───────────────
//   * ORACLE_UNRELIABLE  — for a CORE family the OCCT build MUST be a valid closed solid;
//                          if it is not, the input is not a trustworthy oracle → excluded
//                          from the verdict and FAILS the bar (investigate, never launder).
//   The BAR: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0.
//
// ── SCOPE (honest, bounded — logged exclusions) ────────────────────────────────────
// The native builders' claimed scope (native_construct.h) is: equal- AND mismatched-count
// PLANAR N-section ruled loft, and STRAIGHT + smooth-planar + RMF constant-frame sweep. To
// keep native-vs-OCCT a CLEAN differential at a FIXED 2e-2 relTol, the fuzzer's arbitrated
// AGREE families are the ones where the native mesh and the OCCT solid are EXACT (planar,
// parallel-plane frustums; straight prisms — no tessellation-vs-exact bias). Two claimed
// sub-families are DELIBERATELY EXCLUDED from the batch comparison and covered instead by
// the curated parity harnesses:
//   * TWISTED / ROTATED-section loft (truly bilinear hyperbolic-paraboloid side faces):
//     native-mesh-vs-OCCT-exact is only deflection-bounded, not exact — covered by
//     native_loft_parity (buildRotatedSquareTwist).
//   * SMOOTH-CURVED PLANAR sweep (constant-frame ruled tube): native constant-frame vs
//     OCCT MakePipe agree only deflection-bounded — covered by native_sweep_parity
//     (buildSmoothArcSweep).
// These exclusions are a first-class honest DECLINE at the DOMAIN level; they are noted here
// and in the OpenSpec change so no coverage is silently dropped. The non-planar loft section
// and non-planar sweep spine ARE generated (sparingly) to exercise the native DECLINE branch.
//
// The agreement tolerance is FIXED (relTol 2e-2) and NEVER widened per-trial. The native
// result is measured by the native tessellator (mesh volume/area) while the OCCT build is
// measured exactly by BRepGProp; for the planar/prism families that inscribed-facet
// asymmetry is ZERO (planar facets reproduce the exact solid), so a CORRECT native build
// sits far under the bar while a genuine wrong build is off by a large margin. The mesh is
// tessellated fine (deflection 0.001) as in the siblings.
//
// This TU is OCCT-dependent (ThruSections + MakePipe + BRepGProp) but needs NO numsci: the
// native construct + tessellate + topology + math live in src/native/** — all OCCT-FREE and
// header-only (math bezier/bspline are the only compiled native TUs). Built ONLY by
// scripts/run-sim-native-construct-fuzz.sh; on run-sim-suite.sh's SKIP list (own main()).
// src/native stays OCCT-FREE — this harness is additive test/sim code only. Flushes and
// std::_Exit (OCCT static teardown in the trimmed static build is not exit-clean — same
// rationale as the siblings).
//
#include "native/construct/native_construct.h"    // build_loft_sections / build_sweep
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_construct_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT ThruSections/MakePipe oracle"
#endif

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>

namespace ncst  = cybercad::native::construct;
namespace ntess = cybercad::native::tessellate;
namespace ntopo = cybercad::native::topology;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRelTol = 2e-2;       // FIXED agreement bar (never widened per-trial)
constexpr double kDeflection = 0.001;  // fine mesh — see header (matches sibling rationale)

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream (verbatim discipline of
//    the landed native_boolean_fuzz / native_step_import_fuzz). Keyed ONLY by an explicit
//    uint64 seed. No clock, no rand(): same seed → byte-identical batch. ───────────────
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

enum Family { F_L2_FRUSTUM, F_LN_STACK, F_L2_MISMATCH, F_SW_PRISM,
              F_L_NONPLANAR, F_SW_NONPLANAR, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_L2_FRUSTUM:   return "loft2 frustum(planar)";
    case F_LN_STACK:     return "loftN prismatoid-stack";
    case F_L2_MISMATCH:  return "loft2 mismatched-count";
    case F_SW_PRISM:     return "sweep straight-prism";
    case F_L_NONPLANAR:  return "loft non-planar[DECLINE]";
    case F_SW_NONPLANAR: return "sweep non-planar[DECLINE]";
  }
  return "?";
}
// CORE families demand a valid OCCT oracle; the two DECLINE-exercisers are out-of-scope
// inputs both engines may refuse (BOTH-DECLINED is fine for them, never for a core family).
bool isCoreFamily(int f) { return f == F_L2_FRUSTUM || f == F_LN_STACK ||
                                  f == F_L2_MISMATCH || f == F_SW_PRISM; }
enum Kind { K_LOFT, K_SWEEP };

// A generated case: the flat inputs handed to BOTH builders + a human-readable repro
// tuple + the CLOSED-FORM analytic volume/area (the independent ground-truth ARBITER).
struct GenCase {
  int family = 0;
  Kind kind = K_LOFT;
  // LOFT: sectionsXYZ packs `counts` flat (x,y,z) loops back to back.
  std::vector<double> sectionsXYZ;
  std::vector<int>    counts;
  // SWEEP: closed planar profile (x,y pairs) + a 3D polyline path (x,y,z triples).
  std::vector<double> profileXY;
  std::vector<double> pathXYZ;
  bool   analytic = false;   // is the closed-form ground truth available for this case?
  double aVol = 0, aArea = 0;
  std::string desc;
};

// ── geometry helpers ────────────────────────────────────────────────────────────────
double shoelaceArea(const double* xy, int n) {
  double a = 0.0;
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    a += xy[i * 2] * xy[j * 2 + 1] - xy[j * 2] * xy[i * 2 + 1];
  }
  return std::fabs(a) * 0.5;
}
double polyPerimeter(const double* xy, int n) {
  double p = 0.0;
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double dx = xy[j * 2] - xy[i * 2], dy = xy[j * 2 + 1] - xy[i * 2 + 1];
    p += std::sqrt(dx * dx + dy * dy);
  }
  return p;
}
// Append a coaxial regular n-gon (circumradius R, rotation rot, z) as (x,y,z) triples.
void appendRegularNgon3D(std::vector<double>& buf, int n, double R, double rot, double z) {
  for (int i = 0; i < n; ++i) {
    const double a = rot + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n);
    buf.push_back(R * std::cos(a));
    buf.push_back(R * std::sin(a));
    buf.push_back(z);
  }
}
// Regular n-gon area / edge / apothem at circumradius R (for the analytic prismatoid).
double ngonArea(int n, double R)    { return 0.5 * n * R * R * std::sin(2.0 * kPi / n); }
double ngonEdge(int n, double R)    { return 2.0 * R * std::sin(kPi / n); }
double ngonApothem(int n, double R) { return R * std::cos(kPi / n); }

// Exact volume/area of a stack of coaxial regular-n-gon frustums (a prismatoid stack).
void analyticNgonStack(int n, const std::vector<double>& R, const std::vector<double>& z,
                       double& vol, double& area) {
  vol = 0.0;
  double lateral = 0.0;
  for (std::size_t k = 0; k + 1 < R.size(); ++k) {
    const double dz = std::fabs(z[k + 1] - z[k]);
    const double Ak = ngonArea(n, R[k]), Ak1 = ngonArea(n, R[k + 1]);
    vol += (dz / 3.0) * (Ak + std::sqrt(Ak * Ak1) + Ak1);   // pyramidal-frustum band
    const double da = ngonApothem(n, R[k]) - ngonApothem(n, R[k + 1]);
    const double slant = std::sqrt(dz * dz + da * da);
    lateral += n * 0.5 * (ngonEdge(n, R[k]) + ngonEdge(n, R[k + 1])) * slant;
  }
  area = lateral + ngonArea(n, R.front()) + ngonArea(n, R.back());  // + first & last caps
}

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[224]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

int pickFamily(Rng& r) {
  // Core families weighted heavily; the two DECLINE-exercisers stay SPARSE (per the
  // scope note — they exist to hit the native NULL branch, not to manufacture DISAGREE).
  const int w[F_COUNT] = {5, 5, 4, 6, 1, 1};
  int tot = 0; for (int x : w) tot += x;
  int k = static_cast<int>(r.below(static_cast<uint32_t>(tot)));
  for (int i = 0; i < F_COUNT; ++i) { if (k < w[i]) return i; k -= w[i]; }
  return F_SW_PRISM;
}

GenCase genCase(Rng& r) {
  GenCase c; c.family = pickFamily(r);
  switch (c.family) {
    case F_L2_FRUSTUM: {
      c.kind = K_LOFT;
      const int n = 3 + static_cast<int>(r.below(6));   // 3..8-gon
      const double R0 = r.range(1.0, 3.0), R1 = r.range(1.0, 3.0);
      const double rot = r.range(0.0, kPi), dz = r.range(1.0, 4.0);
      appendRegularNgon3D(c.sectionsXYZ, n, R0, rot, 0.0);
      appendRegularNgon3D(c.sectionsXYZ, n, R1, rot, dz);
      c.counts = {n, n};
      analyticNgonStack(n, {R0, R1}, {0.0, dz}, c.aVol, c.aArea);
      c.analytic = true;
      c.desc = fmt("n=%.0f R0=%.4f R1=%.4f dz=%.4f", n, R0, R1, dz);
      break;
    }
    case F_LN_STACK: {
      // N-section (≥3) planar ruled loft. The native N-section path is EXACT for any
      // section geometry, but its self-verify only accepts candidates the tessellator welds
      // watertight — that is prisms (all sections congruent) and SYMMETRIC spools (the two
      // bands mirror one another so the shared-ring interior sampling matches). A stack whose
      // consecutive bands taper at DIFFERENT ratios T-junctions the shared ring → the engine
      // discards it → OCCT (an honest DECLINE, per native_loft_parity's header note). We
      // therefore draw mostly from the welding sub-families (real AGREE coverage) and leave a
      // fraction free to naturally exercise that N-section self-verify DECLINE.
      c.kind = K_LOFT;
      const int n = 3 + static_cast<int>(r.below(6));   // 3..8-gon
      const double rot = r.range(0.0, kPi), dz = r.range(1.0, 3.0);
      const double roll = r.unit();
      std::vector<double> R, z;
      if (roll < 0.5) {
        // Straight N-section PRISM (all sections congruent) — 3..5 parallel planes.
        const int ns = 3 + static_cast<int>(r.below(3));
        const double R0 = r.range(1.0, 3.0);
        for (int k = 0; k < ns; ++k) { R.push_back(R0); z.push_back(dz * k); }
        c.desc = fmt("PRISM n=%.0f ns=%.0f R=%.3f", n, ns, R0);
      } else if (roll < 0.8) {
        // 3-section SYMMETRIC spool Ra,Rb,Ra (mirror bands weld — cf. the 10→4→10 fixture).
        const double Ra = r.range(1.0, 3.0), Rb = r.range(1.0, 3.0);
        R = {Ra, Rb, Ra}; z = {0.0, dz, 2.0 * dz};
        c.desc = fmt("SYM-SPOOL n=%.0f Ra=%.3f Rb=%.3f", n, Ra, Rb);
      } else {
        // Free random stack (3..5 sections) — may honestly DECLINE if bands T-junction.
        const int ns = 3 + static_cast<int>(r.below(3));
        for (int k = 0; k < ns; ++k) { R.push_back(r.range(1.0, 3.0)); z.push_back(dz * k); }
        c.desc = fmt("FREE n=%.0f ns=%.0f R0=%.3f Rlast=%.3f", n, static_cast<double>(ns),
                     R.front(), R.back());
      }
      for (std::size_t k = 0; k < R.size(); ++k) appendRegularNgon3D(c.sectionsXYZ, n, R[k], rot, z[k]);
      c.counts.assign(R.size(), n);
      analyticNgonStack(n, R, z, c.aVol, c.aArea);
      c.analytic = true;
      break;
    }
    case F_L2_MISMATCH: {
      // Bottom = regular n-gon (n pts); top = the SAME regular n-gon OUTLINE at R1 but
      // sampled at 2n points (each edge midpoint inserted, collinear → outline unchanged).
      // The native T1 path (equalizeSectionCounts) makes the counts compatible; the OCCT
      // ThruSections resamples too. The analytic frustum of the two OUTLINES applies (the
      // collinear midpoints change neither volume nor area).
      c.kind = K_LOFT;
      const int n = 3 + static_cast<int>(r.below(4));   // 3..6-gon (2n stays modest)
      const double R0 = r.range(1.0, 3.0), R1 = r.range(1.0, 3.0);
      const double rot = r.range(0.0, kPi), dz = r.range(1.0, 4.0);
      appendRegularNgon3D(c.sectionsXYZ, n, R0, rot, 0.0);      // bottom: n pts
      for (int i = 0; i < n; ++i) {                            // top: 2n pts (mid-augmented)
        const double a0 = rot + 2.0 * kPi * i / n;
        const double a1 = rot + 2.0 * kPi * (i + 1) / n;
        c.sectionsXYZ.push_back(R1 * std::cos(a0));
        c.sectionsXYZ.push_back(R1 * std::sin(a0));
        c.sectionsXYZ.push_back(dz);
        const double mx = 0.5 * (R1 * std::cos(a0) + R1 * std::cos(a1));
        const double my = 0.5 * (R1 * std::sin(a0) + R1 * std::sin(a1));
        c.sectionsXYZ.push_back(mx);
        c.sectionsXYZ.push_back(my);
        c.sectionsXYZ.push_back(dz);
      }
      c.counts = {n, 2 * n};
      analyticNgonStack(n, {R0, R1}, {0.0, dz}, c.aVol, c.aArea);
      c.analytic = true;
      c.desc = fmt("n=%.0f->2n R0=%.4f R1=%.4f dz=%.4f", n, R0, R1, dz);
      break;
    }
    case F_SW_PRISM: {
      c.kind = K_SWEEP;
      const int n = 3 + static_cast<int>(r.below(5));   // 3..7-gon profile
      const double R = r.range(0.5, 2.0), rot = r.range(0.0, kPi);
      for (int i = 0; i < n; ++i) {
        const double a = rot + 2.0 * kPi * i / n;
        const double rr = R * r.range(0.85, 1.15);      // radius jitter → simple polygon, real variety
        c.profileXY.push_back(rr * std::cos(a));
        c.profileXY.push_back(rr * std::sin(a));
      }
      // Straight path: origin → a random 3D direction of random length.
      double dx = r.range(-1.0, 1.0), dy = r.range(-1.0, 1.0), dz = r.range(-1.0, 1.0);
      double nrm = std::sqrt(dx * dx + dy * dy + dz * dz);
      if (nrm < 1e-6) { dx = 1; dy = 0; dz = 0; nrm = 1; }
      const double L = r.range(3.0, 10.0);
      dx = dx / nrm * L; dy = dy / nrm * L; dz = dz / nrm * L;
      c.pathXYZ = {0, 0, 0, dx, dy, dz};
      const double pa = shoelaceArea(c.profileXY.data(), n);
      const double pp = polyPerimeter(c.profileXY.data(), n);
      c.aVol = pa * L;
      c.aArea = 2.0 * pa + pp * L;
      c.analytic = true;
      c.desc = fmt("n=%.0f R=%.3f L=%.3f", n, R, L);
      break;
    }
    case F_L_NONPLANAR: {
      // DECLINE-exerciser (sparse): a 3-section regular stack whose MIDDLE section has one
      // vertex lifted in z → non-planar → native build declines (NULL) → OCCT. Out of scope.
      c.kind = K_LOFT;
      const int n = 4;
      const double rot = r.range(0.0, kPi), R = r.range(1.5, 3.0);
      appendRegularNgon3D(c.sectionsXYZ, n, R, rot, 0.0);
      appendRegularNgon3D(c.sectionsXYZ, n, R, rot, 3.0);
      appendRegularNgon3D(c.sectionsXYZ, n, R, rot, 6.0);
      c.sectionsXYZ[n * 3 + 2] += r.range(1.0, 2.0);   // lift one middle vertex out of plane
      c.counts = {n, n, n};
      c.analytic = false;
      c.desc = fmt("n=%.0f R=%.3f (middle-section lifted)", n, R);
      break;
    }
    case F_SW_NONPLANAR: {
      // DECLINE-exerciser (sparse): a NON-PLANAR 3D spine → native build_sweep returns NULL
      // (non-planar curve → OCCT corrected-Frenet). Out of scope for the native path.
      c.kind = K_SWEEP;
      const int n = 4;
      const double R = r.range(0.4, 0.9), rot = r.range(0.0, kPi);
      for (int i = 0; i < n; ++i) {
        const double a = rot + 2.0 * kPi * i / n;
        c.profileXY.push_back(R * std::cos(a));
        c.profileXY.push_back(R * std::sin(a));
      }
      // Four non-coplanar spine points (a helix-ish 3D polyline).
      const double s = r.range(2.0, 4.0);
      c.pathXYZ = {0, 0, 0,  s, 0, s,  s, s, 2 * s,  0, s, 3 * s};
      c.analytic = false;
      c.desc = fmt("n=%.0f R=%.3f (non-planar spine)", n, R);
      break;
    }
  }
  return c;
}

// ── native build + measurement (OCCT-FREE: native tessellator) ───────────────────────
ntopo::Shape buildNative(const GenCase& c) {
  if (c.kind == K_LOFT)
    return ncst::build_loft_sections(c.sectionsXYZ.data(), c.counts.data(),
                                     static_cast<int>(c.counts.size()));
  return ncst::build_sweep(c.profileXY.data(), static_cast<int>(c.profileXY.size() / 2),
                           c.pathXYZ.data(), static_cast<int>(c.pathXYZ.size() / 3));
}

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

// ── OCCT oracle build (the SAME construction the cc_* OCCT engine uses) ───────────────
// Loft: BRepOffsetAPI_ThruSections(solid, ruled) over the section polygons (occt_construct
// solid_loft_sections). Sweep: BRepOffsetAPI_MakePipe of the profile FACE along the spine
// polyline, the profile centred on its centroid in the start-tangent frame (occt_construct
// solid_sweep). Both wrapped so a genuine OCCT failure yields a null shape (never a crash).
TopoDS_Shape buildOcctLoft(const GenCase& c) {
  try {
    BRepOffsetAPI_ThruSections gen(Standard_True /*solid*/, Standard_True /*ruled*/);
    std::size_t off = 0;
    for (std::size_t k = 0; k < c.counts.size(); ++k) {
      const int cnt = c.counts[k];
      BRepBuilderAPI_MakePolygon poly;
      for (int i = 0; i < cnt; ++i)
        poly.Add(gp_Pnt(c.sectionsXYZ[off + i * 3], c.sectionsXYZ[off + i * 3 + 1],
                        c.sectionsXYZ[off + i * 3 + 2]));
      poly.Close();
      if (!poly.IsDone()) return {};
      gen.AddWire(poly.Wire());
      off += static_cast<std::size_t>(cnt) * 3;
    }
    gen.Build();
    if (!gen.IsDone()) return {};
    return gen.Shape();
  } catch (const Standard_Failure&) { return {}; }
}

TopoDS_Shape buildOcctSweep(const GenCase& c) {
  try {
    const int profileCount = static_cast<int>(c.profileXY.size() / 2);
    double cx = 0, cy = 0;
    for (int i = 0; i < profileCount; ++i) { cx += c.profileXY[i * 2]; cy += c.profileXY[i * 2 + 1]; }
    cx /= profileCount; cy /= profileCount;
    const gp_Pnt p0(c.pathXYZ[0], c.pathXYZ[1], c.pathXYZ[2]);
    const gp_Pnt p1(c.pathXYZ[3], c.pathXYZ[4], c.pathXYZ[5]);
    gp_Vec tan(p0, p1);
    if (tan.Magnitude() < 1.0e-9) return {};
    tan.Normalize();
    const gp_Vec ref = (std::fabs(tan.Y()) > 0.9) ? gp_Vec(1, 0, 0) : gp_Vec(0, 1, 0);
    gp_Vec nrm = tan.Crossed(ref);
    if (nrm.Magnitude() < 1.0e-6) nrm = gp_Vec(1, 0, 0); else nrm.Normalize();
    gp_Vec up = nrm.Crossed(tan); up.Normalize();
    BRepBuilderAPI_MakePolygon poly;
    for (int i = 0; i < profileCount; ++i) {
      const double u = c.profileXY[i * 2] - cx, v = c.profileXY[i * 2 + 1] - cy;
      poly.Add(gp_Pnt(p0.X() + u * nrm.X() + v * up.X(), p0.Y() + u * nrm.Y() + v * up.Y(),
                      p0.Z() + u * nrm.Z() + v * up.Z()));
    }
    poly.Close();
    if (!poly.IsDone()) return {};
    BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
    if (!face.IsDone()) return {};
    BRepBuilderAPI_MakePolygon spine;
    for (std::size_t i = 0; i < c.pathXYZ.size() / 3; ++i)
      spine.Add(gp_Pnt(c.pathXYZ[i * 3], c.pathXYZ[i * 3 + 1], c.pathXYZ[i * 3 + 2]));
    if (!spine.IsDone()) return {};
    BRepOffsetAPI_MakePipe pipe(spine.Wire(), face.Face());
    pipe.Build();
    if (!pipe.IsDone()) return {};
    return pipe.Shape();
  } catch (const Standard_Failure&) { return {}; }
}

struct OcctMeasure { bool present = false, valid = false, closedShell = false; double volume = 0, area = 0; int solids = 0; };
OcctMeasure measureOcct(const TopoDS_Shape& s) {
  OcctMeasure m;
  if (s.IsNull()) return m;
  m.present = true;
  try {
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
  } catch (const Standard_Failure&) { m.valid = false; m.closedShell = false; }
  return m;
}

double relDiff(double a, double b) { return (b > 1e-12) ? std::fabs(a - b) / b : 1e30; }

// ── the classifier (mirrors the M6/M6b siblings) ─────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famAgreed[F_COUNT] = {0}, g_famDeclined[F_COUNT] = {0}, g_famDisagreed[F_COUNT] = {0};
int g_famOracleInacc[F_COUNT] = {0}, g_famBothDecl[F_COUNT] = {0};

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

  std::printf("== M6c differential-fuzz: native CONSTRUCTION (loft/sweep) vs OCCT ThruSections/MakePipe ==\n");
  std::printf("== seed=0x%llx N=%d relTol=%.0e deflection=%g ==\n",
              static_cast<unsigned long long>(seed), N, kRelTol, kDeflection);
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    const GenCase c = genCase(rng);

    // (1) NATIVE builder (OCCT-FREE). NULL / non-watertight → the engine's mandatory
    //     self-verify would DISCARD it → OCCT: an honest DECLINE (never a bar failure).
    const ntopo::Shape natShape = buildNative(c);
    const NativeMeasure nat = measureNative(natShape);

    // (2) ORACLE — OCCT builds the SAME input.
    const TopoDS_Shape occShape = (c.kind == K_LOFT) ? buildOcctLoft(c) : buildOcctSweep(c);
    const OcctMeasure occ = measureOcct(occShape);
    const bool oracleSolid = occ.present && occ.valid && occ.closedShell &&
                             occ.volume > 1e-9 && occ.area > 1e-9 && occ.solids >= 1;

    const bool nativeUsable = nat.present && nat.watertight && nat.volume > 1e-9;

    // ── classify ───────────────────────────────────────────────────────────────────
    Verdict v;
    if (!nativeUsable) {
      // Native honestly declined (NULL / non-watertight candidate). If OCCT is a valid
      // closed solid it ships (DECLINED). If OCCT ALSO refused: for a CORE family that is
      // an untrustworthy oracle (fails the bar); for a DECLINE-exerciser it just means
      // both engines refuse an out-of-scope input (BOTH-DECLINED, fine).
      if (oracleSolid) v = DECLINED;
      else v = isCoreFamily(c.family) ? ORACLE_UNRELIABLE : BOTH_DECLINED;
    } else if (!oracleSolid) {
      // Native built a watertight solid but OCCT did not → the oracle is untrustworthy for
      // this input; excluded from the verdict and fails the bar (investigate, never launder).
      v = ORACLE_UNRELIABLE;
    } else {
      const double volRel = relDiff(nat.volume, occ.volume);
      const double areaRel = relDiff(nat.area, occ.area);
      if (volRel < kRelTol && areaRel < kRelTol && nat.solids == occ.solids) {
        v = AGREED;
      } else if (c.analytic) {
        // Native watertight but DIFFERS from OCCT. Attribute the disagreement with the
        // independent closed-form ground truth: a native result that MATCHES exact math
        // while OCCT does not is CORRECT (an oracle-side inaccuracy); only a native result
        // that fails the analytic truth is a genuine SILENT WRONG build.
        const double natVsA_V = relDiff(nat.volume, c.aVol), natVsA_A = relDiff(nat.area, c.aArea);
        const double occVsA_V = relDiff(occ.volume, c.aVol), occVsA_A = relDiff(occ.area, c.aArea);
        const bool natMatchesA = natVsA_V < kRelTol && natVsA_A < kRelTol;
        const bool occMatchesA = occVsA_V < kRelTol && occVsA_A < kRelTol;
        v = (natMatchesA && !occMatchesA) ? ORACLE_INACCURATE : DISAGREED;
      } else {
        v = DISAGREED;  // no arbiter → a native≠OCCT difference is an unattributable wrong
      }
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
      std::printf("[FUZZ] AGREED    case=%d %-26s volN=%.6g volO=%.6g dV=%.2e areaN=%.6g areaO=%.6g dA=%.2e solids=%d/%d\n",
                  i, famName(c.family), nat.volume, occ.volume, relDiff(nat.volume, occ.volume),
                  nat.area, occ.area, relDiff(nat.area, occ.area), nat.solids, occ.solids);
    } else if (v == DECLINED) {
      std::printf("[FUZZ] DECLINED  case=%d %-26s native=%s -> OCCT[valid=%d closed=%d volO=%.6g areaO=%.6g solids=%d]  %s\n",
                  i, famName(c.family), nat.present ? "non-watertight" : "NULL",
                  occ.valid, occ.closedShell, occ.volume, occ.area, occ.solids, c.desc.c_str());
    } else if (v == BOTH_DECLINED) {
      std::printf("[FUZZ] BOTH-DECL case=%d %-26s native declined AND OCCT built no valid solid (out-of-scope input)  %s\n",
                  i, famName(c.family), c.desc.c_str());
    } else if (v == ORACLE_INACCURATE) {
      const double natVsA_V = relDiff(nat.volume, c.aVol), natVsA_A = relDiff(nat.area, c.aArea);
      const double occVsA_V = relDiff(occ.volume, c.aVol), occVsA_A = relDiff(occ.area, c.aArea);
      std::printf("[FUZZ] ORACLE_INACCURATE case=%d %-26s native MATCHES analytic, OCCT does NOT "
                  "volN=%.6g volO=%.6g aVol=%.6g natVsA_V=%.2e occVsA_V=%.2e  areaN=%.6g areaO=%.6g aArea=%.6g natVsA_A=%.2e occVsA_A=%.2e\n"
                  "       NOTE seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nat.volume, occ.volume, c.aVol, natVsA_V, occVsA_V,
                  nat.area, occ.area, c.aArea, natVsA_A, occVsA_A,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
    } else if (v == ORACLE_UNRELIABLE) {
      std::printf("[FUZZ] ORACLE_UNRELIABLE case=%d %-26s core-family oracle not a valid closed solid "
                  "[nativeUsable=%d occValid=%d occClosed=%d occSolids=%d volO=%.6g areaO=%.6g]\n"
                  "       REPRO seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nativeUsable ? 1 : 0, occ.valid, occ.closedShell, occ.solids,
                  occ.volume, occ.area, static_cast<unsigned long long>(seed), i, c.desc.c_str());
    } else {  // DISAGREED — native watertight but fails the analytic ground truth
      const double natVsA_V = c.analytic ? relDiff(nat.volume, c.aVol) : -1.0;
      const double natVsA_A = c.analytic ? relDiff(nat.area, c.aArea) : -1.0;
      std::printf("[FUZZ] DISAGREED case=%d %-26s SILENT-WRONG-BUILD "
                  "volN=%.6g volO=%.6g aVol=%.6g dV(occt)=%.3e dV(analytic)=%.3e  areaN=%.6g areaO=%.6g aArea=%.6g dA(analytic)=%.3e solidsN=%d solidsO=%d\n"
                  "       REPRO seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nat.volume, occ.volume, c.aVol,
                  relDiff(nat.volume, occ.volume), natVsA_V,
                  nat.area, occ.area, c.aArea, natVsA_A, nat.solids, occ.solids,
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
    std::printf("     %-28s %d/%d/%d/%d/%d\n", famName(f), g_famAgreed[f], g_famDeclined[f],
                g_famDisagreed[f], g_famOracleInacc[f], g_famBothDecl[f]);
  }
  if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by exact math vs OCCT — oracle-side limitation, logged, NOT a native fault)\n", g_oracleInacc);
  if (g_bothDecl)    std::printf("   BOTH-DECLINED=%d (out-of-scope input both engines refuse — no wrong result, logged)\n", g_bothDecl);
  if (g_oracleBad)   std::printf("   ORACLE_UNRELIABLE=%d (core-family OCCT build not a valid solid — investigate)\n", g_oracleBad);

  const bool bar = (g_disagreed == 0 && g_oracleBad == 0);
  std::printf("== M6c BAR: %s (DISAGREED=%d must be 0) ==\n",
              bar ? "PASS — zero silent wrong builds" : "FAIL", g_disagreed);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
