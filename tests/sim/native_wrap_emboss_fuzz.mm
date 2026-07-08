// SPDX-License-Identifier: Apache-2.0
//
// native_wrap_emboss_fuzz.mm — MOAT M6-breadth-5 (the COMPLETENESS BAR, FIFTH domain):
// a WRAP-EMBOSS differential-fuzzing harness (iOS simulator) for the native OCCT-FREE
// wrap-emboss builder src/native/feature/wrap_emboss.h — a rectangular PAD (emboss,
// material ADDED), a rectangular DEBOSS pocket, and a NON-RECTANGULAR polygon
// emboss/deboss wrapped onto a CYLINDER lateral face.
//
// This extends the four landed differential fuzzers native_boolean_fuzz.mm (curved
// boolean), native_step_import_fuzz.mm (STEP round-trip), native_construct_fuzz.mm
// (loft/sweep) and native_blend_fuzz.mm (fillet/chamfer) to a FIFTH independent native
// domain — the OCCT-FREE feature library src/native/feature. Like its siblings it is
// INFRASTRUCTURE (a test harness, not a geometry capability): the ORACLE is authoritative,
// the bar is ZERO SILENT WRONG WRAP-EMBOSS results over a seeded batch, and an HONEST
// DECLINE is first-class.
//
// ── THE DIFFERENTIAL (native builder vs oracle on the SAME input) ────────────────────
// The discipline mirrors the landed native_construct_fuzz / native_blend_fuzz: it calls
// the OCCT-FREE native builder DIRECTLY (cybercad::native::feature::wrap_emboss), so a
// NULL / non-watertight result is an UNAMBIGUOUS native DECLINE (the shipping engine's
// mandatory self-verify would DISCARD it and forward to OCCT), and the oracle is computed
// SEPARATELY. Per trial:
//   (1) DETERMINISTICALLY generate a random-but-VALID wrap-emboss input from the families
//       the native path CLAIMS (see SCOPE):
//         EMBOSS-RECT  — a centred axis-aligned rectangle raised by `height`  (pad)
//         DEBOSS-RECT  — a centred axis-aligned rectangle recessed by `depth` (pocket)
//         EMBOSS-POLY  — a convex N-gon (n=3..7, jittered) raised by `height`
//         DEBOSS-POLY  — a convex N-gon recessed by `depth`
//       plus SPARSE out-of-scope DECLINE-exercisers (a NON-cylindrical base, a >2π
//       footprint, a deboss depth ≥ R, a self-intersecting polygon) that the native
//       builder honestly returns NULL for. The RNG is a splitmix64-seeded xoshiro256**
//       stream keyed ONLY by an explicit FUZZ_SEED (argv/env) — NO clock, NO rand():
//       same seed → byte-identical batch.
//   (2) BUILD + MEASURE the input:
//         native : construct::build_prism_profile (OCCT-FREE full-circle cylinder) →
//                  feature::wrap_emboss (the native facet-soup builder, welded watertight),
//                  measured by the native tessellator (mesh vol/area/watertight/solids).
//         oracle : (PRIMARY) the CLOSED-FORM curvature-corrected added/removed volume, and
//                  (SECONDARY, rectangle families only, where clean) an OCCT-boolean
//                  reconstruction of the SAME solid (base cylinder FUSED / CUT with a
//                  wrapped shell wedge), measured exactly by BRepGProp.
//   (3) CLASSIFY each trial into EXACTLY ONE bucket (identical scheme to the siblings):
//         AGREED            — native watertight AND its volume matches the CLOSED-FORM
//                             ground truth within a FIXED relTol (and, for the rectangle
//                             families, its volume + area match the OCCT reconstruction).
//         HONESTLY-DECLINED — native returns NULL / a non-watertight candidate on an
//                             IN-SCOPE input (self-verify discards it → OCCT ships): a real,
//                             logged, first-class outcome.
//         DISAGREED         — native watertight but its volume/area does NOT match the
//                             CLOSED-FORM ground truth (and the OCCT reconstruction where
//                             present). A genuine SILENT WRONG result — the failure this
//                             harness exists to catch.
//         ORACLE-INACCURATE — native watertight, MATCHES the closed-form ground truth, but
//                             the OCCT-boolean reconstruction does NOT: the native result is
//                             CORRECT and is VINDICATED by exact math; OCCT is the outlier.
//         BOTH-DECLINED     — an out-of-scope DECLINE-exerciser where native returned NULL:
//                             neither engine produced a wrong result. Logged, NOT a failure.
//   (4) Print a coverage summary. Exit 0 IFF DISAGREED == 0. Any DISAGREE / ORACLE-
//       INACCURATE prints seed + case index + family/param tuple + all measurements.
//
// ── ANALYTIC GROUND-TRUTH ARBITER (why OCCT alone is not the oracle) ─────────────────
// OCCT has NO single wrap-emboss API, so — as in the construction / blend fuzzers — the
// PRIMARY oracle is the CLOSED-FORM changed volume. The native map is u = px/R (arc-length
// px → angle), v = py + vMid. A footprint of flat (shoelace) area A therefore covers a
// (u,v) measure A/R, and the pad is the radial shell over that measure between R and the
// target radius, so the EXACT changed volume is (curvature-corrected — NOT the flat A·h):
//     emboss height h :  +ΔV,  ΔV = A · ((R+h)² − R²) / (2R)
//     deboss depth  d :  −ΔV,  ΔV = A · (R² − (R−d)²) / (2R)
// This closed form is UNIVERSAL across rectangle AND polygon footprints (it depends only on
// the (u,v) measure A/R), so it arbitrates every core family. The SECONDARY OCCT oracle
// reconstructs the SAME solid by a boolean of the base cylinder with a wrapped shell wedge —
// CLEAN only for the rectangle families (a rectangle footprint wraps to an exact angular
// sector); reconstructing a wrapped POLYGON pad in OCCT would re-implement the feature
// (arcs → its own faceting), so it is honestly DECLINED for the polygon families, whose
// oracle is the exact closed form alone. Where the OCCT reconstruction IS present it both
// cross-checks the closed form and supplies the only independent AREA oracle (rectangles).
//
// ── DEFLECTION-BOUNDED, FIXED TOLERANCE (honest — see the OpenSpec change) ───────────
// The native builder facets the whole embossed/debossed cylinder into planar triangles at a
// fine, FIXED `deflection`, so the native solid is an INSCRIBED polygonal approximation whose
// volume/area sit a small, deflection-bounded amount below the smooth closed-form / OCCT
// solid. That bias is kept FAR under the FIXED tolerance (kVolRelTol / kAreaRelTol, NEVER
// widened per-trial) by a fine faceting deflection and BOUNDED cylinder radii (so the angular
// facet count is never capped); the measured max bias is logged in the summary so the margin
// is auditable. The differential's sensitivity to the PAD is bounded by that faceting bias
// relative to the whole solid — so the harness catches gross wrong-volume / broken-topology /
// wrong-area pads (and every non-watertight failure), the defects this bar exists to catch.
// This is the SAME deflection-bounded discipline the curated wrap-emboss parity harness
// (native_wrap_emboss_parity.mm) uses, proven there at the same tolerance.
//
// ── SCOPE (honest, bounded — logged exclusions) ────────────────────────────────────
// Native ONLY for a CYLINDER lateral face with a rectangular or simple-polygon footprint
// that fits on the wall (arc span < 2π, axial span strictly inside the ends) and, for a
// deboss, depth < R. The native path also accepts a NON-CONVEX polygon only if its self-
// verify passes; this harness generates CONVEX N-gons for the AGREE families (a reliable,
// oracle-backed input) and leaves dense / non-convex / star loops to the native DECLINE
// branch — a first-class honest DOMAIN-level decline noted here and in the OpenSpec change,
// no coverage silently dropped. Out-of-scope inputs (non-cylindrical base, >2π footprint,
// depth ≥ R, self-intersecting loop) are generated SPARINGLY to exercise the native DECLINE
// branch, NOT to manufacture DISAGREE.
//
// This TU is OCCT-dependent (BRepPrimAPI_MakeCylinder + BRepAlgoAPI_Fuse/Cut + BRepGProp
// for the rectangle reconstruction) but needs NO numsci: the native feature + construct +
// tessellate + topology + boolean + math live in src/native/** — all OCCT-FREE and header-
// only (math bezier/bspline are the only compiled native TUs). Built ONLY by
// scripts/run-sim-native-wrap-emboss-fuzz.sh; on run-sim-suite.sh's SKIP list (own main()).
// src/native stays OCCT-FREE — this harness is additive test/sim code only. Flushes and
// std::_Exit (OCCT static teardown in the trimmed static build is not exit-clean — same
// rationale as the siblings).
//
#include "native/construct/native_construct.h"     // build_prism / build_prism_profile
#include "native/feature/wrap_emboss.h"             // feature::wrap_emboss (the SUT)
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
#error "native_wrap_emboss_fuzz requires -DCYBERCAD_HAS_OCCT and the OCCT boolean oracle"
#endif

#include <gp_Ax1.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <Standard_Failure.hxx>

namespace ncst  = cybercad::native::construct;
namespace nfeat = cybercad::native::feature;
namespace ntess = cybercad::native::tessellate;
namespace ntopo = cybercad::native::topology;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kVolRelTol = 2e-2;    // FIXED volume agreement bar (never widened)
constexpr double kAreaRelTol = 3e-2;   // FIXED area agreement bar (never widened)
constexpr double kWeDefl = 0.002;      // native faceting deflection (keeps inscribed bias tiny)
constexpr double kTessDefl = 0.002;    // tessellation of the (already-planar) facet soup

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream (verbatim discipline of the
//    landed native_boolean_fuzz / native_construct_fuzz / native_blend_fuzz). Keyed ONLY by
//    an explicit uint64 seed. No clock, no rand(): same seed → byte-identical batch. ───────
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

enum Family {
  F_EMBOSS_RECT, F_DEBOSS_RECT, F_EMBOSS_POLY, F_DEBOSS_POLY,
  F_DECL_NONCYL /*out-of-scope*/, F_DECL_OVER2PI, F_DECL_DEEP, F_DECL_SELFX, F_COUNT
};
const char* famName(int f) {
  switch (f) {
    case F_EMBOSS_RECT:  return "emboss rect (pad)";
    case F_DEBOSS_RECT:  return "deboss rect (pocket)";
    case F_EMBOSS_POLY:  return "emboss poly (N-gon)";
    case F_DEBOSS_POLY:  return "deboss poly (N-gon)";
    case F_DECL_NONCYL:  return "non-cylinder base[DECLINE]";
    case F_DECL_OVER2PI: return "footprint >2pi[DECLINE]";
    case F_DECL_DEEP:    return "deboss depth>=R[DECLINE]";
    case F_DECL_SELFX:   return "self-intersecting[DECLINE]";
  }
  return "?";
}
bool isCoreFamily(int f) { return f <= F_DEBOSS_POLY; }
bool isRectFamily(int f) { return f == F_EMBOSS_RECT || f == F_DEBOSS_RECT; }
bool isEmboss(int f) { return f == F_EMBOSS_RECT || f == F_EMBOSS_POLY; }

// A generated case: the wrap-emboss inputs handed to the native builder + the OCCT
// reconstruction + a repro tuple + the CLOSED-FORM ground truth (the independent arbiter).
struct GenCase {
  int family = 0;
  double Rc = 0, H = 0;         // capped cylinder radius + height (axis +Z, z in [0,H])
  double amount = 0;            // emboss height / deboss depth
  int boss = 1;                 // 1 emboss, 0 deboss
  std::vector<double> prof;     // (px,py) closed footprint loop
  int count = 0;                // number of (px,py) pairs
  double vMin = 0, vMax = 0;    // wrapped axial band (world z) of the footprint bbox
  double dU = 0;                // wrapped angular span of the footprint bbox (rad)
  bool analytic = false;        // in-scope: a valid closed-form ground truth exists
  double baseVol = 0;           // exact smooth base cylinder volume (πR²H)
  double dVol = 0;              // exact closed-form changed volume (signed magnitude)
  double expVol = 0;            // exact expected total volume (baseVol ± dVol)
  bool nonCyl = false;          // build a BOX base instead of a cylinder (decline-exerciser)
  std::string desc;
};

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[224]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

// Twice-signed shoelace / abs area of a closed (px,py) loop.
double shoelaceArea(const std::vector<double>& p, int count) {
  double a2 = 0.0;
  for (int i = 0; i < count; ++i) {
    const int j = (i + 1) % count;
    a2 += p[i * 2] * p[j * 2 + 1] - p[j * 2] * p[i * 2 + 1];
  }
  return std::fabs(a2) * 0.5;
}

// A centred axis-aligned rectangle (arc-length width aw × axial height ah), CCW.
std::vector<double> rectProfile(double aw, double ah) {
  return {-aw / 2, -ah / 2,  aw / 2, -ah / 2,  aw / 2, ah / 2,  -aw / 2, ah / 2};
}
// A convex N-gon (circumradius a, per-vertex radial jitter), CCW, centred at the origin.
std::vector<double> ngonProfile(Rng& r, int n, double a) {
  std::vector<double> p;
  p.reserve(static_cast<std::size_t>(2 * n));
  for (int i = 0; i < n; ++i) {
    const double ang = 2.0 * kPi * i / n;
    const double rad = a * r.range(0.85, 1.0);   // small jitter keeps the loop convex
    p.push_back(rad * std::cos(ang));
    p.push_back(rad * std::sin(ang));
  }
  return p;
}

// The closed-form curvature-corrected changed volume for a footprint of flat area A raised
// by +amount (emboss) or recessed by amount (deboss) on a cylinder of radius R.
double changedVolume(double A, double R, double amount, bool emboss) {
  const double rT = emboss ? (R + amount) : (R - amount);
  return A * std::fabs(rT * rT - R * R) / (2.0 * R);
}

int pickFamily(Rng& r) {
  // Core families weighted heavily; the DECLINE-exercisers stay SPARSE (they exist to hit
  // the native NULL branch, not to manufacture DISAGREE).
  const int w[F_COUNT] = {6, 6, 6, 6, 1, 1, 1, 1};
  int tot = 0; for (int x : w) tot += x;
  int k = static_cast<int>(r.below(static_cast<uint32_t>(tot)));
  for (int i = 0; i < F_COUNT; ++i) { if (k < w[i]) return i; k -= w[i]; }
  return F_EMBOSS_RECT;
}

// Fill the wrapped (vMin,vMax,dU) band + the closed-form ground truth for an in-scope case.
void fillAnalytic(GenCase& c) {
  const double vMid = 0.5 * c.H;
  double pxMin = c.prof[0], pxMax = c.prof[0], pyMin = c.prof[1], pyMax = c.prof[1];
  for (int i = 0; i < c.count; ++i) {
    pxMin = std::min(pxMin, c.prof[i * 2]);     pxMax = std::max(pxMax, c.prof[i * 2]);
    pyMin = std::min(pyMin, c.prof[i * 2 + 1]); pyMax = std::max(pyMax, c.prof[i * 2 + 1]);
  }
  c.vMin = pyMin + vMid; c.vMax = pyMax + vMid;
  c.dU = (pxMax - pxMin) / c.Rc;
  const double A = shoelaceArea(c.prof, c.count);
  c.baseVol = kPi * c.Rc * c.Rc * c.H;
  c.dVol = changedVolume(A, c.Rc, c.amount, isEmboss(c.family));
  c.expVol = c.baseVol + (c.boss == 1 ? 1.0 : -1.0) * c.dVol;
  c.analytic = true;
}

GenCase genCase(Rng& r) {
  GenCase c; c.family = pickFamily(r);
  c.Rc = r.range(4.0, 12.0);
  c.H = r.range(3.0 * c.Rc, 4.0 * c.Rc);
  switch (c.family) {
    case F_EMBOSS_RECT:
    case F_DEBOSS_RECT: {
      c.boss = (c.family == F_EMBOSS_RECT) ? 1 : 0;
      const double aw = r.range(0.6 * c.Rc, 1.6 * c.Rc);   // arc-length width, Δu = aw/Rc < 2π
      const double ah = r.range(0.25 * c.H, 0.5 * c.H);    // axial span, strictly inside wall
      c.prof = rectProfile(aw, ah); c.count = 4;
      c.amount = c.boss ? r.range(0.15 * c.Rc, 0.5 * c.Rc)
                        : r.range(0.15 * c.Rc, 0.6 * c.Rc);  // deboss depth < R
      fillAnalytic(c);
      c.desc = fmt("Rc=%.3f H=%.3f aw=%.3f ah=%.3f", c.Rc, c.H, aw, ah) +
               fmt(" amt=%.3f dU=%.3f", c.amount, c.dU);
      break;
    }
    case F_EMBOSS_POLY:
    case F_DEBOSS_POLY: {
      c.boss = (c.family == F_EMBOSS_POLY) ? 1 : 0;
      const int n = 3 + static_cast<int>(r.below(5));      // 3..7
      // Bound the circumradius (angular span) + amount: a polygon cap is an ear-clipped
      // inscribed facet fan whose few-triangle interior (a single triangle for n=3) sets a
      // deflection-INDEPENDENT inscribing floor that grows with the pad's arc span and
      // radius ratio. Keeping the footprint MODEST holds that floor comfortably under the
      // FIXED tol — the SAME "bounded feature size so the facet bias stays tiny" discipline
      // the blend fuzz applies to its rim/edge size. (Rectangle pads are sagitta-tiled and
      // have an exact OCCT sector oracle, so they stay generous.)
      const double a = r.range(0.3 * c.Rc, std::min(0.6 * c.Rc, 0.18 * c.H));
      c.prof = ngonProfile(r, n, a); c.count = n;
      c.amount = c.boss ? r.range(0.15 * c.Rc, 0.35 * c.Rc)
                        : r.range(0.15 * c.Rc, 0.4 * c.Rc);
      fillAnalytic(c);
      c.desc = fmt("Rc=%.3f H=%.3f n=%.0f a=%.3f", c.Rc, c.H, static_cast<double>(n), a) +
               fmt(" amt=%.3f dU=%.3f", c.amount, c.dU);
      break;
    }
    case F_DECL_NONCYL: {   // a BOX base — the picked face is planar → native declines.
      c.nonCyl = true; c.boss = 1;
      c.prof = rectProfile(0.6 * c.Rc, 0.3 * c.H); c.count = 4;
      c.amount = 0.3 * c.Rc;
      c.desc = fmt("box base Rc=%.3f H=%.3f", c.Rc, c.H);
      break;
    }
    case F_DECL_OVER2PI: {  // arc span ≥ full turn → native guard returns NULL.
      c.boss = 1;
      const double aw = r.range(2.2 * kPi * c.Rc, 2.8 * kPi * c.Rc);  // Δu ≥ 2π
      c.prof = rectProfile(aw, 0.3 * c.H); c.count = 4;
      c.amount = 0.3 * c.Rc;
      c.desc = fmt("Rc=%.3f aw=%.3f (Δu>2π)", c.Rc, aw);
      break;
    }
    case F_DECL_DEEP: {     // deboss depth ≥ R → rFloor ≤ 0 guard → NULL.
      c.boss = 0;
      c.prof = rectProfile(0.6 * c.Rc, 0.3 * c.H); c.count = 4;
      c.amount = r.range(1.05 * c.Rc, 1.5 * c.Rc);
      c.desc = fmt("Rc=%.3f depth=%.3f (>=R)", c.Rc, c.amount);
      break;
    }
    case F_DECL_SELFX: {    // a self-intersecting NON-rectangular loop → polyFootprint rejects.
      // A pentagram {5/2}: connecting 5 rim points in 144° steps crosses itself. (A crossed
      // QUAD would be recovered as its axis-aligned bounding rectangle by rectFootprint —
      // whose 4 corners sit at the bbox extremes — so it must be a count≠4 star.)
      c.boss = 1;
      const double a = 0.5 * c.Rc;
      for (int k = 0; k < 5; ++k) {
        const double ang = 2.0 * kPi * (2.0 * k) / 5.0;   // 144° step → star polygon
        c.prof.push_back(a * std::cos(ang));
        c.prof.push_back(a * std::sin(ang));
      }
      c.count = 5;
      c.amount = 0.3 * c.Rc;
      c.desc = fmt("Rc=%.3f pentagram a=%.3f", c.Rc, a);
      break;
    }
  }
  return c;
}

// ── native input builders (OCCT-FREE) ────────────────────────────────────────────────
// A capped solid cylinder about +Z (radius Rc, z in [0,H]) — a native body with ONE
// Cylinder wall face (the SAME body cc_solid_extrude_profile builds).
ntopo::Shape buildNativeCylinder(double Rc, double H) {
  ncst::ProfileSegment seg; seg.kind = 2; seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return ncst::build_prism_profile({seg}, {}, {}, H);
}
// An axis-aligned box [0,w]×[0,d]×[0,h] via the native polygon prism builder.
ntopo::Shape buildNativeBox(double w, double d, double h) {
  const double rect[8] = {0, 0,  w, 0,  w, d,  0, d};
  return ncst::build_prism(rect, 4, h);
}
// The 1-based face id of the Cylinder lateral face (scan surfaceOf for Kind::Cylinder).
int findCylFace(const ntopo::Shape& solid) {
  const ntopo::ShapeMap fmap = ntopo::mapShapes(solid, ntopo::ShapeType::Face);
  for (std::size_t i = 1; i <= fmap.size(); ++i) {
    const auto surf = ntopo::surfaceOf(fmap.shape(static_cast<int>(i)));
    if (surf && surf->surface->kind == ntopo::FaceSurface::Kind::Cylinder)
      return static_cast<int>(i);
  }
  return 1;  // fallback: a planar face (used only by the non-cylinder decline-exerciser)
}

struct NativeMeasure { bool present = false, watertight = false; double volume = 0, area = 0; int solids = 0; };
NativeMeasure measureNative(const ntopo::Shape& s) {
  NativeMeasure m;
  if (s.isNull()) return m;
  m.present = true;
  for (ntopo::Explorer e(s, ntopo::ShapeType::Solid); e.more(); e.next()) ++m.solids;
  ntess::MeshParams p; p.deflection = kTessDefl;
  const ntess::Mesh mesh = ntess::SolidMesher{p}.mesh(s);
  m.watertight = ntess::isWatertight(mesh);
  m.volume = std::fabs(ntess::enclosedVolume(mesh));
  m.area = ntess::surfaceArea(mesh);
  return m;
}

// ── OCCT SECONDARY oracle: reconstruct the RECTANGLE embossed / debossed solid by a
//    boolean of the base cylinder with a wrapped shell wedge, measured by BRepGProp.
//    Rectangle footprints ONLY (a rectangle wraps to an EXACT angular sector); the polygon
//    families have no clean OCCT reconstruction and rely on the closed form alone. ────────
struct OcctMeasure { bool present = false, valid = false; double volume = 0, area = 0; int solids = 0; };
OcctMeasure measureOcct(const TopoDS_Shape& s) {
  OcctMeasure m;
  if (s.IsNull()) return m;
  m.present = true;
  try {
    BRepCheck_Analyzer an(s);
    m.valid = an.IsValid();
    for (TopExp_Explorer ex(s, TopAbs_SOLID); ex.More(); ex.Next()) ++m.solids;
    GProp_GProps vg; BRepGProp::VolumeProperties(s, vg); m.volume = std::fabs(vg.Mass());
    GProp_GProps ag; BRepGProp::SurfaceProperties(s, ag); m.area = ag.Mass();
  } catch (const Standard_Failure&) { m.valid = false; }
  return m;
}

// A pie-sector solid cylinder: radius R, along +Z from z=zLo, height, angular span from
// `angLo` to angLo+span (radians). MakeCylinder(ax2,R,H,angle) sweeps its X-dir by `angle`.
TopoDS_Shape sectorCylinder(double R, double zLo, double height, double angLo, double span) {
  gp_Ax2 ax(gp_Pnt(0, 0, zLo), gp_Dir(0, 0, 1));
  if (std::fabs(angLo) > 1e-15) ax.Rotate(gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), angLo);
  return BRepPrimAPI_MakeCylinder(ax, R, height, span).Shape();
}

// Reconstruct the embossed / debossed rectangle solid (empty on any boolean failure — a
// best-effort SECONDARY oracle; the closed form is authoritative).
TopoDS_Shape buildOcctReconRect(const GenCase& c) {
  try {
    const TopoDS_Shape base = BRepPrimAPI_MakeCylinder(c.Rc, c.H).Shape();
    const double axialLen = c.vMax - c.vMin;
    if (c.boss == 1) {
      // EMBOSS: FUSE the base with a full pie wedge of radius Rout; the union adds exactly
      // the outer shell over the sector (overlapping solids → a robust single boolean).
      const TopoDS_Shape wedge = sectorCylinder(c.Rc + c.amount, c.vMin, axialLen, 0.0, c.dU);
      BRepAlgoAPI_Fuse fuse(base, wedge); fuse.Build();
      return fuse.IsDone() ? fuse.Shape() : TopoDS_Shape{};
    }
    // DEBOSS: CUT the base with a shell wedge [R−depth, R+margin] over the sector. The
    // inner core wedge is grown slightly in angle/axial so its faces do NOT coincide with
    // the outer wedge's (keeps both booleans transversal / robust). Only the in-base part
    // (radius [R−depth, R]) is removed → the pocket.
    const double rFloor = c.Rc - c.amount;
    const double eZ = 0.05, eA = 0.02;
    const TopoDS_Shape outer = sectorCylinder(c.Rc + 1.0, c.vMin, axialLen, 0.0, c.dU);
    const TopoDS_Shape inner =
        sectorCylinder(rFloor, c.vMin - eZ, axialLen + 2.0 * eZ, -eA, c.dU + 2.0 * eA);
    BRepAlgoAPI_Cut shellCut(outer, inner); shellCut.Build();
    if (!shellCut.IsDone()) return {};
    BRepAlgoAPI_Cut pocket(base, shellCut.Shape()); pocket.Build();
    return pocket.IsDone() ? pocket.Shape() : TopoDS_Shape{};
  } catch (const Standard_Failure&) { return {}; }
}

double relDiff(double a, double b) { return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : 1e30; }

// ── the classifier (mirrors the M6/M6b/M6c/M6d siblings, with the closed-form ground
//    truth as the PRIMARY correctness oracle and the OCCT reconstruction as a secondary
//    cross-check + the only independent AREA oracle on the rectangle families) ───────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACCURATE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_bothDecl = 0, g_surprise = 0;
int g_famAgreed[F_COUNT] = {0}, g_famDeclined[F_COUNT] = {0}, g_famDisagreed[F_COUNT] = {0};
int g_famOracleInacc[F_COUNT] = {0}, g_famBothDecl[F_COUNT] = {0};
int g_occtChecked = 0;                // rectangle trials with a valid OCCT reconstruction
double g_maxVolBias = 0, g_maxAreaBias = 0;   // max native-vs-oracle rel diff on AGREE

// Classify ONE in-scope core trial (native already measured). `occtOk` flags a valid OCCT
// reconstruction (rectangle only); volO/areaO are its measurements.
Verdict classifyCore(const GenCase& c, const NativeMeasure& nat, bool occtOk, double volO,
                     double areaO) {
  const bool nativeUsable = nat.present && nat.watertight && nat.volume > 1e-9;
  if (!nativeUsable) return DECLINED;   // in-scope native decline → OCCT ships (first-class)

  const double volVsAnalytic = relDiff(nat.volume, c.expVol);
  const bool analyticMatch = volVsAnalytic < kVolRelTol;
  const bool occtVolMatch = occtOk && relDiff(nat.volume, volO) < kVolRelTol;
  const bool occtAreaMatch = occtOk && relDiff(nat.area, areaO) < kAreaRelTol;

  if (!analyticMatch) return DISAGREED;                 // watertight but WRONG vs exact math
  // Native matches the exact closed form. Cross-check the OCCT reconstruction (rectangles).
  if (occtOk && !occtVolMatch) return ORACLE_INACCURATE;  // native right by math, OCCT outlier
  if (occtOk && !occtAreaMatch) return DISAGREED;         // vol right but AREA wrong — a defect
  g_maxVolBias = std::max(g_maxVolBias, volVsAnalytic);
  if (occtOk) g_maxAreaBias = std::max(g_maxAreaBias, relDiff(nat.area, areaO));
  return AGREED;
}

void logTrial(int i, const GenCase& c, Verdict v, const NativeMeasure& nat, bool occtOk,
              double volO, double areaO, uint64_t seed) {
  const double volVsA = c.analytic ? relDiff(nat.volume, c.expVol) : -1.0;
  switch (v) {
    case AGREED:
      std::printf("[FUZZ] AGREED    case=%d %-26s volN=%.6g expVol=%.6g dV=%.2e areaN=%.6g "
                  "occt[ok=%d volO=%.6g areaO=%.6g] dVol=%.6g solids=%d\n",
                  i, famName(c.family), nat.volume, c.expVol, volVsA, nat.area,
                  occtOk ? 1 : 0, volO, areaO, c.dVol, nat.solids);
      break;
    case DECLINED:
      std::printf("[FUZZ] DECLINED  case=%d %-26s native=%s (in-scope self-verify discard → "
                  "OCCT ships)  %s\n", i, famName(c.family),
                  nat.present ? "non-watertight" : "NULL", c.desc.c_str());
      break;
    case BOTH_DECLINED:
      std::printf("[FUZZ] BOTH-DECL case=%d %-26s native declined out-of-scope input (no wrong "
                  "result)  %s\n", i, famName(c.family), c.desc.c_str());
      break;
    case ORACLE_INACCURATE:
      std::printf("[FUZZ] ORACLE_INACCURATE case=%d %-26s native MATCHES closed form, OCCT recon "
                  "does NOT  volN=%.6g expVol=%.6g volO=%.6g\n       NOTE seed=0x%llx index=%d %s\n",
                  i, famName(c.family), nat.volume, c.expVol, volO,
                  static_cast<unsigned long long>(seed), i, c.desc.c_str());
      break;
    case DISAGREED:
      std::printf("[FUZZ] DISAGREED case=%d %-26s SILENT-WRONG-WRAP-EMBOSS  volN=%.6g expVol=%.6g "
                  "dV(analytic)=%.3e areaN=%.6g occt[ok=%d volO=%.6g areaO=%.6g]\n"
                  "       REPRO seed=0x%llx index=%d boss=%d %s\n",
                  i, famName(c.family), nat.volume, c.expVol, volVsA, nat.area, occtOk ? 1 : 0,
                  volO, areaO, static_cast<unsigned long long>(seed), i, c.boss, c.desc.c_str());
      break;
  }
  std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x5745E6B055ull;   // "WE" ...
  int N = 120;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 120;

  std::printf("== M6-breadth-5 differential-fuzz: native WRAP-EMBOSS (cylinder) vs closed-form "
              "+ OCCT-boolean reconstruction ==\n");
  std::printf("== seed=0x%llx N=%d volRelTol=%.0e areaRelTol=%.0e defl=%g ==\n",
              static_cast<unsigned long long>(seed), N, kVolRelTol, kAreaRelTol, kWeDefl);
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    GenCase c = genCase(rng);

    // (1) NATIVE builder (OCCT-FREE): build the base body, pick the cylinder wall, wrap.
    ntopo::Shape body = c.nonCyl ? buildNativeBox(2.0 * c.Rc, 2.0 * c.Rc, c.H)
                                 : buildNativeCylinder(c.Rc, c.H);
    const int face = findCylFace(body);
    const ntopo::Shape natShape =
        nfeat::wrap_emboss(body, face, c.prof.data(), c.count, c.amount, c.boss, kWeDefl);
    const NativeMeasure nat = measureNative(natShape);

    // (2) ORACLE — OCCT reconstruction for the rectangle families (best-effort, where clean).
    bool occtOk = false; double volO = 0, areaO = 0;
    if (isRectFamily(c.family)) {
      const OcctMeasure occ = measureOcct(buildOcctReconRect(c));
      occtOk = occ.present && occ.valid && occ.volume > 1e-9 && occ.area > 1e-9 && occ.solids >= 1;
      volO = occ.volume; areaO = occ.area;
      if (occtOk) ++g_occtChecked;
    }

    // (3) classify.
    Verdict v;
    if (isCoreFamily(c.family)) {
      v = classifyCore(c, nat, occtOk, volO, areaO);
    } else {
      // Out-of-scope DECLINE-exerciser: native MUST return NULL / non-watertight.
      const bool nativeUsable = nat.present && nat.watertight && nat.volume > 1e-9;
      if (!nativeUsable) v = BOTH_DECLINED;
      else { v = BOTH_DECLINED; ++g_surprise;   // a guard leak — flagged below, not a bar pass
             std::printf("[FUZZ] SURPRISE  case=%d %-26s native BUILT a watertight solid for an "
                         "out-of-scope input (guard leak?)  volN=%.6g  REPRO seed=0x%llx index=%d %s\n",
                         i, famName(c.family), nat.volume,
                         static_cast<unsigned long long>(seed), i, c.desc.c_str()); }
    }

    switch (v) {
      case AGREED:            ++g_agreed;      ++g_famAgreed[c.family];      break;
      case DECLINED:          ++g_declined;    ++g_famDeclined[c.family];    break;
      case DISAGREED:         ++g_disagreed;   ++g_famDisagreed[c.family];   break;
      case ORACLE_INACCURATE: ++g_oracleInacc; ++g_famOracleInacc[c.family]; break;
      case BOTH_DECLINED:     ++g_bothDecl;    ++g_famBothDecl[c.family];    break;
    }
    if (isCoreFamily(c.family)) logTrial(i, c, v, nat, occtOk, volO, areaO, seed);
    else if (v == BOTH_DECLINED && g_surprise == 0)
      std::printf("[FUZZ] BOTH-DECL case=%d %-26s native declined out-of-scope input  %s\n",
                  i, famName(c.family), c.desc.c_str());
    std::fflush(stdout);
  }

  // ── coverage summary ──────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE-INACCURATE=%d  "
              "BOTH-DECLINED=%d  (OCCT rect reconstructions checked=%d)\n",
              g_agreed, g_declined, g_disagreed, g_oracleInacc, g_bothDecl, g_occtChecked);
  std::printf("   per-family [agreed/declined/DISAGREED/oracle-inaccurate/both-declined]:\n");
  for (int f = 0; f < F_COUNT; ++f)
    std::printf("     %-26s %d/%d/%d/%d/%d\n", famName(f), g_famAgreed[f], g_famDeclined[f],
                g_famDisagreed[f], g_famOracleInacc[f], g_famBothDecl[f]);
  std::printf("   max observed native-vs-oracle bias on AGREE: vol=%.3e area=%.3e "
              "(FIXED tol vol=%.0e area=%.0e)\n",
              g_maxVolBias, g_maxAreaBias, kVolRelTol, kAreaRelTol);
  if (g_oracleInacc) std::printf("   ORACLE-INACCURATE=%d (native VINDICATED by closed form vs OCCT "
                                 "reconstruction — oracle-side limitation, logged)\n", g_oracleInacc);
  if (g_surprise)    std::printf("   SURPRISE=%d (native built a solid for an out-of-scope input — "
                                 "guard leak, investigate)\n", g_surprise);

  const bool bar = (g_disagreed == 0 && g_surprise == 0);
  std::printf("== M6-breadth-5 BAR: %s (DISAGREED=%d must be 0) ==\n",
              bar ? "PASS — zero silent wrong wrap-emboss" : "FAIL", g_disagreed);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
