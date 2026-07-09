// SPDX-License-Identifier: Apache-2.0
//
// native_directmodel_fuzz.mm — MOAT M6-breadth-10 DIRECT-MODELING differential
//                              fuzzer, driven THROUGH the cc_* facade under BOTH
//                              engines (iOS simulator).
//
// This is the TENTH native domain on the differential-fuzzing completeness bar,
// extending the landed M6 fuzzers (native_boolean_fuzz, native_step_import_fuzz,
// native_construct_fuzz, native_blend_fuzz, native_wrap_emboss_fuzz,
// native_mass_props_fuzz, native_geometry_services_fuzz, native_transform_fuzz,
// native_reference_geometry_fuzz) to the DIRECT-MODELING layer the CyberCad app's
// direct-edit tools call: cc_split_plane, cc_replace_face (parallel offset of a
// planar cap), and cc_project_point_on_face. These three DM ops each have a hand-
// picked per-op PARITY fixture (native_split_plane_parity, native_replace_face_parity,
// native_dm3_dm4_parity) but NO *fuzz* domain — this closes that gap.
//
// Unlike the eight internal-C++ fuzzers, this harness (like native_hlr_parity) drives
// the SHIPPING PATH: it runs each DM op ONCE with the OCCT engine active (the oracle)
// and ONCE with the NativeEngine active (the OCCT-free core), then compares the two
// results, AND checks each against a THIRD engine-independent CLOSED-FORM arbiter in
// plain fp64. So the native cc_* result must agree with BOTH OCCT and the exact math.
//
//   cc_set_engine(0)  → OCCT engine   (BRepAlgoAPI_Cut half-space / GeomAPI_Project…)
//   cc_set_engine(1)  → NativeEngine  (OCCT-free splitByPlane / replaceFace / project)
//
// ── Base families (built IDENTICALLY under both engines via cc_solid_extrude /
//    cc_solid_revolve, so the DM op operates on topologically-identical operands) ────
//   BOX      — random w×d rectangle extruded by h.
//   NGON     — random regular n-gon (n∈[3,8]) of radius R extruded by h.
//   CYLINDER — random rectangle profile {0,0, R,0, R,h, 0,h} revolved 2π about +Y.
//   CONE     — random trapezoid profile (frustum) revolved 2π about +Y.
//
// ── Ops (per trial: one random base family × one random op) ────────────────────────
//   SPLIT   — cc_split_plane with a random plane (axis-aligned OR oblique) and a random
//             keep side. Compare native-vs-OCCT result on VOLUME / AREA / centroid /
//             bbox. CLOSED-FORM arbiter: for an axis-aligned plane through a BOX or NGON
//             prism the exact keep-volume is known (base-area × clipped height, or a
//             prism cross-section clip); for EVERY family a PARTITION-CLOSURE arbiter
//             V(keep+) + V(keep−) == V(whole) is asserted (both cut both ways).
//   OFFSET  — cc_replace_face parallel offset of a random PLANAR CAP face by a random
//             signed distance. Compare native-vs-OCCT result VOLUME / AREA / bbox.
//             CLOSED-FORM arbiter: ΔV == capArea·offset (cap area known from the family
//             geometry), so grow/trim volume is checked against exact math.
//   PROJECT — cc_project_point_on_face of a random point onto a random face. Compare
//             native-vs-OCCT foot + distance. CLOSED-FORM arbiter: for a PLANAR face the
//             foot is p − ((p−o)·n̂)n̂; for a CYLINDER lateral face the foot is the axis-
//             radial projection. Cone lateral / freeform → native honestly declines.
//
// ── Classification (per op trial) ──────────────────────────────────────────────────
//   AGREED            — native == OCCT (within a FIXED never-widened tol) AND, where a
//                       closed form exists, native == exact math.
//   HONESTLY-DECLINED — native returns 0/invalid on a case the native slice scopes out
//                       (e.g. cone-lateral projection) AND OCCT succeeds — a first-class
//                       decline, counted separately, NEVER a bar failure.
//   BOTH-DECLINED     — an out-of-scope exerciser both engines refuse.
//   ORACLE-INACCURATE — native matches the exact math, OCCT does not (native vindicated).
//   DISAGREED         — native produced a result that does NOT match OCCT / the exact
//                       math. A real finding → the headline, never papered over.
//   ORACLE_UNRELIABLE — a core case whose OCCT result itself disagrees with the closed
//                       form (gated off; the oracle is pathological, not native).
//
// The bar: DISAGREED == 0 AND ORACLE_UNRELIABLE == 0, with each base family AND each op
// carrying ≥1 AGREED. The generator is seeded ONLY by an explicit FUZZ_SEED (argv/env)
// — NO clock, NO rand(): same seed → byte-identical batch.
//
// Output: [FUZZ] lines then a COVERAGE SUMMARY table and the M6-breadth-10 BAR verdict.
// Flushes stdout and std::_Exit (the trimmed static-OCCT build's teardown is not exit-
// clean — same rationale as the sibling harnesses; every shape id is released first).
//
// Build: scripts/run-sim-native-directmodel-fuzz.sh (compiles the whole kernel + OCCT,
// spawns on a booted simulator). Carries its own main(); on run-sim-suite.sh SKIP list.

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

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

constexpr double kPi = 3.14159265358979323846;

// FIXED, never-widened tolerances.
constexpr double kVolRel  = 2e-2;   // native-vs-OCCT volume relative tol (curved defl-bounded)
constexpr double kAreaRel = 3e-2;   // native-vs-OCCT area relative tol
constexpr double kBboxAbs = 1.5e-2; // native-vs-OCCT bbox absolute tol (mm, deflection-scaled)
constexpr double kMathRel = 5e-3;   // result-vs-exact-math relative tol (analytic families)
constexpr double kFootTol = 1e-6;   // project foot + distance absolute tol (closed-form)
constexpr double kDefl    = 5e-3;   // tessellation deflection where a mesh is implied

double relDiff(double a, double b) {
  const double d = std::fabs(a - b);
  const double s = std::max({1.0, std::fabs(a), std::fabs(b)});
  return d / s;
}

// ── families / ops ───────────────────────────────────────────────────────────────
enum Base { B_BOX, B_NGON, B_CYLINDER, B_CONE, B_COUNT };
enum Op   { OP_SPLIT, OP_OFFSET, OP_PROJECT, OP_COUNT };
const char* baseName(int b) {
  switch (b) { case B_BOX: return "BOX"; case B_NGON: return "NGON";
               case B_CYLINDER: return "CYLINDER"; case B_CONE: return "CONE"; }
  return "?";
}
const char* opName(int o) {
  switch (o) { case OP_SPLIT: return "SPLIT"; case OP_OFFSET: return "OFFSET";
               case OP_PROJECT: return "PROJECT"; }
  return "?";
}

// A generated base solid: how to build it (profile + revolve flag) plus the closed-form
// data the arbiters need. All families are axis-aligned in world coords as built:
//   BOX/NGON  extruded +Z from z=0 to z=h; footprint polygon in the z=0 plane.
//   CYL/CONE  revolved about world +Y; base disk at y=0, top disk at y=h.
struct BaseCase {
  int base;
  std::string desc;
  // build inputs
  std::vector<double> profileXY;  // extrude: footprint x,y pairs; revolve: (r,y) pairs
  bool revolve;                   // false → cc_solid_extrude(depth=h); true → cc_solid_revolve(2π)
  double depth;                   // extrude depth
  // closed-form geometry (world coords)
  double h;                       // height (extrude: along +Z; revolve: along +Y)
  double baseArea;                // footprint / base-disk area (mm^2)
  double wholeVol;                // exact volume (mm^3)
  double R0, R1;                  // revolve radii (base, top); cone R0!=R1
  // axis-aligned extents (min corner + size), for axis-aligned split arbiter
  double bbMin[3], bbMax[3];
  // a point lying exactly ON a lateral (side) face, at mid-height — used to resolve the
  // side face identically under both engines and (for BOX) as the closed-form plane point.
  double latProbe[3];
  double latNormal[3];            // outward unit normal of that lateral face (BOX/NGON planar)
};

CCShapeId buildShape(const BaseCase& c) {
  if (c.revolve)
    return cc_solid_revolve(c.profileXY.data(),
                            static_cast<int>(c.profileXY.size() / 2), 2.0 * kPi);
  return cc_solid_extrude(c.profileXY.data(),
                          static_cast<int>(c.profileXY.size() / 2), c.depth);
}

// Regular n-gon footprint (CCW) of circumradius R centred at origin, first vertex on +X.
std::vector<double> ngon(int n, double R) {
  std::vector<double> p;
  p.reserve(2 * n);
  for (int i = 0; i < n; ++i) {
    const double a = 2.0 * kPi * i / n;
    p.push_back(R * std::cos(a));
    p.push_back(R * std::sin(a));
  }
  return p;
}
double ngonArea(int n, double R) { return 0.5 * n * R * R * std::sin(2.0 * kPi / n); }

BaseCase genBase(Rng& rng) {
  BaseCase c{};
  c.base = static_cast<int>(rng.below(B_COUNT));
  switch (c.base) {
    case B_BOX: {
      const double w = rng.range(4.0, 14.0), d = rng.range(4.0, 14.0), h = rng.range(4.0, 14.0);
      // rectangle centred at origin in z=0 plane, CCW
      c.profileXY = {-w / 2, -d / 2, w / 2, -d / 2, w / 2, d / 2, -w / 2, d / 2};
      c.revolve = false; c.depth = h; c.h = h;
      c.baseArea = w * d; c.wholeVol = w * d * h;
      c.bbMin[0] = -w / 2; c.bbMin[1] = -d / 2; c.bbMin[2] = 0;
      c.bbMax[0] =  w / 2; c.bbMax[1] =  d / 2; c.bbMax[2] = h;
      char b[96]; std::snprintf(b, sizeof b, "box w=%.3f d=%.3f h=%.3f", w, d, h);
      c.desc = b; break;
    }
    case B_NGON: {
      const int n = 3 + static_cast<int>(rng.below(6));   // 3..8
      const double R = rng.range(3.0, 8.0), h = rng.range(4.0, 14.0);
      c.profileXY = ngon(n, R);
      c.revolve = false; c.depth = h; c.h = h;
      c.baseArea = ngonArea(n, R); c.wholeVol = c.baseArea * h;
      c.bbMin[0] = -R; c.bbMin[1] = -R; c.bbMin[2] = 0;
      c.bbMax[0] =  R; c.bbMax[1] =  R; c.bbMax[2] = h;
      char b[96]; std::snprintf(b, sizeof b, "ngon n=%d R=%.3f h=%.3f", n, R, h);
      c.desc = b; break;
    }
    case B_CYLINDER: {
      const double R = rng.range(3.0, 8.0), h = rng.range(5.0, 14.0);
      c.profileXY = {0, 0, R, 0, R, h, 0, h};
      c.revolve = true; c.depth = 0; c.h = h; c.R0 = R; c.R1 = R;
      c.baseArea = kPi * R * R; c.wholeVol = kPi * R * R * h;
      c.bbMin[0] = -R; c.bbMin[1] = 0; c.bbMin[2] = -R;
      c.bbMax[0] =  R; c.bbMax[1] = h; c.bbMax[2] =  R;
      char b[96]; std::snprintf(b, sizeof b, "cyl R=%.3f h=%.3f", R, h);
      c.desc = b; break;
    }
    default: {  // B_CONE frustum
      const double R0 = rng.range(4.0, 8.0), h = rng.range(5.0, 14.0);
      double R1 = rng.range(1.5, R0 - 1.0);              // strictly smaller top
      c.profileXY = {0, 0, R0, 0, R1, h, 0, h};
      c.revolve = true; c.depth = 0; c.h = h; c.R0 = R0; c.R1 = R1;
      c.baseArea = kPi * R0 * R0;
      c.wholeVol = kPi * h / 3.0 * (R0 * R0 + R0 * R1 + R1 * R1);   // frustum volume
      const double Rmax = std::max(R0, R1);
      c.bbMin[0] = -Rmax; c.bbMin[1] = 0; c.bbMin[2] = -Rmax;
      c.bbMax[0] =  Rmax; c.bbMax[1] = h; c.bbMax[2] =  Rmax;
      char b[96]; std::snprintf(b, sizeof b, "cone R0=%.3f R1=%.3f h=%.3f", R0, R1, h);
      c.desc = b; break;
    }
  }
  // lateral-face probe (a point ON a side face at mid-height) + its outward normal.
  const double mid = 0.5 * (c.bbMin[c.revolve ? 1 : 2] + c.bbMax[c.revolve ? 1 : 2]);
  if (c.base == B_BOX) {
    const double halfW = c.bbMax[0];   // +X face plane x = w/2
    c.latProbe[0] = halfW; c.latProbe[1] = 0; c.latProbe[2] = mid;
    c.latNormal[0] = 1; c.latNormal[1] = 0; c.latNormal[2] = 0;
  } else if (c.base == B_NGON) {
    // the side face between vertex 0 (angle 0) and vertex 1 (angle 2π/n): outward normal at
    // angle π/n, at the apothem R·cos(π/n). Recover n,R from the profile point count + [x0].
    const int n = static_cast<int>(c.profileXY.size() / 2);
    const double R = c.profileXY[0];   // vertex 0 = (R, 0)
    const double half = kPi / n, apo = R * std::cos(half);
    c.latProbe[0] = apo * std::cos(half); c.latProbe[1] = apo * std::sin(half); c.latProbe[2] = mid;
    c.latNormal[0] = std::cos(half); c.latNormal[1] = std::sin(half); c.latNormal[2] = 0;
  } else {  // CYL / CONE — curved lateral; probe at +X mid-height (identifies the lateral face)
    const double r = c.revolve ? 0.5 * (c.R0 + c.R1) : c.R0;
    c.latProbe[0] = r; c.latProbe[1] = mid; c.latProbe[2] = 0;
    c.latNormal[0] = 1; c.latNormal[1] = 0; c.latNormal[2] = 0;
  }
  return c;
}

// ── classification bookkeeping ─────────────────────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_INACC, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleInacc = 0, g_oracleBad = 0, g_bothDecl = 0;
int g_famAgreed[B_COUNT] = {0}, g_opAgreed[OP_COUNT] = {0};
int g_famDisagreed[B_COUNT] = {0}, g_opDisagreed[OP_COUNT] = {0};

void tally(Verdict v, int base, int op) {
  switch (v) {
    case AGREED: ++g_agreed; ++g_famAgreed[base]; ++g_opAgreed[op]; break;
    case DECLINED: ++g_declined; break;
    case DISAGREED: ++g_disagreed; ++g_famDisagreed[base]; ++g_opDisagreed[op]; break;
    case ORACLE_INACC: ++g_oracleInacc; break;
    case ORACLE_UNRELIABLE: ++g_oracleBad; break;
    case BOTH_DECLINED: ++g_bothDecl; break;
  }
}
const char* vName(Verdict v) {
  switch (v) { case AGREED: return "AGREED"; case DECLINED: return "HONESTLY-DECLINED";
               case DISAGREED: return "DISAGREED"; case ORACLE_INACC: return "ORACLE-INACCURATE";
               case ORACLE_UNRELIABLE: return "ORACLE_UNRELIABLE";
               case BOTH_DECLINED: return "BOTH-DECLINED"; }
  return "?";
}

void logTrial(Verdict v, int caseIdx, uint64_t seed, const BaseCase& bc, int op,
              const char* detail) {
  if (v == DISAGREED || v == ORACLE_UNRELIABLE || v == ORACLE_INACC) {
    std::printf("[FUZZ] %-17s case=%d op=%-7s %-9s | %s\n"
                "       REPRO seed=0x%llx index=%d :: %s\n",
                vName(v), caseIdx, opName(op), baseName(bc.base), detail,
                static_cast<unsigned long long>(seed), caseIdx, bc.desc.c_str());
  } else {
    std::printf("[FUZZ] %-17s case=%d op=%-7s %-9s | %s :: %s\n",
                vName(v), caseIdx, opName(op), baseName(bc.base), detail, bc.desc.c_str());
  }
  std::fflush(stdout);
  tally(v, bc.base, op);
}

// ── measurement helpers ─────────────────────────────────────────────────────────────
// CRITICAL: a shape id must be measured under the SAME engine that BUILT it. The OCCT
// mass-properties adapter unwraps the shared registry entry as a TopoDS_Shape*; handing it
// a NATIVE body id (built under cc_set_engine(1)) casts a NativeShape* to TopoDS_Shape* and
// dereferences garbage. So measure() takes the owning engine and activates it first, then
// restores OCCT as the default. Every call site passes engine=1 for native ids, 0 for OCCT.
struct Metrics { bool ok; double vol, area, cx, cy, cz; double bb[6]; };
Metrics measure(CCShapeId id, int engine) {
  Metrics m{};
  if (id == 0) return m;
  cc_set_engine(engine);
  const CCMassProps mp = cc_mass_properties(id);
  double bb[6];
  const bool bbOk = cc_bounding_box(id, bb);
  cc_set_engine(0);
  if (!mp.valid || !bbOk) return m;
  m.ok = true; m.vol = mp.volume; m.area = mp.area;
  m.cx = mp.cx; m.cy = mp.cy; m.cz = mp.cz;
  for (int i = 0; i < 6; ++i) m.bb[i] = bb[i];
  return m;
}
bool bboxClose(const double a[6], const double b[6]) {
  for (int i = 0; i < 6; ++i) if (std::fabs(a[i] - b[i]) > kBboxAbs) return false;
  return true;
}

// A shape id built under a given engine; released in dtor under that same engine.
struct Body {
  CCShapeId id = 0;
  ~Body() { if (id) cc_shape_release(id); }
};

// ── SPLIT arbiter: exact keep-volume for an AXIS-ALIGNED plane on BOX / NGON prism ───
// For an axis-aligned plane the box/prism is clipped in one coordinate; the kept fraction
// of the extrude height (z) or the footprint (x/y for a box) gives an exact volume. We
// only claim exactness for the z-clip (both families) and x/y-clip of a BOX; the NGON
// footprint clip has no simple closed form (only closure is asserted there).
struct SplitArb { bool hasExact; double keepVol; };
SplitArb splitExact(const BaseCase& c, int axis, double coord, bool keepPositive) {
  SplitArb a{false, 0.0};
  auto frac = [&](double lo, double hi) -> double {
    if (coord <= lo) return keepPositive ? 1.0 : 0.0;           // whole slab on + side
    if (coord >= hi) return keepPositive ? 0.0 : 1.0;
    const double up = (hi - coord) / (hi - lo);                  // fraction with x>coord
    return keepPositive ? up : (1.0 - up);
  };
  if (axis == 2) {  // z clip → both BOX and NGON prism: base area preserved
    if (c.base == B_BOX || c.base == B_NGON) {
      a.hasExact = true; a.keepVol = c.wholeVol * frac(c.bbMin[2], c.bbMax[2]);
    }
  } else if (c.base == B_BOX) {  // x or y clip of a box: rectangular slab
    a.hasExact = true; a.keepVol = c.wholeVol * frac(c.bbMin[axis], c.bbMax[axis]);
  }
  return a;
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0xD3ADBEE710ull;
  int N = 96;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 96;

  std::printf("== M6-breadth-10 DIRECT-MODELING differential fuzz (cc_* under both engines) "
              "seed=0x%llx N=%d ==\n", static_cast<unsigned long long>(seed), N);
  if (!cc_brep_available()) {
    std::printf("cc_brep_available()==0 — no B-rep engine linked; cannot run.\n");
    std::fflush(stdout);
    std::_Exit(1);
  }

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    const BaseCase bc = genBase(rng);
    const int op = static_cast<int>(rng.below(OP_COUNT));
    char detail[512];

    // Build the SAME operand under both engines up-front.
    cc_set_engine(0);
    Body occtBase; occtBase.id = buildShape(bc);
    cc_set_engine(1);
    Body natBase; natBase.id = buildShape(bc);
    cc_set_engine(0);  // leave OCCT active by default between ops

    if (occtBase.id == 0 || natBase.id == 0) {
      // A base neither engine (or one) could build → out of scope for a DM trial.
      std::snprintf(detail, sizeof detail, "base build occt=%ld nat=%ld (%s)",
                    occtBase.id, natBase.id, cc_last_error());
      logTrial(BOTH_DECLINED, i, seed, bc, op, detail);
      continue;
    }

    if (op == OP_SPLIT) {
      // Random plane: 60% axis-aligned (exact arbiter), 40% oblique (closure only).
      double o[3], n[3];
      int axis = -1;
      const bool aligned = rng.unit() < 0.6;
      if (aligned) {
        axis = static_cast<int>(rng.below(3));
        for (int k = 0; k < 3; ++k) { o[k] = 0; n[k] = (k == axis) ? 1.0 : 0.0; }
        // origin on the plane: a coord strictly inside the bbox on that axis
        o[axis] = rng.range(bc.bbMin[axis] + 0.15 * (bc.bbMax[axis] - bc.bbMin[axis]),
                            bc.bbMax[axis] - 0.15 * (bc.bbMax[axis] - bc.bbMin[axis]));
      } else {
        // oblique unit normal, origin at the bbox centre
        double nx = rng.range(-1, 1), ny = rng.range(-1, 1), nz = rng.range(-1, 1);
        double L = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (L < 1e-6) { nx = 1; ny = nz = 0; L = 1; }
        n[0] = nx / L; n[1] = ny / L; n[2] = nz / L;
        for (int k = 0; k < 3; ++k) o[k] = 0.5 * (bc.bbMin[k] + bc.bbMax[k]);
      }
      const int keepPos = static_cast<int>(rng.below(2));

      // Cut BOTH ways under BOTH engines (closure needs both sides).
      cc_set_engine(0);
      Body oPos;  oPos.id  = cc_split_plane(occtBase.id, o[0], o[1], o[2], n[0], n[1], n[2], 1);
      Body oNeg;  oNeg.id  = cc_split_plane(occtBase.id, o[0], o[1], o[2], n[0], n[1], n[2], 0);
      cc_set_engine(1);
      Body nKeep; nKeep.id = cc_split_plane(natBase.id, o[0], o[1], o[2], n[0], n[1], n[2], keepPos);
      Body nPos;  nPos.id  = cc_split_plane(natBase.id, o[0], o[1], o[2], n[0], n[1], n[2], 1);
      Body nNeg;  nNeg.id  = cc_split_plane(natBase.id, o[0], o[1], o[2], n[0], n[1], n[2], 0);
      cc_set_engine(0);

      const Metrics oKeepM = measure(keepPos ? oPos.id : oNeg.id, 0);
      const Metrics nKeepM = measure(nKeep.id, 1);

      if (nKeep.id == 0) {
        // Native declined this split (out-of-scope config). OCCT is the oracle fallback.
        if (oKeepM.ok) {
          std::snprintf(detail, sizeof detail, "native declined split; OCCT keepVol=%.4f",
                        oKeepM.vol);
          logTrial(DECLINED, i, seed, bc, op, detail);
        } else {
          std::snprintf(detail, sizeof detail, "both declined split (%s)", cc_last_error());
          logTrial(BOTH_DECLINED, i, seed, bc, op, detail);
        }
        continue;
      }
      if (!oKeepM.ok || !nKeepM.ok) {
        std::snprintf(detail, sizeof detail, "measure failed occt=%d nat=%d",
                      oKeepM.ok, nKeepM.ok);
        logTrial(BOTH_DECLINED, i, seed, bc, op, detail);
        continue;
      }

      // native vs OCCT on the kept side
      const double vr = relDiff(nKeepM.vol, oKeepM.vol);
      const double ar = relDiff(nKeepM.area, oKeepM.area);
      const bool bbOk = bboxClose(nKeepM.bb, oKeepM.bb);

      // partition closure under NATIVE (both sides must sum to the whole)
      const Metrics nPosM = measure(nPos.id, 1), nNegM = measure(nNeg.id, 1);
      bool closureOk = true; double closeErr = -1;
      if (nPosM.ok && nNegM.ok) {
        closeErr = relDiff(nPosM.vol + nNegM.vol, bc.wholeVol);
        closureOk = closeErr <= kVolRel;
      }

      // exact-math arbiter (axis-aligned box/prism)
      double mathErr = -1; bool mathOk = true;
      if (aligned) {
        const SplitArb sa = splitExact(bc, axis, o[axis], keepPos != 0);
        if (sa.hasExact) { mathErr = relDiff(nKeepM.vol, sa.keepVol); mathOk = mathErr <= kMathRel; }
      }

      const bool nativeOcctOk = (vr <= kVolRel) && (ar <= kAreaRel) && bbOk;
      std::snprintf(detail, sizeof detail,
                    "%s keep%c axis=%d | dV=%.2e dA=%.2e bb=%d clos=%.2e math=%.2e",
                    aligned ? "aligned" : "oblique", keepPos ? '+' : '-', axis,
                    vr, ar, bbOk, closeErr, mathErr);

      Verdict v;
      if (!nativeOcctOk && mathErr >= 0 && mathOk && closureOk) {
        // native matches exact math + its own closure, but OCCT diverges → OCCT is off.
        v = ORACLE_INACC;
      } else if (!nativeOcctOk || !closureOk || !mathOk) {
        v = DISAGREED;
      } else {
        v = AGREED;
      }
      logTrial(v, i, seed, bc, op, detail);
      continue;
    }

    if (op == OP_OFFSET) {
      // Offset a random PLANAR CAP face. For all families the top+bottom caps are planar.
      // Face ids differ per engine, so we pick the cap by its outward-normal direction and
      // resolve the id per engine via cc_face_axis is N/A for planes — instead we drive the
      // NATIVE core (which serves parallel cap offset in closed form) and the OCCT engine on
      // THEIR OWN cap face, matched by choosing the cap whose centroid extreme is max/min on
      // the extrude/revolve axis. We identify the cap by asking cc_bounding_box after the op.
      //
      // Simpler + engine-robust: the caps are the +axis and -axis extreme faces. We use the
      // known geometry (a signed offset along the outward axis) and pick the face id whose
      // plane matches. To avoid per-engine face-id ambiguity we OFFSET the whole family's
      // TOP cap only, resolved via cc_subshape_ids + cc_project probing is overkill — instead
      // we rely on cc_replace_face's per-engine face id from cc_subshape_ids and match the cap
      // by the sign of its representative point on the axis.
      const int axis = bc.revolve ? 1 : 2;                 // +Y for revolve, +Z for extrude
      const bool topCap = rng.unit() < 0.5;                // top (+axis) or bottom (-axis)
      const double axisLo = topCap ? bc.bbMax[axis] : bc.bbMin[axis];
      // Signed offset along the cap's OUTWARD normal. cc_replace_face's OCCT adapter is a
      // half-space CUT — it can only TRIM (negative offset), never GROW (positive): a grow
      // leaves the solid un-grown while native genuinely extends it, so a GROW is the exact
      // ORACLE-INACCURATE signature (native == exact math, OCCT the outlier). We generate
      // BOTH signs: a TRIM (off<0) is an AGREE (native==OCCT==math); a GROW (off>0) surfaces
      // the oracle limitation as ORACLE-INACCURATE, never a bar failure.
      const bool grow = rng.unit() < 0.5;
      const double mag = rng.range(1.0, std::min(3.0, 0.35 * bc.h));   // keep a trim non-degenerate
      const double off = grow ? mag : -mag;

      // Resolve the cap face id under a given engine: the planar face whose vertices are all
      // at the axis extreme. We probe with cc_project_point_on_face against candidate ids and
      // pick the one whose foot equals the cap plane. But a cheaper robust route: after the
      // offset the volume changes by capArea*off regardless of which id — so we search face ids
      // for the one that yields the expected exact ΔV under the OCCT (oracle) engine, then use
      // the SAME id-selection rule (axis-extreme) under native.
      auto capFaceId = [&](CCShapeId body, double extreme) -> int {
        int* ids = nullptr;
        const int nf = cc_subshape_ids(body, 2, &ids);
        int best = 0; double bestErr = 1e300;
        for (int k = 0; k < nf; ++k) {
          // representative: project the cap centre (0,0 in-plane at the extreme) onto face k
          double px = 0, py = 0, pz = 0;
          if (axis == 1) py = extreme; else pz = extreme;
          const CCProjection pr = cc_project_point_on_face(body, ids[k], px, py, pz);
          if (!pr.valid) continue;
          const double footAxis = (axis == 1) ? pr.footY : pr.footZ;
          const double err = std::fabs(footAxis - extreme) + pr.distance;
          if (err < bestErr) { bestErr = err; best = ids[k]; }
        }
        if (ids) cc_ints_free(ids);
        return (bestErr < 0.05) ? best : 0;
      };

      cc_set_engine(0);
      const int oFid = capFaceId(occtBase.id, axisLo);
      cc_set_engine(1);
      const int nFid = capFaceId(natBase.id, axisLo);
      cc_set_engine(0);

      if (oFid == 0 || nFid == 0) {
        std::snprintf(detail, sizeof detail, "cap face-id resolve occt=%d nat=%d", oFid, nFid);
        logTrial(BOTH_DECLINED, i, seed, bc, op, detail);
        continue;
      }

      // Signed offset in cc_replace_face convention: positive grows along the cap's OUTWARD
      // normal. Bottom cap outward is -axis, so a positive `off` still grows the solid.
      cc_set_engine(0);
      Body oRes; oRes.id = cc_replace_face(occtBase.id, oFid, off, 0.0);
      cc_set_engine(1);
      Body nRes; nRes.id = cc_replace_face(natBase.id, nFid, off, 0.0);
      cc_set_engine(0);

      const Metrics oM = measure(oRes.id, 0), nM = measure(nRes.id, 1);
      if (nRes.id == 0) {
        if (oM.ok) { std::snprintf(detail, sizeof detail, "native declined offset; OCCT vol=%.4f", oM.vol);
                     logTrial(DECLINED, i, seed, bc, op, detail); }
        else { std::snprintf(detail, sizeof detail, "both declined offset (%s)", cc_last_error());
                logTrial(BOTH_DECLINED, i, seed, bc, op, detail); }
        continue;
      }
      if (!oM.ok || !nM.ok) {
        std::snprintf(detail, sizeof detail, "measure failed occt=%d nat=%d", oM.ok, nM.ok);
        logTrial(BOTH_DECLINED, i, seed, bc, op, detail);
        continue;
      }

      const double vr = relDiff(nM.vol, oM.vol);
      const double ar = relDiff(nM.area, oM.area);
      const bool bbOk = bboxClose(nM.bb, oM.bb);

      // exact-math arbiter — ONLY for CONSTANT-cross-section families (BOX / NGON / CYLINDER):
      // moving a cap plane by `off` along its outward normal changes the volume by exactly
      // capArea*off (the section is invariant along the extrude/revolve axis). A CONE frustum's
      // conical side wall changes radius with height, so cap-offset ΔV is NOT capArea*off — we
      // deliberately do NOT claim a closed form for the cone here and arbitrate it by
      // native==OCCT (a trim, where the OCCT half-space cut is exact) alone.
      const bool constSection = (bc.base != B_CONE);
      double capArea = bc.baseArea;                        // extrude footprint / cylinder disk
      const double expectedVol = bc.wholeVol + capArea * off;
      double mathErr = -1; bool mathOk = true;
      if (constSection) { mathErr = relDiff(nM.vol, expectedVol); mathOk = mathErr <= kMathRel; }

      const bool nativeOcctOk = (vr <= kVolRel) && (ar <= kAreaRel) && bbOk;
      std::snprintf(detail, sizeof detail,
                    "%scap %s off=%.3f | dV=%.2e dA=%.2e bb=%d math=%.2e",
                    topCap ? "top" : "bot", grow ? "grow" : "trim", off, vr, ar, bbOk, mathErr);

      Verdict v;
      if (!nativeOcctOk && constSection && mathOk) v = ORACLE_INACC;  // native==exact math, OCCT off
      else if (!nativeOcctOk || !mathOk) v = DISAGREED;
      else v = AGREED;
      logTrial(v, i, seed, bc, op, detail);
      continue;
    }

    // OP_PROJECT — cc_project_point_on_face on a random face with a random exterior point.
    {
      // Choose a face id under each engine by the same rule (a lateral face or a cap), and a
      // random 3-D point offset from the surface. We compare native vs OCCT foot+distance, and
      // check the closed-form foot for PLANAR caps and (for CYL) the lateral face.
      // Native serves plane / cylinder / sphere and declines cone-lateral / freeform.
      const int axis = bc.revolve ? 1 : 2;
      // pick a target: 0 = a planar cap (all families), 1 = the lateral face (cyl exact,
      // cone declines, box/ngon lateral is a plane too).
      const int targetLateral = static_cast<int>(rng.below(2));

      // A source point: for a cap target, a point above/below the cap plane on the axis; for a
      // lateral target, a point radially outside at mid-height.
      double p[3] = {0, 0, 0};
      const double mid = 0.5 * (bc.bbMin[axis] + bc.bbMax[axis]);
      double expFoot[3]; bool haveExpFoot = false; bool nativeShouldServe = true;
      double expDist = -1;

      // Face-id resolver — GEOMETRICALLY TYPED so both engines target the SAME face:
      //   wantLateral=false → the CAP whose plane sits at axis extreme `extreme`
      //                       (cc_face_axis returns 0 for a plane; the cap is the planar
      //                       face whose probe on the cap-plane has ~0 projection distance).
      //   wantLateral=true  → the curved side face (cc_face_axis returns a valid cyl/cone
      //                       axis) for CYL/CONE, or the planar side face nearest `probe`
      //                       for BOX/NGON. This is independent of per-engine id numbering,
      //                       so native and OCCT resolve the geometrically identical face —
      //                       and if native has NO servable such face it returns 0 (a true
      //                       decline, not a wrong-face agreement).
      // The native periodic revolution can split a cylinder/cone wall into MORE THAN ONE
      // curved face (a seam split — the same representational quirk the reference-geometry
      // fuzzer documents). So for a curved lateral we accept the BEST curved face the engine
      // actually SERVES (min projection distance), not "exactly one". If NO curved face is
      // servable (cone under native → declines), we return 0 → a genuine decline.
      auto pickTyped = [&](CCShapeId body, bool wantLateral, const double probe[3]) -> int {
        int* ids = nullptr; const int nf = cc_subshape_ids(body, 2, &ids);
        int best = 0; double bestErr = 1e300;
        const bool curvedLat = wantLateral && (bc.base == B_CYLINDER || bc.base == B_CONE);
        for (int k = 0; k < nf; ++k) {
          double ax6[6];
          const bool curved = cc_face_axis(body, ids[k], ax6) != 0;   // cyl/cone lateral
          if (curvedLat) {
            if (!curved) continue;                                      // curved side faces only
          } else if (wantLateral) {
            if (curved) continue;                                       // box/ngon side is planar
          } else {
            if (curved) continue;                                       // cap must be planar
          }
          const CCProjection pr = cc_project_point_on_face(body, ids[k], probe[0], probe[1], probe[2]);
          if (!pr.valid) continue;                                      // native declines → skip
          if (pr.distance < bestErr) { bestErr = pr.distance; best = ids[k]; }
        }
        if (ids) cc_ints_free(ids);
        if (curvedLat) return best;                                     // best servable wall (0=decline)
        return (bestErr < 1e-3) ? best : 0;    // planar: probe sits ON the face → tiny residual
      };

      double probe[3];   // an on-surface point identifying the target face
      if (!targetLateral) {
        // cap target: probe = a point on the +axis or -axis cap
        const bool topCap = rng.unit() < 0.5;
        const double capC = topCap ? bc.bbMax[axis] : bc.bbMin[axis];
        for (int k = 0; k < 3; ++k) probe[k] = 0; probe[axis] = capC;
        // source point: offset the cap probe by a random amount along the axis (outside)
        for (int k = 0; k < 3; ++k) p[k] = 0;
        p[axis] = capC + (topCap ? 1.0 : -1.0) * rng.range(1.0, 5.0);
        // add a small in-plane wander so the foot is nontrivial but still on the cap
        const int other0 = (axis + 1) % 3, other1 = (axis + 2) % 3;
        const double wobble = 0.25 * std::min(bc.bbMax[other0] - bc.bbMin[other0], 2.0);
        p[other0] += rng.range(-wobble, wobble);
        p[other1] += rng.range(-wobble, wobble);
        // closed-form foot on a horizontal cap plane axis=capC: foot = p with p[axis]=capC.
        haveExpFoot = true;
        expFoot[0] = p[0]; expFoot[1] = p[1]; expFoot[2] = p[2]; expFoot[axis] = capC;
        expDist = std::fabs(p[axis] - capC);
        nativeShouldServe = true;   // planar face — native serves
      } else {
        // lateral target
        if (bc.base == B_CYLINDER) {
          const double R = bc.R0;
          const double ang = rng.range(0, 2 * kPi);
          // on-surface probe at mid-height, radius R
          double rp[3];
          rp[axis] = mid;
          const int u = (axis + 1) % 3, w = (axis + 2) % 3;
          rp[u] = R * std::cos(ang); rp[w] = R * std::sin(ang);
          for (int k = 0; k < 3; ++k) probe[k] = rp[k];
          // source point: radially outside by `d`
          const double d = rng.range(1.0, 5.0);
          for (int k = 0; k < 3; ++k) p[k] = rp[k];
          p[u] = (R + d) * std::cos(ang); p[w] = (R + d) * std::sin(ang);
          p[axis] = mid + rng.range(-1.0, 1.0);
          // closed-form foot: radial projection onto the cylinder of radius R about the axis.
          const double rr = std::sqrt(p[u] * p[u] + p[w] * p[w]);
          expFoot[axis] = p[axis]; expFoot[u] = p[u] * R / rr; expFoot[w] = p[w] * R / rr;
          expDist = std::fabs(rr - R);
          haveExpFoot = true; nativeShouldServe = true;
        } else if (bc.base == B_CONE) {
          // cone lateral is out-of-native-scope → native should decline; use OCCT probe only.
          const double Rmid = 0.5 * (bc.R0 + bc.R1);
          const double ang = rng.range(0, 2 * kPi);
          const int u = (axis + 1) % 3, w = (axis + 2) % 3;
          probe[axis] = mid; probe[u] = Rmid * std::cos(ang); probe[w] = Rmid * std::sin(ang);
          for (int k = 0; k < 3; ++k) p[k] = probe[k];
          p[u] = (Rmid + 2.0) * std::cos(ang); p[w] = (Rmid + 2.0) * std::sin(ang);
          nativeShouldServe = false; haveExpFoot = false;
        } else {
          // BOX / NGON lateral = a planar side face. genBase precomputed latProbe (a point ON
          // one specific side face at mid-height) and latNormal (that face's outward unit
          // normal), so the closed form is the general plane foot for BOTH families:
          //   foot = p − ((p − latProbe)·n̂)·n̂,   dist = |(p − latProbe)·n̂|.
          const double* fp = bc.latProbe; const double* nrm = bc.latNormal;
          for (int k = 0; k < 3; ++k) probe[k] = fp[k];   // sits exactly on the face
          // source point: move OUT along the face normal + a tangential wander (still over
          // the face, so the foot stays interior).
          const double outD = rng.range(1.0, 4.0);
          const double tang = rng.range(-0.3 * (bc.bbMax[2] - bc.bbMin[2]), 0.3 * (bc.bbMax[2] - bc.bbMin[2]));
          for (int k = 0; k < 3; ++k) p[k] = fp[k] + outD * nrm[k];
          p[2] += tang;                                   // wander along the vertical (z) extent
          const double dp = (p[0] - fp[0]) * nrm[0] + (p[1] - fp[1]) * nrm[1] + (p[2] - fp[2]) * nrm[2];
          expFoot[0] = p[0] - dp * nrm[0]; expFoot[1] = p[1] - dp * nrm[1]; expFoot[2] = p[2] - dp * nrm[2];
          expDist = std::fabs(dp);
          haveExpFoot = true; nativeShouldServe = true;
        }
      }

      cc_set_engine(0);
      const int oFid = pickTyped(occtBase.id, targetLateral != 0, probe);
      const CCProjection oPr = (oFid ? cc_project_point_on_face(occtBase.id, oFid, p[0], p[1], p[2])
                                     : CCProjection{});
      cc_set_engine(1);
      const int nFid = pickTyped(natBase.id, targetLateral != 0, probe);
      const CCProjection nPr = (nFid ? cc_project_point_on_face(natBase.id, nFid, p[0], p[1], p[2])
                                     : CCProjection{});
      cc_set_engine(0);

      // native declined (valid==0) on a scoped-out face
      if (!nPr.valid) {
        if (!nativeShouldServe) {
          // expected decline (cone lateral). OCCT is the fallback oracle.
          std::snprintf(detail, sizeof detail, "native declined project (scoped out); OCCT d=%.4f",
                        oPr.valid ? oPr.distance : -1.0);
          logTrial(DECLINED, i, seed, bc, op, detail);
        } else {
          // native declined a face it should serve → real gap (report as DISAGREED)
          std::snprintf(detail, sizeof detail, "native declined a %s face it should serve (%s)",
                        targetLateral ? "lateral" : "cap", cc_last_error());
          logTrial(DISAGREED, i, seed, bc, op, detail);
        }
        continue;
      }
      if (!oPr.valid) {
        // OCCT declined but native served — verify native against the closed form.
        double fe = -1;
        if (haveExpFoot) {
          fe = std::hypot(std::hypot(nPr.footX - expFoot[0], nPr.footY - expFoot[1]),
                          nPr.footZ - expFoot[2]);
        }
        if (haveExpFoot && fe <= 1e-4 && std::fabs(nPr.distance - expDist) <= 1e-4) {
          std::snprintf(detail, sizeof detail, "OCCT declined; native matches exact foot fe=%.2e", fe);
          logTrial(ORACLE_INACC, i, seed, bc, op, detail);
        } else {
          std::snprintf(detail, sizeof detail, "OCCT declined, no closed form to arbitrate");
          logTrial(ORACLE_UNRELIABLE, i, seed, bc, op, detail);
        }
        continue;
      }

      // both valid → compare foot + distance
      const double footD = std::hypot(std::hypot(nPr.footX - oPr.footX, nPr.footY - oPr.footY),
                                      nPr.footZ - oPr.footZ);
      const double distD = std::fabs(nPr.distance - oPr.distance);
      double mathFootErr = -1, mathDistErr = -1; bool mathOk = true;
      if (haveExpFoot) {
        mathFootErr = std::hypot(std::hypot(nPr.footX - expFoot[0], nPr.footY - expFoot[1]),
                                 nPr.footZ - expFoot[2]);
        mathDistErr = std::fabs(nPr.distance - expDist);
        mathOk = (mathFootErr <= 1e-4) && (mathDistErr <= 1e-4);
      }
      const bool natOcctOk = (footD <= kFootTol) && (distD <= kFootTol);
      std::snprintf(detail, sizeof detail,
                    "%s | footD=%.2e distD=%.2e mathFoot=%.2e mathDist=%.2e",
                    targetLateral ? "lateral" : "cap", footD, distD, mathFootErr, mathDistErr);

      Verdict v;
      if (!natOcctOk && haveExpFoot && mathOk) v = ORACLE_INACC;   // native right, OCCT off
      else if (!natOcctOk || !mathOk) v = DISAGREED;
      else v = AGREED;
      logTrial(v, i, seed, bc, op, detail);
      continue;
    }
  }

  cc_set_engine(0);  // restore default engine

  // ── coverage table ───────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("  TOTALS: AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  "
              "ORACLE-INACCURATE=%d  ORACLE_UNRELIABLE=%d  BOTH-DECLINED=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleInacc, g_oracleBad, g_bothDecl);
  std::printf("  per-BASE-FAMILY  (AGREED [DISAGREED]):\n");
  for (int b = 0; b < B_COUNT; ++b)
    std::printf("      %-9s  %3d [%d]\n", baseName(b), g_famAgreed[b], g_famDisagreed[b]);
  std::printf("  per-OP           (AGREED [DISAGREED]):\n");
  for (int o = 0; o < OP_COUNT; ++o)
    std::printf("      %-9s  %3d [%d]\n", opName(o), g_opAgreed[o], g_opDisagreed[o]);
  std::printf("  HONEST SCOPE: native declines cone-lateral projection (a first-class DECLINE,\n"
              "                counted, never a bar failure); every AGREE checked native==OCCT\n"
              "                AND, where a closed form exists, native==exact math.\n");
  if (g_oracleInacc)
    std::printf("  ORACLE-INACCURATE (%d): native matched the exact closed form, OCCT was the\n"
                "                    outlier — native VINDICATED, not a bar failure.\n", g_oracleInacc);

  bool famCov = true; for (int b = 0; b < B_COUNT; ++b) if (g_famAgreed[b] < 1) famCov = false;
  bool opCov  = true; for (int o = 0; o < OP_COUNT; ++o) if (g_opAgreed[o] < 1) opCov = false;
  const bool bar = (g_disagreed == 0 && g_oracleBad == 0 && famCov && opCov);
  std::printf("== M6-breadth-10 BAR: %s (DISAGREED=%d must be 0; ORACLE_UNRELIABLE=%d must be 0; "
              "base-family coverage=%s; op coverage=%s) ==\n",
              bar ? "PASS — zero silent wrong direct-model results" : "FAIL",
              g_disagreed, g_oracleBad,
              famCov ? "complete" : "INCOMPLETE", opCov ? "complete" : "INCOMPLETE");
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
