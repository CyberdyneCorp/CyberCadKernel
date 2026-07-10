// SPDX-License-Identifier: Apache-2.0
//
// native_wrap_emboss_freeform_fuzz.mm — MOAT M6-breadth WRAP-EMBOSS FREEFORM-BASE
// differential-fuzzing harness (iOS simulator), driven THROUGH the cc_* facade under BOTH
// engines. It closes the gap the landed native_wrap_emboss_fuzz.mm left: that harness
// fuzzes the CYLINDER lateral face ONLY (rect + polygon, emboss + deboss), by calling the
// OCCT-FREE native builder feature::wrap_emboss DIRECTLY. This harness certifies the NEW
// F5 arm of src/native/feature/wrap_emboss.h — a RAISED circular pole boss on a FREEFORM
// sphere-cap dome (a NON-developable base) — alongside the cylinder base as a developable
// control, both driven through the SHIPPING cc_wrap_emboss facade under cc_set_engine(0)
// (OCCT) and cc_set_engine(1) (NativeEngine).
//
// ── THE DIFFERENTIAL (native vs oracle on the SAME facade input) ─────────────────────
// Per trial DETERMINISTICALLY generate a random pose for one base×mode cell:
//     CYL-RAISED     — a rectangle/polygon footprint raised on a cylinder wall (boss=1)
//     CYL-RECESSED   — a rectangle/polygon footprint recessed on a cylinder wall (boss=0)
//     SPH-RAISED     — a circular pole boss raised on a sphere-cap dome (boss=1, F5)
//     SPH-RECESSED   — an HONEST DOMAIN-level decline (native has no sphere-deboss path)
//   plus SPARSE out-of-envelope exercisers (a BOX base, a >2π footprint, a deboss depth ≥ R,
//   a self-intersecting pentagram, a boss reaching the dome rim) → native NULL.
// The base solid is built IDENTICALLY under both engines (cc_solid_extrude_profile full
// circle / cc_solid_revolve_profile dome); the wrap face is resolved GEOMETRICALLY on the
// body being embossed. The RNG is a splitmix64-seeded xoshiro256** stream keyed ONLY by
// FUZZ_SEED (argv/env) — NO clock, NO rand(): same seed → byte-identical batch.
//
// ── THE ARBITER — closed-form PRIMARY, OCCT only where developable ───────────────────
// Wrap-emboss has NO single OCCT API, and OCCT's OWN cc_wrap_emboss CANNOT wrap a
// non-cylindrical face — presented with a sphere wall it DECLINES (returns 0). So:
//   * CYLINDER base (developable): PRIMARY = the closed-form curvature-corrected changed
//     volume A·|Rout²−R²|/(2R). OCCT (the SAME cc_wrap_emboss under engine 0, which DOES
//     wrap a cylinder) is a SECONDARY cross-check on volume + area.
//   * SPHERE-CAP base (NON-developable): OCCT DECLINES the sphere wall (asserted per trial),
//     so the CLOSED-FORM shell-sector delta 2π(1−cosφ0)·((R+h)³−R³)/3 is the SOLE boss
//     arbiter, added to the base-dome volume OCCT DOES measure exactly (cc_mass_properties
//     on the revolved dome — an independent OCCT reference for the BASE, not the wrap).
// Each in-scope native boss is additionally checked watertight + Euler χ=2 + a strictly
// grown volume (raise) + mesh-vol ≈ brep-vol.
//
// ── FIXED TOLERANCES (never widened; the curated parity harness proved these poses) ──
//   cylinder native/OCCT/closed-form volume ≤ 2e-2, area ≤ 3e-2;
//   sphere native-vs-closed-form volume ≤ 1.5e-2, mesh-vs-brep ≤ 2e-2.
// The native facet soup is an INSCRIBED approximation whose bias sits far under these; the
// max observed bias is logged so the margin is auditable.
//
// Classify each trial AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE_UNRELIABLE /
// BOTH-DECLINED. Exit 0 IFF DISAGREED==0 && ORACLE_UNRELIABLE==0 with each in-scope cell
// ≥1 AGREED and no guard-leak SURPRISE.
//
// Drives the cc_* facade under both engines → links the WHOLE kernel + OCCT. Built ONLY by
// scripts/run-sim-native-wrap-emboss-freeform-fuzz.sh; on run-sim-suite.sh's SKIP list (own
// main(), std::_Exit — OCCT static teardown in the trimmed static build is not exit-clean).

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kCylVolTol = 2e-2;    // FIXED cylinder volume agreement bar (never widened)
constexpr double kCylAreaTol = 3e-2;   // FIXED cylinder area agreement bar
constexpr double kSphVolTol = 1.5e-2;  // FIXED sphere native-vs-closed-form volume bar
constexpr double kMeshRelTol = 2e-2;   // FIXED mesh-vol vs brep-vol bar
constexpr double kTessDefl = 0.01;     // tessellation deflection for the watertight/χ check

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream (verbatim discipline of the
//    landed native_wrap_emboss_fuzz / native_boolean_fuzz). Keyed ONLY by an explicit
//    uint64 seed. No clock, no rand(): same seed → byte-identical batch. ────────────────
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

// ── families: base × mode + out-of-envelope decline exercisers ───────────────────────
enum Family {
  F_CYL_RAISED, F_CYL_RECESSED, F_SPH_RAISED,
  F_SPH_RECESSED /*honest domain decline*/,
  F_DECL_NONCYL, F_DECL_OVER2PI, F_DECL_DEEP, F_DECL_SELFX, F_DECL_SPH_RIM, F_COUNT
};
const char* famName(int f) {
  switch (f) {
    case F_CYL_RAISED:    return "cylinder raised";
    case F_CYL_RECESSED:  return "cylinder recessed";
    case F_SPH_RAISED:    return "sphere-cap raised";
    case F_SPH_RECESSED:  return "sphere-cap recessed[DECLINE]";
    case F_DECL_NONCYL:   return "box base[DECLINE]";
    case F_DECL_OVER2PI:  return "footprint>2pi[DECLINE]";
    case F_DECL_DEEP:     return "deboss depth>=R[DECLINE]";
    case F_DECL_SELFX:    return "self-intersecting[DECLINE]";
    case F_DECL_SPH_RIM:  return "boss reaches rim[DECLINE]";
  }
  return "?";
}
bool isCyl(int f) { return f == F_CYL_RAISED || f == F_CYL_RECESSED; }
bool isSph(int f) { return f == F_SPH_RAISED || f == F_SPH_RECESSED; }
bool isRaised(int f) { return f == F_CYL_RAISED || f == F_SPH_RAISED; }
// in-scope AGREED-eligible cells: cyl raised/recessed + sph raised. sph-recessed + the
// out-of-envelope exercisers are honest declines (never expected to AGREE).
bool isInScope(int f) { return f == F_CYL_RAISED || f == F_CYL_RECESSED || f == F_SPH_RAISED; }
bool isDeclineFamily(int f) { return f >= F_SPH_RECESSED; }

// ── footprint builders (px,py profile handed to cc_wrap_emboss) ──────────────────────
std::vector<double> rectProfile(double aw, double ah) {   // centred axis-aligned CCW
  return {-aw / 2, -ah / 2,  aw / 2, -ah / 2,  aw / 2, ah / 2,  -aw / 2, ah / 2};
}
std::vector<double> ngonProfile(Rng& r, int n, double a) {  // convex, small radial jitter
  std::vector<double> p; p.reserve(static_cast<std::size_t>(2 * n));
  for (int i = 0; i < n; ++i) {
    const double ang = 2.0 * kPi * i / n, rad = a * r.range(0.85, 1.0);
    p.push_back(rad * std::cos(ang)); p.push_back(rad * std::sin(ang));
  }
  return p;
}
double shoelaceArea(const std::vector<double>& p, int count) {
  double a2 = 0.0;
  for (int i = 0; i < count; ++i) {
    const int j = (i + 1) % count;
    a2 += p[i * 2] * p[j * 2 + 1] - p[j * 2] * p[i * 2 + 1];
  }
  return std::fabs(a2) * 0.5;
}
// The footprint bbox in-radius ρ (half the smaller extent) — the sphere pole disc radius.
double profileInRadius(const std::vector<double>& p, int count) {
  double xmn = p[0], xmx = p[0], ymn = p[1], ymx = p[1];
  for (int i = 1; i < count; ++i) {
    xmn = std::min(xmn, p[i * 2]); xmx = std::max(xmx, p[i * 2]);
    ymn = std::min(ymn, p[i * 2 + 1]); ymx = std::max(ymx, p[i * 2 + 1]);
  }
  return 0.5 * std::min(xmx - xmn, ymx - ymn);
}

// ── closed-form arbiters ─────────────────────────────────────────────────────────────
// Cylinder curvature-corrected changed volume for a footprint of flat area A.
double cylChangedVolume(double A, double R, double amount, bool emboss) {
  const double rT = emboss ? (R + amount) : (R - amount);
  return A * std::fabs(rT * rT - R * R) / (2.0 * R);
}
// Sphere pole-boss shell-sector delta 2π(1−cosφ0)·((R+h)³−R³)/3.
double sphPoleBossDelta(double R, double h, double phi0) {
  return 2.0 * kPi * (1.0 - std::cos(phi0)) * ((R + h) * (R + h) * (R + h) - R * R * R) / 3.0;
}

std::string fmt(const char* form, double a, double b = 0, double c = 0, double d = 0) {
  char buf[256]; std::snprintf(buf, sizeof buf, form, a, b, c, d); return buf;
}

// A generated case.
struct GenCase {
  int family = 0;
  double R = 0;                 // cylinder radius / sphere radius
  double H = 0;                 // cylinder height
  double capOff = 0;            // sphere-cap plane offset along the pole
  double amount = 0;            // emboss height / deboss depth
  int boss = 1;
  std::vector<double> prof;     // (px,py) footprint
  int count = 0;
  double phi0 = 0;              // sphere boss half-angle
  double dU = 0;                // cylinder wrapped angular span
  std::string desc;
};

int pickFamily(Rng& r) {
  // Core cells weighted heavily; the sphere-recessed domain-decline + out-of-envelope
  // exercisers stay SPARSE (they exist to hit the native NULL branch, not to make DISAGREE).
  const int w[F_COUNT] = {6, 6, 6, 2, 1, 1, 1, 1, 1};
  int tot = 0; for (int x : w) tot += x;
  int k = static_cast<int>(r.below(static_cast<uint32_t>(tot)));
  for (int i = 0; i < F_COUNT; ++i) { if (k < w[i]) return i; k -= w[i]; }
  return F_CYL_RAISED;
}

GenCase genCase(Rng& r) {
  GenCase c; c.family = pickFamily(r);
  switch (c.family) {
    case F_CYL_RAISED:
    case F_CYL_RECESSED: {
      c.boss = (c.family == F_CYL_RAISED) ? 1 : 0;
      c.R = r.range(6.0, 12.0);
      c.H = r.range(3.0 * c.R, 4.0 * c.R);
      const bool poly = r.unit() < 0.4;
      if (poly) {
        const int n = 3 + static_cast<int>(r.below(4));  // 3..6
        const double a = r.range(0.3 * c.R, std::min(0.6 * c.R, 0.18 * c.H));
        c.prof = ngonProfile(r, n, a); c.count = n;
      } else {
        const double aw = r.range(0.6 * c.R, 1.6 * c.R);   // Δu = aw/R < 2π
        const double ah = r.range(0.25 * c.H, 0.5 * c.H);
        c.prof = rectProfile(aw, ah); c.count = 4;
      }
      c.amount = c.boss ? r.range(0.15 * c.R, 0.45 * c.R) : r.range(0.15 * c.R, 0.5 * c.R);
      double pxmn = c.prof[0], pxmx = c.prof[0];
      for (int i = 1; i < c.count; ++i) { pxmn = std::min(pxmn, c.prof[i*2]); pxmx = std::max(pxmx, c.prof[i*2]); }
      c.dU = (pxmx - pxmn) / c.R;
      c.desc = fmt("R=%.3f H=%.3f amt=%.3f", c.R, c.H, c.amount) + fmt(" n=%.0f dU=%.3f", (double)c.count, c.dU);
      break;
    }
    case F_SPH_RAISED:
    case F_SPH_RECESSED: {
      c.boss = (c.family == F_SPH_RAISED) ? 1 : 0;
      c.R = r.range(7.0, 13.0);
      c.capOff = r.range(-0.3 * c.R, 0.25 * c.R);   // cap plane offset (dome depth varies)
      // profile in-radius ρ → φ0 = ρ/R, kept strictly inside the dome polar opening φcap.
      const double phiCap = std::acos(std::clamp(c.capOff / c.R, -1.0, 1.0));
      c.phi0 = r.range(0.12, std::min(0.42, 0.75 * phiCap));
      const double rho = c.phi0 * c.R;
      c.prof = rectProfile(2.0 * rho, 2.0 * rho); c.count = 4;   // square, in-radius = rho
      c.amount = r.range(0.12 * c.R, 0.35 * c.R);
      c.desc = fmt("R=%.3f cap=%.3f phi0=%.3f", c.R, c.capOff, c.phi0) + fmt(" h=%.3f", c.amount);
      break;
    }
    case F_DECL_NONCYL: {   // BOX base — the picked face is planar → native declines.
      c.R = r.range(6.0, 12.0); c.H = r.range(3.0 * c.R, 4.0 * c.R); c.boss = 1;
      c.prof = rectProfile(0.6 * c.R, 0.3 * c.H); c.count = 4; c.amount = 0.3 * c.R;
      c.desc = fmt("box base R=%.3f H=%.3f", c.R, c.H);
      break;
    }
    case F_DECL_OVER2PI: {  // arc span ≥ full turn → native guard NULL.
      c.R = r.range(6.0, 12.0); c.H = r.range(3.0 * c.R, 4.0 * c.R); c.boss = 1;
      const double aw = r.range(2.2 * kPi * c.R, 2.8 * kPi * c.R);
      c.prof = rectProfile(aw, 0.3 * c.H); c.count = 4; c.amount = 0.3 * c.R;
      c.desc = fmt("R=%.3f aw=%.3f (du>2pi)", c.R, aw);
      break;
    }
    case F_DECL_DEEP: {     // deboss depth ≥ R → rFloor ≤ 0 → NULL.
      c.R = r.range(6.0, 12.0); c.H = r.range(3.0 * c.R, 4.0 * c.R); c.boss = 0;
      c.prof = rectProfile(0.6 * c.R, 0.3 * c.H); c.count = 4;
      c.amount = r.range(1.05 * c.R, 1.5 * c.R);
      c.desc = fmt("R=%.3f depth=%.3f (>=R)", c.R, c.amount);
      break;
    }
    case F_DECL_SELFX: {    // pentagram {5/2} → polyFootprint rejects the self-crossing loop.
      c.R = r.range(6.0, 12.0); c.H = r.range(3.0 * c.R, 4.0 * c.R); c.boss = 1;
      const double a = 0.5 * c.R;
      for (int k = 0; k < 5; ++k) {
        const double ang = 2.0 * kPi * (2.0 * k) / 5.0;
        c.prof.push_back(a * std::cos(ang)); c.prof.push_back(a * std::sin(ang));
      }
      c.count = 5; c.amount = 0.3 * c.R;
      c.desc = fmt("R=%.3f pentagram a=%.3f", c.R, a);
      break;
    }
    case F_DECL_SPH_RIM: {  // sphere boss half-angle reaching the dome rim → native NULL.
      c.R = r.range(7.0, 13.0); c.capOff = r.range(0.1 * c.R, 0.35 * c.R); c.boss = 1;
      const double phiCap = std::acos(std::clamp(c.capOff / c.R, -1.0, 1.0));
      const double rho = 1.15 * phiCap * c.R;   // ρ/R = 1.15·φcap > φcap → past the rim
      c.prof = rectProfile(2.0 * rho, 2.0 * rho); c.count = 4;
      c.amount = 0.25 * c.R; c.phi0 = rho / c.R;
      c.desc = fmt("R=%.3f cap=%.3f phi0=%.3f (>phiCap)", c.R, c.capOff, c.phi0);
      break;
    }
  }
  return c;
}

// ── base builders (through the facade, under the ACTIVE engine) ──────────────────────
CCShapeId buildCylinder(double R, double H) {   // capped full-circle cylinder about +Z
  CCProfileSeg seg{}; seg.kind = 2; seg.cx = 0; seg.cy = 0; seg.r = R;
  return cc_solid_extrude_profile(&seg, 1, nullptr, 0, nullptr, 0, H);
}
CCShapeId buildBox(double w, double d, double h) {
  const double rect[8] = {0, 0,  w, 0,  w, d,  0, d};
  CCProfileSeg segs[4];
  for (int i = 0; i < 4; ++i) {
    segs[i] = CCProfileSeg{}; segs[i].kind = 0;
    segs[i].x0 = rect[i * 2]; segs[i].y0 = rect[i * 2 + 1];
    segs[i].x1 = rect[((i + 1) % 4) * 2]; segs[i].y1 = rect[((i + 1) % 4) * 2 + 1];
  }
  return cc_solid_extrude_profile(segs, 4, nullptr, 0, nullptr, 0, h);
}
// A sphere-cap dome about +Y (base disc + arc to the pole + closing axis edge, revolved 2π).
// Matches native_wrap_emboss_parity.mm's proven dome so OCCT accepts the closed wire.
CCShapeId buildSphereDome(double R, double capOff) {
  const double rimBase = std::sqrt(std::max(R * R - capOff * capOff, 1e-12));
  CCProfileSeg base{}; base.kind = 0; base.x0 = 0; base.y0 = capOff; base.x1 = rimBase; base.y1 = capOff;
  CCProfileSeg arc{}; arc.kind = 1; arc.x0 = rimBase; arc.y0 = capOff; arc.x1 = 0; arc.y1 = R;
  arc.cx = 0; arc.cy = 0; arc.r = R;
  arc.a0 = std::atan2(capOff, rimBase); arc.a1 = std::atan2(R, 0.0);
  CCProfileSeg axisSeg{}; axisSeg.kind = 0; axisSeg.x0 = 0; axisSeg.y0 = R; axisSeg.x1 = 0; axisSeg.y1 = capOff;
  const CCProfileSeg segs[3] = {base, arc, axisSeg};
  return cc_solid_revolve_profile(segs, 3, 0.0, 0.0, 0.0, 1.0, nullptr, 0, 2.0 * kPi);
}

// The 1-based id of the CYLINDER lateral face. Under OCCT cc_face_axis identifies a
// cylinder/cone face; if it never succeeds (native body — face_axis is OCCT-only) fall back
// to the deterministic wall id 3 (bottom cap, top cap, wall — the SAME resolution the proven
// native_wrap_emboss_parity harness uses for a full-circle extruded cylinder).
int findCylFace(CCShapeId body, double /*R*/) {
  int* ids = nullptr;
  const int n = cc_subshape_ids(body, 2, &ids);
  int found = 0;
  for (int i = 0; i < n && found == 0; ++i) {
    double ax6[6];
    if (cc_face_axis(body, ids[i], ax6)) found = ids[i];
  }
  cc_ints_free(ids);
  return found != 0 ? found : 3;
}
// The 1-based id of the SPHERE wall (all mesh verts ≈ R from the dome centre at the origin).
int findSphereFace(CCShapeId body, double R) {
  CCFaceMesh* faces = nullptr; const int n = cc_face_meshes(body, 0.05, &faces);
  int found = 0;
  for (int f = 0; f < n && found == 0; ++f) {
    const CCFaceMesh& fm = faces[f];
    if (fm.vertexCount < 3) continue;
    bool onSph = true;
    for (int v = 0; v < fm.vertexCount && onSph; ++v) {
      const double* p = &fm.vertices[v * 3];
      const double d = std::sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
      if (std::fabs(d - R) > 1e-3 * R + 1e-6) onSph = false;
    }
    if (onSph) found = fm.faceId;
  }
  if (faces) cc_face_meshes_free(faces, n);
  return found;
}

// ── mesh measurements over a CCMesh (position-welded watertight + Euler χ, matching the
//    curved-fillet / parity welder — the native mesher's coincident corners land a hair
//    apart, so weld to a Euclidean rep before counting edge sharing). ──────────────────
struct Welded { bool watertight = false; int euler = 0; };
Welded weldedTopo(const CCMesh& m) {
  Welded w;
  if (m.triangleCount <= 0) return w;
  constexpr double kWeld = 1e-7;
  std::unordered_map<std::uint64_t, std::vector<int>> cellReps;
  std::vector<int> rep(static_cast<std::size_t>(m.vertexCount));
  auto cellKey = [](long long x, long long y, long long z) -> std::uint64_t {
    std::uint64_t h = static_cast<std::uint64_t>(x) * 73856093u;
    h ^= static_cast<std::uint64_t>(y) * 19349663u; h ^= static_cast<std::uint64_t>(z) * 83492791u;
    return h;
  };
  auto q = [](double v) -> long long { const double s = v / kWeld; return (long long)(s >= 0 ? s + 0.5 : s - 0.5); };
  for (int v = 0; v < m.vertexCount; ++v) {
    const double* p = &m.vertices[v * 3];
    const long long cx = q(p[0]), cy = q(p[1]), cz = q(p[2]);
    int match = -1;
    for (long long dx = -1; dx <= 1 && match < 0; ++dx)
      for (long long dy = -1; dy <= 1 && match < 0; ++dy)
        for (long long dz = -1; dz <= 1 && match < 0; ++dz) {
          auto it = cellReps.find(cellKey(cx + dx, cy + dy, cz + dz));
          if (it == cellReps.end()) continue;
          for (int rid : it->second) {
            const double* rp = &m.vertices[rid * 3];
            if (std::fabs(rp[0]-p[0]) <= kWeld && std::fabs(rp[1]-p[1]) <= kWeld && std::fabs(rp[2]-p[2]) <= kWeld) { match = rid; break; }
          }
        }
    if (match >= 0) rep[(std::size_t)v] = match;
    else { rep[(std::size_t)v] = v; cellReps[cellKey(cx, cy, cz)].push_back(v); }
  }
  std::unordered_map<std::uint64_t, int> edgeCount;
  auto key = [](int a, int b) -> std::uint64_t {
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) | static_cast<std::uint32_t>(b);
  };
  std::unordered_map<int,int> vseen;
  int V = 0;
  auto touchV = [&](int rv){ if (vseen.find(rv)==vseen.end()){ vseen[rv]=1; ++V; } };
  for (int t = 0; t < m.triangleCount; ++t) {
    const int a = rep[(std::size_t)m.triangles[t*3]], b = rep[(std::size_t)m.triangles[t*3+1]], c = rep[(std::size_t)m.triangles[t*3+2]];
    touchV(a); touchV(b); touchV(c);
    ++edgeCount[key(a,b)]; ++edgeCount[key(b,c)]; ++edgeCount[key(c,a)];
  }
  bool wt = true;
  for (const auto& kv : edgeCount) if (kv.second != 2) { wt = false; break; }
  w.watertight = wt;
  const int E = static_cast<int>(edgeCount.size());
  w.euler = V - E + m.triangleCount;   // χ = V − E + F (2 for a closed genus-0 shell)
  return w;
}
double meshVolume(const CCMesh& m) {   // signed divergence volume (|Σ (a×b)·c / 6|)
  double vol = 0.0;
  for (int t = 0; t < m.triangleCount; ++t) {
    const double* a = &m.vertices[m.triangles[t*3]*3];
    const double* b = &m.vertices[m.triangles[t*3+1]*3];
    const double* c = &m.vertices[m.triangles[t*3+2]*3];
    vol += (a[0]*(b[1]*c[2]-b[2]*c[1]) - a[1]*(b[0]*c[2]-b[2]*c[0]) + a[2]*(b[0]*c[1]-b[1]*c[0])) / 6.0;
  }
  return std::fabs(vol);
}

double relDiff(double a, double b) { return (std::fabs(b) > 1e-12) ? std::fabs(a - b) / std::fabs(b) : 1e30; }

// ── classifier ───────────────────────────────────────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_UNRELIABLE, BOTH_DECLINED };

int g_agreed=0, g_declined=0, g_disagreed=0, g_oracleUnrel=0, g_bothDecl=0, g_surprise=0;
int g_famA[F_COUNT]={0}, g_famD[F_COUNT]={0}, g_famX[F_COUNT]={0}, g_famU[F_COUNT]={0}, g_famB[F_COUNT]={0};
int g_sphOcctDeclines=0, g_sphOcctDeclineChecks=0;   // OCCT wrap-emboss decline on the sphere wall
double g_maxCylVolBias=0, g_maxCylAreaBias=0, g_maxSphVolBias=0;

void bump(int f, Verdict v) {
  switch (v) {
    case AGREED:            ++g_agreed; ++g_famA[f]; break;
    case DECLINED:          ++g_declined; ++g_famD[f]; break;
    case DISAGREED:         ++g_disagreed; ++g_famX[f]; break;
    case ORACLE_UNRELIABLE: ++g_oracleUnrel; ++g_famU[f]; break;
    case BOTH_DECLINED:     ++g_bothDecl; ++g_famB[f]; break;
  }
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x5745E6F00Bull;   // "WE-FreefOrm-Boss"
  int N = 64;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 64;

  std::printf("== M6-breadth WRAP-EMBOSS FREEFORM-BASE differential-fuzz: cc_wrap_emboss facade "
              "(both engines) vs closed-form (cylinder + sphere-cap) ==\n");
  std::printf("== seed=0x%llx N=%d cylVolTol=%.0e cylAreaTol=%.0e sphVolTol=%.0e meshRelTol=%.0e ==\n",
              (unsigned long long)seed, N, kCylVolTol, kCylAreaTol, kSphVolTol, kMeshRelTol);
  std::printf("== NOTE: OCCT's own cc_wrap_emboss CANNOT wrap a non-cylindrical face — for the "
              "SPHERE base OCCT DECLINES and the closed-form shell-sector delta is the SOLE arbiter ==\n");
  std::fflush(stdout);

  Rng rng(seed);

  for (int i = 0; i < N; ++i) {
    GenCase c = genCase(rng);
    Verdict v = BOTH_DECLINED;

    // ── (1) NATIVE candidate through the facade ────────────────────────────────────
    cc_set_engine(1);
    CCShapeId nBody = 0;
    if (isCyl(c.family) || c.family == F_DECL_OVER2PI || c.family == F_DECL_DEEP || c.family == F_DECL_SELFX)
      nBody = buildCylinder(c.R, c.H);
    else if (c.family == F_DECL_NONCYL)
      nBody = buildBox(2.0 * c.R, 2.0 * c.R, c.H);
    else /* sphere family / rim exerciser */
      nBody = buildSphereDome(c.R, c.capOff);
    const bool useSphereFace = isSph(c.family) || c.family == F_DECL_SPH_RIM;
    const int nFace = nBody ? (useSphereFace ? findSphereFace(nBody, c.R)
                              : (c.family == F_DECL_NONCYL ? 1 : findCylFace(nBody, c.R))) : 0;
    const CCMassProps nBase = nBody ? cc_mass_properties(nBody) : CCMassProps{0,0,0,0,0,0};
    const CCShapeId nBoss = (nBody && nFace)
        ? cc_wrap_emboss(nBody, nFace, c.prof.data(), c.count, c.amount, c.boss) : 0;
    const CCMassProps nMass = nBoss ? cc_mass_properties(nBoss) : CCMassProps{0,0,0,0,0,0};
    const CCMesh nMesh = nBoss ? cc_tessellate(nBoss, kTessDefl) : CCMesh{nullptr,0,nullptr,0};
    const bool nativeActive = cc_active_engine() == 1;
    const Welded wtopo = nMesh.triangleCount ? weldedTopo(nMesh) : Welded{};
    const double meshVol = nMesh.triangleCount ? meshVolume(nMesh) : 0.0;
    const bool nativeUsable = nBoss != 0 && nativeActive && nMass.valid && nMass.volume > 1e-9
                              && wtopo.watertight && wtopo.euler == 2;

    // ── (2) ORACLE. Cylinder: same facade under OCCT (secondary vol+area cross-check).
    //    Sphere: OCCT DECLINES the wall (asserted); base-dome OCCT volume + closed-form delta.
    cc_set_engine(0);
    CCShapeId oBody = 0;
    if (isCyl(c.family)) oBody = buildCylinder(c.R, c.H);
    else if (useSphereFace) oBody = buildSphereDome(c.R, c.capOff);
    const int oFace = oBody ? (useSphereFace ? findSphereFace(oBody, c.R) : findCylFace(oBody, c.R)) : 0;
    const CCMassProps oBase = oBody ? cc_mass_properties(oBody) : CCMassProps{0,0,0,0,0,0};
    const CCShapeId oWrap = (oBody && oFace)
        ? cc_wrap_emboss(oBody, oFace, c.prof.data(), c.count, c.amount, c.boss) : 0;
    const CCMassProps oMass = oWrap ? cc_mass_properties(oWrap) : CCMassProps{0,0,0,0,0,0};
    const bool occtWrapOk = oWrap != 0 && oMass.valid && oMass.volume > 1e-9;
    if (useSphereFace) {   // OCCT must decline the sphere wall
      ++g_sphOcctDeclineChecks;
      if (oWrap == 0) ++g_sphOcctDeclines;
    }

    // ── (3) classify ────────────────────────────────────────────────────────────────
    if (isDeclineFamily(c.family)) {
      // honest declines (sphere-recessed domain decline + out-of-envelope exercisers):
      // native MUST refuse. A watertight native solid here is a guard leak → SURPRISE.
      if (!nativeUsable) v = BOTH_DECLINED;
      else { v = BOTH_DECLINED; ++g_surprise;
        std::printf("[FUZZ] SURPRISE case=%d %-28s native BUILT a watertight solid for an "
                    "out-of-envelope input (guard leak?) volN=%.6g REPRO seed=0x%llx idx=%d %s\n",
                    i, famName(c.family), nMass.volume, (unsigned long long)seed, i, c.desc.c_str());
      }
    } else if (!nativeUsable) {
      // in-scope native decline → OCCT ships (cylinder) / no engine (sphere): first-class.
      v = occtWrapOk ? DECLINED : BOTH_DECLINED;
      std::printf("[FUZZ] DECLINED  case=%d %-28s native=%s (in-scope self-verify discard) "
                  "nFace=%d occtOk=%d err=\"%s\" %s\n",
                  i, famName(c.family), nBoss ? (wtopo.watertight ? "chi!=2" : "non-watertight") : "NULL",
                  nFace, occtWrapOk ? 1 : 0, cc_last_error(), c.desc.c_str());
    } else if (isCyl(c.family)) {
      const double A = shoelaceArea(c.prof, c.count);
      const double dV = cylChangedVolume(A, c.R, c.amount, c.boss == 1);
      const double expVol = nBase.volume + (c.boss == 1 ? 1.0 : -1.0) * dV;
      const double volVsCF = relDiff(nMass.volume, expVol);
      const bool cfMatch = volVsCF < kCylVolTol;
      const bool signOk = (c.boss == 1) ? (nMass.volume > nBase.volume) : (nMass.volume < nBase.volume);
      const bool occtVolMatch = occtWrapOk && relDiff(nMass.volume, oMass.volume) < kCylVolTol;
      const bool occtAreaMatch = occtWrapOk && relDiff(nMass.area, oMass.area) < kCylAreaTol;
      if (!signOk || !cfMatch) v = DISAGREED;
      else if (occtWrapOk && !occtVolMatch) v = ORACLE_UNRELIABLE;   // native right by math, OCCT outlier
      else if (occtWrapOk && !occtAreaMatch) v = DISAGREED;          // vol right, AREA wrong
      else {
        v = AGREED;
        g_maxCylVolBias = std::max(g_maxCylVolBias, volVsCF);
        if (occtWrapOk) g_maxCylAreaBias = std::max(g_maxCylAreaBias, relDiff(nMass.area, oMass.area));
      }
      std::printf("[FUZZ] %-9s case=%d %-28s volN=%.6g expVol=%.6g dCF=%.2e areaN=%.6g "
                  "occt[ok=%d volO=%.6g areaO=%.6g] wt=%d chi=%d %s\n",
                  v==AGREED?"AGREED":v==DISAGREED?"DISAGREED":v==ORACLE_UNRELIABLE?"ORACLE_U":"DECLINED",
                  i, famName(c.family), nMass.volume, expVol, volVsCF, nMass.area,
                  occtWrapOk?1:0, oMass.volume, oMass.area, wtopo.watertight?1:0, wtopo.euler, c.desc.c_str());
    } else {  // F_SPH_RAISED — OCCT declines; closed-form shell-sector on the OCCT base dome.
      const double dV = sphPoleBossDelta(c.R, c.amount, c.phi0);
      // reference base dome volume: prefer OCCT's exact BRepGProp measure, else native base.
      const double baseVol = (oBase.valid && oBase.volume > 1e-9) ? oBase.volume : nBase.volume;
      const double refVol = baseVol + dV;
      const double volVsCF = relDiff(nMass.volume, refVol);
      const bool cfMatch = volVsCF < kSphVolTol;
      const bool signOk = nMass.volume > nBase.volume;
      const double meshRel = nMass.volume > 0.0 ? std::fabs(meshVol - nMass.volume) / nMass.volume : 1.0;
      const bool meshOk = meshRel < kMeshRelTol;
      const bool baseRefOk = oBase.valid && oBase.volume > 1e-9;   // an independent OCCT base ref exists
      if (!signOk || !cfMatch || !meshOk) v = DISAGREED;
      else if (!baseRefOk) v = ORACLE_UNRELIABLE;   // no trustworthy base reference to ground the delta
      else { v = AGREED; g_maxSphVolBias = std::max(g_maxSphVolBias, volVsCF); }
      std::printf("[FUZZ] %-9s case=%d %-28s volN=%.6g ref(occtBase+dV)=%.6g dCF=%.2e "
                  "occtBase=%.6g dV=%.6g meshRel=%.2e wt=%d chi=%d occtDecl=%d %s\n",
                  v==AGREED?"AGREED":v==DISAGREED?"DISAGREED":v==ORACLE_UNRELIABLE?"ORACLE_U":"DECLINED",
                  i, famName(c.family), nMass.volume, refVol, volVsCF, oBase.volume, dV, meshRel,
                  wtopo.watertight?1:0, wtopo.euler, oWrap==0?1:0, c.desc.c_str());
    }

    if (v == DISAGREED)
      std::printf("       REPRO seed=0x%llx idx=%d family=%s boss=%d %s\n",
                  (unsigned long long)seed, i, famName(c.family), c.boss, c.desc.c_str());
    if (v == ORACLE_UNRELIABLE)
      std::printf("       ORACLE_UNRELIABLE seed=0x%llx idx=%d family=%s %s\n",
                  (unsigned long long)seed, i, famName(c.family), c.desc.c_str());

    bump(c.family, v);

    if (nMesh.triangleCount) cc_mesh_free(nMesh);
    if (nBoss) cc_shape_release(nBoss);
    if (nBody) cc_shape_release(nBody);
    if (oWrap) cc_shape_release(oWrap);
    if (oBody) cc_shape_release(oBody);
    cc_set_engine(0);
    std::fflush(stdout);
  }

  // ── coverage summary ──────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n", (unsigned long long)seed, N);
  std::printf("   AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE_UNRELIABLE=%d  BOTH-DECLINED=%d\n",
              g_agreed, g_declined, g_disagreed, g_oracleUnrel, g_bothDecl);
  std::printf("   per-cell [base × mode]  agreed/declined/DISAGREED/oracle-unreliable/both-declined:\n");
  for (int f = 0; f < F_COUNT; ++f)
    std::printf("     %-28s %d/%d/%d/%d/%d\n", famName(f), g_famA[f], g_famD[f], g_famX[f], g_famU[f], g_famB[f]);
  std::printf("   OCCT sphere-wall wrap declines: %d/%d (OCCT cannot wrap a non-cylindrical face → "
              "closed-form shell-sector is the SOLE sphere arbiter)\n", g_sphOcctDeclines, g_sphOcctDeclineChecks);
  std::printf("   max native-vs-oracle bias on AGREE: cylVol=%.3e cylArea=%.3e sphVol=%.3e "
              "(FIXED tol cylVol=%.0e cylArea=%.0e sphVol=%.0e)\n",
              g_maxCylVolBias, g_maxCylAreaBias, g_maxSphVolBias, kCylVolTol, kCylAreaTol, kSphVolTol);
  if (g_oracleUnrel) std::printf("   ORACLE_UNRELIABLE=%d (investigate — never laundered into a pass)\n", g_oracleUnrel);
  if (g_surprise)    std::printf("   SURPRISE=%d (native built a solid for an out-of-envelope input — guard leak)\n", g_surprise);

  // in-scope cells must each have ≥1 AGREED (sphere-recessed is an honest domain decline, exempt).
  const bool coverage = g_famA[F_CYL_RAISED] > 0 && g_famA[F_CYL_RECESSED] > 0 && g_famA[F_SPH_RAISED] > 0;
  const bool sphDeclineOk = g_sphOcctDeclineChecks == 0 || g_sphOcctDeclines == g_sphOcctDeclineChecks;
  const bool bar = (g_disagreed == 0 && g_oracleUnrel == 0 && g_surprise == 0 && coverage && sphDeclineOk);
  std::printf("== M6-breadth WRAP-EMBOSS FREEFORM BAR: %s (DISAGREED=%d must be 0, ORACLE_UNRELIABLE=%d "
              "must be 0, coverage=%d, occt-sphere-declines-consistent=%d) ==\n",
              bar ? "PASS — zero silent wrong wrap-emboss" : "FAIL",
              g_disagreed, g_oracleUnrel, coverage ? 1 : 0, sphDeclineOk ? 1 : 0);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
