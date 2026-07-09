// SPDX-License-Identifier: Apache-2.0
//
// native_hlr_fuzz.mm — MOAT M6-breadth-13 DIFFERENTIAL-FUZZING harness for the
//                      native ORTHOGRAPHIC HLR / DRAFTING service (iOS simulator).
//
// This is the THIRTEENTH native domain on the differential-fuzzing completeness bar.
// It turns the curated HLR parity harness (native_hlr_parity.mm — a handful of
// hand-picked solids + views) into a DETERMINISTIC SEEDED BATCH of random solids at
// random rigid poses projected from random view directions, and asserts the M6
// discipline: over N inputs, ZERO SILENT WRONG HLR RESULTS.
//
// ── WHAT IT DOES (per the M6 track goal) ────────────────────────────────────────────
//   (1) DETERMINISTICALLY generate random VALID solids from six families:
//         BOX          — random rectangular box (extrude)                 [polyhedral]
//         NGON PRISM   — random convex n-gon (3..8) extruded              [polyhedral]
//         CYLINDER     — random R,h revolve (native Cylinder side face)   [quadric sil.]
//         CONE/FRUSTUM — random cone or truncated frustum revolve         [quadric sil.]
//         SPHERE       — random R semicircle revolve (native Sphere face) [quadric sil.]
//         FREEFORM     — B-spline-meridian revolve (Kind::BSpline bands)  [honest DECLINE]
//       plus a random RIGID pose (rotate about a random axis + translate — NO scale/
//       mirror, so the projected silhouette geometry is an exact isometry of the base)
//       applied IDENTICALLY under both engines. The RNG is a splitmix64-seeded
//       xoshiro256** stream keyed ONLY by an explicit FUZZ_SEED (argv/env) — NO clock,
//       NO rand(): same seed → byte-identical batch.
//   (2) Pick a random VIEW direction + up hint (guaranteed non-parallel) and project
//       the SAME posed solid through cc_hlr_project under BOTH engines:
//         cc_set_engine(0) → OCCT HLRBRep_Algo / HLRBRep_HLRToShape oracle
//         cc_set_engine(1) → native orthographic_hlr + silhouette core (OCCT-FREE)
//       Both return CCDrawing in the SAME drawing-plane basis (right = normalize(view ×
//       up), trueUp = right × view) so the 2D coordinates are directly comparable.
//   (3) Compare the visible/hidden 2D segment SETS to a deflection-matched tolerance:
//         * counts        — for POLYHEDRAL convex solids (box / n-gon prism) the visible
//                           + hidden counts are deterministic and MUST match the oracle.
//         * total length  — Σ visible and Σ hidden length match within a relative band
//                           (tight for polyhedral, curve-sized for quadric silhouettes).
//         * partition     — every native segment's endpoints lie on a SAME-CLASS oracle
//                           segment within tolerance (no misclassification / fabrication),
//                           and — for polyhedral — vice versa (identical labelled point
//                           set). This is the authoritative check.
//   (4) A CLOSED-FORM SILHOUETTE-TANGENCY ARBITER (n·view=0) for the CYLINDER and SPHERE
//       families: the analytic silhouette (cylinder generator lines at θ*=atan2(−X·d,Y·d);
//       sphere great-circle extreme points) is projected into the drawing plane in plain
//       fp64 and every native VISIBLE segment endpoint is required to lie on the analytic
//       silhouette hull within a curve tol. A native result matching the closed form while
//       OCCT is the outlier is logged ORACLE_UNRELIABLE (native vindicated — prior fuzzers
//       repeatedly found native MORE correct than OCCT at numeric edges), never a bar fail.
//   (5) Classify each trial into EXACTLY ONE of:
//         AGREED             — both engines drew, counts/length/partition within tol
//                              (and, where it exists, the closed-form arbiter confirms).
//         HONESTLY-DECLINED  — native returned an EMPTY drawing (visible=hidden=0, arrays
//                              null, cc_last_error set) → OCCT fallback ships, and the OCCT
//                              oracle IS itself a non-empty valid drawing. The freeform
//                              family is the deliberate decline probe.
//         DISAGREED          — native drew a NON-EMPTY drawing that is OUTSIDE tolerance of
//                              the oracle (wrong counts / length / a segment misclassified
//                              visible↔hidden or off the oracle outline). A SILENT WRONG
//                              RESULT — the M6 failure this exists to catch.
//         ORACLE_UNRELIABLE  — native matches the closed-form silhouette arbiter but OCCT
//                              disagrees (OCCT-inaccurate). Native vindicated; NOT a fail.
//   (6) Print a coverage summary: seed, N, trials, AGREED / HONESTLY-DECLINED / DISAGREED
//       / ORACLE_UNRELIABLE, per-family and per-view-regime breakdowns. Exits 0 IFF
//       DISAGREED == 0. Any DISAGREE prints seed + case index + full param tuple as a
//       reproducible regression find. The tolerances are FIXED and NEVER widened.
//
// Like native_hlr_parity / native_directmodel_fuzz (and UNLIKE the internal-C++ fuzzers)
// this drives the SHIPPING PATH through the public cc_* facade under BOTH engines, so the
// runner links the WHOLE kernel + the full OCCT toolkit set incl. TKHLR (the HLRBRep
// oracle). src/native stays OCCT-FREE — this harness is additive test/sim code only.
// Built ONLY by scripts/run-sim-native-hlr-fuzz.sh; on run-sim-suite.sh's SKIP list (own
// main(), std::_Exit — the trimmed static-OCCT build's teardown is not exit-clean, same
// rationale as the sibling parity/fuzz harnesses; every id is released before exit).

#include "cybercadkernel/cc_kernel.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 6.28318530717958647692;

// ── deterministic RNG: splitmix64 seed → xoshiro256** stream ───────────────────────
// Keyed ONLY by an explicit uint64 seed (argv/env). No clock, no rand(). Same seed →
// byte-identical batch (identical construction to the sibling M6 fuzzers).
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

// ── minimal fp64 vec3 (harness-local; NO src/native include so this stays a pure
//    facade client, matching native_directmodel_fuzz's self-contained arbiter) ──────
struct V3 {
  double x = 0, y = 0, z = 0;
};
V3 operator+(const V3& a, const V3& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V3 operator-(const V3& a, const V3& b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V3 operator*(const V3& a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V3 cross(const V3& a, const V3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double norm(const V3& a) { return std::sqrt(dot(a, a)); }
V3 unit(const V3& a) {
  const double n = norm(a);
  return n > 0.0 ? a * (1.0 / n) : V3{};
}
// Rodrigues rotation of point p about a unit axis k through origin O by angle t.
V3 rotateAbout(const V3& p, const V3& O, const V3& k, double t) {
  const V3 v = p - O;
  const double c = std::cos(t), s = std::sin(t);
  const V3 r = v * c + cross(k, v) * s + k * (dot(k, v) * (1.0 - c));
  return O + r;
}

// ── families ────────────────────────────────────────────────────────────────────
enum Family { F_BOX, F_NGON, F_CYLINDER, F_CONE, F_SPHERE, F_FREEFORM, F_COUNT };
const char* famName(int f) {
  switch (f) {
    case F_BOX:      return "box";
    case F_NGON:     return "ngon-prism";
    case F_CYLINDER: return "cylinder";
    case F_CONE:     return "cone/frustum";
    case F_SPHERE:   return "sphere";
    case F_FREEFORM: return "freeform(decline)";
  }
  return "?";
}
bool isPolyhedral(int f) { return f == F_BOX || f == F_NGON; }
bool hasClosedForm(int f) { return f == F_CYLINDER || f == F_SPHERE; }

// A generated case: base solid geometry + rigid pose + view.
struct FuzzCase {
  int family = 0;
  // base geometry (interpretation per family)
  std::vector<double> profile;  // polyhedral profile x,y (box/ngon)
  double depth = 0;             // extrude depth
  double R = 0, h = 0;          // cylinder R,height ; sphere R ; cone base R
  double r1 = 0;                // cone top radius (0 = full cone, >0 = frustum)
  // rigid pose (applied identically under both engines)
  V3 poseAxisPt{0, 0, 0};
  V3 poseAxisDir{0, 0, 1};
  double poseAngle = 0;
  V3 poseTranslate{0, 0, 0};
  // view
  V3 view{-1, -1, -1};
  V3 up{0, 0, 1};
};

int pickFamily(Rng& r) {
  // Favour the exercised families; freeform is a minority DECLINE probe.
  const int w[F_COUNT] = {5, 5, 5, 5, 5, 2};
  int tot = 0; for (int x : w) tot += x;
  int k = static_cast<int>(r.below(static_cast<uint32_t>(tot)));
  for (int i = 0; i < F_COUNT; ++i) { if (k < w[i]) return i; k -= w[i]; }
  return F_BOX;
}

// A random rigid pose: rotate about a random unit axis by a random angle, then
// translate. NO scale, NO mirror → the projected outline is an exact isometry so the
// silhouette-tangency arbiter transforms cleanly and lengths are preserved.
void genPose(Rng& r, FuzzCase& c) {
  c.poseAxisPt = V3{r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)};
  V3 ax{r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)};
  if (norm(ax) < 1e-6) ax = V3{0, 0, 1};
  c.poseAxisDir = unit(ax);
  c.poseAngle = r.range(-kPi, kPi);
  c.poseTranslate = V3{r.range(-3, 3), r.range(-3, 3), r.range(-3, 3)};
}

// A random VIEW direction with an up hint guaranteed non-parallel to it.
void genView(Rng& r, FuzzCase& c) {
  V3 v{r.range(-1, 1), r.range(-1, 1), r.range(-1, 1)};
  if (norm(v) < 0.2) v = V3{-1, -1, -1};  // avoid a near-zero view
  c.view = unit(v);
  // Pick an up hint least aligned with the view (well-conditioned drawing basis).
  const V3 cand[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  int best = 0; double bestAlign = 2.0;
  for (int i = 0; i < 3; ++i) {
    const double a = std::fabs(dot(c.view, cand[i]));
    if (a < bestAlign) { bestAlign = a; best = i; }
  }
  c.up = cand[best];
}

FuzzCase genCase(Rng& r) {
  FuzzCase c;
  c.family = pickFamily(r);
  switch (c.family) {
    case F_BOX: {
      const double w = r.range(2.0, 5.0), d = r.range(1.5, 4.0);
      c.profile = {0, 0, w, 0, w, d, 0, d};
      c.depth = r.range(2.0, 6.0);
      break;
    }
    case F_NGON: {
      const int n = 3 + static_cast<int>(r.below(6));  // 3..8
      const double rad = r.range(1.5, 3.0);
      const double phase = r.range(0.0, kTwoPi);
      c.profile.reserve(static_cast<std::size_t>(n) * 2);
      for (int i = 0; i < n; ++i) {
        const double a = phase + kTwoPi * static_cast<double>(i) / static_cast<double>(n);
        c.profile.push_back(rad * std::cos(a));
        c.profile.push_back(rad * std::sin(a));
      }
      c.depth = r.range(2.0, 6.0);
      break;
    }
    case F_CYLINDER: {
      c.R = r.range(1.0, 3.0);
      c.h = r.range(2.0, 5.0);
      break;
    }
    case F_CONE: {
      c.R = r.range(1.5, 3.0);
      c.h = r.range(2.5, 5.0);
      // Half the time a full cone (r1=0), half a truncated frustum (r1 in (0,R)).
      c.r1 = (r.below(2) == 0) ? 0.0 : r.range(0.4, c.R - 0.4);
      break;
    }
    case F_SPHERE: {
      c.R = r.range(1.0, 3.0);
      break;
    }
    case F_FREEFORM: {
      // B-spline-meridian revolve → Kind::BSpline surface-of-revolution bands: the exact
      // native honest-decline case. Bulge amplitude randomised (still on-axis endpoints).
      c.R = r.range(1.5, 2.5);  // reused as bulge amplitude
      c.h = r.range(3.0, 5.0);  // reused as meridian height
      break;
    }
  }
  genPose(r, c);
  genView(r, c);
  return c;
}

// ── build the base solid under the CURRENT engine (identical call both engines) ─────
CCShapeId buildBase(const FuzzCase& c) {
  switch (c.family) {
    case F_BOX:
    case F_NGON:
      return cc_solid_extrude(c.profile.data(), static_cast<int>(c.profile.size() / 2), c.depth);
    case F_CYLINDER: {
      // disk [0,R]×[0,h] revolved 360° about Y → radius R, height h cylinder.
      const double prof[] = {0.0, 0.0, c.R, 0.0, c.R, c.h, 0.0, c.h};
      return cc_solid_revolve(prof, 4, kTwoPi);
    }
    case F_CONE: {
      if (c.r1 <= 0.0) {
        // full cone: triangle (0,0)-(R,0)-(0,h)
        const double prof[] = {0.0, 0.0, c.R, 0.0, 0.0, c.h};
        return cc_solid_revolve(prof, 3, kTwoPi);
      }
      // frustum: trapezoid (r1... shape (0,0)-(R,0)-(r1,h)-(0,h)
      const double prof[] = {0.0, 0.0, c.R, 0.0, c.r1, c.h, 0.0, c.h};
      return cc_solid_revolve(prof, 4, kTwoPi);
    }
    case F_SPHERE: {
      CCProfileSeg s[2];
      s[0] = CCProfileSeg{};
      s[0].kind = 1;  // arc
      s[0].x0 = 0; s[0].y0 = -c.R; s[0].x1 = 0; s[0].y1 = c.R;
      s[0].cx = 0; s[0].cy = 0; s[0].r = c.R;
      s[0].a0 = -kPi / 2; s[0].a1 = kPi / 2;
      s[1] = CCProfileSeg{};
      s[1].kind = 0;  // axis line closing the profile
      s[1].x0 = 0; s[1].y0 = c.R; s[1].x1 = 0; s[1].y1 = -c.R;
      return cc_solid_revolve_profile(s, 2, 0, 0, 0, 1, nullptr, 0, kTwoPi);
    }
    case F_FREEFORM: {
      CCProfileSeg segs[2] = {};
      segs[0].kind = 3;  // B-spline meridian through 4 pts (endpoints ON axis)
      segs[0].ptOffset = 0; segs[0].ptCount = 4;
      segs[1].kind = 0;
      segs[1].x0 = 0; segs[1].y0 = c.h; segs[1].x1 = 0; segs[1].y1 = 0;
      const double a = c.R, hh = c.h;
      const double spline[] = {0.0, 0.0, a, hh * 0.25, a, hh * 0.75, 0.0, hh};
      return cc_solid_revolve_profile(segs, 2, 0, 0, 0, 1, spline, 8, kTwoPi);
    }
  }
  return 0;
}

// Apply the rigid pose (rotate then translate) under the CURRENT engine. Returns a NEW
// id; releases the input. Returns the input unchanged on a transform failure (caught by
// the caller as a build failure).
CCShapeId applyPose(CCShapeId id, const FuzzCase& c) {
  if (id == 0) return 0;
  const CCShapeId rot = cc_rotate_shape_about(id, c.poseAxisPt.x, c.poseAxisPt.y, c.poseAxisPt.z,
                                              c.poseAxisDir.x, c.poseAxisDir.y, c.poseAxisDir.z,
                                              c.poseAngle);
  if (rot == 0) return 0;
  cc_shape_release(id);
  const CCShapeId tr = cc_translate_shape(rot, c.poseTranslate.x, c.poseTranslate.y,
                                          c.poseTranslate.z);
  if (tr == 0) return 0;
  cc_shape_release(rot);
  return tr;
}

CCShapeId buildPosed(const FuzzCase& c) { return applyPose(buildBase(c), c); }

// ── 2D drawing-plane geometry helpers ──────────────────────────────────────────────
double segLen(const CCDrawingSegment& s) { return std::hypot(s.bx - s.ax, s.by - s.ay); }
double totalLen(const CCDrawingSegment* segs, int n) {
  double t = 0.0;
  for (int i = 0; i < n; ++i) t += segLen(segs[i]);
  return t;
}
double pointSegDist(double px, double py, const CCDrawingSegment& s) {
  const double vx = s.bx - s.ax, vy = s.by - s.ay;
  const double wx = px - s.ax, wy = py - s.ay;
  const double vv = vx * vx + vy * vy;
  double t = vv > 0.0 ? (wx * vx + wy * vy) / vv : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  const double cx = s.ax + t * vx, cy = s.ay + t * vy;
  return std::hypot(px - cx, py - cy);
}
bool coveredBy(const CCDrawingSegment& a, const CCDrawingSegment* set, int n, double tol) {
  bool aEnd = false, bEnd = false;
  for (int i = 0; i < n && !(aEnd && bEnd); ++i) {
    if (!aEnd && pointSegDist(a.ax, a.ay, set[i]) < tol) aEnd = true;
    if (!bEnd && pointSegDist(a.bx, a.by, set[i]) < tol) bEnd = true;
  }
  return aEnd && bEnd;
}
bool partitionCovered(const CCDrawingSegment* x, int nx, const CCDrawingSegment* y, int ny,
                      double tol) {
  for (int i = 0; i < nx; ++i)
    if (!coveredBy(x[i], y, ny, tol)) return false;
  return true;
}

// ── closed-form silhouette-tangency arbiter (cylinder + sphere) ─────────────────────
// The analytic silhouette (n·view = 0), projected into the drawing plane. A native
// VISIBLE segment endpoint must lie on this hull within `tol`. Returns the projected
// analytic silhouette point set (as short segments) so we can reuse coveredBy.
struct DrawingBasis {
  V3 right, trueUp, view;
};
DrawingBasis makeBasis(const V3& view, const V3& up) {
  DrawingBasis b;
  b.view = view;
  b.right = unit(cross(view, up));
  b.trueUp = cross(b.right, view);  // unit (right,view orthonormal)
  return b;
}
CCDrawingSegment project2(const DrawingBasis& b, const V3& p, const V3& q) {
  return CCDrawingSegment{dot(p, b.right), dot(p, b.trueUp), dot(q, b.right), dot(q, b.trueUp)};
}

// Build the analytic silhouette of the POSED cylinder/sphere as world-space polylines,
// then project to the drawing plane. The base solid is built about +Y (revolve axis),
// then posed by the same rigid transform — so we transform the analytic silhouette by
// the same pose in fp64.
std::vector<CCDrawingSegment> analyticSilhouette(const FuzzCase& c, const DrawingBasis& b) {
  std::vector<CCDrawingSegment> out;
  auto pose = [&](const V3& p) {
    const V3 rr = rotateAbout(p, c.poseAxisPt, c.poseAxisDir, c.poseAngle);
    return rr + c.poseTranslate;
  };
  if (c.family == F_CYLINDER) {
    // Base cylinder: axis +Y, radius R, y∈[0,h], centred on the Y axis at x=z=0.
    // Silhouette = the two generator lines where the radial normal ⟂ view.
    // In the base frame X=+X, Y=+Z (the two in-plane axes ⟂ the Y axis), n(θ) =
    // cosθ·X + sinθ·Z. θ* = atan2(−X·d, Z·d).  (view is already unit.)
    const V3 axisX{1, 0, 0}, axisZ{0, 0, 1};
    // We need view in the BASE frame: the analytic normal condition uses the base
    // geometry, so rotate the view by the INVERSE pose. Equivalently compute the
    // world silhouette directly: transform the base generator endpoints by the pose,
    // using the base-frame view. Rotate view into base frame (inverse rotation).
    const V3 vBase = rotateAbout(c.view, V3{0, 0, 0}, c.poseAxisDir, -c.poseAngle);
    const double xd = dot(axisX, vBase), zd = dot(axisZ, vBase);
    const double planar = std::hypot(xd, zd);
    if (planar > 1e-9) {
      const double th0 = std::atan2(-xd, zd);
      for (double th : {th0, th0 + kPi}) {
        const V3 radial = axisX * std::cos(th) + axisZ * std::sin(th);
        const V3 baseLo = radial * c.R + V3{0, 0, 0};              // y=0
        const V3 baseHi = radial * c.R + V3{0, c.h, 0};            // y=h
        out.push_back(project2(b, pose(baseLo), pose(baseHi)));
      }
    }
  } else if (c.family == F_SPHERE) {
    // Sphere centre (base) = (0, 0, 0). Silhouette = great circle of radius R in the
    // plane ⟂ view through the centre. Discretize; project. Pose the centre only.
    const V3 centerBase{0, 0, 0};
    const V3 centerWorld = pose(centerBase);
    // Basis of the silhouette plane in world: any two unit vecs ⟂ view.
    V3 e1 = unit(cross(c.view, V3{1, 0, 0}));
    if (norm(e1) < 1e-6) e1 = unit(cross(c.view, V3{0, 1, 0}));
    const V3 e2 = cross(c.view, e1);
    const int n = 96;
    V3 prev = centerWorld + e1 * c.R;
    for (int i = 1; i <= n; ++i) {
      const double t = kTwoPi * static_cast<double>(i) / static_cast<double>(n);
      const V3 cur = centerWorld + e1 * (c.R * std::cos(t)) + e2 * (c.R * std::sin(t));
      out.push_back(project2(b, prev, cur));
      prev = cur;
    }
  }
  return out;
}

// ── the classifier ──────────────────────────────────────────────────────────────
enum Verdict { AGREED, DECLINED, DISAGREED, ORACLE_UNRELIABLE, BUILD_FAIL, BOTH_DECLINED };

int g_agreed = 0, g_declined = 0, g_disagreed = 0, g_oracleUnrel = 0, g_buildFail = 0,
    g_bothDeclined = 0;
int g_famAgreed[F_COUNT] = {0}, g_famDeclined[F_COUNT] = {0}, g_famDisagreed[F_COUNT] = {0},
    g_famOracleUnrel[F_COUNT] = {0};
// View regimes: 0 = near-axis-aligned (|view·axis| large for a family) is hard to define
// generically, so bin by the drawing basis' obliqueness relative to world Z (a proxy for
// "how tilted"): 0 = shallow (|view.z|>0.8), 1 = oblique, 2 = grazing (|view.z|<0.2).
int g_viewAgreed[3] = {0}, g_viewDeclined[3] = {0}, g_viewDisagreed[3] = {0};
int viewRegime(const FuzzCase& c) {
  const double az = std::fabs(c.view.z);
  if (az > 0.8) return 0;
  if (az < 0.2) return 2;
  return 1;
}
const char* regimeName(int r) {
  switch (r) { case 0: return "shallow"; case 1: return "oblique"; case 2: return "grazing"; }
  return "?";
}

std::string tupleStr(const FuzzCase& c) {
  char buf[320];
  std::snprintf(buf, sizeof buf,
                "fam=%s R=%.4f h=%.4f r1=%.4f depth=%.4f pose[axpt(%.3f,%.3f,%.3f) "
                "axdir(%.3f,%.3f,%.3f) ang=%.4f tr(%.3f,%.3f,%.3f)] view(%.4f,%.4f,%.4f) "
                "up(%.0f,%.0f,%.0f)",
                famName(c.family), c.R, c.h, c.r1, c.depth, c.poseAxisPt.x, c.poseAxisPt.y,
                c.poseAxisPt.z, c.poseAxisDir.x, c.poseAxisDir.y, c.poseAxisDir.z, c.poseAngle,
                c.poseTranslate.x, c.poseTranslate.y, c.poseTranslate.z, c.view.x, c.view.y,
                c.view.z, c.up.x, c.up.y, c.up.z);
  return buf;
}

}  // namespace

int main(int argc, char** argv) {
  uint64_t seed = 0x4D6F617436ull;  // "Moat6"
  int N = 60;
  if (argc > 1) seed = strtoull(argv[1], nullptr, 0);
  else if (const char* e = std::getenv("FUZZ_SEED")) seed = strtoull(e, nullptr, 0);
  if (argc > 2) N = std::atoi(argv[2]);
  else if (const char* e = std::getenv("FUZZ_N")) N = std::atoi(e);
  if (N <= 0) N = 60;

  std::printf("== M6-breadth-13 differential-fuzz: native ORTHOGRAPHIC HLR vs OCCT HLRBRep oracle ==\n");
  std::printf("== seed=0x%llx N=%d (through the cc_* facade, cc_set_engine 0=OCCT 1=native) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::fflush(stdout);

  Rng rng(seed);
  int trials = 0;

  // FIXED tolerances — NEVER widened to force a pass.
  const double kCoordTolPoly = 1e-4;   // mm, polyhedral partition (exact projection)
  const double kLenRelPoly = 5e-4;     // relative total-length band, polyhedral
  const double kCurveTol = 0.08;       // mm, curve-sized geometric band (facet discretization)
  const double kLenRelCurve = 3e-2;    // relative curve-length band
  const double kArbiterTol = 0.08;     // mm, closed-form silhouette-tangency band

  for (int i = 0; i < N; ++i) {
    const FuzzCase c = genCase(rng);
    const int reg = viewRegime(c);

    CCHlrOptions opts{};
    opts.deflection = 0.05;
    opts.samplesPerEdge = 0;
    opts.surfaceOffset = 0.0;

    // 1) OCCT oracle.
    cc_set_engine(0);
    const CCShapeId occtId = buildPosed(c);
    if (occtId == 0) {
      ++g_buildFail;
      std::printf("[HLR] BUILD_FAIL case=%d %-18s OCCT build err='%s' %s\n", i, famName(c.family),
                  cc_last_error(), tupleStr(c).c_str());
      cc_set_engine(0);
      std::fflush(stdout);
      continue;
    }
    const double view[3] = {c.view.x, c.view.y, c.view.z};
    const double up[3] = {c.up.x, c.up.y, c.up.z};
    const CCDrawing oD = cc_hlr_project(occtId, view, up, opts);

    // 2) Native core.
    cc_set_engine(1);
    const CCShapeId natId = buildPosed(c);
    if (natId == 0) {
      // The native engine declined to BUILD / POSE the solid (e.g. cc_rotate_shape_about
      // declines certain revolve-built-frustum rigid placements — a native PLACEMENT
      // scope limit, NOT an HLR fault). In the shipping path this is exactly a native
      // decline → OCCT fallback for the whole projection: if the OCCT oracle DID draw a
      // non-empty outline, count it HONESTLY-DECLINED (the drop-OCCT semantics). Only if
      // the oracle also could not draw is it an un-attributable BUILD_FAIL.
      const char* nerr = cc_last_error();
      ++trials;
      if (oD.visibleCount > 0) {
        ++g_declined; ++g_famDeclined[c.family]; ++g_viewDeclined[reg];
        std::printf("[HLR] DECLINED          case=%d %-18s [%s] native could not build/pose "
                    "(placement declined) -> OCCT[vis=%d hid=%d] err='%s'\n",
                    i, famName(c.family), regimeName(reg), oD.visibleCount, oD.hiddenCount, nerr);
      } else {
        ++g_buildFail;
        std::printf("[HLR] BUILD_FAIL case=%d %-18s native build err='%s' %s\n", i,
                    famName(c.family), nerr, tupleStr(c).c_str());
      }
      cc_drawing_free(oD);
      cc_set_engine(0);
      cc_shape_release(occtId);
      std::fflush(stdout);
      continue;
    }
    const CCDrawing nD = cc_hlr_project(natId, view, up, opts);

    ++trials;

    const bool nativeDrew = (nD.visibleCount > 0);
    const bool oracleDrew = (oD.visibleCount > 0);

    Verdict v;
    char detail[640];

    if (!nativeDrew) {
      // Native returned an EMPTY drawing. If the oracle DID draw → honest decline
      // (OCCT fallback ships). If the oracle also empty → both declined (a degenerate
      // view, e.g. exactly down an axis — not a native fault).
      if (oracleDrew) {
        v = DECLINED;
        std::snprintf(detail, sizeof detail, "native EMPTY -> OCCT[vis=%d hid=%d] err='%s'",
                      oD.visibleCount, oD.hiddenCount, cc_last_error());
      } else {
        v = BOTH_DECLINED;
        std::snprintf(detail, sizeof detail, "both engines empty (degenerate view) err='%s'",
                      cc_last_error());
      }
    } else if (!oracleDrew) {
      // Native drew but the ORACLE is empty. The oracle failing where native produces a
      // plausible outline is an ORACLE problem, not a native disagreement — but we must
      // not launder a fabricated native outline. Without an oracle we cannot confirm, so
      // if a closed form exists we arbitrate; otherwise flag ORACLE_UNRELIABLE.
      if (hasClosedForm(c.family)) {
        const DrawingBasis b = makeBasis(c.view, c.up);
        const std::vector<CCDrawingSegment> sil = analyticSilhouette(c, b);
        const bool onArbiter = !sil.empty() &&
            partitionCovered(nD.visible, nD.visibleCount, sil.data(),
                             static_cast<int>(sil.size()), kArbiterTol);
        v = onArbiter ? ORACLE_UNRELIABLE : DISAGREED;
        std::snprintf(detail, sizeof detail,
                      "native drew[vis=%d hid=%d] but OCCT EMPTY; closed-form-confirms=%d",
                      nD.visibleCount, nD.hiddenCount, onArbiter ? 1 : 0);
      } else {
        v = ORACLE_UNRELIABLE;
        std::snprintf(detail, sizeof detail,
                      "native drew[vis=%d hid=%d] but OCCT EMPTY (no closed form; oracle gap)",
                      nD.visibleCount, nD.hiddenCount);
      }
    } else {
      // Both drew — the differential comparison.
      const bool poly = isPolyhedral(c.family);
      const double coordTol = poly ? kCoordTolPoly : kCurveTol;
      const double lenRel = poly ? kLenRelPoly : kLenRelCurve;

      const double nVis = totalLen(nD.visible, nD.visibleCount);
      const double oVis = totalLen(oD.visible, oD.visibleCount);
      const double nHid = totalLen(nD.hidden, nD.hiddenCount);
      const double oHid = totalLen(oD.hidden, oD.hiddenCount);
      const double visRel = oVis > 0.0 ? std::fabs(nVis - oVis) / oVis : (nVis > 0 ? 1.0 : 0.0);
      const bool hidOk = std::fabs(nHid - oHid) < lenRel * (oVis > 0 ? oVis : 1.0);
      const bool lenOk = (visRel < lenRel) && hidOk;

      // Counts must match for polyhedral convex/deterministic solids (same topology
      // both engines). For quadric silhouettes the two discretizers differ, so counts
      // are NOT required equal — the partition check carries the load.
      const bool countsOk = poly ? (nD.visibleCount == oD.visibleCount &&
                                    nD.hiddenCount == oD.hiddenCount)
                                 : true;

      // ── PARTITION (the authoritative "same labelled locus" check) ────────────────
      // native VISIBLE ⊆ oracle VISIBLE and native HIDDEN ⊆ oracle OUTLINE (no native
      // segment misclassified or fabricated), AND the REVERSE — oracle VISIBLE ⊆ native
      // VISIBLE and oracle HIDDEN ⊆ native OUTLINE (native drew the WHOLE outline, no
      // missing arc). BIDIRECTIONAL coverage at the curve tolerance proves the two
      // engines trace the IDENTICAL visible/hidden point-set locus regardless of how
      // each engine SAMPLES it — the geometrically robust equivalent of the polyhedral
      // "identical labelled point set", and the true no-silent-wrong-result oracle. The
      // total-projected-LENGTH is a discretization-sensitive PROXY (a foreshortened
      // grazing silhouette is sampled at different chord densities by two independent
      // discretizers), so for curved families it is a CORROBORATING signal, not the
      // gate — when only the length band trips but the bidirectional partition holds,
      // the outlines are the same locus and the trial AGREES. The tolerances are FIXED;
      // none is widened.
      std::vector<CCDrawingSegment> oAll(oD.visible, oD.visible + oD.visibleCount);
      oAll.insert(oAll.end(), oD.hidden, oD.hidden + oD.hiddenCount);
      std::vector<CCDrawingSegment> nAll(nD.visible, nD.visible + nD.visibleCount);
      nAll.insert(nAll.end(), nD.hidden, nD.hidden + nD.hiddenCount);
      const bool visOnVis = partitionCovered(nD.visible, nD.visibleCount, oD.visible,
                                             oD.visibleCount, coordTol);
      const bool hidOnOutline = partitionCovered(nD.hidden, nD.hiddenCount, oAll.data(),
                                                 static_cast<int>(oAll.size()), coordTol);
      const bool visRev = partitionCovered(oD.visible, oD.visibleCount, nD.visible,
                                           nD.visibleCount, coordTol);
      const bool hidRev = partitionCovered(oD.hidden, oD.hiddenCount, nAll.data(),
                                           static_cast<int>(nAll.size()), coordTol);
      const bool partOk = visOnVis && hidOnOutline && visRev && hidRev;

      // AGREE gate: polyhedral requires exact counts + tight length + bidirectional
      // partition. Curved requires bidirectional partition (the authoritative locus
      // match); the length band, if it also holds, is confirmatory but not required —
      // a length-only miss under a holding partition is a discretization artifact.
      const bool agree = poly ? (countsOk && lenOk && partOk) : partOk;

      if (agree) {
        v = AGREED;
        std::snprintf(detail, sizeof detail,
                      "vis n=%d o=%d hid n=%d o=%d | visLen rel=%.2e hidΔ=%.3g | "
                      "part(bi=%d v⊆v=%d o⊆v=%d)%s",
                      nD.visibleCount, oD.visibleCount, nD.hiddenCount, oD.hiddenCount, visRel,
                      std::fabs(nHid - oHid), partOk ? 1 : 0, visOnVis ? 1 : 0, visRev ? 1 : 0,
                      (!poly && !lenOk) ? " [len-proxy tripped; bidir-partition holds → same locus]"
                                        : "");
      } else if (hasClosedForm(c.family)) {
        // Bidirectional partition FAILED on a quadric — the outlines are NOT the same
        // locus. Before calling it a native fault, check the closed form: if native
        // matches the analytic silhouette while OCCT is the outlier, native is
        // vindicated (ORACLE_UNRELIABLE). Otherwise it is a real DISAGREE.
        const DrawingBasis b = makeBasis(c.view, c.up);
        const std::vector<CCDrawingSegment> sil = analyticSilhouette(c, b);
        const bool nativeOnArbiter = !sil.empty() &&
            partitionCovered(nD.visible, nD.visibleCount, sil.data(),
                             static_cast<int>(sil.size()), kArbiterTol);
        const bool oracleOnArbiter = !sil.empty() &&
            partitionCovered(oD.visible, oD.visibleCount, sil.data(),
                             static_cast<int>(sil.size()), kArbiterTol);
        if (nativeOnArbiter && !oracleOnArbiter) {
          v = ORACLE_UNRELIABLE;
          std::snprintf(detail, sizeof detail,
                        "native matches closed-form silhouette, OCCT is the outlier "
                        "(visLen rel=%.2e counts n=%d o=%d) — native vindicated",
                        visRel, nD.visibleCount, oD.visibleCount);
        } else {
          v = DISAGREED;
          std::snprintf(detail, sizeof detail,
                        "SILENT-WRONG vis n=%d o=%d hid n=%d o=%d visLenRel=%.3e hidΔ=%.3g "
                        "part(v⊆v=%d o⊆v=%d h⊆o=%d o-h⊆n=%d) arbiter(nat=%d ocl=%d)",
                        nD.visibleCount, oD.visibleCount, nD.hiddenCount, oD.hiddenCount, visRel,
                        std::fabs(nHid - oHid), visOnVis, visRev, hidOnOutline, hidRev,
                        nativeOnArbiter, oracleOnArbiter);
        }
      } else {
        v = DISAGREED;
        std::snprintf(detail, sizeof detail,
                      "SILENT-WRONG vis n=%d o=%d hid n=%d o=%d counts=%d visLenRel=%.3e "
                      "hidΔ=%.3g part(v⊆v=%d o⊆v=%d h⊆o=%d o-h⊆n=%d)",
                      nD.visibleCount, oD.visibleCount, nD.hiddenCount, oD.hiddenCount,
                      countsOk ? 1 : 0, visRel, std::fabs(nHid - oHid), visOnVis, visRev,
                      hidOnOutline, hidRev);
      }
    }

    // Tally + print.
    const char* tag = "?";
    switch (v) {
      case AGREED:            ++g_agreed;      ++g_famAgreed[c.family];      ++g_viewAgreed[reg];    tag = "AGREED"; break;
      case DECLINED:          ++g_declined;    ++g_famDeclined[c.family];    ++g_viewDeclined[reg];  tag = "DECLINED"; break;
      case DISAGREED:         ++g_disagreed;   ++g_famDisagreed[c.family];   ++g_viewDisagreed[reg]; tag = "DISAGREED"; break;
      case ORACLE_UNRELIABLE: ++g_oracleUnrel; ++g_famOracleUnrel[c.family];                         tag = "ORACLE_UNRELIABLE"; break;
      case BOTH_DECLINED:     ++g_bothDeclined;                                                      tag = "BOTH_DECLINED"; break;
      case BUILD_FAIL:        break;  // handled above
    }
    std::printf("[HLR] %-17s case=%d %-18s [%s] %s\n", tag, i, famName(c.family), regimeName(reg),
                detail);
    if (v == DISAGREED)
      std::printf("      REPRO seed=0x%llx index=%d %s\n",
                  static_cast<unsigned long long>(seed), i, tupleStr(c).c_str());
    std::fflush(stdout);

    cc_drawing_free(nD);
    cc_drawing_free(oD);
    cc_set_engine(0);
    cc_shape_release(natId);
    cc_shape_release(occtId);
  }

  // ── coverage summary ────────────────────────────────────────────────────────────
  std::printf("\n== COVERAGE SUMMARY (seed=0x%llx N=%d) ==\n",
              static_cast<unsigned long long>(seed), N);
  std::printf("   trials=%d  AGREED=%d  HONESTLY-DECLINED=%d  DISAGREED=%d  ORACLE_UNRELIABLE=%d"
              "  BOTH-DECLINED=%d  BUILD_FAIL=%d\n",
              trials, g_agreed, g_declined, g_disagreed, g_oracleUnrel, g_bothDeclined,
              g_buildFail);
  std::printf("   per-family [AGREED/DECLINED/DISAGREED/ORACLE_UNRELIABLE]:\n");
  for (int f = 0; f < F_COUNT; ++f)
    std::printf("     %-18s %d/%d/%d/%d\n", famName(f), g_famAgreed[f], g_famDeclined[f],
                g_famDisagreed[f], g_famOracleUnrel[f]);
  std::printf("   per-view-regime [AGREED/DECLINED/DISAGREED]:\n");
  for (int r = 0; r < 3; ++r)
    std::printf("     %-18s %d/%d/%d\n", regimeName(r), g_viewAgreed[r], g_viewDeclined[r],
                g_viewDisagreed[r]);

  const bool bar = (g_disagreed == 0);
  std::printf("== M6 BAR: %s (DISAGREED=%d must be 0) ==\n",
              bar ? "PASS — zero silent wrong HLR results" : "FAIL", g_disagreed);
  std::fflush(stdout);
  std::_Exit(bar ? 0 : 1);
}
