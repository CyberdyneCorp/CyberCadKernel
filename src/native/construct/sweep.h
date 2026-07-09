// SPDX-License-Identifier: Apache-2.0
//
// sweep.h — native swept solid (Phase 4 #4b, Tier C `native-construction`).
//
// Clean-room, OCCT-FREE builder that sweeps a CLOSED planar profile along a 3D
// polyline path (spine), mirroring the subset of OCCT BRepOffsetAPI_MakePipe the
// cc_solid_sweep facade uses (see src/engine/occt/occt_construct.cpp solid_sweep):
// the profile is centred on its centroid, placed PERPENDICULAR to the path tangent
// at the START, then transported along the spine with a CONSTANT frame (the same law
// MakePipe uses on a planar spine — see the TRANSPORT note below), and closed with
// planar end caps.
//
// ── FRAMING (matches the cc_solid_sweep contract + the OCCT oracle) ───────────
// At the path start the profile's local axes are, exactly as the oracle builds
// them: local x = normalize(cross(tangent, ref)), local y = normalize(cross(x,
// tangent)), where ref = +Y unless the tangent is near-vertical (|tan.y| > 0.9), in
// which case ref = +X so the cross products never collapse. For a horizontal tangent
// this is (x, y) = (cross(tan, +Y), +Y) — the contract's "local x = cross(tangent,
// +Y), local y = +Y". A profile point (px,py) maps to
//   world = C(v) + (px − cx)·x + (py − cy)·y   with (cx,cy) the profile centroid.
//
// ── TRANSPORT: CONSTANT frame (matches OCCT's planar corrected-Frenet law) ──────
// This is the crux, and it is CALIBRATED AGAINST THE ORACLE, not chosen for looks.
// The cc_solid_sweep oracle is BRepOffsetAPI_MakePipe, whose default sweep law is
// GeomFill_CorrectedFrenet. For a PLANAR spine that law degenerates — verified in
// OCCT/src GeomFill_CorrectedFrenet.cxx: when FindPlane succeeds the rotation
// `EvolAroundT` collapses to a Law_Constant — so the section is transported with a
// CONSTANT orientation (its local x/y stay fixed in world), i.e. the profile is
// merely TRANSLATED to each spine station, NOT rotated to stay perpendicular to the
// local tangent. We confirmed this numerically against the actual oracle solid on the
// simulator: for a 4×4 square swept along a quarter-arc (R=20, XZ plane), every OCCT
// section keeps width-axis ≈ +X (constant) with the section centre exactly on the
// spine point — a rigidly-translated section, NOT a perpendicular one. So the native
// builder holds the START frame constant across all stations; that is what makes the
// native volume/bbox/centroid AGREE with the MakePipe oracle. (An earlier revision
// used a rotation-minimizing / parallel-transport frame that keeps the section
// perpendicular; it is geometrically "nicer" but produced a DIFFERENT, larger solid
// than the oracle — a real mismatch, correctly rejected by the parity gate. We match
// the oracle, not an idealized pipe.) On a STRAIGHT path the constant frame is anyway
// the only frame ⇒ the sweep is exactly a directional EXTRUDE along the path vector.
//
// ── WHAT IS NATIVE (honest — verified watertight; anything else ⇒ NULL Shape so
//    the engine falls through to OCCT, it NEVER fakes a wrong shape) ───────────
//   NATIVE (this builder returns a real solid):
//     * STRAIGHT path (all path points collinear, any count ≥ 2). The frame is
//       constant, every side face is a PLANAR quad, and the result is a directional
//       prism of the profile along the path vector — volume EXACT (profile area ×
//       path length), always watertight, matching MakePipe on a straight spine. This
//       is Tier-C case (a) in openspec/NATIVE-REWRITE.md.
//     * SMOOTH CURVED but PLANAR spine (case (b)): all spine points coplanar, gentle
//       curvature. The constant OCCT start frame is translated to every station; each
//       profile edge spans one bilinear RULED band per spine segment (loft's
//       ruledSideFace helper) sharing per-station vertex rings, the two ends are
//       planar caps → a watertight solid welded by the two-stage tessellator (the
//       same machinery that welds the Tier-B loft). Because the frame is the SAME law
//       OCCT uses on a planar spine, the native solid matches the MakePipe oracle
//       within the deflection bound (vol/bbox/centroid), not merely "a" valid tube.
//   DEFERRED to OCCT (build_sweep returns NULL — verified fall-through):
//     * a NON-PLANAR curved spine: OCCT then uses the genuine (non-constant)
//       corrected-Frenet rotation law, which the constant native frame does NOT
//       reproduce; rather than ship a solid that disagrees with the oracle we defer.
//     * a TIGHT-CURVATURE / SELF-INTERSECTING spine: an interior vertex whose turn
//       angle is too sharp, or whose local turning radius is smaller than the
//       profile's circumradius (the swept tube folds through itself on the concave
//       side → a non-manifold self-intersection MakePipe resolves and the native
//       ruled-band assembler cannot). Guarded (see spineTooSharp) and deferred rather
//       than shipping a self-overlapping solid.
//     * a self-intersecting / non-planar / degenerate profile, < 2 path points, or a
//       zero-length start tangent.
//
// ── TWISTED / GUIDED / RAIL sweeps (this batch — Tier C residuals) ────────────────
// The plain cc_solid_sweep above matches BRepOffsetAPI_MakePipe (constant planar
// frame). The three RESIDUAL sweep ops use a DIFFERENT oracle, and — crucially — a
// SIMPLE, REPRODUCIBLE frame law, so most of them are now NATIVE:
//
//   cc_twisted_sweep / cc_guided_sweep — the OCCT oracle
//   (src/engine/occt/occt_construct.cpp twisted_sweep / guided_sweep) is a RULED
//   BRepOffsetAPI_ThruSections through a rotated/scaled section wire built at EVERY
//   path station with a PER-STATION Frenet frame `nrm = normalize(tangent × up)`,
//   `up = (0,1,0)` fixed (fallback nrm = +X when the local tangent is ∥ +Y). A profile
//   point (u,v) is placed at world = C + u·nrm + v·up, after a per-station in-plane
//   rotation (twist·f) and uniform scale. Because that frame is a pure per-station
//   function of the local tangent (NO accumulated corrected-Frenet rotation), the
//   native builder reproduces it EXACTLY: sample the SAME frame + rotation + scale at
//   each station, lay one bilinear RULED band per (profile edge × station segment)
//   with shared per-station vertex rings, and cap both ends → a watertight solid that
//   matches the oracle (ruled ThruSections connects vertex k→k between consecutive
//   sections, so NO section re-alignment is applied — the correspondence is the
//   identity, exactly as the oracle does). This is native whenever the resulting tube
//   does not self-intersect (a real twist can fold a wide section into itself — GUARDED
//   and deferred). The RMF (double-reflection) helper below (rmfFrames) is available
//   for a genuinely twist-free non-planar transport, but the twisted/guided oracle uses
//   the per-station Frenet frame, so those ops use frenetSectionFrames.
//
//   cc_loft_along_rail — the oracle is BRepOffsetAPI_MakePipeShell morphing section A
//   at the rail start into section B at the rail end (RoundCorner transitions at rail
//   kinks). For a STRAIGHT rail the pipe-shell reduces EXACTLY to a ruled loft between
//   the two sections, each placed in the plane perpendicular to the (single) rail
//   tangent (perpendicularFrame: uDir = tan × ref, vDir = tan × uDir, ref = +Z unless
//   |tan·Z| > 0.95 then +X). So a straight rail is NATIVE (reuse build_ruled_loft on
//   the two perpendicular-framed sections). A CURVED / kinked rail is a genuine
//   pipe-shell morph (interior transitions, non-constant frame) the ruled loft does not
//   model → NULL → OCCT MakePipeShell.
//
// ── WHAT IS NATIVE vs DEFERRED FOR THE RESIDUALS (honest) ─────────────────────────
//   NATIVE (self-verified watertight; matches the ThruSections / straight-rail oracle):
//     * cc_twisted_sweep — plain (twist≈0, scale≈1) forwards to build_sweep; a REAL
//       twist/scale builds the per-station Frenet-framed ruled ThruSections tube
//       (build_twist_scale_sweep), provided it does not self-intersect.
//     * cc_guided_sweep — the guide-scaled per-station Frenet ThruSections tube.
//     * cc_loft_along_rail — a STRAIGHT rail (perpendicular-framed ruled loft).
//   DEFERRED to OCCT (NULL → verified fall-through, never faked):
//     * a NON-PLANAR plain cc_solid_sweep spine (genuine corrected-Frenet — see above).
//     * a self-intersecting twisted/guided tube (the section folds through itself —
//       guarded), a degenerate profile / path / guide.
//     * a CURVED / kinked cc_loft_along_rail rail (pipe-shell morph — needs SSI).
//     * a NON-PLANAR station section under twist (all placed sections ARE planar in
//       their station frame, so the caps stay planar — no extra guard needed there).
//
// REFERENCE ORACLE ONLY: BRepOffsetAPI_MakePipe / MakePipeShell / ThruSections and
// GeomFill's frame laws were consulted to confirm the framing, the polyline spine and
// the capped-solid result; nothing is copied. The bilinear parametrization matches
// src/native/math bezierSurfacePoint.
//
// Cognitive complexity (measured, clang-tidy readability-function-cognitive-complexity,
// all within the systems band ≤25–35): build_sweep 19, assembleRingTube 15 (the linear
// station→band ruled-tube assembler shared by the twisted/guided sweeps),
// build_loft_along_rail 12, spineTooSharp 11, rmfFrames 9, sectionSweepUnsafe 9,
// build_section_thrusections 8, build_guided_sweep 7; the remaining frame / planarity
// helpers are ≤5. OCCT-FREE. Header-only. clang++ c++20.
//
#ifndef CYBERCAD_NATIVE_CONSTRUCT_SWEEP_H
#define CYBERCAD_NATIVE_CONSTRUCT_SWEEP_H

#include "native/construct/construct.h"  // detail::planarFace / kProfileTol
#include "native/construct/loft.h"        // detail::ruledSideFace
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>

namespace cybercad::native::construct {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

namespace detail {

// The maximum accumulated twist (radians) per ruled band of a densified twisted sweep.
// build_twisted_sweep densifies a straight spine so each band rotates the section by at
// most this bound; at a gentle per-band rotation the twisted (saddle) ruled tube welds
// robustly watertight (the same regime the landed guided_orient_sweep uses, kMaxPerBand
// ≈ 0.05) and its volume converges to the area-preserving analytic value. Measured: a
// square section twisted π over a 10-long spine welds watertight from ~16 bands upward.
inline constexpr double kMaxBandTwist = 0.05;  // ≈ 2.9° per band

// The maximum tangent turn (radians) per band of a densified curved-rail loft — the
// analogous weld bound for build_loft_along_rail's RMF-transported tube. A rail whose
// per-band turn cannot drop under this within the station cap fails the engine self-
// verify → OCCT. Measured: a quarter-arc rail welds watertight from ~16 stations upward.
inline constexpr double kMaxBandTurn = 0.05;

// The hard cap on internally-densified stations (shared twist/rail bound), so a
// pathological input cannot blow up the tiling; a case needing more than this to weld
// honestly defers to OCCT.
inline constexpr std::size_t kMaxDensifyStations = 512;

// A moving orthonormal section frame at one spine station: the world origin C(v)
// (the spine point) plus the transported local axes x, y (the profile plane) and
// the unit tangent t (the spine direction there). x × y = t (right-handed).
struct SweepFrame {
  math::Point3 origin;  ///< C(v): the spine point
  math::Vec3 x;         ///< transported local X (unit)
  math::Vec3 y;         ///< transported local Y (unit)
  math::Vec3 t;         ///< spine unit tangent
};

// The start reference axis the oracle uses: +Y unless the tangent is near-vertical
// (then +X), so cross(tangent, ref) never collapses.
inline math::Vec3 startRef(const math::Vec3& tan) noexcept {
  return std::fabs(tan.y) > 0.9 ? math::Vec3{1, 0, 0} : math::Vec3{0, 1, 0};
}

// De-duplicate consecutive coincident path points (a repeated spine vertex would
// give a zero-length segment / undefined tangent).
inline std::vector<math::Point3> cleanPath(const double* pathXYZ, int pathCount) {
  std::vector<math::Point3> pts;
  pts.reserve(static_cast<std::size_t>(std::max(pathCount, 0)));
  for (int i = 0; i < pathCount; ++i) {
    const math::Point3 p{pathXYZ[i * 3], pathXYZ[i * 3 + 1], pathXYZ[i * 3 + 2]};
    if (pts.empty() || math::distance(pts.back(), p) > kProfileTol) pts.push_back(p);
  }
  return pts;
}

// Resample an OPEN polyline at `count` points evenly spaced by cumulative arc length
// (endpoints preserved). Used to DENSIFY a spine / rail so each ruled band's incremental
// twist / tangent turn stays under the weld bound. A degenerate (zero-length) polyline
// or count < 2 returns the input unchanged. The geometry is preserved exactly on the
// original segments (linear interpolation between the bracketing vertices).
inline std::vector<math::Point3> resamplePolylineByArcLength(const std::vector<math::Point3>& p,
                                                             std::size_t count) {
  const std::size_t n = p.size();
  if (n < 2 || count < 2) return p;
  std::vector<double> cum(n, 0.0);
  for (std::size_t k = 1; k < n; ++k) cum[k] = cum[k - 1] + math::distance(p[k - 1], p[k]);
  const double total = cum.back();
  if (total < kProfileTol) return p;  // all coincident → leave to caller's guard
  std::vector<math::Point3> out;
  out.reserve(count);
  std::size_t seg = 0;
  for (std::size_t s = 0; s < count; ++s) {
    const double target = total * static_cast<double>(s) / static_cast<double>(count - 1);
    while (seg + 2 < n && cum[seg + 1] < target) ++seg;
    const double segLen = cum[seg + 1] - cum[seg];
    const double u = segLen > kProfileTol ? (target - cum[seg]) / segLen : 0.0;
    const double uc = std::clamp(u, 0.0, 1.0);
    out.push_back(math::Point3{p[seg].asVec() + (p[seg + 1].asVec() - p[seg].asVec()) * uc});
  }
  return out;
}

// True if the whole spine is a single straight line (every point collinear with the
// first segment direction, within tolerance). Only straight spines are meshed
// natively (see the header scope note); a bent spine defers to OCCT.
inline bool spineIsStraight(const std::vector<math::Point3>& spine) {
  if (spine.size() < 3) return true;  // 2 points are trivially straight
  const math::Vec3 d = spine.back() - spine.front();
  const double len = math::norm(d);
  if (len < kProfileTol) return false;  // start == end (degenerate)
  const math::Vec3 u = d / len;
  for (std::size_t k = 1; k + 1 < spine.size(); ++k) {
    const math::Vec3 r = spine[k] - spine.front();
    // Perpendicular distance from the point to the start→end line.
    const math::Vec3 perp = r - u * math::dot(r, u);
    if (math::norm(perp) > 1e-7 * std::max(len, 1.0)) return false;
  }
  return true;
}

// Per-station unit tangents of a polyline spine. Interior stations use the average
// of the incoming/outgoing segment directions; the ends use the single adjacent
// segment. Returns one tangent per spine point.
inline std::vector<math::Vec3> stationTangents(const std::vector<math::Point3>& spine) {
  const std::size_t n = spine.size();
  std::vector<math::Vec3> seg(n > 0 ? n - 1 : 0);
  for (std::size_t k = 0; k + 1 < n; ++k) {
    const math::Vec3 d = spine[k + 1] - spine[k];
    const double len = math::norm(d);
    seg[k] = len > kProfileTol ? d / len : math::Vec3{0, 0, 1};
  }
  std::vector<math::Vec3> tan(n);
  for (std::size_t k = 0; k < n; ++k) {
    math::Vec3 t;
    if (k == 0)
      t = seg.front();
    else if (k + 1 == n)
      t = seg.back();
    else
      t = seg[k - 1] + seg[k];  // average of adjacent segment dirs
    const double len = math::norm(t);
    tan[k] = len > kProfileTol ? t / len : seg[std::min(k, seg.size() - 1)];
  }
  return tan;
}

// True when the whole spine lies in a single plane (within tolerance). OCCT's
// corrected-Frenet sweep law collapses to a CONSTANT rotation for a planar spine
// (see the header note); only the planar case is native, so a non-planar curved
// spine is detected here and deferred to OCCT by build_sweep.
inline bool spineIsPlanar(const std::vector<math::Point3>& spine) {
  const std::size_t n = spine.size();
  if (n < 4) return true;  // ≤3 points are always coplanar
  const math::Vec3 e0 = spine[1] - spine.front();
  math::Vec3 nrm{0, 0, 0};
  double best = 0.0;
  for (std::size_t k = 2; k < n; ++k) {  // pick the largest-area plane normal
    const math::Vec3 c = math::cross(e0, spine[k] - spine.front());
    if (math::norm(c) > best) { best = math::norm(c); nrm = c; }
  }
  if (best < kProfileTol) return true;  // all collinear ⇒ trivially planar
  nrm = nrm / math::norm(nrm);
  double span = 0.0;
  for (const math::Point3& p : spine) span = std::max(span, math::norm(p - spine.front()));
  for (const math::Point3& p : spine)
    if (std::fabs(math::dot(p - spine.front(), nrm)) > 1e-7 * std::max(span, 1.0)) return false;
  return true;
}

// The CONSTANT sweep frame: OCCT's planar corrected-Frenet law keeps the section
// orientation fixed in world (see the header note), so every station shares the START
// frame's x/y axes and only the origin advances along the spine. The start axes are
// the oracle's: x = normalize(cross(t0, ref)); y = normalize(cross(x, t0)) — matching
// BRepOffsetAPI_MakePipe's start trihedron (nrm, up). Each station still stores its own
// unit tangent (used only for outward-normal orientation of the side faces / caps).
inline std::vector<SweepFrame> constantFrames(const std::vector<math::Point3>& spine,
                                              const std::vector<math::Vec3>& tan) {
  const std::size_t n = spine.size();
  math::Vec3 x = math::cross(tan[0], startRef(tan[0]));
  const double xn = math::norm(x);
  x = xn > kProfileTol ? x / xn : math::Vec3{1, 0, 0};
  math::Vec3 y = math::cross(x, tan[0]);
  const double yn = math::norm(y);
  y = yn > kProfileTol ? y / yn : math::Vec3{0, 1, 0};
  std::vector<SweepFrame> frames(n);
  for (std::size_t k = 0; k < n; ++k) frames[k] = SweepFrame{spine[k], x, y, tan[k]};
  return frames;
}

// A profile analysed for sweeping: de-duplicated closed loop of (px,py) offsets
// from the centroid, plus its signed area (for winding / degeneracy).
struct SweepProfile {
  std::vector<math::Point3> local;  ///< (px−cx, py−cy, 0) loop, closing dup dropped
  bool valid = false;
};

inline SweepProfile analyzeProfile(const double* profileXY, int profileCount) {
  SweepProfile pr;
  if (profileXY == nullptr || profileCount < 3) return pr;
  int n = profileCount;
  if (std::fabs(profileXY[0] - profileXY[(n - 1) * 2]) < kProfileTol &&
      std::fabs(profileXY[1] - profileXY[(n - 1) * 2 + 1]) < kProfileTol) {
    --n;
  }
  if (n < 3) return pr;

  double cx = 0.0, cy = 0.0, area2 = 0.0;
  for (int i = 0; i < n; ++i) {
    cx += profileXY[i * 2];
    cy += profileXY[i * 2 + 1];
    const int j = (i + 1) % n;
    area2 += profileXY[i * 2] * profileXY[j * 2 + 1] - profileXY[j * 2] * profileXY[i * 2 + 1];
  }
  if (std::fabs(area2) < kProfileTol) return pr;  // zero-area profile
  cx /= n;
  cy /= n;
  pr.local.reserve(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i)
    pr.local.push_back({profileXY[i * 2] - cx, profileXY[i * 2 + 1] - cy, 0.0});
  pr.valid = true;
  return pr;
}

// Place a local profile offset (px,py) into world at a station frame.
inline math::Point3 placeProfile(const SweepFrame& f, const math::Point3& local) noexcept {
  return f.origin + f.x * local.x + f.y * local.y;
}

// The profile circumradius: the max distance from the centroid (origin of the
// centred local loop) to any profile vertex. This is the half-thickness of the
// swept tube — the self-intersection guard compares it against the spine's local
// turning radius.
inline double profileCircumradius(const SweepProfile& pr) noexcept {
  double r = 0.0;
  for (const math::Point3& p : pr.local) r = std::max(r, math::norm(p.asVec()));
  return r;
}

// Guard the concave-side self-intersection / tight-curvature cases (deferred to
// OCCT). A polyline spine turns at each interior vertex; that turn is native-safe
// only when it is gentle enough that the swept tube does not fold through itself:
//   * the turn ANGLE between the incoming and outgoing segment directions must be
//     under `maxTurn` (a very sharp corner is a MakePipe mitre the ruled bands do
//     not model), and
//   * the local turning RADIUS (min adjacent segment length / turn angle, a discrete
//     curvature estimate) must exceed the profile circumradius with a safety margin,
//     so the inner (concave) wall of the tube does not cross the spine.
// Returns true (→ NULL → OCCT) when ANY interior vertex violates either bound.
inline bool spineTooSharp(const std::vector<math::Point3>& spine, double circumR) {
  constexpr double kMaxTurn = 0.6;         // ≈ 34° per vertex — gentle bends only
  constexpr double kRadiusSafety = 1.15;   // turning radius margin over the tube radius
  const std::size_t n = spine.size();
  for (std::size_t k = 1; k + 1 < n; ++k) {
    const math::Vec3 a = spine[k] - spine[k - 1];
    const math::Vec3 b = spine[k + 1] - spine[k];
    const double la = math::norm(a), lb = math::norm(b);
    if (la < kProfileTol || lb < kProfileTol) continue;
    const double turn = std::atan2(math::norm(math::cross(a, b)), math::dot(a, b));
    if (turn > kMaxTurn) return true;
    if (turn > kProfileTol) {
      const double radius = std::min(la, lb) / turn;  // arc r ≈ chord/Δθ
      if (radius < circumR * kRadiusSafety) return true;
    }
  }
  return false;
}

// ── Double-reflection Rotation-Minimizing Frame (RMF) ────────────────────────────
// Wang–Jüttler–Zheng–Yang (2008) "Computation of rotation minimizing frames": given a
// spine polyline with per-station unit tangents, transport an initial reference frame
// (r0 ⟂ t0, s0 = t0 × r0) along the spine so the frame does NOT rotate about the
// tangent (zero torsion) — the twist-free transport a non-planar sweep wants. The
// double-reflection step reflects the previous frame across the bisecting plane of the
// two consecutive tangents (two Householder reflections), which is second-order
// accurate and needs no per-step normalization drift correction.
//
// This is the general-3D twist-free transport the task calls for. It is NOT used by the
// twisted/guided oracle (which uses the simpler per-station Frenet frame, see
// frenetSectionFrames) — it is exposed so a future genuinely-twist-free non-planar plain
// sweep can transport the section without the corrected-Frenet mismatch. Returns one
// SweepFrame per station: origin = spine point, x = r (rotation-minimizing normal),
// y = s = t × r, t = tangent.
inline std::vector<SweepFrame> rmfFrames(const std::vector<math::Point3>& spine,
                                         const std::vector<math::Vec3>& tan) {
  const std::size_t n = spine.size();
  std::vector<SweepFrame> frames(n);
  // Seed r0 perpendicular to t0 (Gram-Schmidt off the oracle start reference).
  math::Vec3 r = math::cross(tan[0], startRef(tan[0]));
  double rn = math::norm(r);
  r = rn > kProfileTol ? r / rn : math::Vec3{1, 0, 0};
  r = r - tan[0] * math::dot(r, tan[0]);  // enforce r ⟂ t0
  rn = math::norm(r);
  r = rn > kProfileTol ? r / rn : math::Vec3{1, 0, 0};
  math::Vec3 s = math::cross(tan[0], r);
  frames[0] = SweepFrame{spine[0], r, s, tan[0]};

  for (std::size_t k = 0; k + 1 < n; ++k) {
    // Reflection 1: across the plane bisecting P[k]→P[k+1] (reflect r and t[k]).
    const math::Vec3 v1 = spine[k + 1] - spine[k];
    const double c1 = math::normSquared(v1);
    math::Vec3 rL = r, tL = tan[k];
    if (c1 > kProfileTol * kProfileTol) {
      rL = r - v1 * (2.0 / c1) * math::dot(v1, r);
      tL = tan[k] - v1 * (2.0 / c1) * math::dot(v1, tan[k]);
    }
    // Reflection 2: across the plane bisecting tL and t[k+1] → aligns tL onto t[k+1].
    const math::Vec3 v2 = tan[k + 1] - tL;
    const double c2 = math::normSquared(v2);
    math::Vec3 rNext = rL;
    if (c2 > kProfileTol * kProfileTol) rNext = rL - v2 * (2.0 / c2) * math::dot(v2, rL);
    // Re-orthonormalize against the exact next tangent (guards drift / degenerate steps).
    rNext = rNext - tan[k + 1] * math::dot(rNext, tan[k + 1]);
    const double nn = math::norm(rNext);
    r = nn > kProfileTol ? rNext / nn : rL;
    s = math::cross(tan[k + 1], r);
    frames[k + 1] = SweepFrame{spine[k + 1], r, s, tan[k + 1]};
  }
  return frames;
}

// The PER-STATION Frenet section frame the twisted/guided OCCT oracle uses (verified in
// src/engine/occt/occt_construct.cpp twisted_sweep / guided_sweep): a FIXED world up =
// (0,1,0), nrm = normalize(tangent × up) with a +X fallback when the tangent is ∥ up,
// and the section spans (nrm, up). Unlike the plain sweep's CONSTANT frame this frame
// re-derives per station from the LOCAL tangent, so the section rotates to track the
// spine — exactly matching the oracle's ThruSections sections. x = nrm, y = up, t =
// tangent. (`up` is the world Y axis at every station, NOT re-orthonormalized against
// the tangent — this reproduces the oracle's literal `poly.Add(c + u·nrm + v·up)`.)
inline std::vector<SweepFrame> frenetSectionFrames(const std::vector<math::Point3>& spine,
                                                   const std::vector<math::Vec3>& tan) {
  const math::Vec3 up{0, 1, 0};
  const std::size_t n = spine.size();
  std::vector<SweepFrame> frames(n);
  for (std::size_t k = 0; k < n; ++k) {
    math::Vec3 nrm = math::cross(tan[k], up);
    const double nn = math::norm(nrm);
    nrm = nn > 1e-6 ? nrm / nn : math::Vec3{1, 0, 0};  // tangent ∥ +Y → +X (oracle)
    frames[k] = SweepFrame{spine[k], nrm, up, tan[k]};
  }
  return frames;
}

// A single section RING at one station: the profile placed into world through the
// station frame after a uniform scale + in-plane rotation about the section normal.
// world(u,v) = origin + (u·cosθ − v·sinθ)·f.x + (u·sinθ + v·cosθ)·f.y, with (u,v) the
// centred profile offset — matching the oracle's `u' = u·ca − v·sa; v' = u·sa + v·ca`.
inline std::vector<math::Point3> sectionRing(const SweepFrame& f, const SweepProfile& pr,
                                             double scale, double twist) {
  const double ca = std::cos(twist), sa = std::sin(twist);
  std::vector<math::Point3> pts;
  pts.reserve(pr.local.size());
  for (const math::Point3& p : pr.local) {
    const double u = p.x * scale, v = p.y * scale;
    const double ur = u * ca - v * sa;
    const double vr = u * sa + v * ca;
    pts.push_back(f.origin + f.x * ur + f.y * vr);
  }
  return pts;
}

// Guard the twisted/guided section-ring sweep against self-intersection: adjacent
// station rings must not fold through each other. The conservative test is that the
// swept step between two consecutive section centres advances FURTHER than the section
// can shrink/rotate sideways — i.e. the along-spine advance exceeds the in-plane radius
// change so no ring inverts. Concretely each consecutive pair must (a) advance a
// positive spine step, and (b) keep a positive scale so the section never collapses; a
// station whose ring folds past its neighbour (the segment centre-to-centre distance is
// below the profile circumradius times the twist-induced lateral sweep) defers. Returns
// true (→ NULL → OCCT) on any violation.
inline bool sectionSweepUnsafe(const std::vector<math::Point3>& centres,
                               const std::vector<double>& scales, double circumR,
                               double totalTwist) {
  const std::size_t n = centres.size();
  if (n < 2) return true;
  const double twistPerSpan = std::fabs(totalTwist) / static_cast<double>(n - 1);
  for (std::size_t k = 0; k + 1 < n; ++k) {
    const double step = math::distance(centres[k], centres[k + 1]);
    if (step < kProfileTol) return true;               // stalled station (no advance)
    if (scales[k] <= kProfileTol || scales[k + 1] <= kProfileTol) return true;  // collapse
    // Lateral sweep of the outermost profile point across this span from the twist: the
    // point at radius (circumR·scale) sweeps an arc ≈ radius·twistPerSpan. If that arc
    // exceeds the axial advance the section rim overtakes the spine step → self-fold.
    const double rimRadius = circumR * std::max(scales[k], scales[k + 1]);
    if (rimRadius * twistPerSpan > step) return true;  // rim folds past the neighbour
  }
  return false;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_sweep — cc_solid_sweep entry point. Sweep the closed profile along the
// path. NATIVE for a STRAIGHT spine (→ an exact directional prism) AND for a SMOOTH
// CURVED but PLANAR spine (→ a constant-frame ruled-band tube matching OCCT MakePipe's
// planar corrected-Frenet law, deflection-bounded vs OCCT). Returns NULL (→ OCCT
// MakePipe fallthrough) for a NON-PLANAR curved spine, a TIGHT-CURVATURE / self-
// intersecting spine, a degenerate profile, < 2 path points, or a zero start tangent
// (see the header scope note).
//
// For the straight case the constant frame is the only frame, so every side face is a
// planar quad and the two ends are planar caps → a watertight prism of volume
// (profile area × path length). For a smooth curved PLANAR spine the frame is held
// CONSTANT (the same law OCCT uses on a planar spine — the section is translated, not
// rotated): each profile edge spans one bilinear RULED band per spine segment (loft's
// ruledSideFace helper) sharing per-station vertex rings, capped at the ends → a
// watertight tube welded by the two-stage tessellator, matching the MakePipe oracle.
//
// Cognitive complexity: linear assembler (14, measured — Acceptable band).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_sweep(const double* profileXY, int profileCount, const double* pathXYZ,
                               int pathCount) {
  if (pathXYZ == nullptr || pathCount < 2) return {};
  const detail::SweepProfile pr = detail::analyzeProfile(profileXY, profileCount);
  if (!pr.valid) return {};

  std::vector<math::Point3> spine = detail::cleanPath(pathXYZ, pathCount);
  if (spine.size() < 2) return {};  // all points coincident
  if (detail::spineIsStraight(spine)) {
    // A straight spine collapses to its two endpoints: every side face is then one
    // planar quad (6 faces total for a quad profile), matching OCCT's prism face
    // count exactly rather than emitting a redundant band per collinear point.
    if (spine.size() > 2) spine = {spine.front(), spine.back()};
  } else if (!detail::spineIsPlanar(spine)) {
    return {};  // non-planar curve → OCCT corrected-Frenet (see header)
  } else if (detail::spineTooSharp(spine, detail::profileCircumradius(pr))) {
    return {};  // tight / self-intersecting curve → OCCT MakePipe (see header)
  }

  const std::vector<math::Vec3> tan = detail::stationTangents(spine);
  if (math::norm(tan.front()) < kProfileTol) return {};  // degenerate start tangent

  const std::vector<detail::SweepFrame> frames = detail::constantFrames(spine, tan);
  const std::size_t nStations = frames.size();
  const std::size_t nProf = pr.local.size();

  // Shared vertex ring per station so the side faces and caps weld watertight.
  std::vector<std::vector<topo::Shape>> ring(nStations, std::vector<topo::Shape>(nProf));
  std::vector<std::vector<math::Point3>> pos(nStations, std::vector<math::Point3>(nProf));
  for (std::size_t s = 0; s < nStations; ++s)
    for (std::size_t i = 0; i < nProf; ++i) {
      pos[s][i] = detail::placeProfile(frames[s], pr.local[i]);
      ring[s][i] = topo::ShapeBuilder::makeVertex(pos[s][i]);
    }

  std::vector<topo::Shape> faces;
  faces.reserve((nStations - 1) * nProf + 2);

  // Planar side faces, one per (profile edge × spine segment). On a straight spine
  // the two stations' frames are identical, so each patch is a planar quad; the
  // ruled-face helper builds it as a (planar) bilinear whose boundary edges the
  // mesher welds to its neighbours and to the caps.
  for (std::size_t s = 0; s + 1 < nStations; ++s)
    for (std::size_t i = 0; i < nProf; ++i) {
      const std::size_t j = (i + 1) % nProf;
      const math::Point3 Ai = pos[s][i], Aj = pos[s][j], Bi = pos[s + 1][i], Bj = pos[s + 1][j];
      const math::Point3 mid{(Ai.asVec() + Aj.asVec() + Bi.asVec() + Bj.asVec()) / 4.0};
      const math::Vec3 du = 0.5 * ((Aj - Ai) + (Bj - Bi));
      const math::Vec3 dv = 0.5 * ((Bi - Ai) + (Bj - Aj));
      const math::Vec3 nat = math::cross(du, dv);
      const math::Vec3 tdir = frames[s].t;
      const math::Vec3 rel = mid - frames[s].origin;
      const math::Vec3 radial = rel - tdir * math::dot(rel, tdir);
      const topo::Orientation o = math::dot(nat, radial) < 0.0 ? topo::Orientation::Reversed
                                                               : topo::Orientation::Forward;
      faces.push_back(detail::ruledSideFace(ring[s][i], ring[s][j], ring[s + 1][i], ring[s + 1][j],
                                            o));
    }

  // Planar end caps. In the CONSTANT frame BOTH caps lie in the fixed x-y plane, so
  // both share the section normal n = x×y; only the OUTWARD sign differs. Orient each
  // away from the body (its overall spine chord), so the start cap points back and the
  // end cap points forward regardless of the local tangent.
  math::Vec3 sectionN = math::cross(frames.front().x, frames.front().y);
  const double snLen = math::norm(sectionN);
  sectionN = snLen > kProfileTol ? sectionN / snLen : math::Vec3{0, 0, 1};
  const math::Vec3 bodyDir = spine.back() - spine.front();
  const math::Vec3 startOut =
      math::dot(sectionN, bodyDir) > 0.0 ? -sectionN : sectionN;  // point away from body
  const math::Vec3 endOut = -startOut;
  faces.push_back(detail::planarFace(ring.front(), math::Dir3{startOut}, topo::Orientation::Forward));
  faces.push_back(detail::planarFace(ring.back(), math::Dir3{endOut}, topo::Orientation::Forward));

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

namespace detail {

// Assemble a watertight ruled tube from per-station section RINGS (one closed loop of
// nProf world points per station, all rings equal size). Between consecutive stations
// each profile edge spans one bilinear RULED band (loft.h ruledSideFace) with shared
// per-station vertex rings; the first and last rings are closed with planar caps. Face
// orientation makes every effective normal point OUTWARD (side bands away from the
// section-centre axis, caps away from the tube body). Returns NULL if the rings are
// unusable. This is the ThruSections(ruled) construction the twisted/guided oracle uses.
//
// `sectionNormals[s]` is the (unit) plane normal of ring s (its station frame x×y), used
// to wind the end caps; `centres[s]` is ring s's centre (for side-band outward tests).
inline topo::Shape assembleRingTube(const std::vector<std::vector<math::Point3>>& rings,
                                    const std::vector<math::Point3>& centres,
                                    const std::vector<math::Vec3>& sectionNormals) {
  const std::size_t nStations = rings.size();
  if (nStations < 2) return {};
  const std::size_t nProf = rings.front().size();
  if (nProf < 3) return {};

  std::vector<std::vector<topo::Shape>> ring(nStations, std::vector<topo::Shape>(nProf));
  for (std::size_t s = 0; s < nStations; ++s)
    for (std::size_t i = 0; i < nProf; ++i) ring[s][i] = topo::ShapeBuilder::makeVertex(rings[s][i]);

  std::vector<topo::Shape> faces;
  faces.reserve((nStations - 1) * nProf + 2);

  for (std::size_t s = 0; s + 1 < nStations; ++s)
    for (std::size_t i = 0; i < nProf; ++i) {
      const std::size_t j = (i + 1) % nProf;
      const math::Point3 Ai = rings[s][i], Aj = rings[s][j], Bi = rings[s + 1][i],
                         Bj = rings[s + 1][j];
      const math::Point3 mid{(Ai.asVec() + Aj.asVec() + Bi.asVec() + Bj.asVec()) / 4.0};
      const math::Vec3 du = 0.5 * ((Aj - Ai) + (Bj - Bi));
      const math::Vec3 dv = 0.5 * ((Bi - Ai) + (Bj - Aj));
      const math::Vec3 nat = math::cross(du, dv);
      // Outward reference: the band centre pushed away from the local spine axis. Use
      // the mean of the two stations' section centres and the local spine direction.
      const math::Vec3 axis = centres[s + 1] - centres[s];
      const math::Vec3 axisDir =
          math::norm(axis) > kProfileTol ? axis / math::norm(axis) : math::Vec3{0, 0, 1};
      const math::Point3 baseC{(centres[s].asVec() + centres[s + 1].asVec()) / 2.0};
      const math::Vec3 rel = mid - baseC;
      const math::Vec3 radial = rel - axisDir * math::dot(rel, axisDir);
      const topo::Orientation o = math::dot(nat, radial) < 0.0 ? topo::Orientation::Reversed
                                                               : topo::Orientation::Forward;
      faces.push_back(detail::ruledSideFace(ring[s][i], ring[s][j], ring[s + 1][i], ring[s + 1][j],
                                            o));
    }

  // End caps: wound so their outward normal points AWAY from the tube body (the start
  // cap along −bodyDir, the end cap along +bodyDir).
  const math::Vec3 bodyDir = centres.back() - centres.front();
  auto capNormal = [&](std::size_t s, const math::Vec3& outward) -> math::Dir3 {
    math::Vec3 nn = sectionNormals[s];
    if (math::norm(nn) < kProfileTol) nn = bodyDir;
    return math::dot(nn, outward) < 0.0 ? math::Dir3{-nn} : math::Dir3{nn};
  };
  faces.push_back(
      detail::planarFace(ring.front(), capNormal(0, -bodyDir), topo::Orientation::Forward));
  faces.push_back(detail::planarFace(ring.back(), capNormal(nStations - 1, bodyDir),
                                     topo::Orientation::Forward));

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// Shared per-station ThruSections builder for cc_twisted_sweep / cc_guided_sweep. At
// every path station the profile is placed through the per-station Frenet section frame
// (frenetSectionFrames — the twisted/guided oracle law) after the station's uniform
// scale + in-plane twist, then the rings are ruled+capped into a watertight tube. The
// per-station scale is supplied by `scaleAt(f)` (twist: linear 1→scaleEnd; guided:
// dist(path,guide)/d0). `totalTwist` accumulates linearly (twist·f). Returns NULL on a
// degenerate profile / path, a collapsing scale, or a self-intersecting tube (guarded).
inline topo::Shape build_section_thrusections(const double* profileXY, int profileCount,
                                              const double* pathXYZ, int pathCount,
                                              double totalTwist,
                                              const std::function<double(double)>& scaleAt) {
  if (pathXYZ == nullptr || pathCount < 2) return {};
  const SweepProfile pr = analyzeProfile(profileXY, profileCount);
  if (!pr.valid) return {};

  std::vector<math::Point3> spine = cleanPath(pathXYZ, pathCount);
  if (spine.size() < 2) return {};
  // A REAL twist needs a FINELY-SAMPLED loft to converge to (and match OCCT's) smoothly-
  // twisted ThruSections: a single ruled segment across a large twist under-fills the true
  // swept solid (the corner chords cut inside the rotating section). The CALLER supplies a
  // spine pre-densified so each band's accumulated twist stays under a small per-band bound
  // (build_twisted_sweep densifies a straight spine to `kMaxBandTwist`; guided_sweep passes
  // its own path with totalTwist == 0). At a bounded per-band twist the twisted (saddle)
  // ruled tube welds robustly watertight — the SAME gentle-rotation-per-band regime the
  // landed guided_orient_sweep uses — and its volume converges to the analytic (area-
  // preserving) value, verified against the OCCT ThruSections oracle fed the same stations.
  // A tube that still fails the engine self-verify (a near-self-fold slipping the guard) is
  // DISCARDED by NativeEngine → OCCT (honest coexistence, never a leaky solid).
  const std::vector<math::Vec3> tan = stationTangents(spine);
  const std::vector<SweepFrame> frames = frenetSectionFrames(spine, tan);
  const std::size_t nStations = frames.size();
  const double denom = static_cast<double>(nStations - 1);

  std::vector<std::vector<math::Point3>> rings(nStations);
  std::vector<math::Point3> centres(nStations);
  std::vector<math::Vec3> normals(nStations);
  std::vector<double> scales(nStations);
  for (std::size_t s = 0; s < nStations; ++s) {
    const double f = static_cast<double>(s) / denom;
    const double sc = scaleAt(f);
    if (!(sc > 0.0)) return {};  // collapsing / non-positive scale → defer
    scales[s] = sc;
    rings[s] = sectionRing(frames[s], pr, sc, totalTwist * f);
    centres[s] = frames[s].origin;
    normals[s] = math::cross(frames[s].x, frames[s].y);
  }

  const double circumR = profileCircumradius(pr);
  if (sectionSweepUnsafe(centres, scales, circumR, totalTwist)) return {};  // self-fold → OCCT

  return assembleRingTube(rings, centres, normals);
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_twisted_sweep — cc_twisted_sweep entry point. The section ROTATES about the
// path tangent by `twistRadians` (accumulated linearly 0→twistRadians) and SCALES
// linearly 1→`scaleEnd`. NATIVE:
//   * plain (twist≈0, scale≈1) → forwards to build_sweep (constant-frame MakePipe law).
//   * a REAL twist/scale → DENSIFY the spine (arc-length) so each ruled band's twist stays
//     under kMaxBandTwist, then build the per-station Frenet-framed ruled ThruSections tube
//     (build_section_thrusections). At a bounded per-band rotation the twisted tube welds
//     watertight and its volume converges to the area-preserving analytic value, matching
//     the OCCT twisted_sweep (ThruSections) oracle fed the same stations.
// NULL (→ OCCT twisted_sweep) on a degenerate profile/path or a self-folding tube.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_twisted_sweep(const double* profileXY, int profileCount,
                                       const double* pathXYZ, int pathCount, double twistRadians,
                                       double scaleEnd) {
  if (std::fabs(twistRadians) <= 1e-9 && std::fabs(scaleEnd - 1.0) <= 1e-9)
    return build_sweep(profileXY, profileCount, pathXYZ, pathCount);

  // Densify the spine so each ruled band's accumulated twist stays under kMaxBandTwist:
  // a coarse (e.g. 2-point) path across a large twist under-fills the true swept solid,
  // so we resample the CLEANED spine polyline at nStations points by arc length and feed
  // the densified path to build_section_thrusections. The twist/scale fraction stays
  // keyed to the arc-length fraction f (matching the OCCT oracle's per-station f), so the
  // densified native and a densified OCCT ThruSections converge to the same solid.
  std::vector<math::Point3> spine = detail::cleanPath(pathXYZ, pathCount);
  if (spine.size() < 2) return {};
  std::size_t nBands = 1;
  if (std::fabs(twistRadians) > 1e-9)
    nBands = static_cast<std::size_t>(std::ceil(std::fabs(twistRadians) / detail::kMaxBandTwist));
  nBands = std::max<std::size_t>(nBands, spine.size() - 1);          // honour caller stations
  nBands = std::min<std::size_t>(nBands, detail::kMaxDensifyStations - 1);
  const std::vector<math::Point3> dense = detail::resamplePolylineByArcLength(spine, nBands + 1);
  if (dense.size() < 2) return {};

  std::vector<double> flat;
  flat.reserve(dense.size() * 3);
  for (const math::Point3& p : dense) { flat.push_back(p.x); flat.push_back(p.y); flat.push_back(p.z); }
  const auto scaleAt = [scaleEnd](double f) { return 1.0 + (scaleEnd - 1.0) * f; };
  return detail::build_section_thrusections(profileXY, profileCount, flat.data(),
                                            static_cast<int>(dense.size()), twistRadians, scaleAt);
}

// ─────────────────────────────────────────────────────────────────────────────
// build_guided_sweep — cc_guided_sweep entry point. Sweep the profile along the path
// with each station's section uniformly SCALED by how far the guide polyline has
// splayed from the path there: sc(f) = dist(path(f), guide(f)) / dist(path(0),
// guide(0)) — matching the OCCT guided_sweep oracle (ThruSections through the
// guide-scaled per-station Frenet sections). The guide is sampled at parameter fraction
// f treating its vertices as evenly spaced (exact for a 2-point guide, as the oracle
// does). No twist. NATIVE when the tube is watertight and non-self-intersecting; NULL
// (→ OCCT) on degenerate input, a coincident guide start, or a self-folding tube.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_guided_sweep(const double* profileXY, int profileCount,
                                      const double* pathXYZ, int pathCount, const double* guideXYZ,
                                      int guideCount) {
  if (profileXY == nullptr || pathXYZ == nullptr || guideXYZ == nullptr) return {};
  if (profileCount < 3 || pathCount < 2 || guideCount < 2) return {};

  // Path/guide sampled at fraction f (path from its cleaned stations, guide linearly
  // over its raw vertices — matching the oracle's guideAt). Both use the RAW path so the
  // station fractions line up with the oracle before cleanPath collapses duplicates.
  auto lerpPoly = [](const double* xyz, int count, double f) -> math::Point3 {
    const double s = f * (count - 1);
    int i0 = std::min(static_cast<int>(std::floor(s)), count - 2);
    if (i0 < 0) i0 = 0;
    const double t = s - i0;
    const math::Point3 a{xyz[i0 * 3], xyz[i0 * 3 + 1], xyz[i0 * 3 + 2]};
    const math::Point3 b{xyz[(i0 + 1) * 3], xyz[(i0 + 1) * 3 + 1], xyz[(i0 + 1) * 3 + 2]};
    return math::Point3{a.asVec() + (b.asVec() - a.asVec()) * t};
  };
  const math::Point3 p0{pathXYZ[0], pathXYZ[1], pathXYZ[2]};
  const double d0 = math::distance(p0, lerpPoly(guideXYZ, guideCount, 0.0));
  if (d0 < 1e-6) return {};  // guide coincident with the path start → OCCT

  const auto scaleAt = [&](double f) {
    return math::distance(lerpPoly(pathXYZ, pathCount, f), lerpPoly(guideXYZ, guideCount, f)) / d0;
  };
  return detail::build_section_thrusections(profileXY, profileCount, pathXYZ, pathCount,
                                            /*totalTwist*/ 0.0, scaleAt);
}

namespace detail {

// The point where the plane through P perpendicular to `T` meets the guide polyline —
// OCCT's GeomFill_GuideTrihedronPlan correspondence (Pprime is the guide point in the
// plane normal to the tangent at the spine point P). The guide is parametrized by its
// projection along T (monotonic for a guide that runs alongside the spine): find the
// segment whose endpoints bracket `P·T` and lerp. Returns false if no segment brackets
// the station (the guide does not span this station's perpendicular plane). By
// construction Pprime−P is exactly ⟂ T (both lie in {X·T = P·T}).
inline bool guidePointInPerpPlane(const std::vector<math::Point3>& guide, const math::Vec3& T,
                                  const math::Point3& P, math::Point3& out) {
  const double target = math::dot(P.asVec(), T);
  for (std::size_t i = 0; i + 1 < guide.size(); ++i) {
    const double a = math::dot(guide[i].asVec(), T);
    const double b = math::dot(guide[i + 1].asVec(), T);
    const double lo = std::min(a, b), hi = std::max(a, b);
    if (target < lo - kProfileTol || target > hi + kProfileTol) continue;
    const double denom = b - a;
    const double u = std::fabs(denom) > kProfileTol ? (target - a) / denom : 0.0;
    const double uc = std::clamp(u, 0.0, 1.0);
    out = math::Point3{guide[i].asVec() + (guide[i + 1].asVec() - guide[i].asVec()) * uc};
    return true;
  }
  return false;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_guided_orient_sweep — cc_guided_orient_sweep entry point. The section's
// ORIENTATION (not its scale) is fixed by a guide wire, reproducing OCCT
// BRepOffsetAPI_MakePipeShell + SetMode(guideWire) with the DEFAULT KeepContact =
// BRepFill_NoContact — i.e. GeomFill_GuideTrihedronPlan with rotation==false, whose
// per-station frame is the RIGID trihedron M = [N, B, T], V = P (verified against OCCT
// source GeomFill_GuideTrihedronPlan.cxx::D0 and GeomFill_LocationGuide.cxx::D0):
//   T = spine unit tangent; N = normalize(Pprime − P) with Pprime the guide point in
//   the plane through P perpendicular to T; B = T × N. No scale, no rotation root-find.
//
// NATIVE SCOPE — STRAIGHT SPINE ONLY. On a straight spine T is constant, so every
// perpendicular plane is parallel and the guide∩plane point (guidePointInPerpPlane) is a
// pure GEOMETRIC intersection, independent of OCCT's internal guide reparametrization
// (BRepFill_CompatibleWires::SetPercent) — native and OCCT therefore land on the SAME
// Pprime and agree SPATIALLY (bbox/Hausdorff), not merely on volume. The straight spine
// is densified so a rotating guide's per-band section rotation stays small enough for the
// native ruled-band mesher to weld watertight; a tube whose rotation is too coarse to
// weld is caught by the engine's robustlyWatertight self-verify → OCCT fallback.
//
// Returns NULL (→ OCCT guided_orient_sweep) for: a CURVED spine (per-station T varies and
// the CompatibleWires guide resample shifts the aim — not spatially reproducible without
// the guide surface itself), a degenerate profile/path, a guide that does not span a
// station's perpendicular plane, or a guide passing through the spine (degenerate N).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_guided_orient_sweep(const double* profileXY, int profileCount,
                                             const double* pathXYZ, int pathCount,
                                             const double* guideXYZ, int guideCount) {
  if (profileXY == nullptr || pathXYZ == nullptr || guideXYZ == nullptr) return {};
  if (profileCount < 3 || pathCount < 2 || guideCount < 2) return {};
  const detail::SweepProfile pr = detail::analyzeProfile(profileXY, profileCount);
  if (!pr.valid) return {};

  const std::vector<math::Point3> raw = detail::cleanPath(pathXYZ, pathCount);
  if (raw.size() < 2) return {};
  if (!detail::spineIsStraight(raw)) return {};  // curved spine → OCCT (see scope note)
  const math::Point3 A = raw.front(), Bend = raw.back();
  math::Vec3 T = Bend - A;
  const double L = math::norm(T);
  if (L < kProfileTol) return {};
  T = T / L;

  std::vector<math::Point3> guide;
  guide.reserve(static_cast<std::size_t>(guideCount));
  for (int i = 0; i < guideCount; ++i)
    guide.push_back({guideXYZ[i * 3], guideXYZ[i * 3 + 1], guideXYZ[i * 3 + 2]});

  // The guide direction N(f) at spine fraction f: the unit vector from the spine point to
  // the guide point in the perpendicular plane there. Returns false on a guide gap or a
  // guide passing through the spine (degenerate N) → whole build defers to OCCT.
  const auto normalAt = [&](double f, math::Vec3& N) -> bool {
    const math::Point3 P{A.asVec() + (Bend.asVec() - A.asVec()) * f};
    math::Point3 Pprime;
    if (!detail::guidePointInPerpPlane(guide, T, P, Pprime)) return false;
    N = Pprime - P;
    const double dn = math::norm(N);
    if (dn < 1e-6) return false;
    N = N / dn;
    return true;
  };

  // Pick the station density from the guide-induced section rotation: a near-constant N
  // (offset guide) collapses to a 2-station prism — matching OCCT's minimal 6-face tiling
  // EXACTLY — while a rotating guide is densified so each band's rotation stays small
  // enough for the native ruled-band mesher to weld watertight. Measure the total turn of
  // N over a coarse pre-sample.
  constexpr int kProbe = 64;
  double totalTurn = 0.0;
  math::Vec3 Nprev;
  if (!normalAt(0.0, Nprev)) return {};
  for (int k = 1; k <= kProbe; ++k) {
    math::Vec3 Nk;
    if (!normalAt(static_cast<double>(k) / kProbe, Nk)) return {};
    const double d = std::clamp(math::dot(Nprev, Nk), -1.0, 1.0);
    totalTurn += std::acos(d);
    Nprev = Nk;
  }
  constexpr double kMaxPerBand = 0.05;  // ≈ 2.9° per band — welds watertight (measured)
  std::size_t nStations = 2;
  if (totalTurn > 1e-4)
    nStations = static_cast<std::size_t>(std::ceil(totalTurn / kMaxPerBand)) + 1;
  nStations = std::min<std::size_t>(std::max<std::size_t>(nStations, 2), 512);
  const double denom = static_cast<double>(nStations - 1);

  std::vector<std::vector<math::Point3>> rings(nStations);
  std::vector<math::Point3> centres(nStations);
  std::vector<math::Vec3> normals(nStations);
  for (std::size_t s = 0; s < nStations; ++s) {
    const double f = static_cast<double>(s) / denom;
    const math::Point3 P{A.asVec() + (Bend.asVec() - A.asVec()) * f};
    math::Vec3 N;
    if (!normalAt(f, N)) return {};
    const math::Vec3 Bx = math::cross(T, N);  // ⟂ both, unit (N⟂T)
    const detail::SweepFrame frame{P, N, Bx, T};
    rings[s] = detail::sectionRing(frame, pr, /*scale*/ 1.0, /*twist*/ 0.0);
    centres[s] = P;
    normals[s] = math::cross(frame.x, frame.y);  // = T
  }

  return detail::assembleRingTube(rings, centres, normals);
}

namespace detail {

// A centred profile loop analysed for the rail loft: the (x,y) offsets from its centroid.
inline std::vector<math::Point3> centredProfile(const double* xy, int count) {
  double cx = 0.0, cy = 0.0;
  for (int i = 0; i < count; ++i) { cx += xy[i * 2]; cy += xy[i * 2 + 1]; }
  cx /= count;
  cy /= count;
  std::vector<math::Point3> out;
  out.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) out.push_back({xy[i * 2] - cx, xy[i * 2 + 1] - cy, 0.0});
  return out;
}

// build_curved_rail_loft — the CURVED-rail arm of build_loft_along_rail. Densify the
// cleaned rail (arc length) so each band's tangent turn stays under kMaxBandTurn, then
// RMF-transport (rotation-minimizing frame, zero spurious twist) the section MORPH A→B
// (linear per-vertex interpolation in the transported frame) into per-station rings, and
// tile them into a watertight tube (assembleRingTube). For a circular-arc rail of radius
// R through angle φ with a constant section of area A the volume converges (as the rail
// densifies) to the Pappus torus-sector value A·R·φ. A rail too tight to weld at the cap
// is caught by the engine self-verify → OCCT. Returns NULL on a degenerate rail / cap.
inline topo::Shape build_curved_rail_loft(const std::vector<math::Point3>& rail,
                                          const std::vector<math::Point3>& localA,
                                          const std::vector<math::Point3>& localB) {
  if (rail.size() < 2 || localA.size() != localB.size() || localA.size() < 3) return {};

  // Probe the total tangent turn on a coarse resample → pick the station count so each
  // band turns at most kMaxBandTurn (the ruled-band weld bound, as guided_orient_sweep).
  const std::vector<math::Vec3> probeTan = stationTangents(rail);
  double totalTurn = 0.0;
  for (std::size_t k = 1; k < probeTan.size(); ++k) {
    const double d = std::clamp(math::dot(probeTan[k - 1], probeTan[k]), -1.0, 1.0);
    totalTurn += std::acos(d);
  }
  std::size_t nStations = 2;
  if (totalTurn > 1e-4)
    nStations = static_cast<std::size_t>(std::ceil(totalTurn / kMaxBandTurn)) + 1;
  nStations = std::min<std::size_t>(std::max<std::size_t>(nStations, rail.size()),
                                    kMaxDensifyStations);

  const std::vector<math::Point3> dense = resamplePolylineByArcLength(rail, nStations);
  if (dense.size() < 2) return {};
  const std::vector<math::Vec3> tan = stationTangents(dense);
  const std::vector<SweepFrame> frames = rmfFrames(dense, tan);
  const std::size_t n = frames.size();
  const double denom = static_cast<double>(n - 1);
  const std::size_t nProf = localA.size();

  std::vector<std::vector<math::Point3>> rings(n);
  std::vector<math::Point3> centres(n);
  std::vector<math::Vec3> normals(n);
  for (std::size_t s = 0; s < n; ++s) {
    const double f = static_cast<double>(s) / denom;
    std::vector<math::Point3> ring(nProf);
    for (std::size_t i = 0; i < nProf; ++i) {
      const double u = localA[i].x + (localB[i].x - localA[i].x) * f;
      const double v = localA[i].y + (localB[i].y - localA[i].y) * f;
      ring[i] = frames[s].origin + frames[s].x * u + frames[s].y * v;
    }
    rings[s] = std::move(ring);
    centres[s] = frames[s].origin;
    normals[s] = math::cross(frames[s].x, frames[s].y);
  }
  return assembleRingTube(rings, centres, normals);
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_loft_along_rail — cc_loft_along_rail entry point. Morph section A (at the rail
// start) into section B (at the rail end) along the rail. The OCCT oracle is
// BRepOffsetAPI_MakePipeShell (RoundCorner transitions). For a STRAIGHT rail the
// pipe-shell reduces EXACTLY to a ruled loft between the two sections, each placed in
// the plane PERPENDICULAR to the (single) rail tangent — so a straight rail is NATIVE
// (reuse build_ruled_loft). Requires equal section vertex counts ≥3 (build_ruled_loft's
// contract). A CURVED rail is served by build_curved_rail_loft — an RMF-transported
// morph densified to a bounded per-band turn; a rail too tight to weld is discarded by
// the engine self-verify → OCCT MakePipeShell.
//
// The perpendicular frame matches the oracle's perpendicularFrame: uDir = tan × ref,
// vDir = tan × uDir, ref = +Z unless |tan·Z| > 0.95 then +X.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_loft_along_rail(const double* railXYZ, int railCount,
                                         const double* profileA_XY, int aCount,
                                         const double* profileB_XY, int bCount) {
  if (railXYZ == nullptr || profileA_XY == nullptr || profileB_XY == nullptr) return {};
  if (railCount < 2 || aCount < 3 || bCount < 3) return {};
  if (aCount != bCount) return {};  // ruled loft pairs vertex k→k → equal counts only

  std::vector<math::Point3> rail = detail::cleanPath(railXYZ, railCount);
  if (rail.size() < 2) return {};
  if (!detail::spineIsStraight(rail))  // curved rail → RMF-transported densified morph
    return detail::build_curved_rail_loft(rail, detail::centredProfile(profileA_XY, aCount),
                                          detail::centredProfile(profileB_XY, bCount));

  const math::Point3 start = rail.front(), end = rail.back();
  const math::Vec3 tanV = end - start;
  const double tl = math::norm(tanV);
  if (tl < kProfileTol) return {};
  const math::Vec3 tanU = tanV / tl;

  // perpendicularFrame: ref = +Z unless the tangent is ~∥ Z, then +X.
  const math::Vec3 ref =
      std::fabs(math::dot(tanU, math::Vec3{0, 0, 1})) > 0.95 ? math::Vec3{1, 0, 0}
                                                             : math::Vec3{0, 0, 1};
  math::Vec3 uDir = math::cross(tanU, ref);
  const double un = math::norm(uDir);
  if (un < kProfileTol) return {};
  uDir = uDir / un;
  math::Vec3 vDir = math::cross(tanU, uDir);
  const double vn = math::norm(vDir);
  if (vn < kProfileTol) return {};
  vDir = vDir / vn;

  // Place each section (centred on its centroid) in the perpendicular frame at the
  // matching rail end — exactly the oracle's buildRailSectionWire.
  auto placeSection = [&](const double* xy, int count, const math::Point3& origin) {
    double cx = 0.0, cy = 0.0;
    for (int i = 0; i < count; ++i) { cx += xy[i * 2]; cy += xy[i * 2 + 1]; }
    cx /= count;
    cy /= count;
    std::vector<math::Point3> out;
    out.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i)
      out.push_back(origin + uDir * (xy[i * 2] - cx) + vDir * (xy[i * 2 + 1] - cy));
    return out;
  };
  const std::vector<math::Point3> secA = placeSection(profileA_XY, aCount, start);
  const std::vector<math::Point3> secB = placeSection(profileB_XY, bCount, end);
  return build_ruled_loft(secA, secB);
}

}  // namespace cybercad::native::construct

#endif  // CYBERCAD_NATIVE_CONSTRUCT_SWEEP_H
