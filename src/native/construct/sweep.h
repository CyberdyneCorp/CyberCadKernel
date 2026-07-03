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
//     * TWISTED sweep (cc_twisted_sweep): NATIVE only when it reduces to the plain
//       sweep (twist ≈ 0, scale ≈ 1) — then it forwards to build_sweep (straight or
//       smooth curved). A genuine twist/scale accumulates an extra section rotation
//       the plain constant-frame sweep does not model, so it is deferred to OCCT
//       ThruSections.
//     * cc_guided_sweep / cc_loft_along_rail — pipe-shell / guide cases, left OCCT
//       fallthrough (labelled) in the engine glue.
//
// REFERENCE ORACLE ONLY: BRepOffsetAPI_MakePipe / MakePipeShell were consulted to
// confirm the start framing, the polyline spine and the capped-solid result; nothing
// is copied. The bilinear parametrization matches src/native/math bezierSurfacePoint.
//
// Cognitive complexity: build_sweep is a linear assembler (14, measured — Acceptable
// band); the constant-frame / planarity helpers are short and the sharpness guard is 10.
// OCCT-FREE. Header-only. clang++ c++20.
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
#include <vector>

namespace cybercad::native::construct {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

namespace detail {

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

// ─────────────────────────────────────────────────────────────────────────────
// build_twisted_sweep — cc_twisted_sweep entry point. A genuine twist / scale
// accumulates an extra per-section rotation/scale the plain constant-frame sweep does
// not model, so this is NATIVE only when it reduces to the plain sweep (twist ≈ 0, scale
// ≈ 1) — then it forwards to build_sweep (which is itself native for a straight OR a
// smooth curved PLANAR spine). Any real twist/scale → NULL so the engine falls through
// to OCCT's twisted_sweep.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_twisted_sweep(const double* profileXY, int profileCount,
                                       const double* pathXYZ, int pathCount, double twistRadians,
                                       double scaleEnd) {
  if (std::fabs(twistRadians) > 1e-9 || std::fabs(scaleEnd - 1.0) > 1e-9) return {};
  return build_sweep(profileXY, profileCount, pathXYZ, pathCount);
}

}  // namespace cybercad::native::construct

#endif  // CYBERCAD_NATIVE_CONSTRUCT_SWEEP_H
