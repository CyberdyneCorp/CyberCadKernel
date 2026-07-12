// SPDX-License-Identifier: Apache-2.0
//
// nurbs_curved_split.h — NURBS roadmap LAYER 3, SLICE 2 (L3-S2): the SECOND
// exact-NURBS B-rep boolean — a genuine NURBS face SPLIT BY A CURVED (analytic)
// FACE, welded watertight. It extends L3-S1 (`nurbs_plane_split.h`, a NURBS face
// cut by a PLANE) from a planar cutter to an ANALYTIC CURVED cutter (Cylinder /
// Sphere / Cone).
//
// ── ROLE (the flagship deep-tail track: face∩CURVED-face) ───────────────────────
// L3-S1 cut the NURBS wall by a PLANE: the seam is NURBS∩plane, the keep test is a
// HALF-SPACE side, and the sew is curved-NURBS↔FLAT cap. L3-S2 is the SAME verb with
// the cutter left as a genuine analytic CURVED surface G (a Cylinder/Sphere/Cone
// solid), so THREE things change vs L3-S1 — each routed to a measured-WORKS piece:
//
//   1. TRACE (stage 1, WORKS-transversal) — `wall ∩ G` → the closed interior seam
//      WLine, via `npsdetail::makeWallAdapter` (the NURBS operand front-end) ∩ the
//      curved cutter's own `ssidetail::CurvedSolid::adapter()` (the Cylinder/Sphere/
//      Cone adapter S3 traces transversally). The WLine node carries `(u1,v1)` on the
//      NURBS wall AND `(u2,v2)` on the curved cutter G.
//   2. FIDELITY (stage 2, WLine-(u,v)-read) — the seam pcurve is READ from the WLine:
//      `(u1,v1)` on F, `(u2,v2)` on G. BOTH are round-trip-gated —
//      `S_F(u1,v1)==node` AND `S_G(u2,v2)==node` — so the seam is verified to lie on
//      BOTH surfaces (a drifted seam on EITHER operand is REJECTED, never welded). NO
//      general `constructPcurve`.
//   3. SPLIT (stage 3, WORKS-closed-seam) — `splitFaceSmoothTrim` partitions the NURBS
//      wall into disk + annulus along the seam, exactly as in L3-S1 (surface-kind
//      agnostic).
//   4. KEEP (stage 4, CURVED membership) — the keep test is a CURVED-solid membership
//      `ssidetail::classifyPoint(cutter, sample)` (inside/outside the cylinder/sphere/
//      cone), NOT a plane half-space. `KeepSide::Above` == COMMON (keep the disk INSIDE
//      the cutter G); `KeepSide::Below` == CUT (keep the annulus OUTSIDE G).
//   5. SEW (stage 5, the NEW WALL — curved↔CURVED) — the cap that closes the kept piece
//      is a patch of the CURVED cutter surface G bounded by the SAME seam. It is
//      synthesized as a deflection-bounded PLANAR-TRIANGLE FAN (the `appendMouthCap`
//      idiom): the OUTER ring is the EXACT traced seam nodes (bit-identical to the NURBS
//      disk's straight seam chords, so the M0 mesher position-welds the two watertight),
//      and every interior ring/centre point is evaluated ON G (so the cap follows G's
//      true curvature to O(1/rings²)). This is the curved-NURBS↔curved-G sew — the L3-S2
//      wall the readiness doc named as the stage-5 residual, resolved for the ANALYTIC
//      curved cutter (the tractable slice), NOT the general freeform↔freeform sew.
//
// ── SCOPE + HONESTY (the moat discipline: a bounded slice beats a shaky general one) ──
// This slice cuts ONE trimmed NURBS wall (+ its flat closing base) by an ANALYTIC
// curved cutter (Cylinder / Sphere / Cone) whose `wall ∩ G` is a CLOSED interior seam.
// Anything outside that envelope is an HONEST DECLINE (a measured `NurbsCurvedSplitDecline`,
// NULL result): a non-NURBS wall, a non-analytic-curved cutter (a freeform cutter → the
// deep-tail freeform↔freeform sew, DEFERRED), a boundary-crossing / open / non-closed
// seam, a drifted seam that fails the fidelity gate on EITHER operand, a degenerate cap,
// or a non-watertight / non-positive-volume weld. NO tolerance is weakened; the self-
// verify is the M0 mesh-level watertight + volume one. The general NURBS↔NURBS split where
// the cutter is itself FREEFORM (both operands arbitrary NURBS), closed-loop-seeding-missed
// seams, and multi-crossing / re-entrant splits stay DEFERRED (the L3 deep tail).
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// L3-S1 `npsdetail::{makeWallAdapter, seamFidelity}` + the fidelity discipline
// (`nurbs_plane_split.h`), S5-a `ssidetail::{CurvedSolid, CurvedKind, recogniseCurvedSolid,
// classifyPoint}` (`ssi_boolean.h`, the curved membership + cutter adapter), B2 SMOOTH-TRIM
// `splitFaceSmoothTrim` (`smooth_trim_split.h`), the curved-cap fan idiom's primitives
// `assemble.h::{VertexPool, detail::triangleFace}` (the SAME shared-vertex planar-facet
// weld S5-a's drill-mouth cap uses), M0 `SolidMesher::mesh` / `enclosedVolume` /
// `isWatertight`, the tessellate evaluators. Additive sibling — touches NONE of them,
// modifies `nurbs_plane_split.h` / `ssi_boolean.{h,cpp}` / `assemble.h` / `face_split.h`
// NOWHERE.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20. Substrate-gated
// (CYBERCAD_HAS_NUMSCI) because the seam is a real S3 trace, like L3-S1.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_NURBS_CURVED_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_NURBS_CURVED_SPLIT_H

#include "native/boolean/assemble.h"          // VertexPool, detail::triangleFace
#include "native/boolean/nurbs_plane_split.h" // npsdetail::{makeWallAdapter, seamFidelity}, KeepSide
#include "native/boolean/smooth_trim_split.h" // splitFaceSmoothTrim (B2 smooth-trim)
#include "native/boolean/ssi_boolean.h"       // ssidetail::{CurvedSolid, recogniseCurvedSolid, classifyPoint}
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/accessors.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

#if defined(CYBERCAD_HAS_NUMSCI)

// ─────────────────────────────────────────────────────────────────────────────
// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
// watertight keep-side solid is returned. NEVER a leaky / partial solid.
// ─────────────────────────────────────────────────────────────────────────────
enum class NurbsCurvedSplitDecline {
  Ok,
  WallNotNurbs,          ///< the wall face is not a BSpline/NURBS FaceSurface with poles
  BaseNotPlanar,         ///< the closing base face is not a Plane
  CutterNotCurved,       ///< the cutter is not an analytic Cylinder/Sphere/Cone solid
  SeamUnusable,          ///< M1 seam missing / < 3 nodes / not a closed interior loop
  SeamOffSurface,        ///< the WLine (u,v) pcurve does NOT round-trip on F or G (drifted seam)
  SmoothSplitFailed,     ///< B2 smooth-trim declined the closed-seam split
  KeepFaceUnusable,      ///< a kept sub-face has no usable outer loop / ambiguous membership
  CapDegenerate,         ///< the synthesized curved cap has < 3 nodes / ~zero area
  WeldOpen,              ///< fewer than two survivor faces (cannot bound a solid)
  NotWatertight,         ///< self-verify: the welded result is not a closed 2-manifold
  VolumeInconsistent     ///< self-verify: the enclosed volume is non-positive / NaN
};

inline const char* nurbsCurvedSplitDeclineName(NurbsCurvedSplitDecline d) noexcept {
  switch (d) {
    case NurbsCurvedSplitDecline::Ok: return "Ok";
    case NurbsCurvedSplitDecline::WallNotNurbs: return "WallNotNurbs";
    case NurbsCurvedSplitDecline::BaseNotPlanar: return "BaseNotPlanar";
    case NurbsCurvedSplitDecline::CutterNotCurved: return "CutterNotCurved";
    case NurbsCurvedSplitDecline::SeamUnusable: return "SeamUnusable";
    case NurbsCurvedSplitDecline::SeamOffSurface: return "SeamOffSurface";
    case NurbsCurvedSplitDecline::SmoothSplitFailed: return "SmoothSplitFailed";
    case NurbsCurvedSplitDecline::KeepFaceUnusable: return "KeepFaceUnusable";
    case NurbsCurvedSplitDecline::CapDegenerate: return "CapDegenerate";
    case NurbsCurvedSplitDecline::WeldOpen: return "WeldOpen";
    case NurbsCurvedSplitDecline::NotWatertight: return "NotWatertight";
    case NurbsCurvedSplitDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

// The honest witnesses of an L3-S2 split — kept-face on the seam of BOTH surfaces
// (fidelity), weld watertight, enclosed volume.
struct NurbsCurvedSplitResult {
  topo::Shape solid;                ///< the welded keep-side Solid (null on decline)
  NurbsCurvedSplitDecline decline = NurbsCurvedSplitDecline::Ok;

  double seamFidelityF = 0.0;       ///< max ‖S_F(u1,v1) − node.point‖ (S(pcurve)==C on the NURBS wall)
  double seamFidelityG = 0.0;       ///< max ‖S_G(u2,v2) − node.point‖ (S(pcurve)==C on the curved cutter)
  double seamOnSurf = 0.0;          ///< max WLine node onSurfResidual (on BOTH F and G)
  int seamNodes = 0;                ///< closed-seam node count
  double areaInside = 0.0;          ///< UV area of the enclosed disk sub-face
  double areaOutside = 0.0;         ///< UV area of the annulus sub-face
  double tilingGap = 0.0;           ///< |parent − (inside + outside)| (smooth-trim tiling residual)
  bool watertight = false;          ///< M0 self-verify: the welded result is a closed 2-manifold
  double enclosedVolume = 0.0;      ///< M0 self-verify: signed-tetra enclosed volume of the keep solid

  bool ok() const noexcept { return decline == NurbsCurvedSplitDecline::Ok && !solid.isNull(); }
};

namespace ncsdetail {

using ssidetail::classifyPoint;
using ssidetail::CurvedSolid;

// Cap ring refinement: a fixed radial refinement for the curved cap fan. The cap follows
// the cutter G's curvature to O(1/rings²) (the `appendMouthCap` bound); kept well below the
// mesh deflection so the cap contributes negligible faceting bias — a deflection bound, NOT
// a result tolerance (nothing is relaxed to force a pass).
inline constexpr int kCapRings = 24;

// Fold `u` (an angular cutter param) into the ±π window around `base` so a mean over a
// seam track is not corrupted by the periodic 2π wrap (mirrors ssi_boolean.cpp nearU).
inline double nearAngle(double base, double u) {
  const double twoPi = 6.28318530717958647692, pi = 3.14159265358979323846;
  while (u < base - pi) u += twoPi;
  while (u > base + pi) u -= twoPi;
  return u;
}

// The seam fidelity on the CURVED cutter G: verify S_G(u2,v2) round-trips to the node.
// The direct analog of npsdetail::seamFidelity (which checks the NURBS wall side); here
// the surface is the analytic cutter, evaluated by the CurvedSolid's own point().
inline double seamFidelityOnCutter(const CurvedSolid& cutter, const ssi::WLine& seam) {
  double maxFid = 0.0;
  for (const ssi::WLinePoint& p : seam.points)
    maxFid = std::max(maxFid, math::distance(cutter.point(p.u2, p.v2), p.point));
  return maxFid;
}

// Pick the keep-side NURBS sub-face by its trim centroid's CURVED-solid membership.
// faceInside is the disk the seam encloses; faceOutside is the parent annulus.
// KeepSide::Above (COMMON) keeps the sub-face INSIDE the cutter; KeepSide::Below (CUT)
// keeps the sub-face OUTSIDE the cutter. Returns the kept face, or nullopt on a
// degenerate outer loop OR an ON (boundary-coincident, ambiguous) centroid membership.
inline std::optional<topo::Shape> pickKeepByMembership(const SmoothFaceSplit& split,
                                                       const topo::FaceSurface& fs,
                                                       const topo::Location& loc,
                                                       const CurvedSolid& cutter, KeepSide side,
                                                       double tol) {
  tess::SurfaceEvaluator seval(fs, loc);
  const tess::UVRegion regIn = tess::buildRegion(split.faceInside, 16);
  if (!regIn.hasOuter() || !tess::buildRegion(split.faceOutside, 16).hasOuter())
    return std::nullopt;
  double su = 0, sv = 0;
  for (const tess::UV& q : regIn.outer) { su += q.u; sv += q.v; }
  const double inv = 1.0 / static_cast<double>(regIn.outer.size());
  const int m = classifyPoint(cutter, seval.value(su * inv, sv * inv), tol);
  if (m == 0) return std::nullopt;                    // ON the cutter → ambiguous → decline
  const bool insideIsKept = (side == KeepSide::Above); // Above=COMMON keeps INSIDE G
  const bool keepInside = (m > 0) == insideIsKept;     // m>0 == inside
  return keepInside ? split.faceInside : split.faceOutside;
}

// Synthesize the CURVED cap on the cutter G bounded by the closed seam, as a deflection-
// bounded PLANAR-TRIANGLE FAN (the `appendMouthCap` idiom). The OUTER ring is the EXACT
// 3-D seam nodes (`seam3d`, bit-identical to the NURBS disk's straight seam chords, so the
// two weld watertight through the SHARED `pool`); every interior ring/centre point is
// evaluated ON G at the (u,v) linearly interpolated from the mouth centre to the boundary
// node, so the cap follows G's curvature. Each facet is a PLANAR triangle through three
// SHARED pool vertices, oriented so its normal faces AWAY from the kept material (outward
// for the keep solid). `capUV[k]` is the seam node k's (u,v) on the cutter G (from the
// WLine's (u2,v2)). Returns the fan faces appended to `faces`; no-op (leaves faces empty)
// on a degenerate seam.
inline void synthCurvedCap(const CurvedSolid& cutter,
                           const std::vector<math::Point3>& seam3d,
                           const std::vector<std::pair<double, double>>& capUV, KeepSide side,
                           VertexPool& pool, std::vector<topo::Shape>& faces) {
  const int n = static_cast<int>(seam3d.size());
  if (n < 3 || static_cast<int>(capUV.size()) != n) return;

  // Mouth centre in the cutter's (u,v): mean of the boundary track (u folded contiguous
  // around the first node so the periodic wrap does not corrupt the mean).
  const double u0 = capUV.front().first;
  double uSum = 0.0, vSum = 0.0;
  for (const auto& p : capUV) { uSum += nearAngle(u0, p.first); vSum += p.second; }
  const double uc = uSum / n, vc = vSum / n;
  const math::Point3 centre = cutter.point(uc, vc);

  // Ring r (1..rings) node k: (u,v) = lerp(centre_uv → boundary_uv[k], r/rings), evaluated
  // ON G. The outer ring (r==rings) reuses the EXACT traced seam node (not a re-evaluation)
  // so the cap↔disk seam is bit-identical.
  auto ringPt = [&](int r, int k) -> math::Point3 {
    if (r == kCapRings) return seam3d[k];
    const double t = static_cast<double>(r) / kCapRings;
    const double u = uc + (nearAngle(u0, capUV[k].first) - uc) * t;
    const double v = vc + (capUV[k].second - vc) * t;
    return cutter.point(u, v);
  };

  // A single facet through three SHARED pool vertices, its plane normal oriented AWAY from
  // the kept solid's interior. For KeepSide::Above (COMMON, keep the disk INSIDE the cutter)
  // the outward cap normal points AWAY from the cutter axis/centre (the +radial of G, i.e.
  // G's own outward surface normal at the facet); for KeepSide::Below (CUT, keep the annulus
  // OUTSIDE the cutter) the outward cap normal points TOWARD the cutter interior (the −radial
  // of G — the cavity wall of the removed slug). We orient off the cutter's radial at the
  // facet centroid and flip by keep side. (Mirrors ssi_boolean.cpp appendMouthCap.)
  const double keepSign = (side == KeepSide::Above) ? 1.0 : -1.0;
  auto radialAt = [&](const math::Point3& p) -> math::Vec3 {
    const math::Vec3 w = p - cutter.frame.origin;
    if (cutter.kind == ssidetail::CurvedKind::Sphere) return w;  // radial = from centre
    // Cylinder / Cone: radial = component perpendicular to the axis.
    return w - cutter.frame.z.vec() * math::dot(w, cutter.frame.z.vec());
  };
  auto tri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c) {
    const math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::norm(nrm) < 1e-14) return;  // degenerate sliver → skip (no T-junction: interior only)
    const math::Point3 ctr{(a.x + b.x + c.x) / 3.0, (a.y + b.y + c.y) / 3.0,
                           (a.z + b.z + c.z) / 3.0};
    const math::Vec3 outward = radialAt(ctr) * keepSign;
    const topo::Shape va = pool.vertexFor(a), vb = pool.vertexFor(b), vc2 = pool.vertexFor(c);
    const bool aligned = math::dot(nrm, outward) >= 0.0;
    const math::Vec3 oN = aligned ? nrm : math::Vec3{-nrm.x, -nrm.y, -nrm.z};
    const math::Ax3 fr = math::Ax3::fromAxisAndRef(a, math::Dir3{oN}, math::Dir3{b - a});
    faces.push_back(aligned ? detail::triangleFace(va, vb, vc2, fr)
                            : detail::triangleFace(va, vc2, vb, fr));
  };

  // Innermost ring: a fan from the centre to ring-1 nodes.
  for (int k = 0; k < n; ++k) tri(centre, ringPt(1, k), ringPt(1, (k + 1) % n));
  // Outer rings: a quad strip between ring r-1 and ring r, split into two triangles.
  for (int r = 2; r <= kCapRings; ++r)
    for (int k = 0; k < n; ++k) {
      const int kn = (k + 1) % n;
      tri(ringPt(r - 1, k), ringPt(r, k), ringPt(r, kn));
      tri(ringPt(r - 1, k), ringPt(r, kn), ringPt(r - 1, kn));
    }
}

// The kept flat base — split.faceInside's disk is welded to the CURVED cap; the flat lid
// (`base`) is kept whole when it sits on the keep side of the cutter (its centroid's
// CURVED membership). Emitted as-is (a genuine planar face) so it meshes exactly.
inline void appendKeptBase(const topo::Shape& base, const topo::FaceSurfaceResult& bsr,
                           const CurvedSolid& cutter, KeepSide side, double tol,
                           std::vector<topo::Shape>& faces) {
  const tess::UVRegion baseReg = tess::buildRegion(base, 16);
  if (!baseReg.hasOuter()) return;
  tess::SurfaceEvaluator beval(*bsr.surface, bsr.location);
  double su = 0, sv = 0;
  for (const tess::UV& q : baseReg.outer) { su += q.u; sv += q.v; }
  const double inv = 1.0 / static_cast<double>(baseReg.outer.size());
  const int m = classifyPoint(cutter, beval.value(su * inv, sv * inv), tol);
  const bool insideIsKept = (side == KeepSide::Above);
  if (m != 0 && ((m > 0) == insideIsKept)) faces.push_back(base);
}

}  // namespace ncsdetail

// ─────────────────────────────────────────────────────────────────────────────
// nurbsFaceCurvedSplit — the L3-S2 verb. `wall` is a trimmed NURBS FACE (Kind::BSpline,
// non-rational first / rational admitted) whose `wall ∩ G` is a CLOSED interior seam;
// `base` is the flat Plane face that closes the operand together with the wall; `cutter`
// is an ANALYTIC CURVED solid (Cylinder / Sphere / Cone — recognised by S5-a
// `recogniseCurvedSolid`). Given the keep `side`, build the welded keep-side Solid, gated
// by the fidelity check (S(pcurve)==C on BOTH F and G) and the mandatory M0 watertight +
// volume self-verify — or a measured decline (NULL solid).
//
// KeepSide::Above == COMMON — keep the material INSIDE the cutter G (the disk the seam
// encloses + the curved-G cap). KeepSide::Below == CUT — keep the material OUTSIDE the
// cutter (the annulus + the flat base + the curved-G cavity cap). The two keep sides
// partition the operand: V(above) + V(below) = V(full).
// ─────────────────────────────────────────────────────────────────────────────
inline NurbsCurvedSplitResult nurbsFaceCurvedSplit(const topo::Shape& wall, const topo::Shape& base,
                                                   const topo::Shape& cutter, KeepSide side,
                                                   double meshDeflection = 0.01) {
  NurbsCurvedSplitResult r;
  auto fail = [&](NurbsCurvedSplitDecline d) -> NurbsCurvedSplitResult {
    r.decline = d;
    r.solid = topo::Shape{};
    return r;
  };

  // ── (0) admit the operands: a NURBS wall + a flat base + an analytic curved cutter ──
  const auto wsr = topo::surfaceOf(wall);
  if (!wsr || !wsr->surface) return fail(NurbsCurvedSplitDecline::WallNotNurbs);
  const topo::FaceSurface& fs = *wsr->surface;
  const bool isNurbs = (fs.kind == topo::FaceSurface::Kind::BSpline) && !fs.poles.empty() &&
                       fs.nPolesU >= 2 && fs.nPolesV >= 2;
  if (!isNurbs) return fail(NurbsCurvedSplitDecline::WallNotNurbs);
  const topo::Location loc = wsr->location;

  const auto bsr = topo::surfaceOf(base);
  if (!bsr || !bsr->surface || bsr->surface->kind != topo::FaceSurface::Kind::Plane)
    return fail(NurbsCurvedSplitDecline::BaseNotPlanar);

  const std::optional<ncsdetail::CurvedSolid> cutOpt = ssidetail::recogniseCurvedSolid(cutter);
  if (!cutOpt) return fail(NurbsCurvedSplitDecline::CutterNotCurved);
  const ncsdetail::CurvedSolid& G = *cutOpt;

  // ── (1) TRACE (stage 1): wall ∩ G → the closed interior seam WLine ──────────
  const ssi::SurfaceAdapter A = npsdetail::makeWallAdapter(fs);
  const ssi::SurfaceAdapter B = G.adapter();
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  const ssi::WLine* best = nullptr;
  for (const ssi::WLine& w : tr.lines) {
    if (w.points.size() < 3) continue;
    if (w.isClosed()) { best = &w; break; }
    if (!best || w.points.size() > best->points.size()) best = &w;
  }
  if (!best || best->points.size() < 3) return fail(NurbsCurvedSplitDecline::SeamUnusable);
  const ssi::WLine seam = *best;
  r.seamNodes = static_cast<int>(seam.points.size());

  // ── (2) FIDELITY GATE (stage 2): S_F(u1,v1)==node AND S_G(u2,v2)==node ──────
  // The seam pcurve on the NURBS wall is (u1,v1); on the curved cutter is (u2,v2). BOTH
  // must round-trip — the seam lies on BOTH surfaces — else a drifted seam is REJECTED
  // (never welded). The tolerance is scale-relative to the wall's control-net diagonal
  // (never widened to force a pass).
  math::Point3 lo = fs.poles.front(), hi = fs.poles.front();
  for (const math::Point3& p : fs.poles) {
    lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
    hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
  }
  const double diag = std::max(math::distance(lo, hi), 1e-9);
  npsdetail::seamFidelity(fs, loc, seam, r.seamFidelityF, r.seamOnSurf);
  r.seamFidelityG = ncsdetail::seamFidelityOnCutter(G, seam);
  const double fidTol = 1e-6 * std::max(diag, 1.0);
  if (!(r.seamFidelityF <= fidTol) || !(r.seamFidelityG <= fidTol) || !(r.seamOnSurf <= fidTol))
    return fail(NurbsCurvedSplitDecline::SeamOffSurface);

  // ── (3) SPLIT (stage 3): smooth-trim the NURBS wall along the closed seam ───
  const SmoothSplitResult sr = splitFaceSmoothTrim(wall, seam);
  if (!sr.ok()) return fail(NurbsCurvedSplitDecline::SmoothSplitFailed);
  const SmoothFaceSplit& split = *sr.split;
  r.areaInside = split.areaInside;
  r.areaOutside = split.areaOutside;
  r.tilingGap = sr.tilingGap;

  // ── (4) KEEP (stage 4): pick the keep-side NURBS sub-face by CURVED membership ─
  const double memTol = 1e-7 * std::max(diag, 1.0);
  const auto keepFaceOpt = ncsdetail::pickKeepByMembership(split, fs, loc, G, side, memTol);
  if (!keepFaceOpt) return fail(NurbsCurvedSplitDecline::KeepFaceUnusable);

  // ── (5) SEW (stage 5, curved↔CURVED): kept NURBS sub-face + curved-G cap + the ─
  //         kept flat base → Shell → Solid. The curved cap fans over the SAME seam
  //         nodes the NURBS disk uses (bit-identical outer ring), so the M0 mesher
  //         position-welds the NURBS sub-face AND the curved cap watertight. ────────
  VertexPool pool;
  std::vector<topo::Shape> faces;
  faces.push_back(*keepFaceOpt);

  // The flat base is kept when its centroid is on the keep side of the cutter.
  ncsdetail::appendKeptBase(base, *bsr, G, side, memTol, faces);

  // The shared closed seam lifted to 3-D on the NURBS wall (bit-identical to
  // splitFaceSmoothTrim's seam loop), plus its (u,v) track on the cutter G (from the
  // WLine's (u2,v2), in the SAME node order splitFaceSmoothTrim consumes).
  const std::vector<math::Point3> seam3d = cwcdetail::seamLoop3d(fs, loc, split.seam);
  if (seam3d.size() < 3) return fail(NurbsCurvedSplitDecline::CapDegenerate);
  // Map each split-seam UV node back to the WLine's cutter (u2,v2) by nearest-node match
  // (the split seam drops a duplicated closing node but preserves node order & positions).
  std::vector<std::pair<double, double>> capUV(seam3d.size());
  {
    const std::size_t nS = seam3d.size();
    for (std::size_t i = 0; i < nS; ++i) {
      double bestD = 1e300;
      std::pair<double, double> bestUV{0.0, 0.0};
      for (const ssi::WLinePoint& p : seam.points) {
        const double d = math::distance(p.point, seam3d[i]);
        if (d < bestD) { bestD = d; bestUV = {p.u2, p.v2}; }
      }
      capUV[i] = bestUV;
    }
  }
  const std::size_t beforeCap = faces.size();
  ncsdetail::synthCurvedCap(G, seam3d, capUV, side, pool, faces);
  if (faces.size() <= beforeCap) return fail(NurbsCurvedSplitDecline::CapDegenerate);

  if (faces.size() < 2) return fail(NurbsCurvedSplitDecline::WeldOpen);
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});

  // ── (6) mandatory self-verify: mesh; require watertight AND positive volume ──
  tess::MeshParams mp;
  mp.deflection = meshDeflection;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
  r.watertight = tess::isWatertight(m);
  if (!r.watertight) return fail(NurbsCurvedSplitDecline::NotWatertight);
  r.enclosedVolume = tess::enclosedVolume(m);
  if (!(r.enclosedVolume > 0.0) || std::isnan(r.enclosedVolume))
    return fail(NurbsCurvedSplitDecline::VolumeInconsistent);

  r.solid = solid;
  r.decline = NurbsCurvedSplitDecline::Ok;
  return r;
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_NURBS_CURVED_SPLIT_H
