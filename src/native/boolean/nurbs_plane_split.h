// SPDX-License-Identifier: Apache-2.0
//
// nurbs_plane_split.h — NURBS roadmap LAYER 3, SLICE 1 (L3-S1): the FIRST
// exact-NURBS B-rep boolean — a genuine NURBS face SPLIT BY A PLANE, welded
// watertight.
//
// ── ROLE (the flagship exact-NURBS boolean entry point) ─────────────────────────
// `curved_wall_cut.h` (`curvedWallHalfSpaceCut`) lands the closed-interior-seam
// half-space CUT / COMMON for a freeform WALL that is a degree-elevated **Bézier**
// patch (`FaceSurface::Kind::Bezier`). L3-S1 is that SAME weld verb with the wall's
// surface kind left as a **genuine B-spline / NURBS** (`Kind::BSpline`, non-rational
// first, rational admitted) — the first boolean whose curved operand is an exact
// NURBS face rather than a single Bézier patch. It is deliberately composed to
// touch ONLY the pieces the L3 readiness harness measured as WORKS / near-WORKS
// (openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md §3, the L3-S1 recipe) and to route
// around EVERY MISSING stage:
//
//   1. TRACE (stage 1, WORKS-transversal) — `wall ∩ P` → the closed interior seam
//      WLine, via `ssi::makeBSplineAdapter` / `ssi::makeNurbsAdapter` (the NURBS
//      operand front-end the harness measured to 5.6e-16 on a rational-NURBS
//      cylinder ∩ plane) + `ssi::makePlaneAdapter` + `ssi::trace_intersection`.
//   2. PCURVE (stage 2, ROUTED AROUND) — the seam pcurve on the NURBS wall is READ
//      DIRECTLY from the WLine's per-node `(u1,v1)` (each WLine node already carries
//      its (u,v) on the NURBS operand). This slice DOES NOT call the general
//      `constructPcurve` (the measured stage-2 residual). The read is gated by an
//      explicit fidelity check S(u1,v1) == node.point ≤ onSurfTol (the S(pcurve)==C
//      invariant — a drifted seam is REJECTED, never welded).
//   3. SPLIT (stage 3, WORKS-closed-seam) — `splitFaceSmoothTrim` (surface-kind
//      agnostic: it reads the wall through `topo::SurfaceEvaluator`, which evaluates
//      a BSpline/NURBS grid natively) partitions the NURBS wall into the enclosed
//      disk + the annulus (seam as a hole). Its own tiling self-verify holds on the
//      NURBS grid.
//   4. KEEP (stage 4, closed-form) — a plane HALF-SPACE side test at the kept
//      sub-face's trim centroid (`signedDist` + `onKeepSide`). No general NURBS
//      solid membership.
//   5. WELD (stage 5, M0w curved↔FLAT) — synthesize ONE flat cap on `P` bounded by
//      the seam polyline (reusing `cwcdetail::synthCircularCap` — the same straight
//      chords `splitFaceSmoothTrim` laid on the NURBS sub-face, so the M0 mesher
//      position-welds them watertight) + the kept flat base face → Shell → Solid;
//      mesh (M0) and require watertight AND a positive enclosed volume. It is the
//      curved-NURBS↔flat weld — it AVOIDS the MISSING freeform↔freeform sew.
//
// `KeepSide::Below` keeps the material below `P` (the cup — CUT removes the cap);
// `KeepSide::Above` keeps the material above `P` (COMMON). The two keep sides
// partition the operand: V(below) + V(above) = V(full).
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// `ssi::{makeBSplineAdapter, makeNurbsAdapter, makePlaneAdapter, trace_intersection}`,
// B2 SMOOTH-TRIM `splitFaceSmoothTrim` (`smooth_trim_split.h`), the curved-wall weld
// helpers `cwcdetail::{seamLoop3d, loopAreaOnPlane, synthCircularCap}` +
// `hscdetail::{signedDist, onKeepSide}` (`half_space_cut.h` / `curved_wall_cut.h`),
// M0 `SolidMesher::mesh` / `enclosedVolume` / `isWatertight`, the tessellate
// evaluators. Additive sibling — touches NONE of them, modifies `assemble.h` /
// `face_split.h` NOWHERE.
//
// ── SCOPE + HONESTY ─────────────────────────────────────────────────────────────
// This first slice handles ONE trimmed NURBS wall whose `wall ∩ P` is a CLOSED
// interior seam, plus its flat closing base (a Plane face). Anything outside that
// envelope is an HONEST DECLINE (a measured `NurbsPlaneSplitDecline`, NULL result):
// a non-NURBS wall, a boundary-crossing / open / non-closed seam (stage-1 recall
// gap or a multi-crossing split), a drifted seam that fails the fidelity gate, a
// degenerate cap, or a non-watertight / non-positive-volume weld. NO tolerance is
// weakened; the self-verify is the M0 mesh-level watertight + volume one. The
// general NURBS↔NURBS split (both operands curved), closed-loop-seeding-missed
// seams, and multi-crossing / re-entrant splits stay DEFERRED (the L3 deep tail).
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_NURBS_PLANE_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_NURBS_PLANE_SPLIT_H

#include "native/boolean/curved_wall_cut.h"       // cwcdetail::{seamLoop3d, loopAreaOnPlane, synthCircularCap}
#include "native/boolean/half_space_cut.h"        // KeepSide, hscdetail::{signedDist, onKeepSide}
#include "native/boolean/smooth_trim_split.h"     // splitFaceSmoothTrim (B2 smooth-trim)
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"                    // makeBSplineAdapter / makeNurbsAdapter / makePlaneAdapter
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/topology/accessors.h"             // surfaceOf
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
// watertight keep-side solid is returned. NEVER a leaky / partial solid.
// ─────────────────────────────────────────────────────────────────────────────
enum class NurbsPlaneSplitDecline {
  Ok,
  WallNotNurbs,          ///< the wall face is not a BSpline/NURBS FaceSurface with poles
  BaseNotPlanar,         ///< the closing base face is not a Plane
  SeamUnusable,          ///< M1 seam missing / < 3 nodes / not a closed interior loop
  SeamOffSurface,        ///< the WLine-(u,v) pcurve does NOT round-trip: S(u,v) != node (drifted seam)
  SmoothSplitFailed,     ///< B2 smooth-trim declined the closed-seam split
  KeepFaceUnusable,      ///< a kept sub-face has no usable outer loop
  CapDegenerate,         ///< the synthesized flat cap has < 3 nodes / ~zero area
  WeldOpen,              ///< fewer than two survivor faces (cannot bound a solid)
  NotWatertight,         ///< self-verify: the welded result is not a closed 2-manifold
  VolumeInconsistent     ///< self-verify: the enclosed volume is non-positive / NaN
};

inline const char* nurbsPlaneSplitDeclineName(NurbsPlaneSplitDecline d) noexcept {
  switch (d) {
    case NurbsPlaneSplitDecline::Ok: return "Ok";
    case NurbsPlaneSplitDecline::WallNotNurbs: return "WallNotNurbs";
    case NurbsPlaneSplitDecline::BaseNotPlanar: return "BaseNotPlanar";
    case NurbsPlaneSplitDecline::SeamUnusable: return "SeamUnusable";
    case NurbsPlaneSplitDecline::SeamOffSurface: return "SeamOffSurface";
    case NurbsPlaneSplitDecline::SmoothSplitFailed: return "SmoothSplitFailed";
    case NurbsPlaneSplitDecline::KeepFaceUnusable: return "KeepFaceUnusable";
    case NurbsPlaneSplitDecline::CapDegenerate: return "CapDegenerate";
    case NurbsPlaneSplitDecline::WeldOpen: return "WeldOpen";
    case NurbsPlaneSplitDecline::NotWatertight: return "NotWatertight";
    case NurbsPlaneSplitDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

// The honest witnesses of an L3-S1 split — the numbers the two-gate discipline
// checks (kept-face on the seam, seam fidelity, weld watertight, enclosed volume).
struct NurbsPlaneSplitResult {
  topo::Shape solid;                ///< the welded keep-side Solid (null on decline)
  NurbsPlaneSplitDecline decline = NurbsPlaneSplitDecline::Ok;

  double seamFidelity = 0.0;        ///< max ‖S(u1,v1) − node.point‖ over the seam (the S(pcurve)==C witness)
  double seamOnSurf = 0.0;          ///< max WLine node onSurfResidual (on BOTH F and P)
  int seamNodes = 0;                ///< closed-seam node count
  double areaInside = 0.0;          ///< UV area of the enclosed disk sub-face
  double areaOutside = 0.0;         ///< UV area of the annulus sub-face
  double tilingGap = 0.0;           ///< |parent − (inside + outside)| (smooth-trim tiling residual)
  bool watertight = false;          ///< M0 self-verify: the welded result is a closed 2-manifold
  double enclosedVolume = 0.0;      ///< M0 self-verify: signed-tetra enclosed volume of the keep solid

  bool ok() const noexcept { return decline == NurbsPlaneSplitDecline::Ok && !solid.isNull(); }
};

namespace npsdetail {

using hscdetail::onKeepSide;
using hscdetail::signedDist;

// Build an SSI SurfaceAdapter for a NURBS/BSpline `FaceSurface` — the L3 operand
// front-end. Rational (non-empty weights) → makeNurbsAdapter; non-rational →
// makeBSplineAdapter. Both bound the surface by the control-net convex hull (sound
// for wᵢ > 0), evaluate via native-math surfacePoint / nurbsSurfacePoint. This is the
// direct NURBS analog of hscdetail::traceWallSeam's Bézier `makeBezierAdapter`.
inline ssi::SurfaceAdapter makeWallAdapter(const topo::FaceSurface& fs) {
  if (fs.weights.empty())
    return ssi::makeBSplineAdapter(fs.degreeU, fs.degreeV, fs.poles, fs.nPolesU, fs.nPolesV,
                                   fs.knotsU, fs.knotsV);
  return ssi::makeNurbsAdapter(fs.degreeU, fs.degreeV, fs.poles, fs.weights, fs.nPolesU,
                               fs.nPolesV, fs.knotsU, fs.knotsV);
}

// Trace the NURBS wall ∩ P seam. The plane ParamBox is sized from the wall's own
// control-net AABB projected onto P's frame (a superset that covers the whole seam),
// mirroring hscdetail::traceWallSeam. Returns the single Closed/BoundaryExit WLine
// with ≥ 3 nodes, or an empty WLine on failure.
inline ssi::WLine traceNurbsWallSeam(const topo::FaceSurface& fs, const math::Plane& P) {
  const ssi::SurfaceAdapter A = makeWallAdapter(fs);
  double u0 = 1e30, u1 = -1e30, v0 = 1e30, v1 = -1e30;
  for (const math::Point3& w : fs.poles) {
    const math::Vec3 d = w - P.pos.origin;
    const double pu = math::dot(d, P.pos.x.vec()), pv = math::dot(d, P.pos.y.vec());
    u0 = std::min(u0, pu); u1 = std::max(u1, pu);
    v0 = std::min(v0, pv); v1 = std::max(v1, pv);
  }
  const double mu = 0.2 * std::max(u1 - u0, 1e-6), mv = 0.2 * std::max(v1 - v0, 1e-6);
  const ssi::SurfaceAdapter B =
      ssi::makePlaneAdapter(P, ssi::ParamBox{u0 - mu, u1 + mu, v0 - mv, v1 + mv});
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  const ssi::WLine* best = nullptr;
  for (const ssi::WLine& w : tr.lines) {
    if (w.points.size() < 3) continue;
    if (w.isClosed()) return w;  // prefer the closed interior loop
    if (!best || w.points.size() > best->points.size()) best = &w;
  }
  return best ? *best : ssi::WLine{};
}

// The S(pcurve)==C fidelity gate (stage 2 routed around constructPcurve): the seam
// pcurve is the WLine's per-node (u1,v1). Verify it round-trips — the NURBS surface
// evaluated at (u1,v1) must equal the node's 3-D point within tol — so a drifted seam
// (u,v not actually on the traced curve) is REJECTED, never welded. `maxFid` is the
// worst ‖S(u1,v1) − point‖; `maxOnSurf` echoes the WLine's on-both-surfaces residual.
inline void seamFidelity(const topo::FaceSurface& fs, const topo::Location& loc,
                         const ssi::WLine& seam, double& maxFid, double& maxOnSurf) {
  tess::SurfaceEvaluator ev(fs, loc);
  maxFid = 0.0;
  maxOnSurf = 0.0;
  for (const ssi::WLinePoint& p : seam.points) {
    const math::Point3 s = ev.value(p.u1, p.v1);
    maxFid = std::max(maxFid, math::distance(s, p.point));
    maxOnSurf = std::max(maxOnSurf, p.onSurfResidual);
  }
}

// Pick the keep-side NURBS sub-face by its trim centroid's side of P. `faceInside` is
// the disk the seam encloses; `faceOutside` is the parent annulus. Returns the kept
// face, or nullopt (→ KeepFaceUnusable) if a sub-region has no usable outer loop.
// (Direct analog of cwcdetail::pickKeepFreeform, on a NURBS surface.)
inline std::optional<topo::Shape> pickKeepNurbs(const SmoothFaceSplit& split,
                                                const topo::FaceSurface& fs,
                                                const topo::Location& loc, const math::Plane& P,
                                                KeepSide side, double band) {
  tess::SurfaceEvaluator seval(fs, loc);
  const tess::UVRegion regIn = tess::buildRegion(split.faceInside, 16);
  if (!regIn.hasOuter() || !tess::buildRegion(split.faceOutside, 16).hasOuter())
    return std::nullopt;
  double su = 0, sv = 0;
  for (const tess::UV& q : regIn.outer) { su += q.u; sv += q.v; }
  const double inv = 1.0 / static_cast<double>(regIn.outer.size());
  const double sideIn = signedDist(P, seval.value(su * inv, sv * inv));
  return onKeepSide(sideIn, side, band) ? split.faceInside : split.faceOutside;
}

}  // namespace npsdetail

// ─────────────────────────────────────────────────────────────────────────────
// nurbsFacePlaneSplit — the L3-S1 verb. `wall` is a trimmed NURBS FACE (Kind::BSpline,
// non-rational first / rational admitted) whose `wall ∩ P` is a CLOSED interior seam;
// `base` is the flat Plane face that closes the operand into a solid together with the
// wall (its rim is shared with the wall). Given the cut plane `P` and keep `side`,
// build the welded keep-side Solid, gated by the fidelity check (S(pcurve)==C) and the
// mandatory M0 watertight + volume self-verify — or a measured decline (NULL solid).
//
// KeepSide::Below keeps the material below `P` (removes the cap above the seam);
// KeepSide::Above keeps the material above `P`. The base is kept when it lies on the
// keep side (dome pose: the flat base is below the cut ⇒ kept for Below); a base on the
// discard side is dropped. The flat cross-section cap on `P` closes the kept piece.
// ─────────────────────────────────────────────────────────────────────────────
inline NurbsPlaneSplitResult nurbsFacePlaneSplit(const topo::Shape& wall, const topo::Shape& base,
                                                 const math::Plane& P, KeepSide side,
                                                 double meshDeflection = 0.01) {
  NurbsPlaneSplitResult r;
  auto fail = [&](NurbsPlaneSplitDecline d) -> NurbsPlaneSplitResult {
    r.decline = d;
    r.solid = topo::Shape{};
    return r;
  };

  // ── (0) admit the operands: a NURBS wall + a flat base ──────────────────────
  const auto wsr = topo::surfaceOf(wall);
  if (!wsr || !wsr->surface) return fail(NurbsPlaneSplitDecline::WallNotNurbs);
  const topo::FaceSurface& fs = *wsr->surface;
  const bool isNurbs = (fs.kind == topo::FaceSurface::Kind::BSpline) && !fs.poles.empty() &&
                       fs.nPolesU >= 2 && fs.nPolesV >= 2;
  if (!isNurbs) return fail(NurbsPlaneSplitDecline::WallNotNurbs);
  const topo::Location loc = wsr->location;

  const auto bsr = topo::surfaceOf(base);
  if (!bsr || !bsr->surface || bsr->surface->kind != topo::FaceSurface::Kind::Plane)
    return fail(NurbsPlaneSplitDecline::BaseNotPlanar);

  // ── (1) TRACE (stage 1): wall ∩ P → the closed interior seam WLine ──────────
  const ssi::WLine seam = npsdetail::traceNurbsWallSeam(fs, P);
  if (seam.points.size() < 3) return fail(NurbsPlaneSplitDecline::SeamUnusable);
  r.seamNodes = static_cast<int>(seam.points.size());

  // ── (2) FIDELITY GATE (stage 2, WLine-(u,v)-read): S(u1,v1) == node.point ───
  // A drifted seam (the read (u,v) not on the traced curve) is REJECTED here — the
  // S(pcurve)==C invariant the readiness recipe demands. The tolerance is scale-
  // relative to the wall's control-net diagonal (never widened to force a pass).
  math::Point3 lo = fs.poles.front(), hi = fs.poles.front();
  for (const math::Point3& p : fs.poles) {
    lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
    hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
  }
  const double diag = std::max(math::distance(lo, hi), 1e-9);
  const double band = 1e-9 * diag;
  npsdetail::seamFidelity(fs, loc, seam, r.seamFidelity, r.seamOnSurf);
  const double fidTol = 1e-6 * std::max(diag, 1.0);
  if (!(r.seamFidelity <= fidTol) || !(r.seamOnSurf <= fidTol))
    return fail(NurbsPlaneSplitDecline::SeamOffSurface);

  // ── (3) SPLIT (stage 3): smooth-trim the NURBS wall along the closed seam ───
  const SmoothSplitResult sr = splitFaceSmoothTrim(wall, seam);
  if (!sr.ok()) return fail(NurbsPlaneSplitDecline::SmoothSplitFailed);
  const SmoothFaceSplit& split = *sr.split;
  r.areaInside = split.areaInside;
  r.areaOutside = split.areaOutside;
  r.tilingGap = sr.tilingGap;

  // ── (4) KEEP (stage 4): pick the keep-side NURBS sub-face by half-space test ─
  const auto keepFaceOpt = npsdetail::pickKeepNurbs(split, fs, loc, P, side, band);
  if (!keepFaceOpt) return fail(NurbsPlaneSplitDecline::KeepFaceUnusable);

  // ── (5) WELD (stage 5, M0w curved↔FLAT): kept NURBS sub-face + kept flat base
  //         + the flat cross-section cap on P → Shell → Solid. The cap reuses the
  //         disk's own seam nodes (bit-identical straight chords) so the M0 mesher
  //         position-welds the NURBS disk AND the cap to the same samples. ────────
  std::vector<topo::Shape> faces;
  faces.push_back(*keepFaceOpt);

  // Keep the flat base when it sits on the keep side (its centroid's signed side of P).
  {
    const tess::UVRegion baseReg = tess::buildRegion(base, 16);
    if (baseReg.hasOuter()) {
      tess::SurfaceEvaluator beval(*bsr->surface, bsr->location);
      double su = 0, sv = 0;
      for (const tess::UV& q : baseReg.outer) { su += q.u; sv += q.v; }
      const double inv = 1.0 / static_cast<double>(baseReg.outer.size());
      const double sd = npsdetail::signedDist(P, beval.value(su * inv, sv * inv));
      if (npsdetail::onKeepSide(sd, side, band)) faces.push_back(base);
    }
  }

  const std::vector<math::Point3> seam3d = cwcdetail::seamLoop3d(fs, loc, split.seam);
  if (seam3d.size() < 3) return fail(NurbsPlaneSplitDecline::CapDegenerate);
  if (cwcdetail::loopAreaOnPlane(seam3d, P.pos) < band * std::max(diag, 1.0))
    return fail(NurbsPlaneSplitDecline::CapDegenerate);
  faces.push_back(cwcdetail::synthCircularCap(seam3d, P, side));

  if (faces.size() < 2) return fail(NurbsPlaneSplitDecline::WeldOpen);
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});

  // ── (6) mandatory self-verify: mesh; require watertight AND positive volume ──
  tess::MeshParams mp;
  mp.deflection = meshDeflection;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
  r.watertight = tess::isWatertight(m);
  if (!r.watertight) return fail(NurbsPlaneSplitDecline::NotWatertight);
  r.enclosedVolume = tess::enclosedVolume(m);
  if (!(r.enclosedVolume > 0.0) || std::isnan(r.enclosedVolume))
    return fail(NurbsPlaneSplitDecline::VolumeInconsistent);

  r.solid = solid;
  r.decline = NurbsPlaneSplitDecline::Ok;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_NURBS_PLANE_SPLIT_H
