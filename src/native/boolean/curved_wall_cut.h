// SPDX-License-Identifier: Apache-2.0
//
// curved_wall_cut.h — MOAT M2 curved-wall freeform half-space CUT / COMMON: the
// weld verb that CONSUMES the B2 smooth-trim enabler (`splitFaceSmoothTrim`).
//
// ── ROLE (the curved-wall companion of the B4 half-space cut) ────────────────────
// `half_space_cut.h` (`freeformHalfSpaceCut`) lands the freeform↔analytic half-space
// CUT for the pose where the cut plane produces an OPEN seam CHORD on the freeform
// wall — a bowl-lidded prism cut by a VERTICAL plane, split by byte-frozen B2
// `splitFace` (`crossings == 2`). It CANNOT handle the DUAL pose the roadmap named as
// the sharpened next blocker: a dome / bowl solid cut by a HORIZONTAL plane, whose
// `wall ∩ plane` seam is a CLOSED SMOOTH curve (a circle) INTERIOR to the freeform
// wall (`crossings == 0`) — exactly the case B2 `splitFace` DECLINES and the landed
// B2 smooth-trim sibling `splitFaceSmoothTrim` (`smooth_trim_split.h`) resolves into a
// disk + an annulus-with-hole.
//
// This verb is that curved-wall weld. Given an admitted freeform operand whose single
// freeform wall is cut by a plane along a CLOSED interior seam, it:
//
//   1. TRACE — `wall ∩ P` → the closed circular seam WLine (M1, reusing the
//      byte-unchanged `hscdetail::traceWallSeam`, which already accepts a `Closed`
//      WLine).
//   2. SMOOTH-TRIM SPLIT — `splitFaceSmoothTrim(wall, seam)` partitions the freeform
//      wall into `faceInside` (the disk the seam encloses) + `faceOutside` (the parent
//      minus that disk, seam as a hole). B2 smooth-trim is CONSUMED byte-identical.
//   3. WALL SPLIT — every planar analytic face the plane crosses is split along its
//      `Face ∩ P` line by byte-frozen `hscdetail::cutAnalyticFace`; faces entirely on
//      one side are kept / dropped whole. (For the reachable dome pose the seam is
//      interior to the ONE freeform wall and the only other face — the flat base — is
//      entirely on the keep side, so no analytic split fires; the machinery is kept so
//      a walled bowl/dome cut mid-wall still routes here.)
//   4. CIRCULAR CAP SYNTH — synthesize ONE flat planar cap on `P` bounded by the seam
//      polyline (the SAME straight-edge 3-D nodes the smooth-trim split laid on the
//      freeform sub-face), oriented so its normal faces the DISCARD side. Because both
//      the cap and the kept freeform sub-face's seam are the SAME straight chords
//      between the SAME 3-D nodes, the M0 mesher position-welds them watertight with NO
//      tessellator change.
//   5. WELD + SELF-VERIFY — kept freeform sub-face + kept analytic faces + the flat cap
//      → Shell → Solid; mesh (M0) and require watertight AND a consistent enclosed
//      volume. ANY decline → NULL Shape (→ OCCT fall-through). NEVER a leaky/partial
//      solid; NO tolerance widened.
//
// `KeepSide::Below` keeps the material where the cut removes the cap ABOVE the seam
// (CUT); `KeepSide::Above` keeps the sliced-off cap (COMMON) — the same partition
// closure the B4 CUT/COMMON pair uses (`V(below)+V(above)=V(full)`).
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// B1 `recogniseFreeformSolid`, B2 SMOOTH-TRIM `splitFaceSmoothTrim`, B4
// `hscdetail::{cutAnalyticFace, planarFaceFromLoop, edgeFromPiece, signedDist,
// onKeepSide, traceWallSeam}`, M0 `SolidMesher::mesh` / `enclosedVolume` /
// `isWatertight`, the tessellate evaluators. Additive sibling — touches NONE of them,
// nor `freeformHalfSpaceCut`, nor B2 `splitFace`.
//
// ── SCOPE + HONESTY ─────────────────────────────────────────────────────────────
// This slice handles a half-space cut of an operand with ONE freeform wall whose
// `wall ∩ P` is a CLOSED interior seam, with PLANAR analytic faces. Anything outside
// that envelope is an HONEST DECLINE (a measured `CurvedWallCutDecline`, NULL result).
// No tolerance is weakened; the closed-interior-loop test is `splitFaceSmoothTrim`'s
// geometry predicate and the self-verify is the M0 mesh-level watertight + volume one.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_CURVED_WALL_CUT_H
#define CYBERCAD_NATIVE_BOOLEAN_CURVED_WALL_CUT_H

#include "native/boolean/freeform_operand.h"
#include "native/boolean/half_space_cut.h"       // KeepSide, hscdetail::*, HalfSpaceCutDecline
#include "native/boolean/smooth_trim_split.h"     // splitFaceSmoothTrim (B2 smooth-trim)
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
// watertight keep-side solid is returned.
// ─────────────────────────────────────────────────────────────────────────────
enum class CurvedWallCutDecline {
  Ok,
  NotAdmitted,           ///< B1 declined the operand
  NoFreeformFace,        ///< no freeform wall (analytic paths own it)
  MultiFreeformFace,     ///< more than one freeform wall — beyond this slice
  WallSurfaceUnusable,   ///< the freeform wall is not a Bézier with poles
  SeamUnusable,          ///< M1 seam missing / < 3 nodes / not a closed interior loop
  SmoothSplitFailed,     ///< B2 smooth-trim declined the closed-seam split
  AnalyticCutFailed,     ///< a crossed analytic face declined (edge kind / crossings)
  CapDegenerate,         ///< the synthesized circular cap has < 3 nodes / ~zero area
  WeldOpen,              ///< fewer than three survivor faces (cannot bound a solid)
  NotWatertight,         ///< self-verify: the welded result is not a closed 2-manifold
  VolumeInconsistent     ///< self-verify: the enclosed volume is non-positive / NaN
};

inline const char* curvedWallDeclineName(CurvedWallCutDecline d) noexcept {
  switch (d) {
    case CurvedWallCutDecline::Ok: return "Ok";
    case CurvedWallCutDecline::NotAdmitted: return "NotAdmitted";
    case CurvedWallCutDecline::NoFreeformFace: return "NoFreeformFace";
    case CurvedWallCutDecline::MultiFreeformFace: return "MultiFreeformFace";
    case CurvedWallCutDecline::WallSurfaceUnusable: return "WallSurfaceUnusable";
    case CurvedWallCutDecline::SeamUnusable: return "SeamUnusable";
    case CurvedWallCutDecline::SmoothSplitFailed: return "SmoothSplitFailed";
    case CurvedWallCutDecline::AnalyticCutFailed: return "AnalyticCutFailed";
    case CurvedWallCutDecline::CapDegenerate: return "CapDegenerate";
    case CurvedWallCutDecline::WeldOpen: return "WeldOpen";
    case CurvedWallCutDecline::NotWatertight: return "NotWatertight";
    case CurvedWallCutDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

namespace cwcdetail {

using hscdetail::onKeepSide;
using hscdetail::Piece;
using hscdetail::planarFaceFromLoop;
using hscdetail::signedDist;

// The closed seam loop lifted to 3-D on the freeform surface, in loop order. These are
// the SAME nodes `splitFaceSmoothTrim` lays as straight-edge endpoints on the freeform
// sub-face; the cap reuses them so the shared boundary position-welds bit-for-bit.
inline std::vector<math::Point3> seamLoop3d(const topo::FaceSurface& fs, const topo::Location& loc,
                                            const UVPolygon& loop) {
  tess::SurfaceEvaluator ev(fs, loc);
  std::vector<math::Point3> out;
  out.reserve(loop.size());
  for (const UV& q : loop) out.push_back(ev.value(q.u, q.v));
  return out;
}

// Signed area (magnitude) of a closed 3-D loop projected onto plane frame `fr`.
inline double loopAreaOnPlane(const std::vector<math::Point3>& loop3d, const math::Ax3& fr) {
  double a = 0.0;
  const std::size_t n = loop3d.size();
  auto uv = [&](const math::Point3& p) {
    const math::Vec3 d = p - fr.origin;
    return std::pair<double, double>{math::dot(d, fr.x.vec()), math::dot(d, fr.y.vec())};
  };
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const auto [ux, uy] = uv(loop3d[i]);
    const auto [vx, vy] = uv(loop3d[j]);
    a += vx * uy - ux * vy;
  }
  return std::fabs(0.5 * a);
}

// The recognise → trace → smooth-trim-split prologue, folded into one helper so the
// driver stays in the backend band. On success `out.*` are filled; on any decline it
// returns false with the measured blocker in `why`.
struct WallSplitCtx {
  const FreeformOperand* op = nullptr;
  const topo::FaceSurface* fs = nullptr;
  topo::Location loc;
  SmoothSplitResult sr;
  double diag = 0, band = 0, weldTol = 0;
};

inline bool recogniseTraceSplit(const FreeformOperand& op, const math::Plane& P,
                                WallSplitCtx& out, CurvedWallCutDecline& why) {
  if (op.freeform.empty()) { why = CurvedWallCutDecline::NoFreeformFace; return false; }
  if (op.freeform.size() != 1) { why = CurvedWallCutDecline::MultiFreeformFace; return false; }
  const OperandFace& wall = op.faces[op.freeform.front()];
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) { why = CurvedWallCutDecline::NoFreeformFace; return false; }
  const topo::FaceSurface& fs = *srf->surface;
  if (fs.kind != topo::FaceSurface::Kind::Bezier || fs.poles.empty()) {
    why = CurvedWallCutDecline::WallSurfaceUnusable;
    return false;
  }
  const double diag = std::max(op.bbox.diagonal(), 1e-9);
  const ssi::WLine seam = hscdetail::traceWallSeam(op, fs, P);
  if (seam.points.size() < 3) { why = CurvedWallCutDecline::SeamUnusable; return false; }
  SmoothSplitResult sr = splitFaceSmoothTrim(wall.face, seam);
  if (!sr.ok()) {
    // a non-closed / boundary-crossing seam here is not the curved-wall pose.
    const bool notPose = sr.decline == SmoothSplitDecline::SeamNotInterior ||
                         sr.decline == SmoothSplitDecline::SeamNotClosed;
    why = notPose ? CurvedWallCutDecline::SeamUnusable : CurvedWallCutDecline::SmoothSplitFailed;
    return false;
  }
  out.op = &op;
  out.fs = &fs;
  out.loc = srf->location;
  out.sr = std::move(sr);
  out.diag = diag;
  out.band = 1e-9 * diag;
  out.weldTol = 1e-7 * std::max(diag, 1.0);
  return true;
}

// Collect the kept analytic (planar) faces for the given keep side: each is split /
// kept-whole / dropped by byte-frozen `hscdetail::cutAnalyticFace`. Returns false (and
// sets `why`) on a declined analytic cut. Isolated so the driver stays in the backend
// band. (For the reachable dome pose no Split fires — every analytic face is on one
// side — but the machinery is kept so a mid-wall cut still routes correctly.)
inline bool collectKeptAnalyticFaces(const FreeformOperand& op, const math::Plane& P,
                                     KeepSide side, double band, double weldTol,
                                     std::vector<topo::Shape>& faces,
                                     CurvedWallCutDecline& why) {
  for (std::size_t idx : op.analytic) {
    const hscdetail::AnalyticCut ac = hscdetail::cutAnalyticFace(op.faces[idx], P, side, band, weldTol);
    if (ac.kind == hscdetail::AnalyticCut::Kind::KeepWhole) faces.push_back(op.faces[idx].face);
    else if (ac.kind == hscdetail::AnalyticCut::Kind::Split) faces.push_back(ac.keepFace);
    else if (ac.kind == hscdetail::AnalyticCut::Kind::Fail) {
      why = CurvedWallCutDecline::AnalyticCutFailed;
      return false;
    }
    // KeepWhole handled above; Drop contributes nothing.
  }
  return true;
}

// Pick the keep-side freeform sub-face by its trim centroid's side of `P`. faceInside is
// the disk the seam encloses; faceOutside is the parent annulus. Returns the kept face,
// or nullopt (→ SmoothSplitFailed) if a sub-region has no usable outer loop.
inline std::optional<topo::Shape> pickKeepFreeform(const SmoothFaceSplit& split,
                                                   const topo::FaceSurface& fs,
                                                   const topo::Location& loc,
                                                   const math::Plane& P, KeepSide side, double band) {
  tess::SurfaceEvaluator seval(fs, loc);
  const tess::UVRegion regIn = tess::buildRegion(split.faceInside, 16);
  if (!regIn.hasOuter() || !tess::buildRegion(split.faceOutside, 16).hasOuter()) return std::nullopt;
  double su = 0, sv = 0;
  for (const tess::UV& q : regIn.outer) { su += q.u; sv += q.v; }
  const double inv = 1.0 / static_cast<double>(regIn.outer.size());
  const double sideIn = signedDist(P, seval.value(su * inv, sv * inv));
  return onKeepSide(sideIn, side, band) ? split.faceInside : split.faceOutside;
}

// Synthesize the flat circular cross-section cap on `P` from the closed seam polyline.
// Each cap boundary edge is a STRAIGHT chord between consecutive seam nodes — the SAME
// chords `splitFaceSmoothTrim` laid on the freeform sub-face — built as ONE degree-1
// edge per segment. A straight edge is discretized by the M0 EdgeCache's endpoint-keyed
// path (order-independent, quantized), so the cap chord and the disk's coincident seam
// chord map to the SAME uniform samples and weld bit-for-bit. The cap normal faces the
// DISCARD side.
inline topo::Shape synthCircularCap(const std::vector<math::Point3>& loop3d,
                                    const math::Plane& P, KeepSide side) {
  math::Ax3 capFrame = P.pos;  // z = plane normal
  const math::Vec3 outwardCap = (side == KeepSide::Below ? 1.0 : -1.0) * P.pos.z.vec();
  std::vector<Piece> loop;
  const std::size_t n = loop3d.size();
  loop.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    loop.push_back(Piece{loop3d[i], loop3d[(i + 1) % n]});  // one straight chord per segment
  return planarFaceFromLoop(loop, capFrame, outwardCap);
}

}  // namespace cwcdetail

// ─────────────────────────────────────────────────────────────────────────────
// The curved-wall CUT / COMMON assembler. Composes recognise[B1] → trace[M1] →
// smooth-trim split[B2] → analytic wall split[B4] → circular cap synth → M0 weld →
// mandatory watertight + volume self-verify. Returns a NULL Shape (→ OCCT fall-through)
// on ANY decline; NEVER emits a leaky / partial solid. `KeepSide::Below` = CUT (remove
// the cap above the seam); `KeepSide::Above` = COMMON (keep the sliced-off cap).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape curvedWallHalfSpaceCut(const topo::Shape& operand, const math::Plane& P,
                                          KeepSide side, double meshDeflection = 0.01,
                                          CurvedWallCutDecline* why = nullptr) {
  using namespace hscdetail;
  using namespace cwcdetail;
  auto fail = [&](CurvedWallCutDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  // (1–3) B1 recognise → M1 trace → B2 smooth-trim split (prologue helper).
  const auto op = recogniseFreeformSolid(operand);
  if (!op) return fail(CurvedWallCutDecline::NotAdmitted);
  WallSplitCtx ctx;
  CurvedWallCutDecline preWhy = CurvedWallCutDecline::Ok;
  if (!recogniseTraceSplit(*op, P, ctx, preWhy)) return fail(preWhy);
  const SmoothFaceSplit& split = *ctx.sr.split;
  const topo::FaceSurface& fs = *ctx.fs;
  const double band = ctx.band;

  // (4) pick the keep-side freeform sub-face by its trim centroid's side of P.
  const auto freeKeepOpt = pickKeepFreeform(split, fs, ctx.loc, P, side, band);
  if (!freeKeepOpt) return fail(CurvedWallCutDecline::SmoothSplitFailed);

  // (5) collect the freeform keep-face + every kept analytic face.
  std::vector<topo::Shape> faces;
  faces.push_back(*freeKeepOpt);
  CurvedWallCutDecline anWhy = CurvedWallCutDecline::Ok;
  if (!collectKeptAnalyticFaces(*op, P, side, band, ctx.weldTol, faces, anWhy)) return fail(anWhy);

  // (6) synthesize the flat circular cross-section cap on P by REUSING the disk's own
  // seam edges (faceInside's outer wire IS the seam loop, for BOTH keep sides): sharing
  // the edge NODES makes the M0 mesher discretize each seam edge ONCE and pin the disk
  // AND the cap to bit-identical samples (a resonance-free watertight weld).
  const std::vector<math::Point3> seam3d = seamLoop3d(fs, ctx.loc, split.seam);
  if (seam3d.size() < 3) return fail(CurvedWallCutDecline::CapDegenerate);
  if (loopAreaOnPlane(seam3d, P.pos) < band * std::max(ctx.diag, 1.0))
    return fail(CurvedWallCutDecline::CapDegenerate);
  faces.push_back(synthCircularCap(seam3d, P, side));

  // (7) weld → Solid. A curved disk + a flat cap already bound a closed solid (the
  // dome/bowl-cut-below pose has just those two faces); ≥ 2 survivors is the floor.
  if (faces.size() < 2) return fail(CurvedWallCutDecline::WeldOpen);
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});

  // (8) mandatory self-verify: mesh; require watertight AND a positive enclosed volume.
  tess::MeshParams mp; mp.deflection = meshDeflection;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
  if (!tess::isWatertight(m)) return fail(CurvedWallCutDecline::NotWatertight);
  const double vol = tess::enclosedVolume(m);
  if (!(vol > 0.0) || std::isnan(vol)) return fail(CurvedWallCutDecline::VolumeInconsistent);

  if (why) *why = CurvedWallCutDecline::Ok;
  return solid;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_CURVED_WALL_CUT_H
