// SPDX-License-Identifier: Apache-2.0
//
// inter_solid_seam.h — MOAT M2-FUSE / the INTER-SOLID seam-set builder: the one new
// verb the FIRST *two-operand* freeform boolean needs beyond the landed single-operand
// freeform↔planar-half-space CUT.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// The landed `half_space_cut.h` cuts a freeform operand `A` by an INFINITE planar
// half-space (one seam, one section cap). A real two-operand boolean `A ⊙ B` instead
// intersects `A` against a FINITE second operand `B`, so the intersection boundary is
// the curve where `B`'s faces meet `A`'s faces. This header assembles that
// inter-solid seam for the SIMPLEST reachable pose (design §1):
//
//   * `A` = a bowl-lidded convex-quad prism (the landed B1 operand: one Bézier wall +
//     planar walls + planar bottom).
//   * `B` = a FINITE all-planar solid (an axis-aligned box) placed so that EXACTLY ONE
//     of its planar faces (`Pcut`) slices `A`'s Bézier wall in ONE clean transversal
//     seam, and `A`'s material on `Pcut`'s interior side lies wholly inside `B`.
//
// Then the inter-solid seam is ONE closed loop on `Pcut`'s plane: the curved
// `Pcut ∩ (A's Bézier wall)` arc (traced by the LANDED `hscdetail::traceWallSeam`)
// spliced to the straight `Pcut ∩ (A's planar faces)` chords (the LANDED
// `hscdetail::cutAnalyticFace` crossings). That loop is exactly the landed CUT's
// section-cap boundary — but here it is REUSED as the shared inter-solid seam that both
// operands weld against. This builder returns that closed loop PLUS `A`'s survivor
// faces (`A` restricted to `Pcut`'s outer side, WITHOUT a cap — the cap region is now
// interior to `A ∪ B`).
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// B1 `recogniseFreeformSolid`, B2 `splitFace`, M1 `traceWallSeam`, and every
// `hscdetail::` primitive of the landed `half_space_cut.h` (Piece, cutAnalyticFace,
// orderLoop, edgeFromPiece, seamChord3d, signedDist, onKeepSide). Additive sibling —
// touches none of them, nor the analytic `recogniseCurvedSolid`/`classifyPoint`, nor
// the landed `freeformHalfSpaceCut` CUT/COMMON path.
//
// ── HONESTY ───────────────────────────────────────────────────────────────────────
// Every predicate below is a geometry test, never a fudge. If `B` does not present a
// SINGLE curved cut of `A`'s wall, or `A`'s interior-side material is not contained in
// `B`, or the seam loop does not close, the builder returns a MEASURED
// `SeamDecline` (nullopt) — a first-class outcome the caller logs before the OCCT
// fall-through. No tolerance is weakened.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_INTER_SOLID_SEAM_H
#define CYBERCAD_NATIVE_BOOLEAN_INTER_SOLID_SEAM_H

#include "native/boolean/freeform_operand.h"
#include "native/boolean/half_space_cut.h"
#include "native/boolean/polygon.h"
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;
namespace ssi  = cybercad::native::ssi;

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a closed
/// inter-solid seam and `A`-outer survivor set are returned.
enum class SeamDecline {
  Ok,
  NotPlanarB,           ///< `B` is not an all-planar solid (the box operand domain)
  NoOverlap,            ///< `A`.bbox ∩ `B`.bbox is empty — no boolean interaction
  NotSingleCurvedCut,   ///< 0 or >1 `B` faces slice `A`'s Bézier wall (beyond this slice)
  NotContained,         ///< `A`'s interior-side material is not wholly inside `B`
  SeamUnusable,         ///< M1 seam missing / < 2 nodes / wrong status
  SplitFailed,          ///< B2 declined the freeform wall split
  AnalyticCrossingNot2, ///< a crossed planar face of `A` did not cross `Pcut` twice
  UnsupportedEdgeKind,  ///< a crossed planar face carried a Circle/BSpline edge
  SectionLoopOpen       ///< the inter-solid seam pieces do not chain into a closed loop
};

inline const char* seamDeclineName(SeamDecline d) noexcept {
  switch (d) {
    case SeamDecline::Ok: return "Ok";
    case SeamDecline::NotPlanarB: return "NotPlanarB";
    case SeamDecline::NoOverlap: return "NoOverlap";
    case SeamDecline::NotSingleCurvedCut: return "NotSingleCurvedCut";
    case SeamDecline::NotContained: return "NotContained";
    case SeamDecline::SeamUnusable: return "SeamUnusable";
    case SeamDecline::SplitFailed: return "SplitFailed";
    case SeamDecline::AnalyticCrossingNot2: return "AnalyticCrossingNot2";
    case SeamDecline::UnsupportedEdgeKind: return "UnsupportedEdgeKind";
    case SeamDecline::SectionLoopOpen: return "SectionLoopOpen";
  }
  return "?";
}

/// The assembled inter-solid seam for the single-curved-cut box pose.
struct InterSolidSeam {
  std::vector<Polygon> bPolys;               ///< every planar face of `B` (outward normals)
  std::size_t pcutIdx = 0;                    ///< index into `bPolys` of the cutting face
  math::Plane tracePlane;                     ///< `Pcut` plane, z = INTO `B`'s interior
  KeepSide keepSide = KeepSide::Below;        ///< `A` keeps the side OUTSIDE `B`

  std::vector<topo::Shape> aKeepFaces;        ///< `A` restricted to `Pcut`'s outer side (NO cap)
  std::vector<topo::Shape> aDropFaces;        ///< `A`'s inner-side survivor faces (for COMMON)
  std::vector<hscdetail::Piece> capLoop;      ///< the CLOSED inter-solid seam (D-outline)
};

namespace isdetail {

using hscdetail::Piece;

/// World control poles of `A`'s Bézier wall (location-placed).
inline std::vector<math::Point3> wallWorldPoles(const OperandFace& wall) {
  const auto srf = topo::surfaceOf(wall.face);
  std::vector<math::Point3> out;
  if (!srf || !srf->surface) return out;
  const topo::Location& loc = srf->location;
  for (const math::Point3& p : srf->surface->poles)
    out.push_back(loc.isIdentity() ? p : loc.transform().applyToPoint(p));
  return out;
}

/// Does `poly`'s plane strictly separate the wall pole hull (some poles each side of
/// the plane, beyond `band`)? That is the unique-curved-cut predicate (design §2.2).
inline bool planeStraddlesWall(const Polygon& poly, const std::vector<math::Point3>& poles,
                               double band) {
  bool pos = false, neg = false;
  for (const math::Point3& p : poles) {
    const double sd = poly.plane.signedDistance(p);
    if (sd > band) pos = true;
    else if (sd < -band) neg = true;
  }
  return pos && neg;
}

/// Is every corner of `A`.bbox on the interior (≤ band) side of `poly`'s outward plane?
/// (The containment guard: `B`'s non-cutting faces do not slice `A`, and `A`'s
/// interior-side material lies within `B` — the pose the FUSE weld assumes.)
inline bool aabbInsidePlane(const Aabb& bb, const Polygon& poly, double band) {
  for (int c = 0; c < 8; ++c) {
    const math::Point3 w{(c & 1) ? bb.hi.x : bb.lo.x, (c & 2) ? bb.hi.y : bb.lo.y,
                         (c & 4) ? bb.hi.z : bb.lo.z};
    if (poly.plane.signedDistance(w) > band) return false;
  }
  return true;
}

/// Locate the unique `B` face slicing `A`'s wall, and verify every OTHER `B` face
/// contains `A` (the single-curved-cut + containment pose). Writes `pcutIdx`.
inline SeamDecline findCuttingFace(const std::vector<Polygon>& bPolys,
                                   const std::vector<math::Point3>& poles, const Aabb& aBox,
                                   double band, std::size_t& pcutIdx) {
  int nCut = 0;
  for (std::size_t i = 0; i < bPolys.size(); ++i)
    if (planeStraddlesWall(bPolys[i], poles, band)) { ++nCut; pcutIdx = i; }
  if (nCut != 1) return SeamDecline::NotSingleCurvedCut;
  for (std::size_t i = 0; i < bPolys.size(); ++i)
    if (i != pcutIdx && !aabbInsidePlane(aBox, bPolys[i], band))
      return SeamDecline::NotContained;
  return SeamDecline::Ok;
}

/// The trace plane on `Pcut`: origin on the face, z = INTO `B`'s interior (= −outward),
/// so `KeepSide::Below` keeps the side OUTSIDE `B` (design §1).
inline math::Plane tracePlaneOf(const Polygon& pcut) {
  const math::Dir3 into{-pcut.plane.normal};
  const math::Dir3 ref = std::fabs(into.z()) < 0.9 ? math::Dir3{0, 0, 1} : math::Dir3{1, 0, 0};
  math::Plane P;
  P.pos = math::Ax3::fromAxisAndRef(pcut.vertices.front(), into, ref);
  return P;
}

/// Split `A` against `Pcut`: keep-side freeform sub-face + kept/dropped analytic faces,
/// gathering the cap boundary pieces (the shared inter-solid seam). Mirrors the landed
/// `halfSpaceCut` face collection EXACTLY, but retains BOTH sides and NO cap.
struct SplitCollect {
  std::vector<topo::Shape> keepFaces;   ///< outer-side survivors (no cap)
  std::vector<topo::Shape> dropFaces;   ///< inner-side survivors (no cap) — for COMMON
  std::vector<Piece> capPieces;         ///< seam3d + straight crossings → the D-outline
  SeamDecline why = SeamDecline::Ok;
};

inline SplitCollect splitOperandA(const FreeformOperand& A, const OperandFace& wall,
                                  const topo::FaceSurface& fs, const topo::Location& loc,
                                  const math::Plane& P, KeepSide side, double band, double weldTol) {
  using namespace hscdetail;
  SplitCollect out;

  const ssi::WLine seam = traceWallSeam(A, fs, P);
  if (seam.points.size() < 2) { out.why = SeamDecline::SeamUnusable; return out; }
  const SplitResult sr = splitFace(wall.face, seam);
  if (!sr.ok()) { out.why = SeamDecline::SplitFailed; return out; }
  const std::vector<math::Point3> seam3d = seamChord3d(fs, loc, *sr.split);

  // Keep-side freeform sub-face by its trim centroid's side of P (landed logic).
  tess::SurfaceEvaluator seval(fs, loc);
  auto centroidSide = [&](const topo::Shape& sub) {
    const tess::UVRegion reg = tess::buildRegion(sub, 16);
    double su = 0, sv = 0;
    for (const tess::UV& q : reg.outer) { su += q.u; sv += q.v; }
    const double inv = 1.0 / static_cast<double>(reg.outer.size());
    return signedDist(P, seval.value(su * inv, sv * inv));
  };
  const bool keepIn = onKeepSide(centroidSide(sr.split->faceIn), side, band);
  out.keepFaces.push_back(keepIn ? sr.split->faceIn : sr.split->faceOut);
  out.dropFaces.push_back(keepIn ? sr.split->faceOut : sr.split->faceIn);
  out.capPieces.push_back(seam3d);

  // Split every planar analytic face on BOTH sides; collect the straight cap chords.
  const KeepSide other = side == KeepSide::Below ? KeepSide::Above : KeepSide::Below;
  for (std::size_t idx : A.analytic) {
    const AnalyticCut ac = cutAnalyticFace(A.faces[idx], P, side, band, weldTol);
    switch (ac.kind) {
      case AnalyticCut::Kind::KeepWhole: out.keepFaces.push_back(A.faces[idx].face); break;
      case AnalyticCut::Kind::Drop: break;
      case AnalyticCut::Kind::Split:
        out.keepFaces.push_back(ac.keepFace);
        out.capPieces.push_back(Piece{ac.cross0, ac.cross1});
        break;
      case AnalyticCut::Kind::Fail:
        out.why = ac.why == HalfSpaceCutDecline::UnsupportedEdgeKind
                      ? SeamDecline::UnsupportedEdgeKind
                      : SeamDecline::AnalyticCrossingNot2;
        return out;
    }
    // The complementary (inner-side) survivor of the same face — for COMMON.
    const AnalyticCut ic = cutAnalyticFace(A.faces[idx], P, other, band, weldTol);
    if (ic.kind == AnalyticCut::Kind::KeepWhole) out.dropFaces.push_back(A.faces[idx].face);
    else if (ic.kind == AnalyticCut::Kind::Split) out.dropFaces.push_back(ic.keepFace);
  }
  return out;
}

}  // namespace isdetail

// ─────────────────────────────────────────────────────────────────────────────
// buildInterSolidSeam — assemble the closed inter-solid seam and `A`'s survivor
// faces for the single-curved-cut box pose, or return a measured `SeamDecline`.
// `A` is the recognised freeform operand; `B` the finite all-planar solid.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<InterSolidSeam> buildInterSolidSeam(const FreeformOperand& A,
                                                         const topo::Shape& B,
                                                         SeamDecline* why = nullptr) {
  auto fail = [&](SeamDecline d) -> std::optional<InterSolidSeam> {
    if (why) *why = d;
    return std::nullopt;
  };
  if (!isAllPlanar(B)) return fail(SeamDecline::NotPlanarB);
  if (A.freeform.size() != 1) return fail(SeamDecline::NotSingleCurvedCut);

  InterSolidSeam seam;
  seam.bPolys = extractPolygons(B);
  if (seam.bPolys.empty()) return fail(SeamDecline::NotPlanarB);

  const double diag = std::max(A.bbox.diagonal(), 1e-9);
  const double band = 1e-9 * diag;
  const double weldTol = 1e-7 * std::max(diag, 1.0);

  const OperandFace& wall = A.faces[A.freeform.front()];
  const std::vector<math::Point3> poles = isdetail::wallWorldPoles(wall);
  if (poles.empty()) return fail(SeamDecline::SeamUnusable);

  const SeamDecline located = isdetail::findCuttingFace(seam.bPolys, poles, A.bbox, band, seam.pcutIdx);
  if (located != SeamDecline::Ok) return fail(located);

  seam.tracePlane = isdetail::tracePlaneOf(seam.bPolys[seam.pcutIdx]);
  seam.keepSide = KeepSide::Below;

  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) return fail(SeamDecline::SeamUnusable);
  const isdetail::SplitCollect sc = isdetail::splitOperandA(
      A, wall, *srf->surface, srf->location, seam.tracePlane, seam.keepSide, band, weldTol);
  if (sc.why != SeamDecline::Ok) return fail(sc.why);

  // Chain the seam pieces into ONE closed loop — the shared inter-solid seam.
  if (!hscdetail::orderLoop(sc.capPieces, weldTol, seam.capLoop))
    return fail(SeamDecline::SectionLoopOpen);

  seam.aKeepFaces = sc.keepFaces;
  seam.aDropFaces = sc.dropFaces;
  if (why) *why = SeamDecline::Ok;
  return seam;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_INTER_SOLID_SEAM_H
