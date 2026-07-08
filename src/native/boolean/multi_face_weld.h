// SPDX-License-Identifier: Apache-2.0
//
// multi_face_weld.h — MOAT M2-multiseam: the MULTI-FACE corner-clip WELD verb, the
// one new assembler the seam-GRAPH boolean needs to turn the landed junction-aware
// wall split (`junction_split.h`) into a watertight two-operand result solid.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// The corner box `B` straddles the corner of `A`'s footprint quad `Q`, so the two
// cutting planes (`x = 0`, `y = 0`) clip MORE than `A`'s Bézier wall: they also clip
// `A`'s flat BOTTOM quad and the TWO side walls whose `Q` edges cross the planes, and
// the result needs the box CAP faces synthesised INSIDE `A` and the whole shell welded
// across MULTIPLE junctions (the wall junction `J`, the bottom junction `J'`, and the
// wall/plane pierce points). This header is exactly that multi-FACE assembly, for the
// three ops:
//
//   * CUT  (`A − B`, the L-solid): `A`'s faces clipped to the KEEP region
//     `{x≤0 ∨ y≤0}` — the bowl L-survivor (`junction_split.faceSurvivor`), the bottom
//     quad L-survivor (a planar corner-clip, reroutes the boundary around the removed
//     quadrant through the corner vertex `J'`), the two whole side walls, the two side
//     walls clipped by ONE plane (byte-frozen `hscdetail::cutAnalyticFace`), PLUS the
//     two synthesised box CAP faces (on `x=0` / `y=0`, bounded inside `A`, sharing the
//     bowl seam arc with the wall survivor and the vertical `J→J'` edge with each other).
//
//   * COMMON (`A ∩ B`, the corner piece): the complementary keep region `{x≥0 ∧ y≥0}` —
//     the bowl corner sub-face, the bottom corner (a convex two-plane clip), the two
//     clipped side walls (corner side), and the two caps (opposite outward normal).
//
//   * FUSE (`A ∪ B`): `A`'s CUT faces (the caps are now INTERIOR, dropped) welded to
//     `B`'s shell — `B`'s four non-cutting faces WHOLE, and `B`'s two cutting faces
//     NOTCHED by the cap region (a rectangle-minus-notch whose curved boundary IS the
//     shared bowl seam arc). The notch attaches to the box face along the shared
//     vertical `J→J'` edge, so the two notched box faces and the wall/bottom survivors
//     weld watertight along the same bit-shared 3-D geometry.
//
// ── SELF-VERIFY → OCCT FALLBACK (load-bearing) ────────────────────────────────────
// The welded result is admitted ONLY if the M0 mesh is WATERTIGHT (every edge shared by
// exactly two triangles) AND its enclosed volume lies in the op's consistent bound
// (`0 ≤ V(A−B) ≤ V(A)`; `0 ≤ V(A∩B) ≤ min(V(A),V(B))`; `max(V(A),V(B)) ≤ V(A∪B) ≤
// V(A)+V(B)`). Any failure returns a NULL `Shape` → OCCT. No leaky/overlapping/wrong-
// volume solid is EVER emitted; no tolerance is weakened. (The TIGHT closed-form corner
// oracle — `V(A∩B)=0.051275`, `V(A−B)=0.145035`, `V(A∪B)=0.529035` and the partition
// identities — lives in the host gate; the sim gate compares native vs `BRepAlgoAPI_*`.)
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// `buildSeamGraph` (`seam_graph.h`), `splitFaceJunction` (`junction_split.h`), every
// `hscdetail::` primitive of the landed `half_space_cut.h` (`Piece`, `cutAnalyticFace`,
// `orderLoop`, `planarFaceFromLoop`, `edgeFromPiece`, `signedDist`), `assemble.h`
// `VertexPool` + `triangulatePolygonToFaces`, and the M0 `SolidMesher`/`isWatertight`/
// `enclosedVolume`. Additive sibling — edits none of them.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_MULTI_FACE_WELD_H
#define CYBERCAD_NATIVE_BOOLEAN_MULTI_FACE_WELD_H

#include "native/boolean/assemble.h"
#include "native/boolean/half_space_cut.h"
#include "native/boolean/junction_split.h"
#include "native/boolean/polygon.h"
#include "native/boolean/seam_graph.h"
#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;

/// The requested op for the multi-face weld (mirrors `MultiSeamOp`).
enum class MultiFaceOp { Fuse, Cut, Common };

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight multi-face corner-clip result solid is returned.
enum class MultiFaceDecline {
  Ok,
  NoStraddlingBottom,   ///< no analytic face straddles BOTH cutting planes (pose guard)
  CornerNotInBottom,    ///< the two-plane corner vertex is not inside the straddling face
  LoopOpen,             ///< a synthesised face's boundary pieces do not chain closed
  WallClipFailed,       ///< a single-plane side-wall clip declined (unexpected for a valid pose)
  WeldOpen,             ///< fewer than four survivor faces (cannot bound a solid)
  NotWatertight,        ///< self-verify: the welded result is not a closed 2-manifold
  VolumeInconsistent    ///< self-verify: the volume is outside the op's consistent bound
};

inline const char* multiFaceDeclineName(MultiFaceDecline d) noexcept {
  switch (d) {
    case MultiFaceDecline::Ok: return "Ok";
    case MultiFaceDecline::NoStraddlingBottom: return "NoStraddlingBottom";
    case MultiFaceDecline::CornerNotInBottom: return "CornerNotInBottom";
    case MultiFaceDecline::LoopOpen: return "LoopOpen";
    case MultiFaceDecline::WallClipFailed: return "WallClipFailed";
    case MultiFaceDecline::WeldOpen: return "WeldOpen";
    case MultiFaceDecline::NotWatertight: return "NotWatertight";
    case MultiFaceDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

/// A measured record of a multi-face weld attempt.
struct MultiFaceReport {
  MultiFaceDecline decline = MultiFaceDecline::Ok;
  int faceCount = 0;
  bool watertight = false;
  double volume = 0.0;   ///< enclosed volume of the welded result (self-verify measure)
  double volA = 0.0;     ///< V(A) from the operand mesh (bound reference)
  double volB = 0.0;     ///< V(B) from the box mesh (bound reference)
};

namespace mfwdetail {

using hscdetail::Piece;
using hscdetail::signedDist;

/// Per-analytic-face classification against the two cutting planes `Px`, `Py`.
/// `loop3d` is the face's outer-loop world polyline (straight-edge flatten); `removed`
/// counts vertices in the removed quadrant `{sx>band ∧ sy>band}`.
struct FaceClass {
  std::vector<math::Point3> loop3d;
  int removed = 0;
  bool straddleX = false, straddleY = false;
};

inline FaceClass classifyAnalytic(const OperandFace& F, const math::Plane& Px,
                                  const math::Plane& Py, double band) {
  FaceClass c;
  const tess::UVRegion reg = tess::buildRegion(F.face, 1);
  const auto sr = topo::surfaceOf(F.face);
  if (!sr || !sr->surface || !reg.hasOuter()) return c;
  tess::SurfaceEvaluator ev(*sr->surface, sr->location);
  bool sXp = false, sXn = false, sYp = false, sYn = false;
  for (const tess::UV& q : reg.outer) {
    const math::Point3 p = ev.value(q.u, q.v);
    c.loop3d.push_back(p);
    const double a = signedDist(Px, p), b = signedDist(Py, p);
    if (a > band) sXp = true;
    if (a < -band) sXn = true;
    if (b > band) sYp = true;
    if (b < -band) sYn = true;
    if (a > band && b > band) ++c.removed;
  }
  c.straddleX = sXp && sXn;
  c.straddleY = sYp && sYn;
  return c;
}

/// Interpolated crossing of edge `a→b` with the nearer of the two removed-quadrant
/// boundary planes (`sx=0` or `sy=0`) it transitions across.
inline math::Point3 quadrantCross(const math::Plane& Px, const math::Plane& Py, double band,
                                  const math::Point3& a, const math::Point3& b) {
  const double ax = signedDist(Px, a), bx = signedDist(Px, b);
  const double ay = signedDist(Py, a), by = signedDist(Py, b);
  double tx = 2.0, ty = 2.0;
  if ((ax > band) != (bx > band)) tx = ax / (ax - bx);
  if ((ay > band) != (by > band)) ty = ay / (ay - by);
  const double t = std::min(tx, ty);
  return math::Point3{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
}

/// CUT/FUSE bottom L-survivor: walk the (convex) bottom loop, keeping vertices OUTSIDE
/// the removed quadrant and rerouting through the corner `J'` around the removed span.
inline std::vector<math::Point3> rerouteSurvivor(const std::vector<math::Point3>& loop,
                                                 const math::Plane& Px, const math::Plane& Py,
                                                 double band, const math::Point3& corner) {
  const int n = static_cast<int>(loop.size());
  auto isRem = [&](const math::Point3& p) {
    return signedDist(Px, p) > band && signedDist(Py, p) > band;
  };
  int start = 0;
  for (int i = 0; i < n; ++i)
    if (!isRem(loop[i])) { start = i; break; }
  std::vector<math::Point3> out;
  bool insertedCorner = false;
  for (int k = 0; k < n; ++k) {
    const math::Point3& a = loop[(start + k) % n];
    const math::Point3& b = loop[(start + k + 1) % n];
    const bool ra = isRem(a), rb = isRem(b);
    if (!ra) out.push_back(a);
    if (ra != rb) {
      out.push_back(quadrantCross(Px, Py, band, a, b));
      if (!ra && rb && !insertedCorner) { out.push_back(corner); insertedCorner = true; }
    }
  }
  return out;
}

/// COMMON bottom corner: Sutherland–Hodgman clip of the bottom loop to `{sx≥0}` then
/// `{sy≥0}` — the convex removed quadrant.
inline std::vector<math::Point3> clipCorner(const std::vector<math::Point3>& loop,
                                            const math::Plane& Px, const math::Plane& Py,
                                            double band) {
  auto clip = [&](const std::vector<math::Point3>& in, const math::Plane& P) {
    std::vector<math::Point3> out;
    const int n = static_cast<int>(in.size());
    for (int i = 0; i < n; ++i) {
      const math::Point3& a = in[i];
      const math::Point3& b = in[(i + 1) % n];
      const double da = signedDist(P, a), db = signedDist(P, b);
      const bool ai = da >= -band, bi = db >= -band;
      if (ai) out.push_back(a);
      if (ai != bi) {
        const double t = da / (da - db);
        out.push_back(math::Point3{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y),
                                   a.z + t * (b.z - a.z)});
      }
    }
    return out;
  };
  return clip(clip(loop, Px), Py);
}

/// Build ONE planar face from a vertex ring (consecutive straight pieces), oriented so
/// its normal matches `outward`. Returns a null Shape on an open ring.
inline topo::Shape ringFace(const std::vector<math::Point3>& ring, const math::Ax3& frame,
                            const math::Vec3& outward, double weldTol) {
  if (ring.size() < 3) return {};
  std::vector<Piece> pcs;
  const int n = static_cast<int>(ring.size());
  for (int i = 0; i < n; ++i) pcs.push_back(Piece{ring[i], ring[(i + 1) % n]});
  std::vector<Piece> loop;
  if (!hscdetail::orderLoop(pcs, weldTol, loop)) return {};
  return hscdetail::planarFaceFromLoop(loop, frame, outward);
}

/// A synthesised box CAP face (on cutting plane frame `frame`, outward `outward`): the
/// bowl seam `arc` (E…J) + the vertical `J→J'` + the bottom `J'→E'` + the vertical
/// `E'→E`. Shares `arc` with the wall sub-face and `J→J'` with the other cap.
inline topo::Shape capFace(const Piece& arc, const math::Point3& E, const math::Point3& Eb,
                           const math::Point3& J, const math::Point3& Jb, const math::Ax3& frame,
                           const math::Vec3& outward, double weldTol) {
  std::vector<Piece> pcs = {arc, Piece{J, Jb}, Piece{Jb, Eb}, Piece{Eb, E}};
  std::vector<Piece> loop;
  if (!hscdetail::orderLoop(pcs, weldTol, loop)) return {};
  return hscdetail::planarFaceFromLoop(loop, frame, outward);
}

/// Is `p` on the interior/endpoints of segment `a→b` (within `tol`)?
inline bool onSegment(const math::Point3& p, const math::Point3& a, const math::Point3& b,
                      double tol) {
  const math::Vec3 ab = b - a;
  const double len2 = math::normSquared(ab);
  if (len2 < tol * tol) return false;
  const double t = math::dot(p - a, ab) / len2;
  if (t < -1e-6 || t > 1.0 + 1e-6) return false;
  const math::Point3 proj{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
  return math::distance(proj, p) <= 100.0 * tol;
}

/// The box face polygon's boundary pieces, with the edge carrying the shared `J↔Jb`
/// segment split at `J` and `Jb` and its `[J,Jb]` middle DROPPED (the notch opening).
inline std::vector<Piece> boxRingMinusAttach(const Polygon& poly, const math::Point3& J,
                                             const math::Point3& Jb, double weldTol) {
  std::vector<Piece> pcs;
  const std::vector<math::Point3>& v = poly.vertices;
  const int n = static_cast<int>(v.size());
  for (int i = 0; i < n; ++i) {
    const math::Point3& a = v[i];
    const math::Point3& b = v[(i + 1) % n];
    if (!onSegment(J, a, b, weldTol) || !onSegment(Jb, a, b, weldTol)) {
      pcs.push_back(Piece{a, b});
      continue;
    }
    const double tJ = math::dot(J - a, b - a), tJb = math::dot(Jb - a, b - a);
    const math::Point3& near = tJ <= tJb ? J : Jb;
    const math::Point3& far = tJ <= tJb ? Jb : J;
    pcs.push_back(Piece{a, near});
    pcs.push_back(Piece{far, b});
  }
  return pcs;
}

/// A NOTCHED box cutting face (FUSE): the box face polygon with the cap region removed.
/// The notch attaches along the shared `J↔Jb` segment on the box face boundary, so the
/// remaining boundary is: box edges (the `J↔Jb` sub-segment dropped) + the cap chain
/// `Jb→Eb→E→arc(E…J)` (i.e. the cap loop minus the `J→Jb` piece). Curved side = `arc`.
inline topo::Shape notchedBoxFace(const Polygon& poly, const Piece& arc, const math::Point3& E,
                                  const math::Point3& Eb, const math::Point3& J,
                                  const math::Point3& Jb, double weldTol) {
  std::vector<Piece> pcs = boxRingMinusAttach(poly, J, Jb, weldTol);
  pcs.push_back(Piece{Jb, Eb});
  pcs.push_back(Piece{Eb, E});
  pcs.push_back(arc);
  std::vector<Piece> loop;
  if (!hscdetail::orderLoop(pcs, weldTol, loop)) return {};
  const math::Dir3 nz{poly.plane.normal};
  const math::Vec3 ref = std::fabs(nz.z()) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  const math::Ax3 frame = math::Ax3::fromAxisAndRef(poly.vertices.front(), nz, math::Dir3{ref});
  return hscdetail::planarFaceFromLoop(loop, frame, poly.plane.normal);
}

/// `A`'s analytic faces clipped to the op's keep region: whole (kept for CUT/FUSE,
/// dropped for COMMON), single-plane clip (byte-frozen `cutAnalyticFace`), or the
/// two-plane bottom corner-clip. Appends into `faces`; reports whether the bottom was seen.
struct AnalyticClip {
  bool sawBottom = false;
  MultiFaceDecline decline = MultiFaceDecline::Ok;
};

inline AnalyticClip clipAnalyticFaces(const FreeformOperand& A, const math::Plane& Px,
                                      const math::Plane& Py, double band, double weldTol,
                                      bool common, const math::Point3& Jb,
                                      std::vector<topo::Shape>& faces) {
  AnalyticClip out;
  for (std::size_t idx : A.analytic) {
    const OperandFace& F = A.faces[idx];
    const FaceClass c = classifyAnalytic(F, Px, Py, band);
    if (c.loop3d.size() < 3) { out.decline = MultiFaceDecline::WeldOpen; return out; }
    if (c.removed == 0) {
      if (!common) faces.push_back(F.face);  // whole wall (CUT/FUSE keep; COMMON drops)
      continue;
    }
    if (c.straddleX && c.straddleY) {  // the bottom quad: a two-plane corner-clip
      out.sawBottom = true;
      const std::vector<math::Point3> ring = common ? clipCorner(c.loop3d, Px, Py, band)
                                                     : rerouteSurvivor(c.loop3d, Px, Py, band, Jb);
      const topo::Shape f = ringFace(ring, F.surface.frame, F.outwardN, weldTol);
      if (f.isNull()) { out.decline = MultiFaceDecline::LoopOpen; return out; }
      faces.push_back(f);
      continue;
    }
    // single-plane side-wall clip: COMMON keeps the corner (Above) side, CUT/FUSE the away.
    const math::Plane& P = c.straddleX ? Px : Py;
    const KeepSide side = common ? KeepSide::Above : KeepSide::Below;
    const hscdetail::AnalyticCut ac = hscdetail::cutAnalyticFace(F, P, side, band, weldTol);
    if (ac.kind == hscdetail::AnalyticCut::Kind::Split) faces.push_back(ac.keepFace);
    else if (ac.kind == hscdetail::AnalyticCut::Kind::KeepWhole) faces.push_back(F.face);
    else if (ac.kind == hscdetail::AnalyticCut::Kind::Drop) { /* corner side empty */ }
    else { out.decline = MultiFaceDecline::WallClipFailed; return out; }
  }
  return out;
}

/// Is the welded volume `v` inside the op's consistent inclusion–exclusion bound?
inline bool volumeConsistent(MultiFaceOp op, double v, double vA, double vB, double band,
                             double tol) {
  switch (op) {
    case MultiFaceOp::Cut:    return v > band && v <= vA + tol;
    case MultiFaceOp::Common: return v > band && v <= std::min(vA, vB) + tol;
    case MultiFaceOp::Fuse:   return v >= std::max(vA, vB) - tol && v <= vA + vB + tol;
  }
  return false;
}

/// Mesh a bag of polygons into a watertight box solid mesh (for the volume bound).
inline tess::Mesh polysMesh(const std::vector<Polygon>& polys, double deflection) {
  VertexPool pool;
  std::vector<topo::Shape> faces;
  for (const Polygon& p : polys) detail::triangulatePolygonToFaces(p, pool, faces);
  const topo::Shape solid =
      topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
  tess::MeshParams mp;
  mp.deflection = deflection;
  return tess::SolidMesher(mp).mesh(solid);
}

/// The seam geometry the result shell shares: the two orthogonal bowl arc halves (E…J on
/// x=0, J…X on y=0) reconstructed exactly as `junction_split` builds them, plus the
/// corner points on the wall (E,X,J) and their bottom-plane projections (Eb,Xb,Jb).
struct SeamGeom {
  Piece arcA, arcB;
  math::Point3 E, X, J, Eb, Xb, Jb;
  bool ok = false;
};

inline SeamGeom buildSeamGeom(const FreeformOperand& A, const SeamGraph& g,
                              const JunctionFaceSplit& js) {
  SeamGeom sg;
  const OperandFace& wall = A.faces[A.freeform.front()];
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) return sg;
  tess::SurfaceEvaluator ev(*srf->surface, srf->location);
  std::vector<math::Point3> seam3d;
  seam3d.reserve(js.seam.size());
  for (const UV& q : js.seam) seam3d.push_back(ev.value(q.u, q.v));
  seam3d[js.jIdx] = g.junction3d;
  sg.arcA = Piece(seam3d.begin(), seam3d.begin() + (js.jIdx + 1));  // E…J (x=0 cut)
  sg.arcB = Piece(seam3d.begin() + js.jIdx, seam3d.end());          // J…X (y=0 cut)
  sg.E = seam3d.front();
  sg.X = seam3d.back();
  sg.J = g.junction3d;
  const double zBot = A.bbox.lo.z;
  sg.Eb = math::Point3{sg.E.x, sg.E.y, zBot};
  sg.Xb = math::Point3{sg.X.x, sg.X.y, zBot};
  sg.Jb = math::Point3{sg.J.x, sg.J.y, zBot};
  sg.ok = true;
  return sg;
}

/// Append the op's remaining faces: the two synthesised box CAP faces inside `A`
/// (CUT/COMMON) or `B`'s shell — four non-cutting faces WHOLE + two NOTCHED cutting
/// faces (FUSE). Returns false (→ LoopOpen) if a synthesised face does not close.
inline bool appendResultShell(MultiFaceOp op, const SeamGraph& g, const SeamGeom& sg,
                              const math::Plane& Px, const math::Plane& Py, double weldTol,
                              std::vector<topo::Shape>& faces) {
  if (op == MultiFaceOp::Fuse) {
    VertexPool pool;
    for (std::size_t i = 0; i < g.bPolys.size(); ++i)
      if (i != g.cutIdx[0] && i != g.cutIdx[1])
        detail::triangulatePolygonToFaces(g.bPolys[i], pool, faces);
    const topo::Shape nx = notchedBoxFace(g.bPolys[g.cutIdx[0]], sg.arcA, sg.E, sg.Eb, sg.J, sg.Jb, weldTol);
    const topo::Shape ny = notchedBoxFace(g.bPolys[g.cutIdx[1]], sg.arcB, sg.X, sg.Xb, sg.J, sg.Jb, weldTol);
    if (nx.isNull() || ny.isNull()) return false;
    faces.push_back(nx);
    faces.push_back(ny);
    return true;
  }
  // caps outward: toward the removed quadrant for CUT, away from it for COMMON.
  const double o = (op == MultiFaceOp::Common) ? -1.0 : 1.0;
  const topo::Shape cx = capFace(sg.arcA, sg.E, sg.Eb, sg.J, sg.Jb, Px.pos, math::Vec3{o, 0, 0}, weldTol);
  const topo::Shape cy = capFace(sg.arcB, sg.X, sg.Xb, sg.J, sg.Jb, Py.pos, math::Vec3{0, o, 0}, weldTol);
  if (cx.isNull() || cy.isNull()) return false;
  faces.push_back(cx);
  faces.push_back(cy);
  return true;
}

/// Mandatory self-verify: mesh the welded result, require watertight, and require the
/// enclosed volume in the op's consistent bound (measured from the A and B meshes).
inline MultiFaceDecline selfVerify(const topo::Shape& result, const FreeformOperand& A,
                                   const SeamGraph& g, MultiFaceOp op, double deflection,
                                   double band, MultiFaceReport& rep) {
  tess::MeshParams mp;
  mp.deflection = deflection;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(result);
  rep.watertight = tess::isWatertight(m);
  if (!rep.watertight) return MultiFaceDecline::NotWatertight;
  rep.volume = std::fabs(tess::enclosedVolume(m));
  rep.volA = std::fabs(tess::enclosedVolume(tess::SolidMesher(mp).mesh(A.solid)));
  rep.volB = std::fabs(tess::enclosedVolume(polysMesh(g.bPolys, deflection)));
  const double tol = 0.05 * std::max(rep.volA + rep.volB, 1e-12);
  return volumeConsistent(op, rep.volume, rep.volA, rep.volB, band, tol)
             ? MultiFaceDecline::Ok
             : MultiFaceDecline::VolumeInconsistent;
}

}  // namespace mfwdetail

// ─────────────────────────────────────────────────────────────────────────────
// multiFaceCornerClip — assemble + self-verify the CUT / COMMON / FUSE multi-face
// corner-clip solid from the recognised operand `A`, its two-arc seam graph `g`, and
// the landed junction-aware wall split `js`. Returns the welded solid, or a NULL
// `Shape` with a measured `MultiFaceDecline`. NEVER emits a leaky/wrong-volume solid.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape multiFaceCornerClip(const FreeformOperand& A, const SeamGraph& g,
                                       const JunctionFaceSplit& js, MultiFaceOp op,
                                       double deflection = 0.01, MultiFaceReport* report = nullptr) {
  using namespace mfwdetail;
  MultiFaceReport rep;
  auto fail = [&](MultiFaceDecline d) -> topo::Shape {
    rep.decline = d;
    if (report) *report = rep;
    return {};
  };

  const double diag = std::max(A.bbox.diagonal(), 1e-9);
  const double band = 1e-9 * diag;
  const double weldTol = 1e-7 * std::max(diag, 1.0);
  const math::Plane Px = g.arcs[0].tracePlane;  // z = into B (+x on this pose)
  const math::Plane Py = g.arcs[1].tracePlane;  // z = into B (+y on this pose)

  const SeamGeom sg = buildSeamGeom(A, g, js);  // shared arc halves + corner points
  if (!sg.ok) return fail(MultiFaceDecline::WeldOpen);

  const bool common = (op == MultiFaceOp::Common);
  std::vector<topo::Shape> faces;
  faces.push_back(common ? js.faceCorner : js.faceSurvivor);  // bowl sub-face

  // `A`'s analytic faces, clipped to the op's keep region (whole / single-plane / bottom).
  const AnalyticClip ac = clipAnalyticFaces(A, Px, Py, band, weldTol, common, sg.Jb, faces);
  if (ac.decline != MultiFaceDecline::Ok) return fail(ac.decline);
  if (!ac.sawBottom) return fail(MultiFaceDecline::NoStraddlingBottom);

  // The op's remaining faces: the two box CAP faces (CUT/COMMON) or `B`'s shell (FUSE).
  if (!appendResultShell(op, g, sg, Px, Py, weldTol, faces)) return fail(MultiFaceDecline::LoopOpen);

  rep.faceCount = static_cast<int>(faces.size());
  if (faces.size() < 4) return fail(MultiFaceDecline::WeldOpen);
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  const topo::Shape result = topo::ShapeBuilder::makeSolid({shell});

  const MultiFaceDecline v = selfVerify(result, A, g, op, deflection, band, rep);
  if (v != MultiFaceDecline::Ok) return fail(v);

  rep.decline = MultiFaceDecline::Ok;
  if (report) *report = rep;
  return result;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_MULTI_FACE_WELD_H
