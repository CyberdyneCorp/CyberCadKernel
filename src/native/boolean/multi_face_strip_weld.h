// SPDX-License-Identifier: Apache-2.0
//
// multi_face_strip_weld.h — MOAT M2 blocker #4 (≥3-seam weld): the MULTI-FACE STRIP
// WELD verb, the additive generalisation of the landed corner-clip weld
// (`multi_face_weld.h`) from a removed QUADRANT + two caps to a removed STRIP + THREE
// caps, welding the two-junction chain seam-graph result watertight.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// The edge-straddling box `B` (`x ∈ [x0,x1]`, `y ≥ y0`) removes the STRIP
// `A ∩ {x0 ≤ x ≤ x1, y ≥ y0}`. Its three cutting planes (`Px0: x=x0`, `Px1: x=x1`,
// `Pm: y=y0`) clip MORE than `A`'s Bézier wall: they also clip `A`'s flat BOTTOM quad
// (a three-plane rectangular notch) and any side wall the strip crosses (the "back" wall
// is clipped by BOTH parallel planes — a MID-NOTCH that leaves two end pieces, the new
// sub-case beyond the corner clip's single-plane wall clips). The result needs THREE box
// CAP faces (on `Px0`, `Px1`, `Pm`) synthesised inside `A` and the whole shell welded
// across the two wall junctions `J1`, `J2` and their bottom projections `J1b`, `J2b`.
//
//   * CUT  (`A − B`): `A`'s faces clipped to the KEEP region `{x<x0 ∨ x>x1 ∨ y<y0}` —
//     the strip-survivor wall sub-face (`splitFaceStrip.faceSurvivor`), each analytic
//     face clipped to the keep region as a set of CONVEX pieces (the complement of a
//     rectangular strip = up to three disjoint convex clips `{x≤x0}`, `{x≥x1}`,
//     `{x0≤x≤x1 ∧ y≤y0}`), PLUS the three synthesised box CAP faces.
//   * COMMON (`A ∩ B`): the complementary keep region `{x0≤x≤x1 ∧ y≥y0}` — the strip
//     wall sub-face, each analytic face clipped to that convex strip, and the three caps
//     (opposite outward normal).
//   * FUSE (`A ∪ B`): `A`'s CUT faces (caps now interior, dropped) welded to `B`'s shell:
//     `B`'s three non-cutting faces WHOLE + its three cutting faces NOTCHED by the cap
//     regions.  (This slice lands CUT/COMMON via caps; FUSE via inclusion–exclusion on
//     the self-verified CUT solid + `B`, which keeps the weld in the robust regime.)
//
// ── SELF-VERIFY → OCCT FALLBACK (load-bearing) ────────────────────────────────────
// Admitted ONLY if the M0 mesh is WATERTIGHT and its enclosed volume lies in the op's
// consistent bound. Any failure → NULL `Shape` → OCCT. No leaky/overlapping/wrong-volume
// solid is EVER emitted; no tolerance is weakened. (The closed-form STRIP oracle —
// `chain_seam_fixture::vol{Common,Cut,Union}` — lives in the host gate; the sim gate
// compares native vs `BRepAlgoAPI_*`.)
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// `buildChainSeamGraph` (`seam_graph_chain.h`), `splitFaceStrip` (`strip_split.h`), every
// `hscdetail::` primitive of `half_space_cut.h` (`Piece`, `orderLoop`, `planarFaceFromLoop`,
// `edgeFromPiece`, `signedDist`), `assemble.h` `VertexPool`+`triangulatePolygonToFaces`, and
// the M0 `SolidMesher`/`isWatertight`/`enclosedVolume`. Additive sibling — edits none.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_MULTI_FACE_STRIP_WELD_H
#define CYBERCAD_NATIVE_BOOLEAN_MULTI_FACE_STRIP_WELD_H

#include "native/boolean/assemble.h"
#include "native/boolean/half_space_cut.h"
#include "native/boolean/multi_face_weld.h"
#include "native/boolean/polygon.h"
#include "native/boolean/seam_graph_chain.h"
#include "native/boolean/strip_split.h"
#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;

/// The requested op for the strip weld (mirrors `MultiFaceOp`).
enum class StripWeldOp { Fuse, Cut, Common };

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight strip-clip result solid is returned.
enum class StripWeldDecline {
  Ok,
  NoStraddlingBottom,   ///< no analytic face straddles ALL three planes (pose guard)
  LoopOpen,             ///< a synthesised face's boundary pieces do not chain closed
  WeldOpen,             ///< fewer than five survivor faces (cannot bound the strip solid)
  NotWatertight,        ///< self-verify: the welded result is not a closed 2-manifold
  VolumeInconsistent    ///< self-verify: the volume is outside the op's consistent bound
};

inline const char* stripWeldDeclineName(StripWeldDecline d) noexcept {
  switch (d) {
    case StripWeldDecline::Ok: return "Ok";
    case StripWeldDecline::NoStraddlingBottom: return "NoStraddlingBottom";
    case StripWeldDecline::LoopOpen: return "LoopOpen";
    case StripWeldDecline::WeldOpen: return "WeldOpen";
    case StripWeldDecline::NotWatertight: return "NotWatertight";
    case StripWeldDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

/// A measured record of a strip weld attempt.
struct StripWeldReport {
  StripWeldDecline decline = StripWeldDecline::Ok;
  int faceCount = 0;
  bool watertight = false;
  double volume = 0.0;   ///< enclosed volume of the welded result (self-verify measure)
  double volA = 0.0;     ///< V(A) from the operand mesh (bound reference)
  double volB = 0.0;     ///< V(B) from the box mesh (bound reference)
};

namespace mfswdetail {

using hscdetail::Piece;
using hscdetail::signedDist;

/// Sutherland–Hodgman clip of a world-space polygon `loop` to the half-space
/// `signedDist(P, ·) ≥ -band` (Above) or `≤ band` (Below). Returns the clipped ring.
inline std::vector<math::Point3> clipHalf(const std::vector<math::Point3>& loop,
                                          const math::Plane& P, KeepSide side, double band) {
  std::vector<math::Point3> out;
  const int n = static_cast<int>(loop.size());
  auto keep = [&](double d) { return side == KeepSide::Above ? d >= -band : d <= band; };
  for (int i = 0; i < n; ++i) {
    const math::Point3& a = loop[i];
    const math::Point3& b = loop[(i + 1) % n];
    const double da = signedDist(P, a), db = signedDist(P, b);
    const bool ai = keep(da), bi = keep(db);
    if (ai) out.push_back(a);
    if (ai != bi) {
      const double t = da / (da - db);
      out.push_back(math::Point3{a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)});
    }
  }
  return out;
}

/// Build ONE planar face from a world-space vertex ring, oriented so its normal matches
/// `outward`. Returns a null Shape on an open/degenerate ring (< 3 verts).
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

/// The three cutting planes of the strip pose, plus the strip's world x/y bounds.
struct StripPlanes {
  math::Plane Px0, Px1, Pm;   ///< x=x0, x=x1, y=y0 (all normals point INTO the strip)
  double x0 = 0, x1 = 0, y0 = 0;
};

/// Wrap a rebuilt planar sub-face as an `OperandFace` so it can be re-cut by a second
/// plane (composing two `cutAnalyticFace` calls for a curved-topped wall the strip cuts
/// with BOTH parallel x-planes). Keeps the same surface frame + outward normal.
inline OperandFace asOperandFace(const topo::Shape& face, const OperandFace& like) {
  OperandFace of = like;
  of.face = face;
  return of;
}

/// Cut a curved-topped analytic wall by BOTH x-planes, keeping the region between them
/// (`Px0 Above ∧ Px1 Above`, i.e. inside the strip's x-slab). Uses the byte-frozen
/// `cutAnalyticFace` (exact curve-crossing on the shared Bézier top edge — the SAME
/// crossing the seam uses) composed twice. Returns the kept sub-face or a null Shape.
inline topo::Shape cutWallBetweenX(const OperandFace& F, const StripPlanes& sp, double band,
                                   double weldTol) {
  const hscdetail::AnalyticCut c0 = hscdetail::cutAnalyticFace(F, sp.Px0, KeepSide::Above, band, weldTol);
  topo::Shape keep;
  if (c0.kind == hscdetail::AnalyticCut::Kind::KeepWhole) keep = F.face;
  else if (c0.kind == hscdetail::AnalyticCut::Kind::Split) keep = c0.keepFace;
  else return {};  // Drop/Fail → empty
  if (keep.isNull()) return {};
  const hscdetail::AnalyticCut c1 =
      hscdetail::cutAnalyticFace(asOperandFace(keep, F), sp.Px1, KeepSide::Above, band, weldTol);
  if (c1.kind == hscdetail::AnalyticCut::Kind::KeepWhole) return keep;
  if (c1.kind == hscdetail::AnalyticCut::Kind::Split) return c1.keepFace;
  return {};
}

/// The strip's shared seam geometry: the three arc segments (E…J1 on Px0, J1…J2 on Pm,
/// J2…X on Px1) reconstructed exactly as `splitFaceStrip` builds them, plus the boundary
/// points on the wall (E, J1, J2, X) and their bottom-plane projections.
struct StripSeamGeom {
  Piece arc0, arcM, arc1;   ///< E→J1 (Px0), J1→J2 (Pm), J2→X (Px1)
  math::Point3 E, J1, J2, X, Eb, J1b, J2b, Xb;
  bool ok = false;
};

inline StripSeamGeom buildStripSeamGeom(const FreeformOperand& A, const ChainSeamGraph& g,
                                        const StripFaceSplit& ss) {
  StripSeamGeom sg;
  const OperandFace& wall = A.faces[A.freeform.front()];
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) return sg;
  tess::SurfaceEvaluator ev(*srf->surface, srf->location);
  std::vector<math::Point3> seam3d;
  seam3d.reserve(ss.seam.size());
  for (const UV& q : ss.seam) seam3d.push_back(ev.value(q.u, q.v));
  seam3d[ss.j1Idx] = g.junction3d[0];
  seam3d[ss.j2Idx] = g.junction3d[1];
  sg.arc0 = Piece(seam3d.begin(), seam3d.begin() + (ss.j1Idx + 1));               // E…J1
  sg.arcM = Piece(seam3d.begin() + ss.j1Idx, seam3d.begin() + (ss.j2Idx + 1));    // J1…J2
  sg.arc1 = Piece(seam3d.begin() + ss.j2Idx, seam3d.end());                       // J2…X
  sg.E = seam3d.front();
  sg.X = seam3d.back();
  sg.J1 = g.junction3d[0];
  sg.J2 = g.junction3d[1];
  const double zBot = A.bbox.lo.z;
  auto drop = [&](const math::Point3& p) { return math::Point3{p.x, p.y, zBot}; };
  sg.Eb = drop(sg.E); sg.Xb = drop(sg.X); sg.J1b = drop(sg.J1); sg.J2b = drop(sg.J2);
  sg.ok = true;
  return sg;
}

/// Is `p` inside the removed strip? All three trace-plane normals point INTO the strip,
/// so a point is inside iff it is on the ≥0 side of ALL three (`signedDist ≥ -band`).
inline bool inStripRegion(const math::Point3& p, const StripPlanes& sp, double band) {
  return signedDist(sp.Px0, p) >= -band && signedDist(sp.Px1, p) >= -band &&
         signedDist(sp.Pm, p) >= -band;
}

/// CUT/FUSE survivor of a face the strip NOTCHES from an open (boundary-touching) side:
/// walk the (convex) loop keeping vertices OUTSIDE the strip and rerouting the two loop
/// crossings around the notch through the three notch-boundary points
/// `Eb → J1b → J2b → Xb` (the three cap bottom edges), so the survivor is ONE connected
/// ring whose inner edges are bit-shared with the three caps. Mirrors the corner clip's
/// `rerouteSurvivor` (one corner → a three-point detour).
inline std::vector<math::Point3> rerouteStripSurvivor(const std::vector<math::Point3>& loop,
                                                      const StripPlanes& sp, double band,
                                                      const math::Point3& Eb, const math::Point3& J1b,
                                                      const math::Point3& J2b, const math::Point3& Xb) {
  const int n = static_cast<int>(loop.size());
  auto isRem = [&](const math::Point3& p) { return inStripRegion(p, sp, band); };
  // The strip notches the bottom quad THROUGH one edge (the back edge), so no VERTEX is
  // removed — an edge segment is. Walk the loop; at the removed span (edge enters then
  // exits the strip), splice the notch detour `[near boundary crossing] → the four notch
  // corners Eb,J1b,J2b,Xb (ordered to match the near crossing) → [far crossing]`, which is
  // the three cap bottom edges. Non-removed vertices pass through unchanged.
  std::vector<math::Point3> out;
  bool detoured = false;
  for (int i = 0; i < n; ++i) {
    const math::Point3& a = loop[i];
    const math::Point3& b = loop[(i + 1) % n];
    const bool ra = isRem(a), rb = isRem(b);
    if (!ra) out.push_back(a);
    // edge a→b crosses INTO the strip (a outside, b inside) OR straddles it entirely
    // (both outside, but the mid crosses — the back-edge case): detour once.
    const bool aOut = !ra, bOut = !rb;
    if (aOut && bOut && !detoured) {
      // Does the open segment pass through the strip? It must cross BOTH x-planes AND the
      // crossing region must lie on the strip side of y (the back edge, not the front edge
      // which also spans the x-slab but at y<y0). Test the segment midpoint's membership.
      const double ax0 = signedDist(sp.Px0, a), bx0 = signedDist(sp.Px0, b);
      const double ax1 = signedDist(sp.Px1, a), bx1 = signedDist(sp.Px1, b);
      const math::Point3 mid{0.5 * (a.x + b.x), 0.5 * (a.y + b.y), 0.5 * (a.z + b.z)};
      const bool crossesSlab = ((ax0 < 0) != (bx0 < 0)) && ((ax1 < 0) != (bx1 < 0)) &&
                               signedDist(sp.Pm, mid) >= -band;
      if (crossesSlab) {
        // order the notch corners so the near end matches `a`'s side (Eb at x0, Xb at x1).
        const bool nearE = math::distance(a, Eb) <= math::distance(a, Xb);
        if (nearE) { out.push_back(Eb); out.push_back(J1b); out.push_back(J2b); out.push_back(Xb); }
        else       { out.push_back(Xb); out.push_back(J2b); out.push_back(J1b); out.push_back(Eb); }
        detoured = true;
      }
    }
  }
  return out;
}

/// Append `A`'s analytic faces, each clipped to the op's KEEP region:
///   * COMMON keep = the strip `{x0≤x≤x1 ∧ y≥y0}` (one convex Sutherland–Hodgman clip);
///   * CUT/FUSE keep = the complement. A face the strip notches from an open side (the
///     bottom quad) is rerouted into ONE connected ring around the notch; a face the
///     strip splits into two disjoint pieces (the back wall, cut by both parallel x
///     planes) is emitted as the two `{x≤x0}` / `{x≥x1}` convex clips.
/// A face untouched by the strip is kept WHOLE (CUT/FUSE) or dropped (COMMON). Reports
/// whether the BOTTOM (a face straddling all three planes) was seen (the pose guard).
struct AnalyticClip {
  bool sawBottom = false;
  StripWeldDecline decline = StripWeldDecline::Ok;
};

inline AnalyticClip clipAnalyticFaces(const FreeformOperand& A, const StripPlanes& sp,
                                      const StripSeamGeom& sg, double band, double weldTol,
                                      bool common, std::vector<topo::Shape>& faces) {
  AnalyticClip out;
  for (std::size_t idx : A.analytic) {
    const OperandFace& F = A.faces[idx];
    const tess::UVRegion reg = tess::buildRegion(F.face, 1);
    const auto sr = topo::surfaceOf(F.face);
    if (!sr || !sr->surface || !reg.hasOuter()) { out.decline = StripWeldDecline::WeldOpen; return out; }
    tess::SurfaceEvaluator ev(*sr->surface, sr->location);
    std::vector<math::Point3> loop3d;
    // All three trace-plane normals point INTO the strip (inward): a point is inside the
    // half-space of a plane iff signedDist ≥ 0. `crossP` = the face has vertices on BOTH
    // sides of plane P (it is cut by P). `inStrip` counts vertices inside all three.
    bool p0 = false, n0 = false, p1 = false, n1 = false, py = false, ny = false;
    int inStrip = 0;
    for (const tess::UV& q : reg.outer) {
      const math::Point3 p = ev.value(q.u, q.v);
      loop3d.push_back(p);
      const double d0 = signedDist(sp.Px0, p), d1 = signedDist(sp.Px1, p), dm = signedDist(sp.Pm, p);
      if (d0 > band) p0 = true; if (d0 < -band) n0 = true;
      if (d1 > band) p1 = true; if (d1 < -band) n1 = true;
      if (dm > band) py = true; if (dm < -band) ny = true;
      if (inStripRegion(p, sp, band)) ++inStrip;
    }
    const bool crossX0 = p0 && n0, crossX1 = p1 && n1, crossY = py && ny;
    // BOTTOM: cut by all three planes (the notch opens through it to the boundary). A
    // SIDE wall the strip crosses is cut by BOTH x-planes AND has material inside the strip
    // (a wall that crosses both x-planes but lies entirely on the FAR side of y — e.g. the
    // front wall at y<y0 — is untouched by the strip). `!py` means no vertex is on the
    // strip side of y, so the wall is entirely outside the strip.
    const bool onStripSideY = py;  // at least one vertex at y ≥ y0
    const bool isBottom = crossX0 && crossX1 && crossY;
    const bool sideCrossesBothX = crossX0 && crossX1 && !crossY && onStripSideY;

    if (inStrip == 0 && !isBottom && !sideCrossesBothX) {
      if (!common) faces.push_back(F.face);  // untouched: keep whole (CUT/FUSE) / drop (COMMON)
      continue;
    }
    if (isBottom) out.sawBottom = true;
    // The BOTTOM is flat (planar, straight edges) — clip it by sampling (EXACT for lines).
    // Every WALL has a CURVED (Bézier) top edge shared with the bowl, so it must be clipped
    // by the byte-frozen `cutAnalyticFace` (exact curve crossing = the SAME point the seam
    // arc uses), never by sampling — otherwise the cap side edges never weld to the wall.
    auto pushIf = [&](const topo::Shape& f) {
      if (!f.isNull()) faces.push_back(f);
    };
    if (isBottom) {
      if (common) {
        std::vector<math::Point3> r = clipHalf(loop3d, sp.Px0, KeepSide::Above, band);
        if (r.size() >= 3) r = clipHalf(r, sp.Px1, KeepSide::Above, band);
        if (r.size() >= 3) r = clipHalf(r, sp.Pm, KeepSide::Above, band);
        if (r.size() >= 3) { const topo::Shape f = ringFace(r, F.surface.frame, F.outwardN, weldTol);
          if (f.isNull()) { out.decline = StripWeldDecline::LoopOpen; return out; } faces.push_back(f); }
      } else {
        const std::vector<math::Point3> ring =
            rerouteStripSurvivor(loop3d, sp, band, sg.Eb, sg.J1b, sg.J2b, sg.Xb);
        const topo::Shape f = ringFace(ring, F.surface.frame, F.outwardN, weldTol);
        if (f.isNull()) { out.decline = StripWeldDecline::LoopOpen; return out; } faces.push_back(f);
      }
      continue;
    }
    // A WALL: use exact-curve `cutAnalyticFace` (composed for the both-x case).
    if (sideCrossesBothX) {
      if (common) {
        pushIf(cutWallBetweenX(F, sp, band, weldTol));  // middle chunk (x0 ≤ x ≤ x1)
      } else {
        for (const math::Plane* P : {&sp.Px0, &sp.Px1}) {  // two end pieces (outside x-slab)
          const hscdetail::AnalyticCut ac = hscdetail::cutAnalyticFace(F, *P, KeepSide::Below, band, weldTol);
          if (ac.kind == hscdetail::AnalyticCut::Kind::Split) pushIf(ac.keepFace);
          else if (ac.kind == hscdetail::AnalyticCut::Kind::KeepWhole) pushIf(F.face);
        }
      }
      continue;
    }
    // A wall the strip crosses with ONE plane (or only dips into the strip via y).
    const math::Plane& P = crossX0 ? sp.Px0 : (crossX1 ? sp.Px1 : sp.Pm);
    const KeepSide keep = common ? KeepSide::Above : KeepSide::Below;
    const hscdetail::AnalyticCut ac = hscdetail::cutAnalyticFace(F, P, keep, band, weldTol);
    if (ac.kind == hscdetail::AnalyticCut::Kind::Split) pushIf(ac.keepFace);
    else if (ac.kind == hscdetail::AnalyticCut::Kind::KeepWhole) { if (!common) pushIf(F.face); }
    else if (ac.kind == hscdetail::AnalyticCut::Kind::Drop) { /* empty on keep side */ }
    else { out.decline = StripWeldDecline::LoopOpen; return out; }
  }
  return out;
}


/// One box CAP face: the seam `arc` (P…Q on the plane) + `Q→Qb` down + `Qb→Pb` along the
/// bottom + `Pb→P` up. Shares `arc` with the wall strip sub-face, `Q→Qb`/`P→Pb` with the
/// adjacent caps, and the bottom edge with the bottom notch.
inline topo::Shape capFace(const Piece& arc, const math::Point3& P, const math::Point3& Q,
                           const math::Point3& Pb, const math::Point3& Qb, const math::Ax3& frame,
                           const math::Vec3& outward, double weldTol) {
  std::vector<Piece> pcs = {arc, Piece{Q, Qb}, Piece{Qb, Pb}, Piece{Pb, P}};
  std::vector<Piece> loop;
  if (!hscdetail::orderLoop(pcs, weldTol, loop)) return {};
  return hscdetail::planarFaceFromLoop(loop, frame, outward);
}

/// Append the three box CAP faces (CUT/COMMON): left (Px0, arc0), middle (Pm, arcM),
/// right (Px1, arc1). Every trace-plane normal points INTO the strip, so the cap's OUTWARD
/// normal (out of the RESULT solid) is +normal for CUT (the cap faces the removed strip)
/// and −normal for COMMON (the result IS the strip, so the cap faces outward from it).
inline bool appendCaps(StripWeldOp op, const ChainSeamGraph& g, const StripSeamGeom& sg,
                       double weldTol, std::vector<topo::Shape>& faces) {
  const double o = (op == StripWeldOp::Common) ? -1.0 : 1.0;
  const math::Plane Px0 = g.arcs[0].tracePlane;  // x=x0 (left)
  const math::Plane Px1 = g.arcs[1].tracePlane;  // x=x1 (right)
  const math::Plane Pm = g.arcs[2].tracePlane;   // y=y0 (middle)
  const math::Vec3 nL = Px0.pos.z.vec() * o, nR = Px1.pos.z.vec() * o, nM = Pm.pos.z.vec() * o;
  const topo::Shape cL = capFace(sg.arc0, sg.E, sg.J1, sg.Eb, sg.J1b, Px0.pos, nL, weldTol);
  const topo::Shape cM = capFace(sg.arcM, sg.J1, sg.J2, sg.J1b, sg.J2b, Pm.pos, nM, weldTol);
  const topo::Shape cR = capFace(sg.arc1, sg.J2, sg.X, sg.J2b, sg.Xb, Px1.pos, nR, weldTol);
  if (cL.isNull() || cM.isNull() || cR.isNull()) return false;
  faces.push_back(cL);
  faces.push_back(cM);
  faces.push_back(cR);
  return true;
}

/// Append the FUSE shell: `B`'s three NON-cutting faces WHOLE + its three CUTTING faces
/// (x=x0, x=x1, y=y0) NOTCHED by their cap footprint (the box rectangle minus the cap
/// region, whose curved side is the shared seam arc). Reuses the byte-frozen corner-clip
/// `mfwdetail::notchedBoxFace` (one notch per cutting face). The caps are now INTERIOR to
/// A∪B and dropped; A's CUT survivor faces (added by the caller) close the rest.
inline bool appendFuseShell(const ChainSeamGraph& g, const StripSeamGeom& sg, double weldTol,
                            std::vector<topo::Shape>& faces) {
  VertexPool pool;
  for (std::size_t i = 0; i < g.bPolys.size(); ++i)
    if (i != g.cutIdx[0] && i != g.cutIdx[1] && i != g.cutIdx[2])
      detail::triangulatePolygonToFaces(g.bPolys[i], pool, faces);  // whole non-cutting faces
  // Notch each cutting box face by its cap: left (arc0 E→J1), right (arc1 J2→X), middle
  // (arcM J1→J2). `notchedBoxFace(poly, arc, E, Eb, J, Jb)` drops the [J,Jb] attach segment.
  const topo::Shape nL =
      mfwdetail::notchedBoxFace(g.bPolys[g.cutIdx[0]], sg.arc0, sg.E, sg.Eb, sg.J1, sg.J1b, weldTol);
  const topo::Shape nR =
      mfwdetail::notchedBoxFace(g.bPolys[g.cutIdx[1]], sg.arc1, sg.X, sg.Xb, sg.J2, sg.J2b, weldTol);
  const topo::Shape nM =
      mfwdetail::notchedBoxFace(g.bPolys[g.cutIdx[2]], sg.arcM, sg.J1, sg.J1b, sg.J2, sg.J2b, weldTol);
  if (nL.isNull() || nR.isNull() || nM.isNull()) return false;
  faces.push_back(nL);
  faces.push_back(nR);
  faces.push_back(nM);
  return true;
}

/// Is the welded volume `v` inside the op's consistent inclusion–exclusion bound?
inline bool volumeConsistent(StripWeldOp op, double v, double vA, double vB, double band,
                             double tol) {
  switch (op) {
    case StripWeldOp::Cut:    return v > band && v <= vA + tol;
    case StripWeldOp::Common: return v > band && v <= std::min(vA, vB) + tol;
    case StripWeldOp::Fuse:   return v >= std::max(vA, vB) - tol && v <= vA + vB + tol;
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

/// Mandatory self-verify: mesh the welded result, require watertight, require the enclosed
/// volume in the op's consistent bound (measured from the A and B meshes).
inline StripWeldDecline selfVerify(const topo::Shape& result, const FreeformOperand& A,
                                   const ChainSeamGraph& g, StripWeldOp op, double deflection,
                                   double band, StripWeldReport& rep) {
  tess::MeshParams mp;
  mp.deflection = deflection;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(result);
  rep.watertight = tess::isWatertight(m);
  if (!rep.watertight) return StripWeldDecline::NotWatertight;
  rep.volume = std::fabs(tess::enclosedVolume(m));
  rep.volA = std::fabs(tess::enclosedVolume(tess::SolidMesher(mp).mesh(A.solid)));
  rep.volB = std::fabs(tess::enclosedVolume(polysMesh(g.bPolys, deflection)));
  const double tol = 0.05 * std::max(rep.volA + rep.volB, 1e-12);
  return volumeConsistent(op, rep.volume, rep.volA, rep.volB, band, tol)
             ? StripWeldDecline::Ok
             : StripWeldDecline::VolumeInconsistent;
}

}  // namespace mfswdetail

// ─────────────────────────────────────────────────────────────────────────────
// multiFaceStripClip — assemble + self-verify the CUT / COMMON strip-clip solid from the
// recognised operand `A`, its three-arc chain seam graph `g`, and the two-junction wall
// split `ss`. Returns the welded solid, or a NULL `Shape` with a measured
// `StripWeldDecline`. NEVER emits a leaky/wrong-volume solid. (FUSE is composed one level
// up by inclusion–exclusion on the CUT solid + `B`.)
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape multiFaceStripClip(const FreeformOperand& A, const ChainSeamGraph& g,
                                      const StripFaceSplit& ss, StripWeldOp op,
                                      double deflection = 0.01, StripWeldReport* report = nullptr) {
  using namespace mfswdetail;
  StripWeldReport rep;
  auto fail = [&](StripWeldDecline d) -> topo::Shape {
    rep.decline = d;
    if (report) *report = rep;
    return {};
  };

  const double diag = std::max(A.bbox.diagonal(), 1e-9);
  const double band = 1e-9 * diag;
  const double weldTol = 1e-7 * std::max(diag, 1.0);

  StripPlanes sp;
  sp.Px0 = g.arcs[0].tracePlane;  // x=x0
  sp.Px1 = g.arcs[1].tracePlane;  // x=x1
  sp.Pm = g.arcs[2].tracePlane;   // y=y0

  const StripSeamGeom sg = buildStripSeamGeom(A, g, ss);
  if (!sg.ok) return fail(StripWeldDecline::WeldOpen);

  const bool common = (op == StripWeldOp::Common);
  const bool fuse = (op == StripWeldOp::Fuse);
  std::vector<topo::Shape> faces;
  faces.push_back(common ? ss.faceStrip : ss.faceSurvivor);  // bowl sub-face

  // FUSE keeps A's CUT survivor region (caps interior → dropped) and welds B's shell.
  const AnalyticClip ac = clipAnalyticFaces(A, sp, sg, band, weldTol, common, faces);
  if (ac.decline != StripWeldDecline::Ok) return fail(ac.decline);
  if (!ac.sawBottom) return fail(StripWeldDecline::NoStraddlingBottom);

  const bool shellOk = fuse ? appendFuseShell(g, sg, weldTol, faces)
                            : appendCaps(op, g, sg, weldTol, faces);
  if (!shellOk) return fail(StripWeldDecline::LoopOpen);

  rep.faceCount = static_cast<int>(faces.size());
  if (faces.size() < 5) return fail(StripWeldDecline::WeldOpen);
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  const topo::Shape result = topo::ShapeBuilder::makeSolid({shell});

  const StripWeldDecline v = selfVerify(result, A, g, op, deflection, band, rep);
  if (v != StripWeldDecline::Ok) return fail(v);

  rep.decline = StripWeldDecline::Ok;
  if (report) *report = rep;
  return result;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_MULTI_FACE_STRIP_WELD_H
