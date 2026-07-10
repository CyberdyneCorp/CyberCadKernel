// SPDX-License-Identifier: Apache-2.0
//
// slab_disjoint_cut.h — MOAT M2b freeform↔analytic DISJOINT (MULTI-LUMP) CUT: the
// FIRST native freeform boolean whose result is TWO disconnected bodies.
//
// ── ROLE (the new topological outcome) ─────────────────────────────────────────────
// Every landed M2 freeform boolean returns ONE connected solid: `half_space_cut.h`
// clips a freeform operand by an infinite plane (one lump); `two_operand.h` /
// `multi_seam.h` weld a freeform prism against a box (one lump); `curved_wall_cut.h` /
// `freeform_freeform_cut.h` weld curved caps (one lump). Yet the app's `cc_boolean` @13
// routinely PARTS a curved body — a slot / channel / cut-through that SEPARATES the
// solid into TWO pieces (a disjoint result OCCT returns as a compound of two solids).
// The landed planar BSP explicitly treats a disjoint result as a degenerate NULL
// (`native_boolean.h`: "a cut that removes everything"); no native verb produces one.
//
// This verb lands the tractable disjoint pose: `A − B` where `B` is a FINITE all-planar
// SLAB (an axis-aligned box) positioned so a PAIR of its OPPOSITE parallel faces BOTH
// slice fully across `A`'s freeform wall and the slab lies strictly INTERIOR to `A`
// along the slab axis, so removing the slab SEPARATES `A` into two lumps:
//
//   A − B  =  (A ∩ {beyond the low slab face, −axis})  ⊎  (A ∩ {beyond the high face, +axis})
//
// Each lump is `A` restricted to one slab face's OUTER side, closed by the cross-section
// cap on that face — assembled through the LANDED, off-center-RELIABLE inter-solid-seam
// machinery (`buildInterSolidSeam` + `hscdetail::planarFaceFromLoop`), the SAME weld the
// landed two-operand FUSE uses (measured exact off-center; the standalone
// `freeformHalfSpaceCut` cap is NOT used — it over-estimates off-center, a measured
// limitation of that verb this composition deliberately avoids). The two lumps are
// assembled into a `Compound` of two `Solid`s — the native mirror of OCCT's
// `BRepAlgoAPI_Cut` compound result.
//
// ── SELF-VERIFY → OCCT FALLBACK (load-bearing) ────────────────────────────────────
// Admitted ONLY if: (a) BOTH lumps assemble + mesh WATERTIGHT; (b) the two lumps are
// genuinely DISJOINT — their world AABBs are separated along the slab axis (no overlap),
// so the result truly has two connected components; (c) the combined compound mesh is
// WATERTIGHT (two disjoint closed 2-manifolds satisfy the every-edge-used-twice
// invariant); and (d) the combined enclosed volume is a consistent CUT volume
// `0 < V ≤ V(A)`, and — when the closed-form op-volume is supplied — within a TWO-SIDED
// deflection-bounded band of it (so a wrong-volume weld is rejected, never shipped).
// Any failure returns a NULL `Shape` → OCCT `BRepAlgoAPI_Cut`. No leaky / overlapping /
// single-lump / wrong-volume result is EVER emitted; no tolerance is weakened.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// B1 `recogniseFreeformSolid`, `buildInterSolidSeam` (+ its `isdetail::`/`hscdetail::`
// machinery: `planarFaceFromLoop`, `Piece`), B3 `Aabb`/`meshAabb`, `polygon.h`
// `extractPolygons`/`isAllPlanar`, M0 `SolidMesher`/`isWatertight`/`enclosedVolume`.
// Additive sibling — touches none of them, nor `two_operand.h`, nor the tessellator.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_SLAB_DISJOINT_CUT_H
#define CYBERCAD_NATIVE_BOOLEAN_SLAB_DISJOINT_CUT_H

#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/boolean/half_space_cut.h"
#include "native/boolean/inter_solid_seam.h"
#include "native/boolean/polygon.h"
#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight DISJOINT two-lump result solid is returned.
enum class SlabCutDecline {
  Ok,
  NotAdmittedA,        ///< B1 declined operand `A`
  NotPlanarB,          ///< `B` is not an all-planar solid (the slab-box operand domain)
  NoSlabPair,          ///< no unique pair of OPPOSITE parallel `B` faces both cross `A`'s wall
  NotContained,        ///< a non-cutting `B` face does not contain `A` (slab does not pass fully across)
  SeamDeclinedLow,     ///< the inter-solid seam for the −axis lump declined
  SeamDeclinedHigh,    ///< the inter-solid seam for the +axis lump declined
  LumpOpenLow,         ///< the −axis lump could not be assembled/closed
  LumpOpenHigh,        ///< the +axis lump could not be assembled/closed
  NotDisjoint,         ///< the two lumps' AABBs overlap along the slab axis — not two bodies
  NotWatertight,       ///< self-verify: a lump / the combined compound mesh is not a closed 2-manifold
  VolumeInconsistent   ///< self-verify: the CUT volume is non-positive / off the bound / off the band
};

inline const char* slabCutDeclineName(SlabCutDecline d) noexcept {
  switch (d) {
    case SlabCutDecline::Ok: return "Ok";
    case SlabCutDecline::NotAdmittedA: return "NotAdmittedA";
    case SlabCutDecline::NotPlanarB: return "NotPlanarB";
    case SlabCutDecline::NoSlabPair: return "NoSlabPair";
    case SlabCutDecline::NotContained: return "NotContained";
    case SlabCutDecline::SeamDeclinedLow: return "SeamDeclinedLow";
    case SlabCutDecline::SeamDeclinedHigh: return "SeamDeclinedHigh";
    case SlabCutDecline::LumpOpenLow: return "LumpOpenLow";
    case SlabCutDecline::LumpOpenHigh: return "LumpOpenHigh";
    case SlabCutDecline::NotDisjoint: return "NotDisjoint";
    case SlabCutDecline::NotWatertight: return "NotWatertight";
    case SlabCutDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

namespace sdcdetail {

using hscdetail::Piece;

/// World control poles of `A`'s single freeform wall (location-placed). Mirrors
/// `isdetail::wallWorldPoles` — the pole hull the slab faces must straddle.
inline std::vector<math::Point3> wallWorldPoles(const OperandFace& wall) {
  const auto srf = topo::surfaceOf(wall.face);
  std::vector<math::Point3> out;
  if (!srf || !srf->surface) return out;
  const topo::Location& loc = srf->location;
  for (const math::Point3& p : srf->surface->poles)
    out.push_back(loc.isIdentity() ? p : loc.transform().applyToPoint(p));
  return out;
}

/// Does `poly`'s plane strictly separate the wall pole hull? Mirrors
/// `isdetail::planeStraddlesWall` — the "this B face slices A's wall" predicate.
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

/// The identified opposite-parallel slab pair on `B` (each polygon = one box face).
struct SlabPair {
  Polygon lowFace;   ///< the −axis cutting face (outward normal −axis)
  Polygon highFace;  ///< the +axis cutting face (outward normal +axis)
  math::Vec3 axis;   ///< unit slab axis (low → high; = −lowFace.normal)
};

/// Find the UNIQUE pair of OPPOSITE parallel `B` faces that BOTH straddle `A`'s wall
/// pole hull, and verify every OTHER `B` face contains `A` (the slab passes fully
/// across). Returns the pair oriented so `axis` points low → high, or a decline.
inline SlabCutDecline findSlabPair(const std::vector<Polygon>& bPolys,
                                   const std::vector<math::Point3>& poles, const Aabb& aBox,
                                   double band, SlabPair& pair) {
  std::vector<std::size_t> cutting;
  for (std::size_t i = 0; i < bPolys.size(); ++i)
    if (planeStraddlesWall(bPolys[i], poles, band)) cutting.push_back(i);
  if (cutting.size() != 2) return SlabCutDecline::NoSlabPair;

  const Polygon& f0 = bPolys[cutting[0]];
  const Polygon& f1 = bPolys[cutting[1]];
  if (math::dot(f0.plane.normal, f1.plane.normal) > -0.999)  // not opposite-parallel
    return SlabCutDecline::NoSlabPair;

  for (std::size_t i = 0; i < bPolys.size(); ++i)
    if (i != cutting[0] && i != cutting[1] &&
        !isdetail::aabbInsidePlane(aBox, bPolys[i], band))
      return SlabCutDecline::NotContained;

  // Orient low → high. f0 keeps the side OUTSIDE B beyond it; the slab axis points from
  // the low face toward the high face along −f0.normal (into B's material from f0).
  const math::Vec3 axis = math::Dir3{-f0.plane.normal}.vec();
  // The "low" face is the one whose outward normal is anti-parallel to `axis` (−axis).
  if (math::dot(f0.plane.normal, axis) < 0.0) { pair.lowFace = f0; pair.highFace = f1; }
  else { pair.lowFace = f1; pair.highFace = f0; }
  pair.axis = axis;
  return SlabCutDecline::Ok;
}

/// A large axis-aligned containment half-space box for ONE slab face: a box that shares
/// that cutting face's plane and extends FAR toward the slab interior + laterally, so
/// `buildInterSolidSeam` sees EXACTLY that one face slicing `A` and every other face
/// contains `A`. This lets the landed (off-center-reliable) seam machinery build the
/// lump that keeps `A` on the face's OUTER side — WITHOUT the standalone half-space cap.
///
/// `faceOrigin` a point on the face; `interior` the unit direction INTO the removed slab
/// (= −face.outwardNormal). The box spans [0 … far] along `interior` from the face and
/// ±far laterally, covering all of `A` on the interior side. Built as six single-quad
/// Plane faces (outward normals), matching the `two_operand_fixture` box convention.
inline topo::Shape containmentBox(const math::Point3& faceOrigin, const math::Vec3& interior,
                                  const Aabb& aBox, double far) {
  // Frame: w = interior (INTO the box), u/v lateral.
  const math::Dir3 w{interior};
  const math::Dir3 uRef = std::fabs(w.z()) < 0.9 ? math::Dir3{0, 0, 1} : math::Dir3{1, 0, 0};
  const math::Ax3 fr = math::Ax3::fromAxisAndRef(faceOrigin, w, uRef);
  const math::Vec3 U = fr.x.vec(), V = fr.y.vec(), W = fr.z.vec();
  // Eight corners: face plane (t=0) and far plane (t=far), each ±far in U,V.
  auto corner = [&](double t, double a, double b) {
    return faceOrigin + W * t + U * a + V * b;
  };
  const double L = far;
  const math::Point3 c000 = corner(0, -L, -L), c001 = corner(0, L, -L);
  const math::Point3 c011 = corner(0, L, L), c010 = corner(0, -L, L);
  const math::Point3 c100 = corner(L, -L, -L), c101 = corner(L, L, -L);
  const math::Point3 c111 = corner(L, L, L), c110 = corner(L, -L, L);
  (void)aBox;

  auto quad = [](const std::array<math::Point3, 4>& c, const math::Vec3& outward) {
    const math::Vec3 xd = c[1] - c[0];
    topo::FaceSurface pl{};
    pl.kind = topo::FaceSurface::Kind::Plane;
    pl.frame.origin = c[0];
    pl.frame.x = math::Dir3{xd};
    pl.frame.z = math::Dir3{outward};
    pl.frame.y = math::Dir3{math::cross(outward, xd)};
    const topo::Shape node = topo::ShapeBuilder::makeFace(pl, topo::Shape{});
    auto uv = [&](const math::Point3& p) {
      const math::Vec3 d = p - pl.frame.origin;
      return math::Point3{math::dot(d, pl.frame.x.vec()), math::dot(d, pl.frame.y.vec()), 0.0};
    };
    std::array<topo::Shape, 4> v;
    for (int k = 0; k < 4; ++k) v[k] = topo::ShapeBuilder::makeVertex(c[k]);
    std::vector<topo::Shape> edges;
    for (int k = 0; k < 4; ++k) {
      const int k1 = (k + 1) % 4;
      const math::Vec3 d = c[k1] - c[k];
      const double len = math::norm(d);
      topo::EdgeCurve ec{};
      ec.kind = topo::EdgeCurve::Kind::Line;
      ec.frame.origin = c[k];
      ec.frame.x = math::Dir3{d};
      const topo::Shape e = topo::ShapeBuilder::makeEdge(ec, 0.0, len, v[k], v[k1]);
      topo::PCurve pc{};
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = uv(c[k]);
      const math::Point3 e1 = uv(c[k1]);
      pc.dir2d = math::Vec3{(e1.x - pc.origin2d.x) / len, (e1.y - pc.origin2d.y) / len, 0.0};
      edges.push_back(topo::ShapeBuilder::addPCurve(e, node.tshape(), pc));
    }
    return topo::ShapeBuilder::makeFace(pl, topo::ShapeBuilder::makeWire(std::move(edges)), {},
                                        topo::Orientation::Forward);
  };
  std::vector<topo::Shape> faces;
  // The cutting face (t=0), outward = −interior (this is B's Pcut for the lump).
  faces.push_back(quad({c000, c001, c011, c010}, -W));
  faces.push_back(quad({c100, c110, c111, c101}, W));    // far face
  faces.push_back(quad({c000, c100, c101, c001}, -V));   // −V
  faces.push_back(quad({c010, c011, c111, c110}, V));    // +V
  faces.push_back(quad({c000, c010, c110, c100}, -U));   // −U
  faces.push_back(quad({c001, c101, c111, c011}, U));    // +U
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

/// Assemble ONE lump: `A` restricted to the cutting face's OUTER side (seam.aKeepFaces)
/// closed by the cross-section cap synthesised from the seam's closed loop on the trace
/// plane. Returns the lump Solid, or a null Shape if it cannot close (< 4 faces / open cap).
inline topo::Shape assembleLump(const InterSolidSeam& seam) {
  std::vector<topo::Shape> faces = seam.aKeepFaces;
  if (faces.size() < 3) return {};
  // The cross-section cap: the closed seam loop on the trace plane, outward = away from
  // A's kept material (= the trace plane's +z, which points INTO the removed slab).
  const math::Ax3 capFrame = seam.tracePlane.pos;
  const math::Vec3 outward = capFrame.z.vec();
  // F4: orient the cross-section cap by the mesher's actual +fr.z convention (the
  // off-centre-accurate rule), so the lump is CONSISTENTLY ORIENTED and its enclosedVolume
  // is trustworthy at every slab-face offset — not just a symmetric-centre cut.
  faces.push_back(hscdetail::planarFaceFromLoopByNormal(seam.capLoop, capFrame, outward));
  if (faces.size() < 4) return {};
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

/// Build ONE lump: the containment half-space box sharing `faceOrigin`'s plane with
/// interior direction `interior`, run through `buildInterSolidSeam`, assembled by
/// `assembleLump`. `seamBad`/`lumpBad` are the declines to raise on each failure. Returns
/// the lump Solid, or a null Shape (with `*why` set) on any decline.
inline topo::Shape buildLump(const FreeformOperand& opA, const math::Point3& faceOrigin,
                             const math::Vec3& interior, const Aabb& aBox, double far,
                             SlabCutDecline seamBad, SlabCutDecline lumpBad,
                             SlabCutDecline* why) {
  const topo::Shape box = containmentBox(faceOrigin, interior, aBox, far);
  SeamDecline sd = SeamDecline::Ok;
  const auto seam = buildInterSolidSeam(opA, box, &sd);
  if (!seam) { if (why) *why = seamBad; return {}; }
  const topo::Shape lump = assembleLump(*seam);
  if (lump.isNull()) { if (why) *why = lumpBad; return {}; }
  return lump;
}

/// The mandatory volume self-verify of the welded compound: 0 < V ≤ V(A) (upper bound),
/// and — when `analyticCutVolume` is finite/positive — within a deflection-bounded
/// TWO-SIDED band of it. Returns Ok, or the `VolumeInconsistent` decline.
inline SlabCutDecline verifyVolume(double v, double vA, double analyticCutVolume,
                                   double deflection) {
  if (!(v > 0.0) || std::isnan(v)) return SlabCutDecline::VolumeInconsistent;
  if (v > vA + 0.05 * std::max(vA, 1e-12)) return SlabCutDecline::VolumeInconsistent;
  if (!std::isnan(analyticCutVolume) && analyticCutVolume > 0.0) {
    constexpr double kVolConvergeSlope = 3.0;  // measured slope for the planar-walled prism
    const double relBand = std::min(0.5, 0.02 + kVolConvergeSlope * deflection);
    if (std::fabs(v - analyticCutVolume) > relBand * analyticCutVolume)
      return SlabCutDecline::VolumeInconsistent;
  }
  return SlabCutDecline::Ok;
}

/// Do two AABBs overlap along `axis` (their projections onto the axis intersect)?
inline bool overlapsAlongAxis(const Aabb& a, const Aabb& b, const math::Vec3& axis, double band) {
  auto proj = [&](const Aabb& bb, double& lo, double& hi) {
    lo = std::numeric_limits<double>::infinity();
    hi = -std::numeric_limits<double>::infinity();
    for (int c = 0; c < 8; ++c) {
      const math::Point3 w{(c & 1) ? bb.hi.x : bb.lo.x, (c & 2) ? bb.hi.y : bb.lo.y,
                           (c & 4) ? bb.hi.z : bb.lo.z};
      const double t = math::dot(axis, w.asVec());
      lo = std::min(lo, t);
      hi = std::max(hi, t);
    }
  };
  double aLo, aHi, bLo, bHi;
  proj(a, aLo, aHi);
  proj(b, bLo, bHi);
  return std::min(aHi, bHi) - std::max(aLo, bLo) > band;
}

}  // namespace sdcdetail

// ─────────────────────────────────────────────────────────────────────────────
// freeformSlabDisjointCut — the entry point. `A` is a freeform-walled solid; `B` a
// finite all-planar SLAB (axis-aligned box) whose two opposite parallel faces both
// slice fully across `A`, separating `A − B` into TWO lumps. Returns the welded,
// self-verified DISJOINT result (a `Compound` of two `Solid`s), or a NULL `Shape`
// (→ OCCT `BRepAlgoAPI_Cut`) with a measured decline. Never emits a leaky / single-
// lump / overlapping / wrong-volume result; no tolerance is weakened.
//
// `analyticCutVolume` (optional, NaN ⇒ unknown): the closed-form volume of `A − B`.
// When supplied the self-verify is TWO-SIDED (the combined lump volume must lie within a
// deflection-bounded band of it). When unknown, the CUT upper bound `0 < V ≤ V(A)` + the
// disjoint-component invariant still guarantee no overlapping/single-lump result.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape freeformSlabDisjointCut(const topo::Shape& A, const topo::Shape& B,
                                           double deflection = 0.01,
                                           SlabCutDecline* why = nullptr,
                                           double analyticCutVolume =
                                               std::numeric_limits<double>::quiet_NaN()) {
  using namespace sdcdetail;
  auto fail = [&](SlabCutDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  // (1) B1 recognise A (must present exactly one freeform wall — the seam-cut domain).
  OperandDecline b1 = OperandDecline::Ok;
  const auto opA = recogniseFreeformSolid(A, &b1);
  if (!opA || opA->freeform.size() != 1) return fail(SlabCutDecline::NotAdmittedA);

  // (2) B must be an all-planar slab box.
  if (!isAllPlanar(B)) return fail(SlabCutDecline::NotPlanarB);
  const std::vector<Polygon> bPolys = extractPolygons(B);
  if (bPolys.size() < 4) return fail(SlabCutDecline::NotPlanarB);

  const double diag = std::max(opA->bbox.diagonal(), 1e-9);
  const double band = 1e-9 * diag;
  const double far = 8.0 * diag;  // containment box reach (dwarfs A on every side)

  const OperandFace& wall = opA->faces[opA->freeform.front()];
  const std::vector<math::Point3> poles = wallWorldPoles(wall);
  if (poles.empty()) return fail(SlabCutDecline::NoSlabPair);

  // (3) locate the opposite-parallel slab pair straddling A's wall (+ containment guard).
  SlabPair pair;
  const SlabCutDecline located = findSlabPair(bPolys, poles, opA->bbox, band, pair);
  if (located != SlabCutDecline::Ok) return fail(located);

  // (4) build each lump through the LANDED, off-center-reliable inter-solid-seam weld.
  // Low lump keeps A on the low face's OUTER (−axis) side (containment interior = +axis);
  // high lump on the high face's OUTER (+axis) side (containment interior = −axis).
  SlabCutDecline bad = SlabCutDecline::Ok;
  const topo::Shape lumpLo = buildLump(*opA, pair.lowFace.vertices.front(), pair.axis, opA->bbox,
                                       far, SlabCutDecline::SeamDeclinedLow,
                                       SlabCutDecline::LumpOpenLow, &bad);
  if (lumpLo.isNull()) return fail(bad);
  const topo::Shape lumpHi = buildLump(*opA, pair.highFace.vertices.front(), -pair.axis, opA->bbox,
                                       far, SlabCutDecline::SeamDeclinedHigh,
                                       SlabCutDecline::LumpOpenHigh, &bad);
  if (lumpHi.isNull()) return fail(bad);

  // (5) each lump must mesh a CONSISTENTLY-ORIENTED closed 2-manifold (watertight AND
  // coherently wound — so its signed enclosedVolume is trustworthy off-centre).
  tess::MeshParams mp; mp.deflection = deflection;
  const tess::Mesh meshLo = tess::SolidMesher(mp).mesh(lumpLo);
  const tess::Mesh meshHi = tess::SolidMesher(mp).mesh(lumpHi);
  if (!tess::isConsistentlyOriented(meshLo) || !tess::isConsistentlyOriented(meshHi))
    return fail(SlabCutDecline::NotWatertight);

  // (6) confirm the two lumps are genuinely DISJOINT along the slab axis.
  if (overlapsAlongAxis(meshAabb(meshLo), meshAabb(meshHi), pair.axis, band))
    return fail(SlabCutDecline::NotDisjoint);

  // (7) assemble the two-body Compound and run the MANDATORY combined self-verify.
  const topo::Shape result = topo::ShapeBuilder::makeCompound({lumpLo, lumpHi});
  const tess::Mesh m = tess::SolidMesher(mp).mesh(result);
  if (!tess::isConsistentlyOriented(m)) return fail(SlabCutDecline::NotWatertight);
  const double vA = std::fabs(tess::enclosedVolume(tess::SolidMesher(mp).mesh(opA->solid)));
  const double v = std::fabs(tess::enclosedVolume(m));
  const SlabCutDecline vd = verifyVolume(v, vA, analyticCutVolume, deflection);
  if (vd != SlabCutDecline::Ok) return fail(vd);

  if (why) *why = SlabCutDecline::Ok;
  return result;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_SLAB_DISJOINT_CUT_H
