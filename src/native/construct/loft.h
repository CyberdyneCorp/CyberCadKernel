// SPDX-License-Identifier: Apache-2.0
//
// loft.h — native 2-section RULED loft (Phase 4 #4b, Tier B `native-construction`).
//
// Clean-room, OCCT-FREE builder that skins TWO closed section wires with EQUAL
// vertex/edge counts into a watertight ruled SOLID, mirroring OCCT's
// BRepOffsetAPI_ThruSections in its ruled=Standard_True mode used by the cc_*
// facade (see src/engine/occt/occt_construct.cpp solid_loft / solid_loft_wires):
// each polygon section is a straight-edge loop, corresponding vertices are paired
// 1:1, and each corresponding EDGE pair spans a RULED side face.
//
//   * build_ruled_loft(sectionA, sectionB) → Solid. Requires the two sections to
//     have the SAME vertex count n (≥3). Vertex A[i] is paired with B[i]; edge
//     A[i]→A[i+1] is ruled to B[i]→B[i+1]. Each ruled side face is a BILINEAR
//     patch (degree-1 Bézier surface, 2×2 poles) whose four corners are the two
//     section-edge endpoints; this is EXACTLY the linear ruled surface OCCT lays
//     between two straight polygon edges. The two sections are capped with planar
//     faces (each section polygon must be planar — the loft input from the facade
//     always is for solid_loft; solid_loft_wires is honestly rejected when a wire
//     is non-planar, see below). Shell → Solid, oriented outward.
//
//   * build_loft(bottomXY, topXY, depth) — entry point for cc_solid_loft: build the
//     two profiles at z=0 (bottom XY) and z=depth (top XY), then build_ruled_loft.
//   * build_loft_wires(aXYZ, bXYZ) — entry point for cc_solid_loft_wires: use the
//     two 3D wires directly.
//
// ── SUPPORTED vs DEFERRED (honest — the builder returns a NULL Shape so the engine
//    falls through to OCCT; it NEVER fakes a wrong shape) ──────────────────────
//   SUPPORTED natively (Tier B):
//     * TWO sections, EQUAL vertex count n ≥ 3, both sections PLANAR, neither
//       degenerate to a point → ruled skin + two planar caps → watertight solid.
//   DEFERRED to OCCT (NULL → NativeEngine forwards the same arguments):
//     * MISMATCHED vertex counts (n_A ≠ n_B) — pairing is ambiguous; OCCT's
//       ThruSections re-parametrizes/​resamples, which is Tier C.
//     * a NON-PLANAR section wire (the ruled skin can still be built, but a planar
//       cap cannot close it; a non-planar cap is Tier C).
//     * a section that DEGENERATES to a point (zero area / all points coincident).
//     * 3+ sections, guided / rail lofts — Tier C.
//
// The RULED side face welds watertight to its neighbours and to the caps because
// its boundary edges are the SAME straight-line edges the caps and adjacent side
// faces use (topology node sharing), and the bilinear surface satisfies
// S(u,0)=A-edge, S(u,1)=B-edge, S(0,v)=A[i]→B[i], S(1,v)=A[i+1]→B[i+1] exactly, so
// the two-stage mesher (edge_mesher + face_mesher) produces identical seam points.
//
// REFERENCE ORACLE ONLY: BRepOffsetAPI_ThruSections was consulted to confirm the
// ruled=true face decomposition (one linear side face per corresponding edge pair,
// planar end caps) and the outward orientation; nothing is copied. The bilinear
// Bézier parametrization matches src/native/math bezierSurfacePoint (row-major,
// U outer) so the native tessellator agrees.
//
// Cognitive complexity: build_ruled_loft is a linear assembler (~13, flagged); the
// ruled-face + planarity helpers are short. OCCT-FREE. Header-only. clang++ c++20.
//
#ifndef CYBERCAD_NATIVE_CONSTRUCT_LOFT_H
#define CYBERCAD_NATIVE_CONSTRUCT_LOFT_H

#include "native/construct/construct.h"  // detail::xy / lineEdge / planarFace / kProfileTol
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace cybercad::native::construct {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

namespace detail {

// A section: the ordered 3D vertices of a closed polygon loop, its unit plane
// normal and a point on the plane. `planar` is false when the points do not lie
// on a common plane within tolerance (→ the loft is deferred to OCCT).
struct Section {
  std::vector<math::Point3> pts;  ///< de-duplicated loop vertices, in order
  math::Dir3 normal;              ///< best-fit / Newell plane normal (unit)
  math::Point3 origin;            ///< a point on the plane (pts[0])
  bool planar = false;            ///< all pts within kProfileTol of the plane
  bool degenerate = true;         ///< < 3 distinct pts or ~zero area
};

// Newell's method: robust polygon normal (also gives 2× the signed planar area
// projected onto that normal). Works for any planar polygon regardless of the
// axis it lies in; degenerate (collinear / coincident) loops yield a null normal.
inline math::Vec3 newellNormal(const std::vector<math::Point3>& p) noexcept {
  math::Vec3 n{0, 0, 0};
  const std::size_t m = p.size();
  for (std::size_t i = 0; i < m; ++i) {
    const math::Point3& a = p[i];
    const math::Point3& b = p[(i + 1) % m];
    n.x += (a.y - b.y) * (a.z + b.z);
    n.y += (a.z - b.z) * (a.x + b.x);
    n.z += (a.x - b.x) * (a.y + b.y);
  }
  return n;
}

// Analyse a raw XYZ point loop into a Section: de-duplicate a repeated closing
// vertex, compute the Newell normal, test planarity (max deviation from the plane
// through pts[0]) and non-degeneracy (≥3 distinct pts and a non-null normal).
inline Section analyzeSection(const std::vector<math::Point3>& raw) {
  Section s;
  s.pts = raw;
  // Drop a closing duplicate (loop[last] == loop[0]).
  if (s.pts.size() >= 2 &&
      math::distance(s.pts.front(), s.pts.back()) < kProfileTol) {
    s.pts.pop_back();
  }
  if (s.pts.size() < 3) return s;  // degenerate: too few distinct points

  const math::Vec3 nv = newellNormal(s.pts);
  const double nlen = math::norm(nv);
  if (nlen < kProfileTol) return s;  // collinear / zero-area → degenerate

  s.origin = s.pts[0];
  s.normal = math::Dir3::fromUnit(nv / nlen);
  s.degenerate = false;

  // Planarity: every point within kProfileTol of the plane {origin, normal}.
  double maxDev = 0.0;
  for (const math::Point3& q : s.pts)
    maxDev = std::max(maxDev, std::fabs(math::dot(q - s.origin, s.normal.vec())));
  s.planar = maxDev <= 1e-6;
  return s;
}

// Reorder section B's vertices so B[i] pairs with A[i] as OCCT's
// BRepFill_CompatibleWires::ComputeOrigin does: the sections keep their own
// vertex ORDER but the rotational START (and traversal DIRECTION) of B is chosen
// to MINIMISE the summed distance between paired vertices, after removing the
// barycenter offset along the reference (A) plane normal. Without this step,
// rotated/twisted sections (e.g. a square lofted to a 45°-rotated square) pair
// mismatched corners → the ruled skin self-intersects and the solid collapses to
// a fraction of its true volume. The oracle compares BOTH forward (start j, then
// j+1, j+2, …) and backward (start j, then j−1, j−2, …) walks of B against A's
// fixed order and keeps the global minimum.
//
// Returns the reordered copy of B.pts (same size as A.pts; caller guarantees the
// counts already match and both sections are non-degenerate + planar).
inline std::vector<math::Point3> alignSectionB(const Section& A, const Section& B) {
  const std::size_t n = A.pts.size();

  // Barycenters, and the offset that slides B onto A's plane (kill the component
  // along A's normal so only the in-plane correspondence is compared, matching
  // OCCT's `anOffsetProj = (aVec·N) N; Offset = PrevBary − (CurBary − proj)`).
  auto bary = [](const std::vector<math::Point3>& p) {
    math::Vec3 c{0, 0, 0};
    for (const auto& q : p) c += q.asVec();
    return math::Point3{c / static_cast<double>(p.size())};
  };
  const math::Point3 prevBary = bary(A.pts);
  const math::Point3 curBary = bary(B.pts);
  const math::Vec3 nrm = A.normal.vec();
  const math::Vec3 aVec = curBary - prevBary;
  const math::Vec3 curBaryProj = curBary.asVec() - nrm * math::dot(aVec, nrm);
  const math::Vec3 offset = prevBary.asVec() - curBaryProj;  // add to each B point

  // Try every rotational start j in both directions; keep the minimum summed
  // paired distance. `dir = +1` walks B forward (k = j+n), `dir = −1` backward.
  double best = std::numeric_limits<double>::infinity();
  std::size_t bestJ = 0;
  int bestDir = 1;
  auto sumDist = [&](std::size_t j, int dir) {
    double s = 0.0;
    for (std::size_t k = 0; k < n; ++k) {
      const std::size_t bi =
          (dir > 0 ? j + k : j + n - k) % n;  // + n keeps the backward index ≥ 0
      const math::Vec3 pb = B.pts[bi].asVec() + offset;
      s += math::norm(A.pts[k].asVec() - pb);
    }
    return s;
  };
  for (std::size_t j = 0; j < n; ++j) {
    for (int dir : {1, -1}) {
      const double s = sumDist(j, dir);
      if (s < best) {
        best = s;
        bestJ = j;
        bestDir = dir;
      }
    }
  }

  std::vector<math::Point3> out(n);
  for (std::size_t k = 0; k < n; ++k)
    out[k] = B.pts[(bestDir > 0 ? bestJ + k : bestJ + n - k) % n];
  return out;
}

// One RULED side face between corresponding straight edges A[i]→A[j] (bottom, v=0)
// and B[i]→B[j] (top, v=1). The surface is a bilinear (degree-1) Bézier with poles
// laid row-major, U outer (nPolesU=2 over u, nPolesV=2 over v):
//   pole(0,0)=A[i]  pole(0,1)=B[i]      // u=0 column: the A[i]→B[i] side edge
//   pole(1,0)=A[j]  pole(1,1)=B[j]      // u=1 column: the A[j]→B[j] side edge
// so S(u,0) traces A[i]→A[j] and S(u,1) traces B[i]→B[j] (the section edges), and
// S(0,v)/S(1,v) trace the two side edges. Each boundary edge carries a Line pcurve
// on this face's (u,v) plane so the mesher flattens the loop exactly. The four
// bounding vertices are SHARED (same TShape nodes) with the caps and neighbours.
//
// `orient` flips the material side so the effective outward normal points out of
// the solid (chosen by the caller from the section winding).
inline topo::Shape ruledSideFace(const topo::Shape& ai, const topo::Shape& aj,
                                 const topo::Shape& bi, const topo::Shape& bj,
                                 topo::Orientation orient) {
  const math::Point3 Ai = *topo::pointOf(ai);
  const math::Point3 Aj = *topo::pointOf(aj);
  const math::Point3 Bi = *topo::pointOf(bi);
  const math::Point3 Bj = *topo::pointOf(bj);

  topo::FaceSurface surf;
  surf.kind = topo::FaceSurface::Kind::Bezier;
  surf.degreeU = 1;
  surf.degreeV = 1;
  surf.nPolesU = 2;
  surf.nPolesV = 2;
  surf.poles = {Ai, Bi, Aj, Bj};  // row-major, U outer: (0,0)(0,1)(1,0)(1,1)

  // Boundary edges with their Line pcurves on the (u,v) plane. Walk the loop
  // a0→a1 (v=0), a1→b1 (u=1), b1→b0 (v=1), b0→a0 (u=0) so it is a simple CCW-in-UV
  // quad; the face Orientation makes the effective normal outward.
  auto edgeWithLinePCurve = [](const topo::Shape& v0, const topo::Shape& v1,
                               const math::Point3& uv0, const math::Point3& uv1) -> topo::Shape {
    const topo::Shape e = detail::lineEdge(v0, v1);
    const double len = detail::edgeLen(*topo::pointOf(v0), *topo::pointOf(v1));
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = uv0;
    pc.dir2d = (uv1 - uv0) / len;  // per-parameter step so uv0 + dir2d·len = uv1
    return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
  };

  const math::Point3 uv00{0, 0, 0}, uv10{1, 0, 0}, uv11{1, 1, 0}, uv01{0, 1, 0};
  std::vector<topo::Shape> edges;
  edges.reserve(4);
  edges.push_back(edgeWithLinePCurve(ai, aj, uv00, uv10));  // A[i]→A[j] at v=0
  edges.push_back(edgeWithLinePCurve(aj, bj, uv10, uv11));  // A[j]→B[j] at u=1
  edges.push_back(edgeWithLinePCurve(bj, bi, uv11, uv01));  // B[j]→B[i] at v=1
  edges.push_back(edgeWithLinePCurve(bi, ai, uv01, uv00));  // B[i]→A[i] at u=0
  const topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));
  return topo::ShapeBuilder::makeFace(surf, wire, {}, orient);
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_ruled_loft — skin two equal-count planar section loops into a solid.
//
// Returns a NULL Shape (→ OCCT fallthrough) when the sections are not both usable
// (mismatched counts, degenerate, or non-planar). Otherwise:
//   * pair A[i]↔B[i], A-edge i ↔ B-edge i;
//   * one bilinear ruled side face per edge pair;
//   * two planar caps (bottom = section A, top = section B), wound so their
//     outward normals point AWAY from the other section (a convex-agnostic choice:
//     the A-cap normal points along −(B_centroid − A_centroid), the B-cap along +);
//   * side-face orientation chosen so its natural (bilinear) normal points OUTWARD
//     (away from the loft's central axis), consistent with the caps.
//
// Cognitive complexity: linear assembler (~13, flagged).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_ruled_loft(const std::vector<math::Point3>& sectionA,
                                    const std::vector<math::Point3>& sectionB) {
  const detail::Section A = detail::analyzeSection(sectionA);
  const detail::Section B = detail::analyzeSection(sectionB);
  if (A.degenerate || B.degenerate) return {};             // a section is a point/line
  if (A.pts.size() != B.pts.size()) return {};             // mismatched counts → Tier C
  if (!A.planar || !B.planar) return {};                   // non-planar cap → Tier C

  const std::size_t n = A.pts.size();

  // Align B's vertex correspondence to A (rotate/flip B's start so paired
  // vertices are nearest — see alignSectionB). Skipping this makes twisted
  // sections pair the wrong corners and the ruled skin self-intersects.
  const std::vector<math::Point3> bpts = detail::alignSectionB(A, B);

  // Shared vertex rings (one node per corresponding vertex, shared by the two
  // adjacent side faces AND the cap that uses it → watertight welds).
  std::vector<topo::Shape> av(n), bv(n);
  for (std::size_t i = 0; i < n; ++i) {
    av[i] = topo::ShapeBuilder::makeVertex(A.pts[i]);
    bv[i] = topo::ShapeBuilder::makeVertex(bpts[i]);
  }

  // Loft axis: from A's centroid toward B's centroid. Used to orient every face's
  // effective normal outward (side faces away from the axis, caps away from the
  // opposite section).
  auto centroid = [](const std::vector<math::Point3>& p) {
    math::Vec3 c{0, 0, 0};
    for (const auto& q : p) c += q.asVec();
    return math::Point3{c / static_cast<double>(p.size())};
  };
  const math::Point3 ca = centroid(A.pts), cb = centroid(bpts);
  const math::Vec3 axis = cb - ca;  // A → B

  std::vector<topo::Shape> faces;
  faces.reserve(n + 2);

  // Ruled side faces. For each edge pair, the bilinear patch's natural normal at
  // the patch centre (u=v=0.5) is compared against an outward reference (the patch
  // centre pushed away from the loft axis line): flip the face when they oppose.
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    // Bilinear patch centre and a natural normal there (∂u × ∂v of the bilinear).
    const math::Point3 mid{(A.pts[i].asVec() + A.pts[j].asVec() + bpts[i].asVec() +
                            bpts[j].asVec()) /
                           4.0};
    const math::Vec3 du = 0.5 * ((A.pts[j] - A.pts[i]) + (bpts[j] - bpts[i]));  // ∂u at v=.5
    const math::Vec3 dv = 0.5 * ((bpts[i] - A.pts[i]) + (bpts[j] - A.pts[j]));  // ∂v at u=.5
    const math::Vec3 nat = math::cross(du, dv);
    // Outward reference: radial component of the patch centre from the axis line
    // (mid − ca projected off the axis direction) points away from the interior.
    const math::Vec3 axisDir =
        math::norm(axis) > kProfileTol ? axis / math::norm(axis) : math::Vec3{0, 0, 1};
    const math::Vec3 rel = mid - ca;
    const math::Vec3 radial = rel - axisDir * math::dot(rel, axisDir);
    const topo::Orientation o =
        math::dot(nat, radial) < 0.0 ? topo::Orientation::Reversed : topo::Orientation::Forward;
    faces.push_back(detail::ruledSideFace(av[i], av[j], bv[i], bv[j], o));
  }

  // Planar caps. The A-cap's outward normal points AWAY from B (−axis side); the
  // B-cap points +axis. planarFace winds the loop to match the supplied normal.
  auto capNormal = [](const detail::Section& sec, const math::Vec3& outward) -> math::Dir3 {
    return math::dot(sec.normal.vec(), outward) < 0.0 ? sec.normal.reversed() : sec.normal;
  };
  faces.push_back(detail::planarFace(av, capNormal(A, -axis), topo::Orientation::Forward));
  faces.push_back(detail::planarFace(bv, capNormal(B, axis), topo::Orientation::Forward));

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// build_loft — cc_solid_loft entry point. Build the bottom profile at z=0 (from
// bottomXY) and the top profile at z=depth (from topXY), then ruled-loft them.
// Requires equal point counts, ≥3 each, depth > 0. NULL → OCCT fallthrough.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_loft(const double* bottomXY, int bottomCount, const double* topXY,
                              int topCount, double depth) {
  if (bottomXY == nullptr || topXY == nullptr || bottomCount < 3 || topCount < 3 ||
      !(depth > kMinDepth)) {
    return {};
  }
  std::vector<math::Point3> a, b;
  a.reserve(static_cast<std::size_t>(bottomCount));
  b.reserve(static_cast<std::size_t>(topCount));
  for (int i = 0; i < bottomCount; ++i) a.push_back({bottomXY[i * 2], bottomXY[i * 2 + 1], 0.0});
  for (int i = 0; i < topCount; ++i) b.push_back({topXY[i * 2], topXY[i * 2 + 1], depth});
  return build_ruled_loft(a, b);
}

// ─────────────────────────────────────────────────────────────────────────────
// build_loft_wires — cc_solid_loft_wires entry point. Use the two arbitrary 3D
// wires directly (aXYZ / bXYZ are flat x,y,z triples). Requires equal point counts
// ≥3, both planar, non-degenerate. NULL → OCCT fallthrough.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_loft_wires(const double* aXYZ, int aCount, const double* bXYZ,
                                    int bCount) {
  if (aXYZ == nullptr || bXYZ == nullptr || aCount < 3 || bCount < 3) return {};
  std::vector<math::Point3> a, b;
  a.reserve(static_cast<std::size_t>(aCount));
  b.reserve(static_cast<std::size_t>(bCount));
  for (int i = 0; i < aCount; ++i) a.push_back({aXYZ[i * 3], aXYZ[i * 3 + 1], aXYZ[i * 3 + 2]});
  for (int i = 0; i < bCount; ++i) b.push_back({bXYZ[i * 3], bXYZ[i * 3 + 1], bXYZ[i * 3 + 2]});
  return build_ruled_loft(a, b);
}

}  // namespace cybercad::native::construct

#endif  // CYBERCAD_NATIVE_CONSTRUCT_LOFT_H
