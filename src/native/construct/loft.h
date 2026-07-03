// SPDX-License-Identifier: Apache-2.0
//
// loft.h — native RULED loft (Phase 4 #4b, Tier B + the 3+-section extension of
// `native-construction`).
//
// Clean-room, OCCT-FREE builder that skins 2..N closed section wires — each with
// the SAME EQUAL vertex/edge count — into a watertight ruled SOLID, mirroring
// OCCT's BRepOffsetAPI_ThruSections in its ruled=Standard_True mode used by the
// cc_* facade (see src/engine/occt/occt_construct.cpp solid_loft / solid_loft_wires
// and the cc_solid_loft chain): each polygon section is a straight-edge loop,
// corresponding vertices are paired 1:1 across the whole section chain, and each
// corresponding EDGE pair between two CONSECUTIVE sections spans a RULED side face.
// For N sections this yields (N−1) ruled bands stacked end to end + two planar end
// caps (first section, last section); the internal sections are NOT capped — they
// are shared vertex rings welding the adjacent bands.
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
//   * build_ruled_loft_sections(sections) → Solid. The N-section generalisation
//     (N ≥ 2). All sections must share the SAME vertex count n (≥3), all be PLANAR
//     and non-degenerate. Section k is aligned to its PREDECESSOR (k−1) with the
//     same rotational/flip correspondence rule the 2-section path uses, so the
//     pairing propagates consistently down the chain. Emits (N−1) ruled bands +
//     the first and last planar cap. Internal sections are shared vertex rings
//     (no cap). build_ruled_loft(A,B) is the N=2 special case (kept intact).
//
//   * build_loft(bottomXY, topXY, depth) — entry point for cc_solid_loft: build the
//     two profiles at z=0 (bottom XY) and z=depth (top XY), then build_ruled_loft.
//   * build_loft_wires(aXYZ, bXYZ) — entry point for cc_solid_loft_wires: use the
//     two 3D wires directly.
//   * build_loft_sections(sectionsXYZ, counts, sectionCount) — entry point for the
//     cc_solid_loft chain / a section list: each section is a flat (x,y,z) triple
//     loop; build every Section then build_ruled_loft_sections. The wiring step
//     (native_engine) calls this for a 3+-section loft request.
//
// ── SUPPORTED vs DEFERRED (honest — the builder returns a NULL Shape so the engine
//    falls through to OCCT; it NEVER fakes a wrong shape) ──────────────────────
//   SUPPORTED natively (Tier B + the 3+-section extension):
//     * 2..N sections, ALL with the SAME vertex count n ≥ 3, ALL PLANAR, none
//       degenerate to a point → (N−1) ruled bands + two planar end caps →
//       watertight solid. The internal sections are shared vertex rings.
//   DEFERRED to OCCT (NULL → NativeEngine forwards the same arguments):
//     * MISMATCHED vertex counts (any two sections differ in n) — pairing is
//       ambiguous; OCCT's ThruSections re-parametrizes/resamples, which is Tier C.
//     * a NON-PLANAR section wire (the ruled skin can still be built, but a planar
//       cap cannot close a non-planar END section; a non-planar cap is Tier C).
//       (An internal section is not capped, but is still required planar here so
//       the correspondence/alignment stays a well-posed in-plane match; a
//       non-planar internal section is deferred.)
//     * a section that DEGENERATES to a point (zero area / all points coincident).
//     * guided / rail lofts, and a loft whose stacked ruled skin SELF-INTERSECTS
//       (a section chain that folds back on itself) — that needs surface-surface
//       intersection (Tier 4), so it is deferred, NOT attempted here. The mesh-level
//       self-verify (engine watertight + volume) DISCARDS any such candidate.
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

// Loft centroid of a point loop (shared by the alignment + orientation code).
inline math::Point3 loopCentroid(const std::vector<math::Point3>& p) {
  math::Vec3 c{0, 0, 0};
  for (const auto& q : p) c += q.asVec();
  return math::Point3{c / static_cast<double>(p.size())};
}

// The material-outward Orientation for one ruled side face of the band between
// two consecutive rings `a` (v=0) and `b` (v=1), edge i→j. Compares the bilinear
// patch's natural normal at its centre against an outward radial reference (the
// patch centre pushed off the local band axis line ca→cb) and reverses the face
// when they oppose — the SAME rule the 2-section path uses, factored out so every
// band in an N-section chain orients consistently.
inline topo::Orientation sideFaceOrientation(const std::vector<math::Point3>& a,
                                             const std::vector<math::Point3>& b, std::size_t i,
                                             std::size_t j, const math::Point3& ca,
                                             const math::Vec3& axis) {
  const math::Point3 mid{(a[i].asVec() + a[j].asVec() + b[i].asVec() + b[j].asVec()) / 4.0};
  const math::Vec3 du = 0.5 * ((a[j] - a[i]) + (b[j] - b[i]));  // ∂u at v=.5
  const math::Vec3 dv = 0.5 * ((b[i] - a[i]) + (b[j] - a[j]));  // ∂v at u=.5
  const math::Vec3 nat = math::cross(du, dv);
  const double al = math::norm(axis);
  const math::Vec3 axisDir = al > kProfileTol ? axis / al : math::Vec3{0, 0, 1};
  const math::Vec3 rel = mid - ca;
  const math::Vec3 radial = rel - axisDir * math::dot(rel, axisDir);
  return math::dot(nat, radial) < 0.0 ? topo::Orientation::Reversed : topo::Orientation::Forward;
}

// Append the (n) ruled side faces of ONE band between consecutive shared vertex
// rings `av` (v=0) and `bv` (v=1), whose 3D points are `ap`/`bp`. The band's local
// axis ca→cb orients each face outward. Rings are SHARED across adjacent bands
// (an internal ring is the top of one band and the bottom of the next), so the
// bands weld watertight with no duplicate seam vertices.
inline void appendRuledBand(std::vector<topo::Shape>& faces,
                            const std::vector<topo::Shape>& av,
                            const std::vector<topo::Shape>& bv,
                            const std::vector<math::Point3>& ap,
                            const std::vector<math::Point3>& bp) {
  const std::size_t n = av.size();
  const math::Point3 ca = loopCentroid(ap), cb = loopCentroid(bp);
  const math::Vec3 axis = cb - ca;
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    const topo::Orientation o = sideFaceOrientation(ap, bp, i, j, ca, axis);
    faces.push_back(ruledSideFace(av[i], av[j], bv[i], bv[j], o));
  }
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_ruled_loft_sections — skin 2..N equal-count planar section loops into one
// watertight ruled solid. This is the N-section generalisation of the Tier-B
// 2-section loft; build_ruled_loft(A,B) below is the N=2 special case.
//
// Returns a NULL Shape (→ OCCT fallthrough) when the sections are not ALL usable
// (fewer than 2 sections, any section degenerate, non-planar, or a count that
// differs from the first section's). Otherwise:
//   * ALIGN each section k (k≥1) to its predecessor k−1 with alignSectionB, so the
//     1:1 vertex correspondence propagates consistently along the whole chain (a
//     twisted / rotated section stays paired to its nearest neighbour corners).
//   * BUILD one shared vertex ring per section; an INTERNAL ring is used by the
//     band below AND the band above it (same TShape nodes → watertight weld).
//   * EMIT (N−1) ruled bands (one bilinear side face per corresponding edge pair
//     per consecutive section pair, oriented outward via the LOCAL band axis).
//   * CAP only the first + last section (planar caps whose outward normals point
//     away from the adjacent section). Internal sections are NOT capped.
//
// The caller (build_loft / build_loft_wires / build_loft_sections and, above all,
// the engine wiring) runs a MANDATORY self-verify (watertight + sane enclosed
// volume) on the result and DISCARDS it → OCCT if the stacked skin self-intersects
// or fails to close; this builder never fakes such a case.
//
// Cognitive complexity: a linear assembler (~10) that leans on the detail helpers.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_ruled_loft_sections(const std::vector<std::vector<math::Point3>>& raw) {
  if (raw.size() < 2) return {};  // need at least two sections to skin

  // Analyse + validate every section: non-degenerate, planar, equal vertex count.
  std::vector<detail::Section> secs;
  secs.reserve(raw.size());
  for (const auto& r : raw) {
    detail::Section s = detail::analyzeSection(r);
    if (s.degenerate || !s.planar) return {};  // point/line/skew → OCCT (Tier C/4)
    secs.push_back(std::move(s));
  }
  const std::size_t n = secs.front().pts.size();
  for (const auto& s : secs)
    if (s.pts.size() != n) return {};  // mismatched counts → OCCT (Tier C)

  const std::size_t sc = secs.size();

  // Propagate the vertex correspondence down the chain: section 0 keeps its own
  // order; section k≥1 is re-indexed to pair nearest to the ALREADY-aligned
  // section k−1. `aligned[k]` holds the reordered 3D points of section k.
  std::vector<std::vector<math::Point3>> aligned(sc);
  aligned[0] = secs[0].pts;
  for (std::size_t k = 1; k < sc; ++k) {
    detail::Section prev = secs[k - 1];
    prev.pts = aligned[k - 1];  // align against the already-fixed predecessor
    aligned[k] = detail::alignSectionB(prev, secs[k]);
  }

  // One shared vertex ring per section (rings are shared between adjacent bands).
  std::vector<std::vector<topo::Shape>> ring(sc, std::vector<topo::Shape>(n));
  for (std::size_t k = 0; k < sc; ++k)
    for (std::size_t i = 0; i < n; ++i)
      ring[k][i] = topo::ShapeBuilder::makeVertex(aligned[k][i]);

  std::vector<topo::Shape> faces;
  faces.reserve(n * (sc - 1) + 2);

  // (N−1) ruled bands, each welded to its neighbour through the shared ring.
  for (std::size_t k = 0; k + 1 < sc; ++k)
    detail::appendRuledBand(faces, ring[k], ring[k + 1], aligned[k], aligned[k + 1]);

  // Planar end caps only. The first cap's outward normal points away from section 1
  // (−axis0); the last cap's outward points along the last band's axis. planarFace
  // winds the loop to match the supplied normal.
  auto capNormal = [](const detail::Section& sec, const math::Vec3& outward) -> math::Dir3 {
    return math::dot(sec.normal.vec(), outward) < 0.0 ? sec.normal.reversed() : sec.normal;
  };
  const math::Vec3 axis0 = detail::loopCentroid(aligned[1]) - detail::loopCentroid(aligned[0]);
  const math::Vec3 axisN =
      detail::loopCentroid(aligned[sc - 1]) - detail::loopCentroid(aligned[sc - 2]);
  faces.push_back(
      detail::planarFace(ring.front(), capNormal(secs.front(), -axis0), topo::Orientation::Forward));
  faces.push_back(
      detail::planarFace(ring.back(), capNormal(secs.back(), axisN), topo::Orientation::Forward));

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// build_ruled_loft — the 2-section entry (Tier B). Kept as the public 2-section
// API; delegates to build_ruled_loft_sections with {A, B}. Returns NULL (→ OCCT)
// for mismatched counts / degenerate / non-planar exactly as before.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_ruled_loft(const std::vector<math::Point3>& sectionA,
                                    const std::vector<math::Point3>& sectionB) {
  return build_ruled_loft_sections({sectionA, sectionB});
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

// ─────────────────────────────────────────────────────────────────────────────
// build_loft_sections — entry point for the cc_solid_loft CHAIN / a section list
// (3+ sections). Each section is a flat (x,y,z) triple loop; `sectionsXYZ` holds
// the sections back to back, `counts[k]` is the vertex count of section k, and
// `sectionCount` is the number of sections (≥2). Builds every Section from its
// slice of the buffer, then build_ruled_loft_sections. NULL → OCCT fallthrough
// (fewer than 2 sections, any section < 3 pts, mismatched counts, non-planar, or
// degenerate — see build_ruled_loft_sections). The wiring step (native_engine)
// calls this for a multi-section loft request; the 2-section facade paths keep
// using build_loft / build_loft_wires unchanged.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_loft_sections(const double* sectionsXYZ, const int* counts,
                                       int sectionCount) {
  if (sectionsXYZ == nullptr || counts == nullptr || sectionCount < 2) return {};
  std::vector<std::vector<math::Point3>> secs;
  secs.reserve(static_cast<std::size_t>(sectionCount));
  std::size_t off = 0;  // running offset into the flat (x,y,z) buffer, in doubles
  for (int k = 0; k < sectionCount; ++k) {
    const int cnt = counts[k];
    if (cnt < 3) return {};  // a section with < 3 vertices → OCCT
    std::vector<math::Point3> pts;
    pts.reserve(static_cast<std::size_t>(cnt));
    for (int i = 0; i < cnt; ++i)
      pts.push_back({sectionsXYZ[off + i * 3], sectionsXYZ[off + i * 3 + 1],
                     sectionsXYZ[off + i * 3 + 2]});
    off += static_cast<std::size_t>(cnt) * 3;
    secs.push_back(std::move(pts));
  }
  return build_ruled_loft_sections(secs);
}

}  // namespace cybercad::native::construct

#endif  // CYBERCAD_NATIVE_CONSTRUCT_LOFT_H
